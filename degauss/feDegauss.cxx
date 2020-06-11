#include "feDegauss.hxx"


// ODB index for reading a particular coil voltage
inline size_t odb_coil_idx(Coil coil) {
  switch(coil) {
    case Coil1:
      return 8;
    case Coil2:
      return 9;
    case Coil3:
      return 2;
    case Coil4:
      return 4;
    case Coil5:
      return 1;
    case Coil6:
      return 0;
    default:
      throw "Go away warnings, I got all the cases";
  }
}


inline Coil _idx_to_coil(size_t idx) {
  switch(idx) {
    case 0:
      return Coil1;
    case 1:
      return Coil2;
    case 2:
      return Coil3;
    case 3:
      return Coil4;
    case 4:
      return Coil5;
    case 5:
      return Coil6;
    default:
      cm_msg(MERROR, "_idx_to_coil", "Invalid coil index %i detected. Must be in 0 <= i <= 5.", (int)idx);
      return Coil1;
  }
}


inline bool set_status(HNDLE hDB, VarStatus status) {
  uint8_t s = static_cast<uint8_t>(status);
  if (s < 0 || s > 4) return false;
  db_set_data(hDB, Keys::Var::status, &s, sizeof(s), 1, TID_BYTE);
  return true;
}


optional<array<double, 6>> get_targets(const array<double, 6>& in_targets, uint8_t step, uint8_t n_steps) {
  if (step > n_steps) {
    cm_msg(MERROR, "feDegauss:get_targets", "Step number (%i) exceeds the total number of steps (%i).", step+1, n_steps);
    return none;
  }
  // done with approach
  else if (step == n_steps) {
    return in_targets;
  }

  array<double, 6> ret;

  for (int i = 0; i < 6; i++) {
    auto coil = _idx_to_coil(i);

    if (in_targets[i] >= max_voltage(coil)) {
      cm_msg(MERROR, "feDegauss:get_targets", "Error: voltage set for coil %i exceeds maximum (%i)", (i+1), (int)max_voltage(coil));
      return none;
    }

    else if (in_targets[i] < 0) {
      cm_msg(MERROR, "feDegauss:get_targets", "Error: voltage set for coil %i is less than zero.", (i+1));
      return none;
    }

    else {
      ret[i] = degauss_step(in_targets[i], coil, step, n_steps);
    }
  }

  return ret;
}


Error ensure_odb_keys(const HNDLE hDB) {
  // first, ensure that structs exist

  auto res = db_create_record(hDB, 0, "/equipment/degauss/settings", SET_STR);
  if (res != DB_SUCCESS) {
    cm_msg(MERROR, "Degauss::ensure_odb_keys", "Error while trying to create settings record: %i", res);
    return Error::CouldNotCreateRecord;
  }
  res = db_create_record(hDB, 0, "/equipment/degauss/variables", VAR_STR);
  if (res != DB_SUCCESS) {
    cm_msg(MERROR, "Degauss::ensure_odb_keys", "Error while trying to create variables record: %i", res);
    cout << "\033[1;31mError: could not create variables record. Error code: " << res << "\033[0m" << endl;
    return Error::CouldNotCreateRecord;
  }

  res = db_find_key(hDB, 0, "/equipment/degauss/settings/targets",  &Keys::Set::targets);
  if (res != DB_SUCCESS) return Error::CouldNotFindKey;
  res = db_find_key(hDB, 0, "/equipment/degauss/settings/run",      &Keys::Set::run);
  if (res != DB_SUCCESS) return Error::CouldNotFindKey;
  res = db_find_key(hDB, 0, "/equipment/degauss/settings/nsteps",   &Keys::Set::nsteps);
  if (res != DB_SUCCESS) return Error::CouldNotFindKey;

  res = db_find_key(hDB, 0, "/equipment/degauss/variables/targets", &Keys::Var::targets);
  if (res != DB_SUCCESS) return Error::CouldNotFindKey;
  res = db_find_key(hDB, 0, "/equipment/degauss/variables/running", &Keys::Var::running);
  if (res != DB_SUCCESS) return Error::CouldNotFindKey;
  res = db_find_key(hDB, 0, "/equipment/degauss/variables/stepn",   &Keys::Var::stepn);
  if (res != DB_SUCCESS) return Error::CouldNotFindKey;
  res = db_find_key(hDB, 0, "/equipment/degauss/variables/status",  &Keys::Var::status);
  if (res != DB_SUCCESS) return Error::CouldNotFindKey;

  res = db_find_key(hDB, 0, COIL_V_READ_KEY, &Keys::coil_read);
  if (res != DB_SUCCESS) {
    return Error::CouldNotFindKey;
  }
  res = db_find_key(hDB, 0, COIL_V_SET_KEY,  &Keys::coil_write);
  if (res != DB_SUCCESS) {
    return Error::CouldNotFindKey;
  }

  return Error::None;
}


array<double, 6> read_voltages(const HNDLE hDB) {
  array<float, 32> input;
  INT bufsize = 32 * sizeof(float);
  db_get_data(hDB, Keys::coil_read, input.data(), &bufsize, TID_FLOAT);
  array<double, 6> ret = {{
    (double)input[8], (double)input[9], (double)input[2], (double)input[4], (double)input[1], (double)input[0]
  }};
  return ret;
}


void write_voltages(const HNDLE hDB, const array<double, 6>& voltages) {
  const array<INT, 6> idxs = {{ 8, 9, 2, 4, 1, 0 }};
  const INT bufsize = sizeof(float);

  for (size_t i = 0; i < 6; ++i) {
    float voltage = static_cast<float>(voltages[i]);
    db_set_data_index(hDB, Keys::coil_write, &voltage, bufsize, idxs[i], TID_FLOAT);
  }

  db_set_data(hDB, Keys::Var::targets, voltages.data(), 6*sizeof(double), 6, TID_DOUBLE);
}


extern "C" {


INT frontend_init() {
#ifdef DEBUG
  cout << "Initializing degauss frontend... ";
#endif

  static char host_name[256] = {0}, exp_name[256] = {0};
  HNDLE hkeyclient;

  cm_get_environment(host_name, 256, exp_name, 256);
  cm_connect_experiment("", exp_name, "feDegauss", 0);
  cm_get_experiment_database(&HDB, &hkeyclient);

  if (!HDB || !hkeyclient) {
    cout << "\033[1;31mCould not connect to experiment. Exiting.\033[0m" << endl;
    return 0;
  } else {
#ifdef DEBUG
    cout << "\033[32mConnected\033[0m to experiment `" << exp_name << "` on host `" << host_name << "`" << endl;
#endif
  }

#ifdef DEBUG
  cout << "Ensuring ODB keys... ";
#endif

  auto res = ensure_odb_keys(HDB);
  switch (res) {
    case Error::None:
#ifdef DEBUG
      cout << "\033[32mSuccess.\033[0m" << endl;
#endif
      break;
    case Error::CouldNotCreateRecord:
    case Error::CouldNotFindKey:
      cm_msg(MERROR, "Degauss init", "Record creation failed.");
      return RET_FAIL;
    default:
      cm_msg(MERROR, "Degauss init", "Unknown error.");
      return RET_FAIL;
  }

  set_status(HDB, VarStatus::Ok);

  // don't continue a run from before feDegauss was started
  BOOL running = false;
  db_set_data(HDB, Keys::Var::running, &running, sizeof(running), 1, TID_BOOL);

  uint8_t stepn = 0;
  db_set_data(HDB, Keys::Var::stepn, &stepn, 1, 1, TID_BYTE);

  // set up hotlink
  db_open_record(HDB, Keys::Set::run, NULL, 4, MODE_READ, degauss_activate, NULL);
  return RET_OK;
}




INT degauss_readout(char* pevent, INT off) {
  /*
   *  If not running, return.
   *  If voltages not yet reached, return.
   *  If voltages reached:
   *    If we're not on the last find next voltages,
   *    If we're on the last voltage, set it.
   *    If we're at the last voltage, set running = false and return.
   */
  BOOL running;
  INT  bufsize = sizeof(running);
  db_get_data(HDB, Keys::Var::running, &running, &bufsize, TID_BOOL);

  if (!running) { return RET_OK; }

  auto vals = read_voltages(HDB);
  array<double, 6> cur_targets;
  bufsize = 6 * sizeof(double);
  db_get_data(HDB, Keys::Var::targets, cur_targets.data(), &bufsize, TID_DOUBLE);

  if (!all_below_threshold(vals, cur_targets)) {
    // cm_msg(MDEBUG, "feDegauss:degauss_readout", "PING: still waiting.");
    return RET_OK;
  }

  uint8_t nsteps, stepn;
  bufsize = sizeof(uint8_t);
  db_get_data(HDB, Keys::Set::nsteps, &nsteps, &bufsize, TID_BYTE);
  bufsize = sizeof(uint8_t);
  db_get_data(HDB, Keys::Var::stepn,  &stepn,  &bufsize, TID_BYTE);

  // cm_msg(MDEBUG, "degauss_readout", "Found N=%i (%x), n=%i (%x)", nsteps, Keys::Set::nsteps, stepn, Keys::Var::stepn);

  if (nsteps == stepn) {
    // all done
    BOOL running = FALSE;
    db_set_data(HDB, Keys::Var::running, &running, sizeof(BOOL), 1, TID_BOOL);
    set_status(HDB, VarStatus::Ok);
    return RET_OK;
  }

  array<double, 6> set_targets;
  bufsize = 6 * sizeof(double);
  db_get_data(HDB, Keys::Set::targets, set_targets.data(), &bufsize, TID_DOUBLE);

  stepn++;
  db_set_data(HDB, Keys::Var::stepn, &stepn, sizeof(stepn), 1, TID_BYTE);

  if (stepn == nsteps) {
    cm_msg(
      MINFO, "feDegauss:degauss_activate",
      "Setting step %i/%i of degauss: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
      nsteps, nsteps, set_targets[0], set_targets[1], set_targets[2], set_targets[3], set_targets[4], set_targets[5]
    );
    write_voltages(HDB, set_targets);
    set_status(HDB, VarStatus::Ok);
    return RET_OK;
  }

  array<double, 6> coils;
  {
    auto _coils = get_targets(set_targets, stepn, nsteps);

    if (!_coils) {
      cm_msg(MERROR, "degauss_readout", "Could not get coil targets on readout. Have the ODB settings for feDegauss changed while a degauss is ongoing?");
      BOOL running = false;
      db_set_data(HDB, Keys::Var::running, &running, sizeof(BOOL), 1, TID_BOOL);
      set_status(HDB, VarStatus::FindingSetsFail);
      return RET_FAIL;
    }

    coils = std::move(*_coils);
  }

  write_voltages(HDB, coils);

  cm_msg(
    MINFO, "feDegauss:degauss_activate",
    "Setting step %i/%i of degauss: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    stepn, nsteps, coils[0], coils[1], coils[2], coils[3], coils[4], coils[5]
  );

  set_status(HDB, VarStatus::Ok);
  return RET_OK;
}




void degauss_activate(HNDLE hDB, HNDLE key, void* info) {
  cm_msg(MDEBUG, "degauss_activate", "Received degauss callback.");

  // check if we're starting or stopping
  BOOL b = false;
  INT run, size = sizeof(run);
  db_get_data(hDB, Keys::Set::run, &run, &size, TID_BOOL);

  set_status(HDB, VarStatus::Ok);

  size = sizeof(b);
  if (!run) {
    cm_msg(MINFO, "degauss_activate", "Received stop degauss command.");
    db_set_data(hDB, Keys::Var::running, &b, size, 1, TID_BOOL);
    return;
  }

  cm_msg(MINFO, "degauss_activate", "Beginning degauss.");

  uint8_t stepn = 0;
  db_set_data(hDB, Keys::Var::stepn, &stepn, sizeof(stepn), 1, TID_BYTE);

  uint8_t nsteps;
  size = sizeof(nsteps);
  db_get_data(hDB, Keys::Set::nsteps, &nsteps, &size, TID_BYTE);

  array<double, 6> set_targets;
  size = 6 * sizeof(double);

  db_get_data(hDB, Keys::Set::targets, set_targets.data(), &size, TID_DOUBLE);

  // check if inputs are valid
  for (size_t i = 0; i < 6; i++) {
    double target = set_targets[i];
    Coil   coil   = _idx_to_coil(i);

    if (isnan(target)) {
      cm_msg(MERROR, "feDegauss:degauss_activate", "Coil target %zd is NaN.", (i+1));
      set_status(HDB, VarStatus::CoilIsNan);
      return;
    }

    else if (target < 0 || target > max_voltage(coil)) {
      cm_msg(
        MERROR, "feDegauss:degauss_activate",
        "Coil target %zd is out of range (must be in [0V, %.2fV]).",
        (i+1), max_voltage(coil)
      );
      set_status(HDB, VarStatus::CoilIsOob);
      return;
    }

    else if (target >= max_voltage(coil) - V_RELIABLE_MARGIN) {
      cm_msg(
        MERROR, "feDegauss:degauss_activate",
        "Warning: voltage set for coil %zd (%.2fV) is too high to be set reliably (must be < %.1f)",
        (i+1), target, max_voltage(coil) - V_RELIABLE_MARGIN
      );
      set_status(HDB, VarStatus::CoilNearOob);
      return;
    }

    else if (target <= V_RELIABLE_MARGIN) {
      cm_msg(
        MERROR, "feDegauss:degauss_activate",
        "Warning: voltage set for coil %zd (%.2fV) is too low to be set reliably (must be > %.1f)",
        (i+1), target, V_RELIABLE_MARGIN
      );
      set_status(HDB, VarStatus::CoilNearOob);
      return;
    }
  }
  
  array<double, 6> coils;
  {
    auto _coils = get_targets(set_targets, 0, nsteps);

    if (!_coils) {
      cm_msg(MERROR, "feMove:degauss_activate", "Finding coil settings failed.");
      set_status(HDB, VarStatus::FindingSetsFail);
      return;
    }

    coils = std::move(*_coils);
  }

  cm_msg(
    MINFO, "feDegauss:degauss_activate",
    "Setting step 0/%i of degauss: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f",
    nsteps, coils[0], coils[1], coils[2], coils[3], coils[4], coils[5]
  );

  write_voltages(hDB, coils);

  b = true;
  size = sizeof(b);
  db_set_data(hDB, Keys::Var::running, &b, size, 1, TID_BOOL);
}



/* Trivial functions */



INT frontend_exit() {
  cm_disconnect_experiment();
  return RET_OK;
}

INT begin_of_run(INT run_number, char *error) {
  return RET_OK;
}

INT end_of_run(INT run_number, char *error) {
  return RET_OK;
}

INT pause_run(INT run_number, char* error) {
  return RET_OK;
}

INT resume_run(INT run_number, char *error) {
  return RET_OK;
}

INT frontend_loop() {
  return RET_OK;
}

INT poll_event(INT source, INT count, BOOL test){
  return FALSE;
}

INT interrupt_configure(INT cmd, INT source[], PTYPE adr) {
  return 1;
}

}