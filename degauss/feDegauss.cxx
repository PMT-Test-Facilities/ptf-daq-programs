#include "feDegauss.hxx"


HNDLE Degauss::get_key(Keys key) {
  string name = "";
  bool set = true;
  switch (key) {
    case SetCoilTargets:
      name = "coil_targets"; break;
    case SetRun:
      name = "run"; break;
    case SetNsteps:
      name = "nsteps"; break;
    case VarCoilSet:
      name = "coil_set"; set = false; break;
    case VarRunning:
      name = "running"; set = false; break;
    case VarStepn:
      name = "stepn"; set = false; break;
    default:
      cm_msg(MERROR, "Degauss::get_key", "Unknown key enum.");
#ifdef DEBUG
      cerr << "\033[1;31mUnknown key enum. Returning -1.\033[0m" << endl;
#endif
      return (HNDLE)-1;
  }

  if (set) {
    return SET_KEYS.find(name)->second;
  } else {
    return VAR_KEYS.find(name)->second;
  }
}


// ODB index for reading a particular coil voltage
int coil_idx(Coil coil) {
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
      //throw "Go away warnings, I got all the cases";
      cm_msg(MERROR, "coil_idx", "Got unknown coil =%i.", (int)coil);
      return -1;
  }
}


using namespace Degauss;


INT frontend_init() {
#ifdef DEBUG
  cerr << "Initializing degauss frontend... ";
#endif

  cm_get_environment(host_name, 256, exp_name, 256);
  cm_connect_experiment("", exp_name, "feDegauss", 0);
  cm_get_experiment_database(&hDB, &hkeyclient);

  if (!hDB || !hkeyclient) {
    cerr << "\033[1;31mCould not connect to experiment. Exiting.\033[0m" << endl;
    return CM_DB_ERROR;
  } else {
#ifdef DEBUG
    cerr << "Connected to database `" << exp_name << "` on host `" << host_name << "`" << endl;
#endif
  }

#ifdef DEBUG
  cerr << "Ensuring ODB keys." << endl;
#endif

  vector<string> keys = {
    {"/equipment/degauss/", }
  };

  auto res = Degauss::ensure_odb_keys(hDB);
  switch (res) {
    case Degauss::Error::None:
#ifdef DEBUG
      cerr << "Success." << endl;
#endif
      break;
    case Degauss::Error::KeyDoesNotExist:
      cm_msg(MERROR, "Degauss init", "Record creation failed.");
      return EXIT_FAILURE;
    default:
      cm_msg(MERROR, "Degauss init", "Unknown error.");
      return EXIT_FAILURE;
  }

  // set up hotlink
  db_open_record(hDB, get_key(Keys::SetRun), NULL, 4, MODE_READ, degauss_activate, NULL);
}


Coil _idx_to_coil(uint32_t idx) {
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


vector< pair<Coil, double> > get_targets(array<float,6> targets, uint8_t step, uint8_t nsteps) {
  vector< pair<Coil, double> > coils;
  for (int i = 0; i < 6; i++) {
    if (targets[i] >= 0) {
      auto coil  = _idx_to_coil(i);
      auto facts = exponential_factors((double)targets[i], coil, nsteps);
      auto stepd = degauss_step((double)targets[i], coil, step, facts.first, facts.second);
      coils.push_back(make_pair(coil, stepd));
    }
  }
  return coils;
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

  uint32_t running = false;
  INT size = 4;

  // are we running?
  db_get_data(hDB, get_key(Keys::VarRunning), &running, &size, TID_BOOL);

  if (!running) {
#ifdef DEBUG
    cerr << "Received callback, but not running. Exiting." << endl;
#endif
  };


  array<float, 6> targets = {-1};
  size = 6 * sizeof(float);

  db_get_data(hDB, get_key(Keys::VarCoilSet), targets.data(), &size, TID_FLOAT);


  for (int i = 0; i < 6; i++) {
    if (targets[i] == -1) continue;
    else {
      auto coil = _idx_to_coil(i);
      auto idx  = coil_idx(coil);
      float measured = nanf("");
      INT size = sizeof(measured);
      db_get_data_index(hDB, COIL_READ, &measured, &size, idx, TID_FLOAT);

      if (measured != measured) {
        cm_msg(MERROR, "readout", "Could not read voltage for coil (or voltage is NaN).");
        return 0;
      }

      if (fabs(measured - targets[i]) > TOLERANCE) {
#ifdef DEBUG
        cerr << "Coil " << (i+1) << " exceeds tolerance by " << fabs(measured - targets[i]) << "V." << endl;
#endif
        return SUCCESS;
      }

    }
  }

  // now, move to next step
  uint8_t step = 0;
  size = sizeof(step);
  db_get_data(hDB, get_key(Keys::VarStepn), &step, &size, TID_CHAR);
  step++;

  uint8_t nsteps = 0;
  size = sizeof(nsteps);
  db_get_data(hDB, get_key(Keys::SetNsteps), &nsteps, &size, TID_CHAR);

  if (step == nsteps+1) {
    // all done!
    INT f = 0;
    size = sizeof(f);
    db_set_data(hDB, get_key(Keys::VarRunning), &f, size, 1, TID_BOOL);
  }
  else if (step == nsteps) {
    // last step
    array<float, 6> finalv = {-1};
    size = 6 * sizeof(float);
    db_get_data(hDB, get_key(Keys::SetCoilTargets), finalv.data(), &size, TID_FLOAT);

    for (int i = 0; i < 6; i++) {
      if (finalv[i] == -1) continue;
      auto coil = _idx_to_coil(i);
      auto idx  = coil_idx(coil);
      db_set_data_index(hDB, COIL_WRITE, finalv.data() + i, sizeof(float), idx, TID_FLOAT);
    }

  } else {
    // next step
    auto coils = get_targets(targets, step, nsteps);
    for (int i = 0; i < coils.size(); i++) {
      Coil coil   = coils[i].first;
      auto target = (float) coils[i].second;
      auto idx    = coil_idx(coil);
      db_set_data_index(hDB, COIL_WRITE, &target, sizeof(float), idx, TID_FLOAT);
    }
  }

  return SUCCESS;
}


void degauss_activate(HNDLE hDB, HNDLE key, void* info) {
  cm_msg(MINFO, "degauss_activate", "Received degauss callback.");

  uint8_t stepn = 0;
  db_set_data(hDB, get_key(Keys::VarStepn), &stepn, sizeof(stepn), 1, TID_CHAR);

  uint8_t nsteps = 0;
  db_set_data(hDB, get_key(Keys::SetNsteps), &stepn, sizeof(stepn), 1, TID_CHAR);

  array<float, 6> targets = {-1};
  INT size = 6 * sizeof(float);
  db_get_data(hDB, get_key(Keys::SetCoilTargets), targets.data(), &size, TID_FLOAT);
  
  auto coils = get_targets(targets, 0, nsteps);

  for (int i = 0; i < coils.size(); i++) {
    Coil coil   = coils[i].first;
    auto target = (float) coils[i].second;
    auto idx    = coil_idx(coil);
    db_set_data_index(hDB, COIL_WRITE, &target, sizeof(float), idx, TID_FLOAT);
  }

  cm_msg(MDEBUG, "degauss_activate", "Will begin degaussing on next readout.");
}


Error Degauss::ensure_odb_keys(const HNDLE hDB) {
  char name[256];

  // first, ensure that structs exist

  auto res = db_create_record(hDB, 0, "/equipment/degauss", SET_STR);
  if (res != DB_SUCCESS) {
    return Error::CouldNotCreateRecord;
  }
  res = db_create_record(hDB, 0, "/equipment/degauss", VAR_STR);
  if (res != DB_SUCCESS) {
    return Error::CouldNotCreateRecord;
  }

  for (auto key = SET_KEYS.begin(); key != SET_KEYS.end(); key++) {
    snprintf(name, 256, "/equipment/degauss/settings/%s", key->first.c_str());
    auto res = db_find_key(hDB, 0, name, &key->second);
    switch(res) {
      case DB_SUCCESS:
        break;
      case DB_INVALID_HANDLE:
        return Error::InvalidHandle;
      case DB_NO_ACCESS:
        return Error::InvalidDb;
      case DB_NO_KEY:
        return Error::KeyDoesNotExist;
      default:
        return Error::Unknown;
    }
  }  

  for (auto key = VAR_KEYS.begin(); key != VAR_KEYS.end(); key++) {
    snprintf(name, 256, "/equipment/degauss/variables/%s", key->first.c_str());
    auto res = db_find_key(hDB, 0, name, &key->second);
    switch(res) {
      case DB_SUCCESS:
        break;
      case DB_INVALID_HANDLE:
        return Error::InvalidHandle;
      case DB_NO_ACCESS:
        return Error::InvalidDb;
      case DB_NO_KEY:
        return Error::KeyDoesNotExist;
      default:
        return Error::Unknown;
    }
  }

  res = db_find_key(hDB, 0, COIL_V_READ_KEY, &COIL_READ);
  if (res != DB_SUCCESS) {
    return Error::KeyDoesNotExist;
  }
  res = db_find_key(hDB, 0, COIL_V_SET_KEY,  &COIL_WRITE);
  if (res != DB_SUCCESS) {
    return Error::KeyDoesNotExist;
  }

  return Error::None;
}


/* Trivial functions */

INT frontend_exit() {
  cm_disconnect_experiment();
}

INT begin_of_run(INT run_number, char *error) {
  return SUCCESS;
}

INT end_of_run(INT run_number, char *error) {
  return SUCCESS;
}

INT pause_run(INT run_number, char* error) {
  return SUCCESS;
}

INT resume_run(INT run_number, char *error) {
  return SUCCESS;
}