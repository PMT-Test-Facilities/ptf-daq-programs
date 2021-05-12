#include "feDegauss.hxx"


HNDLE Degauss::get_key(Keys key) {
  string name = "";
  bool set = true;
  switch (key) {
    case SetCoilTargets:
      name = "targets"; break;
    case SetRun:
      name = "run"; break;
    case SetNsteps:
      name = "nsteps"; break;
    case VarCoilSet:
      name = "targets"; set = false; break;
    case VarRunning:
      name = "running"; set = false; break;
    case VarStepn:
      name = "stepn"; set = false; break;
    default:
      cm_msg(MERROR, "Degauss::get_key", "Unknown key enum.");
#ifdef DEBUG
      cout << "Unknown key enum. Returning -1." << endl;
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
  cout << "Initializing degauss frontend... ";
#endif

  static char host_name[256] = {0}, exp_name[256] = {0};

  cm_get_environment(host_name, 256, exp_name, 256);
  cm_connect_experiment("", exp_name, "feDegauss", 0);
  cm_get_experiment_database(&hDB, &hkeyclient);

  if (!hDB || !hkeyclient) {
    cout << "Could not connect to experiment. Exiting." << endl;
    return 0;
  } else {
#ifdef DEBUG
    cout << "Connected to experiment `" << exp_name << "` on host `" << host_name << "`" << endl;
#endif
  }

#ifdef DEBUG
  cout << "Ensuring ODB keys... ";
#endif

  auto res = Degauss::ensure_odb_keys(hDB);
  switch (res) {
    case Degauss::Error::None:
#ifdef DEBUG
      cout << "Success." << endl;
#endif
      break;
    case Degauss::Error::KeyDoesNotExist:
      cm_msg(MERROR, "Degauss init", "Record creation failed.");
      return 0;
    default:
      cm_msg(MERROR, "Degauss init", "Unknown error.");
      return 0;
  }

  // don't continue a run from before feDegauss was started
  INT running = false;
  db_set_data(hDB, get_key(Keys::VarRunning), &running, sizeof(running), 1, TID_BOOL);

  // set up hotlink
  db_open_record(hDB, get_key(Keys::SetRun), NULL, 4, MODE_READ, degauss_activate, NULL);
  return SUCCESS;
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
    //    return SUCCESS;

  uint32_t running = false;
  INT size = 4;

  // are we running?
  db_get_data(hDB, get_key(Keys::VarRunning), &running, &size, TID_BOOL);

  if (!running) {
#ifdef DEBUG
      //    cout << "\033[35m.\033[0m";
      //std::cout << "degauss readout sdfdsf " << std::endl;
    
#endif
    return SUCCESS;
  };

  uint8_t step = 0;
  size = sizeof(step);
  db_get_data(hDB, get_key(Keys::VarStepn), &step, &size, TID_CHAR);

#ifdef DEBUG
    cout << "Received callback, on step " << ((int)step) << "." << endl;
#endif

  array<float, 6> targets;
  size = 6 * sizeof(float);

  db_get_data(hDB, get_key(Keys::VarCoilSet), targets.data(), &size, TID_FLOAT);

#ifdef DEBUG
    cout << "Got current targets." << endl;
#endif

  for (int i = 0; i < 6; i++) {
    if (targets[i] == -1) {
#ifdef DEBUG
      cout << "Coil " << (i+1) << " is not being degaussed, skipping." << endl;
#endif
    }
    else {
#ifdef DEBUG
      cout << "Checking coil  " << (i+1) << "." << endl;
#endif
      auto coil = _idx_to_coil(i);
      auto idx  = coil_idx(coil);
      float measured = nan("");
      INT size = sizeof(measured);
      db_get_data_index(hDB, COIL_READ, &measured, &size, idx, TID_FLOAT);

#ifdef DEBUG
      cout << "Found " << measured << "V (index " << idx << ")." << endl;
#endif

      if (measured != measured) {
        cm_msg(MERROR, "readout", "Could not read voltage for coil (or voltage is NaN).");
        return 0;
      }

      if (fabs(measured - targets[i]) > TOLERANCE) {
#ifdef DEBUG
        cout << "Coil " << (i+1) << " exceeds tolerance by " << fabs(measured - targets[i]) << "V." << endl;
#endif
        return SUCCESS;
      } else {
#ifdef DEBUG
        cout << "Coil " << (i+1) << " is within tolerance." << endl;
#endif
      }

    }
  }

  // now, move to next step
  step++;
  db_set_data(hDB, get_key(Keys::VarStepn), &step, sizeof(step), 1, TID_CHAR);

  uint8_t nsteps = 0;
  size = sizeof(nsteps);
  db_get_data(hDB, get_key(Keys::SetNsteps), &nsteps, &size, TID_CHAR);

  if (step > nsteps) {
    // all done!
    INT f = 0;
    size = sizeof(f);
    db_set_data(hDB, get_key(Keys::VarRunning), &f, size, 1, TID_BOOL);
  }
  else if (step == nsteps) {
    // last step
    array<float, 6> finalv;
    size = 6 * sizeof(float);
    db_get_data(hDB, get_key(Keys::SetCoilTargets), finalv.data(), &size, TID_FLOAT);

    for (int i = 0; i < 6; i++) {
      if (finalv[i] == -1) continue;
      auto coil = _idx_to_coil(i);
      auto idx  = coil_idx(coil);
      db_set_data_index(hDB, COIL_WRITE, finalv.data() + i, sizeof(float), idx, TID_FLOAT);
      db_set_data_index(hDB, get_key(Keys::VarCoilSet), finalv.data() + i, sizeof(float), i, TID_FLOAT);
    }

  } else {
    // next step
    auto coils = get_targets(targets, step, nsteps);
    for (uint32_t i = 0; i < coils.size(); i++) {
      Coil coil   = coils[i].first;
      auto target = (float) coils[i].second;
      auto idx    = coil_idx(coil);
      db_set_data_index(hDB, COIL_WRITE, &target, sizeof(float), idx, TID_FLOAT);
      db_set_data_index(hDB, get_key(Keys::VarCoilSet), &target, sizeof(float), coil, TID_FLOAT);
    }
  }

  return SUCCESS;
}


void degauss_activate(HNDLE hDB, HNDLE key, void* info) {
  cm_msg(MINFO, "degauss_activate", "Received degauss callback.");

  // check if we're starting or stopping
  INT run, b = false;
  INT size = sizeof(run);
  db_get_data(hDB, get_key(Keys::SetRun), &run, &size, TID_BOOL);

  size = sizeof(b);
  if (!run) {
    db_set_data(hDB, get_key(Keys::VarRunning), &b, size, 1, TID_BOOL);
    return;
  }

  uint8_t stepn = 0;
  db_set_data(hDB, get_key(Keys::VarStepn), &stepn, sizeof(stepn), 1, TID_CHAR);

  uint8_t nsteps = 8;
  size = sizeof(nsteps);
  db_get_data(hDB, get_key(Keys::SetNsteps), &nsteps, &size, TID_CHAR);

  array<float, 6> var_targets, set_targets;
  size = 6 * sizeof(float);
  for (int i = 0; i < 6; i++) var_targets[i] = -1;
  db_set_data(hDB, get_key(Keys::VarCoilSet), var_targets.data(), size, 6, TID_FLOAT);
  db_get_data(hDB, get_key(Keys::SetCoilTargets), set_targets.data(), &size, TID_FLOAT);
  
  auto coils = get_targets(set_targets, 0, nsteps);

  for (uint32_t i = 0; i < coils.size(); i++) {
    cout << "Setting coil " << coils[i].first+1 << " to " << coils[i].second << endl;
    Coil coil   = coils[i].first;
    auto target = (float) coils[i].second;
    auto idx    = coil_idx(coil);
    db_set_data_index(hDB, COIL_WRITE, &target, sizeof(float), idx, TID_FLOAT);
    db_set_data_index(hDB, get_key(Keys::VarCoilSet), &target, sizeof(float), coils[i].first, TID_FLOAT);
  }

  b = true;
  size = sizeof(b);
  db_set_data(hDB, get_key(Keys::VarRunning), &b, size, 1, TID_BOOL);

  cm_msg(MDEBUG, "degauss_activate", "Will begin degaussing on next readout.");
}


Error Degauss::ensure_odb_keys(const HNDLE hDB) {
  char name[256];

  // first, ensure that structs exist

  auto res = db_create_record(hDB, 0, "/equipment/degauss/settings", SET_STR);
  if (res != DB_SUCCESS) {
    cm_msg(MERROR, "Degauss::ensure_odb_keys", "Error while trying to create settings record: %i", res);
    return Error::CouldNotCreateRecord;
  }
  res = db_create_record(hDB, 0, "/equipment/degauss/variables", VAR_STR);
  if (res != DB_SUCCESS) {
    cm_msg(MERROR, "Degauss::ensure_odb_keys", "Error while trying to create variables record: %i", res);
    cout << "Error: could not create variables record. Error code: " << res << "" << endl;
    return Error::CouldNotCreateRecord;
  }

  for (auto key = SET_KEYS.begin(); key != SET_KEYS.end(); key++) {
    snprintf(name, 256, "/equipment/degauss/Settings/%s", key->first.c_str());
    auto res = db_find_key(hDB, 0, name, &key->second);
    switch((uint32_t)res) {
      case DB_SUCCESS:
        break;
      case DB_INVALID_HANDLE:
        return Error::InvalidHandle;
      case DB_NO_ACCESS:
        return Error::InvalidDb;
      case DB_NO_KEY:
	  printf("Fail %s",name);
        return Error::KeyDoesNotExist;
      default:
        return Error::Unknown;
    }
  }  

  for (auto key = VAR_KEYS.begin(); key != VAR_KEYS.end(); key++) {
    snprintf(name, 256, "/equipment/degauss/Variables/%s", key->first.c_str());
    auto res = db_find_key(hDB, 0, name, &key->second);
    switch(res) {
      case DB_SUCCESS:
        break;
      case DB_INVALID_HANDLE:
        return Error::InvalidHandle;
      case DB_NO_ACCESS:
        return Error::InvalidDb;
      case DB_NO_KEY:
	  printf("Fail %s",name);
        return Error::KeyDoesNotExist;
      default:
        return Error::Unknown;
    }
  }

  res = db_find_key(hDB, 0, COIL_V_READ_KEY, &COIL_READ);
  if (res != DB_SUCCESS) {
      printf("Fail coil_v_read");

    return Error::KeyDoesNotExist;
  }
  res = db_find_key(hDB, 0, COIL_V_SET_KEY,  &COIL_WRITE);
  if (res != DB_SUCCESS) {
      printf("Fail coil_v_set");
    return Error::KeyDoesNotExist;
  }

  return Error::None;
}


/* Trivial functions */

INT frontend_exit() {
  cm_disconnect_experiment();
  return SUCCESS;
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

INT frontend_loop() {
  return SUCCESS;
}

INT poll_event(INT source, INT count, BOOL test){
  return FALSE;
}



/*-- Interrupt configuration ---------------------------------------*/
INT interrupt_configure(INT cmd, INT source, POINTER_T adr)
{
  switch (cmd) {
  case CMD_INTERRUPT_ENABLE:
    break;
  case CMD_INTERRUPT_DISABLE:
    break;
  case CMD_INTERRUPT_ATTACH:
    break;
  case CMD_INTERRUPT_DETACH:
    break;
  }
  return SUCCESS;
}
