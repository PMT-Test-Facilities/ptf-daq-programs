#include "odblayer.hxx"


static const char
  *COIL_V_SET_KEY    = "/equipment/ptfwiener/settings/outputvoltage",
  *COIL_V_READ_KEY   = "/equipment/ptfwiener/variables/sensevoltage",
  *COIL_U_READ_KEY   = "/equipment/ptfwiener/variables/current",
  *PHIDG_READ_FORMAT = "/equipment/phidget%02i/variables/ph%02i";


static const size_t SIZEOF_FLOAT = sizeof(float);


PhidgetReading PhidgetReading::all_nan() {
  return {
    nan(""), nan(""), nan(""), nan(""), nan(""), nan(""), nan(""), nan(""), nan(""), nan("")
  };
}


int coil_idx(Coil coil) {
  switch(coil) {
    case Coil1:
      return 9;
    case Coil2:
      return 8;
    case Coil3:
      return 2;
    case Coil4:
      return 6;
    case Coil5:
      return 1;
    case Coil6:
      return 0;
    default:
      throw "Go away warnings, I got all the cases";
  }
}


ODBInterface::ODBInterface(const vector<uint32_t>& phidgets) {
  cm_get_environment(host_name, 256, exp_name, 256);
  cm_connect_experiment("", exp_name, "DegaussOptimizer", 0);
  cm_get_experiment_database(&hDB, &hkeyclient);

  if (!hDB || !hkeyclient) {
    throw "Could not connect to database.";
  }

  // find coil r/w keys
  db_find_key(hDB, 0, COIL_V_READ_KEY, &coil_v_read);
  db_find_key(hDB, 0, COIL_V_SET_KEY,  &coil_v_write);
  db_find_key(hDB, 0, COIL_U_READ_KEY, &coil_u_read);

  if (!coil_v_read || !coil_v_write || !coil_u_read) {
    throw "Could not find coil keys.";
  }

  // for (auto phidg : phidgets) {
  //   char phidget_key[256];

  for (int i = 0; i < phidgets.size(); i++) {
    auto phidg = phidgets[i];
    char phidget_key[256];

    snprintf(phidget_key, 256, PHIDG_READ_FORMAT, phidg, phidg);

#ifdef DEBUG
    cout << "Attempting to connect to phidget " << phidg << " with key " << phidget_key << endl;;
#endif

    HNDLE phidget_hndle = (HNDLE) 0;
    db_find_key(hDB, 0, phidget_key, &phidget_hndle);

    if (!phidget_hndle) {
      throw "Could not find phidget key.";
    }

    phidget_handle_cache[phidg] = phidget_hndle;
  }
}


ODBInterface::~ODBInterface() {
  cm_disconnect_experiment();
}


double ODBInterface::get_coil_voltage(Coil coil) {
  float v;
  size_t size = SIZEOF_FLOAT;
  db_get_data_index(hDB, coil_v_read, &v, (int*) &size, coil_idx(coil), TID_FLOAT);
  return (double) v;
}


void ODBInterface::set_coil_voltage(Coil coil, double voltage) {
  float v = (float) voltage;
  db_set_data_index(hDB, coil_v_write, &v, SIZEOF_FLOAT, coil_idx(coil), TID_FLOAT);
}


PhidgetReading ODBInterface::get_phidget_reading(uint32_t phidget) {
  auto res = phidget_handle_cache.find(phidget);

  if (res == phidget_handle_cache.end()) {
    return PhidgetReading::all_nan();
  }

  double data[10];
  int    size = 10 * sizeof(double);
  db_get_data(hDB, res->second, data, &size, TID_DOUBLE);

  PhidgetReading ret;

  ret.acc0   = data[0];
  ret.acc1   = data[1];
  ret.acc2   = data[2];
  ret.b0     = data[3];
  ret.b1     = data[4];
  ret.b2     = data[5];
  ret.mag    = data[6];
  ret.tilt   = data[7];
  ret.etime  = data[8];
  ret.uetime = data[9];

  return ret;
}
