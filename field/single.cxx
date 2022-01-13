#include "util.hxx"


#define RANDOMIZE true
#define RAND_ALL  false


void write_measurement(ofstream& csv, uint64_t utime, double v0, double v1, double mv0, double mv1, const PhidgetReading& p_up, const PhidgetReading& p_dn, const PhidgetReading& p_sd) {
  csv << utime << ","
      << v0 << "," << v1 << ","
      << mv0 << "," << mv1 << ","
      << p_up.b0 << "," << p_up.b1 << "," << p_up.b2 << ","
      << p_dn.b0 << "," << p_dn.b1 << "," << p_dn.b2 << ","
      << p_sd.b0 << "," << p_sd.b1 << "," << p_sd.b2 << endl;
}


double get_random_coil_voltage() {
  return (double) (rand() % 40)/10 + 2;
}


int main(void) {
  static const auto dim = X;
  
  const auto coils = coils_for_dimension(dim);

  const double
    max_v0   = max_voltage(coils.first),
    start_v0 = (max_v0 / 2) + 1,
    max_v1   = max_voltage(coils.second),
    start_v1 = (max_v1 / 2) - 1;

  const auto dg0  = degauss_path(start_v0, coils.first,  N_STEPS);
  const auto dg1  = degauss_path(start_v1, coils.second, N_STEPS);

  const vector<uint32_t> phidgets = {0,1,3};

  auto iface = ODBInterface(phidgets);

  for (int run = 0; run < 8; run++) {

    cerr << endl << "\t\033[1;4mBeginning run number " << (run+1) << "/" << 8 << "\033[0m" << endl << endl;

    struct timespec now,  init;
    uint64_t        unow, uinit;

    clock_gettime(CLOCK_MONOTONIC, &init);
    uinit = (init.tv_nsec/1000) + (1e6 * init.tv_sec);

    ofstream csv;
    char fname[64];
    snprintf(fname, 32, "single_degauss_%i.csv", run);
    // string fname_ = string(fname);
    csv.open(fname);

    csv << "t, target_v0, target_v1, measured_v0, measured_v1, top_bx, top_by, top_bz, btm_bx, btm_by, btm_bz, side_bx, side_by, side_bz" << endl;

    double mv0, mv1;
    PhidgetReading mp_dn, mp_up, mp_sd;


    if (RANDOMIZE) {
      cerr << "\033[1mRandomizing field.\033[0m" << endl;

      for (int i = 0; i < 32; i++) {
        cerr << "\r  Step " << (i+1) << "/" << 32 << " of randomize.";

        double v0 = get_random_coil_voltage();
        double v1 = get_random_coil_voltage();

        iface.set_coil_voltage(coils.first, v0);
        iface.set_coil_voltage(coils.second, v1);

        if (RAND_ALL) {
          for (int _i = 0; _i < 6; _i++) {
            auto coil = ALL_COILS[_i];
            if (coil == coils.first || coil == coils.second) continue;
            iface.set_coil_voltage(coil, get_random_coil_voltage());
          }
        }

        mv0 = iface.get_coil_voltage(coils.first);
        mv1 = iface.get_coil_voltage(coils.second);
        mp_dn = iface.get_phidget_reading(3); 
        mp_up = iface.get_phidget_reading(0);
        mp_sd = iface.get_phidget_reading(1);

        clock_gettime(CLOCK_MONOTONIC, &now);
        unow = (now.tv_nsec/1000) + (1e6 * now.tv_sec);

        write_measurement(csv, unow-uinit, v0, v1, mv0, mv1, mp_up, mp_dn, mp_sd);

        semibusy_wait(1000);
      }

      cerr << endl;
    }


    cerr << "\033[1mStarting degauss.\033[0m" << endl;
    for (auto i = 0; i < N_STEPS; i++) {
      auto v0 = dg0[i], v1 = dg1[i];
      cerr << "  Step " << (i+1) << "/" << N_STEPS << " of degauss.";
      mv0 = iface.get_coil_voltage(coils.first);
      mv1 = iface.get_coil_voltage(coils.second);
      mp_dn = iface.get_phidget_reading(3); 
      mp_up = iface.get_phidget_reading(0);
      mp_sd = iface.get_phidget_reading(1);

      clock_gettime(CLOCK_MONOTONIC, &now);
      unow = (now.tv_nsec/1000) + (1e6 * now.tv_sec);

      write_measurement(csv, unow-uinit, v0, v1, mv0, mv1, mp_up, mp_dn, mp_sd);

      iface.set_coil_voltage(coils.first, v0);
      iface.set_coil_voltage(coils.second, v1);

      while (
           fabs(mv0 - v0) > MARGIN 
        || fabs(mv1 - v1) > MARGIN
      ) {
        semibusy_wait(10);
        mv0 = iface.get_coil_voltage(coils.first);
        mv1 = iface.get_coil_voltage(coils.second);
        mp_dn = iface.get_phidget_reading(3); 
        mp_up = iface.get_phidget_reading(0);
        mp_sd = iface.get_phidget_reading(1);

        clock_gettime(CLOCK_MONOTONIC, &now);
        unow = (now.tv_nsec/1000) + (1e6 * now.tv_sec);

        write_measurement(csv, unow-uinit, v0, v1, mv0, mv1, mp_up, mp_dn, mp_sd);
      }

      cerr << " Target reached. Taking samples.";

      for (int i = 0; i < 4; i++) {
        semibusy_wait(100);
        mv0 = iface.get_coil_voltage(coils.first);
        mv1 = iface.get_coil_voltage(coils.second);
        mp_dn = iface.get_phidget_reading(3); 
        mp_up = iface.get_phidget_reading(0);
        mp_sd = iface.get_phidget_reading(1);

        clock_gettime(CLOCK_MONOTONIC, &now);
        unow = (now.tv_nsec/1000) + (1e6 * now.tv_sec);

        write_measurement(csv, unow-uinit, v0, v1, mv0, mv1, mp_up, mp_dn, mp_sd);
      }

      cerr << " Samples done." << endl;
    }

    cerr << "\033[1mSetting final voltage... \033[0m"; 

    iface.set_coil_voltage(coils.first, start_v0);
    iface.set_coil_voltage(coils.second, start_v1);


    while (
          fabs(mv0 - start_v0) > MARGIN 
      && fabs(mv1 - start_v1) > MARGIN
    ) {
      semibusy_wait(10);
      mv0 = iface.get_coil_voltage(coils.first);
      mv1 = iface.get_coil_voltage(coils.second);
      mp_dn = iface.get_phidget_reading(3); 
      mp_up = iface.get_phidget_reading(0);
      mp_sd = iface.get_phidget_reading(1);

      clock_gettime(CLOCK_MONOTONIC, &now);
      unow = (now.tv_nsec/1000) + (1e6 * now.tv_sec);

      write_measurement(csv, unow-uinit, start_v0, start_v1, mv0, mv1, mp_up, mp_dn, mp_sd);
    }

    cerr << "Done." << endl;

    for (int i = 0; i < 32; i++) {
      cerr << "\r\033[K  Sample " << (i+1) << "/32...";
      semibusy_wait(1000);
      cerr << " Recording...";
      mv0 = iface.get_coil_voltage(coils.first);
      mv1 = iface.get_coil_voltage(coils.second);
      mp_dn = iface.get_phidget_reading(3); 
      mp_up = iface.get_phidget_reading(0);
      mp_sd = iface.get_phidget_reading(1);

      clock_gettime(CLOCK_MONOTONIC, &now);
      unow = (now.tv_nsec/1000) + (1e6 * now.tv_sec);

      write_measurement(csv, unow-uinit, start_v0, start_v1, mv0, mv1, mp_up, mp_dn, mp_sd);
    }

    cerr << endl << "\033[1mDone.\033[0m" << endl;
    csv.close();

  }
}
