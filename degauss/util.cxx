#include "util.hxx"


OptimizeSettings load_settings(std::string file_name) {
  boost::property_tree::ptree pt;
  boost::property_tree::ini_parser::read_ini(file_name, pt);
  ///Dimension scan =
  //  x_mode == "scan" ? X :
  //  y_mode == "scan" ? Y :
  //  Z;
  
  double min, max, step, set3a, set3b, set1a, set1b, set2a, set2b;
  Dimension static1, static2, static3;
      step  = pt.get<double>("X.step");
      set1a = pt.get<double>("X.seta");
      set1b = pt.get<double>("X.setb");
      set2a = pt.get<double>("Y.seta");
      set2b = pt.get<double>("Y.setb");
      set3a =  pt.get<double>("Z.seta");
      set3b = pt.get<double>("Z.setb");
      static1 = X;
      static2 = Y;
      static3= Z;
     
  

  return (OptimizeSettings) {pt.get<uint8_t>("Meta.steps"),
    step,
    static1,
    set1a, set1b,
    static2,
    set2a, set2b,
    static3,
    set3a,set3b
};

}

double halfw(double lo, double hi) {
  return (lo + hi) / 2;
}


ostream& operator<<(ostream& s, array<double, 6> xs) {
  s << xs[0] << ", "
    << xs[1] << ", "
    << xs[2] << ", "
    << xs[3] << ", "
    << xs[4] << ", "
    << xs[5];
  return s;
}


bool wait_for_all(
  ODBInterface& iface,
  array<Coil, 6> coils,
  array<double, 6> voltages,
  uint32_t milli_max_time,
  uint32_t milli_delay
) {
  struct timespec now, init_time;
  uint64_t unow, uinit, uend;  // in useconds

  clock_gettime(CLOCK_MONOTONIC, &init_time);
  uinit = (init_time.tv_nsec/1000) + (1e6 * init_time.tv_sec);
  uend  = uinit + (1e3 * milli_max_time);

  array<double, 6> mv;

  while (1) {
    cerr << "Waiting for coils to reach (" << voltages << ")... \033[K";

    for (int i = 0; i < 6; i++) {
      mv[i] = iface.get_coil_voltage(coils[i]);
    }

    clock_gettime(CLOCK_MONOTONIC, &now);
    unow = (now.tv_nsec/1000) + (1e6 * now.tv_sec);

    if (
      fabs(voltages[0] - mv[0]) < MARGIN &&
      fabs(voltages[1] - mv[1]) < MARGIN &&
      fabs(voltages[2] - mv[2]) < MARGIN &&
      fabs(voltages[3] - mv[3]) < MARGIN &&
      fabs(voltages[4] - mv[4]) < MARGIN &&
      fabs(voltages[5] - mv[5]) < MARGIN
    ) {
      cerr << "\033[32;1mReached coil targets \033[0m\033[32m(at " << mv << ")\033[0m" << endl;
      return true;
    }

    if (unow > uend) {
      cerr << "\033[31;1mReached timeout without coils matching.\033[31m" << endl;
      return false;
    }

    cerr << "\033[36mAt: (" << mv << ")\033[0m\r";

    usleep(1000 * milli_delay);
  }
}


bool wait_for_coils(
  ODBInterface& iface,
  Coil c0, Coil c1, double v0, double v1,
  uint32_t milli_max_time,
  uint32_t milli_delay
) {
  struct timespec now, init_time;
  uint64_t unow, uinit, uend;  // in useconds

  clock_gettime(CLOCK_MONOTONIC, &init_time);
  uinit = (init_time.tv_nsec/1000) + (1e6 * init_time.tv_sec);
  uend  = uinit + (1e3 * milli_max_time);

  double mv0, mv1;

  while (1) {
    cerr << "Waiting for coils to reach " << v0 << ", " << v1 << "... \033[K";

    mv0 = iface.get_coil_voltage(c0);
    mv1 = iface.get_coil_voltage(c1);

    clock_gettime(CLOCK_MONOTONIC, &now);
    unow = (now.tv_nsec/1000) + (1e6 * now.tv_sec);

    if (fabs(v0 - mv0) < MARGIN && fabs(v1 - mv1) < MARGIN) {
      cerr << "\033[32;1mReached coil targets \033[0m\033[32m(at " << mv0 << "V, " << mv1 << "V)\033[0m" << endl;
      return true;
    }

    if (unow > uend) {
      cerr << "\033[31;1mReached timeout without coils matching.\033[31m"
           << " Current v0: " << mv0 << "V, target is " << v0
           << "V. Current v1: " << mv1 << "V, target is " << v1 << "V.\033[0m" << endl;
      return false;
    }

    cerr << "\033[36mAt: (" << mv0 << ", " << mv1 << "), distance: (" << (mv0 - v0) << ", " << (mv1 - v1) << ")\033[0m\r";

    usleep(1000 * milli_delay);
  }
}


// odb wakes thread with signals, to properly wait time we need to check if we have slept as long as we thought
void semibusy_wait(uint32_t millis) {
  struct timespec now;
  uint64_t unow, uthen; // useconds

  clock_gettime(CLOCK_MONOTONIC, &now);
  unow  = (now.tv_nsec/1000) + (1e6 * now.tv_sec);
  uthen = unow + (1e3 * millis);

  while (true) {
    clock_gettime(CLOCK_MONOTONIC, &now);
    unow = (now.tv_nsec / 1000) + (1e6 * now.tv_sec);
    if (unow >= uthen) return;
    usleep(uthen - unow);
  }
}
