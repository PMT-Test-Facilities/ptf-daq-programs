#include "util.hxx"


OptimizeSettings load_settings(std::string file_name) {
  boost::property_tree::ptree pt;
  boost::property_tree::ini_parser::read_ini(file_name, pt);

  auto x_mode = pt.get<string>("X.mode"),
       y_mode = pt.get<string>("Y.mode"),
       z_mode = pt.get<string>("Z.mode");
  
  if (
    !(x_mode == "scan" && y_mode == "static" && z_mode == "static") &&
    !(x_mode == "static" && y_mode == "scan" && z_mode == "static") &&
    !(x_mode == "static" && y_mode == "static" && z_mode == "scan")
  ) {
    throw runtime_error("Exactly one dimension must have 'scan' mode.");
  }

  Dimension scan =
    x_mode == "scan" ? X :
    y_mode == "scan" ? Y :
    Z;
  
  double min, max, step, set1a, set1b, set2a, set2b;
  Dimension static1, static2;

  switch(scan) {
    case X:
      min   = pt.get<double>("X.min");
      max   = pt.get<double>("X.max");
      step  = pt.get<double>("X.step");
      set1a = pt.get<double>("Y.seta");
      set1b = pt.get<double>("Y.setb");
      set2a = pt.get<double>("Z.seta");
      set2b = pt.get<double>("Z.setb");
      static1 = Y;
      static2 = Z;
      break;
    case Y:
      min   = pt.get<double>("Y.min");
      max   = pt.get<double>("Y.max");
      step  = pt.get<double>("Y.step");
      set1a = pt.get<double>("X.seta");
      set1b = pt.get<double>("X.setb");
      set2a = pt.get<double>("Z.seta");
      set2b = pt.get<double>("Z.setb");
      static1 = X;
      static2 = Z;
      break;
    case Z:
      min   = pt.get<double>("Z.min");
      max   = pt.get<double>("Z.max");
      step  = pt.get<double>("Z.step");
      set1a = pt.get<double>("X.seta");
      set1b = pt.get<double>("X.setb");
      set2a = pt.get<double>("Y.seta");
      set2b = pt.get<double>("Y.setb");
      static1 = X;
      static2 = Y;
      break;
  }

  return (OptimizeSettings) {
    pt.get<uint8_t>("Meta.steps"),
    scan,
    min, max, step,
    static1,
    set1a, set1b,
    static2,
    set2a, set2b
  };
}


double halfw(double lo, double hi) {
  return (lo + hi) / 2;
}


pair<Measurement, Measurement> median_stdev(const vector<Measurement>& measurements) {
  // probably a more efficient way of doing this
  vector<double>
    v0s, v1s, b_up_0s, b_up_1s, b_up_2s, b_dn_0s, b_dn_1s, b_dn_2s, b_sd_0s, b_sd_1s, b_sd_2s;
  vector< vector<double> > vecs = {
    v0s, v1s, b_up_0s, b_up_1s, b_up_2s, b_dn_0s, b_dn_1s, b_dn_2s, b_sd_0s, b_sd_1s, b_sd_2s
  };

  // for (auto vec : vecs) { vec.reserve(measurements.size()); }
  for (int i = 0; i < vecs.size(); i++) { vecs[i].reserve(measurements.size()); }
  
  // for (auto measurement : measurements) {
  for (int i = 0; i < measurements.size(); i++) {
    auto measurement = measurements[i];
    v0s.push_back(measurement.v0);
    v1s.push_back(measurement.v1);
    b_up_0s.push_back(measurement.b_up[0]);
    b_up_1s.push_back(measurement.b_up[1]);
    b_up_2s.push_back(measurement.b_up[2]);
    b_dn_0s.push_back(measurement.b_dn[0]);
    b_dn_1s.push_back(measurement.b_dn[1]);
    b_dn_2s.push_back(measurement.b_dn[2]);
    b_sd_0s.push_back(measurement.b_sd[0]);
    b_sd_1s.push_back(measurement.b_sd[1]);
    b_sd_2s.push_back(measurement.b_sd[2]);
  }

  // for (auto vec : vecs) {
  for (int i = 0; i < vecs.size(); i++) {
    auto vec = vecs[i];
    sort(vec.begin(), vec.end());
  }

  Measurement m;
  if (measurements.size() % 2) { // odd, need to take mean of middle two elements
    int lo = measurements.size() / 2, hi = lo + 1;
    m =  Measurement {
      halfw(v0s[lo], v0s[hi]),
      halfw(v1s[lo], v1s[hi]),
      {halfw(b_up_0s[lo], b_up_0s[hi]), halfw(b_up_1s[lo], b_up_1s[hi]), halfw(b_up_2s[lo], b_up_2s[hi])},
      {halfw(b_dn_0s[lo], b_dn_0s[hi]), halfw(b_dn_1s[lo], b_dn_1s[hi]), halfw(b_dn_2s[lo], b_dn_2s[hi])},
      {halfw(b_sd_0s[lo], b_sd_0s[hi]), halfw(b_sd_1s[lo], b_sd_1s[hi]), halfw(b_sd_2s[lo], b_sd_2s[hi])}
    };
  } else {
    int i = measurements.size() / 2;
    m = Measurement {
      v0s[i],
      v1s[i],
      {b_up_0s[i], b_up_1s[i], b_up_2s[i]},
      {b_dn_0s[i], b_dn_1s[i], b_dn_2s[i]},
      {b_sd_0s[i], b_sd_1s[i], b_sd_2s[i]}
    };
  }

  Measurement mean = {0,0,{0,0,0},{0,0,0},{0,0,0}};
  for (int i = 0; i < measurements.size(); i++) {
    auto meas = measurements[i];
    mean.v0 += meas.v0;
    mean.v1 += meas.v1;
    mean.b_up[0] += meas.b_up[0];
    mean.b_up[1] += meas.b_up[1];
    mean.b_up[2] += meas.b_up[2];
    mean.b_dn[0] += meas.b_dn[0];
    mean.b_dn[1] += meas.b_dn[1];
    mean.b_dn[2] += meas.b_dn[2];
    mean.b_sd[0] += meas.b_sd[0];
    mean.b_sd[1] += meas.b_sd[1];
    mean.b_sd[2] += meas.b_sd[2];
  }
  mean.v0 /= measurements.size();
  mean.v1 /= measurements.size();
  mean.b_up[0] /= measurements.size();
  mean.b_up[1] /= measurements.size();
  mean.b_up[2] /= measurements.size();
  mean.b_dn[0] /= measurements.size();
  mean.b_dn[1] /= measurements.size();
  mean.b_dn[2] /= measurements.size();
  mean.b_sd[0] /= measurements.size();
  mean.b_sd[1] /= measurements.size();
  mean.b_sd[2] /= measurements.size();

  Measurement sum2 = {0,0,{0,0,0},{0,0,0},{0,0,0}};
  for (int i = 0; i < measurements.size(); i++) {
    auto meas = measurements[i];
    sum2.v0 += pow(meas.v0 - mean.v0, 2);
    sum2.v1 += pow(meas.v1 - mean.v1, 2);
    sum2.b_up[0] += pow(meas.b_up[0] - mean.b_up[0], 2);
    sum2.b_up[1] += pow(meas.b_up[1] - mean.b_up[1], 2);
    sum2.b_up[2] += pow(meas.b_up[2] - mean.b_up[2], 2);
    sum2.b_dn[0] += pow(meas.b_dn[0] - mean.b_dn[0], 2);
    sum2.b_dn[1] += pow(meas.b_dn[1] - mean.b_dn[1], 2);
    sum2.b_dn[2] += pow(meas.b_dn[2] - mean.b_dn[2], 2);
    sum2.b_sd[0] += pow(meas.b_sd[0] - mean.b_sd[0], 2);
    sum2.b_sd[1] += pow(meas.b_sd[1] - mean.b_sd[1], 2);
    sum2.b_sd[2] += pow(meas.b_sd[2] - mean.b_sd[2], 2);
  }

  sum2.v0 /= measurements.size() - 1;
  sum2.v1 /= measurements.size() - 1;
  sum2.b_up[0] /= measurements.size() - 1;
  sum2.b_up[1] /= measurements.size() - 1;
  sum2.b_up[2] /= measurements.size() - 1;
  sum2.b_dn[0] /= measurements.size() - 1;
  sum2.b_dn[1] /= measurements.size() - 1;
  sum2.b_dn[2] /= measurements.size() - 1;
  sum2.b_sd[0] /= measurements.size() - 1;
  sum2.b_sd[1] /= measurements.size() - 1;
  sum2.b_sd[2] /= measurements.size() - 1;

  sum2.v0 = sqrt(sum2.v0);
  sum2.v1 = sqrt(sum2.v1);
  sum2.b_up[0] = sqrt(sum2.b_up[0]);
  sum2.b_up[1] = sqrt(sum2.b_up[1]);
  sum2.b_up[2] = sqrt(sum2.b_up[2]);
  sum2.b_dn[0] = sqrt(sum2.b_dn[0]);
  sum2.b_dn[1] = sqrt(sum2.b_dn[1]);
  sum2.b_dn[2] = sqrt(sum2.b_dn[2]);
  sum2.b_sd[0] = sqrt(sum2.b_sd[0]);
  sum2.b_sd[1] = sqrt(sum2.b_sd[1]);
  sum2.b_sd[2] = sqrt(sum2.b_sd[2]);

  return make_pair(m, sum2);
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