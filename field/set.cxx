#include "util.hxx"
#include <algorithm>


vector<string> parse_args(int argc, char** argv) {
  vector<string> ret;
  ret.reserve(argc-1);
  for (int i = 1; i < argc; i++) ret.push_back(string(argv[i]));
  return ret;
}

#define PRINT_USAGE cout << "Usage: \033[1mset dimension v1 v2\033[0m" << endl


array<double, 6> get_targets(vector<string> args) {
  array<double, 6> ret;
  for (int i = 0; i < 6; i++) {
    ret[i] = stod(args[i]);
  }
  return ret;
}


int main(int argc, char** argv) {
  auto args = parse_args(argc, argv);

  if (find(args.begin(), args.end(), "--help") != args.end()
      || args.size() != 6) {
    PRINT_USAGE;
    return 1;
  }

  const uint64_t num_steps = 8;

  const auto targets = get_targets(args);
  const array<Coil, 6> coils = {Coil1, Coil2, Coil3, Coil4, Coil5, Coil6};

 const array< vector<double>, 6 > paths = {
   degauss_path(targets[0], coils[0], num_steps),
   degauss_path(targets[1], coils[1], num_steps),
   degauss_path(targets[2], coils[2], num_steps),
   degauss_path(targets[3], coils[3], num_steps),
   degauss_path(targets[4], coils[4], num_steps),
   degauss_path(targets[5], coils[5], num_steps)
 };

  cerr << "Setting coils:" << endl;

  for (int i = 0; i < 6; i++) {
    cerr << "  Coil " << (i+1) << ": " << targets[i] << "V" << endl;
  }

  const vector<uint32_t> phidgets = {0,1,3};

  auto iface = ODBInterface(phidgets);

  array<double, 6> mv;

  for (int i = 0; i < num_steps; i++) {
    cerr << "\r\033[KDegaussing: [";

    for (int j = 0; j < num_steps; j++) {
      if (j < i) {
        cerr << "\033[107m \033[0m";
      } else {
        cerr << " ";
      }
    }
    cerr << "] (" << (i+1) << "/" << num_steps << ")";

    array<double, 6> vs = {
      paths[0][i], paths[1][i], paths[2][i], paths[3][i], paths[4][i], paths[5][i]
    };

    for (int j = 0; j < 6; j++) {
      iface.set_coil_voltage(coils[j], vs[j]);
    }

    for (int j = 0; j < 6; j++) {
      mv[j] = iface.get_coil_voltage(coils[j]);
    }

    while (
      fabs(paths[0][i] - mv[0]) > MARGIN ||
      fabs(paths[1][i] - mv[1]) > MARGIN ||
      fabs(paths[2][i] - mv[2]) > MARGIN ||
      fabs(paths[3][i] - mv[3]) > MARGIN ||
      fabs(paths[4][i] - mv[4]) > MARGIN ||
      fabs(paths[5][i] - mv[5]) > MARGIN
    ) {
      semibusy_wait(1500);
      for (int j = 0; j < 6; j++) {
        mv[j] = iface.get_coil_voltage(coils[j]);
      }
      cerr << ".";
    }
    semibusy_wait(100);
  }

  cerr << "\r\033[KDeguassing: [";
  for (int j = 0; j < num_steps; j++) {
    cerr << "\033[107m \033[0m";
  }
  cerr << "] Done" << endl;

  for (int j = 0; j < 6; j++) {
    iface.set_coil_voltage(coils[j], targets[j]);
  };

  for (int j = 0; j < 6; j++) {
    mv[j] = iface.get_coil_voltage(coils[j]);
  }

  while (
    fabs(targets[0] - mv[0]) > MARGIN &&
    fabs(targets[1] - mv[1]) > MARGIN &&
    fabs(targets[2] - mv[2]) > MARGIN &&
    fabs(targets[3] - mv[3]) > MARGIN &&
    fabs(targets[4] - mv[4]) > MARGIN &&
    fabs(targets[5] - mv[5]) > MARGIN
  ) {
    semibusy_wait(100);
    for (int j = 0; j < 6; j++) {
      mv[j] = iface.get_coil_voltage(coils[j]);
    }
  }

  cerr << "Done." << endl;

  return 0;
}
