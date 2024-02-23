#include "degauss.hxx"


pair<Coil, Coil> coils_for_dimension(Dimension dim) {
  switch(dim) {
    case X:
      return make_pair(Coil3, Coil4);
    case Y:
      return make_pair(Coil5, Coil6);
    case Z:
      return make_pair(Coil1, Coil2);
    default:
      throw "Should not happen.";
  }
}


double max_voltage(Coil coil) {
  switch(coil) {
    case Coil1:
    case Coil2:
      return 15;
    default:
      return 8;
  }
}


double resistance(Coil coil) {
  // determined from indirect measurements
  switch (coil) {
    case Coil1:
      return 3.3417885413735293;
    case Coil2:
      return 3.2843600061455835;
    case Coil3:
      return 3.2872877504450533;
    case Coil4:
      return 3.4255084639629496;
    case Coil5:
      return 11.098866996360902;
    case Coil6:
      return 11.014250996387714;
    default:
      throw "Should not happen.";
  }
}


 pair<double, double> exponential_factors(double target_voltage, Coil coil, uint32_t steps) {
  const double max_volts = max_voltage(coil) - 1; // margin for safety

  double k = -log(MIN_V_DIFF) / ((double) steps),//-log(MIN_V_DIFF * exp(-((double)steps))),
         c = max_volts - target_voltage;

  if (target_voltage - c * exp(-k) <= 0.05) {
    double c_ = (target_voltage - 0.05) / exp(-k);
    if (c_ > c) {
      throw "Could not find appropriate c";
    }
    c = c_;
  }

  return make_pair(c, k);
}


vector<double> degauss_path(double target_voltage, Coil coil, uint32_t steps) {
  auto facts = exponential_factors(target_voltage, coil, steps);
  const double
    c = facts.first,
    k = facts.second;
  
  // std::cout << "c = " << c << "; k = " << k << std::endl;
  
  // auto volts = [target_voltage, c, k] (uint32_t i) -> double {
  //   return ((i % 2) ? -1 : 1) * c * exp(-k * (double)i) + target_voltage;
  // };

  vector<double> ret;
  ret.reserve(steps);

  for (uint32_t i = 0; i < steps; i++) {
    ret.push_back(((i % 2) ? -1 : 1) * c * exp(-k * (double)i) + target_voltage);
  }

  return ret;
}


// LinspaceMatrix::LinspaceMatrix(uint32_t _res, double _lo_x, double _hi_x, double _lo_y, double _hi_y)
//   : res(_res), lo_x(_lo_x), step_x((_hi_x - lo_x) / ((double)res)), lo_y(_lo_y), step_y((_hi_y - lo_y) / ((double)res))
// {
//   if (_hi_x >= _lo_x || _hi_y >= _lo_y) {
//     throw "High must be greater than low.";
//   } else if (res == 0) {
//     throw "Resolution may not be zero.";
//   }
// }
