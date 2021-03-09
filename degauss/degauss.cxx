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


ExponentialFactors exponential_factors(double target_voltage, Coil coil, size_t steps) {
  const double max_volts = max_voltage(coil) - 1; // margin for safety

  double k = -log(MIN_V_DIFF) / ((double) steps),//-log(MIN_V_DIFF * exp(-((double)steps))),
         c = max_volts - target_voltage;

  if (target_voltage - c * exp(-k) <= V_LOW_MARGIN) {
    double c_ = (target_voltage - V_LOW_MARGIN) / exp(-k);
    if (c_ > c) {
      throw "Could not find appropriate c";
    }
    c = c_;
  }

  return{c, k};
}


inline double _degauss_step(double v, size_t i, ExponentialFactors f) {
  return ((i % 2) ? -1 : 1) * f.c * exp(-f.k * (double)i) + v;
}


vector<double> degauss_path(double target_voltage, Coil coil, size_t n_steps) {
  const auto facts = exponential_factors(target_voltage, coil, n_steps);

  vector<double> ret;
  ret.reserve(n_steps);

  for (size_t i = 0; i < n_steps; i++) {
    ret.push_back(_degauss_step(target_voltage, i, facts));
  }

  return ret;
}


double degauss_step(double target_voltage, Coil coil, size_t step, size_t n_steps) {
  const auto facts = exponential_factors(target_voltage, coil, n_steps);
  return _degauss_step(target_voltage, step, facts);
}
