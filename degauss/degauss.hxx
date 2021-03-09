#ifndef __OPTIM_UTIL
#define __OPTIM_UTIL


#include <utility>
#include <vector>
#include <cstdint>
#include <cmath>
#include <iostream>


#define MIN_V_DIFF 0.01
#define V_LOW_MARGIN 0.05


// For voltage at step i:
// target_voltage + (-1)^i * c * exp(-ik)
// k is chosen so that exp(-ik) goes from 1 to MIN_V_DIFF,
// and c is as large as possible so that the coil does not
// exceed voltage limitations


using namespace std;


enum Dimension {
  X,
  Y,
  Z
};


enum Coil {
  Coil1,
  Coil2,
  Coil3,
  Coil4,
  Coil5,
  Coil6
};


typedef struct {
  double c;
  double k;
} ExponentialFactors;


// which coils correspond to which dimension
pair<Coil, Coil> coils_for_dimension(Dimension dim);


// return the maximum voltage for a coil
double max_voltage(Coil coil);


// the resistance for a coil
double resistance(Coil coil);


// returns c, k (see above)
ExponentialFactors exponential_factors(double target_voltage, Coil coil, size_t steps);


// returns vector of voltage to approach final value
// does _not_ include final voltage
vector<double> degauss_path(double target_voltage, Coil coil, size_t n_steps);


// gets a particular degauss step
double degauss_step(double target_voltage, Coil coil, size_t step, size_t n_steps);


#endif
