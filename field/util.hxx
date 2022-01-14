#ifndef __UTILI__
#define __UTILI__

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <thread>
#include <fstream>
#include <unistd.h>
#include <ctime>
#include <sys/time.h>
#include <array>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>


#include "degauss.hxx"
#include "odblayer.hxx"


#define V_RES   0.3
#define N_STEPS 8
#define MARGIN  0.05 // number of Volts off we'll accept the value before moving on
#define N_SAMPLES 10



struct OptimizeSettings {
  uint8_t   n_steps;

  Dimension scan;
  double    min;
  double    max;
  double    incr;

  Dimension static1;
  double    set1a;
  double    set1b;

  Dimension static2;
  double    set2a;
  double    set2b;
};


OptimizeSettings load_settings(std::string file_name = "optimize.ini");


struct Measurement {
  double v0;
  double v1;
  double b_up[3];
  double b_dn[3];
  double b_sd[3];
};


struct IdxMeasurement {
  int i;
  int j;
  double v0;
  double v1;
  double b_up[3];
  double b_dn[3];
  double b_sd[3];
};


pair<Measurement, Measurement> median_stdev(const vector<Measurement>& measurements);


bool wait_for_all(
  ODBInterface& iface,
  array<Coil, 6> coils,
  array<double, 6> voltages,
  uint32_t milli_max_time = 30000,
  uint32_t milli_delay    = 500
);


bool wait_for_coils(
  ODBInterface& iface,
  Coil c0, Coil c1, double v0, double v1,
  uint32_t milli_max_time = 30000,
  uint32_t milli_delay    = 500
);


void semibusy_wait(uint32_t millis);


static const array<Coil, 6> ALL_COILS = {Coil1, Coil2, Coil3, Coil4, Coil5, Coil6};


#endif
