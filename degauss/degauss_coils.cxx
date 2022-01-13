#include "degauss.hxx"
#include "odblayer.hxx"
#include "util.hxx"


#include <algorithm>
#include <iostream>
#include <iomanip>
#include <thread>
#include <fstream>
#include <unistd.h>
#include <ctime>
#include <sys/time.h>


// typedef __int128 int128_t;
// typedef unsigned __int128 uint128_t;

/*
 * how to use optimize.ini:
 *   - Each dimension should have an ini group
 *   - Each dimension should have a 'mode' property, which can be 'scan' or 'static'
 *   - Exactly one dimension should have 'scan' mode
 *   - For scan mode, the following properties are needed:
 *     - 'min': minimum value for coils
 *     - 'max': maximum value for coils
 *     - 'step': voltage increment
 *   - For static mode, the following properties are needed:
 *     - 'seta': the voltage for the _lower_ of the coils. For example, for Y this would be coil 5.
 *     - 'setb': the voltage for the _higher_ of the coils. For examplee, for Y this would be coil 6.
 */



double get_random_coil_voltage() {
  return (double) (rand() % 40)/10 + 2;
}


#define RANDOMIZE_OTHER_COILS false

int main(void) {
  const auto set = load_settings();

  const Dimension
    
    s1 = set.static1,
    s2 = set.static2,
    s3  = set.static3;
  const auto
    
    c1 = coils_for_dimension(s1),
    c2 = coils_for_dimension(s2),
    c3 = coils_for_dimension(s3);

  const array<Coil, 6> all_coils = {c1.first, c1.second, c2.first, c2.second, c3.first, c3.second};

  const auto
    dg1a = degauss_path(set.set1a, c1.first, set.n_steps),
    dg1b = degauss_path(set.set1b, c1.second, set.n_steps),
    dg2a = degauss_path(set.set2a, c2.first, set.n_steps),
    dg2b = degauss_path(set.set2b, c2.second, set.n_steps),
    dg3a= degauss_path(set.set3a, c3.first, set.n_steps),
    dg3b= degauss_path(set.set3b, c3.second, set.n_steps);

   const vector<uint32_t> phidgets = {0,4}; 
   auto iface = ODBInterface(phidgets);
  
   //vector<IdxMeasurement> measurements;
   //vector<Measurement>    stdevs;
   //vector<double> v0s, v1s;

   //for (double v0 = set.min; v0 <= set.max + 0.001; v0 += set.incr) v0s.push_back(v0);
   //for (double v1 = set.min; v1 <= set.max + 0.001; v1 += set.incr) v1s.push_back(v1);

   // cerr << "Scanning dimension " << name_for_dim(d) << ":" << endl
   //   << "  Scanning coil " << (coils.first+1) << " from " << v0s[0] << "V to " << v0s[v0s.size()-1] << "V in increments of " << set.incr << "V." << endl
   //    << "  Scanning coil " << (coils.second+1) << " from " << v1s[0] << "V to " << v1s[v1s.size()-1] << "V in increments of " << set.incr << "V." << endl
   //    << "  Steps: " << v0s.size() << ", " << v1s.size() << " (" << v0s.size()*v1s.size() << " total)" << endl;

   // cerr << "Holding dimension " << name_for_dim(s1) << " at (" << set.set1a << "V, " << set.set1b << "V)." << endl
   //    << "Holding dimension " << name_for_dim(s2) << " at (" << set.set2a << "V, " << set.set2b << "V)." << endl << endl;

   //for (int i = 0; i < v0s.size(); i++) {
   // const double v0 = v0s[i];
   // const auto dg0  = degauss_path(v0, coils.first, set.n_steps);

   // for (int j = 0; j < v1s.size(); j++) {
   //   const double v1 = v1s[j];
   //   const auto dg1  = degauss_path(v1, coils.second, set.n_steps);

      cerr << "\033[0m\033[1m Starting  Degaussing...\033[0m" << endl;

      for (uint32_t _i = 0; _i < dg1a.size(); _i++) {
        iface.set_coil_voltage(c1.first,  dg1a[_i]);
        iface.set_coil_voltage(c1.second, dg1b[_i]);
        iface.set_coil_voltage(c2.first,  dg2a[_i]);
        iface.set_coil_voltage(c2.second, dg2b[_i]);
	iface.set_coil_voltage(c3.first,  dg3a[_i]);
        iface.set_coil_voltage(c3.second, dg3b[_i]);

        array<double, 6> vs = {dg1a[_i], dg1b[_i], dg2a[_i], dg2b[_i], dg3a[_i], dg3b[_i]};

        wait_for_all(iface, all_coils, vs);
        semibusy_wait(1000);
      }

      cerr << "  \033[1mDegauss done.\033[0m" << endl;

      iface.set_coil_voltage(c1.first,  set.set1a);
      iface.set_coil_voltage(c1.second, set.set1b);
      iface.set_coil_voltage(c2.first,  set.set2a);
      iface.set_coil_voltage(c2.second, set.set2b);
      iface.set_coil_voltage(c3.first,  set.set3a);
      iface.set_coil_voltage(c3.second, set.set3b);
      array<double, 6> vs = {set.set1a, set.set1b, set.set2a, set.set2b, set.set3a, set.set3b};
      cerr << "  \033[1mPutting all the coils to their voltage\033[0m" << endl;
      wait_for_all(iface, all_coils, vs);
      semibusy_wait(1000);

      // now we take measurements
      //vector<Measurement> _measurements;
      //for (int _i = 0; _i < N_SAMPLES; _i++) {
      // cerr << "\r\033[KTaking sample " << _i+1 << "/" << N_SAMPLES;
      // double mv0 = iface.get_coil_voltage(coils.first);
      //  double mv1 = iface.get_coil_voltage(coils.second);
      // cerr << ", at (" << mv0 << "V, " << mv1 << "V)";

      // auto mp_dn = iface.get_phidget_reading(3); 
      //  auto mp_up = iface.get_phidget_reading(0);
      // auto mp_sd = iface.get_phidget_reading(1);

      // _measurements.push_back((Measurement){
      //   mv0, mv1,
      //   {mp_up.b0, mp_up.b1, mp_up.b2},
      //   {mp_dn.b0, mp_dn.b1, mp_dn.b2},
      //  {mp_sd.b0, mp_sd.b1, mp_sd.b2}
      // });

      //  if (i != N_SAMPLES - 1)
      //    semibusy_wait(1000);
      // }

      //cerr << "\r  \033[1m" << N_SAMPLES << "/" << N_SAMPLES << " samples done.\033[0m\033[K" << endl;

      /*
      auto median_std = median_stdev(_measurements);
      auto __measurement = median_std.first;
      auto __stdev       = median_std.second;
      IdxMeasurement _measurement ={
        i, j, __measurement.v0, __measurement.v1,
        {__measurement.b_up[0], __measurement.b_up[1], __measurement.b_up[2]},
        {__measurement.b_dn[0], __measurement.b_dn[1], __measurement.b_dn[2]},
        {__measurement.b_sd[0], __measurement.b_sd[1], __measurement.b_sd[2]}
      };
      measurements.push_back(_measurement);
      stdevs.push_back(__stdev);
      cerr << endl;
      */
      }
  

  // write to CSV
  /*
  ofstream csv;
  csv.open("field_scan.csv");

  csv << "# Dimension " << d << ", from " << set.min << " to " << set.max << " in steps of " << set.incr << endl;

  csv << "i, j, v0, v1, v0_std, v1_std, "
      << "top_bx, top_by, top_bz, "
      << "btm_bx, btm_by, btm_bz, "
      << "side_bx, side_by, side_bz, "
      << "top_bx_std, top_by_std, top_bz_std, "
      << "btm_bx_std, btm_by_std, btm_bz_std, "
      << "side_bx_std, side_by_std, side_bz_std"
      << endl;

  // for (auto measurement : measurements) {
  for (int i = 0; i < measurements.size(); i++) {
    auto measurement = measurements[i];
    auto stdev = stdevs[i];

    csv << measurement.i << ","
        << measurement.j << ","
        << measurement.v0 << ","
        << measurement.v1 << ","
        << stdev.v0 << ","
        << stdev.v1 << ","
        << measurement.b_up[0] << ","
        << measurement.b_up[1] << ","
        << measurement.b_up[2] << ","
        << measurement.b_dn[0] << ","
        << measurement.b_dn[1] << ","
        << measurement.b_dn[2] << ","
        << measurement.b_sd[0] << ","
        << measurement.b_sd[1] << ","
        << measurement.b_sd[2] << ","
        << stdev.b_up[0] << ","
        << stdev.b_up[1] << ","
        << stdev.b_up[2] << ","
        << stdev.b_dn[0] << ","
        << stdev.b_dn[1] << ","
        << stdev.b_dn[2] << ","
        << stdev.b_sd[0] << ","
        << stdev.b_sd[1] << ","
        << stdev.b_sd[2] << endl;
  }

  csv.close();
}  
*/

