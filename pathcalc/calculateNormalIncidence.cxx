#include <cmath>
#include <vector>
#include "calculateNormalIncidence.hxx"

std::vector<double>
calculateNormalIncidence(double center_x, double center_y, double radius, double beam, double x, double y,
                         double incidence, double azimuth) {

  const double PI = 3.141592653589793238463;

  std::vector<double> arr;

  double z, gamma, alpha, x_origin, y_origin, z_origin,
      x_inc, y_inc, z_inc, x_inc_temp, y_inc_temp,
      tilt_angle, rot_angle, offset1_g0, offset2_g0,
      offset3_g0, offset1_g1, offset2_g1, offset3_g1,
      g0_o_mag, g1_o_mag, g0_final_offset_x, g0_final_offset_y,
      g0_final_offset_z, g1_final_offset_x, g1_final_offset_y, g1_final_offset_z,
      output_rot_angle, whichGantry;

  //Calculations for placement of point source

  //Calculates z position on PMT surface
  z = radius * cos(asin(sqrt(x * x + y * y) / radius));

  //Converts angular arguments to radians for use with trig functions
  incidence = PI * incidence / 180;
  azimuth = PI * azimuth / 180;

  //Calculates spherical coordinate angles
  gamma = atan2(y, x);
  alpha = asin(sqrt(x * x + y * y) / radius);

  //Defines point that will be examined at different angles
  x_origin = x + center_x;
  y_origin = y + center_y;
  z_origin = z;

  //Conversion of spherical coordinates to cartesian
  x_inc = beam * sin(incidence) * cos(azimuth);
  y_inc = beam * sin(incidence) * sin(azimuth);
  z_inc = beam * cos(incidence);

  //Rotation about y-axis of PMT
  x_inc_temp = x_inc;
  x_inc = cos(alpha) * x_inc + sin(alpha) * z_inc;
  z_inc = (-1) * sin(alpha) * x_inc_temp + cos(alpha) * z_inc;

  //Rotation about x-axis of PMT
  x_inc_temp = x_inc;
  x_inc = cos(gamma) * x_inc - sin(gamma) * y_inc;
  y_inc = sin(gamma) * x_inc_temp + cos(gamma) * y_inc;

  //Superposition of coordinates to achieve point source positioning
  x = x_origin + x_inc;
  y = y_origin + y_inc;
  z = z_origin + z_inc;

  //Calculates rotation and tilt angle that describe the beam from the gantry
  rot_angle = atan2(y_inc, x_inc);
  tilt_angle = atan2(z_inc, sqrt(x_inc * x_inc + y_inc * y_inc));

  //Corrections for gantry 1 and gantry 0 optical boxes
  //Offsets for Gantry 0
  offset1_g0 = 0.12;
  offset2_g0 = 0.044;
  offset3_g0 = -0.052;
  g0_o_mag = sqrt(offset1_g0 * offset1_g0 + offset2_g0 * offset2_g0 + offset3_g0 * offset3_g0);

  //Offsets for Gantry 1
  offset1_g1 = 0.12;
  offset2_g1 = 0.044;
  offset3_g1 = (-1) * 0.052;
  g1_o_mag = sqrt(offset1_g1 * offset1_g1 + offset2_g1 * offset2_g1 + offset3_g1 * offset3_g1);

  g0_final_offset_x = offset1_g0 * cos(rot_angle) * cos(tilt_angle) - offset2_g0 * sin(rot_angle) +
                      offset3_g0 * cos(rot_angle) * sin(tilt_angle);
  g0_final_offset_y = offset1_g0 * sin(rot_angle) * cos(tilt_angle) + offset2_g0 * cos(rot_angle) +
                      offset3_g0 * sin(rot_angle) * sin(tilt_angle);
  g0_final_offset_z = (-1) * offset1_g0 * sin(tilt_angle) + offset3_g0 * cos(tilt_angle);

  g1_final_offset_x = offset1_g1 * cos(rot_angle) * cos(tilt_angle) - offset2_g1 * sin(rot_angle) +
                      offset3_g1 * cos(rot_angle) * sin(tilt_angle);
  g1_final_offset_y = offset1_g1 * sin(rot_angle) * cos(tilt_angle) + offset2_g1 * cos(rot_angle) +
                      offset3_g1 * sin(rot_angle) * sin(tilt_angle);
  g1_final_offset_z = (-1) * offset1_g1 * sin(tilt_angle) + offset3_g1 * cos(tilt_angle);

  if (std::abs(rot_angle) <= 110 * PI / 180) {
    x = x - g0_final_offset_x;
    y = y - g0_final_offset_y;
    z = z - g0_final_offset_z;
    whichGantry = 0;
    output_rot_angle = rot_angle;
  } else {

    x = x - g1_final_offset_x;
    y = y - g1_final_offset_y;
    z = z - g1_final_offset_z;

    if (std::abs(rot_angle) > 90 * PI / 180) {
      output_rot_angle = rot_angle - PI;
    } else {
      output_rot_angle = PI - rot_angle;
    }
    whichGantry = 1;
  }

  //Final conversions to gantry frame
  z = 0.875 + 0.04 - z;
  tilt_angle = tilt_angle * (-1);

  //sets up return array (non-mutable)
  arr.push_back(x);
  arr.push_back(y);
  arr.push_back(z);
  arr.push_back(output_rot_angle * 180 / PI);
  arr.push_back(tilt_angle * 180 / PI);
  arr.push_back(whichGantry);

  return arr;
}
