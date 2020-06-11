#include "rotations.hpp"


Vec3 rotate_point(Vec3 p, Vec3 about, Quaternion q) {
  return vector_part(q * Quaternion::from_vec3(p - about) * conjugate(q)) + about;
}


Vec3 rotate_point(Vec3 p, Vec3 about, double theta, double phi) {
  Quaternion q = Quaternion::from_spherical_angle(theta, phi);
  return rotate_point(p, about, q);
}
