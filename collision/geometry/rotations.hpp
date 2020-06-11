#ifndef __ROTATIONS
#define __ROTATIONS

#include "vec3.hpp"
#include "quaternion.hpp"
#include <array>


Vec3 rotate_point(Vec3 p, Vec3 about, double theta, double phi);
Vec3 rotate_point(Vec3 p, Vec3 about, Quaternion q);

template<size_t n>
void rotate_points_inplace(std::array<Vec3, n>& p, Vec3 about, Quaternion q) {
  for (size_t i = 0; i < n; i++) {
    p[i] = rotate_point(p[i], about, q);
  }
}

template<size_t n>
void rotate_points_copy(const std::array<Vec3, n>& from, std::array<Vec3, n>& to, Vec3 about, Quaternion q) {
  for (size_t i = 0; i < n; i++) {
    to[i] = rotate_point(from[i], about, q);
  }
}

#endif
