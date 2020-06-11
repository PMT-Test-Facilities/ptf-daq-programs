#include "geom.hpp"
#include <cmath>
#include <cstdio>
#include <string>
#include <iostream>
#include <ostream>
#include <boost/math/quaternion.hpp>
#include <cstring>


void c2s(double x, double y, double z, double* rho, double* theta, double* phi) {
  *rho   = sqrt((x*x) + (y*y) + (z*z));
  *theta = atan2(y, x);
  *phi   = atan2(sqrt((x*x) + (y*y)), z);
}


void s2c(double rho, double theta, double phi, double* x, double* y, double* z) {
  *x = rho * sin(phi) * cos(theta);
  *y = rho * sin(phi) * sin(theta);
  *z = rho * cos(theta);
}


struct conversion_visitor : public boost::static_visitor<boost::optional<Intersectable>> {
  template<typename T>
  boost::optional<Intersectable> operator()(const T t) const {
    return Intersectable(t);
  }

  // template<typename T>
  // boost::optional<Intersectable> operator()(const T& t) {
  //   return Intersectable(t);
  // }

  boost::optional<Intersectable> operator()(const Quaternion q) const {
    return boost::none;
  }
};


boost::optional<Intersectable> geometry_to_intersectable(GeometryObject g) {
  return boost::apply_visitor(conversion_visitor(), g);
}


Vec3 centroid(const vector<Vec3>& vertexes) {
  Vec3 c = Vec3::zero();
  for (size_t i = 0; i < vertexes.size(); i++) {
    c = c + vertexes[i];
  }
  return c / vertexes.size();
}


std::tuple<Vec3, double> furthest(Vec3 from, const vector<Vec3>& to) {
  double max_dist = -1;
  Vec3   furthest;
  for (size_t i = 0; i < to.size(); i++) {
    double d = norm2(to[i] - from);
    if (d > max_dist) {
      max_dist = d;
      furthest = to[i];
    }
  }
  return make_tuple(furthest, sqrt(max_dist));
}
