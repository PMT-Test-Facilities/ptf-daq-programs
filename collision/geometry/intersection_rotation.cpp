#include "geom.hpp"


/*
[ ] Prism Vec3
[x] Prism LineSegment
[x] Prism Prism
[x] Prism Sphere
[x] Prism Cylinder
*/


using namespace std;


// calculates number of steps required to get BASE_ROT_RESOLUTION precision
// distance is distance from centre of rotation to furthest object
// returns tuple of <num_steps, error_factor>
tuple<size_t, double> _precision(const Quaternion rotation, const Vec3 about, const Vec3 sample) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);

  const double
    r      = norm(sample - about),
    theta  = angle(rotation);
  
  const size_t n = max((size_t)ceil(theta / (BASE_ROT_RESOLUTION / r)), (size_t)MINIMUM_ROT_STEPS);
  // const size_t n = 2;
  const double
    dtheta  = theta / n,
    err_fac = (dtheta < 0.1) ? (r + dtheta) / r : (r + 2*sin(dtheta/2)) / r; // small angle approx
    // err_fac = sqrt(pow(1 / (cos(dtheta / 2)), 2) + pow(1/(sin(dtheta / 2)), 2)) / r;
  
  /*cerr << "    r=" << r << "\n"
       << "    t=" << theta << "\n"
       << "    n=" << n << "\n"
       << "    e=" << err_fac << "\n"
       << std::flush;*/
  DEBUG_COUT("For rotation of " << theta << "rad at a distance of " << r << "m, found n=" << n << " and error " << err_fac);

  DEBUG_LEAVE;
  return make_tuple(n, err_fac);
}


/* Prism <=> Vec3 */
bool intersect(Prism x, Vec3 y, Quaternion rotation, Vec3 about) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);
  // transform into prism frame
  const auto f = furthest(about, x.vertexes());

  size_t n_steps;
  double err;
  tie(n_steps, err) = _precision(rotation, about, get<0>(f));

  const double fac = 1 / ((double) n_steps);

  Quaternion rot;
  Prism p;
  for (size_t step = 0; step <= n_steps; step++) {
    rot = slerp(Quaternion::identity(), rotation, step*fac);

    p = {rotate_point(x.center, about, rot), x.ex * err, x.ey * err, x.ez * err, rot * x.orientation};
    if (intersect(p, y)) {
      DEBUG_COUT("Found intersection on step " << step << "/" << n_steps);
      DEBUG_LEAVE;
      return true;
    }
  }

  DEBUG_COUT("No intersection found.");
  DEBUG_LEAVE;
  return false;
}


bool intersect(Prism x, LineSegment y, Quaternion rotation, Vec3 about) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);
  size_t n_steps;
  double err;
  
  auto f = furthest(about, x.vertexes());

  tie(n_steps, err) = _precision(rotation, about, get<0>(f));

  const double fac = 1/((double) n_steps);

  Quaternion rot;
  Prism p;
  for (size_t step = 0; step <= n_steps; step++) {
    rot = slerp(Quaternion::identity(), rotation, step*fac);

    p   = {rotate_point(x.center, about, rot), x.ex * err, x.ey * err, x.ez * err, rot * x.orientation};
    if (intersect(p, y)) {
      DEBUG_COUT("Found intersection on step " << step << "/" << n_steps);
      DEBUG_LEAVE;
      return true;
    }
  }

  DEBUG_COUT("No intersection found.");
  DEBUG_LEAVE;
  return false;
}


bool intersect(Prism x, Prism y, Quaternion rotation, Vec3 about) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);
  size_t n_steps;
  double err;

  auto f = furthest(about, y.vertexes());

  tie(n_steps, err) = _precision(rotation, about, get<0>(f));

  const double fac = 1/((double) n_steps);

  Quaternion rot;
  Prism p;
  for (size_t step = 0; step <= n_steps; step++) {
    rot = slerp(Quaternion::identity(), rotation, step*fac);

    p   = {rotate_point(x.center, about, rot), x.ex * err, x.ey * err, x.ez * err, rot * x.orientation};
    if (intersect(p, y)) {
      DEBUG_COUT("Found intersection on step " << step << "/" << n_steps);
      DEBUG_LEAVE;
      return true;
    }
  }

  DEBUG_COUT("No intersection found.");
  DEBUG_LEAVE;
  return false;
}


bool _bounds_intersect(Prism x, Sphere y, Vec3 about) {
  // computes a bounding sphere for the rotating prism
  // first find point furthest from point we're rotating about
  auto pts = x.vertexes();
  array<double, 8> distances;
  int max_idx = 0;
  for (int i = 0; i < 8; i++) {
    distances[i] = norm(pts[i] - about);
    if (distances[i] > distances[max_idx]) {
      max_idx = i;
    }
  }

  const Sphere bound = { about, distances[max_idx] };

  return intersect(bound, y);
}


bool intersect(Prism x, Sphere y, Quaternion rotation, Vec3 about) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);
  // first, check bounding spheres
  if (!_bounds_intersect(x, y, about)) {
    DEBUG_COUT("Bounds don't intersect.");
    DEBUG_LEAVE;
    return false;
  }

  // now find the furthest point from the sphere
  auto pts = x.vertexes();

  // double maxdist = nanf("");
  // Vec3 furthest;

  auto f = furthest(about, pts);

  Vec3 fv = get<0>(f);

  size_t n_steps;
  double err;

  tie(n_steps, err) = _precision(rotation, about, fv);

  const double fac = 1/((double) n_steps);

  Quaternion rot;
  Prism p;
  for (size_t step = 0; step <= n_steps; step++) {
    rot = slerp(Quaternion::identity(), rotation, step*fac);

    p   = {rotate_point(x.center, about, rot), x.ex, x.ey, x.ez, rot * x.orientation};
    if (intersect(p, y)) {
      DEBUG_COUT("Found intersection on step " << step << "/" << n_steps);
      DEBUG_LEAVE;
      return true;
    }
  }

  DEBUG_COUT("No intersection found.");
  DEBUG_LEAVE;
  return false;
}


bool intersect(Prism x, Cylinder y, Quaternion rotation, Vec3 about) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);
  // first, check bounding spheres
  if (!_bounds_intersect(x, bounding_sphere(y), about)) {
    DEBUG_COUT("Bounds don't intersect.");
    DEBUG_LEAVE;
    return false;
  }

  // now find the furthest point from the sphere
  auto pts = x.vertexes();

  // double maxdist = nanf("");
  // Vec3 furthest;

  auto f = furthest(about, pts);

  Vec3 fv = get<0>(f);

  size_t n_steps;
  double err;

  tie(n_steps, err) = _precision(rotation, about, fv);

  const double fac = 1/((double) n_steps);

  Quaternion rot;
  Prism p;
  for (size_t step = 0; step <= n_steps; step++) {
    rot = slerp(Quaternion::identity(), rotation, step*fac);

    p   = {rotate_point(x.center, about, rot), x.ex, x.ey, x.ez, rot * x.orientation};
    if (intersect(p, y)) {
      DEBUG_COUT("Found intersection on step " << step << "/" << n_steps);
      DEBUG_LEAVE;
      return true;
    }
  }

  DEBUG_COUT("No intersection found.");
  DEBUG_LEAVE;
  return false;
}


struct rotation_intersect_visitor : public boost::static_visitor<bool> {
  rotation_intersect_visitor(Prism prism, Quaternion rotation, Vec3 about)
    : p(prism), r(rotation), a(about) {}
  Prism p;
  Quaternion r;
  Vec3 a;

  template<typename T>
  bool operator()(T t) const {
    return intersect(p, t, r, a);
  }
};


bool intersect(Prism x, Intersectable y, Quaternion rotation, Vec3 about) {
  auto visitor = rotation_intersect_visitor(x, rotation, about);
  return boost::apply_visitor(visitor, y);
}


bool intersect(Prism x, vector<Intersectable> ys, Quaternion rotation, Vec3 about) {
  auto visitor = rotation_intersect_visitor(x, rotation, about);
  for (size_t i = 0; i < ys.size(); i++) {
    if (boost::apply_visitor(visitor, ys[i])) return true;
  }
  return false;
}
