#ifndef __GEOM__
#define __GEOM__
#include <cmath>
#include <string>
#include <iostream>
#include <ostream>
#include <cstring>
#include <unordered_set>
#include <unordered_map>
#include <utility>
#include <array>
#include <vector>
#include <boost/variant.hpp>
#include <boost/optional.hpp>

#include "col.hpp"
#include "debug.hpp"
#include "stl_reader/stl_reader.h"

#include "vec3.hpp"
#include "quaternion.hpp"
#include "prism.hpp"
#include "rotations.hpp"
#include "linesegment.hpp"


// by default, divide rotations so that the displacement (arc length) for individual checks will not exceed this value [m]
// note that making this larger shouldn't produce any false negatives, but will increase false positives for intersection
#define BASE_ROT_RESOLUTION 0.001
// if our resolution means we'd have fewer than this many steps, increase it
#define MINIMUM_ROT_STEPS 32
// pad rotation collision by this much
// in principle 1.0 should be fine, but this allows for some error in measurements
#define ROT_SCALE_EXTRA_FAC 1.02

typedef stl_reader::StlMesh <float, unsigned int> ColStlMesh;

typedef struct {
  Vec3 center;
  double r;  // radius
} Sphere;


typedef struct {
  Vec3 center;
  double r;  // radius
  double e;  // extent
  Quaternion orientation;
} Cylinder;

// using IdxPair = std::pair<uint32_t, uint32_t>;
typedef std::pair<uint32_t, uint32_t> IdxPair;


// vertexes should be stored in positive orientation order (such that the cross product of any two successive points
//    is the normal), the first/last should not be duplicated, and they should be coplanar.
// these are not enforced, but the test will not work if they are not.
typedef struct ConvexPolygon {
  std::vector<Vec3> vertexes;
} ConvexPolygon;

// convexity and vailidity not enforced
typedef struct ConvexPolyhedron {
  std::vector<Vec3>    vertexes;
  std::vector<IdxPair> edges;  // indexes to vertexes
  std::vector<Vec3>    normals;
} ConvexPolyhedron;

typedef boost::variant<Vec3, LineSegment, Prism, Sphere, Cylinder, ConvexPolyhedron> Intersectable;

bool intersect(Vec3 x, Prism y);
bool intersect(LineSegment x, Sphere y);
bool intersect(Sphere x, Sphere y);
bool intersect(Sphere x, Cylinder y);
bool intersect(Cylinder y, Sphere x);

// static
bool intersect(Prism x, Vec3 y);
bool intersect(Prism x, LineSegment y);
bool intersect(Prism x, Prism y);
bool intersect(Prism x, Sphere y);
bool intersect(Prism x, Cylinder y);
bool intersect(Prism p, ConvexPolyhedron poly);
bool intersect(Prism x, Intersectable y); // dispatched


// moving intersections
// `disp` is vector delta for Prism origin
bool intersect(Prism x, Vec3 y,        Vec3 disp);
bool intersect(Prism x, LineSegment y, Vec3 disp);
bool intersect(Prism x, Prism y,       Vec3 disp);
bool intersect(Prism x, Sphere y,      Vec3 disp);
bool intersect(Prism x, Cylinder y,    Vec3 disp);  // needs optimizing
bool intersect(Prism p, ConvexPolyhedron poly,Vec3 disp);

// rotation
// Prism orientation goes from original to rotation * original, and positions are rotated by rotation about `about`
bool intersect(Prism x, Vec3 y,        Quaternion rotation, Vec3 about);
bool intersect(Prism x, LineSegment y, Quaternion rotation, Vec3 about);  // neds optimizing
bool intersect(Prism x, Prism y,       Quaternion rotation, Vec3 about);
bool intersect(Prism x, Sphere y,      Quaternion rotation, Vec3 about);
bool intersect(Prism x, Cylinder y,    Quaternion rotation, Vec3 about);
bool intersect(Prism p, ConvexPolyhedron poly, Quaternion rotation, Vec3 about);



// dispatch
bool intersect(Prism x, Intersectable y);
bool intersect(Prism x, Intersectable y, Vec3 disp);
bool intersect(Prism x, Intersectable y, Quaternion rotation, Vec3 about);
// these will return true if _any_ collisions happen
bool intersect(Prism x, std::vector<Intersectable> ys);
bool intersect(Prism x, std::vector<Intersectable> ys, Vec3 disp);
bool intersect(Prism x, std::vector<Intersectable> ys, Quaternion rotation, Vec3 about);

// these are useful for initial checks (since we are assuming the vast majority of objects will not collide)
// not bounding_sphereallest bounding spheres (except in the case of the sphere), but rather are fast to compute
Sphere bounding_sphere(Vec3 x);  // for completeness, has radius of 1e-9
Sphere bounding_sphere(LineSegment x);  // a pretty terrible bound 
Sphere bounding_sphere(Prism x);  // guaranteed smallest
Sphere bounding_sphere(Sphere x);  // trivial
Sphere bounding_sphere(Cylinder x);  // guaranteed smallest
Sphere bounding_sphere(const std::vector<Vec3>& vertexes);  // estimate using centroid and max distance from centroid
Sphere bounding_sphere(Intersectable x);  // dispatch


Vec3 centroid(const std::vector<Vec3>& vertexes);


// coordinate transforms
void c2s(double x, double y, double z, double* rho, double* theta, double* phi);
void s2c(double rho, double theta, double phi, double* x, double* y, double* z);


// useful math
template<typename T, size_t N>
std::pair<T, T> extrema(const array<T, N>& a) {
  static_assert(N >= 1, "Zero-length array has no extrema.");
  T min = a[0], max = a[0];
  for (size_t i = 1; i < N; i++) {
    if      (a[i] < min) min = a[i];
    else if (a[i] > max) max = a[i];
  }
  return std::make_pair(min, max);
}

template<typename T>
std::pair<T, T> extrema(const std::vector<T>& v) {
  T min = v[0], max = v[0];
  for (size_t i = 1; i < v.size(); i++) {
    if      (v[i] < min) min = v[i];
    else if (v[i] > max) max = v[i];
  }
  return std::make_pair(min, max);
}


std::tuple<Vec3, double> furthest(Vec3 from, const vector<Vec3>& to);


template<size_t N>
std::tuple<Vec3, double> furthest(Vec3 from, const array<Vec3, N>& to) {
  double max_dist = -1;
  Vec3   furthest;
  for (size_t i = 0; i < N; i++) {
    double d = norm2(to[i] - from);
    if (d > max_dist) {
      max_dist = d;
      furthest = to[i];
    }
  }
  return make_tuple(furthest, sqrt(max_dist));
}


// Definitions for making geometry functions polymorphic

typedef boost::variant<Vec3, LineSegment, Quaternion, Prism, Sphere, Cylinder, ConvexPolyhedron> GeometryObject;


boost::optional<Intersectable> geometry_to_intersectable(GeometryObject g);


namespace GeomTypes {
  enum GeomType {
    Vec3,
    LineSegment,
    Quaternion,
    Prism,
    Sphere,
    Cylinder,
    ConvexPolyhedron,
    String, // not geometry type but used for parsing
    Scalar, // not geometry type but used for parsing

    Invalid // invalid geometry
  };

  static const std::unordered_map<std::string, GeomType> GEOM_MAP ({
    {"Vec3",        Vec3},
    {"LineSegment", LineSegment},
    {"Quaternion",  Quaternion},
    {"Prism",       Prism},
    {"Sphere",      Sphere},
    {"Cylinder",    Cylinder},
    {"String",    String},
    {"ConvexPolyhedron",    ConvexPolyhedron},
    {"vec3",        Vec3},
    {"linesegment", LineSegment},
    {"quaternion",  Quaternion},
    {"prism",       Prism},
    {"sphere",      Sphere},
    {"cylinder",    Cylinder},
    {"string",    String},
    {"convexpolyhedron",    ConvexPolyhedron}
  });
}

#endif
