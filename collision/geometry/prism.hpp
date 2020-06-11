#ifndef __PRISM
#define __PRISM

#include <array>

using namespace std;

#include "vec3.hpp"
#include "quaternion.hpp"
#include "linesegment.hpp"


typedef struct Prism {
  Prism();
  Prism(Vec3 _center, double _ex, double _ey, double _ez, Quaternion orientation);
  Prism(Vec3 _center, double _ex, double _ey, double _ez, double theta, double phi);

  Vec3 center;
  double ex;  // extent in x
  double ey;  // extent in y
  double ez;  // extent in z
  Quaternion orientation;

  array<Vec3, 8>         vertexes() const;
  array<LineSegment, 12> edges() const;
  array<Vec3, 6>         normals() const;
} Prism;


#endif
