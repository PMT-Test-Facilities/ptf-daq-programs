#ifndef __SAT_H__
#define __SAT_H__

#include <utility>
#include <vector>
#include "vec3.hpp"
#include "linesegment.hpp"
#include "geom.hpp"

// this file contains multiple implementations of the method of separating axes

// if there are more than this many axes to check, we'll do a bounding sphere check first
// this speeds up *significantly* in the case where objects do not intersect
#define NUM_AXES_FOR_BOUNDS_CHECK 25
// if there are are more than this many axes to check, we'll do pairwise object <-> bounding sphere checks first
#define NUM_AXES_FOR_PAIRWISE 100
// if there are more than this many axes, we'll do a bounding cylinder test (for cases with linear displacement)
#define NUM_AXES_FOR_DISP_BOUNDS 25


#define NUM_NORMALS_FOR_CYLINDER 128


ConvexPolyhedron to_polyhedron(const ConvexPolygon& p);

Vec3 centroid(const ConvexPolygon& poly);
Vec3 centroid(const ConvexPolyhedron& poly);

void scale_around(ConvexPolygon& poly, const Vec3 about, const double factor);
void scale_around(ConvexPolyhedron& poly, const Vec3 about, const double factor);
// operate about centroid
void scale(ConvexPolygon& poly, const double factor);
void scale(ConvexPolyhedron& poly, const double factor);


ConvexPolyhedron polyhedron(const Prism p);
ConvexPolyhedron polyhedron(const Cylinder c);
ConvexPolyhedron polyhedron(const ColStlMesh mesh);

ConvexPolyhedron translate(const ConvexPolyhedron mesh,Vec3 disp);
ConvexPolyhedron rotate(const ConvexPolyhedron mesh,Quaternion orientation,Vec3 rotation_point=Vec3::zero());
ConvexPolyhedron scale(const ConvexPolyhedron mesh,Vec3 scale,Vec3 scale_point=Vec3::zero());


ConvexPolyhedron sweep(const ConvexPolygon& p,    const Vec3 disp);


ConvexPolyhedron _sweep(const Prism p, Vec3 disp);//TODO: removed, this is added for testing and should only be decalered in geom.cxx
ConvexPolyhedron _sweep2(const Prism p, Vec3 disp);//TODO: removed, this is added for testing and should only be decalered in geom.cxx
// ConvexPolyhedron sweep(const ConvexPolyhedron& p, const Vec3 disp);


// ConvexPolyhedron sweep(const ConvexPolygon& p,    Quaternion rotation, Vec3 about, size_t prec_factor = 1);
// ConvexPolyhedron sweep(const ConvexPolyhedron& p, Quaternion rotation, Vec3 about, size_t prec_factor = 1);


Vec3 normal(const ConvexPolygon& p);


// are the polygons coplanar?
bool coplanar(const ConvexPolygon& p1, const ConvexPolygon& p2, double tolerance = 1e-6);


double project(const Vec3& direction, const Vec3& to_project);
void   project(const Vec3& direction, const std::vector<Vec3>& to_project, std::vector<double>& dst);


bool separated(const std::vector<Vec3>& points1, const std::vector<Vec3>& points2, const Vec3& source, const Vec3& displacement);


bool intersect(const ConvexPolygon& poly1, const ConvexPolygon& poly2);
bool intersect(const ConvexPolyhedron& polyh1, const ConvexPolyhedron& polyh2);
bool intersect2(const ConvexPolyhedron& polyh1, const ConvexPolyhedron& polyh2);
bool intersect(const ConvexPolygon& polygon, const ConvexPolyhedron& polyhedron);
// alias to the latter
bool intersect(const ConvexPolyhedron& polyhedron, const ConvexPolygon& polygon);


bool intersect(const Sphere& s, const ConvexPolygon& p);
bool intersect(const Sphere& s, const ConvexPolyhedron& p);

// for these, the displacement is the motion of the poly{gon,hedron}.
bool intersect(const Sphere& s, const ConvexPolygon& p, const Vec3& disp);
bool intersect(const Sphere& s, const ConvexPolyhedron& p, const Vec3& disp);
// for these, the dispacement is the motion of the first argument
bool intersect(const ConvexPolygon& p1, const ConvexPolygon& p2, const Vec3& disp);
bool intersect(const ConvexPolygon& p1, const ConvexPolyhedron& p2, const Vec3& disp);
bool intersect(const ConvexPolyhedron& p1, const ConvexPolygon& p2, const Vec3& disp);
bool intersect(const ConvexPolyhedron& p1, const ConvexPolyhedron& p2, const Vec3& disp);


Sphere bounding_sphere(const ConvexPolyhedron& p);

// Cylinder bounding_cylinder(const ConvexPolygon& p, const Vec3 disp);
// Cylinder bounding_cylinder(const ConvexPolyhedron& p, const Vec3 disp);

#endif
