#include "prism.hpp"
#include "linesegment.hpp"
#include "rotations.hpp"
#include <vtkHexahedron.h>

#include "debug.hpp"

Prism::Prism() : center(Vec3()), ex(0), ey(0), ez(0), orientation(Quaternion()) {}

Prism::Prism(Vec3 _center, double _ex, double _ey, double _ez, Quaternion _orientation)
  : center(_center), ex(_ex), ey(_ey), ez(_ez), orientation(_orientation) {
#ifdef DEBUG
  
  // doing additional checks for debug mode
  bool was_neg = _ex < 0 || _ey < 0 || _ez < 0;
  bool was_nor = !is_versor(_orientation);

  if (was_neg || was_nor) {
    DEBUG_ENTER(__PRETTY_FUNCTION__);
    if (was_neg) {
      DEBUG_COUT(C_BR_YELLOW << "WARNING: Extent below zero." << C_RESET);
    }
    if (was_nor) {
      DEBUG_COUT(C_BR_YELLOW << "WARNING: Quaternion that is not normalized (norm is " << norm(_orientation) << ") used for orientation." << C_RESET);
    }
    DEBUG_LEAVE;
  }
  
#endif
}


// function has to be duplicated instead of using the above in the initializer list due to a bug in this verison of gcc
Prism::Prism(Vec3 _center, double _ex, double _ey, double _ez, double theta, double phi)
  : center(_center), ex(_ex), ey(_ey), ez(_ez), orientation(Quaternion::from_spherical_angle(theta, phi)) {
#ifdef DEBUG
  
  // doing additional checks for debug mode
  bool was_neg = _ex < 0 || _ey < 0 || _ez < 0;
  bool was_nor = !is_versor(orientation);

  if (was_neg || was_nor) {
    DEBUG_ENTER(__PRETTY_FUNCTION__);
    if (was_neg) {
      DEBUG_COUT(C_BR_YELLOW << "WARNING: Extent below zero." << C_RESET);
    }
    if (was_nor) {
      DEBUG_COUT(C_BR_YELLOW << "WARNING: Quaternion that is not normalized (norm is " << norm(orientation) << ") used for orientation." << C_RESET);
    }
    DEBUG_LEAVE;
  }
  
#endif
}

// was closure...
inline Vec3 _add_offset(double x, double y, double z, Vec3 center, Quaternion orientation) {
  return rotate_point(
    center + Vec3(x, y, z),
    center,
    orientation
  );
}

array<Vec3, 8> Prism::vertexes() const {
  array<Vec3, 8> points;

  points[0] = _add_offset( this->ex, this->ey, this->ez, this->center, this->orientation);
  points[1] = _add_offset(-this->ex, this->ey, this->ez, this->center, this->orientation);
  points[2] = _add_offset(-this->ex,-this->ey, this->ez, this->center, this->orientation);
  points[3] = _add_offset( this->ex,-this->ey, this->ez, this->center, this->orientation);
  points[4] = _add_offset( this->ex, this->ey,-this->ez, this->center, this->orientation);
  points[5] = _add_offset(-this->ex, this->ey,-this->ez, this->center, this->orientation);
  points[6] = _add_offset(-this->ex,-this->ey,-this->ez, this->center, this->orientation);
  points[7] = _add_offset( this->ex,-this->ey,-this->ez, this->center, this->orientation);

  return points;
}


// Finds all 12 line segments of a prism. `segments` must be allocated.
// Depends on the order that points are created in `prismPoints`.
array<LineSegment, 12> Prism::edges() const {
  array<LineSegment, 12> segments;

  auto ps = this->vertexes();

  for (int i = 0; i < 4; i++) {
    segments[3*i]   = {ps[i],   ps[i+4]};
    segments[3*i+1] = {ps[i],   ps[(i+1)%4]};
    segments[3*i+2] = {ps[i+4], ps[((i+1)%4)+4]};
  }

  return segments;
}


// finds the 6 normals of a prism.
array<Vec3, 6> Prism::normals() const {
  array<Vec3, 6> norms;

  norms[0] = rotate_point( Vec3::basis_x(), Vec3::zero(), this->orientation);
  norms[1] = rotate_point( Vec3::basis_y(), Vec3::zero(), this->orientation);
  norms[2] = rotate_point( Vec3::basis_z(), Vec3::zero(), this->orientation);
  norms[3] = rotate_point(-Vec3::basis_x(), Vec3::zero(), this->orientation);
  norms[4] = rotate_point(-Vec3::basis_y(), Vec3::zero(), this->orientation);
  norms[5] = rotate_point(-Vec3::basis_z(), Vec3::zero(), this->orientation);

  return norms;
}
