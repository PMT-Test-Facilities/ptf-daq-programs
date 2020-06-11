#ifndef __QUATERNION
#define __QUATERNION


#include "vec3.hpp"


typedef struct Quaternion {
  double w;
  double x;
  double y;
  double z;

  Quaternion();
  Quaternion(double _w, double _x, double _y, double _z);

  void normalize();
  Quaternion normalized() const;

  static Quaternion from_vec3(const Vec3& vec);
  static Quaternion from_axis_angle(const Vec3& axis, double angle);
  static Quaternion from_spherical_angle(double theta, double phi);
  static Quaternion from_azimuthal(double theta);

  static Quaternion identity();
  static Quaternion basis_real();
  static Quaternion basis_i();
  static Quaternion basis_j();
  static Quaternion basis_k();
} Quaternion;


Quaternion operator+(const Quaternion& q1, const Quaternion& q2);
Quaternion operator-(const Quaternion& q1, const Quaternion& q2);
Quaternion operator*(const Quaternion& q1, const Quaternion& q2);  // Hamilton product
Quaternion operator/(const Quaternion& q1, const Quaternion& q2);  // mult. inv. resp. Hamilton product

Quaternion operator*(const Quaternion& q, const double lambda);
Quaternion operator*(const double lambda, const Quaternion& q);


bool       is_versor(const Quaternion& q);
double     scalar_part(const Quaternion& q);
Vec3       vector_part(const Quaternion& q);
Vec3       axis(const Quaternion& q);
double     angle(const Quaternion& q);  // radians
double     norm(const Quaternion& q);
double     norm(const Quaternion* q);
Quaternion hamilton_product(const Quaternion& q1, const Quaternion& q2);
double     dot(const Quaternion& q1, const Quaternion& q2);
// Quaternion pow(const Quaternion& q, int p);
// Quaternion pow(const Quaternion& q, double p, size_t n = 10);
Quaternion inverse(const Quaternion& q);
Quaternion conjugate(const Quaternion& q);
Quaternion slerp(Quaternion q1, Quaternion q2, double t);


#endif
