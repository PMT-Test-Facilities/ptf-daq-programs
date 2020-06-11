#include "quaternion.hpp"


// constructors

Quaternion::Quaternion() : w(0), x(0), y(0), z(0) {}

Quaternion::Quaternion(double _w, double _x, double _y, double _z)
:  w(_w), x(_x), y(_y), z(_z) {}

void Quaternion::normalize() {
  double inorm = 1/norm(this);
  this->w *= inorm;
  this->x *= inorm;
  this->y *= inorm;
  this->z *= inorm;
}

Quaternion Quaternion::normalized() const {
  return (1/norm(this)) * *this;
}

Quaternion Quaternion::identity() {
  return Quaternion(1, 0, 0, 0);
}

Quaternion Quaternion::from_vec3(const Vec3& vec) {
  return Quaternion(0, vec.x, vec.y, vec.z);
}

Quaternion Quaternion::from_axis_angle(const Vec3& axis, double angle) {
  double sin2 = sin(angle/2);
  return Quaternion(cos(angle/2), sin2*axis.x, sin2*axis.y, sin2*axis.z);
}

Quaternion Quaternion::from_spherical_angle(double theta, double phi) {
  return Quaternion(
     cos(phi/2)*cos(theta/2),
    -sin(phi/2)*sin(theta/2),
     sin(phi/2)*cos(theta/2),
     cos(phi/2)*sin(theta/2)
  );
}

Quaternion Quaternion::from_azimuthal(double theta) {
  return Quaternion(
    cos(theta/2),
    0.0,
    0.0,
    sin(theta/2)
  );
}


// constexpr Quaternion Quaternion::identity()  {
// }

Quaternion Quaternion::basis_real() {
  return Quaternion(1, 0, 0, 0);
}

Quaternion Quaternion::basis_i() {
  return Quaternion(0, 1, 0, 0);
}

Quaternion Quaternion::basis_j() {
  return Quaternion(0, 0, 1, 0);
}

Quaternion Quaternion::basis_k() {
  return Quaternion(0, 0, 0, 1);
}


// operators


Quaternion operator+(const Quaternion& q1, const Quaternion& q2) {
  return Quaternion(q1.w + q2.w, q1.x + q2.x, q1.y + q2.y, q1.z + q2.z);
}


Quaternion operator-(const Quaternion& q1, const Quaternion& q2) {
  return Quaternion(q1.w - q2.w, q1.x - q2.x, q1.y - q2.y, q1.z - q2.z);
}


Quaternion operator*(const Quaternion& q1, const Quaternion& q2) {
  return Quaternion(
    (q1.w * q2.w) - (q1.x * q2.x) - (q1.y * q2.y) - (q1.z * q2.z),
    (q1.w * q2.x) + (q1.x * q2.w) + (q1.y * q2.z) - (q1.z * q2.y),
    (q1.w * q2.y) - (q1.x * q2.z) + (q1.y * q2.w) + (q1.z * q2.x),
    (q1.w * q2.z) + (q1.x * q2.y) - (q1.y * q2.x) + (q1.z * q2.w)
  );
}


Quaternion operator/(const Quaternion& q1, const Quaternion& q2) {
  return q1 * inverse(q2);
}

Quaternion operator*(const Quaternion& q, const double lambda) {
  return Quaternion(lambda*q.w, lambda*q.x, lambda*q.y, lambda*q.z);
}

Quaternion operator*(const double lambda, const Quaternion& q) {
  return Quaternion(lambda*q.w, lambda*q.x, lambda*q.y, lambda*q.z);
}


// funcs


bool is_versor(const Quaternion& q) {
  return abs(norm(q) - 1) < 1e-6;
}


double scalar_part(const Quaternion& q) {
  return q.w;
}


Vec3 vector_part(const Quaternion& q) {
  return Vec3(q.x, q.y, q.z);
}


Vec3 axis(const Quaternion& q) {
  return Vec3(
    q.x / sqrt(1 - q.w*q.w),
    q.y / sqrt(1 - q.w*q.w),
    q.z / sqrt(1 - q.w*q.w)
  );
}


double angle(const Quaternion& q) {
  return 2 * acos(q.w);
}


Quaternion hamilton_product(const Quaternion& q1, const Quaternion& q2) {
  return q1 * q2;
}


double dot(const Quaternion& q1, const Quaternion& q2) {
  return (q1.w * q2.w) + (q1.x * q2.x) + (q1.y * q2.y) + (q1.z * q2.z);
}


double norm(const Quaternion& q) {
  return sqrt((q.w*q.w) + (q.x*q.x) + (q.y*q.y) + (q.z*q.z));
}


double norm(const Quaternion* q) {
  if (q == NULL) return nan("");
  return sqrt((q->w*q->w) + (q->x*q->x) + (q->y*q->y) + (q->z*q->z));
}


Quaternion inverse(const Quaternion& q) {
  double _norm = norm(q);
  return (1/(_norm*_norm)) * Quaternion(q.w, -q.x, -q.y, -q.z);
}


Quaternion conjugate(const Quaternion& q) {
  return -0.5 * (
     q + 
    (Quaternion::basis_i() * q * Quaternion::basis_i()) +
    (Quaternion::basis_j() * q * Quaternion::basis_j()) +
    (Quaternion::basis_k() * q * Quaternion::basis_k())
  );
}

#define DOT_THRESHOLD 0.9995

Quaternion slerp(Quaternion q1, Quaternion q2, double t) {
  // taken from Wikipedia article on slerp

  double d = dot(q1, q2);

  if (d < 0.0) {
    q1 = -1 * q1;
    d  = -d;
  }

  if (d > DOT_THRESHOLD) {
      // If the inputs are too close for comfort, linearly interpolate
      // and normalize the result.

      Quaternion result = q1 + t*(q2 - q1);
      return result.normalized();
  }

  // Since dot is in range [0, DOT_THRESHOLD], acos is safe
  double theta_0     = acos(d);      // theta_0 = angle between input vectors
  double theta       = theta_0*t;    // theta = angle between v0 and result
  double sin_theta   = sin(theta);   // compute this value only once
  double sin_theta_0 = sin(theta_0); // compute this value only once

  double s1 = cos(theta) - d * sin_theta / sin_theta_0;  // == sin(theta_0 - theta) / sin(theta_0)
  double s2 = sin_theta / sin_theta_0;

  return (s1 * q1) + (s2 * q2);
}
