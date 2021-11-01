#include "vec3.hpp"


Vec3::Vec3() : x(0), y(0), z(0) {}


Vec3::Vec3(double _x, double _y, double _z)
: x(_x), y(_y), z(_z) {}


Vec3 Vec3::zero() {
  return Vec3(0, 0, 0);
}

Vec3 Vec3::basis_x() {
  return Vec3(1, 0, 0);
}

Vec3 Vec3::basis_y() {
  return Vec3(0, 1, 0);
}

Vec3 Vec3::basis_z() {
  return Vec3(0, 0, 1);
}


// Operators


bool operator==(const Vec3& p1, const Vec3& p2) {
  return p1.x == p2.x && p1.y == p2.y && p1.z == p2.z;
}


Vec3 operator+(const Vec3& p1, const Vec3& p2) {
  Vec3 ret = {
      p1.x + p2.x,
      p1.y + p2.y,
      p1.z + p2.z,
  };
  return ret;
}


Vec3 operator-(const Vec3& p1, const Vec3& p2) {
  Vec3 ret = {
      p1.x - p2.x,
      p1.y - p2.y,
      p1.z - p2.z,
  };
  return ret;
}


Vec3 operator-(const Vec3& p) {
  return (Vec3) {
    -p.x,
    -p.y,
    -p.z
  };
}

// dot product
double operator*(const Vec3& p1, const Vec3& p2) {
  return (p1.x * p2.x) + (p1.y * p2.y) + (p1.z * p2.z);
}


Vec3 operator*(const double d, const Vec3& p) {
  Vec3 ret = {
      d * p.x,
      d * p.y,
      d * p.z
  };
  return ret;
}


Vec3 operator*(const Vec3& p, const double d) {
  return d * p;
}


Vec3 operator/(const Vec3& p, const double d) {
  return (1/d) * p;
}


double dot(const Vec3& p1, const Vec3& p2) {
  return p1 * p2;
}


Vec3 cross(const Vec3 p1, const Vec3 p2) {
  Vec3 ret = {
      (p1.y * p2.z) - (p1.z * p2.y),
      (p1.z * p2.x) - (p1.x * p2.z),
      (p1.x * p2.y) - (p1.y * p2.x)
  };
  return ret;
}


double norm(const Vec3 p) {
  return sqrt((p.x * p.x) + (p.y * p.y) + (p.z * p.z));
}


double norm2(const Vec3 p) {
  return (p.x*p.x) + (p.y*p.y) + (p.z*p.z);
}

double angle(const Vec3 p1, const Vec3 p2){
  return acos(dot(p1,p2)/norm(p1)/norm(p2));
}

Vec3 normalized(const Vec3 p) {
  return p / norm(p);
}


bool approxeq(const Vec3& p1, const Vec3& p2) {
  return
    (fabs(p1.x - p2.x) < APPROX) &&
    (fabs(p1.y - p2.y) < APPROX) &&
    (fabs(p1.z - p2.z) < APPROX);
}


std::ostream& operator<<(std::ostream &os, const Vec3& p) {
  return os << "Vec3{" << p.x << ", " << p.y << ", " << p.z << "}";
}
