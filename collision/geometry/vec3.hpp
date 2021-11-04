#ifndef __VEC3
#define __VEC3

#include "common.hpp"


// represents positions as well (for convinience, even though not all operations make sense)
typedef struct Vec3 {
  Vec3();
  Vec3(double _x, double _y, double _z);

  double x;
  double y;
  double z;

  static Vec3 zero();
  static Vec3 basis_x();
  static Vec3 basis_y();
  static Vec3 basis_z();

  template <typename T>
  static Vec3 basis(T i) {
    static_assert(std::is_integral<T>::value, "Cannot index with a nonintegral type.");
    switch (i) {
      case 0:
        return basis_x();
      case 1:
        return basis_y();
      case 2:
        return basis_z();
      default:
        throw "Invalid index.";
    }
  }

  template <typename T>
  static Vec3 ortho(T i, T j) {
    static_assert(std::is_integral<T>::value, "Cannot index with a nonintegral type.");
    if (i == j) throw "Multiple cases.";
    switch (i) {
      case 0:
        switch (j) {
          case 1:
            return basis_z();
          case 2:
            return basis_y();
          default:
            throw "Invalid index.";
        }
      case 1:
        switch (j) {
          case 0:
            return basis_z();
          case 1:
            return basis_x();
          default:
            throw "Invalid index.";
        }
      case 2:
        switch (j) {
          case 0:
            return basis_y();
          case 1:
            return basis_x();
          default:
            throw "Invalid index.";
        }
      default:
        throw "Invalid index.";
    }
  }

  template <typename T>
  double operator[](T i) const {
    static_assert(std::is_integral<T>::value, "Cannot index with a nonintegral type.");
    switch (i) {
      case 0:
        return this->x;
      case 1:
        return this->y;
      case 2:
        return this->z;
      default:
        throw "Invalid index.";
    }
  }
} Vec3;


bool operator==(const Vec3& p1, const Vec3& p2);
Vec3 operator+(const Vec3& p1, const Vec3& p2);
Vec3 operator-(const Vec3& p1, const Vec3& p2);

double operator*(const Vec3& p1, const Vec3& p2);
// both dot and cross are defined because '*' for Vec3 is the dot product,
//   while for Quaternion it's the Hamilton product. This just makes it a
//   little clearer what's happening.
double dot(const Vec3& p1, const Vec3& p2);
Vec3   cross(const Vec3 p1, const Vec3 p2);
double   angle(const Vec3 p1, const Vec3 p2);
Vec3 scale(const Vec3 p1, const Vec3 scale);

Vec3 operator-(const Vec3& p);
Vec3 operator*(const double d, const Vec3& p);
Vec3 operator*(const Vec3& p, const double d);
Vec3 operator/(const Vec3& p, const double d);

double norm(const Vec3 p);
double norm2(const Vec3 p);  // norm squared
Vec3   normalized(const Vec3 p);

bool approxeq(const Vec3& p1, const Vec3& p2);

std::ostream &operator<<(std::ostream& os, const Vec3& p);


#endif
