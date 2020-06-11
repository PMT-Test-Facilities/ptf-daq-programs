#ifndef __PATH_GEN__
#define __PATH_GEN__

#include <boost/variant.hpp>
#include <boost/optional.hpp>
#include <utility>
#include <array>
#include <vector>

#include "geom.hpp"
#include "serialization.hpp"


namespace PathGeneration {


using boost::variant;
using boost::optional;


// Represents a dimension along which a gantry can travel
enum Dimension {
  X,
  Y,
  Z,
  Theta,
  Phi
};


// gantry angle, in radians
typedef struct Angle {
  double theta;
  double phi;
} Angle;


// Angle operator-(const Angle& l, const Angle& r);


// Represents a point in the scan (position for the gantry) 
typedef struct Point {
  Vec3  position;
  Angle angle;
} Point;


bool operator ==(const Point& l, const Point& r);
bool operator !=(const Point& l, const Point& r);


// Represents a segment that a gantry travels along
typedef struct Segment {
  Point start;
  Point end;
} ScanSegment;


enum WhichGantry {
  Gantry0,
  Gantry1//,
  // Both
};


enum ErrorType {
  NoError,
  InvalidDestination,
  InvalidOrigin,
  InvalidGeometry,
  NoValidPaths,
  InvalidScanParameters,
  ScanTypeNotImplemented,
  GenerationError // internal error, debug only
};


typedef struct GeneralParams {
  uint32_t    millis;
  WhichGantry which_gantry;
} GeneralParams;


typedef struct RectangularParams {
  Vec3  prism_start;
  Vec3  prism_delta;
  Vec3  prism_incr;
  Angle angle;
} RectangularParams;


typedef struct CylindricalParams {
  Vec3   center;
  double radius;
  double height;
  Vec3   incr;
  Angle  angle;
} CylindricalParams;


typedef variant<RectangularParams, CylindricalParams> SpecificParams;
// using SpecificParams = variant<RectangularParams, CylindricalParams>;


typedef struct ScanParams {
  GeneralParams  general_params;
  SpecificParams specific_params;
} ScanParams;


typedef struct MovePoint {
  Point gantry0;
  Point gantry1;

  template<typename T>
  optional<double> operator[](T i) {
    // static_assert(std::is_integral<T>::value(), "Cannot index with a nonintegral type.");
    switch(i) {
      case 0:
        return gantry0.position.x;
      case 1:
        return gantry0.position.y;
      case 2:
        return gantry0.position.z;
      case 3:
        return gantry0.angle.theta;
      case 4:
        return gantry0.angle.phi;
      case 5:
        return gantry1.position.x;
      case 6:
        return gantry1.position.y;
      case 7:
        return gantry1.position.z;
      case 8:
        return gantry1.angle.theta;
      case 9:
        return gantry1.angle.phi;
      default:
        return boost::none;
    }
  }
} MovePoint;

// in a valid `vector<MovePoint>`, only one gantry moves at a time
bool is_valid(const vector<MovePoint>& move_path);


typedef vector<MovePoint> MovePath;


// converts a point from feMove into a MovePoint. NOTE: converts degrees to radians, which are used internally
template<typename T>
MovePoint move_point_from_array(const array<T, 10>& values) {
  MovePoint ret = {
    {{values[0], values[1], values[2]}, {values[3] * DEG2RAD, values[4] * DEG2RAD}},
    {{values[5], values[6], values[7]}, {values[8] * DEG2RAD, values[9] * DEG2RAD}}
  };
  return ret;
}


// Converts back again, changing internal radians to degrees
template<typename T>
array<T, 10> array_from_move_point(const MovePoint& mp) {
  array<T, 10> ret = {{
    mp.gantry0.position.x,
    mp.gantry0.position.y,
    mp.gantry0.position.z,
    mp.gantry0.angle.theta * RAD2DEG,
    mp.gantry0.angle.phi * RAD2DEG,
    mp.gantry1.position.x,
    mp.gantry1.position.y,
    mp.gantry1.position.z,
    mp.gantry1.angle.theta * RAD2DEG,
    mp.gantry1.angle.phi * RAD2DEG
  }};
  return ret;
}


/* These are the normal public interface for getting paths */
variant<MovePath, ErrorType> single_move(
  const MovePoint& from,
  const MovePoint& to,
  const vector<Intersectable>& static_geometry
);


variant<vector<MovePath>, ErrorType> scan_path(
  const ScanParams params,
  const vector<Intersectable>& static_geometry
);


template<typename T>
vector<T> flatten(vector<vector<T>> ts) {
  size_t size = 0;
  for (auto it = ts.begin(); it != ts.end(); ++it) {
    size += it->size();
  }
  vector<T> ret;
  ret.reserve(size);
  for (auto it = ts.begin(); it != ts.end(); ++it) {
    ret.insert(ret.end(), it->begin(), it->end());
  }
  return ret;
}


/* other potentially useful functions */
string error_message(ErrorType e);

bool is_destination_valid(
  const Point& gantry0,
  const Point& gantry1,
  const vector<Intersectable>& static_geometry
);


string dim_name(Dimension d);
string dim_name(size_t i);


array<Prism, 3> point_to_prisms(const Point& p, bool gantry1);

// typedef size_t ODBCollision;

// enum OtherCollision {
//   GantryGantry,
//   InvalidDestination,
// };

// struct NoCollision {};

// typedef variant<int, WhichGantry, NoCollision> CollisionInfo;


// not part of public interface
namespace Private {
  namespace Rect{};
  namespace Cyl {};
  // namespace VarNorm {};
  // namespace ConstNorm {};
}


MovePoint from_pair(const pair<Point, Point>& p, const WhichGantry which_gantry);


} // end namespace PathGeneration


// this could potentially improve the performance of the path generation code dramatically
// due to less heap allocations and less cache invalidation, I just didn't have time to try it out.
// See my (Mia) report for more info
// N is the maximum number of points
template<typename T, size_t N>
class fixed_vec {
private:
  array<T, N> storage;
  size_t      idx; // index of the next free spot

public:
  fixed_vec() : idx(0) {}
  ~fixed_vec() = default;

  // safe functions

  bool push_checked(const T& t) {
    if (__builtin_expect(idx == N+1, 0)) {
      return false;
    }
    storage[idx] = std::copy(t);
    ++idx;    
  }

  bool push_checked(const T&& t) {
    if (__builtin_expect(idx == N+1, 0)) {
      return false;
    }
    storage[idx] = std::move(t);
    ++idx;   
    return true; 
  }

  bool pop_checked() {
    if (__builtin_expect(idx == 0, 0)) {return false;}
    --idx;
    return true;
  }

  template<typename I>
  optional<T> get_checked(I i) const {
    static_assert(std::is_integral<I>::value, "Cannot index with a non-integral type.");
    if (i >= idx || i < 0) {
      return none;
    }
    return storage[i];
  }

  template<typename I>
  optional<T*> get_ptr_checked(I i) const {
    static_assert(std::is_integral<I>::value, "Cannot index with a non-integral type.");
    if (i >= idx || i < 0) {
      return none;
    }
    return storage.data() + i;
  }

  // unsafe functions (no bounds checking)

  void push_unchecked(const T& t) {
    storage[idx] = std::copy(t);
    ++idx;
  }

  void push_unchecked(const T&& t) {
    storage[idx] = std::move(t);
    ++idx;
  }

  void pop_unchecked() {
    --idx;
  }

  template<typename I>
  T get_unchecked(I i) const {
    static_assert(std::is_integral<I>::value, "Cannot index with a non-integral type.");
    return storage[i];
  }

  template<typename I>
  T* get_ptr_unchecked(I i) const {
    static_assert(std::is_integral<I>::value, "Cannot index with a non-integral type.");
    return storage.data() + i;
  }
};


#endif // __PATH_GEN__
