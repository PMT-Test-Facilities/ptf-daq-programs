#include "pathgen.hpp"
#include "pathgen_internal.hpp"
#include "measurements.hpp"

#include "cyl.hpp"
#include "rect.hpp"

#include <ios>
#include <iomanip>

#ifdef DEBUG
#include "serialization.hpp"
namespace SD = Serialization;
#endif

#include "debug.hpp"

namespace Rect = PathGeneration::Private::Rect;
namespace Cyl  = PathGeneration::Private::Cyl;


/*
 * How we generate a path:
 * 
 * For a single destination position,
 *   - Check each gantry first with all axis orders
 *   - If an order is found, return vector<ScanPoints>
 * For a full scan,
 *   - Generate the desired points
 *   - Perform the above on each
 */


namespace PathGeneration {

// alternate version of public function
bool is_destination_valid(
  const Point& gantry0,
  const Point& gantry1,
  const vector<Intersectable>& static_geometry
);

bool is_move_valid(
  const MovePath moving,
  const WhichGantry is_moving,
  const vector<Intersectable>& static_geometry
);

bool check_any_collisions(
  const Point& gantry0,
  const Point& gantry1,
  const vector<Intersectable>& static_geometry,
  const vector<Intersectable>& dynamic_geometry
);

bool check_any_collisions(
  const Point& gantry0,
  const Point& gantry1,
  const vector<Intersectable>& static_geometry
);

// Most-used public functions


variant<MovePath, ErrorType> single_move(const MovePoint& from, const MovePoint& to, const vector<Intersectable>& static_geometry) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);
  DEBUG_COUT("Checking destination and source...");
  if (!is_destination_valid(to.gantry0, to.gantry1, static_geometry)) {
    return ErrorType::InvalidDestination;
  }
  else if (!is_destination_valid(from.gantry0, from.gantry1, static_geometry)) {
    return ErrorType::InvalidOrigin;
  }
  static const auto all_orders = DimensionOrder::all_orders();

  DEBUG_COUT("Attempting to move G0 first.");
  
  // try moving gantry 0 first
  for (size_t i = 0; i < all_orders.size(); i++) {
    auto order0 = all_orders[i];

    DEBUG_COUT("Attempting dim order " << C_BOLD << i << ": " << order0 << C_RESET);
    auto path0  = generate_move({from.gantry0, to.gantry0}, from.gantry1, Gantry0, order0);
    if (is_move_valid(path0, Gantry0, static_geometry)) {
      for (size_t j = 0; j < all_orders.size(); j++) {
        auto order1 = all_orders[j];
        auto path1  = generate_move({from.gantry1, to.gantry1}, to.gantry0, Gantry1, order1);
        // cerr << "," << j;
        if (is_move_valid(path1, Gantry1, static_geometry)) {
          path0.reserve(10);
          path0.insert(path0.end(), path1.begin(), path1.end());
          DEBUG_COUT(C_BR_GREEN << "Move found." << C_RESET);
          DEBUG_LEAVE;
          return path0;
        }
      }
      DEBUG_COUT("Initial move valid, but could not find second move.");
    } else {
      DEBUG_COUT("Initial move invalid.");
    }
  }

  DEBUG_COUT("Attempting to move G1 first.");

  // now try moving gantry 1 first
  for (size_t i = 0; i < all_orders.size(); i++) {
    auto order1 = all_orders[i];

    DEBUG_COUT("Attempting dim order " << C_BOLD << i << ": " << order1 << C_RESET);

    auto path1  = generate_move({from.gantry1, to.gantry1}, from.gantry0, Gantry1, order1);
    if (is_move_valid(path1, Gantry1, static_geometry)) {
      for (size_t j = 0; j < all_orders.size(); j++) {
        auto order0 = all_orders[j];
        auto path0  = generate_move({from.gantry0, to.gantry0}, to.gantry1, Gantry0, order0);
        if (is_move_valid(path0, Gantry0, static_geometry)) {
          path1.reserve(10);
          path1.insert(path1.end(), path0.begin(), path0.end());
          DEBUG_COUT(C_BR_GREEN << "Move found." << C_RESET);
          DEBUG_LEAVE;
          return path1;
        }
      }

      DEBUG_COUT("Initial move valid, but could not find second move.");
    } else {
      DEBUG_COUT("Initial move invalid.");
    }
  }

  DEBUG_COUT("Could not find a valid move order.");
  DEBUG_LEAVE;
  return ErrorType::NoValidPaths;
}


variant<vector<MovePath>, ErrorType> scan_path(ScanParams params, const vector<Intersectable>& static_geometry) {
  // Wish we had Rust tagged unions...
  if (has<RectangularParams>(params.specific_params))
    return Rect::gen_path(params.general_params, get<RectangularParams>(params.specific_params), static_geometry);
  
  else if (has<CylindricalParams>(params.specific_params))
    return Cyl::gen_path(params.general_params, get<CylindricalParams>(params.specific_params), static_geometry);

  else
    return ErrorType::ScanTypeNotImplemented;
}


bool is_destination_valid(
  const Point& gantry0,
  const Point& gantry1,
  const vector<Intersectable>& static_geometry
) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);
  if (fabs(gantry1.position.y - gantry0.position.y) < GANTRY_MIN_Y_SEPARATION) {//TODO: why was this condidtion reqiuire, maybe do coordinate tranform between the positions
    DEBUG_COUT("Y diff less than GANTRY_MIN_Y_SEPARATION.");
    DEBUG_LEAVE;
    return false;
  }
  else if (fabs(gantry1.position.y - gantry0.position.y) < GANTRY_MIN_Y_SEPARATION_FOR_X_MIN_CHECK
           && fabs(gantry0.position.x - gantry1.position.x) < GANTRY_MIN_X_SEPARATION) {
    DEBUG_COUT("X diff less than GANTRY_MIN_X_SEPARATION.");
    DEBUG_LEAVE;
    return false;
  }
  //DEBUG_COUT(gantry0.position.x<<" "<<gantry0.position.y<<" "<<gantry0.position.z);
  //DEBUG_COUT(gantry1.position.x<<" "<<gantry1.position.y<<" "<<gantry1.position.z);
  DEBUG_COUT("Distance constraints ok. Checking collisions.");
  if (check_any_collisions(gantry0, gantry1, static_geometry)) {
    DEBUG_COUT("Found collision.");
    DEBUG_LEAVE;
    return false;
  }
  DEBUG_COUT("Destination seems ok.");
  DEBUG_LEAVE;
  return true;
}


Angle eldiff(const Angle& a1, const Angle& a2) {
  return {a1.theta - a2.theta, a1.phi - a2.phi};
}


bool is_move_valid(
  const MovePath moving,
  const WhichGantry is_moving,
  const vector<Intersectable>& static_geometry
) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);
  DEBUG_COUT("Move contains " << moving.size() << " substeps and gantry " << (is_moving == Gantry0 ? "0" : "1") << " is moving.");
  for (size_t i = 1; i < moving.size(); i++) {
    DEBUG_COUT("Checking move segment " << (i-1) << "-" << i)
    auto pt = moving[i], prev = moving[i-1];
    const auto
      dp0 = pt.gantry0.position - prev.gantry0.position,
      dp1 = pt.gantry1.position - prev.gantry1.position;
    const auto
      da0 = eldiff(pt.gantry0.angle, prev.gantry0.angle),
      da1 = eldiff(pt.gantry1.angle, prev.gantry1.angle);
    
    if (norm2(dp0) > 0) {
      DEBUG_COUT("Found nonzero displacement for gantry 0: " << SD::serialize(dp0));
      if (   intersect(point_to_prisms(prev.gantry0, false)[0], static_geometry, dp0)
          || intersect(point_to_prisms(prev.gantry0, false)[1], static_geometry, dp0)
          || intersect(point_to_prisms(prev.gantry0, false)[2], static_geometry, dp0)
          || intersect(point_to_optical_box(pt.gantry1, true), static_geometry)
         ) {
        DEBUG_COUT("Found collision.");
        DEBUG_LEAVE;
        return false;
      }
    }
    else if (norm2(dp1) > 0) {
      DEBUG_COUT("Found nonzero displacement for gantry 1: " << SD::serialize(dp1));
      if (intersect(point_to_optical_box(pt.gantry0, false), static_geometry)
          || intersect(point_to_prisms(prev.gantry1, true)[0], static_geometry, dp1)
          || intersect(point_to_prisms(prev.gantry1, true)[1], static_geometry, dp1)
          || intersect(point_to_prisms(prev.gantry1, true)[2], static_geometry, dp1)) {
        DEBUG_COUT("Found collision.");
        DEBUG_LEAVE;
        return false;
      }
    }
    else if (da0.theta != 0 || da0.phi != 0) {
      DEBUG_COUT("Found nonzero rotation for gantry 0: theta=" << da0.theta << ", phi=" << da0.phi);
      if (intersect(point_to_optical_box(pt.gantry0, false), static_geometry, Quaternion::from_spherical_angle(da0.theta, da0.phi), pt.gantry0.position)
          || intersect(point_to_optical_box(pt.gantry1, false), static_geometry)) {
        DEBUG_COUT("Found collision.");
        DEBUG_LEAVE;
        return false;
      }
    }
    else if (da1.theta != 0 || da1.phi != 0) {
      DEBUG_COUT("Found nonzero rotation for gantry 1: theta=" << da1.theta << ", phi=" << da1.phi);
      if (intersect(point_to_optical_box(pt.gantry1, false), static_geometry, Quaternion::from_spherical_angle(da1.theta, da1.phi), pt.gantry1.position)
          || intersect(point_to_optical_box(pt.gantry0, false), static_geometry)) {
        DEBUG_COUT("Found collision.");
        DEBUG_LEAVE;
        return false;
      }
    } else {
      DEBUG_COUT("Found no movement. Should be covered by start/dest checks.");
      if (intersect(point_to_optical_box(pt.gantry1, false), static_geometry)
          || intersect(point_to_optical_box(pt.gantry0, false), static_geometry)) {
        DEBUG_COUT("Found collision.");
        DEBUG_LEAVE;
        return false;
      }
    }
  }

  DEBUG_COUT("Found no collision.");
  DEBUG_LEAVE;
  return true;
}


// Utils


bool operator ==(const Point& l, const Point& r) {
  return
    (l.position.x == r.position.x) &&
    (l.position.y == r.position.y) &&
    (l.position.z == r.position.z) &&
    (l.angle.theta == r.angle.theta) &&
    (l.angle.phi   == r.angle.phi);
}


bool operator !=(const Point& l, const Point& r) {
  return !(l == r);
}


// was closure
int _idx_of(Dimension dim) {
  switch(dim) {
    case X: return 0;
    case Y: return 1;
    case Z: return 2;
    case Theta: return 3;
    case Phi: return 4;
    default: throw new runtime_error("Invalid dimension.");
  }
}


optional<DimensionOrder> DimensionOrder::build(Dimension first, Dimension second, Dimension third, Dimension fourth, Dimension fifth) {
  bool seen_dim[5] = {false};
  Dimension dims[5] = {first, second, third, fourth, fifth};

  int i;
  //for (auto dim : dims) {
  for (uint32_t it = 0; it < 5; it++) {
    auto dim = dims[it];
    i = _idx_of(dim);
    if (seen_dim[i]) {
      return boost::none;
    } else {
      seen_dim[i] = true;
    }
  }

  // for (bool seen : seen_dim) {
  for (uint32_t i = 0; i < 5; i++) {
    if (!seen_dim[i]) {
      return boost::none;
    }
  }

  DimensionOrder d = {first, second, third, fourth, fifth};
  return d;
}


Dimension _dim_of(int i) {
  switch(i) {
    case 0: return X;
    case 1: return Y;
    case 2: return Z;
    case 3: return Theta;
    case 4: return Phi;
    default: throw new runtime_error("Invalid dimension id.");
  }
}


vector<DimensionOrder> DimensionOrder::all_orders() {
  vector<DimensionOrder> ret;
  ret.reserve(120); // 5!

// this is gross, I know
  for (int i = 0; i < 5; i++) {

    for (int j = 0; j < 5; j++) {
      if (j == i) continue;

      for (int k = 0; k < 5; k++) {
        if (k == j || k == i) continue;

        for (int l = 0; l < 5; l++) {
          if (l == k || l == j || l == i) continue;
          
          for (int m = 0; m < 5; m++) {
            if (m == l || m == k || m == j || m == i) continue;

            DimensionOrder d = {_dim_of(i), _dim_of(j), _dim_of(k), _dim_of(l), _dim_of(m)};
            ret.push_back(d);
          }
        }
      }
    }
  }

  return ret;
}


string error_message(ErrorType e) {
  switch (e) {
    case NoError:
      return "";
    case InvalidDestination:
      return "Invalid destination.";
    case InvalidOrigin:
      return "Invalid origin. feMove state must be incorrect.";
    case InvalidGeometry:
      return "Geometry is invalid.";
    case NoValidPaths:
      return "No valid paths could be found.";
    case InvalidScanParameters:
      return "Scan parameters are invalid.";
    case ScanTypeNotImplemented:
      return "Scan type not implemented.";
    case GenerationError:
      return "Error in path generation.";
    default:
      return "Error in decoding error type: invalid enumeration value.";
  }
}


string dim_name(Dimension d) {
  switch (d) {
    case X: return "x";
    case Y: return "y";
    case Z: return "z";
    case Theta: return "\u03b8"; // theta
    case Phi: return "\u03d5"; // phi
    default: return "[invalid dimension]";
  }
}


string dim_name(size_t i) {
  switch (i) {
    case 0: return "x";
    case 1: return "y";
    case 2: return "z";
    case 3: return "\u03b8"; // theta
    case 4: return "\u03d5"; // phi
    default: return "[invalid dimension]";
  }
}


/* Collision checks */


bool check_any_collisions(const Point& gantry0, const Point& gantry1, const vector<Intersectable>& static_geometry) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);

  const auto
    g0 = point_to_prisms(gantry0, false),
    g1 = point_to_prisms(gantry1, true);

  DEBUG_COUT("Gantry 0: " << SD::serialize(g0[0]));
  DEBUG_COUT("Gantry 1: " << SD::serialize(g1[0]));

  DEBUG_COUT("Checking gantry-gantry collisions.");
  
  for (size_t i = 0 ; i < 3; i++) {
    for (size_t j = 0; j < 3; j++) {
      if (intersect(g0[i], g1[j])) {
        DEBUG_COUT("Gantry-gantry collision found, " << i << ", " << j);
        DEBUG_LEAVE;
        return true;
      }
    }
  }

  DEBUG_COUT("No gantry-gantry collision found. Checking " << static_geometry.size() << " geometry objects.");

  for (size_t gi = 0; gi < static_geometry.size(); gi++) {
    DEBUG_COUT("Checking with object " << gi);

    for (size_t i = 0; i < 3; i++) {
      if (intersect(g0[i], static_geometry[gi]) || intersect(g1[i], static_geometry[gi])) {
        DEBUG_COUT(
          "Collision found, with prism " << i
          << " for gantry " << (intersect(g0[i], static_geometry[gi]) ? 0 : 1) << " with object " << gi 
        );
        DEBUG_LEAVE;
        return true;
      }
    }
  }

  DEBUG_COUT("No collisions found.");
  DEBUG_LEAVE;
  return false;
}


bool is_valid(const vector<MovePoint>& move_path) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);
  for (size_t i = 1; i < move_path.size(); i++) {
    DEBUG_COUT("Checking segment " << i);
    if (move_path[i].gantry0 != move_path[i-1].gantry0
        && move_path[i].gantry1 != move_path[i-1].gantry1) {
      DEBUG_COUT("Found double movement.");
      DEBUG_LEAVE;
      return false;
    }
  }
  DEBUG_COUT("Seems ok.");
  DEBUG_LEAVE;
  return true;
}


inline MovePoint _generate_move_0(const Point start, const Point end, const Point unmoving, Dimension d) {
  switch(d) {
    case X:
      return {
        { {end.position.x, start.position.y, start.position.z}, {start.angle.theta, start.angle.phi} },
        unmoving
      };
    case Y:
      return {
        { {start.position.x, end.position.y, start.position.z}, {start.angle.theta, start.angle.phi} },
        unmoving
      };
    case Z:
      return {
        { {start.position.x, start.position.y, end.position.z}, {start.angle.theta, start.angle.phi} },
        unmoving
      };
    case Theta:
      return {
        { {start.position.x, start.position.y, start.position.z}, {end.angle.theta, start.angle.phi} },
        unmoving
      };
    case Phi:
      return {
        { {start.position.x, start.position.y, start.position.z}, {start.angle.theta, end.angle.phi} },
        unmoving
      };
    default:
      throw runtime_error("Invalid value used in enumeration.");
  }
}


inline MovePoint _generate_move_1(const Point start, const Point end, const Point unmoving, Dimension d) {
  switch(d) {
    case X:
      return {
        unmoving,
        { {end.position.x, start.position.y, start.position.z}, {start.angle.theta, start.angle.phi} }
      };
    case Y:
      return {
        unmoving,
        { {start.position.x, end.position.y, start.position.z}, {start.angle.theta, start.angle.phi} }
      };
    case Z:
      return {
        unmoving,
        { {start.position.x, start.position.y, end.position.z}, {start.angle.theta, start.angle.phi} }
      };
    case Theta:
      return {
        unmoving,
        { {start.position.x, start.position.y, start.position.z}, {end.angle.theta, start.angle.phi} }
      };
    case Phi:
      return {
        unmoving,
        { {start.position.x, start.position.y, start.position.z}, {start.angle.theta, end.angle.phi} }
      };
    default:
      throw runtime_error("Invalid value used in enumeration.");
  }
}


inline bool operator==(const Angle& l, const Angle& r) {
  return (l.theta == r.theta) && (l.phi == r.phi);
}


inline bool operator==(const MovePoint& l, const MovePoint& r) {
  return
    (l.gantry0.position == r.gantry0.position) &&
    (l.gantry1.position == r.gantry1.position) &&
    (l.gantry0.angle == r.gantry0.angle) &&
    (l.gantry1.angle == r.gantry1.angle);
}

bool equal_path_positions(PathGeneration::MovePoint &p1, PathGeneration::MovePoint &p2){
  if(
    p1.gantry0.position == p2.gantry0.position &&
    p1.gantry0.angle == p2.gantry0.angle
  ){
    return true;
  }
  return false;
}

MovePath generate_move(
  const ScanSegment moving,
  const Point unmoving,
  const WhichGantry is_moving,
  const DimensionOrder order
) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);
  if (is_moving == Gantry0) {
    DEBUG_COUT("Generating move sequence for gantry 0 from " << moving.start.position << " to " << moving.end.position);
  } else {
    DEBUG_COUT("Generating move sequence for gantry 1 from " << moving.start.position << " to " << moving.end.position);
  }

  MovePath ret;
  PathGeneration::MovePoint current_path_position;
  PathGeneration::MovePoint next_path_position;
  ret.reserve(6);

  ret.push_back(std::move( (MovePoint){
    is_moving == Gantry0 ? moving.start : unmoving,
    is_moving == Gantry0 ? unmoving : moving.start
  }));

  DEBUG_COUT("0: " << ret[0].gantry0 << ", " << ret[1].gantry1);

  if (is_moving == Gantry0) {
    for (size_t i = 0; i < 5; i++) {
      std::cout << "Trying dim in order: " << i << std::endl;
      next_path_position = std::move(_generate_move_0(ret[i].gantry0, moving.end, unmoving, order[i]));
      if (!equal_path_positions(ret.back(),next_path_position)) {
        ret.push_back(std::move(next_path_position));
      }
    }
  } else {
    for (size_t i = 0; i < 5; i++) {
      next_path_position = std::move(_generate_move_1(ret[i].gantry1, moving.end, unmoving, order[i]));
      if (!equal_path_positions(ret.back(),next_path_position)) {
        ret.push_back(std::move(next_path_position));
      }
    }
  }
  DEBUG_LEAVE;
  return ret;
}


/* Conversions */


array<Prism, 3> point_to_prisms(const Point& p, bool gantry1) {
  array<Prism, 3> ret;
  Vec3 disp = { GANTRY_X_DIM / 2 + SUPPORT_BEAM_WIDTH / 2, GANTRY_Y_DIM / 2 + SUPPORT_BEAM_WIDTH / 2, 0 };

  Quaternion q1, q2;
  q1 = Quaternion(1.0,0.0,0.0,0.0);
  q2 = Quaternion(1.0,0.0,0.0,0.0);
  

  if (gantry1) {
    q1 = Quaternion::from_azimuthal(PI) * Quaternion::from_azimuthal(p.angle.theta);
    q2 = Quaternion::from_spherical_angle(p.angle.theta, 0);//Quaternion::from_azimuthal(PI) * 
  } else { 
    q1 = Quaternion::from_azimuthal(p.angle.theta);
    q2 = Quaternion::from_spherical_angle(p.angle.theta, 0);
  }
  // optical box
  ret[0] = {
    p.position+disp,//TODO: figure out why this was nessisary: rotate_point(p.position+disp, p.position, q1),
    GANTRY_X_DIM/2, GANTRY_Y_DIM/2, GANTRY_Z_DIM/2,
    q2
  };
  // rotary axis
  ret[1] = {
    p.position+disp,
    ROTARY_AXIS_L/2, ROTARY_AXIS_R/2, ROTARY_AXIS_L/4,
    q1
  };
  ret[2] = {
    p.position + (Vec3){0, 0, -SUPPORT_BEAM_HEIGHT}+disp,
    SUPPORT_BEAM_WIDTH, SUPPORT_BEAM_WIDTH, SUPPORT_BEAM_HEIGHT,
    q1
  };
  return ret;
}


Prism point_to_optical_box(const Point& p, bool gantry1) {
  Vec3 disp = { GANTRY_X_DIM / 2 + SUPPORT_BEAM_WIDTH / 2, GANTRY_Y_DIM / 2 + SUPPORT_BEAM_WIDTH / 2,0 };

  Quaternion q1, q2;
  q1 = Quaternion(1.0,0.0,0.0,0.0);
  q2 = Quaternion(1.0,0.0,0.0,0.0);

  if (gantry1) {
    q1 = Quaternion::from_azimuthal(PI) * Quaternion::from_azimuthal(p.angle.theta);
    q2 = Quaternion::from_spherical_angle(p.angle.theta, 0);//Quaternion::from_azimuthal(PI) *
  } else { 
    q1 = Quaternion::from_azimuthal(p.angle.theta);
    q2 = Quaternion::from_spherical_angle(p.angle.theta, 0);
  }

  return {
    p.position + disp,//rotate_point(p.position + disp, p.position, q1),
    GANTRY_X_DIM/2, GANTRY_Y_DIM/2, GANTRY_Z_DIM/2,
    q2
  };
}


Angle operator-(const Angle& l, const Angle& r) {
  return { l.theta - r.theta, l.phi - r.phi };
}


ostream& operator<<(ostream& l, const DimensionOrder& r) {
  l << "{"
    << dim_name(r.first)
    << dim_name(r.second)
    << dim_name(r.third)
    << dim_name(r.fourth)
    << dim_name(r.fifth) << "}";
  return l;
}


ostream& operator<<(ostream& l, const Point& r) {
  l << std::fixed << std::setprecision(3) << "{"
    << r.position.x << ","
    << r.position.y << ","
    << r.position.y << ","
    << r.angle.theta << ","
    << r.angle.phi << "}";
  return l;
}


// used in individual scan type path generation
// changes from pair<moving, unmoving> to an actual MovePoint
MovePoint from_pair(const pair<Point, Point>& p, const WhichGantry which_gantry) {
  switch (which_gantry) {
    case Gantry0:
      return { p.first, p.second };
    case Gantry1:
      return { p.second, p.first };
    default:
      throw "This can't happen";
  }
}


// bool operator==(const Point& l, const Point& r) {
//   return
//     l.position    == r.position &&
//     l.angle.theta == r.angle.theta &&
//     l.angle.phi   == r.angle.phi;
// }


} // end namespace PathGeneration

