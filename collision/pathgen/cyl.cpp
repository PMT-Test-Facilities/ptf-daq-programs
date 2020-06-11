#include "cyl.hpp"
#include "pathgen_internal.hpp"
#include "measurements.hpp"


namespace PathGeneration { namespace Private { namespace Cyl {

vector<pair<Point,Point>> gen_points(const GeneralParams p, const CylindricalParams sp) {
  throw "Not yet implemented.";
  // const size_t
  //   stepsX = (size_t) floor(sp.radius / sp.incr.x),
  //   stepsY = (size_t) floor(sp.radius / sp.incr.y),
  //   stepsZ = (size_t) floor(sp.height / sp.incr.z);

  // const bool need_to_move_other =
  //   (sp.center.x + sp.radius) > 
}

variant<vector<MovePath>, ErrorType> gen_path(const GeneralParams p, const CylindricalParams sp, const vector<Intersectable>& static_geometry) {
  return ErrorType::ScanTypeNotImplemented;/*

  // Check for sane parameters
  if (
    sp.radius <= 0
    || sp.height < 0
    || sp.incr.x <= 0 || sp.incr.y <= 0 || sp.incr.z <= 0
  ) { return ErrorType::InvalidScanParameters; }

  auto desired_path = gen_points(p, sp);
  vector<MovePath> ret;
  ret.reserve(desired_path.size());

  for (size_t i = 1; i < desired_path.size(); ++i) {
    auto subpath = single_move(
      from_pair(desired_path[i-1], p.which_gantry),
      from_pair(desired_path[i],   p.which_gantry),
      static_geometry
    );
    if (__builtin_expect(has<ErrorType>(subpath), 0)) {
      return get<ErrorType>(subpath);
    } else {
      ret.push_back(std::move(get<MovePath>(subpath)));
    }
  }*/
}

} } }
