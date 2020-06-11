#include "rect.hpp"
#include "pathgen_internal.hpp"
#include "measurements.hpp"
#include "debug.hpp"

#include <limits>

#define D_INF numeric_limits<double>::infinity()


namespace PathGeneration { namespace Private { namespace Rect {


vector<pair<Point,Point>> gen_points(const GeneralParams p, const RectangularParams sp) {
  const size_t
    steps_x = (size_t) floor(sp.prism_delta.x / sp.prism_incr.x) + 1,
    steps_y = (size_t) floor(sp.prism_delta.y / sp.prism_incr.y) + 1,
    steps_z = (size_t) floor(sp.prism_delta.z / sp.prism_incr.z) + 1;
  
  vector<pair<Point,Point>> ret;
  ret.reserve(steps_x * steps_y * steps_z + steps_y * steps_z);
  
  bool y_forward = true,
       x_forward = true,
       um_max    = true;

  const double
    um_max_x = p.which_gantry == Gantry0 ? GANTRY_1_MAX_X : GANTRY_0_MAX_X,
    // m_max_y  = steps_y == 1 ? D_INF : sp.prism_start.y + sp.prism_delta.y / 2,
    // gantry 1 should be at its maximum y, 0 should be at 0
    um_y     = p.which_gantry == Gantry0 ? GANTRY_1_MAX_Y : 0,
    // the y value where we switch x for the non scanning gantry
    x_border = sp.prism_start.x + (sp.prism_delta.x / 2);

  for (size_t zi = 0; zi < steps_z; zi++) {
    for (
      size_t yi = y_forward ? 0 : steps_y - 1;
      y_forward ? (yi < steps_y) : (yi >= 0);
      y_forward ? (yi++) : (yi--)
    ) {

      for (
        size_t xi = x_forward ? 0 : steps_x - 1;
        x_forward ? (xi < steps_x) : (xi >= 0);
        x_forward ? (xi++) : (xi--)
    ) {
        ret.push_back({
          {Vec3(
            sp.prism_start.x + (xi * sp.prism_incr.x),
            sp.prism_start.y + (yi * sp.prism_incr.y),
            sp.prism_start.z + (zi * sp.prism_incr.z)
          ), sp.angle},
          {Vec3(
            um_max ? um_max_x : 0, um_y, 0
          ), {0, 0}}
        });
        // switch the unmoving gantry if needed
        const auto last_pos = &(ret[ret.size()].first.position);
        // if unmoving is at high x, the moving is far enough in y, and the moving > the border in x, then move unmoving to low x
        if (__builtin_expect(um_max && last_pos->y > 0.2 && last_pos->x > x_border, 0)) {
          um_max = false;
        }
        // if unmoving is at low x, the moving is far enough in y, and the moving < the border in x, then move unmoving to high x
        else if (__builtin_expect(!um_max && last_pos->y > 0.2 && last_pos->x < x_border, 0)) {
          um_max = true;
        }
      }

      x_forward = !x_forward;
    }

    y_forward = !y_forward;
  }

  return ret;
}


variant<vector<MovePath>, ErrorType> gen_path(const GeneralParams p, const RectangularParams sp, const vector<Intersectable>& static_geometry) {
  DEBUG_ENTER(__PRETTY_FUNCTION__);
  // Check for sane parameters
  if (
       sp.prism_start.x < 0 || sp.prism_start.y < 0 || sp.prism_start.z < 0
    || sp.prism_delta.x < 0 || sp.prism_delta.y < 0 || sp.prism_delta.z < 0
    || sp.prism_incr.x < 0  || sp.prism_incr.y < 0  || sp.prism_incr.z < 0
  ) {
    DEBUG_COUT("Scan parameters are out of bounds.");
    DEBUG_LEAVE;
    return ErrorType::InvalidScanParameters;
  }

  auto desired_path = gen_points(p, sp);
  DEBUG_COUT("Generated desired path with " << desired_path.size() << " points.");
  vector<MovePath> ret;
  ret.reserve(desired_path.size());

  for (size_t i = 1; i < desired_path.size(); ++i) {
    auto subpath = single_move(
      from_pair(desired_path[i-1], p.which_gantry),
      from_pair(desired_path[i],   p.which_gantry),
      static_geometry
    );
    if (__builtin_expect(has<ErrorType>(subpath), 0)) {
      DEBUG_COUT("Subpath generation failed at index " << (i-1) << "\u2013" << i << ".");
      DEBUG_LEAVE;
      return get<ErrorType>(subpath);
    } else {
      ret.push_back(std::move(get<MovePath>(subpath)));
    }
  }

  DEBUG_LEAVE;
  return ret;
}

} } }
