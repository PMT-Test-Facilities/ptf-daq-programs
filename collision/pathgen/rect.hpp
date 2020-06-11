#ifndef __RECT_H__
#define __RECT_H__

#include "pathgen.hpp"

namespace PathGeneration { namespace Private { namespace Rect {
  // returns a pair of moving, unmoving. NOT gantry 0, gantry 1
  vector<pair<Point,Point>> gen_points(const GeneralParams p, const RectangularParams sp);
  variant<vector<MovePath>, ErrorType> gen_path(const GeneralParams p, const RectangularParams sp, const vector<Intersectable>& static_geometry);
} } }

#endif
