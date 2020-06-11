#include "pathgen.hpp"

namespace PG = PathGeneration;

namespace PathGeneration { namespace Private { namespace Cyl {
  vector<pair<Point,Point>> gen_points(const GeneralParams p, const CylindricalParams sp);
  variant<vector<MovePath>, ErrorType> gen_path(const GeneralParams p, const CylindricalParams sp, const vector<Intersectable>& static_geometry);
} } }
