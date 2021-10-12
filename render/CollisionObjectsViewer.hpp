
#include <boost/optional.hpp>
#include <iostream>
#include <iomanip>
#include <array>
#include <valarray>
#include <algorithm>

#include "geom.hpp"
#include "serialization.hpp"
#include "pathgen.hpp"
//#include "midas.h"

#define BOOL bool
#define TRUE true
#define FALSE false

bool render(Vec3 x);
bool render(Prism x);
bool render(LineSegment x);
bool render(Sphere x);
bool render(Cylinder x);
Intersectable render(Serialization::GeomResult res);


namespace State {
  boost::optional<size_t>  path_index = boost::none;
  PathGeneration::MovePath move_path;

  std::array<BOOL, 10> moving_on_last_check = {{FALSE}};

  namespace Initialization {
    std::array<float, 10> motor_origin = {{nanf("")}};
    std::array<float, 10> position = {{nanf("")}};
  }

  namespace Settings {
    std::array<float, 10> velocity = {{nanf("")}};
    std::array<float, 10> acceleration = {{nanf("")}};
    std::array<float, 10> scale = {{nanf("")}};
    std::array<int, 10> channels = {{0}};
    std::array<float, 10> limits = {{nanf("")}};
  }

  namespace CallbackVars {
    BOOL start;
    BOOL stop;
    BOOL initialize;
    array<BOOL, 8> g0_moving;
    array<BOOL, 8> g1_moving;
  }
}

static const auto
  GANTRY_0 = PathGeneration::Gantry0,
  GANTRY_1 = PathGeneration::Gantry1;

