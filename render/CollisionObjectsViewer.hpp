
#include <boost/optional.hpp>
#include <iostream>
#include <iomanip>
#include <array>
#include <valarray>
#include <algorithm>

#include "midas.h"
#include "geom.hpp"
#include "serialization.hpp"
#include "pathgen.hpp"


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
  namespace Keys {
    // global keys
    HNDLE
      hDB=0,
      collision=0,
      destination=0, start=0, stop=0, reinitialize=0, // control
      position=0, initializing=0, initialized=0, bad_destination=0, completed=0, moving=0,
      ax_moving=0, ax_limit_neg=0, ax_limit_pos=0;  // variables

    typedef std::tuple<HNDLE, HNDLE> GantryPair;

    namespace Motor {
      GantryPair
        destination =  std::make_tuple(0,0),
        start =        std::make_tuple(0,0),
        stop =         std::make_tuple(0,0),
        move =         std::make_tuple(0,0),
        moving =       std::make_tuple(0,0),
        position =     std::make_tuple(0,0),
        limit_neg =    std::make_tuple(0,0),
        limit_pos =    std::make_tuple(0,0),
        velocity =     std::make_tuple(0,0),
        acceleration = std::make_tuple(0,0),
        phidget =      std::make_tuple(0,0);
    }
  }
}

static const auto
  GANTRY_0 = PathGeneration::Gantry0,
  GANTRY_1 = PathGeneration::Gantry1;

