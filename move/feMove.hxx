#ifndef __FEMOVE_H__
#define __FEMOVE_H__

#include <iostream>
#include <iomanip>
#include <array>
#include <valarray>
#include <algorithm>

#include <boost/optional.hpp>

#include "midas.h"

#include "pathgen.hpp"

// maximum string length for collidable objects
#define COLLIDE_STR_MAXLEN 256
// number of possible collision objects
#define COLLIDE_STR_NUM 16

// polling time for waiting for motors to start moving, in milliseconds
#define MOTOR_POLL_TIME 500
//timeout for when we decide motors aren't responding, in seconds
#define MOTOR_TIMEOUT 60
// time to try resending the start signal, in seconds
#define MOTOR_RETRY_TIMEOUT 5
// timeout for when we decide phidgets aren't responding, in seconds
#define PHIDGET_TIMEOUT 5

// how long when starting a move do we consider the motors to be not responding, in seconds
#define MOVE_START_TIMEOUT 300

// minimum and maximum tilt values allowed
#define TILT_MIN -105
#define TILT_MAX   15
#define TILT_TOLERANCE 1.0

#define MAX_TILT_RETRIES 4

#ifndef nullptr
#define nullptr NULL
#endif

// if start_move is called, should we call initialize?
// uncomment next line to turn on
// #define AUTO_INIT

// needed to initialize to all falses because the version of C++ on midptf is too old
static const array<BOOL, 10>
  TEN_FALSE = {{FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE,FALSE}},
  TEN_TRUE  = {{TRUE,TRUE,TRUE,TRUE,TRUE,TRUE,TRUE,TRUE,TRUE,TRUE}};

// global state

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
    std::array<float, 10> channels = {{nanf("")}};
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


// MIDAS requirements
extern "C" {

const char
  *frontend_name = "feMove",
  *frontend_file_name = __FILE__;

BOOL frontend_call_loop = FALSE;

INT
  display_period      = 0,
  max_event_size      = 3000,
  max_event_size_frag = 5 * 1024 * 1024,
  event_buffer_size   = 10 * 1000;

INT frontend_init();
INT frontend_exit();
INT begin_of_run( INT run_number, char *error );
INT end_of_run(   INT run_number, char *error );
INT pause_run(    INT run_number, char *error );
INT resume_run(   INT run_number, char *error );
INT frontend_loop();
INT poll_event(INT source, INT count, BOOL test);
INT interrupt_configure(INT cmd, INT source[], PTYPE adr);

INT readout_event(char* pevent, INT off);

// ODB Callbacks

// for these when used as a callback, info will be nullptr
// however if info is not nullptr, the functions will write `true` or `false`
// when they succeed or fail, respectively. So, only give them a pointer to bool.
// monitor is the exception, which should get a pointer to either GANTRY_0 or GANTRY_1

void start_move( HNDLE hDB, HNDLE hKey = 0, void* info = nullptr );
void stop_move(  HNDLE hDB, HNDLE hKey = 0, void* info = nullptr );
void initialize( HNDLE hDB, HNDLE hKey = 0, void* info = nullptr );
void monitor(    HNDLE hDB, HNDLE hKey = 0, void* info = nullptr );


EQUIPMENT equipment[] = {{
  "feMove",
  {
    1,
    0,
    "USER",
    EQ_PERIODIC,
    1,
    "FIXED",
    TRUE,
    RO_ALWAYS,
    1000,
    0,
    0,
    0,
    "","","",
  },
  readout_event,
  NULL,
  NULL,
  NULL
}};

} // end extern "C"


void move(HNDLE hDB);


template<typename T>
void channel_read(HNDLE hDB, State::Keys::GantryPair keys, array<T, 10>& values, DWORD tid) {
  // tid can't be determined automatically because BOOL looks like DWORD to C++
  array<T, 8> m0, m1;

  INT buf_size = 8 * sizeof(T);
  db_get_data(hDB, get<0>(keys), m0.data(), &buf_size, tid);
  buf_size = 8 * sizeof(T);
  db_get_data(hDB, get<1>(keys), m1.data(), &buf_size, tid);

  // again needed because initializer lists aren't working on gcc 4.4
  values[0] = m0[5]; values[1] = m0[6]; values[2] = m0[7]; values[3] = m0[4]; values[4] = m0[3];
  values[5] = m1[6]; values[6] = m1[1]; values[7] = m1[2]; values[8] = m1[4]; values[9] = m1[5];
}


template<typename T>
void channel_write(HNDLE hDB, State::Keys::GantryPair keys, const array<T, 10>& values, DWORD tid) {
  array<T, 8> m0, m1;

  // again needed because initializer lists aren't working on gcc 4.4
  m0[0] = 0.0;
  m0[1] = 0.0;
  m0[2] = 0.0;
  m0[3] = values[4];
  m0[4] = values[3];
  m0[5] = values[0];
  m0[6] = values[1];
  m0[7] = values[2];
  m1[0] = 0.0;
  m1[1] = values[6];
  m1[2] = values[7];
  m1[3] = 0.0;
  m1[4] = values[8];
  m1[5] = values[9];
  m1[6] = values[5];
  m1[7] = 0.0;

  const INT buf_size = 8 * sizeof(T);
  db_set_data(hDB, get<0>(keys), m0.data(), buf_size, 8, tid);
  db_set_data(hDB, get<1>(keys), m1.data(), buf_size, 8, tid);
}


template <typename T, size_t N>
bool any(array<T, N>& vals) {
  for (size_t i = 0; i < N; i++) {
    if (vals[i]) return true;
  }
  return false;
}


template<typename T, size_t N>
bool any_different(array<T, N>& tsl, array<T, N>& tsr) {
  for (size_t i = 0; i < N; i++) {
    if (tsl[i] != tsr[i]) return true;
  }
  return false;
}


template <typename T, size_t N>
bool all(array<T, N>& vals) {
  for (size_t i = 0; i < N; i++) {
    if (!vals[i]) return false;
  }
  return true;
}


// std::equal is failing on midptf
template<typename T, size_t N>
bool equal(const array<T, N>& l, const array<T, N>& r) {
  for (size_t i = 0; i < N; ++i) {
    if (l[i] != r[i]) return false;
  }
  return true;
}


template<PathGeneration::Dimension D>
bool initialize_axis(HNDLE hDB) {
  using PathGeneration::X;
  using PathGeneration::Y;
  using PathGeneration::Z;
  using PathGeneration::Theta;

  static_assert(
    D == X || D == Y || D == Z || D == Theta,
    "Tilt should be initialized from initialize_tilt instead."
  );

  const tuple<size_t, size_t> axes =
    D == X ? make_tuple((size_t)0, (size_t)5) :
    D == Y ? make_tuple((size_t)1, (size_t)6) :
    D == Z ? make_tuple((size_t)2, (size_t)7) :
    make_tuple((size_t)3, (size_t)8); // Theta
  
  array<float, 10> vals, o_position, n_position;
  channel_read(hDB, State::Keys::Motor::destination, vals, TID_FLOAT);
  vals[get<0>(axes)] = 500 * State::Settings::scale[get<0>(axes)];
  vals[get<1>(axes)] = 500 * State::Settings::scale[get<1>(axes)];
  channel_write(hDB, State::Keys::Motor::destination, vals, TID_FLOAT);

  channel_read(hDB, State::Keys::Motor::position, o_position, TID_FLOAT);
  
  array<BOOL, 10> start = TEN_FALSE;
  start[4] = TRUE; start[7] = TRUE;

  channel_write(hDB, State::Keys::Motor::start, start, TID_BOOL);

  ss_sleep(MOTOR_POLL_TIME);

  // struct timespec now, init;
  // uint64_t unow, uinit;

  // clock_gettime(CLOCK_MONOTONIC, &init);
  // uinit = (init.tv_nsec/1000) + (1e6 * init.tv_sec);

  array<BOOL, 10> moving = TEN_TRUE;
  do {
    channel_read(hDB, State::Keys::Motor::moving, moving, TID_BOOL);
    ss_sleep(100);
  } while (any(moving));
  
  channel_read(hDB, State::Keys::Motor::position, n_position, TID_FLOAT);

  char name[64];
  snprintf(name, 64, "initialize_axis<%s>", dim_name(D).c_str());

  bool no_movement;
  if ((no_movement = equal(n_position, o_position))) {
    cm_msg(MERROR, name, "Warning: One or both motors for dimension %s has not moved. Assuming that it's hit the limit switch, but please check that it is working.", dim_name(D).c_str());
    return false;
  }

  State::Initialization::motor_origin[get<0>(axes)] = n_position[get<0>(axes)];
  State::Initialization::motor_origin[get<1>(axes)] = n_position[get<1>(axes)];
  State::Initialization::position[get<0>(axes)] = State::Settings::limits[get<0>(axes)];
  State::Initialization::position[get<1>(axes)] = State::Settings::limits[get<1>(axes)];

  db_set_data_index(hDB, State::Keys::position, &(n_position[get<0>(axes)]), sizeof(float), get<0>(axes), TID_FLOAT);
  db_set_data_index(hDB, State::Keys::position, &(n_position[get<1>(axes)]), sizeof(float), get<1>(axes), TID_FLOAT);

  if (no_movement) {
    cm_msg(MINFO, name, "Finished initializing axis %s. Note that the gantry did not move, please verify that this is correct.", dim_name(D).c_str());
    return true;
  } else {
    cm_msg(MINFO, name, "Finished initializing axis %s.", dim_name(D).c_str());
    return true;
  }
}

bool phidgets_responding(HNDLE hDB);
bool initialize_tilt(HNDLE hDB, size_t n_attemts = 0);

optional<vector<Intersectable>> load_collision_from_odb(HNDLE hDB);

// UNIX time struct useful functions

struct timespec monotonic_clock();

uint64_t timespec_to_usec(const struct timespec tspc);
uint64_t timespec_to_msec(const struct timespec tspc);
uint64_t timespec_to_sec(const struct timespec tspc);
double   timespec_to_d_sec(const struct timespec tspc);

bool operator==(const struct timespec& l, const struct timespec& r);
bool operator!=(const struct timespec& l, const struct timespec& r);
bool operator<(const struct timespec& l, const struct timespec& r);
struct timespec operator-(const struct timespec& l, const struct timespec& r);

#endif
