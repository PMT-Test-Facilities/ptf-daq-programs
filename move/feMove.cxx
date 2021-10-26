#include "feMove.hxx"
#include "col.hpp"
#include "has.hpp"

#define HAS_PHI false
#define HAS_THETA false

namespace PG = PathGeneration;
namespace SD = Serialization;


using PathGeneration::X;
using PathGeneration::Y;
using PathGeneration::Z;
using PathGeneration::Theta;
using PathGeneration::Phi;


INT frontend_init() {
  cm_get_experiment_database(&State::Keys::hDB, NULL);

  const auto hDB = State::Keys::hDB;
  if (hDB == 0) {
    std::cerr << C_BR_RED << "Could not load database. Got handle: " << hDB << ".\n" << C_RESET;
    return CM_DB_ERROR;
  }

  State::path_index = boost::none;
  State::move_path.clear();

  // load the global keys
  db_find_key(hDB, 0, "/Equipment/Move/Control/Destination",  &State::Keys::destination);
  db_find_key(hDB, 0, "/Equipment/Move/Control/Start Move",   &State::Keys::start);
  db_find_key(hDB, 0, "/Equipment/Move/Control/Stop Move",    &State::Keys::stop);
  db_find_key(hDB, 0, "/Equipment/Move/Control/ReInitialize", &State::Keys::reinitialize);

  db_find_key(hDB, 0, "/Equipment/Move/Variables/Position",        &State::Keys::position);
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Initializing",    &State::Keys::initializing);
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Initialized",     &State::Keys::initialized);
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Bad Destination", &State::Keys::bad_destination);
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Completed",       &State::Keys::completed);
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Moving",          &State::Keys::moving);

  db_find_key(hDB, 0, "/Equipment/Move/Variables/Axis Moving",         &State::Keys::ax_moving);
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Negative Axis Limit", &State::Keys::ax_limit_neg);
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Positive Axis Limit", &State::Keys::ax_limit_pos);

  db_find_key(hDB, 0, "/Equipment/Move/Settings/Collision", &State::Keys::collision);

  // initialize values
  const BOOL tmp = FALSE;
  db_set_data(hDB, State::Keys::start,           &tmp, sizeof(BOOL), 1, TID_BOOL);
  db_set_data(hDB, State::Keys::stop,            &tmp, sizeof(BOOL), 1, TID_BOOL);
  db_set_data(hDB, State::Keys::reinitialize,    &tmp, sizeof(BOOL), 1, TID_BOOL);
  db_set_data(hDB, State::Keys::initialized,     &tmp, sizeof(BOOL), 1, TID_BOOL);
  db_set_data(hDB, State::Keys::initializing,    &tmp, sizeof(BOOL), 1, TID_BOOL);
  db_set_data(hDB, State::Keys::bad_destination, &tmp, sizeof(BOOL), 1, TID_BOOL);
  db_set_data(hDB, State::Keys::completed,       &tmp, sizeof(BOOL), 1, TID_BOOL);

  // read settings
  INT bufsize = 10 * sizeof(float);
  db_get_value(hDB, 0, "/Equipment/Move/Settings/Velocity",        State::Settings::velocity.data(),     &bufsize, TID_FLOAT, FALSE);
  bufsize = 10 * sizeof(float);
  db_get_value(hDB, 0, "/equipment/move/settings/Acceleration",    State::Settings::acceleration.data(), &bufsize, TID_FLOAT, FALSE);
  bufsize = 10 * sizeof(float);
  db_get_value(hDB, 0, "/equipment/move/settings/Motor Scaling",   State::Settings::scale.data(),        &bufsize, TID_FLOAT, FALSE);
  bufsize = 10 * sizeof(float);
  db_get_value(hDB, 0, "/Equipment/Move/Settings/Axis Channels",   State::Settings::channels.data(),     &bufsize, TID_INT, FALSE);
  bufsize = 10 * sizeof(float);
  db_get_value(hDB, 0, "/Equipment/Move/Settings/Limit Positions", State::Settings::limits.data(),       &bufsize, TID_FLOAT, FALSE);

  // load motor & phidget keys
  db_find_key(hDB, 0, "/Equipment/Motors00/Settings/Destination", &get<0>(State::Keys::Motor::destination));
  db_find_key(hDB, 0, "/Equipment/Motors01/Settings/Destination", &get<1>(State::Keys::Motor::destination));

  db_find_key(hDB, 0, "/Equipment/Motors00/Settings/Move", &get<0>(State::Keys::Motor::move));
  db_find_key(hDB, 0, "/Equipment/Motors01/Settings/Move", &get<1>(State::Keys::Motor::move));

  db_find_key(hDB, 0, "/Equipment/Motors00/Variables/Moving", &get<0>(State::Keys::Motor::moving));
  db_find_key(hDB, 0, "/Equipment/Motors01/Variables/Moving", &get<1>(State::Keys::Motor::moving));

  db_find_key(hDB, 0, "/Equipment/Motors00/Settings/Stop", &get<0>(State::Keys::Motor::stop));
  db_find_key(hDB, 0, "/Equipment/Motors01/Settings/Stop", &get<1>(State::Keys::Motor::stop));

  db_find_key(hDB, 0, "/Equipment/Motors00/Variables/Position", &get<0>(State::Keys::Motor::position));
  db_find_key(hDB, 0, "/Equipment/Motors01/Variables/Position", &get<1>(State::Keys::Motor::position));

  db_find_key(hDB, 0, "/Equipment/Motors00/Variables/Limit Pos", &get<0>(State::Keys::Motor::limit_pos));
  db_find_key(hDB, 0, "/Equipment/Motors01/Variables/Limit Pos", &get<1>(State::Keys::Motor::limit_pos));

  db_find_key(hDB, 0, "/Equipment/Motors00/Variables/Limit Neg", &get<0>(State::Keys::Motor::limit_neg));
  db_find_key(hDB, 0, "/Equipment/Motors01/Variables/Limit Neg", &get<1>(State::Keys::Motor::limit_neg));

  db_find_key(hDB, 0, "/Equipment/Motors00/Settings/Velocity", &get<0>(State::Keys::Motor::velocity));
  db_find_key(hDB, 0, "/Equipment/Motors01/Settings/Velocity", &get<1>(State::Keys::Motor::velocity));

  db_find_key(hDB, 0, "/Equipment/Motors00/Settings/Acceleration", &get<0>(State::Keys::Motor::acceleration));
  db_find_key(hDB, 0, "/Equipment/Motors01/Settings/Acceleration", &get<1>(State::Keys::Motor::acceleration));

  db_find_key(hDB, 0, "/Equipment/OpticalBox00/Variables", &get<0>(State::Keys::Motor::phidget));
  db_find_key(hDB, 0, "/Equipment/Phidget01/Variables", &get<1>(State::Keys::Motor::phidget));

  // set hotlinks

  db_open_record(hDB, State::Keys::start, &State::CallbackVars::start, sizeof(BOOL), MODE_READ, &start_move, nullptr);
  db_open_record(hDB, State::Keys::stop,  &State::CallbackVars::stop,  sizeof(BOOL), MODE_READ, &stop_move, nullptr);

  db_open_record(hDB, State::Keys::reinitialize, &State::CallbackVars::initialize, sizeof(BOOL), MODE_READ, initialize, nullptr);

  db_open_record(hDB, get<0>(State::Keys::Motor::moving), State::CallbackVars::g0_moving.data(), 8 * sizeof(BOOL), MODE_READ, monitor, (void*) &GANTRY_0);
  db_open_record(hDB, get<1>(State::Keys::Motor::moving), State::CallbackVars::g1_moving.data(), 8 * sizeof(BOOL), MODE_READ, monitor, (void*) &GANTRY_1);

  array<float, 10> temp_v, temp_a;

  for (size_t i = 0; i < 10; i++) {
    temp_v[i] = State::Settings::velocity[i]     * fabs(State::Settings::scale[i]);
    temp_a[i] = State::Settings::acceleration[i] * fabs(State::Settings::scale[i]);
  }

  channel_write(hDB, State::Keys::Motor::velocity,     temp_v, TID_FLOAT);
  channel_write(hDB, State::Keys::Motor::acceleration, temp_a, TID_FLOAT);

  State::moving_on_last_check = TEN_FALSE;
  cm_msg(MDEBUG, "feMove:frontend_init", "feMove initialized.");
  return CM_SUCCESS;
}


INT frontend_exit() {
  cm_disconnect_experiment();
  State::path_index = boost::none;
  State::move_path.clear();
  return CM_SUCCESS;
}


// ODB callbacks


void start_move(HNDLE hDB, HNDLE hKey, void* info) {
  BOOL initialized;
  INT  bufsize = sizeof(initialized);
  db_get_data(hDB, State::Keys::initialized, &initialized, &bufsize, TID_BOOL);

  if (!State::CallbackVars::start) {
    cm_msg(MDEBUG, "feMove:start_move", "start_move called, but State::CallbackVars::start is false. Not starting move.");
    return;
  }

  if (!initialized) {
#ifdef AUTO_INIT
    cm_msg(MINFO, "feMove:start_move", "Gantries are not initialized. Initializing them.");
    bool succeeded;
    initialize(hDB, 0, &succeeded);
    if (!succeeded) {
      cm_msg(MERROR, "feMove:start_move", "Initialization failed. Cannot start move.");
      if (info != nullptr) {
        *static_cast<bool*>(info) = false;
      }
      return;
    }
#else
    cm_msg(MERROR, "feMove:start_move", "Cannot start move because gantries are not initialized. Please initialize them.");
    if (info != nullptr) {
      *static_cast<bool*>(info) = false;
    }
    return;
#endif
  }

  // check if motors are already moving
  array<BOOL, 10> moving;
  channel_read(hDB, State::Keys::Motor::moving, moving, TID_BOOL);
  if (any(moving)) {
    State::moving_on_last_check = moving;
    cm_msg(MERROR, "feMove:start_move", "Cannot start move because the motors are already moving.");
    return;
  }

  State::moving_on_last_check = TEN_FALSE;

  cm_msg(MINFO, "feMove:start_move", "Checking for collidable objects.");
  
  auto _collidable = load_collision_from_odb(hDB);
  if (!_collidable) {
    cm_msg(MERROR, "feMove:start_move", "Error on loading collidable objects from ODB. Cannot continue.");
    return;
  }
  auto collidable = std::move(*_collidable);
  
  cm_msg(MINFO, "feMove:start_move", "Calculating move.");

  array<float, 10> position, destination;
  bufsize = 10 * sizeof(float);
  db_get_data(hDB, State::Keys::position, position.data(), &bufsize, TID_FLOAT);
  bufsize = 10 * sizeof(float);
  db_get_data(hDB, State::Keys::destination, destination.data(), &bufsize, TID_FLOAT);
  
  auto start = PG::move_point_from_array(position);
  auto end   = PG::move_point_from_array(destination);

  auto path = PG::single_move(start, end, collidable);
  if (has<SD::ErrorType>(path)) {
    cm_msg(MERROR, "feMove:start_move", "Could not generate path. Error message: \"%s\".", PathGeneration::error_message(get<PG::ErrorType>(path)).c_str());
    return;
  } else {
    State::move_path  = get<PG::MovePath>(path);
    State::path_index = 0;
    cm_msg(MINFO, "feMove:start_move", "Path generation successful. Move split into %zd steps.", State::move_path.size());
    move(hDB);
  }
}




void stop_move(HNDLE hDB, HNDLE hKey, void* info) {
  cm_msg(MINFO, "feMove:stop_move", "Stopping move.");

  array<BOOL, 10> values = TEN_TRUE;
  channel_write(hDB, State::Keys::Motor::stop, values, TID_BOOL);

  cm_msg(MINFO, "feMove:stop_move", "Stop command sent.");

  if (info != nullptr) {
    *static_cast<bool*>(info) = true;
  }
}




void initialize(HNDLE hDB, HNDLE hKey, void* info) {
  if (info != nullptr) {
    *(static_cast<bool*>(info)) = false;
  }
  cm_msg(MINFO, "feMove:initialize", "Initializing.");
  BOOL myb = FALSE;
  INT  bsize = sizeof(BOOL);
  
  db_get_data(hDB, State::Keys::initializing, &myb, &bsize, TID_BOOL);
  if (myb) {
    cm_msg(MERROR, "feMove:initialize", "Cannot initialize since we are already initializing.");
    goto failure;
  }
  myb = FALSE;
  db_set_data(hDB, State::Keys::initialized, &myb, sizeof(BOOL), 1, TID_BOOL);
  myb = TRUE;
  db_set_data(hDB, State::Keys::initializing, &myb, sizeof(BOOL), 1, TID_BOOL);
  // now initialize in known safe order Z, Y, X, Theta, Phi
  if (!initialize_axis<Z>(hDB)) {
    cm_msg(MERROR, "feMove:initialize", "Could not initialize axis z.");
    goto failure;
  }
  if (!initialize_axis<Y>(hDB)) {
    cm_msg(MERROR, "feMove:initialize", "Could not initialize axis y.");
    goto failure;
  }
  if (!initialize_axis<X>(hDB)) {
    cm_msg(MERROR, "feMove:initialize", "Could not initialize axis x.");
    goto failure;
  }
  if (HAS_THETA && !initialize_axis<Theta>(hDB)) {
    cm_msg(MERROR, "feMove:initialize", "Could not initialize azimuthal angle (rotation).");
    goto failure;
  }
  if (HAS_PHI && !initialize_tilt(hDB)) {
    cm_msg(MERROR, "feMove:initialize", "Could not initialize polar angle (tilt).");
    goto failure;
  }else{
    goto success;
  }

success:
  myb = TRUE;
  db_set_data(hDB, State::Keys::initialized, &myb, sizeof(BOOL), 1, TID_BOOL);
  myb = FALSE;
  db_set_data(hDB, State::Keys::initializing, &myb, sizeof(BOOL), 1, TID_BOOL);
  if (info != nullptr) {
    *static_cast<bool*>(info) = true;
  }
  return;

failure:
  myb = FALSE;
  db_set_data(hDB, State::Keys::initialized, &myb, sizeof(BOOL), 1, TID_BOOL);
  db_set_data(hDB, State::Keys::initializing, &myb, sizeof(BOOL), 1, TID_BOOL);
  if (info != nullptr) {
    *static_cast<bool*>(info) = false;
  }
  return;
}




void monitor(HNDLE hDB, HNDLE hKey, void* info) {

  if (!State::path_index) {  // if we don't have a path, don't know if the path is done
    //cm_msg(MERROR, "feMove:monitor", "Monitor called without path_index being set. This variable should be set when a path is generated, so something has gone wrong.");
    //State::moving_on_last_check = false;
    return;
  }
#ifdef DEBUG
  //cout << C_BLUE << "Monitor called." << C_RESET << endl;
#endif

  array<BOOL, 10> moving;
  channel_read(hDB, State::Keys::Motor::moving, moving, TID_BOOL);

  BOOL any_moving = any(moving);

  db_set_data(hDB, State::Keys::moving, &any_moving, sizeof(BOOL), 1, TID_BOOL);

  array<float, 10> position;
  channel_read(hDB, State::Keys::Motor::position, position, TID_FLOAT);
  for (size_t i = 0; i < 10; i++){
    position[i] = (position[i] - State::Initialization::motor_origin[i]) / State::Settings::scale[i] + State::Settings::limits[i];
    //if(i==3 || i==4 || i==8 ||i==9)//TODO: use get axis instead?
      //position[i]/=RAD2DEG;
  }
  db_set_data(hDB, State::Keys::position, position.data(), 10*sizeof(float), 10, TID_FLOAT);

  array<BOOL, 10> neglim, poslim;
  channel_read(hDB, State::Keys::Motor::limit_pos, poslim, TID_BOOL);
  channel_read(hDB, State::Keys::Motor::limit_neg, neglim, TID_BOOL);
  db_set_data(hDB, State::Keys::ax_limit_pos, poslim.data(), 10*sizeof(BOOL), 10, TID_BOOL);
  db_set_data(hDB, State::Keys::ax_limit_neg, neglim.data(), 10*sizeof(BOOL), 10, TID_BOOL);

  if (!State::path_index) {  // if we don't have a path, don't know if the path is done
    cm_msg(MERROR, "feMove:monitor", "Monitor called without path_index being set. This variable should be set when a path is generated, so something has gone wrong.");
    State::moving_on_last_check = moving;
    return;
  }

  bool stopped_from_limit = false;

  if (any(State::moving_on_last_check) && !any_moving) {
    for (size_t i = 0; i < 10; i++) {
      //TODO: there could be a bug here for something
    if (State::Settings::channels[i] != -1 && fabs(*State::move_path[*State::path_index][i] - position[i]) > 0.001) {
        if (poslim[i] || neglim[i]) {
          if (stopped_from_limit)
            cm_msg(MINFO, "feMove:monitor", "Limit switch for %s was also triggered.", PG::dim_name(i % 5).c_str());
          else 
            cm_msg(MINFO, "feMove:monitor", "Stopped moving because limit switch for %s was triggered.", PG::dim_name(i % 5).c_str());
          stopped_from_limit = true;
          
        }
        else if (!stopped_from_limit) {
          cm_msg(MERROR, "feMove:monitor", "Move at index %zd failed for axis %s [%zd]", *State::path_index, PG::dim_name(i % 5).c_str(), i);
          return;
        }
      }
    }

    if (*State::path_index + 1 < State::move_path.size()) {
      (*State::path_index)++;
      move(hDB);
    } else {
      State::path_index = boost::none;
      cm_msg(MINFO, "feMove:monitor", "Move path complete.");
    }
  
  }

  State::moving_on_last_check = moving;
}


bool phidgets_responding(HNDLE hDB) {
  std::array<double, 12>
    p0_values_old, p1_values_old,
    p0_values_new, p1_values_new;
  struct timespec now;
  const auto init = monotonic_clock();

  INT bufsize = 12 * sizeof(double), status;
  status = db_get_value(hDB, get<0>(State::Keys::Motor::phidget), "PH00", p0_values_old.data(), &bufsize, TID_DOUBLE, FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "feMove:phdigets_responding", "Could not read data for phidget 0.");
    return false;
  }
  bufsize = 10 * sizeof(double);
  status = db_get_value(hDB, get<1>(State::Keys::Motor::phidget), "PH01", p1_values_old.data(), &bufsize, TID_DOUBLE, FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "feMove:phdigets_responding", "Could not read data for phidget 1.");
    return false;
  }

  while (true) {
    now = monotonic_clock();

    bufsize = 12 * sizeof(double);
    db_get_value(hDB, get<0>(State::Keys::Motor::phidget), "PH00", p0_values_new.data(), &bufsize, TID_DOUBLE, FALSE);
    bufsize = 10 * sizeof(double);
    db_get_value(hDB, get<1>(State::Keys::Motor::phidget), "PH01", p1_values_new.data(), &bufsize, TID_DOUBLE, FALSE);

    bool
      p0eq = equal(p0_values_old, p0_values_new),
      p1eq = equal(p1_values_old, p1_values_new);

    if (!p0eq && !p1eq) {
      break;
    }
    else if (timespec_to_d_sec(now - init) > (PHIDGET_TIMEOUT)) {
      cm_msg(MERROR, "feMove:phidgets_responding", "Phidgets in gantries don't appear to be active. Cannot continue.");
      return false;
    }
    else {
      ss_sleep(250);
    }

  }

  return true;
}




bool initialize_tilt(HNDLE hDB, size_t n_attemts) {
  if (!phidgets_responding(hDB)) {
    return false;
  }
  else if (n_attemts >= MAX_TILT_RETRIES) return false;

  array<double, 12> phidg0, phidg1;
  INT bufsize = 12 * sizeof(double);
  db_get_value(hDB, get<0>(State::Keys::Motor::phidget), "OB10", phidg0.data(), &bufsize, TID_DOUBLE, FALSE);
  bufsize = 10 * sizeof(double);
  db_get_value(hDB, get<1>(State::Keys::Motor::phidget), "PH01", phidg1.data(), &bufsize, TID_DOUBLE, FALSE);

  if (phidg0[7] < TILT_MIN || phidg0[7] > TILT_MAX) {
    cm_msg(MERROR, "feMove:initialize_tilt", "Tilt reading (%.2f deg) for phidget 0 is outside of normal bounds [%i, %i].", phidg0[7], TILT_MIN, TILT_MAX);
    return false;
  }
  else if (phidg1[7] < TILT_MIN || phidg1[7] > TILT_MAX) {
    cm_msg(MERROR, "feMove:initialize_tilt", "Tilt reading (%.2f deg) for phidget 1 is outside of normal bounds [%i, %i].", phidg1[7], TILT_MIN, TILT_MAX);
    return false;
  }

  // read in old destination
  array<double, 10> destination;
  channel_read(hDB, State::Keys::Motor::destination, destination, TID_DOUBLE);
  destination[4] = -(phidg0[7] * State::Settings::scale[4]);
  destination[7] = -(phidg1[7] * State::Settings::scale[7]);
  channel_write(hDB, State::Keys::Motor::destination, destination, TID_DOUBLE);

  // track original tilts to see if we've moved
  double tilt0_0 = phidg0[7], tilt0_1 = phidg1[7];
  cm_msg(MINFO, "feMove:initialize_tilt", "Tilts are %.2f and %.2f. Moving to compensate (attempt %zd/%i).", phidg0[7], phidg1[7], n_attemts+1, MAX_TILT_RETRIES+1);

  array<BOOL, 10> start = TEN_FALSE;
  start[4] = TRUE;
  start[7] = TRUE;
  //channel_write(hDB, State::Keys::Motor::start, start, TID_BOOL);

  ss_sleep(MOTOR_POLL_TIME);  // let the move start. todo: check how long it really needs

  array<BOOL, 10> moving = TEN_TRUE;
  do {
    channel_read(hDB, State::Keys::Motor::moving, moving, TID_BOOL);
    ss_sleep(100);
  } while (any(moving));

  bufsize = 10 * sizeof(double);
  db_get_value(hDB, get<0>(State::Keys::Motor::phidget), "PH00", phidg0.data(), &bufsize, TID_DOUBLE, FALSE);
  bufsize = 10 * sizeof(double);
  db_get_value(hDB, get<1>(State::Keys::Motor::phidget), "PH01", phidg1.data(), &bufsize, TID_DOUBLE, FALSE);

  if (phidg0[7] != phidg0[7] || phidg1[7] != phidg1[7]) {
    cm_msg(MERROR, "feMove:initialize_tilt", "Read NaN from tilt.");
    return false;
  }
  else if (fabs(phidg0[7]) < TILT_TOLERANCE && fabs(phidg1[7] < TILT_TOLERANCE)) {
    array<float, 10> position;
    channel_read(hDB, State::Keys::Motor::position, position, TID_FLOAT);
    State::Initialization::position[4] = State::Settings::limits[4];
    State::Initialization::position[7] = State::Settings::limits[7];
    State::Initialization::motor_origin[4] = position[4] - State::Settings::limits[4] * State::Settings::scale[4];
    State::Initialization::motor_origin[7] = position[7] - State::Settings::limits[7] * State::Settings::scale[7];
    return true;
  }
  else if (fabs(phidg0[7] - tilt0_0) < TILT_TOLERANCE && fabs(phidg1[7] - tilt0_1) < TILT_TOLERANCE) {
    cm_msg(MERROR, "feMove:initialize_tilt", "One or both motors is not moving and we are not at the desired tilt.");
    return false;
  }
  else {
    return initialize_tilt(hDB, n_attemts + 1);
  }
}




optional<vector<Intersectable>> load_collision_from_odb(HNDLE hDB) {
  auto raw = new array<char, COLLIDE_STR_MAXLEN*COLLIDE_STR_NUM>;
  INT  bufsize = COLLIDE_STR_MAXLEN*COLLIDE_STR_NUM;
  db_get_data(hDB, State::Keys::collision, raw->data(), &bufsize, TID_STRING);
  
  vector<string> strings;
  for (size_t i = 0; i < COLLIDE_STR_NUM; i++) {
    auto ptr = raw->data() + (COLLIDE_STR_MAXLEN * i);
    // ignore empty strings, as well as commented ones
    cm_msg(MINFO, "what?", ptr);
    if (std::string(ptr).compare("") != 0 &&
        std::string(ptr).substr(0,1).compare("#") != 0 &&
        std::string(ptr).substr(0,1).compare("!") != 0 ){
      strings.push_back(string(ptr));
    }
  }

  delete raw;

  vector<Intersectable> ret;

  if (strings.size() == 0) {
    cm_msg(MINFO, "feMove:load_collision_from_odb", "There are no objects to collide with in the ODB.");
    return ret;
  }

  for (size_t i = 0; i < strings.size(); i++) {
    auto str = strings[i];
    auto res = SD::deserialize(str);
    if (has<SD::ErrorType>(res)) {
      cm_msg(
        MERROR, "feMove:load_collision_from_odb", "Could not deserialize collidable object %zd (`%s'). Error message is: \"%s\".",
        i, str.c_str(), SD::error_message(get<SD::ErrorType>(res)).c_str()
      );
      return boost::none;
    } else if (has<Vec3>(res)) {
      ret.push_back(get<Vec3>(res));
    } else if (has<LineSegment>(res)) {
      ret.push_back(get<LineSegment>(res));
    } else if (has<Prism>(res)) {
      ret.push_back(get<Prism>(res));
    } else if (has<Sphere>(res)) {
      ret.push_back(get<Sphere>(res));
    } else if (has<Cylinder>(res)) {
      ret.push_back(get<Cylinder>(res));
    } else {
      cm_msg(
        MERROR, "feMove:load_collision_from_odb", "Could not convert type to Intersectable for collidable object %zd (`%s'). "
        "Please ensure that the datatype is one of: Vec3, LineSegment, Prism, Sphere, or Cylinder.", i, str.c_str()
      );
      return boost::none;
    }
  }

  return ret;
}




void move(HNDLE hDB) {
  if (!State::path_index || State::move_path.size() == 0) {
    cm_msg(MERROR, "feMove:move", "Move function was called, but no movement path is set. Cannot continue.");
    return;
  } else if (State::move_path.size() <= *State::path_index) {
    cm_msg(MERROR, "feMove:move", "Move function was called, but the path index is larger than the size of the path. Cannot continue.");
    return;
  }

  const array<BOOL, 10> start_all = TEN_TRUE;

  cm_msg(MDEBUG, "feMove:move", "Moving to movement index %zd (step %zd/%zd).", *State::path_index, (*State::path_index)+1, State::move_path.size());
  auto pt = State::move_path[*State::path_index];

  // read in the current axis positions
  array<float, 10> positions, deltas, dest_cout;
  channel_read(hDB, State::Keys::Motor::position, positions, TID_FLOAT);

  for (size_t i = 0; i < 10; ++i) {
    if(i==3 || i==4 || i >= 5 || i == 8 || i==9){//TODO remove when we add back in tilt
      deltas[i] = 0;
      dest_cout[i] = (positions[i] - State::Initialization::motor_origin[i]);
      continue;
    }
    deltas[i] = round((*pt[i])*State::Settings::scale[i] - (positions[i] - State::Initialization::motor_origin[i]));
    dest_cout[i] =  (*pt[i]) * State::Settings::scale[i];
  }

  /*for (size_t i = 0; i < 10; ++i) {
    std::cout << round((*pt[i])*State::Settings::scale[i] - (positions[i] - State::Initialization::motor_origin[i])) << " " << (*pt[i]) <<" "<< positions[i]/State::Settings::scale[i] <<" "<< State::Initialization::motor_origin[i]/State::Settings::scale[i] << std::endl;
  }*/

  // read current positions of motors to see if the movement actually starts
  array<float, 8> m0start, m1start, m0dest, m1dest, m0pos, m1pos;
  INT bufsize = 8 * sizeof(float);
  db_get_data(hDB, get<0>(State::Keys::Motor::position), m0start.data(), &bufsize, TID_FLOAT);
  bufsize = 8 * sizeof(float);
  db_get_data(hDB, get<1>(State::Keys::Motor::position), m1start.data(), &bufsize, TID_FLOAT);
  bufsize = 8 * sizeof(float);
  db_get_data(hDB, get<0>(State::Keys::Motor::destination), m0dest.data(), &bufsize, TID_FLOAT);
  bufsize = 8 * sizeof(float);
  db_get_data(hDB, get<1>(State::Keys::Motor::destination), m1dest.data(), &bufsize, TID_FLOAT);


  // no movement is necessary
  if (!any(deltas)) {
    cm_msg(MINFO, "feMove:move", "Note: no move is required.");
    (*State::path_index)++;
    if (State::move_path.size() <= *State::path_index) {
      cm_msg(MINFO, "feMove:move", "This was the last move step, so we're done.");
      State::moving_on_last_check = TEN_FALSE;
      State::path_index = boost::none;
    } else {
      move(hDB);
    }
  }else{
    channel_write(hDB, State::Keys::Motor::destination, deltas, TID_FLOAT);
    channel_write(hDB, State::Keys::Motor::move, start_all, TID_BOOL);
  }

  // track total time
  const auto init   = monotonic_clock();
  auto last_restart = init;

  array<BOOL, 8> m0_lim_pos, m0_lim_neg, m1_lim_pos, m1_lim_neg;

  bool waiting = true;

  do {
    const auto now = monotonic_clock();

    if (timespec_to_d_sec(now - init) > MOTOR_TIMEOUT) {
      cm_msg(MERROR, "feMove:move", "The motors have not started moving in the time allowed (%is).", MOTOR_TIMEOUT);
      waiting = false;
    }

    if (timespec_to_d_sec(now - last_restart) > MOTOR_RETRY_TIMEOUT) {
      if (last_restart != init) {
        cm_msg(MINFO, "feMove:move", "No movement after %is. Signalling start again.", MOTOR_RETRY_TIMEOUT);
      }
      bufsize = 8*sizeof(float);
      db_get_data(hDB, get<0>(State::Keys::Motor::position), m0pos.data(), &bufsize, TID_FLOAT);
      db_get_data(hDB, get<1>(State::Keys::Motor::position), m1pos.data(), &bufsize, TID_FLOAT);
      bufsize = 8*sizeof(BOOL);
      db_get_data(hDB, get<0>(State::Keys::Motor::limit_pos), m0_lim_pos.data(), &bufsize, TID_BOOL);
      db_get_data(hDB, get<1>(State::Keys::Motor::limit_pos), m1_lim_pos.data(), &bufsize, TID_BOOL);
      db_get_data(hDB, get<0>(State::Keys::Motor::limit_neg), m0_lim_neg.data(), &bufsize, TID_BOOL);
      db_get_data(hDB, get<1>(State::Keys::Motor::limit_neg), m1_lim_neg.data(), &bufsize, TID_BOOL);
      
      last_restart = monotonic_clock();
    }

    if (any_different(m0pos, m0start)) {
      cm_msg(MDEBUG, "feMove:move", "Gantry 0 motors have started moving.");
      waiting = false;
    }

    else if (any_different(m1pos, m1start)) {
      cm_msg(MDEBUG, "feMove:move", "Gantry 1 motors have started moving.");
      waiting = false;
    }

    else if (
      (m0dest[4] > 0 && m0_lim_neg[4]) ||
      (m0dest[5] > 0 && m0_lim_neg[5]) ||
      (m0dest[6] > 0 && m0_lim_neg[6]) ||
      (m0dest[7] > 0 && m0_lim_neg[7])
    ) {
      cm_msg(MDEBUG, "feMove:move", "Move could not be started because a motor for gantry 0 is at a negative limit switch.");
      waiting = false;
    }

    else if (
      (m0dest[4] < 0 && m0_lim_pos[4]) ||
      (m0dest[5] < 0 && m0_lim_pos[5]) ||
      (m0dest[6] < 0 && m0_lim_pos[6]) ||
      (m0dest[7] < 0 && m0_lim_pos[7])
    ) {
      cm_msg(MDEBUG, "feMove:move", "Move could not be started because a motor for gantry 0 is at a positive limit switch.");
      waiting = false;
    }

    else if (
      (m1dest[1] > 0 && m1_lim_neg[1]) ||
      (m1dest[2] > 0 && m1_lim_neg[2]) ||
      (m1dest[3] > 0 && m1_lim_neg[3]) ||
      (m1dest[4] > 0 && m1_lim_neg[4])
    ) {
      cm_msg(MDEBUG, "feMove:move", "Move could not be started because a motor for gantry 1 is at a negative limit switch.");
      waiting = false;
    }

    else if (
      (m1dest[1] < 0 && m1_lim_pos[1]) ||
      (m1dest[2] < 0 && m1_lim_pos[2]) ||
      (m1dest[3] < 0 && m1_lim_pos[3]) ||
      (m1dest[4] < 0 && m1_lim_pos[4])
    ) {
      cm_msg(MDEBUG, "feMove:move", "Move could not be started because a motor for gantry 1 is at a positive limit switch.");
      waiting = false;
    }

  } while (waiting);
}


// UNIX time struct useful functions


struct timespec monotonic_clock() {
  struct timespec time;
  clock_gettime(CLOCK_MONOTONIC, &time);
  return std::move(time);
}

uint64_t timespec_to_usec(const struct timespec tspc) {
  return static_cast<uint64_t>(round(tspc.tv_nsec/1e3) + (1e6 * tspc.tv_sec));
}

uint64_t timespec_to_msec(const struct timespec tspc) {
  return static_cast<uint64_t>(round(tspc.tv_nsec/1e6) + (1e3 * tspc.tv_sec));
}

uint64_t timespec_to_sec(const struct timespec tspc) {
  return static_cast<uint64_t>(round(tspc.tv_nsec/1e9) + tspc.tv_sec);
}

double timespec_to_d_sec(const struct timespec tspc) {
  return (tspc.tv_nsec/1e9) + tspc.tv_sec;
}

bool operator==(const struct timespec& l, const struct timespec& r) {
  return (l.tv_sec == r.tv_sec) && (l.tv_nsec == r.tv_nsec);
}

bool operator!=(const struct timespec& l, const struct timespec& r) {
  return (l.tv_sec != r.tv_sec) || (l.tv_nsec != r.tv_nsec);
}

bool operator<(const struct timespec& l, const struct timespec& r) {
  if (l.tv_sec < r.tv_sec) {
    return true;
  } else if (l.tv_sec > r.tv_sec) {
    return false;
  } else {
    return l.tv_nsec < r.tv_nsec;
  }
}

struct timespec operator-(const struct timespec& l, const struct timespec& r) {
  int64_t nsec_diff = l.tv_nsec - r.tv_nsec;
  bool sub_one = nsec_diff < 0;
  time_t sec = l.tv_sec - r.tv_sec - (sub_one ? 1 : 0);
  return {
    sec,
    sub_one ? nsec_diff + 1000000000 : nsec_diff
  };
}
