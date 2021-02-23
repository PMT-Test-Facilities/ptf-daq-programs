#ifndef __FE_DEGAUSS
#define __FE_DEGAUSS

#include <iostream>
#include <vector>
#include <unordered_map>
#include <string>
#include <array>
#include <limits>

#include <boost/variant.hpp>
#include <boost/optional.hpp>

// #include "experim.h"
#include "midas.h"
#include "degauss.hxx"


using namespace std;
using namespace boost;


// tolerance for voltages reaching target
#define V_TOLERANCE 0.06
// lowest voltage we think that we can make reliably
#define V_RELIABLE_MARGIN 0.5


/*
 * Notes:
 * We use a couple of codes for setting voltages:
 *   - in settings, if a coil target is set to -1 it will be ignored
 *   - in variables, if a coil target is set to -1 it will be ignored, and if set to -2 it requires calculation
 */


// required by MIDAS
extern "C" {


const char
  *frontend_name = "feDegauss",
  *frontend_file_name = __FILE__;


BOOL frontend_call_loop = FALSE;


INT
  display_period      = 000,
  max_event_size      = 1000,
  max_event_size_frag = 5 * 1024 * 1024,
  event_buffer_size   = 10 * 1000;


INT frontend_init();
INT frontend_exit();
INT begin_of_run(INT run_number, char *error);
INT end_of_run(INT run_number, char *error);
INT pause_run(INT run_number, char *error);
INT resume_run(INT run_number, char *error);
INT frontend_loop();
INT poll_event(INT source, INT count, BOOL test);
INT interrupt_configure(INT cmd, INT source[], PTYPE adr);


INT degauss_readout(char* pevent, INT off);
void degauss_activate(HNDLE hDB, HNDLE key, void* info);


EQUIPMENT equipment[] = {{
  "Degauss",
  {
    1,
    0,
    "USER",
    EQ_PERIODIC,
    1,
    "FIXED",
    TRUE,
    RO_ALWAYS,
    2000,
    0,
    0,
    0,
    "","","",
  },
  degauss_readout,
  NULL,
  NULL
}};


} // end extern C


static const char* SET_STR = 
"run = BOOL : 0\n"
"nsteps = BYTE : 8\n"
"targets = DOUBLE[6]\n"
"  [0] 0\n"
"  [1] 0\n"
"  [2] 0\n"
"  [3] 0\n"
"  [4] 0\n"
"  [5] 0\n";


static const char* VAR_STR = 
"running = BOOL : 0\n"
"stepn = BYTE : 0\n"
"targets = DOUBLE[6]\n"
"  [0] 0\n"
"  [1] 0\n"
"  [2] 0\n"
"  [3] 0\n"
"  [4] 0\n"
"  [5] 0\n"
"status = BYTE : 0\n";


enum VarStatus {
  Ok = 0,
  CoilIsNan = 1,
  CoilIsOob = 2,
  CoilNearOob = 3,
  FindingSetsFail = 4
};


static const char
  *COIL_V_SET_KEY  = "/equipment/ptfwiener/settings/outputvoltage",
  *COIL_V_READ_KEY = "/equipment/ptfwiener/variables/sensevoltage";


/* Globals */


#define RET_OK   1
#define RET_FAIL 0


HNDLE HDB=0;

namespace Keys {
  // setting keys
  namespace Set {
    HNDLE
      targets = 0,
      run     = 0,
      nsteps  = 0;
  }
  // variable keys
  namespace Var {
    HNDLE
      targets = 0,
      running = 0,
      stepn   = 0,
      status  = 0;
  }
  // other
  HNDLE
    coil_read  = 0,
    coil_write = 0;
}

// helpful functions


enum Error {
  None,
  CouldNotFindKey,
  CouldNotCreateRecord
};


Error ensure_odb_keys(const HNDLE hDB);


template<typename T, size_t N>
bool all_below_threshold(const array<T, N>& vs, const array<T, N>& targets) {
  for (size_t i = 0; i < N; ++i) {
    if (fabs(vs[i] - targets[i]) > V_TOLERANCE) {
      return false;
    }
  }
  return true;
}


#endif
