#ifndef __FE_DEGAUSS
#define __FE_DEGAUSS

#include <iostream>
#include <vector>
#include <unordered_map>
#include <string>
#include <array>

#include <boost/variant.hpp>

// #include "experim.h"
#include "midas.h"
#include "degauss.hxx"
#include "msystem.h"
#include "mcstd.h"


using namespace std;
using namespace boost;


// tolerance for voltages reaching target
#define TOLERANCE 0.05


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
// INT frontend_loop();

INT degauss_readout(char* pevent, INT off);
void degauss_activate(HNDLE hDB, HNDLE key, void* info);


EQUIPMENT equipment[] = {
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
    250,
    0,
    0,
    0,
    "","","",
  },
  degauss_readout,
  NULL,
  NULL
};

}


static const char* SET_STR = 
"[Settings]\n"
"run = BOOL : 0\n"
"nsteps = CHAR : 8\n"
"targets = FLOAT[6]\n"
"  [0] 0\n"
"  [1] 0\n"
"  [2] 0\n"
"  [3] 0\n"
"  [4] 0\n"
"  [5] 0\n";


static const char* VAR_STR = 
"[Variables]\n"
"running = BOOL : 0\n"
"stepn = CHAR : 8\n"
"targets = FLOAT[6]\n"
"  [0] 0\n"
"  [1] 0\n"
"  [2] 0\n"
"  [3] 0\n"
"  [4] 0\n"
"  [5] 0\n";


static const char
  *COIL_V_SET_KEY  = "/equipment/ptfwiener/settings/outputvoltage",
  *COIL_V_READ_KEY = "/equipment/ptfwiener/variables/sensevoltage";


/* Globals */


HNDLE
  hDB=0, hkeyclient=0;

char host_name[256] = {0}, exp_name[256] = {0};


unordered_map<string, HNDLE> SET_KEYS = {
  {"coil_targets", 0},
  {"run",          0},
  {"nsteps",       0},
};


unordered_map<string, HNDLE> VAR_KEYS = {
  {"coil_set", 0},
  {"running",  0},
  {"stepn",    0},
};


HNDLE COIL_READ = 0, COIL_WRITE = 0;


// helpful functions

namespace Degauss {


enum Keys {
  SetCoilTargets,
  SetRun,
  SetNsteps,
  VarCoilSet,
  VarRunning,
  VarStepn,
};


HNDLE get_key(Keys key);


enum Error {
  None,
  InvalidHandle,
  InvalidDb,
  KeyDoesNotExist,
  CouldNotCreateRecord,
  Unknown,
};


Error ensure_odb_keys(const HNDLE hDB);

}


#endif