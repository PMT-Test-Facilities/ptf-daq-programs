/********************************************************************\
Name:         feScan.c

Program to coordinate the movement of the gantry through a scan
and the reading out of DVM or PMT.

What does program do?

1) Calls frontend_init once, to setup the callbacks to ODB

2) Once a run is started, method begin_run is called. This method moves
the motor back to its nominal setting (for the start of the sequence).
Then it calls the method move_next_position, which will make the first
move.

3) The methods frontend_loop and read_scan are then periodically called
by the main front-end loop (frontend_loop is called more often).

-> Once a move is finished (move_next_position is done), the next read_scan
will execute a readout of DVM and create a bank with the current position
and reading.

-> Once a reading is finished, the next frontend_loop will initiate
the next move_next_position.

\********************************************************************/

#define FAILURE 0

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include "midas.h"
//#include "experim_new.h"
#include "ScanSequence.hxx"
#include <vector>
#include "mfe.h"

/* make frontend functions callable from the C framework */

/*-- Frontend globals ------------------------------------------------*/

/* odb handles */
extern HNDLE hDB;
HNDLE hFS = 0;
HNDLE hMDestination = 0, hMMove = 0;
HNDLE hMComplexDestination = 0, hMComplexMove = 0;
HNDLE hMoveVariables = 0;
HNDLE hMoveSettings = 0;
HNDLE hMoveDestination = 0;
HNDLE hMoveStart = 0;
HNDLE hPhidgetVars0 = 0, hPhidgetVars1 = 0;
HNDLE hPtfWiener = 0;
HNDLE hMotors00 = 0, hMotors01 = 0;
HNDLE hNScanPoints, hCurrentPoint = 0;
//HNDLE   hReInitialize = 0;
// HNDLE   hIniting = 0;
//HndlehDVMVariables =0;

extern int run_state;

char expt_name[32];

// TF: completely deprecate the "dd"-debugging system: using MDEBUG instead!
/* These variables can be changed to control the level of debugging messages.
They should really be collapsed into one variable placed in the ODB
INT     dd = 1, ddd = 1, dddd=1, ddddd = 1;


ddd frontend_loop, begin_of_run
dddd indicates if running in cycle in frontend loop
dd = scan info
*/

// flag indicates that we are currently moving; this ensures that frontend_loop doesn't
// start another move at the same time.
BOOL gbl_making_move;
// flag indicates that move is finished and we are waiting for next measurement.
BOOL gbl_waiting_measurement = FALSE;

//checks if initializing
// BOOL    gbl_initing = FALSE;

INT gbl_run_number;


// NEW : generic
// Total number of points we will move to
INT gbl_total_number_points = 0;

// List of points
//float **points;
std::vector <std::vector<double> > points;


// allocated memory for list of points
INT max_size = 0;

// Current point we are moving to.
INT gbl_current_point = 0;

// Called begin_of_run; make sure this is called before any moves happen.
BOOL gbl_called_BOR = FALSE;

// Position of gantries, according to feMove front-end.
float gGantryPositions[10];

//NEW:
// Position of limits, according to feMove front-end.
float gGantryLimits[10];

// Destination of gantries, according to feMove front-end.
float gGantryDestinations[10];

// DVM readings, according to fedvm front-end.
//double   gDVMReadings[22];  //OLD : HallProbe --> Phidgets.

// Phidget readings
double gPhidget00[10];
double gPhidget01[10];

// Helmholtz coil readings from Wiener power crate
float gCoilCurrent[40];
float gCoilVoltage[40];

// PMT readings captured by fevme, not here

INT gbl_FREQ_n;                /* current cycle # in X scan */
DWORD gbl_CYCLE_N;               /* current valid cycle */
DWORD gbl_SCYCLE_N;              /* current super cycle */
DWORD gbl_SCAN_N;                /* current scan */
DWORD gbl_watchdog_timeout;      /* midas watchdog timeout */
BOOL gbl_watchdog_flag = TRUE;         /* midas watchdog flag */
BOOL gbl_dachshund_flag = FALSE;  /* true when a long watchdog timeout is set */
BOOL hot_rereference = FALSE, lhot_rereference = TRUE;
BOOL gbl_transition_requested = FALSE;
BOOL gbl_waiting_for_run_stop = FALSE; /* global flag to prevent restarting cycle on error */
BOOL gbl_bank_BONM_created = FALSE;
BOOL gbl_bank_EOM_created = TRUE;
BOOL bad_destination = FALSE;

DWORD time_move_next_position;
DWORD time_Start_motors;
DWORD time_Done_cycle;
DWORD time_Start_read;
DWORD time_Done_read;

SCAN_SETTINGS fs;
ScanSequence scan_seq;

/* Scan Type Flags  */
BOOL first_time = FALSE;  /* used by routine move_next_position to know if this is begin_of_run */
BOOL gbl_first_call = TRUE; /* used for deferred stop routine */
INT N_Scans_wanted;
BOOL hold_flag = FALSE; /* hold flag */
INT ncycle_sk_tol; /* number of cycles to skip AFTER returning in tolerance */
INT psleep = 0;

/* The frontend name (client name) as seen by other MIDAS clients   */
const char *frontend_name = "feSCAN";

/* The frontend file name, don't change it */
const char *frontend_file_name = __FILE__;

/* frontend_loop is called periodically if this variable is TRUE    */
BOOL frontend_call_loop = TRUE;

BOOL equipment_common_overwrite = FALSE;

/* a frontend status page is displayed with this frequency in ms    */
INT display_period = 000;

/* maximum event size produced by this frontend
   must be less than MAX_EVENT_SIZE (include/midas.h)
   which is 4MB or 0x400000 or 4e6.
*/
INT max_event_size = 100000;//10000;

/* buffer size to hold events */
INT event_buffer_size = 10 * 100000; //10*10000;

INT max_event_size_frag = 5 * 1024 * 1024; /* not planning to use this */

/*  Equipment indexes (constants) */
#define CYCLE                    0

/*-- Function declarations -----------------------------------------*/

INT frontend_init();

INT frontend_exit();

INT begin_of_run(INT run_number, char *error);

INT end_of_run(INT run_number, char *error);

INT pause_run(INT run_number, char *error);

INT resume_run(INT run_number, char *error);

INT frontend_loop();

INT poll_event(INT source, INT count, BOOL test);

extern void interrupt_routine(void);

INT scan_read(char *pevent, INT off);

INT move_next_position();
//INT set_scan_params(INT *ninc);

// NEW:
BOOL gen_scan_path();      // for preprogrammed cycli

/*-- Equipment list ------------------------------------------------*/
EQUIPMENT equipment[] = {

    {"Scan",      /* equipment name */
        {1, 1,                 /* event ID, trigger mask */
            "SYSTEM",             /* event buffer */
            EQ_PERIODIC,          /* equipment type */
            0,                    /* event source */
            "MIDAS",              /* format */
            TRUE,                 /* enabled */
            RO_RUNNING,           /* read when running */
            100,                  /* polling period (ms)*/
            0,                    /* stop run after this event limit */
            0,                    /* number of sub-event in the super event */
            1,                    /* log history */
            "", "", ""},
        scan_read,   /* readout routine */
        NULL, NULL, NULL,
    },

    {""}
};




/*-- Interrupt configuration ---------------------------------------*/
INT interrupt_configure(INT cmd, INT source, POINTER_T adr)
{
  switch (cmd) {
  case CMD_INTERRUPT_ENABLE:
    break;
  case CMD_INTERRUPT_DISABLE:
    break;
  case CMD_INTERRUPT_ATTACH:
    break;
  case CMD_INTERRUPT_DETACH:
    break;
  }
  return SUCCESS;
}




// Get the current Settings parameters from the 
INT get_settings_parameters(){

  INT size = sizeof(fs);
  INT status = db_get_record(hDB, hFS, &fs, &size, 0);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "frontend_init", "cannot retrieve %s (size of fs=%d). Try again?", "/Equipment/Scan/Settings/",
	   size);
	
    return DB_NO_ACCESS;    
  }

  printf("Successfully got scan settings; scan type: %i\n",fs.scan_type);
  return 1;
}


/*-- Frontend Init -------------------------------------------------
  called by mfe.c                                */
INT frontend_init() {
  //const char** scan_settings_str;
  SCAN_SETTINGS_STR(scan_settings_str);

  //Declare Variables
  BOOL watchdog_flag;
  INT status, rstate, size;

  gbl_making_move = FALSE;

  /* get the experiment name */
  size = sizeof(expt_name);
  status = db_get_value(hDB, 0, "/experiment/Name", &expt_name, &size, TID_STRING, FALSE);
  cm_msg(MDEBUG, "frontend_init", "Code for experiment %s now running ... ", expt_name);

  /* get basic handle for experiment ODB */
  status = cm_get_experiment_database(&hDB, NULL);
  if (status != CM_SUCCESS) {
    cm_msg(MERROR, "frontend_init", "Not connected to experiment");
    return CM_UNDEF_EXP;
  }

  /* check for current run state; if not stopped, stop it */
  size = sizeof(rstate);
  status = db_get_value(hDB, 0, "/runinfo/State", &rstate, &size, TID_INT, FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "frontend_init", "cannot GET /runinfo/State");
    return FE_ERR_ODB;
  }
  if (rstate != STATE_STOPPED) {
    cm_msg(MERROR, "frontend_init",
           "!!! Run In Progress: Stop the run, shutdown and restart this program !!!");
    return (FE_ERR_HW);
  }

  /* get handles of equipment records   */
  status = db_find_key(hDB, 0, "/Equipment/Scan/Settings/", &hFS);
  if (status != DB_SUCCESS) {
    cm_msg(MINFO, "frontend_init", "Record %s  does not exist. It will be created.", "/Equipment/Scan/Settings/");
    status = db_create_record(hDB, 0, "/Equipment/Scan/Settings/", strcomb(scan_settings_str));  //strcomb : see odb.c
  }
  /* Get Scan  settings */
  status = get_settings_parameters();
  if(status != 1) return status;

  printf("Scan init()\n");
  scan_seq.Init(fs, gGantryLimits);


  // Obtain the ODB keys for various variables
  // Note: when adding or removing variables to/from this list change the variable num_entires to change the size of the arrays used
  const int num_entries = 11;
  char str[num_entries][128];
  HNDLE *ODB_Handles[num_entries];
  //Move settings
  sprintf(str[0], "/Equipment/Move/Variables/");
  ODB_Handles[0] = &hMoveVariables;
  sprintf(str[1], "/Equipment/Move/Control/Destination");
  ODB_Handles[1] = &hMoveDestination;
  sprintf(str[2], "/Equipment/Move/Settings/");
  ODB_Handles[2] = &hMoveSettings;
  sprintf(str[3], "/Equipment/Move/Control/Start Move");
  ODB_Handles[3] = &hMoveStart;
  //Phidget Vars
  sprintf(str[4], "/Equipment/Phidget00/Variables/");
  ODB_Handles[4] = &hPhidgetVars0;
  sprintf(str[5], "/Equipment/Phidget01/Variables/");
  ODB_Handles[5] = &hPhidgetVars1;
  //Wiener Power Supply
  sprintf(str[6], "/Equipment/PtfWiener/Variables/");
  ODB_Handles[6] = &hPtfWiener;
  //Motors
  sprintf(str[7], "/Equipment/Motors00/Settings/TurnMotorsOff");
  ODB_Handles[7] = &hMotors00;
  sprintf(str[8], "/Equipment/Motors01/Settings/TurnMotorsOff");
  ODB_Handles[8] = &hMotors01;
  // Scan Variables
  sprintf(str[9], "/Equipment/Scan/Variables/NPoints");
  ODB_Handles[9] = &hNScanPoints;
  sprintf(str[10], "/Equipment/Scan/Variables/Current Point");
  ODB_Handles[10] = &hCurrentPoint;
  //sprintf(str[11], "/Equipment/Move/Control/ReInitialize");
  //ODB_Handles[11] = &hReInitialize;
  //sprintf(str[12], "/Equipment/Move/Variables/Initializing");
  //ODB_Handles[12] = &hIniting;
  // Get the above ODB Keys and produce error if unsuccessful
  int i;
  for (i = 0; i < num_entries; i++) {
    status = db_find_key(hDB, 0, str[i], ODB_Handles[i]);
    if (status != DB_SUCCESS) {
      cm_msg(MERROR, "frontend_init", "cannot get key for %s", str[i]);
      return DB_NO_ACCESS;
    }
  }

  //Reset watchdog and Alarms
  cm_get_watchdog_params(&watchdog_flag, &gbl_watchdog_timeout);
  cm_msg(MINFO, "frontend_init", "Current watchdog parameters are gbl_watchdog_timeout=%d; watchdog_flag=%d",
         gbl_watchdog_timeout, watchdog_flag);

  status = al_reset_alarm(NULL); /* reset all alarms */
  if (status != CM_SUCCESS)
    cm_msg(MINFO, "frontend_init", "Problem trying to reset alarms automatically (%d)", status);
  else
    cm_msg(MINFO, "frontend_init", "Cleared all alarms on main status page");

  if (status == CM_SUCCESS)
    cm_msg(MINFO, "frontend_init", "End of routine frontend_init. Program is READY");
  return status;
}



    time_at_start_of_check = ss_millitime();
    while ((phidget_Values_Old[0] == phidget_Values_Now[0]) && (phidget_Values_Old[1] == phidget_Values_Now[1]) &&
           (phidget_Values_Old[2] == phidget_Values_Now[2]) && (phidget_Values_Old[3] == phidget_Values_Now[3]) &&
           (phidget_Values_Old[4] == phidget_Values_Now[4]) && (phidget_Values_Old[5] == phidget_Values_Now[5]) &&
           (phidget_Values_Old[6] == phidget_Values_Now[6]) && (phidget_Values_Old[7] == phidget_Values_Now[7]) &&
           (phidget_Values_Old[8] == phidget_Values_Now[8]) && (phidget_Values_Old[9] == phidget_Values_Now[9])) {

      db_get_value(hDB, hPhidgetVars0, "PH00", &phidget_Values_Now, &size_of_array, TID_DOUBLE, FALSE);

      // If 5 seconds pass without any of the values changing assume that the phidgets are not connected properly
      if (ss_millitime() - time_at_start_of_check > 1000 * 5) {
        printf("The readings from Phidget0 are not changing so it is assumed that Phidget0 is not working\n");
        return DB_NO_ACCESS;  //TODO : only comment if know that Phidget IS unplugged!
        //break;
      }

      ss_sleep(1000); //wait a sec
    }

    //Check that the phidget1 is  working.
    db_get_value(hDB, hPhidgetVars1, "PH01", &phidget_Values_Old, &size_of_array, TID_DOUBLE, FALSE);
    db_get_value(hDB, hPhidgetVars1, "PH01", &phidget_Values_Now, &size_of_array, TID_DOUBLE, FALSE);

    time_at_start_of_check = ss_millitime();
    while ((phidget_Values_Old[0] == phidget_Values_Now[0]) && (phidget_Values_Old[1] == phidget_Values_Now[1]) &&
           (phidget_Values_Old[2] == phidget_Values_Now[2]) && (phidget_Values_Old[3] == phidget_Values_Now[3]) &&
           (phidget_Values_Old[4] == phidget_Values_Now[4]) && (phidget_Values_Old[5] == phidget_Values_Now[5]) &&
           (phidget_Values_Old[6] == phidget_Values_Now[6]) && (phidget_Values_Old[7] == phidget_Values_Now[7]) &&
           (phidget_Values_Old[8] == phidget_Values_Now[8]) && (phidget_Values_Old[9] == phidget_Values_Now[9])) {

      db_get_value(hDB, hPhidgetVars1, "PH01", &phidget_Values_Now, &size_of_array, TID_DOUBLE, FALSE);

      // If 5 seconds pass without any of the values changing assume that the phidgets are not connected properly
      if (ss_millitime() - time_at_start_of_check > 1000 * 5) {
        cm_msg(MERROR, "begin_of_run",
               "The readings from Phidget1 are not changing so it is assumed that Phidget1 is not working");
        return DB_NO_ACCESS; //TODO FEB 28 2015: reset !!!
        //break;
      }
    }
    cm_msg(MINFO, "begin_of_run", "Both phidgets are working properly");
  }


  status = gen_scan_path();
  int i;
  for (i = 0; i < gbl_total_number_points; i++) {
    // DEBUG : don't write all scan points to midas.log
    printf("Point %i Gantry0: %.4F %.4F %.4F %.4F %.4F, Gantry1: %.4F %.4F %.4F %.4F %.4F\n", i, points[i][0],
           points[i][1], points[i][2], points[i][3], points[i][4], points[i][5], points[i][6], points[i][7],
           points[i][8], points[i][9]);
    //cm_msg(MINFO,"begin_of_run","Point %i Gantry0: %.4F %.4F %.4F %.4F %.4F, Gantry1: %.4F %.4F %.4F %.4F %.4F\n",i,points[i][0],points[i][1],points[i][2],points[i][3],points[i][4],points[i][5],points[i][6],points[i][7],points[i][8],points[i][9]);
  }

  // replace with proper error later
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "Scan parameters invalid. Path Generation failed");
    return DB_NO_ACCESS;
  }


  ss_sleep(1000); /* sleep before starting loop*/

  /* Start cycle */
  first_time = TRUE;
  gbl_current_point = 0;

  // We are starting to move; set flag to ensure that nothing else tries to start us moving.
  // TF and TL: not making a first move because can take longer (with low velocity and far away point)
  // than the cm_transition timeout (OBD /Experiment/Transition timeout) (switch states between RUN OFF,
  // RUN START, RUNNING, RUN STOP) => No First move!!

  gbl_called_BOR = TRUE;

  cm_msg(MDEBUG, "begin_of_run", "end of begin_of_run");
  return CM_SUCCESS;
}

/*-- Generate path for scan-----------------------------------------*/
BOOL gen_scan_path(void) {

  //Fills points (passed by ref) with the scan sequence specified in the ODB
  // Insert path into global variable/array
  gbl_total_number_points = scan_seq.GeneratePath(points);

  // Check path size
  if (gbl_total_number_points > max_event_size) {
    cm_msg(MERROR, "gen_scan_path", "Path contains too many points (%i points). Max event size = %i.",
           gbl_total_number_points, max_event_size);
    return 0;
  } else {
    cm_msg(MINFO, "gen_scan_path", "Path contains %i points. DONE!", gbl_total_number_points);
    db_set_data(hDB, hNScanPoints, &gbl_total_number_points, sizeof(INT), 1, TID_INT);
    return 1;
  }
}
