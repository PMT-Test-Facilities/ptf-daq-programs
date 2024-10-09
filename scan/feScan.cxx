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
#include <iostream>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include "midas.h"
//#include "experim_new.h" 
#include "ScanSequence.hxx"
#include <vector>
#include "mfe.h"




#include "msystem.h"
#include "mcstd.h"

#include <unistd.h>
#include <string.h>
#include "TPathCalculator.hxx" //Rika: See typedef for XYPoint, XYPolygon, XYLine here.                                                                                              
#include "TRotationCalculator.hxx"
#include "TGantryConfigCalculator.hxx" //Rika (27Mar2017): Added as a class shared between feMove & TRotationCalculator.                                                             

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
HNDLE hMotors00_output_control = 0;
HNDLE hNScanPoints, hCurrentPoint = 0;
HNDLE   hReInitialize = 0;
HNDLE   hIniting = 0;
HNDLE current_read, voltage_read;
//HndlehDVMVariables =0;

extern int run_state;

char expt_name[32];

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

double getVoltage[6];
double getCurrent[6];

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

int coil_idx(int coil) {
  switch(coil) {
    case 0:
      return 8;
    case 1:
      return 9;
    case 2:
      return 2;
    case 3:
      return 6;
    case 4:
      return 1;
    case 5:
      return 0;
    default:
      throw "Go away warnings, I got all the cases";
  }
}

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


//---------remove------------------------
void printScanSettings(const SCAN_SETTINGS& fs) {
  cm_msg(MINFO, "frontend_init", "Scan type %d", fs.scan_type);
  
  
  cm_msg(MINFO, "frontend_init",  "azi_start  %f", fs.spin_scan_par.azi_start);
  cm_msg(MINFO, "frontend_init",  "zen_start  %f", fs.spin_scan_par.zen_start);
  cm_msg(MINFO, "frontend_init",  "azi_step  %f", fs.spin_scan_par.azi_step);
  cm_msg(MINFO, "frontend_init",  "zen_step  %f", fs.spin_scan_par.zen_step);
  cm_msg(MINFO, "frontend_init",  "azi_dist  %f", fs.spin_scan_par.azi_distance);
  cm_msg(MINFO, "frontend_init",  "zen_dist  %f", fs.spin_scan_par.zen_distance);
  cm_msg(MINFO, "frontend_init",  "pos_x  %f" , fs.spin_scan_par.init_x);
  cm_msg(MINFO, "frontend_init",  "pos y  %f", fs.spin_scan_par.init_y);
  cm_msg(MINFO, "frontend_init",  "posz  %f", fs.spin_scan_par.init_z);

  cm_msg(MINFO, "frontend_init",  "spin scan dir %d", fs.patch_scan_par.scan_dir);
  cm_msg(MINFO, "frontend_init",  "patch scan pmt x %f", fs.patch_scan_par.pmt_x);
  cm_msg(MINFO, "frontend_init",  "patch scan pmt y %f", fs.patch_scan_par.pmt_y);
  cm_msg(MINFO, "frontend_init",  "patch scan pmt z %f", fs.patch_scan_par.pmt_tip_z);
  cm_msg(MINFO, "frontend_init",  "patch scan pmt angle %f", fs.patch_scan_par.pmt_angle_center);

  cm_msg(MINFO, "frontend_init",  "patch scan theta %f", fs.tilt_par.theta);
  cm_msg(MINFO, "frontend_init",  "patch scan phi  %f", fs.tilt_par.phi);
  cm_msg(MINFO, "frontend_init",  "patch scan step  %f", fs.tilt_par.step);
  cm_msg(MINFO, "frontend_init",  "patch scan dir %d", fs.tilt_par.scan_dir);

  
}
//remove---------------------------------

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
  //fs.scan_type = 1;
  printScanSettings(fs); //Anubhav's edit
  
  if (status != DB_SUCCESS) {
    // printScanSettings(fs);
    cm_msg(MERROR, "frontend_init", "cannot retrieve %s (size of fs=%d). Try again?", "/Equipment/Scan/Settings/",
	   size);
	
    return DB_NO_ACCESS;    
  }
  //fs.scan_type = 1;

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
  status = get_settings_parameters(); //commented by Anubhav
  if(status != 1) return status; //commented by Anubhav
  //printScanSettings(fs);
  printf("Scan init()\n");
  scan_seq.Init(fs, gGantryLimits);


  // Obtain the ODB keys for various variables
  // Note: when adding or removing variables to/from this list change the variable num_entires to change the size of the arrays used
  const int num_entries = 15;
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
  sprintf(str[5], "/Equipment/Phidget03/Variables/");
  ODB_Handles[5] = &hPhidgetVars1;
  //Wiener Power Supply
  sprintf(str[6], "/Equipment/PtfWiener/Variables/");
  ODB_Handles[6] = &hPtfWiener;
  //Motors
  sprintf(str[6], "/Equipment/Motors00/Settings/TurnMotorsOff");
  ODB_Handles[6] = &hMotors00;
  sprintf(str[7], "/Equipment/Motors01/Settings/TurnMotorsOff");
  ODB_Handles[7] = &hMotors01;
  // Scan Variables
  sprintf(str[8], "/Equipment/Scan/Variables/NPoints");
  ODB_Handles[8] = &hNScanPoints;
  sprintf(str[9], "/Equipment/Scan/Variables/Current Point");
  ODB_Handles[9] = &hCurrentPoint;
  sprintf(str[10], "/Equipment/Move/Control/ReInitialize");
  ODB_Handles[10] = &hReInitialize;
  sprintf(str[11], "/Equipment/Move/Variables/Initializing");
  ODB_Handles[11] = &hIniting;
  sprintf(str[12], "/Equipment/Motors00/Settings/DigitalOut2");
  ODB_Handles[12] = &hMotors00_output_control;

  sprintf(str[13], "/Equipment/PtfWiener/Variables/current");
  ODB_Handles[13] = &current_read;
  sprintf(str[14], "/Equipment/PtfWiener/Variables/sensevoltage");
  ODB_Handles[14] = &voltage_read;

  // Get the above ODB Keys and produce error if unsuccessful
  int i;
  for (i = 0; i < num_entries; i++) {
    status = db_find_key(hDB, 0, str[i], ODB_Handles[i]);
    if (status != DB_SUCCESS) {
      cm_msg(MERROR, "frontend_init", "cannot get key for %s", str[i]);
      return DB_NO_ACCESS;
    }
  }

  
  //gbl_total_number_points = scan_seq.GeneratePath(points);

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


/*-- Frontend Exit -------------------------------------------------
  called by mfe.c                                */
INT frontend_exit() {

  //In case the gbl_current_point does not get reset due a non clean exit:
  gbl_current_point = 0;
  db_set_data(hDB, hCurrentPoint, &gbl_current_point, sizeof(INT), 1, TID_INT);
  gbl_total_number_points = 0;
  db_set_data(hDB, hNScanPoints, &gbl_total_number_points, sizeof(INT), 1, TID_INT);
  //  Kexit();
  cm_msg(MINFO, "frontend_exit", "mfe procedure exiting.");
  return CM_SUCCESS;
}

/*-- Frontend Loop -------------------------------------------------
  called periodically by a loop in mfe.c
  - Main function for starting a new move, if we are running and
  the measurement for the last move has finished..
  BK: scan_read is called after Frontend Loop completes. This is why at certain spots you see return SUCCESS. This allows scan_read to be called and complete various tasks
  ** indicates work in progress
*/
INT frontend_loop() {
  INT status;
  char str[128];
  //INT size; //**

  //size = sizeof(gbl_initing);  //**

  //status = db_get_value(hDB, 0, "/Equipment/Move/Variables/Initializing", &gbl_initing, &size, TID_BOOL, FALSE); //**
  // if (status != DB_SUCCESS){
  //  cm_msg(MERROR, "begin_of_run", "cannot check if initializing");
  //  return DB_NO_ACCESS;
  //}

  // Only want to start checking if the begin_of_run has been called.
  if (!gbl_called_BOR) return SUCCESS;

  if (run_state == STATE_RUNNING) {
    /*       RUNNING
	     check if cycle is finished AND histo update being done */
    //if(gbl_initing) return SUCCESS;  //**
    if (!gbl_making_move) { /* not in cycle, wait until equipment has run */
      if (gbl_waiting_measurement) {
        return SUCCESS; /* processing has not yet finished i.e. all equipments have not run */
      }
      cm_msg(MDEBUG, "frontend_loop", "running: IN_CYCLE is false");

      // Terminate the sequence once we have finished last move.
      if (gbl_current_point >= gbl_total_number_points) {
        cm_msg(MINFO, "frontend_loop", "Stopping run after all points are done. Resetting current point number.");
        gbl_current_point = 0;
        db_set_data(hDB, hCurrentPoint, &gbl_current_point, sizeof(INT), 1, TID_INT);
        status = cm_transition(TR_STOP, 0, str, sizeof(str), TR_SYNC, 0);   //TF: new MIDAS: changed SYNC to TR_SYNC
        return status;
      }

      //Tilt Check Periodically:  NEEDS LOTS OF DEBUGGING

      /*if(gbl_current_point % 50 == 0 && gbl_current_point != 0){
	  status = db_set_data(hDB,hReInitialize,(void *)TRUE,sizeof(BOOL),1,TID_BOOL);
      
	  if (status != DB_SUCCESS){
	      cm_msg(MERROR, "begin_of_run", "cannot reinitialize for tilt check");
	      return DB_NO_ACCESS;
	  }
      }
      */

      cm_msg(MDEBUG, "frontend_loop", "Starting next move");

      /* Start new move */
      // We are starting to move; set flag to ensure that nothing else tries to start us moving.

      //TF: this line FAILS into infinite loop (with scan_read where BONM_created will NEVER get set to TRUE so never will reach next move) IF it's a bad dest!! Hence, only do if not bad_dest
      // BONM is a bank that is the MIDAS file that is there for analysis purposes. BONM and EOM can be used to tell if data was taken while the gantries were moving or not. BONM is created in scan_read()
      if (gbl_bank_BONM_created == FALSE && bad_destination == FALSE) {
        return SUCCESS;
      }

      gbl_making_move = TRUE;
      time_move_next_position = ss_millitime();
      status = move_next_position();
      gbl_bank_BONM_created = FALSE;

      if (status != SUCCESS) {
        sprintf(str, "Stopping run because of error in move_next_position");
        status = cm_transition(TR_STOP, 0, str, sizeof(str), TR_SYNC, 0);
        return status;
      }

    } /* end of not in cycle */
    else { //we are already moving; (i.e. gbl_making_move is TRUE) nothing to do
      cm_msg(MDEBUG, "frontend_loop", "Running in cycle: nothing to do; exit");
      return SUCCESS;
    }
  } //end of RUNNING 
  // else NOT running,  nothing to do 
  return SUCCESS;
}

/*-- Begin of Run sequence ------------------------------------------
  called by mfe.c                        */
INT begin_of_run(INT run_number, char *error)
/*
  - start acq
*/
{
  //declare variables
  BOOL moving, loggerWrite, loggerActive, wantToWrite;
  INT status, size;

  /* Get current Scan  settings */
  status = get_settings_parameters(); //commented by Anubhav
  if(status != 1) return status; //commented by Anubhav

  //variables used to check that phidgets are connected and working
  BOOL Check_Phidgets = FALSE; // change this to false to skip phidget check. This should only be done if you are running a scan without using the phidgets
  double phidget_Values_Old[10];
  double phidget_Values_Now[10];
  int size_of_array = sizeof(phidget_Values_Old);
  DWORD time_at_start_of_check;

  gbl_transition_requested = FALSE; /* used for deferred transition */
  gbl_waiting_for_run_stop = FALSE; /* unrecoverable error */
  gbl_run_number = run_number;
  gbl_making_move = FALSE;
  gbl_first_call = TRUE; /* for prestop */

  cm_msg(MINFO, "begin_of_run", "Start begin of run");

  // CHECK THE LOGGER IS ON if we want to write data
  /* Get Logger  settings */
  size = sizeof(loggerWrite);
  status = db_get_value(hDB, 0, "/Logger/Write data", &loggerWrite, &size, TID_BOOL, FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "Record /Logger/Write data  does not exist.");
  }

  size = sizeof(loggerActive);
  status = db_get_value(hDB, 0, "/Logger/Channels/0/Settings/Active", &loggerActive, &size, TID_BOOL, FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "Record /Logger/Channels/0/Settings/Active does not exist.");
  }

  size = sizeof(wantToWrite);
  status = db_get_value(hDB, 0, "/Experiment/Edit on start/Write data", &wantToWrite, &size, TID_BOOL, FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "Record /Experiment/Edit on start/Write data  does not exist.");
  }

  if (wantToWrite && (!loggerActive || !loggerWrite)) {
    cm_msg(MERROR, "begin_of_run",
           "You want to write MIDAS output files, but logger is not properly set. \n Check whether Logger is Active and Writing Data.\n I'm saving you a lot of time here!\n");
    return CM_SET_ERROR;
  }

  // Ensure that we are not already running.
  size = sizeof(moving);
  status = db_get_value(hDB, hMoveVariables, "Moving", &moving, &size, TID_BOOL, FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "cannot get value for Moving");
    return DB_NO_ACCESS;
  }
  if (moving) {
    cm_msg(MERROR, "begin_of_run", "Gantry already moving.  Panic!");
    return DB_NO_ACCESS;
  }

  // Get the initial position of the gantry (for logging info).
  size = sizeof(gGantryPositions);
  status = db_get_value(hDB, hMoveVariables, "Position", &gGantryPositions, &size, TID_FLOAT, FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "cannot get value for Position");
    return DB_NO_ACCESS;
  }

  // Get the gantry limits for checking parameters.
  size = sizeof(gGantryLimits);
  status = db_get_value(hDB, hMoveSettings, "Limit Positions", &gGantryLimits, &size, TID_FLOAT, FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "cannot get value for Limits");
    return DB_NO_ACCESS;
  }


  //checks that the phidgets are connected by checking that the values are fluctuating. When the phidgets are unplugged the ODB values are constant with no fluctuation. 
  // TF: Yes, but should be true for ALL values, some values *can* stay the same for 5 sec.
  if (Check_Phidgets) {

    size_of_array = sizeof(phidget_Values_Old);
    db_get_value(hDB, hPhidgetVars0, "PH03", &phidget_Values_Old, &size_of_array, TID_DOUBLE, FALSE);
    db_get_value(hDB, hPhidgetVars0, "PH03", &phidget_Values_Now, &size_of_array, TID_DOUBLE, FALSE);

    //Check that phidget0 is working. The phidget is working if the values change over a set period of time. When a phidget is unplugged there is no fluctuation in the values.
    time_at_start_of_check = ss_millitime();
    while ((phidget_Values_Old[0] == phidget_Values_Now[0]) && (phidget_Values_Old[1] == phidget_Values_Now[1]) &&
           (phidget_Values_Old[2] == phidget_Values_Now[2]) && (phidget_Values_Old[3] == phidget_Values_Now[3]) &&
           (phidget_Values_Old[4] == phidget_Values_Now[4]) && (phidget_Values_Old[5] == phidget_Values_Now[5]) &&
           (phidget_Values_Old[6] == phidget_Values_Now[6]) && (phidget_Values_Old[7] == phidget_Values_Now[7]) &&
           (phidget_Values_Old[8] == phidget_Values_Now[8]) && (phidget_Values_Old[9] == phidget_Values_Now[9])) {

      db_get_value(hDB, hPhidgetVars0, "PH03", &phidget_Values_Now, &size_of_array, TID_DOUBLE, FALSE);

      // If 5 seconds pass without any of the values changing assume that the phidgets are not connected properly
      if (ss_millitime() - time_at_start_of_check > 1000 * 5) {
        printf("The readings from Phidget0 are not changing so it is assumed that Phidget0 is not working\n");
        return DB_NO_ACCESS;  //TODO : only comment if know that Phidget IS unplugged!
        //break;
      }

      //ss_sleep(1000);
      usleep(100000); //wait a sec
      //sleep(1);
    }

    //Check that the phidget1 is  working.
    db_get_value(hDB, hPhidgetVars1, "PH03", &phidget_Values_Old, &size_of_array, TID_DOUBLE, FALSE);
    db_get_value(hDB, hPhidgetVars1, "PH03", &phidget_Values_Now, &size_of_array, TID_DOUBLE, FALSE);

    time_at_start_of_check = ss_millitime();
    while ((phidget_Values_Old[0] == phidget_Values_Now[0]) && (phidget_Values_Old[1] == phidget_Values_Now[1]) &&
           (phidget_Values_Old[2] == phidget_Values_Now[2]) && (phidget_Values_Old[3] == phidget_Values_Now[3]) &&
           (phidget_Values_Old[4] == phidget_Values_Now[4]) && (phidget_Values_Old[5] == phidget_Values_Now[5]) &&
           (phidget_Values_Old[6] == phidget_Values_Now[6]) && (phidget_Values_Old[7] == phidget_Values_Now[7]) &&
           (phidget_Values_Old[8] == phidget_Values_Now[8]) && (phidget_Values_Old[9] == phidget_Values_Now[9])) {

      db_get_value(hDB, hPhidgetVars1, "PH03", &phidget_Values_Now, &size_of_array, TID_DOUBLE, FALSE);

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
    cm_msg(MINFO,"begin_of_run","Point %i Gantry0: %.4F %.4F %.4F %.4F %.4F, Gantry1: %.4F %.4F %.4F %.4F %.4F\n",i,points[i][0],points[i][1],points[i][2],points[i][3],points[i][4],points[i][5],points[i][6],points[i][7],points[i][8],points[i][9]);
  }

  // replace with proper error later
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "Scan parameters invalid. Path Generation failed");
    return DB_NO_ACCESS;
  }

  printf("Test for sleep",ss_millitime);
  //ss_sleep(1000); /* sleep before starting loop*/
  usleep(1000000);
  //sleep(1);
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


/*-- Start cycle sequence ------------------------------------------*/
INT move_next_position(void) {
  INT status, size;

  cm_msg(MDEBUG, "move_next_position", "start of move_next_position");
  /* New cycle => start at bin 0 */

  if (gbl_current_point >= gbl_total_number_points) {
    cm_msg(MERROR, "move_next_position", "Not good! We've exceeded the number of moves allowed!");
    return DB_NO_ACCESS;
  }

  // Set the destination
  int i;
  for (i = 0; i < 10; i++)
    if (points[gbl_current_point][i] > -999)
      gGantryDestinations[i] = points[gbl_current_point][i];
    else
      gGantryDestinations[i] = gGantryPositions[i];


  //DEBUG:
  //for(i = 0; i< gbl_total_number_points; i++)
  //  printf("Point %i Gantry0: %.4F %.4F %.4F, Gantry1: %.4F %.4F %.4F \n",i,points[i][0],points[i][1],points[i][2],points[i][5],points[i][6],points[i][7]);

  printf("Now set several destinations at once %.3F %.3F %.3F %.3F %.3F   (%i)\n", gGantryDestinations[0],
         gGantryDestinations[1], gGantryDestinations[2], gGantryDestinations[3], gGantryDestinations[4],
         gbl_current_point);
  printf("And for gantry1 : %.3F %.3F %.3F %.3F %.3F   (%i)\n", gGantryDestinations[5], gGantryDestinations[6],
         gGantryDestinations[7], gGantryDestinations[8], gGantryDestinations[9], gbl_current_point);
  status = db_set_data(hDB, hMoveDestination, gGantryDestinations, sizeof(gGantryDestinations), 10, TID_FLOAT);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "cannot set Destinations");
    return DB_NO_ACCESS;
  }

  // Start the move!
  
  BOOL start_move[1] = {TRUE};
  printf("Moving!\n");
  status = db_set_data(hDB, hMoveStart, start_move, sizeof(start_move), 1, TID_BOOL);
  time_Start_motors = ss_millitime();
  printf("Moved!\n");
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "cannot set move");
    return DB_NO_ACCESS;
  }

  // Check that we are moving (why for half second first)
  //ss_sleep(500); //Why is this? Should it be longer or shorter? We should change to an optimal value
  usleep(500000);
  //sleep(0.5);
  printf("Check moving\n");
  BOOL moving;
  INT size_moving = sizeof(moving);
  status = db_get_value(hDB, hMoveVariables, "Moving", &moving, &size_moving, TID_BOOL, FALSE);
  //  BOOL moving;
  //size = sizeof(moving);
  //status = db_get_value(hDB,hMoveVariables,"Moving",&moving,sizeof(moving),TID_BOOL,FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "cannot get value for Moving");
    return DB_NO_ACCESS;
  }
  if (!moving)
    printf("Yikes, not moving! Not good!\n");
  printf("Check complete\n");
  BOOL completed;
  status = db_get_value(hDB, hMoveVariables, "Completed", &completed, &size_moving, TID_BOOL, FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "cannot get value for Completed");
    return DB_NO_ACCESS;
  }

  BOOL finished_moving = FALSE;
  if (completed && !moving)
    finished_moving = TRUE;
  DWORD time_start_move = ss_millitime();

  INT timeout = 60000 * 20; // 3 (5) minute timeout //JW 20191128 5->20

  // Loop which checks ODB until move to next point is finished
  while (!finished_moving) {

    //ss_sleep(50);
    usleep(50000);
    //sleep(0.5);
    status = db_get_value(hDB, hMoveVariables, "Moving", &moving, &size_moving, TID_BOOL, FALSE);
    status = db_get_value(hDB, hMoveVariables, "Completed", &completed, &size_moving, TID_BOOL, FALSE);


    if (completed && !moving)
      finished_moving = TRUE;

    DWORD time_now = ss_millitime();

    //DEBUG
    if((INT)(time_now-time_start_move)%10000 < 100){
        printf("yield %i ms\n",time_now-time_start_move);
    }

    if ((INT)(time_now - time_start_move) > timeout) {
      cm_msg(MERROR, "move_next_position", "We have waited %i ms, which is too long: Cannot finish move!",
             time_now - time_start_move);
      return DB_NO_ACCESS; //AJ: should break and go to the next position... not return an error code and stop the whole run
    }
  }

  // Finished moving to the current point.
  gbl_current_point++;
  db_set_data(hDB, hCurrentPoint, &gbl_current_point, sizeof(INT), 1, TID_INT);

  /* Cycle process sequencer
     NEW: only if moved to valid new position */

  size = sizeof(bad_destination);
  status = db_get_value(hDB, hMoveVariables, "Bad Destination", &bad_destination, &size, TID_BOOL, FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "move_next_position", "cannot get value for Bad Destination");
    return DB_NO_ACCESS;
  }
  cm_msg(MDEBUG, "move_next_position", "Bad dest: %i", bad_destination);
  if (!bad_destination) {


    //Switch off motors through hotlink with cd_galil to perform minimum noise measurement
    //Galil motors increase noise in PMT signals otherwise.
    
    // first disable relay which enables brake for tilt on gantry0
    BOOL relays_off[8];
    
    for(int i = 0; i < 8;i++){relays_off[i]=FALSE;}
    printf("Brake is On for tilt angle and relay is off! \n");
    db_set_data(hDB, hMotors00_output_control, &relays_off, 8*sizeof(BOOL), 8, TID_BOOL); 
    sleep(1.0); // wait for a second after turning the bake on before turning the motors off 
    // switch motors off 
    BOOL turn_off = TRUE;//Switching that one
    db_set_data(hDB, hMotors00, &turn_off, sizeof(BOOL), 1, TID_BOOL);
    db_set_data(hDB, hMotors01, &turn_off, sizeof(BOOL), 1, TID_BOOL);
    usleep(500000);
    printf("Motors are off!\n");
    //ss_sleep(50);
    //sleep(0.5);
    gbl_waiting_measurement = TRUE;
  }
  first_time = FALSE;

  /* Now we are ready to restart the hardware */
  /* Cycle started */
  gbl_making_move = FALSE;

  cm_msg(MDEBUG, "move_next_position", "%d  SCyc %d   go ", gbl_CYCLE_N, gbl_SCYCLE_N);

  time_Done_cycle = ss_millitime();
  return CM_SUCCESS;
}  /* end of move_next_position routine */


/*-- End of Run ----------------------------------------------------*/

INT end_of_run(INT run_number, char *error) {

  cm_msg(MINFO, "end_of_run", "Ending the run");
  points.clear();

  gbl_called_BOR = FALSE;

  return CM_SUCCESS;
}

/*-- Pause Run -----------------------------------------------------*/
INT pause_run(INT run_number, char *error) {
  cm_msg(MERROR, "pause_run", "This command DOES NOT WORK for this program");
  cm_msg(MERROR, "pause_run", " MIDAS PAUSE detected. RUN MUST BE STOPPED then restarted.");
  return CM_SUCCESS;
}

/*-- Resume Run ----------------------------------------------------*/
INT resume_run(INT run_number, char *error) {
  cm_msg(MERROR, "resume_run", "This command DOES NOT WORK for this program");
  cm_msg(MERROR, "resume_run", "MIDAS RESUME detected. RUN MUST BE STOPPED and restarted.");

  return CM_SUCCESS;
}

/*-- Trigger event routines ----------------------------------------*/
INT poll_event(INT source, INT count, BOOL test)
/* Polling routine for events. Returns TRUE if event
   is available. If test equals TRUE, don't return. The test
   flag is used to time the polling */
{
  int i;
  for (i = 0; i < count; i++) {
    if (!test)
      return FALSE;
  }
  return FALSE;
}

/*-- Event readout -------------------------------------------------*/
INT scan_read(char *pevent, INT off)
/* - periodic equipment reading the scalers sum over a cycle
   - generate event only when cycle has been completed  (through gbl_waiting_measurement)
   
*/
{
  double *pwdata, *pmagdata;
  //double current,time;
  INT read_status, size, status, m;

  cm_msg(MDEBUG, "scan_read", "gbl_waiting_measurement %d", gbl_waiting_measurement);

  if (gbl_waiting_measurement) {
    if (gbl_bank_EOM_created == FALSE) {

      //Do this once!
      bk_init(pevent);

      // for voltage and current field 0->5 relate to the Coil Numbers
      char bk_name[4] = "EOM";
      double *unused_pointer2;
      cm_msg(MDEBUG, "scan_read", "EOM Created");
      bk_create(pevent, bk_name, TID_DOUBLE,(void **) &unused_pointer2);
      *unused_pointer2++ = (double) gbl_current_point;
      bk_close(pevent, unused_pointer2);
      gbl_bank_EOM_created = TRUE;
      return bk_size(pevent);

    } else {
      time_Start_read = ss_millitime();
      //Accumulate PMT data during this sleep after the EOM, and before a new BONM, through a separate stream of banks
      //printf(scan_seq.GetMeasTime());
      usleep(scan_seq.GetMeasTime()*1000);
      cm_msg(MDEBUG, "scan_read", "make bank");
      //cm_msg(MDEBUG,"scan_read","current %e, time %g",current, time);

      size = sizeof(gGantryPositions);
      status = db_get_value(hDB, hMoveVariables, "Position", &gGantryPositions, &size, TID_FLOAT, FALSE);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, "scan_read", "cannot get value for Position");
        return DB_NO_ACCESS;
      }

      cm_msg(MDEBUG, "scan_read", "Position of gantry 0: (X, Y, Z): %.3F, %.3f, %.3f", gGantryPositions[0],
             gGantryPositions[1], gGantryPositions[2]);
      cm_msg(MDEBUG, "scan_read", "Position of gantry 1: (X, Y, Z): %.3F, %.3f, %.3f", gGantryPositions[5],
             gGantryPositions[6], gGantryPositions[7]);



      //Init bank creation once
      bk_init(pevent);
      /* CYCI Bank Contents: 1 per measuring point! */
      bk_create(pevent, "CYC0", TID_DOUBLE, (void **)&pwdata);
      *pwdata++ = (double) gbl_current_point;
      for (m = 0; m < 10; m++) {
        *pwdata++ = (double) gGantryPositions[m]; /* word 4; */
      }
      *pwdata++ = (double) time_Start_read;
      bk_close(pevent, pwdata);

      // for voltage and current field 0->5 relate to the Coil Numbers
      float v = 0;
      size_t size = 4;
      bk_create(pevent, "MAG0", TID_DOUBLE, (void **)&pmagdata);
      *pmagdata++ = (double) gbl_current_point;
      for (m = 0; m < 6; m++){
        db_get_data_index(hDB, current_read, &v, (int*) &size, coil_idx(m), TID_FLOAT);
        *pmagdata++ = (double) v;
      }
      db_get_data_index(hDB, current_read, &v, (int*) &size, 16, TID_FLOAT);
      *pmagdata++ = (double) v; // this is the 20" PMT current
      db_get_data_index(hDB, current_read, &v, (int*) &size, 25, TID_FLOAT);
      *pmagdata++ = (double) v; // this is the Monitor PMT current

      for (m = 0; m < 6; m++){
        db_get_data_index(hDB, voltage_read, &v, (int*) &size, coil_idx(m), TID_FLOAT);
        *pmagdata++ = (double) v;
      }
      db_get_data_index(hDB, voltage_read, &v, (int*) &size, 16, TID_FLOAT);
      *pmagdata++ = (double) v; // this is the 20" PMT voltage
      db_get_data_index(hDB, voltage_read, &v, (int*) &size, 25, TID_FLOAT);
      *pmagdata++ = (double) v; // this is the Monitor PMT voltage
      bk_close(pevent, pmagdata);

      gbl_waiting_measurement = FALSE;
      gbl_making_move = FALSE;

      //switch the turn_off hotlink back to false
      BOOL turn_off = FALSE;
      //db_set_data(hDB, hMotors00, &turn_off, sizeof(BOOL), 1, TID_BOOL);
      db_set_data(hDB, hMotors01, &turn_off, sizeof(BOOL), 1, TID_BOOL);
      
      time_Done_read = ss_millitime();

      read_status = SUCCESS;
      if (read_status != SUCCESS) {
        cm_msg(MDEBUG, "scan_read", "Bad read status - skip event");
        return 0; /* No bank generated when read_status is bad */
      }

      cm_msg(MDEBUG, "scan_read", "returning event length=%d", bk_size(pevent));
      cm_msg(MDEBUG, "scan_read", "-->read is done; total cycle took %i ms",
             (time_Done_read - time_move_next_position));
      cm_msg(MDEBUG, "scan_read", "-->Motor move:  %i ms, read:  %i ms",
             (time_Done_cycle - time_Start_motors), (time_Done_read - time_Start_read));

      return bk_size(pevent);
    }
  }
  //printf("gbl_making_move: %i", gbl_making_move);

  // check that an EOM has been created beofore you make the next BONM
  if (gbl_bank_BONM_created == FALSE && gbl_bank_EOM_created == TRUE) {
    gbl_bank_EOM_created = FALSE;
    //Do this once!
    bk_init(pevent);

    // for voltage and current field 0->5 relate to the Coil Numbers
    char bk_name[5] = "BONM";
    double *unused_pointer;
    cm_msg(MDEBUG, "scan_read", "BONM Created");
    bk_create(pevent, bk_name, TID_DOUBLE, (void **)&unused_pointer);
    *unused_pointer++ = (double) gbl_current_point;
    bk_close(pevent, unused_pointer);
    gbl_bank_BONM_created = TRUE;
    return bk_size(pevent);

  }

  return 0;
}


int user_config(void) {
  return SUCCESS;
}

void user_endScan(void) {
}
