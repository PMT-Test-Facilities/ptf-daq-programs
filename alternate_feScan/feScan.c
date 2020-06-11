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


TODO:

1) Need timeout for move that never finish!!!


\********************************************************************/

#include <stdio.h>
#include <math.h>
#include <sys/time.h>


#define FAILURE 0

#include "midas.h"
#include "experim.h"

/* make frontend functions callable from the C framework */
#ifdef __cplusplus
extern "C" {
#endif

/*-- Frontend globals ------------------------------------------------*/

/* odb handles */
HNDLE hDB = 0, hFS = 0;
HNDLE hMDestination = 0, hMMove = 0;
HNDLE hMComplexDestination = 0, hMComplexMove = 0;
HNDLE hMoveVariables = 0;
HNDLE hMoveSettings = 0;
HNDLE hMoveDestination = 0;
HNDLE hMoveStart = 0;
HNDLE hDVMVariables = 0;

extern run_state;

//static char bars[] = "|/-\\";
//static int  i_bar;

/* These variables can be changed to control the level of debugging messages.
   They should really be collapsed into one variable placed in the ODB */
INT dd = 1, ddd = 1, dddd = 1, ddddd = 1;

/*   
    ddd frontend_loop, begin_of_run
    dddd indicates if running in cycle in frontend loop
    dd = scan info  
*/

char expt_name[32];

// flag indicates that we are currently moving; this ensures that frontend_loop doesn't
// start another move at the same time.
BOOL gbl_making_move;
// flag indicates that move is finished and we are waiting for next measurement.
BOOL gbl_waiting_measurement = TRUE;

INT gbl_run_number;

// Total number of points we will move to
INT gbl_total_number_points;

// List of points
float **points;

// Current point we are moving to.
INT gbl_current_point = 0;

// Called begin_of_run; make sure this is called before any moves happen.
BOOL gbl_called_BOR = FALSE;

// Position of gantries, according to feMove front-end.
float gGantryPositions[10];

// Position of limits, according to feMove front-end.
float gGantryLimits[10];

// Destination of gantries, according to feMove front-end.
float gGantryDestinations[10];

// DVM readings, according to fedvm front-end.
double gDVMReadings[22];

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


DWORD time_move_next_position;
DWORD time_Start_motors;
DWORD time_Done_cycle;
DWORD time_Start_read;
DWORD time_Done_read;

SCAN_SETTINGS fs;


/* Scan Type Flags  */
BOOL first_time = FALSE;  /* used by routine move_next_position to know if this is begin_of_run */
BOOL gbl_first_call = TRUE; /* used for deferred stop routine */
INT N_Scans_wanted;
BOOL hold_flag = FALSE; /* hold flag */
INT ncycle_sk_tol; /* number of cycles to skip AFTER returning in tolerance */
INT psleep = 0;

/* The frontend name (client name) as seen by other MIDAS clients   */

char *frontend_name = "feSCAN";

/* The frontend file name, don't change it */
char *frontend_file_name = __FILE__;

/* frontend_loop is called periodically if this variable is TRUE    */
BOOL frontend_call_loop = TRUE;

/* a frontend status page is displayed with this frequency in ms    */
INT display_period = 000;

/* maximum event size produced by this frontend 
   must be less than MAX_EVENT_SIZE (midas.h)*/
INT max_event_size = 10000;

/* buffer size to hold events */
INT event_buffer_size = 10 * 10000;

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

INT interrupt_configure(INT cmd, INT source[], PTYPE adr);

INT scan_read(char *pevent, INT off);

BOOL gen_scan_path();

INT move_next_position();

INT set_scan_params(INT *ninc);

//#include "keithley.c"

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
            0,                    /* log history */
            "", "", ""},
        scan_read,   /* readout routine */
        NULL, NULL, NULL,
    },

    {""}
};

#ifdef __cplusplus
}
#endif


/*-- Dummy routines ------------------------------------------------
  called by mfe.c                                */
INT interrupt_configure(INT cmd, INT source[], PTYPE adr) { return 1; };


/*-- Frontend Init -------------------------------------------------
  called by mfe.c                                */
INT frontend_init() {
  SCAN_SETTINGS_STR(scan_settings_str);

  BOOL watchdog_flag;
  INT status, rstate, size;
  char str[128];

  gbl_making_move = FALSE;

  /* get the experiment name */
  size = sizeof(expt_name);
  status = db_get_value(hDB, 0, "/experiment/Name", &expt_name, &size, TID_STRING, FALSE);
  printf("feScan Front End code for experiment %s now running ... \n", expt_name);

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

  /* Get Scan  settings */
  sprintf(str, "/Equipment/Scan/Settings/");
  status = db_find_key(hDB, 0, str, &hFS);
  if (status != DB_SUCCESS) {
    printf("Record %s  does not exist. It will be created.\n", str);
    status = db_create_record(hDB, 0, str, strcomb(scan_settings_str));
  }
  size = sizeof(fs);
  status = db_get_record(hDB, hFS, &fs, &size, 0);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "frontend_init", "cannot retrieve %s (size of fs=%d)", str, size);
    return DB_NO_ACCESS;
  }



  // Get the Move settings and keys
  sprintf(str, "/Equipment/Move/Variables/");
  status = db_find_key(hDB, 0, str, &hMoveVariables);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "frontend_init", "cannot get key for %s\n", str);
    return DB_NO_ACCESS;
  }

  sprintf(str, "/Equipment/Move/Control/Destination");
  status = db_find_key(hDB, 0, str, &hMoveDestination);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "frontend_init", "cannot find %s", str);
    return DB_NO_ACCESS;
  }

  sprintf(str, "/Equipment/Move/Settings/");
  status = db_find_key(hDB, 0, str, &hMoveSettings);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "frontend_init", "cannot find %s", str);
    return DB_NO_ACCESS;
  }

  sprintf(str, "/Equipment/Move/Control/Start Move");
  status = db_find_key(hDB, 0, str, &hMoveStart);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "frontend_init", "cannot find %s", str);
    return DB_NO_ACCESS;
  }

  // Ensure that we are not already running.
  BOOL moving;
  size = sizeof(moving);
  status = db_get_value(hDB, hMoveVariables, "Moving", &moving, &size, TID_BOOL, FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "cannot get value for Moving\n");
    return DB_NO_ACCESS;
  }
  if (moving) {
    cm_msg(MERROR, "begin_of_run", "Gantry already moving.  Panic!\n");
    return DB_NO_ACCESS;
  } else {
    printf("Good, gantry not moving.\n");
  }

  // Get the DVM settings and keys
  sprintf(str, "/Equipment/PTFDVM/Variables/");
  status = db_find_key(hDB, 0, str, &hDVMVariables);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "frontend_init", "cannot get key for %s\n", str);
    return DB_NO_ACCESS;
  }


  //Reset watchdog and Alarms
  cm_get_watchdog_params(&watchdog_flag, &gbl_watchdog_timeout);
  printf("frontend_init: current watchdog parameters are gbl_watchdog_timeout=%d; watchdog_flag=%d \n",
         gbl_watchdog_timeout, watchdog_flag);

  status = al_reset_alarm(NULL); /* reset all alarms */
  if (status != CM_SUCCESS)
    cm_msg(MINFO, "frontend_init", "problem trying to reset alarms automatically (%d)\n", status);
  else
    printf("frontend_init: Info... cleared all alarms on main status page\n");

  if (status == CM_SUCCESS)
    printf("\n End of routine frontend_init. Program is READY \n");
  return status;
}


/*-- Frontend Exit -------------------------------------------------
  called by mfe.c                                */
INT frontend_exit() {
  //  Kexit();
  printf("frontend_exit:  mfe procedure exiting \n");
  return CM_SUCCESS;
}

/*-- Frontend Loop -------------------------------------------------
called periodically by a loop in mfe.c           
*/
INT frontend_loop()
/*
  - Main function for starting a new move, if we are running and
    the measurement for the last move has finished..
*/
{
  INT status;
  char str[128];

  // Only want to start checking if the begin_of_run has been called.
  if (!gbl_called_BOR) return SUCCESS;

  if (run_state == STATE_RUNNING) { /*      RUNNING
	      check if cycle is finished AND histo update being done */
    ss_sleep(20); /* slow down to get some debugging */
    if (ddddd)
      printf("frontend_loop: gbl_making_move %d, waiteq %d\n",
             gbl_making_move, gbl_waiting_measurement);

    if (!gbl_making_move) { /* not in cycle */
      /* wait until equipment has run */
      if (gbl_waiting_measurement) {
        if (ddddd)printf("frontend_loop: waiting for scan_read to finish\n");
        return SUCCESS; /* processing has not yet finished i.e. all equipments have not run */
      }
      if (ddddd)printf("frontend_loop: running; IN_CYCLE is false\n");

      // Terminate the sequence once we have finished last move.
      if (gbl_current_point >= gbl_total_number_points) {
        sprintf(str, "Stopping run after all points are done");
        printf("Stopping run after all points are done\n", str);
        status = cm_transition(TR_STOP, 0, str, sizeof(str), SYNC, 0);
        return status;
      }

      if (ddd)printf("\nfrontend_loop: starting next move\n");

      /* Start new move */
      // We are starting to move; set flag to ensure that nothing else tries to start us moving.
      gbl_making_move = TRUE;
      status = move_next_position();
      if (status != SUCCESS) {
        sprintf(str, "Stopping run because of error in move_next_position");
        status = cm_transition(TR_STOP, 0, str, sizeof(str), SYNC, 0);
        return status;
      }

    } /* end of not in cycle */
    else { /* we are already moving; (i.e. gbl_making_move is TRUE)
	     nothing to do */
      if (ddddd)
        printf("frontend_loop: running in cycle; nothing to do; exit\n");
      return SUCCESS;
    }
  } /* end of RUNNING */
  else { /* NOT running,  nothing to do */
    return SUCCESS;
  } /* Not running */
  return SUCCESS;
}

/*-- Begin of Run sequence ------------------------------------------
                              called by mfe.c                        */
INT begin_of_run(INT run_number, char *error)
/*
  - start acq
*/
{
  printf("Start begin of run\n");
  BOOL fmove = 0;
  BOOL fmove_readback[2] = {0, 0};
  BOOL fmove_stat;
  INT status, size;
  INT ninc; /* number of increments in scan */

  gbl_transition_requested = FALSE; /* used for deferred transition */
  gbl_waiting_for_run_stop = FALSE; /* unrecoverable error */
  gbl_run_number = run_number;
  gbl_making_move = FALSE;
  gbl_first_call = TRUE; /* for prestop */


  /* Get current Scan settings */
  //  size = sizeof(fs);
  //printf("Cannot get scan settings\n");
  //status = db_get_record(hDB, hFS, &fs, &size, 0);
  //if (status != DB_SUCCESS)
  // {

  //  cm_msg(MERROR, "begin_of_run", "cannot retrieve Scan/Settings");
  //  return DB_NO_ACCESS;
  // }


  // Ensure that we are not already running.
  BOOL moving;
  size = sizeof(moving);
  status = db_get_value(hDB, hMoveVariables, "Moving", &moving, &size, TID_BOOL, FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "cannot get value for Moving\n");
    return DB_NO_ACCESS;
  }
  if (moving) {
    cm_msg(MERROR, "begin_of_run", "Gantry already moving.  Panic!\n");
    return DB_NO_ACCESS;
  } else {
    printf("Good, gantry not moving.\n");
  }

  // Get the initial position of the gantry (for logging info).
  size = sizeof(gGantryPositions);
  status = db_get_value(hDB, hMoveVariables, "Position", &gGantryPositions, &size, TID_FLOAT, FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "cannot get value for Position\n");
    return DB_NO_ACCESS;
  }

  // Get the gantry limits for checking parameters.
  size = sizeof(gGantryLimits);
  status = db_get_value(hDB, hMoveSettings, "Limit Positions", &gGantryLimits, &size, TID_FLOAT, FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "cannot get value for Limits\n");
    return DB_NO_ACCESS;
  }

  printf("Initial position of gantry (X, Y, Z): %.3F, %.3f, %.3f\n", gGantryPositions[0], gGantryPositions[1],
         gGantryPositions[2]);

  //Generate Path
  status = gen_scan_path();
  // replace with proper error later
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "Scan parameters invalid. Path Generation failed\n");
    return DB_NO_ACCESS;
  }

  ss_sleep(1000); /* sleep before starting loop*/

  /* Start cycle */
  first_time = TRUE;
  gbl_current_point = 0;

  // We are starting to move; set flag to ensure that nothing else tries to start us moving.
  gbl_making_move = TRUE;
  status = move_next_position();
  if (status != SUCCESS) {
    printf("begin_of_run: failure from move_next_position, calling set_client_flag with FAILURE\n");
    cm_msg(MINFO, "begin_of_run", "waiting for someone to stop the run after failure from move_next_position");
    gbl_waiting_for_run_stop = TRUE;
    cm_yield(1000);
    return FE_ERR_HW;
  }

  gbl_called_BOR = TRUE;

  if (ddd)printf("end of begin_of_run\n");
  return CM_SUCCESS;
}

/*-- Generate path for scan-----------------------------------------*/
BOOL gen_scan_path(void) {
  INT j, i;
  INT point_num = 0;
  float temp_points[10000][3];

  // Scan parameters in m (read in eventually)
  int scan_type = 1;
  // For cylindrical scan
  int layer;
  int loop;
  int waypoint;
  float PI = 3.14159265;
  float height = 0.50;
  float radius = 0.32;
  float theta = 0;
  float arc_step = 0.05;
  float loop_separation = 0.04;
  float layer_thickness = 0.05;
  // For linear/plane/rectangular prism scan
  int z_layer;
  int x_layer;
  int y_layer;
  int direction_x = -1.0;
  int direction_y = -1.0;
  float prism_height_z = 0.50;
  float prism_length_x = 0.72;
  float prism_width_y = 0.64;
  float z_step = 0.05;
  float x_step = 0.08;
  float y_step = 0.08;
  float init_pos_z = -1.0 * (prism_height_z / 2.0);
  float init_pos_x = -1.0 * (prism_length_x / 2.0);
  float init_pos_y = prism_width_y / 2.0;

  if (scan_type > 1) {
    printf("Error: Invalid scan type.\n");
    return 0;
  }

  // Generate path based on scan parameters:
  switch (scan_type) {
    case 0:
      // Parameter checks
      if (height >= abs(gGantryLimits[2] * 2.0)) {
        printf("Error: Height outside of limits.\n");
        return 0;
      }
      if (radius >= abs(gGantryLimits[1])) {
        printf("Error: Radius outside of limits.\n");
        return 0;
      }
      if (loop_separation <= 0.001) {
        printf("Error: Loop separation too small for motors (min resolution = 1 mm).\n");
        return 0;
      }
      if (layer_thickness <= 0.001) {
        printf("Error: Layer thickness too small for motors (min resolution = 1 mm).\n");
        return 0;
      }
      if (arc_step <= 0.001) {
        printf("Error: Arc step size too small for motors (min resolution = 1 mm).\n");
        return 0;
      }
      printf("Generating cylindrical path...\n");
      // Generate layers
      for (layer = 0; layer <= (trunc(height / layer_thickness)); layer++) {
        temp_points[point_num][0] = 0;
        temp_points[point_num][1] = 0;
        temp_points[point_num][2] = (-1.0 * (height / 2.0)) + (layer * layer_thickness);
        point_num = point_num + 1;
        // Generate loops
        for (loop = 1; loop <= (trunc(radius / loop_separation)); loop++) {
          for (waypoint = 0; waypoint <= trunc((2 * PI * loop * loop_separation) / arc_step); waypoint++) {
            for (j = 0; j < 3; j++) {
              temp_points[point_num][j] = temp_points[point_num - 1][j];
            }
            theta = (2 * PI) * ((arc_step * waypoint) / (2 * PI * loop * loop_separation));
            temp_points[point_num][0] = (loop * loop_separation) * cos(theta);
            temp_points[point_num][1] = (loop * loop_separation) * sin(theta);
            point_num = point_num + 1;
          }
        }
      }
      // Add end point (origin)
      for (j = 0; j < 3; j++) {
        temp_points[point_num][j] = 0.0;
      }
      break;
    case 1:
      // Parameter checks
      if ((init_pos_z + prism_height_z) > abs(gGantryLimits[2])) {
        printf("Error: Height (+ initial z position) outside of limits.\n");
        return 0;
      }
      if ((init_pos_x + prism_length_x) > abs(gGantryLimits[0])) {
        printf("Error: Length (+ initial x position) outside of limits.\n");
        return 0;
      }
      if ((init_pos_y - prism_width_y) < (-1.0 * gGantryLimits[1])) {
        printf("Error: Width (+ initial y position) outside of limits.\n");
        return 0;
      }
      if (z_step <= 0.001) {
        printf("Error: Z step size too small for motors (min resolution = 1 mm).\n");
        return 0;
      }
      if (x_step <= 0.001) {
        printf("Error: X step size too small for motors (min resolution = 1 mm).\n");
        return 0;
      }
      if (y_step <= 0.001) {
        printf("Error: Y step size too small for motors (min resolution = 1 mm).\n");
        return 0;
      }
      printf("Generating linear/plane/rectangular prism path...\n");
      // Generate path
      for (z_layer = 0; z_layer <= (trunc(prism_height_z / z_step)); z_layer++) {
        direction_y = -1.0 * direction_y;
        if (prism_width_y == 0.0) direction_y = 1.0;
        for (y_layer = 0; y_layer <= (trunc(prism_width_y / y_step)); y_layer++) {
          direction_x = -1.0 * direction_x;
          if (prism_length_x == 0.0) direction_x = 1.0;
          for (x_layer = 0; x_layer <= (trunc(prism_length_x / x_step)); x_layer++) {
            temp_points[point_num][0] = direction_x * (init_pos_x + x_layer * x_step);
            temp_points[point_num][1] = direction_y * (init_pos_y - y_layer * y_step);
            temp_points[point_num][2] = init_pos_z + (z_layer * z_step);
            point_num = point_num + 1;
          }
        }
      }
      // Add end point (origin)
      for (j = 0; j < 3; j++) {
        temp_points[point_num][j] = 0.0;
      }
      break;
  }
  // Insert path into global variable/array
  gbl_total_number_points = point_num + 1;
  points = (float **) calloc(point_num + 1, sizeof(float *));
  for (j = 0; j < (point_num + 1); j++) {
    points[j] = (float *) calloc(3, sizeof(float));
  }
  for (j = 0; j < (point_num + 1); j++) {
    for (i = 0; i < 3; i++) {
      points[j][i] = temp_points[j][i];
    }
  }
  // Check path size
  if (gbl_total_number_points > max_event_size) {
    printf("Error: Path contains too many points (%i points). Max event size = %i.\n", gbl_total_number_points,
           max_event_size);
    return 0;
  } else {
    printf("Path contains %i points.\n", gbl_total_number_points);
    printf("Done.\n");
    return 1;
  }
}

/*-- Start cycle sequence ------------------------------------------*/
INT move_next_position(void) {
  BOOL new_supercycle;
  BOOL fmove = 0;
  INT fmove_readback[2] = {0, 0};
  INT fmove_stat;
  INT status, size;

  if (ddd)
    printf("\nstart of move_next_position\n");
  /* New cycle => start at bin 0 */

  if (gbl_current_point >= gbl_total_number_points) {
    cm_msg(MERROR, "move_next_position", "Not good! We've exceeded the number of moves allowed!\n");
    return DB_NO_ACCESS;
  } else {
    printf("Moving to position %i of %i...\n", gbl_current_point + 1, gbl_total_number_points);
  }

  // Set the destination
  gGantryDestinations[0] = points[gbl_current_point][0];
  gGantryDestinations[1] = points[gbl_current_point][1];
  gGantryDestinations[2] = points[gbl_current_point][2];
  INT i;
  for (i = 3; i < 10; i++) {
    gGantryDestinations[i] = 0;
  }
  float pos[10] = {0.03, 0.02, 0.01, 0, 0, 0, 0, 0, 0, 0};
  printf("Now set several destinations at once %.3F %.3F %.3F \n", gGantryDestinations[0], gGantryDestinations[1],
         gGantryDestinations[2]);
  status = db_set_data(hDB, hMoveDestination, gGantryDestinations, sizeof(gGantryDestinations), 10, TID_FLOAT);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "cannot set Destinations");
    return DB_NO_ACCESS;
  }
  // Start the move!
  BOOL start_move[1] = {TRUE};
  printf("Moving!\n");
  status = db_set_data(hDB, hMoveStart, start_move, sizeof(start_move), 1, TID_BOOL);
  printf("Moved!\n");
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "cannot set move");
    return DB_NO_ACCESS;
  }

  // Check that we are moving (why for half second first)
  ss_sleep(500);
  printf("Check moving\n");
  BOOL moving;
  INT size_moving = sizeof(moving);
  status = db_get_value(hDB, hMoveVariables, "Moving", &moving, &size_moving, TID_BOOL, FALSE);
  //  BOOL moving;
  //size = sizeof(moving);
  //status = db_get_value(hDB,hMoveVariables,"Moving",&moving,sizeof(moving),TID_BOOL,FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "cannot get value for Moving\n");
    return DB_NO_ACCESS;
  }
  if (!moving)
    printf("Yikes, not moving! Not good!\n");
  printf("Check complete\n");
  BOOL completed;
  status = db_get_value(hDB, hMoveVariables, "Completed", &completed, &size_moving, TID_BOOL, FALSE);
  if (status != DB_SUCCESS) {
    cm_msg(MERROR, "begin_of_run", "cannot get value for Completed\n");
    return DB_NO_ACCESS;
  }

  BOOL finished_moving = FALSE;
  if (completed && !moving)
    finished_moving = TRUE;
  DWORD time_start_move = ss_millitime();

  INT timeout = 60000 * 5; // 5 minute timeout
  while (!finished_moving) {

    ss_sleep(50);
    status = db_get_value(hDB, hMoveVariables, "Moving", &moving, &size_moving, TID_BOOL, FALSE);
    status = db_get_value(hDB, hMoveVariables, "Completed", &completed, &size_moving, TID_BOOL, FALSE);

    if (completed && !moving)
      finished_moving = TRUE;

    DWORD time_now = ss_millitime();

    if ((INT)(time_now - time_start_move) > timeout) {
      printf("We have waited %ld ms\n", time_now - time_start_move);
      cm_msg(MERROR, "move_next_position", "cannot finish move!\n");
      return DB_NO_ACCESS;
    }
  }

  // Finished moving to the current point.
  gbl_current_point++;

  /* Cycle process sequencer */
  gbl_waiting_measurement = TRUE;

  first_time = FALSE;

  /* Now we are ready to restart the hardware */
  /* Cycle started */
  gbl_making_move = FALSE;

  if (dd)
    printf("\nmove_next_position %d  SCyc %d   go\n ", gbl_CYCLE_N, gbl_SCYCLE_N);
  if (ddd)
    printf("\n");

  time_Done_cycle = ss_millitime();
  return CM_SUCCESS;
}  /* end of move_next_position routine */


/*-- End of Run ----------------------------------------------------*/

INT end_of_run(INT run_number, char *error) {

  //printf("\nend_of_run: starting... and ending\n");  

  // Return to origin (0,0,0) position!!!

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
   - generate event only when cycle has been completed 
   
*/
{
  double current, time;
  INT read_status, size, status;

  if (dddd)printf("entering scan_read, gbl_waiting_measurement %d\n", gbl_waiting_measurement);
  //fflush(stdout);
  time_Start_read = ss_millitime();
  if (gbl_waiting_measurement) {
    sleep(5);
    if (ddd)
      printf("scan_read - make bank\n ");

    if (dddd) { printf("scan read: current %e, time %g\n", current, time); }

    size = sizeof(gGantryPositions);
    status = db_get_value(hDB, hMoveVariables, "Position", &gGantryPositions, &size, TID_FLOAT, FALSE);
    if (status != DB_SUCCESS) {
      cm_msg(MERROR, "scan_read", "cannot get value for Position\n");
      return DB_NO_ACCESS;
    }

    printf("Position of gantry (X, Y, Z): %.3F, %.3f, %.3f\n", gGantryPositions[0], gGantryPositions[1],
           gGantryPositions[2]);


    bk_init(pevent);
    INT i;
    for (i = 0; i < 3; i++) {
      ss_sleep(3000);
      /* 
	 CYCI Bank Contents:
	 
      */
      //    read_status = Kread(&current,&time);
      double *pwdata;
      size = sizeof(gDVMReadings);
      status = db_get_value(hDB, hDVMVariables, "slot 100", &gDVMReadings, &size, TID_DOUBLE, FALSE);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, "scan_read", "cannot get value for DVM Reading\n");
        return DB_NO_ACCESS;
      }
      printf("Reading %i: value = %f, %f, %f\n", i, gDVMReadings[0], gDVMReadings[1], gDVMReadings[2]);

      char name[100];
      sprintf(name, "CYC%i", i);
      bk_create(pevent, name, TID_DOUBLE, &pwdata);
      *pwdata++ = (double) gbl_current_point;
      *pwdata++ = (double) gGantryPositions[0]; /* word 4; */
      *pwdata++ = (double) gGantryPositions[1]; /* word 4; */
      *pwdata++ = (double) gGantryPositions[2]; /* word 4; */
      *pwdata++ = (double) gDVMReadings[0];
      *pwdata++ = (double) gDVMReadings[1];
      *pwdata++ = (double) gDVMReadings[2];
      *pwdata++ = (double) time_Start_read;
      bk_close(pevent, pwdata);
      gbl_waiting_measurement = FALSE;
      gbl_making_move = FALSE;
      time_Done_read = ss_millitime();

      if (dddd) printf("bksize:%d\n", bk_size(pevent));
      read_status = SUCCESS;
      if (read_status != SUCCESS) {
        if (dddd) printf("Bad read status - skip event\n");
        return 0; /* No bank generated when read_status is bad */
      }


    }
    if (dddd)printf("scan_read: returning event length=%d\n", bk_size(pevent));
    printf("  read is done; total cycle took %ld ms\n", (time_Done_read - time_move_next_position));
    printf(" Motor move:  %ld ms, Keithley read:  %ld ms\n",
           (time_Done_cycle - time_Start_motors), (time_Done_read - time_Start_read));

    return bk_size(pevent);
  } else if (dddd) printf("scan read - no data to return\n");
  return 0;
}

/*-- Calculate scan parameters -------------------------------------*/
INT set_scan_params(INT *ninc) {

  return SUCCESS;
}

/*-- Keithly stuff   -------------------------*/
void user_callback(int chan, double dvalue) {
}

int user_config(void) {
  return SUCCESS;
}

void user_endScan(void) {
}
