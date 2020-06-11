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


NEW:
1) Add Phidget readout and Wiener Readout, separate readout
for gantry position, phidget position and PMT positions, later PMT readout

  2) Included Alternate Scan from Harish, and additional two gantry scan options

  3)


  \********************************************************************/

#include <stdio.h>
#include <stdlib.h>
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
  HNDLE   hDB=0, hFS=0;
  HNDLE   hMDestination=0, hMMove=0;
  HNDLE   hMComplexDestination=0,  hMComplexMove=0;
  HNDLE   hMoveVariables =0;
  HNDLE   hMoveSettings =0;
  HNDLE   hMoveDestination =0;
  HNDLE   hMoveStart =0;
  HNDLE   hPhidgetVars0 = 0, hPhidgetVars1 = 0;
  HNDLE   hPtfWiener = 0;
  //HndlehDVMVariables =0;

  extern  run_state;

  //static char bars[] = "|/-\\";
  //static int  i_bar;

  /* These variables can be changed to control the level of debugging messages.
  They should really be collapsed into one variable placed in the ODB */
  INT     dd = 1, ddd = 1, dddd=1, ddddd = 1;

  /*
  ddd frontend_loop, begin_of_run
  dddd indicates if running in cycle in frontend loop
  dd = scan info
  */

  char    expt_name[32];

  // flag indicates that we are currently moving; this ensures that frontend_loop doesn't
  // start another move at the same time.
  BOOL    gbl_making_move;
  // flag indicates that move is finished and we are waiting for next measurement.
  BOOL    gbl_waiting_measurement = FALSE;

  INT     gbl_run_number;


  // NEW : generic
  // Total number of points we will move to
  INT gbl_total_number_points = 0;

  // List of points
  float **points;

  // allocated memory for list of points
  INT max_size = 0;

  // Current point we are moving to.
  INT gbl_current_point = 0;

  // Called begin_of_run; make sure this is called before any moves happen.
  BOOL gbl_called_BOR = FALSE;

  // Position of gantries, according to feMove front-end.
  float   gGantryPositions[10];

  //NEW:
  // Position of limits, according to feMove front-end.
  float   gGantryLimits[10];

  // Destination of gantries, according to feMove front-end.
  float   gGantryDestinations[10];

  // DVM readings, according to fedvm front-end.
  //double   gDVMReadings[22];  //OLD : HallProbe --> Phidgets.

  // Phidget readings
  double gPhidget00[10];
  double gPhidget01[10];

  // Helmholtz coil readings from Wiener power crate
  float gCoilCurrent[40];
  float gCoilVoltage[40];

  // TODO: PMT readings


  INT     gbl_FREQ_n;                /* current cycle # in X scan */
  DWORD   gbl_CYCLE_N;               /* current valid cycle */
  DWORD   gbl_SCYCLE_N;              /* current super cycle */
  DWORD   gbl_SCAN_N;                /* current scan */
  DWORD   gbl_watchdog_timeout;      /* midas watchdog timeout */
  BOOL    gbl_watchdog_flag=TRUE;         /* midas watchdog flag */
  BOOL    gbl_dachshund_flag=FALSE;  /* true when a long watchdog timeout is set */
  BOOL    hot_rereference=FALSE, lhot_rereference=TRUE;
  BOOL    gbl_transition_requested= FALSE;
  BOOL    gbl_waiting_for_run_stop = FALSE; /* global flag to prevent restarting cycle on error */
  BOOL    gbl_bank_BONM_created = FALSE;
  BOOL    gbl_bank_EOM_created = TRUE;
  BOOL    bad_destination = FALSE;

  DWORD   time_move_next_position;
  DWORD   time_Start_motors;
  DWORD   time_Done_cycle;
  DWORD   time_Start_read;
  DWORD   time_Done_read;

  SCAN_SETTINGS fs;


  /* Scan Type Flags  */
  BOOL first_time = FALSE;  /* used by routine move_next_position to know if this is begin_of_run */
  BOOL gbl_first_call = TRUE; /* used for deferred stop routine */
  INT  N_Scans_wanted;
  BOOL hold_flag = FALSE; /* hold flag */
  INT  ncycle_sk_tol; /* number of cycles to skip AFTER returning in tolerance */
  INT  psleep=0;

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
  INT event_buffer_size = 10*10000;

  INT max_event_size_frag = 5*1024*1024; /* not planning to use this */

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
  INT interrupt_configure(INT cmd, INT source[],  PTYPE adr);

  INT scan_read(char *pevent, INT off);
  INT move_next_position();
  INT set_scan_params(INT *ninc);

  // NEW:
  BOOL gen_scan_path();      // for preprogrammed cycli
  BOOL add_point_to_path();  // to make your own scan path

  //#include "keithley.c"

  /*-- Equipment list ------------------------------------------------*/
  EQUIPMENT equipment[] ={

    { "Scan",      /* equipment name */
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
    NULL,NULL,NULL,
    },

    { "" }
  };

  #ifdef __cplusplus
}
#endif


/*-- Dummy routines ------------------------------------------------
called by mfe.c                                */
INT interrupt_configure(INT cmd, INT source[], PTYPE adr){return 1;};


/*-- Frontend Init -------------------------------------------------
called by mfe.c                                */
INT frontend_init()
{
  SCAN_SETTINGS_STR(scan_settings_str);

  BOOL watchdog_flag;
  INT status, rstate, size;
  char str[128];

  gbl_making_move = FALSE;

  /* get the experiment name */
  size = sizeof(expt_name);
  status = db_get_value(hDB, 0, "/experiment/Name", &expt_name, &size, TID_STRING, FALSE);
  printf("feScan Front End code for experiment %s now running ... \n",expt_name);

  /* get basic handle for experiment ODB */
  status=cm_get_experiment_database(&hDB, NULL);
  if(status != CM_SUCCESS)
  {
    cm_msg(MERROR,"frontend_init","Not connected to experiment");
    return CM_UNDEF_EXP;
  }


  /* check for current run state; if not stopped, stop it */
  size = sizeof(rstate);
  status = db_get_value(hDB, 0, "/runinfo/State", &rstate, &size, TID_INT, FALSE);
  if(status != DB_SUCCESS)
  {
    cm_msg(MERROR,"frontend_init","cannot GET /runinfo/State");
    return FE_ERR_ODB;
  }
  if (rstate != STATE_STOPPED)
  {
    cm_msg(MERROR,"frontend_init",
	   "!!! Run In Progress: Stop the run, shutdown and restart this program !!!");
	   return (FE_ERR_HW);
  }
  /* get handles of equipment records   */

  /* Get Scan  settings */
  sprintf(str, "/Equipment/Scan/Settings/");
  status = db_find_key(hDB, 0, str , &hFS);
  if(status != DB_SUCCESS) {
    printf("Record %s  does not exist. It will be created.\n",str);
    status = db_create_record(hDB, 0, str, strcomb(scan_settings_str));
  }
  size = sizeof(fs);
  status = db_get_record(hDB, hFS, &fs, &size, 0);
  if (status != DB_SUCCESS){
    cm_msg(MERROR, "frontend_init", "cannot retrieve %s (size of fs=%d)", str,size);
    return DB_NO_ACCESS;
  }



  // Get the Move settings and keys
  sprintf(str,"/Equipment/Move/Variables/");
  status = db_find_key(hDB, 0, str , &hMoveVariables);
  if (status != DB_SUCCESS){
    cm_msg(MERROR, "frontend_init", "cannot get key for %s\n",str);
    return DB_NO_ACCESS;
  }

  sprintf(str, "/Equipment/Move/Control/Destination");
  status = db_find_key(hDB, 0, str , &hMoveDestination);
  if (status != DB_SUCCESS)
  {
    cm_msg(MERROR, "frontend_init", "cannot find %s",str);
    return DB_NO_ACCESS;
  }

  sprintf(str, "/Equipment/Move/Settings/");
  status = db_find_key(hDB, 0, str , &hMoveSettings);
  if (status != DB_SUCCESS)
  {
    cm_msg(MERROR, "frontend_init", "cannot find %s",str);
    return DB_NO_ACCESS;
  }

  sprintf(str, "/Equipment/Move/Control/Start Move");
  status = db_find_key(hDB, 0, str , &hMoveStart);
  if (status != DB_SUCCESS)
  {
    cm_msg(MERROR, "frontend_init", "cannot find %s",str);
    return DB_NO_ACCESS;
  }

  // Ensure that we are not already running.
  BOOL moving;
  size = sizeof(moving);
  status = db_get_value(hDB,hMoveVariables,"Moving",&moving,&size,TID_BOOL,FALSE);
  if(status != DB_SUCCESS)
  {
    cm_msg(MERROR, "begin_of_run", "cannot get value for Moving\n");
    return DB_NO_ACCESS;
  }
  if(moving){
    cm_msg(MERROR, "begin_of_run", "Gantry already moving.  Panic!\n");
    return DB_NO_ACCESS;
  }else{
    printf("Good, gantry not moving.\n");
  }

  //Phidget Vars
  sprintf(str,"/Equipment/Phidget00/Variables/");
  status = db_find_key(hDB, 0, str , &hPhidgetVars0);
  if (status != DB_SUCCESS){
    cm_msg(MERROR, "frontend_init", "cannot get key for %s\n",str);
    return DB_NO_ACCESS;
  }

  sprintf(str,"/Equipment/Phidget01/Variables/");
  status = db_find_key(hDB, 0, str , &hPhidgetVars1);
  if (status != DB_SUCCESS){
    cm_msg(MERROR, "frontend_init", "cannot get key for %s\n",str);
    return DB_NO_ACCESS;
  }

  //Wiener Power Supply
  sprintf(str,"/Equipment/PtfWiener/Variables/");
  status = db_find_key(hDB, 0, str , &hPtfWiener);
  if (status != DB_SUCCESS){
    cm_msg(MERROR, "frontend_init", "cannot get key for %s\n",str);
    return DB_NO_ACCESS;
  }

  // Get the DVM settings and keys
  //sprintf(str,"/Equipment/PTFDVM/Variables/");
  //status = db_find_key(hDB, 0, str , &hDVMVariables);
  //if (status != DB_SUCCESS)
  //{
    //  cm_msg(MERROR, "frontend_init", "cannot get key for %s\n",str);
    //  return DB_NO_ACCESS;
    //}


    //Reset watchdog and Alarms
    cm_get_watchdog_params(&watchdog_flag, &gbl_watchdog_timeout);
    printf("frontend_init: current watchdog parameters are gbl_watchdog_timeout=%d; watchdog_flag=%d \n",
	   gbl_watchdog_timeout, watchdog_flag);

	   status = al_reset_alarm(NULL); /* reset all alarms */
	   if(status != CM_SUCCESS)
	     cm_msg(MINFO,"frontend_init","problem trying to reset alarms automatically (%d)\n",status);
	   else
	     printf("frontend_init: Info... cleared all alarms on main status page\n");

	   if(status == CM_SUCCESS)
	     printf("\n End of routine frontend_init. Program is READY \n");
	   return status;
}


/*-- Frontend Exit -------------------------------------------------
called by mfe.c                                */
INT frontend_exit()
{
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
  INT status ;
  char str[128];

  // Only want to start checking if the begin_of_run has been called.
  if(!gbl_called_BOR) return SUCCESS;

  if (run_state == STATE_RUNNING)
  { /*      RUNNING
  check if cycle is finished AND histo update being done */
  ss_sleep(20); /* slow down to get some debugging */
  if(ddddd)printf("frontend_loop: gbl_making_move %d, waiteq %d\n",
    gbl_making_move, gbl_waiting_measurement);

  if (!gbl_making_move)
  { /* not in cycle */
  /* wait until equipment has run */
  if ( gbl_waiting_measurement ) {
    if(ddddd)printf("frontend_loop: waiting for scan_read to finish\n");
    return SUCCESS; /* processing has not yet finished i.e. all equipments have not run */
  }
  if(ddddd)printf("frontend_loop: running; IN_CYCLE is false\n");

  // Terminate the sequence once we have finished last move.
  if(gbl_current_point >= gbl_total_number_points){
    sprintf(str,"Stopping run after all points are done");
    printf("%s\n",str);
    status = cm_transition(TR_STOP, 0, str, sizeof(str), TR_SYNC,0);   //TF: new MIDAS: changed SYNC to TR_SYNC
    return status;
  }

  if(ddd)printf("\nfrontend_loop: starting next move\n");

  /* Start new move */
  // We are starting to move; set flag to ensure that nothing else tries to start us moving.

  //TF: this line FAILS into infinite loop (with scan_read where BONM_created will NEVER get set to TRUE so never will reach next move) IF it's a bad dest!! Hence, only do if not bad_dest
  if(gbl_bank_BONM_created == FALSE && bad_destination == FALSE){
    return SUCCESS;
  }
  gbl_making_move = TRUE;
  time_move_next_position = ss_millitime();
  status = move_next_position();

  gbl_bank_BONM_created = FALSE;

  if(status != SUCCESS)
  {
    sprintf(str,"Stopping run because of error in move_next_position");
    status = cm_transition(TR_STOP, 0, str, sizeof(str), TR_SYNC,0);
    return status;
  }

  } /* end of not in cycle */
  else
  { /* we are already moving; (i.e. gbl_making_move is TRUE)
  nothing to do */
  if(ddddd)
    printf("frontend_loop: running in cycle; nothing to do; exit\n");
  return SUCCESS;
  }
  } /* end of RUNNING */
  else
  { /* NOT running,  nothing to do */
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
  BOOL fmove = 0 ;
  BOOL fmove_readback[2] ={0,0} ;
  BOOL fmove_stat;
  INT status, size;
  INT ninc; /* number of increments in scan */

  gbl_transition_requested=FALSE; /* used for deferred transition */
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
    status = db_get_value(hDB,hMoveVariables,"Moving",&moving,&size,TID_BOOL,FALSE);
    if(status != DB_SUCCESS)
    {
      cm_msg(MERROR, "begin_of_run", "cannot get value for Moving\n");
      return DB_NO_ACCESS;
    }
    if(moving){
      cm_msg(MERROR, "begin_of_run", "Gantry already moving.  Panic!\n");
      return DB_NO_ACCESS;
    }else{
      printf("Good, gantry not moving.\n");
    }

    // Get the initial position of the gantry (for logging info).
    size = sizeof(gGantryPositions);
    status = db_get_value(hDB,hMoveVariables,"Position",&gGantryPositions,&size,TID_FLOAT,FALSE);
    if(status != DB_SUCCESS)
    {
      cm_msg(MERROR, "begin_of_run", "cannot get value for Position\n");
      return DB_NO_ACCESS;
    }

    // Get the gantry limits for checking parameters.
    size = sizeof(gGantryLimits);
    status = db_get_value(hDB,hMoveSettings,"Limit Positions",&gGantryLimits,&size,TID_FLOAT,FALSE);
    if(status != DB_SUCCESS)
    {
      cm_msg(MERROR, "begin_of_run", "cannot get value for Limits\n");
      return DB_NO_ACCESS;
    }


    printf("Initial position of gantry (X, Y, Z, R, T): %.3F, %.3f, %.3f %.3f, %.3f\n",gGantryPositions[0],gGantryPositions[1],gGantryPositions[2],gGantryPositions[3],gGantryPositions[4]);

    // TODO::BK : Right now we are returning DB_NO_ACCESS, it would be better is there was an error specifically for the phidgets

    //check that phidgets are working
    int phidget_count =0;
    double phidget_Values_Old[10];
    double phidget_Values_Now[10];
    int size_of_array = sizeof(phidget_Values_Old);
    DWORD time_at_start_of_check;
    db_get_value(hDB,hPhidgetVars0,"PH01",&phidget_Values_Old,&size_of_array,TID_DOUBLE,FALSE);
    db_get_value(hDB,hPhidgetVars0,"PH01",&phidget_Values_Now,&size_of_array,TID_DOUBLE,FALSE);


    //Check that phidget0 is working. The phidget is working if the values change over a set period of time. When a phidget is unplugged there is no fluctuation in the values.
    time_at_start_of_check = ss_millitime();
    while( (phidget_Values_Old[0] == phidget_Values_Now[0]) ||(phidget_Values_Old[1] == phidget_Values_Now[1]) ||(phidget_Values_Old[2] == phidget_Values_Now[2]) ||(phidget_Values_Old[3] == phidget_Values_Now[3]) ||(phidget_Values_Old[4] == phidget_Values_Now[4]) ||(phidget_Values_Old[5] == phidget_Values_Now[5]) ||(phidget_Values_Old[6] == phidget_Values_Now[6]) ||(phidget_Values_Old[7] == phidget_Values_Now[7])||(phidget_Values_Old[8] == phidget_Values_Now[8]) ||(phidget_Values_Old[9] == phidget_Values_Now[9]) ){
      db_get_value(hDB,hPhidgetVars0,"PH01",&phidget_Values_Now,&size_of_array,TID_DOUBLE,FALSE);
      if(ss_millitime()-time_at_start_of_check > 1000*5){
	printf("The readings from Phidget0 are not changing so it is assumed that phidget0 is not working\n");
	return DB_NO_ACCESS ;
      }
      phidget_count++;
    }


    //Check that the phidget1 is  working.
    phidget_count = 0;
    db_get_value(hDB,hPhidgetVars1,"PH01",&phidget_Values_Old,&size_of_array,TID_DOUBLE,FALSE);
    db_get_value(hDB,hPhidgetVars1,"PH01",&phidget_Values_Now,&size_of_array,TID_DOUBLE,FALSE);
    time_at_start_of_check = ss_millitime();
    while( (phidget_Values_Old[0] == phidget_Values_Now[0]) ||(phidget_Values_Old[1] == phidget_Values_Now[1]) ||(phidget_Values_Old[2] == phidget_Values_Now[2]) ||(phidget_Values_Old[3] == phidget_Values_Now[3]) ||(phidget_Values_Old[4] == phidget_Values_Now[4]) ||(phidget_Values_Old[5] == phidget_Values_Now[5]) ||(phidget_Values_Old[6] == phidget_Values_Now[6]) ||(phidget_Values_Old[7] == phidget_Values_Now[7])||(phidget_Values_Old[8] == phidget_Values_Now[8]) ||(phidget_Values_Old[9] == phidget_Values_Now[9]) ){
      db_get_value(hDB,hPhidgetVars1,"PH01",&phidget_Values_Now,&size_of_array,TID_DOUBLE,FALSE);
      if(ss_millitime()-time_at_start_of_check > 1000*5){
	printf("The readings from Phidget1 are not changing so it is assumed that phidget0 is not working\n");
	return DB_NO_ACCESS ;
      }
      phidget_count++;
    }

    printf("Both phidgets are working properly\n");

    //NEW: Generate Path, from alternateScan : generate path ONCE when run starts, based on Scan Settings!
    // essentially fills **points, based on chosen routine
    // TODO only if **points is not a NULL pointer (we can fill it with add_point as well!)
    status = gen_scan_path();


    //DEBUG:
    int i;
    for(i = 0; i< gbl_total_number_points; i++)
      //for(i = 0; i< 15; i++)
      printf("Point %i Gantry0: %.4F %.4F %.4F %.4F %.4F, Gantry1: %.4F %.4F %.4F %.4F %.4F\n",i,points[i][0],points[i][1],points[i][2],points[i][3],points[i][4],points[i][5],points[i][6],points[i][7],points[i][8],points[i][9]);


    // replace with proper error later
    if(status != DB_SUCCESS)
    {
      cm_msg(MERROR, "begin_of_run", "Scan parameters invalid. Path Generation failed\n");
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
    /*gbl_making_move = TRUE;
    time_move_next_position = ss_millitime();
    status = move_next_position();
    if(status != SUCCESS)
    {
      printf("begin_of_run: failure from move_next_position, calling set_client_flag with FAILURE\n");
      cm_msg(MINFO,"begin_of_run","waiting for someone to stop the run after failure from move_next_position");
      gbl_waiting_for_run_stop =  TRUE;
      cm_yield(1000);
      return FE_ERR_HW;
}*/

    gbl_called_BOR = TRUE;

    if(ddd)printf("end of begin_of_run\n");
    return CM_SUCCESS;
}

/*-- Generate path for scan-----------------------------------------*/
BOOL gen_scan_path(void)
{
  INT j;
  INT point_num = 0;

  // Scan parameters in m (TODO: read in eventually)
  int scan_type = 2;
  // For cylindrical scan
  int layer;
  int loop;
  int waypoint;
  float PI = 3.14159265;
  float height = 0.4; //0.50;
  float radius = 0.3;
  float theta = 0;
  float arc_step = 0.2; //0.05;
  float loop_separation = 0.1; //0.04;
  float layer_thickness = 0.1; //0.05;
  //NEW: origin
  float x_center = 0.3;
  float y_center = 0.305;
  // For linear/plane/rectangular prism scan
  int z_layer;
  int x_layer;
  int y_layer;
  int direction_x = -1.0;
  int direction_y = -1.0;
  float prism_height_z = 0.06;//0.;
  float prism_length_x = 0.67;
  float prism_width_y = 0.645;
  float z_step = 0.05;
  float x_step = 0.03;
  float y_step = 0.05;
  float init_pos_z = 0.45;//0.54;
  float init_pos_x = 0;
  float init_pos_y = 0;
  float z_max_value = 0.54;
  int x_increment;
  int x_limit;

  int prev_max_size = -1;

  // Generate path based on scan parameters:
  switch(scan_type){
    case 0:
      // Parameter checks
      //if(height >= fabs(gGantryLimits[2]*2.0)){ //NEW coordinate system
      if(height >= 0.7){   //max Z is not set in gGantryLimits because position at limit Z = 0!
	printf("Error: Height outside of limits.\n");
	return 0;
      }
      float min = 0.;

      // Find min distance to edges (different for both gantries)
      if(gGantryLimits[5] < gGantryLimits[6])
	min = gGantryLimits[5];
      else
	min = gGantryLimits[6];

      if(radius >= min/2){
	printf("Error: Radius outside of limits.\n");
	return 0;
      }
      if(loop_separation <= 0.001){
	printf("Error: Loop separation too small for motors (min resolution = 1 mm).\n");
	return 0;
      }
      if(layer_thickness <= 0.001){
	printf("Error: Layer thickness too small for motors (min resolution = 1 mm).\n");
	return 0;
      }
      if(arc_step <= 0.001){
	printf("Error: Arc step size too small for motors (min resolution = 1 mm).\n");
	return 0;
      }
      printf("Generating cylindrical path...\n");
      // Generate layers
      for(layer=0;layer<=(trunc(height/layer_thickness));layer++){
	//TODO: should be a short function!!!?
	//if necessary, make array larger
	if (point_num == max_size) {
	  prev_max_size = max_size;
	  max_size =! max_size ? 1 : max_size << 1;
	  points = (float**)realloc(points, sizeof(float *)*max_size);

	  for(j = prev_max_size;j < max_size; j++){
	    points[j] = (float *) calloc(10,sizeof(float));
	  }
	}
	// fix X, Y, GoTo layer Z
	points[point_num][0] = 0 + x_center;
	points[point_num][1] = 0 + y_center;
	points[point_num][2] = height - (layer*layer_thickness);

	//Don't change the destination!
	INT k;
	for(k=3; k < 10; k++){
	  points[point_num][k] = -99999;
	}
	point_num = point_num + 1;

	// Generate loops: for each Z, loop through X and Y in a circle
	for(loop = 1;loop <= (trunc(radius/loop_separation));loop++){
	  for(waypoint = 0; waypoint <= trunc((2*PI*loop*loop_separation)/arc_step) ; waypoint++){

	    //if necessary, make array larger
	    if (point_num == max_size) {
	      prev_max_size = max_size;
	      max_size =! max_size ? 1 : max_size << 1;
	      points = (float**)realloc(points, sizeof(float *)*max_size);

	      for(j = prev_max_size;j < max_size; j++){
		points[j] = (float *) calloc(10,sizeof(float));
	      }
	    }
	    //copy all points of the first point in the layer to this point, but change X and Y
	    //for(j = 0;j < 3;j++){
      //points[point_num][j] = points[point_num-1][j];
      //}
      //--> just want SAME z
      theta = (2*PI)*((arc_step*waypoint)/(2*PI*loop*loop_separation));
      points[point_num][0] = (loop*loop_separation)*cos(theta) + x_center;
      points[point_num][1] = (loop*loop_separation)*sin(theta) + y_center;
      points[point_num][2] = points[point_num-1][2];

      //Don't change the destination!
      for(k=3; k < 10; k++){
	points[point_num][k] = -99999;
      }

      point_num = point_num + 1;

	}
      }
  }

  //if necessary, make array larger
  if (point_num == max_size) {
    prev_max_size = max_size;
    max_size =! max_size ? 1 : max_size << 1;
    points = (float**)realloc(points, sizeof(float *)*max_size);

    for(j = prev_max_size;j < max_size; j++){
      points[j] = (float *) calloc(10,sizeof(float));
    }
  }


  // Add end point: back to their corner!
  for(j = 0;j < 9; j++){
    points[point_num][j] = gGantryLimits[j];
  }
  printf("%i - %.4F %.4F %.4F \n",point_num,points[point_num][0],points[point_num][1],points[point_num][2]);
  printf("%i - %.4F %.4F %.4F \n",point_num,points[point_num][5],points[point_num][6],points[point_num][7]);

  break;
  case 1:
    // Parameter checks
    if((init_pos_z + prism_height_z) > z_max_value) {
      printf("Error: Height (+ initial z position) outside of limits.\n");
      return 0;
    }
    //TF: TEMP: uncommented --> FIX when Counter weights FINAL
    //if((init_pos_z ) < fabs(gGantryLimits[2])){
    //  printf("Error: initial z position outside of limits.\n");
    //  return 0;
    //}
    if((init_pos_x + prism_length_x) > fabs(gGantryLimits[5])){
      printf("Error: Length (+ initial x position) outside of limits.\n");
      return 0;
    }
    if((init_pos_x) < fabs(gGantryLimits[0])){
      printf("Error: initial x position outside of limits.\n");
      return 0;
    }
    if((init_pos_y + prism_width_y) > (gGantryLimits[6])){
      printf("Error: Width (+ initial y position) outside of limits.\n");
      return 0;
    }
    if((init_pos_y) < (gGantryLimits[1])){
      printf("Error: initial y position outside of limits.\n");
      return 0;
    }
    if(z_step <= 0.001){
      printf("Error: Z step size too small for motors (min resolution = 1 mm).\n");
      return 0;
    }
    if(x_step <= 0.001){
      printf("Error: X step size too small for motors (min resolution = 1 mm).\n");
      return 0;
    }
    if(y_step <= 0.001){
      printf("Error: Y step size too small for motors (min resolution = 1 mm).\n");
      return 0;
    }
    printf("Generating linear/plane/rectangular prism path...\n");
    // Generate path

    //scan for gantry 0
    for(z_layer=0;z_layer<=(trunc(prism_height_z/z_step));z_layer++){
      for(y_layer=0;y_layer<=(trunc(prism_width_y/y_step));y_layer++){

	if((y_layer % 2) == 0){
	  x_layer = 0;
	  x_increment = 1;
	  x_limit = trunc(prism_length_x/x_step)+1;
	}
	else{
	  x_layer = trunc(prism_length_x/x_step);
	  x_increment = -1;
	  x_limit = -1;
	}

	for(; x_layer!=x_limit ;  x_layer=x_layer+x_increment){

	  //if necessary, make array larger
	  if (point_num == max_size) {
	    prev_max_size = max_size;
	    max_size =! max_size ? 1 : max_size << 1;
	    points = (float**)realloc(points, sizeof(float *)*max_size);

	    for(j = prev_max_size;j < max_size; j++){
	      points[j] = (float *) calloc(10,sizeof(float));
	    }
	  }

	  points[point_num][0] = init_pos_x + (x_layer*x_step);
	  points[point_num][1] = init_pos_y + (y_layer*y_step);
	  points[point_num][2] = init_pos_z + (z_layer*z_step);
	  points[point_num][3] = 90;
	  points[point_num][4] = 0;
	  points[point_num][7] = init_pos_z + (z_layer*z_step);
	  points[point_num][8] = -90;
	  points[point_num][9] = 0;

	  if(points[point_num][1] > 0.3){

	    points[point_num][5] = 0.67;
	    points[point_num][6] = 0.;

	  }
	  else{
	    points[point_num][5] = 0.67;
	    points[point_num][6] = 0.645;
	  }

	  point_num = point_num + 1;

	}
      }

      // scan for gantry 1

      for(y_layer=0;y_layer<=(trunc(prism_width_y/y_step));y_layer++){

	if((y_layer % 2) == 0){
	  x_layer = 0;
	  x_increment = 1;
	  x_limit = trunc(prism_length_x/x_step)+1;
	}
	else{
	  x_layer = trunc(prism_length_x/x_step);
	  x_increment = -1;
	  x_limit = -1;
	}

	for(; x_layer!=x_limit ;  x_layer=x_layer+x_increment){

	  //if necessary, make array larger
	  if (point_num == max_size) {
	    prev_max_size = max_size;
	    max_size =! max_size ? 1 : max_size << 1;
	    points = (float**)realloc(points, sizeof(float *)*max_size);

	    for(j = prev_max_size;j < max_size; j++){
	      points[j] = (float *) calloc(10,sizeof(float));
	    }
	  }

	  points[point_num][2] = 0.15; // init_pos_z + (z_layer*z_step);
	  points[point_num][3] = -90;
	  points[point_num][4] = 0;
	  points[point_num][5] = init_pos_x + (x_layer*x_step);
	  points[point_num][6] = init_pos_y + (y_layer*y_step);
	  points[point_num][7] = init_pos_z + (z_layer*z_step);
	  points[point_num][8] = -90;
	  points[point_num][9] = 0;

	  if(points[point_num][6] > 0.3){

	    points[point_num][0] = 0.;
	    points[point_num][1] = 0.;
	  }
	  else{
	    points[point_num][0] = 0.;
	    points[point_num][1] = 0.645;
	  }

	  point_num = point_num + 1;

	}
      }
    }

    // Add end point: back to their corner!
    ///// TODO: get these values from MOVE/SETTINGs/LIMIT POSITIONS
   /* for(j=0;j<5;j++){
      points[point_num][j] = 0.0;
    }
    //gantry 1:
    points[point_num][5] = 0.67;
    points[point_num][6] = 0.645;
    points[point_num][7] = 0.0;
    points[point_num][8] = -90;
    points[point_num][9] = 0.0;*/


    break;

    case 2:
      // Parameter checks
      if((init_pos_z + prism_height_z) > z_max_value) {
	printf("Error: Height (+ initial z position) outside of limits.\n");
	return 0;
      }
      //if((init_pos_z ) < fabs(gGantryLimits[2])){
	///printf("Error: initial z position outside of limits.\n");
	//return 0;
      //}
      if((init_pos_x + prism_length_x) > fabs(gGantryLimits[5])){
	printf("Error: Length (+ initial x position) outside of limits.\n");
	return 0;
      }
      if((init_pos_x) < fabs(gGantryLimits[0])){
	printf("Error: initial x position outside of limits.\n");
	return 0;
      }
      if((init_pos_y + prism_width_y) > (gGantryLimits[6])){
	printf("Error: Width (+ initial y position) outside of limits.\n");
	return 0;
      }
      if((init_pos_y) < (gGantryLimits[1])){
	printf("Error: initial y position outside of limits.\n");
	return 0;
      }
      if(z_step <= 0.001){
	printf("Error: Z step size too small for motors (min resolution = 1 mm).\n");
	return 0;
      }
      if(x_step <= 0.001){
	printf("Error: X step size too small for motors (min resolution = 1 mm).\n");
	return 0;
      }
      if(y_step <= 0.001){
	printf("Error: Y step size too small for motors (min resolution = 1 mm).\n");
	return 0;
      }
      printf("Generating linear/plane/rectangular prism path...\n");
      // Generate path

      const double scan_init_y=0.;
      const double scan_step_y=0.05;
      const double scan_width_y=0.645;
      double rotation;

      
      // scan for gantry 0
      // for(y_layer=0;y_layer<=(trunc(scan_width_y/scan_step_y));y_layer++){
      for(rotation=-100;rotation <= 100;rotation++){
	//if necessary, make array larger
	if (point_num == max_size) {
	  prev_max_size = max_size;
	  max_size =! max_size ? 1 : max_size << 1;
	  points = (float**)realloc(points, sizeof(float *)*max_size);

	  for(j = prev_max_size;j < max_size; j++){
	    points[j] = (float *) calloc(10,sizeof(float));
	  }
	}

	points[point_num][0] = 0.1;
//	points[point_num][1] = scan_init_y + (y_layer*scan_step_y);
	points[point_num][1] = 0.1;
	points[point_num][2] = 0.25;
	points[point_num][3] = rotation;
	points[point_num][4] = 0;
	points[point_num][5] = 0.57;
	points[point_num][6] = 0.39;
	points[point_num][7] = 0.25;
	points[point_num][8] = rotation;
	points[point_num][9] = 0;
	point_num = point_num + 1;

  }

      // Add end point: to the initial position!
      ///// TODO: get these values from MOVE/SETTINGs/LIMIT POSITIONS
/*	points[point_num][0] = 0;
	points[point_num][1] = 0;
	points[point_num][2] = 0.54;
	points[point_num][3] = 90;
	points[point_num][4] = -50;
      //gantry 1:
      points[point_num][5] = 0.67;
      points[point_num][6] = 0.39;
      points[point_num][7] = 0.54;
      points[point_num][8] = 0;
      points[point_num][9] = -60;*/

      break;
      case 3:
	// Total number of points we will move to
	//    point_num =284;//13;
	point_num =2;
	max_size =  point_num+1;

	// For good test the speeds should be
	// List of points; little sequence around the 'PMT'
	//TODO : Is there a more elegant way to do this??? USE the ADD point function (TODO)
	//converted sequence from feScan.c to new coordinates,
	       // using: Gant 0: +(0.515,0.305)
	       //        Gant 1: *(-1) +(0.515,0.305)
	       //        z * (-1)

	       //and then changed to something that works!
	       // sequence for debugging gantry 1
	       float sequence[3][10] = {{0.,0.,0.,-90,0.,0.67,0.61,0.,-90.,0.},
	       /*			      {0.,0.,0.,-90,0.,0.57,0.61,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.47,0.61,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.37,0.61,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.47,0.61,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.57,0.61,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.67,0.51,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.57,0.51,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.47,0.51,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.37,0.51,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.47,0.51,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.57,0.51,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.67,0.41,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.57,0.41,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.47,0.41,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.37,0.41,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.47,0.41,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.57,0.41,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.67,0.31,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.57,0.31,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.47,0.31,0.,-90.,0.},
	       */  {0.,0.,0.,-90,0.,0.37,0.31,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,0.47,0.31,0.,-90.,0.}};
	       /*

	       // Original
	       //little sequence for a first scan (one gantry):
	       float sequence[49][10] = {{0.,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.2,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.3,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.4,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.5,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.61,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.61,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.2,0.61,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.61,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.51,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.41,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.31,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.21,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.42,0.21,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.52,0.21,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.62,0.21,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.62,0.11,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.62,0.,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.52,0.,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.42,0.,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.22,0.,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.12,0.,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.02,0.,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.1,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.2,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.3,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.4,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.5,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.61,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.61,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.2,0.61,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.61,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.51,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.41,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.31,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.21,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.42,0.21,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.52,0.21,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.62,0.21,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.62,0.11,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.62,0.,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.52,0.,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.42,0.,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.22,0.,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.12,0.,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.02,0.,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.,0.,-90,0.,1.03,0.61,0.,-90.,0.}};
	       */
	       /*
	       //  Ben's sequence. Check for time dependent magnetic field

	       float sequence[35][10] = {{0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.1,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.}};
	       */


	       // Ben's modified sequence (grid pattern)
	       /*
	       float sequence[285][10] = {{0.,0.,0.,-90,0.,1.03,0.61,0.,-90.,0.},         // x = 0  // Z = 0
	       {0.,0.05,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.15,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.2,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.25,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.3,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.35,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.4,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.45,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.5,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.55,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.61,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.05,0.61,0.,-90,0.,1.03,0.61,0.,-90.,0.},      // transition
	       {0.1,0.61,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.61,0.,-90,0.,1.03,0.61,0.,-90.,0.},        // x = 0.16
	       {0.16,0.55,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.5,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.45,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.4,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.35,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.3,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.25,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.2,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.15,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.05,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.0,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.21,0.,0.,-90,0.,1.03,0.61,0.,-90.,0.},      //transition
	       {0.26,0.,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.,0.,-90,0.,1.03,0.61,0.,-90.,0.},        // x = 0.32
	       {0.32,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.15,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.2,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.25,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.3,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.35,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.4,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.45,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.5,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.55,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.61,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.2,0.,-90,0.,1.03,0.61,0.,-90.,0.},      //transition
	       {0.37,0.2,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.42,0.2,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.2,0.,-90,0.,1.03,0.61,0.,-90.,0.},       // x = 0.48
	       {0.48,0.15,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.05,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.0,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.53,0.0,0.,-90,0.,1.03,0.61,0.,-90.,0.},      // transition
	       {0.58,0.0,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.,0.,-90,0.,1.03,0.61,0.,-90.,0.},        // x = 0.62
	       {0.64,0.05,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.1,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.15,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.2,0.,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.,0.1,-90,0.,1.03,0.61,0.,-90.,0.},         // x = 0 // Z = 0.1
	       {0.,0.05,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.1,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.15,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.2,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.25,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.3,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.35,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.4,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.45,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.5,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.55,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.61,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.05,0.61,0.1,-90,0.,1.03,0.61,0.,-90.,0.},      // transition
	       {0.1,0.61,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.61,0.1,-90,0.,1.03,0.61,0.,-90.,0.},        // x = 0.16
	       {0.16,0.55,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.5,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.45,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.4,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.35,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.3,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.25,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.2,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.15,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.1,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.05,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.0,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.21,0.,0.1,-90,0.,1.03,0.61,0.,-90.,0.},      //transition
	       {0.26,0.,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.,0.1,-90,0.,1.03,0.61,0.,-90.,0.},        // x = 0.32
	       {0.32,0.1,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.15,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.2,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.25,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.3,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.35,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.4,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.45,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.5,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.55,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.61,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.2,0.1,-90,0.,1.03,0.61,0.,-90.,0.},      //transition
	       {0.37,0.2,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.42,0.2,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.2,0.1,-90,0.,1.03,0.61,0.,-90.,0.},       // x = 0.48
	       {0.48,0.15,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.1,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.05,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.0,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.53,0.0,0.1,-90,0.,1.03,0.61,0.,-90.,0.},      // transition
	       {0.58,0.0,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.,0.1,-90,0.,1.03,0.61,0.,-90.,0.},        // x = 0.62
	       {0.64,0.05,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.1,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.15,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.2,0.1,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.,0.2,-90,0.,1.03,0.61,0.,-90.,0.},         // x = 0  // z = 0.2
	       {0.,0.05,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.1,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.15,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.2,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.25,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.3,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.35,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.4,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.45,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.5,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.55,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.61,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.05,0.61,0.2,-90,0.,1.03,0.61,0.,-90.,0.},      // transition
	       {0.1,0.61,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.61,0.2,-90,0.,1.03,0.61,0.,-90.,0.},        // x = 0.16
	       {0.16,0.55,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.5,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.45,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.4,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.35,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.3,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.25,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.2,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.15,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.1,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.05,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.0,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.21,0.,0.2,-90,0.,1.03,0.61,0.,-90.,0.},      //transition
	       {0.26,0.,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.,0.2,-90,0.,1.03,0.61,0.,-90.,0.},        // x = 0.32
	       {0.32,0.1,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.15,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.2,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.25,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.3,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.35,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.4,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.45,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.5,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.55,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.61,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.2,0.2,-90,0.,1.03,0.61,0.,-90.,0.},      //transition
	       {0.37,0.2,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.42,0.2,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.2,0.2,-90,0.,1.03,0.61,0.,-90.,0.},       // x = 0.48
	       {0.48,0.15,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.1,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.05,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.0,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.53,0.0,0.2,-90,0.,1.03,0.61,0.,-90.,0.},      // transition
	       {0.58,0.0,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.,0.2,-90,0.,1.03,0.61,0.,-90.,0.},        // x = 0.62
	       {0.64,0.05,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.1,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.15,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.2,0.2,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.,0.3,-90,0.,1.03,0.61,0.,-90.,0.},         // x = 0  // z = 0.3
	       {0.,0.05,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.1,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.15,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.2,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.25,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.3,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.35,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.4,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.45,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.5,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.55,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.61,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.05,0.61,0.3,-90,0.,1.03,0.61,0.,-90.,0.},      // transition
	       {0.1,0.61,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.61,0.3,-90,0.,1.03,0.61,0.,-90.,0.},        // x = 0.16
	       {0.16,0.55,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.5,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.45,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.4,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.35,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.3,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.25,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.2,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.15,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.1,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.05,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.0,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.21,0.,0.3,-90,0.,1.03,0.61,0.,-90.,0.},      //transition
	       {0.26,0.,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.,0.3,-90,0.,1.03,0.61,0.,-90.,0.},        // x = 0.32
	       {0.32,0.1,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.15,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.2,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.25,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.3,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.35,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.4,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.45,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.5,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.55,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.61,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.2,0.3,-90,0.,1.03,0.61,0.,-90.,0.},      //transition
	       {0.37,0.2,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.42,0.2,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.2,0.3,-90,0.,1.03,0.61,0.,-90.,0.},       // x = 0.48
	       {0.48,0.15,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.1,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.05,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.0,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.53,0.0,0.3,-90,0.,1.03,0.61,0.,-90.,0.},      // transition
	       {0.58,0.0,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.,0.3,-90,0.,1.03,0.61,0.,-90.,0.},        // x = 0.62
	       {0.64,0.05,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.1,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.15,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.2,0.3,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.,0.4,-90,0.,1.03,0.61,0.,-90.,0.},         // x = 0 // z = 0.4
	       {0.,0.05,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.1,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.15,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.2,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.25,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.3,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.35,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.4,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.45,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.5,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.55,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.,0.61,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.05,0.61,0.4,-90,0.,1.03,0.61,0.,-90.,0.},      // transition
	       {0.1,0.61,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.61,0.4,-90,0.,1.03,0.61,0.,-90.,0.},        // x = 0.16
	       {0.16,0.55,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.5,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.45,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.4,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.35,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.3,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.25,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.2,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.15,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.1,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.05,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.16,0.0,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.21,0.,0.4,-90,0.,1.03,0.61,0.,-90.,0.},      //transition
	       {0.26,0.,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.,0.4,-90,0.,1.03,0.61,0.,-90.,0.},        // x = 0.32
	       {0.32,0.1,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.15,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.2,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.25,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.3,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.35,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.4,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.45,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.5,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.55,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.61,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.32,0.2,0.4,-90,0.,1.03,0.61,0.,-90.,0.},      //transition
	       {0.37,0.2,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.42,0.2,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.2,0.4,-90,0.,1.03,0.61,0.,-90.,0.},       // x = 0.48
	       {0.48,0.15,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.1,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.05,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.48,0.0,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.53,0.0,0.4,-90,0.,1.03,0.61,0.,-90.,0.},      // transition
	       {0.58,0.0,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.,0.4,-90,0.,1.03,0.61,0.,-90.,0.},        // x = 0.62
	       {0.64,0.05,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.1,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.15,0.4,-90,0.,1.03,0.61,0.,-90.,0.},
	       {0.64,0.2,0.4,-90,0.,1.03,0.61,0.,-90.,0.}};
	       */
	       /*

	       float sequence[4][10] = {{0.1,0.15,0.0,-90.0,0.0,0.9,0.61,0,-90,0},
	       {0.2,0.1,0.0,-90.0,0.0,1.03,0.61,0,-90,0},  //test how Bad points are treated
	       {1.0,0.21,0.0,-90.0,0.0,1.03,0.61,0,-90,0},
	       {0.1,0.05,0.0,-90.0,0.0,1.03,0.61,0,-90,0}};*/

	       //			      {0.265,0.555,0.0,45,0.0,0.765,0.205,0,-45,0},
	       //			      {0.565,0.635,0.0,90.0,0.0,0.865,0.105,0,0,0},
	       //			      {0.565,0.555,0.25,90.0,-45.0,0.865,0.305,0.0,-45,0},
	       //			      {0.565,0.225,0.27,90.0,-85.0,0.865,0.305,0.2,-45,0},
	       //{0.05,0,-0.27,0.0,-85.0},
	       //			      {0.265,0.305,0.24,0.0,-45.0,0.865,0.105,0.2,0,0},
	       //			      {0.165,0.305,0.0,0.0,0.0,0.865,0.105,0.0,0,0},
	       //			      {0.265,0.055,0.0,-45,0.0,0.865,0.305,0.0,-45,0},
	       //			      {0.465,0.,0.0,-90.0,0.0,0.865,0.305,0.2,-45,0},
	       //			      {0.465,0.055,0.22,-90.0,-45.0,0.865,0.105,0.0,0,0},
	       //			      {0.465,0.385,0.27,-90.0,-85.0,0.865,0.105,0,0,0},
	       //{0.05,0,-0.27,0.0,-85.0},
	       //			      {0.265,0.305,0.24,0.0,-45.0,0.765,0.205,0,-45,0},
	       //			      {0.165,0.305,0.0,0.0,0.0,0.615,0.355,0,-90,0},
	       // Add end point: back to their corner!
	       ///// TODO: get these values from MOVE/SETTINGs/LIMIT POSITIONS
	       //			     {0., 0., 0., -90., 0., 1.03, 0.61, 0.0, -90.0, 0.0}};
	       //			     {0.05, 0.05, 0., -90., 0., 0.95, 0.61, 0.0, -90.0, 0.0}};

	       points = (float **) calloc(point_num+1,sizeof(float *));
	       for(j = 0; j < (point_num+1); j++){
		 points[j] = (float *) calloc(10,sizeof(float));
	       }

	       int p,l;
	       for(p = 0; p < max_size; p++){
		 for(l = 0; l < 10; l++){
		   points[p][l] = sequence[p][l];
		 }
	       }

	       break;

	       default:
		 printf("Error: Invalid scan type.\n");
		 return 0;

		 }//end switch case

		 // Insert path into global variable/array
		 gbl_total_number_points = point_num + 1;

		 //DEBUG:
		 //for(j = 0; j< gbl_total_number_points; j++)
		 //for(j = 0; j< 15; j++)
		 //printf("Point %i Gantry0: %.4F %.4F %.4F, Gantry1: %.4F %.4F %.4F \n",j,points[j][0],points[j][1],points[j][2],points[j][5],points[j][6],points[j][7]);


		 // Check path size
		 if(gbl_total_number_points > max_event_size){
		   printf("Error: Path contains too many points (%i points). Max event size = %i.\n", gbl_total_number_points, max_event_size);
		   return 0;
		 }
		 else{
		   printf("Path contains %i points.\n", gbl_total_number_points);
		   printf("Done.\n");
		   return 1;
		 }
		 }


		 /*-- Start cycle sequence ------------------------------------------*/
		 INT move_next_position(void)
		 {
		   BOOL new_supercycle;
		   BOOL fmove = 0 ;
		   INT fmove_readback[2] ={0,0} ;
		   INT fmove_stat;
		   INT status, size;


		   if(ddd)
		     printf("\nstart of move_next_position\n");
		   /* New cycle => start at bin 0 */

		   if(gbl_current_point >= gbl_total_number_points){
		     cm_msg(MERROR, "move_next_position", "Not good! We've exceeded the number of moves allowed!\n");
		     return DB_NO_ACCESS;
		   }

		   // Set the destination
		   int i;
		   for(i = 0; i < 10; i++)
		     if(points[gbl_current_point][i] > -999)
		       gGantryDestinations[i] = points[gbl_current_point][i];
		     else
		       gGantryDestinations[i] = gGantryPositions[i];


		     //DEBUG:
		     //for(i = 0; i< gbl_total_number_points; i++)
		     //  printf("Point %i Gantry0: %.4F %.4F %.4F, Gantry1: %.4F %.4F %.4F \n",i,points[i][0],points[i][1],points[i][2],points[i][5],points[i][6],points[i][7]);

		   printf("Now set several destinations at once %.3F %.3F %.3F %.3F %.3F   (%i)\n",gGantryDestinations[0],gGantryDestinations[1],gGantryDestinations[2],gGantryDestinations[3],gGantryDestinations[4],gbl_current_point);
		   printf("And for gantry1 : %.3F %.3F %.3F %.3F %.3F   (%i)\n",gGantryDestinations[5],gGantryDestinations[6],gGantryDestinations[7],gGantryDestinations[8],gGantryDestinations[9],gbl_current_point);
		   status = db_set_data(hDB,hMoveDestination,gGantryDestinations,sizeof(gGantryDestinations),10,TID_FLOAT);
		   if (status != DB_SUCCESS)
		   {
		     cm_msg(MERROR, "begin_of_run", "cannot set Destinations");
		     return DB_NO_ACCESS;
		   }

		   // Start the move!
		   BOOL start_move[1] = {TRUE};
		   printf("Moving!\n");
		   status = db_set_data(hDB,hMoveStart,start_move,sizeof(start_move),1,TID_BOOL);
		   printf("Moved!\n");
		   if (status != DB_SUCCESS)
		   {
		     cm_msg(MERROR, "begin_of_run", "cannot set move");
		     return DB_NO_ACCESS;
		   }

		   // Check that we are moving (why for half second first)
		   ss_sleep(500);
		   printf("Check moving\n");
		   BOOL moving;
		   INT size_moving = sizeof(moving);
		   status = db_get_value(hDB,hMoveVariables,"Moving",&moving,&size_moving,TID_BOOL,FALSE);
		   //  BOOL moving;
		   //size = sizeof(moving);
		   //status = db_get_value(hDB,hMoveVariables,"Moving",&moving,sizeof(moving),TID_BOOL,FALSE);
		   if(status != DB_SUCCESS)
		   {
		     cm_msg(MERROR, "begin_of_run", "cannot get value for Moving\n");
		     return DB_NO_ACCESS;
		   }
		   if(!moving)
		     printf("Yikes, not moving! Not good!\n");
		   printf("Check complete\n");
		   BOOL completed;
		   status = db_get_value(hDB,hMoveVariables,"Completed",&completed,&size_moving,TID_BOOL,FALSE);
		   if(status != DB_SUCCESS)
		   {
		     cm_msg(MERROR, "begin_of_run", "cannot get value for Completed\n");
		     return DB_NO_ACCESS;
		   }

		   BOOL finished_moving = FALSE;
		   if(completed && !moving)
		     finished_moving = TRUE;
		   DWORD time_start_move = ss_millitime();

		   INT timeout = 60000*5; // 3 (5) minute timeout

		   // Loop which checks ODB until move to next point is finished
		   while(!finished_moving){

		     ss_sleep(50);
		     status = db_get_value(hDB,hMoveVariables,"Moving",&moving,&size_moving,TID_BOOL,FALSE);
		     status = db_get_value(hDB,hMoveVariables,"Completed",&completed,&size_moving,TID_BOOL,FALSE);


		     if(completed && !moving)
		       finished_moving = TRUE;

		     DWORD time_now = ss_millitime();

		     //DEBUG
		     //if((INT)(time_now-time_start_move)%10000 < 100){
		   //    printf("yield %i ms\n",time_now-time_start_move);
		   //}

		   if((INT)(time_now-time_start_move) > timeout){
		     printf("We have waited %i ms, which is too long \n",time_now-time_start_move);
		     cm_msg(MERROR, "move_next_position", "cannot finish move!\n");
		     return DB_NO_ACCESS;
		   }
		 }

		 // Finished moving to the current point.
		 gbl_current_point++;

		 /* Cycle process sequencer
		 NEW: only if moved to valid new position */
		 
		 //TF: new: need to know this for L.450, so make it global
		 //BOOL bad_destination;
		 size = sizeof(bad_destination);
		 status = db_get_value(hDB,hMoveVariables,"Bad Destination",&bad_destination,&size,TID_BOOL,FALSE);
		 if(status != DB_SUCCESS)
		 {
		   cm_msg(MERROR, "move_next_position", "cannot get value for Bad Destination\n");
		   return DB_NO_ACCESS;
		 }
		 //DEBUG
		 printf("Bad dest: %i \n",bad_destination);
		 if(!bad_destination)
		   gbl_waiting_measurement = TRUE;

		 first_time = FALSE;

		 /* Now we are ready to restart the hardware */
		 /* Cycle started */
		 gbl_making_move = FALSE;

		 if(dd)
		   printf("\nmove_next_position %d  SCyc %d   go\n ",gbl_CYCLE_N,gbl_SCYCLE_N);
		 if(ddd)
		   printf("\n");

		 time_Done_cycle = ss_millitime();
		 return CM_SUCCESS;
		 }  /* end of move_next_position routine */


		 /*-- End of Run ----------------------------------------------------*/

		 INT end_of_run(INT run_number, char *error)
		 {

		   //printf("\nend_of_run: starting... and ending\n");
		   INT k;
		   for(k = 0; k < max_size;k++){
		     free(points[k]);
		   }
		   free(points);

		   gbl_called_BOR = FALSE;

		   return CM_SUCCESS;
		 }

		 /*-- Pause Run -----------------------------------------------------*/
		 INT pause_run(INT run_number, char *error)
		 {
		   cm_msg(MERROR,"pause_run","This command DOES NOT WORK for this program");
		   cm_msg(MERROR,"pause_run"," MIDAS PAUSE detected. RUN MUST BE STOPPED then restarted.");
		   return CM_SUCCESS;
		 }

		 /*-- Resume Run ----------------------------------------------------*/
		 INT resume_run(INT run_number, char *error)
		 {
		   cm_msg(MERROR,"resume_run","This command DOES NOT WORK for this program");
		   cm_msg(MERROR,"resume_run","MIDAS RESUME detected. RUN MUST BE STOPPED and restarted.");

		   return CM_SUCCESS;
		 }

		 /*-- Trigger event routines ----------------------------------------*/
		 INT poll_event(INT source, INT count, BOOL test)
		 /* Polling routine for events. Returns TRUE if event
		 is available. If test equals TRUE, don't return. The test
		 flag is used to time the polling */
		 {
		   int   i;
		   for (i=0 ; i<count ; i++)
		   {
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
		   double *pwdata, *phidgdata, *pmagdata;
		   double current,time;
		   INT read_status, size, status, m;

		   if(dddd)printf("entering scan_read, gbl_waiting_measurement %d\n",gbl_waiting_measurement);
		   //fflush(stdout);
		   time_Start_read = ss_millitime();





		   if (gbl_waiting_measurement)
		   {
		     if (gbl_bank_EOM_created == FALSE) {

		       //Do this once!
		       bk_init(pevent);

		       // for voltage and current field 0->5 relate to the Coil Numbers
		       char bk_name[4] ="EOM";
		       double* unused_pointer2;
		       printf("EOM Created\n");
		       bk_create(pevent, bk_name, TID_DOUBLE, &unused_pointer2);
		       *unused_pointer2++ = (double)gbl_current_point;
		       bk_close(pevent, unused_pointer2);
		       gbl_bank_EOM_created = TRUE;
		       return bk_size(pevent);


		     }
		     else {
		       ss_sleep(5);   //TODO, was used to be sleep, but ss_sleep in ms, should I convert from 5(sec??)
		       if (ddd)
			 printf("scan_read - make bank\n ");

		       if (dddd){ printf("scan read: current %e, time %g\n",current, time);    }

		       size = sizeof(gGantryPositions);
		       status = db_get_value(hDB,hMoveVariables,"Position",&gGantryPositions,&size,TID_FLOAT,FALSE);
		       if(status != DB_SUCCESS)
		       {
			 cm_msg(MERROR, "scan_read", "cannot get value for Position\n");
			 return DB_NO_ACCESS;
		       }

		       printf("Position of gantry 0: (X, Y, Z): %.3F, %.3f, %.3f\n",gGantryPositions[0],gGantryPositions[1],gGantryPositions[2]);
		       printf("Position of gantry 1: (X, Y, Z): %.3F, %.3f, %.3f\n",gGantryPositions[5],gGantryPositions[6],gGantryPositions[7]);

		       size = sizeof(gCoilCurrent);
		       status = db_get_value(hDB,hPtfWiener,"current",&gCoilCurrent,&size,TID_FLOAT,FALSE);
		       if(status != DB_SUCCESS)
		       {
			 cm_msg(MERROR, "scan_read", "cannot get value for Coil Current\n");
			 return DB_NO_ACCESS;
		       }

		       size = sizeof(gCoilVoltage);
		       status = db_get_value(hDB,hPtfWiener,"senseVoltage",&gCoilVoltage,&size,TID_FLOAT,FALSE);
		       if(status != DB_SUCCESS)
		       {
			 cm_msg(MERROR, "scan_read", "cannot get value for Coil Voltage\n");
			 return DB_NO_ACCESS;
		       }

		       //Do this once!
		       bk_init(pevent);

		       INT i;
		       int number_of_data_points=30;
		       for(i = 0; i < number_of_data_points ; i++) {
			 ss_sleep(100);
			 /*
			 CYCI Bank Contents:

			 */
			 //read_status = Kread(&current,&time);
			 /*gDVMReadings;
			 size = sizeof(gDVMReadings);
			 status = db_get_value(hDB,hDVMVariables,"slot 100",&gDVMReadings,&size,TID_DOUBLE,FALSE);
			 if(status != DB_SUCCESS)
			 {
			   cm_msg(MERROR, "scan_read", "cannot get value for DVM Reading\n");
			   return DB_NO_ACCESS;
			 }
			 printf("Reading %i: value = %f, %f, %f\n",i,gDVMReadings[0],gDVMReadings[1],gDVMReadings[2]);
			 */

			 //Two readings of the Phidgets!
			 size = sizeof(gPhidget00);
			 status = db_get_value(hDB,hPhidgetVars0,"PH00",&gPhidget00,&size,TID_DOUBLE,FALSE);
			 if(status != DB_SUCCESS)
			 {
			   cm_msg(MERROR, "scan_read", "cannot get value for Phidget0\n");
			   return DB_NO_ACCESS;
			 }

			 //printf("Ph00: (BX, BY, BZ): %.3F, %.3f, %.3f\n",gPhidget00[3],gPhidget00[4],gPhidget00[5]);

			 size = sizeof(gPhidget01);
			 status = db_get_value(hDB,hPhidgetVars1,"PH01",&gPhidget01,&size,TID_DOUBLE,FALSE);
			 if(status != DB_SUCCESS)
			 {
			   cm_msg(MERROR, "scan_read", "cannot get value for Phidget1\n");
			   return DB_NO_ACCESS;
			 }

			 // if more than 99 data points are being taken change the size of the name array
			 char name[6];
			 sprintf(name,"CYC%i",i);
			 bk_create(pevent, name, TID_DOUBLE, &pwdata);
			 *pwdata++ = (double)gbl_current_point;
			 for(m = 0; m < 10; m++){
			   *pwdata++ = (double)gGantryPositions[m]; /* word 4; */
			 }
			 *pwdata++ = (double)gPhidget00[7]; /* word 4; */
			 *pwdata++ = (double)gPhidget01[7]; /* word 4; */
			 ///*pwdata++ = (double)gDVMReadings[0];
			 ///*pwdata++ = (double)gDVMReadings[1];
			 ///*pwdata++ = (double)gDVMReadings[2];

			 //TODO: add other positions (Phidget, laser, ...)


			 *pwdata++ = (double)time_Start_read;
			 bk_close(pevent, pwdata);


			 //[7]: tilt, [3->5]: x-y-z of field, 6 is total field
			 sprintf(name,"PHI%i",i);
			 bk_create(pevent, name, TID_DOUBLE, &phidgdata);
			 *phidgdata++ = (double)gbl_current_point;
			 for(m = 3; m < 7; m++)
			   *phidgdata++ = (double)gPhidget00[m];
			 for(m = 3; m < 7; m++)
			   *phidgdata++ = (double)gPhidget01[m];
			 bk_close(pevent, phidgdata);

			 // for voltage and current field 0->5 relate to the Coil Numbers
			 sprintf(name,"MAG%i",i);
			 bk_create(pevent, name, TID_DOUBLE, &pmagdata);
			 *pmagdata++ = (double)gbl_current_point;
			 for(m = 0; m < 6; m++)
			   *pmagdata++ = (double)gCoilCurrent[m];
			 for(m = 0; m < 6; m++)
			   *pmagdata++ = (double)gCoilVoltage[m];
			 bk_close(pevent, pmagdata);

			 gbl_waiting_measurement = FALSE;
			 gbl_making_move = FALSE;
			 time_Done_read = ss_millitime();

			 read_status = SUCCESS;
			 if (read_status != SUCCESS){
			   if (dddd) printf("Bad read status - skip event\n");
			   return 0; /* No bank generated when read_status is bad */
			 }

			 }



			 if(dddd)printf("scan_read: returning event length=%d\n",bk_size(pevent));
			 printf("  read is done; total cycle took %i ms\n", (time_Done_read -  time_move_next_position) );
			 //TODO: time_Start_motors NOT calculated
			 printf(" Motor move:  %i ms, Keithley read:  %i ms\n",
				( time_Done_cycle - time_Start_motors), (time_Done_read -  time_Start_read) );

				

				return bk_size(pevent);
		       }
		     }
		     //printf("gbl_making_move: %i", gbl_making_move);


		     
		      // check that an EOM has been created beofore you make the next BONM
		     if(gbl_bank_BONM_created == FALSE && gbl_bank_EOM_created == TRUE){
		       gbl_bank_EOM_created = FALSE;
		       //Do this once!
		       bk_init(pevent);

		       // for voltage and current field 0->5 relate to the Coil Numbers
		       char bk_name[5] ="BONM";
		       double* unused_pointer;
		       printf("BONM Created\n");
		       bk_create(pevent, bk_name, TID_DOUBLE, &unused_pointer);
		       *unused_pointer++ = (double)gbl_current_point;
		       bk_close(pevent, unused_pointer);
		       gbl_bank_BONM_created = TRUE;
		       return bk_size(pevent);

		     }



		     //else
		     // if (dddd) printf("scan read - no data to return\n");
		     return 0;
		   }

		   /*-- Calculate scan parameters -------------------------------------*/
		   INT set_scan_params(INT *ninc)
		   {

		     return SUCCESS;
		   }
		   /*-- Keithly stuff   -------------------------*/
		   void user_callback(int chan,double dvalue)
		   {
		   }

		   int user_config(void)
		   {
		     return SUCCESS;
		   }
		   void user_endScan(void)
		   {
		   }
