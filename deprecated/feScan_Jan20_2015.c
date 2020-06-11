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
  HNDLE   hMotors00 = 0, hMotors01 = 0;
  //HndlehDVMVariables =0;

  extern  int run_state;

  char    expt_name[32];

  /* These variables can be changed to control the level of debugging messages.
  They should really be collapsed into one variable placed in the ODB */
  INT     dd = 1, ddd = 1, dddd=1, ddddd = 1;

  /*
  ddd frontend_loop, begin_of_run
  dddd indicates if running in cycle in frontend loop
  dd = scan info
  */

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

  // PMT readings captured by fevme, not here
  
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
     must be less than MAX_EVENT_SIZE (include/midas.h)
     which is 4MB or 0x400000 or 4e6.
  */
  INT max_event_size = 100000;//10000;
  
  /* buffer size to hold events */
  INT event_buffer_size = 10*100000; //10*10000;
  
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
  
  //Declare Variables
  BOOL watchdog_flag;
  INT status, rstate, size;
  
  gbl_making_move = FALSE;
  
  /* get the experiment name */
  size = sizeof(expt_name);
  status = db_get_value(hDB, 0, "/experiment/Name", &expt_name, &size, TID_STRING, FALSE);
  printf("feScan Front End code for experiment %s now running ... \n",expt_name);
  
  /* get basic handle for experiment ODB */
  status=cm_get_experiment_database(&hDB, NULL);
  if(status != CM_SUCCESS) {
    cm_msg(MERROR,"frontend_init","Not connected to experiment");
    return CM_UNDEF_EXP;
  }
  
  /* check for current run state; if not stopped, stop it */
  size = sizeof(rstate);
  status = db_get_value(hDB, 0, "/runinfo/State", &rstate, &size, TID_INT, FALSE);
  if(status != DB_SUCCESS) {
    cm_msg(MERROR,"frontend_init","cannot GET /runinfo/State");
    return FE_ERR_ODB;
  }
  if (rstate != STATE_STOPPED) {
    cm_msg(MERROR,"frontend_init",
	   "!!! Run In Progress: Stop the run, shutdown and restart this program !!!");
    return (FE_ERR_HW);
  }
  
  /* get handles of equipment records   */
  /* Get Scan  settings */
  status = db_find_key(hDB, 0, "/Equipment/Scan/Settings/", &hFS);
  if(status != DB_SUCCESS) {
    printf("Record %s  does not exist. It will be created.\n","/Equipment/Scan/Settings/");
    status = db_create_record(hDB, 0, "/Equipment/Scan/Settings/", strcomb(scan_settings_str));  //strcomb : see odb.c
  }
  size = sizeof(fs);
  status = db_get_record(hDB, hFS, &fs, &size, 0);
  if (status != DB_SUCCESS){
    cm_msg(MERROR, "frontend_init", "cannot retrieve %s (size of fs=%d). Try again?", "/Equipment/Scan/Settings/",size);
    return DB_NO_ACCESS;
  }

  // Obtain the ODB keys for various variables
  // Note: when adding or removing variables to/from this list change the variable num_entires to change the size of the arrays used
  //************** BK Change :Start
  const int num_entries = 9;
  char str[num_entries][128];
  HNDLE * ODB_Handles[num_entries];
  //Move settings
  sprintf(str[0],"/Equipment/Move/Variables/");
  ODB_Handles[0] = &hMoveVariables;
  sprintf(str[1], "/Equipment/Move/Control/Destination");
  ODB_Handles[1] = &hMoveDestination;  
  sprintf(str[2], "/Equipment/Move/Settings/");
  ODB_Handles[2] = &hMoveSettings;  
  sprintf(str[3], "/Equipment/Move/Control/Start Move");
  ODB_Handles[3] = &hMoveStart;
  //Phidget Vars
  sprintf(str[4],"/Equipment/Phidget00/Variables/");
  ODB_Handles[4] = &hPhidgetVars0;
  sprintf(str[5],"/Equipment/Phidget01/Variables/");
  ODB_Handles[5] = &hPhidgetVars1;
  //Wiener Power Supply
  sprintf(str[6],"/Equipment/PtfWiener/Variables/");
  ODB_Handles[6] = &hPtfWiener;
  //Motors
  sprintf(str[7],"/Equipment/Motors00/Settings/TurnMotorsOff");
  ODB_Handles[7] = &hMotors00;
  sprintf(str[8],"/Equipment/Motors01/Settings/TurnMotorsOff");
  ODB_Handles[8] = &hMotors01;
  
  // Get the above ODB Keys and produce error if unsuccessful
  int i;
  for(i = 0; i<num_entries; i++){
    status = db_find_key(hDB, 0, str[i] , ODB_Handles[i]);
    if (status != DB_SUCCESS){
      cm_msg(MERROR, "frontend_init", "cannot get key for %s\n",str[i]);
      return DB_NO_ACCESS;
    }
  }
  //************** BK Change :Stop
  
  //Reset watchdog and Alarms
  cm_get_watchdog_params(&watchdog_flag, &gbl_watchdog_timeout);
  printf("frontend_init: current watchdog parameters are gbl_watchdog_timeout=%d; watchdog_flag=%d\n",	gbl_watchdog_timeout, watchdog_flag);
  
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
  - Main function for starting a new move, if we are running and
  the measurement for the last move has finished..
  BK: scan_read is called after Frontend Loop completes. This is why at certain spots you see return SUCCESS. This allows scan_read to be called and complete various tasks
*/
INT frontend_loop(){
  INT status ;
  char str[128];
  
  // Only want to start checking if the begin_of_run has been called.
  if(!gbl_called_BOR) return SUCCESS;
  
  if (run_state == STATE_RUNNING){ 
    /*       RUNNING
	     check if cycle is finished AND histo update being done */
    
    if (!gbl_making_move){ /* not in cycle, wait until equipment has run */
      if ( gbl_waiting_measurement ) {
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
      // BONM is a bank that is the MIDAS file that is there for analysis purposes. BONM and EOM can be used to tell if data was taken while the gantries were moving or not. BONM is created in scan_read()
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
    else  { //we are already moving; (i.e. gbl_making_move is TRUE) nothing to do 
      if(ddddd)
      printf("frontend_loop: running in cycle; nothing to do; exit\n");
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
  BOOL moving;
  INT status, size;
  
  
  //variables used to check that phidgets are connected and working
  BOOL Check_Phidgets = TRUE; // change this to false to skip phidget check. This should only be done if you are running a scan without using the phidgets
  double phidget_Values_Old[10];
  double phidget_Values_Now[10];
  int size_of_array = sizeof(phidget_Values_Old);
  DWORD time_at_start_of_check;
  
  gbl_transition_requested=FALSE; /* used for deferred transition */
  gbl_waiting_for_run_stop = FALSE; /* unrecoverable error */
  gbl_run_number = run_number;
  gbl_making_move = FALSE;
  gbl_first_call = TRUE; /* for prestop */
  
  printf("Start begin of run\n"); 
  
  // Ensure that we are not already running.
  size = sizeof(moving);
  status = db_get_value(hDB,hMoveVariables,"Moving",&moving,&size,TID_BOOL,FALSE);
  if(status != DB_SUCCESS){
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
  if(status != DB_SUCCESS){
    cm_msg(MERROR, "begin_of_run", "cannot get value for Position\n");
    return DB_NO_ACCESS;
  }
  
  // Get the gantry limits for checking parameters.
  size = sizeof(gGantryLimits);
  status = db_get_value(hDB,hMoveSettings,"Limit Positions",&gGantryLimits,&size,TID_FLOAT,FALSE);
  if(status != DB_SUCCESS){
    cm_msg(MERROR, "begin_of_run", "cannot get value for Limits\n");
    return DB_NO_ACCESS;
  }
  
  //checks that the phidgets are connected by checking that the values are fluctuating. When the phidgets are unplugged the ODB values are constant with no fluctuation. 
  // TF: Yes, but should be true for ALL values, some values *can* stay the same for 5 sec.
  if(Check_Phidgets){
    
    size_of_array = sizeof(phidget_Values_Old);
    db_get_value(hDB,hPhidgetVars0,"PH00",&phidget_Values_Old,&size_of_array,TID_DOUBLE,FALSE);
    db_get_value(hDB,hPhidgetVars0,"PH00",&phidget_Values_Now,&size_of_array,TID_DOUBLE,FALSE);
    
    //Check that phidget0 is working. The phidget is working if the values change over a set period of time. When a phidget is unplugged there is no fluctuation in the values.
    time_at_start_of_check = ss_millitime();
    while( (phidget_Values_Old[0] == phidget_Values_Now[0]) &&(phidget_Values_Old[1] == phidget_Values_Now[1]) &&(phidget_Values_Old[2] == phidget_Values_Now[2]) &&(phidget_Values_Old[3] == phidget_Values_Now[3]) &&(phidget_Values_Old[4] == phidget_Values_Now[4]) &&(phidget_Values_Old[5] == phidget_Values_Now[5]) &&(phidget_Values_Old[6] == phidget_Values_Now[6]) &&(phidget_Values_Old[7] == phidget_Values_Now[7])&&(phidget_Values_Old[8] == phidget_Values_Now[8]) &&(phidget_Values_Old[9] == phidget_Values_Now[9]) ){
      
      db_get_value(hDB,hPhidgetVars0,"PH00",&phidget_Values_Now,&size_of_array,TID_DOUBLE,FALSE);
      
      // If 5 seconds pass without any of the values changing assume that the phidgets are not connected properly
      if(ss_millitime()-time_at_start_of_check > 1000*5){
	printf("The readings from Phidget0 are not changing so it is assumed that Phidget0 is not working\n");
	//return DB_NO_ACCESS ;  //TODO : only comment if know that Phidget IS unplugged!
	break;
      }
      
      ss_sleep(1000); //wait a sec
    }
    
    //Check that the phidget1 is  working.
    db_get_value(hDB,hPhidgetVars1,"PH01",&phidget_Values_Old,&size_of_array,TID_DOUBLE,FALSE);
    db_get_value(hDB,hPhidgetVars1,"PH01",&phidget_Values_Now,&size_of_array,TID_DOUBLE,FALSE);
    
    time_at_start_of_check = ss_millitime();
    while( (phidget_Values_Old[0] == phidget_Values_Now[0]) &&(phidget_Values_Old[1] == phidget_Values_Now[1]) &&(phidget_Values_Old[2] == phidget_Values_Now[2]) &&(phidget_Values_Old[3] == phidget_Values_Now[3]) &&(phidget_Values_Old[4] == phidget_Values_Now[4]) &&(phidget_Values_Old[5] == phidget_Values_Now[5]) &&(phidget_Values_Old[6] == phidget_Values_Now[6]) &&(phidget_Values_Old[7] == phidget_Values_Now[7])&&(phidget_Values_Old[8] == phidget_Values_Now[8]) &&(phidget_Values_Old[9] == phidget_Values_Now[9]) ){
      
      db_get_value(hDB,hPhidgetVars1,"PH01",&phidget_Values_Now,&size_of_array,TID_DOUBLE,FALSE);
      
      // If 5 seconds pass without any of the values changing assume that the phidgets are not connected properly
      if(ss_millitime()-time_at_start_of_check > 1000*5){
	printf("The readings from Phidget1 are not changing so it is assumed that Phidget1 is not working\n");
	return DB_NO_ACCESS ;
      }
    }
    printf("Both phidgets are working properly\n");
  }
  
  
  status = gen_scan_path();
  int i;
  for(i = 0; i< gbl_total_number_points; i++){
    printf("Point %i Gantry0: %.4F %.4F %.4F %.4F %.4F, Gantry1: %.4F %.4F %.4F %.4F %.4F\n",i,points[i][0],points[i][1],points[i][2],points[i][3],points[i][4],points[i][5],points[i][6],points[i][7],points[i][8],points[i][9]);
  }
  
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
  
  gbl_called_BOR = TRUE;
  
  if(ddd)printf("end of begin_of_run\n");
  return CM_SUCCESS;
}

/*-- Generate path for scan-----------------------------------------*/
BOOL gen_scan_path(void)
{
  INT j;
  INT point_num = 0;
  
  // For cylindrical scan
  int layer;
  int loop;
  int waypoint;
  float PI = 3.14159265;
  float z_max_value = 0.54;
  // For linear/plane/rectangular prism scan
  int z_layer;
  int x_layer;
  int y_layer;
  int x_increment;
  int x_limit;
  
  //for HK demo
  float scan_init_y=0.;
  float scan_step_y=0.05;
  float scan_width_y=0.666;
  INT rot_i;

  int prev_max_size = -1;
  
  // Generate path based on scan parameters:
  switch(fs.scan_type){
  case CYLINDER:
    /*
     * Cylindrical sequence (for magnetic field scans)
     * Not adjusted for two gantries
     * Not tested again: need to be fixed and debugged
     */
    // Parameter checks
     if(fs.cyl_par.height >= z_max_value){   //max Z is not set in gGantryLimits because position at limit Z = 0!
	printf("Error: Height outside of limits.\n");
	return 0;
    }
    float min = 0.;
    
    // Find min distance to edges (different for both gantries)
    if(gGantryLimits[5] < gGantryLimits[6])
      min = gGantryLimits[5];
    else
      min = gGantryLimits[6];
    
    if(fs.cyl_par.radius >= min/2){
      printf("Error: Radius outside of limits.\n");
      return 0;
    }
    if(fs.cyl_par.loop_separation < 0.001){
      printf("Error: Loop separation too small for motors (min resolution = 1 mm).\n");
      return 0;
    }
    if(fs.cyl_par.layer_thickness < 0.001){
      printf("Error: Layer thickness too small for motors (min resolution = 1 mm).\n");
      return 0;
    }
    if(fs.cyl_par.arc_step < 0.001){
      printf("Error: Arc step size too small for motors (min resolution = 1 mm).\n");
      return 0;
    }
    printf("Generating cylindrical path...\n");
    // Generate layers
    for(layer=0;layer<=(trunc(fs.cyl_par.height/fs.cyl_par.layer_thickness));layer++){
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
      points[point_num][0] = 0 + fs.cyl_par.x_center;
      points[point_num][1] = 0 + fs.cyl_par.y_center;
      points[point_num][2] = fs.cyl_par.height - (layer*fs.cyl_par.layer_thickness);
      
      //Don't change the destination!
      INT k;
      for(k=3; k < 10; k++){
	points[point_num][k] = -99999;
      }
      point_num = point_num + 1;
      
      // Generate loops: for each Z, loop through X and Y in a circle
      for(loop = 1;loop <= (trunc(fs.cyl_par.radius/fs.cyl_par.loop_separation));loop++){
	for(waypoint = 0; waypoint <= trunc((2*PI*loop*fs.cyl_par.loop_separation)/fs.cyl_par.arc_step) ; waypoint++){
	  
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
	  float theta = (2*PI)*((fs.cyl_par.arc_step*waypoint)/(2*PI*loop*fs.cyl_par.loop_separation));
	  points[point_num][0] = (loop*fs.cyl_par.loop_separation)*cos(theta) + fs.cyl_par.x_center;
	  points[point_num][1] = (loop*fs.cyl_par.loop_separation)*sin(theta) + fs.cyl_par.y_center;
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
    point_num = point_num + 1;
    
    printf("%i - %.4F %.4F %.4F \n",point_num,points[point_num][0],points[point_num][1],points[point_num][2]);
    printf("%i - %.4F %.4F %.4F \n",point_num,points[point_num][5],points[point_num][6],points[point_num][7]);
    
    break;
  case RECTANGULAR:
    /*
     * Rectangular sequence for magnetic field scans
     * Works for two gantries and many layers.
     * Well tested!
     */
    
    // Parameter checks
    if((fs.rect_par.init_pos_z + fs.rect_par.prism_height_z) > z_max_value) {
      printf("Error: Height (+ initial z position) outside of limits.\n");
      return 0;
    }
    
    if((fs.rect_par.init_pos_z ) < fabs(gGantryLimits[2])){
      printf("Error: initial z position outside of limits.\n");
      return 0;
    }
    if((fs.rect_par.init_pos_x + fs.rect_par.prism_length_x) > fabs(gGantryLimits[5])){
      printf("Error: Length (+ initial x position) outside of limits.\n");
      return 0;
    }
    if((fs.rect_par.init_pos_x) < fabs(gGantryLimits[0])){
      printf("Error: initial x position outside of limits.\n");
      return 0;
    }
    if((fs.rect_par.init_pos_y + fs.rect_par.prism_width_y) > (gGantryLimits[6])){
      printf("Error: Width (+ initial y position) outside of limits.\n");
      return 0;
    }
    if((fs.rect_par.init_pos_y) < (gGantryLimits[1])){
      printf("Error: initial y position outside of limits.\n");
      return 0;
    }
    if(fs.rect_par.z_step < 0.001){
      printf("Error: Z step size too small for motors (min resolution = 1 mm).\n");
      return 0;
    }
    if(fs.rect_par.x_step < 0.001){
      printf("Error: X step size too small for motors (min resolution = 1 mm).\n");
      return 0;
    }
    if(fs.rect_par.y_step < 0.001){
      printf("Error: Y step size too small for motors (min resolution = 1 mm).\n");
      return 0;
    }
    printf("Generating linear/plane/rectangular prism path...\n");
    // Generate path
    
    for(z_layer=0;z_layer<=(trunc(fs.rect_par.prism_height_z/fs.rect_par.z_step));z_layer++){

      //scan for gantry 0
      if(fs.rect_par.which_gantry == GANTRY0 || fs.rect_par.which_gantry == BOTH){
	for(y_layer=0;y_layer<=(trunc(fs.rect_par.prism_width_y/fs.rect_par.y_step));y_layer++){
	  
	  if((y_layer % 2) == 0){
	    x_layer = 0;
	    x_increment = 1;
	    x_limit = trunc(fs.rect_par.prism_length_x/fs.rect_par.x_step)+1;
	  }
	  else{
	    x_layer = trunc(fs.rect_par.prism_length_x/fs.rect_par.x_step);
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
	    
	    points[point_num][0] = fs.rect_par.init_pos_x + (x_layer*fs.rect_par.x_step);
	    points[point_num][1] = fs.rect_par.init_pos_y + (y_layer*fs.rect_par.y_step);
	    points[point_num][2] = fs.rect_par.init_pos_z + (z_layer*fs.rect_par.z_step);
	    points[point_num][3] = fs.rect_par.init_rotation; //-90;       //No idea why BK set this to -80;
	    points[point_num][4] = fs.rect_par.init_tilt;
	    // TODO: STRANGE, makes MORE sense with the IF condition
	    if(fs.rect_par.which_gantry == BOTH)
	      points[point_num][7] = fs.rect_par.init_pos_z + (z_layer*fs.rect_par.z_step);  
	    points[point_num][8] = fs.rect_par.init_rotation;//-90;       //No idea why BK set this to 100;
	    points[point_num][9] = fs.rect_par.init_tilt;
	    
	    if(points[point_num][1] > 0.3){
	      
	      points[point_num][5] = gGantryLimits[5];
	      points[point_num][6] = 0.;
	      
	    }
	    else{
	      points[point_num][5] = gGantryLimits[5];
	      points[point_num][6] = gGantryLimits[6];
	    }
	    
	    point_num = point_num + 1;
	    
	  }
	}//end loop over y_layer for gantry0
      }
      
      // scan for gantry 1 (TODO: too much duplicated code, can be done more efficient!)
      if(fs.rect_par.which_gantry == GANTRY1 || fs.rect_par.which_gantry == BOTH){
	for(y_layer=0;y_layer<=(trunc(fs.rect_par.prism_width_y/fs.rect_par.y_step));y_layer++){
	  
	  if((y_layer % 2) == 0){
	    x_layer = 0;
	    x_increment = 1;
	    x_limit = trunc(fs.rect_par.prism_length_x/fs.rect_par.x_step)+1;
	  }
	  else{
	    x_layer = trunc(fs.rect_par.prism_length_x/fs.rect_par.x_step);
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
	    // TODO: STRANGE
	    if(fs.rect_par.which_gantry == BOTH)
	      points[point_num][2] =  fs.rect_par.init_pos_z + (z_layer*fs.rect_par.z_step); 
	    points[point_num][3] = fs.rect_par.init_rotation;//-90;       //No idea why BK set this to -80;
	    points[point_num][4] = fs.rect_par.init_tilt;
	    points[point_num][5] = fs.rect_par.init_pos_x + (x_layer*fs.rect_par.x_step);
	    points[point_num][6] = fs.rect_par.init_pos_y + (y_layer*fs.rect_par.y_step);
	    points[point_num][7] = fs.rect_par.init_pos_z + (z_layer*fs.rect_par.z_step);
	    points[point_num][8] = fs.rect_par.init_rotation;//-90;       //No idea why BK set this to 100;
	    points[point_num][9] = fs.rect_par.init_tilt;
	    
	    if(points[point_num][6] > 0.3){
	      
	      points[point_num][0] = 0.;
	      points[point_num][1] = 0.;
	    }
	    else{
	      points[point_num][0] = 0.;
	      points[point_num][1] = gGantryLimits[6];
	    }
	    
	    point_num = point_num + 1;
	  }
	}//end loop over Y layer for second gantry
      }//end if
    }//end loop over Z layer for both gantries
    
    // TODO: uncomment:
    /* Move Gantry1 to PMT position for long term scan in history.
    for(j=0;j<5;j++){
      points[point_num][j] = 0.0;
    }
    //gantry 1: //only for FIELD SCAN with extension arm
    points[point_num][5] = 0.4;
    points[point_num][6] = 0.41;
    points[point_num][7] = 0.3;
    points[point_num][8] = -90;
    points[point_num][9] = 0.0;
    */
    // Add end point: back to their corner!
    /* for(j=0;j<5;j++){
       points[point_num][j] = 0.0;
    }
    //gantry 1:
    points[point_num][5] = gGantryLimits[5];
    points[point_num][6] = gGantryLimits[6];
    points[point_num][7] = 0.0;
    points[point_num][8] = -90;
    points[point_num][9] = 0.0;*/
    
    
    break;
    
  case DEMO: 
    /*
     * Demo sequence for HK Workshop
     */
    
    // scan for gantry 0
    // for(y_layer=0;y_layer<=(trunc(scan_width_y/scan_step_y));y_layer++){
    for(rot_i = 0;rot_i <= 200;rot_i++){
      float rotation = -100 + rot_i;
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
    
    break;
  case MANUAL:
    /*
     * Manual sequences and debug sequences
     */


    // Total number of points we will move to
    //    point_num =284;//13;
    point_num =2;
    max_size =  point_num+1;
    
    // List of points; little sequence around the 'PMT'
    // sequence for debugging gantry 1
    float sequence[3][10] = {{0.,0.,0.,-90,0.,0.647,0.61,0.,-90.,0.},
			     {0.,0.,0.,-90,0.,0.37,0.31,0.,-90.,0.},
			     {0.,0.,0.,-90,0.,0.47,0.31,0.,-90.,0.}};
    
    
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
    

  case ALIGNMENT:
    // Parameter checks
    if(fs.align_par.gantry_laser > 1){
      printf("Error: choose Gantry 0 or Gantry 1 as gantry with the emitting source");
      return 0;
    }

    // Create some initial space
    max_size = 1;
    points = (float**)realloc(points, sizeof(float *)*max_size);
    for(j = 0;j < max_size; j++){
      points[j] = (float *) calloc(10,sizeof(float));
    }

    
    //STEP 1: set both gantries at initial position 
    // NOTE : Do I need init_pos as scan setting? Yes, because the scan sequence will call feMove where it defines
    //        its destinations.
    points[0] = fs.align_par.init_pos; 
    point_num = 1;
    //STEP 2: go to corner of grid and start grid scan
    // Start with a simple situation: both tilts horizontal, otherwise do some trigonometry
    int gan_i = 0;
    if(fs.align_par.gantry_laser == GANTRY1)
      gan_i = 1;
    
    int i_len = 0;
    int i_width = 0;
    int len_steps = trunc(fs.align_par.grid_length/fs.align_par.step_length);
    int width_steps = trunc(fs.align_par.grid_width/fs.align_par.step_width);
    int direction = 1;
    while(i_width <= width_steps){
      while(i_len <= len_steps){
	
	//if necessary make array larger
	if (point_num == max_size) {
	  prev_max_size = max_size;
	  max_size =! max_size ? 1 : max_size << 1;
	  points = (float**)realloc(points, sizeof(float *)*max_size);
	  
	  for(j = prev_max_size;j < max_size; j++){
	    points[j] = (float *) calloc(10,sizeof(float));
	  }
	}
	
	//figure out whether to move in x or in y
	int x_or_y = 0;
	if(abs(fs.align_par.init_pos[0] - fs.align_par.init_pos[5]) > 0.1){
	  printf("From initial position assumed that gantries are opposite at fixed Y");
	  x_or_y = 1;
	}

	points[point_num][x_or_y + gan_i*5] = (fs.align_par.init_pos[x_or_y + gan_i*5] - direction * fs.align_par.grid_length/2.) + direction*i_len*fs.align_par.step_length;
	
	points[point_num][2 + gan_i*5] = (fs.align_par.init_pos[2 + gan_i*5] - fs.align_par.grid_width/2.) + i_width*fs.align_par.step_width;
	i_len++;
	point_num++;
	
      }
      direction = -1* direction;
      i_width++;
    }

    //if necessary make array larger
    if (point_num == max_size) {
      prev_max_size = max_size;
      max_size =! max_size ? 1 : max_size << 1;
      points = (float**)realloc(points, sizeof(float *)*max_size);
      
      for(j = prev_max_size;j < max_size; j++){
	points[j] = (float *) calloc(10,sizeof(float));
      }
    }
    
    //STEP 3: back to initial position, but go to corner in solid angle grid, start scan
    points[point_num] = fs.align_par.init_pos; 
    point_num++;

    // Second move in solid angle space, delta theta vs delta tilt
    // TODO: cleanup below with less copy-pasting from above code..., maybe 30 lines of code can be much shorter...
    int i_theta = 0;
    int i_phi = 0;
    int theta_steps = trunc(fs.align_par.delta_rotation/fs.align_par.step_rotation);
    int phi_steps = trunc(fs.align_par.delta_tilt/fs.align_par.step_tilt);
    direction = 1;
    while(i_phi <= phi_steps){
      while(i_theta <= theta_steps){
	
	//if necessary make array larger
	if (point_num == max_size) {
	  prev_max_size = max_size;
	  max_size =! max_size ? 1 : max_size << 1;
	  points = (float**)realloc(points, sizeof(float *)*max_size);
	  
	  for(j = prev_max_size;j < max_size; j++){
	    points[j] = (float *) calloc(10,sizeof(float));
	  }
	}
	
	points[point_num][3 + gan_i*5] = (fs.align_par.init_pos[3 + gan_i*5] - direction * fs.align_par.delta_rotation/2.) + direction*i_theta*fs.align_par.step_rotation;
	
	points[point_num][4 + gan_i*5] = (fs.align_par.init_pos[4 + gan_i*5] - fs.align_par.delta_tilt/2.) + i_phi*fs.align_par.step_tilt;
	i_theta++;
	point_num++;
	
      }
      direction = -1* direction;
      i_phi++;
    }
    
    break;
  case PASS_BY:
    //check Parameters
    if(fs.pass_by_par.gantry_laser > 1){
      printf("Error: choose Gantry 0 or Gantry 1 as gantry with the emitting source");
      return 0;
    }

    //STEP 1: set initial position of laser box
    // Create some initial space
    max_size = 1;
    points = (float**)realloc(points, sizeof(float *)*max_size);
    for(j = 0;j < max_size; j++){
      points[j] = (float *) calloc(10,sizeof(float));
    }

    gan_i = 0;
    int ij = 0;
    if(fs.align_par.gantry_laser == GANTRY1)
      gan_i = 1;
    
    while(ij < 5){
      points[0][ij + gan_i*5] = fs.pass_by_par.init_pos[ij]; 
      points[0][ij + (!gan_i)*5]   = gGantryLimits[ij+ (!gan_i)*5];
      ij++;
    }
    point_num = 1;    

    // TODO !!!

    // STEP 2: move receiver PMT N times between limit and laser position (NOTE : correct for 2 inch distance between
    //         PMT and laser if needed)



    // STEP 3: move N times between opposite limit and requested position

    break;
  case ACCEPTANCE:
    // General idea: - based on rectangular OR cylindrical for sequence in XYZ
    //               - in rectangular I added the option for a fixed tilt and rotation
    //                 which can give a cross-sectional view of the PMT from one tilt angle
    //               - Here the default should be at normal incidence wrt to PMT surface.
    //               - Additional options: for fixed point on PMT: several incidence angle (for angular acceptance!)
    //                 the latter is the hardest one...
    //               - Need model for PMT surface: seems pretty spherical, just need center of sphere and radius of curvature!
    //               - Gradient of spherical surface function evaluated at a point on the sphere gives its normal vector.

    // 











    printf("Error: Not yet implemented.\n");
    break;
  case REFLECTIVITY:
    printf("Error: Not yet implemented.\n");
    break;

  default:
    printf("Error: Invalid scan type.\n");
    return 0;
    
  }//end switch case
  
  // Insert path into global variable/array
  gbl_total_number_points = point_num;
  
  
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
  /* UNUSED ! (old copy-paste code??)
  BOOL new_supercycle;
  BOOL fmove = 0 ;
  INT fmove_readback[2] ={0,0} ;
  INT fmove_stat;
  */
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
  time_Start_motors = ss_millitime();
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
  printf("Bad dest: %i \n",bad_destination);
  if(!bad_destination){
    //Switch off motors through hotlink with cd_galil to perform minimum noise measurement
    //Galil motors increase noise in PMT signals otherwise.
    
    BOOL turn_off = TRUE;
    db_set_data(hDB,hMotors00,&turn_off,sizeof(BOOL),1,TID_BOOL);
    db_set_data(hDB,hMotors01,&turn_off,sizeof(BOOL),1,TID_BOOL);
    
    ss_sleep(50);
    gbl_waiting_measurement = TRUE;
  }
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

  if (gbl_waiting_measurement){
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
      
    } else {
      time_Start_read = ss_millitime();
      //Accumulate PMT data during this sleep after the EOM, and before a new BONM, through a separate stream of banks
      ss_sleep(fs.meas_time);   
      if (ddd)
	printf("scan_read - make bank\n ");
      
      if (dddd){ printf("scan read: current %e, time %g\n",current, time);    }
            
      size = sizeof(gGantryPositions);
      status = db_get_value(hDB,hMoveVariables,"Position",&gGantryPositions,&size,TID_FLOAT,FALSE);
      if(status != DB_SUCCESS) {
	cm_msg(MERROR, "scan_read", "cannot get value for Position\n");
	return DB_NO_ACCESS;
      }
      
      printf("Position of gantry 0: (X, Y, Z): %.3F, %.3f, %.3f\n",gGantryPositions[0],gGantryPositions[1],gGantryPositions[2]);
      printf("Position of gantry 1: (X, Y, Z): %.3F, %.3f, %.3f\n",gGantryPositions[5],gGantryPositions[6],gGantryPositions[7]);
	
      size = sizeof(gCoilCurrent);
      status = db_get_value(hDB,hPtfWiener,"current",&gCoilCurrent,&size,TID_FLOAT,FALSE);
      if(status != DB_SUCCESS) {
	cm_msg(MERROR, "scan_read", "cannot get value for Coil Current\n");
	return DB_NO_ACCESS;
      }
      
      size = sizeof(gCoilVoltage);
      status = db_get_value(hDB,hPtfWiener,"senseVoltage",&gCoilVoltage,&size,TID_FLOAT,FALSE);
      if(status != DB_SUCCESS) {
	cm_msg(MERROR, "scan_read", "cannot get value for Coil Voltage\n");
	return DB_NO_ACCESS;
      }
      
      //Init bank creation once
      bk_init(pevent);
      /* CYCI Bank Contents: 1 per measuring point! */
      bk_create(pevent, "CYC0", TID_DOUBLE, &pwdata);
      *pwdata++ = (double)gbl_current_point;
      for(m = 0; m < 10; m++){
	*pwdata++ = (double)gGantryPositions[m]; /* word 4; */
      }
      *pwdata++ = (double)time_Start_read;
      bk_close(pevent, pwdata);
      
      // for voltage and current field 0->5 relate to the Coil Numbers
      // TF: NOT anymore!!!
      bk_create(pevent, "MAG0", TID_DOUBLE, &pmagdata);
      *pmagdata++ = (double)gbl_current_point;
      for(m = 0; m < 40; m++)
	*pmagdata++ = (double)gCoilCurrent[m];
      for(m = 0; m < 40; m++)
	*pmagdata++ = (double)gCoilVoltage[m];
      bk_close(pevent, pmagdata);

      /* replace by same structure as ADC, as banks are written out already
      INT i;
      //int number_of_data_points=30;                //TF: this is too much and not allowed by mfe.c:1221 !!
      // Results in big memory issues!!
      int number_of_data_points=5;                   
      for(i = 0; i < number_of_data_points ; i++) {
	ss_sleep(100);
	
	//Two readings of the Phidgets!
	size = sizeof(gPhidget00);
	status = db_get_value(hDB,hPhidgetVars0,"PH00",&gPhidget00,&size,TID_DOUBLE,FALSE);
	if(status != DB_SUCCESS){
	  cm_msg(MERROR, "scan_read", "cannot get value for Phidget0\n");
	  return DB_NO_ACCESS;
	}
	
	//printf("Ph00: (BX, BY, BZ): %.3F, %.3f, %.3f\n",gPhidget00[3],gPhidget00[4],gPhidget00[5]);
	
	size = sizeof(gPhidget01);
	status = db_get_value(hDB,hPhidgetVars1,"PH01",&gPhidget01,&size,TID_DOUBLE,FALSE);
	if(status != DB_SUCCESS){
	  cm_msg(MERROR, "scan_read", "cannot get value for Phidget1\n");
	  return DB_NO_ACCESS;
	}
		
	//[7]: tilt, [3->5]: x-y-z of field, 6 is total field
	sprintf(name,"PHI%i",i);
	bk_create(pevent, name, TID_DOUBLE, &phidgdata);
	*phidgdata++ = (double)gbl_current_point;
	for(m = 3; m < 7; m++)
	  *phidgdata++ = (double)gPhidget00[m];
	for(m = 3; m < 7; m++)
	  *phidgdata++ = (double)gPhidget01[m];
	bk_close(pevent, phidgdata);
 		
      }//end loop over data points
      */
      gbl_waiting_measurement = FALSE;
      gbl_making_move = FALSE;
      
      //switch the turn_off hotlink back to false
      BOOL turn_off = FALSE;
      db_set_data(hDB,hMotors00,&turn_off,sizeof(BOOL),1,TID_BOOL);
      db_set_data(hDB,hMotors01,&turn_off,sizeof(BOOL),1,TID_BOOL);
      time_Done_read = ss_millitime();
      
      read_status = SUCCESS;
      if (read_status != SUCCESS){
	if (dddd) printf("Bad read status - skip event\n");
	return 0; /* No bank generated when read_status is bad */
      }
            
      if(dddd)printf("scan_read: returning event length=%d\n",bk_size(pevent));
      printf("-->read is done; total cycle took %i ms\n", (time_Done_read -  time_move_next_position) );
      printf("-->Motor move:  %i ms, read:  %i ms\n",
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
