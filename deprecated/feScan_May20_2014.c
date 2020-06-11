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
HNDLE   hDB=0, hFS=0;
HNDLE   hMDestination=0, hMMove=0;
HNDLE   hMComplexDestination=0,  hMComplexMove=0;
HNDLE   hMoveVariables =0;
HNDLE   hMoveDestination =0;
HNDLE   hMoveStart =0;
HNDLE   hDVMVariables =0;

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
  BOOL    gbl_waiting_measurement = TRUE;
    
  INT     gbl_run_number;      
  
  // Total number of points we will move to
  //INT gbl_total_number_points = 8;
  INT gbl_total_number_points = 13;
  
  // For good test the speeds should be 
  // List of points; little sequence around the 'PMT'
  float points[13][10] = {{-0.35,0.0,0.0,0.0,0.0,-0.10,-0.05,0,-90,0},
			  {-0.25,0.25,0.0,45,0.0,-0.25,0.10,0,-45,0},
			  {0.05,0.33,0.0,90.0,0.0,-0.35,0.2,0,0,0},
			  {0.05,0.25,-0.25,90.0,-45.0,-0.35,0.0,0.0,-45,0},
			  {0.05,-0.08,-0.27,90.0,-85.0,-0.35,0.0,0.2,-45,0},
			  //{0.05,0,-0.27,0.0,-85.0},
			  {-0.25,0,-0.24,0.0,-45.0,-0.35,0.2,0.2,0,0},
			  {-0.35,0.0,0.0,0.0,0.0,-0.35,0.2,0.0,0,0},
			  {-0.25,-0.25,0.0,-45,0.0,-0.35,0.0,0.0,-45,0},
			  {-0.05,-0.32,0.0,-90.0,0.0,-0.35,0.0,0.2,-45,0},
			  {-0.05,-0.25,-0.22,-90.0,-45.0,-0.35,0.2,0.0,0,0},
			  {-0.05,+0.08,-0.27,-90.0,-85.0,-0.35,0.2,0,0,0},
			  //{0.05,0,-0.27,0.0,-85.0},
			  {-0.25,0,-0.24,0.0,-45.0,-0.25,0.10,0,-45,0},
			  {-0.35,0.0,0.0,0.0,0.0,-0.10,-0.05,0,-90,0}
  };


  // Current point we are moving to.
  INT gbl_current_point = 0;
  
  // Called begin_of_run; make sure this is called before any moves happen.
  BOOL gbl_called_BOR = FALSE;

  // Position of gantries, according to feMove front-end.
  float   gGantryPositions[10]; 

  // Destination of gantries, according to feMove front-end.
  float   gGantryDestinations[10]; 

  // DVM readings, according to fedvm front-end.
  double   gDVMReadings[22]; 

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
    0,                    /* log history */
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
  if (status != DB_SUCCESS)
  {
    cm_msg(MERROR, "frontend_init", "cannot retrieve %s (size of fs=%d)", str,size);
    return DB_NO_ACCESS;
  }



  // Get the Move settings and keys
  sprintf(str,"/Equipment/Move/Variables/");
  status = db_find_key(hDB, 0, str , &hMoveVariables);
  if (status != DB_SUCCESS)
  {
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

  // Get the DVM settings and keys
  sprintf(str,"/Equipment/PTFDVM/Variables/");
  status = db_find_key(hDB, 0, str , &hDVMVariables);
  if (status != DB_SUCCESS)
  {
    cm_msg(MERROR, "frontend_init", "cannot get key for %s\n",str);
    return DB_NO_ACCESS;
  }


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
	    printf("Stopping run after all points are done\n",str);
	    status = cm_transition(TR_STOP, 0, str, sizeof(str), TR_SYNC,0);
	    return status;  
	  }
    
	  if(ddd)printf("\nfrontend_loop: starting next move\n");
	  
	  /* Start new move */
	  // We are starting to move; set flag to ensure that nothing else tries to start us moving.
	  gbl_making_move = TRUE;
	  status = move_next_position();
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

  printf("Initial position of gantry (X, Y, Z, R, T): %.3F, %.3f, %.3, %.3f, %.3f\n",gGantryPositions[0],gGantryPositions[1],gGantryPositions[2],gGantryPositions[3],gGantryPositions[4]);




  ss_sleep(1000); /* sleep before starting loop*/

  /* Start cycle */
  first_time = TRUE;
  gbl_current_point = 0;

  // We are starting to move; set flag to ensure that nothing else tries to start us moving.
  gbl_making_move = TRUE;
  status = move_next_position();
  if(status != SUCCESS)
    {
      printf("begin_of_run: failure from move_next_position, calling set_client_flag with FAILURE\n");
      cm_msg(MINFO,"begin_of_run","waiting for someone to stop the run after failure from move_next_position");
      gbl_waiting_for_run_stop =  TRUE;
      cm_yield(1000);
      return FE_ERR_HW;
    }
  
  gbl_called_BOR = TRUE;

  if(ddd)printf("end of begin_of_run\n");
  return CM_SUCCESS;
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
  gGantryDestinations[0] = points[gbl_current_point][0];
  gGantryDestinations[1] = points[gbl_current_point][1];
  gGantryDestinations[2] = points[gbl_current_point][2];
  gGantryDestinations[3] = points[gbl_current_point][3];
  gGantryDestinations[4] = points[gbl_current_point][4];
  gGantryDestinations[5] = points[gbl_current_point][5];
  gGantryDestinations[6] = points[gbl_current_point][6];
  gGantryDestinations[7] = points[gbl_current_point][7];
  gGantryDestinations[8] = points[gbl_current_point][8];
  gGantryDestinations[9] = points[gbl_current_point][9];
  INT i;
  //  for(i = 5; i < 10; i++){
  //gGantryDestinations[i] = 0;
  //}
  //  float pos[10] = {0.03,0.02,0.01,0,0,0,0,0,0,0};
  printf("Now set several destinations at once %.3F %.3F %.3F %.3F %.3F   (%i)\n",gGantryDestinations[0],gGantryDestinations[1],gGantryDestinations[2],gGantryDestinations[3],gGantryDestinations[4],gbl_current_point);
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

  INT timeout = 60000*5; // 5 minute timeout
  while(!finished_moving){
    
    ss_sleep(50);
    status = db_get_value(hDB,hMoveVariables,"Moving",&moving,&size_moving,TID_BOOL,FALSE);    
    status = db_get_value(hDB,hMoveVariables,"Completed",&completed,&size_moving,TID_BOOL,FALSE);
 
    if(completed && !moving)
      finished_moving = TRUE;

    DWORD time_now = ss_millitime();

    if((INT)(time_now-time_start_move) > timeout){
      printf("We have waited %ld ms\n",time_now-time_start_move);
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

  // Return to origin (0,0,0) position!!!

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
   - generate event only when cycle has been completed 
   
*/
{
  double *pwdata;
  double current,time;
  INT read_status, size, status;

  if(dddd)printf("entering scan_read, gbl_waiting_measurement %d\n",gbl_waiting_measurement);
  //fflush(stdout);
  time_Start_read = ss_millitime();
  if (gbl_waiting_measurement)
  {
    sleep(5);
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
    
    printf("Position of gantry (X, Y, Z): %.3F, %.3f, %.3f\n",gGantryPositions[0],gGantryPositions[1],gGantryPositions[2]);


    INT i;
    for(i = 0; i < 2; i++){
      ss_sleep(3000);
      /* 
	 CYCI Bank Contents:
	 
      */
      //    read_status = Kread(&current,&time);
      gDVMReadings;
      size = sizeof(gDVMReadings);
      status = db_get_value(hDB,hDVMVariables,"slot 100",&gDVMReadings,&size,TID_DOUBLE,FALSE);
      if(status != DB_SUCCESS)
	{
	  cm_msg(MERROR, "scan_read", "cannot get value for DVM Reading\n");
	  return DB_NO_ACCESS;
	}
      printf("Reading %i: value = %f, %f, %f\n",i,gDVMReadings[0],gDVMReadings[1],gDVMReadings[2]);
      
      bk_init(pevent);

      bk_create(pevent, "CYC0"+i, TID_DOUBLE, &pwdata);
      *pwdata++ = (double)gbl_current_point;
      *pwdata++ = (double)gGantryPositions[0]; /* word 4; */
      *pwdata++ = (double)gGantryPositions[1]; /* word 4; */
      *pwdata++ = (double)gGantryPositions[2]; /* word 4; */
      *pwdata++ = (double)gDVMReadings[0]; 
      *pwdata++ = (double)gDVMReadings[1]; 
      *pwdata++ = (double)gDVMReadings[2]; 
      *pwdata++ = (double)time_Start_read;
      bk_close(pevent, pwdata);
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
    printf("  read is done; total cycle took %ld ms\n", (time_Done_read -  time_move_next_position) );
    printf(" Motor move:  %ld ms, Keithley read:  %ld ms\n", 
	   ( time_Done_cycle - time_Start_motors), (time_Done_read -  time_Start_read) );    
    
    return bk_size(pevent);
  }
  else
    if (dddd) printf("scan read - no data to return\n");
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
