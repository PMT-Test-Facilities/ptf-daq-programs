/********************************************************************	\
  !!!!NOTE: NOT THE MOST CURRENT COLLISION AVOIDANCE CODE!!!!!

  Name:         feMove.c 
  Created by:   Nils

  Contents:     Motion control frontend for PTF gantries. Implements
                single move functionality of the gantry including path
                planning and collision avoidance. Control of this
                frontend takes place in the ODB under /Equipment/Move.
  
  Debug Log:    While initalizing, the command to send the y axis to its
                limit is not registered. It is sent with channel_rw, but
                feMotor doesn't pick it up.
  
  Further work: 1. Add geometry structure to pInfo that includes all
                   required geometry of the PTF. Use this for collision
                   detection, translation functions.
                2. Translation functions between global and local
                   coordinate frames. Use these to make "Destination"
                   and "Location" ODB variables refer to global 
                   coordinates (while the rest of the code works in
                   local coordinates).
                3. Collision detection functions that determine if a
                   set of local coordinates results in a collision.
                4. Add collision detection check to monitor()
                5. Path generator that can generate a series of
                   collision free waypoints to get to the destination 
                6. Add a set of names for each axes (ie "Gantry 0, X", "Gantry 1, rotary") so t
                   that debug messages are more helpful.
                7. Move the limit disabling functionality to a seperate 
                   ODB variable.

  Revision history
  -------------------------------------------------------------------
  date         by    modification
  ---------    ---   ------------------------------------------------
  27-NOV-12    NSA   Implemented single arm simple motion
   6-FEB-12		 NSA	 Cleaned up a bit, generalized some functionality
										 for use with 2 arms
\********************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include "midas.h"
#include "msystem.h"
#include "mcstd.h"
#include <math.h>
#include <unistd.h>
#include <string.h>
#include "TPathCalculator.hxx"
#include "TRotationCalculator.hxx"

/* make frontend functions callable from the C framework            */
#ifdef __cplusplus
extern "C" {
#endif

/*-- Globals -------------------------------------------------------*/
// The two function-like macros below are used in void channel_rw(INFO *pInfo, HNDLE* hKey, void *values, DWORD type, BOOL rw);
//Note:: See the funciton channel_rw for more information about these macros

//The below preprocessor macro would only work if the "funtion like" macro was defined on a single line. An identical version of the marco is put in the comment below below but is properly spaced to make it possible to read the code
#define READ_VALUES(TYPE){*((TYPE*)(values + 0*size)) = *((TYPE*)(motor00_values+5*size));*((TYPE*)(values + 1*size)) = *((TYPE*)(motor00_values+6*size));*((TYPE*)(values + 2*size)) = *((TYPE*)(motor00_values+7*size));*((TYPE*)(values + 3*size)) = *((TYPE*)(motor00_values+4*size));*((TYPE*)(values + 4*size)) = *((TYPE*)(motor00_values+3*size));*((TYPE*)(values + 5*size)) = *((TYPE*)(motor01_values+3*size));*((TYPE*)(values + 6*size)) = *((TYPE*)(motor01_values+1*size));*((TYPE*)(values + 7*size)) = *((TYPE*)(motor01_values+2*size));*((TYPE*)(values + 8*size)) = *((TYPE*)(motor01_values+4*size));*((TYPE*)(values + 9*size)) = *((TYPE*)(motor01_values+5*size));}

/*#define READ_VALUES(TYPE){
  *((TYPE*)(values + 0*size)) = *((TYPE*)(motor00_values+5*size));
  *((TYPE*)(values + 1*size)) = *((TYPE*)(motor00_values+6*size));
  *((TYPE*)(values + 2*size)) = *((TYPE*)(motor00_values+7*size));
  *((TYPE*)(values + 3*size)) = *((TYPE*)(motor00_values+4*size));
  *((TYPE*)(values + 4*size)) = *((TYPE*)(motor00_values+3*size));
  *((TYPE*)(values + 5*size)) = *((TYPE*)(motor01_values+3*size));
  *((TYPE*)(values + 6*size)) = *((TYPE*)(motor01_values+1*size));
  *((TYPE*)(values + 7*size)) = *((TYPE*)(motor01_values+2*size));
  *((TYPE*)(values + 8*size)) = *((TYPE*)(motor01_values+4*size));
  *((TYPE*)(values + 9*size)) = *((TYPE*)(motor01_values+5*size));}*/
      
//The below preprocessor macro would only work if the funtion like macro was defined on a signle line. An identical version of the marco is put in the comment below but is properly spaced to make it possible to read the code
#define STORE_DATA(TYPE) {TYPE motor00_TYPE_values[8];TYPE motor01_TYPE_values[8];buff_size=sizeof(motor00_TYPE_values);size = sizeof(TYPE);motor00_TYPE_values[0]= 0;motor00_TYPE_values[1]= 0;motor00_TYPE_values[2]= 0;motor00_TYPE_values[3]=*((TYPE*)(values+4*size));motor00_TYPE_values[4]=*((TYPE*)(values+3*size));motor00_TYPE_values[5]=*((TYPE*)(values+0*size));motor00_TYPE_values[6]=*((TYPE*)(values+1*size));motor00_TYPE_values[7]=*((TYPE*)(values+2*size));motor01_TYPE_values[0]= 0;motor01_TYPE_values[1]=*((TYPE*)(values+6*size));motor01_TYPE_values[2]=*((TYPE*)(values+7*size));motor01_TYPE_values[3]=*((TYPE*)(values+5*size));motor01_TYPE_values[4]=*((TYPE*)(values+8*size));motor01_TYPE_values[5]=*((TYPE*)(values+9*size));motor01_TYPE_values[6]= 0;motor01_TYPE_values[7]= 0;motor00_values = motor00_TYPE_values;motor01_values = motor01_TYPE_values;}

/*
#define STORE_DATA(TYPE) {
  TYPE motor00_TYPE_values[8];
  TYPE motor01_TYPE_values[8];

  buff_size=sizeof(motor00_float_values);
  size = sizeof(TYPE);
  
  motor00_TYPE_values[0]= 0;
  motor00_TYPE_values[1]= 0;
  motor00_TYPE_values[2]= 0;
  motor00_TYPE_values[3]=*((TYPE*)(values+4*size));   // Tilt
  motor00_TYPE_values[4]=*((TYPE*)(values+3*size));   // Rotary
  motor00_TYPE_values[5]=*((TYPE*)(values+0*size));   // X
  motor00_TYPE_values[6]=*((TYPE*)(values+1*size));   // Y
  motor00_TYPE_values[7]=*((TYPE*)(values+2*size));   // Z
        
  motor01_TYPE_values[0]= 0;  
  motor01_TYPE_values[1]=*((TYPE*)(values+6*size));   // Y
  motor01_TYPE_values[2]=*((TYPE*)(values+7*size));   // Z
  motor01_TYPE_values[3]=*((TYPE*)(values+5*size));   // X
  motor01_TYPE_values[4]=*((TYPE*)(values+8*size));   // Rotary
  motor01_TYPE_values[5]=*((TYPE*)(values+9*size));   // Tilt
  motor01_TYPE_values[6]= 0;
  motor01_TYPE_values[7]= 0;  
  motor00_values = motor00_TYPE_values;
  motor01_values = motor01_TYPE_values;
}*/

  /* The generate_path return flags */
  #define GENPATH_SUCCESS  0
  #define GENPATH_BAD_DEST 1

  /* The Abort Codes used by stop_move 	*/
  #define AC_USER_INPUT 0
  #define AC_COLLISION  1
  #define AC_TIMEOUT    2

  /* The frontend name (client name) as seen by other MIDAS clients   */
  char *frontend_name = "feMove";

  /* The frontend file name, don't change it                          */
  char *frontend_file_name = __FILE__;

  /* frontend_loop is called periodically if this variable is TRUE    */
  BOOL frontend_call_loop = FALSE;

  /* a frontend status page is displayed with this frequency in ms    */
  INT display_period = 0;

  /* maximum event size produced by this frontend                     */
  INT max_event_size = 3000;

  /* buffer size to hold events                                       */
  INT event_buffer_size = 10*3000;

  /* maximum event size for fragmented events (EQ_FRAGMENTED)         */
  INT max_event_size_frag = 5*300*300;

  /* counter outside initialize tilt, to prevent infinite loop        */
  INT tilt_ini_attempts = 0;

  /* Optionally only control one of two motors */
  INT gantry_motor_start = 0;   //for for-loops (by default from motor 0 to motor 9)
  INT gantry_motor_end   = 10;//10;   //TF TODO: get from ODB and make button on gantry_move page. Only use when one is in repair though!!!

  /*-- Info structure declaration ------------------------------------*/

  typedef struct {
    // Handles for all ODB variables
    HNDLE hDB;
    HNDLE hKeyDest,hKeyStart,hKeyStop,hKeyReInit;            // "Control"
    HNDLE hKeyVeloc,hKeyAccel,hKeyScale,hKeyChan,hKeyLimPos; // "Settings"
    HNDLE hKeyPos,hKeyInit,hKeySwap,hKeyBadDest, hKeyCompleted,
      hKeyMoving, hKeyAxMoving,hKeyAxLimitNeg,hKeyAxLimitPos,hKeyInitializing; // "Variables"
    HNDLE hKeyMDest[2],hKeyMStart[2],hKeyMStop[2],hKeyMMoving[2],
      hKeyMPos[2],hKeyMLimitNeg[2],hKeyMLimitPos[2],hKeyMVel[2],hKeyMAcc[2]; // Motors

    HNDLE hKeyPhidget[2];

    // "Control" ODB variables
    float *Destination;   // Gantry 1 and 2 axis destinations (physical units)
    BOOL Start;           // Start the motors towards the destination
    BOOL Stop;            // Stop all motors
    BOOL ReInitialize;    // Re-initialize the motors and move to current destination

    // "Settings" ODB variables
    float *Velocity;      // Motor velocities (physical units)
    float *Acceleration;  // Motor accelerations (physical units)
    float *mScale;        // Ratio between counts and physical units
    INT *Channels;        // Motor channel corresponding to each axis
    float *LimPos;	  // Coordinates of the limit switch on each axis (physical units)
  
    // "Variables" ODB variables
    float *Position;      // Axis positions (physical units)
    BOOL Initialized;     // Is system initialized (y/n)
    BOOL Completed;       // Have all requested moves completed without error (y/n)
    BOOL Moving;          // Are any of the axes moving (y/n)
    BOOL *AxisMoving;     // Is each axis moving (y/n)
    BOOL *neg_AxisLimit;  // Is each axis' negative limit switch being triggered (y/n)
    BOOL *pos_AxisLimit;  // Is each axis' positive limit switch being triggered (y/n) 
    BOOL BadDest;         // Did you set a Bad Destination?

    // Local variables
    float *CountPos;      // Axis positions (counts)
    float *CountDest;	  // Relative axis destinations (counts)
    float *mOrigin;       // Count locations of axis origins (counts)
    float **MovePath;     // 2d array of axis positions for each waypoint of the path (counts)
    int PathSize;         // Number of waypoints in the path
    int PathIndex;        // Index of the path section currently in progress
    int AbortCode;        // Variable in which to store the reason for aborting a move (AC_*)

    // Phidget Tilt variables
    double Phidget[10];      // 
  
    // Is the motor moving or not 
    BOOL Motor01X;
    BOOL Motor01Y;
    BOOL Motor01Z;
    BOOL Motor01Rot;
    BOOL Motor01Tilt;
    BOOL Motor00X;
    BOOL Motor00Y;
    BOOL Motor00Z;
    BOOL Motor00Rot;
    BOOL Motor00Tilt;
  
  } INFO;


  /*-- Function declarations -----------------------------------------*/

  // Required functions
  INT frontend_init();
  INT frontend_exit();
  INT begin_of_run(INT run_number, char *error);
  INT end_of_run(INT run_number, char *error);
  INT pause_run(INT run_number, char *error);
  INT resume_run(INT run_number, char *error);
  INT frontend_loop();
  INT poll_trigger_event(INT count, PTYPE test);
  INT interrupt_configure(INT cmd, PTYPE adr);
  INT read_trigger_event(char *pevent, INT off);
  INT read_scaler_event(char *pevent, INT off);

  // Added functions
  void move_init(HNDLE hDB,HNDLE hKey,void *data);
  int  generate_path(INFO *pInfo);
  void monitor(HNDLE hDB,HNDLE hKey,void *data);
  void move(INFO *pInfo);
  void stop_move(HNDLE hDB, HNDLE hKey, void *data);
  void initialize(INFO *pInfo);
  void reinitialize(HNDLE hDB,HNDLE hKey,void *data);
  void channel_rw(INFO *pInfo, HNDLE* hKey, void *values, DWORD type, BOOL rw);

  /*-- Equipment list ------------------------------------------------*/

  //#define USE_INT 1 

  EQUIPMENT equipment[] = {

    {"Move",       	    // equipment name 
     {5, 0,              // event ID, trigger mask 
      "SYSTEM",           // event buffer 
      EQ_PERIODIC,        // equipment type 
      0,                  // event source 
      "MIDAS",            // format 
      TRUE,               // enabled 
      RO_ALWAYS,		      // read x 
      10000,              // read every x millisec 
      0,                  // stop run after this event limit 
      0,                  // number of sub event 
      60,                	// log history every x sec 
      "", "", "",},
     read_trigger_event, // readout routine 
     NULL,               // class driver main routine 
     NULL,        	      // device driver list 
     NULL,               // init string 
    },

    { "" }
  };
  
  /********************************************************************\
            UNUSED callback routines for system transitions

  These routines are called whenever a system transition like start/
  stop of a run occurs. The routines are called on the following
  occations:
  
  *Note: Since we do not use run control with our system, none of 
         these routines are being used in our code.

  %begin_of_run:   When a new run is started. Clear scalers, open
                   rungates, etc.

  %end_of_run:     Called on a request to stop a run. Can send 
                   end-of-run event and close run gates.

  %pause_run:      When a run is paused. Should disable trigger events.

  %resume_run:     When a run is resumed. Should enable trigger events.

\********************************************************************/

  /*-- Begin of Run --------------------------------------------------*/

  INT begin_of_run(INT run_number, char *error)
  {
    return CM_SUCCESS;
  }

  /*-- End of Run ----------------------------------------------------*/

  INT end_of_run(INT run_number, char *error)
  {
    return CM_SUCCESS;
  }

  /*-- Pause Run -----------------------------------------------------*/

  INT pause_run(INT run_number, char *error)
  {
    return CM_SUCCESS;
  }

  /*-- Resume Run ----------------------------------------------------*/

  INT resume_run(INT run_number, char *error)
  {
    return CM_SUCCESS;
  }

  /*-- Frontend Loop -------------------------------------------------*/

  INT frontend_loop()
  {
    return SUCCESS;
  }

  /*-- Trigger event routines ----------------------------------------*/

  INT poll_event(INT source, INT count, BOOL test)
  /* Polling routine for events. Returns TRUE if event
     is available. If test equals TRUE, don't return. The test
     flag is used to time the polling */
  {
    return FALSE;
  }

  /*-- Interrupt configuration for trigger event ---------------------*/

  INT interrupt_configure(INT cmd, PTYPE adr)
  {
    return CM_SUCCESS;
  }

  /*-- Event readout -------------------------------------------------*/
  INT read_trigger_event(char *pevent, INT off)
  {
    return 0;
  }

  /*-- Scaler event --------------------------------------------------*/

  INT read_scaler_event(char *pevent, INT off)
  {
    return 0;
  }


#ifdef __cplusplus
}
#endif

/********************************************************************\
             USED callback routines for system transitions

  frontend_init:  When the frontend program is started. This routine
                  sets up all the ODB variables and hotlink for the 
                  system. The memory for pInfo is allocated here. 
  
  frontend_exit:  When the frontend program is shut down. Can be used
                  to release any locked resources like memory, commu-
                  nications ports etc.
\********************************************************************/

/*-- Frontend Init -------------------------------------------------*/
// Initialize ODB variables and set up hotlinks in this function
INT frontend_init()
{
  HNDLE hDB;
  INFO *pInfo;
  int i;
  float tempV[10];
  float tempA[10];

  /* Allocate info struct */
  pInfo = (INFO *) calloc(1, sizeof(INFO));

  pInfo->Destination    = (float  *)  calloc(10,sizeof(float));
  pInfo->Velocity       = (float  *)  calloc(10,sizeof(float)); 
  pInfo->Acceleration   = (float  *)  calloc(10,sizeof(float)); 
  pInfo->mScale         = (float  *)  calloc(10,sizeof(float)); 
  pInfo->Channels       = (INT    *)  calloc(10,sizeof(float)); 
  pInfo->LimPos		      = (float  *)  calloc(10,sizeof(float));
  pInfo->Position       = (float  *)  calloc(10,sizeof(float));
  pInfo->AxisMoving     = (BOOL   *)  calloc(10,sizeof(BOOL));
  pInfo->neg_AxisLimit  = (BOOL   *)  calloc(10,sizeof(BOOL));
  pInfo->pos_AxisLimit  = (BOOL   *)  calloc(10,sizeof(BOOL));
  pInfo->CountPos       = (float  *)  calloc(10,sizeof(float));
  pInfo->CountDest      = (float  *)  calloc(10,sizeof(float));
  pInfo->mOrigin        = (float  *)  calloc(10,sizeof(float));
  pInfo->MovePath       = (float **)  calloc(10,sizeof(float*));
  pInfo->MovePath       = (float **)  calloc(10,sizeof(float*));

  /* Get database handle */ 
  cm_get_experiment_database(&hDB,NULL);
  pInfo->hDB = hDB;

  /* Initialize non-ODB variables */
  pInfo->PathSize = 0;
  pInfo->PathIndex = 0;
  pInfo->AbortCode = AC_USER_INPUT;

  /* Initialize "Control" ODB variables */
  
  // "Destination"
  // Don't reset destination when initializing. TL, i.e don't call db_set_data here
  db_find_key(hDB,0,"/Equipment/Move/Control/Destination",&pInfo->hKeyDest);

  // "Start Move"
  db_find_key(hDB,0,"/Equipment/Move/Control/Start Move",&pInfo->hKeyStart);
  pInfo->Start = 0;
  db_set_data(hDB,pInfo->hKeyStart,&pInfo->Start,sizeof(BOOL),1,TID_BOOL);
  
  // "Stop Move"
  db_find_key(hDB,0,"/Equipment/Move/Control/Stop Move",&pInfo->hKeyStop);
  pInfo->Stop = 0;
  db_set_data(hDB,pInfo->hKeyStop,&pInfo->Stop,sizeof(BOOL),1,TID_BOOL);
  
  // "ReInitialize"
  db_find_key(hDB,0,"/Equipment/Move/Control/ReInitialize",&pInfo->hKeyReInit);
  pInfo->ReInitialize = 0;
  db_set_data(hDB,pInfo->hKeyReInit,&pInfo->ReInitialize,sizeof(BOOL),1,TID_BOOL);

  /* Initialize "Settings" ODB variables */ 
  // Note: The following variables can only be changed when feMove is
  //			 not running. If feMove is already running, changing these variables
  //       in the ODB will have no effect.

  // "Velocity"
  db_merge_data(hDB,0,"/Equipment/Move/Settings/Velocity",pInfo->Velocity,10*sizeof(float),10,TID_FLOAT);
  db_find_key(hDB,0,"/Equipment/Move/Settings/Velocity",&pInfo->hKeyVeloc);
	 	
  // "Acceleration"
  db_merge_data(hDB,0,"/Equipment/Move/Settings/Acceleration",pInfo->Acceleration,10*sizeof(float),10,TID_FLOAT);
  db_find_key(hDB,0,"/Equipment/Move/Settings/Acceleration",&pInfo->hKeyAccel);

  // "Motor Scaling"
  db_merge_data(hDB,0,"/Equipment/Move/Settings/Motor Scaling",pInfo->mScale,10*sizeof(float),10,TID_FLOAT);
  db_find_key(hDB,0,"/Equipment/Move/Settings/Motor Scaling",&pInfo->hKeyScale);
  
  // "Axis Channels" 
  db_merge_data(hDB,0,"/Equipment/Move/Settings/Axis Channels",pInfo->Channels,10*sizeof(INT),10,TID_INT);
  db_find_key(hDB,0,"/Equipment/Move/Settings/Axis Channels",&pInfo->hKeyChan);
  
  // "Limit Positions"
  db_merge_data(hDB,0,"/Equipment/Move/Settings/Limit Positions",pInfo->LimPos,10*sizeof(float),10,TID_FLOAT);
  db_find_key(hDB,0,"/Equipment/Move/Settings/Limit Positions",&pInfo->hKeyLimPos);
  
  /* Initialize "Variables" ODB variables */
  // "Position(X,Y,Z,Theta,Phi)"
  db_find_key(hDB,0,"/Equipment/Move/Variables/Position",&pInfo->hKeyPos);
  db_set_data(hDB,pInfo->hKeyPos,pInfo->Position,10*sizeof(float),10,TID_FLOAT);	

  // "Initialized"
  db_find_key(hDB,0,"/Equipment/Move/Variables/Initialized",&pInfo->hKeyInit);
  pInfo->Initialized = 0;
  db_set_data(hDB,pInfo->hKeyInit,&pInfo->Initialized,sizeof(BOOL),1,TID_BOOL);  
  
  // "Bad Destination"
  db_find_key(hDB,0,"/Equipment/Move/Variables/Bad Destination",&pInfo->hKeyBadDest);
  pInfo->BadDest = 0;
  db_set_data(hDB,pInfo->hKeyBadDest,&pInfo->BadDest,sizeof(BOOL),1,TID_BOOL);

  // "Completed"
  db_find_key(hDB,0,"/Equipment/Move/Variables/Completed",&pInfo->hKeyCompleted);
  pInfo->Completed = 1;
  db_set_data(hDB,pInfo->hKeyCompleted,&pInfo->Completed,sizeof(BOOL),1,TID_BOOL);  

  // "Moving"
  db_find_key(hDB,0,"/Equipment/Move/Variables/Moving",&pInfo->hKeyMoving);
  
  // "Initializing"
  db_find_key(hDB,0,"/Equipment/Move/Variables/Initializing",&pInfo->hKeyInitializing);

  // "Axis Moving"
  db_find_key(hDB,0,"/Equipment/Move/Variables/Axis Moving",&pInfo->hKeyAxMoving);
  db_set_data(hDB,pInfo->hKeyAxMoving,pInfo->AxisMoving,10*sizeof(BOOL),10,TID_BOOL);

  // "Negative Axis Limit"  
  db_find_key(hDB,0,"/Equipment/Move/Variables/Negative Axis Limit",&pInfo->hKeyAxLimitNeg);
  db_set_data(hDB,pInfo->hKeyAxLimitNeg,pInfo->neg_AxisLimit,10*sizeof(BOOL),10,TID_BOOL);

  // "Positive Axis Limit"  
  db_find_key(hDB,0,"/Equipment/Move/Variables/Positive Axis Limit",&pInfo->hKeyAxLimitPos);
  db_set_data(hDB,pInfo->hKeyAxLimitPos,pInfo->pos_AxisLimit,10*sizeof(BOOL),10,TID_BOOL);

  /* Get handles for the "Motor" ODB variables */
  // "Destination"
  db_find_key(hDB,0,"/Equipment/Motors00/Settings/Destination",&pInfo->hKeyMDest[0]);
  db_find_key(hDB,0,"/Equipment/Motors01/Settings/Destination",&pInfo->hKeyMDest[1]);

  // "Move"
  db_find_key(hDB,0,"/Equipment/Motors00/Settings/Move",&pInfo->hKeyMStart[0]);
  db_find_key(hDB,0,"/Equipment/Motors01/Settings/Move",&pInfo->hKeyMStart[1]);

  // "Stop"
  db_find_key(hDB,0,"/Equipment/Motors00/Settings/Stop",&pInfo->hKeyMStop[0]);
  db_find_key(hDB,0,"/Equipment/Motors01/Settings/Stop",&pInfo->hKeyMStop[1]);

  // "Moving"
  db_find_key(hDB,0,"/Equipment/Motors00/Variables/Moving",&pInfo->hKeyMMoving[0]);
  db_find_key(hDB,0,"/Equipment/Motors01/Variables/Moving",&pInfo->hKeyMMoving[1]);

  // "Position"
  db_find_key(hDB,0,"/Equipment/Motors00/Variables/Position",&pInfo->hKeyMPos[0]);
  db_find_key(hDB,0,"/Equipment/Motors01/Variables/Position",&pInfo->hKeyMPos[1]);

  // "Limit Pos"
  db_find_key(hDB,0,"/Equipment/Motors00/Variables/Limit Pos",&pInfo->hKeyMLimitPos[0]);
  db_find_key(hDB,0,"/Equipment/Motors01/Variables/Limit Pos",&pInfo->hKeyMLimitPos[1]);

  // "Limit Neg"
  db_find_key(hDB,0,"/Equipment/Motors00/Variables/Limit Neg",&pInfo->hKeyMLimitNeg[0]);
  db_find_key(hDB,0,"/Equipment/Motors01/Variables/Limit Neg",&pInfo->hKeyMLimitNeg[1]);
  
  // "Velocity"
  db_find_key(hDB,0,"/Equipment/Motors00/Settings/Velocity",&pInfo->hKeyMVel[0]);
  db_find_key(hDB,0,"/Equipment/Motors01/Settings/Velocity",&pInfo->hKeyMVel[1]);
  
  // "Acceleration"
  db_find_key(hDB,0,"/Equipment/Motors00/Settings/Acceleration",&pInfo->hKeyMAcc[0]);
  db_find_key(hDB,0,"/Equipment/Motors01/Settings/Acceleration",&pInfo->hKeyMAcc[1]);

  // "Accelerometer tilt"
  db_find_key(hDB,0,"/Equipment/Phidget00/Variables",&pInfo->hKeyPhidget[0]);
  db_find_key(hDB,0,"/Equipment/Phidget01/Variables",&pInfo->hKeyPhidget[1]);

  /* Set up hotlinks */
  // move_init() hotlink
  db_open_record(hDB,pInfo->hKeyStart,&pInfo->Start,sizeof(BOOL),MODE_READ,move_init,pInfo);
  
  // stop_move() hotlink
  db_open_record(hDB,pInfo->hKeyStop,&pInfo->Stop,sizeof(BOOL),MODE_READ,stop_move,pInfo);
  
  // reinitialize() hotlink
  db_open_record(hDB,pInfo->hKeyReInit,&pInfo->ReInitialize,sizeof(BOOL),MODE_READ,reinitialize,pInfo);

  // monitor() hotlink
  db_open_record(hDB,pInfo->hKeyMMoving[0],NULL,8*sizeof(BOOL),MODE_READ,monitor,pInfo);
  db_open_record(hDB,pInfo->hKeyMMoving[1],NULL,8*sizeof(BOOL),MODE_READ,monitor,pInfo);
  // Note: The variable hotlinked to monitor() is somewhat arbitrary, all that is 
  //	   really needed is a variable that gets periodically updated in feMove.

  /* Set motor velocities and accelerations to in feMotor */
  for( i= gantry_motor_start; i < gantry_motor_end ; i++){
    tempV[i] = pInfo->Velocity[i]*fabs(pInfo->mScale[i]);
    tempA[i] = pInfo->Acceleration[i]*fabs(pInfo->mScale[i]);
  }

  channel_rw(pInfo, pInfo->hKeyMVel, (void *)tempV, TID_FLOAT, 1);
  channel_rw(pInfo, pInfo->hKeyMAcc, (void *)tempA, TID_FLOAT, 1);
  return CM_SUCCESS;
}

/*-- Frontend Exit -------------------------------------------------*/
// Stop the motors and free all memory in this function
INT frontend_exit()
{
  return CM_SUCCESS;
}

/********************************************************************\
                Main routines during frontend operation

  These routines are called when hotlinked ODB variables are changed
  during frontend operation. They operate by changing variables in 
  "Equipment/Motor" to control the gantry operation.

  move_init:      Hotlinked to the variable "Start Move", this 
                  function generates the move path and initializes 
                  the move.

  initialize:	  This function moves the motors to the limit switches
		  (in a gauranteed collision free manner) and determines
		  the motor coordinates (in steps) at the origin. This
		  value is placed in the variable pInfo->mOrigin.

 reinitialize:	  This function calls initialize (initializes the gantry
                  and then starts 
		  move_init. (i.e. it sends the motors to their limit
		  switches, then to their destinations)

  generate_path:  This function takes the current location and the
                  desired destination and returns a series of way-
                  points that will get the arm to the destination
                  without collision.

  move:           This function starts the axes towards the subsequent
                  index of the generated path.

  stop_move:	  This function stops all motor movement

  monitor:        Hotlinked to the variables:
   		  "/Motors00/Variables/Moving","/Motors01/Variables/Moving"
                  this function checks for completion of a move
                  whenever the motors stop. Depending on the result
                  of this check, this function either initiates the
                  next path section, sets completed to 1, or sets 
                  completed to 0 (error).
                  
  channel_rw:     This function handles all communication with feMotor.
                  Only here do we read and write to specific motor 
                  channels.
\********************************************************************/
/*-- Move Init -----------------------------------------------------*/
// Call generate path and start the first move
void move_init(HNDLE hDB,HNDLE hKey,void *data){
  INFO *pInfo = (INFO *) data;
  
  // Check that the user has set "Start Move" to "y"
  int size = sizeof(BOOL);
  db_get_data(hDB,pInfo->hKeyStart,&pInfo->Start,&size,TID_BOOL);
  if(pInfo->Start == 0) return;
  
  // If motors are already moving, set destinations back to previous
  // destination values and return with an error.
  if(pInfo->Moving){
    cm_msg(MERROR, "move_init", "Error: Can't start move. Move already in progress.");    
    return;
  }
	
  // Check if the motors have been initialized. If not, initialize them
  // before proceeding.
  db_get_data(hDB,pInfo->hKeyInit,&pInfo->Initialized,&size,TID_BOOL);
  if(pInfo->Initialized == 0) initialize(pInfo);
  // If initialization fails, return with error
  if(pInfo->Initialized == 0){
    cm_msg(MERROR, "move_init", "Error: Can't start move. Initialization failed.");
    return;
  }

  // Load input destination into pInfo->Destination
  size = 10*sizeof(float);
  db_get_data(hDB,pInfo->hKeyDest,pInfo->Destination,&size,TID_FLOAT);
  
  // Generate collision free path to pInfo->Destination
  cm_msg(MINFO,"move_init","Generating Path");
  int Status = generate_path(pInfo);
 
  // Check for unsuccesful path generation
  switch(Status){
  case GENPATH_SUCCESS:
    pInfo->BadDest = 0;
    db_set_data(hDB,pInfo->hKeyBadDest,&pInfo->BadDest,sizeof(BOOL),1,TID_BOOL);
    break;
  case GENPATH_BAD_DEST:
    cm_msg(MERROR, "move_init", " Bad destination entered in Move");
    pInfo->BadDest = 1;
    db_set_data(hDB,pInfo->hKeyBadDest,&pInfo->BadDest,sizeof(BOOL),1,TID_BOOL);
    return;
  }
  cm_msg(MINFO,"move_init","Path succesfully generated");

  // Set the "completed" variable in the ODB to 0 (since our newly
  // started move is still incomplete.
  pInfo->Completed = 0;
  db_set_data(hDB,pInfo->hKeyCompleted,&pInfo->Completed,sizeof(BOOL),1,TID_BOOL);
   
  // Move to first path index
  move(pInfo);
  
  
  // Set the ODB variable "Start Move" back to 0
  pInfo->Start = 0;
  db_set_data(hDB,pInfo->hKeyStart,&pInfo->Start,sizeof(BOOL),1,TID_BOOL);
}

/*-- Initialize tilt ----------------------------------------------------*/
// We initialize the tilt motor by using the tilt measurement from the 1044 Phidget
// accelerometer.
int initialize_tilt(INFO *pInfo){

  int i;
  double tolerance = 1.; //3.0;
  float *dest = (float *) calloc(10,sizeof(float));
  BOOL *start = (BOOL *) calloc(10,sizeof(BOOL));
  int size = sizeof(pInfo->Phidget);

  //Only loop over the motors from the 
  //gantry we want to use
  int tilt_start = 0;
  int tilt_end = 2;
  if(gantry_motor_start == 5){
    tilt_start = 1;
  } else if(gantry_motor_end == 5){
    tilt_end = 1;
  }

  for(i = tilt_start; i < tilt_end; i++){
  
    int axis = (i == 0? 4: 9);
    // TF TEMP
    //if (axis == 9)
    //  continue;

    //Get readings of phidget. 
    int status = db_get_value(pInfo->hDB,pInfo->hKeyPhidget[0],"PH00",&pInfo->Phidget,&size,TID_DOUBLE,FALSE);
    if(i==1) {status = db_get_value(pInfo->hDB,pInfo->hKeyPhidget[1],"PH01",&pInfo->Phidget,&size,TID_DOUBLE,FALSE);}
    if(status != DB_SUCCESS) {
      cm_msg(MERROR, "initialize_tilt", "Cannot get value for Phidget %i Reading",i);
      return DB_NO_ACCESS;
    }
      
    //Check that the phidget is connected by checking that its internal clock is changing   
    int time_s = pInfo->Phidget[8];
    int time_us = pInfo->Phidget[9];

    // Make sure phidget readings change from previous value:
    sleep(2000); //2sec
    if(i == 0) db_get_value(pInfo->hDB,pInfo->hKeyPhidget[0],"PH00",&pInfo->Phidget,&size,TID_DOUBLE,FALSE);
    else db_get_value(pInfo->hDB,pInfo->hKeyPhidget[1],"PH01",&pInfo->Phidget,&size,TID_DOUBLE,FALSE);


    //TF note: this is not 100% reliable...even with a big sleep : 
    // DEBUG!!!! TODO
    if(time_s == pInfo->Phidget[8] && time_us == pInfo->Phidget[9]){
      printf("Time not updated; fePhidget not running; can't initialize tilt\n");
      printf("%i %f %i %f\n",time_s,pInfo->Phidget[8],time_us,pInfo->Phidget[9]);
      //return 0; //TF: uncomment for Tilt tests...
      //TF: for running tilt without phidgets:
      //pInfo->Phidget[7] = -10;
    }
    
    if(i == 0)
      printf("\n");
    cm_msg(MINFO,"move_init","Tilt %i initial position (from Phidget): tilt angle = %f",i,pInfo->Phidget[7]);
    

    // Now we estimate the distance we need in order to get the tilt to zero.
    dest[axis] = -(pInfo->Phidget[7]*pInfo->mScale[axis]);
    
    // Only activate the tilt!
    start[axis] = 1;
   
  }
  //DEBUG:
  printf("Destinations: Dest[4] = %f, dest[9] = %f\n",dest[4],dest[9]);

  // Write the destinations to the motors
  // Note: channel_rw works for both motors, ie. write both tilt destinations at once and initialize
  // together
  channel_rw(pInfo,pInfo->hKeyMDest,dest,TID_FLOAT,1);  
  
  // Start the move
  channel_rw(pInfo,pInfo->hKeyMStart,(void *)start,TID_BOOL,1);
  sleep(600); // TF and BK: is this to wait for galil_read to update ODB for AxisMoving??
    
  // Monitor the motion until we are finished moving.
  pInfo->Moving = 1;
  bool startedMoving = false;
  while(pInfo->Moving){      
    pInfo->Moving = 0;
    
    //TF note: this only checks whether the motors are moving!!
    channel_rw(pInfo,pInfo->hKeyMMoving,(void *)pInfo->AxisMoving,TID_BOOL,0);
    pInfo->Moving = pInfo->Moving || pInfo->AxisMoving[4];
    pInfo->Moving = pInfo->Moving || pInfo->AxisMoving[9];
    if(pInfo->Moving && !startedMoving)
      startedMoving = true;
    
  }
  cm_msg(MINFO,"initialize_tilt","Move complete.");
  tilt_ini_attempts++;
  for(i = tilt_start; i < tilt_end; i++){
    int axis = (i == 0? 4: 9);

    // Check that the final tilt is within the required tolerance
    if(i==0) db_get_value(pInfo->hDB,pInfo->hKeyPhidget[0],"PH00",&pInfo->Phidget,&size,TID_DOUBLE,FALSE);
    else db_get_value(pInfo->hDB,pInfo->hKeyPhidget[1],"PH01",&pInfo->Phidget,&size,TID_DOUBLE,FALSE);

    //NaN check:
    if(pInfo->Phidget[7] != pInfo->Phidget[7]){
      cm_msg(MERROR,"initialize_tilt","NAN tilt value, either not connected or bug in tilt angle calculation!");
      continue;
    }
    if(fabs(pInfo->Phidget[7]) < tolerance){
      cm_msg(MINFO,"initialize_tilt","Tilt %i = %.2f deg, Acceleration(a_x = %.4f, a_y = %.4f, a_z= %.4f)",i,pInfo->Phidget[7],pInfo->Phidget[0],pInfo->Phidget[1],pInfo->Phidget[2]);
    }else{
      cm_msg(MERROR, "initialize_tilt","Tilt initialization (axis %i) failed; final tilt = %f is not within zero to within required tolerance of %f degrees", axis,pInfo->Phidget[7],tolerance);
      // --> for debugging: switch the initialize_tilt ON and uncomment the return
      // printf("Attempting initialization again");
      if(tilt_ini_attempts < 4)
	initialize_tilt(pInfo); 
      
      if(!startedMoving)
	cm_msg(MERROR,"initialize_tilt","Tilt motors didn't even start moving");
      else
      	cm_msg(MERROR,"initialize_tilt","Tilt motors didn't move the optical box (sufficiently enough). Check the grip of the gears or the motor scaling!");
      //return -1;
     
    }

    // Reset origin position.
    channel_rw(pInfo,pInfo->hKeyMPos,pInfo->CountPos,TID_FLOAT,0);
    pInfo->mOrigin[axis] = pInfo->CountPos[axis] - pInfo->LimPos[axis]*pInfo->mScale[axis];
    pInfo->Position[axis] = pInfo->LimPos[axis];
   
    cm_msg(MINFO,"initialize_tilt","Finished initializing axis %i; Origin = %5.2f counts, Position = %5.2f deg",axis,pInfo->mOrigin[axis], pInfo->Position[axis]);
        
  }//end of loop over both gantries

  return 0;

}

/*-- Initialize ----------------------------------------------------*/
// Move motors to limit switches. Use the motor coordinates at this
// position to determine the motor coordinates at the origin. If the
// negative limit switch is disabled, the current position is set as the
// origin for that axis.
void initialize(INFO *pInfo){
  int i;
  BOOL *tempStart = (BOOL *)  calloc(10,sizeof(BOOL));
  BOOL *tempNegLimitEnabled = (BOOL *) calloc(10,sizeof(BOOL));
  float *tempPos  = (float *)  calloc(10,sizeof(float));
  BOOL tempStop[10] = {1,1,1,1,1,1,1,1,1,1};
  BOOL exitFlag = 0;
  float lastCountPosArm1;
  float lastCountPosArm2;
  

  cm_msg(MINFO,"initialize","Initializing motors");
  BOOL initing = 1;
  db_set_data(pInfo->hDB,pInfo->hKeyInitializing,&initing,sizeof(BOOL),1,TID_BOOL);

  // Set motors to move into their negative limit switches, if they are enabled 
  //(disabled is LimPos = 9999)
  // Motors 4 and 9 are the tilt motors.  These are initialized separately, using 
  // the tilt measurement from the phidget.
  //for(i=0;i<10;i++){
  for(i = gantry_motor_start; i < gantry_motor_end ; i++){
    if(i == 4 || i == 9){
      //Do nothing, tilt should already be initialized
      tempPos[i] = 0;
      tempNegLimitEnabled[i] = 0;
      cm_msg(MINFO,"initialize","Axis %i for tilt motor will be initialized separately.", i);
    }else if(pInfo->LimPos[i] == 9999){
      tempPos[i] = 0;
      tempNegLimitEnabled[i] = 0;
      cm_msg(MINFO,"initialize","Negative limit switch for axis %i disabled. Axis will not be initialized.", i);
    }else{
      tempPos[i] = 500*fabs(pInfo->mScale[i]);
      tempNegLimitEnabled[i] = 1;
      cm_msg(MINFO,"initialize","Negative limit switch for axis %i enabled. Axis will be initialized; use position %f.", i,tempPos[i]);
    }
  }

  channel_rw(pInfo,pInfo->hKeyMDest,(void *)tempPos,TID_FLOAT,1);
  
  // Cycle through each pair of motors corresponding to the same axis on each arm.
  // We want to make sure that we initialize the Z-axis first, so that the laser box is
  // fully out of the tank before we move in X and Y.
  int order[4] = {2,0,1,3};
  int itmp;
  for(itmp=0;itmp<4;itmp++){
    i = order[itmp];

    if((tempNegLimitEnabled[i] == 1) || (tempNegLimitEnabled[i+5] == 1)){					 	
      cm_msg(MINFO,"initialize","Initializing axes %i and %i (enabled = %i %i)",i,i+5,tempNegLimitEnabled[i],tempNegLimitEnabled[i+5]);
      //printf("Going to destination %f %f\n",tempPos[i],tempPos[i+5]);
    }
    tempStart[i] = tempNegLimitEnabled[i];
    tempStart[i+5] = tempNegLimitEnabled[i+5];

    channel_rw(pInfo,pInfo->hKeyMStart,(void *)tempStart,TID_BOOL,1);
    
    // Wait for axes with enabled limit switches to hit their limits
    while(1){
      channel_rw(pInfo,pInfo->hKeyMPos,pInfo->CountPos,TID_FLOAT,0);
      lastCountPosArm1 = pInfo->CountPos[i];
      lastCountPosArm2 = pInfo->CountPos[i+5];
      sleep(600); // Approx. polling period
      if((tempNegLimitEnabled[i] == 1) && (tempNegLimitEnabled[i+5] == 1)){// If both gantry axes are enabled.
      	channel_rw(pInfo,pInfo->hKeyMLimitNeg,(void *)pInfo->neg_AxisLimit,TID_BOOL,0);
        if(pInfo->neg_AxisLimit[i] && pInfo->neg_AxisLimit[i+5]) break;
	else{
	  channel_rw(pInfo,pInfo->hKeyMPos,pInfo->CountPos,TID_FLOAT,0);
	  if(pInfo->CountPos[i] == lastCountPosArm1 && !pInfo->neg_AxisLimit[i]){
	    cm_msg(MERROR,"initialize","Axis %i not moving!!! Stopping initialization since limit switch must be broken...", i);
	    channel_rw(pInfo,pInfo->hKeyMStop,(void *)tempStop,TID_BOOL,1);
	    exitFlag = 1;
	    break;
	  }
	  if(pInfo->CountPos[i+5] == lastCountPosArm2 && !pInfo->neg_AxisLimit[i+5]){
	    cm_msg(MERROR,"initialize","Axis %i not moving!!! Stopping initialization since limit switch must be broken...", i+5);
      channel_rw(pInfo,pInfo->hKeyMStop,(void *)tempStop,TID_BOOL,1);
	    exitFlag = 1;
	    break;
	  }
	}
      }
      else if((tempNegLimitEnabled[i] == 0) && (tempNegLimitEnabled[i+5] == 0)){
        break;
      }
      else if(tempNegLimitEnabled[i] == 0){  // If only second gantry axis is enabled.
	
	channel_rw(pInfo,pInfo->hKeyMLimitNeg,(void *)pInfo->neg_AxisLimit,TID_BOOL,0);
        if(pInfo->neg_AxisLimit[i+5]){
	  break;
	}else{
	  channel_rw(pInfo,pInfo->hKeyMPos,pInfo->CountPos,TID_FLOAT,0);
	  if(pInfo->CountPos[i+5] == lastCountPosArm2){
	    cm_msg(MERROR,"initialize","Axis %i not moving!!! Stopping initialization since limit switch must be broken...", i+5);
	    channel_rw(pInfo,pInfo->hKeyMStop,(void *)tempStop,TID_BOOL,1);
	    exitFlag = 1;
	    break;
	  }
	}
      }
      else{// If only first gantry axis is enabled.
	channel_rw(pInfo,pInfo->hKeyMLimitNeg,(void *)pInfo->neg_AxisLimit,TID_BOOL,0);
        if(pInfo->neg_AxisLimit[i]){
	  break;
	}else{
	  channel_rw(pInfo,pInfo->hKeyMPos,pInfo->CountPos,TID_FLOAT,0);
	  if(pInfo->CountPos[i] == lastCountPosArm1){
	    cm_msg(MERROR,"initialize","Axis %i not moving!!! Stopping initialization since limit switch must be broken...", i);
	    channel_rw(pInfo,pInfo->hKeyMStop,(void *)tempStop,TID_BOOL,1);
	    exitFlag = 1;
	    break;
	  }
	}
      }
    }
    tempStart[i] = 0;
    tempStart[i+5] = 0;
    if(exitFlag == 1) break;
  }
  
  // Wait for all the motors to stop moving (sometimes this 
  // occurs slightly after the limit switches are triggered)
  pInfo->Moving = 1;
  while(pInfo->Moving){
    pInfo->Moving = 0;
    channel_rw(pInfo,pInfo->hKeyMMoving,(void *)pInfo->AxisMoving,TID_BOOL,0);
      for(i = gantry_motor_start; i < gantry_motor_end ; i++) pInfo->Moving = pInfo->Moving || pInfo->AxisMoving[i];
  }
  
  if(exitFlag == 0) {
    // Determine the motor positions at the origin and put the results in
    // pInfo->mOrigin
    channel_rw(pInfo,pInfo->hKeyMPos,pInfo->CountPos,TID_FLOAT,0);
    for(i = 0; i < 10 ; i++){  
    //for(i = gantry_motor_start; i < gantry_motor_end ; i++){  
      // only exception in flexible for loop: even if other motor is off, still initialize to LimPos...TF: safe??
      if(tempNegLimitEnabled[i] == 1 || (i >= gantry_motor_end || i < gantry_motor_start) ){ 
        // The motor origin should always be at the limit switches, since this is 
        // a local variable
        // MARK 
	pInfo->mOrigin[i] = pInfo->CountPos[i];// - pInfo->LimPos[i]*pInfo->mScale[i];
	pInfo->Position[i] = pInfo->LimPos[i];
	cm_msg(MINFO,"initialize","Finished initializing axis %i; Origin = %5.2f counts, Position = %5.2f m.",i,pInfo->mOrigin[i], pInfo->Position[i]);
      }
      else{
	pInfo->mOrigin[i] = 0;
	pInfo->Position[i] = 0;
      }
    }
    db_set_data(pInfo->hDB,pInfo->hKeyPos,&pInfo->Position,10*sizeof(float),10,TID_FLOAT);
  }

  int exitFlagTilt = initialize_tilt(pInfo);


  if(exitFlag == 0 && exitFlagTilt ==0){
    // Set Initialized to 1 and exit    
    pInfo->Initialized = 1;
    db_set_data(pInfo->hDB,pInfo->hKeyInit,&pInfo->Initialized,sizeof(BOOL),1,TID_BOOL);
  }
    
  initing = 0;
  db_set_data(pInfo->hDB,pInfo->hKeyInitializing,&initing,sizeof(BOOL),1,TID_BOOL);

  sleep(100);
  cm_msg(MINFO, "move_init", "Initialization of Gantries is Complete");    
  return;


}

/*-- Re-Initialize -------------------------------------------------*/
// Set initialized to 0, then start the move
void reinitialize(HNDLE hDB,HNDLE hKey,void *data){

  INFO *pInfo = (INFO *) data;
  INT size = sizeof(BOOL);
  db_get_data(hDB,pInfo->hKeyReInit,&pInfo->ReInitialize,&size,TID_BOOL);
  if(!pInfo->ReInitialize) return;
  
  pInfo->Initialized = 0;
  db_set_data(hDB,pInfo->hKeyInit,&pInfo->Initialized,sizeof(BOOL),1,TID_BOOL);

  
  pInfo->Start = 1;
  db_set_data(hDB,pInfo->hKeyStart,&pInfo->Start,sizeof(BOOL),1,TID_BOOL);
  

  pInfo->ReInitialize = 0;
  db_set_data(hDB,pInfo->hKeyReInit,&pInfo->ReInitialize,sizeof(BOOL),1,TID_BOOL);

  pInfo->BadDest = 0;
  db_set_data(hDB,pInfo->hKeyBadDest,&pInfo->BadDest,sizeof(BOOL),1,TID_BOOL);
}

/*-- Generate Path -------------------------------------------------*/
// Generate collision-free path, put the path data into pInfo->MovePath
// Aaron: insert path generator here:
//				- Destination (in physical units) found in pInfo->Destination
//				- Determine required stepper motor coordinates using motor
//					scaling found in pInfo->mScale
//			  - Find collision-free waypoints to get to this point
//			  - Put generated path in pInfo->MovePath (10 X N_Waypoints)
//						**Allocate first**
//				- Set pInfo->PathSize to the number of waypoints
//				- set pInfo->PathIndex to 0
//				- Return Value:
//							GENPATH_SUCCESS: path successfully generated, no swap needed
//							GENPATH_BAD_DEST: impossible destination given as input
// Shaun: Constrained boundaries of box by adding rotation dependance
// 		-gantry 1 & 2 have 1-2 cm of safety margin
// 		-the four possible permutations that system can undertake are:  1) gantry 1 move first, rotation first; 
// 										2) gantry 1 move first, rotation second; 
// 										3) gantry 2 move first, rotation first; 
// 										4) gantry 2 move first, rotation second
// 		-the system will move in Z-direction first if the endpoint z is outside the water tank
//		-paths are calculated for the 4 possible permutations to see if any are possible w/o collision
//				-1) w/ gantry 2 in start position and final rotation, gantry 1 in rotated position can move from start to destination
//				    w/ gantry 1 at destination and final rotation, gantry 2 in rotated position can move from start to destination
//				-2) w/ gantry 2 in start position and initial rotation, gantry 1 in initial rotated position can move from start to destination
//				    w/ gantry 1 at destination and initial rotation, gantry 2 in initial rotated position can move from start to destination
//				    gantry 1 in final rotated position at destination will does not hit gantry 2 in final rotated position at destination
//				-3) w/ gantry 1 in start position and final rotation, gantry 2 in rotated position can move from start to destination
//				    w/ gantry 2 at destination and final rotation, gantry 1 in rotated position can move from start to destination
//				-4) w/ gantry 1 in start position and initial rotation, gantry 2 in initial rotated position can move from start to destination
//				    w/ gantry 2 at destination and initial rotation, gantry 1 in initial rotated position can move from start to destination
//				    gantry 2 in final rotated position at destination will does not hit gantry 1 in final rotated position at destination
//		-to ensure intermediate roation positions will not collide with other gantry or water tank (in case where initial and final rotation is good but intermediate value is not) TRotationCalculator is constructed
int generate_path(INFO *pInfo){

  // Check for illegal destinations.  Currently just check:
  // rot_min < rotary angle < rot_max
  double rot_min = -120, rot_max = 120;
  // tilt_min < tilt angle < tilt_max
  double tilt_min =  -105, tilt_max = 15;

  double tankHeight = 0.05; //actually 0.08 but play safe.

  // PMT height : currently hard Z limit
  // TODO: this limit depends on XY (center) and tilt (can go lower with flat box)
  double pmtHeight = 0.9;//0.482;  //use when tank in place

  bool move_second_gantry_first = false;
  bool move_z1_first = false;  
  bool move_z2_first = false;  
  bool move_rotation_first = false;

  // Create path calculator object
  TPathCalculator pathCalc00;
  TPathCalculator pathCalc01;
  TPathCalculator pathCalc1a;
  TPathCalculator pathCalc1b;
  TPathCalculator pathCalc2a;
  TPathCalculator pathCalc2b;
  TPathCalculator pathCalc2c;
  TPathCalculator pathCalc3a;
  TPathCalculator pathCalc3b;
  TPathCalculator pathCalc4a;
  TPathCalculator pathCalc4b;
  TPathCalculator pathCalc4c;
  TRotationCalculator pathCalc_rotstart;
  TRotationCalculator pathCalc_rotend;
  
  double gantry1XPos = pInfo->Position[0];
  double gantry1YPos = pInfo->Position[1];
  double gantry2XPos = pInfo->Position[5];
  double gantry2YPos = pInfo->Position[6];
  double gantry1XDes = pInfo->Destination[0];
  double gantry1YDes = pInfo->Destination[1];
  double gantry2XDes = pInfo->Destination[5];
  double gantry2YDes = pInfo->Destination[6];

  // for rotation, need to consider both initial and final states of system -- as rotation may affect whether gantry collides with tank or other gantry
  // Gantry XY corners in global coordinates, when at the gantries limit switches, with final rotation conditions
  double gantry1XDimensions_rotfirst[4];
  double gantry1YDimensions_rotfirst[4];
  double gantry2XDimensions_rotfirst[4];
  double gantry2YDimensions_rotfirst[4];
  // Gantry XY corners in global coordinates, when at the gantries limit switches, with initial rotation conditions
  double gantry1XDimensions_rotsecond[4];
  double gantry1YDimensions_rotsecond[4];
  double gantry2XDimensions_rotsecond[4];
  double gantry2YDimensions_rotsecond[4];

  // both gantries can move in different Z layers, eg. g0 above tank and g1 in tank
  // therefore we need different tank boundary conditions for each gantry
  double container1XDimensions_start[4];     
  double container1YDimensions_start[4];
  double container2XDimensions_start[4];     
  double container2YDimensions_start[4];
  double container1XDimensions_end[4]; 
  double container1YDimensions_end[4];
  double container2XDimensions_end[4];
  double container2YDimensions_end[4];

  //Final state of gantry1 & gantry2, after rotation with XY limit corners  
  gantry1YDimensions_rotfirst[0] = -0.070*cos(pInfo->Destination[3]*3.141592654/180)-0.200*sin(pInfo->Destination[3]*3.141592654/180);
  gantry1XDimensions_rotfirst[0] = -0.200*cos(pInfo->Destination[3]*3.141592654/180)+0.070*sin(pInfo->Destination[3]*3.141592654/180);
  gantry1YDimensions_rotfirst[1] = -0.070*cos(pInfo->Destination[3]*3.141592654/180)+0.140*sin(pInfo->Destination[3]*3.141592654/180);
  gantry1XDimensions_rotfirst[1] = 0.140*cos(pInfo->Destination[3]*3.141592654/180)+0.070*sin(pInfo->Destination[3]*3.141592654/180);
  gantry1YDimensions_rotfirst[2] = 0.160*cos(pInfo->Destination[3]*3.141592654/180)+0.140*sin(pInfo->Destination[3]*3.141592654/180);
  gantry1XDimensions_rotfirst[2] = 0.140*cos(pInfo->Destination[3]*3.141592654/180)-0.160*sin(pInfo->Destination[3]*3.141592654/180);  
  gantry1YDimensions_rotfirst[3] = 0.160*cos(pInfo->Destination[3]*3.141592654/180)-0.200*sin(pInfo->Destination[3]*3.141592654/180);
  gantry1XDimensions_rotfirst[3] = -0.200*cos(pInfo->Destination[3]*3.141592654/180)-0.160*sin(pInfo->Destination[3]*3.141592654/180);

  gantry2YDimensions_rotfirst[0] = 0.070*cos(pInfo->Destination[8]*3.141592654/180)+0.200*sin(pInfo->Destination[8]*3.141592654/180);
  gantry2XDimensions_rotfirst[0] = -0.140*cos(pInfo->Destination[8]*3.141592654/180)+0.160*sin(pInfo->Destination[8]*3.141592654/180);
  gantry2YDimensions_rotfirst[1] = 0.070*cos(pInfo->Destination[8]*3.141592654/180)-0.140*sin(pInfo->Destination[8]*3.141592654/180);
  gantry2XDimensions_rotfirst[1] = 0.200*cos(pInfo->Destination[8]*3.141592654/180)+0.160*sin(pInfo->Destination[8]*3.141592654/180);
  gantry2YDimensions_rotfirst[2] = -0.160*cos(pInfo->Destination[8]*3.141592654/180)-0.140*sin(pInfo->Destination[8]*3.141592654/180);
  gantry2XDimensions_rotfirst[2] = 0.200*cos(pInfo->Destination[8]*3.141592654/180)-0.070*sin(pInfo->Destination[8]*3.141592654/180);
  gantry2YDimensions_rotfirst[3] = -0.160*cos(pInfo->Destination[8]*3.141592654/180)+0.200*sin(pInfo->Destination[8]*3.141592654/180);
  gantry2XDimensions_rotfirst[3] = -0.140*cos(pInfo->Destination[8]*3.141592654/180)-0.070*sin(pInfo->Destination[8]*3.141592654/180);

  //Final state of gantry1 & gantry2, before rotation with XY limit corners  
  gantry1YDimensions_rotsecond[0] = -0.070*cos(pInfo->Position[3]*3.141592654/180)-0.200*sin(pInfo->Position[3]*3.141592654/180);
  gantry1XDimensions_rotsecond[0] = -0.200*cos(pInfo->Position[3]*3.141592654/180)+0.070*sin(pInfo->Position[3]*3.141592654/180);
  gantry1YDimensions_rotsecond[1] = -0.070*cos(pInfo->Position[3]*3.141592654/180)+0.140*sin(pInfo->Position[3]*3.141592654/180);
  gantry1XDimensions_rotsecond[1] = 0.140*cos(pInfo->Position[3]*3.141592654/180)+0.070*sin(pInfo->Position[3]*3.141592654/180);
  gantry1YDimensions_rotsecond[2] = 0.160*cos(pInfo->Position[3]*3.141592654/180)+0.140*sin(pInfo->Position[3]*3.141592654/180);
  gantry1XDimensions_rotsecond[2] = 0.140*cos(pInfo->Position[3]*3.141592654/180)-0.160*sin(pInfo->Position[3]*3.141592654/180);
  gantry1YDimensions_rotsecond[3] = 0.160*cos(pInfo->Position[3]*3.141592654/180)-0.200*sin(pInfo->Position[3]*3.141592654/180);
  gantry1XDimensions_rotsecond[3] = -0.200*cos(pInfo->Position[3]*3.141592654/180)-0.160*sin(pInfo->Position[3]*3.141592654/180);
  
  gantry2YDimensions_rotsecond[0] = 0.070*cos(pInfo->Position[8]*3.141592654/180)+0.200*sin(pInfo->Position[8]*3.141592654/180);
  gantry2XDimensions_rotsecond[0] = -0.140*cos(pInfo->Position[8]*3.141592654/180)+0.160*sin(pInfo->Position[8]*3.141592654/180);
  gantry2YDimensions_rotsecond[1] = 0.070*cos(pInfo->Position[8]*3.141592654/180)-0.140*sin(pInfo->Position[8]*3.141592654/180);
  gantry2XDimensions_rotsecond[1] = 0.200*cos(pInfo->Position[8]*3.141592654/180)+0.160*sin(pInfo->Position[8]*3.141592654/180);
  gantry2YDimensions_rotsecond[2] = -0.160*cos(pInfo->Position[8]*3.141592654/180)-0.140*sin(pInfo->Position[8]*3.141592654/180);
  gantry2XDimensions_rotsecond[2] = 0.200*cos(pInfo->Position[8]*3.141592654/180)-0.070*sin(pInfo->Position[8]*3.141592654/180);
  gantry2YDimensions_rotsecond[3] = -0.160*cos(pInfo->Position[8]*3.141592654/180)+0.200*sin(pInfo->Position[8]*3.141592654/180);
  gantry2XDimensions_rotsecond[3] = -0.140*cos(pInfo->Position[8]*3.141592654/180)-0.070*sin(pInfo->Position[8]*3.141592654/180);

  // determine whether boxes are inside tank or above tank
  bool tankheight_gantstart1 = 0;
  if(pInfo->Position[2] < tankHeight) tankheight_gantstart1 = 1;
  bool tankheight_gantstart2 = 0;
  if(pInfo->Position[7] < tankHeight) tankheight_gantstart2 = 1;
  
  // TODO: make this code more elegant as it just changes by i+5
  // This either means I will move up (so z moves first) or I'm moving above the tank
  if(tankheight_gantstart1){
    // conservative outer "tank" dimensions
    container1YDimensions_start[0] = -0.205;
    container1XDimensions_start[0] = -0.05;
    container1YDimensions_start[1] = 0.735;
    container1XDimensions_start[1] = -0.05;
    container1YDimensions_start[2] = 0.735;
    container1XDimensions_start[2] = 1.05;
    container1YDimensions_start[3] = -0.205;
    container1XDimensions_start[3] = 1.05;
  } else {
    // moving into tank (so z will be moving last) or already moving in tank
    // need to compare with more stringent real tank dimensions:
    // measured June 25, 2015 (remeasure for each tank installation??)
    container1YDimensions_start[0] = 0.062;     
    container1XDimensions_start[0] = 0.052;
    container1YDimensions_start[1] = 0.640;
    container1XDimensions_start[1] = 0.052;
    container1YDimensions_start[2] = 0.640;  // not measured, but chose rectangular
    container1XDimensions_start[2] = 0.620;  // not measured, but chose rectangular
    container1YDimensions_start[3] = 0.062;
    container1XDimensions_start[3] = 0.620;
  } 

  if(tankheight_gantstart2){
    container2YDimensions_start[0] = -0.205;
    container2XDimensions_start[0] = -0.05;
    container2YDimensions_start[1] = 0.735;
    container2XDimensions_start[1] = -0.05;
    container2YDimensions_start[2] = 0.735;
    container2XDimensions_start[2] = 1.05;
    container2YDimensions_start[3] = -0.205;
    container2XDimensions_start[3] = 1.05;
  } else {
    container2YDimensions_start[0] = 0.062;     
    container2XDimensions_start[0] = 0.052;
    container2YDimensions_start[1] = 0.640;
    container2XDimensions_start[1] = 0.052;
    container2YDimensions_start[2] = 0.640;  // not measured, but chose rectangular
    container2XDimensions_start[2] = 0.620;  // not measured, but chose rectangular
    container2YDimensions_start[3] = 0.062;
    container2XDimensions_start[3] = 0.620;
  } 

  bool tankheight_gantend1 = 0;
  if(pInfo->Destination[2] < tankHeight) tankheight_gantend1 = 1;
  bool tankheight_gantend2 = 0;
  if(pInfo->Destination[7] < tankHeight) tankheight_gantend2 = 1;

  if(tankheight_gantend1){
    container1YDimensions_end[0] = -0.205;
    container1XDimensions_end[0] = -0.05;
    container1YDimensions_end[1] = 0.735;
    container1XDimensions_end[1] = -0.05;
    container1YDimensions_end[2] = 0.735;
    container1XDimensions_end[2] = 1.05;
    container1YDimensions_end[3] = -0.205;
    container1XDimensions_end[3] = 1.05;
  } else {
    container1YDimensions_end[0] = 0.062;
    container1XDimensions_end[0] = 0.052;
    container1YDimensions_end[1] = 0.640;
    container1XDimensions_end[1] = 0.052;
    container1YDimensions_end[2] = 0.640;  // not measured, but chose rectangular
    container1XDimensions_end[2] = 0.620;  // not measured, but chose rectangular
    container1YDimensions_end[3] = 0.062;
    container1XDimensions_end[3] = 0.620;
  }
  if(tankheight_gantend2){
    container2YDimensions_end[0] = -0.205;
    container2XDimensions_end[0] = -0.05;
    container2YDimensions_end[1] = 0.735;
    container2XDimensions_end[1] = -0.05;
    container2YDimensions_end[2] = 0.735;
    container2XDimensions_end[2] = 1.05;
    container2YDimensions_end[3] = -0.205;
    container2XDimensions_end[3] = 1.05;
  } else {
    container2YDimensions_end[0] = 0.062;
    container2XDimensions_end[0] = 0.052;
    container2YDimensions_end[1] = 0.640;
    container2XDimensions_end[1] = 0.052;
    container2YDimensions_end[2] = 0.640;  // not measured, but chose rectangular
    container2XDimensions_end[2] = 0.620;  // not measured, but chose rectangular
    container2YDimensions_end[3] = 0.062;
    container2XDimensions_end[3] = 0.620;
  }
 
  // possible permutations of start/end positions and rotations
  std::vector<std::pair<double, double> > gantry1_startpos_rotfirst;
  std::vector<std::pair<double, double> > gantry1_startpos_rotsecond;
  std::vector<std::pair<double, double> > gantry1_endpos_rotfirst;
  std::vector<std::pair<double, double> > gantry1_endpos_rotsecond;
  std::vector<std::pair<double, double> > gantry2_startpos_rotfirst;
  std::vector<std::pair<double, double> > gantry2_startpos_rotsecond;
  std::vector<std::pair<double, double> > gantry2_endpos_rotfirst;
  std::vector<std::pair<double, double> > gantry2_endpos_rotsecond;
  std::vector<std::pair<double, double> > container1_start;
  std::vector<std::pair<double, double> > container2_start;
  std::vector<std::pair<double, double> > container1_end;
  std::vector<std::pair<double, double> > container2_end;

  for (int i = 0; i < 4; ++i){
    gantry1_startpos_rotfirst.push_back(std::make_pair(gantry1XPos+gantry1XDimensions_rotfirst[i], gantry1YPos + gantry1YDimensions_rotfirst[i]));
    gantry1_startpos_rotsecond.push_back(std::make_pair(gantry1XPos+gantry1XDimensions_rotsecond[i], gantry1YPos + gantry1YDimensions_rotsecond[i]));
    gantry1_endpos_rotfirst.push_back(std::make_pair(gantry1XDes+gantry1XDimensions_rotfirst[i], gantry1YDes + gantry1YDimensions_rotfirst[i]));
    gantry1_endpos_rotsecond.push_back(std::make_pair(gantry1XDes+gantry1XDimensions_rotsecond[i], gantry1YDes + gantry1YDimensions_rotsecond[i]));
    gantry2_startpos_rotfirst.push_back(std::make_pair(gantry2XPos+gantry2XDimensions_rotfirst[i], gantry2YPos + gantry2YDimensions_rotfirst[i]));
    gantry2_startpos_rotsecond.push_back(std::make_pair(gantry2XPos+gantry2XDimensions_rotsecond[i], gantry2YPos + gantry2YDimensions_rotsecond[i]));
    gantry2_endpos_rotfirst.push_back(std::make_pair(gantry2XDes+gantry2XDimensions_rotfirst[i], gantry2YDes + gantry2YDimensions_rotfirst[i]));
    gantry2_endpos_rotsecond.push_back(std::make_pair(gantry2XDes+gantry2XDimensions_rotsecond[i], gantry2YDes + gantry2YDimensions_rotsecond[i]));
    container1_start.push_back(std::make_pair(container1XDimensions_start[i], container1YDimensions_start[i]));
    container2_start.push_back(std::make_pair(container2XDimensions_start[i], container2YDimensions_start[i]));
    container1_end.push_back(std::make_pair(container1XDimensions_end[i], container1YDimensions_end[i]));
    container2_end.push_back(std::make_pair(container2XDimensions_end[i], container2YDimensions_end[i]));
  }

  // Path calculator checks if destination is inside the other gantry or outside the tank, but doesn't check rotation
  // Leave this check here for now
  if(pInfo->Destination[3] < rot_min || pInfo->Destination[3] > rot_max){
    cm_msg(MERROR, "generate_path", "Illegal value for 1st rotary angle of %5.2f: not between %5.2f and %5.2f",
	   pInfo->Destination[3],rot_min,rot_max);
    return GENPATH_BAD_DEST;
  } 
  if(pInfo->Destination[8] < rot_min || pInfo->Destination[8] > rot_max){
    cm_msg(MERROR, "generate_path", "Illegal value for 2nd rotary angle of %5.2f: not between %5.2f and %5.2f",
	   pInfo->Destination[8],rot_min,rot_max);
    return GENPATH_BAD_DEST;
  } 
  if(pInfo->Destination[4] < tilt_min || pInfo->Destination[4] > tilt_max){
    cm_msg(MERROR, "generate_path", "Illegal value for 1st tilt angle of %5.2f: not between %5.2f and %5.2f",
	   pInfo->Destination[4],tilt_min,tilt_max);
    return GENPATH_BAD_DEST;
  } 
  if(pInfo->Destination[9] < tilt_min || pInfo->Destination[9] > tilt_max){
    cm_msg(MERROR, "generate_path", "Illegal value for 2nd tilt angle of %5.2f: not between %5.2f and %5.2f",
	   pInfo->Destination[9],tilt_min,tilt_max);
    return GENPATH_BAD_DEST;
  }
  //TODO: make this check tilt and xy-dependent!!
  if(pInfo->Destination[2] > pmtHeight || pInfo->Destination[7] > pmtHeight){
    cm_msg(MERROR, "generate_path", "Illegal value for Z position. You will hit the PMT: should be smaller than %5.2f",pmtHeight);
    return GENPATH_BAD_DEST;
  } 
  
  // Z movement: if the end position of gantry 1 or 2 is above the water tank, move in the z-direction first -- SAFE, as always moving out of tank
  if(tankheight_gantend1) move_z1_first = true;
  if(tankheight_gantend2) move_z2_first = true;

std::cout << "Move z first:  " << move_z1_first << "  " << move_z2_first << std::endl;

  // First check: beams should never cross each other
  //if(pInfo->Destination[0] + 0.05 >= pInfo->Destination[5]){ // conservative
  if(pInfo->Destination[0] - 0.05 >= pInfo->Destination[5]){  //closest possible
    cm_msg(MERROR, "generate_path", "Illegal value for X destination: Beams will collide");
    return GENPATH_BAD_DEST;
  }  

  // Initialise gantries and water tank in path calculator for gantry 1 move first, rotation first
  pathCalc1a.InitialiseGantries(gantry1_startpos_rotfirst, gantry2_startpos_rotfirst);
  pathCalc1b.InitialiseGantries(gantry1_endpos_rotfirst, gantry2_startpos_rotfirst);
  //Initialise gantries and water tank in path calculator for gantry 1 move first, rotation second
  pathCalc2a.InitialiseGantries(gantry1_startpos_rotsecond, gantry2_startpos_rotsecond);
  pathCalc2b.InitialiseGantries(gantry1_endpos_rotsecond, gantry2_startpos_rotsecond);
  pathCalc2c.InitialiseGantries(gantry1_endpos_rotfirst, gantry2_endpos_rotfirst);
  // Initialise gantries and water tank in path calculator for gantry 2 move first, rotation first
  pathCalc3a.InitialiseGantries(gantry1_startpos_rotfirst, gantry2_startpos_rotfirst); // gantry 2 move
  pathCalc3b.InitialiseGantries(gantry1_startpos_rotfirst, gantry2_endpos_rotfirst); //gantry 1 move
  // Initialise gantries and water tank in path calculator for gantry 2 move first, rotation second
  pathCalc4a.InitialiseGantries(gantry1_startpos_rotsecond, gantry2_startpos_rotsecond);
  pathCalc4b.InitialiseGantries(gantry1_startpos_rotsecond, gantry2_endpos_rotsecond);
  pathCalc4c.InitialiseGantries(gantry1_endpos_rotfirst, gantry2_endpos_rotsecond);

  // Start, end, and path for gantry 1 move first 
  std::pair<double, double> start1 = std::make_pair(gantry1XPos, gantry1YPos);
  std::pair<double, double> end1 = std::make_pair(gantry1XDes, gantry1YDes);
  std::vector<std::pair<double, double> > path1;
  // Start, end, and path for gantry 2 move first
  std::pair<double, double> start2 = std::make_pair(gantry2XPos, gantry2YPos);
  std::pair<double, double> end2 = std::make_pair(gantry2XDes, gantry2YDes);
  std::vector<std::pair<double, double> > path2;

  // Start, end, and path for gantry 1 move first 
  std::pair<double, double> start_rot1 = std::make_pair(gantry1XPos, gantry1YPos);
  std::pair<double, double> start_rot2 = std::make_pair(gantry2XPos, gantry2YPos);
  std::pair<double, double> end_rot1 = std::make_pair(gantry1XDes, gantry1YDes);
  std::pair<double, double> end_rot2 = std::make_pair(gantry2XDes, gantry2YDes);
  std::pair<double, double> rot1 = std::make_pair(pInfo->Position[3], pInfo->Destination[3]);
  std::pair<double, double> rot2 = std::make_pair(pInfo->Position[8], pInfo->Destination[8]);

  // determine good Paths
  bool goodPath00 = false, goodPath01 = false;

  // calculate good paths for gantry 1 move first, rotate first
  pathCalc1a.InitialiseContainer(container1_end); 
  pathCalc1b.InitialiseContainer(container2_end);
  bool goodPath1a = pathCalc1a.CalculatePath(start1, end1, path1);
  bool goodPath1b = pathCalc1b.CalculatePath(start2, end2, path2);
  // calculate good paths for gantry 1 move first, rotate second
  pathCalc2a.InitialiseContainer(container1_end); 
  pathCalc2b.InitialiseContainer(container2_end); 
  pathCalc2c.InitialiseContainer(container1_end);
  bool goodPath2a = pathCalc2a.CalculatePath(start1, end1, path1);
  bool goodPath2b = pathCalc2b.CalculatePath(start2, end2, path2);
  bool goodPath2c = pathCalc2c.CalculatePath(end1, end1, path1);
  // calculate good paths for gantry 2 move first, rotate first
  pathCalc3a.InitialiseContainer(container2_end); 
  pathCalc3b.InitialiseContainer(container1_end); 
  bool goodPath3a = pathCalc3a.CalculatePath(start2, end2, path2);
  bool goodPath3b = pathCalc3b.CalculatePath(start1, end1, path1);
  // calculate good paths for gantry 2 move first, rotate second
  pathCalc4a.InitialiseContainer(container2_end);
  pathCalc4b.InitialiseContainer(container1_end);
  pathCalc4c.InitialiseContainer(container2_end);
  bool goodPath4a = pathCalc4a.CalculatePath(start2, end2, path2);
  bool goodPath4b = pathCalc4b.CalculatePath(start1, end1, path1);
  bool goodPath4c = pathCalc4c.CalculatePath(end2, end2, path2);
  
  pathCalc_rotstart.InitialiseContainer(container1_start, container2_start);
  pathCalc_rotend.InitialiseContainer(container1_end, container2_end);
  bool goodRotation_start = pathCalc_rotstart.CalculatePath(start_rot1, start_rot2, rot1, rot2, tankheight_gantstart1, tankheight_gantstart2);
  bool goodRotation_end = pathCalc_rotend.CalculatePath(end_rot1, end_rot2, rot1, rot2, tankheight_gantend1, tankheight_gantend2);

  // Determine which motion the gantry system can take: 1) gantry1 move first, rotate first 2) gantry1 move first, rotate second 3) gantry2 move first, rotate first 4) gantry2 move first, rotate second
  if(goodPath1a && goodPath1b && goodRotation_start){
    cm_msg(MINFO,"generate_path","Good path identified: Gantry 1 move first, rotation first");
    pathCalc00.InitialiseGantries(gantry1_startpos_rotfirst, gantry2_startpos_rotfirst);
    pathCalc00.InitialiseContainer(container1_end);
    goodPath00 = pathCalc00.CalculatePath(start1, end1, path1);
    pathCalc01.InitialiseGantries(gantry1_endpos_rotfirst, gantry2_startpos_rotfirst);
    pathCalc01.InitialiseContainer(container2_end);
    goodPath01 = pathCalc01.CalculatePath(start2, end2, path2);
    move_rotation_first = true;
  } else if(goodPath2a && goodPath2b && goodPath2c && goodRotation_end){
    cm_msg(MINFO,"generate_path","Good path identified: Gantry 1 move first, rotation second");
    pathCalc00.InitialiseGantries(gantry1_startpos_rotsecond, gantry2_startpos_rotsecond);
    pathCalc00.InitialiseContainer(container1_end);
    goodPath00 = pathCalc00.CalculatePath(start1, end1, path1);
    pathCalc01.InitialiseGantries(gantry1_endpos_rotsecond, gantry2_startpos_rotsecond);
    pathCalc01.InitialiseContainer(container2_end);
    goodPath01 = pathCalc01.CalculatePath(start2, end2, path2);
  } else if(goodPath3a && goodPath3b && goodRotation_start){
    cm_msg(MINFO,"generate_path","Good path identified: Gantry 2 move first, rotation first");
    pathCalc00.InitialiseGantries(gantry1_startpos_rotfirst, gantry2_endpos_rotfirst);
    pathCalc00.InitialiseContainer(container1_end);
    goodPath00 = pathCalc00.CalculatePath(start1, end1, path1);
    pathCalc01.InitialiseGantries(gantry1_startpos_rotfirst, gantry2_startpos_rotfirst);
    pathCalc01.InitialiseContainer(container2_end);
    goodPath01 = pathCalc01.CalculatePath(start2, end2, path2);
    //Check first whether we are not crossing a beam in a way that gantry1 had to move first
    if(pInfo->Destination[5] + 0.05 <= gantry1XPos){  // closest possible
      cm_msg(MERROR, "generate_path","Illegal value for X destination gantry2: will collide against beam gantry 1.");
      return GENPATH_BAD_DEST;
    }
    move_rotation_first = true;
    move_second_gantry_first = true;
  } else if(goodPath4a && goodPath4b && goodPath4c && goodRotation_end){
    cm_msg(MINFO,"generate_path","Good path identified: Gantry 2 move first, rotation second");
    pathCalc00.InitialiseGantries(gantry1_startpos_rotsecond, gantry2_endpos_rotsecond);
    pathCalc00.InitialiseContainer(container1_end);
    goodPath00 = pathCalc00.CalculatePath(start1, end1, path1);
    pathCalc01.InitialiseGantries(gantry1_startpos_rotsecond, gantry2_startpos_rotsecond);
    pathCalc01.InitialiseContainer(container2_end);
    goodPath01 = pathCalc01.CalculatePath(start2, end2, path2);
    //Check first whether we are not crossing a beam in a way that gantry1 had to move first
    if(pInfo->Destination[5] + 0.05 <= gantry1XPos){  // closest possible
      cm_msg(MERROR, "generate_path","Illegal value for X destination gantry2: will collide against beam gantry 1.");
      return GENPATH_BAD_DEST;
    }
    move_second_gantry_first = true;
  } else {
    cm_msg(MERROR,"generate_path","No good path available");
    return GENPATH_BAD_DEST;
  }

  //Check whether we are not crossing a beam, because PathCalculator does not know about the beam
  //if(pInfo->Destination[0] + 0.05 >= gantry2XPos){   //conservative
  if(pInfo->Destination[0] - 0.05 >= gantry2XPos){     // closest possible
    cm_msg(MINFO,"generate_path","Illegal value for X destination gantry1: will collide against beam gantry 2. Only possible if you move gantry2 out of the way first");
    move_second_gantry_first = true;
  }
  
  //Initialize path to current position
  for(int i = 0; i < 10 ; i++){
    //Why: +3: optional zfirst, then zlast, theta, phi is last move and very first one is initial pos.
    pInfo->MovePath[i] = (float *) calloc(path1.size() + path2.size() +3,sizeof(float));
    for(unsigned int j = 0; j < path1.size() + path2.size() + 3; ++j){
      pInfo->MovePath[i][j] = pInfo->CountPos[i];
    }
  }

  // NEW
  if(move_z1_first){
    pInfo->MovePath[2][1] = round((pInfo->Destination[2]-pInfo->LimPos[2])*pInfo->mScale[2] + pInfo->mOrigin[2]);
  }
  if(move_z2_first){
    pInfo->MovePath[7][1] = round((pInfo->Destination[7]-pInfo->LimPos[7])*pInfo->mScale[7] + pInfo->mOrigin[7]);
  } 

  // move rotation and tilt first if final rotation, final position is allowed  
  if(move_rotation_first){
    pInfo->MovePath[3][1] = round((pInfo->Destination[3]-pInfo->LimPos[3])*pInfo->mScale[3] + pInfo->mOrigin[3]);
    pInfo->MovePath[4][1] = round((pInfo->Destination[4]-pInfo->LimPos[4])*pInfo->mScale[4] + pInfo->mOrigin[4]);

    pInfo->MovePath[8][1] = round((pInfo->Destination[8]-pInfo->LimPos[8])*pInfo->mScale[8] + pInfo->mOrigin[8]);
    pInfo->MovePath[9][1] = round((pInfo->Destination[9]-pInfo->LimPos[9])*pInfo->mScale[9] + pInfo->mOrigin[9]);
  }
 
  if(!move_second_gantry_first){
    // Put in the steps to move gantry 1 in X and Y
    for(int i = gantry_motor_start; i < gantry_motor_end ; i++){
      if(pInfo->Channels[i] != -1){
        for(unsigned int j = 1; j < path1.size() + 1; ++j){
          // Move in X or Y according to determined path for first gantry
          if(i == 0) pInfo->MovePath[i][j + 1] = round(pInfo->MovePath[i][j] + path1[j-1].first*pInfo->mScale[i]);   
          else if (i == 1) pInfo->MovePath[i][j + 1] = round(pInfo->MovePath[i][j] + path1[j-1].second*pInfo->mScale[i]);
          else pInfo->MovePath[i][j + 1] = pInfo->MovePath[i][j];
        }
      }
    }

    // Put in the steps to move gantry 2 in X and Y
    for(int i = gantry_motor_start; i < gantry_motor_end ; i++){
      if(pInfo->Channels[i] != -1){
        for(unsigned int j = 1; j < path2.size() + 1; ++j){
          // Move in X or Y according to determined path for first gantry
          if(i == 5) pInfo->MovePath[i][j + 1 + path1.size()] = round(pInfo->MovePath[i][j + path1.size()] + path2[j-1].first*pInfo->mScale[i]);
          else if (i == 6) pInfo->MovePath[i][j + 1 + path1.size()] = round(pInfo->MovePath[i][j + path1.size()] + path2[j-1].second*pInfo->mScale[i]);
          else pInfo->MovePath[i][j + 1 + path1.size()] = pInfo->MovePath[i][j + path1.size()];
        }
      }
    }
  }
  else{
    // Put in the steps to move gantry 2 in X and Y
    for(int i = gantry_motor_start; i < gantry_motor_end ; i++){
      if(pInfo->Channels[i] != -1){
        for(unsigned int j = 1; j < path2.size() + 1; ++j){
          // Move in X or Y according to determined path for first gantry
          if(i == 5) pInfo->MovePath[i][j + 1] = round(pInfo->MovePath[i][j] + path2[j-1].first*pInfo->mScale[i]);
          else if (i == 6) pInfo->MovePath[i][j + 1] = round(pInfo->MovePath[i][j] + path2[j-1].second*pInfo->mScale[i]);
          else pInfo->MovePath[i][j + 1] = pInfo->MovePath[i][j];
        }
      }
    }
    // Put in the steps to move gantry 1 in X and Y
    for(int i = gantry_motor_start; i < gantry_motor_end ; i++){
      if(pInfo->Channels[i] != -1){
        for(unsigned int j = 1; j < path1.size() + 1; ++j){
          // Move in X or Y according to determined path for first gantry
          if(i == 0) pInfo->MovePath[i][j + 1 + path2.size()] = round(pInfo->MovePath[i][j + path2.size()] + path1[j-1].first*pInfo->mScale[i]);
          else if (i == 1) pInfo->MovePath[i][j + 1 + path2.size()] = round(pInfo->MovePath[i][j + path2.size()] + path1[j-1].second*pInfo->mScale[i]);
          else pInfo->MovePath[i][j + 1 + path2.size()] = pInfo->MovePath[i][j + path2.size()];
        }
      }
    }
  }

  // Keep X and Y coordinates of gantries at end of move
  pInfo->MovePath[0][path1.size() + path2.size() + 2] = pInfo->MovePath[0][path1.size() + path2.size() + 1];
  pInfo->MovePath[1][path1.size() + path2.size() + 2] = pInfo->MovePath[1][path1.size() + path2.size() + 1];
  pInfo->MovePath[5][path1.size() + path2.size() + 2] = pInfo->MovePath[5][path1.size() + path2.size() + 1];
  pInfo->MovePath[6][path1.size() + path2.size() + 2] = pInfo->MovePath[6][path1.size() + path2.size() + 1];

  // Lower gantry heads at end and rotate
  if(!move_z1_first)
    pInfo->MovePath[2][path1.size() + path2.size() + 2] = round((pInfo->Destination[2]-pInfo->LimPos[2])*pInfo->mScale[2] + pInfo->mOrigin[2]);
  else
    pInfo->MovePath[2][path1.size() + path2.size() + 2] = pInfo->MovePath[2][path1.size() + path2.size() + 1];
  if(!move_z2_first)
    pInfo->MovePath[7][path1.size() + path2.size() + 2] = round((pInfo->Destination[7]-pInfo->LimPos[7])*pInfo->mScale[7] + pInfo->mOrigin[7]);
  else
    pInfo->MovePath[7][path1.size() + path2.size() + 2] = pInfo->MovePath[7][path1.size() + path2.size() + 1];

  if(!move_rotation_first){  //move tilt and rotation 
    pInfo->MovePath[3][path1.size() + path2.size() + 2] = round((pInfo->Destination[3]-pInfo->LimPos[3])*pInfo->mScale[3] + pInfo->mOrigin[3]);
    pInfo->MovePath[4][path1.size() + path2.size() + 2] = round((pInfo->Destination[4]-pInfo->LimPos[4])*pInfo->mScale[4] + pInfo->mOrigin[4]);
 
    pInfo->MovePath[8][path1.size() + path2.size() + 2] = round((pInfo->Destination[8]-pInfo->LimPos[8])*pInfo->mScale[8] + pInfo->mOrigin[8]);
    pInfo->MovePath[9][path1.size() + path2.size() + 2] = round((pInfo->Destination[9]-pInfo->LimPos[9])*pInfo->mScale[9] + pInfo->mOrigin[9]);
  } else{
    pInfo->MovePath[3][path1.size() + path2.size() + 2] = pInfo->MovePath[3][path1.size() + path2.size() + 1];
    pInfo->MovePath[4][path1.size() + path2.size() + 2] = pInfo->MovePath[4][path1.size() + path2.size() + 1];

    pInfo->MovePath[8][path1.size() + path2.size() + 2] = pInfo->MovePath[8][path1.size() + path2.size() + 1];
    pInfo->MovePath[9][path1.size() + path2.size() + 2] = pInfo->MovePath[9][path1.size() + path2.size() + 1];
  }

  for (unsigned int i = 0; i < path1.size() + path2.size() + 3; ++i){
    for(int j = gantry_motor_start; j < gantry_motor_end ; j++){
      cm_msg(MDEBUG,"generate_path","Destination[%i][%i] = %6.3f (Count Destination = %6.0f, Remaining =  %6.0f)",j,i,pInfo->Destination[j],pInfo->MovePath[j][i],pInfo->MovePath[j][i]-pInfo->CountPos[j]);
    }
  }

  pInfo->PathSize = path1.size() + path2.size() + 3;
  pInfo->PathIndex = 0; 

  return GENPATH_SUCCESS;
}

/*-- Move ----------------------------------------------------------*/
// Use channel_rw to send the subsequent path index to the motors
void move(INFO *pInfo) {
  int i;
  BOOL zerotest = 0;
  BOOL start[10] = {1,1,1,1,1,1,1,1,1,1};
  float Motor00StartPos[8];
  float Motor01StartPos[8];
  float Motor00Pos[8];
  float Motor01Pos[8];
  float Motor00Dest[8];
  float Motor01Dest[8];
  BOOL Motor00LimitPos[8];
  BOOL Motor00LimitNeg[8];
  BOOL Motor01LimitPos[8];
  BOOL Motor01LimitNeg[8];
  int size_bool = sizeof(Motor00LimitPos);
  int size_float = sizeof(Motor01Pos);
  int waiting = 1; 
  BOOL started_moving = 0;
  DWORD Overall_time_for_loop;
  DWORD start_of_loop; 
  
  //DEBUG
  printf("Moving to path index: %i\n",pInfo->PathIndex);

  // Read in axis positions (in counts)
  channel_rw(pInfo,pInfo->hKeyMPos,(void *)pInfo->CountPos,TID_FLOAT,0);

  // Determine required destinations to be sent to the motors
  for(i = gantry_motor_start; i < gantry_motor_end ; i++){
    pInfo->CountDest[i] = pInfo->MovePath[i][pInfo->PathIndex] - pInfo->CountPos[i];
    //DEBUG
    printf("MD[%i]=%6.0f(%6.0f) ",i,pInfo->CountDest[i],pInfo->CountPos[i]);
    if(i==4 || i == 9){
      printf("\n");
    }
    zerotest = zerotest || pInfo->CountDest[i];
  }

  // This is added so that the monitor recognizes a move as completed, even
  // when no move is required.
  if(!zerotest){
    cm_msg(MINFO,"move","Warning: No move required");
    // This indicates to the monitor that a move has been initiated
    // even though the motors won't start moving.
    //TODO:: BK: Think of a better way of doing this. The Moving variable should only be used to indicate that the system is moving it could cause confusion when you set it based on other conditions.(This is me being picky)
    pInfo->Moving = 1;
    return;
  }

  // Start motors towards the specified destinations
  // Write motor destinations to the ODB
  channel_rw(pInfo,pInfo->hKeyMDest,(void *)pInfo->CountDest,TID_FLOAT,1);
  
  // Get the current location of the motor. This will be used to tell if a motor has started moving yet
  db_get_data(pInfo->hDB,pInfo->hKeyMPos[0],&Motor00StartPos,&size_float,TID_FLOAT);
  db_get_data(pInfo->hDB,pInfo->hKeyMPos[1],&Motor01StartPos,&size_float,TID_FLOAT); 
  db_get_data(pInfo->hDB,pInfo->hKeyMDest[0],&Motor00Dest,&size_float,TID_FLOAT);
  db_get_data(pInfo->hDB,pInfo->hKeyMDest[1],&Motor01Dest,&size_float,TID_FLOAT);

// Get the time at the beggining of the loop so that we can track how long we are in the loop
  Overall_time_for_loop = ss_millitime();

// Don't do anything else until the motors have started moving
  while(!started_moving) { 
    start_of_loop = ss_millitime();
    // DEBUG
    printf("\nChannel_rw called at time : %lf\n",ss_millitime());
    // Set the ODB values to start a move

    channel_rw(pInfo,pInfo->hKeyMStart,(void *)start,TID_BOOL,1);
    sleep(100);

    waiting = 1;
    while(waiting) {
      db_get_data(pInfo->hDB,pInfo->hKeyMPos[0],&Motor00Pos,&size_float,TID_FLOAT);
      db_get_data(pInfo->hDB,pInfo->hKeyMPos[1],&Motor01Pos,&size_float,TID_FLOAT);
      db_get_data(pInfo->hDB,pInfo->hKeyMLimitPos[0],&Motor00LimitPos,&size_bool,TID_BOOL);
      db_get_data(pInfo->hDB,pInfo->hKeyMLimitNeg[0],&Motor00LimitNeg,&size_bool,TID_BOOL);
      db_get_data(pInfo->hDB,pInfo->hKeyMLimitPos[1],&Motor01LimitPos,&size_bool,TID_BOOL);
      db_get_data(pInfo->hDB,pInfo->hKeyMLimitNeg[1],&Motor01LimitNeg,&size_bool,TID_BOOL);
      
      // if 300 seconds has passed and nothing has happened exit because the program is not working
      if (ss_millitime()- Overall_time_for_loop > 1000*300){
	cm_msg(MERROR,"move"," The Motors never started Moving (last 300s)");
	waiting = 0;
	started_moving = 1;
      }
      // TF NOTE: the code below is vulnerable to which motor channels are used. Switching a cable screws this up.
      // If any of the motors for gantry0 are no longer at their start position that means the motors are moving and the program behaved properly
      else if (Motor00Pos[0] != Motor00StartPos[0] || Motor00Pos[1] != Motor00StartPos[1] || Motor00Pos[2] != Motor00StartPos[2] 
	       || Motor00Pos[3] != Motor00StartPos[3] || Motor00Pos[4] != Motor00StartPos[4] || Motor00Pos[5] != Motor00StartPos[5] 
	       || Motor00Pos[6] != Motor00StartPos[6] || Motor00Pos[7] != Motor00StartPos[7]){
	cm_msg(MINFO,"move", " Motors gantry 0 are Moving ");
	waiting = 0;
	started_moving = 1;
	pInfo->Moving = 1;   //not necessary for long moves, but for mm moves, move can stop before monitor can check whether it's moving, so need to set here that is was really moving
      }
      // Same test for gantry1
      else if (Motor01Pos[0] != Motor01StartPos[0] || Motor01Pos[1] != Motor01StartPos[1] || Motor01Pos[2] != Motor01StartPos[2] 
	       || Motor01Pos[3] != Motor01StartPos[3] || Motor01Pos[4] != Motor01StartPos[4] || Motor01Pos[5] != Motor01StartPos[5] 
	       || Motor01Pos[6] != Motor01StartPos[6] || Motor01Pos[7] != Motor01StartPos[7]){
	cm_msg(MINFO,"move"," Motors gantry 1 are Moving ");
	waiting = 0;
	started_moving = 1;
	pInfo->Moving = 1;   //not necessary for long moves, but for mm moves, move can stop before monitor can check whether it's moving, so need to set here that is was really moving
      } 


      // If a motor for gantry0 is trying to move in the positive direction but the corresponding negative limit switch is engaged,
      // a move will not be started however no move is needed so set pinfo->moving to 1 so that the next move will be called
      else if( ((Motor00Dest[4] > 0) && Motor00LimitNeg[4]) || ((Motor00Dest[5] > 0) && Motor00LimitNeg[5]) 
	       || ((Motor00Dest[6] > 0) && Motor00LimitNeg[6]) || ((Motor00Dest[7] > 0) && Motor00LimitNeg[7]) ){
	cm_msg(MERROR,"move","Move could not be started because Motor00 is at a negative limit switch");
	waiting = 0;
	started_moving = 1;
	pInfo->Moving = 1;	
      }

      // Now testing for move in negative direction, and hitting positive limit switch for ganty0
      else if( ((Motor00Dest[4] < 0) && Motor00LimitPos[4]) ||((Motor00Dest[5] < 0) && Motor00LimitPos[5]) 
	       || ((Motor00Dest[6] < 0) && Motor00LimitPos[6]) || ((Motor00Dest[7] < 0) && Motor00LimitPos[7]) ){
	cm_msg(MERROR,"move","Move could not be started because Motor00 is at a Positive limit switch");
	waiting = 0;
	started_moving = 1;
	pInfo->Moving = 1;
      }

      // Same for gantry1
      else if( ((Motor01Dest[1] > 0) && Motor01LimitNeg[1]) ||((Motor01Dest[2] > 0) && Motor01LimitNeg[2]) 
	       || ((Motor01Dest[3] > 0) && Motor01LimitNeg[3]) || ((Motor01Dest[4] > 0) && Motor01LimitNeg[4]) ){
	cm_msg(MERROR,"move","Move could not be started because Motor01 is at a negative limit switch");
	waiting = 0;
	started_moving = 1;
	pInfo->Moving = 1;	
      }

      // Same for gantry1
      else if( ((Motor01Dest[1] < 0) && Motor01LimitPos[1]) ||((Motor01Dest[2] < 0) && Motor01LimitPos[2]) 
	       || ((Motor01Dest[3] < 0) && Motor01LimitPos[3]) || ((Motor01Dest[4] < 0) && Motor01LimitPos[4]) ){
	cm_msg(MERROR,"move","Move could not be started because Motor01 is at a Positive limit switch");
	waiting = 0;
	started_moving = 1;
	pInfo->Moving = 1;	
      }


      // If 5 seconds has passed and the motors haven't started moving reset the ODB values 
      // that should initiate a move with the hope that a move will start.
      else if(ss_millitime() - start_of_loop > 1000*5){
	cm_msg(MINFO,"move", "Calling channel_rw again after past 5s");
	waiting = 0;
      }
    }
  }  
  cm_msg(MINFO,"move","Function Move is complete");

}

/*-- Stop_move -----------------------------------------------------*/
// Aborts a move, and prints the reason for stopping to screen
void stop_move(HNDLE hDB, HNDLE hKey, void *data){
  INFO *pInfo = (INFO *) data;
  
  BOOL stop[10] = {0,0,0,0,0,0,0,0,0,0};
  channel_rw(pInfo,pInfo->hKeyMStop,(void *)stop,TID_BOOL,1);
    
  switch(pInfo->AbortCode){
  case AC_USER_INPUT:
    cm_msg(MERROR,"stop_move","Move aborted due to user input");
    break;
  case AC_COLLISION:
    cm_msg(MERROR,"stop_move","Move aborted due to collision detected");
    break;
  case AC_TIMEOUT:
    cm_msg(MERROR,"stop_move","Move aborted due to move timeout");
  }
  
  pInfo->AbortCode = AC_USER_INPUT; // reset the abort code to default (user input)
}

/*-- Monitor -------------------------------------------------------*/
// Called periodically with /Motors/Variables/Moving, this function
// checks for completion of a move
void monitor(HNDLE hDB,HNDLE hKey,void *data){

  INFO *pInfo = (INFO *) data;
  BOOL OldMov = pInfo->Moving; //Store the old value of pInfo->Moving for comparison to see if the value changes 
  int i = 0;
 
  /*-- Update variables section of ODB -----------------------------*/

  // Copy variable indicating if each axis is moving, and write this to ODB
  channel_rw(pInfo,pInfo->hKeyMMoving,(void *)pInfo->AxisMoving,TID_BOOL,0);
  db_set_data(pInfo->hDB,pInfo->hKeyAxMoving,pInfo->AxisMoving,10*sizeof(float),10,TID_BOOL);	
    
  // Check if any of the axes are moving, and write this to ODB
  pInfo->Moving = 0;
  while ((!pInfo->Moving) && i<10){
    pInfo->Moving = pInfo->Moving || pInfo->AxisMoving[i];
    i++;
  }
  db_set_data(pInfo->hDB,pInfo->hKeyMoving,&pInfo->Moving,sizeof(BOOL),1,TID_BOOL);
		
			
  // Get the axis positions and write to ODB
  channel_rw(pInfo,pInfo->hKeyMPos,(void *)pInfo->CountPos,TID_FLOAT,0);
  for(i = gantry_motor_start; i < gantry_motor_end ; i++){
    pInfo->Position[i] = (pInfo->CountPos[i] - pInfo->mOrigin[i])/pInfo->mScale[i] + pInfo->LimPos[i];	  
  }
  db_set_data(pInfo->hDB,pInfo->hKeyPos,pInfo->Position,10*sizeof(float),10,TID_FLOAT);
	
  // Check for limit switches triggered and write to ODB
  channel_rw(pInfo,pInfo->hKeyMLimitNeg,(void *)pInfo->neg_AxisLimit,TID_BOOL,0);
  channel_rw(pInfo,pInfo->hKeyMLimitPos,(void *)pInfo->pos_AxisLimit,TID_BOOL,0);
  db_set_data(pInfo->hDB,pInfo->hKeyAxLimitNeg,pInfo->neg_AxisLimit,10*sizeof(BOOL),10,TID_BOOL);
  db_set_data(pInfo->hDB,pInfo->hKeyAxLimitPos,pInfo->pos_AxisLimit,10*sizeof(BOOL),10,TID_BOOL);
	
  	
  /* - If motors have stopped moving, determine next course of action */
  bool stoppedDueToLimit = false;
  if(OldMov && !pInfo->Moving){ // i.e. If the motors just stopped moving
    // Check if the destination has been reached, otherwise return an error
    for(i = gantry_motor_start; i < gantry_motor_end ; i++){
      if((pInfo->Channels[i] != -1) && (pInfo->MovePath[i][pInfo->PathIndex] != pInfo->CountPos[i])){
      	if(pInfo->neg_AxisLimit[i] || pInfo->pos_AxisLimit[i]){ //already at limit OR hit limit after move
      	  stoppedDueToLimit = true;
      	  cm_msg(MINFO,"monitor","Stopped moving because LIMIT SWITCH for move %i reached!",i);
      	  // Resetting the path to take the current position at the limit (CountPos) as its destination
	  // This will make sure CountDest in move() is zero (after hitting a limit switch) and also
	  // MDest which is basically the same. The code checking why a move did not start will then 
	  // cause erroneous behaviour.
	  
	  //if moving towards limit and hitting it: stopped due to limit after move, so reset path 
	  if(((pInfo->MovePath[i][pInfo->PathIndex] - pInfo->CountPos[i]) < 0 && pInfo->pos_AxisLimit[i]) ||
	     ((pInfo->MovePath[i][pInfo->PathIndex] - pInfo->CountPos[i]) > 0 && pInfo->neg_AxisLimit[i])){
	    int path_i;
	    for(path_i = 0; path_i < pInfo->PathSize; path_i++){
	      pInfo->MovePath[i][path_i] = pInfo->CountPos[i];
	    }
	  }
	  //break; //no break, because also check for other hit limit switches, and reset their path/destination
      	} else {
	  if(!stoppedDueToLimit){
	    cm_msg(MERROR,"monitor","Move failed at i=%d, pathindex=%d : %6.2f, %6.2f",i,pInfo->PathIndex, pInfo->MovePath[i][pInfo->PathIndex], pInfo->CountPos[i]);
	    return;
	  }
      	}
      }
    }
		
    // If we make it this far, we have reached our specified path index
    printf("Move to index complete\n"); //DEBUG
    printf("Time to complete Move to index %lf\n",ss_millitime()); //DEBUG		
    // Check if we are at the final path index, otherwise initiate next move
    if(pInfo->PathIndex+1 == pInfo->PathSize) {
      // Final destination reached
      pInfo->Completed = 1;
      db_set_data(hDB,pInfo->hKeyCompleted,&pInfo->Completed,sizeof(BOOL),1,TID_BOOL);
      cm_msg(MINFO,"monitor","Move to destination complete");
      if(stoppedDueToLimit)
	cm_msg(MINFO,"monitor","But destination not reached due to limit switch");
    }
    else{
      pInfo->PathIndex++;
      // DEBUG
      printf("Monitor called move() at : %lf\n",ss_millitime());
      move(pInfo);
    }
  }
}

/*-- Channel Read/Write --------------------------------------------*/
// Used to communicate with the variables in the motor frontend.
// Arguments:
// 		pInfo: Info structure
//		hKey: Pointer to the ODB handle array to be read from/written to
//		values: Pointer to the array from which to read/write data
//		type: Type ID of the variable to be written (TID_FLOAT, TID_BOOL, TID_INT)
//		rw: set to 0 for read from ODB, 1 for write to ODB
void channel_rw(INFO *pInfo, HNDLE *hKey, void *values, DWORD type, BOOL rw){

//!!!!The Below comment is meant to help expalin how this function works as it uses "function-like" macros which people may not be famillar with. 

/*
//STORE_DATA and READ_DATA are "function-like" macros using the #define preprocessor macro (See lines 60 and 76) #define performs textual replacement. When used as a function-like macro #define will take arguments like a function. 
//EXAMPLE:: STORE_DATA(float, values, size) is a function-like macro. The preproccesor will replace the line STORE_DATA(float); with the lines of code listed after the #define (see line 76) and everywhere that the text "TYPE" appears in those lines of code will be replaced with the text float. END EXAMPLE
//The "function-like" are used to reduce the number of lines of code and to improve the readibility of the code. It is used to accomplish what templete functions do in C++
//NOTE!!:: Use of template functions can be difficult when debugging this section as the code you see is not what the compiler sees. The compiler sees the code after the preproccesor has completed the text replacement. If debugging this section it may be wise to not use the macro and instead do the textual replacement yourself and then debug. Comment by Ben Krupicz  
*/
   
  int size;
  int buff_size;
  void* motor00_values;
  void* motor01_values;

  
  switch(type){
    case TID_FLOAT:
      // STORE_DATA(TYPE) determines the correct value for the size value and fills  motor00_values and motor01_values with the entries in the values variable
      STORE_DATA(float);
       break;
    case TID_BOOL:
      STORE_DATA(BOOL);
      break;
    case TID_INT:
      STORE_DATA(int)
      break;
  }
    
  if(rw ==0) {
      db_get_data(pInfo->hDB,hKey[0],motor00_values,&buff_size,type);
      db_get_data(pInfo->hDB,hKey[1],motor01_values,&buff_size,type);
      
      switch(type){
      case TID_FLOAT:
	//READ_VALUES sortes the values from motor00_values and motor01_values in the values array 
	READ_VALUES(float);
	break;
      case TID_BOOL:
	READ_VALUES(BOOL);
	break;
      case TID_INT:
	READ_VALUES(int);
	break;
      }
    }
  else{

    // Write data to ODB
    db_set_data(pInfo->hDB,hKey[0],motor00_values,buff_size,8,type);
    db_set_data(pInfo->hDB,hKey[1],motor01_values,buff_size,8,type);
	
  }
}


