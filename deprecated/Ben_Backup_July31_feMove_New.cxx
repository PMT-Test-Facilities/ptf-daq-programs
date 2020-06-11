/********************************************************************	\
 
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

/* make frontend functions callable from the C framework            */
#ifdef __cplusplus
extern "C" {
#endif

  /*-- Globals -------------------------------------------------------*/

  /* The generate_path return flags                                   */
#define GENPATH_SUCCESS  0
#define GENPATH_SWAP     1
#define GENPATH_BAD_DEST 2
#define GENPATH_NO_PATH  3

  /* The Abort Codes used by stop_move																*/
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
  //INT display_period = 3000;
  INT display_period = 0;

  /* maximum event size produced by this frontend                     */
  INT max_event_size = 3000;

  /* buffer size to hold events                                       */
  INT event_buffer_size = 10*3000;

  /* maximum event size for fragmented events (EQ_FRAGMENTED)         */
  INT max_event_size_frag = 5*300*300;

  INT monitor_counter = 0;

  /*-- Info structure declaration ------------------------------------*/

  typedef struct {
    // Handles for all ODB variables
    HNDLE hDB;
    HNDLE hKeyDest,hKeyStart,hKeyStop,hKeyReInit; // "Control"
    HNDLE hKeyVeloc,hKeyAccel,hKeyScale,hKeyChan,hKeyLimPos; // "Settings"
    HNDLE hKeyPos,hKeyInit,hKeySwap,hKeyBadDest, hKeyCompleted,hKeyMoving, hKeyAxMoving,hKeyAxLimit,hKeyInitializing; // "Variables"
    HNDLE hKeyMDest[2],hKeyMStart[2],hKeyMStop[2],hKeyMMoving[2],hKeyMPos[2],hKeyMLimit[2],hKeyMLimitPos[2],hKeyMVel[2],hKeyMAcc[2]; // Motors

    HNDLE hKeyPhidgetTilt;

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
    float *LimPos;				// Coordinates of the limit switch on each axis (physical units)
  
    // "Variables" ODB variables
    float *Position;      // Axis positions (physical units)
    BOOL Initialized;     // Is system initialized (y/n)
    BOOL Swapped;					// Have the axis destinations been swapped (y/n)
    BOOL Completed;       // Have all requested moves completed without error (y/n)
    BOOL Moving;          // Are any of the axes moving (y/n)
    BOOL *AxisMoving;     // Is each axis moving (y/n)
    BOOL *AxisLimit;      // Is each axis' limit switch being triggered (y/n)
    BOOL BadDest;         // Did you set a Bad Destination?

    // Local variables
    float *CountPos;      // Axis positions (counts)
    float *CountDest;			// Relative axis destinations (counts)
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

  pInfo->Destination   = (float *)  calloc(10,sizeof(float));
  
  pInfo->Velocity       = (float *)  calloc(10,sizeof(float)); 
  pInfo->Acceleration   = (float *)  calloc(10,sizeof(float)); 
  pInfo->mScale         = (float *)  calloc(10,sizeof(float)); 
  pInfo->Channels       = (INT *)  calloc(10,sizeof(float)); 
  pInfo->LimPos		= (float *)  calloc(10,sizeof(float));
 
  pInfo->Position       = (float *)  calloc(10,sizeof(float));
  pInfo->AxisMoving     = (BOOL *)   calloc(10,sizeof(BOOL));
  pInfo->AxisLimit      = (BOOL *)   calloc(10,sizeof(BOOL));

  pInfo->CountPos       = (float *)  calloc(10,sizeof(float));
  pInfo->CountDest      = (float *)  calloc(10,sizeof(float));
  pInfo->mOrigin        = (float *)  calloc(10,sizeof(float));
  pInfo->MovePath       = (float **) calloc(10,sizeof(float*));

  //pInfo->Phidget        = (double *)  calloc(9,sizeof(double)); 

  /* Get database handle */ 
  cm_get_experiment_database(&hDB,NULL);
  pInfo->hDB = hDB;

  /* Initialize non-ODB variables */
  pInfo->PathSize = 0;
  pInfo->PathIndex = 0;
  pInfo->AbortCode = AC_USER_INPUT;

  /* Initialize "Control" ODB variables */
  // "Destination"
  db_find_key(hDB,0,"/Equipment/Move/Control/Destination",&pInfo->hKeyDest);
  // Don't reset destination when initializing. TL
  // db_set_data(hDB,pInfo->hKeyDest,pInfo->Destination,10*sizeof(float),10,TID_FLOAT);

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
  
  // "Swapped"
  db_find_key(hDB,0,"/Equipment/Move/Variables/Swapped",&pInfo->hKeySwap);
  pInfo->Swapped = 0;
  db_set_data(hDB,pInfo->hKeySwap,&pInfo->Swapped,sizeof(BOOL),1,TID_BOOL);  

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

  // "Axis Limit"  
  db_find_key(hDB,0,"/Equipment/Move/Variables/Axis Limit",&pInfo->hKeyAxLimit);
  db_set_data(hDB,pInfo->hKeyAxLimit,pInfo->AxisLimit,10*sizeof(BOOL),10,TID_BOOL);

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
  db_find_key(hDB,0,"/Equipment/Motors00/Variables/Limit Neg",&pInfo->hKeyMLimit[0]);
  db_find_key(hDB,0,"/Equipment/Motors01/Variables/Limit Neg",&pInfo->hKeyMLimit[1]);
  
  // "Velocity"
  db_find_key(hDB,0,"/Equipment/Motors00/Settings/Velocity",&pInfo->hKeyMVel[0]);
  db_find_key(hDB,0,"/Equipment/Motors01/Settings/Velocity",&pInfo->hKeyMVel[1]);
  
  // "Acceleration"
  db_find_key(hDB,0,"/Equipment/Motors00/Settings/Acceleration",&pInfo->hKeyMAcc[0]);
  db_find_key(hDB,0,"/Equipment/Motors01/Settings/Acceleration",&pInfo->hKeyMAcc[1]);

  // "Accelerometer tilt"
  int stat = db_find_key(hDB,0,"/Equipment/Phidget00/Variables",&pInfo->hKeyPhidgetTilt);
  if(stat != DB_SUCCESS){
    printf("Didn't get key\n");
  }

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
  for(i=0;i<10;i++){
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
  //int i;
  
  // Check that the user has set "Start Move" to "y"
  int size = sizeof(BOOL);
  db_get_data(hDB,pInfo->hKeyStart,&pInfo->Start,&size,TID_BOOL);
  if(pInfo->Start == 0) return;
  
  // If motors are already moving, set destinations back to previous
  // destination values and return with an error.
  if(pInfo->Moving){
    printf("Error: Can't start move. Move already in progress.\n");
    cm_msg(MERROR, "move_init", "Error: Can't start move. Move already in progress.\n");    //for(i=0;i<10;i++) pInfo->Destination[i] = pInfo->MovePath[i][pInfo->PathSize-1];
    //db_set_data(hDB,pInfo->hKeyDest,pInfo->Destination,10*sizeof(float),10,TID_FLOAT);
    return;
  }
	
  // Check if the motors have been initialized. If not, initialize them
  // before proceeding.
  db_get_data(hDB,pInfo->hKeyInit,&pInfo->Initialized,&size,TID_BOOL);
  if(pInfo->Initialized == 0) initialize(pInfo);
  // If initialization fails, return with error
  if(pInfo->Initialized == 0){
    printf("Error: Can't start move. Initialization failed.\n");
    cm_msg(MERROR, "move_init", "Error: Can't start move. Initialization failed.\n");    return;
  }

  // Load input destination into pInfo->Destination
  size = 10*sizeof(float);
  db_get_data(hDB,pInfo->hKeyDest,pInfo->Destination,&size,TID_FLOAT);
  
  // Generate collision free path to pInfo->Destination
  printf("Generating Path\n");
  int Status = generate_path(pInfo);
 
  // Check for unsuccesful path generation
  switch(Status){
  case GENPATH_SUCCESS:
    pInfo->Swapped = 0;
    db_set_data(hDB,pInfo->hKeySwap,&pInfo->Swapped,sizeof(BOOL),1,TID_BOOL);
    pInfo->BadDest = 0;
    db_set_data(hDB,pInfo->hKeyBadDest,&pInfo->BadDest,sizeof(BOOL),1,TID_BOOL);
    break;
  case GENPATH_SWAP:
    printf("generate_path() Warning: Axis destinations swapped in path generation\n");
    pInfo->Swapped = 1;
    db_set_data(hDB,pInfo->hKeySwap,&pInfo->Swapped,sizeof(BOOL),1,TID_BOOL);
    break;
  case GENPATH_BAD_DEST:
    cm_msg(MERROR, "move_init", " Bad destination entered");
    pInfo->BadDest = 1;
    db_set_data(hDB,pInfo->hKeyBadDest,&pInfo->BadDest,sizeof(BOOL),1,TID_BOOL);
    return;
  case GENPATH_NO_PATH:
    printf("generate_path() Error: No path to destination found");
    return;
  }
  printf("Path succesfully generated\n");

  if(Status != GENPATH_SUCCESS){
    if(Status == GENPATH_BAD_DEST) printf("generate_path() Error: Bad destination entered in Move");
    else printf("generate_path() Error: No path found to destination in Move");
    return;
  }
  printf("Path Generated\n");  
 
  // Set the "completed" variable in the ODB to 0 (since our newly
  // started move is still incomplete.
  pInfo->Completed = 0;
  db_set_data(hDB,pInfo->hKeyCompleted,&pInfo->Completed,sizeof(BOOL),1,TID_BOOL);
   
  // Move to first path index
  std::cout<<"Move_init called move()" <<std::endl;
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
  for(i = 0; i < 1 /*2!*/; i++){
    int axis = 4;
    if(i==1) axis = 9;

    // Need to add a check that phidget is actually active.  Test that it works!!!
    int size = sizeof(pInfo->Phidget);
    int status = db_get_value(pInfo->hDB,pInfo->hKeyPhidgetTilt,"PH00",&pInfo->Phidget,&size,TID_DOUBLE,FALSE);
    if(status != DB_SUCCESS)
      {
	cm_msg(MERROR, "initialize_tilt", "cannot get value for Phidghet Reading\n");
	return DB_NO_ACCESS;
      }
    int time_s = pInfo->Phidget[8];
    int time_us = pInfo->Phidget[9];
    
    sleep(2);
    db_get_value(pInfo->hDB,pInfo->hKeyPhidgetTilt,"PH00",&pInfo->Phidget,&size,TID_DOUBLE,FALSE);

    int ntime_s = pInfo->Phidget[8];
    int ntime_us = pInfo->Phidget[9];
    if(time_s == ntime_s && time_us == ntime_us){

      printf("Time not updated; fePhidget not running; can't initialize\n");
      printf("%i %i %i %i\n",time_s,ntime_s,time_us,ntime_us);
      return 0;
    }

    printf("\nTilt initial position: tilt angle = %f\n",pInfo->Phidget[7]);

    // Now we estimate the distance we need in order to get the tilt to zero.
    float dest[10] = {0,0,0,0,0,0,0,0,0,0};
    dest[axis] = -(pInfo->Phidget[7]*pInfo->mScale[axis]);
    
    channel_rw(pInfo,pInfo->hKeyMDest,dest,TID_FLOAT,1);

    // Start the move.
    BOOL start[10] = {1,1,1,1,1,1,1,1,1,1};
    
    channel_rw(pInfo,pInfo->hKeyMStart,(void *)start,TID_BOOL,1);
    sleep(600); //TF and BK: is this to wait for galil_read to update ODB for AxisMoving??
                // Note: Might be needed on at the end of move as well...
    
    // Monitor the motion until we are finished moving.
    pInfo->Moving = 1;
    while(pInfo->Moving){      
      pInfo->Moving = 0;
      
      channel_rw(pInfo,pInfo->hKeyMMoving,(void *)pInfo->AxisMoving,TID_BOOL,0);
      for(i=0;i<10;i++) pInfo->Moving = pInfo->Moving || pInfo->AxisMoving[i];
      db_get_value(pInfo->hDB,pInfo->hKeyPhidgetTilt,"PH00",&pInfo->Phidget,&size,TID_DOUBLE,FALSE);
      sleep(600);
    }

 
    // Check that the final tilt is within the required tolerance
    db_get_value(pInfo->hDB,pInfo->hKeyPhidgetTilt,"PH00",&pInfo->Phidget,&size,TID_DOUBLE,FALSE);
    double tolerance = 3.0;
    if(fabs(pInfo->Phidget[7]) < tolerance){
      printf("Move complete.  Tilt = %f\n",pInfo->Phidget[7]);
    }else{
      printf("Tilt initialization (axis %i) failed; final tilt = %f is not within zero to within required tolerance of %f degrees\n",
	     axis,pInfo->Phidget[7],tolerance);
      cm_msg(MERROR, "initialize_tilt", 
	     "Tilt initialization (axis %i) failed; final tilt = %f is not within zero to within required tolerance of %f degrees\n",
	     axis,pInfo->Phidget[7],tolerance);
    }

    // Reset origin position.
    
    channel_rw(pInfo,pInfo->hKeyMPos,pInfo->CountPos,TID_FLOAT,0);
    pInfo->mOrigin[axis] = pInfo->CountPos[axis] - pInfo->LimPos[axis]*pInfo->mScale[axis];
    pInfo->Position[axis] = pInfo->LimPos[axis];
   
    printf("\nFinished initializing axis %i; Origin = %5.2f counts, Position = %5.2f m.",i,pInfo->mOrigin[axis], pInfo->Position[axis]);
        
  }

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
  

  printf("Initializing motors:");
  BOOL initing = 1;
  db_set_data(pInfo->hDB,pInfo->hKeyInitializing,&initing,sizeof(BOOL),1,TID_BOOL);

  // Set motors to move into their negative limit switches, if they are enabled 
  //(disabled is LimPos = 9999)
  // Motors 4 and 9 are the tilt motors.  These are initialized separately, using 
  // the tilt measurement from the phidget.
  printf("\n");
  for(i=0;i<10;i++){
    if(i == 4 || i == 9){
      tempPos[i] = 0;
      tempNegLimitEnabled[i] = 0;
      printf("Axis %i for tilt motor will be initialized separately.\n", i);
    }else if(pInfo->LimPos[i] == 9999){
      tempPos[i] = 0;
      tempNegLimitEnabled[i] = 0;
      printf("Negative limit switch for axis %i disabled. Axis will not be initialized.\n", i);
    }else{
      tempPos[i] = 500*fabs(pInfo->mScale[i]);
      // TF: TODO WHEN COUNTER WEIGHTS FOR GANTRY0 ARE REPLACED AND FINAL => UNDO THIS
      // Now moving to positive limit switch for z0 and z1
      if(i == 2 || i == 7)
	tempPos[i] *= -1;
      tempNegLimitEnabled[i] = 1;
      printf("Negative limit switch for axis %i enabled. Axis will be initialized; use position %f.\n", i,tempPos[i]);
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
      printf("Initializing axes %i and %i (enabled = %i %i)\n",i,i+5,tempNegLimitEnabled[i],tempNegLimitEnabled[i+5]);
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
      	channel_rw(pInfo,pInfo->hKeyMLimit,(void *)pInfo->AxisLimit,TID_BOOL,0);
	//TF: TODO UNDO WHEN COUNTER WEIGHTS FOR GANTRY0 ARE REPLACED (and changes Limit Position for Z in ODB to 0.54)
	if(i == 2 || i == 7)
	  channel_rw(pInfo,pInfo->hKeyMLimitPos,(void *)pInfo->AxisLimit,TID_BOOL,0);
        if(pInfo->AxisLimit[i] && pInfo->AxisLimit[i+5]) break;
	else{
	  channel_rw(pInfo,pInfo->hKeyMPos,pInfo->CountPos,TID_FLOAT,0);
	  if(pInfo->CountPos[i] == lastCountPosArm1 && !pInfo->AxisLimit[i]){
	    printf("\nAxis %i not moving!!! Stopping initialization since limit switch must be broken...", i);
	    channel_rw(pInfo,pInfo->hKeyMStop,(void *)tempStop,TID_BOOL,1);
	    exitFlag = 1;
	    break;
	  }
	  if(pInfo->CountPos[i+5] == lastCountPosArm2 && !pInfo->AxisLimit[i+5]){
	    printf("\nAxis %i not moving!!! Stopping initialization since limit switch must be broken...", i+5);
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
	
	channel_rw(pInfo,pInfo->hKeyMLimit,(void *)pInfo->AxisLimit,TID_BOOL,0);
        if(pInfo->AxisLimit[i+5]){
	  break;
	}else{
	  channel_rw(pInfo,pInfo->hKeyMPos,pInfo->CountPos,TID_FLOAT,0);
	  if(pInfo->CountPos[i+5] == lastCountPosArm2){
	    printf("\nAxis %i not moving!!! Stopping initialization since limit switch must be broken...", i+5);
	    channel_rw(pInfo,pInfo->hKeyMStop,(void *)tempStop,TID_BOOL,1);
	    exitFlag = 1;
	    break;
	  }
	}
      }
      else{// If only first gantry axis is enabled.
	channel_rw(pInfo,pInfo->hKeyMLimit,(void *)pInfo->AxisLimit,TID_BOOL,0);
        if(pInfo->AxisLimit[i]){
	  break;
	}else{
	  channel_rw(pInfo,pInfo->hKeyMPos,pInfo->CountPos,TID_FLOAT,0);
	  if(pInfo->CountPos[i] == lastCountPosArm1){
	    printf("\nAxis %i not moving!!! Stopping initialization since limit switch must be broken...", i);
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
    for(i=0;i<10;i++) pInfo->Moving = pInfo->Moving || pInfo->AxisMoving[i];
  }
  
  if(exitFlag == 0) {
    // Determine the motor positions at the origin and put the results in
    // pInfo->mOrigin
    channel_rw(pInfo,pInfo->hKeyMPos,pInfo->CountPos,TID_FLOAT,0);
    for(i=0;i<10;i++){
      if(tempNegLimitEnabled[i] == 1){
        // The motor origin should always be at the limit switches, since this is 
        // a local variable
        // MARK
	pInfo->mOrigin[i] = pInfo->CountPos[i];// - pInfo->LimPos[i]*pInfo->mScale[i];
	pInfo->Position[i] = pInfo->LimPos[i];
	printf("\nFinished initializing axis %i; Origin = %5.2f counts, Position = %5.2f m.",i,pInfo->mOrigin[i], pInfo->Position[i]);
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
  printf("\nInitialization complete\n\n");
  cm_msg(MINFO, "move_init", "Initialization of Gantries is Complete\n");    
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
//							GENPATH_SWAP: path generated, but the destinations needed to be swapped
//							GENPATH_BAD_DEST: impossible destination given as input
//							GENPATH_NO_PATH: no valid path to destination found

int generate_path(INFO *pInfo){

  // Check for illegal destinations.  Currently just check:
  // rot_min < rotary angle < rot_max
  double rot_min = -120, rot_max = 120;
  // tilt_min < tilt angle < tilt_max
  double tilt_min = -105, tilt_max = 15;

  bool move_second_gantry_first = false;

  // Create path calculator object
  TPathCalculator pathCalc;

  double gantry1XPos = pInfo->Position[0];
  double gantry1YPos = pInfo->Position[1];
  double gantry2XPos = pInfo->Position[5];
  double gantry2YPos = pInfo->Position[6];

  // Gantry XY corners in global coordinates, when at the gantries limit switches
  double gantry1XDimensions[4];
  double gantry1YDimensions[4];
  double gantry2XDimensions[4];
  double gantry2YDimensions[4];
  double containerXDimensions[4];
  double containerYDimensions[4];

  gantry1YDimensions[0] = -0.200;
  gantry1XDimensions[0] = -0.010;
  gantry1YDimensions[1] = 0.200;
  gantry1XDimensions[1] = -0.010;
  gantry1YDimensions[2] = 0.200;
  gantry1XDimensions[2] = 0.210;  //TF and BK: changed because the coordinate system is based on the gantry arm where the optical box is attached to.
  gantry1YDimensions[3] = -0.200;
  gantry1XDimensions[3] = 0.210;

  gantry2YDimensions[0] = -0.200;
  gantry2XDimensions[0] = -0.210;
  gantry2YDimensions[1] = 0.200;
  gantry2XDimensions[1] = -0.210;
  gantry2YDimensions[2] = 0.200;
  gantry2XDimensions[2] = 0.010;
  gantry2YDimensions[3] = -0.200;
  gantry2XDimensions[3] = 0.010;

  containerYDimensions[0] = -0.205;
  containerXDimensions[0] = -0.05;
  containerYDimensions[1] = 0.735;
  containerXDimensions[1] = -0.05;
  containerYDimensions[2] = 0.735;
  containerXDimensions[2] = 1.05;
  containerYDimensions[3] = -0.205;
  containerXDimensions[3] = 1.05;

  std::vector<std::pair<double, double> > gantry1;
  std::vector<std::pair<double, double> > gantry2;
  std::vector<std::pair<double, double> > container;

  for (int i = 0; i < 4; ++i){
    gantry1.push_back(std::make_pair(gantry1XPos+gantry1XDimensions[i], gantry1YPos + gantry1YDimensions[i]));
    gantry2.push_back(std::make_pair(gantry2XPos+gantry2XDimensions[i], gantry2YPos + gantry2YDimensions[i]));
    container.push_back(std::make_pair(containerXDimensions[i], containerYDimensions[i]));
  }

  // Initialise gantries and water tank in path calculator
  pathCalc.InitialiseGantries(gantry1, gantry2);
  pathCalc.InitialiseContainer(container);

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
  // BK: I changed these values so that the largest area possible can be reached by both gantries
  if(pInfo->Destination[0] + 0.05 >= gantry2XPos){
    cm_msg(MERROR, "generate_path", "Illegal value for X destination of gantry 1 - it will collide with gantry 2");
    return GENPATH_BAD_DEST;
  }
  if(pInfo->Destination[5] - 0.05 <= gantry1XPos){
    cm_msg(MERROR, "generate_path", "Illegal value for X destination of gantry 2 - it will collide with gantry 1");
    return GENPATH_BAD_DEST;
  }

  std::pair<double, double> start1 = std::make_pair(gantry1XPos, gantry1YPos);
  std::pair<double, double> end1 = std::make_pair(pInfo->Destination[0], pInfo->Destination[1]);
  std::vector<std::pair<double, double> > path1;

  bool goodPath1 = pathCalc.CalculatePath(start1, end1, path1);

  std::pair<double, double> start2 = std::make_pair(gantry2XPos, gantry2YPos);
  std::pair<double, double> end2 = std::make_pair(pInfo->Destination[5], pInfo->Destination[6]);
  std::vector<std::pair<double, double> > path2;

  // Re-initialize gantry positions taking into account movement of first gantry
  gantry1.clear();
  for (int i = 0; i < 4; ++i){
    gantry1.push_back(std::make_pair(pInfo->Destination[0] + gantry1XDimensions[i], pInfo->Destination[1] + gantry1YDimensions[i]));
  }
  pathCalc.InitialiseGantries(gantry1, gantry2);

  bool goodPath2 = pathCalc.CalculatePath(start2, end2, path2);

  if (!goodPath1 && !goodPath2){
    cm_msg(MERROR, "generate_path", "Not goodPath1 or goodPath2 %i %i",goodPath1,goodPath2);
    return GENPATH_BAD_DEST;
  }

  // If only one movement failed try moving the second gantry first
  if ((goodPath1 && !goodPath2) || (!goodPath1 && goodPath2)) {
    // Re-initialise gantry 1 at its current position
    gantry1.clear();
    for (int i = 0; i < 4; ++i){
      gantry1.push_back(std::make_pair(gantry1XPos+gantry1XDimensions[i], gantry1YPos + gantry1YDimensions[i]));
    }
    pathCalc.InitialiseGantries(gantry1, gantry2);
 
    goodPath2 = pathCalc.CalculatePath(start2, end2, path2);

    // If moving the second gantry first doesn't work, give up
    if (!goodPath2){
      cm_msg(MERROR, "generate_path", "Not goodPath2");
	
      return GENPATH_BAD_DEST;
    }

    // Re-initialise gantry 2 to its destination
    gantry2.clear();
    for (int i = 0; i < 4; ++i){
      gantry2.push_back(std::make_pair(pInfo->Destination[5] + gantry2XDimensions[i], pInfo->Destination[6] + gantry2YDimensions[i]));
    }
    pathCalc.InitialiseGantries(gantry1, gantry2);

    goodPath1 = pathCalc.CalculatePath(start1, end1, path1);

    // If moving the second gantry first doesn't work, give up
    if (!goodPath1){
      cm_msg(MERROR, "generate_path", "Not goodPath1");
      return GENPATH_BAD_DEST;
    }

    move_second_gantry_first = true;
   
  }


  //Initialise path to current position
  for(int i = 0; i < 10; ++i){
    pInfo->MovePath[i] = (float *) calloc(path1.size() + path2.size() +2,sizeof(float));
    for(int j = 0; j < path1.size() + path2.size() + 2; ++j){
      pInfo->MovePath[i][j] = pInfo->CountPos[i];
    }
  }

  if(!move_second_gantry_first){
    // Put in the steps to move gantry 1 in X and Y
    for(int i = 0; i < 10; ++i){
      if(pInfo->Channels[i] != -1){
	for(int j = 0; j < path1.size(); ++j){
	  // Move in X or Y according to determined path for first gantry
	  if(i == 0) pInfo->MovePath[i][j + 1] = round(pInfo->MovePath[i][j] + path1[j].first*pInfo->mScale[i]);
	  else if (i == 1) pInfo->MovePath[i][j + 1] = round(pInfo->MovePath[i][j] + path1[j].second*pInfo->mScale[i]);
	  else pInfo->MovePath[i][j + 1] = pInfo->MovePath[i][j];
	}
      }
    }
    
    // Put in the steps to move gantry 2 in X and Y
    for(int i = 0; i < 10; ++i){
      if(pInfo->Channels[i] != -1){
	for(int j = 0; j < path2.size(); ++j){
	  // Move in X or Y according to determined path for first gantry
	  if(i == 5) pInfo->MovePath[i][j + 1 + path1.size()] = round(pInfo->MovePath[i][j + path1.size()] + path2[j].first*pInfo->mScale[i]);
	  else if (i == 6) pInfo->MovePath[i][j + 1 + path1.size()] = round(pInfo->MovePath[i][j + path1.size()] + path2[j].second*pInfo->mScale[i]);
	  else pInfo->MovePath[i][j + 1 + path1.size()] = pInfo->MovePath[i][j + path1.size()];
	}
      }
    }
  }
  else{
    // Put in the steps to move gantry 2 in X and Y
    for(int i = 0; i < 10; ++i){
      if(pInfo->Channels[i] != -1){
	for(int j = 0; j < path2.size(); ++j){
	  // Move in X or Y according to determined path for first gantry
	  if(i == 5) pInfo->MovePath[i][j + 1] = round(pInfo->MovePath[i][j] + path2[j].first*pInfo->mScale[i]);
	  else if (i == 6) pInfo->MovePath[i][j + 1] = round(pInfo->MovePath[i][j] + path2[j].second*pInfo->mScale[i]);
	  else pInfo->MovePath[i][j + 1] = pInfo->MovePath[i][j];
	}
      }
    }
    // Put in the steps to move gantry 1 in X and Y
    for(int i = 0; i < 10; ++i){
      if(pInfo->Channels[i] != -1){
	for(int j = 0; j < path1.size() + 1; ++j){
	  // Move in X or Y according to determined path for first gantry
	  if(i == 0) pInfo->MovePath[i][j + 1 + path2.size()] = round(pInfo->MovePath[i][j + path2.size()] + path1[j].first*pInfo->mScale[i]);
	  else if (i == 1) pInfo->MovePath[i][j + 1 + path2.size()] = round(pInfo->MovePath[i][j + path2.size()] + path1[j].second*pInfo->mScale[i]);
	  else pInfo->MovePath[i][j + 1 + path2.size()] = pInfo->MovePath[i][j + path2.size()];
	}
      }
    }
    
  }

  // Retract gantry heads at start
  // TF: only if X and Y are also different
  // TF: Don't do this anymore, used to be extra safety.
  /*
    if(fabs(pInfo->Destination[0]-gantry1XPos) > 0.01 || fabs(pInfo->Destination[1]-gantry1YPos) > 0.01)
    pInfo->MovePath[2][0] = pInfo->mOrigin[2];
    if(fabs(pInfo->Destination[5]-gantry2XPos) > 0.01 || fabs(pInfo->Destination[6]-gantry2YPos) > 0.01)
    pInfo->MovePath[7][0] = pInfo->mOrigin[7];
  */

  // Keep X and Y coordinates of gantries at end of move
  pInfo->MovePath[0][path1.size() + path2.size() + 1] = pInfo->MovePath[0][path1.size() + path2.size()];
  pInfo->MovePath[1][path1.size() + path2.size() + 1] = pInfo->MovePath[1][path1.size() + path2.size()];
  pInfo->MovePath[5][path1.size() + path2.size() + 1] = pInfo->MovePath[5][path1.size() + path2.size()];
  pInfo->MovePath[6][path1.size() + path2.size() + 1] = pInfo->MovePath[6][path1.size() + path2.size()];

  // Lower gantry heads at end and rotate
  pInfo->MovePath[2][path1.size() + path2.size() + 1] = round((pInfo->Destination[2]-pInfo->LimPos[2])*pInfo->mScale[2] + pInfo->mOrigin[2]);
  pInfo->MovePath[3][path1.size() + path2.size() + 1] = round((pInfo->Destination[3]-pInfo->LimPos[3])*pInfo->mScale[3] + pInfo->mOrigin[3]);
  pInfo->MovePath[4][path1.size() + path2.size() + 1] = round((pInfo->Destination[4]-pInfo->LimPos[4])*pInfo->mScale[4] + pInfo->mOrigin[4]);
  pInfo->MovePath[7][path1.size() + path2.size() + 1] = round((pInfo->Destination[7]-pInfo->LimPos[7])*pInfo->mScale[7] + pInfo->mOrigin[7]);
  pInfo->MovePath[8][path1.size() + path2.size() + 1] = round((pInfo->Destination[8]-pInfo->LimPos[8])*pInfo->mScale[8] + pInfo->mOrigin[8]);
  pInfo->MovePath[9][path1.size() + path2.size() + 1] = round((pInfo->Destination[9]-pInfo->LimPos[9])*pInfo->mScale[9] + pInfo->mOrigin[9]);

  for (int i = 0; i < path1.size() + path2.size() + 2; ++i){
    for(int j = 0; j < 10; ++j){
      printf("Destination[%i][%i] = %6.2f (Count Destination = %6.0f, Remaining =  %6.0f)\n",j,i,pInfo->Destination[j],pInfo->MovePath[j][i],pInfo->MovePath[j][i]-pInfo->CountPos[j]);
    }
  }

  pInfo->PathSize = path1.size() + path2.size() + 2;
  pInfo->PathIndex = 0;

  return GENPATH_SUCCESS;
}

/*-- Move ----------------------------------------------------------*/
// Use channel_rw to send the subsequent path index to the motors
void move(INFO *pInfo) {
  int i;
  BOOL zerotest = 0;
  BOOL start[10] = {1,1,1,1,1,1,1,1,1,1};
  BOOL Motor00Move[8];
  BOOL Motor01Move[8];
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
  int size1 = sizeof(Motor00Move);
  int size2 = sizeof(Motor01Pos);
  int size3 = sizeof(Motor00LimitPos);
  int waiting = 1; 
  BOOL started_moving = 0;
  DWORD Overall_time_for_loop;
  DWORD start_of_loop; 
  
  printf("Moving to path index: %i\n",pInfo->PathIndex);

  // Read in axis positions (in counts)
  channel_rw(pInfo,pInfo->hKeyMPos,(void *)pInfo->CountPos,TID_FLOAT,0);

  // Determine required destinations to be sent to the motors
  for(i=0;i<10;i++){
    pInfo->CountDest[i] = pInfo->MovePath[i][pInfo->PathIndex] - pInfo->CountPos[i];
    printf("MD[%i]=%6.0f ",i,pInfo->CountDest[i]);
    if(i==4) printf("\n");
    zerotest = zerotest || pInfo->CountDest[i];
  }
  printf("\n");
  // Write motor destinations to the ODB
  channel_rw(pInfo,pInfo->hKeyMDest,(void *)pInfo->CountDest,TID_FLOAT,1);
	
  // This is added so that the monitor recognizes a move as completed, even
  // when no move is required.
  if(!zerotest){
    printf("Warning: No move required\n");
    // This indicates to the monitor that a move has been initiated
    // even though the motors won't start moving.
    pInfo->Moving = 1;
    return;
  }
	
 // Start motors towards the specified destinations

  

  // Get the current location of the motor. This will be used to tell if a motor has started moving yet
  db_get_data(pInfo->hDB,pInfo->hKeyMPos[0],&Motor00StartPos,&size2,TID_FLOAT);
  db_get_data(pInfo->hDB,pInfo->hKeyMPos[1],&Motor01StartPos,&size2,TID_FLOAT); 
  db_get_data(pInfo->hDB,pInfo->hKeyMDest[0],&Motor00Dest,&size2,TID_FLOAT);
  db_get_data(pInfo->hDB,pInfo->hKeyMDest[1],&Motor01Dest,&size2,TID_FLOAT);

// Get the time at the beggining of the loop so that we can track how long we are in the loop
  Overall_time_for_loop = ss_millitime();

// Don't do anything else until the motors have started moving
  while(!started_moving) { 
    start_of_loop = ss_millitime();
    std::cout<< "channel_rw called at time : "<<ss_millitime()<<std::endl;
    // Set the ODB values to start a move
    channel_rw(pInfo,pInfo->hKeyMStart,(void *)start,TID_BOOL,1);
      
    
    //    db_get_data(pInfo->hDB,pInfo->hKeyMStart[0],&Motor00Move,&size1,TID_BOOL);
    //  db_get_data(pInfo->hDB,pInfo->hKeyMStart[1],&Motor01Move,&size1,TID_BOOL);

   
    waiting = 1;
    while(waiting) {
      db_get_data(pInfo->hDB,pInfo->hKeyMPos[0],&Motor00Pos,&size2,TID_FLOAT);
      db_get_data(pInfo->hDB,pInfo->hKeyMPos[1],&Motor01Pos,&size2,TID_FLOAT);
      db_get_data(pInfo->hDB,pInfo->hKeyMLimitPos[0],&Motor00LimitPos,&size3,TID_BOOL);
      db_get_data(pInfo->hDB,pInfo->hKeyMLimit[0],&Motor00LimitNeg,&size3,TID_BOOL);
      db_get_data(pInfo->hDB,pInfo->hKeyMLimitPos[1],&Motor01LimitPos,&size3,TID_BOOL);
      db_get_data(pInfo->hDB,pInfo->hKeyMLimit[1],&Motor01LimitNeg,&size3,TID_BOOL);
      
      // if 300 seconds(5 minutes) has passed and nothing has happened exit becasue the program is not working
      if (ss_millitime()- Overall_time_for_loop > 1000*300){
      std::cout<<"The Motors never started Moving"<<std::endl;
      waiting = 0;
      started_moving = 1;
      }
      // If any any of the motors for gantry0 are no longer at there start position that means the motors are mvoing and the program behaved properly
      else if (Motor00Pos[0]!=Motor00StartPos[0] || Motor00Pos[1]!=Motor00StartPos[1] || Motor00Pos[2]!=Motor00StartPos[2] || Motor00Pos[3]!=Motor00StartPos[3] || Motor00Pos[4]!=Motor00StartPos[4] || Motor00Pos[5]!=Motor00StartPos[5] || Motor00Pos[6]!=Motor00StartPos[6] || Motor00Pos[7]!=Motor00StartPos[7]){
	std::cout<<"The Motors are Moving"<<std::endl;
	waiting = 0;
	started_moving = 1;
      }
      // If any any of the motors for gantry1 are no longer at there start position that means the motors are mvoing and the program behaved properly      
      else if (Motor01Pos[0]!=Motor01StartPos[0] || Motor01Pos[1]!=Motor01StartPos[1] || Motor01Pos[2]!=Motor01StartPos[2] || Motor01Pos[3]!=Motor01StartPos[3] || Motor01Pos[4]!=Motor01StartPos[4] || Motor01Pos[5]!=Motor01StartPos[5] || Motor01Pos[6]!=Motor01StartPos[6] || Motor01Pos[7]!=Motor01StartPos[7]){
	std::cout<<"The Motors are Moving"<<std::endl;
	waiting = 0;
	started_moving = 1;
      } 
      // If a motor for gantry0 is trying to move in the positive direction but the corresponding negative limit switch is engaged, a move will not be started however no move is need so set pinfo->moving to 1 so that the next move will be called
      else if( ((Motor00Dest[4]>0)&& Motor00LimitNeg[4]) ||((Motor00Dest[5]>0)&& Motor00LimitNeg[5]) || ((Motor00Dest[6]>0)&& Motor00LimitNeg[6]) || ((Motor00Dest[7]>0)&& Motor00LimitNeg[7]) ){
	std::cout<<"Move could not be started because Motor00 is at a negative limit switch"<<std::endl;
	waiting = 0;
	started_moving = 1;
	pInfo->Moving = 1;	
      }

      // If a motor for gantry0 is trying to move in the negative direction but the corresponding positive limit switch is engaged, a move will not be started however no move is need so set pinfo->moving to 1 so that the next move will be called      
      else if( ((Motor00Dest[4]<0)&& Motor00LimitPos[4]) ||((Motor00Dest[5]<0)&& Motor00LimitPos[5]) || ((Motor00Dest[6]<0)&& Motor00LimitPos[6]) || ((Motor00Dest[7]<0)&& Motor00LimitPos[7]) ){
	std::cout<<"Move could not be started because Motor00 is at a Positive limit switch"<<std::endl;
	waiting = 0;
	started_moving = 1;
	pInfo->Moving = 1;
      }

      // If a motor for gantry1 is trying to move in the positive direction but the corresponding negative limit switch is engaged, a move will not be started however no move is need so set pinfo->moving to 1 so that the next move will be called
      else if( ((Motor01Dest[1]>0)&& Motor01LimitNeg[1]) ||((Motor01Dest[2]>0)&& Motor01LimitNeg[2]) || ((Motor01Dest[3]>0)&& Motor01LimitNeg[3]) || ((Motor01Dest[4]>0)&& Motor01LimitNeg[4]) ){
	std::cout<<"Move could not be started because Motor01 is at a negative limit switch"<<std::endl;
	waiting = 0;
	started_moving = 1;
	pInfo->Moving = 1;	
      }

      // If a motor for gantry1 is trying to move in the negative direction but the corresponding positive limit switch is engaged, a move will not be started however no move is need so set pinfo->moving to 1 so that the next move will be called      
      else if( ((Motor01Dest[1]<0)&& Motor01LimitPos[1]) ||((Motor01Dest[2]<0)&& Motor01LimitPos[2]) || ((Motor01Dest[3]<0)&& Motor01LimitPos[3]) || ((Motor01Dest[4]<0)&& Motor01LimitPos[4]) ){
	std::cout<<"Move could not be started because Motor01 is at a Positive limit switch"<<std::endl;
	waiting = 0;
	started_moving = 1;
	pInfo->Moving = 1;	
      }
      // If 60 seconds(1 minute) has passed and the motors haven't started moving reset the ODB values that should initiate a move with the hope that a move will start.
      else if(ss_millitime() - start_of_loop > 1000*60){
	std::cout<<"Calling channel_rw again"<< std::endl;
	waiting = 0;
      }
    }
  }
  
  std::cout<<"Function Move is complete"<<std::endl;
  
}

/*-- Stop_move -----------------------------------------------------*/
// Aborts a move, and prints the reason for stopping to screen
void stop_move(HNDLE hDB, HNDLE hKey, void *data){
  INFO *pInfo = (INFO *) data;
  
  BOOL stop[10] = {0,0,0,0,0,0,0,0,0,0};
  channel_rw(pInfo,pInfo->hKeyMStop,(void *)stop,TID_BOOL,1);
  
  printf("Error: Move aborted due to");
  
  switch(pInfo->AbortCode){
  case AC_USER_INPUT:
    printf("user input\n");
    break;
  case AC_COLLISION:
    printf("collision detected\n");
    break;
  case AC_TIMEOUT:
    printf("move timeout\n");
  }
  
  pInfo->AbortCode = AC_USER_INPUT; // reset the abort code to default (user input)
}

/*-- Monitor -------------------------------------------------------*/
// Called periodically with /Motors/Variables/Moving, this function
// checks for completion of a move, as well as for collisions 
// during a move.
void monitor(HNDLE hDB,HNDLE hKey,void *data){
  INFO *pInfo = (INFO *) data;
  int i = 0;
  BOOL OldMov = pInfo->Moving; //Store the old value of pInfo->Moving
  BOOL pos_AxisLimit[10];
  
  /*-- Update variables section of ODB -----------------------------*/
  
  /*TF and BK: check whether galil_move actually has been called, otherwise return!
    BOOL Motor00Move[8];
    BOOL Motor01Move[8];
    
    int size = sizeof(Motor00Move);
    db_get_data(pInfo->hDB,pInfo->hKeyMStart[0],&Motor00Move,&size,TID_BOOL);
    db_get_data(pInfo->hDB,pInfo->hKeyMStart[1],&Motor01Move,&size,TID_BOOL);
    if(Motor00Move[0] || Motor00Move[1] || Motor00Move[2] || Motor00Move[3] ||
    Motor00Move[4] || Motor00Move[5] || Motor00Move[6] || Motor00Move[7]){
    std::cout << "MOTOR 00 DID NOT MOVE YET" << std::endl;
    return;
    }
    if(Motor01Move[0] || Motor01Move[1] || Motor01Move[2] || Motor01Move[3] ||
    Motor01Move[4] || Motor01Move[5] || Motor01Move[6] || Motor01Move[7]){
    std::cout << "MOTOR 01 DID NOT MOVE YET" << std::endl;
    return;
    }
  */
 
  // Check if each axis is moving, and write this to ODB
  //monitor_counter++;
  //if(monitor_counter%500 == 0)
  //std::cout <<"Monitor has been called: " << monitor_counter << std::endl;
  channel_rw(pInfo,pInfo->hKeyMMoving,(void *)pInfo->AxisMoving,TID_BOOL,0);
  db_set_data(pInfo->hDB,pInfo->hKeyAxMoving,pInfo->AxisMoving,10*sizeof(float),10,TID_BOOL);	
  
  // Check if any of the axes are moving, and write this to ODB
  pInfo->Moving = 0;
  while ((!pInfo->Moving) && i<10){
    pInfo->Moving = pInfo->Moving || pInfo->AxisMoving[i];
    i++;
  }
  //debug
  /*if(pInfo->Moving)
  std::cout << "Aand we're moving" << std::endl;
  */
  db_set_data(pInfo->hDB,pInfo->hKeyMoving,&pInfo->Moving,sizeof(BOOL),1,TID_BOOL);
			
  // Get the axis positions and write to ODB
  channel_rw(pInfo,pInfo->hKeyMPos,(void *)pInfo->CountPos,TID_FLOAT,0);
  if(!pInfo->Completed){
    // Use transformed count positions if move in progress
    for(i=0;i<10;i++){
      pInfo->Position[i] = (pInfo->CountPos[i] - pInfo->mOrigin[i])/pInfo->mScale[i] + pInfo->LimPos[i] ;	  
    }
  }
  else{
    // Use destination values if the move is completed
    if(pInfo->Swapped)
      {
	// If the axes are swapped, make sure to write the correct destination
	// to the right axis
	for(i=0;i<5;i++){
	  pInfo->Position[i] = (pInfo->CountPos[i+5] - pInfo->mOrigin[i+5])/pInfo->mScale[i+5] + pInfo->LimPos[i+5];
	  pInfo->Position[i+5] = (pInfo->CountPos[i] - pInfo->mOrigin[i])/pInfo->mScale[i] + pInfo->LimPos[i];
	}
      }
    else{
      for(i=0;i<10;i++){
	pInfo->Position[i] = (pInfo->CountPos[i] - pInfo->mOrigin[i])/pInfo->mScale[i] + pInfo->LimPos[i];
      }
    }
  }
  db_set_data(pInfo->hDB,pInfo->hKeyPos,pInfo->Position,10*sizeof(float),10,TID_FLOAT);
	
  // Check for limit switches triggered and write to ODB
  channel_rw(pInfo,pInfo->hKeyMLimit,(void *)pInfo->AxisLimit,TID_BOOL,0);
  channel_rw(pInfo,pInfo->hKeyMLimitPos,(void *)pos_AxisLimit,TID_BOOL,0);
  db_set_data(pInfo->hDB,pInfo->hKeyAxLimit,pInfo->AxisLimit,10*sizeof(BOOL),10,TID_BOOL);
	
  /* -- Check for collision configurations if the motors are moving --*/
  //if(pInfo->Moving){
  // To be added later...
  //}
	
	
  /* - If motors have stopped moving, determine next course of action */
  bool stoppedDueToLimit = false;
  if(OldMov && !pInfo->Moving){ // i.e. If the motors just stopped moving
    // Check if the destination has been reached, otherwise return an error
    for(i=0;i<10;i++){
      if((pInfo->Channels[i] != -1) && (pInfo->MovePath[i][pInfo->PathIndex] != pInfo->CountPos[i])){
	//TF: Add Limit Switch check:
	//printf("TESTS: %i %d\n",i,pInfo->AxisLimit[i]);
	if(pInfo->AxisLimit[i]||pos_AxisLimit[i]){
	  stoppedDueToLimit = true;
	  printf("Stopped moving because LIMIT SWITCH reached!\n");
	  break;
	} else {
	  printf("i : %d\n",i);
	  printf("Error: %6.2f, %6.2f\n", pInfo->MovePath[i][pInfo->PathIndex], pInfo->CountPos[i]);
	  printf("Error: Move failed\n");
	  return;
	}
      }
    }
		
    // If we make it this far, we have reached our specified path index
    printf("Move to index complete\n");
    std::cout<< "Time to complete Move to index "<<ss_millitime()<<std::endl;		
    // Check if we are at the final path index, otherwise initiate next move
    // if((pInfo->PathIndex+1 == pInfo->PathSize) || (stoppedDueToLimit)){
 if(pInfo->PathIndex+1 == pInfo->PathSize) {
      // Final destination reached
      pInfo->Completed = 1;
      db_set_data(hDB,pInfo->hKeyCompleted,&pInfo->Completed,sizeof(BOOL),1,TID_BOOL);
      printf("Move to destination complete\n");
      if(stoppedDueToLimit)
	printf("But destination not reached due to limit switch\n");
    }
    else{
      pInfo->PathIndex++;
      std::cout<<"Monitor called move()" <<std::endl;
      std::cout<< "Monitor called move() at : "<<ss_millitime()<<std::endl;	
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
 
  int i;
  int size;
  int buff_size;
  void* motor00_values;
  void* motor01_values;

  // Determine the proper "size" variable for the r/w
  switch(type){
  case TID_FLOAT:
    float motor00_float_values[8];
    float motor01_float_values[8];
    buff_size=sizeof(motor00_float_values);
    size = sizeof(float);
    motor00_float_values[0]= 0;
    motor00_float_values[1]= 0;
    motor00_float_values[2]= 0;
    motor00_float_values[3]=*((float*)(values+4*size));   // Tilt
    motor00_float_values[4]=*((float*)(values+3*size));   // Rotary
    motor00_float_values[5]=*((float*)(values+0*size));   // X
    motor00_float_values[6]=*((float*)(values+1*size));   // Y
    motor00_float_values[7]=*((float*)(values+2*size));   // Z
    
    motor01_float_values[0]= 0;  
    motor01_float_values[1]=*((float*)(values+6*size));   // Y
    motor01_float_values[2]=*((float*)(values+7*size));   // Z
    motor01_float_values[3]=*((float*)(values+5*size));   // X
    motor01_float_values[4]=*((float*)(values+8*size));   // Rotary
    motor01_float_values[5]=*((float*)(values+9*size));   // Tilt
    motor01_float_values[6]= 0;
    motor01_float_values[7]= 0;  

    motor00_values = motor00_float_values;
    motor01_values = motor01_float_values;
    
    break;
  case TID_BOOL:
    BOOL motor00_BOOL_values[8];
    BOOL motor01_BOOL_values[8];
    buff_size=sizeof(motor00_BOOL_values);

    size = sizeof(BOOL);
     motor00_BOOL_values[0]= 0;
    motor00_BOOL_values[1]= 0;
    motor00_BOOL_values[2]= 0;
    motor00_BOOL_values[3]=*((BOOL*)(values+4*size));   // Tilt
    motor00_BOOL_values[4]=*((BOOL*)(values+3*size));   // Rotary
    motor00_BOOL_values[5]=*((BOOL*)(values+0*size));   // X
    motor00_BOOL_values[6]=*((BOOL*)(values+1*size));   // Y
    motor00_BOOL_values[7]=*((BOOL*)(values+2*size));   // Z
    
    motor01_BOOL_values[0]= 0;  
    motor01_BOOL_values[1]=*((BOOL*)(values+6*size));   // Y
    motor01_BOOL_values[2]=*((BOOL*)(values+7*size));   // Z
    motor01_BOOL_values[3]=*((BOOL*)(values+5*size));   // X
    motor01_BOOL_values[4]= *((BOOL*)(values+8*size));   // Rotary
    motor01_BOOL_values[5]= *((BOOL*)(values+9*size));   // Tilt
    motor01_BOOL_values[6]= 0;
    motor01_BOOL_values[7]= 0; 

    motor00_values = motor00_BOOL_values;
    motor01_values = motor01_BOOL_values;
    break;
  case TID_INT:
    int motor00_int_values[8];
    int motor01_int_values[8];
    buff_size=sizeof(motor00_int_values);

    size = sizeof(INT);
    motor00_int_values[0]= 0;
    motor00_int_values[1]= 0;
    motor00_int_values[2]= 0;
    motor00_int_values[3]=*((int*)(values+4*size));   // Tilt
    motor00_int_values[4]=*((int*)(values+3*size));   // Rotary
    motor00_int_values[5]=*((int*)(values+0*size));   // X
    motor00_int_values[6]=*((int*)(values+1*size));   // Y
    motor00_int_values[7]=*((int*)(values+2*size));   // Z
    
    motor01_int_values[0]= 0;  
    motor01_int_values[1]=*((int*)(values+6*size));   // Y
    motor01_int_values[2]=*((int*)(values+7*size));   // Z
    motor01_int_values[3]=*((int*)(values+5*size));   // X
    motor01_int_values[4]=*((int*)(values+8*size));   // Rotary
    motor01_int_values[5]=*((int*)(values+9*size));   // Tilt
    motor01_int_values[6]= 0;
    motor01_int_values[7]= 0;
    
    motor00_values = motor00_int_values;
    motor01_values = motor01_int_values;
  }
  
  if(rw ==0) {
    db_get_data(pInfo->hDB,hKey[0],motor00_values,&buff_size,type);
  	db_get_data(pInfo->hDB,hKey[1],motor01_values,&buff_size,type);
  	
  	  switch(type){
  case TID_FLOAT:
  	*((float*)(values + 0*size)) = *((float*)(motor00_values+5*size));
  	*((float*)(values + 1*size)) = *((float*)(motor00_values+6*size));
  	*((float*)(values + 2*size)) = *((float*)(motor00_values+7*size));
  	*((float*)(values + 3*size)) = *((float*)(motor00_values+4*size));
  	*((float*)(values + 4*size)) = *((float*)(motor00_values+3*size));
  	*((float*)(values + 5*size)) = *((float*)(motor01_values+3*size));
  	*((float*)(values + 6*size)) = *((float*)(motor01_values+1*size));
  	*((float*)(values + 7*size)) = *((float*)(motor01_values+2*size));
  	*((float*)(values + 8*size)) = *((float*)(motor01_values+4*size));
  	*((float*)(values + 9*size)) = *((float*)(motor01_values+5*size));

    break;
  case TID_BOOL:
  	*((BOOL*)(values + 0*size)) = *((BOOL*)(motor00_values+5*size));
  	*((BOOL*)(values + 1*size)) = *((BOOL*)(motor00_values+6*size));
  	*((BOOL*)(values + 2*size)) = *((BOOL*)(motor00_values+7*size));
  	*((BOOL*)(values + 3*size)) = *((BOOL*)(motor00_values+4*size));
  	*((BOOL*)(values + 4*size)) = *((BOOL*)(motor00_values+3*size));
  	*((BOOL*)(values + 5*size)) = *((BOOL*)(motor01_values+3*size));
  	*((BOOL*)(values + 6*size)) = *((BOOL*)(motor01_values+1*size));
  	*((BOOL*)(values + 7*size)) = *((BOOL*)(motor01_values+2*size));
  	*((BOOL*)(values + 8*size)) = *((BOOL*)(motor01_values+4*size));
  	*((BOOL*)(values + 9*size)) = *((BOOL*)(motor01_values+5*size));

    break;
  case TID_INT:
  	*((int*)(values + 0*size)) = *((int*)(motor00_values+5*size));
  	*((int*)(values + 1*size)) = *((int*)(motor00_values+6*size));
  	*((int*)(values + 2*size)) = *((int*)(motor00_values+7*size));
  	*((int*)(values + 3*size)) = *((int*)(motor00_values+4*size));
  	*((int*)(values + 4*size)) = *((int*)(motor00_values+3*size));
  	*((int*)(values + 5*size)) = *((int*)(motor01_values+3*size));
  	*((int*)(values + 6*size)) = *((int*)(motor01_values+1*size));
  	*((int*)(values + 7*size)) = *((int*)(motor01_values+2*size));
  	*((int*)(values + 8*size)) = *((int*)(motor01_values+4*size));
  	*((int*)(values + 9*size)) = *((int*)(motor01_values+5*size));

  }
  	
  	




    
    // Read data from ODB
    /*for(i=0;i<10;i++){
      if(pInfo->Channels[i]>=0 && pInfo->Channels[i]<16){
	// Use pointer arithmetic to get the proper index of values
	db_get_data_index(pInfo->hDB,hKey[(int)floor(pInfo->Channels[i]/8)],values + i*size,&size,pInfo->Channels[i]%8,type);
      }
    }*/
  }
  else{
    // Write data to ODB

  db_set_data(pInfo->hDB,hKey[0],motor00_values,buff_size,8,type);
	db_set_data(pInfo->hDB,hKey[1],motor01_values,buff_size,8,type);
	

	
  }
}



