/********************************************************************\
 
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

/* make frontend functions callable from the C framework            */
#ifdef __cplusplus
extern "C" {
#endif

/*-- Globals -------------------------------------------------------*/

/* The generate_path return flags                                   */
#define GENPATH_SUCCESS  0
#define GENPATH_SWAP 		 1
#define GENPATH_BAD_DEST 2
#define GENPATH_NO_PATH  3

/* The Abort Codes used by stop_move																*/
#define AC_USER_INPUT 0
#define AC_COLLISION 	1
#define AC_TIMEOUT 		2

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

/*-- Info structure declaration ------------------------------------*/

typedef struct {
  // Handles for all ODB variables
  HNDLE hDB;
  HNDLE hKeyDest,hKeyStart,hKeyStop,hKeyReInit; // "Control"
  HNDLE hKeyVeloc,hKeyAccel,hKeyScale,hKeyChan,hKeyLimPos; // "Settings"
  HNDLE hKeyPos,hKeyInit,hKeySwap,hKeyCompleted,hKeyMoving, hKeyAxMoving,hKeyAxLimit,hKeyInitializing; // "Variables"
  HNDLE hKeyMDest[2],hKeyMStart[2],hKeyMStop[2],hKeyMMoving[2],hKeyMPos[2],hKeyMLimit[2],hKeyMVel[2],hKeyMAcc[2]; // Motors

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
  float *LimPos;        // Coordinates of the limit switch on each axis (physical units)
  
  // "Variables" ODB variables
  float *Position;      // Axis positions (physical units)
  BOOL Initialized;     // Is system initialized (y/n)
  BOOL Swapped;					// Have the axis destinations been swapped (y/n)
  BOOL Completed;       // Have all requested moves completed without error (y/n)
  BOOL Moving;          // Are any of the axes moving (y/n)
  BOOL *AxisMoving;     // Is each axis moving (y/n)
  BOOL *AxisLimit;      // Is each axis' limit switch being triggered (y/n)
  
  // Local variables
  float *CountPos;      // Axis positions (counts)
  float *CountDest;	// Relative axis destinations (counts)
  float *mOrigin;       // Count location of motor limit switches //MARK// Count locations of axis origins (counts)
  float **MovePath;     // 2d array of axis positions for each waypoint of the path (counts)
  int PathSize;         // Number of waypoints in the path
  int PathIndex;        // Index of the path section currently in progress
  int AbortCode;        // Variable in which to store the reason for aborting a move (AC_*)

  // Phidget Tilt variables
  double Phidget[10];      // 
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
  db_open_record(hDB,pInfo->hKeyMMoving[1],NULL,8*sizeof(BOOL),MODE_READ,monitor,pInfo);
  db_open_record(hDB,pInfo->hKeyMMoving[2],NULL,8*sizeof(BOOL),MODE_READ,monitor,pInfo);
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

 reinitialize:	  This function calls initalize and then starts 
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
			break;
		case GENPATH_SWAP:
			printf("generate_path() Warning: Axis destinations swapped in path generation\n");
			pInfo->Swapped = 1;
			db_set_data(hDB,pInfo->hKeySwap,&pInfo->Swapped,sizeof(BOOL),1,TID_BOOL);
			break;
		case GENPATH_BAD_DEST:
			cm_msg(MERROR, "move_init", " Bad destination entered");
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
    sleep(600);
    
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
      // tempPos just needs to be a position beyond the limit switch
      // Replace pInfo->LimPos[i] with fabs 
      // The motor scalings then define the direction of the axis
      // MARK
      tempPos[i] = 500*fabs(pInfo->mScale[i]);
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
  
  if(exitFlag == 0){
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
  double tilt_min = -95, tilt_max = 15;

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
  
  

  // Rudimentary path generation routine
  int i;
  
  for(i=0;i<10;i++){
    pInfo->MovePath[i] = (float *) calloc(1,sizeof(float));

    // Need to change this as mOrigin is different for both motors
    // MARK    
    if(pInfo->Channels[i] != -1){
      // Used to be pInfo->Destination[i]*pInfo->mScale[i] + pInfo->mOrigin[i]
      // LimPos = position of limit switches in global coordinate system
      // mOrigin = position of limit switches in motor coordinate system
      // (pInfo->Destination[i] - pInfo->LimPos[i])*pInfo->mScale[i] = number of motor counts to move from limit switch position to destination
      //  + pInfo->mOrigin[i] 
      pInfo->MovePath[i][0] = round((pInfo->Destination[i] - pInfo->LimPos[i])*pInfo->mScale[i] + pInfo->mOrigin[i]);
    }
    else{
      pInfo->MovePath[i][0] = pInfo->CountPos[i];
    }
  
    printf("Destination[%i][%i] = %6.2f (Count Destination = %6.0f, Remaining =  %6.0f)\n",
	   i,0,pInfo->Destination[i],pInfo->MovePath[i][0],pInfo->MovePath[i][0]-pInfo->CountPos[i]);
  }
  
  pInfo->PathSize = 1;
  pInfo->PathIndex = 0;

  return GENPATH_SUCCESS;
}

/*-- Move ----------------------------------------------------------*/
// Use channel_rw to send the subsequent path index to the motors
void move(INFO *pInfo){
	int i;
	BOOL zerotest = 0;
	BOOL start[10] = {1,1,1,1,1,1,1,1,1,1};
	
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
	}
	
	// Start motors towards the specified destinations
	channel_rw(pInfo,pInfo->hKeyMStart,(void *)start,TID_BOOL,1);
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
	
	/*-- Update variables section of ODB -----------------------------*/
	 
	// Check if each axis is moving, and write this to ODB
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
	if(!pInfo->Completed){
		// Use transformed count positions if move in progress
		for(i=0;i<10;i++){
			pInfo->Position[i] = (pInfo->CountPos[i] - pInfo->mOrigin[i])/pInfo->mScale[i];	  
		}
	}
	else{
		// Use destination values if the move is completed
		if(pInfo->Swapped)
		{
			// If the axes are swapped, make sure to write the correct destination
			// to the right axis
			for(i=0;i<5;i++){
				pInfo->Position[i] = pInfo->Destination[i+5];
				pInfo->Position[i+5] = pInfo->Destination[i];
			}
		}
		else{
			for(i=0;i<10;i++){
				pInfo->Position[i] = pInfo->Destination[i];
			}
		}
	}
	db_set_data(pInfo->hDB,pInfo->hKeyPos,pInfo->Position,10*sizeof(float),10,TID_FLOAT);
	
	// Check for limit switches triggered and write to ODB
	channel_rw(pInfo,pInfo->hKeyMLimit,(void *)pInfo->AxisLimit,TID_BOOL,0);
	db_set_data(pInfo->hDB,pInfo->hKeyAxLimit,pInfo->AxisLimit,10*sizeof(BOOL),10,TID_BOOL);
	
	/* -- Check for collision configurations if the motors are moving --*/
	//if(pInfo->Moving){
		// To be added later...
	//}
	
	
	/* - If motors have stopped moving, determine next course of action */
	if(OldMov && !pInfo->Moving){ // i.e. If the motors just stopped moving
		// Check if the destination has been reached, otherwise return an error
		for(i=0;i<10;i++){
			if((pInfo->Channels[i] != -1) && (pInfo->MovePath[i][pInfo->PathIndex] != pInfo->CountPos[i])){
				printf("Error: Move failed\n");
				return;
			}
		}
		
		// If we make it this far, we have reached our specified path index
		printf("Move to index complete\n");
		
		// Check if we are at the final path index, otherwise initiate next move
		if(pInfo->PathIndex+1 == pInfo->PathSize){
			// Final destination reached
			pInfo->Completed = 1;
			db_set_data(hDB,pInfo->hKeyCompleted,&pInfo->Completed,sizeof(BOOL),1,TID_BOOL);
			printf("Move to destination complete\n");
		}
		else{
			pInfo->PathIndex++;
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
	
	// Determine the proper "size" variable for the r/w
	switch(type){
		case TID_FLOAT:
			size = sizeof(float);
			break;
		case TID_BOOL:
			size = sizeof(BOOL);
			break;
		case TID_INT:
			size = sizeof(INT);
	}
	
	if(rw ==0){
		// Read data from ODB
		for(i=0;i<10;i++){
			if(pInfo->Channels[i]>=0 && pInfo->Channels[i]<16){
				// Use pointer arithmetic to get the proper index of values
				db_get_data_index(pInfo->hDB,hKey[(int)floor(pInfo->Channels[i]/8)],values + i*size,&size,pInfo->Channels[i]%8,type);
			}
		}
	}
	else{
		// Write data to ODB
		for(i=0;i<10;i++){
		  //printf("channel_rw %i; ch = %i/%i\n",i,pInfo->Channels[i]%8,pInfo->Channels[i] );
			if(pInfo->Channels[i]>=0 && pInfo->Channels[i]<16){	  
				db_set_data_index(pInfo->hDB,hKey[(int)floor(pInfo->Channels[i]/8)],values + i*size,size,pInfo->Channels[i]%8,type);

				//if(rw == 1)
				//printf("channel_rw %i; gantry = %i, rw= %i, ch = %i/%i\n",i,(int)floor(pInfo->Channels[i]/8),
				// rw,pInfo->Channels[i]%8,pInfo->Channels[i] );
			}
		}
	}
}
  
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

  
