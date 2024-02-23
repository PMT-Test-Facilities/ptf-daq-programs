
//current version (working)
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
  3-APR-17    Rika   Updated code to include PMT polygon object for improved collision avoidance.
\********************************************************************/

#include <stdio.h>
#include <sys/time.h>
#include "midas.h"
#include "msystem.h"
#include "mcstd.h"
#include <math.h>
#include <unistd.h>
#include <string.h>
#include "TPathCalculator.hxx" //Rika: See typedef for XYPoint, XYPolygon, XYLine here.
#include "TRotationCalculator.hxx"
#include "TGantryConfigCalculator.hxx" //Rika (27Mar2017): Added as a class shared between feMove & TRotationCalculator.
#include "mfe.h"


/*-- Globals -------------------------------------------------------*/
// The two function-like macros below are used in void channel_rw(INFO *pInfo, HNDLE* hKey, void *values, DWORD type, BOOL rw);
//Note:: See the funciton channel_rw for more information about these macros

//The below preprocessor macro would only work if the "function like" macro was defined on a single line. An identical version of the marco is put in the comment below below but is properly spaced to make it possible to read the code
//#define READ_VALUES(TYPE){*((TYPE*)(values + 0*size)) = *((TYPE*)(motor00_values+5*size));*((TYPE*)(values + 1*size)) = *((TYPE*)(motor00_values+6*size));*((TYPE*)(values + 2*size)) = *((TYPE*)(motor00_values+7*size));*((TYPE*)(values + 3*size)) = *((TYPE*)(motor00_values+4*size));*((TYPE*)(values + 4*size)) = *((TYPE*)(motor00_values+3*size));*((TYPE*)(values + 5*size)) = *((TYPE*)(motor01_values+6*size));*((TYPE*)(values + 6*size)) = *((TYPE*)(motor01_values+1*size));*((TYPE*)(values + 7*size)) = *((TYPE*)(motor01_values+2*size));*((TYPE*)(values + 8*size)) = *((TYPE*)(motor01_values+4*size));*((TYPE*)(values + 9*size)) = *((TYPE*)(motor01_values+5*size));}

/*#define READ_VALUES(TYPE){
  *((TYPE*)(values + 0*size)) = *((TYPE*)(motor00_values+5*size));
  *((TYPE*)(values + 1*size)) = *((TYPE*)(motor00_values+6*size));
  *((TYPE*)(values + 2*size)) = *((TYPE*)(motor00_values+7*size));
  *((TYPE*)(values + 3*size)) = *((TYPE*)(motor00_values+4*size));
  *((TYPE*)(values + 4*size)) = *((TYPE*)(motor00_values+3*size));
  *((TYPE*)(values + 5*size)) = *((TYPE*)(motor01_values+6*size));
  *((TYPE*)(values + 6*size)) = *((TYPE*)(motor01_values+1*size));
  *((TYPE*)(values + 7*size)) = *((TYPE*)(motor01_values+2*size));
  *((TYPE*)(values + 8*size)) = *((TYPE*)(motor01_values+4*size));
  *((TYPE*)(values + 9*size)) = *((TYPE*)(motor01_values+5*size));}*/

//The below preprocessor macro would only work if the function like macro was defined on a single line. An identical version of the marco is put in the comment below but is properly spaced to make it possible to read the code
//#define STORE_DATA(TYPE) {TYPE motor00_TYPE_values[8];TYPE motor01_TYPE_values[8];buff_size=sizeof(motor00_TYPE_values);size = sizeof(TYPE);motor00_TYPE_values[0]= 0;motor00_TYPE_values[1]= 0;motor00_TYPE_values[2]= 0;motor00_TYPE_values[3]=*((TYPE*)(values+4*size));motor00_TYPE_values[4]=*((TYPE*)(values+3*size));motor00_TYPE_values[5]=*((TYPE*)(values+0*size));motor00_TYPE_values[6]=*((TYPE*)(values+1*size));motor00_TYPE_values[7]=*((TYPE*)(values+2*size));motor01_TYPE_values[0]= 0;motor01_TYPE_values[1]=*((TYPE*)(values+6*size));motor01_TYPE_values[2]=*((TYPE*)(values+7*size));motor01_TYPE_values[3]=0;motor01_TYPE_values[4]=*((TYPE*)(values+8*size));motor01_TYPE_values[5]=*((TYPE*)(values+9*size));motor01_TYPE_values[6]=*((TYPE*)(values+5*size));motor01_TYPE_values[7]= 0;motor00_values = motor00_TYPE_values;motor01_values = motor01_TYPE_values;}

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
  motor01_TYPE_values[3]= 0;
  motor01_TYPE_values[4]=*((TYPE*)(values+8*size));   // Rotary
  motor01_TYPE_values[5]=*((TYPE*)(values+9*size));   // Tilt
  motor01_TYPE_values[6]=*((TYPE*)(values+5*size));   // X
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
const char *frontend_name = "feMove";

/* The frontend file name, don't change it                          */
const char *frontend_file_name = __FILE__;

/* frontend_loop is called periodically if this variable is TRUE    */
BOOL frontend_call_loop = FALSE;

/* a frontend status page is displayed with this frequency in ms    */
INT display_period = 0;

/* maximum event size produced by this frontend                     */
INT max_event_size = 3000;

/* buffer size to hold events                                       */
INT event_buffer_size = 10 * 3000;

/* maximum event size for fragmented events (EQ_FRAGMENTED)         */
INT max_event_size_frag = 5 * 300 * 300;

/* counter outside initialize tilt, to prevent infinite loop        */
//INT tilt_ini_attempts = 0;

/* Optionally only control one of two motors */
INT gantry_motor_start = 0;   //for for-loops (by default from motor 0 to motor 9)
INT gantry_motor_end = 10;   //TF TODO: get from ODB and make button on gantry_move page. Only use when one is in repair though!!!


/* Rika (20Apr2017): MOVED GLOBAL CALCULATOR OBJECTS BACK TO GENERATE_PATH() */
// Gantry dimensions with buffer zones for collision avoidance
double tiltMotorLength = 0.160;
double gantryFrontHalfLength = 0.140; //Use 0.140m for safety; measured to be 0.114+/-0.001m (27Apr2017)
double gantryBackHalfLength = 0.25; //0.22 Use 0.200m for safety; measured to be 0.114+0.07(pipelength)=0.184+/-0.002m (27Apr2017) // John (17Oct2019) 0.185 -> 0.22 for safety
double gantryOpticalBoxWidth = 0.160; //Use 0.160m for safety; measured to be 0.135+/-0.002m for optical box 0, 0.145+/-0.001m for optical box 1 // John (17Oct2019) 0.15 -> 0.2 for safety
double gantryTiltGearWidth = 0.060; //Use 0.070 for safety; measured to be 0.060+/-0.001m
double gantryOpticalBoxHeight = 0.095; //Use 0.110m for safety; measured to be 0.094 +/- 0.004m (27Apr2017)

// Tank position & dimensions:
double xtankCentre = 0.366; // Rika (24Apr2017): Updated to estimated new position after tank moved back into coils // Kevin (22Jan2018) estimate change from (0.360, 0.345) -> (0.401, 0.288) -> (0.366, 0.361) Feb21 -> (0.366, 0.371) Feb 23)
double ytankCentre = 0.371;
double tankRadius = 0.61; // radius of tank ~0.61 // John (17Oct2019) reduced from 0.61 to 0.58 to prevent collision
double tankPMTholderRadius = 0.53; // max/min from center where PMT holders sit

// Rika: PMT position for collision avoidance
std::vector <XYPolygon> pmtPoly0; // Model of PMT for collision avoidance
std::vector <XYPolygon> pmtPoly1;
double pmtRadius = 0.323;
double pmtXcentre0 = 0.389; // Rika (24Apr2017): Updated to estimated new position. // Kevin (22Jan2018) changed from (0.348, 0.366) -> (0.389, 0.309) // need to change
double pmtYcentre0 = 0.309;
double pmtXcentre1 = 0.359; // estimate (0.418, 0.396) Feb.21.2018 // 0.366, 0.356
double pmtYcentre1 = 0.326; // 0.331
int numPolySides = 12;
double pmtPolyLayerHeight = 0.01; // 1cm thick polygons
int numPolyLayers = 0.38 /
                    pmtPolyLayerHeight; // Rika (27Apr2017): Inserted enough layers to cover the lowest position that the tip of the optical box can go to
// i.e. if the optical box is tilted -40 degrees, the distance from the gantry z position to the tip of the optical box is 0.178m,
//      so if the gantry goes down to its maximum z height (0.534m), the tip of the optical box will be at 0.178+0.534=0.712,
//      which is 0.712-0.390=0.372m lower than the position of the top of the PMT (so make it 0.38m to play it safe).
// PMT height :
double pmtHeight = 0.390; // Rika (24Apr2017): actual PMT height in gantry coordinates;
// Gantry tilted 0 will hit PMT cover at z=0.3m.
double frpHeight = 0.556; // Rika (28Apr2017): z position of the FRP case, which has a rim that is of wider diameter than the PMT.
// Calculated from the fact that the PMT acrylic cover has a height of 16.6cm.
double frpRadius = 0.322; // Rika (27Apr2017): radius of the rim of the FRP case, to which the PMT cover is attached.
// Value taken from PMT acrylic cover diagram found in Mark Hartz's slides on the T2K Canada page.

// Create GantryConfigCalculator object for calculating different gantry & optical box positions in space
// given rotation and tilt.
TGantryConfigCalculator gantryConfigCalc(tiltMotorLength, gantryFrontHalfLength, gantryBackHalfLength,
                                         gantryOpticalBoxWidth, gantryTiltGearWidth, gantryOpticalBoxHeight);

// Create path calculator object
TPathCalculator pathCalc_checkDestination(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc00(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc01(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc1a(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc1b(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc2a(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc2b(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc3a(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc3b(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc4a(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc4b(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc5a(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc5b(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc6a(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc6b(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc7a(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc7b(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc8a(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TPathCalculator pathCalc8b(xtankCentre, ytankCentre, tankRadius, tankPMTholderRadius);
TRotationCalculator pathCalc_rot_tilt(tiltMotorLength, gantryFrontHalfLength, gantryBackHalfLength,
                                      gantryOpticalBoxWidth, gantryTiltGearWidth, gantryOpticalBoxHeight, xtankCentre,
                                      ytankCentre, tankRadius, tankPMTholderRadius);

/*-- Info structure declaration ------------------------------------*/

BOOL equipment_common_overwrite = FALSE;

typedef struct {
  // Handles for all ODB variables
  HNDLE hDB;
  HNDLE hKeyDest, hKeyStart, hKeyStop, hKeyReInit;            // Equipment/Move/"Control"
  HNDLE hKeyVeloc, hKeyAccel, hKeyScale, hKeyChan, hKeyLimPos; // .../"Settings"
  HNDLE hKeyPos, hKeyInit, hKeySwap, hKeyBadDest, hKeyCompleted,
      hKeyMoving, hKeyAxMoving, hKeyAxLimitNeg, hKeyAxLimitPos, hKeyInitializing; // .../"Variables"
  HNDLE hKeyMDest[2], hKeyMStart[2], hKeyMStop[2], hKeyMMoving[2],
      hKeyMPos[2], hKeyMLimitNeg[2], hKeyMLimitPos[2], hKeyMVel[2], hKeyMAcc[2]; // Motors

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
  float *LimPos;    // Coordinates of the limit switch on each axis (physical units)

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
  float *CountDest;    // Relative axis destinations (counts)
  float *mOrigin;       // Count locations of axis origins (counts)
  float **MovePath;     // 2d array of axis positions for each waypoint of the path (counts)
  int PathSize;         // Number of waypoints in the path
  int PathIndex;        // Index of the path section currently in progress
  int AbortCode;        // Variable in which to store the reason for aborting a move (AC_*)

  // Phidget Tilt variables
  double Phidget[10];      // Equipment/Phidget0x in ODB

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

//INT interrupt_configure(INT cmd, PTYPE adr);
extern void interrupt_routine(void);

INT read_trigger_event(char *pevent, INT off);

INT read_scaler_event(char *pevent, INT off);

// Added functions
void move_init(HNDLE hDB, HNDLE hKey, void *data);

int generate_path(INFO *pInfo);

void monitor(HNDLE hDB, HNDLE hKey, void *data);

void move(INFO *pInfo);

void stop_move(HNDLE hDB, HNDLE hKey, void *data);

void initialize(INFO *pInfo);

void reinitialize(HNDLE hDB, HNDLE hKey, void *data);

void channel_rw(INFO *pInfo, HNDLE *hKey, void *values, DWORD type, BOOL rw);

void
getPMTpolygon(XYPolygon &poly, double pmtRadius, int polyNum, double pmtXcentre, double pmtYcentre, int numPolySides,
              double polyHeight);

/*-- Equipment list ------------------------------------------------*/

//#define USE_INT 1

EQUIPMENT equipment[] = {

    {"Move",            // equipment name
        {5, 0,              // event ID, trigger mask
            "SYSTEM",           // event buffer
            EQ_PERIODIC,        // equipment type
            0,                  // event source
            "MIDAS",            // format
            TRUE,               // enabled
            RO_ALWAYS,          // read x
            10000,              // read every x millisec
            0,                  // stop run after this event limit
            0,                  // number of sub event
            60,                  // log history every x sec
            "", "", "",},
        read_trigger_event, // readout routine
        NULL,               // class driver main routine
        NULL,                // device driver list
        NULL,               // init string
    },

    {""}
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

INT begin_of_run(INT run_number, char *error) {
  return CM_SUCCESS;
}

/*-- End of Run ----------------------------------------------------*/

INT end_of_run(INT run_number, char *error) {
  return CM_SUCCESS;
}

/*-- Pause Run -----------------------------------------------------*/

INT pause_run(INT run_number, char *error) {
  return CM_SUCCESS;
}

/*-- Resume Run ----------------------------------------------------*/

INT resume_run(INT run_number, char *error) {
  return CM_SUCCESS;
}

/*-- Frontend Loop -------------------------------------------------*/

INT frontend_loop() {
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


/*-- Event readout -------------------------------------------------*/
INT read_trigger_event(char *pevent, INT off) {
  return 0;
}

/*-- Scaler event --------------------------------------------------*/

INT read_scaler_event(char *pevent, INT off) {
  return 0;
}



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
INT frontend_init() {
  HNDLE hDB;
  INFO *pInfo;
  int i;
  float tempV[10];
  float tempA[10];

  /* Allocate info struct */
  pInfo = (INFO *) calloc(1, sizeof(INFO));

  pInfo->Destination = (float *) calloc(10, sizeof(float));
  pInfo->Velocity = (float *) calloc(10, sizeof(float));
  pInfo->Acceleration = (float *) calloc(10, sizeof(float));
  pInfo->mScale = (float *) calloc(10, sizeof(float));
  pInfo->Channels = (INT *) calloc(10, sizeof(float));
  pInfo->LimPos = (float *) calloc(10, sizeof(float));
  pInfo->Position = (float *) calloc(10, sizeof(float));
  pInfo->AxisMoving = (BOOL *) calloc(10, sizeof(BOOL));
  pInfo->neg_AxisLimit = (BOOL *) calloc(10, sizeof(BOOL));
  pInfo->pos_AxisLimit = (BOOL *) calloc(10, sizeof(BOOL));
  pInfo->CountPos = (float *) calloc(10, sizeof(float));
  pInfo->CountDest = (float *) calloc(10, sizeof(float));
  pInfo->mOrigin = (float *) calloc(10, sizeof(float));
  pInfo->MovePath = (float **) calloc(10, sizeof(float *));
  pInfo->MovePath = (float **) calloc(10, sizeof(float *));

  /* Get database handle */
  cm_get_experiment_database(&hDB, NULL);
  pInfo->hDB = hDB;

  /* Initialize non-ODB variables */
  pInfo->PathSize = 0;
  pInfo->PathIndex = 0;
  pInfo->AbortCode = AC_USER_INPUT;

  /* Initialize "Control" ODB variables */

  // "Destination"
  // Don't reset destination when initializing. TL, i.e don't call db_set_data here
  db_find_key(hDB, 0, "/Equipment/Move/Control/Destination", &pInfo->hKeyDest);

  // "Start Move"
  db_find_key(hDB, 0, "/Equipment/Move/Control/Start Move", &pInfo->hKeyStart);
  pInfo->Start = 0;
  db_set_data(hDB, pInfo->hKeyStart, &pInfo->Start, sizeof(BOOL), 1, TID_BOOL);

  // "Stop Move"
  db_find_key(hDB, 0, "/Equipment/Move/Control/Stop Move", &pInfo->hKeyStop);
  pInfo->Stop = 0;
  db_set_data(hDB, pInfo->hKeyStop, &pInfo->Stop, sizeof(BOOL), 1, TID_BOOL);

  // "ReInitialize"
  db_find_key(hDB, 0, "/Equipment/Move/Control/ReInitialize", &pInfo->hKeyReInit);
  pInfo->ReInitialize = 0;
  db_set_data(hDB, pInfo->hKeyReInit, &pInfo->ReInitialize, sizeof(BOOL), 1, TID_BOOL);

  /* Initialize "Settings" ODB variables */
  // Note: The following variables can only be changed when feMove is
  //			 not running. If feMove is already running, changing these variables
  //       in the ODB will have no effect.

  // "Velocity"
  db_merge_data(hDB, 0, "/Equipment/Move/Settings/Velocity", pInfo->Velocity, 10 * sizeof(float), 10, TID_FLOAT);
  db_find_key(hDB, 0, "/Equipment/Move/Settings/Velocity", &pInfo->hKeyVeloc);

  // "Acceleration"
  db_merge_data(hDB, 0, "/Equipment/Move/Settings/Acceleration", pInfo->Acceleration, 10 * sizeof(float), 10,
                TID_FLOAT);
  db_find_key(hDB, 0, "/Equipment/Move/Settings/Acceleration", &pInfo->hKeyAccel);

  // "Motor Scaling"
  db_merge_data(hDB, 0, "/Equipment/Move/Settings/Motor Scaling", pInfo->mScale, 10 * sizeof(float), 10, TID_FLOAT);
  db_find_key(hDB, 0, "/Equipment/Move/Settings/Motor Scaling", &pInfo->hKeyScale);

  // "Axis Channels" 
  db_merge_data(hDB, 0, "/Equipment/Move/Settings/Axis Channels", pInfo->Channels, 10 * sizeof(INT), 10, TID_INT);
  db_find_key(hDB, 0, "/Equipment/Move/Settings/Axis Channels", &pInfo->hKeyChan);

  // "Limit Positions"
  db_merge_data(hDB, 0, "/Equipment/Move/Settings/Limit Positions", pInfo->LimPos, 10 * sizeof(float), 10, TID_FLOAT);
  db_find_key(hDB, 0, "/Equipment/Move/Settings/Limit Positions", &pInfo->hKeyLimPos);

  /* Initialize "Variables" ODB variables */
  // "Position(X,Y,Z,Theta,Phi)"
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Position", &pInfo->hKeyPos);
  db_set_data(hDB, pInfo->hKeyPos, pInfo->Position, 10 * sizeof(float), 10, TID_FLOAT);

  // "Initialized"
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Initialized", &pInfo->hKeyInit);
  pInfo->Initialized = 0;
  db_set_data(hDB, pInfo->hKeyInit, &pInfo->Initialized, sizeof(BOOL), 1, TID_BOOL);

  // "Bad Destination"
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Bad Destination", &pInfo->hKeyBadDest);
  pInfo->BadDest = 0;
  db_set_data(hDB, pInfo->hKeyBadDest, &pInfo->BadDest, sizeof(BOOL), 1, TID_BOOL);

  // "Completed"
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Completed", &pInfo->hKeyCompleted);
  pInfo->Completed = 1;
  db_set_data(hDB, pInfo->hKeyCompleted, &pInfo->Completed, sizeof(BOOL), 1, TID_BOOL);

  // "Moving"
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Moving", &pInfo->hKeyMoving);

  // "Initializing"
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Initializing", &pInfo->hKeyInitializing);

  // "Axis Moving"
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Axis Moving", &pInfo->hKeyAxMoving);
  db_set_data(hDB, pInfo->hKeyAxMoving, pInfo->AxisMoving, 10 * sizeof(BOOL), 10, TID_BOOL);

  // "Negative Axis Limit"  
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Negative Axis Limit", &pInfo->hKeyAxLimitNeg);
  db_set_data(hDB, pInfo->hKeyAxLimitNeg, pInfo->neg_AxisLimit, 10 * sizeof(BOOL), 10, TID_BOOL);

  // "Positive Axis Limit"  
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Positive Axis Limit", &pInfo->hKeyAxLimitPos);
  db_set_data(hDB, pInfo->hKeyAxLimitPos, pInfo->pos_AxisLimit, 10 * sizeof(BOOL), 10, TID_BOOL);

  /* Get handles for the "Motor" ODB variables */
  // "Destination"
  db_find_key(hDB, 0, "/Equipment/Motors00/Settings/Destination", &pInfo->hKeyMDest[0]);
  db_find_key(hDB, 0, "/Equipment/Motors01/Settings/Destination", &pInfo->hKeyMDest[1]);

  // "Move"
  db_find_key(hDB, 0, "/Equipment/Motors00/Settings/Move", &pInfo->hKeyMStart[0]);
  db_find_key(hDB, 0, "/Equipment/Motors01/Settings/Move", &pInfo->hKeyMStart[1]);

  // "Stop"
  db_find_key(hDB, 0, "/Equipment/Motors00/Settings/Stop", &pInfo->hKeyMStop[0]);
  db_find_key(hDB, 0, "/Equipment/Motors01/Settings/Stop", &pInfo->hKeyMStop[1]);

  // "Moving"
  db_find_key(hDB, 0, "/Equipment/Motors00/Variables/Moving", &pInfo->hKeyMMoving[0]);
  db_find_key(hDB, 0, "/Equipment/Motors01/Variables/Moving", &pInfo->hKeyMMoving[1]);

  // "Position"
  db_find_key(hDB, 0, "/Equipment/Motors00/Variables/Position", &pInfo->hKeyMPos[0]);
  db_find_key(hDB, 0, "/Equipment/Motors01/Variables/Position", &pInfo->hKeyMPos[1]);

  // "Limit Pos"
  db_find_key(hDB, 0, "/Equipment/Motors00/Variables/Limit Pos", &pInfo->hKeyMLimitPos[0]);
  db_find_key(hDB, 0, "/Equipment/Motors01/Variables/Limit Pos", &pInfo->hKeyMLimitPos[1]);

  // "Limit Neg"
  db_find_key(hDB, 0, "/Equipment/Motors00/Variables/Limit Neg", &pInfo->hKeyMLimitNeg[0]);
  db_find_key(hDB, 0, "/Equipment/Motors01/Variables/Limit Neg", &pInfo->hKeyMLimitNeg[1]);

  // "Velocity"
  db_find_key(hDB, 0, "/Equipment/Motors00/Settings/Velocity", &pInfo->hKeyMVel[0]);
  db_find_key(hDB, 0, "/Equipment/Motors01/Settings/Velocity", &pInfo->hKeyMVel[1]);

  // "Acceleration"
  db_find_key(hDB, 0, "/Equipment/Motors00/Settings/Acceleration", &pInfo->hKeyMAcc[0]);
  db_find_key(hDB, 0, "/Equipment/Motors01/Settings/Acceleration", &pInfo->hKeyMAcc[1]);

  // "Accelerometer tilt"
  //db_find_key(hDB, 0, "/Equipment/Phidget00/Variables", &pInfo->hKeyPhidget[0]);
  //db_find_key(hDB, 0, "/Equipment/Phidget01/Variables", &pInfo->hKeyPhidget[1]);

  /* Set up hotlinks */
  // move_init() hotlink
  db_open_record(hDB, pInfo->hKeyStart, &pInfo->Start, sizeof(BOOL), MODE_READ, move_init, pInfo);

  // stop_move() hotlink
  //db_open_record(hDB, pInfo->hKeyStop, &pInfo->Stop, sizeof(BOOL), MODE_READ, stop_move, pInfo);

  // reinitialize() hotlink
  db_open_record(hDB, pInfo->hKeyReInit, &pInfo->ReInitialize, sizeof(BOOL), MODE_READ, reinitialize, pInfo);

  // monitor() hotlink
  // db_open_record(hDB, pInfo->hKeyMMoving[0], NULL, 8 * sizeof(BOOL), MODE_READ, monitor, pInfo);
  //db_open_record(hDB, pInfo->hKeyMMoving[1], NULL, 8 * sizeof(BOOL), MODE_READ, monitor, pInfo);
  // Note: The variable hotlinked to monitor() is somewhat arbitrary, all that is 
  //	   really needed is a variable that gets periodically updated in feMove.

  /* Set motor velocities and accelerations to in feMotor */
  printf("feMove init looping over %i %i\n", gantry_motor_start,gantry_motor_end);
  for (i = gantry_motor_start; i < gantry_motor_end; i++) {
    tempV[i] = pInfo->Velocity[i] * fabs(pInfo->mScale[i]);
    tempA[i] = pInfo->Acceleration[i] * fabs(pInfo->mScale[i]);
    
    printf("Move checks %i velo=%f accel=%f mscale=%f tempV=%f\n",i,pInfo->Velocity[i],pInfo->Acceleration[i], pInfo->mScale[i], tempV[i]);

  }

  channel_rw(pInfo, pInfo->hKeyMVel, (void *) tempV, TID_FLOAT, 1);
  channel_rw(pInfo, pInfo->hKeyMAcc, (void *) tempA, TID_FLOAT, 1);
  printf("feInitialize Success");
  //  return FE_ERR_HW;
  return CM_SUCCESS;


}

/*-- Frontend Exit -------------------------------------------------*/
// Stop the motors and free all memory in this function
INT frontend_exit() {
  return CM_SUCCESS;
}
double tilt_min = -105, tilt_max = 15; 
/********************************************************************\
                Main routines during frontend operation

  These routines are called when hotlinked ODB variables are changed
  during frontend operation. They operate by changing variables in 
  "Equipment/Motor" to control the gantry operation.

  move_init:      Hotlinked to the variable "Start Move", this 
                  function generates the move path and initializes 
                  the move.

  initialize:	  This function moves the motors to the limit switches
		  (in a guaranteed collision free manner) and determines
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
void move_init(HNDLE hDB, HNDLE hKey, void *data) {
  INFO *pInfo = (INFO *) data;
  // BOOL Check_Phidgets = TRUE;
  // HNDLE hPhidgetVars0 = 0, hPhidgetVars1 = 0;
  //double phidget_Values_Old[10];
  // double phidget_Values_Now[10];
  // int size_of_array = sizeof(phidget_Values_Old);
  // DWORD time_at_start_of_check;

  // FOR DEBUGGING
  cm_msg(MDEBUG, "move_init", "Function move_init called.");

  // Check that the user has set "Start Move" to "y"
  int size = sizeof(BOOL);
  db_get_data(hDB, pInfo->hKeyStart, &pInfo->Start, &size, TID_BOOL);
  if (pInfo->Start == 0) return;

  // If motors are already moving, set destinations back to previous
  // destination values and return with an error.
  if (pInfo->Moving) {
    cm_msg(MERROR, "move_init", "Error: Can't start move. Move already in progress.");
    return;
  }

  //cm_msg(MDEBUG, "move_init", "Checking phidget response...");
  //if (!phidget_responding(hDB)) {
  //  return;
  //}

  // Check if the motors have been initialized. If not, initialize them
  // before proceeding.
  db_get_data(hDB, pInfo->hKeyInit, &pInfo->Initialized, &size, TID_BOOL);
  cm_msg(MDEBUG, "move_init", "Checking if motors are initialized...");
  if (pInfo->Initialized == 0) {
    cm_msg(MDEBUG, "move_init", "They aren't. Running initialization.");
    initialize(pInfo);
  }
  // If initialization fails, return with error
  if (pInfo->Initialized == 0) {
    cm_msg(MERROR, "move_init", "Error: Can't start move. Initialization failed.");
    return;
  }
  // Check if phidget tilt readings match the ODB variables before proceeding
  // TODO: since error check involves reading phidget, also make sure phidget is responsive
  // TODO (MAYBE): first check if tilt measurements agree 
  //               if they disagree --> try to move it to desired location first
  //               if they still dont match exit
  double tilt_tolerance = 3.0;
  double tilt_min = -105, tilt_max = 15;
  //size = sizeof(pInfo->Phidget);
  //int tilt_start = 0, tilt_end = 2; //these are the number of gantries we have to correct
  //for (int i = tilt_start; i < tilt_end; i++) {
  //  int axis = (i == 0 ? 4 : 9);
   // db_get_value(pInfo->hDB, pInfo->hKeyPhidget[0], "PH00", &pInfo->Phidget, &size, TID_DOUBLE, FALSE);
    //if (i == 1) { db_get_value(pInfo->hDB, pInfo->hKeyPhidget[1], "PH01", &pInfo->Phidget, &size, TID_DOUBLE, FALSE); }

    // DEBUG
    //cm_msg(MINFO, "move_init", "Phidget0%i tilt: %f ODB tilt: %f", i, pInfo->Phidget[7], pInfo->Position[axis]);

    // Check if  phidget tilt readings match ODB values
    //if (pInfo->Position[axis] < pInfo->Phidget[7] - tilt_tolerance ||
    //    pInfo->Position[axis] > pInfo->Phidget[7] + tilt_tolerance) {
     // cm_msg(MERROR, "move_init", "Phidget0%i tilt: %f ODB tilt: %f", i, pInfo->Phidget[7], pInfo->Position[axis]);
     // cm_msg(MERROR, "move_init", "ERROR: can't start move. Phidget0%i tilt and ODB tilt do not agree within tolerance",
       //      i);
      //return;
    //}

    // Check if phidget reading is within legal range
    //if (pInfo->Phidget[7] < tilt_min || pInfo->Phidget[7] > tilt_max) {
    //  cm_msg(MERROR, "move_init", "ERROR: can't start move. Phidget0%i tilt: %f is in illegal range", i,
    //         pInfo->Phidget[7]);
    //  return;
   // }
 // }

  // Load input destination into pInfo->Destination
  size = 10 * sizeof(float);
  db_get_data(hDB, pInfo->hKeyDest, pInfo->Destination, &size, TID_FLOAT);

  // Generate collision free path to pInfo->Destination
  cm_msg(MINFO, "move_init", "Generating Path");
  //int Status = generate_path(pInfo);
  
  // Check for unsuccesful path generation
  /*  switch (Status) {
    case GENPATH_SUCCESS:
      pInfo->BadDest = 0;
      db_set_data(hDB, pInfo->hKeyBadDest, &pInfo->BadDest, sizeof(BOOL), 1, TID_BOOL);
      break;
    case GENPATH_BAD_DEST:
      cm_msg(MERROR, "move_init", " Bad destination entered in Move");
      pInfo->BadDest = 1;
      db_set_data(hDB, pInfo->hKeyBadDest, &pInfo->BadDest, sizeof(BOOL), 1, TID_BOOL);
      return;
  }*/
  cm_msg(MINFO, "move_init", "Path succesfully generated");

  // Set the "completed" variable in the ODB to 0 (since our newly
  // started move is still incomplete)
  pInfo->Completed = 0;
  db_set_data(hDB, pInfo->hKeyCompleted, &pInfo->Completed, sizeof(BOOL), 1, TID_BOOL);

  // Move to first path index
  //move(pInfo);


  // Set the ODB variable "Start Move" back to 0
  pInfo->Start = 0;
  db_set_data(hDB, pInfo->hKeyStart, &pInfo->Start, sizeof(BOOL), 1, TID_BOOL);
}


/*-- Initialize ----------------------------------------------------*/
// Move motors to limit switches. Use the motor coordinates at this
// position to determine the motor coordinates at the origin. If the
// negative limit switch is disabled, the current position is set as the
// origin for that axis.
void initialize(INFO *pInfo) {
  int i;
  BOOL *tempStart = (BOOL *) calloc(10, sizeof(BOOL));
  BOOL *tempNegLimitEnabled = (BOOL *) calloc(10, sizeof(BOOL));
  float *tempPos = (float *) calloc(10, sizeof(float));
  BOOL tempStop[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
  BOOL exitFlag = 0;
  float lastCountPosArm1;
  float lastCountPosArm2;


  cm_msg(MINFO, "initialize", "Initializing motors");
  BOOL initing = 1;
  db_set_data(pInfo->hDB, pInfo->hKeyInitializing, &initing, sizeof(BOOL), 1, TID_BOOL);

  // Set motors to move into their negative limit switches, if they are enabled 
  //(disabled is LimPos = 9999)
  // Motors 4 and 9 are the tilt motors.  These are initialized separately, using 
  // the tilt measurement from the phidget.
  //for(i=0;i<10;i++){
  for (i = gantry_motor_start; i < gantry_motor_end; i++) {
    if (i == 4 || i == 9) {
      //Do nothing, tilt should already be initialized
      tempPos[i] = 0;
      tempNegLimitEnabled[i] = 0;
      cm_msg(MINFO, "initialize", "Axis %i for tilt motor will be initialized separately.", i);
    } else if (pInfo->LimPos[i] == 9999) {
      tempPos[i] = 0;
      tempNegLimitEnabled[i] = 0;
      cm_msg(MINFO, "initialize", "Negative limit switch for axis %i disabled. Axis will not be initialized.", i);
    } else {
      tempPos[i] = 500 * fabs(pInfo->mScale[i]);
      tempNegLimitEnabled[i] = 1;
      cm_msg(MINFO, "initialize",
             "Negative limit switch for axis %i enabled. Axis will be initialized; use position %f.", i, tempPos[i]);
    }
  }

  channel_rw(pInfo, pInfo->hKeyMDest, (void *) tempPos, TID_FLOAT, 1);

  // Cycle through each pair of motors corresponding to the same axis on each arm.
  // We want to make sure that we initialize the Z-axis first, so that the laser box is
  // fully out of the tank before we move in X and Y.
  int order[4] = {2, 0, 1, 3};
  int itmp;
  for (itmp = 0; itmp < 4; itmp++) {
    i = order[itmp];

    if ((tempNegLimitEnabled[i] == 1) || (tempNegLimitEnabled[i + 5] == 1)) {
      cm_msg(MINFO, "initialize", "Initializing axes %i and %i (enabled = %i %i)", i, i + 5, tempNegLimitEnabled[i],
             tempNegLimitEnabled[i + 5]);
      //printf("Going to destination %f %f\n",tempPos[i],tempPos[i+5]);
    }
    tempStart[i] = tempNegLimitEnabled[i];
    tempStart[i + 5] = tempNegLimitEnabled[i + 5];

    channel_rw(pInfo, pInfo->hKeyMStart, (void *) tempStart, TID_BOOL, 1);

    // Wait for axes with enabled limit switches to hit their limits
    while (1) {
      channel_rw(pInfo, pInfo->hKeyMPos, pInfo->CountPos, TID_FLOAT, 0);
      lastCountPosArm1 = pInfo->CountPos[i];
      lastCountPosArm2 = pInfo->CountPos[i + 5];
      //sleep(5);
      usleep(600000); // Approx. polling period
      if ((tempNegLimitEnabled[i] == 1) && (tempNegLimitEnabled[i + 5] == 1)) {// If both gantry axes are enabled.
        cm_msg(MDEBUG, "initialize", "Polling axes %i and %i.", i, i+5);
        channel_rw(pInfo, pInfo->hKeyMLimitNeg, (void *) pInfo->neg_AxisLimit, TID_BOOL, 0);
        if (pInfo->neg_AxisLimit[i] && pInfo->neg_AxisLimit[i + 5]) break;
        else {
          channel_rw(pInfo, pInfo->hKeyMPos, pInfo->CountPos, TID_FLOAT, 0);

	  //Print tests
	  printf("Pos Count1: %f",pInfo->CountPos[0]);
	  printf("Pos Count2: %f",pInfo->CountPos[1]);
	  printf("Pos Count3: %f",pInfo->CountPos[2]);

	  printf("Pos Count: %f",lastCountPosArm1);
	  printf("Pos Count: %f",pInfo->neg_AxisLimit[i]);
	  //printf();
	  
	  //Changed while testing Jun 14th 
          if (pInfo->CountPos[i] == lastCountPosArm1 && !pInfo->neg_AxisLimit[i]) { 
            cm_msg(MERROR, "initialize",
                   "Axis %i not moving. Stopping initialization since limit switch must be broken.", i);
            channel_rw(pInfo, pInfo->hKeyMStop, (void *) tempStop, TID_BOOL, 1);
            exitFlag = 1;
            break;
          }
          if (pInfo->CountPos[i + 5] != lastCountPosArm2){ //&& !pInfo->neg_AxisLimit[i + 5]) {
            cm_msg(MERROR, "initialize",
                   "Axis %i not moving. Stopping initialization since limit switch must be broken.", i + 5);
            channel_rw(pInfo, pInfo->hKeyMStop, (void *) tempStop, TID_BOOL, 1);
            exitFlag = 1;
            break;
          }
        }
      } else if ((tempNegLimitEnabled[i] == 0) && (tempNegLimitEnabled[i + 5] == 0)) {
        break;
      } else if (tempNegLimitEnabled[i] == 0) {  // If only second gantry axis is enabled.
        cm_msg(MDEBUG, "initialize", "Polling axis %i.", i + 5);
        channel_rw(pInfo, pInfo->hKeyMLimitNeg, (void *) pInfo->neg_AxisLimit, TID_BOOL, 0);
        if (pInfo->neg_AxisLimit[i + 5]) {
          break;
        } else {
          channel_rw(pInfo, pInfo->hKeyMPos, pInfo->CountPos, TID_FLOAT, 0);
          if (pInfo->CountPos[i + 5] == lastCountPosArm2) {
            cm_msg(MERROR, "initialize",
                   "Axis %i not moving. Stopping initialization since limit switch must be broken.", i + 5);
            channel_rw(pInfo, pInfo->hKeyMStop, (void *) tempStop, TID_BOOL, 1);
            exitFlag = 1;
            break;
          }
        }
      } else {// If only first gantry axis is enabled.
        cm_msg(MDEBUG, "initialize", "Polling axis %i.", i);
        channel_rw(pInfo, pInfo->hKeyMLimitNeg, (void *) pInfo->neg_AxisLimit, TID_BOOL, 0);
        if (pInfo->neg_AxisLimit[i]) {
          break;
        } else {
          channel_rw(pInfo, pInfo->hKeyMPos, pInfo->CountPos, TID_FLOAT, 0);
          if (pInfo->CountPos[i] == lastCountPosArm1) {
            cm_msg(MERROR, "initialize",
                   "Axis %i not moving. Stopping initialization since limit switch must be broken.", i);
            channel_rw(pInfo, pInfo->hKeyMStop, (void *) tempStop, TID_BOOL, 1);
            exitFlag = 1;
            break;
          }
        }
      }
    }
    tempStart[i] = 0;
    tempStart[i + 5] = 0;
    if (exitFlag == 1) break;
  }

  // Wait for all the motors to stop moving (sometimes this 
  // occurs slightly after the limit switches are triggered)
  pInfo->Moving = 1;
  while (pInfo->Moving) {
    pInfo->Moving = 0;
    channel_rw(pInfo, pInfo->hKeyMMoving, (void *) pInfo->AxisMoving, TID_BOOL, 0);
    for (i = gantry_motor_start; i < gantry_motor_end; i++) pInfo->Moving = pInfo->Moving || pInfo->AxisMoving[i];
  }

  if (exitFlag == 0) {
    // Determine the motor positions at the origin and put the results in
    // pInfo->mOrigin
    channel_rw(pInfo, pInfo->hKeyMPos, pInfo->CountPos, TID_FLOAT, 0);
    for (i = 0; i < 10; i++) {
      //for(i = gantry_motor_start; i < gantry_motor_end ; i++){
      // only exception in flexible for loop: even if other motor is off, still initialize to LimPos...TF: safe??
      if (tempNegLimitEnabled[i] == 1 || (i >= gantry_motor_end || i < gantry_motor_start)) {
        // The motor origin should always be at the limit switches, since this is 
        // a local variable
        // MARK 
        pInfo->mOrigin[i] = pInfo->CountPos[i];// - pInfo->LimPos[i]*pInfo->mScale[i];
        pInfo->Position[i] = pInfo->LimPos[i];
        cm_msg(MINFO, "initialize", "Finished initializing axis %i; Origin = %5.2f counts, Position = %5.2f m.", i,
               pInfo->mOrigin[i], pInfo->Position[i]);
      } else {
        pInfo->mOrigin[i] = 0;
        pInfo->Position[i] = 0;
      }
    }
    db_set_data(pInfo->hDB, pInfo->hKeyPos, &pInfo->Position, 10 * sizeof(float), 10, TID_FLOAT);
  }

  //int exitFlagTilt = initialize_tilt(pInfo);


  if (exitFlag == 0)  { 
    // Set Initialized to 1 and exit    
    pInfo->Initialized = 1;
    db_set_data(pInfo->hDB, pInfo->hKeyInit, &pInfo->Initialized, sizeof(BOOL), 1, TID_BOOL);
  }//exitFlagTilt == 0) {

  initing = 0;
  db_set_data(pInfo->hDB, pInfo->hKeyInitializing, &initing, sizeof(BOOL), 1, TID_BOOL);

  usleep(100000);//0.1 sec
  cm_msg(MINFO, "move_init", "Initialization of Gantries is Complete");

  /* Rika (20Apr2017): Moved initialization of PMT to initialize() method */
  // PMT position: to be used in generate_path for PMT collision avoidance.
  pmtPoly0.clear();
  pmtPoly1.clear();
  XYPolygon poly0, poly1;
  for (int polyNum = 1; polyNum <= numPolyLayers; ++polyNum) {
    poly0.clear();
    poly1.clear();
    //cm_msg(MINFO, "generate_path", " PMT polygon 0, layer %i:", polyNum);
    getPMTpolygon(poly0, pmtRadius, polyNum, pmtXcentre0, pmtYcentre0, numPolySides, pmtPolyLayerHeight);
    //cm_msg(MINFO, "generate_path", " PMT polygon 1, layer %i:", polyNum);
    getPMTpolygon(poly1, pmtRadius, polyNum, pmtXcentre1, pmtYcentre1, numPolySides, pmtPolyLayerHeight);
    pmtPoly0.push_back(poly0);
    pmtPoly1.push_back(poly1);
  }
  pathCalc00.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc01.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc1a.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc1b.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc2a.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc2b.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc3a.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc3b.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc4a.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc4b.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc5a.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc5b.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc6a.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc6b.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc7a.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc7b.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc8a.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc8b.InitialisePMT(pmtPoly0, pmtPoly1);
  pathCalc_rot_tilt.InitialisePMT(pmtPoly0, pmtPoly1, pmtHeight, pmtPolyLayerHeight);
  cm_msg(MINFO, "move_init", "Initialization of PMT position is Complete");

  return;


}

/*-- Re-Initialize -------------------------------------------------*/
// Set initialized to 0, then start the move
void reinitialize(HNDLE hDB, HNDLE hKey, void *data) {

  INFO *pInfo = (INFO *) data;
  INT size = sizeof(BOOL);

  db_get_data(hDB, pInfo->hKeyReInit, &pInfo->ReInitialize, &size, TID_BOOL);
  if (!pInfo->ReInitialize) return;

  pInfo->Initialized = 0;
  db_set_data(hDB, pInfo->hKeyInit, &pInfo->Initialized, sizeof(BOOL), 1, TID_BOOL);

  pInfo->Start = 1;
  db_set_data(hDB, pInfo->hKeyStart, &pInfo->Start, sizeof(BOOL), 1, TID_BOOL);

  pInfo->ReInitialize = 0;
  db_set_data(hDB, pInfo->hKeyReInit, &pInfo->ReInitialize, sizeof(BOOL), 1, TID_BOOL);

  pInfo->BadDest = 0;
  db_set_data(hDB, pInfo->hKeyBadDest, &pInfo->BadDest, sizeof(BOOL), 1, TID_BOOL);
}





/*-- Channel Read/Write --------------------------------------------*/
// Used to communicate with the variables in the motor frontend.
// Arguments:
// 		pInfo: Info structure
//		hKey: Pointer to the ODB handle array to be read from/written to
//		values: Pointer to the array from which to read/write data
//		type: Type ID of the variable to be written (TID_FLOAT, TID_BOOL, TID_INT)
//		rw: set to 0 for read from ODB, 1 for write to ODB
void channel_rw(INFO *pInfo, HNDLE *hKey, void *values, DWORD type, BOOL rw) {

//!!!!The Below comment is meant to help expalin how this function works as it uses "function-like" macros which people may not be famillar with. 

/*
//STORE_DATA and READ_DATA are "function-like" macros using the #define preprocessor macro (See lines 60 and 76) #define performs textual replacement. When used as a function-like macro #define will take arguments like a function. 
//EXAMPLE:: STORE_DATA(float, values, size) is a function-like macro. The preproccesor will replace the line STORE_DATA(float); with the lines of code listed after the #define (see line 76) and everywhere that the text "TYPE" appears in those lines of code will be replaced with the text float. END EXAMPLE
//The "function-like" are used to reduce the number of lines of code and to improve the readibility of the code. It is used to accomplish what templete functions do in C++
//NOTE!!:: Use of template functions can be difficult when debugging this section as the code you see is not what the compiler sees. The compiler sees the code after the preproccesor has completed the text replacement. If debugging this section it may be wise to not use the macro and instead do the textual replacement yourself and then debug. Comment by Ben Krupicz  
*/

  int size;
  int buff_size;
  void *motor00_values;
  void *motor01_values;



  switch (type) {
    case TID_FLOAT:
      printf("TID_FLOAT\n");
      // STORE_DATA(TYPE) determines the correct value for the size value and fills  motor00_values and motor01_values with the entries in the values variable
      //STORE_DATA(float);
      float motor00_type_values[8];
      float motor01_type_values[8];
      
      buff_size=sizeof(motor00_type_values);
      size = sizeof(float);
      
      motor00_type_values[0]= 0;
      motor00_type_values[1]= 0;
      motor00_type_values[2]= 0;
      motor00_type_values[3]=*((float*)(values+4*size));   // Tilt
      motor00_type_values[4]=*((float*)(values+3*size));   // Rotary
      motor00_type_values[5]=*((float*)(values+0*size));   // X
      motor00_type_values[6]=*((float*)(values+1*size));   // Y
      motor00_type_values[7]=*((float*)(values+2*size));   // Z
        
      motor01_type_values[0]= 0;  
      motor01_type_values[1]=*((float*)(values+6*size));   // Y
      motor01_type_values[2]=*((float*)(values+7*size));   // Z
      motor01_type_values[3]= 0;
      motor01_type_values[4]=*((float*)(values+8*size));   // Rotary
      motor01_type_values[5]=*((float*)(values+9*size));   // Tilt
      motor01_type_values[6]=*((float*)(values+5*size));   // X
      motor01_type_values[7]= 0;  
      motor00_values = motor00_type_values;
      motor01_values = motor01_type_values;
      
      for(int i = 0; i<8; i++){
	
	printf("channel_rw during float %i %f \n",i,((float*)motor00_values)[i]);
      }      

      break;
    case TID_BOOL: 
      printf("TID_BOOL\n");

      BOOL motor00_bool_values[8];
      BOOL motor01_bool_values[8];
      
      buff_size=sizeof(motor00_bool_values);
      size = sizeof(BOOL);
      
      motor00_bool_values[0]= 0;
      motor00_bool_values[1]= 0;
      motor00_bool_values[2]= 0;
      motor00_bool_values[3]= 0;//*((BOOL*)(values+4*size));   // Tilt
      motor00_bool_values[4]=*((BOOL*)(values+3*size));   // Rotary
      motor00_bool_values[5]=*((BOOL*)(values+0*size));   // X
      motor00_bool_values[6]=*((BOOL*)(values+1*size));   // Y
      motor00_bool_values[7]=*((BOOL*)(values+2*size));   // Z
        
      motor01_bool_values[0]= 0;  
      motor01_bool_values[1]=*((BOOL*)(values+6*size));   // Y
      motor01_bool_values[2]=*((BOOL*)(values+7*size));   // Z
      motor01_bool_values[3]= 0;
      motor01_bool_values[4]=*((BOOL*)(values+8*size));   // Rotary
      motor01_bool_values[5]= 0;//*((BOOL*)(values+9*size));   // Tilt
      motor01_bool_values[6]=*((BOOL*)(values+5*size));   // X
      motor01_bool_values[7]= 0;  
      motor00_values = motor00_bool_values;
      motor01_values = motor01_bool_values;

      break;
    case TID_INT: 
      printf("TID_INT\n");

      int motor00_int_values[8];
      int motor01_int_values[8];
      
      buff_size=sizeof(motor00_int_values);
      size = sizeof(int);
      
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
      motor01_int_values[3]= 0;
      motor01_int_values[4]=*((int*)(values+8*size));   // Rotary
      motor01_int_values[5]=*((int*)(values+9*size));   // Tilt
      motor01_int_values[6]=*((int*)(values+5*size));   // X
      motor01_int_values[7]= 0;  
      motor00_values = motor00_int_values;
      motor01_values = motor01_int_values;

	    
      //STORE_DATA(int)
      break;
  }

  if(type == TID_FLOAT){
    //float *fvalues = (float*)motor00_values;
    for(int i = 0; i<10; i++){

      printf("channel_rw before float %i %f \n",i,((float*)values)[i] );
    }

    for(int i = 0; i<8; i++){

      printf("channel_rw after float %i %f \n",i,((float*)motor00_values)[i] );
    }
  }


  if (rw == 0) {
    db_get_data(pInfo->hDB, hKey[0], motor00_values, &buff_size, type);
    db_get_data(pInfo->hDB, hKey[1], motor01_values, &buff_size, type);

    switch (type) {
      case TID_FLOAT:
        //READ_VALUES sortes the values from motor00_values and motor01_values in the values array

	*((float*)(values + 0*size)) = *((float*)(motor00_values+5*size));
	*((float*)(values + 1*size)) = *((float*)(motor00_values+6*size));
	*((float*)(values + 2*size)) = *((float*)(motor00_values+7*size));
	*((float*)(values + 3*size)) = *((float*)(motor00_values+4*size));
	*((float*)(values + 4*size)) = *((float*)(motor00_values+3*size));
	*((float*)(values + 5*size)) = *((float*)(motor01_values+6*size));
	*((float*)(values + 6*size)) = *((float*)(motor01_values+1*size));
	*((float*)(values + 7*size)) = *((float*)(motor01_values+2*size));
	*((float*)(values + 8*size)) = *((float*)(motor01_values+4*size));
	*((float*)(values + 9*size)) = *((float*)(motor01_values+5*size));
	//	READ_VALUES(float); 
        break;
      case TID_BOOL: 
	*((BOOL*)(values + 0*size)) = *((BOOL*)(motor00_values+5*size));
	*((BOOL*)(values + 1*size)) = *((BOOL*)(motor00_values+6*size));
	*((BOOL*)(values + 2*size)) = *((BOOL*)(motor00_values+7*size));
	*((BOOL*)(values + 3*size)) = *((BOOL*)(motor00_values+4*size));
	*((BOOL*)(values + 4*size)) = *((BOOL*)(motor00_values+3*size));
	*((BOOL*)(values + 5*size)) = *((BOOL*)(motor01_values+6*size));
	*((BOOL*)(values + 6*size)) = *((BOOL*)(motor01_values+1*size));
	*((BOOL*)(values + 7*size)) = *((BOOL*)(motor01_values+2*size));
	*((BOOL*)(values + 8*size)) = *((BOOL*)(motor01_values+4*size));
	*((BOOL*)(values + 9*size)) = *((BOOL*)(motor01_values+5*size));
	//READ_VALUES(BOOL);
        break;
      case TID_INT: 
	*((int*)(values + 0*size)) = *((int*)(motor00_values+5*size));
	*((int*)(values + 1*size)) = *((int*)(motor00_values+6*size));
	*((int*)(values + 2*size)) = *((int*)(motor00_values+7*size));
	*((int*)(values + 3*size)) = *((int*)(motor00_values+4*size));
	*((int*)(values + 4*size)) = *((int*)(motor00_values+3*size));
	*((int*)(values + 5*size)) = *((int*)(motor01_values+6*size));
	*((int*)(values + 6*size)) = *((int*)(motor01_values+1*size));
	*((int*)(values + 7*size)) = *((int*)(motor01_values+2*size));
	*((int*)(values + 8*size)) = *((int*)(motor01_values+4*size));
	*((int*)(values + 9*size)) = *((int*)(motor01_values+5*size));
	//READ_VALUES(int);
        break;
    }
  } else {

    // Write data to ODB
    db_set_data(pInfo->hDB, hKey[0], motor00_values, buff_size, 8, type);
    db_set_data(pInfo->hDB, hKey[1], motor01_values, buff_size, 8, type);

  }
}

//-------getPMTpolygon-----------------------------------------//
// Function used in move_init to initialise the PMT position for collision avoidance calculations
// 
// Parameters
//     int polyNum: number to identify the polygon modeling the PMT cross-section at a specific z height;
//                  1 == top-most PMT polygon
//     int numPolySides: number of sides the polygon has; more sides = better approximation of circular x-section.
//     double polyHeight: thickness of each polygon; thinner = better approximation of sphere.
//
// Author: Rika
// Date: 17 March 2017
// Modified: 27 April 2017 - included the FRP radius for collision avoidance with the FRP rim.

void
getPMTpolygon(XYPolygon &poly, double pmtRadius, int polyNum, double pmtXcentre, double pmtYcentre, int numPolySides,
              double polyHeight) {
  const double PI = 3.14159265358979323846;
  double xFromCentre;
  double yFromCentre;
  double zFromCentre = pmtRadius - polyHeight * polyNum;
  double radiusOfEnclosingCircle =
      sqrt(pmtRadius * pmtRadius - zFromCentre * zFromCentre) / cos(PI / numPolySides) + 0.001;
  // EnclosingCircle is the circle that encloses the polygon at the z-height,
  // without the polygon intersecting the PMT cross-section
  // Rika (23Mar2017): added 0.001m to round up.

  // Rika (27April2017): added for collision avoidance with FRP rim.
  if (zFromCentre <= (pmtHeight + pmtRadius) - frpHeight) {
    // If the z height is below the rim on which the acrylic cover sits,
    // make the polygon so that it encloses the rim (i.e. the optical box should never slide under the FRP).
    radiusOfEnclosingCircle = frpRadius / cos(PI / numPolySides) + 0.001;
  }

  for (double theta = 0.0; theta < 2 * PI; theta += 2 * PI / numPolySides) {
    xFromCentre = radiusOfEnclosingCircle * cos(theta);
    yFromCentre = radiusOfEnclosingCircle * sin(theta);
    poly.push_back(std::make_pair(xFromCentre + pmtXcentre, yFromCentre + pmtYcentre));
    //cm_msg(MINFO, "generate_path", " PMT polygon points = %.3f %.3f", xFromCentre + pmtXcentre, yFromCentre + pmtYcentre);
  }

  // Insert last point to close the polygon
  xFromCentre = radiusOfEnclosingCircle * cos(0.0);
  yFromCentre = radiusOfEnclosingCircle * sin(0.0);
  poly.push_back(std::make_pair(xFromCentre + pmtXcentre, yFromCentre + pmtYcentre));

}