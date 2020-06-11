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
#include <stdlib.h>
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

/* make frontend functions callable from the C framework            */
#ifdef __cplusplus
extern "C" {
#endif

/*-- Globals -------------------------------------------------------*/
// The two function-like macros below are used in void channel_rw(INFO *pInfo, HNDLE* hKey, void *values, DWORD type, BOOL rw);
//Note:: See the funciton channel_rw for more information about these macros

//The below preprocessor macro would only work if the "function like" macro was defined on a single line. An identical version of the marco is put in the comment below below but is properly spaced to make it possible to read the code
#define READ_VALUES(TYPE){*((TYPE*)(values + 0*size)) = *((TYPE*)(motor00_values+5*size));*((TYPE*)(values + 1*size)) = *((TYPE*)(motor00_values+6*size));*((TYPE*)(values + 2*size)) = *((TYPE*)(motor00_values+7*size));*((TYPE*)(values + 3*size)) = *((TYPE*)(motor00_values+4*size));*((TYPE*)(values + 4*size)) = *((TYPE*)(motor00_values+3*size));*((TYPE*)(values + 5*size)) = *((TYPE*)(motor01_values+6*size));*((TYPE*)(values + 6*size)) = *((TYPE*)(motor01_values+1*size));*((TYPE*)(values + 7*size)) = *((TYPE*)(motor01_values+2*size));*((TYPE*)(values + 8*size)) = *((TYPE*)(motor01_values+4*size));*((TYPE*)(values + 9*size)) = *((TYPE*)(motor01_values+5*size));}

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
#define STORE_DATA(TYPE) {TYPE motor00_TYPE_values[8];TYPE motor01_TYPE_values[8];buff_size=sizeof(motor00_TYPE_values);size = sizeof(TYPE);motor00_TYPE_values[0]= 0;motor00_TYPE_values[1]= 0;motor00_TYPE_values[2]= 0;motor00_TYPE_values[3]=*((TYPE*)(values+4*size));motor00_TYPE_values[4]=*((TYPE*)(values+3*size));motor00_TYPE_values[5]=*((TYPE*)(values+0*size));motor00_TYPE_values[6]=*((TYPE*)(values+1*size));motor00_TYPE_values[7]=*((TYPE*)(values+2*size));motor01_TYPE_values[0]= 0;motor01_TYPE_values[1]=*((TYPE*)(values+6*size));motor01_TYPE_values[2]=*((TYPE*)(values+7*size));motor01_TYPE_values[3]=0;motor01_TYPE_values[4]=*((TYPE*)(values+8*size));motor01_TYPE_values[5]=*((TYPE*)(values+9*size));motor01_TYPE_values[6]=*((TYPE*)(values+5*size));motor01_TYPE_values[7]= 0;motor00_values = motor00_TYPE_values;motor01_values = motor01_TYPE_values;}

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
INT tilt_ini_attempts = 0;

/* Optionally only control one of two motors */
INT gantry_motor_start = 0;   //for for-loops (by default from motor 0 to motor 9)
INT gantry_motor_end = 10;   //TF TODO: get from ODB and make button on gantry_move page. Only use when one is in repair though!!!


/* Rika (20Apr2017): MOVED GLOBAL CALCULATOR OBJECTS BACK TO GENERATE_PATH() */
// Gantry dimensions with buffer zones for collision avoidance
double tiltMotorLength = 0.160;
double gantryFrontHalfLength = 0.140; //Use 0.140m for safety; measured to be 0.114+/-0.001m (27Apr2017)
double gantryBackHalfLength = 0.220; //Use 0.200m for safety; measured to be 0.114+0.07(pipelength)=0.184+/-0.002m (27Apr2017) // John (17Oct2019) 0.185 -> 0.22 for safety
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

INT interrupt_configure(INT cmd, PTYPE adr);

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

/*-- Interrupt configuration for trigger event ---------------------*/

INT interrupt_configure(INT cmd, PTYPE adr) {
  return CM_SUCCESS;
}

/*-- Event readout -------------------------------------------------*/
INT read_trigger_event(char *pevent, INT off) {
  return 0;
}

/*-- Scaler event --------------------------------------------------*/

INT read_scaler_event(char *pevent, INT off) {
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
  db_find_key(hDB, 0, "/Equipment/Phidget00/Variables", &pInfo->hKeyPhidget[0]);
  db_find_key(hDB, 0, "/Equipment/Phidget01/Variables", &pInfo->hKeyPhidget[1]);

  /* Set up hotlinks */
  // move_init() hotlink
  db_open_record(hDB, pInfo->hKeyStart, &pInfo->Start, sizeof(BOOL), MODE_READ, move_init, pInfo);

  // stop_move() hotlink
  db_open_record(hDB, pInfo->hKeyStop, &pInfo->Stop, sizeof(BOOL), MODE_READ, stop_move, pInfo);

  // reinitialize() hotlink
  db_open_record(hDB, pInfo->hKeyReInit, &pInfo->ReInitialize, sizeof(BOOL), MODE_READ, reinitialize, pInfo);

  // monitor() hotlink
  db_open_record(hDB, pInfo->hKeyMMoving[0], NULL, 8 * sizeof(BOOL), MODE_READ, monitor, pInfo);
  db_open_record(hDB, pInfo->hKeyMMoving[1], NULL, 8 * sizeof(BOOL), MODE_READ, monitor, pInfo);
  // Note: The variable hotlinked to monitor() is somewhat arbitrary, all that is 
  //	   really needed is a variable that gets periodically updated in feMove.

  /* Set motor velocities and accelerations to in feMotor */
  for (i = gantry_motor_start; i < gantry_motor_end; i++) {
    tempV[i] = pInfo->Velocity[i] * fabs(pInfo->mScale[i]);
    tempA[i] = pInfo->Acceleration[i] * fabs(pInfo->mScale[i]);
  }

  channel_rw(pInfo, pInfo->hKeyMVel, (void *) tempV, TID_FLOAT, 1);
  channel_rw(pInfo, pInfo->hKeyMAcc, (void *) tempA, TID_FLOAT, 1);
  return CM_SUCCESS;
}

/*-- Frontend Exit -------------------------------------------------*/
// Stop the motors and free all memory in this function
INT frontend_exit() {
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

/*=====================Phidget Check=================================*/
// Check to see if phidgets are responsive
INT phidget_responding(HNDLE hDB) {
  double tilt_min = -105, tilt_max = 15;
  HNDLE hPhidgetVars0 = 0, hPhidgetVars1 = 0;
  double phidget_Values_Old[10];
  double phidget_Values_Now[10];
  int size_of_array = sizeof(phidget_Values_Old);
  DWORD time_at_start_of_check;

  //cm_msg(MINFO, "move_init", "Checking Phidget Response");
  size_of_array = sizeof(phidget_Values_Old);
  db_get_value(hDB, hPhidgetVars0, "PH00", &phidget_Values_Old, &size_of_array, TID_DOUBLE, FALSE);
  db_get_value(hDB, hPhidgetVars0, "PH00", &phidget_Values_Now, &size_of_array, TID_DOUBLE, FALSE);

  //Check that phidget0 is working. The phidget is working if the values change over a set period of time. When a phidget is unplugged there is no fluctuation in the values.
  time_at_start_of_check = ss_millitime();
  while ((phidget_Values_Old[0] == phidget_Values_Now[0]) && (phidget_Values_Old[1] == phidget_Values_Now[1]) &&
         (phidget_Values_Old[2] == phidget_Values_Now[2]) && (phidget_Values_Old[3] == phidget_Values_Now[3]) &&
         (phidget_Values_Old[4] == phidget_Values_Now[4]) && (phidget_Values_Old[5] == phidget_Values_Now[5]) &&
         (phidget_Values_Old[6] == phidget_Values_Now[6]) && (phidget_Values_Old[7] == phidget_Values_Now[7]) &&
         (phidget_Values_Old[8] == phidget_Values_Now[8]) && (phidget_Values_Old[9] == phidget_Values_Now[9])) {

    db_get_value(hDB, hPhidgetVars0, "PH00", &phidget_Values_Now, &size_of_array, TID_DOUBLE, FALSE);

    if (phidget_Values_Now[7] > tilt_max || phidget_Values_Now[7] < tilt_min) {
      cm_msg(MERROR, "initialize_tilt", "gantry00 exceeded tilt limits while initializing. Currently at: %f",
             phidget_Values_Now[7]);
      return 0;
    }

    // If 5 seconds pass without any of the values changing assume that the phidgets are not connected properly
    if (ss_millitime() - time_at_start_of_check > 1000 * 5) {
      printf("The readings from Phidget0 are not changing so it is assumed that Phidget0 is not working\n");
      //return DB_NO_ACCESS ;  //TODO : only comment if know that Phidget IS unplugged!
      return 0;
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

    if (phidget_Values_Now[7] > tilt_max || phidget_Values_Now[7] < tilt_min) {
      cm_msg(MERROR, "initialize_tilt", "gantry01 exceeded tilt limits while initializing. Currently at: %f",
             phidget_Values_Now[7]);
      return 0;
    }
    // If 5 seconds pass without any of the values changing assume that the phidgets are not connected properly
    if (ss_millitime() - time_at_start_of_check > 1000 * 5) {
      cm_msg(MERROR, "begin_of_run",
             "The readings from Phidget1 are not changing so it is assumed that Phidget1 is not working");
      //return DB_NO_ACCESS ; //TODO FEB 28 2015: reset !!!
      return 0;
    }

    ss_sleep(1000);
  }


  return 1;
}

/*-- Move Init -----------------------------------------------------*/
// Call generate path and start the first move
void move_init(HNDLE hDB, HNDLE hKey, void *data) {
  INFO *pInfo = (INFO *) data;
  // BOOL Check_Phidgets = TRUE;
  // HNDLE hPhidgetVars0 = 0, hPhidgetVars1 = 0;
  double phidget_Values_Old[10];
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

  cm_msg(MDEBUG, "move_init", "Checking phidget response...");
  if (!phidget_responding(hDB)) {
    return;
  }

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
  size = sizeof(pInfo->Phidget);
  int tilt_start = 0, tilt_end = 2; //these are the number of gantries we have to correct
  for (int i = tilt_start; i < tilt_end; i++) {
    int axis = (i == 0 ? 4 : 9);
    db_get_value(pInfo->hDB, pInfo->hKeyPhidget[0], "PH00", &pInfo->Phidget, &size, TID_DOUBLE, FALSE);
    if (i == 1) { db_get_value(pInfo->hDB, pInfo->hKeyPhidget[1], "PH01", &pInfo->Phidget, &size, TID_DOUBLE, FALSE); }

    // DEBUG
    //cm_msg(MINFO, "move_init", "Phidget0%i tilt: %f ODB tilt: %f", i, pInfo->Phidget[7], pInfo->Position[axis]);

    // Check if  phidget tilt readings match ODB values
    if (pInfo->Position[axis] < pInfo->Phidget[7] - tilt_tolerance ||
        pInfo->Position[axis] > pInfo->Phidget[7] + tilt_tolerance) {
      cm_msg(MERROR, "move_init", "Phidget0%i tilt: %f ODB tilt: %f", i, pInfo->Phidget[7], pInfo->Position[axis]);
      cm_msg(MERROR, "move_init", "ERROR: can't start move. Phidget0%i tilt and ODB tilt do not agree within tolerance",
             i);
      return;
    }

    // Check if phidget reading is within legal range
    if (pInfo->Phidget[7] < tilt_min || pInfo->Phidget[7] > tilt_max) {
      cm_msg(MERROR, "move_init", "ERROR: can't start move. Phidget0%i tilt: %f is in illegal range", i,
             pInfo->Phidget[7]);
      return;
    }
  }

  // Load input destination into pInfo->Destination
  size = 10 * sizeof(float);
  db_get_data(hDB, pInfo->hKeyDest, pInfo->Destination, &size, TID_FLOAT);

  // Generate collision free path to pInfo->Destination
  cm_msg(MINFO, "move_init", "Generating Path");
  int Status = generate_path(pInfo);

  // Check for unsuccesful path generation
  switch (Status) {
    case GENPATH_SUCCESS:
      pInfo->BadDest = 0;
      db_set_data(hDB, pInfo->hKeyBadDest, &pInfo->BadDest, sizeof(BOOL), 1, TID_BOOL);
      break;
    case GENPATH_BAD_DEST:
      cm_msg(MERROR, "move_init", " Bad destination entered in Move");
      pInfo->BadDest = 1;
      db_set_data(hDB, pInfo->hKeyBadDest, &pInfo->BadDest, sizeof(BOOL), 1, TID_BOOL);
      return;
  }
  cm_msg(MINFO, "move_init", "Path succesfully generated");

  // Set the "completed" variable in the ODB to 0 (since our newly
  // started move is still incomplete)
  pInfo->Completed = 0;
  db_set_data(hDB, pInfo->hKeyCompleted, &pInfo->Completed, sizeof(BOOL), 1, TID_BOOL);

  // Move to first path index
  move(pInfo);


  // Set the ODB variable "Start Move" back to 0
  pInfo->Start = 0;
  db_set_data(hDB, pInfo->hKeyStart, &pInfo->Start, sizeof(BOOL), 1, TID_BOOL);
}

/*-- Initialize tilt ----------------------------------------------------*/
// We initialize the tilt motor by using the tilt measurement from the 1044 Phidget
// accelerometer.
int initialize_tilt(INFO *pInfo) {

  int i;
  double tolerance = 1.; //3.0;
  float *dest = (float *) calloc(10, sizeof(float));
  BOOL *start = (BOOL *) calloc(10, sizeof(BOOL));
  int size = sizeof(pInfo->Phidget);

  // BOOL Check_Phidgets = TRUE;
  double phidget_Values_Old[10];
  // double phidget_Values_Now[10];
  // int size_of_array = sizeof(phidget_Values_Old);
  // DWORD time_at_start_of_check;
  // HNDLE hPhidgetVars0 = 0, hPhidgetVars1 = 0;

  // double tilt_min = -105;
  // double tilt_max = 15;
  //Only loop over the motors from the 
  //gantry we want to use
  int tilt_start = 0;
  int tilt_end = 2;
  if (gantry_motor_start == 5) {
    tilt_start = 1;
  } else if (gantry_motor_end == 5) {
    tilt_end = 1;
  }

  if (!phidget_responding(pInfo->hDB)) {
    return -1;
  }

  for (i = tilt_start; i < tilt_end; i++) {

    int axis = (i == 0 ? 4 : 9);
    // TF TEMP
    //if (axis == 9)
    //  continue;

    //Get readings of phidget. 
    int status = db_get_value(pInfo->hDB, pInfo->hKeyPhidget[0], "PH00", &pInfo->Phidget, &size, TID_DOUBLE, FALSE);
    if (i == 1) {
      status = db_get_value(pInfo->hDB, pInfo->hKeyPhidget[1], "PH01", &pInfo->Phidget, &size, TID_DOUBLE, FALSE);
    }
    if (status != DB_SUCCESS) {
      cm_msg(MERROR, "initialize_tilt", "Cannot get value for Phidget %i Reading", i);
      return DB_NO_ACCESS;
    }

    //Check that the phidget is connected by checking that its internal clock is changing   
    int time_s = pInfo->Phidget[8];
    int time_us = pInfo->Phidget[9];

    // Make sure phidget readings change from previous value:
    sleep(2000); //2sec
    if (i == 0) db_get_value(pInfo->hDB, pInfo->hKeyPhidget[0], "PH00", &pInfo->Phidget, &size, TID_DOUBLE, FALSE);
    else db_get_value(pInfo->hDB, pInfo->hKeyPhidget[1], "PH01", &pInfo->Phidget, &size, TID_DOUBLE, FALSE);


    //TF note: this is not 100% reliable...even with a big sleep : 
    // DEBUG!!!! TODO
    if (time_s == pInfo->Phidget[8] && time_us == pInfo->Phidget[9]) {
      printf("Time not updated; fePhidget not running; can't initialize tilt\n");
      printf("%i %f %i %f\n", time_s, pInfo->Phidget[8], time_us, pInfo->Phidget[9]);
      //return 0; //TF: uncomment for Tilt tests...
      //TF: for running tilt without phidgets:
      //pInfo->Phidget[7] = -10;
    }

    if (i == 0)
      printf("\n");
    cm_msg(MINFO, "move_init", "Tilt %i initial position (from Phidget): tilt angle = %f", i, pInfo->Phidget[7]);


    // Now we estimate the distance we need in order to get the tilt to zero.
    dest[axis] = -(pInfo->Phidget[7] * pInfo->mScale[axis]);

    // Only activate the tilt!
    start[axis] = 1;

  }

  //DEBUG:
  printf("Destinations: Dest[4] = %f, dest[9] = %f\n", dest[4], dest[9]);

  // Write the destinations to the motors
  // Note: channel_rw works for both motors, ie. write both tilt destinations at once and initialize
  // together
  channel_rw(pInfo, pInfo->hKeyMDest, dest, TID_FLOAT, 1);

  // Start the move
  channel_rw(pInfo, pInfo->hKeyMStart, (void *) start, TID_BOOL, 1);
  sleep(600); // TF and BK: is this to wait for galil_read to update ODB for AxisMoving??


  // Monitor the motion until we are finished moving.
  pInfo->Moving = 1;
  bool startedMoving = false;
  // bool phidget0updating = true;
  // bool phidget1updating = true;
  while (pInfo->Moving) {
    pInfo->Moving = 0;

    //TF note: this only checks whether the motors are moving!!
    channel_rw(pInfo, pInfo->hKeyMMoving, (void *) pInfo->AxisMoving, TID_BOOL, 0);
    pInfo->Moving = pInfo->Moving || pInfo->AxisMoving[4];
    pInfo->Moving = pInfo->Moving || pInfo->AxisMoving[9];
    if (pInfo->Moving && !startedMoving)
      startedMoving = true;

    if (pInfo->Moving && !phidget_responding(pInfo->hDB)) {
      return -1;
    }
  }

  cm_msg(MINFO, "initialize_tilt", "Move complete.");
  tilt_ini_attempts++;
  for (i = tilt_start; i < tilt_end; i++) {
    int axis = (i == 0 ? 4 : 9);

    // Check that the final tilt is within the required tolerance
    if (i == 0) db_get_value(pInfo->hDB, pInfo->hKeyPhidget[0], "PH00", &pInfo->Phidget, &size, TID_DOUBLE, FALSE);
    else db_get_value(pInfo->hDB, pInfo->hKeyPhidget[1], "PH01", &pInfo->Phidget, &size, TID_DOUBLE, FALSE);

    //NaN check:
    if (pInfo->Phidget[7] != pInfo->Phidget[7]) {
      cm_msg(MERROR, "initialize_tilt", "NAN tilt value, either not connected or bug in tilt angle calculation!");
      continue;
    }

    if (fabs(pInfo->Phidget[7]) < tolerance) {
      cm_msg(MINFO, "initialize_tilt", "Tilt %i = %.2f deg, Acceleration(a_x = %.4f, a_y = %.4f, a_z= %.4f)", i,
             pInfo->Phidget[7], pInfo->Phidget[0], pInfo->Phidget[1], pInfo->Phidget[2]);
    } else {
      cm_msg(MERROR, "initialize_tilt",
             "Tilt initialization (axis %i) failed; final tilt = %f is not within zero to within required tolerance of %f degrees",
             axis, pInfo->Phidget[7], tolerance);
      // --> for debugging: switch the initialize_tilt ON and uncomment the return
      // printf("Attempting initialization again");
      if (tilt_ini_attempts < 4)
        initialize_tilt(pInfo);

      if (!startedMoving)
        cm_msg(MERROR, "initialize_tilt", "Tilt motors didn't even start moving");
      else
        cm_msg(MERROR, "initialize_tilt",
               "Tilt motors didn't move the optical box (sufficiently enough). Check the grip of the gears or the motor scaling!");
      //return -1;

    }

    // Reset origin position.
    channel_rw(pInfo, pInfo->hKeyMPos, pInfo->CountPos, TID_FLOAT, 0);
    pInfo->mOrigin[axis] = pInfo->CountPos[axis] - pInfo->LimPos[axis] * pInfo->mScale[axis];
    pInfo->Position[axis] = pInfo->LimPos[axis];

    cm_msg(MINFO, "initialize_tilt", "Finished initializing axis %i; Origin = %5.2f counts, Position = %5.2f deg", axis,
           pInfo->mOrigin[axis], pInfo->Position[axis]);

  }//end of loop over both gantries

  return 0;

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
      sleep(600); // Approx. polling period
      if ((tempNegLimitEnabled[i] == 1) && (tempNegLimitEnabled[i + 5] == 1)) {// If both gantry axes are enabled.
        cm_msg(MDEBUG, "initialize", "Polling axes %i and %i.", i, i+5);
        channel_rw(pInfo, pInfo->hKeyMLimitNeg, (void *) pInfo->neg_AxisLimit, TID_BOOL, 0);
        if (pInfo->neg_AxisLimit[i] && pInfo->neg_AxisLimit[i + 5]) break;
        else {
          channel_rw(pInfo, pInfo->hKeyMPos, pInfo->CountPos, TID_FLOAT, 0);
          if (pInfo->CountPos[i] == lastCountPosArm1 && !pInfo->neg_AxisLimit[i]) {
            cm_msg(MERROR, "initialize",
                   "Axis %i not moving. Stopping initialization since limit switch must be broken.", i);
            channel_rw(pInfo, pInfo->hKeyMStop, (void *) tempStop, TID_BOOL, 1);
            exitFlag = 1;
            break;
          }
          if (pInfo->CountPos[i + 5] == lastCountPosArm2 && !pInfo->neg_AxisLimit[i + 5]) {
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

  int exitFlagTilt = initialize_tilt(pInfo);


  if (exitFlag == 0 && exitFlagTilt == 0) {
    // Set Initialized to 1 and exit    
    pInfo->Initialized = 1;
    db_set_data(pInfo->hDB, pInfo->hKeyInit, &pInfo->Initialized, sizeof(BOOL), 1, TID_BOOL);
  }

  initing = 0;
  db_set_data(pInfo->hDB, pInfo->hKeyInitializing, &initing, sizeof(BOOL), 1, TID_BOOL);

  sleep(100);
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
// Shaun: Constrained boundaries of box by adding rotation & tilt dependance
// 		-gantry 1 & 2 have 1-2 cm safety margin
// 		-the eight possible permutations that system can undertake are:
//                    1) gantry 1 move first, rotation first, tilt first;
// 										2) gantry 1 move first, rotation second, tilt first; 
// 										3) gantry 2 move first, rotation first, tilt first; 
// 										4) gantry 2 move first, rotation second, tilt first
// 										5) gantry 1 move first, rotation first, tilt second; 
// 		                6) gantry 1 move first, rotation second, tilt second;
// 		                7) gantry 2 move first, rotation first, tilt second;
// 		                8) gantry 2 move first, rotation second, tilt second
//
// 		-the system will move in Z-direction first if the endpoint z is outside the water tank
//		-paths are calculated for the 8 possible permutations to see if any are possible w/o collision
//		-for permutations 1-4 tilt is completed at beginning of run, for 5-8 tilt is completed at end of run 
//				-1) w/ gantry 2 in start position and final rotation, gantry 1 in rotated position can move from start to destination (TPathCalculator)
//				    w/ gantry 1 at destination and final rotation, gantry 2 in rotated position can move from start to destination (TPathCalculator)
//				    gantry 1 & 2 can rotate from initial rotation at start position to final location at start position (TRotationCalculator)
//				-2) w/ gantry 2 in start position and initial rotation, gantry 1 in initial rotated position can move from start to destination (TPathCalculator)
//				    w/ gantry 1 at destination and initial rotation, gantry 2 in initial rotated position can move from start to destination (TPathCalculator)
//				    gantry 1 & 2 can rotate from initial rotation at destination to final rotation at destination (TRotationCalculator)
//				-3) w/ gantry 1 in start position and final rotation, gantry 2 in rotated position can move from start to destination (TPathCalculator)
//				    w/ gantry 2 at destination and final rotation, gantry 1 in rotated position can move from start to destination (TPathCalculator)
//				    gantry 1 & 2 can rotate from initial rotation at start position to final location at start position (TRotationCalculator)
//				-4) w/ gantry 1 in start position and initial rotation, gantry 2 in initial rotated position can move from start to destination (TPathCalculator)
//				    w/ gantry 2 at destination and initial rotation, gantry 1 in initial rotated position can move from start to destination (TPathCalculator)
//				    gantry 1 & 2 can rotate from initial rotation at destination to final rotation at destination (TRotationCalculator)
//		-to ensure intermediate roation positions will not collide with other gantry or water tank (in case where initial and final rotation is good but intermediate value is not) TRotationCalculator is constructed
int generate_path(INFO *pInfo) {

  // Check for illegal destinations.  Currently just check:
  // rot_min < rotary angle < rot_max
  double rot_min = -100, rot_max = 120; //Rotation limited to values equal to or greater than -100 degrees
  //to avoid triggering limit switch during scan; updated Feb 28, 2017
  // tilt_min < tilt angle < tilt_max
  double tilt_min = -105, tilt_max = 15;

  double z_max_value = 0.22; // Rika (23Mar2017): gantry positive z limit switch at z = 0.534m.
  // John (16Oct2019): Reducing z_max from 0.535 to 0.22 for PMT scans because getting too close to acrylic

  // double safeZheight = 0.260; // Rika (4Apr2017): z height at which any movement (rot & tilt) is PMT collision free.
  // (24Apr2017) updated safeZheight for new PMT position (previously 0.46m)

  double tankHeight = 0.05; //actually 0.08 but play safe.

  bool move_second_gantry_first = false;
  bool move_z1_first = false;
  bool move_z2_first = false;
  bool move_rotation_first = false;
  bool gant2_movefirst = false;
  bool tiltfirst = false;
  const double pi = boost::math::constants::pi<double>();

  double gantry1XPos = pInfo->Position[0];
  double gantry1YPos = pInfo->Position[1];
  double gantry1ZPos = pInfo->Position[2];
  double gantry2XPos = pInfo->Position[5];
  double gantry2YPos = pInfo->Position[6];
  double gantry2ZPos = pInfo->Position[7];
  double gantry1XDes = pInfo->Destination[0];
  double gantry1YDes = pInfo->Destination[1];
  double gantry1ZDes = pInfo->Destination[2];
  double gantry2XDes = pInfo->Destination[5];
  double gantry2YDes = pInfo->Destination[6];
  double gantry2ZDes = pInfo->Destination[7];

  // both gantries can move in different Z layers, eg. g0 above tank and g1 in tank
  // therefore we need different tank boundary conditions for each gantry
  // double containerXDimensions[4];
  // double containerYDimensions[4];


  // For rotation & tilt, need to consider both initial and final states of system -- as rotation & tilt may affect whether gantry collides with tank or other gantry
  double rad = pi / 180;

  double gant1_rot1 = pInfo->Destination[3] * rad;
  double gant2_rot1 = pInfo->Destination[8] * rad;
  double gant1_rot2 = pInfo->Position[3] * rad;
  double gant2_rot2 = pInfo->Position[8] * rad;
  double gant1_tilt_end = pInfo->Position[4] * rad;
  double gant2_tilt_end = pInfo->Position[9] * rad;
  double gant1_tilt_start = pInfo->Destination[4] * rad;
  double gant2_tilt_start = pInfo->Destination[9] * rad;

  /**NEW**///Rika(31Mar2017)
  // Stores z-height from gantry to lowest point on optical box for different tilt.
  // index 0 = tilt before movement in x & y, index 1 = tilt after movement in x & y.
  std::pair<double, double> opticalBox0_height[2];
  std::pair<double, double> opticalBox1_height[2];
  // Stores z-heights relative to the PMT polygon layers
  //i.e ((gantryZ + opticalBox_height) - (pmtHeight))/pmtPolyLayerHeight
  int opticalBox0_lo_Z[2];
  int opticalBox0_up_Z[2];
  int opticalBox1_lo_Z[2];
  int opticalBox1_up_Z[2];

  // determine whether boxes are inside tank or above tank
  bool tankheight_gantstart1 = 0;
  if (pInfo->Position[2] < tankHeight) tankheight_gantstart1 = 1;
  bool tankheight_gantstart2 = 0;
  if (pInfo->Position[7] < tankHeight) tankheight_gantstart2 = 1;
  std::pair<bool, bool> start_tank = std::make_pair(tankheight_gantstart1, tankheight_gantstart2);

  // TODO: make this code more elegant as it just changes by i+5
  // This either means I will move up (so z moves first) or I'm moving above the tank
  // conservative outer "tank" dimensions - not really necessary b/c of limit switches
  /*containerYDimensions[0] = -0.205;
  containerXDimensions[0] = -0.05;
  containerYDimensions[1] = 0.735;
  containerXDimensions[1] = -0.05;
  containerYDimensions[2] = 0.735;
  containerXDimensions[2] = 1.05;
  containerYDimensions[3] = -0.205;
  containerXDimensions[3] = 1.05;*/
  bool tankheight_gantend1 = 0;
  if (pInfo->Destination[2] < tankHeight) tankheight_gantend1 = 1;
  bool tankheight_gantend2 = 0;
  if (pInfo->Destination[7] < tankHeight) tankheight_gantend2 = 1;
  std::pair<bool, bool> end_tank = std::make_pair(tankheight_gantend1, tankheight_gantend2);


  // Possible permutations of start/end positions and rotations of Gantry & Optical Boxes

  // Rika:
  // NOTE - Looking at the gantry from above, with the laser aperture as the front,
  // the four corners of the gantry area are defined by the gantryDimensions array index as follows:
  //     0 := front right point
  //     1 := front left point 
  //     2 := back left point 
  //     3 := back right point 
  // NOTE: front most points of gantry area are defined by the tiltMotorlength (& gantryWidth)
  //       since tiltMotorLength > gantryLength_FrontHalf no matter what the tilt.
  //       This is why it is necessary to calculate the optical box configuration separately for more precise
  //       collision avoidance control with PMT (see getOpticalBoxConfig below).

  //Note: typedef std::vector<std::pair<double, double> > XYPolygon; defined in TPathCalculator.hxx
  // Gantry
  XYPolygon gantry1_startpos_rotfirst_tiltfirst;
  XYPolygon gantry1_startpos_rotsecond_tiltfirst;
  XYPolygon gantry1_endpos_rotfirst_tiltfirst;
  XYPolygon gantry1_endpos_rotsecond_tiltfirst;
  XYPolygon gantry2_startpos_rotfirst_tiltfirst;
  XYPolygon gantry2_startpos_rotsecond_tiltfirst;
  XYPolygon gantry2_endpos_rotfirst_tiltfirst;
  XYPolygon gantry2_endpos_rotsecond_tiltfirst;
  XYPolygon gantry1_startpos_rotfirst_tiltsecond;
  XYPolygon gantry1_startpos_rotsecond_tiltsecond;
  XYPolygon gantry1_endpos_rotfirst_tiltsecond;
  XYPolygon gantry1_endpos_rotsecond_tiltsecond;
  XYPolygon gantry2_startpos_rotfirst_tiltsecond;
  XYPolygon gantry2_startpos_rotsecond_tiltsecond;
  XYPolygon gantry2_endpos_rotfirst_tiltsecond;
  XYPolygon gantry2_endpos_rotsecond_tiltsecond;

  std::pair <XYPolygon, XYPolygon> box0_startpos_rotfirst_tiltfirst;
  std::pair <XYPolygon, XYPolygon> box0_startpos_rotsecond_tiltfirst;
  std::pair <XYPolygon, XYPolygon> box0_endpos_rotfirst_tiltfirst;
  std::pair <XYPolygon, XYPolygon> box0_endpos_rotsecond_tiltfirst;
  std::pair <XYPolygon, XYPolygon> box1_startpos_rotfirst_tiltfirst;
  std::pair <XYPolygon, XYPolygon> box1_startpos_rotsecond_tiltfirst;
  std::pair <XYPolygon, XYPolygon> box1_endpos_rotfirst_tiltfirst;
  std::pair <XYPolygon, XYPolygon> box1_endpos_rotsecond_tiltfirst;
  std::pair <XYPolygon, XYPolygon> box0_startpos_rotfirst_tiltsecond;
  std::pair <XYPolygon, XYPolygon> box0_startpos_rotsecond_tiltsecond;
  std::pair <XYPolygon, XYPolygon> box0_endpos_rotfirst_tiltsecond;
  std::pair <XYPolygon, XYPolygon> box0_endpos_rotsecond_tiltsecond;
  std::pair <XYPolygon, XYPolygon> box1_startpos_rotfirst_tiltsecond;
  std::pair <XYPolygon, XYPolygon> box1_startpos_rotsecond_tiltsecond;
  std::pair <XYPolygon, XYPolygon> box1_endpos_rotfirst_tiltsecond;
  std::pair <XYPolygon, XYPolygon> box1_endpos_rotsecond_tiltsecond;

  // NEW CODE USING TGANTRYCONFIGCALCULATOR

  // Rika (27Mar2017): 
  // Get possible gantry configurations using TGantryConfigCalculator::GetGantryConfig()

  // Tilt first
  // Final state of gantry0 & gantry1, after rotation with XY limit corners with tilt dependence
  gantry1_startpos_rotfirst_tiltfirst = gantryConfigCalc.GetGantryConfig(0, gant1_rot1, gant1_tilt_start, gantry1XPos,
                                                                         gantry1YPos);
  gantry1_startpos_rotsecond_tiltfirst = gantryConfigCalc.GetGantryConfig(0, gant1_rot2, gant1_tilt_start, gantry1XPos,
                                                                          gantry1YPos);
  gantry1_endpos_rotfirst_tiltfirst = gantryConfigCalc.GetGantryConfig(0, gant1_rot1, gant1_tilt_start, gantry1XDes,
                                                                       gantry1YDes);
  gantry1_endpos_rotsecond_tiltfirst = gantryConfigCalc.GetGantryConfig(0, gant1_rot2, gant1_tilt_start, gantry1XDes,
                                                                        gantry1YDes);
  box0_startpos_rotfirst_tiltfirst = gantryConfigCalc.GetOpticalBoxConfig(0, gant1_rot1, gant1_tilt_start, gantry1XPos,
                                                                          gantry1YPos);
  box0_startpos_rotsecond_tiltfirst = gantryConfigCalc.GetOpticalBoxConfig(0, gant1_rot2, gant1_tilt_start, gantry1XPos,
                                                                           gantry1YPos);
  box0_endpos_rotfirst_tiltfirst = gantryConfigCalc.GetOpticalBoxConfig(0, gant1_rot1, gant1_tilt_start, gantry1XDes,
                                                                        gantry1YDes);
  box0_endpos_rotsecond_tiltfirst = gantryConfigCalc.GetOpticalBoxConfig(0, gant1_rot2, gant1_tilt_start, gantry1XDes,
                                                                         gantry1YDes);
  // Final state of gantry0 & gantry1, before rotation with XY limit corners with tilt dependence
  gantry2_startpos_rotfirst_tiltfirst = gantryConfigCalc.GetGantryConfig(1, gant2_rot1, gant2_tilt_start, gantry2XPos,
                                                                         gantry2YPos);
  gantry2_startpos_rotsecond_tiltfirst = gantryConfigCalc.GetGantryConfig(1, gant2_rot2, gant2_tilt_start, gantry2XPos,
                                                                          gantry2YPos);
  gantry2_endpos_rotfirst_tiltfirst = gantryConfigCalc.GetGantryConfig(1, gant2_rot1, gant2_tilt_start, gantry2XDes,
                                                                       gantry2YDes);
  gantry2_endpos_rotsecond_tiltfirst = gantryConfigCalc.GetGantryConfig(1, gant2_rot2, gant2_tilt_start, gantry2XDes,
                                                                        gantry2YDes);
  box1_startpos_rotfirst_tiltfirst = gantryConfigCalc.GetOpticalBoxConfig(1, gant2_rot1, gant2_tilt_start, gantry2XPos,
                                                                          gantry2YPos);
  box1_startpos_rotsecond_tiltfirst = gantryConfigCalc.GetOpticalBoxConfig(1, gant2_rot2, gant2_tilt_start, gantry2XPos,
                                                                           gantry2YPos);
  box1_endpos_rotfirst_tiltfirst = gantryConfigCalc.GetOpticalBoxConfig(1, gant2_rot1, gant2_tilt_start, gantry2XDes,
                                                                        gantry2YDes);
  box1_endpos_rotsecond_tiltfirst = gantryConfigCalc.GetOpticalBoxConfig(1, gant2_rot2, gant2_tilt_start, gantry2XDes,
                                                                         gantry2YDes);
  //Tilt second
  //Final state of gantry0 & gantry1, after rotation with XY limit corners with tilt dependence
  gantry1_startpos_rotfirst_tiltsecond = gantryConfigCalc.GetGantryConfig(0, gant1_rot1, gant1_tilt_end, gantry1XPos,
                                                                          gantry1YPos);
  gantry1_startpos_rotsecond_tiltsecond = gantryConfigCalc.GetGantryConfig(0, gant1_rot2, gant1_tilt_end, gantry1XPos,
                                                                           gantry1YPos);
  gantry1_endpos_rotfirst_tiltsecond = gantryConfigCalc.GetGantryConfig(0, gant1_rot1, gant1_tilt_end, gantry1XDes,
                                                                        gantry1YDes);
  gantry1_endpos_rotsecond_tiltsecond = gantryConfigCalc.GetGantryConfig(0, gant1_rot2, gant1_tilt_end, gantry1XDes,
                                                                         gantry1YDes);
  box0_startpos_rotfirst_tiltsecond = gantryConfigCalc.GetOpticalBoxConfig(0, gant1_rot1, gant1_tilt_end, gantry1XPos,
                                                                           gantry1YPos);
  box0_startpos_rotsecond_tiltsecond = gantryConfigCalc.GetOpticalBoxConfig(0, gant1_rot2, gant1_tilt_end, gantry1XPos,
                                                                            gantry1YPos);
  box0_endpos_rotfirst_tiltsecond = gantryConfigCalc.GetOpticalBoxConfig(0, gant1_rot1, gant1_tilt_end, gantry1XDes,
                                                                         gantry1YDes);
  box0_endpos_rotsecond_tiltsecond = gantryConfigCalc.GetOpticalBoxConfig(0, gant1_rot2, gant1_tilt_end, gantry1XDes,
                                                                          gantry1YDes);
  // Final state of gantry0 & gantry1, before rotation with XY limit corners with tilt dependence
  gantry2_startpos_rotfirst_tiltsecond = gantryConfigCalc.GetGantryConfig(1, gant2_rot1, gant2_tilt_end, gantry2XPos,
                                                                          gantry2YPos);
  gantry2_startpos_rotsecond_tiltsecond = gantryConfigCalc.GetGantryConfig(1, gant2_rot2, gant2_tilt_end, gantry2XPos,
                                                                           gantry2YPos);
  gantry2_endpos_rotfirst_tiltsecond = gantryConfigCalc.GetGantryConfig(1, gant2_rot1, gant2_tilt_end, gantry2XDes,
                                                                        gantry2YDes);
  gantry2_endpos_rotsecond_tiltsecond = gantryConfigCalc.GetGantryConfig(1, gant2_rot2, gant2_tilt_end, gantry2XDes,
                                                                         gantry2YDes);
  box1_startpos_rotfirst_tiltsecond = gantryConfigCalc.GetOpticalBoxConfig(1, gant2_rot1, gant2_tilt_end, gantry2XPos,
                                                                           gantry2YPos);
  box1_startpos_rotsecond_tiltsecond = gantryConfigCalc.GetOpticalBoxConfig(1, gant2_rot2, gant2_tilt_end, gantry2XPos,
                                                                            gantry2YPos);
  box1_endpos_rotfirst_tiltsecond = gantryConfigCalc.GetOpticalBoxConfig(1, gant2_rot1, gant2_tilt_end, gantry2XDes,
                                                                         gantry2YDes);
  box1_endpos_rotsecond_tiltsecond = gantryConfigCalc.GetOpticalBoxConfig(1, gant2_rot2, gant2_tilt_end, gantry2XDes,
                                                                          gantry2YDes);

  // Path calculator checks if destination is inside the other gantry or outside the tank, but doesn't check rotation
  // Leave this check here for now
  if (pInfo->Destination[3] < rot_min || pInfo->Destination[3] > rot_max) {
    cm_msg(MERROR, "generate_path", "Illegal value for 1st rotary angle of %5.2f: not between %5.2f and %5.2f",
           pInfo->Destination[3], rot_min, rot_max);
    return GENPATH_BAD_DEST;
  }
  if (pInfo->Destination[8] < rot_min || pInfo->Destination[8] > rot_max) {
    cm_msg(MERROR, "generate_path", "Illegal value for 2nd rotary angle of %5.2f: not between %5.2f and %5.2f",
           pInfo->Destination[8], rot_min, rot_max);
    return GENPATH_BAD_DEST;
  }
  if (pInfo->Destination[4] < tilt_min || pInfo->Destination[4] > tilt_max) {
    cm_msg(MERROR, "generate_path", "Illegal value for 1st tilt angle of %5.2f: not between %5.2f and %5.2f",
           pInfo->Destination[4], tilt_min, tilt_max);
    return GENPATH_BAD_DEST;
  }
  if (pInfo->Destination[9] < tilt_min || pInfo->Destination[9] > tilt_max) {
    cm_msg(MERROR, "generate_path", "Illegal value for 2nd tilt angle of %5.2f: not between %5.2f and %5.2f",
           pInfo->Destination[9], tilt_min, tilt_max);
    return GENPATH_BAD_DEST;
  }
  if (pInfo->Destination[2] > z_max_value || pInfo->Destination[7] > z_max_value) {
    cm_msg(MERROR, "generate_path", "Illegal value for Z position: should be smaller than %5.2f", z_max_value);
    return GENPATH_BAD_DEST;
  }

  // Rika (27Apr2017): Added checks to ensure that destination is within the X & Y gantry limits;
  // otherwise the collision avoidance code will assume that the gantry can get to that location safely,
  // when it might in fact get stuck at a limit switch in a collision zone.
  if (pInfo->Destination[0] < pInfo->LimPos[0] || pInfo->Destination[0] > pInfo->LimPos[5]) {
    cm_msg(MERROR, "generate_path",
           "Gantry 0 X destination at %5.3f is outside gantry limits: not between %5.3f and %5.3f",
           pInfo->Destination[0], pInfo->LimPos[0], pInfo->LimPos[5]);
    return GENPATH_BAD_DEST;
  }
  if (pInfo->Destination[5] < pInfo->LimPos[0] || pInfo->Destination[5] > pInfo->LimPos[5]) {
    cm_msg(MERROR, "generate_path",
           "Gantry 1 X destination at %5.3f is outside gantry limits: not between %5.3f and %5.3f",
           pInfo->Destination[5], pInfo->LimPos[0], pInfo->LimPos[5]);
    return GENPATH_BAD_DEST;
  }
  if (pInfo->Destination[1] < pInfo->LimPos[1] || pInfo->Destination[1] > pInfo->LimPos[6]) {
    cm_msg(MERROR, "generate_path",
           "Gantry 0 Y destination at %5.3f is outside gantry limits: not between %5.3f and %5.3f",
           pInfo->Destination[1], pInfo->LimPos[1], pInfo->LimPos[6]);
    return GENPATH_BAD_DEST;
  }
  if (pInfo->Destination[6] < pInfo->LimPos[1] || pInfo->Destination[6] > pInfo->LimPos[6]) {
    cm_msg(MERROR, "generate_path",
           "Gantry 1 Y destination at %5.3f is outside gantry limits: not between %5.3f and %5.3f",
           pInfo->Destination[6], pInfo->LimPos[1], pInfo->LimPos[6]);
    return GENPATH_BAD_DEST;
  }

  // Z movement: if the end position of gantry 1 or 2 is above the water tank, move in the z-direction first -- SAFE, as always moving out of tank
  if (tankheight_gantend1) move_z1_first = true;
  if (tankheight_gantend2) move_z2_first = true;

  std::cout << "Move z first:  " << move_z1_first << "  " << move_z2_first << std::endl;

  // First check: beams should never cross each other
  if(pInfo->Destination[0] + 0.05 >= pInfo->Destination[5]){ // conservative
  //if (pInfo->Destination[0] - 0.05 >= pInfo->Destination[5]) {  //closest possible
    cm_msg(MERROR, "generate_path", "Illegal value for X destination: Beams will collide");
    return GENPATH_BAD_DEST;
  }

  // Rika: Check destination to ensure optical boxes will not collide with each other, with tank, or with PMT.
  // Rika (31Mar2016): Updated to include check for both bottom & top surfaces of boxes
  bool validDestination_box0 = true;
  bool validDestination_box1 = true;
  std::pair<double, double> finalZ_box0_lo_up = gantryConfigCalc.GetOpticalBoxZ(0, gant1_tilt_start, gantry1ZDes);
  std::pair<double, double> finalZ_box1_lo_up = gantryConfigCalc.GetOpticalBoxZ(1, gant2_tilt_start, gantry2ZDes);
  int finalZ_box0_lo = (finalZ_box0_lo_up.first - pmtHeight) / pmtPolyLayerHeight;
  int finalZ_box0_up = (finalZ_box0_lo_up.second - pmtHeight) / pmtPolyLayerHeight;
  int finalZ_box1_lo = (finalZ_box1_lo_up.first - pmtHeight) / pmtPolyLayerHeight;
  int finalZ_box1_up = (finalZ_box1_lo_up.second - pmtHeight) / pmtPolyLayerHeight;

  // Check gantries & tank:
  validDestination_box0 = pathCalc_checkDestination.CheckDestination(gantry1_endpos_rotfirst_tiltfirst,
                                                                     gantry2_endpos_rotfirst_tiltfirst,
                                                                     tankheight_gantend1);
  validDestination_box1 = pathCalc_checkDestination.CheckDestination(gantry2_endpos_rotfirst_tiltfirst,
                                                                     gantry1_endpos_rotfirst_tiltfirst,
                                                                     tankheight_gantend2);

  if (!(validDestination_box0 && validDestination_box1)) {
    cm_msg(MERROR, "generate_path", "Invalid destination: gantries will collide with each other or with tank.");
    return GENPATH_BAD_DEST;
  }

  // Check PMT:
  if (finalZ_box0_lo >= 0) {
    validDestination_box0 = pathCalc_checkDestination.CheckDestination(box0_endpos_rotfirst_tiltfirst.first,
                                                                       pmtPoly0.at(finalZ_box0_lo),
                                                                       tankheight_gantend1);
    if (validDestination_box0 && finalZ_box0_up >= 0) {
      validDestination_box0 = pathCalc_checkDestination.CheckDestination(box0_endpos_rotfirst_tiltfirst.second,
                                                                         pmtPoly0.at(finalZ_box0_up),
                                                                         tankheight_gantend1);
    }
  }
  if (finalZ_box1_lo >= 0) {
    validDestination_box1 = pathCalc_checkDestination.CheckDestination(box1_endpos_rotfirst_tiltfirst.first,
                                                                       pmtPoly1.at(finalZ_box1_lo),
                                                                       tankheight_gantend2);
    if (validDestination_box1 && finalZ_box1_up >= 0) {
      validDestination_box1 = pathCalc_checkDestination.CheckDestination(box1_endpos_rotfirst_tiltfirst.second,
                                                                         pmtPoly1.at(finalZ_box1_up),
                                                                         tankheight_gantend2);
    }
  }
  // Remove comments
  /*
  if (!(validDestination_box0 && validDestination_box1)) {
    cm_msg(MERROR, "generate_path", "Invalid destination: gantry will collide with PMT.");
    return GENPATH_BAD_DEST;
  }
  */

  // Define xy plane in terms of z heights at which path will be checked for collision avoidance.
  double gantry1Z = gantry1ZDes;
  double gantry2Z = gantry2ZDes;

  if (gantry1ZDes <= gantry1ZPos) {
    gantry1Z = gantry1ZDes;
    move_z1_first = true; // It is always safer to move up first (to region with more space)
  }                         // UNLESS the PMT holders are directly above the optical box.
  else {
    gantry1Z = gantry1ZPos;
    move_z1_first = false;
  }

  if (gantry2ZDes <= gantry2ZPos) {
    gantry2Z = gantry2ZDes;
    move_z2_first = true;
  } else {
    gantry2Z = gantry2ZPos;
    move_z2_first = false;
  }


  // Rika (31Mar2017): Optical box z position obtained for different tilt angles.
  // gantryConfigCalc.GetOpticalBoxZ returns a pair of z positions for lower and upper optical box surfaces;
  // therefore lowest point is the first of the two doubles.
  // Tilt first
  opticalBox0_height[0] = gantryConfigCalc.GetOpticalBoxZ(0, gant1_tilt_start, gantry1Z);
  opticalBox1_height[0] = gantryConfigCalc.GetOpticalBoxZ(1, gant2_tilt_start, gantry2Z);
  // Tilt second
  opticalBox0_height[1] = gantryConfigCalc.GetOpticalBoxZ(0, gant1_tilt_end, gantry1Z);
  opticalBox1_height[1] = gantryConfigCalc.GetOpticalBoxZ(1, gant2_tilt_end, gantry2Z);
  for (int tilt = 0; tilt < 2; tilt++) {
    opticalBox0_lo_Z[tilt] = (opticalBox0_height[tilt].first - pmtHeight) / pmtPolyLayerHeight;
    opticalBox0_up_Z[tilt] = (opticalBox0_height[tilt].second - pmtHeight) / pmtPolyLayerHeight;
    opticalBox1_lo_Z[tilt] = (opticalBox1_height[tilt].first - pmtHeight) / pmtPolyLayerHeight;
    opticalBox1_up_Z[tilt] = (opticalBox1_height[tilt].second - pmtHeight) / pmtPolyLayerHeight;
  }

  // Tilt first
  // Initialise gantries and water tank in path calculator for gantry 1 move first, rotation first2
  pathCalc1a.InitialiseGantries(gantry1_startpos_rotfirst_tiltfirst, gantry2_startpos_rotfirst_tiltfirst);
  pathCalc1b.InitialiseGantries(gantry1_endpos_rotfirst_tiltfirst, gantry2_startpos_rotfirst_tiltfirst);

  pathCalc1a.InitialiseOpticalBoxes(box0_startpos_rotfirst_tiltfirst.first, box0_startpos_rotfirst_tiltfirst.second,
                                    box1_startpos_rotfirst_tiltfirst.first, box1_startpos_rotfirst_tiltfirst.second,
                                    opticalBox0_lo_Z[0], opticalBox0_up_Z[0], opticalBox1_lo_Z[0], opticalBox1_up_Z[0]);
  pathCalc1b.InitialiseOpticalBoxes(box0_endpos_rotfirst_tiltfirst.first, box0_endpos_rotfirst_tiltfirst.second,
                                    box1_startpos_rotfirst_tiltfirst.first, box1_startpos_rotfirst_tiltfirst.second,
                                    opticalBox0_lo_Z[0], opticalBox0_up_Z[0], opticalBox1_lo_Z[0], opticalBox1_up_Z[0]);

  // Initialise gantries and water tank in path calculator for gantry 1 move first, rotation second
  pathCalc2a.InitialiseGantries(gantry1_startpos_rotsecond_tiltfirst, gantry2_startpos_rotsecond_tiltfirst);
  pathCalc2b.InitialiseGantries(gantry1_endpos_rotsecond_tiltfirst, gantry2_startpos_rotsecond_tiltfirst);

  pathCalc2a.InitialiseOpticalBoxes(box0_startpos_rotsecond_tiltfirst.first, box0_startpos_rotsecond_tiltfirst.second,
                                    box1_startpos_rotsecond_tiltfirst.first, box1_startpos_rotsecond_tiltfirst.second,
                                    opticalBox0_lo_Z[0], opticalBox0_up_Z[0], opticalBox1_lo_Z[0], opticalBox1_up_Z[0]);
  pathCalc2b.InitialiseOpticalBoxes(box0_endpos_rotsecond_tiltfirst.first, box0_endpos_rotsecond_tiltfirst.second,
                                    box1_startpos_rotfirst_tiltfirst.first, box1_startpos_rotfirst_tiltfirst.second,
                                    opticalBox0_lo_Z[0], opticalBox0_up_Z[0], opticalBox1_lo_Z[0], opticalBox1_up_Z[0]);

  // Initialise gantries and water tank in path calculator for gantry 2 move first, rotation first
  pathCalc3a.InitialiseGantries(gantry1_startpos_rotfirst_tiltfirst, gantry2_startpos_rotfirst_tiltfirst);
  pathCalc3b.InitialiseGantries(gantry1_startpos_rotfirst_tiltfirst, gantry2_endpos_rotfirst_tiltfirst);

  pathCalc3a.InitialiseOpticalBoxes(box0_startpos_rotfirst_tiltfirst.first, box0_startpos_rotfirst_tiltfirst.second,
                                    box1_startpos_rotfirst_tiltfirst.first, box1_startpos_rotfirst_tiltfirst.second,
                                    opticalBox0_lo_Z[0], opticalBox0_up_Z[0], opticalBox1_lo_Z[0], opticalBox1_up_Z[0]);
  pathCalc3b.InitialiseOpticalBoxes(box0_startpos_rotfirst_tiltfirst.first, box0_startpos_rotfirst_tiltfirst.second,
                                    box1_endpos_rotfirst_tiltfirst.first, box1_endpos_rotfirst_tiltfirst.second,
                                    opticalBox0_lo_Z[0], opticalBox0_up_Z[0], opticalBox1_lo_Z[0], opticalBox1_up_Z[0]);

  // Initialise gantries and water tank in path calculator for gantry 2 move first, rotation second
  pathCalc4a.InitialiseGantries(gantry1_startpos_rotsecond_tiltfirst, gantry2_startpos_rotsecond_tiltfirst);
  pathCalc4b.InitialiseGantries(gantry1_startpos_rotsecond_tiltfirst, gantry2_endpos_rotsecond_tiltfirst);

  pathCalc4a.InitialiseOpticalBoxes(box0_startpos_rotsecond_tiltfirst.first, box0_startpos_rotsecond_tiltfirst.second,
                                    box1_startpos_rotsecond_tiltfirst.first, box1_startpos_rotsecond_tiltfirst.second,
                                    opticalBox0_lo_Z[0], opticalBox0_up_Z[0], opticalBox1_lo_Z[0], opticalBox1_up_Z[0]);
  pathCalc4b.InitialiseOpticalBoxes(box0_startpos_rotsecond_tiltfirst.first, box0_startpos_rotsecond_tiltfirst.second,
                                    box1_endpos_rotsecond_tiltfirst.first, box1_endpos_rotsecond_tiltfirst.second,
                                    opticalBox0_lo_Z[0], opticalBox0_up_Z[0], opticalBox1_lo_Z[0], opticalBox1_up_Z[0]);

  // Tilt second
  // Initialise gantries and water tank in path calculator for gantry 1 move first, rotation first
  pathCalc5a.InitialiseGantries(gantry1_startpos_rotfirst_tiltsecond, gantry2_startpos_rotfirst_tiltsecond);
  pathCalc5b.InitialiseGantries(gantry1_endpos_rotfirst_tiltsecond, gantry2_startpos_rotfirst_tiltsecond);

  pathCalc5a.InitialiseOpticalBoxes(box0_startpos_rotfirst_tiltsecond.first, box0_startpos_rotfirst_tiltsecond.second,
                                    box1_startpos_rotfirst_tiltsecond.first, box1_startpos_rotfirst_tiltsecond.second,
                                    opticalBox0_lo_Z[1], opticalBox0_up_Z[1], opticalBox1_lo_Z[1], opticalBox1_up_Z[1]);
  pathCalc5b.InitialiseOpticalBoxes(box0_endpos_rotfirst_tiltsecond.first, box0_endpos_rotfirst_tiltsecond.second,
                                    box1_startpos_rotfirst_tiltsecond.first, box1_startpos_rotfirst_tiltsecond.second,
                                    opticalBox0_lo_Z[1], opticalBox0_up_Z[1], opticalBox1_lo_Z[1], opticalBox1_up_Z[1]);

  // Initialise gantries and water tank in path calculator for gantry 1 move first, rotation second
  pathCalc6a.InitialiseGantries(gantry1_startpos_rotsecond_tiltsecond, gantry2_startpos_rotsecond_tiltsecond);
  pathCalc6b.InitialiseGantries(gantry1_endpos_rotsecond_tiltsecond, gantry2_startpos_rotsecond_tiltsecond);

  pathCalc6a.InitialiseOpticalBoxes(box0_startpos_rotsecond_tiltsecond.first, box0_startpos_rotsecond_tiltsecond.second,
                                    box1_startpos_rotsecond_tiltsecond.first, box1_startpos_rotsecond_tiltsecond.second,
                                    opticalBox0_lo_Z[1], opticalBox0_up_Z[1], opticalBox1_lo_Z[1], opticalBox1_up_Z[1]);
  pathCalc6b.InitialiseOpticalBoxes(box0_endpos_rotsecond_tiltsecond.first, box0_endpos_rotsecond_tiltsecond.second,
                                    box1_startpos_rotsecond_tiltsecond.first, box1_startpos_rotsecond_tiltsecond.second,
                                    opticalBox0_lo_Z[1], opticalBox0_up_Z[1], opticalBox1_lo_Z[1], opticalBox1_up_Z[1]);

  // Initialise gantries and water tank in path calculator for gantry 2 move first, rotation first
  pathCalc7a.InitialiseGantries(gantry1_startpos_rotfirst_tiltsecond, gantry2_startpos_rotfirst_tiltsecond);
  pathCalc7b.InitialiseGantries(gantry1_startpos_rotfirst_tiltsecond, gantry2_endpos_rotfirst_tiltsecond);

  pathCalc7a.InitialiseOpticalBoxes(box0_startpos_rotfirst_tiltsecond.first, box0_startpos_rotfirst_tiltsecond.second,
                                    box1_startpos_rotfirst_tiltsecond.first, box1_startpos_rotfirst_tiltsecond.second,
                                    opticalBox0_lo_Z[1], opticalBox0_up_Z[1], opticalBox1_lo_Z[1], opticalBox1_up_Z[1]);
  pathCalc7b.InitialiseOpticalBoxes(box0_startpos_rotfirst_tiltsecond.first, box0_startpos_rotfirst_tiltsecond.second,
                                    box1_endpos_rotfirst_tiltsecond.first, box1_endpos_rotfirst_tiltsecond.second,
                                    opticalBox0_lo_Z[1], opticalBox0_up_Z[1], opticalBox1_lo_Z[1], opticalBox1_up_Z[1]);

  // Initialise gantries and water tank in path calculator for gantry 2 move first, rotation second
  pathCalc8a.InitialiseGantries(gantry1_startpos_rotsecond_tiltsecond, gantry2_startpos_rotsecond_tiltsecond);
  pathCalc8b.InitialiseGantries(gantry1_startpos_rotsecond_tiltsecond, gantry2_endpos_rotsecond_tiltsecond);

  pathCalc8a.InitialiseOpticalBoxes(box0_startpos_rotsecond_tiltsecond.first, box0_startpos_rotsecond_tiltsecond.second,
                                    box1_startpos_rotsecond_tiltsecond.first, box1_startpos_rotsecond_tiltsecond.second,
                                    opticalBox0_lo_Z[1], opticalBox0_up_Z[1], opticalBox1_lo_Z[1], opticalBox1_up_Z[1]);
  pathCalc8b.InitialiseOpticalBoxes(box0_startpos_rotsecond_tiltsecond.first, box0_startpos_rotsecond_tiltsecond.second,
                                    box1_endpos_rotsecond_tiltsecond.first, box1_endpos_rotsecond_tiltsecond.second,
                                    opticalBox0_lo_Z[1], opticalBox0_up_Z[1], opticalBox1_lo_Z[1], opticalBox1_up_Z[1]);

  // Start, end, and path for gantry 1 move first 
  XYPoint xy_start1 = std::make_pair(gantry1XPos, gantry1YPos);
  XYPoint xy_end1 = std::make_pair(gantry1XDes, gantry1YDes);
  std::vector <XYPoint> path1;
  // Start, end, and path for gantry 2 move first
  XYPoint xy_start2 = std::make_pair(gantry2XPos, gantry2YPos);
  XYPoint xy_end2 = std::make_pair(gantry2XDes, gantry2YDes);
  std::vector <XYPoint> path2;

  // Rotation and tilt paths for both gantries
  std::pair<double, double> rot_path1 = std::make_pair(pInfo->Position[3], pInfo->Destination[3]);
  std::pair<double, double> rot_path2 = std::make_pair(pInfo->Position[8], pInfo->Destination[8]);
  std::pair<double, double> rot_start1 = std::make_pair(pInfo->Position[3], pInfo->Position[3]);
  std::pair<double, double> rot_start2 = std::make_pair(pInfo->Position[8], pInfo->Position[8]);
  std::pair<double, double> rot_end1 = std::make_pair(pInfo->Destination[3], pInfo->Destination[3]);
  std::pair<double, double> rot_end2 = std::make_pair(pInfo->Destination[8], pInfo->Destination[8]);

  std::pair<double, double> tilt_path1 = std::make_pair(pInfo->Position[4], pInfo->Destination[4]);
  std::pair<double, double> tilt_path2 = std::make_pair(pInfo->Position[9], pInfo->Destination[9]);
  std::pair<double, double> tilt_start1 = std::make_pair(pInfo->Position[4], pInfo->Position[4]);
  std::pair<double, double> tilt_start2 = std::make_pair(pInfo->Position[9], pInfo->Position[9]);
  std::pair<double, double> tilt_end1 = std::make_pair(pInfo->Destination[4], pInfo->Destination[4]);
  std::pair<double, double> tilt_end2 = std::make_pair(pInfo->Destination[9], pInfo->Destination[9]);

  // determine good paths
  bool goodPath00 = false, goodPath01 = false;

  // Tilt first
  // Calculate good paths for gantry 1 move first, rotate first
  bool goodPath1a = pathCalc1a.CalculatePath(xy_start1, xy_end1, path1, start_tank, end_tank);
  bool goodPath1b = pathCalc1b.CalculatePath(xy_start2, xy_end2, path2, start_tank, end_tank);
  // Calculate good paths for gantry 1 move first, rotate second
  bool goodPath2a = pathCalc2a.CalculatePath(xy_start1, xy_end1, path1, start_tank, end_tank);
  bool goodPath2b = pathCalc2b.CalculatePath(xy_start2, xy_end2, path2, start_tank, end_tank);
  // Calculate good paths for gantry 2 move first, rotate first
  bool goodPath3a = pathCalc3a.CalculatePath(xy_start2, xy_end2, path2, start_tank, end_tank);
  bool goodPath3b = pathCalc3b.CalculatePath(xy_start1, xy_end1, path1, start_tank, end_tank);
  // Calculate good paths for gantry 2 move first, rotate second
  bool goodPath4a = pathCalc4a.CalculatePath(xy_start2, xy_end2, path2, start_tank, end_tank);
  bool goodPath4b = pathCalc4b.CalculatePath(xy_start1, xy_end1, path1, start_tank, end_tank);
  // Incremental rotation & tilt paths verification 
  bool goodRotationTilt_rotfirst_tiltfirst = pathCalc_rot_tilt.CalculatePath(xy_start1, xy_start2, rot_path1, rot_path2,
                                                                             tilt_path1, tilt_path2, gantry1Z, gantry2Z,
                                                                             start_tank, end_tank);
  bool goodTilt_rotsecond_tiltfirst = pathCalc_rot_tilt.CalculatePath(xy_start1, xy_start2, rot_start1, rot_start2,
                                                                      tilt_path1, tilt_path2, gantry1Z, gantry2Z,
                                                                      start_tank, end_tank);
  bool goodRotation_rotsecond_tiltfirst = pathCalc_rot_tilt.CalculatePath(xy_end1, xy_end2, rot_path1, rot_path2,
                                                                          tilt_end1, tilt_end2, gantry1Z, gantry2Z,
                                                                          start_tank, end_tank);
  bool goodRotationTilt_endcheck_tiltfirst = pathCalc_rot_tilt.CalculatePath(xy_end1, xy_end2, rot_end1, rot_end2,
                                                                             tilt_end1, tilt_end2, gantry1Z, gantry2Z,
                                                                             start_tank, end_tank);
  // Tilt second
  // Calculate good paths for gantry 1 move first, rotate first
  bool goodPath5a = pathCalc5a.CalculatePath(xy_start1, xy_end1, path1, start_tank, end_tank);
  bool goodPath5b = pathCalc5b.CalculatePath(xy_start2, xy_end2, path2, start_tank, end_tank);
  // Calculate good paths for gantry 1 move first, rotate second
  bool goodPath6a = pathCalc6a.CalculatePath(xy_start1, xy_end1, path1, start_tank, end_tank);
  bool goodPath6b = pathCalc6b.CalculatePath(xy_start2, xy_end2, path2, start_tank, end_tank);
  // Calculate good paths for gantry 2 move first, rotate first
  bool goodPath7a = pathCalc7a.CalculatePath(xy_start2, xy_end2, path2, start_tank, end_tank);
  bool goodPath7b = pathCalc7b.CalculatePath(xy_start1, xy_end1, path1, start_tank, end_tank);
  // Calculate good paths for gantry 2 move first, rotate second
  bool goodPath8a = pathCalc8a.CalculatePath(xy_start2, xy_end2, path2, start_tank, end_tank);
  bool goodPath8b = pathCalc8b.CalculatePath(xy_start1, xy_end1, path1, start_tank, end_tank);
  // Incremental rotation path verification 
  bool goodRotation_rotfirst_tiltsecond = pathCalc_rot_tilt.CalculatePath(xy_start1, xy_start2, rot_path1, rot_path2,
                                                                          tilt_start1, tilt_start2, gantry1Z, gantry2Z,
                                                                          start_tank, end_tank);
  bool goodTilt_rotfirst_tiltsecond = pathCalc_rot_tilt.CalculatePath(xy_end1, xy_end2, rot_end1, rot_end2, tilt_path1,
                                                                      tilt_path2, gantry1Z, gantry2Z, start_tank,
                                                                      end_tank);
  bool goodRotationTilt_rotsecond_tiltsecond = pathCalc_rot_tilt.CalculatePath(xy_end1, xy_end2, rot_path1, rot_path2,
                                                                               tilt_path1, tilt_path2, gantry1Z,
                                                                               gantry2Z, start_tank, end_tank);
  bool goodRotationTilt_endcheck_tiltsecond = pathCalc_rot_tilt.CalculatePath(xy_end1, xy_end2, rot_end1, rot_end2,
                                                                              tilt_end1, tilt_end2, gantry1Z, gantry2Z,
                                                                              start_tank, end_tank);

  //Check whether we are not crossing a beam, because PathCalculator does not know about the beam
  //  //if(pInfo->Destination[0] + 0.05 >= gantry2XPos){   //conservative
  if (pInfo->Destination[0] - 0.05 >= gantry2XPos) {     // closest possible
    cm_msg(MINFO, "generate_path",
           "Illegal value for X destination gantry1: will collide against beam gantry 2. Only possible if you move gantry2 out of the way first");
    //move_second_gantry_first = true;
    gant2_movefirst = true;
  }

  // Tilt first 1-4, tilt second 5-8
  // Determine which motion the gantry system can take: IF 1) gantry1 move first, rotate first ELSE 2) gantry1 move first, rotate second ELSE 3) gantry2 move first, rotate first ELSE 4) gantry2 move first, rotate second
  if (goodPath1a && goodPath1b && goodRotationTilt_rotfirst_tiltfirst && goodRotationTilt_endcheck_tiltfirst &&
      !gant2_movefirst) {
    cm_msg(MINFO, "generate_path", "Good path identified: Gantry 1 move first, rotation first, tilt first");
    pathCalc00.InitialiseGantries(gantry1_startpos_rotfirst_tiltfirst, gantry2_startpos_rotfirst_tiltfirst);
    pathCalc00.InitialiseOpticalBoxes(box0_startpos_rotfirst_tiltfirst.first,
                                      box0_startpos_rotfirst_tiltfirst.second,
                                      box1_startpos_rotfirst_tiltfirst.first,
                                      box1_startpos_rotfirst_tiltfirst.second,
                                      opticalBox0_lo_Z[0], opticalBox0_up_Z[0],
                                      opticalBox1_lo_Z[0], opticalBox1_up_Z[0]);
    goodPath00 = pathCalc00.CalculatePath(xy_start1, xy_end1, path1, start_tank, end_tank);
    pathCalc01.InitialiseGantries(gantry1_endpos_rotfirst_tiltfirst, gantry2_startpos_rotfirst_tiltfirst);
    pathCalc01.InitialiseOpticalBoxes(box0_endpos_rotfirst_tiltfirst.first, box0_endpos_rotfirst_tiltfirst.second,
                                      box1_startpos_rotfirst_tiltfirst.first, box1_startpos_rotfirst_tiltfirst.second,
                                      opticalBox0_lo_Z[0], opticalBox0_up_Z[0], opticalBox1_lo_Z[0],
                                      opticalBox1_up_Z[0]);
    goodPath01 = pathCalc01.CalculatePath(xy_start2, xy_end2, path2, start_tank, end_tank);
    move_rotation_first = true;
    tiltfirst = true;

  } else if (goodPath2a && goodPath2b && goodTilt_rotsecond_tiltfirst && goodRotation_rotsecond_tiltfirst &&
             goodRotationTilt_endcheck_tiltfirst && !gant2_movefirst) {
    cm_msg(MINFO, "generate_path", "Good path identified: Gantry 1 move first, rotation second, tilt first");
    pathCalc00.InitialiseGantries(gantry1_startpos_rotsecond_tiltfirst, gantry2_startpos_rotsecond_tiltfirst);
    pathCalc00.InitialiseOpticalBoxes(box0_startpos_rotsecond_tiltfirst.first, box0_startpos_rotsecond_tiltfirst.second,
                                      box1_startpos_rotsecond_tiltfirst.first, box1_startpos_rotsecond_tiltfirst.second,
                                      opticalBox0_lo_Z[0], opticalBox0_up_Z[0], opticalBox1_lo_Z[0],
                                      opticalBox1_up_Z[0]);
    goodPath00 = pathCalc00.CalculatePath(xy_start1, xy_end1, path1, start_tank, end_tank);
    pathCalc01.InitialiseGantries(gantry1_endpos_rotsecond_tiltfirst, gantry2_startpos_rotsecond_tiltfirst);
    pathCalc01.InitialiseOpticalBoxes(box0_endpos_rotsecond_tiltfirst.first, box0_endpos_rotsecond_tiltfirst.second,
                                      box1_startpos_rotfirst_tiltfirst.first, box1_startpos_rotfirst_tiltfirst.second,
                                      opticalBox0_lo_Z[0], opticalBox0_up_Z[0], opticalBox1_lo_Z[0],
                                      opticalBox1_up_Z[0]);
    goodPath01 = pathCalc01.CalculatePath(xy_start2, xy_end2, path2, start_tank, end_tank);
    tiltfirst = true;

  } else if (goodPath3a && goodPath3b && goodRotationTilt_rotfirst_tiltfirst && goodRotationTilt_endcheck_tiltfirst) {
    cm_msg(MINFO, "generate_path", "Good path identified: Gantry 2 move first, rotation first, tilt first");
    pathCalc00.InitialiseGantries(gantry1_startpos_rotfirst_tiltfirst, gantry2_endpos_rotfirst_tiltfirst);
    pathCalc00.InitialiseOpticalBoxes(box0_startpos_rotfirst_tiltfirst.first, box0_startpos_rotfirst_tiltfirst.second,
                                      box1_startpos_rotfirst_tiltfirst.first, box1_startpos_rotfirst_tiltfirst.second,
                                      opticalBox0_lo_Z[0], opticalBox0_up_Z[0], opticalBox1_lo_Z[0],
                                      opticalBox1_up_Z[0]);
    goodPath00 = pathCalc00.CalculatePath(xy_start1, xy_end1, path1, start_tank, end_tank);
    pathCalc01.InitialiseGantries(gantry1_startpos_rotfirst_tiltfirst, gantry2_startpos_rotfirst_tiltfirst);
    pathCalc01.InitialiseOpticalBoxes(box0_startpos_rotfirst_tiltfirst.first, box0_startpos_rotfirst_tiltfirst.second,
                                      box1_endpos_rotfirst_tiltfirst.first, box1_endpos_rotfirst_tiltfirst.second,
                                      opticalBox0_lo_Z[0], opticalBox0_up_Z[0], opticalBox1_lo_Z[0],
                                      opticalBox1_up_Z[0]);
    goodPath01 = pathCalc01.CalculatePath(xy_start2, xy_end2, path2, start_tank, end_tank);
    // Check first whether we are not crossing a beam in a way that gantry1 had to move first
    if (pInfo->Destination[5] + 0.05 <= gantry1XPos) {  // closest possible
      cm_msg(MERROR, "generate_path", "Illegal value for X destination gantry2: will collide against beam gantry 1.");
      return GENPATH_BAD_DEST;
    }
    move_rotation_first = true;
    move_second_gantry_first = true;
    tiltfirst = true;
  } else if (goodPath4a && goodPath4b && goodTilt_rotsecond_tiltfirst && goodRotation_rotsecond_tiltfirst &&
             goodRotationTilt_endcheck_tiltfirst) {
    cm_msg(MINFO, "generate_path", "Good path identified: Gantry 2 move first, rotation second, tilt first");
    pathCalc00.InitialiseGantries(gantry1_startpos_rotsecond_tiltfirst, gantry2_endpos_rotsecond_tiltfirst);
    pathCalc00.InitialiseOpticalBoxes(box0_startpos_rotsecond_tiltfirst.first, box0_startpos_rotsecond_tiltfirst.second,
                                      box1_startpos_rotsecond_tiltfirst.first, box1_startpos_rotsecond_tiltfirst.second,
                                      opticalBox0_lo_Z[0], opticalBox0_up_Z[0], opticalBox1_lo_Z[0],
                                      opticalBox1_up_Z[0]);
    goodPath00 = pathCalc00.CalculatePath(xy_start1, xy_end1, path1, start_tank, end_tank);
    pathCalc01.InitialiseGantries(gantry1_startpos_rotsecond_tiltfirst, gantry2_startpos_rotsecond_tiltfirst);
    pathCalc01.InitialiseOpticalBoxes(box0_startpos_rotsecond_tiltfirst.first, box0_startpos_rotsecond_tiltfirst.second,
                                      box1_endpos_rotsecond_tiltfirst.first, box1_endpos_rotsecond_tiltfirst.second,
                                      opticalBox0_lo_Z[0], opticalBox0_up_Z[0], opticalBox1_lo_Z[0],
                                      opticalBox1_up_Z[0]);
    goodPath01 = pathCalc01.CalculatePath(xy_start2, xy_end2, path2, start_tank, end_tank);
    // Check first whether we are not crossing a beam in a way that gantry1 had to move first
    if (pInfo->Destination[5] + 0.05 <= gantry1XPos) {  // closest possible
      cm_msg(MERROR, "generate_path", "Illegal value for X destination gantry2: will collide against beam gantry 1.");
      return GENPATH_BAD_DEST;
    }
    move_second_gantry_first = true;
    tiltfirst = true;
  } else if (goodPath5a && goodPath5b && goodRotation_rotfirst_tiltsecond && goodTilt_rotfirst_tiltsecond &&
             goodRotationTilt_endcheck_tiltsecond && !gant2_movefirst) {
    cm_msg(MINFO, "generate_path", "Good path identified: Gantry 1 move first, rotation first, tilt second");
    pathCalc00.InitialiseGantries(gantry1_startpos_rotfirst_tiltsecond, gantry2_startpos_rotfirst_tiltsecond);
    pathCalc00.InitialiseOpticalBoxes(box0_startpos_rotfirst_tiltsecond.first, box0_startpos_rotfirst_tiltsecond.second,
                                      box1_startpos_rotfirst_tiltsecond.first, box1_startpos_rotfirst_tiltsecond.second,
                                      opticalBox0_lo_Z[1], opticalBox0_up_Z[1], opticalBox1_lo_Z[1],
                                      opticalBox1_up_Z[1]);
    goodPath00 = pathCalc00.CalculatePath(xy_start1, xy_end1, path1, start_tank, end_tank);
    pathCalc01.InitialiseGantries(gantry1_endpos_rotfirst_tiltsecond, gantry2_startpos_rotfirst_tiltsecond);
    pathCalc01.InitialiseOpticalBoxes(box0_endpos_rotfirst_tiltsecond.first, box0_endpos_rotfirst_tiltsecond.second,
                                      box1_startpos_rotfirst_tiltsecond.first, box1_startpos_rotfirst_tiltsecond.second,
                                      opticalBox0_lo_Z[1], opticalBox0_up_Z[1], opticalBox1_lo_Z[1],
                                      opticalBox1_up_Z[1]);

    goodPath01 = pathCalc01.CalculatePath(xy_start2, xy_end2, path2, start_tank, end_tank);
    move_rotation_first = true;
  } else if (goodPath6a && goodPath6b && goodRotationTilt_rotsecond_tiltsecond &&
             goodRotationTilt_endcheck_tiltsecond && !gant2_movefirst) {
    cm_msg(MINFO, "generate_path", "Good path identified: Gantry 1 move first, rotation second, tilt second");
    pathCalc00.InitialiseGantries(gantry1_startpos_rotsecond_tiltsecond, gantry2_startpos_rotsecond_tiltsecond);
    pathCalc00.InitialiseOpticalBoxes(box0_startpos_rotsecond_tiltsecond.first,
                                      box0_startpos_rotsecond_tiltsecond.second,
                                      box1_startpos_rotsecond_tiltsecond.first,
                                      box1_startpos_rotsecond_tiltsecond.second,
                                      opticalBox0_lo_Z[1], opticalBox0_up_Z[1], opticalBox1_lo_Z[1],
                                      opticalBox1_up_Z[1]);
    goodPath00 = pathCalc00.CalculatePath(xy_start1, xy_end1, path1, start_tank, end_tank);
    pathCalc01.InitialiseGantries(gantry1_endpos_rotsecond_tiltsecond, gantry2_startpos_rotsecond_tiltsecond);
    pathCalc01.InitialiseOpticalBoxes(box0_endpos_rotsecond_tiltsecond.first, box0_endpos_rotsecond_tiltsecond.second,
                                      box1_startpos_rotsecond_tiltsecond.first,
                                      box1_startpos_rotsecond_tiltsecond.second,
                                      opticalBox0_lo_Z[1], opticalBox0_up_Z[1], opticalBox1_lo_Z[1],
                                      opticalBox1_up_Z[1]);
    goodPath01 = pathCalc01.CalculatePath(xy_start2, xy_end2, path2, start_tank, end_tank);
  } else if (goodPath7a && goodPath7b && goodRotation_rotfirst_tiltsecond && goodTilt_rotfirst_tiltsecond &&
             goodRotationTilt_endcheck_tiltsecond) {
    cm_msg(MINFO, "generate_path", "Good path identified: Gantry 2 move first, rotation first, tilt second");
    pathCalc00.InitialiseGantries(gantry1_startpos_rotfirst_tiltsecond, gantry2_endpos_rotfirst_tiltsecond);
    pathCalc00.InitialiseOpticalBoxes(box0_startpos_rotfirst_tiltsecond.first, box0_startpos_rotfirst_tiltsecond.second,
                                      box1_startpos_rotfirst_tiltsecond.first, box1_startpos_rotfirst_tiltsecond.second,
                                      opticalBox0_lo_Z[1], opticalBox0_up_Z[1], opticalBox1_lo_Z[1],
                                      opticalBox1_up_Z[1]);
    goodPath00 = pathCalc00.CalculatePath(xy_start1, xy_end1, path1, start_tank, end_tank);
    pathCalc01.InitialiseGantries(gantry1_startpos_rotfirst_tiltsecond, gantry2_startpos_rotfirst_tiltsecond);
    pathCalc01.InitialiseOpticalBoxes(box0_startpos_rotfirst_tiltsecond.first, box0_startpos_rotfirst_tiltsecond.second,
                                      box1_endpos_rotfirst_tiltsecond.first, box1_endpos_rotfirst_tiltsecond.second,
                                      opticalBox0_lo_Z[1], opticalBox0_up_Z[1], opticalBox1_lo_Z[1],
                                      opticalBox1_up_Z[1]);
    goodPath01 = pathCalc01.CalculatePath(xy_start2, xy_end2, path2, start_tank, end_tank);
    // Check first whether we are not crossing a beam in a way that gantry1 had to move first
    if (pInfo->Destination[5] + 0.05 <= gantry1XPos) {  // closest possible
      cm_msg(MERROR, "generate_path", "Illegal value for X destination gantry2: will collide against beam gantry 1.");
      return GENPATH_BAD_DEST;
    }
    move_rotation_first = true;
    move_second_gantry_first = true;
  } else if (goodPath8a && goodPath8b && goodRotationTilt_rotsecond_tiltsecond &&
             goodRotationTilt_endcheck_tiltsecond) {
    cm_msg(MINFO, "generate_path", "Good path identified: Gantry 2 move first, rotation second, tilt second");
    pathCalc00.InitialiseGantries(gantry1_startpos_rotsecond_tiltsecond, gantry2_endpos_rotsecond_tiltsecond);
    pathCalc00.InitialiseOpticalBoxes(box0_startpos_rotsecond_tiltsecond.first,
                                      box0_startpos_rotsecond_tiltsecond.second,
                                      box1_startpos_rotsecond_tiltsecond.first,
                                      box1_startpos_rotsecond_tiltsecond.second,
                                      opticalBox0_lo_Z[1], opticalBox0_up_Z[1], opticalBox1_lo_Z[1],
                                      opticalBox1_up_Z[1]);
    goodPath00 = pathCalc00.CalculatePath(xy_start1, xy_end1, path1, start_tank, end_tank);
    pathCalc01.InitialiseGantries(gantry1_startpos_rotsecond_tiltsecond, gantry2_startpos_rotsecond_tiltsecond);
    pathCalc01.InitialiseOpticalBoxes(box0_startpos_rotsecond_tiltsecond.first,
                                      box0_startpos_rotsecond_tiltsecond.second,
                                      box1_endpos_rotsecond_tiltsecond.first, box1_endpos_rotsecond_tiltsecond.second,
                                      opticalBox0_lo_Z[1], opticalBox0_up_Z[1], opticalBox1_lo_Z[1],
                                      opticalBox1_up_Z[1]);
    goodPath01 = pathCalc01.CalculatePath(xy_start2, xy_end2, path2, start_tank, end_tank);
    // Check first whether we are not crossing a beam in a way that gantry1 had to move first
    if (pInfo->Destination[5] + 0.05 <= gantry1XPos) {  // closest possible
      cm_msg(MERROR, "generate_path", "Illegal value for X destination gantry2: will collide against beam gantry 1.");
      return GENPATH_BAD_DEST;
    }
    move_second_gantry_first = true;
  } else {
    cm_msg(MERROR, "generate_path", "No good path available");
    return GENPATH_BAD_DEST;
  }

  //Initialize path to current position
  for (int i = 0; i < 10; i++) {
    //Why: +3: optional zfirst, then zlast, theta, phi is last move and very first one is initial pos.
    pInfo->MovePath[i] = (float *) calloc(path1.size() + path2.size() + 3, sizeof(float));
    for (unsigned int j = 0; j < path1.size() + path2.size() + 3; ++j) {
      pInfo->MovePath[i][j] = pInfo->CountPos[i];
    }
  }

  //AJ: To what accuracy is the rounding??
  // move Z first
  if (move_z1_first) {
    pInfo->MovePath[2][1] = round((pInfo->Destination[2] - pInfo->LimPos[2]) * pInfo->mScale[2] + pInfo->mOrigin[2]);
  }
  if (move_z2_first) {
    pInfo->MovePath[7][1] = round((pInfo->Destination[7] - pInfo->LimPos[7]) * pInfo->mScale[7] + pInfo->mOrigin[7]);
  }

  // move rotation first  
  if (move_rotation_first) {
    pInfo->MovePath[3][1] = round((pInfo->Destination[3] - pInfo->LimPos[3]) * pInfo->mScale[3] + pInfo->mOrigin[3]);
    pInfo->MovePath[8][1] = round((pInfo->Destination[8] - pInfo->LimPos[8]) * pInfo->mScale[8] + pInfo->mOrigin[8]);
  }



  // Tilt first - tilt is always either increasing or decreasing boundary size
  if (tiltfirst) {
    pInfo->MovePath[4][1] = round((pInfo->Destination[4] - pInfo->LimPos[4]) * pInfo->mScale[4] + pInfo->mOrigin[4]);
    pInfo->MovePath[9][1] = round((pInfo->Destination[9] - pInfo->LimPos[9]) * pInfo->mScale[9] + pInfo->mOrigin[9]);
  }

  // Set paths and ordering for gantry move 
  if (!move_second_gantry_first) {
    // Put in the steps to move gantry 1 in X and Y
    for (int i = gantry_motor_start; i < gantry_motor_end; i++) {
      //if(pInfo->Channels[i] != -1){ // temporarily remove this condition as it is not accepting i=9, despite pInfo->Channels[9] != -1
      for (unsigned int j = 1; j < path1.size() + 1; ++j) {
        // Move in X or Y according to determined path for first gantry
        if (i == 0) pInfo->MovePath[i][j + 1] = round(pInfo->MovePath[i][j] + path1[j - 1].first * pInfo->mScale[i]);
        else if (i == 1)
          pInfo->MovePath[i][j + 1] = round(pInfo->MovePath[i][j] + path1[j - 1].second * pInfo->mScale[i]);
        else pInfo->MovePath[i][j + 1] = pInfo->MovePath[i][j];
      }
      //}
    }

    // Put in the steps to move gantry 2 in X and Y
    for (int i = gantry_motor_start; i < gantry_motor_end; i++) {
      //if(pInfo->Channels[i] != -1){
      for (unsigned int j = 1; j < path2.size() + 1; ++j) {
        // Move in X or Y according to determined path for first gantry
        if (i == 5)
          pInfo->MovePath[i][j + 1 + path1.size()] = round(
              pInfo->MovePath[i][j + path1.size()] + path2[j - 1].first * pInfo->mScale[i]);
        else if (i == 6)
          pInfo->MovePath[i][j + 1 + path1.size()] = round(
              pInfo->MovePath[i][j + path1.size()] + path2[j - 1].second * pInfo->mScale[i]);
        else pInfo->MovePath[i][j + 1 + path1.size()] = pInfo->MovePath[i][j + path1.size()];
      }
      //}
    }
  } else {
    // Put in the steps to move gantry 2 in X and Y
    for (int i = gantry_motor_start; i < gantry_motor_end; i++) {
      //if(pInfo->Channels[i] != -1){
      for (unsigned int j = 1; j < path2.size() + 1; ++j) {
        // Move in X or Y according to determined path for first gantry
        if (i == 5) pInfo->MovePath[i][j + 1] = round(pInfo->MovePath[i][j] + path2[j - 1].first * pInfo->mScale[i]);
        else if (i == 6)
          pInfo->MovePath[i][j + 1] = round(pInfo->MovePath[i][j] + path2[j - 1].second * pInfo->mScale[i]);
        else pInfo->MovePath[i][j + 1] = pInfo->MovePath[i][j];
      }
      //}
    }
    // Put in the steps to move gantry 1 in X and Y
    for (int i = gantry_motor_start; i < gantry_motor_end; i++) {
      //if(pInfo->Channels[i] != -1){
      for (unsigned int j = 1; j < path1.size() + 1; ++j) {
        // Move in X or Y according to determined path for first gantry
        if (i == 0)
          pInfo->MovePath[i][j + 1 + path2.size()] = round(
              pInfo->MovePath[i][j + path2.size()] + path1[j - 1].first * pInfo->mScale[i]);
        else if (i == 1)
          pInfo->MovePath[i][j + 1 + path2.size()] = round(
              pInfo->MovePath[i][j + path2.size()] + path1[j - 1].second * pInfo->mScale[i]);
        else pInfo->MovePath[i][j + 1 + path2.size()] = pInfo->MovePath[i][j + path2.size()];
      }
      //}
    }
  }

  // Keep X and Y coordinates of gantries at end of move
  pInfo->MovePath[0][path1.size() + path2.size() + 2] = pInfo->MovePath[0][path1.size() + path2.size() + 1];
  pInfo->MovePath[1][path1.size() + path2.size() + 2] = pInfo->MovePath[1][path1.size() + path2.size() + 1];
  pInfo->MovePath[5][path1.size() + path2.size() + 2] = pInfo->MovePath[5][path1.size() + path2.size() + 1];
  pInfo->MovePath[6][path1.size() + path2.size() + 2] = pInfo->MovePath[6][path1.size() + path2.size() + 1];

  // Lower gantry heads at end and rotate
  if (!move_z1_first)
    pInfo->MovePath[2][path1.size() + path2.size() + 2] = round(
        (pInfo->Destination[2] - pInfo->LimPos[2]) * pInfo->mScale[2] + pInfo->mOrigin[2]);
  else
    pInfo->MovePath[2][path1.size() + path2.size() + 2] = pInfo->MovePath[2][path1.size() + path2.size() + 1];
  if (!move_z2_first)
    pInfo->MovePath[7][path1.size() + path2.size() + 2] = round(
        (pInfo->Destination[7] - pInfo->LimPos[7]) * pInfo->mScale[7] + pInfo->mOrigin[7]);
  else
    pInfo->MovePath[7][path1.size() + path2.size() + 2] = pInfo->MovePath[7][path1.size() + path2.size() + 1];


  // Move rotation
  if (!move_rotation_first) {
    pInfo->MovePath[3][path1.size() + path2.size() + 2] = round(
        (pInfo->Destination[3] - pInfo->LimPos[3]) * pInfo->mScale[3] + pInfo->mOrigin[3]);
    pInfo->MovePath[8][path1.size() + path2.size() + 2] = round(
        (pInfo->Destination[8] - pInfo->LimPos[8]) * pInfo->mScale[8] + pInfo->mOrigin[8]);
  } else {
    pInfo->MovePath[3][path1.size() + path2.size() + 2] = pInfo->MovePath[3][path1.size() + path2.size() + 1];
    pInfo->MovePath[8][path1.size() + path2.size() + 2] = pInfo->MovePath[8][path1.size() + path2.size() + 1];
  }

  // Move tilt
  if (!tiltfirst) {
    pInfo->MovePath[4][path1.size() + path2.size() + 2] = round(
        (pInfo->Destination[4] - pInfo->LimPos[4]) * pInfo->mScale[4] + pInfo->mOrigin[4]);
    pInfo->MovePath[9][path1.size() + path2.size() + 2] = round(
        (pInfo->Destination[9] - pInfo->LimPos[9]) * pInfo->mScale[9] + pInfo->mOrigin[9]);
  } else {
    pInfo->MovePath[4][path1.size() + path2.size() + 2] = pInfo->MovePath[4][path1.size() + path2.size() + 1];
    pInfo->MovePath[9][path1.size() + path2.size() + 2] = pInfo->MovePath[9][path1.size() + path2.size() + 1];
  }

  for (unsigned int i = 0; i < path1.size() + path2.size() + 3; ++i) {
    for (int j = gantry_motor_start; j < gantry_motor_end; j++) {
      cm_msg(MDEBUG, "generate_path", "Destination[%i][%i] = %6.3f (Count Destination = %6.0f, Remaining =  %6.0f)", j,
             i, pInfo->Destination[j], pInfo->MovePath[j][i], pInfo->MovePath[j][i] - pInfo->CountPos[j]);
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
  BOOL start[10] = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1};
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
  int size_phidget = sizeof(pInfo->Phidget);

  //DEBUG
  printf("Moving to path index: %i\n", pInfo->PathIndex);

  // Read in axis positions (in counts)
  channel_rw(pInfo, pInfo->hKeyMPos, (void *) pInfo->CountPos, TID_FLOAT, 0);

  // Determine required destinations to be sent to the motors
  for (i = gantry_motor_start; i < gantry_motor_end; i++) {
    pInfo->CountDest[i] = pInfo->MovePath[i][pInfo->PathIndex] - pInfo->CountPos[i];

    //DEBUG
    printf("MD[%i]=%6.0f(%6.0f) ", i, pInfo->CountDest[i], pInfo->CountPos[i]);
    if (i == 4 || i == 9) {
      printf("\n");
    }
    zerotest = zerotest || pInfo->CountDest[i];
  }

  // This is added so that the monitor recognizes a move as completed, even
  // when no move is required.
  if (!zerotest) {
    cm_msg(MINFO, "move", "Warning: No move required");
    // This indicates to the monitor that a move has been initiated
    // even though the motors won't start moving.
    //TODO:: BK: Think of a better way of doing this. The Moving variable should only be used to indicate that the system is moving it could cause confusion when you set it based on other conditions.(This is me being picky)
    pInfo->Moving = 1;
    return;
  }

  // Start motors towards the specified destinations
  // Write motor destinations to the ODB
  channel_rw(pInfo, pInfo->hKeyMDest, (void *) pInfo->CountDest, TID_FLOAT, 1);

  // Get the current location of the motor. This will be used to tell if a motor has started moving yet
  db_get_data(pInfo->hDB, pInfo->hKeyMPos[0], &Motor00StartPos, &size_float, TID_FLOAT);
  db_get_data(pInfo->hDB, pInfo->hKeyMPos[1], &Motor01StartPos, &size_float, TID_FLOAT);
  db_get_data(pInfo->hDB, pInfo->hKeyMDest[0], &Motor00Dest, &size_float, TID_FLOAT);
  db_get_data(pInfo->hDB, pInfo->hKeyMDest[1], &Motor01Dest, &size_float, TID_FLOAT);

// Get the time at the beggining of the loop so that we can track how long we are in the loop
  Overall_time_for_loop = ss_millitime();

// Don't do anything else until the motors have started moving
  while (!started_moving) {
    start_of_loop = ss_millitime();
    // DEBUG
    printf("\nChannel_rw called at time : %lf\n", ss_millitime());
    // Set the ODB values to start a move

    channel_rw(pInfo, pInfo->hKeyMStart, (void *) start, TID_BOOL, 1);
    sleep(100);

    waiting = 1;
    while (waiting) {
      db_get_data(pInfo->hDB, pInfo->hKeyMPos[0], &Motor00Pos, &size_float, TID_FLOAT);
      db_get_data(pInfo->hDB, pInfo->hKeyMPos[1], &Motor01Pos, &size_float, TID_FLOAT);
      db_get_data(pInfo->hDB, pInfo->hKeyMLimitPos[0], &Motor00LimitPos, &size_bool, TID_BOOL);
      db_get_data(pInfo->hDB, pInfo->hKeyMLimitNeg[0], &Motor00LimitNeg, &size_bool, TID_BOOL);
      db_get_data(pInfo->hDB, pInfo->hKeyMLimitPos[1], &Motor01LimitPos, &size_bool, TID_BOOL);
      db_get_data(pInfo->hDB, pInfo->hKeyMLimitNeg[1], &Motor01LimitNeg, &size_bool, TID_BOOL);

      // if 300 seconds has passed and nothing has happened exit because the program is not working
      if (ss_millitime() - Overall_time_for_loop > 1000 * 300) {
        cm_msg(MERROR, "move", " The Motors never started Moving (last 300s)");
        waiting = 0;
        started_moving = 1;
      }
        // TF NOTE: the code below is vulnerable to which motor channels are used. Switching a cable screws this up.
        // If any of the motors for gantry0 are no longer at their start position that means the motors are moving and the program behaved properly
      else if (Motor00Pos[0] != Motor00StartPos[0] || Motor00Pos[1] != Motor00StartPos[1] ||
               Motor00Pos[2] != Motor00StartPos[2]
               || Motor00Pos[3] != Motor00StartPos[3] || Motor00Pos[4] != Motor00StartPos[4] ||
               Motor00Pos[5] != Motor00StartPos[5]
               || Motor00Pos[6] != Motor00StartPos[6] || Motor00Pos[7] != Motor00StartPos[7]) {
        cm_msg(MINFO, "move", " Motors gantry 0 are Moving ");
        waiting = 0;
        started_moving = 1;
        pInfo->Moving = 1;   //not necessary for long moves, but for mm moves, move can stop before monitor can check whether it's moving, so need to set here that is was really moving
      }
        // Same test for gantry1
      else if (Motor01Pos[0] != Motor01StartPos[0] || Motor01Pos[1] != Motor01StartPos[1] ||
               Motor01Pos[2] != Motor01StartPos[2]
               || Motor01Pos[3] != Motor01StartPos[3] || Motor01Pos[4] != Motor01StartPos[4] ||
               Motor01Pos[5] != Motor01StartPos[5]
               || Motor01Pos[6] != Motor01StartPos[6] || Motor01Pos[7] != Motor01StartPos[7]) {
        cm_msg(MINFO, "move", " Motors gantry 1 are Moving ");
        waiting = 0;
        started_moving = 1;
        pInfo->Moving = 1;   //not necessary for long moves, but for mm moves, move can stop before monitor can check whether it's moving, so need to set here that is was really moving
      }


        // If a motor for gantry0 is trying to move in the positive direction but the corresponding negative limit switch is engaged,
        // a move will not be started however no move is needed so set pinfo->moving to 1 so that the next move will be called
      else if (((Motor00Dest[4] > 0) && Motor00LimitNeg[4]) || ((Motor00Dest[5] > 0) && Motor00LimitNeg[5])
               || ((Motor00Dest[6] > 0) && Motor00LimitNeg[6]) || ((Motor00Dest[7] > 0) && Motor00LimitNeg[7])) {
        cm_msg(MERROR, "move", "Move could not be started because Motor00 is at a negative limit switch");
        waiting = 0;
        started_moving = 1;
        pInfo->Moving = 1;
      }

        // Now testing for move in negative direction, and hitting positive limit switch for ganty0
      else if (((Motor00Dest[4] < 0) && Motor00LimitPos[4]) || ((Motor00Dest[5] < 0) && Motor00LimitPos[5])
               || ((Motor00Dest[6] < 0) && Motor00LimitPos[6]) || ((Motor00Dest[7] < 0) && Motor00LimitPos[7])) {
        cm_msg(MERROR, "move", "Move could not be started because Motor00 is at a Positive limit switch");
        waiting = 0;
        started_moving = 1;
        pInfo->Moving = 1;
      }

        // Same for gantry1
      else if (((Motor01Dest[1] > 0) && Motor01LimitNeg[1]) || ((Motor01Dest[2] > 0) && Motor01LimitNeg[2])
               || ((Motor01Dest[3] > 0) && Motor01LimitNeg[3]) || ((Motor01Dest[4] > 0) && Motor01LimitNeg[4])) {
        cm_msg(MERROR, "move", "Move could not be started because Motor01 is at a negative limit switch");
        waiting = 0;
        started_moving = 1;
        pInfo->Moving = 1;
      }

        // Same for gantry1
      else if (((Motor01Dest[1] < 0) && Motor01LimitPos[1]) || ((Motor01Dest[2] < 0) && Motor01LimitPos[2])
               || ((Motor01Dest[3] < 0) && Motor01LimitPos[3]) || ((Motor01Dest[4] < 0) && Motor01LimitPos[4])) {
        cm_msg(MERROR, "move", "Move could not be started because Motor01 is at a Positive limit switch");
        waiting = 0;
        started_moving = 1;
        pInfo->Moving = 1;
      }


        // If 5 seconds has passed and the motors haven't started moving reset the ODB values
        // that should initiate a move with the hope that a move will start.
      else if (ss_millitime() - start_of_loop > 1000 * 5) {
        cm_msg(MINFO, "move", "Calling channel_rw again after past 5s");
        waiting = 0;
      }
    }
  }
  cm_msg(MINFO, "move", "Function Move is complete");
}

/*-- Stop_move -----------------------------------------------------*/
// Aborts a move, and prints the reason for stopping to screen
void stop_move(HNDLE hDB, HNDLE hKey, void *data) {
  INFO *pInfo = (INFO *) data;

  BOOL stop[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  channel_rw(pInfo, pInfo->hKeyMStop, (void *) stop, TID_BOOL, 1);

  switch (pInfo->AbortCode) {
    case AC_USER_INPUT:
      cm_msg(MERROR, "stop_move", "Move aborted due to user input");
      break;
    case AC_COLLISION:
      cm_msg(MERROR, "stop_move", "Move aborted due to collision detected");
      break;
    case AC_TIMEOUT:
      cm_msg(MERROR, "stop_move", "Move aborted due to move timeout");
  }

  pInfo->AbortCode = AC_USER_INPUT; // reset the abort code to default (user input)
}


/*-- Monitor -------------------------------------------------------*/
// Called periodically with /Motors/Variables/Moving, this function
// checks for completion of a move
void monitor(HNDLE hDB, HNDLE hKey, void *data) {

  INFO *pInfo = (INFO *) data;
  BOOL OldMov = pInfo->Moving; //Store the old value of pInfo->Moving for comparison to see if the value changes 
  int i = 0;

  /*-- Update variables section of ODB -----------------------------*/

  // Copy variable indicating if each axis is moving, and write this to ODB
  channel_rw(pInfo, pInfo->hKeyMMoving, (void *) pInfo->AxisMoving, TID_BOOL, 0);
  db_set_data(pInfo->hDB, pInfo->hKeyAxMoving, pInfo->AxisMoving, 10 * sizeof(float), 10, TID_BOOL);

  // Check if any of the axes are moving, and write this to ODB
  pInfo->Moving = 0;
  while ((!pInfo->Moving) && i < 10) {
    pInfo->Moving = pInfo->Moving || pInfo->AxisMoving[i];
    i++;
  }
  db_set_data(pInfo->hDB, pInfo->hKeyMoving, &pInfo->Moving, sizeof(BOOL), 1, TID_BOOL);


  // Get the axis positions and write to ODB
  channel_rw(pInfo, pInfo->hKeyMPos, (void *) pInfo->CountPos, TID_FLOAT, 0);
  for (i = gantry_motor_start; i < gantry_motor_end; i++) {
    pInfo->Position[i] = (pInfo->CountPos[i] - pInfo->mOrigin[i]) / pInfo->mScale[i] + pInfo->LimPos[i];
  }
  db_set_data(pInfo->hDB, pInfo->hKeyPos, pInfo->Position, 10 * sizeof(float), 10, TID_FLOAT);

  // Check for limit switches triggered and write to ODB
  channel_rw(pInfo, pInfo->hKeyMLimitNeg, (void *) pInfo->neg_AxisLimit, TID_BOOL, 0);
  channel_rw(pInfo, pInfo->hKeyMLimitPos, (void *) pInfo->pos_AxisLimit, TID_BOOL, 0);
  db_set_data(pInfo->hDB, pInfo->hKeyAxLimitNeg, pInfo->neg_AxisLimit, 10 * sizeof(BOOL), 10, TID_BOOL);
  db_set_data(pInfo->hDB, pInfo->hKeyAxLimitPos, pInfo->pos_AxisLimit, 10 * sizeof(BOOL), 10, TID_BOOL);


  /* - If motors have stopped moving, determine next course of action */
  bool stoppedDueToLimit = false;
  if (OldMov && !pInfo->Moving) { // i.e. If the motors just stopped moving
    // Check if the destination has been reached, otherwise return an error
    for (i = gantry_motor_start; i < gantry_motor_end; i++) {
      if ((pInfo->Channels[i] != -1) && (pInfo->MovePath[i][pInfo->PathIndex] != pInfo->CountPos[i])) {
        if (abs(pInfo->MovePath[i][pInfo->PathIndex] - pInfo->CountPos[i]) < 20){
          cm_msg(MINFO, "monitor", "final monitor position %i counts different to destination!", abs(pInfo->MovePath[i][pInfo->PathIndex] - pInfo->CountPos[i]));
        } 
        else{
          if (pInfo->neg_AxisLimit[i] || pInfo->pos_AxisLimit[i]) { //already at limit OR hit limit after move
            stoppedDueToLimit = true;
            cm_msg(MINFO, "monitor", "Stopped moving because LIMIT SWITCH for move %i reached!", i);
            // Resetting the path to take the current position at the limit (CountPos) as its destination
            // This will make sure CountDest in move() is zero (after hitting a limit switch) and also
            // MDest which is basically the same. The code checking why a move did not start will then
            // cause erroneous behaviour.

            //if moving towards limit and hitting it: stopped due to limit after move, so reset path
            if (((pInfo->MovePath[i][pInfo->PathIndex] - pInfo->CountPos[i]) < 0 && pInfo->pos_AxisLimit[i]) ||
                ((pInfo->MovePath[i][pInfo->PathIndex] - pInfo->CountPos[i]) > 0 && pInfo->neg_AxisLimit[i])) {
              int path_i;
              for (path_i = 0; path_i < pInfo->PathSize; path_i++) {
                pInfo->MovePath[i][path_i] = pInfo->CountPos[i];
              }
            }
            //break; //no break, because also check for other hit limit switches, and reset their path/destination
          } else {
            if (!stoppedDueToLimit) {
              cm_msg(MERROR, "monitor", "Move failed at i=%d, pathindex=%d : %6.2f, %6.2f", i, pInfo->PathIndex,
                  pInfo->MovePath[i][pInfo->PathIndex], pInfo->CountPos[i]);
              return;
            }
          }
        }
      }
    }

    printf("Move to index complete\n"); //DEBUG
    printf("Time to complete Move to index %lf\n", ss_millitime()); //DEBUG
    // Check if we are at the final path index, otherwise initiate next move
    if (pInfo->PathIndex + 1 == pInfo->PathSize) {
      // Final destination reached
      pInfo->Completed = 1;
      db_set_data(hDB, pInfo->hKeyCompleted, &pInfo->Completed, sizeof(BOOL), 1, TID_BOOL);
      cm_msg(MINFO, "monitor", "Move to destination complete");
      if (stoppedDueToLimit)
        cm_msg(MINFO, "monitor", "But destination not reached due to limit switch");
    } else {
      pInfo->PathIndex++;
      // DEBUG
      printf("Monitor called move() at : %lf\n", ss_millitime());
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
      // STORE_DATA(TYPE) determines the correct value for the size value and fills  motor00_values and motor01_values with the entries in the values variable
    STORE_DATA(float);
      break;
    case TID_BOOL: STORE_DATA(BOOL);
      break;
    case TID_INT: STORE_DATA(int)
      break;
  }

  if (rw == 0) {
    db_get_data(pInfo->hDB, hKey[0], motor00_values, &buff_size, type);
    db_get_data(pInfo->hDB, hKey[1], motor01_values, &buff_size, type);

    switch (type) {
      case TID_FLOAT:
        //READ_VALUES sortes the values from motor00_values and motor01_values in the values array
      READ_VALUES(float);
        break;
      case TID_BOOL: READ_VALUES(BOOL);
        break;
      case TID_INT: READ_VALUES(int);
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
