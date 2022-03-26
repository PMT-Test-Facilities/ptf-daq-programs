//current version (working)
/********************************************************************	\
 
  Name:         feMove.c 

Based on feMove from PTF, but massively simplified.  Always move X then Y

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
#include "odbxx.h"

#include "mfe.h"

  int PathSize;         // Number of waypoints in the path
  int PathIndex;        // Index of the path section currently in progress
  int AbortCode;        // Variable in which to store the reason for aborting a move (AC_*)


#define  EQ_NAME   "Move"
#define  EQ_EVID   1
#define  EQ_TRGMSK 0x1111

/* The generate_path return flags */
#define GENPATH_SUCCESS  0
#define GENPATH_BAD_DEST 1

/* The Abort Codes used by stop_move 	*/
#define AC_USER_INPUT 0
#define AC_COLLISION  1
#define AC_TIMEOUT    2

/* The frontend name (client name) as seen by other MIDAS clients   */
const char *frontend_name = "feMoveMTS";

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

/*-- Info structure declaration ------------------------------------*/

BOOL equipment_common_overwrite = FALSE;


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
void move_init(midas::odb &arg);


void monitor(HNDLE hDB, HNDLE hKey, void *data);

void move();

void stop_move(midas::odb &arg);

void initialize();

void reinitialize(midas::odb &arg);



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


// DB watch variables
midas::odb dbwatch_reinit;
midas::odb dbwatch_start;
midas::odb dbwatch_stop;

// Global variable for storing the origin position (in counts)
float mOrigin[2];


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

  midas::odb o = {
    {"Destination", std::array<float, 2>{}},
    {"Velocity", std::array<float, 2>{}},
    {"Acceleration", std::array<float, 2>{}},
    {"Motor Scaling", std::array<float, 2>{}},
    {"Axis Channels", std::array<int, 2>{}},
    {"Limit Positions", std::array<float, 2>{}},
    {"Start Move", false},
    {"Stop Move", false},
    {"Reinitialize", false},
    
  };

  o.connect("/Equipment/Move/Settings");

  midas::odb move_var = {
    {"Initializing", false},
    {"Initialized", false},
    {"Bad Destination", false},
    {"Completed", false},
    {"Moving", false},
    {"Axis Moving", std::array<bool, 2>{}},
    {"Position", std::array<float, 2>{}},
    
  };
  move_var.connect("/Equipment/Move/Variables");

  // Program restarted, so need to re-initialiaze
  move_var["Initialized"] = false;

  mOrigin[0] = 0; mOrigin[1] = 1;

  /* Initialize non-ODB variables */
  PathSize = 0;
  PathIndex = 0;
  AbortCode = AC_USER_INPUT;

  /// Check that the channel mapping is sensible!!!



  /* Set up hotlinks */
  // move_init() hotlink
  //db_open_record(hDB, pInfo->hKeyStart, &pInfo->Start, sizeof(BOOL), MODE_READ, move_init, pInfo);

  // stop_move() hotlink
  //db_open_record(hDB, pInfo->hKeyStop, &pInfo->Stop, sizeof(BOOL), MODE_READ, stop_move, pInfo);


  // reinitialize() hotlink
  //  db_open_record(hDB, pInfo->hKeyReInit, &pInfo->ReInitialize, sizeof(BOOL), MODE_READ, reinitialize, pInfo);

  // monitor() hotlink
  //db_open_record(hDB, pInfo->hKeyMMoving[0], NULL, 8 * sizeof(BOOL), MODE_READ, monitor, pInfo);
  //db_open_record(hDB, pInfo->hKeyMMoving[1], NULL, 8 * sizeof(BOOL), MODE_READ, monitor, pInfo);
  // Note: The variable hotlinked to monitor() is somewhat arbitrary, all that is 
  //	   really needed is a variable that gets periodically updated in feMove.

  // Setup start move hotlink
  dbwatch_reinit.connect("/Equipment/Move/Settings/Start Move");
  dbwatch_reinit.watch(move_init);

  // Setup stop move hotlink
  dbwatch_reinit.connect("/Equipment/Move/Settings/Stop Move");
  dbwatch_reinit.watch(stop_move);

  // Setup re-initialize() hotlink
  dbwatch_reinit.connect("/Equipment/Move/Settings/Reinitialize");
  dbwatch_reinit.watch(reinitialize);


  /* Set motor velocities and accelerations to in feMotor */
  midas::odb motor_set = {
    {"Velocity", std::array<float, 3>{}},
    {"Acceleration", std::array<float, 3>{}},
    {"Deceleration", std::array<float, 3>{}},
    
  };
  motor_set.connect("/Equipment/Motors00/Settings");


  for(int i =0; i < 2; i++){
    float velocity_in_counts = ((float)o["Velocity"][i]) * fabs( o["Motor Scaling"][i] );
    float acc_in_counts = ((float)o["Acceleration"][i]) * fabs( o["Motor Scaling"][i] );
    int ch = (int) o["Axis Channels"][i];

    if(i==0) 
      printf("Setting motor velocity = %f counts/s and acceleration = %f counts/s^2 for X axis %f\n",velocity_in_counts,acc_in_counts,  (float)o["Velocity"][i]);
    else 
      printf("Setting motor velocity = %f counts/s and acceleration = %f counts/s^2 for Y axis\n",velocity_in_counts,acc_in_counts);

    motor_set["Velocity"][ch] = velocity_in_counts;
    motor_set["Acceleration"][ch] = acc_in_counts;
    motor_set["Deceleration"][ch] = acc_in_counts;
    
    
  }



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



/*-- Move Init -----------------------------------------------------*/
// Call generate path and start the first move
void move_init(midas::odb &arg) {


  std::cout << "Value of key \"" + arg.get_full_path() + "\" changed to " << arg << std::endl;

  if(!((bool)arg)){
    printf("start move var set to false.  Nothing to do.\n");   
    return;  
  }

  printf("Starting move!\n");


  midas::odb move_set = {
    {"Destination", std::array<float, 2>{}},
    {"Velocity", std::array<float, 2>{}},
    {"Acceleration", std::array<float, 2>{}},
    {"Motor Scaling", std::array<float, 2>{}},
    {"Axis Channels", std::array<int, 2>{}},
    {"Limit Positions", std::array<float, 2>{}},
    {"Start Move", false},
    {"Stop Move", false},
    {"Reinitialize", false},
    
  };

  move_set.connect("/Equipment/Move/Settings");

  midas::odb move_var = {
    {"Initializing", false},
    {"Initialized", false},
    {"Bad Destination", false},
    {"Completed", false},
    {"Moving", false},
    {"Axis Moving", std::array<bool, 2>{}},
    {"Position", std::array<float, 2>{}},
    
  };

  move_var.connect("/Equipment/Move/Variables");

  midas::odb motor_set = {
    {"Move", std::array<bool, 3>{}},
    {"Destination", std::array<float, 3>{}},
    
  };

  motor_set.connect("/Equipment/Motors00/Settings");

  midas::odb motor_var = {
    {"Moving", std::array<bool, 3>{}},
    {"Position", std::array<float, 3>{}},
    {"Limit Neg", std::array<bool, 3>{}},
  };

  motor_var.connect("/Equipment/Motors00/Variables");


  // If motors are already moving return with an error.
  if (((bool)move_var["Moving"])) {
    cm_msg(MERROR, "move_init", "Error: Can't start move. Move already in progress.");
    return;
  }


  // Check if the motors have been initialized. If not, initialize them
  // before proceeding.
  //db_get_data(hDB, pInfo->hKeyInit, &pInfo->Initialized, &size, TID_BOOL);
  cm_msg(MDEBUG, "move_init", "Checking if motors are initialized...");
  
  if (((bool)move_var["Initilized"]) == false) {
    cm_msg(MDEBUG, "move_init", "They aren't. Running initialization.");
    initialize();

  }
  // If initialization fails, return with error
  //if (pInfo->Initialized == 0) {
  //  cm_msg(MERROR, "move_init", "Error: Can't start move. Initialization failed.");
  //  return;
  //}


  // Load input destination into pInfo->Destination
  //  size = 10 * sizeof(float);
  //db_get_data(hDB, pInfo->hKeyDest, pInfo->Destination, &size, TID_FLOAT);

  // Generate collision free path to pInfo->Destination

  // Simplify.  Just move in both axis at once
  cm_msg(MINFO, "move_init", "Generating Path");
  //  int Status = generate_path(pInfo);

  
  cm_msg(MINFO, "move_init", "Path succesfully generated");

  // Set the "completed" variable in the ODB to 0 (since our newly
  // started move is still incomplete)
  //pInfo->Completed = 0;
  //db_set_data(hDB, pInfo->hKeyCompleted, &pInfo->Completed, sizeof(BOOL), 1, TID_BOOL);

  // Move to first path index
  //  move(pInfo);


  // Set the ODB variable "Start Move" back to 0
  //pInfo->Start = 0;
  //db_set_data(hDB, pInfo->hKeyStart, &pInfo->Start, sizeof(BOOL), 1, TID_BOOL);
}




/*-- Initialize ----------------------------------------------------*/
// Move motors to limit switches. Use the motor coordinates at this
// position to determine the motor coordinates at the origin. If the
// negative limit switch is disabled, the current position is set as the
// origin for that axis.
void initialize() {


  midas::odb move_set = {
    {"Destination", std::array<float, 2>{}},
    {"Velocity", std::array<float, 2>{}},
    {"Acceleration", std::array<float, 2>{}},
    {"Motor Scaling", std::array<float, 2>{}},
    {"Axis Channels", std::array<int, 2>{}},
    {"Limit Positions", std::array<float, 2>{}},
    {"Start Move", false},
    {"Stop Move", false},
    {"Reinitialize", false},
    
  };

  move_set.connect("/Equipment/Move/Settings");

  midas::odb move_var = {
    {"Initializing", false},
    {"Initialized", false},
    {"Bad Destination", false},
    {"Completed", false},
    {"Moving", false},
    {"Axis Moving", std::array<bool, 2>{}},
    {"Position", std::array<float, 2>{}},
    
  };

  move_var.connect("/Equipment/Move/Variables");

  midas::odb motor_set = {
    {"Move", std::array<bool, 3>{}},
    {"Destination", std::array<float, 3>{}},
    
  };

  motor_set.connect("/Equipment/Motors00/Settings");

  midas::odb motor_var = {
    {"Moving", std::array<bool, 3>{}},
    {"Position", std::array<float, 3>{}},
    {"Limit Neg", std::array<bool, 3>{}},
  };

  motor_var.connect("/Equipment/Motors00/Variables");


  
  cm_msg(MINFO, "initialize", "Initializing motors");

  move_var["Initializing"] = true;
  move_var["Initialized"] = false;
  
  int exitFlag = 0;

  // Set motors to move into their negative limit switches, if they are enabled 
  for(int i = 0; i < 2; i++){
    
    BOOL tempNegLimitEnabled = 0;
    
    // Ignore disabled axes (LimPos==9999)
    if (((int)move_set["Limit Positions"][i]) == 9999) {
      cm_msg(MINFO, "initialize", "Negative limit switch for axis %i disabled. Axis will not be initialized.", i);
      continue;
    }

    int ch = (int) move_set["Axis Channels"][i];
    
    double tempPos = 500 * fabs((float)(move_set["Motor Scaling"][i]));  // seems this should be negative???
    tempNegLimitEnabled = 1;
    cm_msg(MINFO, "initialize",
	   "Negative limit switch for axis %i enabled. Axis will be initialized; use position %f.", i, tempPos);
   
    motor_set["Destination"][ch] = tempPos;


    usleep(200000);

    // Start the motor
    motor_set["Move"][ch] = true;


    // Wait for axes with enabled limit switches to hit their limits
    while (1) {


      int lastCountPos = (int)((float)motor_var["Position"][ch]);
      usleep(500000); // Approx. polling period


      bool negLimit = (bool)motor_var["Limit Neg"][ch];
      //cm_msg(MINFO, "initialize", "Polling axes %i neg limit:%i", i,negLimit);
      if(negLimit){ printf("Reached limit switch\n"); break; }
      else {
	// Check that motor still moving
	int now_pos = (int)((float)motor_var["Position"][ch]);
	if (now_pos == lastCountPos) {

	  // Recheck the negative limit 
	  usleep(500000);
	  negLimit = (bool)motor_var["Limit Neg"][ch];
	  printf("Rechecking limit axes %i %i\n", i,negLimit);
	  if(negLimit){ printf("Reached limit switch\n"); break; }

	  cm_msg(MERROR, "initialize",
		 "Axis %i not moving %i %i. Stopping initialization since limit switch must be broken.", i,now_pos,lastCountPos);
	  
	  exitFlag = 1;
	  break;
	}
      }
    }
  }



  // Wait for all the motors to stop moving (sometimes this 
  // occurs slightly after the limit switches are triggered)
  //pInfo->Moving = 1;

  usleep(100000);

  bool stillMoving = false;
  for(int i = 0; i < 2; i++){
    int ch = (int) move_set["Axis Channels"][i];    
    if(((bool)motor_var["Moving"][ch])) stillMoving = true;
  }

  printf("Is motors still running?  %i\n",stillMoving);
  if(stillMoving){
	  cm_msg(MERROR, "initialize",
		 "One axis is still moving! Error!");

  }  


  // Need to cache the origin
  for(int i = 0; i < 2; i++){
    int ch = (int) move_set["Axis Channels"][i];    
    mOrigin[i] = motor_var["Position"][ch];
  }
  printf("Origin position in counts is %f %f\n",mOrigin[0],mOrigin[1]);

  // Write current position in meters to ODB
  for(int i = 0; i < 2; i++){
    move_var["Position"][i] = (float)move_set["Limit Positions"][i];
  }

  // Set status variables
  move_var["Initializing"] = false;
  move_set["Reinitialize"] = false;

  // Indicate if the initialization succeeded or failed.
  if(exitFlag){
    move_var["Initialized"] = false;
    cm_msg(MINFO, "move_init", "Initialization of Gantries is Complete.  Initialization Failed!");
  }else{    
    move_var["Initialized"] = true;
    cm_msg(MINFO, "move_init", "Initialization of Gantries successfully completed.");
  }

  


  return;


}


/*-- Re-Initialize -------------------------------------------------*/
void reinitialize(midas::odb &arg) {


  std::cout << "Value of key \"" + arg.get_full_path() + "\" changed to " << arg << std::endl;

  if(!((bool)arg)){
    printf("reinitialize var set to false.  Nothing to do.\n");   
    return;  
  }

  printf("Starting re-intializization\n");
  initialize();


}




/*-- Move ----------------------------------------------------------*/
// Use channel_rw to send the subsequent path index to the motors
void move() {
  /*  int i;
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
  cm_msg(MINFO, "move", "Function Move is complete");*/
}

/*-- Stop_move -----------------------------------------------------*/
// Aborts a move, and prints the reason for stopping to screen
void stop_move(midas::odb &arg) {
  //  INFO *pInfo = (INFO *) data;

  /*
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
  */
}


/*-- Monitor -------------------------------------------------------*/
// Called periodically with /Motors/Variables/Moving, this function
// checks for completion of a move
void monitor(HNDLE hDB, HNDLE hKey, void *data) {

  /*
  INFO *pInfo = (INFO *) data;
  BOOL OldMov = pInfo->Moving; //Store the old value of pInfo->Moving for comparison to see if the value changes 
  int i = 0;


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

*/
}




