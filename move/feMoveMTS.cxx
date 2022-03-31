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

INT monitor(char *pevent, INT off);

INT read_scaler_event(char *pevent, INT off);

// Added functions
void move_init(midas::odb &arg);



void move();

void stop_move(midas::odb &arg);

int initialize();

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
            1000,              // read every x millisec
            0,                  // stop run after this event limit
            0,                  // number of sub event
            60,                  // log history every x sec
            "", "", "",},
        monitor, // readout routine
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

  mOrigin[0] = 0; mOrigin[1] = 0;

  /* Initialize non-ODB variables */
  PathSize = 0;
  PathIndex = 0;
  AbortCode = AC_USER_INPUT;

  /// Check that the channel mapping is sensible!!!



  // stop_move() hotlink
  //db_open_record(hDB, pInfo->hKeyStop, &pInfo->Stop, sizeof(BOOL), MODE_READ, stop_move, pInfo);


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


/*-- Move Init -----------------------------------------------------*/
// Call generate path and start the first move
void move_init(midas::odb &arg) {

  //  std::cout << "Value of key \"" + arg.get_full_path() + "\" changed to " << arg << std::endl;

  // Just return if start move set to false
  if(!((bool)arg)){ return;  }

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
  //  cm_msg(MDEBUG, "move_init", "Checking if motors are initialized...");  
  if (((bool)move_var["Initialized"]) == false) {
    cm_msg(MINFO, "move_init", "Motors are not initialized. Running initialization.");
    int status = initialize();

    // If initialization fails, return with error
    if(status != 0){
      cm_msg(MERROR, "move_init", "Error: Can't start move. Initialization failed.");
      return;
    }

  }



  // Calculate the destination for both axis, in Galil counts
  for(int i = 0; i < 2; i++){
    float dest = ((float)move_set["Destination"][i] - (float)move_set["Limit Positions"][i]) * (float)move_set["Motor Scaling"][i]
      + mOrigin[i];

    // Galil deals with difference from current position, so need to substract that off and set the relevant ODB variable
    int ch = (int) move_set["Axis Channels"][i];    
    float dest_cor = dest - (float)motor_var["Position"][ch];
    motor_set["Destination"][ch] = dest_cor;

    // Start the motor
    usleep(200000);
    motor_set["Move"][ch] = true;

  }


  
  move_set["Start Move"] = false;
  move_var["Moving"] = true;
 
 

}

/*-- Event readout -------------------------------------------------*/
INT monitor(char *pevent, INT off) {

  midas::odb move_set = {
    {"Motor Scaling", std::array<float, 2>{}},
    {"Axis Channels", std::array<int, 2>{}},
    {"Limit Positions", std::array<float, 2>{}},
  };

  move_set.connect("/Equipment/Move/Settings");

  midas::odb move_var = {
    {"Initialized", false},
    {"Moving", false},
    {"Position", std::array<float, 2>{}},    
  };

  move_var.connect("/Equipment/Move/Variables");

  midas::odb motor_var = {
    {"Moving", std::array<bool, 3>{}},
    {"Position", std::array<float, 3>{}},
  };

  motor_var.connect("/Equipment/Motors00/Variables");


  // Get current position from motor, figure out current position in meter
  // Only do this if motor is initialized; otherwise no meaningful results possible
  if((bool)move_var["Initialized"]){

    for(int i = 0; i < 2; i++){
      int ch = (int) move_set["Axis Channels"][i];    
      float current_position = (((float)motor_var["Position"][ch]) - mOrigin[i])/(float)move_set["Motor Scaling"][i] 
	+ (float)move_set["Limit Positions"][i];
      move_var["Position"][i] = current_position;
    }
  }else{
    move_var["Position"][0] = 0.0; move_var["Position"][1] = 0.0;
  }
  
  // Check if any axis is moving
  bool is_moving = false;
  for(int i = 0; i < 2; i++){
    int ch = (int) move_set["Axis Channels"][i];    
    if((bool)motor_var["Moving"][ch]){ is_moving = true; }
  }
   
  // Check if the gantry stopped moving
  if((bool)move_var["Moving"] and !is_moving){
      cm_msg(MINFO, "monitor","Gantry move is completed");
  }

  move_var["Moving"] = is_moving;

  //printf("Monitor: moving = %i\n",is_moving);
  return 0;
}


/*-- Initialize ----------------------------------------------------*/
// Move motors to limit switches. Use the motor coordinates at this
// position to determine the motor coordinates at the origin. If the
// negative limit switch is disabled, the current position is set as the
// origin for that axis.
// return 0 if succesful
int initialize() {


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

  


  return exitFlag;


}


/*-- Re-Initialize -------------------------------------------------*/
void reinitialize(midas::odb &arg) {


  //std::cout << "Value of key \"" + arg.get_full_path() + "\" changed to " << arg << std::endl;

  // Don't do anything if reinitialize set to 'n'
  if(!((bool)arg)){ return; }

  printf("Starting re-intializization\n");
  initialize();


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
//void monitor(HNDLE hDB, HNDLE hKey, void *data) {

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
//}




