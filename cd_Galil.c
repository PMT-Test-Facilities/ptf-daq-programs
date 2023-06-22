/**
 * This class driver communicates with the
 * Galil DMC-21x3 controller. Support for different motor
 * driver mezzanines is supported
 */

#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <math.h>
#include "midas.h"
#include "cd_Galil.h"
#include "Motors.h"
#include <unistd.h>

// Should try to control this from command line...
static int verbose = 1;

/*--------------------------------------------------------------------*/
/*   Name: INFO                                                 */
/*                                                                    */
/*   Purpose: This structure is the basic parameter definition for    */
/*   the device to be handled. The name of the structure is device    */
/*   specific. It is local to this module, but probably wise to keep  */
/*   things unique.                                                   */
/*                                                                    */
/*   The order is not relevant. Each paramotors will be allocated at   */
/*   run time (note the pointers) and freed when no longer needed.    */
/*   The layout is purely for ease of use; the actual location in     */
/*   the equipment record is determined by code in the <device>_init  */
/*   function.                                                        */

typedef struct {
  /* ODB keys */
  HNDLE hDB, hKeyRoot;
  HNDLE hKeySetDest, hKeyMove, hKeyHome, hKeyPHome, hKeyStop, hKeyIndexPosition, hKeyBounce;
  HNDLE hKeyVelocity, hKeyAcceleration, hKeyDeceleration, hKeySlope, hKeyOffset, hKeyLimitPolarity, hKeyPressThresh, hKeyEncoderPosition;
  HNDLE hKeyJogPos, hKeyJogNeg, hKeyJogVelocity;
  HNDLE hKeyMotorCurrent, hKeyHoldCurrent, hKeyPowerOn, hKeyLetter, hKeyTorque;
  HNDLE hKeyVarDest, hKeyPosition, hKeyLimitPos, hKeyLimitNeg, hKeyLimitHome, hKeyMoving;
  HNDLE hKeyEnabled, hKeyLastStop;
  HNDLE hKeyEncoder, hKeyEncSlope, hKeyEncOffset;

  HNDLE hKeyAnalog1, hKeyAnalog2, hKeyDigitalIn1, hKeyDigitalIn2, hKeyDigitalOut1, hKeyDigitalOut2;

  HNDLE hKeyStatus_in, hKeyStatus_invar, hKeyMotorType, hKeyMotorP, hKeyMotorI, hKeyMotorD, hKeyMotorER, hKeyMotorMO;

  HNDLE hKeyAdvance;

  HNDLE hKeyTurnMotorsOff;

  HNDLE hKeyEnableDO;

  /* globals */
  INT num_channels;
  INT format;
  INT last_channel;
  DWORD last_update;


  /* items in /Settings */
  char *names;
  char *letter;

  float *fDestination;
  float *fVelocity;
  float *fAcceleration;
  float *fJogVelocity;
  float *fDeceleration;
  float *fEncoderPosition;
  BOOL *bMove;
  BOOL *bAdvance;
  BOOL *bJogPos;
  BOOL *bJogNeg;
  BOOL *bHome;
  BOOL *bPHome;
  BOOL *bIndexPosition;
  BOOL *bStop;
  BOOL *bPowerOn;
  BOOL *bBounce;

  BOOL *bTurnMotorsOff;
  BOOL *bEnableDO;

  float *fSlope;
  float *fOffset;
  float *fEncSlope;
  float *fEncOffset;
  float *fPressThresh;
  INT *iLimitPolarity;
  INT *iMotorCurrent;
  INT *iHoldCurrent;
  float *fTorque;

  BOOL *bDigitalOut1;
  BOOL *bDigitalOut2;

  INT *iStatus_in;
  INT *iStatus_invar;
  float *fMotorType;
  INT *iMotorP;
  INT *iMotorI;
  INT *iMotorD;
  INT *iMotorER;
  INT *iMotorMO;


  /* items in /Variables record */
  float *fPosition;
  float *fEncoder;

  BOOL *bLimitPos;
  BOOL *bLimitNeg;
  BOOL *bLimitHome;
  BOOL *bMoving;
  BOOL *bDone;
  BOOL *bEnabled;
  INT *iLastStop;

  float *fAnalog1;
  float *fAnalog2;
  BOOL *bDigitalIn1;
  BOOL *bDigitalIn2;

  /* local variables */

  INT *iLastPosition;

  /* channel information */
  void **driver;
  INT *channel_offset;
  void **dd_info;

} INFO;

int status_inval = 0;
INFO *pMotors = NULL;

/* Function prototypes */

INT ParseData(char *str, INT *target);

INT PosCapture(EQUIPMENT *pequipment, INT i);

INT Move(EQUIPMENT *pequipment, INT i, float fDestination);

INT Advance(EQUIPMENT *pequipment, INT iLeft, INT iRight, float fDistance);

INT ReloadRecalFile(EQUIPMENT *pequipment, void *info, int Status_in_val);

INT galil_status_in_updated(INT hDB, INT hKey, void *info);

/* some of Fayaz's parameters */
enum {
  STATUS_NONE,
  STATUS_GOHOME,
  STATUS_GOBEAM,
  STATUS_RECAL,
  STATUS_SETACCEL,
  STATUS_SETDECEL,
  STATUS_SETSPEED,
  STATUS_MOVE,
  STATUS_STOP,
  STATUS_ABORT,
  STATUS_RELOADRECAL
};

float fvalue;
int ivalue;
int size;

/* Macro for simplifying call to driver */

#define DRIVER(_i) ((INT (*)(INT cmd, ...))(pInfo->driver[_i]))
#define DRIVERM(_i) ((INT (*)(INT cmd, ...))(pMotors->driver[_i]))

#ifndef abs
#define abs(a) (((a) < 0)   ? -(a) : (a))
#endif

// Complex moves state machine for XY table
#define STATE_ERROR    -1
#define STATE_IDLE      0
#define STATE_START     1
#define STATE_MOVE      2
#define STATE_MOVE_MFM  2
#define STATE_MOVE_LFM  3
#define STATE_SETTLE    4
#define STATE_ACQUIRE   5
#define STATE_FINISH    6
#define STATE_PAUSE     7

/*--------------------------------------------------------------------*/
/*   free_mem                                                         */
/*                                                                    */
/*   Purpose: This static function will free all allocated memory     */
/*   for the device at the end of its lifetime. It is crucial that    */
/*   all entries in the device structure are included here            */
/*                                                                    */
/*   Inputs: <device>_info - pointer to structure for the specific    */
/*   device type.                                                     */
/*                                                                    */
/*   Precond: None                                                    */
/*                                                                    */
/*   Outputs: None                                                    */
/*                                                                    */
/*   Postcond: All allocated memory for the device instance will be   */
/*   freed.                                                           */

static void free_mem(INFO *pInfo) {
  printf("free_mem function");

  free(pInfo->names);

  free(pInfo->driver);
  free(pInfo->dd_info);
  free(pInfo->channel_offset);

  free(pInfo);

}

/**
 * Read the stop code from the controller, parse for each axis
 * and update ODB.
 */
void galil_read_stopcode(EQUIPMENT *pequipment, int debug) {

  INT i;
  INFO *pInfo;
  HNDLE hDB;
  char command[65536];
  char response[20056];
  INT writeCount;
  size_t buffLength;
  char *presponse;

  pInfo = (INFO *) pequipment->cd_info;
  cm_get_experiment_database(&hDB, NULL);

  // Read the last stop code for all motors

  sprintf(command, "SC\r");
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_read", "Error in device driver for SC command");
    return;
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 50, ":", 500);
  response[buffLength] = 0x0;

  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MERROR, "galil_move", "Move command error in response from controller %s", response);
  }

  if (debug) {
    cm_msg(MINFO, "galil_read", "Last Stop - %s ", response);
  }

  presponse = response;
  i = 0;
  do {
    pInfo->iLastStop[i] = atoi(presponse);
    i++;
    presponse = strchr(presponse, ',');
    if ((presponse++ == NULL) && (i < pInfo->num_channels)) {
      cm_msg(MERROR, "galil_read", "Error parsing SC response");
      break;
    }
  } while (i < (pInfo->num_channels));


  return;
}


/**
 * Read all relevant info for each axis, parse and update ODB
 * Read the analog and digital inputs
 */
void galil_read(EQUIPMENT *pequipment, int channel) {
  INT status;
  INT i;
  INFO *pInfo;
  HNDLE hDB;
  char command[20056];
  char response[20056];
  INT writeCount;
  size_t buffLength;
  char *presponse;
  INT count;
  unsigned char bits[8];

  pInfo = (INFO *) pequipment->cd_info;
  cm_get_experiment_database(&hDB, NULL);


// Read all channels for channel number -1 (recursive call)
  if (channel == -1) {
    for (i = 0; i < pInfo->num_channels; i++) {
      galil_read(pequipment, i);
    }
    return;
  }

  // trigger new prompt and flush input stream each time of any extra characters
  sprintf(command, "\r");
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_read", "Error in device driver getting read sync");
    return;
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 10, ":", 500);
  response[buffLength] = 0x0;
  if (response[0] != ':') {  // there may be other characters still in buffer...
    // this may be unsolicited, and these can have upper bits set, so clear them first
    for (i = 0; i < buffLength; i++) {
      response[i] &= 0x7f;
    }
    if (strchr(response, '?') != NULL) {
      cm_msg(MERROR, "galil_read", "Sync fault - %s", response);
    } else { ;//cm_msg(MERROR, "galil_read", "Sync error very weird returned <<%s>>", response);
    }
    buffLength = DRIVER(0)(CMD_READ, pequipment->driver[0].dd_info, response, 64, 200);
    response[buffLength] = 0x0;
  }

  // read all status words and parse out
  sprintf(command, "TS\r");
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_read", "Error in device driver for TS command");
    goto CompletedTS;
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 128, ":", 500);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MERROR, "galil_read", "Response error in TS read - %s", response);
    goto CompletedTS;
  }

  // check for valid data returned

  count = 0;
  for (i = 0; i < buffLength; i++) {
    if (response[i] == ',') {
      count++;
    } else if ((isspace(response[i]) == 0) && (isdigit(response[i]) == 0) && (response[i] != ':')) {
      cm_msg(MINFO, "galil_read", "Non-digit in TS read - %d %s", i, response);
      goto CompletedTS;
    }
  }
  if ((count != 3) && (count != 7)) {
    cm_msg(MINFO, "galil_read", "Missing comma in TS read - %s", response);
    goto CompletedTS;
  }

  presponse = response;
  i = 0;
  do {
    bits[i] = atoi(presponse);
    i++;
    presponse = strchr(presponse, ',');
    if ((presponse++ == NULL) && (i < pInfo->num_channels)) {
      cm_msg(MERROR, "galil_read", "Error parsing TS response");
      break;
    }
  } while (i < (pInfo->num_channels));

//	sscanf(response, " %d, %d, %d, %d",  &bits[0], &bits[1], &bits[2], &bits[3]);

//	printf("Status %s", response);

  CompletedTS:

  for (i = 0; i < pInfo->num_channels; i++) {
    pInfo->bLimitPos[i] = !((bits[i] >> 3) & 0x1);
    pInfo->bLimitNeg[i] = !((bits[i] >> 2) & 0x1);
    pInfo->bLimitHome[i] = ((bits[i] >> 1) & 0x1);
    pInfo->bMoving[i] = (bits[i] >> 7) & 0x1;
    pInfo->bDone[i] = !pInfo->bMoving[i];
    pInfo->bEnabled[i] = (bits[i] >> 5) & 0x1;
  }


  // read all the positions and parse out each one
  sprintf(command, "RP\r");
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_read", "Error in device driver for RP command");
    goto CompletedRP;
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 128, ":", 500);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MINFO, "galil_read", "Invalid string length or error on RP read. Read %u chars. Response %s Rejected",
           buffLength, response);
    goto CompletedRP;
  }

  count = 0;
  for (i = 0; i < buffLength; i++) {
    if (response[i] == ',') {
      count++;
    } else if ((response[i] != ':') && (isdigit(response[i]) == 0) &&
               (response[i] != ',') && (response[i] != '-') && (isspace(response[i]) == 0)) {
      cm_msg(MINFO, "galil_read", "Invalid char from RP read - %u %d %s", buffLength, response[i], response);
      goto CompletedRP;
    }
  }
  if ((count != 3) && (count != 7)) {
    cm_msg(MINFO, "galil_read", "Missing comma in RP read - %s", response);
    goto CompletedRP;
  }

  presponse = response;
  i = 0;
  do {
    pInfo->fPosition[i] = atoi(presponse) * pInfo->fSlope[i];
    i++;
    presponse = strchr(presponse, ',');
    if ((presponse++ == NULL) && (i < pInfo->num_channels)) {
      cm_msg(MERROR, "galil_read", "Error parsing RP response");
      break;
    }
  } while (i < (pInfo->num_channels));


//	printf("Positions %d %d %d %d\n", pInfo->iPosition[0], pInfo->iPosition[1],
//		pInfo->iPosition[2], pInfo->iPosition[3]);

  CompletedRP:

  // read all the encoders and parse out each one
  sprintf(command, "TP\r");
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_read", "Error in device driver for TP command");
    goto CompletedTP;
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 128, ":", 500);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MINFO, "galil_read", "Invalid string length or error on TP read. Read %u chars. Response %s Rejected",
           buffLength, response);
    goto CompletedTP;
  }

  count = 0;
  for (i = 0; i < buffLength; i++) {
    if (response[i] == ',') {
      count++;
    } else if ((response[i] != ':') && (isdigit(response[i]) == 0) &&
               (response[i] != ',') && (response[i] != '-') && (isspace(response[i]) == 0)) {
      cm_msg(MINFO, "galil_read", "Invalid char from TP read - %u %d %s", buffLength, response[i], response);
      goto CompletedTP;
    }
  }
  if ((count != 3) && (count != 7)) {
    cm_msg(MINFO, "galil_read", "Missing comma in TP read - %s", response);
    goto CompletedTP;
  }

  presponse = response;
  i = 0;
  do {
    pInfo->fEncoder[i] = atoi(presponse) * pInfo->fEncSlope[i];
    i++;
    presponse = strchr(presponse, ',');
    if ((presponse++ == NULL) && (i < pInfo->num_channels)) {
      cm_msg(MERROR, "galil_read", "Error parsing TP response");
      break;
    }
  } while (i < (pInfo->num_channels));


//	printf("Positions %d %d %d %d\n", pInfo->iPosition[0], pInfo->iPosition[1],
//		pInfo->iPosition[2], pInfo->iPosition[3]);

  CompletedTP:

  galil_read_stopcode(pequipment, 0);

// Read the analog inputs

  sprintf(command, "MG @AN[%d]\r", channel + 1);
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_read", "Error in device driver for AN command");
    goto CompletedAN;
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 50, ":", 500);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MINFO, "galil_read", "Invalid response for AN read. Read: %s", response);
    goto CompletedAN;
  }
  pInfo->fAnalog1[channel] = (float) atof(response);

  sprintf(command, "MG @AN[%d]\r", channel + 5);
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_read", "Error in device driver for AN command");
    goto CompletedAN;
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 50, ":", 500);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MINFO, "galil_read", "Invalid response for AN read. Read: %s", response);
    goto CompletedAN;
  }
  pInfo->fAnalog2[channel] = (float) atof(response);

  /*
  if (channel == 0) {
     static float min;
     static float max;
     static int first = 1;
     if (first) {
        min = max = pInfo->fAnalog[channel];
        first = 0;
     }
     if (min > pInfo->fAnalog[channel]) {
        min = pInfo->fAnalog[channel];
        printf("min %f max %f\n", min, max);
     }
     if (max < pInfo->fAnalog[channel]) {
        max = pInfo->fAnalog[channel];
        printf("min %f max %f\n", min, max);
     }
  }
  */

  CompletedAN:

// Read the digital inputs

  sprintf(command, "MG @IN[%d]\r", channel + 1);
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_read", "Error in device driver for IN command");
    goto CompletedAN;
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 50, ":", 500);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MINFO, "galil_read", "Invalid string length on IN read. Read %u chars. Rejected", buffLength);
    goto CompletedAN;
  }

  pInfo->bDigitalIn1[channel] = atoi(response);

  sprintf(command, "MG @IN[%d]\r", channel + 5);
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_read", "Error in device driver for IN command");
    goto CompletedIN;
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 50, ":", 500);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MINFO, "galil_read", "Invalid string length on IN read. Read %u chars. Rejected", buffLength);
    goto CompletedIN;
  }

  pInfo->bDigitalIn2[channel] = atoi(response);

  CompletedIN:


  // Write all data into ODB

  status = db_set_data(hDB, pInfo->hKeyLimitPos,
                       pInfo->bLimitPos, pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
  status = db_set_data(hDB, pInfo->hKeyLimitNeg,
                       pInfo->bLimitNeg, pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
  status = db_set_data(hDB, pInfo->hKeyLimitHome,
                       pInfo->bLimitHome, pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
  status = db_set_data(hDB, pInfo->hKeyMoving,
                       pInfo->bMoving, pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
  status = db_set_data(hDB, pInfo->hKeyEnabled,
                       pInfo->bEnabled, pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
  status = db_set_data(hDB, pInfo->hKeyPosition,
                       pInfo->fPosition, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  status = db_set_data(hDB, pInfo->hKeyEncoder,
                       pInfo->fEncoder, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  status = db_set_data(hDB, pInfo->hKeyLastStop,
                       pInfo->iLastStop, pInfo->num_channels * sizeof(INT), pInfo->num_channels, TID_INT);

  status = db_set_data(hDB, pInfo->hKeyAnalog1,
                       pInfo->fAnalog1, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  status = db_set_data(hDB, pInfo->hKeyAnalog2,
                       pInfo->fAnalog2, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  status = db_set_data(hDB, pInfo->hKeyDigitalIn1,
                       pInfo->bDigitalIn1, pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
  status = db_set_data(hDB, pInfo->hKeyDigitalIn2,
                       pInfo->bDigitalIn2, pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);

  pequipment->odb_out++;
}


/**
 * Demand callback for axis move
 */
void galil_move(INT hDB, INT hKey, void *info) {
  printf("galil move  Time %u\n", ss_millitime());
  INFO *pInfo;
  EQUIPMENT *pequipment;
  INT i;
  BOOL bFalse = FALSE;

  pequipment = (EQUIPMENT *) info;
  pInfo = (INFO *) pequipment->cd_info;

  // enable relay (ie disable brake) before moving
  printf("Enabling relay (disabling brake) before starting move\n");
  pInfo->bDigitalOut2[0] = 1;
  db_set_data(hDB, pInfo->hKeyDigitalOut2, pInfo->bDigitalOut2,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);


  for (i = 0; i < pInfo->num_channels; i++) {
    bFalse |= pInfo->bMove[i];
    if (pInfo->bMove[i] == TRUE) {
      INT size = sizeof(float);
      db_get_data_index(hDB, pInfo->hKeySetDest, &pInfo->fDestination[i], &size, i, TID_FLOAT);
      cm_msg(MINFO, "galil_move", "Move request for axis %i, destination=%f ", i, pInfo->fDestination[i]);
      Move(pequipment, i, pInfo->fDestination[i]);
      printf("move complete\n");
      pInfo->bMove[i] = FALSE;

    }
  }
  if (!bFalse) {  // no channels were true for a move
    return;
  }

  // clear the bits after writing command (this triggers another demand but no action)
  db_set_data(hDB, pInfo->hKeyMove, pInfo->bMove,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
}


/**
 * Demand callback for turning motors fully off.
 * Basically just calls MO (motors off)
 */
void turn_motors_off(INT hDB, INT hKey, void *info) {

  INFO *pInfo;
  EQUIPMENT *pequipment;
  // INT i;
  // BOOL bFalse = FALSE;

  pequipment = (EQUIPMENT *) info;
  pInfo = (INFO *) pequipment->cd_info;

  // Only turn off is TurnMotorsOf is y
  if (pInfo->bTurnMotorsOff[0] == TRUE) {

    char command[10024];
    // char buff[10024];
    char response[10024];
    size_t writeCount, buffLength;

    // Turn motor off (??)
    sprintf(command, "MO");
    cm_msg(MINFO, "galil_init", "Turning motors back off (command \"%s\")", command);
    strcat(command, "\r");

    buffLength = strlen(command);

    writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
    if (writeCount != buffLength) {
      cm_msg(MERROR, "galil_init", "Error in device driver - MO - CMD_WRITE");
    }
    buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 100, ":", 500);
    response[buffLength] = 0x0;
    if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
      cm_msg(MERROR, "galil_init", "Bad response from MO command: \"%s\"", response);
      cm_msg(MINFO, "galil_init", "Waiting 100 ms and trying again.");
      usleep(100000);
      buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 100, ":", 500);
      response[buffLength] = 0x0;
      if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
        cm_msg(MERROR, "galil_init", "Bad response from M0 again: \"%s\"", response);
        return;// FE_ERR_HW;
      }
      cm_msg(MINFO, "galil_init", "Second try successful. Continuing.");
    }
    // usleep(10000000);
    // Set the bit back to false.
    pInfo->bTurnMotorsOff[0] = FALSE;//Deactivate for steady point
     // clear the bits after writing command (this triggers another demand but no action)
    db_set_data(hDB, pInfo->hKeyTurnMotorsOff, pInfo->bTurnMotorsOff,
                sizeof(BOOL), 1, TID_BOOL);

  }

}



/**
 * call-back to enable/disable digital output
 */
void enable_do(INT hDB, INT hKey, void *info) {

  INFO *pInfo;
  EQUIPMENT *pequipment;

  pequipment = (EQUIPMENT *) info;
  pInfo = (INFO *) pequipment->cd_info;

  char command[10024];
  // char buff[10024];
  char response[10024];
  size_t writeCount, buffLength;

  // Only turn off is EnableDO is y
  if (pInfo->bEnableDO[0] == TRUE) {
    sprintf(command, "SB1");
    cm_msg(MINFO, "galil_init", "Enabling digital output 1 (command \"%s\")", command);
  }else{
    sprintf(command, "CB1");
    cm_msg(MINFO, "galil_init", "Disabling digital output 1 (command \"%s\")", command);
  }
  
  strcat(command, "\r");

  buffLength = strlen(command);
  
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_init", "Error in device driver - MO - CMD_WRITE");
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 100, ":", 500);
  response[buffLength] = 0x0;
  printf("Response %s\n",response);
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MERROR, "galil_init", "Bad response from SB/CB command: \"%s\"", response);
    cm_msg(MINFO, "galil_init", "Waiting 100 ms and trying again.");
    usleep(100000);
    buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 100, ":", 500);
    response[buffLength] = 0x0;
    if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
      cm_msg(MERROR, "galil_init", "Bad response from M0 again: \"%s\"", response);
      return;// FE_ERR_HW;
    }
    cm_msg(MINFO, "galil_init", "Second try successful. Continuing.");
  }else{
    printf("Good response\n");
  }
  

}


/**
 * Demand callback for AdvanceReel
 */
void galil_advance(INT hDB, INT hKey, void *info) {
  INFO *pInfo;
  EQUIPMENT *pequipment;
  BOOL bFalse = FALSE;
  float fDest;

  pequipment = (EQUIPMENT *) info;
  pInfo = (INFO *) pequipment->cd_info;

  bFalse |= pInfo->bAdvance[0];
  if (pInfo->bAdvance[0] == TRUE) {
    INT size = sizeof(float);
    cm_msg(MINFO, "galil_move", "Advance request");
    db_get_data_index(hDB, pInfo->hKeySetDest, &fDest, &size, 0, TID_FLOAT);
    Advance(pequipment, 0, 2, fDest);
    pInfo->bAdvance[0] = FALSE;
  }
  if (!bFalse) {  // no channels were true for a move
    return;
  }

  // clear the bits after writing command (this triggers another demand but no action)
  db_set_data(hDB, pInfo->hKeyAdvance, pInfo->bAdvance,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
}

/**
 * Demand callback for axis stop
 * Stop thread 1 (Advance carousel)
 */
void galil_stop(INT hDB, INT hKey, void *info) {
  INFO *pInfo;
  EQUIPMENT *pequipment;
  char command[256];
  char response[256];
  size_t writeCount, buffLength;
  BOOL bFalse = FALSE;
  unsigned int i;

  pequipment = (EQUIPMENT *) info;
  pInfo = (INFO *) pequipment->cd_info;

//   cm_msg(MINFO, "galil_stop", "galil_stop demand %d", pInfo->bStop);

  for (i = 0; i < pInfo->num_channels; i++) {
    bFalse |= pInfo->bStop[i];
    if (pInfo->bStop[i] == TRUE) {
      sprintf(command, "ST%s;HX1;HX2", pInfo->letter + i * NAME_LENGTH);
      cm_msg(MINFO, "galil_stop", "Stop request for %s (command \"%s\").", pInfo->names + i * NAME_LENGTH, command);
      strcat(command, "\r");

      buffLength = strlen(command);
      writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
      if (writeCount != buffLength) {
        cm_msg(MERROR, "galil_move", "Error in device driver for stop command.");
      }
      buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 20, ":::", 1000);
      response[buffLength] = 0x0;

      if ((buffLength != 3) || (strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
        cm_msg(MINFO, "galil_stop", "Stop response bad buffer length %u", buffLength);
      }

      pInfo->bStop[i] = FALSE;
      pInfo->bMoving[i] = FALSE;

    }
  }
  if (!bFalse) {  // no channels were true for a stop
    return;
  }

  // clear the bits after writing command (this triggers another demand but no action)
  db_set_data(hDB, pInfo->hKeyStop, pInfo->bStop,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
}

/**
 * Demand callback for axis jog positive
 */
void galil_jog_pos(INT hDB, INT hKey, void *info) {
  INFO *pInfo;
  EQUIPMENT *pequipment;
  char command[256];
  char buff[32];
  char response[256];
  size_t writeCount, buffLength;
  BOOL bFalse = FALSE;
  int i, size;

  pequipment = (EQUIPMENT *) info;
  pInfo = (INFO *) pequipment->cd_info;
  for (i = 0; i < pInfo->num_channels; i++) {
    bFalse |= pInfo->bJogPos[i];
    if (pInfo->bJogPos[i] == TRUE) {
      size = sizeof(float);
      db_get_data_index(hDB, pInfo->hKeyJogVelocity, &pInfo->fJogVelocity[i], &size, i, TID_FLOAT);
      db_get_data_index(hDB, pInfo->hKeySlope, &pInfo->fSlope[i], &size, i, TID_FLOAT);
      size = sizeof(INT);
      db_get_data_index(hDB, pInfo->hKeyLimitPolarity, &pInfo->iLimitPolarity[i], &size, i, TID_INT);
      db_get_data_index(hDB, pInfo->hKeyTorque, &pInfo->fTorque[i], &size, i, TID_FLOAT);

      sprintf(command, "CN %d,1,1,0;", pInfo->iLimitPolarity[i]);
      sprintf(buff, "TL%s=%1.3f;", pInfo->letter + i * NAME_LENGTH, pInfo->fTorque[i]);
      strcat(command, buff);
      sprintf(buff, "JG%s=%1.0f;", pInfo->letter + i * NAME_LENGTH, (fabs(pInfo->fJogVelocity[i])));
      strcat(command, buff);
      sprintf(buff, "SH %c;", 'A' + i);
      strcat(command, buff);
      sprintf(buff, "BG %c", 'A' + i);
      strcat(command, buff);
      cm_msg(MINFO, "galil_jog_pos", "Jog Pos request for %s cmd %s", pInfo->names + i * NAME_LENGTH, command);
      strcat(command, "\r");
      buffLength = strlen(command);
      writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
      if (writeCount != buffLength) {
        cm_msg(MERROR, "galil_jog_pos", "Error in device driver for jog pos command");
      }
      buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 250, "::::", 500);
      response[buffLength] = 0x0;
      if ((buffLength != 4) || (strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
        char *ptr;
        strcpy(command, "TC1\r");
        buffLength = strlen(command);
        writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
        if (writeCount != buffLength) {
          cm_msg(MERROR, "galil_jog_pos", "Error in device driver for jog pos command");
        }
        buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 250, ":", 500);
        response[buffLength] = 0x0;
        ptr = strchr(response, '\r');
        if (ptr) *ptr = 0x0;
        cm_msg(MERROR, "galil_jog_pos", "Jog pos error - %s", response);
      }
      pInfo->bJogPos[i] = FALSE;
    }
  }
  if (!bFalse) {  // no channels were true for a jog pos
    return;
  }

  // clear the bits after writing command (this triggers another demand but no action)
  db_set_data(hDB, pInfo->hKeyJogPos, pInfo->bJogPos,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
}

/**
 * Demand callback for axis jog negative
 */
void galil_jog_neg(INT hDB, INT hKey, void *info) {
  INFO *pInfo;
  EQUIPMENT *pequipment;
  char command[256];
  char buff[32];
  char response[256];
  size_t writeCount, buffLength;
  BOOL bFalse = FALSE;
  int i, size;

  pequipment = (EQUIPMENT *) info;
  pInfo = (INFO *) pequipment->cd_info;

  for (i = 0; i < pInfo->num_channels; i++) {
    bFalse |= pInfo->bJogNeg[i];
    if (pInfo->bJogNeg[i] == TRUE) {
      size = sizeof(float);
      db_get_data_index(hDB, pInfo->hKeyJogVelocity, &pInfo->fJogVelocity[i], &size, i, TID_FLOAT);
      db_get_data_index(hDB, pInfo->hKeySlope, &pInfo->fSlope[i], &size, i, TID_FLOAT);
      size = sizeof(INT);
      db_get_data_index(hDB, pInfo->hKeyLimitPolarity, &pInfo->iLimitPolarity[i], &size, i, TID_INT);
      db_get_data_index(hDB, pInfo->hKeyTorque, &pInfo->fTorque[i], &size, i, TID_FLOAT);

      sprintf(command, "CN %d,1,1,0;", pInfo->iLimitPolarity[i]);
      sprintf(buff, "TL%s=%1.3f;", pInfo->letter + i * NAME_LENGTH, pInfo->fTorque[i]);
      strcat(command, buff);
      sprintf(buff, "JG%s=%1.0f;", pInfo->letter + i * NAME_LENGTH, 0.0F - (fabs(pInfo->fJogVelocity[i])));
      strcat(command, buff);
      sprintf(buff, "SH %c;", 'A' + i);
      strcat(command, buff);
      sprintf(buff, "BG %c", 'A' + i);
      strcat(command, buff);
      cm_msg(MINFO, "galil_jog_pos", "Jog Neg request for %s cmd %s", pInfo->names + i * NAME_LENGTH, command);
      strcat(command, "\r");
      buffLength = strlen(command);
      writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
      if (writeCount != buffLength) {
        cm_msg(MERROR, "galil_jog_neg", "Error in device driver for jog neg command");
      }
      buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 6, ":::::", 500);
      response[buffLength] = 0x0;
      if ((buffLength != 4) || (strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
        char *ptr;
        strcpy(command, "TC1\r");
        buffLength = strlen(command);
        writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
        if (writeCount != buffLength) {
          cm_msg(MERROR, "galil_jog_neg", "Error in device driver for jog neg command");
        }
        buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 250, ":", 500);
        response[buffLength] = 0x0;
        ptr = strchr(response, '\r');
        if (ptr) *ptr = 0x0;
        cm_msg(MERROR, "galil_jog_neg", "Jog neg error - %s", response);
      }
      pInfo->bJogNeg[i] = FALSE;
    }
  }
  if (!bFalse) {  // no channels were true for a jog neg
    return;
  }
  // clear the bits after writing command (this triggers another demand but no action)
  db_set_data(hDB, pInfo->hKeyJogNeg, pInfo->bJogNeg,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
}

/**
 * Demand callback for axis home to home limit
 */
void galil_home(INT hDB, INT hKey, void *info) {
  INFO *pInfo;
  EQUIPMENT *pequipment;
  char command[256];
  char buff[64];
  char response[256];
  size_t writeCount, buffLength;
  BOOL bFalse = FALSE;
  int i, size;

  pequipment = (EQUIPMENT *) info;
  pInfo = (INFO *) pequipment->cd_info;

  for (i = 0; i < pInfo->num_channels; i++) {
    bFalse |= pInfo->bHome[i];
    if (pInfo->bHome[i] == TRUE) {
      size = sizeof(float);
      db_get_data_index(hDB, pInfo->hKeyJogVelocity, &pInfo->fJogVelocity[i], &size, i, TID_FLOAT);
      if ((pInfo->fJogVelocity[i] >= 0) && (pInfo->bLimitPos[i])) {
        cm_msg(MINFO, "galil_home", "Home with pos motion at limit switch not allowed");
      } else if ((pInfo->fJogVelocity[i] < 0) && (pInfo->bLimitNeg[i])) {
        cm_msg(MINFO, "galil_home", "Home with neg motion at limit switch not allowed");
      } else {
        db_get_data_index(hDB, pInfo->hKeySlope, &pInfo->fSlope[i], &size, i, TID_FLOAT);
        size = sizeof(INT);
        db_get_data_index(hDB, pInfo->hKeyLimitPolarity, &pInfo->iLimitPolarity[i], &size, i, TID_INT);
        sprintf(command, "HX1;");
        sprintf(buff, "CN %d,1,1,0;", pInfo->iLimitPolarity[i]);
        strcat(command, buff);
        sprintf(buff, "~a=\"%s\";homevelo=%1.0f;", pInfo->letter + i * NAME_LENGTH,
                (pInfo->fJogVelocity[i] / pInfo->fSlope[i]));
        strcat(command, buff);
        sprintf(buff, "XQ #ADVANCE,1;");
        strcat(command, buff);

        cm_msg(MINFO, "galil_home", "Home request for %s %s", pInfo->names + i * NAME_LENGTH, command);

        strcat(command, "\r");
        buffLength = strlen(command);
        writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
        if (writeCount != buffLength) {
          cm_msg(MERROR, "galil_jog_neg", "Error in device driver for jog neg command");
        }
        buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 10, "::::::", 1000);
        if ((buffLength != 6) || (strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
          char *ptr;
          strcpy(command, "TC1\r");
          buffLength = strlen(command);
          writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
          if (writeCount != buffLength) {
            cm_msg(MERROR, "galil_home", "Error in device driver for home command");
          }
          buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 250, ":", 500);
          response[buffLength] = 0x0;
          ptr = strchr(response, '\r');
          if (ptr) *ptr = 0x0;
          cm_msg(MERROR, "galil_home", "Home error - %s", response);
        }
      }
      pInfo->bHome[i] = FALSE;

    }
  }
  if (!bFalse) {  // no channels were true for a home
    return;
  }

  // clear the bits after writing command (this triggers another demand but no action)
  db_set_data(hDB, pInfo->hKeyHome, pInfo->bHome,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
}

/**
 * Demand callback for axis home to pressure limit
 */
void galil_phome(INT hDB, INT hKey, void *info) {
  INFO *pInfo;
  EQUIPMENT *pequipment;
  char command[256];
  char buff[32];
  char response[256];
  size_t writeCount, buffLength;
  BOOL bFalse = FALSE;
  unsigned int i;

  pequipment = (EQUIPMENT *) info;
  pInfo = (INFO *) pequipment->cd_info;

  for (i = 0; i < pInfo->num_channels; i++) {
    bFalse |= pInfo->bPHome[i];
    if (pInfo->bPHome[i] == TRUE) {
      INT size = sizeof(float);

      db_get_data_index(hDB, pInfo->hKeyPressThresh, &pInfo->fPressThresh[i], &size, i, TID_FLOAT);

      size = sizeof(INT);
      db_get_data_index(hDB, pInfo->hKeyLimitPolarity, &pInfo->iLimitPolarity[i], &size, i, TID_INT);
      sprintf(command, "CN %d,1,1,0;", pInfo->iLimitPolarity[i]);
      sprintf(buff, "~a=\"%s\";pthresh=%1.3f;", pInfo->letter + i * NAME_LENGTH, pInfo->fPressThresh[i]);
      strcat(command, buff);
      sprintf(command, "XQ #PADVANC,2;");
      strcat(command, buff);

      cm_msg(MINFO, "galil_home", "PHome request for %s %s", pInfo->names + i * NAME_LENGTH, command);
      strcat(command, "\r");
      buffLength = strlen(command);
      writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
      if (writeCount != buffLength) {
        cm_msg(MERROR, "galil_phome", "Error in device driver for phome command");
      }
      buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 10, "::::", 500);
      if ((buffLength != 4) || (strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
        char *ptr;
        strcpy(command, "TC1\r");
        buffLength = strlen(command);
        writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
        if (writeCount != buffLength) {
          cm_msg(MERROR, "galil_phome", "Error in device driver for phome command");
        }
        buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 250, ":", 500);
        response[buffLength] = 0x0;
        ptr = strchr(response, '\r');
        if (ptr) *ptr = 0x0;
        cm_msg(MERROR, "galil_pHome", "Phome error - %s", response);
      }

      pInfo->bPHome[i] = FALSE;

    }
  }
  if (!bFalse) {  // no channels were true for a home
    return;
  }

  // clear the bits after writing command (this triggers another demand but no action)
  db_set_data(hDB, pInfo->hKeyPHome, pInfo->bPHome,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
}


/**
 * Demand callback for indexing primary position
 */
void galil_index_position(INT hDB, INT hKey, void *info) {
  INFO *pInfo;
  EQUIPMENT *pequipment;
  char command[256];
  char response[256];
  size_t writeCount, buffLength;
  BOOL bFalse = FALSE;
  unsigned int i;

  pequipment = (EQUIPMENT *) info;
  pInfo = (INFO *) pequipment->cd_info;

  for (i = 0; i < pInfo->num_channels; i++) {
    bFalse |= pInfo->bIndexPosition[i];
    if (pInfo->bIndexPosition[i] == TRUE) {
      INT size = sizeof(float);
      db_get_data_index(hDB, pInfo->hKeySetDest, &pInfo->fDestination[i], &size, i, TID_FLOAT);

      sprintf(command, "DP%s=%1.0f\r", pInfo->letter + i * NAME_LENGTH, pInfo->fDestination[i] / pInfo->fSlope[i]);
      buffLength = strlen(command);
      writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
      if (writeCount != buffLength) {
        cm_msg(MERROR, "galil_index_primary", "Error in device driver for index primary command");
      }
      buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 2, ":", 500);
      response[buffLength + 1] = 0x0;
      if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
        cm_msg(MERROR, "motors_index_position", "Bad response from DP command %s", response);
        return;
      }

      pInfo->bIndexPosition[i] = FALSE;

      cm_msg(MINFO, "galil_index_primary", "Index request channel %s to %f", pInfo->names + i * NAME_LENGTH,
             pInfo->fDestination[i]);

    }
  }
  if (!bFalse) {  // no channels were true for an index primary
    return;
  }

  // clear the bits after writing command (this triggers another demand but no action)
  db_set_data(hDB, pInfo->hKeyIndexPosition, pInfo->bIndexPosition,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
}

/**
 * Demand callback for axis power on
 */
void galil_poweron(INT hDB, INT hKey, void *info) {
  INFO *pInfo;
  EQUIPMENT *pequipment;
  char command[2056];
  char response[2056];
  size_t writeCount, buffLength;
  BOOL bFalse = FALSE;
  INT i;

  pequipment = (EQUIPMENT *) info;
  pInfo = (INFO *) pequipment->cd_info;

//   cm_msg(MINFO, "galil_poweron", "galil_poweron demand %d", pInfo->bPowerOn);

  for (i = 0; i < pInfo->num_channels; i++) {
    bFalse |= pInfo->bPowerOn[i];
    sprintf(command, "%s%s", (pInfo->bPowerOn[i] ? "SH" : "MO"), pInfo->letter + i * NAME_LENGTH);
    cm_msg(MINFO, "galil_poweron", "PowerOn request for %s %s", pInfo->names + i * NAME_LENGTH, command);
    strcat(command, "\r");
    buffLength = strlen(command);
    writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
    if (writeCount != buffLength) {
      cm_msg(MERROR, "galil_move", "Error in device driver for stop command");
    }
    buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 5, ":", 500);
    response[buffLength] = 0x0;
    if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
      cm_msg(MERROR, "motors_poweron", "Bad response from PowerOn (command \"%s\")", response);
      return;
    }
  }
}


/**
 * Demand callback for digital out change
 */
void galil_digital(INT hDB, INT hKey, void *info) {
  INFO *pInfo;
  EQUIPMENT *pequipment;
  char command[256];
  char response[256];
  size_t writeCount, buffLength;
  unsigned int i;

  pequipment = (EQUIPMENT *) info;
  pInfo = (INFO *) pequipment->cd_info;

  for (i = 0; i < pInfo->num_channels; i++) {
    if (hKey == pInfo->hKeyDigitalOut1) {
      sprintf(command, "%cB %d", (pInfo->bDigitalOut1[i] ? 'S' : 'C'), i + 1);
      cm_msg(MINFO, "galil_digital", "DigitalOut1 request %s", command);
    } else if (hKey == pInfo->hKeyDigitalOut2) {
      sprintf(command, "%cB %d", (pInfo->bDigitalOut2[i] ? 'S' : 'C'), i + 9);
      cm_msg(MINFO, "galil_digital", "DigitalOut2 request %s", command);
    } else {
      cm_msg(MERROR, "galil_digital", "Invalid key for digital command");
      break;
    }

    strcat(command, "\r");
    buffLength = strlen(command);
    writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
    if (writeCount != buffLength) {
      cm_msg(MERROR, "galil_digital", "Error in device driver for digital command");
    }
    buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 5, ":", 500);
    if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
      cm_msg(MERROR, "galil_digital", "Error in command for DigitalOut response %s", response);
    }
  }
}


/**
 * Stupid function that lets us immediately enable the out2[0] output, so that the relay is definitely active before the move starts
 */
void enable_out2_0(EQUIPMENT *pequipment) {
  INFO *pInfo;
  //EQUIPMENT *pequipment;
  char command[256];
  char response[256];
  size_t writeCount, buffLength;
  unsigned int i;

  //  pequipment = (EQUIPMENT *) info;
  pInfo = (INFO *) pequipment->cd_info;

  sprintf(command,"SB 9\r");
  
  printf("output2[0] manually enabled (without callback)\n");
  //for (i = 0; i < pInfo->num_channels; i++) {
  //if (hKey == pInfo->hKeyDigitalOut1) {
  //  sprintf(command, "%cB %d", (pInfo->bDigitalOut1[i] ? 'S' : 'C'), i + 1);
  //  cm_msg(MINFO, "galil_digital", "DigitalOut1 request %s", command);
  //} else if (hKey == pInfo->hKeyDigitalOut2) {
  //  sprintf(command, "%cB %d", (pInfo->bDigitalOut2[i] ? 'S' : 'C'), i + 9);
  //  cm_msg(MINFO, "galil_digital", "DigitalOut2 request %s", command);
  //} else {
  //  cm_msg(MERROR, "galil_digital", "Invalid key for digital command");
  //  break;
  //}

  //    strcat(command, "\r");
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_digital", "Error in device driver for digital command");
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 5, ":", 500);
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MERROR, "galil_digital", "Error in command for DigitalOut response %s", response);
  }
}


/**
 * Demand callback for axis bounce
 */
void galil_bounce(INT hDB, INT hKey, void *info) {
  INFO *pInfo;
  EQUIPMENT *pequipment;
  unsigned int i;
  INT size = sizeof(float);

  pequipment = (EQUIPMENT *) info;
  pInfo = (INFO *) pequipment->cd_info;

  for (i = 0; i < pInfo->num_channels; i++) {
    if (pInfo->bBounce[i] == TRUE) {
      cm_msg(MINFO, "galil_bounce", "Bounce request for %s", pInfo->names + i * NAME_LENGTH);
      db_get_data_index(hDB, pInfo->hKeySetDest, &pInfo->fDestination[i], &size, i, TID_FLOAT);
    }
  }
}

/**
 * Idle task for bounce function
 */
void Bounce(HNDLE hDB, EQUIPMENT *pequipment, int channel) {
  INFO *pInfo;
  pInfo = (INFO *) pequipment->cd_info;

  // are we still moving? wait till next time

  if (pInfo->bMoving[channel] == TRUE) {
    return;
  }

  pInfo->fDestination[channel] = (float) (0.0 - pInfo->fDestination[channel]);
  PosCapture(pequipment, channel);
  if (pInfo->bLimitNeg[channel] || pInfo->bLimitPos[channel]) {
    cm_msg(MINFO, "galil_bounce", "Bounce won't run if on a limit switch");
    pInfo->bBounce[channel] = FALSE;
    db_set_data_index(hDB, pInfo->hKeyBounce, &pInfo->bBounce[channel], sizeof(BOOL), channel, TID_BOOL);
  } else {
    Move(pequipment, channel, pInfo->fDestination[channel]);
  }
}

/*--------------------------------------------------------------------*/
/*   Name: galil_init                                                 */
/*                                                                    */
/*   Purpose: This routine is used to initialize the instance of the  */
/*   equipment. The connection to the ODB is established, space is    */
/*   allocated to store all the paramotorss for the devices in the     */
/*   equipment. Values are initialized. The device driver is          */
/*   initialized, and a first collection of data is made.             */
/*                                                                    */
/*   Inputs: pequipment - EQUIPMENT pointer to the equipment           */
/*   instance.                                                        */
/*                                                                    */
/*   Precond:                                                         */
/*                                                                    */
/*   Outputs:                                                         */
/*                                                                    */
/*   Postcond:                                                        */

INT galil_init(EQUIPMENT *pequipment) {
  int status, size, j, index, ch_offset;
  INT i;
  char str[10000];
  char *ptr;
  HNDLE hDB, hKey;
  INFO *pInfo;
  MOTORS_SETTINGS_STR(galil_settings_str);
  FILE *f;

  char command[10024];
  char buff[10024];
  char response[10024];
  size_t writeCount, buffLength;


  /* allocate private data */
  pequipment->cd_info = calloc(1, sizeof(INFO));
  pInfo = (INFO *) pequipment->cd_info;

  /* get class driver root key */
  cm_get_experiment_database(&hDB, NULL);
  sprintf(str, "/Equipment/%s", pequipment->name);
  db_create_key(hDB, 0, str, TID_KEY);
  db_find_key(hDB, 0, str, &pInfo->hKeyRoot);

  /* save event format */
  size = sizeof(str);
  db_get_value(hDB, pInfo->hKeyRoot, "Common/Format", str, &size, TID_STRING, TRUE);

  if (equal_ustring(str, "Fixed")) {
    pInfo->format = FORMAT_FIXED;
  } else if (equal_ustring(str, "MIDAS")) {
    pInfo->format = FORMAT_MIDAS;
  } else if (equal_ustring(str, "YBOS")) {
    pInfo->format = FORMAT_YBOS;
  }

  // If Settings not created, use default from include file
  if (db_find_key(hDB, pInfo->hKeyRoot, "Settings", &hKey) != DB_SUCCESS) {
    db_create_record(hDB, pInfo->hKeyRoot, "Settings", strcomb(galil_settings_str));
  }

  /* count total number of channels */
  db_create_key(hDB, pInfo->hKeyRoot, "Settings/Channels", TID_KEY);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Channels", &hKey);

  for (i = pInfo->num_channels = 0; pequipment->driver[i].name[0]; i++) {
    /* ODB value has priority over driver list */
    size = sizeof(INT);
    db_get_value(hDB, hKey, pequipment->driver[i].name,
                 &pequipment->driver[i].channels, &size, TID_INT, TRUE);

    if (pequipment->driver[i].channels == 0) {
      pequipment->driver[i].channels = 1;
    }
    pInfo->num_channels += pequipment->driver[i].channels;
  }

  if (pInfo->num_channels == 0) {
    cm_msg(MERROR, "galil_init", "No channels found in device driver list");
    return FE_ERR_ODB;
  }

  /* Allocate memory for buffers */
  pInfo->names = (char *) calloc(pInfo->num_channels, NAME_LENGTH);
  pInfo->letter = (char *) calloc(pInfo->num_channels, NAME_LENGTH);

  // ./Settings
  pInfo->fDestination = (float *) calloc(pInfo->num_channels, sizeof(float));
  pInfo->fVelocity = (float *) calloc(pInfo->num_channels, sizeof(float));
  pInfo->fAcceleration = (float *) calloc(pInfo->num_channels, sizeof(float));
  pInfo->fDeceleration = (float *) calloc(pInfo->num_channels, sizeof(float));
  pInfo->fEncoderPosition = (float *) calloc(pInfo->num_channels, sizeof(float));
  pInfo->fJogVelocity = (float *) calloc(pInfo->num_channels, sizeof(float));

  pInfo->bMove = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->bAdvance = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->bJogPos = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->bJogNeg = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->bHome = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->bPHome = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->bIndexPosition = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->bStop = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->bPowerOn = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->bBounce = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->bTurnMotorsOff = (BOOL *) calloc(1, sizeof(BOOL));
  pInfo->bEnableDO = (BOOL *) calloc(1, sizeof(BOOL));

  pInfo->fSlope = (float *) calloc(pInfo->num_channels, sizeof(float));
  pInfo->fOffset = (float *) calloc(pInfo->num_channels, sizeof(float));
  pInfo->fEncSlope = (float *) calloc(pInfo->num_channels, sizeof(float));
  pInfo->fEncOffset = (float *) calloc(pInfo->num_channels, sizeof(float));
  pInfo->fPressThresh = (float *) calloc(pInfo->num_channels, sizeof(float));
  pInfo->iLimitPolarity = (INT *) calloc(pInfo->num_channels, sizeof(INT));
  pInfo->iMotorCurrent = (INT *) calloc(pInfo->num_channels, sizeof(INT));
  pInfo->iHoldCurrent = (INT *) calloc(pInfo->num_channels, sizeof(INT));
  pInfo->fTorque = (float *) calloc(pInfo->num_channels, sizeof(float));

  pInfo->bDigitalOut1 = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->bDigitalOut2 = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));

  pInfo->fMotorType = (float *) calloc(pInfo->num_channels, sizeof(float));
  pInfo->iMotorP = (INT *) calloc(pInfo->num_channels, sizeof(INT));
  pInfo->iMotorI = (INT *) calloc(pInfo->num_channels, sizeof(INT));
  pInfo->iMotorD = (INT *) calloc(pInfo->num_channels, sizeof(INT));
  pInfo->iMotorER = (INT *) calloc(pInfo->num_channels, sizeof(INT));
  pInfo->iMotorMO = (INT *) calloc(pInfo->num_channels, sizeof(INT));

  pInfo->iStatus_in = (INT *) calloc(1, sizeof(INT));
  pInfo->iStatus_invar = (INT *) calloc(1, sizeof(INT));

  // ./Variables
  pInfo->fPosition = (float *) calloc(pInfo->num_channels, sizeof(float));
  pInfo->fEncoder = (float *) calloc(pInfo->num_channels, sizeof(float));
  pInfo->bLimitPos = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->bLimitNeg = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->bLimitHome = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->bMoving = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->bDone = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->bEnabled = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->iLastStop = (INT *) calloc(pInfo->num_channels, sizeof(INT));

  pInfo->fAnalog1 = (float *) calloc(pInfo->num_channels, sizeof(float));
  pInfo->fAnalog2 = (float *) calloc(pInfo->num_channels, sizeof(float));
  pInfo->bDigitalIn1 = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));
  pInfo->bDigitalIn2 = (BOOL *) calloc(pInfo->num_channels, sizeof(BOOL));

  // Local variables

  pInfo->iLastPosition = (INT *) calloc(pInfo->num_channels, sizeof(INT));

  pInfo->dd_info = (void **) calloc(pInfo->num_channels, sizeof(void *));
  pInfo->channel_offset = (INT *) calloc(pInfo->num_channels, sizeof(INT));
  pInfo->driver = (void **) calloc(pInfo->num_channels, sizeof(void *));

  if (!pInfo->driver) {
    cm_msg(MERROR, "galil_init", "Not enough memory");
    return FE_ERR_ODB;
  }

  // Create ./Settings

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/Names",
                pInfo->names, pInfo->num_channels * NAME_LENGTH,
                pInfo->num_channels, TID_STRING);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/Letter",
                pInfo->letter, pInfo->num_channels * NAME_LENGTH,
                pInfo->num_channels, TID_STRING);

  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Move", &pInfo->hKeyMove);
  memset(pInfo->bMove, 0, pInfo->num_channels * sizeof(BOOL));
  db_set_data(hDB, pInfo->hKeyMove, pInfo->bMove,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);

  // galil_move
  db_open_record(hDB, pInfo->hKeyMove, pInfo->bMove, pInfo->num_channels * sizeof(BOOL),
                 MODE_READ, galil_move, pequipment);

  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Advance", &pInfo->hKeyAdvance);
  memset(pInfo->bAdvance, 0, pInfo->num_channels * sizeof(BOOL));
  db_set_data(hDB, pInfo->hKeyAdvance, pInfo->bAdvance,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);

  // galil_advance
  db_open_record(hDB, pInfo->hKeyAdvance, pInfo->bAdvance, pInfo->num_channels * sizeof(BOOL),
                 MODE_READ, galil_advance, pequipment);

  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Jog Pos", &pInfo->hKeyJogPos);
  memset(pInfo->bJogPos, 0, pInfo->num_channels * sizeof(BOOL));
  db_set_data(hDB, pInfo->hKeyJogPos, pInfo->bJogPos,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);

  // galil_jog_pos
  db_open_record(hDB, pInfo->hKeyJogPos, pInfo->bJogPos, pInfo->num_channels * sizeof(BOOL),
                 MODE_READ, galil_jog_pos, pequipment);

  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Jog Neg", &pInfo->hKeyJogNeg);
  memset(pInfo->bJogNeg, 0, pInfo->num_channels * sizeof(BOOL));
  db_set_data(hDB, pInfo->hKeyJogNeg, pInfo->bJogNeg,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);

  // galil_jog_neg
  db_open_record(hDB, pInfo->hKeyJogNeg, pInfo->bJogNeg, pInfo->num_channels * sizeof(BOOL),
                 MODE_READ, galil_jog_neg, pequipment);

  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Home", &pInfo->hKeyHome);
  memset(pInfo->bHome, 0, pInfo->num_channels * sizeof(BOOL));
  db_set_data(hDB, pInfo->hKeyHome, pInfo->bHome,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);

  // galil_home
  db_open_record(hDB, pInfo->hKeyHome, pInfo->bHome, pInfo->num_channels * sizeof(BOOL),
                 MODE_READ, galil_home, pequipment);

  db_find_key(hDB, pInfo->hKeyRoot, "Settings/PHome", &pInfo->hKeyPHome);
  memset(pInfo->bPHome, 0, pInfo->num_channels * sizeof(BOOL));
  db_set_data(hDB, pInfo->hKeyPHome, pInfo->bPHome,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
  db_open_record(hDB, pInfo->hKeyPHome, pInfo->bPHome, pInfo->num_channels * sizeof(BOOL),
                 MODE_READ, galil_phome, pequipment);

  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Index Position", &pInfo->hKeyIndexPosition);
  memset(pInfo->bIndexPosition, 0, pInfo->num_channels * sizeof(BOOL));
  db_set_data(hDB, pInfo->hKeyIndexPosition, pInfo->bIndexPosition,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);

  // galil_index_postion
  db_open_record(hDB, pInfo->hKeyIndexPosition, pInfo->bIndexPosition, pInfo->num_channels * sizeof(BOOL),
                 MODE_READ, galil_index_position, pequipment);

  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Stop", &pInfo->hKeyStop);
  memset(pInfo->bStop, 0, pInfo->num_channels * sizeof(BOOL));
  db_set_data(hDB, pInfo->hKeyStop, pInfo->bStop,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);

  // galil_stop
  db_open_record(hDB, pInfo->hKeyStop, pInfo->bStop, pInfo->num_channels * sizeof(BOOL),
                 MODE_READ, galil_stop, pequipment);

  db_find_key(hDB, pInfo->hKeyRoot, "Settings/PowerOn", &pInfo->hKeyPowerOn);
  memset(pInfo->bPowerOn, 0, pInfo->num_channels * sizeof(BOOL));
  db_set_data(hDB, pInfo->hKeyPowerOn, pInfo->bPowerOn,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);

  // galil_poweron
  db_open_record(hDB, pInfo->hKeyPowerOn, pInfo->bPowerOn, pInfo->num_channels * sizeof(BOOL),
                 MODE_READ, galil_poweron, pequipment);

  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Bounce", &pInfo->hKeyBounce);
  memset(pInfo->bBounce, 0, pInfo->num_channels * sizeof(BOOL));
  db_set_data(hDB, pInfo->hKeyBounce, pInfo->bBounce,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);

  // galil_bounce
  db_open_record(hDB, pInfo->hKeyBounce, pInfo->bBounce, pInfo->num_channels * sizeof(BOOL),
                 MODE_READ, galil_bounce, pequipment);

  // Add a hot-link for turning the motors off between moves
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/TurnMotorsOff", &pInfo->hKeyTurnMotorsOff);
  memset(pInfo->bTurnMotorsOff, 0, sizeof(BOOL));
  db_set_data(hDB, pInfo->hKeyTurnMotorsOff, pInfo->bTurnMotorsOff,
              sizeof(BOOL), 1, TID_BOOL);

  // galil turn motors off
  db_open_record(hDB, pInfo->hKeyTurnMotorsOff, pInfo->bTurnMotorsOff, sizeof(BOOL),
                 MODE_READ, turn_motors_off, pequipment);

  // Add a hot-link for enabling/disabling digital output
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/EnableDO", &pInfo->hKeyEnableDO);
  memset(pInfo->bEnableDO, 0, sizeof(BOOL));
  db_set_data(hDB, pInfo->hKeyEnableDO, pInfo->bEnableDO,
              sizeof(BOOL), 1, TID_BOOL);

  db_open_record(hDB, pInfo->hKeyEnableDO, pInfo->bEnableDO, sizeof(BOOL),
                 MODE_READ, enable_do, pequipment);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/Destination",
                pInfo->fDestination, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Destination", &pInfo->hKeySetDest);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/Velocity",
                pInfo->fVelocity, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Velocity", &pInfo->hKeyVelocity);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/Acceleration",
                pInfo->fAcceleration, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Acceleration", &pInfo->hKeyAcceleration);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/Deceleration",
                pInfo->fDeceleration, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Deceleration", &pInfo->hKeyDeceleration);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/Jog Velocity",
                pInfo->fJogVelocity, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Jog Velocity", &pInfo->hKeyJogVelocity);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/EncoderPosition",
                pInfo->fEncoderPosition, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/EncoderPosition", &pInfo->hKeyEncoderPosition);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/Slope",
                pInfo->fSlope, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Slope", &pInfo->hKeySlope);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/Offset",
                pInfo->fOffset, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Offset", &pInfo->hKeyOffset);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/Encoder Slope",
                pInfo->fEncSlope, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Encoder Slope", &pInfo->hKeyEncSlope);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/Encoder Offset",
                pInfo->fEncOffset, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Encoder Offset", &pInfo->hKeyEncOffset);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/PressThresh",
                pInfo->fPressThresh, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/PressThresh", &pInfo->hKeyPressThresh);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/LimitPolarity",
                pInfo->iLimitPolarity, pInfo->num_channels * sizeof(INT), pInfo->num_channels, TID_INT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/LimitPolarity", &pInfo->hKeyLimitPolarity);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/MotorCurrent",
                pInfo->iMotorCurrent, pInfo->num_channels * sizeof(INT), pInfo->num_channels, TID_INT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/MotorCurrent", &pInfo->hKeyMotorCurrent);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/HoldCurrent",
                pInfo->iHoldCurrent, pInfo->num_channels * sizeof(INT), pInfo->num_channels, TID_INT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/HoldCurrent", &pInfo->hKeyHoldCurrent);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/Torque",
                pInfo->fTorque, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Torque", &pInfo->hKeyTorque);

  db_find_key(hDB, pInfo->hKeyRoot, "Settings/DigitalOut1", &pInfo->hKeyDigitalOut1);
  memset(pInfo->bDigitalOut1, 0, pInfo->num_channels * sizeof(BOOL));
  db_set_data(hDB, pInfo->hKeyDigitalOut1, pInfo->bDigitalOut1,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);

  // galil_digital
  db_open_record(hDB, pInfo->hKeyDigitalOut1, pInfo->bDigitalOut1, pInfo->num_channels * sizeof(BOOL),
                 MODE_READ, galil_digital, pequipment);

  db_find_key(hDB, pInfo->hKeyRoot, "Settings/DigitalOut2", &pInfo->hKeyDigitalOut2);

  // Do initial setting of the Digital output status bits
  // Set all the outputs to 0
  memset(pInfo->bDigitalOut2, 1, pInfo->num_channels * sizeof(BOOL));
  // Set first output (DigitalOut2[0]) to 1
  pInfo->bDigitalOut2[0] = 1;
  printf("Set the first digitalout2 to true\n");
  db_set_data(hDB, pInfo->hKeyDigitalOut2, pInfo->bDigitalOut2,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);

  // galil_digital
  db_open_record(hDB, pInfo->hKeyDigitalOut2, pInfo->bDigitalOut2, pInfo->num_channels * sizeof(BOOL),
                 MODE_READ, galil_digital, pequipment);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/MotorType",
                pInfo->fMotorType, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/MotorType", &pInfo->hKeyMotorType);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/MotorP",
                pInfo->iMotorP, pInfo->num_channels * sizeof(INT), pInfo->num_channels, TID_INT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/MotorP", &pInfo->hKeyMotorP);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/MotorI",
                pInfo->iMotorI, pInfo->num_channels * sizeof(INT), pInfo->num_channels, TID_INT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/MotorI", &pInfo->hKeyMotorI);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/MotorD",
                pInfo->iMotorD, pInfo->num_channels * sizeof(INT), pInfo->num_channels, TID_INT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/MotorD", &pInfo->hKeyMotorD);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/MotorER",
                pInfo->iMotorER, pInfo->num_channels * sizeof(INT), pInfo->num_channels, TID_INT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/MotorER", &pInfo->hKeyMotorER);

  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/MotorMO",
                pInfo->iMotorMO, pInfo->num_channels * sizeof(INT), pInfo->num_channels, TID_INT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/MotorMO", &pInfo->hKeyMotorMO);

#if 0
  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/Status_in",
    pInfo->iStatus_in, sizeof(INT), 1, TID_INT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Status_in", &pInfo->hKeyStatus_in);
  db_open_record(hDB, pInfo->hKeyStatus_in, pInfo->iStatus_in,sizeof(INT),
     MODE_READ, galil_status_in_updated, pequipment);


  db_merge_data(hDB, pInfo->hKeyRoot, "Settings/Status_invar",
    pInfo->iStatus_invar, sizeof(INT), 1, TID_INT);
  db_find_key(hDB, pInfo->hKeyRoot, "Settings/Status_invar", &pInfo->hKeyStatus_invar);

#endif

  // Create ./Variables

  db_merge_data(hDB, pInfo->hKeyRoot, "Variables/Destination",
                pInfo->fDestination, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Variables/Destination", &pInfo->hKeyVarDest);

  db_merge_data(hDB, pInfo->hKeyRoot, "Variables/Position",
                pInfo->fPosition, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Variables/Position", &pInfo->hKeyPosition);

  db_merge_data(hDB, pInfo->hKeyRoot, "Variables/Encoder",
                pInfo->fEncoder, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Variables/Encoder", &pInfo->hKeyEncoder);

  db_merge_data(hDB, pInfo->hKeyRoot, "Variables/Limit Pos",
                pInfo->bLimitPos, pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
  db_find_key(hDB, pInfo->hKeyRoot, "Variables/Limit Pos", &pInfo->hKeyLimitPos);

  db_merge_data(hDB, pInfo->hKeyRoot, "Variables/Limit Neg",
                pInfo->bLimitNeg, pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
  db_find_key(hDB, pInfo->hKeyRoot, "Variables/Limit Neg", &pInfo->hKeyLimitNeg);

  db_merge_data(hDB, pInfo->hKeyRoot, "Variables/Limit Home",
                pInfo->bLimitHome, pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
  db_find_key(hDB, pInfo->hKeyRoot, "Variables/Limit Home", &pInfo->hKeyLimitHome);

  db_merge_data(hDB, pInfo->hKeyRoot, "Variables/Moving",
                pInfo->bMoving, pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
  db_find_key(hDB, pInfo->hKeyRoot, "Variables/Moving", &pInfo->hKeyMoving);

  db_merge_data(hDB, pInfo->hKeyRoot, "Variables/Enabled",
                pInfo->bEnabled, pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
  db_find_key(hDB, pInfo->hKeyRoot, "Variables/Enabled", &pInfo->hKeyEnabled);

  db_merge_data(hDB, pInfo->hKeyRoot, "Variables/LastStop",
                pInfo->iLastStop, pInfo->num_channels * sizeof(INT), pInfo->num_channels, TID_INT);
  db_find_key(hDB, pInfo->hKeyRoot, "Variables/LastStop", &pInfo->hKeyLastStop);

  db_merge_data(hDB, pInfo->hKeyRoot, "Variables/Analog1",
                pInfo->fAnalog1, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Variables/Analog1", &pInfo->hKeyAnalog1);

  db_merge_data(hDB, pInfo->hKeyRoot, "Variables/Analog2",
                pInfo->fAnalog2, pInfo->num_channels * sizeof(float), pInfo->num_channels, TID_FLOAT);
  db_find_key(hDB, pInfo->hKeyRoot, "Variables/Analog2", &pInfo->hKeyAnalog2);

  db_merge_data(hDB, pInfo->hKeyRoot, "Variables/DigitalIn1",
                pInfo->bDigitalIn1, pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
  db_find_key(hDB, pInfo->hKeyRoot, "Variables/DigitalIn1", &pInfo->hKeyDigitalIn1);

  db_merge_data(hDB, pInfo->hKeyRoot, "Variables/DigitalIn2",
                pInfo->bDigitalIn2, pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
  db_find_key(hDB, pInfo->hKeyRoot, "Variables/DigitalIn2", &pInfo->hKeyDigitalIn2);



  /*---- Initialize device drivers ----*/

  /* call init method */
  for (i = 0; pequipment->driver[i].name[0]; i++) {
    sprintf(str, "Settings/Devices/%s", pequipment->driver[i].name);
    status = db_find_key(hDB, pInfo->hKeyRoot, str, &hKey);
    if (status != DB_SUCCESS) {
      db_create_key(hDB, pInfo->hKeyRoot, str, TID_KEY);
      status = db_find_key(hDB, pInfo->hKeyRoot, str, &hKey);
      if (status != DB_SUCCESS) {
        cm_msg(MERROR, "galil_init", "Cannot create %s entry in online database", str);
        free_mem(pInfo);
        return FE_ERR_ODB;
      }
    }
    status = pequipment->driver[i].dd(CMD_INIT, hKey, &pequipment->driver[i].dd_info);
    if (status == FE_ERR_HW) {
      cm_msg(MERROR, "galil_init", "Error connecting to serial port. Is there another program using it?");
      free_mem(pInfo);
      return status;
    }
  }


  /* compose device driver channel assignment */
  for (i = 0, j = 0, index = 0, ch_offset = 0; i < pInfo->num_channels; i++, j++) {
    while (j >= pequipment->driver[index].channels &&
           pequipment->driver[index].name[0]) {
      ch_offset += j;
      index++;
      j = 0;
    }

    pInfo->driver[i] = (void *) pequipment->driver[index].dd;
    pInfo->dd_info[i] = pequipment->driver[index].dd_info;
    pInfo->channel_offset[i] = ch_offset;
  }


  // trigger new prompt and flush input stream
  sprintf(command, "\r");
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_init", "Error in device driver getting read sync");
    return FE_ERR_HW;
  }
  buffLength = DRIVER(0)(CMD_READ, pequipment->driver[0].dd_info, response, 250, 500);
  response[buffLength] = 0x0;

  // Check version of motor controller 

  sprintf(command, "%c%c\r", 0x12, 0x16); /* Control-R Control-V */
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_init", "Error in device driver for read command");
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 100, ":", 500);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MERROR, "galil_init", "Error reading controller info %s", response);
    return FE_ERR_HW;
  }

  //if (strstr(response, "DMC21")) { // Change by TL 2012/09/01: DMC response has changed?
  if (strstr(response, "DMC41")) {
    response[buffLength - 2] = 0x0;
    cm_msg(MINFO, "galil_init", "Galil controller found - %s", response);
  } else {
    cm_msg(MERROR, "galil_init", "Galil controller not found - %s", response);
    return FE_ERR_HW;
  }

  // do reset: this seems to be necessary.  If I don't do this then 
  // feMotor does not seem to get correctly reset when feMotor is restarted
  // after a motor move; in particular, the 'AG' command fails.  Not sure why.
  // TL: April 19, 2013
  sprintf(command, "RS");
  cm_msg(MINFO, "galil_init", "%s", command);
  strcat(command, "\r");

  buffLength = strlen(command);

  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_init", "Error in device driver - RS - CMD_WRITE");
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 100, ":", 500);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MERROR, "galil_init", "Bad response from RS command %s", response);
    return FE_ERR_HW;
  }

  // define the motor type
  sprintf(command, "MT ");
  for (i = 0; i < pInfo->num_channels; i++) {
    sprintf(buff, "%1.0f,", pInfo->fMotorType[i]);
    strcat(command, buff);
  }
  cm_msg(MINFO, "galil_init", "%s", command);
  strcat(command, "\r");

  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_init", "Error in device driver - CMD_WRITE");
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 100, ":", 500);
  response[buffLength] = 0x0;

  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MERROR, "galil_init", "Bad response from MT command %s", response);
    return FE_ERR_HW;
  }

  
  //define encoder type
  sprintf(command, "CE ");
  for (i = 0; i < pInfo->num_channels; i++) {
    sprintf(buff, "%1.0f,", 0);
    strcat(command, buff);
  }
  cm_msg(MINFO, "galil_init", "%s", command);
  strcat(command, "\r");

  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_init", "Error in device driver - CMD_WRITE");
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 100, ":", 500);
  response[buffLength] = 0x0;

  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MERROR, "galil_init", "Bad response from CE command %s", response);
    return FE_ERR_HW;
  }

  // Turn motor off (??)
  sprintf(command, "MO");
  cm_msg(MINFO, "galil_init", "%s", command);
  strcat(command, "\r");

  buffLength = strlen(command);

  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_init", "Error in device driver - MO - CMD_WRITE");
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 100, ":", 500);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MERROR, "galil_init", "Bad response from MO command %s", response);
    return FE_ERR_HW;
  }

  // set amplifier gain/motor current
  sprintf(command, "AG ");
  for (i = 0; i < pInfo->num_channels; i++) {
    sprintf(buff, "%d,", pInfo->iMotorCurrent[i]);
    strcat(command, buff);
  }
  cm_msg(MINFO, "galil_init", "%s", command);
  strcat(command, "\r");

  buffLength = strlen(command);

  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_init", "Error in device driver - AG - CMD_WRITE");
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 100, ":", 500);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    printf("AGGG %s\n", response);
    cm_msg(MERROR, "galil_init", "Bad response from AG command %s", response);
    return FE_ERR_HW;
  }

  // set the holding current


  sprintf(command, "LC ");
  for (i = 0; i < pInfo->num_channels; i++) {
    if (fabs(pInfo->fMotorType[i]) > 1) {
      sprintf(buff, "%d,", pInfo->iHoldCurrent[i]);
    } else {
      sprintf(buff, ",");
    }
    strcat(command, buff);
  }
  cm_msg(MINFO, "galil_init", "%s", command);
  strcat(command, "\r");

  buffLength = strlen(command);

  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_init", "Error in device driver - CMD_WRITE");
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 100, ":", 500);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MERROR, "galil_init", "Bad response from LC command %s", response);
    return FE_ERR_HW;
  }


  // disable the Off-on-Error command
  sprintf(command, "OE ");
  for (i = 0; i < pInfo->num_channels; i++) {
    sprintf(buff, "0,");
    strcat(command, buff);
  }
  cm_msg(MINFO, "galil_init", "%s", command);
  strcat(command, "\r");

  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_init", "Error in device driver - OE -CMD_WRITE");
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 100, ":", 500);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MERROR, "galil_init", "Bad response from OE command %s", response);
    return FE_ERR_HW;
  }




  // Read the state of the output bits and set ODB to match

  for (i = 0; i < pInfo->num_channels; i++) {
    sprintf(command, "MG @OUT[%d]\r", i + 1);
    buffLength = strlen(command);
    writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
    if (writeCount != buffLength) {
      cm_msg(MERROR, "galil_init", "Error in device driver for @OUT command");
    }
    buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 50, ":", 500);
    response[buffLength] = 0x0;
    if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
      cm_msg(MERROR, "galil_init", "Invalid string length on @OUT read. Read %u chars command %s", buffLength, response);
      return FE_ERR_HW;
    }
    pInfo->bDigitalOut1[i] = atoi(response);

    sprintf(command, "MG @OUT[%d]\r", i + 9);
    printf("Executing command %s \n",command);
    buffLength = strlen(command);
    writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
    if (writeCount != buffLength) {
      cm_msg(MERROR, "galil_init", "Error in device driver for @OUT command");
    }
    buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 50, ":", 500);
    response[buffLength] = 0x0;
    if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
      cm_msg(MERROR, "galil_init", "Invalid string length on @OUT read. Read %u chars command %s", buffLength, response);
      return FE_ERR_HW;
    }
    printf("default setting for this output: %s\n",response);
    pInfo->bDigitalOut2[i] = atoi(response);
  }

  // Set the output for DigitalOut2[0] to True (disable the motor) before move
  pInfo->bDigitalOut2[0] = 1;
  db_set_data(hDB, pInfo->hKeyDigitalOut1, pInfo->bDigitalOut1,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);
  db_set_data(hDB, pInfo->hKeyDigitalOut2, pInfo->bDigitalOut2,
              pInfo->num_channels * sizeof(BOOL), pInfo->num_channels, TID_BOOL);


  // force the ethernet port for all messages to our port
  sprintf(command, "WH            ");
  cm_msg(MINFO, "galil_init", "%s", command);
  strcat(command, "\r");

  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_init", "Error in device driver - CMD_WRITE");
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 100, ":", 500);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MERROR, "galil_init", "Bad response from WH command %s", response);
    return FE_ERR_HW;
  }

  ptr = strchr(response, 'H');
  if (ptr == NULL) {
    cm_msg(MERROR, "galil_init", "Bad response from WH command %s", response);
    return FE_ERR_HW;
  }

  ptr++;
  sprintf(command, "CF%c            ", *ptr);
  cm_msg(MINFO, "galil_init", "%s", command);
  strcat(command, "\r");

  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_init", "Error in device driver - CMD_WRITE");
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 100, ":", 500);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MERROR, "galil_init", "Bad response from WH command %s", response);
    return FE_ERR_HW;
  }

  // remove extra :s from input
  buffLength = DRIVER(0)(CMD_READ, pequipment->driver[0].dd_info, response, 100, 200);
  response[buffLength] = 0x0;
  if (strlen(response) != 0) {
    cm_msg(MERROR, "galil_init", "Extra responses found at end of init! %s", response);
  }


  cm_msg(MINFO, "galil_init", "Add code for current and error bands motor type dependent?");
  cm_msg(MINFO, "galil_init", "Add code for program load and execute.");


  return FE_SUCCESS;


  f = fopen("./Recal.dmc", "r");
  if (f == NULL) {
    cm_msg(MERROR, "galil_init", "Failed to open file for download to DMC controller");
    return FE_ERR_HW;
  }
  buffLength = fread(buff, sizeof(char), 2500, f);
  buff[buffLength] = 0x0;
  fclose(f);

  for (i = 0; i < buffLength; i++) {
    if (buff[i] == '\n') {
      buff[i] = '\r';
    }
  }

  sprintf(command, "DL\r%s%c\r", buff, 0x1A); // ^Z at end to indicate EOF
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_init", "Error in device driver - CMD_WRITE");
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 100, "::", 1000);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MERROR, "galil_init", "Bad response from Execute command %s", response);
    return FE_ERR_HW;
  }

  return FE_SUCCESS;
}


/*--------------------------------------------------------------------*/
/*   Name: <device>_exit                                              */
/*                                                                    */
/*   Purpose: This function frees all memory associated with the      */
/*   device and calls the exit function associated with the           */
/*   equipment device.                                                */
/*                                                                    */
/*   Inputs: pequipment - EQUIPMENT pointer                           */
/*                                                                    */
/*   Precond: None                                                    */
/*                                                                    */
/*   Outputs: INT - result code                                       */
/*                                                                    */
/*   Postcond: Device is deleted and parent exit code is run          */

INT galil_exit(EQUIPMENT *pequipment) {
  INFO *pInfo;
  INT i;
  char buff[256];
  size_t writeCount, buffLength;

  pInfo = (INFO *) pequipment->cd_info;

  // stop all motors
  sprintf(buff, "ST ABCDEFGH\r");
  buffLength = strlen(buff);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, buff, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_exit", "Error stopping all motors");
  }

  /* call exit method of device drivers */
  for (i = 0; pequipment->driver[i].dd != NULL; i++) {
    pequipment->driver[i].dd(CMD_EXIT, pequipment->driver[i].dd_info);
  }

  // free memory
  free_mem(pInfo);
  return FE_SUCCESS;
}


/*--------------------------------------------------------------------*/
/*   Name: <device>_idle                                              */
/*                                                                    */
/*   Purpose: This function is the idle task called on a regular      */
/*   interval by the front end to keep things going. Tasks that may   */
/*   need to be run that take a appreciable amount of time, such as   */
/*   ramping outputs or monitoring for change can be done here. They  */
/*   must be designed to not spend much time in this function, but    */
/*   allow regular polling to react to changes. The other option is   */
/*   to use Threads or Tasks.                                         */
/*                                                                    */
/*   Inputs: pequipment - EQUIPMENT pointer to the equipment record   */
/*                                                                    */
/*   Precond:                                                         */
/*                                                                    */
/*   Outputs: INT - result code                                       */
/*                                                                    */
/*   Postcond: This would depend on the functions called in the       */
/*   routine. Very device specific                                    */

INT galil_idle(EQUIPMENT *pequipment) {
  HNDLE hDB;
  INFO *pInfo;

  pInfo = (INFO *) pequipment->cd_info;
  cm_get_experiment_database(&hDB, NULL);

  //printf(".", ss_time());

  galil_read(pequipment, pInfo->last_channel);

  if (pInfo->bBounce[pInfo->last_channel] == TRUE) {
    Bounce(hDB, pequipment, pInfo->last_channel);
  }

  pInfo->last_channel = (pInfo->last_channel + 1) % pInfo->num_channels;

  return FE_SUCCESS;
}


/*--------------------------------------------------------------------*/
/*   Name: cd_<device>_read                                           */
/*                                                                    */
/*   Purpose: This sends the equipment to the data stream. Enabled    */
/*   by equipment paramotors RO items                                  */
/*                                                                    */
/*   Inputs: pevent - char pointer to equipment                       */
/*                                                                    */
/*   Precond: None                                                    */
/*                                                                    */
/*   Outputs: INT - size of the added data in the data stream         */
/*                                                                    */
/*   Postcond: None                                                   */

INT cd_Galil_read(char *pevent, INT off) {
  return 0;
}

/*--------------------------------------------------------------------*/
/*   Name: cd_<device>                                                */
/*                                                                    */
/*   Purpose: This function is the class driver dispatch table for    */
/*   the standard class function calls such as Init, Exit etc. The    */
/*   called functions are described above.                            */
/*                                                                    */
/*   Inputs: cmd - INT command descriptor                             */
/*   pequipment - PEQUIPMENT pointer to EQUIPMENT structure           */
/*                                                                    */
/*   Precond: The equipment instance must be valid                    */
/*                                                                    */
/*   Outputs: INT result code                                         */
/*                                                                    */
/*   Postcond: This would depend on the function requested.           */

INT cd_Galil(INT cmd, EQUIPMENT *pequipment) {//printf ("\ncd_galil\n");
  INT status = FE_SUCCESS;

  switch (cmd) {
    case CMD_INIT:
      cm_msg(MINFO, "Galil class driver", "Galil Class Driver Version %s %s", __DATE__, __TIME__);
      status = galil_init(pequipment);
      break;

    case CMD_EXIT:
      status = galil_exit(pequipment);
      break;

    case CMD_IDLE:
      status = galil_idle(pequipment);
      break;

    case CMD_START:
      break;

    default:
      cm_msg(MERROR, "Galil class driver", "Received unknown command %d", cmd);
      status = FE_ERR_DRIVER;
      break;
  }

  return status;
}

/**
 * Capture current position in latch and arm latch for next capture
 */
INT PosCapture(EQUIPMENT *pequipment, INT i) {
  HNDLE hDB;
  INFO *pInfo;
  char command[256];
  char response[256];
  size_t writeCount, buffLength;

  pInfo = (INFO *) pequipment->cd_info;
  cm_get_experiment_database(&hDB, NULL);

  sprintf(command, "RL %s\r", pInfo->letter + i * NAME_LENGTH);

  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_move", "Error in device driver for move command");
    return -1;
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 20, ":", 500);
  response[buffLength] = 0x0;

  cm_msg(MINFO, "PosCapture", "Capture %d                      ", atoi(response));

  sprintf(command, "AL %s\r", pInfo->letter + i * NAME_LENGTH);

  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_move", "Error in device driver for move command");
    return -1;
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 20, ":", 500);
  response[buffLength] = 0x0;

  return FE_SUCCESS;
}



/**
 * Single axis move function
 */
#define MOVE_MIN 5

INT Move(EQUIPMENT *pequipment, INT i, float fDestination) {
  HNDLE hDB;
  INFO *pInfo;
  char buff[32];
  char command[256];
  char response[256];
  size_t writeCount, buffLength;
  float fSteps;
  INT iMoveTime;
  INT size;

  pInfo = (INFO *) pequipment->cd_info;
  cm_get_experiment_database(&hDB, NULL);

  size = sizeof(float);
  db_get_data_index(hDB, pInfo->hKeySlope, &pInfo->fSlope[i], &size, i, TID_FLOAT);
  db_get_data_index(hDB, pInfo->hKeyAcceleration, &pInfo->fAcceleration[i], &size, i, TID_FLOAT);
  db_get_data_index(hDB, pInfo->hKeyDeceleration, &pInfo->fDeceleration[i], &size, i, TID_FLOAT);
  db_get_data_index(hDB, pInfo->hKeyVelocity, &pInfo->fVelocity[i], &size, i, TID_FLOAT);
  size = sizeof(INT);
  db_get_data_index(hDB, pInfo->hKeyLimitPolarity, &pInfo->iLimitPolarity[i], &size, i, TID_INT);

  fSteps = fDestination / pInfo->fSlope[i];
  iMoveTime = (INT)(fabs(fDestination - pInfo->fPosition[i]) / pInfo->fVelocity[i] + MOVE_MIN);

  // manually enable out2[0] output
  enable_out2_0(pequipment);

  if (verbose)
    cm_msg(MINFO, "galil_idle", "Move of %s to %f will take max %d seconds", pInfo->names + i * NAME_LENGTH,
           fDestination, iMoveTime);

  command[0] = 0x0;
  sprintf(buff, "CN %d,1,1,0;", pInfo->iLimitPolarity[i]);
  strcat(command, buff);
  sprintf(buff, "AC%s=%1.0f;", pInfo->letter + i * NAME_LENGTH, fabs(pInfo->fAcceleration[i]));
  strcat(command, buff);
  sprintf(buff, "DC%s=%1.0f;", pInfo->letter + i * NAME_LENGTH, fabs(pInfo->fDeceleration[i]));
  strcat(command, buff);
  sprintf(buff, "SP%s=%1.0f;", pInfo->letter + i * NAME_LENGTH, fabs(pInfo->fVelocity[i]));
  strcat(command, buff);
  sprintf(buff, "PR%s=%1.0f;", pInfo->letter + i * NAME_LENGTH, fSteps);
  strcat(command, buff);
  sprintf(buff, "SH %c;", 'A' + i);
  strcat(command, buff);
  sprintf(buff, "BG %c\r", 'A' + i);
  strcat(command, buff);

  if (verbose)
    cm_msg(MINFO, "galil_move", "Move command = %s", command);

  //cm_msg(MINFO,"%s",command);
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_move", "Error in device driver for move command");
    return -1;
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 20, ":::::::", 500);
  response[buffLength] = 0x0;

  if ((buffLength != 7) || (strchr(response, '?') != NULL)) {
    char *ptr;
    strcpy(command, "TC1\r");
    buffLength = strlen(command);
    writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
    if (writeCount != buffLength) {
      cm_msg(MERROR, "galil_move", "Error in device driver for move command");
    }
    buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 250, ":", 500);
    response[buffLength] = 0x0;
    ptr = strchr(response, '\r');
    if (ptr) *ptr = 0x0;
    cm_msg(MERROR, "galil_move", "Move error - %s", response);
  }

  pInfo->bDone[i] = FALSE;

  // update the Variables destination value to agree with the Settings
  db_set_data_index(hDB, pInfo->hKeyVarDest,
                    &pInfo->fDestination[i], sizeof(float), i, TID_FLOAT);


  printf("Finished move\n");
  return iMoveTime;
}


/**
 * The Advance function will take a relative distance to move with one reel advancing the distance, and
 * the other reel jogging opposite to maintain tension on the tape
 */
INT Advance(EQUIPMENT *pequipment, INT iLeft, INT iRight, float fDistance) {
  HNDLE hDB;
  INFO *pInfo;
  char buff[32];
  char command[1024];
  char response[1024];
  char *presponse;
  char szSupply[NAME_LENGTH];
  char szTakeup[NAME_LENGTH];
  size_t writeCount, buffLength;
  float encSupply, encTakeup, encDiffTakeupSupply;
  INT iSupply, iTakeup;
  float torqueSupply, torqueTakeup;
  INT size;
  INT stopCode;
  float fDest;
  float fSpeed;
  float fTorque;

  pInfo = (INFO *) pequipment->cd_info;
  cm_get_experiment_database(&hDB, NULL);


  size = pInfo->num_channels * sizeof(float);
  db_get_data(hDB, pInfo->hKeySlope, pInfo->fSlope, &size, TID_FLOAT);
  db_get_data(hDB, pInfo->hKeyAcceleration, pInfo->fAcceleration, &size, TID_FLOAT);
  db_get_data(hDB, pInfo->hKeyDeceleration, pInfo->fDeceleration, &size, TID_FLOAT);
  db_get_data(hDB, pInfo->hKeyVelocity, pInfo->fVelocity, &size, TID_FLOAT);
  db_get_data(hDB, pInfo->hKeyJogVelocity, pInfo->fJogVelocity, &size, TID_FLOAT);
  db_get_data(hDB, pInfo->hKeyTorque, pInfo->fTorque, &size, TID_FLOAT);
  db_get_data(hDB, pInfo->hKeyEncoderPosition, pInfo->fEncoderPosition, &size, TID_FLOAT);
  db_get_data(hDB, pInfo->hKeySetDest, pInfo->fDestination, &size, TID_FLOAT);
  size = pInfo->num_channels * sizeof(int);
  db_get_data(hDB, pInfo->hKeyMotorMO, pInfo->iMotorMO, &size, TID_INT);
  db_get_data(hDB, pInfo->hKeyMotorER, pInfo->iMotorER, &size, TID_INT);
  db_get_data(hDB, pInfo->hKeyMotorD, pInfo->iMotorD, &size, TID_INT);

  //    cm_msg(MINFO, "galil_idle", "AdvanceReel of %s and %s to %7.0f", pInfo->names + iLeft * NAME_LENGTH, pInfo->names + iRight * NAME_LENGTH, fDistance);

  if (fDistance > 0.0) {
    iSupply = iLeft;
    iTakeup = iRight;
  } else {
    iSupply = iRight;
    iTakeup = iLeft;
  }
  strcpy(szSupply, pInfo->letter + iSupply * NAME_LENGTH);
  strcpy(szTakeup, pInfo->letter + iTakeup * NAME_LENGTH);

  // Use torque from [0] for supply [2] for takeup
  torqueSupply = pInfo->fTorque[iLeft];
  torqueTakeup = pInfo->fTorque[iRight];


  command[0] = 0x0;
  sprintf(buff, "SH %s%s;", szSupply, szTakeup);
  strcat(command, buff);
  sprintf(buff, "TL%s=%1.3f;", szSupply, torqueSupply / 2);
  strcat(command, buff);
  sprintf(buff, "TL%s=%1.3f\r", szTakeup, torqueSupply / 2);
  strcat(command, buff);
  cm_msg(MINFO, "galil_move", "Motors started and holding tension : %s", command);
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_move", "Error in device driver for Advance SC command");
    return -1;
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 20, ":::", 500);
  response[buffLength] = 0x0;

  if ((buffLength != 3) || (strchr(response, '?') != NULL)) {
    char *ptr;
    strcpy(command, "TC1\r");
    buffLength = strlen(command);
    writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
    if (writeCount != buffLength) {
      cm_msg(MERROR, "galil_move", "Error in device driver for Advance DP command");
    }
    buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 250, ":", 500);
    response[buffLength] = 0x0;
    ptr = strchr(response, '\r');
    if (ptr) *ptr = 0x0;
    cm_msg(MERROR, "galil_move", "Advance DP error - %s", response);
  }

  command[0] = 0x0;
  strcat(command, "SB 1;SB 3\r");
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_digital", "Error in device driver for digital command");
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 5, "::", 500);
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MERROR, "galil_digital", "Error in command for DigitalOut response %s", response);
  }



  // read the encoders and parse out each one
  command[0] = 0x0;
  sprintf(command, "TP %s\r", szSupply);
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_read", "Error in device driver for TP command");
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 128, ":", 500);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MINFO, "galil_read", "Invalid string length or error on TP read. Read %u chars. Response %s Rejected",
           buffLength, response);
  }

  presponse = response;
  encSupply = (float) atoi(presponse);

  command[0] = 0x0;
  sprintf(command, "TP %s\r", szTakeup);
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_read", "Error in device driver for TP command");
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 128, ":", 500);
  response[buffLength] = 0x0;
  if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
    cm_msg(MINFO, "galil_read", "Invalid string length or error on TP read. Read %u chars. Response %s Rejected",
           buffLength, response);
  }

  presponse = response;
  encTakeup = (float) atoi(presponse);

  /*   cm_msg(MINFO, "galil_move", "Previous AdvanceReel supply enc %7.0f takeup enc %7.0f  diff %7.0f\n", pInfo->fEncoderPosition[iSupply   command[0] = 0x0;
       strcat(command, "SB 1;SB 3\r");
       buffLength = strlen(command);
       writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
       if (writeCount != buffLength) {
       cm_msg(MERROR, "galil_digital", "Error in device driver for digital command");
       }
       buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 5, "::", 500);
       if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
       cm_msg(MERROR, "galil_digital", "Error in command for DigitalOut response %s", response);
       }
       
       ],  pInfo->fEncoderPosition[iTakeup],pInfo->fEncoderPosition[iSupply] - pInfo->fEncoderPosition[iTakeup]);
       cm_msg(MINFO, "galil_move", "AdvanceReel supply enc %7.0f takeup enc %7.0f  diff %7.0f\n", encSupply, encTakeup,encSupply - encTakeup);*/
  encDiffTakeupSupply = encSupply - pInfo->fEncoderPosition[iSupply] - encTakeup + pInfo->fEncoderPosition[iTakeup];
  cm_msg(MINFO, "galil_move", "Advance: Takup-Supply Encoder Difference %7.0f", encDiffTakeupSupply);
  ss_sleep(200);

  command[0] = 0x0;
  /*   sprintf(buff, "SB 1; SB 3;WT 200;");
       strcat(command, buff);*/
  sprintf(buff, "AC%s=%1.0f;", szTakeup, fabs(pInfo->fAcceleration[iTakeup]));
  strcat(command, buff);
  sprintf(buff, "DC%s=%1.0f;", szTakeup, fabs(pInfo->fDeceleration[iTakeup]));
  strcat(command, buff);

  // Calculation of relative speed of takeup based on last move of system
  // second ratio is circumference when both reels are equally loaded
  // but this must be co
  // What do we do for initial startup?
  if (fabs(encTakeup - pInfo->fEncoderPosition[iTakeup]) > 100) {
    fSpeed =
        sqrt(fabs((encTakeup - pInfo->fEncoderPosition[iTakeup]) / (encSupply - pInfo->fEncoderPosition[iSupply]))) *
        pInfo->fVelocity[iTakeup];
    fTorque = torqueTakeup / sqrt(
        fabs((encTakeup - pInfo->fEncoderPosition[iTakeup]) / (encSupply - pInfo->fEncoderPosition[iSupply])));
  } else {
    fSpeed = pInfo->fVelocity[iTakeup];
    fTorque = torqueTakeup;
  }
  if (fSpeed > 20000) fSpeed = pInfo->fVelocity[iTakeup];  // make sure speed doesn't get too high
  if (fTorque > 8.0) fTorque = torqueTakeup;  // make sure torque doesn't get too high
  sprintf(buff, "SP%s=%1.0f;", szTakeup, fSpeed);
  strcat(command, buff);

  // Calculation of relative move of takeup based on last move of system
  // second ratio is circumference when both reels are equally loaded
  // but this must be co
  // What do we do for initial startup?

  if (fabs(encTakeup - pInfo->fEncoderPosition[iTakeup]) > 100) {
    fDest =
        sqrt(fabs((encTakeup - pInfo->fEncoderPosition[iTakeup]) / (encSupply - pInfo->fEncoderPosition[iSupply]))) *
        fDistance * pInfo->fSlope[iTakeup];
  } else {
    fDest = fDistance * pInfo->fSlope[iTakeup];
  }
  sprintf(buff, "PR%s=%1.0f;", szTakeup, fDest);
  strcat(command, buff);

  sprintf(buff, "JG%s=%s%1.0f;", szSupply, (fDistance < 0 ? "-" : ""), pInfo->fJogVelocity[iSupply]);
  strcat(command, buff);
  sprintf(buff, "TL%s=%1.3f;", szSupply, torqueSupply);
  strcat(command, buff);
  //    sprintf(buff, "IT%s=%1.3f;", szTakeup, 0.5);
  //    strcat(command, buff);

  sprintf(buff, "SH %s%s;", szSupply, szTakeup);
  strcat(command, buff);
  sprintf(buff, "BG %s;", szSupply);
  strcat(command, buff);

  sprintf(buff, "BG %s;", szTakeup);
  strcat(command, buff);

  sprintf(buff, "TL%s=%1.3f\r", szTakeup, fTorque);
  strcat(command, buff);

  cm_msg(MINFO, "galil_move", "Advance command = %s", command);

  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_move", "Error in device driver for move command");
    return -1;
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 20, "::::::::::", 500);
  response[buffLength] = 0x0;

  if ((buffLength != 10) || (strchr(response, '?') != NULL)) {
    char *ptr;
    strcpy(command, "TC1\r");
    buffLength = strlen(command);
    writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
    if (writeCount != buffLength) {
      cm_msg(MERROR, "galil_move", "Error in device driver for move command");
    }
    buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 250, ":", 500);
    response[buffLength] = 0x0;
    ptr = strchr(response, '\r');
    if (ptr) *ptr = 0x0;
    cm_msg(MERROR, "galil_move", "Move error - %s", response);
  }

  // Read stop code for finish of Advance
  //cm_msg(MINFO, "galil_move", "Now stop motors but hold tension");
  do {
    command[0] = 0x0;
    sprintf(buff, "SC%s\r", szTakeup);
    strcat(command, buff);
    buffLength = strlen(command);
    writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
    if (writeCount != buffLength) {
      cm_msg(MERROR, "galil_move", "Error in device driver for Advance SC command");
      return -1;
    }
    buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 20, ":", 500);
    response[buffLength] = 0x0;

    if ((buffLength < 3) || (strchr(response, '?') != NULL)) {
      char *ptr;
      strcpy(command, "TC1\r");
      buffLength = strlen(command);
      writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
      if (writeCount != buffLength) {
        cm_msg(MERROR, "galil_move", "Error in device driver for Advance DP command");
      }
      buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 250, ":", 500);
      response[buffLength] = 0x0;
      ptr = strchr(response, '\r');
      if (ptr) *ptr = 0x0;
      cm_msg(MERROR, "galil_move", "Advance DP error - %s", response);
    }

    stopCode = atoi(presponse);

  } while (stopCode == 0);

  if (stopCode == 1) {
    cm_msg(MINFO, "galil_move", "Advance stop code %d", stopCode);
  } else {
    cm_msg(MERROR, "galil_move", "Tape Advance error - %d", stopCode);
  }

  command[0] = 0x0;
  sprintf(buff, "ST%s;", szSupply);
  strcat(command, buff);
  sprintf(buff, "TL%s=%1.3f\r", szTakeup, torqueSupply);
  strcat(command, buff);
  cm_msg(MINFO, "galil_move", "Motors stopped and holding tension : %s", command);
  buffLength = strlen(command);
  writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
  if (writeCount != buffLength) {
    cm_msg(MERROR, "galil_move", "Error in device driver for Advance SC command");
    return -1;
  }
  buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 20, "::", 500);
  response[buffLength] = 0x0;

  if ((buffLength != 2) || (strchr(response, '?') != NULL)) {
    char *ptr;
    strcpy(command, "TC1\r");
    buffLength = strlen(command);
    writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
    if (writeCount != buffLength) {
      cm_msg(MERROR, "galil_move", "Error in device driver for Advance DP command");
    }
    buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 250, ":", 500);
    response[buffLength] = 0x0;
    ptr = strchr(response, '\r');
    if (ptr) *ptr = 0x0;
    cm_msg(MERROR, "galil_move", "Advance DP error - %s", response);
  }

  // turn motors off after tape advance if pInfo->iMotorMO[0] > 0
  if (pInfo->iMotorMO[0]) {
    command[0] = 0x0;
    sprintf(buff, "AM %s\r", szTakeup);
    strcat(command, buff);
    cm_msg(MINFO, "galil_move", "Wait for Motor end move : %s", command);
    buffLength = strlen(command);
    writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
    if (writeCount != buffLength) {
      cm_msg(MERROR, "galil_move", "Error in device driver for stop command");
    }
    buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 20, ":", 500);
    response[buffLength] = 0x0;
    if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
      cm_msg(MERROR, "galil_move", "Bad response from PowerOFF command %s", response);
    }
    ss_sleep(300);
    command[0] = 0x0;
    strcat(command, "CB 1;CB 3\r");
    buffLength = strlen(command);
    writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
    if (writeCount != buffLength) {
      cm_msg(MERROR, "galil_digital", "Error in device driver for digital command");
    }
    buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 500, "::", 500);
    response[buffLength] = 0x0;
    if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
      cm_msg(MERROR, "galil_digital", "Error in command for DigitalOut response %u", buffLength);
    }


    command[0] = 0x0;
    sprintf(buff, "WT 50;MO %s;MO %s\r", szTakeup, szSupply);
    strcat(command, buff);
    cm_msg(MINFO, "galil_move", "Motors OFF : %s", command);
    buffLength = strlen(command);
    writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, buffLength);
    if (writeCount != buffLength) {
      cm_msg(MERROR, "galil_move", "Error in device driver for stop command");
    }
    buffLength = DRIVER(0)(CMD_GETS, pequipment->driver[0].dd_info, response, 20, ":::", 500);
    response[buffLength] = 0x0;
    if ((strchr(response, ':') == NULL) || (strchr(response, '?') != NULL)) {
      cm_msg(MERROR, "galil_move", "Bad response from PowerOFF command %s", response);
    }
  }
  pInfo->fEncoderPosition[iSupply] = encSupply;
  pInfo->fEncoderPosition[iTakeup] = encTakeup;
  db_set_data(hDB, pInfo->hKeyEncoderPosition, pInfo->fEncoderPosition, pInfo->num_channels * sizeof(float),
              pInfo->num_channels, TID_FLOAT);

  if ((pInfo->iMotorER[0] > 130) && (fabs(encDiffTakeupSupply) > pInfo->iMotorD[0])) {
    pInfo->fDestination[0] = pInfo->fDestination[0] * (-1);
    db_set_data(hDB, pInfo->hKeySetDest, pInfo->fDestination, pInfo->num_channels * sizeof(float), pInfo->num_channels,
                TID_FLOAT);
    pInfo->iMotorER[0] = 0;
    cm_msg(MINFO, "galil_move", "Tape Advance direction changed : %f", pInfo->fDestination[0]);
  } else {
    pInfo->iMotorER[0]++; // increase advance counter
  }
  db_set_data(hDB, pInfo->hKeyMotorER, pInfo->iMotorER, pInfo->num_channels * sizeof(int), pInfo->num_channels,
              TID_INT);

  return 0;
}


#if 0




/**||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
FAYAZ's FUNCTION ADDITION||||||||||||||||||||||||||||||||||||||||||||||||||||||
||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||**/


/**THIS READS THE VALUE OF "STATUS IN" AND TAKES APPROPIATE ACTIONS **/

INT galil_status_in_updated(INT hDB, INT hKey, void *info) {


EQUIPMENT *pequipment;

int Status_in_val;
int size;
INT hKeyStatus;
size = sizeof(Status_in_val);
pequipment = (EQUIPMENT *) info;
cm_get_experiment_database(&hDB, &hKey);

db_find_key(hDB, 0, "Equipment/Motors/Settings/Status_in", &hKeyStatus);

db_get_value(hDB, 0, "Equipment/Motors/Settings/Status_in", &Status_in_val, &size, TID_INT,TRUE); 

printf ("%d",Status_in_val);
//   db_get_value(hDB, pInfo->hKeyRoot, "Common/Format", str, &size, TID_STRING, TRUE);

// cm_msg(MINFO, "galil_status_in_updated", "Status was updated %d",Status_in_val);

if (Status_in_val != 0){ReloadRecalFile(pequipment,info, Status_in_val);



}

return 0;

}

/**THIS reloads the Recalibration Program to the Galil controller prior to Recalibration**/

INT ReloadRecalFile (EQUIPMENT *pequipment,void *info, int Status_in_val)
{	

  FILE *pFile;
        long bufferLength;
        char buffer[1000000];
        long lSize;
        long i;

        INFO * pInfo;
        HNDLE hDB,hKeyStatus,hKey;
        char command[100000];
        char response[100000];
        INT writeCount;
        char * presponse;
        pequipment = (EQUIPMENT *) info;



        pInfo = (INFO *) pequipment->cd_info;

        cm_get_experiment_database(&hDB, NULL);

switch (Status_in_val){

  case(STATUS_RECAL):{
      cm_msg(MINFO, "galil_init", "\nReCalibrating HPGe\n");

      sprintf(command,"XQ #AUTO\r");
      bufferLength = strlen(command);


      writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, bufferLength);
      if (writeCount != bufferLength) {
      cm_msg(MERROR, "galil_init", "Error in device driver - CMD_WRITE for XQ #AUTO");
      }




      printf("\nExecuting Recalibration\n");



      sprintf(command,"XQ #RECAL,1\r");
      bufferLength = strlen(command);

      writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, bufferLength);


      printf("\nResetting the Status_in key\n");
      status_inval = 0;
      cm_get_experiment_database(&hDB, &hKey);
      /* now open run_number with automatic updates */
      db_find_key(hDB, 0, "Equipment/Motors/Settings/Status_in", &hKeyStatus);
      db_set_value(hDB,
          0,
          "Equipment/Motors/Settings/Status_in",
          &status_inval,
          sizeof(status_inval), 1,
          TID_INT);


      break;}
case(STATUS_GOHOME):
sprintf(command,"XQ #GOHOME,1\r");
      bufferLength = strlen(command);


      writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, bufferLength);
      if (writeCount != bufferLength) {
      cm_msg(MERROR, "galil_init", "Error in device driver - CMD_WRITE for XQ #GOHOME");
      }
status_inval = 0;
      cm_get_experiment_database(&hDB, &hKey);
      /* now open run_number with automatic updates */
      db_find_key(hDB, 0, "Equipment/Motors/Settings/Status_in", &hKeyStatus);
      db_set_value(hDB,
          0,
          "Equipment/Motors/Settings/Status_in",
          &status_inval,
          sizeof(status_inval), 1,
          TID_INT);



      break;

case(STATUS_GOBEAM):

sprintf(command,"XQ #GOBEAM,1\r");
      bufferLength = strlen(command);


      writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, bufferLength);
      if (writeCount != bufferLength) {
      cm_msg(MERROR, "galil_init", "Error in device driver - CMD_WRITE for XQ #GOHOME");
      }

status_inval = 0;
      cm_get_experiment_database(&hDB, &hKey);
      /* now open run_number with automatic updates */
      db_find_key(hDB, 0, "Equipment/Motors/Settings/Status_in", &hKeyStatus);
      db_set_value(hDB,
          0,
          "Equipment/Motors/Settings/Status_in",
          &status_inval,
          sizeof(status_inval), 1,
          TID_INT);


      break;
case(STATUS_SETACCEL):


size =  sizeof(float);
cm_get_experiment_database(&hDB, &hKey);

db_find_key(hDB, 0, "Equipment/Motors/Settings/Acceleration",  &pInfo->hKeyAcceleration);

db_get_data_index (hDB, pInfo->hKeyAcceleration,&fvalue,&size,4,TID_FLOAT);

ivalue = (int)fvalue;

sprintf(command,"AC,,,,%d\r",ivalue);
      bufferLength = strlen(command);


      writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, bufferLength);
      if (writeCount != bufferLength) {
      cm_msg(MERROR, "galil_init", "Error in device driver - CMD_WRITE for STATUS_SETACCEL");
      }
status_inval = 0;
      cm_get_experiment_database(&hDB, &hKey);
      /* now open run_number with automatic updates */
      db_find_key(hDB, 0, "Equipment/Motors/Settings/Status_in", &hKeyStatus);
      db_set_value(hDB,
          0,
          "Equipment/Motors/Settings/Status_in",
          &status_inval,
          sizeof(status_inval), 1,
          TID_INT);

break;
case(STATUS_SETSPEED):
size =  sizeof(float);
cm_get_experiment_database(&hDB, &hKey);

db_find_key(hDB, 0, "Equipment/Motors/Settings/Velocity",  &pInfo->hKeyVelocity);

db_get_data_index (hDB, pInfo->hKeyVelocity,&fvalue,&size,4,TID_FLOAT);
ivalue = (int)fvalue;

sprintf(command,"speed=%d\rSP,,,,%d\r",ivalue,ivalue);
      bufferLength = strlen(command);


      writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, bufferLength);
      if (writeCount != bufferLength) {
      cm_msg(MERROR, "galil_init", "Error in device driver - CMD_WRITE for STATUS_SETSPEED");
      }

status_inval = 0;
      cm_get_experiment_database(&hDB, &hKey);
      /* now open run_number with automatic updates */
      db_find_key(hDB, 0, "Equipment/Motors/Settings/Status_in", &hKeyStatus);
      db_set_value(hDB,
          0,
          "Equipment/Motors/Settings/Status_in",
          &status_inval,
          sizeof(status_inval), 1,
          TID_INT);



break;

case(STATUS_SETDECEL):{

size =  sizeof(float);
cm_get_experiment_database(&hDB, &hKey);

db_find_key(hDB, 0, "Equipment/Motors/Settings/Deceleration",  &pInfo->hKeyDeceleration);

db_get_data_index (hDB, pInfo->hKeyDeceleration,&fvalue,&size,4,TID_FLOAT);

ivalue = (int)fvalue;
printf("\nThe value of DC is IFLOAT) %f INT %d",fvalue,ivalue);

sprintf(command,"DC,,,,%d\r",ivalue);
      bufferLength = strlen(command);


      writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, bufferLength);
      if (writeCount != bufferLength) {
      cm_msg(MERROR, "galil_init", "Error in device driver - CMD_WRITE for STATUS_SETDECEL");
      }

status_inval = 0;
      cm_get_experiment_database(&hDB, &hKey);
      /* now open run_number with automatic updates */
      db_find_key(hDB, 0, "Equipment/Motors/Settings/Status_in", &hKeyStatus);
      db_set_value(hDB,
          0,
          "Equipment/Motors/Settings/Status_in",
          &status_inval,
          sizeof(status_inval), 1,
          TID_INT);

break;}

case(STATUS_MOVE):{
  int status;
  status = db_find_key(hDB, 0,"Equipment/Motors/Settings/Status_invar",&pInfo->hKeyStatus_invar);
  size = sizeof(int);
  status = db_get_value(hDB, 0,"Equipment/Motors/Settings/Status_invar", &ivalue, &size, TID_INT,FALSE);


  cm_get_experiment_database(&hDB, &hKey);


  sprintf(command,"XQ#AUTO\rPR,,,,%d\rBGE\r",ivalue);

        bufferLength = strlen(command);


        writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, bufferLength);
        if (writeCount != bufferLength) {
        cm_msg(MERROR, "galil_init", "Error in device driver - CMD_WRITE for STATUS_MOVE");
        }

      status_inval = 0;
      cm_get_experiment_database(&hDB, &hKey);
      /* now open run_number with automatic updates */
      db_find_key(hDB, 0, "Equipment/Motors/Settings/Status_in", &hKeyStatus);
      db_set_value(hDB,
          0,
          "Equipment/Motors/Settings/Status_in",
          &status_inval,
          sizeof(status_inval), 1,
          TID_INT);

      break;}

case(STATUS_STOP):
break;

case(STATUS_ABORT):sprintf(command,"AB",ivalue);

        bufferLength = strlen(command);


        writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, bufferLength);
        if (writeCount != bufferLength) {
        cm_msg(MERROR, "galil_init", "Error in device driver - CMD_WRITE for STATUS_MOVE");
        }
break;

case(STATUS_RELOADRECAL):



      pFile = fopen("Recal.dmc", "rb");
      if (pFile == NULL) {
      cm_msg(MERROR, "galil_init", "Failed to open file for download to DMC controller");
      return FE_ERR_HW;
        }
      fseek (pFile , 0 , SEEK_END);

      lSize = ftell (pFile);
      rewind (pFile);



      bufferLength = fread(buffer, sizeof(char), 99999, pFile);
      buffer[bufferLength] = 0x0;
      fclose(pFile);

      for ( i = 0; i < bufferLength; i ++) {
        if (buffer[i] == '\n') {
        buffer[i] = '\r';
              }
        }
      sprintf(command, "DL\r%s%c\r", buffer, 0x1A); // ^Z at end to indicate EOF

      bufferLength = strlen(command);

      writeCount = DRIVER(0)(CMD_WRITE, pequipment->driver[0].dd_info, command, bufferLength);


break;

default:
break;

}




}

#endif
