/********************************************************************\
fedcrc.c - MIDAS readout of DCRC waveforms, as demo'd at Nov 2012 DAQ workshop at TRIUMF
This version: January 8, 2013
It was re-optimized for speed since the inaugural version
This is the front-end for communicating with the DCRC.  
-> It uses the KOSocket class for connecting to the DCRC.
-> It currently creates a bank called SCD0 that contains the 6 waveforms
for a single DCRC.
Creators: Scott Oser (UBC) & Thomas Lindner (TRIUMF)
\********************************************************************/


#include <vector>
#include <stdio.h>
#include <algorithm>
#include <stdlib.h>
#include "midas.h"
#include <stdint.h>
#include <iostream>
#include <sstream>

/* make frontend functions callable from the C framework */
#ifdef __cplusplus
extern "C" {
#endif

/*-- Globals -------------------------------------------------------*/

/* The frontend name (client name) as seen by other MIDAS clients   */
char *frontend_name = "fedcrc";
/* The frontend file name, don't change it */
char *frontend_file_name = __FILE__;

/* frontend_loop is called periodically if this variable is TRUE    */
BOOL frontend_call_loop = FALSE;

/* a frontend status page is displayed with this frequency in ms */
INT display_period = 000;

/* maximum event size produced by this frontend */
INT max_event_size = 3 * 1024 * 1024;

/* maximum event size for fragmented events (EQ_FRAGMENTED) --- not really used here */
INT max_event_size_frag = 2 * 1024 * 1024;

/* buffer size to hold events */
INT event_buffer_size = 20 * 1000000;
void **info;
char strin[256];
HNDLE hDB, hSet;


/*-- Function declarations -----------------------------------------*/

INT frontend_init();

INT frontend_exit();

INT begin_of_run(INT run_number, char *error);

INT end_of_run(INT run_number, char *error);

INT pause_run(INT run_number, char *error);

INT resume_run(INT run_number, char *error);

INT frontend_loop();

INT read_trigger_event(char *pevent, INT off);

INT read_scaler_event(char *pevent, INT off);


/*-- Equipment list ------------------------------------------------*/

#undef USE_INT

EQUIPMENT equipment[] = {

    {"DCRC",               /* equipment name */
        {1, 0,                   /* event ID, trigger mask */
            "SYSTEM",               /* event buffer */
#ifdef USE_INT
            EQ_INTERRUPT,           /* equipment type */
#else
            EQ_PERIODIC,              /* equipment type */
#endif
            LAM_SOURCE(0, 0xFFFFFF),        /* event source crate 0, all stations */
            "MIDAS",                /* format */
            TRUE,                   /* enabled */
            RO_RUNNING,           /* read only when running */
            1000,                    /* poll for 1000ms */
            0,                      /* stop run after this event limit */
            0,                      /* number of sub events */
            0,                      /* don't log history */
            "", "", "",},
        read_trigger_event,      /* readout routine */
    },

    {""}
};

#ifdef __cplusplus
}
#endif

/********************************************************************\
              Callback routines for system transitions

  These routines are called whenever a system transition like start/
  stop of a run occurs. The routines are called on the following
  occations:

  frontend_init:  When the frontend program is started. This routine
                  should initialize the hardware.

  frontend_exit:  When the frontend program is shut down. Can be used
                  to releas any locked resources like memory, commu-
                  nications ports etc.

  begin_of_run:   When a new run is started. Clear scalers, open
                  rungates, etc.

  end_of_run:     Called on a request to stop a run. Can send
                  end-of-run event and close run gates.

  pause_run:      When a run is paused. Should disable trigger events.

  resume_run:     When a run is resumed. Should enable trigger events.
\********************************************************************/

/*-- Frontend Init -------------------------------------------------*/
// Upon init, read ODB settings and write them to DCRC
INT frontend_init() {

  return SUCCESS;
}

/*-- Frontend Exit -------------------------------------------------*/

INT frontend_exit() {
  printf("Exiting fedcrc!\n");
  return SUCCESS;
}

/*-- Begin of Run --------------------------------------------------*/
// Upon run stasrt, read ODB settings and write them to DCRC
INT begin_of_run(INT run_number, char *error) {


  return SUCCESS;
}

/*-- End of Run ----------------------------------------------------*/

INT end_of_run(INT run_number, char *error) {
  return SUCCESS;
}

/*-- Pause Run -----------------------------------------------------*/

INT pause_run(INT run_number, char *error) {
  return SUCCESS;
}

/*-- Resuem Run ----------------------------------------------------*/

INT resume_run(INT run_number, char *error) {
  return SUCCESS;
}

/*-- Frontend Loop -------------------------------------------------*/

INT frontend_loop() {
  /* if frontend_call_loop is true, this routine gets called when
     the frontend is idle or once between every event */
  return SUCCESS;
}

/*------------------------------------------------------------------*/

/********************************************************************\

  Readout routines for different events

\********************************************************************/

/*-- Trigger event routines ----------------------------------------*/
// Not currently used for DCRC readout
extern "C" {
INT poll_event(INT source, INT count, BOOL test)
/* Polling routine for events. Returns TRUE if event
   is available. If test equals TRUE, don't return. The test
   flag is used to time the polling */
{
  int i;

  for (i = 0; i < count; i++) {
//      cam_lam_read(LAM_SOURCE_CRATE(source), &lam);

//      if (lam & LAM_SOURCE_STATION(source))
    if (!test)
      return 1;
  }

  return 0;
}
}

/*-- Interrupt configuration ---------------------------------------*/
// This is not currently used by the DCRC readout
extern "C" {
INT interrupt_configure(INT cmd, INT source, POINTER_T adr) {
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
}

#include "math.h" // for RAND, and rand

double sampleNormal() {
  double u = ((double) rand() / (RAND_MAX)) * 2 - 1;
  double v = ((double) rand() / (RAND_MAX)) * 2 - 1;
  double r = u * u + v * v;
  if (r == 0 || r > 1) return sampleNormal();
  double c = sqrt(-2 * log(r) / r);
  return u * c;
}

#include <sys/time.h>

/*-- Event readout -------------------------------------------------*/
INT read_trigger_event(char *pevent, INT off) {

  /* init bank structure */
  bk_init32(pevent);

  uint32_t *pdata32;
  /* create structured ADC0 bank of double words (i.e. 4-byte words)  */
  bk_create(pevent, "ADC0", TID_DWORD, (void **)&pdata32);

  // Add a header, with number of words in event.
  // Use the top two bits to indicate different control words.
  // 11 -> 0xcXXXXXXX  : overall header
  // 01 -> 0x4XXXXXXX  : trigger header
  // 00 -> 0x0XXXXXXX  : channel header
  // 10 -> 0x8XXXXXXX  : trailer

  // Write the bank header to the bank, containing the number of triggers
  *pdata32++ = 0xfa000200;

  int sample = (int) (sampleNormal() * 5);
  if ((rand() % 100) > 80) sample += 20;
  *pdata32++ = 0xf800406b + sample;

  int sample2 = (int) (sampleNormal() * 2);
  *pdata32++ = 0xf804405e + sample2;

  *pdata32++ = 0xfcfd57e8;


  int size2 = bk_close(pevent, pdata32);

  struct timeval start, stop;
  gettimeofday(&start, NULL);
  if (0)
    printf("simdaq request: %f\n", start.tv_sec
                                   + 0.000001 * start.tv_usec);
  // close the bank
  return bk_size(pevent);
}



