/********************************************************************\

  Name:         osakadfe.c 
  Created by:   Pierre-Andre

  Contents:     Experiment specific readout code (user part) of
                Midas frontend. This example implement ...
                event" and a "scaler event" which are filled with
                random data. The trigger event is filled with
                two banks (ADC0 and TDC0), the scaler event with 
                one bank (SCLR).


  Revision history
  ------------------------------------------------------------------
  date         by    modification
  ---------    ---   ------------------------------------------------
  (25-JAN-95    SR    created)
  22-Jan-98    PAA/UG  Ohio Chamber tests in Detector facility
  11-Feb-98    UG     clear TDC with A7F2 (like ADC earlier)

\********************************************************************/

#include <stdlib.h>
#include <stdio.h>
#include "midas.h"
#include "msystem.h"
#include "mcstd.h"
#include "tcpip.h"
#include "cd_Galil.h"
#include "mfe.h"


/*-- Globals -------------------------------------------------------*/

/* The frontend name (client name) as seen by other MIDAS clients   */
const char *frontend_name = "feMotor";
/* The frontend file name, don't change it */
const char *frontend_file_name = __FILE__;

/* frontend_loop is called periodically if this variable is TRUE    */
BOOL frontend_call_loop = FALSE;


BOOL equipment_common_overwrite = FALSE;

/* a frontend status page is displayed with this frequency in ms */
//INT display_period = 3000;
INT display_period = 0;

/* maximum event size produced by this frontend */
INT max_event_size = 3000;

/* buffer size to hold events */
INT event_buffer_size = 10*3000;

/* maximum event size for fragmented events (EQ_FRAGMENTED) */
INT max_event_size_frag = 5*300*300;

/*-- Function declarations -----------------------------------------*/

INT frontend_init();
INT frontend_exit();
INT begin_of_run(INT run_number, char *error);
INT end_of_run(INT run_number, char *error);
INT pause_run(INT run_number, char *error);
INT resume_run(INT run_number, char *error);
INT frontend_loop();

INT poll_trigger_event(INT count, PTYPE test);
extern void interrupt_routine(void);
INT read_trigger_event(char *pevent, INT off);
INT read_scaler_event(char *pevent, INT off);

/*-- Equipment list ------------------------------------------------*/

//#define USE_INT 1


DEVICE_DRIVER motor_driver[] = {
  { "Motors",  tcpip, 8},
  { "" }
};

EQUIPMENT equipment[] = {

  {"Motors" "%02d",       	/* equipment name */
  {3, 0,                 /* event ID, trigger mask */
  "SYSTEM",             /* event buffer */
  EQ_SLOW,              /* equipment type */
  0,                    /* event source */
  "MIDAS",              /* format */
  TRUE,                 /* enabled */
  RO_ALWAYS,		/* read x */
  10000,                /* read every x millisec */
  0,                    /* stop run after this event limit */
  0,                    /* number of sub event */
  60,                	/* log history every x sec */
  "", "", "",},
  cd_Galil_read,      /* readout routine */
  cd_Galil,           /* class driver main routine */
  motor_driver,        	/* device driver list */
  NULL,                 /* init string */
  },

  { "" }
};


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

INT frontend_init()
{
  return CM_SUCCESS;
}

/*-- Frontend Exit -------------------------------------------------*/

INT frontend_exit()
{
  return CM_SUCCESS;
}

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

/*------------------------------------------------------------------*/

/********************************************************************\
  
  Readout routines for different events

\********************************************************************/

/*-- Trigger event routines ----------------------------------------*/

INT poll_event(INT source, INT count, BOOL test)
/* Polling routine for events. Returns TRUE if event
   is available. If test equals TRUE, don't return. The test
   flag is used to time the polling */
{
  return FALSE;
}

/*-- Interrupt configuration for trigger event ---------------------*/
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
INT read_trigger_event(char *pevent, INT off)
{
  return 0;
}

/*-- Scaler event --------------------------------------------------*/

INT read_scaler_event(char *pevent, INT off)
{
  return 0;
}

// end file
