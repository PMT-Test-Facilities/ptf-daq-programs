/********************************************************************\

  Name:         fePhidget.c 
  Created by:   Thomas Lindner

  Contents:    Readout the phidget accelerometer/magnetometer for the 
               Phidget Spatial 1044

  Program stores the following values in ODB

  - Acceleration (in g)
  - MagField (in Gauss)
  - Tilt from horizontal (degree)   
  - Time in seconds/microseconds since program started)                        

  The program is a little crude:
   -> we register a phidget call-back that is called each time the phidget
      wants to produce data.  In that function we save the phidget information
      that we care about in global variables.
   -> then in read_trigger_event we save those values in an midas bank.
   -> the phidget call-back and read_trigger_event functions are being called
      asynchronously, which is a little pointless; this could probably be done
      better.

Update Jan 2014:
Added support for second phidget. The different instances know which USB device to connect to 
using the Phidget serial numbers.  You can now specify these phidget serial numbers using
ODB variable:
/Equipment/Phidget0[01]/Settings/SerialNumber

Update Aug 2014 (TF):
Making sure EventID of Phidget Equipment is unique to be recognized by MIDAS History. Also mfe.c adjusted
to accommodate multiple phidgets.

\********************************************************************/
#include <stdlib.h>
#include <stdio.h>
#include "midas.h"
#include "msystem.h"
#include "mcstd.h"
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <phidget22.h>
#include <stdint.h>
#include "vc_vector.h"

/* make frontend functions callable from the C framework            */
#ifdef __cplusplus
extern "C" {
#endif

/*-- Globals -------------------------------------------------------*/



/* The frontend name (client name) as seen by other MIDAS clients   */
char *frontend_name = "fePhidget_accelerometer";

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


/*-- Equipment list ------------------------------------------------*/

//#define USE_INT 1

EQUIPMENT equipment[] = {

  {"PhidgetAccel" "%02d",       	    // equipment name // TL changed this
  {11, 0,              // event ID, trigger mask 
  "SYSTEM",           // event buffer 
  EQ_PERIODIC,        // equipment type 
  0,                  // event source 
  "MIDAS",            // format 
  TRUE,               // enabled 
  RO_ALWAYS,		      // read x 
  1000,               // read every x millisec 
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

// Keep a copy of the last spatial data event.
double acceleration[3] = {0,0,0};
double mag[3] = {0,0,0};
double tilt = 0;
int etime = 0;
int etime_us = 0;

// ---------------------------------------------------------------
// Phidget Spatial setup routines.

//callback that will run if the Spatial is attached to the computer

void CCONV AttachHandler(PhidgetHandle ch, void * ctx) {

        int serialNumber;
        const char *name;

        
	Phidget_getDeviceName((PhidgetHandle)ch, &name);
        
	Phidget_getDeviceSerialNumber((PhidgetHandle)ch, &serialNumber);
  
	printf("Phidget Spatial attached. Serial Number = %i", serialNumber);
	printf("Test to ensure the device is attached");
  
  return ;

}

//callback that will run if the Spatial is detached from the computer
void CCONV DetachHandler(PhidgetHandle ch, void * ctx)  
{ 

    int serialNumber;
    const char *name;
   
    Phidget_getDeviceName((PhidgetHandle)ch, &name);
    Phidget_getDeviceSerialNumber((PhidgetHandle)ch, &serialNumber); 

    cm_msg(MINFO, "DetachHandler", "Phidget Acc detached! Serial Number = %i", serialNumber);
    return;
}


//callback that will run if the Spatial generates an error
static void CCONV onError(PhidgetHandle ch, void * ctx, Phidget_ErrorEventCode code, const char * description) 
{
    //cm_msg(MINFO,"onError", "Error",description);
  printf("Description: %s\n", description);
  printf("----------\n");
 
}


//callback that will run at datarate
//data - array of spatial event data structures that holds the spatial data packets that were sent in this event
//count - the number of spatial data event packets included in this event
// Save the information in global variables to be packed into bank later.

//accelerationx* v = vc_vector_create(0, sizeof(double), NULL);
//accelerationy* v = vc_vector_create(0, sizeof(double), NULL);
//accelerationz* v = vc_vector_create(0, sizeof(double), NULL);

//magx* v = vc_vector_create(0, sizeof(double), NULL);
//magy* v = vc_vector_create(0, sizeof(double), NULL);
//magz* v = vc_vector_create(0, sizeof(double), NULL);

static void CCONV onAccelerationChange(PhidgetAccelerometerHandle ch, void * ctx, const double temp_acceleration[3], double temp_timestamp)
{ 

    printf("Call AccelerationChange\n");
    //int count=50;
    //int i;
  
  
      acceleration[0] = temp_acceleration[0];
	
      acceleration[1] = temp_acceleration[1];
      acceleration[2] = temp_acceleration[2];
      
      etime = temp_timestamp;//.seconds;                                                                                                                                             
      etime_us =temp_timestamp;//.microseconds;      
      printf("Getting data: %f\n",acceleration[0]);
 
      //if( temp_acceleration[2] != 0)
	/*previous formula:
	//tilt = atan(sqrt(data[i]->acceleration[0]*data[i]->acceleration[0] + data[i]->acceleration[1]*data[i]->acceleration[1]) /data[i]->acceleration[2])*180/3.14159265;	
	// Get the right sign for the angle based on the sign of the Y-acceleration; happens to 
	// be switched (just depends on orientation of phidget.
	//if(data[i]->acceleration[1] >0)
	tilt = -tilt; */



}


//Display the properties of the attached phidget to the screen.  
//We will be displaying the name, serial number, version of the attached device, the number of accelerometer, gyro, and compass Axes, and the current data rate
// of the attached Spatial.
int display_properties(PhidgetHandle phid, PhidgetAccelerometerHandle phid_acc)
{
  int version;
  const char* ptr;
  int numAccelAxes;
  int serialNumber;
  const char *name;
 
  // PhidgetAccelerometerHandle accelerometer0;
  Phidget_getDeviceName((PhidgetHandle)phid, &ptr);
  Phidget_getDeviceSerialNumber((PhidgetHandle)phid, &serialNumber);
  Phidget_getDeviceVersion((PhidgetHandle)phid, &version);
  int status = PhidgetAccelerometer_getAxisCount(phid_acc, &numAccelAxes);
  // unint32_t dataRateMax, dataRateMin;
  //status = PhidgetSpatial_getMinDataInterval(spatial0, &dataRateMin);
  //status = PhidgetSpatial_getMaxDataInterval(spatial0, &dataRateMax);
 
  printf("%s\n", ptr);
  cm_msg(MINFO,"display_properties", "Successfully connected to Phidget accelerometer USB");
  cm_msg(MINFO, "display_properties", "Serial Number: %i  |  Version: %f", serialNumber, version);
  cm_msg(MINFO,"display_properties","Number of Accel Axes: %i", numAccelAxes);

  //cm_msg(MINFO,"display_properties","datarate> Max: %d  Min: %d", dataRateMax, dataRateMin); 

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



extern int frontend_index;

HNDLE   hDB=0, hFS=0;

/*-- Frontend Init -------------------------------------------------*/
// Initialize ODB variables and set up hotlinks in this function
INT frontend_init()
{

  // Initialize connection to ODB.  Get the serial number for phidget.
  char    expt_name[32];
  INT status, serial_number, size;
  size = sizeof(expt_name);
  status = db_get_value(hDB, 0, "/experiment/Name", &expt_name, &size, TID_STRING, FALSE);
  cm_msg(MINFO,"frontend_init","fePhidget Front End code for experiment %s now running ... \n",expt_name);
  
  /* get basic handle for experiment ODB */
  status=cm_get_experiment_database(&hDB, NULL);
  if(status != CM_SUCCESS)
    {
    cm_msg(MERROR,"frontend_init","Not connected to experiment");
    return CM_UNDEF_EXP;
  }


  /* check for serial_number */
  size = sizeof(serial_number);
  
  char variable_name[100];
  sprintf(variable_name,"/Equipment/PhidgetAccel%02d/Settings/SerialNumber",frontend_index); // TL changed this

  status = db_get_value(hDB, 0, variable_name, &serial_number, &size, TID_INT, TRUE);
  if(status != DB_SUCCESS)
  {
    cm_msg(MERROR,"frontend_init","cannot GET /Equipment/PhidgetAccelXX/Settings/SerialNumber");
    return FE_ERR_ODB;
  }
  printf("Connected to phidget with serial number %i \n",serial_number);



  // --------------------------------- ---------------------  
  // Phidget Spatial initialization; define the call-backs.
  // Basically all copied from Spatial-Simple example program.

  //int result;
  //const char *err;

  //Declare a spatial handle
  //PhidgetSpatialHandle spatial = 0;
  PhidgetAccelerometerHandle accelerometer0;
  

  //create the spatial object
  //int serial_number=
  PhidgetAccelerometer_create(&accelerometer0);
  Phidget_setDeviceSerialNumber((PhidgetHandle)accelerometer0, 372365);  // TL added this.

  //Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
  //Phidget_set_OnAttach_Handler((PhidgetHandle)spatial0, AttachHandler, NULL);
  //Phidget_set_OnDetach_Handler((PhidgetHandle)spatial0, DetachHandler, NULL);
  //Phidget_set_OnError_Handler((PhidgetHandle)spatial0, ErrorHandler, NULL);
 
  PhidgetAccelerometer_setOnAccelerationChangeHandler(accelerometer0, onAccelerationChange, NULL);
  // PhidgetSpatial_setOnSpatialDataHandler(ch, spatialDataHandler, NULL);
  Phidget_setOnAttachHandler((PhidgetHandle)accelerometer0, AttachHandler, NULL);
  Phidget_setOnDetachHandler((PhidgetHandle)accelerometer0, DetachHandler, NULL);
  Phidget_setOnErrorHandler((PhidgetHandle)accelerometer0, onError, NULL);


  //Registers a callback that will run according to the set data rate that will return the spatial data changes
  //Requires the handle for the Spatial, the callback handler function that will be called, 
  //and an arbitrary pointer that will be supplied to the callback function (may be NULL)
  //PhidgetSpatial_set_OnSpatialData_Handler(spatial, SpatialDataHandler, NULL);
    
  //open the spatial object for device connections
  printf("Frontend index %i\n",frontend_index);

  // Use the serial number when opening the device.  This will ensure that this
  // front-end talks to the correct phidget.
  Phidget_open((PhidgetHandle)accelerometer0);
  PhidgetReturnCode ret;
  PhidgetReturnCode errorCode;
  const char * errorString;
  char errorDetail[100];
  size_t errorDetailLen = 100;
  

  //get the program to wait for a spatial device to be attached
  printf("Waiting for acc to be attached.... \n");
  ret = Phidget_openWaitForAttachment((PhidgetHandle)accelerometer0, 5000);
  if (ret != EPHIDGET_OK) {
      Phidget_getLastError(&errorCode, &errorString, errorDetail, &errorDetailLen);
      cm_msg(MINFO,"Error: %s", errorString);
      //cm_msg(MINFO,"Error: %d", errorCode);
      exit(1);
  }
  


  
  //Display the properties of the attached spatial device
  display_properties((PhidgetHandle)accelerometer0,accelerometer0);
  
  //Set the data rate for the spatial events
  PhidgetAccelerometer_setDataInterval(accelerometer0, 320);  

  printf("Finished spatial accelerometer setup\n");
  //LocalErrorCatcher(
  //		    PhidgetManager_close(manager));
  //LocalErrorCatcher(
  //		    PhidgetManager_delete(&manager)); 
  return CM_SUCCESS;
}

/*-- Frontend Exit -------------------------------------------------*/
// Stop the motors and free all memory in this function
INT frontend_exit()
{
	return CM_SUCCESS;
}

  


/********************************************************************\
             callback routines for system transitions

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

  // Create a bank with the stored information from phidget
  bk_init32(pevent);

  double sum_mag = sqrt(mag[0]*mag[0] + mag[1]*mag[1] + mag[2]*mag[2]);
 
  double *pdata32;

  char bank_name[100];
  sprintf(bank_name,"PA%02d",frontend_index); // TL changed this

  bk_create(pevent, bank_name, TID_DOUBLE, &pdata32);
  *pdata32++ = acceleration[0];
  *pdata32++ =  acceleration[1];
  *pdata32++ =  acceleration[2];
  *pdata32++ =  mag[0] ;
  *pdata32++ =   mag[1];
  *pdata32++ =  mag[2];
  *pdata32++ =  sum_mag;
  *pdata32++ =  tilt;
  *pdata32++ =  etime;
  *pdata32++ =  etime_us;
   
  bk_close(pevent, pdata32);


  return bk_size(pevent);
}


  
