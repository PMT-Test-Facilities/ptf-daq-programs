/********************************************************************\

  Name:         fewiener.cxx
  Created by:   K.Olchanski

  Contents:     Frontend for Wiener power supply via snmpwalk

\********************************************************************/

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <string>
#include <sys/time.h>
#include <assert.h>
#include <math.h>
#include <time.h>

#include <vector>

#include "mfe.h"
#include "midas.h"
#include "msystem.h"

/* make frontend functions callable from the C framework */

#ifndef FE_NAME
#define FE_NAME "fewiener"
#endif

#ifndef EQ_NAME
#define EQ_NAME "Wiener"
#endif

#ifndef EQ_EVID
#define EQ_EVID 1
#endif


/*-- Globals -------------------------------------------------------*/

/* The frontend name (client name) as seen by other MIDAS clients   */
   const char *frontend_name = FE_NAME;
/* The frontend file name, don't change it */
   const char *frontend_file_name = __FILE__;
/* frontend_loop is called periodically if this variable is TRUE    */
   BOOL frontend_call_loop = TRUE;

/* a frontend status page is displayed with this frequency in ms */
   INT display_period = 0;

/* maximum event size produced by this frontend */
   INT max_event_size = 200*1024;

/* maximum event size for fragmented events (EQ_FRAGMENTED) */
   INT max_event_size_frag = 1024*1024;

/* buffer size to hold events */
   INT event_buffer_size = 1*1024*1024;

BOOL equipment_common_overwrite = FALSE;
  extern HNDLE hDB;

   char eq_name[256];

/*-- Function declarations -----------------------------------------*/
  INT frontend_init();
  INT frontend_exit();
  INT begin_of_run(INT run_number, char *error);
  INT end_of_run(INT run_number, char *error);
  INT pause_run(INT run_number, char *error);
  INT resume_run(INT run_number, char *error);
  INT frontend_loop();
  
  INT read_wiener_event(char *pevent, INT off);

/*-- Equipment list ------------------------------------------------*/
  
  EQUIPMENT equipment[] = {

    {
       EQ_NAME,          /* equipment name */
       {
          EQ_EVID, (1<<EQ_EVID),  /* event ID, trigger mask */
          "",                     /* event buffer */
          EQ_PERIODIC,            /* equipment type */
          0,                      /* event source */
          "MIDAS",                /* format */
          TRUE,                   /* enabled */
          RO_ALWAYS,              /* when to read this event */
          10,                     /* poll time in milliseconds */
          0,                      /* stop run after this event limit */
          0,                      /* number of sub events */
          0,                      /* period for logging history, seconds */
          "", "", "",
       }
       ,
       read_wiener_event,         /* readout routine */
       NULL,
       NULL,
       NULL,       /* bank list */
    }
    ,
    {""}
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

#include "utils.cxx"

/*-- Global variables ----------------------------------------------*/


static std::string gWienerDev;
static std::string gSnmpwalkCommand;
static int gReadPeriod = 10;

/*-- Frontend Exit -------------------------------------------------*/

INT frontend_exit()
{
  return SUCCESS;
}

/*-- Begin of Run --------------------------------------------------*/

INT begin_of_run(INT run_number, char *error)
{
  return SUCCESS;
}

/*-- End of Run ----------------------------------------------------*/
INT end_of_run(INT run_number, char *error)
{
  return SUCCESS;
}

/*-- Pause Run -----------------------------------------------------*/
INT pause_run(INT run_number, char *error)
{
  return SUCCESS;
}

/*-- Resume Run ----------------------------------------------------*/
INT resume_run(INT run_number, char *error)
{
  return SUCCESS;
}

/*-- Frontend Loop -------------------------------------------------*/
INT frontend_loop()
{
  ss_sleep(10);
  return SUCCESS;
}

/********************************************************************\

  Readout routines for different events

\********************************************************************/

/*-- Interrupt configuration ---------------------------------------*/
 INT interrupt_configure(INT cmd, INT source, PTYPE adr)
{
  assert(!"interrupt_configure() is not implemented");
  return 0;
}

void Replace(std::string&x, const char* replace, const char* with)
{
  int i=x.find(replace);
  if (i>=0)
    x.replace(i, strlen(replace), with);
}

int write_data_float(HNDLE hDB, HNDLE hVar, const char* keyname, int num, const std::vector<float>& data)
{
   float val[num];
   for (int i=0; i<num; i++)
      val[i] = 0;

   int s = data.size();
   
   if (s != num)
      printf("write_data_float: array size mismatch: key \'%s\', num %d, data.size: %d\n", keyname, num, s);

   if (s > num)
      s = num;
      
   for (int i=0; i<s; i++)
      val[i] = data[i];

   int status = db_set_value(hDB, hVar, keyname, val, num*sizeof(float), num, TID_FLOAT);
   assert(status == DB_SUCCESS);
   
   return SUCCESS;
}

int write_data_int(HNDLE hDB, HNDLE hVar, const char* keyname, int num, const std::vector<int>& data)
{
   int val[num];
   for (int i=0; i<num; i++)
      val[i] = 0;

   int s = data.size();

   if (s != num)
      printf("write_data_int: array size mismatch: key \'%s\', num %d, data.size: %d\n", keyname, num, s);

   if (s > num)
      s = num;
      
   for (int i=0; i<s; i++)
      val[i] = data[i];

   int status = db_set_value(hDB, hVar, keyname, val, num*sizeof(int), num, TID_INT);
   assert(status == DB_SUCCESS);
   
   return SUCCESS;
}

int write_data_string(HNDLE hDB, HNDLE hVar, const char* keyname, int num, const std::vector<std::string>& data, int length)
{
   char val[length*num];
   memset(val, 0, length*num);

   int s = data.size();

   if (s != num)
      printf("write_data_string: array size mismatch: key \'%s\', num %d, data.size: %d\n", keyname, num, s);

   if (s > num)
      s = num;
      
   for (int i=0; i<s; i++)
      strlcpy(val+length*i, data[i].c_str(), length);

   int status = db_set_value(hDB, hVar, keyname, val, num*length, num, TID_STRING);
   assert(status == DB_SUCCESS);
   
   return SUCCESS;
}

bool gEnableControl = false;
bool gIgnoreOidNotIncreasing = false;

int set_snmp_float(const char* name, int index, float value)
{
   if (!gEnableControl)
      return SUCCESS;

   char str[1024];
   char s[1024];

   if (index<0)
      s[0] = 0;
   else
      sprintf(s, ".u%d", index);
   
   sprintf(str, "snmpset -v 2c -M +%s -m +WIENER-CRATE-MIB -c guru %s %s%s F %f", WIENER_MIB_DIR, gWienerDev.c_str(), name, s, value);
   
   printf("Set wiener float: %s\n", str);
   
   system(str);

   return SUCCESS;
}

int set_snmp_int(const char* name, int index, int value)
{
   if (!gEnableControl)
      return SUCCESS;

   char str[1024];
   char s[1024];

   if (index<0)
      s[0] = 0;
   else
      sprintf(s, ".u%d", index);

   sprintf(str, "snmpset -v 2c -M +%s -m +WIENER-CRATE-MIB -c guru %s %s%s i %d", WIENER_MIB_DIR, gWienerDev.c_str(), name, s, value);

   printf("Set wiener integer: %s\n", str);

   system(str);

   return SUCCESS;
}

extern HNDLE hDB;
static HNDLE hVar = 0;
static HNDLE hSet = 0;
static HNDLE hRdb = 0;
static HNDLE hStatus = 0;
static HNDLE hNames = 0;
static HNDLE hTemperatureNames = 0;
static HNDLE hFanSpeedNames = 0;
static HNDLE hFanAirTemperatureNames = 0;

const char* kGray   = "#A0A0A0";
const char* kRed    = "#FFA0A0";
const char* kGreen  = "#A0FFA0";
const char* kBlue   = "#A0A0FF";
const char* kYellow = "#FFFFA0";
const char* kWhite  = "#FFFFFF";

class Status
{
public:
   std::string fName;
   int fNumValues;

   int*  fStatus;
   char* fColors;

   Status(const char* name, int numValues)
   {
      fName      = name;
      fNumValues = numValues;
      fStatus    = new int[fNumValues];
      fColors    = new char[NAME_LENGTH*fNumValues];

      Reset();
   }

   ~Status() // dtor
   {
      delete fColors;
      fColors = NULL;
      delete fStatus;
      fStatus = NULL;
      fNumValues = 0;
   }

   void Reset()
   {
      for (int i=0; i<fNumValues; i++) {
         fStatus[i] = -1;
         strlcpy(fColors+i*NAME_LENGTH, kGray, NAME_LENGTH);
      }
   }

   void Resize(int newsize)
   {
      if (newsize <= fNumValues)
         return;

      delete fColors;
      delete fStatus;

      fNumValues = newsize;
      fStatus    = new int[fNumValues];
      fColors    = new char[NAME_LENGTH*fNumValues];

      Reset();
   }

   void Add(int index, int status, const char* colour)
   {
      assert(index>=0);
      assert(index<fNumValues);

      if (status > fStatus[index]) {
         fStatus[index] = status;
         strlcpy(fColors+index*NAME_LENGTH, colour, NAME_LENGTH);
      }
   }

   void Add(int index, const Status& s, int sindex)
   {
      assert(index>=0);
      assert(index<fNumValues);

      assert(sindex>=0);
      assert(sindex<s.fNumValues);

      if (s.fStatus[sindex] > fStatus[index]) {
         fStatus[index] = s.fStatus[sindex];
         strlcpy(fColors+index*NAME_LENGTH, s.fColors+sindex*NAME_LENGTH, NAME_LENGTH);
      }
   }

   void Write()
   {
      //printf("writing \'%s\' value %d, size %d\n", (fName).c_str(), fStatus[0], fNumValues);

      int status = db_set_value(hDB, hStatus, (fName).c_str(), fStatus, fNumValues*sizeof(int), fNumValues, TID_INT);
      assert(status == DB_SUCCESS);

      status = db_set_value(hDB, hStatus, (fName + "Color").c_str(), fColors, fNumValues*NAME_LENGTH, fNumValues, TID_STRING);
      assert(status == DB_SUCCESS);
   }
};

static int    gMainStatusDelay = 0;
static Status gDelayedMainStatus("delayedMainStatus", 1);
static Status gMainStatus("mainStatus", 1);
static Status gMainSwitchStatus("mainSwitch", 1);
static Status gOutputStatus("output", 1);
static Status gSwitchStatus("switch", 1);
static Status gStatusStatus("status", 1);
static Status gDemandStatus("demand", 1);
static Status gMeasuredStatus("measured", 1);
static Status gSensorTemperatureStatus("sensorTemperature", 1);
static Status gFanAirTemperatureStatus("fanAirTemperature", 1);

static int gCommError = 0;

struct Settings
{
public:
   int                mainSwitch;
   int                numOutputs;
   int                outputEnable;
   std::vector<int>   outputSwitch;
   std::vector<float> maxVoltage;
   std::vector<float> resistance;
   std::vector<float> odbDemand;
   std::vector<float> demand;
   float              rampRate;

   std::vector<int>   maxSensorTemperature;
   std::vector<int>   maxFanAirTemperature;
   bool               overTemperatureTurnOff;
   std::string        overTemperatureScript;

   int enableKill;

   int enableSparkMode;
   int sparkShutdownTime;

   Settings()
   {
      mainSwitch = -1;
      numOutputs = 0;
      outputEnable = 0;
      enableKill = -1;
      rampRate = 0;
      enableSparkMode = 0;
      sparkShutdownTime = 5;
      overTemperatureTurnOff = false;
   }
};

struct Readback
{
public:
   int numOutputs;
   int sysMainSwitch;

   std::vector<std::string> names;
   std::vector<int>   indices;
   std::vector<int>   switches;
   std::vector<int>   status;
   std::vector<std::string> statusString;
   std::vector<float> demandV;
   std::vector<float> demandI;
   std::vector<float> senseV;
   std::vector<float> currents;
   std::vector<float> rampUpRates;
   std::vector<float> rampDownRates;
   std::vector<std::string> groupsSwitchNames;

   std::vector<time_t> sparkTime;
   std::vector<int>    sparkCount;

   std::vector<int>    sensorTemperature;
   std::vector<int>    fanSpeed;
   std::vector<int>    fanAirTemperature;

   Readback()
   {
      numOutputs = 0;
      sysMainSwitch = -1;
   }

   void Clear()
   {
      names.clear();
      indices.clear();
      switches.clear();
      status.clear();
      statusString.clear();
      demandV.clear();
      demandI.clear();
      senseV.clear();
      currents.clear();
      rampUpRates.clear();
      rampDownRates.clear();
      groupsSwitchNames.clear();
      sensorTemperature.clear();
      fanSpeed.clear();
      fanAirTemperature.clear();
   }
};

Settings set;
Readback rdb;

static int xMainSwitch = -1;
static bool gWeSetMainSwitch = false;

void set_main_switch(int value);

static time_t gNextRead = 0;
static int    gNextUpdate = 0;
static int    gFastRead = 0;

static void handle_hotlink(INT a, INT b, void *c)
{
   printf("handle_hotlink!\n");
   gNextRead   = time(NULL) + 1;
   gNextUpdate = 1;
}

static void check_temperatures()
{
   //printf("check_temperatures!\n");

   bool alarm = false;

   gSensorTemperatureStatus.Reset();
   gFanAirTemperatureStatus.Reset();

   for (unsigned i=0; i<set.maxSensorTemperature.size(); i++)
      if (set.maxSensorTemperature[i] > 0) {
	 if (rdb.sensorTemperature[i] > set.maxSensorTemperature[i]) {
	    alarm = true;
	    gSensorTemperatureStatus.Add(i, 30, kRed);
	    gMainStatus.Add(0, 30, kRed);
	    cm_msg(MERROR, frontend_name, "Over temperature condition: sensor %d temperature: %d, limit: %d", i, rdb.sensorTemperature[i], set.maxSensorTemperature[i]);
	 }
      }

   for (unsigned i=0; i<set.maxFanAirTemperature.size(); i++)
      if (set.maxFanAirTemperature[i] > 0) {
	 if (rdb.fanAirTemperature[i] > set.maxFanAirTemperature[i]) {
	    alarm = true;
	    gFanAirTemperatureStatus.Add(i, 30, kRed);
	    gMainStatus.Add(0, 30, kRed);
	    cm_msg(MERROR, frontend_name, "Over temperature condition: fan %d air temperature: %d, limit: %d", i, rdb.fanAirTemperature[i], set.maxFanAirTemperature[i]);
	 }
      }

   gSensorTemperatureStatus.Write();
   gFanAirTemperatureStatus.Write();

   static bool gActive = false;

   if (alarm) {
      if (rdb.sysMainSwitch > 0 && set.overTemperatureTurnOff) {
         cm_msg(MERROR, frontend_name, "Over temperature condition: Turning off the main switch");
         set_main_switch(0);
      }

      if (!gActive) {
	 gActive = true;

         if (set.overTemperatureScript.length() > 0) {
            cm_msg(MERROR, frontend_name, "Over temperature condition: Running the OverTemperature script: %s", set.overTemperatureScript.c_str());
            ss_system(set.overTemperatureScript.c_str());
         }
      }
   } else {
      gActive = false;
   }
}

#include <iostream>
//#include <stdio>
static void update_settings()
{
  char str[1024];

  cm_msg(MDEBUG, frontend_name, "Updating settings.");

  gMainStatus.Reset();
  gMainStatus.Add(0, 1000, kBlue);
  gMainStatus.Write();

  sprintf(str, "/Equipment/%s/Settings/ReadPeriod", eq_name);

  gReadPeriod = odbReadInt(str, 0, 10);

  sprintf(str, "/Equipment/%s/Settings/IgnoreOidNotIncreasing", eq_name);

  gIgnoreOidNotIncreasing = odbReadInt(str, 0, 0);

  //if (gIgnoreOidNotIncreasing)
  //cm_msg(MINFO, frontend_name, "Ignoring the \'OID not increasing\' error");

  sprintf(str, "/Equipment/%s/Settings/EnableControl", eq_name);

  gEnableControl = odbReadInt(str, 0, 0);

  sprintf(str, "/Equipment/%s/Settings/EnableSparkMode", eq_name);

  set.enableSparkMode = odbReadInt(str, 0, 0);

  sprintf(str, "/Equipment/%s/Settings/SparkShutdownTime", eq_name);

  set.sparkShutdownTime = odbReadInt(str, 0, 5);

  // turn the mainframe on or off

  sprintf(str, "/Equipment/%s/Settings/mainSwitch", eq_name);

  set.mainSwitch = odbReadInt(str, 0, 0);

  gMainSwitchStatus.Add(0, 1000, kBlue);
  gMainSwitchStatus.Write();

#if 0
  set_snmp_int("sysMainSwitch.0", -1, set.mainSwitch);

  if (set.mainSwitch != rdb.sysMainSwitch) {
     printf("mainswitch %d %d\n", set.mainSwitch, rdb.sysMainSwitch);
     ss_sleep(30000);
  }
#endif

  if (xMainSwitch < 0)
     xMainSwitch = set.mainSwitch;

  printf("Set main switch %i %i\n",set.mainSwitch,xMainSwitch);
  if (set.mainSwitch != xMainSwitch) {
     xMainSwitch = set.mainSwitch;
     printf("Set 2 main switch %i %i\n",rdb.sysMainSwitch,set.mainSwitch);
     if (rdb.sysMainSwitch != set.mainSwitch) {
	printf("Setting main swith \n");
        gWeSetMainSwitch = true;
        set_snmp_int("sysMainSwitch.0", -1, set.mainSwitch);
        gNextRead = time(NULL);
        gFastRead = 10;
        return;
     }
  }

  sprintf(str, "/Equipment/%s/Settings/outputEnable", eq_name);

  set.outputEnable = odbReadInt(str, 0, 1);

  sprintf(str, "/Equipment/%s/Settings/numOutputs", eq_name);

  set.numOutputs = odbReadInt(str, 0, 0);

  if (rdb.numOutputs > set.numOutputs) {
     cm_msg(MINFO, frontend_name, "Number of output channels changed from %d to %d", set.numOutputs, rdb.numOutputs);
     set.numOutputs = rdb.numOutputs;
     odbWriteInt(str, 0, set.numOutputs);
  }

  gOutputStatus.Resize(set.numOutputs);
  gSwitchStatus.Resize(set.numOutputs);
  gStatusStatus.Resize(set.numOutputs);
  gDemandStatus.Resize(set.numOutputs);
  gMeasuredStatus.Resize(set.numOutputs);

  int num = rdb.numOutputs;

  // set the ramping rates

  sprintf(str, "/Equipment/%s/Settings/rampRate", eq_name);

  set.rampRate = odbReadFloat(str, 0, 0);

  if (set.rampRate > 0) {
     cm_msg(MINFO, frontend_name, "Ramp rate %i", rdb.rampUpRates.empty());
     std::cout << "Ramp rate " << rdb.rampUpRates.empty() << std::endl;
     //assert(!rdb.rampUpRates.empty());
     //assert(!rdb.rampDownRates.empty());

     for (int i=0; i<num; i++)
        {
           if (fabs(set.rampRate - rdb.rampUpRates[i]) > 0.1)
              set_snmp_float("outputVoltageRiseRate", rdb.indices[i], set.rampRate);
           
        if (fabs(set.rampRate - rdb.rampDownRates[i]) > 0.1)
           set_snmp_float("outputVoltageFallRate", rdb.indices[i], set.rampRate);
        }
  }
     
  // set enableKill

  sprintf(str, "/Equipment/%s/Settings/enableKill", eq_name);

  set.enableKill = odbReadInt(str, 0, 1);

  if (set.enableKill >= 0)
     {
        int value = 0;
        if (set.enableKill == 0)
           value = 4;
        else if (set.enableKill == 1)
           value = 5;

        if (value)
           {
              if (rdb.groupsSwitchNames.size() > 0)
                 {
                    for (unsigned i=0; i<rdb.groupsSwitchNames.size(); i++)
                       set_snmp_int(rdb.groupsSwitchNames[i].c_str(), -1, value);
                 }
           }
     }

  // set output voltage limits

  sprintf(str, "/Equipment/%s/Settings/maxVoltage", eq_name);

  odbResizeArray(str, TID_FLOAT, set.numOutputs);

  set.maxVoltage.resize(set.numOutputs);

  for (int i=0; i<num; i++)
     {
	set.maxVoltage[i] = odbReadFloat(str, i);
     }

  // set resistance

  sprintf(str, "/Equipment/%s/Settings/resistance", eq_name);

  odbResizeArray(str, TID_FLOAT, set.numOutputs);

  set.resistance.resize(set.numOutputs);

  for (int i=0; i<num; i++)
     {
	set.resistance[i] = odbReadFloat(str, i);
     }

  // set output current limits

  sprintf(str, "/Equipment/%s/Settings/outputCurrent", eq_name);

  odbResizeArray(str, TID_FLOAT, set.numOutputs);

  for (int i=0; i<num; i++)
     {
        float outputCurrent = odbReadFloat(str, i);

        if (outputCurrent != rdb.demandI[i])
           {
              set_snmp_float("outputCurrent", rdb.indices[i], outputCurrent);
           }
     }

  sprintf(str, "/Equipment/%s/Settings/outputVoltage", eq_name);

  odbResizeArray(str, TID_FLOAT, set.numOutputs);

  set.odbDemand.resize(set.numOutputs);
  set.demand.resize(set.numOutputs);

  for (int i=0; i<num; i++)
     {
        set.odbDemand[i] = odbReadFloat(str, i);
        set.demand[i] = set.odbDemand[i];

	if (set.resistance[i])
	   {
	      double current = rdb.currents[i];
	      if (current < 0)
		 current = 0;
	      double vcorr = set.resistance[i] * current;
	      printf("chan %d, voltage %f V, current %f A, resistance %f Ohm, correction %f V\n", i, set.demand[i], current, set.resistance[i], vcorr);
	      set.demand[i] += vcorr;
	   }

	if (set.maxVoltage[i])
	   if (set.demand[i] > set.maxVoltage[i])
	      set.demand[i] = set.maxVoltage[i];

	if (rdb.sysMainSwitch > 0) {
	   if (fabs(set.demand[i] - rdb.demandV[i]) > 0.001)
	      {
		 printf("mainswitch %d %d, num chan %d %d\n", set.mainSwitch, rdb.sysMainSwitch, set.numOutputs, rdb.numOutputs);
		 set_snmp_float("outputVoltage", rdb.indices[i], set.demand[i]);
		 gDemandStatus.Add(i, 1000, kBlue);
	      }
	}
     }

  gDemandStatus.Write();

  // turn channels on and off

  sprintf(str, "/Equipment/%s/Settings/outputSwitch", eq_name);

  odbResizeArray(str, TID_INT, set.numOutputs);

  set.outputSwitch.resize(set.numOutputs);

  for (int i=0; i<num; i++)
     {
        set.outputSwitch[i] = odbReadInt(str, i);

	if (!set.outputEnable)
	   set.outputSwitch[i] = 0;

        if (set.outputSwitch[i] != rdb.switches[i])
           {
              if (set.outputSwitch[i]==1) {

                 // before channel turn-on, clear error status
                 set_snmp_int("outputSwitch", rdb.indices[i], 10);

                 if (set.enableSparkMode) {
                    // in spark counting mode, set supervisor behaviour to 0 (no trip)
                    set_snmp_int("outputSupervisionBehavior", rdb.indices[i], 0);

                    // in spark counting mode, set trip time to 1
                    set_snmp_int("outputTripTimeMaxCurrent", rdb.indices[i], 1);

                    // clear spark count?
                    rdb.sparkCount[i] = 0;
                    rdb.sparkTime[i]  = 0;
                 }
              }

              set_snmp_int("outputSwitch", rdb.indices[i], set.outputSwitch[i]);

              gSwitchStatus.Add(i, 1000, kBlue);
           }
     }

  gSwitchStatus.Write();

  if (1) { // temperature limits

     sprintf(str, "/Equipment/%s/Variables/sensorTemperature", eq_name);

     int s = odbReadArraySize(str);

     if (s > 0) {
	sprintf(str, "/Equipment/%s/Settings/maxSensorTemperature", eq_name);
     
	odbResizeArray(str, TID_INT, s);
	
	set.maxSensorTemperature.resize(s);

	gSensorTemperatureStatus.Resize(s);
	gSensorTemperatureStatus.Write();

	for (int i=0; i<s; i++) {
	   set.maxSensorTemperature[i] = odbReadInt(str, i);
	}
     }
  }

  if (1) { // fan air temperature limits

     sprintf(str, "/Equipment/%s/Variables/fanAirTemperature", eq_name);

     int s = odbReadArraySize(str);

     if (s > 0) {
	sprintf(str, "/Equipment/%s/Settings/maxFanAirTemperature", eq_name);
     
	odbResizeArray(str, TID_INT, s);
	
	set.maxFanAirTemperature.resize(s);

	gFanAirTemperatureStatus.Resize(s);
	gFanAirTemperatureStatus.Write();
	
	for (int i=0; i<s; i++) {
	   set.maxFanAirTemperature[i] = odbReadInt(str, i);
	}
     }
  }

  if (1) {
     sprintf(str, "/Equipment/%s/Settings/OverTemperatureTurnOff", eq_name);
     set.overTemperatureTurnOff = odbReadBool(str, 0, false);
  }

  if (1) {
     sprintf(str, "/Equipment/%s/Settings/OverTemperatureScript", eq_name);
     set.overTemperatureScript = odbReadString(str, 0, "", 250);
  }

  check_temperatures();

  gNextRead = time(NULL);
  gFastRead = 10;
}

void set_main_switch(int value)
{
   gWeSetMainSwitch = true;
   set_snmp_int("sysMainSwitch.0", -1, value);
   int size = sizeof(value);
   db_set_value(hDB, hSet, "mainSwitch", &value, size, 1, TID_INT);
}


static void open_hotlink(HNDLE hDB, HNDLE hSet)
{
  static bool once = false;
  if (once)
    return;
  once = true;

  update_settings();

  int status = db_open_record(hDB, hSet, NULL, 0, MODE_READ, handle_hotlink, NULL);
  assert(status == DB_SUCCESS);
}

INT rpc_callback(INT index, void *prpc_param[])
{
   const char* cmd  = CSTRING(0);
   const char* args = CSTRING(1);
   char* return_buf = CSTRING(2);
   int   return_max_length = CINT(3);

   return_buf[0] = 0;

   cm_msg(MINFO, "rpc_callback", "--------> rpc_callback: index %d, max_length %d, cmd [%s], args [%s]", index, return_max_length, cmd, args);

#if 0
   int example_int = strtol(args, NULL, 0);
   int size = sizeof(int);
   int status = db_set_value(hDB, 0, "/Equipment/" EQ_NAME "/Settings/example_int", &example_int, size, 1, TID_INT);

   char tmp[256];
   time_t now = time(NULL);
   sprintf(tmp, "current time is %d %s", (int)now, ctime(&now));

   strlcpy(return_buf, tmp, return_max_length);
#endif

   if (strcmp(cmd, "TurnOff") == 0) {
      set_main_switch(0);
   } else if (strcmp(cmd, "TurnOn") == 0) {
      set_main_switch(1);
   }

   return RPC_SUCCESS;
}

/*-- Frontend Init -------------------------------------------------*/

INT frontend_init()
{
   setbuf(stdout,NULL);
   setbuf(stderr,NULL);

   // set transition priorities to "async"

   cm_set_transition_sequence(TR_START, 0);
   cm_set_transition_sequence(TR_STOP, 0);
   cm_set_transition_sequence(TR_PAUSE, 0);
   cm_set_transition_sequence(TR_RESUME, 0);

#ifdef RPC_JRPC
   // register AJAX RPC handler

   cm_register_function(RPC_JRPC, rpc_callback);
#endif

   // unregister transition handlers

   cm_deregister_transition(TR_START);
   cm_deregister_transition(TR_STOP);
   cm_deregister_transition(TR_PAUSE);
   cm_deregister_transition(TR_RESUME);

   sprintf(eq_name, "%s", EQ_NAME);

   char str[1024];
   sprintf(str, "/Equipment/%s/Settings/Hostname", eq_name);

   gWienerDev = odbReadString(str, 0, "", 200);

   if (gWienerDev.length() < 2)
      {
         cm_msg(MERROR, frontend_name, "frontend_init(): Hostname is blank, please set \'%s\'", str);
         return !SUCCESS;
      }

   sprintf(str, "/Equipment/%s/Settings/SnmpwalkCommand", eq_name);

   gSnmpwalkCommand = odbReadString(str, 0, "", 200);

   sprintf(str, "/Equipment/%s/Settings/DelayMainStatus", eq_name);

   gMainStatusDelay = odbReadInt(str, 0, 60);

   gDelayedMainStatus.Reset();
   gDelayedMainStatus.Add(0, 1000, kBlue);

   set_equipment_status(eq_name, "Init Ok", "#00FF00");

   return SUCCESS;
}

void set_eq_status()
{
   if (gDelayedMainStatus.fStatus[0] == 10)
      set_equipment_status(eq_name, "Off", "#FFFFFF");
   else if (gDelayedMainStatus.fStatus[0] == 20 && rdb.sysMainSwitch == 0)
      set_equipment_status(eq_name, "Off", "#FFFFFF");
   else if (gDelayedMainStatus.fStatus[0] == 20)
      set_equipment_status(eq_name, "Ok", "#00FF00");
   else
      set_equipment_status(eq_name, "Check status", "yellow");
}

int read_wiener_event(char *pevent, INT off)
{
  time_t now;
  time(&now);

  if (hVar==0)
    {
      char str[1024];
      sprintf(str, "/Equipment/%s/Variables", eq_name);

      int status = db_find_key(hDB, 0, str, &hVar);
      if (status == DB_NO_KEY)
	{
	  status = db_create_key(hDB, 0, str, TID_KEY);
	  if (status == DB_SUCCESS)
	    status = db_find_key(hDB, 0, str, &hVar);
	}

      if (status != SUCCESS)
	{
	  cm_msg(MERROR, frontend_name, "read_wiener_event: Cannot find or create %s, status %d, exiting", str, status);
	  exit(1);
	}
    }

  if (hSet==0)
    {
      char str[1024];
      sprintf(str, "/Equipment/%s/Settings", eq_name);

      int status = db_find_key(hDB, 0, str, &hSet);
      if (status == DB_NO_KEY)
	{
	  status = db_create_key(hDB, 0, str, TID_KEY);
	  if (status == DB_SUCCESS)
	    status = db_find_key(hDB, 0, str, &hSet);
	}

      if (status != SUCCESS)
	{
	  cm_msg(MERROR, frontend_name, "read_wiener_event: Cannot find or create %s, status %d, exiting", str, status);
	  exit(1);
	}
    }

  if (hRdb==0)
    {
      char str[1024];
      sprintf(str, "/Equipment/%s/Readback", eq_name);

      int status = db_find_key(hDB, 0, str, &hRdb);
      if (status == DB_NO_KEY)
	{
	  status = db_create_key(hDB, 0, str, TID_KEY);
	  if (status == DB_SUCCESS)
	    status = db_find_key(hDB, 0, str, &hRdb);
	}

      if (status != SUCCESS)
	{
	  cm_msg(MERROR, frontend_name, "read_wiener_event: Cannot find or create %s, status %d, exiting", str, status);
	  exit(1);
	}
    }

  if (hStatus==0)
    {
      char str[1024];
      sprintf(str, "/Equipment/%s/Status", eq_name);

      int status = db_find_key(hDB, 0, str, &hStatus);
      if (status == DB_NO_KEY)
	{
	  status = db_create_key(hDB, 0, str, TID_KEY);
	  if (status == DB_SUCCESS)
	    status = db_find_key(hDB, 0, str, &hStatus);
	}

      if (status != SUCCESS)
	{
	  cm_msg(MERROR, frontend_name, "read_wiener_event: Cannot find or create %s, status %d, exiting", str, status);
	  exit(1);
	}
    }

  if (now < gNextRead)
     return 0;

  if (gFastRead > 0)
     {
	gFastRead--;
	gNextRead = now + 1;
     }
  else
     gNextRead += gReadPeriod; // ODB settings read period

  if (gNextRead < now)
     gNextRead = now + 10; // ODB settings read period

  if (gNextUpdate) {
     gNextUpdate = 0;
     update_settings();
     return 0;
  }

  //
  // Note: please copy WIENER-CRATE-MIB.txt to /usr/share/snmp/mibs/
  //
  // Control commands:
  //
  // snmpset -v 2c -m +WIENER-CRATE-MIB -c guru daqtmp1 sysMainSwitch.0 i 1
  // snmpset -v 2c -m +WIENER-CRATE-MIB -c guru daqtmp1 outputSwitch.u100 i 1
  // snmpset -v 2c -m +WIENER-CRATE-MIB -c guru daqtmp1 outputVoltage.u100 F 10
  //

  bool read_ok = false;

  char str[1024];

  if (gSnmpwalkCommand.length() > 2)
     sprintf(str, "%s 2>&1", gSnmpwalkCommand.c_str());
  else
     sprintf(str, "snmpwalk -v 2c -M +%s -m +WIENER-CRATE-MIB -c guru %s crate 2>&1", WIENER_MIB_DIR, gWienerDev.c_str());

  //  printf("Read wiener event: %s\n", str);

  rdb.Clear();

  FILE *fp = popen(str, "r");
  if (fp == NULL)
    {
      ss_sleep(200);
      return 0;
    }

  while (1)
    {
      char *s = fgets(str, sizeof(str), fp);
      if (!s)
	break;

      //printf("wiener: %s\n", s);

      char name[1024];

      s = strstr(str, "No Response from");
      if (s)
	{
	  printf("No response from : %s", str);
	  if (gCommError == 0) {
	     cm_msg(MERROR, frontend_name, "read_wiener_event: No response from \'%s\': %s", gWienerDev.c_str(), str);
	     gCommError = 1;
	  }
	  continue;
	}

      s = strstr(str, "Error: OID not increasing");
      if (s)
	{
	   if (gIgnoreOidNotIncreasing) {
	      pclose(fp);
	      return 0;
	   }
          gCommError = 1;
	  printf("Strange SNMP error : %s", str);
	  cm_msg(MERROR, frontend_name, "read_wiener_event: Strange SNMP error from \'%s\': %s", gWienerDev.c_str(), str);
          pclose(fp);
	  return 0;
	}

      s = strstr(str, "WIENER-CRATE-MIB::");
      if (s == NULL)
	{
          gCommError = 1;
	  printf("unknown response (no WIENER-CRATE-MIB::) : %s", str);
	  continue;
	}

      strcpy(name, s+18);

      char* q = strstr(name, " = ");
      if (q == NULL)
	{
	  printf("unknown response (no \'=\'): %s", str);
	  continue;
	}

      *q = 0;

      //printf("name [%s]\n", name);

      if (1)
	{
	  std::string x = name;
	  Replace(x, "outputMeasurement", "oMeas");
	  Replace(x, "outputSupervision", "oSup");
	  Replace(x, "outputConfig", "oConf");
	  Replace(x, "TerminalVoltage", "TermV");
	  Replace(x, "SenseVoltage", "SenseV");
	  Replace(x, "sensorTemperature", "sensorT");
	  Replace(x, "sensorWarningThreshold", "sensorWarnThr");
	  Replace(x, "sensorFailureThreshold", "sensorFailThr");
	  Replace(x, "psAuxiliary", "psAux");
	  Replace(x, "moduleAuxiliaryMeasurementVoltage", "moAuxMeasV");
	  Replace(x, "moduleAuxiliaryMeasurementTemperature", "moAuxMeasT");
	  strcpy(name, x.c_str());
	}

      if (strlen(name) >= NAME_LENGTH-1)
	{
	  std::string x = name;
	  int i;

	  i=x.find("Threshold");
	  if (i>=0)
	    x.replace(i, 9, "Thr");

	  i=x.find("outputConfig");
	  if (i>=0)
	    x.replace(i, strlen("outputConfig"), "oConf");

	  i=x.find("outputSupervision");
	  if (i>=0)
	    x.replace(i, strlen("outputSupervision"), "oSup");

	  i=x.find("outputMeasurement");
	  if (i>=0)
	    x.replace(i, strlen("outputMeasurement"), "oMeas");

	  i=x.find("Terminal");
	  if (i>=0)
	    x.replace(i, strlen("Terminal"), "Term");

	  //printf("transform %s -> %s\n", name, x.c_str());

	  strcpy(name, x.c_str());
	}

      if (strlen(name) >= NAME_LENGTH-1)
	{
           cm_msg(MERROR, frontend_name, "read_wiener_event: Variable name \'%s\' is too long %d, limit %d", name, (int)strlen(name), NAME_LENGTH-1);
	  exit(1);
	}

      if ((s = strstr(str, "No more variables")) != NULL)
	{
	  continue;
	}
      else if ((s = strstr(str, "INTEGER:")) != NULL)
	{
	  s += 8;
	  while (*s != 0)
	    {
	      if (isdigit(*s))
		break;
	      if (*s == '-')
		break;
	      if (*s == '+')
		break;
	      s++;
	    }

	  int val = atoi(s);
	  //printf("%s = int value %d from %s", name, val, str);
	  db_set_value(hDB, hRdb, name, &val, sizeof(val), 1, TID_INT);

	  if (strstr(name, "Meas"))
	    {
	      if (strstr(name, "Temp"))
		{
#if 0
		  char *ss = strstr(name, ".u");
		  assert(ss);
		  int chan = atoi(ss+2);
		  assert(chan>=0 && chan<MAX_CHAN);
		  //printf("chan %d temperature %d\n", chan, val);
		  temperature[chan] = val;
#endif
		}
	    }
          else if (strstr(name, "sysMainSwitch"))
	    {
               char *ss = strstr(name, ".0");
               assert(ss);
               int chan = atoi(ss+2);
               rdb.sysMainSwitch = val;
	       read_ok = true;
	       gCommError = 0;
	    }
          else if (strstr(name, "outputSwitch"))
	    {
               char *ss = strstr(name, ".u");
               assert(ss);
               int chan = atoi(ss+2);
               rdb.switches.push_back(val);
	    }
          else if (strstr(name, "groupsSwitch"))
	    {
               //printf("group name [%s]\n", name);
               rdb.groupsSwitchNames.push_back(name);
	    }
          else if (strstr(name, "outputNumber"))
	    {
               // nothing
	    }
	  else if (strstr(name, "sensorT"))
	    {
	       char *ss = strstr(name, ".temp");
	       assert(ss);
	       int chan = atoi(ss+5);
	       rdb.sensorTemperature.push_back(val);
	       //printf("name [%s] chan %d, val %d\n", name, chan, val);
	    }
	  else if (strstr(name, "fanSpeed"))
	    {
	       char *ss = strstr(name, ".");
	       assert(ss);
	       int chan = atoi(ss+1);
	       rdb.fanSpeed.push_back(val);
	       //printf("name [%s] chan %d, val %d\n", name, chan, val);
	    }
	  else if (strstr(name, "fanAirTemperature"))
	    {
	       char *ss = strstr(name, ".");
	       assert(ss);
	       int chan = atoi(ss);
	       rdb.fanAirTemperature.push_back(val);
	       //printf("name [%s] chan %d, val %d\n", name, chan, val);
	    }
	}
      else if ((s = strstr(str, "Float:")) != NULL)
	{
	  float val = atof(s + 6);
	  //printf("%s = float value %f\n", name, val);
	  db_set_value(hDB, hRdb, name, &val, sizeof(val), 1, TID_FLOAT);

	  if (strstr(name, "oMeasCurrent"))
	     {
		char *ss = strstr(name, ".u");
		assert(ss);
		int chan = atoi(ss+2);
		//printf("chan %d current %f\n", chan, val);
		rdb.currents.push_back(val);
	     }
	  else if (strstr(name, "psAux"))
	    {
	       // ignore
	    }
	  else if (strstr(name, "Meas"))
	    {
	      if (strstr(name, "SenseV"))
		{
		  char *ss = strstr(name, ".u");
		  assert(ss);
		  int chan = atoi(ss+2);
                  rdb.senseV.push_back(val);
		}
	      else if (strstr(name, "Current"))
		{
		  char *ss = strstr(name, ".u");
		  assert(ss);
		  int chan = atoi(ss+2);
		  //printf("chan %d current %f\n", chan, val);
                  rdb.currents.push_back(val);
		}
	    }
          else if (strstr(name, "outputVoltage.u"))
             {
                char *ss = strstr(name, ".u");
                assert(ss);
                int chan = atoi(ss+2);
                //printf("chan %d current %f\n", chan, val);
                rdb.demandV.push_back(val);
             }
          else if (strstr(name, "outputVoltageRiseRate.u"))
             {
                char *ss = strstr(name, ".u");
                assert(ss);
                int chan = atoi(ss+2);
                //printf("chan %d current %f\n", chan, val);
                rdb.rampUpRates.push_back(val);
             }
          else if (strstr(name, "outputVoltageFallRate.u"))
             {
                char *ss = strstr(name, ".u");
                assert(ss);
                int chan = atoi(ss+2);
                //printf("chan %d current %f\n", chan, val);
                rdb.rampDownRates.push_back(val);
             }
          else if (strstr(name, "outputCurrent.u"))
             {
                char *ss = strstr(name, ".u");
                assert(ss);
                int chan = atoi(ss+2);
                //printf("chan %d current %f\n", chan, val);
                rdb.demandI.push_back(val);
                rdb.indices.push_back(chan);
             }
	}
      else if ((s = strstr(str, "BITS:")) != NULL)
	{
	  uint32_t val = 0;
	  char* ss = s+5;

          //printf("bits %s\n", ss);

          int ishift = 0;

	  while (*ss)
	    {
	      while (isspace(*ss))
		ss++;

              int xval = 0;

              if (isdigit(*ss)) {
                 xval = (*ss - '0');
              } else if ((toupper(*ss) >= 'A') && (toupper(*ss) <= 'F')) {
                 xval = 10 + (toupper(*ss) - 'A');
              } else {
                 break;
              }

              // bits go in reverse order

              int ival = 0;

              if (xval&1)
                 ival |= 8;

              if (xval&2)
                 ival |= 4;

              if (xval&4)
                 ival |= 2;

              if (xval&8)
                 ival |= 1;

	      val |= (ival<<ishift);

              ishift += 4;

	      ss++;
	    }

          if (1)
             {
                char *xss;
                xss = strchr(ss, '\n');
                if (xss)
                   *xss = 0;
                xss = strchr(ss, '\r');
                if (xss)
                   *xss = 0;
             }

          char* text = ss;

	  //printf("%s = bit value 0x%08x from [%s], text [%s]\n", name, val, str, text);

	  db_set_value(hDB, hRdb, name, &val, sizeof(val), 1, TID_DWORD);

	  if (strstr(name, "outputStatus"))
	    {
               char *ss = strstr(name, ".u");
               assert(ss);
               int chan = atoi(ss+2);
               rdb.status.push_back(val);
               rdb.statusString.push_back(text);
	    }
	}
      else if ((s = strstr(str, "STRING:")) != NULL)
	{
	  char *ss =  (s + 8);
	  while (isspace(*ss))
	    ss++;
	  if (ss[strlen(ss)-1]=='\n')
	    ss[strlen(ss)-1]=0;

	  //printf("%s = string value [%s]\n", name, ss);
	  db_set_value(hDB, hRdb, name, ss, strlen(ss)+1, 1, TID_STRING);

	  if (strstr(name,"outputName"))
             {
                rdb.names.push_back(ss);
             }
	}
      else if ((s = strstr(str, "IpAddress:")) != NULL)
	{
	  char *ss =  (s + 10);
	  while (isspace(*ss))
	    ss++;
	  if (ss[strlen(ss)-1]=='\n')
	    ss[strlen(ss)-1]=0;

	  //printf("%s = IpAddress value [%s]\n", name, ss);
	  db_set_value(hDB, hRdb, name, ss, strlen(ss)+1, 1, TID_STRING);
	}
      else if ((s = strstr(str, " = \"\"")) != NULL)
	 {
	    db_set_value(hDB, hRdb, name, "", 1, 1, TID_STRING);
	 }
      else
	 {
	    printf("%s = unknown data type: %s", name, str);
	 }
    }

  pclose(fp);

  printf("read_ok %d, gCommError %d, main status %d, delayed main status %d\n", read_ok, gCommError, gMainStatus.fStatus[0], gDelayedMainStatus.fStatus[0]);

  if (read_ok && gCommError==0) {
     set_eq_status();
  } else
     set_equipment_status(eq_name, "Communication problem", "red");

  //printf("sizes: %d %d %d %d %d %d\n", names.size(), switches.size(), status.size(), demandV.size(), senseV.size(), currents.size());

  int num = rdb.names.size();

  if (num == 0)
     {
        if (rdb.numOutputs == 0) {
           open_hotlink(hDB, hSet);
           return 0;
        }

        num = rdb.numOutputs;

        for (int i=0; i<num; i++)
           {
              rdb.switches.push_back(0);
              rdb.status.push_back(0);
              rdb.statusString.push_back("");
              rdb.demandV.push_back(0);
              rdb.senseV.push_back(0);
              rdb.currents.push_back(0);
           }
     }

  rdb.numOutputs = num;
  //  printf("sizes: %d \n", num);

  if (hNames==0)
     {
        bool doWriteNames = false;
        char str[1024];
        sprintf(str, "/Equipment/%s/Settings/Names", eq_name);
        
        int status = db_find_key(hDB, 0, str, &hNames);
        if (status == DB_NO_KEY)
           {
              status = db_create_key(hDB, 0, str, TID_STRING);
              if (status == DB_SUCCESS)
                 status = db_find_key(hDB, 0, str, &hNames);
              doWriteNames = true;
           }
        
        if (status != SUCCESS)
           {
              cm_msg(MERROR, frontend_name, "read_wiener_event: Cannot find or create %s, status %d, exiting", str, status);
              exit(1);
           }

        if (doWriteNames)
           {
              for (int i=0; i<num; i++)
                 {
                    status = db_set_data_index(hDB, hNames, rdb.names[i].c_str(), NAME_LENGTH, i, TID_STRING);
                    assert(status == DB_SUCCESS);
                 }
           }

        if (1)
           {
              HNDLE hKey;
              sprintf(str, "/Equipment/%s/Status/HwNames", eq_name);
        
              int status = db_find_key(hDB, 0, str, &hKey);
              if (status == DB_NO_KEY)
                 {
                    status = db_create_key(hDB, 0, str, TID_STRING);
                    if (status == DB_SUCCESS)
                       status = db_find_key(hDB, 0, str, &hKey);
                 }

              for (int i=0; i<num; i++)
                 {
                    status = db_set_data_index(hDB, hKey, rdb.names[i].c_str(), NAME_LENGTH, i, TID_STRING);
                    assert(status == DB_SUCCESS);
                 }
           }
     }

  if (rdb.sensorTemperature.size()>0 && hTemperatureNames==0)
     {
        bool doWriteNames = false;
        char str[1024];
        sprintf(str, "/Equipment/%s/Settings/Names sensorTemperature", eq_name);
        
        int status = db_find_key(hDB, 0, str, &hTemperatureNames);
        if (status == DB_NO_KEY)
           {
              status = db_create_key(hDB, 0, str, TID_STRING);
              if (status == DB_SUCCESS)
                 status = db_find_key(hDB, 0, str, &hTemperatureNames);
              doWriteNames = true;
           }
        
        if (status != SUCCESS)
           {
              cm_msg(MERROR, frontend_name, "read_wiener_event: Cannot find or create %s, status %d, exiting", str, status);
              exit(1);
           }

        if (doWriteNames)
           {
              for (int i=0; i<rdb.sensorTemperature.size(); i++)
                 {
		    char xname[256];
		    sprintf(xname, "temp%d", i+1);
                    status = db_set_data_index(hDB, hTemperatureNames, xname, NAME_LENGTH, i, TID_STRING);
                    assert(status == DB_SUCCESS);
                 }
           }
     }

  if (rdb.fanSpeed.size()>0 && hFanSpeedNames==0)
     {
        bool doWriteNames = false;
        char str[1024];
        sprintf(str, "/Equipment/%s/Settings/Names fanSpeed", eq_name);
        
        int status = db_find_key(hDB, 0, str, &hFanSpeedNames);
        if (status == DB_NO_KEY)
           {
              status = db_create_key(hDB, 0, str, TID_STRING);
              if (status == DB_SUCCESS)
                 status = db_find_key(hDB, 0, str, &hFanSpeedNames);
              doWriteNames = true;
           }
        
        if (status != SUCCESS)
           {
              cm_msg(MERROR, frontend_name, "read_wiener_event: Cannot find or create %s, status %d, exiting", str, status);
              exit(1);
           }

        if (doWriteNames)
           {
              for (int i=0; i<rdb.fanSpeed.size(); i++)
                 {
		    char xname[256];
		    sprintf(xname, "Fan %d", i+1);
                    status = db_set_data_index(hDB, hFanSpeedNames, xname, NAME_LENGTH, i, TID_STRING);
                    assert(status == DB_SUCCESS);
                 }
           }
     }

  if (rdb.fanAirTemperature.size()>0 && hFanAirTemperatureNames==0)
     {
        bool doWriteNames = false;
        char str[1024];
        sprintf(str, "/Equipment/%s/Settings/Names fanAirTemperature", eq_name);
        
        int status = db_find_key(hDB, 0, str, &hFanAirTemperatureNames);
        if (status == DB_NO_KEY)
           {
              status = db_create_key(hDB, 0, str, TID_STRING);
              if (status == DB_SUCCESS)
                 status = db_find_key(hDB, 0, str, &hFanAirTemperatureNames);
              doWriteNames = true;
           }
        
        if (status != SUCCESS)
           {
              cm_msg(MERROR, frontend_name, "read_wiener_event: Cannot find or create %s, status %d, exiting", str, status);
              exit(1);
           }

        if (doWriteNames)
           {
              for (int i=0; i<rdb.fanAirTemperature.size(); i++)
                 {
		    char xname[256];
		    sprintf(xname, "FanAirTemp%d", i);
                    status = db_set_data_index(hDB, hFanAirTemperatureNames, xname, NAME_LENGTH, i, TID_STRING);
                    assert(status == DB_SUCCESS);
                 }
           }
     }

  if (1)
     {
        HNDLE hkey;
        char str[1024];
        sprintf(str, "/Equipment/%s/Settings/outputSwitch", eq_name);
        
        int status = db_find_key(hDB, 0, str, &hkey);

        if (status != DB_SUCCESS)
           write_data_int(hDB, hSet, "outputSwitch",  num, rdb.switches);
     }

  
  if (1)
     {
        HNDLE hkey;
        char str[1024];
        sprintf(str, "/Equipment/%s/Settings/outputVoltage", eq_name);
        
        int status = db_find_key(hDB, 0, str, &hkey);

        if (status != DB_SUCCESS)
           write_data_float(hDB, hSet, "outputVoltage",  num, rdb.demandV);
     }

  if (1)
     {
        HNDLE hkey;
        char str[1024];
        sprintf(str, "/Equipment/%s/Settings/outputCurrent", eq_name);
        
        int status = db_find_key(hDB, 0, str, &hkey);

        if (status != DB_SUCCESS)
           write_data_float(hDB, hSet, "outputCurrent",  num, rdb.demandI);
     }

#define STATUS_ON                           (1)
#define STATUS_OutputFailureMinSenseVoltage (0x0004)
#define STATUS_OutputFailureMaxCurrent (0x0020)
#define STATUS_OutputCurrentLimited    (0x0400)
#define STATUS_RAMPUP                  (0x0800)
#define STATUS_RAMPDOWN                (0x1000)
#define STATUS_ENABLEKILL              (0x2000)
#define STATUS_15                      (0x8000)
#define STATUS_16                      (0x10000)

  int xnum = set.numOutputs;
  if (xnum == 0)
     xnum = num;

  //  printf("xnum: %i \n",xnum);
  
  rdb.sparkCount.resize(xnum);
  rdb.sparkTime.resize(xnum);

  gOutputStatus.Resize(xnum);
  gSwitchStatus.Resize(xnum);
  gStatusStatus.Resize(xnum);
  gDemandStatus.Resize(xnum);
  gMeasuredStatus.Resize(xnum);

  for (int i=0; i<num; i++) {
     if ((rdb.status[i]&STATUS_ON) == 0)
        continue;

     if (rdb.status[i]&(STATUS_OutputCurrentLimited|STATUS_OutputFailureMaxCurrent)) {

        if (rdb.sparkTime[i] == 0) {
           rdb.sparkTime[i] = now;
           rdb.sparkCount[i] ++;
        }

        int age = time(NULL) - rdb.sparkTime[i];

        cm_msg(MINFO, frontend_name, "Spark on channel %d (HWCH %s), outputStatus 0x%x, spark count %d, spark age %d", i, rdb.names[i].c_str(), rdb.status[i], rdb.sparkCount[i], age);

        if (set.enableSparkMode) {
           if (age > set.sparkShutdownTime) {
	      cm_msg(MERROR, frontend_name, "Turning off channel %d (HWCH %s), outputStatus 0x%x, spark count %d, spark age %d", i, rdb.names[i].c_str(), rdb.status[i], rdb.sparkCount[i], age);
              set_snmp_int("outputSwitch", rdb.indices[i], 0);
	   } else
              set_snmp_int("outputSwitch", rdb.indices[i], 10);

           gNextRead = time(NULL) + 1;
        }
     } else {
        rdb.sparkTime[i] = 0;
     }
  }

  write_data_int(hDB, hVar, "switch",  num, rdb.switches);
  write_data_int(hDB, hVar, "status",  num, rdb.status);
  write_data_string(hDB, hRdb, "outputStatusString",  num, rdb.statusString, 100);
  write_data_float(hDB, hVar, "demandVoltage", num, rdb.demandV);
  write_data_float(hDB, hVar, "senseVoltage",  num, rdb.senseV);
  write_data_float(hDB, hVar, "current",       num, rdb.currents);
  write_data_int(hDB, hVar, "sparkCount",      num, rdb.sparkCount);

  if (rdb.sensorTemperature.size() > 0)
     write_data_int(hDB, hVar, "sensorTemperature", rdb.sensorTemperature.size(), rdb.sensorTemperature);

  if (rdb.fanSpeed.size() > 0)
     write_data_int(hDB, hVar, "fanSpeed", rdb.fanSpeed.size(), rdb.fanSpeed);

  if (rdb.fanAirTemperature.size() > 0)
     write_data_int(hDB, hVar, "fanAirTemperature", rdb.fanAirTemperature.size(), rdb.fanAirTemperature);

  open_hotlink(hDB, hSet);

  if (set.numOutputs != rdb.numOutputs) {
     update_settings();
     return 0;
  }

  static int prevSysMainSwitch = -1;
  if (prevSysMainSwitch != rdb.sysMainSwitch) {
     prevSysMainSwitch = rdb.sysMainSwitch;
     update_settings();
     return 0;
  }

  gMainStatus.Reset();

  gMainSwitchStatus.Reset();
  
  if (gWeSetMainSwitch && set.mainSwitch != rdb.sysMainSwitch) {
     // cannot disable this here - the red alarm will never fire because gWeSetMainSwitch will be always set to false // gWeSetMainSwitch = false;
     gMainSwitchStatus.Add(0, 100, kRed);
  } else if (rdb.sysMainSwitch == 0)
     gMainSwitchStatus.Add(0, 10, kWhite);
  else if (rdb.sysMainSwitch == 1)
     gMainSwitchStatus.Add(0, 20, kGreen);
  else
     gMainSwitchStatus.Add(0, 100, kRed);

  gMainStatus.Add(0, gMainSwitchStatus, 0);

  gOutputStatus.Reset();
  gSwitchStatus.Reset();
  gStatusStatus.Reset();
  gDemandStatus.Reset();
  gMeasuredStatus.Reset();

  for (int i=0; i<num; i++) {
     if (set.resistance[i]) {
	double current = rdb.currents[i];
	if (current < 0)
	   current = 0;
	double vcorr = set.resistance[i] * current;
	printf("chan %d, voltage %f / %f V, current %f A, resistance %f Ohm, correction %f V\n", i, set.odbDemand[i], set.demand[i], current, set.resistance[i], vcorr);

	set.demand[i] = set.odbDemand[i] + vcorr;
     }
     
     if (set.maxVoltage[i])
	if (set.demand[i] > set.maxVoltage[i])
	   set.demand[i] = set.maxVoltage[i];

     if (rdb.sysMainSwitch > 0) {
	if (fabs(set.demand[i] - rdb.demandV[i]) > 0.100) {
	   set_snmp_float("outputVoltage", rdb.indices[i], set.demand[i]);
	   
	   gDemandStatus.Add(i, 1000, kBlue);
	   
	   gNextRead = time(NULL);
	   gFastRead = 10;
	}
     }
  }

  for (int i=0; i<num; i++) {
     if (rdb.switches[i] != set.outputSwitch[i])
        gSwitchStatus.Add(i, 100, kRed);
     else if (rdb.switches[i] == 0)
        gSwitchStatus.Add(i, 10, kWhite);
     else if (rdb.switches[i] == 1)
        gSwitchStatus.Add(i, 20, kGreen);
     else
        gSwitchStatus.Add(i, 100, kRed);

     if ((rdb.status[i]&~STATUS_ENABLEKILL&~STATUS_15&~STATUS_16) == 0)
        gStatusStatus.Add(i, 10, kWhite);
     else if ((rdb.status[i]&~STATUS_ENABLEKILL&~STATUS_15&~STATUS_16) == STATUS_ON)
        gStatusStatus.Add(i, 20, kGreen);
     else if ((rdb.status[i]&~STATUS_ENABLEKILL&~STATUS_ON&~STATUS_15&~STATUS_16) == STATUS_RAMPUP)
        gStatusStatus.Add(i, 40, kBlue);
     else if ((rdb.status[i]&~STATUS_ENABLEKILL&~STATUS_ON&~STATUS_15&~STATUS_16) == STATUS_RAMPDOWN)
        gStatusStatus.Add(i, 40, kBlue);
     else if (rdb.sysMainSwitch == 0 && rdb.status[i] == STATUS_OutputFailureMinSenseVoltage) // special condition for VME and LVPS power supplies
        gStatusStatus.Add(i, 10, kWhite);
     else
        gStatusStatus.Add(i, 100, kRed);

     if ((rdb.status[i]&STATUS_ON) == 0)
        gDemandStatus.Add(i, 10, kWhite);
     else if (fabs(set.demand[i] - rdb.demandV[i]) < 1.0)
        gDemandStatus.Add(i, 20, kGreen);
     else
        gDemandStatus.Add(i, 30, kRed);
     
     if ((rdb.status[i]&STATUS_ON) == 0)
        gMeasuredStatus.Add(i, 10, kWhite);
     else if (fabs(set.demand[i] - fabs(rdb.senseV[i])) < 1.0)
        gMeasuredStatus.Add(i, 20, kGreen);
     else
        gMeasuredStatus.Add(i, 30, kYellow);
     
     gOutputStatus.Add(i, gSwitchStatus, i);
     gOutputStatus.Add(i, gStatusStatus, i);
     gOutputStatus.Add(i, gDemandStatus, i);
     gOutputStatus.Add(i, gMeasuredStatus, i);

     gMainStatus.Add(0, gOutputStatus, i);
  }

  check_temperatures();

  gMainStatus.Write();

  if (1) {
     static time_t x = 0;

     bool update = false;

     if (gMainStatus.fStatus[0] <= gDelayedMainStatus.fStatus[0])
	update = true;

     if (gMainStatus.fStatus[0] > gDelayedMainStatus.fStatus[0]) {
	if (x == 0) {
	   x = now + gMainStatusDelay;
	}

	if (now >= x)
	   update = true;

	//printf("status %d -> %d, delay %d, update %d\n", gDelayedMainStatus.fStatus[0], gMainStatus.fStatus[0], x-now, update);
     }

     if (update) {
	gDelayedMainStatus.Reset();
	gDelayedMainStatus.Add(0, gMainStatus, 0);
	gDelayedMainStatus.Write();
        set_eq_status();
	x = 0;
     }
  }

  gMainSwitchStatus.Write();

  gOutputStatus.Write();
  gSwitchStatus.Write();
  gStatusStatus.Write();
  gDemandStatus.Write();
  gMeasuredStatus.Write();

  return 0;
}

 INT poll_event(INT source, INT count, BOOL test)
/* Polling routine for events. Returns TRUE if event
   is available. If test equals TRUE, don't return. The test
   flag is used to time the polling */
{
  assert(!"poll_event is not implemented");
  return 0;
}

/* emacs
 * Local Variables:
 * tab-width: 8
 * c-basic-offset: 3
 * End:
 */

//end
