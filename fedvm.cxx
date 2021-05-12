/********************************************************************\

  Name:         fedvm.cxx
  Created by:   K.Olchanski

  Contents:     Generic frontend for HP/Agilent DVM

March 2013: Modified by T. Lindner for PTF.

  $Id$

\********************************************************************/

#include <stdio.h>
#include <errno.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
//#include <string>
#include <sys/time.h>
#include <assert.h>
#include "midas.h"
#include "evid.h"

#define EQ_NAME   "PTFDVM"
#define EQ_EVID   EVID_DVM
#define FE_NAME   "fedvm"

/* make frontend functions callable from the C framework */
#ifdef __cplusplus
extern "C" {
#endif

/*-- Globals -------------------------------------------------------*/

/* The frontend name (client name) as seen by other MIDAS clients   */
   char *frontend_name = FE_NAME;
/* The frontend file name, don't change it */
   char *frontend_file_name = __FILE__;

/* frontend_loop is called periodically if this variable is TRUE    */
   BOOL frontend_call_loop = FALSE;

/* a frontend status page is displayed with this frequency in ms */
   INT display_period = 0;

/* maximum event size produced by this frontend */
   INT max_event_size = 200*1024;

/* maximum event size for fragmented events (EQ_FRAGMENTED) */
   INT max_event_size_frag = 1024*1024;

/* buffer size to hold events */
   INT event_buffer_size = 1*1024*1024;

  extern HNDLE hDB;
  HNDLE   hMoveVariables =0;

/*-- Function declarations -----------------------------------------*/
  INT frontend_init();
  INT frontend_exit();
  INT begin_of_run(INT run_number, char *error);
  INT end_of_run(INT run_number, char *error);
  INT pause_run(INT run_number, char *error);
  INT resume_run(INT run_number, char *error);
  INT frontend_loop();
  
  INT read_dvm_event(char *pevent, INT off);
/*-- Bank definitions ----------------------------------------------*/

/*-- Equipment list ------------------------------------------------*/
  
  EQUIPMENT equipment[] = {

    {EQ_NAME,                 /* equipment name */
     {EVID_DVM, (1<<EVID_DVM), /* event ID, trigger mask */
      "SYSTEM",               /* event buffer */
      EQ_PERIODIC,            /* equipment type */
      0,                      /* event source */
      "MIDAS",                /* format */
      TRUE,                   /* enabled */
      RO_RUNNING|RO_ODB,       /* when to read this event */
      1000,                   /* poll time in milliseconds */
      0,                      /* stop run after this event limit */
      0,                      /* number of sub events */
      1,                      /* whether to log history */
      "", "", "",}
     ,
     read_dvm_event,          /* readout routine */
     NULL,
     NULL,
     NULL,       /* bank list */
    }
    ,
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

#include "utils.cxx"

/*-- Global variables ----------------------------------------------*/

static int gHaveRun        = 0;
static int gRunNumber      = 0; // from ODB

/*-- Frontend Init -------------------------------------------------*/

INT frontend_init()
{
  setbuf(stdout,NULL);
  setbuf(stderr,NULL);

  //cm_enable_watchdog(0);

  // Get the Move settings and keys
  INT status, rstate, size;
  char str[128];
  
  sprintf(str,"/Equipment/Move/Variables/");
  status = db_find_key(hDB, 0, str , &hMoveVariables);
  if (status != DB_SUCCESS)
  {
    cm_msg(MERROR, "frontend_init", "cannot get key for %s\n",str);
    return DB_NO_ACCESS;
  }


  return SUCCESS;
}

/*-- Frontend Exit -------------------------------------------------*/

INT frontend_exit()
{
  gHaveRun = 0;
  return SUCCESS;
}

/*-- Begin of Run --------------------------------------------------*/

INT begin_of_run(INT run_number, char *error)
{
  gRunNumber = run_number;
  gHaveRun = 1;

  printf("begin run: %d\n", run_number);

  return SUCCESS;
}

/*-- End of Run ----------------------------------------------------*/
INT end_of_run(INT run_number, char *error)
{
  gRunNumber = run_number;

  static bool gInsideEndRun = false;

  if (gInsideEndRun)
    {
      printf("breaking recursive end_of_run()\n");
      return SUCCESS;
    }

  gInsideEndRun = true;

  gHaveRun = 0;
  printf("end run %d\n",run_number);

  gInsideEndRun = false;

  return SUCCESS;
}

/*-- Pause Run -----------------------------------------------------*/
INT pause_run(INT run_number, char *error)
{
  gHaveRun = 0;
  gRunNumber = run_number;
  return SUCCESS;
}

/*-- Resume Run ----------------------------------------------------*/
INT resume_run(INT run_number, char *error)
{
  gHaveRun = 1;
  gRunNumber = run_number;
  return SUCCESS;
}

/*-- Frontend Loop -------------------------------------------------*/
INT frontend_loop()
{
  assert(!"frontend_loop() is not implemented");
  return SUCCESS;
}

/********************************************************************\

  Readout routines for different events

\********************************************************************/

/*-- Interrupt configuration ---------------------------------------*/
extern "C" INT interrupt_configure(INT cmd, INT source, PTYPE adr)
{
  assert(!"interrupt_configure() is not implemented");
  return 0;
}


#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <sys/stat.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <errno.h>
#include <ctype.h>
#include <unistd.h>
#include <assert.h>
#include <string.h>

int serial_trace = 0;

int serial_open(const char *addr)
{
  char laddr[256];
  strlcpy(laddr, addr, sizeof(laddr));
  char* s = strchr(laddr,':');
  if (!s)
    {
      cm_msg (MERROR, frontend_name, "serial_open: Invalid address \'%s\': no \':\', should look like \'hostname:tcpport\'", laddr);
      return -1;
    }

  *s = 0;

  int port = atoi(s+1);
  if (port == 0)
    {
      cm_msg (MERROR, frontend_name, "serial_open: Invalid address: \'%s\', tcp port number is zero", laddr);
      return -1;
    }

  struct hostent *ph = gethostbyname(laddr);
  if (ph == NULL)
    {
      cm_msg (MERROR, frontend_name, "serial_open: Cannot find ip address for \'%s\', errno %d (%s)", laddr, h_errno, hstrerror(h_errno));
      return -1;
    }

  int fd = socket (AF_INET, SOCK_STREAM, 0);
  if (fd < 0)
    {
      cm_msg (MERROR, frontend_name, "serial_open: Cannot create tcp socket, errno %d (%s)", errno, strerror(errno));
      return -1;
    }

  struct sockaddr_in inaddr;

  memset(&inaddr, 0, sizeof(inaddr));
  inaddr.sin_family = AF_INET;
  inaddr.sin_port = htons(port);
  memcpy((char *) &inaddr.sin_addr, ph->h_addr, 4);
  
  cm_msg(MINFO, "serial_open: Connecting to %s port %d", laddr, port);
  printf("serial_open: Connecting to %s port %d", laddr, port);
  int status = connect(fd, (sockaddr*)&inaddr, sizeof(inaddr));
  if (status == -1)
    {
      cm_msg (MERROR, frontend_name, "serial_open: Cannot connect to %s:%d, connect() error %d (%s)", laddr, port, errno, strerror(errno));
      return -1;
    }
  printf("Connected to %s:%d, connect())\n", laddr, port);

  int flags = fcntl(fd, F_GETFL, 0);
  flags |= O_NDELAY;

  status = fcntl (fd, F_SETFL, flags);
  if (status == -1)
    {
      cm_msg (MERROR, frontend_name, "serial_open: Cannot set no-delay mode, fcntl(F_SETFL,O_NDELAY) error %d (%s)", errno, strerror(errno));
      return -1;
    }

  return fd;
}

int serial_close(int fd)
{
  int status = close(fd);
  if (status == -1)
    {
      cm_msg (MERROR, frontend_name, "serial_close: close() error %d (%s)", errno, strerror(errno));
      return -1;
    }

  return 0;
}

int serial_write(int fd, const char *s)
{
  if (serial_trace)
    printf("serial_write: [%s]\n",s);

  /* Write string to the device */
  assert(fd > 0);

  int len = strlen(s);
  while (len > 0)
    {
      int status = write(fd, s, len);
      if (status == -1 && errno == EAGAIN)
	continue;
      if (status != len)
	{
	  cm_msg (MERROR, frontend_name, "serial_write: write() error %d (%s)", errno, strerror(errno));
	  return -1;
	}
      break;
    }

  return 0;
}

int serial_writeline(int fd, const char *s)
{
  char buf[256];
  snprintf(buf, sizeof(buf)-1, "%s\n", s);
  return serial_write(fd, buf);
}

int serial_readline(int fd, char *s, int maxlen, int timeout)
{
  /* Read response from the DVM into buffer
   * Read until LF or maxlen characters 
   * Waits at most timeout msec before declaring an error (0 == no timeout)
   */

  DWORD milli_start = ss_millitime();
  int nflush = 0;
  const char* inputbuf = s;
  
  assert(fd > 0);

  while (1)
    {
      int rd = read(fd, s, 1);

      if (rd == 0)
	{
	  cm_msg (MERROR, frontend_name, "serial_readline: EOF");
	  return -1;
	}

      //printf("serial_readline: rd %d, data 0x%x [%c]\n",rd,*s,*s);
      
      if (rd != 1)
	{
	  if (errno == EAGAIN)
	    {
	      if (timeout < 0)
		{
		  if (s==inputbuf)
		    {
		      if (serial_trace)
			printf("serial_readline: no data\n");
		      
		      return 0;
		    }

		  timeout = -timeout;
		}

	      if ((ss_millitime () - milli_start) > timeout)
		{
		  if (serial_trace)
		    printf("serial_readline: timeout, return -2\n");
		  //cm_msg (MERROR, frontend_name, "serial_readline: timeout");
		  *s = 0;
		  return -2;
		}

	      ss_sleep (10);
	      continue;
	    }

	  cm_msg (MERROR, frontend_name, "serial_readline: read() error %d (%s)", errno, strerror(errno));
	  return -1;
	}
      
      /* Telnet connections have these extra character sequences every now and then */
      
      if (nflush > 0)
	{
	  --nflush;
	  continue;
	}

      if (*s == (char)255)
	{
	  nflush = 2;
	  continue;
	}

      if (*s == '\r') continue;
      if (*s == '\n') break;
      if (!isprint(*s)) continue;
      if (--maxlen <= 0) break;
      s++;
    }

  *s = '\0';

  if (serial_trace)
    printf("serial_readline: return [%s]\n",inputbuf);

  return (s - inputbuf);
}

int serial_flush(int fd)
{
  char buf[128];
  int count = 0;
  int i;
  for (i=0; i<1000; i++)
    {
      int rd = read(fd,buf,sizeof(buf));
      if (rd > 0)
	{
	  count += rd;
	  continue;
	}

      if (rd == 0)
	{
	  cm_msg(MERROR, frontend_name, "serial_flush: unexpected EOF");
	  return -1;
	}

      // we are in the NDELAY mode
      if (errno == EAGAIN)
	{
	  printf("serial_flush: Flushed %d bytes\n",count);
	  return 0;
	}

      cm_msg(MERROR, frontend_name, "serial_flush: read() error %d (%s)", errno, strerror(errno));
      return -1;
    }

  cm_msg(MERROR, frontend_name, "serial_flush: the serial line is babbling");
  return -1;
}

/*-- calibrations --*/

struct Calib
{
  double mx;
  double my;
  double slope;

  Calib() // default ctor
  {
    mx = 0;
    my = 0;
    slope = 1;
  }

  void Init(const char* name) // default ctor
  {
    int n = odbReadArraySize(name);

    //printf("Array size %d\n", n);

    switch (n)
      {
      case 0:
        printf("Creating calibrations for %s\n", name);
        odbReadDouble(name, 3, 0);
	break;

      case 2:
	mx = 0;
	my = odbReadDouble(name, 0, 0);
	slope = odbReadDouble(name, 1, 1);
	break;

      case 3:
	mx = odbReadDouble(name, 0, 0);
	my = odbReadDouble(name, 1, 0);
	slope = odbReadDouble(name, 2, 1);
	break;

      case 4:
	double x1 = odbReadDouble(name, 0, 0);
	double y1 = odbReadDouble(name, 1, 0);
	double x2 = odbReadDouble(name, 2, 1);
	double y2 = odbReadDouble(name, 3, 1);
	if (x1 != x2)
	  {
	    mx = 0.5*(x1+x2);
	    my = 0.5*(y1+y2);
	    slope = (y1-y2)/(x1-x2);
	  }
	break;
      }

    printf("Calib %s, mx %f, my %f, slope %f\n", name, mx, my, slope);
  }

  double Calibrate(double v)
  {
    return my + slope*(v-mx);
  }

  double Reverse(double v)
  {
    return (v-my)/slope + mx;
  }
};

/*-- fehp -------------------------------------------------*/

/* Root of Equipment/Settings tree */
HNDLE hSettingRoot;

/* Root of Equipment/Variables tree */
HNDLE hVarRoot;

extern HNDLE hDB;
int isInitialized = 0;

/* File descriptor to talk to it */
int hpfd = -1;

/* Timeout for data readout, ms */
static int gScanTimeout; // from ODB

/* Timeout for serial comm during initialization, ms */
static int gInitTimeout = 5000;

/* Timeout for serial comm during error reporting, ms */
static int gErrTimeout = 10000;

/* Scan lists */
typedef struct
{
  char* scancmd;
  int   scanperiod;
  time_t nextscan;
} scanGroup_t;

#define kMaxScan 100
int numscan = 0;
scanGroup_t scan[kMaxScan];

static char *gIdleCmd;

#define MAXSLOTS 3
static HNDLE  gSlotKey[MAXSLOTS];
static int    gSlotNumChan[MAXSLOTS];

static double gSlotData[MAXSLOTS][100];
static Calib *gSlotCalib[MAXSLOTS][100];
static char   gSlotNames[MAXSLOTS][100][NAME_LENGTH];


// MIDAS ODB helpers

static HNDLE odbGetHandle(HNDLE hDB, HNDLE dir, char *name, int type)
{
  /* Find key in the ODB */
  HNDLE key;
  int status = db_find_key (hDB, dir, name, &key);
  if (type != 0 && status == DB_NO_KEY)
    {
      // create key if it does not exist
      status = db_create_key(hDB, dir, name, type);
      if (status != SUCCESS)
	{
	  cm_msg (MERROR, frontend_name, "Cannot create \'%s\', error %d", name, status);
	  abort();
	  // NOT REACHED
	  return 0;
	}
      status = db_find_key(hDB, dir, name, &key);
      cm_msg (MERROR, frontend_name, "Created ODB key \'%s\' of type %d", name, type);
    }

  if (status != SUCCESS)
    {
      if (dir==0)
	cm_msg (MERROR, frontend_name, "Cannot find odb handle for \'%s\', error %d", name, status);
      else
	cm_msg (MERROR, frontend_name, "Cannot find odb handle for %d/\'%s\', error %d", dir, name, status);
      abort();
      // NOT REACHED
      return 0;
    }

  return key;
}

static void odbPutString(HNDLE hDB, HNDLE hRoot, char* name, int size, char* string)
{
  HNDLE h;
  int status;

  h = odbGetHandle(hDB, hRoot, name, TID_STRING);
  status = db_set_data(hDB, h, string, size, 1, TID_STRING);
  if (status != SUCCESS)
    {
      cm_msg (MERROR, frontend_name, "Cannot write value of \'%s\', error %d", name, status);
      abort();
    }
}

// service functions

char** split(const char* buf)
{
  static char* str = NULL;
  static char* ptr[100];
  char* s;
  int i;

  if (str)
    free(str);

  str = strdup(buf);

  i = 0;
  ptr[i] = str;
  for (s=str; *s; s++)
    {
      if (*s == ',')
	{
	  *s = 0;
	  i++;
	  s++;
	  ptr[i] = s;
	}

    }

  i++;
  ptr[i] = 0;

  if (0)
    {
      printf("split [%s]\n",buf);
      for (i=0; ptr[i]; i++)
	printf("ptr[%d] is [%s]\n",i,ptr[i]);
    }

  return ptr;
}

// low-level DVM communication functions

int hpdvm_readError(int fd,char errbuf[],int errbufsize)
{
  int status = serial_writeline(fd, "syst:err?");
  if (status != 0)
    return status;
  status = serial_readline(fd, errbuf, errbufsize-1, gErrTimeout);
  if (status <= 0)
    {
      snprintf(errbuf,errbufsize-1,"Cannot read DVM error buffer, serial_readline error %d",status);
      return -1;
    }

  status = 0;
  sscanf (errbuf, "%d", &status);

  //printf("hpdvm_readError: DVM error %d - %s\n", status, errbuf);
  return status;
}

int hpdvm_errcheck(int fd,char errbuf[],int errbufsize)
{
  /* flush old errors, remember the last one */
  char buf[256];
  int status;
  int lasterr = 0;

  while (1)
    {
      status = hpdvm_readError(fd,buf,sizeof(buf));
      if (status == 0)
	return lasterr;

      if (lasterr != 0)
	cm_msg(MERROR,frontend_name,"Flush old DVM error: %s",errbuf);

      lasterr = status;
      strncpy(errbuf,buf,errbufsize-1);

      /* cannot read the error buffer */
      if (status == -1)
	return lasterr;
    }
}

int hpdvm_flushErrors(int fd)
{
  return 0;
}

int hpdvm_writeConfig(int fd,const char* s)
{
  int status;
  char errbuf[256];

  printf("hpdvm_writeConfig: [%s]\n",s);

  status = serial_writeline(fd, s);
  if (status != 0)
    return status;

  status = hpdvm_errcheck(fd,errbuf,sizeof(errbuf));
  if (status != 0)
    {
      cm_msg(MERROR,frontend_name,"hpdvm_writeConfig: [%s], error: %s\n",s,errbuf);
      return -1;
    }

  return 0;
}

/********************************************************************\
  
  Readout routines for different events

\********************************************************************/

/*-- Event readout -------------------------------------------------*/

int getRelayCycles(int hpfd, int ichan1, int ichan2)
{
  char buf[256];
  char reply[256];

  sprintf(buf, "diag:rel:cycl? (@%d:%d)", ichan1, ichan2);
  //printf("Send [%s]\n", buf);

  if (serial_writeline(hpfd, buf) != 0) return FE_ERR_DRIVER;
  if (serial_readline(hpfd, reply, sizeof(reply), 2000) <= 0) return FE_ERR_DRIVER;
  
  printf("Relay cycles for channels %d..%d: %s\n", ichan1, ichan2, reply);

  return SUCCESS;
}

int initSlot(int hpfd, int islot)
{
  char sslot[256];
  char buf[2560];
  sprintf(sslot, "syst:ctyp? %d", islot);
  if (serial_writeline(hpfd, sslot) != 0) return (FE_ERR_DRIVER);
  if (serial_readline(hpfd, buf, sizeof(buf), gInitTimeout) <= 0) return (FE_ERR_DRIVER);
  printf("DVM slot %d: %s\n", islot, buf);
  sprintf(sslot, "/equipment/" EQ_NAME "/Settings/Slot %d type", islot);
  odbPutString(hDB, 0, sslot, 40, buf);

  if (strstr(buf,"34901A") != NULL) // 20 chan mux
    {
      getRelayCycles(hpfd, islot+ 1, islot+10);
      getRelayCycles(hpfd, islot+11, islot+20);
      getRelayCycles(hpfd, islot+21, islot+22);
      getRelayCycles(hpfd, islot+95, islot+95);
      getRelayCycles(hpfd, islot+96, islot+96);
      getRelayCycles(hpfd, islot+97, islot+97);
      getRelayCycles(hpfd, islot+98, islot+98);
      getRelayCycles(hpfd, islot+99, islot+99);

      if (hpdvm_errcheck(hpfd, buf, sizeof(buf)) != 0)
	{
	  cm_msg(MERROR,frontend_name,"Unexpected DVM errors during initialization: %s\n",buf);
	  //return FE_ERR_DRIVER;
	}

      if (1)
	{
	  HNDLE h;
	  int status;
	  sprintf(sslot,"Names slot %d",islot);
	  status = db_find_key(hDB,hSettingRoot,sslot,&h);
	  if (status == DB_NO_KEY)
	    {
	      int i;
	      char sss[22][16];
	      status = db_create_key(hDB,hSettingRoot,sslot,TID_STRING);
	      assert(status == SUCCESS);
	      status = db_find_key(hDB,hSettingRoot,sslot,&h);
	      assert(status == SUCCESS);
	      for (i=0; i<20; i++)
		{
		  sprintf(sss[i],"ch%03d",islot+i+1);
		}
	      sprintf(sss[20],"ch%03d",islot+21);
	      sprintf(sss[21],"ch%03d",islot+22);
	      status = db_set_data(hDB,h,sss,sizeof(sss),22,TID_STRING);
	      assert(status == SUCCESS);
	    }

	  sprintf(sslot,"slot %d",islot);
	  status = db_find_key(hDB,hVarRoot,sslot,&h);
	  if (status == DB_NO_KEY)
	    {
	      int i;
	      double sss[22];
	      status = db_create_key(hDB, hVarRoot, sslot, TID_DOUBLE);
	      assert(status == SUCCESS);
	      status = db_find_key(hDB, hVarRoot, sslot, &h);
	      assert(status == SUCCESS);
	      for (i=0; i<22; i++)
		sss[i] = 0;
	      status = db_set_data(hDB, h, sss, sizeof(sss), 22, TID_DOUBLE);
	      assert(status == SUCCESS);
	    }

	  if (1)
	    {
	      int i = islot/100-1;
	      int size;
	      gSlotKey[i] = h;
	      gSlotNumChan[i] = 22;
	      size = sizeof(double)*gSlotNumChan[i];
	      status = db_get_data(hDB, gSlotKey[i], gSlotData[i], &size, TID_DOUBLE);
	      assert(status == SUCCESS);
	    }

	  if (1)
	    {
	      char s[256];
	      sprintf(s,"/Equipment/%s/Settings/Names Slot %d", EQ_NAME, islot);
	      for (int i=0; i<22; i++)
		{
		  sprintf(gSlotNames[islot/100-1][i], "ch%d", islot + i);
		  const char* name = odbReadString(s, i, NULL, 200);
		  if (name && strlen(name) > 0)
		    strlcpy(gSlotNames[islot/100-1][i], name, NAME_LENGTH);
		}
	    }

	  if (1)
	    {
	      for (int i=0; i<22; i++)
		{
		  char cname[256];
		  sprintf(cname,"/Equipment/%s/Settings/Calib %s", EQ_NAME, gSlotNames[islot/100-1][i]);
		  int sz = odbReadArraySize(cname);
		  if (sz > 1)
		    {
		      gSlotCalib[islot/100-1][i] = new Calib();
		      gSlotCalib[islot/100-1][i]->Init(cname);

		      sprintf(cname, "/Equipment/%s/Variables/Calib %s", EQ_NAME, gSlotNames[islot/100-1][i]);
		      odbReadDouble(cname, 0, 0);
		    }
		  else
		    gSlotCalib[islot/100-1][i] = NULL;
		}
	    }
	}
    }
  else if (strstr(buf,"34908A") != NULL) // 40 chan MUX
    {
      sprintf(sslot, "diag:rel:cycl? (@%d,%d)",islot+98,islot+99);
      printf("Send [%s]\n",sslot);
      if (serial_writeline(hpfd, sslot) != 0) return (FE_ERR_DRIVER);
      if (serial_readline(hpfd, buf, sizeof(buf), 10000) <= 0) { /* ignore */ } ; // return (FE_ERR_DRIVER);

      printf("Relay cycles: %s\n",buf);

      getRelayCycles(hpfd, islot+ 1, islot+10);
      getRelayCycles(hpfd, islot+11, islot+20);
      getRelayCycles(hpfd, islot+21, islot+30);
      getRelayCycles(hpfd, islot+31, islot+40);
      getRelayCycles(hpfd, islot+21, islot+22);
      getRelayCycles(hpfd, islot+98, islot+99);

      if (hpdvm_errcheck(hpfd,buf,sizeof(buf)) != 0)
	{
	  cm_msg(MERROR,frontend_name,"Unexpected DVM errors during initialization: %s\n",buf);
	  //return FE_ERR_DRIVER;
	}

      if (1)
	{
	  HNDLE h;
	  int status;
	  sprintf(sslot,"Names slot %d",islot);
	  status = db_find_key(hDB,hSettingRoot,sslot,&h);
	  if (status == DB_NO_KEY)
	    {
	      int i;
	      char sss[40][16];
	      status = db_create_key(hDB,hSettingRoot,sslot,TID_STRING);
	      assert(status == SUCCESS);
	      status = db_find_key(hDB,hSettingRoot,sslot,&h);
	      assert(status == SUCCESS);
	      for (i=0; i<40; i++)
		{
		  sprintf(sss[i],"ch%03d",islot+i+1);
		}
	      status = db_set_data(hDB,h,sss,sizeof(sss),40,TID_STRING);
	      assert(status == SUCCESS);
	    }

	  sprintf(sslot,"slot %d",islot);
	  status = db_find_key(hDB,hVarRoot,sslot,&h);
	  if (status == DB_NO_KEY)
	    {
	      int i;
	      double sss[40];
	      status = db_create_key(hDB, hVarRoot, sslot, TID_DOUBLE);
	      assert(status == SUCCESS);
	      status = db_find_key(hDB, hVarRoot, sslot, &h);
	      assert(status == SUCCESS);
	      for (i=0; i<40; i++)
		sss[i] = 0;
	      status = db_set_data(hDB, h, sss, sizeof(sss), 40, TID_DOUBLE);
	      assert(status == SUCCESS);
	    }

	  if (1)
	    {
	      int i = islot/100-1;
	      int size;
	      gSlotKey[i] = h;
	      gSlotNumChan[i] = 40;
	      size = sizeof(double)*gSlotNumChan[i];
	      status = db_get_data(hDB, gSlotKey[i], gSlotData[i], &size, TID_DOUBLE);
	      assert(status == SUCCESS);
	    }
	}
    }
  else
    {
      printf("Unknown module type %s\n",buf);
    }

  return SUCCESS;
}

int initialize()
{
  int status;
  char buf[256];

  //cm_set_watchdog_params (FALSE, 0);

  /* Open the Device */
  const char* devname = odbReadString("/Equipment/" EQ_NAME "/Settings/Device", 0, "localhost:9999", 32);

  /* Get handles to various points in the ODB tree */
  sprintf(buf, "/Equipment/%s/Settings", equipment[0].name);
  assert(strlen(buf) < sizeof(buf));

  hSettingRoot = odbGetHandle(hDB,0,buf,0);

  sprintf(buf, "/Equipment/%s/Variables", equipment[0].name);
  assert(strlen(buf) < sizeof(buf));

  hVarRoot = odbGetHandle(hDB, 0, buf, 0);

  hpfd = serial_open(devname);
  if (hpfd < 0)
    {
      cm_msg (MERROR, "frontend_init", "Cannot open \'%s\'",devname);
      return FE_ERR_HW;
    }

  /* Initialize DVM */
  printf("Clearing state errors\n");

  /* Clear stale errors, ignore all errors */
  hpdvm_writeConfig(hpfd, "*cls");


  /* Reset, check that there are no errors. */
  if (hpdvm_writeConfig(hpfd, "*rst") != 0) return (FE_ERR_DRIVER);

  /* Clear event register, error queue, alarm queue, p.291 */

  if (hpdvm_writeConfig(hpfd, "*cls") != 0) return (FE_ERR_DRIVER);


  /* Identify the DVM.
   * Apart from the fact that this might be useful information to have,
   * it is a test of whether or not I can really talk to this thing, since
   * it requires a response from the device.
   */
  if (serial_writeline(hpfd, "*idn?") != 0) return (FE_ERR_DRIVER);
  if (serial_readline(hpfd, buf, sizeof(buf), gInitTimeout) <= 0) return (FE_ERR_DRIVER);
  printf("DVM identification: %s\n", buf);

  if (serial_writeline(hpfd, "syst:vers?") != 0) return (FE_ERR_DRIVER);
  if (serial_readline(hpfd, buf, sizeof(buf), gInitTimeout) <= 0) return (FE_ERR_DRIVER);
  printf("DVM SCPI version: %s\n", buf);


  if (serial_writeline(hpfd, "diag:dmm:cycles?") != 0) return (FE_ERR_DRIVER);
  if (serial_readline(hpfd, buf, sizeof(buf), gInitTimeout) <= 0) return (FE_ERR_DRIVER);
  printf("DMM relay cycles: %s\n", buf);
  {
    char** s = split(buf);
    int c[3];
    int status;
    HNDLE h;
    c[0] = atoi(s[0]);
    c[1] = atoi(s[1]);
    c[2] = atoi(s[2]);

    h = odbGetHandle(hDB, hVarRoot, "DMM relay cycles", TID_INT);
    status = db_set_data(hDB, h, c, 3*sizeof(int), 3, TID_INT);
    if (status != SUCCESS)
      {
	cm_msg(MERROR,frontend_name,"Cannot write ODB \"DMM replay cycles\", status %d",status);
	abort();
      }
  }


  status = initSlot(hpfd, 100); if (status != SUCCESS) return FE_ERR_DRIVER;
  status = initSlot(hpfd, 200); if (status != SUCCESS) return FE_ERR_DRIVER;
  status = initSlot(hpfd, 300); if (status != SUCCESS) return FE_ERR_DRIVER;

  if (hpdvm_errcheck(hpfd, buf, sizeof(buf)) != 0)
    {
      cm_msg(MERROR,frontend_name,"Unexpected DVM errors during initialization: %s\n",buf);
      return FE_ERR_DRIVER;
    }

  /* Timeout for HPDVM scans */
  gScanTimeout = odbReadInt("/Equipment/" EQ_NAME "/Settings/Timeout", 0, 60000);


  int n = odbReadArraySize("/Equipment/" EQ_NAME "/Settings/Config");
  for (int i=0; i<n; i++)
    {
      /* Get configuration strings for channels */
      const char* cfg = odbReadString("/Equipment/" EQ_NAME "/Settings/Config", i, NULL, 255);

      if (!cfg)
	break;

      if (strlen(cfg) < 2)
	break;

      status = hpdvm_writeConfig(hpfd, cfg);
      if (status != 0)
	{
	  cm_msg (MERROR, frontend_name, "Cannot configure the DVM");
	  return FE_ERR_HW;
	}
    }

  /* Set up Scan Lists */

  numscan = 0;


  int nn = odbReadArraySize("/Equipment/"EQ_NAME"/Settings/Scan");
  for (int i=0; i<nn; i++)
    {
      const char* buf = odbReadString("/Equipment/"EQ_NAME"/Settings/Scan", i, NULL, 255);

      if (!buf)
	break;

      if (strlen(buf) < 2)
	break;

      scan[i].scancmd = strdup(buf);
      scan[i].scanperiod = odbReadInt("/Equipment/"EQ_NAME"/Settings/ScanPeriod", i);

      numscan = i+1;
    }

  printf("numscan %i\n",numscan);
  /* while we're here, is there an idle command? */

  if (gIdleCmd)
    {
      free (gIdleCmd);
      gIdleCmd = NULL;
    }

  for (int i=0; i<1; i++) // loop once
    {
      const char* buf = odbReadString("/Equipment/"EQ_NAME"/Settings/Idle", 0, NULL, 255);

      if (!buf)
	break;

      if (strlen(buf) < 2)
	break;

      gIdleCmd = strdup(buf);
    }
    
  /* Trigger source comes from me */
  if (hpdvm_writeConfig(hpfd, "trig:sour imm") != 0) return (FE_ERR_DRIVER);

  /* Include channel number with readout */
  if (hpdvm_writeConfig(hpfd, "form:read:chan on") != 0) return (FE_ERR_DRIVER);

  /* Save current state, and enable "Reload on Power on" */
  if (hpdvm_writeConfig(hpfd, "*sav 0") != 0) return (FE_ERR_DRIVER);

  /* On power-on recall settings from storage location "0" */
  if (hpdvm_writeConfig(hpfd, "mem:stat:rec:auto on") != 0) return (FE_ERR_DRIVER);



  /* Fill the queues */

  time_t now;
  time(&now);

  for (int i=0; i<numscan; i++ )
    {
      if (scan[i].scanperiod > 0)
	scan[i].nextscan = now;
      else
	scan[i].nextscan = 0;

      printf("scan group %d: period %d ms, scancmd: %s\n",
	     i,scan[i].scanperiod,scan[i].scancmd);
    }

  if (gIdleCmd)
    printf("idle command: %s\n",gIdleCmd);

     
  return SUCCESS;
}

void commtest()
{
  int err;
  while (1)
    {
      /* show message, p.266 */
      err = hpdvm_writeConfig(hpfd, "disp:text 'comm'");
      if (err != 0) break;
      err = hpdvm_writeConfig(hpfd, "disp:text 'test'");
      if (err != 0) break;
    }
  printf("communication test failure!\n");
  exit(1);
}

int start_scan(int igroup)
{
  char buf[256];

  /*printf("doscan: scan group %d\n",igroup);*/

  /* set to local mode, p.269 */
  serial_writeline(hpfd, "syst:rem");
  
  /* show message, p.266 */
  //serial_writeline(hpfd, "disp:text 'scan...'");
  sprintf(buf,"disp:text '%s'",frontend_name);
  assert(strlen(buf) < sizeof(buf));
  serial_writeline(hpfd,buf);
   
  /* send the scan list */

  if (serial_writeline(hpfd, scan[igroup].scancmd) != 0){
    printf("-1!\n");
    return -1;
  }
      /* check error status */
      int status = hpdvm_errcheck(hpfd, buf, sizeof(buf));
      if (status != 0)
	{
	  extern INT fe_stop; // in mfe.c
	  cm_msg(MERROR,frontend_name,"DVM scan error %d: %s",status,buf);
	  fe_stop = 1;
	}


  if (serial_writeline(hpfd,"read?") != 0) return -1;

  return 0;
}

void parse_data(const char*buf)
{
  const char* sptr;

  for (sptr = buf; ; )
    {
      int chan;
      double value;

      //printf("sptr [%s]\n",sptr);
      value = strtod(sptr,(char**)&sptr);
      //printf("value %f, sptr [%s]\n",value,sptr);
      if (sptr[0] != ',')
	{
	  cm_msg(MERROR,frontend_name,"DVM scan error: Bad data format at %d: %s",(sptr-buf),buf);
	  return;
	}
      sptr ++;
      chan = strtol(sptr,(char**)&sptr,10);
      //printf("channel %d, sptr [%s]\n",chan,sptr);

      gSlotData[chan/100-1][chan%100-1] = value;

      if (gSlotCalib[chan/100-1][chan%100-1])
	{
	  double cvalue = gSlotCalib[chan/100-1][chan%100-1]->Calibrate(value);
	  char name[256];
	  sprintf(name, "/Equipment/%s/Variables/Calib %s", EQ_NAME, gSlotNames[chan/100-1][chan%100-1]);
	  odbWriteDouble(name, 0, cvalue);
	}

      if (sptr[0] == 0) break;
      if (sptr[0] != ',')
	{
	  cm_msg(MERROR,frontend_name,"DVM scan error: Bad data format at %d: %s",(sptr-buf),buf);
	  return;
	}
      sptr ++;
    }
}

void end_scan()
{
  int i, status;
  char buf[256];

  /* check error status */
  status = hpdvm_errcheck(hpfd, buf, sizeof(buf));
  if (status != 0)
    {
      extern INT fe_stop; // in mfe.c
      cm_msg(MERROR,frontend_name,"DVM scan error %d: %s",status,buf);
      fe_stop = 1;
    }

  if (gIdleCmd)
    {
      /* send idle command from ODB */
      serial_writeline(hpfd, gIdleCmd);

      /* check error status */
      status = hpdvm_errcheck(hpfd, buf, sizeof(buf));
      if (status != 0)
	{
	  extern INT fe_stop; // in mfe.c
	  cm_msg(MERROR,frontend_name,"DVM scan error %d: %s",status,buf);
	  fe_stop = 1;
	}
    }

  /* show message, p.266 */
  sprintf(buf,"disp:text '%s'", EQ_NAME);
  assert(strlen(buf) < sizeof(buf));
  serial_writeline(hpfd,buf);

  /* set to local mode */
  serial_writeline(hpfd, "syst:loc");

  /* save data to ODB */
  for (i=0; i<MAXSLOTS; i++)
    if (gSlotKey[i])
      {
	status = db_set_data(hDB, gSlotKey[i], gSlotData[i], sizeof(double)*gSlotNumChan[i], gSlotNumChan[i], TID_DOUBLE);
	assert(status == SUCCESS);
      }
}

#define gEvMax 400
static void *gEvParam[gEvMax];

void schedule_event(void*param,int channel)
{
  assert(channel>=0);
  assert(channel<gEvMax);
  gEvParam[channel] = param;
}

int read_dvm_event(char *pevent, INT off)
{
  static time_t gScanStartTime = 0;

  INT status, rstate, size;
  char str[128];
  BOOL moving;
  size = sizeof(moving);  
  status = db_get_value(hDB,hMoveVariables,"Moving",&moving,&size,TID_BOOL,FALSE);
  if(status != DB_SUCCESS)
  {
    cm_msg(MERROR, "begin_of_run", "cannot get value for Moving\n");
    return DB_NO_ACCESS;
  }
  if(moving){
    printf("Gantry still moving. wait.\n");
    return 0;    
  }

  printf("read DVM\n");
  if (!isInitialized)
    { 
      static time_t gNextTry = 0;
      static int gRetry = 2;

      int st;
      time_t now;

      time(&now);

      if (gNextTry == 0)
	gNextTry = now - 10;

      if (now < gNextTry)
	return 0;

      st = initialize();
      if (st != SUCCESS)
	{
	  if (hpfd > 0)
	    {
	      serial_close(hpfd);
	      hpfd = 0;
	    }
	  printf("Here?\n");

	  gNextTry = now + gRetry;
	  cm_msg (MERROR, "frontend_init", "Cannot initialize, error %d, will retry in %d seconds.",st,gRetry);

	  if (gRetry <= 1*60*60)
	    gRetry = 2*gRetry;

	  if (gRetry > 15)
	    {
	      char text[256];
	      sprintf(text,"Cannot initialize %s, will retry, see messages",frontend_name);
	      al_trigger_alarm(frontend_name,text,"Warning","",AT_INTERNAL);
	    }

	  return 0;
	}

      isInitialized = 1;
      cm_msg (MINFO, "frontend_init", "Initialized and configured.");
    }
 if (1)
    {
      // call user event callbacks
      static int gEvPtr = 0;
      int evptr;

      if (gEvPtr >= gEvMax)
	gEvPtr = 0;

      evptr = gEvPtr;
      do
	{
	  if (gEvParam[gEvPtr])
	    {
	      int size = 0;

	      //	      if (isRunning) // generate events only when running
	      //		size = user_makeEvent(gbl_run_number,gEvParam[gEvPtr],gEvPtr,pevent);

	      gEvParam[gEvPtr] = 0;

	      if (size > 0) // do not generate empty events
		{
		  gEvPtr++;
		  return size;
		}
	    }

	  gEvPtr++;

	  if (gEvPtr >= gEvMax)
	    gEvPtr = 0;
	}
      while (gEvPtr != evptr);
      gEvPtr++;
    }
  printf("gScanStartTime %i DVM\n",gScanStartTime);

  if (gScanStartTime)
    {
      int status;
      char buf[10*1024];
      time_t now;

      status = serial_readline(hpfd, buf, sizeof(buf)-1, -gScanTimeout);

      if (status < 0)
	{
	  cm_msg(MERROR,frontend_name,"DVM scan error %d: Error in serial_readline, aborting",status);
	  abort();
	}
      else if (status > 0)
	{
	  //printf("scan data [%s]\n",buf);

	  parse_data(buf);
	  end_scan();

	  gScanStartTime = 0;
	  return 0;
	}

      time(&now);
      if (now - gScanStartTime > gScanTimeout/1000)
	{
	  cm_msg(MERROR,frontend_name,"DVM scan error %d: Timeout waiting for DVM data",status);
	  gScanStartTime = 0;
	}
    }
  else
    {
      static int gScan = 0;
      int iscan;
      time_t now;

      time(&now);

      if (gScan >= numscan)
	gScan = 0;

      iscan = gScan;
      do
	{
	  if (scan[gScan].scanperiod > 0 && now >= scan[gScan].nextscan)
	    {
	      printf("start_scan\n");
	      int status = start_scan(gScan);
	      if (status != 0)
		{
		  cm_msg(MERROR,frontend_name,"Error %d in start_scan, aborting",status);
		  abort();
		  return 0;
		}

	      gScanStartTime = now;

	      scan[gScan].nextscan = now + scan[gScan].scanperiod/1000;

	      gScan++;
	      printf("no event generated\n");
	      return 0; // no event generated
	    }

	  gScan++;

	  if (gScan >= numscan)
	    gScan = 0;
	}
      while (gScan != iscan);
      gScan++;
    }

  /* If I really am idle, wait a while and try again */
  ss_sleep (200);
  return 0;
}

extern "C" INT poll_event(INT source, INT count, BOOL test)
/* Polling routine for events. Returns TRUE if event
   is available. If test equals TRUE, don't return. The test
   flag is used to time the polling */
{
  assert(!"poll_event is not implemented");
  return 0;
}

//end
