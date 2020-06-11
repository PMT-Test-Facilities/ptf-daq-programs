/**
 *
 *   @author David Morris
 *   
 * $Log: cd_Galil.h,v $
 * Revision 1.1  2011/04/07 00:19:58  cadfael
 * Working version with fairly good motion
 *
 * Revision 1.1.1.1  2006/11/26 02:19:45  cadfael
 * Importing from Windows. System working but needs fine tuning
 * Runs motors, reads ADCs controls laser etc.
 *
 * Revision 1.1  2004/09/17 23:56:43  cadfael
 * Motors turning, rudimentary positioning happening, MSCB lasting for several days.
 *
 */

/* function prototypes */

INT cd_Galil(INT cmd, PEQUIPMENT pequipment);
INT cd_Galil_read(char *pevent, INT off);
