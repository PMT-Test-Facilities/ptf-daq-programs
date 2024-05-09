/********************************************************************\

  Name:         experim.h
  Created by:   ODBedit program

  Contents:     This file contains C structures for the "Experiment"
                tree in the ODB and the "/Analyzer/Parameters" tree.

                Additionally, it contains the "Settings" subtree for
                all items listed under "/Equipment" as well as their
                event definition.

                It can be used by the frontend and analyzer to work
                with these information.

                All C structures are accompanied with a string represen-
                tation which can be used in the db_create_record function
                to setup an ODB structure which matches the C structure.

 Created on:   Wed Oct 25 13:50:40 2006

 Restructured by Tom F. on Oct 24 2014
                           Jan 21 2015
 Last modified by Anubhav Prakash on Jul 07 2023
\********************************************************************/


#define SCAN_COMMON_STR(_name) char *_name[] = {\
"[.]",\
"Event ID = WORD : 1",\
"Trigger mask = WORD : 1",\
"Buffer = STRING : [32] SYSTEM",\
"Type = INT : 1",\
"Source = INT : 0",\
"Format = STRING : [8] MIDAS",\
"Enabled = BOOL : y",\
"Read on = INT : 1",\
"Period = INT : 100",\
"Event limit = DOUBLE : 0",\
"Num subevents = DWORD : 0",\
"Log history = INT : 0",\
"Frontend host = STRING : [32] midptf01.triumf.ca",\
"Frontend name = STRING : [32] feSCAN",\
"Frontend file name = STRING : [256] feScan_new.c",\
"",\
NULL }

#define SCAN_SETTINGS_DEFINED
typedef enum{
  CYLINDER,
  RECTANGULAR,
  ALIGNMENT,
  TILT_SCAN,//Anubhav's edit
  SPIN_SCAN, // this is a fixed point and it spins around 
  PATCH_SCAN, // the takenaka-san scan type 
  PASS_BY,
  PMTSURFACE,
  MANUAL,
  DEMO,
  TANKAVOIDANCE,
  FIX_POINT,
  ROTATION_SCAN,
} scan_type_t;

typedef enum{
  GANTRY0,
  GANTRY1,
  BOTH
} gantry_t;

//SubType  
typedef enum{
  DEFAULT,
  MONITOR_REF,
  NORM_INCIDENCE
} rect_sub_t;

/* Merge these (and have them as limit cases) or keep separate?
// Not necessary: h+phi define fixed point, stepsizes define what's fixed and which loop.
typedef enum{
  FIX_LATITUDE,   //fix h
  FIX_LONGITUDE,  //fix phi
  FIX_POINT       //fix both
} pmtsurf_sub_t;
*/

typedef struct {
  struct {
    float       height;
    float       radius;
    float       arc_step;
    float       loop_separation;
    float       layer_thickness;
    float       x_center;
    float       y_center;
    INT         which_gantry;
  } cyl_par;
  struct {
    float       prism_height_z;
    float       prism_length_x;
    float       prism_width_y;
    float       z_step;
    float       x_step;
    float       y_step;
    float       init_pos_z;
    float       init_pos_x;
    float       init_pos_y;
    float       init_tilt;
    float       init_rotation;
    INT         which_gantry;
    INT         subtype;
    struct{
      INT         nth_point;
      float       monitor_pos_x0;
      float       monitor_pos_y0;
      float       monitor_pos_z0;
      // And same for gantry1, *can* be the same point though
      float       monitor_pos_x1;
      float       monitor_pos_y1;
      float       monitor_pos_z1;
    } monitor_rect_par;
    /* None at the moment
    struct{
    } default_rect_par;
    struct{
    } accept_rect_par;
    */
    
  } rect_par;
  struct {
    float       init_pos[10];
    float       delta_rotation;
    float       delta_tilt;
    float       step_rotation;
    float       step_tilt;
    float       grid_length;
    float       grid_width;
    float       step_length;
    float       step_width;
    INT         gantry_laser;
  } align_par;  //Anubhav's edit start
  struct {
    float       theta;
    float       phi;
    float       step;
    } tilt_par; //Anubhav's edit end 
  struct {
    float       init_pos[5];
    float       velocity[5];
    INT         iterations;
    INT         gantry_laser;
  } pass_by_par;
  struct {
    float plane_angle_start;
    float plane_angle_end;
    float plane_angle_step;
    float tilt_angle_fixed;
    float starting_radius;
    float linear_step;
    float pmt_x;
    float pmt_y; 
  } rotaion_scan_par;
  struct{
    float azi_start;
    float zen_start;
    float azi_step;
    float zen_step;
    float azi_distance;
    float zen_distance;
    float init_x;
    float init_y;
    float init_z;
  } spin_scan_par;
  struct{
    float pmt_x;
    float pmt_y; 
    float pmt_tip_z;
    float pmt_angle_center;
    INT scan_dir;
  } patch_scan_par;
  //PMT coordinate system
  struct {
    //defines starting point on PMT surface
    float       init_height;
    float       init_polar;
    // angles of main PMT axis wrt z axis
    float       main_axis_alpha;
    float       main_axis_beta;

    float       delta_phi;
    float       delta_h;
  } pmt_surf_par;
  INT scan_type;
  INT meas_time;
  //  DWORD     timeout;
} SCAN_SETTINGS;

#define SCAN_SETTINGS_STR(_name) const char *_name[] = {\
"[CylinderParams]",\
"height = FLOAT : 0.4",\
"radius = FLOAT : 0.3",\
"arc_step = FLOAT : 0.2",\
"loop_separation = FLOAT : 0.1",\
"layer_thickness = FLOAT : 0.1",\
"x_center = FLOAT : 0.3",\
"y_center = FLOAT : 0.305",\
"which_gantry = INT : 0",\
"",\
"[RectangularParams]",\
"prism_height_z = FLOAT : 0.54",\
"prism_length_x = FLOAT : 0.647",\
"prism_width_y = FLOAT : 0.666",\
"z_step = FLOAT : 0.05",\
"x_step = FLOAT : 0.04",\
"y_step = FLOAT : 0.04",\
"init_pos_z = FLOAT : 0.",\
"init_pos_x = FLOAT : 0.",\
"init_pos_y = FLOAT : 0.",\
"init_tilt = FLOAT : 0.",\
"init_rotation = FLOAT : -90.",\
"which_gantry = INT : 2",\
"SubType = INT : 0",\
"",\
"[RectangularParams/MonitorPoint]",\
"nth_point = INT : 100",\
"monitor_pos_x0 = FLOAT : 0.145",\
"monitor_pos_y0 = FLOAT : 0.311",\
"monitor_pos_z0 = FLOAT : 0.4",\
"monitor_pos_x1 = FLOAT : 0.647",\
"monitor_pos_y1 = FLOAT : 0.666",\
"monitor_pos_z1 = FLOAT : 0.4",\
"",\
"[AlignmentParams]",\
"init_pos = FLOAT[10] : ",\
"0.1",\
"0.3",\
"0.2",\
"0.",\
"0.",\
"0.647",\
"0.3",\
"0.2",\
"0.",\
"0.",\
"delta_rotation = FLOAT : 3.0",\
"delta_tilt = FLOAT : 3.0",\
"step_rotation = FLOAT : 0.5",\
"step_tilt = FLOAT : 0.5",\
"grid_length = FLOAT : 0.05",\
"grid_width = FLOAT : 0.05",\
"step_length = FLOAT : 0.001",\
"step_width = FLOAT : 0.001",\
"gantry_laser = INT : 0",\
"",\
"[TiltParams]",\
"theta = FLOAT : 0",\			
"phi = FLOAT : -45",\
"step = FLOAT : 0.010",\
"",\
"[PassByParams]",\
"init_pos = FLOAT[5] : ",\
"0.0",\
"0.2",\
"0.0",\
"0.",\
"0.",\
"velocity = FLOAT[5] : ",\
"0.01",\
"0.01",\
"0.02",\
"1.3",\
"1.3",\
"iterations = INT : 20",\
"gantry_laser = INT : 0",\
"",\
"[PMTSurfaceParams]",\
"init_height = FLOAT : 0.2",\
"init_polar = FLOAT : 0.",\
"main_axis_alpha = FLOAT : 0.",\
"main_axis_beta = FLOAT : 0.",\
"delta_phi = FLOAT : 0.2",\
"delta_h = FLOAT : 0.02",\
"",\
"[.]",\
"ScanType = INT : 0",\
"meas_time = INT : 1000.",\
"",\
NULL }


