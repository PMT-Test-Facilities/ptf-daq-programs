#include "ScanSequence.hxx"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cmath>
#include <vector> //Anubhav's edit(to keep track, sorry I don't know git)
#include <sys/time.h>
#include <sstream>

#include "demo_path.cxx"

//globals from feScan, which I need here (are global any  way, otherwise pass by ref Init?)

/*
#ifdef __cplusplus
  };
#endif
*/

//----------------------------------------------------------
ScanSequence::ScanSequence(){
//----------------------------------------------------------

}

//----------------------------------------------------------
void ScanSequence::Init(SCAN_SETTINGS &fs_in, float *gantryLim){
//----------------------------------------------------------

  fs = fs_in;
  gGantryLimits = gantryLim;
  z_max_value = 0.08;
  return;
}


//---------Anubhav's edit(start)------------------------------------
bool ScanSequence::is_destinationvalid(const std::vector<double>& l){
  double gx = l[0];
  double gy = l[1];
  double gz = l[2];
  double gphi = l[4];
  double r_less = round((0.104 + std::sqrt(0.263 * 0.263 - pow(0.388 - gz, 2))) * 1e3);
  double r_more = round((0.104 + std::sqrt(0.290 * 0.290 - pow(0.388 - gz, 2))) * 1e3);
  double Z = round(gz * 1e3);
  double Y = round(gy * 1e3);
  double X = round(gx * 1e3);

  if (X < 0 || Y < 0 || Z < 0) {
    return false;
  }
  else if (Z > 0.400 * 1e3 || X > 0.649 * 1e3 || Y > 0.566 * 1e3) {
    return false;
  }
  else if (round(gphi) >= -45) {
    if (0 <= Z && Z < 180 && sqrt(pow((Z - 515), 2) + pow((Y - 265), 2) + pow((X - 333), 2)) < 428) {
      return false;
    }
    else if (180 <= Z && Z < 243 && sqrt(pow((X - 333), 2) + pow((Y - 265), 2)) < r_less) {
      return false;
    }
    else if (Z >= 243 && sqrt(pow((X - 333), 2) + pow((Y - 265), 2)) <= 393) {
      return false;
    }
  }
  else if (round(gphi) < -45) {
    if (0 <= Z && Z < 160 && sqrt(pow((Z - 515), 2) + pow((Y - 265), 2) + pow((X - 333), 2)) < 455) {
      return false;
    }
    else if (160 <= Z && Z < 216 && sqrt(pow((X - 333), 2) + pow((Y - 265), 2)) < r_more) {
      return false;
    }
    else if (Z >= 216 && sqrt(pow((X - 333), 2) + pow((Y - 265), 2)) <= 393) {
      return false;
    }
  }
  return true;
}

//--Anubhav's edit(end)-----------------------------------------



//----------------------------------------------------------
int ScanSequence::GeneratePath(std::vector<std::vector<double> > &points_in){
//----------------------------------------------------------


  // Generate path based on scan parameters:
  point_num = 0;
  int non_zero_points = 0;

  
  cm_msg(MINFO, "GeneratePath", "Generating path with kind %i: %s.\n", fs.scan_type,
      fs.scan_type == CYLINDER ? "cylindrical" :
      fs.scan_type == RECTANGULAR ? "rectangular" :
      fs.scan_type == PMTSURFACE ? "PMT surface" :
      fs.scan_type == DEMO ? "demo" :
      fs.scan_type == MANUAL ? "manual" :
      fs.scan_type == TANKAVOIDANCE ? "tank avoidance" :
      fs.scan_type == ALIGNMENT ? "alignment" :
      fs.scan_type == TILT_SCAN ? "tilt Scan" : //Anubhav's edit
      fs.scan_type == FIX_POINT ? "fixed point [NOT IMPLEMENTED]":
      "INVALID"
  );
  switch(fs.scan_type){
    case CYLINDER:
      non_zero_points = CylinderPath(points_in);
      break;
    case RECTANGULAR:
      non_zero_points = RectangularPath(points_in);
      break;
    case PMTSURFACE:
      non_zero_points = PMTSurfacePath(points_in);
      break;
    case DEMO:
      non_zero_points = DemoPath(points_in);
      break;
    case MANUAL:
      non_zero_points = ManualPath(points_in);
      break;
    case TANKAVOIDANCE:
      non_zero_points = TankAvoidancePath(points_in);
      break;
    case ALIGNMENT:
      non_zero_points = AlignmentPath(points_in);
      break;
  case TILT_SCAN: //Anubhav's edit
      non_zero_points = TiltPath(points_in);
      break;
    case FIX_POINT:
      //    non_zero_points = FixedPointPath(points_in);
    default:
      cm_msg(MERROR,"GeneratePath","Invalid scan type.");
      return 0;

  }//end switch case
  
  //Return back to base
  ReturnToBase(points_in);
  non_zero_points++;


  //Return the number of points of the path
  //return points_in.size();  //use this if we'll push_back to the vector
  return non_zero_points;

}


//--Anubhav's edit-----------------------------------------

int ScanSequence::TiltPath(std::vector<std::vector<double> > &points){

  std::vector<std::vector<double>> planepoints;
  std::vector<std::vector<double>> points1;

  /*

  UPDATE THE BELOW 5 VARIABLES TO BE DEPENDANT UPON THE USER INPUT.

  The variables "step", "phi" and "theta" are the three parameters 
  that will be specific to a scan and has to be taken from the 
  frontend "Scan" webpage of the midptf website!

  */  

  const float step = fs.tilt_par.step; //0.002;  // in meters
  const float phi1 = fs.tilt_par.phi; //-45;    // to be taken from the user as input (in degrees)
  const float phi = -phi1 * 3.14159/180.0;
  const float theta1 = fs.tilt_par.theta;//95;   // to be taken from the user as input (in degrees)
  const float theta = theta1 * 3.14159/180.0;
  
  int nlows = 0;
  points.clear();

  if (-90 < theta1 && theta1 < 0) {
    for (int i = 0; i < std::floor(0.650/step); i++) {
      points1.clear();
      planepoints.clear();
      for (int j = -1*std::floor(0.650/step); j < std::floor(0.650/step); j++) {
	for (int k = -1*std::floor(0.650/step); k < std::floor(0.650/step); k++) {
	  std::vector<double> p = {
	    std::cos(phi) * std::cos(theta) * step * i - j * step * std::sin(theta) - k * step * std::sin(phi) * std::cos(theta),
	    0.566 + std::cos(phi) * std::sin(theta) * step * i + j * step * std::cos(theta) - k * step * std::sin(phi) * std::sin(theta),
	    std::sin(phi) * step * i + k * step * std::cos(phi),
	    theta1+104,
	    phi1,
	    -99999,
	    -99999,
	    -99999,
	    -99999,
            -99999
	  };
	  planepoints.push_back(p);
	}
      }
      for (const auto& i : planepoints) {
	if (is_destinationvalid(i)) {
	  points1.push_back(i);
	}
      }
      if (points1.size() > points.size()) {
	points = points1;
      }
      else if (points1.size() < points.size()) {
	nlows++;
      }
      if (nlows == 3) {
	break;
      }
    }
  }

  if (90 < theta1 && theta1 < 180) {
    for (int i = 0; i < std::floor(0.650/step); i++) {
      points1.clear();
      planepoints.clear();
      for (int j = -1*std::floor(0.650/step); j < std::floor(0.650/step); j++) {
	for (int k = -1*std::floor(0.650/step); k < std::floor(0.650/step); k++) {
	  std::vector<double> p = {
	    0.649 + std::cos(phi) * std::cos(theta) * step * i - j * step * std::sin(theta) - k * step * std::sin(phi) * std::cos(theta),
	    std::cos(phi) * std::sin(theta) * step * i + j * step * std::cos(theta) - k * step * std::sin(phi) * std::sin(theta),
	    std::sin(phi) * step * i + k * step * std::cos(phi),
	    theta1-76,
	    -180 - phi1,
	    -99999,
	    -99999,
	    -99999,
	    -99999,
            -99999
	  };
	  planepoints.push_back(p);
	}
      }
      for (const auto& i : planepoints) {
	if (is_destinationvalid(i)) {
	  points1.push_back(i);
	}
      }
      if (points1.size() > points.size()) {
	points = points1;
      }
      else if (points1.size() < points.size()) {
	nlows++;
      }
      if (nlows == 3) {
	break;
      }
    }
  }

  if (0 <= theta1 && theta1 <= 90) {
    for (int i = 0; i < std::floor(0.650/step); i++) {
      points1.clear();
      planepoints.clear();
      for (int j = -1*std::floor(0.650/step); j < std::floor(0.650/step); j++) {
	for (int k = -1*std::floor(0.650/step); k < std::floor(0.650/step); k++) {
	  std::vector<double> p = {
	    std::cos(phi) * std::cos(theta) * step * i - j * step * std::sin(theta) - k * step * std::sin(phi) * std::cos(theta),
	    std::cos(phi) * std::sin(theta) * step * i + j * step * std::cos(theta) - k * step * std::sin(phi)* std::sin(theta),
	    std::sin(phi)* step* i + k * step * std::cos(phi),
	    theta1-76,
	    -180-phi1,
	    -99999,
	    -99999,
	    -99999,
	    -99999,
            -99999
	  };
	  planepoints.push_back(p);
	}
      }
      for (const auto& i : planepoints) {
	if (is_destinationvalid(i)) {
	  points1.push_back(i);
	}
      }
      if (points1.size() > points.size()) {
	points = points1;
      }
      else if (points1.size() < points.size()) {
	nlows++;
      }
      if (nlows == 3) {
	break;
      }
    }
  }

  return points.size();

}
//--Anubhav's Edit(end)------------------------------------


//----------------------------------------------------------
int ScanSequence::CylinderPath(std::vector<std::vector<double> > &points){
//----------------------------------------------------------

  //TODO : test
  //       add optional norm incidence


  // For cylindrical scan
  float PI = 3.14159265;
  int max_size = 0;
  int prev_max_size = -1;
  points.clear();

  /*
     * Cylindrical sequence (for magnetic field scans)
     * Not adjusted for two gantries
     * Not tested again: need to be fixed and debugged
     */
  // Parameter checks
  if(fs.cyl_par.height >= z_max_value){   //max Z is not set in gGantryLimits because position at limit Z = 0!
    cm_msg(MERROR,"CylinderPath","Error: Height outside of limits.");
    return 0;
  }
  float min = 0.;

  // Find min distance to edges (different for both gantries)
  if(gGantryLimits[5] < gGantryLimits[6])
    min = gGantryLimits[5];
  else
    min = gGantryLimits[6];

  if(fs.cyl_par.radius >= min/2){
    cm_msg(MERROR,"CylinderPath","Radius outside of limits.");
    return 0;
  }
  if(fs.cyl_par.loop_separation < 0.001){
    cm_msg(MERROR,"CylinderPath","Loop separation too small for motors (min resolution = 1 mm).");
    return 0;
  }
  if(fs.cyl_par.layer_thickness < 0.001){
    cm_msg(MERROR,"CylinderPath","Layer thickness too small for motors (min resolution = 1 mm).");
    return 0;
  }
  if(fs.cyl_par.arc_step < 0.001){
    cm_msg(MERROR,"CylinderPath","Arc step size too small for motors (min resolution = 1 mm).");
    return 0;
  }
  cm_msg(MINFO,"CylinderPath","Generating cylindrical path...");
  // Generate layers
  for(int layer = 0;layer <= (trunc(fs.cyl_par.height/fs.cyl_par.layer_thickness));layer++){

    // for efficient vector usage: already resize
    if (point_num == max_size) {
      prev_max_size = max_size;
      max_size =! max_size ? 1 : max_size << 1;
      points.resize(max_size);

      for(int j = prev_max_size;j < max_size; j++){
        points[j].reserve(10);
      }
    }


    // fix X, Y, GoTo layer Z
    points[point_num][0] = 0 + fs.cyl_par.x_center;
    points[point_num][1] = 0 + fs.cyl_par.y_center;
    points[point_num][2] = fs.cyl_par.height - (layer*fs.cyl_par.layer_thickness);

    //Don't change the destination!
    for(int k = 3; k < 10; k++){
      points[point_num][k] = -99999;
    }
    point_num = point_num + 1;

    // Generate loops: for each Z, loop through X and Y in a circle
    for(int loop = 1;loop <= (trunc(fs.cyl_par.radius/fs.cyl_par.loop_separation));loop++){
      for(int waypoint = 0; waypoint <= trunc((2*PI*loop*fs.cyl_par.loop_separation)/fs.cyl_par.arc_step) ; waypoint++){

        // for efficient vector usage: already resize
        if (point_num == max_size) {
          prev_max_size = max_size;
          max_size =! max_size ? 1 : max_size << 1;
          points.resize(max_size);

          for(int j = prev_max_size;j < max_size; j++){
            points[j].reserve(10);
          }
        }


        //copy all points of the first point in the layer to this point, but change X and Y
        //for(j = 0;j < 3;j++){
        //points[point_num][j] = points[point_num-1][j];
        //}
        //--> just want SAME z
        float theta = (2*PI)*((fs.cyl_par.arc_step*waypoint)/(2*PI*loop*fs.cyl_par.loop_separation));
        points[point_num][0] = (loop*fs.cyl_par.loop_separation)*cos(theta) + fs.cyl_par.x_center;
        points[point_num][1] = (loop*fs.cyl_par.loop_separation)*sin(theta) + fs.cyl_par.y_center;
        points[point_num][2] = points[point_num-1][2];

        //Don't change the destination!
        for(int k = 3; k < 10; k++){
          points[point_num][k] = -99999;
        }

        point_num = point_num + 1;

      }
    }
  }

  // for efficient vector usage: already resize
  if (point_num == max_size) {
    prev_max_size = max_size;
    max_size =! max_size ? 1 : max_size << 1;
    points.resize(max_size);

    for(int j = prev_max_size;j < max_size; j++){
      points[j].reserve(10);
    }
  }

  // Add end point: back to their corner!
  for(int j = 0;j < 9; j++){
    points[point_num][j] = gGantryLimits[j];
  }
  point_num = point_num + 1;

  printf("%i - %.4F %.4F %.4F \n",point_num,points[point_num][0],points[point_num][1],points[point_num][2]);
  printf("%i - %.4F %.4F %.4F \n",point_num,points[point_num][5],points[point_num][6],points[point_num][7]);

  return point_num;
}


//----------------------------------------------------------
int ScanSequence::RectangularPath(std::vector<std::vector<double> > &points){
//----------------------------------------------------------

// 1) Use for magnetic field scan (with zero tilt and -90 rotation or so), but using two gantries
// 2) Use for vertical acceptance (with -90 tilt and a chosen rotation)
//    = > fixed tilt and rotation which can give a cross-sectional view of the PMT from one tilt angle
// 3) Use for acceptance at each point of the rectangular and get norm incidence for rotation and tilt
// 4) NOT for angular acceptance, as this is as function of fixed point on PMT. Use PMTSurfacePath

  printf("Setting up a rectangular path\n");

// For linear/plane/rectangular prism scan
  int x_layer;
  int x_increment = -1;
  int x_limit = -1;

  points.clear();

  /*
   * Rectangular sequence for magnetic field scans
   * Works for two gantries and many layers.
   * Well tested!
   */

  // Parameter checks
  if((fs.rect_par.init_pos_z + fs.rect_par.prism_height_z) > z_max_value) {
    cm_msg(MERROR,"RectangularPath","Height (+ initial z position) outside of limits.");
    return 0;
  }

  if((fs.rect_par.init_pos_z ) < fabs(gGantryLimits[2])){
    cm_msg(MERROR,"RectangularPath","Initial z position outside of limits.");
    return 0;
  }

  if((fs.rect_par.init_pos_x + fs.rect_par.prism_length_x) > fabs(gGantryLimits[5])){
    cm_msg(MERROR,"RectangularPath","Length (+ initial x position) outside of limits.");
    return 0;
  }
  if((fs.rect_par.init_pos_x) < fabs(gGantryLimits[0])){
    cm_msg(MERROR,"RectangularPath","Initial x position outside of limits.");
    return 0;
  }
  if((fs.rect_par.init_pos_y + fs.rect_par.prism_width_y) > (gGantryLimits[6])){
    cm_msg(MERROR,"RectangularPath","Width (+ initial y position) outside of limits.");
    return 0;
  }
  if((fs.rect_par.init_pos_y) < (gGantryLimits[1])){
    cm_msg(MERROR,"RectangularPath","Initial y position outside of limits.");
    return 0;
  }
  if(fs.rect_par.z_step < 0.001){
    cm_msg(MERROR,"RectangularPath","Z step size too small for motors (min resolution = 1 mm).");
    return 0;
  }
  if(fs.rect_par.x_step < 0.001){
    cm_msg(MERROR,"RectangularPath","X step size too small for motors (min resolution = 1 mm).");
    return 0;
  }
  if(fs.rect_par.y_step < 0.001){
    cm_msg(MERROR,"RectangularPath","Y step size too small for motors (min resolution = 1 mm).");
    return 0;
  }
cm_msg(MINFO,"RectangularPath","Generating linear/plane/rectangular prism path..."); 
  // Generate path

  // 19/Oct/2017 - Kevin Xie - added extra code to rectangular scan paths so that the entire region can be scanned. Previously, false collision messages appeared at the Y-position where the non-scanning gantry switched hemispheres
  bool crossedHemisphere = false;
  double offset = 0.005; // 5mm offset added to corners to avoid limit switch errors
  for(int z_layer = 0;z_layer<=(trunc(fs.rect_par.prism_height_z/fs.rect_par.z_step));z_layer++){

    //scan for gantry 0
    if(fs.rect_par.which_gantry == GANTRY0 || fs.rect_par.which_gantry == BOTH){
      for(int y_layer=0;y_layer<=(trunc(fs.rect_par.prism_width_y/fs.rect_par.y_step));y_layer++){

        if((y_layer % 2) == 0){
          x_layer = 0;
          x_increment = 1;
          x_limit = trunc(fs.rect_par.prism_length_x/fs.rect_par.x_step)+1;
        }
        else{
          x_layer = trunc(fs.rect_par.prism_length_x/fs.rect_par.x_step);
          x_increment = -1;
          x_limit = -1;
        }

        for(; x_layer!=x_limit ;  x_layer=x_layer+x_increment){

          std::vector<double> single_point (10);

          single_point[0] = fs.rect_par.init_pos_x + (x_layer*fs.rect_par.x_step);
          single_point[1] = fs.rect_par.init_pos_y + (y_layer*fs.rect_par.y_step);
          single_point[2] = fs.rect_par.init_pos_z + (z_layer*fs.rect_par.z_step);
          if(fs.rect_par.subtype == DEFAULT ||
             fs.rect_par.subtype == MONITOR_REF){
            single_point[3] = fs.rect_par.init_rotation;
            single_point[4] = fs.rect_par.init_tilt;
          } else if(fs.rect_par.subtype == NORM_INCIDENCE){
            CalculateNormIncidence(single_point, 0.359, 0.326, 0.05, 0, 0, GANTRY0);//Rika (25Apr2017): estimated new PMT centre (0.341, 0.328) // Kevin's new estimate (0.359, 0.326)
            //Beamlength changed to 0.05m
            for (int trial = 0; trial < 10; trial++)
            {
              char buffer[32];
              snprintf(buffer, sizeof(buffer), "%g", single_point[trial]);
              //cm_msg(MINFO,"Testing", buffer);  //TF: too slow to dump, only for debugging purposes for short sequences
            }
          }

          if(fs.rect_par.which_gantry == BOTH){
            single_point[7] = fs.rect_par.init_pos_z + (z_layer*fs.rect_par.z_step);
            single_point[8] = fs.rect_par.init_rotation;
            single_point[9] = fs.rect_par.init_tilt;
          }

          // 30.Nov.2017 do same hemisphere check as gantry 1
          if (fs.rect_par.subtype == DEFAULT) {
            single_point[7] = offset; // G1 z-axis set to 5mm

            if (points.size() == 0 && single_point[1] > 0.32) { // if scan starts at y > 0.32, then automatically set the flag
              cm_msg(MINFO, "RectangularPath", "Gantry 0 started at y > 0.32, so G1 moves immediately. Point number is %u", points.size()); // DEBUG
              crossedHemisphere = true;
            }

            if(single_point[1] > 0.32 && crossedHemisphere){
              single_point[5] = gGantryLimits[5] - offset;
              single_point[6] = 0. + offset;
            }
            else if (!crossedHemisphere && single_point[1] > 0.32 && single_point[0] < 0.32) {
              single_point[5] = gGantryLimits[5] - offset;
              single_point[6] = 0. + offset;
              crossedHemisphere = true;
              cm_msg(MINFO, "RectangularPath", "Gantry will cross hemisphere at (%f, %f).", single_point[0], single_point[1]);
              cm_msg(MINFO, "RectangularPath", "The point number will be %u", points.size());

            } else {
              single_point[5] = gGantryLimits[5] - offset;
              single_point[6] = gGantryLimits[6] - offset;
            }

          } else { // if not doing a default scan (i.e: normal incidence) then more basic code is used because the crossedHemisphere check is only applicable on linear scan paths
            if(single_point[1] > 0.32){
              single_point[5] = gGantryLimits[5] - offset;
              single_point[6] = 0. + offset;
            }
            else{
              single_point[5] = gGantryLimits[5] - offset;
              single_point[6] = gGantryLimits[6] - offset;
            }
          }
          points.push_back(single_point);

          // If wanted, monitor stability at ref point:
          if(fs.rect_par.subtype == MONITOR_REF){
            if((points.size()-1)%fs.rect_par.monitor_rect_par.nth_point == 0){

              //printf("test: %i %i",point_num,points.size());
              point_num = point_num + 1;
              std::vector<double> new_single_point(10);

              new_single_point[0] = fs.rect_par.monitor_rect_par.monitor_pos_x0;
              new_single_point[1] = fs.rect_par.monitor_rect_par.monitor_pos_y0;
              new_single_point[2] = fs.rect_par.monitor_rect_par.monitor_pos_z0;
              for(int u = 3; u < 10; u++){
                new_single_point[u] = points[point_num-1][u];
              }
              points.push_back(new_single_point);
              printf("points size %i test %lf\n",(int)points.size(),points[point_num][0]);
            }
          }

          point_num = point_num + 1;

        }// end loop over x_layer for gantry0
      }//end loop over y_layer for gantry0
    }

    // scan for gantry 1 (TODO: too much duplicated code, can be done more efficient!)
    if(fs.rect_par.which_gantry == GANTRY1 || fs.rect_par.which_gantry == BOTH){



      for(int y_layer = 0;y_layer<=(trunc(fs.rect_par.prism_width_y/fs.rect_par.y_step));y_layer++){

        if((y_layer % 2) == 0){
          x_layer = 0;
          x_increment = 1;
          x_limit = trunc(fs.rect_par.prism_length_x/fs.rect_par.x_step)+1;
        }
        else{
          x_layer = trunc(fs.rect_par.prism_length_x/fs.rect_par.x_step);
          x_increment = -1;
          x_limit = -1;
        }

        for(; x_layer!=x_limit ;  x_layer=x_layer+x_increment){

          std::vector<double> single_point (10);

          single_point[5] = fs.rect_par.init_pos_x + (x_layer*fs.rect_par.x_step);
          single_point[6] = fs.rect_par.init_pos_y + (y_layer*fs.rect_par.y_step);
          single_point[7] = fs.rect_par.init_pos_z + (z_layer*fs.rect_par.z_step);

          if(fs.rect_par.subtype == DEFAULT || fs.rect_par.subtype == MONITOR_REF){
            single_point[8] = fs.rect_par.init_rotation;
            single_point[9] = fs.rect_par.init_tilt;
          } else if(fs.rect_par.subtype == NORM_INCIDENCE){
            CalculateNormIncidence(single_point, 0.359, 0.326, 0.05, 0, 0, GANTRY1);
            for (int trial = 0; trial < 10; trial++)
            {
              char buffer[32];
              snprintf(buffer, sizeof(buffer), "%g", single_point[trial]);
              //cm_msg(MINFO,"Testing", buffer); //TF: too slow to dump, only for debugging purposes for short sequences
            }
          }
          if(fs.rect_par.which_gantry == BOTH){
            single_point[2] =  fs.rect_par.init_pos_z + (z_layer*fs.rect_par.z_step);
            single_point[3] = fs.rect_par.init_rotation;
            single_point[4] = fs.rect_par.init_tilt;
          }

          // 19/Oct/2017 // doing crossedHemisphere check so that gantry collision check does not interfere with scan region

          if (fs.rect_par.subtype == DEFAULT) {

            single_point[2] = 0. + offset; // adding Z axis offset now as well Apr 11

            if (points.size() == 0 && single_point[6] > 0.34) { // if scan starts at y > 0.34, then automatically set the flag
              cm_msg(MINFO, "RectangularPath", "Gantry 1 started at y > 0.34, so G0 does not need to move. Point number is %u", points.size()); // DEBUG
              crossedHemisphere = true;
            }

            if(single_point[6] > 0.34 && crossedHemisphere){

              single_point[0] = 0. + offset;
              single_point[1] = 0. + offset;
            }
            else if (!crossedHemisphere && single_point[6] > 0.34 && single_point[5] > 0.34) {
              single_point[0] = 0. + offset;
              single_point[1] = 0. + offset;
              crossedHemisphere = true;
              cm_msg(MINFO, "RectangularPath", "Gantry will cross hemisphere at (%f, %f).", single_point[5], single_point[6]); // DEBUG
              cm_msg(MINFO, "RectangularPath", "The point number will be %u", points.size()); // DEBUG

            } else {
              single_point[0] = 0. + offset;
              single_point[1] = gGantryLimits[6] - offset;
            }
          } else {
            if(single_point[6] > 0.34){

              single_point[0] = 0. + offset;
              single_point[1] = 0. + offset;
            }
            else{
              single_point[0] = 0. + offset;
              single_point[1] = gGantryLimits[6] - offset;
            }
          }
          points.push_back(single_point);

          // If wanted, monitor stability at ref point:
          if(fs.rect_par.subtype == MONITOR_REF){
            if(point_num%fs.rect_par.monitor_rect_par.nth_point == 0){
              point_num = point_num + 1;

              std::vector<double> new_single_point(10);
              new_single_point[5] = fs.rect_par.monitor_rect_par.monitor_pos_x1;
              new_single_point[6] = fs.rect_par.monitor_rect_par.monitor_pos_y1;
              new_single_point[7] = fs.rect_par.monitor_rect_par.monitor_pos_z1;
              for(int u = 0; u < 5; u++){
                single_point[u] = points[point_num-1][u];
              }
              for(int u = 8; u < 10; u++){
                new_single_point[u] = points[point_num-1][u];
              }
              points.push_back(new_single_point);
            }
          }

          point_num = point_num + 1;
        }
      }//end loop over Y layer for second gantry
    }//end if
  }//end loop over Z layer for both gantries


  // TODO: uncomment:
  /* Move Gantry1 to PMT position for long term scan in history.
  for(j=0;j<5;j++){
    single_point[j] = 0.0;
  }
  //gantry 1: //only for FIELD SCAN with extension arm
  single_point[5] = 0.4;
  single_point[6] = 0.41;
  single_point[7] = 0.3;
  single_point[8] = -90;
  single_point[9] = 0.0;
  */
  // Add end point: back to their corner!
  /* for(j=0;j<5;j++){
     single_point[j] = 0.0;
  }
  //gantry 1:
  single_point[5] = gGantryLimits[5];
  single_point[6] = gGantryLimits[6];
  single_point[7] = 0.0;
  single_point[8] = -90;
  single_point[9] = 0.0;
  points.push_back(single_point);
  */

  return point_num;
}

//----------------------------------------------------------
int ScanSequence::DemoPath(std::vector<std::vector<double> > &points){
//----------------------------------------------------------

// Demo sequence for HK Workshop

  /* for HK demo
  float scan_init_y=0.;
  float scan_step_y=0.05;
  float scan_width_y=0.666;
  */

  points.clear();
  // scan for gantry 0
  // for(y_layer=0;y_layer<=(trunc(scan_width_y/scan_step_y));y_layer++){
  for(int rot_i = 0;rot_i <= 200;rot_i++){
    float rotation = -100 + rot_i;

    std::vector<double> single_point(10);

    single_point[0] = 0.1;
    //	single_point[1] = scan_init_y + (y_layer*scan_step_y);
    single_point[1] = 0.1;
    single_point[2] = 0.25;
    single_point[3] = rotation;
    single_point[4] = 0;
    single_point[5] = 0.57;
    single_point[6] = 0.39;
    single_point[7] = 0.25;
    single_point[8] = rotation;
    single_point[9] = 0;
    points.push_back(single_point);

    point_num = point_num + 1;
  }

  return point_num;
}

//----------------------------------------------------------
int ScanSequence::TankAvoidancePath(std::vector<std::vector<double> > &points){
//---------------------------------------------------------

  points.clear();
  int max_size = 0;
  /*
  * Manual sequences and debug sequences
  */

  // Total number of points we will move to
  point_num =2;
  max_size =  point_num+1;

  // List of points; 
  float sequence[8][10] = {{0.000,0.000,0.000,-90.0,0.00, 0.620,0.621,0.200,0.00,0.00}, // Gantry 1 top-top limit
                           {0.000,0.000,0.000,-90.0,0.00, 0.620,0.622,0.200,0.00,0.00}, // Over limit (0.621-0.622)
                           {0.000,0.000,0.000,-90.0,0.00, 0.013,0.660,0.200,-45.0,0.00}, // Gantry 1 bottom-top limit w/ tilt
                           {0.000,0.000,0.000,-90.0,0.00, 0.012,0.660,0.200,-45.0,0.00}, // Over limit (0.013-0.012)
                           {0.037,0.040,0.200,0.0,0.00, 0.650,0.647,0.200,-45.0,0.00}, // Gantry 0 bottom-bottom limit
                           {0.036,0.040,0.200,0.0,0.00, 0.650,0.647,0.200,-45.0,0.00}, // Over limit (0.037-0.036)
                           {0.631,0.000,0.200,-45.0,0.00, 0.650,0.647,0.200,-45.0,0.00}, // Gantry 0 top-bottom limit
                           {0.632,0.000,0.200,-45.0,0.00, 0.650,0.647,0.200,-45.0,0.00}}; // Over limit (0.631-0.632)

  points.reserve(point_num+1);
  for(int j = 0; j < (point_num+1); j++){
    points[j].reserve(10);
  }

  for(int p = 0; p < max_size; p++){
    for(int l = 0; l < 10; l++){
      points[p][l] = sequence[p][l];
    }
  }
  return point_num;
}


//----------------------------------------------------------
int ScanSequence::ManualPath(std::vector<std::vector<double> > &points){
//----------------------------------------------------------
  cm_msg(MDEBUG, "ManualPath", "Loading manual path...");
  point_num = DEMO_PATH_LENGTH - 1;
  points.clear();

  points.resize(DEMO_PATH_LENGTH);

  for (int i = 0; i < DEMO_PATH_LENGTH; i++) {
    points[i].resize(10);
    for (int j = 0; j < 10; j++)
      points[i][j] = DEMO_PATH[i][j];
  }

  cm_msg(MDEBUG, "ManualPath", "Manual path loaded. There are %i points.", DEMO_PATH_LENGTH);

  return point_num;
}


//----------------------------------------------------------
int ScanSequence::AlignmentPath(std::vector<std::vector<double> > &points){
//----------------------------------------------------------


  points.clear();
  int max_size = 0;

  // Parameter checks
  if(fs.align_par.gantry_laser > 1){
    cm_msg(MERROR,"AlignmentPath","Choose Gantry 0 or Gantry 1 as gantry with the emitting source!");
    return 0;
  }

  // Create some initial space
  max_size = 1;
  points.resize(max_size);
  for(int j = 0;j < max_size; j++){
    points[j].reserve(10);
  }


  //STEP 1: set both gantries at initial position 
  // NOTE : Do I need init_pos as scan setting? Yes, because the scan sequence will call feMove where it defines
  //        its destinations.

  std::vector<double> single_point(10);
  for(int l = 0; l < 10; l++)
    single_point[l] = fs.align_par.init_pos[l];

  points.push_back(single_point);
  point_num = 1;
  //STEP 2: go to corner of grid and start grid scan
  // Start with a simple situation: both tilts horizontal, otherwise do some trigonometry
  int gan_i = 0;
  if(fs.align_par.gantry_laser == GANTRY1)
    gan_i = 1;

  int i_len = 0;
  int i_width = 0;
  int len_steps = trunc(fs.align_par.grid_length/fs.align_par.step_length);
  int width_steps = trunc(fs.align_par.grid_width/fs.align_par.step_width);
  int direction = 1;
  while(i_width <= width_steps){
    while(i_len <= len_steps){

      std::vector<double> single_point(10);

      //figure out whether to move in x or in y
      int x_or_y = 0;
      if(abs(fs.align_par.init_pos[0] - fs.align_par.init_pos[5]) > 0.1){
        printf("From initial position assumed that gantries are opposite at fixed Y");
        x_or_y = 1;
      }

      single_point[x_or_y + gan_i*5] = (fs.align_par.init_pos[x_or_y + gan_i*5] - direction * fs.align_par.grid_length/2.) + direction*i_len*fs.align_par.step_length;

      single_point[2 + gan_i*5] = (fs.align_par.init_pos[2 + gan_i*5] - fs.align_par.grid_width/2.) + i_width*fs.align_par.step_width;
      points.push_back(single_point);
      i_len++;
      point_num++;

    }
    direction = -1* direction;
    i_width++;
  }


  //STEP 3: back to initial position, but go to corner in solid angle grid, start scan
  single_point.clear();
  single_point.reserve(10);

  for(int l = 0; l < 10; l++)
    single_point[l] = fs.align_par.init_pos[l];

  points.push_back(single_point);
  point_num++;

  // Second move in solid angle space, delta theta vs delta tilt
  // TODO: cleanup below with less copy-pasting from above code..., maybe 30 lines of code can be much shorter...
  int i_theta = 0;
  int i_phi = 0;
  int theta_steps = trunc(fs.align_par.delta_rotation/fs.align_par.step_rotation);
  int phi_steps = trunc(fs.align_par.delta_tilt/fs.align_par.step_tilt);
  direction = 1;
  while(i_phi <= phi_steps){
    while(i_theta <= theta_steps){

      std::vector<double> new_single_point(10);

      new_single_point[3 + gan_i*5] = (fs.align_par.init_pos[3 + gan_i*5] - direction * fs.align_par.delta_rotation/2.) + direction*i_theta*fs.align_par.step_rotation;

      new_single_point[4 + gan_i*5] = (fs.align_par.init_pos[4 + gan_i*5] - fs.align_par.delta_tilt/2.) + i_phi*fs.align_par.step_tilt;

      points.push_back(new_single_point);
      i_theta++;
      point_num++;

    }
    direction = -1* direction;
    i_phi++;
  }
  return point_num;
}

//----------------------------------------------------------
int ScanSequence::PassByPath(std::vector<std::vector<double> > &points){
//----------------------------------------------------------

  points.clear();

  //check Parameters
  if(fs.pass_by_par.gantry_laser > 1){
    cm_msg(MERROR,"PassByPath","Choose Gantry 0 or Gantry 1 as gantry with the emitting source!");
    return 0;
  }

  //STEP 1: set initial position of laser box

  int gan_i = 0;
  int ij = 0;
  if(fs.align_par.gantry_laser == GANTRY1)
    gan_i = 1;

  while(ij < 5){
    /* TODO!!
       points[0][ij + gan_i*5] = fs.pass_by_par.init_pos[ij]; 
       points[0][ij + (!gan_i)*5]   = gGantryLimits[ij+ (!gan_i)*5];
    */
    ij++;
  }
  point_num = 1;

  // TODO !!!

  // STEP 2: move receiver PMT N times between limit and laser position (NOTE : correct for 2 inch distance between
  //         PMT and laser if needed)



  // STEP 3: move N times between opposite limit and requested position


  return point_num;

}


//----------------------------------------------------------
int ScanSequence::PMTSurfacePath(std::vector<std::vector<double> > &points){
//----------------------------------------------------------

  /* This is a sequence on the surface of the PMT: an azimuthal symmetric object, which
     therefore requires cylindrical coordinates. If we choose the xyz orientation the
     same as the gantry system, then going from PMT cylindrical coordinates to Gantry
     coordinates is a simple shift */

  float PI = 3.14159265;

  // PMT properties : currently hard coded here (and also in CalcNormInc (should get that from simple PMT class which is hidden)
  double max_height;  //relative to center of lowest sphere, ie. relative to the rim/band

  //start with simple,single sphere, later : local sphere (ToDo: need errors)
  double x_c = 0.348; //from run 2009
  double y_c = 0.366; //from run 2009 //measured
  double z_c = 0.3;
  double radius = 0.323; //from acrylic cover model
  // double beamLength = 0.1; //desired beam length from gantry laser aperture to Acrylic Cover

  //dummy vector to hold values returned from calculated normal incidence function
  std::vector<double> single_point;

  points.clear();

  // Parameter checks:

  if((fs.pmt_surf_par.delta_phi) > 0 && (fs.pmt_surf_par.delta_h) > 0){
    cm_msg(MERROR,"PMTSurfacePath","Looping in both height and polar angle not yet implemented");
    return 0;
  }

  cm_msg(MINFO,"PMTSurfacePath","Generating path on the PMT surface...");

  // Generate path: 

  if((fs.pmt_surf_par.delta_phi) > 0){
    // 1) loop over polar_angle at fixed height
    int n_points = (int)360./(fs.pmt_surf_par.delta_phi);
    for(int i = 0; i < n_points; i++){
      double phi = fs.pmt_surf_par.init_polar + i*fs.pmt_surf_par.delta_phi; //same definition as rotation angle

      // Now I know the coordinates on the surface, convert to gantry_co system:
      std::vector<double> point_on_surf(5);
      point_on_surf[0] = radius*cos(phi*PI/180.) - x_c;
      point_on_surf[1] = radius*sin(phi*PI/180.) - y_c;
      point_on_surf[2] = -fs.pmt_surf_par.init_height - z_c;

      // next the normal incidence: 
      // TODO : THIS IS UNNECESSARY as in the PMT coordinate system we just need to convert to spherical coordinates! then we can directly use those angles for rotation and tilt
      //CalculateNormIncidence(point_on_surf,GANTRY0);


      //LATER TODO: Define loop of angles relative to this normal for angular acceptance


      //Now I have the rotation and tilt, just need to calculate the x and y of the gantry at fixed z

      // And fill the sequence

      //TODO: improve this by going directly from cylindrical to spherical? Need gantry position though

    }
  } else if((fs.pmt_surf_par.delta_h) > 0){
    // 2) loop over height at fixed polar_angle (up to minus sign)
    // int n_points = (int)max_height/(fs.pmt_surf_par.delta_h);
    /*
      for(int i = 0; i < n_points; i++){
      
      
      }*/
  } else {
    // 3) fixed point: different angles	
    return 0;
  }

  return 0;

}

/**
//----------------------------------------------------------
int ScanSequence::FixedPointPath(std::vector<std::vector<double> > &points){
//----------------------------------------------------------


    /*
     * Given a coordinate on the PMT surface,
     * Do Theta-Phi scan (and later add reflectivity)
     */
/**
    // Parameters for FixedPointPath (for testing code: later will be taken from ODB)
    double x_pt = 0.374;
    double y_pt = 0.348;
    double startTheta;
    double startPhi;
    double stepTheta;
    double stepPhi;
    double totalTheta;
    double totalPhi;

    double beamlength = 0.1;
    int gantryScanning = 0;
    int gantryNotScanning = 1;

    // Variables that should be specified in the ODB
    // Centre point of PMT:
    double x_c = 0.374;
    double y_c = 0.348;

  points.clear();
  int max_size = 0;
  std::vector<double> single_point (10);

  double gSwitchPoint = 0.32; // y-position of scanning gantry at which
                                      // the non-scanning gantry should move to the other side.

  // Parameter checks
  if(fs.align_par.gantry_laser > 1){
    cm_msg(MERROR,"AlignmentPath","Choose Gantry 0 or Gantry 1 as gantry with the emitting source!");
    return 0;
  }
  
  // Create some initial space
  max_size = 1;
  points.resize(max_size);
  for(int j = 0;j < max_size; j++){
    points[j].reserve(10);
  }

  for(int theta = startTheta; theta <= startTheta + totalTheta; theta += stepTheta){

      for(int phi = startPhi; phi <= startPhi + totalPhi; phi += stepPhi){

	  // Prepare single_point to be passed to CalculateNormIncidence
	  single_point[0 + 5*gantryScanning] = x_pt;
	  single_point[1 + 5*gantryScanning] = y_pt;

	  // Get gantry position for given x,y point and theta, phi angles
	  CalculateNormIncidence( single_point, x_c, y_c, beamlength, phi, theta, gantryScanning );
	  
	  // Make sure that non-scanning gantry moves out of the way
	  if( single_point[1 + 5*gantryScanning] > gSwitchPoint ){
	      single_point[0 + 5*gantryNotScanning] = gGantryLimits[];

      }
  }

  return point_num;
}
**/

//----------------------------------------------------------
void ScanSequence::ReturnToBase(std::vector<std::vector<double> > &points){
//----------------------------------------------------------
  std::vector<double> single_point(10);

  for(int j = 0;j < 9; j++){
    single_point[j] = gGantryLimits[j];
  }
  points.push_back(single_point);
  return;

}


//----------------------------------------------------------
void ScanSequence::CalculateNormIncidence(std::vector<double> &point, double center_x, double center_y, double beam, double incidence, double azimuth, gantry_t gantry)
{
//----------------------------------------------------------

/** 
 *  Parameters:
 *      point    - vector of size 10 where the gantry position will eventually be stored;
 *                 when passed to the function, the vector should contain:
 *                   point[0] = x coordinate of point on PMT (if gantry==GANTRY0)
 *                   point[1] = y coordinate of point on PMT (if gantry==GANTRY0)
 *                   point[5] = x coordinate of point on PMT (if gantry==GANTRY1)
 *                   point[6] = y coordinate of point on PMT (if gantry==GANTRY1)
 *      center_x, center_y
 *               - coordinates of PMT center
 *      beam     - desired beamlength of laser
 *      incidence- desired angle of incidence of laser at the specified point on the PMT;
 *      azimuth  - desired azimuth angle
 *      gantry   - choice of gantry to be used for the scan
 **/


/* General idea: - Gradient of spherical surface function evaluated at a point on the sphere gives its normal vector.
                 - for a circle, sphere or sum of spheres (good model for PMT with azimuthal symm): 
		   => Normal is ALWAYS line going through center (radius is perpendicular with tangential plane!!)
 */

  const double PI  = 3.141592653589793238463;

  double radius = 0.323;
  double pmtHeight = 0.390;

  double x, y, z, gamma, alpha, x_origin, y_origin, z_origin,
      x_inc, y_inc, z_inc, x_inc_temp, /*y_inc_temp,*/
      tilt_angle, rot_angle, offset1_g0, offset2_g0,
      offset3_g0, offset1_g1, offset2_g1, offset3_g1,
      g0_o_mag, g1_o_mag, g0_final_offset_x, g0_final_offset_y,
      g0_final_offset_z, g1_final_offset_x, g1_final_offset_y, g1_final_offset_z,
      output_rot_angle;

  //Offsets for Gantry 0
  offset1_g0 = 0.115; //TODO: determine accuracy
  offset2_g0 = 0.042; //- 0.003*beam*10; //0.042 +- 0.002 correction factor: -0.003*beam*10 added Feb24, 2017 (4pm)-> edited to -0.003*beam*10 Mar6,2017
  offset3_g0 = -0.0365; //0.037 +/- 0.002 Mar6,2017
  g0_o_mag = sqrt(offset1_g0*offset1_g0 + offset2_g0*offset2_g0 + offset3_g0*offset3_g0);

  //Offsets for Gantry 1
  offset1_g1 = 0.115;
  offset2_g1 = 0.052; //- 0.002*beam*10; //correction factor: -0.002*beam*10 added Mar9,2017
  offset3_g1 = -0.0395;
  g1_o_mag = sqrt(offset1_g1*offset1_g1 + offset2_g1*offset2_g1 + offset3_g1*offset3_g1);



  //Converts gantry position in absolute coordinates to relative coordinates
  if(gantry==GANTRY0){
    x = point[0] - center_x;
    y = point[1] - center_y;
  } else{
    x = point[5] - center_x;
    y = point[6] - center_y;
  }

  if(sqrt(x*x + y*y) < radius){
    //Calculates z position on PMT surface
    z = radius*cos(asin(sqrt(x*x + y*y)/radius));

    //Converts angular arguments to radians for use with trig functions
    incidence = PI*incidence/180;
    azimuth = PI*azimuth/180;

    //Calculates spherical coordinate angles
    gamma = atan2(y,x);
    alpha = asin(sqrt(x*x + y*y)/radius);

    //Defines point that will be examined at different angles
    x_origin = x + center_x;
    y_origin = y + center_y;
    z_origin = z;

    //Conversion of spherical coordinates to cartesian
    x_inc = beam*sin(incidence)*cos(azimuth);
    y_inc = beam*sin(incidence)*sin(azimuth);
    z_inc = beam*cos(incidence);

    //Rotation about y-axis of PMT
    x_inc_temp = x_inc;
    x_inc = cos(alpha)*x_inc + sin(alpha)*z_inc;
    z_inc = (-1)*sin(alpha)*x_inc_temp +cos(alpha)*z_inc;

    //Rotation about x-axis of PMT
    x_inc_temp = x_inc;
    x_inc = cos(gamma)*x_inc - sin(gamma)*y_inc;
    y_inc = sin(gamma)*x_inc_temp + cos(gamma)*y_inc;

    //Superposition of coordinates to achieve point source positioning
    x = x_origin + x_inc;
    y = y_origin + y_inc;
    z = z_origin + z_inc;

    //Calculates rotation and tilt angle that describe the beam from the gantry
    rot_angle = atan2(y_inc, x_inc); //Range for atan2 is (-PI,+PI) since it takes into account sign of arguments to determine quadrant
    tilt_angle = atan2(z_inc, sqrt(x_inc*x_inc + y_inc*y_inc)); //tilt_angle comes out positive

    //Converts beam rotation about point of laser incidence (x_origin, y_origin) to rotation angle of Gantry 0
    if(rot_angle < 0)
    {
      rot_angle = rot_angle + PI;
    }
    else
    {
      rot_angle = rot_angle - PI;
    }
  }
  else{ //The point (x,y) specified is not on the PMT surface, so do a vertical scan
    if(gantry==GANTRY0){
      x = point[0];
      y = point[1];
      z = pmtHeight + radius - offset1_g0; //Distance away from PMT cover centre of curvature - optical box height
    }
    else{
      x = point[5];
      y = point[6];
      z = pmtHeight + radius - offset1_g1; //Distance away from PMT cover centre of curvature
    }
    rot_angle = 0.0;
    tilt_angle = 90.0*PI/180;
  }
  //Corrections for gantry 1 and gantry 0 optical boxes
  //Calculations for placement of point source
  g0_final_offset_x = offset1_g0*cos(rot_angle)*cos(tilt_angle) - offset2_g0*sin(rot_angle) + offset3_g0*cos(rot_angle)*sin(tilt_angle);
  g0_final_offset_y = offset1_g0*sin(rot_angle)*cos(tilt_angle) + offset2_g0*cos(rot_angle) + offset3_g0*sin(rot_angle)*sin(tilt_angle);
  g0_final_offset_z = (-1)*offset1_g0*sin(tilt_angle) + offset3_g0*cos(tilt_angle);

  g1_final_offset_x = offset1_g1*cos(rot_angle)*cos(tilt_angle) - offset2_g1*sin(rot_angle) + offset3_g1*cos(rot_angle)*sin(tilt_angle);
  g1_final_offset_y = offset1_g1*sin(rot_angle)*cos(tilt_angle) + offset2_g1*cos(rot_angle) + offset3_g1*sin(rot_angle)*sin(tilt_angle);
  g1_final_offset_z = (-1)*offset1_g1*sin(tilt_angle) + offset3_g1*cos(tilt_angle);

  if (gantry==GANTRY0)
  {
    x = x - g0_final_offset_x;
    y = y - g0_final_offset_y;
    z = z - g0_final_offset_z;
    output_rot_angle = rot_angle;
  }
  else
  {
    x = x - g1_final_offset_x;
    y = y - g1_final_offset_y;
    z = z - g1_final_offset_z;

    if( rot_angle > 0.0) //Updated from (rot_angle > 90*PI/180) in order to fix problem of illegal angles being produced - Mar 1, 2017
    {
      output_rot_angle = rot_angle - PI;
    }
    else
    {
      output_rot_angle = rot_angle + PI;
    }
  }

  //Final conversions to gantry frame
  z =  pmtHeight + radius - z; //0.712 == z position of centre of PMT cover in PTF coordinates - updated 24Apr2017
  tilt_angle = tilt_angle*(-1);

  //sets up return array

  if(gantry==GANTRY0)
  {

    point[0] = x;
    point[1] = y;
    point[2] = z;
    point[3] = output_rot_angle*180/PI;
    point[4] = tilt_angle*180/PI;

    point[5] = 0.647;
    point[6] = 0;
    point[7] = 0;
    point[8] = -100;
    point[9] = 0;

  }
  else
  {

    point[0] = 0;
    point[1] = 0;
    point[2] = 0;
    point[3] = -100;
    point[4] = 0;

    point[5] = x;
    point[6] = y;
    point[7] = z;
    point[8] = output_rot_angle*180/PI;
    point[9] = tilt_angle*180/PI;

  }

  return;

}

//---------------------------------------------------------
void TransformPMTsurfaceToGantry(void){
//--------------------------------------------------------  
  return;

}
//--------------------------------------------------------  
void TransformGantryToPMTsurface(void){
//--------------------------------------------------------  
  return;
}
