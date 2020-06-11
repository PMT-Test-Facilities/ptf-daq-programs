#include "ScanSequence.hxx"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>


//globals from feScan, which I need here (are global anyway, otherwise pass by ref Init?)

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
  z_max_value = 0.54;
  return;
}


//----------------------------------------------------------
int ScanSequence::GeneratePath(std::vector<std::vector<double> > &points_in){
//----------------------------------------------------------
  
  
  // Generate path based on scan parameters:
  point_num = 0;
  int non_zero_points = 0;
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
  case ALIGNMENT:
    non_zero_points = AlignmentPath(points_in);
    break;
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
	      CalculateNormIncidence(single_point,GANTRY0);
	    }
	    
	      
	    // TODO: STRANGE, makes MORE sense with the IF condition
	    if(fs.rect_par.which_gantry == BOTH)
	      single_point[7] = fs.rect_par.init_pos_z + (z_layer*fs.rect_par.z_step);  
	    single_point[8] = fs.rect_par.init_rotation;
	    single_point[9] = fs.rect_par.init_tilt;
	    
	    if(single_point[1] > 0.3){
	      
	      single_point[5] = gGantryLimits[5];
	      single_point[6] = 0.;
	      
	    }
	    else{
	      single_point[5] = gGantryLimits[5];
	      single_point[6] = gGantryLimits[6];
	    }
	    
	    // DONE, now append
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
	    
	    // TODO: STRANGE
	    if(fs.rect_par.which_gantry == BOTH)
	      single_point[2] =  fs.rect_par.init_pos_z + (z_layer*fs.rect_par.z_step); 
	    single_point[3] = fs.rect_par.init_rotation;
	    single_point[4] = fs.rect_par.init_tilt;
	    single_point[5] = fs.rect_par.init_pos_x + (x_layer*fs.rect_par.x_step);
	    single_point[6] = fs.rect_par.init_pos_y + (y_layer*fs.rect_par.y_step);
	    single_point[7] = fs.rect_par.init_pos_z + (z_layer*fs.rect_par.z_step);
	    single_point[8] = fs.rect_par.init_rotation;
	    single_point[9] = fs.rect_par.init_tilt;
	    
	    if(single_point[6] > 0.3){
	      
	      single_point[0] = 0.;
	      single_point[1] = 0.;
	    }
	    else{
	      single_point[0] = 0.;
	      single_point[1] = gGantryLimits[6];
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
int ScanSequence::ManualPath(std::vector<std::vector<double> > &points){
//----------------------------------------------------------
  
  points.clear();
  int max_size = 0;
  
  /*
   * Manual sequences and debug sequences
   */

  
  // Total number of points we will move to
  //    point_num =284;//13;
  point_num =2;
  max_size =  point_num+1;
  
  // List of points; little sequence around the 'PMT'
  // sequence for debugging gantry 1
  float sequence[3][10] = {{0.,0.,0.,-90,0.,0.647,0.61,0.,-90.,0.},
			   {0.,0.,0.,-90,0.,0.37,0.31,0.,-90.,0.},
			   {0.,0.,0.,-90,0.,0.47,0.31,0.,-90.,0.}};
  
  
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
  double x_c = 0.145; //from FIT to vertical run 835
  double y_c = 0.311; //from FIT to vertical run 835
  double z_c = 0.; //measured
  double radius = 0.0914; //from FIT to vertical run 835



  // Parameter checks:
  
  if((fs.pmt_surf_par.delta_phi) > 0 && (fs.pmt_surf_par.delta_h) > 0){
    cm_msg(MERROR,"PMTSurfacePath","Looping in both height and polar angle not yet implemented");
    return 0;
  }
  
  points.clear();
  
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
      CalculateNormIncidence(point_on_surf,GANTRY0);


      //LATER TODO: Define loop of angles relative to this normal for angular acceptance
      

      //Now I have the rotation and tilt, just need to calculate the x and y of the gantry at fixed z
      
      
      
      
      
      
      // And fill the sequence
    
      //TODO: improve this by going directly from cylindrical to spherical? Need gantry position though

    }
  } else if((fs.pmt_surf_par.delta_h) > 0){
    // 2) loop over height at fixed polar_angle (up to minus sign)
    int n_points = (int)max_height/(fs.pmt_surf_par.delta_h);
    /*
      for(int i = 0; i < n_points; i++){
      
      
      }*/
  } else {
    // 3) fixed point: different angles	
    return 0;
  }
  
  return 0;
  
}

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
  void ScanSequence::CalculateNormIncidence(std::vector<double> &point, gantry_t gantry){
//----------------------------------------------------------

/* General idea: - Gradient of spherical surface function evaluated at a point on the sphere gives its normal vector.
                 - for a circle, sphere or sum of spheres (good model for PMT with azimuthal symm): 
		   => Normal is ALWAYS line going through center (radius is perpendicular with tangential plane!!)
 */  


  // Need details of optical boxes
  // Need/Depend on curvature => measured radii, offsets and position of center of spheres in setup/gantry system
  //                          => radii only needed to measure where normal and PMT surface cross, while centers are crucial
  //                          => radii needed for PMTSurfacePath though!!
 
    double x_0 = 1.2; //distance from x at PMT to gantry datum
    double y_0 = 1.2; //distance from y at PMT to gantry datum
    double z_0 = 1.2; //distance from z at PMT to gantry datum
    double dx = 1.75; //	
    double dy = 4.5; //

    //set theta and psi to initial estimate values
    double theta = atan((y_0-y_g)/(x_0-x_g));
    double psi = acos((z_0-z_g)/(sqrt((x_0-x_g)^2+(y_0-y_g)^2+(z_0-z_g)^2))); 

    double theta_cor = atan((y_0-y_g-dx*sin(theta)*(1-cos(psi))-dy*cos(theta))/(x_0-x_g)-dx*cos(theta)*(1-cos(psi))-dy*sin(theta)); //corrected theta
    double psi_cor = acos((z_0-z_g-dx*sin(theta))/(sqrt((x_0-x_g-dx*cos(theta)*(1-cos(psi))-dy*sin(theta))^2+(y_0-y_g-dx*sin(theta)*(1-cos(psi))-dy*cos(theta))^2+(z_0-z_g-dx*(sin(theta)))^2))); //corrected psi
    double theta_dif = abs(theta-theta_cor); //
    double psi_dif = abs(psi-psi_cor); // 
    double theta_newdif = 0;
    double test = 0;
    for(psi_dif > 0.01 && theta_dif > 0.01){
        if(theta_dif > 0.01 && test == 0){
	    theta = theta + 0.01;
	    theta_cor = atan((y_0-y_g-dx*sin(theta)*(1-cos(psi))-dy*cos(theta))/(x_0-x_g)-dx*cos(theta)*(1-cos(psi))-dy*sin(theta)); //corrected theta
	    if(theta_dif > abs(theta_cor-theta)){
	        theta_dif = abs(theta_cor-theta);
	    } else{
	        theta = theta - 0.02;
	        theta_cor = atan((y_0-y_g-dx*sin(theta)*(1-cos(psi))-dy*cos(theta))/(x_0-x_g)-dx*cos(theta)*(1-cos(psi))-dy*sin(theta));
	        if(theta_dif > abs(theta_cor-theta)){
                    theta_dif = abs(theta_cor-theta); 
		    else{
		    test == 1;
		    theta = theta + 0.01;
       	        }
	    }  
        }
        test = 0;
        if(theta_dif > 0.01 && test == 0){
            psi = psi + 0.01;
            theta_cor = atan((y_0-y_g-dx*sin(theta)*(1-cos(psi))-dy*cos(theta))/(x_0-x_g)-dx*cos(theta)*(1-cos(psi))-dy*sin(theta)); //corrected theta
            if(theta_dif > abs(theta_cor-theta)){
                theta_dif = abs(theta_cor-theta);
            } else{
                psi = psi - 0.02;
                theta_cor = atan((y_0-y_g-dx*sin(theta)*(1-cos(psi))-dy*cos(theta))/(x_0-x_g)-dx*cos(theta)*(1-cos(psi))-dy*sin(theta));
                if(theta_dif > abs(theta_cor-theta)){
                    theta_dif = abs(theta_cor-theta);
                    else{
                    test == 1;
                    psi = psi + 0.01;
                }
           }
        }
    test = 0;
    if(psi_dif > 0.01 && test == 0){
        theta = theta + 0.01;
        psi_cor = acos((z_0-z_g-dx*sin(theta))/(sqrt((x_0-x_g-dx*cos(theta)*(1-cos(psi))-dy*sin(theta))^2+(y_0-y_g-dx*sin(theta)*(1-cos(psi))-dy*cos(theta))^2+(z_0-z_g-dx*(sin(theta)))^2)));
	if(psi_dif > abs(psi_cor-psi)){
            psi_dif = abs(psi_cor-psi);
        } else{
            theta = theta - 0.02;
            psi_cor = acos((z_0-z_g-dx*sin(theta))/(sqrt((x_0-x_g-dx*cos(theta)*(1-cos(psi))-dy*sin(theta))^2+(y_0-y_g-dx*sin(theta)*(1-cos(psi))-dy*cos(theta))^2+(z_0-z_g-dx*(sin(theta)))^2)));
	    if(psi_dif > abs(psi_cor-psi)){
                psi_dif = abs(psi_cor-psi);
                else{
                test == 1;
                theta = theta + 0.01;
            }
        }
    }
    test = 0;
    if(psi_dif > 0.01 && test == 0){
        psi = psi + 0.01;
        psi_cor = acos((z_0-z_g-dx*sin(theta))/(sqrt((x_0-x_g-dx*cos(theta)*(1-cos(psi))-dy*sin(theta))^2+(y_0-y_g-dx*sin(theta)*(1-cos(psi))-dy*cos(theta))^2+(z_0-z_g-dx*(sin(theta)))^2)));
        if(psi_dif > abs(psi_cor-psi)){
            psi_dif = abs(psi_cor-psi);
        } else{
            psi = psi - 0.02;
            psi_cor = acos((z_0-z_g-dx*sin(theta))/(sqrt((x_0-x_g-dx*cos(theta)*(1-cos(psi))-dy*sin(theta))^2+(y_0-y_g-dx*sin(theta)*(1-cos(psi))-dy*cos(theta))^2+(z_0-z_g-dx*(sin(theta)))^2)));
            if(psi_dif > abs(psi_cor-psi)){
                psi_dif = abs(psi_cor-psi);
                else{
                test == 1;
                psi = psi + 0.01;
            }
        }
    }
}


   //start with simple,single sphere, later : local sphere (ToDo: need errors)
    double x_c = 0.145; //from FIT to vertical run 835
    double y_c = 0.311; //from FIT to vertical run 835
    double z_c = 0.; //measured
    double radius = 0.0914; //from FIT to vertical run 835
  

    if(gantry != BOTH){
      //normal is (x-x_c,y-y_c,z-z_c)
      // convert to spherical coordinates (rotation and tilt)
      double rotation = atan2(point[1+ gantry*5]-y_c,point[gantry*5]-x_c);
      double tilt = acos(point[2+gantry*5]-z_c);
      
      //Checks

      point[3+gantry*5] = rotation;
      point[4+gantry*5] = tilt;
      //TODO: Calculate cross-section of normal on PMT surface and store somewhere
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
