#include "TRotationCalculator.hxx"
#include "midas.h" // for logging

// Old code, not using TGantryConfigCalculator 
//----------------------------------------------------------
TRotationCalculator::TRotationCalculator(){
//----------------------------------------------------------

    fObject_rot1 = new BoostPolygon();
    fObject_rot2 = new BoostPolygon();
}

//----------------------------------------------------------
bool TRotationCalculator::CalculatePath(XYPoint start1, XYPoint start2, std::pair<double, double> rotation1, std::pair<double, double> rotation2, double gant1_tilt, double gant2_tilt, std::pair<bool, bool> tank_height_start, std::pair<bool, bool> tank_height_end){
//----------------------------------------------------------

  const double pi = boost::math::constants::pi<double>();

  // initialize variables
  double startX1 = start1.first;
  double startY1 = start1.second;
  double startX2 = start2.first;
  double startY2 = start2.second;
  double rot_start1 = rotation1.first;
  double rot_end1 = rotation1.second;
  double rot_start2 = rotation2.first;
  double rot_end2 = rotation2.second;

  double pmtHeight = 0.59; //TODO: find a way to bring this value from feMove; needed for PMT collision avoidance.
  double pmtPolyLayerHeight = 0.01; // Each PMT polygon is 1cm thick.

  std::vector<double> X1(4);
  std::vector<double> Y1(4);
  std::vector<double> X2(4);
  std::vector<double> Y2(4);

  bool collisionFree = true;

  //determine number of increments in rotation analyzed 
  double increment = 5.0; // increment size (degrees) - 5.0 default
  double tot_rot1 = rot_start1 - rot_end1;
  int N1_steps = abs(tot_rot1)/increment;
  double tot_rot2 = rot_start2 - rot_end2;
  int N2_steps = abs(tot_rot2)/increment;
  double rot1 = rot_start1*pi/180;
  double rot2 = rot_start2*pi/180;

  //cm_msg(MINFO,"CreatePath"," NUMBER STEPS  = %3.3i & %3.3i", N1_steps, N2_steps);
  // if number of rotation steps in gantry 1 >= gantry 2
  if(N1_steps >= N2_steps){
    for(int i = 0; i < N1_steps + 1; i++){
      // Rounding errors - take to final rotation position on last step
      if(i > N1_steps - 1){
        rot2 = rot_end2*pi/180;
        rot1 = rot_end1*pi/180;
      }     
      
      //TODO the boundary conditions should be carried in from feMove, not defined here
      // determine boundary coordintes for gantry 1 & 2
      Y1[0] = cos(gant1_tilt)*(-0.070*cos(rot1)-0.200*sin(rot1))+(1-cos(gant1_tilt))*(-0.070*cos(rot1)-0.110*sin(rot1)) + startY1;
      X1[0] = cos(gant1_tilt)*(-0.200*cos(rot1)+0.070*sin(rot1))+(1-cos(gant1_tilt))*(-0.110*cos(rot1)+0.070*sin(rot1)) + startX1;
      Y1[1] = cos(gant1_tilt)*(-0.070*cos(rot1)+0.140*sin(rot1))+(1-cos(gant1_tilt))*(-0.070*cos(rot1)+0.110*sin(rot1)) + startY1;
      X1[1] = cos(gant1_tilt)*(0.140*cos(rot1)+0.070*sin(rot1))+(1-cos(gant1_tilt))*(0.110*cos(rot1)+0.070*sin(rot1)) + startX1;
      Y1[2] = cos(gant1_tilt)*(0.160*cos(rot1)+0.140*sin(rot1))+(1-cos(gant1_tilt))*(0.160*cos(rot1)+0.110*sin(rot1)) + startY1;
      X1[2] = cos(gant1_tilt)*(0.140*cos(rot1)-0.160*sin(rot1))+(1-cos(gant1_tilt))*(0.110*cos(rot1)-0.160*sin(rot1)) + startX1;
      Y1[3] = cos(gant1_tilt)*(0.160*cos(rot1)-0.200*sin(rot1))+(1-cos(gant1_tilt))*(0.160*cos(rot1)-0.110*sin(rot1)) + startY1;
      X1[3] = cos(gant1_tilt)*(-0.200*cos(rot1)-0.160*sin(rot1))+(1-cos(gant1_tilt))*(-0.110*cos(rot1)-0.160*sin(rot1)) + startX1;

      Y2[0] = cos(gant2_tilt)*(0.070*cos(rot2)+0.200*sin(rot2))+(1-cos(gant2_tilt))*(0.070*cos(rot2)+0.110*sin(rot2)) + startY2;
      X2[0] = cos(gant2_tilt)*(0.200*cos(rot2)-0.070*sin(rot2))+(1-cos(gant2_tilt))*(0.110*cos(rot2)-0.070*sin(rot2)) + startX2;
      Y2[1] = cos(gant2_tilt)*(0.070*cos(rot2)-0.140*sin(rot2))+(1-cos(gant2_tilt))*(0.070*cos(rot2)-0.110*sin(rot2)) + startY2;
      X2[1] = cos(gant2_tilt)*(-0.140*cos(rot2)-0.070*sin(rot2))+(1-cos(gant2_tilt))*(-0.110*cos(rot2)-0.070*sin(rot2)) + startX2;
      Y2[2] = cos(gant2_tilt)*(-0.160*cos(rot2)-0.140*sin(rot2))+(1-cos(gant2_tilt))*(-0.160*cos(rot2)-0.110*sin(rot2)) + startY2;
      X2[2] = cos(gant2_tilt)*(-0.140*cos(rot2)+0.160*sin(rot2))+(1-cos(gant2_tilt))*(-0.110*cos(rot2)+0.160*sin(rot2)) + startX2;
      Y2[3] = cos(gant2_tilt)*(-0.160*cos(rot2)+0.200*sin(rot2))+(1-cos(gant2_tilt))*(-0.160*cos(rot2)+0.110*sin(rot2)) + startY2;
      X2[3] = cos(gant2_tilt)*(0.200*cos(rot2)+0.160*sin(rot2))+(1-cos(gant2_tilt))*(0.110*cos(rot2)+0.160*sin(rot2)) + startX2; 
     
      // append these coordinates to fObject_rot1 & fObject_rot2
      for(int j = 0; j < 4; ++j){
        boost::geometry::append(*fObject_rot1, boost::geometry::make<boost::geometry::model::d2::point_xy<double> >(X1[j],Y1[j]));
        boost::geometry::append(*fObject_rot2, boost::geometry::make<boost::geometry::model::d2::point_xy<double> >(X2[j],Y2[j]));
        std::cout<< "RF DEBUG " << "Corner values:  " << X1[j] << " " << Y1[j] << " : " << X2[j] << " " << Y2[j] << std::endl;
      }
      boost::geometry::correct(*fObject_rot1);
      boost::geometry::correct(*fObject_rot2);

      // Check gantry-to-gantry collision:
      collisionFree = CheckPathForCollisions(fObject_rot1, fObject_rot2, tank_height_start.first, tank_height_end.first);
      if(!collisionFree){
          cm_msg(MINFO, "CalculatePath", "Invalid rotation path: gantries will collide.");
	  return false;
      }

      // increment the rotation b/c rotation 2 is smaller or equal to rotation 1 need to determine how many increments before at its end rotation value
      if(i < N2_steps - 1) {
	// determine if rotation should be increasing or decreasing in value
	if(tot_rot2 < 0){
	  rot2 = rot2 + increment*pi/180;
	} else rot2 = rot2 - increment*pi/180;
      } else rot2 = rot_end2*pi/180;
      // determine if rotation should be increasing or decreasing in value
      if(tot_rot1 < 0){
	rot1 = rot1 + increment*pi/180;
      } else rot1 = rot1 - increment*pi/180;
      
      boost::geometry::clear(*fObject_rot1);
      boost::geometry::clear(*fObject_rot2);
    }
  } else { // if number of rotation steps in gantry 2 > gantry 1
    for(int i = 0; i < N2_steps + 1; i ++){
      
      // Rounding errors - take to final rotation position on last step 
      if(i > N2_steps - 1){
        rot2 = rot_end2*pi/180;
        rot1 = rot_end1*pi/180;
      }
      //TODO the boundary conditions should be carried in from feMove, not defined here
      // determine boundary coordintes for gantry 1 & 2
      Y1[0] = cos(gant1_tilt)*(-0.070*cos(rot1)-0.200*sin(rot1))+(1-cos(gant1_tilt))*(-0.070*cos(rot1)-0.110*sin(rot1)) + startY1;
      X1[0] = cos(gant1_tilt)*(-0.200*cos(rot1)+0.070*sin(rot1))+(1-cos(gant1_tilt))*(-0.110*cos(rot1)+0.070*sin(rot1)) + startX1;
      Y1[1] = cos(gant1_tilt)*(-0.070*cos(rot1)+0.140*sin(rot1))+(1-cos(gant1_tilt))*(-0.070*cos(rot1)+0.110*sin(rot1)) + startY1;
      X1[1] = cos(gant1_tilt)*(0.140*cos(rot1)+0.070*sin(rot1))+(1-cos(gant1_tilt))*(0.110*cos(rot1)+0.070*sin(rot1)) + startX1;
      Y1[2] = cos(gant1_tilt)*(0.160*cos(rot1)+0.140*sin(rot1))+(1-cos(gant1_tilt))*(0.160*cos(rot1)+0.110*sin(rot1)) + startY1;
      X1[2] = cos(gant1_tilt)*(0.140*cos(rot1)-0.160*sin(rot1))+(1-cos(gant1_tilt))*(0.110*cos(rot1)-0.160*sin(rot1)) + startX1;
      Y1[3] = cos(gant1_tilt)*(0.160*cos(rot1)-0.200*sin(rot1))+(1-cos(gant1_tilt))*(0.160*cos(rot1)-0.110*sin(rot1)) + startY1;
      X1[3] = cos(gant1_tilt)*(-0.200*cos(rot1)-0.160*sin(rot1))+(1-cos(gant1_tilt))*(-0.110*cos(rot1)-0.160*sin(rot1)) + startX1;

      Y2[0] = cos(gant2_tilt)*(0.070*cos(rot2)+0.200*sin(rot2))+(1-cos(gant2_tilt))*(0.070*cos(rot2)+0.110*sin(rot2)) + startY2;
      X2[0] = cos(gant2_tilt)*(0.200*cos(rot2)-0.070*sin(rot2))+(1-cos(gant2_tilt))*(0.110*cos(rot2)-0.070*sin(rot2)) + startX2;
      Y2[1] = cos(gant2_tilt)*(0.070*cos(rot2)-0.140*sin(rot2))+(1-cos(gant2_tilt))*(0.070*cos(rot2)-0.110*sin(rot2)) + startY2;
      X2[1] = cos(gant2_tilt)*(-0.140*cos(rot2)-0.070*sin(rot2))+(1-cos(gant2_tilt))*(-0.110*cos(rot2)-0.070*sin(rot2)) + startX2;
      Y2[2] = cos(gant2_tilt)*(-0.160*cos(rot2)-0.140*sin(rot2))+(1-cos(gant2_tilt))*(-0.160*cos(rot2)-0.110*sin(rot2)) + startY2;
      X2[2] = cos(gant2_tilt)*(-0.140*cos(rot2)+0.160*sin(rot2))+(1-cos(gant2_tilt))*(-0.110*cos(rot2)+0.160*sin(rot2)) + startX2;
      Y2[3] = cos(gant2_tilt)*(-0.160*cos(rot2)+0.200*sin(rot2))+(1-cos(gant2_tilt))*(-0.160*cos(rot2)+0.110*sin(rot2)) + startY2;
      X2[3] = cos(gant2_tilt)*(0.200*cos(rot2)+0.160*sin(rot2))+(1-cos(gant2_tilt))*(0.110*cos(rot2)+0.160*sin(rot2)) + startX2;

      for(std::vector<BoostPoint>::size_type j = 0; j < 4; ++j){
        boost::geometry::append(*fObject_rot1, boost::geometry::make<boost::geometry::model::d2::point_xy<double> >(X1[j],Y1[j]));
        boost::geometry::append(*fObject_rot2, boost::geometry::make<boost::geometry::model::d2::point_xy<double> >(X2[j],Y2[j]));
      }
      boost::geometry::correct(*fObject_rot1);
      boost::geometry::correct(*fObject_rot2);     

      // Check gantry-to-gantry collision:
      collisionFree = CheckPathForCollisions(fObject_rot1, fObject_rot2, tank_height_start.first, tank_height_end.first);
      if(!collisionFree){
          cm_msg(MINFO, "CalculatePath", "Invalid rotation path: gantries will collide.");
          return false;
      }

      if(i < N1_steps - 1) {
        if(tot_rot1 < 0){
          rot1 = rot1 + increment*pi/180;
        } else rot1 = rot1 - increment*pi/180;
      } else rot1 = rot_end1*pi/180;
      if(tot_rot2 < 0){
        rot2 = rot2 + increment*pi/180;
      } else rot2 = rot2 - increment*pi/180;
      
      boost::geometry::clear(*fObject_rot1);
      boost::geometry::clear(*fObject_rot2);
    }
  }
  return collisionFree;

} 

// Rika (23Mar2017): Implemented CheckPathForCollisions function for PMT collision avoidance.
// Taken from TPathCalculator.cxx
//----------------------------------------------------------
bool TRotationCalculator::CheckPathForCollisions(BoostPolygon* objectMoving, BoostPolygon* objectStationary, bool tank_height_start, bool tank_height_end){
//----------------------------------------------------------

  //tank boundaries
  //TODO the center pos, maxval, and Ydifmax should be called in from feMove, not defined here
  double Xdif;
  double Ydif;
  double Xcent = 0.325;
  double Ycent = 0.335;
  double Maxval = 0.61; // radius of tank ~0.61
  double Ydifmax = 0.53; // max/min from center where PMT holders sit

  std::vector<BoostPoint> objectPoints = objectMoving->outer(); 

      bool collides_tank = 0;
      bool collides_obj = 0;
	       
      for(std::vector<BoostPoint>::size_type i = 0; i < objectPoints.size(); ++i){
	  double newX = boost::geometry::get<0>(objectPoints[i]);
	  double newY = boost::geometry::get<1>(objectPoints[i]);

	  // std::cout<< "TF DEBUG " << iter << " " << i << " " << newX << " " << newY << " " << path[iter].first << " " << path[iter].second << std::endl;
	  boost::geometry::set<0>(objectPoints[i], newX);
	  boost::geometry::set<1>(objectPoints[i], newY);

	  // Determine if box moves outside of tank limits. If gantry doesn't move above tank first and if starting height is not outside of tank, must check limits
	  if(!tank_height_end){
	      if(!tank_height_start){
		  if(newX >= Xcent) Xdif = newX-Xcent;
		  else Xdif = Xcent-newX;
		  if(newY >= Ycent) Ydif = newY-Ycent;
		  else Ydif = Ycent-newY;
		  if((sqrt(Xdif*Xdif+Ydif*Ydif) > Maxval) || (Ydifmax < Ydif)){
		      collides_tank = true;
		  }
	      }
	  }
      }
      BoostPolygon *fTestObject = new BoostPolygon();

      boost::geometry::assign_points(*fTestObject,objectPoints);
      boost::geometry::correct(*fTestObject);
      collides_obj = boost::geometry::intersects(*fTestObject, *objectStationary);

      if(collides_obj || collides_tank) return false;

  return true;
}

// Rika (23Mar2017): From TPathCalculator.cxx
//----------------------------------------------------------
void TRotationCalculator::CreatePolygon(BoostPolygon* object, XYPolygon points){
//----------------------------------------------------------

    boost::geometry::clear(*object);
    
    for(int i = 0; i < (int)points.size(); ++i){
	if(points.size()<5){ // To ensure that it doesn't print repeatedly for PMT initialization
	     cm_msg(MINFO,"CreatePolygon"," Position = %.3lf  %.3lf", points[i].first, points[i].second); // Rika: Do we need this?
	}
	boost::geometry::append(*object, boost::geometry::make<BoostPoint>(points[i].first, points[i].second));
    }

    boost::geometry::correct(*object);

}
