#include "TRotationCalculator.hxx"
#include "midas.h" // for logging
#include "TGantryConfigCalculator.hxx"


//----------------------------------------------------------
TRotationCalculator::TRotationCalculator(double tiltMotorLength, double gantryFrontHalfLength,
                                         double gantryBackHalfLength, double gantryOpticalBoxWidth,
                                         double gantryTiltGearWidth, double gantryOpticalBoxHeight,
                                         double tankCentreXPos, double tankCentreYPos, double tankRadius,
                                         double tankPMTholderRadius) {
//----------------------------------------------------------

  // gantry dimensions
  tiltMotor = tiltMotorLength;
  gFront = gantryFrontHalfLength;
  gBack = gantryBackHalfLength;
  gOpticalBoxWidth = gantryOpticalBoxWidth;
  gTiltGearWidth = gantryTiltGearWidth;
  gOpticalBoxHeight = gantryOpticalBoxHeight;

  //tank boundaries
  Xcent = tankCentreXPos;
  Ycent = tankCentreYPos;
  tRadius = tankRadius;
  tPMTholder = tankPMTholderRadius; // max/min from center where PMT holders sit

}

// Rika (23Mar2017): Added to improve collision avoidance for PMT.
// Taken from TPathCalculator.cxx
// Rika (28Mar2017): Updated to include pmtHeight and pmtPolyLayerHeight.
//----------------------------------------------------------
void TRotationCalculator::InitialisePMT(std::vector <XYPolygon> pmt0, std::vector <XYPolygon> pmt1, double height,
                                        double polygonThickness) {
//----------------------------------------------------------

  for (int i = 0; i < pmt0.size(); ++i) {
    if (i >= fPMTmultiPoly0.size()) {
      fPMTmultiPoly0.push_back(new BoostPolygon());
    }
    CreatePolygon(fPMTmultiPoly0.at(i), pmt0.at(i));
  }
  for (int j = 0; j < pmt1.size(); ++j) {
    if (j >= fPMTmultiPoly1.size()) {
      fPMTmultiPoly1.push_back(new BoostPolygon());
    }
    CreatePolygon(fPMTmultiPoly1.at(j), pmt1.at(j));
  }

  pmtHeight = height;
  pmtPolyLayerHeight = polygonThickness;

}

// Rika (23Mar2017): Updated to deligate collision avoidance control to CheckPathForCollisions.
// Now, main role of CalculatePath is to determine the gantry / optical box orientation based on rotation.
// Rika (28Mar2017): Updated to deligate gantry config calculations to GantryConfigCalculator.
// Rika (3Apr2017): Updated to include tilt path check.
//----------------------------------------------------------
bool TRotationCalculator::CalculatePath(XYPoint start0, XYPoint start1, std::pair<double, double> rotationPath0,
                                        std::pair<double, double> rotationPath1, std::pair<double, double> tiltPath0,
                                        std::pair<double, double> tiltPath1, double gant0_z, double gant1_z,
                                        std::pair<bool, bool> tank_height_start,
                                        std::pair<bool, bool> tank_height_end) {
//----------------------------------------------------------

  const double pi = boost::math::constants::pi<double>();

  BoostPolygon fGantry0;
  BoostPolygon fGantry1;
  BoostPolygon fOpticalBox0_lo;
  BoostPolygon fOpticalBox0_up;
  BoostPolygon fOpticalBox1_lo;
  BoostPolygon fOpticalBox1_up;

  TGantryConfigCalculator gantryConfigCalc(tiltMotor, gFront, gBack, gOpticalBoxWidth, gTiltGearWidth,
                                           gOpticalBoxHeight);

  // initialize variables
  double startX0 = start0.first;
  double startY0 = start0.second;
  double startX1 = start1.first;
  double startY1 = start1.second;

  double rot_start0 = rotationPath0.first;
  double rot_end0 = rotationPath0.second;
  double rot_start1 = rotationPath1.first;
  double rot_end1 = rotationPath1.second;

  double tilt_start0 = tiltPath0.first;
  double tilt_end0 = tiltPath0.second;
  double tilt_start1 = tiltPath1.first;
  double tilt_end1 = tiltPath1.second;

  XYPolygon gantry0;
  XYPolygon gantry1;

  // The first polygon vector defines the lower surface of the box, the second is the upper surface.
  std::pair <XYPolygon, XYPolygon> opticalBox0;
  std::pair <XYPolygon, XYPolygon> opticalBox1;
  XYPolygon opticalBox0_lo;
  XYPolygon opticalBox0_up;
  XYPolygon opticalBox1_lo;
  XYPolygon opticalBox1_up;

  // Z position of the two optical box surfaces relative to the PMT.
  std::pair<double, double> Z0_lo_up;
  std::pair<double, double> Z1_lo_up;
  // PMT polygon layer index calculated as Z = ( opticalBoxZheight + gantZpos - pmtHeight )/ pmtPolyLayerHeight.
  int Z0_lo;
  int Z0_up;
  int Z1_lo;
  int Z1_up;

  bool collisionFree = true;

  //determine number of increments in rotation analyzed 
  double increment_rot = 1.0; // increment size (degrees) - 1.0 new default (31Mar2017)
  double tot_rot0 = rot_start0 - rot_end0;
  int N0_rot_steps = abs(tot_rot0) / increment_rot;
  double tot_rot1 = rot_start1 - rot_end1;
  int N1_rot_steps = abs(tot_rot1) / increment_rot;
  double rot0 = rot_start0 * pi / 180;
  double rot1 = rot_start1 * pi / 180;

  //determine number of increments in tilt analyzed 
  double increment_tilt = 1.0;
  double tot_tilt0 = tilt_start0 - tilt_end0;
  int N0_tilt_steps = abs(tot_tilt0) / increment_tilt;
  double tot_tilt1 = tilt_start1 - tilt_end1;
  int N1_tilt_steps = abs(tot_tilt1) / increment_tilt;
  double tilt0 = tilt_start0 * pi / 180;
  double tilt1 = tilt_start1 * pi / 180;

  //cm_msg(MINFO,"CreatePath"," NUMBER STEPS  = %3.3i & %3.3i", N0_rot_steps, N1_rot_steps);

  // Find max number of steps required to loop through:
  int maxSteps = N0_rot_steps;
  if (N1_rot_steps > maxSteps) {
    maxSteps = N1_rot_steps;
  }
  if (N0_tilt_steps > maxSteps) {
    maxSteps = N0_tilt_steps;
  }
  if (N1_tilt_steps > maxSteps) {
    maxSteps = N1_tilt_steps;
  }

  for (int i = 0; i < maxSteps + 1; i++) {
    // Rounding errors - take to final rotation position on last step
    if (i > maxSteps - 1) {
      rot0 = rot_end0 * pi / 180;
      rot1 = rot_end1 * pi / 180;
      tilt0 = tilt_end0 * pi / 180;
      tilt1 = tilt_end1 * pi / 180;
    }

    // determine boundary coordintes for gantry 0 & 1
    gantry0 = gantryConfigCalc.GetGantryConfig(0, rot0, tilt0, startX0, startY0);
    gantry1 = gantryConfigCalc.GetGantryConfig(1, rot1, tilt1, startX1, startY1);

    opticalBox0 = gantryConfigCalc.GetOpticalBoxConfig(0, rot0, tilt0, startX0, startY0);
    opticalBox1 = gantryConfigCalc.GetOpticalBoxConfig(1, rot1, tilt1, startX1, startY1);

    opticalBox0_lo = opticalBox0.first;
    opticalBox0_up = opticalBox0.second;
    opticalBox1_lo = opticalBox1.first;
    opticalBox1_up = opticalBox1.second;

    Z0_lo_up = gantryConfigCalc.GetOpticalBoxZ(0, tilt0, gant0_z);
    Z1_lo_up = gantryConfigCalc.GetOpticalBoxZ(1, tilt1, gant1_z);

    Z0_lo = (Z0_lo_up.first - pmtHeight) / pmtPolyLayerHeight;
    Z0_up = (Z0_lo_up.second - pmtHeight) / pmtPolyLayerHeight;
    Z1_lo = (Z1_lo_up.first - pmtHeight) / pmtPolyLayerHeight;
    Z1_up = (Z1_lo_up.second - pmtHeight) / pmtPolyLayerHeight;

    CreatePolygon(&fGantry0, gantry0);
    CreatePolygon(&fGantry1, gantry1);

    CreatePolygon(&fOpticalBox0_lo, opticalBox0_lo);
    CreatePolygon(&fOpticalBox0_up, opticalBox0_up);
    CreatePolygon(&fOpticalBox1_lo, opticalBox1_lo);
    CreatePolygon(&fOpticalBox1_up, opticalBox1_up);

    // Check gantry-to-gantry collision:
    collisionFree = CheckPathForCollisions(&fGantry0, &fGantry1, tank_height_start.first, tank_height_end.first);
    if (!collisionFree) {
      cm_msg(MINFO, "CalculatePath",
             "Invalid rotation/tilt path: gantries will collide against each other or with tank.");
      return false;
    }

    // Temporarily disabling
    /*
    if (Z0_lo >= 0) { // If Z0_lo>=0, lower surface of optical box 0 is lowered to PMT region, therefore check against PMT for collision.
      collisionFree = CheckPathForCollisions(&fOpticalBox0_lo, fPMTmultiPoly0.at(Z0_lo), tank_height_start.first,
                                             tank_height_end.first);
    }
    if (collisionFree && Z0_up >= 0) { // Upper surface of optical box 0 must be checked against PMT for collision.
      collisionFree = CheckPathForCollisions(&fOpticalBox0_up, fPMTmultiPoly0.at(Z0_up), tank_height_start.first,
                                             tank_height_end.first);
    }
    if (collisionFree && Z1_lo >= 0) { // If Z1_lo>=0, optical box 1 must be checked against PMT for collision.
      collisionFree = CheckPathForCollisions(&fOpticalBox1_lo, fPMTmultiPoly1.at(Z1_lo), tank_height_start.second,
                                             tank_height_end.second);
    }
    if (collisionFree && Z1_up >= 0) { // Upper surface of optical box 0 must be checked against PMT for collision.
      collisionFree = CheckPathForCollisions(&fOpticalBox1_up, fPMTmultiPoly1.at(Z1_up), tank_height_start.second,
                                             tank_height_end.second);
    }
    */

    if (!collisionFree) {
      cm_msg(MINFO, "CalculatePath", "Invalid rotation/tilt path: gantry will collide with PMT.");
      return false;
    }


    // increment the rot & tilt angles:
    // b/c some angles require less steps then others, we need to determine
    // how many increments before they are at their end rotation value.

    if (i < N0_rot_steps - 1) {
      // determine if rotation should be increasing or decreasing in value
      if (tot_rot0 < 0) {
        rot0 += increment_rot * pi / 180;
      } else {
        rot0 -= increment_rot * pi / 180;
      }
    } else {
      rot0 = rot_end0 * pi / 180;
    }

    if (i < N1_rot_steps - 1) {
      // determine if rotation should be increasing or decreasing in value
      if (tot_rot1 < 0) {
        rot1 += increment_rot * pi / 180;
      } else {
        rot1 -= increment_rot * pi / 180;
      }
    } else {
      rot1 = rot_end1 * pi / 180;
    }

    if (i < N0_tilt_steps - 1) {
      // determine if tilt should be increasing or decreasing in value
      if (tot_tilt0 < 0) {
        tilt0 += increment_tilt * pi / 180;
      } else {
        tilt0 -= increment_tilt * pi / 180;
      }
    } else {
      tilt0 = tilt_end0 * pi / 180;
    }

    if (i < N1_tilt_steps - 1) {
      // determine if tilt should be increasing or decreasing in value
      if (tot_tilt1 < 0) {
        tilt1 += increment_tilt * pi / 180;
      } else {
        tilt1 -= increment_tilt * pi / 180;
      }
    } else {
      tilt1 = tilt_end1 * pi / 180;
    }
  }

  return collisionFree;

}

// Rika (23Mar2017): Implemented CheckPathForCollisions function for PMT collision avoidance.
// Taken from TPathCalculator.cxx
//----------------------------------------------------------
bool TRotationCalculator::CheckPathForCollisions(BoostPolygon *objectMoving, BoostPolygon *objectStationary,
                                                 bool tank_height_start, bool tank_height_end) {
//----------------------------------------------------------

  double Xdif;
  double Ydif;

  double Maxval = tRadius;
  double Ydifmax = tPMTholder; // max/min from center where PMT holders sit

  std::vector <BoostPoint> objectPoints = objectMoving->outer();

  bool collides_tank = 0;
  bool collides_obj = 0;

  // Collision check agaisnt tank:
  for (std::vector<BoostPoint>::size_type i = 0; i < objectPoints.size(); ++i) {
    double newX = boost::geometry::get<0>(objectPoints[i]);
    double newY = boost::geometry::get<1>(objectPoints[i]);

    // std::cout<< "TF DEBUG " << iter << " " << i << " " << newX << " " << newY << " " << path[iter].first << " " << path[iter].second << std::endl;
    boost::geometry::set<0>(objectPoints[i], newX);
    boost::geometry::set<1>(objectPoints[i], newY);

    // Determine if box moves outside of tank limits. If gantry doesn't move above tank first and if starting height is not outside of tank, must check limits
    if (!tank_height_end) {
      if (!tank_height_start) {
        if (newX >= Xcent) Xdif = newX - Xcent;
        else Xdif = Xcent - newX;
        if (newY >= Ycent) Ydif = newY - Ycent;
        else Ydif = Ycent - newY;
        if ((sqrt(Xdif * Xdif + Ydif * Ydif) > Maxval) || (Ydifmax < Ydif)) {
          collides_tank = true;
        }
      }
    }
  }
  BoostPolygon fTestObject;

  boost::geometry::assign_points(fTestObject, objectPoints);
  boost::geometry::correct(fTestObject);
  collides_obj = boost::geometry::intersects(fTestObject, *objectStationary);

  if (collides_obj || collides_tank) return false;

  return true;
}

// Rika (23Mar2017): From TPathCalculator.cxx
//----------------------------------------------------------
void TRotationCalculator::CreatePolygon(BoostPolygon *object, XYPolygon points) {
//----------------------------------------------------------

  boost::geometry::clear(*object);

  for (int i = 0; i < (int) points.size(); ++i) {
    /**
    if(points.size()<5){ // To ensure that it doesn't print repeatedly for PMT initialization
         cm_msg(MINFO,"CreatePolygon"," Position = %.3lf  %.3lf", points[i].first, points[i].second); // Rika: Do we need this?
          }**/
    boost::geometry::append(*object, boost::geometry::make<BoostPoint>(points[i].first, points[i].second));
  }

  boost::geometry::correct(*object);

}
