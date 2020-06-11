/********************************************************************\
 
  Name:         TGantryConfigCalculator.cxx
  Created by:   Rika
  Date:         27 March 2017

  -------------------------------------------------------------------
  date         by    modification
  ---------    ---   ------------------------------------------------

\********************************************************************/

#include "TGantryConfigCalculator.hxx"
#include "midas.h" // for logging


//----------------------------------------------------------
TGantryConfigCalculator::TGantryConfigCalculator(double tiltMotor, double gFront, double gBack, double gBoxWidth,
                                                 double gTiltGearWidth, double gBoxHeight) {
//----------------------------------------------------------

  tiltMotorLength = tiltMotor;
  gantryFrontHalfLength = gFront;
  gantryBackHalfLength = gBack;
  gantryOpticalBoxWidth = gBoxWidth;
  gantryTiltGearWidth = gTiltGearWidth;
  gantryOpticalBoxHeight = gBoxHeight;

}


//----------------------------------------------------------
XYPolygon TGantryConfigCalculator::GetGantryConfig(int whichGantry, double rot, double tilt, double xPos, double yPos) {
//----------------------------------------------------------

  XYPolygon gantryConfig;

  // Gantry dimensions:
  double front_tiltMotor = tiltMotorLength;
  double back = gantryBackHalfLength;
  double left = gantryOpticalBoxWidth;
  double right = gantryTiltGearWidth;
  double height = gantryOpticalBoxHeight;

  double gantryXDimensions[4];
  double gantryYDimensions[4];

  if (whichGantry == 1) { // Gantry 1 is rotated 180 degrees compared to Gantry 0
    front_tiltMotor *= -1;
    back *= -1;
    left *= -1;
    right *= -1;
    height *= -1;
  }

  // Calculating gantry configuration, including tilt motor at the top
  gantryXDimensions[0] = front_tiltMotor * cos(rot) + right * sin(rot);
  gantryYDimensions[0] = front_tiltMotor * sin(rot) - right * cos(rot);

  gantryXDimensions[1] = front_tiltMotor * cos(rot) - left * sin(rot);
  gantryYDimensions[1] = front_tiltMotor * sin(rot) + left * cos(rot);

  gantryXDimensions[2] = (-1) * back * cos(rot) * cos(tilt) - left * sin(rot) + height * cos(rot) * sin(tilt);
  gantryYDimensions[2] = (-1) * back * sin(rot) * cos(tilt) + left * cos(rot) + height * sin(rot) * sin(tilt);

  gantryXDimensions[3] = (-1) * back * cos(rot) * cos(tilt) + right * sin(rot) + height * cos(rot) * sin(tilt);
  gantryYDimensions[3] = (-1) * back * sin(rot) * cos(tilt) - right * cos(rot) + height * sin(rot) * sin(tilt);

  gantryConfig.clear();

  for (int i = 0; i < 4; ++i) {
    gantryConfig.push_back(std::make_pair(gantryXDimensions[i] + xPos, gantryYDimensions[i] + yPos));
  }

  return gantryConfig;

}


//----------------------------------------------------------
std::pair <XYPolygon, XYPolygon>
TGantryConfigCalculator::GetOpticalBoxConfig(int whichGantry, double rot, double tilt, double xPos, double yPos) {
//----------------------------------------------------------

  XYPolygon boxConfig_lo;
  XYPolygon boxConfig_up;

  // Optical box dimensions:
  double front = gantryFrontHalfLength;
  double back = gantryBackHalfLength;
  double left = gantryOpticalBoxWidth;
  double right = gantryTiltGearWidth;
  double height = gantryOpticalBoxHeight;

  double boxXDimensions[8];
  double boxYDimensions[8];

  if (whichGantry == 1) {
    front *= -1;
    back *= -1;
    left *= -1;
    right *= -1;
    height *= -1;
  }

  //Calculating optical box configuration
  boxXDimensions[0] = front * cos(rot) * cos(tilt) + right * sin(rot) + height * cos(rot) * sin(tilt);
  boxYDimensions[0] = front * sin(rot) * cos(tilt) - right * cos(rot) + height * sin(rot) * sin(tilt);

  boxXDimensions[1] = front * cos(rot) * cos(tilt) - left * sin(rot) + height * cos(rot) * sin(tilt);
  boxYDimensions[1] = front * sin(rot) * cos(tilt) + left * cos(rot) + height * sin(rot) * sin(tilt);

  boxXDimensions[2] = (-1) * back * cos(rot) * cos(tilt) - left * sin(rot) + height * cos(rot) * sin(tilt);
  boxYDimensions[2] = (-1) * back * sin(rot) * cos(tilt) + left * cos(rot) + height * sin(rot) * sin(tilt);

  boxXDimensions[3] = (-1) * back * cos(rot) * cos(tilt) + right * sin(rot) + height * cos(rot) * sin(tilt);
  boxYDimensions[3] = (-1) * back * sin(rot) * cos(tilt) - right * cos(rot) + height * sin(rot) * sin(tilt);

  boxXDimensions[4] = front * cos(rot) * cos(tilt) + right * sin(rot);
  boxYDimensions[4] = front * sin(rot) * cos(tilt) - right * cos(rot);

  boxXDimensions[5] = front * cos(rot) * cos(tilt) - left * sin(rot);
  boxYDimensions[5] = front * sin(rot) * cos(tilt) + left * cos(rot);

  boxXDimensions[6] = (-1) * back * cos(rot) * cos(tilt) - left * sin(rot);
  boxYDimensions[6] = (-1) * back * sin(rot) * cos(tilt) + left * cos(rot);

  boxXDimensions[7] = (-1) * back * cos(rot) * cos(tilt) + right * sin(rot);
  boxYDimensions[7] = (-1) * back * sin(rot) * cos(tilt) - right * cos(rot);

  for (int i = 0; i < 4; ++i) {
    boxConfig_lo.push_back(std::make_pair(boxXDimensions[i] + xPos, boxYDimensions[i] + yPos));
    boxConfig_up.push_back(std::make_pair(boxXDimensions[i + 4] + xPos, boxYDimensions[i + 4] + yPos));
  }

  return std::make_pair(boxConfig_lo, boxConfig_up);

}


//----------------------------------------------------------
std::pair<double, double> TGantryConfigCalculator::GetOpticalBoxZ(int whichGantry, double tilt, double zPos) {
//----------------------------------------------------------

  // Optical box dimensions:
  double front = gantryFrontHalfLength;
  double back = gantryBackHalfLength;
  double height = gantryOpticalBoxHeight;

  double z_opticalBoxLowerSurface = gantryOpticalBoxHeight;
  double z_opticalBoxUpperSurface = 0.0;

  if (tilt < 0) {
    z_opticalBoxLowerSurface = (-1) * front * sin(tilt) + height * cos(tilt);
    z_opticalBoxUpperSurface = (-1) * front * sin(tilt);
  } else {
    z_opticalBoxLowerSurface = back * sin(tilt) + height * cos(tilt);
    z_opticalBoxUpperSurface = back * sin(tilt);
  }

  return std::make_pair(z_opticalBoxLowerSurface + zPos, z_opticalBoxUpperSurface + zPos);

}
