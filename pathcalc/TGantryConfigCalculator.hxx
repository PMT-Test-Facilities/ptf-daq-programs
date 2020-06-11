/********************************************************************\
 
  Name:         TGantryConfigCalculator.hxx
  Created by:   Rika
  Date:         27 March 2017

  This class is used to determine the space occupied by the gantries
  and their optical boxes, given the x, y, z position, and the
  rotation and tilt angles.

  -------------------------------------------------------------------
  date         by    modification
  ---------    ---   ------------------------------------------------

\********************************************************************/

#ifndef GantryConfigCalculator_H
#define GantryConfigCalculator_H

#include <cmath>
#include <vector>

typedef std::pair<double, double> XYPoint;
typedef std::vector<XYPoint> XYPolygon;

class TGantryConfigCalculator {

public:
    /**
     * Input dimensions of gantry and optical box:
     **/
    TGantryConfigCalculator(double tiltMotorLength, double gantryFrontHalfLength, double gantryBackHalfLength, double gantryOpticalBoxWidth, double gantryTiltGearWidth, double gantryOpticalBoxHeight);

    /**
     * Calculates the position of the four widest corners of the gantry.
     *
     * Parameters:
     *     whichGantry -> 0 := Gantry 0; 1:= Gantry 1
     *     rot, tilt -> rotation and tilt angles in radians
     *     xPos, yPos -> gantry position, in metres.
     *
     * Returns: 
     * A vector of four corners outlining the gantry area.
     * 
     * Looking at the gantry from above, with the laser aperture as the front,
     * the four corners of the gantry area are defined by the gantryDimensions array index as follows:
     *     0 := front right point
     *     1 := front left point 
     *     2 := back left point 
     *     3 := back right point 
     * NOTE: front most points of gantry area are defined by the tiltMotorlength (& gantryWidth)
     *       since tiltMotorLength > gantryLength_FrontHalf no matter what the tilt.
     *       This is why it is necessary to calculate the optical box configuration separately for more precise
     *       collision avoidance control with PMT (see getOpticalBoxConfig below).
     **/
    XYPolygon GetGantryConfig(int whichGantry, double rot, double tilt, double xPos, double yPos);

    /**
     * Calculates the position of the eight corners of the optical box.
     *
     * Parameters:
     *     whichGantry -> 0 := OpticalBox 0; 1:= OpticalBox 1
     *     rot, tilt -> rotation and tilt angles in radians
     *     xPos, yPos -> gantry position, in metres.
     *
     * Returns:
     * A pair of vectors, each of the four corners outlining the optical box lower and upper surfaces.
     * 
     * The optical box configuration is defined by the upper and lower rectangular surfaces of the box,
     * (when tilt = 0 degrees).
     * Looking at the gantry from above, with the laser aperture as the front,
     * the 8 corners of the optical box area are defined by the boxDimensions array index as follows:
     *     0 := front right point of bottom rectangular surface.
     *     1 := front left point . . . 
     *     2 := back left point . . . 
     *     3 := back right point . . . 
     *     4 := front right point of top rectangular surface.
     *     5 := front left point . . . 
     *     6 := back left point . . . 
     *     7 := back right point . . . 
     **/
    std::pair<XYPolygon, XYPolygon> GetOpticalBoxConfig(int whichGantry, double rot, double tilt, double xPos, double yPos);

    /**
     * Calculates the z height of the optical box "lower" and "upper" surfaces, which are defined when tilt = 0.
     * When tilt < 0,
     *     z_opticalBoxLowerSurface := z position of points 0 & 1 (as defined above, in GetOpticalBoxConfig());
     *     z_opticalBoxUpperSurface := z position of points 4 & 5.
     * When tilt >= 0,
     *     z_opticalBoxLowerSurface := z position of points 2 & 3;
     *     z_opticalBoxUpperSurface := z position of points 6 & 7.
     *
     * Parameters:
     *     whichGantry -> 0 := OpticalBox 0; 1:= OpticalBox 1
     *     tilt -> tilt angle in radians
     *     zPos -> gantry Z position, in metres.
     *
     * Returns:
     * A pair of doubles representing the z position of the lower and upper surfaces, respectively.
     **/
    std::pair<double, double> GetOpticalBoxZ(int whichGantry, double tilt, double zPos);

private:
    // Store gantry dimensions here:
    double tiltMotorLength;
    double gantryFrontHalfLength;
    double gantryBackHalfLength;
    double gantryOpticalBoxWidth;
    double gantryTiltGearWidth;
    double gantryOpticalBoxHeight;


};

#endif
