#ifndef RotationCalculator_H
#define RotationCalculator_H

#include <boost/geometry/geometry.hpp> 
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include "TGantryConfigCalculator.hxx"

typedef boost::geometry::model::d2::point_xy<double> BoostPoint;
typedef boost::geometry::model::polygon<BoostPoint> BoostPolygon;

typedef std::pair<double, double> XYPoint;
typedef std::vector<XYPoint> XYPolygon;


class TRotationCalculator {

    public:
        // Input dimensions of the gantry, optical box, & tank.
        TRotationCalculator(double tiltMotorLength, double gantryFrontHalfLength, double gantryBackHalfLength, double gantryOpticalBoxWidth, double gantryTiltGearWidth, double gantryOpticalBoxHeight, double tankCentreXPos, double tankCentreYPos, double tankRadius, double tankPMTholderRadius);

        // Rika (23Mar2017):
        // Input positions of PMT polygon for different z layers.
        void InitialisePMT(std::vector<XYPolygon> pmt0, std::vector<XYPolygon> pmt1, double pmtHeight, double pmtPolyLayerHeight); 

        // Calculate rotation and tilt paths in n-degree increments between start and end rotation / tilt. 
        // If collision with water tank or other gantry detected at any increment returns false.
        // Rika (3Apr2017): Updated to include tilt path collision check
        bool CalculatePath(XYPoint start0, XYPoint start1, std::pair<double, double> rotationPath0, std::pair<double, double> rotationPath1, std::pair<double, double> tiltPath0, std::pair<double, double> tiltPath1, double gant0_z, double gant1_z, std::pair<bool, bool> tank_height_start, std::pair<bool, bool> tank_height_end);

    private:
        // Rika (23Mar2017): Generic function to create BoostPolygon objects.
        void CreatePolygon(BoostPolygon* object, XYPolygon points);

        // Rika (23Mar2017):
        // Checks position of objectMoving to see if it collides with objectStationary or with the water tank.
        // Returns true if path is free of collision; false otherwise.
        bool CheckPathForCollisions(BoostPolygon* objectMoving, BoostPolygon* objectStationary, bool tank_height_start, bool tank_height_end);

        // BOOST Geometry polygons to store the position and dimensions of PMT
        std::vector<BoostPolygon*> fPMTmultiPoly0; // PMT position as seen by Gantry 0
        std::vector<BoostPolygon*> fPMTmultiPoly1; // PMT position as seen by Gantry 1

        double pmtHeight; // z position of PMT top surface
        double pmtPolyLayerHeight; // Height of PMT polygon layers

        // Gantry dimensions; to be passed to TGantryConfigCalculator to determine orientation of gantry
        double tiltMotor;
        double gFront;
        double gBack;
        double gOpticalBoxWidth;
        double gTiltGearWidth;
        double gOpticalBoxHeight;

        // tank dimensions:
        double Xcent;
        double Ycent;
        double tRadius;
        double tPMTholder;

};

#endif
