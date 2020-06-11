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
        TRotationCalculator();

        // Rika (23Mar2017):
        // Input positions of PMT polygon for different z layers.
        void InitialisePMT(std::vector<XYPolygon> pmt0, std::vector<XYPolygon> pmt1); 

        // Calculate rotation and tilt paths in n-degree increments between start and end rotation / tilt. 
        // If collision with water tank or other gantry detected at any increment returns false.
        // Rika (3Apr2017): Updated to include tilt path collision check
    bool CalculatePath(XYPoint start1, XYPoint start2, std::pair<double, double> rotation1, std::pair<double, double> rotation2, double gant1_tilt, double gant2_tilt, std::pair<bool, bool> tank_height_start, std::pair<bool, bool> tank_height_end);

    private:
        // Rika (23Mar2017): Generic function to create BoostPolygon objects.
        void CreatePolygon(BoostPolygon* object, XYPolygon points);

        // Rika (23Mar2017):
        // Checks position of objectMoving to see if it collides with objectStationary or with the water tank.
        // Returns true if path is free of collision; false otherwise.
        bool CheckPathForCollisions(BoostPolygon* objectMoving, BoostPolygon* objectStationary, bool tank_height_start, bool tank_height_end);

        // BOOST Geometry polygons to store the position and dimensions Gantries
        BoostPolygon* fObject_rot1;
        BoostPolygon* fObject_rot2;

};

#endif
