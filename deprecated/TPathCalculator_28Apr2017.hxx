#ifndef PathCalculator_H
#define PathCalculator_H

#include <boost/geometry/geometry.hpp> 
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

typedef boost::geometry::model::d2::point_xy<double> BoostPoint;
typedef boost::geometry::model::polygon<BoostPoint> BoostPolygon;

typedef std::pair<double, double> XYPoint;
typedef std::vector<XYPoint> XYPolygon;


class TPathCalculator {

    public:
        TPathCalculator(double tankCentreXPos, double tankCentreYPos, double tankRadius, double tankPMTholderRadius);

        // Input dimensions of the two gantries
        void InitialiseGantries(XYPolygon points0, XYPolygon points1);

        // Input dimensions of the two optical boxes
        // z0 & z1 relate the optical box heights to the indecies of the corresponding PMT polygon to be compared.
        // lo & up correspond to the lower and upper surfaces of the optical box.
        void InitialiseOpticalBoxes(XYPolygon points0_lo, XYPolygon points0_up, XYPolygon points1_lo, XYPolygon points1_up, int z0_lo, int z0_up, int z1_lo, int z1_up);

        // Input dimensions of PMT
        // Two models for the PMT, one for each gantry, in case the coordinates are different between the gantries
        void InitialisePMT(std::vector<XYPolygon> pmt0, std::vector<XYPolygon> pmt1);
 
        // Calculate a path between two x,y points that does not create collisions
        // Returns true if this is possible, with the vector 'path' holding the good path
        bool CalculatePath(XYPoint start, XYPoint end, std::vector<XYPoint>& path, std::pair<bool, bool> tank_height_start, std::pair<bool, bool> tank_height_end);

        // Checks a path to see if it collides with either gantry or the water tank
        bool CheckPathForCollisions(BoostPolygon* objectMoving, BoostPolygon* objectStationary, std::vector<XYPoint>& path, bool tank_height_start, bool tank_height_end);

        // Method that creates a possible path between the start and end points
        // TF: added option to avoid unnecessary waypoints for ACTUAL movement
        void CreatePath(double startX, double startY, double endX, double endY, std::vector<XYPoint>& path, bool xFirst, bool simplePath);

        // Method that checks destination for optical box-to-PMT collision
        bool CheckDestination(XYPolygon points0, XYPolygon points1, bool tank_height_end);

    private:
        void CreatePolygon(BoostPolygon* object, XYPolygon points);

        // BOOST Geometry polygons to store the position and dimensions of the gantries and PMT
        BoostPolygon* fObject0; //Gantry0
        BoostPolygon* fObject1; //Gantry1
        BoostPolygon* fOpticalBox0_lo;
        BoostPolygon* fOpticalBox1_lo;
        BoostPolygon* fOpticalBox0_up;
        BoostPolygon* fOpticalBox1_up;
        std::vector<BoostPolygon*> fPMTmultiPoly0; // PMT position as seen by Gantry 0
        std::vector<BoostPolygon*> fPMTmultiPoly1; // PMT position as seen by Gantry 1

        int height_OpticalBox0_lo; // Index relating optical box 0 z height to the index in the PMT polygon vector
                                   // for collision avoidance calc w/ PMT    
        int height_OpticalBox1_lo;
        int height_OpticalBox0_up;
        int height_OpticalBox1_up;

        // tank dimensions:
        double Xcent;
        double Ycent;
        double tRadius;
        double tPMTholder;
};

#endif

