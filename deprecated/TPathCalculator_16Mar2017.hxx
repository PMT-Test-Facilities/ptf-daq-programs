#ifndef PathCalculator_H
#define PathCalculator_H

#include <boost/geometry/geometry.hpp> 
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

typedef boost::geometry::model::d2::point_xy<double> Point;
typedef boost::geometry::model::polygon<Point> Polygon;



class TPathCalculator {

    public:
        TPathCalculator();

        // Input dimensions of the two gantries & optical boxes
        // z0 & z1 relate the optical box heights to the indecies of the corresponding PMT polygon to be compared.
        void InitialiseGantries(std::vector<std::pair<double, double> > points1, std::vector<std::pair<double, double> > points2);

        //TF: TODO: input dimensions of PMT here if needed:
        // Input dimensions of PMT polygon at z = 0.460 + 0.115 m
        void InitialisePMT(std::vector< std::pair<double, double> > points);
 
        // Calculate a path between two x,y points that does not create collisions
        // Returns true if this is possible, with the vector 'path' holding the good path
        bool CalculatePath(std::pair<double, double> start, std::pair<double, double> end, std::vector<std::pair<double, double> >& path, std::pair<bool, bool> tank_height_start, std::pair<bool, bool> tank_height_end);
        // Checks a path to see if it collides with either gantry or the water tank
        bool CheckPathForCollisions(bool isObject1, std::vector<std::pair<double, double> >& path,  std::pair<bool, bool> tank_height_start, std::pair<bool, bool> tank_height_end);
        // Method that creates a possible path between the start and end points
        // TF: added option to avoid unnecessary waypoints for ACTUAL movement
  void CreatePath(double startX, double startY, double endX, double endY, std::vector<std::pair<double, double> >& path, bool xFirst, bool simplePath);

    private:
        // BOOST Geometry polygons to store the position and dimensions of the gantries and PMT
        Polygon* fObject1; //Gantry0
        Polygon* fObject2; //Gantry1
        Polygon* fOpticalBox0;
        Polygon* fOpticalBox1;
        Polygon* fPMTpoly0; // PMT position as seen by Gantry 0
        Polygon* fPMTpoly1; // PMT position as seen by Gantry 1

        int height_OpticalBox0; // Index relating optical box 0 z height to the index in the PMT polygon vector
                                // for collision avoidance calc w/ PMT    
        int height_OpticalBox1;
};

#endif

