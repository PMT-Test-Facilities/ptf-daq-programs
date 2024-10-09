#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <math.h> //abs
#include <algorithm> // used for upper_bound

#include <cmath> // nan

const  double CLOSE_ENOUGH = 1e-7;

int binary_search( double value, std::vector< double> points);

class NormCalc{
    public:
        NormCalc();
        std::vector<double> surface(double theta, double phi);
        std::vector< double> evaluate_norm( double theta,  double phi);
        double pmt_height();
        void test_print();

    private:

        void add_angle( double angle, std::vector< double>& point_list);

        double a,b,c;

        std::vector< double> phi_points; 
        std::vector< double> theta_points; 
        // organized as theta/phi 
        std::vector<std::vector< double>> norm_x;
        std::vector<std::vector< double>> norm_y;
        std::vector<std::vector< double>> norm_z;
        std::vector<std::vector< double>> radius;

         double biliner_inerpolation( double theta,  double phi,const std::vector<std::vector< double>>& point_vecs);

        const std::string filename = "/home/midptf/online/ptf-daq-programs/norm_calc/spline_fit_data.dat"; 

};