#include "norm_eval.h"


int binary_search(const  double value, const std::vector< double> points){
    /*
        returns the index in which to place "value" in "points" to maintain order 

        if the object is provided 0.5 

        [0, 1, 2, 3]
    */

   if(value<points[0]){
    return -1;
   }else if(value>points[points.size()-1]){
    return points.size();
   }

   int min_abs = 0;
   int max_abs = points.size()-1;
   int lower_bin = abs(max_abs - min_abs)/2;
   lower_bin /= 2;
   int upper_bin = lower_bin+1; 

    while( !(points[lower_bin]<=value && points[upper_bin]>=value) ){
        if( abs(max_abs - min_abs)<=1){
            std::cout <<max_abs<<std::endl;
            throw std::runtime_error("Should be unreachable - are the points ordered?");
        }

        if(value<points[lower_bin]){
            max_abs = lower_bin;
        }
        if (value>points[upper_bin]){
            min_abs=upper_bin;
        }

        lower_bin = max_abs-min_abs;
        lower_bin/=2;
        lower_bin += min_abs;
        
        upper_bin = lower_bin+1;
    }

    if( !(value>=points[lower_bin] && value<=points[upper_bin]) ){
        throw std::runtime_error("Somehow the point is not between two other entries");
    }

    return upper_bin;    

}

NormCalc::NormCalc(){
    a = 0.254564663;
    b = 0.254110205;
    c = 0.186002389;

    

}

void NormCalc::add_angle( double angle, std::vector< double>& point_list){
    int index=0;

    // if it's length one, we just check if it's already there
    if(point_list.size()==1){
        if (point_list[0]==angle){
            return; // skip it then 
        }else{
            // now we add it to the back, and swap them if the order is wrong 
            point_list.push_back(angle);
            if(point_list[0]>point_list[1]){
                std::swap(point_list[0], point_list[1]);
            }
            return;
        }
    }else if(point_list.size()==0){
        point_list.push_back(angle);
        return;
    }

    // if the number is already here, it'll be one of the two beside where we will insert it. 
    index = binary_search(angle, point_list);
    if (abs(angle - point_list[index])<CLOSE_ENOUGH || abs(angle - point_list[index-1])<CLOSE_ENOUGH){
        // too close! 
        return;
    }
    point_list.insert(std::upper_bound(point_list.begin(), point_list.end(), angle), angle);

}

void NormCalc::test_print(){
    
    std::cout <<"Phi ";
    for(int i=0; i<phi_points.size();i++){
        std::cout << phi_points[i] << ", ";
    }
    std::cout<<std::endl;

    std::cout <<"Theta ";
    for(int i=0; i<theta_points.size();i++){
        std::cout << theta_points[i] << ", ";
    }
    std::cout << std::endl;

    std::cout << theta_points.size() <<" thetas and "<<phi_points.size() << " phis " <<std::endl;

    std::cout<<std::endl;
    std::cout<<std::endl;

    std::cout <<"norm x time"<<std::endl;
    for (int i=0; i<norm_x.size(); i++){
        for(int j=0; j<norm_x[i].size(); j++){
            std::cout << norm_x[i][j] << ", ";
        }
        std::cout << std::endl;
    }
}

double NormCalc::pmt_height(){
    return c;
}

double NormCalc::biliner_inerpolation( double theta,  double phi, const std::vector<std::vector< double>>& point_vecs){
    /*
    Performs a bilinearinterpolation of one of the data arrays we have stored, which is passed as a reference 

    Implicitly requires that the vector be one of the norm_x/y/zs or the radius, otherwise it'll break

    We get the corners for the patch around the given (theta/phi) point...
    get the coordinates of those corners, and the value of the vector<vector> there

        (1,2)----(2,2)
          |        |
          |        |
        (1,1)----(2,1)

    Then carry out the math for a binlinear interpolation! 
        https://en.wikipedia.org/wiki/Bilinear_interpolation
    */
    
    bool bad = (phi<phi_points[0] || phi>phi_points[phi_points.size()-1] || theta<theta_points[0] || theta>theta_points[theta_points.size()-1]);
    if(bad){
        return std::nanf("");
    }

    int phi_upper_index = binary_search(phi, phi_points);
    int theta_upper_index = binary_search(theta, theta_points);
    int theta_lower = theta_upper_index-1;
    int phi_lower = phi_upper_index-1;

     double x1 = theta_points[theta_lower];
     double x2 = theta_points[theta_upper_index];
     double y1 = phi_points[phi_lower];
     double y2 = phi_points[phi_upper_index];

     double q11, q12, q21, q22;
    q11 = point_vecs[theta_lower][phi_lower];
    q12 = point_vecs[theta_lower][phi_upper_index];
    q21 = point_vecs[theta_upper_index][phi_lower];
    q22 = point_vecs[theta_upper_index][phi_upper_index];

     double matmul_x = q11*(y2-phi) + q12*(phi-y1); 
     double matmul_y = q21*(y2-phi) + q22*(phi-y1);

    return ((x2-theta)*matmul_x + (theta-x1)*matmul_y)/((x2-x1)*(y2-y1));

}

std::vector<double> NormCalc::surface(double theta, double phi){
    /* 
        Expects theta running from 0 at the equator (xy plane) to pi at the pole (max z)
    */
    return{
        a*cos(phi)*cos(theta),
        b*sin(phi)*cos(theta),
        c*sin(theta)
    };
}

std::vector< double> NormCalc::evaluate_norm( double theta,  double phi){ 
    double norm_y_nosin = 1.0/sqrt(\
        pow(sin(theta),2) + pow((b/a)*cos(phi), 2) + pow((b/c)*tan(theta), 2)
    );

    double norm_y = norm_y_nosin*sin(phi);



    // in this case, we extract the wrong sign solution for sqrt(blah)
    if(phi<0){
        norm_y*=-1; 
    }

    double norm_z = norm_y_nosin*(b/c)*tan(theta);
    double norm_x = norm_y_nosin*(b/a)*cos(phi);
    
    double mag = pow(pow(norm_x,2) + pow(norm_y,2) + pow(norm_z, 2), 0.5);

    norm_x/=mag;
    norm_y/=mag;
    norm_z/=mag; 

    return {norm_x, norm_y, norm_z};

}
