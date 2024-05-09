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
    std::ifstream file(filename);

    // load the data into a temporary  `data` vector
    std::vector<std::vector< double>> data; 
    if(file.is_open()){
        std::string line;
        while(std::getline(file, line)){
            std::vector< double> row;
            std::stringstream ss(line); 
             double value;

            while (ss>> value){
                row.push_back(value);
            }
            data.push_back(row);
        }
        file.close();
    }else{
        std::runtime_error("Could not find file for norm-calculations");
    }

     double this_phi;
     double this_theta;
    // first pass! We sort out the unique thetas and phis 
    for(int i=0; i<data.size(); i++){
        this_theta = data[i][0];
        this_phi = data[i][1];

        add_angle(this_theta, theta_points);
        add_angle(this_phi, phi_points);
        
    }

    // now we instantiate the data vectors 
    for(int i=0; i<theta_points.size(); i++){
        norm_x.emplace_back(
            phi_points.size(), 0.0
        );
        norm_y.emplace_back(
            phi_points.size(), 0.0
        );
        norm_z.emplace_back(
            phi_points.size(), 0.0
        );
        radius.emplace_back(
            phi_points.size(), 0.0
        );
    }

    // now we actually assign everything as part of a second pass 
    int theta_index = 0;
    int phi_index = 0;

     double this_x;
     double this_y;
     double this_z;

     double theta_width = theta_points[1]- theta_points[0];
     double phi_width = phi_points[1]- phi_points[0];
    for(int i=0; i<data.size(); i++){
        this_theta = data[i][0];
        this_phi = data[i][1];

        theta_index = binary_search(this_theta+0.5*theta_width, theta_points)-1;
        phi_index = binary_search(this_phi+0.5*phi_width, phi_points)-1;

        radius[theta_index][phi_index] = data[i][2];
        norm_x[theta_index][phi_index] = data[i][3];
        norm_y[theta_index][phi_index] = data[i][4];
        norm_z[theta_index][phi_index] = data[i][5];

    }

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

 double NormCalc::evaluate_radius( double theta,  double phi){
    return biliner_inerpolation(theta, phi, radius);
}

std::vector< double> NormCalc::evaluate_norm( double theta,  double phi){ 
    // not the most efficient, but the code is neater 
    return {
        biliner_inerpolation(theta, phi, norm_x),
        biliner_inerpolation(theta, phi, norm_y),
        biliner_inerpolation(theta, phi, norm_z),
    };
}
