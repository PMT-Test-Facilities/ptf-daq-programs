// TPathCalculator from March 16, 2017, before PMT collision avoidance has been updated.
// Created for debugging

#include "TPathCalculator.hxx"
#include "midas.h" // for logging

//----------------------------------------------------------
TPathCalculator::TPathCalculator(){
//----------------------------------------------------------

    fObject1 = new Polygon();
    fObject2 = new Polygon();
    fOpticalBox0 = new Polygon();
    fOpticalBox1 = new Polygon();
    fPMTpoly0 = new Polygon();
    fPMTpoly1 = new Polygon();

}

//----------------------------------------------------------
void TPathCalculator::InitialiseGantries(std::vector<std::pair<double, double> > points1, std::vector<std::pair<double, double> > points2){
//----------------------------------------------------------

    boost::geometry::clear(*fObject1);
    boost::geometry::clear(*fObject2);
    
    for(int i = 0; i < (int)points1.size(); ++i){
      //std::cout << " Gantry 1 position  = " << points1[i].first << " " << points1[i].second << std::endl;
      cm_msg(MINFO,"InitialiseGantries"," Gantry 1 position  = %.3lf  %.3lf", points1[i].first, points1[i].second);
      boost::geometry::append(*fObject1, boost::geometry::make<Point>(points1[i].first, points1[i].second));
    }
    for(int i = 0; i < (int)points2.size(); ++i){
      //std::cout << " Gantry 2 position  = " << points2[i].first << " " << points2[i].second << std::endl;
      cm_msg(MINFO,"InitialiseGantries"," Gantry 2 position  = %.3lf  %.3lf", points2[i].first, points2[i].second);
      boost::geometry::append(*fObject2, boost::geometry::make<Point>(points2[i].first, points2[i].second));
    }

    boost::geometry::correct(*fObject1);
    boost::geometry::correct(*fObject2);
}

//----------------------------------------------------------
void TPathCalculator::InitialisePMT(std::vector< std::pair<double, double> > points){
//----------------------------------------------------------
// Rika: Added Mar 14, 2017 to improve collision avoidance for PMT
// TODO: Later update to take std::vector< std::vector< std::pair<double,double> > > as argument;
// i.e. a vector of polygons;
// and loop through the vector to build polygons.

    boost::geometry::clear(*fPMTpoly0);

    for(int i = 0; i < (int)points.size(); ++i){
      cm_msg(MINFO,"InitialisePMT"," PMT position  = %.3lf  %.3lf", points[i].first, points[i].second);
      boost::geometry::append(*fPMTpoly0, boost::geometry::make<Point>(points[i].first, points[i].second));
    }

    boost::geometry::correct(*fPMTpoly0);

}


//----------------------------------------------------------
bool TPathCalculator::CalculatePath(std::pair<double, double> start, std::pair<double, double> end, std::vector<std::pair<double, double> >& path, std::pair<bool, bool> tank_height_start, std::pair<bool, bool> tank_height_end){
//----------------------------------------------------------


  cm_msg(MINFO,"CalculatePath","Calculating path from %.3lf in X to %.3lf and from %.3lf in Y to %.3lf", start.first, end.first, start.second, end.second);

    bool isObject1 = 1;
    bool validEndPoint = 0;
    bool goodPath = 0;

    isObject1 = boost::geometry::within(boost::geometry::make<Point>(start.first,start.second), *fObject1);

    //debug
    //std::cout << "This is/isn't object 1 ? = " << isObject1 << std::endl;

    if(isObject1){
      validEndPoint = !boost::geometry::within(boost::geometry::make<Point>(end.first,end.second), *fObject2);
    }
    else{
      validEndPoint = !boost::geometry::within(boost::geometry::make<Point>(end.first,end.second), *fObject1);
    }
 
    if(!validEndPoint){
      cm_msg(MERROR,"CalculatePath","Endpoint is within the other gantry"); //outside the water tank");
      return false;
    }

    // Create path doing X steps first
    CreatePath(start.first, start.second, end.first, end.second, path, 1, false);
   
    // Check path does not collide with other objects
    goodPath = CheckPathForCollisions(isObject1, path, tank_height_start, tank_height_end);

    if(goodPath) {
      //TF: the path is fine, let's remove the intermediate steps
      //    and only keep the corners where the direction changes!
      CreatePath(start.first, start.second, end.first, end.second, path, 1, true);
      return true; 
    }
    else{
      CreatePath(start.first, start.second, end.first, end.second, path, 0, false);
      goodPath = CheckPathForCollisions(isObject1, path, tank_height_start, tank_height_end);
    }
    if(goodPath) {
      CreatePath(start.first, start.second, end.first, end.second, path, 0, true);
      return true;
    }
    return false;
}

//----------------------------------------------------------
bool TPathCalculator::CheckPathForCollisions(bool isObject1, std::vector<std::pair<double, double> >& path, std::pair<bool, bool> tank_height_start, std::pair<bool, bool> tank_height_end){
//----------------------------------------------------------

  //tank boundaries
  //TODO the center pos, maxval, and Ydifmax should be called in from feMove, not defined here
  double Xdif;
  double Ydif;
  double Xcent = 0.325;
  double Ycent = 0.335;
  double Maxval = 0.61; // radius of tank ~0.61
  double Ydifmax = 0.53; // max/min from center where PMT holders sit

    if(isObject1){

        std::vector<Point> objectPoints = fObject1->outer(); // For Gantry 0
	
        for(int iter = 0; iter < (int)path.size(); ++iter){
	    bool collides_tank = 0;
            bool collides_gant = 0;
	       
            for(std::vector<Point>::size_type i = 0; i < objectPoints.size(); ++i){
                double newX = boost::geometry::get<0>(objectPoints[i]) + path[iter].first;
                double newY = boost::geometry::get<1>(objectPoints[i]) + path[iter].second;

		//std::cout<< "TF DEBUG " << iter << " " << i << " " << newX << " " << newY << " " << path[iter].first << " " << path[iter].second << std::endl;
                boost::geometry::set<0>(objectPoints[i], newX);
                boost::geometry::set<1>(objectPoints[i], newY);

                // Determine if box moves outside of tank limits. If gantry doesn't move above tank first and if starting height is not outside of tank, must check limits
		if(!tank_height_end.first){
		    if(!tank_height_start.first){
                        if(newX >= Xcent) Xdif = newX-Xcent;
                        else Xdif = Xcent-newX;
                        if(newY >= Ycent) Ydif = newY-Ycent;
                        else Ydif = Ycent-newY;
                        if((sqrt(Xdif*Xdif+Ydif*Ydif) > Maxval) || (Ydifmax < Ydif)){
 			    collides_tank = true;
			}
		    }
		}
            }
	    //TF: BUG found in above : fObject1 IS altered by CheckPathForCollisions!
	    //Do check on a copy, because NEED fObject1 for creating path through Y first
	    // starting from ORIGINAL fObject1, not altered one!
	    //Q: need to clear/del this first/later?
	    Polygon *fTestObject = new Polygon();

	    boost::geometry::assign_points(*fTestObject,objectPoints);
	    boost::geometry::correct(*fTestObject);
	    collides_gant = boost::geometry::intersects(*fTestObject, *fObject2);

            if(collides_gant || collides_tank) return false;
        }
        return true;
    }
    else{
 
        std::vector<Point> objectPoints = fObject2->outer();

        for(int iter = 0; iter < (int)path.size(); ++iter){
	    bool collides_tank = 0;
            bool collides_gant = 0;        

            for(std::vector<Point>::size_type i = 0; i < objectPoints.size(); ++i){
                double newX = boost::geometry::get<0>(objectPoints[i])+path[iter].first;
                double newY = boost::geometry::get<1>(objectPoints[i])+path[iter].second;

		//std::cout<< "TF DEBUG " << iter << " " << i << " " << newX << " " << newY << " " << path[iter].first << " " << path[iter].second << std::endl;
                boost::geometry::set<0>(objectPoints[i], newX);
                boost::geometry::set<1>(objectPoints[i], newY);

		// Determine if box moves outside of tank limits. If gantry doesn't move above tank first and if starting height is not outside of tank, must check limits
                if(!tank_height_end.second){
                    if(!tank_height_start.second){
                        if(newX >= Xcent) Xdif = newX-Xcent;
                        else Xdif = Xcent-newX;
                        if(newY >= Ycent) Ydif = newY-Ycent;
                        else Ydif = Ycent-newY;
                        if((sqrt(Xdif*Xdif+Ydif*Ydif) > Maxval) || (Ydifmax < Ydif)){
                            collides_tank = true;
                        }
                    }
                }
            }
	    //same BUG was here, use TestObject instead!
	    Polygon *fTestObject = new Polygon();

	    boost::geometry::assign_points(*fTestObject,objectPoints);
	    boost::geometry::correct(*fTestObject);
	    collides_gant = boost::geometry::intersects(*fObject1,*fTestObject); 
            if(collides_gant || collides_tank) return false;
        }
        return true;
    }
    return false;
}

//----------------------------------------------------------
void TPathCalculator::CreatePath(double startX, double startY, double endX, double endY, std::vector<std::pair<double, double> >& path, bool xFirst, bool simplePath){
//----------------------------------------------------------
  
  path.clear();
  
  //Only used to check for collision avoidance. This allows also to move in mm steps!! 
  double XSpeed = 0.001;
  double YSpeed = 0.001;
  
  double deltaX = endX - startX;
  double deltaY = endY - startY;
  
  if(deltaX < 0) XSpeed*= -1;
  if(deltaY < 0) YSpeed*= -1;
  
  //TF: use option for creating a SIMPLE path that only does the corners
  // where the path changes direction! The rest slows down the movement
  // and calls the motors more than needed.
  // NOTE: so far no intermediate many corners (not necessary?!)
  //       therefore, go to endX/endY in one go, but one after the other!
  if(simplePath){
    XSpeed = deltaX;
    YSpeed = deltaY;
  }
  
  int NStepsX = deltaX/XSpeed + 0.5;
  int NStepsY = deltaY/YSpeed + 0.5;
  
  if(xFirst){
    //std::cout << "Try X FIRST with Xspeed " << XSpeed << std::endl;
    for(int i = 0; i < NStepsX; ++i){
      path.push_back(std::make_pair<double, double>(XSpeed, 0));
    }
    for(int i = 0; i < NStepsY; ++i){
      path.push_back(std::make_pair<double, double>(0, YSpeed));
    }
  }
  else{
    //std::cout << "Try Y FIRST with Yspeed " << YSpeed << std::endl;
    for(int i = 0; i < NStepsY; ++i){
      path.push_back(std::make_pair<double, double>(0, YSpeed));
    }
    for(int i = 0; i < NStepsX; ++i){
      path.push_back(std::make_pair<double, double>(XSpeed, 0));
    }
  }
  //std::cout << "TF debug: path size " << (int)path.size() << std::endl;
  return;
}

