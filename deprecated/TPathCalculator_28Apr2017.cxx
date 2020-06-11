#include "TPathCalculator.hxx"
#include "midas.h" // for logging


//----------------------------------------------------------
TPathCalculator::TPathCalculator(double tankCentreXPos, double tankCentreYPos, double tankRadius, double tankPMTholderRadius){
//----------------------------------------------------------

    fObject0 = new BoostPolygon();
    fObject1 = new BoostPolygon();
    fOpticalBox0_lo = new BoostPolygon();
    fOpticalBox0_up = new BoostPolygon();
    fOpticalBox1_lo = new BoostPolygon();
    fOpticalBox1_up = new BoostPolygon();

    //tank boundaries
    Xcent = tankCentreXPos;
    Ycent = tankCentreYPos;
    tRadius = tankRadius;
    tPMTholder = tankPMTholderRadius; // max/min from center where PMT holders sit


}

//----------------------------------------------------------
void TPathCalculator::InitialiseGantries(XYPolygon points0, XYPolygon points1){
//----------------------------------------------------------

    //cm_msg(MINFO,"InitialiseGantries"," Gantry 0: ");
    CreatePolygon(fObject0, points0);
    //cm_msg(MINFO,"InitialiseGantries"," Gantry 1: ");
    CreatePolygon(fObject1, points1);

}

//----------------------------------------------------------
void TPathCalculator::InitialiseOpticalBoxes(XYPolygon points0_lo, XYPolygon points0_up, XYPolygon points1_lo, XYPolygon points1_up, int z0_lo, int z0_up, int z1_lo, int z1_up){
//----------------------------------------------------------

    height_OpticalBox0_lo = z0_lo;
    height_OpticalBox0_up = z0_up;
    height_OpticalBox1_lo = z1_lo;
    height_OpticalBox1_up = z1_up;

    //cm_msg(MINFO,"InitialiseOpticalBoxes"," Optical Box 0: z height = %i cm below PMT cover top.", height_OpticalBox0_lo);
    CreatePolygon(fOpticalBox0_lo, points0_lo);
    CreatePolygon(fOpticalBox0_up, points0_up);
    //cm_msg(MINFO,"InitialiseOpticalBoxes"," Optical Box 1: z height = %i cm below PMT cover top.", height_OpticalBox1_lo);
    CreatePolygon(fOpticalBox1_lo, points1_lo);
    CreatePolygon(fOpticalBox1_up, points1_up);

}


// Rika (14Mar2017): Added to improve collision avoidance for PMT
// (21Mar2017): updated to take std::vector< XYPolygon > as argument;
// i.e. a vector of polygons;
// and loop through the vector to build polygons.
//----------------------------------------------------------
void TPathCalculator::InitialisePMT(std::vector<XYPolygon> pmt0, std::vector<XYPolygon> pmt1){
//----------------------------------------------------------

    for(int i=0; i<pmt0.size(); ++i){
	if(i>=fPMTmultiPoly0.size()){
	    fPMTmultiPoly0.push_back(new BoostPolygon());
	}
	CreatePolygon(fPMTmultiPoly0.at(i), pmt0.at(i));
    }
    for(int j=0; j<pmt1.size(); ++j){
	if(j>=fPMTmultiPoly1.size()){
	    fPMTmultiPoly1.push_back(new BoostPolygon());
	}
	CreatePolygon(fPMTmultiPoly1.at(j), pmt1.at(j));
    }

}

// Rika (16Mar2017):
// New CalculatePath function to complement the change in arguments passed to CheckPathForCollisions.
// PMT collision avoidance added.
//----------------------------------------------------------
bool TPathCalculator::CalculatePath(XYPoint start, XYPoint end, std::vector<XYPoint>& path, std::pair<bool, bool> tank_height_start, std::pair<bool, bool> tank_height_end){
//----------------------------------------------------------

  cm_msg(MINFO,"CalculatePath","Calculating path from %.3lf in X to %.3lf and from %.3lf in Y to %.3lf", start.first, end.first, start.second, end.second);

    bool isObject0 = 1;
    bool validEndPoint = 0;
    bool goodPath = 0;

    isObject0 = boost::geometry::within(boost::geometry::make<BoostPoint>(start.first,start.second), *fObject0);

    if(isObject0){
      validEndPoint = !boost::geometry::within(boost::geometry::make<BoostPoint>(end.first,end.second), *fObject1);
    }
    else{
      validEndPoint = !boost::geometry::within(boost::geometry::make<BoostPoint>(end.first,end.second), *fObject0);
    }
    
    if(!validEndPoint){
      //cm_msg(MERROR,"CalculatePath","Endpoint is within the other gantry"); //For debugging
      return false;
    }

    if(isObject0){
        // Create path doing X steps first
        CreatePath(start.first, start.second, end.first, end.second, path, 1, false);

        // Check path does not collide with other objects

	goodPath = CheckPathForCollisions(fObject0, fObject1, path, tank_height_start.first, tank_height_end.first);
	/**
	if(!goodPath){ // FOR DEBUGGING
	    cm_msg(MINFO,"CalculatePath"," Gantry 0 moving in x first will collide with Gantry 1 or with tank.");
	}
	**/

	// Check PMT collision only if height_OpticalBox >= 0, where 0 represents top of PMT
	// and 1 represents 1cm below the top of PMT, 2 represents 2cm, etc.
	if(goodPath && height_OpticalBox0_lo >= 0){
            goodPath = CheckPathForCollisions(fOpticalBox0_lo, fPMTmultiPoly0.at(height_OpticalBox0_lo), path, tank_height_start.first, tank_height_end.first);
	}
	if(goodPath && height_OpticalBox0_up >= 0){
            goodPath = CheckPathForCollisions(fOpticalBox0_up, fPMTmultiPoly0.at(height_OpticalBox0_up), path, tank_height_start.first, tank_height_end.first);
	}
	
        /**
        if(!goodPath){ // FOR DEBUGGING
	    cm_msg(MINFO,"CalculatePath"," Gantry 0 moving in x first will collide.");
	}
	**/

        if(goodPath) {
          //TF: the path is fine, let's remove the intermediate steps
          //    and only keep the corners where the direction changes!
          CreatePath(start.first, start.second, end.first, end.second, path, 1, true);
          return true; 
        }
        else{
          CreatePath(start.first, start.second, end.first, end.second, path, 0, false);

	  goodPath = CheckPathForCollisions(fObject0, fObject1, path, tank_height_start.first, tank_height_end.first);

	  /**
	  if(!goodPath){ // FOR DEBUGGING
	      cm_msg(MINFO,"CalculatePath"," Gantry 0 moving in y first will collide with Gantry 1 or with tank.");
	  }
	  **/

	  if(goodPath && height_OpticalBox0_lo >= 0){
            goodPath = CheckPathForCollisions(fOpticalBox0_lo, fPMTmultiPoly0.at(height_OpticalBox0_lo), path, tank_height_start.first, tank_height_end.first);
	  }
	  if(goodPath && height_OpticalBox0_up >= 0){
            goodPath = CheckPathForCollisions(fOpticalBox0_up, fPMTmultiPoly0.at(height_OpticalBox0_up), path, tank_height_start.first, tank_height_end.first);
	  }
	}
	/**
	if(!goodPath){ // FOR DEBUGGING
	    cm_msg(MINFO,"CalculatePath"," Gantry 0 moving in y first will collide.");
	}
	**/

        if(goodPath) {
          CreatePath(start.first, start.second, end.first, end.second, path, 0, true);
         return true;
        }
        return false;

    }
    else{ // GANTRY 1

        // Create path doing X steps first
        CreatePath(start.first, start.second, end.first, end.second, path, 1, false);

	goodPath = CheckPathForCollisions(fObject1, fObject0, path, tank_height_start.second, tank_height_end.second);

	/**
	if(!goodPath){ // FOR DEBUGGING
	    cm_msg(MINFO,"CalculatePath"," Gantry 1 moving in x first will collide with Gantry 0 or with tank.");
	}
	**/

	// Check PMT collision only if height_OpticalBox >= 0, where 0 represents top of PMT
	// and 1 represents 1cm below the top of PMT, 2 represents 2cm, etc.
	if(goodPath && height_OpticalBox1_lo >= 0){
            goodPath = CheckPathForCollisions(fOpticalBox1_lo, fPMTmultiPoly1.at(height_OpticalBox1_lo), path, tank_height_start.second, tank_height_end.second);
	}
	if(goodPath && height_OpticalBox1_up >= 0){
            goodPath = CheckPathForCollisions(fOpticalBox1_up, fPMTmultiPoly1.at(height_OpticalBox1_up), path, tank_height_start.second , tank_height_end.second);
	}
	/**
	if(!goodPath){ // FOR DEBUGGING
	    cm_msg(MINFO,"CalculatePath"," Gantry 1 moving in x first will collide.");
	}
	**/

        if(goodPath) {
          //TF: the path is fine, let's remove the intermediate steps
          //    and only keep the corners where the direction changes!
          CreatePath(start.first, start.second, end.first, end.second, path, 1, true);
          return true; 
        }
        else{
          CreatePath(start.first, start.second, end.first, end.second, path, 0, false);

	  goodPath = CheckPathForCollisions(fObject1, fObject0, path, tank_height_start.second, tank_height_end.second);

	  /**
	  if(!goodPath){ // FOR DEBUGGING
	      cm_msg(MINFO,"CalculatePath"," Gantry 1 moving in y first will collide with Gantry 0 or with tank.");
	  }
	  **/

	  if(goodPath && height_OpticalBox1_lo >= 0){
              goodPath = CheckPathForCollisions(fOpticalBox1_lo, fPMTmultiPoly1.at(height_OpticalBox1_lo), path, tank_height_start.second, tank_height_end.second);
	  }
	  if(goodPath && height_OpticalBox1_up >= 0){
              goodPath = CheckPathForCollisions(fOpticalBox1_up, fPMTmultiPoly1.at(height_OpticalBox1_up), path, tank_height_start.second, tank_height_end.second);
	  }
        }
	/**
	if(!goodPath){ // FOR DEBUGGING
	    cm_msg(MINFO,"CalculatePath"," Gantry 1 moving in y first will collide.");
	}
	**/
        if(goodPath) {
          CreatePath(start.first, start.second, end.first, end.second, path, 0, true);
         return true;
        }
        return false;
    }
}

//----------------------------------------------------------
bool TPathCalculator::CheckDestination(XYPolygon points0, XYPolygon points1, bool tank_height_end){
//----------------------------------------------------------

    BoostPolygon obj0;
    BoostPolygon obj1;

    std::vector<XYPoint> stationaryPath;

    CreatePolygon(&obj0, points0);
    CreatePolygon(&obj1, points1);
    stationaryPath.push_back(std::make_pair(0.0, 0.0));

    return CheckPathForCollisions(&obj0, &obj1, stationaryPath, tank_height_end, tank_height_end);

    /**
    bool validDestination = true;

    validDestination = !boost::geometry::intersects(obj0, obj1);
    if(!validDestination){
	return validDestination;
    }

    for(int i=0; i<points0.size(); ++i){
      validDestination = !boost::geometry::within(boost::geometry::make<BoostPoint>(points0[i].first,points0[i].second), obj1);

      cm_msg(MINFO, "CheckDestination", " Checking destination");
      cm_msg(MINFO,"CheckDestination"," Position = %.3lf  %.3lf", points0[i].first, points0[i].second);

      if(!validDestination){
	  return validDestination;
      }

    }
    return validDestination;
    **/
}

//----------------------------------------------------------
void TPathCalculator::CreatePath(double startX, double startY, double endX, double endY, std::vector<XYPoint>& path, bool xFirst, bool simplePath){
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

// Rika (16Mar2017): Created a generic function to be used in the Initialise functions
//----------------------------------------------------------
void TPathCalculator::CreatePolygon(BoostPolygon* object, XYPolygon points){
//----------------------------------------------------------

    boost::geometry::clear(*object);
    
    for(int i = 0; i < (int)points.size(); ++i){
	/**
        if(points.size()<5){ // To ensure that it doesn't print repeatedly for PMT initialization
	    cm_msg(MINFO,"CreatePolygon"," Position = %.3lf  %.3lf", points[i].first, points[i].second); // Rika: Do we need this?
	}
	**/
	boost::geometry::append(*object, boost::geometry::make<BoostPoint>(points[i].first, points[i].second));
    }

    boost::geometry::correct(*object);

}

// Rika (16Mar2017): CheckPathForCollisions updated to take pointers to the two objects to be compared
// (either gantry-gantry or opticalBox-PMT).
// Rika (23Mar2017): Updated function to take in only one tank_height_* variable for z position information
// corresponding to objectMoving only (previously a vector was passed, containing info for both gantries).
//----------------------------------------------------------
bool TPathCalculator::CheckPathForCollisions(BoostPolygon* objectMoving, BoostPolygon* objectStationary, std::vector<XYPoint>& path, bool tank_height_start, bool tank_height_end){
//----------------------------------------------------------

  double Xdif;
  double Ydif;

  double Maxval = tRadius;
  double Ydifmax = tPMTholder; // max/min from center where PMT holders sit

  std::vector<BoostPoint> objectPoints = objectMoving->outer(); 

  // Collision check agaisnt tank:
  for(int iter = 0; iter < (int)path.size(); ++iter){
      bool collides_tank = 0;
      bool collides_obj = 0;
	       
      for(std::vector<BoostPoint>::size_type i = 0; i < objectPoints.size(); ++i){
	  double newX = boost::geometry::get<0>(objectPoints[i]) + path[iter].first;
	  double newY = boost::geometry::get<1>(objectPoints[i]) + path[iter].second;

	  // std::cout<< "TF DEBUG " << iter << " " << i << " " << newX << " " << newY << " " << path[iter].first << " " << path[iter].second << std::endl;
	  boost::geometry::set<0>(objectPoints[i], newX);
	  boost::geometry::set<1>(objectPoints[i], newY);

	  // Determine if box moves outside of tank limits. If gantry doesn't move above tank first and if starting height is not outside of tank, must check limits
	  if(!tank_height_end){
	      if(!tank_height_start){
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
      BoostPolygon fTestObject;

      boost::geometry::assign_points(fTestObject,objectPoints);
      boost::geometry::correct(fTestObject);
      collides_obj = boost::geometry::intersects(fTestObject, *objectStationary);

      if(collides_obj || collides_tank) return false;
  }
  return true;
}
