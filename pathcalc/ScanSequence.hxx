#ifndef ScanSequence_H
#define ScanSequence_H

#include "midas.h"    
#include "experim_new.h"  
#include <vector>
#include "norm_calc/norm_eval.h"


class ScanSequence {
  
public:
  ScanSequence();
  
  // Input dimensions of the water tank
  void Init(SCAN_SETTINGS &fs_in, float* gantryLim);
  
  //
  int GeneratePath(std::vector<std::vector<double> > &points_in);
  
  // Helper functions
  int GetMeasTime(){ return fs.meas_time; };
  
  
private:
  
  /* 7 classes of sequence: in gantry coordinates : Cylinder, Rectangular, Alignment, PassBy, Manual, Demo
   *                        in PMT surface coordinates (cylindrical coo system, then transform to gantry):
   *                                                Surface: 3 subclasses
   *                                                Loops in phi, fix r and h
   *                                                Loops in h, fix phi and r: big circles
   *                                                Fixed point
   */
  int CylinderPath(std::vector<std::vector<double> > &points);
  int RectangularPath(std::vector<std::vector<double> > &points);
  int PMTSurfacePath(std::vector<std::vector<double> > &points);
  int AlignmentPath(std::vector<std::vector<double> > &points);
  int PassByPath(std::vector<std::vector<double> > &points);
  int ManualPath(std::vector<std::vector<double> > &points);
  int TankAvoidancePath(std::vector<std::vector<double> > &points);
  int DemoPath(std::vector<std::vector<double> > &points);
  int TiltPath(std::vector<std::vector<double> > &points);//Anubhav's edit
  int SpinPath(std::vector<std::vector<double>> &points);
  int PatchPath(std::vector<std::vector<double>> &points);

  bool is_destinationvalid(const std::vector<double>& l); //Anubhav's edit
  
  void ReturnToBase(std::vector<std::vector<double> > &points);

  // ToDo : the above ones can call the Reflectivity function, for a reflectivity sequence for each point
  // Probably very similar to acceptance with two heads....both pointing at the PMT center...


  

  // Probably overload this: first version: getting a point in space, normal to surface 
  // goes through that point and the center of the local sphere.
  void CalculateNormIncidence(std::vector<double> &point, double center_x, double center_y, double beam, double incidence, double azimuth, gantry_t gantry);
  // second version: if the point in space by itself depends on the rotation and tilt.
  //void CalculateNormIncidence(double &rotation, double &tilt, double x_pos, double y_pos, double z_pos);

  // Separate function for coordinate transforms?
  void TransformPMTsurfaceToGantry(void);
  
  void TransformGantryToPMTsurface(void);
  
  //globals from feScan, which I need here (are global anyway, otherwise pass by ref Init?)
  // struct with settings from obd, created by experim.h 
  SCAN_SETTINGS fs;
  
  // Position of limits, according to feMove front-end.
  float  *gGantryLimits;

  //Count number of non-zero points: could be size of vector (TODO)
  // if we dynamically push_back
  // NOW: do a resize beforehand for more efficient mem usage..
  int point_num;
  float z_max_value;

  NormCalc pmt_norm;
  
};

#endif

