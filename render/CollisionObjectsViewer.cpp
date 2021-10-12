#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCylinderSource.h>
#include <vtkSphereSource.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkLine.h>
#include <vtkPoints.h>
#include <vtkOrientationMarkerWidget.h>
#include <vtkAxesActor.h>
#include <vtkHexahedron.h>
#include <vtkUnstructuredGrid.h>
#include <vtkDataSetMapper.h>
#include <vtkDiskSource.h>
#include <vtkFeatureEdges.h>
#include <vtkCubeSource.h>
#include <iostream>


#include <array>
#include "CollisionObjectsViewer.hpp"


vtkNew<vtkNamedColors> colors;
vtkNew<vtkRenderer> renderer;

std::vector<std::string> exmaple = {"Vec3[x:-2,y:-2,z:-2]","LineSegment[a: Vec3[x:-1,y:-1,z:-1], b: Vec3[x:-2,y:-2,z:-2]]","Sphere[center: Vec3[x:0,y:0,z:1], r:0.5]","Cylinder[center: Vec3[x:0,y:0, z:0], r: 1, e: 2, orientation: Quaternion[w:1,x:0,y:0,z:0]]"};
std::vector<std::string> data = {"Cylinder[center: Vec3[x:0.3745,y:0.348, z:0.85], r: 0.305, e: 1.28, orientation: Quaternion[w:1,x:0,y:0,z:0]]",
                                "Cylinder[center: Vec3[x:0.3745,y:0.348, z:0.765], r: 0.1615, e: 0.792, orientation: Quaternion[w:1,x:0,y:0,z:0]]"};
namespace SD = Serialization;

bool render(Vec3 x){
  vtkNew<vtkPoints> points;
  const double p[3] = {x.x, x.y, x.z};

  // Create the topology of the point (a vertex)
  vtkNew<vtkCellArray> vertices;
  // We need an an array of point id's for InsertNextCell.
  vtkIdType pid[1];
  pid[0] = points->InsertNextPoint(p);
  vertices->InsertNextCell(1, pid);

  // Create a polydata object
  vtkNew<vtkPolyData> point;

  // Set the points and vertices we created as the geometry and topology of the
  // polydata
  point->SetPoints(points);
  point->SetVerts(vertices);

  // Visualize
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(point);

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(colors->GetColor3d("Blue").GetData());
  actor->GetProperty()->SetPointSize(20);
  renderer->AddActor(actor);
  return true;
}

bool render(Prism y){
  std::vector<std::array<double, 3>> pointCoordinates;

  // Create the points.
  vtkNew<vtkPoints> points;

  // Create a hexahedron from the points.
  vtkNew<vtkHexahedron> hex;

  for (auto i = 0; i < 8; ++i)
  {
    pointCoordinates.push_back({{y.vertexes()[i].x, y.vertexes()[i].y, y.vertexes()[i].z}});
    points->InsertNextPoint(pointCoordinates[i].data());
    hex->GetPointIds()->SetId(i, i);
  }

  // Add the hexahedron to a cell array.
  vtkNew<vtkCellArray> hexs;
  hexs->InsertNextCell(hex);

  // Add the points and hexahedron to an unstructured grid.
  vtkNew<vtkUnstructuredGrid> uGrid;
  uGrid->SetPoints(points);
  uGrid->InsertNextCell(hex->GetCellType(), hex->GetPointIds());

  // Visualize.
  vtkNew<vtkDataSetMapper> mapper;
  mapper->SetInputData(uGrid);

  vtkNew<vtkActor> actor;
  actor->GetProperty()->SetColor(colors->GetColor3d("PeachPuff").GetData());
  actor->SetMapper(mapper);
  renderer->AddActor(actor);
  return true;
}

bool render(LineSegment x){
  double origin[3] = {x.a.x, x.a.y, x.a.z};
  double p0[3] = {x.b.x, x.b.y, x.b.z};
  vtkNew<vtkPoints> points;
  points->InsertNextPoint(origin);
  points->InsertNextPoint(p0);

  vtkNew<vtkCellArray> lines;

  vtkNew<vtkLine> line;
  line->GetPointIds()->SetId(0, 1);
  //line->GetPointIds()->SetId(1, i + 1);
  lines->InsertNextCell(line);

  vtkNew<vtkPolyData> linesPolyData;
  linesPolyData->SetPoints(points);

  linesPolyData->SetLines(lines);
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(linesPolyData);

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  renderer->AddActor(actor);

  return true;
}

bool render(Sphere y){
  vtkNew<vtkSphereSource> sphere;
  sphere->SetCenter(0.0, 0.0, 0.0);
  sphere->SetRadius(1.0);
  // Make the surface smooth.
  sphere->SetPhiResolution(100);
  sphere->SetThetaResolution(100);
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(sphere->GetOutputPort());
  vtkNew<vtkActor> actor;

  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(colors->GetColor4d("Yellow").GetData());

  vtkNew<vtkTransform> transform;
  transform->Identity();
  transform->Scale(y.r,y.r,y.r);
  transform->Translate(y.center.x,y.center.y,y.center.z);
  actor->SetUserTransform(transform);
  //
  renderer->AddActor(actor);
  return true;
}

bool render(Cylinder y){
  vtkNew<vtkCylinderSource> cylinder;
  cylinder->SetResolution(32);
  cylinder->SetCapping(false);
  cylinder->SetHeight(y.e/2);
  cylinder->SetRadius(y.r);
  vtkNew<vtkPolyDataMapper> cylinderMapper;
  cylinderMapper->SetInputConnection(cylinder->GetOutputPort());
  vtkNew<vtkActor> cylinderActor;

  cylinderActor->SetMapper(cylinderMapper);
  cylinderActor->GetProperty()->SetColor(colors->GetColor4d("Tomato").GetData());

  vtkNew<vtkTransform> transform;
  transform->Identity();
  
  transform->Translate(y.center.x,y.center.y,y.center.z);
  transform->RotateX(90);

  transform->RotateWXYZ(y.orientation.w,y.orientation.x,y.orientation.y,y.orientation.z);
  cylinderActor->SetUserTransform(transform);
  //
  renderer->AddActor(cylinderActor);
  return true;
}

Intersectable render(SD::GeomResult res){
  if (has<SD::ErrorType>(res)) {
  } else if (has<Vec3>(res)) {
    render(get<Vec3>(res));
    return get<Vec3>(res);
  } else if (has<LineSegment>(res)) {
    render(get<LineSegment>(res));
    return get<LineSegment>(res);
  } else if (has<Prism>(res)) {
    render(get<Prism>(res));
    return get<Prism>(res);
  } else if (has<Sphere>(res)) {
    render(get<Sphere>(res));
    return get<Sphere>(res);
  } else if (has<Cylinder>(res)) {
    render(get<Cylinder>(res));
    return get<Cylinder>(res);
  } else {
    return get<Cylinder>(res);
  }
}

bool render_limit_box(){
  //TODO re these in from ODB
  double lim_neg[3] = {0.0,0.0,0.0};
  double lim_pos[3] = {0.749,0.696,0.53};

  vtkNew<vtkCubeSource> cube;
  cube->SetXLength(lim_pos[0]-lim_neg[0]);
  cube->SetYLength(lim_pos[1]-lim_neg[1]);
  cube->SetZLength(lim_pos[2]-lim_neg[2]);
  cube->SetCenter(0.5*(lim_pos[0]+lim_neg[0]),
                  0.5*(lim_pos[1]+lim_neg[1]),
                  0.5*(lim_pos[2]+lim_neg[2]));
  cube->Update();

  vtkNew<vtkFeatureEdges> featureEdges;
  featureEdges->SetInputConnection(cube->GetOutputPort());
  featureEdges->BoundaryEdgesOn();
  featureEdges->FeatureEdgesOff();
  featureEdges->ManifoldEdgesOff();
  featureEdges->NonManifoldEdgesOff();
  featureEdges->ColoringOn();
  featureEdges->Update();

  // Visualize
  vtkNew<vtkPolyDataMapper> edgeMapper;
  edgeMapper->SetInputConnection(featureEdges->GetOutputPort());
  edgeMapper->SetScalarModeToUseCellData();
  vtkNew<vtkActor> edgeActor;
  edgeActor->SetMapper(edgeMapper);

  renderer->AddActor(edgeActor);
}

int main(int, char*[])
{
  // Set the background color.
  std::array<unsigned char, 4> bkg{{26, 51, 102, 255}};
  colors->SetColor("BkgColor", bkg.data());

  vector<Intersectable> ret;
  for(std::string s:data){
    Serialization::GeomResult res = Serialization::deserialize(s);
    auto intersectable = render(res);
    ret.push_back(intersectable);
  }
  render_limit_box();

  /*
   *Uses odb and creates a fake path to get the grantry collidable box
   */
  /*cm_get_experiment_database(&State::Keys::hDB, NULL);

  const auto hDB = State::Keys::hDB;
  db_find_key(hDB, 0, "/Equipment/Move/Control/Destination", &State::Keys::destination);
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Position", &State::Keys::position);*/
  //                           x   y   z    t   p
  array<float, 10> position ={0.0,0.0,0.0 ,0.0,0.0,   0.0,0.0,0.0 ,0.0,0.0};
  array<float, 10>destination ={1.0,0.0,0.0 ,0.0,0.0,   0.0,0.0,0.0 ,0.0,0.0};
  auto bufsize = 10 * sizeof(float);
  //db_get_data(hDB, State::Keys::position, position.data(), &bufsize, TID_FLOAT);
  bufsize = 10 * sizeof(float);
  //db_get_data(hDB, State::Keys::destination, destination.data(), &bufsize, TID_FLOAT);
  auto from = PathGeneration::move_point_from_array(position);
  auto to   = PathGeneration::move_point_from_array(destination);
  //auto path = PathGeneration::single_move(from, to, ret);
  auto path  = PathGeneration::generate_move({from.gantry0, to.gantry0}, from.gantry1, PathGeneration::Gantry0, PathGeneration::DimensionOrder::all_orders()[0]);
  auto pt = get<PathGeneration::MovePath>(path)[0];
  Prism ob0 = PathGeneration::point_to_optical_box(pt.gantry0, false);
  Prism ob1 = PathGeneration::point_to_optical_box(pt.gantry1, false);
  render(ob0);
  render(ob1);

  renderer->GetActiveCamera()->Yaw(180);
  renderer->GetActiveCamera()->Elevation(-45);
  //renderer->GetActiveCamera()->Azimuth(10);

  renderer->SetBackground(colors->GetColor3d("BkgColor").GetData());
  renderer->ResetCamera();
  renderer->GetActiveCamera()->Zoom(1.0);

  //Window stuff
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->SetSize(300, 300);
  renderWindow->AddRenderer(renderer);
  renderWindow->SetWindowName("Collision Model");
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkNew<vtkAxesActor> axes;
  vtkNew<vtkOrientationMarkerWidget> widget;
  double rgba[4]{0.0, 0.0, 0.0, 0.0};
  colors->GetColor("Carrot", rgba);
  widget->SetOutlineColor(rgba[0], rgba[1], rgba[2]);
  widget->SetOrientationMarker(axes);
  widget->SetInteractor(renderWindowInteractor);
  widget->SetViewport(0.0, 0.0, 0.4, 0.4);
  widget->SetEnabled(1);
  widget->InteractiveOn();

  
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}