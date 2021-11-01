#include <vtkCamera.h>
#include <vtkCylinderSource.h>
#include <vtkSphereSource.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkInteractorStyleTrackballCamera.h>
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
#include <vtkCommand.h>
#include <vtkWindowToImageFilter.h>
#include <vtkSmartPointer.h>
#include <vtkPNGWriter.h>
#include <iostream>
#include <vtkObjectFactory.h>
#include <math.h>

#include "col.hpp"
#include "has.hpp"
#include "measurements.hpp"

#include <array>
#include "CollisionObjectsViewer.hpp"

//####### feMove params #######
// maximum string length for collidable objects
#define COLLIDE_STR_MAXLEN 256
// number of possible collision objects
#define COLLIDE_STR_NUM 16

bool record;
bool live;

vtkNamedColors* colors;
vtkRenderer* renderer;
vtkRenderWindow *renderWindow;

namespace SD = Serialization;


// Define interaction style
class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
  vector<Intersectable> collidable;
  std::vector<vtkSmartPointer<vtkActor>> gantry_actors;//ob1 r1 rg1, ob2, r2 rg2
  std::vector<vtkActor*> line_segments;
  static KeyPressInteractorStyle* New();
  vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);

  virtual void OnKeyPress() override
  {
    // Get the keypress
    vtkRenderWindowInteractor* rwi = this->Interactor;
    std::string key = rwi->GetKeySym();

    // Output the key that was pressed
    std::cout << "Pressed " << key << std::endl;

    // Handle an arrow key
    if (key == "Up")
    {
      std::cout << "The up arrow was pressed." << std::endl;
      array<float, 10> position, destination;
      position = {0.0,0.0,0.0,0.0,0.0,0.749,0.696,0.0,0.0,0.0};//recoment to
      destination = {0.5,0.5,0.5,0.0,0.0,0.749,0.696,0.0,0.0,0.0};
      INT bufsize = 10 * sizeof(float);
      db_get_data(State::Keys::hDB, State::Keys::position, position.data(), &bufsize, TID_FLOAT);
      bufsize = 10 * sizeof(float);
      db_get_data(State::Keys::hDB, State::Keys::destination, destination.data(), &bufsize, TID_FLOAT);
      
      auto start = PathGeneration::move_point_from_array(position);
      auto end   = PathGeneration::move_point_from_array(destination);

      auto _path = PathGeneration::single_move(start, end, this->collidable);

      for(auto line_actor:line_segments)
        renderer->RemoveActor(line_actor);
      line_segments = std::vector<vtkActor*>();
  
      if (has<PathGeneration::ErrorType>(_path)) {
        cm_msg(MERROR, "feMove:start_move", "Could not generate path. Error message: \"%s\".", PathGeneration::error_message(get<PathGeneration::ErrorType>(_path)).c_str());
        LineSegment l = {start.gantry0.position,
                         end.gantry0.position};
        line_segments.push_back(render(l));
        return;
      }
      PathGeneration::MovePath path = get<PathGeneration::MovePath>(_path);


      for(int i = 1; i < path.size(); i++){
        if(path[i-1].gantry0.position == end.gantry0.position)
          break;
        LineSegment l = {path[i-1].gantry0.position,
                         path[i].gantry0.position};
        line_segments.push_back(render(l));
      }

      vtkTransform* transform = vtkTransform::New();
      transform->Identity();
      
      std::cout << destination[1]<<" "<<destination[0]<<" "<<destination[2]<<std::endl;
      transform->Translate(destination[1],destination[0],destination[2]);
      this->gantry_actors[0]->SetUserTransform(transform);
      this->gantry_actors[0]->Modified();

      this->gantry_actors[1]->SetUserTransform(transform);
      this->gantry_actors[1]->Modified();

      this->gantry_actors[2]->SetUserTransform(transform);
      this->gantry_actors[2]->Modified();
      renderWindow->Render();
    }

    // Handle a "normal" key
    if (key == "a")
    {
      std::cout << "The a key was pressed." << std::endl;
      array<float, 10> position, destination;
      //destination = {0.5,0.5,0.5,0.0,0.0,0.749,0.696,0.0,0.0,0.0};
      INT bufsize = 10 * sizeof(float);
      db_get_data(State::Keys::hDB, State::Keys::position, position.data(), &bufsize, TID_FLOAT);
      bufsize = 10 * sizeof(float);
      db_get_data(State::Keys::hDB, State::Keys::destination, destination.data(), &bufsize, TID_FLOAT);
      
      PathGeneration::MovePoint start = PathGeneration::move_point_from_array(destination);

      vtkTransform* transform = vtkTransform::New();
      transform->Identity();
      
      std::cout << destination[1]<<" "<<destination[0]<<" "<<destination[2]<<std::endl;
      transform->Translate(destination[1],destination[0],destination[2]);
      this->gantry_actors[0]->SetUserTransform(transform);
      this->gantry_actors[0]->Modified();

      this->gantry_actors[1]->SetUserTransform(transform);
      this->gantry_actors[1]->Modified();

      this->gantry_actors[2]->SetUserTransform(transform);
      this->gantry_actors[2]->Modified();
      renderWindow->Render();
      if(intersect(point_to_optical_box(start.gantry0, false), this->collidable))
        std::cout << "Collistion" << std::endl;
    }

    // Forward events
    vtkInteractorStyleTrackballCamera::OnKeyPress();
  }
};
vtkStandardNewMacro(KeyPressInteractorStyle);

namespace {
class vtkTimerCallback2 : public vtkCommand
{
public:
  vtkTimerCallback2() = default;

  static vtkTimerCallback2* New()
  {
    vtkTimerCallback2* cb = new vtkTimerCallback2;
    cb->TimerCount = 0;
    return cb;
  }

  virtual void Execute(vtkObject* caller, unsigned long eventId,
                       void* vtkNotUsed(callData))
  {
    if (vtkCommand::TimerEvent == eventId)
    {
      ++this->TimerCount;
    }
    INT bufsize = 10 * sizeof(float);
    array<float, 10> position ={0.0,0.0,0.0 ,0.0,0.0,   0.0,0.0,0.0 ,0.0,0.0};
    db_get_data(State::Keys::hDB, State::Keys::position, position.data(), &bufsize, TID_FLOAT);
    //std::cout << position[0]<<" "<<position[1]<<" "<<position[2] << std::endl;
    //PathGeneration::Point p2 = {{position[0],position[1],position[2]},{position[3],position[4]}};
    //Prism ob = PathGeneration::point_to_optical_box(p2, false);
    //actor->SetPosition(ob.center[1],ob.center[0],ob.center[2]);
    vtkTransform* transform = vtkTransform::New();
    transform->Identity();
    
    transform->Translate(position[1],position[0],position[2]);
    actor->SetUserTransform(transform);
    actor->Modified();
    vtkRenderWindowInteractor* iren = dynamic_cast<vtkRenderWindowInteractor*>(caller);
    iren->GetRenderWindow()->Render();

    if(record){
      vtkPNGWriter* writer = vtkPNGWriter::New();

      vtkWindowToImageFilter* windowToImageFilter = vtkWindowToImageFilter::New();
      windowToImageFilter->SetInput(renderWindow);

      windowToImageFilter->SetInputBufferTypeToRGBA(); // also record the alpha
                                                      // (transparency) channel
      windowToImageFilter->ReadFrontBufferOff();       // read from the back buffer
      writer->SetInputConnection(windowToImageFilter->GetOutputPort());
      windowToImageFilter->Update();
      writer->SetFileName(("output/"+std::to_string(TimerCount)+".png").c_str());
      writer->Write();
    }
  }

private:
  int TimerCount = 0;

public:
  vtkActor* actor = nullptr;
  int timerId = 0;
  int maxCount = -1;
};
} // namespace

vtkActor* render(Vec3 x){
  vtkPoints* points = vtkPoints::New();
  const double p[3] = {x.y, x.x, x.z};

  // Create the topology of the point (a vertex)
  vtkCellArray* vertices = vtkCellArray::New();
  // We need an an array of point id's for InsertNextCell.
  vtkIdType pid[1];
  pid[0] = points->InsertNextPoint(p);
  vertices->InsertNextCell(1, pid);

  // Create a polydata object
  vtkPolyData *point = vtkPolyData::New();

  // Set the points and vertices we created as the geometry and topology of the
  // polydata
  point->SetPoints(points);
  point->SetVerts(vertices);

  // Visualize
  vtkPolyDataMapper* mapper = vtkPolyDataMapper::New();
  mapper->SetInputData(point);

  vtkActor* actor = vtkActor::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(colors->GetColor3d("Blue").GetData());
  actor->GetProperty()->SetPointSize(20);
  renderer->AddActor(actor);
  return actor;
}

vtkActor* render(ConvexPolyhedron cp){
  std::vector<std::array<double, 3>> pointCoordinates;

  vtkPoints* points = vtkPoints::New();

  int i =0;
  std::cout << "Vertex Size: " << cp.vertexes.size() << std::endl;
  std::cout << "Edge Size: " << cp.edges.size() << std::endl;
  for(Vec3 point : cp.vertexes){
    pointCoordinates.push_back({{point.y,point.x,point.z}});
    std::cout << point << std::endl;
    points->InsertNextPoint(pointCoordinates.back().data());
    i++;
  }

  vtkPolyData* linesPolyData = vtkPolyData::New();
  linesPolyData->SetPoints(points);

  vtkCellArray* lines = vtkCellArray::New();
  std::vector<vtkLine*> lines_vec;
  for(IdxPair pair: cp.edges){
    lines_vec.push_back(vtkLine::New());
    lines_vec.back()->GetPointIds()->SetId(0, std::get<0>(pair));
    lines_vec.back()->GetPointIds()->SetId(1, std::get<1>(pair));
    lines->InsertNextCell(lines_vec.back());
  }

  

  linesPolyData->SetLines(lines);
  vtkPolyDataMapper* mapper = vtkPolyDataMapper::New();
  mapper->SetInputData(linesPolyData);
  vtkActor* actor = vtkActor::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetLineWidth(4);
  renderer->AddActor(actor);
  return actor;
}

vtkActor* render(Prism y){
  std::vector<std::array<double, 3>> pointCoordinates;

  // Create the points.
  vtkPoints* points = vtkPoints::New();

  // Create a hexahedron from the points.
  vtkHexahedron* hex = vtkHexahedron::New();

  for (auto i = 0; i < 8; ++i)
  {
    pointCoordinates.push_back({{y.vertexes()[i].y, y.vertexes()[i].x, y.vertexes()[i].z}});
    points->InsertNextPoint(pointCoordinates[i].data());
    hex->GetPointIds()->SetId(i, i);
  }

  // Add the hexahedron to a cell array.
  vtkCellArray* hexs = vtkCellArray::New();
  hexs->InsertNextCell(hex);

  // Add the points and hexahedron to an unstructured grid.
  vtkUnstructuredGrid* uGrid = vtkUnstructuredGrid::New();
  uGrid->SetPoints(points);
  uGrid->InsertNextCell(hex->GetCellType(), hex->GetPointIds());

  // Visualize.
  vtkDataSetMapper* mapper = vtkDataSetMapper::New();
  mapper->SetInputData(uGrid);

  vtkActor* actor = vtkActor::New();
  actor->GetProperty()->SetColor(colors->GetColor3d("PeachPuff").GetData());
  actor->SetMapper(mapper);
  renderer->AddActor(actor);
  return actor;
}

vtkActor* render(LineSegment x){
  double origin[3] = {x.a.y, x.a.x, x.a.z};
  double p0[3] = {x.b.y, x.b.x, x.b.z};
  vtkPoints* points = vtkPoints::New();
  points->InsertNextPoint(origin);
  points->InsertNextPoint(p0);
  vtkCellArray* lines = vtkCellArray::New();

  vtkLine* line = vtkLine::New();
  line->GetPointIds()->SetId(0, 1);
  //line->GetPointIds()->SetId(1, i + 1);
  lines->InsertNextCell(line);

  vtkPolyData* linesPolyData = vtkPolyData::New();
  linesPolyData->SetPoints(points);

  linesPolyData->SetLines(lines);
  vtkPolyDataMapper* mapper = vtkPolyDataMapper::New();
  mapper->SetInputData(linesPolyData);
  vtkActor* actor = vtkActor::New();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetLineWidth(4);
  renderer->AddActor(actor);
  
  return actor;
}

vtkActor* render(Sphere y){
  vtkSphereSource* sphere = vtkSphereSource::New();
  sphere->SetCenter(y.center.y,y.center.x,y.center.z);
  sphere->SetRadius(y.r);
  // Make the surface smooth.
  sphere->SetPhiResolution(100);
  sphere->SetThetaResolution(100);
  vtkPolyDataMapper* mapper = vtkPolyDataMapper::New();
  mapper->SetInputConnection(sphere->GetOutputPort());
  vtkActor* actor = vtkActor::New();

  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(colors->GetColor4d("Yellow").GetData());
  //
  renderer->AddActor(actor);
  return actor;
}

vtkActor* render(Cylinder y){
  vtkCylinderSource* cylinder = vtkCylinderSource::New();
  cylinder->SetResolution(32);
  cylinder->SetCapping(false);
  cylinder->SetHeight(y.e*2);
  cylinder->SetRadius(y.r);
  //cylinder->SetCenter(y.center.x,y.center.y,y.center.z);
  vtkPolyDataMapper* cylinderMapper = vtkPolyDataMapper::New();
  cylinderMapper->SetInputConnection(cylinder->GetOutputPort());
  vtkActor* cylinderActor = vtkActor::New();

  cylinderActor->SetMapper(cylinderMapper);
  cylinderActor->GetProperty()->SetColor(colors->GetColor4d("Tomato").GetData());

  vtkTransform* transform = vtkTransform::New();
  transform->Identity();
  
  transform->Translate(y.center.y,y.center.x,y.center.z);
  transform->RotateX(90);

  transform->RotateWXYZ(y.orientation.w,y.orientation.x,y.orientation.y,y.orientation.z);
  cylinderActor->SetUserTransform(transform);
  //
  renderer->AddActor(cylinderActor);
  return cylinderActor;
}

vtkActor* render(SD::GeomResult res){
  if (has<SD::ErrorType>(res)) {
  } else if (has<Vec3>(res)) {
    return render(get<Vec3>(res));
  } else if (has<LineSegment>(res)) {
    return render(get<LineSegment>(res));
  } else if (has<Prism>(res)) {
    return render(get<Prism>(res));
  } else if (has<Sphere>(res)) {
    return render(get<Sphere>(res));
  } else if (has<Cylinder>(res)) {
    return render(get<Cylinder>(res));
  } else {
    return nullptr;
  }
}

bool render_limit_box(){
  //TODO re these in from ODB
  double lim_neg[3] = {0.0,0.0,0.0};
  double lim_pos[3] = {GANTRY_0_MAX_Y,GANTRY_1_MAX_X,GANTRY_0_MAX_Z};

  vtkCubeSource* cube = vtkCubeSource::New();
  cube->SetXLength(lim_pos[0]-lim_neg[0]);
  cube->SetYLength(lim_pos[1]-lim_neg[1]);
  cube->SetZLength(lim_pos[2]-lim_neg[2]);
  cube->SetCenter(0.5*(lim_pos[0]+lim_neg[0]),
                  0.5*(lim_pos[1]+lim_neg[1]),
                  0.5*(lim_pos[2]+lim_neg[2]));
  cube->Update();

  vtkFeatureEdges* featureEdges = vtkFeatureEdges::New();
  featureEdges->SetInputConnection(cube->GetOutputPort());
  featureEdges->BoundaryEdgesOn();
  featureEdges->FeatureEdgesOff();
  featureEdges->ManifoldEdgesOff();
  featureEdges->NonManifoldEdgesOff();
  featureEdges->ColoringOn();
  featureEdges->Update();

  // Visualize
  vtkPolyDataMapper* edgeMapper = vtkPolyDataMapper::New();
  edgeMapper->SetInputConnection(featureEdges->GetOutputPort());
  edgeMapper->SetScalarModeToUseCellData();
  vtkActor* edgeActor = vtkActor::New();
  edgeActor->SetMapper(edgeMapper);

  renderer->AddActor(edgeActor);
}

 optional<vector<Intersectable>> load_collision_from_odb(HNDLE hDB) {
  auto raw = new array<char, COLLIDE_STR_MAXLEN*COLLIDE_STR_NUM>;
  INT  bufsize = COLLIDE_STR_MAXLEN*COLLIDE_STR_NUM;
  db_get_data(hDB, State::Keys::collision, raw->data(), &bufsize, TID_STRING);
  
  vector<string> strings;
  for (size_t i = 0; i < COLLIDE_STR_NUM; i++) {
    auto ptr = raw->data() + (COLLIDE_STR_MAXLEN * i);
    // ignore empty strings, as well as commented ones
    cm_msg(MINFO, "what?", ptr);
    if (std::string(ptr).compare("") != 0 &&
        std::string(ptr).substr(0,1).compare("#") != 0 &&
        std::string(ptr).substr(0,1).compare("!") != 0 ){
      strings.push_back(string(ptr));
    }
  }

  delete raw;

  vector<Intersectable> ret;

  if (strings.size() == 0) {
    cm_msg(MINFO, "feMove:load_collision_from_odb", "There are no objects to collide with in the ODB.");
    return ret;
  }

  for (size_t i = 0; i < strings.size(); i++) {
    auto str = strings[i];
    auto res = SD::deserialize(str);
    if (has<SD::ErrorType>(res)) {
      cm_msg(
        MERROR, "feMove:load_collision_from_odb", "Could not deserialize collidable object %zd (`%s'). Error message is: \"%s\".",
        i, str.c_str(), SD::error_message(get<SD::ErrorType>(res)).c_str()
      );
      return boost::none;
    } else if (has<Vec3>(res)) {
      ret.push_back(get<Vec3>(res));
    } else if (has<LineSegment>(res)) {
      ret.push_back(get<LineSegment>(res));
    } else if (has<Prism>(res)) {
      ret.push_back(get<Prism>(res));
    } else if (has<Sphere>(res)) {
      ret.push_back(get<Sphere>(res));
    } else if (has<Cylinder>(res)) {
      ret.push_back(get<Cylinder>(res));
    } else {
      cm_msg(
        MERROR, "feMove:load_collision_from_odb", "Could not convert type to Intersectable for collidable object %zd (`%s'). "
        "Please ensure that the datatype is one of: Vec3, LineSegment, Prism, Sphere, or Cylinder.", i, str.c_str()
      );
      return boost::none;
    }
  }

  return ret;
}

/*bool test_move(){
    array<float, 10> position, destination;
    bufsize = 10 * sizeof(float);
    db_get_data(hDB, State::Keys::position, position.data(), &bufsize, TID_FLOAT);
    bufsize = 10 * sizeof(float);
    db_get_data(hDB, State::Keys::destination, destination.data(), &bufsize, TID_FLOAT);
    
    auto start = PG::move_point_from_array(position);
    auto end   = PG::move_point_from_array(destination);

    auto path = PG::single_move(start, end, collidable);
}*/

int main(int args, char** argc)
{
    record = false;
    live=false;
    for(int i = 1; i < args;i++){
      
      if(std::string(argc[i]).compare("-r")==0){
        record = true;
        live=true;
      }else if(std::string(argc[i]).compare("-l")==0){
        std::cout << argc[i] << std::endl;
        live=true;
      }
    }



  INT  status, i;
  char host_name[256],exp_name[32];
  int hns = 256;
  int eps = 32;
  // get default values from environment
  cm_get_environment(host_name, hns, exp_name, eps);
  status = cm_connect_experiment(host_name, exp_name, "Test", NULL);
  if (status != CM_SUCCESS)
    return 1;
  
  cm_get_experiment_database(&State::Keys::hDB, NULL);


  const auto hDB = State::Keys::hDB;
  if (hDB == 0) {
    std::cerr << C_BR_RED << "Could not load database. Got handle: " << hDB << ".\n" << C_RESET;
    return CM_DB_ERROR;
  }
  array<float, 10> position ={0.0,0.0,0.0 ,0.0,0.0,   0.0,0.0,0.0 ,0.0,0.0};
  array<float, 10>destination ={1.0,0.0,0.0 ,0.0,0.0,   0.0,0.0,0.0 ,0.0,0.0};
  db_find_key(hDB, 0, "/Equipment/Move/Control/Destination", &State::Keys::destination);
  db_find_key(hDB, 0, "/Equipment/Move/Variables/Position", &State::Keys::position);
  db_find_key(hDB, 0, "/Equipment/Move/Settings/Collision", &State::Keys::collision);
  db_find_key(hDB, 0, "/Equipment/Move/Control/Input",  &State::Keys::input);

  //                           x   y   z    t   p
  INT bufsize = 10 * sizeof(float);
  db_get_data(hDB, State::Keys::position, position.data(), &bufsize, TID_FLOAT);
  bufsize = 10 * sizeof(float);
  db_get_data(hDB, State::Keys::destination, destination.data(), &bufsize, TID_FLOAT);
  // Set the background color.
  colors = vtkNamedColors::New();
  renderer = vtkRenderer::New();
  std::array<unsigned char, 4> bkg{{26, 51, 102, 255}};
  colors->SetColor("BkgColor", bkg.data());

  /*vector<Intersectable> ret;
  for(std::string s:data){
    Serialization::GeomResult res = Serialization::deserialize(s);
    auto intersectable = render(res);
    ret.push_back(intersectable);
  }*/
  auto _collidable = load_collision_from_odb(hDB);
  if (!_collidable) {
    cm_msg(MERROR, "feMove:start_move", "Error on loading collidable objects from ODB. Cannot continue.");
    return EXIT_FAILURE;
  }
  
  auto collidable = std::move(*_collidable);
  for(int i = 0; i < collidable.size();i++){
    render(collidable[i]);
  }
  render_limit_box();

  /*
   *Uses odb and creates a fake path to get the grantry collidable box
   */
  auto from = PathGeneration::move_point_from_array(position);
  auto to   = PathGeneration::move_point_from_array(destination);
  //auto path = PathGeneration::single_move(from, to, ret);
  auto path  = PathGeneration::generate_move({from.gantry0, to.gantry0}, from.gantry1, PathGeneration::Gantry0, PathGeneration::DimensionOrder::all_orders()[0]);
  auto pt = get<PathGeneration::MovePath>(path)[0];
  PathGeneration::Point p2 = {{0.0,0.0,0.0},pt.gantry0.angle};
  //Prism ob0 = PathGeneration::point_to_optical_box(p2, false);//Used Because the animation will set the right values
  auto prs0 = point_to_prisms(p2, false);
  Prism ob1 = PathGeneration::point_to_optical_box(pt.gantry1, true);

  KeyPressInteractorStyle* style = KeyPressInteractorStyle::New();
   
  std::vector<vtkActor*> animated_actors;
  animated_actors.push_back(render(prs0[0]));
  animated_actors.push_back(render(prs0[1]));
  animated_actors.push_back(render(prs0[2]));

  /*render(_sweep(prs0[0],Vec3(-0.3,0.0,0.0) ));
  render(polyhedron(get<Cylinder>(collidable[1])));

  std::cout << "Does Intersect? " << intersect2(_sweep(prs0[0],Vec3(-0.3,0.0,0.0) ),polyhedron(get<Cylinder>(collidable[1]))) << std::endl;*/

  style->gantry_actors.push_back(vtkSmartPointer<vtkActor>(animated_actors[0]));
  style->gantry_actors.push_back(vtkSmartPointer<vtkActor>(animated_actors[1]));
  style->gantry_actors.push_back(vtkSmartPointer<vtkActor>(animated_actors[2]));

  render(ob1);
  //render(PathGeneration::point_to_optical_box(p2, false));

  renderer->GetActiveCamera()->Yaw(180);
  renderer->GetActiveCamera()->Elevation(-45);
  //renderer->GetActiveCamera()->Azimuth(10);

  renderer->SetBackground(colors->GetColor3d("BkgColor").GetData());
  renderer->ResetCamera();
  renderer->GetActiveCamera()->Zoom(1.0);

  //Window stuff
  renderWindow = vtkRenderWindow::New();
  renderWindow->SetSize(1280, 720);
  renderWindow->AddRenderer(renderer);
  renderWindow->SetWindowName("Collision Model");
  vtkRenderWindowInteractor* renderWindowInteractor = vtkRenderWindowInteractor::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  
  style->collidable = collidable;
  renderWindowInteractor->SetInteractorStyle(style);
  style->SetCurrentRenderer(renderer);

  vtkAxesActor* axes = vtkAxesActor::New();
  vtkOrientationMarkerWidget* widget = vtkOrientationMarkerWidget::New();
  double rgba[4]{0.0, 0.0, 0.0, 0.0};
  colors->GetColor("Carrot", rgba);
  widget->SetOutlineColor(rgba[0], rgba[1], rgba[2]);
  widget->SetOrientationMarker(axes);
  widget->SetInteractor(renderWindowInteractor);
  widget->SetViewport(0.0, 0.0, 0.4, 0.4);
  widget->SetEnabled(1);
  widget->InteractiveOn();

  renderWindowInteractor->Initialize();


  // Sign up to receive TimerEvent
  if(live){
    for(vtkActor* annimated_actor:animated_actors){
      vtkTimerCallback2* cb = vtkTimerCallback2::New();
      cb->actor = annimated_actor;
      renderWindowInteractor->AddObserver(vtkCommand::TimerEvent, cb);
      int timerId = renderWindowInteractor->CreateRepeatingTimer(133);
      std::cout << "timerId: " << timerId << std::endl;
      break;
    }
  }

  renderWindow->Render();
  renderWindowInteractor->Start();

  cm_disconnect_experiment();
  return EXIT_SUCCESS;
}
