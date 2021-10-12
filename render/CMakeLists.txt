cmake_minimum_required(VERSION 3.3 FATAL_ERROR)

project(CollisionObjectsViewer)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDEBUG")

find_package(VTK COMPONENTS 
  vtkCommonColor
  vtkCommonCore
  vtkFiltersSources
  vtkInteractionStyle
  vtkRenderingContextOpenGL2
  vtkRenderingCore
  vtkRenderingFreeType
  vtkRenderingGL2PSOpenGL2
  vtkRenderingOpenGL2
  vtkInteractionWidgets
  vtkRenderingAnnotation
  QUIET
)

find_package(Boost REQUIRED COMPONENTS regex) 

set(SOURCE
  CollisionObjectsViewer.cpp
  ../collision/geometry/bounds.cpp 
  ../collision/geometry/geom.cpp 
  ../collision/geometry/intersection_displacement.cpp 
  ../collision/geometry/intersection_rotation.cpp 
  ../collision/geometry/intersection_static.cpp 
  ../collision/geometry/prism.cpp 
  ../collision/geometry/quaternion.cpp 
  ../collision/geometry/rotations.cpp 
  ../collision/geometry/sat.cpp 
  ../collision/serialization/serialization_internal.cpp 
  ../collision/pathgen/rect.cpp
  ../collision/pathgen/cyl.cpp 
  ../collision/geometry/vec3.cpp 
  ../collision/pathgen/pathgen.cpp 
  ../collision/serialization/serialization.cpp
)

if (NOT VTK_FOUND)
  message("Skipping CylinderExample: ${VTK_NOT_FOUND_MESSAGE}")
  return ()
endif()
message (STATUS "VTK_VERSION: ${VTK_VERSION}")
if (VTK_VERSION VERSION_LESS "8.90.0")
  # old system
  include(${VTK_USE_FILE})
  add_executable(CollisionObjectsViewer MACOSX_BUNDLE ${SOURCE} )
  target_link_libraries(CollisionObjectsViewer PRIVATE ${VTK_LIBRARIES})
else ()
  # include all components
  include_directories(${Boost_INCLUDE_DIRS} ../ ../collision ../collision/geometry ../collision/pathgen ../collision/serialization)
  add_executable(CollisionObjectsViewer MACOSX_BUNDLE ${SOURCE} )
  target_link_libraries(CollisionObjectsViewer PRIVATE ${VTK_LIBRARIES} ${Boost_LIBRARIES})
  # vtk_module_autoinit is needed
  vtk_module_autoinit(
    TARGETS CollisionObjectsViewer
    MODULES ${VTK_LIBRARIES}
  )
endif ()