cmake_minimum_required(VERSION 3.15)
project(cell_decomposition)

set(CMAKE_CXX_STANDARD 14)

set(dir  "../bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${dir}) # to make a customized build directory

set (MAPPING_LIB_DIR "/usr/local/lib/MappingLibrary/Library/lib64_linux")
set (MAPPING_INCLUDE_DIR "/usr/local/lib/MappingLibrary/Library")

## boost 1.65
find_package(Boost COMPONENTS system filesystem regex REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

## CGAL 5.3 (without drawing)
#find_package(CGAL)

#CGAL_Qt5 is needed for the drawing.
find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)
if(CGAL_Qt5_FOUND)
  #required to use basic_viewer
    add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
endif()

include_directories(../../../Library/Photogrammetry/include)
include_directories(../../../Library/Buildings/include)
include_directories(../../../Library/LaserScan/include)
include_directories(../../../Foreign/ANN/include/ANN)

link_directories(${MAPPING_LIB_DIR})
link_libraries(-lLaserScan -lPhotogrammetry -lBuildings -lEispack
        -lLinpack -lANN -lnewmat -lgfortran -g3 -llas
        -lboost_system -lboost_filesystem)

file(GLOB visualisation_Sources
        ../visualization_tools/VisualizePlanes.cpp
        ../visualization_tools/Offset_Polygons.cpp
        ../visualization_tools/Minimum3DRectangle.cpp
        ../visualization_tools/Min3DBox_OFF.cpp
        ../visualization_tools/Project2Plane.cpp)

file(GLOB utils_Sources
        ../utils/post_processing.cpp
        ../utils/MLS_preprocessing.cpp
        ../utils/Directory_Processing.cpp
        ../utils/GenerateWallPatches.cpp
        ../utils/segment_refinement.cpp
        ../utils/Buffer.cpp
        ../utils/MergeSurfaces.cpp)

set(SOURCE_FILES main.cpp
        main_pipeline.cpp
        #cgal_ransac.cpp
        #../../../Library/Photogrammetry/src/Vector3D.cc
        Laservoxel.cpp
        ReadAscii.cpp
        sample_laserpoints.cpp
        intersecting_clippers.cpp
        face_selection.cpp
        planar_segmentation.cpp
        alpha_shape.cpp
        occupancy_grid.cpp
        ../utils/utils.h
        ../visualization_tools/visualization_tools.h
        ${visualisation_Sources}
        ${utils_Sources})

add_executable(cell_decomposition ${SOURCE_FILES})
target_link_libraries(cell_decomposition ${Boost_LIBRARIES} CGAL::CGAL)
target_link_options(cell_decomposition PRIVATE -no-pie)

if(CGAL_Qt5_FOUND)
  #link with the required CGAL libraries
  target_link_libraries(cell_decomposition PUBLIC CGAL::CGAL_Qt5)
endif()
