cmake_minimum_required(VERSION 3.5)

project(modeling_pipeline LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# set a customized build directory
set(dir  "../bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${dir}) # to make a customized build directory

set (MAPPING_LIB_DIR "/home/shayan/MappingLibrary/Library/lib64_linux")
set (MAPPING_INCLUDE_DIR "/home/shayan/MappingLibrary/Library")

# for boost library
find_package(Boost COMPONENTS system filesystem regex REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

## CGAL 5.3
find_package(CGAL)

#include_directories(${MAPPING_INCLUDE_DIR}/Photogrammetry/include)
include_directories(../../../Library/Photogrammetry/include)
include_directories(../../../Library/Buildings/include)
#include_directories(/usr/local/lib/MappingLibrary/Library/LaserScan/include)
include_directories(../../../Library/LaserScan/include)
include_directories(../../../Foreign/ANN/include/ANN)
include_directories(../../../Foreign/ANN/src)

link_directories(${MAPPING_LIB_DIR})
link_libraries(-lLaserScan -lPhotogrammetry -lBuildings -lEispack -lLinpack
                -lANN -lnewmat -lgfortran -g3 -llas
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
        modeling_pipeline.cpp
        ReadAscii.cpp
        planar_segmentation.cpp
        ../geometry_modeling/ModelingInteriorWalls.cpp
        ../geometry_modeling/ModelingInteriorWalls.h
        ../utils/utils.h
        ../visualization_tools/visualization_tools.h
        ${visualisation_Sources}
        ${utils_Sources})

add_executable(3dmodeling ${SOURCE_FILES})
target_link_libraries(3dmodeling ${Boost_LIBRARIES} CGAL::CGAL)
target_link_options(3dmodeling PRIVATE -no-pie)

