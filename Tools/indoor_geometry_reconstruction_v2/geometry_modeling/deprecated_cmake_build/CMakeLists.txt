cmake_minimum_required(VERSION 3.3)
project(geometry_modeling)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# set a customized bin directory
set(dir  "../../bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${dir}) # to make a customized build directory

set (MAPPING_LIB_DIR "/home/shayan/MappingLibrary/Library/lib64_linux")
set (MAPPING_INCLUDE_DIR "/home/shayan/MappingLibrary/Library")

# for boost library
#set(Boost_INCLUDE_DIR C:/Program\ Files/boost_gcc4.7/include/boost-1_64)
#set(Boost_LIBRARY_DIR C:/Program\ Files/boost_gcc4.7/lib)
find_package(Boost COMPONENTS system filesystem regex REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

#include_directories(${MAPPING_INCLUDE_DIR}/Photogrammetry/include)
include_directories(../../../../Library/Photogrammetry/include)
include_directories(../../../../Library/Buildings/include)
#include_directories(/usr/local/lib/MappingLibrary/Library/LaserScan/include)
include_directories(../../../../Library/LaserScan/include)
include_directories(../../../../Foreign/ANN/include/ANN)
include_directories(../../../../Foreign/ANN/src)
link_directories(${MAPPING_LIB_DIR})
link_libraries(-lLaserScan -lPhotogrammetry -lBuildings -lEispack -lLinpack -lANN -lnewmat -lgfortran -g3 -llas)

#link_directories(../../tools/build/src)
#link_directories(../build/src)

#[[file(GLOB tools_Sources ../../tools/src/VisualizePlanes.cpp ../tools/src/Offset_Polygons.cpp
        ../../tools/src/Minimum3DRectangle.cpp ../../tools/src/Min3DBox_OFF.cpp
        ../../tools/src/Project2Plane.cpp
        ../../tools/src/post_processing.cpp ../../tools/src/MLS_preprocessing.cpp
        ../../tools/src/Directory_Processing.cpp
        ../../tools/src/GenerateWallPatches.cpp ../../tools/src/segment_refinement.cpp
        ../../tools/src/MergeSurfaces.cpp)]]

#[[set(SOURCE_FILES main.cpp ../ModelingInteriorWalls.cpp ../../tools/include/utils.h
        ../../tools/include/visualization_tools.h ${tools_Sources})]]

#add_executable(modeling_ ${SOURCE_FILES})
add_executable(modeling_ test_mergeSegFiles.cpp)
target_link_libraries(modeling_ PUBLIC ${BOOST_LIBRARIES} ${CMAKE_CURRENT_SOURCE_DIR}/../lib/libtools.a)

