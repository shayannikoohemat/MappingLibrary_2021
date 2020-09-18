cmake_minimum_required(VERSION 3.3)
project(tools)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# set a customized build directory
#set(dir  "/usr/local/lib/MappingLibrary/Tools/indoor_geometry_reconstrcution_v2/permanent_structure_detection/Debug")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${dir}) # to make a customized build directory

set (MAPPING_LIB_DIR "/usr/local/lib/MappingLibrary/Library/lib64_linux")
set (MAPPING_INCLUDE_DIR "/usr/local/lib/MappingLibrary/Library")

# for boost library
#set(Boost_INCLUDE_DIR C:/Program\ Files/boost_gcc4.7/include/boost-1_64)
#set(Boost_LIBRARY_DIR C:/Program\ Files/boost_gcc4.7/lib)
find_package(Boost COMPONENTS system filesystem regex REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
#link_directories(${Boost_LIBRARY_DIR})

#include_directories(${MAPPING_INCLUDE_DIR}/Photogrammetry/include)
include_directories(../../../../Library/Photogrammetry/include)
include_directories(../../../../Library/Buildings/include)
#include_directories(/usr/local/lib/MappingLibrary/Library/LaserScan/include)
include_directories(../../../../Library/LaserScan/include)
include_directories(../../../../Foreign/ANN/include/ANN)
include_directories(../../../../Foreign/ANN/src)
include_directories(../../../../Tools/visualization_tools)
#include_directories(../../../Tools/building_modelling/pcm/include)
#link_directories(../../../Library/lib64)
link_directories(${MAPPING_LIB_DIR})
link_libraries(-lLaserScan -lPhotogrammetry -lBuildings -lEispack -lLinpack -lANN -lnewmat -lgfortran -g3 -llas)

#set(SOURCE_FILES )

#set(SOURCE_FILES main.cpp utils.h MLS_preprocessing.cpp Directory_Processing.cpp GenerateWallPatches.cpp)
#add_executable(utils ${SOURCE_FILES})
#target_link_libraries(utils ${Boost_LIBRARIES})

add_library(
        tools
        MLS_preprocessing.cpp Directory_Processing.cpp GenerateWallPatches.cpp
        VisualizePlanes.cpp Minimum3DRectangle.cpp Minimum3DBox.cpp Min3DBox_OFF.cpp
        Project2Plane.cpp MergeSurfaces.cpp post_processing.cpp
        GenerateConvexPolygon.cpp Offset_Polygons.cpp ReadAscii.cpp
        #../include/visualization_tools.h ../include/utils.h ../include/post_processing.h
)
message(${Boost_LIBRARIES})
message(${Boost_INCLUDE_DIR})
message(${MAPPING_LIB_DIR})
target_include_directories(tools PUBLIC ${Boost_INCLUDE_DIR} "${CMAKE_CURRENT_SOURCE_DIR}/../include/")

target_link_libraries(tools PUBLIC ${Boost_LIBRARIES})
