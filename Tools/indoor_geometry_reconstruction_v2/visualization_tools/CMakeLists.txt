cmake_minimum_required(VERSION 3.6)
project(visualization_tools)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#set(dir  "D:/Mapping/Tools/visualization_tools/Debug")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${dir})

set (MAPPING_LIB_DIR "/usr/local/lib/MappingLibrary/Library/lib64_linux") #D:\MappingLibrary\Mapping\Library\lib64
set (MAPPING_INCLUDE_DIR "/usr/local/lib/MappingLibrary/Library")

# for boost library
#set(Boost_INCLUDE_DIR C:/Program\ Files/boost_gcc4.7/include/boost-1_64)
#set(Boost_LIBRARY_DIR C:/Program\ Files/boost_gcc4.7/lib)
find_package(Boost COMPONENTS system filesystem regex REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

#include_directories(${MAPPING_INCLUDE_DIR}/Photogrammetry/include)
include_directories(../../../Library/Photogrammetry/include)
include_directories(../../../Library/Buildings/include)
include_directories(../../../Library/LaserScan/include)
include_directories(../../../Foreign/ANN/include/ANN)
include_directories(../../../Foreign/ANN/src)
#include_directories(../../../Tools/indoor_geometry_reconstruction)
#link_directories(D:/Mapping/Library/lib64)
link_directories(${MAPPING_LIB_DIR})
link_libraries(-lLaserScan -lPhotogrammetry -lBuildings -lEispack -lLinpack -lANN -lnewmat -lgfortran -g3 -llas)

set(SOURCE_FILES main.cpp VisualizePlanes.cpp Minimum3DRectangle.cpp Minimum3DBox.cpp
        Project2Plane.cpp GenerateConvexPolygon.cpp visualization_tools.h Min3DBox_OFF.cpp
        visualization_tools.h Offset_Polygons.cpp
        ../utils/utils.h  # added to use PartitionLpByTag() function
        ../utils/MLS_preprocessing.cpp    # added to use PartitionLpByTag() function
        ../utils/Directory_Processing.cpp
        )
add_executable(visualization ${SOURCE_FILES})
target_link_libraries(visualization ${Boost_LIBRARIES})