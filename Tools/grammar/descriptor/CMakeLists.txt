cmake_minimum_required(VERSION 3.7)
project(descriptor)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CONFIGURATION_TYPES "Debug")  # change it to release for production
set (MAPPING_LIB_DIR "D:/Mapping/Library/lib64")
set (MAPPING_INCLUDE_DIR "D:/Mapping/Library")

set(dir  "D:/Mapping/Tools/grammar/descriptor/Debug")  # Release or Debug
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${dir})

set(dir2 "D:Mapping/Tools/grammar/annotator/")

# for boost library
set(Boost_INCLUDE_DIR C:/Program\ Files/boost_gcc4.7/include/boost-1_64)
set(Boost_LIBRARY_DIR C:/Program\ Files/boost_gcc4.7/lib)
find_package(Boost COMPONENTS system filesystem regex REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

#include_directories(${MAPPING_INCLUDE_DIR}/Photogrammetry/include)
include_directories(../../../Library/Photogrammetry/include)
include_directories(../../../Library/Buildings/include)
include_directories(../../../Library/LaserScan/include)
include_directories(../../../Foreign/ANN/include/ANN)
include_directories(../../../Foreign/ANN/src)
include_directories(../annotator)

#link_directories(D:/Mapping/Library/lib64)
link_directories(${MAPPING_LIB_DIR})
link_libraries(-lLaserScan -lPhotogrammetry -lBuildings -lEispack -lLinpack
        -lANN -lnewmat -lgfortran -g3 -llas)

set(SOURCE_FILES main.cpp Mls_Preparation.cpp ../annotator/Annotator.cpp
        segment_refinement.cpp boosttree.cpp indoor_graph.cpp selectpyramid.cpp read_laserblocks.cpp)
add_executable(descriptor ${SOURCE_FILES})
target_link_libraries(descriptor ${Boost_LIBRARIES})
