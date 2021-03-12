cmake_minimum_required(VERSION 3.3)
project(permanent_structure_detection)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(dir  "../../bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${dir}) # to make a customized build directory

set (MAPPING_LIB_DIR "/usr/local/lib/MappingLibrary/Library/lib64_linux")
set (MAPPING_INCLUDE_DIR "/usr/local/lib/MappingLibrary/Library")

# for boost library
#set(Boost_INCLUDE_DIR C:/Program\ Files/boost_gcc4.7/include/boost-1_64)
#set(Boost_LIBRARY_DIR C:/Program\ Files/boost_gcc4.7/lib)
find_package(Boost COMPONENTS system filesystem regex REQUIRED)


#include_directories(${MAPPING_INCLUDE_DIR}/Photogrammetry/include)
include_directories(../../../Library/Photogrammetry/include)
include_directories(../../../Library/Buildings/include)
#include_directories(/usr/local/lib/MappingLibrary/Library/LaserScan/include)
include_directories(../../../Library/LaserScan/include)
include_directories(../../../Foreign/ANN/include/ANN)
include_directories(../../../Foreign/ANN/src)
#include_directories(../../../Tools/building_modelling/pcm/include)
#link_directories(../../../Library/lib64)
link_directories(${MAPPING_LIB_DIR})
link_libraries(-lLaserScan -lPhotogrammetry -lBuildings -lEispack -lLinpack
    -lANN -lnewmat -lgfortran -g3 -llas -lboost_filesystem -lboost_system)


# adding other directories source files
#file(GLOB utils_Sources "${PROJECT_SOURCE_DIR}/../utils/*.cpp") # not a good solution because it adds all cpp which we dont need for this build
file(GLOB utils_Sources ../utils/GenerateWallPatches.cpp ../utils/segment_refinement.cpp ../utils/Buffer.cpp
        ../utils/PartitionLpByTag.cpp ../utils/MLS_preprocessing.cpp ../utils/post_processing.cpp
        ../utils/Directory_Processing.cpp)

#file(GLOB visualization_tools_Sources RELATIVE  "visualization_tools" "*.cpp")

set(SOURCE_FILES main.cpp indoorTopology_v2.cpp ../utils/utils.h ${utils_Sources})

add_executable(indoor_reconstruction ${SOURCE_FILES})
target_link_options(indoor_reconstruction PRIVATE -no-pie)
target_link_libraries(indoor_reconstruction ${Boost_LIBRARIES})
target_include_directories(indoor_reconstruction PRIVATE ${utils_Sources})
