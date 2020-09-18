cmake_minimum_required(VERSION 3.3)
project(geometry_modeling)

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# set a customized build directory
set(dir  "../../bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${dir}) # to make a customized build directory

set (MAPPING_LIB_DIR "/home/shayan/MappingLibrary/Library/lib64_linux")
set (MAPPING_INCLUDE_DIR "/home/shayan/MappingLibrary/Library")

#include_directories(${MAPPING_INCLUDE_DIR}/Photogrammetry/include)
include_directories(../../../../../Library/Photogrammetry/include)
include_directories(../../../../../Library/Buildings/include)
#include_directories(/usr/local/lib/MappingLibrary/Library/LaserScan/include)
include_directories(../../../../../Library/LaserScan/include)
include_directories(../../../../../Foreign/ANN/include/ANN)
include_directories(../../../../../Foreign/ANN/src)
link_directories(${MAPPING_LIB_DIR})
link_libraries(-lLaserScan -lPhotogrammetry -lBuildings -lEispack -lLinpack -lANN -lnewmat -lgfortran -g3 -llas)

## added for this library
#link_directories(../cmake-build-debug/src)
#link_libraries(-lsegmentation)


set(SOURCE_FILES main.cpp)
add_executable(surfacegrowing_test ${SOURCE_FILES})
target_link_libraries(surfacegrowing_test PRIVATE segmentation) # very important to link library to exe
