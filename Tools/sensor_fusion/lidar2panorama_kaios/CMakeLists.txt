cmake_minimum_required(VERSION 3.5)

project(lidar2panorama LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

#set (MAPPING_LIB_DIR $ENV{MAPPING_LIB_DIR})
#set (MAPPING_INCLUDE_DIR $ENV{MAPPING_INCLUDE_DIR})
set (MAPPING_LIB_DIR "/home/shayan/MappingLibrary/Library/lib64_linux")
set (MAPPING_INCLUDE_DIR "/home/shayan/MappingLibrary/Library")

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(../../../Library/Photogrammetry/include)
include_directories(../../../Library/Buildings/include)
include_directories(../../../Library/LaserScan/include)
include_directories(../../../Foreign/ANN/include/ANN)

link_directories(${MAPPING_LIB_DIR})
link_libraries(-lBuildings -lLaserScan -lPhotogrammetry -lEispack -lLinpack -lANN -lnewmat
        -lgfortran -g3 -llas -lglut -lGLU -lGL)


add_executable(lidar2panorama main.cpp equi_proj.cpp lidar_projection.cpp)
target_link_options(lidar2panorama PRIVATE -no-pie)
target_link_libraries (lidar2panorama Eigen3::Eigen)
