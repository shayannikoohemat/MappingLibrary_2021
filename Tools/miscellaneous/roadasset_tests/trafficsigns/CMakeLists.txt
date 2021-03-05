cmake_minimum_required(VERSION 3.15)
project(trafficsigns)

set(CMAKE_CXX_STANDARD 14)

set (MAPPING_LIB_DIR "/usr/local/lib/MappingLibrary/Library/lib64_linux")
set (MAPPING_INCLUDE_DIR "/usr/local/lib/MappingLibrary/Library")

# for boost library on Windows
#set(Boost_INCLUDE_DIR C:/Program\ Files/boost_gcc4.7/include/boost-1_64)
#set(Boost_LIBRARY_DIR C:/Program\ Files/boost_gcc4.7/lib)

find_package(Boost COMPONENTS system filesystem regex REQUIRED)
#include_directories(${Boost_INCLUDE_DIR})
#link_directories(${Boost_LIBRARY_DIR})

### CGAL
find_package(CGAL)


#include_directories(${MAPPING_INCLUDE_DIR}/Photogrammetry/include)
include_directories(../../../../Library/Photogrammetry/include)
include_directories(../../../../Library/Buildings/include)
#include_directories(/usr/local/lib/MappingLibrary/Library/LaserScan/include)
include_directories(../../../../Library/LaserScan/include)
include_directories(../../../../Foreign/ANN/include/ANN)
include_directories(../../../../Foreign/ANN/src)
include_directories(../../../../Foreign/ENLSIP)
include_directories(../../../../Foreign/newmat)
include_directories(../../../../Tools/visualization_tools)
#include_directories(../../../../Tools/building_modelling/pcm/include)
#link_directories(../../../../Library/lib64)
link_directories(${MAPPING_LIB_DIR})
link_libraries(-lBuildings -lLaserScan -lPhotogrammetry -lBuildings -lEispack -lLinpack -lANN -lnewmat -lENLSIP
        -lgfortran -g3 -llas -lglut -lGLU -lGL -lboost_system)


add_executable(trafficsigns
        main.cpp
        poledetection.cpp
        poledetection.h
        pole_decomposition.cpp
        pole_decomposition.h
        shapeDetection_cgal.cpp
        #poles.cc
        ../../../indoor_geometry_reconstruction_v2/morphology_voxelspace_spacepartitioning/Morphology/ConnCompSegmentation.cpp
        ../../../indoor_geometry_reconstruction_v2/morphology_voxelspace_spacepartitioning/Morphology/ConnCompSegmentation.h
        ../utils/MLS_preprocessing.cpp
        ../utils/MLS_preprocessing.h
        ../../../../Foreign/f2c/d_sign.c
        ../../../../Foreign/f2c/pow_dd.c)

target_link_libraries(trafficsigns ${Boost_LIBRARIES} CGAL::CGAL)