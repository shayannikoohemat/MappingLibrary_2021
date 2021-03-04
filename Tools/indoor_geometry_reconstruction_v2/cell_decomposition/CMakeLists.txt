cmake_minimum_required(VERSION 3.15)
project(cell_decomposition)

set(CMAKE_CXX_STANDARD 14)

set(dir  "../bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${dir}) # to make a customized build directory

set (MAPPING_LIB_DIR "/usr/local/lib/MappingLibrary/Library/lib64_linux")
set (MAPPING_INCLUDE_DIR "/usr/local/lib/MappingLibrary/Library")

find_package(Boost COMPONENTS system filesystem regex REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

include_directories(../../../Library/Photogrammetry/include)
include_directories(../../../Library/Buildings/include)
include_directories(../../../Library/LaserScan/include)
include_directories(../../../Foreign/ANN/include/ANN)

link_directories(${MAPPING_LIB_DIR})
link_libraries(-lLaserScan -lPhotogrammetry -lBuildings -lEispack
        -lLinpack -lANN -lnewmat -lgfortran -g3 -llas)

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
        intersecting_clippers.cpp
        ../utils/utils.h
        ../visualization_tools/visualization_tools.h
        ${visualisation_Sources}
        ${utils_Sources})

add_executable(cell_decomposition ${SOURCE_FILES})
target_link_libraries(cell_decomposition ${Boost_LIBRARIES})
target_link_options(cell_decomposition PRIVATE -no-pie)
