cmake_minimum_required(VERSION 3.3)
project(permanent_structure_detection)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#set(dir  "/usr/local/lib/MappingLibrary/Tools/indoor_geometry_reconstrcution_v2/permanent_structure_detection/Release")
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${dir}) # to make a customized build directory

set (MAPPING_LIB_DIR "/usr/local/lib/MappingLibrary/Library/lib64_linux")
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
#include_directories(/usr/local/lib/MappingLibrary/Library/LaserScan/include)
include_directories(../../../Library/LaserScan/include)
include_directories(../../../Foreign/ANN/include/ANN)
include_directories(../../../Foreign/ANN/src)
#include_directories(../../../Tools/building_modelling/pcm/include)
#link_directories(../../../Library/lib64)
link_directories(${MAPPING_LIB_DIR})
link_libraries(-lLaserScan -lPhotogrammetry -lBuildings -lEispack -lLinpack -lANN -lnewmat -lgfortran -g3 -llas)

# suppressing compiler warnings, to fix warnings you can comment this part
if(CMAKE_C_COMPILER_ID MATCHES "MSVC")
    set(MSVC_DISABLED_WARNINGS_LIST
            "C4018" # 'expression' : signed/unsigned mismatch
            "C4057" # 'operator' : 'identifier1' indirection to
            # slightly different base types from 'identifier2'
            "C4100" # 'identifier' : unreferenced formal parameter
            "C4127" # conditional expression is constant
            "C4146" # unary minus operator applied to unsigned type,
            # result still unsigned
            "C4244" # 'argument' : conversion from 'type1' to 'type2',
            # possible loss of data
            "C4245" # 'conversion' : conversion from 'type1' to 'type2',
            # signed/unsigned mismatch
           # "C4267" # 'var' : conversion from 'size_t' to 'type',
            # possible loss of data
            "C4389" # 'operator' : signed/unsigned mismatch
            "C4706" # assignment within conditional expression
            "C4996" # The POSIX name for this item is deprecated.
            # Instead, use the ISO C and C++ conformant name
            )
endif()

# adding other directories source files
set(proj_dir "${PROJECT_SOURCE_DIR}/../")
#file(GLOB utils_Sources "${PROJECT_SOURCE_DIR}/../utils/*.cpp") # not a good solution because it adds all cpp which we dont need for this build
file(GLOB utils_Sources ../utils/GenerateWallPatches.cpp ../utils/segment_refinement.cpp ../utils/Buffer.cpp
        ../utils/PartitionLpByTag.cpp ../utils/MLS_preprocessing.cpp ../utils/post_processing.cpp
        ../utils/Directory_Processing.cpp)

#file(GLOB visualization_tools_Sources RELATIVE  "visualization_tools" "*.cpp")



set(SOURCE_FILES main.cpp indoorTopology_v2.cpp ../utils/utils.h data_preparation.cpp ${utils_Sources})

add_executable(indoor_reconstruction ${SOURCE_FILES})
# These two lines make sure the dependencies are included in the MakeFile
# from:https://stackoverflow.com/questions/13703647/how-to-properly-add-include-directories-with-cmake
target_link_libraries(indoor_reconstruction ${Boost_LIBRARIES})
target_include_directories(indoor_reconstruction PRIVATE ${utils_Sources})
