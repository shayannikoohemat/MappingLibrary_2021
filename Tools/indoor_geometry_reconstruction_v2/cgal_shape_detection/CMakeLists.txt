cmake_minimum_required(VERSION 3.15)
project(ransac)

set(CMAKE_CXX_STANDARD 14)

set(dir  "../bin")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${dir}) # to make a customized build directory

## boost 1.65
find_package(Boost COMPONENTS system filesystem regex REQUIRED)
include_directories(${Boost_INCLUDE_DIR})
link_directories(${Boost_LIBRARY_DIR})

## CGAL 5.3 (without drawing)
#find_package(CGAL)

#CGAL_Qt5 is needed for the drawing.
find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt5)
if(CGAL_Qt5_FOUND)
  #required to use basic_viewer
    add_definitions(-DCGAL_USE_BASIC_VIEWER -DQT_NO_KEYWORDS)
endif()


set(SOURCE_FILES main.cpp planar_ransac.cpp planar_ransac.h)

add_executable(ransac ${SOURCE_FILES})
target_link_libraries(ransac ${Boost_LIBRARIES} CGAL::CGAL)
target_link_options(ransac PRIVATE -no-pie)

if(CGAL_Qt5_FOUND)
  #link with the required CGAL libraries
  target_link_libraries(ransac PUBLIC CGAL::CGAL_Qt5)
endif()
