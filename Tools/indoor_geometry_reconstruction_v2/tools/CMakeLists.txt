cmake_minimum_required(VERSION 3.3)
project(tools)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#this includes the CMakeLists.txt from the subdirectories
add_subdirectory(./src) #order here matters, first sub_dir containing the library and then sub_dir containing the main.cpp
add_subdirectory(./test)


