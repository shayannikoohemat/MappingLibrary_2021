# CMAKE generated file: DO NOT EDIT!
# Generated by "MinGW Makefiles" Generator, CMake Version 2.8

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list

# Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

SHELL = cmd.exe

# The CMake executable.
CMAKE_COMMAND = "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe"

# The command to remove a file.
RM = "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E remove -f

# Escaping for special characters.
EQUALS = =

# The program to use to edit the cache.
CMAKE_EDIT_COMMAND = "C:\Program Files (x86)\CMake 2.8\bin\cmake-gui.exe"

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4

# Include any dependencies generated for this target.
include apps/haartraining/CMakeFiles/opencv_performance.dir/depend.make

# Include the progress variables for this target.
include apps/haartraining/CMakeFiles/opencv_performance.dir/progress.make

# Include the compile flags for this target's objects.
include apps/haartraining/CMakeFiles/opencv_performance.dir/flags.make

apps/haartraining/CMakeFiles/opencv_performance.dir/performance.cpp.obj: apps/haartraining/CMakeFiles/opencv_performance.dir/flags.make
apps/haartraining/CMakeFiles/opencv_performance.dir/performance.cpp.obj: apps/haartraining/CMakeFiles/opencv_performance.dir/includes_CXX.rsp
apps/haartraining/CMakeFiles/opencv_performance.dir/performance.cpp.obj: apps/haartraining/performance.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object apps/haartraining/CMakeFiles/opencv_performance.dir/performance.cpp.obj"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\apps\haartraining && C:\Dev-Cpp\MinGW64\bin\g++.exe   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles\opencv_performance.dir\performance.cpp.obj -c D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\apps\haartraining\performance.cpp

apps/haartraining/CMakeFiles/opencv_performance.dir/performance.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_performance.dir/performance.cpp.i"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\apps\haartraining && C:\Dev-Cpp\MinGW64\bin\g++.exe  $(CXX_DEFINES) $(CXX_FLAGS) -E D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\apps\haartraining\performance.cpp > CMakeFiles\opencv_performance.dir\performance.cpp.i

apps/haartraining/CMakeFiles/opencv_performance.dir/performance.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_performance.dir/performance.cpp.s"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\apps\haartraining && C:\Dev-Cpp\MinGW64\bin\g++.exe  $(CXX_DEFINES) $(CXX_FLAGS) -S D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\apps\haartraining\performance.cpp -o CMakeFiles\opencv_performance.dir\performance.cpp.s

apps/haartraining/CMakeFiles/opencv_performance.dir/performance.cpp.obj.requires:
.PHONY : apps/haartraining/CMakeFiles/opencv_performance.dir/performance.cpp.obj.requires

apps/haartraining/CMakeFiles/opencv_performance.dir/performance.cpp.obj.provides: apps/haartraining/CMakeFiles/opencv_performance.dir/performance.cpp.obj.requires
	$(MAKE) -f apps\haartraining\CMakeFiles\opencv_performance.dir\build.make apps/haartraining/CMakeFiles/opencv_performance.dir/performance.cpp.obj.provides.build
.PHONY : apps/haartraining/CMakeFiles/opencv_performance.dir/performance.cpp.obj.provides

apps/haartraining/CMakeFiles/opencv_performance.dir/performance.cpp.obj.provides.build: apps/haartraining/CMakeFiles/opencv_performance.dir/performance.cpp.obj

# Object files for target opencv_performance
opencv_performance_OBJECTS = \
"CMakeFiles/opencv_performance.dir/performance.cpp.obj"

# External object files for target opencv_performance
opencv_performance_EXTERNAL_OBJECTS =

bin/opencv_performance.exe: apps/haartraining/CMakeFiles/opencv_performance.dir/performance.cpp.obj
bin/opencv_performance.exe: apps/haartraining/CMakeFiles/opencv_performance.dir/build.make
bin/opencv_performance.exe: lib/libopencv_core244.dll.a
bin/opencv_performance.exe: lib/libopencv_imgproc244.dll.a
bin/opencv_performance.exe: lib/libopencv_highgui244.dll.a
bin/opencv_performance.exe: lib/libopencv_objdetect244.dll.a
bin/opencv_performance.exe: lib/libopencv_calib3d244.dll.a
bin/opencv_performance.exe: lib/libopencv_video244.dll.a
bin/opencv_performance.exe: lib/libopencv_features2d244.dll.a
bin/opencv_performance.exe: lib/libopencv_flann244.dll.a
bin/opencv_performance.exe: lib/libopencv_legacy244.dll.a
bin/opencv_performance.exe: lib/libopencv_haartraining_engine.a
bin/opencv_performance.exe: lib/libopencv_objdetect244.dll.a
bin/opencv_performance.exe: lib/libopencv_legacy244.dll.a
bin/opencv_performance.exe: lib/libopencv_calib3d244.dll.a
bin/opencv_performance.exe: lib/libopencv_video244.dll.a
bin/opencv_performance.exe: lib/libopencv_features2d244.dll.a
bin/opencv_performance.exe: lib/libopencv_highgui244.dll.a
bin/opencv_performance.exe: lib/libopencv_imgproc244.dll.a
bin/opencv_performance.exe: lib/libopencv_flann244.dll.a
bin/opencv_performance.exe: lib/libopencv_ml244.dll.a
bin/opencv_performance.exe: lib/libopencv_core244.dll.a
bin/opencv_performance.exe: 3rdparty/lib/libzlib.a
bin/opencv_performance.exe: apps/haartraining/CMakeFiles/opencv_performance.dir/objects1.rsp
bin/opencv_performance.exe: apps/haartraining/CMakeFiles/opencv_performance.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ..\..\bin\opencv_performance.exe"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\apps\haartraining && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\opencv_performance.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
apps/haartraining/CMakeFiles/opencv_performance.dir/build: bin/opencv_performance.exe
.PHONY : apps/haartraining/CMakeFiles/opencv_performance.dir/build

apps/haartraining/CMakeFiles/opencv_performance.dir/requires: apps/haartraining/CMakeFiles/opencv_performance.dir/performance.cpp.obj.requires
.PHONY : apps/haartraining/CMakeFiles/opencv_performance.dir/requires

apps/haartraining/CMakeFiles/opencv_performance.dir/clean:
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\apps\haartraining && $(CMAKE_COMMAND) -P CMakeFiles\opencv_performance.dir\cmake_clean.cmake
.PHONY : apps/haartraining/CMakeFiles/opencv_performance.dir/clean

apps/haartraining/CMakeFiles/opencv_performance.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4 D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\apps\haartraining D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4 D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\apps\haartraining D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\apps\haartraining\CMakeFiles\opencv_performance.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : apps/haartraining/CMakeFiles/opencv_performance.dir/depend

