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
include modules/features2d/CMakeFiles/opencv_perf_features2d.dir/depend.make

# Include the progress variables for this target.
include modules/features2d/CMakeFiles/opencv_perf_features2d.dir/progress.make

# Include the compile flags for this target's objects.
include modules/features2d/CMakeFiles/opencv_perf_features2d.dir/flags.make

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.obj: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/flags.make
modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.obj: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/includes_CXX.rsp
modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.obj: modules/features2d/perf/perf_batchDistance.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.obj"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d && C:\Dev-Cpp\MinGW64\bin\g++.exe   $(CXX_DEFINES) $(CXX_FLAGS)  -include "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/perf_precomp.hpp" -Winvalid-pch  -o CMakeFiles\opencv_perf_features2d.dir\perf\perf_batchDistance.cpp.obj -c D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d\perf\perf_batchDistance.cpp

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.i"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d && C:\Dev-Cpp\MinGW64\bin\g++.exe  $(CXX_DEFINES) $(CXX_FLAGS)  -include "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/perf_precomp.hpp" -Winvalid-pch  -E D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d\perf\perf_batchDistance.cpp > CMakeFiles\opencv_perf_features2d.dir\perf\perf_batchDistance.cpp.i

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.s"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d && C:\Dev-Cpp\MinGW64\bin\g++.exe  $(CXX_DEFINES) $(CXX_FLAGS)  -include "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/perf_precomp.hpp" -Winvalid-pch  -S D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d\perf\perf_batchDistance.cpp -o CMakeFiles\opencv_perf_features2d.dir\perf\perf_batchDistance.cpp.s

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.obj.requires:
.PHONY : modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.obj.requires

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.obj.provides: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.obj.requires
	$(MAKE) -f modules\features2d\CMakeFiles\opencv_perf_features2d.dir\build.make modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.obj.provides.build
.PHONY : modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.obj.provides

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.obj.provides.build: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.obj

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.obj: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/flags.make
modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.obj: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/includes_CXX.rsp
modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.obj: modules/features2d/perf/perf_fast.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.obj"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d && C:\Dev-Cpp\MinGW64\bin\g++.exe   $(CXX_DEFINES) $(CXX_FLAGS)  -include "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/perf_precomp.hpp" -Winvalid-pch  -o CMakeFiles\opencv_perf_features2d.dir\perf\perf_fast.cpp.obj -c D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d\perf\perf_fast.cpp

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.i"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d && C:\Dev-Cpp\MinGW64\bin\g++.exe  $(CXX_DEFINES) $(CXX_FLAGS)  -include "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/perf_precomp.hpp" -Winvalid-pch  -E D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d\perf\perf_fast.cpp > CMakeFiles\opencv_perf_features2d.dir\perf\perf_fast.cpp.i

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.s"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d && C:\Dev-Cpp\MinGW64\bin\g++.exe  $(CXX_DEFINES) $(CXX_FLAGS)  -include "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/perf_precomp.hpp" -Winvalid-pch  -S D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d\perf\perf_fast.cpp -o CMakeFiles\opencv_perf_features2d.dir\perf\perf_fast.cpp.s

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.obj.requires:
.PHONY : modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.obj.requires

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.obj.provides: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.obj.requires
	$(MAKE) -f modules\features2d\CMakeFiles\opencv_perf_features2d.dir\build.make modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.obj.provides.build
.PHONY : modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.obj.provides

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.obj.provides.build: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.obj

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.obj: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/flags.make
modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.obj: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/includes_CXX.rsp
modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.obj: modules/features2d/perf/perf_main.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.obj"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d && C:\Dev-Cpp\MinGW64\bin\g++.exe   $(CXX_DEFINES) $(CXX_FLAGS)  -include "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/perf_precomp.hpp" -Winvalid-pch  -o CMakeFiles\opencv_perf_features2d.dir\perf\perf_main.cpp.obj -c D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d\perf\perf_main.cpp

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.i"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d && C:\Dev-Cpp\MinGW64\bin\g++.exe  $(CXX_DEFINES) $(CXX_FLAGS)  -include "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/perf_precomp.hpp" -Winvalid-pch  -E D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d\perf\perf_main.cpp > CMakeFiles\opencv_perf_features2d.dir\perf\perf_main.cpp.i

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.s"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d && C:\Dev-Cpp\MinGW64\bin\g++.exe  $(CXX_DEFINES) $(CXX_FLAGS)  -include "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/perf_precomp.hpp" -Winvalid-pch  -S D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d\perf\perf_main.cpp -o CMakeFiles\opencv_perf_features2d.dir\perf\perf_main.cpp.s

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.obj.requires:
.PHONY : modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.obj.requires

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.obj.provides: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.obj.requires
	$(MAKE) -f modules\features2d\CMakeFiles\opencv_perf_features2d.dir\build.make modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.obj.provides.build
.PHONY : modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.obj.provides

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.obj.provides.build: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.obj

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.obj: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/flags.make
modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.obj: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/includes_CXX.rsp
modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.obj: modules/features2d/perf/perf_orb.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.obj"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d && C:\Dev-Cpp\MinGW64\bin\g++.exe   $(CXX_DEFINES) $(CXX_FLAGS)  -include "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/perf_precomp.hpp" -Winvalid-pch  -o CMakeFiles\opencv_perf_features2d.dir\perf\perf_orb.cpp.obj -c D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d\perf\perf_orb.cpp

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.i"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d && C:\Dev-Cpp\MinGW64\bin\g++.exe  $(CXX_DEFINES) $(CXX_FLAGS)  -include "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/perf_precomp.hpp" -Winvalid-pch  -E D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d\perf\perf_orb.cpp > CMakeFiles\opencv_perf_features2d.dir\perf\perf_orb.cpp.i

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.s"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d && C:\Dev-Cpp\MinGW64\bin\g++.exe  $(CXX_DEFINES) $(CXX_FLAGS)  -include "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/perf_precomp.hpp" -Winvalid-pch  -S D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d\perf\perf_orb.cpp -o CMakeFiles\opencv_perf_features2d.dir\perf\perf_orb.cpp.s

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.obj.requires:
.PHONY : modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.obj.requires

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.obj.provides: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.obj.requires
	$(MAKE) -f modules\features2d\CMakeFiles\opencv_perf_features2d.dir\build.make modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.obj.provides.build
.PHONY : modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.obj.provides

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.obj.provides.build: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.obj

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.obj: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/flags.make
modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.obj: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/includes_CXX.rsp
modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.obj: modules/features2d/perf/perf_precomp.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.obj"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d && C:\Dev-Cpp\MinGW64\bin\g++.exe   $(CXX_DEFINES) $(CXX_FLAGS)  -include "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/perf_precomp.hpp" -Winvalid-pch  -o CMakeFiles\opencv_perf_features2d.dir\perf\perf_precomp.cpp.obj -c D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d\perf\perf_precomp.cpp

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.i"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d && C:\Dev-Cpp\MinGW64\bin\g++.exe  $(CXX_DEFINES) $(CXX_FLAGS)  -include "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/perf_precomp.hpp" -Winvalid-pch  -E D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d\perf\perf_precomp.cpp > CMakeFiles\opencv_perf_features2d.dir\perf\perf_precomp.cpp.i

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.s"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d && C:\Dev-Cpp\MinGW64\bin\g++.exe  $(CXX_DEFINES) $(CXX_FLAGS)  -include "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/perf_precomp.hpp" -Winvalid-pch  -S D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d\perf\perf_precomp.cpp -o CMakeFiles\opencv_perf_features2d.dir\perf\perf_precomp.cpp.s

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.obj.requires:
.PHONY : modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.obj.requires

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.obj.provides: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.obj.requires
	$(MAKE) -f modules\features2d\CMakeFiles\opencv_perf_features2d.dir\build.make modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.obj.provides.build
.PHONY : modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.obj.provides

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.obj.provides.build: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.obj

# Object files for target opencv_perf_features2d
opencv_perf_features2d_OBJECTS = \
"CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.obj" \
"CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.obj" \
"CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.obj" \
"CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.obj" \
"CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.obj"

# External object files for target opencv_perf_features2d
opencv_perf_features2d_EXTERNAL_OBJECTS =

bin/opencv_perf_features2d.exe: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.obj
bin/opencv_perf_features2d.exe: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.obj
bin/opencv_perf_features2d.exe: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.obj
bin/opencv_perf_features2d.exe: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.obj
bin/opencv_perf_features2d.exe: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.obj
bin/opencv_perf_features2d.exe: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/build.make
bin/opencv_perf_features2d.exe: lib/libopencv_core244.dll.a
bin/opencv_perf_features2d.exe: lib/libopencv_imgproc244.dll.a
bin/opencv_perf_features2d.exe: lib/libopencv_flann244.dll.a
bin/opencv_perf_features2d.exe: lib/libopencv_highgui244.dll.a
bin/opencv_perf_features2d.exe: lib/libopencv_features2d244.dll.a
bin/opencv_perf_features2d.exe: lib/libopencv_ts244.a
bin/opencv_perf_features2d.exe: lib/libopencv_highgui244.dll.a
bin/opencv_perf_features2d.exe: lib/libopencv_core244.dll.a
bin/opencv_perf_features2d.exe: lib/libopencv_imgproc244.dll.a
bin/opencv_perf_features2d.exe: lib/libopencv_flann244.dll.a
bin/opencv_perf_features2d.exe: lib/libopencv_highgui244.dll.a
bin/opencv_perf_features2d.exe: lib/libopencv_features2d244.dll.a
bin/opencv_perf_features2d.exe: lib/libopencv_flann244.dll.a
bin/opencv_perf_features2d.exe: lib/libopencv_highgui244.dll.a
bin/opencv_perf_features2d.exe: lib/libopencv_imgproc244.dll.a
bin/opencv_perf_features2d.exe: lib/libopencv_core244.dll.a
bin/opencv_perf_features2d.exe: 3rdparty/lib/libzlib.a
bin/opencv_perf_features2d.exe: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/objects1.rsp
bin/opencv_perf_features2d.exe: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable ..\..\bin\opencv_perf_features2d.exe"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\opencv_perf_features2d.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
modules/features2d/CMakeFiles/opencv_perf_features2d.dir/build: bin/opencv_perf_features2d.exe
.PHONY : modules/features2d/CMakeFiles/opencv_perf_features2d.dir/build

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/requires: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_batchDistance.cpp.obj.requires
modules/features2d/CMakeFiles/opencv_perf_features2d.dir/requires: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_fast.cpp.obj.requires
modules/features2d/CMakeFiles/opencv_perf_features2d.dir/requires: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_main.cpp.obj.requires
modules/features2d/CMakeFiles/opencv_perf_features2d.dir/requires: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_orb.cpp.obj.requires
modules/features2d/CMakeFiles/opencv_perf_features2d.dir/requires: modules/features2d/CMakeFiles/opencv_perf_features2d.dir/perf/perf_precomp.cpp.obj.requires
.PHONY : modules/features2d/CMakeFiles/opencv_perf_features2d.dir/requires

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/clean:
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d && $(CMAKE_COMMAND) -P CMakeFiles\opencv_perf_features2d.dir\cmake_clean.cmake
.PHONY : modules/features2d/CMakeFiles/opencv_perf_features2d.dir/clean

modules/features2d/CMakeFiles/opencv_perf_features2d.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4 D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4 D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\features2d\CMakeFiles\opencv_perf_features2d.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : modules/features2d/CMakeFiles/opencv_perf_features2d.dir/depend

