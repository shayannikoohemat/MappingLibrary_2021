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
include modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/depend.make

# Include the progress variables for this target.
include modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/progress.make

# Include the compile flags for this target's objects.
include modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/flags.make

modules/video/opencv_perf_video_pch_dephelp.cxx: modules/video/perf/perf_precomp.hpp
	$(CMAKE_COMMAND) -E cmake_progress_report D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating opencv_perf_video_pch_dephelp.cxx"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E echo #include \"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/perf/perf_precomp.hpp\" > D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/opencv_perf_video_pch_dephelp.cxx
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E echo "int testfunction();" >> D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/opencv_perf_video_pch_dephelp.cxx
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E echo "int testfunction()" >> D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/opencv_perf_video_pch_dephelp.cxx
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E echo { >> D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/opencv_perf_video_pch_dephelp.cxx
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E echo "    return 0;" >> D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/opencv_perf_video_pch_dephelp.cxx
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E echo } >> D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/opencv_perf_video_pch_dephelp.cxx

modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.obj: modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/flags.make
modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.obj: modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/includes_CXX.rsp
modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.obj: modules/video/opencv_perf_video_pch_dephelp.cxx
	$(CMAKE_COMMAND) -E cmake_progress_report D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.obj"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video && C:\Dev-Cpp\MinGW64\bin\g++.exe   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles\opencv_perf_video_pch_dephelp.dir\opencv_perf_video_pch_dephelp.cxx.obj -c D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video\opencv_perf_video_pch_dephelp.cxx

modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.i"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video && C:\Dev-Cpp\MinGW64\bin\g++.exe  $(CXX_DEFINES) $(CXX_FLAGS) -E D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video\opencv_perf_video_pch_dephelp.cxx > CMakeFiles\opencv_perf_video_pch_dephelp.dir\opencv_perf_video_pch_dephelp.cxx.i

modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.s"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video && C:\Dev-Cpp\MinGW64\bin\g++.exe  $(CXX_DEFINES) $(CXX_FLAGS) -S D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video\opencv_perf_video_pch_dephelp.cxx -o CMakeFiles\opencv_perf_video_pch_dephelp.dir\opencv_perf_video_pch_dephelp.cxx.s

modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.obj.requires:
.PHONY : modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.obj.requires

modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.obj.provides: modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.obj.requires
	$(MAKE) -f modules\video\CMakeFiles\opencv_perf_video_pch_dephelp.dir\build.make modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.obj.provides.build
.PHONY : modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.obj.provides

modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.obj.provides.build: modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.obj

# Object files for target opencv_perf_video_pch_dephelp
opencv_perf_video_pch_dephelp_OBJECTS = \
"CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.obj"

# External object files for target opencv_perf_video_pch_dephelp
opencv_perf_video_pch_dephelp_EXTERNAL_OBJECTS =

lib/libopencv_perf_video_pch_dephelp.a: modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.obj
lib/libopencv_perf_video_pch_dephelp.a: modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/build.make
lib/libopencv_perf_video_pch_dephelp.a: modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library ..\..\lib\libopencv_perf_video_pch_dephelp.a"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video && $(CMAKE_COMMAND) -P CMakeFiles\opencv_perf_video_pch_dephelp.dir\cmake_clean_target.cmake
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\opencv_perf_video_pch_dephelp.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/build: lib/libopencv_perf_video_pch_dephelp.a
.PHONY : modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/build

modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/requires: modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/opencv_perf_video_pch_dephelp.cxx.obj.requires
.PHONY : modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/requires

modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/clean:
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video && $(CMAKE_COMMAND) -P CMakeFiles\opencv_perf_video_pch_dephelp.dir\cmake_clean.cmake
.PHONY : modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/clean

modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/depend: modules/video/opencv_perf_video_pch_dephelp.cxx
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4 D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4 D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video\CMakeFiles\opencv_perf_video_pch_dephelp.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : modules/video/CMakeFiles/opencv_perf_video_pch_dephelp.dir/depend

