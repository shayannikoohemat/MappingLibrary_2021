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
include modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/depend.make

# Include the progress variables for this target.
include modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/progress.make

# Include the compile flags for this target's objects.
include modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/flags.make

modules/contrib/opencv_contrib_pch_dephelp.cxx: modules/contrib/src/precomp.hpp
	$(CMAKE_COMMAND) -E cmake_progress_report D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating opencv_contrib_pch_dephelp.cxx"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E echo #include \"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/contrib/src/precomp.hpp\" > D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/contrib/opencv_contrib_pch_dephelp.cxx
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E echo "int testfunction();" >> D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/contrib/opencv_contrib_pch_dephelp.cxx
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E echo "int testfunction()" >> D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/contrib/opencv_contrib_pch_dephelp.cxx
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E echo { >> D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/contrib/opencv_contrib_pch_dephelp.cxx
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E echo "    return 0;" >> D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/contrib/opencv_contrib_pch_dephelp.cxx
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E echo } >> D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/contrib/opencv_contrib_pch_dephelp.cxx

modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.obj: modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/flags.make
modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.obj: modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/includes_CXX.rsp
modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.obj: modules/contrib/opencv_contrib_pch_dephelp.cxx
	$(CMAKE_COMMAND) -E cmake_progress_report D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.obj"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib && C:\Dev-Cpp\MinGW64\bin\g++.exe   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles\opencv_contrib_pch_dephelp.dir\opencv_contrib_pch_dephelp.cxx.obj -c D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib\opencv_contrib_pch_dephelp.cxx

modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.i"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib && C:\Dev-Cpp\MinGW64\bin\g++.exe  $(CXX_DEFINES) $(CXX_FLAGS) -E D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib\opencv_contrib_pch_dephelp.cxx > CMakeFiles\opencv_contrib_pch_dephelp.dir\opencv_contrib_pch_dephelp.cxx.i

modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.s"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib && C:\Dev-Cpp\MinGW64\bin\g++.exe  $(CXX_DEFINES) $(CXX_FLAGS) -S D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib\opencv_contrib_pch_dephelp.cxx -o CMakeFiles\opencv_contrib_pch_dephelp.dir\opencv_contrib_pch_dephelp.cxx.s

modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.obj.requires:
.PHONY : modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.obj.requires

modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.obj.provides: modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.obj.requires
	$(MAKE) -f modules\contrib\CMakeFiles\opencv_contrib_pch_dephelp.dir\build.make modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.obj.provides.build
.PHONY : modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.obj.provides

modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.obj.provides.build: modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.obj

# Object files for target opencv_contrib_pch_dephelp
opencv_contrib_pch_dephelp_OBJECTS = \
"CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.obj"

# External object files for target opencv_contrib_pch_dephelp
opencv_contrib_pch_dephelp_EXTERNAL_OBJECTS =

lib/libopencv_contrib_pch_dephelp.a: modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.obj
lib/libopencv_contrib_pch_dephelp.a: modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/build.make
lib/libopencv_contrib_pch_dephelp.a: modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX static library ..\..\lib\libopencv_contrib_pch_dephelp.a"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib && $(CMAKE_COMMAND) -P CMakeFiles\opencv_contrib_pch_dephelp.dir\cmake_clean_target.cmake
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles\opencv_contrib_pch_dephelp.dir\link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/build: lib/libopencv_contrib_pch_dephelp.a
.PHONY : modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/build

modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/requires: modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/opencv_contrib_pch_dephelp.cxx.obj.requires
.PHONY : modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/requires

modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/clean:
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib && $(CMAKE_COMMAND) -P CMakeFiles\opencv_contrib_pch_dephelp.dir\cmake_clean.cmake
.PHONY : modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/clean

modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/depend: modules/contrib/opencv_contrib_pch_dephelp.cxx
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4 D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4 D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\contrib\CMakeFiles\opencv_contrib_pch_dephelp.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : modules/contrib/CMakeFiles/opencv_contrib_pch_dephelp.dir/depend

