# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /opt/clion-2019.3.5/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /opt/clion-2019.3.5/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/shayan/MappingLibrary/Tools/sensor_fusion/lidar2panorama_kaios

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/shayan/MappingLibrary/Tools/sensor_fusion/build-lidar2panorama_kaios-Imported_Kit_42195a-Debug

# Include any dependencies generated for this target.
include CMakeFiles/lidar2panorama.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/lidar2panorama.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lidar2panorama.dir/flags.make

CMakeFiles/lidar2panorama.dir/main.cpp.o: CMakeFiles/lidar2panorama.dir/flags.make
CMakeFiles/lidar2panorama.dir/main.cpp.o: /home/shayan/MappingLibrary/Tools/sensor_fusion/lidar2panorama_kaios/main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shayan/MappingLibrary/Tools/sensor_fusion/build-lidar2panorama_kaios-Imported_Kit_42195a-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/lidar2panorama.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lidar2panorama.dir/main.cpp.o -c /home/shayan/MappingLibrary/Tools/sensor_fusion/lidar2panorama_kaios/main.cpp

CMakeFiles/lidar2panorama.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lidar2panorama.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shayan/MappingLibrary/Tools/sensor_fusion/lidar2panorama_kaios/main.cpp > CMakeFiles/lidar2panorama.dir/main.cpp.i

CMakeFiles/lidar2panorama.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lidar2panorama.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shayan/MappingLibrary/Tools/sensor_fusion/lidar2panorama_kaios/main.cpp -o CMakeFiles/lidar2panorama.dir/main.cpp.s

CMakeFiles/lidar2panorama.dir/equi_proj.cpp.o: CMakeFiles/lidar2panorama.dir/flags.make
CMakeFiles/lidar2panorama.dir/equi_proj.cpp.o: /home/shayan/MappingLibrary/Tools/sensor_fusion/lidar2panorama_kaios/equi_proj.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shayan/MappingLibrary/Tools/sensor_fusion/build-lidar2panorama_kaios-Imported_Kit_42195a-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/lidar2panorama.dir/equi_proj.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lidar2panorama.dir/equi_proj.cpp.o -c /home/shayan/MappingLibrary/Tools/sensor_fusion/lidar2panorama_kaios/equi_proj.cpp

CMakeFiles/lidar2panorama.dir/equi_proj.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lidar2panorama.dir/equi_proj.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shayan/MappingLibrary/Tools/sensor_fusion/lidar2panorama_kaios/equi_proj.cpp > CMakeFiles/lidar2panorama.dir/equi_proj.cpp.i

CMakeFiles/lidar2panorama.dir/equi_proj.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lidar2panorama.dir/equi_proj.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shayan/MappingLibrary/Tools/sensor_fusion/lidar2panorama_kaios/equi_proj.cpp -o CMakeFiles/lidar2panorama.dir/equi_proj.cpp.s

CMakeFiles/lidar2panorama.dir/lidar_projection.cpp.o: CMakeFiles/lidar2panorama.dir/flags.make
CMakeFiles/lidar2panorama.dir/lidar_projection.cpp.o: /home/shayan/MappingLibrary/Tools/sensor_fusion/lidar2panorama_kaios/lidar_projection.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/shayan/MappingLibrary/Tools/sensor_fusion/build-lidar2panorama_kaios-Imported_Kit_42195a-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/lidar2panorama.dir/lidar_projection.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/lidar2panorama.dir/lidar_projection.cpp.o -c /home/shayan/MappingLibrary/Tools/sensor_fusion/lidar2panorama_kaios/lidar_projection.cpp

CMakeFiles/lidar2panorama.dir/lidar_projection.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lidar2panorama.dir/lidar_projection.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/shayan/MappingLibrary/Tools/sensor_fusion/lidar2panorama_kaios/lidar_projection.cpp > CMakeFiles/lidar2panorama.dir/lidar_projection.cpp.i

CMakeFiles/lidar2panorama.dir/lidar_projection.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lidar2panorama.dir/lidar_projection.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/shayan/MappingLibrary/Tools/sensor_fusion/lidar2panorama_kaios/lidar_projection.cpp -o CMakeFiles/lidar2panorama.dir/lidar_projection.cpp.s

# Object files for target lidar2panorama
lidar2panorama_OBJECTS = \
"CMakeFiles/lidar2panorama.dir/main.cpp.o" \
"CMakeFiles/lidar2panorama.dir/equi_proj.cpp.o" \
"CMakeFiles/lidar2panorama.dir/lidar_projection.cpp.o"

# External object files for target lidar2panorama
lidar2panorama_EXTERNAL_OBJECTS =

lidar2panorama: CMakeFiles/lidar2panorama.dir/main.cpp.o
lidar2panorama: CMakeFiles/lidar2panorama.dir/equi_proj.cpp.o
lidar2panorama: CMakeFiles/lidar2panorama.dir/lidar_projection.cpp.o
lidar2panorama: CMakeFiles/lidar2panorama.dir/build.make
lidar2panorama: CMakeFiles/lidar2panorama.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/shayan/MappingLibrary/Tools/sensor_fusion/build-lidar2panorama_kaios-Imported_Kit_42195a-Debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Linking CXX executable lidar2panorama"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lidar2panorama.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lidar2panorama.dir/build: lidar2panorama

.PHONY : CMakeFiles/lidar2panorama.dir/build

CMakeFiles/lidar2panorama.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lidar2panorama.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lidar2panorama.dir/clean

CMakeFiles/lidar2panorama.dir/depend:
	cd /home/shayan/MappingLibrary/Tools/sensor_fusion/build-lidar2panorama_kaios-Imported_Kit_42195a-Debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/shayan/MappingLibrary/Tools/sensor_fusion/lidar2panorama_kaios /home/shayan/MappingLibrary/Tools/sensor_fusion/lidar2panorama_kaios /home/shayan/MappingLibrary/Tools/sensor_fusion/build-lidar2panorama_kaios-Imported_Kit_42195a-Debug /home/shayan/MappingLibrary/Tools/sensor_fusion/build-lidar2panorama_kaios-Imported_Kit_42195a-Debug /home/shayan/MappingLibrary/Tools/sensor_fusion/build-lidar2panorama_kaios-Imported_Kit_42195a-Debug/CMakeFiles/lidar2panorama.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lidar2panorama.dir/depend
