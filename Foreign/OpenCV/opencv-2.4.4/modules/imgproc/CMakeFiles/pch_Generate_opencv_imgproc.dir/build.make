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

# Utility rule file for pch_Generate_opencv_imgproc.

# Include the progress variables for this target.
include modules/imgproc/CMakeFiles/pch_Generate_opencv_imgproc.dir/progress.make

modules/imgproc/CMakeFiles/pch_Generate_opencv_imgproc: modules/imgproc/precomp.hpp.gch/opencv_imgproc_Release.gch

modules/imgproc/precomp.hpp.gch/opencv_imgproc_Release.gch: modules/imgproc/src/precomp.hpp
modules/imgproc/precomp.hpp.gch/opencv_imgproc_Release.gch: modules/imgproc/precomp.hpp
modules/imgproc/precomp.hpp.gch/opencv_imgproc_Release.gch: lib/libopencv_imgproc_pch_dephelp.a
	$(CMAKE_COMMAND) -E cmake_progress_report D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating precomp.hpp.gch/opencv_imgproc_Release.gch"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\imgproc && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E make_directory D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/imgproc/precomp.hpp.gch
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\imgproc && C:\Dev-Cpp\MinGW64\bin\g++.exe -O2 -DNDEBUG -DNDEBUG -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/core/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/imgproc" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/imgproc/src" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/imgproc/include" -isystem"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4" -DHAVE_CVCONFIG_H -DCVAPI_EXPORTS -DHAVE_CVCONFIG_H -mstackrealign -W -Wall -Werror=return-type -Werror=address -Werror=sequence-point -Wformat -Werror=format-security -Wmissing-declarations -Wundef -Winit-self -Wpointer-arith -Wshadow -Wsign-promo -Wno-narrowing -Wno-delete-non-virtual-dtor -fdiagnostics-show-option -fomit-frame-pointer -msse -msse2 -ffunction-sections -x c++-header -o D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/imgproc/precomp.hpp.gch/opencv_imgproc_Release.gch D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/imgproc/precomp.hpp

modules/imgproc/precomp.hpp: modules/imgproc/src/precomp.hpp
	$(CMAKE_COMMAND) -E cmake_progress_report D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating precomp.hpp"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\imgproc && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E copy D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/imgproc/src/precomp.hpp D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/imgproc/precomp.hpp

pch_Generate_opencv_imgproc: modules/imgproc/CMakeFiles/pch_Generate_opencv_imgproc
pch_Generate_opencv_imgproc: modules/imgproc/precomp.hpp.gch/opencv_imgproc_Release.gch
pch_Generate_opencv_imgproc: modules/imgproc/precomp.hpp
pch_Generate_opencv_imgproc: modules/imgproc/CMakeFiles/pch_Generate_opencv_imgproc.dir/build.make
.PHONY : pch_Generate_opencv_imgproc

# Rule to build all files generated by this target.
modules/imgproc/CMakeFiles/pch_Generate_opencv_imgproc.dir/build: pch_Generate_opencv_imgproc
.PHONY : modules/imgproc/CMakeFiles/pch_Generate_opencv_imgproc.dir/build

modules/imgproc/CMakeFiles/pch_Generate_opencv_imgproc.dir/clean:
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\imgproc && $(CMAKE_COMMAND) -P CMakeFiles\pch_Generate_opencv_imgproc.dir\cmake_clean.cmake
.PHONY : modules/imgproc/CMakeFiles/pch_Generate_opencv_imgproc.dir/clean

modules/imgproc/CMakeFiles/pch_Generate_opencv_imgproc.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4 D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\imgproc D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4 D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\imgproc D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\imgproc\CMakeFiles\pch_Generate_opencv_imgproc.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : modules/imgproc/CMakeFiles/pch_Generate_opencv_imgproc.dir/depend

