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

# Utility rule file for pch_Generate_opencv_perf_video.

# Include the progress variables for this target.
include modules/video/CMakeFiles/pch_Generate_opencv_perf_video.dir/progress.make

modules/video/CMakeFiles/pch_Generate_opencv_perf_video: modules/video/perf_precomp.hpp.gch/opencv_perf_video_Release.gch

modules/video/perf_precomp.hpp.gch/opencv_perf_video_Release.gch: modules/video/perf/perf_precomp.hpp
modules/video/perf_precomp.hpp.gch/opencv_perf_video_Release.gch: modules/video/perf_precomp.hpp
modules/video/perf_precomp.hpp.gch/opencv_perf_video_Release.gch: lib/libopencv_perf_video_pch_dephelp.a
	$(CMAKE_COMMAND) -E cmake_progress_report D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating perf_precomp.hpp.gch/opencv_perf_video_Release.gch"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E make_directory D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/perf_precomp.hpp.gch
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video && C:\Dev-Cpp\MinGW64\bin\g++.exe -O2 -DNDEBUG -DNDEBUG -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/perf" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/flann/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/imgproc/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/core/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/ts/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/imgproc/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/core/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/src" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/test" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/flann/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/imgproc/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/core/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/ts/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/imgproc/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/core/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/src" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/imgproc/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/core/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/src" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/include" -isystem"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4" -DHAVE_CVCONFIG_H -DCVAPI_EXPORTS -DHAVE_CVCONFIG_H -mstackrealign -W -Wall -Werror=return-type -Werror=address -Werror=sequence-point -Wformat -Werror=format-security -Wmissing-declarations -Wundef -Winit-self -Wpointer-arith -Wshadow -Wsign-promo -Wno-narrowing -Wno-delete-non-virtual-dtor -fdiagnostics-show-option -fomit-frame-pointer -msse -msse2 -ffunction-sections -x c++-header -o D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/perf_precomp.hpp.gch/opencv_perf_video_Release.gch D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/perf_precomp.hpp

modules/video/perf_precomp.hpp: modules/video/perf/perf_precomp.hpp
	$(CMAKE_COMMAND) -E cmake_progress_report D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating perf_precomp.hpp"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E copy D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/perf/perf_precomp.hpp D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/video/perf_precomp.hpp

pch_Generate_opencv_perf_video: modules/video/CMakeFiles/pch_Generate_opencv_perf_video
pch_Generate_opencv_perf_video: modules/video/perf_precomp.hpp.gch/opencv_perf_video_Release.gch
pch_Generate_opencv_perf_video: modules/video/perf_precomp.hpp
pch_Generate_opencv_perf_video: modules/video/CMakeFiles/pch_Generate_opencv_perf_video.dir/build.make
.PHONY : pch_Generate_opencv_perf_video

# Rule to build all files generated by this target.
modules/video/CMakeFiles/pch_Generate_opencv_perf_video.dir/build: pch_Generate_opencv_perf_video
.PHONY : modules/video/CMakeFiles/pch_Generate_opencv_perf_video.dir/build

modules/video/CMakeFiles/pch_Generate_opencv_perf_video.dir/clean:
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video && $(CMAKE_COMMAND) -P CMakeFiles\pch_Generate_opencv_perf_video.dir\cmake_clean.cmake
.PHONY : modules/video/CMakeFiles/pch_Generate_opencv_perf_video.dir/clean

modules/video/CMakeFiles/pch_Generate_opencv_perf_video.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4 D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4 D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\video\CMakeFiles\pch_Generate_opencv_perf_video.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : modules/video/CMakeFiles/pch_Generate_opencv_perf_video.dir/depend

