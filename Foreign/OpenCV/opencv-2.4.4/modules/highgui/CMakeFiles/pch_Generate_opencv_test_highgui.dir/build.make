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

# Utility rule file for pch_Generate_opencv_test_highgui.

# Include the progress variables for this target.
include modules/highgui/CMakeFiles/pch_Generate_opencv_test_highgui.dir/progress.make

modules/highgui/CMakeFiles/pch_Generate_opencv_test_highgui: modules/highgui/test_precomp.hpp.gch/opencv_test_highgui_Release.gch

modules/highgui/test_precomp.hpp.gch/opencv_test_highgui_Release.gch: modules/highgui/test/test_precomp.hpp
modules/highgui/test_precomp.hpp.gch/opencv_test_highgui_Release.gch: modules/highgui/test_precomp.hpp
modules/highgui/test_precomp.hpp.gch/opencv_test_highgui_Release.gch: lib/libopencv_test_highgui_pch_dephelp.a
	$(CMAKE_COMMAND) -E cmake_progress_report D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating test_precomp.hpp.gch/opencv_test_highgui_Release.gch"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\highgui && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E make_directory D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui/test_precomp.hpp.gch
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\highgui && C:\Dev-Cpp\MinGW64\bin\g++.exe -O2 -DNDEBUG -DNDEBUG -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui/test" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/features2d/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/flann/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/imgproc/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/core/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/ts/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/imgproc/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/core/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui/src" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/imgproc/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/core/include" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui/src" -I"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui/include" -isystem"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/libjasper" -isystem"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/libtiff" -isystem"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/libtiff" -isystem"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/libpng" -isystem"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/libjpeg" -isystem"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/zlib" -isystem"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/zlib" -isystem"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4" -isystem"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/openexr/Half" -isystem"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/openexr/Iex" -isystem"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/openexr/IlmThread" -isystem"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/openexr/Imath" -isystem"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/openexr/IlmImf" -isystem"D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/include" -DHAVE_CVCONFIG_H -DHAVE_JPEG -DHAVE_PNG -DHAVE_TIFF -DHAVE_JASPER -DHAVE_OPENEXR -DCVAPI_EXPORTS -DHIGHGUI_EXPORTS -DHAVE_CVCONFIG_H -mstackrealign -W -Wall -Werror=return-type -Werror=address -Werror=sequence-point -Wformat -Werror=format-security -Wmissing-declarations -Wundef -Winit-self -Wpointer-arith -Wshadow -Wsign-promo -Wno-narrowing -Wno-delete-non-virtual-dtor -fdiagnostics-show-option -fomit-frame-pointer -msse -msse2 -ffunction-sections -Wno-deprecated-declarations -x c++-header -o D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui/test_precomp.hpp.gch/opencv_test_highgui_Release.gch D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui/test_precomp.hpp

modules/highgui/test_precomp.hpp: modules/highgui/test/test_precomp.hpp
	$(CMAKE_COMMAND) -E cmake_progress_report D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold "Generating test_precomp.hpp"
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\highgui && "C:\Program Files (x86)\CMake 2.8\bin\cmake.exe" -E copy D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui/test/test_precomp.hpp D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/highgui/test_precomp.hpp

pch_Generate_opencv_test_highgui: modules/highgui/CMakeFiles/pch_Generate_opencv_test_highgui
pch_Generate_opencv_test_highgui: modules/highgui/test_precomp.hpp.gch/opencv_test_highgui_Release.gch
pch_Generate_opencv_test_highgui: modules/highgui/test_precomp.hpp
pch_Generate_opencv_test_highgui: modules/highgui/CMakeFiles/pch_Generate_opencv_test_highgui.dir/build.make
.PHONY : pch_Generate_opencv_test_highgui

# Rule to build all files generated by this target.
modules/highgui/CMakeFiles/pch_Generate_opencv_test_highgui.dir/build: pch_Generate_opencv_test_highgui
.PHONY : modules/highgui/CMakeFiles/pch_Generate_opencv_test_highgui.dir/build

modules/highgui/CMakeFiles/pch_Generate_opencv_test_highgui.dir/clean:
	cd /d D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\highgui && $(CMAKE_COMMAND) -P CMakeFiles\pch_Generate_opencv_test_highgui.dir\cmake_clean.cmake
.PHONY : modules/highgui/CMakeFiles/pch_Generate_opencv_test_highgui.dir/clean

modules/highgui/CMakeFiles/pch_Generate_opencv_test_highgui.dir/depend:
	$(CMAKE_COMMAND) -E cmake_depends "MinGW Makefiles" D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4 D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\highgui D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4 D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\highgui D:\Software\Mapping\Foreign\OpenCV\opencv-2.4.4\modules\highgui\CMakeFiles\pch_Generate_opencv_test_highgui.dir\DependInfo.cmake --color=$(COLOR)
.PHONY : modules/highgui/CMakeFiles/pch_Generate_opencv_test_highgui.dir/depend

