/*
                      Copyright 2013 University of Twente
 
       This file is part of the Mapping libraries and tools, developed
  for research, education and projects in photogrammetry and laser scanning.

  The Mapping libraries and tools are free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the License,
                   or (at your option) any later version.

 The Mapping libraries and tools are distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
          along with the Mapping libraries and tools.  If not, see
                      <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------*/

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("indoorslam reconstructs frame position and scene planes from\n");
  printf("point clouds in a three scanner frame.\n");
  printf("Input data can be specified through either a name of a block file\n");
  printf("with the meta data of strips with data of the three scanners or by\n");
  printf("the names of three laser point files\n");
  printf("Usage: indoorslam [-b <block with data of the three sensors>]\n");
  printf("                  OR\n");
  printf("                  [-i0 <point cloud sensor 0>]\n");
  printf("                  [-i1 <point cloud sensor 1>]\n");
  printf("                  [-i2 <point cloud sensor 2>]\n");
  printf("          optional parameters:\n");
  printf("                  [-rapp <appendix for reconstructed 3D point cloud file names>]\n");
  printf("                  [-rdir] <output directory for reconstructed 3D point cloud>\n");
  printf("                  [-pl <reconstructed planes>]\n");
  printf("                  [-t <reconstructed frame trajectory>]\n");
  printf("                  [-idir] <output directory for results per interval>\n");
  printf("                  [-nlc <number of scans> (no loop closure after X scans\n");
  printf("                  [-wo <weight of offset constraint> (def: 1.0)\n");
  printf("                  [-wc <weight of curvature constraint> (def: 1.0)\n");
  printf("                  [-ol (use old label numbers)\n");
  printf("                  [-dmax <maximum distance of segment to plane> (def: 0.1 m)\n");
  printf("                  [-scin <system configuration file needed for self calibration>\n");
  printf("                  [-s5 <only estimate 5 registration self calibration parameters per sensor\n");
  printf("                  [-scout <updated system configuration after self calibration>\n");
  printf("                  [-tmin <start time for processing> (def: 0.0)\n");
  printf("                  [-tmax <end time for processing> (def: 0.0 = all data)\n");
  printf("                  [-2d (only 2D splines are estimated)]\n");
  printf("                  [-icp (use ICP to check and correct SLAM result)]\n");
  printf("                  [-nil <number of intervals for local estimation> (def: 3)\n");
  printf("                  [-nis <number of intervals for estimation in a section> (def: 20)\n");
}
 
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void IndoorSLAMProcess(char *, char *, char *, char *, char *, char *, char *,
                         char *, int, char *, double, double, bool, double,
					     char *, bool, char *, double, double, bool, bool,
					     int, int);

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }

  if (!args->Contains("-b") &&
      !(args->Contains("-i0") && args->Contains("-i1") &&
        args->Contains("-i2"))) {
    printf("Error: Input files should be specified with either -b or -i0, -i1 and -i2.\n");
    PrintUsage();
    exit(0);
  }
  
  if (args->Contains("-scout") && !args->Contains("-scin")) {
  	printf("Error: Output of self calibration requires approximate sensor configuration with -scin\n");
  	PrintUsage();
  	exit(0);
  }

  IndoorSLAMProcess(args->String("-b"), args->String("-i0"), args->String("-i1"),
                    args->String("-i2"), args->String("-rapp"), args->String("-t"),
				    args->String("-rdir"), args->String("-idir"),
				    args->Integer("-nlc", 0),
				    args->String("-pl"), args->Double("-wo", 1.0),
				    args->Double("-wc", 1.0), args->Contains("-ol"),
				    args->Double("-dmax", 0.1), args->String("-scin"),
				    args->Contains("-s5"), args->String("-scout"),
				    args->Double("-tmin", 0.0), args->Double("-tmax", 0.0),
				    args->Contains("-2d"), args->Contains("-icp"),
				    args->Integer("-nil", 3), args->Integer("-nis", 20));

  return EXIT_SUCCESS;
}
