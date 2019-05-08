
/*
    Copyright 2010 University of Twente and Delft University of Technology
 
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
  printf("indoorregistration estimates the relative pose between 2D laser scanners\n\n");
  printf("Usage: indoorregistration\n");
  printf("            -i0 <observations sensor 0>\n");
  printf("            -i1 <observations sensor 1>\n");
  printf("            -i2 <observations sensor 2>\n");
  printf("            -c0 <calibration sensor 0>\n");
  printf("            -c1 <calibration sensor 1>\n");
  printf("            -c2 <calibration sensor 2>\n");
  printf("            -o <registration parameters>\n");
  printf("            [-r <sensor rotation in degrees, default: 20>]\n");
  printf("            [-n <max number iterations (default: 10)]\n");
  printf("            [-mp] <max number of planes, default: 3>]\n");
  printf("            [-v verbose]\n");
  printf("            [-scin <input system configuration file name>\n");
  printf("            [-per use perpendicularity constraint\n");
  printf("            [-fr laser data in frame coordinate system\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void indoorregistration_cpp(char *, char *, char *, 
                              char * ,char *, char *,
							  double, bool, int, int, char *, char *, bool, bool);

  if (args->Contains("-usage")) {
  	PrintUsage();
  	exit(0);
  }  
  
  if (!args->Contains("-i0")) {
  	printf("Error: no input file specified with -i0.\n");
  	PrintUsage();
  	exit(0);
  }
  if (!args->Contains("-i1")) {
  	printf("Error: no input file specified with -i1.\n");
  	PrintUsage();
  	exit(0);
  }
  if (!args->Contains("-i2")) {
  	printf("Error: no input file specified with -i2.\n");
  	PrintUsage();
  	exit(0);
  }
  if (!args->Contains("-c0") && !args->Contains("-scin")) {
  	printf("Error: no calibration file specified for sensor 0 with -c0.\n");
  	PrintUsage();
  	exit(0);
  }
  if (!args->Contains("-c1") && !args->Contains("-scin")) {
  	printf("Error: no calibration file specified for sensor 1 with -c1.\n");
  	PrintUsage();
  	exit(0);
  }
  if (!args->Contains("-c2") && !args->Contains("-scin")) {
  	printf("Error: no calibration file specified for sensor 2 with -c2.\n");
  	PrintUsage();
  	exit(0);
  }
  if (!args->Contains("-o")) {
  	printf("Error: no output file specified with -o.\n");
  	PrintUsage();
  	exit(0);
  }
  if (args->Integer("-mp", 3) > 3 || args->Integer("-mp", 3) < 1) {
  	printf("Error: Number of planes should be between 1 and 3\n");
  	PrintUsage();
  	exit(0);
  }

  indoorregistration_cpp(args->String("-i0"), args->String("-i1"),
                         args->String("-i2"),
                         args->String("-c0"), args->String("-c1"),
                         args->String("-c2"),
                         args->Double("-r", 20.0),
						 args->Contains("-v"), args->Integer("-n", 10),
						 args->Integer("-mp", 3),
						 args->String("-o"),args->String("-scin"),
						 args->Contains("-per"),args->Contains("-fr"));

  return EXIT_SUCCESS;
}
