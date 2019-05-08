
/*
                    Copyright 2017 University of Twente 
 
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
#include "LaserPoints.h"
#include "LineSegments2D.h"

using namespace std;

void PrintUsage()
{
  printf("indoorcalibration estimates calibration parameters of a 2D laser scanner\n\n");
  printf("Usage: indoorcalibration\n");
  printf("            -i <text file with list of input laser files per scan>\n");
  printf("            -wpts <3D wall corner points>\n");
  printf("            -wtop <3D wall topology>\n");
  printf("            -id <scanner id>\n");
  printf("            -o <scanner meta data output file>\n");
  printf("            [-npar <number of calibration parameters>]\n");
  printf("\nThe number of calibration parameters can be one of the following:\n");
  printf("  1 - Only estimate range scale\n");
  printf("  2 - Estimate range scale and range offset\n");
  printf("  3 - Estimate range scale, range offset, and angle scale (default)\n");
}
 
int main(int argc, char *argv[])
{
  InlineArguments     *args = new InlineArguments(argc, argv);
  vector<LaserPoints> scans;
  FILE                *fd;
  char                *buffer, *filename;
  int                 len, npar;
  ObjectPoints        wall_points;
  LineTopologies      wall_tops;
  LineSegments2D      *walls;
  
  void indoorcalibration_cpp(vector<LaserPoints> &, LineSegments2D &, int,
                             char *, int);

  if (args->Contains("-usage")) {
  	PrintUsage();
  	exit(0);
  }  
  
  // Check input data
  if (!args->Contains("-i")) {
  	printf("Error: no input file specified with -i.\n");
  	PrintUsage();
  	exit(0);
  }
  if (!args->Contains("-id")) {
  	printf("Error: no scanner id specified with -id.\n");
  	PrintUsage();
  	exit(0);
  }
  if (!args->Contains("-wpts") || !args->Contains("-wtop")) {
  	printf("Error: no walls specified with -wpts and -wtop.\n");
  	PrintUsage();
  	exit(0);
  }
  if (!args->Contains("-o")) {
  	printf("Error: no output file specified with -o\n");
  	PrintUsage();
  	exit(0);
  }
  npar = args->Integer("-npar", 3);
  if (npar < 1 || npar > 3) {
  	printf("Error: number of parameters should be between 1 and 3\n");
  	PrintUsage();
  	exit(0);
  }

  // Read laser files
  if (!(fd = fopen(args->String("-i"), "r"))) {
  	printf("Error opening input file %s\n", args->String("-i"));
  	exit(0);
  }
  buffer = (char *) malloc(MAXCHARS);
  while ((filename = fgets(buffer, MAXCHARS, fd))) {
  	len = strlen(filename);
  	filename[len-1] = 0; // Remove end-of-line character
    scans.push_back(LaserPoints());
    if (!(scans.end()-1)->Read(filename, false)) {
      printf("Error reading file %s\n", filename);
      exit(0);
    }
    printf("Scan %d with %d points\n",
	       (scans.end()-1)->begin()->Attribute(ScanLineNumberTag),
		   (scans.end()-1)->size());
  }
  
  // Read the wall points and topology and convert to 2D line segments
  if (!wall_points.Read(args->String("-wpts"))) {
  	printf("Error reading wall points from %s\n", args->String("-wpts"));
  	exit(0);
  }
  if (!wall_tops.Read(args->String("-wtop"))) {
  	printf("Error reading wall topology from %s\n", args->String("-wtop"));
  	exit(0);
  }
  walls = new LineSegments2D(wall_points, wall_tops);
  
  // Call the main calibration function
  indoorcalibration_cpp(scans, *walls, args->Integer("-id", 0),
                        args->String("-o"), npar);

  return EXIT_SUCCESS;
}
