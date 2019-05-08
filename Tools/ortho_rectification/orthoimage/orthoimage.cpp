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

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include "InlineArguments.h"

using namespace std;

extern "C" void orthoimage_c(const char *, const char *,
                             const char *, const char *,
                             float, float, float, const char *,
                             float, int, int, float, float, float, float);

void PrintUsage()
{
  printf("Calculation of an orthoimage.\n\n");
  printf("Usage: orthoimage -i <Input image>\n");
  printf("                  -io <Interior orientation file>\n");
  printf("                  -eo <Exterior orientation file>\n");
  printf("                  -dem <Digital elevation model>\n");
  printf("                  -x0dem <X-coordinate of lower left DEM corner>\n");
  printf("                  -y0dem <Y-coordinate of lower left DEM corner>\n");
  printf("                  -sdem <Pixel spacing of DEM>\n");
  printf("                  -o <Output ortho image>\n");
  printf("                  -so <Pixel spacing of ortho image>\n");
  printf("                  -area <Area for ortho image production>\n");
  printf("                      1 - the size of the image projected into the terrain\n");
  printf("                      2 - the size of the supplied DEM\n");
  printf("                      3 - the boundaries specified by the user\n");
  printf("                  -in <Interpolation method>\n");
  printf("                      1 - nearest neighbour\n");
  printf("                      2 - bilinear (default)\n");
  printf("                      3 - bicubic\n");
  printf("                  -xmin <Minimum X-coordinate in ortho image>\n");
  printf("                  -ymin <Minimum Y-coordinate in ortho image>\n");
  printf("                  -xmax <Maximum X-coordinate in ortho image>\n");
  printf("                  -ymax <Maximum Y-coordinate in ortho image>\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

// Usage

  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
// Check on required input and output parameters

  if (!args->Contains("-i")) {
    printf("Error: Input image should be specified with -i.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-io")) {
    printf("Error: Interior orientation should be specified with -io.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-eo")) {
    printf("Error: Exterior orientation should be specified with -eo.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-dem")) {
    printf("Error: Digital elevation model should be specified with -dem.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error;: Output image should be specified with -o.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-x0dem") || !args->Contains("-y0dem")) {
  	printf("Error: Lower left corner of DEM should be specified with -x0dem and -y0dem.\n");
  	PrintUsage();
  	return EXIT_SUCCESS;
  }
  if (!args->Contains("-sdem")) {
  	printf("Error: Pixel size of DEM should be specified with -sdem.\n");
  	PrintUsage();
  	return EXIT_SUCCESS;
  }
  if (!args->Contains("-so")) {
  	printf("Error: Pixel size of ortho image should be specified with -so.\n");
  	PrintUsage();
  	return EXIT_SUCCESS;
  }
  if (!args->Contains("-area")) {
  	printf("Error: The area for the ortho image should be specified with -area.\n");
  	PrintUsage();
  	return EXIT_SUCCESS;
  }
  else {
  	if (args->Integer("-area", 0) < 1 || args->Integer("-area", 0) > 3) {
  	  printf("Error: Invalid number for parameter -area\n");
  	  PrintUsage();
  	  return EXIT_SUCCESS;
  	}
  }
  if (args->Integer("-area", 0) == 3 &&
      (!args->Contains("-xmin") || !args->Contains("-xmax") ||
       !args->Contains("-ymin") || !args->Contains("-ymax"))) {
    printf("Error: for -area 3 all bounds need to be specified with parameters\n");
    printf("       -xmin, -xmax, -ymin, and -ymax\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Call the main orthoimage function in C
  orthoimage_c(args->String("-i"), args->String("-io"),
               args->String("-eo"), args->String("-dem"),
               args->Float("-x0dem", 0.0), args->Float("-y0dem", 0.0),
               args->Float("-sdem", 1.0),
               args->String("-o"),
               args->Float("-so", 1.0),
               args->Integer("-area", 1),
               args->Integer("-in", 2),
               args->Float("-xmin", 0.0),
               args->Float("-ymin", 0.0),
               args->Float("-xmax", 0.0),
               args->Float("-ymax", 0.0));
   
  return EXIT_SUCCESS;
}
