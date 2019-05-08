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

extern "C" int rectify_c(char *, char *, char *, char *,
                         double, int, double, double, double, double,
                         int, char *);

void PrintUsage()
{
  printf("Rectification of an image. If bounds are specified in object space\n");
  printf("only this part is rectified. Otherwise, the complete image will be\n");
  printf("rectified.\n\n");
  printf("Usage: rectify -i <original image>\n");
  printf("               -imgpts <image point measured in original image>\n");
  printf("               -ctrlpts <control points in object space\n");
  printf("               -o <rectified image>\n");
  printf("               -pix_size <pixel size (in meters)>\n");
  printf("               [-xmin <minimum X-coordinate in rectified image>]\n");
  printf("               [-ymin <minimum Y-coordinate in rectified image>]\n");
  printf("               [-xmax <maximum X-coordinate in rectified image>]\n");
  printf("               [-ymax <maximum Y-coordinate in rectified image>]\n");
  printf("               [-in] (interpolation method: 1: nearest neighbour,\n");
  printf("                      2: bilinear (default), 3: cubic convolution)\n");
  printf("               [-par <transformation from image to object space>\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  
  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input and output files
  if (!args->Contains("-i")) {
    printf("Error: original image should be specified with -i\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: rectified image should be specified with -o\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-imgpts")) {
    printf("Error: image points measured in original image should be specified with -imgpts\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-ctrlpts")) {
    printf("Error: control points should be specified with -ctrlpts\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-pix_size")) {
    printf("Error: pixel size should be specified with -pix_size\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (args->Contains("-xmin") || args->Contains("-xmax") ||
      args->Contains("-ymin") || args->Contains("-ymax"))
    if (!args->Contains("-xmin") || !args->Contains("-ymin") ||
        !args->Contains("-xmax") || !args->Contains("-ymax")) {
      printf("Error: for specification of the bounds of the rectified part all\n");
      printf("       four bounds should be given using -xmin, -ymin, -xmax, and -ymax\n");
      return EXIT_SUCCESS;
    }
    
  // Call the rectify C routine
  rectify_c(args->String("-i"), args->String("-imgpts"),
            args->String("-ctrlpts"), args->String("-o"),
            args->Double("-pix_size", 1.0), args->Contains("-xmin"),
            args->Double("-xmin", 0.0), args->Double("-ymin", 0.0),
            args->Double("-xmax", 1.0), args->Double("-ymax", 1.0),
            args->Integer("-in", 2), args->String("-par"));
  
  return EXIT_SUCCESS;
}
