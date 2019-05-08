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
#include <math.h>
#include "InlineArguments.h"
#include "Image.h"

using namespace std;

void PrintUsage()
{
  printf("Determine the pixel value range of a flost image.\n\n");
  printf("Usage: floatrange -i <input image>\n");
}
  
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  Image image;
  float *pixel, gv_min, gv_max;

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input file
  if (!args->Contains("-i")) {
    printf("Error: no input image specified with -i <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Read the input image
  if (!image.Read(args->String("-i"))) {
    printf("Error reading first input image from %s\n", args->String("-i"));
    exit(0);
  }
  
  gv_min = gv_max = *(image.float_begin());
  for (pixel=image.float_begin()+1; pixel!=image.float_end(); pixel++) {
    if (*pixel < gv_min) gv_min = *pixel;
    if (*pixel > gv_max) gv_max = *pixel;
  }
  printf("Minimum pixel value %12.6f\n", gv_min);
  printf("Maximum pixel value %12.6f\n", gv_max);

  exit(0);
}
