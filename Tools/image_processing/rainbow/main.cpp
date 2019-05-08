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
#include <math.h>
#include "InlineArguments.h"
#include "Image.h"

using namespace std;

void PrintUsage()
{
  printf("Converting a grey value image to a colour image, e.g. used\n");
  printf("to generate colour coded height images.\n\n");
  printf("Usage: rainbow -i <input grey value image>\n");
  printf("               -o <output image with rainbow colour map>\n");
  printf("               -v <violet grey value>\n");
  printf("               -b <blue grey value>\n");
  printf("               -lb <light blue grey value>\n");
  printf("               -g <green grey value>\n");
  printf("               -y <yellow grey value>\n");
  printf("               -r <red grey value>\n");
  printf("               [-c <colour redution factor>]\n");
  printf("               [-grid <3D grid for conversion of data values to pixel values>\n");
}

extern "C" void rainbow(const char *, const char *,
                        double, double, double, double, double, double,
                        int, const char *, int);
                        
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
    
  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Call C interface
  rainbow(args->String("-i"),  args->String("-o"),
          args->Double("-v", 0.0), args->Double("-b", 51.0),
          args->Double("-lb", 102.0), args->Double("-g", 153.0), 
          args->Double("-y", 204.0), args->Double("-r", 255.0),
          args->Integer("-c", 1), args->String("-grid", NULL),
          args->Integer("-h", 0));
          
  return EXIT_SUCCESS;
}
