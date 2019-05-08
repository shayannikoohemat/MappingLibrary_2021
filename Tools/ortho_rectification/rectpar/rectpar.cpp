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

extern "C" int rectpar_c(char *, char *, char *, char *, int, double, int);

void PrintUsage()
{
  printf("Robust estimation of rectification parameters (2D-2D projective\n");
  printf("transformation). Transformation can be between an image and a\n");
  printf("horizontal plane in object space, or between two image coordinate systems.\n\n");
  printf("Usage: rectpar -i <Image point measured in original image>\n");
  printf("               -o <Transformation parameters from image to object space>\n");
  printf("               EITHER\n");
  printf("               -c <Control points in reference coordinate system\n");
  printf("               OR\n");
  printf("               -iref <Image points in reference coordinate system\n");
  printf("               [-minp <Minimum number of points in estimation, default: 4>]\n");
  printf("               [-maxrms <Maximum RMS value of residuals in pixels, default: 1>]\n");
  printf("               [-maxiter <Minimum number of iterations, default: 5>]\n");
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
    printf("Error: Output file for transformation parameters should be specified with -o\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-c") && !args->Contains("-iref")) {
    printf("Error: Reference points should be specified with either -c or -iref\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (args->Contains("-c") && args->Contains("-iref")) {
    printf("Error: -c and -iref are mutually exclusive.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
    
  // Call the rectpar C routine
  rectpar_c(args->String("-i"), args->String("-o"),
            args->String("-c"), args->String("-iref"),
            args->Integer("-minp", 4), args->Double("-maxrms", 1.0),
            args->Integer("-maxiter", 5));
  
  return EXIT_SUCCESS;
}
