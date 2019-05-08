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

extern "C" int relor_c(char *, char *, char *, char *, int);

void PrintUsage()
{
  printf("Least squares estimation of parameters of the relative orientation between\n");
  printf("two images. Approximate values are calculated when 8 or more points are\n");
  printf("measured in both images. When specifying the switch -a, the normal case for\n");
  printf("aerial photography is assumed (no rotations, base line on X-axis with length 1).\n\n");
  printf("Usage: relor -i1 <Camera coordinates of points in left image>\n");
  printf("             -i2 <Camera coordinates of points in right image>\n");
  printf("             -int <Interior orientation file>\n");
  printf("             -o <Relative orientation file> (output)\n");
  printf("             [-a] Use standard approximate values\n");
  printf("                  If not specified, approximate values are derived\n");
  printf("                  by a direct solution\n");
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

  if (!args->Contains("-i1") || !args->Contains("-i2")) {
    printf("Error: Camera coordinates of left and right image points should be specified with -i1 and -i2\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-int")) {
    printf("Error: Interior orientation should be specified with -int\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: Relative orientation file should be specified with -o\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

// Call the relor C routine

  relor_c(args->String("-i1"), args->String("-i2"), args->String("-int"),
          args->String("-o"), args->Contains("-a"));
  
  return EXIT_SUCCESS;
}
