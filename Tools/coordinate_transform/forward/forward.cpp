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

extern "C" int forward_c(char *, char *, char *, char *,
                         char *, char *, char *, char *);

void PrintUsage()
{
  printf("Forward intersection of camera points from two images.\n\n");
  printf("Usage: forward -i1 <Camera points of first image>\n");
  printf("               -i2 <Camera points of second image>\n");
  printf("               -int1 <Interior orientation of first image>\n");
  printf("               [-int2 <Interior orientation of second image>]\n");
  printf("               EITHER\n");
  printf("               -ext1 <Exterior orientation of first image>\n");
  printf("               -ext2 <Exterior orientation of second image>\n");
  printf("               OR\n");
  printf("               -rel <Relative orientation parameters>\n");
  printf("               -o Calculated 3D points. In case exterior orientation is\n");
  printf("                  supplied, these are object points. In case relative\n");
  printf("                  orientation is supplied, model points are calculated.\n");
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
    printf("Error: Camera points should be specified with -i1 and -i2.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: No output file specified with -o.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-int1")) {
    printf("Error: Interior orientation of first image must be specified with -int1.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-rel") && !(args->Contains("-ext1") && args->Contains("-i2"))) {
    printf("Error: Either specify exterior orientations with -ext1 and -ext2, or\n");
    printf("       the relative orientation with -rel.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (args->Contains("-rel") && (args->Contains("-ext1") || args->Contains("-ext2"))) {
    printf("Error: -rel is mutually exclusive with -ext1 and -ext2.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

// Call the forward C routine

  if (args->Contains("-int2"))
    forward_c(args->String("-i1"), args->String("-i2"),
              args->String("-int1"), args->String("-int2"),
              args->String("-ext1"), args->String("-ext2"),
              args->String("-rel"), args->String("-o"));
  else
    forward_c(args->String("-i1"), args->String("-i2"),
              args->String("-int1"), args->String("-int1"),
              args->String("-ext1"), args->String("-ext2"),
              args->String("-rel"), args->String("-o"));
  
  return EXIT_SUCCESS;
}
