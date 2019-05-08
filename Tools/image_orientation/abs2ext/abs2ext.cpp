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
#include "InlineArguments.h"
#include <stdio.h>

using namespace std;

extern "C" int abs2ext_c(char *, char *, char *, char *);

void PrintUsage()
{
  printf("Conversion of absolute and relative orientation parameters to\n");
  printf("the exterior orientation parameters of two images\n\n");
  printf("Usage: abs2ext -abs <Absolute orientation parameters>\n");
  printf("               -rel <Relative orientation parameters>\n");
  printf("               -ext1 <Exterior orientation parameters of first image>\n");
  printf("               -ext2 <Exterior orientation parameters of second image>\n");
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

  if (!args->Contains("-abs")) {
    printf("Error: Absolute orientation file should be specified with -abs.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-rel")) {
    printf("Error: Relative orientation file should be specified with -rel.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-ext1") || !args->Contains("-ext2")) {
    printf("Error: Exterior orientation files should be specified with -ext1 and -ext2.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

// Call the abs2ext C routine

  abs2ext_c(args->String("-abs"), args->String("-rel"), args->String("-ext1"),
            args->String("-ext2"));
  
  return EXIT_SUCCESS;
}
