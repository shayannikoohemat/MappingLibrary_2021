
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

extern "C" int extor_c(char *, char *, char *, char *, int, char *);

void PrintUsage()
{
  printf("Calculation of exterior orientation by a spatial resection.\n");
  printf("Approximate values are calculated based on four points, distributed\n");
  printf("over the image (unless value are specified with -apext)\n");
  printf("After calculation of approximate values, all available points\n");
  printf("are used in a least squares estimation\n\n");
  printf("Usage: extor -campts <camera point file>\n");
  printf("             -ctrlpts <control points file>\n");
  printf("             -int <interior orientation file>\n");
  printf("             [-apext <approximate exterior orientation file>]\n");
  printf("             [-adjout] (output of adjustment information)\n");
  printf("             -ext <exterior orientation file> (output)\n");
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

  if (!args->Contains("-campts")) {
    printf("Error: camera points should be specified with -campts\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-ctrlpts")) {
    printf("Error: control points should be specified with -ctrlpts\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-int")) {
    printf("Error: interior orientation should be specified with -int\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-ext")) {
    printf("Error: exterior orientation should be specified with -ext\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

// Call the extor C routine

  extor_c(args->String("-campts"), args->String("-ctrlpts"),
          args->String("-int"), args->String("-apext"),
          args->Contains("-adjout"), args->String("-ext"));
  
  return EXIT_SUCCESS;
}
