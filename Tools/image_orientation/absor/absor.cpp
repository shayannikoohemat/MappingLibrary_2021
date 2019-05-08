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

extern "C" int absor_c(char *, char *, char *, int);

void PrintUsage()
{
  printf("Calculation of absolute orientation parameters (3D similarity\n");
  printf("transformation) in a least squares adjustment\n\n");
  printf("Usage: absor -ctrlpts <Object coordinates of control points>\n");
  printf("             -modelpts <Model coordinates of control points>\n");
  printf("             -o <Absolute orientation file> (output)\n");
  printf("             [-v] (Verbose, output of adjustment results)\n");
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

  if (!args->Contains("-ctrlpts")) {
    printf("Error: Object coordinates of control points should be specified with -ctrlpts\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-modelpts")) {
    printf("Error: Model coordinates of control points should be specified with -modelpts\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: Absolute orientation file should be specified with -o\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

// Call the absor C routine

  absor_c(args->String("-ctrlpts"), args->String("-modelpts"), args->String("-o"),
          args->Contains("-v"));
  
  return EXIT_SUCCESS;
}
