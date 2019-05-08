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
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("Conversion of an image to a VRML elevation grid.\n\n");
  printf("Usage: viff2vrml -i <viff file> [-grid <grid file>]\n");
  printf("       [-r <red, def:1.0>] [-g <green, def:1.0>]\n");
  printf("       [-b <blue, def:1.0>] -o <vrml file>\n");
}
  
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void viff2vrml_cpp(char *, char *, double, double, double, char *);

// Usage

  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
// Check on required input and output files

  if (!args->Contains("-i")) {
    printf("Error: no input viff file specified with -i <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: no output vrml file specified with -o <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }


  // Output to a VRML file
  viff2vrml_cpp(args->String("-i"), args->String("-grid"),
                args->Double("-r", 1.0), args->Double("-g", 1.0),
                args->Double("-b", 1.0), args->String("-o"));

  return EXIT_SUCCESS;
}
