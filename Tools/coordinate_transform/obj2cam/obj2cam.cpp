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

extern "C" int obj2cam_c(char *, char *, char *, char *);

void PrintUsage()
{
  printf("Projection of object points into a camera coordinate system.\n\n");
  printf("Usage: obj2cam -i <Object point file>\n");
  printf("               -o <Camera point file>\n");
  printf("               -int <Interior orientation file>\n");
  printf("               -ext <Exterior orientation file>\n");
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
    printf("Error: Object points should be specified with -i\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: Camera points should be specified with -o\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-int")) {
    printf("Error: Interior orientation should be specified with -int\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-ext")) {
    printf("Error: Exterior orientation should be specified with -ext\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Call the obj2cam C routine
  obj2cam_c(args->String("-i"), args->String("-o"),
            args->String("-int"), args->String("-ext"));
  
  return EXIT_SUCCESS;
}
