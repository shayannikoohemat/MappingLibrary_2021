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

extern "C" int model2obj_c(char *, char *, char *);

void PrintUsage()
{
  printf("Transformation of model coordinates to object coordinates using\n");
  printf("the specified absolute orientation\n\n");
  printf("Usage: model2obj -i <model point file>\n");
  printf("                 -o <object point file>\n");
  printf("                 -a <absolute orientation file>\n");
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
    printf("Error: model points should be specified with -i\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: object points should be specified with -o\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-a")) {
    printf("Error: absolute orientation should be specified with -a\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Call the model2obj C routine
  model2obj_c(args->String("-i"), args->String("-o"), args->String("-a"));
  
  return EXIT_SUCCESS;
}
