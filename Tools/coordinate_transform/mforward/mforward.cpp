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

extern "C" int mforward_c(char *, char *, char *, char *);

void PrintUsage()
{
  printf("Forward intersection of image points from multiple images.\n\n");
  printf("Usage: mforward -imgpts <File filter of image points>\n");
  printf("                -ext <File filter of exterior orientations>\n");
  printf("                -int <File filter of interior orientations>\n");
  printf("                -objpts <Object points computed by forward intersection\n");
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
  if (!args->Contains("-imgpts")) {
    printf("Error: Image point file filter should be specified with -imgpts.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-objpts")) {
    printf("Error: No output file specified with -objpts.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-int")) {
    printf("Error: Interior orientation file filter must be specified with -int.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-ext")) {
    printf("Error: Exterior orientation file filter must be specified with -ext.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Call the mforward C routine
  mforward_c(args->String("-imgpts"), args->String("-ext"),
             args->String("-int"), args->String("-objpts"));
  
  return EXIT_SUCCESS;
}
