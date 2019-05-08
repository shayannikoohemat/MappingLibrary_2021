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

extern "C" int rectpts_c(char *, char *, char *, char *);

void PrintUsage()
{
  printf("Projective transformation of image points to either another image coordinate\n");
  printf("system or a horizontal plane in object space. The (rectification) parameters\n");
  printf("can be calculated with the program rectpar.\n\n");
  printf("Usage: rectpts -i <Image points in original image>\n");
  printf("               -par <Rectification parameters>\n");
  printf("               EITHER\n");
  printf("               -oo <Rectified 2D object points\n");
  printf("               OR\n");
  printf("               -oi <Rectified image points>\n");
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
    printf("Error: Image points in original image should be specified with -i\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-par")) {
    printf("Error: Rectification parameters should be specified with -par\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-oo") && !args->Contains("-oi")) {
    printf("Error: Rectified points should be specified with -oo or -oi\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (args->Contains("-oi") && args->Contains("-oo")) {
    printf("Error: Arguments -oo and -oi are mutually exclusive.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
    
  // Call the rectpts C routine
  rectpts_c(args->String("-i"), args->String("-par"),
            args->String("-oo"), args->String("-oi"));
  
  return EXIT_SUCCESS;
}
