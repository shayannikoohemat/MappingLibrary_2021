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

extern "C" int pointx_c(char *, char *, int, int, double, int,
                        int, int, int, int);


void PrintUsage()
{
  printf("Extraction of points with the Foerstner operator.\n\n");
  printf("Usage: pointx -i <input image>\n");
  printf("              -o <extracted image points>\n");
  printf("              [-win <window size, default: 7 pixels>]\n");
  printf("              [-s <threshold for gradient strenght, default 10000>]\n");
  printf("              [-q <threshold for isotropy measure q, default 0.7>]\n");
  printf("              [-locmax <window size for local maximum detection, default: 7 pixels>]\n");
  printf("              [-r1 <first row of region of interest>]\n");
  printf("              [-c1 <first column of region of interest>]\n");
  printf("              [-nr <number of rows of region of interest>]\n");
  printf("              [-nc <number of columns of region of interest>]\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  int roi=0;
  
  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input and output files
  if (!args->Contains("-i")) {
    printf("Error: image  file should be specified with -i\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: output file for extracted points should be specified with -o\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (args->Contains("-r1") || args->Contains("-c1") ||
      args->Contains("-nr") || args->Contains("-nc")) {
    roi = 1;
    if (!args->Contains("-r1") || !args->Contains("-c1") ||
        !args->Contains("-nr") || !args->Contains("-nc")) {
      printf("Error: To specify a region of interest, all parameters -r1, -c1, -nr, and\n");
      printf("       -nc need to be specified.\n");
      PrintUsage();
      return EXIT_SUCCESS;
    }
  }

  // Call the pointx C routine
  pointx_c(args->String("-i"), args->String("-o"), args->Integer("-win", 7),
           args->Integer("-s", 10000), args->Double("-q", 0.7),
           args->Integer("-locmax", 7), args->Integer("-r1", 0),
           args->Integer("-c1", 0), args->Integer("-nr", 0),
           args->Integer("-nc", 0));
  
  return EXIT_SUCCESS;
}
