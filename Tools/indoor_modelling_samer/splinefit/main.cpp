
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
  printf("Usage: splinefit -i <laser point input file>\n");
  printf("                 [-o <laser point file of fitted splines>]\n");
  printf("                 [-order <spline order> (def: 4)\n");
  printf("                 [-dt <time interval between knots> (def: 1.0 s)\n");
  printf("                 [-dmin <minimum distance between knots> (def: 0 m)\n");
}
 
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void splinefit_cpp(char *, char *, int, double, double);
  
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }

  if (!args->Contains("-i")) {
    printf("Error: No input file specified with -i.\n");
    PrintUsage();
    exit(0);
  }

  splinefit_cpp(args->String("-i"), args->String("-o"),
                args->Integer("-order", 4), args->Double("-dt", 1.0),
				args->Double("-dmin", 0.0));

  return EXIT_SUCCESS;
}
