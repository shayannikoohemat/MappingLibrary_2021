
/*
                    Copyright 2018 University of Twente 
 
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
#include <math.h>
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("indoorregistration2 accurately estimate the parameters of the relative orientation\n");
  printf("between the three scanners of the indoor mapping system.\n");
  printf("Approximate values need to be provided in a scanner system configuration file.\n");
  printf("The input data has to be preprocessed by scanlinesegmentation.\n");
  printf("\n");
  printf("Usage: indoorregistration2 -i <block file>\n");
  printf("                  -isys <input system configuration file with approximate values>]\n");
  printf("                  -osys <output system configuration file with improved values>]\n");
  printf("                  [-minl <minimum segment length> (default: 1.0 m)]\n");
  printf("                  [-minn <minimum number of segment points> (default: 20)]\n");
  printf("                  [-maxdp <maximum distance of coplanar lines to plane> (default: 0.1 m)]\n");
  printf("                  [-maxdl <maximum distance between coplanar line segments> (default: 1.0 m)]\n");
  printf("                  [-mina <mininum angle between coplanar line segments> (default: 10 degrees)]\n");
  printf("                  [-maxa <maximum angle deviation from perpendicularity>\n");
  printf("                        (default: 0 degrees = use no perpendicularity constraint)\n");
  printf("                  [-ti <time interval in seconds> (default: 0.025)]\n");
  printf("                  [-v (verbose, report more output)]\n");
  printf("                  [-iter <maximum number of iterations> (default: 10)]\n");
}
 
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  double pi = 4.0 * atan(1.0);

  void indoorregistration2_cpp(char *, char *, char *, double, double, int,
                               double, double, double, double, bool, int);

  if (args->Contains("-usage")) {
  	PrintUsage();
  	exit(0);
  }  
  if (!args->Contains("-i")) {
    printf("Error: -i is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-isys")) {
    printf("Error: -isys is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-osys")) {
    printf("Error: -osys is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  indoorregistration2_cpp(args->String("-i"), args->String("-isys"),
                          args->String("-osys"), args->Double("-ti", 0.025),
					 	  args->Double("-minl", 1.0), args->Integer("-minn", 20), 
					 	  args->Double("-maxdp", 0.1),
						  args->Double("-maxdl", 1.0), 
						  args->Double("-mina", 10.0) * pi / 180.0,
						  args->Double("-maxa", 0.0) * pi / 180.0, 
						  args->Contains("-v"), args->Integer("iter", 10));

  return EXIT_SUCCESS;
}
