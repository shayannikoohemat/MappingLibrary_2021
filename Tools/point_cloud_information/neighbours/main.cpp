
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

/*-----------------------------------------------------------

Neighbours - determining the nearest neighbours of laser points

------------------------------------------------------------*/

void PrintUsage()
{
  printf("Usage: neighbours -i <input laser data> -o <neighbour text file>\n");
  printf("                  [-tin] OR [-knn <number of neighbours, default: 20]\n");
  printf("                  [-dmax <maximum distance to neighbour>]\n");
  printf("                  [-dim <dimension for distance computation (2 or 3), default: 3>]\n");
  printf("If -tin or -knn are not specified, the default is -knn 20\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  bool neighbours_cpp(char *, char *, int, int, int, int, bool, double);
    
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i")) {
    printf("Error: -i is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-o")) {
    printf("Error: -o is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (args->Contains("-tin") && args->Contains("-knn")) {
    printf("Error: -tin and -knn are exclusive arguments.\n");
    PrintUsage();
    exit(0);
  }

  neighbours_cpp(args->String("-i"), args->String("-o"),
                 args->Contains("-tin"), args->Contains("-knn"),
                 args->Integer("-knn", 20), 
                 args->Contains("-dmax"), 
                 args->Integer("-dim", 3) == 2,
                 args->Double("-dmax", 1.0));
                 
  return EXIT_SUCCESS;
}
