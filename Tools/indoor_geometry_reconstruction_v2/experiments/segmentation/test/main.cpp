/*
                  Copyright 2010 University of Twente
 
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
/*-----------------------------------------------------------

Construction of data graphs using laser segments

------------------------------------------------------------*/

#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"
#include "dxf.h"
#include <stdio.h>
#include <ctime>
#include "../include/segmentation.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: surfacegrowing_test -i <input *.laser points>\n");
  printf("                       -o <output points *.laser>\n");
  printf("                       -d <optional:MaxDistToPlane [default 0.05m]>\n");
  printf("                       [-r <optional:growing radius [default 1.0m]>]\n");
  }

int main(int argc, char *argv[])
{
    InlineArguments *args = new InlineArguments(argc, argv);

    std::clock_t start;
    double duration;
    start = std::clock();

  //void surfacegrowing_test(char *, char*, double, double);

  // Check on required input files
  if (args->Contains("-usage") ||
      !args->Contains("-i") ||
      !args->Contains("-o"))
  {
    if (!args->Contains("-usage")) printf("Error: missing programme option.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Call the main function
    surfacegrowing_test(args->String("-i"),
            args->String("-o"),
            args->Double("-d", 0.05),
            args->Double ("-r", 1.0));

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Total processing time: "<< duration << "s" << '\n';

  return EXIT_SUCCESS;
}
