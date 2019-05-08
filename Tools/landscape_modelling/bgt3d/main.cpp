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

Reconstruction of 3D BGT from 2D BGT and laser
scanner data for Geonovum

------------------------------------------------------------*/
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: bgt3d -il <laser points>\n");
  printf("             -ip <input map points (3D object points)>\n");
  printf("             -it <input map topology>\n");
  printf("             -op <output points (3D object points)>\n");
  printf("             -ot <output topology>\n");
  printf("             -opw <output wall points (3D object points)>\n");
  printf("             -otw <output wall topology>\n");
  printf("             [-hd_gen <minimal height distance to seperate 2 heights (double, def: 1.5)>]\n");
  printf("             [-hd_road <minimal height distance to seperate 2 road heights (double, def: 1.5)>]\n");
  printf("             [-rf <reduction factor to decrease progress information, def: 10>]\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void bgt3DMap(char *, char *, char *, char *, char *,char *, char *, double, double, bool, int);

  // Check on required input files
  if (args->Contains("-usage") ||
      !args->Contains("-il") ||
      !args->Contains("-ip") ||
      !args->Contains("-it") ||
      !args->Contains("-op") ||
      !args->Contains("-ot") ||
       !args->Contains("-opw") ||
      !args->Contains("-otw")) {
    if (!args->Contains("-usage")) printf("Error: missing programme option.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Call the main function
  bgt3DMap(args->String("-il"), 
            args->String("-ip"), args->String("-it"),
            args->String("-op"), args->String("-ot"),
            args->String("-opw"), args->String("-otw"),
            args->Double("-hd_gen", 1.5), args->Double("-hd_road", 1.5),
            args->Contains("-debug"), args->Integer("-rf", 10));
  return EXIT_SUCCESS;
}
