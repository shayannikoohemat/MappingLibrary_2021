
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



/*-----------------------------------------------------------

Construction of surfaces in 3D Topography from 2D Topography and laser
scanner data

------------------------------------------------------------*/

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"
#include "dxf.h"


using namespace std;

void PrintUsage()
{
  printf("Usage: 3dtop2tin -il <laser points>\n");
  printf("                 -ip <input map points (3D object points)>\n");
  printf("                 -it <input map topology>\n");
  printf("                 -ik <input kernel image\n");
  printf("                 -ik2 <input kernel image\n");
  printf("                 -op <output points (3D object points)>\n");
  printf("                 -ot <output topology>\n");
  printf("                 -ov <output VRML file>\n");
  printf("                 -ol <output laser file>\n");
//  printf("                 [-sd <sample distance (double, def: 1.0)>]\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void Make3DSurface(char *, char *, char *, char *, char *, char *,char *, char *, char *);

  // Check on required input files
  if (args->Contains("-usage") ||
      !args->Contains("-il") ||
      !args->Contains("-ip") ||
      !args->Contains("-it") ||
      !args->Contains("-ik") ||
      !args->Contains("-ik2") ||
      !args->Contains("-op") || 
      !args->Contains("-ot") ||
      !args->Contains("-ov") ||
      !args->Contains("-ol")) {
    if (!args->Contains("-usage")) printf("Error: missing programme option.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Call the main function
  Make3DSurface(args->String("-il"), 
            args->String("-ip"), args->String("-it"),
            args->String("-ik"), args->String("-ik2"),
            args->String("-op"), args->String("-ot"), 
            args->String("-ov"), args->String("-ol"));
  return EXIT_SUCCESS;
}
