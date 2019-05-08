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
using namespace std;


void PrintUsage()
{
  printf("Usage: createindoordatagraphs -il <laser points>\n");
  printf("                       -op <output points (3D object points)>\n");
  printf("                       -ot <output topology>\n");
  printf("                       [-mss <minimum segment size, # points (default: 50)>]\n");
  printf("                       [-mll <minimum line length, in m (default: 0.5 m)>]\n");
  }

int main(int argc, char *argv[])
{
    InlineArguments *args = new InlineArguments(argc, argv);

  void createindoordatagraphs(char *, char *, char *, int, double);

  // Check on required input files
  if (args->Contains("-usage") ||
      !args->Contains("-il") ||
      !args->Contains("-op") || 
      !args->Contains("-ot")) { 
    if (!args->Contains("-usage")) printf("Error: missing programme option.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Call the main function
  createindoordatagraphs(args->String("-il"), 
            args->String("-op"), args->String("-ot"), 
            args->Integer("-mss", 50),
            args->Double("-mll", 0.5));
  return EXIT_SUCCESS;
}
