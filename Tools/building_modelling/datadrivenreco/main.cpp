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

Create building hypotheses, after graph matching

------------------------------------------------------------*/

#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"
#include <stdio.h>

using namespace std;

void PrintUsage()
{
  printf("Usage: datadrivenreco  -ip <input map points (3D object points)>\n");
  printf("                    -it <input  map topologies>\n");
  printf("                    -ipm <input match results points (3D object points)>\n");
  printf("                    -itm <input  match results topologies>\n");
  printf("                    -it2 <input match results topologies>\n");
  printf("                    -il <input laser points>\n");
  printf("                    -op  <output object points>\n");
  printf("                    -ot <output topologies>\n");
  printf("                    -info <information file>\n");
//  printf("             [-sd <sample distance (double, def: 1.0)>]\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void datadrivenreco(char *, char *, char *, char *, char *, char *, char *, char *, char *);

  // Check on required input files
  if (args->Contains("-usage") ||
      !args->Contains("-ip") ||
      !args->Contains("-it") ||
      !args->Contains("-ipm") ||
      !args->Contains("-itm") ||
      !args->Contains("-it2") ||
      !args->Contains("-il") ||
      !args->Contains("-op") ||
      !args->Contains("-ot") ||
      !args->Contains("-info")) {
    if (!args->Contains("-usage")) printf("Error: missing programme option.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Call the main function
  datadrivenreco(args->String("-ip"), args->String("-it"),
                            args->String("-ipm"), args->String("-itm"),
                            args->String("-it2"), args->String("-il"), 
                            args->String("-op"),
            args->String("-ot"), args->String("-info"));
  return EXIT_SUCCESS;
}
