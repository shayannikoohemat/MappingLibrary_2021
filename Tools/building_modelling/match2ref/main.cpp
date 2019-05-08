
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

Reconstruction of 3D Topography from 2D Topography and laser
scanner data

Adjust height of hidden objects
------------------------------------------------------------*/

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: match2ref    -ip <input map points (3D object points)>\n");
  printf("                    -it <input map topology>\n");
  printf("                    -ipr <input map points reference data (3D object points)>\n");
  printf("                    -itr <input map topology reference data>\n");
  printf("                    -il  <input laser points>\n");
  printf("                    -lri <laser points inside reference data>\n");
  printf("                    -lro <laser points outside reference data>\n");
//  printf("             [-sd <sample distance (double, def: 1.0)>]\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void matchreference(char *, char *, char *, char *, char *, char *, char *);

  // Check on required input files
  if (args->Contains("-usage") ||
      !args->Contains("-ip") ||
      !args->Contains("-it") ||
      !args->Contains("-ipr") ||
      !args->Contains("-itr") ||
      !args->Contains("-il") ||
      !args->Contains("-lri") ||
      !args->Contains("-lro")) {
    if (!args->Contains("-usage")) printf("Error: missing programme option.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Call the main function
  matchreference(args->String("-ip"), args->String("-it"),
                            args->String("-ipr"), args->String("-itr"),
                            args->String("-il"),
            args->String("-lri"), args->String("-lro"));
  return EXIT_SUCCESS;
}
