/*
                  Copyright 2013 University of Twente
 
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

Prepare 2d map and laser data for reconstruction
Both come from FME, expectation is that linenumber is assigned to laser points
In the preparation phase, ascii laser points are converted into laser format,
the label and polygonID are also transfered

------------------------------------------------------------*/
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: prepare3DBGT -il <laser points, txt or xyz format>\n");
  printf("             -ip <input map points (3D object points)>\n");
  printf("             -it <input map topology>\n");
  printf("             -ol <output tagged laser points.\n");
  printf("             -op <output points (3D object points)>\n");
  printf("             -ot <output topology>\n");
  printf("             [-sd <sample distance (double, def: 5.0)>]\n");
  printf("             [-buildings <select building polygons from input data (default:false)>]\n");
  printf("             [-plantcover <select plantcover polygons (forest) from input data (default:false)>]\n");
  printf("             [-debug <show more progress information>]\n");
  printf("             [-rf <reduction factor to decrease progress information, def: 10>]\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void prepare3DBGT(char *, char *, char *, char *,char *, char *, double, 
                         bool, bool, bool, int);

  // Check on required input files
  if (args->Contains("-usage") ||
      !args->Contains("-il") ||
      !args->Contains("-ip") ||
      !args->Contains("-it") ||
  //    !args->Contains("-map2points") ||
  //    !args->Contains("-dtm2map") ||
      !args->Contains("-ol")) {
    if (!args->Contains("-usage")) printf("Error: missing programme option.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Call the main function
 prepare3DBGT(args->String("-il"), 
            args->String("-ip"), args->String("-it"),
            args->String("-ol"), 
            args->String("-op"), args->String("-ot"),
            args->Double("-sd", 5.0),
            args->Contains("-buildings"), args->Contains("-plantcover"),
            args->Contains("-debug"),
            args->Integer("-rf", 10));
  return EXIT_SUCCESS;
}
