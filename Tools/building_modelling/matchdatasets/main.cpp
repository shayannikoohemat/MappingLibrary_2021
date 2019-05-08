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

Match polygon numbers and labels to laser scanner data

------------------------------------------------------------*/
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: matchdatasets -il <laser points>\n");
  printf("             -ip <input map points (3D object points)>\n");
  printf("             -it <input map topology>\n");
  printf("             -ol <output tagged laser points.\n");
  printf("             -op <output points (3D object points)>\n");
  printf("             -ot <output topology>\n");
  printf("             -oh <include overhanging points (default:false)>\n");
  printf("             -reduce <reduce laser points to 1 p / [x] m (default:0)>\n");
  printf("             -map2points <assign map polygon number to laserpoints (default:false)>\n");
  printf("             -dtm2map <assign dtm value to map height (default:false)>\n");
  printf("             -buildings <select only building polygons from input data (default:false)>\n");
  printf("             -rm_label <remove polygons with a certain label value (default:0)>\n");
printf("               -label_by_segments <label by checking whether segments' centre of gravity is inside map (default:false)>\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void matchmapandlaser(char *, char *, char *, char *,char *, char *, bool, double, bool, bool, bool, int, bool);

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
  matchmapandlaser(args->String("-il"), 
            args->String("-ip"), args->String("-it"),
            args->String("-ol"), 
            args->String("-op"), args->String("-ot"),
            args->Contains("-oh"), args->Double("-reduce",0),
            args->Contains("-map2points"), args->Contains("-dtm2map"), args->Contains("-buildings"),
            args->Integer("-rm_label",0), args->Contains("-label_by_segments"));
  return EXIT_SUCCESS;
}
