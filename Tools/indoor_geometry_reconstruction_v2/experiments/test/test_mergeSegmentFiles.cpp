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
#include <ctime>
#include "../include/visualization_tools.h"
#include "../include/utils.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: merge_segmentFiles  -master_lp <input *.laser>\n");
  printf("                              -dir <directory with all laser files>\n");
  printf("                              -out_file <output laserpoints>\n");
  }

int main(int argc, char *argv[])
{
    InlineArguments *args = new InlineArguments(argc, argv);

    std::clock_t start;
    double duration;
    start = std::clock();

  // Check on required input files
  if (args->Contains("-usage") ||
      !args->Contains("-master_lp") ||
      !args->Contains("-dir") ||
      !args->Contains("-out_file"))
  {
    if (!args->Contains("-usage")) printf("Error: missing programme option.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

    // read laserfile
    LaserPoints master_lp;
    master_lp = master_lp.Read(args->String("-master_lp"));

    //master_lp .Read("/../test_data/lp_master.laser");

  // processing directory
    char * root_dir;
  root_dir = args->String("-dir");
  char * out_file;
  out_file = args->String("-out_file");

  //root_dir = (char*) "/../test_data/dir/";

  //out_file = (char*) "/../test_data/merged.laser";

  // Call the main function
  merge_lp_segmented(master_lp, root_dir, out_file);

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Total processing time: "<< duration << "s" << '\n';

  return EXIT_SUCCESS;
}
