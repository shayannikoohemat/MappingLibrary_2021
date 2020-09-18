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
  printf("Usage: LineTopologies_to_OFF  -vertices <input *.objpts>\n");
  printf("                              -faces <input *.top>\n");
  printf("                              -out_dir <output directory>\n");
  }

int main(int argc, char *argv[])
{
    InlineArguments *args = new InlineArguments(argc, argv);

    std::clock_t start;
    double duration;
    start = std::clock();

  // Check on required input files
  if (args->Contains("-usage") ||
      !args->Contains("-vertices") ||
      !args->Contains("-faces") ||
      !args->Contains("-out_dir"))
  {
    if (!args->Contains("-usage")) printf("Error: missing programme option.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

 // define input variables
  ObjectPoints vertices;
  LineTopologies faces;
  char * root_dir;

  // read vertices and faces from the file
  vertices.Read(args->String("-vertices")); // /test_data/vertices.objpts
  faces.Read(args->String("-faces"));       // /test_data/faces.top

  // set output directory where the OFF file is stored
  root_dir = args->String("-out_dir");      // /test_data/

  // Call the main function
  LineTopologies_to_OFF(vertices, faces, root_dir);

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Total processing time: "<< duration << "s" << '\n';

  return EXIT_SUCCESS;
}
