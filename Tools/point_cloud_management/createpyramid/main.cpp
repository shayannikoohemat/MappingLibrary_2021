
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


#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"
#include "LaserPyramid.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: createpyramid -i <tiled_block>\n");
  printf("                     [-r <linear reduction factor (int, default: 2)>]\n");
  printf("                     [-m <reduction method (int, default: 1)>]\n");
  printf("                     [-ps <original point spacing> (double, default: determine from first tile)]\n");
  printf("                     [-overwrite <overwrite already existing tiles>]\n\n");
  printf("Reduction methods:\n");
  printf("1 - Minimum 2D distance between points, based on original point spacing\n");
  printf("2 - Minimum 3D distance between points, based on original point spacing\n");
  printf("3 - Every n'th point\n");
  printf("4 - Randomly select a point from every n points\n");
  printf("5 - knn based selection of every n'th point\n");
}

int main(int argc, char *argv[])
{
  LaserPyramid    pyramid;
  LaserBlock      block;
  InlineArguments *args = new InlineArguments(argc, argv);

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input files
  if (!args->Contains("-i")) {
    printf("Error: no input data specified with -i <tiled_block>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Read the block meta data
  if (!block.ReadMetaData(args->String("-i"))) {
    printf("Error reading meta data from block file %s\n",
           args->String("-i"));
    return 0;
  }
  
  // Create the pyramid
  if (!pyramid.Create(block, args->Integer("-m", 1), args->Integer("-r", 2),
                      args->Double("-ps", 0.0),
                      args->Contains("-overwrite")))
    printf("Error in creating pyramid of block %s\n", block.Name());
  
  return EXIT_SUCCESS;
}
