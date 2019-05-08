
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


#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"
#include "LaserPyramid.h"
#include "BNF_io.h"

using namespace std;

void PrintUsage()
{
  printf("setname sets the name of a laser pyramid, block, strip, tile, or part\n\n");

  printf("Usage: setname -i <meta data file>\n");
  printf("               -name <new name>\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  LaserPyramid    pyramid;
  LaserBlock      block;
  LaserUnit       strip;
  LaserSubUnit    subunit;

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i")) {
    printf("Error: -i is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-name")) {
    printf("Error: -name is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  switch (BNF_LaserFileType(args->String("-i"))) {
    case LASER_PYRAMID:
      printf("setname cannot yet handle pyramid meta data files\n");
      break;
      
    case LASER_BLOCK:
      if (!block.ReadMetaData(args->String("-i"), false, false)) {
        printf("Error reading block meta data from %s\n", args->String("-i"));
        exit(0);
      }
      block.SetName(args->String("-name"));
      block.WriteMetaData(args->String("-i"), false, false);
      break;
      
    case LASER_STRIP:
      if (!strip.ReadMetaData(args->String("-i"), false)) {
        printf("Error reading strip meta data from %s\n", args->String("-i"));
        exit(0);
      }
      strip.SetName(args->String("-name"));
      strip.WriteMetaData(args->String("-i"), false);
      break;
         
    case LASER_TILE:
    case LASER_STRIP_PART:
    case LASER_POINT_SET:
      if (!subunit.ReadMetaData(args->String("-i"))) {
        printf("Error reading part or tile meta data from %s\n", args->String("-i"));
        exit(0);
      }
      subunit.SetName(args->String("-name"));
      subunit.WriteMetaData(args->String("-i"));
      break;
         
    case LASER_RAW_DATA:
      printf("No meta data type recognised in file name %s\n", args->String("-i"));
      break;
  }

  return EXIT_SUCCESS;
}
