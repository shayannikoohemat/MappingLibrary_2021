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
#include <stdio.h>
#include "InlineArguments.h"
#include "ObjectPoints.h"
#include "LineTopologies.h"

using namespace std;

void PrintUsage()
{
  printf("Conversion of a dxf file to object points and topology.\n\n");
  printf("Usage: dxf2obj -i <dxf file>\n");
  printf("               -op <object points>\n");
  printf("               -ot <topology>\n");
  printf("               [-label <number>] (1: store no label (default)\n");
  printf("                                  2: store colour as label\n");
  printf("                                  3: store layer as label)\n");
  printf("               [-colour <number> (select on specified colour number)\n");
  printf("               [-layer <number> (select on specified layer number)\n");
  printf("               [-loose_points] (store loose points as well)\n");
  printf("               [-ignore_zeros] (don't use point (0, 0, 0)\n");
  printf("               [-force_closure] (close all polylines)\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  ObjectPoints    points;
  LineTopologies  polygons;

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input and output files
  if (!args->Contains("-i")) {
    printf("Error: no input data specified with -i <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-op")) {
    printf("Error: no output points specified with -op <point file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-ot")) {
    printf("Error: no output topology specified with -ot <topology file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Read the files
  if (!points.ReadDXF(args->String("-i"), polygons, args->Integer("-label", 1),
                 args->Contains("-colour"), args->Integer("-colour", 0),
                 args->Contains("-layer"), args->Integer("-layer", 0) ,
                 (bool) args->Contains("-loose_points"),
				 (bool) args->Contains("-ignore_zeros"),
				 (bool) args->Contains("-force_closure"))) {
    printf("Error reading DXF file %s\n", args->String("-i"));
    exit(0);
  }
  
  // Write the point and topology files
  points.Write(args->String("-op"));
  polygons.Write(args->String("-ot"), false);

  return EXIT_SUCCESS;
}
