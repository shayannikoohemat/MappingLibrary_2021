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
  printf("Conversion of object points and topology to dxf.\n\n");
  printf("Usage: obj2dxf -ip <object points>\n");
  printf("               -it <topology>\n");
  printf("               -o <dxf file>\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  ObjectPoints    points;
  LineTopologies  polygons;
  FILE            *dxffile;

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input and output files
  if (!args->Contains("-ip")) {
    printf("Error: no input points specified with -ip <point file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-it")) {
    printf("Error: no input points specified with -it <topology file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: no output data specified with -o <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Read the files
  if (!points.Read(args->String("-ip"))) {
    printf("Error reading points from file %s\n", args->String("-ip"));
    return EXIT_SUCCESS;
  }
  if (!polygons.Read(args->String("-it"))) {
    printf("Error reading polygons from file %s\n", args->String("-it"));
    return EXIT_SUCCESS;
  }
  
  // Write the DXF file
  dxffile = fopen(args->String("-o"), "w");
  if (dxffile == NULL) {
    printf("Error opening output DXF file %s\n", args->String("-o"));
    return EXIT_SUCCESS;
  }
  points.WriteDXFMesh(dxffile, polygons, 7, true, true, true, true);
  fclose(dxffile);

  return EXIT_SUCCESS;
}
