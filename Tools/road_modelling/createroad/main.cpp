
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
|
| Generation of road part outlines based on a GPS track and
| user defined parameters of the road part
|
|    Written By: George Vosselman
|          Date: December 28, 2009
| Modifications:
|
------------------------------------------------------------*/

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: createroad -gps <GPS track positions in Positions3D format> OR\n");
  printf("                  -gpsl <GPS track positions in Laser format>\n");
  printf("                  -name <road name>\n");
  printf("                  [-l <length of road part (def: 20 m)>]\n");
  printf("                  [-w <width of road part (def: 10 m)>]\n");
  printf("                  [-o <overlap between road parts (def: 0 m)>]\n");
  printf("                  [-d <discretisation step size of road part outlines (def: 5 m)>]\n");
  printf("                  [-b <tiled laser data block>\n");
  printf("                  [-odir <output directory (def: ./)]>\n");
  printf("                  [-olp <output laser points]\n");
  printf("                  [-clip <clip points at outline bounds]\n");
  printf("                  [-oop <road part outline points>]\n");
  printf("                  [-oot <road part outline topology>]\n");
}
  
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void createroad_cpp(char *, char *, char *, double, double, double, double, 
                      char *, char *, bool, bool, char *, char *);

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input and output files
  if (!args->Contains("-gps") && !args->Contains("gpsl")) {
    printf("Error: no input data specified with -gps or -gpsl\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Create the road parts
  createroad_cpp(args->String("-gps"), args->String("-gpsl"),
                 args->String("-name"),
                 args->Double("-l", 20.0),
                 args->Double("-w", 10.0), args->Double("-o", 0.0),
                 args->Double("-d", 5.0), args->String("-b"),
                 args->String("-odir"),
                 args->Contains("-olp"), args->Contains("-clip"),
                 args->String("-oop"), args->String("-oot"));

  return EXIT_SUCCESS;
}
