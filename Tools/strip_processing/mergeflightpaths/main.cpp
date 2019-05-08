
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
|
| Collect flights paths of all strips in a block and merge
| them to one file
|
------------------------------------------------------------*/

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: mergeflightpaths -i <block or strip meta data> OR\n");
  printf("                        -f <file filter of meta data>\n");
  printf("                        -op <points of all flight paths>\n");
  printf("                        -ot <topology of all flight paths\n");
  printf("                        [-d] devide strip number by 10 to remove scan direction\n");
}
  
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void mergeflightpaths(char *, char *, char *, char *, bool);

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input files
  if (!args->Contains("-f") && !args->Contains("-i")) {
    printf("Error: no input data specified with -f <filter> or -i <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (args->Contains("-f") && args->Contains("-i")) {
    printf("Error: -f and -i are mutual exclusive arguments\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-op") || !args->Contains("-ot")) {
    printf("Error: -op and -ot are required arguments of output file names\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Merge flight paths
  mergeflightpaths(args->String("-i"), args->String("-f"), 
                   args->String("-op"), args->String("-ot"),
                   args->Contains("-d"));

  return EXIT_SUCCESS;
}
