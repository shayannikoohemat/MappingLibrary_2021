
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
| Merge meta data and point files of laser pyramids
| (useful for quicker file I/O)
|
------------------------------------------------------------*/

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: mergepyramiddata -i <pyramid meta data>\n");
  printf("                        -app <appendix string for output files>\n");
  printf("                        -dir <directory for output files\n");
}
  
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void mergepyramiddata(char *, const char *, char *);

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input file
  if (!args->Contains("-i")) {
    printf("Error: no input data specified with -i <pyramid meta data>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Create the merged pyramid
  mergepyramiddata(args->String("-i"), args->String("-app", "_merged"), 
                   args->String("-dir"));

  return EXIT_SUCCESS;
}
