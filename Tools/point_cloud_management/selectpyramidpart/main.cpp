
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
  printf("Usage: selectpyramidpart -i <pyramid meta data>\n");
  printf("                         -l <pyramid level>\n");
  printf("                         -rb <first tile row of selection>\n");
  printf("                         -re <last tile row of selection>\n");
  printf("                         -cb <first tile column of selection>\n");
  printf("                         -ce <last tile column of selection>\n");
  printf("                         -dir <directory for output files\n");
}
  
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void selectpyramidpart(char *, char *, int, int, int, int, int);

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
  if (!args->Contains("-l")) {
    printf("Error: pyramid level not specified with -l.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-rb") || !args->Contains("-re")) {
    printf("Error: tile row range not specified with -rb and -re.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-cb") || !args->Contains("-ce")) {
    printf("Error: tile column range not specified with -cb and -ce.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Select the pyramid part
  selectpyramidpart(args->String("-i"), args->String("-dir"),
                    args->Integer("-l", 0),
					args->Integer("-rb", 0), args->Integer("-re", 0),
					args->Integer("-cb", 0), args->Integer("-ce", 0));
					
  return EXIT_SUCCESS;
}
