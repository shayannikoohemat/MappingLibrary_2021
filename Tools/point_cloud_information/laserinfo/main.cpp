
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
|  Routine Name: main() - Provide information on header and first point of laser file
|
|       Purpose: main program for laserinfo
|
|         Input:
|		char *clui_info->i_file; {Laser point file}
|		int   clui_info->i_flag; {TRUE if -i specified}
|
|        Output:
|       Returns:
|
|    Written By: 
|          Date: Mar 06, 2005
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
  printf("Usage: laserinfo -i <laser points file>\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void laserinfo_cpp(char *);

// Usage

  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
// Check on required input files

  if (!args->Contains("-i")) {
    printf("Error: no input data specified with -i <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

// Print the information to the screen

  laserinfo_cpp(args->String("-i"));

  return EXIT_SUCCESS;
}
