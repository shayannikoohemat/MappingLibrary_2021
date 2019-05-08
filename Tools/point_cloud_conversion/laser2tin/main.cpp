
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

using namespace std;

/*-----------------------------------------------------------
|
|  Routine Name: main() - Conversion of laser points to object points
|
|       Purpose: main program for laser2obj
|
|         Input:
|		char *clui_info->i_file; {Laser points}
|		int   clui_info->i_flag; {TRUE if -i specified}
|
|		char *clui_info->o_file; {Object points}
|		int   clui_info->o_flag; {TRUE if -o specified}
|
|        Output:
|       Returns:
|
|    Written By: 
|          Date: Jul 08, 1999
| Modifications:
|
------------------------------------------------------------*/

#include <stdio.h>

void PrintUsage()
{
  printf("Usage: laser2tin -i <laser points> -o <TIN>\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void laser2tin_cpp(char *, char *, char *);
  
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i")) {
    printf("Error: -i is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-o")) {
    printf("Error: -o is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  laser2tin_cpp(NULL, args->String("-i"), args->String("-o"));

  return EXIT_SUCCESS;
}
