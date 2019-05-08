
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

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"

using namespace std;

/*-----------------------------------------------------------
|
|  Programme Name: pla - pulse length analysis
|
|		Required M.E. group:
|		    char *clui_info->f_string; {File filter of point files or meta data files}
|		    int   clui_info->f_flag; {TRUE if -f specified}
|
|		    char *clui_info->i_file; {File name of point file or meta data file}
|		    int   clui_info->i_flag; {TRUE if -i specified}
|
|		char *clui_info->o_file; {Analysis output file}
|		int   clui_info->o_flag; {TRUE if -om specified}
|
|        Output:
|       Returns:
|
|    Written By: 
|          Date: May 04, 1999
| Modifications:
|
------------------------------------------------------------*/

void PrintUsage()
{
  printf("Usage: pla [-i <file name>] || [-f <file filter>] -o <text file>\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void pla(char *, char *, char *);

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i") && !args->Contains("-f")) {
    printf("Error: -i or -f is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  if (args->Contains("-i") && args->Contains("-f")) {
    printf("Error: -i and -f are exclusive arguments.\n");
    PrintUsage();
    exit(0);
  }

  if (!args->Contains("-o")) {
    printf("Error: -o is a required argument.\n");
    PrintUsage();
  }

  pla(args->String("-f"), args->String("-i"), args->String("-o"));

  return EXIT_SUCCESS;
}
