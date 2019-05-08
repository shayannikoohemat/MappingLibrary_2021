
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
|  Routine Name: main() - Split filter results
|
|       Purpose: main program for filsplit
|
|         Input:
|		Required M.E. group:
|		    char *clui_info->f_string; {File filter of point files or meta data files}
|		    int   clui_info->f_flag; {TRUE if -f specified}
|
|		    char *clui_info->i_file; {File name of point file or meta data file}
|		    int   clui_info->i_flag; {TRUE if -i specified}
|
|		char *clui_info->gp_string; {Appendix for ground points}
|		int   clui_info->gp_flag; {TRUE if -gp specified}
|
|		char *clui_info->fp_string; {Appendix for filtered points}
|		int   clui_info->fp_flag; {TRUE if -fp specified}
|
|		char *clui_info->odir_string; {Output directory}
|		int   clui_info->odir_flag; {TRUE if -odir specified}
|
|		int clui_info->meta_flag; {TRUE if -meta specified}
|
|        Output:
|       Returns:
|
|    Written By: 
|          Date: Mar 08, 2001
| Modifications:
|
------------------------------------------------------------*/

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include "InlineArguments.h"
#include "LaserPoint.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: filsplit -i <meta of point file> OR -f <file filter>\n");
  printf("                [-gp <extension of ground points>]\n");
  printf("                [-fp <extension of filtered points>]\n");
  printf("                [-odir <output directory>]\n");
  printf("                [-meta (output of meta data)]\n");
  printf("                [-tag <tag for selectiong> (default: IsFilteredTag)\n");
  printf("                [-v <attribute value of ground points> (default: 0)\n");
  printf("\nUse tag numbers as defined in LaserPoints.h\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  void filsplit_cpp(char *, char *, char *, const char *, const char *, int,
                    int, int);

// Usage

  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
// Check on required input files

  if (!args->Contains("-i") && !args->Contains("-f")) {
    printf("Error: no input data specified with -i <filename> or -f <filter>.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (args->Contains("-i") && args->Contains("-f")) {
    printf("Error: parameters -i and -f are mutually exclusive.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Split the filtered data
  filsplit_cpp(args->String("-f"), args->String("-i"),
               args->String("-odir"), args->String("-gp"),
               args->String("-fp"), args->Contains("-meta"),
			   args->Integer("-tag", (int) IsFilteredTag),
			   args->Integer("-v", 0));

  return EXIT_SUCCESS;
}
