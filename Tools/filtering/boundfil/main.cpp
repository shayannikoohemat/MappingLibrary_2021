
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

using namespace std;

/*-----------------------------------------------------------
|
|  Routine Name: main() - Filter points outside bounds
|
|       Purpose: main program for boundfil
|
|         Input:
|		char *clui_info->app_string; {Appendix of root name}
|		int   clui_info->app_flag; {TRUE if -app specified}
|
|		Required M.E. group:
|		    char *clui_info->f_string; {File filter of point files or meta data files}
|		    int   clui_info->f_flag; {TRUE if -f specified}
|
|		    char *clui_info->i_file; {File name of point file or meta data file}
|		    int   clui_info->i_flag; {TRUE if -i specified}
|
|		Required M.E. group:
|		    char *clui_info->im_file; {Meta data file with bounds}
|		    int   clui_info->im_flag; {TRUE if -im specified}
|
|		    int clui_info->iu_flag; {TRUE if -iu specified}
|
|		double clui_info->xmin_double; {Minimum X-coordinate}
|		int    clui_info->xmin_flag; {TRUE if -xmin specified}
|
|		double clui_info->xmax_double; {Maximum X-coordinate}
|		int    clui_info->xmax_flag; {TRUE if -xmax specified}
|
|		double clui_info->ymin_double; {Minimum Y-coordinate}
|		int    clui_info->ymin_flag; {TRUE if -ymin specified}
|
|		double clui_info->ymax_double; {Maximum Y-coordinate}
|		int    clui_info->ymax_flag; {TRUE if -ymax specified}
|
|		double clui_info->zmin_double; {Minimum Z-coordinate}
|		int    clui_info->zmin_flag; {TRUE if -zmin specified}
|
|		double clui_info->zmax_double; {Maximum Z-coordinate}
|		int    clui_info->zmax_flag; {TRUE if -zmax specified}
|
|		int clui_info->snt_flag; {TRUE if -snt specified}
|
|		char *clui_info->odir_string; {Directory for output files}
|		int   clui_info->odir_flag; {TRUE if -odir specified}
|
|		int clui_info->oa_flag; {TRUE if -oa specified}
|
|		int clui_info->meta_flag; {TRUE if -meta specified}
|
|        Output:
|       Returns:
|
|    Written By: 
|          Date: Apr 18, 2001
| Modifications:
|
------------------------------------------------------------*/

void PrintUsage()
{
  system("C:\\cygwin\\bin\\man boundfil");
  exit(0);
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void boundfil_cpp(char *, char *, char *, int, int, double, int, double,
			        int, double, int, double, int, double, int, double,
			        int, int, int, int, int, int, int, int,
			        int, int, int, int, int, int, int, int,
                    int, char *, char *, int, int);
                    
  if (args->Contains("-usage")) PrintUsage();
                 
  if (!args->Contains("-i") && !args->Contains("-f")) {
    printf("Error: -i or -f is a required argument.\n");
    PrintUsage();
  }

  if (args->Contains("-i") && args->Contains("-f")) {
    printf("Error: -i and -f are exclusive arguments.\n");
    PrintUsage();
  }

  if (!args->Contains("-im") && !args->Contains("-iu")) {
    printf("Error: -im or -iu is a required argument.\n");
    PrintUsage();
  }

  if (args->Contains("-im") && args->Contains("-iu")) {
    printf("Error: -im and -iu are exclusive arguments.\n");
    PrintUsage();
  }

  if (!args->Contains("-app")) {
    printf("Error: -app is a required argument.\n");
    PrintUsage();
  }

  boundfil_cpp(args->String("-f"), args->String("-i"),
               args->String("-im"), args->Contains("-iu"),
               args->Contains("-xmin"), args->Double("-xmin", 0.0),
               args->Contains("-xmax"), args->Double("-xmax", 0.0),
               args->Contains("-ymin"), args->Double("-ymin", 0.0),
               args->Contains("-ymax"), args->Double("-ymax", 0.0),
               args->Contains("-zmin"), args->Double("-zmin", 0.0),
               args->Contains("-zmax"), args->Double("-zmax", 0.0),
               args->Contains("-rmin"), args->Integer("-rmin", 0),
               args->Contains("-rmax"), args->Integer("-rmax", 0),
               args->Contains("-pcmin"), args->Integer("-pcmin", 0),
               args->Contains("-pcmax"), args->Integer("-pcmax", 0),
               args->Contains("-plmin"), args->Integer("-plmin", 0),
               args->Contains("-plmax"), args->Integer("-plmax", 0),
               args->Contains("-lmin"), args->Integer("-lmin", 0),
               args->Contains("-lmax"), args->Integer("-lmax", 0),
               args->Contains("-snt"), args->String("-app"),
               args->String("-odir"), args->Contains("-oa"),
               args->Contains("-meta"));

  return EXIT_SUCCESS;
}
