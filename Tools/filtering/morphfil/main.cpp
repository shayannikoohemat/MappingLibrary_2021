
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
|  Routine Name: main() - Filter laser data by mathematical morphology
|
|       Purpose: main program for morphfil
|
|         Input:
|		char *clui_info->k_file; {Filter kernel}
|		int   clui_info->k_flag; {TRUE if -k specified}
|
|		char *clui_info->app_string; {Appendix of root name for filter results}
|		int   clui_info->app_flag; {TRUE if -app specified}
|
|		Required M.E. group:
|		    char *clui_info->f_string; {File filter of point files or meta data files}
|		    int   clui_info->f_flag; {TRUE if -f specified}
|
|		    char *clui_info->i_file; {File name of point file or meta data file}
|		    int   clui_info->i_flag; {TRUE if -i specified}
|
|		double clui_info->kr_double; {Range of kernel to be used}
|		int    clui_info->kr_flag; {TRUE if -kr specified}
|
|		double clui_info->stdev_double; {Standard deviation of height}
|		int    clui_info->stdev_flag; {TRUE if -stdev specified}
|
|		double clui_info->tol_double; {Margin for height confidence on top of kernel values}
|		int    clui_info->tol_flag; {TRUE if -tol specified}
|
|		int clui_info->dilate_flag; {TRUE if -dilate specified}
|
|		int clui_info->ow_logic; {Overwrite existing output files}
|		int clui_info->ow_flag; {TRUE if -ow specified}
|
|		char *clui_info->odir_string; {Output directory}
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
|          Date: May 08, 2003
| Modifications:
|
------------------------------------------------------------*/

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  system("C:\\cygwin\\bin\\man morphfil");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void morphfil_cpp(char *, char *, char *, int, double, double, double, int,
                    char *, char *, int, int, int);

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
    printf("Error: -i and -f can not be used simultaneously!\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-k")) {
    printf("Error: no filter kernel specified with -k <filename>.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-app")) {
    printf("Error: no appendix for output files specified with -app <appendix>.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  morphfil_cpp(args->String("-f"), args->String("-i"), args->String("-k"),
               args->Contains("-kr"), args->Double("-kr", 10.0),
               args->Double("-stdev", 0.0), args->Double("-tol", 0.0),
               args->Contains("-dilate"), args->String("-app"),
               args->String("-odir"), args->Contains("-oa"),
               args->Contains("-meta"), args->Integer("-ow", 1));
  

  return EXIT_SUCCESS;
}
