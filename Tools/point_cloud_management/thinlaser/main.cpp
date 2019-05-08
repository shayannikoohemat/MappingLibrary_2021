
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
|  Routine Name: main() - (Random) thinning of a laser data set
|
|       Purpose: main program for thinlaser
|
|         Input:
|		char *clui_info->i_file; {File name of strip meta data or point data}
|		int   clui_info->i_flag; {TRUE if -i specified}
|
|		char *clui_info->o_file; {Thinned laser data}
|		int   clui_info->o_flag; {TRUE if -o specified}
|
|		Required M.E. group:
|		    int clui_info->np_int; {Reduction factor for selecting points within scan line}
|		    int clui_info->np_flag; {TRUE if -np specified}
|
|		    double clui_info->dmin_double; {Minimum distance between points}
|		    int    clui_info->dmin_flag; {TRUE if -dmin specified}
|
|		int clui_info->rp_flag; {TRUE if -rp specified}
|
|		int clui_info->ns_int; {Reduction factor for selecting scan lines}
|		int clui_info->ns_flag; {TRUE if -ns specified}
|
|		int clui_info->rs_flag; {TRUE if -rs specified}
|
|		char *clui_info->om_file; {Meta data file of thinned laser data}
|		int   clui_info->om_flag; {TRUE if -om specified}
|
|        Output:
|       Returns:
|
|    Written By: 
|          Date: May 08, 2003
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
  printf("Usage: thinlaser -i <input point file or laser block meta data file>\n");
  printf("                 -o <output point file> OR\n");
  printf("                 -app <appendix for generated output files, for blocks only\n");
  printf("                 -odir <output directory, for blocks only>\n");
  printf("                 [-overwrite] (force overwriting of earlier results, for blocks only)\n");
  printf("                 [-om_file <file name>] (output of meta data)\n");
  printf("                 [-om] (output of meta data, for blocks only)\n");
  printf("                 [-m <reduction method (int, default: 1)>]\n");
  printf("                 [-dmin <distance>] (minimum distance between points, methods 1, 2 and 9)\n");
  printf("                 [-np <number>] (select every np'th point, methods 3, 4, 6 and 7)\n");
  printf("                 [-sp <number, def: 0>] (start point offset, method 3 and 6)\n");
  printf("                 [-ns <number>] (select every ns' scan line, methods 6 and 7)\n");
  printf("                 [-rk <factor, def: 4.0>] (reduction factor using kd-tree, method 5)\n");
  printf("                 [-knn_start <number, def: 2>] (minimum knn in kd-tree, method 5)\n");
  printf("                 [-knn_max <number, def: 10>] (maximum knn in kd-tree, method 5)\n\n");
  printf("Reduction methods:\n");
  printf("1 - Minimum 2D distance between points, based on original point spacing\n");
  printf("2 - Minimum 3D distance between points, based on original point spacing\n");
  printf("3 - Every n'th point\n");
  printf("4 - Randomly select a point from every n points\n");
  printf("5 - knn based selection of every n'th point\n");
  printf("6 - Every n'th point from every n'th scan line\n");
  printf("7 - Randomly select every n'th point from every n'th scan line\n");
  printf("8 - Randomly select every n'th point from every randomly selected n'th scan line\n");
  printf("9 - Special purpose filtering for MLS data of Lynx scanner. Use -dmin to specify\n");
  printf("    minimum height difference between two points on same XY location.\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  int             method;

  void thinlaser_cpp(char *, char *, char *, int, double, int, int, int,
                     double, int, int, char *, char *, bool, bool);

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input files
  if (!args->Contains("-i")) {
    printf("Error: No input data specified with -i <filename>.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o") && !args->Contains("-app")) {
    printf("Error: No output file name specified with -o <filename> or -app <appendix>.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Check on parameters
  method = args->Integer("-m", 1);
  if (method == 1 || method == 2 || method == 9) {
    if (!args->Contains("-dmin")) {
      printf("Error: Parameter -dmin needs to be specified for methods 1, 2 and 9\n");
      PrintUsage();
      return EXIT_SUCCESS;
    }
  }
  if (method == 3 || method == 4 || method == 6 || method == 7 || method == 8) {
    if (!args->Contains("-np")) {
      printf("Error: Parameter -np needs to be specified for methods 3, 4, 6, 7 and 8\n");
      PrintUsage();
      return EXIT_SUCCESS;
    }
  }
  if (method == 6 || method == 7 || method == 8) {
    if (!args->Contains("-ns")) {
      printf("Error: Parameter -ns needs to be specified for methods 6, 7 and 8\n");
      PrintUsage();
      return EXIT_SUCCESS;
    }
  }

         
// Check default values -np, -ns, -dmin
  thinlaser_cpp(args->String("-i"), args->String("-o"),
                args->String("-om_file"), args->Integer("-m", 1),
                args->Double("-dmin", 5.0),
                args->Integer("-np", 2), args->Integer("-sp", 0),
                args->Integer("-ns", 2), args->Double("-rk", 4.0),
                args->Integer("-knn_start", 2), args->Integer("-knn_max", 10),
				args->String("-app"), args->String("-odir"), 
				args->Contains("-overwrite"), args->Contains("-om"));

  return EXIT_SUCCESS;
}
