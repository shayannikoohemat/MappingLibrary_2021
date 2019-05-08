
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
|  Routine Name: main() - Select a subset of laser points
|
|       Purpose: main program for createpointset
|
|         Input:
|		char *clui_info->o_file; {Selected laser data points}
|		int   clui_info->o_flag; {TRUE if -o specified}
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
|		    char *clui_info->ic_file; {Image points of corners of rectangular image part}
|		    int   clui_info->ic_flag; {TRUE if -ic specified}
|
|		    char *clui_info->ip_file; {Image points of closed polygons}
|		    int   clui_info->ip_flag; {TRUE if -ip specified}
|
|		    char *clui_info->il_file; {Labeled image}
|		    int   clui_info->il_flag; {TRUE if -il specified}
|
|		    int clui_info->iu_flag; {TRUE if -iu specified}
|
|		char *clui_info->it_file; {Topology of closed image polygons}
|		int   clui_info->it_flag; {TRUE if -it specified}
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
|		int clui_info->rmin_int; {Minimum reflectance value}
|		int clui_info->rmin_flag; {TRUE if -rmin specified}
|
|		int clui_info->rmax_int; {Maximum reflectance value}
|		int clui_info->rmax_flag; {TRUE if -rmax specified}
|
|		int clui_info->pcmin_int; {Minimum pulse count value}
|		int clui_info->pcmin_flag; {TRUE if -pcmin specified}
|
|		int clui_info->pcmax_int; {Maximum pulse count value}
|		int clui_info->pcmax_flag; {TRUE if -pcmax specified}
|
|		char *clui_info->g_file; {Image grid specification}
|		int   clui_info->g_flag; {TRUE if -g specified}
|
|		char *clui_info->om_file; {Meta data output file}
|		int   clui_info->om_flag; {TRUE if -om specified}
|
|        Output:
|       Returns:
|
|    Written By: 
|          Date: Oct 03, 2000
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
  printf("Select points within some bounds and write them to a single file.\n");
  printf("For output of selected points in tiles, use the programme boundfil.\n");
  printf("Input file(s) can be specified by a (metadata) file name or filter.\n");
  printf("Bounds should be specified by one of the argument-im, -ic, -ip, -il, and -iu\n\n");
  printf("Usage: createpointset [-i <meta data or point file>]\n");
  printf("                      [-f <meta data or point file filter>]\n");
  printf("                      [-im <meta data file>] (Use bounds from meta data file)\n");
  printf("                      [-ic <image file>] (Use image corners, also needs -g <grid file>)\n");
  printf("                      [-ip <image points>] (Use bounds from image point polygon, also needs -t and -g)\n");
  printf("                      [-it <topology file>] (Polygon of image points)\n");
  printf("                      [-il <labeled image] (Select points in non-zero pixels, also needs -g)\n");
  printf("                      [-iu] (Use explicit bounds, like -xmin, -xmax, etc.)\n");
  printf("                      [-g <grid file> (Image grid definition, needed for -ic, -ip and -il\n");
  printf("                      [-xmin <value>] [-xmax <value>] (Minimum and maximum X-coordinate\n");
  printf("                      [-ymin <value>] [-ymax <value>] (Minimum and maximum Y-coordinate\n");
  printf("                      [-zmin <value>] [-zmax <value>] (Minimum and maximum Z-coordinate\n");
  printf("                      [-rmin <value>] [-rmax <value>] (Minimum and maximum reflectance value\n");
  printf("                      [-pcmin <value>] [-pcmax <value>] (Minimum and maximum pulse count value\n");
  printf("                      [-plmin <value>] [-plmax <value>] (Minimum and maximum pulse length value\n");
  printf("                      [-lmin <value>] [-lmax <value>] (Minimum and maximum label value\n");
  printf("                      -o <laser point file> (Selected laser points)\n");
  printf("                      [-om <metadata file> (Meta data of selected points)\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  int             num_methods = 0;

  void createpointset_cpp(char *, char *, char *, char *, char *, char *,
                          char *, int, char *, int, double, int, double,
                          int, double, int, double, int, double, int, double,
                          int, int, int, int, int, int, int, int,
                          int, int, int, int, int, int, int, int,
                          char *, char *);

// Usage

  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
// Check on required input files

  if (!args->Contains("-i") && !args->Contains("-f")) {
    printf("Error: no input data specified with -i <file> or -f <filter>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (args->Contains("-i") && args->Contains("-f")) {
    printf("Error: -i and -f are mutually exclusive.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

// Check on the selection strategy

  if (args->Contains("-im")) num_methods++;
  if (args->Contains("-ic")) num_methods++;
  if (args->Contains("-ip")) num_methods++;
  if (args->Contains("-il")) num_methods++;
  if (args->Contains("-iu")) num_methods++;
  if (num_methods == 0) {
    printf("Error: no selection method specified. Use\n");
    printf("       -im for bounds from a meta data file\n");
    printf("       -ic for image corners\n");
    printf("       -ip for image polygons\n");
    printf("       -il for a labeled image\n");
    printf("       -iu for user specified bounds\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (num_methods > 1) {
    printf("Error: -im, -ic, -ip, -il, and -iu are mutually exclusive.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

// Check on the grid file

  if (args->Contains("-ic") || args->Contains("-ip") || args->Contains("-il")) {
    if (!args->Contains("-g")) {
      printf("Error: a grid specification (-g) is needed for selection\n");
      printf("       methods -ic, -ip, and -il.\n");
      PrintUsage();
    }
  }

// Create the point set

  createpointset_cpp(args->String("-f"), args->String("-i"),
		     args->String("-im"), args->String("-ic"),
		     args->String("-ip"), args->String("-it"),
		     args->String("-il"), args->Contains("-iu"),
		     args->String("-g"),
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
		     args->String("-o"), args->String("-om"));

  return EXIT_SUCCESS;
}
