
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
|  Routine Name: main() - Reorganise laser data in tiles
|
|       Purpose: main program for createtiles
|
|         Input:
|		Required M.E. group:
|		    char *clui_info->f_string; {File filter of point files or meta data files}
|		    int   clui_info->f_flag; {TRUE if -f specified}
|
|		    char *clui_info->i_file; {File name of point file or meta data file}
|		    int   clui_info->i_flag; {TRUE if -i specified}
|
|		Required M.E. group:
|		    int clui_info->bi_flag; {TRUE if -bi specified}
|
|		    char *clui_info->bm_file; {Use bounds from this file}
|		    int   clui_info->bm_flag; {TRUE if -bm specified}
|
|		    int clui_info->bu_flag; {TRUE if -bu specified}
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
|		double clui_info->tw_double; {Tile width in meters}
|		int    clui_info->tw_flag; {TRUE if -tw specified}
|
|		double clui_info->th_double; {Tile height in meters}
|		int    clui_info->th_flag; {TRUE if -th specified}
|
|		double clui_info->tb_double; {Border around tile in meters}
|		int    clui_info->tb_flag; {TRUE if -tb specified}
|
|		int clui_info->st_flag; {TRUE if -st specified}
|
|		char *clui_info->root_string; {Root name for laser data files}
|		int   clui_info->root_flag; {TRUE if -root specified}
|
|		char *clui_info->odir_string; {Directory for data files}
|		int   clui_info->odir_flag; {TRUE if -odir specified}
|
|		int clui_info->meta_flag; {TRUE if -meta specified}
|
|		char *clui_info->oc_file; {2D object points of the tile corners}
|		int   clui_info->oc_flag; {TRUE if -oc specified}
|
|		char *clui_info->ot_file; {Topology of the lines of the tile boundaries}
|		int   clui_info->ot_flag; {TRUE if -ot specified}
|
|        Output:
|       Returns:
|
|    Written By: 
|          Date: Jan 10, 2005
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
  printf("Usage: createtiles -i <meta data or point file> OR\n");
  printf("                   -f <input file filter>\n");
  printf("                   -bi (use bounds of input file) OR\n");
  printf("                   -bm <meta data file> (use bounds from this file) OR\n");
  printf("                   -bu (user specified bounds)\n");
  printf("                   -xmin <value>, -xmax <value> (X range)\n");
  printf("                   -ymin <value>, -ymax <value> (Y range)\n");
  printf("                   [-tw <value, def: 1000.0>] (tile width)\n");
  printf("                   [-th <value, def: 1000.0>] (tile height)\n");
  printf("                   [-tb <value, def: 100.0>] (tile border)\n");
  printf("                   [-st] (derive tiles for each strip seperately\n");
  printf("                   -root <name> (root name for output files)\n");
  printf("                   [-huge] (use 6 digits for tile rows and columns (def: 3))\n");
  printf("                   [-odir <name>] (output directory name)\n");
  printf("                   [-meta] (output of meta data)\n");
  printf("                   [-oc <object points>] (of tile corners)\n");
  printf("                   [-ot <topology>] (of tiles)\n");
  printf("                   [-p] (preserve multiple multiple pulses in one tile)\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void createtiles_cpp(char *, char *, int, char *, int,
                       int, double, int, double, int, double, int, double,
                       double, double, double, int, char *, char *, int,
                       char *, char *, bool, bool);

// Usage

  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
// Check on required input files

  if (!args->Contains("-i") && !args->Contains("-f")) {
    printf("Error: no input data specified with -i <filename> or -f <filter>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (args->Contains("-i") && args->Contains("-f")) {
    printf("Error: -i and -f can not be used simultaneously!\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

// Check if tile corners are output if topology is requested.

  if (args->Contains("-ot") && !args->Contains("-oc"))
    printf("Warning: Tile topology is meaningless without tile corners.\n");

// Create the tiles

  createtiles_cpp(args->String("-f"), args->String("-i"),
                  args->Contains("-bi"), args->String("-bm"),
                  args->Contains("-bu"),
                  args->Contains("-xmin"), args->Double("-xmin", 0.0),
                  args->Contains("-xmax"), args->Double("-xmax", 0.0),
                  args->Contains("-ymin"), args->Double("-ymin", 0.0),
                  args->Contains("-ymax"), args->Double("-ymax", 0.0),
                  args->Double("-tw", 1000.0), args->Double("-th", 1000.0),
                  args->Double("-tb", 100.0), args->Contains("-st"),
                  args->String("-root"), args->String("-odir"),
                  args->Contains("-meta"),
                  args->String("-oc"), args->String("-ot"),
                  (bool) args->Contains("-p"),
                  (bool) args->Contains("-huge"));

  return EXIT_SUCCESS;
}
