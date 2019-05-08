
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
|  Routine Name: main() - Conversion of laser points and/or TINs to the VRML format
|
|       Purpose: main program for laser2vrml
|
|         Input:
|		char *clui_info->o_file; {VRML file}
|		int   clui_info->o_flag; {TRUE if -o specified}
|
|		Required M.E. group:
|		    char *clui_info->f_string; {File filter of point files or meta data files}
|		    int   clui_info->f_flag; {TRUE if -f specified}
|
|		    char *clui_info->i_file; {File name of point file or meta data file}
|		    int   clui_info->i_flag; {TRUE if -i specified}
|
|		Optional M.E. group:
|		    int clui_info->tm_flag; {TRUE if -tm specified}
|
|		    char *clui_info->tin_file; {TIN of the input laser data}
|		    int   clui_info->tin_flag; {TRUE if -tin specified}
|
|		int   clui_info->ot_cycle; {Output of data as points, crosses, spheres or TIN}
|		char *clui_info->ot_label;
|		int   clui_info->ot_flag; {TRUE if -ot specified}
|
|		double clui_info->p_double; {Point radius of spheres or crosses}
|		int    clui_info->p_flag; {TRUE if -p specified}
|
|        Output:
|       Returns:
|
|    Written By: 
|          Date: Nov 01, 2002
| Modifications:
|
------------------------------------------------------------*/

#include <stdio.h>

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void laser2vrml_cpp(char *, char *, int, char *, int, double, char *);
    
  if (args->Contains("-usage")) {
    system("C:\\cygwin\\bin\\man laser2vrml");
    exit(0);
  }
                       
  if (!args->Contains("-i") && !args->Contains("-f")) {
    printf("Error: -i or -f is a required argument.\n");
    system("C:\\cygwin\\bin\\man laser2vrml");
    exit(0);
  }
  if (args->Contains("-i") && args->Contains("-f")) {
    printf("Error: -i and -f are exclusive arguments.\n");
    system("C:\\cygwin\\bin\\man laser2vrml");
    exit(0);
  }
  if (!args->Contains("-o")) {
    printf("Error: -o is a required argument.\n");
    system("C:\\cygwin\\bin\\man laser2vrml");
    exit(0);
  }

  laser2vrml_cpp(args->String("-f"), args->String("-i"),
                 args->Contains("-tm"), args->String("-tin"),
                 args->Integer("-ot", 1), args->Double("-p", 0.2),
                 args->String("-o"));

  return EXIT_SUCCESS;
}
