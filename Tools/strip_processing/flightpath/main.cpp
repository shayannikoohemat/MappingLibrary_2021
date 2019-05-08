
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
|  Routine Name: main() - Flight path reconstruction
|
|       Purpose: main program for fp
|
|         Input:
|		int   clui_info->m_cycle; {Method for flight path reconstruction}
|		char *clui_info->m_label;
|		int   clui_info->m_flag; {TRUE if -m specified}
|
|		Required M.E. group:
|		    char *clui_info->f_string; {File filter of meta data filesof block or strip}
|		    int   clui_info->f_flag; {TRUE if -f specified}
|
|		    char *clui_info->i_file; {File name of meta data file of block or strip}
|		    int   clui_info->i_flag; {TRUE if -i specified}
|
|		double clui_info->fh_double; {Assumed flight height}
|		int    clui_info->fh_flag; {TRUE if -fh specified}
|
|		double clui_info->ai_double; {Scan angle increment}
|		int    clui_info->ai_flag; {TRUE if -ai specified}
|
|        Output:
|       Returns:
|
|    Written By: 
|          Date: Jul 10, 2003
| Modifications:
|
------------------------------------------------------------*/

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: flightpath [-i <meta data input file>] OR\n");
  printf("                  [-f <meta data input filter>]\n");
  printf("                  [-m <method number (0-2)> (see flightpath -methods)\n");
  printf("                  [-fh <flight height> (required for method 0)\n");
  printf("                  [-ai <angle increment in degrees> (required for method 2)\n");
}

void PrintMethods()
{
  printf("Methods used in flightpath:\n\n");
  printf("Method 0 (default) determines the X- and Y-coordinate by taking the\n");
  printf("averages of the two end points of the scan line. The user-specified\n");
  printf("height is used as the Z-coordinate.\n");
  printf("Method 1 and 2 reconstruct the scanner position by estimating the\n");
  printf("location at which the angles between two consecutive laser beams is\n");
  printf("best approximated by a constant. I.e. the assumption is that the mirror\n");
  printf("speed as well as the laser pulse frequency is constant. In the case\n");
  printf("of method 1, this constant value is estimated. In the case of method 2,\n");
  printf("the user-specified value is used. This method is recommended for\n");
  printf("scanners with small opening angles, as the angle increment is highly\n");
  printf("correlated with the flight height.\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  void fp_cpp(char *, char *, int, double, double);

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  if (args->Contains("-methods")) {
    PrintMethods();
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

  // Check on parameters
  if (args->Integer("-m", 0) == 0 && !args->Contains("-fh")) {
    printf("Error: method 0 requires specification of flight height with -fh\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (args->Integer("-m", 0) == 2 && !args->Contains("-ai")) {
    printf("Error: method 2 requires specification of angle increment with -ai\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Determine flight path
  fp_cpp(args->String("-f"), args->String("-i"),
               args->Integer("-m", 0), args->Double("-fh", 1000.0),
               args->Double("-ai", 0.1) * atan(1.0) / 45.0);

  return EXIT_SUCCESS;
}
