
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
  poles
  
  labeling of poles from MLS
  
  (c) by Martin Rutzinger Nov. 2009, ITC Enschede
  
------------------------------------------------------------*/

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include "InlineArguments.h"


using namespace std;

void PrintUsage()
{
  printf("Usage: poles [-i <file name> OR -f <file filter>] : input files\n");
//  printf("        [-odir <directory name>]            : output directory\n");
//  printf("        [-app <appendix>]                   : append string for output files\n");
//  printf("        [-meta]                             : generate meta data files\n");
//  printf("        [-overwrite]                        : overwrite old files\n");
  printf("\n");
  printf("Parameters for poles extraction\n");
  printf("        [-per (def: 4)]                      : number of percentiles\n");
  printf("        [-nperpole (def: 3)]                 : number of percentile for pole extraction (min. enclosing rectangle)\n");
  printf("        [-parts (def: 0.1)]                  : height of parts to select points for min. encl. rect. in nperpole\n");
  printf("        [-maxpts (def: 5000)]                : maximum number of points in parts for min. encl. rect.\n"); 
  printf("        [-numparts (def: 2)]                 : class: min number of rect. parts passing class criteria\n");
  printf("        [-maxdiagpart (def: 0.3)]            : rect: maximum diagonal of rect.\n");  
  printf("        [-diffpos (def: 0.05)]               : rect: diff of position of rect. center between two adjacent parts\n");
  printf("        [-diffdiag (def: 0.2)]               : rect: diff of diagonal between two adjacent rect. parts\n");
  printf("        [-tag (def: 1)]                      : tag used for labeling: 1 -> LabelTag, 2 -> ComponentNumberTag\n");  
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  
  void poles(char *, char *, char *, char *, bool, bool,
                    int, int, float, int, int, float, float, float, int);

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
  
  // Check on required input files
  if (!args->Contains("-i") && !args->Contains("-f")) {
    printf("Error: no input data specified with -i <filename> or -f <filter>.\n");
    PrintUsage();
    exit(0);
  }
  if (args->Contains("-i") && args->Contains("-f")) {
    printf("Error: -i and -f can not be used simultaneously!\n");
    PrintUsage();
    exit(0);
  }
  
/*  if (!args->Contains("-app") && !args->Contains("-overwrite")) {
    printf("Error: no appendix for output files specified with -app <appendix>.\n");
    printf("       The input file is only overwritten if you specify -overwrite.\n");
    PrintUsage();
    exit(0);
  }
*/
  
  //Parameters for poles extraction

  poles(args->String("-f"), args->String("-i"),
               args->String("-app"),
               args->String("-odir"),
               args->Contains("-meta"), 
               args->Contains("-overwrite"),
               args->Integer("-per", 4),
               args->Integer("-nperpole", 3),
               args->Double("-parts", 0.1),           
               args->Integer("-maxpts", 5000),
               args->Double("-numparts", 2),
               args->Double("-maxdiagpart", 0.3),
               args->Double("-diffpos", 0.05),
               args->Double("-diffdiag", 0.02),
               args->Integer("-tag", 1));



  return EXIT_SUCCESS;
}
