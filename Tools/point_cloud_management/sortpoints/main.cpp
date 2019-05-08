
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

  sortpoints
  
  Sorting laser points after X-, Y-, and Z-coordinates
  
------------------------------------------------------------*/

#include <cstdlib>
#include <iostream>
#include <math.h>
#include "InlineArguments.h"
#include "SegmentationParameters.h"
#include "Line2D.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: sortpoints [-i <file name> OR -f <file filter>] : input files\n");
  printf("        [-x1, -y1, -x2, -y2]           : sort along line (x1, y1) to (x2, y2)\n");
  printf("        [-odir <directory name>]       : output directory\n");
  printf("        [-app <appendix>]              : append string for output files\n");
  printf("        [-overwrite]                   : overwrite input files\n");
  printf("        [-meta]                        : generate meta data files\n\n");
  printf("By default points are sorted after X-, Y- and Z-coordinates. If the\n");
  printf("coordinates of two points are specified with -x1, -y1, -x2 and -y2,\n");
  printf("points are sorted along the line between these two points.\n");
}

int main(int argc, char *argv[])
{
  InlineArguments        *args = new InlineArguments(argc, argv);
  SegmentationParameters par;
  bool                   sort_along_line=false;
  Line2D                 line;

  void sortpoints(char *, char *, char *, char *, bool, const Line2D &, bool, bool);

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
  if (!args->Contains("-app") && !args->Contains("-overwrite")) {
    printf("Error: no appendix for output files specified with -app <appendix>.\n");
    printf("       The input file is only overwritten if you specify -overwrite.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (args->Contains("-x1") && args->Contains("-y1") &&
      args->Contains("-x2") && args->Contains("-y2")) {
    sort_along_line = true;
    line = Line2D(Position2D(args->Double("-x1", 0.0), args->Double("-y1", 0.0)),
                  Position2D(args->Double("-x2", 0.0), args->Double("-y2", 0.0)));
  }
  
  // Sort the data
  sortpoints(args->String("-f"), args->String("-i"), args->String("-app"),
             args->String("-odir"), sort_along_line, line,
             args->Contains("-meta"), args->Contains("-overwrite")); 

  return EXIT_SUCCESS;
}
