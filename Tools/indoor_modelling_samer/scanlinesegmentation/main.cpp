
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

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("scanlinesegmentation segments data of an indoor scan and labels\n");
  printf("horizontal and vertical line segments.\n");
  printf("The input file can be either a file with laser points or\n");
  printf("a meta data file of a block of strips with laser points\n");
  printf("In the first case, the output is specified with -o. In the latter\n");
  printf("case, -app is used to specify an appendix to the input file names.\n");
  printf("For a block, it will be assumed that the strips are listed in the\n");
  printf("sequence of the sensor ids 0, 1 and 2. For a single point file the\n");
  printf("scanner id needs to be specified with -scanner.\n\n");
  printf("Usage: scanlinesegmentation -i <input file>\n");
  printf("                            [-o <output file>]\n");
  printf("                            [-scanner <id: 0, 1 or 2>]\n");
  printf("                            [-app <appendix string>]\n");
  printf("                            [-m <method: 0 - greedy, 1 (default) - range of residuals (needs ptsStart, percentagePts, threshold parameters)>]\n");
  printf("                            [-ptsStart <number of points for the initial line estimate, default: 3>\n");
  printf("                            [-percentagePts <percentage of segment points to be checked, default: 0.15>\n");
  printf("                            [-threshold <maximum deviation from the line, default: 0.0095>\n");
  printf("                            [-minSize <minimum number of points in a segment, default: 10>\n");
  printf("                            [-t (add time offset)]\n");
  printf("                            [-ol] (use old point labels)\n");
  printf("                            [-labeloffset (offset segment labels per file)\n");
}
 
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void scanlinesegmentation_cpp(char *, const char *, bool, bool, int, int, int,
                                double, double, int, char *, bool);
  
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }

  if (!args->Contains("-i")) {
    printf("Error: No input file specified with -i.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-o") && !args->Contains("-app")) {
    printf("Error: No output file specified with -o.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-scanner") && !args->Contains("-app")) {
    printf("Error: No scanner id specified with -scanner.\n");
    PrintUsage();
    exit(0);
  }

  scanlinesegmentation_cpp(args->String("-i"),
                           args->String("-o", args->String("-i")),
						   args->Contains("-t"),
						   args->Contains("-ol"),
						   args->Integer("-scanner", 0),
						   args->Integer("-m", 1),
						   args->Integer("-ptsStart", 3),
						   args->Double("-percentagePts", 0.15),
						   args->Double("-threshold", 0.0095),
						   args->Integer("-minSize", 10),
						   args->String("-app"),
						   args->Contains("-labeloffset"));
  return EXIT_SUCCESS;
}
