
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
|    Written By: 
|          Date: Nov 05, 2003
| Modifications:
|
------------------------------------------------------------*/

#include <stdio.h>

void PrintUsage()
{
  printf("Usage: histolaser [-i <laser file> OR -f <file filter>]\n");
  printf("                  [-x OR -y OR -z OR -t tagnumber]\n");
  printf("                  [-n <number of bins, def: 100>]\n");
  printf("                  -min <lower bound>\n");
  printf("                  -max <upper bound>\n");
  printf("                  -o <output image file>\n");
  printf("                  -ot <output text file>\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  int             type;
  void histolaser_cpp(char *, char *, int, int, int,
                      double, double, char *, char *);
  
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i") && !args->Contains("-f")) {
    printf("Error: -i or -f is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (args->Contains("-i") && args->Contains("-f")) {
    printf("Error: -i and -f are exclusive arguments.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-o") && !args->Contains("-ot")) {
    printf("Error: No output file specified with -o or -ot.\n");
    PrintUsage();
    exit(0);
  }
  type = -1;
  if (args->Contains("-t")) type = 0;
  if (args->Contains("-x")) {
    if (type != -1) {
      printf("Error: -x, -y, -z and -t are mutually exclusive.\n");
      PrintUsage();
      exit(0);
    }
    type = 1;
  }
  if (args->Contains("-y")) {
    if (type != -1) {
      printf("Error: -x, -y, -z and -t are mutually exclusive.\n");
      PrintUsage();
      exit(0);
    }
    type = 2;
  }
  if (args->Contains("-z")) {
    if (type != -1) {
      printf("Error: -x, -y, -z and -t are mutually exclusive.\n");
      PrintUsage();
      exit(0);
    }
    type = 3;
  }

  histolaser_cpp(args->String("-f"), args->String("-i"),
                 type, args->Integer("-t", 0),
                 args->Integer("-n", 100), args->Double("-min", 0.0),
                 args->Double("-max", 100.0), args->String("-o"),
				 args->String("-ot"));

  return EXIT_SUCCESS;
}
