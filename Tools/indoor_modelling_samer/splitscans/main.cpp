
/*
                    Copyright 2018 University of Twente 
 
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
  printf("splitscan splits the data of indoor mapping system into pieces of data\n");
  printf("acquired in short time intervals, typically corresponding to a single scan.\n");
  printf("If a system configuration file is supplied, splitscan assumes that the\n");
  printf("input data is in the original scanner coordinate systems and will apply\n");
  printf("the scanner orientations to obtain point sets in the frame coordinate system.\n");
  printf("\n");
  printf("Usage: splitscans -i <block file>\n");
  printf("                  -root <root name>\n");
  printf("                  [-ti <time interval in seconds> (default: 0.025)]\n");
  printf("                  [-odir <output directory for scan files> (default ./)]\n");
  printf("                  [-sys <system configuration file>]\n");
}
 
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void splitscans_cpp(char *, char *, double, char *, char *);

  if (args->Contains("-usage")) {
  	PrintUsage();
  	exit(0);
  }  
  if (!args->Contains("-i")) {
    printf("Error: -i is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-root")) {
    printf("Error: -root is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  splitscans_cpp(args->String("-i"), args->String("-root"),
                 args->Double("-ti", 0.025), args->String("-odir"),
				 args->String("-sys"));

  return EXIT_SUCCESS;
}
