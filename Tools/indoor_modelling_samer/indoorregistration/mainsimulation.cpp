
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
  printf("indoorregistrationsimulation simulates measurements to test the\n");
  printf("registration of the scanners w.r.t. each other.\n\n");
  printf("Usage: indoorregistrationsimulation\n");
  printf("            -p <simulated point cloud\n");
  printf("            [-np <number of planes, default 1, max 6>\n");
  printf("            [-o0 <simulated observations sensor 0>]\n");
  printf("            [-o1 <simulated observations sensor 1>]\n");
  printf("            [-o2 <simulated observations sensor 2>]\n");
  printf("            [-sdir <output directory for scan files>]\n");
  printf("            [-of] (output scans in frame coordinate system)\n");
  printf("            [-r <sensor rotation in degrees, default: 20>]\n");
  printf("            [-a <opening angle, default: 270.0 degree>]\n");
  printf("            [-ai <angle increment, default: 0.25 degree>]\n");
  printf("            [-std <standard deviation of range noise, default: 0.0>]\n");
  printf("            [-pl <planes> (needed for absolute registration)]\n");
  printf("            [-osys <output system configuration file with improved values>]\n");
}
 
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void indoorregistrationsimulation_cpp(char *, char *, char *, char *, char *, bool,
                              double, double, double, double, int, char *, char *);

  if (args->Contains("-usage")) {
  	PrintUsage();
  	exit(0);
  }  
  if (!args->Contains("-p")) {
  	printf("Error: output file not specified with -p\n");
  	PrintUsage();
  	exit(0);
  }

  indoorregistrationsimulation_cpp(args->String("-p"), args->String("-o0"),
                                   args->String("-o1"), args->String("-o2"),
						           args->String("-sdir"), args->Contains("-of"),
                                   args->Double("-r", 20.0),
                                   args->Double("-a", 270.0), 
								   args->Double("-ai", 0.25), 
						           args->Double("-std", 0.0),
								   args->Integer("-np", 1),
								   args->String("-pl"),
								   args->String("-osys"));

  return EXIT_SUCCESS;
}
