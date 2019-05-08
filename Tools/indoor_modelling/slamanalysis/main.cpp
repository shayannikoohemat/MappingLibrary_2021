
/*
    Copyright 2014 University of Twente
 
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
  printf("Usage: slamanalysis -r <root file name without number>\n");
  printf("                    -n <number of files to include>\n");
  printf("                    [-f <number>] (first experiment number)\n");
}
 
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void slamanalysis_cpp(char *, int, int);

  if (args->Contains("-usage")) {
  	PrintUsage();
  	exit(0);
  }  
  if (!args->Contains("-r")) {
    printf("Error: -r is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-n")) {
    printf("Error: -n is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  slamanalysis_cpp(args->String("-r"), args->Integer("-n", 0),
                   args->Integer("-f", 0));

  return EXIT_SUCCESS;
}
