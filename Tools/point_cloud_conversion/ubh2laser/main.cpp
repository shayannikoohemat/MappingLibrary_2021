
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
#include <stdio.h>
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("ubh2laser converts output files of Hokuyo laser scanner to own format\n\n");
  printf("Usage: ubh2laser -i <input ubh file>\n");
  printf("                 -o <output laser file> OR\n");
  printf("                 -root <root name of output laser files per scan line>\n");
  printf("                 -txt (input is text file, no ubh file)\n");
  
  printf("\n         General optional parameters\n");
  printf("                 -odir <output directory name, def: .\\>\n");
  printf("                 -sscan <set value for scan number>\n");
  
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void ubh2laser_cpp(char *, char *, char *, char *, int, bool);

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i")) {
    printf("Error: Input file should be specified with -i.\n");
    PrintUsage();
    exit(0);
  }
  if (args->Contains("-o") && args->Contains("-root")) {
    printf("Error: -o and -root are mutually exclusive arguments.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-o") && !args->Contains("-root")) {
    printf("Error: Output file(s) should be specified with -o or -root.\n");
    PrintUsage();
    exit(0);
  }
  
  ubh2laser_cpp(args->String("-i"), args->String("-o"), args->String("-root"),
                args->String("-odir"), args->Integer("-sscan", -1),
				args->Contains("-txt"));

  return EXIT_SUCCESS;
}
