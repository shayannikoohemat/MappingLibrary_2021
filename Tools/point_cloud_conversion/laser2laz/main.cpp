
/*
                   Copyright 2013 University of Twente
 
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
  printf("Conversion of .laser file(s) to .laz file(s)\n");
  printf("Note that many attributes of .laser files cannot be stored in .laz files\n");
  printf("Usage: laser2laz -i <laser points file> OR\n");
  printf("                 -f <file filter of laser point files>\n");
  printf("                 [-odir <output directory>] (default .)\n");
  printf("                 [-o <output file>] (e.g. for storing a whole block in one laz file\n");
}
 
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void laser2laz_cpp(char *, char *, const char *, bool, char *);
  
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

  laser2laz_cpp(args->String("-f"), args->String("-i"),
                args->String("-odir", "./"),
				args->Contains("-o"), args->String("-o"));

  return EXIT_SUCCESS;
}
