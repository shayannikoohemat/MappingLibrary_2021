
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
  printf("Usage: imageoffsets -i <offset file>\n");
  printf("                    -b <overlap block file>\n");
  printf("                    [-idir <overlap image directory>]\n");
  printf("                    [-hoff] (image height offsets\n");
  printf("                    [-hrms] (image height RMS values\n");
  printf("                    [-hstdev] (image height standard deviations\n");
  printf("                    [-poff] (image planimetric offsets\n");
  printf("                    [-prms] (image planimetric RMS values\n");
  printf("                    [-pstdev] (image planimetric standard deviations\n");
  printf("                    [-pqual] (image planimetric quality test value\n");
  printf("                    [-minroofs <minimum number of roofs (def: 10)>]\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void imageoffsets_cpp(const char *, const char *, const char *,
                        bool, bool, bool, bool, bool, bool, bool, int);

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i")) {
    printf("Error: -i is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-b")) {
    printf("Error: -b is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  imageoffsets_cpp(args->String("-i", NULL), args->String("-b", NULL),
                   args->String("-idir", NULL),
                   args->Contains("-hoff"), args->Contains("-hrms"),
                   args->Contains("-hstdev"),
                   args->Contains("-poff"), args->Contains("-prms"),
                   args->Contains("-pstdev"), args->Contains("-pqual"),
                   args->Integer("-minroofs", 10));

  return EXIT_SUCCESS;
}
