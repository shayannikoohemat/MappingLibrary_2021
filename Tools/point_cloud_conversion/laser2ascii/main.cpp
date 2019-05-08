
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
  printf("Usage: laser2ascii -i <laser points file> OR\n");
  printf("   -f <file filter of laser point files>\n");
  printf("   -o <ascii output file>\n");
  printf("   -x (output of X-coordinate)\n");
  printf("   -y (output of Y-coordinate)\n");
  printf("   -z (output of Z-coordinate)\n");
  printf("   -r (output of red or reflectance value)\n");
  printf("   -g (output of green value)\n");
  printf("   -b (output of blue value)\n");
  printf("   -p (output of pulse count)\n");
  printf("   -lpf (output of last pulse flag)\n");
  printf("   -l (output of label)\n");
  printf("   -pl (output of pulse length)\n");
  printf("   -pn (output of plane number)\n");
  printf("   -scl (output of scan line)\n");
  printf("   -sn (output of segment number)\n");
  printf("   -lsn (output of long segment number, including segment start tile number\n");
  printf("   -poln (output of polygon number)\n");
  printf("   -fs (output of filter status)\n");
  printf("   -scan (output of scan (strip) number)\n");
  printf("   -t (output of time stamp)\n");
  printf("   -a (output of scan angle)\n");
  printf("   -sclstep n (only write every nth scanline)\n");
}
 
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void laser2ascii_cpp(char *, char *, int, int, int, int, int, int, int, int,
                       int, int, int, int, int, int, int, int, int, int, int, int, char *);
  
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
  if (!args->Contains("-o")) {
    printf("Error: -o is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  laser2ascii_cpp(args->String("-f"), args->String("-i"),
                  args->Contains("-x"), args->Contains("-y"),
                  args->Contains("-z"), args->Contains("-r"),
                  args->Contains("-g"), args->Contains("-b"),
                  args->Contains("-p"), args->Contains("-lpf"),
                  args->Contains("-l"),
                  args->Contains("-pl"), args->Contains("-pn"),
                  args->Contains("-sn"), args->Contains("-lsn"),
				  args->Contains("-poln"),
                  args->Contains("-fs"), args->Contains("-scan"),
                  args->Contains("-t"), args->Contains("-a"), args->Contains("-scl"),
                  args->Integer("-sclstep", 1),
                  args->String("-o"));

  return EXIT_SUCCESS;
}
