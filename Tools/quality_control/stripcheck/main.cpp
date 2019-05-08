
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
#include "LaserUnit.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: stripcheck -i1 <strip 1 meta file> -i2 <strip 2 meta file>\n");
  printf("         [-o] <output file>\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  LaserUnit       strip1, strip2, overlap;
  FILE            *fd;
  int             num_pts1, num_pts2;
  
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i1") || !args->Contains("-i2")) {
    printf("Error: -i1 and -i2 are required arguments.\n");
    PrintUsage();
    exit(0);
  }

  // Open the two strips
  if (!strip1.ReadMetaData(args->String("-i1"))) {
    printf("Error reading strip 1: %s\n", args->String("-i1"));
    exit(0);
  }
  if (!strip2.ReadMetaData(args->String("-i2"))) {
    printf("Error reading strip 2: %s\n", args->String("-i2"));
    exit(0);
  }
  
  num_pts1 = strip1.NumberOfPoints();
  num_pts2 = strip2.NumberOfPoints();
  
  if (num_pts1 == num_pts2)
    printf("Number of points in strips %s and %s equals %d\n",
           strip1.Name(), strip2.Name(), num_pts1);
  else
    printf("Number of points in strips %s (%d) and %s (%d) DIFFER!\n",
           strip1.Name(), num_pts1, strip2.Name(), num_pts2);
           
  if (args->Contains("-o")) {
    fd = fopen(args->String("-o"), "a");
    if (fd == NULL)
      printf("Error opening file %s\n", args->String("-o"));
    else {
      if (num_pts1 == num_pts2)
        fprintf(fd, "Number of points in strips %s and %s equals %d\n",
               strip1.Name(), strip2.Name(), num_pts1);
      else
        fprintf(fd, "Number of points in strips %s (%d) and %s (%d) DIFFER!\n",
               strip1.Name(), num_pts1, strip2.Name(), num_pts2);
      fclose(fd);
    }
  }
  
  return EXIT_SUCCESS;
}
