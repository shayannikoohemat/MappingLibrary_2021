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

extern "C" int epipolar_c(char *, char *, char *, char *, char *,
                          char *, char *, char *, char *, char *);

void PrintUsage()
{
  printf("Derivation of normal images (epipolar lines in same rows).\n\n");
  printf("Usage: epipolar -i1 <First image>\n");
  printf("                -i2 <Second image>\n");
  printf("                -rel <Relative orientation parameters>\n");
  printf("                -int1 <Interior orientation parameters of first image>\n");
  printf("                [-int2 <Interior orientation parameters of second image>]\n");
  printf("                -o1 <Normal image of first image>\n");
  printf("                -o2 <Normal image of second image>\n");
  printf("                [-orel <Relative orientation of normal images>]\n");
  printf("                [-oint1 <Interior orientation of first normal image>]\n");
  printf("                [-oint2 <Interior orientation of second normal image>]\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  
// Usage

  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
// Check on required input and output parameters

  if (!args->Contains("-i1") || !args->Contains("-i2")) {
    printf("Error: Images should be specified with -i1 and -i2.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-rel")) {
    printf("Error: Relative orientation should be specified with -rel.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-int1")) {
    printf("Error: Interior orientation of first image should be specified with -int1.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o1") || !args->Contains("-o2")) {
    printf("Error: Output normal images should be specified with -o1 and -o2.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

// Call the epipolar C routine

  if (args->Contains("-int2"))
    epipolar_c(args->String("-i1"), args->String("-i2"),
               args->String("-rel"), args->String("-int1"), args->String("-int2"),
               args->String("-o1"), args->String("-o2"), args->String("-orel"),
               args->String("-oint1"), args->String("-oint2"));
  else
    epipolar_c(args->String("-i1"), args->String("-i2"),
               args->String("-rel"), args->String("-int1"), args->String("-int1"),
               args->String("-o1"), args->String("-o2"), args->String("-orel"),
               args->String("-oint1"), args->String("-oint2"));
    
  return EXIT_SUCCESS;
}
