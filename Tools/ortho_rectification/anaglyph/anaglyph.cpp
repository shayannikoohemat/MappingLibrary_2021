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

extern "C" int anaglyph_c(char *, char *, char *, int, int, int, int);

void PrintUsage()
{
  printf("Composition of an anaglyph image from two normal images.\n\n");
  printf("Usage: anaglyph -i1 <Left image>\n");
  printf("                -i2 <Right image>\n");
  printf("                -o <Anaglyph image (24 bit)>\n");
  printf("                [-left_band <Band for left image, default: 1=red>]\n");
  printf("                [-right_band <Band for right image, default: 2=green>]\n");
  printf("                [-dr <Row shift between the images, default: 0 pixels>]\n");
  printf("                -dc <Column shift between the images>\n");
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
  if (!args->Contains("-o")) {
    printf("Error: Anaglyph output image should be specified with -o.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-dc")) {
    printf("Error: Column shift between images should be specified with -dc.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

// Call the anaglyph C routine

  anaglyph_c(args->String("-i1"), args->String("-i2"),
             args->String("-o"), args->Integer("-left_band", 1),
             args->Integer("-right_band", 2),
             args->Integer("-dr", 0), args->Integer("-dc", 0));
    
  return EXIT_SUCCESS;
}
