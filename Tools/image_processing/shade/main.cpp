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
#include <math.h>
#include "InlineArguments.h"
#include "Image.h"

using namespace std;

void PrintUsage()
{
  printf("Generating a shaded image (using the logarithm of a diagonal gradient).\n\n");
  printf("Usage: shade -i <input image> -o <output image>\n");
}

extern "C" xvimage *readimage(const char*);
extern "C" void writeimage(const char *, xvimage *);

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  xvimage *image1, *image2;
  void ShadeImage(xvimage *, xvimage **, int);
  
  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input and output files
  if (!args->Contains("-i")) {
    printf("Error: no input image specified with -i <image file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: no output image specified with -o <image file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Shade the image
  image1 = readimage(args->String("-i"));
  if (!image1) {
    fprintf(stderr, "Error reading image %s\n", args->String("-i"));
    exit(0);
  }
  
  ShadeImage(image1, &image2, 1);

  writeimage(args->String("-o"), image2);
              
  return EXIT_SUCCESS;
}
