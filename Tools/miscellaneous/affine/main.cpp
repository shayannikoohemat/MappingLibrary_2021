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
#include <math.h>
#include "InlineArguments.h"
#include "Image.h"

using namespace std;

void PrintUsage()
{
  printf("Affine transformation of an image with fixed parameters.\n\n");
  printf("Usage: affine -i <input float image>\n");
  printf("              -o <output unsigned char image>\n");
  printf("              -min <minimum float pixel value (grey value 0)>\n");
  printf("              -max <maximum float pixel value (grey value 255)>\n");
  printf("              [-rs <row scale (int, def: 2)>]\n");
  printf("              [-sf <shear factor (int, def: 1)]>\n");
}
  
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  Image image1, image2;
  float *pixel, gv_min, gv_max;
  int r, c, shear, row_scale;

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input and output files
  if (!args->Contains("-i")) {
    printf("Error: no input image specified with -i <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: no output image specified with -o <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-min") || !args->Contains("-max")) {
    printf("Error: minimum and maximum values should be specified with -min and -max\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Read the input image
  if (!image1.Read(args->String("-i"))) {
    printf("Error reading first input image from %s\n", args->String("-i"));
    exit(0);
  }
  
  shear = args->Integer("-sf", 1);
  row_scale = args->Integer("-rs", 2);
  // Set up the output image
  image2.NewImage(image1.NumRows()/row_scale,
                  image1.NumColumns() + image1.NumRows()*shear/row_scale,
                  VFF_TYP_1_BYTE, 1);

  // Transform image
  gv_min = args->Double("-min", 0.0);
  gv_max = args->Double("-max", 0.0);
  image2.SetPixels((unsigned char) 255);
  for (r=0; r<image2.NumRows(); r++) {
    for (c=0; c<image1.NumColumns(); c++)
      // White border
      if (r < 2 || r > image2.NumRows()-3 || c < shear*2 || c > image1.NumColumns()-1-shear*2)
        *(image2.Pixel(r, c+image2.NumRows()*shear-1-r*shear)) = 254;
      else
        *(image2.Pixel(r, c+image2.NumRows()*shear-1-r*shear)) = 
          (unsigned char) (255.0 * (*(image1.FloatPixel(r*row_scale, c)) - gv_min) / (gv_max - gv_min));
  }
  
  // Write the image
  if (!image2.Write(args->String("-o")))
    printf("Error writing output image %s\n", args->String("-o"));
  exit(0);
}
