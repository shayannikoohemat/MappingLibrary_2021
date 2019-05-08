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
#include <math.h>
#include <stdio.h>
#include "InlineArguments.h"
#include "Image.h"

using namespace std;

void PrintUsage()
{
  printf("Generate a float image with a Gaussian probability density function.\n\n");
  printf("Usage: gengauss -o <viff file>\n");
  printf("                [-nr <number of rows>] (default: 100)\n");
  printf("                [-nc <number of columns>] (default: 100)\n");
  printf("                [-mr <mean row coordinate>] (default: centre)\n");
  printf("                [-mc <mean column coordinate>] (default: centre)\n");
  printf("                [-s <sigma>] (default: 1.0)\n");
}
  
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  Image image;
  float mr, mc, sigma, *pixel, dr, dc, pi, k, sum;
  int r, c;

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required output file
  if (!args->Contains("-o")) {
    printf("Error: no output file specified with -o <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Set up the image
  image.NewImage(args->Integer("-nr", 100), args->Integer("-nc", 100),
                 VFF_TYP_FLOAT, 1);

  // Get parameters of the distribution
  mr = args->Double("-mr", ((double) image.NumRows() - 1.0) / 2.0);
  mc = args->Double("-mc", ((double) image.NumColumns() - 1.0) / 2.0);
  sigma = args->Double("-s", 1.0);
  pi = 4.0 * atan(1.0);
  
  // Set all pixel values
  for (r=0, pixel=image.float_begin(), sum=0.0; r!=image.NumRows(); r++) {
    dr = r - mr;
    for (c=0; c!=image.NumColumns(); c++, pixel++) {
      dc = c - mc;
      k = (dr*dr + dc*dc) / (sigma * sigma);
      *pixel = (1.0 / (2.0 * pi * sigma * sigma)) * exp(-0.5 * k);
      sum += *pixel;
    }
  }
  printf("Sum of all pixels %.4f\n", sum);
  
  // Write the image
  if (!image.Write(args->String("-o")))
    printf("Error writing output image %s\n", args->String("-o"));
  exit(0);
}
