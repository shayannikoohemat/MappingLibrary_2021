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
  printf("Convolve two float images.\n\n");
  printf("Usage: convolve -i1 <first input image>\n");
  printf("                -i2 <second input image>\n");
  printf("                -0 <output image>\n");
}
  
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  Image image1, image2, output_image, *input_image, *kernel;
  float *pixel;
  int r, c, rk, ck;

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input and output files
  if (!args->Contains("-i1")) {
    printf("Error: no first input image specified with -i1 <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-i2")) {
    printf("Error: no second input file specified with -i2 <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: no output image specified with -o <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Read the input images
  if (!image1.Read(args->String("-i1"))) {
    printf("Error reading first input image from %s\n", args->String("-i1"));
    exit(0);
  }
  if (!image2.Read(args->String("-i2"))) {
    printf("Error reading second input image from %s\n", args->String("-i2"));
    exit(0);
  }

  // Determine kernel and image
  if (image1.NumRows() >= image2.NumRows() &&
      image1.NumColumns() >= image2.NumColumns()) {
    input_image = &image1;
    kernel = &image2;
  }
  else if (image1.NumRows() <= image2.NumRows() &&
           image1.NumColumns() <= image2.NumColumns()) {
    input_image = &image2;
    kernel = &image1;
  }
  else {
    printf("Error: An image with size (%dx%d) cannot be convolved with an image of size (%dx%d)\n",
           image1.NumRows(), image1.NumColumns(),
           image2.NumRows(), image2.NumColumns());
  }
  
  // Check if the kernel is a float image
  if (kernel->DataType() != VFF_TYP_FLOAT) {
    printf("Error: Kernel image should be of type VFF_TYP_FLOAT.\n");
    exit(0);
  }
  
  // Set up the image
  output_image.NewImage(input_image->NumRows() - kernel->NumRows() + 1,
                        input_image->NumColumns() - kernel->NumColumns() + 1,
                        VFF_TYP_FLOAT, 1);

  // Convolve float image
  if (input_image->DataType() == VFF_TYP_FLOAT) {
    for (r=0, pixel=output_image.float_begin(); r<output_image.NumRows(); r++) {
      for (c=0; c<output_image.NumColumns(); c++, pixel++) {
        *pixel = 0.0;
        for (rk=0; rk<kernel->NumRows(); rk++) {
          for (ck=0; ck<kernel->NumColumns(); ck++) {
//            printf("pix %d %d input at %d %d, kernel at %d %d\n", r, c,
//                   r+rk, c+ck, kernel->NumRows()-1-rk, kernel->NumColumns()-1-ck);
            *pixel += *(input_image->FloatPixel(r+rk, c+ck)) *
                      *(kernel->FloatPixel(kernel->NumRows()-1-rk,
                                           kernel->NumColumns()-1-ck));
          }
        }
      }
    }
  }
  // Convolve unsigned char image
  else {
    for (r=0, pixel=output_image.float_begin(); r<output_image.NumRows(); r++) {
      for (c=0; c<output_image.NumColumns(); c++, pixel++) {
        *pixel = 0.0;
        for (rk=0; rk<kernel->NumRows(); rk++) {
          for (ck=0; ck<kernel->NumColumns(); ck++) {
//            printf("pix %d %d input at %d %d, kernel at %d %d\n", r, c,
//                   r+rk, c+ck, kernel->NumRows()-1-rk, kernel->NumColumns()-1-ck);
            *pixel += *(input_image->Pixel(r+rk, c+ck)) *
                      *(kernel->FloatPixel(kernel->NumRows()-1-rk,
                                           kernel->NumColumns()-1-ck));
          }
        }
      }
    }
  }
  
  // Write the image
  if (!output_image.Write(args->String("-o")))
    printf("Error writing output image %s\n", args->String("-o"));
  exit(0);
}
