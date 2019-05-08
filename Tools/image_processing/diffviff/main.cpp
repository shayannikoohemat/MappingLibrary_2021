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
  printf("Subtraction of two images. Image2 is subtracted from image1.\n\n");
  printf("Usage: diffviff -i1 <input image 1> -i2 <input image 2> -o <output image>\n");
  printf("                 [-vi <no data value in input image (double), default: 0.0>]\n");
  printf("                 [-vo <no data value in output image (double), default: 0.0>]\n");
  printf("                 if -vi nor -vo are specified, no data values are not assumed\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  Image image1, image2, diffimage;
  unsigned char *pix1, *pix2;
  float *pixf1, *pixf2, *pixdiff;
  double vi, vo;
  bool all_data;

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input and output files
  if (!args->Contains("-i1")) {
    printf("Error: no image 1 specified with -i1 <image file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-i2")) {
    printf("Error: no image 2 specified with -i2 <image file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: no output image specified with -o <image file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Read the input images
  if (!image1.Read(args->String("-i1"))) {
    printf("Error reading  image 1 %s\n", args->String("-i1"));
    return EXIT_SUCCESS;
  }
  if (!image2.Read(args->String("-i2"))) {
    printf("Error reading  image 2 %s\n", args->String("-i2"));
    return EXIT_SUCCESS;
  }

  // Get no data values
  vi = args->Double("-vi", 0.0);
  vo = args->Double("-vo", 0.0);
  all_data = (!args->Contains("-vi") && !args->Contains("-vo"));

  // Combine the two images
  if (diffimage.NewImage(image1.NumRows(), image1.NumColumns(),
                         VFF_TYP_FLOAT) == NULL) {
    printf("Error allocating memory for output image.\n");
    return EXIT_SUCCESS;
  }
  
  if (image1.DataType() == VFF_TYP_FLOAT) {
    for (pixf1=image1.float_begin(), pixf2=image2.float_begin(),
         pixdiff=diffimage.float_begin();
         pixf1!=image1.float_end();
         pixf1++, pixf2++, pixdiff++) {
      if (fabs(*pixf1 - vi) < 0.0001 || fabs(*pixf2 - vi) < 0.0001)
        *pixdiff = vo;
      else
        *pixdiff = *pixf1 - (*pixf2);
    }
  }
  else {
    for (pix1=image1.begin(), pix2=image2.begin(),
         pixdiff=diffimage.float_begin();
         pix1!=image1.end();
         pix1++, pix2++, pixdiff++) {
      if ((fabs((float) (*pix1) - vi) < 0.0001 || 
           fabs((float) (*pix2) - vi) < 0.0001) && !all_data)
        *pixdiff = vo;
      else
        *pixdiff = (float) (*pix1) - (float) (*pix2);
    }
  }
              
  // Write the output
  diffimage.Write(args->String("-o"));
  
  return EXIT_SUCCESS;
}
