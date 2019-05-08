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
  printf("Colouring of a float image. Written for colour coding of\n");
  printf("a height difference image, but applicable to all float images.\n\n");
  printf("Usage: colourdiff -i <input image> -o <output image>\n");
  printf("                 -t1 <value of first threshold (double)>\n");
  printf("                 -r0, -g0, -b0 <colour of values lower than t1\n");
  printf("                 -r1, -g1, -b1 <colour of values between t1 and t2\n");
  printf("                 maximum threshold t9\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  Image image1, image2;
  unsigned char *pixr, *pixb, *pixg, *pixc;
  bool hast[10], done;
  float *pixf;
  double t[11], r[11], g[11], b[11];
  char par[6];
  int i, numpix;
  

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
  if (!args->Contains("-t1")) {
    printf("Error: missing threshold t1\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-g0") || !args->Contains("-g1")) {
    printf("Error: missing colour values g0 or g1 \n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Read the input images
  if (!image1.Read(args->String("-i"))) {
    printf("Error reading image %s\n", args->String("-i"));
    return EXIT_SUCCESS;
  }

  // Get thresholds and colours
  for (i=0; i<10; i++) {
    sprintf(par, "-t%d", i);
    hast[i] = args->Contains(par);
    t[i] = args->Double(par, 0.0);
    sprintf(par, "-g%d", i);
    g[i] = args->Double(par, 0.0);
    sprintf(par, "-r%d", i);
    if (args->Contains(par)) r[i] = args->Double(par, 0.0);
    else r[i] = g[i];
    sprintf(par, "-b%d", i);
    if (args->Contains(par)) b[i] = args->Double(par, 0.0);
    else b[i] = g[i];
  }
  hast[0] = true;

  // Colour the image
  if (image2.NewImage(image1.NumRows(), image1.NumColumns(),
                      VFF_TYP_1_BYTE, 3) == NULL) {
    printf("Error allocating memory for output image.\n");
    return EXIT_SUCCESS;
  }
  image2.GetImage()->color_space_model = VFF_CM_genericRGB;

  numpix = image1.NumRows() * image1.NumColumns();
  if (image1.DataType() == VFF_TYP_FLOAT) {
    for (pixf=image1.float_begin(), pixr=image2.begin(),
         pixg=pixr+numpix, pixb=pixg+numpix;
         pixf!=image1.float_end();
         pixf++, pixr++, pixg++, pixb++) {
      done = false;
      for (i=1; i<10 && !done; i++) {
        if ((hast[i] && *pixf < t[i]) || !hast[i]) {
          *pixr = (unsigned char) r[i-1];
          *pixg = (unsigned char) g[i-1];
          *pixb = (unsigned char) b[i-1];
          done = true;
        }
      }
    }
  }
  else {
    for (pixc=image1.begin(), pixr=image2.begin(),
         pixg=pixr+numpix, pixb=pixg+numpix;
         pixc!=image1.begin()+numpix;
         pixc++, pixr++, pixg++, pixb++) {
      done = false;
      for (i=1; i<10 && !done; i++) {
        if ((hast[i] && *pixc < t[i]) || !hast[i]) {
          *pixr = (unsigned char) r[i-1];
          *pixg = (unsigned char) g[i-1];
          *pixb = (unsigned char) b[i-1];
          done = true;
        }
      }
    }
  }
              
  // Write the output
  image2.Write(args->String("-o"));
  
  return EXIT_SUCCESS;
}
