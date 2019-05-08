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
  printf("Create an image of a profile of a row of another.\n\n");
  printf("Usage: imageprofile -i <input image>\n");
  printf("                    -o <output profile image>\n");
  printf("                    [-r <row number> (default: middle row)]\n");
}
  
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  Image image, profile;
  float *pixelf, min_gv, max_gv, gv;
  unsigned char *pixel;
  int r, c, rp, cp, rp_prev, rpp, rpp_start, rpp_end, num_profile_rows;

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

  // Read the input images
  if (!image.Read(args->String("-i"))) {
    printf("Error reading input image from %s\n", args->String("-i"));
    exit(0);
  }
  
  // Set up the image with white pixels
  num_profile_rows = 201;
  profile.NewImage(num_profile_rows + image.NumRows(), 
                   image.NumColumns(), VFF_TYP_1_BYTE, 1);
  profile.SetPixels((unsigned char) 255);
  
  // Select the row
  r = args->Integer("-r", image.NumRows()/2);
  if (r < 0 || r >= image.NumRows()) {
    printf("Error: row outside range (0-%d)\n", image.NumRows()-1);
    exit(0);
  }

  // Draw horizontal line 
  // dog_gauss min -0.015  max 0.045
  // unknown -0.25 0.75
  // dog_circles -15 45
  // nussallee -100 100
  min_gv = -100.0;
  max_gv =  100.0;
  rp = (int) (max_gv / (max_gv - min_gv) * num_profile_rows);
  for (cp=0; cp<profile.NumColumns(); cp++)
    *(profile.Pixel(rp, cp)) = 0;
  rp_prev = rp;
  
  // Draw profile
  for (cp=0; cp<profile.NumColumns(); cp++) {
    gv = *(image.FloatPixel(r, cp));
    rp = (int) ((max_gv - gv) / (max_gv - min_gv) * num_profile_rows);
    if (rp < 0 || rp > num_profile_rows) {
      printf("Error: row outside range: %.4f %d (0-%d)\n", gv, rp, num_profile_rows-1);
      exit(0);
    }
    else {
      rpp_start = rpp_end = rp;
      if (rp_prev < rp) rpp_start = rp_prev;
      else rpp_end = rp_prev;
      for (rpp=rpp_start; rpp<=rpp_end; rpp++)
        *(profile.Pixel(rpp, cp)) = 0;
      rp_prev = rp;
    }
  }
  
  // Convert image float values to grey values below the profile
  for (pixelf=image.float_begin(), pixel=profile.Pixel(num_profile_rows, 0);
       pixelf!=image.float_end(); pixelf++, pixel++)
    *pixel = (unsigned char) ((*pixelf - max_gv) / (max_gv - min_gv) * 255);
    
  // Write the image 
  if (!profile.Write(args->String("-o")))
    printf("Error writing output image %s\n", args->String("-o"));
  exit(0);
}
