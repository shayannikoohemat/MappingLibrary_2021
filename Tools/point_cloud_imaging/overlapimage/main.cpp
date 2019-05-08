
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
#include <string.h>
#include "InlineArguments.h"
#include "Image.h"

using namespace std;

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(char *, char *, int *);

void PrintUsage()
{
  printf("Usage: overlapimage -f <filter for strip images> -o <output image>\n");
  printf("         [-r0 <n> -g0 <n> -b0 <n> (background colour, def: 0,0,0)]\n");
  printf("         [-r1 <n> -g1 <n> -b1 <n> (single coverage colour, def: 0,0,255)]\n");
  printf("         [-r2 <n> -g2 <n> -b2 <n> (double coverage colour, def: 255,255,0)]\n");
  printf("         [-r3 <n> -g3 <n> -b3 <n> (3-fold coverage colour, def: 255,0,0)]\n");
  printf("         [-r4 <n> -g4 <n> -b4 <n> (4-fold coverage colour, def: 255,0,0)]\n");
  printf("         [-r5 <n> -g5 <n> -b5 <n> (5-fold coverage colour, def: 255,0,0)]\n");
  printf("         [-r6 <n> -g6 <n> -b6 <n> (6-fold coverage colour, def: 255,0,0)]\n");
  printf("         [-r7 <n> -g7 <n> -b7 <n> (7-fold coverage colour, def: 255,0,0)]\n");
  printf("         [-r8 <n> -g8 <n> -b8 <n> (8-fold coverage colour, def: 255,0,0)]\n");
  printf("         [-r9 <n> -g9 <n> -b9 <n> (9-fold coverage colour, def: 255,0,0)]\n");
  printf("         [-r10 <n> -g10 <n> -b10 <n> (10 or morefold coverage colour, def: 255,0,0)]\n");
  printf("         [-binary (do not use grey values of strip images)]\n");
  printf("         [-low <n> (lower bound for grey values, def: 0)]\n");
  printf("         [-high <n> (higher bound for grey values, def: 255)]\n");
  printf("         [-v (print progress information)]\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  char            *directory, *filter, *filename;
  int             icon, index;
  bool            init_done=false, binary, verbose;
  Image           strip_image, overlap_image;
  xvimage         *overlap_xvimage;
  unsigned char   *strip_pixel, *count, *average, *red, *green, *blue;
  int             r[11], g[11], b[11], low, high;
  double          scale;

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                  
  verbose = args->Contains("-v");
       
  if (!args->Contains("-f")) {
    printf("Error: -f is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-o")) {
    printf("Error: -o is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  // Loop over all strip images
  // Set up the file filter for the input file(s)
  filter = args->String("-f");
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    // Read the strip image
    if (!strip_image.Read(filename)) {
      printf("Error reading strip image %s\n", filename);
      exit(0);
    }
    
    if (verbose) printf("Processing strip image %s        \r", filename);
    
    // Initialise the overlap image
    if (!init_done) {
      overlap_xvimage = overlap_image.NewImage(strip_image.NumRows(),
                                               strip_image.NumColumns(),
                                               VFF_TYP_1_BYTE, 3);
      overlap_image.ClearImage();   
      init_done = true;                        
    }
    // Update the strip counts and grey values
    for (strip_pixel=strip_image.begin(), count=overlap_image.begin(),
         average=count+strip_image.NumRows()*strip_image.NumColumns();
         strip_pixel!=strip_image.end();
         strip_pixel++, count++, average++) {
      if (*strip_pixel != 0) {
        *average = (unsigned char) (((float) (*average) * (*count) + (*strip_pixel)) /
                    ((float) (*count) + 1.0));
        (*count)++;
      }
    }
  }
  
  // Get colour specifications
  r[0] = args->Integer("-r0", 0);
  g[0] = args->Integer("-g0", 0);
  b[0] = args->Integer("-b0", 0);
  r[1] = args->Integer("-r1", 0);
  g[1] = args->Integer("-g1", 0);
  b[1] = args->Integer("-b1", 255);
  r[2] = args->Integer("-r2", 255);
  g[2] = args->Integer("-g2", 255);
  b[2] = args->Integer("-b2", 0);
  r[3] = args->Integer("-r3", 255);
  g[3] = args->Integer("-g3", 0);
  b[3] = args->Integer("-b3", 0);
  r[4] = args->Integer("-r4", 255);
  g[4] = args->Integer("-g4", 0);
  b[4] = args->Integer("-b4", 0);
  r[5] = args->Integer("-r5", 255);
  g[5] = args->Integer("-g5", 0);
  b[5] = args->Integer("-b5", 0);
  r[6] = args->Integer("-r6", 255);
  g[6] = args->Integer("-g6", 0);
  b[6] = args->Integer("-b6", 0);
  r[7] = args->Integer("-r7", 255);
  g[7] = args->Integer("-g7", 0);
  b[7] = args->Integer("-b7", 0);
  r[8] = args->Integer("-r8", 255);
  g[8] = args->Integer("-g8", 0);
  b[8] = args->Integer("-b8", 0);
  r[9] = args->Integer("-r9", 255);
  g[9] = args->Integer("-g9", 0);
  b[9] = args->Integer("-b9", 0);
  r[10] = args->Integer("-r10", 255);
  g[10] = args->Integer("-g10", 0);
  b[10] = args->Integer("-b10", 0);
  binary = args->Contains("-binary");
  low    = args->Integer("-low", 0);
  if (low < 0) low = 0;
  if (low > 254) low = 254;
  high   = args->Integer("-high", 255);
  if (high < 1) high = 1;
  if (high > 255) high = 255;
  scale = (double) (high - low) / 255.0;
  
  if (verbose) printf("Converting counts to colours                                                         \n");
  // Convert counts and averages to colours
  for (strip_pixel=strip_image.begin(), count=overlap_image.begin(),
       average=count+strip_image.NumRows()*strip_image.NumColumns(),
       red=count, green=average,
       blue=green+strip_image.NumRows()*strip_image.NumColumns();
       strip_pixel!=strip_image.end();
       strip_pixel++, count++, average++, red++, green++, blue++) {
    if (binary) {
      if (*count == 0) *average = 0;
      else *average = 255;
    }
    if (*count < 10) {
      index = *count;
      *red   = (unsigned char) ((int) *average * r[index] / 255);
      *blue  = (unsigned char) ((int) *average * b[index] / 255);
      *green = (unsigned char) ((int) *average * g[index] / 255);
    }
    else {
      *red   = (unsigned char) ((int) *average * r[10] / 255);
      *blue  = (unsigned char) ((int) *average * b[10] / 255);
      *green = (unsigned char) ((int) *average * g[10] / 255);  
    }
    // Apply grey value transformation
    if (low != 0 || high != 255) {
      *red   = (unsigned char) (scale * (*red) + low);
      *green = (unsigned char) (scale * (*green) + low);
      *blue  = (unsigned char) (scale * (*blue) + low);
    }
  }
  
  // Set colour characteristics
  overlap_xvimage->color_space_model = VFF_CM_genericRGB;
 
  // Write the overlap image
  overlap_image.Write(args->String("-o"));
  
  return EXIT_SUCCESS;
}
