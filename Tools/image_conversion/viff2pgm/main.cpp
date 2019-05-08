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
#include <string.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"
#include "Image.h"

using namespace std;

void PrintUsage()
{
  printf("Conversion of an image from VIFF to PGM format.\n\n");
  printf("Usage: viff2pgm -i <viff file> -o <pgm file>\n");
  printf("                [-min <value> (value to be mapped to 0 for float image)]\n");
  printf("                [-max <value> (value to be mapped to 255 for float image)]\n");
}
  
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  Image image;
  FILE *pgm;
  unsigned char *pixel;
  float *fpixel, min_gv, max_gv;

// Usage

  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
// Check on required input and output files

  if (!args->Contains("-i")) {
    printf("Error: no input viff file specified with -i <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: no output pgm file specified with -o <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Read viff file
  if (!image.Read(args->String("-i"))) {
    printf("Error reading VIFF image from %s\n", args->String("-i"));
    exit(0);
  }
   
  // Check if it is a grey scale image
  if (image.DataType() != VFF_TYP_1_BYTE &&
      image.DataType() != VFF_TYP_FLOAT) {
    printf("Error: Image type is not VFF_TYP_1_BYTE or VFF_TYP_FLOAT\n");
    exit(0);
  }
  if (image.NumBands() != 1) {
    printf("Warning: Only the first of the %d bands will be used\n", image.NumBands());
  }

  // Convert float to unsigned char
  if (image.DataType() == VFF_TYP_FLOAT) {
    min_gv = args->Double("-min", 0.0);
    max_gv = args->Double("-max", 255.0);
    for (fpixel=image.float_begin(), pixel=image.begin();
         fpixel!=image.float_end(); fpixel++, pixel++)
      *pixel = (unsigned char) (255.0 * (*fpixel - min_gv) / (max_gv - min_gv));
  }

  // Open pgm file
  pgm = fopen(args->String("-o"), "w");
  if (pgm == NULL) {
    printf("Error opening PGM file %s\n", args->String("-o"));
    exit(0);
  }
  
  // Write pgm file
  fprintf(pgm, "P5\n");
  fprintf(pgm, "%d %d\n", image.NumColumns(), image.NumRows());
  fprintf(pgm, "255\n");
  fwrite(image.begin(), sizeof(char), image.NumRows() * image.NumColumns(), pgm);
  fclose(pgm);
  exit(0);
}
