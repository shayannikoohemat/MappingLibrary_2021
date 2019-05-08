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
  printf("Conversion of an image from PGM to VIFF format.\n\n");
  printf("Usage: pgm2viff -i <pgm file> -o <viff file>\n");
}
  
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  Image image;
  FILE *pgm;
  char line[20];
  int num_rows, num_columns, grey_levels;

// Usage

  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
// Check on required input and output files

  if (!args->Contains("-i")) {
    printf("Error: no input pgm file specified with -i <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: no output viff file specified with -o <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Open pgm file
  pgm = fopen(args->String("-i"), "rb");
  if (pgm == NULL) {
    printf("Error opening PGM file %s\n", args->String("-i"));
    exit(0);
  }
  
  // Read pgm file header
  fgets(line, 20, pgm);
  if (strncmp(line, "P5", 2) != 0) {
    printf("Warning: First line of PGM file shown below does not equal P5\n");
    printf("%s\n", line);
  }
  fgets(line, 20, pgm);
  sscanf(line, "%d%d", &num_columns, &num_rows);
  fgets(line, 20, pgm);
  sscanf(line, "%d", &grey_levels);
  if (grey_levels != 255)
    printf("Warning: Number of grey levels is %d, expected 255\n", grey_levels);

  // Set up the image
  image.NewImage(num_rows, num_columns, VFF_TYP_1_BYTE, 1);
  
  // Read the grey values
  fread(image.begin(), sizeof(char), num_rows * num_columns, pgm);
  
  // Write viff file
  if (!image.Write(args->String("-o")))
    printf("Error writing VIFF image to %s\n", args->String("-o"));
  exit(0);
}
