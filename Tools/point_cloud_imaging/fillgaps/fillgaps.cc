
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


/*-----------------------------------------------------------
|
|  Routine Name: main() - Fill gaps in an image
|
|       Purpose: main program for fillgaps
|
|         Input:
|		char *clui_info->i_file; {input image}
|		int   clui_info->i_flag; {TRUE if -i specified}
|
|		int clui_info->md_int; {maximum distance to neighbour pixel}
|		int clui_info->md_flag; {TRUE if -md specified}
|
|		char *clui_info->o_file; {output image}
|		int   clui_info->o_flag; {TRUE if -o specified}
|
|        Output:
|       Returns:
|
|    Written By: George Vosselman
|          Date: May 04, 1999
| Modifications:
|
------------------------------------------------------------*/

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include "InlineArguments.h"
#include "Image.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: fillgaps -i <input VIFF image> -o <output VIFF image>\n");
  printf("                [-gap <maximum gap size to close (def: 1 pixel)]\n");
  printf("                [-e (extrapolate)]\n");
}

extern "C" void Fill_Gaps_In_Raster(unsigned char *, unsigned char *, int, int,
                                    int, int);
extern "C" void Fill_Gaps_In_Raster_Float(float *, float *, int, int, int, int);

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  Image gaps, filled; 
  
  // Usage
  if (args->Contains("-usage")) {
    PrintUsage(); exit(0);
  }
        
  // Check arguments
  if (!args->Contains("-i")) {
    printf("Error: No input image specified with -i\n");
    PrintUsage(); exit(0);
  }
  if (!args->Contains("-o")) {
    printf("Error: No output image specified with -o\n");
    PrintUsage(); exit(0);
  }

  // Read image
  if (!gaps.Read(args->String("-i"))) {
    printf("Error reading input image %s\n", args->String("-i"));
    exit(0);
  }
  
  // Unsigned char images
  if (gaps.DataType() == VFF_TYP_1_BYTE) {
    // Allocate memory for the output image
    if (filled.NewImage(gaps.NumRows(), gaps.NumColumns(), VFF_TYP_1_BYTE) == NULL) {
      printf("Error allocating memory for output image\n");
      exit(0);
    }
    Fill_Gaps_In_Raster(gaps.begin(), filled.begin(),
                        gaps.NumRows(), gaps.NumColumns(),
                        args->Integer("-gap", 1),
						args->Contains("-e"));
  }
  
  // Float image
  else if (gaps.DataType() == VFF_TYP_FLOAT) {
    // Allocate memory for the output image
    if (filled.NewImage(gaps.NumRows(), gaps.NumColumns(), VFF_TYP_FLOAT) == NULL) {
      printf("Error allocating memory for output image\n");
      exit(0);
    }
    Fill_Gaps_In_Raster_Float(gaps.float_begin(), filled.float_begin(),
                              gaps.NumRows(), gaps.NumColumns(),
                              args->Integer("-gap", 1),
							  args->Contains("-e"));
  }
  
  // Unknown type
  else {
  	printf("Error: fillgaps is only implemented for unsigned char and float images\n");
  	exit(0);
  }

  printf("Writing filled image to %s\n", args->String("-o"));
  filled.Write(args->String("-o"));
  exit(0);
}
