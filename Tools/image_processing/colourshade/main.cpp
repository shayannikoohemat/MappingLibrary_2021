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
|  Routine Name: main() - Produce a coloured shaded relief image
|
|       Purpose: main program for colourshade
|
|         Input:
|		char *clui_info->ic_file; {Colour image}
|		int   clui_info->ic_flag; {TRUE if -ic specified}
|
|		char *clui_info->is_file; {Shaded relief image}
|		int   clui_info->is_flag; {TRUE if -is specified}
|
|		char *clui_info->o_file; {Resulting output data object}
|		int   clui_info->o_flag; {TRUE if -o specified}
|;2
|		int clui_info->c_int; {Reduction factor for colour levesl}
|		int clui_info->c_flag; {TRUE if -c specified}
|
|        Output:
|       Returns:
|
|    Written By: 
|          Date: Dec 03, 2000
| Modifications:
|
------------------------------------------------------------*/

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include "InlineArguments.h"
#include "Image.h"

using namespace std;

extern "C" void ColourShade(xvimage *, xvimage *, int, xvimage **);

void PrintUsage()
{
  printf("Combination of a colour image and a shaded image into a shaded colour image.\n\n");
  printf("Usage: colourshade -ic <colour image>\n");
  printf("                   -is <shaded image>\n");
  printf("                   -o <output image>\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  Image colour, shade, result;
  xvimage *combined;  

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input and output files
  if (!args->Contains("-ic")) {
    printf("Error: no colour image specified with -ic <image file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-is")) {
    printf("Error: no shaded image specified with -is <image file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: no output image specified with -o <image file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Read the input images
  if (!colour.Read(args->String("-ic"))) {
    printf("Error reading colour image %s\n", args->String("-ic"));
    return EXIT_SUCCESS;
  }
  if (!shade.Read(args->String("-is"))) {
    printf("Error reading shaded image %s\n", args->String("-is"));
    return EXIT_SUCCESS;
  }

  // Combine the two images
  ColourShade(colour.GetImage(), shade.GetImage(), 1, &combined);
              
  // Write the output
  result.SetImage(combined);
  result.Write(args->String("-o"));
  
  return EXIT_SUCCESS;
}
