
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
#include "InlineArguments.h"
#include "Image.h"
#include "Database4Cpp.h"
#include "ImageLines.h"
#include "ImagePoints.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: linex -i <input grey value image>\n");
  printf("             -win <window size, default: 3>\n");
  printf("             -t <gradient strength threshold>\n");
  printf("             -minl <minimum line length, default: 10 pixels>\n");
  printf("             -maxw <maximum line region width, default: 3 pixels>\n");
  printf("             -ol <extracted lines>\n");  
  printf("             -op <extracted end points of lines\n");
  printf("             -ot <extracted line topology\n");
}

extern "C" int llinex(unsigned char *, int, int, double, double,
                      int, int, int, int,  int, int, float, float, 
                      ImgLines **);
                        
int main(int argc, char *argv[])
{
  Image           image;
  ImgLines        *lines_ptr=NULL;
  InlineArguments *args = new InlineArguments(argc, argv);
  
  // Variables for conversion to points and topology
  ImageLines      image_lines;
  ImagePoints     image_points;
  ImagePoints::iterator image_point;
  LineTopologies  line_tops;
  ObjectPoints    object_points;
  ObjectPoint     object_point;
    
  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check required arguments
  if (!args->Contains("-i")) {
    printf("Error: No input image specified with -i\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-ol") &&
      !(args->Contains("-op") && args->Contains("-ot")) &&
      !(args->Contains("-oo") && args->Contains("-ot"))) {
    printf("Error: No output file specified with -ol or combination of -op and -ot\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-t")) {
    printf("Error: No gradient strength threshold specified with -t\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Read the image
  if (!image.Read(args->String("-i"))) {
    printf("Error reading input image %s\n", args->String("-i"));
    return EXIT_SUCCESS;
  }
  
  // Call C interface
  llinex(image.begin(), image.NumRows(), image.NumColumns(), 0.0, 0.0,
         0, 0, image.NumRows(), image.NumColumns(),
         args->Integer("-win", 3), args->Integer("-t", 1000),
         (float) args->Double("-minl", 10.0), (float) args->Double("-maxw", 3.0),
         &lines_ptr);
         
  // Output image lines
  if (lines_ptr != NULL) {
    if (args->Contains("-ol")) Put_ImgLines(lines_ptr, args->String("-ol"));

    // Conversion to image points and topology
    if (args->Contains("-ot")) {
      image_lines.C2Cpp(lines_ptr);
      image_lines.Convert2LineTops(&image_points, &line_tops); 
      if (args->Contains("-op")) image_points.Write(args->String("-op"));
      line_tops.Write(args->String("-ot"));
      if (args->Contains("-oo")) {
        for (image_point=image_points.begin(); image_point!=image_points.end(); image_point++) {
          object_point.Number() = image_point->Number();
          object_point.X() = image_point->Column();
          object_point.Y() = -image_point->Row();
          object_point.Z() = 0.0;
          object_points.push_back(object_point);
        }	
        object_points.Write(args->String("-oo"));
      }
    }
  }

  return EXIT_SUCCESS;
}
