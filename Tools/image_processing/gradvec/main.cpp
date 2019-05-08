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
#include "ImagePoints.h"
#include "LineTopologies.h"

using namespace std;

void PrintUsage()
{
  printf("Generate gradient vector plots of an image.\n\n");
  printf("Usage: convolve -i <input image>\n");
  printf("                -op <output vector points>\n");
  printf("                -ot <output vector topology>\n");
}
  
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  Image image, dir, strength;
  float *fpixel, dr, dc, s, maxs=0.0, alpha;
  int r, c, numarrows, numpts, n, i;
  ImagePoints points;
  ImagePoint point;
  LineTopologies arrows;
  LineTopology arrow;
  FILE *fd;

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
  if (!args->Contains("-op")) {
    printf("Error: no output vector point file specified with -op <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-ot")) {
    printf("Error: no output vector topology file specified with -ot <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Read the input image
  if (!image.Read(args->String("-i"))) {
    printf("Error reading input image from %s\n", args->String("-i"));
    exit(0);
  }

  // Allocate gradient direction and strength images
  dir.NewImage(image.NumRows(), image.NumColumns(), VFF_TYP_FLOAT, 1);
  strength.NewImage(image.NumRows(), image.NumColumns(), VFF_TYP_FLOAT, 1);

  // Calculate gradient direction and strength for a float image
  if (image.DataType() == VFF_TYP_FLOAT) {
    for (r=1; r<image.NumRows()-1; r++) {
      for (c=1; c<image.NumColumns()-1; c++) {
        dr = *(image.FloatPixel(r+1, c)) - *(image.FloatPixel(r-1, c));
        dc = *(image.FloatPixel(r, c+1)) - *(image.FloatPixel(r, c-1));
        *(dir.FloatPixel(r, c)) = atan2(dr, dc);
        s = sqrt(dr*dr + dc*dc);
        if (s > maxs) maxs = s;
        *(strength.FloatPixel(r, c)) = s;        
      }
    }
  }
  // The same for an unsigned char image
  else {
    for (r=1; r<image.NumRows()-1; r++) {
      for (c=1; c<image.NumColumns()-1; c++) {
        dr = *(image.Pixel(r+1, c)) - *(image.Pixel(r-1, c));
        dc = *(image.Pixel(r, c+1)) - *(image.Pixel(r, c-1));
        *(dir.FloatPixel(r, c)) = atan2(dr, dc);
        s = sqrt(dr*dr + dc*dc);
        if (s > maxs) maxs = s;
        *(strength.FloatPixel(r, c)) = s;
      }
    }
  }
  
  // Write image
  dir.Write("directions.xv");
  strength.Write("strength.xv");

  fd = fopen("directions.txt", "w");
  
  // Generate arrows
  numpts = numarrows = 0;
  for (r=1; r<image.NumRows()-1; r++) {
    for (c=1; c<image.NumColumns()-1; c++) {
      s = *(strength.FloatPixel(r, c));
      alpha = *(dir.FloatPixel(r, c));
      // Topology
      arrow.Erase();
      arrow.push_back(PointNumber(numpts+1));
      arrow.push_back(PointNumber(numpts+2));
      arrows.push_back(arrow);
      if (s > 0.0) {
        arrow.Erase();
        arrow.push_back(PointNumber(numpts+3));
        arrow.push_back(PointNumber(numpts+2));
        arrow.push_back(PointNumber(numpts+4));
        arrows.push_back(arrow);
      }
      // Points
      s = *(strength.FloatPixel(r, c));
      alpha = *(dir.FloatPixel(r, c));
      dr = 0.35 * (s / maxs) * sin(alpha);
      dc = 0.35 * (s / maxs) * cos(alpha);
      numpts++;
      point.Number() = numpts;
      point.Row() = r + dr;
      point.Column() = c + dc;
      points.push_back(point);
      numpts++;
      point.Number() = numpts;
      point.Row() = r - dr;
      point.Column() = c - dc;
      points.push_back(point);
      if (s > 0.0) {
        numpts++;
        point.Number() = numpts;
        point.Row() = r - dr + 0.1 * sin(alpha+0.5);
        point.Column() = c - dc + 0.1 * cos(alpha+0.5);
        points.push_back(point);
        numpts++;
        point.Number() = numpts;
        point.Row() = r - dr + 0.1 * sin(alpha-0.5);
        point.Column() = c - dc + 0.1 * cos(alpha-0.5);
        points.push_back(point);
      }
      // Weight of direction for histogram
      dr = (float) r - 17.0;
      dc = (float) c - 17.0;
//      dr = (float) r - (float) image.NumRows()/2;
//      dc = (float) c - (float) image.NumColumns()/2;
      dr = dr*dr + dc*dc;
      if (dr < 49) {
        s = 10.0 * s / maxs * (7 - sqrt(dr+0.01));
        n = (int) (s+0.5);
        for (i=0; i<n; i++) fprintf(fd, "%.1f\n", alpha * 45.0 / atan(1.0));
      }
    }
  }
  fclose(fd);
  
  // Write vectors
  points.Write(args->String("-op"));
  arrows.Write(args->String("-ot"), false);
  exit(0);
}
