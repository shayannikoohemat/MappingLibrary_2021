
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
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: indoorsimulation -m <model number, see below>\n");
  printf("                        -p <simulated point cloud\n");
  printf("                        [-o0 <simulated observations sensor 0>]\n");
  printf("                        [-o1 <simulated observations sensor 1>]\n");
  printf("                        [-o2 <simulated observations sensor 2>]\n");
  printf("                        [-sdir <output directory for scan files>]\n");
  printf("                        [-of] (output scans in frame coordinate system)\n");
  printf("                        [-r <sensor rotation in degrees, default: 20>]\n");
  printf("                        [-a <opening angle, default: 270.0 degree>]\n");
  printf("                        [-ai <angle increment, default: 0.25 degree>]\n");
  printf("                        [-sf <number of scans per second, default: 40>]\n");
  printf("                        [-ya <Y-amplitude, default: 0.0 (m)>]\n");
  printf("                        [-kd <kappa damping, default: 0.3>]\n");
  printf("                        [-par <text file with pose parameters>]\n");
  printf("                        [-std <standard deviation of range noise, default: 0.0>]\n");
  printf("                        [-mp <corridor map points>]\n");
  printf("                        [-mt <corridor map topology>]\n");
  printf("                        [-pp <path points>]\n");
  printf("                        [-pt <path topology>]\n");
  printf("                        [-sw] (single wall, double ceiling)\n");
  printf("\nAvailable models:\n");
  printf("  0 - Straight corridor of 20 m\n");
  printf("  1 - Closed loop corridor of 20 x 14 m, one loop\n");
  printf("  2 - Closed loop corridor of 20 x 14 m, three loops\n");
  printf("  3 - User specified corridor map and path\n");
}
 
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void indoorsimulation_cpp(int, char *, char *, char *, char *, char *,
                            double, double, double, int, double, double, char *,
							double, char *, char *, char *, char *, bool, bool);

  if (args->Contains("-usage")) {
  	PrintUsage();
  	exit(0);
  }  
  if (!args->Contains("-m")) {
    printf("Error: -m is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-p")) {
    printf("Error: -p is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  // The default values are the specs of Hokuyo UTM-30LX scanner, 
  // 40 scans per second, angle increment of 0.25 degree, 
  // opening angle 270 degree

  indoorsimulation_cpp(args->Integer("-m", 0), args->String("-p"),
                       args->String("-o0"),
                       args->String("-o1"), args->String("-o2"),
					   args->String("-sdir"), 
					   args->Double("-r", 20.0), args->Double("-a", 270.0),
					   args->Double("-ai", 0.25), args->Integer("-sf", 40),
					   args->Double("-ya", 0.0), args->Double("-kd", 0.3),
					   args->String("-par"), args->Double("-std", 0.0),
					   args->String("-mp"), args->String("-mt"),
					   args->String("-pp"), args->String("-pt"),
					   !(args->Contains("-sw")),
					   args->Contains("-of"));

  return EXIT_SUCCESS;
}

/* Code for creating a colour composite from three separate 1 band images

#include "Image.h"

  Image red, green, blue, colour;
  unsigned char *colour_pix, *band_pix;
  int i, numpix;
  
  red.Read(args->String("-r")); 
  green.Read(args->String("-g")); 
  blue.Read(args->String("-b")); 
  colour.NewImage(red.NumRows(), red.NumColumns(), VFF_TYP_1_BYTE, 3);
  numpix = red.NumRows() * red.NumColumns();
  for (i=0, colour_pix=colour.Pixel(0,0), band_pix=red.Pixel(0,0);
       i<numpix; i++, colour_pix++, band_pix++)
    *colour_pix = *band_pix;
  for (i=0, band_pix=green.Pixel(0,0); i<numpix; i++, colour_pix++, band_pix++)
    *colour_pix = *band_pix;
  for (i=0, band_pix=blue.Pixel(0,0);  i<numpix; i++, colour_pix++, band_pix++)
    *colour_pix = *band_pix;
  colour.GetImage()->color_space_model = VFF_CM_ntscRGB;
  colour.Write(args->String("-o"));
  exit(0);
*/
