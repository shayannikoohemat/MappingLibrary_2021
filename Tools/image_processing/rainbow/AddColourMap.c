
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
#include <stdlib.h>
#include "viff.h"
#include "Database.h"

void AddColourMap(xvimage *image, double violet, double blue,
                  double light_blue, double green, double yellow, double red,
                  Grid3D *grid, int colour_reduction)
{
  unsigned char *map, *pixel;
  int           i, r, g, b, warning;
  
/* Check the colour order */

  warning = 0;
  if (blue < violet) {blue = violet; warning = 1;}
  if (light_blue < blue) {light_blue = blue; warning = 1;}
  if (green < light_blue) {green = light_blue; warning = 1;}
  if (yellow < green) {yellow = green; warning = 1;}
  if (red < yellow) {red = yellow; warning = 1;}
  if (warning) {
    fprintf(stderr, "Warning: colours need to be in rainbow order.\n");
    fprintf(stderr, "The grey values of the colours have been changed to:\n");
    fprintf(stderr, " Violet     : %f\n", violet);
    fprintf(stderr, " Blue       : %f\n", blue);
    fprintf(stderr, " Light blue : %f\n", light_blue);
    fprintf(stderr, " Green      : %f\n", green);
    fprintf(stderr, " Yellow     : %f\n", yellow);
    fprintf(stderr, " Red        : %f\n", red);
  }

/* Convert the data values to pixel values in case of a grid */

  if (grid) {
    violet     = (violet     - grid->data_offset) / grid->data_scale;
    blue       = (blue       - grid->data_offset) / grid->data_scale;
    light_blue = (light_blue - grid->data_offset) / grid->data_scale;
    green      = (green      - grid->data_offset) / grid->data_scale;
    yellow     = (yellow     - grid->data_offset) / grid->data_scale;
    red        = (red        - grid->data_offset) / grid->data_scale;
  }

/* Allocate the map */

  map = (unsigned char *) malloc(3 * 256);

/* Red */

  for (i=0, pixel=map; i<256; i++, pixel++)
    if (i<violet) *pixel = 255;
    else if (i<blue) *pixel = 255 - (i-violet)/(blue-violet)*255;
    else if (i<green) *pixel = 0;
    else if (i<yellow) *pixel = (i-green)/(yellow-green)*255;
    else *pixel = 255;

/* Green */

  for (i=0; i<256; i++, pixel++)
    if (i<blue) *pixel = 0;
    else if (i<light_blue) *pixel = (i-blue)/(light_blue-blue)*255;
    else if (i<yellow) *pixel = 255;
    else if (i<red) *pixel = 255 - (i-yellow)/(red-yellow)*255;
    else *pixel = 0;

/* Blue */

  for (i=0; i<256; i++, pixel++)
    if (i<light_blue) *pixel = 255;
    else if (i<green) *pixel = 255 - (i-light_blue)/(green-light_blue)*255;
    else *pixel = 0;

/* Round all colour map entries to multiples of some power of 2 */

  if (colour_reduction == 2 || colour_reduction == 4 ||
      colour_reduction == 8 || colour_reduction == 16 ||
      colour_reduction == 32 || colour_reduction == 64)
    for (i=0, pixel=map; i<3*256; i++, pixel++)
      *pixel = (*pixel/colour_reduction)*colour_reduction;
  else if (colour_reduction != 1) {
    fprintf(stderr, "Warning: No colour reduction performed.\n");
    fprintf(stderr, "Colour reduction factor should be 1, 2, 4, 8, 16, 32 or 64.\n");
  }

/* Store the map */

  image->map_row_size = 3;
  image->map_col_size = 256;
  image->map_scheme = VFF_MS_ONEPERBAND;
  image->map_storage_type = VFF_MAPTYP_1_BYTE;
  image->map_enable = VFF_MAP_FORCE;
  image->color_space_model = VFF_CM_genericRGB;
  image->maps = (char *) map;
}
