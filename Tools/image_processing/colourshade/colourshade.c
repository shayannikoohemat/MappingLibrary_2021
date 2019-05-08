
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

void ColourShade(xvimage *colour, xvimage *shaded, int colour_reduction, 
                 xvimage **result_ptr)
{
  xvimage       *result;
  xvimage *createimage(unsigned long, unsigned long, unsigned long,
                       unsigned long, unsigned long, const char *,
                       unsigned long, unsigned long, unsigned long,
                       unsigned long, unsigned long, unsigned long);
  unsigned char *colours, *map, *pix1, *pix2;
  int           icol, ir, ic, wrong_colour_image;

/* Check if the two input images have the same size and the correct colour maps.
 */

  if (colour->col_size != shaded->col_size ||
      colour->row_size != shaded->row_size) {
    fprintf(stderr,
            "Error: Colour and shaded image are not of the same size!\n");
    exit(0);
  }
  wrong_colour_image = 0;
  if (colour->map_row_size == 3 && colour->map_col_size == 256)
    wrong_colour_image = (colour->num_data_bands != 1);
  else if (colour->num_data_bands != 3) wrong_colour_image = 1;
  if (wrong_colour_image) {
    fprintf(stderr, "Error: Colour image should have 3 bands or 1 band with a map of 3 x 256 entries.\n");
    exit(0);
  }

/* Create a template output image */

  result = createimage(1, 1, VFF_TYP_1_BYTE, 1, 1,
                       "Created by ColourShade", 0, 0, VFF_MS_NONE,
                       VFF_MAPTYP_NONE, VFF_LOC_IMPLICIT, 0);
  if (!result) {
    fprintf(stderr, "Error allocating memory for output image.\n");
    exit(0);
  }
  *result_ptr = result;
  result->col_size = colour->col_size;
  result->row_size = colour->row_size;

/* Map the colour data if this is not done yet */

  if (colour->num_data_bands == 3)
    colours = (unsigned char *) colour->imagedata;
  else {
    map = (unsigned char *) colour->maps;
    colours = (unsigned char *) malloc(colour->col_size * colour->row_size * 3);
    if (!colours) {
      fprintf(stderr, "Error allocating memory for mapped image.\n");
      exit(0);
    }
    for (icol=0, pix2=colours; icol<3; icol++) {
      for (ir=0, pix1=(unsigned char *) colour->imagedata;
           ir<colour->col_size;
           ir++) {
        for (ic=0;
             ic<colour->row_size;
             ic++, pix2++, pix1++) {
          *pix2 = map[icol*256 + *pix1];
        }
      }
    }
  }

/* Multiply the colour values with the shades */

  for (icol=0, pix1=colours; icol<3; icol++)
    for (ir=0, pix2=(unsigned char *) shaded->imagedata;
         ir<colour->col_size;
         ir++)
      for (ic=0; ic<colour->row_size; ic++, pix1++, pix2++)
        *pix1 = ((int) *pix1) * ((int) *pix2) / 255;
 
/* Store the resulting colours */

  result->col_size = colour->col_size;
  result->row_size = colour->row_size;
  result->num_data_bands = 3;
  result->color_space_model = VFF_CM_genericRGB;
  result->imagedata = (char *) colours;

/* Create colour map of the result image and store it */
/*
  map = (unsigned char *) malloc(3 * 256);
  for (icol=0, pix1=map; icol<3; icol++)
    for (ic=0; ic<256; ic++, pix1++)
      *pix1 = (ic / colour_reduction) * colour_reduction;

  result->map_row_size = 3;
  result->map_col_size = 256;
  result->map_scheme = VFF_MS_ONEPERBAND;
  result->map_storage_type = VFF_MAPTYP_1_BYTE;
  result->map_enable = VFF_MAP_OPTIONAL;
  result->maps = (char *) map;
*/
}
