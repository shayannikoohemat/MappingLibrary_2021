
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

#include "viff.h"
#include "Database.h"

extern int writeimage(const char *, xvimage *);

void rainbow(const char *infile, const char *outfile,
             double violet, double blue, double light_blue,
             double green, double yellow, double red,
             int c, const char *gridfile, int height_dif)
{
  xvimage       *image, *createimage(), *readimage(), *colour_image;
  unsigned char *pixel, *grey, *redpix, *greenpix, *bluepix;
  int           ir, ic, height_correction, grey_cor;
  double        slope, height_correctionf;
  Grid3D        *grid;
  float         *height, low;
  void          AddColourMap();

/* Read input image or create a grey value wedge image */

  if (infile) {
    image = readimage(infile);
    slope = (double) height_dif / (double) image->row_size;
    if (image->data_storage_type == VFF_TYP_FLOAT) {
      // Create colour output image
      colour_image = createimage(image->col_size, image->row_size, VFF_TYP_1_BYTE, 1, 3, "Rainbow height image",
                          0, 0,
                          VFF_MS_NONE, VFF_MAPTYP_NONE, VFF_LOC_IMPLICIT, 0);
      colour_image->color_space_model = VFF_CM_genericRGB;
      // Determine lowest height
      low = 1e6;
      for (ir=0, height=(float *) image->imagedata;
           ir<image->col_size; ir++)
        for (ic=0; ic<image->row_size; ic++, height++)
          if (*height < low) low = *height;
      // Set colours
      for (ir=0, height=(float *) image->imagedata,
           redpix = (unsigned char *) colour_image->imagedata,
           greenpix = redpix + (image->col_size * image->row_size),
           bluepix = greenpix + (image->col_size * image->row_size);
           ir<image->col_size; ir++) {
        for (ic=0; ic<image->row_size; ic++, height++, redpix++, greenpix++, bluepix++) {
          height_correctionf = slope * ic;
          if (*height != low) {
            if (*height - height_correctionf < low)
              *height = low + 1.0;
            else
              *height = *height - height_correctionf;
          }
          /* Red */
          if (*height <= violet) *redpix = 255;
          else if (*height<blue) *redpix = 255 - (*height-violet)/(blue-violet)*255;
          else if (*height<green) *redpix = 0;
          else if (*height<yellow) *redpix = (*height-green)/(yellow-green)*255;
          else *redpix = 255;

          /* Green */
          if (*height<blue) *greenpix = 0;
          else if (*height<light_blue) *greenpix = (*height-blue)/(light_blue-blue)*255;
          else if (*height<yellow) *greenpix = 255;
          else if (*height<red) *greenpix = 255 - (*height-yellow)/(red-yellow)*255;
          else *greenpix = 0;

          /* Blue */
          if (*height<light_blue) *bluepix = 255;
          else if (*height<green) *bluepix = 255 - (*height-light_blue)/(green-light_blue)*255;
          else *bluepix = 0;
        }
      }
    }
    else {
      slope = (double) height_dif / (double) image->row_size;
      for (ir=0, grey=(unsigned char *) image->imagedata;
           ir<image->col_size; ir++) {
        for (ic=0; ic<image->row_size; ic++, grey++) {
          height_correction = (int) (slope * ic);
          grey_cor = (int) (*grey) - height_correction;
          if (*grey != 0) {
            if (grey_cor < 1) *grey = 1;
            else if (grey_cor > 255) *grey = 255;
            else *grey = grey_cor;
          }
        }
      }
    }
  }    
  else {
    image = createimage(256, 256, VFF_TYP_1_BYTE, 1, 1, "Rainbow test image",
                        0, 0,
                        VFF_MS_NONE, VFF_MAPTYP_NONE, VFF_LOC_IMPLICIT, 0);
    for (ir=0, pixel = (unsigned char *) image->imagedata; ir<256; ir++)
      for (ic=0; ic<256; ic++, pixel++) *pixel = ir;
  }

/* No grid if not specified */

  if (!gridfile) grid = NULL;
  else grid = Get_Grid3D(gridfile);

  if (image->data_storage_type != VFF_TYP_FLOAT) {
    AddColourMap(image, violet, blue, light_blue, green, yellow, red, grid, c);
    writeimage(outfile, image);
  }
  else
    writeimage(outfile, colour_image);
    

}
