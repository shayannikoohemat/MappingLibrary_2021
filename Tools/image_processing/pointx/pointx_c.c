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
#include <stdlib.h>
#include <stdio.h>
#include "Database.h"
#include "viff.h"

extern int proper_num_images(const char *, xvimage *, unsigned long, int);
extern int proper_map_enable(const char *, xvimage *, unsigned long, int);
extern int freeimage(xvimage *);

#define TRUE 1

/*-----------------------------------------------------------
|
|    Written By: George Vosselman
|          Date: Jul 29, 1999
| Modifications:
|
------------------------------------------------------------*/

int pointx_c(char *i_file, char *o_file, int window_size,
             int thres_s, double thres_q, int locmax_size,
             int r1, int c1, int nr, int nc)
{
  xvimage *image;
  ImgPts  *points;
  int     retval;
  double  q;

  // Functions
  xvimage *readimageheader(), *readimage(), *readimagepart();
  int     foerstner();

  // Read the image header and check the specifications
  image = readimageheader(i_file);
  proper_num_images("pointx", image, 1, TRUE);
  proper_map_enable("pointx", image, VFF_MAP_OPTIONAL, TRUE);
  freeimage(image);

  // Read the grey values of the region of interest

  if (nr == 0) { // Read complete image
    r1 = c1 = 0;
    image = readimage(i_file);
  }
  else // Read region of interest
    image = readimagepart(i_file, r1, c1, nr, nc);

  if (image == NULL) {
    fprintf(stderr, "Error reading (part of) image %s.\n", i_file);
    exit(1);
  }

  // Initialize the points pointer
  points = NULL;

  retval = foerstner(image->imagedata,
                     image->col_size, image->row_size,
                     (double) r1, (double) c1,
                     0, 0, image->col_size, image->row_size,
                     window_size, thres_s, thres_q,
                     locmax_size, &points);

  if (retval && points != NULL) Put_ImgPts(points, o_file);
  else fprintf(stderr, "Point extraction failed.\n");
  freeimage(image);

  return 1;
}
