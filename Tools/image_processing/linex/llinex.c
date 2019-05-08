
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
#include "viff.h"
#include "Database.h"

#define EXTRACT_POINTS 1
#define EXTRACT_EDGES  2

#define DEBUG          0

/****************************************************************
* 
*  Routine Name: llinex - Extraction of straight lines by line growing on a gradient image.
* 
*       Purpose: The routine llinex extracts straight lines by grouping adjacent pixels with
similar gradient directions and fitting a line through these pixels. After
calculating and thresholding gradients, the line growing algorithm selects
the candidate line pixel with the strongest
gradient as a starting pixel. The normal direction of the line through this
pixel is initially defined by the grey value gradient. Like a region growing
algorithm, the line growing algorithm tries to add eight-connected pixels to
the area of already classified pixels. If a pixel that is eight-connected to a
pixel of the current line lies close to this line (e.g. within two pixels
distance) and has a gradient that is about perpendicular to the direction of
the line, it is classified as a line pixel and added to the area that describes
the line. The position and direction of the line are updated by calculating the
first and second moments of the pixels in the line area. This process continues
until no more pixels can be added to the line area. The line growing algorithmthen selecte the next pixel with the strongest gradient that has not been used
before and uses this pixel as the starting pixel for the next line. This
process continues until all candidate line pixels have been processed.

Control parameters that can be specified by the user are the window size for
gradient calculation (with the modified Roberts operator), the threshold for
the selecting candidate line pixels, the minimum required line length and the
maximum width of a line area.

*    Written By: George Vosselman
*          Date: Jul 29, 1999
*      Verified: 
*  Side Effects: 
* Modifications: 
****************************************************************/

extern int writeimage(const char *, xvimage *);
extern int linegrow(int *, int *, int, int, double, double, int, int,
             float, float, ImgLines **);

int llinex(unsigned char *image, int nrows, int ncols, double roff, double coff,
           int r1_roi, int c1_roi, int nrows_roi, int ncols_roi,
           int window_size, int s_thres, float minlength, float maxwidth, 
           ImgLines **lines_ptr)
{
  int           *g_r, *g_c, *g_rc, *g_s, *row_s, *pixel_s, ir, ic, num_alloc,
                ip, num_sel;
  double        sum_r2, sum_c2, sum_rc, det;
  ImgPts        *points;
  ImgPt         *point;

  int           mod_roberts(), fo_normal(), fo_select();
  void          fo_box_filter(), fo_direction();

/* Debug variables */

  xvimage *debug_image, *createimage();

/* Calculate the modified roberts gradients in row and column direction */

  if (DEBUG) printf("Calculating gradients\n");
  g_r = g_c = NULL;
  if (!mod_roberts(image, nrows, ncols, r1_roi, c1_roi, nrows_roi, ncols_roi,
                   &g_r, &g_c))
    return(0);

/* Calculate the gradient squares and cross product */

  if (DEBUG) printf("Calculating squared gradients\n");
  g_rc = NULL;
  if (!fo_normal(g_r, g_c, &g_rc, nrows_roi, ncols_roi)) return(0);

/* Summarize over window size by convolution with a box filter */

  if (DEBUG) printf("Convolution with box filter\n");
  fo_box_filter(g_r, g_c, g_rc, nrows_roi, ncols_roi, window_size-1);
  if (DEBUG) {
    debug_image = createimage(1, 1, VFF_TYP_4_BYTE, 1, 1, "Debug image", 0, 0,
                              VFF_MS_NONE, VFF_MAPTYP_NONE, VFF_LOC_IMPLICIT, 0);
    debug_image->col_size = nrows_roi;
    debug_image->row_size = ncols_roi;
    debug_image->imagedata = (char *) g_r;
    writeimage("gr.xv", debug_image);
  }

/* The eigen values of the normal matrix are the gradient sums in the direction
 * of the highest and lowest contrast respectivily. If this routine is called
 * for edge extraction, we should check the sum of the eigen values. If this
 * sum of a pixel exceeds the user specified threshold, it is returned.
 * Otherwise, the pixel is set to zero.
 */

  if (DEBUG) printf("Selection of edges\n");
  g_s = NULL;
  if (!fo_select(EXTRACT_EDGES, g_r, g_c, g_rc, &g_s, nrows_roi, ncols_roi,
                 window_size-2, s_thres, 1.01, &num_sel)) return(0);
  if (DEBUG) {
    debug_image->imagedata = (char *) g_s;
    writeimage("edges.xv", debug_image);
    printf("%d edge pixels selected\n", num_sel);
  }

/* Calculate the gradient directions of the selected pixels */

  if (DEBUG) printf("Calculation of directions\n");
  fo_direction(g_r, g_c, g_rc, g_s, nrows_roi, ncols_roi, window_size-2);
  if (DEBUG) {
    debug_image->imagedata = (char *) g_rc;
    writeimage("dir.xv", debug_image);
  }

/* Extract the straight lines by growing linear segments */

  if (DEBUG) printf("Line growing\n");
  if (!linegrow(g_s, g_rc, nrows_roi, ncols_roi,
                roff + r1_roi + (window_size-1)/2.0,
                coff + c1_roi + (window_size-1)/2.0,
                window_size-2, num_sel, maxwidth, minlength, lines_ptr)) {
    fprintf(stderr, "Error in routine linegrow.\n");
    return(0);
  }

/* Debug */

/*
  debug_image = createimage(1, 1, VFF_TYP_4_BYTE, 1, 1, "Debug image", 0, 0,
                            VFF_MS_NONE, VFF_MAPTYP_NONE, VFF_LOC_IMPLICIT, 0);
  debug_image->col_size = nrows_roi;
  debug_image->row_size = ncols_roi;
  debug_image->imagedata = (unsigned char *) g_r;
  writeimage("r1.xv", debug_image);
  debug_image->imagedata = (unsigned char *) g_c;
  writeimage("c1.xv", debug_image);
  debug_image->imagedata = (unsigned char *) g_rc;
  writeimage("rc1.xv", debug_image);
*/

  free(g_r);  free(g_c);  free(g_rc);  free(g_s);
  return(1);
}
/* -library_code_end */
