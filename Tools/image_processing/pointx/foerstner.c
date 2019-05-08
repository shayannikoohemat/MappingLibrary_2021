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
#include <math.h>
#include "viff.h"
#include "Database.h"
#include "stdmath.h"

#define EXTRACT_POINTS 1
#define EXTRACT_EDGES  2

int foerstner(image, nrows, ncols, roff, coff,
              r1_roi, c1_roi, nrows_roi, ncols_roi,
              window_size, s_thres, q_thres, locmax_size, points_ptr)
  unsigned char *image;
  int           nrows, ncols, r1_roi, c1_roi, nrows_roi, ncols_roi,
                window_size, s_thres, locmax_size;
  double        roff, coff, q_thres;
  ImgPts        **points_ptr;
{
  int           *g_r, *g_c, *g_rc, *g_s, *row_s, *pixel_s, ir, ic, num_alloc,
                ip, num_sel;
  double        sum_r2, sum_c2, sum_rc, det;
  ImgPts        *points;
  ImgPt         *point;

  int           mod_roberts(), fo_normal(), fo_select();
  void          fo_box_filter(), fo_undo_box_filter(), fo_local_max(),
                fo_subpixel();

/* Debug */

  xvimage       *debug_image, *createimage();

/* Calculate the modified roberts gradients in row and column direction */

  g_r = g_c = NULL;
  if (!mod_roberts(image, nrows, ncols, r1_roi, c1_roi, nrows_roi, ncols_roi,
                   &g_r, &g_c))
    return(0);

/* Calculate the gradient squares and cross product */

  g_rc = NULL;
  if (!fo_normal(g_r, g_c, &g_rc, nrows_roi, ncols_roi)) return(0);

/* Summarize over window size by convolution with a box filter */

  fo_box_filter(g_r, g_c, g_rc, nrows_roi, ncols_roi, window_size-1);

/* The eigen values of the normal matrix are the gradient sums in the direction
 * of the highest and lowest contrast respectivily. If this routine is called
 * for point extraction, we should check the smallest eigen value. If this
 * value of a pixel exceeds the user specified threshold, the trace of the
 * normal matrix is returned. Otherwise the pixel is set to zero.
 */

  g_s = NULL;
  if (!fo_select(EXTRACT_POINTS, g_r, g_c, g_rc, &g_s, nrows_roi, ncols_roi,
                 window_size-2, s_thres, q_thres, &num_sel)) return(0);

/* Selection of local maxima */

  fo_local_max(g_s, nrows_roi, ncols_roi, window_size-2, locmax_size, &num_sel);

/* Store the selected points in the structure to be returned. Store the inverse
 * of the normal matrix in the variance/covariance variables.
 */

  points = (ImgPts *) malloc(sizeof(ImgPts));
  *points_ptr = points;
  points->num_pts = 0;
  num_alloc = 10;
  points->pts = (ImgPt *) malloc(num_alloc * sizeof(ImgPt));
  for (ir=0, row_s=g_s; ir<nrows_roi-window_size+1; ir++, row_s+=ncols_roi) {
    for (ic=0, pixel_s=row_s; ic<ncols_roi-window_size+1; ic++, pixel_s++) {
      if (*pixel_s > 0) {
        (points->num_pts)++;
        if (points->num_pts > num_alloc) {
          num_alloc += 10;
          points->pts = (ImgPt *) realloc(points->pts,
                                          num_alloc * sizeof(ImgPt));
        }
        points->pts[points->num_pts-1].r = (double) ir;
        points->pts[points->num_pts-1].c = (double) ic;
        sum_r2 = (double) *(g_r + ir*ncols_roi + ic);
        sum_c2 = (double) *(g_c + ir*ncols_roi + ic);
        sum_rc = (double) *(g_rc + ir*ncols_roi + ic);
        det    = sum_r2 * sum_c2 - sum_rc * sum_rc;
        points->pts[points->num_pts-1].v_r = sum_c2 / det;
        points->pts[points->num_pts-1].v_c = sum_r2 / det;
        points->pts[points->num_pts-1].cv_rc = -sum_rc / det;
        points->pts[points->num_pts-1].num = points->num_pts;
      }
    }
  }

/* Restore the originale gradients by deconvolving with the box filter */

  fo_undo_box_filter(g_r, g_c, g_rc, nrows_roi, ncols_roi, window_size-1);

/* Get the subpixel location of the selected points */

  fo_subpixel(g_r, g_c, g_rc, nrows_roi, ncols_roi, window_size-1, points);

/* Adjust the point coordinates for the offset of the image and the offset
 * of the region of interest.
 */

  for (ip=0, point=points->pts; ip<points->num_pts; ip++, point++) {
    point->r += roff + r1_roi;
    point->c += coff + c1_roi;
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

int mod_roberts(image, nrows, ncols, r1_roi, c1_roi, nrows_roi, ncols_roi,
                row_gr_ptr, col_gr_ptr)
  unsigned char *image;
  int           nrows, ncols, r1_roi, c1_roi, nrows_roi, ncols_roi,
                **row_gr_ptr, **col_gr_ptr;
{
  unsigned char *row_ptr, *pixel_ptr;
  int           ir, ic, *row_r, *pixel_r, *row_c, *pixel_c;

/* Allocate memory for gradient images */

  if (*row_gr_ptr == NULL)
    *row_gr_ptr = (int *) malloc(nrows_roi * ncols_roi * sizeof(int));
  if (*col_gr_ptr == NULL)
    *col_gr_ptr = (int *) malloc(nrows_roi * ncols_roi * sizeof(int));
  if (*row_gr_ptr == NULL || *col_gr_ptr == NULL) {
    fprintf(stderr, "Error allocating memory for gradient images.\n");
    return(0);
  }

/* Calculate simple row and column gradients */

  for (ir=0, row_ptr=image+r1_roi*ncols, row_r=*row_gr_ptr, row_c=*col_gr_ptr;
       ir<nrows_roi-1;
       ir++, row_ptr+=ncols, row_r+=ncols_roi, row_c+=ncols_roi) {
    for (ic=0, pixel_ptr=row_ptr+c1_roi, pixel_r=row_r, pixel_c=row_c;
         ic<ncols_roi-1;
         ic++, pixel_ptr++, pixel_r++, pixel_c++) {
      *pixel_r = (int) *(pixel_ptr+ncols) - (int) *pixel_ptr;
      *pixel_c = (int) *(pixel_ptr+1) - (int) *pixel_ptr;
    }
  }

/* Separate loops for row gradients in the last column and column gradients in
 * the last row.
 */

  for (ir=0, pixel_ptr=image+r1_roi*ncols+c1_roi+ncols_roi-1,
             pixel_r=*row_gr_ptr+ncols_roi-1;
       ir<nrows_roi-1;
       ir++, pixel_ptr+=ncols, pixel_r+=ncols_roi) {
    *pixel_r = (int) *(pixel_ptr+ncols) - (int) *pixel_ptr;
  }
  for (ic=0, pixel_ptr=image+(r1_roi+nrows_roi-1)*ncols+c1_roi,
             pixel_c=*col_gr_ptr+(nrows_roi-1)*ncols_roi;
       ic<ncols_roi-1;
       ic++, pixel_ptr++, pixel_c++) {
    *pixel_c = (int) *(pixel_ptr+1) - (int) *pixel_ptr;
  }

/* Integrate over two adjacent row and column gradients */

  for (ir=0, row_r=*row_gr_ptr, row_c=*col_gr_ptr;
       ir<nrows_roi-1;
       ir++, row_r+=ncols_roi, row_c+=ncols_roi) {
    for (ic=0, pixel_r=row_r, pixel_c=row_c;
         ic<ncols_roi-1;
         ic++, pixel_r++, pixel_c++) {
      *pixel_r += *(pixel_r+1);
      *pixel_c += *(pixel_c+ncols_roi);
    }
  }

/* Done */

  return(1);
}


int fo_normal(g_r, g_c, g_rc, nrows, ncols)
  int *g_r, *g_c, **g_rc, nrows, ncols;
{
  int ir, ic, *pixel_r, *pixel_c, *pixel_rc, *row_r, *row_c, *row_rc;

/* Allocate memory for the cross gradients */

  if (*g_rc == NULL) *g_rc = (int *) malloc(nrows * ncols * sizeof(int));
  if (*g_rc == NULL) {
    fprintf(stderr, "Error allocating memory for cross gradient image.\n");
    return(0);
  }

/* Calculate the cross gradients and square the gradients. Last row and
 * column do not contain modified roberts gradients and are not squared.
 */

  for (ir=0, row_r=g_r, row_c=g_c, row_rc=*g_rc;
       ir<nrows-1;
       ir++, row_r+=ncols, row_c+=ncols, row_rc+=ncols) {
    for (ic=0, pixel_r=row_r, pixel_c=row_c, pixel_rc=row_rc;
	 ic<ncols-1;
	 ic++, pixel_r++, pixel_c++, pixel_rc++) {
      *pixel_rc = *pixel_r * *pixel_c;
      *pixel_r *= *pixel_r;
      *pixel_c *= *pixel_c;
    }
  }

/* Done */

  return(1);
}


void fo_box_filter(g_r, g_c, g_rc, nrows, ncols, window_size)
  int *g_r, *g_c, *g_rc, nrows, ncols, window_size;
{
  int ir, ic, sum_r, sum_c, sum_rc, *row_r, *row_c, *row_rc,
      *pixel_r, *pixel_c, *pixel_rc, save_r, save_c, save_rc,
      *col_r, *col_c, *col_rc;

/* First summarize in the rows */

  for (ir=0, row_r=g_r, row_c=g_c, row_rc=g_rc;
       ir<nrows-1;
       ir++, row_r+=ncols, row_c+=ncols, row_rc+=ncols) {

    /* Calculate the sum of the first "window_size-1" pixels of the row */

    sum_r = sum_c = sum_rc = 0;
    for (ic=0, pixel_r=row_r, pixel_c=row_c, pixel_rc=row_rc;
         ic<window_size-1;
         ic++, pixel_r++, pixel_c++, pixel_rc++) {
      sum_r  += *pixel_r;
      sum_c  += *pixel_c;
      sum_rc += *pixel_rc;
    }

    /* Recursively calculate the sums */

    save_r = save_c = save_rc = 0;
    for (ic=0, pixel_r=row_r, pixel_c=row_c, pixel_rc=row_rc;
         ic<ncols-window_size;
         ic++, pixel_r++, pixel_c++, pixel_rc++) {
      sum_r  += *(pixel_r +window_size-1) - save_r;
      sum_c  += *(pixel_c +window_size-1) - save_c;
      sum_rc += *(pixel_rc+window_size-1) - save_rc;
      save_r  = *pixel_r;
      save_c  = *pixel_c;
      save_rc = *pixel_rc;
      *pixel_r  = sum_r;
      *pixel_c  = sum_c;
      *pixel_rc = sum_rc;
    }
  }

/* Do the same for the columns */

  for (ic=0, col_r=g_r, col_c=g_c, col_rc=g_rc;
       ic<ncols-window_size;
       ic++, col_r++, col_c++, col_rc++) {

    /* Calculate the sum of the first "window_size-1" pixels of the column */

    sum_r = sum_c = sum_rc = 0;
    for (ir=0, pixel_r=col_r, pixel_c=col_c, pixel_rc=col_rc;
         ir<window_size-1;
         ir++, pixel_r+=ncols, pixel_c+=ncols, pixel_rc+=ncols) {
      sum_r  += *pixel_r;
      sum_c  += *pixel_c;
      sum_rc += *pixel_rc;
    }

    /* Recursively calculate the sums */

    save_r = save_c = save_rc = 0;
    for (ir=0, pixel_r=col_r, pixel_c=col_c, pixel_rc=col_rc;
         ir<nrows-window_size;
         ir++, pixel_r+=ncols, pixel_c+=ncols, pixel_rc+=ncols) {
      sum_r  += *(pixel_r  + ncols*(window_size-1)) - save_r;
      sum_c  += *(pixel_c  + ncols*(window_size-1)) - save_c;
      sum_rc += *(pixel_rc + ncols*(window_size-1)) - save_rc;
      save_r  = *pixel_r;
      save_c  = *pixel_c;
      save_rc = *pixel_rc;
      *pixel_r  = sum_r;
      *pixel_c  = sum_c;
      *pixel_rc = sum_rc;
    }
  }
}

void fo_undo_box_filter(g_r, g_c, g_rc, nrows, ncols, window_size)
  int *g_r, *g_c, *g_rc, nrows, ncols, window_size;
{
  int ir, ic, sum_r, sum_c, sum_rc, *row_r, *row_c, *row_rc,
      *pixel_r, *pixel_c, *pixel_rc, save_r, save_c, save_rc,
      *col_r, *col_c, *col_rc;

/* First restore in the columns */

  for (ic=0, col_r=g_r, col_c=g_c, col_rc=g_rc;
       ic<ncols-window_size;
       ic++, col_r++, col_c++, col_rc++) {

    /* Calculate the sum of the last "window_size-1" pixels */

    sum_r = sum_c = sum_rc = 0;
    for (ir=0, pixel_r=col_r+(nrows-2)*ncols,
               pixel_c=col_c+(nrows-2)*ncols,
               pixel_rc=col_rc+(nrows-2)*ncols;
         ir<window_size-1;
         ir++, pixel_r-=ncols, pixel_c-=ncols, pixel_rc-=ncols) {
      sum_r  += *pixel_r;
      sum_c  += *pixel_c;
      sum_rc += *pixel_rc;
    }

    /* Recursively calculate the sums and restore the original values */

    save_r = save_c = save_rc = 0;
    for (ir=0, pixel_r=col_r+(nrows-window_size-1)*ncols,
               pixel_c=col_c+(nrows-window_size-1)*ncols,
               pixel_rc=col_rc+(nrows-window_size-1)*ncols;
         ir<nrows-window_size-1;
         ir++, pixel_r-=ncols, pixel_c-=ncols, pixel_rc-=ncols) {
      *pixel_r  -= sum_r;
      *pixel_c  -= sum_c;
      *pixel_rc -= sum_rc;
      sum_r     += *pixel_r - *(pixel_r + ncols*(window_size-1));
      sum_c     += *pixel_c - *(pixel_c + ncols*(window_size-1));
      sum_rc    += *pixel_rc - *(pixel_rc + ncols*(window_size-1));
    }
  }

/* Do the same for the rows */

  for (ir=0, row_r=g_r, row_c=g_c, row_rc=g_rc;
       ir<nrows-1;
       ir++, row_r+=ncols, row_c+=ncols, row_rc+=ncols) {

    /* Calculate the sum of the last "window_size-1" pixels */

    sum_r = sum_c = sum_rc = 0;
    for (ic=0, pixel_r=row_r+ncols-2,
               pixel_c=row_c+ncols-2,
               pixel_rc=row_rc+ncols-2;
         ic<window_size-1;
         ic++, pixel_r--, pixel_c--, pixel_rc--) {
      sum_r  += *pixel_r;
      sum_c  += *pixel_c;
      sum_rc += *pixel_rc;
    }

    /* Recursively calculate the sums */

    save_r = save_c = save_rc = 0;
    for (ic=0, pixel_r=row_r+ncols-window_size-1,
               pixel_c=row_c+ncols-window_size-1,
               pixel_rc=row_rc+ncols-window_size-1;
         ic<ncols-window_size-1;
         ic++, pixel_r--, pixel_c--, pixel_rc--) {
      *pixel_r  -= sum_r;
      *pixel_c  -= sum_c;
      *pixel_rc -= sum_rc;
      sum_r     += *pixel_r - *(pixel_r + window_size-1);
      sum_c     += *pixel_c - *(pixel_c + window_size-1);
      sum_rc    += *pixel_rc - *(pixel_rc + window_size-1);
    }
  }
}


int fo_select(feature_type, g_r, g_c, g_rc, g_s, nrows, ncols,
              border, s_thres, q_thres, num_sel)
  int    feature_type, *g_r, *g_c, *g_rc, **g_s, nrows, ncols, border, s_thres,
         *num_sel;
  double q_thres;
{
  int    ir, ic, *row_r, *row_c, *row_rc, *row_s,
         *pixel_r, *pixel_c, *pixel_rc, *pixel_s;
  double exp1, exp2, root, eigenvalue, det, trace, roundness;

/* Initialize selection counter */

  *num_sel = 0;

/* Allocate memory for the output image */

  if (*g_s == NULL) *g_s = (int *) malloc(nrows * ncols * sizeof(int));
  if (*g_s == NULL) {
    fprintf(stderr, "Error allocating memory for eigenvalue image.\n");
    return(0);
  }

/* Different selection criteria for points and edges:
 * Points: - strength check on smallest eigenvalue
 *         - roundness should be above threshold
 * Edges:  - strength check on sum of eigenvalues (trace)
 *         - roundness should be below threshold
 */

  for (ir=0, row_r=g_r, row_c=g_c, row_rc=g_rc, row_s=*g_s;
       ir<nrows-border;
       ir++, row_r+=ncols, row_c+=ncols, row_rc+=ncols, row_s+=ncols) {
    for (ic=0, pixel_r=row_r, pixel_c=row_c, pixel_rc=row_rc, pixel_s=row_s;
         ic<ncols-border;
         ic++, pixel_r++, pixel_c++, pixel_rc++, pixel_s++) {

      /* Determinant and trace */

      det = (double) *pixel_r * (double) *pixel_c -
            (double) *pixel_rc * (double) *pixel_rc;
      trace = (double) *pixel_r + (double) (*pixel_c);

      /* Roundness */

      if (trace) roundness = 4 * det / (trace * trace);
      else roundness = 1.0;

      /* Eigenvalue calculation and checks for point selection */

      if (feature_type == EXTRACT_POINTS) {
        exp1 = (double) *pixel_r - (double) *pixel_c;
        exp2 = (double) *pixel_rc;
        root = exp1*exp1 + 4*exp2*exp2;
        if (root) root = sqrt(root);
        eigenvalue = ((double) *pixel_r + (double) *pixel_c - root) / 2;
        if (eigenvalue > s_thres && roundness > q_thres) {
          *pixel_s = (int) eigenvalue;
          (*num_sel)++;
        }
        else
          *pixel_s = 0;
      }

      /* Checks for edge selection */

      else {
        if (trace > s_thres && roundness < q_thres) {
          *pixel_s = (int) trace;
          (*num_sel)++;
        }
        else
          *pixel_s = 0;
      }
    }
  }
  return(1);
}

void fo_local_max(g, nrows, ncols, border, filter_size, num_sel)
  int *g, nrows, ncols, border, filter_size, *num_sel;
{
  int ir, ic, *row, *pixel, ir2, ic2, ir2b, ir2e, ic2b, ic2e, *row2, *pixel2,
      offset;

  offset = filter_size / 2;

  for (ir=0, row=g; ir<nrows-border; ir++, row+=ncols) {
    for (ic=0, pixel=row; ic<ncols-border; ic++, pixel++) {
      if (*pixel > 0) {
        ir2b = ir - offset;
        ic2b = ic - offset;
        ir2e = ir2b + filter_size;
        ic2e = ic2b + filter_size;
        if (ir2b < 0) ir2b = 0;
        if (ic2b < 0) ic2b = 0;
        if (ir2e > nrows - border) ir2e = nrows - border;
        if (ic2e > ncols - border) ic2e = ncols - border;
        for (ir2=ir2b, row2=g+ir2b*ncols; ir2<ir2e; ir2++, row2+=ncols) {
          for (ic2=ic2b, pixel2=row2+ic2b; ic2<ic2e; ic2++, pixel2++) {
            if (*pixel < ABS(*pixel2)) {
              *pixel *= -1;
              (*num_sel)--;
              break;
            }
            else if (*pixel > *pixel2 && *pixel2 > 0) {
              *pixel2 *= -1;
              (*num_sel)--;
            }
          }
          if (*pixel < 0) break;
        }
      }
    }
  }
}


void fo_subpixel(g_r, g_c, g_rc, nrows, ncols, window_size, points)
  int    *g_r, *g_c, *g_rc, nrows, ncols, window_size;
  ImgPts *points;
{
  int    ir, ic, *row_r, *row_c, *row_rc, *pixel_r, *pixel_c, *pixel_rc, ip,
         irb, icb, ire, ice;
  double sum_r_gr2, sum_r_grc, sum_c_gc2, sum_c_grc;
  ImgPt  *point;

  for (ip=0, point=points->pts; ip<points->num_pts; ip++, point++) {
    sum_r_gr2 = sum_r_grc = sum_c_gc2 = sum_c_grc = 0;
    irb = (int) (point->r + 0.001);
    icb = (int) (point->c + 0.001);
    ire = irb + window_size;
    ice = icb + window_size;
    for (ir=irb, row_r=g_r+ir*ncols, row_c=g_c+ir*ncols, row_rc=g_rc+ir*ncols;
         ir<ire;
         ir++, row_r+=ncols, row_c+=ncols, row_rc+=ncols) {
      for (ic=icb, pixel_r=row_r+ic, pixel_c=row_c+ic, pixel_rc=row_rc+ic;
           ic<ice;
           ic++, pixel_r++, pixel_c++, pixel_rc++) {
        sum_r_gr2 += ir * *pixel_r;
        sum_r_grc += ir * *pixel_rc;
        sum_c_gc2 += ic * *pixel_c;
        sum_c_grc += ic * *pixel_rc;
      }
    }
    point->r = point->v_r * (sum_r_gr2 + sum_c_grc) +
               point->cv_rc * (sum_r_grc + sum_c_gc2) + 0.5;
    point->c = point->cv_rc * (sum_r_gr2 + sum_c_grc) +
               point->v_c * (sum_r_grc + sum_c_gc2) + 0.5;
  }
}

void fo_direction(g_r, g_c, g_rc, g_s, nrows, ncols, border)
  int *g_r, *g_c, *g_rc, *g_s, nrows, ncols, border;
{
  int   ir, ic, *row_r, *row_c, *row_rc, *row_s,
        *pixel_r, *pixel_c, *pixel_rc, *pixel_s;
  double enum2, denom, pi;

  pi = 4.0 * atan(1.0);

  for (ir=0, row_r=g_r, row_c=g_c, row_rc=g_rc, row_s=g_s;
       ir<nrows-border;
       ir++, row_r+=ncols, row_c+=ncols, row_rc+=ncols, row_s+=ncols) {
    for (ic=0, pixel_r=row_r, pixel_c=row_c, pixel_rc=row_rc, pixel_s=row_s;
         ic<ncols-border;
         ic++, pixel_r++, pixel_c++, pixel_rc++, pixel_s++) {
      if (*pixel_s) {
        enum2 = 2 * *pixel_rc;
/*        denom = *pixel_r - *pixel_c; */
        denom = *pixel_c - *pixel_r;
        *pixel_rc = (int) (100 * atan2(enum2, denom) / pi + 100);
      }
      else *pixel_rc = 0;
    }
  }
}
