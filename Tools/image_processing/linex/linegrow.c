
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
#include <string.h>
#include <math.h>
#include "Database.h"

#define DEBUG 0

int linegrow(g_s, g_dir, nrows, ncols, roff, coff, border, num_sel,
             maxwidth, minlength, lines_ptr)
  int      *g_s, *g_dir, nrows, ncols, border, num_sel;
  double   roff, coff;
  float    maxwidth, minlength;
  ImgLines **lines_ptr;
{
  int      ir, ic, bufsize, nextline, linedone, *row_s, *pixel_s, *pixel_dir,
           icrit, max_s, ip1, ip2, ip, rmin, cmin, rmax, cmax, iphi, iadd, id,
           angle, ok, k, num_alloc, rnb[8], cnb[8], num_nb=8, sum1, iphiadd,
           newline, *s_sorted, *r_sorted, *c_sorted;
  short    *r, *c;
  double   rho, pp, sump, sumrp, sumcp, sumrrp, sumccp, sumrcp, p11, p12, p22,
           phi, sinphi, cosphi, distorg, dist, rmean, cmean, linelen,
           r1, c1, r2, c2, distperp, rf1, cf1, rf2, cf2;
  ImgLines *lines;
  ImgLine  *line;
  ImgPt    point;

  int      order_edge_pixels();

/* Set the relative coordinates of the neighbourhood pixels */

  rnb[0] = 0;      cnb[0] = 1;
  rnb[1] = 0;      cnb[1] = -1;
  rnb[2] = 1;      cnb[2] = 0;
  rnb[3] = -1;     cnb[3] = 0;
  rnb[4] = 1;      cnb[4] = 1;
  rnb[5] = 1;      cnb[5] = -1;
  rnb[6] = -1;     cnb[6] = 1;
  rnb[7] = -1;     cnb[7] = -1;

/* Allocate space for the pixel buffers */

  if (nrows > ncols) bufsize = 10 * nrows;
  else bufsize = 10 * ncols;
  r = (short *) malloc(bufsize * sizeof(short));
  c = (short *) malloc(bufsize * sizeof(short));
  if (r == NULL || c == NULL) {
    fprintf(stderr, "Error allocating pixel buffers.\n");
    return(0);
  }
  icrit = (2 * bufsize) / 3;

/* Allocate space for the lines structure */

  lines = (ImgLines *) malloc(sizeof(ImgLines));
  num_alloc = 10;
  lines->lines = (ImgLine *) malloc(10 * sizeof(ImgLine));
  if (lines == NULL || lines->lines == NULL) {
    fprintf(stderr, "Error allocating space for lines.\n");
    return(0);
  }
  strcpy(lines->img_name," ");
  lines->num_lines = 0;

/* Sort the pixels after gradient strenght. In this order they will be used
 * as start points.
 */

  if (num_sel == 0) {
    printf("There are no pixels with significant gradients.\n");
    return(0);
  }
  if (DEBUG) printf("Sorting selected pixels after gradient strength\n");
  s_sorted = (int *) malloc(num_sel * sizeof(int));
  r_sorted = (int *) malloc(num_sel * sizeof(int));
  c_sorted = (int *) malloc(num_sel * sizeof(int));
  if (s_sorted == NULL || r_sorted == NULL || c_sorted == NULL) {
    fprintf(stderr, "Error allocating memory for sorting pixels.\n");
    return(0);
  }
  (void) order_edge_pixels(g_s, nrows, ncols, border, num_sel,
                           s_sorted, r_sorted, c_sorted);
  if (DEBUG) {
    printf("Done with sorting.\n");
    printf("First pixel (%d,%d) with gradient %d\n",
           r_sorted[0], c_sorted[0], s_sorted[0]);
    printf("Last pixel (%d,%d) with gradient %d\n",
           r_sorted[num_sel-1], c_sorted[num_sel-1], s_sorted[num_sel-1]);
  }

/* Get the first start point */

  max_s = s_sorted[0];
  r[0]  = r_sorted[0];
  c[0]  = c_sorted[0];
  newline  = max_s;
  nextline = 1;

/* Start expanding the next line */

  rho = 50.0 / atan(1.0);
  while (newline) {

/* Initialise the buffer pointers */

    ip1 = 0;
    ip2 = 1;
    linedone = 0;

/* Initialise the line region parameters */

    ir = r[0];
    ic = c[0];

    rmin = ir;   /* Bounding box */
    rmax = ir;
    cmin = ic;
    cmax = ic;

    pp     = (double) max_s;  /* Moments weighted by gradients */
    sum1   = 1;
    sump   = pp;
    sumrp  = (double) ir * pp;
    sumcp  = (double) ic * pp;
    sumrrp = (double) ir * (double) ir * pp;
    sumrcp = (double) ir * (double) ic * pp;
    sumccp = (double) ic * (double) ic * pp;
    p11    = 0.0;
    p12    = 0.0;
    p22    = 0.0;

    pixel_dir = g_dir + ir * ncols + ic;  /* Line parameters */
    iphi      = (int) fmod((double) (*pixel_dir+400), 200.0);
    phi       = (double) *pixel_dir / rho;
    sinphi    = sin(phi);
    cosphi    = cos(phi);
    distorg   = -1.0 * (double) ir * cosphi + (double) ic * sinphi;

/* Negate the gradient so the pixel will not be selected again. */

    pixel_s = g_s + ir * ncols + ic;
    *pixel_s = - *pixel_s;

/* Check the neighbours of the pixels stored in the buffer */

    while (!linedone) {
      iadd = 0;
      for (ip=ip1; ip<ip2; ip++) {
        for (id=0; id<num_nb; id++) {
          ir = r[ip] + rnb[id];
          ic = c[ip] + cnb[id];
          if (ir < 0 || ir >= nrows-border ||
	      ic < 0 || ic >= ncols-border) continue;
          pixel_s = g_s + ir * ncols + ic;
          if (*pixel_s > 0) {

/* Check whether the new pixel may be added to the line region */

            pixel_dir = g_dir + ir * ncols + ic;/* Check gradient orientation */
            angle = iphi - *pixel_dir + 400;
            angle = (int) fmod((double) angle, 200.0);
            if (angle > 100) angle = 200 - angle;
            ok = (angle < 25);

            if (ok) {                           /* Check distance to line */
              dist = -ir * cosphi + ic * sinphi - distorg;
              if (dist < 0) dist = -dist;
              ok = (dist <= maxwidth);
            }

/* Update the region buffer */

            if (ok) {
              iadd++;
              pp       = *pixel_s;
              *pixel_s = - *pixel_s;
              if (ip2 + iadd > bufsize) {
                fprintf(stderr, "Exceeded 10 rows buffer.\n");
                fprintf(stderr, "Program aborted.\n");
                exit(1);
              }
              r[ip2+iadd-1] = ir;
              c[ip2+iadd-1] = ic;

/* Update the line region parameters */

              if (ir < rmin) rmin = ir;  /* Bounding box */
              if (ir > rmax) rmax = ir;
              if (ic < cmin) cmin = ic;
              if (ic > cmax) cmax = ic;

              sum1++;                    /* Moments weighted by gradients */
              sump   += pp;
              sumrp  += (double) ir * pp;
              sumcp  += (double) ic * pp;
              sumrrp += (double) ir * (double) ir * pp;
              sumrcp += (double) ir * (double) ic * pp;
              sumccp += (double) ic * (double) ic * pp;
              rmean  = sumrp/sump;
              cmean  = sumcp/sump;

              /* Line parameters: two different strategies for calculation of
               * line direction. If the number of pixels is small, the average
               * gradient direction is used instead of the direction calculated
               * from the first and second order moments.
               */

              if (sum1 < 10 ||
                  (rmax-rmin < 2 * maxwidth && cmax-cmin < 2 * maxwidth)) {
                iphiadd = *pixel_dir;
                if (iphiadd - iphi > 100) {
                  iphiadd -= 200;
                  if (iphiadd - iphi > 100) iphiadd -= 200;
                }
                else
                  if (iphiadd - iphi < -100) iphiadd += 200;
/*                printf("1 no. %3d old %4d add %4d ", sum1, iphi, iphiadd); */
                iphi = ((sum1 - 1) * iphi + iphiadd) / sum1 + 400;
                iphi = (int) fmod((double) iphi, 200.0);
                phi = (double) iphi / rho;
/*                printf("new %4d dist %5.2f angle %4d\n", iphi, dist, angle); */
              }
              else {
/*                printf("2 no. %3d old %4d ", sum1, iphi); */
                p11 = sumrrp - sump*rmean*rmean;
                p12 = sumrcp - sump*rmean*cmean;
                p22 = sumccp - sump*cmean*cmean;
                if (p12 == 0 && p11 == p22) phi = 0;
                else phi = 0.5 * atan2(2.0 * p12, p22 - p11);
                iphi = (int) (rho * phi) + 400;
                iphi = (int) fmod((double) iphi, 200.0);
/*                printf("new %4d dist %5.2f angle %4d\n", iphi, dist, angle); */
              }
              sinphi = sin(phi);
              cosphi = cos(phi);
              distorg = -rmean*cosphi + cmean*sinphi;
            }
          }
        }
      }

/* Update the buffer pointers and, if necessary, reorder the buffers */

      if (iadd) {
        ip1  = ip2;
        ip2 += iadd;

        if (ip2 > icrit) {
          for (ip=ip1, k=0; ip<ip2; ip++, k++) {
            r[k] = r[ip];
            c[k] = c[ip];
          }
          ip2 -= ip1;
          ip1  = 0;
        }
      }
      else {

/* Store the line, if it is long enough */

        linedone = 1;
        linelen = (double) (rmax - rmin) * (double) (rmax - rmin) +
                  (double) (cmax - cmin) * (double) (cmax - cmin);
        if (linelen) linelen = sqrt(linelen);
        if (linelen > minlength) {
          r1 = rmin;
          c1 = cmin;
          r2 = rmax;
          c2 = cmax;
          if (iphi > 100) {
            rmax = r1;
            r1   = r2;
            r2   = rmax;
          }
          distperp = r1 * cosphi - c1 * sinphi + distorg;
          rf1      = r1 - distperp * cosphi + roff;
          cf1      = c1 + distperp * sinphi + coff;
          distperp = r2 * cosphi - c2 * sinphi + distorg;
          rf2      = r2 - distperp * cosphi + roff;
          cf2      = c2 + distperp * sinphi + coff;
          (lines->num_lines)++;
          if (lines->num_lines > num_alloc) {
            num_alloc += 10;
            lines->lines = (ImgLine *) realloc(lines->lines,
                                               num_alloc * sizeof(ImgLine));
            if (lines->lines == NULL) {
              fprintf(stderr, "Error reallocating lines buffer.\n");
              return(0);
            }
          }
          line = lines->lines + lines->num_lines - 1;
          line->num_pts = 2;
          line->pts = (ImgPt *) malloc(2 * sizeof(ImgPt));
          line->num = lines->num_lines;
	  line->label = 0;
          line->pts[0].r = rf1;
          line->pts[0].c = cf1;
          line->pts[0].num = 10 * line->num + 1;
          line->pts[0].v_r = line->pts[0].v_c = line->pts[0].cv_rc = 0;
          line->pts[1].r = rf2;
          line->pts[1].c = cf2;
          line->pts[1].num = 10 * line->num + 2;
          line->pts[1].v_r = line->pts[1].v_c = line->pts[1].cv_rc = 0;
        }
      }
    }

/* Find the next start point */

    max_s = 0;
    while (nextline < num_sel && !max_s) {
      pixel_s = g_s + r_sorted[nextline] * ncols + c_sorted[nextline];
      if (*pixel_s > 0) {
        max_s = *pixel_s;
        r[0] = r_sorted[nextline];
        c[0] = c_sorted[nextline];
      }
      nextline++;
    }
    newline = max_s;
  }

  *lines_ptr = lines;
  free(r);  free(c);
  free(s_sorted);  free(r_sorted);  free(c_sorted); 
  return(1);
}


int order_edge_pixels(g_s, nrows, ncols, border, num_sel, s, r, c)
  int *g_s, nrows, ncols, border, num_sel, *s, *r, *c;
{
  int ir, ic, ibuf, *row_s, *pixel_s;

  void sort_arrays();

  if (num_sel <= 0) return(0);

  ibuf = 0;
  for (ir=0, row_s=g_s; ir<nrows-border; ir++, row_s+=ncols) {
    for (ic=0, pixel_s=row_s; ic<ncols-border; ic++, pixel_s++) {
      if (*pixel_s) {
        s[ibuf] = *pixel_s;
        r[ibuf] = ir;
        c[ibuf] = ic;
        ibuf++;
      }
    }
  }

  sort_arrays(s, r, c, num_sel);
  return(1);
}

void sort_arrays(s, r, c, num)
  int *s, *r, *c, num;
{
  int n, k, i, itop, j, l, tmp;

  n = num;
  k = (n == 2) ? 4 : 2;
  while (k < n) k *= 2;
  while (1) {
    k = (k-1) / 2;
    if (k <= 0) break;
    itop = n - k;
    for (i=0; i<itop; i++) {
      j = i;
      while (j >= 0) {
        l = j + k;
        if (s[l] > s[j]) {
          tmp = s[l]; s[l] = s[j]; s[j] = tmp;
          tmp = r[l]; r[l] = r[j]; r[j] = tmp;
          tmp = c[l]; c[l] = c[j]; c[j] = tmp;
          j -= k;
        }
        else break;
      }
    }
  }
}
