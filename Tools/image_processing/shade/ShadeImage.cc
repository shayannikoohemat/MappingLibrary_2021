
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
#include <math.h>
#include "viff.h"
#include "stdmath.h"

extern "C" xvimage *createimage(int, int, int, int, int, const char *,
                                int, int, int, int, int, int);

void ShadeImage(xvimage *image, xvimage **shaded_ptr, int log_base)
{
  unsigned char *grey1, *grey2, *greyrow;
  float         *grad, maxgrad, *greyrowf, *grey1f, *grey2f;
  int           ir, ic;
  xvimage       *shaded;
  
/* Create a FLOAT image */

  shaded = createimage(image->col_size-1, image->row_size-1, VFF_TYP_FLOAT,
                       1, 1, "Created by Shade", 0, 0, VFF_MS_NONE,
                       VFF_MAPTYP_NONE, VFF_LOC_IMPLICIT, 0);
  *shaded_ptr = shaded;
  if (!shaded) {
    fprintf(stderr, "Error allocating float image space\n");
    return;
  }

/* Calculate the diagonal gradients */

  if (image->data_storage_type == VFF_TYP_1_BYTE) {
    for (ir=0, greyrow=(unsigned char *) image->imagedata,
               grad=(float *) shaded->imagedata;
         ir<shaded->col_size;
         ir++, greyrow+=image->row_size)
      for (ic=0, grey1=greyrow, grey2=grey1+image->row_size+1;
           ic<shaded->row_size;
           ic++, grad++, grey1++, grey2++)
        *grad = (float) *grey2 - (float) *grey1;
  }                  
  else if (image->data_storage_type == VFF_TYP_FLOAT) {
    for (ir=0, greyrowf=(float *) image->imagedata,
               grad=(float *) shaded->imagedata;
         ir<shaded->col_size;
         ir++, greyrowf+=image->row_size)
      for (ic=0, grey1f=greyrowf, grey2f=grey1f+image->row_size+1;
           ic<shaded->row_size;
           ic++, grad++, grey1f++, grey2f++)
        *grad = *grey2f - *grey1f;
  }
  else {
    printf("Error: invalid data storage type %d\n", image->data_storage_type);
    return;
  }
  
/* Take the signed logarithm of the absolute gradient plus 1 */

  maxgrad = 0;
  for (ir=0, grad=(float *) shaded->imagedata;
       ir<shaded->col_size;
       ir++) {
    for (ic=0; ic<shaded->row_size; ic++, grad++) {
      if (*grad >= 0) {
        if (log_base == 1) *grad = (float) log10((double) *grad + 1);
        else *grad = (float) log((double) *grad + 1);
        if (*grad > maxgrad) maxgrad = *grad;
      }
      else {
        if (log_base == 1) *grad = -1 * (float) log10(ABS((double) *grad) + 1);
        else *grad = -1 * (float) log(ABS((double) *grad) + 1);
        if (*grad < -maxgrad) maxgrad = -1 * (*grad);
      }
    }
  }

/* Normalise the data and store it in unsigned bytes */

  for (ir=0, grad=(float *) shaded->imagedata,
       grey1=(unsigned char *) shaded->imagedata;
       ir<shaded->col_size;
       ir++) {
    for (ic=0; ic<shaded->row_size; ic++, grad++, grey1++)
      *grey1 = (unsigned char) (*grad * 127.5 / maxgrad + 127.5);
    *grey1++ = *(grey1-1); /* Duplicate the last column */
  }
  for (ic=0; ic<=shaded->row_size; ic++, grey1++)
    *grey1 = *(grey1 - shaded->row_size - 1); /* Duplicate the last row */

/* Change the image size and data storage type */

  shaded->col_size++;
  shaded->row_size++;
  shaded->data_storage_type = VFF_TYP_1_BYTE;
}
