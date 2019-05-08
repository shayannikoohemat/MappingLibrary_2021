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

/*-----------------------------------------------------------
|
|    Written By: Evelyn Koeller
Modifications by R.Th. Ursem
|          Date: May 18, 2002
| Modifications:
|
------------------------------------------------------------*/

int rectify_c(char *original_file, char *img_pts_file,
              char *ctrl_pts_file, char *rectified_file,
              double pixel_size, int selected_area,
              double xmin, double ymin, double xmax, double ymax,
              int interpolation_method, char *parameter_file)
{
  int i;
  xvimage *viff_in, *viff_out, *readimage();
  ImgPts *org;
  CtrlPts *ctrl;

  viff_in = readimage(original_file);
  if (viff_in == NULL) {
    fprintf(stderr, "Error: Can not read input image %s.\n", original_file);
    exit(1);
  }
  proper_num_images("rectify" ,viff_in, 1, 1);

  org  = Get_ImgPts(img_pts_file);
  if (org == NULL) {
    printf("Error reading image points from file %s\n", img_pts_file);
    exit(1);
  }
  ctrl = Get_CtrlPts(ctrl_pts_file);
  if (ctrl == NULL) {
    printf("Error reading control points from file %s\n", ctrl_pts_file);
    exit(1);
  }
   
  if (lrectify(org,                    /* measured image points (pix)       */
               ctrl,                   /* given control points  (metric)    */
               viff_in,                /* input image                       */
               interpolation_method,   /* resample methode                  */
               !selected_area,         /* 1 if complete image to be rectified */
               xmin, xmax, ymin, ymax, /* Area to rectify (user defined)    */
               0,                      /* Border color                      */
               pixel_size,             /* Pixel size                        */
               &viff_out,              /* Output image                      */
               parameter_file)) {
    printf("Writing rectified image to %s\n", rectified_file);
    writeimage(rectified_file, viff_out);
  }
  else return 0;
  return 1;
}
