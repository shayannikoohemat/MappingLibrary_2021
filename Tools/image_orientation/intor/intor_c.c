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
#include "Database.h"

extern int lintor(ImgPts *, CamPts *, Interior *, double, double);

/*-----------------------------------------------------------
|
|    Written By: George Vosselman
|          Date: Aug 18, 1999
| Modifications:
|
------------------------------------------------------------*/

int intor_c(char *img_pts_file, char *pht_pts_file, char *intor_file,
            double cc, double xh, double yh, int dim_r, int dim_c,
            double k1, double k2, double k3, double p1, double p2)
{
   Interior *interior;
   ImgPts *imgpts;
   CamPts *photopts;

/* Get the image and camera coordinates of the fiducial marks */

   imgpts = Get_ImgPts(img_pts_file);
   photopts = Get_CamPts(pht_pts_file);
   
/* Get the user specified parameters of the interior orientation
   and put them into the structure */

   interior = (Interior *) calloc(1, sizeof(Interior));
   interior->cc = cc;
   interior->k1 = k1;
   interior->k2 = k2;
   interior->k3 = k3;
   interior->p1 = p1;
   interior->p2 = p2;
   interior->dim_c = dim_c;
   interior->dim_r = dim_r;
   interior->shear = 0.0;
   
   if (lintor(imgpts, photopts, interior, xh, yh))
     Put_Interior( interior, intor_file);

   return 1;
}
