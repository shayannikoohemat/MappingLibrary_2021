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

extern void File_To_Metric(Interior *, ImgPts *, CamPts *);

/*-----------------------------------------------------------
|
|    Written By: F.A. van den Heuvel
|          Date: Jul 28, 1999
| Modifications:
|
------------------------------------------------------------*/

int pix2cam_c(char *img_pts_file, char *cam_pts_file, char *intor_file) 
{
  ImgPts *img;
  CamPts *cam;
  Interior *interior;

  interior = Get_Interior(intor_file);
  if (interior == NULL) {
    printf("pix2cam_c: Error reading interior orientation from file %s\n",
           intor_file);
    return 0;
  }
  Print_Interior(interior);

  img = Get_ImgPts(img_pts_file);
  if (img == NULL) {
    printf("pix2cam_c: Error reading image points from file %s\n",
           img_pts_file);
    return 0;
  }
  Print_ImgPts(img);
  
  cam = (CamPts *) malloc( sizeof(CamPts) );
  cam->pts = (CamPt *) calloc(img->num_pts, sizeof(CamPt) );
  
  File_To_Metric(interior,img,cam);

  Put_CamPts(cam, cam_pts_file);
  Print_CamPts(cam);

  return 1;
}
