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
#include <string.h>
#include "Database.h"

extern void File_To_Pix(Interior *, CamPts *, ImgPts *);

/*-----------------------------------------------------------
|
|    Written By: F.A. van den Heuvel
|          Date: Jul 28, 1999
| Modifications:
|
------------------------------------------------------------*/

int cam2pix_c(char *cam_pts_file, char *img_pts_file, char *intor_file) 
{
  ImgPts *img;
  Interior *interior;
  CamPts *cam;

  interior = Get_Interior(intor_file);
  Print_Interior(interior);

  cam = Get_CamPts(cam_pts_file);
  Print_CamPts(cam);
  
  img = (ImgPts *)malloc( sizeof(ImgPts) );
  img->pts = (ImgPt *)calloc(cam->num_pts, sizeof(ImgPt) );
  strcpy(img->img_name, img_pts_file);
  
  File_To_Pix(interior,cam,img);

  Print_ImgPts(img);
  Put_ImgPts(img, img_pts_file);
  
  return 1;
}
