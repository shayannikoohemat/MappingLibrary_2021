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

extern void Image_To_Object(Grid *, ImgPt *, ObjPt *, double);

/*-----------------------------------------------------------
|
|    Written By: George Vosselman
|          Date: Jul 28, 1999
| Modifications:
|
------------------------------------------------------------*/

int pix2obj_c(char *i_file, char *o_file, char *g_file, double height)
{
  ImgPts   *imgpts;
  Grid     *grid;
  ObjPts   *objpts;
  ImgPt    *imgpt;
  ObjPt    *objpt;
  int      i;

  // Read the grid and the image points
  grid = Get_Grid(g_file);
  imgpts = Get_ImgPts(i_file);
  
  // Allocate space for the object points 
  objpts = (ObjPts *) malloc(sizeof(ObjPts));
  objpts->pts = (ObjPt *) calloc(imgpts->num_pts, sizeof(ObjPt));
  objpts->num_pts = imgpts->num_pts;


  for (i=0, imgpt=imgpts->pts, objpt=objpts->pts;
       i<imgpts->num_pts;
       i++, imgpt++, objpt++)
    Image_To_Object(grid, imgpt, objpt, height);

  Put_ObjPts(objpts, o_file);

  return 1;
}
