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

extern void Image_To_Object_2D(Grid *, ImgPt *, ObjPt2D *);

/*-----------------------------------------------------------
|
|    Written By: George Vosselman
|          Date: Aug 18, 1999
| Modifications:
|
------------------------------------------------------------*/

int pix2obj2d_c(char *i_file, char *o_file, char *g_file)
{
  ImgPts   *imgpts;
  Grid     *grid;
  ObjPts2D *objpts2d;
  ImgPt    *imgpt;
  ObjPt2D  *objpt2d;
  int      i;

  grid = Get_Grid(g_file);
  imgpts = Get_ImgPts(i_file);
  
  objpts2d = (ObjPts2D *) malloc(sizeof(ObjPts2D));
  objpts2d->pts = (ObjPt2D *) calloc(imgpts->num_pts, sizeof(ObjPt2D));
  objpts2d->num_pts = imgpts->num_pts;

  for (i=0, imgpt=imgpts->pts, objpt2d=objpts2d->pts;
       i<imgpts->num_pts;
       i++, imgpt++, objpt2d++)
    Image_To_Object_2D(grid, imgpt, objpt2d);

  Put_ObjPts2D(objpts2d, o_file);

  return 1;
}
