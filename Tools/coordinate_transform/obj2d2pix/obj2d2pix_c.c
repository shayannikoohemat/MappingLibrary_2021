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

extern void Object_2D_To_Image(Grid *, ObjPt2D *, ImgPt *);

/*-----------------------------------------------------------
|
|    Written By: George Vosselman
|          Date: Nov 10, 2000
| Modifications:
|
------------------------------------------------------------*/

int obj2d2pix_c(char *i_file, char *o_file, char *g_file)
{
  ImgPts   *imgpts;
  Grid     *grid;
  ObjPts2D *objpts2d;
  ImgPt    *imgpt;
  ObjPt2D  *objpt2d;
  int      i;

  grid = Get_Grid(g_file);
  objpts2d = Get_ObjPts2D(i_file);
  
  if (!grid || !objpts2d) exit(0);

  imgpts = (ImgPts *) malloc(sizeof(ImgPts));
  imgpts->num_pts = objpts2d->num_pts;
  imgpts->pts = (ImgPt *) calloc(imgpts->num_pts, sizeof(ImgPt));
  strcpy(imgpts->img_name, i_file);

  for (i=0, imgpt=imgpts->pts, objpt2d=objpts2d->pts;
       i<imgpts->num_pts;
       i++, imgpt++, objpt2d++)
    Object_2D_To_Image(grid, objpt2d, imgpt);

  Put_ImgPts(imgpts, o_file);

  return 1;
}
