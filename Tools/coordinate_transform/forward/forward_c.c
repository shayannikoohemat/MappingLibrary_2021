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

extern int lforward(CamPts *, CamPts *, Interior *, Interior *,
                    Exterior *, Exterior *, ObjPts *);

/*------------------------------------------------------------
|
|    Written By: Ir. R.Th. Ursem
|          Date: Jul 28, 1999
| Modifications:
|
------------------------------------------------------------*/

int forward_c(char *i1_file, char *i2_file, char *int1_file, char *int2_file,
              char *ext1_file, char *ext2_file, char *rel_file, char *o_file)
{
  CamPts *pts1, *pts2;
  ModelPts *pts3;
  Interior *int1, *int2;
  Exterior *ext1, *ext2;
  int i, j;

  pts1 = Get_CamPts(i1_file);
  Print_CamPts(pts1);
   
  pts2 = Get_CamPts(i2_file);
  Print_CamPts(pts2);
   
  pts3 = (ModelPts *) calloc(1, sizeof(ModelPts) );
  pts3->num_pts = 0;
   
  for (i = 0; i < pts1->num_pts; i++)
    for (j = 0; j < pts2->num_pts; j++)
      if (pts1->pts[i].num == pts2->pts[j].num)
        pts3->num_pts++;
  pts3->pts = (ModelPt *) calloc( pts3->num_pts, sizeof(ModelPt) );
   
  if (rel_file) {
    ext1 = (Exterior *)calloc(1, sizeof(Exterior));
    ext2 = (Exterior *)calloc(1, sizeof(Exterior));
    Get_Relative(rel_file, ext1, ext2);
    Print_Exterior(ext2);
  }
  else {
    ext1 = Get_Exterior(ext1_file, &i);
    ext2 = Get_Exterior(ext2_file, &i);
    Print_Exterior(ext1);
    Print_Exterior(ext2);
  }
   
  int1 = Get_Interior(int1_file);
  int2 = Get_Interior(int2_file);

  if (!lforward(pts1, pts2, int1, int2, ext1, ext2, pts3)) {
    fprintf(stderr, "\tlforward failed!\n");
    exit(1);
  }

  if (rel_file) Put_ModelPts(pts3, o_file);
  else Put_ObjPts(pts3, o_file);

  return 1;
}
