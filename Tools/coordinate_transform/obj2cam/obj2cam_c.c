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

extern int lobj2cam(ObjPt *, Interior *, Exterior *, CamPt *);
extern int CheckID(FILE *, int);

/*-----------------------------------------------------------
|
|    Written By: R.Th. Ursem
|          Date: Jul 28, 1999
| Modifications:
|
------------------------------------------------------------*/

int obj2cam_c(char *i_file, char *o_file, char *int_file, char *ext_file)
{
  Interior *interior;
  Exterior *exterior;
  CamPts *campts;
  ObjPts *objpts;
  ObjPt  objpt;
  CtrlPts *ctrlpts;
  int i, error, numpts, data_is_object;
  FILE *fp;

  // Read orientation data
  interior = Get_Interior(int_file);
  exterior = Get_Exterior(ext_file, &error);

  // Read object or control points
  if ((fp = fopen(i_file, "r")) == NULL) {
    fprintf(stderr, "%obj2cam: Error opening point database %s\n", i_file);
    exit(0);
  }
  data_is_object = CheckID(fp, OBJECT_POINTS);
  fclose(fp);

  if (data_is_object) {
    objpts   = Get_ObjPts(i_file);
    Print_ObjPts(objpts);
    numpts = objpts->num_pts;
  }
  else {
    ctrlpts = Get_CtrlPts(i_file);
    Print_CtrlPts(ctrlpts);
    numpts = ctrlpts->num_pts;
    objpt.v_x = objpt.v_y = objpt.v_z = 0.0;
    objpt.cv_xy = objpt.cv_xz = objpt.cv_yz = 0.0;
  }
   

  // Allocate space for the camera points
  campts = (CamPts *)calloc(1, sizeof(CamPts));
  campts->num_pts = numpts;
  campts->pts = (CamPt *)calloc(campts->num_pts, sizeof(CamPt));

  for (i = 0; i < numpts; i++) {
    if (data_is_object) {
      objpt = objpts->pts[i];
    }
    else {
      objpt.num = ctrlpts->pts[i].num;
      objpt.x   = ctrlpts->pts[i].x;
      objpt.y   = ctrlpts->pts[i].y;
      objpt.z   = ctrlpts->pts[i].z;
    }
    lobj2cam(&objpt, interior, exterior, &(campts->pts[i]));
  }

  Put_CamPts(campts, o_file);
  Print_CamPts(campts);

  return 1;
}
