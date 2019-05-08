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

int rectpts_c(char *org_imgpts_file, char *par_file,
              char *rect_objpts_file, char *rect_imgpts_file)
{
  ImgPts   *org_imgpts, *rect_imgpts;
  ObjPts2D *rect_objpts;
  double   par[8];

  void     Rectify_to_ImgPts(), Rectify_to_ObjPts2D();


  if ((org_imgpts = Get_ImgPts(org_imgpts_file)) == NULL) {
    fprintf(stderr, "Error reading image points from file %s\n",
            org_imgpts_file);
    exit(0);
  }

  if (!Get_ProjTrans2D(par_file, par)) {
    fprintf(stderr, "Error reading rectification parameters from file %s\n",
            par_file);
    exit(0);
  }


/* -main_before_lib_call_end */

/* -main_library_call */

  if (rect_imgpts_file) {
    rect_imgpts = (ImgPts *) calloc(1, sizeof(ImgPts));
    rect_imgpts->num_pts = org_imgpts->num_pts;
    rect_imgpts->pts = (ImgPt *) calloc(org_imgpts->num_pts, sizeof(ImgPt));
    Rectify_to_ImgPts(org_imgpts, par, rect_imgpts);
    Put_ImgPts(rect_imgpts, rect_imgpts_file);
  }
  else {
    rect_objpts = (ObjPts2D *) calloc(1, sizeof(ObjPts2D));
    rect_objpts->num_pts = org_imgpts->num_pts;
    rect_objpts->pts = (ObjPt2D *) calloc(org_imgpts->num_pts, sizeof(ObjPt2D));
    Rectify_to_ObjPts2D(org_imgpts, par, rect_objpts);
    Put_ObjPts2D(rect_objpts, rect_objpts_file);
  }
  return 1;
}
