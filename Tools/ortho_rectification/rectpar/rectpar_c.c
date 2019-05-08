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

int rectpar_c(char *imgpts_file, char *par_file, char *ctrl_file,
              char *imgptsref_file, int min_num_pts, double max_rms,
              int max_iter)
{
  ImgPts  *imgpts, *imgptsref;
  CtrlPts *ctrlpts;
  double  par[8];

  int Calculate_Rectification_Parameters();

  if ((imgpts = Get_ImgPts(imgpts_file)) == NULL) {
    fprintf(stderr, "Error reading image points from file %s\n", imgpts_file);
    exit(0);
  }
  if (ctrl_file) { /* Control points as reference */
    if ((ctrlpts = Get_CtrlPts(ctrl_file)) == NULL) {
      fprintf(stderr, "Error reading control points from file %s\n", ctrl_file);
      exit(0);
    }
    imgptsref = NULL;
  }
  else { /* Image points in reference image */
    if ((imgptsref = Get_ImgPts(imgptsref_file)) == NULL) {
      fprintf(stderr, "Error reading reference image points from file %s\n",
              imgptsref_file);
      exit(0);
    }
    ctrlpts = NULL;
  }

  if (!Calculate_Rectification_Parameters(imgpts, ctrlpts, imgptsref, par,
                                          min_num_pts, max_rms, max_iter)) {
    fprintf(stderr, "rectpar failed.\n");
    exit(0);
  }

  if (!Put_ProjTrans2D(par_file, par)) {
    fprintf(stderr, "Error writing parameters to file %s\n", par_file);
    exit(0);
  }

  return 1;
}
