
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

//
#ifndef Tools_H
#define Tools_H

#include "MathTools_TUB.h"
#include "cv.h"
#include "cvaux.h"
#include "cxcore.h"
#include "highgui.h"
#include "MatrixTools.h"
#include "LaserPoints.h"
#include "EdgeTools.h"
#include "ImageLines.h"


bool P_convert(double *P, matrix **P_mat);
bool KRt_from_P(double *P, matrix **K, matrix**R, matrix **t);
bool KRt_from_P(matrix *P, matrix **K, matrix**R, matrix **t);
bool P_from_KRt(matrix *K, matrix*R, matrix *t, matrix **P);
void JoinDual(double A[4],double B[4],double *C);
bool Q_from_P(double *P, vector<double> &Q);
void lQL(vector<double> Q, vector<double> L, vector<double> &l);
void lQL(vector<double> Q, matrix* L, vector<double> &l);
void dlQdLi_num(vector<double> Q, vector<double> L, int L_index, double *dl1, double *dl2, double *dl3);//Jacobian (only with respect to Li) by numercial diff,
void dlQdLi_num(vector<double> Q, matrix* L, int L_index, double *dl1, double *dl2, double *dl3);//overloaded version

bool F_from_P(double *P_left, double *P_right, matrix**F);


bool check_value_in_vec(vector<int> &v, int val);
bool check_value_in_vec(vector<unsigned int> &v, unsigned int val);
bool get_lines(int view, const char* dirname, const char* image, ImageLines  &ILI);

#endif //Tools_H
