
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
//
/*
 * MatrixTools.h
 *
 *  Created on: 17-Jul-2008
 *      Author: gerke
 */
#ifndef MATRIX_TOOLS_H_
#define MATRIX_TOOLS_H_

#include "MathTools_TUB.h"
#include "cv.h"
#include "cvaux.h"
#include "cxcore.h"
#include <vector>

#ifndef STANDALONE
#include "Rectification/src/Rectify/General2Im.h"
#endif

inline double signof(double a) { return (a == 0) ? 0 : (a<0 ? -1 : 1); }

void add_noise(matrix *A, double max_rel);
void add_outliers(matrix *A,double number_rel , double size_rel); //outliers: number of outliers (percentage) and size (offset: percentage of actual value)
double mat_max_norm_val_vec(matrix *v, int &max_r); //help function for outlier search: finds largest value in v (normed)
bool mat_PseudoInverseofP(matrix *P, matrix **PP);
double mat_largest_abs_element(matrix *A);
bool mat_save(char * fname, matrix * A);
bool mat_load(char * fname, matrix **A);
matrix* sort_matrix_per_row(matrix *A);
///Least Squares Adjustment, Gauss Markov.
///INPUT: A, P , AtP , Ninv, l, XNull,k (for blunder detection)
///OUTPUT
///XNull=XNull+x_dach, v, test1_Val_zero, FAIL (return value)
///Ninv and AtP are desired because it is not always necessary to compute the A new in every iteration, so N, Ninv need to be computed outside to save computing time
///convergence is not checked!!! This needs to be done outside this function
bool LSA_Gauss_Markov(matrix *A, matrix *P, matrix *AtP, matrix *Ninv, matrix *l, double k, matrix *XNull, matrix **v, double *test1_val_zero, bool exit_on_blunder=0);
matrix * mat_invert_opencv(matrix *A);

#ifndef STANDALONE
 matrix * mat_invert_lapack(matrix *A);
 matrix * mat_invert_lapack_bandmat(matrix *A, int width);
 bool mat_PseudoInverse(matrix *A, matrix **P);
#endif
#endif /* MATRIX_TOOLS_H_ */
