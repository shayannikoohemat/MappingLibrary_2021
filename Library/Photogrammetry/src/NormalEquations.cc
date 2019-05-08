
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



/*
--------------------------------------------------------------------------------
 Collection of functions for class NormalEquations

 Initial creation
 Author : George Vosselman
 Date   : 06-06-2017

 Update #1
 Author :
 Date   :
 Changes:

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files                  
--------------------------------------------------------------------------------
*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "NormalEquations.h"
#include "normal_equations.h" // for the old c functions

/*
--------------------------------------------------------------------------------
                               Default constructor
--------------------------------------------------------------------------------
*/

NormalEquations::NormalEquations()
{
  ata = aty = NULL;
  np = nb = 0;
  ata_first_column_of_row = NULL;
  ata_first_element_of_row = NULL;
  ata_inverted = ata_contains_correlations = false;
}

/*
--------------------------------------------------------------------------------
                     Constructor of a partial band matrix
--------------------------------------------------------------------------------
*/

NormalEquations::NormalEquations(int num_par, int &error, int num_band,
	                             int half_band_width, bool sp)

{
  ata = aty = NULL;
  ata_first_column_of_row = NULL;
  ata_first_element_of_row = NULL;
  error = Initialise(num_par, num_band, half_band_width, sp);
}

/*
--------------------------------------------------------------------------------
                     Initialisation of a partial band matrix
--------------------------------------------------------------------------------
*/

int NormalEquations::Initialise(int num_par, int num_band,
	                            int half_band_width, bool sp)
{
  int nf, row;
  
  // Free old memory
  FreeMemory();
    
  // Set internal parameters
  np = num_par;
  nb = num_band;
  ata_inverted = ata_contains_correlations = false;
  if (nb == 0) {
  	ata_num_elements = np * np;
  	sparse = false;
  }
  else {
  	hbw = half_band_width;
  	if (hbw < 0 || hbw > nb) return 2; // Invalid parameter
  	if (nb > np) return 2; // Invalid parameter
  	sparse = sp;
  	if (sparse) {
  	  nf = np - nb;
  	  ata_num_elements = nb * hbw - (hbw * (hbw-1)) / 2 + // Band part
                         nf * nb + // Mixed part
                         (nf * (nf+1)) / 2;  // Full part
    }
    else ata_num_elements = np * np;
  }
  
  // Allocate ata and aty
  ata = (double *) calloc(ata_num_elements, sizeof(double));
  if (!ata) {
  	printf("Error allocating normal matrix of %d elements\n", ata_num_elements);
  	return 1;
  }
  aty = (double *) calloc(np, sizeof(double));
  if (!aty) {
  	printf("Error allocating aty vector of %d elements\n", np);
  	return 1;
  }

  // Data structures for sparse matrix
  if (sparse) {
  	
    // Allocate look up tables for start of row indices and pointers
    ata_first_column_of_row = (int *) malloc(np * sizeof(int));
    ata_first_element_of_row = (double **) malloc(np * sizeof(double *));
    if (!ata_first_column_of_row || !ata_first_element_of_row) {
  	  printf("Error allocating ata pointers sparse matrix\n");
  	  free(ata);
  	  free(aty);
  	  return 1;
    }
  
    // Set up the start of row indices and pointers
    ata_first_column_of_row[0] = 0;
    ata_first_element_of_row[0] = ata;
    for (row=1; row<np; row++) {
  	  if (row < hbw) {
  	    ata_first_column_of_row[row] = 0;
  	    ata_first_element_of_row[row] = ata_first_element_of_row[row-1] + row;
  	  }
  	  else if (row < nb) {
  	    ata_first_column_of_row[row] = row - hbw + 1;
  	    ata_first_element_of_row[row] = ata_first_element_of_row[row-1] + hbw;
  	  }
  	  else {
  	    ata_first_column_of_row[row] = 0;
  	    if (row == nb)
    	  ata_first_element_of_row[row] = ata_first_element_of_row[row-1] + hbw;
  	    else  
  	      ata_first_element_of_row[row] = ata_first_element_of_row[row-1] + row;
  	  }
	}
  }

  return 0;
}

/*
--------------------------------------------------------------------------------
                     Copy assignment
--------------------------------------------------------------------------------
*/
NormalEquations & NormalEquations::operator=(const NormalEquations &neq)
{
  // Initialise with parameters of neq
  Initialise(neq.np, neq.nb, neq.hbw, neq.sparse);
  
  // Copy ata and aty data
  memcpy((void *) ata, (void *) neq.ata, ata_num_elements * sizeof(double));
  memcpy((void *) aty, (void *) neq.aty, np * sizeof(double));
}

/*
--------------------------------------------------------------------------------
                     Resetting ATA and ATY
--------------------------------------------------------------------------------
*/

void NormalEquations::ClearEquations()
{
  memset((void *) ata, 0, ata_num_elements * sizeof(double));
  memset((void *) aty, 0, np * sizeof(double));
}

/*
--------------------------------------------------------------------------------
                    Releasing allocated memory
--------------------------------------------------------------------------------
*/
void NormalEquations::FreeMemory()
{
  if (ata) {free(ata); ata=NULL;}
  if (aty) {free(aty); aty=NULL;}
  if (ata_first_column_of_row) {
    free(ata_first_column_of_row);
    ata_first_column_of_row = NULL;
  }
  if (ata_first_element_of_row) {
    free(ata_first_element_of_row);
    ata_first_element_of_row = NULL;
  }
}


/*
--------------------------------------------------------------------------------
        Retrieve pointer or value of an ATA element
--------------------------------------------------------------------------------
*/

double * NormalEquations::Sparse_ATA_Element(int row, int column)
{
  int offset;
  
  if (column > row) {
   printf("Sparse_ATA_Element: Attempt to access upper triangle, row %d, column %d\n",
          row, column);
   return NULL;
  }
  offset = column - ata_first_column_of_row[row];
  if (offset < 0) {
    printf("Sparse_ATA_Element: Attempt to access element outside band, band start %d, column %d of row %d\n",
           ata_first_column_of_row[row], column, row);
    return NULL;
  }
  return ata_first_element_of_row[row] + offset;
}
  

double * NormalEquations::ATA_Element(int row, int column)
{
  // Check on matrix bounds
  if (row < 0 || row >= np || column < 0 || column >= np) {
  	printf("ATA_Element: Attempt to access element (%d, %d) outside matrix of size (%d x %d)\n",
  	       row, column, np, np);
  	return NULL;
  }
  
  // Sparse matrix
  if (sparse) return Sparse_ATA_Element(row, column);
  
  // Full matrix
  return ata + row * np + column;
}

double NormalEquations::Sparse_ATA_Value(int row, int column)
{
  int offset;
  
  if (column > row) return Sparse_ATA_Value(column, row); // Take value from lower triangle
  offset = column - ata_first_column_of_row[row];
  if (offset < 0) return 0.0; // Outside band must be zero
  return *(ata_first_element_of_row[row] + offset);
}
  

double NormalEquations::ATA_Value(int row, int column)
{
  // Check on matrix bounds
  if (row < 0 || row >= np || column < 0 || column >= np) {
  	printf("ATA_Value: Attempt to access element (%d, %d) outside matrix of size (%d x %d)\n",
  	       row, column, np, np);
  	return NULL;
  }
  
  // Sparse matrix
  if (sparse) return Sparse_ATA_Value(row, column);
  
  // Full matrix
  return *(ata + row * np + column);
}


/*
--------------------------------------------------------------------------------
        Add observation to the normal equation system (various versions)
--------------------------------------------------------------------------------
*/

// Full matrix update
int NormalEquations::AddObservation(double *a, double y, double w)
{
  if (sparse) return 1;
  Update_Normal_Eq(a, y, w, ata, aty, np);
  return 0;
}

// Only a part of a band matrix (either stored as full matrix or sparse matrix)
int NormalEquations::AddObservation(double *a, double y, double w,
                                    int start, int end)
{
  // Check range
  if (start < 0 || end >= np || start > end) return 1;

  if (sparse) {
    double *aptr1, *aptr2, *ataptr, *atyptr, w2, *ata_first_element;
    int    ir, ic;

    w2 = w * w;
    for (ir=start, aptr1=a+start, atyptr=aty+start;
         ir<=end; ir++, aptr1++, atyptr++) {
      for (ic=start, aptr2=a+start, ataptr=Sparse_ATA_Element(ir, start);
	       ic<=ir; ic++, aptr2++, ataptr++) {
        (*ataptr) += (*aptr1) * (*aptr2) * w2;
      }
      (*atyptr) += (*aptr1) * y * w2;
    }
  }
  else { // Use old c function
  	Partial_Update_Normal_Eq1(a, y, w, ata, aty, np, start, end);
  }
  return 0;
}

// Only two parts of a band matrix (either stored as full matrix or sparse matrix)
int NormalEquations::AddObservation(double *a, double y, double w,
                                    int start1, int end1, int start2, int end2)
{
  // Check range
  if (start1 < 0 || end1 >= np || start1 > end1) return 1;
  if (start2 < 0 || end2 >= np || start2 > end2) return 1;
  if (end1 >= start2) return 1;

  if (sparse) {
    double *aptr1, *aptr2, *ataptr, *atyptr, w2, *ata_first_element;
    int    ir, ic;

    w2 = w * w;
    for (ir=start1, aptr1=a+start1, atyptr=aty+start1;
         ir<=end1; ir++, aptr1++, atyptr++) {
      for (ic=start1, aptr2=a+start1, ataptr=Sparse_ATA_Element(ir, start1);
	       ic<=ir; ic++, aptr2++, ataptr++)
        (*ataptr) += (*aptr1) * (*aptr2) * w2;
      (*atyptr) += (*aptr1) * y * w2;
    }
    for (ir=start2, aptr1=a+start2, atyptr=aty+start2;
         ir<=end2; ir++, aptr1++, atyptr++) {
      for (ic=start1, aptr2=a+start1, ataptr=Sparse_ATA_Element(ir, start1);
	       ic<=end1; ic++, aptr2++, ataptr++)
        (*ataptr) += (*aptr1) * (*aptr2) * w2;
      for (ic=start2, aptr2=a+start2, ataptr=Sparse_ATA_Element(ir, start2);
	       ic<=ir; ic++, aptr2++, ataptr++)
        (*ataptr) += (*aptr1) * (*aptr2) * w2;
      (*atyptr) += (*aptr1) * y * w2;
    }
  }
  else { // Use old c function
  	Partial_Update_Normal_Eq2(a, y, w, ata, aty, np, start1, end1, start2, end2);
  }
  return 0;
}

// Only three parts of a band matrix (either stored as full matrix or sparse matrix)
int NormalEquations::AddObservation(double *a, double y, double w,
                                    int start1, int end1, int start2, int end2,
									int start3, int end3)
{
  // Check range
  if (start1 < 0 || end1 >= np || start1 > end1) return 1;
  if (start2 < 0 || end2 >= np || start2 > end2) return 1;
  if (start3 < 0 || end3 >= np || start3 > end3) return 1;
  if (end1 >= start2 || end2 >= start3) return 1;

  if (sparse) {
    double *aptr1, *aptr2, *ataptr, *atyptr, w2, *ata_first_element;
    int    ir, ic;

    w2 = w * w;
    for (ir=start1, aptr1=a+start1,  atyptr=aty+start1;
         ir<=end1; ir++, aptr1++, atyptr++) {
      for (ic=start1, aptr2=a+start1, ataptr=Sparse_ATA_Element(ir, start1);
	       ic<=ir; ic++, aptr2++, ataptr++)
        (*ataptr) += (*aptr1) * (*aptr2) * w2;
      (*atyptr) += (*aptr1) * y * w2;
    }
    for (ir=start2, aptr1=a+start2, atyptr=aty+start2;
         ir<=end2; ir++, aptr1++, atyptr++) {
      for (ic=start1, aptr2=a+start1, ataptr=Sparse_ATA_Element(ir, start1);
	       ic<=end1; ic++, aptr2++, ataptr++)
        (*ataptr) += (*aptr1) * (*aptr2) * w2;
      for (ic=start2, aptr2=a+start2, ataptr=Sparse_ATA_Element(ir, start2);
	       ic<=ir; ic++, aptr2++, ataptr++)
        (*ataptr) += (*aptr1) * (*aptr2) * w2;
      (*atyptr) += (*aptr1) * y * w2;
    }
    for (ir=start3, aptr1=a+start3, atyptr=aty+start3;
         ir<=end3; ir++, aptr1++, atyptr++) {
      for (ic=start1, aptr2=a+start1, ataptr=Sparse_ATA_Element(ir, start1);
	       ic<=end1; ic++, aptr2++, ataptr++)
        (*ataptr) += (*aptr1) * (*aptr2) * w2;
      for (ic=start2, aptr2=a+start2, ataptr=Sparse_ATA_Element(ir, start2);
	       ic<=end2; ic++, aptr2++, ataptr++)
        (*ataptr) += (*aptr1) * (*aptr2) * w2;
      for (ic=start3, aptr2=a+start3, ataptr=Sparse_ATA_Element(ir, start3);
	       ic<=ir; ic++, aptr2++, ataptr++) 
        (*ataptr) += (*aptr1) * (*aptr2) * w2;
      (*atyptr) += (*aptr1) * y * w2;
    }
  }
  else { // Use old c function for full matrices
  	Partial_Update_Normal_Eq3(a, y, w, ata, aty, np, start1, end1, start2, end2,
	                          start3, end3);
  }
  return 0;
}

/*
--------------------------------------------------------------------------------
                 Solve the normal equation system (various versions)
--------------------------------------------------------------------------------
*/

double * NormalEquations::Solve(double &cond)
{
  if (sparse) return NULL;
  // Use the c function calling LINPACK library
  Solve_Normal_Eq_Cond(ata, aty, np, &cond);
  return aty;
}

double * NormalEquations::InvertAndSolve(double &cond)
{
  if (sparse) return NULL;
  // Use the c function calling LINPACK library
  Invert_And_Solve_Normal_Eq(ata, aty, np, &cond);
  ata_inverted = true;
  return aty;
}

double * NormalEquations::CovarianceMatrix()
{
  if (!ata_inverted || ata_contains_correlations) return NULL;
  return ata;
}

double * NormalEquations::Solve()
{
  if (ata_inverted || ata_contains_correlations) {
  	printf("Error: call to NormalEquations::Solve() after the normal matrix has been inverted\n");
  	return NULL;
  }

  if (!sparse) {
  	if (nb == 0) // Full matrix, no band part, use C function
  	  Solve_Normal_Eq_Cholesky(ata, aty, np);
  	else // Full matrix, band part, use C function
  	  Solve_Normal_Eq_Cholesky_Band(ata, aty, np, nb, hbw);
  }
  else // Sparse matrix with (partial) band
    SolveSparse();

  return aty;
}

void NormalEquations::SolveSparse()
{
  int i, j, k, kstop, debug=0;
  double *element, *l_j_k, *l_i_k, *aty_k;
  
  // In-place Cholesky factorisation
  if (debug) printf("Factorisation\n");
  for (i=0; i<np; i++) {
  	if (i < nb) { // Inside band part
  	  j = i - hbw + 1; // First column inside band
  	  if (j < 0) j = 0;
  	  for (element=ata_first_element_of_row[i]; j<=i; j++, element++) {
  	  	k = i - hbw + 1;
  	  	if (k < 0) k = 0;
  	    if (i == j) {
  	  	  for (l_j_k=ata_first_element_of_row[j]; k<j; k++, l_j_k++) {
  	  	    *element -= (*l_j_k) * (*l_j_k);
  	  	  }
  	  	  *element = sqrt(*element);
  	    }
  	    else {
  	  	  for (l_i_k=Sparse_ATA_Element(i, k), l_j_k=Sparse_ATA_Element(j, k);
			   k<j; k++, l_i_k++, l_j_k++) {
  	  	    *element -= (*l_i_k) * (*l_j_k);
  	  	  }
  	  	  *element /= *Sparse_ATA_Element(j, j);
  	    }
  	  }
  	}
  	else { // Outside band part
  	  for (j=0, element=ata_first_element_of_row[i]; j<=i; j++, element++) {
  	    if (i == j) {
  	  	  for (k=0, l_j_k=ata_first_element_of_row[j]; k<j; k++, l_j_k++) {
  	  	    *element -= (*l_j_k) * (*l_j_k);
  	  	  }
  	  	  *element = sqrt(*element);
  	    }
  	    else {
  	      if (j < nb) { // Don't get outside the band if row j is in the band part
  	      	k = j - hbw + 1;
  	      	if (k < 0) k = 0;
  	      }
  	      else k = 0;
  	  	  for (l_i_k=Sparse_ATA_Element(i, k), l_j_k=Sparse_ATA_Element(j, k);
			   k<j; k++, l_i_k++, l_j_k++) {
  	  	    *element -= (*l_i_k) * (*l_j_k);
  	  	  }
  	  	  *element /= *Sparse_ATA_Element(j, j);
  	    }
  	  }
  	}
  }
  
  // Forward substitution
  if (debug) printf("Forward substitution\n");
  for (i=0, element=aty; i<np; i++, element++) {
  	if (i < nb) { // Inside band part
  	  k = i - hbw + 1;
  	  if (k < 0) k = 0;
  	  for (l_i_k=ata_first_element_of_row[i], aty_k=aty+k; k<i; k++, l_i_k++, aty_k++) {
  	    *element -= (*l_i_k) * (*aty_k);
  	  }
  	}
  	else { // Outside band part
  	  for (k=0, l_i_k=ata_first_element_of_row[i], aty_k=aty; k<i; k++, l_i_k++, aty_k++) {
  	    *element -= (*l_i_k) * (*aty_k);
  	  }
  	}
  	*element /= *Sparse_ATA_Element(i, i);
  }
  
  // Backward substitution
  if (debug) printf("Backward substitution\n");
  for (i=np-1, element=aty+np-1; i>=0; i--, element--) {
  	// First the part outside the band
  	kstop = nb - 1;           // Stop at the band part
  	if (kstop < i) kstop = i; // and do not cross diagonal
  	for (k=np-1, aty_k=aty+np-1; k>kstop; k--, aty_k--) {
	  l_i_k = Sparse_ATA_Element(k, i);
  	  *element -= (*l_i_k) * (*aty_k);
  	}
  	// Then the band part
  	k = i + hbw - 1;           // Start at half band width from diagonal
  	if (k > nb - 1) k = nb- 1; // and avoid the non-band area
  	for (aty_k=aty+k; k>i; k--, l_i_k-=np, aty_k--) {
  	  l_i_k = Sparse_ATA_Element(k, i);
  	  *element -= (*l_i_k) * (*aty_k);
  	}
    *element /= *Sparse_ATA_Element(i, i);
  }
}

/*
--------------------------------------------------------------------------------
                 Convert to and return correlation coefficients
--------------------------------------------------------------------------------
*/

double * NormalEquations::ConvertToCorrelations()
{
  if (!ata_inverted) {
  	printf("Correlation coefficients cannot be computed, because the normal matrix has not been inverted.\n");
    return NULL;
  }
  Convert_To_Correlations(ata, np);
  ata_contains_correlations = true;
  return ata;
}

double * NormalEquations::CorrelationCoefficients()
{
  if (!ata_contains_correlations) {
  	printf("Error: Correlations have not yet been computed with ConvertToCorrelations()\n");
  	return NULL;
  }
  return ata;
}


