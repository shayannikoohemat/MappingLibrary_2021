
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



/* Two routines to do a simple least squares adjustment.
 *
 * Update_Normal_Eq
 * Solve_Normal_Eq
 *
 * Last update: 15-07-97
 *
 * George Vosselman
 */

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>

extern void dgedi_(double *, int *, int *, int *, double *, double *, int *);

//#include "digphot_arch.h"

/*------------------------------------------------------------------------------
Updating of the normal equation system by one additional observation.

  a   - row of the design matrix
  y   - corresponding observation
  w   - weight of the observation
  ata - normal matrix
  aty - right hand side of normal equation system
  np  - number of unknown parameters
------------------------------------------------------------------------------*/

void Update_Normal_Eq(double *a, double y, double w, double *ata, 
                      double *aty, int np)
{
  double *aptr1, *aptr2, *ataptr, *atyptr, w2;
  int    ir, ic;

  w2 = w * w;
  for (ir=0, aptr1=a, ataptr=ata, atyptr=aty;
       ir<np;
       ir++, aptr1++, atyptr++) {
    for (ic=0, aptr2=a;
	 ic<np;
	 ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    (*atyptr) += (*aptr1) * y * w2;
  }
}

/*------------------------------------------------------------------------------
 Partial updating of the normal equation system by one additional observation.
 All values outside the given range are 0.0 and do not require updating.

  a     - row of the design matrix
  y     - corresponding observation
  w     - weight of the observation
  ata   - normal matrix
  aty   - right hand side of normal equation system
  np    - number of unknown parameters
  start - first non-zero index of a
  end   - last non-zero index of a
------------------------------------------------------------------------------*/

void Partial_Update_Normal_Eq1(double *a, double y, double w, double *ata, 
                               double *aty, int np, int start, int end)
{
  double *aptr1, *aptr2, *ataptr, *atyptr, w2, *atarowptr;
  int    ir, ic;

  w2 = w * w;
  for (ir=start, aptr1=a+start, atarowptr=ata+start*np, atyptr=aty+start;
       ir<=end;
       ir++, aptr1++, atarowptr+=np, atyptr++) {
    for (ic=start, aptr2=a+start, ataptr=atarowptr+start;
	     ic<=end;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    (*atyptr) += (*aptr1) * y * w2;
  }
}

void Partial_Update_Normal_Eq2(double *a, double y, double w, double *ata, 
                               double *aty, int np, int start1, int end1,
							   int start2, int end2)
{
  double *aptr1, *aptr2, *ataptr, *atyptr, w2, *atarowptr;
  int    ir, ic;

  w2 = w * w;
  for (ir=start1, aptr1=a+start1, atarowptr=ata+start1*np, atyptr=aty+start1;
       ir<=end1;
       ir++, aptr1++, atarowptr+=np, atyptr++) {
    for (ic=start1, aptr2=a+start1, ataptr=atarowptr+start1;
	     ic<=end1;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    for (ic=start2, aptr2=a+start2, ataptr=atarowptr+start2;
	     ic<=end2;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    (*atyptr) += (*aptr1) * y * w2;
  }
  for (ir=start2, aptr1=a+start2, atarowptr=ata+start2*np, atyptr=aty+start2;
       ir<=end2;
       ir++, aptr1++, atarowptr+=np, atyptr++) {
    for (ic=start1, aptr2=a+start1, ataptr=atarowptr+start1;
	     ic<=end1;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    for (ic=start2, aptr2=a+start2, ataptr=atarowptr+start2;
	     ic<=end2;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    (*atyptr) += (*aptr1) * y * w2;
  }
}

void Partial_Update_Normal_Eq3(double *a, double y, double w, double *ata, 
                               double *aty, int np, int start1, int end1,
							   int start2, int end2, int start3, int end3)
{
  double *aptr1, *aptr2, *ataptr, *atyptr, w2, *atarowptr;
  int    ir, ic;

  w2 = w * w;
  for (ir=start1, aptr1=a+start1, atarowptr=ata+start1*np, atyptr=aty+start1;
       ir<=end1;
       ir++, aptr1++, atarowptr+=np, atyptr++) {
    for (ic=start1, aptr2=a+start1, ataptr=atarowptr+start1;
	     ic<=end1;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    for (ic=start2, aptr2=a+start2, ataptr=atarowptr+start2;
	     ic<=end2;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    for (ic=start3, aptr2=a+start3, ataptr=atarowptr+start3;
	     ic<=end3;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    (*atyptr) += (*aptr1) * y * w2;
  }
  for (ir=start2, aptr1=a+start2, atarowptr=ata+start2*np, atyptr=aty+start2;
       ir<=end2;
       ir++, aptr1++, atarowptr+=np, atyptr++) {
    for (ic=start1, aptr2=a+start1, ataptr=atarowptr+start1;
	     ic<=end1;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    for (ic=start2, aptr2=a+start2, ataptr=atarowptr+start2;
	     ic<=end2;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    for (ic=start3, aptr2=a+start3, ataptr=atarowptr+start3;
	     ic<=end3;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    (*atyptr) += (*aptr1) * y * w2;
  }
  for (ir=start3, aptr1=a+start3, atarowptr=ata+start3*np, atyptr=aty+start3;
       ir<=end3;
       ir++, aptr1++, atarowptr+=np, atyptr++) {
    for (ic=start1, aptr2=a+start1, ataptr=atarowptr+start1;
	     ic<=end1;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    for (ic=start2, aptr2=a+start2, ataptr=atarowptr+start2;
	     ic<=end2;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    for (ic=start3, aptr2=a+start3, ataptr=atarowptr+start3;
	     ic<=end3;
	     ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    (*atyptr) += (*aptr1) * y * w2;
  }
}


/*------------------------------------------------------------------------------
Solving the normal equation system with LINPACK subroutines.

   ata - normal matrix
   aty - right hand side of normal equation system
	 this array is overwritten with the determined parameters
   np  - number of unknown parameters
------------------------------------------------------------------------------*/

void Solve_Normal_Eq_Cond(double *ata, double *aty, int np, double *rcond)
{
  int    *ipvt, job=0;
  double *z;

#ifdef hpux
  void dgeco(double *, int *, int *, int *, double *, double *);
  void dgesl(double *, int *, int *, int *, double *, int *);
#else
  void dgeco_(double *, int *, int *, int *, double *, double *);
  void dgesl_(double *, int *, int *, int *, double *, int *);
#endif

  ipvt = (int *) malloc(np * sizeof(int));
  z    = (double *) malloc(np * sizeof(double));

/* LU decomposition */

#ifdef hpux
  dgeco(ata, &np, &np, ipvt, rcond, z);
#else
  dgeco_(ata, &np, &np, ipvt, rcond, z);
#endif

/* Solve the equation system */

#ifdef hpux
  dgesl(ata, &np, &np, ipvt, aty, &job);
#else
  dgesl_(ata, &np, &np, ipvt, aty, &job);
#endif

  free(ipvt);  free(z);
}

void Solve_Normal_Eq(double *ata, double *aty, int np)
{
  double rcond;
  Solve_Normal_Eq_Cond(ata, aty, np, &rcond);
}

/*------------------------------------------------------------------------------
Solving the normal equation system with LINPACK subroutines.

   ata - normal matrix
     this array is overwritten with the covariance matrix
   aty - right hand side of normal equation system
	 this array is overwritten with the determined parameters
   np  - number of unknown parameters
------------------------------------------------------------------------------*/

void Invert_And_Solve_Normal_Eq(double *ata, double *aty, int np, double *rcond)
{
  int    i, j, *ipvt, job=1;
  double *z, det[2];

#ifdef hpux
  void dgeco(double *, int *, int *, int *, double *, double *);
  void dgesl(double *, int *, int *, int *, double *, int *);
#else
  void dgeco_(double *, int *, int *, int *, double *, double *);
  void dgesl_(double *, int *, int *, int *, double *, int *);
#endif

  ipvt = (int *) malloc(np * sizeof(int));
  z    = (double *) malloc(np * sizeof(double));

/* LU decomposition */

#ifdef hpux
  dgeco(ata, &np, &np, ipvt, rcond, z);
#else
  dgeco_(ata, &np, &np, ipvt, rcond, z);
#endif

/* Invert the normal matrix */

#ifdef hpux
  dgedi(ata, &np, &np, ipvt, det, z, &job);
#else
  dgedi_(ata, &np, &np, ipvt, det, z, &job);
#endif

/* Solve the unknowns */

  for (i=0; i<np; i++) { z[i] = aty[i];  aty[i] = 0.0; }
  for (i=0; i<np; i++)
  	for (j=0; j<np; j++)
      aty[i] += ata[i*np+j] * z[j];

  free(ipvt);  free(z);
}

void Convert_To_Correlations(double *cov, int np)
{
  int i, j;
  for (i=0; i<np; i++) {
  	for (j=0; j<np; j++) {
  	  if (i == j) continue;
  	  cov[i*np+j] /= sqrt(cov[i*np+i] * cov[j*np+j]);
  	}
  }
  for (i=0; i<np; i++) cov[i*np+i] = 1.0;
}

// Solve equation system using Cholesky decomposition

void Solve_Normal_Eq_Cholesky(double *ata, double *aty, int np)
{
  int i, j, k;
  double *element, *l_j_k, *l_i_k, *aty_k;
  
  // In-place Cholesky factorisation
  for (i=0; i<np; i++) {
  	for (j=0, element=ata+i*np; j<=i; j++, element++) {
  	  if (i == j) {
  	  	for (k=0, l_j_k=ata+j*np; k<j; k++, l_j_k++) {
  	  	  *element -= (*l_j_k) * (*l_j_k);
  	  	}
  	  	*element = sqrt(*element);
  	  }
  	  else {
  	  	for (k=0, l_i_k=ata+i*np, l_j_k=ata+j*np; k<j; k++, l_i_k++, l_j_k++) {
  	  	  *element -= (*l_i_k) * (*l_j_k);
  	  	}
  	  	*element /= ata[j*np+j];
  	  }
  	}
  }
  
  // Forward substitution
  for (i=0, element=aty; i<np; i++, element++) {
  	for (k=0, l_i_k=ata+i*np, aty_k=aty; k<i; k++, l_i_k++, aty_k++) {
  	  *element -= (*l_i_k) * (*aty_k);
  	}
  	*element /= ata[i*np+i];
  }
  
  // Backward substitution
  for (i=np-1, element=aty+np-1; i>=0; i--, element--) {
  	for (k=np-1, l_i_k=ata+(np-1)*np+i, aty_k=aty+np-1; k>i; k--, l_i_k-=np, aty_k--) {
  	  *element -= (*l_i_k) * (*aty_k);
  	}
    *element /= ata[i*np+i];
  }
}


// Solve equation system using Cholesky decomposition, optimised for (partial)
// band matrices.
// ata, aty - Normal equation system
// np       - Number of parameters
// nb       - Number of parameters in the band
// hbw      - Half the band width

void Solve_Normal_Eq_Cholesky_Band(double *ata, double *aty, int np,
                                   int nb, int hbw)
{
  int i, j, k, kstop;
  double *element, *l_j_k, *l_i_k, *aty_k;
  
  // In-place Cholesky factorisation
  for (i=0; i<np; i++) {
  	if (i < nb) { // Inside band part
  	  j = i - hbw + 1;
  	  if (j < 0) j = 0;
  	  for (element=ata+i*np+j; j<=i; j++, element++) {
  	  	k = i - hbw + 1;
  	  	if (k < 0) k = 0;
  	    if (i == j) {
  	  	  for (l_j_k=ata+j*np+k; k<j; k++, l_j_k++) {
  	  	    *element -= (*l_j_k) * (*l_j_k);
  	  	  }
  	  	  *element = sqrt(*element);
  	    }
  	    else {
  	  	  for (l_i_k=ata+i*np+k, l_j_k=ata+j*np+k; k<j; k++, l_i_k++, l_j_k++) {
  	  	    *element -= (*l_i_k) * (*l_j_k);
  	  	  }
  	  	  *element /= ata[j*np+j];
  	    }
  	  }
  	}
  	else { // Outside band part
  	  for (j=0, element=ata+i*np; j<=i; j++, element++) {
  	    if (i == j) {
  	  	  for (k=0, l_j_k=ata+j*np; k<j; k++, l_j_k++) {
  	  	    *element -= (*l_j_k) * (*l_j_k);
  	  	  }
  	  	  *element = sqrt(*element);
  	    }
  	    else {
  	  	  for (k=0, l_i_k=ata+i*np, l_j_k=ata+j*np; k<j; k++, l_i_k++, l_j_k++) {
  	  	    *element -= (*l_i_k) * (*l_j_k);
  	  	  }
  	  	  *element /= ata[j*np+j];
  	    }
  	  }
  	}
  }
  
  // Forward substitution
  for (i=0, element=aty; i<np; i++, element++) {
  	if (i < nb) { // Inside band part
  	  k = i - hbw + 1;
  	  if (k < 0) k = 0;
  	  for (l_i_k=ata+i*np+k, aty_k=aty+k; k<i; k++, l_i_k++, aty_k++) {
  	    *element -= (*l_i_k) * (*aty_k);
  	  }
  	}
  	else { // Outside band part
  	  for (k=0, l_i_k=ata+i*np, aty_k=aty; k<i; k++, l_i_k++, aty_k++) {
  	    *element -= (*l_i_k) * (*aty_k);
  	  }
  	}
  	*element /= ata[i*np+i];
  }
  
  // Backward substitution
  for (i=np-1, element=aty+np-1; i>=0; i--, element--) {
  	// First the part outside the band
  	kstop = nb - 1;           // Stop at the band part
  	if (kstop < i) kstop = i; // and do not cross diagonal
  	for (k=np-1, l_i_k=ata+(np-1)*np+i, aty_k=aty+np-1; k>kstop;
	     k--, l_i_k-=np, aty_k--) {
  	  *element -= (*l_i_k) * (*aty_k);
  	}
  	// Then the band part
  	k = i + hbw - 1;           // Start at half band width from diagonal
  	if (k > nb - 1) k = nb- 1; // and avoid the non-band area
  	for (l_i_k=ata+k*np+i, aty_k=aty+k; k>i; k--, l_i_k-=np, aty_k--) {
  	  *element -= (*l_i_k) * (*aty_k);
  	}
    *element /= ata[i*np+i];
  }
}

// Functions for sparse partial band matrices

// Global variables for sparse partial band matrices
int    *ata_first_column_of_row;
double **ata_first_element_of_row;
int    ata_number_of_elements;

// Verify whether pointer in in a valid range of ata
void CheckATA(double *ata, double *ptr, char *str)
{
  int index=((int) ptr - (int) ata)/sizeof(double);
  if (index < 0 || index >=ata_number_of_elements) {
    printf("%s index %d\n", str, index);
    exit(0);
  }
}

// Allocation of memory for sparse band triangular normal matrix and setting
// up of internal data structure for quick access to matrix elements
// np       - Number of parameters
// nb       - Number of parameters in the band
// hbw      - Half the band width
// return   - Pointer to allocated ata, or NULL if allocation failed
double * Allocate_Sparse_Band_Normal_Matrix(int np, int nb, int hbw)
{
  double *ata;
  int    nf, row;
  
  nf = np - nb; // Number of full rows
  ata_number_of_elements = nb * hbw - (hbw * (hbw-1)) / 2 + // Band part
                           nf * nb + // Mixed part
                           (nf * (nf+1)) / 2;  // Full part
  ata = (double *) calloc(ata_number_of_elements, sizeof(double));
  if (!ata) {
  	printf("Error allocating normal matrix of %d elements\n", ata_number_of_elements);
  	return NULL;
  }

  // Allocate look up tables for start of row indices and pointers
  ata_first_column_of_row = (int *) malloc(np * sizeof(int));
  ata_first_element_of_row = (double **) malloc(np * sizeof(double *));
  if (!ata_first_column_of_row || !ata_first_element_of_row) {
  	printf("Error allocating ata pointers in Allocate_Sparse_Band_Normal_Matrix\n");
  	free(ata);
  	return NULL;
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
  CheckATA(ata, ata_first_element_of_row[np-1]+np-1, "Init");
  return ata;
}

void Clear_Sparse_Band_Normal_Matrix(double *ata)
{
  memset((void *) ata, 0, ata_number_of_elements * sizeof(double));
}

void Free_Sparse_Band_Normal_Matrix(double *ata)
{
  free(ata);
  free(ata_first_column_of_row);
  free(ata_first_element_of_row);
  ata_number_of_elements = 0;
}

double * Sparse_ATA_Element(double *ata, int row, int column)
{
  int offset;
  if (column > row) {
  	printf("Sparse_ATA_Element: Attempt to access upper triangle, row %d, column %d\n",
	       row, column);
  	exit(0);
  }
  offset = column - ata_first_column_of_row[row];
  if (offset < 0) {
  	printf("Sparse_ATA_Element: Attempt to access element outside band, band start %d, column %d of row %d\n",
	       ata_first_column_of_row[row], column, row);
  	exit(0);
  }
  return ata_first_element_of_row[row] + offset;
}

double Sparse_ATA_Value(double *ata, int row, int column)
{
  int offset;
  if (column > row) return 0.0;
  offset = column - ata_first_column_of_row[row];
  if (offset < 0) return 0.0;
  return *(ata_first_element_of_row[row] + offset);
}

void Update_Sparse_Band_Normal_Eq1(double *a, double y, double w, double *ata, 
                                   double *aty, int np, int nb, int hbw,
								   int start, int end)
{
  double *aptr1, *aptr2, *ataptr, *atyptr, w2, *ata_first_element;
  int    ir, ic, ic_first;

  w2 = w * w;
  for (ir=start, aptr1=a+start, atyptr=aty+start;
       ir<=end;
       ir++, aptr1++, atyptr++) {
//    ic_first = ata_first_column_of_row[ir];
//    ata_first_element = ata_first_element_of_row[ir];
//    for (ic=start, aptr2=a+start, ataptr=ata_first_element + start - ic_first;
    for (ic=start, aptr2=a+start, ataptr=Sparse_ATA_Element(ata, ir, start);
	     ic<=ir;
	     ic++, aptr2++, ataptr++) {
	  CheckATA(ata, ataptr, "Eq1 1");
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    (*atyptr) += (*aptr1) * y * w2;
  }
}

void Update_Sparse_Band_Normal_Eq2(double *a, double y, double w, double *ata, 
                                   double *aty, int np, int nb, int hbw,
								   int start1, int end1, int start2, int end2)
{
  double *aptr1, *aptr2, *ataptr, *atyptr, w2, *ata_first_element;
  int    ir, ic, ic_first;

  w2 = w * w;
  for (ir=start1, aptr1=a+start1, atyptr=aty+start1;
       ir<=end1; ir++, aptr1++, atyptr++) {
    for (ic=start1, aptr2=a+start1, ataptr=Sparse_ATA_Element(ata, ir, start1);
	     ic<=ir; ic++, aptr2++, ataptr++) {
	  CheckATA(ata, ataptr, "Eq2 1");
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    (*atyptr) += (*aptr1) * y * w2;
  }
  for (ir=start2, aptr1=a+start2, atyptr=aty+start2;
       ir<=end2; ir++, aptr1++, atyptr++) {
    for (ic=start1, aptr2=a+start1, ataptr=Sparse_ATA_Element(ata, ir, start1);
	     ic<=end1; ic++, aptr2++, ataptr++) {
	  CheckATA(ata, ataptr, "Eq2 2");
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    for (ic=start2, aptr2=a+start2, ataptr=Sparse_ATA_Element(ata, ir, start2);
	     ic<=ir; ic++, aptr2++, ataptr++) {
	  CheckATA(ata, ataptr, "Eq2 3");
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    (*atyptr) += (*aptr1) * y * w2;
  }
}

void Update_Sparse_Band_Normal_Eq3(double *a, double y, double w, double *ata, 
                                   double *aty, int np, int nb, int hbw,
								   int start1, int end1, int start2, int end2,
								   int start3, int end3)
{
  double *aptr1, *aptr2, *ataptr, *atyptr, w2, *ata_first_element;
  int    ir, ic, ic_first;

  w2 = w * w;
  for (ir=start1, aptr1=a+start1,  atyptr=aty+start1;
       ir<=end1; ir++, aptr1++, atyptr++) {
    ic_first = ata_first_column_of_row[ir];
    ata_first_element = ata_first_element_of_row[ir];
    for (ic=start1, aptr2=a+start1, ataptr=ata_first_element+start1-ic_first;
	     ic<=ir; ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    (*atyptr) += (*aptr1) * y * w2;
  }
  for (ir=start2, aptr1=a+start2, atyptr=aty+start2;
       ir<=end2; ir++, aptr1++, atyptr++) {
    ic_first = ata_first_column_of_row[ir];
    ata_first_element = ata_first_element_of_row[ir];
    for (ic=start1, aptr2=a+start1, ataptr=ata_first_element+start1-ic_first;
	     ic<=end1; ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    for (ic=start2, aptr2=a+start2, ataptr=ata_first_element+start2-ic_first;
	     ic<=ir; ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    (*atyptr) += (*aptr1) * y * w2;
  }
  for (ir=start3, aptr1=a+start3, atyptr=aty+start3;
       ir<=end3; ir++, aptr1++, atyptr++) {
    ic_first = ata_first_column_of_row[ir];
    ata_first_element = ata_first_element_of_row[ir];
    for (ic=start1, aptr2=a+start1, ataptr=ata_first_element+start1-ic_first;
	     ic<=end1; ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    for (ic=start2, aptr2=a+start2, ataptr=ata_first_element+start2-ic_first;
	     ic<=end2; ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    for (ic=start3, aptr2=a+start3, ataptr=ata_first_element+start3-ic_first;
	     ic<=ir; ic++, aptr2++, ataptr++) {
      (*ataptr) += (*aptr1) * (*aptr2) * w2;
    }
    (*atyptr) += (*aptr1) * y * w2;
  }
}

// Solve equation system using Cholesky decomposition, optimised for (partial)
// band matrices.
// ata, aty - Normal equation system
// np       - Number of parameters
// nb       - Number of parameters in the band
// hbw      - Half the band width

void Solve_Normal_Eq_Cholesky_Sparse_Band(double *ata, double *aty, int np,
                                          int nb, int hbw)
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
  	  	  for (l_i_k=Sparse_ATA_Element(ata, i, k), l_j_k=Sparse_ATA_Element(ata, j, k); k<j; k++, l_i_k++, l_j_k++) {
  	  	    *element -= (*l_i_k) * (*l_j_k);
  	  	  }
  	  	  *element /= *Sparse_ATA_Element(ata, j, j);
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
  	  	  for (l_i_k=Sparse_ATA_Element(ata, i, k), l_j_k=Sparse_ATA_Element(ata, j, k); k<j; k++, l_i_k++, l_j_k++) {
  	  	    *element -= (*l_i_k) * (*l_j_k);
  	  	  }
  	  	  *element /= *Sparse_ATA_Element(ata, j, j);
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
  	*element /= *Sparse_ATA_Element(ata, i, i);
  }
  
  // Backward substitution
  if (debug) printf("Backward substitution\n");
  for (i=np-1, element=aty+np-1; i>=0; i--, element--) {
  	// First the part outside the band
  	kstop = nb - 1;           // Stop at the band part
  	if (kstop < i) kstop = i; // and do not cross diagonal
  	for (k=np-1, aty_k=aty+np-1; k>kstop; k--, aty_k--) {
	  l_i_k = Sparse_ATA_Element(ata, k, i);
  	  *element -= (*l_i_k) * (*aty_k);
  	}
  	// Then the band part
  	k = i + hbw - 1;           // Start at half band width from diagonal
  	if (k > nb - 1) k = nb- 1; // and avoid the non-band area
  	for (aty_k=aty+k; k>i; k--, l_i_k-=np, aty_k--) {
  	  l_i_k = Sparse_ATA_Element(ata, k, i);
  	  *element -= (*l_i_k) * (*aty_k);
  	}
    *element /= *Sparse_ATA_Element(ata, i, i);
  }
}
