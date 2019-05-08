
/*
                    Copyright 2014 University of Twente 
 
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

// Re-using C functions already in use in several programmes

extern "C" void Update_Normal_Eq(double *a, double y, double w, double *ata, 
                                 double *aty, int np);
extern "C" void Partial_Update_Normal_Eq1(double *a, double y, double w, double *ata, 
                                          double *aty, int np, int start1, int end1);
extern "C" void Partial_Update_Normal_Eq2(double *a, double y, double w, double *ata, 
                                          double *aty, int np, int start1, int end1,
							              int start2, int end2);
extern "C" void Partial_Update_Normal_Eq3(double *a, double y, double w, double *ata, 
                                          double *aty, int np, int start1, int end1,
							              int start2, int end2, int start3, int end3);
extern "C" void Solve_Normal_Eq(double *ata, double *aty, int np);
extern "C" void Solve_Normal_Eq_Cond(double *ata, double *aty, int np, double *cond);
extern "C" void Invert_And_Solve_Normal_Eq(double *ata, double *aty, int np, double *cond);
extern "C" void Convert_To_Correlations(double *cov, int np);
extern "C" void Solve_Normal_Eq_Cholesky(double *ata, double *aty, int np);
extern "C" void Solve_Normal_Eq_Cholesky_Band(double *ata, double *aty, int np,
                                              int nb, int hbw);
                                              

#ifndef _NormalEquations_h_
#define _NormalEquations_h_


//------------------------------------------------------------------------------
//                             NormalEquations
/*
Set of functions for allocating, initialising, filling and solving normal
equation systems with full matrices or band matrices with sparse matrix
structures.

*/
//------------------------------------------------------------------------------

class NormalEquations {

  protected:
  	
  	/// The normal matrix A^T A
  	double *ata;
  	
  	/// The vector A^T y
  	double *aty;
  	
  	/// Number of unknowns
  	int np;
  	
  	/// Number of rows and columns in the band part
  	int nb;
  	
  	/// Half the band width
  	int hbw;
  	
  	/// Number of elements allocated for A^T A
  	int ata_num_elements;
  	
  	/// Use of a sparse matrix for storing the normal matrix
	bool sparse;
  	
  	/// Column index of first non-zero element of (partial) band matrix of a row
  	int *ata_first_column_of_row;
  	
  	/// Pointer to first non-zero element of (partial) band matrix of a row
  	double **ata_first_element_of_row; 
  	
  	/// Switch to indicate whether ATA has been inverted or not
  	bool ata_inverted;
  	
  	/// Switch to indicate whether the inverted ATA has been converted to correlations
  	bool ata_contains_correlations;
  
  private:
  	/// Retrieve pointer to an element in a sparse ATA matrix
  	double *Sparse_ATA_Element(int row, int column);
  	
  	/// Retrieve value of an element in a sparse ATA matrix
  	double Sparse_ATA_Value(int row, int column);
  	
  	/// Solve the normal equation system with a sparse ATA matrix
  	void SolveSparse();

  public:

    /// Default constructor
    NormalEquations();

    /// Constructor of a normal equation system with a (partial) band
    /** Construction of a normal equation system with a (partial) band.
        Optionally, a sparse matrix can be used to store the normal matrix.
        Currently, inversion of sparse matrices is not implemented.
        If the number of rows in the band part is zero, the matrix is a full
        matrix and no sparse matrix will be used.
        @param num_par Number of parameters (dimension of normal matrix)
        @param error 0 - success, 1 - allocation error, 2 - invalid parameters
        @param num_band Number of parameters in the band part
        @param half_band_width Half the width of the band part
        @param sp Boolean to indicate use of sparse matrix storage
    **/
    NormalEquations(int num_par, int &error, int num_band=0,
	                int half_band_width=0, bool sp=false);
    
    
    /// Default destructor
    ~NormalEquations() {FreeMemory();}
	
	/// Initialise a normal equation system
    /** Initialisation of a normal equation system with a (partial) band.
        Optionally, a sparse matrix can be used to store the normal matrix.
        Currently, inversion of sparse matrices is not implemented.
        @param num_par Number of parameters (dimension of normal matrix)
        @param num_band Number of parameters in the band part
        @param half_band_width Half the width of the band part
        @param sp Boolean to indicate use of sparse matrix storage
        @return 0 - success, 1 - allocation error, 2 - invalid parameters
    **/
    int Initialise(int num_par, int num_band=0, int half_band_width=0,
	               bool sp=false);
	               

    /// Copy assignment
	NormalEquations& operator = (const NormalEquations &neq);

	/// Clear the equation system
	void ClearEquations();
    
    /// Free all allocated memory
    void FreeMemory();
    
    /// Add observation to the equation system
    /** Add an equation to the normal equation system. This function
        can only be used for full (i.e. non sparse) normal equation systems.
        @param a Pointer to a row of the design matrix (partial derivatives)
        @param y Value of the observation
        @param w Weight of the observation
        @return 0 - success, 1 - error: sparse matrix
    **/
    int AddObservation(double *a, double y, double w);
    
    /// Add observation to the equation system
    /** Add an equation to the normal equation system. Only the elements within
        the indicated range are updated. Other elements are assumed to be zero.
        @param a Pointer to a row of the design matrix (partial derivatives)
        @param y Value of the observation
        @param w Weight of the observation
        @param start1, end1 The range of "a" to be used in the update
        @return 0 - success, 1 - range outside (sparse) matrix bounds
    **/
    int AddObservation(double *a, double y, double w, int start1, int end1);
    
    /// Add observation to the equation system
    /** Add an equation to the normal equation system. Only the elements within
        the indicated ranges are updated. Other elements are assumed to be zero.
        @param a Pointer to a row of the design matrix (partial derivatives)
        @param y Value of the observation
        @param w Weight of the observation
        @param start1, end1 The first range of "a" to be used in the update
        @param start2, end2 The second range of "a" to be used in the update
        @return 0 - success, 1 - range outside (sparse) matrix bounds
    **/
    int AddObservation(double *a, double y, double w, int start1, int end1,
	                   int start2, int end2);
    
    /// Add observation to the equation system
    /** Add an equation to the normal equation system. Only the elements within
        the indicated ranges are updated. Other elements are assumed to be zero.
        @param a Pointer to a row of the design matrix (partial derivatives)
        @param y Value of the observation
        @param w Weight of the observation
        @param start1, end1 The first range of "a" to be used in the update
        @param start2, end2 The second range of "a" to be used in the update
        @param start3, end3 The third range of "a" to be used in the update
        @return 0 - success, 1 - range outside (sparse) matrix bounds
    **/
    int AddObservation(double *a, double y, double w, int start1, int end1,
	                   int start2, int end2, int start3, int end3);
    
    /// Retrieve a pointer to an element of ATA
    /** The pointer is set to NULL if the element is outside the bounds of
        ATA or outside the band of a sparse ATA.
        @param row Row of the requested element
        @param column Column of the requested element
        @return Pointer to the requested element or NULL in case of an illegal access
    **/
    double * ATA_Element(int row, int column);
    
    /// Retrieve a value of an element of ATA
    /** The value 0.0 is returned if the element is outside the bounds of
        ATA or outside the band of a sparse ATA.
        @param row Row of the requested element
        @param column Column of the requested element
        @return Value of the requested element or 0.0 in case of an illegal access
    **/
    double ATA_Value(int row, int column);
    
    /// Solve the normal equation system using LINPACK functions
    /** This function is only available for full normal matrices (not for sparse
        ones. The condition value indicates how close the normal matrix is to
        a rank defect.
        @param cond Condition value
        @return Pointer to the vector of estimated parameters. If the normal
                matrix is sparse, NULL will be returned.
    **/
    double * Solve(double &cond);
    
    /// Invert and solve the normal equation system using LINPACK functions
    /** This function is only available for full normal matrices (not for sparse
        ones. The condition value indicates how close the normal matrix is to
        a rank defect. After execution of this function the ATA matrix contains
        the covariance matrix of the estimated parameters and can be retrieved
        by the function ATA_Element(0,0) or CovarianceMatrix().
        @param cond Condition value
        @return Pointer to the vector of estimated parameters. If the normal
                matrix is sparse, NULL will be returned.
    **/
    double * InvertAndSolve(double &cond);
    
    /// Return the covariance matrix
    /** This function can only be used after execution of InvertAndSolved. If
        InvertAndSolved has not been used or after a call to ConvertToCorrelations,
		NULL will be returned.
        @return Pointer to the covariance matrix or NULL (see above).
    **/
    double *CovarianceMatrix();
    
    /// Solve the normal equation system using Cholesky decomposition
    /** This function is available for both full and sparse matrices. The
        normal matrix is not inverted. Hence, covariances cannot be obtained.
        @return Pointer to the vector of estimated parameters.
    **/
    double * Solve();
    
    /// Convert the inverted normal matrix to correlation coefficients
    /** This function can only be invoked after the use of InvertAndSolve as it
        requires a full inverted ATA. ATA will be overwritten with the
        correlation coefficients.
        @return Pointer to matrix with correlation coefficients. If InvertAndSolve
                has not been used NULL will be returned.
    **/
    double *ConvertToCorrelations();
    
    /// Return the correlation coefficients
    /** This function can only be invoked after the use of ConvertToCorrelations.
        @return Pointer to matrix with correlation coefficients. If correlations
                have not yet been computed, NULL is returned.
    **/
    double *CorrelationCoefficients();
};
#endif /* _NormalEquations_h_ */   /* Do NOT add anything after this line */

                                 

