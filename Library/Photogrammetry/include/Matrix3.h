
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



/*!
 * \file
 * \brief Interface to Class Matrix3 - A 3x3 matrix class
 *
 */
/*!
 * \class Matrix3
 * \ingroup LinearAlgebra
 * \brief Interface to Class Matrix3 - A 3x3 matrix class
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \Ildiko_Suveg
 * \date		---- (Created)
 *
 * \remark \li None
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */

#ifndef _Matrix_h_
#define _Matrix_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following class:

  Matrix3  - A 3x3 matrix class
  
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                                Include files
--------------------------------------------------------------------------------
*/

#include <iostream>
class Vector2D;
class Vector3D;

/*
--------------------------------------------------------------------------------
                                 3x3 Matrix
--------------------------------------------------------------------------------
*/


class Matrix3
{
protected:
	double m[3][3];
	
public:
	/// Default constructor, all elements initialised to zero
	Matrix3()
	  {
	    m[0][0] = 0; m[0][1] = 0; m[0][2] = 0; 
	    m[1][0] = 0; m[1][1] = 0; m[1][2] = 0; 
	    m[2][0] = 0; m[2][1] = 0; m[2][2] = 0;
	  } 
		
	/// Construct with user specified matrix elements
	Matrix3(double a11, double a12, double a13, 
	         double a21, double a22, double a23,
	         double a31, double a32, double a33) 	
	  { m[0][0] = a11; m[0][1] = a12; m[0][2] = a13; 
	    m[1][0] = a21; m[1][1] = a22; m[1][2] = a23; 
	    m[2][0] = a31; m[2][1] = a32; m[2][2] = a33;
	  }

    /// Construct from a two dimensional 3x3 double array
    Matrix3(double[3][3]);

    /// Construct from a double array
	Matrix3(double *);	  
	  				
	/// Copy assignment
	Matrix3 &operator=(const Matrix3&);
	  
	/// Default destructor
	~Matrix3() {}
	
    /// Return the readable reference
    const Matrix3 &Matrix3Ref() const
      { return *this; }

    /// Return the writable reference
    Matrix3 &Matrix3Ref()
      { return *this; }

	// Return the determinant
	double Det();
	
	/// Return the transposed matrix	
	Matrix3 Transpose() const;

	/// Trace of the matrix.
	double Trace() const { return m[0][0] + m[1][1] + m[2][2]; }

	/// Return the inverted matrix
	Matrix3 Inverse();
	
	/// Return the null space
	Vector2D Nullspace();

	/// Return a row as a vector.
	Vector3D Row(int row) const;
	
	/// Return a column as a vector.
	Vector3D Column(int column) const;
	
	// Print the matrix to stdout
	void Print() const;
	
	// Print the matrix to std::ostream
	void Print(std::ostream &os) const;
	
	/// Multiply the matrix with a constant
	friend Matrix3 operator*(double, const Matrix3 &);
	
	/// Multiply two matrices
	friend Matrix3 operator*(const Matrix3 &, const Matrix3 &);
	
	/// Add two matrices
	friend Matrix3 operator+(const Matrix3 &, const Matrix3 &);
	
	/// Multiply a matrix with a vector
	friend Vector3D operator*(const Matrix3 &, const Vector3D &); 

};

#endif /* _Matrix_h_ */   /* Do NOT add anything after this line */
