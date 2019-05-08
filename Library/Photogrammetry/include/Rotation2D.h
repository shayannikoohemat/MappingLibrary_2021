
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


#ifndef _Rotation2D_h_
#define _Rotation2D_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  Rotation2D         - A rotation matrix in a two-dimensional coordinate system

--------------------------------------------------------------------------------
*/

#include "Vector2D.h"

//------------------------------------------------------------------------------
/// A rotation matrix for a two-dimensional coordinate system
//------------------------------------------------------------------------------

class Rotation2D {

  protected:

    /// The elements of the rotation matrix
    double r[2][2];

  public:

    /// Default constructor (unit matrix)
    Rotation2D();

	/// Construct from a specified angle
	Rotation2D(double angle);

	/// Copy constructor
	Rotation2D(const Rotation2D &p);

    /// Default destructor
    ~Rotation2D() {};

	/// Copy assignment
	Rotation2D &operator = (const Rotation2D &rot)
	{int i,j;
	for (i=0; i<2; i++)
	  for (j=0; j<2; j++) r[i][j] = rot.r[i][j];
	return(*this);
	}

	/// Return readable reference
	const Rotation2D &rotation() const { return *this; }

	/// Return writable reference
	Rotation2D &rotation() { return *this; }

    /// Retrieve rotation angle from rotation matrix
    double DeriveAngle() const;
    
    /// Partial derivative of rotation matrix w.r.t. the angle
    Rotation2D PartialDeriv() const;

	/// Derive the transposed rotation matrix
	Rotation2D Transpose() const;

    /// Trace of rotation matrix.
    double Trace() const { return r[0][0] + r[1][1]; }

    /// Element access, read only.
    double R(int row, int column) const { return r[row][column]; }

    /// Element access, writable
    double &R(int row, int column) { return r[row][column]; }
    
    /// Multiply two rotation matrices
	friend Rotation2D operator *(const Rotation2D &r1,const Rotation2D &r2);

    /// Multiply a 2D rotation matrix with a 2D vector
	friend Vector2D operator *(const Rotation2D &r1,const Vector2D &p1);

    /// Retrieve a matrix row
    Vector2D Row(int i) const;
	
	/// Retrieve a matrix column
    Vector2D Column(int i) const;

    /// Print the rotation matrix
    void Print(std::ostream &os) const;
 
};
#endif /* _Rotation2D_h_ */   /* Do NOT add anything after this line */
