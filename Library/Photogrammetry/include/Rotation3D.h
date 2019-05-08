
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
 * \brief Interface to Class Rotation3D
 *
 */
/*!
 * \class Rotation3D
 * \ingroup LinearAlgebra
 * \brief Interface to Class Rotation3D
 *
 * <table width="100%" border=0 cellpadding=0 cellspacing=2 bgcolor="#eeeeee"><tr><td>
 *
 * \author        \Pierre_Ermes
 * \date		January 1998 (Created)
 *
 * \remark \li Define base geometries, Vector3D, Position3D, Rotation and Orientation3D.
 *
 * \todo None
 *
 * \bug None
 *
 * \warning None
 *
 *
 */


#ifndef _ROTATION3D_H_
#define _ROTATION3D_H_ 
/*--------------------------------------------------------------------
*   Project   : STW, close range photogrammetry, piping installations
*
*   File made : januari 1998
*   Author    : Pierre Ermes
*	Modified  :
*   Purpose   : Define base geometries, Vector3D, Position3D, Rotation and 
*	Orientation3D.
*
*--------------------------------------------------------------------*/

#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include <iostream>
#include <list>

#include "Matrix3.h"
#include "Vector3D.h"

//class Orientation3D;
class GenericRotation;


/// A rotation matrix for a three-dimensional coordinate system

class Rotation3D : public Matrix3 {

public:
	/// Construct with identity matrix.
	Rotation3D();

	/// Construct from specified Euler angles
	Rotation3D(double omega, double phi, double kappa);

	/// Construct from rotation axis and rotation angle
	Rotation3D(double x, double y, double z, double angle);

	/// Construct from rotation axis and rotation angle
	Rotation3D(const Vector3D &axis, double angle);

	/// Construct from the specified elements
	Rotation3D(double r11, double r12, double r13,
	     	   double r21, double r22, double r23,
		   double r31, double r32, double r33);

	/// Copy constructor
	Rotation3D(const Rotation3D &p);

	/// Construct with initialisation from parameterized rotation.
	Rotation3D(const GenericRotation &gr);

	/// Default destructor
	~Rotation3D() {}

	/// Copy assignment
	Rotation3D &operator = (const Rotation3D &rot)
	{ Matrix3Ref() = rot.Matrix3Ref();
	  return *this; }

	/// Return readable reference
	const Rotation3D &rotation() const { return *this; }

	/// Return writable reference
	Rotation3D &rotation() { return *this; }

	/// Derive the transposed rotation matrix
	Rotation3D Transpose() const;

	/// Element access, read only.
	double R(int row, int column) const { return m[row][column]; }

	/// Element access, writable
	double &R(int row, int column) { return m[row][column]; }

	/// Assignment from a parameterized rotation.
	Rotation3D operator = (const GenericRotation &r);

	/// Convert rotation matrix to angle axis with rotation angle.
	void as_axis(Vector3D &axis, double &angle) const;

    /// Multiply with another 3D rotation matrix
//	const Rotation3D &operator *= (const Rotation3D &p);

    /// Output of a rotation matrix
	friend std::ostream &operator<<(std::ostream &os, const Rotation3D &p);

    /// Multiply two rotation matrices
	friend Rotation3D operator *(const Rotation3D &r1, const Rotation3D &r2);

    /// Multiply a 3D rotation matrix with a 3D vector
	friend Vector3D operator *(const Rotation3D &r1, const Vector3D &p1);

    /// Multiply a 3D rotation matrix with an arbitrary 3x3 matrix
    friend Matrix3 operator *(const Rotation3D &r, const Matrix3 &m);
    
    /// Multiply an arbitrary 3x3 matrix with a 3D rotation matrix
    friend Matrix3 operator *(const Matrix3 &m, const Rotation3D &r);
    
    /// Rotate a 3D vector
	Vector3D Rotate(const Vector3D &p1) const
          {return(*this * p1);}
    
    /// Derive the angles from the rotation matrix elements
    void DeriveAngles(double &omega, double &phi, double &kappa) const;
    
    /// Partial derivative of rotation matrix w.r.t. one of the angles
    /// Param 0=omega, 1=phi, 2=kappa
    Rotation3D PartialDeriv(int param) const;
};


#endif /* _ROTATION3D_H_ */
