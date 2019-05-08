
/*
                  Copyright 2017 University of Twente 
 
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

#include <math.h>
#include "Rotation2D.h"

// Rotation constructors ********************************************

Rotation2D::Rotation2D() {
	r[0][0] = r[1][1] = 1;
	r[0][1] = r[1][0] = 0;
}

Rotation2D::Rotation2D(double angle) {
	r[0][0] =  cos(angle);
	r[0][1] = -sin(angle);
	r[1][0] =  sin(angle);
	r[1][1] =  cos(angle);
}

Rotation2D::Rotation2D(const Rotation2D &p) {
	memmove(this, &p, sizeof(Rotation2D));
}

double Rotation2D::DeriveAngle() const
{
  return atan2(r[1][0], r[1][1]);
}

Rotation2D Rotation2D::PartialDeriv() const
{
  Rotation2D rot;
  double     sina = r[1][0];
  double     cosa = r[0][0];
  
  rot.R(0,0) = -sina;
  rot.R(0,1) = -cosa;
  rot.R(1,0) =  cosa;
  rot.R(1,1) = -sina;
  return(rot);
}


// Rotation operators **************************************************

Rotation2D Rotation2D::Transpose() const {

	Rotation2D R;
	for (int i=0; i<2; i++) {
		for (int j=0; j<2; j++) {
			R.r[i][j] = r[j][i];
		}
	}
	return R;
}


Rotation2D operator *(const Rotation2D &r1, const Rotation2D &r2) {

	Rotation2D r;
	for (int i=0; i<2; i++) {
		for (int j=0; j<2; j++) {

			r.r[i][j] = 0;
			for (int k=0; k<2; k++) {
				r.r[i][j] += r1.r[i][k]*r2.r[k][j];
			}
		}
	}
	return r;
}

Vector2D operator *(const Rotation2D &r1, const Vector2D &v2) {

	Vector2D v;
	for (int i=0; i<2; i++) {
	  v.X(i) = 0;
	  for (int k=0; k<2; k++)
		v.X(i) += r1.r[i][k] * v2.X(k);
	}
	return v;
}


Vector2D Rotation2D::Row(int i) const {
	Vector2D res(r[i][0], r[i][1]);
	return res;
}

Vector2D Rotation2D::Column(int i) const {
	Vector2D res(r[0][i], r[1][i]);
	return res;
}

void Rotation2D::Print(std::ostream &os) const {

	std::cerr << r[0][0] <<",  "<< r[0][1] << std::endl;
	std::cerr << r[1][0] <<",  "<< r[1][1] << std::endl;
}

