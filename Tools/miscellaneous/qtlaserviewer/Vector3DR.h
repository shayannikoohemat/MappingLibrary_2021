
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


/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : Feb 2006
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : A child of Vector3D that also has a reflectance value.
*
*--------------------------------------------------------------------*/
#ifndef __VECTOR3D_R____H__
#define __VECTOR3D_R____H__
#include <Vector3D.h>
#include <LaserPoint.h>

///A child of Vector3D that also has a reflectance value.
///For keeping stxxl happy as LaserPoint has a pointer which cannot be safely saved on a file.
class Vector3DR:public Vector3D
{
public:
	Vector3DR(double x=0,double y=0, double z=0, double ref=0)
	:Vector3D(x,y,z),reflectance(ref)
	{
	}
	
	Vector3DR(const Vector3D& v,double ref=0)
	:Vector3D(v),reflectance(ref)
	{
	}
	
	LaserPoint operator()()const
	{
		return LaserPoint(X(),Y(),Z(),reflectance);
	}
	double reflectance;
};


#endif //__VECTOR3D_R____H__