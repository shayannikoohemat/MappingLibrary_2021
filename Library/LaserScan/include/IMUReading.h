/*
                  Copyright 2018 University of Twente
 
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

#ifndef _IMUReading_h_
#define _IMUReading_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  IMUReading    - Read angular velocities and accelerations of an IMU

--------------------------------------------------------------------------------
*/
#include <ostream>
#include "Vector3D.h"

//------------------------------------------------------------------------------
///                         IMU reading
//------------------------------------------------------------------------------

class IMUReading
{
  protected:

    /// Time
    double time;

    /// Vector of angular velocities
    Vector3D angular_velocities;

    /// Vector of position accelerations
    Vector3D accelerations;

  public:
    /// Default constructor
    IMUReading()
      { time = 0.0; angular_velocities = Vector3D();
        accelerations = Vector3D(); }

    /// Construct reading
    IMUReading(double t, double avx, double avy, double avz, 
               double ax, double ay, double az)
      { time = t; angular_velocities = Vector3D(avx, avy, avz);
        accelerations = Vector3D(ax, ay, az); }

    /// Copy constructor
    IMUReading(const IMUReading &r);
	
    /// Default destructor
    ~IMUReading() {};

    /// Copy assignment
    IMUReading & operator = (const IMUReading &r);
	
    /// Return readable time
    double Time() const
      { return time; }

    /// Return writable time
    double &Time()
      { return time; }

    /// Return readable angular velocities
    const Vector3D AngularVelocities() const
      { return angular_velocities; }

    /// Return writable angular velocities
    Vector3D &AngularVelocities()
      { return angular_velocities; }

    /// Return readable accelerations
    const Vector3D Accelerations() const
      { return accelerations; }

    /// Return writable accelerations
    Vector3D &Accelerations()
      { return accelerations; }
};

#endif /* _IMUReading_h_ */  /* Don't add after this point */
