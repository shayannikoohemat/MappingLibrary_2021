
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



/*
--------------------------------------------------------------------------------
 Collection of functions for class IMUReading

 Initial creation
 Author : George Vosselman
 Date   : 09-12-2018

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

#include <stdio.h>
#include <stdlib.h>
#include "IMUReading.h"

/*
--------------------------------------------------------------------------------
                        Copy constructor and assignment
--------------------------------------------------------------------------------
*/

/// Copy constructor
IMUReading::IMUReading(const IMUReading &r)
{
  *this = r;
}

/// Copy assignment
IMUReading & IMUReading::operator = (const IMUReading &r)
{
  if (this == &r) return *this; // Don't copy onto itself
  time = r.time;
  angular_velocities = r.angular_velocities;
  accelerations = r.accelerations;
  return *this;
}

