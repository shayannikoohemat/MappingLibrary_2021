
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



#ifndef _LaserScannerOrientation_h_
#define _LaserScannerOrientation_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

 LaserScannerOrientation               - Orientation of a laser scanner

--------------------------------------------------------------------------------
*/

#include "Orientation3D.h"
#include "BNF_io.h"

//------------------------------------------------------------------------------
///                     Laser scanner orientation
//------------------------------------------------------------------------------

class LaserScannerOrientation : public Orientation3D
{
  protected:

    /// Date of the orientation determination
    char *date;

  public:

    /// Default constructor
    LaserScannerOrientation()
      {Initialise();}

    /// Construct by reading
    LaserScannerOrientation(FILE *filename)
      {Initialise(); Read(filename);}

    /// Default destructor
    ~LaserScannerOrientation() {};

    /// Copy assignment
    LaserScannerOrientation operator = (const LaserScannerOrientation &orient);

    /// Return the const reference
    const LaserScannerOrientation &LaserScannerOrientationRef() const
      {return(*this);}

    /// Return the reference
    LaserScannerOrientation &LaserScannerOrientationRef()
      {return(*this);}

    /// Initialisation of a new instance
    void Initialise();

    /// Reinitialisation of an old instance
    void ReInitialise();

    /// Read scanner orientation data
    void Read(FILE *filename);

    /// Write scanner orientation data
    /** @param file File descripto
        @param indent Number of characters to indent in the BNF file
    */
    void Write(FILE *file, int indent) const;

    /// Return the date of the scanner orientation
    char *Date() const
      {return(date);};
      
    /// Set the orientation date to the current time
    void SetDateNow();
    
    /// Set the orientation date to a specified time
    void SetDate(char *string)
      {StringCopy(&date, string);}
    
    /// Check if some data is known
    bool HasSomeData() const;
    
    /// Set the orientation
    void Set(const Vector3D &offset, const Rotation3D &rot);
};

#endif /* _LaserScannerOrientation_h_ */  /* Don't add after this point */
