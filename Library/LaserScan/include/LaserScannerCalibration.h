
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

#ifndef _LaserScannerCalibration_h_
#define _LaserScannerCalibration_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

 LaserScannerCalibration          - Calibration parameters of a laser scanner

--------------------------------------------------------------------------------
*/

#include "BNF_io.h"

//------------------------------------------------------------------------------
///                     Laser scanner calibration parameters
//------------------------------------------------------------------------------

class LaserScannerCalibration
{
  protected:

    /// Range offset
    double range_offset;

    /// Range scale
    double range_scale;
    
    /// Angle scale
    double angle_scale;
    
    /// Calibration data
    char *date;

  public:

    /// Default constructor
    LaserScannerCalibration()
      {Initialise();}
      
    /// Construct by reading
    LaserScannerCalibration(FILE *filename)
      {Initialise(); Read(filename);}

    /// Default destructor
    ~LaserScannerCalibration() {};

    /// Copy assignment
    LaserScannerCalibration operator = (const LaserScannerCalibration &calibration);

    /// Return the const reference
    const LaserScannerCalibration &LaserScannerCalibrationRef() const
      {return(*this);}

    /// Return the reference
    LaserScannerCalibration &LaserScannerCalibrationRef()
      {return(*this);}

    /// Initialisation of a new instance
    void Initialise();

    /// Reinitialisation of an old instance
    void ReInitialise();

    /// Read scanner calibration data
    void Read(FILE *filename);

    /// Write scanner calibration data
    /** @param file File descripto
        @param indent Number of characters to indent in the BNF file
    */
    void Write(FILE *file, int indent) const;

    /// Retrieve readable range offset
    double RangeOffset() const
      {return(range_offset);}

    /// Retrieve writable range offset
    double & RangeOffset()
      {return(range_offset);}

    /// Retrieve readable range scale
    double RangeScale() const
      {return(range_scale);}

    /// Retrieve writable range scale
    double & RangeScale()
      {return(range_scale);}

    /// Retrieve readable angle scale
    double AngleScale() const
      {return(angle_scale);}

    /// Retrieve writable angle scale
    double & AngleScale()
      {return(angle_scale);}

    /// Return the date of the scanner calibration
    char *Date() const
      {return(date);}

    /// Set the calibration date to the current time
    void SetDateNow();
    
    /// Set the calibration date to a specified time
    void SetDate(char *string)
      {StringCopy(&date, string);}
    
    /// Check if some data is known
    bool HasSomeData() const;
};

#endif /* _LaserScannerCalibration_h_ */  /* Don't add after this point */
