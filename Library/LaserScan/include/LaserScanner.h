
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

#ifndef _LaserScanner_h_
#define _LaserScanner_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

 LaserScanner                          - Characteristics of laser scanner

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                               Type definitions
--------------------------------------------------------------------------------
*/

/** Type of laser point.
    Possible types are:
         UnknownPoint,
         NormalPoint (X, Y, Z),
         ReflectancePoint (X, Y, Z, reflectance),
         MultiPoint (X, Y, Z, pulse count), 
         MultiReflectancePoint (X, Y, Z, reflectance, pulse count),
         ColourPoint (X, Y, Z, RGB), and
         MultiColourPoint (X, Y, Z, RGB, pulse count)
*/
    
enum LaserPointType { UnknownPoint=-1,
                      NormalPoint,
                      ReflectancePoint,
                      MultiPoint,
                      MultiReflectancePoint,
                      ColourPoint,
                      MultiColourPoint};
typedef enum LaserPointType LaserPointType;

//------------------------------------------------------------------------------
//                           Include files
//------------------------------------------------------------------------------

#include "LaserScannerCalibration.h"
#include "LaserScannerOrientation.h"

//------------------------------------------------------------------------------
///                     Laser scanner characteristics
//------------------------------------------------------------------------------

class LaserScanner
{
  protected:

    /// Name of the laser scanner
    char *scanner_name;
    
    /// ID of laser scanner
    int id;

    /// Type of laser data
    LaserPointType point_type;

    /// Opening angle in degrees
    double opening_angle;

    /// Precision in in-line, across-line, and range direction
    double stdev[3];
    
    /// Scanner calibration data
    LaserScannerCalibration *calibration;
    
    /// Scanner orientation data
    LaserScannerOrientation *orientation;

  public:

    /// Default constructor
    LaserScanner()
      {Initialise();}

    /// Default destructor
    ~LaserScanner() {};

    /// Copy assignment
    LaserScanner operator = (const LaserScanner &scanner);

    /// Return the const reference
    const LaserScanner &LaserScannerReference() const
      {return(*this);}

    /// Return the reference
    LaserScanner &LaserScannerReference()
      {return(*this);}

    /// Initialisation of a new instant
    void Initialise();

    /// Reinitialisation of an old instant
    void ReInitialise();

    /// Read scanner data
    bool Read(const char *filename);

    /// Read scanner data
    void Read(FILE *file);

    /// Write scanner data
    /** @param file File descripto
        @param indent Number of characters to indent in the BNF file
    */
    void Write(FILE *file, int indent) const;

    /// Write scanner data
    /** @param file name
        @param indent Number of characters to indent in the BNF file
    */
    void Write(const char *filename) const;

    /// Return the name of the scanner
    char *ScannerName() const
      {return(scanner_name);};

    /// Return the id of the scanner
    int ScannerID() const
      {return(id);};

    /// Return the id of the scanner
    int &ScannerID()
      {return(id);};

    /// Return the point type
    LaserPointType PointType() const
      {return(point_type);}

    /// Set the point type
    void SetPointType(LaserPointType ptype)
      {point_type = ptype;}

    /// Return the opening angle
    double OpeningAngle() const
      {return(opening_angle);}

    /// Return the in-line precision
    double InLinePrecision() const
      {return(stdev[0]);}

    /// Return the across line precision
    double AcrossLinePrecision() const
      {return(stdev[1]);}

    /// Return the range precision (same as height precision)
    double RangePrecision() const
      {return(stdev[2]);}

    /// Return the height precision (same as range precision)
    double HeightPrecision() const
      {return(stdev[2]);}

    /// Check if some data is known
    int HasSomeData() const;
    
    /// Return the calibration data
    const LaserScannerCalibration *Calibration() const
      {return calibration;}
	      
    /// Return the calibration data
    LaserScannerCalibration *Calibration()
      {return calibration;}
	      
    /// Initialise calibration data
    void InitialiseCalibrationData();
    
	/// Return the orientation data
    const LaserScannerOrientation *Orientation() const
      {return orientation;}
	      
    /// Return the orientation data
    LaserScannerOrientation *Orientation()
      {return orientation;}

    /// Initialise orientation data
    void InitialiseOrientationData();
};

#endif /* _LaserScanner_h_ */  /* Don't add after this point */
