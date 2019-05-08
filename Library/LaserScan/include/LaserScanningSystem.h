
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


#ifndef _LaserScanningSystem_h_
#define _LaserScanningSystem_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  LaserScanningSystem           - System of multiple 2D laser scanners

--------------------------------------------------------------------------------
*/

#include <vector>
#include "LaserScanner.h"

//------------------------------------------------------------------------------
///                  System of multiple 2D laser scanners
//------------------------------------------------------------------------------

class LaserScanningSystem : public std::vector<LaserScanner>
{
  protected:

    /// Name of the file with the scanner information
    char *meta_file;
    
    /// Name of the scanning system
    char *name;

  public:

    /// Default constructor
    LaserScanningSystem()
      {Initialise();}

    /// Default destructor
    ~LaserScanningSystem() {};

    /// Return the reference
    LaserScanningSystem & LaserScanningSystemRef()
      {return(*this);}

    /// Return the const reference
    const LaserScanningSystem & LaserScanningSystemRef() const
      {return(*this);}

    /// Copy assignment
    LaserScanningSystem operator = (const LaserScanningSystem &system);

    /// Initialisation of new instant
    void Initialise();

    /// Reinitialisation of old instant
    void ReInitialise();

    /// Read the scanning system information from a file
    int Read(const char *filename);

    /// Write the scanning system information to a file
    /** @param filename File for the scanline information. This file name is
                        also stored in the class variable scanline_file.
        @return 1 for success, 0 for failure
    */
    int Write(const char *filename);

    /// Write the scanning system information to "meta_file"
    /** @return 1 for success, 0 for failure
    */
    int Write(int compress) const;

    /// Write the scan lines to "scanline_file" and compress it
    /** @return 1 for success, 0 for failure
    */
    int Write() const;
};

#endif /* _LaserScanningSystem_h_ */  /* Don't add after this point */
