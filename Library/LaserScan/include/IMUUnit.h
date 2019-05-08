
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

#ifndef _IMUUnit_h_
#define _IMUUnit_h_

/*
--------------------------------------------------------------------------------
 This file contains the definitions of the following classes:

 IMUUnit - Meta data for a set of files with IMU data

 Initial creation
 Author : George Vosselman
 Date   : 11-12-2018

 Update #1
 Author : 
 Date   : 
 Changes: 

--------------------------------------------------------------------------------
*/

#include <vector>
#include "IMUReadings.h"

//------------------------------------------------------------------------------
///                   Meta data for a set of files with IMU data
//------------------------------------------------------------------------------

class IMUUnit: public std::vector<IMUReadings>, public LaserMetaFile
{
  protected:
    // Currently no local variables
    
  public:

    /// Default constructor
    IMUUnit()
      {Initialise();}

    /// Construct by reading from meta data file
    IMUUnit(const char *filename, int *success, bool read_subunits=true)
      {Initialise(); *success = ReadMetaData(filename, read_subunits);}

    /// Copy constructor
    IMUUnit(const IMUUnit &unit);

    /// Copy assignment
    IMUUnit & operator = (const IMUUnit &unit);

    /// Initialisation of a new instant
    void Initialise();

    /// Reinitialisation of an old instant
    void ReInitialise();

    /// Read meta data
    int ReadMetaData(const char *filename, bool read_subunits=true);
  
    /// Write meta data to "meta_file"
    int WriteMetaData() const;

    /// Write meta data to a file
    int WriteMetaData(const char *filename) const;

    /// Write meta data to "meta_file" and write sub units
    /** @param write_parts If true, the meta data of the subunits is also
                           written
    */
    int WriteMetaData(int write_parts) const;

    /// Write meta data to a file and write sub units
    /** @param filename Name of the meta data file
        @param write_parts If true, the meta data of the subunits is also
                           written
    */
    int WriteMetaData(const char *filename, int write_parts) const;

    /// Derive the file name of the meta data
    char *DeriveMetaDataFileName(const char *directory);
};

#endif /* _IMUUnit_h_ */  /* Don't add after this point */
