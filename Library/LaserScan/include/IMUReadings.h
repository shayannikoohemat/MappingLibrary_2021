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

#ifndef _IMUReadings_h_
#define _IMUReadings_h_

/*
--------------------------------------------------------------------------------
This file contains the definitions of the following classes:

  IMUReading    - Read angular velocities and accelerations of an IMU

--------------------------------------------------------------------------------
*/
#include "IMUReading.h"
#include "LaserMetaFile.h"
#include <vector>

//------------------------------------------------------------------------------
///                         IMU readings
//------------------------------------------------------------------------------

class IMUReadings : public std::vector<IMUReading>, public LaserMetaFile
{
  protected:

    /// File name
    char *imu_file;

    /// Start time of IMU readings set
    double start_time;
    
    /// End time of IMU readings set
    double end_time;

  public:
    /// Default constructor
    IMUReadings() { Initialise(); }
    
    /// Copy constructor
    IMUReadings(const IMUReadings &);
    
    /// Construct by reading from meta data file
    IMUReadings(const char *filename, int *success)
      {Initialise(); *success = ReadMetaData(filename);}

    /// Default destructor
    ~IMUReadings() {};

    /// Return the reference
    IMUReadings & IMUReadingsRef()
      {return(*this);}

    /// Return the const reference
    const IMUReadings & IMUReadingsRef() const
      {return(*this);}

    /// Copy assignment
    IMUReadings operator = (const IMUReadings &r);

    /// Initialisation of new instant
    void Initialise();

    /// Reinitialisation of old instant
    void ReInitialise();

    /// Read the IMU readings from a file
    int Read(const char *filename);

    /// Read the scan lines from "imu_file"
    int Read();

    /// Write the IMU readings to a file
    /** @param filename File for the IMU readings. This file name is
                        also stored in the class variable imu_file.
        @return 1 for success, 0 for failure
    */
    int Write(const char *filename);

    /// Write the IMU readings to "imu_file"
    /** @return 1 for success, 0 for failure
    */
    int Write() const;

    /// Set file name of IMU readings
    void SetIMUFile(const char *filename);

    /// Return file name of IMU readings
    char *IMUFile() const
      {return(imu_file);}
      
    /// Set the time of the first and last IMU reading
    void SetTimeRange(double t1, double t2)
      {start_time = t1; end_time = t2;}

    /// Return start time
    double StartTime() const
      {return start_time;}
      
    /// Return end time
    double EndTime() const
      {return end_time;}
      
    /// Read meta data from "meta_file"
    int ReadMetaData();

    /// Read meta data from file
    int ReadMetaData(const char *filename);
    
    /// Read meta data from file descriptor
    int ReadMetaData(FILE *fd);

    /// Write meta data to "meta_file"
    int WriteMetaData() const;

    /// Write meta data to file
    int WriteMetaData(const char *filename) const;

    /// Write meta data file to file descriptor
    int WriteMetaData(FILE *fd) const;

    /// Derive name of imu data set in case of multiple sets
    char *DeriveName(const char *parentname, IMUReadings *first_part);

    /// Derive name of imu data file
    char *DeriveIMUDataFileName(const char *directory);

    /// Derive name of meta data file
    char *DeriveMetaDataFileName(const char *directory);
    
    /// Erase all IMU readings
    void EraseData()
      {erase(begin(), end());}
};

#endif /* _IMUReadings_h_ */  /* Don't add after this point */
