
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
 Collection of functions for class IMUUnit

 Initial creation
 Author : George Vosselman
 Date   : 11-12-2018

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
#include <string.h>
#include <strings.h>
#include "IMUUnit.h"
#include "BNF_io.h"
#include "stdmath.h"
#include "Database4Cpp.h"

/*
--------------------------------------------------------------------------------
                     Copy constructor and assignment
--------------------------------------------------------------------------------
*/

IMUUnit::IMUUnit(const IMUUnit &unit)
{
  Initialise();
  if (!empty()) erase(begin(), end());
  if (!unit.empty()) insert(begin(), unit.begin(), unit.end());
  LaserMetaFileReference() = unit.LaserMetaFileReference();
}

IMUUnit & IMUUnit::operator = (const IMUUnit &unit)
{
  if (this == &unit) return *this;  // Check for self assignment
  if (!empty()) erase(begin(), end());
  if (!unit.empty()) insert(begin(), unit.begin(), unit.end());
  LaserMetaFileReference() = unit.LaserMetaFileReference();
  return(*this);
}

/*
--------------------------------------------------------------------------------
                         Initialise the meta data
--------------------------------------------------------------------------------
*/

void IMUUnit::Initialise()
{
  LaserMetaFile::Initialise();
}

void IMUUnit::ReInitialise()
{
  if (!empty()) erase(begin(), end());  /* Delete old sub units */
  LaserMetaFile::ReInitialise();
}

/*
--------------------------------------------------------------------------------
                       Read the unit meta data
--------------------------------------------------------------------------------
*/

int IMUUnit::ReadMetaData(const char *filename, bool read_subunits)
{
  FILE           *fd;
  char           *buffer, *line, *keyword, *file;
  int            keyword_length, success;
  
  // Open the file
  fd = Open_Compressed_File(filename, "r");
  if (fd == NULL) {
    fprintf(stderr, "Error opening IMU unit meta data file %s\n", filename);
    return(0);
  }

  // Verify that the first keyword is "imuunit" */

  buffer = (char *) malloc(MAXCHARS);
  do {
    line = fgets(buffer, MAXCHARS, fd);
  } while (line && Is_Comment(line));
  if (line == NULL) {
    fprintf(stderr, "Error reading IMU unit meta data file %s\n", filename);
    fclose(fd);
    return(0);
  }
  keyword = BNF_KeyWord(line, &keyword_length);
  if (strncmp(keyword, "imuunit", MAX(keyword_length, 7))) {
    fprintf(stderr,"Error: File %s is not an IMU unit meta data file.\n", filename);
    fprintf(stderr,"       First keyword is %s, not imuunit.\n", keyword);
    exit(0);
  }

  // Initialise all meta data
  ReInitialise();

  // Process all lines
  SetMetaDataFile(filename);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
	
        if (!strncmp(keyword, "name", MAX(keyword_length, 4)))
          name = BNF_String(line);		

        else if (!strncmp(keyword, "imupart", MAX(keyword_length, 7))) {
          file = BNF_String(line);
          if (file) {
            if (read_subunits) {
              push_back(IMUReadings(file, &success));
              if (!success) erase(end()-1); 
            }
            else {
              push_back(IMUReadings());
              (end()-1)->SetMetaDataFile(file);
            }
          }
        }
        
        else if (!strncmp(keyword, "endimuunit", MAX(keyword_length,10))) {
          free(buffer);
          fclose(fd);
          return(1);
        }

        else {
          keyword[keyword_length] = 0;
          fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
        }
      }
    }
  }
  fprintf(stderr, "Error: Did not find keyword endimuunit.\n");
  fclose(fd);
  return(0);
}


/*
--------------------------------------------------------------------------------
                   Write the unit meta data to file
--------------------------------------------------------------------------------
*/

int IMUUnit::WriteMetaData() const
{
  return(WriteMetaData(meta_file, 0));
}

int IMUUnit::WriteMetaData(const char *filename) const
{
  SetMetaDataFile(filename);
  return(WriteMetaData(filename, 0));
}

int IMUUnit::WriteMetaData(int write_sub_units) const
{
  return(WriteMetaData(meta_file, write_sub_units));
}

int IMUUnit::WriteMetaData(const char *filename, int write_sub_units) const
{
  FILE       *fd;
  char       *file;
  int        indent;
  IMUUnit::const_iterator subunit;

  // Open the file
  fd = fopen(filename, "w");
  if (fd == NULL) {
    fprintf(stderr, "Error opening IMU unit meta data file %s\n", filename);
    return(0);
  }

  // Write the meta data
  indent = 0;
  BNF_Write_String(fd, "imuunit", indent, NULL);
  indent += 2;
  if (name) BNF_Write_String(fd, "name", indent, name);
  for (subunit=begin(); subunit!=end(); subunit++) {
    BNF_Write_String(fd, "imupart", indent, subunit->MetaDataFile());
    if (write_sub_units) (void) subunit->WriteMetaData();
  }
  indent -= 2;
  BNF_Write_String(fd, "endimuunit", indent, NULL);

  // Close the file and return
  fclose(fd);
  return(1);
}

/*
--------------------------------------------------------------------------------
                     Derive and set meta data file name
--------------------------------------------------------------------------------
*/

char *IMUUnit::DeriveMetaDataFileName(const char *directory)
{
  LaserMetaFile::DeriveMetaDataFileName(directory, ".imuunit");
}
