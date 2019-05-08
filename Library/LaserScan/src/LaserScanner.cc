
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



/*
--------------------------------------------------------------------------------
 Collection of functions for class LaserScanner

 LaserScanner LaserScanner::operator =        Copy assignment
   (const LaserScanner &)
 void LaserScanner::Initialise()              Initialisation
 void LaserScanner::ReInitialise()            Reinitialisation
 void LaserScanner::Read(FILE *)              Read scanner characteristics
 int LaserScanner::HasSomeData() const        True if some data is set
 void LaserScanner::Write(FILE *, int) const  Write scanner characteristics

 Initial creation
 Author : George Vosselman
 Date   : 13-04-1999

 Update #1
 Author : George Vosselman
 Date   : 29-01-2017
 Changes: Added calibration and orientation data and functions

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
#include "LaserScanner.h"
#include "BNF_io.h"
#include "stdmath.h"   /* Definition of MAX and MIN */
#include "Database4Cpp.h"

/*
--------------------------------------------------------------------------------
                              Copy assignment
--------------------------------------------------------------------------------
*/

LaserScanner LaserScanner::operator = (const LaserScanner &i)
{
  if (this == &i) return *this;  // Check for self assignment
  StringCopy(&scanner_name, i.scanner_name);
  id = i.id;
  point_type = i.point_type;
  opening_angle = i.opening_angle;
  for (int j=0; j<3; j++) stdev[j] = i.stdev[j];
  if (i.calibration) {
  	if (!calibration) calibration = new LaserScannerCalibration();
  	*calibration = *(i.calibration);
  }
  if (i.orientation) {
  	if (!orientation) orientation = new LaserScannerOrientation();
  	*orientation = *(i.orientation);
  }
  return(*this);
}

/*
--------------------------------------------------------------------------------
                     Initialise the laser scanner characteristics
--------------------------------------------------------------------------------
*/

void LaserScanner::Initialise()
{
  scanner_name = NULL;
  id = -1;
  point_type = UnknownPoint;
  opening_angle = -1.0;
  for (int i=0; i<3; i++) stdev[i] = -1.0;
  calibration = NULL;
  orientation = NULL;
}

void LaserScanner::ReInitialise()
{
  if (scanner_name) free(scanner_name);
  Initialise();
}

/*
--------------------------------------------------------------------------------
              Read scanner characteristics from file in BNF format
--------------------------------------------------------------------------------
*/

bool LaserScanner::Read(const char *filename)
{
  char   *buffer, *line, *keyword;
  int    keyword_length;
  FILE   *fd;
  
  fd = fopen(filename, "r");
  if (fd == NULL) return false;
  
  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        // Look for the start of the laser scanner data
        if (!strncmp(keyword, "laserscanner", 
                     MAX(keyword_length, 12))) {
          Read(fd);
          fclose(fd);
          free(buffer);
          return true;
        }
      }
    }
  }
  printf("Error: Keyword laserscanner not found in file %s\n",
         filename);
  free(buffer);
  return false;
}


void LaserScanner::Read(FILE *fd)
{
  char *buffer, *line, *keyword;
  int  keyword_length;

  Initialise();
  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        if (!strncmp(keyword, "name", MAX(keyword_length, 4)))
          scanner_name = BNF_String(line);

        else if (!strncmp(keyword, "id", MAX(keyword_length, 2)))
          id = BNF_Integer(line);

        else if (!strncmp(keyword, "point_type", MAX(keyword_length, 10)))
          point_type = (LaserPointType) BNF_Integer(line);

        else if (!strncmp(keyword, "opening_angle", MAX(keyword_length, 13)))
          opening_angle = BNF_Double(line);

        else if (!strncmp(keyword, "stdev_inline", MAX(keyword_length, 12)))
          stdev[0] = BNF_Double(line);

        else if (!strncmp(keyword, "stdev_across", MAX(keyword_length, 12)))
          stdev[1] = BNF_Double(line);

        else if (!strncmp(keyword, "stdev_height", MAX(keyword_length, 12)))
          stdev[2] = BNF_Double(line);

        else if (!strncmp(keyword, "laserscannercalibration", MAX(keyword_length, 23)))
          calibration = new LaserScannerCalibration(fd);

        else if (!strncmp(keyword, "laserscannerorientation", MAX(keyword_length, 23)))
          orientation = new LaserScannerOrientation(fd);

        else if (!strncmp(keyword, "endscanner", MAX(keyword_length, 10)) ||
		         !strncmp(keyword, "endlaserscanner", MAX(keyword_length, 15))) {
          free(buffer);
          return;
      }
      else
        fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
      }
    }
  }
  fprintf(stderr, "Error: Did not find keyword endlaserscanner.\n");
}

/*
--------------------------------------------------------------------------------
                Write scanner characteristics to file in BNF format
--------------------------------------------------------------------------------
*/

int LaserScanner::HasSomeData() const
{
  return(scanner_name || id != -1 ||
         point_type != UnknownPoint ||
         opening_angle >= 0.0 ||
         stdev[0] >= 0.0 ||
         stdev[1] >= 0.0 ||
         stdev[2] >= 0.0 || calibration || orientation);
}

void LaserScanner::Write(FILE *fd, int indent) const
{
  if (!HasSomeData()) return;
  BNF_Write_String(fd, "laserscanner", indent, NULL);
  if (scanner_name)
    BNF_Write_String(fd, "name", indent+2, scanner_name);
  if (id != -1)
    BNF_Write_Integer(fd, "id", indent+2, id, "%2d");
  if (point_type != UnknownPoint)
    BNF_Write_Integer(fd, "point_type", indent+2, (int) point_type, "%2d");
  if (opening_angle >= 0.0)
    BNF_Write_Double(fd, "opening_angle", indent+2, opening_angle, "%8.3f");
  if (stdev[0] >= 0.0)
    BNF_Write_Double(fd, "stdev_inline", indent+2, stdev[0], "%8.3f");
  if (stdev[1] >= 0.0)
    BNF_Write_Double(fd, "stdev_across", indent+2, stdev[1], "%8.3f");
  if (stdev[2] >= 0.0)
    BNF_Write_Double(fd, "stdev_across", indent+2, stdev[2], "%8.3f");
  if (calibration)
    if (calibration->HasSomeData()) calibration->Write(fd, indent+2);
  if (orientation)
    if (orientation->HasSomeData()) orientation->Write(fd, indent+2);
  BNF_Write_String(fd, "endlaserscanner", indent, NULL);
}

void LaserScanner::Write(const char *filename) const
{
  FILE *fd;
  
  fd = fopen(filename, "w");
  if (!fd) {
  	printf("Error opening file %s for storing scanner data\n", filename);
  	exit(0);
  }
  Write(fd, 0);
}

/*
--------------------------------------------------------------------------------
                Initialise calibration and orientation data
--------------------------------------------------------------------------------
*/

void LaserScanner::InitialiseCalibrationData()
{
  calibration = new LaserScannerCalibration();
}

void LaserScanner::InitialiseOrientationData()
{
  orientation = new LaserScannerOrientation();
}
