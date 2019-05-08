
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



/*
--------------------------------------------------------------------------------
 Collection of functions for class LaserScannerCalibration

 Initial creation
 Author : George Vosselman
 Date   : 27-01-2017

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
#include "LaserScannerCalibration.h"
#include "stdmath.h"   /* Definition of MAX and MIN */
#include "Database4Cpp.h"
#include <iostream>
#include <ctime>

/*
--------------------------------------------------------------------------------
                              Copy assignment
--------------------------------------------------------------------------------
*/

LaserScannerCalibration LaserScannerCalibration::operator = (const LaserScannerCalibration &cal)
{
  if (this == &cal) return *this;  // Check for self assignment
  range_offset = cal.range_offset;
  range_scale  = cal.range_scale;
  angle_scale  = cal.angle_scale;
  StringCopy(&date, cal.date);
  return(*this);
}

/*
--------------------------------------------------------------------------------
                  Initialise the laser scanner calibration
--------------------------------------------------------------------------------
*/

void LaserScannerCalibration::Initialise()
{
  range_offset = 0.0;
  range_scale  = 1.0;
  angle_scale  = 1.0;
  date   = NULL;
}

void LaserScannerCalibration::ReInitialise()
{
  if (date) free(date);
  Initialise();
}


/*
--------------------------------------------------------------------------------
              Read scanner calibration data from file in BNF format
--------------------------------------------------------------------------------
*/

void LaserScannerCalibration::Read(FILE *fd)
{
  char *buffer, *line, *keyword;
  int  keyword_length;

  Initialise();
  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        if (!strncmp(keyword, "rangeoffset", MAX(keyword_length, 11)))
          range_offset = BNF_Double(line);

        else if (!strncmp(keyword, "rangescale", MAX(keyword_length, 10)))
          range_scale = BNF_Double(line);

        else if (!strncmp(keyword, "anglescale", MAX(keyword_length, 10)))
          angle_scale = BNF_Double(line);

        else if (!strncmp(keyword, "date", MAX(keyword_length, 4)))
          date = BNF_String(line);

        else if (!strncmp(keyword, "endlaserscannercalibration", MAX(keyword_length, 26))) {
          free(buffer);
          return;
      }
      else
        fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
      }
    }
  }
  fprintf(stderr, "Error: Did not find keyword endlaserscannercalibration.\n");
}

/*
--------------------------------------------------------------------------------
                Write scanner calibration to file in BNF format
--------------------------------------------------------------------------------
*/

bool LaserScannerCalibration::HasSomeData() const
{
  return(range_offset != 0.0 || range_scale != 1.0 || angle_scale != 1.0 || date);
}

void LaserScannerCalibration::Write(FILE *fd, int indent) const
{
  if (!HasSomeData()) return;
  BNF_Write_String(fd, "laserscannercalibration", indent, NULL);
  if (range_offset != 0.0)
    BNF_Write_Double(fd, "rangeoffset", indent+2, range_offset, "%13.8f");
  if (range_scale != 1.0)
    BNF_Write_Double(fd, "rangescale", indent+2, range_scale, "%13.8f");
  if (angle_scale != 1.0)
    BNF_Write_Double(fd, "anglescale", indent+2, angle_scale, "%13.8f");
  if (date)
    BNF_Write_String(fd, "date", indent+2, date);
  BNF_Write_String(fd, "endlaserscannercalibration", indent, NULL);
}

/*
--------------------------------------------------------------------------------
                Set date
--------------------------------------------------------------------------------
*/

void LaserScannerCalibration::SetDateNow()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer, 80, "%a %d-%m-%Y %H:%M:%S", timeinfo);
  StringCopy(&date, buffer);
}
