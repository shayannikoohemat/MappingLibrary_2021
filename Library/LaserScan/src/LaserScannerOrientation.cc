
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
 Collection of functions for class LaserScannerOrientation

 Initial creation
 Author : George Vosselman
 Date   : 21-01-2017

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
#include "LaserScannerOrientation.h"
#include "stdmath.h"   /* Definition of MAX and MIN */
#include "Database4Cpp.h"
#include <iostream>
#include <ctime>

/*
--------------------------------------------------------------------------------
                              Copy assignment
--------------------------------------------------------------------------------
*/

LaserScannerOrientation LaserScannerOrientation::operator = (const LaserScannerOrientation &orient)
{
  if (this == &orient) return *this;  // Check for self assignment
  StringCopy(&date, orient.date);
  Orientation3DRef() = orient.Orientation3DRef();
  return(*this);
}

/*
--------------------------------------------------------------------------------
                     Initialise the laser scanner characteristics
--------------------------------------------------------------------------------
*/

void LaserScannerOrientation::Initialise()
{
  date = NULL;
  x[0] = x[1] = x[2] = 0.0;  // Initialise vector
  rotation() = Rotation3D(); // Initialise rotation
}

void LaserScannerOrientation::ReInitialise()
{
  if (date) free(date);
  Initialise();
}

/*
--------------------------------------------------------------------------------
              Read scanner orientation from file in BNF format
--------------------------------------------------------------------------------
*/

void LaserScannerOrientation::Read(FILE *fd)
{
  char *buffer, *line, *keyword;
  int  keyword_length;

  Initialise();
  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        if (!strncmp(keyword, "date", MAX(keyword_length, 4)))
          date = BNF_String(line);

        else if (!strncmp(keyword, "offset", MAX(keyword_length, 6)))
          BNF_Three_Doubles(line, &(x[0]), &(x[1]), &(x[2]));

        else if (!strncmp(keyword, "rotation_row1", MAX(keyword_length, 13)))
          BNF_Three_Doubles(line, &(R(0,0)), &(R(0,1)), &(R(0,2)));

        else if (!strncmp(keyword, "rotation_row2", MAX(keyword_length, 13)))
          BNF_Three_Doubles(line, &(R(1,0)), &(R(1,1)), &(R(1,2)));

        else if (!strncmp(keyword, "rotation_row3", MAX(keyword_length, 13)))
          BNF_Three_Doubles(line, &(R(2,0)), &(R(2,1)), &(R(2,2)));

        else if (!strncmp(keyword, "endlaserscannerorientation", MAX(keyword_length, 26))) {
          free(buffer);
          return;
      }
      else
        fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
      }
    }
  }
  fprintf(stderr, "Error: Did not find keyword endlaserscannerorientation.\n");
}

/*
--------------------------------------------------------------------------------
                Write scanner orientation to file in BNF format
--------------------------------------------------------------------------------
*/

bool LaserScannerOrientation::HasSomeData() const
{
  return(x[0] != 0.0 || x[1] != 0.0 || x[2] != 0.0 || 
         Trace() < 2.99999 || date);
}


void LaserScannerOrientation::Write(FILE *fd, int indent) const
{
  if (!HasSomeData()) return;
  BNF_Write_String(fd, "laserscannerorientation", indent, NULL);
  if (date)
    BNF_Write_String(fd, "date", indent+2, date);
  BNF_Write_Three_Doubles(fd, "offset", indent+2, x[0], x[1], x[2],
                          "%13.8f %13.8f %13.8f");
  BNF_Write_Three_Doubles(fd, "rotation_row1", indent+2, R(0,0), R(0,1), R(0,2),
                          "%13.8f %13.8f %13.8f");
  BNF_Write_Three_Doubles(fd, "rotation_row2", indent+2, R(1,0), R(1,1), R(1,2),
                          "%13.8f %13.8f %13.8f");
  BNF_Write_Three_Doubles(fd, "rotation_row3", indent+2, R(2,0), R(2,1), R(2,2),
                          "%13.8f %13.8f %13.8f");
  BNF_Write_String(fd, "endlaserscannerorientation", indent, NULL);
}

/*
--------------------------------------------------------------------------------
                Set date
--------------------------------------------------------------------------------
*/

void LaserScannerOrientation::SetDateNow()
{
  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);
  strftime(buffer, 80, "%a %d-%m-%Y %H:%M:%S", timeinfo);
  StringCopy(&date, buffer);
}

/*
--------------------------------------------------------------------------------
                Set orientation
--------------------------------------------------------------------------------
*/
void LaserScannerOrientation::Set(const Vector3D &offset, const Rotation3D &rot)
{
  vect() = offset;
  rotation() = rot;
}
