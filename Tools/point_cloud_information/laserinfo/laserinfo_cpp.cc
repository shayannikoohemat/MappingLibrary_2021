
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

 Initial creation:
 Author : George Vosselman
 Date   : 06-03-2005

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include "LaserPoints.h"

#include "LaserPyramid.h"
/*
--------------------------------------------------------------------------------
                               Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" FILE *Open_Compressed_File(const char *, const char *);
extern "C" int  Close_Compressed_File(FILE *);

/*
--------------------------------------------------------------------------------
                      The main laserinfo function
--------------------------------------------------------------------------------
*/

void laserinfo_cpp(char *point_file)
{
  LaserPoints           points;
  double                x_offset, y_offset, z_offset;
  unsigned char         max_num_attributes;
  FILE                  *fd;
  int                   file_id, num_pts;

  // Open the file
  if ((fd = Open_Compressed_File(point_file, "rb")) == NULL) {
    fprintf(stderr, "Could not open laser points file %s\n", point_file);
    exit(0);
  }

  // Read the header information
  points.SetPointFile(point_file);
  file_id = points.ReadHeader(fd, &num_pts, &x_offset, &y_offset, &z_offset,
                              &max_num_attributes);
  Close_Compressed_File(fd);

  // Output the header information
  printf("Header information of file %s:\n", point_file);
  switch (file_id) {
    case 0:               printf("  File format version 0\n"); break;
    case LASER_POINTS_V1: printf("  File format version 1\n"); break;
    case LASER_POINTS_V2: printf("  File format version 2\n"); break;
    case LASER_POINTS_V3: printf("  File format version 3\n"); break;
    default:              printf("  Unrecognised file format\n"); exit(0);
  }
  printf("  Total number of points: %d\n", num_pts);
  if (file_id == 3)
    printf("  Maximum number of point attributes: %d\n", max_num_attributes);

  // Read the points
  points.Read(point_file, false);

  // Output the information on the first point
  if (points.empty()) return;
  printf("First point data:\n  ");
  points.begin()->Print();
}
