
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


/* Conversion of laser altimetry data to object points
 *
 * Initial creation:
 * Author: George Vosselman
 * Date  : 08-07-1999
 */

#include <stdio.h>
#include "LaserPoints.h"

void laser2obj_cpp(char *laserfile, char *objectfile)
{
  LaserPoints  laserpoints;

/* Read the laser points */

  if (!laserpoints.Read(laserfile)) {
    fprintf(stderr, "Error reading file %s.\n", laserfile);
    exit(0);
  }

/* Construct the object points and write them to the output file */

  if (!laserpoints.ConstructObjectPoints().Write(objectfile)) {
    fprintf(stderr, "Error writing the object points to file %s\n",
	    objectfile);
  }

}
