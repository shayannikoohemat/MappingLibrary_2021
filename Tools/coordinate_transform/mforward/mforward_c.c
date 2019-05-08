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

#include <stdio.h>
#include <stdlib.h>
#include "Database.h"

extern int lmforward(char *, char *, char *, ObjPts **);

/*------------------------------------------------------------
|
|    Written By: George Vosselman
|          Date: Jul 28, 1999
| Modifications:
|
------------------------------------------------------------*/

int mforward_c(char *imgpts_filter, char *exterior_filter,
               char *interior_filter, char *objpts_file)
{
  ObjPts *objpts;

/* Since I/O is the main part of this program, nearly everything is done in the
 * library routine.
 */
  if (!lmforward(imgpts_filter,   // Filter for image point files
                 exterior_filter, // Filter for exterior orientations
                 interior_filter, // Filter for interior orientations
                 &objpts)) {      // Calculated object points
    fprintf(stderr, "mforward: A failure occurred in library routine lmforward.\n");
    exit(0);
  }

  Put_ObjPts(objpts, objpts_file);

  return 1;
}
