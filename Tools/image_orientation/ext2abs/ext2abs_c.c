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

extern int lext2abs(Exterior *, Exterior *, Exterior *, Exterior *, Absolute *);

/*-----------------------------------------------------------
|
|    Written By: George Vosselman
|          Date: Aug 18, 1999
| Modifications:
|
------------------------------------------------------------*/

int ext2abs_c(char *ext1_file, char *ext2_file, char *abs_file, char *rel_file)
{
  Exterior *ext1, *ext2, *rel1, *rel2;
  Absolute *abs;
  int       error;

  // Allocate memory
  rel1 = (Exterior *) calloc(1, sizeof(Exterior));
  rel2 = (Exterior *) calloc(1, sizeof(Exterior));
  abs  = (Absolute *) calloc(1, sizeof(Absolute));

  // Read the exterior orientation data
  ext1 = Get_Exterior(ext1_file, &error);
  ext2 = Get_Exterior(ext2_file, &error);

  if (!lext2abs(ext1, ext2, rel1, rel2, abs)) {
    fprintf(stderr, "\tlext2abs failed!\n");
    exit(1);
  }

  // Write the relative and absolute orientation files

  Put_Relative(rel1, rel2, rel_file);
  Put_Absolute(abs, abs_file);

  return 1;
}
