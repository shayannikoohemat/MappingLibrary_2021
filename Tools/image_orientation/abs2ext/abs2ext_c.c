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

extern int labs2ext(Exterior *, Exterior *, Absolute *, Exterior *, Exterior *);

/*----------------------------------------------------------
|
|    Written By: George Vosselman
|          Date: Aug 18, 1999
| Modifications:
|
------------------------------------------------------------*/

int abs2ext_c(char *abs_file, char *rel_file, char *ext1_file, char *ext2_file)
{
  Exterior *ext1, *ext2, *rel1, *rel2;
  Absolute *abs;
  int       error;

  // Allocate memory
  ext1 = (Exterior *) calloc(1, sizeof(Exterior));
  ext2 = (Exterior *) calloc(1, sizeof(Exterior));
  rel1 = (Exterior *) calloc(1, sizeof(Exterior));
  rel2 = (Exterior *) calloc(1, sizeof(Exterior));

  // Read the relative and absolute orientation data
  Get_Relative(rel_file, rel1, rel2);
  abs = Get_Absolute(abs_file, &error);
  
  if (!labs2ext(rel1, rel2, abs, ext1, ext2)) {
    fprintf(stderr, "\tlabs2ext failed!\n");
    exit(1);
  }

  // Write the exterior orientation files
  Put_Exterior(ext1, ext1_file);
  Put_Exterior(ext2, ext2_file);

  return 1;
}
