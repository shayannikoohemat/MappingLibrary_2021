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
#include "viff.h"

/*-----------------------------------------------------------
|
|    Written By: Ir. R.Th. Ursem
|          Date: Jul 28, 1999
| Modifications:
|
------------------------------------------------------------*/

#define READINPUT(image, name, program) \
  image = readimage(name);  \
  if (image == NULL) {  \
    (void) fprintf(stderr, "%s: Can not read input image %s.\n", program, name);  \
    exit(1);  /* Quit if bad image */ \
  }
  
#define TRUE 1

#define CHECKINPUT(program, image) \
  propertype(program, image, VFF_TYP_1_BYTE, TRUE); \
  proper_num_images(program, image, 1, TRUE);  \
  proper_num_bands(program, image, 1, TRUE);  \
  proper_map_enable(program, image, VFF_MAP_OPTIONAL, TRUE);
  
#define ROUND(x) ( (int) ( (x) + 0.5 ) )


int epipolar_c(char *i1_file, char *i2_file, char *rel_file, char *int1_file,
               char *int2_file, char *o1_file, char *o2_file, char *orel_file,
               char *oint1_file, char *oint2_file)
{
  xvimage *img_1, *img_2, *nor_1, *nor_2, *readimage();
  Exterior ext_1, ext_2;
  Interior *int_1, *int_2;

  // Read the specified information

  // Images
  READINPUT(img_1, i1_file, "epipolar");
  CHECKINPUT("epipolar", img_1);
  READINPUT(img_2, i2_file, "epipolar");
  CHECKINPUT("epipolar", img_2);

  // Interior orientation data
  int_1 = Get_Interior(int1_file);
  int_2 = Get_Interior(int2_file);

  // Relative orientation data
  Get_Relative(rel_file, &ext_1, &ext_2);

  if (!lepipolar(img_1, &ext_1, int_1, img_2, &ext_2, int_2, &nor_1, &nor_2))
    exit(1);

  printf("Writing first normal image to %s\n", o1_file); 
  writeimage(o1_file, nor_1);
  
  printf("Writing second normal image to %s\n", o2_file); 
  writeimage(o2_file, nor_2);
  
  if (oint1_file)
     Put_Interior(int_1, oint1_file);
  if (oint2_file)
     Put_Interior(int_2, oint2_file);
  
  if (orel_file) {
    ext_1.x = ext_1.y = ext_1.z = 0.0;
    ext_1.a = ext_1.b = ext_1.c = 0.0;
    Rot_From_Quat(&ext_1);
    Angles_From_Rot(&ext_1);
     
    ext_2.x = 1.0; ext_2.y = ext_2.z = 0.0;
    ext_2.a = ext_2.b = ext_2.c = 0.0;
    Rot_From_Quat(&ext_2);
    Angles_From_Rot(&ext_2);
     
    Put_Relative(&ext_1, &ext_2, orel_file);
  }

  return 1;
}
