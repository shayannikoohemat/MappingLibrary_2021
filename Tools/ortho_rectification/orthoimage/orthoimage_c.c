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
#include "viff.h"
#include "Database.h"

#define TRUE 1

#define CHECKINPUT(program, image, dem) \
        (void) proper_num_images(program,image,1,TRUE); \
        (void) proper_num_images(program,dem,1,TRUE); \
        \
        (void) proper_num_bands(program,image,1,TRUE); \
        (void) proper_num_bands(program,dem,1,TRUE); \
        \
        (void) proper_map_enable(program,image,VFF_MAP_OPTIONAL,TRUE); \
        (void) proper_map_enable(program,dem,VFF_MAP_OPTIONAL,TRUE); \
        \
        (void) propertype(program,image,VFF_TYP_1_BYTE,TRUE); \
        (void) propertype(program,dem,VFF_TYP_FLOAT,TRUE);

/*-----------------------------------------------------------
|
|    Written By: George Vosselman
|          Date: Jul 27, 1999
| Modifications:
|
------------------------------------------------------------*/

void orthoimage_c(const char *image_file, const char *interior_file,
                  const char *exterior_file, const char *dem_file,
                  float x0dem, float y0dem, float dem_pixel_size, 
				  const char *ortho_file, float ortho_pixel_size,
				  int area, int interpolation_method,
				  float xmin, float ymin, float xmax, float ymax)
{
  Interior *interior;
  Exterior *exterior;
  xvimage  *image, *dem, *ortho, *readimage(), *createimage();
  int      error;
  
  // Input of image and DEM
  image = readimage(image_file);
  if (image == NULL){
    fprintf(stderr,"orthoimage: Image %s could not be read.\n", image_file);
    exit(1);
  }
  printf("Image read from file %s\n", image_file);

  dem = readimage(dem_file);
  if (dem == NULL){
    fprintf(stderr,"orthoimage: DEM could not be read from %s.\n", dem_file);
    exit(1);
  }
  printf("DEM read from file %s\n", dem_file);
  dem->fspare1 = x0dem;
  dem->fspare2 = y0dem;
  dem->pixsizx = dem_pixel_size;
  dem->pixsizy = dem_pixel_size;

  CHECKINPUT("orthoimage", image, dem);

  // Creation of orthophoto structure

  ortho = createimage(1, 1, VFF_TYP_1_BYTE, 1, 1,
                      "Created by orthoimage", 0, 0,
                      VFF_MS_NONE, VFF_MAPTYP_NONE,
                      VFF_LOC_IMPLICIT, 0);
  if (ortho == NULL) {
    fprintf(stderr, "Error creating orthophoto structure.\n");
    exit(1);
  }

  // Input of orientation parameters

  exterior = Get_Exterior(exterior_file, &error);
  if (exterior == NULL) exit(1);

  interior = Get_Interior(interior_file);
  if (interior == NULL) exit(1);

  // Calculation of area of orthophoto. Calculated parameters are stored
  // in the appropriate fields of the ortho structure.

  if (!get_ortho_size(image, dem, ortho, interior, exterior,
                      area, xmin, xmax, ymin, ymax, ortho_pixel_size)) {
    fprintf(stderr, "Cannot proceed with orthophoto calculation.\n");
    exit(1);
  }

  // Allocation of orthophoto space

  ortho->imagedata = (char *) malloc(ortho->row_size * ortho->col_size);
  if (ortho->imagedata == NULL) {
    fprintf(stderr, "Error allocating memory for orthophoto.\n");
    exit(1);
  }

  if (!lorthoimage(image, dem, ortho, interior, exterior,
                   area, interpolation_method)) {
    fprintf(stderr, "lorthoimage failed\n");
    exit(1);
  }

  // Write the ortho image
  writeimage(ortho_file, ortho);
}
