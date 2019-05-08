
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
 Output of an image in VRML format

 Initial creation:
 Author : George Vosselman
 Date   : 17-02-2000

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include "Image.h"
#include "DataGrid.h"

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                         The main viff2vrml function
--------------------------------------------------------------------------------
*/

void viff2vrml_cpp(char *image_file, char *grid_file,
			       double red, double green, double blue,
                   char *vrml_file)
{
  Image           image;
  DataGrid        grid;
  unsigned char   *pixel, *row;
  float           *pixelf, *rowf, max_height;
  int             ir, ic;
  FILE            *vrml;

/* Read the image and the image grid */

  if (!image.Read(image_file)) {
    fprintf(stderr, "Error reading image %s\n", image_file);
    exit(0);
  }

  if (grid_file) {
    if (!grid.Read(grid_file)) {
      fprintf(stderr, "Error reading data grid %s\n", grid_file);
      exit(0);
    }
  }

/* Open the VRML file */

  vrml = fopen(vrml_file, "w");
  if (!vrml) {
    fprintf(stderr, "Error opening file %s\n", vrml_file);
    exit(0);
  }

/* Write the header */

  fprintf(vrml, "#VRML V2.0 utf8\n");
  fprintf(vrml, "Transform { \n");
  fprintf(vrml, "  rotation 1 0 0 1.570796\n");
  fprintf(vrml, "  translation %10.3f %10.3f 0\n",
          grid.XOffset(), grid.YOffset());
  fprintf(vrml, "  children [ Shape {\n");
  fprintf(vrml, "    geometry DEF EG ElevationGrid {\n");
  fprintf(vrml, "      xDimension %d\n", image.NumRows());
  fprintf(vrml, "      xSpacing %6.2f\n", grid.Pixelsize());
  fprintf(vrml, "      zDimension %d\n", image.NumColumns());
  fprintf(vrml, "      zSpacing %6.2f\n", grid.Pixelsize());
  fprintf(vrml, "      height [\n");

/* Write the heights */

  max_height = 0.0;
  if (image.DataType() == VFF_TYP_FLOAT) {
    for (ic=0, rowf=image.float_begin(); ic<image.NumColumns(); ic++, rowf++) {
      for (ir=0, pixelf=rowf; ir<image.NumRows(); ir++, pixelf+=image.NumColumns()) {
        fprintf(vrml, "%5.2f ", *pixelf * grid.DataScale() + grid.DataOffset());
        if (*pixelf > max_height) max_height = *pixelf;
      }
      fprintf(vrml, "\n");
    }      
  }
  else {
    for (ic=0, row=image.begin(); ic<image.NumColumns(); ic++, row++) {
      for (ir=0, pixel=row; ir<image.NumRows(); ir++, pixel+=image.NumColumns()) {
        fprintf(vrml, "%5.2f ", *pixel * grid.DataScale() + grid.DataOffset());
        if ((float) *pixel > max_height) max_height = (float) *pixel;
      }
      fprintf(vrml, "\n");
    }
  }
  
/* Write the trailer */

  fprintf(vrml, "      ]\n");
  fprintf(vrml, "      creaseAngle 0.8\n");
  fprintf(vrml, "    }\n");
  fprintf(vrml, "    appearance Appearance {\n");
  fprintf(vrml,
          "      material DEF M Material { diffuseColor %5.2f %5.2f %5.2f }\n",
          red, green, blue);
  fprintf(vrml, "    }\n");
  fprintf(vrml, "  }\n");
//fprintf(vrml, "  DirectionalLight {direction -0.80 -0.60 0 }\n");
  fprintf(vrml, "  Viewpoint { position %6.2f %6.2f %6.2f\n",
          0.5 * grid.Pixelsize() * image.NumRows() + grid.XOffset(), 
          -grid.Pixelsize() * image.NumColumns() + grid.YOffset(), 
          1.5 * grid.Pixelsize() * image.NumRows() +
          max_height * grid.DataScale() + grid.DataOffset());
  fprintf(vrml, "              orientation 0 -0.8 -0.6 0 }\n");
  fprintf(vrml, "]}\n");
  fclose(vrml);
}
