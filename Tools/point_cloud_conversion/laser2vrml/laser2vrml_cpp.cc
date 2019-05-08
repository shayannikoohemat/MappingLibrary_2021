
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
 Date   : 09-06-1999

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
#include "LaserBlock.h"
#include "VRML_io.h"

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(char *, char *, int *);

/*
--------------------------------------------------------------------------------
                         The main laser2tin function
--------------------------------------------------------------------------------
*/

void laser2vrml_cpp(char *long_filter, char *infile,
                               int tinname_in_metadata, char *tinfile,
                               int output_type, double radius, char *vrmlfile)
{
  char                 *directory, *filter, *filename;
  int                  icon, filetype, readtin;
  LaserBlock           block;
  LaserBlock::iterator unitptr;
  LaserUnit::iterator  subunitptr;
  FILE                 *vrml;

/* Set up the file filter for the input file(s) */

  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

/* Open the output file */

  vrml = VRML_Open(vrmlfile);
  if (!vrml) {
    fprintf(stderr, "Error opening VRML output file %s\n", vrmlfile);
    exit(0);
  }

/* Process all input files */

  block.Initialise();
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    
/* Set up a laser block */

    if (!block.Create(filename, &filetype)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }

/* Loop over all strips */

    for (unitptr=block.begin(); unitptr!=block.end(); unitptr++) {

/* Loop over all strip parts */

      for (subunitptr=unitptr->begin();
           subunitptr!=unitptr->end();
           subunitptr++) {

/* Read the point data */

        if (!subunitptr->Read()) {
          fprintf(stderr, "Error reading laser points from file %s\n",
                  subunitptr->PointFile());
          exit(0);
        }

/* Write the point data to the VRML file */

        switch (output_type) {
          case 2: subunitptr->VRML_Write_Spheres(vrml, radius); break;
          case 3: subunitptr->VRML_Write_Crosses(vrml, radius); break;
          case 1:
          case 4: subunitptr->VRML_Write(vrml); // Just the points
                  if (output_type == 1) { // as point set
                    fprintf(vrml, "PointSet {\n startIndex 0\n");
                    fprintf(vrml, "  numPoints %d\n}\n", subunitptr->size());
                  }
        }
         
/* Read or derive the TIN data, if required */

        if (output_type == 4) {
          readtin = 1;
          if (tinname_in_metadata) {
            if (subunitptr->TINFile() == NULL) {
              fprintf(stderr,
                      "Warning: Meta data did not contain a TIN filename.\n");
              fprintf(stderr, "         A new TIN will be produced.\n");
              readtin = 0;
            }
          }
          else if (tinfile) subunitptr->SetTINFile(tinfile); 
          else readtin = 0;
          if (readtin) subunitptr->ReadTIN();
          else subunitptr->DeriveTIN();

/* Write the TIN data to the VRML file */

          subunitptr->GetTIN()->VRML_Write(vrml);
        }

/* Delete the points and the TIN */

        subunitptr->ErasePoints();
        if (output_type == 3) subunitptr->TINReference().Erase();
      }
    }
  }

/* Close the VRML file */

  VRML_Close(vrml);
}
