
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
 Derivation of data bounds of a laser altimetry block, strip, strip
 part, tile, strip tile, point set or a point file. Files can be specified
 explicitly or by a file filter. Meta data is updated in the input file.
 One new output file for the top level meta data can be specified.

 Initial creation:
 Author : George Vosselman
 Date   : 21-04-1999

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

#include <malloc.h>
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

void laserbounds_cpp(char *long_filter, char *infile, char *meta_file_out,
                     int use_known_bounds)
{
  char                 *directory, *filter, *filename;
  int                  icon, fileclass, num_pts=0;
  LaserBlock           block;
  LaserBlock::iterator unitptr;
  LaserUnit::iterator  subunitptr;

/* Set up the file filter for the input file(s) */

  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

  // Process all input files
  block.Initialise();
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    
    // Set up a laser block
    if (!block.Create(filename, &fileclass)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }

    // Loop over all units
    for (unitptr=block.begin(); unitptr!=block.end(); unitptr++) {
      if (fileclass == LASER_BLOCK || fileclass == LASER_STRIP) printf("\n");
      if (unitptr->Name() != NULL) {
        printf("\rProcessing strip %s", unitptr->Name());
        fflush(stdout);
      }
      
      // Check if unit bounds are to be determined
      if (!use_known_bounds || !unitptr->Bounds().XYZBoundsSet()) {

        // Loop over all sub units
        for (subunitptr=unitptr->begin();
             subunitptr!=unitptr->end();
             subunitptr++) {
          if (fileclass == LASER_BLOCK || fileclass == LASER_STRIP) {
            if (subunitptr->Name() != NULL)
              printf("\rProcessing strip %s, part %s", unitptr->Name(),
                     subunitptr->Name());
            else
              printf("\rProcessing strip %s", unitptr->Name());
            fflush(stdout);
          }

          // Check if bounds are to be determined
          if (!use_known_bounds || !subunitptr->DataBounds().XYZBoundsSet()) {
                              
            // Read data, determine bounds and delete data
            if (!subunitptr->Read(subunitptr->PointFile(), false)) {
              fprintf(stderr, "Error reading laser points from file %s\n",
                      subunitptr->PointFile());
              exit(0);
            }
            subunitptr->DeriveDataBounds(use_known_bounds);
            num_pts += subunitptr->size();
            subunitptr->ErasePoints(false);

            // Update the meta data of sub unit, point set or points
            switch (fileclass) {
              case LASER_SUB_UNIT:
                if (meta_file_out) subunitptr->SetMetaDataFile(meta_file_out);
                if (!subunitptr->WriteMetaData()) {
                  fprintf(stderr,
                          "Error writing sub unit meta data to file %s\n",
                          subunitptr->MetaDataFile());
                  exit(0);
                }
                break;
 
              case LASER_POINT_SET:
              case LASER_RAW_DATA:
                if (meta_file_out) subunitptr->SetMetaDataFile(meta_file_out);
                else if (fileclass == LASER_RAW_DATA)
                  subunitptr->LaserPoints::DeriveMetaDataFileName(NULL);
                if (!subunitptr->LaserPoints::WriteMetaData()) {
                  fprintf(stderr,
                          "Error writing point set meta data to file %s\n",
                          subunitptr->MetaDataFile());
                  exit(0);
                }
                break;

              default: /* Nothing to do here for block and unit meta data */
                break;
            }
          }
        }

        // Aggregate the sub unit bounds to unit bounds */
        unitptr->DeriveDataBounds(1);

        // Update the meta data of the strip
        if (fileclass == LASER_STRIP) {
          if (meta_file_out) unitptr->SetMetaDataFile(meta_file_out);
          if (!unitptr->WriteMetaData(1)) {
            fprintf(stderr, "Error writing strip meta data to file %s\n",
                    unitptr->MetaDataFile());
            exit(0);
          }
        }
      }
    }

    // Aggregate the unit bounds to block bounds
    block.DeriveDataBounds(1);

    // Update the meta data of the block
    if (fileclass == LASER_BLOCK) {
      if (meta_file_out) block.SetMetaDataFile(meta_file_out);
      if (!block.WriteMetaData(1, 1)) {
        fprintf(stderr, "Error writing block meta data to file %s\n",
                block.MetaDataFile());
        exit(0);
      }
    }
  }
  printf("\nProcessed %d points\n", num_pts);
}
