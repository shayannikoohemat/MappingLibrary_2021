
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
 Split labeled filtering results into ground points and filtered points
 Files can be specified explicitly or by a file filter.
 Optionally, a meta data file is generated.

 Initial creation:
 Author : George Vosselman
 Date   : 24-01-2000

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
#include <malloc.h>
#include <time.h>
#include "LaserPoints.h"   /* Extended version */
#include "LaserSubUnit.h"  /* Extended version */
#include "LaserBlock.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(char *, char *, int *);

/*
--------------------------------------------------------------------------------
                         The main filsplit function
--------------------------------------------------------------------------------
*/

void filsplit_cpp(char *long_filter, char *infile,
 			      char *output_directory,
			      const char *gp_app, const char *fp_app,
                  int output_meta_data,
				  int tag, int value)
{
  char                 *directory, *filter, *filename, *newname;
  int                  icon, fileclass, iapp;
  LaserBlock           block;
  LaserBlock::iterator unitptr;
  LaserUnit::iterator  subunitptr;
  const char           *appendix[2];

/* Store the appendices in an array */

  appendix[0] = gp_app;
  appendix[1] = fp_app;

/* Set up the file filter for the input file(s) */

  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

/* Collect all points from all input files */

  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    
/* Loop over appendices */

    for (iapp=0; iapp<2; iapp++) {
      if (appendix[iapp]) {

/* Set up a laser block */

        if (!block.Create(filename, &fileclass)) {
          fprintf(stderr, "Error reading meta data file %s\n", filename);
          exit(0);
        }

/* Loop over all units */

        for (unitptr=block.begin(); unitptr!=block.end(); unitptr++) {

/* Loop over all sub units */

          for (subunitptr=unitptr->begin(); subunitptr!=unitptr->end();
	           subunitptr++) {
	       	
	       	if (subunitptr->Name()) printf("Processing %s\r", subunitptr->Name());

/* Read the point data */

            if (!subunitptr->Read()) {
              fprintf(stderr, "Error reading laser points from file %s\n",
                      subunitptr->PointFile());
              exit(0);
            }

/* Derive the names of the output files */

            subunitptr->LaserDataFiles::DeriveName();
            newname = (char *) malloc(strlen(subunitptr->Name()) +
                                      strlen(appendix[iapp]) + 1);
            sprintf(newname, "%s%s", subunitptr->Name(), appendix[iapp]);
            subunitptr->SetName(newname);
            subunitptr->DeriveMetaDataFileName(output_directory);
            subunitptr->DerivePointFileName(output_directory);
            subunitptr->SetTINFile(NULL);
            free(newname);

/* Select the ground points or filtered points */

            if (iapp == 0)
			  subunitptr->LaserPoints::CropTaggedPoints(value, (LaserPointTag) tag);
            else
			  subunitptr->LaserPoints::RemoveTaggedPoints(value, (LaserPointTag) tag);

/* Write the points and meta data */

            subunitptr->LaserPoints::Write(subunitptr->PointFile(), 0, false);
            if (output_meta_data) subunitptr->WriteMetaData();

/* Erase the point data */

            subunitptr->ErasePoints();
          }

/* Output of meta data at unit level */

          if (output_meta_data &&
              (fileclass == LASER_STRIP || fileclass == LASER_BLOCK) &&
              unitptr->DataOrganisation() & StripWise) {
            if (!unitptr->Name()) unitptr->LaserDataFiles::DeriveName();
            newname = (char *) malloc(strlen(unitptr->Name()) +
                                      strlen(appendix[iapp]) + 1);
            sprintf(newname, "%s%s", unitptr->Name(), appendix[iapp]);
            unitptr->SetName(newname);
            free(newname);
            unitptr->DeriveMetaDataFileName(output_directory);
            if (unitptr->begin()->IsCompleteStrip())
              unitptr->DerivePointFileName(output_directory);
            else unitptr->SetPointFile(NULL);
            unitptr->SetTINFile(NULL);
            unitptr->WriteMetaData();
          }
        }

/* Output of meta data at block level */

        if (output_meta_data && fileclass == LASER_BLOCK) {
          if (!block.Name()) block.DeriveName();
          newname = (char *) malloc(strlen(block.Name()) +
                                    strlen(appendix[iapp]) + 1);
          sprintf(newname, "%s%s", block.Name(), appendix[iapp]);
          block.SetName(newname);
          free(newname);
          block.DeriveMetaDataFileName(output_directory);
          block.WriteMetaData();
        }
      }
    }
  }
  printf("\n");
}
