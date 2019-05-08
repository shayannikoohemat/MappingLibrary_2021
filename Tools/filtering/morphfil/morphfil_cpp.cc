
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
 Morphological filtering of laser altimetry data.
 Files can be specified explicitly or by a file filter.
 The filtered points are written to file. Optionally, a meta data file is
 generated.

 Initial creation:
 Author : George Vosselman
 Date   : 11-01-2000

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
#include "LaserPoints.h"
#include "LaserSubUnit.h"
#include "LaserBlock.h"
#include "TINEdges.h"
#include "VRML_io.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                         Declaration of external functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(const char *, const char *, int *);
extern void timer_start(clock_t *time1);
extern void timer_end(clock_t time1, char *string);

/*
--------------------------------------------------------------------------------
                         The main morphfil function
--------------------------------------------------------------------------------
*/

void morphfil_cpp(char *long_filter, char *infile,
                  char *kernel_file, int set_range,
			      double range, double stdev, double tolerance,
                  int dilate,
			      char *appendix, char *output_directory,
                  int output_all_points,
                  int output_meta_data, int overwrite)
{
  char                 *directory, *filter, *filename, *newname;
  int                  icon, fileclass;
  double               min_range, max_range, min_dummy, max_dummy;
  LaserBlock           block;
  LaserBlock::iterator unitptr;
  LaserUnit::iterator  subunitptr;
  LaserSubUnit         newsubunit;
  LaserSubUnit::iterator point;
  TINEdges             edges;
  Image                kernel;
  clock_t              start;

/* Read the filter kernel and determine the maximum range to be used for 
 * filtering.
 */

  kernel.Read(kernel_file);
  if (kernel.NumRows() == 1) kernel.Get1DLocationData(&min_range, &max_range);
  else kernel.Get2DLocationData(&min_range, &max_range, &min_dummy, &max_dummy);
  if (min_range != 0 || max_range == 0) {
    fprintf(stderr, "Invalid kernel range: %6.2f %6.2f\n",
            min_range, max_range);
    exit(0);
  }
  if (set_range) max_range = range;

/* Set up the file filter for the input file(s) */

  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

/* Collect all points from all input files */

  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    
/* Set up a laser block */

    if (!block.Create(filename, &fileclass)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }

/* Loop over all units */

    for (unitptr=block.begin(); unitptr!=block.end(); unitptr++) {

/* Loop over all sub units */

      for (subunitptr=unitptr->begin();
	   subunitptr!=unitptr->end();
	   subunitptr++) {

/* Derive the names of the output files. Note that the variable newsubunit
 * is only used for checking the existance of files. New file names are later
 * transfered to subunitptr.
 */

        if (!subunitptr->Name()) subunitptr->LaserDataFiles::DeriveName();
        newname = (char *) malloc(strlen(subunitptr->Name()) +
                                  strlen(appendix) + 1);
        sprintf(newname, "%s%s", subunitptr->Name(), appendix);
        newsubunit.SetName(newname);
        newsubunit.DataOrganisation() = subunitptr->DataOrganisation();
        newsubunit.DeriveMetaDataFileName(output_directory);
        newsubunit.DerivePointFileName(output_directory);
        free(newname);

/* Do not go ahead if the point file and meta file already exists and 
 * overwriting these files is not permitted.
 */

        if (overwrite || !FileExists(newsubunit.MetaDataFile()) ||
            !FileExists(newsubunit.PointFile())) {
        
/* Read the point data */

          if (!subunitptr->Read()) {
            fprintf(stderr, "Error reading laser points from file %s\n",
                    subunitptr->PointFile());
            exit(0);
        }

/* Read or create the TIN and extract the TIN edges. */

          timer_start(&start);
          if (subunitptr->TINFile()) {
            subunitptr->ReadTIN();
            timer_end(start, (char *) "reading TIN");
          }
          else {
            printf("Deriving TIN for sub unit %s\n", subunitptr->Name());
            subunitptr->DeriveTIN();
            timer_end(start, (char *) "deriving TIN");
          }
          timer_start(&start);
          edges.Derive(subunitptr->TINReference());
          timer_end(start, (char *) "deriving TIN edges");

/* After reading all data, the file names can be overwritten with the names
 * of the output files.
 */
          subunitptr->SetName(newsubunit.Name());
          subunitptr->DeriveMetaDataFileName(output_directory);
          subunitptr->DerivePointFileName(output_directory);
          if (!output_all_points) subunitptr->SetTINFile(NULL);

// Change sign of height in case of dilation

          if (dilate) {
            for (point=subunitptr->begin(); point!=subunitptr->end(); point++)
              point->Z() = -point->Z();
          }

/* Make sure we have bounds. */

          subunitptr->DeriveDataBounds(1);

/* Filter the data with the morphological kernel */

          timer_start(&start);
          subunitptr->FilterOnMorphology(kernel, range, stdev, tolerance,
                                         edges);
//        subunitptr->Scanner().SetPointType(NormalPoint);
          timer_end(start, (char *) "morphological filtering");

/* Delete the filtered points */

          if (!output_all_points) subunitptr->RemoveFilteredPoints();

// Restore original sign of height in case of dilation

          if (dilate) {
            for (point=subunitptr->begin(); point!=subunitptr->end(); point++)
              point->Z() = -point->Z();
          }

/* Write the points and meta data */

          subunitptr->Write();
          if (output_meta_data) subunitptr->WriteMetaData();

/* Erase the points, the TIN, and the TIN edges */

          subunitptr->ErasePoints();
          subunitptr->TINReference().Erase();
          if (!edges.empty()) edges.erase(edges.begin(), edges.end());
        }
        else {
          printf("Sub unit %s was already done.\n", newsubunit.Name());
          if (!subunitptr->ReadMetaData(newsubunit.MetaDataFile())) {
            fprintf(stderr, "Error reading meta data from file %s\n",
                    newsubunit.MetaDataFile());
            exit(0);
          }
        }
      }

/* Output of meta data at unit level */

      if (output_meta_data &&
          (fileclass == LASER_STRIP || fileclass == LASER_BLOCK) &&
          unitptr->DataOrganisation() & StripWise) {
        if (!unitptr->Name()) unitptr->LaserDataFiles::DeriveName();
        newname = (char *) malloc(strlen(unitptr->Name()) +
                                  strlen(appendix) + 1);
        sprintf(newname, "%s%s", unitptr->Name(), appendix);
        unitptr->SetName(newname);
        free(newname);
        unitptr->DeriveMetaDataFileName(output_directory);
        if (unitptr->begin()->IsCompleteStrip())
          unitptr->DerivePointFileName(output_directory);
        else unitptr->SetPointFile(NULL);
        if (!output_all_points) unitptr->SetTINFile(NULL);
        unitptr->WriteMetaData();
      }
    }

/* Output of meta data at block level */

    if (output_meta_data && fileclass == LASER_BLOCK) {
      if (!block.Name()) block.DeriveName();
      newname = (char *) malloc(strlen(block.Name()) +
                                strlen(appendix) + 1);
      sprintf(newname, "%s%s", block.Name(), appendix);
      block.SetName(newname);
      free(newname);
      block.DeriveMetaDataFileName(output_directory);
      block.WriteMetaData();
    }
  }
}
