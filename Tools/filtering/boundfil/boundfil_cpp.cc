
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
 Filtering of buildings out of preprocessed laser altimetry data.
 Files can be specified explicitly or by a file filter.
 The filtered points are written to file. Optionally, a meta data file is
 generated.

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
#include "TINEdges.h"
#include "VRML_io.h"
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
                         The main boundfil function
--------------------------------------------------------------------------------
*/

void boundfil_cpp(char *long_filter, char *infile,
			      char *bounds_file, int user_bounds,
			      int xmin_is_set, double xmin,
			      int ymin_is_set, double ymin,
			      int zmin_is_set, double zmin,
			      int xmax_is_set, double xmax,
			      int ymax_is_set, double ymax,
			      int zmax_is_set, double zmax,
			      int rmin_is_set, int rmin,
                  int rmax_is_set, int rmax,
			      int pcmin_is_set, int pcmin,
			      int pcmax_is_set, int pcmax,
			      int plmin_is_set, int plmin,
			      int plmax_is_set, int plmax,
			      int lmin_is_set, int lmin,
			      int lmax_is_set, int lmax,
                  int remove_tile_border,
                  char *appendix, char *output_directory,
                  int output_all_points, int output_meta_data)
{
  char                   *directory, *filter, *filename, *newname;
  int                    icon, fileclass;
  DataBoundsLaser        bounds;
  LaserBlock             block;
  LaserBlock::iterator   unitptr;
  LaserUnit::iterator    subunitptr;
  LaserSubUnit           newsubunit;
  LaserSubUnit::iterator point;

/* Set the bounds */

  if (bounds_file) {
    if (!bounds.Extract(bounds_file)) {
      fprintf(stderr, "Error extracting bounds from file %s\n", bounds_file);
      exit(0);
    }
  }
  else {
    bounds.Initialise();
    if (xmin_is_set) bounds.SetMinimumX(xmin);
    if (xmax_is_set) bounds.SetMaximumX(xmax);
    if (ymin_is_set) bounds.SetMinimumY(ymin);
    if (ymax_is_set) bounds.SetMaximumY(ymax);
    if (zmin_is_set) bounds.SetMinimumZ(zmin);
    if (zmax_is_set) bounds.SetMaximumZ(zmax);
    if (rmin_is_set) bounds.SetMinimumReflectance(rmin);
    if (rmax_is_set) bounds.SetMaximumReflectance(rmax);
    if (pcmin_is_set) bounds.SetMinimumPulseCount(pcmin);
    if (pcmax_is_set) bounds.SetMaximumPulseCount(pcmax);
    if (plmin_is_set) bounds.SetMinimumPulseLength(plmin);
    if (plmax_is_set) bounds.SetMaximumPulseLength(plmax);
    if (lmin_is_set) bounds.SetMinimumLabel(lmin);
    if (lmax_is_set) bounds.SetMaximumLabel(lmax);
  }

/* Set up the file filter for the input file(s) */

  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

/* Process all input files */

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

/* Read the point data */

        if (!subunitptr->Read()) {
          fprintf(stderr, "Error reading laser points from file %s\n",
                  subunitptr->PointFile());
          exit(0);
        }

/* After reading all data, the file names can be overwritten with the names
 * of the output files.
 */
        subunitptr->SetName(newsubunit.Name());
        subunitptr->DeriveMetaDataFileName(output_directory);
        subunitptr->DerivePointFileName(output_directory);
        if (!output_all_points) subunitptr->SetTINFile(NULL);

/* Filter points outside the bounds */

        for (point=subunitptr->begin();
             point!=subunitptr->end();
             point++)
          if (!bounds.Inside(&*point)) point->SetFiltered();
          else point->SetUnFiltered();

/* Delete the filtered points */

        if (!output_all_points) {
          if (remove_tile_border) subunitptr->RemoveFilteredPoints();
          else subunitptr->LaserPoints::RemoveFilteredPoints();
          subunitptr->RemoveAttribute(IsFilteredTag);
        }

/* Write the points and meta data */

        subunitptr->Write();
        if (output_meta_data) subunitptr->WriteMetaData();

/* Erase the points, the TIN, and the TIN edges */

        subunitptr->ErasePoints();
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
