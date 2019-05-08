
/*
    Copyright 2013 University of Twente
 
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
 Removal of segment numbers from points in small segments

 Initial creation:
 Author : George Vosselman
 Date   : 21-08-2013

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
#include "LaserBlock.h"
#include "TINEdges.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(const char *, const char *, int *);

/*
--------------------------------------------------------------------------------
                         The main unlabelsurface function
--------------------------------------------------------------------------------
*/

void unlabelsurfaces(char *long_filter, char *infile,
                     char *appendix, char *output_directory,
                     bool output_meta_data, bool overwrite,
                     int minimum_segment_size)
{
  char                  *directory, *filter, *filename, *newname;
  int                   icon, fileclass, tile_number;
  LaserBlock            block;
  LaserBlock::iterator  unitptr;
  LaserUnit::iterator   subunitptr;
  LaserSubUnit          newsubunit;
  LaserPoints::iterator point;
  bool                  reset_start_tile;

  // Set up the file filter for the input file(s)
  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

  // Collect all points from all input files
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    // Set up a laser block
    if (!block.Create(filename, &fileclass)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }

    // Loop over all units and subunits
    for (unitptr=block.begin(); unitptr!=block.end(); unitptr++) {
      // If the block has tiles, reset SegmentStartTileNumberTag to local tile
      reset_start_tile = (unitptr->DataOrganisation() == TileWise);
      for (subunitptr=unitptr->begin(); subunitptr!=unitptr->end();
	       subunitptr++) {
        if (reset_start_tile)
          tile_number = subunitptr->TileNumber();

        // Derive the names of the output files. Note that the variable
        // newsubunit is only used for checking the existance of files. New file 
        // names are later transfered to subunitptr.
        if (!subunitptr->Name()) subunitptr->LaserDataFiles::DeriveName();
        if (appendix) {
          newname = (char *) malloc(strlen(subunitptr->Name()) +
                                    strlen(appendix) + 1);
          sprintf(newname, "%s%s", subunitptr->Name(), appendix);
        }
        else {
          newname = (char *) malloc(strlen(subunitptr->Name()) + 1);
          sprintf(newname, "%s", subunitptr->Name());
        }
        newsubunit.SetName(newname);
        newsubunit.DataOrganisation() = subunitptr->DataOrganisation();
        newsubunit.DeriveMetaDataFileName(output_directory);
        newsubunit.DerivePointFileName(output_directory);
        free(newname);

        // Do not go ahead if the point file and meta file already exists and 
        // overwriting these files is not permitted.
        if (overwrite || !BNF_FileExists(newsubunit.MetaDataFile()) ||
            !BNF_FileExists(newsubunit.PointFile())) {
        
          // Read the point data
          if (!subunitptr->Read()) {
            fprintf(stderr, "Error reading laser points from file %s\n",
                    subunitptr->PointFile());
            exit(0);
          }

          // After reading all data, the file names can be overwritten with the
          // names of the output files.
          subunitptr->SetName(newsubunit.Name());
          subunitptr->DeriveMetaDataFileName(output_directory);
          subunitptr->DerivePointFileName(output_directory);
          
          // Remove seek offset as all tiles are written separately
          subunitptr->SetSeekOffset(0);

          // Check if the segment sizes have been computed
          if (!subunitptr->HasAttribute(SegmentSizeTag)) {
	        printf("Error: The segment sizes are not available.\n");
	        printf("       They can be computed with the programme blockattributes.\n");
            exit(0);
	      }

          // Unlabel points of small segments
          for (point=subunitptr->begin(); point!=subunitptr->end(); point++) {
            if (point->HasAttribute(SegmentSizeTag)) {
              if (point->Attribute(SegmentSizeTag) < minimum_segment_size) {
                point->RemoveAttribute(SegmentSizeTag);
                point->RemoveAttribute(SegmentNumberTag);
                if (reset_start_tile &&
				    point->HasAttribute(SegmentStartTileNumberTag))
                  point->RemoveAttribute(SegmentStartTileNumberTag);
              }
            }
          }

          // Write the points and meta data
          subunitptr->Write();
          if (output_meta_data) subunitptr->WriteMetaData();

          // Erase the points
          subunitptr->ErasePoints();
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

      // Output of meta data at unit level
      if (output_meta_data &&
          (fileclass == LASER_STRIP || fileclass == LASER_BLOCK) &&
          unitptr->DataOrganisation() & StripWise) {
        if (!unitptr->Name()) unitptr->LaserDataFiles::DeriveName();
        if (appendix) {
          newname = (char *) malloc(strlen(unitptr->Name()) +
                                    strlen(appendix) + 1);
          sprintf(newname, "%s%s", unitptr->Name(), appendix);
        }
        else {
          newname = (char *) malloc(strlen(unitptr->Name()) + 1);
          sprintf(newname, "%s", unitptr->Name());
        }
        unitptr->SetName(newname);
        free(newname);
        unitptr->DeriveMetaDataFileName(output_directory);
        if (unitptr->begin()->IsCompleteStrip())
          unitptr->DerivePointFileName(output_directory);
        else unitptr->SetPointFile(NULL);
        unitptr->WriteMetaData();
      }
    }

    // Output of meta data at block level
    if (output_meta_data && fileclass == LASER_BLOCK) {
      if (!block.Name()) block.DeriveName();
      if (appendix) {
        newname = (char *) malloc(strlen(block.Name()) +
                                  strlen(appendix) + 1);
        sprintf(newname, "%s%s", block.Name(), appendix);
      }
      else {
        newname = (char *) malloc(strlen(block.Name()) + 1);
        sprintf(newname, "%s", block.Name());
      }
      block.SetName(newname);
      free(newname);
      block.DeriveMetaDataFileName(output_directory);
      block.WriteMetaData();
    }
  }
}
