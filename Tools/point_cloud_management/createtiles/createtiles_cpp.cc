
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
 Reordering of laser data in tiles.
 Input files can be meta data of an altimetry block, strip, strip part, point
 set or a point file.
 Files can be specified explicitly or by a file filter.
 Meta data can be updated with the new name(s) of the image file(s).

 Initial creation:
 Author : George Vosselman
 Date   : 22-11-1999

 Update #1
 Author : 
 Date   : 
 Purpose: 

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
                         The main createtiles function
--------------------------------------------------------------------------------
*/

void createtiles_cpp(char *long_filter, char *infile,
                     int input_bounds, char *bounds_file, int spec_bounds,
                     int xminset, double xmin, int xmaxset, double xmax,
                     int yminset, double ymin, int ymaxset, double ymax,
                     double tile_width, double tile_height,
                     double tile_border, int tiles_per_strip,
                     char *rootname, char *output_directory,
                     int create_meta_files,
                     char *tile_corners, char *tile_topology,
                     bool preserve_multiple_pulses,
                     bool huge_block)
{
  char                 *directory, *filter, *filename, *name;
  int                  icon, fileclass;
  LaserBlock           block, tiled_block;
  LaserBlock::iterator unitptr;
  LaserUnit::iterator  subunitptr;
  LaserUnit            tiled_unit;
  DataBoundsLaser      bounds;

/* Get the bounds for setting up the tile structure */

  if (input_bounds) {
    if (long_filter) {
      fprintf(stderr, "Error: Bounds can not be extracted from the input meta data file if you specify a file filter.\n");
      exit(0);
    }
    bounds.Extract(infile);
  }
  else if (bounds_file)
    bounds.Extract(bounds_file);
  else if (spec_bounds) {
    bounds.Initialise();
    if (!xminset || !xmaxset || !yminset || !ymaxset) {
      fprintf(stderr, "Error: Not all X and Y bounds were specified.\n");
      exit(0);
    }
    bounds.SetMinimumX(xmin);   bounds.SetMaximumX(xmax);
    bounds.SetMinimumY(ymin);   bounds.SetMaximumY(ymax);
  }

/* Set up the file filter for the input file(s) */

  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter)+1);
  parse_filter(&filter, directory);
  icon = 0;

/* In case of aggregated data, define the tiles and the names for the output
 * files.
 */

  if (!tiles_per_strip) {
    tiled_unit.SetBounds(bounds);
    tiled_unit.DefineTiles(TileWise, tile_width, tile_height, tile_border);
    if (rootname)
      tiled_unit.SetTileNames(rootname, output_directory, huge_block);
    else if (infile) 
      tiled_unit.SetTileNames(DeriveNameFromFile(infile), output_directory, huge_block);
    else {
      if (!infile) {
        fprintf(stderr, "Error: Can not extract root name from file filter.\n");
        exit(0);
      }
    }
  }

/* Process all input files */

  block.Initialise();
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    
/* Set up a laser block */

    if (!block.Create(filename, &fileclass)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }

/* Check whether we are dealing with valid file types */

    if (tiles_per_strip) {
      if (fileclass != LASER_BLOCK && fileclass != LASER_STRIP) {
        fprintf(stderr, "Error: For tiles per strip, the input file should be a block or strip meta data file.\n");
        fprintf(stderr, "       File %s is of another type.\n", filename);
        exit(0);
      }
    }

/* Reorder the data in all units */

    for (unitptr=block.begin(); unitptr!=block.end(); unitptr++) {

/* Set up the structure for tiles per strip */

      if (tiles_per_strip) {
        tiled_unit.ReInitialise();
        tiled_unit.SetBounds(bounds);
        if (!unitptr->Name()) unitptr->DeriveName();
        name = (char *) malloc(strlen(unitptr->Name()) + 7);
        sprintf(name, "%s%s", unitptr->Name(), "_tiled");
        tiled_unit.SetName(name);
        free(name);
        tiled_unit.DeriveMetaDataFileName(output_directory);
        tiled_unit.DefineTiles(StripTileWise, tile_width, tile_height,
                               tile_border);
        tiled_unit.SetTileNames(unitptr->Name(), output_directory);
      }

/* Insert the data in the tiles */

      for (subunitptr=unitptr->begin();
           subunitptr!=unitptr->end();
           subunitptr++) {
        printf("Processing strip %s, part %s\r", unitptr->Name(),
               subunitptr->Name());
        if (!subunitptr->Read(subunitptr->PointFile(), false)) {
          fprintf(stderr, "Error reading laser points from file %s\n",
                  subunitptr->PointFile());
          exit(0);
        }
        if (tile_border == 0.0)
          tiled_unit.InsertIntoTilesWithoutBorder(subunitptr->LaserPointsReference(), preserve_multiple_pulses);
        else
          tiled_unit.InsertIntoTiles(subunitptr->LaserPointsReference(), preserve_multiple_pulses);
        subunitptr->ErasePoints();
      }

// Copy scanner information from sub unit level to unit level

      tiled_unit.Scanner() = tiled_unit.begin()->Scanner();

/* Add tiled strip to tiled block */

      if (tiles_per_strip) {
        tiled_unit.RemoveEmptyTiles();
        tiled_block.push_back(tiled_unit);
      }
      printf("\n");
    }
  }

/* Create block in case of aggregated tiles */

  if (!tiles_per_strip) {
    printf("Removing empty tiles\n");
    tiled_unit.RemoveEmptyTiles();
    tiled_block.push_back(tiled_unit);
  }

/* Output of the meta data */

  if (create_meta_files) {
    printf("Generating meta data\n");
    if (rootname)
      tiled_block.SetName(rootname);
    else if (infile) {
      name = (char *) malloc(strlen(DeriveNameFromFile(infile))+7);
      sprintf(name, "%s%s", DeriveNameFromFile(infile), "_tiled");
      tiled_block.SetName(name);
    }
    else {
      tiled_block.SetName("data_tiled");
      fprintf(stderr, "Warning: No useful block name could be derived.\n");
      fprintf(stderr,
           "         Meta data of block will be stored in data_tiled.block.\n");
    }
    tiled_block.DeriveMetaDataFileName(output_directory);
    tiled_block.Scanner() = tiled_block.begin()->Scanner();
    tiled_block.WriteMetaData(1, 1);
  }

/* Output of tile boundaries */

  if (tile_corners || tile_topology) {
    printf("Generating tile corner outlines\n");
    tiled_block.WriteTileBoundaries(tile_corners, tile_topology);
  }
}
