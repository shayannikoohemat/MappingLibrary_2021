
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

 Selection of a part of a point cloud pyramid

 Initial creation:
 Author : George Vosselman
 Date   : 22-05-2013

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
#include "LaserPyramid.h"

/*
--------------------------------------------------------------------------------
                         The main selectpyramidpart function
--------------------------------------------------------------------------------
*/

void selectpyramidpart(char *pyramid_filename, char *directory,
                       int level, int first_row, int last_row,
					   int first_column, int last_column)
{
  const LaserBlock           *block;
  LaserBlock                 block_part;
  LaserBlock::const_iterator tile_container;
  LaserBlock::iterator       selected_tiles;
  LaserUnit::iterator        selected_tile;
  const LaserSubUnit         *tile;
  int                        success, row, column;  
    
  // Check validity of input
  if (first_row > last_row) {
  	printf("Error: first selected row is larger than last selected row.\n");
  	return;
  }
  if (first_column > last_column) {
  	printf("Error: first selected column is larger than last selected column.\n");
  	return;
  }

  // Read the input pyramid and check whether there are tiles
  LaserPyramid pyramid = LaserPyramid(pyramid_filename, &success);
  if (pyramid.empty()) {
    printf("Error: this pyramid contains no blocks\n");
    return;
  }
  block = pyramid.BlockAtLevel(level);
  if (block == NULL) {
  	printf("Error: pyramid has no block at level %d\n", level);
  	return;
  }
  if (block->empty()) {
    printf("Error: selected block contains no tiles!\n");
    return;
  }
  tile_container = block->begin();
  if (tile_container->empty()) {
    printf("Error: selected block contains no tiles!\n");
    return;
  }
  if (tile_container->begin()->DataOrganisation() != TileWise) {
    printf("Error: selected block contains no tiles!\n");
    return;
  }
  printf("Read pyramid with %d levels\n", pyramid.size());
  printf("Read level %d block with %d tiles\n", level, tile_container->size());

  // Initialise block part
  block_part.push_back(LaserUnit());
  block_part.SetName(block->Name());
  block_part.DeriveMetaDataFileName(directory);
  selected_tiles = block_part.begin();
  selected_tiles->DataOrganisation() = TileWise;
  
  // Select the requested tiles
  for (row=first_row; row<=last_row; row++) {
  	for (column=first_column; column<=last_column; column++) {
  	  tile = block->Tile(row, column);
  	  if (tile) {
  	  	printf("Copying tile %s\r", tile->Name());
  	  	// Store a copy of the tile meta data
  	  	selected_tiles->push_back(*tile);
  	  	selected_tile = selected_tiles->end() - 1;
  	  	// Read the laser points
  	  	selected_tile->Read(selected_tile->PointFile(), false);
  	  	// Set new file names
  	  	selected_tile->DerivePointFileName(directory);
  	  	selected_tile->DeriveMetaDataFileName(directory);
        selected_tile->SetSeekOffset(0);
        // Write the laser points
        selected_tile->Write(selected_tile->PointFile(), false, false);
        selected_tile->ErasePoints();
  	  }
  	}
  }

  // Write meta data of the new block
  block_part.WriteMetaData(true, true);
  printf("\nBlock part with %d tiles saved in %s\n", (int) selected_tiles->size(),
         directory);
}
