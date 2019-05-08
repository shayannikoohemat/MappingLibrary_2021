
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
 Restructuring of tile meta data and point data of a laser pyramid:
 - point data is of all pyramid tiles is stored in one file
 - seek offsets to access a tile are added to tile meta data
 - meta data of tiles is combined in a one file per block with extension .tiles

 Initial creation:
 Author : George Vosselman
 Date   : 06-03-2007

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
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                         The main mergepyramiddata function
--------------------------------------------------------------------------------
*/

void mergepyramiddata(char *pyramid_filename, const char *app, char *directory)
{
  char *tiles_file, *name;
  LaserPyramid::iterator block;
  LaserBlock::iterator tile_container;
  LaserUnit::iterator tile;
  FILE *fd_points, *fd_block, *fd_tiles;
  int success, num_bytes;
  long long seek_offset;
    
  // Read the input pyramid and check whether there are tiles
  LaserPyramid pyramid = LaserPyramid(pyramid_filename, &success);
  if (pyramid.empty()) {
    printf("Error: this pyramid contains no blocks\n");
    return;
  }
  block = pyramid.begin();
  if (block->empty()) {
    printf("Error: first block contains no tiles!\n");
    return;
  }
  tile_container = block->begin();
  if (tile_container->empty()) {
    printf("Error: first block contains no tiles!\n");
    return;
  }
  if (tile_container->begin()->DataOrganisation() != TileWise) {
    printf("Error: first block contains no tiles!\n");
    return;
  }
  printf("Read pyramid with %d levels\n", pyramid.size());
  printf("Read first block with %d tiles\n", tile_container->size());

  // Derive new names of pyramid files
  name = (char *) malloc(256);
  sprintf(name, "%s%s", pyramid.Name(), app);
  pyramid.SetName(name);
  pyramid.DerivePointFileName(directory);
  pyramid.DeriveMetaDataFileName(directory);
  
  // Open pyramid point file
//#ifdef windows
  if ((fd_points = fopen64(pyramid.PointFile(), "wb")) == NULL) {
//#else
//  if ((fd_points = Open_Compressed_File(pyramid.PointFile(), "w")) == NULL) {
//#endif
    fprintf(stderr, "Could not open laser points file %s\n", pyramid.PointFile());
    return;
  }
  
  seek_offset = 0;
  // Loop over all pyramid levels
  for (block=pyramid.begin(); block!=pyramid.end(); block++) {
    
    // Loop over all tiles to copy points and set seek offsets in meta data
    tile_container = block->begin();
    for (tile=tile_container->begin(); tile!=tile_container->end(); tile++) {
      // Read tile points
      if (!tile->Read(tile->PointFile(), false)) {
        printf("Error reading points of tile %s\n", tile->Name());
        return;
      }
      
      // Write tile points to merged point file
      num_bytes = tile->Write(fd_points);
      if (num_bytes == 0) {
        printf("Error writing points of tile %s\n", tile->Name());
        return;
      }
      
      // Set seek offset in tile meta data
      tile->SetSeekOffset(seek_offset);
      seek_offset += num_bytes;
      printf("Wrote %d bytes of tile %s, total now %I64d\n", num_bytes,
             tile->Name(), seek_offset);
      tile->SetPointFile(pyramid.PointFile());
      
      // Erase tile points
      tile->ErasePoints();
    }
    
    // Construct new block file names
    sprintf(name, "%s%s", block->Name(), app);
    block->SetName(name);
    block->DeriveMetaDataFileName(directory);
    tiles_file = ComposeFileName(directory, block->Name(), "tiles");
  
    // Write block meta data (largely copied from LaserBlock::WriteMetaData)
    fd_block = fopen(block->MetaDataFile(), "w");
    if (fd_block == NULL) {
      fprintf(stderr, "Error opening block database %s\n", block->MetaDataFile());
      exit(0);
    }
    BNF_Write_String(fd_block, "laserblock", 0, NULL);
    BNF_Write_String(fd_block, "name", 2, block->Name());
    if (block->ControlPointsFileName())
      BNF_Write_String(fd_block, "control_points", 2, block->ControlPointsFileName());
    block->LaserPointsInfoReference().WriteMetaData(fd_block, 2);
    BNF_Write_String(fd_block, "tiles", 2, tiles_file);
    BNF_Write_String(fd_block, "endlaserblock", 0, NULL);
    fclose(fd_block);

    // Write meta data of all tiles
    fd_tiles = fopen(tiles_file, "w");
    if (fd_tiles == NULL) {
      fprintf(stderr, "Error opening meta data file of all tiles %s\n", tiles_file);
      exit(0);
    }
    for (tile=tile_container->begin(); tile!=tile_container->end(); tile++)
      tile->WriteMetaData(fd_tiles);
    fclose(fd_tiles);
  }
  
  // Write pyramid meta data
  pyramid.WriteMetaData(false, false, false);
}
