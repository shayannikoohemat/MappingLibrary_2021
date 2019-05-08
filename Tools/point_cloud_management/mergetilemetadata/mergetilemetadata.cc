
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
 Restructuring of tile meta data: meta data of all tiles is combined in a
 single file with extension .tiles

 Initial creation:
 Author : George Vosselman
 Date   : 04-09-2007

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
                         The main mergetilemetadata function
--------------------------------------------------------------------------------
*/

void mergetilemetadata(char *inblock, char *outblock, char *directory)
{
  char *tiles_file;
  LaserBlock::iterator tile_container;
  LaserUnit::iterator tile;
  FILE *fd;
  int success;
    
  // Read the input block and check whether there are tiles
  LaserBlock block = LaserBlock(inblock, &success);
  if (block.empty())
    printf("Error: this block contains no tiles!\n");
  tile_container = block.begin();
  if (tile_container->empty())
    printf("Error: this block contains no tiles!\n");
  if (tile_container->begin()->DataOrganisation() != TileWise)
    printf("Error: this block contains no tiles!\n");
  printf("Read block with %d tiles\n", tile_container->size());

  // Compose name of meta data with all tiles
  tiles_file = ComposeFileName(directory, block.Name(), "tiles");
  
  // Write block meta data (largely copied from LaserBlock::WriteMetaData)
  fd = fopen(outblock, "w");
  if (fd == NULL) {
    fprintf(stderr, "Error opening block database %s\n", outblock);
    exit(0);
  }
  BNF_Write_String(fd, "laserblock", 0, NULL);
  if (block.Name()) BNF_Write_String(fd, "name", 2, block.Name());
  if (block.ControlPointsFileName())
    BNF_Write_String(fd, "control_points", 2, block.ControlPointsFileName());
  block.LaserPointsInfoReference().WriteMetaData(fd, 2);
  BNF_Write_String(fd, "tiles", 2, tiles_file);
  BNF_Write_String(fd, "endlaserblock", 0, NULL);
  fclose(fd);

  // Write meta data of all tiles
  fd = fopen(tiles_file, "w");
  if (fd == NULL) {
    fprintf(stderr, "Error opening meta data file of all tiles %s\n", tiles_file);
    exit(0);
  }
  for (tile=tile_container->begin(); tile!=tile_container->end(); tile++)
    tile->WriteMetaData(fd);
  fclose(fd);
}
