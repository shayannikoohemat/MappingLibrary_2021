
/*
                  Copyright 2010 University of Twente
 
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


#include "TileInformation.h"

TileInformation::TileInformation()
{
  // Initialise pointers
  tile_corners = NULL;
  tile_top     = NULL;

  // Create the appearance of the tile boundaries
  appearance = new DataAppearance(TileBoundaryData);
}

void TileInformation::Update(const LaserPyramid &pyramid,
                             QGLCanvas *canvas, bool refresh)
{
  ObjectPoints2D tile_corners_2D;
  int            level;
  LineTopologies::iterator tile;
  
  // Create empty topology list
  if (tile_top == NULL) tile_top = new LineTopologies();
  else tile_top->Erase();
  
  // Collect tile boundaries
  pyramid.CollectUsedTileBoundaries(tile_corners_2D, *tile_top);

  // Convert to 3D
  if (tile_corners != NULL) tile_corners->Erase();
  tile_corners = new ObjectPoints(&tile_corners_2D, 100.0);
  
  // Add 1000000 times the pyramid level to the tile numbers
  level = pyramid.UsedLevel();
  for (tile=tile_top->begin(); tile!=tile_top->end(); tile++)
    tile->Number() = level * 1000000 + tile->Number();
  
  // Replace tile boundaries on canvas
  canvas->RemoveObjectData(appearance, false);
  canvas->AddObjectData(tile_corners, tile_top, appearance, false, refresh);
}

void TileInformation::Clear(QGLCanvas *canvas, bool refresh)
{
  canvas->RemoveObjectData(appearance, refresh);
}
