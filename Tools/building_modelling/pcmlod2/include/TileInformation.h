
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


#ifndef TILEINFORMATION_H
#define TILEINFORMATION_H

#include <stdio.h>
#include <stdlib.h>
#include "ObjectPoints.h"
#include "LineTopologies.h"
#include "LaserPyramid.h"
#include "QGLCanvas.h"

class TileInformation
{
  protected:
    /// Tile corners in world coordinate system
    ObjectPoints *tile_corners;

    /// Tile topology
    LineTopologies *tile_top;

    /// Data appearance
    DataAppearance *appearance;

  public:
    /// Default constructor
    TileInformation();

    /// Default destructor
    ~TileInformation() {};

    /// Update tile bounds
    void Update(const LaserPyramid &pyramid, QGLCanvas *canvas,
                bool refresh=true);

    /// Remove the tile bounds from the canvas
    void Clear(QGLCanvas *canvas, bool refresh);

    /// Return the world tile corner points
    ObjectPoints *TilePoints() {return tile_corners;}

    /// Return the world tile topology
    LineTopologies *TileTopologies() {return tile_top;}
};

#endif // TILEINFORMATION_H
