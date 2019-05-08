
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


#ifndef SELECTIONBOX_H
#define SELECTIONBOX_H

#include <stdio.h>
#include <stdlib.h>
#include <QPoint>
#include "QGLCanvas.h"
#include "DataBounds2D.h"

class SelectionBox
{
  protected:
    /// Box corners in canvas coordinate system
    QPoint canvas_corners[2];

    /// Box bounds in canvas coordinate system
    DataBounds2D canvas_box_bounds;

    /// Box corners in world coordinate system
    ObjectPoints *box_world_corners;

    /// Box topology
    LineTopologies *box_top;

    /// Data appearance
    DataAppearance *appearance;

    /// Valid data
    bool valid;

  public:
    /// Default constructor
    SelectionBox();

    /// Default destructor
    ~SelectionBox() {};

    /// Set a new corner
    void SetCorner(int id, const QPoint &canvas_corner, QGLCanvas *canvas,
                   bool refresh=true);

    /// Update the world coordinate corners for a new transformation
    void UpdateWorldCorners(const QGLCanvas *canvas);

    /// Update the canvas box bounds
    void UpdateBoxBounds();

    /// Remove the selection box from the canvas
    void Clear(QGLCanvas *canvas, bool refresh);

    /// Return the status of the selection rectangle on the canvas
    bool IsValid() const {return valid;}

    /// Check if a point is inside the selection rectangle
    bool InsideBox(const Position3D &world_pos, const QGLCanvas *canvas) const;

    /// Return the world box corner points
    ObjectPoints *BoxPoints() {return box_world_corners;}

    /// Return the world box topology
    LineTopologies *BoxTopologies() {return box_top;}
};

#endif // SELECTIONBOX_H
