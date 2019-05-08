
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
  	/// Shape type (0=rectangle, 1=circle)
  	char type;
  	
    /// Shape corners in canvas coordinate system
    QPoint canvas_corners[2];

    /// Shape bounds in canvas coordinate system
    DataBounds2D canvas_shape_bounds;

    /// Circle centre in canvas coordinate system
    QPoint canvas_centre;
    
    /// Radius of selection circle
    double radius;
    
    /// Shape corners in world coordinate system
    ObjectPoints *shape_world_corners;

    /// Shape topology
    LineTopologies *shape_top;

    /// Data appearance
    DataAppearance *appearance;

    /// Valid data
    bool valid;

  public:
  	/// Default constructor
  	SelectionBox() {SelectionBox(0);}
  	
    /// Constructor
    SelectionBox(int shape_type);

    /// Default destructor
    ~SelectionBox() {};

    /// Set a new corner
    void SetCorner(int id, const QPoint &canvas_point, QGLCanvas *canvas,
                   bool refresh=true);

    /// Set circle
    void SetCircle(const QPoint &canvas_point, double canvas_radius, 
	               QGLCanvas *canvas, bool refresh=true);

    /// Return circle centre
    QPoint CircleCentre() const {return canvas_centre;}
    
    /// Return circle radius
    double CircleRadius() const {return radius;}
    
    /// Update the world coordinate corners for a new transformation
    void UpdateWorldCorners(const QGLCanvas *canvas);

    /// Update the canvas box bounds
    void UpdateShapeBounds();

    /// Remove the selection shape from the canvas
    void Clear(QGLCanvas *canvas, bool refresh);

    /// Return the status of the selection shape on the canvas
    bool IsValid() const {return valid;}

    /// Check if a point is inside the selection shape
    bool InsideShape(const Position3D &world_pos, const QGLCanvas *canvas) const;

    /// Return the world shape corner points
    ObjectPoints *ShapePoints() {return shape_world_corners;}

    /// Return the world shape topology
    LineTopologies *ShapeTopologies() {return shape_top;};
};

#endif // SELECTIONBOX_H
