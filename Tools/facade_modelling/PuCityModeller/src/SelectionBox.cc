
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


#include "SelectionBox.h"

SelectionBox::SelectionBox()
{
  LineTopology *top;
  int          i;

  // Create the four corner points without dummy coordinates and variances
  box_world_corners = new ObjectPoints();
  for (i=0; i<4; i++)
    box_world_corners->push_back(ObjectPoint(0.0, 0.0, 0.0, i, 0.0, 0.0, 0.0,
                                             0.0, 0.0, 0.0));
  // Create the box topology
  top = new LineTopology();
  for (i=0; i<4; i++) top->push_back(PointNumber(i));
  top->push_back(PointNumber(0));
  box_top = new LineTopologies();
  box_top->push_back(*top);
  delete top;

  // Create the appearance of the box
  appearance = new DataAppearance(SelectionBoxData);

  // Data is not yet valid
  valid = false;
}

void SelectionBox::SetCorner(int id, const QPoint &new_canvas_corner, 
                             QGLCanvas *canvas, bool refresh)
{
  // Set new corner coordinates
  switch (id) {
    case 0 : canvas_corners[0] = new_canvas_corner;
    case 1 : canvas_corners[1] = new_canvas_corner;
             break;
    default: break;
  }

  // Update the world coordinates
  UpdateWorldCorners(canvas);

  // Update the canvas box bounds
  UpdateBoxBounds();

  // Add data to the canvas
  if (!valid) {
    valid = true;
    canvas->AddObjectData(box_world_corners, box_top, appearance, false, false);
  }
  if (refresh) canvas->update();
}

void SelectionBox::UpdateWorldCorners(const QGLCanvas *canvas)
{
  QPoint corner;
  ObjectPoints::iterator world_corner;

  corner.setX(canvas_corners[0].x());  corner.setY(canvas_corners[0].y());
  world_corner = box_world_corners->begin();
  world_corner->vect() = canvas->Canvas2World(corner, 1000).vect();
  corner.setX(canvas_corners[0].x());  corner.setY(canvas_corners[1].y());
  world_corner++;
  world_corner->vect() = canvas->Canvas2World(corner, 1000).vect();
  corner.setX(canvas_corners[1].x());  corner.setY(canvas_corners[1].y());
  world_corner++;
  world_corner->vect() = canvas->Canvas2World(corner, 1000).vect();
  corner.setX(canvas_corners[1].x());  corner.setY(canvas_corners[0].y());
  world_corner++;
  world_corner->vect() = canvas->Canvas2World(corner, 1000).vect();
}

void SelectionBox::UpdateBoxBounds()
{
  canvas_box_bounds.Initialise();
  canvas_box_bounds.Update(Position2D(canvas_corners[0].x(),
                                      canvas_corners[0].y()));
  canvas_box_bounds.Update(Position2D(canvas_corners[1].x(),
                                      canvas_corners[1].y()));
}

bool SelectionBox::InsideBox(const Position3D &world_pos,
                             const QGLCanvas *canvas) const
{
  if (!valid) return false;
  QPoint canvas_pos = canvas->World2Canvas(world_pos);
  return canvas_box_bounds.Inside(Position2D(canvas_pos.x(), canvas_pos.y()));
}

void SelectionBox::Clear(QGLCanvas *canvas, bool refresh)
{
  if (!valid) return; // Box is already removed
  canvas->RemoveObjectData(appearance, refresh);
  valid = false;
}
