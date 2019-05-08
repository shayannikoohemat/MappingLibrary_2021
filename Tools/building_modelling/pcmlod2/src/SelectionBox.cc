
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


#include "SelectionBox.h"

SelectionBox::SelectionBox(int shape_type)
{
  LineTopology *top;
  int          i, numpts;

  // Copy and check the type
  type = shape_type;
  if (type <0 || type > 1) {
  	printf("Invalid shape type %d in SelectionShape constructor\n", type);
  	exit(0);
  }
  
  // Create the corner points without dummy coordinates and variances
  shape_world_corners = new ObjectPoints();
  if (type == 0) numpts = 4;
  else {
    numpts = 16;
    radius = 10.0;
  }
  for (i=0; i<numpts; i++)
    shape_world_corners->push_back(ObjectPoint(0.0, 0.0, 0.0, i, 0.0, 0.0, 0.0,
                                               0.0, 0.0, 0.0));
  // Create the box topology
  top = new LineTopology();
  for (i=0; i<numpts; i++) top->push_back(PointNumber(i));
  top->push_back(PointNumber(0));
  shape_top = new LineTopologies();
  shape_top->push_back(*top);
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
  UpdateShapeBounds();

  // Add data to the canvas
  if (!valid) {
    valid = true;
    canvas->AddObjectData(shape_world_corners, shape_top, appearance, false, false);
  }
  if (refresh) canvas->update();
}

void SelectionBox::SetCircle(const QPoint &canvas_point, double canvas_radius,
                             QGLCanvas *canvas, bool refresh)
{
  // Save circle parameters
  canvas_centre = canvas_point;
  radius = canvas_radius;
  
  // Update the world coordinates
  UpdateWorldCorners(canvas);

  // Add data to the canvas
  if (!valid) {
    valid = true;
    canvas->AddObjectData(shape_world_corners, shape_top, appearance, false, false);
  }
  if (refresh) canvas->update();
}

void SelectionBox::UpdateWorldCorners(const QGLCanvas *canvas)
{
  QPoint                 corner;
  ObjectPoints::iterator world_corner;
  int                    i;
  double                 angle, increment=0.5 * atan(1.0);

  switch (type) {
  	case 0: // Rectangle
      corner.setX(canvas_corners[0].x());  corner.setY(canvas_corners[0].y());
      world_corner = shape_world_corners->begin();
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
      break;
      
    case 1: // Circle
      for (i=0, world_corner=shape_world_corners->begin(), angle=0.0;
	       i<16; i++, world_corner++, angle+=increment) {
	    corner.setX(canvas_centre.x() + radius * sin(angle));
	    corner.setY(canvas_centre.y() + radius * cos(angle));
	    world_corner->vect() = canvas->Canvas2World(corner, 1000).vect();
	  }
	  break;
	  
	default:
  	  printf("Warning: UpdateWorldCorners only implemented for rectangular selection shape\n");
  	  break;
  }
}

void SelectionBox::UpdateShapeBounds()
{
  if (type != 0) {
  	printf("Warning: UpdateShapeBounds only implemented for rectangular selection shape\n");
  	return;
  }
  
  canvas_shape_bounds.Initialise();
  canvas_shape_bounds.Update(Position2D(canvas_corners[0].x(),
                                        canvas_corners[0].y()));
  canvas_shape_bounds.Update(Position2D(canvas_corners[1].x(),
                                        canvas_corners[1].y()));
}

bool SelectionBox::InsideShape(const Position3D &world_pos,
                               const QGLCanvas *canvas) const
{
  if (!valid) return false;
  QPoint canvas_pos = canvas->World2Canvas(world_pos);
  switch (type) {
  	default:
  	case 0: // Rectangle
      return canvas_shape_bounds.Inside(Position2D(canvas_pos.x(), canvas_pos.y()));
    case 1: // Circle
      return Position2D(canvas_pos.x(), canvas_pos.y()).Distance(Position2D(canvas_centre.x(), canvas_centre.y())) <=
             radius;
  }
  return false; // Should not get here
}

void SelectionBox::Clear(QGLCanvas *canvas, bool refresh)
{
  if (!valid) return; // Shape is already removed
  canvas->RemoveObjectData(appearance, refresh);
  valid = false;
}
