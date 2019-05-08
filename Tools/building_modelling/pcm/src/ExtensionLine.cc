
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


#include "ExtensionLine.h"
#include <QMouseEvent>

ExtensionLine::ExtensionLine()
{
  LineTopology *top;
  int          i;

  // Create the two line ends with dummy coordinates and variances
  line_ends = new ObjectPoints();
  for (i=0; i<2; i++)
    line_ends->push_back(ObjectPoint(0.0, 0.0, 0.0, i, 0.0, 0.0, 0.0,
                                     0.0, 0.0, 0.0));
  // Create the line topology
  top = new LineTopology();
  for (i=0; i<2; i++) top->push_back(PointNumber(i));
  line_top = new LineTopologies();
  line_top->push_back(*top);
  delete top;

  // Create the appearance of the box
  appearance = new DataAppearance(ExtensionLineData);

  // Data is not yet valid
  valid = false;
}

void ExtensionLine::Initialise(ObjectPoints *selected_line_points,
                               LineTopsIterVector::iterator selected_line_top,
                               LineTopology::iterator start_point_src,
                               LineTopology::iterator moving_point_src,
                               QGLCanvas *canvas, bool refresh)
{
  ObjectPoint *extension_point;
  ObjectPoints::iterator line_end;

  // Copy the information on the line
  valid           = false;
  selected_points = selected_line_points;
  selected_top    = selected_line_top;
  start_point     = start_point_src;
  moving_point    = moving_point_src;

  // Check if the moving point can be moved at all
  if (!(*selected_top)->IsClosed()) {
    if (moving_point == (*selected_top)->begin() &&
        start_point == (*selected_top)->end() - 1) return;
    if (moving_point == (*selected_top)->end() - 1 &&
        start_point == (*selected_top)->begin()) return;
  }

  // Copy coordinates of the start point of the selected polygon
  extension_point = selected_points->GetPoint(start_point->NumberRef());
  line_end = line_ends->begin();
  *line_end = *extension_point;
  line_end->Number() = 0;
  line_end++;
  *line_end = *extension_point;
  line_end->Number() = 1;
  
  // Add the data to the canvas
  canvas->AddObjectData(line_ends, line_top, appearance, false, refresh);
  valid = true;
}

void ExtensionLine::Initialise(ObjectPoints *selected_line_points,
                               LineTopsIterVector::iterator selected_line_top)
{
  // Copy the information on the line
  valid           = false;
  selected_points = selected_line_points;
  selected_top    = selected_line_top;
}

void ExtensionLine::Initialise(QMouseEvent *event, QGLCanvas *canvas,
                               bool refresh)
{
  Position3D             world_pos;
  ObjectPoints::iterator line_end;
  int                    number;

  if (!(*selected_top)->empty()) return;

  // Convert the mouse coordinates to the object space
  if (snap) {
    if (TypeOfData() != ModelData)
      snapped_point = canvas->NearestObjectPoint(event, MapData,
                                                 SelectedMapData);
    else
      snapped_point = canvas->NearestObjectPoint(event, LastModelData,
                                                 SelectedModelFaceData);
    world_pos.vect() = snapped_point->vect();
  }
  else {
    if (TypeOfData() != ModelData)
      // Determine the cursor position in the XOY plane for map (partition) data
      world_pos = canvas->Canvas2World(event->pos(), 0.0);
    else
      world_pos = PositionOnModelFace(event, canvas);
  }

  // Copy the coordinates of the mouse event
  line_end = line_ends->begin();
  line_end->X() = world_pos.X();
  line_end->Y() = world_pos.Y();
  line_end->Z() = 0.0;
  line_end->Number() = 0;
  line_end++;
  *line_end = *(line_end-1);
  line_end->Number() = 1;

  // Add the last point to the selected line
  if (snap)
    number = snapped_point->Number();
  else {
    if (selected_points->empty()) number = 1;
    else number = (selected_points->end()-1)->Number() + 1;
    selected_points->push_back(ObjectPoint(line_end->vect(),
                                           PointNumber(number),
                                           Covariance3D()));
  }
  (*selected_top)->push_back(PointNumber(number));

  // Add the data to the canvas
  canvas->AddObjectData(line_ends, line_top, appearance, false, refresh);
  valid = true;
}

void ExtensionLine::Update(QMouseEvent *event, QGLCanvas *canvas, bool refresh)
{
  Position3D                     world_pos, world_pos2;
  ObjectPoints::iterator         line_end, last_point;
  Vector3D                       normal;
  Plane                          plane;
  Line3D                         line;
  LineTopology::iterator         last_node;

  if (!valid) return;

  if (snap) {
    if (TypeOfData() != ModelData)
      snapped_point = canvas->NearestObjectPoint(event, MapData, SelectedMapData);
    else
      snapped_point = canvas->NearestObjectPoint(event, LastModelData,
                                                 SelectedModelFaceData);
    world_pos.vect() = snapped_point->vect();
  }
  else {
    if (TypeOfData() != ModelData)
      // Determine the cursor position in the XOY plane for map (partition) data
      world_pos = canvas->Canvas2World(event->pos(), 0.0);
    else
      world_pos = PositionOnModelFace(event, canvas);
  }
  
  // Store the position in the end point of the extension line
  line_end = line_ends->begin() + 1;
  line_end->X() = world_pos.X();
  line_end->Y() = world_pos.Y();
  if (TypeOfData() != ModelData) line_end->Z() = 0.0;
  else line_end->Z() = world_pos.Z();
 
  // Update the canvas
  if (refresh) canvas->update();
}

void ExtensionLine::Extend(QGLCanvas *canvas, bool refresh)
{
  ObjectPoints::iterator last_point;
  int                    number;
  bool                   add;

  if (!valid) return;

  // Check if the extension line has some length
  last_point = line_ends->begin() + 1;
  if (line_ends->begin()->X() == last_point->X() &&
      line_ends->begin()->Y() == last_point->Y() &&
      line_ends->begin()->Z() == last_point->Z()) return;

  // Add the last point to the selected line
  if (snap) {
    number = snapped_point->Number();
  }
  else {
    number = (selected_points->end()-1)->Number() + 1;
    selected_points->push_back(ObjectPoint(last_point->vect(),
                                           PointNumber(number),
                                           Covariance3D()));
  }
  
  // Only add point number if the snapped point is not the same as the
  // previous point
  add = true;
  if (!(*selected_top)->empty())
    if (((*selected_top)->end()-1)->Number() == number && snap) add = false;
  if (add) (*selected_top)->push_back(PointNumber(number));
  
  // Reset the extension line
  line_ends->begin()->vect() = last_point->vect();

  // Update the canvas
  if (refresh) canvas->update();
}

void ExtensionLine::Clear(QGLCanvas *canvas, bool refresh)
{
  // Remove the data from the canvas
  canvas->RemoveObjectData(appearance, refresh);
  valid = false;
}

DataType ExtensionLine::TypeOfData() const
{
  if ((*selected_top)->Label() == MapLabel) return MapData;
  else if ((*selected_top)->Label() == MapPartitionLabel)
    return MapPartitionData;
  else if ((*selected_top)->Label() == RoofLabel ||
           (*selected_top)->Label() == WallLabel)
    return ModelData;
  else
    printf("Unknown line label in ExtensionLine::TypeOfData.\n");
  return NumDataTypes;
}

void ExtensionLine::MoveSelectedNode(QGLCanvas *canvas, bool refresh)
{
  ObjectPoints::iterator end_point, previous_point, next_point;
  int                    number=0;
  Position3D             new_location;
  LineTopology::iterator previous_node, selected_node, next_node;
  bool                   delete_node;

  // Determine the relevant nodes
  selected_node  = moving_point;
  previous_node  = (*selected_top)->PreviousNode(selected_node);
  next_node      = (*selected_top)->NextNode(selected_node);
  previous_point = selected_points->PointIterator(*previous_node);
  next_point     = selected_points->PointIterator(*next_node);

  // Determine the new location
  if (snap) {
    new_location = snapped_point->vect();
    number = snapped_point->Number();
  }
  else {
    end_point = line_ends->begin() + 1;
    new_location = end_point->vect();
  }

  // If the mouse location coincides with the previous or next node,
  // just delete the current node.
  delete_node = false;
  if (snap) {
    if (number == previous_node->Number() ||
        number == next_node->Number()) delete_node = true;
  }
  else {
    if (previous_point->Distance(new_location) == 0.0 ||
        next_point->Distance(new_location) == 0.0) delete_node = true;
  }
  if (delete_node) {
    if (selected_node == (*selected_top)->begin() &&
        (*selected_top)->IsClosed()) {
      printf("Closed pol not yet implemented\n"); return;
    }
    (*selected_top)->erase(selected_node);
  }
  
  else {
    if (!snap) {
      number = (selected_points->end()-1)->Number() + 1;
      selected_points->push_back(ObjectPoint(end_point->vect(),
                                             PointNumber(number),
                                             Covariance3D()));
    }
    if (selected_node == (*selected_top)->begin() &&
        (*selected_top)->IsClosed()) {
      printf("Closed pol not yet implemented\n"); return;
    }
    *selected_node = PointNumber(number);
  }

  // Update the canvas
  if (refresh) canvas->update();
}

Position3D ExtensionLine::PositionOnModelFace(QMouseEvent *event, QGLCanvas *canvas)
{
  Position3D                     world_pos, world_pos2;
  ObjectPoints::iterator         last_point;
  Vector3D                       normal;
  Plane                          plane;
  Line3D                         line;
  LineTopology::iterator         last_node;

  switch ((int) (*selected_top)->size()) {
    case 0: // Take point in XOY plane
      world_pos = canvas->Canvas2World(event->pos(), 0.0);
      break;
    case 1: // Take point in horizontal plane through last point
    case 2:
      last_node = (*selected_top)->end() - 1;
      last_point = selected_points->PointIterator(*last_node);
      world_pos = canvas->Canvas2World(event->pos(), last_point->Z());
      break;
    default: // Estimate plane through points and get mouse position in plane
      // Determine plane of the object face
      last_node = (*selected_top)->end() - 1;
      last_point = selected_points->PointIterator(*last_node);
      normal = (*selected_top)->Normal(*selected_points, 1);
      plane = Plane(last_point->vect(), normal);
      // Determine line perpendicular to canvas
      world_pos = canvas->Canvas2World(event->pos(), 0.0);
      world_pos2 = canvas->Canvas2World(event->pos(), 100.0);
      if (world_pos.Z() == world_pos2.Z()) // Problem: view along Z-axis
        world_pos = canvas->Canvas2World(event->pos(), last_point->Z());
      else {
        line = Line3D(world_pos, world_pos2);
        if (!IntersectLine3DPlane(line, plane, world_pos))
          world_pos = canvas->Canvas2World(event->pos(), last_point->Z());
      }
      break;
  }
  return world_pos;
}
