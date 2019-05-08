
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


#include "SplitLine.h"

SplitLine::SplitLine()
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
  appearance = new DataAppearance(SplitLineData);

  // No segments yet
  segments = NULL;

  // By default snapping is on at 0.5 m
  snap = true;
  snap_distance = 0.5;

  // Data is not yet valid
  valid = false;
}

void SplitLine::Initialise(const ObjectPoints *outline_points,
                           LineTopologies::const_iterator outline_top,
                           QGLCanvas *canvas, bool refresh)
{
  LineSegments2D::const_iterator segment;

  // Remove old line directions
  if (line_directions.size())
    line_directions.erase(line_directions.begin(), line_directions.end());
  // Delete old segments, if any
  if (segments) delete segments;
  // Convert the polygon to line segments
  segments = new LineSegments2D(outline_points->ObjectPointsRef(),
                                outline_top->LineTopologyReference());
  // Generate new line directions
  for (segment=segments->begin(); segment!=segments->end(); segment++) {
    line_directions.push_back(segment->Direction());
    line_directions.push_back(segment->Direction().Normal());
  }
  // Set index to first line direction
  dir_index = 0;

  // Add the data to the canvas
  canvas->AddObjectData(line_ends, line_top, appearance, false, refresh);
  // Make sure to receive mouse motion events without buttons pressed
  canvas->setMouseTracking(true);
  valid = true;
}

void SplitLine::Update(const QPoint &canvas_pos, QGLCanvas *canvas,
                       bool refresh)
{
  Position3D                     world_pos;
  Line2D                         split_line;
  LineSegments2D::const_iterator segment;
  double                         scalar, cursor_scalar, scalar_low, scalar_high;
  Position2D                     intersection, map_pos[2], corner;
  ObjectPoints::iterator         line_end;
  int                            i;

  if (!valid) return;
  // Determine the split line in the XOY plane
  world_pos = canvas->Canvas2World(canvas_pos, 0.0);
  split_line =
    Line2D(Position2D(world_pos.vect2D() + line_directions[dir_index]),
           Position2D(world_pos.vect2D() - line_directions[dir_index]));
  // Determine the scalars of the part around the cursor
  cursor_scalar = split_line.Scalar(Position2D(world_pos.X(), world_pos.Y()));
  scalar_low    = cursor_scalar - 10000.0;
  scalar_high   = cursor_scalar + 10000.0;
  for (segment=segments->begin(); segment!=segments->end(); segment++) {
    if (segment->Intersect(split_line, intersection, 0.1)) {
      scalar = split_line.Scalar(intersection);
      if (scalar > cursor_scalar && scalar < scalar_high)
        scalar_high = scalar;
      else if (scalar < cursor_scalar && scalar > scalar_low)
        scalar_low = scalar;
    }
  }

  // Check if the end points are close to a polygon corner if snapping
  map_pos[0] = split_line.Position(scalar_low);
  map_pos[1] = split_line.Position(scalar_high);
  if (snap) {
    for (segment=segments->begin(); segment!=segments->end(); segment++) {
      corner = segment->BeginPoint();
      for (i=0; i<2; i++)
        if (Distance(corner, map_pos[i]) < snap_distance) map_pos[i] = corner;
    }
  }

  // Store the values in the 3D data structure
  for (i=0, line_end=line_ends->begin(); i<2; i++, line_end++) {
    line_end->X() = map_pos[i].X();
    line_end->Y() = map_pos[i].Y();
  }

  // Update the canvas
  if (refresh) canvas->update();
}

void SplitLine::Clear(QGLCanvas *canvas, bool refresh)
{
  // Remove the data from the canvas
  canvas->RemoveObjectData(appearance, refresh);
  // Only receive mouse motion events when a button is pressed
  canvas->setMouseTracking(false);
  // Delete the segments
  if (segments) {
    delete segments;
    segments = NULL;
  }
  valid = false;
}

void SplitLine::NextDirection(const QPoint &canvas_pos, QGLCanvas *canvas,
                              bool refresh)
{
  if (!valid) return;
  dir_index++;
  if (dir_index == (int) line_directions.size()) dir_index = 0;
  Update(canvas_pos, canvas, refresh);
}

void SplitLine::PreviousDirection(const QPoint &canvas_pos, QGLCanvas *canvas,
                                  bool refresh)
{
  if (!valid) return;
  if (dir_index == 0) dir_index = line_directions.size() - 1;
  else dir_index--;
  Update(canvas_pos, canvas, refresh);
}

LineSegment2D SplitLine::SplitSegment() const
{
  ObjectPoints::const_iterator p_begin, p_end;

  p_begin = line_ends->begin();
  p_end   = p_begin + 1;
  return LineSegment2D(Position2D(p_begin->vect2D()),
                       Position2D(p_end->vect2D()));
}
