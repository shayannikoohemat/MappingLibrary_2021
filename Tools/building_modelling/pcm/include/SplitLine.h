
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


#ifndef SPLITLINE_H
#define SPLITLINE_H

#include <stdio.h>
#include <stdlib.h>
#include "QGLCanvas.h"
#include "LineSegments2D.h"

class SplitLine
{
  protected:
    /// Line ends in world coordinate system
    ObjectPoints *line_ends;

    /// Line topology
    LineTopologies *line_top;

    /// Data appearance
    DataAppearance *appearance;

    /// Line segments of the current map (partition) outline
    LineSegments2D *segments;

    /// Possible directions for the split line
    std::vector <Vector2D> line_directions;

    /// Index of current line direction
    int dir_index;

    /// Snap to polygon point if within some distance
    bool snap;

    /// Snap distance
    double snap_distance;

    /// Valid data
    bool valid;

  public:
    /// Default constructor
    SplitLine();

    /// Default destructor
    ~SplitLine() {};

    /// Initialise the split line for the specific map (partition) outline
    void Initialise(const ObjectPoints *outline_points, 
                    LineTopologies::const_iterator outline_top,
                    QGLCanvas *canvas, bool refresh);

    /// Update the split line for a new mouse position
    void Update(const QPoint &canvas_pos, QGLCanvas *canvas, bool refresh);

    /// Remove the split line from the canvas
    void Clear(QGLCanvas *canvas, bool refresh);

    /// Return the status of the split line
    bool IsValid() const {return valid;}

    /// Next preferred line direction
    void NextDirection(const QPoint &canvas_pos, QGLCanvas *canvas,
                       bool refresh);

    /// Previous preferred line direction
    void PreviousDirection(const QPoint &canvas_pos, QGLCanvas *canvas,
                           bool refresh);

    /// Return the split line in segment structure
    LineSegment2D SplitSegment() const;
};

#endif // SPLITLINE_H
