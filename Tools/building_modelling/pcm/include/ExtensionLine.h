
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


#ifndef EXTENSIONLINE_H
#define EXTENSIONLINE_H

#include <stdio.h>
#include <stdlib.h>
#include "QGLCanvas.h"
#include "LineTopsIterVector.h"

class ExtensionLine
{
  protected:
    /// Points of the selected line
    ObjectPoints *selected_points;

    /// Topology of the selected line
    LineTopsIterVector::iterator selected_top;

    /// Point to be used as start of the extension line
    LineTopology::iterator start_point;

    /// Point to be moved
    LineTopology::iterator moving_point;

    /// Extension line ends in world coordinate system
    ObjectPoints *line_ends;

    /// Extension line topology
    LineTopologies *line_top;

    /// Data appearance
    DataAppearance *appearance;

    /// Snapping switch
    bool snap;

    /// Snapped point
    ObjectPoint *snapped_point;

    /// Valid data
    bool valid;

  public:
    /// Default constructor
    ExtensionLine();

    /// Default destructor
    ~ExtensionLine() {};

    /// Return the reference to this extension line
    ExtensionLine & ExtensionLineRef() {return *this;}

    /// Initialise the extension line from an existing line
    void Initialise(ObjectPoints *selected_line_points,
                    LineTopsIterVector::iterator selected_line_top,
                    LineTopology::iterator start_point_src,
                    LineTopology::iterator moving_point_src,
                    QGLCanvas *canvas, bool refresh);

    /// Initialise the selected points and lines data for an extension line
    void Initialise(ObjectPoints *selected_line_points,
                    LineTopsIterVector::iterator selected_line_top);

    /// Initialise the extension line for a new line from a mouse position
    void Initialise(QMouseEvent *event, QGLCanvas *canvas, bool refresh=false);

    /// Update the extension line for a new mouse position
    void Update(QMouseEvent *event, QGLCanvas *canvas, bool refresh);

    /// Extend the selected line with the current extension line
    void Extend(QGLCanvas *canvas, bool refresh);

    /// Remove the extension line from the canvas
    void Clear(QGLCanvas *canvas, bool refresh);

    /// Return the status of the extension line
    bool IsValid() const {return valid;}

    /// Set the snapping switch
    void Snap(bool value) {snap = value;}

    /// Check if selected line is closed
    bool SelectedLineIsClosed() const
      {return (*selected_top)->IsClosed();}

    /// Return data type of selected line
    DataType TypeOfData() const;

    /// Return the polygon points
    ObjectPoints * SelectedLinePoints()
      {return selected_points;}

    /// Return the polygon topology
    LineTopsIterVector::iterator SelectedLineTopology()
      {return selected_top;}

    /// Move the location of the next point to the mouse position
    void MoveSelectedNode(QGLCanvas *canvas, bool refresh);
    
    /// Derive point in selected model face based on mouse position
    Position3D PositionOnModelFace(QMouseEvent *event, QGLCanvas *canvas);
};
#endif // EXTENSIONLINE_H
