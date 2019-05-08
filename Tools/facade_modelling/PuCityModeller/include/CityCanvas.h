
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


#ifndef CITYCANVAS_H
#define CITYCANVAS_H

#include "QGLCanvas.h"
#include "LaserPoints.h"
#include "SelectionBox.h"
#include "SplitLine.h"
#include "ExtensionLine.h"

enum MouseMode {PoseChangeMode, SelectMapMode, SelectMapPartitionMode,
                SelectModelMode, SelectModelPartMode, SelectLaserSegmentMode,
                SelectRectangleMode,
                SplitMapMode, SplitMapPartitionMode, ExtendLineMode,
                MoveNodeMode, NumModes, SelectPointMode};
typedef enum MouseMode MouseMode;

enum CityScaleLocation {NoScale, LeftBottomScale, CentreScale};
typedef enum CityScaleLocation;

class CityCanvas : public QGLCanvas
{
  Q_OBJECT

  protected:
    /// The canvas id
    int canvas_id;

    /// Mouse mode
    MouseMode mouse_mode;

    
    /// Split line
    SplitLine *split_line;

    /// Extension line
    ExtensionLine *extension_line;

    /// Move node line (implemented as a second extension line)
    ExtensionLine *move_node_line;
    
    /// Scale location
    CityScaleLocation scale_location;
    
    /// Scale tick distance
    double scale_tick_distance;

  public:
    /// Default constructor
    CityCanvas() {}

    /// Construct with id and parent widget
    CityCanvas(int id, QWidget *parent);

    /// Default destructor
    ~CityCanvas() {};
    
    /// Selection box
    SelectionBox *selection_box;
    
    Plane localplane;
    

  private:
    /// Select the object point nearest to the mouse position and omit signal
    void SelectObjectPoint(QMouseEvent *event, DataType type);

    /// Select the laser point nearest to the mouse position and omit signal
    void SelectLaserPoint(QMouseEvent *event);

  protected:
    /// Process mouse press events
    void mousePressEvent(QMouseEvent *event);

    /// Process mouse release events
    void mouseReleaseEvent(QMouseEvent *event);

    /// Process mouse motion events
    void mouseMoveEvent(QMouseEvent *event);

    /// Process key press events
    void keyPressEvent(QKeyEvent *event);

    /// Process key release events
    void keyReleaseEvent(QKeyEvent *event);

  public:
    /// Set the mouse mode
    void SetMode(MouseMode new_mode) {mouse_mode = new_mode;}

    /// Return the current mode
    MouseMode Mode() const {return mouse_mode;}

    /// Translate the selection mode into a selection data type
    DataType CorrespondingDataType(MouseMode mode) const;

    /// Determine the (unselected) data type corresponding to a selected type
    DataType CorrespondingBaseDataType(DataType selection_type) const;

    /// Determine the selected data type corresponding to a type
    DataType CorrespondingSelectedDataType(DataType type) const;

    /// Remove the selection rectangle
    void RemoveSelectionRectangle(bool refresh=true)
      {if (selection_box) selection_box->Clear(QGLCanvasPtr(), refresh);}

    /// Return the status of the selection rectangle
    bool HasValidSelectionRectangle() const
      {if (!selection_box) return false;
       return selection_box->IsValid();}

    /// Select data inside the selection rectangle
    void SelectInsideRectangle(const ObjectPoints *points,
                               PointNumberList &inside_point_numbers) const;

    /// Return the selection box corner points
    ObjectPoints *SelectionBoxPoints() {return selection_box->BoxPoints();}

    /// Return the selection box topology
    LineTopologies *SelectionBoxTopologies()
      {return selection_box->BoxTopologies();}

    /// Select laser data with the selection box
    void SelectLaserData(const LaserPoints &points,
                         LaserPoints &selected_points, bool inside) const;

    /// Set up the split line for a map (partition) outline
    void InitialiseSplitLine(const ObjectPoints *outline_points,
                             LineTopologies::const_iterator outline_top);

    /// Remove the split line
    void RemoveSplitLine(bool refresh=true)
      {if (split_line) split_line->Clear(QGLCanvasPtr(), refresh);}

    /// Initialise the extension line
    void InitialiseExtensionLine(ObjectPoints *points,
                                 LineTopsIterVector::iterator selected_line);

    /// Remove the extension line
    void RemoveExtensionLine(bool refresh=true)
      {if (extension_line) extension_line->Clear(QGLCanvasPtr(), refresh);}

    /// Return the status of the extension line
    bool HasValidExtensionLine() const
      {if (!extension_line) return false;
       return extension_line->IsValid();}

    /// Extend selected line using the extension line
    void ExtendSelectedLine(bool refresh=true);

    /// Initialise the moving point lines
    void InitialiseMovingPointLines(ObjectPoints *points,
                                    LineTopsIterVector::iterator selected_line);

    /// Reset the moving point lines to the nearest corner
    void ResetMovingPointLines(QMouseEvent *event);

    /// Remove the move node lines
    void RemoveMoveNodeLines(bool refresh=true);

    /// Move a node of the selected line
    void MoveNode(bool refresh=true);

    /// Return scale location
    CityScaleLocation ScaleBarLocation() const
      { return scale_location; }
      
    /// Set scale location
    void SetScaleLocation(CityScaleLocation new_location)
      { scale_location = new_location; }
      
    /// Return scale tick distance
    double ScaleBarTickDistance() const
      { return scale_tick_distance; }
      
    /// Set scale tick distance
    void SetScaleTickDistance(double new_distance)
      { scale_tick_distance = new_distance; }

    /// Draw the scale
    void DrawScale();
    
    /// Draw a scale line
    void DrawScaleLine(Position3D &pos1, Position3D &pos2, Vector3D &view_dir);

    /// Paint all data
    void PaintAllData();
        
  signals:
    /// Nearest object point selected
    void SelectedObjectPoint(const PointNumber &number,
                             const Position3D &map_pos,
                             DataType selection_type, DataType base_type);

    /// Nearest laser point selected
    void SelectedLaserPoint(const LaserPoint &point, DataType type);

    /// Request to toggle selection from nearby data buffer
    void ToggleSelectionRequest(DataType selection_type);

    /// A key has been pressed on the canvas
    void CanvasKeyPressed(QKeyEvent *event);

    /// Request to split a building (partition) outline
    void SplitRequest(MouseMode mode, const LineSegment2D &line);

    /// Request to delete the last edge
    void RequestLastEdgeDelete();

    /// Selected line has been closed using the extension line
    void LineHasBeenClosed(MouseMode);
    
      
    void pop_menu(const QPoint &); 
    
     /// Request to display information on nearest laser point
    void RequestForPointInformation(const LaserPoint &point);
    
    void AttractPoint(const LaserPoint &point);
    
    void AddCanvasTiePoint(const LaserPoint &point);
    
    void DeleteCanvasTiePoint();
};

#endif // CityCANVAS_H
