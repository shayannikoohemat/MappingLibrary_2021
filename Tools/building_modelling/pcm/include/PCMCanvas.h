
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


#ifndef PCMCANVAS_H
#define PCMCANVAS_H

#include <QMenu>
#include "QGLCanvas.h"
#include "LaserPoints.h"
#include "SelectionBox.h"
#include "SplitLine.h"
#include "ExtensionLine.h"
#include "LaserPyramid.h"

enum MouseMode {PoseChangeMode, PoseChangePerspectiveMode,
                SelectMapMode, SelectMapPartitionMode,
                SelectModelMode, SelectModelPartMode, 
                SelectModelFaceMode, SelectLaserPointMode,
				SelectLaserSegmentMode, SelectRectangleMode,
                SplitMapMode, SplitMapPartitionMode, ExtendLineMode,
                MoveNodeMode, BrowseMode, NumModes};
typedef enum MouseMode MouseMode;

enum PCMScaleLocation {NoScale, LeftBottomScale, CentreScale};
typedef enum PCMScaleLocation PCMScaleLocation;

class PCMCanvas : public QGLCanvas
{
  Q_OBJECT

  protected:
    /// The canvas id
    int canvas_id;

    /// Mouse mode
    MouseMode mouse_mode;

    /// Selection box
    SelectionBox *selection_box;
    
    /// Selection circle
    SelectionBox *selection_circle;
    
    /// Split line
    SplitLine *split_line;

    /// Extension line
    ExtensionLine *extension_line;

    /// Move node line (implemented as a second extension line)
    ExtensionLine *move_node_line;
    
    /// Scale location
    PCMScaleLocation scale_location;
    
    /// Scale tick distance
    double scale_tick_distance;
    
    /// Pop-up menu
    QMenu *popup_menu;

  public:
    /// Default constructor
    PCMCanvas() {}

    /// Construct with id and parent widget
    PCMCanvas(int id, QWidget *parent, bool create_popup_menu=false);

    /// Default destructor
    ~PCMCanvas() {};

  private:
    /// Select the object point nearest to the mouse position and emit signal
    void SelectObjectPoint(QMouseEvent *event, DataType type);

    /// Select the laser point nearest to the mouse position and emit signal
    void SelectLaserPoint(QMouseEvent *event);

    /// Select all laser point in selection circle and emit signal
    void SelectLaserPointsInCircle(QMouseEvent *event, bool add);

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
    
    /// Process mouse wheel events
    void wheelEvent(QWheelEvent *event);

  public:
    /// Set the mouse mode
    void SetMode(MouseMode new_mode) {mouse_mode = new_mode;}

    /// Return the current mode
    MouseMode Mode() const {return mouse_mode;}

    /// Return the pop-up menu
    QMenu *PopUpMenu() {return popup_menu;}
    
    /// Translate the selection mode into a selection data type
    DataType CorrespondingDataType(MouseMode mode) const;

    /// Determine the (unselected) data type corresponding to a selected type
    DataType CorrespondingBaseDataType(DataType selection_type) const;

    /// Determine the selected data type corresponding to a type
    DataType CorrespondingSelectedDataType(DataType type) const;

// Selection in rectangle

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
    ObjectPoints *SelectionBoxPoints() {return selection_box->ShapePoints();}

    /// Return the selection box topology
    LineTopologies *SelectionBoxTopologies()
      {return selection_box->ShapeTopologies();}

    /// Select laser data with the selection box
    void SelectLaserDataInRectangle(LaserPoints &points,
                                    LaserPoints &selected_points, bool inside,
                                    bool set_IsSelectedTag) const;

// Selection in circle

    /// Remove the selection circle
    void RemoveSelectionCircle(bool refresh=true)
      {if (selection_circle) selection_circle->Clear(QGLCanvasPtr(), refresh);}

    /// Return the status of the selection circle
    bool HasValidSelectionCircle() const
      {if (!selection_circle) return false;
       return selection_circle->IsValid();}


// Split line

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
    PCMScaleLocation ScaleBarLocation() const
      { return scale_location; }
      
    /// Set scale location
    void SetScaleLocation(PCMScaleLocation new_location)
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

    /// Shift the canvas
    void TranslateCanvas(float x_perc, float y_perc, bool refresh);
   
    /// Scale the canvas
    void ScaleCanvas(float scale_factor, bool refresh);
   
    /// Scale the focal length for the perspective projection
    void ResizeFocalLengthCanvas(float scale_factor, bool refresh);
   
  signals:
    /// Nearest object point selected
    void SelectedObjectPoint(const PointNumber &number,
                             const Position3D &map_pos,
                             DataType selection_type, DataType base_type);

    /// Laser point selected
    void SelectedLaserPoints(LaserPoints *points, bool add);
    
    /// Laser segment selected
    void SelectedLaserSegment(const LaserPoint &point, DataType type);
    
    /// Request to display information on nearest laser point
    void RequestForPointInformation(const LaserPoint &point);

    /// Request to display information on nearest object point
    void RequestForPointInformation(const ObjectPoint &point, DataType type);

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
    
    /// New canvas bounds (used for browsing in pyramid)
    void NewCanvasBounds(const DataBounds2D &);
    
    /// A model face has been edited
    void LastModelDataHasBeenEdited();
};

#endif // PCMCANVAS_H
