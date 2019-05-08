
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


#include "PCMCanvas.h"
#include <QMouseEvent>

PCMCanvas::PCMCanvas(int id, QWidget *parent,
                     bool create_popup_menu) : QGLCanvas(parent)
{
  canvas_id        = id;
  mouse_mode       = PoseChangeMode;
  selection_box    = NULL;
  selection_circle = NULL;
  split_line       = NULL;
  extension_line   = NULL;
  move_node_line   = NULL;
  scale_location   = NoScale;
  scale_tick_distance = 100.0;
  if (create_popup_menu) popup_menu = new QMenu();
  else popup_menu = NULL;
}

void PCMCanvas::keyPressEvent(QKeyEvent *event)
{
  // First give key to the QGLCanvas
  QGLCanvas::keyPressEvent(event);
  // Let the main application deal with the short cut keys
  emit CanvasKeyPressed(event);
}

void PCMCanvas::keyReleaseEvent(QKeyEvent *event)
{
  // Give key to QGLCanvas
  QGLCanvas::keyReleaseEvent(event);
}

void PCMCanvas::mousePressEvent(QMouseEvent *event)
{
  DataType                    selection_type;
  MouseMode                   real_mode = mouse_mode;
  LaserPoints::const_iterator nearest_laser_point;
  DataType                    type;
  ObjectPoint                 *nearest_object_point;

  // Always let QGLCanvas set the button_down markers
  QGLCanvas::mousePressEvent(event);

  if (CtrlKeyDown()) real_mode = PoseChangeMode;
  switch (real_mode) {
    case PoseChangeMode: break; // Nothing to do for PCMCanvas
    case PoseChangePerspectiveMode: break; // Nothing to do for PCMCanvas
    case BrowseMode: break; // Nothing to do for PCMCanvas
    case SelectMapMode:
    case SelectMapPartitionMode:
    case SelectModelMode:
    case SelectModelPartMode:
    case SelectModelFaceMode:
      selection_type = CorrespondingDataType(real_mode);
      switch (event->button()) {
        case Qt::LeftButton:
          SelectObjectPoint(event, selection_type); break;
        case Qt::MidButton:
          type = CorrespondingBaseDataType(selection_type);
          nearest_object_point = NearestObjectPoint(event, type, selection_type);
          if (nearest_object_point != NULL)
            emit RequestForPointInformation(*nearest_object_point, type);
          break;
        case Qt::RightButton:
          emit ToggleSelectionRequest(selection_type); break;
        default: break;
      }
      break;
    case SelectLaserPointMode:
      if (!selection_circle) selection_circle = new SelectionBox(1);
      switch (event->button()) {
        case Qt::LeftButton:
          SelectLaserPointsInCircle(event, true);
          break;
        case Qt::MidButton:
          if (NearestLaserPoint(event, nearest_laser_point, type))
            emit RequestForPointInformation(*nearest_laser_point);
          break;
        case Qt::RightButton:
          SelectLaserPointsInCircle(event, false);
          break;
        default: break;
      }
      break;
    case SelectLaserSegmentMode:
      switch (event->button()) {
        case Qt::LeftButton:
          SelectLaserPoint(event); break;
        case Qt::MidButton:
          if (NearestLaserPoint(event, nearest_laser_point, type))
            emit RequestForPointInformation(*nearest_laser_point);
          break;
        case Qt::RightButton:
          popup_menu->exec(QCursor::pos()); break;
        default: break;
      }
      break;
    case SelectRectangleMode:
      switch (event->button()) {
        case Qt::LeftButton:
          if (!selection_box) selection_box = new SelectionBox(0);
          selection_box->SetCorner(0, event->pos(), QGLCanvasPtr()); break;
        case Qt::MidButton:
          if (NearestLaserPoint(event, nearest_laser_point, type))
            emit RequestForPointInformation(*nearest_laser_point);
          break;
        case Qt::RightButton:
          popup_menu->exec(QCursor::pos()); break;
        default: break;
      }
      break;
    case SplitMapMode:
    case SplitMapPartitionMode:
      switch (event->button()) {
        case Qt::LeftButton:
          if (split_line->IsValid())
            split_line->NextDirection(event->pos(), QGLCanvasPtr(), true);
          break;
        case Qt::MidButton:
          if (split_line->IsValid())
            split_line->PreviousDirection(event->pos(), QGLCanvasPtr(), true);
          break;
        case Qt::RightButton:
          if (split_line->IsValid())
            emit SplitRequest(real_mode, split_line->SplitSegment());
          break;
        default:
          break;
      }
      break;
    case ExtendLineMode:
      switch (event->button()) {
        case Qt::LeftButton:
          extension_line->Snap(false);
          if (!extension_line->IsValid())
            extension_line->Initialise(event, QGLCanvasPtr());
          mouseMoveEvent(event);
          break;
        case Qt::MidButton:
          extension_line->Snap(true);
          if (!extension_line->IsValid())
            extension_line->Initialise(event, QGLCanvasPtr());
          mouseMoveEvent(event);
          break;
        case Qt::RightButton:
          // Undo last line extension
          emit RequestLastEdgeDelete();
          break;
        default:
          break;
      }
      break;
    case MoveNodeMode:
      switch (event->button()) {
        case Qt::LeftButton:
          ResetMovingPointLines(event);
          extension_line->Snap(false);
          move_node_line->Snap(false);
          mouseMoveEvent(event);
          break;
        case Qt::MidButton:
          ResetMovingPointLines(event);
          extension_line->Snap(true);
          move_node_line->Snap(true);
          mouseMoveEvent(event);
          break;
        case Qt::RightButton:
          // Undo action
          printf("Undo action to be implemented\n");
          break;
        default:
          break;
      }
      break;
    default:
      printf("Mouse mode %d not implemented in PCMCanvas::mousePressEvent\n",
             mouse_mode);
  }
}

void PCMCanvas::mouseReleaseEvent(QMouseEvent *event)
{
  MouseMode real_mode = mouse_mode;

  // Always let QGLCanvas set the button_down markers
  QGLCanvas::mouseReleaseEvent(event);

  if (CtrlKeyDown()) real_mode = PoseChangeMode;
  switch (real_mode) {
    case ExtendLineMode:
      switch (event->button()) {
        case Qt::LeftButton:
        case Qt::MidButton:
          ExtendSelectedLine(true);
        default:
          break;
      }
      break;
    case MoveNodeMode:
      switch (event->button()) {
        case Qt::LeftButton:
        case Qt::MidButton:
          MoveNode(true);
          // Move node action to be implemented
          break;
        default:
          break;
      }
      break;
    default: // Nothing to do for all other modes
      break;
  }
}

void PCMCanvas::TranslateCanvas(float x_perc, float y_perc, bool refresh)
{
  // The last argument should be false. refresh != refresh is used to avoid
  // a compiler warning for not using this variable.
  QGLCanvas::TranslateCanvas(x_perc, y_perc, refresh != refresh);
  emit NewCanvasBounds(BoundsXY());
  QGLCanvas::update();
}

void PCMCanvas::ScaleCanvas(float scale_factor, bool refresh)
{
  // The last argument should be false. refresh != refresh is used to avoid
  // a compiler warning for not using this variable.
  QGLCanvas::ScaleCanvas(scale_factor, refresh != refresh);
  emit NewCanvasBounds(BoundsXY());
  QGLCanvas::update();
}

void PCMCanvas::ResizeFocalLengthCanvas(float scale_factor, bool refresh)
{
  // The last argument should be false. refresh != refresh is used to avoid
  // a compiler warning for not using this variable.
  QGLCanvas::ResizeFocalLengthCanvas(scale_factor, refresh != refresh);
  emit NewCanvasBounds(BoundsXY());
  QGLCanvas::update();
}

void PCMCanvas::mouseMoveEvent(QMouseEvent *event)
{
  MouseMode                   real_mode = mouse_mode;
  LaserPoints::const_iterator nearest_laser_point;
  DataType                    type, selection_type;
  ObjectPoint                 *nearest_object_point;

  if (CtrlKeyDown()) real_mode = PoseChangeMode;
  switch (real_mode) {
    case PoseChangeMode:
      QGLCanvas::mouseMoveEvent(event); // Event for QGLCanvas
      if (selection_box) {
        if (selection_box->IsValid()) {
          selection_box->UpdateWorldCorners(QGLCanvasPtr());
          QGLCanvas::update();
        }
      }
      break;
    case PoseChangePerspectiveMode:
      QGLCanvas::mouseMoveEvent(event);
      break;
    case BrowseMode:
      if (button_down[1] || button_down[2]) {
        QGLCanvas::mouseMoveEvent(event, false);
        emit NewCanvasBounds(BoundsXY());
        QGLCanvas::update();
      }
      break;
    case SelectMapMode:
    case SelectMapPartitionMode:
    case SelectModelMode:
    case SelectModelPartMode:
    case SelectModelFaceMode:
      if (button_down[1] || // Middle button down, info on nearest object point
          (button_down[0] && button_down[2])) { 
        selection_type = CorrespondingDataType(real_mode);
        type = CorrespondingBaseDataType(selection_type);
        nearest_object_point = NearestObjectPoint(event, type, selection_type);
        if (nearest_object_point != NULL)
          emit RequestForPointInformation(*nearest_object_point, type);
      }  
      break;
    case SelectLaserPointMode:
      if (!selection_circle) selection_circle = new SelectionBox(1);
      selection_circle->SetCircle(event->pos(), selection_circle->CircleRadius(),
		                          QGLCanvasPtr());
      if (button_down[0] && !button_down[2]) { // Left button down, add points in circle
		SelectLaserPointsInCircle(event, true);
      }
      else if (button_down[1] || // Middle button down, info on nearest laser point
               (button_down[0] && button_down[2])) {
        if (NearestLaserPoint(event, nearest_laser_point, type))
          emit RequestForPointInformation(*nearest_laser_point);
      }
      else if (button_down[2] && !button_down[0]) { // Right button down, remove points in circle
        SelectLaserPointsInCircle(event, false);
      }
      else {
      	update();
      }
      break;
    case SelectLaserSegmentMode:
      if (button_down[1] || // Middle button down, info on nearest laser point
          (button_down[0] && button_down[2]))
        if (NearestLaserPoint(event, nearest_laser_point, type))
          emit RequestForPointInformation(*nearest_laser_point);
      break;
    case SelectRectangleMode:
      if (button_down[0]) // Left button down, change selection box size
        selection_box->SetCorner(1, event->pos(), QGLCanvasPtr());
      else if (button_down[1]) // Middle button down, info on nearest laser point
        if (NearestLaserPoint(event, nearest_laser_point, type))
          emit RequestForPointInformation(*nearest_laser_point);
      break;
    case SplitMapMode:
    case SplitMapPartitionMode:
      split_line->Update(event->pos(), QGLCanvasPtr(), true);
      break;
    case ExtendLineMode:
      extension_line->Update(event, this, true);
      break;
    case MoveNodeMode:
      extension_line->Update(event, this, false);
      move_node_line->Update(event, this, false);
      update();
      break;
    default:
      break;
  }
}

void PCMCanvas::wheelEvent(QWheelEvent *event)
{
  MouseMode real_mode = mouse_mode;

  if (CtrlKeyDown()) real_mode = PoseChangeMode;
  if (real_mode == BrowseMode) {
    QGLCanvas::wheelEvent(event, false);
    emit NewCanvasBounds(BoundsXY());
    QGLCanvas::update();
  }
  else if (real_mode == SelectLaserPointMode) {
    if (!selection_circle) selection_circle = new SelectionBox(1);
  	if (event->delta() < 0) {
  	  if (selection_circle->CircleRadius() > 1.5) {
        selection_circle->SetCircle(event->pos(),
		                            selection_circle->CircleRadius() - 1.0,
		                            QGLCanvasPtr());
	  }
  	}
  	else {
      selection_circle->SetCircle(event->pos(),
		                          selection_circle->CircleRadius() + 1.0,
		                          QGLCanvasPtr());
  	}
  }
  else
    QGLCanvas::wheelEvent(event);
}

void PCMCanvas::SelectObjectPoint(QMouseEvent *event, DataType selection_type)
{
  ObjectPoint *nearest_point;
  DataType    type;
  Position3D  world_pos;

  // Determine base data type
  type = CorrespondingBaseDataType(selection_type);

  // Get nearest point
  nearest_point = NearestObjectPoint(event, type, selection_type);
  if (nearest_point == NULL) return;

  // Project the canvas position to the world's X0Y plane
  world_pos = Canvas2World(QPoint(event->x(), event->y()), 0.0);

  // Let the main application deal with the rest of the selection request
  emit SelectedObjectPoint(nearest_point->NumberRef(), world_pos,
                           selection_type, type);
}

void PCMCanvas::SelectLaserPoint(QMouseEvent *event)
{
  LaserPoints::const_iterator nearest_point;
  DataType                    type;

  // Get the nearest laser point
  if (!NearestLaserPoint(event, nearest_point, type)) return;

  // Let the main application deal with the rest of the selection request
  emit SelectedLaserSegment(*nearest_point, type);
}

void PCMCanvas::SelectLaserPointsInCircle(QMouseEvent *event, bool add)
{
  LaserPoints                 *points = new LaserPoints();
  
  // Get all points inside the circle
  SelectLaserPointsWithinRadius(*points, event, selection_circle->CircleRadius());
    
  // Let the main application deal with the rest of the selection request
  emit SelectedLaserPoints(points, add);
}

DataType PCMCanvas::CorrespondingDataType(MouseMode mode) const
{
  return (DataType) (mode - SelectMapMode + SelectedMapData);
}

DataType PCMCanvas::CorrespondingBaseDataType(DataType type) const
{
  switch (type) {
    case MapData:
    case SelectedMapData:
      return MapData;
    case MapPartitionData:
    case SelectedMapPartitionData:
      return MapPartitionData;
    case ModelData:
    case SelectedModelData:
    case SelectedModelPartData:
      return ModelData;
    case LastModelData:
    case SelectedModelFaceData:
      return LastModelData;
    case LaserData:
    case SelectedLaserData:
      return LaserData;
    default:
      break;
      printf("Error: Invalid data type in PointCloudMapper::CorrespondingBaseDataType\n");
  }
  return MapData; // We should not get here. Just to satisfy the compiler
}

DataType PCMCanvas::CorrespondingSelectedDataType(DataType type) const
{
  switch (type) {
    case MapData:
    case SelectedMapData:
      return SelectedMapData;
    case MapPartitionData:
    case SelectedMapPartitionData:
      return SelectedMapPartitionData;
    case ModelData:
    case SelectedModelData:
      return SelectedModelData;
    case SelectedModelPartData:
      return SelectedModelPartData;
    case LastModelData:
    case SelectedModelFaceData:
      return LastModelData;
    case LaserData:
    case SelectedLaserData:
      return SelectedLaserData;
    default:
      break;
      printf("Error: Invalid data type in PointCloudMapper::CorrespondingSelectedDataType\n");
  }
  return SelectedMapData; // We should not get here. Just to satisfy compilers
}

void PCMCanvas::SelectInsideRectangle(const ObjectPoints *points,
                                      PointNumberList &inside_points) const
{
  ObjectPoints::const_iterator point;

  if (!HasValidSelectionRectangle()) return;
  for (point=points->begin(); point!=points->end(); point++)
    if (selection_box->InsideShape(point->Position3DRef(), QGLCanvasPtr()))
      inside_points.push_back(point->NumberRef());
}

void PCMCanvas::SelectLaserDataInRectangle(LaserPoints &points,
                                           LaserPoints &selected_points,
                                           bool select_inside,
										   bool set_IsSelectedTag) const
{
  LaserPoints::iterator point;
  bool                  inside;

  if (!HasValidSelectionRectangle()) return;
  for (point=points.begin(); point!=points.end(); point++) {
    inside = selection_box->InsideShape(point->Position3DRef(), QGLCanvasPtr());
    if ((inside && select_inside) || (!inside && !select_inside)) {
      selected_points.push_back(*point);
      if (set_IsSelectedTag) point->Select();
    }
    else
      if (set_IsSelectedTag) point->RemoveAttribute(IsSelectedTag);
  }
}

void PCMCanvas::InitialiseSplitLine(const ObjectPoints *outline_points,
                                    LineTopologies::const_iterator outline_top)
{
  if (!split_line) split_line = new SplitLine();
  split_line->Initialise(outline_points, outline_top, QGLCanvasPtr(), false);
}

void PCMCanvas::InitialiseExtensionLine(ObjectPoints *points,
                                     LineTopsIterVector::iterator selected_line)
{
  if (!extension_line) extension_line = new ExtensionLine();
  if ((*selected_line)->empty())
    extension_line->Initialise(points, selected_line);
  else
    extension_line->Initialise(points, selected_line, (*selected_line)->end()-1,
                               (*selected_line)->end(), QGLCanvasPtr(), false);
}

void PCMCanvas::ExtendSelectedLine(bool refresh)
{
  if (!extension_line) return;
  extension_line->Extend(QGLCanvasPtr(), false);
  if (extension_line->SelectedLineIsClosed())
    switch (extension_line->TypeOfData()) {
      case MapData:
        emit LineHasBeenClosed(SelectMapMode);
        break;
      case MapPartitionData:
        emit LineHasBeenClosed(SelectMapPartitionMode);
        break;
      case ModelData: // Editing model data
        emit LineHasBeenClosed(SelectModelFaceMode);
        break;
      default:
        break;
    }
  if (refresh) update();
  if (extension_line->TypeOfData() == ModelData)
    emit LastModelDataHasBeenEdited();
}

void PCMCanvas::InitialiseMovingPointLines(ObjectPoints *points,
                                LineTopsIterVector::iterator selected_line)
{
  // Make sure both lines exist
  if (!extension_line) extension_line = new ExtensionLine();
  if (!move_node_line) move_node_line = new ExtensionLine();
  // Dummy initialisation to transfer the point and line data
  extension_line->Initialise(points, selected_line, (*selected_line)->begin(),
                             (*selected_line)->end(), QGLCanvasPtr(), false);
  move_node_line->Initialise(points, selected_line, (*selected_line)->begin(),
                             (*selected_line)->end(), QGLCanvasPtr(), false);
}

void PCMCanvas::ResetMovingPointLines(QMouseEvent *event)
{
  LineTopology::iterator       selected_node, node;
  LineTopsIterVector::iterator line_topology;
  ObjectPoints                 *line_points;
  ObjectPoints::iterator       point;
  Position3D                   world_pos;
  double                       dist, min_dist;

  // Retrieve the points and topology of the selected line
  line_points   = extension_line->SelectedLinePoints();
  line_topology = extension_line->SelectedLineTopology();

  // Remove the lines from the canvas
  extension_line->Clear(QGLCanvasPtr(), false);
  move_node_line->Clear(QGLCanvasPtr(), false);
  
  // Determine mouse position in the X0Y plane
  world_pos = Canvas2World(event->pos(), 0.0);

  // Determine the nearest point of the selected polygon
  min_dist = 1e10;
  for (node=(*line_topology)->begin(); node!=(*line_topology)->end(); node++) {
    point = line_points->PointIterator(*node);
    dist = point->Distance(world_pos);
    if (dist < min_dist) {
      min_dist = dist;
      selected_node = node;
    }
  }

  // Initialise the moving point lines
  extension_line->Initialise(line_points, line_topology, 
                             (*line_topology)->PreviousNode(selected_node),
                             selected_node, QGLCanvasPtr(), false);
  move_node_line->Initialise(line_points, line_topology, 
                             (*line_topology)->NextNode(selected_node),
                             selected_node, QGLCanvasPtr(), false);
}

void PCMCanvas::RemoveMoveNodeLines(bool refresh)
{
  if (extension_line) extension_line->Clear(QGLCanvasPtr(), false);
  if (move_node_line) move_node_line->Clear(QGLCanvasPtr(), false);
  if (refresh) update();
}

void PCMCanvas::MoveNode(bool refresh)
{
  if (extension_line->IsValid())
    extension_line->MoveSelectedNode(QGLCanvasPtr(), false);
  else
    move_node_line->MoveSelectedNode(QGLCanvasPtr(), false);
  RemoveMoveNodeLines(refresh);
}

// Draw the scale
void PCMCanvas::DrawScale()
{
  Position3D  pt1, pt2, pt3, pt4, origin, new_origin;
  Vector3D    viewdir;
  
  if (scale_location == NoScale) return;

  glColor3f(0.0, 1.0, 0.0);
  glBegin(GL_LINES);
  if (scale_location == CentreScale)
    origin = Position3D(width() / 2.0, height() / 2.0, 0.0);
  else origin = Position3D(10.0, height() - 10.0, 0.0);

  // Horizontal scale line
  pt1 = Canvas2World(QPoint(10, (int) origin.Y()), 100, false);
  pt2 = Canvas2World(QPoint(width() - 10, (int) origin.Y()), 100, false);
  // Vertical scale line
  pt3 = Canvas2World(QPoint((int) origin.X(), height() - 10), 100, false);
  pt4 = Canvas2World(QPoint((int) origin.X(), 10), 100, false);
  
  // Draw the horizontal scale line
  viewdir = (pt4 - pt3).Normalize();
  if (scale_location == CentreScale) {
    new_origin = (pt1 + pt2) / 2.0;
    DrawScaleLine(new_origin, pt1, viewdir);
    DrawScaleLine(new_origin, pt2, viewdir);
  }
  else
    DrawScaleLine(pt1, pt2, viewdir);

  // Draw the vertical scale line
  viewdir = (pt2 - pt1).Normalize();
  if (scale_location == CentreScale) {
    new_origin = (pt3 + pt4) / 2.0;
    DrawScaleLine(new_origin, pt3, viewdir);
    DrawScaleLine(new_origin, pt4, viewdir);
  }
  else
    DrawScaleLine(pt3, pt4, viewdir);

  glEnd();
}
    
// Draw a scale line
void PCMCanvas::DrawScaleLine(Position3D &pt1, Position3D &pt2,
                              Vector3D &viewdir)
{
  Vector3D   dif, crossdir;
  Position3D footpt;
  double     dist, length, mult_fact;

  // The main scale line
  glVertex3f(pt1.X(), pt1.Y(), pt1.Z());
  glVertex3f(pt2.X(), pt2.Y(), pt2.Z());

  // The cross lines
  dif = pt2 - pt1;
  length = dif.Length();
  if (length == 0.0) return; // Bad luck
  mult_fact = (scale_location == CentreScale) ? 2.0 : 1.0;
  crossdir = viewdir * (mult_fact * length / 60.0);
  for (dist=0; dist < length; dist+=scale_tick_distance) {
    footpt = pt1 + dif * (dist/length);
    glVertex3f(footpt.X() + crossdir.X(),
               footpt.Y() + crossdir.Y(),
               footpt.Z() + crossdir.Z());
    glVertex3f(footpt.X() - crossdir.X(),
               footpt.Y() - crossdir.Y(),
               footpt.Z() - crossdir.Z());
  }
}

// Paint all data
void PCMCanvas::PaintAllData()
{
  DrawScale();
  QGLCanvas::PaintAllData();
}
