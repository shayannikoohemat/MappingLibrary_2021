
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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "PointCloudMapper.h"
#include <QMessageBox>
#include <QStatusBar>
#include <QPoint>
#include "BNF_io.h"

extern int BNF_FileExists(const char *);

/*
--------------------------------------------------------------------------------
                           Set file names
--------------------------------------------------------------------------------
*/

void PointCloudMapper::
     SetMapPointFile(const char *point_file)
{
  if (point_file) {
    map_point_file = (char *) malloc(strlen(point_file) + 1);
    strcpy(map_point_file, point_file);
  }
  else map_top_file = NULL;
}

void PointCloudMapper::SetMapTopologyFile(const char *top_file)
{
  if (top_file) {
    map_top_file = (char *) malloc(strlen(top_file) + 1);
    strcpy(map_top_file, top_file);
  }
  else map_top_file = NULL;
}

void PointCloudMapper::SetModelPointFile(const char *point_file)
{
  if (point_file) {
    model_point_file = (char *) malloc(strlen(point_file) + 1);
    strcpy(model_point_file, point_file);
  }
  else model_top_file = NULL;
}

void PointCloudMapper::SetModelTopologyFile(const char *top_file)
{
  if (top_file) {
    model_top_file = (char *) malloc(strlen(top_file) + 1);
    strcpy(model_top_file, top_file);
  }
  else model_top_file = NULL;
}

void PointCloudMapper::SetLaserMetaFile(const char *meta_file)
{
  if (meta_file) {
    laser_meta_file = (char *) malloc(strlen(meta_file) + 1);
    strcpy(laser_meta_file, meta_file);
  }
  else laser_meta_file = NULL;
}

void PointCloudMapper::SetLaserPointFile(const char *point_file)
{
  if (point_file) {
    laser_point_file = (char *) malloc(strlen(point_file) + 1);
    strcpy(laser_point_file, point_file);
  }
  else laser_point_file = NULL;
}

void PointCloudMapper::SetLaserPyramidFile(const char *pyramid_file)
{
  if (pyramid_file) {
    laser_pyramid_file = (char *) malloc(strlen(pyramid_file) + 1);
    strcpy(laser_pyramid_file, pyramid_file);
  }
  else laser_pyramid_file = NULL;
}

/*
--------------------------------------------------------------------------------
                       Read all mapping data
--------------------------------------------------------------------------------
*/

bool PointCloudMapper::ReadProject(const char *filename, bool meta_data_only)
{
  FILE *fd;
  char *buffer, *line, *keyword;
  int  keyword_length, fileclass;
  bool finished;

  // Initialise all meta data
  if (project_file != filename) {
    project_file = (char *) malloc(strlen(filename) + 1);
    strcpy(project_file, filename);
  }
  map_point_file = map_top_file = NULL;
  model_point_file = model_top_file = NULL;
  laser_meta_file = laser_point_file = laser_pyramid_file = NULL;

  // Open the file
  fd = fopen(project_file, "r");
  if (fd == NULL) {
    fprintf(stderr, "Error opening point cloud mapping file %s\n",
            project_file);
    return(false);
  }
  
  // Check if we may be dealing with a laser point file only
  if (strstr(project_file, ".laser") != NULL ||
      strstr(project_file, ".las") != NULL ||
	  strstr(project_file, ".laz") != NULL) {
    laser_point_file = (char *) malloc(strlen(project_file) + 1);
    strcpy(laser_point_file, project_file);
    fclose(fd);
    if (meta_data_only) return true;
    laser_points.Read(laser_point_file, false);
    laser_points.DeriveDataBounds(0);
    return true;
  }

  // Check if we may be dealing with a laser pyramid only
  if (strstr(project_file, ".pyramid") != NULL) {
    laser_pyramid_file = (char *) malloc(strlen(project_file) + 1);
    strcpy(laser_pyramid_file, project_file);
    fclose(fd);
    laser_pyramid.ReInitialise(); // Clear old data
    if (!laser_pyramid.ReadMetaData(laser_pyramid_file)) {
      printf("Error reading laser pyramid data from file %s\n",
             laser_pyramid_file);
      return false;
    }
    if (laser_pyramid.empty()) return false;
    // Switch mode to browse
    SetMode(BrowseMode);      
    return true;
  }

  // Verify that the first keyword is "pointcloudmapping"
  buffer = (char *) malloc(MAXCHARS);
  do { line = fgets(buffer, MAXCHARS, fd); } while (line && Is_Comment(line));
  if (line == NULL) {
    fprintf(stderr, "Error reading point cloud mapping file %s\n",
            project_file);
    fclose(fd);
    return(false);
  }
  keyword = BNF_KeyWord(line, &keyword_length);
  if (strncmp(keyword, "pointcloudmapping", max(keyword_length, 17))) {
    fprintf(stderr, "Error: File %s is not a point cloud mapping file.\n",
            filename);
    fprintf(stderr, "       First keyword is %s, not pointcloudmapping.\n",
	    keyword);
  }

  // Process all lines
  finished = false;
  line = fgets(buffer, MAXCHARS, fd);
  while (line && !finished) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
	
	if (!strncmp(keyword, "project", max(keyword_length, 7)))
	  project_name = BNF_String(line);

        else if (!strncmp(keyword, "map_points", max(keyword_length, 10)))
          map_point_file = BNF_String(line);

        else if (!strncmp(keyword, "map_topology", max(keyword_length, 12)))
          map_top_file = BNF_String(line);

        else if (!strncmp(keyword, "model_points", max(keyword_length, 12)))
          model_point_file = BNF_String(line);

        else if (!strncmp(keyword, "model_topology", max(keyword_length, 14)))
          model_top_file = BNF_String(line);

        else if (!strncmp(keyword, "laser_metadata", max(keyword_length, 14)))
          laser_meta_file = BNF_String(line);

        else if (!strncmp(keyword, "laser_pyramid", max(keyword_length, 13)))
          laser_pyramid_file = BNF_String(line);

        else if (!strncmp(keyword, "laser_points", max(keyword_length, 12))) {
          laser_point_file = BNF_String(line);
        }

        else if (!strncmp(keyword, "endpointcloudmapping",
                          max(keyword_length, 20))) {
          free(buffer);
          fclose(fd);
          finished = true;
        }

	else {
	  keyword[keyword_length] = 0;
	  fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
	}
      }
    }
    if (!finished) line = fgets(buffer, MAXCHARS, fd); // Read next line
  }

  if (!finished) {
    fprintf(stderr, "Error: Did not find keyword endpointcloudmapping.\n");
    return(false);
  }

  if (meta_data_only) return true;

  // Read the data
  if (map_point_file && map_top_file)
    buildings.ImportPCMMapData(map_point_file, map_top_file, map_points, true);
  if (model_point_file && model_top_file)
    buildings.ImportPCMModelData(model_point_file, model_top_file, model_points);
  if (laser_point_file) {
    laser_points.Read(laser_point_file, false);
    laser_points.DeriveDataBounds(0);
  }
  if (laser_meta_file) {
    // Read meta data
    laser_block.Create(laser_meta_file, &fileclass);
    // Try to read all points up to a maximum
    int numpts = laser_block.ReadPoints(appearance[LaserData]->MaximumNumberOfPointsInMemory());
    if (numpts == -1)
      printf("Block contains more than %d laser points.\n",
             appearance[LaserData]->MaximumNumberOfPointsInMemory());
    else printf("Block with %d laser points read.\n", numpts);
  }
  if (laser_pyramid_file) {
    // Read meta data
    if (!laser_pyramid.ReadMetaData(laser_pyramid_file)) {
      printf("Error reading laser pyramid file %s\n", laser_pyramid_file);
    }
  }
  return true;
}

bool PointCloudMapper::ContainsData(DataType type) const
{
  switch (type) {
    case LaserData:
      return laser_points.size() > 0;
    case SelectedLaserData:
      return selected_laser_points.size() > 0;
    case LastModelData:
      return last_model_part.ContainsData(ModelData) ||
             !last_model_points.empty();
    case SelectedMapData:
      return selected_map_data.size() > 0;
    case SelectedMapPartitionData:
      return selected_map_part_data.size() > 0;
    case SelectedModelData:
      return selected_model_data.size() > 0;
    case SelectedModelPartData:
      return selected_model_part_data.size() > 0;
    case SelectedModelFaceData:
      return selected_model_face_data.size() > 0;
    default: // Now treat the normal building data (map, model)
      return buildings.ContainsData(type);
  }
  return false;
}

void PointCloudMapper::UpdateSelection(LineTopsIterVector &selection,
                                       LineTopsIterVector &nearby_data,
                                       LineTopsIterVector::iterator reference,
                                       ObjectPoints *points,
                                       DataType type, bool add, 
                                       bool check_reference, bool refresh,
                                       bool change_reference_line_only)
{
  LineTopsIterVector::iterator polygon;
  int                          part_number, building_number;
  bool                         change_pol=false, debug=false;

  if (nearby_data.empty()) return;
  // Get the building and building part number of the data to be added
  if (debug) {
    printf("Partition data at start of UpdateSelection\n");
    buildings.begin()->Print(MapPartitionData);
  }
  building_number = (*reference)->Attribute(BuildingNumberTag);
  if ((*reference)->HasAttribute(BuildingPartNumberTag))
    part_number = (*reference)->Attribute(BuildingPartNumberTag);
  else
    part_number = 0;
  if (debug) {
    printf("Partition data after retreiving part number\n");
    buildings.begin()->Print(MapPartitionData);
  }
  for (polygon=nearby_data.begin(); polygon!=nearby_data.end(); polygon++) {
    // Check if the nearby data matches the ID
    switch (type) {
      case MapData:
      case SelectedMapData:
        change_pol = ((*polygon)->Attribute(BuildingNumberTag) == building_number); break;
      case MapPartitionData:
      case SelectedMapPartitionData:
        change_pol = ((*polygon)->Attribute(BuildingNumberTag) == building_number);
        if (debug)
          printf("pol building number %d, ref building number %d\n",
                 (*polygon)->Attribute(BuildingNumberTag), building_number);
        if (change_reference_line_only && *polygon != *reference)
          change_pol = false;
        break;
      case ModelData:
      case SelectedModelData:
        change_pol = ((*polygon)->Attribute(BuildingNumberTag) == building_number);
        if (debug)
          printf("pol building number %d, ref building number %d\n",
                 (*polygon)->Attribute(BuildingNumberTag), building_number);
        break;
      case SelectedModelPartData:
        change_pol = ((*polygon)->Attribute(BuildingNumberTag) == building_number &&
                      (*polygon)->Attribute(BuildingPartNumberTag) == part_number); break;
      case LastModelData:
      case SelectedModelFaceData:
        change_pol = true;
        if (change_reference_line_only && *polygon != *reference)
          change_pol = false;
        break;
      default:
        break;
    }
    if (change_pol || !check_reference) {
      // Change the appearance of this data
      Canvas()->ChangeObjectAppearance(points, &**polygon,
                                       appearance[type]->DataAppearancePtr(),
                                       false);
      // Add or remove the polygon to the selection
      if (add) selection.push_back(*polygon);
      else { 
        selection.Remove(*polygon);
        if (&selection == &nearby_data) polygon--;
      }
    }
  }
  if (refresh) Canvas()->update();

  if (debug) {
    printf("Partition data at end of UpdateSelection\n");
    buildings.begin()->Print(MapPartitionData);
  }
}

void PointCloudMapper::SelectObjectData(const PointNumber &number,
                                        const Position3D &map_pos,
                                        DataType selection_type,
                                        DataType base_type)
{
  Buildings::const_iterator    building;
  LineTopsIterVector           polygon_selection;
  LineTopsIterVector::iterator polygon;
  LineTopsIterVector           *selected_data, *nearby_data;
  ObjectPoints                 *point_data;
  LineTopologies::iterator     enclosing_polygon;
  bool                         found, change_reference_line_only = false;
  ObjectPoint2D                map_point;
  ObjectPoints::const_iterator sel_point;
  ObjectPoints::iterator       model_corner;
  ObjectPoints                 model_corners;
  LineTopology::iterator       node;
  QPoint                       canvas_point;
  bool                         debug=false;

  // Store selected point
  if (selected_point.size())
    selected_point.erase(selected_point.begin(), selected_point.end());
  if (CorrespondingPoints(base_type) == &map_points) {
    selected_point.push_back(*(map_points.GetPoint(number)));
    sel_point = map_points.ConstPointIterator(number);
  }
  else if (CorrespondingPoints(base_type) == &model_points) {
    selected_point.push_back(*(model_points.GetPoint(number)));
    sel_point = model_points.ConstPointIterator(number);
  }
  else { // Last model data
    selected_point.push_back(*(last_model_points.GetPoint(number)));
    sel_point = last_model_points.ConstPointIterator(number);
  }

  // Select new data
  if (base_type != LastModelData) {
    if (debug)
      printf("DEBUG: Selection type %d, %d buildings, number %d\n", 
             selection_type, (int) buildings.size(), (int) number.Number());
    for (building=buildings.begin(); building!=buildings.end(); building++)
      building->SelectPolygons(polygon_selection, number, selection_type);
    if (debug)
      printf("DEBUG: %d polygons selected\n", (int) polygon_selection.size());
  }
  else
    last_model_part.SelectPolygons(polygon_selection, number, selection_type);

  // In the case of map (partition) data, see if the map position of the
  // mouse is inside one of these polygons. If so, put this polygon in
  // first position.
  if (base_type == MapData || base_type == MapPartitionData) {
    map_point = ObjectPoint2D(map_pos.X(), map_pos.Y(), 0, 0.0, 0.0, 0.0);
    for (polygon=polygon_selection.begin(), found=false;
         polygon!=polygon_selection.end() && !found; polygon++) {
      if (map_point.InsidePolygon(map_points, **polygon)) {
        found = true;
        enclosing_polygon = *polygon;
        polygon_selection.erase(polygon);
        polygon_selection.insert(polygon_selection.begin(), enclosing_polygon);
      }
    }
    if (base_type == MapPartitionData) change_reference_line_only = true;
  }

  // In the case of model face data, project all involved model points onto
  // the canvas and see if the position of the mouse is inside one of these
  // polygons. If so, put this polygon in the first position.
  if (base_type == LastModelData) {
    // Collect the points of all selected lines
    for (polygon=polygon_selection.begin();
         polygon!=polygon_selection.end(); polygon++) {
      for (node=(*polygon)->begin(); node!=(*polygon)->end(); node++) {
        if (!model_corners.Contains(*node)) {
          model_corner = last_model_points.PointIterator(*node);
          model_corners.push_back(*model_corner);
        }
      }
    }
    
    // Transform coordinates to canvas coordinate system
    for (model_corner=model_corners.begin(); model_corner!=model_corners.end();
         model_corner++) {
      canvas_point = canvas->World2Canvas(model_corner->Position3DRef());
      model_corner->X() = (double) canvas_point.x();
      model_corner->Y() = (double) canvas_point.y();
    }
    
    // Transform mouse position to canvas coordinate system
    canvas_point = canvas->World2Canvas(map_pos);
    
    // Check if mouse is inside on of the polygons
    map_point = ObjectPoint2D(canvas_point.x(), canvas_point.y(), 0,
                              0.0, 0.0, 0.0);
    for (polygon=polygon_selection.begin(), found=false;
         polygon!=polygon_selection.end() && !found; polygon++) {
      if (map_point.InsidePolygon(model_corners, **polygon)) {
        found = true;
        enclosing_polygon = *polygon;
        polygon_selection.erase(polygon);
        polygon_selection.insert(polygon_selection.begin(), enclosing_polygon);
      }
    }
    change_reference_line_only = true;
  }

  // Get pointers of selected data,  nearby data buffer and point data
  selected_data = CorrespondingSelection(selection_type);
  nearby_data   = CorrespondingNearbyData(selection_type);
  point_data    = CorrespondingPoints(selection_type);

  // Clear old buffers
  nearby_data->Clear(); // Clear nearby data buffer
  if (!Canvas()->ShiftKeyDown()) // If no data should be added to the selection
    // Clear the selection buffer
    UpdateSelection(*selected_data, *selected_data, selected_data->begin(),
                    point_data, base_type, false, false, false);

  // Return if the nearest point is not part of any polygon
  if (polygon_selection.empty()) {
    Canvas()->update();
    return;
  }
  if (debug)
    printf("DEBUG: after clearing: selected_data %d, nearby_data %d, point_data %d\n",
           (int) selected_data->size(), (int) nearby_data->size(), (int) point_data->size());

  // Add new data to the nearby data buffer if there
  nearby_data->insert(nearby_data->begin(),
                      polygon_selection.begin(),polygon_selection.end());

  // Update the selected data buffer
  if (selected_data->Contains(*(nearby_data->begin())))
    // The new nearby data is already selected, therefore unselect that data.
    UpdateSelection(*selected_data, *nearby_data, nearby_data->begin(),
                    point_data, base_type, false, true, true,
                    change_reference_line_only);
  else {
    // Add the nearby data to the selection
    UpdateSelection(*selected_data, *nearby_data, nearby_data->begin(),
                    point_data, selection_type, true, true, true,
                    change_reference_line_only);
    if (debug)
      printf("DEBUG: after UpdateSelection: selected_data %d, nearby_data %d, point_data %d\n",
             (int) selected_data->size(), (int) nearby_data->size(), (int) point_data->size());
  }
  polygon_selection.Clear();

  // Print message on selected point in status bar
  DisplayPointInformation(*sel_point, selection_type);
}

void PointCloudMapper::ToggleSelectionData(DataType selection_type)
{
  LineTopsIterVector *selected_data;
  LineTopsIterVector *nearby_data;
  LineTopsIterVector::iterator last_selection, polygon, nearby_polygon;
  ObjectPoints       *point_data;
  bool               found, change_reference_line_only;
  DataType           base_type;

  selected_data = CorrespondingSelection(selection_type);
  nearby_data   = CorrespondingNearbyData(selection_type);
  point_data    = CorrespondingPoints(selection_type);
  if (selection_type == SelectedModelData ||
      selection_type == SelectedModelPartData ||
      selection_type == SelectedModelFaceData) {
    QMessageBox::information(this, "Error",
                 "Toggling of selected model data is not yet implemented.\n");
    return;
  }

  // Check if there's something to toggle
  if (selected_data->size() == 0 || nearby_data->size() <= 1) return;
  // Locate the last item of the selection in the nearby data buffer
  last_selection = selected_data->end() - 1;
  for (polygon=nearby_data->begin(), found=false;
       polygon!=nearby_data->end() && !found; polygon++)
    if (*polygon == *last_selection) {
      nearby_polygon = polygon;
      found = true;
    }
  if (!found) {
    printf("Missing toggle data: bug?\n");
    return;
  }
  // Select the next item of the nearby data buffer
  nearby_polygon++;
  if (nearby_polygon == nearby_data->end())
    nearby_polygon = nearby_data->begin();
  base_type = Canvas()->CorrespondingBaseDataType(selection_type);
  change_reference_line_only = (base_type == MapPartitionData);
  // Remove the last selection
  UpdateSelection(*selected_data, *selected_data, last_selection,
                  point_data, base_type, false, true, false,
                  change_reference_line_only);
  // Add the new selection
  UpdateSelection(*selected_data, *nearby_data, nearby_polygon,
                  point_data, selection_type, true, true, true,
                  change_reference_line_only);
}

ObjectPoints *PointCloudMapper::CorrespondingPoints(DataType type)
{
  switch (type) {
    case MapData:
    case SelectedMapData:
    case MapPartitionData:
    case SelectedMapPartitionData:
      return &map_points;
    case ModelData:
    case SelectedModelData:
    case SelectedModelPartData:
      return &model_points;
    case LastModelData:
    case SelectedModelFaceData:
      return &last_model_points;
    default:
      return NULL;
  }
  return NULL;
}

LineTopsIterVector *PointCloudMapper::CorrespondingSelection(DataType type)
{
  switch (type) {
    case MapData:
    case SelectedMapData:          return &selected_map_data;
    case MapPartitionData:
    case SelectedMapPartitionData: return &selected_map_part_data;
    case ModelData:
    case SelectedModelData:        return &selected_model_data;
    case SelectedModelPartData:    return &selected_model_part_data;
    case LastModelData:     
    case SelectedModelFaceData:    return &selected_model_face_data;    
    case LaserData:
    case SelectedLaserData:        return NULL;
    default:
      printf("Error: Invalid data type in PointCloudMapper::CorrespondingSelection (%d)\n", type);
      break;
  }
  return NULL;
}

LineTopsIterVector *PointCloudMapper::CorrespondingNearbyData(DataType type)
{
  switch (type) {
    case MapData:
    case SelectedMapData:          return &nearby_map_data;
    case MapPartitionData:
    case SelectedMapPartitionData: return &nearby_map_part_data;
    case ModelData:
    case SelectedModelData:        return &nearby_model_data;
    case SelectedModelPartData:    return &nearby_model_part_data;
    case LastModelData:
    case SelectedModelFaceData:    return &nearby_model_face_data;
    default:
      printf("Error: Invalid data type in PointCloudMapper::CorrespondingNearbyData\n");
      break;
  }
  return NULL;
}

void PointCloudMapper::SelectInsideRectangle(MouseMode selection_mode)
{
  Buildings::const_iterator    building;
  ObjectPoints                 *point_data;
  LineTopsIterVector           *selected_data, *nearby_data, polygon_selection;
  LineTopsIterVector::iterator polygon;
  DataType                     selection_type, base_type;
  PointNumberList              inside_points;

  // Get the involved data types
  selection_type = Canvas()->CorrespondingDataType(selection_mode);
  base_type      = Canvas()->CorrespondingBaseDataType(selection_type);
  // Get pointers to the data
  point_data = CorrespondingPoints(selection_type);
  selected_data = CorrespondingSelection(selection_type);
  nearby_data   = CorrespondingNearbyData(selection_type);

  // Get the points inside the selection rectangle
  Canvas()->SelectInsideRectangle(point_data, inside_points);

  // Select new data
  for (building=buildings.begin(); building!=buildings.end(); building++)
    building->SelectPolygons(polygon_selection, inside_points, selection_type);
  if (polygon_selection.empty()) return;

  // Clear old buffers
  nearby_data->Clear(); // Clear nearby data buffer
  if (!Canvas()->ShiftKeyDown()) // If no data should be added to the selection
    // Clear the selection buffer
    UpdateSelection(*selected_data, *selected_data, selected_data->begin(),
                    point_data, base_type, false, false, false);

  // Add the selected data to the selection buffer. Make sure not to duplicate
  for (polygon=polygon_selection.begin(); polygon!=polygon_selection.end();
       polygon++)
    if (!selected_data->Contains(*polygon))
      UpdateSelection(*selected_data, polygon_selection, polygon,
                      point_data, selection_type, true, true, false);

  // Refresh the canvas
  Canvas()->update();

  // Clear the local selections
  inside_points.erase(inside_points.begin(), inside_points.end());
  polygon_selection.Clear();
}

void PointCloudMapper::SelectMapPartitionLines(int building_number)
{
  Buildings::iterator building = buildings.BuildingIterator(building_number);
  LineTopologies *map_part_data = building->MapPartitionDataPtr();
  LineTopologies::iterator polygon;
  LineTopsIterVector::iterator polygon_iterator;

  if(map_part_data == NULL) {
  	QMessageBox::information(this, "Error",
                         "The selected building has no map partition lines.\n");
    return;
  }
  
  // Declare all map partition lines as nearby map partition lines  
  nearby_map_part_data.Clear();
  for (polygon=map_part_data->begin(); polygon!=map_part_data->end();
       polygon++)
    nearby_map_part_data.push_back(polygon);
  
  // Put all nearby lines in the selection buffer
  for (polygon_iterator=nearby_map_part_data.begin();
       polygon_iterator!=nearby_map_part_data.end();
       polygon_iterator++) {
    UpdateSelection(selected_map_part_data, nearby_map_part_data,
                    polygon_iterator,
                    &map_points, SelectedMapPartitionData, true, true, false);
  }
  Canvas()->update();
}

void PointCloudMapper::SelectLaserData()
{
  // Select points inside selection rectangle
  if (Canvas()->HasValidSelectionRectangle())
    SelectLaserDataByBox(true, true, true);
  // Select points inside map (partition) line
  else if (selected_map_data.size() || selected_map_part_data.size())
    SelectLaserDataByMap(true, true, true);
  // Transfer selected laser points to standard laser point set
  else if (selected_laser_points.size()) {
    laser_points.swap(selected_laser_points);
    laser_points.RemoveAttribute(IsSelectedTag);
    laser_points.DeriveDataBounds(0);
    selected_laser_points.ErasePoints();
    ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, false,
             true, true);
  }
  else
    QMessageBox::information(this, "Error",
                          "No selection box or selected map data available.\n");
}

void PointCloudMapper::ChangeLaserPointSelection(LaserPoints *selected_points,
                                                 bool add)
{
  LaserPoints::const_iterator selected_point;
  LaserPoints::iterator       point;
  bool                        done;
  
  for (selected_point=selected_points->begin();
       selected_point!=selected_points->end(); selected_point++) {
  	if (add) {
      if (!selected_laser_points.Contains(*selected_point)) {
      	// Add point to selection
  	    selected_laser_points.push_back(*selected_point);
  	    (selected_laser_points.end()-1)->Select();
  	    // Mark point as selected in the set of all points
  	    for (point=laser_points.begin(), done=false;
		     point!=laser_points.end() && !done; point++) {
  	      if (*point == *selected_point) {
  	        point->Select();
  	        done = true;
  	      }
  	    }
  	  }
  	}
  	else { // Remove from selection
  	  if (selected_laser_points.Contains(*selected_point)) {
  	  	// Remove point from selection
  	    for (point=selected_laser_points.begin(), done=false;
		     point!=selected_laser_points.end() && !done; point++) {
  	      if (*point == *selected_point) {
  	        selected_laser_points.erase(point);
  	        done = true;
  	      }
  	    }
  	  	// Mark point as unselected in the set of all points
  	    for (point=laser_points.begin(), done=false;
		     point!=laser_points.end() && !done; point++) {
  	      if (*point == *selected_point) {
  	        point->RemoveAttribute(IsSelectedTag);
  	        done = true;
  	      }
  	    }
  	  }
  	}
  }
  
  // Clean up
  selected_points->ErasePoints();
  delete selected_points;
  
  // Update display
  ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, true,
           true, true);
}

void PointCloudMapper::SelectLaserData(const LaserPoint &selected_point,
                                       DataType type)
{
  DataType real_type;

  real_type = type;
  if (selected_laser_points.Contains(selected_point))
    real_type = SelectedLaserData;

  // Clear selected laser points if no data should be added to the selection
  if (!Canvas()->ShiftKeyDown()) {
    selected_laser_points.ErasePoints();
    laser_points.RemoveAttribute(IsSelectedTag);
  }

  // Add all points with the label of the selected point
  if (real_type == LaserData || !Canvas()->ShiftKeyDown()) {
    if (laser_selection_tag != SegmentStartTileNumberTag) {
      selected_laser_points.AddTaggedPoints(laser_points,
                                    selected_point.Attribute(laser_selection_tag),
                                    laser_selection_tag, IsSelectedTag);
    }
    else {
      selected_laser_points.AddTaggedPoints(laser_points,
                                    selected_point.Attribute(SegmentNumberTag),
                                    SegmentNumberTag,
                                    selected_point.Attribute(laser_selection_tag),
                                    laser_selection_tag, IsSelectedTag);
    }
  }

  // or remove points from the selection
  else {
    if (laser_selection_tag != SegmentStartTileNumberTag) {
      selected_laser_points.RemoveTaggedPoints(selected_point.Attribute(laser_selection_tag),
                                               laser_selection_tag);
      laser_points.ConditionalReTag(1, 0, IsSelectedTag,
                                    selected_point.Attribute(laser_selection_tag),
                                    laser_selection_tag);
    }
    else {
      selected_laser_points.RemoveTaggedPoints(selected_point.Attribute(SegmentNumberTag),
	                                           SegmentNumberTag,
											   selected_point.Attribute(laser_selection_tag),
                                               laser_selection_tag);
      laser_points.MultiConditionalReTag(0, IsSelectedTag,
                                    selected_point.Attribute(SegmentNumberTag),
                                    SegmentNumberTag,
                                    selected_point.Attribute(laser_selection_tag),
                                    laser_selection_tag);
    }
  } 

  // Update display
  ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, true,
           true, true);
}

void PointCloudMapper::DisplayPointInformation(const LaserPoint &point)
{
  point_information_window->DisplayPointInformation(point, frameGeometry());
  point_information_window->show();
}

void PointCloudMapper::DisplayPointInformation(const ObjectPoint &point,
                                               DataType type)
{
  char                *string, *string2;
  Buildings::iterator building;
  BuildingPart        *part;
  LineTopsIterVector  *selected_data, touched_polygons;
  LineTopsIterVector::iterator selected_polygon, found_polygon;
  bool                         found;
  
  string = (char *) malloc(1024);
  string2 = (char *) malloc(1024);

  string[0] = 0;
  // Find the corresponding building and building part (if available)
  if (type == LastModelData || type == SelectedModelFaceData) {
    sprintf(string, "Edited building, ");
  }
  else {
    switch (type) {
      case SelectedMapData:
        if (selected_map_data.size()) selected_data = &selected_map_data; break;
      case SelectedMapPartitionData:
        if (selected_map_part_data.size()) selected_data = &selected_map_part_data; break;
      case SelectedModelData:
        if (selected_model_data.size()) selected_data = &selected_model_data; break;
      case SelectedModelPartData:
        if (selected_model_part_data.size()) selected_data = &selected_model_part_data; break;
      default: // No selection available, just mouse pointing on a canvas location
        selected_data = NULL; break;
    }
    if (selected_data) {
      // Find the first selected polygon that contains the selected point
      for (selected_polygon = selected_data->begin(), found=false;
           selected_polygon != selected_data->end() && !found;
           selected_polygon++) {
        if ((*selected_polygon)->Contains(point.NumberRef())) {
          found = true;
          found_polygon = selected_polygon;
        }
      } 
    }
    if (!selected_data || !found) {
      building = buildings.BuildingIterator(point.NumberRef(), type);
      if (building != buildings.end()) {
        sprintf(string, "Building %d, ", building->Number());
        part = building->BuildingPartPtr(point.NumberRef(), type);
        if (part != NULL && type == SelectedModelPartData) {
          sprintf(string2, "part %d, ", part->Number());
          strcat(string, string2);
        }
        // Add line number
        building->SelectPolygons(touched_polygons, point.NumberRef(), type);
        if (touched_polygons.size() == 1) { 
          sprintf(string2, "line %d, ", (*(touched_polygons.begin()))->Number());
          strcat(string, string2);
        }
        touched_polygons.Clear();
      }
    }
    else { // Use building and building part number of polygon
      sprintf(string, "Building %d, ", (*found_polygon)->Attribute(BuildingNumberTag));
      if ((*found_polygon)->HasAttribute(BuildingPartNumberTag) &&
          type == SelectedModelPartData) {
        sprintf(string2, "part %d, ", (*found_polygon)->Attribute(BuildingPartNumberTag));
        strcat(string, string2);
      }
      // Add line number if only one line is selected
      if (selected_data->size() == 1) {
        sprintf(string2, "line %d, ", (*found_polygon)->Number());
        strcat(string, string2);
      }
    }
  }

  // Add point information
  if (type == MapData || type == SelectedMapData ||
      type == MapPartitionData || type == SelectedMapPartitionData)  
    sprintf(string2, "map point %d; XYZ (%.2f, %.2f, %.2f)", point.Number(),
            point.X(), point.Y(), point.Z());
  else if (type == ModelData || type == SelectedModelData ||
           type == SelectedModelPartData ||
           type == LastModelData || type == SelectedModelFaceData)
    sprintf(string2, "model point %d; XYZ (%.2f, %.2f, %.2f)", point.Number(),
            point.X(), point.Y(), point.Z());
  else string2[0] = 0;
  strcat(string, string2);
  
  // Display information
  statusBar()->showMessage(QString(string));
  free(string);
  free(string2);
}

void PointCloudMapper::CropToSelectedLaserData()
{
  if (Canvas()->HasValidSelectionRectangle())
    SelectLaserDataByBox(false, false, true);
  else if (selected_map_data.size() || selected_map_part_data.size())
    SelectLaserDataByMap(false, false, true);
  else if (selected_laser_points.size()) { // Crop to selected laser points
    laser_points.ErasePoints();
    laser_points.swap(selected_laser_points);
    laser_points.DeriveDataBounds(0);
    laser_points.RemoveAttribute(IsSelectedTag);
    if (laser_points.GetTIN() != NULL || selected_laser_points.GetTIN() != NULL)
      laser_points.DeriveTIN();
    ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, false,
             true, true);
  }
  else
    QMessageBox::information(this, "Error",
                          "No selection box or selected map data available.\n");
}

void PointCloudMapper::DeleteSelectedLaserData()
{
  if (Canvas()->HasValidSelectionRectangle())
    SelectLaserDataByBox(false, true, true);
  else if (selected_map_data.size() || selected_map_part_data.size())
    SelectLaserDataByMap(false, true, true);
  else if (selected_laser_points.size()) {
    laser_points.RemoveTaggedPoints(1, IsSelectedTag);
    selected_laser_points.ErasePoints();
    laser_points.DeriveDataBounds(0);
    if (laser_points.GetTIN() != NULL) laser_points.DeriveTIN();
    ShowData(PCMWindowPtr(), LaserData, !laser_points.empty(),
             SelectedLaserData, false, true, true);
  }
  else
    QMessageBox::information(this, "Error",
        "No selected points, selection box or selected map data available.");
}

void PointCloudMapper::DeletePoints(int selection)
{
  switch (selection) {
  	case 0: // Delete all points
  	  laser_points.ErasePoints();
  	  selected_laser_points.ErasePoints();
  	  break;
  	  
  	case 1: // Delete selected points
  	  DeleteSelectedLaserData();
  	  return;
  	  
  	case 2: // Delete segmented points
  	  laser_points.RemoveTaggedPoints(segmentation_parameters->SegmentAttribute());
  	  if (selected_laser_points.size())
  	    selected_laser_points.RemoveTaggedPoints(segmentation_parameters->SegmentAttribute());
  	  break;
  	    
  	case 3: // Delete unsegmented points
  	  laser_points.CropTaggedPoints(segmentation_parameters->SegmentAttribute());
  	  if (selected_laser_points.size())
  	    selected_laser_points.CropTaggedPoints(segmentation_parameters->SegmentAttribute());
  	  break;
  }

  // Update canvas
  ShowData(PCMWindowPtr(), LaserData, !laser_points.empty(),
           SelectedLaserData, !selected_laser_points.empty(),
           true, true);
}

void PointCloudMapper::SelectLaserDataByBox(bool add, bool inside,
                                            bool show_data)
{
  LaserBlock::iterator               unit;
  LaserUnit::iterator                subunit;
  LaserPoints                        selection, tile_points, *selection_source,
                                     *selection_destination;
  LaserPoints::iterator              point, goodpoint;
  bool                               keep_old_selection, delete_points;

  // Delete old selections, unless the shift key is down
  keep_old_selection = Canvas()->ShiftKeyDown();
  
  // Set the source and destination pointers and clean old selections if required
  if (add & inside) { // Select inside
    if (keep_old_selection) {
      if (!selected_laser_points.empty()) {
        selection_source = &laser_points;
        selection_destination = &selected_laser_points;
      }
      else {
        selection_source = NULL;
        selection_destination = &laser_points;
      }
    }
    else {        
      if (!laser_points.empty()) {
        if (!selected_laser_points.empty()) selected_laser_points.ErasePoints();
        selection_source = &laser_points;
        selection_destination = &selected_laser_points;
      }
      else {
        selection_source = NULL;
        selection_destination = &laser_points;
      }
    }
  }
  else if (!add) { // Delete either inside or outside (=crop)
    if (!selected_laser_points.empty()) selection_source = &selected_laser_points;
    else selection_source = &laser_points;
  }
  else { // if (add & !inside) Select outside, not available
    QMessageBox::information(this, "Error",
                    "Selecting points outside map polygon is not available.\n");
    return;
  }

  // Select all points inside the box
  if (selection_source == NULL) {
    // Select from all laser block sub units
    for (unit=laser_block.begin(); unit!=laser_block.end(); unit++) {
      for (subunit=unit->begin(); subunit!=unit->end(); subunit++) {
        if (subunit->size() == 0) {
          subunit->Read();
          delete_points = true;
        }
        else delete_points = false;
        // Make sure not to select in tile borders
        if (subunit->DataOrganisation() == StripTileWise ||
            subunit->DataOrganisation() == TileWise) {
          // Select points within tile bounds
          subunit->LaserPoints::Select(tile_points, subunit->TileBounds());
          // Erase the old points
          subunit->ErasePoints();
          // Replace them by the points within the tile bounds
          subunit->swap(tile_points);
        }
        Canvas()->SelectLaserDataInRectangle(subunit->LaserPointsReference(),
                                             selection, true, true);
        if (delete_points) subunit->ErasePoints();
      }
    }
  }
  else { // Select from the current data set
    Canvas()->SelectLaserDataInRectangle(selection_source->LaserPointsReference(),
                                         selection, true, true);
  }

  // Add selected data
  if (add && inside) {
    if (selection_destination->empty()) {
      selection_destination->swap(selection);
      selection.ErasePoints();
      if (selection_destination == &selected_laser_points)
        selected_laser_points.SetAttribute(IsSelectedTag, 1);
      else
        laser_points.RemoveAttribute(IsSelectedTag);
    }
    else {
      if (selection_destination == &selected_laser_points)
        selection.SetAttribute(IsSelectedTag, 1);
      for (point=selection.begin(); point!=selection.end(); point++)
        if (!selection_destination->Contains(*point))
          selection_destination->push_back(*point);
      if (selection_destination == &laser_points)
        selection_destination->RemoveAttribute(IsSelectedTag);
    }
  }

  // Crop to selected data (i.e. remove points outside polygon)
  else if (!add && !inside) {
    if (selection_source == &laser_points) {
      laser_points.swap(selection);
      laser_points.RemoveAttribute(IsSelectedTag);
      selection.ErasePoints();
    }
    else {
      selected_laser_points.swap(selection);
      selected_laser_points.SetAttribute(IsSelectedTag, 1);
      selection.ErasePoints();
      for (point=laser_points.begin(); point!=laser_points.end(); point++) {
        if (!selected_laser_points.Contains(*point))
          point->RemoveAttribute(IsSelectedTag);
      }
    }
  }
  
  // Delete selected data
  else if (!add && inside) {
    if (selection_source == &laser_points) {
      for (point=laser_points.begin(), goodpoint=laser_points.begin();
           point!=laser_points.end(); point++) {
        if (!point->IsSelected()) {
          *goodpoint = *point;
          goodpoint++;
        }
      }
      laser_points.erase(goodpoint, laser_points.end());
    }
    else {
      for (point=selected_laser_points.begin(),
           goodpoint=selected_laser_points.begin();
           point!=selected_laser_points.end(); point++) {
        if (!selection.Contains(*point)) {
          *goodpoint = *point;
          goodpoint++;
        }
      }
      selected_laser_points.erase(goodpoint, selected_laser_points.end());
      // update selection tags in laser_points
      for (point=laser_points.begin(); point!=laser_points.end(); point++)
        if (selection.Contains(*point)) point->UnSelect(); // check  
    }
  }
   
  // Remove the selection rectangle
  Canvas()->RemoveSelectionRectangle(false);

  // Recompute bounds and TINs
  laser_points.DeriveDataBounds(0);
  if (laser_points.GetTIN() != NULL) laser_points.DeriveTIN();
  selected_laser_points.DeriveDataBounds(0);
  if (selected_laser_points.GetTIN() != NULL) selected_laser_points.DeriveTIN();
  
  // Refresh display of laser data
  ShowData(PCMWindowPtr(), LaserData, show_data && !laser_points.empty(),
           SelectedLaserData, show_data && !selected_laser_points.empty(),
           true, true);
}

void PointCloudMapper::SelectLaserDataByMap(bool add, bool inside,
                                            bool show_data)
{
  LineTopologies                     *map_tops;
  LineTopologies::iterator           top;
  LineTopsIterVector::const_iterator polygon;
  LineTopsIterVector                 *selected_data;
  LaserPoints                        selection, *selection_source,
                                     *selection_destination;
  LaserPoints::iterator              point, goodpoint;
  LaserBlock::iterator               unit;
  LaserUnit::iterator                subunit;
  bool                               keep_old_selection, delete_points;

  // Delete old selections, unless the shift key is down
  keep_old_selection = Canvas()->ShiftKeyDown();
  
  // Set the source and destination pointers and clean old selections if required
  if (add & inside) { // Select inside
    if (keep_old_selection) {
      if (!selected_laser_points.empty()) {
        selection_source = &laser_points;
        selection_destination = &selected_laser_points;
      }
      else {
        selection_source = NULL;
        selection_destination = &laser_points;
      }
    }
    else {        
      if (!laser_points.empty()) {
        if (!selected_laser_points.empty()) selected_laser_points.ErasePoints();
        selection_source = &laser_points;
        selection_destination = &selected_laser_points;
      }
      else {
        selection_source = NULL;
        selection_destination = &laser_points;
      }
    }
  }
  else if (!add) { // Delete either inside or outside (=crop)
    if (!selected_laser_points.empty()) selection_source = &selected_laser_points;
    else selection_source = &laser_points;
  }
  else { // if (add & !inside) Select outside, not available
    QMessageBox::information(this, "Error",
                    "Selecting points outside map polygon is not available.\n");
    return;
  }

  // Get the points and topologies for the selection
  if (selected_map_data.size()) selected_data = &selected_map_data;
  else selected_data  = &selected_map_part_data;
  map_tops = new LineTopologies();
  for (polygon=selected_data->begin(); polygon!=selected_data->end(); polygon++)
    map_tops->push_back(**polygon);

  // Select all points inside the polygons
  if (selection_source == NULL) {
    // Select from all laser block sub units
    for (unit=laser_block.begin(); unit!=laser_block.end(); unit++) {
      for (subunit=unit->begin(); subunit!=unit->end(); subunit++) {
        delete_points = subunit->empty();
        if (delete_points) subunit->Read();
        subunit->Select(selection, map_points,
                        map_tops->LineTopologiesReference());
        if (delete_points) subunit->ErasePoints();
      }
    }
  }
  else { // Select from the current data set
    printf("Select with add %d and inside %d\n", add, inside);
    // This should be more flexible with selecting/unselecting marks in this
    selection_source->Select(selection, map_points,
                             map_tops->LineTopologiesReference(), true);
  }

  // Add selected data
  if (add && inside) {
    if (selection_destination->empty()) {
      selection_destination->swap(selection);
      selection.ErasePoints();
      if (selection_destination == &selected_laser_points)
        selected_laser_points.SetAttribute(IsSelectedTag, 1);
      else
        laser_points.RemoveAttribute(IsSelectedTag);
    }
    else {
      if (selection_destination == &selected_laser_points)
        selection.SetAttribute(IsSelectedTag, 1);
      for (point=selection.begin(); point!=selection.end(); point++)
        if (!selection_destination->Contains(*point))
          selection_destination->push_back(*point);
      if (selection_destination == &laser_points)
        selection_destination->RemoveAttribute(IsSelectedTag);
    }
  }

  // Crop to selected data (i.e. remove points outside polygon)
  else if (!add && !inside) {
    if (selection_source == &laser_points) {
      laser_points.swap(selection);
      laser_points.RemoveAttribute(IsSelectedTag);
      selection.ErasePoints();
    }
    else {
      selected_laser_points.swap(selection);
      selected_laser_points.SetAttribute(IsSelectedTag, 1);
      selection.ErasePoints();
      for (point=laser_points.begin(); point!=laser_points.end(); point++) {
        if (!selected_laser_points.Contains(*point))
          point->RemoveAttribute(IsSelectedTag);
      }
    }
  }
  
  // Delete selected data
  else if (!add && inside) {
    if (selection_source == &laser_points) {
      for (point=laser_points.begin(), goodpoint=laser_points.begin();
           point!=laser_points.end(); point++) {
        if (!point->IsSelected()) {
          *goodpoint = *point;
          goodpoint++;
        }
      }
      laser_points.erase(goodpoint, laser_points.end());
    }
    else {
      for (point=selected_laser_points.begin(),
           goodpoint=selected_laser_points.begin();
           point!=selected_laser_points.end(); point++) {
        if (!selection.Contains(*point)) {
          *goodpoint = *point;
          goodpoint++;
        }
      }
      selected_laser_points.erase(goodpoint, selected_laser_points.end());
      // update selection tags in laser_points
      for (point=laser_points.begin(); point!=laser_points.end(); point++)
        if (selection.Contains(*point)) point->UnSelect(); // check  
    }
  }
   

  // Delete the just created polygon list
  for (top=map_tops->begin(); top!=map_tops->end(); top++)
    top->erase(top->begin(), top->end());
  delete map_tops;
  // Clear the selection buffer if we're deleting laser data
  if (!inside)
    UpdateSelection(*selected_data, *selected_data, selected_data->begin(),
                    &map_points, MapData, false, false, false);

  // Recompute bounds and TINs
  laser_points.DeriveDataBounds(0);
  if (laser_points.GetTIN() != NULL) laser_points.DeriveTIN();
  selected_laser_points.DeriveDataBounds(0);
  if (selected_laser_points.GetTIN() != NULL) selected_laser_points.DeriveTIN();
  
  // Refresh display of laser data
  ShowData(PCMWindowPtr(), LaserData, show_data && !laser_points.empty(),
           SelectedLaserData, show_data && !selected_laser_points.empty(),
           true, true);
}

void PointCloudMapper::ReadView()      { Canvas()->ReadView((char *) ".pcmview"); }
void PointCloudMapper::SaveView()      { Canvas()->SaveView((char *) ".pcmview"); }

void PointCloudMapper::FitViewToSelectedData()
{
  DataType selected_type;

  // Determine selected data type
  if (selected_map_data.size())
    selected_type = SelectedMapData;
  else if (selected_map_part_data.size())
    selected_type = SelectedMapPartitionData;
  else if (selected_model_data.size())
    selected_type = SelectedModelData;
  else if (selected_model_part_data.size())
    selected_type = SelectedModelPartData;
  else if (selected_model_face_data.size())
    selected_type = SelectedModelFaceData;
  else return;

  // Fit view to selected data type
   Canvas()->FitViewToData(selected_type);
}

void PointCloudMapper::FitViewToSelectedLaserData()
{
  if (selected_laser_points.empty()) return;
  Canvas()->FitViewToData(SelectedLaserData);
}

void PointCloudMapper::DeleteLoosePoints()
{
  ObjectPoints::iterator point;
  int                    index, percentage;

  // Clean map points
  for (point=map_points.begin(), index=0; point!=map_points.end();
       point++, index++) {
    percentage = index * 100 / map_points.size();
    if ((percentage/10)*10 == percentage)
      statusBar()->
        showMessage(QString("Cleaning map points (%1\%)").arg(percentage));
    if (!buildings.Contains(&map_points, point->NumberRef(), MapData))
      if (!buildings.Contains(&map_points, point->NumberRef(),
                              MapPartitionData)) {
        map_points.erase(point);
        point--;
      }
  }

  // Clean model points
  for (point=model_points.begin(); point!=model_points.end(); point++) {
    percentage = index * 100 / model_points.size();
    if ((percentage/10)*10 == percentage)
      statusBar()->
        showMessage(QString("Cleaning model points (%1\%)").arg(percentage));
    if (!buildings.Contains(&model_points, point->NumberRef(), ModelData)) {
      model_points.erase(point);
      point--;
    }
  }
  statusBar()->showMessage("Cleaning finished", 2000);
}

void PointCloudMapper::PartitionBuilding()
{
  LineTopsIterVector::iterator map_line;
  Buildings::iterator          building;
  int                          building_number, error_code;
  LineTopsIterVector           *selected_data;
  DataType                     base_type, selection_type;

  // Check if there is a selected map or map partition line
  if (selected_map_data.size()) {
    selected_data  = &selected_map_data;
    base_type      = MapData;
    selection_type = SelectedMapData;
  }
  else if (selected_map_part_data.size()) {
    selected_data  = &selected_map_part_data;
    base_type      = MapPartitionData;
    selection_type = SelectedMapPartitionData;
  }
  else {
    QMessageBox::information(this, "Error", "No map line selected.\n");
    return;
  }

  // Process all map (partition) lines
  for (map_line=selected_data->begin(), error_code = 0;
       map_line!=selected_data->end() && error_code == 0; map_line++) {
    // Retrieve the building
    building_number = (*map_line)->Attribute(BuildingNumberTag);
    building = buildings.BuildingIterator(building_number);
    // Partition the building
    error_code = building->Partition(selection_type, map_line);
    // Process error codes
    switch (error_code) {
      case 1:
        QMessageBox::information(this, "Error",
                                 "Data inconsistency! Bug?\n");
        return;
      case 2:
        QMessageBox::information(this, "Error",
                                 "Only closed polygons can be partitioned!\n");
        break;
      case 3:
        QMessageBox::information(this, "Error",
                  "Polygons with less than 5 corners cannot be partitioned!\n");
        error_code = 0; // Continue with other selected lines
        break;
      case 4:
        QMessageBox::information(this, "Error",
                                 "Cannot retrieve corner point! Bug?\n");
        return;
      case 5:
        QMessageBox::information(this, "Error", "Partitioning failed!\n");
        error_code = 0; // Continue with other selected lines
        break;
      default:
        break;
    }
  }

  // Unselect all map (partition) lines
  UpdateSelection(selected_data->LineTopsIterVectorRef(),
                  selected_data->LineTopsIterVectorRef(),
                  selected_data->begin(), &map_points,
                  base_type, false, false, false);

  // Display new map partitions
  ShowData(PCMWindowPtr(), MapPartitionData, true, SelectedMapPartitionData,
           true, true, true);
}

bool PointCloudMapper::FitRoof(RoofType type)
{
  LineTopsIterVector           *selected_data, selected_data_copy;
  LineTopsIterVector::iterator map_line;
  bool                         success, use_laser_block;
  PCMWindow                    *window;
  LaserPoints                  *active_laser_points;
  DataType                     base_type, selection_type;
  Building                     *building;
  int                          error_code;
  
  // Check if there is a selected map or map partition line
  if (selected_map_data.size()) {
    selected_data  = &selected_map_data;
    base_type      = MapData;
    selection_type = SelectedMapData;
  }
  else if (selected_map_part_data.size()) {
    selected_data  = &selected_map_part_data;
    base_type      = MapPartitionData;
    selection_type = SelectedMapPartitionData;
  }
  else {
    QMessageBox::information(this, "Error",
                             "No map (partition) line selected.\n");
    return false;
  }

// If more than one map line is available, loop over all map lines
  if (selected_data->size() > 1 && base_type == MapData) {
    // Copy selected lines
    selected_data_copy = *selected_data;
    selected_laser_points.ErasePoints();
    use_laser_block = laser_points.empty();
    for (map_line=selected_data_copy.begin();
         map_line!=selected_data_copy.end(); map_line++) {
      // Put the current line in the selection
      selected_data->Clear();
      selected_data->push_back(*map_line);
      SelectLaserDataByMap(true, true, false);
      // Call this routine again
      FitRoof(type);
    }
    // Remove selected lines
    selected_data->Clear();
    // Unselect laser data
    if (use_laser_block) laser_points.ErasePoints();
    else selected_laser_points.ErasePoints();
    // Refresh display to unselect the map lines
    ShowData(PCMWindowPtr(), base_type, true, selection_type, false,
             true, true);
    return true;
  }

// Select laser data, if no laser data has been selected before or if the
// laser data is not shown on the canvas

  if (!selected_laser_points.empty())
    active_laser_points = &selected_laser_points;
  else {
    active_laser_points = &laser_points;
    if (laser_points.empty() || !show_data_actions[LaserData]->isChecked())
      SelectLaserDataByMap(true, true, false);
    if (laser_points.empty()) {
      QMessageBox::information(this, "Error",
                             "No laser data available for selected objects.\n");
      return false;
    }
  }

// Initialise the last model part with the selected (partition) outline(s)
// Note that the building number is stored as building part number
  map_line = selected_data->begin();
  last_model_part = BuildingPart((*map_line)->Attribute(BuildingNumberTag),
                                 &map_points);

  // Copy all selected map (partition) lines. In case of map partitions,
  // this can be more than one line. In case of map lines, this is not possible,
  // because multiple map lines are already dealt with in the loop above.
  // Use map partition data if a map line is selected and the building contains
  // map partition data to be used (according to the fitting parameters).
  // NOTE: For fitting a roof primitive only the first partition is used!!!
  building = buildings.BuildingPtr((*map_line)->Attribute(BuildingNumberTag));
  if (base_type == MapData && !fitting_parameters->UseInitialPartitioning()) {
    if (!building->ContainsData(MapPartitionData)) {
      error_code = building->Partition(SelectedMapData, map_line);
      if (error_code != 0) printf("Error %d in partitioning\n", error_code);
    }
    last_model_part.AddMapPartitionData(building->MapPartitionDataPtr()->LineTopologiesReference(),
                                        false);
  }
  else {
    for (map_line=selected_data->begin(); map_line!=selected_data->end();
         map_line++)
      last_model_part.AddMapPartitionData((*map_line)->LineTopologyReference(),
                                          false);
  }

  if (type == UnknownRoof) { // Automatic reconstruction of roof shape
    success = last_model_part.ReconstructComplexRoof(
                active_laser_points->LaserPointsReference(), last_model_points,
                fitting_parameters->FittingParametersRef(), local_ground_height);
  }
  else { // Fit the selected primitive shape
    success = last_model_part.ReconstructRoofPrimitive(type,
                active_laser_points->LaserPointsReference(), last_model_points,
                local_ground_height);
  }

// Show the reconstructed model in a new window

  if (success) {
    last_model_part.DeriveResiduals(active_laser_points->LaserPointsReference(),
                                    true);
    // Create a new window
    window = new PCMWindow(PCMResult, NULL, Qt::Window);
    subwindows.push_back(window);
    map_line = selected_data->begin();
    window->setWindowTitle(QString("Reconstructed (part of) building %1").
                           arg((*map_line)->Attribute(BuildingNumberTag)));
    window->AddReconstructedModel(last_model_points, last_model_part,
                                appearance[LastModelData]->DataAppearancePtr());
    window->AddLaserData(active_laser_points->LaserPointsReference(),
                         result_appearance->DataAppearancePtr(), true);
    window->show();
    // Add connections for the various updates
    AddShowDataConnectors(window);
    connect(window, SIGNAL(WindowWillBeClosed(PCMWindow *)),
            this, SLOT(RemovePCMWindow(PCMWindow *)));
    connect(window, SIGNAL(RequestSavingReconstructedModel(PCMWindow *)),
            this, SLOT(SaveReconstructedModel(PCMWindow *)));
    connect(window, SIGNAL(RequestMoveToMainWindow(PCMWindow *)),
            this, SLOT(TransferDataFromSubWindow(PCMWindow *)));
    connect(window, SIGNAL(RequestAddToMainWindow(PCMWindow *)),
            this, SLOT(AddDataFromSubWindow(PCMWindow *)));
    connect(window, SIGNAL(RequestEditingBuildingModel(PCMWindow *)),
            this, SLOT(EditBuildingModel(PCMWindow *)));
  }

  // Display the result and store the building number in the building part
  if (success) {
    statusBar()->showMessage("Roof fitting succeeded.", 2000);
    // Store building number as the building part number of the last
    // reconstructed model part
    map_line = selected_data->begin();
    last_model_part.Number() = (*map_line)->Attribute(BuildingNumberTag);
  }
  else {
    statusBar()->showMessage("Roof fitting failed.", 4000);
    last_model_part.Number() = -1; // Invalid model
  }

  return success;
}

void PointCloudMapper::SaveReconstructedModel(PCMWindow *window)
{
  Building                  *building;
  Building::iterator        building_part;
  LineTopologies            new_model_data;
  ObjectPoints              new_model_points;
  ObjectPoints::iterator    last_model_point;
  int                       next_point_number, num_parts;
  PointNumberList           deleted_points;
  PointNumberList::iterator deleted_point;
  BuildingPart              *reconstructed_model;
  vector <PCMWindow *>::iterator subwindow;

  // Check if there is a valid reconstruction
  reconstructed_model = window->LastModelPart();
  if (reconstructed_model->Number() == -1) {
    QMessageBox::information(window, "Error",
                             "No valid reconstruction available!");
    return;
  }

  // Get the building part with this outline.
  // Note that the reconstructed model part contains the building number, not
  // the part number.
  building = buildings.BuildingPtr(reconstructed_model->Number());
  if (building == NULL) {
    QMessageBox::information(this, "Error", "Cannot retrieve building. Bug?");
    return;
  }
  // Retrieve the building part. If there is no part with this map outline,
  // a new part will be created by the RetrievePart function.
  num_parts = building->size();
  building_part = building->RetrievePart(reconstructed_model->
                                         MapPartitionTopology()->
                                         LineTopologiesReference());

  // Delete the old model data and remove unused points
  building_part->DeleteData(ModelData, deleted_points);
  for (deleted_point=deleted_points.begin();
       deleted_point!=deleted_points.end(); deleted_point++)
    if (!buildings.Contains(&model_points, *deleted_point, ModelData))
      model_points.DeletePoint(&*deleted_point);

  // Renumber the new model data
  new_model_data.insert(new_model_data.begin(),
                        reconstructed_model->RoofFaces()->begin(),
                        reconstructed_model->RoofFaces()->end());
  new_model_data.insert(new_model_data.begin(),
                        reconstructed_model->WallFaces()->begin(),
                        reconstructed_model->WallFaces()->end());
  new_model_points.insert(new_model_points.begin(),
                          window->LastModelPoints()->begin(),
                          window->LastModelPoints()->end());
  if (model_points.size()) {
    last_model_point = model_points.end() - 1;
    next_point_number = last_model_point->Number() + 1;
  }
  else next_point_number = 0;
  new_model_data.ReNumber(new_model_points, next_point_number);
  
  // Add the new model data
  model_points.insert(model_points.end(),
                      new_model_points.begin(), new_model_points.end());
  if (building->Points(ModelData) == NULL)
    building->AddModelData(&model_points);
  building_part->AddModelData(new_model_data);
  building_part->AddModelData(&model_points);
  building_part->SetLineNumbers(building->Number());

  // Erase local data
  new_model_data.Erase();
  new_model_points.erase(new_model_points.begin(), new_model_points.end());

  // Message in status bar
  if (num_parts == (int) building->size())
    statusBar()->showMessage(QString("Model added to part %1 of building %2")
                         .arg(building_part->Number()).arg(building->Number()),
                         4000);
  else
    statusBar()->
      showMessage(QString("Model added to new part %1 of building %2")
                  .arg(building_part->Number()).arg(building->Number()),
                  4000);

  // Update the main display. Also do this if the main display does not
  // show the model data, because it may be shown in other windows that are
  // updated by ShowData.
  ShowData(PCMWindowPtr(), ModelData,
           show_data_actions[ModelData]->isChecked(), SelectedModelData,
           show_selection_actions[SelectedModelData]->isChecked(), true, true);

  // Close the result window and delete all local data
  delete window;
}

void PointCloudMapper::FitRectangularMapLine()
{
  LineTopologies::iterator polygon;
  LineTopologies           polygons;
  LineTopologies::iterator map_line;
  LineTopology             minimal_enclosing_rectangle;
  int                      building_number;
  Building                 *building;
  LaserPoints              *active_laser_points;

  // Check if there are laser points
  if (!selected_laser_points.empty())
    active_laser_points = &selected_laser_points;
  else if (!laser_points.empty())
    active_laser_points = &laser_points;
  else {
    statusBar()->
      showMessage("No laser points available for fitting rectangle.", 4000);
    return;
  }

  // Fit a polygon, add its points to map_points
  active_laser_points->VerifyTIN();
  active_laser_points->DeriveDataBounds(0);
  if (!active_laser_points->
  // To be changed for multiple polygons
    EnclosingRectangle(segmentation_parameters->MaxDistanceInComponent(),
                     map_points, minimal_enclosing_rectangle))
    return;
  polygons.push_back(minimal_enclosing_rectangle);
  
  // If a map outline is selected, add the polygon as map partition to it
  if (selected_map_data.size()) {
    map_line = *selected_map_data.begin();
    building_number = map_line->Attribute(BuildingNumberTag);
    building = buildings.BuildingPtr(building_number);
    if (building == NULL) {
      QMessageBox::information(this, "Error",
                               "Error locating building. Bug?");
      return;
    }
    for (polygon=polygons.begin(); polygon!=polygons.end(); polygon++)
      building->AddMapPartitionData(*polygon);
    building->SetLineNumbers();
    ShowData(PCMWindowPtr(), MapPartitionData, true, SelectedMapPartitionData,
             true, true,true);
  }

  // Otherwise, create a new building
  else {
    if (buildings.empty()) building_number = 1;
    else building_number = (buildings.end()-1)->Number() + 1;
    for (polygon=polygons.begin(); polygon!=polygons.end(); polygon++) {
      buildings.push_back(Building(building_number, &map_points, *polygon));
      building_number++;
    }
    ShowData(PCMWindowPtr(), MapData, true, SelectedMapData, true, true, true);
  }
}

void PointCloudMapper::FitPolygonalMapLine()
{
  LineTopology             polygon;
  LineTopologies::iterator map_line;
  int                      building_number;
  Building                 *building;
  LaserPoints              *active_laser_points;

  // Check if there are laser points
  if (!selected_laser_points.empty())
    active_laser_points = &selected_laser_points;
  else if (!laser_points.empty())
    active_laser_points = &laser_points;
  else {
    statusBar()->
      showMessage("No laser points available for fitting polygon.", 4000);
    return;
  }

  // Fit a polygon, add its points to map_points
  active_laser_points->VerifyTIN();
  active_laser_points->DeriveDataBounds(0);
  if (!active_laser_points->
    EnclosingPolygon(map_points, polygon, *segmentation_parameters,
                     outlining_parameters->OutliningParametersRef(), true))
    return;
  
  // If a map outline is selected, add the polygon as map partition to it
  if (selected_map_data.size()) {
    map_line = *selected_map_data.begin();
    building_number = map_line->Attribute(BuildingNumberTag);
    building = buildings.BuildingPtr(building_number);
    if (building == NULL) {
      QMessageBox::information(this, "Error",
                               "Error locating building. Bug?");
      return;
    }
    building->AddMapPartitionData(polygon);
    building->SetLineNumbers();
    ShowData(PCMWindowPtr(), MapPartitionData, true, SelectedMapPartitionData,
             true, true,true);
  }

  // Otherwise, create a new building
  else {
    if (buildings.empty()) building_number = 1;
    else building_number = (buildings.end()-1)->Number() + 1;
    buildings.push_back(Building(building_number, &map_points, polygon));
    ShowData(PCMWindowPtr(), MapData, true, SelectedMapData, true, true, true);
  }
}

void PointCloudMapper::FitCircularMapLine()
{
  Circle2D                 circle;
  LineTopology             polygon;
  LineTopologies::iterator map_line;
  int                      building_number;
  Building                 *building;
  LaserPoints              *active_laser_points;

  // Check if there are laser points
  if (!selected_laser_points.empty())
    active_laser_points = &selected_laser_points;
  else if (!laser_points.empty())
    active_laser_points = &laser_points;
  else {
    statusBar()->showMessage("No laser points available for fitting circle.",
                             4000);
    return;
  }
  if (active_laser_points->size() < 3) {
    QMessageBox::information(this, "Error",
                             "Insufficient laser points for fitting circle.");
    return;
  }

  // Fit the circle
  circle = active_laser_points->EnclosingCircle();

  // Sample the circle
  circle.Polygon(24, map_points, polygon);

  // If a map outline is selected, add the circle as map partition to it
  if (selected_map_data.size()) {
    map_line = *selected_map_data.begin();
    building_number = map_line->Attribute(BuildingNumberTag);
    building = buildings.BuildingPtr(building_number);
    if (building == NULL) {
      QMessageBox::information(this, "Error",
                               "Error locating building. Bug?");
      return;
    }
    building->AddMapPartitionData(polygon);
    building->SetLineNumbers();
    ShowData(PCMWindowPtr(), MapPartitionData, true, SelectedMapPartitionData,
             true, true,true);
  }

  // Otherwise, create a new building
  else {
    if (buildings.empty()) building_number = 1;
    else building_number = (buildings.end()-1)->Number() + 1;
    buildings.push_back(Building(building_number, &map_points, polygon));
    ShowData(PCMWindowPtr(), MapData, true, SelectedMapData, true, true, true);
  }
}

void PointCloudMapper::SpawnWindow(PCMWindowType window_type, int selection)
{
  PCMWindow      *window;
  int            data_type, data_selection;
  QGLCanvas      *main_canvas, *browseview_canvas;
  LaserPoints    selected_points;
  DataAppearance *new_appearance;

  // Create a new window
  window = new PCMWindow(window_type, NULL, Qt::Window);
  subwindows.push_back(window);
  if (window_type == PCMPyramidView) window->setWindowTitle("Pyramid view window");
  else if (window_type == PCMCopiedDataView) window->setWindowTitle("View window (copied data)");
  else if (window_type == PCMMainDataView) window->setWindowTitle("View window (main window data)");
  else printf("Unused window type %d in SpawnWindow(PCMWindowType, int)\n", window_type);
  
  // Create new appearance for PCMMainDataView
  if (window_type == PCMMainDataView)
    new_appearance = new DataAppearance(appearance[LaserData]->DataAppearanceRef());
  else
    new_appearance = appearance[LaserData]->DataAppearancePtr();
  
  // Copy laser points
  data_selection = selection;
  if (window_type == PCMPyramidView || window_type == PCMMainDataView)
    data_selection = 0;
  switch (data_selection) {
  	case 0: // Copy all points
  	  window->AddLaserData(laser_points, new_appearance, false);
      break;
      
    case 1: // Copy all selected points
  	  window->AddLaserData(selected_laser_points, new_appearance, false);
      break;
      
    case 2: // Copy points with a segment number
    case 3: // Copy points without a segment number
      if (data_selection == 2)
        laser_points.SelectByTag(segmentation_parameters->SegmentAttribute(),
	                             selected_points);
      else
        laser_points.SelectByNotTag(segmentation_parameters->SegmentAttribute(),
	                                selected_points);
   	  window->AddLaserData(selected_points, new_appearance, false);
      selected_points.ErasePoints();
      break;    
  }

  for (data_type=0; data_type<NumNormalDataTypes; data_type++)
    if (DataIsShown((DataType) data_type) && data_type != LastModelData)
      DisplayData((DataType) data_type, window);
  if (window_type == PCMPyramidView || window_type == PCMMainDataView)
    laser_points.DeriveDataBounds(0);
  window->Canvas()->InitialiseTransformation();
  window->show();

  // Add connections for the various updates
  AddShowDataConnectors(window);
  connect(window, SIGNAL(WindowWillBeClosed(PCMWindow *)),
          this, SLOT(RemovePCMWindow(PCMWindow *)));
  connect(window, SIGNAL(RequestMoveToMainWindow(PCMWindow *)),
          this, SLOT(TransferDataFromSubWindow(PCMWindow *)));
  connect(window, SIGNAL(RequestAddToMainWindow(PCMWindow *)),
          this, SLOT(AddDataFromSubWindow(PCMWindow *)));

  // Couple the pose changes in the main canvas to changes in the PCMPyramidView
  if (window_type == PCMPyramidView) {
    browseview_canvas = window->Canvas()->QGLCanvasPtr();
    main_canvas = Canvas()->QGLCanvasPtr();
    connect(main_canvas, SIGNAL(CentreOfCanvasChanged(double, double)),
            browseview_canvas, SLOT(SetDataOffsetInCanvas(double, double)));
    connect(main_canvas, SIGNAL(CanvasScaled(float)),
            browseview_canvas, SLOT(ScaleCanvas(float)));
    connect(main_canvas, SIGNAL(CanvasPainted()),
            browseview_canvas, SLOT(PaintCanvas()));
  }
}

void PointCloudMapper::TransferDataFromSubWindow(PCMWindow *window)
{
  // Clear selections
  ClearSelections(NumDataTypes);

  // Copy laser data
  laser_points.swap(window->PointCloud()->LaserPointsReference());
  if (laser_points.size()) laser_points.DeriveDataBounds(0);

  // Copy last reconstruction
  last_model_points.swap(window->LastModelPoints()->ObjectPointsRef());
  last_model_part = *(window->LastModelPart());

  // Display objects that were shown in sub window
  ShowData(PCMWindowPtr(), MapData, window->DataIsShown(MapData),
           SelectedMapData, false, false, true);
  ShowData(PCMWindowPtr(), MapPartitionData,
           window->DataIsShown(MapPartitionData), SelectedMapPartitionData,
           false, false, true);
  ShowData(PCMWindowPtr(), ModelData, window->DataIsShown(ModelData),
           SelectedModelData, false, false, true);
  ShowData(PCMWindowPtr(), LaserData, window->DataIsShown(LaserData),
           LaserData, false, false, true);
  ShowData(PCMWindowPtr(), LastModelData, window->DataIsShown(LastModelData),
           LastModelData, false, false, true);

  // Copy transformation parameters

  // Refresh canvas
  canvas->update();

  // Close sub window
  delete window;
}

void PointCloudMapper::AddDataFromSubWindow(PCMWindow *window)
{
  // Copy laser data to the main window
  if (!window->PointCloud()->empty())
    laser_points.insert(laser_points.end(),
                        window->PointCloud()->begin(),
                        window->PointCloud()->end());
  if (laser_points.size()) laser_points.DeriveDataBounds(0);

  // Display laser data
  ShowData(PCMWindowPtr(), LaserData, true, LaserData, false, false, true);

  // Refresh canvas
  canvas->update();
}

void PointCloudMapper::ClearSelections(DataType type)
{
  switch (type) {
    default:
    case SelectedMapData:
      selected_map_data.Clear();
      nearby_map_data.Clear();
      if (type == SelectedMapData) break;

    case SelectedMapPartitionData:
      selected_map_part_data.Clear();
      nearby_map_part_data.Clear();
      if (type == SelectedMapPartitionData) break;

    case SelectedModelData:
      selected_model_data.Clear();
      nearby_model_data.Clear();
      if (type == SelectedModelData) break;

    case SelectedModelPartData:
      selected_model_part_data.Clear();
      nearby_model_part_data.Clear();
      if (type == SelectedModelPartData) break;

    case SelectedModelFaceData:
      selected_model_face_data.Clear();
      nearby_model_face_data.Clear();
      if (type == SelectedModelFaceData) break;
  }
}

void PointCloudMapper::WriteDefaults(const char *filename) const
{
  FILE *fd;
  int data_app;
  
  fd = fopen(filename, "w");
  if (fd == NULL) return; // Return silently
  
  BNF_Write_String(fd, "pcmdefaults", 0, NULL);
  
  // Save data appearances
  for (data_app=0; data_app<NumDataTypes; data_app++)
    appearance[data_app]->Write(fd, 2);
  
  // Save segmentation parameters
  segmentation_parameters->Write(fd, 2);
  
  // Save outlining parameters
  outlining_parameters->Write(fd, 2);
  
  // Save fitting parameters
  fitting_parameters->Write(fd, 2);
  
  // Save filtering parameters
  filtering_parameters->Write(fd, 2);
  
  BNF_Write_String(fd, "endpcmdefaults", 0, NULL);
  fclose(fd);
}

void PointCloudMapper::ReadDefaults(const char *filename)
{
  FILE *fd;
  char *buffer, *line, *keyword;
  int  keyword_length, data_type;
    
  fd = fopen(filename, "r");
  if (fd == NULL) return; // Return silently
  
  buffer = (char *) malloc(MAXCHARS);

  // Check first keyword
  do {
    line = fgets(buffer, MAXCHARS, fd);
  } while (line && Is_Comment(line));
  if (line == NULL) {
    fprintf(stderr, "Error reading pcm.ini");
    fclose(fd);
    return;
  }
  keyword = BNF_KeyWord(line, &keyword_length);
  if (strncmp(keyword, "pcmdefaults", max(keyword_length, 11))) {
    fprintf(stderr, "Error: pcm.ini does not start with keyword pcmdefaults:\n");
    fclose(fd);
    return;
  }

  // Read all further lines
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        if (!strncmp(keyword, "dataappearance", max(keyword_length, 14))) {
          // The next line should contain the data type
          line = fgets(buffer, MAXCHARS, fd);
          keyword = BNF_KeyWord(line, &keyword_length);
          if (strncmp(keyword, "datatype", max(keyword_length, 8))) {
            printf("Syntax error in pcm.ini\n");
            printf("Keyword dataappearance should be followed by keyword datatype\n");
            free(buffer);
            fclose(fd);
            return;
          }
          // Read the appearance defaults
          else {
            data_type = BNF_Integer(line);
            appearance[data_type]->Read(fd);
            appearance[data_type]->Update();
          }
        }
          
        // Read segmentation parameters
        else if  (!strncmp(keyword, "segmentationparameters", max(keyword_length, 22))) {
          segmentation_parameters->Read(fd);
          general_segmentation_parameters->Update();
          connected_component_parameters->Update();
          surface_growing_parameters->Update();
          segment_growing_parameters->Update();
          mean_shift_parameters->Update();
          majority_filtering_parameters->Update();
          surface_merging_parameters->Update();
        }
        
        // Read outlining parameters
        else if  (!strncmp(keyword, "outliningparameters", max(keyword_length, 19))) {
          outlining_parameters->Read(fd);
          outlining_parameters->Update();
        }
        
        // Read fitting parameters
        else if  (!strncmp(keyword, "fittingparameters", max(keyword_length, 17))) {
          fitting_parameters->Read(fd);
          fitting_parameters->Update();
        }
        
        // Read filtering parameters
        else if  (!strncmp(keyword, "filteringparameters", max(keyword_length, 19))) {
          filtering_parameters->Read(fd);
          filtering_parameters->Update();
        }
        
        
        // End of pcm defaults block
        else if  (!strncmp(keyword, "endpcmdefaults", max(keyword_length, 14))) {
          free(buffer);
          fclose(fd);
          return;
        }
      }
    }
  }
}

void PointCloudMapper::NextUndecidedRoof()
{
  vector <char *>::iterator roof_record;
  int record_number, undecided_record_number, decision;
  bool found;
  char *ch;
  
  for (roof_record=roof_records.begin()+current_roof_record,
       record_number=current_roof_record, found=false;
       roof_record!=roof_records.end() && !found;
	   roof_record++, record_number++) {
    // Retrieve the verification status stored in the last character
    // of the roof record
    ch = *roof_record + strlen(*roof_record) - 1;
    if (*ch == 10) ch--; // Correction for line feed
    sscanf(ch, "%d", &decision);
    if (decision == 1) {
      found = true;
      undecided_record_number = record_number;
    }
  }

  if (!found) {
    QMessageBox::information(this, "Error",
                  QString("All roofs have been verified already!"));
    return;
  }
  
  // Load the first undecided roof
  SwitchProject(undecided_record_number - current_roof_record);
}


void PointCloudMapper::SwitchProject(int increment)
{
  char *dot, *ch;
  int  project_number = -1, num_digits=5, decision, i;
  
  // Retrieve project number
  if (project_file != NULL) {
    dot = strrchr(project_file, '.');
    if (dot != NULL) {
      *dot = 0;
      if (*(dot-6) >= '0' && *(dot-6) <= '9') num_digits = 6;
      sscanf(dot-num_digits, "%d", &project_number);
      *dot = '.';
    }
    else {
      QMessageBox::information(this, "Error",
                  QString("Unable to retrieve project number from file name ") +
                  QString(project_file));
      return;
    }
  }

  // Determine the next project number
  if (roof_records.size()) { // Retrieve building number from roof record
    if (current_roof_record + increment < 0) {
      QMessageBox::information(this, "Error",
                             QString("Negative roof record numbers are not allowed."));
      return;
    }
    else if (current_roof_record + increment >= (int) roof_records.size()) {
      QMessageBox::information(this, "Error",
                             QString("Roof record number beyond end of record list."));
      return;
    }
    current_roof_record += increment;
    sscanf(roof_records[current_roof_record], "%d", &project_number);
    
    // Retrieve the current verification status stored in the last character
    // of the roof record
    ch = roof_records[current_roof_record] +
	     strlen(roof_records[current_roof_record]) - 1;
    if (*ch == 10) ch--; // Correction for line feed
    sscanf(ch, "%d", &decision);
    
    // Set the buttons
    for (i=0; i<3; i++)
      roof_verification_button[i]->setChecked(decision == i);
  }
  else // Just increment the project number
    project_number += increment;

  // Compose new file name
  if (project_number < 0) {
    QMessageBox::information(this, "Error",
                             QString("Negative project numbers are not allowed."));
    return;
  }
  if (num_digits == 5) sprintf(dot-5, "%5d", project_number);
  else sprintf(dot-6, "%6d", project_number);
  ch = dot - num_digits;
  while (ch != dot) { // Replace spaces with zeros
    if (*ch == ' ') *ch = '0';
    ch++;
  }
  *dot = '.';

  // Check if the file exists
  if (!BNF_FileExists(project_file)) {
    QMessageBox::information(this, "Error",
                             QString("Project file ") +
                             QString(project_file) + 
                             QString("does not exist."));
    return;
  }
  
  // Remove old laser data
  laser_points.ErasePoints();
  
  // Only delete model data if the project file is not a laser data file 
  if (strstr(project_file, ".laser") == NULL &&
      strstr(project_file, ".las") == NULL &&
	  strstr(project_file, ".laz") == NULL) {
    buildings.Erase();
    map_points.Erase();
    model_points.Erase();
  }
  
  // Read new project
  OpenProject(project_file, NULL, reset_on_loading_points->isChecked());
  
  if (roof_records.size())
    statusBar()->showMessage(roof_records[current_roof_record], 10000);
  else
    statusBar()->showMessage("Project " + QString(project_file) + " opened.", 2000);
}

void PointCloudMapper::SetLowestPointForWallReconstruction()
{
  LaserPoints *active_laser_points;
  DataBoundsLaser bounds;
  
  // Check if there are laser points
  if (!selected_laser_points.empty())
    active_laser_points = &selected_laser_points;
  else if (!laser_points.empty())
    active_laser_points = &laser_points;
  else {
    statusBar()->showMessage("No laser points available.", 4000);
    return;
  }

  // Determine lowest point
  bounds = active_laser_points->DeriveDataBounds(0);
  local_ground_height = bounds.Minimum().Z();
}

void PointCloudMapper::ReconstructRoofCorner()
{
  vector <int>             segment_numbers;
  vector <int>::iterator   segment_number;
  Planes                   planes;
  ObjectPoint              new_point;
  ObjectPoints::iterator   map_point;
  int                      success, new_number;
  PointNumber              highest_number;
  LineTopology::iterator   node;
  Line3D                   intersection_line;
  Position3D               pos;
  Position2D               pos2D, map_pos;
  LineTopologies::iterator map_line;
  LineSegments2D           intersection_segments;
  LineSegments2D::iterator intersection_segment;
  double                   min_dist, dist;
  
  // Check if we're editing a building model
  if (edit_model_window == NULL || !edit_model_action->isChecked()) {
    QMessageBox::information(this, "Error",
     "Roof corner reconstruction is only available when editing a building model");
    return;
  }
  
  // Determine number of selected laser segment
  if (selected_laser_points.empty()) {
    QMessageBox::information(this, "Error", "No selected laser data");
    return;
  }
  segment_numbers = selected_laser_points.AttributeValues(SegmentNumberTag);
  if (segment_numbers.empty()) {
    QMessageBox::information(this, "Error", "Selected laser data is not segmented");
    return;
  }
  
  // Reconstruct the planes
  for (segment_number=segment_numbers.begin();
       segment_number!=segment_numbers.end(); segment_number++)
    planes.push_back(selected_laser_points.FitPlane(*segment_number,
                             *segment_number, SegmentNumberTag));  
  
  // Determine next model point number
  highest_number = last_model_points.HighestPointNumber();
  new_number = highest_number.Number() + 1;
  
  // Select reconstruction method
  if (segment_numbers.size() >= 3) {
    // Intersect (first) three planes
    if (!Intersect2Planes(planes[0], planes[1], intersection_line)) {
      QMessageBox::information(this, "Error",
        "First two segments are parallel. Failure to calculate intersection line.");
      return;
    }
    if (!IntersectLine3DPlane(intersection_line, planes[2], pos)) {
      QMessageBox::information(this, "Error",
        "Third segment parallel to either first or second segment.");
      return;
    }
    new_point.vect() = pos.vect();
    new_point.Number() = new_number;
    last_model_points.push_back(new_point);
  }
  
  else if (segment_numbers.size() == 2 && selected_map_data.size() == 1) {
    // Intersect intersection line of planes with map line
    
    // Intersect the two laser segments
    if (!Intersect2Planes(planes[0], planes[1], intersection_line)) {
      QMessageBox::information(this, "Error",
       "First two segments are parallel. Failure to calculate intersection line.");
      return;
    }
    
    // Intersect the map polygon with the intersection line of the laser segments
    map_line = *(selected_map_data.begin());
    if (!map_line->IntersectPolygonByLine(map_points,
                 intersection_line.ProjectOntoXOYPlane(), 
                 intersection_segments, 0.05, 0.05)) {
      QMessageBox::information(this, "Error",
       "Intersection line of segments does not intersect map line.");
      return;
    }
    
    // Select the point that is nearest to the selected map point
    min_dist = 1.0e20;
    map_pos.X() = selected_point.begin()->X();
    map_pos.Y() = selected_point.begin()->Y();
    for (intersection_segment=intersection_segments.begin();
         intersection_segment!=intersection_segments.end();
         intersection_segment++) {
      pos2D = intersection_segment->BeginPoint();
      dist = pos2D.Distance(map_pos);
      if (dist < min_dist) {
        new_point.X() = pos2D.X();
        new_point.Y() = pos2D.Y();
        if (fabs(intersection_line.Direction().X()) >
            fabs(intersection_line.Direction().Y()))
          new_point.Z() = intersection_line.DetPositionX(pos2D.X()).Z();
        else
          new_point.Z() = intersection_line.DetPositionY(pos2D.Y()).Z();
      }
      pos2D = intersection_segment->EndPoint();
      dist = pos2D.Distance(map_pos);
      if (dist < min_dist) {
        new_point.X() = pos2D.X();
        new_point.Y() = pos2D.Y();
        if (fabs(intersection_line.Direction().X()) >
            fabs(intersection_line.Direction().Y()))
          new_point.Z() = intersection_line.DetPositionX(pos2D.X()).Z();
        else
          new_point.Z() = intersection_line.DetPositionY(pos2D.Y()).Z();
      }
    }
    new_point.Number() = new_number;
    last_model_points.push_back(new_point);
  }

  else if (segment_numbers.size() == 1 && selected_map_data.size() == 1) {
    // Determine height of map line corners in selected plane
    for (node=(*(selected_map_data.begin()))->begin();
         node!=(*(selected_map_data.begin()))->end(); node++) {
      map_point = map_points.PointIterator(*node);
      new_point = *map_point;
      new_point.Z() = planes[0].Z_At(new_point.X(), new_point.Y(), &success);
      new_point.Number() = new_number++;
      last_model_points.push_back(new_point);
    }
  }
  else {
    QMessageBox::information(this, "Error",
     "No reconstruction method defined for the selected number of planes and lines");
    return;
  }
  
  // Make points visible
  appearance[LastModelData]->SetShowPointsSwitch(true);
  appearance[SelectedModelFaceData]->SetShowPointsSwitch(true);
  ShowData(PCMWindowPtr(), LastModelData, true, SelectedModelFaceData, true,
           true, true);
  edit_model_window->AddReconstructedModel(last_model_points,
                                           last_model_part,
                                appearance[LastModelData]->DataAppearancePtr());

}

void PointCloudMapper::Reconstruct3DLineSegment()
{
  vector <int>             segment_numbers;
  vector <int>::iterator   segment_number;
  Planes                   planes;
  int                      first_segment_number, index, new_number;
  PointNumber              highest_number;
  PointNumberList          face1, face2;
  LaserPoints::iterator    point;
  Position3D               point1, point2;
  double                   max_dist = fitting_parameters->MaximumDistancePointToIntersection();
  LineTopology             line_top;
  Buildings::iterator      building;
  Building::iterator       building_part;
  
  // Determine number of selected laser segment
  if (selected_laser_points.empty()) {
    QMessageBox::information(this, "Error", "No selected laser data");
    return;
  }
  segment_numbers = selected_laser_points.AttributeValues(SegmentNumberTag);
  if (segment_numbers.empty()) {
    QMessageBox::information(this, "Error", "Selected laser data is not segmented");
    return;
  }
  if (segment_numbers.size() == 1) {
    QMessageBox::information(this, "Error", "Only one laser segment has been selected");
    return;
  }
  if (segment_numbers.size() > 2) {
    QMessageBox::information(this, "Error", "More than two laser segment have been selected");
    return;
  }

  // Collect the point numbers of the two segments
  first_segment_number = *(segment_numbers.begin());
  for (point=selected_laser_points.begin(), index=0;
       point!=selected_laser_points.end();
       point++, index++)
    if (point->Attribute(SegmentNumberTag) == first_segment_number)
      face1.push_back(PointNumber(index));
    else
      face2.push_back(PointNumber(index));
  
  // Reconstruct the planes
  for (segment_number=segment_numbers.begin();
       segment_number!=segment_numbers.end(); segment_number++)
    planes.push_back(selected_laser_points.FitPlane(*segment_number,
                             *segment_number, SegmentNumberTag));  
  
  if (!selected_laser_points.IntersectFaces(face1, face2,
                                            *(planes.begin()),
                                            *(planes.begin()+1),
                                            max_dist,
                                            point1, point2)) {
    QMessageBox::information(this, "Error", "No intersection line detected.");
    return;
  }

  // Determine next model point number
  highest_number = model_points.HighestPointNumber();
  new_number = highest_number.Number() + 1;

  // Store new points
  model_points.push_back(ObjectPoint(point1.vect(), PointNumber(new_number),
                                     Covariance3D()));
  model_points.push_back(ObjectPoint(point2.vect(), PointNumber(new_number+1),
                                     Covariance3D()));

  // Store new topology in a new building
  line_top.push_back(PointNumber(new_number));
  line_top.push_back(PointNumber(new_number+1));
  buildings.push_back(Building(buildings.NextNumber()));
  building = buildings.end() - 1;
  building->AddModelData(&model_points);
  building->push_back(BuildingPart(1, &map_points));
  building_part = building->end() - 1;
  building_part->AddModelData(line_top, &model_points);
  
  // Make the line visible
  appearance[ModelData]->SetShowLinesSwitch(true);
  ShowData(PCMWindowPtr(), ModelData, true, SelectedModelData, true,
           true, true);
}

void PointCloudMapper::ReconstructWalls()
{
  printf("Function PointCloudMapper::ReconstructWalls is not yet implemented\n");
}

// Set up new edit building model based on a selection from the main canvas
void PointCloudMapper::EditBuildingModel()
{
  int          building_number;
  BuildingPart *building_part;
  Building     *building;
  LaserPoints  *active_laser_points;
  
  // Deal with switching off the edit mode first
  if (!edit_model_action->isChecked()) {
    edit_model_window = NULL;
    return;
  }
  
  // Determine selected building part
  building_part = NULL;
  if (!selected_model_part_data.empty()) {
    building_number = (*(selected_model_part_data.begin()))->Attribute(BuildingNumberTag);
    building = buildings.BuildingPtr(building_number);
    if (building)
      building_part =
        building->BuildingPartPtr(**(selected_model_part_data.begin()),
                                  ModelData);
  }
  else if (!selected_model_data.empty()) {
    building_number = (*(selected_model_data.begin()))->Attribute(BuildingNumberTag);
    // Take first building part, if any
    building = buildings.BuildingPtr(building_number);
    if (!building->empty()) building_part = &*(building->begin());
  }
  else if (!selected_map_part_data.empty()) {
    building_number = (*(selected_map_part_data.begin()))->Attribute(BuildingNumberTag);
    building = buildings.BuildingPtr(building_number);
    if (building)
      building_part =
        building->BuildingPartPtr(**(selected_map_part_data.begin()),
                                  MapPartitionData);
  }
  else if (!selected_map_data.empty()) {
    building_number = (*(selected_map_data.begin()))->Attribute(BuildingNumberTag);
    // Take first building part, if any
    building = buildings.BuildingPtr(building_number);
    if (!building->empty()) building_part = &*(building->begin());
  }
  else if (last_model_part.ContainsData()) { // Use previous last model if any
    printf("Last model contains %d roof faces and %d wall faces\n",
           (int) last_model_part.RoofFaces()->size(),
           (int) last_model_part.WallFaces()->size());
    building_part = &last_model_part;
  }
  else { // No data selected, create new building with empty map outline
    building_number = buildings.NextNumber();
    buildings.push_back(Building(building_number, &map_points));
    building = buildings.BuildingPtr(building_number);
  }

  // If there's no building part, create a new one
  if (building_part == NULL) {
    building->push_back(BuildingPart(building->NextPartNumber(), &map_points));
    building_part = &*(building->end()-1);
  }

  // Copy the selected building model data to the last reconstructed model buffers
  // Don't copy if we want to edit the current last reconstructed model
  if (building_part != &last_model_part) {
    last_model_points.Erase();
    building_part->CollectModelPoints(last_model_points);
    last_model_part = *building_part;
    last_model_part.AddModelData(&last_model_points);
  }

  // Create a new window
  edit_model_window = new PCMWindow(PCMResult, NULL, Qt::Window);
  subwindows.push_back(edit_model_window);
  edit_model_window->setWindowTitle(QString("Reconstructed (part of) building %1").
                                    arg(building_number));
  edit_model_window->AddReconstructedModel(last_model_points, last_model_part,
                                appearance[LastModelData]->DataAppearancePtr());
  if (selected_laser_points.empty()) active_laser_points = &laser_points;
  else active_laser_points = &selected_laser_points;
  edit_model_window->AddLaserData(active_laser_points->LaserPointsReference(),
                                  result_appearance->DataAppearancePtr(), false);
  edit_model_window->show();

  // Add connections for the various updates
  AddShowDataConnectors(edit_model_window);
  connect(edit_model_window, SIGNAL(WindowWillBeClosed(PCMWindow *)),
          this, SLOT(RemovePCMWindow(PCMWindow *)));
  connect(edit_model_window, SIGNAL(RequestSavingReconstructedModel(PCMWindow *)),
          this, SLOT(SaveReconstructedModel(PCMWindow *)));
  connect(edit_model_window, SIGNAL(RequestMoveToMainWindow(PCMWindow *)),
          this, SLOT(TransferDataFromSubWindow(PCMWindow *)));
  connect(edit_model_window, SIGNAL(RequestAddToMainWindow(PCMWindow *)),
          this, SLOT(AddDataFromSubWindow(PCMWindow *)));
  connect(edit_model_window, SIGNAL(RequestEditingBuildingModel(PCMWindow *)),
          this, SLOT(EditBuildingModel(PCMWindow *)));
}

// Set up new edit building model based on a selected result window
void PointCloudMapper::EditBuildingModel(PCMWindow *window)
{
  // Remove old last model data
  last_model_points.Erase();
  last_model_part.DeleteData();
  
  // Copy data from the result window
  last_model_points = *(window->LastModelPoints());
  last_model_part = *(window->LastModelPart());
  last_model_part.AddModelData(&last_model_points);

  // Set main canvas in building edit mode
  edit_model_action->setChecked(true);
  edit_model_window = window;
  
  // Make sure the last building model is visible
  if (!show_data_actions[LastModelData]->isChecked()) {
    show_data_actions[LastModelData]->setChecked(true);
    ShowLastModelData();
  }
}
