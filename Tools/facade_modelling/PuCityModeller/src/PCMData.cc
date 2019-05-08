
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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "PointCloudMapper.h"
#include <QMessageBox>
#include <QStatusBar>
#include "BNF_io.h"

extern int FileExists(const char *);

/*
--------------------------------------------------------------------------------
                           Set file names
--------------------------------------------------------------------------------
*/

void PointCloudMapper::SetMapPointFile(const char *point_file)
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
  if (strstr(project_file, ".laser") != NULL) {
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
  if (strncmp(keyword, "pointcloudmapping", MAX(keyword_length, 17))) {
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
	
	if (!strncmp(keyword, "project", MAX(keyword_length, 7)))
	  project_name = BNF_String(line);

        else if (!strncmp(keyword, "map_points", MAX(keyword_length, 10)))
          map_point_file = BNF_String(line);

        else if (!strncmp(keyword, "map_topology", MAX(keyword_length, 12)))
          map_top_file = BNF_String(line);

        else if (!strncmp(keyword, "model_points", MAX(keyword_length, 12)))
          model_point_file = BNF_String(line);

        else if (!strncmp(keyword, "model_topology", MAX(keyword_length, 14)))
          model_top_file = BNF_String(line);

        else if (!strncmp(keyword, "laser_metadata", MAX(keyword_length, 14)))
          laser_meta_file = BNF_String(line);

        else if (!strncmp(keyword, "laser_pyramid", MAX(keyword_length, 13)))
          laser_pyramid_file = BNF_String(line);

        else if (!strncmp(keyword, "laser_points", MAX(keyword_length, 12))) {
          laser_point_file = BNF_String(line);
        }

        else if (!strncmp(keyword, "endpointcloudmapping",
                          MAX(keyword_length, 20))) {
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
    buildings.MergePCMMapData(map_point_file, map_top_file, map_points);
  if (model_point_file && model_top_file)
    buildings.MergePCMModelData(model_point_file, model_top_file, model_points);
  if (laser_point_file) {
    laser_points.Read(laser_point_file, false);
    laser_points.DeriveDataBounds(0);
  }
  if (laser_meta_file) {
    // Read meta data
    laser_block.Create(laser_meta_file, &fileclass);
    // Try to read all points up to a maximum of 1.000.000
    int numpts = laser_block.ReadPoints(1000000);
    if (numpts == -1)
      printf("Block contains more than 1.000.000 laser points.\n");
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
      return last_model_part.ContainsData(ModelData);
    case SelectedMapData:
      return selected_map_data.size() > 0;
    case SelectedMapPartitionData:
      return selected_map_part_data.size() > 0;
    case SelectedModelData:
      return selected_model_data.size() > 0;
    case SelectedModelPartData:
      return selected_model_part_data.size() > 0;
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
  int                          line_number, part_number, building_number,
                               pol_number;
  bool                         change_pol=false;

  if (nearby_data.empty()) return;
  // Get the ID's of the data to be added
  line_number = part_number = (*reference)->Number();
  building_number = (*reference)->Number() / 1000;
  for (polygon=nearby_data.begin(); polygon!=nearby_data.end(); polygon++) {
    pol_number = (*polygon)->Number();
    // Check if the nearby data matches the ID
    switch (type) {
      case MapData:
      case SelectedMapData:
        change_pol = (pol_number == line_number); break;
      case MapPartitionData:
      case SelectedMapPartitionData:
        change_pol = (pol_number == line_number);
        if (change_reference_line_only && *polygon != *reference)
          change_pol = false;
        break;
      case ModelData:
      case SelectedModelData:
        change_pol = (pol_number / 1000 == building_number);
        break;
      case SelectedModelPartData:
        change_pol = (pol_number == part_number); break;
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

  // Store selected points
  if (selected_point.size())
    selected_point.erase(selected_point.begin(), selected_point.end());
  if (CorrespondingPoints(base_type) == &map_points) {
    selected_point.push_back(*(map_points.GetPoint(number)));
    sel_point = map_points.ConstPointIterator(number);
    statusBar()->showMessage(QString("Selected map point %1 (%2, %3, %4)")
           .arg(number.Number())
           .arg(sel_point->X(), 0, 'f', 2)
           .arg(sel_point->Y(), 0, 'f', 2)
           .arg(sel_point->Z(), 0, 'f', 2));
  }
  else {
    selected_point.push_back(*(model_points.GetPoint(number)));
    sel_point = model_points.ConstPointIterator(number);
    statusBar()->showMessage(QString("Selected model point %1 (%2, %3, %4)")
           .arg(number.Number())
           .arg(sel_point->X(), 0, 'f', 2)
           .arg(sel_point->Y(), 0, 'f', 2)
           .arg(sel_point->Z(), 0, 'f', 2));
  }

  // Select new data
  for (building=buildings.begin(); building!=buildings.end(); building++)
    building->SelectPolygons(polygon_selection, number, selection_type);

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
    // For map partitions, only keep the first line if the mouse is inside.
/*
    if (base_type == MapPartitionData && found) {
      if (polygon_selection.size() > 1)
        polygon_selection.erase(polygon_selection.begin()+1,
                                polygon_selection.end());
    }
*/
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

  // Add new data to the nearby data buffer if there
  nearby_data->insert(nearby_data->begin(),
                      polygon_selection.begin(),polygon_selection.end());

  // Update the selected data buffer
  if (selected_data->Contains(*(nearby_data->begin())))
    // The new nearby data is already selected, therefore unselect that data.
    UpdateSelection(*selected_data, *nearby_data, nearby_data->begin(),
                    point_data, base_type, false, true, true,
                    change_reference_line_only);
  else
    // Add the nearby data to the selection
    UpdateSelection(*selected_data, *nearby_data, nearby_data->begin(),
                    point_data, selection_type, true, true, true,
                    change_reference_line_only);

  polygon_selection.Clear();
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
      selection_type == SelectedModelPartData) {
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
    case LaserData:
    case LastModelData:         
    case SelectedLaserData:        return NULL;
    default:
      printf("Error: Invalid data type in PointCloudMapper::CorrespondingSelection (%d)\n", type);
      return NULL;
  }
  return NULL; // We do not get here. Just to satisfy the compiler
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
    case LastModelData:            return NULL;
    default:
      printf("Error: Invalid data type in PointCloudMapper::CorrespondingNearbyData\n");
      return NULL;
  }
  return NULL; // We do not get here. Just to satisfy the compiler
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
  if (real_type == LaserData || !Canvas()->ShiftKeyDown())
    selected_laser_points.AddTaggedPoints(laser_points,
                                    selected_point.Attribute(SegmentNumberTag),
                                    SegmentNumberTag, IsSelectedTag);

  // or remove points from the selection
  else {
    selected_laser_points.RemoveTaggedPoints(selected_point.Attribute(SegmentNumberTag),
                                             SegmentNumberTag);
    laser_points.ConditionalReTag(1, 0, IsSelectedTag,
                                  selected_point.Attribute(SegmentNumberTag),
                                  SegmentNumberTag);
  } 

  // Update display
  ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, true,
           true, true);
}

void PointCloudMapper::DisplayPointInformation(const LaserPoint &point)
{
  char                *string, *value;
  const unsigned char *tag;
  const int           *attribute;
  const float         *floatattribute;
  int                 i;
  
  string = (char *) malloc(1024);
  value  = (char *) malloc(32);
  
  sprintf(string, "XYZ (%.2f, %.2f, %.2f)", point.X(), point.Y(), point.Z());
  
  for (i=0, tag=point.AttributeTags(), attribute=point.AttributeValues();
       i<point.NumAttributes(); i++, tag++, attribute++) {
    switch (*tag) {            
      case ReflectanceTag: strcat(string, "; Reflectance "); break;
      case PulseCountTag : strcat(string, "; Pulse count ");
                           sprintf(value, "%d", point.PulseCount()); break;
      case LabelTag      : strcat(string, "; Label "); break;
      case IsFilteredTag : strcat(string, "; Is filtered "); break;
      case IsProcessedTag: strcat(string, "; Is processed "); break;
      case IsSelectedTag : strcat(string, "; Is selected "); break;
      case ColourTag     : strcat(string, "; Colour ");
                           sprintf(value, "(%d, %d, %d)",
                           point.Red(), point.Green(), point.Blue()); break; 
      case ResidualTag   : floatattribute = (const float *) attribute;
                           strcat(string, "; Residual ");
                           sprintf(value, "%7.3f", *floatattribute); break;
      case SegmentNumberTag: strcat(string, "; Segment number "); break;
      case PlaneNumberTag: strcat(string, "; Plane number "); break;
      case ScanNumberTag : strcat(string, "; Scan number "); break;
      case PointNumberTag: strcat(string, "; Point number "); break;
      case PulseLengthTag: strcat(string, "; Pulse length "); break;
      case PolygonNumberTag: strcat(string, "; Polygon number "); break;
      default            : strcat(string, "; Undefined ");
                           sprintf(value, "(tag %d) ", *tag);
                           strcat(string, value); break;
    }
    if (*tag != ColourTag && *tag != PulseCountTag && *tag != ResidualTag)
      sprintf(value, "%d", *attribute);
    strcat(string, value); 
  }
  statusBar()->showMessage(QString(string));
  free(string);
  free(value);
}

void PointCloudMapper::DisplayPointInformation(const ObjectPoint &point,
                                               DataType type)
{
  char                *string, *string2;
  Buildings::iterator building;
  BuildingPart        *part;
  
  string = (char *) malloc(1024);
  string2 = (char *) malloc(1024);

  // Find the corresponding building and building part (if available)
  building = buildings.BuildingIterator(point.NumberRef(), type);
  if (building != buildings.end()) {
    sprintf(string, "Building %d, ", building->Number());
    part = building->BuildingPartPtr(point.NumberRef(), type);
    if (part != NULL) {
      sprintf(string2, "part %d, ", part->Number());
      strcat(string, string2);
    }
  }
    
  // Add point information
  if (type == MapData || type == SelectedMapData)  
    sprintf(string2, "map point %d; XYZ (%.2f, %.2f, %.2f)", point.Number(),
            point.X(), point.Y(), point.Z());
  else if (type == ModelData || type == SelectedModelData)
    sprintf(string2, "model point %d; XYZ (%.2f, %.2f, %.2f)", point.Number(),
            point.X(), point.Y(), point.Z());
  strcat(string, string2);
  
  // Display information
  statusBar()->showMessage(QString(string));
  free(string);
  free(string2);
}

void PointCloudMapper::CropLaserData()
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

void PointCloudMapper::DeleteLaserData()
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
                          "No selection box or selected map data available.\n");
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
        Canvas()->SelectLaserData(subunit->LaserPointsReference(),
                                  selection, true, true);
        if (delete_points) subunit->ErasePoints();
      }
    }
  }
  else { // Select from the current data set
    Canvas()->SelectLaserData(selection_source->LaserPointsReference(),
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

void PointCloudMapper::ReadView()      { Canvas()->ReadView(".pcmview"); }
void PointCloudMapper::SaveView()      { Canvas()->SaveView(".pcmview"); }

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
    building_number = (*map_line)->Number() / 1000;
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
  last_model_part = BuildingPart((*map_line)->Number() / 1000, &map_points);

  // Copy all selected map (partition) lines. In case of map partitions,
  // this can be more than one line. In case of map lines, this is not possible,
  // because multiple map lines are already dealt with in the loop above.
  // Use map partition data if a map line is selected and the building contains
  // map partition data to be used (according to the fitting parameters).
  // NOTE: For fitting a roof primitive only the first partition is used!!!
  building = buildings.BuildingPtr((*map_line)->Number() / 1000);
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
                fitting_parameters->FittingParametersRef());
  }
  else { // Fit the selected primitive shape
    success = last_model_part.ReconstructRoofPrimitive(type,
                active_laser_points->LaserPointsReference(), last_model_points);
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
                           arg((*map_line)->Number() / 1000));
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
  }
  // Remove laser data from main window
//  ShowData(PCMWindowPtr(), LaserData, false, LaserData, false, true, false);

// Display the result and store the building number in the building part

  if (success) {
    statusBar()->showMessage("Roof fitting succeeded.", 2000);
    // Store building number as the building part number of the last
    // reconstructed model part
    map_line = selected_data->begin();
    last_model_part.Number() = (*map_line)->Number() / 1000;
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
  reconstructed_model = window->ReconstructedModel();
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
                          window->ReconstructedPoints()->begin(),
                          window->ReconstructedPoints()->end());
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

  // Close the result window
  window->close();
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
    building_number = map_line->Number() / 1000;
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
  LineTopologies::iterator polygon;
  LineTopologies           polygons;
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
  // To be changed for multiple polygons
    EnclosingPolygon(segmentation_parameters->MaxDistanceInComponent(),
    // To be restored to "map_points, polygon"
                     map_points, polygons))
    return;
  
  // If a map outline is selected, add the polygon as map partition to it
  if (selected_map_data.size()) {
    map_line = *selected_map_data.begin();
    building_number = map_line->Number() / 1000;
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
    building_number = map_line->Number() / 1000;
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

void PointCloudMapper::SpawnWindow()
{
  PCMWindow *window;
  int       data_type;

  // Create a new window
  window = new PCMWindow(PCMView, NULL, Qt::Window);
// TODO: Check if the destructor is called for PCMWindow
  subwindows.push_back(window);
  window->setWindowTitle("View window");
  window->AddLaserData(laser_points,
                       appearance[LaserData]->DataAppearancePtr(), false);
  for (data_type=0; data_type<NumNormalDataTypes; data_type++)
    if (DataIsShown((DataType) data_type) && data_type != LastModelData)
      DisplayData((DataType) data_type, window);
  window->Canvas()->InitialiseTransformation();
  window->show();

  // Add connections for the various updates
  AddShowDataConnectors(window);
  connect(window, SIGNAL(WindowWillBeClosed(PCMWindow *)),
          this, SLOT(RemovePCMWindow(PCMWindow *)));
  connect(window, SIGNAL(RequestMoveToMainWindow(PCMWindow *)),
          this, SLOT(TransferDataFromSubWindow(PCMWindow *)));
}

void PointCloudMapper::TransferDataFromSubWindow(PCMWindow *window)
{
  // Clear selections
  ClearSelections(NumDataTypes);

  // Copy laser data
  laser_points.swap(window->PointCloud()->LaserPointsReference());
  if (laser_points.size()) laser_points.DeriveDataBounds(0);

  // Copy last reconstruction
  reconstructed_points.swap(window->ReconstructedPoints()->ObjectPointsRef());
  reconstructed_part = *(window->ReconstructedModel());

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
  window->close();
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
  
  // Save fitting parameters
  fitting_parameters->Write(fd, 2);
  
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
  if (strncmp(keyword, "pcmdefaults", MAX(keyword_length, 11))) {
    fprintf(stderr, "Error: pcm.ini does not start with keyword pcmdefaults:\n");
    fclose(fd);
    return;
  }

  // Read all further lines
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        if (!strncmp(keyword, "dataappearance", MAX(keyword_length, 14))) {
          // The next line should contain the data type
          line = fgets(buffer, MAXCHARS, fd);
          keyword = BNF_KeyWord(line, &keyword_length);
          if (strncmp(keyword, "datatype", MAX(keyword_length, 8))) {
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
        else if  (!strncmp(keyword, "segmentationparameters", MAX(keyword_length, 22))) {
          segmentation_parameters->Read(fd);
          segmentation_parameters->Update();
        }
        
        // Read fitting parameters
        else if  (!strncmp(keyword, "fittingparameters", MAX(keyword_length, 17))) {
          fitting_parameters->Read(fd);
          fitting_parameters->Update();
        }
        
        
        // End of pcm defaults block
        else if  (!strncmp(keyword, "endpcmdefaults", MAX(keyword_length, 14))) {
          free(buffer);
          fclose(fd);
          return;
        }
      }
    }
  }
}

void PointCloudMapper::SwitchProject(int offset)
{
  char *dot, *ch;;
  int  project_number = -1;
  
  // Retrieve project number
  if (project_file != NULL) {
    dot = strrchr(project_file, '.');
    if (dot != NULL) {
      *dot = 0;
      sscanf(dot-5, "%d", &project_number);
      *dot = '.';
    }
    else {
      QMessageBox::information(this, "Error",
                  QString("Unable to retrieve project number from file name ") +
                  QString(project_file));
      return;
    }
  }
  
  // Compose new file name
  project_number += offset;
  if (project_number == -1) {
    QMessageBox::information(this, "Error",
                             QString("Current project is already project 0."));
    return;
  }
  sprintf(dot-5, "%5d", project_number);
  ch = dot - 5;
  while (ch != dot) { // Replace spaces with zeros
    if (*ch == ' ') *ch = '0';
    ch++;
  }
  *dot = '.';

  // Check if the file exists
  if (!FileExists(project_file)) {
    QMessageBox::information(this, "Error",
                             QString("Project file ") +
                             QString(project_file) + 
                             QString("does not exist."));
    return;
  }
  
  // Remove old data
  buildings.Erase();
  map_points.Erase();
  model_points.Erase();
  laser_points.ErasePoints();
  
  // Read new project
  OpenProject(project_file);
  statusBar()->showMessage("Project " + QString(project_file) + " opened.",2000);

}
