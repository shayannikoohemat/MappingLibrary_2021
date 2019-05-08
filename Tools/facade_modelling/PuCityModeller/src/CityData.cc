
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
#include "City.h"
#include <QMessageBox>
#include <QFileDialog>
#include <QStatusBar>
#include "BNF_io.h"
#include "DataTypes.h"

/*
--------------------------------------------------------------------------------
                           Set file names
--------------------------------------------------------------------------------
*/

void CityWindow::SetMapPointFile(const char *point_file)
{
  if (point_file) {
    map_point_file = (char *) malloc(strlen(point_file) + 1);
    strcpy(map_point_file, point_file);
  }
  else map_top_file = NULL;
}

void CityWindow::SetMapTopologyFile(const char *top_file)
{
  if (top_file) {
    map_top_file = (char *) malloc(strlen(top_file) + 1);
    strcpy(map_top_file, top_file);
  }
  else map_top_file = NULL;
}

void City::SetModelPointFile(const char *point_file)
{
  if (point_file) {
    model_point_file = (char *) malloc(strlen(point_file) + 1);
    strcpy(model_point_file, point_file);
  }
  else model_top_file = NULL;
}

void City::SetModelTopologyFile(const char *top_file)
{
  if (top_file) {
    model_top_file = (char *) malloc(strlen(top_file) + 1);
    strcpy(model_top_file, top_file);
  }
  else model_top_file = NULL;
}

void City::SetLaserMetaFile(const char *meta_file)
{
  if (meta_file) {
    laser_meta_file = (char *) malloc(strlen(meta_file) + 1);
    strcpy(laser_meta_file, meta_file);
  }
  else laser_meta_file = NULL;
}

void City::SetLaserPointFile(const char *point_file)
{
  if (point_file) {
    laser_point_file = (char *) malloc(strlen(point_file) + 1);
    strcpy(laser_point_file, point_file);
  }
  else laser_point_file = NULL;
}

void City::SetCurrentDirectory(QString *dir)
{
  QFileInfo fileinfo(*dir);
 
  current_dir = fileinfo.absoluteDir().absolutePath ();
 
//  printf("Current directory is: %d *\n",current_dir.toAscii());
  cout<<current_dir.toStdString ()<<endl; 
}




/*
--------------------------------------------------------------------------------
                       Read all mapping data
--------------------------------------------------------------------------------
*/

bool City::ReadProject(const char *filename, bool meta_data_only)
{
  FILE *fd;
  char *buffer, *line, *keyword;
  int  keyword_length, fileclass;
  bool finished;

// Initialise all meta data

  project_file = (char *) malloc(strlen(filename) + 1);
  strcpy(project_file, filename);
  map_point_file = map_top_file = NULL;
  model_point_file = model_top_file = NULL;
  laser_meta_file = laser_point_file = NULL;

// Open the file

  fd = fopen(project_file, "r");
  if (fd == NULL) {
    fprintf(stderr, "Error opening point cloud mapping file %s\n",
            project_file);
    return(false);
  }
  
// Check if we may be dealing with a laser file only
  if (strstr(project_file, ".laser") != NULL) {
    laser_point_file = (char *) malloc(strlen(project_file) + 1);
    strcpy(laser_point_file, project_file);
    fclose(fd);
    if (meta_data_only) return true;
    laser_points.Read(laser_point_file);
    laser_points.DeriveDataBounds(0);
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
/*
  if (map_point_file && map_top_file)
    buildings.MergeCityMapData(map_point_file, map_top_file, map_points);
  if (model_point_file && model_top_file)
    buildings.MergeCityModelData(model_point_file, model_top_file, model_points);
    */
  if (laser_point_file) {
    laser_points.Read(laser_point_file);
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
  return true;
}

bool CityWindow::ContainsData(DataType type) const
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

void CityWindow::UpdateSelection(LineTopsIterVector &selection,
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

void CityWindow::SelectObjectData(const PointNumber &number,
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
                                    
    //selected_point.push_back(*(map_points.GetPoint(number)));
    
    selected_point.push_back(map_points.PointByNumber(number.Number()));
    
    sel_point = map_points.ConstPointIterator(number);
    statusBar()->showMessage(QString("Selected map point %1 (%2, %3, %4)")
           .arg(number.Number())
           .arg(sel_point->X(), 0, 'f', 2)
           .arg(sel_point->Y(), 0, 'f', 2)
           .arg(sel_point->Z(), 0, 'f', 2));

    Canvas()->RemoveObjectData(selected_point_appearance);
    if (!selected_point_appearance)
      selected_point_appearance = new DataAppearance(SelectedPointData);
    Canvas()->AddObjectData(&selected_point, (LineTopology *) NULL,
                             selected_point_appearance);
     
           
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

void City::ToggleSelectionData(DataType selection_type)
{
     
      printf("toggle selection data\n");
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

ObjectPoints *CityWindow::CorrespondingPoints(DataType type)
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

LineTopsIterVector *CityWindow::CorrespondingSelection(DataType type)
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
      printf("Error: Invalid data type in City::CorrespondingSelection (%d)\n", type);
      return NULL;
  }
  return NULL; // We do not get here. Just to satisfy the compiler
}

LineTopsIterVector *CityWindow::CorrespondingNearbyData(DataType type)
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
      printf("Error: Invalid data type in City::CorrespondingNearbyData\n");
      break;
  }
  return NULL;
}

void CityWindow::SelectInsideRectangle(MouseMode selection_mode)
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

void CityWindow::SelectLaserData()
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
    ShowData(CityWindowPtr(), LaserData, true, SelectedLaserData, false,
             true, true);
  }
  else
    QMessageBox::information(this, "Error",
                          "No selection box or selected map data available.\n");
}

void CityWindow::SelectLaserData(const LaserPoint &selected_point,
                                       DataType type)
{
  DataType real_type;
  int number; 
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
    {
                
                number=selected_point.Attribute(SegmentNumberTag);
               // printf("selected segment number is: %i\n, totally %i points.", number, laser_points.SelectTagValue(SegmentNumberTag,number).size());
                // statusBar()->showMessage(QString("Select asdfasdf"));
                
                emit Show_segment_info((int)(selected_point.Attribute(SegmentNumberTag)), feature_type);
    selected_laser_points.AddTaggedPoints(laser_points,
                                    selected_point.Attribute(SegmentNumberTag),
                                    SegmentNumberTag, IsSelectedTag);
         
                                    
}
  // or remove points from the selection
  else {
    selected_laser_points.RemoveTaggedPoints(selected_point.Attribute(SegmentNumberTag),
                                             SegmentNumberTag);
    laser_points.ConditionalReTag(1, 0, IsSelectedTag,
                                  selected_point.Attribute(SegmentNumberTag),
                                  SegmentNumberTag);
  } 

  // Update display
  ShowData(CityWindowPtr(), LaserData, true, SelectedLaserData, true,
           true, true);
}

void CityWindow::CropLaserData()
{
  if (Canvas()->HasValidSelectionRectangle())
    SelectLaserDataByBox(false, true, true);
  else if (selected_map_data.size() || selected_map_part_data.size())
    SelectLaserDataByMap(false, true, true);
  else if (selected_laser_points.size()) { // Crop to selected laser points
    laser_points.ErasePoints();
    laser_points.swap(selected_laser_points);
    laser_points.DeriveDataBounds(0);
    laser_points.RemoveAttribute(IsSelectedTag);
    if (laser_points.GetTIN() != NULL || selected_laser_points.GetTIN() != NULL)
      laser_points.DeriveTIN();
    
    
      
    ShowData(CityWindowPtr(), LaserData, true, SelectedLaserData, false,
             true, true);
    emit LaserChanged(feature_type);     
    
  }
  else
    QMessageBox::information(this, "Error",
                          "No selection box or selected map data available.\n");
}

void CityWindow::DeleteLaserData()
{
  if (Canvas()->HasValidSelectionRectangle())
    SelectLaserDataByBox(false, false, true);
  else if (selected_map_data.size() || selected_map_part_data.size())
    SelectLaserDataByMap(false, false, true);
  else if (selected_laser_points.size()) {
    laser_points.RemoveTaggedPoints(1, IsSelectedTag);
    selected_laser_points.ErasePoints();
    laser_points.DeriveDataBounds(0);
    if (laser_points.GetTIN() != NULL) laser_points.DeriveTIN();
    ShowData(CityWindowPtr(), LaserData, !laser_points.empty(),
             SelectedLaserData, false, true, true);       
    emit LaserChanged(feature_type); 
  }
  else
    QMessageBox::information(this, "Error",
                          "No selection box or selected map data available.\n");
}

void CityWindow::SelectLaserDataByBox(bool from_block, bool inside,
                                            bool show_data)
{
  LaserBlock::iterator               unit;
  LaserUnit::iterator                subunit;
  LaserPoints                        selection, tile_points;
  LaserPoints::iterator              point;
  bool                               from_laser_points, delete_laser_data;

  // Select from visible points to the selected laser point set if
  // the shift key is down
  from_laser_points = Canvas()->ShiftKeyDown();
  if (!from_laser_points && selected_laser_points.size())
    selected_laser_points.ErasePoints();

  // Select all points inside the box
  if (from_block && !from_laser_points) {
    // Select from all laser block sub units
    for (unit=laser_block.begin(); unit!=laser_block.end(); unit++) {
      for (subunit=unit->begin(); subunit!=unit->end(); subunit++) {
        if (subunit->size() == 0) {
          subunit->Read();
          delete_laser_data = true;
        }
        else delete_laser_data = false;
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
                                  selection, inside);
        if (delete_laser_data) subunit->ErasePoints();
      }
    }
  }
  else { // Select from the current data set
    // Always select inside if selection to be made from the laser points
    Canvas()->SelectLaserData(laser_points, selection,
                              inside || from_laser_points);
  }

  // If there already are selected laser points, add or remove the new selection
  if (from_laser_points) {
    if (!from_block && inside) { // Crop selection to bounds
      for (point=selected_laser_points.begin();
           point!=selected_laser_points.end(); point++) {
        if (!selection.Contains(*point)) {
          selected_laser_points.erase(point);
          point--;
        }
      }
    }
    else if (inside) { // Add new selection
      for (point=selection.begin(); point!=selection.end(); point++) {
        if (!selected_laser_points.Contains(*point))
          selected_laser_points.push_back(*point);
      }
    }
    else { // Remove new selection
      for (point=selected_laser_points.begin();
           point!=selected_laser_points.end(); point++) {
        if (selection.Contains(*point)) {
          selected_laser_points.erase(point);
          point--;
        }
      }
    }
  }
  // If no points are selected, just erase the old laser data
  else if (selection.empty()) {
    laser_points.ErasePoints();
  }
  else {
    // Swap the selection with the laser points
    laser_points.swap(selection);
    // Erase the old laser points (now in selection)
    selection.ErasePoints();
    // Recompute bounds of the laser points
    laser_points.DeriveDataBounds(0);
  }

  // Remove the selection rectangle
  Canvas()->RemoveSelectionRectangle(false);

  // Recompute bounds and TINs
  laser_points.DeriveDataBounds(0);
  if (laser_points.GetTIN() != NULL) laser_points.DeriveTIN();
  selected_laser_points.DeriveDataBounds(0);
  if (selected_laser_points.GetTIN() != NULL) selected_laser_points.DeriveTIN();
  
  // Refresh display of laser data
  ShowData(CityWindowPtr(), LaserData, show_data && !laser_points.empty(),
           SelectedLaserData, show_data && !selected_laser_points.empty(),
           true, true);
}

void CityWindow::SelectLaserDataByMap(bool from_block, bool inside,
                                            bool show_data)
{
  LineTopologies                     *map_tops;
  LineTopologies::iterator           top;
  LineTopsIterVector::const_iterator polygon;
  LineTopsIterVector                 *selected_data;
  LaserPoints                        selection;
  LaserPoints::iterator              point;
  LaserBlock::iterator               unit;
  LaserUnit::iterator                subunit;
  bool                               from_laser_points, delete_points;

  // Select from visible points to the selected laser point set if
  // the shift key is down
  from_laser_points = Canvas()->ShiftKeyDown();
  if (!from_laser_points && selected_laser_points.size())
    selected_laser_points.ErasePoints();

  // Get the points and topologies for the selection
  if (selected_map_data.size()) selected_data = &selected_map_data;
  else selected_data  = &selected_map_part_data;
  map_tops = new LineTopologies();
  for (polygon=selected_data->begin(); polygon!=selected_data->end(); polygon++)
    map_tops->push_back(**polygon);

  // Select all points inside the polygons
  if (from_block && !from_laser_points) {
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
    if (!inside && !from_laser_points) {
      QMessageBox::information(this, "Error",
                      "Deleting points inside map polygon is not available.\n");
      return;
    }
    laser_points.Select(selection, map_points,
                        map_tops->LineTopologiesReference());
  }

  // If there already are selected laser points, add or remove the new selection
  if (from_laser_points) {
    if (!from_block && inside) { // Crop selection to bounds
      for (point=selected_laser_points.begin();
           point!=selected_laser_points.end(); point++) {
        if (!selection.Contains(*point)) {
          selected_laser_points.erase(point);
          point--;
        }
      }
    }
    else if (inside) { // Add new selection
      for (point=selection.begin(); point!=selection.end(); point++) {
        if (!selected_laser_points.Contains(*point))
          selected_laser_points.push_back(*point);
      }
    }
    else { // Remove new selection
      for (point=selected_laser_points.begin();
           point!=selected_laser_points.end(); point++) {
        if (selection.Contains(*point)) {
          selected_laser_points.erase(point);
          point--;
        }
      }
    }
  }
  else {
    // Swap the selection with the laser points
    laser_points.swap(selection);
    // Erase the old laser points (now in selection)
    selection.ErasePoints();
    // Recompute bounds
    laser_points.DeriveDataBounds(0);
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
  ShowData(CityWindowPtr(), LaserData, show_data && !laser_points.empty(),
           SelectedLaserData, show_data && !selected_laser_points.empty(),
           true, true);
}

void City::ReadView()      { Canvas()->ReadView(".Cityview"); }
void City::SaveView()      { Canvas()->SaveView(".Cityview"); }

void City::FitViewToSelectedData()
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

void City::DeleteLoosePoints()
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

void CityWindow::PartitionBuilding()
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
  ShowData(CityWindowPtr(), MapPartitionData, true, SelectedMapPartitionData,
           true, true, true);
}

bool City::FitRoof(RoofType type)
{
  LineTopsIterVector           *selected_data;
  LineTopsIterVector::iterator map_line;
  bool                         success;
  CityWindow                    *window;
  LaserPoints                  *active_laser_points;
  
  // Check if there is a selected map or map partition line
  if (selected_map_data.size()) {
    selected_data  = &selected_map_data;
  }
  else if (selected_map_part_data.size()) {
    selected_data  = &selected_map_part_data;
  }
  else {
    QMessageBox::information(this, "Error",
                             "No map (partition) line selected.\n");
    return false;
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
  // NOTE: this doesn't work!! Only one map part allowed in a partition
  for (map_line=selected_data->begin(); map_line!=selected_data->end();
       map_line++)
    last_model_part.AddMapPartitionData((*map_line)->LineTopologyReference());

  if (type == UnknownRoof) { // Automatic reconstruction of roof shape
    /*
    success = last_model_part.ReconstructComplexRoof(
                active_laser_points->LaserPointsReference(), last_model_points,
                fitting_parameters->FittingParametersRef());*/
  }
  else { // Fit the selected primitive shape
    /*success = last_model_part.ReconstructRoofPrimitive(type,
                active_laser_points->LaserPointsReference(), last_model_points);*/
  }
  
// Show the reconstructed model in a new window

  if (success) {
    last_model_part.DeriveResiduals(active_laser_points->LaserPointsReference(),
                                    true);
    // Create a new window
    window = new CityWindow(CityResult, NULL, Qt::Window);
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
    connect(window, SIGNAL(WindowWillBeClosed(CityWindow *)),
            this, SLOT(RemoveCityWindow(CityWindow *)));
    connect(window, SIGNAL(RequestSavingReconstructedModel(CityWindow *)),
            this, SLOT(SaveReconstructedModel(CityWindow *)));
    connect(window, SIGNAL(RequestMoveToMainWindow(CityWindow *)),
            this, SLOT(TransferDataFromSubWindow(CityWindow *)));
  }
  // Remove laser data from main window
//  ShowData(CityWindowPtr(), LaserData, false, LaserData, false, true, false);

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
bool City::SaveProject(bool ask_file_names, bool save_map,
                                   bool save_model)
{
  QString filename;

  if (save_map) {
    // Save map points
    if (!map_point_file || ask_file_names) {
      filename = QFileDialog::getSaveFileName(this, "Select map point file",
                                              map_point_file,
                                      "Map points (*.objpts);;All files (*.*)");
      if (filename.isEmpty()) return false;
      SetMapPointFile(filename.toAscii());
    }
    if (!map_points.Write(map_point_file)) return false;

    // Save map topology
    if (!map_top_file || ask_file_names) {
      filename = QFileDialog::getSaveFileName(this, "Select map topology file",
                                              map_top_file,
                                       "Map topology (*.top);;All files (*.*)");
      if (filename.isEmpty()) return false;
      SetMapTopologyFile(filename.toAscii());
    }
    if (!buildings.WriteMapData(map_top_file)) return false;
  }

  if (save_model) {
    // Save model points
    if (!model_point_file || ask_file_names) {
      filename = QFileDialog::getSaveFileName(this, "Select model point file",
                                              model_point_file,
             "Model points (*.objpts);;VRML model (*.wrl);;All files (*.*)");
      if (filename.isEmpty()) return false;
      if (filename.indexOf(".wrl", 0, Qt::CaseInsensitive) == -1)
        // No .wrl or .WRL file
        SetModelPointFile(filename.toAscii());
      else {
        //buildings.WriteVRML(filename.toAscii());
        return true;
      }
    }
    if (!model_points.Write(model_point_file)) return false;

    // Save model topology
    if (!model_top_file || ask_file_names) {
      filename = QFileDialog::getSaveFileName(this,
                                              "Select model topology file",
                                              model_top_file,
                                     "Model topology (*.top);;All files (*.*)");
      if (filename.isEmpty()) return false;
      SetModelTopologyFile(filename.toAscii());
    }
    if (!buildings.WriteModelData(model_top_file)) return false;
  }

  return true;
}

void City::FitPolygonalMapLine()
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
      showMessage("No laser points available for fitting circle.", 4000);
    return;
  }

  // Fit a polygon, add its points to map_points
  active_laser_points->VerifyTIN();
  active_laser_points->DeriveDataBounds(0);
  /*
  if (!active_laser_points->
  // To be changed for multiple polygons
   
    EnclosingPolygon(map_points, polygon,
                     segmentation_parameters->SegmentationParametersRef(),
                     outlining_parameters->OutliningParametersRef(), true))
                     
    return;
  */
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
    ShowData(CityWindowPtr(), MapPartitionData, true, SelectedMapPartitionData,
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
    ShowData(CityWindowPtr(), MapData, true, SelectedMapData, true, true, true);
  }
}

void City::FitCircularMapLine()
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
    ShowData(CityWindowPtr(), MapPartitionData, true, SelectedMapPartitionData,
             true, true,true);
  }

  // Otherwise, create a new building
  else {
    if (buildings.empty()) building_number = 1;
    else building_number = (buildings.end()-1)->Number() + 1;
    buildings.push_back(Building(building_number, &map_points, polygon));
    ShowData(CityWindowPtr(), MapData, true, SelectedMapData, true, true, true);
  }
}

void City::SpawnWindow()
{
  CityWindow *window;
  int       data_type;

  // Create a new window
  window = new CityWindow(CityView, NULL, Qt::Window);
// TODO: Check if the destructor is called for CityWindow
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
  connect(window, SIGNAL(WindowWillBeClosed(CityWindow *)),
          this, SLOT(RemoveCityWindow(CityWindow *)));
  connect(window, SIGNAL(RequestMoveToMainWindow(CityWindow *)),
          this, SLOT(TransferDataFromSubWindow(CityWindow *)));
}

void City::TransferDataFromSubWindow(CityWindow *window)
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
  ShowData(CityWindowPtr(), MapData, window->DataIsShown(MapData),
           SelectedMapData, false, false, true);
  ShowData(CityWindowPtr(), MapPartitionData,
           window->DataIsShown(MapPartitionData), SelectedMapPartitionData,
           false, false, true);
  ShowData(CityWindowPtr(), ModelData, window->DataIsShown(ModelData),
           SelectedModelData, false, false, true);
  ShowData(CityWindowPtr(), LaserData, window->DataIsShown(LaserData),
           LaserData, false, false, true);
  ShowData(CityWindowPtr(), LastModelData, window->DataIsShown(LastModelData),
           LastModelData, false, false, true);

  // Copy transformation parameters

  // Refresh canvas
  canvas->update();

  // Close sub window
  window->close();
}

void CityWindow::ClearSelections(DataType type)
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

void City::MoveMapPoint(FeatureWindowType type)
{
     
switch(type){
  case Wall: outline_obj=wall_win->map_points;break;   
  case Extrusion: cwindow_objpts=extrusion_win->map_points;break;           
  case RoofExtrusion: roof_extru_obj=roof_extru_win->map_points;break;    
  case Window: window_obj=window_win->map_points;break;           
  case Door: door_obj=door_win->map_points;break;             
             }
}

void City::RemoveVertexPoint(FeatureWindowType type, int vertex_number)
{
   switch(type){
   case Wall: 
        RemovePoint(&outline_obj, &outline_tops, vertex_number);
        wall_win->map_points=outline_obj;
        break;   
  
   }  
}

void City::AddVertexPoint(FeatureWindowType type, int vertex_number)
{
   switch(type){
   case Wall:  
        AddPoint(&outline_obj, &outline_tops, vertex_number);
        wall_win->map_points=outline_obj;
        break;   
  
   }  
}

void City::AddPoint(ObjectPoints *objpts, LineTopologies *tops, int vertex_number)
{
     LineTopologies::iterator line_maptop;
     LineTopology::iterator map_node;
     ObjectPoints::iterator point;
     double z;
     
  ObjectPoint new_point;   
  new_point=objpts->PointByNumber(vertex_number);
  z=new_point.GetZ();
  new_point.SetZ(z+0.1);
  new_point.Number()=objpts->MaxPointNumber()+1;
  objpts->push_back(new_point);
  
  
  for(line_maptop=tops->begin();line_maptop!=tops->end();line_maptop++)
     for(map_node=line_maptop->begin();map_node!=line_maptop->end();map_node++)
      {
      if(map_node->Number()==vertex_number)
        {line_maptop->insert(map_node, PointNumber(new_point.Number()));return;}
      }
      
}


void City::RemovePoint(ObjectPoints *objpts, LineTopologies *tops, int vertex_number)
{
     LineTopologies::iterator line_maptop;
     LineTopology::iterator map_node;
     ObjectPoints::iterator point;
     
  for(line_maptop=tops->begin();line_maptop!=tops->end();line_maptop++)
     for(map_node=line_maptop->begin();map_node!=line_maptop->end();map_node++)
      {
      if(map_node->Number()==vertex_number)
        {line_maptop->erase(map_node);break;}
      }
      
  for(point=objpts->begin();point!=objpts->end();point++)
     {
     if(point->Number()==vertex_number)
        {objpts->erase(point);break;}
     }   
}
