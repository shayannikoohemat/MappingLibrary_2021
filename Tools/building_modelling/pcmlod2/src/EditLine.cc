
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

void PointCloudMapper::MergeMapLines()
{
  LineTopsIterVector::iterator line1, line2, lineswap;
  Building                     *building;

  if (selected_map_data.size()) {
    if (selected_map_data.size() != 2) {
      QMessageBox::information(this, "Error",
                              "Two map polygons should be selected at a time.");
      return;
    }
    line1 = selected_map_data.begin();
    line2 = line1 + 1;
    if ((*line1)->Number() > (*line2)->Number())
      { lineswap = line1; line1 = line2; line2 = lineswap; }
    building = buildings.BuildingPtr((*line1)->Attribute(BuildingNumberTag));
    buildings.MergeBuildings((*line1)->Attribute(BuildingNumberTag),
                             (*line2)->Attribute(BuildingNumberTag));
    selected_map_data.Clear(); // Clear old selection
    nearby_map_data.Clear();
    // Put merged line in the selection
    selected_map_data.push_back(building->MapDataPtr()->begin());
    ShowData(PCMWindowPtr(), MapData, true, SelectedMapData, true, true, true);
  }
  else if (selected_map_part_data.size()) {
    if (selected_map_part_data.size() != 2) {
      QMessageBox::information(this, "Error",
                            "Two map partitions should be selected at a time.");
      return;
    }
    line1 = selected_map_part_data.begin();
    line2 = line1 + 1;
    if (*line1 > *line2) { lineswap = line1; line1 = line2; line2 = lineswap; }
    building = buildings.BuildingPtr((*line1)->Attribute(BuildingNumberTag));
    building->MergePartitions(*line1, *line2);
    // Remove the second line from the selection
    selected_map_part_data.erase(line2);
    nearby_map_part_data.Clear();
    ShowData(PCMWindowPtr(), MapPartitionData, true, SelectedMapPartitionData,
             true, true,true);
  }
  else {
    QMessageBox::information(this, "Error",
                             "No map (partition) lines selected.");
    return;
  }
}

void PointCloudMapper::CropLines()
{
  LineTopsIterVector           *selected_data, *nearby_data;
  LineTopsIterVector::iterator line;
  DataType                     data_type, selected_data_type;
  LineTopologies               selected_lines;
  Buildings::iterator          building;
  bool                         show_base_type;

  // Determine the selected data
  if (selected_map_data.size()) {
    selected_data      = &selected_map_data;
    data_type          = MapData;
    selected_data_type = SelectedMapData;
    nearby_data        = &nearby_map_data;
  }
  else if (selected_map_part_data.size()) {
    selected_data      = &selected_map_part_data;
    data_type          = MapPartitionData;
    selected_data_type = SelectedMapPartitionData;
    nearby_data        = &nearby_map_part_data;
  }
  else if (selected_model_data.size()) {
    selected_data      = &selected_model_data;
    data_type          = ModelData;
    selected_data_type = SelectedModelData;
    nearby_data        = &nearby_model_data;
  }
  else if (selected_model_part_data.size()) {
    selected_data      = &selected_model_part_data;
    data_type          = ModelData;
    selected_data_type = SelectedModelPartData;
    nearby_data        = &nearby_model_part_data;
  }
  else if (selected_model_face_data.size()) {
    selected_data      = &selected_model_face_data;
    data_type          = LastModelData;
    selected_data_type = SelectedModelFaceData;
    nearby_data        = &nearby_model_face_data;
  }
  else {
    QMessageBox::information(this, "Error",
                             "No map (partition) line selected.");
    return;
  }
  
  // Make a local copy of the lines
  for (line=selected_data->begin(); line!=selected_data->end(); line++)
    selected_lines.push_back(**line);

  // Crop to the selected lines
  if (data_type != LastModelData) // Crop data in buildings
    for (building=buildings.begin(); building!=buildings.end(); building++)
      building->CropPolygons(selected_lines, data_type);
  else // Crop data in last reconstructed model
    last_model_part.CropPolygons(selected_lines, ModelData);
    
  // Clear selected data
  selected_data->erase(selected_data->begin(), selected_data->end());
  nearby_data->erase(nearby_data->begin(), nearby_data->end());
  selected_lines.erase(selected_lines.begin(), selected_lines.end());
  if (data_type == LastModelData)
    show_base_type = last_model_part.ContainsData(ModelData);
  else
    show_base_type = buildings.ContainsData(data_type);
  ShowData(PCMWindowPtr(), data_type, show_base_type,
           selected_data_type, false, true, true);
  // Update result view canvas by copying modified data
  if (data_type == LastModelData) UpdateLastModelDataInResultWindow();
}


void PointCloudMapper::DeleteLine()
{
  LineTopsIterVector           *selected_data, *nearby_data;
  DataType                     data_type, selected_data_type;
  LineTopsIterVector::iterator line;
  Building                     *building;
  LineTopologies               one_line_set, selected_lines;
  LineTopologies::iterator     selected_line;
  bool                         show_base_type;

  if (selected_map_data.size()) {
    selected_data      = &selected_map_data;
    data_type          = MapData;
    selected_data_type = SelectedMapData;
    nearby_data        = &nearby_map_data;
  }
  else if (selected_map_part_data.size()) {
    selected_data      = &selected_map_part_data;
    data_type          = MapPartitionData;
    selected_data_type = SelectedMapPartitionData;
    nearby_data        = &nearby_map_part_data;
  }
  else if (selected_model_data.size()) {
    selected_data      = &selected_model_data;
    data_type          = ModelData;
    selected_data_type = SelectedModelData;
    nearby_data        = &nearby_model_data;
  }
  else if (selected_model_part_data.size()) {
    selected_data      = &selected_model_part_data;
    data_type          = ModelData;
    selected_data_type = SelectedModelPartData;
    nearby_data        = &nearby_model_part_data;
  }
  else if (selected_model_face_data.size()) {
    selected_data      = &selected_model_face_data;
    data_type          = LastModelData;
    selected_data_type = SelectedModelFaceData;
    nearby_data        = &nearby_model_face_data;
  }
  else {
    QMessageBox::information(this, "Error",
                             "No map (partition) line selected.");
    return;
  }
  
  // Make a local copy of the selected lines (needed because deleting them
  // will change the iterators if multiple lines are selected from one
  // LineTopologies set)
  for (line=selected_data->begin(); line!=selected_data->end(); line++)
    selected_lines.push_back(**line);

  // Delete the lines one by one, as they may belong to different buildings
  for (selected_line=selected_lines.begin();
       selected_line!=selected_lines.end(); selected_line++) {
    // Create a vector with one selected line
    one_line_set.push_back(*selected_line);
    if (data_type != LastModelData) {
      // Retrieve the building
      building = buildings.BuildingPtr(selected_line->Attribute(BuildingNumberTag));
      // Remove the line from the building
      if (!building->DeletePolygons(one_line_set, data_type)) {
        printf("Line %d was NOT deleted!\n", selected_line->Number());
        printf("Building number %d\n", building->Number());
      }
      // Delete the building if it does no longer contain data
      if (!building->ContainsData())
        buildings.erase(buildings.BuildingIterator(building->Number()));
    }
    else { // Last model data
      if (!last_model_part.DeletePolygons(one_line_set, data_type))
        printf("Line %d was NOT deleted from last model part!\n",
               selected_line->Number());
    }
    // Empty the vector with the selected line
    one_line_set.erase(one_line_set.begin());
  }

  // Clear selected data
  selected_data->erase(selected_data->begin(), selected_data->end());
  nearby_data->erase(nearby_data->begin(), nearby_data->end());
  selected_lines.erase(selected_lines.begin(), selected_lines.end());
  // Update main canvas
  if (data_type == LastModelData)
    show_base_type = last_model_part.ContainsData(ModelData);
  else
    show_base_type = buildings.ContainsData(data_type);
  ShowData(PCMWindowPtr(), data_type, show_base_type,
           selected_data_type, false, true, true);
  // Update result view canvas by copying modified data
  if (data_type == LastModelData) UpdateLastModelDataInResultWindow();
}

void PointCloudMapper::DeleteLastEdge()
{
  LineTopsIterVector::iterator line;

  // Get the selected line
  if (selected_map_data.size())
    line = selected_map_data.begin();
  else if (selected_map_part_data.size())
    line = selected_map_part_data.begin();
  else if (selected_model_face_data.size())
    line = selected_model_face_data.begin();
  else {
    QMessageBox::information(this, "Error",
                             "No map (partition) line selected.");
    return;
  }

  // Switch off the extension line
  if (Canvas()->Mode() == ExtendLineMode)
    Canvas()->RemoveExtensionLine(false);

  // Call DeleteLine if there's only one point left in the line
  if ((*line)->size() == 1) {
    DeleteLine();
    // Change mode if the current mode was line extension
    if (Canvas()->Mode() == ExtendLineMode) {
      if (selected_map_data.size()) SetMode(SelectMapMode);
      else if (selected_map_part_data.size()) SetMode(SelectMapPartitionMode);
      else SetMode(SelectModelFaceMode);
    }
  }

  // Otherwise remove the last node
  else {
    (*line)->erase((*line)->end()-1);
    // Switch the extension line on again
    if (Canvas()->Mode() != ExtendLineMode) SetMode(ExtendLineMode);
    else Canvas()->InitialiseExtensionLine(&map_points, line);
  }
  Canvas()->update();

  // Update result view canvas by copying modified data
  if (selected_model_face_data.size())
    if (line == selected_model_face_data.begin())
      UpdateLastModelDataInResultWindow();
}

void PointCloudMapper::ReverseLine()
{
  LineTopologies::iterator line;

  // Get the selected line
  if (selected_map_data.size())
    line = *(selected_map_data.begin());
  else if (selected_map_part_data.size())
    line = *(selected_map_part_data.begin());
  else if (selected_model_face_data.size())
    line = *(selected_model_face_data.begin());
  else {
    QMessageBox::information(this, "Error",
                             "No map (partition) line or model face selected.");
    return;
  }

  // Reverse the node order
  line->RevertNodeOrder();
  Canvas()->update(); // Not needed now, but later for first point display
  // Update result view canvas by copying modified data
  if (selected_model_face_data.size())
    if (line == *(selected_model_face_data.begin()))
      UpdateLastModelDataInResultWindow();
}

void PointCloudMapper::CreateNewLine()
{
  LineTopologies::iterator map_line, new_selection;
  Buildings::iterator      building;
  LineTopology             new_line;

  // Create a new building model face
  if (edit_model_window != NULL && edit_model_action->isChecked()) {
    new_line.Label() = RoofLabel;
    last_model_part.AddModelData(new_line);
    // Select map line
    new_selection = last_model_part.RoofFaces()->end()-1;
    nearby_model_face_data.Clear();
    nearby_model_face_data.push_back(new_selection);
    if (selected_model_face_data.size()) {
      selected_model_face_data.Clear();
      Canvas()->RemoveObjectData(SelectedModelFaceData, false);
    }
    selected_model_face_data.push_back(new_selection);
    ShowData(PCMWindowPtr(), LastModelData, true, SelectedModelFaceData, true,
             true, true);
    UpdateLastModelDataInResultWindow();
  }
  
  // Create new partition outline inside the selected building
  else if (selected_map_data.size()) {
    map_line = *(selected_map_data.begin());
    building = buildings.BuildingIterator(map_line->Attribute(BuildingNumberTag));
    new_line = LineTopology(map_line->Number(), MapPartitionLabel);
    building->AddMapPartitionData(new_line);
    // Unselect map line
    nearby_map_data.Clear();
    selected_map_data.Clear();
    // Select map partition line
    new_selection = (building->MapPartitionDataPtr()->end())-1;
    nearby_map_part_data.push_back(new_selection);
    selected_map_part_data.push_back(new_selection);
    // Refresh display
    ShowData(PCMWindowPtr(), MapData, true, SelectedMapData, false,
             false, true);
    ShowData(PCMWindowPtr(), MapPartitionData, true, SelectedMapPartitionData,
             true, true, true);
  }

  // Create new building outline
  else {
    buildings.push_back(Building(buildings.NextNumber(), &map_points,
                                 LineTopology()));
    // Select map line
    new_selection = (buildings.end()-1)->MapDataPtr()->end()-1;
    nearby_map_data.Clear();
    nearby_map_data.push_back(new_selection);
    selected_map_data.push_back(new_selection);
    ShowData(PCMWindowPtr(), MapData, true, SelectedMapData, true, true, true);
  }

  // Switch to extend line mode
  SetMode(ExtendLineMode);
}

void PointCloudMapper::CloseLine()
{
  LineTopologies::iterator line;

  // Get the selected line
  if (selected_map_data.size())
    line = *(selected_map_data.begin());
  else if (selected_map_part_data.size())
    line = *(selected_map_part_data.begin());
  else if (selected_model_face_data.size())
    line = *(selected_model_face_data.begin());
  else {
    QMessageBox::information(this, "Error",
                             "No map (partition) line or model face selected.");
    return;
  }

  // Check if the line can be closed
  if (line->IsClosed()) {
    QMessageBox::information(this, "Error",
                             "The selected line is already closed.\n");
    return;
  }
  if (line->size() <= 2) {
    QMessageBox::information(this, "Error",
                             "Insufficient nodes to create a closed line.\n");
    return;
  }

  // Close the line
  line->push_back(*line->begin());
  Canvas()->update();

  // Change the mode if we are currently in the ExtendLineMode
  if (Canvas()->Mode() == ExtendLineMode) {
    if (selected_map_data.size()) SetMode(SelectMapMode);
    else if (selected_map_part_data.size()) SetMode(SelectMapPartitionMode);
    else SetMode(SelectModelFaceMode);
  }

  // Update result view canvas by copying modified data
  if (selected_model_face_data.size())
    if (line == *(selected_model_face_data.begin()))
      UpdateLastModelDataInResultWindow();
}

void PointCloudMapper::SetNewStartNode()
{
  LineTopologies::iterator line;
  PointNumber              selected_number;
  int                      point_index;

  // Get the selected line
  if (selected_map_data.size())
    line = *(selected_map_data.begin());
  else if (selected_map_part_data.size())
    line = *(selected_map_part_data.begin());
  else if (selected_model_face_data.size())
    line = *(selected_model_face_data.begin());
  else {
    QMessageBox::information(this, "Error",
                             "No map (partition) line or model face selected.");
    return;
  }

  // Check if there is a selected point
  if (selected_point.size() == 0) {
    QMessageBox::information(this, "Error",
                             "No map (partition) point selected.");
    return;
  }
  // Check if the polygon is closed
  if (!line->IsClosed()) {
    QMessageBox::information(this, "Error",
                       "New start nodes can not be selected for open polygons");
    return;
  }

  // Get the selected point
  selected_number = selected_point.begin()->NumberRef();
  point_index = line->FindPoint(selected_number);
  if (point_index == -1) {
    QMessageBox::information(this, "Bug!",
                             "Selected point not part of selected line.");
    return;
  }
  else if (point_index == 0) {
    QMessageBox::information(this, "Error",
                             "Selected point already is the first line point.");
    return;
  }

  // Make the selected point the first point
  line->insert(line->end(), line->begin() + 1, line->begin() + point_index + 1);
  line->erase(line->begin(), line->begin() + point_index);
  Canvas()->update();
  // Update result view canvas by copying modified data
  if (selected_model_face_data.size())
    if (line == *(selected_model_face_data.begin()))
      UpdateLastModelDataInResultWindow();
}

void PointCloudMapper::SplitLineAtSelectedNode()
{
  LineTopologies::iterator line;
  PointNumber              selected_number;
  int                      point_index, building_number;
  LineTopology             new_line;
  Building                 *building;

  // Get the selected line
  if (selected_map_data.size())
    line = *(selected_map_data.begin());
  else if (selected_map_part_data.size())
    line = *(selected_map_part_data.begin());
  else {
    QMessageBox::information(this, "Error",
                             "No map (partition) line selected.");
    return;
  }

  // Check if there is a selected point
  if (selected_point.size() == 0) {
    QMessageBox::information(this, "Error",
                             "No map (partition) point selected.");
    return;
  }

  // Get the selected point
  selected_number = selected_point.begin()->NumberRef();
  point_index = line->FindPoint(selected_number);
  if (point_index == -1) {
    QMessageBox::information(this, "Bug!",
                             "Selected point not part of selected line.");
    return;
  }
  else if (point_index == 0 || point_index == (int) line->size() - 1) {
    QMessageBox::information(this, "Error",
                             "Line cannot be split at begin or end point.");
    return;
  }

  // Copy the second part to a new line
  new_line.insert(new_line.begin(), line->begin() + point_index, line->end());
  // Remove the second part from the selected line
  line->erase(line->begin() + point_index + 1, line->end());

  // Retrieve the building
  building = buildings.BuildingPtr(line->Attribute(BuildingNumberTag));

  // Create a new building for the second line
  if (selected_map_data.size()) {
    building_number = (buildings.end()-1)->Number() + 1;
    buildings.push_back(Building(building_number, &map_points, new_line));
    selected_map_data.Clear(); // Clear old selection
    nearby_map_data.Clear();
    ShowData(PCMWindowPtr(), MapData, true, SelectedMapData, false, true, true);
  }
  // Add a map partition line to the building
  else {
    building->AddMapPartitionData(new_line, false);
    selected_map_part_data.Clear(); // Clear old selection
    nearby_map_part_data.Clear();
    ShowData(PCMWindowPtr(), MapPartitionData, true, SelectedMapPartitionData,
             false,true,true);
  }
}
