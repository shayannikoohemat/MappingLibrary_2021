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
#include <math.h>
#include "PointCloudMapper.h"
#include <QMessageBox>
#include <QStatusBar>
#include "BNF_io.h"
#include "LaserOctree.h"
#include "KNNFinder.h"
#include <QProgressDialog>
#include <QInputDialog>
#include <algorithm>

int num_planar_segments;

/*
--------------------------------------------------------------------------------
                           Segment the laser data
--------------------------------------------------------------------------------
*/

void PointCloudMapper::SegmentLaserData()
{
  TINEdges     *edges;
  ColourMethod colourmethod;

  // Check for data
  if (laser_points.empty()) {
    QMessageBox::information(this, "Error", "No laser data available.");
    return;
  }
  
  // TEMP: Store highest segment number in num_planar_segments
  num_planar_segments = 0;
  if (!segmentation_parameters->EraseOldLabels()) {
  	int min_segm_num;
  	if (!laser_points.AttributeRange(SegmentNumberTag, min_segm_num, num_planar_segments))
  	  num_planar_segments = 0;
  }
  // END TEMP
  
  // Derive the edges that define the neighbour relations
  edges = laser_points.DeriveEdges(*segmentation_parameters);
  
  // Remove long edges
  if (segmentation_parameters->MaxDistanceInComponent() > 0.0)
    laser_points.RemoveLongEdges(edges->TINEdgesRef(), 
                       segmentation_parameters->MaxDistanceInComponent(),
                       segmentation_parameters->DistanceMetricDimension() == 2);

  // Label the connected components
  laser_points.LabelComponents(edges->TINEdgesRef(), 
                               segmentation_parameters->ComponentAttribute(),
							   segmentation_parameters->EraseOldLabels());

  // Delete the edges
  laser_points.EraseNeighbourhoodEdges();

  // Remove labels of small components
  if (segmentation_parameters->MinNumberOfPointsComponent() > 1)
    laser_points.UnlabelSmallSegments(segmentation_parameters->ComponentAttribute(),
	                                  segmentation_parameters->MinNumberOfPointsComponent());
  
  // Change laser data appearance styles
  colourmethod = TagToColourMethod(segmentation_parameters->ComponentAttribute());
  appearance[LaserData]->SetPointColourMethod(colourmethod);
  appearance[SelectedLaserData]->SetPointColourMethod(colourmethod);
  appearance[LaserData]->Update();
  appearance[SelectedLaserData]->Update();

  // Display the segmented laser data
  ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, false, true,
           true);
}

void PointCloudMapper::GrowSurfaces()
{
  int point_colour_method, lowest_number, highest_number;
  
  // Check for data
  if (laser_points.empty()) {
    QMessageBox::information(this, "Error", "No laser data available.");
    return;
  }

  // Apply the surface growing, do not delete neighbourhood edges
  laser_points.SurfaceGrowing(*segmentation_parameters, false);

  // Set the highest segment number
  laser_points.AttributeRange(segmentation_parameters->SegmentAttribute(),
                              lowest_number, highest_number);
  laser_points.SetHighestSurfaceNumber(highest_number);
  
  // Change laser data appearance styles
  point_colour_method = TagToColourMethod(segmentation_parameters->SegmentAttribute());
  appearance[LaserData]->SetPointColourMethod(point_colour_method);
  appearance[SelectedLaserData]->SetPointColourMethod(point_colour_method);
  appearance[LaserData]->Update();
  appearance[SelectedLaserData]->Update();

  // Display the segmented laser data
  ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, false, true,
           true);
}

void PointCloudMapper::UnifySegmentNumberSelectedLaserData()
{
  LaserPoints::iterator point;
  int                   segment_number, i;
  
  // Check if there is selected laser data
  if (selected_laser_points.empty()) {
    QMessageBox::information(this, "Error",
                             "No selected laser data available.\n");
    return;
  }

  QProgressDialog dialog("Unifying segment numbers in selection", "Cancel",
                         0, laser_points.size(), this);
  // Get the segment number of a point in the selection
  for (point=selected_laser_points.begin(), segment_number=-1;
       point!=selected_laser_points.end() && segment_number == -1; point++)
    if (point->HasAttribute(SegmentNumberTag))
      segment_number = point->Attribute(SegmentNumberTag);
    
  // Assign this segment number to points that are also in the selection
  for (point=laser_points.begin(), i=0; point!=laser_points.end(); point++, i++) {
    if (selected_laser_points.Contains(point->LaserPointRef()))
      point->SetAttribute(SegmentNumberTag, segment_number);
    dialog.setValue(i);
    if (dialog.wasCanceled()) break;
  }
  dialog.setValue(laser_points.size());
  
  // Relabel all points in the selection
  selected_laser_points.SetAttribute(SegmentNumberTag, segment_number);

  // Display the data with the new labels
  ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, true, false,
           true);
}

void PointCloudMapper::RemoveSmallSegments()
{
  // Check if there is laser data
  if (laser_points.empty()) {
    QMessageBox::information(this, "Error",
                             "No laser data available.\n");
    return;
  }

  // Check on availability of segmentation
  if (!laser_points.HasAttribute(segmentation_parameters->ComponentAttribute())) {
    QMessageBox::information(this, "Error",
                             "The laser data has not been segmented.");
    return;
  }

  laser_points.RemoveSmallSegments(segmentation_parameters->ComponentAttribute(),
	                               segmentation_parameters->MinNumberOfPointsComponent());

  ShowData(PCMWindowPtr(), LaserData, !laser_points.empty(),
           SelectedLaserData, true, true, true);
}

void PointCloudMapper::UnlabelSmallSegments()
{
  // Check if there is laser data
  if (laser_points.empty()) {
    QMessageBox::information(this, "Error",
                             "No laser data available.\n");
    return;
  }

  // Check on availability of segmentation
  if (!laser_points.HasAttribute(segmentation_parameters->ComponentAttribute())) {
    QMessageBox::information(this, "Error",
                             "The laser data has not been segmented.");
    return;
  }

  laser_points.UnlabelSmallSegments(segmentation_parameters->ComponentAttribute(),
	                                segmentation_parameters->MinNumberOfPointsComponent());

  ShowData(PCMWindowPtr(), LaserData, !laser_points.empty(),
           SelectedLaserData, true, true, true);
}

void PointCloudMapper::RemoveSegmentsBasedOnPulseType(LaserPulseType pulse_type,
                                                    double max_pulse_type_ratio)
{
  LaserPoints::iterator point, good_point;
  int                   max_segment_number=0, *segment_number_count,
                        *segment_pulse_type_count, segment_number;
  
  // Check if there is laser data
  if (laser_points.empty()) {
    QMessageBox::information(this, "Error",
                             "No laser data available.");
    return;
  }
  
  // Check on availability of segmentation
  if (!laser_points.HasAttribute(SegmentNumberTag)) {
    QMessageBox::information(this, "Error",
                             "The laser data has not been segmented.");
    return;
  }

  // Determine the highest segment number
  for (point=laser_points.begin(); point!=laser_points.end(); point++) {
    if (point->HasAttribute(SegmentNumberTag))
      if (point->Attribute(SegmentNumberTag) > max_segment_number)
        max_segment_number = point->Attribute(SegmentNumberTag);
  }
  
  // Determine the segment sizes and the number of points with the pulse type
  segment_number_count = (int *) calloc(max_segment_number+1, sizeof(int));
  segment_pulse_type_count = (int *) calloc(max_segment_number+1, sizeof(int));
  for (point=laser_points.begin(); point!=laser_points.end(); point++) {
    if (point->HasAttribute(SegmentNumberTag)) {
      segment_number_count[point->Attribute(SegmentNumberTag)]++;
      if (point->IsPulseType(pulse_type))
        segment_pulse_type_count[point->Attribute(SegmentNumberTag)]++;
    }
  }
    
  // Remove points of small segments and of segments with a high percentage
  // of points with the specified pulse type
  for (point=laser_points.begin(), good_point=laser_points.begin();
       point!=laser_points.end(); point++) {
    if (point->HasAttribute(SegmentNumberTag)) {
      segment_number = point->Attribute(SegmentNumberTag);
      if (segment_number_count[segment_number] >=
          segmentation_parameters->MinNumberOfPointsComponent() &&
          (double) segment_pulse_type_count[segment_number] /
          (double) segment_number_count[segment_number] < max_pulse_type_ratio) {
        *good_point = *point;
        good_point++;
      }
    }
  }
  laser_points.erase(good_point, laser_points.end());
  
  // Also remove points of these segments in the selected data
  if (!selected_laser_points.empty()) {
    for (point=selected_laser_points.begin(),
         good_point=selected_laser_points.begin();
         point!=selected_laser_points.end();
         point++) {
      if (point->HasAttribute(SegmentNumberTag)) {
        segment_number = point->Attribute(SegmentNumberTag);
        if (segment_number_count[segment_number] >=
            segmentation_parameters->MinNumberOfPointsComponent() &&
            (double) segment_pulse_type_count[segment_number] /
            (double) segment_number_count[segment_number] < max_pulse_type_ratio) {
          *good_point = *point;
          good_point++;
        }
      }
    }
    selected_laser_points.erase(good_point, selected_laser_points.end());
  }
  free(segment_number_count);
  free(segment_pulse_type_count);

  ShowData(PCMWindowPtr(), LaserData, !laser_points.empty(),
           SelectedLaserData, true, true, true);
}

// Change the label of the selected laser points
void PointCloudMapper::ModifyLabel(int new_label)
{
  LaserPoints::iterator point;

  // Change attribute of selected points
  selected_laser_points.SetAttribute(LabelTag, new_label);
  for (point=laser_points.begin(); point!=laser_points.end(); point++)
    if (point->HasAttribute(IsSelectedTag))
      if (point->Attribute(IsSelectedTag))
        point->SetAttribute(LabelTag, new_label);

  // Refresh display
  ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, true,
           true, true);
}

void PointCloudMapper::ModifyLaserAttribute(const LaserPointTag tag)
{ 
  LaserPoints::iterator point;
  int                   new_value;
  bool                  ok;

  // Check if there are points   
  if (selected_laser_points.empty()) {
    QMessageBox::information(this, "Error",
                             "No laser points selected.\n");
    return;
  }

  // Ask for the new value
  switch (tag) {
    case LabelTag:
      new_value = QInputDialog::getInt(this, "Modify label", "New label:",
                                       default_laser_attributes.Attribute(tag),
                                       0, 2147483647, 1, &ok);
      break;
    case IsFilteredTag:
      new_value = QInputDialog::getInt(this, "Modify filter status",
                                       "New status:",
                                       default_laser_attributes.Attribute(tag),
                                       0, 1, 1, &ok);
      break;
    case PlaneNumberTag:
      new_value = QInputDialog::getInt(this, "Modify plane number",
                                       "New plane number:",
                                       default_laser_attributes.Attribute(tag),
                                       0, 2147483647, 1, &ok);
      break;
    case SegmentNumberTag:
      new_value = QInputDialog::getInt(this, "Modify segment number",
                                       "New segment number:",
                                       default_laser_attributes.Attribute(tag),
                                       0, 2147483647, 1, &ok);
      break;
    case ScanNumberTag:
      new_value = QInputDialog::getInt(this, "Modify scan number",
                                       "New scan number:",
                                       default_laser_attributes.Attribute(tag),
                                       0, 2147483647, 1, &ok);
      break;
    default:
      printf("Invalid attribute, no editing foreseen\n");
  }
                                       
  if (!ok) return;  // User selected cancel button                                     
                                       
  // Store value as new default
  default_laser_attributes.Attribute(tag) = new_value;
  
  // Change attribute of selected points
  selected_laser_points.SetAttribute(tag, new_value);
  for (point=laser_points.begin(); point!=laser_points.end(); point++)
    if (point->HasAttribute(IsSelectedTag))
      if (point->Attribute(IsSelectedTag))
        point->SetAttribute(tag, new_value);

  // Refresh display
  ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, true,
           true, true);
}

void PointCloudMapper::FilterLaserData()
{
  Image kernel;
  TINEdges edges;
  
  // Create a filter kernel based on the specified parameters
  kernel.CreateFilterKernel(filtering_parameters->HeightDifAtSameLocation(),
                            -1.0, 0.0, -1.0, 0.0, -1.0, 0.0,
                            -1.0, 0.0, -1.0, 0.0,
                            filtering_parameters->FilterRadius(),
                            filtering_parameters->HeightDifAtFilterRadius(),
                            filtering_parameters->FilterRadius() / 1000.0);

  // Verify TIN and recreate edges
  laser_points.VerifyTIN();
  laser_points.EraseNeighbourhoodEdges();
  edges.Derive(laser_points.TINReference());
  laser_points.SetNeighbourhoodEdges(&edges);
  
  // Filter
  laser_points.FilterOnMorphology(kernel, filtering_parameters->FilterRadius(),
                                  0.0, 0.0, edges);
  
  if (filtering_parameters->RemoveNonGroundPoints()) {
    laser_points.RemoveTaggedPoints(1, IsFilteredTag);            
  }
  else {
    // Change laser data appearance styles
    appearance[LaserData]->SetPointColourMethod(ColourByFilterStatus);
    appearance[SelectedLaserData]->SetPointColourMethod(ColourByFilterStatus);
    appearance[LaserData]->Update();
    appearance[SelectedLaserData]->Update();
  }

  // Display the segmented laser data
  ShowData(PCMWindowPtr(), LaserData, true, SelectedLaserData, false, true,
           true);
}

void PointCloudMapper::MeanShiftSegmentation()
{
  const unsigned char *attribute_tags, *tag;
  int                 itag, num_tags;

  // Check if all selected attributes are available
  num_tags = segmentation_parameters->AttributeBandWidths().NumAttributes();
  attribute_tags = segmentation_parameters->AttributeBandWidths().AttributeTags();
  for (itag=0, tag=attribute_tags; itag<num_tags; itag++, tag++) {
    if ((LaserPointTag) *tag >= NoTag) continue;
    // Try to calculate an attribute if it's not available
    if (!laser_points.HasAttribute((LaserPointTag) *tag))
      laser_points.DerivePointAttribute((LaserPointTag) *tag,
	                                    *segmentation_parameters);
	// Return if the attribute could not be calculated
	if (!laser_points.HasAttribute((LaserPointTag) *tag)) {
	  QMessageBox::information(this, "Error",
                               QString("Attribute ") +
							   QString(AttributeName((LaserPointTag) *tag, false)) +
							   QString("is not available"));
      return;
	}
  }

  // Mean shift segmentation
  printf("Starting mean shift\n");
  laser_points.MeanShift(*segmentation_parameters, 0, true);
}

void PointCloudMapper::SegmentGrowing()
{
  const unsigned char *tolerance_tags, *tag;
  int                 itag, num_tags;

  // Check if all selected attributes are available
  num_tags = segmentation_parameters->GrowingTolerances().NumAttributes();
  tolerance_tags = segmentation_parameters->GrowingTolerances().AttributeTags();
  for (itag=0, tag=tolerance_tags; itag<num_tags; itag++, tag++) {
    if ((LaserPointTag) *tag >= NoTag) continue;
    // Try to calculate an attribute if it's not available
    if (!laser_points.HasAttribute((LaserPointTag) *tag))
      laser_points.DerivePointAttribute((LaserPointTag) *tag,
	                                    *segmentation_parameters);
	// Return if the attribute could not be calculated
	if (!laser_points.HasAttribute((LaserPointTag) *tag)) {
	  QMessageBox::information(this, "Error",
                               QString("Attribute ") +
							   QString(AttributeName((LaserPointTag) *tag, false)) +
							   QString("is not available"));
      return;
	}
  }

  // Segment growing
  printf("Start segment growing\n");
  laser_points.SegmentGrowing(*segmentation_parameters, 0, true, true);
  printf("Done\n");
}

void PointCloudMapper::MergeSurfaces()
{
  laser_points.MergeSurfaces(*segmentation_parameters);
}

void PointCloudMapper::MajorityFilterSegmentation()
{
  TINEdges *edges;
  
  // Verify edges
  edges = laser_points.VerifyEdges(*segmentation_parameters);

  // Majority filtering
  printf("Start majority filtering\n");
  laser_points.MajorityFilter(*segmentation_parameters, edges->TINEdgesRef());
  printf("Done\n");
}
