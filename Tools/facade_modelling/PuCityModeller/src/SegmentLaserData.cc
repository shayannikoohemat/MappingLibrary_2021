
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
#include <QStatusBar>
#include "BNF_io.h"
#include "LaserOctree.h"
#include "KNNFinder.h"
#include <QProgressDialog>

/*
--------------------------------------------------------------------------------
                           Segment the laser data
--------------------------------------------------------------------------------
*/

void City::SegmentLaserData()
{
  TINEdges    *edges;

  // Check for data
  if (laser_points.empty()) {
    QMessageBox::information(this, "Error", "No laser data available.\n");
    return;
  }

  // Derive the edges that define the neighbour relations
  edges = laser_points.DeriveEdges(*segmentation_parameters);
  
  // Remove long edges
  if (segmentation_parameters->MaxDistanceInComponent() > 0.0)
    laser_points.RemoveLongEdges(edges->TINEdgesRef(), 
                       segmentation_parameters->MaxDistanceInComponent(),
                       segmentation_parameters->DistanceMetricDimension() == 2);

  // Label the connected components
  laser_points.LabelComponents(edges->TINEdgesRef(), SegmentNumberTag);

  // Delete the edges
  delete edges;

  // Remove small components
  if (segmentation_parameters->MinNumberOfPointsComponent() > 1)
    RemoveSmallSegments();
  
  // Change laser data appearance styles
  appearance[LaserData]->SetPointColourMethod(ColourBySegment);
  appearance[SelectedLaserData]->SetPointColourMethod(ColourBySegment);

  // Display the segmented laser data
  ShowData(CityWindowPtr(), LaserData, true, SelectedLaserData, false, true,
           true);
}

void City::GrowSurfaces()
{
  // Check for data
  if (laser_points.empty()) {
    QMessageBox::information(this, "Error", "No laser data available.\n");
    return;
  }

  // Apply the surface growing
  laser_points.SurfaceGrowing(*segmentation_parameters);

  // Remove small components
//  if (segmentation_parameters->MinNumberOfPointsComponent() > 1)
//    RemoveSmallSegments();
  
  // Change laser data appearance styles
  appearance[LaserData]->SetPointColourMethod(ColourBySegment);
  appearance[SelectedLaserData]->SetPointColourMethod(ColourBySegment);
  
   vector<int> indices=laser_points.GetAttribute(SegmentNumberTag);
    max_segment_value=0;  
   for(int i=0;i<indices.size();i++)
      max_segment_value=(max_segment_value>indices[i])?max_segment_value:indices[i];          
  
  
  // Display the segmented laser data
  ShowData(CityWindowPtr(), LaserData, true, SelectedLaserData, false, true,
           true);
}

void CityWindow::UnifySegmentNumberSelectedLaserData()
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
  
  
  emit LaserChanged(feature_type); 
  // Display the data with the new labels
  ShowData(CityWindowPtr(), LaserData, true, SelectedLaserData, true, false,
           true);
           
           
}

void City::RemoveSmallSegments()
{
  LaserPoints::iterator point, good_point;
  int                   max_segment_number=0, *segment_number_count;
  
  // Check if there is laser data
  if (laser_points.empty()) {
    QMessageBox::information(this, "Error",
                             "No laser data available.\n");
    return;
  }

  // Determine the highest segment number
  for (point=laser_points.begin(); point!=laser_points.end(); point++) {
    if (point->HasAttribute(SegmentNumberTag))
      if (point->Attribute(SegmentNumberTag) > max_segment_number)
        max_segment_number = point->Attribute(SegmentNumberTag);
  }
  
  // Count the segment numbers
  segment_number_count = (int *) calloc(max_segment_number+1, sizeof(int));
  for (point=laser_points.begin(); point!=laser_points.end(); point++)
    if (point->HasAttribute(SegmentNumberTag))
      segment_number_count[point->Attribute(SegmentNumberTag)]++;
    
  // Remove points of small segments
  for (point=laser_points.begin(), good_point=laser_points.begin();
       point!=laser_points.end(); point++) {
    if (point->HasAttribute(SegmentNumberTag)) {
      if (segment_number_count[point->Attribute(SegmentNumberTag)] >=
          segmentation_parameters->MinNumberOfPointsComponent()) {
        *good_point = *point;
        good_point++;
      }
    }
  }
  laser_points.erase(good_point, laser_points.end());
  
  // Also remove points of small segments in the selected data
  if (!selected_laser_points.empty()) {
    for (point=selected_laser_points.begin(),
         good_point=selected_laser_points.begin();
         point!=selected_laser_points.end();
         point++) {
      if (point->HasAttribute(SegmentNumberTag)) {
        if (segment_number_count[point->Attribute(SegmentNumberTag)] >=
            segmentation_parameters->MinNumberOfPointsComponent()) {
          *good_point = *point;
          good_point++;
        }
      }
    }
    selected_laser_points.erase(good_point, selected_laser_points.end());
  }
  free(segment_number_count);

  ShowData(CityWindowPtr(), LaserData, !laser_points.empty(),
           SelectedLaserData, true, true, true);
}

void City::SetLaserAttribute(const LaserPointTag tag)
{ 
  LaserPoints::iterator point;
   
  if (selected_laser_points.empty()) {
    QMessageBox::information(this, "Error",
                             "No laser points selected.\n");
    return;
  }

  selected_laser_points.SetAttribute(tag,
                                     default_laser_attributes->Attribute(tag));
  for (point=laser_points.begin(); point!=laser_points.end(); point++)
    if (point->HasAttribute(IsSelectedTag))
      point->SetAttribute(tag, default_laser_attributes->Attribute(tag));

  ShowData(CityWindowPtr(), LaserData, true, SelectedLaserData, true,
           true, true);
}
