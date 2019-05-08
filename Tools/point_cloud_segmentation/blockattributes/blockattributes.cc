/*
    Copyright 2013 University of Twente
 
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


/*
--------------------------------------------------------------------------------
 Derivation of segment attributes in a tiled block of laser data
 Segments may extend over multiple tiles.
  

 Initial creation:
 Author : George Vosselman
 Date   : 24-06-2013

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <malloc.h>
#include "LaserBlock.h"
#include "TINEdges.h"
#include "BNF_io.h"


// Derive segment attributes, special version of
// LaserPoints::DeriveSegmentAttributes to deal with small and large segments

void UpdateAttributeSums(vector<LaserPointTag> &segment_attributes,
                         LaserUnit::iterator tile,
                         vector <int> &tile_numbers, 
                         vector <vector <long long int> > &large_segments_in_block, 
	                     long long int *small_counts_in_tile, 
						 double *small_values_in_tile,
						 Plane *small_planes_in_tile,
						 vector <long long int *> &large_counts_in_block,
						 vector <double *> &large_values_in_block, 
						 vector <Plane *> &large_planes_in_block)
{
  LaserPointTag           point_attribute;
  vector<LaserPointTag>::iterator segment_attribute;
  LaserPointAttributeType attribute_type;
  int                     attribute_category, pulse_type, segment_index, iattr,
                          tile_number, tile_index, this_tile_index, 
						  point_index, num_attributes;
  long long int           segment, *count_ptr;
  double                  value, dist, *value_ptr, pi=4.0*atan(1.0);
  Plane                   *plane_ptr;
  LaserPoints::iterator   point, segment_point, neighbour_point;
  bool                    segment_is_large, plane_done, counting_done;
  vector< vector<long long int> >::iterator large_segments_in_tile_iterator;
  vector<int>::iterator   tile_number_iterator;
  vector<long long int>::iterator segment_iterator;
  vector<long long int>   processed_segments;
  LaserPoints             segment_points;
  TINEdges                *edges;
  TINEdges::iterator      neighbours;
  PointNumberList::iterator node; 
  SegmentationParameters  parameters;
  Vector3D                normal;
  
  // Loop over all segment attributes
  plane_done = counting_done = false;
  num_attributes = segment_attributes.size();
  for (iattr=0, segment_attribute=segment_attributes.begin();
       iattr<num_attributes; iattr++, segment_attribute++) {
  
    // Determine the category of attribute
    if (*segment_attribute < SlopeAngleVarianceTag) attribute_category = 1; // Average
    else if (*segment_attribute < MinReflectanceTag) attribute_category = 9; // Variance, needs special processing
    else if (*segment_attribute < MaxReflectanceTag) attribute_category = 2; // Minimum
    else if (*segment_attribute < EdgeCountTag) attribute_category = 3; // Maximum
    else if (*segment_attribute <= SegmentSizeTag) attribute_category = 4; // Segment or component size
    else if (*segment_attribute <= PercMultiPulseTag) attribute_category = 5; // Pulse type percentage
    else if (*segment_attribute < InclinationTag) attribute_category = 6; // Other percentage
    else attribute_category = 7; // Plane attribute
    if (*segment_attribute == AveragePointSpacingTag) attribute_category = 8; // Needs special processing
    if (*segment_attribute == EdgeCountTag) continue; // Only used for average point spacing calculations
    if (*segment_attribute == Sum1Tag) continue; // Only used for variance calculations
    if (*segment_attribute == Sum2Tag) continue; // Only used for variance calculations
  
    // Pulse types
    if (attribute_category == 5) {
      switch (*segment_attribute) {
        default:
        case PercFirstPulseTag:    pulse_type = FirstPulse; break;
        case PercSecondPulseTag:   pulse_type = SecondPulse; break;
        case PercThirdPulseTag:    pulse_type = ThirdPulse; break;
        case PercFourthPulseTag:   pulse_type = FourthPulse; break;
        case PercLastPulseTag:     pulse_type = LastPulse; break;
        case PercNotFirstPulseTag: pulse_type = NotFirstPulse; break;
        case PercNotLastPulseTag:  pulse_type = NotLastPulse; break;
        case PercMultiPulseTag:    pulse_type = MultiplePulse; break;
      }
    }

    if (attribute_category < 7) {
      point_attribute = BasicLaserPointTag(*segment_attribute);
      attribute_type = AttributeType(point_attribute);
    }
    
    // Retrieve tile number and index in the tile number list
    tile_number = tile->TileNumber();
    tile_number_iterator = std::find(tile_numbers.begin(), tile_numbers.end(),
                                     tile_number);
    this_tile_index = std::distance(tile_numbers.begin(), tile_number_iterator);
  
    // Collect point information
    for (point=tile->begin(), point_index=0; point!=tile->end(); point++, point_index++) {
      if (point->HasAttribute(SegmentNumberTag)) {
        // Determine segment number
        segment = point->LongSegmentNumber();
        
        // Check if this segment has already been processed segment wise
        if (attribute_category == 8) {
          if (std::find(processed_segments.begin(),
		                processed_segments.end(), segment) !=
			  processed_segments.end()) {
			continue;
		  }
        }
      
        // Check if this is a small or large segment and get the right index and
        // pointers to counts, values and planes
        segment_is_large = true;
        if (segment / 1000000 == tile_number) {
          tile_index = this_tile_index;
          large_segments_in_tile_iterator = large_segments_in_block.begin() +
		                                    tile_index;
      	  if (std::find(large_segments_in_tile_iterator->begin(),
		                large_segments_in_tile_iterator->end(), segment) ==
			  large_segments_in_tile_iterator->end())
		    segment_is_large = false;
        }
        else {
          tile_number_iterator = std::find(tile_numbers.begin(), tile_numbers.end(),
                                           segment / 1000000);
          tile_index = std::distance(tile_numbers.begin(), tile_number_iterator);
          large_segments_in_tile_iterator = large_segments_in_block.begin() +
                                            tile_index;
        }
        if (segment_is_large) {
      	  segment_iterator = std::find(large_segments_in_tile_iterator->begin(),
		                               large_segments_in_tile_iterator->end(),
									   segment);
		  if (segment_iterator == large_segments_in_tile_iterator->end()) {
		    printf("Large segment %lld not found!\n", segment);
		    printf("Point index %d\n", point_index);
		    printf("X %.2f, Y %.2f\n", point->X(), point->Y());
		    printf("Has segment %d\n", point->HasAttribute(SegmentNumberTag));
		    printf("Segment number %d\n", point->Attribute(SegmentNumberTag));
		    printf("Has segment start tile %d\n", point->HasAttribute(SegmentStartTileNumberTag));
		    printf("Segment start tile %d\n", point->Attribute(SegmentStartTileNumberTag));
		    
		    printf("Tile number: %d     segment/1000000 %lld\n", tile_number,
		           segment/1000000);
		    printf("Tile index: %d\n", tile_index);
		    printf("This tile index: %d\n", this_tile_index);
		    printf("Large segments of this tile:");
		    for (segment_iterator=large_segments_in_tile_iterator->begin();
		         segment_iterator!=large_segments_in_tile_iterator->end();
			     segment_iterator++)
			  printf(" %lld", *segment_iterator);
		    printf("\n");
		    exit(0);
	      }
      	  segment_index = std::distance(large_segments_in_tile_iterator->begin(),
		                                segment_iterator);
		  // pointers to array values
		  count_ptr = &large_counts_in_block[tile_index][segment_index];
		  value_ptr = &large_values_in_block[tile_index][segment_index*num_attributes+iattr];
		  plane_ptr = &large_planes_in_block[tile_index][segment_index];
        }
        else {
      	  segment_index = segment % 1000000;
      	  count_ptr = &small_counts_in_tile[segment_index];
      	  value_ptr = &small_values_in_tile[segment_index*num_attributes+iattr];
      	  plane_ptr = &small_planes_in_tile[segment_index];
        }
      
        // Determine point attribute value, if required
        if (attribute_category < 7) {
          switch (attribute_type) {
            default:
            case IntegerAttributeType: 
              value = (double) point->Attribute(point_attribute); break;
            case FloatAttributeType:
              value = (double) point->FloatAttribute(point_attribute); break;
            case DoubleAttributeType:
              value = point->DoubleAttribute(point_attribute); break;
          }
        }
        
        // Process point value
        switch (attribute_category) {
          case 1: // Average
            *value_ptr += value; break;
          case 2: // Minimum
            if (*count_ptr == 0) *value_ptr = value; // This only works if the minimum is the first attribute!
            else if (value < *value_ptr) *value_ptr = value;
            break;
          case 3: // Maximum
            if (*count_ptr == 0) *value_ptr = value; // This only works if the maximum is the first attribute!
            else if (value > *value_ptr) *value_ptr = value;
            break;
          default:
          case 4: // Segment size
            break; // Nothing to do, just counting
          case 5: // Pulse type percentage
            if (point->IsPulseType((LaserPulseType) pulse_type))
              *value_ptr += 1.0;
            break;
          case 6: // Other percentages
            if (value > 0.01) *value_ptr += 1.0;
            break;
          case 7: // Plane attribute
            if (!plane_done) plane_ptr->AddPoint(point->Position3DRef(), false);
            break;
          case 8: // Segment wise processing based on a neighbourhood
            // Select all points of this segment
            if (point->HasAttribute(SegmentStartTileNumberTag)) {
              segment_points.AddTaggedPoints(tile->LaserPointsReference(),
                                             point->Attribute(SegmentNumberTag),
								             SegmentNumberTag,
								             point->Attribute(SegmentStartTileNumberTag),
								             SegmentStartTileNumberTag, IsSelectedTag);
            }
            else {
              segment_points.AddTaggedPoints(tile->LaserPointsReference(),
                                           point->Attribute(SegmentNumberTag),
								           SegmentNumberTag, IsSelectedTag);
            }
			tile->RemoveAttribute(IsSelectedTag);
            switch (*segment_attribute) {
              case AveragePointSpacingTag:
              	// We need at least 5 points to have an interior TIN edge
              	if (segment_points.size() < 5) {
              	  if (*(value_ptr+1) < 0.01) *(value_ptr+1) = 0.01; // To avoid division by zero
              	  break;
                }
                // Derive TIN edges
                parameters.NeighbourhoodStorageModel() = 0; // TIN
                parameters.DistanceMetricDimension() = 2;
                parameters.MaxDistanceInComponent() = 0.0; // No filtering of long edges
                segment_points.DeriveTIN();
                edges = segment_points.DeriveEdges(parameters);
                // Mark convex hull points
                segment_points.MarkConvexHullPoints();
  				if (segment_points.size() != edges->size()) {
				  printf("Error: edge sets (%d) and points (%d) numbers not the same\n",
				       edges->size(), segment_points.size());
				  {
				    FILE *fd_debug=fopen("edge_debug.txt", "w");
				    for (segment_point=segment_points.begin();
					     segment_point!=segment_points.end(); segment_point++)
					  fprintf(fd_debug, "%10.3f %10.3f\n", segment_point->X(), segment_point->Y());
				  }
				  exit(0);
				}
                // Use all interior edges for the point spacing statistics
                for (segment_point=segment_points.begin(), neighbours=edges->begin();
				     segment_point!=segment_points.end();
					 segment_point++, neighbours++) {
				  // Skip this point if it is on the convex hull
				  if (segment_point->Attribute(ConvexHullTag) == 1) continue;
				  // Debugging
				  if (neighbours == edges->end()) {
				  	printf("Error: less edge sets (%d) than points (%d) in the loop\n",
					       edges->size(), segment_points.size());
					exit(0);
				  }
				  // Loop over all neighbours
				  for (node=neighbours->begin(); node!=neighbours->end(); node++) {
				  	neighbour_point = segment_points.begin() + node->Number();
				  	if (neighbour_point->Attribute(ConvexHullTag) == 1) continue;
				  	value = segment_point->Distance(neighbour_point->Position3DRef());
				  	*value_ptr += value;   // Add distance
				  	*(value_ptr+1) += 1.0; // Increase edge count
				  }
			    }
				if (*(value_ptr+1) < 0.01) *(value_ptr+1) = 0.01; // To avoid division by zero
			    break;
            }
			// Clean up
			segment_points.EraseNeighbourhoodEdges();
			segment_points.EraseTIN();
			segment_points.ErasePoints();
            processed_segments.push_back(segment); // Mark segment as done
            break;
          case 9: // Slope angle variance
            if (point->HasAttribute(NormalZTag)) {
              value = acos(fabs(point->FloatAttribute(NormalZTag))) * 180.0 / pi;
			  *value_ptr += value * value;   // Accumulate square sum in SlopeAngleVarianceTag
			  *(value_ptr+1) += value;       // Accumulate sum in Sum1Tag
			  *(value_ptr+2) += 1.0;         // Accumulate count in Sum2Tag
		    }   
            break;
        }
        if (!counting_done && attribute_category != 8) *count_ptr += 1;
      }
    }
    // Set booleans to avoid second time counting or plane calculation
    if (attribute_category != 8) counting_done = true;
    if (attribute_category == 7) plane_done = true;
    
	// Clean up
	if (processed_segments.size())
      processed_segments.erase(processed_segments.begin(), processed_segments.end());
  }
}

void SetSmallSegmentAttribute(vector<LaserPointTag> &segment_attributes,
                              LaserUnit::iterator tile,
                              vector <int> &tile_numbers, 
                              vector <vector <long long int> > &large_segments_in_block, 
                              vector <long long int> highest_segments,
	                          long long int *small_counts_in_tile, 
						      double *small_values_in_tile,
						      Plane *small_planes_in_tile)
{
  vector<LaserPointTag>::iterator segment_attribute;
  LaserPointAttributeType         attribute_type;
  int                             attribute_category, num_segments, tile_index,
                                  tile_number, index, iattr, num_attributes;
  vector <int>::iterator          tile_number_iterator;
  LaserPoints::iterator           point;
  long long int                   segment;
  Vector3D                        normal;
  double                          dist, average, eigenvalue_sum;
  bool                            counting_done=false, plane_done=false;
  
  // Retrieve tile number and index in the tile number list
  tile_number = tile->TileNumber();
  tile_number_iterator = std::find(tile_numbers.begin(), tile_numbers.end(),
                                   tile_number);
  tile_index = std::distance(tile_numbers.begin(), tile_number_iterator);
  
  // Loop over all segment attributes
  num_attributes = segment_attributes.size();
  for (iattr=0, segment_attribute=segment_attributes.begin();
       iattr<num_attributes; iattr++, segment_attribute++) {

    // Determine the category of attribute
    if (*segment_attribute < SlopeAngleVarianceTag) attribute_category = 1; // Average
    else if (*segment_attribute < MinReflectanceTag) attribute_category = 9; // Variance, needs special processing
    else if (*segment_attribute < MaxReflectanceTag) attribute_category = 2; // Minimum
    else if (*segment_attribute < EdgeCountTag) attribute_category = 3; // Maximum
    else if (*segment_attribute <= SegmentSizeTag) attribute_category = 4; // Segment or component size
    else if (*segment_attribute <= PercMultiPulseTag) attribute_category = 5; // Pulse type percentage
    else if (*segment_attribute < InclinationTag) attribute_category = 6; // Other percentage
    else attribute_category = 7; // Plane attribute
    if (*segment_attribute == AveragePointSpacingTag) attribute_category = 8; // Needs special processing
    if (*segment_attribute == EdgeCountTag) continue; // Only used for average point spacing calculations
    if (*segment_attribute == Sum1Tag) continue; // Only used for variance calculations
    if (*segment_attribute == Sum2Tag) continue; // Only used for variance calculations

    // Calculate segment attribute
    num_segments = highest_segments[tile_index];
    switch (attribute_category) {
      case 1: // Average
        for (index=0; index<=num_segments; index++)
          if (small_counts_in_tile[index] > 0.0)
		    small_values_in_tile[index*num_attributes+iattr] /= small_counts_in_tile[index];
        break;
      case 2: // Minimum
      case 3: // Maximum
      case 4: // Segment size
        break; // Nothing to do
      case 5: // Pulse type percentages
      case 6: // Other percentages
        for (index=0; index<=num_segments; index++)
          if (small_counts_in_tile[index] > 0.0)
		    small_values_in_tile[index*num_attributes+iattr] *= 
			  100.0 / small_counts_in_tile[index];
        break;
      case 7: // Plane attributes
        for (index=0; index<=num_segments; index++) {
          // A plane needs at least three points
          if (small_counts_in_tile[index] >= 3) {
            // Only fit the plane once	
            if (!plane_done) small_planes_in_tile[index].Recalculate();
            normal = small_planes_in_tile[index].Normal();
            if (normal.Z() < 0.0) normal *= -1.0;
            eigenvalue_sum = small_planes_in_tile[index].FloatAttribute(PT_Lambda0) +
                             small_planes_in_tile[index].FloatAttribute(PT_Lambda1) +
                             small_planes_in_tile[index].FloatAttribute(PT_Lambda2);
            switch (*segment_attribute) {
          	  case InclinationTag:
                dist = normal.vect2D().Length();
                small_values_in_tile[index*num_attributes+iattr] = 
			      atan2(dist, fabs(normal.Z()));
			    break;
			  case AzimuthTag:
                if (normal.X() == 0.0 && normal.Y() == 0.0)
                  small_values_in_tile[index*num_attributes+iattr] = 0.0;
                else {
                  small_values_in_tile[index*num_attributes+iattr] = 
				    atan2(normal.X(), normal.Y());
                  if (small_values_in_tile[index*num_attributes+iattr] < 0.0)
			        small_values_in_tile[index*num_attributes+iattr] += atan(1.0) * 8.0;
                }
                break;
              case Lambda0Tag:
                small_values_in_tile[index*num_attributes+iattr] = 
                  (double) small_planes_in_tile[index].FloatAttribute(PT_Lambda0);
                break;
              case Lambda1Tag:
                small_values_in_tile[index*num_attributes+iattr] = 
                  (double) small_planes_in_tile[index].FloatAttribute(PT_Lambda1);
                break;
              case Lambda2Tag:
                small_values_in_tile[index*num_attributes+iattr] = 
                  (double) small_planes_in_tile[index].FloatAttribute(PT_Lambda2);
                break;
              case Lambda0ScaledTag:
              	if (eigenvalue_sum == 0.0)
              	  small_values_in_tile[index*num_attributes+iattr] = 1.0 / 3.0;
              	else
                  small_values_in_tile[index*num_attributes+iattr] = 
                    (double) small_planes_in_tile[index].FloatAttribute(PT_Lambda0) /
					eigenvalue_sum;
                break;
              case Lambda1ScaledTag:
              	if (eigenvalue_sum == 0.0)
              	  small_values_in_tile[index*num_attributes+iattr] = 1.0 / 3.0;
              	else
                  small_values_in_tile[index*num_attributes+iattr] = 
                    (double) small_planes_in_tile[index].FloatAttribute(PT_Lambda1) /
					eigenvalue_sum;
                break;
              case Lambda2ScaledTag:
              	if (eigenvalue_sum == 0.0)
              	  small_values_in_tile[index*num_attributes+iattr] = 1.0 / 3.0;
              	else
                  small_values_in_tile[index*num_attributes+iattr] = 
                    (double) small_planes_in_tile[index].FloatAttribute(PT_Lambda2) /
					eigenvalue_sum;
                break;
              default:  // Should not come here
                small_values_in_tile[index*num_attributes+iattr] = 999.0;
            }
          }
          else {
            small_values_in_tile[index*num_attributes+iattr] = 999.0;
          }
        }
        break;
      case 8: // Segment wise processed data
        switch (*segment_attribute) {
          case AveragePointSpacingTag:
          	// Divide edge length sum by the number of edges stored as the next attribute
            for (index=0; index<=num_segments; index++)
          	  small_values_in_tile[index*num_attributes+iattr] /=
          	    small_values_in_tile[index*num_attributes+iattr+1];
          	break;
        }
        break;
      case 9: // Variances
        switch (*segment_attribute) {
          case SlopeAngleVarianceTag:
          	// Calculate variance based on square sum, sum and count stored in next attributes
            for (index=0; index<=num_segments; index++) {
              if (small_values_in_tile[index*num_attributes+iattr+2] < 0.01) { // No points with normal
                small_values_in_tile[index*num_attributes+iattr] = 0.0;
              }
              else {
                average = small_values_in_tile[index*num_attributes+iattr+1] /
                          small_values_in_tile[index*num_attributes+iattr+2];
                small_values_in_tile[index*num_attributes+iattr] =
                  small_values_in_tile[index*num_attributes+iattr] /
                  small_values_in_tile[index*num_attributes+iattr+2] - average * average;
              }
            }
          	break;
        }
        break;
    }
   
    // Store segment attributes in points      
    attribute_type = AttributeType(*segment_attribute);
    for (point=tile->begin(); point!=tile->end(); point++) {
      if (point->HasAttribute(SegmentNumberTag)) {
        segment = point->LongSegmentNumber();
        if (segment < 0) continue;
      
        // Skip this point if the segment does not start in this tile
        if (segment / 1000000 != tile_number) continue;
      
        // Skip this point if it belongs to a large segment
        if (std::find(large_segments_in_block[tile_index].begin(),
	                  large_segments_in_block[tile_index].end(), segment) !=
		    large_segments_in_block[tile_index].end()) continue;
      
        // Get the right index by removing the tile number from the segment number
        index = segment % 1000000;
      
        // Set the attribute value
        if (*segment_attribute != SegmentSizeTag) {
          switch (attribute_type) {
            default:
            case IntegerAttributeType:
              point->SetAttribute(*segment_attribute,
		                          (int) (small_values_in_tile[index*num_attributes+iattr]+0.5));
              break;
            case FloatAttributeType:
              point->SetAttribute(*segment_attribute,
	  	                        (float) small_values_in_tile[index*num_attributes+iattr]);
              break;
            case DoubleAttributeType:
              point->SetDoubleAttribute(*segment_attribute,
		                                small_values_in_tile[index*num_attributes+iattr]);
              break;
          }
        }
        if (!counting_done)
		  point->SetAttribute(SegmentSizeTag, (int) small_counts_in_tile[index]);
      }
//    else point->SetAttribute(SegmentSizeTag, 0);
    }
    counting_done = true;
    if (attribute_category == 7) plane_done = true;
  }
  
  // Re-initialise planes to de-allocate memory
  if (plane_done) {
  	for (index=0; index<=num_segments; index++)
      small_planes_in_tile[index].Initialise();
  }
}

void CalculateLargeSegmentAttribute(vector<LaserPointTag> segment_attributes,
                                    vector< vector<long long int> > large_segments_in_block,
                                    vector<long long int *> large_counts_in_block,
						            vector<double *> large_values_in_block,
						            vector<Plane *> large_planes_in_block)
{
  vector<LaserPointTag>::iterator           segment_attribute;
  int                                       attribute_category, index, num_segments,
                                            iattr, num_attributes;
  vector <long long int *>::iterator        large_counts_in_tile_iterator;
  vector <double *>::iterator               large_values_in_tile_iterator;
  vector <Plane *>::iterator                large_planes_in_tile_iterator;
  vector< vector<long long int> >::iterator large_segments_in_tile_iterator;
  Vector3D                                  normal;
  double                                    dist, average, eigenvalue_sum;
  bool                                      plane_done=false;
  
  // Loop over all segment attributes
  num_attributes = segment_attributes.size();
  for (iattr=0, segment_attribute=segment_attributes.begin();
       iattr<num_attributes; iattr++, segment_attribute++) {

    // Determine the category of attribute
    if (*segment_attribute < SlopeAngleVarianceTag) attribute_category = 1; // Average
    else if (*segment_attribute < MinReflectanceTag) attribute_category = 9; // Variance, needs special processing
    else if (*segment_attribute < MaxReflectanceTag) attribute_category = 2; // Minimum
    else if (*segment_attribute < EdgeCountTag) attribute_category = 3; // Maximum
    else if (*segment_attribute <= SegmentSizeTag) attribute_category = 4; // Segment or component size
    else if (*segment_attribute <= PercMultiPulseTag) attribute_category = 5; // Pulse type percentage
    else if (*segment_attribute < InclinationTag) attribute_category = 6; // Other percentage
    else attribute_category = 7; // Plane attribute
    if (*segment_attribute == AveragePointSpacingTag) attribute_category = 8; // Needs special processing
    if (*segment_attribute == EdgeCountTag) continue; // Only used for average point spacing calculations
    if (*segment_attribute == Sum1Tag) continue; // Only used for variance calculations
    if (*segment_attribute == Sum2Tag) continue; // Only used for variance calculations

    // Loop over all tiles
    for (large_counts_in_tile_iterator=large_counts_in_block.begin(),
         large_values_in_tile_iterator=large_values_in_block.begin(),
	     large_planes_in_tile_iterator=large_planes_in_block.begin(),
	     large_segments_in_tile_iterator=large_segments_in_block.begin();
	     large_counts_in_tile_iterator!=large_counts_in_block.end();
	     large_counts_in_tile_iterator++, large_values_in_tile_iterator++,
	     large_planes_in_tile_iterator++, large_segments_in_tile_iterator++) {

      // Calculate segment attribute
      num_segments = large_segments_in_tile_iterator->size();
      switch (attribute_category) {
        case 1: // Average
          for (index=0; index<=num_segments; index++)
            if ((*large_counts_in_tile_iterator)[index] > 0.0)
		      (*large_values_in_tile_iterator)[index*num_attributes+iattr] /= 
			    (*large_counts_in_tile_iterator)[index];
          break;
        case 2: // Minimum
        case 3: // Maximum
        case 4: // Segment size
          break; // Nothing to do
        case 5: // Pulse type percentages
        case 6: // Other percentages
          for (index=0; index<=num_segments; index++)
            if ((*large_counts_in_tile_iterator)[index] > 0.0)
		      (*large_values_in_tile_iterator)[index*num_attributes+iattr] *= 
			    100.0 / (*large_counts_in_tile_iterator)[index];
          break;

        case 7: // Plane attributes
          for (index=0; index<=num_segments; index++) {
            // A plane needs at least three points
            if ((*large_counts_in_tile_iterator)[index] >= 3) {
              // Only fit the plane once	
              if (!plane_done) (*large_planes_in_tile_iterator)[index].Recalculate();
              normal = (*large_planes_in_tile_iterator)[index].Normal();
              if (normal.Z() < 0.0) normal *= -1.0;
              eigenvalue_sum = (*large_planes_in_tile_iterator)[index].FloatAttribute(PT_Lambda0) +
                               (*large_planes_in_tile_iterator)[index].FloatAttribute(PT_Lambda1) +
                               (*large_planes_in_tile_iterator)[index].FloatAttribute(PT_Lambda2);
              switch (*segment_attribute) {
          	    case InclinationTag:
                  dist = normal.vect2D().Length();
                  (*large_values_in_tile_iterator)[index*num_attributes+iattr] = 
			        atan2(dist, fabs(normal.Z()));
			      break;
			    case AzimuthTag:
                  if (normal.X() == 0.0 && normal.Y() == 0.0)
                    (*large_values_in_tile_iterator)[index*num_attributes+iattr] = 0.0;
                  else {
                   (*large_values_in_tile_iterator)[index*num_attributes+iattr] = 
				      atan2(normal.X(), normal.Y());
                    if ((*large_values_in_tile_iterator)[index*num_attributes+iattr] < 0.0)
			          (*large_values_in_tile_iterator)[index*num_attributes+iattr] += atan(1.0) * 8.0;
                  }
                  break;
                case Lambda0Tag:
                  (*large_values_in_tile_iterator)[index*num_attributes+iattr] = 
                    (double) (*large_planes_in_tile_iterator)[index].FloatAttribute(PT_Lambda0);
                  break;
                case Lambda1Tag:
                  (*large_values_in_tile_iterator)[index*num_attributes+iattr] = 
                    (double) (*large_planes_in_tile_iterator)[index].FloatAttribute(PT_Lambda1);
                  break;
                case Lambda2Tag:
                  (*large_values_in_tile_iterator)[index*num_attributes+iattr] = 
                    (double) (*large_planes_in_tile_iterator)[index].FloatAttribute(PT_Lambda2);
                  break;
                case Lambda0ScaledTag:
              	  if (eigenvalue_sum == 0.0)
              	    (*large_values_in_tile_iterator)[index*num_attributes+iattr] = 1.0 / 3.0;
              	  else
                    (*large_values_in_tile_iterator)[index*num_attributes+iattr] = 
                      (double) (*large_planes_in_tile_iterator)[index].FloatAttribute(PT_Lambda0) /
				  	  eigenvalue_sum;
                  break;
                case Lambda1ScaledTag:
              	  if (eigenvalue_sum == 0.0)
              	    (*large_values_in_tile_iterator)[index*num_attributes+iattr] = 1.0 / 3.0;
              	  else
                    (*large_values_in_tile_iterator)[index*num_attributes+iattr] = 
                      (double) (*large_planes_in_tile_iterator)[index].FloatAttribute(PT_Lambda1) /
				  	  eigenvalue_sum;
                  break;
                case Lambda2ScaledTag:
              	  if (eigenvalue_sum == 0.0)
              	    (*large_values_in_tile_iterator)[index*num_attributes+iattr] = 1.0 / 3.0;
              	  else
                    (*large_values_in_tile_iterator)[index*num_attributes+iattr] = 
                      (double) (*large_planes_in_tile_iterator)[index].FloatAttribute(PT_Lambda2) /
				  	  eigenvalue_sum;
                  break;

                default:  // Should not come here
                  (*large_values_in_tile_iterator)[index*num_attributes+iattr] = 999.0;
              }
            }
            else {
              (*large_values_in_tile_iterator)[index*num_attributes+iattr] = 999.0;
            }
          }
          break;

        case 8: // Segment wise processed data
          switch (*segment_attribute) {
            case AveragePointSpacingTag:
          	  // Divide edge length sum by the number of edges stored as the next attribute
              for (index=0; index<=num_segments; index++)
          	    (*large_values_in_tile_iterator)[index*num_attributes+iattr] /=
          	      (*large_values_in_tile_iterator)[index*num_attributes+iattr+1];
          	  break;
          }
          break;

        case 9: // Variances
          switch (*segment_attribute) {
            case SlopeAngleVarianceTag:
          	  // Calculate variance based on count, square sum, and sum stored in next attribute
              for (index=0; index<=num_segments; index++) {
              	if ((*large_values_in_tile_iterator)[index*num_attributes+iattr+2] < 0.01) { // No point with normal vector
              	  (*large_values_in_tile_iterator)[index*num_attributes+iattr] = 0.0;
              	}
              	else {
                  average = (*large_values_in_tile_iterator)[index*num_attributes+iattr+1] /
                            (*large_values_in_tile_iterator)[index*num_attributes+iattr+2];
                  (*large_values_in_tile_iterator)[index*num_attributes+iattr] =
                    (*large_values_in_tile_iterator)[index*num_attributes+iattr] /
                    (*large_values_in_tile_iterator)[index*num_attributes+iattr+2] -
				    average * average;
              	}
              }
          	  break;
          }
          break;
      }
    }
    if (attribute_category == 7) plane_done = true;
  }

  // Re-initialise all planes to de-allocate memory
  if (plane_done) {
    for (large_planes_in_tile_iterator=large_planes_in_block.begin();
	     large_planes_in_tile_iterator!=large_planes_in_block.end();
	     large_planes_in_tile_iterator++) {
  	  for (index=0; index<=num_segments; index++)
        (*large_planes_in_tile_iterator)[index].Initialise();
    }
  }
}

void SetLargeSegmentAttribute(vector<LaserPointTag> segment_attributes,
                              LaserUnit::iterator tile,
                              vector <int> &tile_numbers, 
                              vector< vector<long long int> > large_segments_in_block,
                              vector<long long int *> large_counts_in_block,
						      vector<double *> large_values_in_block)
{
  vector<LaserPointTag>::iterator           segment_attribute;
  LaserPoints::iterator                     point;
  long long int                             segment;
  bool                                      large_segment, counts_set=false;
  int                                       tile_number, tile_index,
                                            this_tile_index, segment_index,
											iattr, num_attributes;
  vector< vector<long long int> >::iterator large_segments_in_tile,
                                            large_segments_in_this_tile;
  vector<long long int>::iterator           segment_iterator;
  vector <int>::iterator                    tile_number_iterator;
  LaserPointAttributeType                   attribute_type;
  
  // Get the index of the current tile and its large segment numbers
  tile_number = tile->TileNumber();
  tile_number_iterator = std::find(tile_numbers.begin(), tile_numbers.end(),
                                   tile_number);
  this_tile_index = std::distance(tile_numbers.begin(), tile_number_iterator);
  large_segments_in_this_tile = large_segments_in_block.begin() + this_tile_index;


  // Loop over all segment attributes
  num_attributes = segment_attributes.size();
  for (iattr=0, segment_attribute=segment_attributes.begin();
       iattr<num_attributes; iattr++, segment_attribute++) {
    if (*segment_attribute == EdgeCountTag) continue; // Only used for average point spacing calculations
    if (*segment_attribute == Sum1Tag) continue; // Only used for variance calculations
    if (*segment_attribute == Sum2Tag) continue; // Only used for variance calculations
    attribute_type = AttributeType(*segment_attribute);
    for (point=tile->begin(); point!=tile->end(); point++) {
      if (!point->HasAttribute(SegmentNumberTag)) continue;
      segment = point->LongSegmentNumber();
      
      // Determine if this is a large segment and get the right segment list
      large_segment = false;
      if (segment / 1000000 != tile_number) {
        large_segment = true;
        tile_number_iterator = std::find(tile_numbers.begin(), tile_numbers.end(),
	                                     segment / 1000000);
	    tile_index = std::distance(tile_numbers.begin(), tile_number_iterator);
	    large_segments_in_tile = large_segments_in_block.begin() + tile_index;
	    segment_iterator = std::find(large_segments_in_tile->begin(),
	                                 large_segments_in_tile->end(), segment);
	    if (segment_iterator == large_segments_in_tile->end()) {
	  	  printf("Error in SetLargeSegmentAttribute: segment number not found\n");
	  	  exit(0);
	    }
	    segment_index = std::distance(large_segments_in_tile->begin(),
	                                  segment_iterator);
      }
      else {
	    segment_iterator = std::find(large_segments_in_this_tile->begin(),
	                                 large_segments_in_this_tile->end(), segment);
	    if (segment_iterator != large_segments_in_this_tile->end()) {
	  	  large_segment = true;
	  	  tile_index = this_tile_index;
	      segment_index = std::distance(large_segments_in_this_tile->begin(),
		                                segment_iterator);
	    }
      }
      if (!large_segment) continue;
    
      // Look up value and count and set attribute and segment size
      if (*segment_attribute != SegmentSizeTag) {
        switch (attribute_type) {
          case IntegerAttributeType:
      	    point->SetAttribute(*segment_attribute,
	  	                      (int) large_values_in_block[tile_index][segment_index*num_attributes+iattr]);
		    break;
 
          case FloatAttributeType:
      	    point->SetAttribute(*segment_attribute,
		                        (float) large_values_in_block[tile_index][segment_index*num_attributes+iattr]);
		    break;

          case DoubleAttributeType:
      	    point->SetDoubleAttribute(*segment_attribute,
		                              large_values_in_block[tile_index][segment_index*num_attributes+iattr]);
		    break;
	    }
      }
      if (!counts_set)
        point->Attribute(SegmentSizeTag) =
          (int) large_counts_in_block[tile_index][segment_index];
    }	
    counts_set = true;
  }
}

/*
--------------------------------------------------------------------------------
                         The main blockattributes function
--------------------------------------------------------------------------------
*/

void blockattributes(char *block_file, vector<LaserPointTag> &segment_attributes,
                     const SegmentationParameters &segmentation_parameters)
{
  LaserBlock              block;
  LaserBlock::iterator    tile_container;
  LaserUnit::iterator     tile;
  LaserPoints::iterator   point;
  int                     success, index, tile_number, num_tiles, i;
  
  vector<int>                               tile_numbers;
  vector<int>::iterator                     tile_number_iterator;
  long long int                             segment, highest_segment, num_pts=0,
                                            num_large=0, num_small=0;
  vector< vector<long long int> >           segments_in_block,
                                            small_segments_in_block,
											large_segments_in_block;
  vector< vector<long long int> >::iterator segments_in_tile_iterator,
                                            small_segments_in_tile_iterator,
											large_segments_in_tile_iterator;
  vector<long long int *>                   large_counts_in_block;
  vector<long long int *>::iterator         large_counts_in_tile_iterator;
  vector<double *>                          large_values_in_block;
  vector<double *>::iterator                large_values_in_tile_iterator;
  vector<Plane *>                           large_planes_in_block;
  vector<Plane *>::iterator                 large_planes_in_tile_iterator;
  vector<long long int>                     segments_in_tile, highest_segments;
  vector<long long int>::iterator           segment_in_tile;
  long long int                             *small_counts_in_tile=NULL;
  double                                    *small_values_in_tile=NULL;
  Plane                                     *small_planes_in_tile=NULL;
  const unsigned char                       *attribute;
  
  // Open the block
  if (!block.ReadMetaData(block_file)) {
    fprintf(stderr, "Error reading block meta data file %s\n", block_file);
    exit(0);
  }
  
  // Check if there are tiles
  if (block.empty()) {
  	printf("Error: block %s contains no data\n", block.Name());
  	exit(0);
  }
  tile_container = block.begin();
  if (tile_container->empty() ||
      tile_container->DataOrganisation() != TileWise) {
  	printf("Error: block %s contains no tiles\n", block.Name());;
  	exit(0);
  }

  // First collect all segment numbers per tile
  printf("Collecting all segment numbers per tile\n");
  for (tile=tile_container->begin(); tile!=tile_container->end(); tile++) {
    printf("Tile (%3d, %3d)\r", tile->TileRow(), tile->TileColumn());

    // Read the point data
    if (!tile->Read(tile->PointFile(), false)) {
      fprintf(stderr, "Error reading laser points from file %s\n",
              tile->PointFile());
      exit(0);
    }
    num_pts += (long long int) tile->size();

    // Make a list of all segment numbers in this tile
    for (point=tile->begin(); point!=tile->end(); point++) {
      segment = point->LongSegmentNumber();
      if (segment == INT_MIN) continue; // No segment number
      if (std::find(segments_in_tile.begin(),
	                segments_in_tile.end(), segment) ==
		  segments_in_tile.end()) {
		segments_in_tile.push_back(segment);
	  }
	}
	segments_in_block.push_back(segments_in_tile);
	segments_in_tile.erase(segments_in_tile.begin(), segments_in_tile.end());
	                              
	// Erase points
	tile->ErasePoints();
  }
      
  // For every tile, create a list of segment numbers only used in this tile
  // and a list of segment numbers also used in other tiles. These lists
  // are called small_segments_in_tile and large_segments_in_tile.
  printf("\nDetermining segments across tile boundaries\n");
  for (tile=tile_container->begin(); tile!=tile_container->end(); tile++) {
    tile_numbers.push_back(tile->TileNumber());
    small_segments_in_block.push_back(vector<long long int>());
    large_segments_in_block.push_back(vector<long long int>());
  }
  // First collect the segments crossing tile boundaries
  num_tiles = tile_container->size();
  for (tile=tile_container->begin(), 
       segments_in_tile_iterator=segments_in_block.begin();
       tile!=tile_container->end();
	   tile++, segments_in_tile_iterator++) {
	tile_number = tile->TileNumber();
    for (segment_in_tile=segments_in_tile_iterator->begin();
	     segment_in_tile!=segments_in_tile_iterator->end();
		 segment_in_tile++) {
	  // Check if this segments starts in another tile	
	  if (*segment_in_tile / 1000000 != tile_number) {
	  	// Look up the tile index
	  	tile_number_iterator = std::find(tile_numbers.begin(),
		                                 tile_numbers.end(),
										 *segment_in_tile / 1000000);
		index = std::distance(tile_numbers.begin(), tile_number_iterator);
		// Add the tile number to the large segment vector if that hasn't
		// been done so yet
		if (index < num_tiles) { // Check needed for incomplete blocks
		  if (std::find(large_segments_in_block[index].begin(),
		                large_segments_in_block[index].end(),
			  		    *segment_in_tile) ==
			  large_segments_in_block[index].end()) {
		    large_segments_in_block[index].push_back(*segment_in_tile);
		  }
	    }
	  }
	}
  }
  // All segments that are not in a large segment number list, must be small
  for (tile=tile_container->begin(), 
       segments_in_tile_iterator=segments_in_block.begin(),
	   small_segments_in_tile_iterator=small_segments_in_block.begin(),
	   large_segments_in_tile_iterator=large_segments_in_block.begin();
       tile!=tile_container->end();
	   tile++, segments_in_tile_iterator++, small_segments_in_tile_iterator++,
	   large_segments_in_tile_iterator++) {
	tile_number = tile->TileNumber();
	num_large += large_segments_in_tile_iterator->size();
    for (segment_in_tile=segments_in_tile_iterator->begin();
	     segment_in_tile!=segments_in_tile_iterator->end();
		 segment_in_tile++) {
	  // Only check segments that start in this tile
	  if (*segment_in_tile / 1000000 == tile_number) {
	  	// Check if they are in the large segment list
        if (std::find(large_segments_in_tile_iterator->begin(),
		              large_segments_in_tile_iterator->end(),
					  *segment_in_tile) ==
			large_segments_in_tile_iterator->end()) {
		  // If not, store it in the small segment list if not yet done so.
		  if (std::find(small_segments_in_tile_iterator->begin(),
		                small_segments_in_tile_iterator->end(),
						*segment_in_tile) ==
			  small_segments_in_tile_iterator->end()) {
			small_segments_in_tile_iterator->push_back(*segment_in_tile);
		  }		
	    }
	  }
    }
    // No longer need for the segment in tile list
    segments_in_tile_iterator->erase(segments_in_tile_iterator->begin(),
	                                 segments_in_tile_iterator->end());
  }
  // No longer need for the segment in block list
  segments_in_block.erase(segments_in_block.begin(),
                                   segments_in_block.end());

  // Output some check statistics
  printf("Block %s consists of %d tiles with %lld points\n",
         block.Name(), (int) tile_container->size(), num_pts);
  for (small_segments_in_tile_iterator=small_segments_in_block.begin();
	   small_segments_in_tile_iterator!=small_segments_in_block.end();
       small_segments_in_tile_iterator++)
    num_small += small_segments_in_tile_iterator->size();
  printf("with %lld segments inside tiles and %lld segments across tile boundaries\n",
         num_small, num_large);

/*
  for (tile=tile_container->begin(), 
	   small_segments_in_tile_iterator=small_segments_in_block.begin(),
	   large_segments_in_tile_iterator=large_segments_in_block.begin();
       tile!=tile_container->end();
	   tile++, small_segments_in_tile_iterator++,
	   large_segments_in_tile_iterator++) {
    printf("Tile (%3d, %3d), small %4d, large %4d\n",
           tile->TileRow(), tile->TileColumn(),
		   small_segments_in_tile_iterator->size(), 
		   large_segments_in_tile_iterator->size());
	printf("Small:");
	for (segment_in_tile=small_segments_in_tile_iterator->begin();
	     segment_in_tile!=small_segments_in_tile_iterator->end();
		 segment_in_tile++)
	  printf(" %lld", *segment_in_tile);
	printf("\n");
	printf("Large:");
	for (segment_in_tile=large_segments_in_tile_iterator->begin();
	     segment_in_tile!=large_segments_in_tile_iterator->end();
		 segment_in_tile++)
	  printf(" %lld", *segment_in_tile);
	printf("\n");
  }
*/

  // Create count, value and plane arrays for all segments across tile boundaries
  for (tile=tile_container->begin(), 
	   large_segments_in_tile_iterator=large_segments_in_block.begin();
       tile!=tile_container->end();
	   tile++, large_segments_in_tile_iterator++) {
	large_counts_in_block.push_back(
	  (long long int *) calloc(large_segments_in_tile_iterator->size()+1,
	                           sizeof(long long int)));
	large_values_in_block.push_back(
	  (double *) calloc(large_segments_in_tile_iterator->size()+1, sizeof(double) *
	                    segment_attributes.size()));
	large_planes_in_block.push_back(
	  (Plane *) calloc(large_segments_in_tile_iterator->size()+1, sizeof(Plane)));
  }
  
  // Processing loop over all tiles
  printf("Attribute calculations and storing results of segments inside tiles\n");
  for (tile=tile_container->begin(), 
	   small_segments_in_tile_iterator=small_segments_in_block.begin();
       tile!=tile_container->end();
	   tile++, small_segments_in_tile_iterator++) {
    printf("Tile (%3d, %3d)\r", tile->TileRow(), tile->TileColumn());

    // Determine the highest segment number of the small segments inside the tile
    highest_segment = 0;
    for (segment_in_tile=small_segments_in_tile_iterator->begin();
	     segment_in_tile!=small_segments_in_tile_iterator->end();
		 segment_in_tile++)
	  if (*segment_in_tile > highest_segment) highest_segment = *segment_in_tile;
	highest_segment = highest_segment % 1000000;
	highest_segments.push_back(highest_segment);
		
    // Create count, value and plane arrays for all segments within tile only
    small_counts_in_tile = (long long int *)
	  realloc(small_counts_in_tile, (highest_segment+1) * sizeof(long long int));
    small_values_in_tile = (double *)
	  realloc(small_values_in_tile, (highest_segment+1) * sizeof(double) *
	          segment_attributes.size());
    small_planes_in_tile = (Plane *)
	  realloc(small_planes_in_tile, (highest_segment+1) * sizeof(Plane));
    
	// Initialise all arrays
	memset(small_counts_in_tile, 0, (highest_segment+1) * sizeof(long long int));
	memset(small_values_in_tile, 0, (highest_segment+1) * sizeof(double) *segment_attributes.size());
	memset(small_planes_in_tile, 0, (highest_segment+1) * sizeof(Plane));
    
    // Read tile points
    if (!tile->Read(tile->PointFile(), false)) {
      fprintf(stderr, "Error reading laser points from file %s\n",
              tile->PointFile());
      exit(0);
    }

    // Calculate point attributes, if needed
    if ((std::find(segment_attributes.begin(), segment_attributes.end(),
	               AverageLambda0LocalTag) != segment_attributes.end()) ||
        (std::find(segment_attributes.begin(), segment_attributes.end(),
	               AverageLambda1LocalTag) != segment_attributes.end()) ||
        (std::find(segment_attributes.begin(), segment_attributes.end(),
	               AverageLambda2LocalTag) != segment_attributes.end()) ||
        (std::find(segment_attributes.begin(), segment_attributes.end(),
	               AverageLambda0LocalScaledTag) != segment_attributes.end()) ||
        (std::find(segment_attributes.begin(), segment_attributes.end(),
	               AverageLambda1LocalScaledTag) != segment_attributes.end()) ||
        (std::find(segment_attributes.begin(), segment_attributes.end(),
	               AverageLambda2LocalScaledTag) != segment_attributes.end()) ||
        (std::find(segment_attributes.begin(), segment_attributes.end(),
	               AverageFlatnessLocalTag) != segment_attributes.end()) ||
        (std::find(segment_attributes.begin(), segment_attributes.end(),
	               AverageLinearityLocalTag) != segment_attributes.end())) {
	  if (std::find(segment_attributes.begin(), segment_attributes.end(),
	               AverageLambda0LocalTag) != segment_attributes.end())
	    tile->CalculateLocalEigenValues(segmentation_parameters, false);
	  else
	    tile->CalculateLocalEigenValues(segmentation_parameters, true);
	  if ((std::find(segment_attributes.begin(), segment_attributes.end(),
	                 AverageFlatnessLocalTag) != segment_attributes.end()) ||
          (std::find(segment_attributes.begin(), segment_attributes.end(),
	                 AverageLinearityLocalTag) != segment_attributes.end()))
	    tile->CalculateLocalShape();
	}

    // Update counters, values etc for all points
	UpdateAttributeSums(segment_attributes, tile, tile_numbers,
	                    large_segments_in_block, 
	                    small_counts_in_tile, small_values_in_tile,
	                    small_planes_in_tile, large_counts_in_block,
						large_values_in_block, large_planes_in_block);
    
    // Calculate and set attribute values for points in segments within tile
    SetSmallSegmentAttribute(segment_attributes, tile, tile_numbers,
	                         large_segments_in_block, highest_segments,
							 small_counts_in_tile, small_values_in_tile,
							 small_planes_in_tile);
							 
    // Save the points and erase them
    tile->Write(tile->PointFile(), false, false);
    tile->ErasePoints();
  }
  
  // Calculate attribute for points in segments crossing tile boundaries
  CalculateLargeSegmentAttribute(segment_attributes, large_segments_in_block,
                                 large_counts_in_block, large_values_in_block,
						         large_planes_in_block);
  
  // Loop over all tiles
  printf("\nStoring results for segments across tile boundaries\n");
  for (tile=tile_container->begin(); tile!=tile_container->end(); tile++) {
    printf("Tile (%3d, %3d)\r", tile->TileRow(), tile->TileColumn());
    
    // Read tile points
    if (!tile->Read(tile->PointFile(), false)) {
      fprintf(stderr, "Error reading laser points from file %s\n",
              tile->PointFile());
      exit(0);
    }

    // Set attributes for points in segments across boundaries
    SetLargeSegmentAttribute(segment_attributes, tile, tile_numbers,
                             large_segments_in_block, large_counts_in_block,
						     large_values_in_block);

    // Save the points and erase them
    tile->Write(tile->PointFile(), false, false);
    tile->ErasePoints();
  }
}
