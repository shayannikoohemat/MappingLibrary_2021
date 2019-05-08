/*
    Copyright 2014 University of Twente
 
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
 Derivation of neighbour attributes in a tiled block of laser data
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
;
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <malloc.h>
#include "LaserBlock.h"
#include "TINEdges.h"
#include "BNF_io.h"

enum NeighbourAttributeTag { NA_NumberOfPointsTag, NA_AverageHeightDifferenceTag,
                             NA_AverageAngleTag,
                             // Point distribution attributes
							 NA_Lambda0Tag=100, NA_Lambda1Tag, NA_Lambda2Tag,
							 NA_Lambda0ScaledTag, NA_Lambda1ScaledTag, NA_Lambda2ScaledTag,
							 NA_Lambda0LocalScaledTag, NA_Lambda1LocalScaledTag, NA_Lambda2LocalScaledTag,
							 // Sums 
							 NA_Sum1Tag=200
};

typedef enum NeighbourAttributeTag NeighbourAttributeTag;



// Derive neighbourhood attributes

void UpdateAttributeSums(vector<NeighbourAttributeTag> &attributes,
                         LaserUnit::iterator tile,
                         vector <int> &tile_numbers, 
                         vector< vector<long long int> > large_segments_in_block,
                         vector< vector< pair<long long int, long long int> > >::iterator small_nbs_in_tile_iterator,
                         vector< vector< pair<long long int, long long int> > > &large_nbs_in_block,
	                     long long int *small_counts_in_tile, 
	                     long long int *small_edge_counts_in_tile, 
						 double *small_values_in_tile,
						 Plane *small_planes_in_tile,
						 vector <long long int *> &large_counts_in_block,
						 vector <long long int *> &large_edge_counts_in_block,
						 vector <double *> &large_values_in_block, 
						 vector <Plane *> &large_planes_in_block,
						 SegmentationParameters &parameters)
{
  LaserPointTag           point_attribute;
  vector<NeighbourAttributeTag>::iterator nb_attribute;
  LaserPointAttributeType attribute_type;
  int                     iattr, nb_index,  num_attributes, tile_index, 
                          tile_number, i;
  long long int           segment1, segment2, *count_ptr, *edge_count_ptr;
  double                  value, dist, *value_ptr, angle, pi=4.0*atan(1.0);
  Plane                   *plane_ptr;
  LaserPoints::iterator   point1, point2;
  bool                    large_segment1, large_segment2, plane_done, counting_done;
  vector<int>::iterator   tile_number_iterator;
  TINEdges                *edges;
  TINEdges::iterator      nbh_nodes;
  PointNumberList::iterator node; 
  pair<long long int, long long int>        neighbours;
  vector< pair<long long int, long long int> >::iterator nb_in_tile_iterator;
  vector< vector<long long int> >::iterator large_segments_in_this_tile;
  bool debug=false;

  // Determine neighbourhoods
  if (parameters.GrowingNeighbourhoodDefinition() == 0)
    edges = tile->DeriveEdges(parameters);
  else
    edges = tile->DeriveFDNEdges(parameters);
  
  // Get this tile's large segments
  tile_number = tile->TileNumber();
  tile_number_iterator = std::find(tile_numbers.begin(), tile_numbers.end(),
								   tile_number);
  tile_index = std::distance(tile_numbers.begin(), tile_number_iterator);
  large_segments_in_this_tile = large_segments_in_block.begin() + tile_index;
  
  // Looop over all points
  num_attributes = attributes.size();
  for (point1=tile->begin(), nbh_nodes=edges->begin(), i=0;
	   point1!=tile->end(); point1++, nbh_nodes++, i++) {

	// Determine segment number
    if (!point1->HasAttribute(SegmentNumberTag)) continue;
    segment1 = point1->LongSegmentNumber();
    
	// Determine whether it's a large or small segment
    tile_number = segment1/1000000;
    if (tile_number != tile->TileNumber()) large_segment1 = true;
    else {
      if (std::find(large_segments_in_this_tile->begin(),
	                large_segments_in_this_tile->end(), segment1) ==
		  large_segments_in_this_tile->end()) {
		large_segment1 = false;
	  }
	  else {
	    large_segment1 = true;
	  }
	}
	  
    // Loop over all edges
    for (node=nbh_nodes->begin(); node!=nbh_nodes->end(); node++) {
    
      // Determine segment number
      point2 = tile->begin() + node->Number();
      if (!point2->HasAttribute(SegmentNumberTag)) continue; // Not part of a segment
      segment2 = point2->LongSegmentNumber();
      if (segment1 == segment2) continue; // Points belong to the same segment
      
	  // Determine whether it's a large or small segment
      tile_number = segment2/1000000;
      if (tile_number != tile->TileNumber()) large_segment2 = true;
      else {
        if (std::find(large_segments_in_this_tile->begin(),
	                  large_segments_in_this_tile->end(), segment2) ==
		    large_segments_in_this_tile->end()) {
		  large_segment2 = false;
	    }
	    else {
	      large_segment2 = true;
	    }
	  }

      // Compose the neighbour pair
      if (large_segment1 && !large_segment2) { // Pair in this tile, segment 2 first
      	neighbours.first  = segment2;
      	neighbours.second = segment1;
      }
      else if (!large_segment1 && large_segment2) { // Pair in this tile, segment 1 first
      	neighbours.first  = segment1;
      	neighbours.second = segment2;
      }
      else { // Pair in the tile of the segment with the smallest number
        if (segment1 < segment2) {
      	  neighbours.first  = segment1;
      	  neighbours.second = segment2;
      	}
      	else {
      	  neighbours.first  = segment2;
      	  neighbours.second = segment1;
   	    }
      }

      if (large_segment1 && large_segment2) { // Large neighbourhood
        tile_number = neighbours.first / 1000000;
        tile_number_iterator = std::find(tile_numbers.begin(), tile_numbers.end(),
								         tile_number);
        tile_index = std::distance(tile_numbers.begin(), tile_number_iterator);
        nb_in_tile_iterator = std::find(large_nbs_in_block[tile_index].begin(),
		                                large_nbs_in_block[tile_index].end(),
										neighbours);
		if (nb_in_tile_iterator == large_nbs_in_block[tile_index].end()) {
		  printf("\nError: large neighbour pair not found!\n");
		  printf("Segment 1: %lld\n", neighbours.first);
		  printf("Segment 2: %lld\n", neighbours.second);
		  printf("Point %d, segment %lld\n", i, segment1);
		  continue;
//		  exit(0);	
		}
		nb_index = std::distance(large_nbs_in_block[tile_index].begin(),
		                         nb_in_tile_iterator);
	    count_ptr      = &large_counts_in_block[tile_index][nb_index];
	    edge_count_ptr = &large_edge_counts_in_block[tile_index][nb_index];
		plane_ptr      = &large_planes_in_block[tile_index][nb_index];
      }      
      else {
   	    nb_in_tile_iterator = std::find(small_nbs_in_tile_iterator->begin(),
		                                small_nbs_in_tile_iterator->end(),
									    neighbours);
		if (nb_in_tile_iterator == small_nbs_in_tile_iterator->end()) {
		  printf("\nError: small neighbour pair not found!\n");
		  printf("Segment 1: %lld\n", neighbours.first);
		  printf("Segment 2: %lld\n", neighbours.second);
		  printf("Point %d, segment %lld\n", i, segment1);
		  continue;
//		  exit(0);	
		}
		nb_index = std::distance(small_nbs_in_tile_iterator->begin(),
		                         nb_in_tile_iterator);
	    count_ptr      = &small_counts_in_tile[nb_index];
	    edge_count_ptr = &small_edge_counts_in_tile[nb_index];
		plane_ptr      = &small_planes_in_tile[nb_index];
	  }
 	  
      // Loop over all neighbour attributes
      plane_done = false;
	  counting_done = false;
      for (iattr=0, nb_attribute=attributes.begin();
           iattr<num_attributes; iattr++, nb_attribute++) {
        // Set value pointer
        if (large_segment1 && large_segment2) // Large neighbourhood
		  value_ptr = &large_values_in_block[tile_index][nb_index*num_attributes+iattr];
		else
		  value_ptr = &small_values_in_tile[nb_index*num_attributes+iattr];
  

        // Update attribute value
        switch (*nb_attribute) {
          default:
          case NA_NumberOfPointsTag:
          	break; // Counting will be done anyway
          	
          case NA_AverageHeightDifferenceTag:
          	if (neighbours.first == segment1)
          	  *value_ptr += point1->Z() - point2->Z();
          	else
          	  *value_ptr += point2->Z() - point1->Z();
          	break;
        
          case NA_AverageAngleTag:
          	
          	// use scaled normal if normal is not available. Where are normals
          	// computed? Only available for segmented points, only planes?
          	// but normals are locally defined, i.e. slightly vary within planar segments
          	if (point1->HasAttribute(NormalXTag) &&
			    point2->HasAttribute(NormalXTag)) {
			  *(value_ptr+1) += 1.0; // Increase counter in NA_Sum1Tag
			  angle = Angle(point1->Normal(), point2->Normal());
			  if (angle > pi / 2.0) angle = pi - angle;
			  *value_ptr += angle * 180.0 / pi;
			}
			else if (point1->HasAttribute(ScaledNormalXTag) &&
			         point2->HasAttribute(ScaledNormalXTag)) {
			  *(value_ptr+1) += 1.0; // Increase counter in NA_Sum1Tag
			  angle = Angle(point1->ScaledNormal(), point2->ScaledNormal());
			  if (angle > pi / 2.0) angle = pi - angle;
			  *value_ptr += angle * 180.0 / pi;
		    }
/*
			else {
			  if (!point1->HasAttribute(ScaledNormalXTag))
			    printf("Point (%.2f, %.2f) in segment %d without normal vector\n",
				       point1->X(), point1->Y(), point1->Attribute(SegmentNumberTag));
			  else
			    printf("Point (%.2f, %.2f) in segment %d without normal vector\n",
				       point2->X(), point2->Y(), point2->Attribute(SegmentNumberTag));
			  exit(0);
			}
*/
          	break;
          	
          case NA_Lambda0Tag:
          case NA_Lambda1Tag:
          case NA_Lambda2Tag:
          case NA_Lambda0ScaledTag:
          case NA_Lambda1ScaledTag:
          case NA_Lambda2ScaledTag:
          	if (!plane_done) {
          	  plane_ptr->AddPoint(point1->Position3DRef(), false);
          	  plane_ptr->AddPoint(point2->Position3DRef(), false);
          	  plane_done = true;
            }
            break;
        
          case NA_Lambda0LocalScaledTag:
			*value_ptr += point1->FloatAttribute(Lambda0LocalScaledTag);
			*value_ptr += point2->FloatAttribute(Lambda0LocalScaledTag);
          	break;
          	
          case NA_Lambda1LocalScaledTag:
			*value_ptr += point1->FloatAttribute(Lambda1LocalScaledTag);
			*value_ptr += point2->FloatAttribute(Lambda1LocalScaledTag);
          	break;

          case NA_Lambda2LocalScaledTag:
			*value_ptr += point1->FloatAttribute(Lambda2LocalScaledTag);
			*value_ptr += point2->FloatAttribute(Lambda2LocalScaledTag);
          	break;

          // Nothing to do for sums, already used for other attributes
          case NA_Sum1Tag:
          	break;
        }
        
        // Update edge counter
        if (!counting_done) {
          *edge_count_ptr += 1;
          counting_done = true;
        }
      }
    }
  }
}

void WriteSegmentAttributes(FILE *fd, const LaserPoint &point,
                            const LaserPoint &ref_point)
{
  const unsigned char *attribute;
  int                 num_attributes, i;
  LaserPointTag       tag;
  
  // Write long segment number
  fprintf(fd, "%lld", point.LongAttribute(LongSegmentNumberTag));
  
  // The reference points serves to have the same sequence of attributes for every point
  num_attributes = ref_point.NumAttributes();
  for (i=0, attribute=ref_point.AttributeTags(); i<num_attributes; i++, attribute++) {
  	if (*attribute >= AverageReflectanceTag &&
	    *attribute < NoTag) {
	  tag = (LaserPointTag) *attribute;
	  if (!point.HasAttribute(tag)) {
	  	printf("Error: point without attribute %d\n", *attribute);
	  	exit(0);
	  }
	  
	  switch (AttributeType(tag)) {
	  	default:
	  	case IntegerAttributeType:
	  	  fprintf(fd, " %d", point.Attribute(tag)); break;
	  	case FloatAttributeType:
	  	  fprintf(fd, " %.3f", point.FloatAttribute(tag)); break;
	  	case DoubleAttributeType:
	  	  fprintf(fd, " %.6e", point.DoubleAttribute(tag)); break;
	  }    	
    }
  }
  fprintf(fd, "\n");
}

void WriteSegmentAttributes(FILE *fd, 
                            vector<LaserPoints>::iterator segment_points_in_tile_iterator,
							const LaserPoint &ref_point)
{
  LaserPoints::iterator point;
  
  for (point=segment_points_in_tile_iterator->begin();
       point!=segment_points_in_tile_iterator->end(); point++)
    WriteSegmentAttributes(fd, *point, ref_point);
}

void WriteNeighbourAttributes(FILE *fd,
                         vector<NeighbourAttributeTag> &neighbour_attributes,
                         vector< vector< pair<long long int, long long int> > >::iterator nbs_in_tile_iterator,
	                     long long int *counts_in_tile,
	                     long long int *edge_counts_in_tile,
						 double *values_in_tile,
						 Plane *planes_in_tile,
                         vector <int> &tile_numbers, 
                         vector< vector<long long int> >::iterator small_segments_in_tile_iterator,
						 vector< vector<long long int> > large_segments_in_block,
						 vector<LaserPoints>::iterator small_segment_points_in_tile_iterator,
						 vector<LaserPoints> large_segment_points_in_block)
{
  vector<NeighbourAttributeTag>::iterator neighbour_attribute;
  vector<int>::iterator           tile_iterator;
  long long int                   *count_ptr, *edge_count_ptr;
  double                          *value_ptr, *first_value_ptr,
                                  *local_eigenvalues[3], eigenvalue_sum;
  bool                            plane_done=false;
  Plane                           *plane_ptr;
  int                             iattr, num_attributes, i;
  vector< pair<long long int, long long int> >::iterator nb_in_tile;
  long long int                   segment;
  int                             tile_number, tile_index, segment_index;
  vector< vector<long long int> >::iterator large_segments_in_tile_iterator;
  vector<long long int>::iterator segment_in_tile_iterator;
  LaserPoints::iterator           ref_point=large_segment_points_in_block.begin()->begin(),
                                  segment_point;
  vector<LaserPoints>::iterator   large_segment_points_in_tile_iterator;
  bool debug=false;

  // Set reference point
  if (large_segment_points_in_block.begin()->size())
    ref_point = large_segment_points_in_block.begin()->begin();
  else
    ref_point = small_segment_points_in_tile_iterator->begin();

  num_attributes = neighbour_attributes.size();  	

  // Loop over all neighbour pairs
  for (nb_in_tile=nbs_in_tile_iterator->begin(), 
       count_ptr=counts_in_tile, edge_count_ptr=edge_counts_in_tile,
	   value_ptr=values_in_tile, plane_ptr=planes_in_tile;
	   nb_in_tile!=nbs_in_tile_iterator->end();
	   nb_in_tile++, count_ptr++, edge_count_ptr++, plane_ptr++) {
    plane_done = false;
    local_eigenvalues[0] = local_eigenvalues[1] = local_eigenvalues[2] = NULL;
	// Calculate all attributes	 
    for (iattr=0, neighbour_attribute=neighbour_attributes.begin(), first_value_ptr=value_ptr;
         iattr<num_attributes; iattr++, neighbour_attribute++, value_ptr++) {
      switch (*neighbour_attribute) {
        case NA_AverageHeightDifferenceTag:
          *value_ptr /= (double) (*edge_count_ptr);
          break;

        case NA_AverageAngleTag:
          if (*(value_ptr+1) == 0.0) {
//            printf("angle sum %.2f  angle count %.2f\n", *value_ptr,
//			       *(value_ptr+1));
			*value_ptr = 0.0;
          }
          else
            *value_ptr /= (*(value_ptr+1)); // value_ptr+1 contains the angle count
		  break;
		            	
        case NA_Lambda0Tag:
        case NA_Lambda1Tag:
        case NA_Lambda2Tag:
        case NA_Lambda0ScaledTag:
        case NA_Lambda1ScaledTag:
        case NA_Lambda2ScaledTag:
          if (!plane_done) {
          	plane_ptr->Recalculate();
          	plane_done = true;
          }
          break;

        case NA_Lambda0LocalScaledTag:
        case NA_Lambda1LocalScaledTag:
        case NA_Lambda2LocalScaledTag:
          switch (*neighbour_attribute) {
            case NA_Lambda0LocalScaledTag: local_eigenvalues[0] = value_ptr; break;
            case NA_Lambda1LocalScaledTag: local_eigenvalues[1] = value_ptr; break;
            case NA_Lambda2LocalScaledTag: local_eigenvalues[2] = value_ptr; break;
			default: break; 	
          }
          // Normalise when all three eigenvalues are collected
          if (local_eigenvalues[0] && local_eigenvalues[1] && local_eigenvalues[2]) {
          	eigenvalue_sum = *local_eigenvalues[0] + *local_eigenvalues[1] +
			                 *local_eigenvalues[2];
			if (eigenvalue_sum != 0.0) {
			  *local_eigenvalues[0] /= eigenvalue_sum;
			  *local_eigenvalues[1] /= eigenvalue_sum;
			  *local_eigenvalues[2] /= eigenvalue_sum;
			}
			else {
			  *local_eigenvalues[0] = 1.0 / 3.0;
			  *local_eigenvalues[1] = 1.0 / 3.0;
			  *local_eigenvalues[2] = 1.0 / 3.0;
			}
          }

        default: // Nothing to do for the other attributes
          break;
      }
    }
	
	// Write all neighbour attributes
	fprintf(fd, "%lld %lld", nb_in_tile->first, nb_in_tile->second);
	fprintf(fd, " %lld", *edge_count_ptr);
    for (iattr=0, neighbour_attribute=neighbour_attributes.begin(), value_ptr=first_value_ptr;
         iattr<num_attributes; iattr++, neighbour_attribute++, value_ptr++) {
      switch (*neighbour_attribute) {
        case NA_AverageHeightDifferenceTag:
          fprintf(fd, " %.2f", *value_ptr); break;
          	
        case NA_AverageAngleTag:
          fprintf(fd, " %.2f", *value_ptr); break;
          	
        case NA_Lambda0Tag:
          fprintf(fd, " %.4f", plane_ptr->FloatAttribute(PT_Lambda0));
          break;
          
        case NA_Lambda1Tag:
          fprintf(fd, " %.4f", plane_ptr->FloatAttribute(PT_Lambda1));
          break;
          
        case NA_Lambda2Tag:
          fprintf(fd, " %.4f", plane_ptr->FloatAttribute(PT_Lambda2));
          break;
          
        case NA_Lambda0ScaledTag:
          eigenvalue_sum = plane_ptr->FloatAttribute(PT_Lambda0) +
                           plane_ptr->FloatAttribute(PT_Lambda1) +
                           plane_ptr->FloatAttribute(PT_Lambda2);
          if (eigenvalue_sum != 0.0)
            fprintf(fd, " %.4f", plane_ptr->FloatAttribute(PT_Lambda0) / eigenvalue_sum);
          else
            fprintf(fd, " %.4f", 1.0 / 3.0);
          break;

        case NA_Lambda1ScaledTag:
          eigenvalue_sum = plane_ptr->FloatAttribute(PT_Lambda0) +
                           plane_ptr->FloatAttribute(PT_Lambda1) +
                           plane_ptr->FloatAttribute(PT_Lambda2);
          if (eigenvalue_sum != 0.0)
            fprintf(fd, " %.4f", plane_ptr->FloatAttribute(PT_Lambda1) / eigenvalue_sum);
          else
            fprintf(fd, " %.4f", 1.0 / 3.0);
          break;

        case NA_Lambda2ScaledTag:
          eigenvalue_sum = plane_ptr->FloatAttribute(PT_Lambda0) +
                           plane_ptr->FloatAttribute(PT_Lambda1) +
                           plane_ptr->FloatAttribute(PT_Lambda2);
          if (eigenvalue_sum != 0.0)
            fprintf(fd, " %.4f", plane_ptr->FloatAttribute(PT_Lambda2) / eigenvalue_sum);
          else
            fprintf(fd, " %.4f", 1.0 / 3.0);
          break;

          	
        case NA_Lambda0LocalScaledTag:
        case NA_Lambda1LocalScaledTag:
        case NA_Lambda2LocalScaledTag:
          fprintf(fd, " %.4f", *value_ptr); break;
          break;

//        case NA_Sum1Tag: // for debugging output only
//          fprintf(fd, " %d", (int) *value_ptr); break;
          
        default: // Ignore other attributes (edge counters are always written)
          break;
      }
    }
    fprintf(fd, "\n");
    
    // Write attributes of both segments
    for (i=0, segment=nb_in_tile->first; i<2; i++, segment=nb_in_tile->second) {

      // Get tile index
      tile_number = segment / 1000000;
   	  tile_iterator = std::find(tile_numbers.begin(), tile_numbers.end(),
   	                            tile_number);
      if (tile_iterator == tile_numbers.end()) {
      	printf("Error: Tile %d not found\n", tile_number);
      	exit(0);
      }
   	  tile_index = std::distance(tile_numbers.begin(), tile_iterator);

      // Get segment index
      segment_in_tile_iterator = std::find(small_segments_in_tile_iterator->begin(),
	                                       small_segments_in_tile_iterator->end(),
										   segment);
	  // Small segment
	  if (segment_in_tile_iterator != small_segments_in_tile_iterator->end()) {
	  	segment_index = std::distance(small_segments_in_tile_iterator->begin(),
		                              segment_in_tile_iterator);
		segment_point = small_segment_points_in_tile_iterator->begin() + segment_index;
	  }
	  else { // Large segment
        large_segments_in_tile_iterator = large_segments_in_block.begin() + tile_index;
        segment_in_tile_iterator = std::find(large_segments_in_tile_iterator->begin(),
		                                     large_segments_in_tile_iterator->end(),
											 segment);
		if (segment_in_tile_iterator == large_segments_in_tile_iterator->end()) {
		  printf("Large segment %lld not found\n", segment);
		  exit(0);
		}
	  	segment_index = std::distance(large_segments_in_tile_iterator->begin(),
		                              segment_in_tile_iterator);
		large_segment_points_in_tile_iterator = large_segment_points_in_block.begin() + tile_index;
		segment_point = large_segment_points_in_tile_iterator->begin() + segment_index;
	  }
      // Write attributes
      WriteSegmentAttributes(fd, *segment_point, *ref_point);
    }
	   	
  }
  
  // Re-initialise planes to de-allocate memory
  if (plane_done) {
    for (nb_in_tile=nbs_in_tile_iterator->begin(), plane_ptr=planes_in_tile;
	     nb_in_tile!=nbs_in_tile_iterator->end();
	     nb_in_tile++, plane_ptr++)
      plane_ptr->Initialise();
  }
}

void PrintPointAttributeNames(FILE *fd, LaserPoint &ref_point)
{
  const unsigned char *attribute;
  int                 num_attributes, i;
  LaserPointTag       tag;
  
  fprintf(fd, "# - Segment number\n"); 
  num_attributes = ref_point.NumAttributes();
  for (i=0, attribute=ref_point.AttributeTags(); i<num_attributes; i++, attribute++) {
  	if (*attribute >= AverageReflectanceTag && *attribute < NoTag) {
	  tag = (LaserPointTag) *attribute;
	  fprintf(fd, "# - %s\n", AttributeName(tag, true));
    }
  }
}

/*
--------------------------------------------------------------------------------
                         The main neighbourattributes function
--------------------------------------------------------------------------------
*/

void neighbourattributes(char *block_file,
                         vector<NeighbourAttributeTag> &attributes,
                         SegmentationParameters &parameters,
						 char *segment_output_file, char *neighbour_output_file)
{
  LaserBlock              block;
  LaserBlock::iterator    tile_container;
  LaserUnit::iterator     tile;
  LaserPoints::iterator   point, nb_point;
  int                     success, index, nb_index, neighbours_index,
                          tile_number, num_tiles, i, num_nbs;
  
  vector<int>                               tile_numbers;
  vector<int>::iterator                     tile_number_iterator;
  long long int                             segment, num_pts=0,
                                            num_large=0, num_small=0, nb_segment;
  vector< vector<long long int> >           segments_in_block,
                                            small_segments_in_block,
											large_segments_in_block;
  vector< vector<long long int> >::iterator segments_in_tile_iterator,
                                            small_segments_in_tile_iterator,
											large_segments_in_tile_iterator;
  vector<LaserPoints>                       segment_points_in_block,
                                            small_segment_points_in_block,
                                            large_segment_points_in_block;
  vector<LaserPoints>::iterator             segment_points_in_tile_iterator,
                                            small_segment_points_in_tile_iterator,
                                            large_segment_points_in_tile_iterator;
  vector<long long int *>                   large_counts_in_block, large_edge_counts_in_block;
  vector<long long int *>::iterator         large_counts_in_tile_iterator, large_edge_counts_in_tile_iterator;
  vector<double *>                          large_values_in_block;
  vector<double *>::iterator                large_values_in_tile_iterator;
  vector<Plane *>                           large_planes_in_block;
  vector<Plane *>::iterator                 large_planes_in_tile_iterator;
  vector<long long int>                     segments_in_tile;
  vector<long long int>::iterator           segment_in_tile;
  LaserPoints                               segment_points_in_tile;
  LaserPoints::iterator                     segment_point_in_tile, ref_point;
  long long int                             *small_counts_in_tile=NULL,
                                            *small_edge_counts_in_tile=NULL;
  double                                    *small_values_in_tile=NULL;
  Plane                                     *small_planes_in_tile=NULL;
  const unsigned char                       *attribute;
  TINEdges                                  *edges;
  TINEdges::iterator                        nbh_nodes;
  PointNumberList::iterator                 nbh_node;
  bool                                      large_segment1, large_segment2;
  pair<long long int, long long int>        neighbours;
  vector< vector< pair<long long int, long long int> > > small_nbs_in_block,
                                                         large_nbs_in_block;
  vector< vector< pair<long long int, long long int> > >::iterator nbs_in_tile_iterator,
                                                                   small_nbs_in_tile_iterator,
																   large_nbs_in_tile_iterator;
  vector< pair<long long int, long long int> >::iterator nb_pair_iterator;
  FILE *segment_fd, *neighbour_fd;
  vector<NeighbourAttributeTag>::iterator nb_attribute;
  
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

  // Open the output files and start writing the header lines
  segment_fd = fopen(segment_output_file, "w");
  if (!segment_fd) {
  	printf("Error opening segment output file %s\n", segment_output_file);
  	exit(0);
  }
  fprintf(segment_fd, "# Segment attributes (node attributes), one record per segment with:\n");
  neighbour_fd = fopen(neighbour_output_file, "w");
  if (!neighbour_fd) {
  	printf("Error opening neighbour output file %s\n", neighbour_output_file);
  	exit(0);
  }
  fprintf(neighbour_fd, "# Neighbouring segment pair attributes (edge attributes)\n");
  if (parameters.GrowingNeighbourhoodDefinition() == 1)
    fprintf(neighbour_fd, "# Neighbourhood defined by radius of %.2f m", 
	        parameters.GrowingRadius());
  else
    fprintf(neighbour_fd, "# Neighbourhood defined by %d nearest neighbours",
	        parameters.NumberOfNeighbours());
  fprintf(neighbour_fd, " in %dD\n", parameters.DistanceMetricDimension());
  fprintf(neighbour_fd, "# One record with neighbourship attributes, followed by two records\n");
  fprintf(neighbour_fd, "# with segment attributes of the neighbouring segments\n");
  fprintf(neighbour_fd, "# The neighbourship record contains:\n");
  fprintf(neighbour_fd, "# - Number of segment 1\n");
  fprintf(neighbour_fd, "# - Number of segment 2\n");
  for (nb_attribute=attributes.begin(); nb_attribute!=attributes.end(); nb_attribute++) {
  	switch (*nb_attribute) {
  	  case NA_NumberOfPointsTag:
		fprintf(neighbour_fd, "# - Number of points along common segment border\n"); break;
  	  case NA_AverageHeightDifferenceTag:
  	  	fprintf(neighbour_fd, "# - Average height difference along common border\n"); break;
  	  case NA_AverageAngleTag:
  	  	fprintf(neighbour_fd, "# - Average angle between local normal vectors\n"); break;
  	  case NA_Lambda0Tag:
  	  	fprintf(neighbour_fd, "# - Largest eigenvalue of 2nd order moments of points along common border\n"); break;
  	  case NA_Lambda1Tag:
  	  	fprintf(neighbour_fd, "# - Middle eigenvalue of 2nd order moments of points along common border\n"); break;
  	  case NA_Lambda2Tag:
  	  	fprintf(neighbour_fd, "# - Smallest eigenvalue of 2nd order moments of points along common border\n"); break;
  	  case NA_Lambda0ScaledTag:
  	  	fprintf(neighbour_fd, "# - Largest eigenvalue of 2nd order moments of points along common border scaled to 1\n"); break;
  	  case NA_Lambda1ScaledTag:
  	  	fprintf(neighbour_fd, "# - Middle eigenvalue of 2nd order moments of points along common border scaled to 1\n"); break;
  	  case NA_Lambda2ScaledTag:
  	  	fprintf(neighbour_fd, "# - Smallest eigenvalue of 2nd order moments of points along common border scaled to 1\n"); break;
  	  case NA_Lambda0LocalScaledTag:
  	  	fprintf(neighbour_fd, "# - Average local largest eigenvalue of 2nd order moments of points along common border scaled to 1\n"); break;
  	  case NA_Lambda1LocalScaledTag:
  	  	fprintf(neighbour_fd, "# - Average local middle eigenvalue of 2nd order moments of points along common border scaled to 1\n"); break;
  	  case NA_Lambda2LocalScaledTag:
  	  	fprintf(neighbour_fd, "# - Average local smallest eigenvalue of 2nd order moments of points along common border scaled to 1\n"); break;
  	}
  }
  fprintf(neighbour_fd, "# The segment attributes are:\n");
  
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
		// Store the first point of this segment to later retrieve segment attributes
		segment_points_in_tile.push_back(*point);
	  }
	}
	segments_in_block.push_back(segments_in_tile);
	segments_in_tile.erase(segments_in_tile.begin(), segments_in_tile.end());
	segment_points_in_block.push_back(segment_points_in_tile);
	segment_points_in_tile.ErasePoints();
	                              
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
    small_segment_points_in_block.push_back(LaserPoints());
    large_segment_points_in_block.push_back(LaserPoints());
  }
  // First collect the segments crossing tile boundaries
  num_tiles = tile_container->size();
  for (tile=tile_container->begin(), 
       segments_in_tile_iterator=segments_in_block.begin(),
	   segment_points_in_tile_iterator=segment_points_in_block.begin();
       tile!=tile_container->end();
	   tile++, segments_in_tile_iterator++, segment_points_in_tile_iterator++) {
	tile_number = tile->TileNumber();
    for (segment_in_tile=segments_in_tile_iterator->begin(),
	     segment_point_in_tile=segment_points_in_tile_iterator->begin();
	     segment_in_tile!=segments_in_tile_iterator->end();
		 segment_in_tile++, segment_point_in_tile++) {
	  // Check if this segments starts in another tile	
	  if (*segment_in_tile / 1000000 != tile_number) {
	  	// Look up the tile index
	  	tile_number_iterator = std::find(tile_numbers.begin(),
		                                 tile_numbers.end(),
										 *segment_in_tile / 1000000);
		index = std::distance(tile_numbers.begin(), tile_number_iterator);
		// Add the til;2e number to the large segment vector if that hasn't
		// been done so yet
		if (index < num_tiles) { // Check needed for incomplete blocks
		  if (std::find(large_segments_in_block[index].begin(),
		                large_segments_in_block[index].end(),
			  		    *segment_in_tile) ==
			  large_segments_in_block[index].end()) {
		    large_segments_in_block[index].push_back(*segment_in_tile);
		    large_segment_points_in_block[index].push_back(*segment_point_in_tile);
		  }
	    }
	  }
	}
  }
  // All segments that are not in a large segment number list, must be small
  // (only needed for determination of largest segment number inside tile
  //  and the statistics on small segments)
  for (tile=tile_container->begin(), 
       segments_in_tile_iterator=segments_in_block.begin(),
	   small_segments_in_tile_iterator=small_segments_in_block.begin(),
	   large_segments_in_tile_iterator=large_segments_in_block.begin(),
	   segment_points_in_tile_iterator=segment_points_in_block.begin(),
	   small_segment_points_in_tile_iterator=small_segment_points_in_block.begin(),
	   large_segment_points_in_tile_iterator=large_segment_points_in_block.begin();
       tile!=tile_container->end();
	   tile++, segments_in_tile_iterator++, small_segments_in_tile_iterator++,
	   large_segments_in_tile_iterator++,
	   segment_points_in_tile_iterator++, small_segment_points_in_tile_iterator++,
	   large_segment_points_in_tile_iterator++) {
	tile_number = tile->TileNumber();
	num_large += large_segments_in_tile_iterator->size();
    for (segment_in_tile=segments_in_tile_iterator->begin(),
	     segment_point_in_tile=segment_points_in_tile_iterator->begin();
	     segment_in_tile!=segments_in_tile_iterator->end();
		 segment_in_tile++, segment_point_in_tile++) {
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
			small_segment_points_in_tile_iterator->push_back(*segment_point_in_tile);
		  }		
	    }
	  }
    }
    // No longer need for the segment in tile list
    segments_in_tile_iterator->erase(segments_in_tile_iterator->begin(),
	                                 segments_in_tile_iterator->end());
    segment_points_in_tile_iterator->ErasePoints();
  }
  // No longer need for the segment in block list
  segments_in_block.erase(segments_in_block.begin(),
                          segments_in_block.end());
  segment_points_in_block.erase(segment_points_in_block.begin(),
                                segment_points_in_block.end());

  // Output some check statistics
  printf("Block %s consists of %d tiles with %lld points\n",
         block.Name(), (int) tile_container->size(), num_pts);
  for (small_segments_in_tile_iterator=small_segments_in_block.begin();
	   small_segments_in_tile_iterator!=small_segments_in_block.end();
       small_segments_in_tile_iterator++)
    num_small += small_segments_in_tile_iterator->size();
  printf("with %lld segments inside tiles and %lld segments across tile boundaries\n",
         num_small, num_large);

  // Now determine the neighbouring segments. These will be called large
  // neighbours if both segments cross a tile boundary, and small neighbours
  // otherwise.

  printf("Collecting neighbouring segments per tile\n");
  for (tile=tile_container->begin(); tile!=tile_container->end(); tile++) {
    small_nbs_in_block.push_back(vector< pair<long long int, long long int> >());
    large_nbs_in_block.push_back(vector< pair<long long int, long long int> >());
  }


  for (tile=tile_container->begin(); tile!=tile_container->end(); tile++) {
    printf("Tile (%3d, %3d)\r", tile->TileRow(), tile->TileColumn());

    // Read the point data
    if (!tile->Read(tile->PointFile(), false)) {
      fprintf(stderr, "Error reading laser points from file %s\n",
              tile->PointFile());
      exit(0);
    }

    // Derive point neighbourhood edges
    if (parameters.GrowingNeighbourhoodDefinition() == 0)
      edges = tile->DeriveEdges(parameters);
    else
      edges = tile->DeriveFDNEdges(parameters);

    // Collect all segment neighbour pairs in this tile
    for (point=tile->begin(), nbh_nodes=edges->begin(), i=0;
	     point!=tile->end(); point++, nbh_nodes++, i++) {
      segment = point->LongSegmentNumber();
      if (segment == INT_MIN) continue; // No segment number
      // Check if this is a large segment
      tile_number = segment / 1000000;
	  tile_number_iterator = std::find(tile_numbers.begin(), tile_numbers.end(),
									   tile_number);
	  index = std::distance(tile_numbers.begin(), tile_number_iterator);
	  large_segment1 = (std::find(large_segments_in_block[index].begin(),
		                          large_segments_in_block[index].end(),
			  		              segment) !=
			           large_segments_in_block[index].end());
      for (nbh_node=nbh_nodes->begin(); nbh_node!=nbh_nodes->end(); nbh_node++) {
      	nb_point = tile->begin() + nbh_node->Number();
      	nb_segment = nb_point->LongSegmentNumber();
      	if (nb_segment == INT_MIN) continue; // No segment number
      	if (segment == nb_segment) continue; // Points belong to same segment
        tile_number = nb_segment / 1000000;
	    tile_number_iterator = std::find(tile_numbers.begin(), tile_numbers.end(),
								         tile_number);
	    nb_index = std::distance(tile_numbers.begin(), tile_number_iterator);
      	// Check whether this segment is large
        large_segment2 = (std::find(large_segments_in_block[nb_index].begin(),
		                            large_segments_in_block[nb_index].end(),
			  		                nb_segment) !=
			              large_segments_in_block[nb_index].end());
      	// Store the neighbour pair if not yet done so
      	if (large_segment1 && !large_segment2) { // Store in the tile of segment 2
      	  neighbours.first  = nb_segment;
      	  neighbours.second = segment;
      	  neighbours_index  = nb_index;
      	}
      	else if (!large_segment1 && large_segment2) { // Store in the tile of segment 1
      	  neighbours.first  = segment;
      	  neighbours.second = nb_segment;
      	  neighbours_index = index;
      	}
      	else { // Store in the tile of the segment with the smallest number
      	  if (segment < nb_segment) {
      	    neighbours.first  = segment;
      	    neighbours.second = nb_segment;
      	    neighbours_index = index;
      	  }
      	  else {
      	    neighbours.first  = nb_segment;
      	    neighbours.second = segment;
      	    neighbours_index = nb_index;
      	  }
      	}
      	if (large_segment1 && large_segment2)
      	  nbs_in_tile_iterator = large_nbs_in_block.begin() + neighbours_index;
      	else
      	  nbs_in_tile_iterator = small_nbs_in_block.begin() + neighbours_index;
      	if (std::find(nbs_in_tile_iterator->begin(),
		              nbs_in_tile_iterator->end(), neighbours) == 
			nbs_in_tile_iterator->end())
		  nbs_in_tile_iterator->push_back(neighbours);
      }
	}
	                              
	// Erase points and edges
	tile->EraseNeighbourhoodEdges();
	tile->EraseTIN();
	tile->ErasePoints();
  }
      
  // Statistics on neighbouring segments
  num_small = 0;
  num_large = 0;
  for (tile=tile_container->begin(), 
       small_segments_in_tile_iterator=small_segments_in_block.begin(),
       large_segments_in_tile_iterator=large_segments_in_block.begin(),
       small_nbs_in_tile_iterator=small_nbs_in_block.begin(),
       large_nbs_in_tile_iterator=large_nbs_in_block.begin();
	   tile!=tile_container->end();
	   tile++, small_segments_in_tile_iterator++, large_segments_in_tile_iterator++,
	   small_nbs_in_tile_iterator++, large_nbs_in_tile_iterator++) {
/*
    printf("Tile (%3d, %3d): %d small, %d large segments; %d small, %d large neighbours\n",
	       tile->TileRow(), tile->TileColumn(),
	       small_segments_in_tile_iterator->size(),
	       large_segments_in_tile_iterator->size(),
		   small_nbs_in_tile_iterator->size(),
		   large_nbs_in_tile_iterator->size());
*/
	num_large += large_nbs_in_tile_iterator->size();
	num_small += small_nbs_in_tile_iterator->size();
  }
  printf("In total %lld small and %lld large neighbours\n", num_small, num_large);

  // Create count, value and plane arrays for all large neighbour pairs
  for (tile=tile_container->begin(), 
	   large_nbs_in_tile_iterator=large_nbs_in_block.begin();
       tile!=tile_container->end();
	   tile++, large_nbs_in_tile_iterator++) {
	large_counts_in_block.push_back(
	  (long long int *) calloc(large_nbs_in_tile_iterator->size()+1,
	                           sizeof(long long int)));
	large_edge_counts_in_block.push_back(
	  (long long int *) calloc(large_nbs_in_tile_iterator->size()+1,
	                           sizeof(long long int)));
	large_values_in_block.push_back(
	  (double *) calloc(large_nbs_in_tile_iterator->size()+1,
	                    sizeof(double) * attributes.size()));
	large_planes_in_block.push_back(
	  (Plane *) calloc(large_nbs_in_tile_iterator->size()+1, sizeof(Plane)));
  }

  // Processing loop over all tiles
  printf("Attribute calculations and storing results of neighbours inside tiles\n");
  ref_point = small_segment_points_in_block.begin()->begin();
  PrintPointAttributeNames(segment_fd, *ref_point);
  PrintPointAttributeNames(neighbour_fd, *ref_point);

  for (tile=tile_container->begin(), 
	   small_nbs_in_tile_iterator=small_nbs_in_block.begin(),
	   small_segments_in_tile_iterator=small_segments_in_block.begin(),
	   small_segment_points_in_tile_iterator=small_segment_points_in_block.begin(),
	   large_segment_points_in_tile_iterator=large_segment_points_in_block.begin();
       tile!=tile_container->end();
	   tile++, small_nbs_in_tile_iterator++, small_segments_in_tile_iterator++, 
	   small_segment_points_in_tile_iterator++,
	   large_segment_points_in_tile_iterator++) {

    printf("Tile (%3d, %3d)\r", tile->TileRow(), tile->TileColumn());
		
    // Create count, value and plane arrays for all segments within tile only
    num_nbs = small_nbs_in_tile_iterator->size();
    small_counts_in_tile = (long long int *)
	  realloc(small_counts_in_tile, num_nbs * sizeof(long long int));
    small_edge_counts_in_tile = (long long int *)
	  realloc(small_edge_counts_in_tile, num_nbs * sizeof(long long int));
    small_values_in_tile = (double *)
	  realloc(small_values_in_tile, num_nbs * sizeof(double) *
	          attributes.size());
    small_planes_in_tile = (Plane *)
	  realloc(small_planes_in_tile, num_nbs * sizeof(Plane));
    
	// Initialise all arrays
	memset(small_counts_in_tile, 0, num_nbs * sizeof(long long int));
	memset(small_edge_counts_in_tile, 0, num_nbs * sizeof(long long int));
	memset(small_values_in_tile, 0, num_nbs * sizeof(double) *attributes.size());
	memset(small_planes_in_tile, 0, num_nbs * sizeof(Plane));
    
    // Read tile points
    if (!tile->Read(tile->PointFile(), false)) {
      fprintf(stderr, "Error reading laser points from file %s\n",
              tile->PointFile());
      exit(0);
    }

    // Update counters, values etc for all points
	UpdateAttributeSums(attributes, tile, tile_numbers, large_segments_in_block,
	                    small_nbs_in_tile_iterator, large_nbs_in_block, 
	                    small_counts_in_tile, small_edge_counts_in_tile,
						small_values_in_tile, small_planes_in_tile, 
						large_counts_in_block, large_edge_counts_in_block,
						large_values_in_block, large_planes_in_block,
						parameters);

    // Write attributes of small neighbours
    WriteNeighbourAttributes(neighbour_fd, attributes,
                             small_nbs_in_tile_iterator,
	                         small_counts_in_tile, small_edge_counts_in_tile,
						     small_values_in_tile, small_planes_in_tile,
                             tile_numbers, 
                             small_segments_in_tile_iterator,
						     large_segments_in_block,
						     small_segment_points_in_tile_iterator,
						     large_segment_points_in_block);

    // Write attributes of small and large segments starting in this tile
    WriteSegmentAttributes(segment_fd, small_segment_points_in_tile_iterator, *ref_point);
    WriteSegmentAttributes(segment_fd, large_segment_points_in_tile_iterator, *ref_point);
    
	// Erase points and edges
	tile->EraseNeighbourhoodEdges();
	tile->EraseTIN();
	tile->ErasePoints();
  }

  // Write attributes of large neighbours
  printf("Attribute calculations and storing results of neighbours across tiles\n");
  for (tile=tile_container->begin(), 
	   large_nbs_in_tile_iterator=large_nbs_in_block.begin(),
	   
	   large_counts_in_tile_iterator=large_counts_in_block.begin(),
	   large_edge_counts_in_tile_iterator=large_edge_counts_in_block.begin(),
	   large_values_in_tile_iterator=large_values_in_block.begin(),
	   large_planes_in_tile_iterator=large_planes_in_block.begin(),	   
	   
	   small_segments_in_tile_iterator=small_segments_in_block.begin(),
	   small_segment_points_in_tile_iterator=small_segment_points_in_block.begin();
       tile!=tile_container->end();
	   tile++, large_nbs_in_tile_iterator++, small_segments_in_tile_iterator++, 
	   large_counts_in_tile_iterator++, large_edge_counts_in_tile_iterator++,
	   large_values_in_tile_iterator++, large_planes_in_tile_iterator++,
	   small_segment_points_in_tile_iterator++) {

    printf("Tile (%3d, %3d)\r", tile->TileRow(), tile->TileColumn());
    WriteNeighbourAttributes(neighbour_fd, attributes,
                             large_nbs_in_tile_iterator,
	                         *large_counts_in_tile_iterator, *large_edge_counts_in_tile_iterator,
						     *large_values_in_tile_iterator, *large_planes_in_tile_iterator,
                             tile_numbers, 
                             small_segments_in_tile_iterator,
						     large_segments_in_block,
						     small_segment_points_in_tile_iterator,
						     large_segment_points_in_block);
  }

  // Close output files
  fclose(segment_fd); fclose(neighbour_fd);
}
