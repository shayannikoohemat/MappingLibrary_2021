
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


/*
--------------------------------------------------------------------------------
 Segmentation of a block of laser scanning data.
 First all tiles are segmented individually.
 Then, segments are merged across tile boundaries.
 Segment numbers are stored in SegmentNumberTag. The start of the segment
 is stored in SegmentTileNumberTag. Together the attributes provide unique
 segment numbers in a block of up to 1000 x 1000 tiles.

 Initial creation:
 Author : George Vosselman
 Date   : 04-06-2013

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
#include <time.h>
#include "LaserBlock.h"
#include "TINEdges.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(const char *, const char *, int *);

/*
--------------------------------------------------------------------------------
                         The main segmentlaser function
--------------------------------------------------------------------------------
*/

extern "C" void timer_start(clock_t *time1)
{
  *time1 = clock();
}

extern "C" void timer_end(clock_t time1, char *string)
{
  clock_t time2;
  time2 = clock();
  printf("Time used for %s: %5.2f minutes\n", string,
         (double) (time2 - time1) / (60 * CLOCKS_PER_SEC));
}

void blocksurfaces(char *block_file,
                  char *appendix, char *output_directory,
                  bool output_meta_data, bool overwrite,
                  const SegmentationParameters &parameters,
				  double border_zone_width, bool tilefit,
				  double minimum_common_border_length)
{
  char                  *directory, *filter, *filename, *newname;
  int                   icon, fileclass, point_index, i, num_merged, index,
                        num_pts1, num_pts2, num_segments1;
  LaserBlock            block;
  LaserBlock::iterator  tile_container;
  LaserUnit::iterator   tile;
  LaserSubUnit          newtile, *nb_tile;
  clock_t               start;
  DataBoundsLaser       border_zone;
  LaserPoints           border_points, nb_border_points;
  TINEdges              *edges;
  LaserPoints::iterator point1, point2, point3;
  PointNumberList       nbh;
  PointNumberList::iterator nb, nb3;
  LaserPointTag         segment_tag;
  std::vector<int>           segment_numbers;
  std::vector<int>::iterator segment_number;
  std::vector<bool>          validities;
  std::vector<long long>     long_segment_numbers, new_segment_numbers,
                             tested_segments;
  std::vector<long long>::iterator long_segment_number, new_segment_number,
                                   first_segment_number, last_segment_number;
  int                              segment_number1, segment_number2,
                                   segment_number3, tile_number,
								   new_tile_number;
  long long                        long_segment_number1, long_segment_number2,
                                   long_segment_number3, new_segment_number1,
								   new_segment_number2;
  Plane                      separation_plane;
  Planes                     planes;
  Planes::iterator           plane1, plane2;
  Line3D                     intersection_line;
  Position3D                 pos;
  bool                       merge_segments, valid, found;
  double                     angle, pi = 4.0 * atan(1.0), scalar, dist,
                             min_scalar1, max_scalar1, min_scalar2, max_scalar2;
  std::vector<std::pair<long long,long long> >  changed_numbers;	
  std::vector<std::pair<long long,long long> >::iterator  changed_number;	
  std::pair<long long, long long> old_and_new_number;
  
  // Set up a laser block
  if (!block.Create(block_file, &fileclass)) {
    fprintf(stderr, "Error reading block data file %s\n", filename);
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
  	printf("Error: block %s contains no tiles\n", block.Name());
  	exit(0);
  }


  // Try to merge segments across tile boundaries
  segment_tag = parameters.SegmentAttribute();
  FILE *fd = fopen("merged_segments.txt", "w");
  for (tile=tile_container->begin(); tile!=tile_container->end(); tile++) {

  	for (i=0; i<2; i++) {
  	  // See if there is a tile with a higher row or column number
	  if (i == 0) nb_tile = block.Tile(tile->TileRow()+1, tile->TileColumn());
  	  else nb_tile = block.Tile(tile->TileRow(), tile->TileColumn()+1);
  	  if (nb_tile == NULL) continue;
	         
  	  // Read points and add tile numbers
  	  if (tile->empty()) {
        if (!tile->Read(tile->PointFile(), false)) {
          fprintf(stderr, "Error reading laser points from file %s\n",
                  tile->PointFile());
          exit(0);
        }
        tile->SetAttribute(SegmentStartTileNumberTag,
  	                       tile->TileRow() * 1000 + tile->TileColumn());
 		  	  
      }
      if (!nb_tile->Read(nb_tile->PointFile(), false)) {
        fprintf(stderr, "Error reading laser points from file %s\n",
                nb_tile->PointFile());
        exit(0);
      }
      nb_tile->SetAttribute(SegmentStartTileNumberTag,
	                        nb_tile->TileRow() * 1000 + nb_tile->TileColumn());

      // Select points near border
      border_zone.Initialise();
      if (i == 0) {
      	border_zone.SetMinimumY(tile->TileBounds().Minimum().Y() -
		                        border_zone_width / 2.0);
      	border_zone.SetMaximumY(tile->TileBounds().Minimum().Y() +
		                        border_zone_width / 2.0);
      }
      else {
      	border_zone.SetMinimumX(tile->TileBounds().Maximum().X() -
		                        border_zone_width / 2.0);
      	border_zone.SetMaximumX(tile->TileBounds().Maximum().X() +
		                        border_zone_width / 2.0);
      }
      border_points.ErasePoints();
      tile->Select(border_points, border_zone);
      if (border_points.empty()) continue;
      nb_border_points.ErasePoints();
      nb_tile->Select(nb_border_points, border_zone);
      if (nb_border_points.empty()) continue;
      	  
      // Create a sorted vector of all different segment numbers,
      // fit all planes and create long segment numbers including tile number
      segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
      border_points.AttributeValues(segment_tag, segment_numbers);
      std::sort(segment_numbers.begin(), segment_numbers.end());
      for (segment_number=segment_numbers.begin();
           segment_number!=segment_numbers.end(); segment_number++) {
        if (tilefit)
          planes.push_back(tile->FitPlane(*segment_number, *segment_number, segment_tag));
        else
          planes.push_back(border_points.FitPlane(*segment_number, *segment_number, segment_tag));
	    validities.push_back((planes.end()-1)->NumberOfPoints() > 2);
	    long_segment_numbers.push_back((long long) tile->TileRow() * 1000000000 +
		                               (long long) tile->TileColumn() * 1000000 +
									   *segment_number);
      }
      num_segments1 = segment_numbers.size();

      // Add segments of neighbouring tile
      segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
      nb_border_points.AttributeValues(segment_tag, segment_numbers);
      std::sort(segment_numbers.begin(), segment_numbers.end());
      for (segment_number=segment_numbers.begin();
           segment_number!=segment_numbers.end(); segment_number++) {
        if (tilefit)
		  planes.push_back(nb_tile->FitPlane(*segment_number, *segment_number, segment_tag));
        else
		  planes.push_back(nb_border_points.FitPlane(*segment_number, *segment_number, segment_tag));
	    validities.push_back((planes.end()-1)->NumberOfPoints() > 2);
	    long_segment_numbers.push_back((long long) nb_tile->TileRow() * 1000000000 +
		                               (long long) nb_tile->TileColumn() * 1000000 +
									   *segment_number);
      }
      nb_tile->ErasePoints();

      // Copy the vector of long segment numbers
      new_segment_numbers.insert(new_segment_numbers.begin(),
                                 long_segment_numbers.begin(),
								 long_segment_numbers.end());

      // Output of statistics
      num_merged = 0;
      printf("Merging tiles (%3d, %3d) and (%3d, %3d), %6d points in border   \r",
             tile->TileRow(), tile->TileColumn(),  	  
             nb_tile->TileRow(), nb_tile->TileColumn(),
             border_points.size() + nb_border_points.size());

      // The tile wise option assumes that surfaces are completely planar
      // Pairs of planes (one from each tile) are intersected with the vertical plane
      // separating the tiles. If points of both planes are found close to the
      // intersection line and in an overlapping range, the segments can be 
      // merged.
      if (tilefit) {
      	// Determine plane separating the two tiles
      	if (i == 0) {
      	  separation_plane.SetNormal(Vector3D(0.0, 1.0, 0.0));
      	  separation_plane.SetDistance(tile->TileBounds().Minimum().Y());
      	}
      	else {
      	  separation_plane.SetNormal(Vector3D(1.0, 0.0, 0.0));
      	  separation_plane.SetDistance(tile->TileBounds().Maximum().X());
      	}
      	
      	// Loop over all plane pairs
      	for (plane1=planes.begin(), first_segment_number=new_segment_numbers.begin();
		     plane1<planes.begin()+num_segments1; plane1++, first_segment_number++) {

          // Get the corresponding new segment number (which initially is the
          // original number
          long_segment_number1 = *first_segment_number;
          long_segment_number = std::find(long_segment_numbers.begin(),
		                                  long_segment_numbers.end(),
	                                      long_segment_number1);
	      index = std::distance(long_segment_numbers.begin(),
		                        long_segment_number);
	      new_segment_number1 = *(new_segment_numbers.begin() + index);

          // Calculate intersection line of plane 1 with separation plane
          Intersect2Planes(*plane1, separation_plane, intersection_line);

          // Test if segment 1 has points close to intersection line
          min_scalar1 = 1e10;
          max_scalar1 = -1e10;
          num_pts1 = 0;
          for (point1=border_points.begin(); point1!=border_points.end();
		       point1++) {
			// Check if the point belongs to the current segment
			if (point1->Attribute(SegmentNumberTag) != plane1->Number())
			  continue;
			  
			// Check the distance
			dist = intersection_line.DistanceToPoint(point1->Position3DRef());
			if (dist < parameters.MaxDistPointToOtherSegment()) {
				
			  // Update scalar range
			  scalar = intersection_line.Scalar(point1->Position3DRef());
			  if (scalar < min_scalar1) min_scalar1 = scalar;
			  if (scalar > max_scalar1) max_scalar1 = scalar;
			  num_pts1++;
			}
		  }
		  if (num_pts1 < parameters.MinNumberPointsBothPlanes()) continue;
          
          // Loop over all planes of the other tile
      	  for (plane2=planes.begin()+num_segments1,
			   last_segment_number=long_segment_numbers.begin()+num_segments1;
			   plane2!=planes.end(); plane2++, last_segment_number++) {
		    
            // Get the corresponding new segment number (which initially is the
            // original number
            long_segment_number2 = *last_segment_number;
            long_segment_number = std::find(long_segment_numbers.begin(),
		                                    long_segment_numbers.end(),
	                                        long_segment_number2);
	        index = std::distance(long_segment_numbers.begin(),
		                          long_segment_number);
	        new_segment_number2 = *(new_segment_numbers.begin() + index);

			// Test normal vector directions
            angle = Angle(plane1->Normal(), plane2->Normal());
            if (angle > pi/2.0) angle -= pi;
            if (fabs(angle) > parameters.MaxAnglePlanes()) continue;
           
            // Test if segment 2 has points close to intersection line
            min_scalar2 = 1e10;
            max_scalar2 = -1e10;
            num_pts2 = 0;
            for (point2=nb_border_points.begin(); point2!=nb_border_points.end();
		         point2++) {
			  // Check if the point belongs to the current segment
			  if (point2->Attribute(SegmentNumberTag) != plane2->Number())
			    continue;
			  
			  // Check the distance
			  dist = intersection_line.DistanceToPoint(point2->Position3DRef());
			  if (dist < parameters.MaxDistPointToOtherSegment()) {
				
			    // Update scalar range
			    scalar = intersection_line.Scalar(point2->Position3DRef());
			    if (scalar < min_scalar2) min_scalar2 = scalar;
			    if (scalar > max_scalar2) max_scalar2 = scalar;
			    num_pts2++;
			  }
		    }
		    if (num_pts2 < parameters.MinNumberPointsBothPlanes()) continue;
            
            // Test whether the scalar ranges on the intersection line overlap
            if (min_scalar2 > min_scalar1) min_scalar1 = min_scalar2;
            if (max_scalar2 < max_scalar2) max_scalar1 = max_scalar2;
            if (max_scalar1 - min_scalar1 < minimum_common_border_length)
			  continue;
            
            // Register the equivalence of these two planes
      	    num_merged++;
      	    printf("Merge %d between %lld and %lld\n", num_merged,
				   long_segment_number1, long_segment_number2);
            fprintf(fd, "%6lld %6lld     %6lld %6lld\n",
					long_segment_number1/1000000,
					long_segment_number1%1000000,
					long_segment_number2/1000000,
					long_segment_number2%100000);
			// Replace all occurrences of segment number 2 by segment number 1
			for (long_segment_number=new_segment_numbers.begin();
			     long_segment_number!=new_segment_numbers.end();
				 long_segment_number++)
			  if (*long_segment_number == new_segment_number2)
				*long_segment_number = new_segment_number1;

		  }
      	}
      	
      	// Clean up
      	nb_border_points.ErasePoints();
      }
	  
	  // The default option performs a local analysis of surface smoothness.
	  // This is more suitable for non-planar but smooth surfaces.
	  else {
	  
	    // Merge point sets
        border_points.insert(border_points.end(), nb_border_points.begin(),
	                         nb_border_points.end());
	    nb_border_points.ErasePoints(); // No longer needed
	                       
        // Derive edges
        edges = border_points.DeriveEdges(parameters);
      
        // Check all neighbourhoods
        for (point1=border_points.begin(), point_index=0;
	         point1!=border_points.end(); point1++, point_index++) {
          if (point1->HasAttribute(segment_tag)) {
    	
            // Get segment number
            segment_number1 = point1->Attribute(segment_tag);
            long_segment_number1 =
		      (long long) point1->Attribute(SegmentStartTileNumberTag) * 1000000 +
		      (long long) segment_number1;

            // Get the corresponding new segment number (which initially is the
            // original number
            long_segment_number = std::find(long_segment_numbers.begin(),
		                                    long_segment_numbers.end(),
	                                        long_segment_number1);
	        index = std::distance(long_segment_numbers.begin(),
		                          long_segment_number);
	        new_segment_number1 = *(new_segment_numbers.begin() + index);
	  
            // Get plane parameters through corresponding location in planes vector
            plane1 = planes.begin() + index;
      
            // Check if the plane is valid
            valid = *(validities.begin() + index);
	        if (!valid) continue;

            // Get neighbouring nodes
            nbh = border_points.Neighbourhood(PointNumber(point_index),
                                parameters.MergingNeighbourhoodRadius(),
			    			    edges->TINEdgesRef(),
                                parameters.DistanceMetricDimension() == 2,
                                parameters.MergingNeighbourhoodDefinition() == 0);

	        // Loop over all neighbouring points
            for (nb=nbh.begin(); nb!=nbh.end(); nb++) {
          	  point2 = border_points.begin() + nb->Number();
          	
          	  // Ignore this point if it is from the same tile
          	  if (point1->Attribute(SegmentStartTileNumberTag) ==
			      point2->Attribute(SegmentStartTileNumberTag)) continue;
			    
      	      if (point2->HasAttribute(segment_tag)) {
      	        // Get the new segment number of this point
      	        segment_number2= point2->Attribute(segment_tag);
                long_segment_number2 =
		          (long long) point2->Attribute(SegmentStartTileNumberTag) * 1000000 +
		          (long long) segment_number2;
                long_segment_number = std::find(long_segment_numbers.begin(),
			                                    long_segment_numbers.end(),
	                                          long_segment_number2);
	            index = std::distance(long_segment_numbers.begin(),
			                          long_segment_number);
	            new_segment_number2 = *(new_segment_numbers.begin() + index);
      	        if (new_segment_number1 != new_segment_number2) {

          	  	  // Only test adjacent segment once
                  if (std::find(tested_segments.begin(), tested_segments.end(),
			                   long_segment_number2) == tested_segments.end()) {
			        merge_segments = true;

    			    // Get the plane parameters
                    plane2 = planes.begin() + index;

                    // Check if the plane has sufficient points
                    merge_segments = *(validities.begin() + index);

                    // Test normal vector directions
                    if (merge_segments) {
                      angle = Angle(plane1->Normal(), plane2->Normal());
                      if (angle > pi/2.0) angle -= pi;
                      if (fabs(angle) > parameters.MaxAnglePlanes())
					    merge_segments = false;
                    }

                    // Test point to plane distances in neighbourhood
                    if (merge_segments) {
              	      num_pts1 = num_pts2 = 0;
                      for (nb3=nbh.begin(); nb3!=nbh.end() && merge_segments; nb3++) {
      	                point3 = border_points.begin() + nb3->Number();
      	                if (point3->HasAttribute(segment_tag)) {
      	           	      segment_number3 = point3->Attribute(segment_tag);
                          long_segment_number3 =
		                    (long long) point3->Attribute(SegmentStartTileNumberTag) * 1000000 +
		                    (long long) segment_number3;

      	          	      if (long_segment_number3 == long_segment_number1) {
      	          	        if (fabs(plane2->Distance(point3->Position3DRef())) >
						        parameters.MaxDistPointToOtherSegment()) {
						      merge_segments = false;
					        }
					        else num_pts2++;
      	          	      }
      	          	      else if (long_segment_number3 == long_segment_number2) {
      	          	        if (fabs(plane1->Distance(point3->Position3DRef())) >
						        parameters.MaxDistPointToOtherSegment()) {
						      merge_segments = false;
					        }
					        else num_pts1++;
      	          	      }
      	                }
      	              }
      	            }
      	      
      	            // Check if we've tested sufficient points for both planes
      	            if (merge_segments) {
      	      	      if (num_pts1 < parameters.MinNumberPointsBothPlanes() ||
					      num_pts2 < parameters.MinNumberPointsBothPlanes())
				        merge_segments = false;
      	            }
      	      
      	            // Avoid testing this segment again for this point
      	            tested_segments.push_back(long_segment_number2);

      	            // Merge the segments
      	            if (merge_segments) {
      	      	      num_merged++;
      	      	      printf("Merge %d between %lld and %lld\n", num_merged,
					  	     long_segment_number1, long_segment_number2);
                      fprintf(fd, "%6lld %6lld     %6lld %6lld\n",
					          long_segment_number1/1000000,
						  	  long_segment_number1%1000000,
							  long_segment_number2/1000000,
							  long_segment_number2%100000);
			          // Replace all occurrences of segment number 2 by
                      // segment number 1
			          for (long_segment_number=new_segment_numbers.begin();
				           long_segment_number!=new_segment_numbers.end();
						   long_segment_number++)
				        if (*long_segment_number == new_segment_number2)
				          *long_segment_number = new_segment_number1;
				    }
                  }
      	        }
      	      }
            }
            // Erase list of compared neighbouring segments
            tested_segments.erase(tested_segments.begin(), tested_segments.end());
          }
        }

	    // Clean up
        edges->Erase();
	    
  	  }  // End of local smoothness test 	  
	  
      // Save changed segment numbers
      for (long_segment_number = long_segment_numbers.begin(),
	       new_segment_number = new_segment_numbers.begin();
		   long_segment_number != long_segment_numbers.end();
		   long_segment_number++, new_segment_number++) {
		if (*long_segment_number != *new_segment_number) {
		  if (*new_segment_number < *long_segment_number) {
		    old_and_new_number.first  = *long_segment_number;
		    old_and_new_number.second = *new_segment_number;
		  }
		  else {
		    old_and_new_number.first  = *new_segment_number;
		    old_and_new_number.second = *long_segment_number;
	      }
	      changed_numbers.push_back(old_and_new_number);
	    }
	  }


      // Clean up
      border_points.ErasePoints();
      planes.Erase();
      validities.erase(validities.begin(), validities.end());
      long_segment_numbers.erase(long_segment_numbers.begin(),
	                             long_segment_numbers.end());
      new_segment_numbers.erase(new_segment_numbers.begin(),
	                            new_segment_numbers.end());
    }
    tile->ErasePoints();
  }
  fclose(fd);
  
  // Make a sorted vector of all numbers involved in the changes
  fd = fopen("changed_numbers.txt", "w");
  for (changed_number=changed_numbers.begin();
       changed_number!=changed_numbers.end(); changed_number++) {
    fprintf(fd, "%lld %lld\n", changed_number->first, changed_number->second);
    if(std::find(long_segment_numbers.begin(), long_segment_numbers.end(),
			     changed_number->first) == long_segment_numbers.end())
	  long_segment_numbers.push_back(changed_number->first);
    if(std::find(long_segment_numbers.begin(), long_segment_numbers.end(),
			     changed_number->second) == long_segment_numbers.end())
	  long_segment_numbers.push_back(changed_number->second);
  }
  std::sort(long_segment_numbers.begin(), long_segment_numbers.end());
  fclose(fd);

  // Copy the vector of long segment numbers
  new_segment_numbers.insert(new_segment_numbers.begin(),
                             long_segment_numbers.begin(),
    					     long_segment_numbers.end());

  // Combine all changes to the final look-up table
  for (changed_number=changed_numbers.begin();
       changed_number!=changed_numbers.end(); changed_number++) {
    if (changed_number->first > changed_number->second) {
      long_segment_number1 = changed_number->first;
      long_segment_number2 = changed_number->second;
    }
    else {
      long_segment_number1 = changed_number->second;
      long_segment_number2 = changed_number->first;
    }
    // Look up smaller segment number in original point number list
    long_segment_number = std::find(long_segment_numbers.begin(),
	                                long_segment_numbers.end(),
									long_segment_number2);
	// Get the index of this point
    index = std::distance(long_segment_numbers.begin(), long_segment_number);
    // Get the new segment number at the same index
    new_segment_number = new_segment_numbers.begin() + index;
    
    // Look up larger segment number in original point number list
    long_segment_number = std::find(long_segment_numbers.begin(),
	                                long_segment_numbers.end(),
									long_segment_number1);
	// Get the index of this point
    index = std::distance(long_segment_numbers.begin(), long_segment_number);
    // Get the corresponding new segment number location
    long_segment_number = new_segment_numbers.begin() + index;
    // Set all occurrences of the highest new number to the lowest new number
    if (*long_segment_number > *new_segment_number) {
      new_segment_number1 = *long_segment_number;
      new_segment_number2 = *new_segment_number;
    }
    else {
      new_segment_number1 = *new_segment_number;
      new_segment_number2 = *long_segment_number;
    }
    for (new_segment_number=new_segment_numbers.begin();
         new_segment_number!=new_segment_numbers.end();
         new_segment_number++) {
      if (*new_segment_number == new_segment_number1)
        *new_segment_number = new_segment_number2;
    }
    
    // Set the new segment number
//    *long_segment_number = *new_segment_number;
  }
  
  // Print all changes again
  printf("\n");
  fd = fopen("renumbering.txt", "w");
  for (long_segment_number=long_segment_numbers.begin(),
       new_segment_number=new_segment_numbers.begin();
	   long_segment_number!=long_segment_numbers.end();
	   long_segment_number++, new_segment_number++) {
	if (*long_segment_number != *new_segment_number) {
      fprintf(fd, "%6lld %6lld     %6lld %6lld\n",
			  (*long_segment_number)/1000000,
			  (*long_segment_number)%1000000,
			  (*new_segment_number)/1000000,
			  (*new_segment_number)%100000);
	}
  }
  fclose(fd);
  
  // Apply new numbers to all points and save results in a new directory
  for (tile=tile_container->begin(); tile!=tile_container->end(); tile++) {

    // Derive the names of the output files. Note that the variable
    // newtile is only used for checking the existance of files. New file 
    // names are later transfered to tile.
    if (!tile->Name()) tile->LaserDataFiles::DeriveName();
    if (appendix) {
      newname = (char *) malloc(strlen(tile->Name()) +
                                strlen(appendix) + 1);
      sprintf(newname, "%s%s", tile->Name(), appendix);
    }
    else {
      newname = (char *) malloc(strlen(tile->Name()) + 1);
      sprintf(newname, "%s", tile->Name());
    }
    newtile.SetName(newname);
    newtile.DataOrganisation() = tile->DataOrganisation();
    newtile.DeriveMetaDataFileName(output_directory);
    newtile.DerivePointFileName(output_directory);
    free(newname);

    // Do not go ahead if the point file and meta file already exists and 
    // overwriting these files is not permitted.
    if (overwrite || !BNF_FileExists(newtile.MetaDataFile()) ||
        !BNF_FileExists(newtile.PointFile())) {
      
      // Read the point data
      if (!tile->Read(tile->PointFile(), false)) {
        fprintf(stderr, "Error reading laser points from file %s\n",
                tile->PointFile());
        exit(0);
      }

      // After reading all data, the file names can be overwritten with the
      // names of the output files.
      tile->SetName(newtile.Name());
      tile->DeriveMetaDataFileName(output_directory);
      tile->DerivePointFileName(output_directory);

      // Find the first and last change that applies to this tile
      tile_number = tile->TileRow() * 1000 + tile->TileColumn();
      for (long_segment_number=long_segment_numbers.begin(), found=false,
	       index=0;
	       long_segment_number!=long_segment_numbers.end() && !found;
		   long_segment_number++, index++) {
		if (*long_segment_number / 1000000 == tile_number) {
		  found = true;
		  first_segment_number = long_segment_number;
		  new_segment_number = new_segment_numbers.begin() + index;
		}
	  }
	  if (found) {
	  	for (long_segment_number=first_segment_number, found=false;
		     long_segment_number!=long_segment_numbers.end() && !found;
			 long_segment_number++) {
		  if (*long_segment_number / 1000000 != tile_number) {
		    found = true;
		    last_segment_number = long_segment_number-1;
		  }
	    }
		if (!found) {
		  found = true;
		  last_segment_number = long_segment_numbers.end() - 1;
		}  
	  }
      
      // Set tile numbers for all points
      tile->SetAttribute(SegmentStartTileNumberTag, tile_number);
      
      // Apply new segment numbers
      if (found) {
        for (long_segment_number=first_segment_number;
	         long_segment_number<=last_segment_number;
			 long_segment_number++, new_segment_number++) {
		  segment_number1 = (int) ((*long_segment_number)%1000000);
		  segment_number2 = (int) ((*new_segment_number)%1000000);
		  new_tile_number = (int) ((*new_segment_number)/1000000);
		  for (point1=tile->begin(); point1!=tile->end(); point1++) {
		    if (point1->HasAttribute(segment_tag)) {
		  	  if (point1->Attribute(segment_tag) == segment_number1 &&
			      point1->Attribute(SegmentStartTileNumberTag) == tile_number) {
		  	    point1->Attribute(segment_tag) = segment_number2;
			    point1->Attribute(SegmentStartTileNumberTag) = new_tile_number;
			  }
			}
		  }
		}
	  }

      // Write the points and meta data
      tile->Write(tile->PointFile(), false, false);
      printf("Saved tile %s\r", tile->Name());
      if (output_meta_data) tile->WriteMetaData();

      // Erase the points, the TIN, and the TIN edges
      tile->ErasePoints();
    }
    else {
      printf("Tile %s was already done.\n", newtile.Name());
      if (!tile->ReadMetaData(newtile.MetaDataFile())) {
        fprintf(stderr, "Error reading meta data from file %s\n",
                newtile.MetaDataFile());
        exit(0);
      }
    }
  }
  printf("\n");

  // Output of meta data at block level
  if (!block.Name()) block.DeriveName();
  if (appendix) {
    newname = (char *) malloc(strlen(block.Name()) +
                              strlen(appendix) + 1);
    sprintf(newname, "%s%s", block.Name(), appendix);
  }
  else {
    newname = (char *) malloc(strlen(block.Name()) + 1);
    sprintf(newname, "%s", block.Name());
  }
  block.SetName(newname);
  free(newname);
  block.DeriveMetaDataFileName(output_directory);
  block.WriteMetaData();
}
