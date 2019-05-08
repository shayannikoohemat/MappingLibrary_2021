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
 Segmentation of a block of laser scanning data based on segment attributes.
 First all tiles are segmented individually with growsegments.
 In this programme segments are merged across tile boundaries.
 Segment numbers are stored in SegmentNumberTag. The start of the segment
 is stored in SegmentTileNumberTag. Together the attributes provide unique
 segment numbers in a block of up to 1000 x 1000 tiles.
 Only non-planar segments are merged. Non planar segments are segments with
 numbers higher than tile.HighestSurfaceNumber().

 Initial creation:
 Author : George Vosselman
 Date   : 23-08-2013

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
                         Declaration of external functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(const char *, const char *, int *);
extern void timer_start(clock_t *time1);
extern void timer_end(clock_t time1, char *string);

/*
--------------------------------------------------------------------------------
                         The main segmentlaser function
--------------------------------------------------------------------------------
*/

void blocksegments(char *block_file,
                   char *appendix, char *output_directory,
                   bool output_meta_data, bool overwrite,
                   const SegmentationParameters &parameters,
				   double border_zone_width,
				   int min_num_pts_segment)
{
  char                  *directory, *filter, *filename, *newname;
  int                   icon, fileclass, point_index, i, num_merged, index,
                        num_pts1, num_pts2;
  LaserBlock            block;
  LaserBlock::iterator  tile_container;
  LaserUnit::iterator   tile;
  LaserSubUnit          newtile, *nb_tile;
  clock_t               start;
  DataBoundsLaser       border_zone;
  LaserPoints           border_points, nb_border_points;
  LaserPoint            sum1, sum2, *sum,
                        tolerances = parameters.GrowingTolerances();    
  TINEdges              *edges;
  LaserPoints::iterator point1, point2, point3;
  PointNumberList       nbh;
  PointNumberList::iterator nb, nb3;
  LaserPointTag         segment_tag;
  std::vector<int>           segment_numbers;
  std::vector<int>::iterator segment_number;
  std::vector<long long>     long_segment_numbers, new_segment_numbers,
                             tested_segments;
  std::vector<long long>::iterator long_segment_number, new_segment_number,
                                   first_segment_number, last_segment_number;
  int                              segment_number1, segment_number2,
                                   segment_number3, tile_number,
								   new_tile_number, highest_surface_number;
  long long                        long_segment_number1, long_segment_number2,
                                   long_segment_number3, new_segment_number1,
								   new_segment_number2;
  bool                       merge_segments, valid, found, debug=true;
  double                     angle, pi = 4.0 * atan(1.0);
  std::vector<std::pair<long long,long long> >  changed_numbers;	
  std::vector<std::pair<long long,long long> >::iterator  changed_number;	
  std::pair<long long, long long> old_and_new_number;
  const unsigned char       *tolerance_tags, *tag;
  int                       num_tolerances, itag;
  
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

  // Tolerance information
  tolerance_tags = tolerances.AttributeTags();
  num_tolerances = tolerances.NumAttributes();
  
  // Try to merge segments across tile boundaries
  segment_tag = parameters.SegmentAttribute();
  FILE *fd = fopen("merged_segments.txt", "w");
  for (tile=tile_container->begin(); tile!=tile_container->end(); tile++) {

  	for (i=0; i<2; i++) {
  	  // See if there is a tile with a higher row or column number
	  if (i == 0) nb_tile = block.Tile(tile->TileRow()+1, tile->TileColumn());
  	  else nb_tile = block.Tile(tile->TileRow(), tile->TileColumn()+1);
  	  if (nb_tile == NULL) continue;

      if (debug)
	    printf("Merging tile %s and %s\n", tile->Name(), nb_tile->Name());	         
	    
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
      nb_tile->ErasePoints();
      if (nb_border_points.empty()) continue;
      	  
      // Create a sorted vector of all different segment numbers,
      // and create long segment numbers including tile number for all
      // segments, but not for planar surfaces.
      segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
      border_points.AttributeValues(segment_tag, segment_numbers);
      std::sort(segment_numbers.begin(), segment_numbers.end());
      highest_surface_number = tile->HighestSurfaceNumber();
      if (debug) printf("Highest surface number tile %d\n", highest_surface_number);
      for (segment_number=segment_numbers.begin();
           segment_number!=segment_numbers.end(); segment_number++) {
        if (*segment_number > highest_surface_number)
	      long_segment_numbers.push_back((long long) tile->TileRow() * 1000000000 +
		                                 (long long) tile->TileColumn() * 1000000 +
			  						     *segment_number);
      }
      if (debug) printf("%d segments from tile %s\n", long_segment_numbers.size(),
	                    tile->Name());
	  num_merged = long_segment_numbers.size();

      // Add segments of neighbouring tile
      segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
      nb_border_points.AttributeValues(segment_tag, segment_numbers);
      std::sort(segment_numbers.begin(), segment_numbers.end());
      highest_surface_number = nb_tile->HighestSurfaceNumber();
      if (debug) printf("Highest surface number nb_tile %d\n", highest_surface_number);
      for (segment_number=segment_numbers.begin();
           segment_number!=segment_numbers.end(); segment_number++) {
        if (*segment_number > highest_surface_number)
  	      long_segment_numbers.push_back((long long) nb_tile->TileRow() * 1000000000 +
		                                 (long long) nb_tile->TileColumn() * 1000000 +
									     *segment_number);
      }
      if (debug) 
	    printf("%d segments from tile %s\n", long_segment_numbers.size() - num_merged);

      // Copy the vector of long segment numbers
      new_segment_numbers.insert(new_segment_numbers.begin(),
                                 long_segment_numbers.begin(),
								 long_segment_numbers.end());

      // Merge point sets
      border_points.insert(border_points.end(), nb_border_points.begin(),
	                       nb_border_points.end());
	  nb_border_points.ErasePoints(); // No longer needed
	                       
      // Derive edges
      edges = border_points.DeriveEdges(parameters);
      
      // Check all neighbourhoods
      num_merged = 0;
      if (!debug) 
	    printf("Merging tiles (%3d, %3d) and (%3d, %3d), %6d points in border   \r",
	         tile->TileRow(), tile->TileColumn(),  	  
	         nb_tile->TileRow(), nb_tile->TileColumn(),
             border_points.size());
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
	      // If it's not in the list, the segment number belongs to a planar
	      // surface and the segment can be ignored.
	      if (long_segment_number == long_segment_numbers.end()) continue;
	      // Otherwise, get the index and number
	      index = std::distance(long_segment_numbers.begin(),
		                        long_segment_number);
	      new_segment_number1 = *(new_segment_numbers.begin() + index);
	  
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
	          // If it's not in the list, the segment number belongs to a planar
	          // surface and the segment can be ignored.
	          if (long_segment_number == long_segment_numbers.end()) continue;
	          // Otherwise, get the index and number
	          index = std::distance(long_segment_numbers.begin(),
			                        long_segment_number);
	          new_segment_number2 = *(new_segment_numbers.begin() + index);
      	      if (new_segment_number1 != new_segment_number2) {

          	  	// Only test adjacent segment once
                if(std::find(tested_segments.begin(), tested_segments.end(),
			                 long_segment_number2) == tested_segments.end()) {
			      merge_segments = true;
			      
			      // Initialise sums and counts
                  sum1.Initialise(); sum2.Initialise();
                  num_pts1 = num_pts2 = 0;
                  
                  // Calculate attribute sums for both segments
                  for (nb3=nbh.begin(); nb3!=nbh.end(); nb3++) {
          	        point3 = border_points.begin() + nb3->Number();
        	        if (point3->HasAttribute(segment_tag)) {
      	              // Get the new segment number of this point
      	              segment_number3= point3->Attribute(segment_tag);
                      long_segment_number3 =
		                (long long) point3->Attribute(SegmentStartTileNumberTag) * 1000000 +
		                (long long) segment_number3;
		            
		              // See if this point is in either segment, if not continue to next neighbour
		              if (long_segment_number3 == long_segment_number1) {
		              	num_pts1++;
		              	sum = &sum1;
		              }
		              else if (long_segment_number3 == long_segment_number2) {
		              	num_pts2++;
		              	sum = &sum2;
		              }
		              else continue;
		              
		              // Update sum
                      for (itag=0, tag=tolerance_tags; itag < num_tolerances; itag++, tag++) {
		                switch (AttributeType((LaserPointTag) *tag)) {
		                  default:
		                  case IntegerAttributeType:
	                        sum->Attribute((LaserPointTag) *tag) +=
	                          point3->Attribute((LaserPointTag) *tag);
	                        break;
		                  case FloatAttributeType:
	                        sum->FloatAttribute((LaserPointTag) *tag) +=
	                          point3->FloatAttribute((LaserPointTag) *tag);
	                        break;
		                  case DoubleAttributeType:
	                        sum->DoubleAttribute((LaserPointTag) *tag) +=
	                          point3->DoubleAttribute((LaserPointTag) *tag);
	                        break;
	                    }
	                  }
		            }
		          }
                   
      	          // Check if we've tested sufficient points for both segments
                  merge_segments = (num_pts1 >= min_num_pts_segment &&
					                num_pts2 >= min_num_pts_segment);
					                
				  // Check attribute similarity
                  if (merge_segments) {
				    for (itag=0, tag=tolerance_tags;
					     itag < num_tolerances && merge_segments;
						 itag++, tag++) {
		              switch (AttributeType((LaserPointTag) *tag)) {
		  	            case IntegerAttributeType:
		  	            default:
		  	              if (fabs((float) sum1.Attribute((LaserPointTag) *tag) / num_pts1 -
					               (float) sum2.Attribute((LaserPointTag) *tag) / num_pts2) >
				              tolerances.Attribute((LaserPointTag) *tag))
				            merge_segments = false;
				          break;
				
		  	            case FloatAttributeType:
		  	              // Possibly flip (scaled) normal
		  	  	          if (*tag >= NormalXTag && *tag <= NormalZTag) {
		  	  	            if (sum1.Normal().Length() > 0.0 &&
					            sum2.Normal().Length() > 0.0) {
					          if (Angle(sum1.Normal(), sum2.Normal()) >
					              pi / 2.0) sum2.FlipNormal();
				            }
		  	  	          }
		  	  	          else if (*tag >= ScaledNormalXTag && *tag <= ScaledNormalZTag) {
		  	  	            if (sum1.ScaledNormal().Length() > 0.0 &&
					            sum2.ScaledNormal().Length() > 0.0) {
					          if (Angle(sum1.ScaledNormal(), sum2.ScaledNormal()) >
					              pi / 2.0) sum2.FlipScaledNormal();
				            }
		  	  	          }
		  	              if (fabs((float) sum1.FloatAttribute((LaserPointTag) *tag) / num_pts1 -
					               (float) sum2.FloatAttribute((LaserPointTag) *tag) / num_pts2) >
				              tolerances.FloatAttribute((LaserPointTag) *tag))
				            merge_segments = false;
				          break;
				
		  	            case DoubleAttributeType:
		  	              if (fabs((float) sum1.DoubleAttribute((LaserPointTag) *tag) / num_pts1 -
					               (float) sum2.DoubleAttribute((LaserPointTag) *tag) / num_pts2) >
				              tolerances.DoubleAttribute((LaserPointTag) *tag))
				            merge_segments = false;
				          break;
					  }
				    }
                  }

      	          // Avoid testing this segment again for this point
      	          tested_segments.push_back(long_segment_number2);

      	          // Merge the segments
      	          if (merge_segments) {
      	      	    num_merged++;
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
      edges->Erase();
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

      // Add default segment start tile number for all points of this tile
      tile_number = tile->TileRow() * 1000 + tile->TileColumn();
      tile->SetAttribute(SegmentStartTileNumberTag, tile_number);

      // Find the first and last change that applies to this tile
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
