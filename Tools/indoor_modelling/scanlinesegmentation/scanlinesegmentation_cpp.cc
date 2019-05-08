
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

 Initial creation:
 Author : George Vosselman
 Date   : 14-12-2013

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include "LaserBlock.h"
#include "LaserScanLines.h"
#include <ctime>

void rangeOfResidualsSegmentationForward(LaserPoints &points, int ptsStart,double percentagePts,double threshold, int min_size)
{
  LaserPoints::iterator    point, first_seed_point, last_seed_point,
                           first_segment_point, next_seed_point;
  bool                     done, good;   
  Line3D                   line;    
  int 						segment=1;                    
  
  // Initialise first seed line
  first_seed_point = points.begin();
  last_seed_point = first_seed_point + ptsStart;
  line.Erase();
  for (point=first_seed_point; point!=last_seed_point+1; point++)
    line.AddPoint(point->Position3DRef(), false);
  line.Recalculate();

  done = false;
  do {
  	// Check if the complete seed is in one scan line
  	good = (first_seed_point->Attribute(ScanLineNumberTag) ==
  	        last_seed_point->Attribute(ScanLineNumberTag));
  	        
	// Check if the seed range is linear (both individual points and overall
	// fitting accuracy) and if all points are valid
	if (good) 
	{
	  double average_distance=0.0;
      for (point=first_seed_point; point!=last_seed_point+1 && good; point++) 
	  {
	    if (point->Label() < 0) 
		{
	    	good = false;
	    	break;
	    }
      	average_distance+=line.DistanceToPoint(point->Position3DRef());
      }
      if (average_distance/float(ptsStart) > threshold) {good = false;}
	}
	if (good) {if (line.SigmaLineFitting() > 1.8 * threshold) good = false;}
	
	// Move seed range one point further if the seed is not good
  	if (!good) {
  	  last_seed_point++;
	  if (last_seed_point == points.end()) {done = true;}
	  else 
	  {
  	    line.RemovePoint(first_seed_point->Position3DRef(), false);
  	    first_seed_point++;
  	    line.AddPoint(last_seed_point->Position3DRef(), true);
        line.Recalculate();
	  }
  	}
    // Otherwise, try to extend to a larger segment
  	else {
      first_segment_point = first_seed_point;
      next_seed_point=last_seed_point+1;
      bool segmentDone=false;
      do 
	  {
  	    good=true;

 	    if (next_seed_point == points.end()) {good = false;}
  	    
        // Check if the next point is valid
        if (good && next_seed_point->Label() < 0) good = false;
        
        // Check if the next point is in the same scan line
  	    if (good) 
		{
  	      if (next_seed_point->Attribute(ScanLineNumberTag) !=
		      (next_seed_point-1)->Attribute(ScanLineNumberTag)) good = false;
	    }
		    
		// Check distance of the next point
		if (good) 
		{
			int nPts=round(percentagePts*line.NumberOfPoints());
//			printf("%d\n",nPts);
			LaserPoints::iterator last_pt=next_seed_point-nPts-1;
//			printf("%d %d\n",next_seed_point-points.begin(),last_pt-points.begin());
			for(LaserPoints::iterator pt=next_seed_point;pt!=last_pt;pt--) 
			{
				double meandist=0.0;
				for(LaserPoints::iterator pt2=next_seed_point;pt2!=pt-1;pt2--) 
				{
					meandist+=line.DistanceToPoint(pt2->Position3DRef());
				}
				meandist/=double(next_seed_point-pt+1);
				double thresh=3.*threshold/sqrt(next_seed_point-pt+1);
//				double thresh=5.*threshold/sqrt(next_seed_point-pt+1);
//				printf("%f %f\n",meandist,thresh);
				if(meandist > thresh)
				{
					good=false;
					break;
				}
			}			
			
////////// test: simultaneous forward/backward testing			
//			last_pt=next_seed_point+nPts;
////			printf("%d %d\n",next_seed_point-points.begin(),last_pt-points.begin());
//			for(LaserPoints::iterator pt=next_seed_point;pt!=last_pt;pt++) 
//			{
//				double meandist=0.0;
//				for(LaserPoints::iterator pt2=next_seed_point;pt2!=pt+1;pt2++) 
//				{
//					meandist+=line.DistanceToPoint(pt2->Position3DRef());
//				}
//				meandist/=double(pt-next_seed_point+1);
//				double thresh=3.*threshold/sqrt(pt-next_seed_point+1);
////				double thresh=5.*threshold/sqrt(next_seed_point-pt+1);
////				printf("%f %f\n",meandist,thresh);
//				if(meandist > thresh)
//				{
//					good=false;
//					break;
//				}
//			}			
////////// test: simultaneous forward/backward testing			
		}


      // Update the line
      if (good) {
  	      line.AddPoint(next_seed_point->Position3DRef(), true);
 		  line.Recalculate();
//          if (line.SigmaLineFitting() > 1.8 * threshold) good = false;
//          else last_seed_point = next_seed_point;
          last_seed_point = next_seed_point;
		  next_seed_point++;
	  }
		
		if(!good) 
		{
	  	  if(last_seed_point-first_segment_point>min_size)
	  	  {
			segment++;
		    for (point=first_segment_point; point!=last_seed_point && point!=points.end(); point++) {
		      point->Attribute(SegmentNumberTag)=segment;
		    }
		  }
			break;
		}
	} while (true);	
	  		
      // Set new seed range
      if (!good) {
      // Store this scan s;egment
//      printf("here\n");
	  
	  first_seed_point = next_seed_point + 1;
	  last_seed_point = first_seed_point + ptsStart;
		  if (first_seed_point == points.end() || last_seed_point>=points.end()) {
		     done = true;
		 }
		 else
		 {
		  line.Erase();
		  for (point=first_seed_point; point!=last_seed_point+1; point++)
		    line.AddPoint(point->Position3DRef(), false);
		  line.Recalculate();
		 }
      }
	}

  } while(!done);
}

void rangeOfResidualsSegmentationBackward(LaserPoints &points, int ptsStart,double percentagePts,double threshold, int min_size)
{
  LaserPoints::iterator    point, first_seed_point, last_seed_point,
                           first_segment_point, next_seed_point;
  bool                     done, good;   
  Line3D                   line;    
  
  int 						segment=1, segment_fw=-1;                    

  vector<int> values=points.AttributeValues(SegmentNumberTag);
  segment=*std::max_element(values.begin(),values.end());
  
  // Initialise first seed line
  first_seed_point = points.begin();
  last_seed_point = first_seed_point + ptsStart;
  line.Erase();
  for (point=first_seed_point; point!=last_seed_point+1; point++)
    line.AddPoint(point->Position3DRef(), false);
  line.Recalculate();

  done = false;
  do {
  	// Check if the complete seed is in one scan line
  	good = (first_seed_point->Attribute(ScanLineNumberTag) ==
  	        last_seed_point->Attribute(ScanLineNumberTag));
  	        
	// Check if the seed range is linear (both individual points and overall
	// fitting accuracy) and if all points are valid
	if (good) 
	{
	  double average_distance=0.0;
      for (point=first_seed_point; point!=last_seed_point+1 && good; point++) 
	  {
	    if (point->Label() < 0) 
		{
	    	good = false;
	    	break;
	    }
      	average_distance+=line.DistanceToPoint(point->Position3DRef());
      }
      if (average_distance/float(ptsStart) > threshold) {good = false;}
	}
	if (good) {if (line.SigmaLineFitting() > 1.8 * threshold) good = false;}
	
	// Move seed range one point further if the seed is not good
  	if (!good) {
  	  last_seed_point++;
	  if (last_seed_point == points.end()) {done = true;}
	  else 
	  {
  	    line.RemovePoint(first_seed_point->Position3DRef(), false);
  	    first_seed_point->RemoveAttribute(SegmentNumberTag);
  	    first_seed_point++;
  	    line.AddPoint(last_seed_point->Position3DRef(), true);
        line.Recalculate();
	  }
  	}
    // Otherwise, try to extend to a larger segment
  	else {
      first_segment_point = first_seed_point;
      next_seed_point=last_seed_point+1;
      bool segmentDone=false;
      do 
	  {
  	    good=true;

 	    if (next_seed_point == points.end()) {good=false;}
  	    
        // Check if the next point is valid
        if (good && next_seed_point->Label() < 0) good = false;
        
        // Check if the next point is in the same scan line
  	    if (good) 
		{
  	      if (next_seed_point->Attribute(ScanLineNumberTag) !=
		      (next_seed_point-1)->Attribute(ScanLineNumberTag)) good = false;
	    }
		    
		// Check distance of the next point
		if (good) 
		{
			int nPts=round(percentagePts*line.NumberOfPoints());
//			printf("%d\n",nPts);
			LaserPoints::iterator last_pt=next_seed_point-nPts-1;
//			printf("%d %d\n",next_seed_point-points.begin(),last_pt-points.begin());
			for(LaserPoints::iterator pt=next_seed_point;pt!=last_pt;pt--) 
			{
				double meandist=0.0;
				for(LaserPoints::iterator pt2=next_seed_point;pt2!=pt-1;pt2--) 
				{
					meandist+=line.DistanceToPoint(pt2->Position3DRef());
				}
				meandist/=double(next_seed_point-pt+1);
				double thresh=3.*threshold/sqrt(next_seed_point-pt+1);
//				double thresh=5.*threshold/sqrt(next_seed_point-pt+1);
//				printf("%f %f\n",meandist,thresh);
				if(meandist > thresh)
				{
					good=false;
					break;
				}
			}			
		}

      // Update the line
      if (good) {
  	      line.AddPoint(next_seed_point->Position3DRef(), true);
 		  line.Recalculate();
//          if (line.SigmaLineFitting() > 1.8 * threshold) good = false;
//          else last_seed_point = next_seed_point;
          last_seed_point = next_seed_point;
		  next_seed_point++;
	  }
		
		if(!good) 
		{
	  	  if(last_seed_point-first_segment_point>min_size)
	  	  {
			segment++;
		    for (point=first_segment_point; point!=last_seed_point && point!=points.end(); point++) {
		    	if(!point->HasAttribute(SegmentNumberTag))
		    	{
		    		segment++;
		    		continue;
		    	}
		    	else if(point->Attribute(SegmentNumberTag)!=segment_fw)
		    	{
		    		segment_fw=point->Attribute(SegmentNumberTag);
		    		segment++;
		    	}
		      point->Attribute(SegmentNumberTag)=segment;
		    }
		  }
			break;
		}
	} while (true);	
	  		
      // Set new seed range
      if (!good) {
      // Store this scan s;egment
//      printf("here\n");
	  
	  first_seed_point = next_seed_point + 1;
	  last_seed_point = first_seed_point + ptsStart;
		  if (first_seed_point == points.end() || last_seed_point>=points.end()) {
		     done = true;
		 }
		 else
		 {
		  line.Erase();
		  for (point=first_seed_point; point!=last_seed_point+1; point++)
		    line.AddPoint(point->Position3DRef(), false);
		  line.Recalculate();
		 }
      }
	}

  } while(!done);	
  
}


/*
--------------------------------------------------------------------------------
                      The main scanlinesegmentation_cpp function
--------------------------------------------------------------------------------
*/

void scanlinesegmentation_cpp(char *in_file, const char *out_file,
                              bool add_time_offset, bool use_old_labels,
							  int scanner_id_user, int method, int ptsStart,
							  double percentagePts, double threshold,
							  int min_size, char *appendix, bool labeloffset)
{
  LaserBlock               block;
  LaserBlock::iterator     strip;
  LaserUnit::iterator      points;
  LaserPoints::iterator    point, first_seed_point, last_seed_point,
                           first_segment_point, next_seed_point,
						   start_point, end_point, start_point2, end_point2,
						   old_start_point2, old_end_point;
  LaserScanLines           last_scan_segments, scan_segments;
  LaserScanLines::iterator matching_scan_segment, scan_segment, scan_segment2;
  Line3D                   line, line2;
  Plane                    plane;
  bool                     done, good, debug=false;
  int                      label, i, segment, file_type, istrip, scanner_id,
                           num_fitted_points, num_fitted_points2, scan;
  double                   pi = 4.0 * atan(1.0), sigma, sigma2, sqsum,
                           best_sqsum, dist, prev_dist;
  Position3D               reference_pos=Position3D(0.0, 0.0, 0.0);
  Vector3D                 direction;
  char                     *newname, *output_directory;
  
  double sigma_dist = 0.02; // Noise of the Hokuyo scanner is 0.03
//  double sigma_dist = 0.0003; // To process noiseless data
  double max_dist_to_line = 5.0 * sigma_dist;  // Maximum distance of a point to a line segment

  ObjectPoints   debug_points;
  ObjectPoint    debug_point;
  LineTopologies debug_tops;
    
  // Set up the block to be processed.
  if (appendix) {
  	if (!block.ReadMetaData(in_file)) {
  	  printf("Error reading block meta data file %s\n", in_file);
  	  exit(0);
  	}
  }
  else {
  	if (!block.Create(in_file, &file_type)) {
  	  printf("Error reading file %s\n", in_file);
  	  exit(0);
  	}
  }
  
  // Loop over all strips
  segment = 0;
  for (strip=block.begin(), istrip=0; strip!=block.end(); strip++, istrip++) {
  	if (appendix) scanner_id = istrip;
  	else scanner_id = scanner_id_user;
  	if (!labeloffset) segment = 0;
  	else segment++;
  	// Loop over all strip parts
  	for (points=strip->begin(); points!=strip->end(); points++) {
  
      // Read points
      if (!points->Read(points->PointFile(), false)) {
  	    printf("Error reading points from file %s\n", points->PointFile());
  	    exit(0);
      }
      if (appendix) {
      	printf("Processing dataset %s\r", points->LaserMetaFile::DeriveName());
      	fflush(stdout);
      }
    
      // Remove possible old attributes
      points->RemoveAttribute(ComponentNumberTag);
      points->RemoveAttribute(SegmentNumberTag);
      if (use_old_labels) points->RemoveAttribute(LabelTag);
      else {
        for (point=points->begin(); point!=points->end(); point++)
          if (point->Label() > 0 || !point->HasAttribute(LabelTag)) point->Label(1); // Valid unclassified
          else point->Label(-1); // Invalid unclassified
      }

//------------------------------------------------------------------------------
// Find all segments in a greedy search
//------------------------------------------------------------------------------
      
      if (method==0) {

        // Initialise first seed line
        first_seed_point = points->begin();
        if (points->size() < min_size) {
  	      printf("Error: point set has less than %d points (%d)\n",
	             min_size, points->size());
      	exit(0);
        }
        last_seed_point = first_seed_point + min_size - 1;
        line.Erase();
        for (point=first_seed_point; point!=last_seed_point+1; point++)
          line.AddPoint(point->Position3DRef(), false);
        line.Recalculate();

        done = false;
        do {
          debug = (first_seed_point->Attribute(ScanLineNumberTag) == 2025 &&
		           first_seed_point->FloatAttribute(AngleTag) > 246.5 * pi / 180.0 &&
			       first_seed_point->FloatAttribute(AngleTag) < 258.5 * pi / 180.0);
          debug = false;
    
  	      // Check if the complete seed is in one scan line
  	      good = (first_seed_point->Attribute(ScanLineNumberTag) ==
  	              last_seed_point->Attribute(ScanLineNumberTag));
  	        
	      // Check if the seed range is linear (both individual points and overall
	      // fitting accuracy) and if all points are valid
	      if (good) {
            for (point=first_seed_point; point!=last_seed_point+1 && good; point++) {
	          if (line.DistanceToPoint(point->Position3DRef()) > max_dist_to_line)
	            good = false;
	          if (point->Label() < 0) good = false;
	        }
	      }
	      if (good) {
	        if (line.SigmaLineFitting() > 1.8 * sigma_dist) good = false;
	      }
	
	      // Check if there is a large irregularity in the point spacing
	      // The point spacing corrected for possible noise should not increase by 
	      // a factor larger than 2.0 (probably a smaller value is possible)
	      if (good) {
	        prev_dist = first_seed_point->Distance((first_seed_point+1)->Position3DRef());
	        for (point=first_seed_point+2; point!=last_seed_point+1 && good; point++) {
	  	      dist = (point-1)->Distance(point->Position3DRef());
	  	      if (debug) printf("dist %.2f  prev_dist %.2f", dist, prev_dist);
	  	      if (dist > prev_dist) {
	  	        if (dist - 4.0 * 1.41 * sigma_dist > 2.0 * (prev_dist + 4.0 * 1.41 * sigma_dist))
	  	          good = false;
	  	      }
	  	      else {
	  	        if (prev_dist - 4.0 * 1.41 * sigma_dist > 2.0 * (dist + 4.0 * 1.41 * sigma_dist))
	  	          good = false;
	  	      }
	          if (debug) {
	            if (good) printf("   accepted\n");
	            else      printf("   rejected\n");
	          }
	  	      prev_dist = dist;
	        }
//	        if (!good) printf("Seed in scan %d rejected because of point spacing irregularity\n",
//	                          first_seed_point->Attribute(ScanLineNumberTag));
	      }
	
	      // Move seed range one point further if the seed is not good
  	      if (!good) {
  	        last_seed_point++;
	        if (last_seed_point == points->end()) {
	          done = true;
	        }
	        else {
  	          line.RemovePoint(first_seed_point->Position3DRef(), false);
  	          first_seed_point++;
  	          line.AddPoint(last_seed_point->Position3DRef(), true);
	        }
  	      }
  	
 	      // Otherwise, try to extend to a larger segment
  	      else {
  	        debug = (first_seed_point->Attribute(ScanLineNumberTag) == 2025 &&
		             first_seed_point->FloatAttribute(AngleTag) > 246.5 * pi / 180.0 &&
			         first_seed_point->FloatAttribute(AngleTag) < 258.5 * pi / 180.0);
	        debug = false;
	  
            first_segment_point = first_seed_point;
            for (next_seed_point=last_seed_point+1;
	             next_seed_point!=points->end() && good; next_seed_point++) {
  	    
              // Check if the next point is valid
              if (next_seed_point->Label() < 0) good = false;

  	          // Check if the next point is in the same scan line
  	          if (good) {
  	            if (next_seed_point->Attribute(ScanLineNumberTag) !=
		            (next_seed_point-1)->Attribute(ScanLineNumberTag)) good = false;
	          }
		    
		      // Check distance of the next point
		      if (good) {
	            if (line.DistanceToPoint(next_seed_point->Position3DRef()) > max_dist_to_line) {
	              good = false;
	              if (debug) {
	      	        printf("Point at %.3f to segment of %d points\n",
			               line.DistanceToPoint(next_seed_point->Position3DRef()),
				           std::distance(first_segment_point, last_seed_point)+1);
			      }
	            }
	          }
	    
		      // Check the point spacing
	          if (good) {
	  	        dist = (next_seed_point-1)->Distance(next_seed_point->Position3DRef());
	  	        if (debug) printf("dist %.2f  prev_dist %.2f", dist, prev_dist);
	  	        if (dist > prev_dist) {
	  	          if (dist - 4.0 * 1.41 * sigma_dist > 2.0 * (prev_dist + 4.0 * 1.41 * sigma_dist))
	  	            good = false;
	  	        }
	  	        else {
	  	          if (prev_dist - 4.0 * 1.41 * sigma_dist > 2.0 * (dist + 4.0 * 1.41 * sigma_dist))
	  	            good = false;
	  	        }
	  	        prev_dist = dist;
	          }
	          if (debug) {
	            if (good) printf("   accepted\n");
	            else      printf("   rejected\n");
	          }
	    
		      // Update the line
	          if (good) {
  	            line.AddPoint(next_seed_point->Position3DRef(), true);
                if (line.SigmaLineFitting() > 1.8 * sigma_dist) good = false;
                else last_seed_point = next_seed_point;
	          }
            }
            // Store this scan segment
            scan_segments.push_back(LaserScanLine(*points, first_segment_point,
	                                              last_seed_point));
            if (debug) {
	          printf("Segment with %d points from angle %5.1f to %5.1f\n",
	                 (scan_segments.end()-1)->NumberOfPoints(),
			         first_segment_point->FloatAttribute(AngleTag) * 180 / pi,
			         last_seed_point->FloatAttribute(AngleTag) * 180 / pi);
		      printf("  along line direction %6.3f %6.3f %6.3f\n",
		             line.Direction().X(), line.Direction().Y(),
			         line.Direction().Z());
		      for (point=first_segment_point; point!=last_seed_point+1; point++)
		        printf("%.1f  %6.2f %6.2f %6.2f\n",
		               point->FloatAttribute(AngleTag) * 180 / pi,
				       point->X(), point->Y(), point->Z());
		      debug_tops.push_back(LineTopology(scan_segments.size(), 
		                                        debug_points.size(),
										        debug_points.size() + 1));
		      debug_point.vect() = 
		        line.Project(first_segment_point->Position3DRef()).vect();
		      debug_point.Number() = debug_points.size();
		      debug_points.push_back(debug_point);
		      debug_point.vect() = 
		        line.Project(last_seed_point->Position3DRef()).vect();
		      debug_point.Number() = debug_points.size();
		      debug_points.push_back(debug_point);
	        }

            // Set new seed range
            if (!good) {
      	      first_seed_point = last_seed_point+1;
      	      last_seed_point = first_seed_point + min_size - 1;
      	      if (last_seed_point >= points->end()) done = true;
      	      // Estimate line
              line.Erase();
              for (point=first_seed_point; point!=last_seed_point+1; point++)
                line.AddPoint(point->Position3DRef(), false);
              line.Recalculate();
            }
            else done = true;
  	      }
        } while (!done);

//      debug_points.Write("debug_segm.objpts");
//      debug_tops.Write("debug_segm.top", false);
  
//------------------------------------------------------------------------------
// Correct transitions between adjacent scan segments for greedy segmentation
//------------------------------------------------------------------------------
  
        for (scan_segment=scan_segments.begin(), scan_segment2=scan_segment+1;
             scan_segment2!=scan_segments.end(); scan_segment++, scan_segment2++) {
          // Check if scan segments are directly adjacent
          start_point  = scan_segment->begin(*points);
          end_point    = scan_segment->end(*points);
          start_point2 = scan_segment2->begin(*points);
          end_point2   = scan_segment2->end(*points);
          if (end_point + 1 == start_point2) {
  	        debug = (start_point->Attribute(ScanLineNumberTag) == 2025);
            debug = false;
      
            // Re-fit two lines to the min_size nearest points 
            line = points->FitLine(end_point - min_size + 1, end_point);
            line2 = points->FitLine(start_point2, start_point2 + min_size - 1);

            // Determine total least squares sum
            sigma  = line.SigmaLineFitting();
            sigma2 = line2.SigmaLineFitting();
            best_sqsum = min_size * (sigma * sigma + sigma2 * sigma2);
    
            // Shift one point back until there's no further improvement  of the
            // total least squares sum
            done = false;
            num_fitted_points = num_fitted_points2 = min_size;
            do {
      	      // First check if the points are regularly spaced
      	      prev_dist = end_point->Distance((end_point-1)->Position3DRef());
      	      dist      = end_point->Distance(start_point2->Position3DRef());
      	      good = true;
      	      if (dist > prev_dist) {
	  	        if (dist - 4.0 * 1.41 * sigma_dist > 2.0 * (prev_dist + 4.0 * 1.41 * sigma_dist))
	  	          good = false;
	  	      }
	  	      else {
	  	        if (prev_dist - 4.0 * 1.41 * sigma_dist > 2.0 * (dist + 4.0 * 1.41 * sigma_dist))
	  	          good = false;
	  	      }

              // Test the square sum if the distance check is okay
              if (good) {

                // Shift segment borders one point back and adjust the lines
                line.RemovePoint(end_point->Position3DRef(), false); 
	            end_point--;
	            if (num_fitted_points == min_size)
                  line.AddPoint((end_point-num_fitted_points+1)->Position3DRef(), true);
                if (num_fitted_points2 == min_size)
                line2.RemovePoint((start_point2+num_fitted_points2-1)->Position3DRef(), false);
                start_point2--; 
	            line2.AddPoint(start_point2->Position3DRef(), true);

                // Determine new total least squares sum
                sigma  = line.SigmaLineFitting();
                sigma2 = line2.SigmaLineFitting();
                sqsum = num_fitted_points * sigma * sigma +
	                    num_fitted_points2 * sigma2 * sigma2;
      
                // Check if this is an improvement
                if (sqsum < best_sqsum) {
      	          best_sqsum = sqsum;
  	              // Check if we can shift back once more (we need at least two points on a line)
                  if (num_fitted_points == 2) done = true;
                  else {
                    // Check if the number of points for fitting need adjustment
                    if (end_point-num_fitted_points+1 == start_point) {
          	          num_fitted_points--;
          	          num_fitted_points2++;
                    }
                  }
                }
	            else {
	  	          // No improvement, restore previous segment bounds
	  	          end_point++;
	  	          start_point2++;
	  	          done = true;
	            }
	          }
	    
	          // If the distance test failed, we're done with this transition
	          else {
	            done = true;
	            if (debug) {
	      	      printf("Point at %.1f does not switch segments because of distance check.\n",
			             end_point->FloatAttribute(AngleTag)*180/pi);
			      printf("  prev_dist %.2f  dist %.2f\n", prev_dist, dist);
	            }
	          }
            } while (!done);
    
            // Check on minimum size of first segment
            if (num_fitted_points < min_size) {
              // Delete segment and adjust iterators
              if (debug) 
		        printf("Deleting segment with %d points from %.1f to %.1f\n",
		               scan_segment->NumberOfPoints(),
				       scan_segment->begin(*points)->FloatAttribute(AngleTag)*180.0/pi,
				       scan_segment->end(*points)->FloatAttribute(AngleTag)*180.0/pi);
              scan_segments.erase(scan_segment);
              scan_segment--; scan_segment2--;
            }
            else {
              // Re-set end points of the segments
              scan_segment->SetBeginAndEnd(*points, start_point, end_point);
              scan_segment2->SetBeginAndEnd(*points, start_point2, end_point2);
              if (debug) {
		        printf("Resetting segment with %d points from %.1f to %.1f\n",
		               scan_segment->NumberOfPoints(),
				       start_point->FloatAttribute(AngleTag)*180.0/pi,
				       end_point->FloatAttribute(AngleTag)*180.0/pi);
		        printf("  and segment with %d points from %.1f to %.1f\n",
		               scan_segment2->NumberOfPoints(),
				       start_point2->FloatAttribute(AngleTag)*180.0/pi,
				       end_point2->FloatAttribute(AngleTag)*180.0/pi);
		      }
            }
          }
    
          // For non-adjacent segments remove the ends
          else {
            done = false;
	        old_end_point = end_point;
            do {
      	      if (start_point == end_point - min_size + 3 ||
      	          end_point->Distance(old_end_point->Position3DRef()) > max_dist_to_line)
      	        done = true;
      	      else end_point--;
            } while (!done);
            done = false;
            old_start_point2 = start_point;
            do {
      	      if (start_point == end_point - min_size + 3 ||
      	          start_point->Distance(old_start_point2->Position3DRef()) > max_dist_to_line)
      	        done = true;
      	      else start_point++;
            } while (!done);
            // Reset the end of the first segment if it is still long enough
            if (std::distance(start_point, end_point) >= min_size - 1)
              scan_segment->SetBeginAndEnd(*points, start_point, end_point);
            else { // For now just leave the short segment intact and don't delete segments
            }
            if (std::distance(start_point2, end_point2) >= min_size - 1)
              scan_segment2->SetBeginAndEnd(*points, start_point2, end_point2);   
          }
        }
  
    
//------------------------------------------------------------------------------
// Labelling of segments (floor, wall, ceiling)
//------------------------------------------------------------------------------

        for (scan_segment=scan_segments.begin(); 
		     scan_segment!=scan_segments.end(); scan_segment++) {
      
          // Retrieve end points
          start_point = scan_segment->begin(*points);
          end_point   = scan_segment->end(*points);

          // Final check on length without using the end points (With the end points
          // a line could be formed by a local cluster and an arbitrary point)
          // To be sure, just remove the end points
          start_point++;
          end_point--;
          if (start_point->Distance(end_point->Position3DRef()) < 0.1) continue;

          // Determine floor and ceiling
          if (use_old_labels) label = 0; // Wall
          else label = LPWall;
          if (scanner_id > 0) {
            direction = end_point->vect() - start_point->vect();
            if (fabs(direction.Z()) / direction.Length() < 0.2) {
      	      if (start_point->Z() + end_point->Z() > 0.0) {
      	        if (use_old_labels) label = 1; // Ceiling
      	        else label = LPCeiling;
      	      }
      	      else {
      	        if (use_old_labels) label = 2; // Floor
      	        else label = LPFloor;
      	      }
            }
          }
      
          // Set label and segment number
	      segment++;
  	      if (debug) {
  	        printf("Segment %d with %d points from from %.1f to %.1f\n",
  	               segment, scan_segment->NumberOfPoints(),
			       start_point->FloatAttribute(AngleTag),
			       end_point->FloatAttribute(AngleTag));
  	      }
          for (point=start_point; point!=end_point+1; point++) {
            point->Attribute(SegmentNumberTag) = segment;
            point->Label(label);
          }
        }
  
      }
  else if (method==1)
  {
//------------------------------------------------------------------------------
// Find all segments by comparing a range of residuals to a range of thresholds
//------------------------------------------------------------------------------

// Test if this makes a difference (spoiler: it does not)
//  std::reverse(points.begin(),points.end()); 

	printf("forward: working..");
	clock_t begin = clock();
  rangeOfResidualsSegmentationForward(*points,ptsStart,percentagePts,threshold,min_size);
	clock_t end = clock();
	printf("done in %4.2f seconds\n",double(end-begin)/CLOCKS_PER_SEC);
	printf("backward: working..");
	begin = clock();
  std::reverse(points->begin(),points->end()); 						   	
  rangeOfResidualsSegmentationBackward(*points,ptsStart,percentagePts,threshold,min_size);
  std::reverse(points->begin(),points->end()); 
//	LaserPoints cp;
//    for (int i=points.size()-1;i!=0;i--) {
//      cp.push_back(points[i]);
//    }
//  rangeOfResidualsSegmentationBackward(cp,ptsStart,percentagePts,threshold,min_size);
//  	points.clear();
//    for (int i=cp.size()-1;i!=0;i--) {
//      points.push_back(cp[i]);
//    }
	end = clock();
	printf("done in %4.2f seconds\n",double(end-begin)/CLOCKS_PER_SEC);
	
	printf("removing small segments and labelling: working..");
	begin = clock();
  int seg_start=0,seg=-1;
  for(int i=0;i<points->size();i++)
  {
  	if(seg==-1 && (*points)[i].HasAttribute(SegmentNumberTag))
  	{
  		seg=(*points)[i].Attribute(SegmentNumberTag);
  		seg_start=i;
  	}
  	else if(seg!=-1 && (!(*points)[i].HasAttribute(SegmentNumberTag) || (*points)[i].Attribute(SegmentNumberTag)!=seg))
  	{
  		if(i-seg_start<min_size)
			for(int k=seg_start;k<i;k++)
				(*points)[k].RemoveAttribute(SegmentNumberTag);
		else
		{
		    if (use_old_labels) label = 0; // Wall
		    else label = LPWall;
		    if (scanner_id > 0) 
			{
		      direction = (*points)[i-1].vect() - (*points)[seg_start].vect();
		      if (fabs(direction.Z()) / direction.Length() < 0.2) {
		      	if ((*points)[seg_start].Z() + (*points)[i-1].Z() > 0.0) {
		      	  if (use_old_labels) label = 1; // Ceiling
		      	  else label = LPCeiling;
		      	}
		      	else {
		      	  if (use_old_labels) label = 2; // Floor
		      	  else label = LPFloor;
		      	}
		      }
		    }	
			for(int k=seg_start;k<i;k++)
				(*points)[k].Label(label);
		}
  		seg=-1;
  		if((*points)[i].HasAttribute(SegmentNumberTag) && (*points)[i].Attribute(SegmentNumberTag)!=seg)
  		{
	  		seg=(*points)[i].Attribute(SegmentNumberTag);
	  		seg_start=i;
  		}
  	}
  }
  
  seg=-1;
  int newseg=0;
  for(int i=0;i<(*points).size();i++)
  {
  	// segment changes if segmentnumber changes or if nosegment->segment
  	if(seg==-1 && (*points)[i].HasAttribute(SegmentNumberTag))
  	{
  		seg=(*points)[i].Attribute(SegmentNumberTag);
  		(*points)[i].Attribute(SegmentNumberTag)=newseg;
  	}
  	else if(seg!=-1 && (*points)[i].HasAttribute(SegmentNumberTag))
  	{
  		if((*points)[i].Attribute(SegmentNumberTag)!=seg)
  		{
	  		seg=(*points)[i].Attribute(SegmentNumberTag);
	  		newseg++;
  		}
  		(*points)[i].Attribute(SegmentNumberTag)=newseg;
  	}
  	else if(seg!=-1 && !(*points)[i].HasAttribute(SegmentNumberTag))
	{
		seg=-1;
		newseg++;
	}
  }
  
	end = clock();
	printf("done in %4.2f seconds\n",double(end-begin)/CLOCKS_PER_SEC);

  } // method
  
      // Add time offset for visualisation
      if (add_time_offset) {
        // Add offset perpendicular to scanning plane for better visualisation
        for (point=points->begin();
		     point<points->begin()+1000 && point<points->end(); point++)
          plane.AddPoint(point->Position3DRef(), false);
        plane.Recalculate();
        printf("normal %.2f %.2f %.2f\n", plane.Normal().X(),
               plane.Normal().Y(), plane.Normal().Z());
        for (point=points->begin(); point!=points->end(); point++)
          point->vect() += plane.Normal() * point->DoubleAttribute(TimeTag);
        printf("added offset to last point %.2f %.2f %.2f   Time %.4f\n",
               plane.Normal().X() * (points->end()-1)->DoubleAttribute(TimeTag),
               plane.Normal().Y() * (points->end()-1)->DoubleAttribute(TimeTag),
               plane.Normal().Z() * (points->end()-1)->DoubleAttribute(TimeTag),
	           (points->end()-1)->DoubleAttribute(TimeTag));
      }

      if (appendix) {
        if (!points->Name()) points->LaserDataFiles::DeriveName();
        newname = (char *) malloc(strlen(points->Name()) +
                                  strlen(appendix) + 1);
        sprintf(newname, "%s%s", points->Name(), appendix);
        points->SetName(newname);
        output_directory = DeriveDirectoryFromFile(points->PointFile());
        points->DeriveMetaDataFileName(output_directory);
        points->DerivePointFileName(output_directory);
        free(newname);
        points->Write(points->PointFile(), 0, false);
      }
	  else points->Write(out_file, 0, false); 
	  points->ErasePoints(); 

      // Clear all segments of this point set
      scan_segments.erase(scan_segments.begin(), scan_segments.end());
    }

    // Set strip meta data
    if (appendix) {
      if (!strip->Name()) strip->LaserDataFiles::DeriveName();
      newname = (char *) malloc(strlen(strip->Name()) + strlen(appendix) + 1);
      sprintf(newname, "%s%s", strip->Name(), appendix);
      strip->SetName(newname);
      output_directory = DeriveDirectoryFromFile(strip->MetaDataFile());
      strip->DeriveMetaDataFileName(output_directory);
      free(newname);
    }
  }
  
  // Set and write block meta data
  if (appendix) {
    if (!block.Name()) block.DeriveName();
    newname = (char *) malloc(strlen(block.Name()) + strlen(appendix) + 1);
    sprintf(newname, "%s%s", block.Name(), appendix);
    block.SetName(newname);
    output_directory = DeriveDirectoryFromFile(block.MetaDataFile());
    block.DeriveMetaDataFileName(output_directory);
    free(newname);
    block.WriteMetaData(1, 1);
  }
}
