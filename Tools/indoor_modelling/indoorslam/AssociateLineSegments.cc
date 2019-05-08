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

#include "LaserBlock.h"
#include "BSplineFit.h"

// Collect all segments in a time interval of all sensors
void CollectSegmentsInInterval(int interval, const LaserBlock &block, 
	                           int part_number,
							   LaserPoints::iterator **start_point,
							   LaserScanLines *segments)
{
  LaserPoints::const_iterator point, first_segment_point, last_segment_point,
                              end_point;
  bool                        first_point_is_set, debug=false;
  int                         sensor;
  LaserBlock::const_iterator  strip;
  LaserUnit::const_iterator   part;
  
 
  for (sensor=0, strip=block.begin(); sensor<3; sensor++, strip++) {

//    debug = (interval == 198 && sensor == 0);
    if (debug) printf("\n  Debugging CollectSegmentsInInterval\n\n");

    part = strip->begin() + part_number;
    first_point_is_set=false;
  
    // Clear old segments
    if (segments[sensor].size()) 
	  segments[sensor].erase(segments[sensor].begin(), segments[sensor].end());
  
    // Collect the segments
    end_point = start_point[sensor][interval+1];
    if (debug)
	  printf("Interval from %.8f till %.8f\n",
	         start_point[sensor][interval]->DoubleAttribute(TimeTag),
			 end_point->DoubleAttribute(TimeTag));
    for (point=start_point[sensor][interval]; point!=end_point; point++) {
      if (point->HasAttribute(SegmentNumberTag)) {
        if (first_point_is_set) { // First point of a segment is already known
          // Check if this point belongs to the same segment
      	  if (point->Attribute(SegmentNumberTag) ==
		      first_segment_point->Attribute(SegmentNumberTag)) {
		    // If so, add point to current segment
		    last_segment_point = point;
		  }
		  else {
		    // If not, store segment and start a new one
		    segments[sensor].push_back(LaserScanLine(*part, first_segment_point,
		                                             last_segment_point));
		    first_segment_point = last_segment_point = point;
		  }
        }
        else { // No first point set, so this is the start of a new segment
          first_segment_point = last_segment_point = point;
          first_point_is_set = true;
        }
      }
      else { // Current point has no segment number
        // If we were collecting points of a segment, this is the end of this segment.
        if (first_point_is_set) {
      	  // Store the current segment
		  segments[sensor].push_back(LaserScanLine(*part, first_segment_point,
		                                           last_segment_point));
	      first_point_is_set = false;
        }
      }
    }
    // Check if there is a current segment. If there is, the last point of this
    // scan is also the last point of this segment.
    if (first_point_is_set)
	  segments[sensor].push_back(LaserScanLine(*part, first_segment_point,
		                                       last_segment_point));
  }
}

bool AssignSegmentsToPlanes(LaserPoints &sensordata, int interval,
                            const LaserScanLines &segments, 
							Planes &planes,
							bool instantiate_new_planes,
							BSplineFit &omega_spline, BSplineFit &phi_spline,
                            BSplineFit &kappa_spline, BSplineFit &X_spline,
						    BSplineFit &Y_spline, BSplineFit &Z_spline,
							vector<DataBounds3D> &bounds_of_planes,
							int max_scans_for_loop_closure,
							bool only_2D, bool use_old_labels,
							double max_dist_segment_to_plane)
{
  LaserScanLines::const_iterator segment;
  Planes::iterator               plane, best_plane, second_best_plane;
  Plane                          new_plane;
  bool                           found, debug=false, ambiguous,
                                 new_assignment=false, vertical,
								 bound_test=true, merge=true;
  Position3D                     world_point, world_point2;
  int                            num_pts, i, plane_number,
                                 best_plane_number, second_best_plane_number;
  LaserPoints::iterator          point;
  double                         dist, min_dist, second_best_dist, time, scalar,
                                 angle, pi = 4.0 * atan(1.0), min_segment_scalar,
								 max_segment_scalar, min_bb_scalar, 
								 max_bb_scalar, overlap, bound_dist;
  Rotation3D                     rotation;
  Vector3D                       translation;
  Line3D                         line;
  Line2D                         line_in_XY_plane;
  LaserPoints                    points_on_line;
  DataBounds3D                   segment_bounds;
  vector<DataBounds3D>::iterator plane_bounds, second_best_plane_bounds;
  
  void FitLineSegment(const LaserPoints &, LaserScanLines::const_iterator,
                      BSplineFit &, BSplineFit &, BSplineFit &, BSplineFit &,
					  BSplineFit &, BSplineFit &, Line3D &);
  void SaveInitialPlanes(const LaserPoints &, LaserScanLines::const_iterator,
                         Line3D &, Plane &, BSplineFit &, BSplineFit &,
                         BSplineFit &, BSplineFit &, BSplineFit &, BSplineFit &);
				       
//  debug = ((interval == 496) && 
//          segments.begin()->begin(sensordata)->Attribute(ScanNumberTag) == 1);
//  debug = (interval <= 1);
    
  if (debug)
    printf("\nDebugging AssignSegmentsToPlane, interval %d, sensor %d\n\n",
            interval, segments.begin()->begin(sensordata)->Attribute(ScanNumberTag));
  
  if (debug) {
  	printf("Debugging in interval %d, instantiate new planes %d\n", interval, (int) instantiate_new_planes);
  	for (plane=planes.begin(), i=0; plane!=planes.end(); plane++, i++)
  	  printf("Plane %d normal %6.3f %6.3f %6.3f  dist %6.3f\n",
		     i, plane->Normal().X(), plane->Normal().Y(), plane->Normal().Z(),
			 plane->Distance());
	printf("List of segments:\n");
    for (segment=segments.begin(); segment!=segments.end(); segment++) {
      if (segment->begin(sensordata)->HasAttribute(PlaneNumberTag))
	    printf("Segment %d of sensor %d, length %.2f, %d points, assigned to plane %d\n",
		       segment->begin(sensordata)->Attribute(SegmentNumberTag),
		       segment->begin(sensordata)->Attribute(ScanNumberTag),
		       segment->Length(sensordata), segment->NumberOfPoints(),
			   segment->begin(sensordata)->Attribute(PlaneNumberTag));
	  else
	    printf("Segment %d of sensor %d, length %.2f, %d points, not assigned to a plane\n",
	 	       segment->begin(sensordata)->Attribute(SegmentNumberTag),
		       segment->begin(sensordata)->Attribute(ScanNumberTag),
		       segment->Length(sensordata), segment->NumberOfPoints());
    }
  }  
  // Loop over all segments
  for (segment=segments.begin(); segment!=segments.end(); segment++) {
  	
  	// Skip this segment if it's already assigned to a plane
  	if (segment->begin(sensordata)->HasAttribute(PlaneNumberTag)) continue;
  	if (debug)
	  printf("Checking segment %d\n",
	         segment->begin(sensordata)->Attribute(SegmentNumberTag));
  	found = false; ambiguous = false;
  	
  	// Try to make a new assignment if the length is at least 0.5 m
//  	if (segment->Length(sensordata) > 0.5) {
  	if (segment->Length(sensordata) > 0.2 &&
	    segment->NumberOfPoints() > 10) {
  		
  	  // Try to find a plane close to this segment
  	  if (debug && segment->NumberOfPoints() == 2) {
  	    printf("Segment %d of sensor %d searching nearby plane\n",
		       segment->begin(sensordata)->Attribute(SegmentNumberTag),
		       segment->begin(sensordata)->Attribute(ScanNumberTag));
		printf("Segment length %.2f with %d points\n",
		       segment->Length(sensordata), segment->NumberOfPoints());
      } 
	  
	  // Fit a line to the transformed segment  
      FitLineSegment(sensordata, segment, omega_spline, phi_spline,
                     kappa_spline, X_spline, Y_spline, Z_spline, line);

      // Determine the projections of the end points onto the fitted line
	  time = segment->begin(sensordata)->DoubleAttribute(TimeTag);
  	  rotation = Rotation3D(omega_spline.Value(time),
		                    phi_spline.Value(time), kappa_spline.Value(time));
	  translation = Vector3D(X_spline.Value(time), Y_spline.Value(time),
	                         Z_spline.Value(time));
  	  world_point.vect() = rotation * segment->begin(sensordata)->vect() +
  	                       translation;
      world_point = line.Project(world_point);
	  time = segment->end(sensordata)->DoubleAttribute(TimeTag);
  	  rotation = Rotation3D(omega_spline.Value(time),
		                    phi_spline.Value(time), kappa_spline.Value(time));
	  translation = Vector3D(X_spline.Value(time), Y_spline.Value(time),
	                         Z_spline.Value(time));
  	  world_point2.vect() = rotation * segment->end(sensordata)->vect() +
  	                       translation;
      world_point2 = line.Project(world_point2);

  	  min_dist = second_best_dist = 1e10; found = false;
  	  for (i=0, plane=planes.begin(), plane_bounds = bounds_of_planes.begin();
		   plane!=planes.end(); i++, plane++, plane_bounds++) {

        // Skip this plane if it has been merged with another plane. In that
        // case the plane number doesn't equal the plane index.
        if (plane->Number() != i) continue;

        // Check if we can link back to this plane if loop closures are not allowed
        if (max_scans_for_loop_closure > 0 &&
			interval - plane->Attribute(PT_LastScanLine) > max_scans_for_loop_closure)
          continue;			

        // Determine distance of end points to plane
  	  	dist = fabs(plane->Distance(world_point));
  	  	dist += fabs(plane->Distance(world_point2));
  	  	dist /= 2.0; // Average distance of end points
  	  	if (debug && dist < 0.5) printf("Plane %i, dist %6.3f\n", i, dist);
  	  	// No more than 2.0 * max_dist_segment_to_plane for second best distance
  	  	// Reduction to 1.5 lead to errors in long corridor
		if (dist > 2.0 * max_dist_segment_to_plane) continue;
		
		// Discard this plane if it cannot be seen from the scanner's
		// location (the location is in variable translation)
		// This check is important for thin walls
		if (plane->Distance(Position3D(translation)) < 0.0) {
		  if (debug) printf("Plane refused because of normal vector direction\n");
		  continue;
	    }
		
		// Check the angle with the normal vector (should be > 80 degrees)
		// Angle of 85 led to insufficient observations in Diemen1
		angle = Angle(plane->Normal(), world_point2.vect() - world_point.vect());
		if (angle > pi / 2.0) angle = pi - angle;
		if (angle < 80.0 * pi / 180.0) {
		  if (debug) printf("Plane refused because of angle %6.2f\n", angle * 180.0 / pi);
		  continue;
	    }

        // Also check the horizontal overlap of the segment with the bounding
        // box of the wall
        if (bound_test && fabs(plane->Normal().Z()) < 0.99) {
          // Project line onto XY-plane
          line_in_XY_plane = line.ProjectOntoXOYPlane();
          
		  // Determine scalar range of the end points
          min_segment_scalar = max_segment_scalar =
		    line_in_XY_plane.Scalar(world_point.Position2DOnly());
          scalar = line_in_XY_plane.Scalar(world_point2.Position2DOnly());
          if (scalar < min_segment_scalar) min_segment_scalar = scalar;
          else max_segment_scalar = scalar;
          
          // Determin scalar range of the bounding box
          min_bb_scalar = max_bb_scalar = 
            line_in_XY_plane.Scalar(plane_bounds->Minimum().Position2DOnly());
          scalar = line_in_XY_plane.Scalar(plane_bounds->Maximum().Position2DOnly());
          if (scalar < min_bb_scalar) min_bb_scalar = scalar;
          else if (scalar > max_bb_scalar) max_bb_scalar = scalar;
          scalar = line_in_XY_plane.Scalar(Position2D(plane_bounds->Minimum().X(),
		                                              plane_bounds->Maximum().Y()));
          if (scalar < min_bb_scalar) min_bb_scalar = scalar;
          else if (scalar > max_bb_scalar) max_bb_scalar = scalar;
          scalar = line_in_XY_plane.Scalar(Position2D(plane_bounds->Maximum().X(),
		                                              plane_bounds->Minimum().Y()));
          if (scalar < min_bb_scalar) min_bb_scalar = scalar;
          else if (scalar > max_bb_scalar) max_bb_scalar = scalar;
          
          // Determine if there is overlap
          overlap = fmax(0.0, fmin(max_segment_scalar, max_bb_scalar) -
		                 fmax(min_segment_scalar, min_bb_scalar));
		  if (overlap < 0.01) {
		    if (debug) 
			  printf("No overlap, %.2f gap between segment %d and bounding box of plane %d\n",
			         fmax(min_segment_scalar, min_bb_scalar) - 
					 fmin(max_segment_scalar, max_bb_scalar),
					 segment->begin(sensordata)->Attribute(SegmentNumberTag), i);
		    continue;
		  }
        }
        
        // For a horizontal plane, check if the segment is near the bounding box
        else if (bound_test) {
          // Create bounds of segment
          segment_bounds.Initialise();
          segment_bounds.Update(world_point);
          segment_bounds.Update(world_point2);
          // Get distance to bounding box of plane
          bound_dist = plane_bounds->DistanceXY(segment_bounds);
          if (bound_dist > 0.5) {
          	if (debug)
          	  printf("Distance of %.2f between segment %d and bounding box of plane %d\n",
			         bound_dist, segment->begin(sensordata)->Attribute(SegmentNumberTag), i);
			continue;
          }
        }

		found = true;
		if (dist < min_dist) {
		  if (min_dist < second_best_dist) {
		  	second_best_dist = min_dist;
		  	second_best_plane = best_plane;
		  }
		  min_dist = dist;
		  best_plane = plane;
	    }
	    else {
	      if (dist < second_best_dist) {
	      	second_best_dist = dist;
	      	second_best_plane = plane;
	      }
	    }
	    if (debug) {
	      if (second_best_dist < 1e10)
	        printf("Best plane %d (%.3f), second best plane %d (%.3f)\n",
	               best_plane->Number(), min_dist,
	               second_best_plane->Number(), second_best_dist);
	      else
	        printf("Best plane %d (%.3f), no second best plane\n",
	               best_plane->Number(), min_dist);
	    }
  	  }
  	  found = (min_dist < max_dist_segment_to_plane);
  	  if (found) {
  	    // Check if the difference between the best and second best solution
  	    // is significant
  	    if (second_best_dist < 2.0 * max_dist_segment_to_plane) {
  	      // Check if the two best planes should be merged based on angle
  	      // The distances to the line segment already have been tested.
  	      angle = Angle(best_plane->Normal(), second_best_plane->Normal());
  	      if (debug)
  	        printf("Best plane %d, second best plane %d with angle %.2f\n",
			       best_plane->Number(), second_best_plane->Number(),
				   angle * 180.0 / pi);
  	      found = ((angle * 180.0 / pi < 5.0) && merge);
 	      
  	      // Merge the two planes and update all point to plane assignments
  	      if (found) {
  	      	new_assignment = true; // Treat merging as new plane assignment
 	        printf("Merging planes %d (%d points) and %d (%d points)\n",
		           best_plane->Number(), best_plane->Attribute(PT_NumberOfPoints),
		           second_best_plane->Number(),
				   second_best_plane->Attribute(PT_NumberOfPoints));

  	      	// Keep the plane with the largest number of points
  	      	if (best_plane->Attribute(PT_NumberOfPoints) <
  	      	    second_best_plane->Attribute(PT_NumberOfPoints)) {
  	      	  // Swap the two planes
  	      	  plane = best_plane; best_plane = second_best_plane;
  	      	  second_best_plane = plane;
  	      	}
  	      	printf("Best plane %d, interval range %d-%d, %d points, normal (%.2f %.2f %.2f), distance %.2f\n",
				   best_plane->Number(), best_plane->Attribute(PT_FirstScanLine),
				   best_plane->Attribute(PT_LastScanLine),
				   best_plane->Attribute(PT_NumberOfPoints),
				   best_plane->Normal().X(), best_plane->Normal().Y(),
				   best_plane->Normal().Z(), best_plane->Distance());
  	      	printf("Second best plane %d, interval range %d-%d, %d points, normal (%.2f %.2f %.2f), distance %.2f\n",
				   second_best_plane->Number(), 
				   second_best_plane->Attribute(PT_FirstScanLine),
				   second_best_plane->Attribute(PT_LastScanLine),
				   second_best_plane->Attribute(PT_NumberOfPoints),
				   second_best_plane->Normal().X(), second_best_plane->Normal().Y(),
				   second_best_plane->Normal().Z(), second_best_plane->Distance());
  	      	
  	      	// Merge the bounding boxes
  	      	plane_bounds = bounds_of_planes.begin() + best_plane->Number();
  	      	second_best_plane_bounds = bounds_of_planes.begin() +
				                       second_best_plane->Number();
			plane_bounds->Update(*second_best_plane_bounds);
			
			// Combine attributes
			if (best_plane->Attribute(PT_FirstScanLine) >
			    second_best_plane->Attribute(PT_FirstScanLine))
			  best_plane->Attribute(PT_FirstScanLine) =
			    second_best_plane->Attribute(PT_FirstScanLine);
			if (best_plane->Attribute(PT_LastScanLine) <
			    second_best_plane->Attribute(PT_LastScanLine))
			  best_plane->Attribute(PT_LastScanLine) =
			    second_best_plane->Attribute(PT_LastScanLine);
            best_plane->Attribute(PT_NumberOfPoints) +=
              second_best_plane->Attribute(PT_NumberOfPoints);
  	      	printf("Merged plane %d, interval range %d-%d, %d points\n",
				   best_plane->Number(), best_plane->Attribute(PT_FirstScanLine),
				   best_plane->Attribute(PT_LastScanLine),
				   best_plane->Attribute(PT_NumberOfPoints));

            // Renumber all planes with the number of the second best plane
            // to the number of the best plane
            best_plane_number = best_plane->Number();
			second_best_plane_number = second_best_plane->Number();
			for (plane=planes.begin(); plane!=planes.end(); plane++)
			  if (plane->Number() == second_best_plane_number)
			    plane->Number() = best_plane_number;
  	      }
  	      else {
  	  	    if (debug) printf("Multiple nearby planes, decision postponed\n");
   	        ambiguous = true;
  	      }
  	    }
  	    // If a plane is found, assign the component to this plane
  	    if (found) {
  	      if (debug)
  	  	    printf("Assigned segment %d to plane %d in interval %d of sensor %d\n", 
			       segment->begin(sensordata)->Attribute(SegmentNumberTag),
			       std::distance(planes.begin(), best_plane),
			       interval,
			       segment->begin(sensordata)->Attribute(ScanNumberTag));
  	    }
  	  }
  	  
  	  // Otherwise, create a new plane if the line is at least 2.0 m long
  	  // and has at least 20 points
  	  if (!found && !ambiguous && instantiate_new_planes && 
//		  segment->Length(sensordata) > 2.0 &&
		  segment->Length(sensordata) > 0.5 &&
		  segment->NumberOfPoints() >= 20) {
  	  	// Check if the line fit is accurate. If not, quit the loop and do not
  	  	// create a new plane
  	  	if (line.SigmaLineFitting() > 0.05) {
  	  	  printf("High sigma0 in line fitting %6.2f in interval %d. No plane created.\n",
			     line.SigmaLineFitting(), interval);
  	  	  continue;
  	  	}
  	  	// Skip this line segment if the angle between the two end points and
  	  	// the scanner location is very small (< 20 degrees)
  	  	// translation contains the scanner location
  	  	if (world_point.Distance(Position3D(translation)) <
		    world_point2.Distance(Position3D(translation))) 
  	  	  angle = Angle(translation - world_point.vect(),
			            world_point2.vect() - world_point.vect());
		else   
  	  	  angle = Angle(translation - world_point2.vect(),
			            world_point.vect() - world_point2.vect());
		if (angle > 160.0 * pi / 180.0) {
		  if (debug)
		    printf("No plane created because of small intersection angle with wall %.2f\n",
		           180.0 - angle * 180.0 / pi);
		  continue;
		}
			          
 	  	// Determine if the plane is vertical or horizontal
 	  	if (segment->begin(sensordata)->Attribute(ScanNumberTag) == 0) {
 	  	  vertical = true;
 	  	}
 	  	else {
 	  	  // Check if the line segment has a vertical component
 	  	  // Occasionally, this may be a wrong decision!
 	      // Check on slope and height difference for horizontal plane
 	      if (fabs(line.Direction().Z()) < 0.02 &&
 	      	  fabs(world_point.Z() - world_point2.Z()) < 0.10) 
 	      	vertical = false;
 	      // Check on slope and height difference for vertical plane
 	      else if (fabs(line.Direction().Z()) > 0.20 && // Some vertical extent
		           (world_point - world_point2).Length2D() > 0.20) // Some horizontal extent
//		           fabs(world_point.Z() - world_point2.Z()) < 0.50)
		    vertical = true;
		  // Don't instantiate a new plane if it's not clearly vertical
		  // or horizontal
		  else {
		  	if (debug) {
		  	  printf("Segment not used to create new plane\n");
		  	}
		    continue;
		  }
		  // Don't generate horizontal planes for 2D pose estimation
 	  	  if (only_2D && !vertical) continue;
 	  	  
// Old code for deciding on horizontal / vertical planes
// 	  	  if (only_2D) {
// 	  	    vertical = (fabs(line.Direction().Z()) > 0.2);
// 	  	    // Don't generate horizontal planes for 2D pose estimation
// 	  	    if (!vertical) continue;
// 	      }
// 	      else {
// 	  	    vertical = (fabs(line.Direction().Z()) > 0.02);
// 	  	  }

	  	}
		new_plane.Erase();
 	  	
  	  	// Case: vertical plane (wall)
 	  	if (vertical) {  // Wall segment
  	  	  new_plane.Normal() = line.Direction().VectorProduct(Vector3D(0.0, 0.0, 1.0)).Normalize();
  	  	  new_plane.Distance() = new_plane.Normal().DotProduct(line.FootPoint());
  	  	  // All code below is for debugging output only
  	  	  if (debug) {
  	  	    printf("New vertical plane, distances to start point to other planes\n");
  	  	    // Transform both end points to the world c.s.
  	  	    time = segment->begin(sensordata)->DoubleAttribute(TimeTag);
  	  	    rotation = Rotation3D(omega_spline.Value(time),
			                      phi_spline.Value(time), kappa_spline.Value(time));
	  	    translation = Vector3D(X_spline.Value(time), Y_spline.Value(time),
		                           Z_spline.Value(time));
  	  	    world_point.vect() = rotation * segment->begin(sensordata)->vect() +
  	  	                         translation;
  	  	    printf("(%6.2f, %6.2f, %6.2f) ", world_point.X(),
  	  	           world_point.Y(), world_point.Z()); 
  	  	    for (i=0, plane=planes.begin(); plane!=planes.end(); i++, plane++)
              printf("%7.2f ", fabs(plane->Distance(world_point)));
            printf("\n");
  	  	    // new_plane.AddPoint(world_point, false);
  	  	    time = segment->end(sensordata)->DoubleAttribute(TimeTag);
  	  	    rotation = Rotation3D(omega_spline.Value(time),
			                      phi_spline.Value(time), kappa_spline.Value(time));
	  	    translation = Vector3D(X_spline.Value(time), Y_spline.Value(time),
		                           Z_spline.Value(time));
  	  	    world_point.vect() = rotation * segment->end(sensordata)->vect() +
  	  	                         translation;
  	  	    printf("(%6.2f, %6.2f, %6.2f) ", world_point.X(),
  	  	           world_point.Y(), world_point.Z()); 
  	  	    for (i=0, plane=planes.begin(); plane!=planes.end(); i++, plane++)
              printf("%7.2f ", fabs(plane->Distance(world_point)));
            printf("\n");
  	      }
  	    }
  	  	// Case: horizontal plane (floor or ceiling)
  	  	else if ((use_old_labels && segment->begin(sensordata)->Label() != 0) ||
			     (!use_old_labels && segment->begin(sensordata)->Label() != LPWall)) {
  	  	  // Add plane at the average height of the end points
  	  	  time = segment->begin(sensordata)->DoubleAttribute(TimeTag);
  	  	  rotation = Rotation3D(omega_spline.Value(time),
			                    phi_spline.Value(time), kappa_spline.Value(time));
	  	  translation = Vector3D(X_spline.Value(time), Y_spline.Value(time),
		                         Z_spline.Value(time));
  	  	  world_point.vect() = rotation * segment->begin(sensordata)->vect() +
  	  	                       translation;
  	  	  scalar = line.Scalar(world_point);
  	  	  time = segment->end(sensordata)->DoubleAttribute(TimeTag);
  	  	  rotation = Rotation3D(omega_spline.Value(time),
			                    phi_spline.Value(time), kappa_spline.Value(time));
	  	  translation = Vector3D(X_spline.Value(time), Y_spline.Value(time),
		                         Z_spline.Value(time));
  	  	  world_point.vect() = rotation * segment->begin(sensordata)->vect() +
  	  	                       translation;
		  scalar += line.Scalar(world_point);
		  world_point = line.Position(scalar / 2.0);
          new_plane = Plane(world_point.vect(), Vector3D(0.0, 0.0, 1.0));
  	  	}
  	  	else {
  	  	  printf("Unassigned horizontal segment %d with wall label of sensor %d, height difference %.4f, slope %.4f\n",
  	  	         segment->begin(sensordata)->Attribute(SegmentNumberTag),
				 segment->begin(sensordata)->Attribute(ScanNumberTag),
  	  	         fabs(world_point.Z() - world_point2.Z()),
  	  	         fabs(line.Direction().Z()));
  	  	  printf("Can't decide what to instantiate, skipping this segment.\n");
  	  	  continue;
  	  	}
  	  	// Add plane and assign component number to this plane
  	  	new_plane.Number() = planes.size();
  	  	new_plane.Attribute(PT_FirstScanLine) = interval;
  	  	new_plane.Attribute(PT_LastScanLine) = interval;
  	  	new_plane.Attribute(PT_NumberOfPoints) = 0;
  	  	// Make sure the normal is pointing towards the scanner location
  	  	time = segment->begin(sensordata)->DoubleAttribute(TimeTag);
	  	translation = Vector3D(X_spline.Value(time), Y_spline.Value(time),
		                       Z_spline.Value(time));
 	  	if (new_plane.Distance(Position3D(translation)) < 0.0)
  	  	  new_plane.SwapNormal();
  	  	planes.push_back(new_plane);
  	  	best_plane = planes.end() - 1;
  	  	found = true;
  	  	// Save the planes
        SaveInitialPlanes(sensordata, segment, line, new_plane,
				          omega_spline, phi_spline, kappa_spline, X_spline,
				          Y_spline, Z_spline);  	  	
        // Initialise bounds
        bounds_of_planes.push_back(DataBounds3D());
  	  }
  	}
  	
  	// Set plane number for all points of this segment if a plane has been found
  	if (found && !(segment->begin(sensordata)->HasAttribute(PlaneNumberTag))) {
  	  plane_number = best_plane->Number();
	  if (plane_number == -1) {
	  	printf("Incorrect plane number -1 set!\n");
	  }
      num_pts = segment->NumberOfPoints();
      for (i=0, point=segment->begin(sensordata); i<num_pts; i++, point++)
  	    point->Attribute(PlaneNumberTag) = plane_number;
  	  // Update plane bounds with the two transformed end points
	  plane_bounds = bounds_of_planes.begin() + plane_number;
	  world_point.vect() = rotation * segment->begin(sensordata)->vect() +
  	  	                   translation;
      plane_bounds->Update(world_point);
	  world_point.vect() = rotation * segment->end(sensordata)->vect() +
  	  	                   translation;
      plane_bounds->Update(world_point);
      // Update plane attributes
      best_plane->Attribute(PT_LastScanLine) = interval;
  	  best_plane->Attribute(PT_NumberOfPoints) += num_pts;
      // Report a new assignment
      new_assignment = true;
      if (debug) printf("New assignment of segment %d of sensor %d to plane %d\n",
	                    segment->begin(sensordata)->Attribute(SegmentNumberTag),
						segment->begin(sensordata)->Attribute(ScanNumberTag),
						plane_number);
  	}
  }
  
  // Report whether a segment has been assigned to a plane
  return new_assignment;
}


// Fit line segment after time-dependent transformation

void FitLineSegment(const LaserPoints &sensordata,
                    LaserScanLines::const_iterator segment,
                    BSplineFit &omega_spline, BSplineFit &phi_spline,
					BSplineFit &kappa_spline, BSplineFit &X_spline,
					BSplineFit &Y_spline, BSplineFit &Z_spline,
					Line3D &line)
{
  LaserPoints::const_iterator point;
  double                      time;
  Rotation3D                  rotation;
  Vector3D                    translation;
  Position3D                  world_point;
  
  // Clear all line data
  line.Erase();
  // Loop over all points in a segment
  for (point=segment->begin(sensordata);
	   point!=segment->end(sensordata)+1; point++) {
  	// Retrieve rotation and translation at the time of the point
	time = point->DoubleAttribute(TimeTag);
  	rotation = Rotation3D(omega_spline.Value(time),
			              phi_spline.Value(time), kappa_spline.Value(time));
	translation = Vector3D(X_spline.Value(time), Y_spline.Value(time),
		                   Z_spline.Value(time));
	// Transform point from frame to object coordinate system
  	world_point.vect() = rotation * point->vect() + translation;
  	// Add point to the line
  	line.AddPoint(world_point, false);
  }
  // Estimate the line parameters
  line.Recalculate();
}

