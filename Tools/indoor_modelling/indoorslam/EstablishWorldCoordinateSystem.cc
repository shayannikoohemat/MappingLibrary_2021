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

void EstablishWorldCoordinateSystem(LaserBlock &block, int start_part_number,
                                    LaserScanLines *segments,
                                    Planes &planes, Rotation3D &rotation,
                                    Vector3D &translation,
									vector<DataBounds3D> &bounds_of_planes,
									bool only_2D, bool use_old_labels)
{
  LaserScanLines::iterator segment, horizontal1, horizontal2, wall0, wall1,
                           segment0, segment1;
  double                   best_wall_area, wall_area, segment_length,
                           scalar_b, scalar_e, scalar;
  Vector3D                 w0, w1, h1, h2, horizontal_normal, wall_normal,
                           x_axis, y_axis;
  Plane                    plane;
  Line3D                   wline0, wline1, hline1, hline2;
  bool                     done;
  int                      i, j, num_pts, label, sensor;
  LaserPoints::iterator    point;
  Position3D               line_foot_point, world_point;
  LaserUnit::iterator      sensor0_data, sensor1_data, sensor2_data, sensor_data;
  LineSegment3D            wsegment0, wsegment1;
  DataBounds3D		       wall_bounds;
  
  void SaveInitialPlanes(LaserPoints &, LaserScanLines::const_iterator ,
                         Line3D &, Plane &, const Rotation3D &, const Vector3D &);
                         
  // Pointers to point sets of the three sensors
  sensor0_data = block[0].begin() + start_part_number;
  sensor1_data = block[1].begin() + start_part_number;
  sensor2_data = block[2].begin() + start_part_number;
  
  // Find the longest ceiling or floor segments in sensor 1 and 2 data
  segment_length = 0.0;
  if (use_old_labels) label = 0;
  else label = LPWall;
  for (segment=segments[1].begin(); segment!=segments[1].end(); segment++) {
  	if (segment->begin(*sensor1_data)->Label() != label) {
  	  if (segment->NumberOfPoints() > segment_length) {
  	  	segment_length = segment->NumberOfPoints();
  	  	horizontal1 = segment;
  	  }
  	}
  }
  if (segment_length == 0.0) {
  	printf("Error: no ceiling or floor segment in sensor 1 data.\n");
  	exit(0);
  }
  segment_length = 0.0;
  for (segment=segments[2].begin(); segment!=segments[2].end(); segment++) {
  	if (segment->begin(*sensor2_data)->Label() != label) {
  	  if (segment->NumberOfPoints() > segment_length) {
  	  	segment_length = segment->NumberOfPoints();
  	  	horizontal2 = segment;
  	  }
  	}
  }
  if (segment_length == 0.0) {
  	printf("Error: no ceiling or floor segment in sensor 2 data.\n");
  	exit(0);
  }
 
  // Determine the normal vector of a horizontal plane
  hline1 = sensor1_data->FitLine(horizontal1->begin(*sensor1_data),
                                 horizontal1->end(*sensor1_data));
  h1 = hline1.Direction();
  printf("Horizontal 1 direction %.2f %.2f %.2f, %d points, segment %d, sensor %d\n",
         h1.X(), h1.Y(), h1.Z(), horizontal1->NumberOfPoints(),
		 horizontal1->begin(*sensor1_data)->Attribute(SegmentNumberTag),
		 horizontal1->begin(*sensor1_data)->Attribute(ScanNumberTag));
  hline2 = sensor2_data->FitLine(horizontal2->begin(*sensor2_data),
                                 horizontal2->end(*sensor2_data));
  h2 = hline2.Direction();
  printf("Horizontal 2 direction %.2f %.2f %.2f, %d points, segment %d, sensor %d\n",
         h2.X(), h2.Y(), h2.Z(), horizontal2->NumberOfPoints(),
		 horizontal2->begin(*sensor2_data)->Attribute(SegmentNumberTag),
		 horizontal2->begin(*sensor2_data)->Attribute(ScanNumberTag));
  horizontal_normal = h1.VectorProduct(h2);
  horizontal_normal = horizontal_normal / horizontal_normal.Length();
  if (horizontal_normal.Z() < 0.0) horizontal_normal *= -1.0;
  printf("Horizontal normal %.2f %.2f %.2f\n", horizontal_normal.X(), 
         horizontal_normal.Y(), horizontal_normal.Z());
 
  // Determing intersecting scan segments on a wall
  // Find the crossing segment pair with the longest smaller segment
  best_wall_area = 0.0; 
  for (segment0=segments[0].begin(); segment0!=segments[0].end(); segment0++) {
  	// Fit line to segment points
    wline0 = sensor0_data->FitLine(segment0->begin(*sensor0_data),
	                               segment0->end(*sensor0_data));
	// Get end locations
	scalar_b = wline0.Scalar(segment0->begin(*sensor0_data)->Position3DRef());
	scalar_e = wline0.Scalar(segment0->end(*sensor0_data)->Position3DRef());
	if (scalar_b > scalar_e) { // Swap begin and end scalars
	  scalar = scalar_b; scalar_b = scalar_e; scalar_e = scalar;
	}
	// Create 3D line segment
	wsegment0 = LineSegment3D(wline0, scalar_b, scalar_e);
    // Check segments of sensors 1 and 2
	for (sensor=1; sensor<3; sensor++) {
	  if (sensor == 1) sensor_data = sensor1_data;
	  else sensor_data = sensor2_data;
      for (segment1=segments[sensor].begin(); segment1!=segments[sensor].end(); segment1++) {
        if ((use_old_labels && segment1->begin(*sensor_data)->Label() == 0) ||
	        (!use_old_labels && segment1->begin(*sensor_data)->Label() == LPWall)) { // Wall segment
  	      // Fit line to segment points
          wline1 = sensor_data->FitLine(segment1->begin(*sensor_data),
	                                    segment1->end(*sensor_data));
	      // Get end locations
	      scalar_b = wline1.Scalar(segment1->begin(*sensor_data)->Position3DRef());
	      scalar_e = wline1.Scalar(segment1->end(*sensor_data)->Position3DRef());
	      if (scalar_b > scalar_e) { // Swap begin and end scalars
	        scalar = scalar_b; scalar_b = scalar_e; scalar_e = scalar;
	      }
	      // Create 3D line segment
	      wsegment1 = LineSegment3D(wline1, scalar_b, scalar_e);
  	      if (Distance2Lines(wline0, wline1) < 0.05 && // Coplanar
			  wsegment0.Distance(wsegment1) < 0.5) { // Not too far apart
			// Determine wall bounding box
			wall_bounds.Initialise();
			wall_bounds.Update(wsegment0.BeginPoint());
			wall_bounds.Update(wsegment0.EndPoint());
			wall_bounds.Update(wsegment1.BeginPoint());
			wall_bounds.Update(wsegment1.EndPoint());
			wall_area = sqrt(wall_bounds.XRange() * wall_bounds.XRange() +
			                 wall_bounds.YRange() * wall_bounds.YRange()) *
			            wall_bounds.ZRange();
  	        if (wall_area > best_wall_area) {
  	      	  best_wall_area = wall_area;
  	      	  wall0 = segment0;
  	      	  wall1 = segment1;
  	        }
          }
  	    }
 	  }
  	}
  }
  
  if (best_wall_area == 0.0) {
  	printf("No crossing wall line found!\n");
  	exit(0);
  }
  
  // Determine the wall normal vector after re-fitting wall points
  wline0 = sensor0_data->FitLine(wall0->begin(*sensor0_data),
                                 wall0->end(*sensor0_data));
  wline1 = sensor_data->FitLine(wall1->begin(*sensor_data),
                                wall1->end(*sensor_data));
  w0 = wline0.Direction();
  w0 = w0 / w0.Length();
  printf("Wall line 0 direction %.2f %.2f %.2f, %d points, segment %d, sensor %d\n",
         w0.X(), w0.Y(), w0.Z(), wall0->NumberOfPoints(),
		 wall0->begin(*sensor0_data)->Attribute(SegmentNumberTag),
		 wall0->begin(*sensor0_data)->Attribute(ScanNumberTag));
  w1 = wline1.Direction();
  w1 = w1 / w1.Length();
  printf("Wall line 1 direction %.2f %.2f %.2f, %d points, segment %d, sensor %d\n",
         w1.X(), w1.Y(), w1.Z(), wall1->NumberOfPoints(),
		 wall1->begin(*sensor_data)->Attribute(SegmentNumberTag),
		 wall1->begin(*sensor_data)->Attribute(ScanNumberTag));
  wall_normal = w0.VectorProduct(w1);
  wall_normal = wall_normal / wall_normal.Length();
  printf("Wall normal %8.4f %8.4f %8.4f\n", wall_normal.X(), 
         wall_normal.Y(), wall_normal.Z());
         
  // Derive the X-axis
  x_axis = wall_normal.VectorProduct(horizontal_normal);
  x_axis = x_axis / x_axis.Length();
  if (x_axis.X() < 0.0) x_axis *= -1.0;
  printf("X-axis %.2f %.2f %.2f\n", x_axis.X(), x_axis.Y(), x_axis.Z());
  
  // Derive the Y-axis
  y_axis = horizontal_normal.VectorProduct(x_axis);
  printf("Y-axis %.2f %.2f %.2f\n", y_axis.X(), y_axis.Y(), y_axis.Z());
  
  // Compose the rotation matrix from the frame to the world coordinate system
  rotation = Rotation3D(x_axis.X(), y_axis.X(), horizontal_normal.X(),
                        x_axis.Y(), y_axis.Y(), horizontal_normal.Y(),
                        x_axis.Z(), y_axis.Z(), horizontal_normal.Z());
  rotation = rotation.Transpose();
  printf("Rotation matrix frame to world\n");
  for (i=0; i<3; i++) {
    for (j=0; j<3; j++)
      printf(" %6.2f", rotation.R(i,j));
    printf("\n");
  }
  
  // The origin of the world coordinate system is the origin of the first
  // frame location
  translation = Vector3D(0.0, 0.0, 0.0);

  // For the horizontal plane, take the height of the middle of the longer
  // segment
  if (!only_2D) {
    if (horizontal1->Length(*sensor1_data) >  horizontal2->Length(*sensor2_data))
      line_foot_point = hline1.Position(0.0);
    else
      line_foot_point = hline2.Position(0.0);
    line_foot_point.vect() = rotation * line_foot_point.vect();
    printf("Horizontal plane at height %6.2f\n", line_foot_point.Z());
    plane.SetDistance(line_foot_point.Z());
    plane.SetNormal(rotation * horizontal_normal);
    printf("Horizontal normal %6.2f %6.2f %6.2f\n", plane.Normal().X(),
           plane.Normal().Y(), plane.Normal().Z());
    plane.Number() = 0;
    plane.Attribute(PT_FirstScanLine) = 0;
    plane.Attribute(PT_LastScanLine) = 0;
    plane.Attribute(PT_NumberOfPoints) = horizontal1->NumberOfPoints() +
	                                     horizontal2->NumberOfPoints();
    // Make sure the normal points towards the origin (= initial scanner position)
    if (plane.Distance(Position3D(0.0, 0.0, 0.0)) < 0.0) plane.SwapNormal();
    planes.push_back(plane);
    // Save the planes
    if (horizontal1->Length(*sensor1_data) >  horizontal2->Length(*sensor2_data))
      SaveInitialPlanes(*sensor1_data, horizontal1, hline1, plane, rotation, translation);
    else
      SaveInitialPlanes(*sensor2_data, horizontal2, hline2, plane, rotation, translation);
    bounds_of_planes.push_back(DataBounds3D());
    // Set plane numbers for all points of the segment used for the horizontal plane
    if (horizontal1->Length(*sensor1_data) >  horizontal2->Length(*sensor2_data)) {
      num_pts = horizontal1->NumberOfPoints();
      for (i=0, point=horizontal1->begin(*sensor1_data); i<num_pts; i++, point++)
  	    point->Attribute(PlaneNumberTag) = 0;
  	}
  	else {
      num_pts = horizontal2->NumberOfPoints();
      for (i=0, point=horizontal1->begin(*sensor2_data); i<num_pts; i++, point++)
  	    point->Attribute(PlaneNumberTag) = 0;
    }
  }
  
  // Add the initial wall plane in the world coordinate system by estimating
  // planes through all points of the involved scan lines and rotating the
  // plane's normal vector
  plane.Erase();
  num_pts = wall0->NumberOfPoints();
  for (i=0, point=wall0->begin(*sensor0_data); i<num_pts; i++, point++)
    plane.AddPoint(point->Position3DRef(), false);
  num_pts = wall1->NumberOfPoints();
  for (i=0, point=wall1->begin(*sensor_data); i<num_pts; i++, point++)
    plane.AddPoint(point->Position3DRef(), false);
  plane.Recalculate();
  printf("Wall normal before rotation %8.4f %8.4f %8.4f\n", plane.Normal().X(),
         plane.Normal().Y(), plane.Normal().Z());
  wall_normal = rotation * plane.Normal();
  wall_normal.Z() = 0.0; // Force vertical wall
  wall_normal = wall_normal / wall_normal.Length();
  plane.SetNormal(wall_normal);
  printf("Wall normal, horizontal and rotated %8.4f %8.4f %8.4f\n", plane.Normal().X(),
         plane.Normal().Y(), plane.Normal().Z());
  if (only_2D) plane.Number() = 0;
  else plane.Number() = 1;
  plane.Attribute(PT_FirstScanLine) = 0;
  plane.Attribute(PT_LastScanLine) = 0;
  plane.Attribute(PT_NumberOfPoints) = 0;
  // Make sure the normal points towards the origin (= initial scanner position)
  if (plane.Distance(Position3D(0.0, 0.0, 0.0)) < 0.0) plane.SwapNormal(); //GV
  planes.push_back(plane);
  // Save the planes
  SaveInitialPlanes(*sensor0_data, wall0, wline0, plane, rotation, translation);
  // Initialise wall bounds 
  wall_bounds.Initialise();
  world_point.vect() = rotation * wall0->begin(*sensor0_data)->vect() +
  	  	               translation;
  wall_bounds.Update(world_point);
  world_point.vect() = rotation * wall0->end(*sensor0_data)->vect() +
  	  	               translation;
  wall_bounds.Update(world_point);
  world_point.vect() = rotation * wall1->begin(*sensor_data)->vect() +
  	  	               translation;
  wall_bounds.Update(world_point);
  world_point.vect() = rotation * wall1->end(*sensor_data)->vect() +
  	  	               translation;
  wall_bounds.Update(world_point);
  bounds_of_planes.push_back(wall_bounds);
  // Set plane numbers for all points
  num_pts = wall0->NumberOfPoints();
  for (i=0, point=wall0->begin(*sensor0_data); i<num_pts; i++, point++)
    point->Attribute(PlaneNumberTag) = 1;
  num_pts = wall1->NumberOfPoints();
  for (i=0, point=wall1->begin(*sensor_data); i<num_pts; i++, point++)
    point->Attribute(PlaneNumberTag) = 1;
}

  // Set six plane parameters as reference parameters
  // - Normal of horizontal plane: 2 par
  // - Distance of horizontal plane to origin: 1 par
  // - Azimuth of one vertical wall plane: 1 par
  // - Distance of this wall to origin: 1 par
  // - Distance of a further independent plane to origin: 1 par
  // This piece of code is not yet used. It was designed to provide an alternative
  // way of fixing a world coordinate system. In the current implementation
  // the position of the frame location at the start is kept at (0, 0, 0) by
  // adding a soft constraint with a high weight. To avoid the use of weights,
  // one can also fix the distance and/or rotation of a few planes in the world
  // coordinate system. Those plane parameters would then not be added as
  // unknowns in the equations and the frame location at the start can move
  // freely. Removing the plane parameters is equivalent to a hard constraint
  // on the wall locations/orientations and does not require tuning of a
  // constraint weight. As noted above, this has, however, not yet been
  // fully implemented. In the function below attributes are set that mark
  // the plane parameters to be fixed, but this is not exploited furtheron.
  
void SetReferencePlaneParameters(Planes &planes)
{
  Planes::iterator plane1, plane2;
  double           angle, pi=4.0*atan(1.0);
  
  // Normal and distance of first (horizontal) plane to origin
  plane1 = planes.begin();
  plane1->Attribute(PT_Reference) = PT_FullReference;
  
  // Direction of first wall and distance to origin
  plane1++;
  plane1->Attribute(PT_Reference) = PT_FullReference;
  
  // Find a second independent wall
  for (plane2=plane1+1; plane2!=planes.end(); plane2++) {
  	if (fabs(plane2->Normal().Z()) > 0.01) continue; // Not a wall
  	angle = Angle(plane1->Normal(), plane2->Normal());
  	if (angle > pi/2.0) angle = pi - angle;
  	if (angle > pi/4.0) { // At least 45 degree angle for second reference wall
  	  plane2->Attribute(PT_Reference) = PT_OrientationReference;
  	  return;
  	}
  }
  
  // We should not get here
  printf("Insufficient reference planes in SetReferencePlaneParameters\n");
  exit(0);
}
