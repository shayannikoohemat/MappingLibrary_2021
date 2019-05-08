
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

 Initial creation:
 Author : George Vosselman
 Date   : 27-11-2013

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
#include "LaserPoints.h"
#include "ObjectPoints.h"
#include "LineTopologies.h"
#include "Planes.h"
#include "BNF_io.h"
#include "BSplineFit.h"
#include "GeneralUtility.h"

/*
--------------------------------------------------------------------------------
                      The main indoorsimulation function
--------------------------------------------------------------------------------
*/

void indoorsimulation_cpp(int model, char *p_file, char *o0_file, char *o1_file,
                          char *o2_file, char *scan_dir,
						  double scanner_rotation, 
						  double opening_angle_in_degrees,
						  double angle_increment_in_degrees,
						  int num_scans_per_second, double amplitude,
						  double kappa_damping, char *par_file,
						  double noise_std,
						  char *map_points_file, char *map_topology_file,
						  char *path_points_file, char *path_topology_file,
						  bool double_wall, bool scans_in_frame_cs)
{
  void CreateModel(int, ObjectPoints &, LineTopologies &, Planes &,
				   vector<DataBounds3D> &, char *, char *);
  void CreateSmoothPath(int, double, BSplineFit &, BSplineFit &, BSplineFit &,
                        BSplineFit &, BSplineFit &, BSplineFit &, double &,
						double &, char *, char *);
  void DeriveFramePose(double, BSplineFit &, BSplineFit &, BSplineFit &,
                       BSplineFit &, BSplineFit &, BSplineFit &, Position3D &,
					   Rotation3D &, double, double, double);
				   
  ObjectPoints         corridor_corners;
  LineTopologies       corridor_faces;
  LineTopology         corridor_face;
  Planes               corridor_planes;
  Planes::iterator     corridor_plane;
  vector<DataBounds3D> face_bounds;
  vector<DataBounds3D>::iterator face_bound;
  Vector3D             sensor_offset_in_frame[3], beam;;
  Rotation3D           sensor_rotation_in_frame[3];
  LaserPoints          points, observations[3], path;
  LaserPoint           point;
  double               angle, degree = atan(1.0) / 45.0, angle0_start,
                       angle1_start, angle2_start, omega, phi, kappa,
					   wave_length;
  Position3D           reflection_pos;
  BSplineFit           omega_spline, phi_spline, kappa_spline, X_spline,
                       Y_spline, Z_spline;
  
  Rotation3D frame_rotation = Rotation3D();
  Vector3D position_increment, beam_world;
  Position3D frame_position, sensor_position;
  double time, time_increment, total_time, angle_increment,
         scalar_min, scalar_reflection, scalar_sensor, noise,
		 frame_speed, opening_angle, start_time, end_time;
  Line3D beam_line;
  int sensor, i, num_pts_per_scan, num_scans, scan;
  char scan_name[20], *point_file_name;
  FILE *par_fd;
  int noise_level[5], i_noise;

  // Conversions and deriving some variables
  angle_increment = angle_increment_in_degrees * degree;
  opening_angle = opening_angle_in_degrees * degree;
  num_pts_per_scan = (opening_angle + 0.00001) / angle_increment;
  time_increment = 1.0 / (num_scans_per_second * num_pts_per_scan);
  wave_length = 1.0;
  frame_speed = 1.0;
  printf("%d points per scan\n", num_pts_per_scan);
  printf("%.2f degree angle increment\n", angle_increment / degree);
  
  // Create the model
  CreateModel(model, corridor_corners, corridor_faces, corridor_planes,
              face_bounds, map_points_file, map_topology_file);

  // Save model data
  corridor_corners.Write("corridor.objpts");
  corridor_faces.Write("corridor.top", false);

  // Create smooth trajectory for this model
  CreateSmoothPath(model, frame_speed, omega_spline, phi_spline, kappa_spline,
                   X_spline, Y_spline, Z_spline, start_time, end_time,
			       path_points_file, path_topology_file);
  num_scans = (int) ((end_time - start_time) * num_scans_per_second);
  printf("%d scan lines\n", num_scans);
                   
  // Output smooth path
  for (time=start_time; time<end_time; time+=0.1)
  	path.push_back(LaserPoint(X_spline.Value(time), Y_spline.Value(time),
	                          Z_spline.Value(time)));
  path.Write("smooth_path.laser", 0, false);
  path.ErasePoints();
  
  // Generate a few sensor beam directions
  sensor_offset_in_frame[0] = Vector3D(0.0, 0.0, 0.5);
  sensor_offset_in_frame[1] = Vector3D(0.0, -0.25, 0.0);
  sensor_offset_in_frame[2] = Vector3D(0.0, 0.25, 0.0);
  if (double_wall) {
  	// All walls captured twice
    angle0_start =  135.0 * degree;
    angle1_start =   45.0 * degree;
    angle2_start = -135.0 * degree;
  }
  else { 
    // Ceiling and floor captured twice
    angle0_start =  135.0 * degree;
    angle1_start =  -45.0 * degree;
    angle2_start =  135.0 * degree;
  }
  
  sensor_rotation_in_frame[0] = Rotation3D(0.0, 0.0, angle0_start);
  sensor_rotation_in_frame[1] = Rotation3D(0.0, -90.0 * degree, 0.0) *
                               Rotation3D(45.0 * degree, 0.0, 0.0) *
							   Rotation3D(0.0, scanner_rotation * degree, 0.0) *
							   Rotation3D(0.0, 0.0, angle1_start);
  sensor_rotation_in_frame[2] = Rotation3D(0.0, -90.0 * degree, 0.0) *
                               Rotation3D(-45.0 * degree, 0.0, 0.0) *
							   Rotation3D(0.0, scanner_rotation * degree, 0.0) *
							   Rotation3D(0.0, 0.0, angle2_start);
  DeriveFramePose(0.0, omega_spline, phi_spline, kappa_spline,
                  X_spline, Y_spline, Z_spline, frame_position, frame_rotation,
				  wave_length, amplitude, kappa_damping);
  for (angle=0.0; angle < 300.0; angle += 5.0) {
  	beam = Rotation3D(0.0, 0.0, angle * degree) * Vector3D(0.1, 0.0, 0.0);
    for (sensor=0; sensor<3; sensor++) {
      point.vect() = frame_position + frame_rotation *
	                 (sensor_offset_in_frame[sensor] +
                      sensor_rotation_in_frame[sensor] * beam);
      point.Attribute(ScanNumberTag) = sensor;
      points.push_back(point);
      // Add sensor origin, X- and Z-axis
      if (angle == 0.0) {
      	point.vect() = frame_position + 
		               frame_rotation * sensor_offset_in_frame[sensor];
      	points.push_back(point);
      	point.vect() = frame_position + frame_rotation *
		               (sensor_offset_in_frame[sensor] +
      	               sensor_rotation_in_frame[sensor] *
      	               Vector3D(0.03, 0.0, 0.0));
      	points.push_back(point);
      	point.vect() = frame_position + frame_rotation *
		               (sensor_offset_in_frame[sensor] +
      	               sensor_rotation_in_frame[sensor] *
      	               Vector3D(0.0, 0.0, 0.03));
      	points.push_back(point);
      }
    }
  }

  for (i=0; i<5; i++) noise_level[i] = 0;
  
  if (par_file) par_fd = fopen(par_file, "w");
  // Move sensor and calculate sensor points
  for (scan=0, time=0.0; scan<num_scans; scan++) {
  	printf("%6d\r", scan);
    for (i=0, angle=0.0; i<num_pts_per_scan;
	     i++, angle+=angle_increment, time+=time_increment) {
 
	  // Determine frame pose in the world coordinate system
      DeriveFramePose(time, omega_spline, phi_spline, kappa_spline,
                      X_spline, Y_spline, Z_spline, frame_position,
					  frame_rotation, wave_length, amplitude, kappa_damping);
      // and the beam in the sensor coordinate system
   	  beam = Rotation3D(0.0, 0.0, angle) * Vector3D(1.0, 0.0, 0.0);
   	  point.DoubleAttribute(TimeTag) = time;
   	
   	  // Output of pose parameters
   	  if (par_file && i == num_pts_per_scan/2) {
   	    frame_rotation.DeriveAngles(omega, phi, kappa);
   	    fprintf(par_fd, "%4d %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f\n", scan,
		        omega * 45.0 / atan(1.0), phi * 45.0 / atan(1.0),
				kappa * 45.0 / atan(1.0), frame_position.X(),
				frame_position.Y(), frame_position.Z());
   	  }
   	  // Store the frame origin
      point.RemoveAttribute(ScanNumberTag);
      point.RemoveAttribute(ScanLineNumberTag);
      point.vect() = frame_position.vect();
      points.push_back(point);
    
      if (i == 0) {
        path.push_back(LaserPoint(frame_position.vect()));
        path.push_back(LaserPoint(frame_position.vect()+frame_rotation*Vector3D(0.005, 0.0, 0.0)));
      }
    
      // Generate all reflections
      point.Attribute(ScanLineNumberTag) = scan;
      point.FloatAttribute(AngleTag) = angle;
   	  for (sensor=0; sensor<3; sensor++) {

   	    // Construct the beam line in the world coordinate system
        sensor_position = frame_position +
	                      frame_rotation * sensor_offset_in_frame[sensor];
   	    beam_world = frame_rotation * sensor_rotation_in_frame[sensor] * beam;
   	    beam_line = Line3D(sensor_position, beam_world);
   	  
   	    // Intersect beam line with walls and select the nearest intersection
   	    // point in positive direction
   	    scalar_min = 1e10;
   	    scalar_sensor = beam_line.Scalar(sensor_position);
        point.Attribute(ScanNumberTag) = sensor;
   	    for (corridor_plane=corridor_planes.begin(), face_bound=face_bounds.begin();
		     corridor_plane!=corridor_planes.end(); 
			 corridor_plane++, face_bound++) {
	      if (IntersectLine3DPlane(beam_line, *corridor_plane, reflection_pos)) {
		    // Check if the reflection is inside the face bounds
	      	if (!face_bound->Inside(reflection_pos)) continue;
	        scalar_reflection = beam_line.Scalar(reflection_pos);
	        // Check if it is the nearest plane
	        if (scalar_reflection > scalar_sensor && 
		        scalar_reflection < scalar_min) {
	      	  point.vect() = reflection_pos.vect();
	      	  scalar_min = scalar_reflection;
	        }
	      }
	    }
	    if (noise_std > 0.0) {
	      scalar_reflection = beam_line.Scalar(point.Position3DRef());
	      noise = GenerateRandomG(noise_std);
	      scalar_reflection += noise;
	      point.vect() = beam_line.Position(scalar_reflection).vect();
	      i_noise = (int) (fabs(noise) / noise_std);
	      if (i_noise > 3) noise_level[4]++;
	      else noise_level[i_noise]++;
	    }
	    points.push_back(point);

        // Reconstruct the observation in the frame coordinate system
        point.vect() -= frame_position;
        point.vect() = frame_rotation.Transpose() * point.vect();
        observations[sensor].push_back(point);
   	  }
    }
    
    // Add optional output of scans
    if (scan_dir) {
      if (scan == 0) printf("First scan %d points\n", points.size());
      LaserPoints scan_points;

      if (scans_in_frame_cs) { // Observations in frame coordinate system
        scan_points.insert(scan_points.end(), observations[0].end()-num_pts_per_scan,
	                       observations[0].end());
        scan_points.insert(scan_points.end(), observations[1].end()-num_pts_per_scan,
	                       observations[1].end());
        scan_points.insert(scan_points.end(), observations[2].end()-num_pts_per_scan,
	                       observations[2].end());
	  }
	  else { // Observations in object coordinate system
  	    // Collect points (3 scans + 1 x num_pts_per_scan for path)
        scan_points.insert(scan_points.begin(), points.end()-4*num_pts_per_scan, points.end());
        // Add frame position
        point.vect() = frame_position.vect();
        scan_points.push_back(point);
	  }
      
      // Compose file name
      sprintf(scan_name, "scan_sim%5d", scan);
      for (int k=8; k<13; k++) if (scan_name[k] == ' ') scan_name[k] = '0';
      point_file_name = ComposeFileName(scan_dir, scan_name, ".laser");
      // Write points and delete them
      scan_points.Write(point_file_name, 0, false);
      scan_points.ErasePoints();
      free(point_file_name);
    }
  }
  printf("\n");
  if (par_file) fclose(par_fd);
  path.Write("frame_path.laser", 0, false);
  
  points.Write(p_file, 0, false);
  if (o0_file) observations[0].Write(o0_file, 0, false);
  if (o1_file) observations[1].Write(o1_file, 0, false);
  if (o2_file) observations[2].Write(o2_file, 0, false);
  
  // Output noise statistics
  if (noise_std > 0.0) {
    for (i=0, i_noise=0; i<5; i++) i_noise += noise_level[i];
    noise_level[1] += noise_level[0];
    noise_level[2] += noise_level[1];
    noise_level[3] += noise_level[2];
    printf("Below 1 sigma  %6.2f\%\n", 100.0 * (double) noise_level[0] / (double) i_noise);
    printf("Below 2 sigma  %6.2f\%\n", 100.0 * (double) noise_level[1] / (double) i_noise);
    printf("Below 3 sigma  %6.2f\%\n", 100.0 * (double) noise_level[2] / (double) i_noise);
    printf("Below 4 sigma  %6.2f\%\n", 100.0 * (double) noise_level[3] / (double) i_noise);
  }
}

void CreateModel(int index, ObjectPoints &corridor_corners,
                 LineTopologies &corridor_faces, Planes &corridor_planes,
				 vector<DataBounds3D> &face_bounds,
				 char *map_points_file, char *map_topology_file)
{
  Plane                     plane;
  PointNumberList::iterator node;
  int                       node_index, offset;
  LineTopologies::iterator  corridor_face;
  DataBounds3D              bounds;
  ObjectPoints::iterator    corner;
  ObjectPoints              map_points;
  LineTopologies            map_lines;
  LineTopologies::iterator  map_line;
  LineTopology::iterator    map_node1, map_node2;
  
  switch (index) {
    case 0: // Straight corridor
      // Set corridor model corners
      corridor_corners.push_back(ObjectPoint(Vector3D(-1.0, -2.0, -1.5), PointNumber(0), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D(19.0, -2.0, -1.5), PointNumber(1), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D(-1.0,  2.0, -1.5), PointNumber(2), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D(19.0,  2.0, -1.5), PointNumber(3), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D(-1.0, -2.0, 1.0), PointNumber(4), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D(19.0, -2.0, 1.0), PointNumber(5), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D(-1.0,  2.0, 1.0), PointNumber(6), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D(19.0,  2.0, 1.0), PointNumber(7), Covariance3D()));  

      // Set corridor model faces
      corridor_faces.push_back(LineTopology(0, 0, 4, 6, 2));  // Start wall
      corridor_faces.push_back(LineTopology(1, 0, 2, 3, 1));  // Floor
      corridor_faces.push_back(LineTopology(2, 0, 4, 5, 1));  // Left wall
      corridor_faces.push_back(LineTopology(3, 2, 6, 7, 3));  // Right wall
      corridor_faces.push_back(LineTopology(4, 4, 6, 7, 5));  // Ceiling
      corridor_faces.push_back(LineTopology(5, 1, 5, 7, 3));  // End wall
      break;
      
    case 1: // Loop corridor
    case 2:
      // Set corridor model corners
      corridor_corners.push_back(ObjectPoint(Vector3D(-10.0, -1.5, -1.5), PointNumber(0), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D( 10.0, -1.5, -1.5), PointNumber(1), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D( 10.0,  8.5, -1.5), PointNumber(2), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D(-10.0,  8.5, -1.5), PointNumber(3), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D( -7.0,  1.5, -1.5), PointNumber(4), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D(  7.0,  1.5, -1.5), PointNumber(5), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D(  7.0,  5.5, -1.5), PointNumber(6), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D( -7.0,  5.5, -1.5), PointNumber(7), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D(-10.0, -1.5,  1.0), PointNumber(10), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D( 10.0, -1.5,  1.0), PointNumber(11), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D( 10.0,  8.5,  1.0), PointNumber(12), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D(-10.0,  8.5,  1.0), PointNumber(13), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D( -7.0,  1.5,  1.0), PointNumber(14), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D(  7.0,  1.5,  1.0), PointNumber(15), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D(  7.0,  5.5,  1.0), PointNumber(16), Covariance3D()));  
      corridor_corners.push_back(ObjectPoint(Vector3D( -7.0,  5.5,  1.0), PointNumber(17), Covariance3D()));  

      // Set corridor model faces
      corridor_faces.push_back(LineTopology(0,  0,  1,  2,  3));  // Floor
      corridor_faces.push_back(LineTopology(1, 10, 11, 12, 13));  // Ceiling
      corridor_faces.push_back(LineTopology(2,  0,  1, 11, 10));  // Front outside
      corridor_faces.push_back(LineTopology(3,  1,  2, 12, 11));  // Right outside
      corridor_faces.push_back(LineTopology(4,  2,  3, 13, 12));  // Back outside
      corridor_faces.push_back(LineTopology(5,  3,  0, 10, 13));  // Left outside
      corridor_faces.push_back(LineTopology(6,  4,  5, 15, 14));  // Front inside
      corridor_faces.push_back(LineTopology(7,  5,  6, 16, 15));  // Right inside
      corridor_faces.push_back(LineTopology(8,  6,  7, 17, 16));  // Back inside
      corridor_faces.push_back(LineTopology(9,  7,  4, 14, 17));  // Left inside
      break;
      
    case 3: // User provided map. assume 3.0 m high ceiling (floor at -1.5, ceiling at 1.5 m)
	  if (!map_points.Read(map_points_file)) {
	  	printf("Error reading map points from %s\n", map_points_file);
	  	exit(0);
	  }  
	  if (!map_lines.Read(map_topology_file)) {
	  	printf("Error reading map topology from %s\n", map_topology_file);
	  	exit(0);
	  }
	  // For every map point, create a corridor corner at height -1.5 and 1.5
	  offset = map_points.HighestPointNumber().Number() + 1;
	  bounds.Initialise();
	  for (corner=map_points.begin(); corner!=map_points.end(); corner++) {
	  	corner->Z() = -1.5;
	  	bounds.Update(corner->Position3DRef());
	  	corridor_corners.push_back(*corner);
	  	corner->Z() = 1.5;
	  	corner->Number() += offset;
	  	corridor_corners.push_back(*corner);
	  }
	  // Create all corridor walls
	  for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
	  	for (map_node1=map_line->begin(), map_node2=map_node1+1;
		     map_node2!=map_line->end(); map_node1++, map_node2++)
		  corridor_faces.push_back(LineTopology(corridor_faces.size(),
		          map_node1->Number(), map_node2->Number(),
				  map_node2->Number() + offset,
				  map_node1->Number() + offset));
	  }
	  // Add big planes covering all walls for floor and ceiling
	  offset = map_points.HighestPointNumber().Number() + 1;
      corridor_faces.push_back(LineTopology(corridor_faces.size(),
	               offset, offset+1, offset+2, offset+3));
	  corridor_corners.push_back(ObjectPoint(Vector3D(bounds.Minimum().X(),
	                                                  bounds.Minimum().Y(), -1.5),
											 PointNumber(offset),
											 Covariance3D()));	  
	  corridor_corners.push_back(ObjectPoint(Vector3D(bounds.Minimum().X(),
	                                                  bounds.Maximum().Y(), -1.5),
											 PointNumber(offset+1),
											 Covariance3D()));	  
	  corridor_corners.push_back(ObjectPoint(Vector3D(bounds.Maximum().X(),
	                                                  bounds.Maximum().Y(), -1.5),
											 PointNumber(offset+2),
											 Covariance3D()));	  
	  corridor_corners.push_back(ObjectPoint(Vector3D(bounds.Maximum().X(),
	                                                  bounds.Minimum().Y(), -1.5),
											 PointNumber(offset+3),
											 Covariance3D()));	  
      offset += 4;
      corridor_faces.push_back(LineTopology(corridor_faces.size(),
	               offset, offset+1, offset+2, offset+3));
	  corridor_corners.push_back(ObjectPoint(Vector3D(bounds.Minimum().X(),
	                                                  bounds.Minimum().Y(), 1.5),
											 PointNumber(offset),
											 Covariance3D()));	  
	  corridor_corners.push_back(ObjectPoint(Vector3D(bounds.Minimum().X(),
	                                                  bounds.Maximum().Y(), 1.5),
											 PointNumber(offset+1),
											 Covariance3D()));	  
	  corridor_corners.push_back(ObjectPoint(Vector3D(bounds.Maximum().X(),
	                                                  bounds.Maximum().Y(), 1.5),
											 PointNumber(offset+2),
											 Covariance3D()));	  
	  corridor_corners.push_back(ObjectPoint(Vector3D(bounds.Maximum().X(),
	                                                  bounds.Minimum().Y(), 1.5),
											 PointNumber(offset+3),
											 Covariance3D()));	  
	  corridor_corners.Sort();
	  break;
  }
 
  // Set corridor model planes and bounding boxes
  for (corridor_face=corridor_faces.begin();
       corridor_face!=corridor_faces.end(); corridor_face++) {
    plane.Erase();
    bounds.Initialise();
    for (node_index=0, node=corridor_face->begin(); node_index<3;
	     node_index++, node++) {
	  corner = corridor_corners.OrderedPointIterator(*node);
      plane.AddPoint(corner->Position3DRef(), false);
      bounds.Update(corner->Position3DRef());
    }
    // Calculate and store plane data
    plane.Recalculate();
    corridor_planes.push_back(plane);
    // Add bounds buffer and store bounds
    bounds.SetMinimumX(bounds.Minimum().X() - 0.001);
    bounds.SetMinimumY(bounds.Minimum().Y() - 0.001);
    bounds.SetMinimumZ(bounds.Minimum().Z() - 0.001);
    bounds.SetMaximumX(bounds.Maximum().X() + 0.001);
    bounds.SetMaximumY(bounds.Maximum().Y() + 0.001);
    bounds.SetMaximumZ(bounds.Maximum().Z() + 0.001);
    face_bounds.push_back(bounds);
  }
}

void CreateSmoothPath(int index, double speed, BSplineFit &omega_spline, 
                      BSplineFit &phi_spline, BSplineFit &kappa_spline,
                      BSplineFit &X_spline, BSplineFit &Y_spline, 
				      BSplineFit &Z_spline, double &start_time,
					  double &end_time,
				      char *path_points_file, char *path_topology_file)
{
  double                   pi=4.0*atan(1.0), x, y, time, x_inc, y_inc,
                           time_inc, kappa_old, kappa, dx, dy, old_kappa,
						   path_length, dist, min_dist, total_edge_time,
						   edge_time;
  int                      i, j, num_loops, loop;
  ObjectPoints             path_points;
  ObjectPoints::iterator   point1, point2;
  LineTopologies           path_lines;
  LineTopologies::iterator path_line;
  LineTopology::iterator   path_node1, path_node2;
  Vector3D                 direction;
  
  switch (index) {
  	case 0: // Just a straight line, with X from 0 until 9 with margin 1
  	  start_time = -1.0 / speed;  end_time = 10.0 / speed;
  	  X_spline.Initialise(2, start_time, end_time, 2);
	  X_spline.SetLinear(start_time, end_time, 0.0, 18.0);
  	  Y_spline.Initialise(2, start_time, end_time, 2);
	  Y_spline.SetLinear(start_time, end_time, 0.0, 0.0);
  	  Z_spline.Initialise(2, start_time, end_time, 2);
	  Z_spline.SetLinear(start_time, end_time, 0.0, 0.0);
  	  omega_spline.Initialise(2, start_time, end_time, 2);
	  omega_spline.SetLinear(start_time, end_time, 0.0, 0.0);
  	  phi_spline.Initialise(2, start_time, end_time, 2);
	  phi_spline.SetLinear(start_time, end_time, 0.0, 0.0);
  	  kappa_spline.Initialise(2, start_time, end_time, 2);
	  kappa_spline.SetLinear(start_time, end_time, 0.0, 0.0);
	  start_time = 0.0;
	  end_time   = 9.0 / speed;
	  break;
	  
	case 1: // A smoothed rectangle in the XOY plane
	case 2:
	  if (index == 1) num_loops = 1;
	  else num_loops = 3;
	  start_time = -7.0 / speed;
	  end_time = (num_loops * (2.0 * 14.0 + 2.0 * 4.0 + 2.0 * pi * 1.5) + 7.0) / speed;
	  
	  // Construct XY trajectory
	  X_spline.Initialise(3, start_time, end_time, 2.0 / speed);
	  Y_spline.Initialise(3, start_time, end_time, 2.0 / speed);
	  x = -8.5; y = 0.0; time = start_time;;
	  for (i=0; i<num_loops*4+1; i++) {
	  	switch (i-4*(i/4)) {
	  	  case 0: x_inc =  1.0; y_inc =  0.0; break;
	  	  case 1: x_inc =  0.0; y_inc =  1.0; break;
	  	  case 2: x_inc = -1.0; y_inc =  0.0; break;
	  	  case 3: x_inc =  0.0; y_inc = -1.0; break;
	  	}
	  	x += 1.5 * x_inc;  y += 1.5 * y_inc;
	  	for (j=0; j<15; j++) {
	  	  if (2*(i/2) != i && j == 5) {
	  	  	j = 15;
	  	  	continue;
	  	  }
	  	  X_spline.AddObservation(time, x);	
	  	  Y_spline.AddObservation(time, y);
		  x += x_inc;  y += y_inc;  time += 1.0 / speed;	
	  	}
	  	time += (0.5 * pi * 1.5 - 1.0) / speed;
	  	x += 0.5 * x_inc;  y += 0.5 * y_inc;	  	
	  }
	  X_spline.FitSpline();
	  Y_spline.FitSpline();
	  
	  // Derive kappa spline
	  kappa_spline.Initialise(3, start_time, end_time, 2.0 / speed);
	  time_inc = 2.0 / (10.0 * speed);
	  for (time=start_time+time_inc; time<end_time-time_inc; time+=time_inc) {
	  	dx = X_spline.Value(time+time_inc/10.0) - X_spline.Value(time-time_inc/10.0);
	  	dy = Y_spline.Value(time+time_inc/10.0) - Y_spline.Value(time-time_inc/10.0);
	  	kappa = atan2(dy, dx);
	  	// Add/subtract 2 pi if needed
	  	if (time > start_time + time_inc) {
	  	  while (kappa - old_kappa > pi) kappa -= 2.0 * pi;
		  while (kappa - old_kappa < -pi) kappa += 2.0 * pi;	
	  	}
	  	old_kappa = kappa;
	  	kappa_spline.AddObservation(time, kappa);
	  }
      // Add an extra observation for the end time
 	  dx = X_spline.Value(end_time) - X_spline.Value(end_time-time_inc/100.0);
	  dy = Y_spline.Value(end_time) - Y_spline.Value(end_time-time_inc/100.0);
  	  kappa = atan2(dy, dx);
  	  while (kappa - old_kappa > pi) kappa -= 2.0 * pi;
	  while (kappa - old_kappa < -pi) kappa += 2.0 * pi;	
	  kappa_spline.AddObservation(end_time, kappa);
	  
	  kappa_spline.FitSpline();
	  
	  // Set other; splines to 0.0
  	  Z_spline.Initialise(2, start_time, end_time, 2);
	  Z_spline.SetLinear(start_time, end_time, 0.0, 0.0);
  	  omega_spline.Initialise(2, start_time, end_time, 2);
	  omega_spline.SetLinear(start_time, end_time, 0.0, 0.0);
  	  phi_spline.Initialise(2, start_time, end_time, 2);
	  phi_spline.SetLinear(start_time, end_time, 0.0, 0.0);
      start_time = 0.0;
	  end_time   = num_loops * (2.0 * 14.0 + 2.0 * 4.0 + 2.0 * pi * 1.5) / speed;
	  break;
	  
	case 3: // Use provided polygon - to be converted to spline
	  // Read path polygon
	  if (!path_points.Read(path_points_file)) {
	  	printf("Error reading path points from %s\n", path_points_file);
	  	exit(0);
	  }  
	  if (!path_lines.Read(path_topology_file)) {
	  	printf("Error reading path topology from %s\n", path_topology_file);
	  	exit(0);
	  }
      
      // Calculate path length
      path_line = path_lines.begin();
      min_dist = 1e10; path_length = 0.0;
      for (path_node1=path_line->begin(), path_node2=path_node1+1;
	       path_node2!=path_line->end(); path_node1++, path_node2++) {
	    dist = path_points.PointIterator(*path_node1)
		         ->Distance(path_points.PointIterator(*path_node2)->Position3DRef());
		if (dist < min_dist) min_dist = dist;
		path_length += dist;
	  }
	  start_time = 0.0;
	  end_time = path_length / speed;
	  time_inc = 0.5 * min_dist / speed;

	  // Construct XY trajectory
	  X_spline.Initialise(3, start_time-time_inc, end_time+time_inc, 4*time_inc);
	  Y_spline.Initialise(3, start_time-time_inc, end_time+time_inc, 4*time_inc);
	  time = 0.0;
      for (path_node1=path_line->begin(), path_node2=path_node1+1;
	       path_node2!=path_line->end(); path_node1++, path_node2++) {
	    point1 = path_points.PointIterator(*path_node1);
	    point2 = path_points.PointIterator(*path_node2);
	    dist = point1->Distance(point2->Position3DRef());
	    direction = (point2->vect() - point1->vect()).Normalize();
	    // Add start of path
        if (path_node1 == path_line->begin()) {
          X_spline.AddObservation(0.0, point1->X());
          Y_spline.AddObservation(0.0, point1->Y());
          X_spline.AddObservation(-time_inc, point1->X() - time_inc * direction.X() * speed);
          Y_spline.AddObservation(-time_inc, point1->Y() - time_inc * direction.Y() * speed);
        }
        // Add nodes on the way
        total_edge_time = dist / speed;
        if (total_edge_time > time_inc) {
          edge_time = 0;
          do {
          	edge_time += time_inc;
            time += time_inc;
          	X_spline.AddObservation(time, point1->X() + edge_time * direction.X() * speed);
          	Y_spline.AddObservation(time, point1->Y() + edge_time * direction.Y() * speed);
          } while (edge_time + time_inc < total_edge_time);
        }
        // Add the end node
        time += total_edge_time - edge_time;
        X_spline.AddObservation(time, point2->X());
        Y_spline.AddObservation(time, point2->Y());
	    // Add start of path
        if (path_node2 + 1 == path_line->end()) {
          time += time_inc;
          X_spline.AddObservation(time, point2->X() + time_inc * direction.X() * speed);
          Y_spline.AddObservation(time, point2->Y() + time_inc * direction.Y() * speed);
        }
      }
      X_spline.FitSpline();
      Y_spline.FitSpline();
      
	  // Derive kappa spline
	  kappa_spline.Initialise(3, start_time-time_inc, end_time+time_inc, 4*time_inc);
	  time_inc = 2.0 / (10.0 * speed);
	  for (time=start_time-0.5*time_inc; time<end_time; time+=time_inc) {
	  	dx = X_spline.Value(time+time_inc/10.0) - X_spline.Value(time-time_inc/10.0);
	  	dy = Y_spline.Value(time+time_inc/10.0) - Y_spline.Value(time-time_inc/10.0);
	  	kappa = atan2(dy, dx);
	  	// Add/subtract 2 pi if needed
	  	if (time > start_time -0.5 * time_inc) {
	  	  while (kappa - old_kappa > pi) kappa -= 2.0 * pi;
		  while (kappa - old_kappa < -pi) kappa += 2.0 * pi;	
	  	}
	  	old_kappa = kappa;
	  	kappa_spline.AddObservation(time, kappa);
	  }
      // Add an extra observation for the end time
 	  dx = X_spline.Value(end_time+time_inc) - X_spline.Value(end_time+time_inc*0.99);
	  dy = Y_spline.Value(end_time+time_inc) - Y_spline.Value(end_time+time_inc*0.99);
  	  kappa = atan2(dy, dx);
  	  while (kappa - old_kappa > pi) kappa -= 2.0 * pi;
	  while (kappa - old_kappa < -pi) kappa += 2.0 * pi;	
	  kappa_spline.AddObservation(end_time+time_inc, kappa);	  
	  kappa_spline.FitSpline();
      
	  // Set other; splines to 0.0
  	  Z_spline.Initialise(2, start_time, end_time, 2);
	  Z_spline.SetLinear(start_time, end_time, 0.0, 0.0);
  	  omega_spline.Initialise(2, start_time, end_time, 2);
	  omega_spline.SetLinear(start_time, end_time, 0.0, 0.0);
  	  phi_spline.Initialise(2, start_time, end_time, 2);
	  phi_spline.SetLinear(start_time, end_time, 0.0, 0.0);
	  break;
  }
}

void DeriveFramePose(double time, BSplineFit &omega_spline, 
                     BSplineFit &phi_spline, BSplineFit &kappa_spline,
                     BSplineFit &X_spline, BSplineFit &Y_spline, 
				     BSplineFit &Z_spline, Position3D &pos, Rotation3D &rot,
					 double wave_length, double amplitude, double kappa_damping)
{
  double dx, dy, kappa, dx_local, dy_local, dkappa_local, pi=4.0*atan(1.0);
  
  // Determine local line direction
  kappa = kappa_spline.Value(time);
  
  // Determine local curve displacement
  dx_local = 0.0;
  dy_local = amplitude * sin(time * 2 * pi / wave_length);
  
  // Rotate this displacement
  dx = cos(kappa) * dx_local - sin(kappa) * dy_local;
  dy = sin(kappa) * dx_local + cos(kappa) * dy_local;
  
  // Determine frame position
  pos.X() = X_spline.Value(time) + dx;
  pos.Y() = Y_spline.Value(time) + dy;
  pos.Z() = Z_spline.Value(time);
  
  // Determine local direction
  dkappa_local = kappa_damping *
                 atan(2 * pi * amplitude * cos(2.0 * pi * time / wave_length));
  kappa += dkappa_local;
  rot = Rotation3D(omega_spline.Value(time), phi_spline.Value(time), kappa);
}


