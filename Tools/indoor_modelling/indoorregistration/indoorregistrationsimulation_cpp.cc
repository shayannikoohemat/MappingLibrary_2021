
/*
                      Copyright 2015 University of Twente 
 
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
 Date   : 21-12-2015

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
#include "GeneralUtility.h"

/*
--------------------------------------------------------------------------------
                      The main indoorregistration function
--------------------------------------------------------------------------------
*/

void indoorregistrationsimulation_cpp(char *p_file, char *o0_file, char *o1_file,
                                      char *o2_file, char *scan_dir, 
                                      bool scans_in_frame_cs,
							          double scanner_rotation,
							          double opening_angle_in_degrees,
                                      double angle_increment_in_degrees,
                                      double noise_std,
									  int num_planes,
									  char *pl_file)
{
  void CreateModel(int, ObjectPoints &, LineTopologies &, Planes &,
				   vector<DataBounds3D> &);
				   
  ObjectPoints         corridor_corners;
  LineTopologies       corridor_faces;
  LineTopologies::iterator corridor_face, reflecting_face;
  Planes               corridor_planes;
  Planes::iterator     corridor_plane;
  vector<DataBounds3D> face_bounds;
  vector<DataBounds3D>::iterator face_bound;
  Vector3D             sensor_offset_in_frame[3], beam;
  Rotation3D           sensor_rotation_in_frame[3];
  LaserPoints          points, observations[3];
  LaserPoint           point;
  double               angle, degree = atan(1.0) / 45.0, angle0_start,
                       angle1_start, angle2_start, omega, phi, kappa,
					   wave_length;
  Position3D           reflection_pos;
  bool                 double_wall=true, beam_reflects;
  
  Rotation3D frame_rotation;
  Vector3D position_increment, beam_world;
  Position3D frame_position, sensor_position;
  double angle_increment,
         scalar_min, scalar_reflection, scalar_sensor, noise,
		 opening_angle;
  Line3D beam_line;
  int sensor, i, num_pts_per_scan, num_scans, scan;
  char scan_name[20], *point_file_name;
  int noise_level[5], i_noise;

  // Conversions and deriving some variables
  angle_increment = angle_increment_in_degrees * degree;
  opening_angle = opening_angle_in_degrees * degree;
  num_pts_per_scan = (opening_angle + 0.00001) / angle_increment;
  wave_length = 1.0;
  printf("%d points per scan\n", num_pts_per_scan);
  printf("%.2f degree angle increment\n", angle_increment / degree);
  
  // Create the model
  CreateModel(num_planes, corridor_corners, corridor_faces, corridor_planes,
              face_bounds);

  // Save model data
  corridor_corners.Write("corridor.objpts");
  corridor_faces.Write("corridor.top", false);
 
  // Default sensor in frame poses
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

  // Generate a few sensor beam directions for visualisation
  for (angle=0.0; angle < 270.0; angle += 5.0) {
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
  
  // Move sensor and calculate sensor points
  for (scan=0; scan<6; scan++) {
    switch (scan) {
      case 0: // Default frame poses
        frame_position = Vector3D(0.0, 0.0, 0.0);
        frame_rotation = Rotation3D(-40.0 * degree, -30.0 * degree, 0.0);
        break;

      case 1: // Kappa rotation
        frame_position = Vector3D(0.0, 0.0, 0.0);
        frame_rotation = Rotation3D(-40.0 * degree, 30.0 * degree, 10.0 * degree);
        break;

      case 2: // Phi rotation
        frame_position = Vector3D(0.0, 0.0, 0.0);
        frame_rotation = Rotation3D(0.0, 10.0 * degree, 0.0);
        break;

      case 3: // case 0 with 0.5 m shift in X
        frame_position = Vector3D(0.5, 0.0, 0.0);
        frame_rotation = Rotation3D(0.0, 0.0, 0.0);
        break;

      case 4: // case 1 with 0.5 m shift in X
        frame_position = Vector3D(0.5, 0.0, 0.0);
        frame_rotation = Rotation3D(0.0, 0.0, 10.0 * degree);
        break;

      case 5: // case 2 with 0.5 m shift in X
        frame_position = Vector3D(0.5, 0.0, 0.0);
        frame_rotation = Rotation3D(0.0, 10.0 * degree, 0.0);
        break;
    }
    for (i=0, angle=0.0; i<num_pts_per_scan; i++, angle+=angle_increment) {
   	  beam = Rotation3D(0.0, 0.0, angle) * Vector3D(1.0, 0.0, 0.0);
   	
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
        beam_reflects = false;
   	    for (corridor_plane=corridor_planes.begin(), corridor_face=corridor_faces.begin(),
		     face_bound=face_bounds.begin(); corridor_plane!=corridor_planes.end(); 
			 corridor_plane++, corridor_face++, face_bound++) {
	      if (IntersectLine3DPlane(beam_line, *corridor_plane, reflection_pos)) {
		    // Check if the reflection is inside the face bounds
	      	if (!face_bound->Inside(reflection_pos)) continue;
	        scalar_reflection = beam_line.Scalar(reflection_pos);
	        // Check if it is the nearest plane
	        if (scalar_reflection > scalar_sensor && 
		        scalar_reflection < scalar_min) {
	      	  point.vect() = reflection_pos.vect();
	      	  scalar_min = scalar_reflection;
	      	  beam_reflects = true;
	      	  reflecting_face = corridor_face;
	        }
	      }
	    }
	    if (beam_reflects) {
	      // Add noise
		  if (noise_std > 0.0) {
	        scalar_reflection = beam_line.Scalar(point.Position3DRef());
	        noise = GenerateRandomG(noise_std);
	        scalar_reflection += noise;
	        point.vect() = beam_line.Position(scalar_reflection).vect();
	        i_noise = (int) (fabs(noise) / noise_std);
	        if (i_noise > 3) noise_level[4]++;
	        else noise_level[i_noise]++;
	      }
	      point.Attribute(PlaneNumberTag) = reflecting_face->Number();
	      
	      // Store point of object coordinate system
	      points.push_back(point);

          // Reconstruct the observation in the frame coordinate system
          point.vect() -= frame_position;
          point.vect() = frame_rotation.Transpose() * point.vect();
          
          // Reconstruct the observation in the sensor coordinate system
          point.vect() -= sensor_offset_in_frame[sensor];
          point.vect() = sensor_rotation_in_frame[sensor].Transpose() * point.vect();
          
          observations[sensor].push_back(point);
          
          // Alternative (scalar_reflection - scalar_sensor) * beam
          
          // Debug
          
          if (fabs(point.Z()) > 0.01) {
          	printf("Something wrong scan %d sensor %d angle %.0f X %7.3f Y %7.3f Z %7.3f\n",
			       scan, sensor, angle / degree, point.X(), point.Y(), point.Z());
          }
          
        }
   	  }
    }
  }
  
  points.Write(p_file, 0, false);
  if (o0_file) observations[0].Write(o0_file, 0, false);
  if (o1_file) observations[1].Write(o1_file, 0, false);
  if (o2_file) observations[2].Write(o2_file, 0, false);
  if (pl_file) corridor_planes.Write(pl_file);
  
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

void CreateModel(int num_planes, ObjectPoints &corridor_corners,
                 LineTopologies &corridor_faces, Planes &corridor_planes,
				 vector<DataBounds3D> &face_bounds)
{
  Plane                     plane;
  PointNumberList::iterator node;
  int                       node_index, offset;
  LineTopologies::iterator  corridor_face;
  DataBounds3D              bounds;
  ObjectPoints::iterator    corner;

  // Set corridor model corners
  corridor_corners.push_back(ObjectPoint(Vector3D(-1.5, -1.5, -1.5), PointNumber(0), Covariance3D()));  
  corridor_corners.push_back(ObjectPoint(Vector3D(11.5, -1.5, -1.5), PointNumber(1), Covariance3D()));  
  corridor_corners.push_back(ObjectPoint(Vector3D(-1.5,  1.5, -1.5), PointNumber(2), Covariance3D()));  
  corridor_corners.push_back(ObjectPoint(Vector3D(11.5,  1.5, -1.5), PointNumber(3), Covariance3D()));  
  corridor_corners.push_back(ObjectPoint(Vector3D(-1.5, -1.5,  1.5), PointNumber(4), Covariance3D()));  
  corridor_corners.push_back(ObjectPoint(Vector3D(11.5, -1.5,  1.5), PointNumber(5), Covariance3D()));  
  corridor_corners.push_back(ObjectPoint(Vector3D(-1.5,  1.5,  1.5), PointNumber(6), Covariance3D()));  
  corridor_corners.push_back(ObjectPoint(Vector3D(11.5,  1.5,  1.5), PointNumber(7), Covariance3D()));  

  // Set corridor model faces
  switch (num_planes) {
  	case 6: corridor_faces.push_back(LineTopology(5, 1, 5, 7, 3));  // End wall
	case 5: corridor_faces.push_back(LineTopology(4, 2, 6, 7, 3));  // Right wall
	case 4: corridor_faces.push_back(LineTopology(3, 0, 2, 3, 1));  // Floor
	case 3: corridor_faces.push_back(LineTopology(2, 4, 6, 7, 5));  // Ceiling
	case 2: corridor_faces.push_back(LineTopology(1, 0, 4, 5, 1));  // Left wall
	case 1: corridor_faces.push_back(LineTopology(0, 0, 4, 6, 2));  // Start wall
	        break;
	    
	default: printf("Error: number of planes should be in 1-6 range.\n");
	         exit(0);
 }
 corridor_faces.Sort();
      
/*
  // Just a single wall at 1 m distance
  corridor_corners.push_back(ObjectPoint(Vector3D(-1.0, -3.0, -1.5), PointNumber(0), Covariance3D()));  
  corridor_corners.push_back(ObjectPoint(Vector3D(-1.0, -3.0,  1.0), PointNumber(1), Covariance3D()));  
  corridor_corners.push_back(ObjectPoint(Vector3D(-1.0,  3.0,  1.0), PointNumber(2), Covariance3D()));  
  corridor_corners.push_back(ObjectPoint(Vector3D(-1.0,  3.0, -1.5), PointNumber(3), Covariance3D()));  

  // Set corridor model face
  corridor_faces.push_back(LineTopology(0, 0, 3, 2, 1));
*/

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
    plane.Number() = corridor_face->Number();
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
