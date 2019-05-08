
/*
                       Copyright 2018 University of Twente 
 
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
 Date   : 15-06-2018

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
#include "LaserBlock.h"
#include "LaserScanningSystem.h"
#include "LineSegments3D.h"
#include "normal_equations.h"
#include "BNF_io.h"

Vector3D   W_X     = Vector3D(1.0, 0.0, 0.0);
Vector3D   W_Y     = Vector3D(0.0, 1.0, 0.0);
Vector3D   W_Z     = Vector3D(0.0, 0.0, 1.0);

/*
--------------------------------------------------------------------------------
                 Three types of observation equations:
                 	- Coplanar lines
                 	- Coplanar direction vectors
                 	- Perpendicular planes
--------------------------------------------------------------------------------
*/


void CoplanarLines(int interval, int sensor1, int sensor2, 
			       Line3D &line1, Line3D &line2,
		           double *ata, double *aty, double *x_approx, int num_par,
				   bool verbose, int &num_obs, double &sqsum_obs,
				   bool update_normal_equations, double &y)
{
  double     a[12];
  Rotation3D rot1, rot2, Z_omega, Z_phi, Z_kappa;
  Vector3D   trans1, trans2, rot1l1, rot2l2, m, p1p2, l1, l2, p1, p2;
  int        offset, i, j;
   
  // Derive approximate rotation matrices and translations
  if (sensor1 == 0) {
  	if (sensor2 == 1) offset = 0; else offset = 6;
  	rot1 = Rotation3D();
  	trans1 = Vector3D(0.0, 0.0, 0.0);
  	rot2 = Rotation3D(x_approx[offset], x_approx[offset+1], x_approx[offset+2]);
  	trans2 = Vector3D(x_approx[offset+3], x_approx[offset+4], x_approx[offset+5]);
  }
  else {
  	offset = 6;
  	rot1 = Rotation3D(x_approx[0], x_approx[1], x_approx[2]);
  	trans1 = Vector3D(x_approx[3], x_approx[4], x_approx[5]);
  	rot2 = Rotation3D(x_approx[6], x_approx[7], x_approx[8]);
  	trans2 = Vector3D(x_approx[9], x_approx[10], x_approx[11]);
  }
  
  // Some frequent terms
  l1     = line1.Direction();
  l2     = line2.Direction();
  p1     = line1.CentreOfGravity();
  p2     = line2.CentreOfGravity();
  rot1l1 = rot1 * l1;
  rot2l2 = rot2 * l2;
  m      = rot1l1.VectorProduct(rot2l2);
  p1p2   = rot1 * p1 + trans1 - rot2 * p2 - trans2;

  // Observation
  y = -1.0 * m.DotProduct(p1p2);
  num_obs++;
  sqsum_obs += y * y;
  if (verbose) printf("CL: interval %d, sensors %d %d, misclosure %9.5f\n",
                      interval, sensor1, sensor2, y);
  if (!update_normal_equations) return;
  
  // Initialise partial derivatives
  for (i=0; i<12; i++) a[i] = 0.0;
  
  // Partial derivatives
  if (sensor1 == 1) {
    Z_omega = rot1.PartialDeriv(0);
    Z_phi   = rot1.PartialDeriv(1);
    Z_kappa = rot1.PartialDeriv(2);
      
	a[0] = ((Z_omega * l1).VectorProduct(rot2l2)).DotProduct(p1p2) +
           m.DotProduct(Z_omega * p1);
    a[1] = ((Z_phi   * l1).VectorProduct(rot2l2)).DotProduct(p1p2) +
           m.DotProduct(Z_phi * p1);
    a[2] = ((Z_kappa * l1).VectorProduct(rot2l2)).DotProduct(p1p2) +
           m.DotProduct(Z_kappa * p1);
    a[3] = m.DotProduct(W_X);
    a[4] = m.DotProduct(W_Y);
    a[5] = m.DotProduct(W_Z);
  }

  Z_omega = rot2.PartialDeriv(0);
  Z_phi   = rot2.PartialDeriv(1);
  Z_kappa = rot2.PartialDeriv(2);

  a[offset]   = (rot1l1.VectorProduct(Z_omega * l2)).DotProduct(p1p2) - m.DotProduct(Z_omega * p2);
  a[offset+1] = (rot1l1.VectorProduct(Z_phi   * l2)).DotProduct(p1p2) - m.DotProduct(Z_phi   * p2);
  a[offset+2] = (rot1l1.VectorProduct(Z_kappa * l2)).DotProduct(p1p2) - m.DotProduct(Z_kappa * p2);
  a[offset+3] = -1.0 * (m.DotProduct(W_X));
  a[offset+4] = -1.0 * (m.DotProduct(W_Y));
  a[offset+5] = -1.0 * (m.DotProduct(W_Z));

  // Update equation system
  Update_Normal_Eq(a, y, 1.0, ata, aty, num_par);
}



void CoplanarVectors(int interval, Line3D &line0, Line3D &line1, Line3D &line2,
		             double *ata, double *aty, double *x_approx, bool verbose,
					 int &num_obs, double &sqsum_obs,
					 bool update_normal_equations)
{
  Rotation3D rot1, rot2;
  double     y, a[12];
  Vector3D   l0, l1, l2, rot1l1, rot2l2;
  int        i;
 
  rot1 = Rotation3D(x_approx[0], x_approx[1], x_approx[2]);
  rot2 = Rotation3D(x_approx[6], x_approx[7], x_approx[8]);
  
  // Some frequent terms
  l0     = line0.Direction();
  l1     = line1.Direction();
  l2     = line2.Direction();
  rot1l1 = rot1 * l1;
  rot2l2 = rot2 * l2;
  
  // Observation	
  y = -1.0 * (l0.VectorProduct(rot1l1)).DotProduct(rot2l2);
  num_obs++;
  sqsum_obs += y * y;
  if (verbose) printf("CV: interval %d, sensors 0 1 2, misclosure %9.5f\n",
                      interval, y);
  if (!update_normal_equations) return;
  
  // Initialise partial derivatives
  for (i=0; i<12; i++) a[i] = 0.0;
  
  // Partial derivatives
  a[0] = (l0.VectorProduct(rot1.PartialDeriv(0) * l1)).DotProduct(rot2l2);
  a[1] = (l0.VectorProduct(rot1.PartialDeriv(1) * l1)).DotProduct(rot2l2);
  a[2] = (l0.VectorProduct(rot1.PartialDeriv(2) * l1)).DotProduct(rot2l2);
  a[6] = (l0.VectorProduct(rot1l1)).DotProduct(rot2.PartialDeriv(0) * l2);
  a[7] = (l0.VectorProduct(rot1l1)).DotProduct(rot2.PartialDeriv(1) * l2);
  a[8] = (l0.VectorProduct(rot1l1)).DotProduct(rot2.PartialDeriv(2) * l2);
  
  // Update equation system
  Update_Normal_Eq(a, y, 1.0, ata, aty, 12);  
}

void PerpendicularPlanes(int interval, int sensor1a, int sensor2a, 
                         int sensor1b, int sensor2b, 
			             Line3D &line1a, Line3D &line2a, 
						 Line3D &line1b, Line3D &line2b,
						 double *ata, double *aty, double *x_approx, 
						 int num_par, bool verbose, 
						 int &num_obs, double &sqsum_obs,
						 bool update_normal_equations)
{
  Vector3D   ma, mb, direction[4], rotl[4];
  Rotation3D rot[3], W, Z_omega, Z_phi, Z_kappa;
  double     y, a[12];
  int        offset, i, sensor[4];
  bool       debug=false;

  // Store sensor indices and line directions in vectors
  sensor[0] = sensor1a; direction[0] = line1a.Direction();
  sensor[1] = sensor2a; direction[1] = line2a.Direction();
  sensor[2] = sensor1b; direction[2] = line1b.Direction();
  sensor[3] = sensor2b; direction[3] = line2b.Direction();
  
  // Derive approximate rotation matrices
  rot[0] = Rotation3D(0.0, 0.0, 0.0);
  rot[1] = Rotation3D(x_approx[0], x_approx[1], x_approx[2]);
  rot[2] = Rotation3D(x_approx[6], x_approx[7], x_approx[8]);
  
  // Some common terms
  for (i=0; i<4; i++) rotl[i] = rot[sensor[i]] * direction[i];
  ma = rotl[0].VectorProduct(rotl[1]); // Unscaled normal vector of plane a
  mb = rotl[2].VectorProduct(rotl[3]); // Unscaled normal vector of plane b

  // Check planes normals and angles
  if (debug) {
  	printf("PP: rotated directions\n");
  	printf("PP: line 1a %6.2f %6.2f %6.2f\n", rotl[0].X(), rotl[0].Y(), rotl[0].Z());
  	printf("PP: line 2a %6.2f %6.2f %6.2f\n", rotl[1].X(), rotl[1].Y(), rotl[1].Z());
  	printf("PP: line 1b %6.2f %6.2f %6.2f\n", rotl[2].X(), rotl[2].Y(), rotl[2].Z());
  	printf("PP: line 2b %6.2f %6.2f %6.2f\n", rotl[3].X(), rotl[3].Y(), rotl[3].Z());
    printf("PP: normal A (%.2f %.2f %.2f), normal B (%.2f %.2f %.2f), angle %.2f\n",
           ma.Normalize().X(), ma.Normalize().Y(), ma.Normalize().Z(),
		   mb.Normalize().X(), mb.Normalize().Y(), mb.Normalize().Z(),
           Angle(ma, mb) * 45.0 / atan(1.0));
  }

  // Observation
  y = -1.0 * ma.DotProduct(mb);
  num_obs++;
  sqsum_obs += y * y;
  if (verbose)
    printf("PP: interval %d, plane A (sensors %d %d), plane B (sensors %d %d), misclosure %9.5f\n",
	       interval, sensor1a, sensor2a, sensor1b, sensor2b, y);
  if (!update_normal_equations) return;
  
  // Initialise partial derivatives
  for (i=0; i<12; i++) a[i] = 0.0;

  // Partial derivatives
  for (i=0; i<4; i++) {
  	if (sensor[i] == 0) continue; // No changes to sensor 0
    Z_omega = rot[sensor[i]].PartialDeriv(0);
    Z_phi   = rot[sensor[i]].PartialDeriv(1);
    Z_kappa = rot[sensor[i]].PartialDeriv(2);
  	if (sensor[i] == 1) offset = 0; else offset = 6;
  	switch (i) {
  	  case 0:
		a[offset]   += ((Z_omega * direction[0]).VectorProduct(rotl[1])).DotProduct(mb);
  	    a[offset+1] += ((Z_phi   * direction[0]).VectorProduct(rotl[1])).DotProduct(mb);
  	    a[offset+2] += ((Z_kappa * direction[0]).VectorProduct(rotl[1])).DotProduct(mb);
  	    break;
  	  case 1:
  	  	a[offset]   += (rotl[0].VectorProduct(Z_omega * direction[1])).DotProduct(mb);
  	  	a[offset+1] += (rotl[0].VectorProduct(Z_phi   * direction[1])).DotProduct(mb);
  	  	a[offset+2] += (rotl[0].VectorProduct(Z_kappa * direction[1])).DotProduct(mb);
  	  	break;
  	  case 2:
  	  	a[offset]   += ma.DotProduct((Z_omega * direction[2]).VectorProduct(rotl[3]));
  	  	a[offset+1] += ma.DotProduct((Z_phi   * direction[2]).VectorProduct(rotl[3]));
  	  	a[offset+2] += ma.DotProduct((Z_kappa * direction[2]).VectorProduct(rotl[3]));
  	  	break;
  	  case 3:
  	  	a[offset]   += ma.DotProduct(rotl[2].VectorProduct(Z_omega * direction[3]));
  	  	a[offset+1] += ma.DotProduct(rotl[2].VectorProduct(Z_phi   * direction[3]));
  	  	a[offset+2] += ma.DotProduct(rotl[2].VectorProduct(Z_kappa * direction[3]));
  	  	break;
  	}
  }

  // Update equation system
  Update_Normal_Eq(a, y, 1.0, ata, aty, num_par);  
}

/*
--------------------------------------------------------------------------------
                      The main indoorregistration2 function
--------------------------------------------------------------------------------
*/

void indoorregistration2_cpp(char *block_file, char *approx_sysconf_file, 
				             char *output_sysconf_file, double time_interval,
							 double mininum_segment_length,
							 int minimum_segment_size,
							 double maximum_distance_to_plane,
							 double maximum_distance_between_coplanar_segments,
							 double minimum_angle_between_coplanar_segments,
							 double maximum_angle_deviation_perpendicularity,
							 bool verbose, int max_iter)
{
  LaserBlock             block;
  LaserPoints::iterator  point_iterator[3];
  LaserPoints            scanlinedata, segment_points;
  int                    sensor, dataset_index[3], num_datasets[3], interval, i,
<<<<<<< .mine
                         j, num_par, index, num_iter, max_iter=20, num_intervals,
=======
                         j, num_par, index, num_iter, num_intervals,
>>>>>>> .r1537
						 sensor2, num_obs, num_lines, num_coplanar_vectors,
						 num_bins, *residual_histogram, bin,
						 num_perpendicular_planes, num_coplanar_lines;
  double                 end_time, scalar_start, scalar_end, scalar,
                         *x_approx, *ata, *aty, omega, phi, kappa, sqsum_obs,
						 sqsum_coplanar_lines, sqsum_coplanar_vectors,
						 sqsum_perpendicular_planes, 
						 cond, degree=atan(1.0)/45.0, residual, angle;
  bool                   done_with_interval, done_with_all, converged,
                         use_perpendicularity;
  LaserScanningSystem    system;
  Rotation3D             rotation;
  Vector3D               translation;
  Position3D             start_pos, end_pos;
  vector<int>            segment_numbers, segment_sizes;
  Line3D                 line;
  LineSegment3D          line_segment, line_segment2;
  LineSegments3D::iterator segment, segment2, segment3;
  vector<LineSegments3D::iterator> plane_segments1, plane_segments2;
  vector<LineSegments3D> line_segments_sensor[3];
  vector<LineSegments3D>::iterator current_line_segments;
  Vector3D               sensor_offset_in_frame[3], sensor_offset_in_sensor0[3];
  Rotation3D             sensor_rotation_in_frame[3], sensor_rotation_in_sensor0[3];
  Plane                  plane;
  Planes                 planes;
  Planes::iterator       plane1, plane2;

  // Open the block and read all meta data
  if (!block.ReadMetaData(block_file, true, true)) {
  	printf("Error reading block meta data from %s\n", block_file);
  	exit(0);
  }
  if (block.size() != 3) {
  	printf("Error: This block contains %d strips, 3 were expected\n", block.size());
  	exit(0);
  }
  
  // Read system configuration file
  if (!system.Read(approx_sysconf_file)) {
  	printf("Error reading scanner system configuration file %s\n", 
	       approx_sysconf_file);
  	exit(0);
  }

  // Read the first data of all three scanners
  for (sensor=0; sensor<3; sensor++) {
  	if (!block[sensor][0].Read()) {
  	  printf("Error reading first dataset of sensor %d from %s\n", sensor,
		      block[sensor][0].PointFile());
	  exit(0);
  	}
  	point_iterator[sensor] = block[sensor][0].begin();
  	dataset_index[sensor] = 0;
  	num_datasets[sensor] = block[sensor].size();
  }

  // Collect longer line segments from all time intervals and sensors
  printf("Collecting line segments...\n");
  num_lines = 0;
  end_time = 0.0;
  done_with_all = false;
  interval = 0;
  while (!done_with_all) {
  	end_time += time_interval;
  	
  	for (sensor=0; sensor<3; sensor++) {
  	  line_segments_sensor[sensor].push_back(LineSegments3D());
  	  current_line_segments = line_segments_sensor[sensor].end() - 1;
  	  scanlinedata.ErasePoints();
  	  done_with_interval = false;

  	  while (!done_with_interval) {
  	    // Check if there is further data
  	    if (point_iterator[sensor] != block[sensor][num_datasets[sensor]-1].end()) {
  	      // Check if a next data set should be read
  	  	  if (point_iterator[sensor] == block[sensor][dataset_index[sensor]].end()) {
  	  	    block[sensor][dataset_index[sensor]].ErasePoints();
  	  	    dataset_index[sensor]++;
  	  	    if (!block[sensor][dataset_index[sensor]].Read()) {
  	  	  	  printf("Error reading data set %d of sensor %d\n",
				     dataset_index[sensor], sensor);
			  exit(0);
  	  	    }
  	  	    point_iterator[sensor] = block[sensor][dataset_index[sensor]].begin();
  	  	  }
  	  	  // Check if the point is on the current interval
  	  	  if (point_iterator[sensor]->DoubleAttribute(TimeTag) < end_time) {
  	  	    scanlinedata.push_back(*(point_iterator[sensor]));
  	  	    point_iterator[sensor]++;
  	  	  }
  	  	  else done_with_interval = true;
  	  	}
  	  	else done_with_interval = true;
  	  }
  	  
  	  // Select the longest segments on this time interval
  	  scanlinedata.AttributeValueCounts(SegmentNumberTag, segment_numbers,
	                                    segment_sizes);
	  for (i=0; i<segment_numbers.size(); i++) {
	  	// Check the mininum number of points
	  	if (segment_sizes[i] < minimum_segment_size) continue;

	  	// Fit a line
	  	segment_points.ErasePoints();
  	    segment_points.AddTaggedPoints(scanlinedata, segment_numbers[i],
                                       SegmentNumberTag);
		line = segment_points.FitLine(segment_points.begin(), segment_points.end()-1);
		scalar_start = line.Scalar(segment_points.begin()->Position3DRef());
		scalar_end = line.Scalar((segment_points.end()-1)->Position3DRef());
		if (scalar_start > scalar_end) {
		  scalar = scalar_start; scalar_start = scalar_end; scalar_end = scalar;
		}
		line_segment = LineSegment3D(line, scalar_start, scalar_end,
		                             0, sensor);

		// Check the line length
		if (line_segment.Length() < mininum_segment_length) continue;

		// Store the line segment
		current_line_segments->push_back(line_segment);
	  }
  	}
  	
  	if (scanlinedata.empty()) {
  	  done_with_all = true;
  	  continue;
  	}

    interval++;
    num_lines += (line_segments_sensor[0].end()-1)->size() +
                 (line_segments_sensor[1].end()-1)->size() +
	             (line_segments_sensor[2].end()-1)->size();
    if (interval == (interval/100)*100) {
      printf("%6d %3d %3d %3d\r", interval,
	         (line_segments_sensor[0].end()-1)->size(),
	         (line_segments_sensor[1].end()-1)->size(),
	         (line_segments_sensor[2].end()-1)->size());
      fflush(stdout);
    }
  }
  printf("\nFound %d line segments in %d intervals\n",
         num_lines, interval);
  
  // Allocate equation system
  num_par = 12;
  x_approx = (double *) malloc(num_par * sizeof(double));
  ata      = (double *) malloc(num_par * num_par * sizeof(double));
  aty      = (double *) malloc(num_par * sizeof(double));
  
  // Initialise histogram of distances between coplanar segments
  num_bins = 2 * (int) (maximum_distance_to_plane / 0.01 + 0.00001);
  residual_histogram = (int *) malloc(num_bins * sizeof(int));
                                 
  // Retrieve approximate orientations in frame coordinate system
  for (sensor=0; sensor<3; sensor++) { 
    sensor_offset_in_frame[sensor] = system[sensor].Orientation()->vect();
    sensor_rotation_in_frame[sensor] = system[sensor].Orientation()->rotation();
  }
  
  // Approximate orientations w.r.t. sensor coordinate system
  sensor_rotation_in_sensor0[0] = Rotation3D(0.0, 0.0, 0.0);
  sensor_offset_in_sensor0[0] = Vector3D(0.0, 0.0, 0.0);
  for (sensor=1; sensor<3; sensor++) {
    sensor_rotation_in_sensor0[sensor] = sensor_rotation_in_frame[0].Transpose() * 
                                         sensor_rotation_in_frame[sensor];
    sensor_offset_in_sensor0[sensor] = sensor_rotation_in_frame[0].Transpose() * 
                (sensor_offset_in_frame[sensor] - sensor_offset_in_frame[0]);
  }

  // Set initial approximate values of relative poses
  for (sensor=1; sensor<3; sensor++) {
    sensor_rotation_in_sensor0[sensor].DeriveAngles(omega, phi, kappa);
  	index = (sensor - 1) * 6;
    x_approx[index]   = omega;
    x_approx[index+1] = phi;
    x_approx[index+2] = kappa;
    x_approx[index+3] = sensor_offset_in_sensor0[sensor].X();
    x_approx[index+4] = sensor_offset_in_sensor0[sensor].Y();
    x_approx[index+5] = sensor_offset_in_sensor0[sensor].Z();
  }

  // Iterative least squares estimation
  converged = false; num_iter = 0;
  num_intervals = line_segments_sensor[0].size();
  use_perpendicularity = (maximum_angle_deviation_perpendicularity > 0.0);
  while (!converged && num_iter < max_iter) {
    num_iter++;
  	
    // Initialise equation system and histogram
    memset((void *) ata, 0, num_par * num_par * sizeof(double));
    memset((void *) aty, 0, num_par * sizeof(double));
    num_obs = num_coplanar_lines = num_coplanar_vectors =
	          num_perpendicular_planes = 0;
    sqsum_obs = sqsum_coplanar_lines = sqsum_coplanar_vectors = 
	            sqsum_perpendicular_planes = 0;
    memset((void *) residual_histogram, 0, num_bins * sizeof(int));
  
    // Loop over all intervals
    for (interval=0; interval<num_intervals; interval++) {
//    for (interval=0; interval<num_intervals; interval+=100) {
    	
      // Clear old planes and line segments
      planes.Erase();
      plane_segments1.erase(plane_segments1.begin(), plane_segments1.end());
      plane_segments2.erase(plane_segments2.begin(), plane_segments2.end());
    	
      for (sensor=0; sensor<2; sensor++) {
      	for (segment=line_segments_sensor[sensor][interval].begin();
			 segment!=line_segments_sensor[sensor][interval].end();
			 segment++) {
			 	
		  // Transform segment to the sensor 0 coordinate system
		  if (sensor > 0) {
		    start_pos.vect() = sensor_rotation_in_sensor0[sensor] * 
			                   segment->BeginPoint().vect() +
							   sensor_offset_in_sensor0[sensor];
		    end_pos.vect() = sensor_rotation_in_sensor0[sensor] * 
			                 segment->EndPoint().vect() +
							 sensor_offset_in_sensor0[sensor];
            line_segment = LineSegment3D(start_pos, end_pos);
		  }
		  else line_segment = *segment;
		  
       	  for (sensor2=sensor+1; sensor2<3; sensor2++) {
      	    for (segment2=line_segments_sensor[sensor2][interval].begin();
			     segment2!=line_segments_sensor[sensor2][interval].end();
			     segment2++) {
			     	
		      // Transform segment to the sensor 0 coordinate system
		      start_pos.vect() = sensor_rotation_in_sensor0[sensor2] * 
			                     segment2->BeginPoint().vect() +
							     sensor_offset_in_sensor0[sensor2];
		      end_pos.vect() = sensor_rotation_in_sensor0[sensor2] * 
			                   segment2->EndPoint().vect() +
							   sensor_offset_in_sensor0[sensor2];
              line_segment2 = LineSegment3D(start_pos, end_pos);
			  
			  // Check angle between segments (as parallel segments are always coplanar)
              if (Angle(line_segment.Direction(), line_segment2.Direction()) <
			      minimum_angle_between_coplanar_segments) continue;
			      
			  // Check perpendicular distance between segments
			  if (Distance2Lines(line_segment.Line3DReference(),
			                     line_segment2.Line3DReference()) >
				  maximum_distance_to_plane) continue;

              // Check distance between segments in common plane
              if (line_segment.Distance(line_segment2) >
			      maximum_distance_between_coplanar_segments) continue;
			      
			  // Add observation equation for coplanar lines. Note that the
			  // original segments are used (segment, segment2), not those
			  // transformed with the approximate registration parameters
			  // (line_segment, line_segment2).  
	          CoplanarLines(interval, sensor, sensor2, 
	                        segment->Line3DReference(), 
							segment2->Line3DReference(),
		                    ata, aty, x_approx, num_par, verbose, 
							num_coplanar_lines, sqsum_coplanar_lines, true, residual);
			  bin = (int) ((residual + maximum_distance_to_plane) / 0.01);
			  if (bin < 0) bin = 0;
			  if (bin >= num_bins) bin = num_bins-1;
			  residual_histogram[bin]++;
              
      	      // Create plane through the two segments
      	      plane.Erase(); // Clear previous data
      	      plane.AddPoint(line_segment.BeginPoint(), false);
      	      plane.AddPoint(line_segment.EndPoint(), false);
      	      plane.AddPoint(line_segment2.BeginPoint(), false);
      	      plane.AddPoint(line_segment2.EndPoint(), false);
      	      plane.Recalculate();
      	      
      	      // Save the plane and both segments to check perpendicularity 
			  // constraints later
      	      planes.push_back(plane);
			  plane_segments1.push_back(segment);
			  plane_segments2.push_back(segment2);	

      	      // Check for triplets of coplanar direction vectors
      	      if (sensor == 0 && sensor2 == 1) {
      	      	
      	      	// Loop over segments of sensor 2
      	      	for (segment3=line_segments_sensor[2][interval].begin();
			         segment3!=line_segments_sensor[2][interval].end();
			         segment3++) {

        	      // Transform segment with approximate registration parameters
		          start_pos.vect() = sensor_rotation_in_sensor0[2] * 
			                         segment3->BeginPoint().vect() +
							         sensor_offset_in_sensor0[2];
		          end_pos.vect()   = sensor_rotation_in_sensor0[2] * 
			                         segment3->EndPoint().vect() +
							         sensor_offset_in_sensor0[2];
        	      
      	      	  // Check distance of end points to plane
      	      	  if (fabs(plane.Distance(start_pos)) > 
					  maximum_distance_to_plane) continue;
      	      	  if (fabs(plane.Distance(end_pos)) > 
					  maximum_distance_to_plane) continue;
					  
      	      	  // Add observation equation for coplanar direction vectors
		          CoplanarVectors(interval, 
	                              segment->Line3DReference(), 
							      segment2->Line3DReference(),
							      segment3->Line3DReference(),
		                          ata, aty, x_approx, verbose, 
		                          num_coplanar_vectors, sqsum_coplanar_vectors,
								  true);
		        }
      	      }
		    }
		  }	
      	}
      }
      
      // Add perpendicularity constraints
      if (use_perpendicularity) {
        for (plane1=planes.begin(), i=0; plane1!=planes.end(); plane1++, i++) {
          for (plane2=plane1+1, j=i+1; plane2!=planes.end(); plane2++, j++) {
          	angle = Angle(plane1->Normal(), plane2->Normal());
          	angle -= 90.0 * degree;
          	if (fabs(angle) <= maximum_angle_deviation_perpendicularity) {
          	  PerpendicularPlanes(interval,
				    plane_segments1[i]->Label(), plane_segments2[i]->Label(),
				    plane_segments1[j]->Label(), plane_segments2[j]->Label(),
				    plane_segments1[i]->Line3DReference(),
					plane_segments2[i]->Line3DReference(),
				    plane_segments1[j]->Line3DReference(),
					plane_segments2[j]->Line3DReference(),
                    ata, aty, x_approx, num_par, verbose, 
					num_perpendicular_planes, sqsum_perpendicular_planes, true);
          	}
          }
        }
      }
    }
    
    if (num_iter == 1) {
      if (use_perpendicularity)
	    printf("Found %d coplanar line segments, %d coplanar vector triplets, and %d perpendicular planes.\n",
	           num_coplanar_lines, num_coplanar_vectors,
			   num_perpendicular_planes);
      else
	    printf("Found %d coplanar line segments and %d coplanar vector triplets.\n",
	           num_coplanar_lines, num_coplanar_vectors);
	}

/*
    if (num_iter == 1) {
      printf("ATA\n");
      for (i=0; i<num_par; i++) {
        for (j=0; j<num_par; j++) printf("%5.1f ", ata[i*num_par+j]);
        printf("\n");
      }
    }
*/
  
	// Solve equation system and check estimability of parameters
    Invert_And_Solve_Normal_Eq(ata, aty, num_par, &cond);
    num_obs = num_coplanar_lines + num_coplanar_vectors + num_perpendicular_planes;
    sqsum_obs = sqsum_coplanar_lines + sqsum_coplanar_vectors + sqsum_perpendicular_planes;
    printf("Iteration %2d  stdev obs %.4f  # obs %6d  Condition number %.4e\n  stdevs",
	       num_iter, sqrt(sqsum_obs/num_obs), num_obs, cond);
    if (num_coplanar_lines)
      printf(" coplanar lines %.4f (%d)",
	         sqrt(sqsum_coplanar_lines/num_coplanar_lines), num_coplanar_lines);
    if (num_coplanar_vectors)
      printf(" coplanar vectors %.4f (%d)",
	         sqrt(sqsum_coplanar_vectors/num_coplanar_vectors), num_coplanar_vectors);
    if (num_perpendicular_planes)
      printf(" perpendicular planes %.4f (%d)",
	         sqrt(sqsum_perpendicular_planes/num_perpendicular_planes) / degree,
			 num_perpendicular_planes);
    printf("\n");
    
    // Update parameters
    for (i=0; i<num_par; i++) x_approx[i] += aty[i];

    // Update relative poses in vectors and rotation matrices
    sensor_rotation_in_sensor0[1] = Rotation3D(x_approx[0], x_approx[1], x_approx[2]);
    sensor_offset_in_sensor0[1] = Vector3D(x_approx[3], x_approx[4], x_approx[5]);
    sensor_rotation_in_sensor0[2] = Rotation3D(x_approx[6], x_approx[7], x_approx[8]);
    sensor_offset_in_sensor0[2] = Vector3D(x_approx[9], x_approx[10], x_approx[11]);

    // Print increments, parameters, correlations
    printf("Increments and new estimates:\n");
    for (i=0; i<num_par; i++) {
      printf("R%d  ", i/6 + 1);
      if (i<3 || (i>5 && i<9 || (i>11 && i<15)))
	    printf("%2d  %10.5f %10.5f\n", i, aty[i] / degree, x_approx[i] / degree);
	  else
	    printf("%2d  %10.5f %10.5f\n", i, aty[i], x_approx[i]);
    }
	if (num_iter == 1) {
      Convert_To_Correlations(ata, num_par);
      printf("Correlations\n");
      for (i=0; i<num_par; i++) {
        for (j=0; j<num_par; j++) printf("%5.2f ", ata[i*num_par+j]);
        printf("\n");
      }
    }

    // Check for convergence
    converged = true;
    for (i=0; i<num_par; i++) {
      if (i<3 || (i>5 && i<9 || (i>11 && i<15))) { // Angle
        if (fabs(aty[i]) > 0.00001 * degree) converged = false;
      }
      else { // Translation
      	if (fabs(aty[i]) > 0.00001) converged = false;
      }
    }
    
    // Print histogram of residuals
    if (num_iter == 1 || converged || num_iter == max_iter) {
      if (num_iter == 1) printf("Frequencies of initial coplanarity residuals:\n");
      else printf("Frequences of final coplanarity residuals:\n");
      for (bin=0, residual=-maximum_distance_to_plane; bin<num_bins;
	       bin++, residual+=0.01) {
  	    printf("%5.2f - %5.2f: %6d\n", residual, residual+0.01,
	           residual_histogram[bin]);
      }
    }
  }

  if (converged) printf("Convergence in %d iterations\n", num_iter);
  else printf("No convergence after %d iterations\n", num_iter);
  
  // Print standard deviations of estimated parameters
  printf("Standard deviations of estimated parameters:\n");
  for (i=0; i<num_par; i++) {
    printf("R%d  ", i/6 + 1);
    if (i<3 || (i>5 && i<9 || (i>11 && i<15)))
      printf("%2d  %10.5f\n", i, sqrt(sqsum_obs/num_obs) * sqrt(ata[i+i*num_par]) / degree);
	else
	  printf("%2d  %10.5f\n", i, sqrt(sqsum_obs/num_obs) * sqrt(ata[i+i*num_par]));
  }
  
  // Update the system configuration data
  for (sensor=0; sensor<3; sensor++) {
    system[sensor].Orientation()->SetDateNow();
    system[sensor].Orientation()->Orientation3DRef() =
      Orientation3D(sensor_offset_in_sensor0[sensor],
	                sensor_rotation_in_sensor0[sensor]);
  }
  system.Write(output_sysconf_file);
}


