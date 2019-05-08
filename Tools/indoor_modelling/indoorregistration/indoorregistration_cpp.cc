
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

 Enabled reading in approximate values from lasconf file
 Author: Michael Peter
 Date  : 23-05-2018

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
#include "Planes.h"
#include "Line3D.h"
#include "normal_equations.h"
#include "LaserScanningSystem.h"

Vector3D   W_X     = Vector3D(1.0, 0.0, 0.0);
Vector3D   W_Y     = Vector3D(0.0, 1.0, 0.0);
Vector3D   W_Z     = Vector3D(0.0, 0.0, 1.0);

void CoplanarLines(int scan, int plane_num, int sensor1, int sensor2, 
			       Line3D &line1, Line3D &line2,
		           double *ata, double *aty, double *x_approx, int num_par,
				   bool verbose, int &num_obs, double &sqsum_obs,
				   bool update_normal_equations)
{
  double     a[12], y;
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
  if (verbose) printf("CL: scan %d, plane %d, sensors %d %d, misclosure %9.5f\n",
                      scan, plane_num, sensor1, sensor2, y);
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



void CoplanarVectors(int scan, int plane_num,
			         Line3D &line0, Line3D &line1, Line3D &line2,
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
  if (verbose) printf("CV: scan %d, plane %d, sensors 0 1 2, misclosure %9.5f\n",
                      scan, plane_num, y);
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

void PerpendicularPlanes(int scan, int plane_num1, int plane_num2, 
                         int sensor1, int sensor2, 
			             Line3D &line1a, Line3D &line2a, 
						 Line3D &line1b, Line3D &line2b,
						 double *ata, double *aty, double *x_approx, 
						 int num_par, bool verbose, 
						 int &num_obs, double &sqsum_obs,
						 bool update_normal_equations)
{
  Vector3D   rot1l1a, rot1l1b, rot2l2a, rot2l2b, ma, mb,
             q1a, q1b, q2a, q2b, l1a, l1b, l2a, l2b;
  Rotation3D rot1, rot2, W, Z_omega, Z_phi, Z_kappa;
  double     y, a[12];
  int        offset, i, j;

  // Derive approximate rotation matrices
  if (sensor1 == 0) {
  	if (sensor2 == 1) offset = 0; else offset = 6;
  	rot1 = Rotation3D(0.0, 0.0, 0.0);
  	rot2 = Rotation3D(x_approx[offset], x_approx[offset+1], x_approx[offset+2]);
  }
  else {
  	offset = 6;
  	rot1 = Rotation3D(x_approx[0], x_approx[1], x_approx[2]);
  	rot2 = Rotation3D(x_approx[6], x_approx[7], x_approx[8]);
  }
  
  // Some common terms
  rot1l1a = rot1 * line1a.Direction();
  rot2l2a = rot2 * line2a.Direction();
  rot1l1b = rot1 * line1b.Direction();
  rot2l2b = rot2 * line2b.Direction();
  l1a     = line1a.Direction();
  l1b     = line1b.Direction();
  l2a     = line2a.Direction();
  l2b     = line2b.Direction();
  ma      = rot1l1a.VectorProduct(rot2l2a); // Unscaled normal vector of plane a
  mb      = rot1l1b.VectorProduct(rot2l2b); // Unscaled normal vector of plane b

  // Observation
  y = -1.0 * ma.DotProduct(mb);
  num_obs++;
  sqsum_obs += y * y;
  if (verbose) printf("PP: scan %d, planes %d %d, sensors %d %d, misclosure %9.5f\n", scan, 
                      plane_num1, plane_num2, sensor1, sensor2, y);
  if (!update_normal_equations) return;
  
  // Initialise partial derivatives
  for (i=0; i<12; i++) a[i] = 0.0;
  
  // Partial derivatives
  if (sensor1 == 1) {
   Z_omega = rot1.PartialDeriv(0);
   Z_phi   = rot1.PartialDeriv(1);
    Z_kappa = rot1.PartialDeriv(2);
    a[0] = ((Z_omega * l1a).VectorProduct(rot2l2a)).DotProduct(mb) +
           ma.DotProduct((Z_omega * l1b).VectorProduct(rot2l2b));
    a[1] = ((Z_phi   * l1a).VectorProduct(rot2l2a)).DotProduct(mb) +
           ma.DotProduct((Z_phi   * l1b).VectorProduct(rot2l2b));
    a[2] = ((Z_kappa * l1a).VectorProduct(rot2l2a)).DotProduct(mb) +
           ma.DotProduct((Z_kappa * l1b).VectorProduct(rot2l2b));
  }
  Z_omega = rot2.PartialDeriv(0);
  Z_phi   = rot2.PartialDeriv(1);
  Z_kappa = rot2.PartialDeriv(2);
  a[offset]   = (rot1l1a.VectorProduct(Z_omega * l2a)).DotProduct(mb) +
                ma.DotProduct(rot1l1b.VectorProduct(Z_omega * l2b));
  a[offset+1] = (rot1l1a.VectorProduct(Z_phi * l2a)).DotProduct(mb) +
                ma.DotProduct(rot1l1b.VectorProduct(Z_phi * l2b));
  a[offset+2] = (rot1l1a.VectorProduct(Z_kappa * l2a)).DotProduct(mb) +
                ma.DotProduct(rot1l1b.VectorProduct(Z_kappa * l2b));

  // Update equation system
  Update_Normal_Eq(a, y, 1.0, ata, aty, num_par);  
}


// Main indoor registration function

void indoorregistration_cpp(char *i0_file, char *i1_file, char *i2_file,
                            char *c0_file, char *c1_file, char *c2_file,
							double scanner_rotation, bool verbose, int max_iter,
							int max_num_planes, char *o_file, char *system_configuration_file, bool use_perpendicularity_constraints, bool in_frame_system)
{
  Vector3D              sensor_offset_in_frame[3], sensor_offset_in_sensor0[3],
                        frame_offset_in_sensor0, rotcolX, rotcolY, rotcolZ;
  Rotation3D            sensor_rotation_in_frame[3], sensor_rotation_in_sensor0[3],
                        frame_rotation_in_sensor0;
  LaserPoints           observations[3], selection;
  LaserPoints::iterator point;
  double                angle0_start, angle1_start, angle2_start,
                        degree = atan(1.0) / 45.0, *x_approx,
						*ata, *aty, omega, phi, kappa, cond, sqsum_obs;
  Position3D            reflection_pos;
  bool                  double_wall=true, more_scans, converged;
  int                   sensor, sensor2, scan, plane_num, plane_num2, num_par,
                        num_lines, num_iter, i, j, num_sensors, num_obs, offset,
						index;
  Line3D                lines[3][3], base_line_in_sensor0;
  LaserScanningSystem   scanner_system, scanner_system_in, *scanner_system_in_ptr;
  LaserScanner          scanner;

  // Read observation files
  if (!observations[0].Read(i0_file)) {
  	printf("Error reading observation of sensor 0 from %s\n", i0_file);
  	exit(0);
  }
  if (!observations[1].Read(i1_file)) {
	printf("Error reading observation of sensor 1 from %s\n", i1_file);
  	exit(0);
  }
  if (!observations[2].Read(i2_file)) {
  	printf("Error reading observation of sensor 2 from %s\n", i2_file);
  	exit(0);
  }
  num_sensors = 3;
  num_par = (num_sensors - 1) * 6;

  // Read the scanner system configuration
  if (system_configuration_file) {
  	if (!scanner_system_in.Read(system_configuration_file)) {
  	  printf("Error reading scanner system configuration file %s\n",
		     system_configuration_file);
	  exit(0);
  	}
 	scanner_system_in_ptr = &scanner_system_in;
  }
  else scanner_system_in_ptr = NULL;

  // Default approximate sensor in frame poses
  // Code copied from indoorregistrationsimulation, but approximate values
  // may be changed to test parameter estimation.
//  sensor_offset_in_frame[0] = Vector3D(0.2, 0.0, 0.5);  // Testing
//  sensor_offset_in_frame[0] = Vector3D(0.0, 0.0, 0.5);
//  sensor_offset_in_frame[1] = Vector3D(0.0, -0.25, 0.0);
//  sensor_offset_in_frame[2] = Vector3D(0.0, 0.25, 0.0);
	if(!scanner_system_in_ptr)
	{
	  sensor_offset_in_frame[0] = Vector3D(0.0, 0.0, 0.0);
	  sensor_offset_in_frame[1] = Vector3D(-0.29, -0.17, -0.20);
	  sensor_offset_in_frame[2] = Vector3D(0.29, -0.17, -0.20);
	}
	else
	{
	  // use the parameters from the input lasconf file
	  sensor_offset_in_frame[0] = scanner_system_in[0].Orientation()->vect();
	  sensor_offset_in_frame[1] = scanner_system_in[1].Orientation()->vect();
	  sensor_offset_in_frame[2] = scanner_system_in[2].Orientation()->vect();
	}	
    
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
/* Testing
  sensor_rotation_in_frame[0] = Rotation3D(10.0 * degree, 0.0 * degree, angle0_start);
  sensor_rotation_in_frame[1] = Rotation3D(1.0 * degree, -90.0 * degree, 0.0) *
                               Rotation3D(45.0 * degree, 0.0, 0.0) *
							   Rotation3D(0.0 * degree, scanner_rotation * degree, 0.0) *
							   Rotation3D(0.0, 0.0, angle1_start);
  sensor_rotation_in_frame[2] = Rotation3D(0.0 * degree, -90.0 * degree, 0.0) *
                               Rotation3D(-45.0 * degree, 0.0, 0.0) *
							   Rotation3D(0.0, scanner_rotation * degree, 0.0) *
							   Rotation3D(0.0, 0.0, angle2_start);
*/
	if(!scanner_system_in_ptr)
	{
	  sensor_rotation_in_frame[0] = Rotation3D(0.0 * degree, 0.0 * degree, 0.0*degree);
	  sensor_rotation_in_frame[1] = Rotation3D(0 * degree, -45 * degree, 0 * degree)*Rotation3D(70 * degree, 0 * degree, 0 * degree)*Rotation3D(0 * degree, 0 * degree, -135 * degree);
	  sensor_rotation_in_frame[2] = Rotation3D(0 * degree, 45 * degree, 0 * degree)*Rotation3D(70 * degree, 0 * degree, 0 * degree)*Rotation3D(0 * degree, 0 * degree, 135 * degree);\
    }
    else
    {
	  sensor_rotation_in_frame[0] = scanner_system_in[0].Orientation()->rotation();
	  sensor_rotation_in_frame[1] = scanner_system_in[1].Orientation()->rotation();
	  sensor_rotation_in_frame[2] = scanner_system_in[2].Orientation()->rotation();
    }
    
    if (in_frame_system)
    {
	  for (point=observations[1].begin(); point!=observations[1].end(); point++) {
	  	point->vect() = sensor_rotation_in_frame[1].Transpose() * point->vect() - sensor_offset_in_frame[1];
	  }
	  for (point=observations[2].begin(); point!=observations[2].end(); point++) {
	  	point->vect() = sensor_rotation_in_frame[2].Transpose() * point->vect() - sensor_offset_in_frame[2];
	  }
    }

    
//  sensor_rotation_in_frame[0] = Rotation3D(0.0 * degree, 0.0 * degree, angle0_start);
//  sensor_rotation_in_frame[1] = Rotation3D(0.0 * degree, -90.0 * degree, 0.0) *
//                               Rotation3D(45.0 * degree, 0.0, 0.0) *
//							   Rotation3D(0.0 * degree, scanner_rotation * degree, 0.0) *
//							   Rotation3D(0.0, 0.0, angle1_start);
//  sensor_rotation_in_frame[2] = Rotation3D(0.0 * degree, -90.0 * degree, 0.0) *
//                               Rotation3D(-45.0 * degree, 0.0, 0.0) *
//							   Rotation3D(0.0, scanner_rotation * degree, 0.0) *
//							   Rotation3D(0.0, 0.0, angle2_start);

  // Allocate equation system
  x_approx = (double *) malloc(num_par * sizeof(double));
  ata      = (double *) malloc(num_par * num_par * sizeof(double));
  aty      = (double *) malloc(num_par * sizeof(double));
  
  // Determine approximate relative poses of sensors w.r.t. sensor 0
  sensor_rotation_in_sensor0[0] = Rotation3D(0.0, 0.0, 0.0);
  sensor_offset_in_sensor0[0] = Vector3D(0.0, 0.0, 0.0);
  for (sensor=1; sensor<num_sensors; sensor++) {
    sensor_rotation_in_sensor0[sensor] = sensor_rotation_in_frame[0].Transpose() * 
                                         sensor_rotation_in_frame[sensor];
    sensor_offset_in_sensor0[sensor] = sensor_rotation_in_frame[0].Transpose() * 
                (sensor_offset_in_frame[sensor] - sensor_offset_in_frame[0]);
  }

  // Set initial approximate values of relative poses
  for (sensor=1; sensor<num_sensors; sensor++) {
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
  while (!converged && num_iter < max_iter) {
    num_iter++;
  	
    // Initialise equation system
    memset((void *) ata, 0, num_par * num_par * sizeof(double));
    memset((void *) aty, 0, num_par * sizeof(double));
    num_obs = 0;
    sqsum_obs = 0.0;
  
    // Loop over all scan lines
    more_scans = true;
    scan = 0;
    while (more_scans) {
  	
	  // Determine all line segments in this scan
  	  num_lines = 0;
  	  for (sensor=0; sensor<num_sensors; sensor++) { // Loop over sensors
  	    for (plane_num=0; plane_num<max_num_planes; plane_num++) {  // Loop over planes
  	      selection.ErasePoints();
  	      selection.AddTaggedPoints(observations[sensor],
	                                scan, ScanLineNumberTag,
								    plane_num, PlaneNumberTag,
								    IsSelectedTag);
		  observations[sensor].RemoveAttribute(IsSelectedTag);
		  if (selection.size() >= 10) {
		    lines[plane_num][sensor] = selection.FitLine(selection.begin(), selection.end()-1);
            num_lines++;
		  }
		  else
		    lines[plane_num][sensor] = LineSegment3D();
  	    }
  	  }
  	  if (num_lines == 0) {more_scans = false; continue;}
  	  
	  if (num_iter == 1) printf("Scan %d, %d fitted lines\n", scan, num_lines);
	
  	  // Add observations to equation system
  	
	  // Coplanarity of two lines in a plane
	  for (plane_num=0; plane_num<max_num_planes; plane_num++) {
	    for (sensor=0; sensor<num_sensors-1; sensor++) {
	  	  for (sensor2=sensor+1; sensor2<num_sensors; sensor2++) {
	        if (lines[plane_num][sensor].NumberOfPoints() &&
	            lines[plane_num][sensor2].NumberOfPoints())
	          CoplanarLines(scan, plane_num, sensor, sensor2, 
		                    lines[plane_num][sensor], lines[plane_num][sensor2],
		                    ata, aty, x_approx, num_par, verbose, 
							num_obs, sqsum_obs, true);
	  	  }
	    }
	  }
	
	  // Coplanarity of three direction vectors in a plane
	  for (plane_num=0; plane_num<max_num_planes; plane_num++) {
	    if (lines[plane_num][0].NumberOfPoints() &&
	        lines[plane_num][1].NumberOfPoints() &&
		    lines[plane_num][2].NumberOfPoints())
		  CoplanarVectors(scan, plane_num, 
		                  lines[plane_num][0], lines[plane_num][1], 
		                  lines[plane_num][2], ata, aty, x_approx, verbose, 
				  		  num_obs, sqsum_obs, true);
	  }	
	
	  if(use_perpendicularity_constraints)
	  {
		  // Perpendicularity of planes
		  for (plane_num=0; plane_num<max_num_planes-1; plane_num++) {
		    for (plane_num2=plane_num+1; plane_num2<max_num_planes; plane_num2++) {
		      for (sensor=0; sensor<num_sensors-1; sensor++) {
		  	    for (sensor2=sensor+1; sensor2<num_sensors; sensor2++) {
		          if (lines[plane_num][0].NumberOfPoints() &&
		              lines[plane_num][1].NumberOfPoints() &&
		              lines[plane_num2][0].NumberOfPoints() &&
		              lines[plane_num2][1].NumberOfPoints())
		            PerpendicularPlanes(scan, plane_num, plane_num2, sensor, sensor2, 
				                        lines[plane_num][sensor], lines[plane_num][sensor2],
			                            lines[plane_num2][sensor], lines[plane_num2][sensor2],
			                            ata, aty, x_approx, num_par, verbose, 
							  			num_obs, sqsum_obs, true);
			    }
			  }
		    }
		  }
      }
      
      scan++;
    }

    if (num_iter == 1) {
      printf("ATA\n");
      for (i=0; i<num_par; i++) {
        for (j=0; j<num_par; j++) printf("%5.1f ", ata[i*num_par+j]);
        printf("\n");
      }
    }
    
	// Solve equation system and check estimability of parameters
    Invert_And_Solve_Normal_Eq(ata, aty, num_par, &cond);
    printf("Iter %2d  stdev obs  %.4f      Condition number %.4e\n",
	       num_iter, sqrt(sqsum_obs/num_obs), cond);
    
    // Update parameters
    for (i=0; i<num_par; i++) x_approx[i] += aty[i];

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
  }
  if (converged) {
    printf("Convergence in %d iterations\n", num_iter);
    printf("Standard deviations of estimated parameters:\n");
    for (i=0; i<num_par; i++) {
      printf("R%d  ", i/6 + 1);
      if (i<3 || (i>5 && i<9 || (i>11 && i<15)))
	    printf("%2d  %10.5f\n", i, sqrt(sqsum_obs/num_obs) * sqrt(ata[i+i*num_par]) / degree);
	  else
	    printf("%2d  %10.5f\n", i, sqrt(sqsum_obs/num_obs) * sqrt(ata[i+i*num_par]));
    }
  }

  // Store the estimated relative poses in vectors and rotation matrices
  sensor_rotation_in_sensor0[1] = Rotation3D(x_approx[0], x_approx[1], x_approx[2]);
  sensor_offset_in_sensor0[1] = Vector3D(x_approx[3], x_approx[4], x_approx[5]);
  sensor_rotation_in_sensor0[2] = Rotation3D(x_approx[6], x_approx[7], x_approx[8]);
  sensor_offset_in_sensor0[2] = Vector3D(x_approx[9], x_approx[10], x_approx[11]);

  // Transform observations of sensors 1 and 2 to sensor 0 coordinate system
  // to verify the registration
  for (point=observations[1].begin(); point!=observations[1].end(); point++) {
  	point->vect() = sensor_rotation_in_sensor0[1] * point->vect() + sensor_offset_in_sensor0[1];
  	observations[0].push_back(*point);
  }
  for (point=observations[2].begin(); point!=observations[2].end(); point++) {
  	point->vect() = sensor_rotation_in_sensor0[2] * point->vect() + sensor_offset_in_sensor0[2];
  	observations[0].push_back(*point);
  }
  observations[0].Write("registered_in_sensor0.laser", 0, false);
  
  /* This piece of code was used to explicitly set orientation values

  // OVERRIDING WHATEVER IS CALCULATED ABOVE !!!
  printf("\n\n          OVERRIDING CALCULATIONS WITH HARD CODED NUMBERS!\n\n");
  sensor_offset_in_sensor0[1] = Vector3D(-0.27749, -0.07828, -0.20444);
  sensor_offset_in_sensor0[2] = Vector3D(0.27227, -0.07822, -0.20660);
  sensor_rotation_in_sensor0[1] = 
    Rotation3D(44.56850 * degree, -28.72257 * degree, -136.85441 * degree);
  sensor_rotation_in_sensor0[2] =
    Rotation3D(44.68575 * degree, 28.89024 * degree, 45.86189 * degree);
  printf("Sensor 1 rotation in sensor 0 system:\n");
  for (i=0; i<3; i++) {
  	for (j=0; j<3; j++) printf("%10.7f ", sensor_rotation_in_sensor0[1].R(i,j));
  	printf("\n");
  }
  printf("Sensor 2 rotation in sensor 0 system:\n");
  for (i=0; i<3; i++) {
  	for (j=0; j<3; j++) printf("%10.7f ", sensor_rotation_in_sensor0[2].R(i,j));
  	printf("\n");
  }
  // END OF OVERRIDING CODE
*/  

  // Save orientation data
  scanner.ReInitialise(); // Scanner 0
  if (!scanner.Read(c0_file) && !scanner_system_in_ptr) {
  	printf("Error reading calibration data of sensor 0 from %s\n", c0_file);
  	exit(0);
  }
  else if(scanner_system_in_ptr)
  {
  	scanner=scanner_system_in[0];
  }
  scanner.InitialiseOrientationData();
  scanner.Orientation()->Orientation3DRef() =
    Orientation3D(sensor_offset_in_sensor0[0], sensor_rotation_in_sensor0[0]);
  scanner.Orientation()->SetDateNow();
  scanner_system.push_back(scanner);
  
  scanner.ReInitialise(); // Scanner 1
  if (!scanner.Read(c1_file) && !scanner_system_in_ptr) {
  	printf("Error reading calibration data of sensor 1 from %s\n", c1_file);
  	exit(0);
  }
  else if(scanner_system_in_ptr)
  {
  	scanner=scanner_system_in[1];
  }
  scanner.InitialiseOrientationData();
  scanner.Orientation()->Orientation3DRef() =
    Orientation3D(sensor_offset_in_sensor0[1], sensor_rotation_in_sensor0[1]);
  scanner.Orientation()->SetDateNow();
  scanner_system.push_back(scanner);
  
  scanner.ReInitialise(); // Scanner 2
  if (!scanner.Read(c2_file) && !scanner_system_in_ptr) {
  	printf("Error reading calibration data of sensor 2 from %s\n", c2_file);
  	exit(0);
  }
  else if(scanner_system_in_ptr)
  {
  	scanner=scanner_system_in[2];
  }
  scanner.InitialiseOrientationData();
  scanner.Orientation()->Orientation3DRef() =
    Orientation3D(sensor_offset_in_sensor0[2], sensor_rotation_in_sensor0[2]);
  scanner.Orientation()->SetDateNow();
  scanner_system.push_back(scanner);
  
  scanner_system.Write(o_file);
  
/* Above the orientation data has been stored w.r.t. the coordinate system
   of sensor 0. The code below can be used to store all orientation data
   w.r.t. the frame coordinate system. It is currently not used.
  
  
  // Reconstruct sensor poses w.r.t. the frame coordinate system
  // The frame origin is the projection of S0 onto the axis through S1 and S2
  // The Z-direction is from the origin to S0
  // The Y-direction is from the origin to S2
  
  // Line through sensors 1 and 2 in sensor 0 coordinate system
  base_line_in_sensor0 = Line3D(Position3D(sensor_offset_in_sensor0[1]),
                                Position3D(sensor_offset_in_sensor0[2]));
  // Project sensor 0 onto this line
  frame_offset_in_sensor0 = base_line_in_sensor0.Project(Position3D()).vect();
  // Compose the rotation matrix of the frame in sensor 0 coordinate system
  rotcolZ = -1.0 * frame_offset_in_sensor0.Normalize();
  rotcolY = (sensor_offset_in_sensor0[2] - frame_offset_in_sensor0).Normalize();
  rotcolX = rotcolY.VectorProduct(rotcolZ);
  for (i=0; i<3; i++) frame_rotation_in_sensor0.R(i, 0) = rotcolX.X(i);
  for (i=0; i<3; i++) frame_rotation_in_sensor0.R(i, 1) = rotcolY.X(i);
  for (i=0; i<3; i++) frame_rotation_in_sensor0.R(i, 2) = rotcolZ.X(i);
  
  // Determine rotations and offsets of the three sensors in the frame
  // coordinate system
  sensor_rotation_in_frame[0] = frame_rotation_in_sensor0.Transpose();
  sensor_offset_in_frame[0] = -1.0 * frame_offset_in_sensor0;
  sensor_rotation_in_frame[1] = sensor_rotation_in_frame[0] *
                                sensor_rotation_in_sensor0[1];
  sensor_offset_in_frame[1] = sensor_rotation_in_frame[0] *
                              sensor_offset_in_sensor0[1] +
                              sensor_offset_in_frame[0];
  sensor_rotation_in_frame[2] = sensor_rotation_in_frame[0] *
                                sensor_rotation_in_sensor0[2];
  sensor_offset_in_frame[2] = sensor_rotation_in_frame[0] *
                              sensor_offset_in_sensor0[2] +
                              sensor_offset_in_frame[0];
  
  // Save orientation data
  scanner.ReInitialise(); // Scanner 0
  if (!scanner.Read(c0_file)) {
  	printf("Error reading calibration data of sensor 0 from %s\n", c0_file);
  	exit(0);
  }
  scanner.InitialiseOrientationData();
  scanner.Orientation()->Orientation3DRef() =
    Orientation3D(sensor_offset_in_frame[0], sensor_rotation_in_frame[0]);
  scanner.Orientation()->SetDateNow();
  scanner_system.push_back(scanner);
  
  scanner.ReInitialise(); // Scanner 1
  if (!scanner.Read(c1_file)) {
  	printf("Error reading calibration data of sensor 1 from %s\n", c1_file);
  	exit(0);
  }
  scanner.InitialiseOrientationData();
  scanner.Orientation()->Orientation3DRef() =
    Orientation3D(sensor_offset_in_frame[1], sensor_rotation_in_frame[1]);
  scanner.Orientation()->SetDateNow();
  scanner_system.push_back(scanner);
  
  scanner.ReInitialise(); // Scanner 2
  if (!scanner.Read(c2_file)) {
  	printf("Error reading calibration data of sensor 2 from %s\n", c2_file);
  	exit(0);
  }
  scanner.InitialiseOrientationData();
  scanner.Orientation()->Orientation3DRef() =
    Orientation3D(sensor_offset_in_frame[2], sensor_rotation_in_frame[2]);
  scanner.Orientation()->SetDateNow();
  scanner_system.push_back(scanner);
  
  scanner_system.Write(o_file);
  */
}

