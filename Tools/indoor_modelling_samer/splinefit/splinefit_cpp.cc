
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
 Date   : 10-12-2013

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

#include "BSplineFit.h"

/*
--------------------------------------------------------------------------------
                      The main splinefit_cpp function
--------------------------------------------------------------------------------
*/

void splinefit_cpp(char *point_file, char *spline_file, int order,
                   double knot_interval_size, double min_knot_distance)
{
  LaserPoints           points, x_points;
  LaserPoints::iterator point, last_knot_point;
  LaserPoint            spline_point, x_point;
  BSplineFit            x_spline, y_spline, z_spline, x_spline2;
  int                   ip, num_points;
  double                time_start, time_end, dist, sqsum, sqsum_xy, sqsum_z,
                        emax, emax_xy, emax_z, last_knot_time, time, sqsum2,
						degree = atan(1.0) / 45.0, angle, max_angle=360.0,
						obs_int=30.0;
  std::vector<double>   knots;


  // testing derivatives on sine function
  printf("Spline order %d\n", order);
  if (min_knot_distance == 0.0) { // Uniform splines
    printf("Uniform splines, knot interval %.2f degree\n", knot_interval_size);
    x_spline.Initialise(order, 0.0, max_angle * degree, knot_interval_size * degree);
  }
  else { // Non-uniform, but for testing at constant interval of 10 degrees
    printf("Non-uniform splines at constant knot interval of 10 degrees\n");
  	for (angle=0; angle<max_angle+0.01; angle+=10.0)
  	  knots.push_back(angle * degree);
  	x_spline.Initialise(order, knots);
  }
  
  printf("Number of B-splines: %d\n", x_spline.NumberOfSplines());
  for (angle=0; angle<max_angle+0.01; angle+=obs_int)
    x_spline.AddObservation(angle * degree, sin(angle * degree), 1.0, true);
  x_spline.FitSpline();
  
  // print bspline values for a few intervals
  printf("Bspline values\n");
  for (angle=0; angle<10.1; angle+=1.0) {
  	printf("%5.1f  %7.4f   %7.4f %7.4f\n",
	       angle, angle * degree,
		   x_spline.BSplineValue(angle * degree, 0),
		   x_spline.BSplineValue(angle * degree, 1));
  }

  // print bspline first and second derivative values for a few intervals
  printf("Bspline first derivatives\n");
  for (angle=0; angle<10.1; angle+=1.0) {
  	printf("%5.1f  %7.4f   %7.4f %7.4f   %7.4f %7.4f\n",
	       angle, angle * degree,
		   x_spline.BSplineDerivative(angle * degree, 0, 1),
		   x_spline.BSplineDerivative(angle * degree, 1, 1),
		   x_spline.BSplineDerivative(angle * degree, 0, 2),
		   x_spline.BSplineDerivative(angle * degree, 1, 2));
  }

  // print derivatives
  printf("Sine function and first and second derivatives\n");
  for (angle=0; angle<max_angle+0.01; angle+=obs_int)
    printf("%5.1f  %7.4f   %7.4f   %7.4f  %7.4f  %7.4f\n", 
	       angle, angle * degree, sin(angle * degree),
	       x_spline.Value(angle * degree),
		   x_spline.Derivative(angle * degree, 1),
		   x_spline.Derivative(angle * degree, 2));

  // Check sine function reconstruction based on curvature constraints
  // Initialise a new uniform or non-uniform spline
  if (min_knot_distance == 0.0) // Uniform splines
    x_spline2.Initialise(order, 0.0, max_angle * degree, knot_interval_size * degree);
  else
    x_spline2.Initialise(order, knots);
  
  // Set observations for end locations
  x_spline2.AddObservation(0.0 * degree, sin(0.0 * degree), 1.0, true);  
  x_spline2.AddObservation(max_angle * degree, sin(max_angle * degree), 1.0, true);
  
  // Just the start and end location are not enough.
  // We also need sine values at other locations or add first derivatives
  // at the start and end.

  // Additional sine value in the middle of the range
//  x_spline2.AddObservation(0.5 * max_angle * degree, sin(0.5 * max_angle * degree), 1.0, true);

  // Additional sine values close to the start and end of the range
//  x_spline2.AddObservation(obs_int * degree, sin(obs_int * degree), 1.0, true);  
//  x_spline2.AddObservation((max_angle - obs_int) * degree, sin((max_angle - obs_int) * degree), 1.0, true);

  // First derivatives
  x_spline2.AddConstraint(0.0 * degree, x_spline.Derivative(0.0, 1), 1, 1.0, true);
  x_spline2.AddConstraint(max_angle * degree, 
                          x_spline.Derivative(max_angle * degree, 1), 1, 1.0, true);
  
  // Only set derivatives for all times
  for (angle=0; angle<max_angle+0.01; angle+=obs_int) {
    // First derivative
	// x_spline2.AddConstraint(angle * degree, 
	//                        x_spline.Derivative(angle * degree, 1), 1, 1.0, true);
    // Second derivative
    x_spline2.AddConstraint(angle * degree, 
	                        x_spline.Derivative(angle * degree, 2), 2, 1.0, true);
  }
  
  // Adjust spline coefficients
  x_spline2.FitSpline();
  
  // Output of "X"-points
  x_points.ErasePoints();
  sqsum = sqsum2 = 0.0;
  for (angle=0; angle<max_angle+0.01; angle+=obs_int) {
  	// Original observations
  	x_point.X() = angle;
  	x_point.Y() = 180.0 * sin(angle * degree);
  	x_point.SetAttribute(LabelTag, 1);
  	x_points.push_back(x_point);
  	// Reconstruction by spline fitting to original observations
  	x_point.Y() = 180.0 * x_spline.Value(angle * degree);
  	x_point.SetAttribute(LabelTag, 2);
  	dist = x_spline.Value(angle * degree) - sin(angle * degree);
  	x_point.FloatAttribute(ResidualTag) = dist;
  	x_points.push_back(x_point);
  	sqsum += dist * dist;
  	// Reconstruction by spline fitting to derivatives
  	x_point.Y() = 180.0 * x_spline2.Value(angle * degree);
  	x_point.SetAttribute(LabelTag, 3);
  	dist = x_spline2.Value(angle * degree) - x_spline.Value(angle * degree);
  	x_point.FloatAttribute(ResidualTag) = dist;
  	x_points.push_back(x_point);
  	sqsum2 += dist * dist;
  	printf("%5.1f  %7.4f   %7.4f   %7.4f  %7.4f\n", 
	       angle, angle * degree, sin(angle * degree),
	       x_spline.Value(angle * degree),
		   x_spline2.Value(angle * degree));

  }
  printf("RMS dif sine and spline %.4f\n", sqrt(sqsum / (double) max_angle));
  printf("RMS dif spline and reconstructed spline %.4f\n", sqrt(sqsum2 / (double) max_angle));
  x_points.Write("x2.laser", 0, false);


  exit(0);


  // Read the points
  if (!points.Read(point_file, false)) {
  	printf("Error reading points from file %s\n", point_file);
  	exit(0);
  }
  
  // Check the presence of the time attribute and determine the time range
  if (!points.size()) {
  	printf("Error: No points in input file\n");
  	exit(0);
  }
  if (!points.begin()->HasAttribute(TimeTag)) {
  	printf("Error: Points do not have a time tag\n");
  	exit(0);
  }
  time_start = points.begin()->DoubleAttribute(TimeTag);
  time_end   = (points.end()-1)->DoubleAttribute(TimeTag);
  printf("Trajectory from %.6f to %.6f seconds, duration %.6f seconds\n",
         time_start, time_end, time_end - time_start);  
  
  // Initialise uniform splines
  points.SetAttribute(LabelTag, 1);
  if (min_knot_distance == 0.0) { // Uniform splines
    x_spline.Initialise(order, time_start, time_end, knot_interval_size);
    y_spline.Initialise(order, time_start, time_end, knot_interval_size);
    z_spline.Initialise(order, time_start, time_end, knot_interval_size);
    printf("Uniform splines with %d knots\n", x_spline.NumberOfKnots());
  }
  else { // Non-uniform splines
    knots.push_back(time_start);
    last_knot_point = points.begin();
    last_knot_time  = time_start;
    points.begin()->Label(0);
    for (point=points.begin()+1; point!=points.end(); point++) {
      time = point->DoubleAttribute(TimeTag);
      if (time - last_knot_time >= knot_interval_size) {
      	dist = point->Distance(last_knot_point->Position3DRef());
      	if (dist >= min_knot_distance) {
      	  // Add two or four knots to make sure to get the curve
      	  if (time - last_knot_time > 5.0 * knot_interval_size) {
      	  	knots.push_back(last_knot_time + knot_interval_size);
      	  	knots.push_back(last_knot_time + 2.0 * knot_interval_size);
      	  	knots.push_back(time - 2.0 * knot_interval_size);
      	  	knots.push_back(time - knot_interval_size);
      	  }
      	  else if (time - last_knot_time > 3.0 * knot_interval_size) {
      	  	knots.push_back(last_knot_time + knot_interval_size);
      	  	knots.push_back(time - knot_interval_size);
      	  }
      	  // Add the new knot
      	  knots.push_back(time);
      	  last_knot_point = point;
      	  last_knot_time  = time;
      	  point->Label(0);
      	}
      }
    }
    if (time != last_knot_time) {
      // Add two or four knots to make sure to get the curve
      if (time - last_knot_time > 5.0 * knot_interval_size) {
      	knots.push_back(last_knot_time + knot_interval_size);
      	knots.push_back(last_knot_time + 2.0 * knot_interval_size);
      	knots.push_back(time - 2.0 * knot_interval_size);
      	knots.push_back(time - knot_interval_size);
      }
      else if (time - last_knot_time > 3.0 * knot_interval_size) {
      	knots.push_back(last_knot_time + knot_interval_size);
      	knots.push_back(time - knot_interval_size);
      }
      knots.push_back(time);
    }
    (points.end()-1)->Label(0);
    x_spline.Initialise(order, knots);
    y_spline.Initialise(order, knots);
    z_spline.Initialise(order, knots);
    printf("Non-uniform splines with %d knots\n", x_spline.NumberOfKnots());
  }

  // Add observations
  for (point=points.begin(); point!=points.end(); point++) {
    time = point->DoubleAttribute(TimeTag);
  	x_spline.AddObservation(time, point->X());
  	y_spline.AddObservation(time, point->Y());
  	z_spline.AddObservation(time, point->Z());
  }
  
  // Estimate spline coefficients
  x_spline.FitSpline();
  y_spline.FitSpline();
  z_spline.FitSpline();

  // Reconstruct the trajectory with the splines
  num_points = points.size();
  spline_point.SetAttribute(LabelTag, 3);
  sqsum = sqsum_xy = sqsum_z = 0.0;
  emax = emax_xy = emax_z = 0.0;
  x_point.Z() = 0.0;
  for (ip=0; ip<num_points; ip++) {
  	// Set iterator outside "for" because of reallocation by push_back
  	point = points.begin() + ip; 
    time = point->DoubleAttribute(TimeTag);
  	// Reconstructed point
  	spline_point.X() = x_spline.Value(time);
  	spline_point.Y() = y_spline.Value(time);
  	spline_point.Z() = z_spline.Value(time);
  	spline_point.SetDoubleAttribute(TimeTag, time);
  	// Store X coordinates as function of time
  	x_point.X() = time;
  	x_point.Y() = point->X();
  	x_point.SetAttribute(LabelTag, point->Label());
  	x_points.push_back(x_point);
  	x_point.Y() = spline_point.X();
  	x_point.SetAttribute(LabelTag, 3);
  	x_point.FloatAttribute(ResidualTag) = fabs(spline_point.Z() - point->Z());
  	x_points.push_back(x_point);
  	// Error statistics
    dist = point->Distance(spline_point.Position3DRef());
    spline_point.FloatAttribute(ResidualTag) = dist;
    if (dist > emax) emax = dist;
    sqsum += dist * dist;
    dist = spline_point.Position2DOnly().Distance(point->Position2DOnly());
    if (dist > emax_xy) emax_xy = dist;
    sqsum_xy += dist * dist;
    dist = fabs(spline_point.Z() - point->Z());
    if (dist > emax_z) emax_z = dist;
    sqsum_z += dist * dist;
  	points.push_back(spline_point);
  }
  printf("RMS all %.4f, RMS xy %.4f, RMS z %.4f\n",
         sqrt(sqsum/(double) num_points),
         sqrt(sqsum_xy/(double) num_points),
         sqrt(sqsum_z/(double) num_points));
  printf("max err all %.4f, max err xy %.4f, max err z %.4f\n",
         emax, emax_xy, emax_z);

  points.Write(spline_file, 0, false);
  x_points.Write("x.laser", 0, false);
  
  //
  //
  // Test to check reconstruction with derivatives
  //
  //
  
  // Initialise uniform or non-uniform spline
  if (min_knot_distance == 0.0) // Uniform splines
    x_spline2.Initialise(order, time_start, time_end, knot_interval_size);
  else
    x_spline2.Initialise(order, knots);
  
  // Set observations for end locations and first derivatives
  point = points.begin();
  time = point->DoubleAttribute(TimeTag);
  x_spline2.AddObservation(time, point->X());
  x_spline2.AddConstraint(time, x_spline.Derivative(time, 1), 1, 1.0);
  point = points.end() - 1;
  time = point->DoubleAttribute(TimeTag);
  x_spline2.AddObservation(time, point->X());
  x_spline2.AddConstraint(time, x_spline.Derivative(time, 1), 1, 1.0);
  
  // Only set derivatives for all times
  for (point=points.begin(); point!=points.end(); point++) {
    time = point->DoubleAttribute(TimeTag);
    // First derivative
    // x_spline2.AddConstraint(time, x_spline.Derivative(time, 1), 1, 1.0);
    // Second derivative
    x_spline2.AddConstraint(time, x_spline.Derivative(time, 2), 2, 1.0);
  }
  
  // Adjust spline coefficients
  x_spline2.FitSpline();
  
  // Output of "X"-points
  x_points.ErasePoints();
  sqsum = 0.0;
  for (ip=0; ip<num_points; ip++) {
  	point = points.begin() + ip; 
    time = point->DoubleAttribute(TimeTag);
  	// Store X coordinates as function of time
  	x_point.X() = time;
  	x_point.Y() = point->X();
  	x_point.SetAttribute(LabelTag, point->Label());
  	x_points.push_back(x_point);
  	x_point.Y() = x_spline2.Value(time);
  	x_point.SetAttribute(LabelTag, 3);
  	x_point.FloatAttribute(ResidualTag) = fabs(x_point.Y() - point->X());
  	x_points.push_back(x_point);
  	dist = x_spline2.Value(time) - x_spline.Value(time);
  	sqsum += dist * dist;
  }
  printf("RMS spline dif %.4f\n", sqrt(sqsum / (double) num_points));
  x_points.Write("x2.laser", 0, false);
}
