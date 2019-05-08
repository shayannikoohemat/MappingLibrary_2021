
/*
                      Copyright 2017 University of Twente 
 
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
#include "LineSegments2D.h"
#include "Lines2D.h"
#include "Rotation2D.h"
#include "normal_equations.h"


/*
--------------------------------------------------------------------------------
                        Determine approximate scan pose
--------------------------------------------------------------------------------
*/

void DetermineApproximateScanPose(LaserPoints &scan, LineSegments2D &ref_walls,
                                  Position2D &ref_midpoint, double *x_approx)
{
  Lines2D                  scan_walls;
  LineSegments2D::iterator ref_wall;
  LaserPoints              wall_points;
  LaserPoints::iterator    point;
  Position2D               scan_midpoint;
  Line3D                   line;
  Line2D                   line2d;
  int                      num, longest_wall, wall_index; 
  double                   sqlen, sqlenmax=-1.0, pi=4.0*atan(1.0), angle; 
  DataBoundsLaser          wall_bounds;
  Rotation2D               rot;
  Vector2D                 offset;
  
  // Determine midpoint of scan
  scan_midpoint = scan.DeriveDataBounds(0).MidPoint().Position2DOnly();
  
  // Determine wall lines by fitting to laser points
  for (ref_wall=ref_walls.begin(), wall_index=0; ref_wall!=ref_walls.end(); 
       ref_wall++, wall_index++) {
	num = wall_points.AddTaggedPoints(scan, ref_wall->Number(), PlaneNumberTag);
  	if (num == 0) {
  	  printf("Error: no points on wall %d in scan %d\n", ref_wall->Number(),
		     scan.begin()->Attribute(ScanLineNumberTag));
	  exit(0);
  	}
  	point = wall_points.begin();
  	for (point=wall_points.begin(); point!=wall_points.end(); point++)
  	  line.AddPoint(point->Position3DRef(), false);
  	line.Recalculate();
  	line2d = line.ProjectOntoXOYPlane();
  	if (line2d.DistanceToPointSigned(scan_midpoint) < 0) line2d.SwapNormal();
  	scan_walls.push_back(line2d);
  	// Check if this is the longest wall
  	wall_bounds = wall_points.DeriveDataBounds(0);
  	sqlen = wall_bounds.XRange() * wall_bounds.XRange() +
	        wall_bounds.YRange() * wall_bounds.YRange();
  	if (sqlen > sqlenmax) {
  	  sqlenmax = sqlen;
  	  longest_wall = wall_index;
  	}
  	// Clear data of this set of wall points
  	wall_points.ErasePoints();
  	line.Erase();
  }
  
  // Use longest wall to determine rotation angle
  angle = ref_walls[longest_wall].NormalAngle() -
          scan_walls[longest_wall].NormalAngle();
  if (angle < 0.0) angle += 2.0 * pi;
  
  // Determine offset
  rot = Rotation2D(angle);  
  offset = ref_midpoint - rot * scan_midpoint;
  
  // Set approximate value vector
  x_approx[0] = angle;
  x_approx[1] = offset.X();
  x_approx[2] = offset.Y();
}

/*
--------------------------------------------------------------------------------
                           Observation equation
--------------------------------------------------------------------------------
*/

void PointOnLine(int scan_index, const LaserPoint &scan_point, 
                 const LineSegment2D &ref_wall, double *ata, double *aty, 
				 double *x_approx, int num_par, int num_cal_par,
				 int &num_obs, double &sqsum_obs)
{
  double     y, *a, pi=4.0*atan(1.0), angle_beam, range, range_corrected,
             angle_beam_corrected;
  int        i, num_pose_par=3;
  Rotation2D rot_scan, rot_scan_deriv, rot_beam, rot_beam_deriv, rotrot;
  Vector2D   offset, beam;
  
  // Allocate memory for coefficients
  a = (double *) calloc(num_par, sizeof(double));
  
  // Common expressions
  rot_scan        = Rotation2D(x_approx[num_cal_par+scan_index*num_pose_par]);
  rot_scan_deriv  = rot_scan.PartialDeriv();
  offset          = Vector2D(x_approx[num_cal_par+scan_index*num_pose_par+1],
                             x_approx[num_cal_par+scan_index*num_pose_par+2]);
  range           = scan_point.vect().Length();
  range_corrected = range;
  if (num_cal_par > 0) range_corrected *= x_approx[0]; // Range scale
  if (num_cal_par > 1) range_corrected += x_approx[1]; // Range offset
  angle_beam      = atan2(scan_point.Y(), scan_point.X());
  angle_beam_corrected = angle_beam;
  if (num_cal_par > 2) angle_beam_corrected *= x_approx[2]; // Angle scale
  rot_beam        = Rotation2D(angle_beam_corrected);
  rot_beam_deriv  = rot_beam.PartialDeriv();
  rotrot          = rot_scan * rot_beam;
  beam            = Vector2D(range_corrected, 0.0);

  // Observation	
  y = (rotrot * beam + offset).DotProduct(ref_wall.Normal()) -
      ref_wall.DistanceToOrigin();
  num_obs++;
  sqsum_obs += y * y;
  
  // Partial derivatives for calibration parameters
  switch (num_cal_par) {
  	case 3: a[2] = angle_beam * (rot_scan *  rot_beam_deriv * beam).
	               DotProduct(ref_wall.Normal());
  	case 2: a[1] = rotrot.Column(0).DotProduct(ref_wall.Normal());
  	case 1: a[0] = range * (rotrot.Column(0).DotProduct(ref_wall.Normal()));
  	default: break;
  }
  
  // Partial derivatives for scan pose parameters
  a[num_cal_par+scan_index*num_pose_par]     = 
    (rot_scan_deriv * rot_beam * beam).DotProduct(ref_wall.Normal());
  a[num_cal_par+scan_index*num_pose_par + 1] = ref_wall.Normal().X();
  a[num_cal_par+scan_index*num_pose_par + 2] = ref_wall.Normal().Y();
  
  // Update equation system
  Update_Normal_Eq(a, y, 1.0, ata, aty, num_par);  
  free(a);
}

/*
--------------------------------------------------------------------------------
                           Main calibration function
--------------------------------------------------------------------------------
*/
void indoorcalibration_cpp(vector<LaserPoints> &scans, LineSegments2D &ref_walls,
                           int id, char *filename, int num_cal_par)
{
  vector<LaserPoints>::iterator scan;
  LineSegments2D::iterator      ref_wall;
  double                        degree = atan(1.0) / 45.0, *x_approx, *ata, *aty,
                                xsum, ysum, sqsum_obs, cond, std_obs;
  int                           num_par, i, j, num_pose_par=3, num_iter,
                                max_iter=10, num_obs;
  Position2D                    ref_midpoint;
  LaserPoints                   ref_points, wall_points;
  LaserPoints::iterator         scan_point;
  LaserPoint                    ref_point;
  Rotation3D                    rot;
  Vector3D                      offset;
  bool                          converged;
  LaserScanner                  scanner;
  
  // Allocate memory
  num_par = num_cal_par + num_pose_par * scans.size();
  x_approx = (double *) malloc(num_par * sizeof(double));
  ata      = (double *) malloc(num_par * num_par * sizeof(double));
  aty      = (double *) malloc(num_par * sizeof(double));
  
  // Set all reference wall normals pointing to room centre
  xsum = ysum = 0.0;  // First determine approximate room centre
  for (ref_wall=ref_walls.begin(); ref_wall!=ref_walls.end(); ref_wall++) {
  	xsum += ref_wall->MiddlePoint().X();
  	ysum += ref_wall->MiddlePoint().Y();
  }
  ref_midpoint = Position2D(xsum / ref_walls.size(), ysum / ref_walls.size());
  for (ref_wall=ref_walls.begin(); ref_wall!=ref_walls.end(); ref_wall++) {
  	if (ref_wall->DistanceToPointSigned(ref_midpoint) < 0)
  	  *ref_wall = LineSegment2D(ref_wall->EndPoint(), ref_wall->BeginPoint()); // Swap direction
  }
  
  // Determine approximate parameters
  if (num_cal_par > 0) x_approx[0] = 1.0;   // Range scale
  if (num_cal_par > 1) x_approx[1] = 0.0;   // Range offset
  if (num_cal_par > 2) x_approx[2] = 1.0;   // Angle scale
  for (scan=scans.begin(), i=0; scan!=scans.end(); scan++, i++) {
  	DetermineApproximateScanPose(*scan, ref_walls, ref_midpoint,
	                             x_approx+num_cal_par+i*3);

    /*/ Testing: transform scans to reference system
    rot = Rotation3D(0.0, 0.0, x_approx[num_cal_par+i*num_pose_par]);
    offset = Vector3D(x_approx[num_cal_par+i*num_pose_par+1],
	                  x_approx[num_cal_par+i*num_pose_par+2], 0.0);
    for (scan_point = scan->begin(); scan_point != scan->end(); scan_point++) {
      ref_point.vect() = (rot * scan_point->Position3DRef() + offset).vect();
      if (scan_point->HasAttribute(PlaneNumberTag))
	    ref_point.Attribute(PlaneNumberTag) = scan_point->Attribute(PlaneNumberTag);
	  else
	    ref_point.RemoveAttribute(PlaneNumberTag);
      ref_points.push_back(ref_point);
    }
    */
  }
  
//  ref_points.Write("approximate_points.laser", false);
  
  // Iterative least squares estimation
  converged = false; num_iter = 0;
  while (!converged && num_iter < max_iter) {
    num_iter++;
  	
    // Initialise equation system
    memset((void *) ata, 0, num_par * num_par * sizeof(double));
    memset((void *) aty, 0, num_par * sizeof(double));
    num_obs = 0;
    sqsum_obs = 0.0;
  
    // Loop over all scans
    for (scan=scans.begin(), i=0; scan!=scans.end(); scan++, i++) {
      
      // Loop over all walls (not efficient, but fast enough for small datasets)
      for (ref_wall=ref_walls.begin(); ref_wall!=ref_walls.end(); ref_wall++) {

     	// Get all points on a reference wall
		wall_points.AddTaggedPoints(*scan, ref_wall->Number(), PlaneNumberTag);
		
		// Add observations to equation system
		for (scan_point=wall_points.begin(); scan_point!=wall_points.end();
		     scan_point++) {
		  PointOnLine(i, *scan_point, *ref_wall, ata, aty, x_approx, num_par,
		              num_cal_par, num_obs, sqsum_obs);
		}

        wall_points.ErasePoints();
      }
    }

/*
    if (num_iter == 1) {
      printf("ATA\n");
      for (i=0; i<num_par; i++) {
        for (j=0; j<num_par; j++) printf("%8.1f ", ata[i*num_par+j]);
        printf("\n");
      }
    }
*/
    
	// Solve equation system and check estimability of parameters
    Invert_And_Solve_Normal_Eq(ata, aty, num_par, &cond);
    printf("Iter %2d  stdev obs  %.4f      Condition number %.4e\n",
	       num_iter, sqrt(sqsum_obs/num_obs), cond);
    
    // Update parameters
    for (i=0; i<num_par; i++) x_approx[i] -= aty[i];

    // Print increments, parameters, correlations
    printf("               Estimate  Increment\n");
    if (num_cal_par > 0)
      printf("Range scale    %8.5f   %8.5f\n", x_approx[0], aty[0]);
    if (num_cal_par > 1)
      printf("Range offset   %8.5f   %8.5f\n", x_approx[1], aty[1]);
    if (num_cal_par > 2)
      printf("Rotation scale %8.5f   %8.5f\n", x_approx[2], aty[2]);
    for (scan=scans.begin(), i=0; scan!=scans.end(); scan++, i++) {
      printf("Scan %d\n", scan->begin()->Attribute(ScanLineNumberTag));
      printf("Scan rotation  %8.3f   %8.3f\n",
	         x_approx[num_cal_par+i*num_pose_par] / degree,
	         aty[num_cal_par+i*num_pose_par] / degree);
      printf("Scan pos X     %8.3f   %8.3f\n",
	         x_approx[num_cal_par+i*num_pose_par+1],
	         aty[num_cal_par+i*num_pose_par+1]);
      printf("Scan pos Y     %8.3f   %8.3f\n",
	         x_approx[num_cal_par+i*num_pose_par+2],
	         aty[num_cal_par+i*num_pose_par+2]);
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
    if (num_cal_par > 0)
      if (fabs(aty[0]) > 0.00001) converged = false;
    if (num_cal_par > 1)
      if (fabs(aty[1]) > 0.00001) converged = false;
    if (num_cal_par > 2)
      if (fabs(aty[2]) > 0.00001) converged = false;
    for (scan=scans.begin(), i=0; scan!=scans.end() && converged; scan++, i++) {
      if (fabs(aty[num_cal_par+i*num_pose_par]) > 0.00001 * degree) converged = false;
      if (fabs(aty[num_cal_par+i*num_pose_par+1]) > 0.00001) converged = false;
      if (fabs(aty[num_cal_par+i*num_pose_par+2]) > 0.00001) converged = false;
    }
  }

  // Output of accuracies
  if (converged) {
    printf("Convergence in %d iterations\n", num_iter);
    printf("Standard deviations of estimated parameters:\n");
    std_obs = sqrt(sqsum_obs / num_obs);
    if (num_cal_par > 0)
      printf("Range scale    %8.5f\n", std_obs * sqrt(ata[0]));
    if (num_cal_par > 1)
      printf("Range offset   %8.5f\n", std_obs * sqrt(ata[1+num_par]));
    if (num_cal_par > 2)
      printf("Rotation scale %8.5f\n", std_obs * sqrt(ata[2+2*num_par]));
    for (scan=scans.begin(), i=0; scan!=scans.end(); scan++, i++) {
      printf("Scan %d\n", scan->begin()->Attribute(ScanLineNumberTag));
      j = num_cal_par + i * num_pose_par;
      printf("Scan rotation  %8.3f\n", std_obs * sqrt(ata[j+j*num_par]) / degree); j++;
      printf("Scan pos X     %8.3f\n", std_obs * sqrt(ata[j+j*num_par])); j++;
      printf("Scan pos Y     %8.3f\n", std_obs * sqrt(ata[j+j*num_par])); 
    }
  }
  
  // Output of calibration parameters
  scanner.ScannerID() = id;
  scanner.InitialiseCalibrationData();
  scanner.Calibration()->SetDateNow();
  if (num_cal_par > 0) scanner.Calibration()->RangeScale()  = x_approx[0];
  if (num_cal_par > 1) scanner.Calibration()->RangeOffset() = x_approx[1];
  if (num_cal_par > 2) scanner.Calibration()->AngleScale()  = x_approx[2];
  scanner.Write(filename);
}
