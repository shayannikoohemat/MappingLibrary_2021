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
#include "LaserScanningSystem.h"
#include "Timer.h"
#include "IMUUnit.h"

void EstimatePoseAndPlanes(int interval, LaserBlock &block, bool read_all_parts,
                           BSplineFit &omega_spline,
                           BSplineFit &phi_spline, BSplineFit &kappa_spline,
						   BSplineFit &X_spline, BSplineFit &Y_spline,
						   BSplineFit &Z_spline, Planes &planes,
						   double start_time, double end_time, 
						   bool fix_all_planes,
						   bool full_progress_report, bool print_stdevs,
						   double &sigma0,
						   double weight_offset_constraint, 
						   double weight_curvature_constraint,
						   LaserScanningSystem *scanner_system,
						   bool only_5_regpar, char *self_calibration_output_file,
						   bool only_2D, double scan_line_duration,
						   IMUUnit & imu_datasets,
						   IMUReadings::iterator imu_obs_begin,
						   IMUReadings::iterator imu_obs_end)
{
  LaserBlock::iterator     strip;
  LaserUnit::iterator      part;
  LaserPoints::iterator    point;
  int                      i, j, iteration, max_iteration=4, num_splines,
                           num_plane_par, *plane_index, next_index, order,
						   par_index, spline_index, first_spline_index,
						   num_unknowns, num_obs, fixed_wall=1, sensor,
						   num_fixed_planes, fixed_horizontal_plane=-1,
						   num_obsolete_planes, registration_index,
						   num_reg_par, num_pose_par, max1, max2,
						   num_cal_par, calibration_index, partNr;
  double                   *a_row, obs, omega, phi, kappa,
                           pi=4.0*atan(1.0), degree, coef[6], *x,
						   time, bspline_value, bspline_2derivative, bspline_1derivative, alpha, *spline_coef[6],
						   sqsum_obs, sqsum_obs_start, cond, sqsum_obs_previous,
						   reg_par[12], cal_par[8], *increments, max_cor,
						   original_range, updated_range,
						   original_angle, updated_angle,
						   original_range_offset[3], original_range_scale[3],
						   original_angle_scale[3], updated_range_offset[3],
						   updated_range_scale[3], updated_angle_scale[3];
  Vector3D                 trans_frame_world, original_trans_sensor_frame[3],
                           updated_trans_sensor_frame[3], sensor_point,
                           rotated_frame_point, normal, frame_point, beam,
                           IMU_Acc, SLAM_Acc, rot_m_IMU_Acc, Acc_obs,
                           Coef_AcX, Coef_AcY, Coef_AcZ, coefAcc, gravity,
                           IMU_AngV, SLAM_AngV, rot_s_SLAM_AngV, AngV_obs,
                           Coef_AngV_omega1, Coef_AngV_phi1, Coef_AngV_kappa1, coefAngV1,
                           Coef_AngV_omega2, Coef_AngV_phi2, Coef_AngV_kappa2, coefAngV2;
  Rotation3D               rot_frame_world, original_rot_frame_sensor[3],
                           updated_rot_sensor_frame[3], rot_beam_sensor,
                           rot_imu_frame, rot_frame_imu, rot_world_frame;
  Planes::iterator         plane;
  bool                     converged, fix_plane_rotation=false,
                           fix_frame_pose=false, debug=false,
						   fix_one_horizontal_plane=false,
						   horizontal_plane_fixed=false,
						   remove_outliers=false, delete_points, delete_IMU_ObsS,
						   print_plane_indices=false, print_spline_coef=false,
						   stabilise_equations=false, sparse_ata;
  Timer                    timer;
  NormalEquations          neq, neqata;
  IMUUnit::iterator        IMU_part;
  IMUReadings::iterator    ObsIMU, first_imu_obs, last_imu_obs;
  Matrix3                  S, S_world_sensor, S_deriv_kappa, S_deriv_phi;



  // debug
  bool first_obs_reported, report_counts=false;
  bool use_cal=true;
  bool use_reg=true;
  int  first_interval, num_counts, interval_index;
  int  interval_obs_count[610], interval_point_count[610], plane_count[50];
  // report_counts = (end_time - start_time > 10.0);
  if (report_counts) printf("\nInterval %d\n\n", interval);

  // Only stabilise equations of local estimations (fewer than 10 intervals)
  stabilise_equations = ((end_time - start_time) / scan_line_duration) < 10.0;
  
  // Only use sparse band normal matrix for the larger adjustments with
  // higher order splines
  sparse_ata = (omega_spline.Order() > 2);
  
  FILE *fd_perf, *fd_acc;
  if (interval == 1 && fix_all_planes) {
  	printf("Opening performance.txt\n");
  	fd_perf = fopen("performance.txt", "w");
  	fd_acc  = fopen("accuracies.txt", "w");
  }
  else {
  	fd_perf = fopen("performance.txt", "a");
  	fd_acc  = fopen("accuracies.txt", "a");
  }

  // Determine the number of unknowns
  num_splines = omega_spline.NumberOfSplines();
  if (only_2D) num_pose_par = 3;
  else num_pose_par = 6;
  plane_index = (int *) malloc((planes.size()+1) * sizeof(int));
  if (!plane_index) {
    printf("Error alloating plane_index vector for %d planes\n", planes.size());
    exit(0);
  }
  if (fix_frame_pose) next_index = 0;
  else next_index = num_pose_par * num_splines;
  if (!fix_all_planes) {
    num_fixed_planes = num_plane_par = num_obsolete_planes = 0;
    for (i=0, plane=planes.begin(); plane!=planes.end(); i++, plane++) {
      // Some planes may be fixed. Other planes are no longer used, because
      // they are merged with other planes. In the latter case the plane number
      // no longer corresponds to the index in the planes vector.
      if (plane->Attribute(PT_Fixed) || plane->Number() != i) {
      	plane_index[i] = next_index; // May be used as end of previous plane index
      	if (plane->Attribute(PT_Fixed)) num_fixed_planes++;
      	else num_obsolete_planes++;
      	continue;
      }
  	  plane_index[i] = next_index;
  	  next_index++;
  	  num_plane_par++;
  	  if (i == fixed_wall) { // One wall is used to fix the kappa rotation.
	    if (print_plane_indices) printf("Plane %d index %d distance (fixed wall)\n", i, plane_index[i]);
	  }
	  else if (fabs(plane->Normal().Z()) > 0.99) {
	  	if (only_2D) {
	  	  if (print_plane_indices) printf("Plane %d is horizontal and not used in 2D estimation\n", i);
	  	  next_index--; num_plane_par--;
	  	}
	  	else {
	  	  if (fix_one_horizontal_plane && !horizontal_plane_fixed) {
	  	    if (print_plane_indices) printf("Plane %d is used as horizontal reference\n", i);
	  	    next_index--; num_plane_par--;
	  	    horizontal_plane_fixed = true;
	  	    fixed_horizontal_plane = i;
	  	  }
	  	  else {
	        if (print_plane_indices) printf("Plane %d index %d distance (horizontal)\n", i, plane_index[i]);
	  	  }
	    }
	  }
	  else {
	    if (print_plane_indices) printf("Plane %d index %d distance (vertical)\n", i, plane_index[i]);
	  }
	  if (fabs(plane->Normal().Z()) < 0.99 && i != fixed_wall && !fix_plane_rotation) {
	    next_index++;
	    num_plane_par++;
	    if (print_plane_indices) printf("Plane %d index %d direction (vertical)\n", i, plane_index[i]+1);
	  }
	}
  }
  else {
  	num_fixed_planes = planes.size();
  }
  plane_index[planes.size()] = next_index;
  if (scanner_system && !only_2D) { // Add self calibration parameters (only for 3D)
    registration_index = next_index;
    if (only_5_regpar) num_reg_par = 10;
    else num_reg_par = 12;
    next_index += num_reg_par;
    if (!use_reg) {
      next_index -= num_reg_par;
      num_reg_par = 0;
    }
    calibration_index = next_index;
    num_cal_par = 8;
    next_index += num_cal_par;
    if (!use_cal) {
      next_index -= num_cal_par;
      num_cal_par = 0;
    }
  }
  else {
    num_reg_par = 0;
    num_cal_par = 0;
  }
  num_unknowns = next_index;
  if (full_progress_report) {
    if (fix_frame_pose)
      printf("No frame pose parameters, only %d parameters of %d planes",
             num_plane_par, planes.size()-num_obsolete_planes);
    else if (fix_all_planes)
      printf("No plane parameters, only %d splines x 6 pose parameters",
	         num_splines);
    else
      printf("%d splines x %d pose parameters + %d parameters of (%d/%d) planes",
             num_splines, num_pose_par, num_plane_par, 
			 planes.size()-num_fixed_planes-num_obsolete_planes,
			 planes.size());
	if (num_reg_par || num_cal_par)
	  printf(" + %d calibration and %d registration parameters\n",
	         num_cal_par, num_reg_par);
    else
      printf("\n");
    printf("Total number of unknowns: %d\n", num_unknowns);
  }
  
  // Allocate normal equation system
  a_row = (double *) malloc(num_unknowns * sizeof(double));
  if (!a_row) {printf("Error allocating a_row\n"); exit(0);}
  if (sparse_ata)
    i = neq.Initialise(num_unknowns, num_pose_par * num_splines, 4 * num_pose_par,
	                   true);
  else 
    i = neq.Initialise(num_unknowns);
  if (i != 0) {printf("Error allocating normal equation system\n"); exit(0);}

  // Collect pointers to spline coefficients for more efficient parameter updating code
  if (only_2D) {
    spline_coef[0] = kappa_spline.SplineCoefficients();
    spline_coef[1] = X_spline.SplineCoefficients();
    spline_coef[2] = Y_spline.SplineCoefficients();
  }
  else {
    spline_coef[0] = omega_spline.SplineCoefficients();
    spline_coef[1] = phi_spline.SplineCoefficients();
    spline_coef[2] = kappa_spline.SplineCoefficients();
    spline_coef[3] = X_spline.SplineCoefficients();
    spline_coef[4] = Y_spline.SplineCoefficients();
    spline_coef[5] = Z_spline.SplineCoefficients();
  }
  
  // Set current sensor calibration parameter values as approximate values
  if (num_cal_par) {
  	original_angle_scale[0]  = cal_par[0] =
	  (*scanner_system)[0].Calibration()->AngleScale();
  	original_range_offset[0] = cal_par[1] =
	  (*scanner_system)[0].Calibration()->RangeOffset();
  	original_range_scale[0]  = 1.0; // NB no self calibration to avoid singularity
  	original_angle_scale[1]  = cal_par[2] =
	  (*scanner_system)[1].Calibration()->AngleScale();
  	original_range_offset[1] = cal_par[3] =
	  (*scanner_system)[1].Calibration()->RangeOffset();
  	original_range_scale[1]  = cal_par[4] =
	  (*scanner_system)[1].Calibration()->RangeScale();
  	original_angle_scale[2]  = cal_par[5] =
	  (*scanner_system)[2].Calibration()->AngleScale();
  	original_range_offset[2] = cal_par[6] =
	  (*scanner_system)[2].Calibration()->RangeOffset();
  	original_range_scale[2]  = cal_par[7] =
	  (*scanner_system)[2].Calibration()->RangeScale();
  }
  
  // Set current sensor registration parameter values as approximate values
  if (num_reg_par || num_cal_par) {
  	// Sensor 1 w.r.t. sensor 0
  	(*scanner_system)[1].Orientation()->rotation().DeriveAngles(omega, phi, kappa);
  	reg_par[0]  = omega;
  	reg_par[1]  = phi;
  	reg_par[2]  = kappa;
  	reg_par[3]  = (*scanner_system)[1].Orientation()->vect().X();
  	reg_par[4]  = (*scanner_system)[1].Orientation()->vect().Y();
  	if (only_5_regpar) next_index = 5;
  	else {
  	  reg_par[5]  = (*scanner_system)[1].Orientation()->vect().Z();
  	  next_index = 6;
  	}
  	original_rot_frame_sensor[1] = (*scanner_system)[1].Orientation()->rotation().Transpose();
  	original_trans_sensor_frame[1] = (*scanner_system)[1].Orientation()->vect();
  	// Sensor 2 w.r.t. sensor 0
  	(*scanner_system)[2].Orientation()->rotation().DeriveAngles(omega, phi, kappa);
  	reg_par[next_index]  = omega; next_index++;
  	reg_par[next_index]  = phi; next_index++;
  	reg_par[next_index]  = kappa; next_index++;
  	reg_par[next_index]  = (*scanner_system)[2].Orientation()->vect().X();
  	next_index++;
  	reg_par[next_index]  = (*scanner_system)[2].Orientation()->vect().Y();
  	if (!only_5_regpar) {
  	  next_index++;
  	  reg_par[next_index] = (*scanner_system)[2].Orientation()->vect().Z();
  	}
  	original_rot_frame_sensor[2] = (*scanner_system)[2].Orientation()->rotation().Transpose();
  	original_trans_sensor_frame[2] = (*scanner_system)[2].Orientation()->vect();
  }
  
  // Iterative pose estimation
      degree = pi / 180.0;
      iteration = 0;
      order = omega_spline.Order();
      converged = false;
      do {
          iteration++;

          // Reset normal equation system
          num_obs = 0;  sqsum_obs = 0.0;
          neq.ClearEquations();

          // Loop over all sensors
          first_obs_reported = false;
          for (sensor=0, strip=block.begin(); sensor<3; sensor++, strip++) {

              // Copy updated calibration parameters
              if (num_cal_par) {
                  updated_angle_scale[0]  = cal_par[0];
                  updated_range_offset[0] = cal_par[1];
                  updated_range_scale[0]  = 1.0; // No range scale calibration for sensor 0
                  updated_angle_scale[1]  = cal_par[2];
                  updated_range_offset[1] = cal_par[3];
                  updated_range_scale[1]  = cal_par[4];
                  updated_angle_scale[2]  = cal_par[5];
                  updated_range_offset[2] = cal_par[6];
                  updated_range_scale[2]  = cal_par[7];
              }

              // Compose approximate transformation from sensor to frame c.s.
              // Only needed for sensors 1 and 2 in case of registration
              // Also needed for the rotation matrices in case of self calibration
              if ((sensor > 0 && num_reg_par) || num_cal_par) {
                  if (sensor == 1) par_index = 0;
                  else if (only_5_regpar) par_index = 5;
                  else par_index = 6;
                  updated_rot_sensor_frame[sensor] =
                          Rotation3D(reg_par[par_index], reg_par[par_index+1],
                                     reg_par[par_index+2]);
                  if (only_5_regpar) // Keep original sensor height
                      updated_trans_sensor_frame[sensor] =
                              Vector3D(reg_par[par_index+3], reg_par[par_index+4],
                                       original_trans_sensor_frame[sensor].Z());
                  else // Estimate sensor heights
                      updated_trans_sensor_frame[sensor] =
                              Vector3D(reg_par[par_index+3], reg_par[par_index+4],
                                       reg_par[par_index+5]);
              }

              // Loop over all datasets for this sensor
              for (part=strip->begin(); part!=strip->end(); part++) {

                  // Check whether data should be read (and deleted after processing)
                  if (read_all_parts && part->size() == 0) {
                      // Check whether this part overlaps with the time window
                      if (part->StartTime() > end_time) continue;
                      if (part->EndTime() < start_time) continue;
                      // Read the points of this part
                      if (!part->Read()) {
                          printf("Error reading points from %s\n", part->PointFile());
                          exit(0);
                      }
                      delete_points = true;
                  }
                  else delete_points = false;

                  // Loop over all points
                  for (point=part->begin(); point!=part->end(); point++) {

                      // Extract and check time interval
                      time = point->DoubleAttribute(TimeTag);
                      if (time < start_time) continue;
                      if (time > end_time) continue;

                      if (report_counts) {
                          if (!first_obs_reported) {
                              first_obs_reported = true;
                              first_interval = (int) ((time - start_time) / scan_line_duration);
                              for (j=0; j<610; j++) interval_obs_count[j] = 0;
                              for (j=0; j<610; j++) interval_point_count[j] = 0;
                              for (j=0; j<50; j++) plane_count[j] = 0;
                          }
                          interval_index = (int) ((time - start_time) / scan_line_duration) - first_interval;
                          interval_point_count[interval_index]++;
                      }

                      // Obtain plane data
                      if (!point->HasAttribute(PlaneNumberTag)) continue;
                      plane = planes.begin() + point->Attribute(PlaneNumberTag);

                      // In case a plane was merged, the plane number of the point needs
                      // updating and the parameters of the merged plane should be used.
                      if (plane->Number() != point->Attribute(PlaneNumberTag)) {
                          point->Attribute(PlaneNumberTag) = plane->Number();
                          plane = planes.begin() + plane->Number();
                      }
                      normal = plane->Normal();

                      // Ignore horizontal planes in the 2D estimation
                      if (only_2D && fabs(normal.Z()) > 0.99) continue;


                      // Reconstruct the point in the sensor c.s. if registration or
                      // calibration is needed and determine frame point with improved
                      // registration parameters
                      if ((sensor > 0 && num_reg_par) || num_cal_par) {
                          sensor_point = original_rot_frame_sensor[sensor] * (point->vect() -
                                                                              original_trans_sensor_frame[sensor]);
                          // Reconstruct original range and angle if calibration is needed and
                          // apply updated calibration parameters
                          if (num_cal_par) {
                              original_range  = (sensor_point.Length() - original_range_offset[sensor]) /
                                                original_range_scale[sensor];
                              original_angle  = atan2(sensor_point.Y(), sensor_point.X()) /
                                                original_angle_scale[sensor];
                              updated_range   = original_range * updated_range_scale[sensor] +
                                                updated_range_offset[sensor];
                              updated_angle   = original_angle * updated_angle_scale[sensor];
                              rot_beam_sensor = Rotation3D(0.0, 0.0, updated_angle);
                              beam            = Vector3D(updated_range, 0.0, 0.0);
                              sensor_point    = rot_beam_sensor * beam;

                          }
                          if (sensor > 0 && num_reg_par) // Use updated registration
                              frame_point = updated_rot_sensor_frame[sensor] * sensor_point +
                                            updated_trans_sensor_frame[sensor];
                          else if (sensor == 0) // Nothing to transform for sensor 0
                              frame_point = sensor_point;
                          else // Use original registration
                              frame_point = (*scanner_system)[sensor].Orientation()->rotation() *
                                            sensor_point +
                                            (*scanner_system)[sensor].Orientation()->vect();
                      }
                      else // No self calibration or registration
                          frame_point = point->vect();

                      // Get the approximate transformation from frame to world c.s.
                      rot_frame_world = Rotation3D(omega_spline.Value(time), phi_spline.Value(time),
                                                   kappa_spline.Value(time));
                      trans_frame_world = Vector3D(X_spline.Value(time), Y_spline.Value(time),
                                                   Z_spline.Value(time));

                      // Frame point rotated to approximate world c.s.
                      rotated_frame_point = rot_frame_world * frame_point;

                      // Construct observation (signed distance of point to plane) and store
                      // it as residual
                      obs = plane->Distance() -
                            normal.DotProduct(rotated_frame_point + trans_frame_world);
                      point->FloatAttribute(ResidualTag) = obs;

                      // debug
                      if (report_counts) {
                          interval_obs_count[interval_index]++;
                          plane_count[point->Attribute(PlaneNumberTag)]++;
                      }

                      // Check if this should be considered an outlier
                      if (remove_outliers && iteration > 1 && fabs(obs) > 0.06) {
                          point->RemoveAttribute(PlaneNumberTag);
                          point->RemoveAttribute(ResidualTag);
                          continue;
                      }

                      // Update observation statistics
                      num_obs++;
                      sqsum_obs += obs * obs;

                      // Initialise row of A matrix
                      memset((void *) a_row, 0, num_unknowns * sizeof(double));

                      // Construct row of A matrix - spline coefficients
                      if (!fix_frame_pose) {
                          // Determine the first basic spline used at the time of this point
                          first_spline_index = omega_spline.FirstBSplineIndex(time);
                          if (only_2D) {
                              // Coefficients of kappa, X, and Y spline increments
                              coef[0] = normal.DotProduct(rot_frame_world.PartialDeriv(2) * frame_point);
                              coef[1] = normal.X();
                              coef[2] = normal.Y();
                          }
                          else { // 3D
                              // Coefficients of omega, phi, and kappa spline increments
                              coef[0] = normal.DotProduct(rot_frame_world.PartialDeriv(0) * frame_point);
                              coef[1] = normal.DotProduct(rot_frame_world.PartialDeriv(1) * frame_point);
                              coef[2] = normal.DotProduct(rot_frame_world.PartialDeriv(2) * frame_point);
                              // Coefficients of X, Y, Z spline increments
                              coef[3] = normal.X();
                              coef[4] = normal.Y();
                              coef[5] = normal.Z();
                          }
                          // Loop over all basic splines used at the time of the current point
                          for (i=0, spline_index=first_spline_index; i<order; i++, spline_index++) {
                              if (spline_index < num_splines) {
                                  // Get uniform B-spline value (from an arbitrary spline instantiation)
                                  bspline_value = omega_spline.BSplineValue(time, spline_index);
                                  // Set the coefficients of the design matrix for the pose
                                  // spline coefficients
                                  for (j=0, par_index = num_pose_par * spline_index;
                                       j<num_pose_par; j++, par_index++) {
                                      if (par_index >= num_unknowns) {
                                          printf("par_index %d, num_unknowns %d\n", par_index, num_unknowns);
                                          exit(0);
                                      }
                                      a_row[par_index] = coef[j] * bspline_value;
                                  }
                              }
                          }
                      }

                      // Construct row of A matrix - plane coefficients
                      if (!fix_all_planes && !plane->Attribute(PT_Fixed)) {
                          // distance coefficient
                          if (point->Attribute(PlaneNumberTag) != fixed_horizontal_plane) {
                              if (plane_index[point->Attribute(PlaneNumberTag)] >= num_unknowns) {
                                  printf("plane_index[point->Attribute(PlaneNumberTag)] %d, num_unknowns %d\n", plane_index[point->Attribute(PlaneNumberTag)], num_unknowns);
                                  exit(0);
                              }
                              a_row[plane_index[point->Attribute(PlaneNumberTag)]] = -1.0;
                          }
                          // angle coefficient for vertical planes, but not for one wall (to fix kappa rotation)
                          if (point->Attribute(PlaneNumberTag) != fixed_wall &&
                              fabs(normal.Z()) < 0.99 && !fix_plane_rotation) {
                              if (plane_index[point->Attribute(PlaneNumberTag)]+1 >= num_unknowns) {
                                  printf("plane_index[point->Attribute(PlaneNumberTag)]+1 %d, num_unknowns %d\n", plane_index[point->Attribute(PlaneNumberTag)]+1, num_unknowns);
                                  printf("plane %d\n", point->Attribute(PlaneNumberTag));
                                  exit(0);
                              }
                              a_row[plane_index[point->Attribute(PlaneNumberTag)] + 1] =
                                      Vector3D(-normal.Y(), normal.X(), 0.0).
                                              DotProduct(rotated_frame_point + trans_frame_world);
                          }
                      }

                      // Construct row of A matrix - calibration coefficients
                      if (num_cal_par) {
                          par_index = calibration_index;
                          if (sensor == 1) par_index += 2;
                          else if (sensor == 2) par_index += 5;
                          // Coefficient of angle scale
                          a_row[par_index] = normal.DotProduct(rot_frame_world *
                                                               updated_rot_sensor_frame[sensor] *
                                                               rot_beam_sensor.PartialDeriv(2) * beam) * original_angle;
                          // Coefficient of range offset
                          a_row[par_index+1] = normal.DotProduct(rot_frame_world *
                                                                 updated_rot_sensor_frame[sensor] *
                                                                 rot_beam_sensor * Vector3D(1.0, 0.0, 0.0));
                          // Coefficient of range scale
                          if (sensor > 0)
                              a_row[par_index+2] = a_row[par_index+1] * original_range;
                      }

                      // Construct row of A matrix - registration coefficients
                      if (sensor > 0 && num_reg_par > 0) {
                          if (sensor == 1) par_index = registration_index;
                          else if (only_5_regpar) par_index = registration_index + 5; // Sensor 2
                          else par_index = registration_index + 6;
                          // Coefficients of omega, phi, and kappa rotations from sensor to frame c.s.
                          for (i=0; i<3; i++)
                              a_row[par_index+i] = normal.DotProduct(rot_frame_world *
                                                                     updated_rot_sensor_frame[sensor].PartialDeriv(i) * sensor_point);
                          // Coefficients of X, Y, Z translations from sensor to frame c.s.
                          a_row[par_index+3] = normal.DotProduct(rot_frame_world * Vector3D(1.0, 0.0, 0.0));
                          a_row[par_index+4] = normal.DotProduct(rot_frame_world * Vector3D(0.0, 1.0, 0.0));
                          if (!only_5_regpar)
                              a_row[par_index+5] = normal.DotProduct(rot_frame_world * Vector3D(0.0, 0.0, 1.0));
                      }

                      // Add row to normal equation system
                      if (fix_all_planes || fix_frame_pose) {
                          neq.AddObservation(a_row, obs, 1.0);
                      }
                      else {
                          // No selfcalibration or registration parameters
                          if ((sensor == 0 || num_reg_par == 0) && num_cal_par == 0) {
                              if (plane->Attribute(PT_Fixed)) {
                                  // Update only spline part
                                  neq.AddObservation(a_row, obs, 1.0,
                                                     num_pose_par*first_spline_index,
                                                     num_pose_par*first_spline_index+order*num_pose_par-1);
                              }
                              else {
                                  // Update spline part and plane part
                                  neq.AddObservation(a_row, obs, 1.0,
                                                     num_pose_par*first_spline_index,
                                                     num_pose_par*first_spline_index+order*num_pose_par-1,
                                                     plane_index[point->Attribute(PlaneNumberTag)],
                                                     plane_index[point->Attribute(PlaneNumberTag)+1]-1);
                              }
                          }
                          else { // Include selfcalibration parameters
                              if (plane->Attribute(PT_Fixed)) {
                                  // Update only spline part and calibration/registration part
                                  neq.AddObservation(a_row, obs, 1.0,
                                                     num_pose_par*first_spline_index,
                                                     num_pose_par*first_spline_index+order*num_pose_par-1,
                                                     registration_index,
                                                     registration_index+num_reg_par+num_cal_par-1);
                              }
                              else {
                                  // Update spline part, plane part and calibration/registration part
                                  neq.AddObservation(a_row, obs, 1.0,
                                                     num_pose_par*first_spline_index,
                                                     num_pose_par*first_spline_index+order*num_pose_par-1,
                                                     plane_index[point->Attribute(PlaneNumberTag)],
                                                     plane_index[point->Attribute(PlaneNumberTag)+1]-1,
                                                     registration_index,
                                                     registration_index+num_reg_par+num_cal_par-1);
                              }
                          }
                      }

                      if ((num_obs/10000)*10000 == num_obs && full_progress_report)
                          printf("%8d\r", num_obs);
                  }

                  if (delete_points) part->ErasePoints();
              }
          }
    if (full_progress_report) {
      printf("%8d\n", num_obs);
      printf("stdev %9.6f\n", sqrt(sqsum_obs / num_obs));
    }
        
	if (report_counts && iteration == 1) {
      for (j=0; j<610; j++) {
        if (interval_point_count[j])
          printf("interval %3d  num obs %5d  num points %5d\n",
		       j+first_interval,
		       interval_obs_count[j], interval_point_count[j]);
      }
      printf("planes ");
      for (j=0; j<50; j++)
        if (plane_count[j]) printf("%2d %5d   ", j, plane_count[j]);
      printf("\n");
      printf("%8d\n", num_obs);
      printf("stdev %9.6f\n", sqrt(sqsum_obs / num_obs));
    }

  	// Additional (soft) constraints to fix origin
  	// no constraints needed if planes are fixed
  	if (!fix_frame_pose && !fix_all_planes) {
  	  // Check if there are enough fixed planes
  	  if (num_fixed_planes >= 3 && debug) {
  	  	printf("No constraint on spline start (%d planes are fixed)\n",
			   num_fixed_planes);
  	  }
  	  else {
  	  	if (end_time - start_time > 2.0)
  	      printf("Spline at start %6.3f %6.3f %6.3f\n", X_spline.Value(start_time),
	             Y_spline.Value(start_time), Z_spline.Value(start_time));
        for (j=0; j<3; j++) {
          // Don't fix height if there is a horizontal reference plane fixed
          if (j == 2 && horizontal_plane_fixed) continue;
          // Don't fix height for 2D estimations either
          if (only_2D && j == 2) continue;
          
          memset((void *) a_row, 0, num_unknowns * sizeof(double));
          switch (j) {
      	    case 0: obs = -X_spline.Value(start_time); break; // X observation
      	    case 1: obs = -Y_spline.Value(start_time); break; // Y observation
      	    case 2: obs = -Z_spline.Value(start_time); break; // Z observation
          }
          
  	      for (i=0, spline_index=0; i<order; i++, spline_index++) {
  	        // Get uniform B-spline value (from an arbitrary spline instantiation)
  	        bspline_value = omega_spline.BSplineValue(start_time, spline_index);
  	        if (only_2D) par_index = spline_index * num_pose_par + j + 1;
  	        else par_index = spline_index * num_pose_par + j + 3;
  	        a_row[par_index] = bspline_value;
  	      }
          neq.AddObservation(a_row, obs, weight_offset_constraint,
							 0, num_pose_par * order - 1);
        }
      }
    }
    
    // Soft constraints to avoid high correlation at start and end of
    // cubic splines
// To be analysed. The constraints do not yet lead to lower correlation coefficients
    if (omega_spline.Order() == 4) {
		obs=0.0;
  	  first_spline_index = omega_spline.FirstBSplineIndex(start_time);
      for (j=0; j<num_pose_par; j++) {
        memset((void *) a_row, 0, num_unknowns * sizeof(double));
	    for (i=0, spline_index=first_spline_index;
		     i<order && spline_index < num_splines; i++, spline_index++) {
	      par_index = spline_index * num_pose_par + j;
    	  a_row[par_index] = omega_spline.BSplineDerivative(start_time, spline_index, 2);
	    }
        neq.AddObservation(a_row, obs, weight_curvature_constraint,
						   first_spline_index * num_pose_par + j, par_index);
      }
  	  first_spline_index = omega_spline.FirstBSplineIndex(end_time);
      for (j=0; j<num_pose_par; j++) {
        memset((void *) a_row, 0, num_unknowns * sizeof(double));
	    for (i=0, spline_index=first_spline_index;
		     i<order && spline_index < num_splines; i++, spline_index++) {
	      par_index = spline_index * num_pose_par + j;
    	  a_row[par_index] = omega_spline.BSplineDerivative(end_time, spline_index, 2);
	    }
        neq.AddObservation(a_row, obs, weight_curvature_constraint,
						   first_spline_index * num_pose_par + j, par_index);
      }
    }

     // Start Using IMU data
          if (omega_spline.Order() > 3) {
              printf("START USING IMU DATA \n");
              printf("Using IMU data recorded between start_time (%lf) and end_time (%lf) \n", start_time, end_time);
              printf("Before transformation to model system \n");
              printf("   Time      Acc(X, Y, Z) \n");
              // rotation matrix from sensor (IMU) to frame system
              rot_imu_frame = Rotation3D(0.0, 0.0, -90 * degree);
              // rotation matrix from frame to sensor (IMU) system
              rot_frame_imu=rot_imu_frame.Transpose();
              // gravity vector
              gravity = Vector3D(0.0, 0.0, 9.812519800618945);

               // Loop over all datasets for the IMU sensor
              for (IMU_part = imu_datasets.begin(), partNr = 1;
                   IMU_part != imu_datasets.end(); IMU_part++, partNr++) {
                  //printf("Part Number %d \n", partNr);

                      // Check whether data should be read (and deleted after processing)
                      if (read_all_parts && IMU_part->size() == 0) {
                          // Check whether this part overlaps with the time window
                          if (IMU_part->StartTime() > end_time) continue;
                          if (IMU_part->EndTime() < start_time) continue;
                          // Read the IMU observations of this part
                          if (!IMU_part->Read()) {
                              printf("Error reading IMU observations from %s\n", IMU_part->IMUFile());
                              exit(0);
                          }
                          delete_IMU_ObsS = true;
                      }
                      else delete_IMU_ObsS = false;
                      
                      // Set the first and last observation iterator of this part
                      if (read_all_parts) { // Process all readings
                      	first_imu_obs = IMU_part->begin();
                      	last_imu_obs  = IMU_part->end() - 1;
                      }
                      else { // Only process the specified part for local and section adjustments
                        first_imu_obs = imu_obs_begin;
                        last_imu_obs  = imu_obs_end - 1;
                      }

                //Loop over all IMU observations (Accelerations, Angular Velocities)

                  // checking observations inside a local or section window only
                  for (ObsIMU = first_imu_obs; ObsIMU != last_imu_obs+1; ObsIMU++) {
                      // time of IMU observation
                      time = ObsIMU->Time();

                      if (time < start_time || time > end_time) continue;

                  // ACCELERATIONS: Insert Acceleration equation in SLAM equation system
                      /*printf("IMU Acc: %lf %lf %lf %lf\n", time, ObsIMU->Accelerations().X(),
                             ObsIMU->Accelerations().Y(), ObsIMU->Accelerations().Z());*/
                      // Vector of IMU accelerations at time (time)
                      IMU_Acc = Vector3D(ObsIMU->Accelerations().X(),
                                         ObsIMU->Accelerations().Y(),
                                         ObsIMU->Accelerations().Z());

                      // Rotation matrix from frame to world system
                      rot_frame_world = Rotation3D(omega_spline.Value(time),
                                        phi_spline.Value(time),
                                        kappa_spline.Value(time)); // R_fm=R(ω)R(φ)R(κ) in the word file

                      // IMU accelerations in the model system
                      rot_m_IMU_Acc = rot_frame_world * rot_imu_frame * IMU_Acc - gravity;

                      // SLAM: approximate values of position accelarations
                      SLAM_Acc = Vector3D(X_spline.Derivative(time, 2),
                                          Y_spline.Derivative(time, 2),
                                          Z_spline.Derivative(time, 2));

                      //printf("SLAM-based position X: %lf Y: %lf Z: %lf \n", X_spline.Value(time), Y_spline.Value(time), Z_spline.Value(time));
                     /* printf("SLAM Acc: %lf %lf %lf %lf \n", time, X_spline.Derivative(time, 2),
                             Y_spline.Derivative(time, 2), Z_spline.Derivative(time, 2));*/

                      // 1st IMU observation=The differences between SLAM-based acceleration and IMU accelerations in the model system (Error)
                      Acc_obs = SLAM_Acc - rot_m_IMU_Acc;

                      //printf("Acc Error: %lf %lf %lf lf\n", time, Acc_obs.X(), Acc_obs.Y(), Acc_obs.Z());

                      Coef_AcX = rot_frame_world.PartialDeriv(0) * rot_imu_frame * IMU_Acc;
                      Coef_AcY = rot_frame_world.PartialDeriv(1) * rot_imu_frame * IMU_Acc;
                      Coef_AcZ = rot_frame_world.PartialDeriv(2) * rot_imu_frame * IMU_Acc;

                      for (int ObsRow_index = 0; ObsRow_index < 3; ObsRow_index++) {
                          coefAcc = Vector3D(Coef_AcX[ObsRow_index],
                                             Coef_AcY[ObsRow_index],
                                             Coef_AcZ[ObsRow_index]);

                          first_spline_index = omega_spline.FirstBSplineIndex(time);

                          // Initialise row of A matrix
                          memset((void *) a_row, 0, num_unknowns * sizeof(double));

                          for (i = 0, spline_index = first_spline_index;
                               i < order && spline_index < num_splines; i++, spline_index++) {

                              // B-spline value = Bi(t)
                              bspline_value = omega_spline.BSplineValue(time, spline_index);

                              // B-spline derivatives = B̈i(t)
                              bspline_2derivative = omega_spline.BSplineDerivative(time, spline_index, 2);

                              for (int k = 0, par_index = spline_index * num_pose_par;
                                   k < num_pose_par; k++, par_index++) {

                                  // Construct row of A matrix - spline coefficients
                                  a_row[par_index] = k < 3 ? coefAcc[k] * bspline_value :
                                                     ObsRow_index == (k - 3) ? -bspline_2derivative : 0.0;
                              }
                          }
                          // Add row to the normal equation system and update the spline part
                          neq.AddObservation(a_row, Acc_obs[ObsRow_index], 1.0,
                                             first_spline_index * num_pose_par,
                                             num_pose_par * first_spline_index + order * num_pose_par - 1);
                      }

                  // ANGULAR VELOCITY: Insert Angular Velocity equation in SLAM equation system
                      /*printf("IMU AngV: %lf %lf %lf %lf\n", time, ObsIMU->AngularVelocities().X(),
                             ObsIMU->AngularVelocities().Y(), ObsIMU->AngularVelocities().Z());*/
                      // Vector of IMU angular velocities at time (time)
                      IMU_AngV = Vector3D(ObsIMU->AngularVelocities().X(),
                                          ObsIMU->AngularVelocities().Y(),
                                          ObsIMU->AngularVelocities().Z());

                      // Rotation matrix from world to frame system
                      rot_world_frame = rot_frame_world.Transpose(); // R_mf=R(-κ)R(-φ)R(-ω)

                      // computing the phi and kappa angles from splines at time of IMU observation (time)
                      phi = phi_spline.Value(time);
                      kappa = kappa_spline.Value(time);
                      // Transformation matrix S=a1 + R_kappa_T * a2 + R_kappa_T * R_phi_T * a3;
                      S = Matrix3(cos(phi) * cos(kappa), sin(kappa), 0.0,
                                  -cos(phi) * sin(kappa), cos(kappa), 0.0,
                                  sin(phi), 0.0, 1.0);

                      // SLAM: approximate values of Angular Velocity
                      SLAM_AngV = Vector3D(omega_spline.Derivative(time, 1),
                                           phi_spline.Derivative(time, 1),
                                           kappa_spline.Derivative(time, 1));
                      //printf("SLAM-based Euler Angles Omega: %lf Phi: %lf Kappa: %lf \n", omega_spline.Value(time), phi_spline.Value(time), kappa_spline.Value(time));
                      /*printf("SLAM AngV: %lf %lf %lf %lf \n", time, omega_spline.Derivative(time, 1),
                             phi_spline.Derivative(time, 1), kappa_spline.Derivative(time, 1));*/

                      // SLAM approximate angular velocities in the IMU sensor system
                      S_world_sensor = rot_frame_imu * S; // S_ms
                      rot_s_SLAM_AngV = S_world_sensor * SLAM_AngV;

                      // 2nd IMU observation=The differences between SLAM-based approximate angular velocities and IMU angular velocities in the IMU sensor system (Error)
                      AngV_obs = IMU_AngV - rot_s_SLAM_AngV;

                      //printf("AngV Error: %lf %lf %lf lf\n", time, AngV_obs.X(), AngV_obs.Y(), AngV_obs.Z());

                      // partial derivative of S matrix w.r.t kappa
                      S_deriv_kappa = Matrix3(-cos(phi) * sin(kappa), cos(kappa), 0.0,
                                              -cos(phi) * cos(kappa), -sin(kappa), 0.0,
                                               0.0                  ,  0.0       , 0.0);

                      // partial derivative of S matrix w.r.t phi
                      S_deriv_phi = Matrix3(-sin(phi) * cos(kappa), 0.0, 0.0,
                                            sin(phi) * sin(kappa) , 0.0, 0.0,
                                            cos(phi)              , 0.0, 0.0);

                      Coef_AngV_omega1 = Vector3D();
                      Coef_AngV_phi1 = rot_frame_imu * S_deriv_phi * SLAM_AngV;
                      Coef_AngV_kappa1 = rot_frame_imu * S_deriv_kappa * SLAM_AngV;

                      Coef_AngV_omega2 = S_world_sensor.Column(0);
                      Coef_AngV_phi2 = S_world_sensor.Column(1);
                      Coef_AngV_kappa2 =S_world_sensor.Column(2);

                      for (int ObsRow_index = 0; ObsRow_index < 3; ObsRow_index++) {
                          coefAngV1 = Vector3D(Coef_AngV_omega1[ObsRow_index],
                                               Coef_AngV_phi1[ObsRow_index],
                                               Coef_AngV_kappa1[ObsRow_index]);

                          coefAngV2 = Vector3D(Coef_AngV_omega2[ObsRow_index],
                                               Coef_AngV_phi2[ObsRow_index],
                                               Coef_AngV_kappa2[ObsRow_index]);

                          first_spline_index = omega_spline.FirstBSplineIndex(time);

                          // Initialise row of A matrix
                          memset((void *) a_row, 0, num_unknowns * sizeof(double));

                          for (i = 0, spline_index = first_spline_index;
                               i < order && spline_index < num_splines; i++, spline_index++) {

                              // B-spline value = Bi(t)
                              bspline_value = omega_spline.BSplineValue(time, spline_index);

                              // B-spline derivative = B'i(t)
                              bspline_1derivative = omega_spline.BSplineDerivative(time, spline_index, 1);

                              for (int k = 0, par_index = spline_index * num_pose_par;
                                   k < 3; k++, par_index++) {

                                  // Construct row of A matrix - spline coefficients
                                  a_row[par_index] =
                                          coefAngV2[k] * bspline_1derivative + coefAngV1[k] * bspline_value;
                              }
                          }
                          // Add row to the normal equation system and update the spline part
                          neq.AddObservation(a_row, AngV_obs[ObsRow_index], 1.0,
                                             first_spline_index * num_pose_par,
                                             num_pose_par * first_spline_index + order * num_pose_par - 1);
                      }

                  }

                  if (delete_IMU_ObsS) IMU_part->EraseData();
              }
          }

    // Stabilisation of equation system by adding zero as observation for spline
    // coefficient increments. In case the observations don't allow the estimation
    // of the spline coefficients, the approximate values should be maintainted.
    if (stabilise_equations) {
      memset((void *) a_row, 0, num_unknowns * sizeof(double));
      for (i=0; i<num_pose_par * num_splines; i++) {
      	a_row[i] = 1.0;
        neq.AddObservation(a_row, 0.0, 10.0, i, i);
       	a_row[i] = 0.0;
      }
    }

    // Check structure of ata before Cholesky decomposition
    if (num_unknowns > 300 && num_unknowns < 10000) {
      Image image;
      double max_coef=0.0;
      image.NewImage(num_unknowns, num_unknowns, VFF_TYP_1_BYTE);
      for (i=0; i<num_unknowns; i++) {
        for (j=0; j<num_unknowns; j++) {
      	  if (j <= i) {
            if (neq.ATA_Value(i, j) == 0.0) *(image.Pixel(i,j)) = 255;
     	    else *(image.Pixel(i,j)) = 0;      	  		
          }
          else *(image.Pixel(i,j)) = 255;
        }
      }
      image.Write("Normal_matrix.xv");
    }

	// Estimate increments
	timer.Reset();
    if (!sparse_ata && print_stdevs)
	  increments = neq.InvertAndSolve(cond);
	else
	  increments = neq.Solve();
	if (increments == NULL) {
	  printf("Error solving normal equation system\n");
	  exit(0);
	}
    if (num_unknowns > 300)
	  printf("Time for LS solution %.2f minutes\n", timer.Minutes());

    // Check structure of ata after Choleskey decomposition
    if (num_unknowns > 300 && num_unknowns < 10000) {
      Image image;
      double max_coef=0.0;
      image.NewImage(num_unknowns, num_unknowns, VFF_TYP_1_BYTE);
      for (i=0; i<num_unknowns; i++) {
        for (j=0; j<num_unknowns; j++) {
      	  if (j <= i) {
            if (neq.ATA_Value(i, j) == 0.0) *(image.Pixel(i,j)) = 255;
     	    else *(image.Pixel(i,j)) = 0;      	  		
          }
          else *(image.Pixel(i,j)) = 255;
        }
      }
      image.Write("Cholesky_L_matrix.xv");
    }
    
	// Update spline coefficients
    if (!fix_frame_pose) {
      if (num_splines > 3 && num_splines < 10 && print_spline_coef) printf("Spline coefficients\n");
      for (i=0, par_index=0; i<num_splines; i++) {
	    for (j=0; j<num_pose_par; j++, par_index++) {
	      if (num_splines > 3 && num_splines < 10 && print_spline_coef) {
	      	switch (j) {
	      	  case 0: if (only_2D) printf("K"); else printf("O"); break;
	      	  case 1: if (only_2D) printf("X"); else printf("P"); break;
	      	  case 2: if (only_2D) printf("Y"); else printf("K"); break;
	      	  case 3: printf("X"); break;
	      	  case 4: printf("Y"); break;
	      	  case 5: printf("Z"); break;
	        }
	      	printf(" %8.3f to %8.3f   ", spline_coef[j][i],
				   spline_coef[j][i]+increments[par_index]);
	      }
	  	  spline_coef[j][i] += increments[par_index];
	    }
	    if (num_splines > 3 && num_splines < 10 && print_spline_coef) printf("\n");
	  }
	}
	
	// Update plane parameters
	if (!fix_all_planes) {
	  for (i=0, plane=planes.begin(); plane!=planes.end(); i++, plane++) {
	  	if (plane->Attribute(PT_Fixed) || plane->Number() != i) continue;
	  	if (only_2D && fabs(plane->Normal().Z()) > 0.99) continue;
	  	if (i != fixed_horizontal_plane) {
	      printf("Plane %3d index %3d distance from  %8.3f to %8.3f\n",
	             i, plane_index[i], plane->Distance(),
			     plane->Distance() + increments[plane_index[i]]);
	      plane->Distance() += increments[plane_index[i]];
	  	}
  	    if (i != fixed_wall && fabs(plane->Normal().Z()) < 0.99 && !fix_plane_rotation) {
  	  	  alpha = atan2(plane->Normal().Y(), plane->Normal().X());
	      printf("Plane %3d index %3d direction from %8.3f to %8.3f\n",
	             i, plane_index[i]+1, alpha / degree,
			     (alpha + increments[plane_index[i]+1]) / degree);
  	  	  alpha += increments[plane_index[i] + 1];
  	  	  plane->SetNormal(Vector3D(cos(alpha), sin(alpha), 0.0));
  	    }
  	  }
	}

    // Update calibration parameters
    if (num_cal_par > 0) {
      for (sensor=0, i=0; sensor<3; sensor++) {
      	printf("Calibration of sensor %d angle scale from %8.5f to %8.5f\n",
      	       sensor, cal_par[i], cal_par[i] + increments[calibration_index+i]); 
		cal_par[i] += increments[calibration_index+i]; i++;
      	printf("Calibration of sensor %d range offset from %8.5f to %8.5f\n",
      	       sensor, cal_par[i], cal_par[i] + increments[calibration_index+i]); 
		cal_par[i] += increments[calibration_index+i]; i++;
      	if (sensor == 0) continue;
      	printf("Calibration of sensor %d range scale from %8.5f to %8.5f\n",
      	       sensor, cal_par[i], cal_par[i] + increments[calibration_index+i]); 
		cal_par[i] += increments[calibration_index+i]; i++;
      }
    }
    
    // Update registration parameters
    if (num_reg_par > 0) {
      for (i=0; i<num_reg_par; i++) {
      	if (i<num_reg_par/2) sensor=1; else sensor=2;
      	printf("Registration sensor %d ", sensor);
      	switch (i%(num_reg_par/2)) {
      	  case 0: printf("omega from %8.3f to %8.3f\n",
			reg_par[i] / degree, (reg_par[i] + increments[registration_index+i]) / degree); break;
      	  case 1: printf("phi   from %8.3f to %8.3f\n",
			reg_par[i] / degree, (reg_par[i] + increments[registration_index+i]) / degree); break;
      	  case 2: printf("kappa from %8.3f to %8.3f\n",
			reg_par[i] / degree, (reg_par[i] + increments[registration_index+i]) / degree); break;
      	  case 3: printf("X     from %8.3f to %8.3f\n", reg_par[i],
			             reg_par[i] + increments[registration_index+i]); break;
      	  case 4: printf("Y     from %8.3f to %8.3f\n", reg_par[i],
			             reg_par[i] + increments[registration_index+i]); break;
      	  case 5: printf("Z     from %8.3f to %8.3f\n", reg_par[i],
			             reg_par[i] + increments[registration_index+i]); break;
      	}
        reg_par[i] += increments[registration_index+i];
      }
    }
    
    // Output standard deviations
    if (iteration == 2 && !sparse_ata && print_stdevs) {
      sigma0 = sqrt(sqsum_obs / num_obs);
      fprintf(fd_acc, "%4d", interval);
      for (i=0; i<num_pose_par; i++)
        if (i<num_pose_par/2)
          fprintf(fd_acc, "%6.3f", sigma0 * sqrt(neq.ATA_Value(i, i)) / degree);
        else
		  fprintf(fd_acc, "%6.3f", sigma0 * sqrt(neq.ATA_Value(i, i)));
      for (i=num_pose_par; i<2*num_pose_par; i++)
        if (i<num_pose_par + num_pose_par/2)
          fprintf(fd_acc, "%6.3f", sigma0 * sqrt(neq.ATA_Value(i, i)) / degree);
        else
		  fprintf(fd_acc, "%6.3f", sigma0 * sqrt(neq.ATA_Value(i, i)));
      fprintf(fd_acc, "\n");

      // Convert variances to correlation coefficients
	  neq.ConvertToCorrelations();

      max_cor = 0.0;
      for (i=0; i<num_unknowns; i++) {
        for (j=0; j<num_unknowns; j++) {
          if (i == j) continue;
          if (fabs(neq.ATA_Value(i, j)) > max_cor) {
            max_cor = fabs(neq.ATA_Value(i, j));
            max1 = i;
            max2 = j;
          }
        }
      }
      fprintf(fd_perf, "interval %4d  #p1 %6d  stdev %7.4f  cond %.5e  cor %6.4f (%2d, %2d)\n",
              interval, planes[1].Attribute(PT_NumberOfPoints), sigma0, cond, max_cor, max1, max2);
    }
    else if (iteration == 2) {
      fprintf(fd_perf, "interval %4d  #p1 %6d  stdev %7.4f\n",
	          interval, planes[1].Attribute(PT_NumberOfPoints), sigma0);
    }
    
	// Check on convergence
	if (iteration == 1) sqsum_obs_start = sqsum_obs;
    else if (sqrt(sqsum_obs/sqsum_obs_previous) > 0.999) converged = true;
	sqsum_obs_previous = sqsum_obs;
	
	if (num_reg_par > 0 && iteration < 8) converged = false; // caltest
	
  } while (iteration < max_iteration && !converged);
  sigma0 = sqrt(sqsum_obs / num_obs);
  if (sigma0 > 0.05 && debug)
    printf("stdev initial %8.4f final %8.4f, %d iterations\n", 
           sqrt(sqsum_obs_start / num_obs), sigma0, iteration);
  
  // Store improved calibration
  if ((num_cal_par || num_reg_par) && self_calibration_output_file) {
  	if (num_reg_par) {
  	  updated_rot_sensor_frame[1] = 
	    Rotation3D(reg_par[0], reg_par[1], reg_par[2]);
	  if (only_5_regpar) {
	    updated_trans_sensor_frame[1] = 
	      Vector3D(reg_par[3], reg_par[4], original_trans_sensor_frame[1].Z());
	    updated_rot_sensor_frame[2] =
	      Rotation3D(reg_par[5], reg_par[6], reg_par[7]);
	    updated_trans_sensor_frame[2] = 
	      Vector3D(reg_par[8], reg_par[9], original_trans_sensor_frame[2].Z());
	  }
	  else {
	    updated_trans_sensor_frame[1] =
	      Vector3D(reg_par[3], reg_par[4], reg_par[5]);
	    updated_rot_sensor_frame[2] =
	      Rotation3D(reg_par[6], reg_par[7], reg_par[8]);
	    updated_trans_sensor_frame[2] = 
	      Vector3D(reg_par[9], reg_par[10], reg_par[11]);
	  }
    }
	LaserScanningSystem new_system = *scanner_system;
	for (sensor=0, i=0; sensor<3; sensor++) {
	  if (num_cal_par) {
	  	new_system[sensor].Calibration()->AngleScale() = cal_par[i]; i++;
	  	new_system[sensor].Calibration()->RangeOffset() = cal_par[i]; i++;
	  	if (sensor > 0) {
	  	  new_system[sensor].Calibration()->RangeScale() = cal_par[i]; i++;
	    }
	  	new_system[sensor].Calibration()->SetDateNow();
	  }
	  if (num_reg_par && sensor > 0) {
	    new_system[sensor].Orientation()->rotation() = updated_rot_sensor_frame[sensor];
	    new_system[sensor].Orientation()->vect() = updated_trans_sensor_frame[sensor];
	    new_system[sensor].Orientation()->SetDateNow();
	  }
	}
	new_system.Write(self_calibration_output_file);
  }

  // Close files
  fclose(fd_perf);
  fclose(fd_acc);
  
  // Release memory
  free(a_row);
  neq.FreeMemory();
}

bool NeedToAdjustSection(int current_interval, double interval_duration,
                         Planes &planes, double start_time_first_interval, 
						 int num_intervals_per_section,
						 double &section_start_time, double &section_end_time,
						 bool only_2D)
{
  bool             adjust=false;
  int              index, interval_of_new_plane;
  Planes::iterator plane;
  
  // Determine if a plane has been introduced num_intervals_per_section ago
  interval_of_new_plane = current_interval - num_intervals_per_section + 1;
  for (plane=planes.begin(), index=0; plane!=planes.end() && !adjust;
       plane++, index++) {
  	if (plane->Number() != index) continue; // Plane no longer used
  	if (only_2D && fabs(plane->Normal().Z() > 0.99)) continue; // Ignore horizontal planes for 2D estimation
  	if (plane->Attribute(PT_FirstScanLine) == interval_of_new_plane)
	  adjust = true;
  }
  
  // Set whether planes should kept fixed or not and determine 
  // the time interval of the section
  if (adjust) {
    for (plane=planes.begin(), index=0; plane!=planes.end(); plane++, index++) {
  	  if (plane->Number() != index) continue; // Plane no longer used
  	  if (only_2D && fabs(plane->Normal().Z() > 0.99)) continue; // Ignore horizontal planes for 2D estimation
  	  if (plane->Attribute(PT_FirstScanLine) >= interval_of_new_plane) {
  	    plane->Attribute(PT_Fixed) = 0;
  	    printf("Plane %d instantiated in interval %d should be adjusted\n",
		       plane->Number(), plane->Attribute(PT_FirstScanLine));
  	  }
  	  else
  	    plane->Attribute(PT_Fixed) = 1;
  	}
  	section_start_time = start_time_first_interval +
	                     interval_of_new_plane * interval_duration;
  	section_end_time   = section_start_time + num_intervals_per_section *
	                     interval_duration;
  }

  return adjust;
}

