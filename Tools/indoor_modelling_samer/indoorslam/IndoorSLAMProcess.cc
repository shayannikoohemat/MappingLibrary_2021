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
#include "LaserScanningSystem.h"
#include "LaserScanLines.h"
#include "BSplineFit.h"
#include "Image.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                      The main indoorslam_cpp function
--------------------------------------------------------------------------------
*/

void IndoorSLAMProcess(char *block_file,
                       char *sensor0_file, char *sensor1_file, char *sensor2_file,
                       char *appendix_point_cloud_files, char *trajectory_file,
					   char *reconstruction_dir,
					   char *interval_dir, int max_scans_for_loop_closure,
					   char *plane_file, double weight_offset_constraint,
					   double weight_curvature_constraint,
					   bool use_old_labels,
					   double max_dist_segment_to_plane,
					   char *system_configuration_file, bool only_5_regpar,
					   char *self_calibration_output_file,
					   double user_start_time, double user_end_time,
					   bool only_2D, bool use_ICP,
					   int num_intervals_local_max,
                       int num_intervals_per_section)
{
  void CollectSegmentsInInterval(int, const LaserBlock &,  int,
	                             LaserPoints::iterator **start_point,
							     LaserScanLines *segments);
  void EstablishWorldCoordinateSystem(LaserBlock &, int, LaserScanLines *,
                                      Planes &, Rotation3D &, Vector3D &,
									  vector<DataBounds3D> &, bool, bool);
  void SetReferencePlaneParameters(Planes &planes);
  bool AssignSegmentsToPlanes(LaserPoints &, int, const LaserScanLines &, 
							  Planes &, bool, BSplineFit &, BSplineFit &,
                              BSplineFit &, BSplineFit &, BSplineFit &,
							  BSplineFit &, vector<DataBounds3D> &, int, bool,
							  bool, double);
  void EstimatePoseAndPlanes(int, LaserBlock &, bool, BSplineFit &, BSplineFit &,
                             BSplineFit &, BSplineFit &, BSplineFit &,
							 BSplineFit &, Planes &, double, double, bool, bool,
							 bool, double &, double, double,
							 LaserScanningSystem *, bool, char *, bool, double);
  void SaveTransformedPointSet(int, double, double, const LaserBlock &, 
                               BSplineFit &, BSplineFit &, BSplineFit &,
							   BSplineFit &, BSplineFit &, BSplineFit &, 
							   char *, char *, Planes &);
  void SaveTransformedBlock(double, double, LaserBlock &,
                            BSplineFit &, BSplineFit &, BSplineFit &,
							BSplineFit &, BSplineFit &, BSplineFit &,
						    char *, char *, double);
  void SaveWalls(int, Planes &, vector<DataBounds3D> &, char *);
  void FitPoseSplines(int, double, double, int, int, int, bool, BSplineFit &,
                      BSplineFit &, BSplineFit &, BSplineFit &, BSplineFit &,
				      BSplineFit &, const Rotation3D *, const Vector3D *);
  void ExtractScanLinePoses(int, double, int, double, const BSplineFit &,
                            const BSplineFit &, const BSplineFit &,
                            const BSplineFit &, const BSplineFit &,
                            const BSplineFit &, Rotation3D *, Vector3D *);
  bool NeedToAdjustSection(int, double, Planes &, double, int, double &,
                           double &, bool);
  void SetIntervalStarts(LaserBlock &, int, int, double, double, double, int, int &,
                         LaserPoints::iterator **);
  void LoadNewSensorData(LaserBlock &, int &, int, double, double, double, int,
                         int &, int &, LaserPoints::iterator **);
  bool IntervalICP(LaserPoints &, int, double &, double &, double &);

  LaserBlock            block, local_block;
  LaserBlock::iterator  strip;
  LaserUnit::iterator   sensordata;
  LaserPoints           copied_points;
  LaserPoints::iterator scan0_point, point,
                        start_point, end_point, *start_point_interval[3];
  int                   i, scan0, num_scans, num_planes, order_local,
                        order_section, order_global,
			            num_pts_per_scan, sensor,
						num_intervals_per_bspline_global,
			            num_intervals_per_bspline_in_section,
						num_intervals, interval,
						first_available_interval, last_available_interval,
						current_part, start_part, num_assignment_in_loop,
						num_intervals_local_current;
  Planes                planes;
  Planes::iterator      plane;
  vector<DataBounds3D>  bounds_of_planes;
  Rotation3D            *rotations;
  Vector3D              *translations;
  LaserScanLines        segments[3];
  LaserPoint            frame_position;
  double                omega, phi, kappa, pi = 4.0 * atan(1.0),
                        degree = pi / 180.0,
                        trajectory_start_time, trajectory_end_time, scan_time,
						time, sigma0, previous_X, previous_Y, X, Y,
						previous_omega, previous_phi, previous_kappa,
						section_start_time, section_end_time, part_start_time,
						local_start_time, local_end_time, scan_line_duration,
						dX, dY, dkappa;
  BSplineFit            local_omega_spline, local_phi_spline, local_kappa_spline,
                        local_X_spline, local_Y_spline, local_Z_spline,
						omega_spline, phi_spline, kappa_spline,
						X_spline, Y_spline, Z_spline,
						fallback_omega_spline, fallback_phi_spline,
						fallback_kappa_spline,
                        fallback_X_spline, fallback_Y_spline, fallback_Z_spline;
  char                  interval_name[20], *point_file_name, part_name[100];
  bool                  found, debug=false, output_approximate=true, 
                        output_approximate_point_cloud=false,
                        output_initial=false, new_assignment;
  FILE                  *par_fd, *slam_ICP_fd;
  LaserScanningSystem   scanner_system, *scanner_system_ptr;

  
  // This programme reconstructs the pose parameters of the frame with three
  // laser scanners as well as the parameters of recorded planes using a SLAM
  // approach. The trajectory of the frame as well as the rotations are modelled
  // with splines. Three different spline estimations are used in this programme:
  //   	1) A local estimation using the data of only a few (2-4) scan lines of
  //       each of the laser scanners. This estimation serves to determine the
  //   	   next piece of the trajectory. The time interval of this small trajectory
  //   	   piece is bound by the variables local_start_time and local_end_time.
  //   	   The splines only model a linear change of the pose parameters over
  //	   time. The names of the spline variables all start with local_. The
  //	   plane parameters are not updated in these local estimations.
  //   	2) An estimation over a section of num_intervals_per_section scan lines. 
  //   	   This estimation serves to improve the parameters of planes that were
  //   	   instantiated a number of scan lines ago (variable
  //	   num_intervals_per_section). Ideally, one would like to update the
  //	   complete trajectory and all plane parameters in every SLAM iteration.
  //	   However, this is time consuming. If a long line segment is not
  //	   associated to a plane during the local estimation mentioned above, a
  //	   new plane is instantiated, but not immediately updated in the next
  //	   iterations. Only after num_intervals_per_section, all information
  //	   gathered in the past num_intervals_per_section scan lines is used
  //	   to obtain more accurate plane parameters. The time interval
  // 	   of this trajectory section is bound by the variables 
  //	   section_start_time and section_end_time. After this estimation the 
  //	   parameters of these planes are kept fixed until the final estimation.
  // 	3) A final estimation of spline functions over the complete trajectory
  // 	   as well as the parameters of all detected planes. The whole
  //	   trajectory is bound by the variables trajectory_start_time and
  //	   trajectory_end_time. The same spline variables are used for the last
  //	   two estimations. The splines are typically cubic splines (order 4,
  //	   degree 3).
  //  Because the three laser scanners operate at slightly different speeds we 
  //  do not exactly select the data per scan line for processing. Instead we
  //  select data based on the average time interval of a single scan line. The
  //  first estimate uses data of two intervals, the second data of 
  //  num_intervals_per_section intervals, and the last one data of all intervals.
  //  The SLAM estimation proceeds with one interval per iteration.
  
  // When needed adjust a section of a couple of scan line intervals 
  // with new planes
  
  // Parameters defining the spline order and spline knot intervals
  order_local = 2;
  order_section = 4;
  order_global = 4;
  num_intervals_per_bspline_in_section = 5;
  num_intervals_per_bspline_global = 5;

  // Read or compose the block meta data
  if (block_file) {
  	if (!block.ReadMetaData(block_file)) {
  	  printf("Error reading block meta data from file %s\n", block_file);
  	  exit(0);
  	}
  	if (block.size() != 3) {
  	  printf("Error: expecting data of 3 sensors, but found %d.\n", block.size());
  	  exit(0);
  	}
  }
  else { // Construct a block and put the three file names in the meta data
    // Create three strips
    for (sensor=0; sensor<3; sensor++) block.push_back(LaserUnit());
    // Put an empty point set in each strip
    for (strip=block.begin(); strip!=block.end(); strip++) strip->push_back(LaserSubUnit());
    // Set the laser point file names of the point sets
    block[0].begin()->SetPointFile(sensor0_file);
    block[1].begin()->SetPointFile(sensor1_file);
    block[2].begin()->SetPointFile(sensor2_file);
  }
  
  // Read the scanner system configuration
  if (system_configuration_file) {
  	if (!scanner_system.Read(system_configuration_file)) {
  	  printf("Error reading scanner system configuration file %s\n",
		     system_configuration_file);
	  exit(0);
  	}
  	scanner_system_ptr = &scanner_system;
  }
  else scanner_system_ptr = NULL;

  // Read the first and last point set of every sensor
  // It is assumed that the measurements of all scanners are already transformed
  // to the frame coordinate system. scanlinesegmentation, that needs to be run
  // before, also assumes this and needs the points in the frame coordinate
  // system to determine the point labels (ceiling, floor, wall). 
  
  for (strip=block.begin(), sensor=0; sensor<3; strip++, sensor++) {
  	
	// Read first point set of this sensor
  	if (!strip->begin()->Read()) {
  	  printf("Error reading points from file %s\n", strip->begin()->PointFile());
  	  exit(0);
  	}
  	
	// Add sensor number as point attribute
    strip->begin()->SetAttribute(ScanNumberTag, sensor);
    
    // Remove plane numbers and residuals that may have been stored in a previous run
    strip->begin()->RemoveAttribute(PlaneNumberTag);
    strip->begin()->RemoveAttribute(ResidualTag);
    
	// Read last point set of this sensor (if there are multiple parts)
  	if (strip->size() > 1) {
  	  if (!(strip->end()-1)->Read()) {
  	    printf("Error reading points from file %s\n", (strip->end()-1)->PointFile());
  	    exit(0);
  	  }
  	}
  	
	// Determine common period of sensor data
  	if (sensor == 0) { // Set start and end time for the first sensor
  	  trajectory_start_time = strip->begin()->begin()->DoubleAttribute(TimeTag);
  	  trajectory_end_time = ((strip->end()-1)->end()-1)->DoubleAttribute(TimeTag);
  	}
  	else { // Update overlap of start and end time for further sensors
  	  if (strip->begin()->begin()->DoubleAttribute(TimeTag) > trajectory_start_time)
        trajectory_start_time = strip->begin()->begin()->DoubleAttribute(TimeTag);
      if (((strip->end()-1)->end()-1)->DoubleAttribute(TimeTag) < trajectory_end_time)
        trajectory_end_time = ((strip->end()-1)->end()-1)->DoubleAttribute(TimeTag);
  	}
  	
  	// Delete data of the last point set if there are multiple parts
  	if (strip->size() > 1) (strip->end()-1)->ErasePoints();
  }
  printf("Data available from %.5f to %.5f seconds\n",
         trajectory_start_time, trajectory_end_time);
  
  // Estimate the average duration of a scan line recording
  sensordata = block.begin()->begin();
  if (!sensordata->begin()->HasAttribute(ScanLineNumberTag)) {
  	printf("Error: data doesn't contain scan line numbers.\n");
  	exit(0);
  }
  // Scan line number of first point
  scan0 = sensordata->begin()->Attribute(ScanLineNumberTag);
  // Search first point of next scan line
  for (scan0_point=sensordata->begin()+1, found=false;
	   scan0_point!=sensordata->end() && !found; scan0_point++) {
	if (scan0_point->Attribute(ScanLineNumberTag) == scan0+1) {
	  found = true;
	  start_point = scan0_point;
	}
  }
  // Search first point of last scan line
  scan0 = (sensordata->end()-1)->Attribute(ScanLineNumberTag);
  for (scan0_point=sensordata->end()-2, found=false;
	   scan0_point!=sensordata->begin() && !found; scan0_point--) {
	if (scan0_point->Attribute(ScanLineNumberTag) == scan0-1) {
	  found = true;
	  end_point = scan0_point;
	}
  }
  num_scans = end_point->Attribute(ScanLineNumberTag) -
              start_point->Attribute(ScanLineNumberTag) + 1;
  num_pts_per_scan = (std::distance(sensordata->begin(), end_point) -
	                  std::distance(sensordata->begin(), start_point) + 1) /
					 num_scans;
  printf("Determined %d points per scan line\n", num_pts_per_scan);
  scan_line_duration = (end_point->DoubleAttribute(TimeTag) -
	                    start_point->DoubleAttribute(TimeTag)) / num_scans;
  printf("Scan line duration %.5f\n", scan_line_duration);

  // Check if the user wants to process a specific time interval
  if (user_start_time > 0.0) {
    if (user_start_time < trajectory_end_time)
      trajectory_start_time = user_start_time;
    else {
      printf("Error: user set start time %.2f is past the end of the data (%.2f)\n",
	         user_start_time, trajectory_end_time);
	  exit(0);
    }
    if (user_start_time >= user_end_time && user_end_time > 0) {
  	  printf("Error: user set end time %.5f not after user set start time %.5f\n");
  	  exit(0);
    }
  }
  if (user_end_time > 0.0) {
  	if (user_end_time > trajectory_start_time && 
	    user_end_time < trajectory_end_time)
	  trajectory_end_time = user_end_time;
	else {
	  printf("Error: user set end time %.2f is outside the trajectory time interval (%.2f-%.2f)\n",
	         user_end_time, trajectory_start_time, trajectory_end_time);
	  exit(0);
	}
  }
  
  // Determine the number of time intervals with the duration of a single scan
  // that the three sensors have in common. 
  num_intervals = (int) ((trajectory_end_time - trajectory_start_time) /
                  scan_line_duration);
  printf("Trajectory from %.5f to %.5f seconds\n", trajectory_start_time,
         trajectory_end_time);
  printf("The trajectory will be reconstructed in %d intervals\n",
         num_intervals);
         
  // Determine the part of the trajectory start time
  for (sensordata=block[0].begin(), i=0, found=false;
       sensordata!=block[0].end() && !found; sensordata++, i++) {
    if (trajectory_start_time >= sensordata->StartTime() &&
	    trajectory_start_time <= sensordata->EndTime()) {
      start_part = i;
	  found = true;
    }
  }
  printf("Trajectory starts in data part %d\n", start_part);
    
  // Load new data if the start part is not 0
  if (start_part > 0) {
    for (strip=block.begin(), sensor=0; sensor<3; strip++, sensor++) {
  	  // Erase data of part 0
      strip->begin()->ErasePoints();  	
      
	  // Read data of the start part
	  sensordata = strip->begin() + start_part;
	  if (!sensordata->Read()) {
	    printf("Error reading sensor data from file %s\n",
	           sensordata->PointFile());
	    exit(0);
  	  }

	  // Add sensor number as point attribute
      sensordata->SetAttribute(ScanNumberTag, sensor);
    
      // Remove plane numbers and residuals
      sensordata->RemoveAttribute(PlaneNumberTag);
      sensordata->RemoveAttribute(ResidualTag);
    }
  }

  // Determine the first point of each interval for each sensor
  first_available_interval = 0;
  current_part = start_part;
  SetIntervalStarts(block, current_part, start_part, trajectory_start_time, 
                    trajectory_end_time, scan_line_duration,
                    first_available_interval, last_available_interval,
                    start_point_interval);

  // Collect all segments of the first scan line interval
  interval = 0;
  CollectSegmentsInInterval(interval-first_available_interval, block, 
                            current_part, start_point_interval, segments);

  // Allocate vectors for storing locally estimated rotations and translations 
  rotations = (Rotation3D *) malloc(num_intervals * sizeof(Rotation3D));
  if (!rotations) {printf("Error allocating rotations vector\n"); exit(0);}
  translations = (Vector3D *) malloc(num_intervals * sizeof(Vector3D));
  if (!translations) {printf("Error allocating translations vector\n"); exit(0);}

  // Define world coordinate system based on ceiling and wall scan lines
  // in the first scan line interval
  EstablishWorldCoordinateSystem(block, start_part, segments, planes,
                                 rotations[0], translations[0],
								 bounds_of_planes, only_2D, use_old_labels);
								 
  // Set the time range for the first two scan line intervals
  num_intervals_local_current = 2;
  local_start_time = trajectory_start_time;
  local_end_time   = trajectory_start_time +
                     num_intervals_local_current * scan_line_duration;

  // Define constant splines for the first two scan line intervals, based on the
  // definition of the established world coordinate system
  // Because we don't know anything about the motion of the scanner frame
  // we simply assume as approximation that there is no motion at all. Hence,
  // the rotation and translation defined by EstablishWorldCoordinateSystem are
  // used to predict the orientation and translation over the whole time
  // interval of the first two scans.
  rotations[0].DeriveAngles(omega, phi, kappa);
  if (only_2D) {
    local_omega_spline.SetLinear(local_start_time, local_end_time, 0.0, 0.0);
    local_phi_spline.SetLinear(local_start_time, local_end_time, 0.0, 0.0);
  }
  else {
    local_omega_spline.SetLinear(local_start_time, local_end_time, omega, omega);
    local_phi_spline.SetLinear(local_start_time, local_end_time, phi, phi);
  }
  local_kappa_spline.SetLinear(local_start_time, local_end_time, kappa, kappa);
  local_X_spline.SetLinear(local_start_time, local_end_time,
                           translations[0].X(), translations[0].X());
  local_Y_spline.SetLinear(local_start_time, local_end_time,
                           translations[0].Y(), translations[0].Y());
  local_Z_spline.SetLinear(local_start_time, local_end_time,
                           translations[0].Z(), translations[0].Z());
  if (debug)
	printf("First approximate pose %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n",
	       omega / degree, phi / degree, kappa / degree,
		   translations[0].X(), translations[0].Y(), translations[0].Z());

  // Determine all planes of the first scans in the world coordinate system
  // Collect all further planes in this coordinate system based on line segments
  // All segments are now assigned to planes. The first three planes have 
  // already been determined in EstablishWorldCoordinateSystem. All segments not 
  // yet used there are used now to generate further planes.
  for (sensor=0; sensor<3; sensor++)
    AssignSegmentsToPlanes(block[sensor][current_part], 0, 
	                       segments[sensor], planes, true,
				           local_omega_spline, local_phi_spline, local_kappa_spline, 
	                       local_X_spline, local_Y_spline, local_Z_spline,
						   bounds_of_planes, max_scans_for_loop_closure, only_2D,
						   use_old_labels, max_dist_segment_to_plane);

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
  SetReferencePlaneParameters(planes);
  
  // Output of the planes found so far
  i=0;
  for (Planes::iterator plane_itr=planes.begin(); plane_itr!=planes.end();
       i++, plane_itr++)
    printf("Plane %d, normal %6.3f %6.3f %6.3f dist %7.3f\n",
	       i, plane_itr->Normal().X(), plane_itr->Normal().Y(),
		   plane_itr->Normal().Z(), plane_itr->Distance());  

  // Set up a local block structure to store the points of two scan line
  // intervals for local processing.
  for (sensor=0; sensor<3; sensor++) {
  	local_block.push_back(LaserUnit());
  	local_block[sensor].push_back(LaserSubUnit());
  }

  // Loop over all further intervals
  if (output_approximate) par_fd = fopen("approx_par.txt", "w");
  if (use_ICP) {
    slam_ICP_fd = fopen("slam_vs_ICP.txt", "w");
    fprintf(slam_ICP_fd, "Interval, extrapolation, ICP, and SLAM result (X, Y, kappa)\n");
    fclose(slam_ICP_fd);
  }
  printf("Scan pose estimation\n");
  num_assignment_in_loop = 0;
  for (interval=1; interval<num_intervals; interval++) {

    // Load further data parts if needed
    if (interval > last_available_interval) {
      LoadNewSensorData(block, current_part, start_part, trajectory_start_time, 
                        trajectory_end_time, scan_line_duration,
                        num_intervals_per_section,
                        first_available_interval, last_available_interval,
                        start_point_interval);
    }

  	// Collect segments of this scan line interval
    CollectSegmentsInInterval(interval-first_available_interval, block, 
                              current_part, start_point_interval, segments);

    // Set approximate spline values by linearly extrapolating the current
    // local spline. If interval = 1, we use the constant splines defined above
    if (interval > 1) {
      local_omega_spline.Extrapolate(local_start_time, local_end_time);
      local_phi_spline.Extrapolate(local_start_time, local_end_time);
      local_kappa_spline.Extrapolate(local_start_time, local_end_time);
      local_X_spline.Extrapolate(local_start_time, local_end_time);
      local_Y_spline.Extrapolate(local_start_time, local_end_time);
      local_Z_spline.Extrapolate(local_start_time, local_end_time);
    }

    // Compare extrapolated values with ICP prediction
    if (use_ICP) {
      // Make a copy of the two intervals to be aligned
      local_block[0][0].ErasePoints();
      local_block[0][0].insert(local_block[0][0].end(),
                 start_point_interval[0][interval-first_available_interval-1],
                 start_point_interval[0][interval-first_available_interval+1]);
      // Run the ICP
      if (IntervalICP(local_block[0][0], 
	                  std::distance(start_point_interval[0][interval-first_available_interval-1],
					                start_point_interval[0][interval-first_available_interval]),
				      dX, dY, dkappa)) {
		// Write the extrapolation
		slam_ICP_fd = fopen("slam_vs_ICP.txt", "a");
		fprintf(slam_ICP_fd, "%5d %8.3f %8.3f %8.3f   ", interval,
		        local_X_spline.Value(local_end_time - 0.5 * scan_line_duration),
	            local_Y_spline.Value(local_end_time - 0.5 * scan_line_duration),
	            local_kappa_spline.Value(local_end_time - 0.5 * scan_line_duration) *
			    180.0 / pi);
		
		// Determine X, Y, and kappa for central time of both intervals
		previous_X     = local_X_spline.Value(local_end_time - 1.5 * scan_line_duration);
		previous_Y     = local_Y_spline.Value(local_end_time - 1.5 * scan_line_duration);
		previous_kappa = local_kappa_spline.Value(local_end_time - 1.5 * scan_line_duration);
		X     = previous_X + cos(previous_kappa) * dX - sin(previous_kappa) * dY;
		Y     = previous_Y + sin(previous_kappa) * dX + cos(previous_kappa) * dY;
		kappa = previous_kappa + dkappa;

		// Write the ICP prediction
		fprintf(slam_ICP_fd, "%8.3f %8.3f %8.3f   ", X, Y, kappa * 180.0 / pi);
		
		// Improve the local splines for X, Y, and kappa
	    dX = X - previous_X; // Change between mid points of last two intervals
	    dY = Y - previous_Y;
	    X     += dX / 2.0; // Linearly extrapolated to local_end_time
	    Y     += dY / 2.0;
	    kappa += dkappa / 2.0;
	    previous_X     = local_X_spline.Value(local_start_time); 
	    previous_Y     = local_Y_spline.Value(local_start_time); 
	    previous_kappa = local_kappa_spline.Value(local_start_time); 
        local_X_spline.SetLinear(local_start_time, local_end_time, previous_X, X);
        local_Y_spline.SetLinear(local_start_time, local_end_time, previous_Y, Y);
        local_kappa_spline.SetLinear(local_start_time, local_end_time, 
	                                 previous_kappa, kappa);
	                                 
	    // Save extrapolated Z, phi and omega, and ICP-based X, Y, and kappa
	    // splines as fallback option in case the SLAM fails.
	    fallback_X_spline = local_X_spline;
	    fallback_Y_spline = local_Y_spline;
	    fallback_Z_spline = local_Z_spline;
	    fallback_omega_spline = local_omega_spline;
	    fallback_phi_spline = local_phi_spline;
	    fallback_kappa_spline = local_kappa_spline;
      }
      else {
        printf("ICP failed\n");
      }
    }
      
	// Assign segments to planes based on the approximate transformation as
	// defined by the approximate local splines.
	for (sensor=0; sensor<3; sensor++)
      AssignSegmentsToPlanes(block[sensor][current_part], interval,
	                         segments[sensor], planes, false,
						     local_omega_spline, local_phi_spline, local_kappa_spline, 
	                         local_X_spline, local_Y_spline, local_Z_spline, 
							 bounds_of_planes, max_scans_for_loop_closure, only_2D,
							 use_old_labels, max_dist_segment_to_plane);

    // Copy points of the local scan line intervals together
	for (sensor=0, strip=local_block.begin(); sensor<3; sensor++, strip++) {
	  sensordata = strip->begin();
	  sensordata->ErasePoints();
      sensordata->insert(sensordata->end(),
                         start_point_interval[sensor][interval-first_available_interval+1-num_intervals_local_current],
                         start_point_interval[sensor][interval-first_available_interval+1]);
    }

    // Output of scans transformed by initial transformation
    if (output_initial)
      SaveTransformedPointSet(interval, local_start_time, local_end_time,
	                          local_block, local_omega_spline, local_phi_spline,
						      local_kappa_spline, local_X_spline, local_Y_spline,
							  local_Z_spline, (char *) "initial", interval_dir,
							  planes);

    // Estimate pose and assign segments to planes until no further segments
    // are assigned. As the approximate transformation may not be very accurate,
    // it may be that some segments will not be associated to the planes, 
	// because they are a little too far away from a plane.
    // Therefore, the process of association (assignment of segments to planes)
    // is iterated after the estimation of the pose and plane parameters until
    // no further associations are found.
	num_planes = planes.size(); 
    do {
      
      // Estimate pose splines but fix the plane parameters
      EstimatePoseAndPlanes(interval, local_block, false,
	                        local_omega_spline, local_phi_spline, local_kappa_spline, 
	                        local_X_spline, local_Y_spline, local_Z_spline, planes,
						    local_start_time, local_end_time,
							true, false, true, sigma0,
							weight_offset_constraint,
							weight_curvature_constraint, NULL, only_5_regpar,
							NULL, only_2D, scan_line_duration);

	  // Assign segments to planes based on the improved transformation. Do not
	  // create new planes in this step. We first need to optimise the
	  // pose estimation using the already known planes.
      new_assignment = false;
      for (sensor=0; sensor<3; sensor++) {
        if (AssignSegmentsToPlanes(block[sensor][current_part], interval, 
		                           segments[sensor], planes, false,
						           local_omega_spline, local_phi_spline, local_kappa_spline, 
	                               local_X_spline, local_Y_spline, local_Z_spline, bounds_of_planes,
						           max_scans_for_loop_closure, only_2D,
								   use_old_labels, max_dist_segment_to_plane))
		  new_assignment = true;
	  }
	  if (new_assignment) {
	    printf("Assignment loop in interval %d\n", interval);
	    num_assignment_in_loop++;
	    // Recompose the scan points
	    for (sensor=0, strip=local_block.begin(); sensor<3; sensor++, strip++) {
	      sensordata = strip->begin();
	      sensordata->ErasePoints();
          sensordata->insert(sensordata->end(),
                             start_point_interval[sensor][interval-first_available_interval+1-num_intervals_local_current],
                             start_point_interval[sensor][interval-first_available_interval+1]);
        }
	  }
    } while (new_assignment);
    
    // Write the SLAM result
	if (use_ICP) {
	  fprintf(slam_ICP_fd, "%8.3f %8.3f %8.3f %8.3f %8.3f %8.3f %d\n",
	          local_X_spline.Value(local_end_time-scan_line_duration/2.0),
	          local_Y_spline.Value(local_end_time-scan_line_duration/2.0),
	          local_Z_spline.Value(local_end_time-scan_line_duration/2.0),
	          local_omega_spline.Value(local_end_time-scan_line_duration/2.0) *
		      180.0 / pi,
	          local_phi_spline.Value(local_end_time-scan_line_duration/2.0) *
		      180.0 / pi,
	          local_kappa_spline.Value(local_end_time-scan_line_duration/2.0) *
		      180.0 / pi,
			  (fabs(local_X_spline.Value(local_end_time-scan_line_duration/2.0) -
	                fallback_X_spline.Value(local_end_time-scan_line_duration/2.0)) > 0.01 ||
	           fabs(local_Y_spline.Value(local_end_time-scan_line_duration/2.0) -
	                fallback_Y_spline.Value(local_end_time-scan_line_duration/2.0)) > 0.01 ||
	           fabs(local_kappa_spline.Value(local_end_time-scan_line_duration/2.0) -
	                fallback_kappa_spline.Value(local_end_time-scan_line_duration/2.0)) > 0.5 * pi / 180.0));
	  fclose(slam_ICP_fd);
    }
	
	// Check if the SLAM is in line with the ICP result. If it's not, restore
	// the ICP result for X, Y, kappa and extrapolation result for Z, omega, phi.
	if (use_ICP && interval > num_intervals_local_max) {
	  if (fabs(local_X_spline.Value(local_end_time-scan_line_duration/2.0) -
	           fallback_X_spline.Value(local_end_time-scan_line_duration/2.0)) > 0.01 ||
	      fabs(local_Y_spline.Value(local_end_time-scan_line_duration/2.0) -
	           fallback_Y_spline.Value(local_end_time-scan_line_duration/2.0)) > 0.01 ||
	      fabs(local_kappa_spline.Value(local_end_time-scan_line_duration/2.0) -
	           fallback_kappa_spline.Value(local_end_time-scan_line_duration/2.0)) > 0.5 * pi / 180.0) {
	    printf("\nReplacement of SLAM result by ICP result in interval %d NOT EXECUTED\n", interval);
//	    local_X_spline = fallback_X_spline;
//	    local_Y_spline = fallback_Y_spline;
//	    local_Z_spline = fallback_Z_spline;
//	    local_omega_spline = fallback_omega_spline;
//	    local_phi_spline = fallback_phi_spline;
//	    local_kappa_spline = fallback_kappa_spline;
	  }
	}
	
	// Assign remaining segments to new planes
	new_assignment = false;
	for (sensor=0; sensor<3; sensor++) {
      if (AssignSegmentsToPlanes(block[sensor][current_part], interval,
	                             segments[sensor], planes, true,
						         local_omega_spline, local_phi_spline, local_kappa_spline, 
	                             local_X_spline, local_Y_spline, local_Z_spline, bounds_of_planes,
						         max_scans_for_loop_closure, only_2D,
								 use_old_labels, max_dist_segment_to_plane))
	    new_assignment = true;
	}
	if (new_assignment) {
	  // Recompose the scan points
	  for (sensor=0, strip=local_block.begin(); sensor<3; sensor++, strip++) {
	    sensordata = strip->begin();
	    sensordata->ErasePoints();
        sensordata->insert(sensordata->end(),
                           start_point_interval[sensor][interval-first_available_interval+1-num_intervals_local_current],
                           start_point_interval[sensor][interval-first_available_interval+1]);
      }
	}
						   
    if (!debug) {
      printf("Interval %4d  sigma0 %8.4f  assignment in loop %3d\r", interval,
	         sigma0, num_assignment_in_loop);
      if ((interval/25)*25 == interval)
	    printf("Interval %4d  sigma0 %8.4f  assignment in loop %3d\n", interval,
		       sigma0, num_assignment_in_loop);
    }
    
    // Output of scans transformed by locally estimate spline (approximate)
    if (output_approximate) {
      SaveTransformedPointSet(interval, local_start_time, local_end_time, 
	                          local_block, local_omega_spline, local_phi_spline,
						      local_kappa_spline, local_X_spline, local_Y_spline,
							  local_Z_spline, (char *) "approx", interval_dir,
							  planes);
      SaveWalls(interval, planes, bounds_of_planes, interval_dir);
    }
    
    // Store the resulting orientations
    if (interval == 1) {
      time = local_start_time + 0.5 * scan_line_duration; // Middle of first interval
      rotations[0] = Rotation3D(local_omega_spline.Value(time),
                                local_phi_spline.Value(time),
	  						    local_kappa_spline.Value(time));
      translations[0] = Vector3D(local_X_spline.Value(time), 
	                             local_Y_spline.Value(time),
                                 local_Z_spline.Value(time));
      if (output_approximate)
	    fprintf(par_fd, "%4d %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f\n", 0,
	            local_omega_spline.Value(time) * 45.0 / atan(1.0),
                local_phi_spline.Value(time) * 45.0 / atan(1.0),
	  		    local_kappa_spline.Value(time) * 45.0 / atan(1.0),
			    local_X_spline.Value(time), local_Y_spline.Value(time), 
				local_Z_spline.Value(time));
	  if (debug)
        printf("First adjusted pose %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n",
	           local_omega_spline.Value(time) * 45.0 / atan(1.0),
               local_phi_spline.Value(time) * 45.0 / atan(1.0),
	  		   local_kappa_spline.Value(time) * 45.0 / atan(1.0),
			   local_X_spline.Value(time), local_Y_spline.Value(time),
			   local_Z_spline.Value(time));
    }
    time = local_end_time - 0.5 * scan_line_duration; // Middle of current interval
    rotations[interval] = Rotation3D(local_omega_spline.Value(time),
                                     local_phi_spline.Value(time),
	  						         local_kappa_spline.Value(time));
    translations[interval] = Vector3D(local_X_spline.Value(time), 
	                                  local_Y_spline.Value(time),
                                      local_Z_spline.Value(time));
    if (output_approximate)
	  fprintf(par_fd, "%4d %7.4f %7.4f %7.4f %7.4f %7.4f %7.4f\n", interval,
	          local_omega_spline.Value(time) * 45.0 / atan(1.0),
              local_phi_spline.Value(time) * 45.0 / atan(1.0),
	  		  local_kappa_spline.Value(time) * 45.0 / atan(1.0),
			  local_X_spline.Value(time), local_Y_spline.Value(time), 
			  local_Z_spline.Value(time));
	if (debug)
      printf("Interval %d adjusted pose %6.3f %6.3f %6.3f %6.3f %6.3f %6.3f\n",
	         interval, local_omega_spline.Value(time) * 45.0 / atan(1.0),
             local_phi_spline.Value(time) * 45.0 / atan(1.0),
	  		 local_kappa_spline.Value(time) * 45.0 / atan(1.0),
			 local_X_spline.Value(time), local_Y_spline.Value(time),
			 local_Z_spline.Value(time));

    // Output new plane instantiations
	if (planes.size() > num_planes) {
	  printf("\nNew plane(s) in interval %d\n", interval);
	  for (plane=planes.begin()+num_planes; plane!=planes.end(); plane++)
	    printf("  %2d nx %6.2f ny %6.2f nz %6.2f dist %6.2f, interval %d\n",
		       plane->Number(), plane->Normal().X(), plane->Normal().Y(),
		       plane->Normal().Z(), plane->Distance(),
			   plane->Attribute(PT_FirstScanLine));
	}
	
    // After a plane has been instantiated num_intervals_per_section before, segments
	// have been associated to this plane for num_intervals_per_section which 
	// should provide a good amount of observations to improve the plane
	// parameters. Therefore, pose splines are defined covering the past
	// num_intervals_per_section and the spline and plane parameters are estimated.
    if (NeedToAdjustSection(interval, scan_line_duration, planes, 
	                        trajectory_start_time, num_intervals_per_section,
                            section_start_time, section_end_time, only_2D)) {

	  // Define approximate pose splines. This is done by simply fitting splines
	  // to the orientation data collected in the past num_intervals_per_section scans.
	  printf("Need to adjust from %.5f to %.5f, starting at interval %d\n", 
	         section_start_time, section_end_time, interval-num_intervals_per_section+1);
      FitPoseSplines(order_section, section_start_time, section_end_time, 
	                 interval-num_intervals_per_section+1, num_intervals_per_section,
	                 num_intervals_per_bspline_in_section, true,
                     omega_spline, phi_spline, kappa_spline, X_spline, Y_spline,
				     Z_spline, rotations, translations);
	  
	  // Improve approximate scan poses and planes in this section
      EstimatePoseAndPlanes(interval, block, false, 
	                        omega_spline, phi_spline, kappa_spline, 
	                        X_spline, Y_spline, Z_spline, planes,
						    section_start_time, section_end_time, false, true, false,
							sigma0, weight_offset_constraint, 
							weight_curvature_constraint, NULL, only_5_regpar,
							NULL, only_2D, scan_line_duration);
	  
	  // Save pose data. The rotations and translations for every processed scan 
	  // are extracted from the estimated splines
	  ExtractScanLinePoses(interval-num_intervals_per_section+1, 
	                       section_start_time + scan_line_duration / 2.0, 
						   num_intervals_per_section, scan_line_duration,
                           omega_spline, phi_spline, kappa_spline,
					       X_spline, Y_spline, Z_spline, 
						   rotations, translations);
	  
	  // Redefine local linear splines by extracting two values at the end of the
	  // estimated spline.
      local_omega_spline.SetLinear(local_start_time, local_end_time, 
	    omega_spline.Value(local_start_time), omega_spline.Value(local_end_time));
      local_phi_spline.SetLinear(local_start_time, local_end_time, 
	    phi_spline.Value(local_start_time), phi_spline.Value(local_end_time));
      local_kappa_spline.SetLinear(local_start_time, local_end_time, 
	    kappa_spline.Value(local_start_time), kappa_spline.Value(local_end_time));
      local_X_spline.SetLinear(local_start_time, local_end_time, 
	    X_spline.Value(local_start_time), X_spline.Value(local_end_time));
      local_Y_spline.SetLinear(local_start_time, local_end_time, 
	    Y_spline.Value(local_start_time), Y_spline.Value(local_end_time));
      local_Z_spline.SetLinear(local_start_time, local_end_time, 
	    Z_spline.Value(local_start_time), Z_spline.Value(local_end_time));
	}

    // Check if the number of local intervals is already at its maximum
	if (num_intervals_local_current < num_intervals_local_max)
	  num_intervals_local_current++;

	// Set time for next loop
	local_end_time  += scan_line_duration;
	local_start_time = local_end_time - num_intervals_local_current * scan_line_duration;
  }

  if (output_approximate) fclose(par_fd);
  
  // Remove the data of the last part that should not be saved
  printf("\nCurrent part %d\n", current_part);
  for (sensor=0; sensor<3; sensor++) {
    sensordata = block[sensor].begin() + current_part;
    if (current_part > 0) {
  	  // For other parts this is data which was copied from earlier parts and
  	  // has time stamps before the start of this part.
  	  part_start_time = sensordata->StartTime();
  	  for (point=sensordata->begin(), found=false;
		   point!=sensordata->end() && !found; point++) {
		if (point->DoubleAttribute(TimeTag) >= part_start_time) found = true;
	  }
	  if (!found) {
	    printf("Error: start time of part not found in part point set %s\n",
		       sensordata->Name());
		exit(0);
	  }
	  sensordata->erase(sensordata->begin(), point);
  	}
  	
  	// Save the data of the old part
  	if (sensordata->Name())
  	  sprintf(part_name, "temp_%s", sensordata->Name());
  	else {
  	  sprintf(part_name, "temp_s%d_part%6d", sensor, current_part);
  	  for (i=12; i<16; i++)
  	    if (part_name[i] == ' ') part_name[i] = '0';
  	}
	sensordata->SetName(part_name);
  	sensordata->DerivePointFileName("./");
  	sensordata->Write();
  	
  	// If the current part is not the last part, remove future parts.
  	// This may happen in case a users wants to process a smaller trajectory.
//  	if (block[sensor].size() > current_part + 1)
//  	  block[sensor].erase(block[sensor].begin()+current_part+1,
//		                  block[sensor].end());
  }  
  
  // Now the whole trajectory has been processed and we've obtained pose and
  // plane parameters based on very local estimations and estimations in sections
  // of num_intervals_per_section intervals. This now serves as approximation for
  // a global adjustment of all pose and plane parameters.
  
  // Fit splines through all orientation parameters. This gives the approximate
  // coefficients of the pose splines over the whole trajectory.
  FitPoseSplines(order_global, trajectory_start_time, trajectory_end_time, 0, 
                 num_intervals, num_intervals_per_bspline_global, true,
                 omega_spline, phi_spline, kappa_spline, X_spline, Y_spline,
				 Z_spline, rotations, translations);

  // Remove the "temp_" string from the dataset names
  for (sensor=0; sensor<3; sensor++)
  	for (sensordata=block[sensor].begin(); sensordata!=block[sensor].end();
	     sensordata++)
	  if (sensordata->StartTime() < trajectory_end_time)
	    sensordata->SetName(sensordata->Name()+5);

  // Output approximate registration result
  if (output_approximate_point_cloud) {
  	printf("Saving approximate registration\n");
  	SaveTransformedBlock(trajectory_start_time, trajectory_end_time, block,
	                     omega_spline, phi_spline, kappa_spline,
	                     X_spline, Y_spline, Z_spline, reconstruction_dir, 
						 (char *) "_approx", scan_line_duration);
  }
  
  // Simultaneously estimate all pose splines and plane parameters
  for (plane=planes.begin(); plane!=planes.end(); plane++)
    plane->Attribute(PT_Fixed) = 0;
  EstimatePoseAndPlanes(num_intervals-1, block, true,
                        omega_spline, phi_spline, kappa_spline, 
	                    X_spline, Y_spline, Z_spline, planes,
						trajectory_start_time, trajectory_end_time,
						false, true, false, sigma0,
						weight_offset_constraint, weight_curvature_constraint,
						scanner_system_ptr, only_5_regpar, 
						self_calibration_output_file, only_2D,
						scan_line_duration);

  // Transform all observations to the world coordinate system
  if (appendix_point_cloud_files) {
    printf("Transforming points to world coordinate system\n");
    SaveTransformedBlock(trajectory_start_time, trajectory_end_time, block,
	                     omega_spline, phi_spline, kappa_spline,
	                     X_spline, Y_spline, Z_spline, reconstruction_dir,
						 appendix_point_cloud_files, scan_line_duration);
  }
	                        
  // Output frame trajectory 
  if (trajectory_file) {
  	FILE *fd=fopen(trajectory_file, "w");
	for (interval=0, time=trajectory_start_time+scan_line_duration/2.0; 
	     interval<num_intervals; interval++, time+=scan_line_duration) {
      fprintf(fd, "%3d %10.6f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f\n",
	          interval, time, omega_spline.Value(time) / degree,
			  phi_spline.Value(time) / degree, 
			  kappa_spline.Value(time) / degree,
			  X_spline.Value(time), Y_spline.Value(time), 
			  Z_spline.Value(time));
    }
    fclose(fd);
  }
  
  // Output of final planes
  if (plane_file) planes.Write(plane_file); 
  
  // Output of final walls
  SaveWalls(0, planes, bounds_of_planes, (char *) "./");
}

