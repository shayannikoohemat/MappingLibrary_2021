
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
 Conversion of Hokuyo laser data to the binary laser data format

 Initial creation:
 Author : George Vosselman
 Date   : 12-02-2017

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
#include "IMUUnit.h"

/*
--------------------------------------------------------------------------------
                               Storing a point set
--------------------------------------------------------------------------------
*/

void StorePoints(LaserUnit &strip, LaserSubUnit &pointset, char *rootname, 
                 char *output_directory,
				 int max_num_scanlines, double max_time_interval,
				 int first_scanline, int last_scanline,
				 double start_time, double end_time)
{
  char *name;
   
  name = (char *) malloc(strlen(rootname) + 8);
  sprintf(name, "%s_%6d", rootname, strip.size()+1);
  for (int i=0; i<strlen(name); i++) if (name[i] == ' ') name[i] = '0';
  pointset.SetName(name);
  free(name);
  pointset.DeriveMetaDataFileName(output_directory);
  pointset.DerivePointFileName(output_directory);
  if (max_num_scanlines)
  	pointset.SetScanLineRange(first_scanline, last_scanline);
  if (max_time_interval > 0.0)
    pointset.SetTimeRange(start_time, end_time);
  pointset.Write(false);
  pointset.ErasePoints();
  strip.push_back(pointset); // Add meta data to strip
}

/*
--------------------------------------------------------------------------------
                               Storing a point set
--------------------------------------------------------------------------------
*/

void StoreIMUReadings(IMUUnit &imu_unit, IMUReadings &imu_readings, 
                      char *rootname, char *output_directory,
				      double start_time, double end_time)
{
  char *name;
  IMUUnit::iterator readings;
   
  name = (char *) malloc(strlen(rootname) + 8);
  sprintf(name, "%s_%6d", rootname, imu_unit.size()+1);
  for (int i=0; i<strlen(name); i++) if (name[i] == ' ') name[i] = '0';
  imu_readings.SetName(name);
  free(name);
  imu_readings.DeriveMetaDataFileName(output_directory);
  imu_readings.DeriveIMUDataFileName(output_directory);
  imu_readings.SetTimeRange(start_time, end_time);
  imu_readings.Write();
  imu_readings.erase(imu_readings.begin(), imu_readings.end());
  imu_unit.push_back(imu_readings); // Add meta data to IMU unit
}

/*
--------------------------------------------------------------------------------
                  Read the start time of the first record
--------------------------------------------------------------------------------
*/

long long int StartTime(char *filename, bool imu)
{
  FILE *fd;
  long long int itime_ref;
  char *token, delimiters[2], line[40000];
  
  fd = fopen(filename, "r");
  if (!fd) {
  	printf("Error opening file %s\n", filename);
  	exit(0);
  }
  for (int i=0; i<2; i++) {
    if (!fgets(line, 40000, fd)) {
      printf("Error reading time reference from %s\n", filename);
      exit(0);
  	}
  }
  sprintf(delimiters, "%s", ","); // Delimiter for comma separated file
  if (imu) {
    token = strtok(line, delimiters); // Use time
  }
  else {
    token = strtok(line, delimiters); // Ignore time
    token = strtok(NULL, delimiters); // Ignore record index
    token = strtok(NULL, delimiters); // Time stamp
  }
  sscanf(token, "%lld", &itime_ref);
  fclose(fd);
  return itime_ref;
}

/*
--------------------------------------------------------------------------------
                      The main hokuyo2laser function
--------------------------------------------------------------------------------
*/

void hokuyo2laser_cpp(char *in_file, char *in_file0, char *in_file1,
                      char *in_file2,
                      char *rootname, char *output_directory,
                      int max_num_points, int max_num_scanlines,
					  double max_time_interval, char *time_ref_file,
					  char *cal_file, char *sys_file, int scanner_id_user, 
					  int scanlinestep, char *imu_file)
{
  LaserBlock             block;
  LaserUnit              strip;
  LaserSubUnit           pointset;
  LaserPoint             point;
  LaserScanner           scanner;
  LaserScanningSystem    system;
  Rotation3D             rotation;
  Vector3D               translation;
  IMUReadings            imu_readings;
  IMUUnit                imu_unit;
  FILE                   *in_fd, *time_ref_fd;
  char                   line[40000], *name, *directory, *outfile, delimiters[2],
                         *token, *in_files[3], *rootname_strip;
  int                    i, num_points, num_scanlines, num_steps, sensor,
                         num_points_for_storing, num_scanlines_for_storing,
						 scanner_id, num_readings;
  float                  angle, start_angle, end_angle, angle_step;
  long long int          itime, itime_ref;
  double                 time, scan_start_time, time_step, time_for_storing,
                         range, start_time, range_offset, range_scale,
						 angle_scale, avx, avy, avz, accx, accy, accz;
  bool                   found, debug_imu=true;

  // Store file names in array
  if (in_file) {
  	in_files[0] = in_file;
  	in_files[1] = in_files[2] = NULL;
  }
  else {
  	in_files[0] = in_file0;
  	in_files[1] = in_file1;
  	in_files[2] = in_file2;
  }
  sprintf(delimiters, "%s", ","); // Delimiter for comma separated file

  // Determine the reference time
  if (time_ref_file) {
  	itime_ref = StartTime(time_ref_file, false);
  }
  else { // Take the start time of the sensor that started as the last one
  	itime_ref = 0;
  	for (sensor=0; sensor<3; sensor++) {
  	  if (in_files[sensor]) {
  	  	itime = StartTime(in_files[sensor], false);
  	  	if (itime > itime_ref) itime_ref = itime;
  	  }
  	}
  	if (imu_file) {
  	  itime = StartTime(imu_file, true);
  	  if (itime > itime_ref) itime_ref = itime;
  	}
  }
  printf("Start time %lld\n", itime_ref);
  
  // Read calibration and orientation data
  if (sys_file) {
  	if (!system.Read(sys_file)) {
  	  printf("Error reading scanner system configuration file %s\n", sys_file);
  	  exit(0);
  	}
  }
  // Or read the scanner calibration data and use no scanner orientation
  else if (cal_file) {
    if (!scanner.Read(cal_file)) {
      printf("Error reading scanner calibration file %s\n", cal_file);
      exit(0);
    }
    scanner.InitialiseOrientationData();
  }
  // Or use no calibration or orientation data
  else {
    scanner.InitialiseCalibrationData();
    scanner.InitialiseOrientationData();
  }
  
  // Loop over the input Hokuyo files
  if (!in_file) rootname_strip = (char *) malloc(strlen(rootname) + 4);
  sprintf(delimiters, "%s", ","); // Delimiter for comma separated file
  for (sensor=0; sensor<3; sensor++) {
  	if (!in_files[sensor]) continue;
  	strip.ReInitialise();
  	printf("Processing input file %s\n", in_files[sensor]);
	
	// Set strip root name
	if (in_file) rootname_strip = rootname;
	else sprintf(rootname_strip, "%s_s%d", rootname, sensor);
	
  	// Select the scanner and extract calibration and orientation data
  	if (scanner_id_user == -1) scanner_id = sensor;
  	else scanner_id = scanner_id_user;
    if (sys_file) {
  	  // Copy the information on the selected scanner
  	  found = false;
  	  for (i=0; i<system.size() && !found; i++) {
  	    if (system[i].ScannerID() == scanner_id) {
  	      scanner = system[i];
  	      found = true;
  	    }
      }
  	  if (!found) {
  	    printf("Error: scanner system configuration file does not contain information on scanner %d\n",
		       scanner_id);
	    exit(0);
  	  }
    }
    // Only calibration data, use default orientation data
    else if (cal_file) {
      scanner.InitialiseOrientationData();
    }
    // Use default calibration and orientation data
    else {
      scanner.InitialiseCalibrationData();
      scanner.InitialiseOrientationData();
    }
  
    // Extract the required parameters
    range_offset = scanner.Calibration()->RangeOffset();
    range_scale  = scanner.Calibration()->RangeScale();
    angle_scale  = scanner.Calibration()->AngleScale();
    rotation     = scanner.Orientation()->rotation();
    translation  = scanner.Orientation()->vect();

	// Re-set counters
    num_points = num_scanlines = 0;
    num_points_for_storing = max_num_points;
    num_scanlines_for_storing = max_num_scanlines;
    printf("processed points / scan lines / seconds / files\n");

    // Open the input file
    in_fd = fopen(in_files[sensor], "r");
    if (!in_fd) {
      fprintf(stderr, "Error opening input file %s\n", in_files[sensor]);
      exit(0);
    }

    // Skip the header record
    fgets(line, 40000, in_fd);
    if (feof(in_fd)) {
      fprintf(stderr, "Error: end of file reached after reading header line.\n");
      exit(0);
    }
  
    // Process all records
    point.Z() = 0.0;
    point.Attribute(ScanNumberTag) = scanner_id;
    pointset.DataOrganisation() = StripWise;
    do {
      if (fgets(line, 40000, in_fd)) {
        num_scanlines++;

        // Extract the record header information
        token = strtok(line, delimiters); // Ignore time
        token = strtok(NULL, delimiters); // Ignore record index
        token = strtok(NULL, delimiters); // Time stamp
        sscanf(token, "%lld", &itime);
        scan_start_time = (double) (itime - itime_ref) / 1.0e9; // Convert to seconds
        if (num_scanlines == 1) {
          start_time = scan_start_time;
          if (max_time_interval > 0.0) {
		    time_for_storing = max_time_interval;
		    while (start_time > time_for_storing) time_for_storing += max_time_interval;
	      }
        }
	    token = strtok(NULL, delimiters); // Ignore frame id
        token = strtok(NULL, delimiters); // Start angle
        sscanf(token, "%f", &start_angle);
        token = strtok(NULL, delimiters); // End angle
        sscanf(token, "%f", &end_angle);
        token = strtok(NULL, delimiters); // Angle step
        sscanf(token, "%f", &angle_step);
        num_steps = (int) ((end_angle - start_angle + 1e-6) / angle_step) + 1;
        token = strtok(NULL, delimiters); // Time step
        sscanf(token, "%lf", &time_step);
        token = strtok(NULL, delimiters); // Ignore scan time
        token = strtok(NULL, delimiters); // Ignore range minimum
        token = strtok(NULL, delimiters); // Ignore range maximum

        for (i=0, time=scan_start_time, angle=start_angle; i<num_steps;
	         i++, time+=time_step, angle+=angle_step) {
          // Read the next range  
          token = strtok(NULL, delimiters);
		   
		  // Skip points before start of reference time
      	  if (time < 0.0) continue;
      	
      	  // Check for storing based on time interval
		  if (max_time_interval > 0.0 && time >= time_for_storing) {
      	    StorePoints(strip, pointset, rootname_strip, output_directory,
			            max_num_scanlines, max_time_interval,
					    num_scanlines_for_storing - max_num_scanlines,
					    num_scanlines_for_storing - 1,
					    time_for_storing - max_time_interval, time_for_storing);
      	    time_for_storing += max_time_interval;
      	  }

          if (!strncmp(token, "inf", 3)) {
            range = 1.0;
            point.Label(-1); // Invalid point
          }
          else {
      	    sscanf(token, "%lf", &range);
      	    range = range * range_scale + range_offset;
      	    point.Label(1); // Valid point
          }
          // Calculate point coordinates in sensor coordinate system
      	  point.X() = range * cos(angle * angle_scale);
      	  point.Y() = range * sin(angle * angle_scale);
      	  point.Z() = 0.0;
      	  // Transform to frame coordinate system if orientation data
      	  // has been supplied.
      	  if (sys_file) point.vect() = rotation * point.vect() + translation;
      	  // Set attributes time and scan line number
      	  point.SetDoubleAttribute(TimeTag, time);
      	  point.SetAttribute(ScanLineNumberTag, num_scanlines);
      	  if(scanlinestep==1 || (scanlinestep!=1 && num_scanlines%scanlinestep==0))
	      	  pointset.push_back(point);
      	  num_points++;
      	  if ((num_points / 10000) * 10000 == num_points) {
      	    printf("%d / %d / %.2f / %d\r", num_points, num_scanlines,
			       time - start_time, strip.size());
            fflush(stdout);
      	  }
      	
      	  // Check for storing based on number of points
      	  if (max_num_points && num_points == num_points_for_storing) {
      	    StorePoints(strip, pointset, rootname_strip, output_directory,
			            max_num_scanlines, max_time_interval,
					    num_scanlines_for_storing - max_num_scanlines,
					    num_scanlines_for_storing - 1,
					    time_for_storing - max_time_interval, time_for_storing);
      	    num_points_for_storing += max_num_points;
      	  }
        }
      
        // Check for storing based on number of scanlines
        if (max_num_scanlines && num_scanlines == num_scanlines_for_storing) {
      	  StorePoints(strip, pointset, rootname_strip, output_directory,
		              max_num_scanlines, max_time_interval,
					  num_scanlines_for_storing - max_num_scanlines,
					  num_scanlines_for_storing - 1,
					  time_for_storing - max_time_interval, time_for_storing);
      	  num_scanlines_for_storing += max_num_scanlines;
        }      
      }
    } while (!feof(in_fd));
    fclose(in_fd);

    // Save the remaining points
    if (pointset.size())
      StorePoints(strip, pointset, rootname_strip, output_directory,
	  	          max_num_scanlines, max_time_interval,
				  num_scanlines_for_storing - max_num_scanlines,
				  num_scanlines - 1,
				  time_for_storing - max_time_interval, time);
    printf("%d / %d / %.2f / %d\n", num_points, num_scanlines,
		   time - start_time, strip.size());
  	
  	 
    // Set strip meta data
    strip.DataOrganisation() = StripWise;
    if (strip.size() == 1) strip.SetPointFile(pointset.PointFile());
    strip.SetName(rootname_strip);
    strip.DeriveMetaDataFileName(output_directory);
    if (!in_file) block.push_back(strip);
  }

  // Write all meta Hokuyo data
  if (in_file) strip.WriteMetaData(1);
  else {
  	block.SetName(rootname);
  	block.DeriveMetaDataFileName(output_directory);
    block.WriteMetaData(1, 1);
  }
    
  // Import the IMU readings
  if (imu_file) {
  	printf("Processing input file %s\n", imu_file);
	
	// Set counter
    num_readings = 0;
    printf("processed readings / files\n");

    // Open the input file
    in_fd = fopen(imu_file, "r");
    if (!in_fd) {
      fprintf(stderr, "Error opening input file %s\n", imu_file);
      exit(0);
    }

    // Skip the header record
    fgets(line, 40000, in_fd);
    if (feof(in_fd)) {
      fprintf(stderr, "Error: end of file reached after reading header line.\n");
      exit(0);
    }
  
    // Process all records
    do {
      if (fgets(line, 40000, in_fd)) {
        num_readings++;
        // Extract the record header information
        token = strtok(line, delimiters); // Get time
        sscanf(token, "%lld", &itime);
        time = (double) (itime - itime_ref) / 1.0e9; // Convert to seconds
        if (num_readings == 1) {
	      time_for_storing = max_time_interval;
		  while (time > time_for_storing) time_for_storing += max_time_interval;
        }
        for (i=0; i<17; i++) token = strtok(NULL, delimiters); // Jump to angular velocities
        sscanf(token, "%lf", &avx);
        token = strtok(NULL, delimiters); 
        sscanf(token, "%lf", &avy);
        token = strtok(NULL, delimiters); 
        sscanf(token, "%lf", &avz);
        for (i=0; i<10; i++) token = strtok(NULL, delimiters); // Jump to accelerations
        sscanf(token, "%lf", &accx);
        token = strtok(NULL, delimiters); 
        sscanf(token, "%lf", &accy);
        token = strtok(NULL, delimiters); 
        sscanf(token, "%lf", &accz);
				
		// Skip points before start of reference time
      	if (time < 0.0) continue;
      	
      	// Check for storing based on time interval
		if (time >= time_for_storing) {
      	  StoreIMUReadings(imu_unit, imu_readings, rootname, output_directory,
			               time_for_storing-max_time_interval, time_for_storing);
      	  time_for_storing += max_time_interval;
        }

      	if ((num_readings / 10000) * 10000 == num_readings) {
      	  printf("%d / %d\r", num_readings, imu_unit.size());
          fflush(stdout);
      	}
        
		// Save the next reading
        imu_readings.push_back(IMUReading(time, avx, avy, avz, accx, accy, accz));
      }
    } while (!feof(in_fd));
    fclose(in_fd);

    // Save the remaining IMU readings
    if (imu_readings.size())
      StoreIMUReadings(imu_unit, imu_readings, rootname, output_directory,
	                   time_for_storing - max_time_interval, time);
    printf("%d / %d\n", num_readings, imu_unit.size());
  	 
    // Set and save IMU unit meta data
    imu_unit.SetName(rootname);
    imu_unit.DeriveMetaDataFileName(output_directory);
    imu_unit.WriteMetaData(true);
  } 
}
