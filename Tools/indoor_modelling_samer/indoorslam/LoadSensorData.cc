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

void LoadNewSensorData(LaserBlock &block, int &part_number, int start_part_number,
                       double trajectory_start_time, double trajectory_end_time,
					   double interval_duration, int num_intervals_per_section,
                       int &first_available_interval, int &last_available_interval,
					   LaserPoints::iterator **start_point_interval)
{
  void SetIntervalStarts(LaserBlock &, int, int, double, double, double, int, int &,
                         LaserPoints::iterator **);

  int                   sensor, index_new_start, index_old_start, i;
  LaserUnit::iterator   old_part, new_part;
  LaserPoints::iterator point;
  bool                  found, debug=false;
  double                switch_time, start_time;
  char                  part_name[100];
  
  printf("Deleting data from interval %d till %d\n", first_available_interval,
         last_available_interval-num_intervals_per_section+1);
  if (debug) {
    printf("Start times of intervals for the three sensors\n");
    for (int i=0; i<3; i++) {
  	  printf("Interval %3d  ", last_available_interval-num_intervals_per_section+1+i);
      for (sensor=0; sensor<3; sensor++) 
  	    printf("%.8f   ", start_point_interval[sensor][last_available_interval-num_intervals_per_section+1+i-first_available_interval]->DoubleAttribute(TimeTag));
  	  printf("\n");
    }
    for (int i=0; i<3; i++) {
  	  printf("Interval %3d  ", last_available_interval-1+i);
      for (sensor=0; sensor<3; sensor++) 
  	    printf("%.8f   ", start_point_interval[sensor][last_available_interval-1+i-first_available_interval]->DoubleAttribute(TimeTag));
  	  printf("\n");
    }
  }

  // Calculate the time to switch from the old to the new part
  // Data up to and including the last available interval has been processed
  // and should be stored in the old part. Data of the last 
  // num_intervals_per_section intervals should, however, be copied to the new part.
  switch_time = trajectory_start_time +
                (last_available_interval+1) * interval_duration;
                
  // Change data for the three sensors
  for (sensor=0; sensor<3; sensor++) {
  	old_part = block[sensor].begin() + part_number;
  	new_part = old_part + 1;
  	
    // Adjust the end time of the old part and start time of the new part 
    old_part->SetTimeRange(old_part->StartTime(), switch_time);
	new_part->SetTimeRange(switch_time, new_part->EndTime());  

    // Determine the index of the first point to be kept
    index_new_start = std::distance(old_part->begin(),
	                       start_point_interval[sensor][last_available_interval - 
                     num_intervals_per_section + 1 - first_available_interval]);
	  	
  	// Remove the data that should not be saved
  	// For the first part this is data before the start of the trajectory
  	if (part_number == start_part_number)
  	  old_part->SetTimeRange(trajectory_start_time, switch_time);
  	
	// For later parts this is data which was copied from earlier parts and
  	// was processed already earlier. This is data before the switch time to
  	// the old part set in the previous call to this function.
  	start_time = old_part->StartTime();
  	for (point=old_part->begin(), found=false;
		 point!=old_part->end() && !found; point++) {
	  if (point->DoubleAttribute(TimeTag) >= start_time) found = true;
	}
	if (!found) {
	  printf("Error: start time of part not found in part point set %s\n",
		     old_part->Name());
	  exit(0);
	}
	index_old_start= std::distance(old_part->begin(), point);
	old_part->erase(old_part->begin(), point);
  	
  	// Save the data of the old part
  	if (old_part->Name())
  	  sprintf(part_name, "temp_%s", old_part->Name());
  	else {
	  sprintf(part_name, "temp_s%d_part%5d", sensor, part_number);
  	  for (i=12; i<16; i++)
  	    if (part_name[i] == ' ') part_name[i] = '0';
  	}
  	old_part->SetName(part_name);
  	old_part->DerivePointFileName("./");
  	old_part->Write();
  	
    // Delete data of the old part, but keep the last num_intervals_per_section
	// intervals
  	old_part->erase(old_part->begin(),
	                old_part->begin() + index_new_start - index_old_start);
	  
	// Read data of the next part
	if (!new_part->Read()) {
	  printf("Error reading sensor data from file %s\n",
	         new_part->PointFile());
	  exit(0);
	}
	
	// Remove plane numbers and residuals that be have been stored in a previous run
	new_part->RemoveAttribute(PlaneNumberTag);
	new_part->RemoveAttribute(ResidualTag);
	
	// Insert data to be kept from the old part 
	new_part->insert(new_part->begin(), old_part->begin(), old_part->end());
	old_part->ErasePoints();
	
	// Set sensor number as point attribute
	new_part->SetAttribute(ScanNumberTag, sensor);
  }
  
  // Set the interval start iterators for the intervals of the new part
  part_number++;
  first_available_interval = last_available_interval - 
                             num_intervals_per_section + 1;
  SetIntervalStarts(block, part_number, start_part_number, trajectory_start_time, 
                    trajectory_end_time, interval_duration,
                    first_available_interval, last_available_interval,
                    start_point_interval);

  if (debug) {
    printf("Start times of intervals for the three sensors after reloading\n");
    for (int i=0; i<3; i++) {
  	  printf("Interval %3d  ", first_available_interval+i);
      for (sensor=0; sensor<3; sensor++) 
  	    printf("%.8f   ", start_point_interval[sensor][i]->DoubleAttribute(TimeTag));
  	  printf("\n");
    }
    for (int i=0; i<3; i++) {
  	  printf("Interval %3d  ", first_available_interval + num_intervals_per_section - 2+i);
      for (sensor=0; sensor<3; sensor++) 
  	    printf("%.8f   ", start_point_interval[sensor][num_intervals_per_section - 2+i]->DoubleAttribute(TimeTag));
  	  printf("\n");
    }
  }

}

void SetIntervalStarts(LaserBlock &block, int part_number, int start_part_number,
                       double trajectory_start_time, double trajectory_end_time,
					   double interval_duration,
                       int first_available_interval, int &last_available_interval,
					   LaserPoints::iterator **start_point_interval)
{
  int                   sensor, num_intervals, interval;
  double                part_start_time, part_end_time, time;
  LaserBlock::iterator  strip;
  LaserUnit::iterator   part;
  LaserPoints::iterator start_point;
  
  // Determine last time with data of all three sensors
  for (sensor=0, strip=block.begin(); sensor<3; sensor++, strip++) {
    part = strip->begin() + part_number;
    time = (part->end()-1)->DoubleAttribute(TimeTag);
    if (sensor == 0)
      part_end_time = time;
    else {
      if (time < part_end_time) part_end_time = time;
    }
  }
  if (part_end_time > trajectory_end_time) part_end_time = trajectory_end_time;
  
  // Determine the last interval of this part
  last_available_interval = 
    (int) ((part_end_time - trajectory_start_time) / interval_duration) - 1;
  
  // Adjust the part end time to the end of this interval
  part_end_time = trajectory_start_time +
                  (last_available_interval+1) * interval_duration;
  part_start_time = trajectory_start_time +
                    first_available_interval * interval_duration;

  printf("Setting start points for intervals %d till %d in period %.5f till %.5f\n",
         first_available_interval, last_available_interval, part_start_time,
		 part_end_time);
		 
  // Determine the first point of every time interval to be processed
  num_intervals = last_available_interval - first_available_interval + 1;
  for (sensor=0, strip=block.begin(); sensor<3; sensor++, strip++) {
  	// Allocate memory for array of point iterators
  	if (part_number == start_part_number)
  	  start_point_interval[sensor] =
	    (LaserPoints::iterator *) malloc((num_intervals+1) *
		                                 sizeof(LaserPoints::iterator));
	else
  	  start_point_interval[sensor] =
	    (LaserPoints::iterator *) realloc(start_point_interval[sensor], 
		                                  (num_intervals+1) *
		                                  sizeof(LaserPoints::iterator));
    if (!start_point_interval[sensor]) {
      printf("Error allocating start_point_interval vector for sensor %d\n",
	         sensor);
	  exit(0);
    }
	part = strip->begin() + part_number;
	start_point = part->begin();
	for (interval=0, time=part_start_time; interval<=num_intervals; 
	     interval++, time+=interval_duration) {
	  while (start_point->DoubleAttribute(TimeTag) < time) start_point++;
	  start_point_interval[sensor][interval] = start_point;
	}
	printf("Sensor %d has data in part %d from scan line %d till scan line %d\n",
	       sensor, part_number, start_point_interval[sensor][0]->Attribute(ScanLineNumberTag),
		   start_point_interval[sensor][num_intervals-1]->Attribute(ScanLineNumberTag));
  }
}
