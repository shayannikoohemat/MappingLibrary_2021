
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
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                      The main splitscans function
--------------------------------------------------------------------------------
*/

void splitscans_cpp(char *block_file, char *root_name, 
				    double time_interval, char *directory,
					char *system_configuration_file)
{
  LaserBlock            block;
  LaserPoints::iterator point_iterator[3];
  LaserPoints           scanlinedata;
  int                   sensor, dataset_index[3], num_datasets[3], interval, i;
  double                end_time;
  bool                  done_with_interval, done_with_all;
  char                  *interval_file, *name, *ch;
  LaserScanningSystem   system;
  Rotation3D            rotation;
  Vector3D              translation;

  
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
  if (system_configuration_file) {
  	if (!system.Read(system_configuration_file)) {
  	  printf("Error reading scanner system configuration file %s\n", 
		     system_configuration_file);
  	  exit(0);
  	}
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

  end_time = 0.0;
  done_with_all = false;
  interval = 0;
  name = (char *) malloc(strlen(root_name) + 7);
  strcpy(name, root_name);
  while (!done_with_all) {
  	scanlinedata.ErasePoints();
  	end_time += time_interval;
  	
  	for (sensor=0; sensor<3; sensor++) {
  	  done_with_interval = false;
  	  if (system_configuration_file) {
        rotation    = system[sensor].Orientation()->rotation();
        translation = system[sensor].Orientation()->vect();
      }

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
  	  	  	// Transform point if system configuration has been specified
  	  	  	if (system_configuration_file) point_iterator[sensor]->vect() =
  	  	  	  rotation * point_iterator[sensor]->vect() + translation;
  	  	    scanlinedata.push_back(*(point_iterator[sensor]));
  	  	    point_iterator[sensor]++;
  	  	  }
  	  	  else done_with_interval = true;
  	  	}
  	  	else done_with_interval = true;
  	  }
  	}
  	
  	if (scanlinedata.empty()) {
  	  done_with_all = true;
  	  continue;
  	}
  	
  	// Write data of the current interval
    sprintf(name+strlen(root_name), "%6d", interval);
    for (i=0, ch=name+strlen(root_name); i<5; i++, ch++)
      if (*ch == ' ') *ch = '0';
    interval_file = ComposeFileName(directory, name, ".laser");
    scanlinedata.Write(interval_file, 0, false);
    interval++;
    if (interval == (interval/100)*100) {
      printf("%6d\r", interval);
      fflush(stdout);
    }
  }
  printf("\n");
}


