
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
 Conversion of Hokuyo laser data to the binary laser data format

 Initial creation:
 Author : George Vosselman
 Date   : 11-01-2015

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
#include "LaserUnit.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                      The main ubh2laser function
--------------------------------------------------------------------------------
*/

void ubh2laser_cpp(char *input_file, char *output_file, char *rootname,
                   char *output_directory, int scan, bool txt)
{
  LaserPoints            points;
  LaserPoint             point;
  FILE                   *fd;
  char                   *line, *buffer, *measurement, *filename, *fullfilename,
                         *ch, *token;
  int                    steps_per_scanline, points_per_scanline, range, i,
                         num_scanlines=0, msec_per_scanline, rootname_length,
						 dir, previous_dir, intensity;
  double                 angle, angle_increment, pi=4.0*atan(1.0),
                         time_increment, time, time_per_scanline;

  // Open the input file
  fd = fopen(input_file, "r");
  if (!fd) {
  	printf("Error opening input file %s\n", input_file);
  	exit(0);
  }
  
  // Set scan number
  if (scan != -1) point.Attribute(ScanNumberTag) = scan;
  
  // Prepare output file name
  if (rootname) {
  	rootname_length = strlen(rootname);
    filename = (char *) malloc(rootname_length + 6);
    strcpy(filename, rootname);
  }

  // Assume default Hokuyo parameters for text files  
  if (txt) {
    angle_increment = 0.25 * pi / 180.0;
    time_per_scanline = 0.025; // seconds
  	time_increment = time_per_scanline / 1440;
  	num_scanlines = -1;
  	previous_dir = 1080;
  }
  
  // Process all records
  buffer = (char *) malloc(7000);
  while ((line = fgets(buffer, 7000, fd))) {
  	
  	if (txt) {
      token = strtok (line,",");
  	  sscanf(token, "%d", &dir);
      token = strtok (NULL, ",");
      sscanf(token, "%d", &range);
      token = strtok (NULL, ",");
      sscanf(token, "%d", &intensity);
      // Output previous scan line if needed
      if (dir < previous_dir) {
        if (rootname && points.size()) {
          sprintf(filename+rootname_length, "%5d", num_scanlines);
          for (ch=filename+rootname_length, i=0; i<5; i++, ch++)
            if (*ch == ' ') *ch = '0';
          fullfilename = ComposeFileName(output_directory, filename, ".laser");
          points.Write(fullfilename, 0, false);
          points.ErasePoints();
        }
  	    num_scanlines++;
  	    point.Attribute(ScanLineNumberTag) = num_scanlines;
  	  }
      // Add point to current scan line
      angle = (double) dir * angle_increment;
      point.X() = (double) range * cos(angle) / 1000.0;
      point.Y() = (double) range * sin(angle) / 1000.0;
      point.FloatAttribute(AngleTag) = angle;
      time = num_scanlines * time_per_scanline + dir * time_increment;
      point.DoubleAttribute(TimeTag) = time;
      point.Reflectance() = intensity; 
      points.push_back(point);
      previous_dir = dir;
  	}
  	
  	else {  	
  	  if (!strcmp(line, "[totalSteps]\n")) {
  	    if ((line = fgets(buffer, 7000, fd))) {
  	  	  sscanf(line, "%d", &steps_per_scanline);
  	  	  printf("%d steps per scanline\n", steps_per_scanline);
  	  	  angle_increment = 2.0 * pi / ((double) steps_per_scanline);
  	    }
  	    else {
  	  	  printf("Error reading number of steps per scanline\n");
  	  	  exit(0);
  	    }
  	  }
  	
  	  else if (!strcmp(line, "[endStep]\n")) {
  	    if ((line = fgets(buffer, 7000, fd))) {
  	  	  sscanf(line, "%d", &points_per_scanline);
  	  	  printf("%d points per scanline\n", points_per_scanline);
  	    }
  	    else {
  	  	  printf("Error reading number of points per scanline\n");
  	  	  exit(0);
  	    }
      }
  	
  	  else if (!strcmp(line, "[scanMsec]\n")) {
  	    if ((line = fgets(buffer, 7000, fd))) {
  	  	  sscanf(line, "%d", &msec_per_scanline);
  	  	  printf("%d milliseconds per scanline\n", msec_per_scanline);
  	  	  time_per_scanline = (double) msec_per_scanline	 / 1000.0;
  	  	  time_increment    = time_per_scanline / steps_per_scanline;
  	    }
  	    else {
  	  	  printf("Error reading time per scanline\n");
  	  	  exit(0);
  	    }
  	  }
  	
  	  else if (!strcmp(line, "[scan]\n")) {
  	    if ((line = fgets(buffer, 7000, fd))) {
  	      measurement = strtok (line,";");
  	      angle = 0.0;
  	      time  = num_scanlines * time_per_scanline;
  	      point.Attribute(ScanLineNumberTag) = num_scanlines;
          while (measurement != NULL) {
            sscanf(measurement, "%d", &range);
            if (range > 1) { // Range 1 indicates no measurement
              point.X() = (double) range * cos(angle) / 1000.0;
              point.Y() = (double) range * sin(angle) / 1000.0;
              point.FloatAttribute(AngleTag) = angle;
              point.DoubleAttribute(TimeTag) = time;
              points.push_back(point);
            }
            angle += angle_increment;
            time  += time_increment;
            measurement = strtok (NULL, ";");
          }
          // Output if output per scanline
          if (rootname) {
            sprintf(filename+rootname_length, "%5d", num_scanlines);
            for (ch=filename+rootname_length, i=0; i<5; i++, ch++)
              if (*ch == ' ') *ch = '0';
            fullfilename = ComposeFileName(output_directory, filename, ".laser");
            points.Write(fullfilename, 0, false);
            points.ErasePoints();
          }
  	      num_scanlines++;
  	    }
  	    else {
  	  	  printf("Error reading scanline %d\n", num_scanlines);
  	  	  exit(0);
  	    }
  	  }
  	
  	  else { // Ignore other tags and skip the next line
  	    if (!(line = fgets(buffer, 7000, fd))) {
  	  	  printf("Error skipping unused tag\n");
  	  	  exit(0);
  	    }
  	  }
    }
  }
  free(buffer);
  
  // Output of all points
  if (output_file) points.Write(output_file, 0, false);
}
