
/*
    Copyright 2010 University of Twente and Delft University of Technology
 
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
 Collect flights paths of all strips in a block and merge
 them to one file

 Initial creation:
 Author : George Vosselman
 Date   : 19-1-2010

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
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(char *, char *, int *);

/*
--------------------------------------------------------------------------------
                         The main mergeflightpaths function
--------------------------------------------------------------------------------
*/

void mergeflightpaths(char *long_filter, char *infile,
                      char *point_filename, char *topology_filename,
                      bool devide_strip_number_by_ten)
{
  char                     *directory, *filter, *filename, *strip_name, *chsrc,
                           *chdest;
  int                      icon, fileclass, number=0, next_number, strip_number;
  LaserBlock               block;
  LaserBlock::iterator     strip;
  ObjectPoints             points, all_points;
  LineTopologies           tops;
  LineTopology             top;
  
  // Set up the file filter for the input file(s)
  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

  // Allocate strip name
  strip_name = (char *) malloc(1000);
  
  // Process all input files
  block.Initialise();
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    
    // Set up a laser block and check the type of input
    if (!block.Create(filename, &fileclass, true, false)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }
    if (fileclass != LASER_BLOCK && fileclass != LASER_STRIP) {
      fprintf(stderr, "Error: Input file should contain meta data of block or strip.\n");
      exit(0);
    }

    // Loop over all strips
    for (strip=block.begin(); strip!=block.end(); strip++) {
      // Read flight path
      if (!strip->ReadFlightPath()) return;
      // Convert to object points
      points = ObjectPoints(strip->FlightPath(), number);
      // Add object points of this strip to the collection of all strips
      all_points.insert(all_points.end(), points.begin(), points.end());
      // Extract strip number from strip name
      chsrc = strip->Name();
      if (chsrc == NULL) strip_number = tops.size();
      else {
        chdest = strip_name;
        while (*chsrc != 0) {
          if (*chsrc >= '0' && *chsrc <= '9') {
            *chdest = *chsrc;
            chdest++;
          }
          chsrc++;
        }
        *chdest = 0;
        if (strlen(strip_name)) {
          sscanf(strip_name, "%d", &strip_number);
          if (devide_strip_number_by_ten) strip_number /= 10;
        }
        else strip_number = tops.size();
      }
      // Create topology
      next_number = number + points.size();
      for (; number<next_number; number++) top.push_back(PointNumber(number));
      top.Number() = strip_number;
      tops.push_back(top);
      number = next_number;
      // Erase data of this strip
      points.Erase();
      strip->ReInitialise();
      top.Erase();
    }
  }
  
  // Output collected flight path data
  if (!all_points.Write(point_filename)) {
    printf("Error writing flight path points to %s\n", point_filename);
    return;
  }
  if (!tops.Write(topology_filename))
    printf("Error writing flight path topology to %s\n", topology_filename);
}
