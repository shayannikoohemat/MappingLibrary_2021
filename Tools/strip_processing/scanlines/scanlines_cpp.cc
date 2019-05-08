
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
 Derivation of scan lines ends from strips of laser data. The input can be
 a block file or a strip file. Files can be specified explicitly or
 by a file filter. Meta data can be updated with the new name of the scan
 lines file.

 Initial creation:
 Author : George Vosselman
 Date   : 16-07-1999

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

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(char *, char *, int *);

/*
--------------------------------------------------------------------------------
                         The main scanlines function
--------------------------------------------------------------------------------
*/

void scanlines_cpp(char *long_filter, char *infile,
                   double max_topography_height,
                   int update, char *scanlines_directory,
                   char *scanlines_file, char *meta_file_out,
                   int num_scanlines_flight_path,
                   double approximate_flight_height)
{
  char                     *directory, *filter, *filename;
  int                      icon, fileclass;
  LaserBlock               block;
  LaserBlock::iterator     stripptr;
  LaserUnit::iterator      partptr;
  LaserPoints::iterator    point;

  LaserScanLines::iterator line;

  // Set up the file filter for the input file(s)
  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

  // Process all input files
  block.Initialise();
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    
    // Set up a laser block and check the type of input
    if (!block.Create(filename, &fileclass)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }
    if (fileclass != LASER_BLOCK && fileclass != LASER_STRIP) {
      fprintf(stderr, "Error: Input file should contain meta data of block or strip.\n");
      exit(0);
    }

    // Loop over all strips
    for (stripptr=block.begin(); stripptr!=block.end(); stripptr++) {

      // Derive the scan lines
      stripptr->DeriveScanLines(max_topography_height,
                                num_scanlines_flight_path,
                                approximate_flight_height);

      // Output of the scan lines information
      if (scanlines_file)
        stripptr->LaserScanLinesReference().SetScanLinesFile(scanlines_file);
      else
        stripptr->DeriveScanLinesFileName(scanlines_directory);
      stripptr->LaserScanLinesReference().Write(false);

/*
      printf("Number of scan lines: %d\n",
             stripptr->LaserScanLinesReference().size());
      for (line=stripptr->LaserScanLinesReference().begin();
           line!=stripptr->LaserScanLinesReference().end(); line++)
        printf("%d ", line->NumberOfPoints());
      printf("\n");
*/

      // Output of flight path
      if (num_scanlines_flight_path) {
        stripptr->DeriveFlightPathFileName(scanlines_directory);
        stripptr->WriteFlightPath();
      }

      // Update the meta data of the strip
      if (update) {
        if (meta_file_out) stripptr->SetMetaDataFile(meta_file_out);
        if (!stripptr->WriteMetaData(1)) {
          fprintf(stderr, "Error writing strip meta data to file %s\n",
                  stripptr->MetaDataFile());
          exit(0);
        }
      }
    }
  }
}
