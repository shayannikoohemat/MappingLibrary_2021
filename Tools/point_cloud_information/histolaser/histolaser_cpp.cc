
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

 Initial creation:
 Author : George Vosselman
 Date   : 09-06-1999

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
#include "Histogram.h"

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(char *, char *, int *);

/*
--------------------------------------------------------------------------------
                         The main laser2tin function
--------------------------------------------------------------------------------
*/

void histolaser_cpp(char *long_filter, char *infile,
                    int type, int tag, int num_bins,
                    double min_value, double max_value,
                    char *histo_file, char *histo_text_file)
{
  char                 *directory, *filter, *filename;
  int                  icon, filetype, numpts, *frequency, bin, num_values;
  double               bin_size, bin_start;
  LaserBlock           block;
  LaserBlock::iterator unitptr;
  LaserUnit::iterator  subunitptr;
  LaserPointType       datatype;
  FILE                 *fd;

  // Set up the file filter for the input file(s)
  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

  // Allocate histogram
  Histogram histo = Histogram(num_bins, min_value, max_value);

  // Process all input files
  block.Initialise();
  numpts = 0;
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    
    // Set up a laser block
    if (!block.Create(filename, &filetype)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }

    // Loop over all units
    for (unitptr=block.begin(); unitptr!=block.end(); unitptr++) {

      // Loop over all sub units
      for (subunitptr=unitptr->begin();
           subunitptr!=unitptr->end();
           subunitptr++) {

        // Read the point data
        if (!subunitptr->Read()) {
          fprintf(stderr, "Error reading laser points from file %s\n",
                  subunitptr->PointFile());
          exit(0);
        }

        // Add the points to the histogram
        subunitptr->AddToHistogram(histo, type, (LaserPointTag) tag);
        numpts += subunitptr->size();
         
        // Delete the points
        subunitptr->ErasePoints();
      }
    }
  }

  // Store the histogram in an image or text file
  if (histo_file) histo.Write(histo_file);
  if (histo_text_file) {
  	fd = fopen(histo_text_file, "w");
  	if (!fd) {
  	  printf("Error opening file %s\n", histo_text_file);
  	  exit(0);
  	}
  	bin_size = (max_value - min_value) / num_bins;
  	for (bin=0, frequency=histo.Frequencies(), bin_start=min_value;
	     bin<num_bins; bin++, frequency++, bin_start+=bin_size)
  	  fprintf(fd, "%10.4f - %10.4f  : %10d\n", bin_start, bin_start+bin_size,
		      *frequency);
	fclose(fd);
  }
  for (bin=0, frequency=histo.Frequencies(), num_values=0;
	   bin<num_bins; bin++, frequency++)
    num_values += *frequency;
  printf("Histogram contains %d attribute values of %d points.\n",
         num_values, numpts);
}
