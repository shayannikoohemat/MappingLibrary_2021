
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
 Analysis of pulse length data. Input files can be specified
 explicitly or by a file filter. Averages, and standard deviations of
 pulse length data and reflection strength are calculated as a function of
 the reflection strength and pulse length.

 Initial creation:
 Author : George Vosselman
 Date   : 27-11-2005

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

#include <malloc.h>
/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(char *, char *, int *);

/*
--------------------------------------------------------------------------------
                         The main pla function
--------------------------------------------------------------------------------
*/

void pla(char *long_filter, char *infile, char *file_out)
{
  char                   *directory, *filter, *filename;
  int                    icon, fileclass, r, pl, pc;
  LaserBlock             block;
  LaserBlock::iterator   unitptr;
  LaserUnit::iterator    subunitptr;
  LaserSubUnit::iterator point;
  int                    *countr, *countpl, maxr=100, maxpl=15, count,
                         countplpc[16][6], countrpc[101][6];
  double                 *sumr, *sumpl;
  FILE                   *out;

// Set up the file filter for the input file(s)

  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

  // Initialise counters and sums
  
  countr  = (int *) calloc(maxr+1, sizeof(int));
  sumr    = (double *) calloc(maxpl+1, sizeof(double));
  countpl = (int *) calloc(maxpl+1, sizeof(int));
  sumpl   = (double *) calloc(maxr+1, sizeof(double));
  count   = 0;
  for (int i=0; i<6; i++) {
    for (int j=0; j<=maxpl; j++) countplpc[j][i] = 0;
    for (int j=0; j<=maxr; j++) countrpc[j][i] = 0;
  }
  
// Process all input files

  block.Initialise();
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    
// Set up a laser block

    if (!block.Create(filename, &fileclass)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }

    // Loop over all units
    for (unitptr=block.begin(); unitptr!=block.end(); unitptr++) {

      // Loop over all sub units
      for (subunitptr=unitptr->begin();
           subunitptr!=unitptr->end(); subunitptr++) {

        // Read the point data
        if (!subunitptr->Read()) {
          fprintf(stderr, "Error reading laser points from file %s\n",
                  subunitptr->PointFile());
          exit(0);
        }
        
        // Add data to the statistics sums
        for (point=subunitptr->begin(); point!=subunitptr->end(); point++) {
          r  = point->Reflectance();
          pl = point->PulseLength();
          pc = point->PulseCount();
          if (r <= maxr && pl <= maxpl) {
            count++;
            countr[r]++;    sumpl[r] += pl;
            countpl[pl]++;  sumr[pl] += r;
            countplpc[pl][pc]++;
            countrpc[r][pc]++;
          }
        }
        // Erase the points
        subunitptr->ErasePoints();

      } // End of sub-unit loop
    } // End of unit loop
  } // End of file loop
  
  // Output of analysis
  out = fopen(file_out, "w");
  fprintf(out, "Number of points: %d\n", count);
  fprintf(out, "\n\nReflectance as function of pulse length\n");
  fprintf(out, "Pulse length, count, average reflectance\n");
  for (int i=0; i<=maxpl; i++) {
    fprintf(out, "%3d %7d %7.2f", i, countpl[i], sumr[i]/countpl[i]);
    for (int j=1; j<5; j++)
      fprintf(out, " %7.2f", 100.0 * (double) countplpc[i][j] / countpl[i]);
    fprintf(out, "\n");
  }  
  
  fprintf(out, "\n\nPulse length as function of reflectance\n");
  fprintf(out, "Reflectance, count, average pulse length\n");
  for (int i=0; i<=maxr; i++) {
    fprintf(out, "%3d %7d %7.2f", i, countr[i], sumpl[i]/countr[i]);
    for (int j=1; j<5; j++)
      fprintf(out, " %7.2f", 100.0 * (double) countrpc[i][j] / countr[i]);
    fprintf(out, "\n");
  }  
  fclose(out);
}
