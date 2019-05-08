
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

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(char *, char *, int *);

/*
--------------------------------------------------------------------------------
                      The main laser2ascii function
--------------------------------------------------------------------------------
*/

void laser2ascii_cpp(char *long_filter, char *infile,
                     int store_x, int store_y, int store_z,
                     int store_r, int store_g, int store_b,
                     int store_p, int store_lpf, int store_l, int store_pl,
                     int store_pn, int store_sn, int store_lsn, int store_poln,
                     int store_fs, int store_scan, int store_t,
                     int store_a, int store_scanline, int sclstep, char *asciifile)
{
  char                   *directory, *filter, *filename;
  int                    icon, fileclass, colour;
  LaserBlock             block;
  LaserBlock::iterator   unitptr;
  LaserUnit::iterator    subunitptr;
  LaserSubUnit::const_iterator point;
  FILE                   *ascii;


/* Set up the file filter for the input file(s) */

  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

/* Open the output file */

  ascii = fopen(asciifile, "w");
  if (!ascii) {
    fprintf(stderr, "Error opening output file %s\n", asciifile);
    exit(0);
  }

/* Process all input files */

  block.Initialise();
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    
/* Set up a laser block */

    if (!block.Create(filename, &fileclass)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }

/* Loop over all strips */

    for (unitptr=block.begin(); unitptr!=block.end(); unitptr++) {

/* Loop over all strip parts */

      for (subunitptr=unitptr->begin();
           subunitptr!=unitptr->end();
           subunitptr++) {

/* Read the point data */

        if (!subunitptr->Read()) {
          fprintf(stderr, "Error reading laser points from file %s\n",
                  subunitptr->PointFile());
          exit(0);
        }

// Check if we have colour

        colour = (subunitptr->Scanner().PointType() == ColourPoint ||
                  subunitptr->Scanner().PointType() == MultiColourPoint) ||
                 (store_r && store_g && store_b);

/* Write the point data to the ASCII file */

        for (point=subunitptr->begin(); point!=subunitptr->end(); point++) {
//        	printf("%d\n",point->Attribute(ScanLineNumberTag)%sclstep);
          if (sclstep!=1 && point->Attribute(ScanLineNumberTag)%sclstep!=0) {continue;} 
          if (store_x) fprintf(ascii, "%11.3f", point->X());
          if (store_y) fprintf(ascii, " %11.3f", point->Y());
          if (store_z) fprintf(ascii, " %7.3f", point->Z());
          if (colour) {
            if (store_r) fprintf(ascii, " %3d", point->Red());
            if (store_g) fprintf(ascii, " %3d", point->Green());
            if (store_b) fprintf(ascii, " %3d", point->Blue());
          }
         else if (store_r) fprintf(ascii, " %3d", point->Reflectance());
          if (store_p) fprintf(ascii, " %3d", point->PulseCount());
          if (store_lpf) {
            if (point->IsLastPulse()) fprintf(ascii, " 1");
            else fprintf(ascii, " 0");
          }
          if (store_l) fprintf(ascii, " %3d", point->Label());
          if (store_pl) fprintf(ascii, " %3d", point->PulseLength());
          if (store_pn) fprintf(ascii, " %5d", point->PlaneNumber());
          if (store_scanline) fprintf(ascii, " %5d", point->Attribute(ScanLineNumberTag));
          if (store_sn) fprintf(ascii, " %5d", point->SegmentNumber());
          if (store_lsn) fprintf(ascii, " %lld", point->LongSegmentNumber());
          if (store_poln) fprintf(ascii, " %5d", point->PolygonNumber());
          if (store_fs) fprintf(ascii, " %5d", point->Filtered());
          if (store_scan) fprintf(ascii, " %5d", point->ScanNumber());
          if (store_t) fprintf(ascii, " %.6f", point->DoubleAttribute(TimeTag));
          if (store_a) fprintf(ascii, " %.2f", point->FloatAttribute(AngleTag));
          fprintf(ascii, "\n");
        }

/* Delete the points */

        subunitptr->ErasePoints();
      }
    }
  }

/* Close the ASCII file */

  fclose(ascii);
}
