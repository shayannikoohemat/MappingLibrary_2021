
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
 Counting the number of points in a laser altimetry block, strip, strip
 part, tile, strip tile, point set or a point file. Files can be specified
 explicitly or by a file filter. 

 Initial creation:
 Author : George Vosselman
 Date   : 16-05-2007

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
                         The main laser2tin function
--------------------------------------------------------------------------------
*/

void countpoints_cpp(char *long_filter, char *infile)
{
  char                 *directory, *filter, *filename;
  int                  icon, fileclass, num_pts;
  long long            num_pts_block, num_pts_strip, grand_total;
  unsigned char        max_num_attributes;
  LaserBlock           block;
  LaserBlock::iterator unitptr;
  LaserUnit::iterator  subunitptr;
  FILE                 *fd;
  double               x_offset, y_offset, z_offset;

/* Set up the file filter for the input file(s) */

  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

/* Process all input files */

  block.Initialise();
  grand_total = 0;
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    // Set up a laser block
    if (!block.Create(filename, &fileclass)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }
    num_pts_block = 0;

    // Loop over all units
    for (unitptr=block.begin(); unitptr!=block.end(); unitptr++) {
      num_pts_strip = 0;
      
      // Loop over all sub units
      for (subunitptr=unitptr->begin();
           subunitptr!=unitptr->end(); subunitptr++) {

        // Open the binary file
        fd = Open_Compressed_File(subunitptr->PointFile(), "rb");
        if (fd == NULL) {
          fprintf(stderr, "Error opening file %s\n",
                  subunitptr->PointFile());
          exit(0);
        }
        // Get number of points from the header
        subunitptr->ReadHeader(fd, &num_pts, &x_offset, &y_offset, &z_offset,
                       &max_num_attributes);
        num_pts_strip += num_pts;
        Close_Compressed_File(fd);
        if (block.Name() == NULL && unitptr->Name() == NULL)
          printf("File %s contains %d points\n", subunitptr->PointFile(),
                 num_pts);
      }
      if (unitptr->Name() != NULL)
        printf("Strip %s has %I64d points\n", unitptr->Name(), num_pts_strip);
      num_pts_block += num_pts_strip;
    }
    if (block.Name() != NULL)
      printf("Block %s has %I64d points\n", block.Name(), num_pts_block);
    grand_total += num_pts_block;
  }
  if (long_filter != NULL)
    printf("Grand total of %I64d points\n", grand_total);
}
