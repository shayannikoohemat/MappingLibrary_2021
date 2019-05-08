
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
 Sort the tags in laser point files 

 Initial creation:
 Author : George Vosselman
 Date   : 01-12-2015

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
                         The main laser2tin function
--------------------------------------------------------------------------------
*/

void sorttags_cpp(char *long_filter, char *infile)
{
  char                  *directory, *filter, *filename;
  int                   icon, fileclass;
  LaserBlock            block;
  LaserBlock::iterator  unitptr;
  LaserUnit::iterator   subunitptr;
  LaserPoints::iterator point;

  // Set up the file filter for the input file(s)
  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

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
           subunitptr!=unitptr->end();
           subunitptr++) {

        // Read the point data, set the tags, write and delete the point data
        if (!subunitptr->Read(subunitptr->PointFile(), false)) {
          fprintf(stderr, "Error reading laser points from file %s\n",
                  subunitptr->PointFile());
          exit(0);
        }

        for (point=subunitptr->begin(); point!=subunitptr->end(); point++) {
          point->SortAttributes();
        }
        subunitptr->Write(false);
        subunitptr->ErasePoints();
      }
    }
  }
}
