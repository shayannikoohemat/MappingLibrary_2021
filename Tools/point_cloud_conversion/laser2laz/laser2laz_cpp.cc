
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


/*
--------------------------------------------------------------------------------

 Initial creation:
 Author : George Vosselman
 Date   : 21-09-2013

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
#include "laswriter.hpp"

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(char *, char *, int *);

/*
--------------------------------------------------------------------------------
                      The main laser2laz function
--------------------------------------------------------------------------------
*/


void laser2laz_cpp(char *long_filter, char *input_file,
                   const char *output_dir, bool one_output_file,
				   char *output_file)
{
  char                   *directory, *filter, *filename, *laz_name;
  int                    icon, fileclass;
  LaserBlock             block;
  LaserBlock::iterator   unitptr;
  LaserUnit::iterator    subunitptr;
  LaserSubUnit::const_iterator point;
  LASwriter              *laswriter;
  LASheader              lasheader;
  LASpoint               laspoint;
  long long int          num_pts;
  bool                   colour, output_progress;
  DataBoundsLaser        bounds;
  
  // Set up the file filter for the input file(s)
  if (long_filter) filter = long_filter;
  else filter = input_file;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);

  // Count all points in case of output to one file
  if (one_output_file) {
    printf("Counting points and determining bounding box\n");
    block.Initialise();
    num_pts = 0;
    icon = 0;
    while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    
      // Set up a laser block
      if (!block.Create(filename, &fileclass)) {
        fprintf(stderr, "Error reading meta data file %s\n", filename);
        exit(0);
      }
      
      output_progress = (long_filter || block.size() > 1 ||
                         block.begin()->size() > 1);
                       
      // Loop over all units
      for (unitptr=block.begin(); unitptr!=block.end(); unitptr++) {

        // Loop over all subunits
        for (subunitptr=unitptr->begin(); subunitptr!=unitptr->end();
             subunitptr++) {

          if (output_progress)
            printf("Processing point set %s\r", subunitptr->Name());
          
          // Read the point data
          if (!subunitptr->Read(subunitptr->PointFile(), false)) {
            fprintf(stderr, "Error reading laser points from file %s\n",
                    subunitptr->PointFile());
            exit(0);
          }

          // Count points
          num_pts += subunitptr->size();
          
          // Update bounds
          subunitptr->DeriveDataBounds(1);
          bounds.Update(subunitptr->DataBounds());
          
          // Delete points
          subunitptr->ErasePoints();
        }
      }
    }
    if (output_progress) printf("\n");
    
    // Check if there's colour in the data bounds
    colour = bounds.Minimum().HasAttribute(ColourTag);
    
   
    // Open LAS output file and write the header
    laswriter = block.begin()->begin()->WriteLASHeader(output_file, num_pts,
                                                       colour, laspoint,
													   lasheader, &bounds);
	printf("LAS header initialised for %lld points\n", num_pts);

    block.begin()->begin()->ErasePoints();
  }
  
  // Process all input files
  printf("Setting point colours\n");
  block.Initialise();
  icon = 0;
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    
    // Set up a laser block
    if (!block.Create(filename, &fileclass)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }

    output_progress = (long_filter || block.size() > 1 ||
                       block.begin()->size() > 1);
                       
    // Loop over all strips
    for (unitptr=block.begin(); unitptr!=block.end(); unitptr++) {

      // Loop over all strip parts
      for (subunitptr=unitptr->begin();
           subunitptr!=unitptr->end();
           subunitptr++) {

        if (output_progress)
          printf("Processing point set %s\r", subunitptr->Name());
          
        // Read the point data
        if (!subunitptr->Read(subunitptr->PointFile(), false)) {
          fprintf(stderr, "Error reading laser points from file %s\n",
                  subunitptr->PointFile());
          exit(0);
        }
        
        // Write all points to the same LAS file
        if (one_output_file) {
          subunitptr->WriteLASPoints(laswriter, laspoint, colour);	
        }
        
        // Otherwise convert each file separately
        else {
          // Determine the bounds
          subunitptr->DeriveDataBounds(0);

          // Construct laz file name
          if (!subunitptr->Name()) subunitptr->LaserDataFiles::DeriveName();
          laz_name = ComposeFileName(output_dir, subunitptr->Name(), ".laz", NULL);

          // Write the LAS file
		  subunitptr->WriteLAS(laz_name, true);        
        }
        
        // Delete the points
        subunitptr->ErasePoints();
      }
    }
  }
  
  // Close the LAS file for all points
  if (one_output_file) laswriter->close();
  if (output_progress) printf("\nDone.\n");
}
