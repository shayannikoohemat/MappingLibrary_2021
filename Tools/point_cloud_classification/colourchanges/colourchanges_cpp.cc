
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
 Colouring of a large set of points based on point colours in a small set.
 First the small set is used to create a colour image. Then the image coordinates
 of all points of the large set are calculated. The colour of this image location
 is assigned to the points in the large set.
 
 Initial creation:
 Author : George Vosselman
 Date   : 10-06-2011

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
                         The main colourchanges function
--------------------------------------------------------------------------------
*/

void colourchanges_cpp(char *infile, char *roof_file, const char *output_directory)
{
  char                   *directory, *filter, *filename;
  int                    icon, fileclass, num_pts=0;
  LaserBlock             block;
  LaserBlock::iterator   unitptr;
  LaserUnit::iterator    subunitptr;
  LaserPoints            roof_points;
  DataBoundsLaser        tile_bounds;
  LaserPoints::iterator  point;
  Image                  *roof_image;
  ImageGrid              grid;
  ImagePoint             image_point;
  int                    row, column, red, green, blue;
  
  // Set up the file filter for the input file(s)
  filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

  // Read roof points
  if (!roof_points.Read(roof_file, false)) {
    printf("Error reading points from %s\n", roof_file);
    exit(0);
  }
  
  // Make a colour picture of the roof points
  roof_points.DeriveDataBounds(0);
  roof_image = roof_points.CreateImage(1.0, VFF_TYP_1_BYTE, ColourData, 1, 0.0);
  grid = roof_points.ImagedColour()->Grid()->ImageGridReference();
  roof_points.ErasePoints();
  
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
      
      // Loop over all tiles
      for (subunitptr=unitptr->begin();
           subunitptr!=unitptr->end();
           subunitptr++) {
        printf("\rProcessing tile %s", subunitptr->Name());

        // Read tile points
        subunitptr->Read(subunitptr->PointFile(), false);
        
        // Colour all points
        for (point=subunitptr->begin(); point!=subunitptr->end(); point++) {
          // Convert to image coordinates
          image_point = point->Map_Into_Image(0, grid);
          row = (int) (image_point.Row() + 0.5);
          column = (int) (image_point.Column() + 0.5);
          // Look up colour
          if (roof_image->Inside(row, column)) {
            roof_image->Colour(row, column, red, green, blue);                         
            if (red == 0 && green == 0 && blue == 0)
              point->SetColour(255, 255, 255);
            else
              point->SetColour(red, green, blue);
          }
          else point->SetColour(255, 255, 255);
        }
        
        // Set meta data
        subunitptr->DeriveMetaDataFileName(output_directory);
        subunitptr->DerivePointFileName(output_directory);
        subunitptr->SetSeekOffset(0);
        
        // Write points
        subunitptr->Write(subunitptr->PointFile(), 0, false);
        
        // Erase points
        subunitptr->ErasePoints();
      }
    }
  }
  
  // Set block meta data
  block.DeriveMetaDataFileName(output_directory);
  
  // Write all meta data
  block.WriteMetaData(1, 1);
  printf("\n");       
}
