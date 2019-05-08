
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


#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"
#include "LaserBlock.h"
#include "BNF_io.h"
#include <windows.h>

using namespace std;

void PrintUsage()
{
  printf("block2segments sorts points of a tiled block into one point set per segment.\n");
  printf("Usage: block2segments -i <tilewise block meta data file>\n");
  printf("                    -odir <output directory>\n");
}

int main(int argc, char *argv[])
{
  InlineArguments                 *args = new InlineArguments(argc, argv);
  LaserBlock                      block;
  LaserBlock::iterator            unit;
  LaserUnit::iterator             tile;
  LaserPoints::iterator           point;
  char                            *root_dir, segment_name[20], *file_name;
  vector<long long int>           segment_numbers;
  vector<long long int>::iterator segment_number_iterator;
  long long int                   segment_number;
  vector<LaserPoints>             point_sets;
  vector<LaserPoints>::iterator   point_set;
  int                             index;
  FILE                            *segment_file;
  

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i")) {
    printf("Error: -i is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  // Read the meta data
  if (!block.ReadMetaData(args->String("-i"))) {
    printf("Error reading input meta data from %s\n", args->String("-i"));
    exit(0);
  }
  
  root_dir = args->String("-odir");

  // Loop over all tiles
  for (unit=block.begin(); unit!=block.end(); unit++) {
    for (tile=unit->begin(); tile!=unit->end(); tile++) {
      printf("Processing tile %s\r", tile->Name());
      
      // Read tile points
      if (!tile->Read(tile->PointFile(), false)) {
        printf("Error reading points from file %s\n", tile->PointFile());
        exit(0);
      }
  
      // Determine all segment numbers
      tile->LongSegmentNumbers(segment_numbers);

      // Create a point set for every segment number
      for (segment_number_iterator=segment_numbers.begin();
	       segment_number_iterator!=segment_numbers.end(); 
		   segment_number_iterator++)
	    point_sets.push_back(LaserPoints());
	    
      // Distribute all points over the point sets
      for (point=tile->begin(); point!=tile->end(); point++) {
      	if (!point->HasAttribute(SegmentNumberTag)) continue;
      	segment_number = point->LongSegmentNumber();
      	segment_number_iterator = std::find(segment_numbers.begin(),
		                                    segment_numbers.end(), segment_number);
		index = std::distance(segment_numbers.begin(), segment_number_iterator);
		point_set = point_sets.begin() + index;
		point_set->push_back(*point);
      }
	  
	  // Store all points in files per segment
      for (point_set=point_sets.begin(),
	       segment_number_iterator=segment_numbers.begin();
		   point_set!=point_sets.end(); 
		   point_set++, segment_number_iterator++) {
		   	
		// Construct file name
		sprintf(segment_name, "%lld", *segment_number_iterator);
		if (args->Contains("-odir"))
		  file_name = ComposeFileName(root_dir, segment_name, ".txt");
		else
		  file_name = ComposeFileName("./", segment_name, ".txt");
		
		// Write all points
		segment_file = fopen(file_name, "a");		
		for (point=point_set->begin(); point!=point_set->end(); point++)
		  fprintf(segment_file, "%.2f %.2f %.2f %d\n", point->X(), point->Y(),
		          point->Z(), point->Label());
	    fclose(segment_file);
	  }
	  
	  // Clear all data before processing the next tile
      segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
      tile->ErasePoints();
      for (point_set=point_sets.begin(); point_set!=point_sets.end(); 
		   point_set++)
		point_set->ErasePoints();
	  point_sets.erase(point_sets.begin(), point_sets.end());
    }
  }

  return EXIT_SUCCESS;
}
