
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
 Segmentation of laser altimetry data based on attribute similarity.
 Files can be specified explicitly or by a file filter.
 The segmented points are written to file. Optionally, a meta data file is
 generated.

 Initial creation:
 Author : George Vosselman
 Date   : 16-07-2013

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <map>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <malloc.h>
#include <time.h>
#include "LaserBlock.h"
#include "TINEdges.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(const char *, const char *, int *);

/*
--------------------------------------------------------------------------------
                         The main majorityfiltering function
--------------------------------------------------------------------------------
*/

extern "C" void timer_start(clock_t *time1)
{
  *time1 = clock();
}

extern "C" void timer_end(clock_t time1, char *string)
{
  clock_t time2;
  time2 = clock();
  printf("Time used for %s: %5.2f minutes\n", string,
         (double) (time2 - time1) / (60 * CLOCKS_PER_SEC));
}

void MajorityFilterTile(LaserUnit::iterator tile,
                        LaserBlock::iterator tile_container, 
                        vector<int> tile_numbers,
                        const SegmentationParameters &parameters,
						TINEdges edges)
{
  int                               number_this_tile, tile_number, tile_index,
                                    highest_surface_this_tile, highest_surface,
                                    point_number, majority_value, count;
  vector<int>::iterator             start_tile;
  LaserPoints::iterator             point, nb_point;
  PointNumberList                   nbh_nodes;
  LaserPointTag                     tag = parameters.MajorityAttribute();
  bool                              unlabeled_only = parameters.MajorityNoAttributeOnly(),
                                    long_attribute, debug=false;
  long long int                     long_majority_value;
  map<long long int, int>           value_counters;
  map<long long int, int>::iterator value_counter;
  long long int                     value;
  PointNumberList::const_iterator   node;

  number_this_tile = tile->TileNumber();
  highest_surface_this_tile = tile->HighestSurfaceNumber();
  
  // Check attribute type
  switch (AttributeType(tag)) {
    case IntegerAttributeType: long_attribute = false; break;
    default:
    case FloatAttributeType:
    case DoubleAttributeType:
      printf("Error: Majority filtering is only available for integer attributes\n");
  	  exit(0);
  	case LongAttributeType: long_attribute = true; break;
  }
  
  for (point=tile->begin(), point_number=0; point!=tile->end();
       point++, point_number++) {
  	// Skip points that already have this attribute
  	if (unlabeled_only && point->HasAttribute(tag)) continue;
  	
  	// Get neighbourhood
  	nbh_nodes = tile->Neighbourhood(PointNumber(point_number),
                              parameters.MajorityNeighbourhoodRadius(), edges,
                              parameters.DistanceMetricDimension() == 2,
                              parameters.MajorityNeighbourhoodDefinition() == 0);

    // Set majority value for short attributes
    if (debug) printf("Long attribute? %d\n", (int) long_attribute);

    if (!long_attribute) {
      majority_value = tile->MostFrequentAttributeValue(tag, nbh_nodes, count,
	                                              tile->HighestSurfaceNumber() + 1);
      if (count > 0) point->Attribute(tag) = majority_value;
    }
    
    // A bit more work for the long segment number
    else if (tag == LongSegmentNumberTag) {
      // Collect the value counts
      count = 0;
      for (node=nbh_nodes.begin(); node!=nbh_nodes.end(); node++) {
  	    nb_point = tile->begin() + node->Number();
        if (nb_point->HasAttribute(tag)) {
          value = nb_point->LongAttribute(tag);
          // Determine the highest surface number for the start tile of this segment
          tile_number = value / 1000000;
          if (tile_number = number_this_tile) {
          	highest_surface = highest_surface_this_tile;
          }
          else {
          	// Look up tile
          	start_tile = std::find(tile_numbers.begin(), tile_numbers.end(),
          	                       tile_number);
          	if (start_tile == tile_numbers.end()) {
          	  printf("Error: No meta data for tile %d\n", tile_number);
          	  exit(0);
          	}
          	tile_index = std::distance(tile_numbers.begin(), start_tile);
          	highest_surface = (tile_container->begin() + tile_index)->HighestSurfaceNumber();
          }
          // Skip this point if it belongs to a surface
          if (value <= highest_surface) continue;
          // Otherwise count it
          value_counter = value_counters.find(value);
          if (value_counter == value_counters.end())
            value_counters.insert(pair<long long int, int>(value, 1));
          else
            value_counter->second++;
        }
      }
      // Determine the highest count
      for (value_counter=value_counters.begin(), count=0;
           value_counter!=value_counters.end(); value_counter++) {
        if (value_counter->second > count) {
          value = value_counter->first;
          count = value_counter->second;
        }
      }
      if (count > 0) point->SetLongAttribute(tag, value);
      if (debug) printf("mj nbh size %d, maj val %lld, count %d\n",
	                    nbh_nodes.size(), value, count);
	  // Clear counters
	  value_counters.erase(value_counters.begin(), value_counters.end());
    }
  }
}

void majorityfiltering(char *long_filter, char *infile,
                       char *appendix, char *output_directory,
                       bool output_meta_data, bool overwrite,
                       const SegmentationParameters &parameters)
{
  char                 *directory, *filter, *filename, *newname;
  int                  icon, fileclass, first_surface_number, num_tags, itag,
                       last_surface_number, first_segment_number;
  const unsigned char  *tag, *tolerance_tags;
  LaserBlock           block;
  LaserBlock::iterator unitptr;
  LaserUnit::iterator  subunitptr;
  LaserSubUnit         newsubunit;
  clock_t              start;
  TINEdges             *edges;
  vector<int>          tile_numbers;

  printf("begin of majorityfiltering\n");
  // Set up the file filter for the input file(s)
  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

  // Collect all points from all input files
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    // Set up a laser block
    if (!block.Create(filename, &fileclass)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }

    // Make a tile number list if surfaces should not be used for filtering
    if (block.begin()->DataOrganisation() == TileWise &&
		parameters.MajorityNoSurfaces()) {
	  for (subunitptr=block.begin()->begin();
	       subunitptr!=block.begin()->end(); subunitptr++)
		tile_numbers.push_back(subunitptr->TileNumber());		
	}

    // Loop over all units and subunits
    for (unitptr=block.begin(); unitptr!=block.end(); unitptr++) {
      for (subunitptr=unitptr->begin(); subunitptr!=unitptr->end();
	       subunitptr++) {

        // Derive the names of the output files. Note that the variable
        // newsubunit is only used for checking the existance of files. New file 
        // names are later transfered to subunitptr.
        if (!subunitptr->Name()) subunitptr->LaserDataFiles::DeriveName();
        if (appendix) {
          newname = (char *) malloc(strlen(subunitptr->Name()) +
                                    strlen(appendix) + 1);
          sprintf(newname, "%s%s", subunitptr->Name(), appendix);
        }
        else {
          newname = (char *) malloc(strlen(subunitptr->Name()) + 1);
          sprintf(newname, "%s", subunitptr->Name());
        }
        newsubunit.SetName(newname);
        newsubunit.DataOrganisation() = subunitptr->DataOrganisation();
        newsubunit.DeriveMetaDataFileName(output_directory);
        newsubunit.DerivePointFileName(output_directory);
        free(newname);

        // Do not go ahead if the point file and meta file already exists and 
        // overwriting these files is not permitted.
        if (overwrite || !FileExists(newsubunit.MetaDataFile()) ||
            !FileExists(newsubunit.PointFile())) {
        
          // Read the point data, currently ignoring adjacent tiles
          if (!subunitptr->Read()) {
            fprintf(stderr, "Error reading laser points from file %s\n",
                    subunitptr->PointFile());
            exit(0);
          }

          // After reading all data, the file names can be overwritten with the
          // names of the output files.
          subunitptr->SetName(newsubunit.Name());
          subunitptr->DeriveMetaDataFileName(output_directory);
          subunitptr->DerivePointFileName(output_directory);
          
          // Remove seek offset as all tiles are written separately
          subunitptr->SetSeekOffset(0);

          // Check if the selected attributes is available
          if (!subunitptr->HasAttribute(parameters.MajorityAttribute())) {
	        printf("Warning: Attribute %s is not available in tile %s.\n", 
	               AttributeName(parameters.MajorityAttribute(), false),
				   subunitptr->Name());
	      }
          
          // Otherwise filter
          else {
            // Check neighbourhood edges
            edges = subunitptr->VerifyEdges(parameters);
          
            // Filter the data
            timer_start(&start);
            if (block.begin()->DataOrganisation() == TileWise &&
		        parameters.MajorityNoSurfaces())
		      MajorityFilterTile(subunitptr, block.begin(), tile_numbers,
			                     parameters, edges->TINEdgesRef());
            else
              subunitptr->MajorityFilter(parameters, edges->TINEdgesRef());
            timer_end(start, (char *)"filtering");
          }
          
          // Write the points and meta data
          subunitptr->Write();
          if (output_meta_data) subunitptr->WriteMetaData();

          // Erase the points, the TIN, and the TIN edges
          subunitptr->ErasePoints();
          subunitptr->EraseTIN();
          subunitptr->EraseNeighbourhoodEdges();
        }
        else {
          printf("Sub unit %s was already done.\n", newsubunit.Name());
          if (!subunitptr->ReadMetaData(newsubunit.MetaDataFile())) {
            fprintf(stderr, "Error reading meta data from file %s\n",
                    newsubunit.MetaDataFile());
            exit(0);
          }
        }
      }

      // Output of meta data at unit level
      if (output_meta_data &&
          (fileclass == LASER_STRIP || fileclass == LASER_BLOCK) &&
          unitptr->DataOrganisation() & StripWise) {
        if (!unitptr->Name()) unitptr->LaserDataFiles::DeriveName();
        if (appendix) {
          newname = (char *) malloc(strlen(unitptr->Name()) +
                                    strlen(appendix) + 1);
          sprintf(newname, "%s%s", unitptr->Name(), appendix);
        }
        else {
          newname = (char *) malloc(strlen(unitptr->Name()) + 1);
          sprintf(newname, "%s", unitptr->Name());
        }
        unitptr->SetName(newname);
        free(newname);
        unitptr->DeriveMetaDataFileName(output_directory);
        if (unitptr->begin()->IsCompleteStrip())
          unitptr->DerivePointFileName(output_directory);
        else unitptr->SetPointFile(NULL);
        unitptr->WriteMetaData();
      }
    }

    // Output of meta data at block level
    if (output_meta_data && fileclass == LASER_BLOCK) {
      if (!block.Name()) block.DeriveName();
      if (appendix) {
        newname = (char *) malloc(strlen(block.Name()) +
                                  strlen(appendix) + 1);
        sprintf(newname, "%s%s", block.Name(), appendix);
      }
      else {
        newname = (char *) malloc(strlen(block.Name()) + 1);
        sprintf(newname, "%s", block.Name());
      }
      block.SetName(newname);
      free(newname);
      block.DeriveMetaDataFileName(output_directory);
      block.WriteMetaData();
    }
  }
}
