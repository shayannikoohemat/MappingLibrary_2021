/*
    Copyright 2014 University of Twente
 
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
 Derivation of point attributes in a block of laser data

 Initial creation:
 Author : George Vosselman
 Date   : 24-05-2014

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/
;
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <malloc.h>
#include "LaserBlock.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                         The main pointattributes function
--------------------------------------------------------------------------------
*/

void pointattributes(char *meta_file, vector<LaserPointTag> &point_attributes,
                     char *dtm_file, char *grid_file,
					 char *parameter_file)
{
  LaserBlock                      block;
  LaserBlock::iterator            unit;
  LaserUnit::iterator             part;
  LaserPoints::iterator           point, nb_point;
  int                             fileclass, point_number;
  Image                           dtm;
  DataGrid                        grid;
  vector<LaserPointTag>::iterator point_attribute;
  ImagePoint                      image_point;
  float                           ground_height;
  SegmentationParameters          parameters;
  TINEdges                        *edges;
  TINEdges::iterator              nbh_nodes;
  PointNumberList::iterator       nbh_node;
  bool                            use_nbh_nodes=false, done;
  long long int                   segment_number;
  Plane                           plane;
    
  if (!block.Create(meta_file, &fileclass)) {
    fprintf(stderr, "Error reading file %s\n", meta_file);
    exit(0);
  }

  if (dtm_file) {
  	if (!dtm.Read(dtm_file)) {
  	  printf("Error reading DTM from file %s\n", dtm_file);
  	  exit(0);
  	}
  }  
  if (grid_file) {
  	if (!grid.Read(grid_file)) {
  	  printf("Error reading DTM grid from file %s\n", grid_file);
  	  exit(0);
  	}
  }
  if (parameter_file){
  	if (!parameters.Read(parameter_file)) {
  	  printf("Error reading segmentation parameters from file %s\n", parameter_file);
  	  exit(0);
  	}
  }
  
  // Determine whether a neighbourhood is needed
  if (std::find(point_attributes.begin(), point_attributes.end(),
		        NearOtherSegmentTag) != point_attributes.end()) {
	use_nbh_nodes = true;
  }
  if (std::find(point_attributes.begin(), point_attributes.end(),
		        NormalXTag) != point_attributes.end()) {
	use_nbh_nodes = true;
  }

  // Loop over all data sets
  for (unit=block.begin(); unit!=block.end(); unit++) {
  	for (part=unit->begin(); part!=unit->end(); part++) {
  		
  	  // Read the points
  	  if (!part->Read(part->PointFile(), false)) {
  	  	printf("Error reading points from file %s\n", part->PointFile());
  	  	exit(0);
  	  }
  	  printf("Processing part %s\r", part->Name());
  	  
  	  // Derive neighbourhoods if needed
  	  if (std::find(point_attributes.begin(), point_attributes.end(),
		            NearOtherSegmentTag) != point_attributes.end()) {
		use_nbh_nodes = true;
	  }
	  if (use_nbh_nodes) {
		if (parameters.SeedNeighbourhoodDefinition() == 0) // Direct edges
		  edges = part->DeriveEdges(parameters);
		else
		  edges = part->DeriveFDNEdges(parameters);
		nbh_nodes = edges->begin();
	  }
  	  
  	  // Loop over all points
	  for (point=part->begin(), point_number=0;
	       point!=part->end(); point++, point_number++) {

 	    // Loop over all attributes
  	    for (point_attribute=point_attributes.begin();
		     point_attribute!=point_attributes.end();
		     point_attribute++) {
  		  switch (*point_attribute) {
		    case HeightAboveGroundTag:
		  	  image_point = point->Map_Into_Image(0, grid.ImageGridReference());
              ground_height =
			    dtm.InterpolateFloat(image_point.Row(), image_point.Column(), 2);
              point->FloatAttribute(HeightAboveGroundTag) =
			    (float) point->Z() - ground_height;
		  	  break;
		  	
		    case NearOtherSegmentTag:
		      if (point->HasAttribute(SegmentNumberTag)) {
		  	    for (nbh_node=nbh_nodes->begin(), done=false;
				     nbh_node!=nbh_nodes->end() && !done; nbh_node++) {
		  	  	  nb_point = part->begin() + nbh_node->Number();
		  	  	  if (nb_point->HasAttribute(SegmentNumberTag)) {
		  	  	  	if (point->LongAttribute(LongSegmentNumberTag) !=
					    nb_point->LongAttribute(LongSegmentNumberTag)) {
					  point->Attribute(NearOtherSegmentTag) = 1;
					  done = true;
				    }
		  	  	  }
		  	    }
		  	    if (!done) point->Attribute(NearOtherSegmentTag) = 0;
		  	  }
		  	  else point->Attribute(NearOtherSegmentTag) = 0;
		  	  break;
		  	
		    case NormalXTag:
		      if (point->HasAttribute(SegmentNumberTag)) {
		      	segment_number = point->LongAttribute(LongSegmentNumberTag);
		      	plane.Initialise();
		  	    for (nbh_node=nbh_nodes->begin(); nbh_node!=nbh_nodes->end(); nbh_node++) {
		  	  	  nb_point = part->begin() + nbh_node->Number();
		  	  	  if (nb_point->HasAttribute(SegmentNumberTag)) {
		  	  	  	if (segment_number ==
					    nb_point->LongAttribute(LongSegmentNumberTag)) {
					  plane.AddPoint(nb_point->Position3DRef(), false);  	
				    }
		  	  	  }
		  	    }
		  	    if (plane.NumberOfPoints() > 2) {
		  	      plane.Recalculate();
		  	      point->SetNormal(plane.Normal());
		  	    }
		  	  }
		  	  break;
		  	
		    default:
		  	  break;
		  }
	    }
		if (use_nbh_nodes) nbh_nodes++;
	  }
	  
	  // Write the points
	  part->Write(part->PointFile(), 1, false);
	  
	  // Clean up
	  if (use_nbh_nodes) part->EraseNeighbourhoodEdges();
	  if (use_nbh_nodes) part->EraseTIN();
	  part->ErasePoints();
  	}
  }
}
