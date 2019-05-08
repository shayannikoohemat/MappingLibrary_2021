
/*
                      Copyright 2010 University of Twente 
    
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
 Creation of road part outlines

 Initial creation:
 Author : George Vosselman
 Date   : 28-12-2009

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include "Road.h"
#include "LaserBlock.h"

/*
--------------------------------------------------------------------------------
                         The main createroad function
--------------------------------------------------------------------------------
*/

void createroad_cpp(char *gps_track_file, char *gpsl_track_file,
                    char *road_name,
                    double road_part_length,
                    double road_width, double road_part_overlap,
                    double road_outline_step_size, char *block_file,
                    char *output_directory,
                    bool output_laser_points, bool clip_at_outline_bounds,
                    char *road_points_file, char *road_tops_file)
{
  Positions3D    gps_track;
  LaserPoints    gps_laser_points;
  LaserPoints::iterator gps_laser_point;
  Road           road;
  Road::iterator road_part;
  ObjectPoints   road_part_points;
  LineTopologies road_part_tops;
  int            num_pts, status;
  LaserBlock     block;
  
  // Read GPS track
  if (gps_track_file) {
    if (!gps_track.Read(gps_track_file)) {
      printf("Error reading GPS track from %s\n", gps_track_file);
      exit(0);
    }
  }
  else {
    if (!gps_laser_points.Read(gpsl_track_file)) {
      printf("Error reading GPS track from %s\n", gpsl_track_file);
      exit(0);
    }
    for (gps_laser_point=gps_laser_points.begin();
         gps_laser_point!=gps_laser_points.end(); gps_laser_point++)
      gps_track.push_back(gps_laser_point->Position3DRef());
    gps_laser_points.ErasePoints();
  }
  
  // Set road meta data
  road.SetName(road_name);
  road.DeriveMetaDataFileName(output_directory);
  
  // Generate road part outlines
  status = road.InitialiseRoadParts(gps_track, road_part_length, road_width,
                                    road_part_overlap, road_outline_step_size,
                                    output_directory);
  printf("Initialised road with %d parts\n", road.size());
  
  if (status < 0) {
    printf("Error status in Road::InitialiseRoadParts: %d\n", status);
    exit(0);
  }
  
  // Select tiles of each road part
  if (block_file) {
    // Read meta data
    if (!block.ReadMetaData(block_file)) {
      printf("Error reading block meta data file %s\n", block_file);
      exit(0);
    }
    if (block.empty()) {
      printf("Block %s contains no tiles\n", block.Name());
      exit(0);
    }
    if (block.begin()->empty()) {
      printf("Block %s contains no tiles\n", block.Name());
      exit(0);
    }
    if (block.begin()->begin()->DataOrganisation() != TileWise) {
      printf("Error: Block %s is not organised tile wise\n", block.Name());
      exit(0);
    }
    printf("Read meta data of %d tiles of block\n", block.begin()->size(), block.Name());
    for (road_part=road.begin(); road_part!=road.end(); road_part++) {
      // Collect tiles for all road parts
      road_part->CollectTiles(block, true);
      printf("Road part %s has data in %d tiles\n", road_part->Name(), road_part->Tiles().size());
      if (output_laser_points) {
        road_part->DeriveLaserPointsFileName(output_directory);
        num_pts = road_part->CollectLaserPoints(true, clip_at_outline_bounds);
        printf("%d points selected from the tiles\n", num_pts);
        road_part->WriteLaserPoints();
        road_part->EraseLaserPoints();
      }
    }
  }
 
  // Save meta data files
  road.WriteMetaData(true);
   
  // Write all road part outlines
  road.WriteRoadPartOutlines();
  
  // Write road part outlines collected in one point file and one topology file
  if (road_points_file && road_tops_file) {
    road.CollectRoadPartOutlines(road_part_points, road_part_tops, true);
    road_part_points.Write(road_points_file);
    road_part_tops.Write(road_tops_file);
  }
}
