
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
 Author : Sander Oude Elberink
 Date   : 01-10-2006

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
#include "LaserPoints.h"
#include "LineTopsIterVector.h"

/*
--------------------------------------------------------------------------------
                           The main function
--------------------------------------------------------------------------------
*/

void matchmapandlaser(char *laser_input, char *map_points_input,
	       char *map_topology_input, char *tagged_laser_points_output)

{
  LaserPoints                  laser_points;
  LaserPoints::iterator        laser_point;
  ObjectPoints                 map_points;
  LineTopologies               map_lines;
  LineTopologies::iterator     map_line, last_map_line;
  int                          count, success, index, pol_num,
                               iter, index1, index2;
  bool                         found, done;
  vector<int>                  point_count;
  bool debug=false;
  DataBounds3D                 bounds;
  
  // Local constants to be made global
  bool   force_polygon_check   = true;
  
  // Read input data
  if (!laser_points.Read(laser_input)) {
    printf("Error reading laser points from file %s\n", laser_input);
    exit(0);
  }
  if (!map_points.Read(map_points_input)) {
    printf("Error reading map points from file %s\n", map_points_input);
    exit(0);
  }
  if (!map_lines.Read(map_topology_input)) {
    printf("Error reading map lines from file %s\n", map_topology_input);
    exit(0);
  }
  
  // Check if the laser points have the required attributes
  if (!laser_points.begin()->HasAttribute(SegmentNumberTag)) {
    printf("Error: Matchmapandlaser requires laser points with SegmentNumberTags\n");
    return;
  }
  
  // Remove invisible polygons
  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (map_line->TOP10Invisible()) {
      map_lines.erase(map_line);
      map_line--;
      count++;
    }
  }
  printf("Removed %d invisible polygons\n", count);
  
    map_points.RemoveDoublePoints(map_lines, 0.01);
  
  
  // Add polygon numbers and labels to the points if they are not yet
  // available
  if (!laser_points.HasAttribute(PolygonNumberTag) ||
      !laser_points.HasAttribute(LabelTag) ||
      force_polygon_check) {
    printf("Match map and laser derives PolygonNumberTags and LabelTags.\n");
    printf("Point    Perc. Outside\n");
    // Remove old polygon numbers and labels
    if (force_polygon_check) {
      laser_points.RemoveAttribute(PolygonNumberTag);
      laser_points.RemoveAttribute(LabelTag);
    }
    point_count.resize(map_lines.size());
    for (index2=0; index2<map_lines.size(); index2++) point_count[index2] = 0;
    last_map_line = map_lines.begin();
    index2 = 0;
    for (laser_point=laser_points.begin(), index1=0, count=0;
         laser_point!=laser_points.end(); laser_point++, index1++) {
      // First check if the point is inside the map line of the previous point
      if (laser_point->InsidePolygon(map_points,
                                     last_map_line->LineTopologyReference()))
        found = true;
      else { // Check all map lines
        for (map_line=map_lines.begin(), found = false, index2=0;
             map_line!=map_lines.end() && !found; map_line++, index2++) {
          if (laser_point->InsidePolygon(map_points,
                                         map_line->LineTopologyReference())) {
            found = true;
            last_map_line = map_line;
          }
        }
      }
      if (found) {
        laser_point->Attribute(PolygonNumberTag) = last_map_line->Number();
        laser_point->Attribute(LabelTag)         = last_map_line->Label();
        point_count[index2]++;
      }
      else count++;
      if (index1 == (index1/100)*100) {
        printf("%7d  %5.1f %7d\r", index1, 100.0 * index1 / laser_points.size(),
               count);
        fflush(stdout);
      }
    }
    // Second loop for polygons without polygon number. These may be holes.
    printf("\nSecond loop for holes\n");
    for (map_line=map_lines.begin(), index2=0;
         map_line!=map_lines.end(); map_line++, index2++) {
      if (point_count[index2] == 0) {
        count = 0;
        bounds = map_line->Bounds(map_points);
        for (laser_point=laser_points.begin(); laser_point!=laser_points.end();
             laser_point++) {
          // Only check for a specific polygon number
          if (count == 0 || laser_point->Attribute(PolygonNumberTag) == pol_num) {
            // First check rectangle bounds
            if (bounds.InsideXY(laser_point->Position3DRef())) {
              // Finally check polygon bounds
              if (laser_point->InsidePolygon(map_points,
                                             map_line->LineTopologyReference())) {
                if (count == 0) pol_num = laser_point->Attribute(PolygonNumberTag);
                laser_point->Attribute(PolygonNumberTag) = map_line->Number();
                laser_point->Attribute(LabelTag)         = map_line->Label();
                count++;
              }
            }
          }
        }
        printf("%7d %7d\r", index2, count); fflush(stdout);
      }
    }
    printf("\nDone with deriving PolygonNumberTags and LabelTags.\n");
    printf("These tags will be saved to the output file\n");
    laser_points.Write(tagged_laser_points_output, false);
  }
}
