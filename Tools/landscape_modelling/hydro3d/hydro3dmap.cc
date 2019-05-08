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
 Date   : 11.10.2011

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/


#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "LaserPoints.h"
#include "LineTopsIterVector.h"
#include "VRML_io.h"
#include "dxf.h"

/*
--------------------------------------------------------------------------------
                           The main function
--------------------------------------------------------------------------------
*/

void Hydro3DMap(char *laser_input, char *map_points_input,
	       char *map_topology_input, char *map_points_output,
	       char *map_topology_output, double sample_distance, bool only_buildings,
           double min_height_difference, double min_height_difference_roads)

{
  LaserPoints                  laser_points, sel_laser_points, meadow_laser_points,
                               quality_points, poly_laser_points;
  LaserPoints::iterator        laser_point, nearest_point;
  LaserPoint                   nb_laser_point;
  TINEdges                     edges, seledges, poledges;
  ObjectPoints                 map_points, dtm_tin_points, clean_map_points;
  LineTopologies               map_lines, dtm_tin_lines;
  ObjectPoints::iterator       map_point, previous_point, next_point;
  ObjectPoint                  new_map_point;
  LineTopologies::iterator     map_line, last_map_line;
  LineTopsIterVector           sel_map_lines;
  LineTopsIterVector::iterator sel_map_line, sel_map_line2;
  PointNumberList              neighbourhood, road_nbh;
  PointNumberList::iterator    node, previous_node, next_node;
  PointNumber                  *node_ptr;
  int                          nearest_laser_point, dominant_segment, count,
                               success, index1, index2, index3, close_index1,
                               close_index2, next_point_number, index, pol_num,
                               num_2d_points, index_road, index_non_road,
                               top10class, iter;
  Plane                        plane;
  Planes                       planes;
  Line3D                       line;
  vector<double>               heights, height1, height2;
  vector<int>                  use_map_line, use_map_point, point_count;
  double                       diff, min_diff, new_height, dist1, dist2,
                               nbh_radius, min_height;
  bool                         found, done, emptymap;
  DataBounds3D                 bounds;
  TIN                          dtmtin;
  Position3D                   p1, p2, p3;
  Position2D                   pos;
  LineTopology::iterator       nb_node, node2;
  bool debug=true;
  FILE                         *fd, *dxffile;
  // Local constants to be made global
  //double min_height_difference = 0.5; //1.0//pcp1.5
//  double min_height_difference_roads = 0.03; //just see what happens if the threshold is small
  
  double other_nbh_radius      = 1.0;
  double road_nbh_radius       = 1.0;//25 pcp10
  int    min_num_hough_points  = 10; // Was 20//pcp30
  bool   force_polygon_check   = true;
  double max_dist_point_line   = 10.0; // Distance from map point to intersection line
  double max_road_slope        = 0.1;
  Image                        kernel;
  kernel.Read("kernel.xv");
  
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
    printf("Warning: Make3DMap requires laser points with SegmentNumberTags\n");
    printf("Warning: Setting all segmentvalues at 0\n");
    for (laser_point=laser_points.begin(), index1=0;
         laser_point!=laser_points.end(); laser_point++, index1++) {
         printf("%5.1f \r", 100.0 * index1 / laser_points.size());
        if (!laser_point->HasAttribute(SegmentNumberTag)){
          laser_point->Attribute(SegmentNumberTag) = 0;
        }
        
    }  
//    return;
  }
  
  bool invisible_polygons;
  invisible_polygons = false;
  if (invisible_polygons){
  // Remove invisible polygons
  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (map_line->TOP10Invisible()) {
//    if (map_line->TOP10MajorClass() != TOP10_Road) {
      map_lines.erase(map_line);
      map_line--;
      count++;
    }
  }
  printf("Removed %d invisible polygons\n", count);
 }
for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
 //   if (map_line->TOP10Invisible()) {
    if (map_line->TOP10MajorClass() != TOP10_Road) {
//    if (map_line->Attribute(IDNumberTag)!=13 && map_line->Attribute(IDNumberTag)!=30 && map_line->Attribute(IDNumberTag)!=54 ){
//      map_lines.erase(map_line);
 //     map_line--;
 //     count++;
    }
  }
  
  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (!map_line->IsClosed()) {
      map_lines.erase(map_line);
      map_line--;
      count++;
    }
  }
  printf("Removed %d not closed polygons\n", count);
/*
 for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (map_line->CalculateArea(map_points) < 0.5) {
      map_lines.erase(map_line);
      map_line--;
      count++;
    }
  }
  printf("Removed %d polygons smaller than 0.5 m2\n", count);

*/
// printf("Reducing laser points such that there's 1 point per %4.2f\n", sample_distance/3);
 
 //laser_points.ReduceData(sample_distance/3);
 
  // Derive the TIN edges of the laser points
  printf("Deriving TIN ..."); fflush(stdout);
  laser_points.DeriveTIN();
  printf("done\nDeriving TIN edges ..."); fflush(stdout);
  edges.Derive(laser_points.TINReference());
  
  // Densify the map points
  printf("done\nDensifying map points ..."); fflush(stdout);
  count = map_points.size();
  map_lines.Densify(map_points, sample_distance); 
  printf("done, map points increased from %d to %d\n",
         count, map_points.size());
  // Merge points within 0.01 m
  printf("Removing double points ..."); fflush(stdout);
  map_points.RemoveDoublePoints(map_lines, 0.01);
  printf("done, %d map points left\n", map_points.size());
  
  
  // Determine heights for all map points
  num_2d_points = map_points.size();
  for (map_point=map_points.begin(), index=0;
       index < num_2d_points; map_point++, index++) {
   
 
    if (debug)
      printf("Debugging point %d, index %d\n", map_point->Number(), index);
    
    // Determine all polygons to which this map point belongs
    sel_map_lines.Clear();
    for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
      if (map_line->Contains(map_point->NumberRef())) {
        sel_map_lines.push_back(map_line);
      }
    }
    if (debug)
      printf("number of sel map lines %d\n", sel_map_lines.size());
    
    // Skip this point if it is not in a line
    if (sel_map_lines.empty()) continue;
    // Find nearest laser point in 2D
 //   nearest_laser_point = laser_points.NearestPoint(map_point->Position3DRef(),
 //                                                   edges, true);
    // Determine the height in all polygons
    if (!heights.empty()) heights.erase(heights.begin(), heights.end());
    if (!planes.empty()) planes.erase(planes.begin(), planes.end());
    for (sel_map_line=sel_map_lines.begin(); sel_map_line!=sel_map_lines.end();
         sel_map_line++) {
      // Select the laser points inside the polygon
      sel_laser_points.ErasePoints();
      emptymap = false;
      poly_laser_points.ErasePoints();
      poly_laser_points.AddTaggedPoints(laser_points,
                                         (*sel_map_line)->Attribute(BuildingNumberTag),
                                         PolygonNumberTag);
      if (poly_laser_points.size()<10) emptymap = true;
      
      top10class = (*sel_map_line)->TOP10MajorClass();
      if (debug) printf("top10class %d\n", top10class);

      if (top10class == TOP10_Road) nbh_radius = road_nbh_radius;
      else nbh_radius = other_nbh_radius;
      // For water and buildings just get all points inside the polygon
      if (top10class == TOP10_Water) {
        if (debug) printf("water, line number %d\n", (*sel_map_line)->Number());
        sel_laser_points = poly_laser_points;
        if (debug) printf("sel laser points %d\n", sel_laser_points.size());
      }
      if (top10class == TOP10_Building || emptymap){ //look for nearby laser points (note, these are on DTM surface.)
        nearest_laser_point = laser_points.NearestPoint(map_point->Position3DRef(),
                                                    edges, true);
        iter = 0; done = false;
        
        do {
          iter++;
          printf("iter %d ", iter);
          neighbourhood.erase(neighbourhood.begin(), neighbourhood.end());
          neighbourhood = 
            laser_points.Neighbourhood(PointNumber(nearest_laser_point),
                                             nbh_radius, edges,
                                             true, false);
          if (neighbourhood.size() < min_num_hough_points) {
            nbh_radius *= 2.0;
          }
          else done = true;
        } while (!done && iter < 20);
        if (debug)
          printf("Region %d pts, radius %5.2f\n", neighbourhood.size(),
                 nbh_radius);
        // Create a set of laser points of this neighbourhood
        sel_laser_points.ErasePoints();
        for (node=neighbourhood.begin(); node!=neighbourhood.end(); node++)
          sel_laser_points.push_back(laser_points[node->Number()]);
      }
      // For other surfaces, get a minimum set of points
      if ((top10class == TOP10_Meadow || top10class == TOP10_Road) && !emptymap) {
           printf("poly ");
        
        poledges.erase(poledges.begin(), poledges.end());
        poly_laser_points.DeriveTIN();
           printf("edges (%d)", poly_laser_points.size());

        poledges.Derive(poly_laser_points.TINReference());
        printf("nearest ");

        nearest_laser_point = poly_laser_points.NearestPoint(map_point->Position3DRef(),
                                                    poledges, true);
        
        iter = 0; done = false;
        
        printf("other surface ");
        
        do {
          iter++;
          printf("iter %d ", iter);
          neighbourhood.erase(neighbourhood.begin(), neighbourhood.end());
          neighbourhood = 
            poly_laser_points.Neighbourhood(PointNumber(nearest_laser_point),
                                             nbh_radius, poledges,
                                             true, false);
          if (neighbourhood.size() < min_num_hough_points) {
            nbh_radius *= 2.0;
          }
          else done = true;
        } while (!done && iter < 20);
        if (debug)
          printf("Region %d pts, radius %5.2f\n", neighbourhood.size(),
                 nbh_radius);
        // Create a set of laser points of this neighbourhood
           sel_laser_points.ErasePoints();
           for (node=neighbourhood.begin(); node!=neighbourhood.end(); node++)
               sel_laser_points.push_back(poly_laser_points[node->Number()]);
        }
        sel_laser_points.DeriveDataBounds(0);
        
        // For water, just take the lowest point
        if (top10class == TOP10_Water){
          if (sel_laser_points.empty()) {
            if (debug) printf("no points ");
            heights.push_back(-100.0);
            planes.push_back(Plane());
          }
          else {
            if (debug) printf(" points ");
            heights.push_back(sel_laser_points.DataBounds().Minimum().Z());
            planes.push_back(Plane(Vector3D(0.0, 0.0, *(heights.end()-1)),
                                 Vector3D(0.0, 0.0, 1.0)));
          }
          if (debug) printf("plane pushed back ");
        }
        if (top10class == TOP10_Building){       
          heights.push_back(sel_laser_points.Mean().Z());
          planes.push_back(Plane(Vector3D(0.0, 0.0, *(heights.end()-1)),
                                 Vector3D(0.0, 0.0, 1.0)));
          }
      // For other classes, determine dominant plane by fitting to points
      // with the most frequent segment number 
        if (top10class == TOP10_Meadow || top10class == TOP10_Road) {
           dominant_segment =
           sel_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
           if (debug) printf("sel_laser_points %d COUNT %d, dominant segment: %d ", sel_laser_points.size(), count, dominant_segment);
           if (count >= 8) { 
              plane = sel_laser_points.FitPlane(dominant_segment, dominant_segment, SegmentNumberTag);
              if (fabs(plane.Z_At(map_point->X(), map_point->Y(), &success) - sel_laser_points.Mean().Z()) < 1){
                  heights.push_back(plane.Z_At(map_point->X(), map_point->Y(), &success));
                  planes.push_back(plane);              
                  }
              else {
                heights.push_back(sel_laser_points.Mean().Z());
                planes.push_back(Plane(Vector3D(0.0, 0.0, *(heights.end()-1)),
                                 Vector3D(0.0, 0.0, 1.0)));
                }
              if (debug) printf("X,Y %4.2f, %4.2f,X,Y %4.2f, %4.2f, height = %4.2f, meanheight %4.2f\n", map_point->X(), map_point->Y(), sel_laser_points.Mean().X(),sel_laser_points.Mean().Y(), plane.Z_At(map_point->X(), map_point->Y(), &success), sel_laser_points.Mean().Z());
           }
           else {
            if (top10class == TOP10_Meadow && count > 0) { //meadow and a few points, take average of greatest segment
              meadow_laser_points.ErasePoints();
              meadow_laser_points.AddTaggedPoints(sel_laser_points, dominant_segment, SegmentNumberTag);
              heights.push_back(meadow_laser_points.Mean().Z());
              if (debug) printf("MEAN height = %4.2f\n", meadow_laser_points.Mean().Z());
              planes.push_back(Plane(Vector3D(0.0, 0.0, *(heights.end()-1)),
                                 Vector3D(0.0, 0.0, 1.0)));                                             
            }
            else {
              heights.push_back(-100.0);
              planes.push_back(Plane());
            }
          }
      }
    }
    
    // Initially, all lines should use their own determined height
    use_map_line.resize(heights.size());
    for (index1=0; index1<heights.size(); index1++)
      use_map_line[index1] = index1;
  
    for (sel_map_line=sel_map_lines.begin(), index1=0;
         sel_map_line!=sel_map_lines.end(); sel_map_line++, index1++) {
         printf("map line %d, label %d, height %4.2f, usemaplineindex %d\n",
                     (*sel_map_line)->Number(), (*sel_map_line)->Attribute(LineLabelTag),
                     heights[index1], use_map_line[index1]);
    }
      
    // If one of the lines is a water polygon, take this height for adjacent
    // meadows.
    for (sel_map_line=sel_map_lines.begin(), index1=0;
         sel_map_line!=sel_map_lines.end(); sel_map_line++, index1++) {
      if ((*sel_map_line)->TOP10MajorClass() == TOP10_Water) {
        for (sel_map_line2=sel_map_lines.begin(), index2=0;
             sel_map_line2!=sel_map_lines.end(); sel_map_line2++, index2++) {
          if ((*sel_map_line2)->TOP10MajorClass() == TOP10_Meadow)
            use_map_line[index2] = index1;
        }
      }
    }
    
    // Combine heights if they are within a specified range
    if (sel_map_lines.size()>1){
    do {                                  
    if (debug) printf("start while loop with %d heights\n", heights.size());
      // Determine the two nearest heights
      for (index1=0, min_diff=1e10; index1<heights.size()-1; index1++) {
        if (use_map_line[index1] == index1 && heights[index1] > -100.0) { 
          for (index2=index1+1; index2<heights.size(); index2++) {
            if (use_map_line[index2] == index2 &&
                heights[index2] > -100.0) {
              diff = fabs(heights[index1] - heights[index2]);
              if (diff < min_diff) {
                close_index1 = index1;
                close_index2 = index2;
                min_diff = diff;
                printf("Label 1 = %d, index1 = %d\n", sel_map_lines[close_index1]->Attribute(LineLabelTag), index1);
                printf("Label 2 = %d, index2 = %d\n", sel_map_lines[close_index2]->Attribute(LineLabelTag), index2);
              }
            }
          }
        }
      }
      if (debug) printf("Height difference = %5.2f, close_index1 = %d close_index2 = %d\n", min_diff, close_index1,close_index2);
      if (min_diff > min_height_difference && min_diff < 3* min_height_difference &&
          sel_map_lines[close_index2]->TOP10MajorClass() == TOP10_Meadow 
       && sel_map_lines[close_index1]->TOP10MajorClass() == TOP10_Meadow){
         if (debug) printf("Merging two terrain segments \n");
         heights[close_index1] = (heights[close_index1] +
                                  heights[close_index2]) / 2.0;
  //       heights[close_index2] = heights[close_index1];
         
         if (debug) printf("Merging two other segments\n");
         for (index1=0; index1<=heights.size(); index1++)
           if (use_map_line[index1] == close_index2)
             use_map_line[index1] = close_index1;

           nb_laser_point.X() = map_point->X();
           nb_laser_point.Y() = map_point->Y();
           nb_laser_point.Z() = heights[close_index1];
           nb_laser_point.SetAttribute(LabelTag, 10);                               
           quality_points.push_back(nb_laser_point);              
      }
      
      if ((min_diff < min_height_difference)
    //               || ((sel_map_lines[close_index2]->TOP10MajorClass() == TOP10_Meadow 
    //                   && sel_map_lines[close_index1]->TOP10MajorClass() == TOP10_Meadow)
    //                   && min_diff < 2*min_height_difference)
         ) { //if close or both meadow
                                                            
        if (sel_map_lines[close_index2]->TOP10MajorClass() == TOP10_Water 
                     || sel_map_lines[close_index1]->TOP10MajorClass() == TOP10_Water){ // if one or both water, get out of loop
                     if (debug) printf("one or both is water, keep both \n");
                     min_diff = min_height_difference +1;
                     heights[close_index1] = heights[close_index1];
                     heights[close_index2] = heights[close_index2];
                     }
          else{
          if (debug) printf("Height difference %5.2f\n", min_diff);
        // Merge heights depending on polygon classification
          if (sel_map_lines[close_index1]->TOP10MajorClass() == TOP10_Road) {
            if (sel_map_lines[close_index2]->TOP10MajorClass() == TOP10_Road) {
            if (min_diff < min_height_difference_roads){
              // Two road segments && very close, just take average
              heights[close_index1] = (heights[close_index1] + 
                                       heights[close_index2]) / 2.0;
  //            heights[close_index2] = heights[close_index1];

              nb_laser_point.X() = map_point->X();
              nb_laser_point.Y() = map_point->Y();
              nb_laser_point.Z() = heights[close_index1];
//            nb_laser_point.SetPointNumber(map_point->Number());
              nb_laser_point.SetAttribute(LabelTag, 4);                               
              quality_points.push_back(nb_laser_point);
              if (debug) printf("Merging two road segments\n");
             for (index1=0; index1<=heights.size(); index1++)
                if (use_map_line[index1] == close_index2)
                   use_map_line[index1] = close_index1;
             }
             else {
                  if (sel_map_lines[close_index1]->Attribute(LineLabelTag) == sel_map_lines[close_index2]->Attribute(LineLabelTag) && min_diff<0.3+min_height_difference_roads){ //if same function allow for higher minheightdiff
                  heights[close_index1] = (heights[close_index1] + 
                                       heights[close_index2]) / 2.0;
   //               heights[close_index2] = heights[close_index1];

              nb_laser_point.X() = map_point->X();
              nb_laser_point.Y() = map_point->Y();
              nb_laser_point.Z() = heights[close_index1];
//            nb_laser_point.SetPointNumber(map_point->Number());
              nb_laser_point.SetAttribute(LabelTag, 5);                               
              quality_points.push_back(nb_laser_point);
              if (debug) printf("Merging two road segments with same function\n");
                for (index1=0; index1<=heights.size(); index1++)
                if (use_map_line[index1] == close_index2)
                   use_map_line[index1] = close_index1;     
                  }
                  else {
                    min_diff = min_height_difference +1;
                    heights[close_index1] = heights[close_index1];
                   heights[close_index2] = heights[close_index2];
                   
                  }
                  }
           }
          else {
            // Road segment with other segment, keep road height
            if (debug) printf("Merging road and other segment, %d %d\n", heights.size(), use_map_line.size());
            for (index1=0; index1<=heights.size(); index1++)
              if (use_map_line[index1] == close_index2)
                use_map_line[index1] = close_index1;
            if (debug) printf("middle of merging road and other segment\n");

                nb_laser_point.X() = map_point->X();
            nb_laser_point.Y() = map_point->Y();
            nb_laser_point.Z() = (heights[close_index1] + heights[close_index2]) / 2.0;
//            nb_laser_point.SetPointNumber(map_point->Number());
            nb_laser_point.SetAttribute(LabelTag, 5);                               
            quality_points.push_back(nb_laser_point);
            if (debug) printf("end of merging road and other segment\n");

          }
        printf("before else loop\n");
        }
        else { //if the first wasn't road
          if (debug) printf(" first wasn't road\n");
          if (sel_map_lines[close_index2]->TOP10MajorClass() == TOP10_Road) {
            // Other segment with road segment, keep road height
            if (debug) printf("Merging other and road segment\n");
            for (index1=0; index1<=heights.size(); index1++)
              if (use_map_line[index1] == close_index1)
                use_map_line[index1] = close_index2;
          }
         else { // second not road either
              if (debug) printf("both not road\n");
                  if (sel_map_lines[close_index2]->TOP10MajorClass() == TOP10_Meadow 
                       && sel_map_lines[close_index1]->TOP10MajorClass() == TOP10_Meadow){ // if both meadow, use triangulation of laser points to determine heigth
                      nb_laser_point.X()=map_point->X();
                      nb_laser_point.Y()=map_point->Y();
                      nb_laser_point.Z() = (heights[close_index1] + heights[close_index2]) / 2.0; 
                      nb_laser_point.SetAttribute(LabelTag, 2);      
                      quality_points.push_back(nb_laser_point);
                      // Two other segments, just take average
                      heights[close_index1] = (heights[close_index1] +
                                     heights[close_index2]) / 2.0;
    //                  heights[close_index2] = heights[close_index1];

                      for (index1=0; index1<=heights.size(); index1++)
                           if (use_map_line[index1] == close_index2)
                               use_map_line[index1] = close_index1;
                   }
          else {
            if (debug) printf("Merging two other segments\n");
            // Two other segments, just take average
            heights[close_index1] = (heights[close_index1] +
                                     heights[close_index2]) / 2.0;
  //          heights[close_index2] = heights[close_index1];

            if (debug) printf("Merging two other segments\n");
            for (index1=0; index1<=heights.size(); index1++)
              if (use_map_line[index1] == close_index2)
                use_map_line[index1] = close_index1;
                nb_laser_point.X() = map_point->X();
                nb_laser_point.Y() = map_point->Y();
                nb_laser_point.Z() = heights[close_index1];
                nb_laser_point.SetAttribute(LabelTag, 6);                               
                quality_points.push_back(nb_laser_point);
          }
        }
        if (debug) printf("go here?\n");
        }
        if (debug) printf("go here\n");
        }
        if (debug) printf("or go here\n");
      } 
     if (debug) printf("end of while loop %4.2f < %4.2f \n", min_diff, min_height_difference);
    } while (min_diff < min_height_difference);
    }
    printf("/nAnd so..:/n");
    for (sel_map_line=sel_map_lines.begin(), index1=0;
         sel_map_line!=sel_map_lines.end(); sel_map_line++, index1++) {
         printf("map line %d, label %d, height %4.2f, usemaplineindex %d\n",
                     (*sel_map_line)->Number(), (*sel_map_line)->Attribute(LineLabelTag),
                     heights[index1], use_map_line[index1]);
    }
    // Combine heights on a road side if the non-road surface is intersecting
    // near the map point.
/*    do {
    if (debug) printf("second while loop\n");
      found = false;
      for (index1=0, min_diff=1e10; index1<heights.size()-1; index1++) {
        if (use_map_line[index1] == index1 && heights[index1] > -100.0) {
          for (index2=index1+1; index2<heights.size(); index2++) {
            if (use_map_line[index2] == index2 &&
                heights[index2] > -100.0) {
              // Check if this is a combination of a road and non-road line
              index_road = index_non_road = -1;
              if (sel_map_lines[index1]->TOP10MajorClass() == TOP10_Road)
                index_road = index1;
              else
                if (sel_map_lines[index1]->TOP10MajorClass() == TOP10_Meadow) index_non_road = index1; 
              if (sel_map_lines[index2]->TOP10MajorClass() == TOP10_Road)
                index_road = index2;
              else
                if (sel_map_lines[index1]->TOP10MajorClass() == TOP10_Meadow) index_non_road = index1; 
              if (index_road != -1 && index_non_road != -1) {
                if (debug)
                  printf("Road and side at %6.2f and %6.2f m\n",
                         heights[index_road], heights[index_non_road]);
                // Intersect the planes
                if (Intersect2Planes(planes[index1], planes[index2], line)) {
                  // Check if map point is near intersection line
                  map_point->Z() = heights[index_road];
                  dist1 = line.DistanceToPoint(map_point->Position3DRef());
                  if (debug)
                    printf("Planes intersect at %5.1f degrees and %5.1f m from map point\n",
                           Angle(planes[index1].Normal(), planes[index2].Normal()) *
                           45 / atan(1.0), dist1);
                  if (dist1 < max_dist_point_line) {
                    found = true;
                    for (index3=0; index3<=heights.size(); index3++)
                      if (use_map_line[index3] == index_non_road) {
                        use_map_line[index3] = index_road;
                        if (debug) printf("Reset to road index\n");
                      }
                  }
                }
              }
            }
          }
        }
      }
    } while (found);
*/
    // Create 3D points for every remaining height
    use_map_point.resize(heights.size());
    // Store height for first line in the current map point
    map_point->Z() = heights[use_map_line[0]];
    use_map_point[0] = map_point->Number();
    // For further heights, create new 3D points
    next_point_number = map_points.HighestPointNumber().Number() + 1;
    for (sel_map_line=sel_map_lines.begin()+1, index1=1;
         sel_map_line!=sel_map_lines.end(); sel_map_line++, index1++) {
      if (use_map_line[0] == index1)
        use_map_point[index1] = map_point->Number();
      else if (use_map_line[index1] == index1) {
        new_map_point          = *map_point;
        new_map_point.Z()      = heights[index1];
        new_map_point.Number() = next_point_number;
        use_map_point[index1]  = next_point_number;
        map_points.push_back(new_map_point); // Add the new point
        map_point = map_points.begin() + index; // Re-set iterator of this loop
        next_point_number++;
      }
    }
    // Update the point numbers in the topologies
    for (sel_map_line=sel_map_lines.begin()+1, index1=1;
         sel_map_line!=sel_map_lines.end(); sel_map_line++, index1++) {
      // Refer to the correct map point
      if (use_map_line[index1] != index1)
        use_map_point[index1] = use_map_point[use_map_line[index1]];
      // Check if update is required
      if (use_map_point[index1] != map_point->Number()) {
        node_ptr = (*sel_map_line)->NodePointer(map_point->Number());
        if (node_ptr == NULL) {
          printf("Error: Could not look up point %d in polygon %d, index1 is %d.\n",
                 map_point->Number(), (*sel_map_line)->Number(), index1);
          printf("Height     use_map_line    use_map_point\n");
          for (index2=0; index2<heights.size(); index2++)
            printf("%6.1f %5d %5d\n", heights[index2], use_map_line[index2],
                   use_map_point[index2]);
          exit(0);
        }
        else {
          node_ptr->Number() = use_map_point[index1];
          if (use_map_point[index1] == 0) {
            printf("Error: use_map_point[%d]=0, use_map_line[]=%d, size %d\n",
                   index1, use_map_line[index1], heights.size());
            printf("Height     use_map_line    use_map_point\n");
            for (index2=0; index2<heights.size(); index2++)
              printf("%6.1f %5d %5d\n", heights[index2], use_map_line[index2],
                     use_map_point[index2]);
          }
        }
        // Repeat the renumbering for the case of closed polygons starting here
        node_ptr = (*sel_map_line)->NodePointer(map_point->Number());
        if (node_ptr != NULL) node_ptr->Number() = use_map_point[index1];
      }
    }
    if (index == (index/5)*5 && !debug)
      printf("%6d (%5.1f\%) %5d %5.2f %5.2f\r", index, 100.0 * index / num_2d_points,
             map_points.size() - num_2d_points, map_point->X(), map_point->Y());
  }
  printf("\n");

  // Try to infer heights from neighbouring polygon points if there
  // were no nearby laser points
  for (map_point=map_points.begin(); map_point!=map_points.end(); map_point++) {
    if (map_point->Z() == -100.0) {//change to 100.0 if one want to use this loop
      // Find a line with this point number
      map_line = map_lines.begin();
      while ((node = map_line->NodeIterator(map_point->Number())) ==
              map_line->end()) map_line++;
      // Find an earlier point
      count = 0;
      do {
        count++;
        node = map_line->PreviousNode(node);
        previous_point = map_points.PointIterator(*node);
      } while (previous_point->Z() == -100.0 && count < map_line->size());
      if (count == map_line->size()) continue; // No point with valid height
      // Find a later point
      node = map_line->NodeIterator(map_point->Number());
      do {
        node = map_line->NextNode(node);
        next_point = map_points.PointIterator(*node);
      } while (next_point->Z() == -100.0);
      // Take a weighted height
      dist1 = (map_point->vect2D() - previous_point->vect2D()).Length();
      dist2 = (map_point->vect2D() - next_point->vect2D()).Length();
      map_point->Z() = (dist1 * next_point->Z() +
                        dist2 * previous_point->Z()) / (dist1 + dist2);
    }
  }  

  // Correct height jumps in the road surface (due to terrain points below
  // the road)
  printf("Analysing height jumps...\n");
  for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
    if (map_line->TOP10MajorClass() == TOP10_Road) {
      if (debug) printf("Analysing map line %d\n", map_line->Number());
      node=map_line->begin();
      map_point = map_points.PointIterator(*node);
      previous_node=map_line->PreviousNode(node);
      previous_point = map_points.PointIterator(*previous_node);
      dist1 = (map_point->vect2D() - previous_point->vect2D()).Length();
      next_node=map_line->NextNode(node);
      for (; node!=map_line->end()-1; next_node=map_line->NextNode(node)) {
        next_point = map_points.PointIterator(*next_node);
        dist2 = (map_point->vect2D() - next_point->vect2D()).Length();
//        if (debug) printf("dist 1, dist 2, %4.2f, %4.2f\n", dist1, dist2);

        if (fabs((previous_point->Z() - map_point->Z())) / dist1 > max_road_slope &&
            fabs((next_point->Z() - map_point->Z())) / dist2 > max_road_slope) {
          new_map_point = *map_point;
          new_map_point.Z() = (dist1 * next_point->Z() +
                               dist2 * previous_point->Z()) / (dist1 + dist2);
          new_map_point.Number() = map_points.HighestPointNumber().Number() + 1;
          map_points.push_back(new_map_point);
  //        node->Number() = new_map_point.Number();
          if (node == map_line->begin()) // Double update for close polygon
  //          (map_line->end()-1)->Number() = new_map_point.Number();
          if (debug) printf("Correcting height of road point %d from %5.2f to %5.2f\n",
                            map_point->Number(), map_point->Z(), new_map_point.Z());
          map_point = map_points.end() - 1;
        }
        previous_node = node;      previous_point = map_point;
        node          = next_node; map_point      = next_point;
        dist1         = dist2;
      }
    }
  }
  printf("Writing results...\n");

  // Write the resulting 3D topography
  if (!map_points.Write(map_points_output))
    printf("Error writing the 3D map points\n");
  if (!map_lines.Write(map_topology_output, false))
    printf("Error writing the topology of the 3D map lines\n");
  if (!quality_points.Write("quality_laser_points.laser", false))
    printf("Error writing the quality laser points\n");  
}

