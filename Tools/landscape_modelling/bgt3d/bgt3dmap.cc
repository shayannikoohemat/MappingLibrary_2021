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

 Initial creation:
 Author : Sander Oude Elberink
 Date   : 11.03.2011, based on top3d

// updates: 
   - reduction factor for decreasing log files, for kadaster
   - keeping only valid wall polygons, after comment of AZ, 25072013
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

void bgt3DMap(char *laser_input, char *map_points_input,
	       char *map_topology_input, char *map_points_output,
	       char *map_topology_output, char *wall_points_output,
	       char *wall_topology_output, double min_height_difference, double min_height_difference_roads,
           bool debug, int reductionfactor)

{
  LaserPoints                  laser_points, sel_laser_points, meadow_laser_points,
                               quality_points;//, poly_laser_points;
  LaserPoints::iterator        laser_point, nearest_point;
  LaserPoints::const_iterator  nearest_tagged_point;
  LaserPoint                   nb_laser_point;
  TINEdges                     edges;//, poledges;
  ObjectPoints                 map_points, dtm_tin_points, clean_map_points, steppoints,
                               firststeppoints, wallpoints;
  LineTopologies               map_lines, dtm_tin_lines, steplines, commonedges,
                               sel_step_lines, sel_lines, map_lines2d, selcommonedges,
                               walls, copycommonsegments, uniquesegments,
                               sel_lines3, sel_lines4,commonedges2d3d,selcommonedges2d3d,
                               selcommonedges3d, notcommonsegments;
  ObjectPoints::iterator       map_point, previous_point, next_point, second_map_point;
  ObjectPoint                  new_map_point, first_wall_point, second_wall_point,
                               third_wall_point, last_wall_point, fourth_wall_point;
  LineTopologies::iterator     map_line, last_map_line, map_line2, map_line3,
                               map_line4;
  LineTopsIterVector           sel_map_lines;
  LineTopsIterVector::iterator sel_map_line, sel_map_line2;
  PointNumberList              neighbourhood, polygon_nbh;
  PointNumberList::iterator    node, previous_node, next_node;
  PointNumber                  *node_ptr;
  int                          nearest_laser_point, dominant_segment, count,
                               success, index1, index2, index3, close_index1,
                               close_index2, next_point_number, index, pol_num,
                               num_2d_points, top10class, iter, line_number3=0, line_number4 = 0,
                               nearest_tagged_laser_point, count_points;
  Plane                        plane;
  Planes                       planes;
  Line3D                       line;
  vector<double>               heights, height1, height2;
  vector<int>                  use_map_line, use_map_point, point_count;
  double                       diff, min_diff, new_height, dist1, dist2,
                               nbh_radius, min_height, dx, dy;
  bool                         found, done, emptymap, foundsecond, foundthird, firstislower, lastislower;
  DataBounds3D                 bounds;
  TIN                          dtmtin;
  Position3D                   p1, p2, p3;
  Position2D                   pos;
  LineTopology::iterator       nb_node, node2, startnode, endnode;
  LineTopology                 stepline, commonedge, wall, sel_line3, sel_line4;
  
  double other_nbh_radius      = 4.0;
  double road_nbh_radius       = 4.0;//25 pcp10
  int    min_num_hough_points  = 10; // Was 20//pcp30
  double max_road_slope        = 0.1;
  
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
  map_lines.ReNumber(map_points, 0, 0);
  // Check if the laser points have the required attributes
  //if (!laser_points.begin()->HasAttribute(SegmentNumberTag)) {
  //  printf("BGT3DMap requires laser points with SegmentNumberTags\n");
    for (laser_point = laser_points.begin(); laser_point!=laser_points.end(); laser_point++){
         if (!laser_point->HasAttribute(SegmentNumberTag)) laser_point->SetFiltered() ;
    }
    laser_points.RemoveFilteredPoints();
    
    
printf ("Reduction factor to decrease amount of print information to screen: %d.\n", reductionfactor);
   
  //}
laser_points.DeriveDataBounds(0);

double pointdistance = laser_points.MedianInterPointDistance(laser_points.size()-1);
other_nbh_radius = 2*pointdistance;
 road_nbh_radius = other_nbh_radius;
    
  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (!map_line->IsClosed()) {
       printf("%d \n", map_line->Attribute(BuildingNumberTag));
      map_lines.erase(map_line);
      map_line--;
      count++;
    }
  }
 if (count!=0) printf("Removed %d not closed polygons\n", count);fflush(stdout);
 if (count == 0) printf("All polygons are closed\n");fflush(stdout);

  printf("Deriving TIN ..."); fflush(stdout);
  laser_points.DeriveTIN();
  printf("done\nDeriving TIN edges ...\n"); fflush(stdout);
  edges.Derive(laser_points.TINReference());
  
 
  map_lines2d = map_lines;
  
  // Determine heights for all map points
  num_2d_points = map_points.size();
  for (map_point=map_points.begin(), index=0;
       index < num_2d_points; map_point++, index++) {
  //  continue;
        pos = Position2D(map_point->X(), map_point->Y());

    if (debug){
      printf("Debugging point %d, index %d (%5.1f\%)\n", map_point->Number(), index, 100.0*index/(1.0*num_2d_points));fflush(stdout);
      printf("%6d (%5.1f\%) %5d %5.2f %5.2f\r", index, 100.0 * index / num_2d_points,
             map_points.size() - num_2d_points, map_point->X(), map_point->Y());fflush(stdout);
      }
    if (map_point->X() < laser_points.DataBounds().Minimum().X() || map_point->X() > laser_points.DataBounds().Maximum().X()) continue;
    if (map_point->Y() < laser_points.DataBounds().Minimum().Y() || map_point->Y() > laser_points.DataBounds().Maximum().Y()) continue;
    
    // Determine all polygons to which this map point belongs
    sel_map_lines.Clear();
    for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
      if (map_line->Contains(map_point->NumberRef())) {
        sel_map_lines.push_back(map_line);
      }
    }
    if (debug)
      printf("number of sel map lines %d\n", sel_map_lines.size());fflush(stdout);
    
    // Skip this point if it is not in a line
    if (sel_map_lines.empty()) continue;
    // Determine the height in all polygons
    if (!heights.empty()) heights.erase(heights.begin(), heights.end());
    if (!planes.empty()) planes.erase(planes.begin(), planes.end());
    for (sel_map_line=sel_map_lines.begin(); sel_map_line!=sel_map_lines.end();
         sel_map_line++) {
      // Select the laser points inside the polygon
      sel_laser_points.ErasePoints();
      emptymap = false;
      
      top10class = (*sel_map_line)->TOP10MajorClass();
      if (debug) printf("top10class %d\n", top10class);fflush(stdout);

      if (top10class == TOP10_Road) nbh_radius = road_nbh_radius;
      else nbh_radius = other_nbh_radius;
      // For water and buildings just get all points inside the polygon
      if (top10class == TOP10_Water) {
        if (debug) printf("water, line number %d\n", (*sel_map_line)->Number());fflush(stdout);
  //      poly_laser_points.ErasePoints();
//        sel_laser_points.AddTaggedPoints(laser_points,
//                                         (*sel_map_line)->Attribute(BuildingNumberTag),
//                                         PolygonNumberTag);
//      if (poly_laser_points.size()<10) emptymap = true;
    
  //      sel_laser_points = poly_laser_points;
  //      if (debug) printf("sel laser points %d\n", sel_laser_points.size());fflush(stdout);
      }
//      polygon_nbh.erase(polygon_nbh.begin(), polygon_nbh.end());
      
//      polygon_nbh = laser_points.TaggedPointNumberList(PolygonNumberTag, (*sel_map_line)->Attribute(BuildingNumberTag));
      sel_laser_points.AddTaggedPoints(laser_points,
                                         (*sel_map_line)->Attribute(BuildingNumberTag),
                                         PolygonNumberTag);
//      if (debug) printf("polygon_nbh %d\n", polygon_nbh.size());fflush(stdout);
      if (debug) printf("sellasersize %d\n", sel_laser_points.size());fflush(stdout);      
//      if (polygon_nbh.size()<10) emptymap = true;
      if (sel_laser_points.size()<10 && top10class != TOP10_Water) emptymap = true;
      // For other surfaces, get a minimum set of points
      if ((top10class == TOP10_Meadow || top10class == TOP10_Road) && !emptymap) {
     if (debug)      printf("poly ");fflush(stdout);
        
 //       poledges.erase(poledges.begin(), poledges.end());
 //       poly_laser_points.DeriveTIN();
 //      if (debug)    printf("edges (%d)", poly_laser_points.size());fflush(stdout);
 //       poledges.Derive(poly_laser_points.TINReference());
 //      if (debug) printf("nearest ");fflush(stdout);

 //       nearest_laser_point = poly_laser_points.NearestPoint(map_point->Position3DRef(),
 //                                                   poledges, true);
  //      nearest_laser_point = laser_points.NearestPoint(map_point->Position3DRef(),
 //                                                   edges, true);
 /*       nearest_tagged_laser_point = laser_points.NearestTaggedPoint(pos,
                                               (*sel_map_line)->Attribute(BuildingNumberTag),
                                               nearest_tagged_point,PolygonNumberTag);      
        iter = 0; done = false;
        
       if (debug) printf("other surface ");fflush(stdout);
        
        do {
          iter++;
          if (debug) printf("iter %d ", iter);fflush(stdout);
          neighbourhood.erase(neighbourhood.begin(), neighbourhood.end());
   //       neighbourhood = 
   //         poly_laser_points.Neighbourhood(PointNumber(nearest_laser_point),
   //                                          nbh_radius, poledges,
   //                                          true, false);
          neighbourhood = 
            laser_points.Neighbourhood(PointNumber(nearest_tagged_laser_point),
                         //                    (*sel_map_line)->Attribute(BuildingNumberTag),
                                             nbh_radius, edges,
                           //                  PolygonNumberTag,
                                             true, false);
   
          if (neighbourhood.size() < min_num_hough_points) {
            nbh_radius *= 2.0;
          }
          else done = true;
        } while (!done && iter < 20);
        if (debug)
          printf("Region %d pts, radius %5.2f\n", neighbourhood.size(),
                 nbh_radius);fflush(stdout);
          if (neighbourhood.size()>10){
        // Create a set of laser points of this neighbourhood
           sel_laser_points.ErasePoints();
           for (node=neighbourhood.begin(); node!=neighbourhood.end(); node++)
//               sel_laser_points.push_back(poly_laser_points[node->Number()]);
               if (laser_points[node->Number()].Attribute(PolygonNumberTag)==(*sel_map_line)->Attribute(BuildingNumberTag)){
               sel_laser_points.push_back(laser_points[node->Number()]);
               }
            }
            if (sel_laser_points.size()<6) emptymap = true;
        
          }
          else emptymap = true;
  
      iter = 0; done = false;
      do {
          iter++;
          if (debug) printf("iter %d ", iter);fflush(stdout);
          sel_laser_points.ErasePoints();
          for (node=polygon_nbh.begin(); node!=polygon_nbh.end(); node++){
               if (fabs(laser_points[node->Number()].X()-map_point->X())<nbh_radius){
                if   (fabs(laser_points[node->Number()].Y()-map_point->Y())<nbh_radius){
                    sel_laser_points.push_back(laser_points[node->Number()]);
                }
               }
          }
          nbh_radius*= 2.0;
               
      } while (sel_laser_points.size()<10 && iter<3);
      */
      iter = 0;
      do {
      iter++;
      count_points = 0;
      for (laser_point = sel_laser_points.begin(); laser_point!=sel_laser_points.end(); laser_point++){
          laser_point->SetFiltered();
           if (fabs(laser_point->X()-map_point->X())<nbh_radius){
           if (fabs(laser_point->Y()-map_point->Y())<nbh_radius){
              dx = laser_point->X()-map_point->X();
              dy = laser_point->Y()-map_point->Y();
           if (dx*dx + dy*dy < nbh_radius*nbh_radius){
               laser_point->SetUnFiltered();
               count_points++;
               }
            }
            }  
          }
      nbh_radius*=2.0;
      } while (count_points<min_num_hough_points && iter<5);
      sel_laser_points.RemoveFilteredPoints();
      if (sel_laser_points.size()<10) emptymap = true;
      if (debug) printf("now here with iter %d and sellaserpoints %d", iter, sel_laser_points.size());fflush(stdout);
      }   
      if (top10class == TOP10_Road) nbh_radius = road_nbh_radius;
      else nbh_radius = other_nbh_radius;
      if (top10class == TOP10_Building || emptymap){ //look for nearby laser points (note, these are on DTM surface.)
        nearest_laser_point = laser_points.NearestPoint(map_point->Position3DRef(),
                                                    edges, true);
        iter = 0; done = false;
        
        do {
          iter++;
          if (debug)  printf("iter %d ", iter);fflush(stdout);
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
                 nbh_radius);fflush(stdout);
        // Create a set of laser points of this neighbourhood
        sel_laser_points.ErasePoints();
        for (node=neighbourhood.begin(); node!=neighbourhood.end(); node++)
          sel_laser_points.push_back(laser_points[node->Number()]);
      }
     sel_laser_points.DeriveDataBounds(0);  
        // For water, just take the lowest point
        if (top10class == TOP10_Water){
          if (sel_laser_points.empty()) {
            if (debug) printf("no points ");fflush(stdout);
            heights.push_back(-100.0);
            planes.push_back(Plane());
          }
          else {
            if (debug) printf(" points ");fflush(stdout);
            heights.push_back(sel_laser_points.DataBounds().Minimum().Z());
            planes.push_back(Plane(Vector3D(0.0, 0.0, *(heights.end()-1)),
                                 Vector3D(0.0, 0.0, 1.0)));
          }
          if (debug) printf("plane pushed back ");fflush(stdout);
        }
        if (top10class == TOP10_Building){       
          //heights.push_back(sel_laser_points.Mean().Z());
          double ground_height;
          if ((*sel_map_line)->HasAttribute(PredictedHeight)){
          ground_height = 0.01*((*sel_map_line)->Attribute(PredictedHeight)-1000); //take height of whole polygon
          }
          else ground_height = sel_laser_points.DataBounds().Minimum().Z();
          if (debug) printf("\nheights%4.2f and %4.2f = %4.2f\n", sel_laser_points.Mean().Z(), ground_height, sel_laser_points.Mean().Z()- ground_height);fflush(stdout);
          heights.push_back(ground_height);
          planes.push_back(Plane(Vector3D(0.0, 0.0, *(heights.end()-1)),
                                 Vector3D(0.0, 0.0, 1.0)));
          }

//        if (emptymap && sel_laser_points.size()>0 && top10class != TOP10_Building){       
//          heights.push_back(sel_laser_points.Mean().Z());
//          planes.push_back(Plane(Vector3D(0.0, 0.0, *(heights.end()-1)),
//                                 Vector3D(0.0, 0.0, 1.0)));
//          }
      // For other classes, determine dominant plane by fitting to points
      // with the most frequent segment number 
        if (top10class == TOP10_Meadow || top10class == TOP10_Road) {
           dominant_segment =
           sel_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
           if (debug) printf("sel_laser_points %d COUNT %d, dominant segment: %d ", sel_laser_points.size(), count, dominant_segment);fflush(stdout);
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
              if (debug) printf("X,Y %4.2f, %4.2f,X,Y %4.2f, %4.2f, height = %4.2f, meanheight %4.2f\n", map_point->X(), map_point->Y(), sel_laser_points.Mean().X(),sel_laser_points.Mean().Y(), plane.Z_At(map_point->X(), map_point->Y(), &success), sel_laser_points.Mean().Z());fflush(stdout);
           }
           else {
//            if (top10class == TOP10_Meadow && count > 0) { //meadow and a few points, take average of greatest segment
            if (count > 0) { // a few points, take average of sel laser points
//              meadow_laser_points.ErasePoints();
//              meadow_laser_points.AddTaggedPoints(sel_laser_points, dominant_segment, SegmentNumberTag);
              heights.push_back(sel_laser_points.Mean().Z());
              if (debug) printf("MEAN height = %4.2f\n", sel_laser_points.Mean().Z());fflush(stdout);
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
    for (index1=0; index1<heights.size(); index1++){
      use_map_line[index1] = index1;
      if (debug)printf("%d %4.2f\n", index1, heights[index1]);
      }  
  
    for (sel_map_line=sel_map_lines.begin(), index1=0;
         sel_map_line!=sel_map_lines.end(); sel_map_line++, index1++) {
         if (debug) printf("map line %d (%d), label %d, height %4.2f, usemaplineindex %d\n",
                     (*sel_map_line)->Attribute(BuildingNumberTag),(*sel_map_line)->Number(), (*sel_map_line)->Attribute(LineLabelTag),
                     heights[index1], use_map_line[index1]);fflush(stdout);
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
    // If one of the lines is a building polygon, take this height for adjacent
    // meadows.
    for (sel_map_line=sel_map_lines.begin(), index1=0;
         sel_map_line!=sel_map_lines.end(); sel_map_line++, index1++) {
      if ((*sel_map_line)->TOP10MajorClass() == TOP10_Building) {
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
    if (debug) printf("start while loop with %d heights\n", heights.size());fflush(stdout);
       if (debug) printf("start while loop with %d use map lines\n", use_map_line.size());fflush(stdout);
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
                if (debug)printf("Label 1 = %d, index1 = %d\n", sel_map_lines[close_index1]->Attribute(LineLabelTag), index1);fflush(stdout);
                if (debug)printf("Label 2 = %d, index2 = %d\n", sel_map_lines[close_index2]->Attribute(LineLabelTag), index2);fflush(stdout);
              }
            }
          }
        }
      }
      if (debug) printf("Height difference = %5.2f, close_index1 = %d close_index2 = %d\n", min_diff, close_index1,close_index2);fflush(stdout);
      if (min_diff > min_height_difference && min_diff < 3* min_height_difference &&
          sel_map_lines[close_index2]->TOP10MajorClass() == TOP10_Meadow 
       && sel_map_lines[close_index1]->TOP10MajorClass() == TOP10_Meadow){
         if (debug) printf("Merging two terrain segments \n");fflush(stdout);
         heights[close_index1] = (heights[close_index1] +
                                  heights[close_index2]) / 2.0;
  //       heights[close_index2] = heights[close_index1];
         
         if (debug) printf("Merging two other segments\n");fflush(stdout);
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
                     || sel_map_lines[close_index1]->TOP10MajorClass() == TOP10_Water ||
            sel_map_lines[close_index2]->TOP10MajorClass() == TOP10_Building 
                     || sel_map_lines[close_index1]->TOP10MajorClass() == TOP10_Building ){ // if one or both water or building, get out of loop
                     if (debug) printf("one or both is water or building, keep both \n");fflush(stdout);
                     min_diff = min_height_difference +1;
                     heights[close_index1] = heights[close_index1];
                     heights[close_index2] = heights[close_index2];
                     }
          else{
          if (debug) printf("Height difference %5.2f\n", min_diff);fflush(stdout);
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
              if (debug) printf("Merging two road segments\n");fflush(stdout);
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
              if (debug) printf("Merging two road segments with same function\n");fflush(stdout);
                for (index1=0; index1<=heights.size(); index1++)
                if (use_map_line[index1] == close_index2)
                   use_map_line[index1] = close_index1;     
                  }
                  else {
                    min_diff = min_height_difference +1;
                    heights[close_index1] = heights[close_index1];
        //           heights[close_index2] = heights[close_index2];
                   
                  }
                  }
           }
          else {
            // Road segment with other segment, keep road height
            if (debug) printf("Merging road and other segment, %d %d\n", heights.size(), use_map_line.size());fflush(stdout);
            for (index1=0; index1<=heights.size(); index1++)
              if (use_map_line[index1] == close_index2)
                use_map_line[index1] = close_index1;
            if (debug) printf("middle of merging road and other segment\n");fflush(stdout);

                nb_laser_point.X() = map_point->X();
            nb_laser_point.Y() = map_point->Y();
            nb_laser_point.Z() = (heights[close_index1] + heights[close_index2]) / 2.0;
//            nb_laser_point.SetPointNumber(map_point->Number());
            nb_laser_point.SetAttribute(LabelTag, 5);                               
            quality_points.push_back(nb_laser_point);
            if (debug) printf("end of merging road and other segment\n");fflush(stdout);

          }
        if (debug) printf("before else loop\n");
        }
        else { //if the first wasn't road
          if (debug) printf(" first wasn't road\n");
          if (sel_map_lines[close_index2]->TOP10MajorClass() == TOP10_Road) {
            // Other segment with road segment, keep road height
            if (debug) printf("Merging other and road segment\n");fflush(stdout);
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
          //            heights[close_index2] = heights[close_index1];

                      for (index1=0; index1<=heights.size(); index1++)
                           if (use_map_line[index1] == close_index2)
                               use_map_line[index1] = close_index1;
                   }
          else {
            if (debug) printf("Merging two other segments\n");fflush(stdout);
            // Two other segments, just take average
            heights[close_index1] = (heights[close_index1] +
                                     heights[close_index2]) / 2.0;
     //       heights[close_index2] = heights[close_index1]; //needed or not?

            if (debug) printf("Merging two other segments\n");fflush(stdout);
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
     if (debug) printf("end of while loop %4.2f < %4.2f \n", min_diff, min_height_difference);fflush(stdout);
    } while (min_diff < min_height_difference);
   
    
    }
    
     if (debug) printf("\nAnd so..:\n");
    for (sel_map_line=sel_map_lines.begin(), index1=0;
         sel_map_line!=sel_map_lines.end(); sel_map_line++, index1++) {
         if (debug) printf("map line %d, label %d, height %4.2f, usemaplineindex %d\n",
                     (*sel_map_line)->Attribute(BuildingNumberTag), (*sel_map_line)->Attribute(LineLabelTag),
                     heights[index1], use_map_line[index1]);fflush(stdout);
    }
    if (debug) printf("create 3d points for every new height...%d heights\n", heights.size());fflush(stdout);
    
    // Create 3D points for every remaining height
    use_map_point.resize(heights.size());
    // Store height for first line in the current map point
    map_point->Z() = heights[use_map_line[0]];
    use_map_point[0] = map_point->Number();
    if (debug) printf("orig map point %d with height %5.2f\n", map_point->Number(), map_point->Z());fflush(stdout);

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
        if (debug) printf("new map point %d with height %5.2f\n", new_map_point.Number(), new_map_point.Z());
  //      system("pause");
        if (debug) printf("prepare for making walls\n");fflush(stdout);
        firststeppoints.push_back(*map_point);
        if (debug) printf("old map point pushed back\n");fflush(stdout);
        //prepare for walls to fill gaps
        steppoints.push_back(new_map_point);
        if (debug) printf("new map point pushed back\n");fflush(stdout);

        steppoints.push_back(*map_point);
        if (debug) printf("old map point pushed back\n");fflush(stdout);

        

        stepline = LineTopology(line_number3, 1, new_map_point.Number(), map_point->Number());
        if (debug) printf("create step line\n");fflush(stdout);

        stepline.SetAttribute(BuildingNumberTag, (*sel_map_line)->Number());//actually, take the preferred one, 'eis' 7
        steplines.push_back(stepline);
        line_number3++;
        
        
        if (debug) printf("new map point %d with height %4.2f, for line %d\n", new_map_point.Number(), new_map_point.Z(), index1);fflush(stdout);
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
        if (debug) printf("update required point %d in polygon %d, index1 is %d.\n",
                 map_point->Number(), (*sel_map_line)->Number(), index1);fflush(stdout);
          if (debug) printf("Height     use_map_line    use_map_point\n");fflush(stdout);
          for (index2=0; index2<heights.size(); index2++)
            if (debug) printf("%6.2f %5d %5d\n", heights[index2], use_map_line[index2],
                   use_map_point[index2]);
   
        if (node_ptr == NULL) {
          if (debug) printf("Error: Could not look up point %d in polygon %d, index1 is %d.\n",
                 map_point->Number(), (*sel_map_line)->Number(), index1);fflush(stdout);
          if (debug) printf("Height     use_map_line    use_map_point\n");fflush(stdout);
          for (index2=0; index2<heights.size(); index2++)
            if (debug) printf("%6.2f %5d %5d\n", heights[index2], use_map_line[index2],
                   use_map_point[index2]);fflush(stdout);
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
                     use_map_point[index2]);fflush(stdout);
     //       system("pause");
          }
        }
        // Repeat the renumbering for the case of closed polygons starting here
        node_ptr = (*sel_map_line)->NodePointer(map_point->Number());
        if (node_ptr != NULL) {
          node_ptr->Number() = use_map_point[index1];
          if (debug) printf("yes yes node ptr number %d\n", node_ptr->Number());fflush(stdout);
          }
      }
    }
    if (index == (index/reductionfactor)*reductionfactor && !debug)
     printf("%6d (%5.1f\%) %5d %5.2f %5.2f\r", index, 100.0 * index / num_2d_points,
             map_points.size() - num_2d_points, map_point->X(), map_point->Y());fflush(stdout);
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
      
      nb_laser_point.X() = map_point->X();
                nb_laser_point.Y() = map_point->Y();
                nb_laser_point.Z() = map_point->Z();
                nb_laser_point.SetAttribute(LabelTag, 7);                               
                quality_points.push_back(nb_laser_point);
      
    }
  }  

  printf("\nWriting results...\n");
  steppoints.RemoveDoublePoints(steplines, 0.01);
      map_points.Write(map_points_output);
       map_lines.Write(map_topology_output, false);

 if (debug) printf("Size of map lines 2d: %d\n",map_lines2d.size());
 if (debug) printf("Size of map lines 3d: %d\n",map_lines.size());
  printf("Filling holes at locations of step edges... this can take a while...\n");
//  system("pause");
     LineTopologies::iterator polygon;
    LineTopology commonsegment, commonsegment3d;
    LineTopologies commonsegments, commonsegments3d;
    double px1, px2, py1, py2;
    for (map_line=map_lines2d.begin(), index = 0, index2=0; map_line!=map_lines2d.end(); map_line++, index++) {
      if (debug || index == (index/reductionfactor)*reductionfactor) printf("\nprogress %4.2f \r", 100.0*index/map_lines2d.size());
//      if (index2<map_lines2d.size()){
      for (map_line2=map_line, index2=index; map_line2!=map_lines2d.end(); map_line2++, index2++) {
          if (map_line2!=map_line && map_line2->Adjacent(*map_line, 2)){
             selcommonedges.erase(selcommonedges.begin(), selcommonedges.end());
              if (map_line->Attribute(BuildingNumberTag)!=map_line2->Attribute(BuildingNumberTag)){
                selcommonedges.clear();
                selcommonedges = map_line->AllCommonParts(*map_line2);
//                printf("size of selcommonedges = %d", selcommonedges.size());
                commonedges.insert(commonedges.end(), selcommonedges.begin(), selcommonedges.end());
                sel_line3 = map_lines[index];
                sel_line4 = map_lines[index2];
                selcommonedges3d = sel_line3.AllCommonParts(sel_line4);
              
                selcommonedges2d3d.clear();
  
                commonedges2d3d.insert(commonedges2d3d.end(), selcommonedges3d.begin(), selcommonedges3d.end());
                commonsegments.clear();
                commonsegments3d.clear();
                     for (polygon=selcommonedges.begin(), line_number3=0; polygon!=selcommonedges.end(); polygon++) {
                         if (polygon->size()>1){
                             node2 = polygon->begin();
                             startnode = node2;
                             node2++;
                             for (; node2!=polygon->end(); node2++) {
                                    endnode = node2;
                                    commonsegment = LineTopology(line_number3, 1, startnode->Number(), endnode->Number());
                                    commonsegments.push_back(commonsegment);
                                    line_number3++;
                                    startnode = node2;
                             }
                         }
                     }
                     for (polygon=selcommonedges3d.begin(), line_number3=0; polygon!=selcommonedges3d.end(); polygon++) {
                         if (polygon->size()>1){
                             node2 = polygon->begin();
                             startnode = node2;
                             node2++;
                             for (; node2!=polygon->end(); node2++) {
                                    endnode = node2;
                                    commonsegment3d = LineTopology(line_number3, 1, startnode->Number(), endnode->Number());
                                    commonsegments3d.push_back(commonsegment3d);
                                    line_number3++;
                                    startnode = node2;
                             }
                         }
                     }
                     bool adjac;
                     for (map_line3=commonsegments.begin(); map_line3!=commonsegments.end(); map_line3++) {  
                         for (map_line4=commonsegments3d.begin(), adjac = false; map_line4!=commonsegments3d.end() && !adjac; map_line4++) {
                           if (map_line4->Adjacent(*map_line3, 2)){
                                              map_line3->SetAttribute(LineLabelTag, 7);
                                              adjac = true;
                                              commonsegments.erase(map_line3);
                                              map_line3--;
                           }
                         }
                     }
                     notcommonsegments.insert(notcommonsegments.end(), commonsegments.begin(), commonsegments.end());
                     if (commonsegments.size()>0){
                     for (map_line3=commonsegments.begin(); map_line3!=commonsegments.end(); map_line3++) {  
                             node2 = map_line3->begin();
                             startnode = node2;
                             node2++;
                             endnode = node2;
                             wall.clear();wall.Initialise();
                             wall.Number() = line_number4; line_number4++;
                             wall.SetAttribute(LineLabelTag, 4);
      
                             for (node2=sel_line3.begin(), found = false; node2!=sel_line3.end() && !found; node2++){
                             if ((map_points.PointIterator(*node2)->vect2D()-map_points.PointIterator(*startnode)->vect2D()).Length()<0.001){
                               first_wall_point = *(map_points.PointIterator(*node2));
                               wall.push_back(first_wall_point.Number());
                               wallpoints.push_back(first_wall_point);
                               found = true;
                               }
                             }
                             for (node2=sel_line3.begin(), found = false; node2!=sel_line3.end() && !found; node2++){
                             if ((map_points.PointIterator(*node2)->vect2D()-map_points.PointIterator(*endnode)->vect2D()).Length()<0.001){
                               second_wall_point = *(map_points.PointIterator(*node2));
                               wall.push_back(second_wall_point.Number());
                               wallpoints.push_back(second_wall_point);
                               found = true;
                               }
                             }
                             for (node2=sel_line4.begin(), found = false; node2!=sel_line4.end() && !found; node2++){
                             if ((map_points.PointIterator(*node2)->vect2D()-map_points.PointIterator(*endnode)->vect2D()).Length()<0.001){
                               third_wall_point = *(map_points.PointIterator(*node2));
                               found=true;
                               if (third_wall_point.Z()!=second_wall_point.Z()){
                                 wall.push_back(third_wall_point.Number());
                                 wallpoints.push_back(third_wall_point);
                               }
                               }
                             }
                             for (node2=sel_line4.begin(), found = false; node2!=sel_line4.end() && !found; node2++){
                             if ((map_points.PointIterator(*node2)->vect2D()-map_points.PointIterator(*startnode)->vect2D()).Length()<0.001){
                               fourth_wall_point = *(map_points.PointIterator(*node2));
                               found = true;
                               if (fourth_wall_point.Z()!=first_wall_point.Z()){
                                 wall.push_back(fourth_wall_point.Number());
                                 wallpoints.push_back(fourth_wall_point);
                               }
                               }
                             }
                             wall.push_back(first_wall_point.Number());
                             if (wall.SameOrientation(sel_line3)==0) wall.RevertNodeOrder();
                             if (wall.size()==5){
                             bool revert = false;
                             if (wall.SameOrientation(sel_line3)==0) revert =true;

                             wall.clear();wall.Initialise();
                             wall.Number() = line_number4; line_number4++;
                             wall.SetAttribute(LineLabelTag, 4);
                              wall.SetAttribute(IDNumberTag, sel_line3.Attribute(IDNumberTag)); //take the IDNumber from one of both... can be adapted if rules are clear
                             if (revert){
                               wall.push_back(second_wall_point.Number());
                               wall.push_back(first_wall_point.Number());
                               wall.push_back(third_wall_point.Number());
                               wall.push_back(second_wall_point.Number());
                               walls.push_back(wall);          
                               wall.clear();wall.Initialise();
                               wall.Number() = line_number4; line_number4++;
                               wall.SetAttribute(LineLabelTag, 4);
                               wall.SetAttribute(IDNumberTag, sel_line3.Attribute(IDNumberTag));
                               wall.push_back(third_wall_point.Number());
                               wall.push_back(first_wall_point.Number());
                               wall.push_back(fourth_wall_point.Number());
                               wall.push_back(third_wall_point.Number());
                               walls.push_back(wall);          
        
                                         }
                             else {
                               wall.push_back(first_wall_point.Number());
                               wall.push_back(second_wall_point.Number());
                               wall.push_back(third_wall_point.Number());
                               wall.push_back(first_wall_point.Number());
                               walls.push_back(wall);          
                               wall.clear();wall.Initialise();
                               wall.Number() = line_number4; line_number4++;
                               wall.SetAttribute(LineLabelTag, 4);
                               wall.SetAttribute(IDNumberTag, sel_line3.Attribute(IDNumberTag));
                               wall.push_back(first_wall_point.Number());
                               wall.push_back(third_wall_point.Number());
                               wall.push_back(fourth_wall_point.Number());
                               wall.push_back(first_wall_point.Number());
                               walls.push_back(wall);          
        
                                  }
                             }
                             else {
                                   wall.SetAttribute(IDNumberTag, sel_line3.Attribute(IDNumberTag));
                                   walls.push_back(wall); 
                        
                             }
                           }                             
                                                  
                                                  
                     }
  //                   }
              }
          }
      }
      }
      printf("\n writing output files.\n");
      if (debug) {
      if (!notcommonsegments.Write("notcommonsegments.top", false)) printf("error writing not common segments\n"); fflush(stdout);
      if (!map_points.Write("notcommonsegments.objpts")) printf ("Error writing notcommon segments objpts\n");fflush(stdout);  
      }
      printf("Keeping the valid walls...\n");
      wallpoints.erase(wallpoints.begin(), wallpoints.end());
    for (map_line=walls.begin(), count=0; map_line!=walls.end(); map_line++, count++) {
    //    printf("%d %4.2f\r", count, 100.0*count/walls.size());
        if (map_line->size()<3) {
           walls.erase(map_line); 
           map_line--;
           count--;
           }
    }
      printf("Keeping the valid wall points...\n");

    for (map_line=walls.begin(), count=0; map_line!=walls.end(); map_line++, count++) {
  //      printf("%d %4.2f\r", count, 100.0*count/walls.size());
  
      for (node2=map_line->begin(); node2!=map_line->end(); node2++){
         wallpoints.push_back(*(map_points.PointIterator(*node2)));
      }
    }
    wallpoints.RemoveDoublePoints(walls, 0.01);
      wallpoints.erase(wallpoints.begin(), wallpoints.end());
    for (map_line=walls.begin(); map_line!=walls.end(); map_line++) {
        if (map_line->size()<3){
            walls.erase(map_line); 
            map_line--;
            }
    }
    for (map_line=walls.begin(); map_line!=walls.end(); map_line++) {
      for (node2=map_line->begin(); node2!=map_line->end(); node2++){
         wallpoints.push_back(*(map_points.PointIterator(*node2)));
      }
    }
    if (!walls.Write(wall_topology_output, false)) printf("Error writing walls to fill gaps\n");fflush(stdout);
     if(!wallpoints.Write(wall_points_output)) printf("Error writing wall points to fill gaps\n");fflush(stdout);
  
    map_points.erase(map_points.begin(), map_points.end());
  map_lines.erase(map_lines.begin(), map_lines.end());
  map_lines2d.erase(map_lines2d.begin(), map_lines2d.end());
  
  walls.erase(walls.begin(), walls.end());
  wallpoints.erase(wallpoints.begin(), wallpoints.end());
  laser_points.ErasePoints();  
  quality_points.ErasePoints();
  meadow_laser_points.ErasePoints();
 // poly_laser_points.ErasePoints();
 // poledges.erase(poledges.begin(), poledges.end());
  edges.erase(edges.begin(), edges.end());
  printf("\nProgram bgt3d finished.\n"); fflush(stdout);
  
 // return;
  
}


 
