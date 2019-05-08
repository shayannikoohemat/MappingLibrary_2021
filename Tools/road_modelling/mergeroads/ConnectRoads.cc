
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
 Date   : 24-08-2006

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
#include "LaserPoint.h"
#include "LaserPoints.h"
#include "LineTopsIterVector.h"
#include "dxf.h"
#include <math.h>
#include <ObjectPoint2D.h> 
#include <ObjectPoints2D.h> 
#include <LineSegments2D.h> 

/*
--------------------------------------------------------------------------------
                           The main function
--------------------------------------------------------------------------------
*/

void ConnectRoads(char *laser_input, char *map_points_input, char *map_topology_input,
                  char *map_points_output, char *map_topology_output, char *laser_output)

{
  LaserPoints                  middle_points, sharp_points, dir_points,
                               sel_pos_points, sel_neg_points, sel_dir_points,
                               temp_height_points, tot_height_points, keep_dir_points,
                               sel_laser_points;                        
  LaserPoint                   middle_point, sharp_point, dir_point, virtual_point;
  LaserPoints::iterator        laser_point;
  LineTopology::iterator       node2, node3;
  ObjectPoints                 map_points, sel_map_points, middle_objpoints, 
                               sel_map_points2, connect_points, end_points,
                               graph_points, new_end_points, merge_graph_points,
                               common_points, sel_common_points;
  ObjectPoint                  middle_objpoint, begin_point, end_point, com_point,
                               previous_point, next_point, this_point, sel_map_point,
                               grow_point, previous_point2, next_point2;
  LineTopologies               map_lines, sel_map_lines, graph_lines, graph_lines2,
                               sel_graph_lines, merged_road_lines, grow_lines, grow_lines2,
                               history_lines, connectlines, new_merged_lines, new_merged_lines2,
                               merge_graph, single_lines, common_parts;
  LineTopology                 line, new_map_line, merged_road_line, merged_road_line2,
                               merged_road_line3, merged_road_line4, grow_line, prev_line,
                               merged_road_line5, grow_line2, prev_line2, merged_polygon, common_part,
                               merged_polygon2, temp_line, merged_line, orig_line, new_merged_line;
  ObjectPoints::iterator       map_point, map_point2;
  LineTopologies::iterator     map_line, map_line2, map_line3;
  PointNumberList              neighbourhood;
  PointNumberList::iterator    node, prev_node, next_node, begin_node, end_node;
  LineNumber                   findline, hist_line, hist_line2, find_line;//, findline1;
  HoughSpace                   houghspace;
  int                          countzmin, n, i, j, k, kk, index_line, index_hist, nn,
                               intdist, nearest_pos_laser_point, nearest_neg_laser_point,
                               next_point_number, growing, growing_total, index_hist2, count,
                               intdir, keep_number, success, nearest_laser_point, *numpts,
                               next_pnr, next_line, nnn;
  double                       meanx, meany, dist, angle, PI, dx, dy, direction,
                               mean_positive_direction, mean_negative_direction,
                               sumdist,a, b, sumdist2, xdir, ydir, mean_direction,
                               mean_direction2, max_dist, dist1, dist2, m00, m10, m01,
                               m20, m11, m02, remx, remy, local_direction, //big_angle,
                               dir;
  FILE                         *dxffile;
 // bool                         one_neighbour;
  Vector3D                     p1, p2, p3, p0, p4, pb, pe, ml;
 // TINEdges                     edgesneg, edgespos, edges, edges2; 
  Line3D                       first_line, second_line;
  Plane                        plane;
//  Image                        kernel;
  DataBounds3D                 bounds;
  
PI = 3.14159 ;
bool debug = true;
double sample_distance = 3; //3


if (!sel_pos_points.Read(laser_input)) {
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
  map_points.RemoveDoublePoints(map_lines, 0.1);
  for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
    if (map_line->TOP10MajorClass() == TOP10_Road ){//&& !map_line->TOP10Invisible()){ //Only keep visible road objects
        if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end());
        for (node2=map_line->begin(); node2!=map_line->end()-1; node2++) {
          sel_map_points.push_back(*(map_points.PointIterator(*node2)));
        }
          countzmin=0;
          meanx = 0;
          meany = 0;
          for (map_point=sel_map_points.begin(); map_point!=sel_map_points.end(); map_point++) {
              countzmin++;
              meanx = (meanx*(countzmin-1) + map_point->X())/countzmin;
              meany = (meany*(countzmin-1) + map_point->Y())/countzmin;
            }
            middle_point.X()=meanx; middle_point.Y()=meany; middle_point.Z()=0;
            middle_point.SetPointNumber(map_line->Number());
            middle_points.push_back(middle_point);
            middle_objpoint.X()=meanx; middle_objpoint.Y()=meany; middle_objpoint.Z()=0;
            middle_objpoint.Number() = map_line->Number();
            middle_objpoints.push_back(middle_objpoint);
            sel_map_lines.push_back(*map_line);           
     }
   }
  // Densify the map points
  printf("done\nDensifying map points ..."); fflush(stdout);
  count = map_points.size();
  ObjectPoints keep_map_points;
  LineTopologies keep_map_lines;
  keep_map_points = map_points;
  keep_map_lines = map_lines;
  sel_map_lines.Densify(map_points, sample_distance); 
  printf("done, map points increased from %d to %d\n",
         count, map_points.size());
  // Merge points within 0.1 m
  printf("Removing double points ..."); fflush(stdout);
  map_points.RemoveDoublePoints(sel_map_lines, 0.1);
  printf("done, %d map points left\n", map_points.size());
  
    printf("size of sel_map_lines: %d\n", sel_map_lines.size()); 
  n=0;
  for (map_point=middle_objpoints.begin(), map_line = sel_map_lines.begin(); map_point!=middle_objpoints.end(); 
              map_point++, map_line++) {
//  for (map_point=middle_objpoints.begin(); map_point!=middle_objpoints.end(); map_point++) {
      i=0;
      if(!sel_map_points2.empty()) sel_map_points2.erase(sel_map_points2.begin(),sel_map_points2.end());
      for (map_point2=map_point+1; map_point2!=middle_objpoints.end(); map_point2++) {
          dist = (map_point->vect2D() - map_point2->vect2D()).Length();
          if (dist<= 0.01) {
                     i++;
    //                 if (debug) printf("found dp (%d) ", map_point2->Number());
                     map_point->Z() = map_point->Z() + 1;
                     findline = map_point2->Number();
                     find_line = map_point->Number();
                     index_line = sel_map_lines.FindLine(find_line);
                     if (map_line->TOP10Invisible()) {                                           
                         map_line->Label() = 2010; 
                     }
                     index_line = sel_map_lines.FindLine(findline);
                     if (sel_map_lines[index_line].TOP10Invisible()) {
                         sel_map_lines[index_line].Label() = 2010;
                         }                                            
                     if (i==1) sel_map_points2.push_back(*map_point);
                     sel_map_points2.push_back(*map_point2);                     
          }
      }
}   
  for (map_line=sel_map_lines.begin(), count=0;
       map_line!=sel_map_lines.end(); map_line++) {
    if (map_line->Label() == 2010) {
      sel_map_lines.erase(map_line);
      map_line--;
      count++;
    }
  }
  printf("Removed %d invisible polygons\n", count);
   
  n=0;
  count = 0;
  double min_size_start_polygon = 130; //pcp130

   for (map_line=sel_map_lines.begin(); map_line!=sel_map_lines.end(); map_line++) {
     for (map_line2 = map_line+1; map_line2!=sel_map_lines.end(); map_line2++){
        if (map_line->Adjacent(*map_line2, 2) && !map_line->Adjacent(*map_line2 , map_line->size()-1)){
            line = LineTopology(n, 2006, map_line->Number(), map_line2->Number());
            n++;
 //           sel_graph_lines.push_back(*map_line2);
            graph_lines.push_back(line);
        } 
     }
     bounds = map_line->Bounds(map_points);
     if (sqrt(bounds.XRange()*bounds.XRange()+ bounds.YRange()*bounds.YRange()) > min_size_start_polygon){
        // printf("large polygon found\n");
        single_lines.push_back(*map_line);
         count++;
         }
   }
   printf("%d large polygons found\n", count);

  if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end());
   for (map_line=graph_lines.begin(); map_line!=graph_lines.end();map_line++){
      for (node=map_line->begin(); node!=map_line->end(); node++){
           sel_map_point = *(middle_objpoints.PointIterator(*node));
     //      sel_map_point.Z() = map_line2->Label(); //Direction will be transfered to height value
            graph_points.push_back(sel_map_point);
       }
    }
  
   LaserPoints                    grow_laser_points, potential_points, keep_laser_points;
   LineTopologies                 used_lines, output_lines;
   LineTopology                   keep_merge_line, keep_growing; 
   LineSegments2D segments;                   
   LineSegments2D::const_iterator segment;
   Position2D                     int_point, pos1, pos2;
   Line2D                         line2d;
   ObjectPoint                    temppoint;
   int dominant_segment, num_of_large, dominant_segment2, count2;
   double max_dist_seed = 3; //vertical distance to growing plane in meters; //was 2.5 good on 20-12-06
   double max_dist_grow = 50; //pcp 50
   bool hit_large_polygon, intersect;
   next_pnr = 0;
   next_line = 0, nnn = 0;
   sel_laser_points.ErasePoints();
  // keep_laser_points = sel_pos_points; //transfer all laser points to keep_laser_points

   

//  for (map_point = end_points.begin(); map_point !=end_points.end(); map_point++){ 
//  for (map_line = sel_map_lines.begin(); map_line!=sel_map_lines.end(); map_line++) {
  for (map_line = single_lines.begin(); map_line!=single_lines.end(); map_line++) {
      if(!history_lines.empty()) history_lines.erase(history_lines.begin(),history_lines.end()); //let's see what happens if we forget the history
      if(!sel_graph_lines.empty()) sel_graph_lines.erase(sel_graph_lines.begin(),sel_graph_lines.end());
      if(!new_merged_lines.empty()) new_merged_lines.erase(new_merged_lines.begin(),new_merged_lines.end());
//      hist_line = map_point->Number();
      hist_line = map_line->Number();
      index_hist = history_lines.FindLine(hist_line);
      num_of_large=0;
//      find_line = map_point->Number(); //find the line 
      find_line = map_line->Number(); //find the line 
      index_line = sel_map_lines.FindLine(find_line); 
      merged_line = sel_map_lines[index_line]; // found line, named merged_line
      keep_merge_line = keep_map_lines[keep_map_lines.FindLine(find_line)];
      orig_line = merged_line;
      growing_total = 0;
      hit_large_polygon = false;
      history_lines.push_back(orig_line);
      used_lines.push_back(orig_line);
      grow_laser_points.ErasePoints();
      grow_laser_points.AddTaggedPoints(sel_pos_points, map_line->Number(), PolygonNumberTag);
  //    dominant_segment = grow_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
  //    plane = grow_laser_points.FitPlane(dominant_segment, dominant_segment, SegmentNumberTag);
  //    temp_height_points = grow_laser_points;
  //    grow_laser_points.ErasePoints();
  //    grow_laser_points.AddTaggedPoints(temp_height_points, dominant_segment, SegmentNumberTag); 
  //    temp_height_points.RemoveTaggedPoints(dominant_segment, SegmentNumberTag);
  //    for (laser_point=temp_height_points.begin(); laser_point!=temp_height_points.end(); laser_point++){
  //       if (fabs(plane.Distance(laser_point->Position3DRef())) <  max_dist_seed){
  //           grow_laser_points.push_back(*laser_point);
  //           }
  //     }
       
    //  new_merged_lines.push_back(orig_line);
      if (index_hist == -1) { // Line not merged before
      do {
        growing = 0;  
        n=0; nn =0; mean_positive_direction = 0; mean_negative_direction = 0;
       sel_dir_points.ErasePoints(); keep_dir_points.ReInitialise();
       sumdist=0, sumdist2=0;
       for (node=merged_line.begin(); node!=merged_line.end(); node++) {
         prev_node = merged_line.PreviousNode(node);
         previous_point = *(map_points.PointIterator(*prev_node));
         prev_node = merged_line.PreviousNode(prev_node);
         previous_point2 = *(map_points.PointIterator(*prev_node));
         this_point = *(map_points.PointIterator(*node));
         next_node = merged_line.NextNode(node);
         next_point = *(map_points.PointIterator(*next_node));
         next_node = merged_line.NextNode(next_node);
         next_point2 = *(map_points.PointIterator(*next_node));
         p0 = Vector3D(previous_point2.X(), previous_point2.Y(), previous_point2.Z());
         p1 = Vector3D(previous_point.X(), previous_point.Y(), previous_point.Z());
         p2 = Vector3D(this_point.X(), this_point.Y(), this_point.Z());
         p3 = Vector3D(next_point.X(), next_point.Y(), next_point.Z());     
         p4 = Vector3D(next_point2.X(), next_point2.Y(), next_point2.Z());     
         angle = p2.Angle2D(p1, p3);
 //        big_angle = p2.Angle2D(p0, p4);
         direction = p2.Direction2D(p3);
         angle = angle * 180 / PI;
  //       big_angle = big_angle * 180 / PI;
         direction = direction * 180 / PI;
         dist = (next_point.vect2D() - previous_point.vect2D()).Length();
         intdist = (int) dist;      
//       printf("angle %6.2f dir %6.2f, dist %6.2f (sum %6.2f)\n", big_angle, direction, dist, sumdist);
         if (angle > 150 && direction > 0) {
 //       if (direction > 0) {
                 n++;
                 a = sumdist/(sumdist+dist);
                 b = dist / (sumdist+dist);
                 mean_positive_direction = (mean_positive_direction * a + b*direction); //Take a weighted direction
 //               printf("\n mpd %6.2f (a=%6.2f, b=%6.2f)\n", mean_positive_direction, a, b); 
                 dir_point.X()= this_point.X();
                 dir_point.Y()= this_point.Y();
//                 dir_point.Z()= 100 * sin(direction * PI /180);
                 dir_point.Z()= direction;
//                 dir_point.Attribute(PolygonNumberTag) = map_line->Number(); 
                 dir_point.Attribute(PolygonNumberTag) = merged_line.Number(); 
//                 dir_point.Attribute(LabelTag) = merged_line.Number();
                 dir_point.SetPointNumber(this_point.Number());
                 sel_dir_points.push_back(dir_point);
//                 sel_pos_points.push_back(dir_point);
//                 dir_points.push_back(dir_point);
                 sumdist = sumdist + dist;
                 }
         if (angle > 150  && direction < 0) {
 //        if (direction < 0) {          
                 nn++;
                 a = sumdist2/(sumdist2+dist);
                 b = dist / (sumdist2+dist);
                 mean_negative_direction = mean_negative_direction * a + b* (180+direction);
   //              printf("\n mpd %6.2f (a=%6.2f, b=%6.2f)\n", mean_negative_direction, a, b); 
                 dir_point.X()= this_point.X();
                 dir_point.Y()= this_point.Y();
//                 dir_point.Z()= 100 * sin((direction+180) * PI /180);
                 dir_point.Z()= direction+180;
//                 dir_point.Attribute(PolygonNumberTag) = map_line->Number();
                 dir_point.Attribute(PolygonNumberTag) = merged_line.Number(); 
                 dir_point.SetPointNumber(this_point.Number());
  //               dir_point.Attribute(LabelTag) = merged_line.Number();
                 sel_dir_points.push_back(dir_point);
 //                sel_neg_points.push_back(dir_point);
//                 dir_points.push_back(dir_point);
                 sumdist2 = sumdist2 + dist;
                 }
         }          
         
  //   printf("Pos = %5.2f, Neg = %5.2f\n", mean_positive_direction, mean_negative_direction);
     mean_direction = (mean_positive_direction + mean_negative_direction)/2;
     intdir = (int) mean_direction;
 //    printf("Mean %5.2f, Int = %5d\n", mean_direction, intdir);
     if (sel_dir_points.size()==0) printf("Empty!!: %d\n", map_line->Number());
     merged_line.Label()=intdir;
     grow_line = merged_line;    
     sel_dir_points.Label(merged_line.Number());
     dir_points.insert(dir_points.end(), sel_dir_points.begin(), sel_dir_points.end());
      printf("Grow_line: %d\n", map_line->Number());
       for (map_line2=sel_map_lines.begin(); map_line2!=sel_map_lines.end(); map_line2++){ 
           if (growing < 1) {//when grow_line has grown, growing == 1, loop stops
           if (map_line2->Adjacent(grow_line, 2)){// && !map_line2->Adjacent(grow_line , map_line2->size()-1)){
               index_hist = history_lines.FindLine(map_line2->Number());
               index_line = single_lines.FindLine(map_line2->Number()); 
               if (index_hist == -1 && (index_line == -1 || !hit_large_polygon)){//&& index_line == -1) { //Line not merged before in this polygon... && not large polygon
                   sel_graph_lines.push_back(*map_line2);
                  
                  if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end());
                  for (node2=map_line2->begin(); node2!=map_line2->end()-1; node2++) {
                      sel_map_points.push_back(*(map_points.PointIterator(*node2)));
                  }
                  countzmin=0; meanx = 0;  meany = 0;
                  for (map_point2=sel_map_points.begin(); map_point2!=sel_map_points.end(); map_point2++) {
                     countzmin++;
                     meanx = (meanx*(countzmin-1) + map_point2->X())/countzmin;
                     meany = (meany*(countzmin-1) + map_point2->Y())/countzmin;
                  }
                  middle_objpoint.X()=meanx; middle_objpoint.Y()=meany; middle_objpoint.Z()=0; 
                  middle_point.X()=meanx; middle_point.Y()=meany, middle_point.Z()=0;                       
                  if (!middle_point.InsidePolygon(map_points, grow_line)) {                                                            
                  common_part = map_line2->CommonPart(grow_line);
                  if (common_part.size()>0){
                  begin_node = common_part.begin();
                  end_node = common_part.end()-1;
                  begin_point = *(map_points.PointIterator(*begin_node));
                  end_point = *(map_points.PointIterator(*end_node));
                  pb = Vector3D(begin_point.X(), begin_point.Y(), begin_point.Z());
                  pe = Vector3D(end_point.X(), end_point.Y(), end_point.Z());   
                  dir = pb.Direction2D(pe);
                  dir = dir * 180 /PI;
                  if (dir < 0) dir = dir+180;
                  dist1 = (begin_point.vect2D() - end_point.vect2D()).Length();
                 
  //                printf("size of common part %d, dir = %5.2f\n", common_part.size(), dir);
  //                printf("Direction of growing line: %5.2d \n", intdir);
//                  mean_direction = map_point->Z() * PI /180; //Remember that the z value represented the direction in degrees
                  mean_direction = intdir * PI/180;
                  
                  virtual_point = begin_point;
                  virtual_point.X() = (begin_point.X()+end_point.X()) / 2;
                  virtual_point.Y() = (begin_point.Y()+end_point.Y()) / 2;
                  virtual_point.Z() = dir;
                  // new ideas on 6-10-06
                  sel_laser_points.ErasePoints();
                  for (laser_point=sel_dir_points.begin(); laser_point!=sel_dir_points.end(); laser_point++){
        //              if (fabs(laser_point->Z()-virtual_point.Z())>20 && fabs(laser_point->Z()-virtual_point.Z())<160){
                           dist = (laser_point->vect2D()-virtual_point.vect2D()).Length();                                          
                           if (dist < 50) {
                               sel_laser_points.push_back(*laser_point);
                               if  (laser_point->Z() < 25) {             //Create "double points" near zero/180 crossing...
                                   laser_point->Z() = laser_point->Z() + 180;
                                   sel_laser_points.push_back(*laser_point);
                                   }
                               }
          //            }
                  }
                  sel_laser_points.Label(merged_line.Number());
      //            printf("Laser points: %d\n", sel_laser_points.size());
 //                 dir_points.insert(dir_points.end(), sel_laser_points.begin(), sel_laser_points.end());
                  sel_laser_points.InitialiseHoughSpace(houghspace, merged_line.Number(), 85, 3, 5, 1, LabelTag);
//             sel_laser_points.InitialiseHoughSpace(houghspace, merged_line.Number(), 55, 3,5, 1, LabelTag); // a good parameter set
                  sel_laser_points.IncrementHoughSpace(houghspace);


                  plane = houghspace.BestPlane(numpts, 0, 30); 
     //                               printf("Plane found");
                 local_direction = plane.Z_At(virtual_point.X(), virtual_point.Y(), &success); 
                 if (local_direction < 0) local_direction = local_direction + 180;
     //            local_direction = plane.Z_At(middle_objpoint.X(), middle_objpoint.Y(), &success); 
                  middle_point.Z() = local_direction;//100 * sin (local_direction * PI /180);
                  
                  mean_direction = local_direction * PI /180;
 //                 printf("Num of points: %d (%d), %5.2f\n", *numpts, sel_laser_points.size(), local_direction);
                  xdir = cos(mean_direction); 
                  ydir = sin(mean_direction);
 //                 printf("rad %5.2f, xdir = %5.2f ydir = %5.2f\n", mean_direction, xdir, ydir);
                  first_line = Line3D(begin_point.X(), begin_point.Y(), 0, xdir, ydir, 0); 
                  second_line = Line3D(end_point.X(), end_point.Y(), 0, xdir, ydir, 0);
                  dist1 = first_line.DistanceToPoint(middle_objpoint.Position3DRef());
                  dist2 = second_line.DistanceToPoint(middle_objpoint.Position3DRef());
                  max_dist = first_line.DistanceToPoint(end_point.Position3DRef());
  //                printf("Distance to first line, second line, dblines: %5.2f, %5.2f, %5.2f\n", 
  //                    first_line.DistanceToPoint(middle_objpoint.Position3DRef()),
  //                    second_line.DistanceToPoint(middle_objpoint.Position3DRef()), 
  //                    max_dist);
                  dist = (middle_objpoint.vect2D()-virtual_point.vect2D()).Length();  
//                  pb = Vector3D(middle_objpoint.X(), middle_objpoint.Y(), 0);
//                  pe = Vector3D(virtual_point.X(), virtual_point.Y(), 0);   
//                  dir2 = pb.Direction2D(pe);
//                  dir2 = dir2 * 180 /PI;
//                  if (dir2 < 0) dir2 = dir2+180;
                  
                  if (!sel_common_points.empty()) sel_common_points.erase(sel_common_points.begin(),sel_common_points.end());
                               for (node3=common_part.begin(); node3!=common_part.end(); node3++) {
                                    com_point = *(map_points.PointIterator(*node3));
                                    com_point.Z() = local_direction;
                                    com_point.Number() = next_pnr;
                                    node3->Number() = next_pnr;
                                    next_pnr++;
                                    sel_common_points.push_back(com_point);
                               }
                               common_parts.push_back(common_part);
                               common_points.insert(common_points.end(), sel_common_points.begin(), sel_common_points.end());
                  if (!segments.empty()) segments.erase(segments.begin(), segments.end());
                  printf("Com points %d, com parts %d, dir = %5.2f\n", sel_common_points.size(), common_part.size(), local_direction);
                  segments = LineSegments2D(sel_common_points, common_part);
                  intersect = false;
                  pos1 = Position2D(middle_objpoint.X() + xdir, middle_objpoint.Y() + ydir);
                  pos2 = Position2D(middle_objpoint.X(), middle_objpoint.Y());
                  line2d = Line2D(pos1, pos2);
                  for (segment=segments.begin(); segment!=segments.end(); segment++){
                     if (segment->Intersect(line2d, int_point, 0.1)){
                        intersect = true;
                     }
                  }

//                  if ((dist1 < max_dist && dist2 < max_dist) // ){  // || (dist > 50 && (dist1 < 3*max_dist && dist2 < 3*max_dist))//){ 
                  if (intersect //(dist1 < max_dist && dist2 < max_dist) // ){  // || (dist > 50 && (dist1 < 3*max_dist && dist2 < 3*max_dist))//){ 
                                                                 || (dist > 50 && (dist1 < 0.4*dist && dist2 < 0.4*dist) 
                                                                 && (fabs(local_direction - dir) > 20 && fabs(local_direction - dir) < 160))){//||
//                      (fabs(local_direction - dir) > 30 && fabs(local_direction - dir) < 150)){//|| (dist1 < max_dist && dist2 < 2 * max_dist)){ 
                               printf("YESH, growing (%d) %d with %d!\n", growing, grow_line.Number(), map_line2->Number());        
                               growing++; growing_total++;
                               
                             //  if(index_line != -1) num_of_large++;
                               keep_number = grow_line.Number();
                               grow_line.Merge(*map_line2, merged_polygon2);
                               merged_line = merged_polygon2;
                               merged_line.Number()=keep_number;
                               
                               keep_growing = keep_map_lines[keep_map_lines.FindLine(map_line2->Number())];
                               keep_merge_line.Merge(keep_growing, merged_road_line3);
                               keep_merge_line = merged_road_line3;
                               keep_merge_line.Number() = keep_number;
                               map_line2->Label() = keep_number;
  //                             single_lines.push_back(*map_line2);
                               history_lines.push_back(*map_line2);
                               used_lines.push_back(*map_line2);
              //                 line = LineTopology(n, keep_number, grow_line.Number(), map_line2->Number());
                               nnn++;// merge_graph.push_back(line);  
                               sel_map_point = middle_objpoint;
                               sel_map_point.Number() = nnn; //map_line2->Number();
                             //  sel_map_point.Z() = map_point->Z();
                            //  sel_map_point.Z() = mean_direction * 180/PI;
                               sel_map_point.Z() = 100 * sin (local_direction* PI /180) ; //100 * sin (local_direction * PI /180);
                               merge_graph_points.push_back(sel_map_point);
                               new_merged_line = *map_line2;
                               new_merged_line.Number() = nnn;
                               new_merged_lines.push_back(new_merged_line);
                               
                               middle_point.Attribute(LabelTag) = merged_line.Number();
                               dir_points.push_back(middle_point);
                               if (growing_total == 1) {
                                   new_merged_line = grow_line;
                                   nnn++;new_merged_line.Number() = nnn;
                                   new_merged_lines.push_back(new_merged_line);              
                                   sel_map_point = *(middle_objpoints.PointIterator(keep_number));
                                   sel_map_point.Number() = nnn;
                                   sel_map_point.Z() = 100 * sin(intdir* PI /180); 
                                   merge_graph_points.push_back(sel_map_point);
                               }
                               //Now let us look if the laser data fits to the growing polygon
                               index_line = single_lines.FindLine(map_line2->Number()); 
                               temp_height_points.ErasePoints();
                               
                               // select the nearest points (<max_dist_grow m) withing the growing polygon
                               for (laser_point=grow_laser_points.begin(); laser_point!=grow_laser_points.end(); laser_point++){
                                    dist = (laser_point->vect2D()-virtual_point.vect2D()).Length();                                          
                                    if (dist < max_dist_grow) {
                                       temp_height_points.push_back(*laser_point);
                                    }
                                }
                                printf("Number of laser height points in growing segment (in radius/all) %d/%d:\n", 
                                        temp_height_points.size(), grow_laser_points.size());
                                //calculate the height at connection point
                               dominant_segment = temp_height_points.MostFrequentAttributeValue(SegmentNumberTag, count);
                               printf ("Ratio of biggest segment / all points in radius: %d/%d\n", count, temp_height_points.size());
                                                               potential_points.ErasePoints();
                               potential_points.AddTaggedPoints(sel_pos_points, map_line2->Number(), PolygonNumberTag);
         //                        keep_laser_points.RemoveTaggedPoints(map_line2->Number(), PolygonNumberTag); 
                               if (index_line != -1) { //if potential polygon is large than add largest segment
                                   temp_height_points.AddTaggedPoints(sel_pos_points, map_line2->Number(), PolygonNumberTag); //test, just add all points
                                   grow_laser_points.AddTaggedPoints(sel_pos_points, map_line2->Number(), PolygonNumberTag);
                      //             hit_large_polygon = true;
                                 }
                                 else {
                               if (count>8){
                                 printf("Segment size before %d ", temp_height_points.size());
                         //        
                                 temp_height_points.AddTaggedPoints(potential_points, dominant_segment, SegmentNumberTag); // add points of same segment to growing points in radius
                                 grow_laser_points.AddTaggedPoints(potential_points, dominant_segment, SegmentNumberTag); // add points of same segment to all growing laserpoints
                                 printf("and after growing %d (1)", temp_height_points.size());
                                 plane = temp_height_points.FitPlane(dominant_segment, dominant_segment, SegmentNumberTag);
                                 potential_points.RemoveTaggedPoints(dominant_segment, SegmentNumberTag); //Remove the points allready added from the potential points
                                 for (laser_point=potential_points.begin(); laser_point!=potential_points.end(); laser_point++){
                                    if (fabs(plane.Distance(laser_point->Position3DRef())) <  max_dist_seed){
                                        temp_height_points.push_back(*laser_point);
                                        grow_laser_points.push_back(*laser_point);            
                                    }
                                 }
                                 printf("and %d (2) \n", temp_height_points.size());
                                                                     
/*                                    dist = (laser_point->vect2D()-virtual_point.vect2D()).Length();                                          
                                    if (dist < 70) {
                                       temp_height_points.push_back(*laser_point);
                                    }
                                }
                        /*         a = plane.Z_At(virtual_point.X(), virtual_point.Y(), &success);
                                 ml = temp_height_points.Mean();
                                 printf("Calculated height = %f, mean = %f, potential height = ", a, ml[2]);
                                 potential_points.ErasePoints();
                                 potential_points.AddTaggedPoints(sel_pos_points, map_line2->Number(), PolygonNumberTag);

                          */    }
                                }
                                 }
                  }                             
                  }
              }
            }
        }
      }
     } while (growing > 0 );//&& growing_total < 10); 
    // single_lines.push_back(orig_line);  
    grow_laser_points.Label(merged_line.Number());
    grow_laser_points.SetAttribute(PolygonNumberTag, merged_line.Number());   
    tot_height_points.AddTaggedPoints(grow_laser_points, merged_line.Number(), PolygonNumberTag);      
    grow_laser_points.Label(2001); //set label to road label
    grow_laser_points.SetAttribute(PolygonNumberTag, merged_line.Number());
    keep_laser_points.AddTaggedPoints(grow_laser_points, merged_line.Number(), PolygonNumberTag); 
  }
  for (map_line2 = new_merged_lines.begin(); map_line2 != new_merged_lines.end(); map_line2++){
      for (map_line3 = map_line2+1; map_line3 != new_merged_lines.end(); map_line3++){
          if (map_line2->Adjacent(*map_line3, 3)) {
              line =  LineTopology(next_line, 2006, map_line2->Number(), map_line3->Number());
              merge_graph.push_back(line);                               
              next_line++;
          }
      }    
  }
 // if (growing_total > 0) {
                    merged_road_lines.push_back(merged_line);
                    output_lines.push_back(keep_merge_line);
   //                 }
}
 /* sel_laser_points.ErasePoints();
  sel_laser_points.AddTaggedPoints(sel_pos_points, 2001, LabelTag);
  printf("Size of sel_laser_points: %d\n", sel_laser_points.size());
  for (map_line = merged_road_lines.begin(); map_line != merged_road_lines.end(); map_line++){
      for (laser_point = sel_laser_points.begin(); laser_point != sel_laser_points.end(); laser_point++) {
           if (laser_point->InsidePolygon(map_points, map_line->LineTopologyReference())) {
               laser_point->Attribute(LabelTag) = map_line->Number();                                       
           }
      }
  }
   */
   LaserPoints sel_laser_points2;
   sel_laser_points.ErasePoints();
   single_lines.erase(single_lines.begin(), single_lines.end());
   for (map_line = sel_map_lines.begin(); map_line!=sel_map_lines.end(); map_line++) {
       findline = map_line->Number();
       index_line = used_lines.FindLine(findline);
       if (index_line == -1) { //Line not used in merging algorithm
          sel_laser_points.ErasePoints();
          sel_laser_points.AddTaggedPoints(sel_pos_points, map_line->Number(), PolygonNumberTag);
          if (sel_laser_points.size()<3){                  //polygon not merged && no laser points ->problems with reconstruction, try to merge anyway
              for (map_line2 = sel_map_lines.begin(); map_line2!=sel_map_lines.end(); map_line2++) {
                  sel_laser_points2.ErasePoints();
                  if (map_line2->Adjacent(*map_line, 2)){
                      sel_laser_points2.AddTaggedPoints(sel_pos_points, map_line2->Number(), PolygonNumberTag);
                         if (sel_laser_points2.size()>5) {
                             sel_laser_points2.SetAttribute(PolygonNumberTag, map_line->Number());                                                              
                        //     sel_laser_points.AddTaggedPoints(sel_laser_points2, map_line->Number(),PolygonNumberTag );
                             keep_laser_points.AddTaggedPoints(sel_laser_points2, map_line->Number(), PolygonNumberTag);
                             }
                   }
              }
          }                   
          single_lines.push_back(*map_line);
          keep_merge_line = keep_map_lines[keep_map_lines.FindLine(findline)];
          output_lines.push_back(keep_merge_line);
         keep_laser_points.AddTaggedPoints(sel_pos_points, map_line->Number(), PolygonNumberTag); 
       }
   }
   
   dxffile = fopen ("mergeroads.dxf","w");
//    keep_map_points.WriteDXF(dxffile, output_lines);
    graph_points.WriteDXF(dxffile, graph_lines);
    fclose(dxffile);
   for (map_line = keep_map_lines.begin(); map_line!=keep_map_lines.end(); map_line++) {
       if (map_line->TOP10MajorClass() != TOP10_Road ){
          output_lines.push_back(*map_line);
          keep_laser_points.AddTaggedPoints(sel_pos_points, map_line->Number(), PolygonNumberTag); 
       }
   }  
  
   
//   if (!tot_height_points.Write(laser_output, false))
//   if (!sel_laser_points.Write(laser_output, false))
     if (!keep_laser_points.Write(laser_output, false))
        printf("Error writing laser points.\n");
//   if (!middle_objpoints.Write(map_points_output))
//     if (!graph_points.Write(map_points_output))
//     if (!common_points.Write(map_points_output))
   if (!keep_map_points.Write(map_points_output))
//   if (!map_points.Write(map_points_output))
//   if (!connect_points.Write(map_points_output))
      printf("Error writing the 3D map points\n");                
//    if (!new_merged_lines.Write(map_topology_output))
 //     if (!graph_lines.Write(map_topology_output))
//      if (!common_parts.Write(map_topology_output))
//    if (!merge_graph.Write(map_topology_output))
//    if (!single_lines.Write(map_topology_output))
    if (!output_lines.Write(map_topology_output))
//    if (!merged_road_lines.Write(map_topology_output))
   // if (!sel_map_lines.Write(map_topology_output))
      printf("Error writing the topology of the 3D map lines\n"); 
      
    
//  map_points.WriteDXFMesh(dxffile, sel_map_lines, 5, true, true, true);
//    map_points.WriteDXF(dxffile, sel_map_lines);
                 
  }
            
      
// Old text
 /*     if (i!=0) { // Double polygon found
            if (debug) printf("\nNumber of directions needed: %d.", sel_map_points2.size());
            if (debug) printf("\n");
            if(!sel_graph_lines.empty()) sel_graph_lines.erase(sel_graph_lines.begin(),sel_graph_lines.end());  
               for (map_line2 = sel_map_lines.begin(); map_line2!=sel_map_lines.end(); map_line2++){  
                 if (map_line->Adjacent(*map_line2, 2) && !map_line->Adjacent(*map_line2 , map_line->size()-1)){
                     sel_graph_lines.push_back(*map_line2);
                 }
               }
            printf ("# of neighbours: %d (pn %d ln %d)\n", sel_graph_lines.size(), map_point->Number(),map_line->Number());            
            if (sel_map_points2.size()==2 && sel_graph_lines.size()==4) { //found interchange
               k = 0; kk = 0;
                sel_graph_lines[0].Label() = map_line->Number();
     //           history_lines.push_back(sel_graph_lines[0]);
      //          history_lines.push_back(*map_line);
               for (j=1; j!=sel_graph_lines.size(); j++) {
                  if (!sel_graph_lines[0].Adjacent(sel_graph_lines[j], 1)){
//                    merged_road_line = *map_line;
  //                  map_line->Merge(sel_graph_lines[0], merged_road_line);
  //                  merged_road_line.Merge(sel_graph_lines[j], merged_road_line2);
  //                  merged_road_line2.Number()=map_line->Number();
                    index_line = sel_map_lines.FindLine(find_line);
                    if (sel_map_lines[index_line].TOP10Invisible()) history_lines.push_back(sel_map_lines[index_line]);
                    //sel_map_lines[index_line] = merged_road_line2; 
  //                  merged_road_lines.push_back(merged_road_line2);
                    sel_graph_lines[j].Label() = map_line->Number(); 
       //             history_lines.push_back(sel_graph_lines[j]);            
                }
                    else {
                       if (k==0) {k = j;}
                       else {kk = j;}
                    }
                }
                index_line = sel_map_lines.FindLine(findline);
                if (sel_map_lines[index_line].TOP10Invisible()) history_lines.push_back(sel_map_lines[index_line]);
 //               sel_map_lines[index_line].Merge(sel_graph_lines[k], merged_road_line3);
 //               merged_road_line3.Merge(sel_graph_lines[kk], merged_road_line4);
 //               merged_road_line4.Number()=sel_map_lines[index_line].Number();
     //           sel_map_lines[index_line] = merged_road_line4;
    //            history_lines.push_back(sel_graph_lines[k]);
    //            history_lines.push_back(sel_graph_lines[kk]);
//                sel_map_lines.erase(index_line); if (map_line > index_line) map_line--;
//                sel_map_lines.push_back(merged_road_line4);
            }
      }
   *///}
   
    /*   for (map_point=graph_points.begin(); map_point!=graph_points.end(); map_point++){
       count=0;
       for (map_point2=graph_points.begin(); map_point2!=graph_points.end(); map_point2++){
           if (map_point2->Number()==map_point->Number()){
               count++;
           }
       }
       if (count==1) {
           printf ("Point %d endpoint\n", map_point->Number());
           end_points.push_back(*map_point);
           dir_point.SetPointNumber(map_point->Number());
           dir_point.X()= map_point->X();
           dir_point.Y()= map_point->Y();
           dir_point.Z()= 0;//map_point->Z();
 //          dir_points.push_back(dir_point);
       }
   }
*/
   //Hier maandag verder gaan...

