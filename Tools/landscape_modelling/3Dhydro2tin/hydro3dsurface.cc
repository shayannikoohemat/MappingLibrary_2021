
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
 Date   : 12-10-2011

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
#include "VectorPoint.h"
#include "VRML_io.h"
#include "dxf.h"
#include "TIN.h"


/*
--------------------------------------------------------------------------------
                           The main function
--------------------------------------------------------------------------------
*/

void Hydro3DSurface(char *laser_input, char *map_points_input,
	       char *map_topology_input, 
           char *map_points_output, char *map_topology_output, 
           char *vrml_file, char *laser_output, double default_grid_size, bool top10version,
           bool rasterise_points, bool produce_grid, bool produce_grid_ID, bool only_output_slivers)

{
  LaserPoints                  laser_points, sel_laser_points, nb_laser_points, dtm_laser_points,
                               copy_nb_laser_points, save_points, holes_laser, final_points,
                               building_laser_points, dtmtest, dtmtest2, temp_height_points,
                               poly_laser_points, loc_laser_points, addedpoints, sel_addedpoints,
                               new_laserpoints,suspicious_laser_points, simplifiedpoints;
  LaserPoint                   nb_laser_point, mean, temp_point, point;
  LaserPoints::iterator        laser_point, node;
  ObjectPoints                 map_points, sel_map_points, all_tin_points,
                               meadow_points, road_points, holes, keep_map_points,
                               new_road_points, dtm_points, new_tin_points, new_map_points,
                               dtm_tin_points, filtered_points, edge_points, all_edge_points,
                               copy_sel_map_points, common_points, holes_points, water_points,
                               dddmap_points, dummy_points,
                               all_tin_building_points, building_map_points, obj,
                               null_map_points, hole_points, all_messedup_points,
                               only_boundary_points, all_wall_points;
  LineTopologies               map_lines, map_tin_lines, all_tin_lines,
                               one_map_line, all_road_lines, all_water_lines,
                               all_building_lines, all_meadow_lines,
                               hidden_tin_lines, keep_map_lines, new_polygons,
                               all_shadow_lines, dtm_tin_lines, edge_lines, all_edge_lines,
                               common_parts, building_map_lines, all_tin_building_lines, *top,
                               hole_lines, inner_rings, all_messedup_lines, all_wall_lines,sel_map_lines2,
                               sel_map_lines3;
  LineTopology                 new_polygon, edge_line, common_part, common_line1, common_line2;
  ObjectPoints::iterator       map_point, this_point, previous_point, next_point, map_point2;
  ObjectPoint                  mean_point, new_road_point, new_map_point, sel_map_point, new_point,
                               null_map_point, hole_point;
  LineTopologies::iterator     map_line, last_map_line, map_line2, hole_line, map_line_ben;
  LineTopology::iterator       node2, node3, nb_node;
  LineTopsIterVector           sel_map_lines;
  PointNumberList::iterator    node_pnl, previous_node, next_node, next_node1, noden;;
  PointNumberList              neighbourhood;
  PointNumber                  *node_ptr, *ptn, pt1, pt2;
  int                          i, ii, iii, number_offset, countzmin, next_pnr, next_pnr2, 
                               count, nearest_laser_point, index2, index1,
                               pol_num,j,jj,n, edge_size, edgelines_size, number_offset2,
                               changed, success, iter, holeornot, check, addedlaserpoint,
                               change, change2;
  TIN                          tin, dtmtin, *mytin, *tin2;
  //TIN::const_iterator          mesh;
  TINMesh                      *mesh;
  MeshNumber                   *m;
  FILE                         *fd, *fdimage, *dxffile, *benfile;
  Image                        kernel, large_kernel;
  TINEdges                     edges, new_edges, edges2, new_edges2, *edges_3d, edges3, edges22, poledges;
  float                        dist, meanx, meany, meanz, max_diff, diff, dist1, dist2, height,
                               remember_height;
  double                       highestbuildingpoint;
  vector<int>                  point_count;
  bool                         found, done, changedb, pointinhole, pointinside, calculate_quality = false;
  DataBounds3D                 bounds;
  Plane                        plane;
  Position3D                   pos;
  Position3D p1,p2,p3;
  Position2D                   centroid;
  
  double min_dist              = 0.1;
  double min_dist2             = 0.4;
  double max_road_slope        = 0.1;
  double max_road_slope2        = 0.2;
  double max_vegetation_slope  = 0.3;
  double max_height_difference = 1.5;
  double neighbourhood_radius  = 10.0;
  double nbh_radius            = 10.0;
  double max_dist_grow         = 40;
  double connect_dist           = 4;
  
  bool debug=true;
  bool buildings=false;
  bool dtm_surface=false;
//  bool rasterise_points = false;
  double grid_size, dx, dy;
 // double default_grid_size = 3;
  int min_x, max_x, min_y, max_y, min_z, max_z, numcols, numrows, x, y;
  
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
 //   printf("Error: Make3DSurface requires laser points with SegmentNumberTags\n");
 //   return;
  }
  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
//    if (map_line->TOP10MajorClass() == TOP10_Building) {
  //  if (map_line->Number() != 1211) {
  //    map_lines.erase(map_line);
  //    map_line--;
  //    count++;
   // }
  }
 if (produce_grid){
                  printf("a grid will be produced at the end\n");
                   }
  /* dtm_tin_points.erase(dtm_tin_points.begin(), dtm_tin_points.end());
   dtm_tin_points = laser_points.ConstructObjectPoints();
   dtmtin = dtm_tin_points.Triangulate();
   dtm_tin_lines = LineTopologies(dtmtin);
   dxffile = fopen ("laserpointsTIN.dxf","w");
   dtm_tin_points.WriteDXF(dxffile, dtm_tin_lines);
   fclose(dxffile);
   return;
  */
laser_points.RemoveAlmostDoublePoints(true, 0.3);
if (only_output_slivers) laser_points.ErasePoints();
for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
     // Find objects with all points at -100 m                                    
       int countzmin=0, count = 0;
       
       for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
           count++;
           if ((map_points.PointIterator(*node2))->Z() == -100.0) {
                countzmin++;
            }
       }    
       if (count==countzmin){
           map_lines.erase(map_line);
           map_line--; 
       }
}
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
*/ObjectPoints clean_map_points;
for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
         for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
           clean_map_points.push_back(*(map_points.PointIterator(*node2)));
         }
      }

  
  clean_map_points.RemoveDoublePoints(map_lines, 0.01);
  map_points = clean_map_points;
  clean_map_points.erase(clean_map_points.begin(), clean_map_points.end());
  
bool set_building_number_tag = false;
  map_line = map_lines.begin();
  if (!map_line->HasAttribute(BuildingNumberTag)) set_building_number_tag=true;
  
 if (set_building_number_tag){
  printf("Start setting building number tag = map line number...\n");
  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++, count++) {
       printf("%7d  %5.1f \r", count, 100.0 * count / map_lines.size());
       map_line->Attribute(BuildingNumberTag) = map_line->Number();
    }
}
  
  if (debug) printf("start\n");
  
    // Densify the map points
  printf("done\nDensifying map points ..."); fflush(stdout);
  count = map_points.size();
 // map_lines.Densify(map_points, 0.5*grid_size); 
  printf("done, map points increased from %d to %d\n",
         count, map_points.size());
  // Merge points within 0.1 m
  printf("Removing double points ..."); fflush(stdout);
 // map_points.RemoveDoublePoints(map_lines, 0.1);
  printf("done, %d map points left\n", map_points.size());
 // laser_points.ReduceData(0.5);
  map_points.RemoveDoublePoints(map_lines, 0.01);
  
  next_pnr = map_points.HighestPointNumber().Number()+1;

  ii=1; 
  edge_size=0; edgelines_size=0; pt1=0; pt2=0; n=0, next_pnr2=0;
  
  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (!map_line->IsClosed()) {
      map_lines.erase(map_line);
      map_line--;
      count++;
    }
  }
  printf("Removed %d not closed polygons\n", count);

  //printf("Removed %d not closed polygons\n", count);
//return;
/*for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
         for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
           null_map_point = *(map_points.PointIterator(*node2));
           null_map_point.Z() = 0;
           null_map_points.push_back(null_map_point);
         }
  }
  printf("Put all map points to zero, test");
  
  map_points = null_map_points;
*/

dxffile = fopen ("curbs.dxf","w");
 for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (map_line->TOP10MajorClass() == TOP10_Road) {
      if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end());
      if(!map_tin_lines.empty()) map_tin_lines.erase(map_tin_lines.begin(),map_tin_lines.end());
      for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
         sel_map_point = *(map_points.PointIterator(*node2));
         sel_map_points.push_back(sel_map_point);
      }
      one_map_line.erase(one_map_line.begin(), one_map_line.end());
      one_map_line.push_back(*map_line);
      sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
      one_map_line.ReNumber(sel_map_points, 0, 0);
//      tin = sel_map_points.Triangulate(one_map_line);
//      map_tin_lines = LineTopologies(tin);
//      tin.Erase();
      number_offset = sel_map_points.HighestPointNumber().Number()+1;
      sel_map_points.DuplicateWithFixedOffset(-0.2, number_offset);
      map_tin_lines.AddTINWalls(one_map_line, number_offset);
      map_tin_lines.SetAttribute(LineLabelTag, 5);
      map_tin_lines.SetAttribute(IDNumberTag, map_line->Attribute(IDNumberTag));
      if (!all_wall_points.empty())
         map_tin_lines.ReNumber(sel_map_points, all_wall_points.size()+1, all_wall_lines.size()+1);
      all_wall_lines.insert(all_wall_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
      all_wall_points.insert(all_wall_points.end(), sel_map_points.begin(), sel_map_points.end());
    }
  }
    all_wall_points.WriteDXF(dxffile, all_wall_lines);
    
fclose(dxffile);
  all_wall_points.Write("curbs.objpts");
  all_wall_lines.Write("curbs.top", false);
    

dxffile = fopen ("3dpolygons.dxf","w");
    map_points.WriteDXF(dxffile, map_lines);
fclose(dxffile);
bool do_surface_reco = true;
if (do_surface_reco){
//return;
  iii=0;
  benfile = fopen("tins.txt", "w");
  inner_rings = map_lines.SelectAttributedLines(HoleTag, 1);
  for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
      time_t start;
   //   time( &start );
      check = -1;
      //If label indicate, building or water object, do not add laser points, otherwise add laser points
//      fprintf(benfile, "mapline %d\n", map_line->Number());
      
    hole_lines.erase(hole_lines.begin(), hole_lines.end());
    hole_points.erase(hole_points.begin(), hole_points.end());
    holes_laser.ErasePoints();
    holeornot = map_line->Attribute(HoleTag);
    found = false;
    if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end());
    if(!map_tin_lines.empty()) map_tin_lines.erase(map_tin_lines.begin(),map_tin_lines.end());
    if (map_line->Attribute(IDNumberTag) != 0 ) hole_lines = inner_rings.SelectAttributedLines(IDNumberTag, map_line->Attribute(IDNumberTag));
    if (holeornot ==0){

       if (hole_lines.size()>0){
 //        found = true;
         for (hole_line=hole_lines.begin(); hole_line!=hole_lines.end(); hole_line++) {
             centroid = hole_line->CalculateCentroid(map_points);
             hole_point = ObjectPoint(centroid.X(), centroid.Y(), 0.0, next_pnr, 0,0,0,0,0,0);
             found = true;
             point = LaserPoint(hole_point.X(), hole_point.Y(), 0);
             printf("X,Y, = %10.2f, %10.2f\n", point.X(), point.Y());
             if (!point.InsidePolygonJordan(map_points,
                                         hole_line->LineTopologyReference())){
                 found = false;
                 printf("NOT FOUND!!!!!!!!!!!!!!!!!\n");
                 iter = 0;
                 do {
                    point.X()=point.X()+0.1;
                    iter++;
     //               printf(" %d", iter);
                    } while (!point.InsidePolygonJordan(map_points,
                                         hole_line->LineTopologyReference())&& iter<2000);
                 if (iter > 1999){
                     iter = 0;
                     point.X() = hole_point.X();
                     do {
                        point.X()=point.X()-0.1;
                        iter++;
     //                   printf(" %d", iter);
                        } while (!point.InsidePolygonJordan(map_points,
                                         hole_line->LineTopologyReference())&& iter<2000);
                 }
                 if (iter > 1999){
                     iter = 0;
                     point.X() = hole_point.X();
                     point.Y() = hole_point.Y();
                     do {
                        point.Y()=point.Y()-0.1;
                        iter++;
        //                printf(" %d", iter);
                        } while (!point.InsidePolygonJordan(map_points,
                                         hole_line->LineTopologyReference())&& iter<2000);
                 }
                 if (iter > 1999){
                     iter = 0;
                     point.Y() = hole_point.Y();
                     do {
                        point.Y()=point.Y()+0.1;
                        iter++;
         //               printf(" %d", iter);
                        } while (!point.InsidePolygonJordan(map_points,
                                         hole_line->LineTopologyReference())&& iter<2000);
                 }
               
                 if (iter<1999) {
                               found = true;
                               hole_point = ObjectPoint(point.X(), point.Y(), 0.0, next_pnr, 0,0,0,0,0,0);
                               next_pnr ++;   
                               }
                 else {printf(" still not found, map line %d\n", map_line->Number());
                      one_map_line.erase(one_map_line.begin(), one_map_line.end());
                      one_map_line.push_back(*map_line);
                      for (node2=map_line->begin(); node2!=map_line->end(); node2++) {      
                          sel_map_points.push_back(*(map_points.PointIterator(*node2)));
                          }
                      one_map_line.Write("test_holenotfound.top", false);
                      sel_map_points.Write("test_holenotfound.objpts");                 
                      return;
                      }
             }  
                
             
             if (found){
             holes_laser.push_back(point);
             hole_points.push_back(hole_point);
             for (node2=hole_line->begin(); node2!=hole_line->end()-1; node2++) {
                 sel_map_points.push_back(*(map_points.PointIterator(*node2)));
               }
             }
         }
         }
      if (debug) printf("polygon %d containing %d holes\n", map_line->Number(), hole_lines.size());
      holes_laser.Write("holes.laser", false);  
 //     if ((map_line->TOP10MajorClass() == TOP10_Road) || (map_line->TOP10MajorClass() == TOP10_Water) 
     if ((map_line->TOP10MajorClass() == TOP10_Water) 
      || map_line->TOP10MajorClass() == TOP10_Building ||
      (map_line->TOP10MajorClass() == TOP10_Road && top10version)){
         if (debug) printf("Label = %d, area %4.2f\n", map_line->Label(),map_line->CalculateArea(map_points));
         //add all map points
         highestbuildingpoint = -100;
         for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
           sel_map_point = *(map_points.PointIterator(*node2));
           if (map_line->TOP10MajorClass() == TOP10_Building){
               sel_map_point.Z() += 10;
               if (sel_map_point.Z()>highestbuildingpoint) highestbuildingpoint = sel_map_point.Z();
               } 
            sel_map_points.push_back(sel_map_point);
         }
         
         //Make LineTopologies with one polygon
         one_map_line.erase(one_map_line.begin(), one_map_line.end());
         one_map_line.push_back(*map_line);
         if (found) one_map_line.insert(one_map_line.end(), hole_lines.begin(), hole_lines.end());             
         if (debug) printf("map line pushed back\n"); 
         sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
         one_map_line.ReNumber(sel_map_points, 0, 0);
         one_map_line.Write("thisisthecurrentmapline.top", false);
         sel_map_points.Write("thisisthecurrentmapline.objpts");

         if (debug) printf("map line renumbered, size of sel_map_points %d\n", sel_map_points.size());
         if (hole_lines.size()>0 && found && hole_lines.size()==hole_points.size()){
           tin = sel_map_points.Triangulate(one_map_line, hole_points);
           }
         else tin = sel_map_points.Triangulate(one_map_line);
         
//         holes_laser.Write("holes.laser", false);    
         if (debug) printf("points triangulated\n");
         map_tin_lines = LineTopologies(tin);
         if (debug) printf("TIN line-topologised\n");
         tin.Erase();
         if (!map_line->TOP10MajorClass() == TOP10_Building){
         
         dtm_points.insert(dtm_points.end(), sel_map_points.begin(), sel_map_points.end());
         if (map_line->TOP10MajorClass() == TOP10_Road) road_points.insert(road_points.end(), sel_map_points.begin(), sel_map_points.end());                                  
         if (map_line->TOP10MajorClass() == TOP10_Water){
            water_points.insert(water_points.end(), sel_map_points.begin(), sel_map_points.end());            
            }                              
            for (node3=map_line->begin(); node3!=map_line->end(); node3++) {
               this_point = sel_map_points.PointIterator(*node3);
               nb_laser_point.X()=this_point->X();
               nb_laser_point.Y()=this_point->Y();
               nb_laser_point.Z()=this_point->Z();
               if (nb_laser_point.Z()>= -15) {
                   nb_laser_point.SetPointNumber(iii);
                   nb_laser_point.Attribute(LabelTag) = map_line->Label();                        
                   nb_laser_points.push_back(nb_laser_point); 
                   iii++;
                   }
             }
         }
          if (map_line->TOP10MajorClass() == TOP10_Building){ //buildings, add walls to z=-10
               number_offset = sel_map_points.HighestPointNumber().Number()+1;
               sel_map_points.DuplicateWithFixedOffset(-10.0, number_offset);
          //     map_tin_lines.AddWalls(one_map_line, number_offset);
               map_tin_lines.AddTINWalls(one_map_line, number_offset);
               for (this_point=sel_map_points.begin(); this_point!=sel_map_points.begin()+sel_map_points.size()/2; this_point++) {
                    this_point->Z() = highestbuildingpoint;
                    }
               map_tin_lines.ReNumber(sel_map_points, 0, 0);
          }

         map_tin_lines.SetAttribute(LineLabelTag, map_line->Attribute(LineLabelTag));
 //        map_tin_lines.SetAttribute(LineLabelTag, map_line->Attribute(IDNumberTag));
//         map_tin_lines.SetAttribute(LineLabelTag, holeornot);
         map_tin_lines.SetAttribute(IDNumberTag, map_line->Attribute(IDNumberTag));
         check = map_tin_lines.size() - sel_map_points.size() + 2 - 2*hole_lines.size();              
         if (hole_lines.size()!=hole_points.size()) check = 1;
         
         if (!all_tin_points.empty())
           map_tin_lines.ReNumber(sel_map_points, (all_tin_points.end()-1)->Number()+1, (all_tin_lines.end()-1)->Number()+1);
             if (!only_output_slivers) {
              all_tin_lines.insert(all_tin_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
              all_tin_points.insert(all_tin_points.end(), sel_map_points.begin(), sel_map_points.end());                                                                          
              }
              else {
                   if (check!=0){
              all_tin_lines.insert(all_tin_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
              all_tin_points.insert(all_tin_points.end(), sel_map_points.begin(), sel_map_points.end());                                                                          
                    }
             }
      }
      else
         {
         if (debug) printf("Label = %d\n", map_line->Label());
         // Add map points first
         only_boundary_points.erase(only_boundary_points.begin(), only_boundary_points.end());
         for (node2=map_line->begin(); node2!=map_line->end()-1; node2++) {      
              sel_map_points.push_back(*(map_points.PointIterator(*node2)));
              only_boundary_points.push_back(*(map_points.PointIterator(*node2)));
      //        fprintf(benfile, "%d %10.2f %10.2f %10.2f\n", (map_points.PointIterator(*node2))->Number(), (map_points.PointIterator(*node2))->X(),(map_points.PointIterator(*node2))->Y(),(map_points.PointIterator(*node2))->Z());
         }
            keep_map_points = sel_map_points;  
            one_map_line.erase(one_map_line.begin(), one_map_line.end());
            one_map_line.push_back(*map_line);
            if (found) one_map_line.insert(one_map_line.end(), hole_lines.begin(), hole_lines.end());
            sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
          
            sel_laser_points.ErasePoints();
//            sel_laser_points.AddTaggedPoints(laser_points, map_line->Number(), PolygonNumberTag);
//            sel_laser_points = laser_points.SelectTagValue(PolygonNumberTag, map_line->Number());
          if (map_line->HasAttribute(BuildingNumberTag)) sel_laser_points = laser_points.SelectTagValue(PolygonNumberTag, map_line->Attribute(BuildingNumberTag));
          else sel_laser_points = laser_points.SelectTagValue(PolygonNumberTag, map_line->Number());
          sel_laser_points.Write("thesearethecurrentlaserpoints.laser", false);
            
 //           if (map_line->TOP10MajorClass() == TOP10_Meadow){
           grid_size = default_grid_size;
           if (map_line->TOP10MajorClass() == TOP10_Meadow || (map_line->TOP10MajorClass() == TOP10_Road && !top10version)){                                            
   //        if (map_line->TOP10MajorClass() == TOP10_Meadow){                                            
            if (rasterise_points){
              if (map_line->TOP10MajorClass() == TOP10_Road) grid_size = 1;
              printf("map line: %d\n", map_line->Number());
              poledges.erase(poledges.begin(), poledges.end());
              poly_laser_points.ErasePoints();
              sel_laser_points.DeriveTIN();
              poledges.Derive(sel_laser_points.TINReference());
              sel_laser_points.DeriveDataBounds(0);
              max_x = int (sel_laser_points.DataBounds().Maximum().X());
              max_y = int (sel_laser_points.DataBounds().Maximum().Y());
              min_x = int (sel_laser_points.DataBounds().Minimum().X());
              min_y = int (sel_laser_points.DataBounds().Minimum().Y());
              min_z = int (sel_laser_points.DataBounds().Minimum().Z());
              max_z = int (sel_laser_points.DataBounds().Maximum().Z());
              numcols = int ((max_x-min_x)/grid_size)+1;
              numrows = int ((max_y-min_y)/grid_size)+1;
              count = 0;
              for (x=min_x; x<max_x; x+=int(grid_size)){
                  for (y=min_y; y<max_y; y+=int(grid_size)){
                      pos = Position3D(x,y,0);
                      neighbourhood.erase(neighbourhood.begin(), neighbourhood.end());
   //                   temp_point = LaserPoint(x, y, 0);
       //               printf("x: %d, y: %d ", x,y);
                      nearest_laser_point = sel_laser_points.NearestPoint(pos,
                                            poledges, true);
                      loc_laser_points.ErasePoints();
                      nbh_radius = 3;
                      do{
                        neighbourhood = sel_laser_points.Neighbourhood(PointNumber(nearest_laser_point),
                                       nbh_radius, poledges,
                                       true, true);
                                       nbh_radius = nbh_radius*2;
                      } while (neighbourhood.size()<3 && nbh_radius<50);
                      if (neighbourhood.size() > 3){
                      for (noden=neighbourhood.begin(); noden!=neighbourhood.end(); noden++){
                           loc_laser_points.push_back(sel_laser_points[noden->Number()]);
                      }
                      temp_point = LaserPoint(x, y, loc_laser_points.Mean().Z());
                      temp_point.Attribute(PolygonNumberTag) = map_line->Number();
                      temp_point.Attribute(PolygonNumberTag) = map_line->Attribute(BuildingNumberTag);
                      
                      temp_point.Attribute(LabelTag)         = map_line->Label();
                      if (map_line->HasAttribute(IDNumberTag)) temp_point.Attribute(PlaneNumberTag) = map_line->Attribute(IDNumberTag);

                      pointinside = false;
                     /*  if (temp_point.InsidePolygon(map_points,
                                         map_line->LineTopologyReference())) {            
                         poly_laser_points.push_back(temp_point);
                         pointinside = true;
                       }
                      */ if (temp_point.InsidePolygonJordan(map_points,
                                         map_line->LineTopologyReference())==1) {            
                         poly_laser_points.push_back(temp_point);
                         pointinside = true;
                       }
                       
/*                       int c = 0;
                       for (i = 0, j = only_boundary_points.size()-1; i < only_boundary_points.size(); j = i++) {
                     //      printf("i = %d (%d), j = %d (%d)", i, sel_map_points[i].Number(), j, sel_map_points[j].Number());
                           if ( ((only_boundary_points[i].Y()>y) != (only_boundary_points[j].Y()>y)) &&
                            	 (x < (only_boundary_points[j].X()-only_boundary_points[i].X()) * (y-only_boundary_points[i].Y()) / (only_boundary_points[j].Y()-only_boundary_points[i].Y()) + only_boundary_points[i].X()) )
                                  c = !c;
                       }
                      printf("c: %d, selmappoints: %d \n", c, only_boundary_points.size());
                      if (c==0) temp_point.Attribute(LabelTag) = 0;
  */                   // if (pointinside)  suspicious_laser_points.push_back(temp_point);
                       
                       }
                  }
              
              }
              sel_laser_points = poly_laser_points;
            }                                
                                            
            
            }
            printf("Number = %d, laser points %d\n", map_line->Number(), sel_laser_points.size());
            sel_laser_points.Write("thesearethecurrentlaserpoints.laser", false);
              addedlaserpoint = 0;
//          for (i=0;i<sel_laser_points.size();i++) {
            change2 = sel_laser_points.size();
//            sel_laser_points.ReduceData(1);
            sel_addedpoints.ErasePoints();
            change2 = change2 - sel_laser_points.size();
            for (i=0;i<sel_laser_points.size();i++) {              
              // remove the points inside the holes....
              pointinhole = false;
              if (i<sel_laser_points.size()){
               for (hole_line=hole_lines.begin(); hole_line!=hole_lines.end() && !pointinhole; hole_line++) {
                  if (sel_laser_points[i].InsidePolygonJordan(map_points,
                                         hole_line->LineTopologyReference())==1) pointinhole = true;
               }     
              //Add sel_laser_points to sel_map_points
                if (!pointinhole) {
      //             sel_map_points.push_back(ObjectPoint(sel_laser_points[i].vect(), next_pnr, Covariance3D()));
      //             next_pnr ++;
      //             addedlaserpoint++;
                   sel_addedpoints.push_back(sel_laser_points[i]);
                }
              }
          }   
          simplifiedpoints.ErasePoints();
   //       sel_addedpoints.ReduceData(0.1);
   //       if (map_line->TOP10MajorClass() == TOP10_Meadow) simplifiedpoints = sel_addedpoints.SimplifyMesh(0.1);
   //       if (map_line->TOP10MajorClass() == TOP10_Road) simplifiedpoints = sel_addedpoints.SimplifyMesh(0.05);
   //       if (simplifiedpoints.size()>0) sel_addedpoints = simplifiedpoints;
        printf("size of seladdedpoints %d\n", sel_addedpoints.size());      
          for (i=0;i<sel_addedpoints.size();i++) {              
     //          sel_map_points.push_back(ObjectPoint(sel_addedpoints[i].vect(), next_pnr, Covariance3D()));
     //          next_pnr ++;
          }          
          sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
          one_map_line.ReNumber(sel_map_points, 0, 0);
          keep_map_points = sel_map_points;
          loc_laser_points.ErasePoints();
          for (map_line_ben = one_map_line.begin(); map_line_ben!=one_map_line.end(); map_line_ben++){
            for (node2=map_line_ben->begin(); node2!=map_line_ben->end()-1; node2++) {      
               point = LaserPoint((sel_map_points.PointIterator(*node2))->X(),(sel_map_points.PointIterator(*node2))->Y(),(sel_map_points.PointIterator(*node2))->Z());
               point.SetAttribute(LabelTag, 1);

               loc_laser_points.push_back(point);
            }
          }
          for (i=0;i<sel_addedpoints.size();i++) {
               point = LaserPoint(sel_addedpoints[i].X(), sel_addedpoints[i].Y(), sel_addedpoints[i].Z());
               point.SetAttribute(LabelTag, 0);
               point.SetAttribute(PolygonNumberTag, map_line->Attribute(BuildingNumberTag));
               addedpoints.push_back(point);
               loc_laser_points.push_back(point);
          }                                          
         if (map_line->TOP10MajorClass() == TOP10_Meadow) simplifiedpoints = loc_laser_points.SimplifyMesh_KeepLabel(0.05, LabelTag, 1);
         if (map_line->TOP10MajorClass() == TOP10_Road) simplifiedpoints = loc_laser_points.SimplifyMesh_KeepLabel(0.02, LabelTag, 1);
          if (simplifiedpoints.size()>0) {
             sel_addedpoints = simplifiedpoints;
             simplifiedpoints.ErasePoints();
      //       addedpoints.AddPoints(sel_addedpoints);
             sel_addedpoints.RemoveTaggedPoints(1, LabelTag);
             }
          
          sel_map_points = keep_map_points;
          for (i=0;i<sel_addedpoints.size();i++) {              
               sel_map_points.push_back(ObjectPoint(sel_addedpoints[i].vect(), next_pnr, Covariance3D()));
               next_pnr ++;
          }          
          
          printf("renumbered");
          sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
          one_map_line.ReNumber(sel_map_points, 0, 0);
          
          one_map_line.Write("selmapline.top", false);
          sel_map_points.Write("selmapline.objpts");
          if (hole_lines.size()>0 && found && hole_lines.size()==hole_points.size()){
              tin = sel_map_points.Triangulate(one_map_line, hole_points);
              printf("triangulated_a");
              one_map_line.Write("test.top", false);
              sel_map_points.Write("test.objpts");
       //       
              one_map_line.Write("test2.top", false);
              sel_map_points.Write("test2.objpts");            
           }
           else {
              tin = sel_map_points.Triangulate(one_map_line);
              printf("triangulated_b");
           }
           for (map_line_ben = one_map_line.begin(); map_line_ben!=one_map_line.end(); map_line_ben++){
                 fprintf(benfile, "pointsonboundary\n");
                 for (node2=map_line_ben->begin(); node2!=map_line_ben->end(); node2++) {      
                    fprintf(benfile, "%10.2f %10.2f %10.2f\n", (sel_map_points.PointIterator(*node2))->X(),(sel_map_points.PointIterator(*node2))->Y(),(sel_map_points.PointIterator(*node2))->Z());
                 }
                }
                fprintf(benfile, "pointsinsideboundary\n");
                for (i=0;i<sel_addedpoints.size();i++) {
                    fprintf(benfile, "%10.2f %10.2f %10.2f\n", sel_addedpoints[i].X(), sel_addedpoints[i].Y(), sel_addedpoints[i].Z());
                }                                          
              if (debug)printf("Map points triangulated\n");   
              
              map_tin_lines = LineTopologies(tin);
              check = map_tin_lines.size() - sel_map_points.size() + 2 - 2*hole_lines.size();
              
              if (hole_lines.size()!=hole_points.size()) check = 1;
       
              tin.Erase();
              if (debug)printf("TIN erased...\n");
          map_tin_lines.Write("maptinline.top", false);
          sel_map_points.Write("maptinline.objpts");
             
              map_tin_lines.SetAttribute(LineLabelTag, map_line->Attribute(LineLabelTag));
              map_tin_lines.SetAttribute(IDNumberTag, map_line->Attribute(IDNumberTag));
              if (!all_tin_points.empty()) map_tin_lines.ReNumber(sel_map_points, (all_tin_points.end()-1)->Number()+1, (all_tin_lines.end()-1)->Number()+1);
             if (!only_output_slivers) {
              all_tin_lines.insert(all_tin_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
              all_tin_points.insert(all_tin_points.end(), sel_map_points.begin(), sel_map_points.end());                                                                          
              }
              else {
                   if (check!=0){
              all_tin_lines.insert(all_tin_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
              all_tin_points.insert(all_tin_points.end(), sel_map_points.begin(), sel_map_points.end());                                                                          
                    }
             }
              time_t end;
//	          time( &end );
//		std::cout << "Done in " << difftime( end, start ) << " seconds." << std::endl;
        }
      ii++;
      if (map_line->TOP10MajorClass() == TOP10_Road){
         all_road_lines.insert(all_road_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
      }
      if (map_line->TOP10MajorClass() == TOP10_Building){
         all_building_lines.insert(all_building_lines.end(), map_tin_lines.begin(), map_tin_lines.end());      
      }
      if (map_line->TOP10MajorClass() == TOP10_Water){
         all_water_lines.insert(all_water_lines.end(), map_tin_lines.begin(), map_tin_lines.end());      
      }
      if (map_line->TOP10MajorClass() == TOP10_Meadow){
         all_meadow_lines.insert(all_meadow_lines.end(), map_tin_lines.begin(), map_tin_lines.end());  
      }
      }
      }
//      i=0;
       printf("%d, %d\n", iii, nb_laser_points.size());
  //     return;
      iii=nb_laser_points.size();
      all_messedup_points.Write("messeduppoints.objpts");
      all_messedup_lines.Write("messeduppoints.top", false);
      addedpoints.Write("allmodelpoints.laser", false);
      suspicious_laser_points.Write("suspiciouslaserpoints.laser", false);
          if (!all_tin_points.Write(map_points_output))
    printf("Error writing the 3D TIN points\n");
    if (!all_tin_lines.Write(map_topology_output, false))
    printf("Error writing the topology of the 3D TIN lines\n");
dxffile = fopen ("dxffile2.dxf","w");
    all_tin_points.WriteDXFMesh(dxffile, all_road_lines, 9, false, true, false, true);
    all_tin_points.WriteDXFMesh(dxffile, all_water_lines, 5, false, false, false, true);
    all_tin_points.WriteDXFMesh(dxffile, all_building_lines, 7, false, false, false, true);
    all_tin_points.WriteDXFMesh(dxffile, all_meadow_lines, 3, false, false, true, true);
 // all_tin_points.WriteDXFMesh(dxffile, all_shadow_lines, 4, false, false, false, true);  
   //    dtm_tin_points.WriteDXFMesh(dxffile, dtm_tin_lines, 6, false, false, true, true);  
    fclose(dxffile);
    fclose(benfile);
    if (calculate_quality){
                           LaserPoints quality_laser_points;
                           int largeresidual, largeresidualonroads, roadpoints;
                           roadpoints=largeresidualonroads=largeresidual=0;
                           
        if (!map_lines.Read(map_topology_input)) {
            printf("Error reading map lines from file %s\n", map_topology_input);
              exit(0);
              }
              if (!laser_points.Read(laser_input)) {
               printf("Error reading laser points from file %s\n", laser_input);
                exit(0);
                 }
        printf("Calculating residuals on laser points\n");
        laser_points.ReduceData(0.5);
        for (map_line=map_lines.begin(), count=0; map_line!=map_lines.end(); map_line++, count++) {
        sel_laser_points.ErasePoints();
        printf("%d\n", count);
        sel_laser_points.AddTaggedPoints(laser_points, map_line->Attribute(IDNumberTag), PlaneNumberTag);
        sel_map_lines3 = all_tin_lines.SelectAttributedLines(IDNumberTag, map_line->Attribute(IDNumberTag));
        sel_map_lines2 = sel_map_lines3.SelectAttributedLines(LineLabelTag, map_line->Attribute(LineLabelTag));
 //       sel_laser_points2.ErasePoints();
 //       sel_laser_points2.AddTaggedPoints(final_points, map_line->Attribute(IDNumberTag), PlaneNumberTag);
    //       printf("\n%d %4.2f", count, map_lines.size()/count);
        for (laser_point=sel_laser_points.begin(), index1=0;
         laser_point!=sel_laser_points.end(); laser_point++, index1++) {
               for (map_line2=sel_map_lines2.begin(), found = false;
                    map_line2!=sel_map_lines2.end() && !found; map_line2++) {
                    if (laser_point->InsidePolygon(all_tin_points,
                                         map_line2->LineTopologyReference())) {
                      nb_node=map_line2->begin();
                      p1 = Position3D(*(all_tin_points.PointIterator(*nb_node))); nb_node++;
                      p2 = Position3D(*(all_tin_points.PointIterator(*nb_node))); nb_node++;
                      p3 = Position3D(*(all_tin_points.PointIterator(*nb_node)));
                      plane = Plane(p1, p2, p3);
                      laser_point->Residual() = fabs(plane.Z_At(laser_point->X(), laser_point->Y(), &success)-laser_point->Z());
            //          laser_point->Z() = laser_point->Residual();
                      found = true;
                      quality_laser_points.push_back(*laser_point);
                      if (map_line->TOP10MajorClass() == TOP10_Road){
                        roadpoints++;
                        if (laser_point->Residual()>0.05) largeresidualonroads++;
                      }
                      if (map_line->TOP10MajorClass() != TOP10_Road){
                        if (laser_point->Residual()>0.05) largeresidual++;
                        }
                      }
               }
 //              printf("%d %4.2f\r", index1, sel_laser_points.size()/index1);
               
         }
//         printf("\n%d %4.2f", count, map_lines.size());
               
        }
        printf("laserpoints: %d\n, largeresiduals %d\n, pointsonroads %d\n, largeresidualsonroads %d\n",
                             quality_laser_points.size(), largeresidual, roadpoints, largeresidualonroads);
                if (!quality_laser_points.Write("qualitylaserpoints.laser", false))
        printf("Error writing all 3D map points in laser file format.\n");                

        }
//   bool produce_grid = false;

}
   if (produce_grid){
   bool found_large_polygon, found_tin;
   double                   xminc, xmaxc, yminc, ymaxc, dist;
   LaserPoints              grid_laser_points;
   grid_size = 0.5;
     if (!map_points.Read(map_points_input)) {
    printf("Error reading map points from file %s\n", map_points_input);
    exit(0);
  }
  if (!map_lines.Read(map_topology_input)) {
    printf("Error reading map lines from file %s\n", map_topology_input);
    exit(0);
  }
//  if (!laser_points.Read(laser_input)) {
//    printf("Error reading laser points from file %s\n", laser_input);
//    exit(0);
//  }
  map_points.RemoveDoublePoints(map_lines, 0.01);
  edges.erase(edges.begin(), edges.end());
 if (do_surface_reco) laser_points = addedpoints;
 else {
   if (!laser_points.Read(laser_input)) {
    printf("Error reading laser points from file %s\n", laser_input);
    exit(0);
  }
  }
  
  laser_points.DeriveTIN();
  edges.Derive(laser_points.TINReference());
     
  next_pnr = map_points.HighestPointNumber().Number()+1;

  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (!map_line->IsClosed()) {
      map_lines.erase(map_line);
      map_line--;
      count++;
    }
  }
  printf("Removed %d not closed polygons\n", count);
   map_point=map_points.begin();
   xminc = xmaxc = map_point->X();
   yminc = ymaxc = map_point->Y();
  for (map_point=map_points.begin();map_point!=map_points.end(); map_point++) {
      if (map_point->X() < xminc) xminc = map_point->X();
      if (map_point->X() > xmaxc) xmaxc = map_point->X();
      if (map_point->Y() < yminc) yminc = map_point->Y();
      if (map_point->Y() > ymaxc) ymaxc = map_point->Y();
  }
  printf ("%10.2f %10.2f %10.2f %10.2f ...\n", xminc, xmaxc, yminc, ymaxc);
  min_x = int (xminc);
double  frac_x = xminc - min_x;
  max_x = int (xmaxc);
  min_y = int (yminc);
double  frac_y = yminc - min_y;
  max_y = int (ymaxc);
  printf ("%10d %10d %10d %10d ...\n", min_x, max_x, min_y, max_y);
  last_map_line = map_lines.begin();
  sel_map_lines2 = all_tin_lines;
  printf("producing grid ...\n");
   for (dx=xminc; dx<xmaxc; dx=dx+grid_size){
    for (dy=yminc; dy<ymaxc; dy=dy+grid_size){
        temp_point = LaserPoint(dx, dy, 0);
        pos = Position3D(dx,dy, 0);
        printf("10.2%f %10.2f \n",dx,dy);
//  for (x=min_x; x<max_x; x+=int(grid_size)){
//    for (y=min_y; y<max_y; y+=int(grid_size)){
 //       temp_point = LaserPoint(x + frac_x, y+ frac_y, 0);
 //       pos = Position3D(x + frac_x, y+ frac_y, 0);
 //       printf("%d %d \n",x,y);
        found_large_polygon = false;
        found_tin = false;
        nearest_laser_point = laser_points.NearestPoint(pos,
                                            edges, true);
        nb_laser_point = laser_points[nearest_laser_point];
        dist = pos.Distance(nb_laser_point.vect());
        printf("label %d, dist %4.2f ", nb_laser_point.Attribute(PolygonNumberTag), dist);
        temp_point.Attribute(PlaneNumberTag) = nb_laser_point.Attribute(PolygonNumberTag);
  //      nb_laser_point.Attribute(PlaneNumberTag) = nb_laser_point.Attribute(PolygonNumberTag);
  //      sel_map_lines2.erase(sel_map_lines2.begin(), sel_map_lines2.end());
 //       sel_map_lines2 = all_tin_lines.SelectAttributedLines(IDNumberTag, nb_laser_point.Attribute(PlaneNumberTag));
          
        printf("size %d ", sel_map_lines2.size());
        for (map_line2=sel_map_lines2.begin(), found_tin = false;
             map_line2!=sel_map_lines2.end() && !found_tin; map_line2++) {
             if (map_line2->Attribute(IDNumberTag)==nb_laser_point.Attribute(PlaneNumberTag)){
             if (temp_point.InsidePolygonJordan(all_tin_points,
                                         map_line2->LineTopologyReference())) {
                plane.Initialise(); 
                nb_node=map_line2->begin();
     //           printf("%d ",last_map_line->Number());
                p1 = Position3D(*(all_tin_points.PointIterator(*nb_node))); nb_node++;
                p2 = Position3D(*(all_tin_points.PointIterator(*nb_node))); nb_node++;
                p3 = Position3D(*(all_tin_points.PointIterator(*nb_node)));
                plane = Plane(p1, p2, p3);
                printf("lmn %4.2f \n",plane.Z_At(temp_point.X(), temp_point.Y(), &success));
                temp_point.Z() = plane.Z_At(temp_point.X(), temp_point.Y(), &success);
                found_tin = true;
                temp_point.Attribute(PlaneNumberTag) = map_line2->Attribute(IDNumberTag);
                temp_point.Attribute(LabelTag) = map_line2->Attribute(LineLabelTag);
 
             //         last_map_line = map_line;
                }
          }
          }
          if (!found_tin && dist<20){
        for (map_line2=sel_map_lines2.begin(), found_tin = false;
             map_line2!=sel_map_lines2.end() && !found_tin; map_line2++) {
             if (temp_point.InsidePolygonJordan(all_tin_points,
                                         map_line2->LineTopologyReference())) {
                plane.Initialise(); 
                nb_node=map_line2->begin();
     //           printf("%d ",last_map_line->Number());
                p1 = Position3D(*(all_tin_points.PointIterator(*nb_node))); nb_node++;
                p2 = Position3D(*(all_tin_points.PointIterator(*nb_node))); nb_node++;
                p3 = Position3D(*(all_tin_points.PointIterator(*nb_node)));
                plane = Plane(p1, p2, p3);
                printf("lmn %4.2f \n",plane.Z_At(temp_point.X(), temp_point.Y(), &success));
                temp_point.Z() = plane.Z_At(temp_point.X(), temp_point.Y(), &success);
                found_tin = true;
                temp_point.Attribute(PlaneNumberTag) = map_line2->Attribute(IDNumberTag);
                temp_point.Attribute(LabelTag) = map_line2->Attribute(LineLabelTag);
             //         last_map_line = map_line;
                }
          }
         }
 /*       if (temp_point.InsidePolygonJordan(map_points,
                   last_map_line->LineTopologyReference())==1) {
                   printf("\nlm%d ",last_map_line->Number());
                   sel_map_lines2.erase(sel_map_lines2.begin(), sel_map_lines2.end());
                   sel_map_lines2 = all_tin_lines.SelectAttributedLines(IDNumberTag, last_map_line->Attribute(IDNumberTag));
//                   printf("%d ",sel_map_lines3.size());
//                   sel_map_lines2 = sel_map_lines3.SelectAttributedLines(LineLabelTag, last_map_line->Attribute(LineLabelTag));
                   printf("%d ",sel_map_lines2.size());
                   found_tin = true;
       
                   for (map_line2=sel_map_lines2.begin(), found_tin = false;
                    map_line2!=sel_map_lines2.end() && !found_tin; map_line2++) {
                    if (temp_point.InsidePolygonJordan(all_tin_points,
                                         map_line2->LineTopologyReference())) {
                      plane.Initialise(); 
                      nb_node=map_line2->begin();
                      printf("%d ",last_map_line->Number());
                      p1 = Position3D(*(all_tin_points.PointIterator(*nb_node))); nb_node++;
                      p2 = Position3D(*(all_tin_points.PointIterator(*nb_node))); nb_node++;
                      p3 = Position3D(*(all_tin_points.PointIterator(*nb_node)));
                      plane = Plane(p1, p2, p3);
                      printf("lmn %4.2f \n",plane.Z_At(temp_point.X(), temp_point.Y(), &success));
                      temp_point.Z() = plane.Z_At(temp_point.X(), temp_point.Y(), &success);
                      found_tin = true;
             //         last_map_line = map_line;
                    }
                   }
        }
        if (!found_tin) {
        for (map_line=map_lines.begin(); map_line!=map_lines.end() && !found_tin; map_line++) {
            printf("nm%10d \r",map_line->Number());
           if (temp_point.InsidePolygonJordan(map_points,
                   map_line->LineTopologyReference())==1) {            
                   printf("\n%d ",map_line->Number());
                   found_large_polygon = true;
                   found_tin = true;last_map_line = map_line;
                   sel_map_lines2.erase(sel_map_lines2.begin(), sel_map_lines2.end());
                   sel_map_lines2 = all_tin_lines.SelectAttributedLines(IDNumberTag, map_line->Attribute(IDNumberTag));
                   printf("%d ",sel_map_lines2.size());
       //            sel_map_lines2 = sel_map_lines3.SelectAttributedLines(LineLabelTag, map_line->Attribute(LineLabelTag));
       //            printf("%d ",sel_map_lines2.size());

/*                   for (map_line2=sel_map_lines2.begin(), found_tin = false;
                    map_line2!=sel_map_lines2.end() && !found_tin; map_line2++) {
                    if (temp_point.InsidePolygonJordan(all_tin_points,
                                         map_line2->LineTopologyReference())) {
                      plane.Initialise(); 
                      nb_node=map_line2->begin();
                      printf("fndtin %d ",map_line2->Number());
                      p1 = Position3D(*(all_tin_points.PointIterator(*nb_node))); nb_node++;
                      p2 = Position3D(*(all_tin_points.PointIterator(*nb_node))); nb_node++;
                      p3 = Position3D(*(all_tin_points.PointIterator(*nb_node)));
                      plane = Plane(p1, p2, p3);
                      printf("nmn%4.2f \n",plane.Z_At(temp_point.X(), temp_point.Y(), &success));
                      temp_point.Z() = plane.Z_At(temp_point.X(), temp_point.Y(), &success);
                      found_tin = true;
                      last_map_line = map_line;
                    }
                   }
           }
           }
        }
  */      if (found_tin) {
     //              temp_point.Attribute(PolygonNumberTag) = last_map_line->Number();
     //              temp_point.Attribute(LabelTag)         = last_map_line->Label();
    //               if (last_map_line->HasAttribute(IDNumberTag)) temp_point.Attribute(PlaneNumberTag) = last_map_line->Attribute(IDNumberTag);
                   printf("point added\n");
                       grid_laser_points.push_back(temp_point);
         //              grid_laser_points.push_back(nb_laser_point);
                       }
               else{
                temp_point.Attribute(PlaneNumberTag) = 99999;
                temp_point.Attribute(LabelTag) = 99999;
                grid_laser_points.push_back(temp_point);
                        }
    }    
  }
grid_laser_points.Write("gridlaserpoints.laser", false);
}

   if (produce_grid_ID){
   bool found_large_polygon, found_tin;
   double                   xminc, xmaxc, yminc, ymaxc, dist;
   LaserPoints              grid_laser_points_ID;
   grid_size = 0.5;
     if (!map_points.Read(map_points_input)) {
    printf("Error reading map points from file %s\n", map_points_input);
    exit(0);
  }
  if (!map_lines.Read(map_topology_input)) {
    printf("Error reading map lines from file %s\n", map_topology_input);
    exit(0);
  }
//  if (!laser_points.Read(laser_input)) {
//    printf("Error reading laser points from file %s\n", laser_input);
//    exit(0);
//  }
  map_points.RemoveDoublePoints(map_lines, 0.01);
  edges.erase(edges.begin(), edges.end());
  if (do_surface_reco) laser_points = addedpoints;
 else {
   if (!laser_points.Read(laser_input)) {
    printf("Error reading laser points from file %s\n", laser_input);
    exit(0);
  }
  }
  
  laser_points.DeriveTIN();
  edges.Derive(laser_points.TINReference());
     
  next_pnr = map_points.HighestPointNumber().Number()+1;

  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (!map_line->IsClosed()) {
      map_lines.erase(map_line);
      map_line--;
      count++;
    }
  }
  printf("Removed %d not closed polygons\n", count);
   map_point=map_points.begin();
   xminc = xmaxc = map_point->X();
   yminc = ymaxc = map_point->Y();
  for (map_point=map_points.begin();map_point!=map_points.end(); map_point++) {
      if (map_point->X() < xminc) xminc = map_point->X();
      if (map_point->X() > xmaxc) xmaxc = map_point->X();
      if (map_point->Y() < yminc) yminc = map_point->Y();
      if (map_point->Y() > ymaxc) ymaxc = map_point->Y();
  }
  printf ("%10.2f %10.2f %10.2f %10.2f ...\n", xminc, xmaxc, yminc, ymaxc);
  min_x = int (xminc);
double  frac_x = xminc - min_x;
  max_x = int (xmaxc);
  min_y = int (yminc);
double  frac_y = yminc - min_y;
  max_y = int (ymaxc);
  printf ("%10d %10d %10d %10d ...\n", min_x, max_x, min_y, max_y);
  last_map_line = map_lines.begin();
  sel_map_lines2 = map_lines;
  printf("producing grid ...\n");
  for (dx=xminc; dx<xmaxc; dx=dx+grid_size){
    for (dy=yminc; dy<ymaxc; dy=dy+grid_size){
        temp_point = LaserPoint(dx, dy, 0);
        pos = Position3D(dx,dy, 0);
        printf("10.2%f %10.2f \n",dx,dy);
        found_large_polygon = false;
        found_tin = false;
        nearest_laser_point = laser_points.NearestPoint(pos,
                                            edges, true);
        nb_laser_point = laser_points[nearest_laser_point];
        dist = pos.Distance(nb_laser_point.vect());
        printf("label %d, dist %4.2f ", nb_laser_point.Attribute(PolygonNumberTag), dist);
        temp_point.Attribute(PlaneNumberTag) = nb_laser_point.Attribute(PolygonNumberTag);
  //      nb_laser_point.Attribute(PlaneNumberTag) = nb_laser_point.Attribute(PolygonNumberTag);
  //      sel_map_lines2.erase(sel_map_lines2.begin(), sel_map_lines2.end());
 //       sel_map_lines2 = all_tin_lines.SelectAttributedLines(IDNumberTag, nb_laser_point.Attribute(PlaneNumberTag));
          
        printf("size %d ", sel_map_lines2.size());
        for (map_line2=sel_map_lines2.begin(), found_tin = false;
             map_line2!=sel_map_lines2.end() && !found_tin; map_line2++) {
             if (map_line2->Attribute(IDNumberTag)==nb_laser_point.Attribute(PlaneNumberTag)){
             if (temp_point.InsidePolygonJordan(map_points,
                                         map_line2->LineTopologyReference())) {
                found_tin = true;
                temp_point.Attribute(PlaneNumberTag) = map_line2->Attribute(IDNumberTag);
                temp_point.Attribute(LabelTag) = map_line2->Attribute(LineLabelTag);
             //         last_map_line = map_line;
                }
          }
          }
          if (!found_tin && dist<20){
        for (map_line2=sel_map_lines2.begin(), found_tin = false;
             map_line2!=sel_map_lines2.end() && !found_tin; map_line2++) {
             if (temp_point.InsidePolygonJordan(map_points,
                                         map_line2->LineTopologyReference())) {
                found_tin = true;
                temp_point.Attribute(PlaneNumberTag) = map_line2->Attribute(IDNumberTag);
                temp_point.Attribute(LabelTag) = map_line2->Attribute(LineLabelTag);
              }
          }
         }
      if (found_tin) {
                   printf("point added\n");
                   grid_laser_points_ID.push_back(temp_point);
                   }
                   else{
                temp_point.Attribute(PlaneNumberTag) = 99999;
                temp_point.Attribute(LabelTag) = 99999;
                grid_laser_points_ID.push_back(temp_point);
                        }
    }    
  }
grid_laser_points_ID.Write("gridlaserpoints_ID.laser", false);
}
    
return;    
//      printf("i = %d\n", iii);
      printf("Filling holes beneath 3D roads... (this can take a while)\n");
      
      for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
      if (map_line->TOP10MajorClass() == TOP10_Road){ //create lower road parts
         new_polygon = *map_line;
         new_polygon.Attribute(LineLabelTag)=map_line->Attribute(IDNumberTag);
         new_polygon.Attribute(IDNumberTag)=map_line->Attribute(IDNumberTag);
         ii = 0;                         
         for (node2=new_polygon.begin(); node2!=new_polygon.end(); node2++) {
             this_point = map_points.PointIterator(*node2);
             new_road_point = *this_point;
             //new_road_point.Z()=-10;
             i=0;  
             for (map_point = dtm_points.begin(); map_point != dtm_points.end(); map_point++) {
                 dist = (map_point->vect2D() - this_point->vect2D()).Length();                                                     
                    if (dist<= min_dist2 && new_road_point.Z() >= map_point->Z() && map_point->Z()!=-100){
          //                     diff = fabs(map_point->Z()-this_point->Z());
                               i++;
                               ii++;
                               next_pnr++;
                               new_road_point = *map_point;
                               new_road_point.Number() = next_pnr;
                               new_road_point.Z() = map_point->Z();
                               node2->Number() = new_road_point.Number();
                               nb_laser_point.X()=map_point->X();
                               nb_laser_point.Y()=map_point->Y();
                               nb_laser_point.Z()=map_point->Z();
                               if (nb_laser_point.Z()>= -15) {
                                                       
                                  nb_laser_point.SetPointNumber(iii);   
                                  nb_laser_points.push_back(nb_laser_point); 
                                  iii++; 
                               }
                    }
             }
             if (i==0) new_road_point.Z() = -10;
             new_road_points.push_back(new_road_point);
         }
      if (ii!=0)   new_polygons.push_back(new_polygon);
      }
      }
 //     printf("i = %d\n", i);
      printf("Done\nProducing TIN of DTM...");  
 //     nb_laser_points.DeriveTIN();
      printf("Done\nProducing edges of TIN...");
 
   printf("Producing DTM laser point set...");
   printf("Deriving TIN ..."); fflush(stdout);
//  laser_points.DeriveTIN();
  printf("done\nDeriving TIN edges ..."); fflush(stdout);
//  edges.Derive(laser_points.TINReference());
     // iii = dtm_laser_points.size();
      iii=0;
      for (node=nb_laser_points.begin(); node!=nb_laser_points.end(); node++) {
           nb_laser_point= *node;                              
           nb_laser_point.SetPointNumber(iii);   
           final_points.push_back(nb_laser_point); iii++;   
      }
      for (node=dtm_laser_points.begin(); node!=dtm_laser_points.end(); node++) {
           nb_laser_point= *node;                                
           nb_laser_point.SetPointNumber(iii);   
           final_points.push_back(nb_laser_point);iii++; 
      }      
    final_points.DeriveTIN();
//    edges2.Derive(final_points.TINReference());  
//    final_points.FilterOnMorphology(large_kernel, 30, 0.1, 0.1, edges2);
//    printf("Filtered on Morphology\n");
//    final_points.RemoveFilteredPoints();      
//    final_points.DeriveTIN();
    edges2.Derive(final_points.TINReference());
    TINEdges *tin_edges, *edgesn;
    final_points.VerifyTIN();
    printf("1");
    tin_edges = new TINEdges();
    tin_edges->Derive(final_points.TINReference());
    printf(" 2");
    edgesn = final_points.NeighbourhoodEdges3D(tin_edges->TINEdgesRef(), connect_dist);
    printf(" 3");
    delete tin_edges;
    printf(" 4");
    final_points.LabelComponents(*edgesn, SegmentNumberTag);
    printf(" 5");
  //  top = edgesn->EdgeTopologies();
  //  obj = final_points.ConstructObjectPoints();  

    //return edges;
 //   if (!final_points.Write(laser_output, false))
 //       printf("Error writing all 3D map points in laser file format.\n");
//return;    
  
    if (dtm_surface) {
    printf("Started minimum filtering...\n");
     next_pnr = 0;
      for (i=0;i<final_points.size();i++) {
          printf("%d (%d) \r", i, final_points.size());
          sel_laser_points.ErasePoints();
          nearest_laser_point = final_points.NearestPoint(final_points[i].Position3DRef(), edges2, true);
          neighbourhood = final_points.Neighbourhood(PointNumber(nearest_laser_point), nbh_radius, edges2, true, true);
          for (node_pnl=neighbourhood.begin(); node_pnl!=neighbourhood.end(); node_pnl++) {
               sel_laser_points.push_back(final_points[node_pnl->Number()]);
          }
          sel_laser_points.DeriveDataBounds(0);
          remember_height = final_points[i].Z();
          if (sel_laser_points.DataBounds().Minimum().Z()<=final_points[i].Z()){
              final_points[i].Z() = sel_laser_points.DataBounds().Minimum().Z();
              }
              else {
                   printf("%6.2f is higher than %6.2f\n", final_points[i].Z(), sel_laser_points.DataBounds().Minimum().Z());
                   }
            dtmtest.push_back(final_points[i]);
            final_points[i].Z()=remember_height;
      }
      printf("\ndone.\n");
      dtmtest.DeriveTIN();
      printf("Maximum filtering...\n");
      edges3.Derive(dtmtest.TINReference());
      for (i=0;i<dtmtest.size();i++) {
          printf("%d (%d) \r", i, dtmtest.size());
          sel_laser_points.ErasePoints();
          nearest_laser_point = dtmtest.NearestPoint(dtmtest[i].Position3DRef(), edges3, true);
          neighbourhood = dtmtest.Neighbourhood(PointNumber(nearest_laser_point), nbh_radius, edges3, true, true);
          for (node_pnl=neighbourhood.begin(); node_pnl!=neighbourhood.end(); node_pnl++) {
               sel_laser_points.push_back(dtmtest[node_pnl->Number()]);
          }
          sel_laser_points.DeriveDataBounds(0);
          remember_height = dtmtest[i].Z();
          dtmtest[i].Z() = sel_laser_points.DataBounds().Maximum().Z();
          dtmtest2.push_back(dtmtest[i]);
          dtmtest[i].Z()=remember_height;
          next_pnr++;
      }
      printf("\ndone.\n");
      dtmtest2.DeriveTIN();
      edges22.Derive(dtmtest2.TINReference());
      dtm_tin_points = dtmtest2.ConstructObjectPoints();
      dtmtest2.RemoveLongEdges(edges22, 20);
      dtmtin = * (dtmtest2.DeriveTIN());
      dtm_tin_lines = LineTopologies(dtmtin);
      // begin test 
    for (map_line=dtm_tin_lines.begin(); map_line!=dtm_tin_lines.end(); map_line++) {
        count=0;
        meanx = 0;
        meany = 0;
        meanz = 0;
        dist1=0;
       for (nb_node=map_line->begin(); nb_node!=map_line->end(); nb_node++) { //double pass for starting point ruled out
         next_node=map_line->NextNode(nb_node);
         next_point = dtm_tin_points.PointIterator(*next_node);
         this_point = dtm_tin_points.PointIterator(*nb_node);
         dist = (this_point->vect2D() - next_point->vect2D()).Length();
//         count++;
//         meanx = (meanx*(count-1) + this_point->X())/count;
//         meany = (meany*(count-1) + this_point->Y())/count;
//         meanz = (meanz*(count-1) + this_point->Z())/count;
         
         if (dist > dist1) dist1 = dist;
      }
      if (dist1 > 40) { //Remove long edges
   //         printf("Map line %d ", map_line->Number());
            dtm_tin_lines.erase(map_line);
            map_line--;
   //         printf("erased.\n");
         }
 //        dist1 = 0;
         nb_laser_point.X()=meanx;//this_point->X();
         nb_laser_point.Y()=meany;//this_point->Y();
         nb_laser_point.Z()=meanz;
    //     printf("Point: %10.2f , %10.2f, %4.2f\n", meanx, meany, meanz);
         //*laser_point = nb_laser_point;
         for (map_line2=map_lines.begin(), found = false; map_line2!=map_lines.end() && !found; map_line2++) {
             if (map_line2->TOP10MajorClass() == TOP10_Road || map_line2->TOP10MajorClass() == TOP10_Water) {
                 if (nb_laser_point.InsidePolygon(map_points, 
                      map_line2->LineTopologyReference())) {
                      dtm_tin_lines.erase(map_line);
                      map_line--;
                      found = true;
                  }
             }
         }                                         
     }  
       
       } // end of bool dtm_surface          
       

    new_road_points.RemoveDoublePoints(new_polygons, 0.1);
//    seg_par.SurfaceModel()=1;
//    dtm_laser_points.SurfaceGrowing(seg_par);
   printf("Done\nCalculating new map point heights...");
   for (map_line=new_polygons.begin(); map_line!=new_polygons.end(); map_line++){
       for (map_line2=map_line +1; map_line2!=new_polygons.end();map_line2++){
           if ((*map_line).CommonEdge(*map_line2)){
     //         printf("Line %d connects to %d with", map_line->Number(), map_line2->Number());                                    
              common_part = (*map_line).CommonPart(*map_line2);
              for (node2=common_part.begin()+1; node2!=common_part.end()-1; node2++){
                  this_point = new_road_points.PointIterator(*node2);
   //               printf("point %d ", this_point->Number());
        //          this_point->Z()= -10;
              }
              common_parts.push_back(common_part);
   //           printf("\n");  
            }
       }
   }
  printf("Number of common parts = %d\n", common_parts.size());
  new_road_points.RemoveDoublePoints(new_polygons, 0.01);
  next_pnr = new_road_points.HighestPointNumber().Number()+1;
  for (map_line = common_parts.begin(); map_line != common_parts.end(); map_line++) {
      for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
             this_point = new_road_points.PointIterator(*node2);
             common_points.push_back(*this_point);
      }
  }
//  common_points.Write(map_points_output);
//  common_parts.Write(map_topology_output);
//    dtm_laser_points.DeriveTIN();
//    edges2.Derive(dtm_laser_points.TINReference());
    new_road_points.RemoveDoublePoints(new_polygons, 0.01);
    for (map_point=new_road_points.begin(); map_point!=new_road_points.end(); map_point++) {
      if (map_point->Z() == -10.0) {
//              printf("Height changed from %4.2f ", map_point->Z());
//         if (dtm_surface) {
//              nearest_laser_point = dtmtest2.NearestPoint(map_point->Position3DRef(),
//                                                    edges22, false);
//               map_point->Z() = dtmtest2[nearest_laser_point].Z();                       
//               }
 //         else {
               nearest_laser_point = final_points.NearestPoint(map_point->Position3DRef(),
                                     edges2, false);
//               nearest_laser_point = nb_laser_points.NearestPoint(map_point->Position3DRef(),
//                                     edges22, false);
//               map_point->Z() = nb_laser_points[nearest_laser_point].Z();                                                            
               map_point->Z() = final_points[nearest_laser_point].Z();                       
 //              }

  //             printf("to %4.2f \n", map_point->Z());
      // Find a line with this point number
/*      map_line = new_polygons.begin();
      while ((node_pnl = map_line->NodeIterator(map_point->Number())) ==
              map_line->end()) map_line++;
      // Find an earlier point
      count = 0;
      do {
        count++;
        node_pnl = map_line->PreviousNode(node_pnl);
        previous_point = new_road_points.PointIterator(*node_pnl);
      } while ((previous_point->Z() == 450.0 || previous_point->Z() == -100.0) && count < map_line->size());
      if (count == map_line->size()) continue; // No point with valid height
      // Find a later point
      node_pnl = map_line->NodeIterator(map_point->Number());
      do {
        node_pnl = map_line->NextNode(node_pnl);
        next_point = new_road_points.PointIterator(*node_pnl);
      } while (next_point->Z() == 450.0 || next_point->Z() == -100);
      // Take a weighted height
      dist1 = (map_point->vect2D() - previous_point->vect2D()).Length();
      dist2 = (map_point->vect2D() - next_point->vect2D()).Length();
      map_point->Z() = (dist1 * next_point->Z() +
                        dist2 * previous_point->Z()) / (dist1 + dist2);
//      printf("to %4.2f \n", map_point->Z());                  
  //   }
 */     }
  }
  
                           
  for (map_point=new_road_points.begin(); map_point!=new_road_points.end(); map_point++) {
      for (map_point2=road_points.begin(); map_point2!=road_points.end(); map_point2++) {
          dist2 = (map_point->vect2D() - map_point2->vect2D()).Length();
          if  (dist2 < min_dist2 && (map_point->Z()>map_point2->Z())){
 //             printf("New road point height changed from %4.2f ", map_point->Z());
              map_point->Z()=map_point2->Z();
 //             printf("to %4.2f.\n", map_point->Z());
          }
      }
  }
  /*        
  for (map_point=dtm_tin_points.begin(); map_point!=dtm_tin_points.end(); map_point++) {
      for (map_point2=all_tin_points.begin(); map_point2!=all_tin_points.end(); map_point2++) {
          dist2 = (map_point->vect2D() - map_point2->vect2D()).Length();
          if (map_point2->Z()!=-2.0){
            if  (dist2 < min_dist && (map_point->Z()>map_point2->Z())){
              printf("DTM point height changed from %4.2f ", map_point->Z());
              map_point->Z()=map_point2->Z();
              printf("to %4.2f.\n", map_point->Z());
            } 
          }
      }
  }       
  */
  for (map_point=water_points.begin(); map_point!=water_points.end(); map_point++) {
      for (map_point2=meadow_points.begin(); map_point2!=meadow_points.end(); map_point2++) {
          dist2 = (map_point->vect2D() - map_point2->vect2D()).Length();
          if  (dist2 < 2* min_dist && (map_point->Z()<map_point2->Z())){
//              printf("Meadow point height changed from %4.2f ", map_point2->Z());
              map_point2->Z()=map_point->Z();
//              printf("to %4.2f.\n", map_point2->Z());
          }
      }
  }        
    

      // Remove jumps to higher roads    


//               __  
// First remove /  \ shape
     for (map_line=new_polygons.begin(); map_line!=new_polygons.end(); map_line++) {
  //    printf("Label = %d\n", map_line->Label());
      node_pnl=map_line->begin();
      map_point = new_road_points.PointIterator(*node_pnl);
      previous_node=map_line->PreviousNode(node_pnl);
      previous_point = new_road_points.PointIterator(*previous_node); 
      dist1 = (map_point->vect2D() - previous_point->vect2D()).Length();
      next_node1=map_line->NextNode(node_pnl);
      next_node=map_line->NextNode(next_node1);
      for (; node_pnl!=map_line->end()-1; next_node1=map_line->NextNode(node_pnl)) {
        next_node=map_line->NextNode(next_node1);
        next_point = new_road_points.PointIterator(*next_node);
        dist2 = (map_point->vect2D() - next_point->vect2D()).Length();
        if ((map_point->Z() - previous_point->Z() ) / dist1 > max_vegetation_slope &&
            (map_point->Z() - next_point->Z()) / dist2 > max_vegetation_slope) {
           map_point->Z() = (dist1 * next_point->Z() +
                             dist2 * previous_point->Z()) / (dist1 + dist2);
        }
        previous_node = node_pnl;      previous_point = map_point;
        node_pnl          = next_node; map_point      = next_point;
        dist1         = dist2;
      }
    }
     for (map_line=new_polygons.begin(); map_line!=new_polygons.end(); map_line++) {
      changed = 0;
 //     do {          
  //    printf("Label = %d\n", map_line->Label());
      node_pnl=map_line->begin();
      changedb = false;
      map_point = new_road_points.PointIterator(*node_pnl);
      previous_node=map_line->PreviousNode(node_pnl);
      previous_point = new_road_points.PointIterator(*previous_node); 
      dist1 = (map_point->vect2D() - previous_point->vect2D()).Length();
      next_node=map_line->NextNode(node_pnl);
      for (; node_pnl!=map_line->end()-1; next_node=map_line->NextNode(node_pnl)) {
        next_point = new_road_points.PointIterator(*next_node);
        dist2 = (map_point->vect2D() - next_point->vect2D()).Length();
        if ((map_point->Z() - previous_point->Z() ) / dist1 > max_vegetation_slope &&
            (map_point->Z() - next_point->Z()) / dist2 > max_vegetation_slope) {
           map_point->Z() = (dist1 * next_point->Z() +
                             dist2 * previous_point->Z()) / (dist1 + dist2);
           changed++;
           changedb = true;
        }
        previous_node = node_pnl;      previous_point = map_point;
        node_pnl          = next_node; map_point      = next_point;
        dist1         = dist2;
      }
  //    } while (changedb && changed <4);
    }
  for (map_line=new_polygons.begin(); map_line!=new_polygons.end(); map_line++) {
         holeornot = 0;
         if (map_line->Attribute(HoleTag)>0) holeornot = 1;
         if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end());
         if(!map_tin_lines.empty()) map_tin_lines.erase(map_tin_lines.begin(),map_tin_lines.end());
         //add all map points
         for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
           sel_map_points.push_back(*(new_road_points.PointIterator(*node2)));
         }
         //Make LineTopologies with one polygon                               
         one_map_line.erase(one_map_line.begin(), one_map_line.end());
         one_map_line.push_back(*map_line); 
         sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
         one_map_line.ReNumber(sel_map_points, 0, 0);
         tin = sel_map_points.Triangulate(one_map_line);
         map_tin_lines = LineTopologies(tin);
         tin.Erase();
         map_tin_lines.SetAttribute(LineLabelTag, map_line->Attribute(LineLabelTag));
         map_tin_lines.SetAttribute(LineLabelTag, map_line->Attribute(IDNumberTag));
         map_tin_lines.SetAttribute(LineLabelTag, holeornot);
         map_tin_lines.SetAttribute(IDNumberTag, map_line->Attribute(IDNumberTag));
         if (!all_tin_points.empty())
           map_tin_lines.ReNumber(sel_map_points, (all_tin_points.end()-1)->Number()+1, (all_tin_lines.end()-1)->Number()+1);
         all_tin_lines.insert(all_tin_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
         all_tin_points.insert(all_tin_points.end(), sel_map_points.begin(), sel_map_points.end());
         all_shadow_lines.insert(all_shadow_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
  } 

     
    dtm_tin_points.erase(dtm_tin_points.begin(), dtm_tin_points.end());
      dtm_tin_points = final_points.ConstructObjectPoints();
      dtmtin = dtm_tin_points.Triangulate();
      dtm_tin_lines = LineTopologies(dtmtin);
    printf("Done\n Writing output\n");
    fd = VRML2_Open(vrml_file);
//    laser_points.VRML_Write(fd, 2, true); 
//     if (dtm_surface) dtm_tin_points.VRML2_Write(fd, dtm_tin_lines, 1, 0.9, 0.9, 0.9);

//    all_tin_points.VRML2_Write(fd, all_tin_lines, 1, 1.0, 1.0, 0.8);
//    all_tin_points.VRML2_Write(fd, all_shadow_lines, 1, 0, 0.5, 0); //dark green
//    all_tin_points.VRML2_Write(fd, all_shadow_lines, 1, 0, 1, 0); //green
//    dtm_tin_points.VRML2_Write(fd, dtm_tin_lines, 1, 0.3, 0.3, 0.3);
    all_tin_points.VRML2_Write(fd, all_meadow_lines, 1, 0, 1.0, 0); //green
//    all_tin_points.VRML2_Write(fd, all_building_lines, 1, 0.9, 0.9, 0.9); // light grey
    if (buildings) all_tin_building_points.VRML2_Write(fd, all_tin_building_lines, 1, 1, 0, 0); 
    all_tin_points.VRML2_Write(fd, all_road_lines, 1, 0.5, 0.5, 0.5); //dark grey
    all_tin_points.VRML2_Write(fd, all_water_lines, 1, 0, 0, 1.0); //blue
//    all_tin_points.VRML2_Write(fd, all_road_lines, 1, 0.5, 0.5, 0.5); //dark grey
/*    all_tin_points.VRML2_Write(fd, all_meadow_lines, 1, 0.9, 0.9, 0.9); // light grey
    all_tin_points.VRML2_Write(fd, all_water_lines, 1, 0.9, 0.9, 0.9); // light grey
    all_tin_points.VRML2_Write(fd, all_shadow_lines, 1, 0.9, 0.9, 0.9); // light grey        
*///    all_tin_points.VRML2_Write(fd, hidden_tin_lines, 1, 0, 1.0, 0); //green
//    nb_laser_points.LaserPointsTIN2Vrml(fd, edges, 1, 0, 1.0, 0);

      VRML2_Close(fd);
      
    dxffile = fopen ("dxffile2.dxf","w");
    all_tin_points.WriteDXFMesh(dxffile, all_road_lines, 9, false, true, false, true);
    all_tin_points.WriteDXFMesh(dxffile, all_water_lines, 5, false, false, false, true);
    all_tin_points.WriteDXFMesh(dxffile, all_meadow_lines, 3, false, false, true, true);
 // all_tin_points.WriteDXFMesh(dxffile, all_shadow_lines, 4, false, false, false, true);  
   //    dtm_tin_points.WriteDXFMesh(dxffile, dtm_tin_lines, 6, false, false, true, true);  
    fclose(dxffile);
 //   all_road_lines.insert(all_road_lines.end(), all_shadow_lines.begin(), all_shadow_lines.end());
//    if (!new_road_points.Write(map_points_output))
    if (!all_tin_points.Write(map_points_output))
//    if (!dtm_tin_points.Write(map_points_output))
    printf("Error writing the 3D TIN points\n");
//    if (!all_road_lines.Write(map_topology_output))
    if (!all_tin_lines.Write(map_topology_output, false))
//    if (!new_polygons.Write(map_topology_output,false))
//    if (!dtm_tin_lines.Write(map_topology_output))
    printf("Error writing the topology of the 3D TIN lines\n");

    if (!final_points.Write(laser_output, false))
        printf("Error writing all 3D map points in laser file format.\n");
}    



