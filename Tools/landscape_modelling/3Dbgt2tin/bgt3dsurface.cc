
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

void bgt3DSurface(char *laser_input, char *map_points_input,
	       char *map_topology_input, 
           char *map_points_output, char *map_topology_output, 
           double default_grid_size,
           bool calculate_quality,
           bool debug, int reductionfactor)

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
                               only_boundary_points, all_wall_points, hole_map_points;
  LineTopologies               map_lines, map_tin_lines, all_tin_lines,
                               one_map_line, all_road_lines, all_water_lines,
                               all_building_lines, all_meadow_lines,
                               hidden_tin_lines, keep_map_lines, new_polygons,
                               all_shadow_lines, dtm_tin_lines, edge_lines, all_edge_lines,
                               common_parts, building_map_lines, all_tin_building_lines, *top,
                               hole_lines, inner_rings, all_messedup_lines, all_wall_lines,sel_map_lines2,
                               sel_map_lines3, notclosed_lines;
  LineTopology                 new_polygon, edge_line, common_part, common_line1, common_line2;
  ObjectPoints::iterator       map_point, this_point, previous_point, next_point, map_point2;
  ObjectPoint                  mean_point, new_road_point, new_map_point, sel_map_point, new_point,
                               null_map_point, hole_point;
  LineTopologies::iterator     map_line, last_map_line, map_line2, hole_line, map_line_bgt;
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
  TIN                          tin, dtmtin;
  //TIN::const_iterator          mesh;
//  TINMesh                      *mesh;
//  MeshNumber                   *m;
  FILE                         *dxffile;
  TINEdges                     edges, new_edges, edges2, new_edges2,edges3, edges22, poledges;
  double                        dist, meanx, meany, meanz, max_diff, diff, dist1, dist2, height,
                               remember_height;
  double                       highestbuildingpoint;
  vector<int>                  point_count;
  bool                         found, done, changedb, pointinhole, pointinside,
                               top10version = true,
                               rasterise_points = true;// calculate_quality = false;
  DataBounds3D                 bounds;
  Plane                        plane;
  Position3D                   pos, projpos,p1,p2,p3;
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
  
 // bool debug=true;
  bool buildings=false;
  bool dtm_surface=false;
//  bool rasterise_points = false;
  double grid_size;
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
  if (debug) printf("start\n");fflush(stdout);
  
  printf ("Reduction factor to decrease amount of print information to screen: %d.\n", reductionfactor);

  next_pnr = map_points.HighestPointNumber().Number()+1;

  ii=1; 
  edge_size=0; edgelines_size=0; pt1=0; pt2=0; n=0, next_pnr2=0;

  
  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (!map_line->IsClosed()) {
      notclosed_lines.push_back(*map_line);
      map_lines.erase(map_line);
      
      map_line--;
      count++;
    }
  }
  
 if (count!=0) printf("Removed %d not closed polygons\n", count);fflush(stdout);
 if (count == 0) printf("All polygons are closed\n");fflush(stdout);
 if (debug) notclosed_lines.Write("notclosedlines.top", false);
 if (debug) map_points.Write("notclosedlines.objpts");
//  return;
  
for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (map_line->CalculateArea(map_points) < 0.1) {
      map_lines.erase(map_line);
      map_line--;
      count++;
    }
  }
 if (count!=0)  printf("Removed %d polygons smaller than 0.1 m2\n", count); fflush(stdout); 
 if (count == 0) printf("All polygons are larger than 0.1 m2: continue with all.\n");fflush(stdout);


dxffile = fopen ("3dpolygons.dxf","w");
    map_points.WriteDXF(dxffile, map_lines);
fclose(dxffile);

//return;
  iii=0;
  inner_rings = map_lines.SelectAttributedLines(HoleTag, 1);
  for (map_line=map_lines.begin(), count=0; map_line!=map_lines.end(); map_line++, count++) {
      //If label indicate, building or water object, do not add laser points, otherwise add laser points
    if (debug || count == (count/reductionfactor)*reductionfactor) printf("processing line %d, ID %d, progress = %5.2f\r", count, map_line->Attribute(IDNumberTag), 100.0*count/map_lines.size());
    hole_lines.erase(hole_lines.begin(), hole_lines.end());
    hole_points.erase(hole_points.begin(), hole_points.end());
    holes_laser.ErasePoints();
    holeornot = map_line->Attribute(HoleTag);
    found = false;
    if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end());
    if(!map_tin_lines.empty()) map_tin_lines.erase(map_tin_lines.begin(),map_tin_lines.end());
    if (map_line->Attribute(IDNumberTag) != 0 ) hole_lines = inner_rings.SelectAttributedLines(IDNumberTag, map_line->Attribute(IDNumberTag));
    if (debug) printf("polygon %d (hole = %d) containing %d holes\n", map_line->Attribute(IDNumberTag), holeornot, hole_lines.size());
    if (holeornot ==0){

       if (hole_lines.size()>0){
 //        found = true;
         for (hole_line=hole_lines.begin(); hole_line!=hole_lines.end(); hole_line++) {
//             centroid = hole_line->CalculateCentroid(map_points);
             meanx = 0;
             meany = 0;
             for (node2=hole_line->begin(); node2!=hole_line->end(); node2++) {      
                  meanx = meanx + (map_points.PointIterator(*node2))->X();
                  meany = meany + (map_points.PointIterator(*node2))->Y();
             }
             meanx = meanx/hole_line->size();
             meany = meany/hole_line->size();
             
      //       hole_point = ObjectPoint(centroid.X(), centroid.Y(), 0.0, next_pnr, 0,0,0,0,0,0);
             hole_point = ObjectPoint(meanx, meany, 0.0, next_pnr, 0,0,0,0,0,0);
             found = true;
             point = LaserPoint(hole_point.X(), hole_point.Y(), 0);
            if (debug) printf("X,Y, = %10.2f, %10.2f\n", point.X(), point.Y());
             if (!point.InsidePolygonJordan(map_points,
                                         hole_line->LineTopologyReference())){
                 found = false;
               if (debug)  printf("NOT FOUND!!!!!!!!!!!!!!!!!\n");
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
                  if (debug){
                      one_map_line.erase(one_map_line.begin(), one_map_line.end());
                      if(!hole_map_points.empty()) hole_map_points.erase(hole_map_points.begin(),hole_map_points.end());   
                      one_map_line.push_back(*map_line);
                      one_map_line.push_back(*hole_line);
                      for (node2=map_line->begin(); node2!=map_line->end(); node2++) {      
                          hole_map_points.push_back(*(map_points.PointIterator(*node2)));
                          }
                      for (node2=hole_line->begin(); node2!=hole_line->end(); node2++) {      
                          hole_map_points.push_back(*(map_points.PointIterator(*node2)));
                          }

                      one_map_line.Write("test_holenotfound.top", false);
                      hole_map_points.Write("test_holenotfound.objpts");                 
                      return;
                      }
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
     if (debug) holes_laser.Write("holes.laser", false);  
 //     if ((map_line->TOP10MajorClass() == TOP10_Road) || (map_line->TOP10MajorClass() == TOP10_Water) 
     if ((map_line->TOP10MajorClass() == TOP10_Water) 
      || map_line->TOP10MajorClass() == TOP10_Building ||
      (map_line->TOP10MajorClass() == TOP10_Road && top10version)){
         if (debug) printf("Label = %d, area %4.2f\n", map_line->Label(),map_line->CalculateArea(map_points));
         //add all map points
         highestbuildingpoint = -100;
         for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
           sel_map_point = *(map_points.PointIterator(*node2));
           sel_map_points.push_back(sel_map_point);
         }
         
         //Make LineTopologies with one polygon
         one_map_line.erase(one_map_line.begin(), one_map_line.end());
         one_map_line.push_back(*map_line);
         if (found) one_map_line.insert(one_map_line.end(), hole_lines.begin(), hole_lines.end());             
         if (debug) printf("map line pushed back\n");
           if (debug)  one_map_line.Write("test.top", false);
         if (debug)   sel_map_points.Write("test.objpts"); 
         sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
         one_map_line.ReNumber(sel_map_points, 0, 0);
         if (debug) printf("map line renumbered, size of sel_map_points %d\n", sel_map_points.size());
         if (hole_lines.size()>0 && found && hole_lines.size()==hole_points.size()){
           tin = sel_map_points.Triangulate(one_map_line, hole_points);
           }
         else tin = sel_map_points.Triangulate(one_map_line);
         
       if (debug)  one_map_line.Write("test.top", false);
      if (debug)   sel_map_points.Write("test.objpts");
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
         map_tin_lines.SetAttribute(LineLabelTag, map_line->Attribute(LineLabelTag));
         map_tin_lines.SetAttribute(IDNumberTag, map_line->Attribute(IDNumberTag));
         if (!all_tin_points.empty())
           map_tin_lines.ReNumber(sel_map_points, (all_tin_points.end()-1)->Number()+1, (all_tin_lines.end()-1)->Number()+1);
         all_tin_lines.insert(all_tin_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
         all_tin_points.insert(all_tin_points.end(), sel_map_points.begin(), sel_map_points.end());
      }
      else
         {
         if (debug) printf("Label = %d\n", map_line->Label());
         // Add map points first
         only_boundary_points.erase(only_boundary_points.begin(), only_boundary_points.end());
         for (node2=map_line->begin(); node2!=map_line->end()-1; node2++) {      
              sel_map_points.push_back(*(map_points.PointIterator(*node2)));
              only_boundary_points.push_back(*(map_points.PointIterator(*node2)));
         }
            keep_map_points = sel_map_points;  
            one_map_line.erase(one_map_line.begin(), one_map_line.end());
            one_map_line.push_back(*map_line);
            if (found) one_map_line.insert(one_map_line.end(), hole_lines.begin(), hole_lines.end());
            sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
          
            sel_laser_points.ErasePoints();
          if (map_line->HasAttribute(BuildingNumberTag)) sel_laser_points = laser_points.SelectTagValue(PolygonNumberTag, map_line->Attribute(BuildingNumberTag));
          else sel_laser_points = laser_points.SelectTagValue(PolygonNumberTag, map_line->Number());
            
 //           if (map_line->TOP10MajorClass() == TOP10_Meadow){
           grid_size = default_grid_size;
           if (map_line->TOP10MajorClass() == TOP10_Meadow || (map_line->TOP10MajorClass() == TOP10_Road && !top10version)){                                            
   //        if (map_line->TOP10MajorClass() == TOP10_Meadow){                                            
            if (rasterise_points){
//              if (map_line->TOP10MajorClass() == TOP10_Road) grid_size = 1;
    if (debug)          printf("map line: %d\n", map_line->Number());
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
                       
                       
                       }
                  }
              
              }
              sel_laser_points = poly_laser_points;
            }                                
                                            
            
            }
    if (debug)        printf("Number = %d, laser points %d\n", map_line->Number(), sel_laser_points.size());
    if (debug)        sel_laser_points.Write("sel_laser_points.laser", false);
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
  if (debug)      printf("size of seladdedpoints %d\n", sel_addedpoints.size());      fflush(stdout);
          for (i=0;i<sel_addedpoints.size();i++) {              
     //          sel_map_points.push_back(ObjectPoint(sel_addedpoints[i].vect(), next_pnr, Covariance3D()));
     //          next_pnr ++;
          }          
          sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
          one_map_line.ReNumber(sel_map_points, 0, 0);
          keep_map_points = sel_map_points;
          loc_laser_points.ErasePoints();
          for (map_line_bgt = one_map_line.begin(); map_line_bgt!=one_map_line.end(); map_line_bgt++){
            for (node2=map_line_bgt->begin(); node2!=map_line_bgt->end()-1; node2++) {      
               point = LaserPoint((sel_map_points.PointIterator(*node2))->X(),(sel_map_points.PointIterator(*node2))->Y(),(sel_map_points.PointIterator(*node2))->Z());
               point.SetAttribute(LabelTag, 1);

               loc_laser_points.push_back(point);
            }
          }
          for (i=0;i<sel_addedpoints.size();i++) {
               point = LaserPoint(sel_addedpoints[i].X(), sel_addedpoints[i].Y(), sel_addedpoints[i].Z());
               point.SetAttribute(LabelTag, 0);
               point.SetAttribute(PolygonNumberTag, map_line->Attribute(BuildingNumberTag));
      if (debug)         addedpoints.push_back(point);
               loc_laser_points.push_back(point);
          }                                          
  //       if (map_line->TOP10MajorClass() == TOP10_Meadow) simplifiedpoints = loc_laser_points.SimplifyMesh_KeepLabel(0.05, LabelTag, 1);
  //       if (map_line->TOP10MajorClass() == TOP10_Road) simplifiedpoints = loc_laser_points.SimplifyMesh_KeepLabel(0.02, LabelTag, 1);
           simplifiedpoints = loc_laser_points;
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
          
     if (debug)     printf("renumbered");fflush(stdout);
          sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
          one_map_line.ReNumber(sel_map_points, 0, 0);
          
      if (debug)    one_map_line.Write("selmapline.top", false);
      if (debug)    sel_map_points.Write("selmapline.objpts");
          if (hole_lines.size()>0 && found && hole_lines.size()==hole_points.size()){
              tin = sel_map_points.Triangulate(one_map_line, hole_points);
            if (debug)  printf("triangulated_a");fflush(stdout);
            if (debug)  one_map_line.Write("test.top", false);
           if (debug)   sel_map_points.Write("test.objpts");
       //       
            if (debug)  one_map_line.Write("test2.top", false);
            if (debug)  sel_map_points.Write("test2.objpts");            
           }
           else {
              tin = sel_map_points.Triangulate(one_map_line);
           if (debug)   printf("triangulated_b");
           }
              if (debug)printf("Map points triangulated\n");   
              
              map_tin_lines = LineTopologies(tin);
              tin.Erase();
              if (debug)printf("TIN erased...\n");
         if (debug) map_tin_lines.Write("maptinline.top", false);
     if (debug)     sel_map_points.Write("maptinline.objpts");
             
              map_tin_lines.SetAttribute(LineLabelTag, map_line->Attribute(LineLabelTag));
              map_tin_lines.SetAttribute(IDNumberTag, map_line->Attribute(IDNumberTag));
              if (!all_tin_points.empty()) map_tin_lines.ReNumber(sel_map_points, (all_tin_points.end()-1)->Number()+1, (all_tin_lines.end()-1)->Number()+1);
              all_tin_lines.insert(all_tin_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
              all_tin_points.insert(all_tin_points.end(), sel_map_points.begin(), sel_map_points.end());                                                                          
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
       printf("\n %d points, %d triangles\n", all_tin_points.size(), all_tin_lines.size());fflush(stdout);
  //     return;
      iii=nb_laser_points.size();
//      all_messedup_points.Write("messeduppoints.objpts");
//      all_messedup_lines.Write("messeduppoints.top", false);
    if (debug)  addedpoints.Write("allmodelpoints.laser", false);
      addedpoints.ErasePoints();
      laser_points.ErasePoints();
 //     suspicious_laser_points.Write("suspiciouslaserpoints.laser", false);
    if (!all_tin_points.Write(map_points_output)) printf("Error writing the 3D TIN points\n");fflush(stdout);
    if (!all_tin_lines.Write(map_topology_output, false)) printf("Error writing the topology of the 3D TIN lines\n");fflush(stdout);


    if (calculate_quality){
                           LaserPoints quality_laser_points;
                           int largeresidual=0;
                           
     if (!map_lines.Read(map_topology_input)) {
          printf("Error reading map lines from file %s\n", map_topology_input);
          exit(0);
     }
     if (!laser_points.Read(laser_input)) {
          printf("Error reading laser points from file %s\n", laser_input);
          exit(0);
     }
        printf("Calculating residuals on laser points\n");fflush(stdout);
        for (map_line=map_lines.begin(), count=0; map_line!=map_lines.end(); map_line++, count++) {
        holeornot = map_line->Attribute(HoleTag);
        if (holeornot != 0) continue;
        sel_laser_points.ErasePoints();
        sel_laser_points.AddTaggedPoints(laser_points, map_line->Attribute(IDNumberTag), PolygonNumberTag);
        sel_map_lines2 = all_tin_lines.SelectAttributedLines(IDNumberTag, map_line->Attribute(IDNumberTag));
         if (debug || count == (count/reductionfactor)*reductionfactor) printf("progress: line %5d, percentage %5.2f, (checking %6d points, on %6d triangles)\r", 
         count, 100.0*count/map_lines.size(), sel_laser_points.size(), sel_map_lines2.size());
        for (laser_point=sel_laser_points.begin(), index1=0;
         laser_point!=sel_laser_points.end(); laser_point++, index1++) {
         if (debug) printf("%4.2f\r", index1*100.0/sel_laser_points.size());
               for (map_line2=sel_map_lines2.begin(), found = false;
                    map_line2!=sel_map_lines2.end() && !found; map_line2++) {
                    if (laser_point->InsidePolygonJordan(all_tin_points,
                                         map_line2->LineTopologyReference())) {
                      nb_node=map_line2->begin();
                      p1 = Position3D(*(all_tin_points.PointIterator(*nb_node))); nb_node++;
                      p2 = Position3D(*(all_tin_points.PointIterator(*nb_node))); nb_node++;
                      p3 = Position3D(*(all_tin_points.PointIterator(*nb_node)));
                      pos = Position3D(laser_point->X(), laser_point->Y(), laser_point->Z());
                      plane = Plane(p1, p2, p3);
                      projpos = plane.Project(pos);
                      laser_point->Residual() = projpos.Distance(pos) ;//fabs(plane.Z_At(laser_point->X(), laser_point->Y(), &success)-laser_point->Z());
                      found = true;
                      quality_laser_points.push_back(*laser_point);
                      if (laser_point->Residual()>0.05) largeresidual++;
                      }
               }
         }
        }
        printf("\nlaserpoints: %d, largeresiduals %d\n",
                             quality_laser_points.size(), largeresidual);fflush(stdout);
                if (!quality_laser_points.Write("qualitylaserpoints.laser", false))
        printf("Error writing residual file between laser and 3d tin.\n"); fflush(stdout);
        }
 all_tin_points.erase(all_tin_points.begin(), all_tin_points.end());
 all_tin_lines.erase(all_tin_lines.begin(), all_tin_lines.end());   
 printf("\nProgram 3dbgt2tin finished.\n"); fflush(stdout);
return;    
}    



