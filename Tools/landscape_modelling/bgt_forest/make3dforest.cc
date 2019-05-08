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
 Author : George Vosselman
 Date   : 08-03-2006

 Modified by Sander Oude Elberink, 2006-2013
 removed core, kept part to add forestry on top of 3d terrain polygons...
 buildings (see bgt_buildings) and forest can be put on top of terrain (+water/roads/foundation buildings)
 
 code from top10 project at kadaster
 
 added hole info, lod 1 and lod2, 20-03-2013
 
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

void make3dforest(char *laser_input, char *map_points_input,
	       char *map_topology_input, char *map_points_output,
	       char *map_topology_output, bool lod1, bool lod2, bool debug,
           int reductionfactor)

{
  LaserPoints                  laser_points, sel_laser_points, meadow_laser_points,
                               quality_points, poly_laser_points,loc_laser_points,
                               holes_laser, sel_addedpoints;
  LaserPoints::iterator        laser_point, nearest_point;
  LaserPoint                   nb_laser_point, temp_point, point;
  TINEdges                     edges, seledges, poledges;
  ObjectPoints                 map_points, dtm_tin_points, clean_map_points,
                               floor_tin_forest_points, wallpoints, all_wall_points;
  LineTopologies               map_lines, dtm_tin_lines, wall_map_lines, inner_rings,
                               floor_tin_forest_lines, all_wall_lines, final_tin_lines,
                               notvalidlines;
  ObjectPoints::iterator       map_point, previous_point, next_point;
  ObjectPoint                  new_map_point, hole_point;
  LineTopologies::iterator     map_line, last_map_line, hole_line;
  LineTopsIterVector           sel_map_lines;
  LineTopsIterVector::iterator sel_map_line, sel_map_line2;
  PointNumberList              neighbourhood, road_nbh;
  PointNumberList::iterator    node, previous_node, next_node, noden;
  PointNumber                  *node_ptr;
  int                          nearest_laser_point, dominant_segment, count,
                               success, index1, index2, index3, close_index1,
                               close_index2, next_point_number, index, pol_num,
                               num_2d_points, index_road, index_non_road,
                               top10class, iter, run_index, notvalid;
  Plane                        plane;
  Planes                       planes;
  Line3D                       line;
  vector<double>               heights, height1, height2;
  vector<int>                  use_map_line, use_map_point, point_count;
  vector <int>                 building_numbers;
 vector <int>::iterator        building_number;
 
  double                       diff, min_diff, new_height, dist1, dist2,
                               min_height, meanx, meany;
  bool                         found, done, emptymap, pointinside, pointinhole;
  DataBounds3D                 bounds;
  TIN                          dtmtin;
  Position3D                   p1, p2, p3, pos;
//  Position2D                   ;
  LineTopology::iterator       nb_node, node2;
//  bool debug=true;
  bool rasterise_points;

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
  

  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (map_line->Attribute(LineLabelTag)!=5501){ //should be corresponding to forest polygons
      map_lines.erase(map_line);
      map_line--;
      count++;
    }
  }
  printf("Removed %d not forest polygons\n", count);
    
  if (map_lines.size()==0){
  printf("No forest polygons in dataset. Program finished!\n");
  return;
  }
if (!lod2){
 lod1 = true;
 printf("producing lod1 forest with %d polygons...\n", map_lines.size());
}

if (lod2){
 lod1 = true;
 printf("producing %d 'lod2' forest polygons...\n", map_lines.size());
}
printf ("Reduction factor to decrease amount of print information to screen: %d.\n", reductionfactor);

// Derive the TIN edges of the laser points
  printf("Deriving TIN of all laser points ..."); fflush(stdout);
  laser_points.DeriveTIN();
  printf("done\nDeriving TIN edges ..."); fflush(stdout);
  edges.Derive(laser_points.TINReference());
 printf("done\nDeriving 3d forest ...\n"); fflush(stdout);
 


  int                number_offset2, number_offset, size, next_pnr, holeornot;
  double median, fixed_floor_height, added_floorheight = 0;
  ObjectPoints       sel_map_points, all_tin_forest_points, hole_points, hole_map_points;
  ObjectPoints::iterator       this_point;
  ObjectPoint                  new_point;
  LineTopologies               one_map_line, map_tin_lines, all_tin_forest_lines,
                               hole_lines;
  TIN                          tin;
  number_offset2 = map_points.HighestPointNumber().Number()+1;
  next_pnr = map_points.HighestPointNumber().Number()+1;
  double grid_size_forest = 5;
  double nbh_radius            = 10.0;
  int min_x, max_x, min_y, max_y, min_z, max_z, numcols, numrows, x, y, i;
  bool add_groundfloor =true;
  rasterise_points = true;
  inner_rings = map_lines.SelectAttributedLines(HoleTag, 1);
  
  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++, count++) {
       if (debug || count == (count/reductionfactor)*reductionfactor) printf("polygon %d count %d, percentage %4.2f\r", map_line->Attribute(IDNumberTag), count, 100.0*count/map_lines.size());
       hole_lines.erase(hole_lines.begin(), hole_lines.end());
       hole_points.erase(hole_points.begin(), hole_points.end());
       holes_laser.ErasePoints();
       wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
       if (map_line->Attribute(IDNumberTag) != 0 ) hole_lines = inner_rings.SelectAttributedLines(IDNumberTag, map_line->Attribute(IDNumberTag));
       holeornot = map_line->Attribute(HoleTag);
       if (debug) printf("polygon %d (hole = %d) containing %d holes\n", map_line->Attribute(IDNumberTag), holeornot, hole_lines.size());
       sel_laser_points.ErasePoints();                                                                              
       sel_laser_points.AddTaggedPoints(laser_points, map_line->Attribute(BuildingNumberTag),
                                         PolygonNumberTag);
       if (debug) printf("sel laser points %d \n", sel_laser_points.size());
       if (sel_laser_points.size()<10) continue;
       sel_laser_points.DeriveDataBounds(0);
       sel_laser_points.DeriveTIN();
       median = sel_laser_points.ReturnHeightOfPercentilePoint(50);
       
       if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end());
       
       if (holeornot !=0) continue;

   //    if (hole_lines.size()>0){
         for (hole_line=hole_lines.begin(); hole_line!=hole_lines.end(); hole_line++) {
             meanx = 0;
             meany = 0;
             for (node2=hole_line->begin(); node2!=hole_line->end(); node2++) {      
                  meanx = meanx + (map_points.PointIterator(*node2))->X();
                  meany = meany + (map_points.PointIterator(*node2))->Y();
             }
             meanx = meanx/hole_line->size();
             meany = meany/hole_line->size();
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
                 else {
                      if (debug){
                      printf(" still not found, map line %d\n", map_line->Number());
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
    //     }
      if (debug) printf("polygon %d containing %d holes\n", map_line->Number(), hole_lines.size());
 
     
       
       
  
           for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
             this_point = map_points.PointIterator(*node2);        
             sel_map_points.push_back(*this_point);
          }
          for (hole_line=hole_lines.begin(); hole_line!=hole_lines.end(); hole_line++) {
            for (node2=hole_line->begin(); node2!=hole_line->end()-1; node2++) {
   //              sel_map_points.push_back(*(map_points.PointIterator(*node2)));
               }
            }           
   
          one_map_line.erase(one_map_line.begin(), one_map_line.end());
          one_map_line.push_back(*map_line);
          one_map_line.insert(one_map_line.end(), hole_lines.begin(), hole_lines.end());             
          sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
          one_map_line.ReNumber(sel_map_points, 0, 0);
          if (hole_lines.size()>0) tin = sel_map_points.Triangulate(one_map_line, hole_points); 
          else {                
          tin = sel_map_points.Triangulate(one_map_line);
          }
         
          map_tin_lines = LineTopologies(tin);
          tin.Erase(); 
          map_tin_lines.SetAttribute(LineLabelTag, map_line->Label());
          map_tin_lines.SetAttribute(IDNumberTag, map_line->Attribute(IDNumberTag));
          number_offset = sel_map_points.HighestPointNumber().Number()+1;
          wallpoints = sel_map_points;  
          wallpoints.DuplicateWithFixedZ(median, number_offset);
               wall_map_lines.AddTINWalls(one_map_line, number_offset);
          if (!all_wall_points.empty())
          wall_map_lines.ReNumber(wallpoints, all_wall_points.size()+1, all_wall_lines.size()+1);
          if (map_line->HasAttribute(IDNumberTag)) wall_map_lines.SetAttribute(IDNumberTag, map_line->Attribute(IDNumberTag));

          all_wall_lines.insert(all_wall_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
          all_wall_points.insert(all_wall_points.end(), wallpoints.begin(), wallpoints.end());
          if (!floor_tin_forest_points.empty()){
              map_tin_lines.ReNumber(sel_map_points, (floor_tin_forest_points.end()-1)->Number()+1, (floor_tin_forest_lines.end()-1)->Number()+1);
              }
          
          floor_tin_forest_lines.insert(floor_tin_forest_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
          floor_tin_forest_points.insert(floor_tin_forest_points.end(), sel_map_points.begin(), sel_map_points.end());                              

//          map_tin_lines.AddWalls(one_map_line, number_offset);
          
          //}
       if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end());
         for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
             this_point = map_points.PointIterator(*node2);        
             sel_map_points.push_back(*this_point);
          }
         for (hole_line=hole_lines.begin(); hole_line!=hole_lines.end(); hole_line++) {
            for (node2=hole_line->begin(); node2!=hole_line->end()-1; node2++) {
                 sel_map_points.push_back(*(map_points.PointIterator(*node2)));
               }
            }           
          one_map_line.erase(one_map_line.begin(), one_map_line.end());
          one_map_line.push_back(*map_line);
       one_map_line.insert(one_map_line.end(), hole_lines.begin(), hole_lines.end());             
          sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
       if (lod1){

          for(map_point=sel_map_points.begin(); map_point!=sel_map_points.end(); map_point++){
            map_point->Z() = median;
          }
          }
          one_map_line.ReNumber(sel_map_points, 0, 0);
          if (lod2){
              poledges.erase(poledges.begin(), poledges.end());
              poly_laser_points.ErasePoints();
              poledges.Derive(sel_laser_points.TINReference());                            
              max_x = int (sel_laser_points.DataBounds().Maximum().X());
              max_y = int (sel_laser_points.DataBounds().Maximum().Y());
              min_x = int (sel_laser_points.DataBounds().Minimum().X());
              min_y = int (sel_laser_points.DataBounds().Minimum().Y());
              numcols = int ((max_x-min_x)/grid_size_forest)+1;
              numrows = int ((max_y-min_y)/grid_size_forest)+1;
              for (x=min_x; x<max_x; x+=int(grid_size_forest)){
                  for (y=min_y; y<max_y; y+=int(grid_size_forest)){
                      pos = Position3D(x,y,0);
                      neighbourhood.erase(neighbourhood.begin(), neighbourhood.end());
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
                      temp_point.Attribute(PolygonNumberTag) = map_line->Attribute(BuildingNumberTag);
                      temp_point.Attribute(LabelTag)         = map_line->Label();
                      if (map_line->HasAttribute(IDNumberTag)) temp_point.Attribute(PlaneNumberTag) = map_line->Attribute(IDNumberTag);

                      pointinside = false;
                      if (temp_point.InsidePolygonJordan(map_points,
                                         map_line->LineTopologyReference())==1) {            
                         poly_laser_points.push_back(temp_point);
                         pointinside = true;
                       }
                     }
                   }
                   }
            sel_addedpoints.ErasePoints();
            sel_laser_points = poly_laser_points;
            if (debug) printf("size of sel_laser_points %d\n", sel_laser_points.size());
            for (i=0;i<sel_laser_points.size();i++) {              
              // remove the points inside the holes....
              pointinhole = false;
               for (hole_line=hole_lines.begin(); hole_line!=hole_lines.end() && !pointinhole; hole_line++) {
                   if (sel_laser_points[i].InsidePolygonJordan(map_points,
                                         hole_line->LineTopologyReference())==1) pointinhole = true;
               }     
              //Add sel_laser_points to sel_map_points
                if (!pointinhole) {
                   sel_addedpoints.push_back(sel_laser_points[i]);
                }
              }   
          sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
          one_map_line.ReNumber(sel_map_points, 0, 0);

          next_pnr = sel_map_points.HighestPointNumber().Number()+1;
          if (debug) printf("\nsize of selmappoints: %d plus %d ", sel_map_points.size(), sel_addedpoints.size());

        for (i=0;i<sel_addedpoints.size();i++) {              
               sel_map_points.push_back(ObjectPoint(sel_addedpoints[i].vect(), next_pnr, Covariance3D()));
               next_pnr ++;
          }          
          }
          if (debug) printf(" %d\n ", sel_map_points.size());
          if (hole_lines.size()>0) tin = sel_map_points.Triangulate(one_map_line, hole_points); 
          else {                
          tin = sel_map_points.Triangulate(one_map_line);
          }
          map_tin_lines = LineTopologies(tin);
          if (debug) printf("size of tin %d\n", map_tin_lines.size());
          tin.Erase();  
          map_tin_lines.SetAttribute(LineLabelTag, map_line->Label());
          map_tin_lines.SetAttribute(IDNumberTag, map_line->Attribute(IDNumberTag));
          if (!all_tin_forest_points.empty()){
              map_tin_lines.ReNumber(sel_map_points, (all_tin_forest_points.end()-1)->Number()+1, (all_tin_forest_lines.end()-1)->Number()+1);
              }
          all_tin_forest_lines.insert(all_tin_forest_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
          all_tin_forest_points.insert(all_tin_forest_points.end(), sel_map_points.begin(), sel_map_points.end());                              
          }
printf("\n removing double points, this can take a while...");  

  floor_tin_forest_lines.MakeClockWise(floor_tin_forest_points);
  floor_tin_forest_lines.ReNumber(floor_tin_forest_points, (all_tin_forest_points.end()-1)->Number()+1, (all_tin_forest_lines.end()-1)->Number()+1);
  floor_tin_forest_lines.SetAttribute(LineLabelTag, 4);
     all_tin_forest_lines.insert(all_tin_forest_lines.end(), floor_tin_forest_lines.begin(), floor_tin_forest_lines.end());
     all_tin_forest_points.insert(all_tin_forest_points.end(), floor_tin_forest_points.begin(), floor_tin_forest_points.end());
    if (debug)      floor_tin_forest_lines.Write("groundfloor_forest.top", false);
   if (debug)       floor_tin_forest_points.Write("groundfloor_forest.objpts");
  all_wall_lines.SetAttribute(LineLabelTag, 5);
  all_wall_lines.ReNumber(all_wall_points, (all_tin_forest_points.end()-1)->Number()+1, (all_tin_forest_lines.end()-1)->Number()+1);
  all_tin_forest_lines.insert(all_tin_forest_lines.end(), all_wall_lines.begin(), all_wall_lines.end());
  all_tin_forest_points.insert(all_tin_forest_points.end(), all_wall_points.begin(), all_wall_points.end());
  
  all_tin_forest_points.RemoveDoublePoints(all_tin_forest_lines, 0.01);
   building_numbers = all_tin_forest_lines.AttributedValues(IDNumberTag);
     for (building_number = building_numbers.begin(); building_number !=building_numbers.end(); building_number++){
         if (*building_number >= 0){
         map_tin_lines = all_tin_forest_lines.SelectAttributedLines(IDNumberTag, *building_number);
         final_tin_lines.insert(final_tin_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
         } 
     }
     printf("\nChecking for not valid polygons...\n");
all_tin_forest_lines.erase(all_tin_forest_lines.begin(), all_tin_forest_lines.end());
    for (map_line = final_tin_lines.begin(), run_index=0, notvalid=0; map_line!=final_tin_lines.end(); map_line++, run_index++){
   if (debug)           printf("%4.1f %4d\r", (100.0*run_index)/final_tin_lines.size(), notvalid);
         if (!map_line->IsValid()) {
            notvalidlines.push_back(*map_line);
            final_tin_lines.erase(map_line);
            map_line--;
            notvalid++;
         }
    }
     if (!all_tin_forest_points.Write(map_points_output))
         printf("Error writing the 3D TIN points\n");
     if (!final_tin_lines.Write(map_topology_output, false))
         printf("Error writing the topology of the 3D TIN lines\n");
    if (debug)      if (!notvalidlines.Write("notvalidlines.top", false))
         printf("Error writing the topology of the 3D TIN lines\n");

    return;           
}
