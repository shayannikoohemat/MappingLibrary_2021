
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
 Date   : 08-03-2006

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

void Make3DSurface(char *laser_input, char *map_points_input,
	       char *map_topology_input, char *kernel_file, char *large_kernel_file,
           char *map_points_output, char *map_topology_output, 
           char *vrml_file, char *laser_output)

{
  LaserPoints                  laser_points, sel_laser_points, nb_laser_points, dtm_laser_points,
                               copy_nb_laser_points, save_points, holes_laser, final_points,
                               building_laser_points, dtmtest, dtmtest2, temp_height_points;
  LaserPoint                   nb_laser_point, mean, temp_point;
  LaserPoints::iterator        laser_point, node;
  ObjectPoints                 map_points, sel_map_points, all_tin_points,
                               meadow_points, road_points, holes, keep_map_points,
                               new_road_points, dtm_points, new_tin_points, new_map_points,
                               dtm_tin_points, filtered_points, edge_points, all_edge_points,
                               copy_sel_map_points, common_points, holes_points, water_points,
                               dddmap_points, dummy_points,
                               all_tin_building_points, building_map_points, obj;
  LineTopologies               map_lines, map_tin_lines, all_tin_lines,
                               one_map_line, all_road_lines, all_water_lines,
                               all_building_lines, all_meadow_lines,
                               hidden_tin_lines, keep_map_lines, new_polygons,
                               all_shadow_lines, dtm_tin_lines, edge_lines, all_edge_lines,
                               common_parts, building_map_lines, all_tin_building_lines, *top;
  LineTopology                 new_polygon, edge_line, common_part, common_line1, common_line2;
  ObjectPoints::iterator       map_point, this_point, previous_point, next_point, map_point2;
  ObjectPoint                  mean_point, new_road_point, new_map_point, sel_map_point, new_point;
  LineTopologies::iterator     map_line, last_map_line, map_line2;
  LineTopology::iterator       node2, node3, nb_node;
  LineTopsIterVector           sel_map_lines;
  PointNumberList::iterator    node_pnl, previous_node, next_node, next_node1;
  PointNumberList              neighbourhood;
  PointNumber                  *node_ptr, *ptn, pt1, pt2;
  int                          i, ii, iii, number_offset, countzmin, next_pnr, next_pnr2, 
                               count, nearest_laser_point, index2, index1,
                               pol_num,j,jj,n, edge_size, edgelines_size, number_offset2,
                               changed, success, iter;
  TIN                          tin, dtmtin, *mytin, *tin2;
  //TIN::const_iterator          mesh;
  TINMesh                      *mesh;
  MeshNumber                   *m;
  FILE                         *fd, *fdimage, *dxffile;
  Image                        kernel, large_kernel;
  TINEdges                     edges, new_edges, edges2, new_edges2, *edges_3d, edges3, edges22;
  float                        dist, meanx, meany, meanz, max_diff, diff, dist1, dist2, height,
                               remember_height;
  vector<int>                  point_count;
  bool                         found, done, changedb;
  DataBounds3D                 bounds;
  Plane                        plane;
  
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
  
  bool debug=false;
  bool buildings=false;
  bool dtm_surface=false;

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
  /*if (!keep_map_points.Read(map_points_input)) {
    printf("Error reading map points from file %s\n", map_points_input);
    exit(0);
  }
  if (!keep_map_lines.Read(map_topology_input)) {
    printf("Error reading map lines from file %s\n", map_topology_input);
    exit(0);
  }*/
  // Check if the laser points have the required attributes
  if (!laser_points.begin()->HasAttribute(SegmentNumberTag)) {
    printf("Error: Make3DSurface requires laser points with SegmentNumberTags\n");
    return;
  }

  if (!laser_points.begin()->HasAttribute(PolygonNumberTag)) {
    printf("Error: Make3DSurface requires laser points with PolygonNumberTags\n");
    return;
  }
  if (!laser_points.begin()->HasAttribute(LabelTag)) {
    printf("Error: Make3DSurface requires laser points with LabelTags\n");
    return;
  }
//final_points = laser_points;


//return;    
  map_points.RemoveDoublePoints(map_lines, 0.2);
  for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
            // Find objects with all points at -100 m                                    
                if(!dummy_points.empty()) dummy_points.erase(dummy_points.begin(), dummy_points.end());
                for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
                     dummy_points.push_back(*(map_points.PointIterator(*node2)));
                }
                countzmin=0;
                for (map_point=dummy_points.begin(); map_point!=dummy_points.end(); map_point++) {
                     if (map_point->Z() == -100.0) {
                         countzmin++;
                     }
                }    
                if (dummy_points.size()==countzmin){
                    map_lines.erase(map_line);
                    map_line--; 
                }
                else {
                    dddmap_points.insert(dddmap_points.end(), dummy_points.begin(), dummy_points.end());                               
                    if (map_line->TOP10MajorClass() == TOP10_Road) all_road_lines.push_back(*map_line);
                }
      }

  map_points.erase(map_points.begin(), map_points.end());
  map_points = dddmap_points;
//  map_points.RemoveDoublePoints(map_lines, 0.2);
//  map_points.AverageHeightsOfDoublePoints(map_lines, 0.3, 1.0);
  keep_map_points = map_points;
  keep_map_lines = map_lines;
  //all_road_lines.erase(all_road_lines.begin(), all_road_lines.end());
  //map_points.Write(map_points_output);
  //all_road_lines.Write(map_topology_output);
  //return;
  
  if (debug) printf("start\n");
  next_pnr = map_points.HighestPointNumber().Number()+1;

  for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
    if (map_line->TOP10MajorClass() == TOP10_Meadow) {
      node_pnl=map_line->begin();
      map_point = map_points.PointIterator(*node_pnl);
      previous_node=map_line->PreviousNode(node_pnl);
      previous_point = map_points.PointIterator(*previous_node);
      dist1 = (map_point->vect2D() - previous_point->vect2D()).Length();
      next_node=map_line->NextNode(node_pnl);
      for (; node_pnl!=map_line->end()-1; next_node=map_line->NextNode(node_pnl)) {
        next_point = map_points.PointIterator(*next_node);
        dist2 = (map_point->vect2D() - next_point->vect2D()).Length();
        if ((map_point->Z() - previous_point->Z()) / dist1 > max_vegetation_slope &&
            (map_point->Z() - next_point->Z()) / dist2 > max_vegetation_slope) {
          new_map_point = *map_point;
          new_map_point.Z() = (dist1 * next_point->Z() +
                               dist2 * previous_point->Z()) / (dist1 + dist2);
          new_map_point.Number() = next_pnr;
          next_pnr ++;
          new_map_points.push_back(new_map_point);
          node_pnl->Number() = new_map_point.Number();
          if (node_pnl == map_line->begin()) // Double update for close polygon
            (map_line->end()-1)->Number() = new_map_point.Number();
          printf("Correcting height of meadow point %d from %5.2f to %5.2f\n",
                            map_point->Number(), map_point->Z(), new_map_point.Z());
          map_point = map_points.end() - 1;
        }
        previous_node = node_pnl;      previous_point = map_point;
        node_pnl          = next_node; map_point      = next_point;
        dist1         = dist2;
      }
    }
  }
  
 // map_points.RemoveDoublePoints(common_parts, 1.0);
//  map_points.Write(map_points_output);
//  map_lines.Write(map_topology_output);
 // printf("Number of common parts = %d\n", common_parts.size());   
//  return;
/*
for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++){
    if (map_line->TOP10MajorClass() == TOP10_Road) {
       common_parts.erase(common_parts.begin(), common_parts.end());
       for (map_line2=map_line +1; map_line2!=map_lines.end();map_line2++){
              if (map_line->Adjacent(*map_line2, 10)){                                         
                  common_parts.push_back(*map_line);
                  common_parts.push_back(*map_line2);
            }
       }
     map_points.AverageHeightsOfDoublePoints(common_parts, 0.3, 2.51);  
     }
   }
  
  //Remove peaks in road objects
for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
    if (map_line->TOP10MajorClass() == TOP10_Road) {
      node_pnl=map_line->begin();
      map_point = map_points.PointIterator(*node_pnl);
      previous_node=map_line->PreviousNode(node_pnl);
      previous_point = map_points.PointIterator(*previous_node);
      dist1 = (map_point->vect2D() - previous_point->vect2D()).Length();
      next_node=map_line->NextNode(node_pnl);
      for (; node_pnl!=map_line->end()-1; next_node=map_line->NextNode(node_pnl)) {
        next_point = map_points.PointIterator(*next_node);
        dist2 = (map_point->vect2D() - next_point->vect2D()).Length();
        if ((fabs(map_point->Z() - previous_point->Z())) / dist1 > max_road_slope &&
            (fabs(map_point->Z() - next_point->Z())) / dist2 > max_road_slope) {
          new_map_point = *map_point;
          new_map_point.Z() = (dist1 * next_point->Z() +
                               dist2 * previous_point->Z()) / (dist1 + dist2);
          new_map_point.Number() = next_pnr;
          next_pnr ++;
          new_map_points.push_back(new_map_point);
          node_pnl->Number() = new_map_point.Number();
          if (node_pnl == map_line->begin()) // Double update for close polygon
            (map_line->end()-1)->Number() = new_map_point.Number();
          printf("Correcting height of road point %d from %5.2f to %5.2f\n",
                            map_point->Number(), map_point->Z(), new_map_point.Z());
          map_point = map_points.end() - 1;
          
        }        
        previous_node = node_pnl;      previous_point = map_point;
        node_pnl          = next_node; map_point      = next_point;
        dist1         = dist2;
      }
    }
  } 
  */   
 map_points.insert(map_points.end(), new_map_points.begin(), new_map_points.end());  
 next_pnr = map_points.HighestPointNumber().Number()+1; 
 new_map_points.erase(new_map_points.begin(), new_map_points.end());
/*
 for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++){
    if (map_line->TOP10MajorClass() == TOP10_Road) {
       common_parts.erase(common_parts.begin(), common_parts.end());
       for (map_line2=map_line +1; map_line2!=map_lines.end();map_line2++){
              if (map_line->Adjacent(*map_line2, 10)){                                         
                  common_parts.push_back(*map_line);
                  common_parts.push_back(*map_line2);
            }
       }
     map_points.AverageHeightsOfDoublePoints(common_parts, 0.3, 2.51);  
     }
   }
  
 // Remove remaining steep slopes in road objects
 
 bool check, double_check;
 iter = 0; int *numpts;
 do {
 double_check = false;     
 iter++;
 map_points.insert(map_points.end(), new_map_points.begin(), new_map_points.end());  
 new_map_points.erase(new_map_points.begin(), new_map_points.end());
 for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
    if (map_line->TOP10MajorClass() == TOP10_Road) {
      if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end());
      check = false;                              
      node_pnl=map_line->begin();
      map_point = map_points.PointIterator(*node_pnl);
      previous_node=map_line->PreviousNode(node_pnl);
      previous_point = map_points.PointIterator(*previous_node);
      dist1 = (map_point->vect2D() - previous_point->vect2D()).Length();
      next_node=map_line->NextNode(node_pnl);
      for (; node_pnl!=map_line->end()-1; next_node=map_line->NextNode(node_pnl)) {
        next_point = map_points.PointIterator(*next_node);
        dist2 = (map_point->vect2D() - next_point->vect2D()).Length();
        if (fabs(map_point->Z() - previous_point->Z()) / dist1 > max_road_slope2 ||
            (fabs(map_point->Z() - next_point->Z())) / dist2 > max_road_slope2) {
           check = true; 
        }        
        previous_node = node_pnl;      previous_point = map_point;
        node_pnl          = next_node; map_point      = next_point;
        dist1         = dist2;
      }
      if (check){      
      i=0;
      meanz = 0;
      printf("Mapline %d:\n",map_line->Number());
      for (node2=map_line->begin(); node2!=map_line->end()-1; node2++) {
           i++;
           sel_map_point = *(map_points.PointIterator(*node2));
           height = sel_map_point.Z();
           meanz = (meanz*(i-1) + height)/i;
           sel_map_points.push_back(sel_map_point);
         }           
         printf("Mean = %5.2f\n", meanz);
         node_pnl=map_line->begin();
         map_point = map_points.PointIterator(*node_pnl);
         previous_node=map_line->PreviousNode(node_pnl);
         previous_point = map_points.PointIterator(*previous_node);
         dist1 = (map_point->vect2D() - previous_point->vect2D()).Length();
         next_node=map_line->NextNode(node_pnl);
      for (; node_pnl!=map_line->end()-1; next_node=map_line->NextNode(node_pnl)) {
        next_point = map_points.PointIterator(*next_node);
        dist2 = (map_point->vect2D() - next_point->vect2D()).Length();
        if (fabs((map_point->Z() - next_point->Z()) / dist2) > max_road_slope2) {
            if (fabs(next_point->Z()-meanz) < fabs(map_point->Z()-meanz)){
              new_map_point = *map_point;
              new_map_point.Z() = next_point->Z();//plane.Z_At(map_point->X(), map_point->Y(), &success);//next_point->Z();
              new_map_point.Number() = next_pnr;
              next_pnr ++;
              new_map_points.push_back(new_map_point);
              node_pnl->Number() = new_map_point.Number();
              double_check = true;
             if (node_pnl == map_line->begin()) // Double update for close polygon
                 (map_line->end()-1)->Number() = new_map_point.Number();
             printf("Correcting height of road point %d from %5.2f to %5.2f\n",
                            map_point->Number(), map_point->Z(), new_map_point.Z());                               
             }                                                    
        }        
        if (fabs((map_point->Z() - previous_point->Z()) / dist1) > max_road_slope2) {
 
            if (fabs(next_point->Z()-meanz) < fabs(map_point->Z()-meanz)){
              new_map_point = *map_point;
              new_map_point.Z() = previous_point->Z();//plane.Z_At(map_point->X(), map_point->Y(), &success);//next_point->Z();                     
              new_map_point.Number() = next_pnr;
              next_pnr ++;
              new_map_points.push_back(new_map_point);
              node_pnl->Number() = new_map_point.Number();
              double_check = true;

             if (node_pnl == map_line->begin()) // Double update for close polygon
                 (map_line->end()-1)->Number() = new_map_point.Number();
             printf("Correcting height of road point %d from %5.2f to %5.2f\n",
                            map_point->Number(), map_point->Z(), new_map_point.Z());                               
             }                               
                                 
        }           
        previous_node = node_pnl;      previous_point = map_point;
        node_pnl          = next_node; map_point      = next_point;
        dist1         = dist2;
      }
      } 
    }
  }
  }while (double_check && iter < 50);   
  
  map_points.insert(map_points.end(), new_map_points.begin(), new_map_points.end());  
 next_pnr = map_points.HighestPointNumber().Number()+1; 
 new_map_points.erase(new_map_points.begin(), new_map_points.end());
 
 for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++){
    if (map_line->TOP10MajorClass() == TOP10_Road) {
       common_parts.erase(common_parts.begin(), common_parts.end());
       for (map_line2=map_line +1; map_line2!=map_lines.end();map_line2++){
              if (map_line->Adjacent(*map_line2, 10)){                                         
                  common_parts.push_back(*map_line);
                  common_parts.push_back(*map_line2);
            }
       }
     map_points.AverageHeightsOfDoublePoints(common_parts, 0.3, 2.51);  
     }
   }
  
  // return;
  map_points.RemoveDoublePoints(map_lines, 0.2);
 // map_points.AverageHeightsOfDoublePoints(map_lines, 0.3, 1.0);
  for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
    if (map_line->TOP10MajorClass() == TOP10_Road) {
      node_pnl=map_line->begin();
      map_point = map_points.PointIterator(*node_pnl);
      previous_node=map_line->PreviousNode(node_pnl);
      previous_point = map_points.PointIterator(*previous_node);
      dist1 = (map_point->vect2D() - previous_point->vect2D()).Length();
      next_node=map_line->NextNode(node_pnl);
      for (; node_pnl!=map_line->end()-1; next_node=map_line->NextNode(node_pnl)) {
        next_point = map_points.PointIterator(*next_node);
        dist2 = (map_point->vect2D() - next_point->vect2D()).Length();
        if ((fabs(map_point->Z() - previous_point->Z())) / dist1 > max_road_slope &&
            (fabs(map_point->Z() - next_point->Z())) / dist2 > max_road_slope) {
          new_map_point = *map_point;
          new_map_point.Z() = (dist1 * next_point->Z() +
                               dist2 * previous_point->Z()) / (dist1 + dist2);
          new_map_point.Number() = next_pnr;
          next_pnr ++;
          new_map_points.push_back(new_map_point);
          node_pnl->Number() = new_map_point.Number();
          if (node_pnl == map_line->begin()) // Double update for close polygon
            (map_line->end()-1)->Number() = new_map_point.Number();
          printf("Correcting height of road point %d from %5.2f to %5.2f\n",
                            map_point->Number(), map_point->Z(), new_map_point.Z());
          map_point = map_points.end() - 1;
          
        }        
        previous_node = node_pnl;      previous_point = map_point;
        node_pnl          = next_node; map_point      = next_point;
        dist1         = dist2;
      }
    }
  } 
  
     
 map_points.insert(map_points.end(), new_map_points.begin(), new_map_points.end());  
 new_map_points.erase(new_map_points.begin(), new_map_points.end());
 next_pnr = map_points.HighestPointNumber().Number()+1; 
  for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++){
    if (map_line->TOP10MajorClass() == TOP10_Road) {
       common_parts.erase(common_parts.begin(), common_parts.end());
       for (map_line2=map_line +1; map_line2!=map_lines.end();map_line2++){
              if (map_line->Adjacent(*map_line2, 10)){                                         
                  common_parts.push_back(*map_line);
                  common_parts.push_back(*map_line2);
            }
       }
     map_points.AverageHeightsOfDoublePoints(common_parts, 0.3, 2.51);  
     }
   }
   */
  kernel.Read(kernel_file);
  large_kernel.Read(large_kernel_file);
  ii=1; 
  edge_size=0; edgelines_size=0; pt1=0; pt2=0; n=0, next_pnr2=0;
/*  if (!map_points.Write(map_points_output))
    printf("Error writing the 3D TIN points\n");
//    if (!all_road_lines.Write(map_topology_output))
    if (!map_lines.Write(map_topology_output))
    printf("Error writing the topology of the 3D TIN lines\n");
 //  return;
  */
  all_road_lines.erase(all_road_lines.begin(), all_road_lines.end());
  map_points.RemoveDoublePoints(map_lines, 0.2);
  iii=0;
  for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
      //If label indicate road or water object, do not add laser points, otherwise add laser points
      if (debug) printf("polygon %d\n", ii);
      if ((map_line->TOP10MajorClass() == TOP10_Road) || (map_line->TOP10MajorClass() == TOP10_Water)){
         if (debug) printf("Label = %d\n", map_line->Label());
         //empty temporary object & topology file
         if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end());
         if(!map_tin_lines.empty()) map_tin_lines.erase(map_tin_lines.begin(),map_tin_lines.end());
         //add all map points
         for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
           sel_map_point = *(map_points.PointIterator(*node2));
           sel_map_points.push_back(sel_map_point);
         }
         //Make LineTopologies with one polygon
         one_map_line.erase(one_map_line.begin(), one_map_line.end());
         one_map_line.push_back(*map_line);
         if (debug) printf("map line pushed back\n"); 
         sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
         one_map_line.ReNumber(sel_map_points, 0, 0);
         if (debug) printf("map line renumbered\n");
         tin = sel_map_points.Triangulate(one_map_line);
         if (debug) printf("points triangulated\n");
         map_tin_lines = LineTopologies(tin);
         if (debug) printf("TIN line-topologised\n");
         tin.Erase();
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
      //    }
         number_offset = sel_map_points.HighestPointNumber().Number()+1;
         sel_map_points.DuplicateWithFixedOffset(-1.0, number_offset);
         if (debug) printf("Points duplicated with fixed offset\n");
         map_tin_lines.AddWalls(one_map_line, number_offset);
         if (debug) printf("walls added\n");
         if (!all_tin_points.empty())
           map_tin_lines.ReNumber(sel_map_points, (all_tin_points.end()-1)->Number()+1, (all_tin_lines.end()-1)->Number()+1);
         all_tin_lines.insert(all_tin_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
         all_tin_points.insert(all_tin_points.end(), sel_map_points.begin(), sel_map_points.end());
      }
      else
         {
         if (debug) printf("Label = %d\n", map_line->Label());
         // Add map points first
         if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end());
         if(!map_tin_lines.empty()) map_tin_lines.erase(map_tin_lines.begin(),map_tin_lines.end());
         
         for (node2=map_line->begin(); node2!=map_line->end(); node2++) {      
              sel_map_points.push_back(*(map_points.PointIterator(*node2)));
         }
            sel_laser_points.ErasePoints();
            sel_laser_points.AddTaggedPoints(laser_points, map_line->Number(), PolygonNumberTag);
            if (map_line->TOP10MajorClass() == TOP10_Meadow){
               sel_laser_points.DeriveTIN();
               edges.Derive(sel_laser_points.TINReference());
               sel_laser_points.FilterOnMorphology(kernel, 20, 0.2, 0, edges);
               sel_laser_points.RemoveFilteredPoints();
               dtm_laser_points.insert(dtm_laser_points.end(), sel_laser_points.begin(), sel_laser_points.end());
               dtm_points.insert(dtm_points.end(), sel_map_points.begin(), sel_map_points.end());
               meadow_points.insert(meadow_points.end(), sel_map_points.begin(), sel_map_points.end());                                  
            }
//            printf("Number = %d\n", map_line->Number());
          for (i=0;i<sel_laser_points.size();i++) {
              //Add sel_laser_points to sel_map_points
                sel_map_points.push_back(ObjectPoint(sel_laser_points[i].vect(), next_pnr, Covariance3D()));
                next_pnr ++;
              }
              one_map_line.erase(one_map_line.begin(), one_map_line.end());
              one_map_line.push_back(*map_line);
              sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
              one_map_line.ReNumber(sel_map_points, 0, 0);
//              copy_sel_map_points = sel_map_points;
              tin = sel_map_points.Triangulate(one_map_line);
              
              /* begin of edge finding test 
//              *mytin= tin;
              tin2 = copy_sel_map_points.Triangulate(one_map_line);
              printf("TIN size: %d\n", tin2.size());
//              copy_sel_map_points = sel_map_points;
              if (!edge_points.empty()) edge_points.erase(edge_points.begin(), edge_points.end());
              if (!edge_lines.empty()) edge_lines.erase(edge_lines.begin(), edge_lines.end());
              for (i=0; i<tin2.size();i++) {
                m=tin2[i].Neighbours();
                ptn=tin2[i].Nodes();
                for (j=0;j<3;j++){
                  if (m->Number()==-1){
                    for (jj=0;jj<3;jj++){                 
                    printf("%d (j=%d, jj=%d):", (*ptn).Number(), j, jj);//(*bigtin).push_back((*mytin)[i]);
//                    if ((j==0) && (jj==1 || jj==2)) save_points.push_back(nb_laser_points[(*ptn).Number()]);
                      if ((j==0) && (jj==1 || jj==2)) {
                                 this_point = copy_sel_map_points.PointIterator((*ptn));
                                 this_point->Number()=next_pnr2;
                                 next_pnr2++;
                                 printf("This point number: %d\n", this_point->Number());
                                 edge_points.push_back(*this_point);
//                                 edge_points.push_back(*(sel_map_points.PointIterator((*ptn).Number())));
                                 }
//                    if ((j==1) && (jj==2 || jj==0)) save_points.push_back(nb_laser_points[(*ptn).Number()]);
                      if ((j==1) && (jj==2 || jj==0)) {
                                 this_point = copy_sel_map_points.PointIterator((*ptn));
                                 this_point->Number()=next_pnr2;
                                 next_pnr2++;
                                 printf("This point number: %d\n", this_point->Number());
                                 edge_points.push_back(*this_point);                                 
//                                 edge_points.push_back(*(sel_map_points.PointIterator((*ptn).Number())));
                                 }
//                    if ((j==2) && (jj==0 || jj==1)) save_points.push_back(nb_laser_points[(*ptn).Number()]);
                      if ((j==2) && (jj==0 || jj==1)) {
                                 this_point = copy_sel_map_points.PointIterator((*ptn));
                                 this_point->Number()=next_pnr2;
                                 next_pnr2++;
                                 printf("This point number: %d\n", this_point->Number());
                                 edge_points.push_back(*this_point);
                                 //edge_points.push_back(*(sel_map_points.PointIterator((*ptn).Number())));
                                 }
                      ptn++;
                    }
                    pt2=pt1;  pt2++;
                    edge_line = LineTopology(n,0, pt1, pt2);
                    edge_lines.push_back(edge_line);
                    pt1=pt2, pt1++, n++;
                  printf("\n");
                  }
                m++;
               }           
             printf("\n");
              }
//      save_points.VerifyTIN();
//          edge_points = save_points.ConstructObjectPoints();
//            edge_points.RemoveDoublePoints(edge_lines, 0.01);
//            all_edge_lines.insert(all_edge_lines.end(), edge_lines.begin(), edge_lines.end());
//            all_edge_points.insert(all_edge_points.end(), edge_points.begin(), edge_points.end());
//            printf("Edge points and lines inserted...\n"); 
           
 //             /* end of edge finding test */              
              if (debug)printf("Map points triangulated\n");   
              
              map_tin_lines = LineTopologies(tin);
              tin.Erase();
    //          tin2.Erase();
              if (debug)printf("TIN erased...\n");
              if (map_line->TOP10MajorClass() == TOP10_Building){ //buildings, add walls to z=-2
                       number_offset = sel_map_points.HighestPointNumber().Number()+1;
                       sel_map_points.DuplicateWithFixedZ(-2.0, number_offset);
                       map_tin_lines.AddWalls(one_map_line, number_offset);
              }
       //       if (map_line->TOP10MajorClass() == TOP10_Meadow){ //meadows, add walls to z=-4
       //                number_offset = sel_map_points.HighestPointNumber().Number()+1;
       //                sel_map_points.DuplicateWithFixedZ(-4.0, number_offset);
       //                map_tin_lines.AddWalls(one_map_line, number_offset);
       //       }
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
      
//      i=0;
       printf("%d, %d\n", iii, nb_laser_points.size());
  //     return;
      iii=nb_laser_points.size();
//      printf("i = %d\n", iii);
      printf("Filling holes beneath 3D roads... (this can take a while)\n");
      
      for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
      if (map_line->TOP10MajorClass() == TOP10_Road){ //create lower road parts
         new_polygon = *map_line;
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
//      new_edges.Derive(nb_laser_points.TINReference());
 //     new_polygons.MergePolygons();
//      printf("Done\n Filtering...");
//      nb_laser_points.FilterOnMorphology(large_kernel, 40, 0, 0, new_edges);
/*      printf("Done\n Removing filtered points...");
      copy_nb_laser_points = nb_laser_points;
      nb_laser_points.RemoveFilteredPoints();
      copy_nb_laser_points.RemoveGroundPoints();
      filtered_points = copy_nb_laser_points.ConstructObjectPoints();
*/      

    //  mytin = nb_laser_points.DeriveTIN();

//      printf("Done\nMy Tin derived...");

      /* testing taking height from constructed TIN*/
//      for (map_point=new_road_points.begin(); map_point!=new_road_points.end(); map_point++) {
//       for (mesh=nb_laser_points.TINReference().begin(); mesh!=nb_laser_points.TINReference().end();mesh++) {
//           *mesh
//           if (nb_laser_points.InsideTriangle(&*mesh, map_point->X(), map_point->Y())) {
//               printf("size of mesh %d\n", mesh.size());
//               mesh = nb_laser_points.TINReference().end()-1;
//           }
//        }
  //     }
//      printf("Done\n Producing TIN of DTM...");  
//      nb_laser_points.DeriveTIN();
//      new_edges.Derive(nb_laser_points.TINReference());
  //      nb_laser_points.RemoveLongEdges(new_edges, 100);
  //      printf("Done\n Producing TIN of DTM2...");
/*        dtmtin = nb_laser_points.TINReference();
        printf("Done\n Producing Line Topologies of DTM...");
        dtm_tin_lines = LineTopologies(dtmtin);
        printf("Done\n");
        for (i=0;i<nb_laser_points.size();i++) {
              //Add sel_laser_points to sel_map_points
                new_tin_points.push_back(ObjectPoint(nb_laser_points[i].vect(), nb_laser_points[i].GetPointNumber(), Covariance3D()));
        }
      printf("Size of new_polygons = %d\n", new_polygons.size());
      
      
      
   */  
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
       
/*    for (map_line=all_road_lines.begin(); map_line!=all_road_lines.end(); map_line++) {
        count=0;
        meanx = 0;
        meany = 0;
        meanz = 0;
       for (nb_node=map_line->begin(); nb_node!=map_line->end()-1; nb_node++) { //double pass for starting point ruled out
//         next_node=map_line->NextNode(nb_node);
//         next_point = all_tin_points.PointIterator(*next_node);
         this_point = all_tin_points.PointIterator(*nb_node);
         road_points.push_back(*this_point);
         nb_laser_point.X()=this_point->X();//meanx;//this_point->X();
         nb_laser_point.Y()=this_point->Y();
         nb_laser_point.Z()=this_point->Z();
         for (map_line2=dtm_tin_lines.begin(), found = false; map_line2!=dtm_tin_lines.end() && !found; map_line2++) {
           if (nb_laser_point.InsidePolygon(all_tin_points, 
//                      map_line2->LineTopologyReference())) {
         for (mesh=dtmtin.begin(); mesh!=dtmtin.end(); mesh++);
             if (
         if (this_point->Z() > 
//         dist = (this_point->vect2D() - next_point->vect2D()).Length();
//         count++;
//         meanx = (meanx*(count-1) + this_point->X())/count;
//         meany = (meany*(count-1) + this_point->Y())/count;
//         meanz = (meanz*(count-1) + this_point->Z())/count;
//         if (dist > dist1) dist1 = dist;
      }
//      if (dist1 > 8) {
     //       printf("Map line %d ", map_line->Number());
//            dtm_tin_lines.erase(map_line);
//            map_line--;
     //       printf("erased.\n");
//         }
//         dist1 = 0;
         nb_laser_point.X()=meanx;//this_point->X();
         nb_laser_point.Y()=meany;//this_point->Y();
         nb_laser_point.Z()=meanz;
         holes_laser.push_back(nb_laser_point);
    //     printf("Point: %10.2f , %10.2f, %4.2f\n", meanx, meany, meanz);
         //*laser_point = nb_laser_point;
//         for (map_line2=new_polygons.begin(), found = false; map_line2!=new_polygons.end() && !found; map_line2++) {
    //         if (map_line2->TOP10MajorClass() == TOP10_Road || map_line2->TOP10MajorClass() == TOP10_Water) {
//                 if (nb_laser_point.InsidePolygon(new_road_points, 
//                      map_line2->LineTopologyReference())) {
//                      found = true;
    //                  dtm_tin_lines.erase(map_line);
    //                  map_line--;
//                  }
//             }
//             if(!found) holes_laser.push_back(nb_laser_point);
    //     }                                         
    }            
      holes_points = holes_laser.ConstructObjectPoints();
//      for (i=0;i<holes_points.size();i++) {
//          new_road_point = holes_points[i];
//          new_road_point.Number() = next_pnr;
//          dtm_tin_points.push_back(new_road_point);
//          next_pnr++;
//      }
      dtmtin = dtm_tin_points.Triangulate();//(all_tin_lines, holes_points);
      dtm_tin_lines = LineTopologies(dtmtin);
 * einde test 
//    copy_nb_laser_points.RemoveGroundPoints();
//    filtered_points = copy_nb_laser_points.ConstructObjectPoints();
//    dtm_laser_points.DeriveTIN();
//    new_edges2.Derive(dtm_laser_points.TINReference());
//      printf("Done\n Producing TIN of DTM...");  
    nb_laser_points.DeriveTIN();
    edges22.Derive(nb_laser_points.TINReference());
//    nb_laser_points.FilterOnMorphology(large_kernel, 30, 0.1, 0.1, edges22);
//    nb_laser_points.RemoveFilteredPoints();      
//    nb_laser_points.DeriveTIN();
//    edges22.Derive(nb_laser_points.TINReference());
    final_points.DeriveTIN();
    edges2.Derive(final_points.TINReference());  
    */
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

         if (!all_tin_points.empty())
           map_tin_lines.ReNumber(sel_map_points, (all_tin_points.end()-1)->Number()+1, (all_tin_lines.end()-1)->Number()+1);
         all_tin_lines.insert(all_tin_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
         all_tin_points.insert(all_tin_points.end(), sel_map_points.begin(), sel_map_points.end());
         all_shadow_lines.insert(all_shadow_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
  } 
/*    printf("Size of shadow lines: %d (before)\n", all_shadow_lines.size());
  for (map_line=all_water_lines.begin(); map_line!=all_water_lines.end(); map_line++) {
       for (map_line2 = all_shadow_lines.begin(); map_line2!=all_shadow_lines.end(); map_line2++){
            if (map_line->size() == map_line2->size()){
               if (map_line->Adjacent(*map_line2 , map_line->size()-1)) {                       
                  all_shadow_lines.erase(map_line2);
                  map_line2--;
               } 
            }
       }
  }
  printf("Size of shadow lines: %d (after)\n", all_shadow_lines.size());
    //all_tin_points.AverageHeightsOfDoublePoints(all_shadow_lines, 0.1, 0.5);                     
//*/      
//          }        
//        nb_laser_points.insert(nb_laser_points.end(), dtm_laser_points.begin(), dtm_laser_points.end());
//        } 
//        for (map_line = all_road_lines.begin(); map_line!=all_road_lines.end(); map_line++) {
//                for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
//                  this_point = all_tin_points.PointIterator(*node2);
//                  road_points.push_back(*this_point);
//                }
//        }   
//    for (i=0;i<all_tin_points.size();i++) {
//        nb_laser_point.X()=all_tin_points[i].X();
//        nb_laser_point.Y()=all_tin_points[i].Y();
//        nb_laser_point.Z()=all_tin_points[i].Z();
//        if (nb_laser_point.Z()>= -15) nb_laser_points.push_back(nb_laser_point);      
//    }
  
//    meadow_points.RemoveDoublePoints(all_meadow_lines, 0.1);
//    tin = meadow_points.Triangulate(all_meadow_lines, holes);
//    hidden_tin_lines = LineTopologies(tin);
//    tin.Erase();

 /*         for (map_line = map_lines.begin(); map_line!=map_lines.end(); map_line++) {
                if (map_line->Label()>=5000 && map_line->Label()<=5999){
                for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
                  this_point = all_tin_points.PointIterator(*node2);
                  if (this_point->Z()>=-15){
                    for (i=0;i<road_points.size();i++){
                    dist = (this_point->vect2D() - road_points[i].vect2D()).Length();
                      if (dist<=min_dist){
                        nb_laser_point.X()=this_point->X();
                        nb_laser_point.Y()=this_point->Y();
                        nb_laser_point.Z()=this_point->Z();
                        nb_laser_points.push_back(nb_laser_point);            
                      }
                    }
                  }  
                }
                }
        }    
*//*    nb_laser_points.DeriveTIN();
      edges.Derive(nb_laser_points.TINReference());
      nb_laser_points.FilterOnMorphology(kernel, 10, 0.2, 0, edges);
      nb_laser_points.RemoveFilteredPoints();      
  *///  dtm_laser_points.DeriveTIN();
//      edges2.Derive(dtm_laser_points.TINReference());
//      dtmtin = dtm_laser_points.TINReference();
//      printf("Done\n Producing Line Topologies of DTM...");
//      dtm_tin_lines = LineTopologies(dtmtin);
/*      next_pnr = 0;
      for (i=0;i<dtm_laser_points.size();i++) {
        dtm_tin_points.push_back(ObjectPoint(dtm_laser_points[i].vect(), next_pnr, Covariance3D()));
        next_pnr++;
      }
      dtmtin = dtm_tin_points.Triangulate();
      dtm_tin_lines = LineTopologies(dtmtin);
*///      LaserSmoothSegments.LaserSmoothSegments(*dtm_laser_points);
//      dtm_laser_points.FilterOnMorphology(kernel, 20, 0.2, 0, edges2);
//      dtm_laser_points.RemoveFilteredPoints();      
//
//      hidden_tin_lines = *edges.EdgeTopologies();
    // Write the resulting 3D topography
//    if (!all_tin_points.Write(map_points_output))
if (buildings){
  if (!building_map_points.Read("buildmap.objpts")) {
    printf("Error reading building map points from file buildtest.objpts\n");
  }
  if (!building_map_lines.Read("buildmap.top")) {
    printf("Error reading map lines from file buildtest.top\n");
     }
     building_laser_points.Read("buildmap.laser");
 //    building_map_points.RemoveDoublePoints(building_map_lines, 0.1);
     number_offset2 = building_map_points.HighestPointNumber().Number()+1;
     number_offset = building_map_points.HighestPointNumber().Number()+1;

     for (map_line = building_map_lines.begin(); map_line!=building_map_lines.end(); map_line++){

        if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end()); 
        if(!map_tin_lines.empty()) map_tin_lines.erase(map_tin_lines.begin(),map_tin_lines.end());
        sel_laser_points.ErasePoints();                                               
        sel_laser_points.AddTaggedPoints(building_laser_points, map_line->Number(),
                                         PolygonNumberTag);
        if (sel_laser_points.empty()) {
                                   height = -100;
                                   }
        else {
            mean = sel_laser_points.Mean();
 //         sel_laser_points.DeriveDataBounds(0);
//          height = sel_laser_points.DataBounds().Maximum().Z();
           height = mean.Z();
/*            if (sel_laser_points.size()<5){
              height = 50;
              }
              else {
                   height = 0;  //For testing, visualise the # of laser points as height.
                   }
//                   printf("laser heights: %5.2f\n", height);
  */        
          for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
             this_point = building_map_points.PointIterator(*node2);
             new_point = *this_point;
             new_point.Z() = height;
             new_point.Number() = number_offset2;
             number_offset2++;
             node2->Number()=new_point.Number();
             sel_map_points.push_back(new_point);
          }
          one_map_line.erase(one_map_line.begin(), one_map_line.end());
          one_map_line.push_back(*map_line);
          sel_map_points.RemoveDoublePoints(one_map_line, 0.1);
          if (sel_map_points.size()>3){ 
            one_map_line.ReNumber(sel_map_points, 0, 0);
            tin = sel_map_points.Triangulate(one_map_line);
            map_tin_lines = LineTopologies(tin);
            number_offset = sel_map_points.HighestPointNumber().Number()+1;
            sel_map_points.DuplicateWithFixedZ(-2.0, number_offset);
   //         printf("Points duplicated with fixed offset\n");
            map_tin_lines.AddWalls(one_map_line, number_offset);
   //      if (debug) printf("walls added\n");
//          number_offset2++;//=sel_map_points.size()+1;
//          sel_map_points.DuplicateWithFixedZ(-4.0, number_offset2);
//          map_tin_lines.AddWalls(one_map_line, number_offset2);
            tin.Erase();  
            if (!all_tin_building_points.empty())
              map_tin_lines.ReNumber(sel_map_points, (all_tin_building_points.end()-1)->Number()+1, (all_tin_building_lines.end()-1)->Number()+1);
            all_tin_building_lines.insert(all_tin_building_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
            all_tin_building_points.insert(all_tin_building_points.end(), sel_map_points.begin(), sel_map_points.end()); 
          }   
        }    
     }
     } //end if buildings
     
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
//    all_tin_points.VRML2_Write(fd, all_meadow_lines, 1, 0, 1.0, 0); //green
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
    all_tin_points.WriteDXFMesh(dxffile, all_road_lines, 9, false, true, false);
    all_tin_points.WriteDXFMesh(dxffile, all_meadow_lines, 3, false, false, false);
    all_tin_points.WriteDXFMesh(dxffile, all_water_lines, 5, false, false, false);
  all_tin_points.WriteDXFMesh(dxffile, all_shadow_lines, 4, false, false, false);  
       dtm_tin_points.WriteDXFMesh(dxffile, dtm_tin_lines, 6, false, false, true);  
    fclose(dxffile);
 //   all_road_lines.insert(all_road_lines.end(), all_shadow_lines.begin(), all_shadow_lines.end());
    if (!new_road_points.Write(map_points_output))
//    if (!all_tin_building_points.Write(map_points_output))
//    if (!dtm_tin_points.Write(map_points_output))
    printf("Error writing the 3D TIN points\n");
//    if (!all_road_lines.Write(map_topology_output))
//    if (!all_tin_building_lines.Write(map_topology_output))
    if (!new_polygons.Write(map_topology_output))
//    if (!dtm_tin_lines.Write(map_topology_output))
    printf("Error writing the topology of the 3D TIN lines\n");

    if (!final_points.Write(laser_output, false))
        printf("Error writing all 3D map points in laser file format.\n");
}    



