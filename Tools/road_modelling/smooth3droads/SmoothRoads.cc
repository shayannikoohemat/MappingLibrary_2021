
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
 Date   : 28-11-2006

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
#include <math.h>
#include "dxf.h"

/*
--------------------------------------------------------------------------------
                           The main function
--------------------------------------------------------------------------------
*/

void SmoothRoads(char *map_points_input, char *map_topology_input,
                  char *map_points_output, char *map_topology_output,
                  char *laser_output)

{
  
  ObjectPoints                 map_points, sel_map_points, 
                               meadow_points, road_points, keep_map_points,
                               new_road_points, common_points, all_tin_points,
                               dddmap_points, dummy_points, new_map_points;
  LineTopologies               map_lines, all_road_lines, keep_map_lines, 
                               common_parts, one_map_line, all_tin_lines;
  LineTopology                 common_part;
  ObjectPoints::iterator       map_point, this_point, previous_point, next_point, map_point2;
  ObjectPoint                  new_road_point, new_map_point, sel_map_point, new_point;
  LineTopologies::iterator     map_line, last_map_line, map_line2;
  LineTopology::iterator       node2, node3, nb_node;
  LineTopsIterVector           sel_map_lines;
  PointNumberList::iterator    node_pnl, previous_node, next_node, next_node1;
  PointNumber                  *node_ptr, *ptn, pt1, pt2;
  int                          i, ii, iii, number_offset, countzmin, next_pnr, next_pnr2, 
                               count, nearest_laser_point, index2, index1,
                               pol_num,j,jj,n, edge_size, edgelines_size, number_offset2,
                               changed, success, iter;
  FILE                         *fd, *fdimage, *dxffile;
  Image                        kernel, large_kernel;
  TINEdges                     edges, new_edges, edges2, new_edges2, *edges_3d, edges3, edges22;
  double                        dist, meanx, meany, meanz, max_diff, diff, dist1, dist2, height,
                               remember_height;
  vector<int>                  point_count;
  bool                         found, done, changedb;
  DataBounds3D                 bounds;
  Plane                        plane;
  TIN                          tin;
  double smooth_height, difference;
  double max_dist_grow = 20;
  LaserPoints                  laser_points, sel_laser_points, nb_laser_points, temp_height_points,
                               deviation_points, inside_points;
  LaserPoints::iterator        laser_point, node;
  LaserPoint nb_laser_point, deviation_point; 
  
  double min_dist              = 0.1;
  double min_dist2             = 0.4;
  double max_road_slope        = 0.1;
  double max_road_slope2        = 0.2; //pcp0.5 and option trow away roads
  double max_vegetation_slope  = 0.3;
  //THings to do in 2007: make a better solution for throwing away roads with steep slopes: fix them.
if (!map_points.Read(map_points_input)) {
    printf("Error reading map points from file %s\n", map_points_input);
    exit(0);
  }
if (!map_lines.Read(map_topology_input)) {
    printf("Error reading map lines from file %s\n", map_topology_input);
    exit(0);
  }
/*number_offset = map_points.HighestPointNumber().Number()+1;
map_points.DuplicateWithFixedZ(0, number_offset);
tin = map_points.Triangulate();
keep_map_lines = LineTopologies(tin);
keep_map_lines.AddWalls(map_lines, number_offset);
if (!map_points.Write(map_points_output))
    printf("Error writing the 3D map points\n");
    if (!keep_map_lines.Write(map_topology_output))
    printf("Error writing the topology of the 3D lines\n");
return;
for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
    if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end());
    if(!keep_map_lines.empty()) keep_map_lines.erase(keep_map_lines.begin(), keep_map_lines.end());
    for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
           sel_map_point = *(map_points.PointIterator(*node2));
           sel_map_points.push_back(sel_map_point);
         }
    one_map_line.erase(one_map_line.begin(), one_map_line.end());
    one_map_line.push_back(*map_line);
    sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
    one_map_line.ReNumber(sel_map_points, 0, 0);
   tin = sel_map_points.Triangulate(one_map_line);
    keep_map_lines = LineTopologies(tin);
    tin.Erase();
    keep_map_lines = LineTopologies();
    number_offset = sel_map_points.HighestPointNumber().Number()+1;
    sel_map_points.DuplicateWithFixedZ(0, number_offset);
    keep_map_lines.AddWalls(one_map_line, number_offset);
    if (!all_tin_points.empty()) keep_map_lines.ReNumber(sel_map_points, (all_tin_points.end()-1)->Number()+1, (all_tin_lines.end()-1)->Number()+1);
    all_tin_lines.insert(all_tin_lines.end(), keep_map_lines.begin(), keep_map_lines.end());
    all_tin_points.insert(all_tin_points.end(), sel_map_points.begin(), sel_map_points.end());   
}
//map_lines.insert(map_lines.end(), keep_map_lines.begin(), keep_map_lines.end());
  map_lines.ReNumber(map_points, 3000, 5000);
  all_tin_lines.insert(all_tin_lines.end(), map_lines.begin(), map_lines.end());
    all_tin_points.insert(all_tin_points.end(), map_points.begin(), map_points.end()); 
    all_tin_points.RemoveDoublePoints(all_tin_lines, 0.2);  
  if (!all_tin_points.Write(map_points_output))
    printf("Error writing the 3D map points\n");
    if (!all_tin_lines.Write(map_topology_output))
    printf("Error writing the topology of the 3D lines\n");
    dxffile = fopen ("walls.dxf","w");
    all_tin_points.WriteDXFMesh(dxffile, all_tin_lines, 3, true, true, true);
        fclose(dxffile);
return;*/
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
  keep_map_points = map_points;
  keep_map_lines = map_lines;
  
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
map_points.insert(map_points.end(), new_map_points.begin(), new_map_points.end());  
 next_pnr = map_points.HighestPointNumber().Number()+1; 
 new_map_points.erase(new_map_points.begin(), new_map_points.end());
 
for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++){
    if (map_line->TOP10MajorClass() == TOP10_Road) {
       common_parts.erase(common_parts.begin(), common_parts.end());
       common_parts.push_back(*map_line);
//       for (node2=map_line->begin(); node2!=map_line->end()-1; node2++) {
//              common_points.push_back(*(map_points.PointIterator(*node2)));
//       }
       for (map_line2=map_line +1; map_line2!=map_lines.end();map_line2++){
              if (map_line->Adjacent(*map_line2, 10)){                                           
                  common_parts.push_back(*map_line2);
            //      nb_laser_points.ErasePoints();                             
 /*                 for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
                       this_point = map_points.PointIterator(*node2);
                       nb_laser_point.X()=this_point->X();
                       nb_laser_point.Y()=this_point->Y();
                       nb_laser_point.Z()=this_point->Z();
                       nb_laser_point.SetPointNumber(this_point->Number());
                       nb_laser_point.Label(map_line->Number());
                 //      nb_laser_points.push_back(nb_laser_point);
                       if (nb_laser_point.InsidePolygon(map_points, map_line2->LineTopologyReference())){
                          if (map_line2->NodePointer(this_point->Number())==NULL){                                          
                           inside_points.push_back(nb_laser_point);
//                           map_line->erase(node2);
//                           node2--;
                           printf("%d\n", map_line->Number());
                           }
                       }
                           
                 }
   */    //      }
            }
       }
     map_points.AverageHeightsOfDoublePoints(common_parts, 0.2, 2.5);  
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
     map_points.AverageHeightsOfDoublePoints(common_parts, 0.3, 2.5);  
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
   /*   map_lines.erase(map_line);
      map_line--;
     */ 
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
     map_points.AverageHeightsOfDoublePoints(common_parts, 0.3, 2.5);  
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
     map_points.AverageHeightsOfDoublePoints(common_parts, 0.3, 2.5);  
     }
   }
   

  
  for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++){
    if (map_line->TOP10MajorClass() == TOP10_Road) {
       nb_laser_points.ErasePoints();                             
       
       for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
             this_point = map_points.PointIterator(*node2);
             nb_laser_point.X()=this_point->X();
             nb_laser_point.Y()=this_point->Y();
             nb_laser_point.Z()=this_point->Z();
             nb_laser_point.SetPointNumber(this_point->Number());
             nb_laser_points.push_back(nb_laser_point);
             }
       nb_laser_points.Label(map_line->Number());      
       for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
             temp_height_points.ErasePoints();
             this_point = map_points.PointIterator(*node2);      
             for (laser_point=nb_laser_points.begin(); laser_point!=nb_laser_points.end(); laser_point++){
                  dist = (laser_point->vect2D()-this_point->vect2D()).Length();                                          
                  if (dist < max_dist_grow) {
                      temp_height_points.push_back(*laser_point);
                  }
             }
             plane = temp_height_points.FitPlane(map_line->Number(), map_line->Number(), LabelTag);
             smooth_height = plane.Z_At(this_point->X(), this_point->Y(), &success);
             difference = fabs(smooth_height-this_point->Z()); 
             printf("difference = %5.2f\n", difference);
         //    if (fabs(difference) > 0.5){
                 deviation_point.X()=this_point->X();
                 deviation_point.Y()=this_point->Y();
                 deviation_point.Z()=difference;
                 deviation_point.SetPointNumber(this_point->Number());
                 deviation_points.push_back(deviation_point);
          //       }       
             this_point->Z() = smooth_height;                            
       }
    }
   }
  map_points.AverageHeightsOfDoublePoints(map_lines, 0.3, 2.0);           
  map_points.RemoveDoublePoints(map_lines, 0.2);
  
             
  if (!map_points.Write(map_points_output))
    printf("Error writing the 3D map points\n");
    if (!map_lines.Write(map_topology_output))
    printf("Error writing the topology of the 3D lines\n");
    if (!inside_points.Write(laser_output, false))
        printf("Error writing laser points.\n");
      
}
