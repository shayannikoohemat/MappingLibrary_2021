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

 Initial creation of top3d:
 Author : George Vosselman
 Date   : 08-03-2006

 Modified by Sander Oude Elberink, 2006-2013
 removed core, kept part to add buildings on top of 3d terrain polygons...
 buildings and forest (bgt_forest) can be put on top of terrain (+water/roads/foundation buildings)
 
 code from top10 project at kadaster
 
 added hole info, lod 1 and lod2, 20-03-2013
 
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
#include "VRML_io.h"
#include "dxf.h"

/*
--------------------------------------------------------------------------------
                           The main function
--------------------------------------------------------------------------------
*/

void make3dbuilding(char *laser_input, char *map_points_input,
	       char *map_topology_input, char *map_points_output,
	       char *map_topology_output, bool lod1, bool lod2, 
           bool not_histogram_filtering, bool debug, int reductionfactor)

{
  LaserPoints                  laser_points, sel_laser_points, meadow_laser_points,
                               quality_points, poly_laser_points,loc_laser_points,
                               holes_laser, sel_addedpoints,
                               seg_laser_points, projectedpoints, projectedpointsperbuilding,
                               nearbylaserpoints, simplifiedpoints, qualitylaserpoints,
                               maplaserpoints, mixed_points;
  LaserPoints::iterator        laser_point, nearest_point;
  LaserPoint                   nb_laser_point, temp_point, point, mean_point,
                               laspoint, laspoint2, lpoint;
  TINEdges                     edges, seledges, poledges;
  ObjectPoints                 map_points, dtm_tin_points, clean_map_points,
                               floor_tin_building_points, wallpoints, all_wall_points,
                               orig_sel_map_points, sel_intersection_3dpoints,
                               keep_map_points;
  LineTopologies               map_lines, dtm_tin_lines, wall_map_lines, inner_rings,
                               floor_tin_building_lines, all_wall_lines, final_tin_lines,
                               notvalidlines;
  LineTopology                 intsect;
  ObjectPoints::iterator       map_point, previous_point, next_point;
  ObjectPoint                  new_map_point, hole_point, beginp, endp;
  LineTopologies::iterator     map_line, last_map_line, hole_line, map_line_int;
  LineTopsIterVector           sel_map_lines;
  LineTopsIterVector::iterator sel_map_line, sel_map_line2;
  PointNumberList              neighbourhood, road_nbh, pnlseg1, pnlseg2;
  PointNumberList::iterator    node, previous_node, next_node, noden;
  PointNumber                  *node_ptr, pn1, pn2;
  int                          nearest_laser_point, dominant_segment, count,
                               success, index1, index2, index3, close_index1,
                               close_index2, next_point_number, index, pol_num,
                               num_2d_points, index_road, index_non_road,
                               top10class, iter, run_index, notvalid, ii,
                               size1, size2, totedges3, totedges2, sn1, sn2;
  Plane                        plane, plane1, plane2;
  Planes                       planes;
  Line3D                       line, intline;
  vector<double>               heights, height1, height2;
  vector<int>                  use_map_line, use_map_point, point_count;
  vector <int>                 segment_numbers, building_numbers;
 vector <int>::iterator        segment_number, segment_number2, building_number;
 
  double                       diff, min_diff, new_height, dist1, dist2,
                               min_height, meanx, meany, ground_height,
                               pointdistance, dist, PI;
  bool                         found, done, emptymap, pointinside, pointinhole,
                               histogram_filtering;
  DataBounds3D                 bounds;
  TIN                          dtmtin;
  Position3D                   p1, p2, p3, pos, projpos, pos1, pos2;
//  Position2D                   ;
  LineTopology::iterator       nb_node, node2;
//  bool debug=true;
  bool rasterise_points = false;
 Covariance3D cov3d = Covariance3D(0,0,0,0,0,0);
 //countingintsect = 0;
int line_number3 = 0;
PI = 3.14159;

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
  printf ("Reduction factor to decrease amount of print information to screen: %d.\n", reductionfactor);

  histogram_filtering = true;
  if (not_histogram_filtering) histogram_filtering = false;

  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (map_line->TOP10MajorClass() != TOP10_Building){
        map_lines.erase(map_line);
      map_line--;
      count++;
    }
  }
 if (count!=0) printf("Removed %d not buildings polygons\n", count);
  
  if (map_lines.size()==0){
  printf("No building polygons in dataset. Program finished!\n");
  return;
  }
if (!lod2){
 lod1 = true;
 printf("producing lod1 buildings with %d polygons...\n", map_lines.size());
}

if (lod2){
 lod1 = false;
 printf("producing %d 'lod2 lookalike' building polygons...\n", map_lines.size());
 
    segment_numbers = laser_points.AttributeValues(SegmentNumberTag);
  if (segment_numbers.size()<2) {
    printf("TIN version requires laser points with SegmentNumberTags\n");
    printf("Start segmenting laser data\n");
     SegmentationParameters *segpar;
  segpar=new SegmentationParameters();
  segpar->MaxDistanceInComponent()=1;
  segpar->SeedNeighbourhoodRadius()=1;
  segpar->MaxDistanceSeedPlane()=0.15;
  segpar->GrowingRadius()=1;
  segpar->MaxDistanceSurface()=0.25;
 
  laser_points.SurfaceGrowing(*segpar);
 
  }
 
 
     
 if (histogram_filtering){
 LaserPoints             grid_laser_points, filteredpoints, goodgridpoints,
                         sel_goodgridpoints, meangridpoints;
 LaserPoint              mean_point;
 double max_x, max_y, min_x, min_y, percvalue20, percvalue90, x, y;
 int numcols, numrows, count0, sn1;// grid_size;
 double grid_size;
 count0 = 0;
 grid_size = 1;
 printf("filtering high and low points within a gridstructure...\n");
  for (map_line = map_lines.begin(), run_index=0; map_line!=map_lines.end(); map_line++, run_index++){
     if (run_index == (run_index/reductionfactor)*reductionfactor)  printf("%4.1f \r", (100.0*run_index)/map_lines.size());
     sel_laser_points.ErasePoints();
     if (map_line->Attribute(HoleTag) ==1) continue;
     if (map_line->HasAttribute(BuildingNumberTag)) sel_laser_points = laser_points.SelectTagValue(PolygonNumberTag, map_line->Attribute(BuildingNumberTag));
     sel_laser_points.DeriveDataBounds(0);
  //   printf("size of sel laser points...%d\n", sel_laser_points.size());
  //   system("pauze");
     max_x =  (sel_laser_points.DataBounds().Maximum().X());
     max_y =  (sel_laser_points.DataBounds().Maximum().Y());
  min_x =  (sel_laser_points.DataBounds().Minimum().X());
  min_y =  (sel_laser_points.DataBounds().Minimum().Y());
  numcols =  int ((max_x-min_x)/grid_size)+1;
  numrows =  int ((max_y-min_y)/grid_size)+1;
  count0 = 0;
  if (debug) printf("min_x: %4.2f, min_y: %4.2f\n", min_x, min_y);
 if (debug) printf("max_x: %4.2f, max_y: %4.2f\n", max_x, max_y);
  for (x=min_x; x<max_x; x+=(grid_size)){
    for (y=min_y; y<max_y; y+=(grid_size)){
        count0++;
   
 if (debug)        printf("%4.2f %4.2f, %4.2f \r", x, y, count0*100.0/(numcols*numrows));
        grid_laser_points.ErasePoints();
        for (laser_point = sel_laser_points.begin(); laser_point != sel_laser_points.end(); laser_point++){
            if ((laser_point->X() < x+0.5*grid_size) && (laser_point->X() > x-0.5*grid_size)){// here the points are selected that are in a particular grid cell
               if ((laser_point->Y()< y+0.5*grid_size) && (laser_point->Y() > y-0.5*grid_size)){
                  laser_point->Label(10);
                  grid_laser_points.push_back(*laser_point);
               }
            }
        }
        if (grid_laser_points.size()>1){
        grid_laser_points.SwapXZ();
        grid_laser_points.SortOnCoordinates();
        grid_laser_points.SwapXZ();
        goodgridpoints.ErasePoints();      
        sel_goodgridpoints.ErasePoints();           
        percvalue20 = grid_laser_points[int(grid_laser_points.size()/5)].Z();                         
        percvalue90 = grid_laser_points[int(9.0*grid_laser_points.size()/10.0)].Z();                         
        for (laser_point = grid_laser_points.begin(); laser_point != grid_laser_points.end(); laser_point++){
            if(laser_point->Z() >= percvalue20 -0.1 && laser_point->Z() <= percvalue90 + 0.1) {
                   laser_point->Label(10); //point is in between 10th & 90th % point
                   goodgridpoints.push_back(*laser_point);
                }
                else { 
                 laser_point->Label(29); //or 87, 875, 43, ...
                     }
         }
        count = 0;
        sn1 = goodgridpoints.MostFrequentAttributeValue(SegmentNumberTag, count);

        if (count>0){
            for (laser_point = goodgridpoints.begin(); laser_point != goodgridpoints.end(); laser_point++){
                if (laser_point->Attribute(SegmentNumberTag) == sn1)filteredpoints.push_back(*laser_point);
                }
//                    filteredpoints.AddTaggedPoints(goodgridpoints, sn1, SegmentNumberTag);
        }
  //      seg_laser_points.ErasePoints();
  //      seg_laser_points = sel_laser_points.SelectTagValue(SegmentNumberTag, sn1);
        
        mean_point = LaserPoint(x,y,0);
        if (count>0 && mean_point.InsidePolygonJordan(map_points, map_line->LineTopologyReference())) {
          sel_goodgridpoints.AddTaggedPoints(goodgridpoints, sn1, SegmentNumberTag);
          mean_point = LaserPoint(x,y,sel_goodgridpoints.Mean()[2]);
          mean_point.SetAttribute(SegmentNumberTag, sn1);
          mean_point.SetAttribute(PolygonNumberTag,map_line->Attribute(BuildingNumberTag));
          meangridpoints.push_back(mean_point);
          }
        }
        else {
             if (grid_laser_points.size()>0){
              count = 0;
        //      sn1 = goodgridpoints.MostFrequentAttributeValue(SegmentNumberTag, count);                               
              sn1 = grid_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
              sel_goodgridpoints.ErasePoints();   
              sel_goodgridpoints.AddTaggedPoints(grid_laser_points, sn1, SegmentNumberTag);
              mean_point = LaserPoint(x,y,sel_goodgridpoints.Mean()[2]);
              mean_point.SetAttribute(SegmentNumberTag, sn1);
              mean_point.SetAttribute(PolygonNumberTag, map_line->Attribute(BuildingNumberTag));
              meangridpoints.push_back(mean_point);
              }
          }
     }
     }
     }
     if (debug) filteredpoints.Write("filteredpoints.laser", false);
     laser_points = filteredpoints.SelectTagValue(LabelTag, 10);
      if (debug) meangridpoints.Write("meangridpoints.laser", false);
      if (debug) laser_points.Write("tinpoints.laser", false);
  //    laser_points = meangridpoints;
       filteredpoints.ErasePoints();
       meangridpoints.ErasePoints();
       } 
  
}

// Derive the TIN edges of the laser points
  printf("\nDeriving TIN ..."); fflush(stdout);
  laser_points.DeriveTIN();
  printf("done\nDeriving TIN edges ..."); fflush(stdout);
  edges.Derive(laser_points.TINReference());
 printf("done\nDeriving 3d buildings ...\n"); fflush(stdout);
 


  int                number_offset2, number_offset, size, next_pnr, holeornot, i;
  double median, fixed_floor_height, added_floorheight = 0;
  ObjectPoints       sel_map_points, all_tin_building_points, hole_points, hole_map_points,
                     intersection_3dpoints;
  ObjectPoints::iterator       this_point;
  ObjectPoint                  new_point;
  LineTopologies               one_map_line, map_tin_lines, all_tin_building_lines,
                               hole_lines, intersection_lines;
  TIN                          tin;
  number_offset2 = map_points.HighestPointNumber().Number()+1;
  next_pnr = map_points.HighestPointNumber().Number()+1;
  double min_z;
  
  bool add_groundfloor =true;
  inner_rings = map_lines.SelectAttributedLines(HoleTag, 1);
  
  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++, count++) {
       if (debug || count == (count/reductionfactor)*reductionfactor) printf("polygon %d count %d, percentage %4.2f\r", map_line->Attribute(IDNumberTag), count, 100.0*count/map_lines.size());
       sel_laser_points.ErasePoints();
       projectedpointsperbuilding.ErasePoints();
       loc_laser_points.ErasePoints();
       sel_addedpoints.ErasePoints();
       intersection_3dpoints.erase(intersection_3dpoints.begin(), intersection_3dpoints.end());
       intersection_lines.erase(intersection_lines.begin(), intersection_lines.end());
       segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
       if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end());
       if(!map_tin_lines.empty()) map_tin_lines.erase(map_tin_lines.begin(),map_tin_lines.end());
       wallpoints.erase(wallpoints.begin(), wallpoints.end());
       wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
       hole_lines.erase(hole_lines.begin(), hole_lines.end());
       hole_points.erase(hole_points.begin(), hole_points.end());
       holes_laser.ErasePoints();
       if (map_line->Attribute(IDNumberTag) != 0 ) hole_lines = inner_rings.SelectAttributedLines(IDNumberTag, map_line->Attribute(IDNumberTag));
       holeornot = map_line->Attribute(HoleTag);
       if (debug) printf("polygon %d (hole = %d) containing %d holes\n", map_line->Attribute(IDNumberTag), holeornot, hole_lines.size());
        if (holeornot !=0) continue;
        sel_laser_points.ErasePoints();                                                                              
       sel_laser_points.AddTaggedPoints(laser_points, map_line->Attribute(BuildingNumberTag),
                                         PolygonNumberTag);
       if (debug) printf("sel laser points %d \n", sel_laser_points.size());
       if (sel_laser_points.size()<10) continue;
       segment_numbers = sel_laser_points.AttributeValues(SegmentNumberTag);
       ground_height = -20;
       if (map_line->HasAttribute(PredictedHeight)) ground_height = 0.01*(map_line->Attribute(PredictedHeight)-1000);
       pointdistance = sel_laser_points.MedianInterPointDistance(sel_laser_points.size()-1);
       if (debug) printf("seg numbers %4d, point distance %4.2f\n", segment_numbers.size(), pointdistance);
       sel_laser_points.RemoveAlmostDoublePoints(0.1);
       sel_laser_points.DeriveDataBounds(0);
       sel_laser_points.DeriveTIN();
       median = sel_laser_points.ReturnHeightOfPercentilePoint(50);
       min_z = sel_laser_points.Mean()[2]; //will be refined later in the intersection phase
      

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
         if (debug) printf("polygon %d containing %d holes\n", map_line->Number(), hole_lines.size());
 
         for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
             this_point = map_points.PointIterator(*node2);        
             sel_map_points.push_back(*this_point);
             if (ground_height==-20) ground_height = this_point->Z(); // if ground height not speficied, then use height of points
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
          if (!floor_tin_building_points.empty()){
              map_tin_lines.ReNumber(sel_map_points, (floor_tin_building_points.end()-1)->Number()+1, (floor_tin_building_lines.end()-1)->Number()+1);
              }
          
          floor_tin_building_lines.insert(floor_tin_building_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
          floor_tin_building_points.insert(floor_tin_building_points.end(), sel_map_points.begin(), sel_map_points.end());                              
          if(!map_tin_lines.empty()) map_tin_lines.erase(map_tin_lines.begin(),map_tin_lines.end());
   
 
          if (lod2){
      
           for (segment_number=segment_numbers.begin(), index2=0;segment_number!=segment_numbers.end(); segment_number++, index2++) {
         
            if (debug) printf("%4.1f \n", (100.0*index2)/segment_numbers.size());
            seg_laser_points.ErasePoints();
            if (debug) printf("segment %d ", *segment_number);
            seg_laser_points.AddTaggedPoints(sel_laser_points, *segment_number, SegmentNumberTag);
            plane1 = sel_laser_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
            if (plane1.IsVertical(10*PI/180)) continue;
          
            if (seg_laser_points.size()>10){
            if (debug)       printf("seg size %4d \n", seg_laser_points.size());
        
              seg_laser_points.DeriveDataBounds(0);
              if (seg_laser_points.DataBounds().Minimum().Z()<min_z) min_z = seg_laser_points.DataBounds().Minimum().Z();
              
              
              for (laser_point = seg_laser_points.begin(); laser_point !=seg_laser_points.end(); laser_point++){
                pos = Position3D(laser_point->X(), laser_point->Y(), laser_point->Z());
                projpos = plane1.Project(pos);
                laser_point->X() = projpos.X();
                laser_point->Y() = projpos.Y();
                laser_point->Z() = projpos.Z();
              }
              for (ii=0 ; ii<seg_laser_points.size();ii=ii+1) {
                  mean_point = seg_laser_points[ii];
                  if (mean_point.InsidePolygonJordan(map_points, map_line->LineTopologyReference())){
                  projectedpointsperbuilding.push_back(seg_laser_points[ii]);
                  }
              }
            for (segment_number2=segment_number; segment_number2!=segment_numbers.end(); segment_number2++) {
                if (segment_number!=segment_number2){
                if (debug)     printf("segment2 %d ", *segment_number2);
                  totedges3=0;
                  plane2 = sel_laser_points.FitPlane(*segment_number2, *segment_number2, SegmentNumberTag);
                  if (plane2.IsVertical(10*PI/180)) continue;
                  seg_laser_points.ErasePoints();
                  seg_laser_points.AddTaggedPoints(sel_laser_points, *segment_number, SegmentNumberTag);
                  size1 = seg_laser_points.size();
                  seg_laser_points.AddTaggedPoints(sel_laser_points, *segment_number2, SegmentNumberTag);
                  size2 = seg_laser_points.size()-size1;
                if (debug)     printf("size 2 %4d \n", size2);
                  if (size2>10){
                  seg_laser_points.DeriveTIN();
                  edges.Erase();
                  edges.Derive(seg_laser_points.TINReference());
                 if (debug)    printf("%4d \n", seg_laser_points.size());
                  seg_laser_points.RemoveLongEdges(edges, 2*pointdistance, true);
                 if (debug)    printf("long edges removed\n");
                  totedges2 = seg_laser_points.CountMixedEdges(edges, SegmentNumberTag);
               if (debug)      printf("totedges2 = %d\n", totedges2);

                  mixed_points.ErasePoints();
                  if (totedges2>4){ //if connected in 2D
              if (debug)           printf("connected in 2d");
                      mixed_points = seg_laser_points.ReturnMixedEdges(edges, SegmentNumberTag);    
                if (debug)         printf("%4d \n", mixed_points.size());
                      seg_laser_points.RemoveLongEdges(edges, pointdistance, false); //now in 3D
                      totedges3 = seg_laser_points.CountMixedEdges(edges, SegmentNumberTag);
                      sn1 = *segment_number;
                      sn2 = *segment_number2;
                      edges.Erase();
                      pnlseg1.erase(pnlseg1.begin(), pnlseg1.end());
                      pnlseg2.erase(pnlseg2.begin(), pnlseg2.end());
                      pnlseg1 = sel_laser_points.SelectTagValueList(SegmentNumberTag, sn1);
                      pnlseg2 = sel_laser_points.SelectTagValueList(SegmentNumberTag, sn2);
                  if (totedges3 > 4){      // also in 3D connected
                     if (Intersect2Planes(plane1, plane2, intline)){
                        if (sel_laser_points.IntersectFaces(pnlseg1, pnlseg2, plane1, plane2, pointdistance, pos1, pos2)){
                          dist = pos1.Distance(pos2);            
                          laspoint = LaserPoint(pos1.X(), pos1.Y(), pos1.Z());
                          laspoint2 = LaserPoint(pos2.X(), pos2.Y(), pos2.Z());  
                          if (dist>2*pointdistance && laspoint.InsidePolygonJordan(map_points, map_line->LineTopologyReference())&&
                           laspoint2.InsidePolygonJordan(map_points, map_line->LineTopologyReference())){ //for the moment; only look at intersections longer than mll
                          next_pnr++;
                          pn1 = PointNumber(next_pnr);                        
                          next_pnr++;
                          beginp = ObjectPoint(pos1, pn1, cov3d);
                          pn2 = PointNumber(next_pnr);
                          endp = ObjectPoint(pos2, pn2, cov3d);
                          intsect = LineTopology(line_number3, 1, pn1, pn2);
                          sel_intersection_3dpoints.erase(sel_intersection_3dpoints.begin(), sel_intersection_3dpoints.end());
                          sel_intersection_3dpoints.push_back(beginp);
                          sel_intersection_3dpoints.push_back(endp);
                          
                          line_number3++;
                          one_map_line.erase(one_map_line.begin(), one_map_line.end());
                          one_map_line.push_back(intsect);
                          one_map_line.ReNumber(sel_intersection_3dpoints, intersection_3dpoints.size()+1, intersection_lines.size()+1);
                         if (debug)    printf("%d\n", sel_intersection_3dpoints.size());
                          intsect = one_map_line[0];
                           intersection_lines.push_back(intsect);
                           intersection_3dpoints.insert(intersection_3dpoints.end(), sel_intersection_3dpoints.begin(), sel_intersection_3dpoints.end());                        
                          }
                        }
                     }
                 
                  }
                  }
                  }
                }
            }
      }
     }
 


         } //end of lod2
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
    //      sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
          for(map_point=sel_map_points.begin(); map_point!=sel_map_points.end(); map_point++){
              if (lod1) map_point->Z() = median; // what to do with the heights of the initial boundaries...
              if (lod2) map_point->Z() = min_z;
          }
           orig_sel_map_points.erase(orig_sel_map_points.begin(), orig_sel_map_points.end());  
          orig_sel_map_points = sel_map_points;     
          sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
          one_map_line.ReNumber(sel_map_points, 0, 0);
          wallpoints = sel_map_points;
          wallpoints.RemoveDoublePoints(one_map_line, 0.01);
          one_map_line.ReNumber(wallpoints, 0, 0);
          number_offset = wallpoints.HighestPointNumber().Number()+1;
          wallpoints.DuplicateWithFixedZ(ground_height, number_offset);
          wall_map_lines.AddTINWalls(one_map_line, number_offset);
          if (!all_wall_points.empty())
              wall_map_lines.ReNumber(wallpoints, all_wall_points.size()+1, all_wall_lines.size()+1);
          if (map_line->HasAttribute(IDNumberTag)) wall_map_lines.SetAttribute(IDNumberTag, map_line->Attribute(IDNumberTag));
          all_wall_lines.insert(all_wall_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
          all_wall_points.insert(all_wall_points.end(), wallpoints.begin(), wallpoints.end());
          
          one_map_line.erase(one_map_line.begin(), one_map_line.end());
          one_map_line.push_back(*map_line);
          one_map_line.insert(one_map_line.end(), hole_lines.begin(), hole_lines.end());             
          sel_map_points = orig_sel_map_points;          
          sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
          one_map_line.ReNumber(sel_map_points, 0, 0);
          if (lod2){
          intersection_3dpoints.RemoveDoublePoints(intersection_lines, 0.001);
          intersection_lines.ReNumber(intersection_3dpoints, sel_map_points.size(), one_map_line.size());
          for (ii=0 ; ii<intersection_3dpoints.size();ii++) {              
               sel_map_points.push_back(intersection_3dpoints[ii]);
          }
          one_map_line.insert(one_map_line.end(), intersection_lines.begin(), intersection_lines.end());
          sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
      loc_laser_points.ErasePoints();
      simplifiedpoints.ErasePoints();
      
      for (map_line_int = one_map_line.begin(); map_line_int!=one_map_line.end(); map_line_int++){
        for (node2=map_line_int->begin(); node2!=map_line_int->end(); node2++) {      
               lpoint = LaserPoint((sel_map_points.PointIterator(*node2))->X(),(sel_map_points.PointIterator(*node2))->Y(),(sel_map_points.PointIterator(*node2))->Z());
               lpoint.SetAttribute(LabelTag, 1);
               loc_laser_points.push_back(lpoint);
         }
       }
      loc_laser_points.RemoveDoublePoints(false);
      projectedpointsperbuilding.RemoveAttribute(LabelTag);
          projectedpointsperbuilding.SetAttribute(LabelTag,0);
          loc_laser_points.AddPoints(projectedpointsperbuilding);
       simplifiedpoints = loc_laser_points.SimplifyMesh_KeepLabel(0.15, LabelTag, 1);
          if (simplifiedpoints.size()>0) {
            projectedpointsperbuilding.ErasePoints();
            projectedpointsperbuilding = simplifiedpoints;
            }
          projectedpointsperbuilding.RemoveTaggedPoints(1, LabelTag);
          simplifiedpoints.ErasePoints();
          loc_laser_points.ErasePoints();

          next_pnr = sel_map_points.HighestPointNumber().Number()+1;
         for (laser_point=projectedpointsperbuilding.begin(); laser_point!=projectedpointsperbuilding.end(); laser_point++) {
             sel_map_points.push_back(laser_point->ConstructObjectPoint(next_pnr, Covariance3D()));
             next_pnr++;
         }  
        
          } //end lod2
          one_map_line.ReNumber(sel_map_points, 0, 0);
          
          if (debug) one_map_line.Write("test3dbuildings.top", false);
          if (debug) sel_map_points.Write("test3dbuildings.objpts");
          if (debug) projectedpointsperbuilding.Write("test3dbuildings.laser", false);
          
          if (hole_lines.size()>0) tin = sel_map_points.Triangulate(one_map_line, hole_points); 
          else {                
          tin = sel_map_points.Triangulate(one_map_line);
          }
          map_tin_lines = LineTopologies(tin);
          if (debug) printf("size of tin %d\n", map_tin_lines.size());
          tin.Erase();  
          map_tin_lines.SetAttribute(LineLabelTag, map_line->Label());
          map_tin_lines.SetAttribute(IDNumberTag, map_line->Attribute(IDNumberTag));
          if (!all_tin_building_points.empty()){
              map_tin_lines.ReNumber(sel_map_points, (all_tin_building_points.end()-1)->Number()+1, (all_tin_building_lines.end()-1)->Number()+1);
              }
          all_tin_building_lines.insert(all_tin_building_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
          all_tin_building_points.insert(all_tin_building_points.end(), sel_map_points.begin(), sel_map_points.end());                              
          }
printf("\nPreparing for writing output, this can take a while...");  
  if (!floor_tin_building_points.empty()){
     
  floor_tin_building_lines.MakeClockWise(floor_tin_building_points);
  floor_tin_building_lines.ReNumber(floor_tin_building_points, (all_tin_building_points.end()-1)->Number()+1, (all_tin_building_lines.end()-1)->Number()+1);
  floor_tin_building_lines.SetAttribute(LineLabelTag, 4);
     all_tin_building_lines.insert(all_tin_building_lines.end(), floor_tin_building_lines.begin(), floor_tin_building_lines.end());
     all_tin_building_points.insert(all_tin_building_points.end(), floor_tin_building_points.begin(), floor_tin_building_points.end());
  if (debug)    floor_tin_building_lines.Write("groundfloor_building.top", false);
   if (debug)   floor_tin_building_points.Write("groundfloor_building.objpts");
}
  if (!all_tin_building_points.empty() && !all_wall_points.empty()){
     
  all_wall_lines.SetAttribute(LineLabelTag, 5);
  all_wall_lines.ReNumber(all_wall_points, (all_tin_building_points.end()-1)->Number()+1, (all_tin_building_lines.end()-1)->Number()+1);
  all_tin_building_lines.insert(all_tin_building_lines.end(), all_wall_lines.begin(), all_wall_lines.end());
  all_tin_building_points.insert(all_tin_building_points.end(), all_wall_points.begin(), all_wall_points.end());
}  
  all_tin_building_points.RemoveDoublePoints(all_tin_building_lines, 0.01);

   building_numbers = all_tin_building_lines.AttributedValues(IDNumberTag);
     for (building_number = building_numbers.begin(); building_number !=building_numbers.end(); building_number++){
         if (*building_number >= 0){
         map_tin_lines = all_tin_building_lines.SelectAttributedLines(IDNumberTag, *building_number);
         final_tin_lines.insert(final_tin_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
         } 
     }
     printf("\nChecking for not valid polygons...\n");
all_tin_building_lines.erase(all_tin_building_lines.begin(), all_tin_building_lines.end());
    for (map_line = final_tin_lines.begin(), run_index=0, notvalid=0; map_line!=final_tin_lines.end(); map_line++, run_index++){
     if (debug)     printf("%4.1f %4d\r", (100.0*run_index)/final_tin_lines.size(), notvalid);
         if (!map_line->IsValid()) {
            notvalidlines.push_back(*map_line);
            final_tin_lines.erase(map_line);
            map_line--;
            notvalid++;
         }
    }
     if (!all_tin_building_points.Write(map_points_output))
         printf("Error writing the 3D TIN points\n");
     if (!final_tin_lines.Write(map_topology_output, false))
         printf("Error writing the topology of the 3D TIN lines\n");
   if (debug){   if (!notvalidlines.Write("notvalidlines.top", false))
         printf("Error writing the topology of the 3D TIN lines\n");
    }
    printf("\nProgram bgt_buildings finished.\n");
    return;           
}
