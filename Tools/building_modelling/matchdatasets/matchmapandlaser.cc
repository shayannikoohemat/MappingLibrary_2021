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
#include <matrix3.h>
#include "LaserPoints.h"
#include "LineTopsIterVector.h"

/*
--------------------------------------------------------------------------------
                           The main function
--------------------------------------------------------------------------------
*/

void matchmapandlaser(char *laser_input, char *map_points_input,
	       char *map_topology_input, char *tagged_laser_points_output, char *map_points_output,
	       char *map_topology_output, bool select_buildings_plus_overhang, double reduction, bool assign_map2points, bool assign_dtm2map,
           bool select_buildings, int remove_label_value, bool label_by_segments)

{
  LaserPoints                  laser_points, sel_laser_points, out_laser_points,
                               tobedone;
  LaserPoints::iterator        laser_point;
  LaserPoint                   point, previous_point;
  ObjectPoints                 map_points, newmappoints,outbuildingpoints;
  ObjectPoint                  map_point;
  LineTopologies               map_lines, sel_map_lines, sorted_map_lines, empty_lines,
                               inner_rings, same_id_lines, outer_rings, master_rings,
                               outbuildings;
  LineTopology                 master_ring;
  LineTopologies::iterator     map_line, last_map_line;
  LineTopology::iterator             node;
  int                          count, success, index, pol_num,
                               iter, index1, index2, pn, value, segment;
  bool                         found, done, remove, attr_removed;
  Plane                        plane;
  vector<int>                  point_count;
  vector <int>                 segment_numbers, label_values, *values = new vector <int>(),
                               inner_ring_numbers, affected_lines;
  vector <int>::iterator       segment_number, label_value, stored_value, same_id;
  bool debug=false;
  DataBounds3D                 bounds;
  Position3D                   pos;
  PointNumberList              pnl;

  double PI = 3.14159, area, orig_size, xmin,ymin,xmax,ymax; 
  
  // Local constants to be made global
  bool   force_polygon_check   = true;
  bool sort_on_label = false;
//  bool select_buildings = true;
  bool remove_empty_polygons = true; 
  bool ignore_inner_rings = false;
  
//  bool select_buildings_plus_overhang = false;
if (select_buildings_plus_overhang) printf("Overhanging parts will be added to segments\n");
else printf("Overhanging parts will not be added to segments\n");

if (assign_map2points) printf("Map polygon information will be added to points\n");
else printf("Map polygon information will not be added to points\n");

if (assign_dtm2map) printf("DTM information will be added to map height attribute (in cm, +10 meter)\n");
else printf("DTM information will not be added to map height attribute\n");
  
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
  
  bool sort_laser_by_gridcel = true; // this is to group laser points gridcel wise, in order to fasten points in polygon operation
  if (sort_laser_by_gridcel){
  printf("Laserpoints are sorted on x coordinate per row of 2 m in y\n");
  LaserPoints sortedpoints, ysortedpoints;
  laser_points.DeriveDataBounds(0);
  xmin = laser_points.DataBounds().Minimum().X();
  ymin = laser_points.DataBounds().Minimum().Y();
  xmax = laser_points.DataBounds().Maximum().X();
  ymax = laser_points.DataBounds().Maximum().Y();
  double ydiff = ymax - ymin;
  //check whether points are in row between ymin and ymin + 2
  do {
   for (laser_point=laser_points.begin();
         laser_point!=laser_points.end(); laser_point++, index1++) {
             if (laser_point->Y()>ymin && laser_point->Y()<ymin+2){                          
             ysortedpoints.push_back(*laser_point);
             }
    }
   printf("%10.1f %10.1f (%4.2f)\r", ymin, ymax, 100*(1-(ymax-ymin)/ydiff));
    ymin = ymin+2;
    ysortedpoints.SortOnCoordinates(); //then sort these on x coordinates first...
    sortedpoints.AddPoints(ysortedpoints);
    ysortedpoints.ErasePoints();
    } while (ymin<ymax);
   laser_points = sortedpoints;
    printf("\nFinished sorting %d points\n", laser_points.size());
bool set_points_at_zero = false;
 if (set_points_at_zero){
  ObjectPoints pointsatzero;
  printf("Start setting heights at zero...\n");
  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++, count++) {
       printf("%7d  %5.1f \r", count, 100.0 * count / map_lines.size());
 
         for(node = map_line->begin(); node!=map_line->end()-1; node++){
            map_point = *(map_points.PointIterator(*node));
            map_point.Z() = 0;
            pointsatzero.push_back(map_point);
         }
    }
    map_points = pointsatzero;
}
                             }
printf("Start removing double points...(this can take a while)\n");
map_points.RemoveDoublePoints(map_lines, 0.01);
printf("Finished removing double points...\n");
 for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (!map_line->IsClosed()) {
      map_lines.erase(map_line);
      map_line--;
      count++;
    }
  }
  printf("Removed %d not closed polygons\n", count);

 for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (map_line->CalculateArea(map_points) < 0.5) {
      map_lines.erase(map_line);
      map_line--;
      count++;
    }
  }
  printf("Removed %d polygons smaller than 0.5 m2\n", count);



map_points.Write(map_points_input);
map_lines.Write(map_topology_input, false);

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

/*  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
//    if (map_line->Attribute(LineLabelTag)==2) map_line->Label() = 6001; //water
    if (map_line->Attribute(LineLabelTag)==3) map_line->Label() = 6001; //water
    if (map_line->Attribute(LineLabelTag)==1) map_line->Label() = 5001; //meadow
    if (map_line->Attribute(LineLabelTag)==4) map_line->Label() = 2001; //road
//    if (map_line->Attribute(LineLabelTag)==3) map_line->Label() = 1001; //buildings
    if (map_line->Attribute(LineLabelTag)==2) map_line->Label() = 1001; //buildings
    if (map_line->Attribute(LineLabelTag)>99 && map_line->Attribute(LineLabelTag)<110) map_line->Label() = 1001;
    if (map_line->Attribute(LineLabelTag)==1101) {
        outbuildings.push_back(*map_line);
        map_lines.erase(map_line);
        map_line--;
        }
//    }
  }
  */
  inner_ring_numbers = map_lines.AttributedValues(HoleTag);
  if (inner_ring_numbers.size() > 0) {
    ignore_inner_rings = true;
    printf("\nInner rings will be used to remove points inside the inner ring\n");
 }
 else {
      printf("\nThere are no inner rings in this dataset/structure\n");
      }

    for (map_line=outbuildings.begin(), index2=0;
         map_line!=outbuildings.end(); map_line++, index2++) {
         printf("%7d  %5.1f \r", index2, 100.0 * index2 / outbuildings.size());
         for(node = map_line->begin(); node!=map_line->end()-1; node++){
            map_point = *(map_points.PointIterator(*node));
            outbuildingpoints.push_back(map_point);
         }
    }
  outbuildingpoints.Write("outbuildingspoints.objpts");
  outbuildings.Write("outbuildingspoints.top", false);
    
  // Remove unsegmented points
  if (label_by_segments) printf("Start removing unsegmented points && collecting segment numbers (takes some time)\n");
  else   printf("Start removing unsegmented points\n");

  LaserPoints cleaned_laserpoints;
 /* for (laser_point=laser_points.begin(), index1=0;
         laser_point!=laser_points.end(); laser_point++, index1++) {
         printf("%5.1f \r", 100.0 * index1 / laser_points.size());
        if (laser_point->Attribute(LabelTag)>7){
          cleaned_laserpoints.push_back(*laser_point);
          }
    }
    printf("\nFinished removing %d unsegmented points\n", laser_points.size() - cleaned_laserpoints.size());
    laser_points = cleaned_laserpoints;
    laser_points.Write("ground.laser", false);
return;
    cleaned_laserpoints.ErasePoints();  
*/
    
    for (laser_point=laser_points.begin(), index1=0;
         laser_point!=laser_points.end(); laser_point++, index1++) {
         printf("%5.1f \r", 100.0 * index1 / laser_points.size());
        if (laser_point->HasAttribute(SegmentNumberTag)){
          cleaned_laserpoints.push_back(*laser_point);
          if (label_by_segments){
              value = laser_point->Attribute(SegmentNumberTag);
              for (stored_value=values->end(), found=false;
                   stored_value!=values->begin() && !found; stored_value--)
                   if (value == *stored_value) found = true;
              if (!found) values->push_back(value);
          }
        }
        
    }
    if (cleaned_laserpoints.size()==0){
         printf("\ndataset not segmented, keep all points\n");
         }
    else{
       printf("\nFinished removing %d unsegmented points\n", laser_points.size() - cleaned_laserpoints.size());
       laser_points = cleaned_laserpoints;
    }
    
    if (label_by_segments) segment_numbers = *values;
    cleaned_laserpoints.ErasePoints();  
   printf("\nReduction = %4.2f\n", reduction);
    if (reduction>0) {
      laser_points.ReduceData(reduction);
      printf("\nAfter reduction, laser file size = %d\n", laser_points.size());
      }
   if (sort_on_label){
      printf("Start sorting map_lines to label values\n");
      printf("Removing map lines with label %d\n", remove_label_value);
      
      label_values = map_lines.AttributedValues(LineLabelTag);
      sort(label_values.begin(), label_values.end());
      for (label_value=label_values.begin(); label_value!=label_values.end(); label_value++) {
          if (*label_value != remove_label_value){
          printf("label value %d\n", *label_value);
          sel_map_lines.erase(sel_map_lines.begin(), sel_map_lines.end());
          sel_map_lines = map_lines.SelectAttributedLines(LineLabelTag, *label_value);
          sorted_map_lines.insert(sorted_map_lines.end(), sel_map_lines.begin(), sel_map_lines.end());
          }
          }
      map_lines = sorted_map_lines;
      printf("Finished sorting map_lines\n");
      }
      
    if (select_buildings){
      printf("Selecting map_lines of buildings\n");
      sel_map_lines.erase(sel_map_lines.begin(), sel_map_lines.end());
      for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
         if (map_line->TOP10MajorClass() == TOP10_Building) {
            sel_map_lines.push_back(*map_line);
       }
      }
      printf("Finished selecting %d (out of %d)map lines \n", sel_map_lines.size(), map_lines.size());
      map_lines = sel_map_lines;
    }
   
  
  if (assign_dtm2map){
    double nbh_radius = 3;
    LineTopologies    map_lines_height;
    PointNumberList::iterator          noden;
    PointNumberList neighbourhood;
    int             nearest_laser_point;
    LaserPoints     dtmpoints, sel_laser_points;
    TINEdges        edges;
    dtmpoints.Read("../dtmL03_aoi.laser");
    if (dtmpoints.size()==0) dtmpoints.Read("groundlevel.laser");
    if (dtmpoints.size()==0) {
                             dtmpoints = laser_points;
                             printf("DTM file not found, taking points from input laser points\n");
                             }
    printf("Assigning DTM height in cm to map_line attribute predicted height. Added 10 meters (1000 cm) to avoid problems below sea level...\n");
      dtmpoints.DeriveTIN();
      edges.Derive(dtmpoints.TINReference());

    for (map_line=map_lines.begin(), index2=0;
         map_line!=map_lines.end(); map_line++, index2++) {
         printf("%7d  %5.1f \r", index2, 100.0 * index2 / map_lines.size());
         sel_laser_points.ErasePoints();
         for(node = map_line->begin(); node!=map_line->end(); node++){
            map_point = *(map_points.PointIterator(*node));
            
            nearest_laser_point = dtmpoints.NearestPoint(map_point.Position3DRef(),
                                                    edges, true);
            nbh_radius = 3;
            do{
            neighbourhood = dtmpoints.Neighbourhood(PointNumber(nearest_laser_point),
                                       nbh_radius, edges,
                                       true, false);
            nbh_radius = nbh_radius*2;
            } while (neighbourhood.size()<2 && nbh_radius<20);
            
            for (noden=neighbourhood.begin(); noden!=neighbourhood.end(); noden++){
              sel_laser_points.push_back(dtmpoints[noden->Number()]);
            }
         }
         sel_laser_points.DeriveDataBounds(0);
 //        map_line->Number() = index2;
         map_line->Attribute(PredictedHeight) = 1000 + int(100*sel_laser_points.DataBounds().Minimum().Z()); //DTM height in cm, assigned to attribute predicted height
         map_lines_height.push_back(*map_line);
    }
    pn=0;
    for (map_line=map_lines_height.begin(), index2=0;
         map_line!=map_lines_height.end(); map_line++, index2++) {
         printf("%7d  %5.1f \r", index2, 100.0 * index2 / map_lines_height.size());
         for(node = map_line->begin(); node!=map_line->end(); node++, pn++){
            map_point = *(map_points.PointIterator(*node));
            map_point.Z() = 0.01*(map_line->Attribute(PredictedHeight)-1000);
            map_point.Number() = pn;
            node->Number() = pn;
            newmappoints.push_back(map_point);
         }
    }
    newmappoints.RemoveDoublePoints(map_lines_height, 0.001);
    newmappoints.Write("newmappoints.objpts");
    newmappoints.Write(map_points_input);
    map_lines_height.Write(map_topology_input, false);

    map_points = newmappoints;
    map_lines = map_lines_height;
    }
    
  if (assign_map2points){
bool set_points_at_zero = false;
 if (set_points_at_zero){
  ObjectPoints pointsatzero;
  printf("Start setting heights at zero...\n");
  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++, count++) {
       printf("%7d  %5.1f \r", count, 100.0 * count / map_lines.size());
 
         for(node = map_line->begin(); node!=map_line->end()-1; node++){
            map_point = *(map_points.PointIterator(*node));
            map_point.Z() = 0;
            pointsatzero.push_back(map_point);
         }
    }
    map_points = pointsatzero;
}

  bool export_vertical_segments;
  LaserPoints allverticalsegments, seg_laser_points;
  export_vertical_segments = false;
  if (export_vertical_segments){
 if (!label_by_segments)  segment_numbers = laser_points.AttributeValues(SegmentNumberTag);
   
   for (segment_number=segment_numbers.begin(), index1=0; segment_number!=segment_numbers.end(); segment_number++, index1++) {
        printf("%7d  %5.1f \r", index1, 100.0 * index1 / segment_numbers.size());
        seg_laser_points.ErasePoints();
        seg_laser_points.AddTaggedPoints(laser_points, *segment_number, SegmentNumberTag);
        
        if (seg_laser_points.size()>50){
         plane = laser_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
         if (plane.IsVertical(10*PI/180)){
           allverticalsegments.AddTaggedPoints(laser_points, *segment_number, SegmentNumberTag);
           }
         }
         
   }                    
   allverticalsegments.Write("allverticalsegments.laser", false);
//   return;
   }
//   bool   label_by_segments = true;
  if (label_by_segments){
   printf("\nStart labelling by segments. Counting segments...\n");
 //  segment_numbers = laser_points.AttributeValues(SegmentNumberTag);
   printf("Number of segments: %d\n", segment_numbers.size());
   
   for (segment_number=segment_numbers.begin(), index1=0; segment_number!=segment_numbers.end(); segment_number++, index1++) {
        printf("%7d  %5.1f \r", index1, 100.0 * index1 / segment_numbers.size());
        seg_laser_points.ErasePoints();
  //      seg_laser_points.AddTaggedPoints(laser_points, *segment_number, SegmentNumberTag);
         pnl = laser_points.TaggedPointNumberList(SegmentNumberTag, *segment_number);
//         pos = laser_points.CentreOfGravity(pnl);
         map_point = laser_points.Centroid(pnl, *segment_number);

         point = LaserPoint(map_point.X(), map_point.Y(), map_point.Z());
        for (map_line=map_lines.begin(), found = false, index2=0;
             map_line!=map_lines.end() && !found; map_line++, index2++) {
                   
          if (point.InsidePolygon(map_points,
                                         map_line->LineTopologyReference())) {
            found = true;
            seg_laser_points.AddTaggedPoints(laser_points, *segment_number, SegmentNumberTag);
            for (laser_point=seg_laser_points.begin(), count=0;
                 laser_point!=seg_laser_points.end(); laser_point++) {
                 if (laser_point->InsidePolygon(map_points,
                                         map_line->LineTopologyReference())){
                 count++;
//                  laser_point->Attribute(PolygonNumberTag) = map_line->Number();
                  laser_point->Attribute(PolygonNumberTag) = map_line->Attribute(BuildingNumberTag);
                  
                  laser_point->Attribute(LabelTag)         = map_line->Label();
                  if (map_line->HasAttribute(IDNumberTag)) laser_point->Attribute(PlaneNumberTag) = map_line->Attribute(IDNumberTag);
                  //        point_count[index2]++;
                  out_laser_points.push_back(*laser_point);
                 }
                 else  tobedone.push_back(*laser_point);
            }
            }
//            }
  //        }
        }
//         out_laser_points.AddPoints(seg_laser_points);
   }                    
    
printf("\nsize of points to be done: %d\n", tobedone.size());
    last_map_line = map_lines.begin();
    index2 = 0;
    for (laser_point=tobedone.begin(), index1=0, count=0;
         laser_point!=tobedone.end(); laser_point++, index1++) {
      // First check if the point is inside the map line of the previous point
//      if (laser_point->InsidePolygon(map_points,
//                                     last_map_line->LineTopologyReference()))
      if (laser_point->InsidePolygonJordan(map_points,
                                     last_map_line->LineTopologyReference())==1)
        found = true;
      else { // Check all map lines
        for (map_line=map_lines.begin(), found = false, index2=0;
             map_line!=map_lines.end() && !found; map_line++, index2++) {
             
//          if (laser_point->InsidePolygon(map_points,
//                                         map_line->LineTopologyReference())) {
          if (laser_point->InsidePolygonJordan(map_points,
                                         map_line->LineTopologyReference())==1) {
            found = true;
            last_map_line = map_line;
   //         printf("laser map line: %d \n", last_map_line->Number());
          }
         

        }
      }
      if (found) {
//        laser_point->Attribute(PolygonNumberTag) = last_map_line->Number();
        laser_point->Attribute(PolygonNumberTag) = last_map_line->Attribute(BuildingNumberTag);
        
        laser_point->Attribute(LabelTag)         = last_map_line->Label();
        if (last_map_line->HasAttribute(IDNumberTag)) laser_point->Attribute(PlaneNumberTag) = last_map_line->Attribute(IDNumberTag);
//        point_count[index2]++;
          out_laser_points.push_back(*laser_point);
      }
      else count++;
      if (index1 == (index1/100)*100) {
        printf("%7d  %5.1f %7d\r", index1, 100.0 * index1 / tobedone.size(),
               count);
   //     fflush(stdout);
          }
          }
          
   
out_laser_points.Write(tagged_laser_points_output, false);
    printf("\nDone with deriving PolygonNumberTags and LabelTags.\n");

    map_points.Write(map_points_output);
    map_lines.Write(map_topology_output, false);
    return;
   }



//  printf("sort laserpoints\n");
 // laser_points.ReduceData(1);
  //laser_points.SortOnCoordinates(); //maybe not so good idea as it sorts all points first on x, then y
 // printf("laserpoints sorted\n");
  // Check if the laser points have the required attributes
 
/*  // Remove invisible polygons
  for (map_line=map_lines.begin(), count=0;
       map_line!=map_lines.end(); map_line++) {
    if (map_line->TOP10Invisible()) {
      map_lines.erase(map_line);
      map_line--;
      count++;
    }
  }
  printf("Removed %d invisible polygons\n", count);
  */
 // map_points.RemoveDoublePoints(map_lines, 0.01);
  
  outer_rings = map_lines.SelectAttributedLines(HoleTag, 0);
//  outer_rings = map_lines;
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
//    point_count.resize(map_lines.size());
    
 //   for (index2=0; index2<map_lines.size(); index2++) point_count[index2] = 0;
 
    
 //   last_map_line = map_lines.begin();
   last_map_line = outer_rings.begin();
   bool last_map_label_isground = false;
   
    index2 = 0;
    for (laser_point=laser_points.begin(), index1=0, count=0;
         laser_point!=laser_points.end(); laser_point++, index1++) {
      // First check if the point is inside the map line of the previous point
 //     printf("%7d  %5.1f %7d\r", index1, 100.0 * index1 / laser_points.size(),
 //              count);   
  //    printf("punt %17d  %7d\n", index1, count);   

      if (laser_point->InsidePolygonJordan(map_points,
                                     last_map_line->LineTopologyReference())==1){ // and ! lastmaplabelisground removed
        found = true;
  //    printf("last %17d  %7d last %d\n", index1, count, last_map_line->Number());   
               }
      else { // Check all map lines
      found = false;
        for (map_line=last_map_line, index2=0;
             map_line!=outer_rings.end() && !found && index2<200; map_line++, index2++) {
  //        printf("look %7d \r", index2);  
            
          if (laser_point->InsidePolygonJordan(map_points,
                                         map_line->LineTopologyReference())==1) {
            found = true;
            last_map_line = map_line;
//            printf("cl+se%17d  %7d cl+se  %d\n", index1, count, last_map_line->Number());   

  //          last_map_label_isground = false;
//         if (last_map_line->Label()==5001) last_map_label_isground = true;
   //         printf("laser map line: %d \n", last_map_line->Number());
          }
        }
        for (map_line=last_map_line, index2=0;
             map_line!=outer_rings.begin() && !found && index2<200; map_line--, index2++) {
  //        printf("look %7d \r", index2);  
            
          if (laser_point->InsidePolygonJordan(map_points,
                                         map_line->LineTopologyReference())==1) {
            found = true;
            last_map_line = map_line;
 //           printf("cl-se%17d  %7d cl-se %d\n", index1, count, last_map_line->Number());   

  //          last_map_label_isground = false;
//         if (last_map_line->Label()==5001) last_map_label_isground = true;
   //         printf("laser map line: %d \n", last_map_line->Number());
          }
        }
        for (map_line=outer_rings.begin(), index2=0;
             map_line!=outer_rings.end() && !found; map_line++, index2++) {
     //     printf("look %7d \r", index2);  
            
          if (laser_point->InsidePolygonJordan(map_points,
                                         map_line->LineTopologyReference())==1) {
            found = true;
            last_map_line = map_line;
 //           printf("look %17d  %7d new  %d\r", index1, count, last_map_line->Number());   

  //          last_map_label_isground = false;
//         if (last_map_line->Label()==5001) last_map_label_isground = true;
   //         printf("laser map line: %d \n", last_map_line->Number());
          }
        }

      }
      if (found) {
  //      laser_point->Attribute(PolygonNumberTag) = last_map_line->Number();
        laser_point->Attribute(PolygonNumberTag) = last_map_line->Attribute(BuildingNumberTag);
        
        laser_point->Attribute(LabelTag)         = last_map_line->Label();
        if (last_map_line->HasAttribute(IDNumberTag)) laser_point->Attribute(PlaneNumberTag) = last_map_line->Attribute(IDNumberTag);
//        point_count[index2]++;
      }
      else {
           count++;
 //          printf("punt %17d  %7d\n", index1, count);
           }
      if (index1 == (index1/100)*100) {
        printf("%7d  %5.1f %7d\r", index1, 100.0 * index1 / laser_points.size(),
               count);
   //     fflush(stdout);
      }
    }
    printf("These tags will be saved to the output file\n");
//    sel_laser_points = laser_points.SelectTagValue(LabelTag, last_map_line->Label());
//    sel_laser_points.Write(tagged_laser_points_output, false);
//ignore_inner_rings = false;
    if (ignore_inner_rings){
 inner_rings = map_lines.SelectAttributedLines(HoleTag, 1);
 outer_rings = map_lines.SelectAttributedLines(HoleTag, 0);

 affected_lines = inner_rings.AttributedValues(IDNumberTag);
 printf("\n%d inner rings will be used to remove points inside the inner ring.", inner_rings.size());
 printf("\n%d major lines.\n", affected_lines.size());
  for (same_id=affected_lines.begin(), index1=0; same_id!=affected_lines.end(); same_id++, index1++) {
        printf("%7d  %5.1f \n", index1, 100.0 * index1 / affected_lines.size());
        same_id_lines = inner_rings.SelectAttributedLines(IDNumberTag, *same_id);
        seg_laser_points.ErasePoints();
        seg_laser_points.AddTaggedPoints(laser_points, *same_id, PlaneNumberTag);
        printf("\n%d seg laser points.", seg_laser_points.size());
        laser_points.RemoveTaggedPoints(*same_id, PlaneNumberTag);
        master_rings = outer_rings.SelectAttributedLines(IDNumberTag, *same_id);
        master_ring = master_rings[0];
        orig_size = master_ring.CalculateArea(map_points);
        printf("size of polygon %4.2f \n", orig_size);
        
        for (laser_point=seg_laser_points.begin(), index2=0;laser_point!=seg_laser_points.end(); laser_point++, index2++) {
          printf("%7d  %5.1f \r", index2, 100.0 * index2 / seg_laser_points.size());
          attr_removed = false;
          orig_size = master_ring.CalculateArea(map_points);
          segment = laser_point->Attribute(SegmentNumberTag);
          for (map_line=same_id_lines.begin(); map_line!=same_id_lines.end() && !attr_removed; map_line++) {
            if (laser_point->InsidePolygonJordan(map_points,
                                         map_line->LineTopologyReference())==1) {
               laser_point->RemoveAttributes();
        //       printf("removed attribute ");
               attr_removed = true;
               }
          }
          // check if in any polygon, which is inside ring
          if (attr_removed){
           for (map_line=outer_rings.begin(); map_line!=outer_rings.end(); map_line++) {
            if (laser_point->InsidePolygonJordan(map_points,
                                         map_line->LineTopologyReference())==1){
//              printf("maybe into %d (%4.2f)", map_line->Number(), map_line->CalculateArea(map_points));
              if(map_line->CalculateArea(map_points)<orig_size){
               orig_size = map_line->CalculateArea(map_points);
//               laser_point->Attribute(PolygonNumberTag) = map_line->Number();
               laser_point->Attribute(PolygonNumberTag) = map_line->Attribute(BuildingNumberTag);
               laser_point->Attribute(LabelTag)         = map_line->Label();
               laser_point->Attribute(SegmentNumberTag) = segment;
               if (map_line->HasAttribute(IDNumberTag)) laser_point->Attribute(PlaneNumberTag) = map_line->Attribute(IDNumberTag);
     //           printf("changed into %d\n", map_line->Number());
               }
               }
          }
          }
        }
 //       printf("\n");
        laser_points.AddPoints(seg_laser_points);
  }
    
}
    laser_points.Write(tagged_laser_points_output, false);
    printf("\nDone with deriving PolygonNumberTags and LabelTags.\n");

    if (remove_empty_polygons){
       for (map_line=map_lines.begin(), index2=0;
         map_line!=map_lines.end(); map_line++, index2++) {
         sel_laser_points.ErasePoints();                                                    
//         sel_laser_points.AddTaggedPoints(laser_points, map_line->Number(),
//                                         PolygonNumberTag);
         sel_laser_points.AddTaggedPoints(laser_points, map_line->Attribute(BuildingNumberTag),
                                         PolygonNumberTag);
                                         
          area = map_line->CalculateArea(map_points); 
             printf("area = %4.2f, number of points = %d\n", area, sel_laser_points.size());
             remove = false;
         if (map_line->CalculateArea(map_points) > sel_laser_points.size()*10*(reduction+1) && sel_laser_points.size()<10000){
       //     map_line->Label() = map_line->Label()+1;
            empty_lines.push_back(*map_line);
//           map_lines.erase(map_line);
//           map_line--;
             remove = true;
           }
           if (sel_laser_points.size()==0){
//           map_lines.erase(map_line);
   //        empty_lines.push_back(*map_line);
  //         map_line--;
   //         remove = true;
           }
           if (remove  && map_line->Label() != 6001){
              map_lines.erase(map_line);
              map_line--;
           }
         
       }
       for (map_line=map_lines.begin(), index2=0;
         map_line!=map_lines.end(); map_line++, index2++) {
          for(node = map_line->begin(); node!=map_line->end(); node++){
            map_point = *(map_points.PointIterator(*node));
            newmappoints.push_back(map_point);
         }
         }
         newmappoints.RemoveDoublePoints(map_lines, 0.01);
//       newmappoints.Write(map_points_output);
//       map_lines.Write(map_topology_output, false);
         map_points = newmappoints;
       empty_lines.Write("emptylines.top", false);
    }  
//    sel_laser_points = laser_points.SelectTagValue(LabelTag, 1001);
}
   if (select_buildings_plus_overhang){
   LaserPoints                  seg_laser_points, all_seg_laser_points, p1, p2;
   
   int                          part1, part2;
   printf ("adding nearby laser points with same segment number.\n");
   for (map_line=map_lines.begin(), index2=0;
         map_line!=map_lines.end(); map_line++, index2++) {
   //      printf("map line number: %d ", map_line->Number());                           
         sel_laser_points.ErasePoints();                                                    
         sel_laser_points.AddTaggedPoints(laser_points, map_line->Attribute(BuildingNumberTag),
                                         PolygonNumberTag);
         segment_numbers = sel_laser_points.AttributeValues(SegmentNumberTag);
         for (segment_number=segment_numbers.begin();
             segment_number!=segment_numbers.end(); segment_number++) {
             seg_laser_points.ErasePoints();
             seg_laser_points.AddTaggedPoints(laser_points, *segment_number, SegmentNumberTag);
             plane = laser_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
             p1.ErasePoints();
             p1.AddTaggedPoints(seg_laser_points, map_line->Attribute(BuildingNumberTag), PolygonNumberTag);
             
             if (p1.size() > 0.5*seg_laser_points.size()&& !plane.IsHorizontal(5*PI/180)){ // if more than 50% is inside the building add them to building
                 seg_laser_points.RemoveTaggedPoints(map_line->Attribute(BuildingNumberTag), PolygonNumberTag);
                 for (laser_point=seg_laser_points.begin(); laser_point!=seg_laser_points.end(); laser_point++){
                    laser_point->Residual() = 0.5;
                    if (laser_point->PolygonNumber()>0){ // if point allready in other polygon
                        laser_point->SetFiltered();        
                    }                                             
                 }
                 seg_laser_points.RemoveFilteredPoints();
  //             seg_laser_points.SetAttribute(ResidualTag, 100);
                 seg_laser_points.AddPoints(p1);              
                 seg_laser_points.SetAttribute(PolygonNumberTag, map_line->Attribute(BuildingNumberTag));
                 all_seg_laser_points.AddPoints(seg_laser_points);
             }
             else { 
                  all_seg_laser_points.AddPoints(p1);
                  }
           }  
    }
    all_seg_laser_points.ReduceData(0.01);
    all_seg_laser_points.Write(tagged_laser_points_output, false);
    printf ("Building laser points plus overhanging parts written to output file.\n");    
    }
    printf("These tags will be saved to the output file\n");
  }
       map_points.Write(map_points_output);
       map_lines.Write(map_topology_output, false);
}
