
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
 Date   : 20-04-2007

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
#include <Matrix3.h> 
#include "LaserPoints.h"
#include "LineTopsIterVector.h"

/*
--------------------------------------------------------------------------------
                           The main function
--------------------------------------------------------------------------------
*/

void matchreference(char *map_points_input, char *map_topology_input,
                    char *map_points_input_ref, char *map_topology_input_ref,
                    char *laser_input, char *laser_points_inside, char *laser_points_outside)

{
  ObjectPoints                 map_points, map_points_ref, sel_map_points,
                               all_tin_points, orig_points;
  ObjectPoint                  sel_map_point;
  LineTopologies               map_lines, map_lines_ref, one_map_line, 
                               map_tin_lines, all_tin_lines, orig_lines;
  ObjectPoints::iterator       map_point;
  LineTopologies::iterator     map_line, last_map_line, map_line2, map_line3;
  LineTopology::iterator       node2, nb_node;
  LaserPoints                  ref_laser_points, laserin, laserout, laser_points,
                               predicted_points;
  LaserPoint                   ref_laser_point, pred_point;
  LaserPoints::iterator        laser_point;
  PointNumberList              neighbourhood;
  int                          countin, countout, index, success, count,
                               nearest_laser_point;
  TIN                          tin;
  TINEdges                     edges;
  Plane                        plane, qplane;
  Position3D                   p1, p2, p3, q1, q2, q3;
  double                       diff, lasercount, q;
  bool found;
  bool triangulated = false;
  double nbh_radius =15;
  double sample_distance = 5;
  // Read input data
  if (!map_points.Read(map_points_input)) {
    printf("Error reading map points from file %s\n", map_points_input);
    exit(0);
  }
  if (!map_lines.Read(map_topology_input)) {
    printf("Error reading map lines from file %s\n", map_topology_input);
    exit(0);
  }
  if (!map_points_ref.Read(map_points_input_ref)) {
    printf("Error reading map points from file %s\n", map_points_input);
    exit(0);
  }
  if (!map_lines_ref.Read(map_topology_input_ref)) {
    printf("Error reading map lines from file %s\n", map_topology_input);
    exit(0);
  }
  if (!laser_points.Read(laser_input)) {
    printf("Error reading laser points from file %s\n", laser_input);
    exit(0);
  }

  map_points.RemoveDoublePoints(map_lines, 0.01);
  orig_points = map_points;
  orig_lines = map_lines;
  // Densify the map points
  printf("done\nDensifying map points ..."); fflush(stdout);
  count = map_points_ref.size();
  map_lines_ref.Densify(map_points_ref, sample_distance); 
  printf("done, map points increased from %d to %d\n",
         count, map_points_ref.size());
  // Merge points within 0.1 m
  printf("Removing double points ..."); fflush(stdout);
  map_points_ref.RemoveDoublePoints(map_lines_ref, 0.1);
  printf("done, %d map points left\n", map_points_ref.size());
  

//  printf("Deriving TIN ..."); fflush(stdout);
//  laser_points.DeriveTIN();
//  printf("done\nDeriving TIN edges ..."); fflush(stdout);
//  edges.Derive(laser_points.TINReference());
  
  
  if (!triangulated){
                     printf("Road polygons will be triangulated...\n");
  for (map_line=map_lines.begin(), index=0; map_line!=map_lines.end(); map_line++, index++) {
      if (map_line->TOP10MajorClass() == TOP10_Road){
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
         tin = sel_map_points.Triangulate(one_map_line);
         map_tin_lines = LineTopologies(tin);
         tin.Erase();
           for (map_line2=map_tin_lines.begin(); map_line2!=map_tin_lines.end(); map_line2++) {
               map_line2->SetAttribute(LineLabelTag, map_line->Number());
           }
         if (!all_tin_points.empty())
           map_tin_lines.ReNumber(sel_map_points, (all_tin_points.end()-1)->Number()+1, (all_tin_lines.end()-1)->Number()+1);
         all_tin_lines.insert(all_tin_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
         all_tin_points.insert(all_tin_points.end(), sel_map_points.begin(), sel_map_points.end());
     }
  if (index == (index/5)*5)
      printf("%6d (%5.1f\%)\r", index, 100.0 * index / map_lines.size());
  }
  printf("\n(done)\n Matching reference points with TIN patches...\n");
  } 
  
  if (triangulated){
     all_tin_lines = map_lines;
     all_tin_points = map_points;
  }
                          
  last_map_line = all_tin_lines.begin();
  countin = countout = 0;
  double range = 0.1;
  double r = 0.09; //standard deviation of reference data;
  double qr;
  int foundcount;
  for (map_point = map_points_ref.begin(), index=0; map_point!=map_points_ref.end(); map_point++, index++){
      ref_laser_point.SetPointNumber(map_point->Number());
      ref_laser_point.X() = map_point->X();
      ref_laser_point.Y() = map_point->Y();
      ref_laser_point.Z() = map_point->Z();
      found = false;
      diff = 3.0;
      
      predicted_points.ErasePoints();
        for (map_line=all_tin_lines.begin(), found = false;
             map_line!=all_tin_lines.end() && !found; map_line++) {
          if (ref_laser_point.InsidePolygon(all_tin_points,
                                         map_line->LineTopologyReference())) {
             nb_node=map_line->begin();
             p1 = Position3D(*(all_tin_points.PointIterator(*nb_node))); nb_node++;
             p2 = Position3D(*(all_tin_points.PointIterator(*nb_node))); nb_node++;
             p3 = Position3D(*(all_tin_points.PointIterator(*nb_node)));
             plane = Plane(p1, p2, p3);
             diff = plane.Z_At(ref_laser_point.X(), ref_laser_point.Y(), &success)
                      -ref_laser_point.Z();
             
 //          for (map_line3 = orig_lines.begin(); map_line3!=orig_lines.end(); map_line3++){
 //                 if (ref_laser_point.InsidePolygon(orig_points,
 //                                        map_line3->LineTopologyReference())) {
 //                  predicted_points.AddTaggedPoints(laser_points, map_line3->Number(), PolygonNumberTag);
 //                  }
 //                  }
//           predicted_points = laser_points.SelectTagValue(PolygonNumberTag, map_line->Number()); 
 //          printf("Size of pred_points: %d\n", predicted_points.size());          
           if (fabs(diff) < 2) {
  //             printf(" %5.2f ", diff); 
               found = true;
               ref_laser_point.Residual()=diff;      //in PCM if residual<0.2 green, <0.5 yellow, else red                                                                  
               last_map_line = map_line;
               foundcount = 0;
             q1=p1, q2=p2, q3=p3;
             q1.SetZ(0.15),q2.SetZ(0.15),q3.SetZ(0.15);
             for (laser_point = laser_points.begin(); laser_point!=laser_points.end(); laser_point++){
                 if (fabs(laser_point->X() - p1.GetX())<range){
                   if (fabs(laser_point->Y() - p1.GetY())<range){
                      if (fabs(laser_point->Z() - p1.GetZ())<range){
                         q1.SetZ(laser_point->Residual());                       
            //           foundcount++;                                                  
            //           printf("laserpoint found (%d) with z-diff %5.2f\n", foundcount,fabs(laser_point->Z() - p1.GetZ()));
                       }
                    }
                 }
                 if (fabs(laser_point->X() - p2.GetX())<range){
                   if (fabs(laser_point->Y() - p2.GetY())<range){
                      if (fabs(laser_point->Z() - p2.GetZ())<range){
                         q2.SetZ(laser_point->Residual());                       
            //           foundcount++;                                                  
            //           printf("laserpoint found (%d) with z-diff %5.2f\n", foundcount,fabs(laser_point->Z() - p1.GetZ()));
                       }
                    }
                 }
                 if (fabs(laser_point->X() - p3.GetX())<range){
                   if (fabs(laser_point->Y() - p3.GetY())<range){
                      if (fabs(laser_point->Z() - p3.GetZ())<range){
                         q3.SetZ(laser_point->Residual());                       
            //           foundcount++;                                                  
            //           printf("laserpoint found (%d) with z-diff %5.2f\n", foundcount,fabs(laser_point->Z() - p1.GetZ()));
                       }
                    }
                 }
           }
              qplane = Plane(q1, q2, q3);
              q = qplane.Z_At(ref_laser_point.X(), ref_laser_point.Y(), &success);
              qr = sqrt(q*q + r*r);
              printf("Calculated qr = %5.2f, q = %5.2f, diff = %5.2f, w-test %5.2f)\n", qr, q, diff, diff/qr);
              }
          }
        }
   //   }
      if (found) {
             //    ref_laser_point.Label(last_map_line->Attribute(LineLabelTag)); 
                 countin++; 
                 laserin.push_back(ref_laser_point);    
                 
                 // check if there is a relation between residual and number of 'near' laser points
  /*               nearest_laser_point = laser_points.NearestPoint(map_point->Position3DRef(),
                                                    edges, true);
                 neighbourhood = 
                 laser_points.TaggedNeighbourhood(PointNumber(nearest_laser_point),
                                             last_map_line->Attribute(LineLabelTag),
                                             nbh_radius, edges,
                                             PolygonNumberTag,true, false);                           
    */                    
   //         if (neighbourhood.size()>0) ref_laser_point.Residual() = 0.5/sqrt(neighbourhood.size());     
   //                else ref_laser_point.Residual() = 0.8;
            //     ref_laser_point.Z() = diff/q; // for histogram analysis, put diff as z-value, store in laserout                 
  //               ref_laser_point.Residual() = lasercount; //relate lasercount with residual
      //           ref_laser_point.Z() = ref_laser_point.Residual();
                 
                 ref_laser_point.Residual() = 0;
                 if(fabs(diff/qr)>3) ref_laser_point.Residual() = 0.4;
                 if(fabs(diff/qr)>4) ref_laser_point.Residual() = 1.0;
                 ref_laser_point.Z() = diff/qr;//for histogram analysis, put diff as z-value, store in laserout   
                 laserout.push_back(ref_laser_point);
      }
      else {           
           countout++; 
  //         ref_laser_point.RemoveAttribute(LabelTag); // no label for reference points outside 3d reconstructed model
  //         ref_laser_point.Residual()= diff;//1.0;//Set residual on 3.0 meter -> will show in red in PCM
      //     laserin.push_back(ref_laser_point);
  //         laserout.push_back(ref_laser_point); //also put in seperate file, for further analysis
           }
 //     if (index == (index/5)*5)
  //    printf("%6d (%5.1f\%)\r", index, 100.0 * index / map_points_ref.size());
  }
  printf("\nIn: %d, Out: %d\n", countin, countout);
  if (!laserin.Write(laser_points_inside, false))
        printf("Error writing reference points INSIDE as laser points.\n");
  if (!laserout.Write(laser_points_outside, false))
        printf("Error writing reference points OUTSIDE as laser points.\n");

  return;
}

