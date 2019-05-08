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
 Date   : 18-07-2013

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
#include "VectorPoint.h"
#include "VRML_io.h"
#include "dxf.h"
#include "TIN.h"
#include "Building.h"
#include "Buildings.h"
#include "stdmath.h"
#include "triangle.h"
#include "Database.h"


/*
--------------------------------------------------------------------------------
                           The main function
--------------------------------------------------------------------------------
*/

void createindoordatagraphs(char *laser_input, 
           char *map_points_output, char *map_topology_output, 
           int minsizesegment, double minlinelength)
{
  LaserPoints                  laser_points, sel_laser_points, seg_laser_points,
                               outputpoints;
  TINEdges                     *edges, edges_3d, edges1, edges2;
  ObjectPoints                 graph_points, origintsectobjpts, shape_points,
                               pairpoints;
  LineTopologies               graph_lines, origintsectlines, shapelines, pairlines;
  LineTopologies::iterator     map_line;
  ObjectPoints::iterator       map_point, previous_point, next_point, this_point;
  ObjectPoint                  new_map_point, new_point, beginp, endp,
                               objpseg1, objpseg2, height_point, max_point,
                               cornerpoint;

  LineTopology::iterator       node2;
  LineTopology                 contour, intsect, height_line, pairline, shapeline;
  vector <int>                 segment_numbers;
  vector <int>::iterator       segment_number;
  PointNumberLists             segments, pnls;
  PointNumberList              component, pnl, pnlseg1, pnlseg2, tpnl, 
                               neighbourhood1, neighbourhood2, step, highestseg;
  PointNumberList::iterator    item;
  PointNumberLists::iterator   segment, segment2, pnlit, pnlit2;        
  TIN                          tin;
  TINEdges::iterator           neighbours;         
  Planes                       planes, cornerplanes;
  Plane                        planecorner1, planecorner2, planecorner3, plane, planev, planeh, planel;
  Planes::iterator             plane1, plane2;
  Position3D                   pos1, pos2, posseg1, posseg2, posmin, posmax,
                               runpos, laspos, newpos, pos3, pos4, pos5, midpos;
  Position2D                   centroid;
  PointNumber                  pn1, pn2;
  Covariance3D                 cov3d;
  Vector3D                     zenithvec, direction, outervec, dominant_outervec,
                               perp_dom_outervec;
  FILE                         *dxffile, *statfile, *graph_file;
  double                       PI, flat_angle, dist, vertical_angle;
  int                          count, sn1, sn2, lb1, lb2, countingintsect=0, line_number3=0,
                               labwallceiling, labwallfloor, labwallwall, takelabel, count1,
                               countwallwall, countwallceiling, countwallfloor;
  bool                         b1, b2;
  Line3D           intline;

  cov3d = Covariance3D(0,0,0,0,0,0);
  PI = 3.14159;
  zenithvec = Vector3D(0, 0, 1);
  flat_angle = 10 ;
  vertical_angle = 10;
  labwallceiling = 1;
  labwallfloor = 2;
  labwallwall = 3;
  
if (!laser_points.Read(laser_input)) {
    printf("Error reading laser points from file %s\n", laser_input);
    exit(0);
  }
printf("Processing segments larger than %d points.\n", minsizesegment);
 statfile = fopen("graphinfo.txt","w");

//if (splitstrangeshapedsegments){
 printf("Splitting strange shaped segments... & removing small (%d points) segments\n", minsizesegment);        

 LaserPoints relabeled_laser_points, seg2_laser_points;
 int new_segment_number = 0, count2;
 int dominant_segment2, dominant_segment;
  SegmentationParameters *segpar;
  segpar=new SegmentationParameters();

      
 if (laser_points.size()>minsizesegment){
       do {
        dominant_segment = laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
        seg_laser_points.ErasePoints();
        seg_laser_points.AddTaggedPoints(laser_points, dominant_segment, SegmentNumberTag);
        printf("%10d %10d\r", laser_points.size(), seg_laser_points.size());     
  
        edges = seg_laser_points.DeriveEdges(*segpar);
        segpar->MaxDistanceInComponent() = 0.2;  
        seg_laser_points.RemoveLongEdges(edges->TINEdgesRef(), 
                       segpar->MaxDistanceInComponent(),
                       segpar->DistanceMetricDimension() == 2);
     
        seg_laser_points.LabelComponents(edges->TINEdgesRef(), SegmentNumberTag);
      
          do {
            dominant_segment2 = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count2);
            if (count2>minsizesegment) {new_segment_number++;
             seg2_laser_points.ErasePoints();
             seg2_laser_points.AddTaggedPoints(seg_laser_points, dominant_segment2, SegmentNumberTag);
             seg2_laser_points.SetAttribute(SegmentNumberTag, new_segment_number);
             relabeled_laser_points.AddPoints(seg2_laser_points);
             seg_laser_points.RemoveTaggedPoints(dominant_segment2, SegmentNumberTag);
             }
            } while (count2>minsizesegment);
        laser_points.RemoveTaggedPoints(dominant_segment, SegmentNumberTag);
        dominant_segment = laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
        } while (count>minsizesegment );
  }
  relabeled_laser_points.Write("relabeledpoints.laser", false);
 
// return;
 laser_points = relabeled_laser_points;
 relabeled_laser_points.ErasePoints();
//}  
segment_numbers = laser_points.AttributeValues(SegmentNumberTag);
printf("# of laser segments: %d\n", segment_numbers.size());
laser_points.SetAttribute(LabelTag, 0);
  for (segment_number = segment_numbers.begin(); segment_number!=segment_numbers.end(); segment_number++){
    seg_laser_points.ErasePoints();
    seg_laser_points = laser_points.SelectTagValue(SegmentNumberTag, *segment_number);
    if (seg_laser_points.size()>minsizesegment){
      plane = laser_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
      if (plane.IsHorizontal(flat_angle*PI/180)) laser_points.ConditionalReTag(0,1,LabelTag, *segment_number, SegmentNumberTag); 
      if (plane.IsVertical(vertical_angle*PI/180)) laser_points.ConditionalReTag(0,2,LabelTag, *segment_number, SegmentNumberTag); 

      pnl.erase(pnl.begin(),pnl.end());
      pnl = laser_points.TaggedPointNumberList(SegmentNumberTag, *segment_number);
      pnls.push_back(pnl);
        
      planes.push_back(plane);
      }
   }
   
printf("of which %d are larger than %d\n", planes.size(), minsizesegment);
   
      for (pnlit=pnls.begin(), plane1=planes.begin();
                    pnlit!=pnls.end(); pnlit++, plane1++) {
                   sn1 = (laser_points[pnlit->begin()->Number()]).Attribute(SegmentNumberTag);
                   lb1 = (laser_points[pnlit->begin()->Number()]).Attribute(LabelTag);
         
                for (pnlit2=pnlit+1, plane2=plane1+1;
                       pnlit2!=pnls.end(); pnlit2++, plane2++) {
                   sn2 = (laser_points[pnlit2->begin()->Number()]).Attribute(SegmentNumberTag);
                   lb2 = (laser_points[pnlit2->begin()->Number()]).Attribute(LabelTag);
     
                   printf("\nanalysing %d and %d: ", sn1, sn2);

                   if (Intersect2Planes(*plane1, *plane2, intline)){
                    b1 = (*plane1).IsHorizontal(flat_angle*PI/180);
                    b2 = (*plane2).IsHorizontal(flat_angle*PI/180);                            
                   printf(" planes intersect ");
                    
                   if (laser_points.IntersectFaces(*pnlit, *pnlit2, *plane1, *plane2, 0.1, pos1, pos2)){
                        printf(" faces intersect ");
                        countingintsect++;
                        pn1 = PointNumber(countingintsect);                        
                        countingintsect++;
                        beginp = ObjectPoint(pos1, pn1, cov3d);
                        pn2 = PointNumber(countingintsect);
                        endp = ObjectPoint(pos2, pn2, cov3d);
                        intsect = LineTopology(line_number3, 1, pn1, pn2);
                        direction = intline.Direction();
                        if (fabs(direction[2])<flat_angle*PI/180) intsect.Label() = 6; //horizontal, label 6
                        else {
                             intsect.Label() = 3; 
                             }
                        if (b1 && b2) { //line created by intersection of 2 flat surfaces... not reliable..do nothing
                               intsect.Label() = 4;
                               } 
                        else {
                        dist = pos1.Distance(pos2);
                        
                        if (dist>minlinelength){ //for the moment; only look at intersections longer than minlinelength
                          
                          objpseg1 = laser_points.Centroid(*pnlit, sn1);
                          objpseg2 = laser_points.Centroid(*pnlit2, sn2);
   //                       pairpoints.push_back(objpseg1);
   //                       pairpoints.push_back(objpseg2);
                          if (lb1==2 && lb2==2) takelabel = labwallwall;
                          if (lb1==2 && lb2==1 && objpseg2.Z()>objpseg1.Z()) takelabel = labwallceiling;
                          if (lb1==2 && lb2==1 && objpseg1.Z()>objpseg2.Z()) takelabel = labwallfloor;
                          if (lb1==1 && lb2==2 && objpseg2.Z()>objpseg1.Z()) takelabel = labwallfloor;
                          if (lb1==1 && lb2==2 && objpseg1.Z()>objpseg2.Z()) takelabel = labwallceiling;
                                 
                          pairline = LineTopology(line_number3, 1, sn1, sn2);
                          pairline.Label() = takelabel;
                          pairlines.push_back(pairline);
                          line_number3++;
                          shapeline = intsect;
                          shapeline.Label() = takelabel;
                          shape_points.push_back(beginp);
                          shape_points.push_back(endp);
                          shapelines.push_back(shapeline);
                          fprintf(statfile, "segment %4d links to segment %4d, with length %5.2f, and edge label %d\n", sn1, sn2, dist, takelabel);
                          }
                        }
                        }
                    }
     //            }
                 }
                 objpseg1 = laser_points.Centroid(*pnlit, sn1);
                 pairpoints.push_back(objpseg1);
              }
    pairpoints.RemoveDoublePoints(pairlines, 0.01);
    fprintf(statfile, "\nprint statistics..\n");
    for (map_point = pairpoints.begin(); map_point!=pairpoints.end(); map_point++){
     count=0;
     countwallwall =0;
     countwallceiling =0;
     countwallfloor =0;
     for (map_line=pairlines.begin(); map_line!=pairlines.end(); map_line++) {
      if (map_line->Contains(map_point->NumberRef())){
       count++;
      if (map_line->Attribute(LineLabelTag)==labwallceiling) countwallceiling++;
      if (map_line->Attribute(LineLabelTag)==labwallwall) countwallwall++;
      if (map_line->Attribute(LineLabelTag)==labwallfloor) countwallfloor++;
      }
     }
     seg_laser_points.ErasePoints();
     seg_laser_points = laser_points.SelectTagValue(SegmentNumberTag, map_point->Number());

     if (count==0){
         seg_laser_points.SetAttribute(LabelTag, 0);  
     }
    takelabel = seg_laser_points.MostFrequentAttributeValue(LabelTag, count1);
    if (takelabel==2) fprintf(statfile, "segment %d (%d points) is vertical, has %d connections.\n", map_point->Number(), seg_laser_points.size(), count); 
    if (takelabel==1) fprintf(statfile, "segment %d (%d points) is horizontal, has %d connections.\n", map_point->Number(), seg_laser_points.size(), count); 
    if (takelabel==0) fprintf(statfile, "segment %d (%d points) is not connected.\n", map_point->Number(), seg_laser_points.size()); 
    
    fprintf(statfile, "of which %d wallwall, %d wallceiling and %d wallfloor connections.\n", countwallwall, countwallceiling, countwallfloor);
    if (countwallceiling == 1 && countwallfloor ==1) seg_laser_points.SetAttribute(LabelTag, 4);
    if (countwallfloor > 2 && countwallwall==0) seg_laser_points.SetAttribute(LabelTag, 5);
    if (countwallceiling > 2 && countwallwall==0) seg_laser_points.SetAttribute(LabelTag, 6);
  
        outputpoints.AddPoints(seg_laser_points);

        }
    
    bool calculatesegmentinfo = true;
    if (calculatesegmentinfo){
    double x5, x95, z5, z95, y5, y95, h, p;
    laser_points = outputpoints;
    segment_numbers = laser_points.AttributeValues(SegmentNumberTag);
    printf("# of laser segments: %d\n", segment_numbers.size());
    for (segment_number = segment_numbers.begin(); segment_number!=segment_numbers.end(); segment_number++){
      seg_laser_points.ErasePoints();
      seg_laser_points = laser_points.SelectTagValue(SegmentNumberTag, *segment_number);
      dominant_segment = seg_laser_points.MostFrequentAttributeValue(LabelTag, count);
      if (dominant_segment == 4 || dominant_segment == 2){
        z5 = seg_laser_points.ReturnHeightOfPercentilePoint(2);
        z95 = seg_laser_points.ReturnHeightOfPercentilePoint(98);
        seg_laser_points.SwapXZ();
        x5 = seg_laser_points.ReturnHeightOfPercentilePoint(2);
        x95 = seg_laser_points.ReturnHeightOfPercentilePoint(98);
        seg_laser_points.SwapXZ();
        seg_laser_points.SwapYZ();
        y5 = seg_laser_points.ReturnHeightOfPercentilePoint(2);
        y95 = seg_laser_points.ReturnHeightOfPercentilePoint(98);
        seg_laser_points.SwapYZ();
        p = sqrt((x95-x5)*(x95-x5)+(y95-y5)*(y95-y5));
        h = z95-z5;
        if (p>0.7 && p<1 && h>1.8 && h<2.5) {
         laser_points.RemoveTaggedPoints(*segment_number, SegmentNumberTag);
         seg_laser_points.SetAttribute(LabelTag, 3);
         laser_points.AddPoints(seg_laser_points);
        }  
        fprintf(statfile, "segment %d: width %5.2f, and height %4.2f.\n", *segment_number, p, h);
      }  
      }      
      outputpoints = laser_points;
          
                              
    } 
    pairpoints.Write(map_points_output);
    pairlines.Write(map_topology_output, false);
    fclose(statfile); 
    
    shape_points.Write("origintsectlines.objpts");
    shapelines.Write("origintsectlines.top", false);
    outputpoints.Write("outputlaser.laser", false);
    return;
    }
