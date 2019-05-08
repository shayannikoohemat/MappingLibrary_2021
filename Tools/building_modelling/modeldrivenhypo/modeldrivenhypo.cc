
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

----------------------------------------------------------------------------*//*
--------------------------------------------------------------------------------

 Initial creation:
 Author : Sander Oude Elberink
 Date   : 6-5-09

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/


#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <Matrix3.h> 
#include "LaserPoints.h"
#include "LineTopsIterVector.h"
#include <LineSegment2D.h> 



/*
--------------------------------------------------------------------------------
                           The main function
--------------------------------------------------------------------------------
*/

void modeldrivenhypo(char *map_points_input, char *map_topology_input,
                     char *matchresults_topology_input, char *laser_input, 
                    char *map_points_output, char *map_topology_output, 
                    char *infofile, double dist2faces)

{
  double                       len, alfa, h0, h1, h2, d1, d2, d, p1x, p2x, p3x, p4x, p5x, p6x,
                               p1y, p2y, p3y, p4y, p5y, p6y, p1z, p2z, p3z, p4z, p5z, p6z, PI,
                               p7x,p7y,p7z,p8x,p8y,p8z,p9x,p9y,p9z,p10x,p10y,p10z,
                               floor_height, od, len2, alfa2, alfadum, p0x, p0y, p0z, dist, maxdist, 
                               first_floor_height, diff2gutter, thres1, thres2, mindist, maxheight, height,
                               keepresidual, residual, d_assym1, d_assym2, inclination,
                               snap_distance, orig_ridgelength, horlength1, horlength2, mean_height,
                               meanx, meany, meanz, meanx2, meany2, meanz2;
  Position3D                   pos1, pos2, pos3, pos4, pos5, pos6, pos, dumpos, 
                               pos12, pos22, dumpos2, pos42, pos9, pos61, pos62,
                               pos91, pos92, pos14, pos23, pos13, pos24, dumpos1,
                               keepdumpos, projpos, posassym, lowestpointassym,
                               projposonmap1;
  Position2D                   dumpos2d, pos12d, pos22d, nbmppos12d, nbmppos22d;
  LineTopology                 top, highesthypo;
  LineTopologies               tops, map_lines, matchresults, sel_matchresults, sel_matchresults2,
                               all_wall_lines, wall_map_lines, one_map_line, sellines1, nexttops, keeptops,
                               sellines2, first_floor_lines, sel_mapresults, shapetops;
  ObjectPoint                  point, cornerpoint, objpseg1, centrepoint, nbmpoint1, nbmpoint2;
  ObjectPoints                 points, map_points, sel_map_points, all_wall_points, nextpoints, keeppoints,
                               first_floor_points, ext_map_points;
  ObjectPoints::iterator       this_point, map_point;
  int                          highestcoverage, target, seg1, seg2, seg3, seg4,
                               pn, p0, p1, p2, p3, p4, p5, p6, i, p7,p8,p9,p10,
                               number_offset, hyponr, index, success, supseg, dormerseg, segnr, count,
                               nearestpoint, flatseg, nr1, line_number3, next_pnr;
  vector <int>                 segment_numbers, hyponrs, ridgenumbers;
  vector <int>::iterator       hypo, segment_number;
  bool                         gable, gambrel, hip, halfhip, mansard, flat,
                               l_shape, pyramid, segmentsnotused, overhang, hyponotused, dormer, changed, add_first_floor,
                               connectiontarget17, connectiontarget18, step, convexmansard, concavemansard,
                               connectiontarget19, firstfound, found, assym, extend_to_map, 
                               extend_found1, extend_found2, inside1, inside2, include_quality;
  LineTopologies::iterator     map_line, sel_match_line, selline, selline2, face;
  LineTopology::iterator       node, node2, next_node;
  LaserPoint                   lowestlaserpoint, lowestlaserpoint_secondlevel, 
                               centroidofseg, maplaserpoint, lowestlaserpoint2,
                               assymlowestlaserpoint, laspos1, laspos2;
  LaserPoints                  laser_points, sel_laser_points, partlaserpoints,
                               processedlaserpoints, seg_laser_points, partlaserpoints2, maplaserpoints,
                               assymlaserpoints, ext_to_maplaserpoints, flatroofpoints;
  LaserPoints::iterator        laser_point;
  Plane                        plane1, plane2, plane3, plane4, supplane, plane;
  Line3D                       line, linecorner, line2, dumline, line3d;
  Line2D                       line2d, nbmline;
  PointNumberList              pnl1, pnl2, pnl3, pnl4, pnl;
  TINEdges                     edges;
  FILE                         *statfile, *dxffile;
  statfile = fopen(infofile,"w");  

    PI = 3.14159;
    od = 0.2;
    overhang = false;//false;//true; //to be made local
    thres1 = 0.5;
    thres1 = 1.0;
    thres2 = 1.5;
    assym = false;
    extend_to_map = false;
    snap_distance = 0.5;
    include_quality = true;
    
    
  // Read input data
  
    if (!map_points.Read(map_points_input)) {
    printf("Error reading map points from file %s\n", map_points_input);
    exit(0);
  }
  if (!map_lines.Read(map_topology_input)) {
    printf("Error reading map lines from file %s\n", map_topology_input);
    exit(0);
  }
  if (!matchresults.Read(matchresults_topology_input)) {
    printf("Error reading map lines from file %s\n", matchresults_topology_input);
    exit(0);
  }
  laser_points.Read(laser_input);
  
  pn = 0;
   printf("\nstart building 3D models from match results.\nReconstruct by target models.\n");
   for (map_line = map_lines.begin(), index=0; map_line!=map_lines.end(); map_line++, index++){
   //      printf("%6d (%5.1f\%)\r", index, 100.0 * index / map_lines.size());
         printf("\n%6d ", map_line->Number());
         sel_matchresults = matchresults.SelectAttributedLines(BuildingNumberTag, map_line->Number());
         sel_laser_points.ErasePoints();
         sel_laser_points = laser_points.SelectTagValue(PolygonNumberTag, map_line->Number());
         sel_matchresults2 = sel_matchresults.SelectAttributedLines(MatchResultTag, 0);
    //     printf("\n%6d and match results %d", sel_laser_points.size(), sel_matchresults2.size());
    //     }
    //     return;
    //     {
         processedlaserpoints.ErasePoints();
         hyponrs.erase(hyponrs.begin(), hyponrs.end());
         add_first_floor = true;
         floor_height = 0.01*(map_line->Attribute(PredictedHeight)-1000);
         first_floor_height = floor_height + 2.85;
  //     first_floor_height = floor_height + 15; //for city centre
  //       add_first_floor = false;
   //      if (!add_first_floor) first_floor_height = floor_height;
         do {
         target = 0;
         changed = false;
          gable= false; gambrel= false; hip= false; halfhip= false; mansard = false; 
          flat = false, l_shape = false, pyramid = false, dormer = false, connectiontarget17 = false, step = false,
          convexmansard = false, connectiontarget18 = false, connectiontarget19 = false;
          if (sel_matchresults2.size() == 0 && sel_laser_points.size()>0){ //check if flat roof
              flat = true;
              add_first_floor = false;
              }
          for (sel_match_line = sel_matchresults2.begin(); sel_match_line !=sel_matchresults2.end(); sel_match_line++){
             hyponotused = true;
             for (hypo = hyponrs.begin(); hypo!=hyponrs.end(); hypo++){
                 if (sel_match_line->Number()== (*hypo)) hyponotused = false;
                 }
             if (sel_match_line->Attribute(TargetNumberTag)==10|| //first check if shape targets...
                 sel_match_line->Attribute(TargetNumberTag)==20||
                 sel_match_line->Attribute(TargetNumberTag)==21){
                 if (hyponotused){
                   highestcoverage = sel_match_line->Attribute(CoverageTag);
                   highesthypo = *sel_match_line;
                   target = highesthypo.Attribute(TargetNumberTag);
                   hyponr = sel_match_line->Number();
                   changed =true;
                 }
             }
          }
          if (!changed){
          for (sel_match_line = sel_matchresults2.begin(); sel_match_line !=sel_matchresults2.end(); sel_match_line++){
             hyponotused = true;
             for (hypo = hyponrs.begin(); hypo!=hyponrs.end(); hypo++){
                 if (sel_match_line->Number()== (*hypo)) hyponotused = false;
                 }
                 if (sel_match_line->Attribute(TargetNumberTag)==11){
                 if (hyponotused){
                   highestcoverage = sel_match_line->Attribute(CoverageTag);
                   highesthypo = *sel_match_line;
                   target = highesthypo.Attribute(TargetNumberTag);
                   hyponr = sel_match_line->Number();
                   changed =true;
                 }
             }
          }
          }
          if (!changed){
          for (sel_match_line = sel_matchresults2.begin(); sel_match_line !=sel_matchresults2.end(); sel_match_line++){
             hyponotused = true;
             for (hypo = hyponrs.begin(); hypo!=hyponrs.end(); hypo++){
                 if (sel_match_line->Number()== (*hypo)) hyponotused = false;
                 }
                 if(sel_match_line->Attribute(TargetNumberTag)==14){
                 if (hyponotused){
                   highestcoverage = sel_match_line->Attribute(CoverageTag);
                   highesthypo = *sel_match_line;
                   target = highesthypo.Attribute(TargetNumberTag);
                   hyponr = sel_match_line->Number();
                   changed =true;
                 }
             }
          }
          }
          if (!changed){ // no shape targets anymore in sel_matchresults2, look for connections targets
          for (sel_match_line = sel_matchresults2.begin(); sel_match_line !=sel_matchresults2.end(); sel_match_line++){
             hyponotused = true;
             for (hypo = hyponrs.begin(); hypo!=hyponrs.end(); hypo++){
                 if (sel_match_line->Number()== (*hypo)) hyponotused = false;
                 }
             if (hyponotused){
                   highestcoverage = sel_match_line->Attribute(CoverageTag);
                   highesthypo = *sel_match_line;
                   target = highesthypo.Attribute(TargetNumberTag);
                   hyponr = sel_match_line->Number();
                   changed =true;
                }
             }
          }
        
         hyponrs.push_back(hyponr);
         if (target == 10) gable = true;
         if (target == 11) halfhip = true;
         if (target == 12) pyramid = true;
         if (target == 13) step = true;
         if (target == 14) hip = true;
         if (target == 16) l_shape = true;
//         if (target == 17 || target == 27) connectiontarget17 = true;
         if (target == 17) connectiontarget17 = true;
         if (target == 18) connectiontarget18 = true;
         if (target == 19) connectiontarget19 = true;
         if (target == 20) gambrel = true;
         if (target == 21) mansard = true;
 //        if (target == 22) dormer = true;
 //        if (target == 23) convexmansard = true;
         
         
         printf("building %d, main shape is %d\n", map_line->Number(), target);
         
         if (gable){
             
//             pos    -- mid point of intersection line
//             len    -- length from mid point to end point (=0.5 line length)
//             alfa   -- 2d direction of intersection line
//             h0     -- mid point height
//             h1     -- lowest point of segments
//             d      -- perp dist of lowest point to line
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             partlaserpoints.ErasePoints();
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg1, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
     //        sel_laser_points.IntersectFaces(pnl1, pnl2, plane1, plane2, 0.5, pos1, pos2);
             sel_laser_points.IntersectFaces(pnl1, pnl2, plane1, plane2, dist2faces, pos1, pos2);
             partlaserpoints.SwapXZ(); partlaserpoints.SortOnCoordinates();
             partlaserpoints.SwapXZ();
             lowestlaserpoint = partlaserpoints[0];
             line = Line3D(pos1, pos2);
             pos = Position3D((pos1.GetX()+pos2.GetX())/2, (pos1.GetY()+pos2.GetY())/2, (pos1.GetZ()+pos2.GetZ())/2);
             len = 0.5*(pos1.Distance(pos2));
             alfa = atan((pos2.GetX()-pos1.GetX())/(pos2.GetY()-pos1.GetY()));
             h0 = pos.GetZ();
             h1 = lowestlaserpoint.Z();
             dumpos = Position3D(lowestlaserpoint.X(), lowestlaserpoint.Y(), h0);
             d = line.DistanceToPoint(dumpos);
             p1x = pos.GetX() - len*sin(alfa);
             p1y = pos.GetY() - len*cos(alfa);
             p1z = h0;
             point = ObjectPoint(p1x,p1y,p1z, pn, 0,0,0,0,0,0);
             p1 = pn;
             points.push_back(point);
             pn++;
             p2x = pos.GetX() + len*sin(alfa);
             p2y = pos.GetY() + len*cos(alfa);
             p2z = h0;
             point = ObjectPoint(p2x,p2y,p2z, pn, 0,0,0,0,0,0);
             p2 = pn;
             points.push_back(point);
             ridgenumbers.push_back(p1);
             ridgenumbers.push_back(p2);
             pn++;
             p3x = pos.GetX() - len*sin(alfa) +d*cos(alfa);
             p3y = pos.GetY() - len*cos(alfa) -d*sin(alfa);
             p3z = h1;
             point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
             p3 = pn;
             points.push_back(point);
             if (overhang){
                          p3x = pos.GetX() - (len-od)*sin(alfa) +(d-od)*cos(alfa);
                          p3y = pos.GetY() - (len-od)*cos(alfa) -(d-od)*sin(alfa);
                          point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);                           
                          }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p4x = pos.GetX() + len*sin(alfa) +d*cos(alfa);
             p4y = pos.GetY() + len*cos(alfa) -d*sin(alfa);
             p4z = h1;
             point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
             p4 = pn;
             points.push_back(point);
             if (overhang){
                           p4x = pos.GetX() + (len-od)*sin(alfa) +(d-od)*cos(alfa);
                           p4y = pos.GetY() + (len-od)*cos(alfa) -(d-od)*sin(alfa);
                           point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p5x = pos.GetX() - len*sin(alfa) - d*cos(alfa);
             p5y = pos.GetY() - len*cos(alfa) + d*sin(alfa);
             p5z = h1;
             point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
             p5 = pn;
             points.push_back(point);
             if (overhang){
                           p5x = pos.GetX() - (len-od)*sin(alfa) -(d-od)*cos(alfa);
                           p5y = pos.GetY() - (len-od)*cos(alfa) +(d-od)*sin(alfa);
                           point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             
             pn++;
             p6x = pos.GetX() + len*sin(alfa) -d*cos(alfa);
             p6y = pos.GetY() + len*cos(alfa) +d*sin(alfa);
             p6z = h1;
             point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
             p6 = pn;
             points.push_back(point);
             if (overhang){
                           p6x = pos.GetX() + (len-od)*sin(alfa) -(d-od)*cos(alfa);
                           p6y = pos.GetY() + (len-od)*cos(alfa) +(d-od)*sin(alfa);
                           point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             tops.push_back(top);
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             nexttops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
 //            for (segment_number = segment_numbers.begin(); segment_number!=segment_numbers.end(); segment_number++){
 //                pnl = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, *segment_number);
 //                objpseg1 = sel_laser_points.Centroid(pnl, pn);
 //                centroidofseg = LaserPoint(objpseg1.X(), objpseg1.Y(), objpseg1.Z());
 //                if (centroidofseg.InsidePolygon(points, top.LineTopologyReference())){
 //                   top.SetAttribute(SegmentLabel, *segment_number);
 //                   top.SetAttribute(LineLabelTag, *segment_number);
 //                }                    
 //            }
    
 //            top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));
              seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             nexttops.push_back(top);
             processedlaserpoints.AddPoints(partlaserpoints);
             }
  if (gambrel){
//             pos    -- mid point of intersection line
//             len    -- length from mid point to end point (=0.5 line length)
//             alfa   -- 2d direction of intersection line
//             h0     -- mid point height
//             h1     -- lowest point of segment_numbers
//             d      -- perp dist of lowest point to line
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
//                 printf("looking for segment %d\n", node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             seg3 = segment_numbers[2];
             seg4 = segment_numbers[3];
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             plane3 = sel_laser_points.FitPlane(seg3, seg3, SegmentNumberTag);
             plane4 = sel_laser_points.FitPlane(seg4, seg4, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             partlaserpoints.ErasePoints();
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg1, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
             
             sel_laser_points.IntersectFaces(pnl1, pnl2, plane1, plane2, dist2faces, pos1, pos2);
             partlaserpoints.SwapXZ(); partlaserpoints.SortOnCoordinates();
             partlaserpoints.SwapXZ();
             lowestlaserpoint = partlaserpoints[0];
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg3, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg4, SegmentNumberTag);
             partlaserpoints.SwapXZ(); partlaserpoints.SortOnCoordinates();
             partlaserpoints.SwapXZ();
             lowestlaserpoint_secondlevel = partlaserpoints[0];
             
             line = Line3D(pos1, pos2);
             
             pos = Position3D((pos1.GetX()+pos2.GetX())/2, (pos1.GetY()+pos2.GetY())/2, (pos1.GetZ()+pos2.GetZ())/2);
             len = 0.5*(pos1.Distance(pos2));
             alfa = atan((pos2.GetX()-pos1.GetX())/(pos2.GetY()-pos1.GetY()));
             h0 = pos.GetZ();
             h1 = lowestlaserpoint.Z();
             h2 = lowestlaserpoint_secondlevel.Z();
             
             dumpos = Position3D(lowestlaserpoint.X(), lowestlaserpoint.Y(), h0);
             dumpos2 = Position3D(lowestlaserpoint_secondlevel.X(), lowestlaserpoint_secondlevel.Y(), h0);

             d1 = line.DistanceToPoint(dumpos);
             d2 = line.DistanceToPoint(dumpos2);
             if (d2<d1) d2 = d1+0.5;
             
             p1x = pos.GetX() - len*sin(alfa);
             p1y = pos.GetY() - len*cos(alfa);
             p1z = h0;
             point = ObjectPoint(p1x,p1y,p1z, pn, 0,0,0,0,0,0);
             p1 = pn;
             points.push_back(point);
             pn++;
             p2x = pos.GetX() + len*sin(alfa);
             p2y = pos.GetY() + len*cos(alfa);
             p2z = h0;
             point = ObjectPoint(p2x,p2y,p2z, pn, 0,0,0,0,0,0);
             p2 = pn;
             points.push_back(point);
             ridgenumbers.push_back(p1);
             ridgenumbers.push_back(p2);
             pn++;
             p3x = pos.GetX() - len*sin(alfa) +d1*cos(alfa);
             p3y = pos.GetY() - len*cos(alfa) -d1*sin(alfa);
             p3z = h1;
             point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
             p3 = pn;
             points.push_back(point);
             pn++;
             p4x = pos.GetX() + len*sin(alfa) +d1*cos(alfa);
             p4y = pos.GetY() + len*cos(alfa) -d1*sin(alfa);
             p4z = h1;
             point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
             p4 = pn;
             points.push_back(point);
             pn++;
             p5x = pos.GetX() - len*sin(alfa) -d1*cos(alfa);
             p5y = pos.GetY() - len*cos(alfa) +d1*sin(alfa);
             p5z = h1;
             point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
             p5 = pn;
             points.push_back(point);
             pn++;
             p6x = pos.GetX() + len*sin(alfa) -d1*cos(alfa);
             p6y = pos.GetY() + len*cos(alfa) +d1*sin(alfa);
             p6z = h1;
             point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
             p6 = pn;
             points.push_back(point);
             pn++;
             
             p7x = pos.GetX() - len*sin(alfa) -d2*cos(alfa);
             p7y = pos.GetY() - len*cos(alfa) +d2*sin(alfa);
             p7z = h2;
             point = ObjectPoint(p7x,p7y,p7z, pn, 0,0,0,0,0,0);
             p7 = pn;
             points.push_back(point);
             if (overhang){
                           p7x = pos.GetX() - len*sin(alfa) -(d2-od)*cos(alfa);
             p7y = pos.GetY() - len*cos(alfa) +(d2-od)*sin(alfa);
             p7z = h2;
             point = ObjectPoint(p7x,p7y,p7z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p8x = pos.GetX() + len*sin(alfa) -d2*cos(alfa);
             p8y = pos.GetY() + len*cos(alfa) +d2*sin(alfa);
             p8z = h2;
             point = ObjectPoint(p8x,p8y,p8z, pn, 0,0,0,0,0,0);
             p8 = pn;
             points.push_back(point);
             if (overhang){
                           p8x = pos.GetX() + len*sin(alfa) -(d2-od)*cos(alfa);
             p8y = pos.GetY() + len*cos(alfa) +(d2-od)*sin(alfa);
             p8z = h2;
             point = ObjectPoint(p8x,p8y,p8z, pn, 0,0,0,0,0,0);
             }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p9x = pos.GetX() - len*sin(alfa) +d2*cos(alfa);
             p9y = pos.GetY() - len*cos(alfa) -d2*sin(alfa);
             p9z = h2;
             point = ObjectPoint(p9x,p9y,p9z, pn, 0,0,0,0,0,0);
             p9 = pn;
             points.push_back(point);
             if (overhang){
                           p9x = pos.GetX() - len*sin(alfa) +(d2-od)*cos(alfa);
                           p9y = pos.GetY() - len*cos(alfa) -(d2-od)*sin(alfa);
                           point = ObjectPoint(p9x,p9y,p9z, pn, 0,0,0,0,0,0);
                }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p10x = pos.GetX() + len*sin(alfa) +d2*cos(alfa);
             p10y = pos.GetY() + len*cos(alfa) -d2*sin(alfa);
             p10z = h2;
             point = ObjectPoint(p10x,p10y,p10z, pn, 0,0,0,0,0,0);
             p10 = pn;
             points.push_back(point);
             if (overhang){
                           p10x = pos.GetX() + len*sin(alfa) +(d2-od)*cos(alfa);
                           p10y = pos.GetY() + len*cos(alfa) -(d2-od)*sin(alfa);
                           point = ObjectPoint(p10x,p10y,p10z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
        //     nexttops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p10));
             top.push_back(PointNumber(p9));
             top.push_back(PointNumber(p3));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
      //       nexttops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p7));
             top.push_back(PointNumber(p8));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p5));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             processedlaserpoints.AddPoints(partlaserpoints);
               
               
             }
  if (hip){
//             pos    -- mid point of intersection line
//             len    -- length from mid point to end points (=0.5 line length)
//             alfa   -- 2d direction of intersection line
//             h0     -- mid point height
//             h1     -- lowest point of (two major) segment_numbers
//             d      -- perp dist of lowest point to line
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
//                 printf("looking for segment %d\n", node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             seg3 = segment_numbers[2];
             seg4 = segment_numbers[3];
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             plane3 = sel_laser_points.FitPlane(seg3, seg3, SegmentNumberTag);
             plane4 = sel_laser_points.FitPlane(seg4, seg4, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             partlaserpoints.ErasePoints();
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg1, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
             
             sel_laser_points.IntersectFaces(pnl1, pnl2, plane1, plane2, dist2faces, pos1, pos2);
             partlaserpoints.SwapXZ(); partlaserpoints.SortOnCoordinates();
             partlaserpoints.SwapXZ();
             lowestlaserpoint = partlaserpoints[0];
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg3, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg4, SegmentNumberTag);
             line = Line3D(pos1, pos2);
             IntersectLine3DPlane(line, plane3, cornerpoint);
             pos12 = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z());
             IntersectLine3DPlane(line, plane4, cornerpoint);
             pos22 = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z());
              
             pos = Position3D((pos12.GetX()+pos22.GetX())/2, (pos12.GetY()+pos22.GetY())/2, (pos12.GetZ()+pos22.GetZ())/2);
             len = 0.5*(pos12.Distance(pos22));
             alfa = atan((pos22.GetX()-pos12.GetX())/(pos22.GetY()-pos12.GetY()));
             h0 = pos.GetZ();
             h1 = lowestlaserpoint.Z();
             dumpos = Position3D(lowestlaserpoint.X(), lowestlaserpoint.Y(), h0);
             d = line.DistanceToPoint(dumpos);
             p1x = pos.GetX() - len*sin(alfa);
             p1y = pos.GetY() - len*cos(alfa);
             p1z = h0;
             point = ObjectPoint(p1x,p1y,p1z, pn, 0,0,0,0,0,0);
             p1 = pn;
             points.push_back(point);
             pn++;
             p2x = pos.GetX() + len*sin(alfa);
             p2y = pos.GetY() + len*cos(alfa);
             p2z = h0;
             point = ObjectPoint(p2x,p2y,p2z, pn, 0,0,0,0,0,0);
             p2 = pn;
             points.push_back(point);
             ridgenumbers.push_back(p1);
             ridgenumbers.push_back(p2);
             dumpos = Position3D(p1x, p1y, p1z);
             for (i=0; i<segment_numbers.size();i++){
             seg1 = segment_numbers[i]; 
             sellines1 = tops.SelectAttributedLines(SegmentLabel, seg1); 
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){
                 mindist = 1000;
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-dumpos).Length()<mindist) mindist = (this_point->vect()-dumpos).Length();
                 }
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-dumpos).Length()==mindist && mindist < 1.5) this_point->vect()=dumpos;
                 }
              }
              }
             dumpos = Position3D(p2x, p2y, p2z);
             for (i=0; i<segment_numbers.size();i++){
             seg1 = segment_numbers[i]; 
             sellines1 = tops.SelectAttributedLines(SegmentLabel, seg1); 
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){
                 mindist = 1000;
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-dumpos).Length()<mindist) mindist = (this_point->vect()-dumpos).Length();
                 }
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-dumpos).Length()==mindist && mindist < 1.5) this_point->vect()=dumpos;
                 }
              }
              }
   
             pn++;
             p3x = pos.GetX() - (len+d)*sin(alfa) +d*cos(alfa);
             p3y = pos.GetY() - (len+d)*cos(alfa) -d*sin(alfa);
             p3z = h1;
             point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
             p3 = pn;
             points.push_back(point);
             if (overhang){
                          p3x = pos.GetX() - (len+d-od)*sin(alfa) +(d-od)*cos(alfa);
                          p3y = pos.GetY() - (len+d-od)*cos(alfa) -(d-od)*sin(alfa);
                          point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p4x = pos.GetX() + (len+d)*sin(alfa) +d*cos(alfa);
             p4y = pos.GetY() + (len+d)*cos(alfa) -d*sin(alfa);
             p4z = h1;
             point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
             p4 = pn;
             points.push_back(point);
             if (overhang){
                           p4x = pos.GetX() + (len+d-od)*sin(alfa) +(d-od)*cos(alfa);
                           p4y = pos.GetY() + (len+d-od)*cos(alfa) -(d-od)*sin(alfa);
                           point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p5x = pos.GetX() - (len+d)*sin(alfa) -d*cos(alfa);
             p5y = pos.GetY() - (len+d)*cos(alfa) +d*sin(alfa);
             p5z = h1;
             point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
             p5 = pn;
             points.push_back(point);
             if (overhang){
                          p5x = pos.GetX() - (len+d-od)*sin(alfa) -(d-od)*cos(alfa);
                          p5y = pos.GetY() - (len+d-od)*cos(alfa) +(d-od)*sin(alfa);
                          point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p6x = pos.GetX() + (len+d)*sin(alfa) -d*cos(alfa);
             p6y = pos.GetY() + (len+d)*cos(alfa) +d*sin(alfa);
             p6z = h1;
             point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
             p6 = pn;
             points.push_back(point);
             if (overhang){
                          p6x = pos.GetX() + (len+d-od)*sin(alfa) -(d-od)*cos(alfa);
                          p6y = pos.GetY() + (len+d-od)*cos(alfa) +(d-od)*sin(alfa);
                          point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
//             nexttops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
  //           nexttops.push_back(top);
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p2));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             processedlaserpoints.AddPoints(partlaserpoints);

             }
  if (halfhip){
//             pos    -- point of intersection 3 planes
//             len    -- length of horizontal int line
//             alfa   -- 2d direction of intersection line
//             h0     -- pos height
//             h1     -- lowest point of (two major) segment_numbers
//             d      -- perp dist of lowest point to line
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
//                 printf("looking for segment %d\n", node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             seg3 = segment_numbers[2];
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             plane3 = sel_laser_points.FitPlane(seg3, seg3, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             pnl3 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg3);
             partlaserpoints.ErasePoints();
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg1, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
             
             sel_laser_points.IntersectFaces(pnl1, pnl2, plane1, plane2, dist2faces, pos1, pos2);
             partlaserpoints.SwapXZ(); partlaserpoints.SortOnCoordinates();
             partlaserpoints.SwapXZ();
             lowestlaserpoint = partlaserpoints[0];
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg3, SegmentNumberTag);
             line = Line3D(pos1, pos2);
             IntersectLine3DPlane(line, plane3, cornerpoint);
             centrepoint = sel_laser_points.Centroid(pnl3, pn); // trick, but more reliable on small horizontal lines between pos1 and pos2
             pos = Position3D(centrepoint.X(), centrepoint.Y(), centrepoint.Z());
             if (pos.Distance(pos1)>pos.Distance(pos2)) {
                                                        pos22 = pos1;
                                                        dumpos = pos2;
                                                        }
             else {
                  pos22 = pos2;
                  dumpos = pos1;
                  }
             pos = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z()); 
             len = pos.Distance(pos22);
             line = Line3D(pos, pos22);
             alfa = atan((pos22.GetX()-pos.GetX())/(pos22.GetY()-pos.GetY()));
             if (pos.GetY()>pos22.GetY()) alfa = alfa + PI;
//             else alfa = atan((pos.GetX()-pos22.GetX())/(pos.GetY()-pos22.GetY()));
//             printf("building %d, main shape is %d\n", map_line->Number(), target);
//             printf("alfa =  %4.2f\n", alfa);
     
             for (i=0; i<segment_numbers.size();i++){
             seg1 = segment_numbers[i];
             fprintf(statfile,"\nbuilding %d, target %d, try to adjust segment %d\n", map_line->Number(), target, seg1);         
    
             sellines1 = tops.SelectAttributedLines(SegmentLabel, seg1); 
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){
                 mindist = 1000;
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-dumpos).Length()<mindist) mindist = (this_point->vect()-dumpos).Length();
                 }
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-dumpos).Length()==mindist && mindist < 0.5) {
                        this_point->vect()=pos;
                        fprintf(statfile,"\nchanged %d to %6.2f %6.2f %6.2f\n", node->Number(), pos.GetX(), pos.GetY(), pos.GetZ());
                        }
                 }
              }
              }
   
             h0 = pos.GetZ();
             h1 = lowestlaserpoint.Z();
             dumpos = Position3D(lowestlaserpoint.X(), lowestlaserpoint.Y(), h0);
             d = line.DistanceToPoint(dumpos);
             p1x = pos.GetX();// - len*sin(alfa);
             p1y = pos.GetY();// - len*cos(alfa);
             p1z = h0;
             point = ObjectPoint(p1x,p1y,p1z, pn, 0,0,0,0,0,0);
             p1 = pn;
             points.push_back(point);
             pn++;
             p2x = pos22.GetX();// + len*sin(alfa);
             p2y = pos22.GetY();// + len*cos(alfa);
             p2z = h0;
             point = ObjectPoint(p2x,p2y,p2z, pn, 0,0,0,0,0,0);
             p2 = pn;
             points.push_back(point);
             ridgenumbers.push_back(p1);
             ridgenumbers.push_back(p2);
             pn++;
             p3x = pos.GetX() -  d*sin(alfa) +d*cos(alfa);
             p3y = pos.GetY() -  d*cos(alfa) -d*sin(alfa);
             p3z = h1;
             point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
             p3 = pn;
             points.push_back(point);
             if (overhang){
                           p3x = pos.GetX() -  (d-od)*sin(alfa) +(d-od)*cos(alfa);
                           p3y = pos.GetY() -  (d-od)*cos(alfa) -(d-od)*sin(alfa);
                           point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p4x = pos22.GetX() + d*cos(alfa);
             p4y = pos22.GetY()  -d*sin(alfa);
             p4z = h1;
             point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
             p4 = pn;
             points.push_back(point);
             if (overhang){
                          p4x = pos22.GetX() + (d-od)*cos(alfa);
                          p4y = pos22.GetY()  -(d-od)*sin(alfa);
                          point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p5x = pos.GetX() - d*sin(alfa) -d*cos(alfa);
             p5y = pos.GetY() - d*cos(alfa) +d*sin(alfa);
             p5z = h1;
             point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
             p5 = pn;
             points.push_back(point);
             if (overhang){
                         p5x = pos.GetX() - (d-od)*sin(alfa) -(d-od)*cos(alfa);
                         p5y = pos.GetY() - (d-od)*cos(alfa) +(d-od)*sin(alfa);
                         point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p6x = pos22.GetX() - d*cos(alfa);
             p6y = pos22.GetY() + d*sin(alfa);
             p6z = h1;
             point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
             p6 = pn;
             points.push_back(point);
             if (overhang){
                          p6x = pos22.GetX() - (d-od)*cos(alfa);
                          p6y = pos22.GetY() + (d-od)*sin(alfa);
                          point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
  //           nexttops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
  //           nexttops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));
             top.SetAttribute(LineLabelTag, target);
             top.SetAttribute(SegmentLabel, 0);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

    
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p2));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             processedlaserpoints.AddPoints(partlaserpoints);

             }
  if (l_shape){
//             pos    -- point of intersection 3 planes
//             len    -- length of horizontal int line between 1 and 4
//             len2   -- length of second line
//             alfa   -- 2d direction of intersection line
//             alfa2
//             h0     -- pos height
//             h1     -- lowest point of (two major) segment_numbers
//             d      -- perp dist of lowest point to line
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
//                 printf("looking for segment %d\n", node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             seg3 = segment_numbers[2];
             seg4 = segment_numbers[3];
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             plane3 = sel_laser_points.FitPlane(seg3, seg3, SegmentNumberTag);
             plane4 = sel_laser_points.FitPlane(seg4, seg4, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             pnl3 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg3);
             pnl4 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg4);
             Intersect2Planes(plane1, plane3, linecorner);
             partlaserpoints.ErasePoints();
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg1, SegmentNumberTag);  
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg4, SegmentNumberTag);
             partlaserpoints.SwapXZ(); partlaserpoints.SortOnCoordinates();
             partlaserpoints.SwapXZ();
             lowestlaserpoint = partlaserpoints[0];
             //lowest point now determined first two segment_numbers.. now add other segment_numbers..
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg3, SegmentNumberTag);
             sel_laser_points.IntersectFaces(pnl1, pnl4, plane1, plane4, dist2faces, pos1, pos2);
             sel_laser_points.IntersectFaces(pnl2, pnl3, plane2, plane3, dist2faces, pos3, pos4);
             sel_laser_points.IntersectFaces(pnl1, pnl3, plane1, plane3, dist2faces, pos61, pos62);
             sel_laser_points.IntersectFaces(pnl2, pnl4, plane2, plane4, dist2faces, pos91, pos92);
             line = Line3D(pos1, pos2);
             IntersectLine3DPlane(line, plane3, cornerpoint);
             pos = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z());
             if (pos.Distance(pos1)>pos.Distance(pos2)) pos22 = pos1;
             else pos22 = pos2;

             if (pos.Distance(pos3)>pos.Distance(pos4)) pos42 = pos3;
             else pos42 = pos4;
             if (pos.Distance(pos61)>pos.Distance(pos62)) pos6 = pos61;
             else pos6 = pos62;
             if (pos.Distance(pos91)>pos.Distance(pos92)) pos9 = pos91;
             else pos9 = pos92;
             
             for (i=0; i<segment_numbers.size();i++){
             seg1 = segment_numbers[i]; 
             sellines1 = tops.SelectAttributedLines(SegmentLabel, seg1); 
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){
                 mindist = 1000;
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos).Length()<mindist) mindist = (this_point->vect()-pos).Length();
                 }
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos).Length()==mindist && mindist < 1.5) this_point->vect()=pos;
                 }
              }
              }
             len = pos.Distance(pos22);
             line = Line3D(pos, pos22);
             alfa = atan((pos22.GetX()-pos.GetX())/(pos22.GetY()-pos.GetY())); 
             if (pos.GetY()>pos22.GetY()) alfa = alfa + PI;
//             else alfa = atan((pos.GetX()-pos22.GetX())/(pos.GetY()-pos22.GetY()));
  //           printf("building %d, main shape is %d\n", map_line->Number(), target);
  //           printf("alfa =  %4.2f\n", alfa);
             
             len2 = pos.Distance(pos42);
             line2 = Line3D(pos, pos42);
             alfa2 = atan((pos.GetX()-pos42.GetX())/(pos.GetY()-pos42.GetY()));
             if (pos42.GetY()< pos.GetY()) alfa2 = alfa2 + PI;
                         
             h0 = pos.GetZ();
             h1 = lowestlaserpoint.Z();
//             pos6 = linecorner.DetPositionZ(h1);
//             pos9.X() = pos.X()+(pos.X()-pos6.X());
//             pos9.Y() = pos.Y()+(pos.Y()-pos6.Y());
//             pos9.Z() = pos6.Z();
 
             dumline = Line3D(pos, pos6);
             dumpos = dumline.DetPositionZ(h1);
             pos6 = dumpos;
             dumline = Line3D(pos, pos9);
             dumpos = dumline.DetPositionZ(h1);
             pos9 = dumpos;
             alfadum = atan((pos6.GetX()-pos9.GetX())/(pos6.GetY()-pos9.GetY()));            
             dumpos = Position3D(lowestlaserpoint.X(), lowestlaserpoint.Y(), h0);
             d = line.DistanceToPoint(dumpos);
             p1x = pos.GetX();// - len*sin(alfa);
             p1y = pos.GetY();// - len*cos(alfa);
             p1z = h0;
             point = ObjectPoint(p1x,p1y,p1z, pn, 0,0,0,0,0,0);
             p1 = pn;
             points.push_back(point);
             pn++;
             p2x = pos22.GetX();// + len*sin(alfa);
             p2y = pos22.GetY();// + len*cos(alfa);
             p2z = h0;
             point = ObjectPoint(p2x,p2y,p2z, pn, 0,0,0,0,0,0);
             p2 = pn;
             points.push_back(point);
             pn++;
             p3x = pos42.GetX();// + len*sin(alfa);
             p3y = pos42.GetY();// + len*cos(alfa);
             p3z = h0;
             point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
             p3 = pn;
             points.push_back(point);
             pn++;
             dumpos.X() =  pos22.GetX() +d*cos(alfa);
             dumpos.Y() = pos22.GetY() -d*sin(alfa);
             dumpos.Z() = h1;
             dumpos2.X() =  pos22.GetX() -d*cos(alfa);
             dumpos2.Y() = pos22.GetY() +d*sin(alfa);
             dumpos2.Z() = h1;
             if (dumpos.Distance(pos9)>dumpos2.Distance(pos9)) alfa = alfa+PI;
             
             p4x = pos22.GetX() + d*cos(alfa);
             p4y = pos22.GetY()  -d*sin(alfa);
             p4z = h1;
             point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
             p4 = pn;
             points.push_back(point);
             if (overhang){
                          p4x = pos22.GetX() + (d-od)*cos(alfa);
                          p4y = pos22.GetY()  -(d-od)*sin(alfa);
                          point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;  
             p5x = pos22.GetX() - d*cos(alfa);
             p5y = pos22.GetY()  +d*sin(alfa);
             p5z = h1;
             point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
             p5 = pn;
             points.push_back(point);
             if (overhang){
                          p5x = pos22.GetX() - (d-od)*cos(alfa);
                          p5y = pos22.GetY()  +(d-od)*sin(alfa);
                          point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;  
             p6x = pos6.GetX();// - d*cos(alfa);
             p6y = pos6.GetY();//  +d*sin(alfa);
             p6z = h1;
             point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
             p6 = pn;
             points.push_back(point);
             if (overhang){
                          p6x = pos6.GetX() - od*cos(alfadum);
                          p6y = pos6.GetY() - od*sin(alfadum);
                          point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             dumpos.X() =  pos42.GetX() +d*cos(alfa2);
             dumpos.Y() = pos42.GetY() -d*sin(alfa2);
             dumpos.Z() = h1;
             dumpos2.X() =  pos42.GetX() -d*cos(alfa2);
             dumpos2.Y() = pos42.GetY() +d*sin(alfa2);
             dumpos2.Z() = h1;
             if (dumpos.Distance(pos6)>dumpos2.Distance(pos6)) alfa2 = alfa2+PI;
             pn++;  
             p7x = pos42.GetX() + d*cos(alfa2);
             p7y = pos42.GetY()  -d*sin(alfa2);
             p7z = h1;
             point = ObjectPoint(p7x,p7y,p7z, pn, 0,0,0,0,0,0);
             p7 = pn;
             points.push_back(point);
             if (overhang){
                          p7x = pos42.GetX() + (d-od)*cos(alfa2);
                          p7y = pos42.GetY()  -(d-od)*sin(alfa2);
                          point = ObjectPoint(p7x,p7y,p7z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;  
             p8x = pos42.GetX() - d*cos(alfa2);
             p8y = pos42.GetY()  +d*sin(alfa2);
             p8z = h1;
             point = ObjectPoint(p8x,p8y,p8z, pn, 0,0,0,0,0,0);
             p8 = pn;
             points.push_back(point);
             if (overhang){
                          p8x = pos42.GetX() - (d-od)*cos(alfa2);
                          p8y = pos42.GetY()  +(d-od)*sin(alfa2);
                          point = ObjectPoint(p8x,p8y,p8z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             
             pn++;  
             p9x = pos9.GetX();// - d*cos(alfa);
             p9y = pos9.GetY();//  +d*sin(alfa);
             p9z = h1;
             point = ObjectPoint(p9x,p9y,p9z, pn, 0,0,0,0,0,0);
             p9 = pn;
             points.push_back(point);
             if (overhang){
                          p9x = pos9.GetX() + od*cos(alfadum);
                          p9y = pos9.GetY() + od*sin(alfadum);
                          point = ObjectPoint(p9x,p9y,p9z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p9));
             top.push_back(PointNumber(p1));             
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
    //         nexttops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
  //           nexttops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p8));
             top.push_back(PointNumber(p9));
             top.push_back(PointNumber(p1));             
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p3));
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
 //            nexttops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p7));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p1));             
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p3));
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
      //       nexttops.push_back(top);
               }

  if (mansard){
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());

             }
  if (connectiontarget17){
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             seg3 = segment_numbers[2]; //this is the segment where the first two should be fit on...
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             plane3 = sel_laser_points.FitPlane(seg3, seg3, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             pnl3 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg3);
             Intersect2Planes(plane1, plane2, linecorner);
             IntersectLine3DPlane(linecorner, plane3, cornerpoint);
             pos = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z());
             printf("\nbuilding %d ", map_line->Number());
             for (i=0; i<segment_numbers.size()-1;i++){ //only take seg 1 and seg 2
             seg1 = segment_numbers[i]; 
             sellines1 = tops.SelectAttributedLines(SegmentLabel, seg1);
             fprintf(statfile,"\nbuilding %d (%4.2f), target %d, try to adjust segment %d\n", map_line->Number(), 100.0 * index / map_lines.size(), target, seg1);         
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){
                 mindist = 1000;
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      height = plane3.Z_At(this_point->X(), this_point->Y(), &success);
                      fprintf(statfile,"\n node %d height %6.2f %6.2f\n", node->Number(), height, pos.GetZ());
                      if ((this_point->vect()-pos).Length()<1 && (this_point->Z()-height) < mindist) mindist = this_point->Z()-height;
                 }
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      height = plane3.Z_At(this_point->X(), this_point->Y(), &success);
                      if ((this_point->vect()-pos).Length()<1 && (this_point->Z()-height) ==mindist) {
                        this_point->vect()=pos;
                      fprintf(statfile,"\nchanged %d to %6.2f %6.2f %6.2f\n", node->Number(), pos.GetX(), pos.GetY(), pos.GetZ());
                      }
                 }
              }
              }
              printf("...done\n");
              } 
  if (connectiontarget18){

             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[2];
             seg3 = segment_numbers[1]; //seg3 =  node20, so that is why switched [1] and [2]
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             plane3 = sel_laser_points.FitPlane(seg3, seg3, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             pnl3 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg3);
             Intersect2Planes(plane1, plane2, linecorner);
             IntersectLine3DPlane(linecorner, plane3, cornerpoint);
             pos = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z());
             fprintf(statfile,"\nbuilding %d, target %d, try to adjust segment %d\n", map_line->Number(), target, seg2);
              
             sellines1 = tops.SelectAttributedLines(SegmentLabel, seg2); 
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){
                 mindist = 1000;
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos).Length()<mindist && fabs(pos.GetZ()-this_point->Z())<0.15) mindist = (this_point->vect()-pos).Length();
                 }
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos).Length()==mindist && mindist < 1 && fabs(pos.GetZ()-this_point->Z())<0.15) {
                        this_point->vect()=pos;
                        fprintf(statfile,"\nchanged %d to %6.2f %6.2f %6.2f\n", node->Number(), pos.GetX(), pos.GetY(), pos.GetZ());
                        }
                 }
              }
              
              /*     sellines1 = tops.SelectAttributedLines(SegmentLabel, seg1); 
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){
                 mindist = 1000;
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos).Length()<mindist && abs(pos.GetZ()-this_point->Z())<0.15) mindist = (this_point->vect()-pos).Length();
                 }
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos).Length()==mindist && mindist < 1 && abs(pos.GetZ()-this_point->Z())<0.15) this_point->vect()=pos;
                 }
              }
          */   
              } 
if (connectiontarget19){
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[3];
             seg3 = segment_numbers[1]; //seg3 =  node20, so that is why switched [1] and [2]
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             plane3 = sel_laser_points.FitPlane(seg3, seg3, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             pnl3 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg3);
             Intersect2Planes(plane1, plane2, linecorner);
             IntersectLine3DPlane(linecorner, plane3, cornerpoint);
             pos = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z());
             fprintf(statfile,"\nbuilding %d, target %d, try to adjust segment %d\n", map_line->Number(), target, seg2);         
             sellines1 = tops.SelectAttributedLines(SegmentLabel, seg1); 
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){
                 mindist = 1000;
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos).Length()<mindist && fabs(pos.GetZ()-this_point->Z())<0.15) mindist = (this_point->vect()-pos).Length();
                 }
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos).Length()==mindist && mindist < 1 && fabs(pos.GetZ()-this_point->Z())<0.15) {
                        this_point->vect()=pos;
                        fprintf(statfile,"\nchanged %d to %6.2f %6.2f %6.2f\n", node->Number(), pos.GetX(), pos.GetY(), pos.GetZ());
                        }
                 }
              }
     /*        sellines1 = tops.SelectAttributedLines(SegmentLabel, seg2); 
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){
                 mindist = 1000;
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos).Length()<mindist && abs(pos.GetZ()-this_point->Z())<0.15) mindist = (this_point->vect()-pos).Length();
                 }
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos).Length()==mindist && mindist < 1 && abs(pos.GetZ()-this_point->Z())<0.15) this_point->vect()=pos;
                 }
              }
*/
              } 

         }while (changed);
 //       }while (hyponrs.size()<1);         
         tops.erase(tops.begin(), tops.end());    
         processedlaserpoints.ErasePoints();
         hyponrs.erase(hyponrs.begin(), hyponrs.end());
         add_first_floor = true;
         
         do {
         target = 0;
         changed = false;
          gable= false; gambrel= false; hip= false; halfhip= false; mansard = false; 
          flat = false, l_shape = false, pyramid = false, dormer = false, connectiontarget17 = false, step = false,
          convexmansard = false, concavemansard = false, connectiontarget18 = false, connectiontarget19 = false;
          if (sel_matchresults2.size() == 0 && sel_laser_points.size()>0){ //check if flat roof
              flat = true;
              add_first_floor = false;
              }
          for (sel_match_line = sel_matchresults2.begin(); sel_match_line !=sel_matchresults2.end(); sel_match_line++){
             hyponotused = true;
             for (hypo = hyponrs.begin(); hypo!=hyponrs.end(); hypo++){
                 if (sel_match_line->Number()== (*hypo)) hyponotused = false;
                 }
             if (sel_match_line->Attribute(TargetNumberTag)==10|| //first check if shape targets...
                 sel_match_line->Attribute(TargetNumberTag)==20||
                 sel_match_line->Attribute(TargetNumberTag)==21){
                 if (hyponotused){
                   highestcoverage = sel_match_line->Attribute(CoverageTag);
                   highesthypo = *sel_match_line;
                   target = highesthypo.Attribute(TargetNumberTag);
                   hyponr = sel_match_line->Number();
                   changed =true;
                 }
             }
          }
          if (!changed){
          for (sel_match_line = sel_matchresults2.begin(); sel_match_line !=sel_matchresults2.end(); sel_match_line++){
             hyponotused = true;
             for (hypo = hyponrs.begin(); hypo!=hyponrs.end(); hypo++){
                 if (sel_match_line->Number()== (*hypo)) hyponotused = false;
                 }
                 if (sel_match_line->Attribute(TargetNumberTag)==11){
                 if (hyponotused){
                   highestcoverage = sel_match_line->Attribute(CoverageTag);
                   highesthypo = *sel_match_line;
                   target = highesthypo.Attribute(TargetNumberTag);
                   hyponr = sel_match_line->Number();
                   changed =true;
                 }
             }
          }
          }
          if (!changed){
          for (sel_match_line = sel_matchresults2.begin(); sel_match_line !=sel_matchresults2.end(); sel_match_line++){
             hyponotused = true;
             for (hypo = hyponrs.begin(); hypo!=hyponrs.end(); hypo++){
                 if (sel_match_line->Number()== (*hypo)) hyponotused = false;
                 }
                 if(sel_match_line->Attribute(TargetNumberTag)==14){
                 if (hyponotused){
                   highestcoverage = sel_match_line->Attribute(CoverageTag);
                   highesthypo = *sel_match_line;
                   target = highesthypo.Attribute(TargetNumberTag);
                   hyponr = sel_match_line->Number();
                   changed =true;
                 }
             }
          }
          }
          if (!changed){ // no shape targets anymore in sel_matchresults2, look for connections targets
          for (sel_match_line = sel_matchresults2.begin(); sel_match_line !=sel_matchresults2.end(); sel_match_line++){
             hyponotused = true;
             for (hypo = hyponrs.begin(); hypo!=hyponrs.end(); hypo++){
                 if (sel_match_line->Number()== (*hypo)) hyponotused = false;
                 }
             if (hyponotused){
                   highestcoverage = sel_match_line->Attribute(CoverageTag);
                   highesthypo = *sel_match_line;
                   target = highesthypo.Attribute(TargetNumberTag);
                   hyponr = sel_match_line->Number();
                   changed =true;
                }
             }
          }
        
         hyponrs.push_back(hyponr);
         if (target == 10) gable = true;
         if (target == 11) halfhip = true;
         if (target == 12) pyramid = true;
         if (target == 13) step = true;
         if (target == 14) hip = true;
         if (target == 16) l_shape = true;
//         if (target == 17 || target == 16) connectiontarget = true;
         if (target == 20) gambrel = true;
    //     if (target == 21) mansard = true;
         if (target == 22) dormer = true;
         if (target == 23) convexmansard = true;
         if (target == 24) concavemansard = true;
//         floor_height = 0.01*map_line->Attribute(PredictedHeight);
//         first_floor_height = floor_height + 2.6;
         
         if (gable){
             
//             pos    -- mid point of intersection line
//             len    -- length from mid point to end point (=0.5 line length)
//             alfa   -- 2d direction of intersection line
//             h0     -- mid point height
//             h1     -- lowest point of segments
//             d      -- perp dist of lowest point to line
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             partlaserpoints.ErasePoints();
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg1, SegmentNumberTag);

//             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
             sel_laser_points.IntersectFaces(pnl1, pnl2, plane1, plane2,dist2faces, pos1, pos2);
             sellines1 = nexttops.SelectAttributedLines(SegmentLabel, seg1);
          
           mindist =1000;
           dumpos1 = pos1;
           horlength1 = 0, horlength2 =0;
           inside1 = false;
           inside2 = false;
           for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){                 
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos1).Length()<mindist) {
                        mindist = (this_point->vect()-pos1).Length();
                        dumpos = this_point->vect();
                        horlength1 = (this_point->vect()-pos1).Length2D();
                        }
                 }
             }
             if (mindist<1.5) dumpos1 = dumpos;
             mindist = 1000;
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){                 
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos2).Length()<mindist) {
                        mindist = (this_point->vect()-pos2).Length();
                        dumpos = this_point->vect();
                        horlength2 = (this_point->vect()-pos2).Length2D();
                        }
                 }
             }
             if (mindist<1.5 && dumpos.Distance(dumpos1)>0.001) pos2 = dumpos;
             pos1 = dumpos1;
             laspos1.X() = pos1.GetX(),laspos1.Y() = pos1.GetY(),laspos1.Z() = pos1.GetZ();
             if (laspos1.InsidePolygon(map_points, map_line->LineTopologyReference())) inside1 = true;
             laspos2.X() = pos2.GetX(),laspos2.Y() = pos2.GetY(),laspos2.Z() = pos2.GetZ();             
             if (laspos2.InsidePolygon(map_points, map_line->LineTopologyReference())) inside2 =true;

             if(extend_to_map){
                               orig_ridgelength = (pos1-pos2).Length();
                               ext_map_points.erase(ext_map_points.begin(), ext_map_points.end());     
                               for (node2 = map_line->begin(); node2!=map_line->end()-1;node2++){
                                    map_point = map_points.PointIterator(*node2);
                                    ext_map_points.push_back(*map_point);
                               }
                               line3d = Line3D(pos1, pos2);
                               extend_found1 = false;
                               extend_found2 = false;
                               for (node= map_line->begin(); node!=map_line->end()-1;node++){
                                    nbmpoint1 = *(ext_map_points.PointIterator(*node));
                                    nbmppos12d = Position2D(nbmpoint1.X(), nbmpoint1.Y());
                                    next_node=map_line->NextNode(node);
                                    nbmpoint2 = *(ext_map_points.PointIterator(*next_node));
                                    nbmppos22d = Position2D(nbmpoint2.X(), nbmpoint2.Y());
                                    nbmline = Line2D(nbmppos12d, nbmppos22d);
                                    if (line3d.FindIntersection(nbmline, projposonmap1)){
                                      if ((projposonmap1 - pos1).Length()<snap_distance && (projposonmap1 - pos1).Length()<(projposonmap1 - pos2).Length()){
                                          dumpos1 = projposonmap1;
                                          extend_found1 = true;
                                      }
                                      if ((projposonmap1 - pos2).Length()<snap_distance && (projposonmap1 - pos2).Length()<(projposonmap1 - pos1).Length()){
                                          dumpos2 = projposonmap1;
                                          extend_found2 = true;
                                      }
                                    }
                               }
                               if ((dumpos1 - pos2).Length() > orig_ridgelength && extend_found1 && horlength1<0.001 && inside1){
                                   if ((dumpos2 - pos1).Length() > orig_ridgelength && extend_found2 && horlength2<0.001 && inside2){
                                       pos1 = dumpos1;
                                       pos2 = dumpos2;
                                       maplaserpoint.X() = pos1.GetX();
                                       maplaserpoint.Y() = pos1.GetY();
                                       maplaserpoint.Z() = pos1.GetZ();
                                       maplaserpoint.SetAttribute(PolygonNumberTag, map_line->Attribute(BuildingNumberTag));
                                       ext_to_maplaserpoints.push_back(maplaserpoint);
                                        maplaserpoint.X() = pos2.GetX();
                                        maplaserpoint.Y() = pos2.GetY();
                                        maplaserpoint.Z() = pos2.GetZ();
                                        maplaserpoint.SetAttribute(PolygonNumberTag, map_line->Attribute(BuildingNumberTag));
                                        ext_to_maplaserpoints.push_back(maplaserpoint);
                             
                                       }
                                   else {
                                        pos1 = dumpos1;
                                        maplaserpoint.X() = pos1.GetX();
                                        maplaserpoint.Y() = pos1.GetY();
                                        maplaserpoint.Z() = pos1.GetZ();
                                        maplaserpoint.SetAttribute(PolygonNumberTag, map_line->Attribute(BuildingNumberTag));
                                        ext_to_maplaserpoints.push_back(maplaserpoint);
                                       }
                               }
                               else {
                                    if ((dumpos2 - pos1).Length() > orig_ridgelength && extend_found2 && horlength2<0.001 && inside2){
                                        pos2 = dumpos2;
                                        maplaserpoint.X() = pos2.GetX();
                                        maplaserpoint.Y() = pos2.GetY();
                                        maplaserpoint.Z() = pos2.GetZ();
                                        maplaserpoint.SetAttribute(PolygonNumberTag, map_line->Attribute(BuildingNumberTag));
                                        ext_to_maplaserpoints.push_back(maplaserpoint);
                                       }
                                    }          
             }
             partlaserpoints.SwapXZ(); partlaserpoints.SortOnCoordinates();
             partlaserpoints.SwapXZ();
             lowestlaserpoint = partlaserpoints[0];
             meanx = partlaserpoints.Mean()[0];
             meany = partlaserpoints.Mean()[1];
             meanz = partlaserpoints.Mean()[2];
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
             line = Line3D(pos1, pos2);
             pos = Position3D((pos1.GetX()+pos2.GetX())/2, (pos1.GetY()+pos2.GetY())/2, (pos1.GetZ()+pos2.GetZ())/2);
             len = 0.5*(pos1.Distance(pos2));
             alfa = atan((pos2.GetX()-pos1.GetX())/(pos2.GetY()-pos1.GetY()));
             h0 = pos.GetZ();
//             h1 = lowestlaserpoint.Z();
//             h1 = plane1.Z_At(lowestlaserpoint.X(),lowestlaserpoint.Y(),&success);
               h1 = meanz - (h0-meanz); 
             diff2gutter = sel_laser_points.ReturnDifferenceToGutterHeight(seg1, h1);
             
             //dumpos = Position3D(lowestlaserpoint.X(), lowestlaserpoint.Y(), h0);
             dumpos = Position3D(meanx, meany, h0);
//             d = line.DistanceToPoint(dumpos);
             d = 2*(line.DistanceToPoint(dumpos));             
             
             p1x = pos.GetX() - len*sin(alfa);
             p1y = pos.GetY() - len*cos(alfa);
             p1z = h0;
             point = ObjectPoint(p1x,p1y,p1z, pn, 0,0,0,0,0,0);
             p1 = pn;
             points.push_back(point);
             pn++;
             p2x = pos.GetX() + len*sin(alfa);
             p2y = pos.GetY() + len*cos(alfa);
             p2z = h0;
             point = ObjectPoint(p2x,p2y,p2z, pn, 0,0,0,0,0,0);
             p2 = pn;
             points.push_back(point);
             ridgenumbers.push_back(p1);
             ridgenumbers.push_back(p2);
             pn++;
             p3x = pos.GetX() - len*sin(alfa) +d*cos(alfa);
             p3y = pos.GetY() - len*cos(alfa) -d*sin(alfa);
             p3z = h1;
             point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
             p3 = pn;
             points.push_back(point);
             if (overhang){
                          p3x = pos.GetX() - (len-od)*sin(alfa) +(d-od)*cos(alfa);
                          p3y = pos.GetY() - (len-od)*cos(alfa) -(d-od)*sin(alfa);
                          point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);                           
                          }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p4x = pos.GetX() + len*sin(alfa) +d*cos(alfa);
             p4y = pos.GetY() + len*cos(alfa) -d*sin(alfa);
             p4z = h1;
             point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
             p4 = pn;
             points.push_back(point);
             if (overhang){
                           p4x = pos.GetX() + (len-od)*sin(alfa) +(d-od)*cos(alfa);
                           p4y = pos.GetY() + (len-od)*cos(alfa) -(d-od)*sin(alfa);
                           point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p5x = pos.GetX() - len*sin(alfa) - d*cos(alfa);
             p5y = pos.GetY() - len*cos(alfa) + d*sin(alfa);
             p5z = h1;
             point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
             p5 = pn;
             points.push_back(point);
             if (overhang){
                           p5x = pos.GetX() - (len-od)*sin(alfa) -(d-od)*cos(alfa);
                           p5y = pos.GetY() - (len-od)*cos(alfa) +(d-od)*sin(alfa);
                           point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             
             pn++;
             p6x = pos.GetX() + len*sin(alfa) -d*cos(alfa);
             p6y = pos.GetY() + len*cos(alfa) +d*sin(alfa);
             p6z = h1;
             point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
             p6 = pn;
             points.push_back(point);
             if (overhang){
                           p6x = pos.GetX() + (len-od)*sin(alfa) -(d-od)*cos(alfa);
                           p6y = pos.GetY() + (len-od)*cos(alfa) +(d-od)*sin(alfa);
                           point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
 //            for (segment_number = segment_numbers.begin(); segment_number!=segment_numbers.end(); segment_number++){
 //                pnl = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, *segment_number);
 //                objpseg1 = sel_laser_points.Centroid(pnl, pn);
 //                centroidofseg = LaserPoint(objpseg1.X(), objpseg1.Y(), objpseg1.Z());
 //                if (centroidofseg.InsidePolygon(points, top.LineTopologyReference())){
 //                   top.SetAttribute(SegmentLabel, *segment_number);
 //                   top.SetAttribute(LineLabelTag, *segment_number);
 //                }                    
 //            }
    
 //            top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));
              seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));


             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p2));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             one_map_line.erase(one_map_line.begin(), one_map_line.end());
             wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p5));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             one_map_line.push_back(top);
             sel_map_points.RemoveDoublePoints(one_map_line, 0.1);
             one_map_line.ReNumber(sel_map_points, 0, 0);
             number_offset = sel_map_points.HighestPointNumber().Number()+1;

//             sel_map_points.DuplicateWithFixedZ(first_floor_height, number_offset);
             sel_map_points.DuplicateWithFixedZ(first_floor_height, number_offset);
             wall_map_lines.AddWalls(one_map_line, number_offset);
             wall_map_lines.ReNumber(sel_map_points, all_wall_points.size()+1, all_wall_lines.size()+1);
             all_wall_lines.insert(all_wall_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
             all_wall_points.insert(all_wall_points.end(), sel_map_points.begin(), sel_map_points.end());
             
             processedlaserpoints.AddPoints(partlaserpoints);
             }
  if (gambrel){
//             pos    -- mid point of intersection line
//             len    -- length from mid point to end point (=0.5 line length)
//             alfa   -- 2d direction of intersection line
//             h0     -- mid point height
//             h1     -- lowest point of segment_numbers
//             d      -- perp dist of lowest point to line
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
//                 printf("looking for segment %d\n", node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             seg3 = segment_numbers[2];
             seg4 = segment_numbers[3];
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             plane3 = sel_laser_points.FitPlane(seg3, seg3, SegmentNumberTag);
             plane4 = sel_laser_points.FitPlane(seg4, seg4, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             partlaserpoints.ErasePoints();
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg1, SegmentNumberTag);
//             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
             
             sel_laser_points.IntersectFaces(pnl1, pnl2, plane1, plane2, dist2faces, pos1, pos2);
    /*         sellines1 = nexttops.SelectAttributedLines(SegmentLabel, seg1);
             mindist = 1000;
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){                 
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos1).Length()<mindist) {
                        mindist = (this_point->vect()-pos1).Length();
                        dumpos = this_point->vect();
                        }
                 }
             }
             if (mindist<1.5) pos1 = dumpos;
             mindist = 1000;
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){                 
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos2).Length()<mindist) {
                        mindist = (this_point->vect()-pos2).Length();
                        dumpos = this_point->vect();
                        }
                 }
             }
             if (mindist<1.5) pos2 = dumpos;
  */            sellines1 = nexttops.SelectAttributedLines(SegmentLabel, seg1);
             sellines2 = nexttops.SelectAttributedLines(SegmentLabel, seg2);
              if (sellines1.size() >0 && sellines2.size()>0){
             firstfound = false;
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){                 
                 for (node = selline->begin(); node!=selline->end();node++){
                     nr1 = node->Number();
                     for (selline2 = sellines2.begin(); selline2!=sellines2.end(); selline2++){                 
                        for (node2 = selline2->begin(); node2!=selline2->end();node2++){
                          if (node2->Number() == nr1){
                            if (!firstfound) {
                                           pos3 = points.PointIterator(*node2)->vect();
                                           firstfound = true;
                                           }
                            else {
                               pos4 = points.PointIterator(*node2)->vect();
                               }
                           }
                          }
                     }
                 }
             }
           if (pos3.GetX()>pos4.GetX()) dumpos = pos4, pos4 = pos3, pos3 = dumpos;
           if (pos1.GetX()>pos2.GetX()) dumpos = pos2, pos2 = pos1, pos1 = dumpos;
           pos1 = pos3;
           pos2 = pos4;
           }
             partlaserpoints.SwapXZ(); partlaserpoints.SortOnCoordinates();
             partlaserpoints.SwapXZ();
             lowestlaserpoint = partlaserpoints[0];
             meanx = partlaserpoints.Mean()[0];
             meany = partlaserpoints.Mean()[1];
             meanz = partlaserpoints.Mean()[2];
             
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
             line = Line3D(pos1, pos2);
             
             pos = Position3D((pos1.GetX()+pos2.GetX())/2, (pos1.GetY()+pos2.GetY())/2, (pos1.GetZ()+pos2.GetZ())/2);
             len = 0.5*(pos1.Distance(pos2));
             alfa = atan((pos2.GetX()-pos1.GetX())/(pos2.GetY()-pos1.GetY()));
             h0 = pos.GetZ();
             
             h1 = meanz - (h0-meanz); 
             dumpos = Position3D(meanx, meany, h0);
             partlaserpoints.ErasePoints();
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg3, SegmentNumberTag);
             meanx2 = partlaserpoints.Mean()[0];
             meany2 = partlaserpoints.Mean()[1];
             meanz2 = partlaserpoints.Mean()[2];
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg4, SegmentNumberTag);
             partlaserpoints.SwapXZ(); partlaserpoints.SortOnCoordinates();
             partlaserpoints.SwapXZ();
             lowestlaserpoint_secondlevel = partlaserpoints[0];
             h2 = meanz2 - (h1-meanz2); 
             dumpos2 = Position3D(meanx2, meany2, h0);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg1, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
  
//             h1 = lowestlaserpoint.Z();
//             h1 = plane1.Z_At(lowestlaserpoint.X(),lowestlaserpoint.Y(),&success);
             
//             h2 = lowestlaserpoint_secondlevel.Z();
             
          //   dumpos = Position3D(lowestlaserpoint.X(), lowestlaserpoint.Y(), h0);
//             dumpos2 = Position3D(lowestlaserpoint_secondlevel.X(), lowestlaserpoint_secondlevel.Y(), h0);
              
             d1 = 2*(line.DistanceToPoint(dumpos));
             d2 = line.DistanceToPoint(dumpos2) - d1 + line.DistanceToPoint(dumpos2);
             if (d2<d1) d2 = d1+0.5;
             
             p1x = pos.GetX() - len*sin(alfa);
             p1y = pos.GetY() - len*cos(alfa);
             p1z = h0;
             point = ObjectPoint(p1x,p1y,p1z, pn, 0,0,0,0,0,0);
             p1 = pn;
             points.push_back(point);
             pn++;
             p2x = pos.GetX() + len*sin(alfa);
             p2y = pos.GetY() + len*cos(alfa);
             p2z = h0;
             point = ObjectPoint(p2x,p2y,p2z, pn, 0,0,0,0,0,0);
             p2 = pn;
             points.push_back(point);
             ridgenumbers.push_back(p1);
             ridgenumbers.push_back(p2);
             pn++;
             p3x = pos.GetX() - len*sin(alfa) +d1*cos(alfa);
             p3y = pos.GetY() - len*cos(alfa) -d1*sin(alfa);
             p3z = h1;
             point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
             p3 = pn;
             points.push_back(point);
             pn++;
             p4x = pos.GetX() + len*sin(alfa) +d1*cos(alfa);
             p4y = pos.GetY() + len*cos(alfa) -d1*sin(alfa);
             p4z = h1;
             point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
             p4 = pn;
             points.push_back(point);
             pn++;
             p5x = pos.GetX() - len*sin(alfa) -d1*cos(alfa);
             p5y = pos.GetY() - len*cos(alfa) +d1*sin(alfa);
             p5z = h1;
             point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
             p5 = pn;
             points.push_back(point);
             pn++;
             p6x = pos.GetX() + len*sin(alfa) -d1*cos(alfa);
             p6y = pos.GetY() + len*cos(alfa) +d1*sin(alfa);
             p6z = h1;
             point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
             p6 = pn;
             points.push_back(point);
             pn++;
             
             p7x = pos.GetX() - len*sin(alfa) -d2*cos(alfa);
             p7y = pos.GetY() - len*cos(alfa) +d2*sin(alfa);
             p7z = h2;
             point = ObjectPoint(p7x,p7y,p7z, pn, 0,0,0,0,0,0);
             p7 = pn;
             points.push_back(point);
             if (overhang){
                           p7x = pos.GetX() - len*sin(alfa) -(d2-od)*cos(alfa);
                           p7y = pos.GetY() - len*cos(alfa) +(d2-od)*sin(alfa);
                           p7z = h2;
                           point = ObjectPoint(p7x,p7y,p7z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p8x = pos.GetX() + len*sin(alfa) -d2*cos(alfa);
             p8y = pos.GetY() + len*cos(alfa) +d2*sin(alfa);
             p8z = h2;
             point = ObjectPoint(p8x,p8y,p8z, pn, 0,0,0,0,0,0);
             p8 = pn;
             points.push_back(point);
             if (overhang){
                           p8x = pos.GetX() + len*sin(alfa) -(d2-od)*cos(alfa);
                           p8y = pos.GetY() + len*cos(alfa) +(d2-od)*sin(alfa);
                           p8z = h2;
                           point = ObjectPoint(p8x,p8y,p8z, pn, 0,0,0,0,0,0);
             }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p9x = pos.GetX() - len*sin(alfa) +d2*cos(alfa);
             p9y = pos.GetY() - len*cos(alfa) -d2*sin(alfa);
             p9z = h2;
             point = ObjectPoint(p9x,p9y,p9z, pn, 0,0,0,0,0,0);
             p9 = pn;
             points.push_back(point);
             if (overhang){
                           p9x = pos.GetX() - len*sin(alfa) +(d2-od)*cos(alfa);
                           p9y = pos.GetY() - len*cos(alfa) -(d2-od)*sin(alfa);
                           point = ObjectPoint(p9x,p9y,p9z, pn, 0,0,0,0,0,0);
                }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p10x = pos.GetX() + len*sin(alfa) +d2*cos(alfa);
             p10y = pos.GetY() + len*cos(alfa) -d2*sin(alfa);
             p10z = h2;
             point = ObjectPoint(p10x,p10y,p10z, pn, 0,0,0,0,0,0);
             p10 = pn;
             points.push_back(point);
             if (overhang){
                           p10x = pos.GetX() + len*sin(alfa) +(d2-od)*cos(alfa);
                           p10y = pos.GetY() + len*cos(alfa) -(d2-od)*sin(alfa);
                           point = ObjectPoint(p10x,p10y,p10z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p10));
             top.push_back(PointNumber(p9));
             top.push_back(PointNumber(p3));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p7));
             top.push_back(PointNumber(p8));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p5));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p7));
             top.push_back(PointNumber(p9));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p8));
             top.push_back(PointNumber(p10));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p2));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             one_map_line.erase(one_map_line.begin(), one_map_line.end());
             wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p7));
             top.push_back(PointNumber(p8));
             top.push_back(PointNumber(p10));
             top.push_back(PointNumber(p9));
             top.push_back(PointNumber(p7));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             one_map_line.push_back(top);
             sel_map_points.RemoveDoublePoints(one_map_line, 0.1);
             one_map_line.ReNumber(sel_map_points, 0, 0);
             number_offset = sel_map_points.HighestPointNumber().Number()+1;
             sel_map_points.DuplicateWithFixedZ(first_floor_height, number_offset);
             wall_map_lines.AddWalls(one_map_line, number_offset);
             wall_map_lines.ReNumber(sel_map_points, all_wall_points.size()+1, all_wall_lines.size()+1);
             all_wall_lines.insert(all_wall_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
             all_wall_points.insert(all_wall_points.end(), sel_map_points.begin(), sel_map_points.end());
             
             processedlaserpoints.AddPoints(partlaserpoints);
               
               
             }
  if (hip){
//             pos    -- mid point of intersection line
//             len    -- length from mid point to end points (=0.5 line length)
//             alfa   -- 2d direction of intersection line
//             h0     -- mid point height
//             h1     -- lowest point of (two major) segment_numbers
//             d      -- perp dist of lowest point to line
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
                 printf("looking for segment %d\n", node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             seg3 = segment_numbers[2];
             seg4 = segment_numbers[3];
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             plane3 = sel_laser_points.FitPlane(seg3, seg3, SegmentNumberTag);
             plane4 = sel_laser_points.FitPlane(seg4, seg4, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             partlaserpoints.ErasePoints();
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg1, SegmentNumberTag);
   //          partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
  //           inclination = plane1.Normal()[2];
//            printf("inclination: %4.2f \n", inclination);
             
             sel_laser_points.IntersectFaces(pnl1, pnl2, plane1, plane2, dist2faces, pos1, pos2);
     sellines1 = nexttops.SelectAttributedLines(SegmentLabel, seg1);
             sellines2 = nexttops.SelectAttributedLines(SegmentLabel, seg2);
             firstfound = false;
             if (sellines1.size() >0 && sellines2.size()>0){
             printf("size of sellines2 = %d\n", sellines2.size());
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){                 
                 for (node = selline->begin(); node!=selline->end();node++){
                     nr1 = node->Number();
                     for (selline2 = sellines2.begin(); selline2!=sellines2.end(); selline2++){                 
                        for (node2 = selline2->begin(); node2!=selline2->end();node2++){
                          if (node2->Number() == nr1){
                            if (!firstfound) {
                                           pos3 = points.PointIterator(*node2)->vect();
                                           firstfound = true;
                                           }
                            else {
                               pos4 = points.PointIterator(*node2)->vect();
                               }
                           }
                          }
                     }
                 }
             }
           if (pos3.GetX()>pos4.GetX()) dumpos = pos4, pos4 = pos3, pos3 = dumpos;
           if (pos1.GetX()>pos2.GetX()) dumpos = pos2, pos2 = pos1, pos1 = dumpos;
           pos1 = pos3;
           pos2 = pos4;
           }
    /*         sellines1 = nexttops.SelectAttributedLines(SegmentLabel, seg1);
             mindist = 1000;
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){                 
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos1).Length()<mindist) {
                        mindist = (this_point->vect()-pos1).Length();
                        dumpos = this_point->vect();
                        }
                 }
             }
             if (mindist<1.5) pos1 = dumpos;
             mindist = 1000;
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){                 
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos2).Length()<mindist) {
                        mindist = (this_point->vect()-pos2).Length();
                        dumpos = this_point->vect();
                        }
                 }
             }
             if (mindist<1.5) pos2 = dumpos;
      */       
             partlaserpoints.SwapXZ(); partlaserpoints.SortOnCoordinates();
             partlaserpoints.SwapXZ();
             lowestlaserpoint = partlaserpoints[0];
             meanx = partlaserpoints.Mean()[0];
             meany = partlaserpoints.Mean()[1];
             meanz = partlaserpoints.Mean()[2];
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg3, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg4, SegmentNumberTag);
             line = Line3D(pos1, pos2);
             IntersectLine3DPlane(line, plane3, cornerpoint);
             pos12 = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z());
             IntersectLine3DPlane(line, plane4, cornerpoint);
             pos22 = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z());
             pos = Position3D((pos12.GetX()+pos22.GetX())/2, (pos12.GetY()+pos22.GetY())/2, (pos12.GetZ()+pos22.GetZ())/2);
             len = 0.5*(pos12.Distance(pos22));
             alfa = atan((pos22.GetX()-pos12.GetX())/(pos22.GetY()-pos12.GetY()));
             h0 = pos.GetZ();
             
             h1 = lowestlaserpoint.Z();
             h1 = plane1.Z_At(lowestlaserpoint.X(),lowestlaserpoint.Y(),&success);
             
             dumpos = Position3D(lowestlaserpoint.X(), lowestlaserpoint.Y(), h0);
             d = line.DistanceToPoint(dumpos);

             h1 = meanz - (h0-meanz); 
             dumpos = Position3D(meanx, meany, h0);
//             d = line.DistanceToPoint(dumpos);
             d = 2*(line.DistanceToPoint(dumpos)); 
             
             
             d2=d;
             if (assym){
             // check if hip roof assymetric: 2 different inclinations
             assymlaserpoints.ErasePoints();
             assymlaserpoints.AddTaggedPoints(sel_laser_points, seg3, SegmentNumberTag);
             assymlaserpoints.SwapXZ(); assymlaserpoints.SortOnCoordinates();
             assymlaserpoints.SwapXZ();
             assymlowestlaserpoint = assymlaserpoints[0];
             lowestpointassym = Position3D(assymlowestlaserpoint.X(), assymlowestlaserpoint.Y(), assymlowestlaserpoint.Z());
             posassym = line.Project(lowestpointassym);             
             d_assym1 = posassym.Distance(pos12);
             assymlaserpoints.ErasePoints();
             assymlaserpoints.AddTaggedPoints(sel_laser_points, seg4, SegmentNumberTag);
             assymlaserpoints.SwapXZ(); assymlaserpoints.SortOnCoordinates();
             assymlaserpoints.SwapXZ();
             assymlowestlaserpoint = assymlaserpoints[0];
             lowestpointassym = Position3D(assymlowestlaserpoint.X(), assymlowestlaserpoint.Y(), assymlowestlaserpoint.Z());
             posassym = line.Project(lowestpointassym);
             d_assym2 = posassym.Distance(pos22);
              
             
   //          printf("Assym distances: %4.2f %4.2f %4.2f \n", d - d_assym1, d-d_assym2, d);
             if (d_assym1>d_assym2) d2 = d_assym1;
             else d2 = d_assym2;
             
             if ((d - d_assym1)> 1 && (d-d_assym2)>1) d2=d2;
             else d2=d;
             } //end if assym
             p1x = pos.GetX() - len*sin(alfa);
             p1y = pos.GetY() - len*cos(alfa);
             p1z = h0;
             point = ObjectPoint(p1x,p1y,p1z, pn, 0,0,0,0,0,0);
             p1 = pn;
             points.push_back(point);
             pn++;
             p2x = pos.GetX() + len*sin(alfa);
             p2y = pos.GetY() + len*cos(alfa);
             p2z = h0;
             point = ObjectPoint(p2x,p2y,p2z, pn, 0,0,0,0,0,0);
             p2 = pn;
             points.push_back(point);
             ridgenumbers.push_back(p1);
             ridgenumbers.push_back(p2);
             dumpos = Position3D(p1x, p1y, p1z);
             
             pn++;
             p3x = pos.GetX() - (len+d2)*sin(alfa) +d*cos(alfa);
             p3y = pos.GetY() - (len+d2)*cos(alfa) -d*sin(alfa);
             p3z = h1;
             point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
             p3 = pn;
             points.push_back(point);
             if (overhang){
                          p3x = pos.GetX() - (len+d2-od)*sin(alfa) +(d-od)*cos(alfa);
                          p3y = pos.GetY() - (len+d2-od)*cos(alfa) -(d-od)*sin(alfa);
                          point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p4x = pos.GetX() + (len+d2)*sin(alfa) +d*cos(alfa);
             p4y = pos.GetY() + (len+d2)*cos(alfa) -d*sin(alfa);
             p4z = h1;
             point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
             p4 = pn;
             points.push_back(point);
             if (overhang){
                           p4x = pos.GetX() + (len+d2-od)*sin(alfa) +(d-od)*cos(alfa);
                           p4y = pos.GetY() + (len+d2-od)*cos(alfa) -(d-od)*sin(alfa);
                           point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p5x = pos.GetX() - (len+d2)*sin(alfa) -d*cos(alfa);
             p5y = pos.GetY() - (len+d2)*cos(alfa) +d*sin(alfa);
             p5z = h1;
             point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
             p5 = pn;
             points.push_back(point);
             if (overhang){
                          p5x = pos.GetX() - (len+d2-od)*sin(alfa) -(d-od)*cos(alfa);
                          p5y = pos.GetY() - (len+d2-od)*cos(alfa) +(d-od)*sin(alfa);
                          point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p6x = pos.GetX() + (len+d2)*sin(alfa) -d*cos(alfa);
             p6y = pos.GetY() + (len+d2)*cos(alfa) +d*sin(alfa);
             p6z = h1;
             point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
             p6 = pn;
             points.push_back(point);
             if (overhang){
                          p6x = pos.GetX() + (len+d2-od)*sin(alfa) -(d-od)*cos(alfa);
                          p6y = pos.GetY() + (len+d2-od)*cos(alfa) +(d-od)*sin(alfa);
                          point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p2));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             one_map_line.erase(one_map_line.begin(), one_map_line.end());
             wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p5));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             one_map_line.push_back(top);
             sel_map_points.RemoveDoublePoints(one_map_line, 0.1);
             one_map_line.ReNumber(sel_map_points, 0, 0);
             number_offset = sel_map_points.HighestPointNumber().Number()+1;

             sel_map_points.DuplicateWithFixedZ(first_floor_height, number_offset);
             wall_map_lines.AddWalls(one_map_line, number_offset);
             wall_map_lines.ReNumber(sel_map_points, all_wall_points.size()+1, all_wall_lines.size()+1);
             all_wall_lines.insert(all_wall_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
             all_wall_points.insert(all_wall_points.end(), sel_map_points.begin(), sel_map_points.end());
             
             processedlaserpoints.AddPoints(partlaserpoints);

             }
  if (halfhip){
//             pos    -- point of intersection 3 planes
//             len    -- length of horizontal int line
//             alfa   -- 2d direction of intersection line
//             h0     -- pos height
//             h1     -- lowest point of (two major) segment_numbers
//             d      -- perp dist of lowest point to line
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
//                 printf("looking for segment %d\n", node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             seg3 = segment_numbers[2];
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             plane3 = sel_laser_points.FitPlane(seg3, seg3, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             pnl3 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg3);
             partlaserpoints.ErasePoints();
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg1, SegmentNumberTag);
 //            partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
             
             sel_laser_points.IntersectFaces(pnl1, pnl2, plane1, plane2, dist2faces, pos1, pos2);
     sellines1 = nexttops.SelectAttributedLines(SegmentLabel, seg1);
             sellines2 = nexttops.SelectAttributedLines(SegmentLabel, seg2);
              if (sellines1.size() >0 && sellines2.size()>0){
             firstfound = false;
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){                 
                 for (node = selline->begin(); node!=selline->end();node++){
                     nr1 = node->Number();
                     for (selline2 = sellines2.begin(); selline2!=sellines2.end(); selline2++){                 
                        for (node2 = selline2->begin(); node2!=selline2->end();node2++){
                          if (node2->Number() == nr1){
                            if (!firstfound) {
                                           pos3 = points.PointIterator(*node2)->vect();
                                           firstfound = true;
                                           }
                            else {
                               pos4 = points.PointIterator(*node2)->vect();
                               }
                           }
                          }
                     }
                 }
             }
           if (pos3.GetX()>pos4.GetX()) dumpos = pos4, pos4 = pos3, pos3 = dumpos;
           if (pos1.GetX()>pos2.GetX()) dumpos = pos2, pos2 = pos1, pos1 = dumpos;
           pos1 = pos3;
           pos2 = pos4;
           }
/*             sellines1 = nexttops.SelectAttributedLines(SegmentLabel, seg1);
             mindist = 1000;
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){                 
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos1).Length()<mindist) {
                        mindist = (this_point->vect()-pos1).Length();
                        dumpos = this_point->vect();
                        }
                 }
             }
             if (mindist<1.5) pos1 = dumpos;
             mindist = 1000;
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){                 
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos2).Length()<mindist) {
                        mindist = (this_point->vect()-pos2).Length();
                        dumpos = this_point->vect();
                        }
                 }
             }
             if (mindist<1.5) pos2 = dumpos;
             
  */           
             partlaserpoints.SwapXZ(); partlaserpoints.SortOnCoordinates();
             partlaserpoints.SwapXZ();
             lowestlaserpoint = partlaserpoints[0];
             meanx = partlaserpoints.Mean()[0];
             meany = partlaserpoints.Mean()[1];
             meanz = partlaserpoints.Mean()[2];

             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg3, SegmentNumberTag);
             line = Line3D(pos1, pos2);
             IntersectLine3DPlane(line, plane3, cornerpoint);
             centrepoint = sel_laser_points.Centroid(pnl3, pn); // trick, but more reliable on small horizontal lines between pos1 and pos2
             pos = Position3D(centrepoint.X(), centrepoint.Y(), centrepoint.Z());
             if (pos.Distance(pos1)>pos.Distance(pos2)) pos22 = pos1;
             else pos22 = pos2;
             pos = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z()); 
             len = pos.Distance(pos22);
             line = Line3D(pos, pos22);
             alfa = atan((pos22.GetX()-pos.GetX())/(pos22.GetY()-pos.GetY()));
             if (pos.GetY()>pos22.GetY()) alfa = alfa + PI;
//             else alfa = atan((pos.GetX()-pos22.GetX())/(pos.GetY()-pos22.GetY()));
//             printf("building %d, main shape is %d\n", map_line->Number(), target);
//             printf("alfa =  %4.2f\n", alfa);

             h0 = pos.GetZ();
             h1 = lowestlaserpoint.Z();
             h1 = plane1.Z_At(lowestlaserpoint.X(),lowestlaserpoint.Y(),&success);
             dumpos = Position3D(lowestlaserpoint.X(), lowestlaserpoint.Y(), h0);
             d = line.DistanceToPoint(dumpos);
             h1 = meanz - (h0-meanz); 
             dumpos = Position3D(meanx, meany, h0);
//             d = line.DistanceToPoint(dumpos);
             d = 2*(line.DistanceToPoint(dumpos));       
             // check if half hip roof assymetric: 2 different inclinations
             d2=d;
             if (assym){
             assymlaserpoints.ErasePoints();
             assymlaserpoints.AddTaggedPoints(sel_laser_points, seg3, SegmentNumberTag);
             assymlaserpoints.SwapXZ(); assymlaserpoints.SortOnCoordinates();
             assymlaserpoints.SwapXZ();
             assymlowestlaserpoint = assymlaserpoints[0];
             lowestpointassym = Position3D(assymlowestlaserpoint.X(), assymlowestlaserpoint.Y(), assymlowestlaserpoint.Z());
             posassym = line.Project(lowestpointassym);             
             d2 = posassym.Distance(pos);
             if ((d - d2)< 1) d2=d;
             }             
             p1x = pos.GetX();// - len*sin(alfa);
             p1y = pos.GetY();// - len*cos(alfa);
             p1z = h0;
             point = ObjectPoint(p1x,p1y,p1z, pn, 0,0,0,0,0,0);
             p1 = pn;
             points.push_back(point);
             pn++;
             p2x = pos22.GetX();// + len*sin(alfa);
             p2y = pos22.GetY();// + len*cos(alfa);
             p2z = h0;
             point = ObjectPoint(p2x,p2y,p2z, pn, 0,0,0,0,0,0);
             p2 = pn;
             points.push_back(point);
             ridgenumbers.push_back(p1);
             ridgenumbers.push_back(p2);
             pn++;
             p3x = pos.GetX() -  d2*sin(alfa) +d*cos(alfa);
             p3y = pos.GetY() -  d2*cos(alfa) -d*sin(alfa);
             p3z = h1;
             point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
             p3 = pn;
             points.push_back(point);
             if (overhang){
                           p3x = pos.GetX() -  (d2-od)*sin(alfa) +(d-od)*cos(alfa);
                           p3y = pos.GetY() -  (d2-od)*cos(alfa) -(d-od)*sin(alfa);
                           point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p4x = pos22.GetX() + d*cos(alfa);
             p4y = pos22.GetY()  -d*sin(alfa);
             p4z = h1;
             point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
             p4 = pn;
             points.push_back(point);
             if (overhang){
                          p4x = pos22.GetX() + (d-od)*cos(alfa);
                          p4y = pos22.GetY()  -(d-od)*sin(alfa);
                          point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p5x = pos.GetX() - d2*sin(alfa) -d*cos(alfa);
             p5y = pos.GetY() - d2*cos(alfa) +d*sin(alfa);
             p5z = h1;
             point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
             p5 = pn;
             points.push_back(point);
             if (overhang){
                         p5x = pos.GetX() - (d2-od)*sin(alfa) -(d-od)*cos(alfa);
                         p5y = pos.GetY() - (d2-od)*cos(alfa) +(d-od)*sin(alfa);
                         point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p6x = pos22.GetX() - d*cos(alfa);
             p6y = pos22.GetY() + d*sin(alfa);
             p6z = h1;
             point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
             p6 = pn;
             points.push_back(point);
             if (overhang){
                          p6x = pos22.GetX() - (d-od)*cos(alfa);
                          p6y = pos22.GetY() + (d-od)*sin(alfa);
                          point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));
             top.SetAttribute(LineLabelTag, target);
             top.SetAttribute(SegmentLabel, 0);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

    
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p2));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             one_map_line.erase(one_map_line.begin(), one_map_line.end());
             wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p5));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             one_map_line.push_back(top);
             sel_map_points.RemoveDoublePoints(one_map_line, 0.1);
             one_map_line.ReNumber(sel_map_points, 0, 0);
             number_offset = sel_map_points.HighestPointNumber().Number()+1;
             sel_map_points.DuplicateWithFixedZ(first_floor_height, number_offset);
             wall_map_lines.AddWalls(one_map_line, number_offset);
             wall_map_lines.ReNumber(sel_map_points, all_wall_points.size()+1, all_wall_lines.size()+1);
             all_wall_lines.insert(all_wall_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
             all_wall_points.insert(all_wall_points.end(), sel_map_points.begin(), sel_map_points.end());
             
             processedlaserpoints.AddPoints(partlaserpoints);

             }
  if (l_shape){
//             pos    -- point of intersection 3 planes
//             len    -- length of horizontal int line between 1 and 4
//             len2   -- length of second line
//             alfa   -- 2d direction of intersection line
//             alfa2
//             h0     -- pos height
//             h1     -- lowest point of (two major) segment_numbers
//             d      -- perp dist of lowest point to line
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
//                 printf("looking for segment %d\n", node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             seg3 = segment_numbers[2];
             seg4 = segment_numbers[3];
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             plane3 = sel_laser_points.FitPlane(seg3, seg3, SegmentNumberTag);
             plane4 = sel_laser_points.FitPlane(seg4, seg4, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             pnl3 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg3);
             pnl4 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg4);
             Intersect2Planes(plane1, plane3, linecorner);
             partlaserpoints.ErasePoints();
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg1, SegmentNumberTag); 
             meanx = partlaserpoints.Mean()[0];
             meany = partlaserpoints.Mean()[1];
             meanz = partlaserpoints.Mean()[2];
 
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg4, SegmentNumberTag);
             partlaserpoints.SwapXZ(); partlaserpoints.SortOnCoordinates();
             partlaserpoints.SwapXZ();
             lowestlaserpoint = partlaserpoints[0];
             //lowest point now determined first two segment_numbers.. now add other segment_numbers..
             partlaserpoints.ErasePoints();
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
             meanx2 = partlaserpoints.Mean()[0];
             meany2 = partlaserpoints.Mean()[1];
             meanz2 = partlaserpoints.Mean()[2];
             
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg3, SegmentNumberTag);
             partlaserpoints.SwapXZ(); partlaserpoints.SortOnCoordinates();
             partlaserpoints.SwapXZ();
             lowestlaserpoint2 = partlaserpoints[0];
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg1, SegmentNumberTag);  
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg4, SegmentNumberTag);
             
             sel_laser_points.IntersectFaces(pnl1, pnl4, plane1, plane4, dist2faces, pos1, pos2);
             sel_laser_points.IntersectFaces(pnl2, pnl3, plane2, plane3, dist2faces, pos3, pos4);
             sel_laser_points.IntersectFaces(pnl1, pnl3, plane1, plane3, dist2faces, pos61, pos62);
             sel_laser_points.IntersectFaces(pnl2, pnl4, plane2, plane4, dist2faces, pos91, pos92);
             line = Line3D(pos1, pos2);
             IntersectLine3DPlane(line, plane3, cornerpoint);
             pos = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z());
             if (pos.Distance(pos1)>pos.Distance(pos2)) pos22 = pos1;
             else pos22 = pos2;

             if (pos.Distance(pos3)>pos.Distance(pos4)) pos42 = pos3;
             else pos42 = pos4;
             if (pos.Distance(pos61)>pos.Distance(pos62)) pos6 = pos61;
             else pos6 = pos62;
             if (pos.Distance(pos91)>pos.Distance(pos92)) pos9 = pos91;
             else pos9 = pos92;

             sellines1 = nexttops.SelectAttributedLines(SegmentLabel, seg1);
             if (sellines1.size() >0){
             mindist = 1000;
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){                 
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos).Length()<mindist) {
                        mindist = (this_point->vect()-pos).Length();
                        dumpos = this_point->vect();
                        }
                 }
             }
             if (mindist<1.5) pos = dumpos;
             mindist = 1000;
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){                 
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos22).Length()<mindist) {
                        mindist = (this_point->vect()-pos22).Length();
                        dumpos = this_point->vect();
                        }
                 }
             }
             if (mindist<1.5) pos22 = dumpos;
             sellines1 = nexttops.SelectAttributedLines(SegmentLabel, seg2);
             mindist = 1000;
             for (selline = sellines1.begin(); selline!=sellines1.end(); selline++){                 
                 for (node = selline->begin(); node!=selline->end();node++){
                      this_point = points.PointIterator(*node);
                      if ((this_point->vect()-pos42).Length()<mindist) {
                        mindist = (this_point->vect()-pos42).Length();
                        dumpos = this_point->vect();
                        }
                 }
             }
             if (mindist<1.5) pos42 = dumpos;
             
             }
             len = pos.Distance(pos22);
             line = Line3D(pos, pos22);
             alfa = atan((pos22.GetX()-pos.GetX())/(pos22.GetY()-pos.GetY())); 
             if (pos.GetY()>pos22.GetY()) alfa = alfa + PI;
//             else alfa = atan((pos.GetX()-pos22.GetX())/(pos.GetY()-pos22.GetY()));
  //           printf("building %d, main shape is %d\n", map_line->Number(), target);
  //           printf("alfa =  %4.2f\n", alfa);
             
             len2 = pos.Distance(pos42);
             line2 = Line3D(pos, pos42);
             alfa2 = atan((pos.GetX()-pos42.GetX())/(pos.GetY()-pos42.GetY()));
             if (pos42.GetY()< pos.GetY()) alfa2 = alfa2 + PI;

             h0 = pos.GetZ();
             dumpos = Position3D(lowestlaserpoint.X(), lowestlaserpoint.Y(), h0);
             dumpos = Position3D(meanx, meany, h0);
             d1 = 2*(line.DistanceToPoint(dumpos));
             dumpos = Position3D(lowestlaserpoint2.X(), lowestlaserpoint2.Y(), h0);
             dumpos = Position3D(meanx2, meany2, h0);
             d2 = 2*(line2.DistanceToPoint(dumpos));
             d2 = d1;                  
             h0 = pos.GetZ();
             h1 = lowestlaserpoint.Z();
             h1 = meanz - (pos.GetZ()-meanz);
             pos6 = linecorner.DetPositionZ(h1);
             pos9.X() = pos.X()+(pos.X()-pos6.X());
             pos9.Y() = pos.Y()+(pos.Y()-pos6.Y());
             pos9.Z() = pos6.Z();
             dumline = Line3D(pos, pos6);
             dumpos = dumline.DetPositionZ(h1);
             pos6 = dumpos;
             dumline = Line3D(pos, pos9);
             dumpos = dumline.DetPositionZ(h1);
             pos9 = dumpos;
             alfadum = atan((pos6.GetX()-pos9.GetX())/(pos6.GetY()-pos9.GetY()));            
             dumpos = Position3D(lowestlaserpoint.X(), lowestlaserpoint.Y(), h0);
//             d = line.DistanceToPoint(dumpos);
//             d2 = d;//line.DistanceToPoint(pos6);
//             d1 = d;//line2.DistanceToPoint(pos6);
             p1x = pos.GetX();// - len*sin(alfa);
             p1y = pos.GetY();// - len*cos(alfa);
             p1z = h0;
             point = ObjectPoint(p1x,p1y,p1z, pn, 0,0,0,0,0,0);
             p1 = pn;
             points.push_back(point);
             pn++;
             p2x = pos22.GetX();// + len*sin(alfa);
             p2y = pos22.GetY();// + len*cos(alfa);
             p2z = h0;
             point = ObjectPoint(p2x,p2y,p2z, pn, 0,0,0,0,0,0);
             p2 = pn;
             points.push_back(point);
             pn++;
             p3x = pos42.GetX();// + len*sin(alfa);
             p3y = pos42.GetY();// + len*cos(alfa);
             p3z = h0;
             point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
             p3 = pn;
             points.push_back(point);
             pn++;
             dumpos.X() =  pos22.GetX() +d1*cos(alfa);
             dumpos.Y() = pos22.GetY() -d1*sin(alfa);
             dumpos.Z() = h1;
             dumpos2.X() =  pos22.GetX() -d1*cos(alfa);
             dumpos2.Y() = pos22.GetY() +d1*sin(alfa);
             dumpos2.Z() = h1;
             if (dumpos.Distance(pos9)>dumpos2.Distance(pos9)) alfa = alfa+PI;
             
             p4x = pos22.GetX() + d1*cos(alfa);
             p4y = pos22.GetY()  -d1*sin(alfa);
             p4z = pos9.GetZ();
             point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
             p4 = pn;
             points.push_back(point);
             if (overhang){
                          p4x = pos22.GetX() + (d1-od)*cos(alfa);
                          p4y = pos22.GetY()  -(d1-od)*sin(alfa);
                          point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;  
             p5x = pos22.GetX() - d1*cos(alfa);
             p5y = pos22.GetY()  +d1*sin(alfa);
             p5z = pos6.GetZ();
             point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
             p5 = pn;
             points.push_back(point);
             if (overhang){
                          p5x = pos22.GetX() - (d1-od)*cos(alfa);
                          p5y = pos22.GetY()  +(d1-od)*sin(alfa);
                          point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;  
             p6x = pos6.GetX();// - d*cos(alfa);
             p6y = pos6.GetY();//  +d*sin(alfa);
             p6z = pos6.GetZ();
             point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
             p6 = pn;
             points.push_back(point);
             if (overhang){
                          p6x = pos6.GetX() - od*cos(alfadum);
                          p6y = pos6.GetY() - od*sin(alfadum);
                          point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             dumpos.X() =  pos42.GetX() +d1*cos(alfa2);
             dumpos.Y() = pos42.GetY() -d1*sin(alfa2);
             dumpos.Z() = h1;
             dumpos2.X() =  pos42.GetX() -d1*cos(alfa2);
             dumpos2.Y() = pos42.GetY() +d1*sin(alfa2);
             dumpos2.Z() = h1;
             if (dumpos.Distance(pos6)>dumpos2.Distance(pos6)) alfa2 = alfa2+PI;
             pn++;  
             p7x = pos42.GetX() + d2*cos(alfa2);
             p7y = pos42.GetY()  -d2*sin(alfa2);
             p7z = pos6.GetZ();
             point = ObjectPoint(p7x,p7y,p7z, pn, 0,0,0,0,0,0);
             p7 = pn;
             points.push_back(point);
             if (overhang){
                          p7x = pos42.GetX() + (d2-od)*cos(alfa2);
                          p7y = pos42.GetY()  -(d2-od)*sin(alfa2);
                          point = ObjectPoint(p7x,p7y,p7z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;  
             p8x = pos42.GetX() - d2*cos(alfa2);
             p8y = pos42.GetY()  +d2*sin(alfa2);
             p8z = pos9.GetZ();
             point = ObjectPoint(p8x,p8y,p8z, pn, 0,0,0,0,0,0);
             p8 = pn;
             points.push_back(point);
             if (overhang){
                          p8x = pos42.GetX() - (d2-od)*cos(alfa2);
                          p8y = pos42.GetY()  +(d2-od)*sin(alfa2);
                          point = ObjectPoint(p8x,p8y,p8z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             
             pn++;  
             p9x = pos9.GetX();// - d*cos(alfa);
             p9y = pos9.GetY();//  +d*sin(alfa);
             p9z = pos9.GetZ();
             point = ObjectPoint(p9x,p9y,p9z, pn, 0,0,0,0,0,0);
             p9 = pn;
             points.push_back(point);
             if (overhang){
                          p9x = pos9.GetX() + od*cos(alfadum);
                          p9y = pos9.GetY() + od*sin(alfadum);
                          point = ObjectPoint(p9x,p9y,p9z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p9));
             top.push_back(PointNumber(p1));             
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p8));
             top.push_back(PointNumber(p9));
             top.push_back(PointNumber(p1));             
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p7));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p1));             
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);

             one_map_line.erase(one_map_line.begin(), one_map_line.end());
             wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p2));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p7));
             top.push_back(PointNumber(p8));
             top.push_back(PointNumber(p3));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p7));
             top.push_back(PointNumber(p8));
             top.push_back(PointNumber(p9));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p5));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             one_map_line.push_back(top);
             sel_map_points.RemoveDoublePoints(one_map_line, 0.1);
             one_map_line.ReNumber(sel_map_points, 0, 0);
             number_offset = sel_map_points.HighestPointNumber().Number()+1;

             sel_map_points.DuplicateWithFixedZ(first_floor_height, number_offset);
             wall_map_lines.AddWalls(one_map_line, number_offset);
             wall_map_lines.ReNumber(sel_map_points, all_wall_points.size()+1, all_wall_lines.size()+1);
             all_wall_lines.insert(all_wall_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
             all_wall_points.insert(all_wall_points.end(), sel_map_points.begin(), sel_map_points.end());
             
             
  
               }
  if (pyramid){
//             pos    -- point of intersection of biggest 3 planes
//             h0     -- pos height
//             h1     -- lowest point of (two major) segment_numbers
//             d      -- perp dist of lowest point to line
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
//                 printf("looking for segment %d\n", node->Number());
             }
             sort(segment_numbers.begin(), segment_numbers.end());
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             seg3 = segment_numbers[2];
             seg4 = segment_numbers[3];
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             plane3 = sel_laser_points.FitPlane(seg3, seg3, SegmentNumberTag);
             plane4 = sel_laser_points.FitPlane(seg4, seg4, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             pnl3 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg3);
             pnl4 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg4);
             Intersect2Planes(plane1, plane2, linecorner);
             IntersectLine3DPlane(linecorner, plane3, cornerpoint);
             pos = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z());
             h0 = cornerpoint.Z();
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             seg3 = segment_numbers[2];
             seg4 = segment_numbers[3];
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             plane3 = sel_laser_points.FitPlane(seg3, seg3, SegmentNumberTag);
             plane4 = sel_laser_points.FitPlane(seg4, seg4, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             pnl3 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg3);
             pnl4 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg4);

             partlaserpoints.ErasePoints();
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg1, SegmentNumberTag);  
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg4, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg3, SegmentNumberTag);
             partlaserpoints.SwapXZ(); partlaserpoints.SortOnCoordinates();
             partlaserpoints.SwapXZ();
             lowestlaserpoint = partlaserpoints[0];
             h1 = lowestlaserpoint.Z();
             
             sel_laser_points.IntersectFaces(pnl1, pnl4, plane1, plane4, dist2faces, pos1, pos2);
             sel_laser_points.IntersectFaces(pnl2, pnl3, plane2, plane3, dist2faces, pos3, pos4);
             sel_laser_points.IntersectFaces(pnl1, pnl3, plane1, plane3, dist2faces, pos61, pos62);
             sel_laser_points.IntersectFaces(pnl2, pnl4, plane2, plane4, dist2faces, pos91, pos92);

             if (pos.Distance(pos1)>pos.Distance(pos2)) pos14 = pos1;
             else pos14 = pos2;
             if (pos.Distance(pos3)>pos.Distance(pos4)) pos23 = pos3;
             else pos23 = pos4;
             if (pos.Distance(pos61)>pos.Distance(pos62)) pos13 = pos61;
             else pos13 = pos62;
             if (pos.Distance(pos91)>pos.Distance(pos92)) pos24 = pos91;
             else pos24 = pos92;
           
             dumline = Line3D(pos, pos14);
             dumpos = dumline.DetPositionZ(h1);
             pos14 = dumpos;
             dumline = Line3D(pos, pos23);
             dumpos = dumline.DetPositionZ(h1);
             pos23 = dumpos;
             dumline = Line3D(pos, pos13);
             dumpos = dumline.DetPositionZ(h1);
             pos13 = dumpos;
             dumline = Line3D(pos, pos24);
             dumpos = dumline.DetPositionZ(h1);
             pos24 = dumpos;
             
             dumline =Line3D(pos13, pos14);
             alfa = atan((pos13.GetX()-pos14.GetX())/(pos13.GetY()-pos14.GetY()));
             if (pos14.GetY()< pos13.GetY()) alfa = alfa + PI;
                    
             dumpos =Position3D(pos.X(), pos.Y(), h1);
             d = dumline.DistanceToPoint(dumpos);
             
             p0x = pos.GetX();
             p0y = pos.GetY();
             p0z = pos.GetZ();
             point = ObjectPoint(p0x,p0y,p0z, pn, 0,0,0,0,0,0);
             p0 = pn;
             points.push_back(point);
             pn++;
             p1x = pos.GetX() +  d*sin(alfa) -d*cos(alfa);
             p1y = pos.GetY() +  d*cos(alfa) +d*sin(alfa);
             p1z = h1;
             point = ObjectPoint(p1x,p1y,p1z, pn, 0,0,0,0,0,0);
             p1 = pn;
             points.push_back(point);
             if (overhang){
                          p1x = pos.GetX() + (d-od)*sin(alfa) -(d-od)*cos(alfa);
                          p1y = pos.GetY() + (d-od)*cos(alfa) +(d-od)*sin(alfa);
                          point = ObjectPoint(p1x,p1y,p1z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p2x = pos.GetX() -  d*sin(alfa) -d*cos(alfa);
             p2y = pos.GetY() -  d*cos(alfa) +d*sin(alfa);
             p2z = h1;
             point = ObjectPoint(p2x,p2y,p2z, pn, 0,0,0,0,0,0);
             p2 = pn;
             points.push_back(point);
             if (overhang){
                          p2x = pos.GetX() - (d-od)*sin(alfa) -(d-od)*cos(alfa);
                          p2y = pos.GetY() - (d-od)*cos(alfa) +(d-od)*sin(alfa);
                          point = ObjectPoint(p2x,p2y,p2z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p3x = pos.GetX() -  d*sin(alfa) +d*cos(alfa);
             p3y = pos.GetY() -  d*cos(alfa) -d*sin(alfa);
             p3z = h1;
             point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
             p3 = pn;
             points.push_back(point);
             if (overhang){
                          p3x = pos.GetX() - (d-od)*sin(alfa) +(d-od)*cos(alfa);
                          p3y = pos.GetY() - (d-od)*cos(alfa) -(d-od)*sin(alfa);
                          point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p4x = pos.GetX() +  d*sin(alfa) +d*cos(alfa);
             p4y = pos.GetY() +  d*cos(alfa) -d*sin(alfa);
             p4z = h1;
             point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
             p4 = pn;
             points.push_back(point);
             if (overhang){
                          p4x = pos.GetX() + (d-od)*sin(alfa) +(d-od)*cos(alfa);
                          p4y = pos.GetY() + (d-od)*cos(alfa) -(d-od)*sin(alfa);
                          point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
                           }
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p0));
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p0));             
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p0));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p0));             
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p0));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p0));             
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p0));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p0));             
             top.SetAttribute(SegmentLabel, 0);
             top.SetAttribute(LineLabelTag, target);
             seg_laser_points.ErasePoints();
             for (laser_point=partlaserpoints.begin(); laser_point!=partlaserpoints.end(); laser_point++) {
                  if (laser_point->InsidePolygon(points, top.LineTopologyReference())){
                     seg_laser_points.push_back(*laser_point);
                  }  
             }
             
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.SetAttribute(SegmentLabel, segnr);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));

             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);

             one_map_line.erase(one_map_line.begin(), one_map_line.end());
             wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p1));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             one_map_line.push_back(top);
             sel_map_points.RemoveDoublePoints(one_map_line, 0.1);
             one_map_line.ReNumber(sel_map_points, 0, 0);
             number_offset = sel_map_points.HighestPointNumber().Number()+1;
             sel_map_points.DuplicateWithFixedZ(first_floor_height, number_offset);
             wall_map_lines.AddWalls(one_map_line, number_offset);
             wall_map_lines.ReNumber(sel_map_points, all_wall_points.size()+1, all_wall_lines.size()+1);
             all_wall_lines.insert(all_wall_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
             all_wall_points.insert(all_wall_points.end(), sel_map_points.begin(), sel_map_points.end());             
               }

  if (connectiontarget17){
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             seg3 = segment_numbers[2]; //this is the segment where the first two should be fit on...
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
                        
                        }
  if (dormer){
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             partlaserpoints.ErasePoints();
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg1, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
             partlaserpoints2.ErasePoints();
             partlaserpoints2.AddTaggedPoints(partlaserpoints, 1600, LabelTag); //dormer label 
             partlaserpoints2.SwapXZ(); partlaserpoints2.SortOnCoordinates();
             partlaserpoints2.SwapXZ();
             lowestlaserpoint = partlaserpoints2[0];
             segnr = partlaserpoints2.MostFrequentAttributeValue(SegmentNumberTag, count);
             supseg = seg1;
             dormerseg = seg2;
             if (seg1==segnr) supseg = seg2, dormerseg = seg1;
             supplane = sel_laser_points.FitPlane(supseg, supseg, SegmentNumberTag);
             
             sel_laser_points.IntersectFaces(pnl1, pnl2, plane1, plane2, 0.5, pos1, pos2);
             line = Line3D(pos1, pos2);
             pos22d = Position2D(pos2.GetX(), pos2.GetY());
             pos12d = Position2D(pos1.GetX(), pos1.GetY());
             line2d = Line2D(pos12d, pos22d);

             pos = Position3D((pos1.GetX()+pos2.GetX())/2, (pos1.GetY()+pos2.GetY())/2, (pos1.GetZ()+pos2.GetZ())/2);
             len = 0.5*(pos1.Distance(pos2));
             alfa = atan((pos2.GetX()-pos1.GetX())/(pos2.GetY()-pos1.GetY()));
             h0 = pos.GetZ();
             h1 = lowestlaserpoint.Z();
             maxdist = 0;
             for (laser_point=partlaserpoints2.begin(); laser_point!=partlaserpoints2.end(); laser_point++) {
                  dumpos2d = laser_point->Position2DOnly();
                  dumpos.X() = dumpos2d.X();
                  dumpos.Y() = dumpos2d.Y();
                  dumpos.Z() = h1;
                  dist = line2d.DistanceToPoint(laser_point->Position2DOnly());
                  if (dist > maxdist) {
                      maxdist = dist;
                      keepdumpos = dumpos;
                  }
             }
             dumpos = keepdumpos;
             d = maxdist;
             dumpos1.X() = pos.GetX() +d*cos(alfa);
             dumpos1.Y() = pos.GetY() -d*sin(alfa);
             dumpos1.Z() = h1;
             
             dumpos2.X() = pos.GetX() -d*cos(alfa);
             dumpos2.Y() = pos.GetY() +d*sin(alfa);
             dumpos2.Z() = h1;
             if (dumpos2.Distance(dumpos)<dumpos1.Distance(dumpos)) alfa = alfa +PI;

             p1x = pos.GetX() - len*sin(alfa);
             p1y = pos.GetY() - len*cos(alfa);
             p1z = h0;
             point = ObjectPoint(p1x,p1y,p1z, pn, 0,0,0,0,0,0);
             p1 = pn;
             points.push_back(point);
             pn++;
             
             p2x = pos.GetX() + len*sin(alfa);
             p2y = pos.GetY() + len*cos(alfa);
             p2z = h0;
             point = ObjectPoint(p2x,p2y,p2z, pn, 0,0,0,0,0,0);
             p2 = pn;
             points.push_back(point);
             pn++;

             p3x = pos.GetX() + len*sin(alfa) + d*cos(alfa);
             p3y = pos.GetY() + len*cos(alfa) - d*sin(alfa);
             p3z = h1;
             point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
             p3 = pn;
             points.push_back(point);
             pn++;
            
             p4x = pos.GetX() - len*sin(alfa) + d*cos(alfa);
             p4y = pos.GetY() - len*cos(alfa) - d*sin(alfa);
             p4z = h1;
             point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
             p4 = pn;
             points.push_back(point);
             pn++;
             
             p5x = p3x;
             p5y = p3y;
             p5z = supplane.Z_At(p5x, p5y, &success);
             point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
             p5 = pn;
             points.push_back(point);
             pn++;
             
             p6x = p4x;
             p6y = p4y;
             p6z = supplane.Z_At(p6x, p6y, &success);
             point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
             p6 = pn;
             points.push_back(point);
             pn++;
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p1));                          
             top.SetAttribute(SegmentLabel, dormerseg);
             top.SetAttribute(LineLabelTag, target);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p1));             
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p2));             
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p5));             
             top.push_back(PointNumber(p3));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             }
  if (mansard){
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             partlaserpoints.ErasePoints();
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg1, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
             partlaserpoints2.ErasePoints();
             partlaserpoints2.AddTaggedPoints(partlaserpoints, 1000, LabelTag); //flat part label 
             partlaserpoints.RemoveTaggedPoints(1000, LabelTag);
             partlaserpoints.SwapXZ(); partlaserpoints.SortOnCoordinates();
             partlaserpoints.SwapXZ();
             lowestlaserpoint = partlaserpoints[0];
             maxheight = partlaserpoints.ReturnHeightOfPercentilePoint(100);
             
             segnr = partlaserpoints2.MostFrequentAttributeValue(SegmentNumberTag, count);
             supseg = seg1;
             flatseg = seg2;
             if (seg1==segnr) supseg = seg2, flatseg = seg1;
             supplane = sel_laser_points.FitPlane(supseg, supseg, SegmentNumberTag);
             
             sel_laser_points.IntersectFaces(pnl1, pnl2, plane1, plane2, dist2faces, pos1, pos2);
             line = Line3D(pos1, pos2);
             pos22d = Position2D(pos2.GetX(), pos2.GetY());
             pos12d = Position2D(pos1.GetX(), pos1.GetY());
             line2d = Line2D(pos12d, pos22d);

             pos = Position3D((pos1.GetX()+pos2.GetX())/2, (pos1.GetY()+pos2.GetY())/2, (pos1.GetZ()+pos2.GetZ())/2);
             len = 0.5*(pos1.Distance(pos2));
             alfa = atan((pos2.GetX()-pos1.GetX())/(pos2.GetY()-pos1.GetY()));
             h0 = pos.GetZ();
             h1 = lowestlaserpoint.Z();
             if (h0>h1 && h0+0.5>maxheight){
              dumpos2 = Position3D(lowestlaserpoint.X(), lowestlaserpoint.Y(), h0);
              d2 = line.DistanceToPoint(dumpos2);

        for (laser_point=partlaserpoints2.begin(); laser_point!=partlaserpoints2.end(); laser_point++) {
                  dumpos2d = laser_point->Position2DOnly();
                  dumpos.X() = dumpos2d.X();
                  dumpos.Y() = dumpos2d.Y();
                  dumpos.Z() = h1;
                  dist = line2d.DistanceToPoint(laser_point->Position2DOnly());
                  if (dist > maxdist) {
                      maxdist = dist;
                      keepdumpos = dumpos;
                  }
             }
             dumpos = keepdumpos;
             d = maxdist;
             dumpos1.X() = pos.GetX() +d*cos(alfa);
             dumpos1.Y() = pos.GetY() -d*sin(alfa);
             dumpos1.Z() = h1;
             
             dumpos2.X() = pos.GetX() -d*cos(alfa);
             dumpos2.Y() = pos.GetY() +d*sin(alfa);
             dumpos2.Z() = h1;
             if (dumpos2.Distance(dumpos)<dumpos1.Distance(dumpos)) alfa = alfa +PI;
             
             p1x = pos.GetX() - len*sin(alfa);
             p1y = pos.GetY() - len*cos(alfa);
             p1z = h0;
             point = ObjectPoint(p1x,p1y,p1z, pn, 0,0,0,0,0,0);
             p1 = pn;
             points.push_back(point);
             pn++;
             
             p2x = pos.GetX() + len*sin(alfa);
             p2y = pos.GetY() + len*cos(alfa);
             p2z = h0;
             point = ObjectPoint(p2x,p2y,p2z, pn, 0,0,0,0,0,0);
             p2 = pn;
             points.push_back(point);
             pn++;

             p3x = pos.GetX() + len*sin(alfa) + d*cos(alfa);
             p3y = pos.GetY() + len*cos(alfa) - d*sin(alfa);
             p3z = h0;
             point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
             p3 = pn;
             points.push_back(point);
             pn++;
            
             p4x = pos.GetX() - len*sin(alfa) + d*cos(alfa);
             p4y = pos.GetY() - len*cos(alfa) - d*sin(alfa);
             p4z = h0;
             point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
             p4 = pn;
             points.push_back(point);
             pn++;
             
             p5x = pos.GetX() - len*sin(alfa) -d2*cos(alfa);
             p5y = pos.GetY() - len*cos(alfa) +d2*sin(alfa);
             
             p5z = h1;
             point = ObjectPoint(p5x,p5y,p5z, pn, 0,0,0,0,0,0);
             p5 = pn;
             points.push_back(point);
             pn++;
             
             p6x = pos.GetX() + len*sin(alfa) -d2*cos(alfa);
             p6y = pos.GetY() + len*cos(alfa) +d2*sin(alfa);
             p6z = h1;
             point = ObjectPoint(p6x,p6y,p6z, pn, 0,0,0,0,0,0);
             p6 = pn;
             points.push_back(point);
             pn++;
             
             p7x = p3x;
             p7y = p3y;
             p7z = h1;
             point = ObjectPoint(p7x,p7y,p7z, pn, 0,0,0,0,0,0);
             p7 = pn;
             points.push_back(point);
             pn++;
             
             p8x = p4x;
             p8y = p4y;
             p8z = h1;
             point = ObjectPoint(p8x,p8y,p8z, pn, 0,0,0,0,0,0);
             p8 = pn;
             points.push_back(point);
             pn++;
            
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p1));                          
             top.SetAttribute(SegmentLabel, flatseg);
             top.SetAttribute(LineLabelTag, target);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p8));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p1));             
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p7));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p2));             
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p8));
             top.push_back(PointNumber(p7));
             top.push_back(PointNumber(p3));             
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p5));             
             top.push_back(PointNumber(p1));
             top.SetAttribute(SegmentLabel, supseg);
             top.SetAttribute(LineLabelTag, supseg);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             one_map_line.erase(one_map_line.begin(), one_map_line.end());
             wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p8));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p7));
             top.push_back(PointNumber(p8));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             one_map_line.push_back(top);
             sel_map_points.RemoveDoublePoints(one_map_line, 0.1);
             one_map_line.ReNumber(sel_map_points, 0, 0);
             number_offset = sel_map_points.HighestPointNumber().Number()+1;
             sel_map_points.DuplicateWithFixedZ(first_floor_height, number_offset);
             wall_map_lines.AddWalls(one_map_line, number_offset);
             wall_map_lines.ReNumber(sel_map_points, all_wall_points.size()+1, all_wall_lines.size()+1);
             all_wall_lines.insert(all_wall_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
             all_wall_points.insert(all_wall_points.end(), sel_map_points.begin(), sel_map_points.end());             

             }
             }
  if (convexmansard){
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             seg3 = segment_numbers[2];
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             plane3 = sel_laser_points.FitPlane(seg3, seg3, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             pnl3 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg3);
             partlaserpoints.ErasePoints();
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg1, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg3, SegmentNumberTag);
             sel_laser_points.IntersectFaces(pnl1, pnl2, plane1, plane2, dist2faces, pos1, pos2);
             partlaserpoints.SwapXZ(); partlaserpoints.SortOnCoordinates();
             partlaserpoints.SwapXZ();
             lowestlaserpoint = partlaserpoints[0];
             line = Line3D(pos1, pos2);
             IntersectLine3DPlane(line, plane3, cornerpoint);
             pos = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z());
             h0 = pos.GetZ();
             if (pos.Distance(pos1)>pos.Distance(pos2)) pos22 = pos1;
             else pos22 = pos2;
             h1 = pos22.GetZ();
             sel_laser_points.IntersectFaces(pnl2, pnl3, plane2, plane3, dist2faces, pos1, pos2);
             if (pos.Distance(pos1)>pos.Distance(pos2)) pos42 = pos1;
             else pos42 = pos2;
             sel_laser_points.IntersectFaces(pnl1, pnl3, plane1, plane3, dist2faces, pos1, pos2);
             if (pos.Distance(pos1)>pos.Distance(pos2)) pos62 = pos1;
             else pos62 = pos2;
             
             p1x = pos.GetX();
             p1y = pos.GetY();
             p1z = h0;
             point = ObjectPoint(p1x,p1y,p1z, pn, 0,0,0,0,0,0);
             p1 = pn;
             points.push_back(point);
             pn++;
             
             p2x = pos42.GetX();
             p2y = pos42.GetY();
             p2z = h0;
             point = ObjectPoint(p2x,p2y,p2z, pn, 0,0,0,0,0,0);
             p2 = pn;
             points.push_back(point);
             pn++;

             p3x = pos62.GetX();
             p3y = pos62.GetY();
             p3z = h0;
             point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
             p3 = pn;
             points.push_back(point);
             pn++;
            
             p4x = pos42.GetX() + (pos62.GetX()-pos.GetX());
             p4y = pos42.GetY() + (pos62.GetY()-pos.GetY());
             p4z = h0;
             point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
             p4 = pn;
             points.push_back(point);
             pn++;
             
             dumpos1 = Position3D(p4x, p4y, h1);
             dumpos2 = Position3D(p3x, p3y, h1);
             dumline = Line3D(dumpos1, dumpos2);
             IntersectLine3DPlane(dumline, plane1, cornerpoint);
             pos = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z());
             p5x = pos.GetX();
             p5y = pos.GetY();
             p5z = h1;
             point = ObjectPoint(p5x, p5y, p5z, pn, 0,0,0,0,0,0);
             p5 = pn;
             points.push_back(point);
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             dumpos1 = Position3D(p4x, p4y, h1);
             dumpos2 = Position3D(p2x, p2y, h1);
             dumline = Line3D(dumpos1, dumpos2);
             IntersectLine3DPlane(dumline, plane2, cornerpoint);
             pos = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z());
             p6x = pos.GetX();
             p6y = pos.GetY();
             p6z = h1;
             point = ObjectPoint(p6x, p6y, p6z, pn, 0,0,0,0,0,0);
             p6 = pn;
             points.push_back(point);
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p7x = pos22.GetX();
             p7y = pos22.GetY();
             p7z = h1;
             point = ObjectPoint(p7x, p7y, p7z, pn, 0,0,0,0,0,0);
             p7 = pn;
             points.push_back(point);
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p8x = p4x;
             p8y = p4y;
             p8z = h1;
             point = ObjectPoint(p8x, p8y, p8z, pn, 0,0,0,0,0,0);
             p8 = pn;
             points.push_back(point);
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));                          
             top.SetAttribute(SegmentLabel, seg3);
             top.SetAttribute(LineLabelTag, target);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p7));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));             
             top.SetAttribute(LineLabelTag, target);
             top.SetAttribute(SegmentLabel, seg1);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p7));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p1));             
             top.SetAttribute(LineLabelTag, target);
             top.SetAttribute(SegmentLabel, seg2);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p8));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p8));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p8));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p8));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             one_map_line.erase(one_map_line.begin(), one_map_line.end());
             wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p8));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p7));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p8));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             one_map_line.push_back(top);
             sel_map_points.RemoveDoublePoints(one_map_line, 0.1);
             one_map_line.ReNumber(sel_map_points, 0, 0);
             number_offset = sel_map_points.HighestPointNumber().Number()+1;
             sel_map_points.DuplicateWithFixedZ(first_floor_height, number_offset);
             wall_map_lines.AddWalls(one_map_line, number_offset);
             wall_map_lines.ReNumber(sel_map_points, all_wall_points.size()+1, all_wall_lines.size()+1);
             all_wall_lines.insert(all_wall_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
             all_wall_points.insert(all_wall_points.end(), sel_map_points.begin(), sel_map_points.end());             
                                  
                     }
  if (concavemansard){
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
             for (node = highesthypo.begin(); node!=highesthypo.end();node++){
                 segment_numbers.push_back(node->Number());
             }
             seg1 = segment_numbers[0];
             seg2 = segment_numbers[1];
             seg3 = segment_numbers[2];
             plane1 = sel_laser_points.FitPlane(seg1, seg1, SegmentNumberTag);
             plane2 = sel_laser_points.FitPlane(seg2, seg2, SegmentNumberTag);
             plane3 = sel_laser_points.FitPlane(seg3, seg3, SegmentNumberTag);
             pnl1 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg1);
             pnl2 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg2);
             pnl3 = sel_laser_points.TaggedPointNumberList(SegmentNumberTag, seg3);
             partlaserpoints.ErasePoints();
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg1, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg2, SegmentNumberTag);
             partlaserpoints.AddTaggedPoints(sel_laser_points, seg3, SegmentNumberTag);
             sel_laser_points.IntersectFaces(pnl1, pnl2, plane1, plane2, dist2faces, pos1, pos2);
             partlaserpoints.SwapXZ(); partlaserpoints.SortOnCoordinates();
             partlaserpoints.SwapXZ();
             lowestlaserpoint = partlaserpoints[0];
             line = Line3D(pos1, pos2);
             IntersectLine3DPlane(line, plane3, cornerpoint);
             pos = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z());
             h0 = pos.GetZ();
             if (pos.Distance(pos1)>pos.Distance(pos2)) pos22 = pos1;
             else pos22 = pos2;
             h1 = pos22.GetZ();
             sel_laser_points.IntersectFaces(pnl2, pnl3, plane2, plane3, dist2faces, pos1, pos2);
             if (pos.Distance(pos1)>pos.Distance(pos2)) pos42 = pos1;
             else pos42 = pos2;
             sel_laser_points.IntersectFaces(pnl1, pnl3, plane1, plane3, dist2faces, pos1, pos2);
             if (pos.Distance(pos1)>pos.Distance(pos2)) pos62 = pos1;
             else pos62 = pos2;
             
             p1x = pos.GetX();
             p1y = pos.GetY();
             p1z = h0;
             point = ObjectPoint(p1x,p1y,p1z, pn, 0,0,0,0,0,0);
             p1 = pn;
             points.push_back(point);
             pn++;
             
             p2x = pos42.GetX();
             p2y = pos42.GetY();
             p2z = h0;
             point = ObjectPoint(p2x,p2y,p2z, pn, 0,0,0,0,0,0);
             p2 = pn;
             points.push_back(point);
             pn++;

             p3x = pos62.GetX();
             p3y = pos62.GetY();
             p3z = h0;
             point = ObjectPoint(p3x,p3y,p3z, pn, 0,0,0,0,0,0);
             p3 = pn;
             points.push_back(point);
             pn++;
            
             p4x = pos42.GetX() + (pos62.GetX()-pos.GetX());
             p4y = pos42.GetY() + (pos62.GetY()-pos.GetY());
             p4z = h0;
             point = ObjectPoint(p4x,p4y,p4z, pn, 0,0,0,0,0,0);
             p4 = pn;
             points.push_back(point);
             pn++;
             
             dumpos1 = Position3D(p4x, p4y, h1);
             dumpos2 = Position3D(p3x, p3y, h1);
             dumline = Line3D(dumpos1, dumpos2);
             IntersectLine3DPlane(dumline, plane1, cornerpoint);
             pos = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z());
             p5x = pos.GetX();
             p5y = pos.GetY();
             p5z = h1;
             point = ObjectPoint(p5x, p5y, p5z, pn, 0,0,0,0,0,0);
             p5 = pn;
             points.push_back(point);
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             dumpos1 = Position3D(p4x, p4y, h1);
             dumpos2 = Position3D(p2x, p2y, h1);
             dumline = Line3D(dumpos1, dumpos2);
             IntersectLine3DPlane(dumline, plane2, cornerpoint);
             pos = Position3D(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z());
             p6x = pos.GetX();
             p6y = pos.GetY();
             p6z = h1;
             point = ObjectPoint(p6x, p6y, p6z, pn, 0,0,0,0,0,0);
             p6 = pn;
             points.push_back(point);
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p7x = pos22.GetX();
             p7y = pos22.GetY();
             p7z = h1;
             point = ObjectPoint(p7x, p7y, p7z, pn, 0,0,0,0,0,0);
             p7 = pn;
             points.push_back(point);
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             p8x = p4x;
             p8y = p4y;
             p8z = h1;
             point = ObjectPoint(p8x, p8y, p8z, pn, 0,0,0,0,0,0);
             p8 = pn;
             points.push_back(point);
             sel_map_points.push_back(point); //point belongs to wall
             pn++;
             
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));                          
             top.SetAttribute(SegmentLabel, seg3);
             top.SetAttribute(LineLabelTag, target);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p7));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p1));             
             top.SetAttribute(LineLabelTag, target);
             top.SetAttribute(SegmentLabel, seg1);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p1));
             top.push_back(PointNumber(p7));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p1));             
             top.SetAttribute(LineLabelTag, target);
             top.SetAttribute(SegmentLabel, seg2);
             top.SetAttribute(LineLabelTag, top.Attribute(SegmentLabel));
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);

             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p8));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p3));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p8));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p8));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p2));
             top.push_back(PointNumber(p4));
             top.push_back(PointNumber(p8));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             one_map_line.erase(one_map_line.begin(), one_map_line.end());
             wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
             top.erase(top.begin(), top.end());
             top.Initialise();
             top.push_back(PointNumber(p8));
             top.push_back(PointNumber(p5));
             top.push_back(PointNumber(p7));
             top.push_back(PointNumber(p6));
             top.push_back(PointNumber(p8));
             top.SetAttribute(LineLabelTag, 5);
             top.SetAttribute(BuildingNumberTag, map_line->Number());
             tops.push_back(top);
             
             one_map_line.push_back(top);
             sel_map_points.RemoveDoublePoints(one_map_line, 0.1);
             one_map_line.ReNumber(sel_map_points, 0, 0);
             number_offset = sel_map_points.HighestPointNumber().Number()+1;
             sel_map_points.DuplicateWithFixedZ(first_floor_height, number_offset);
             wall_map_lines.AddWalls(one_map_line, number_offset);
             wall_map_lines.ReNumber(sel_map_points, all_wall_points.size()+1, all_wall_lines.size()+1);
             all_wall_lines.insert(all_wall_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
             all_wall_points.insert(all_wall_points.end(), sel_map_points.begin(), sel_map_points.end());             
                                  
                                  
                     }
  if (flat){
       //      add_first_floor = false;
             sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
             seg_laser_points.ErasePoints();
             seg_laser_points.AddTaggedPoints(sel_laser_points, 1500, LabelTag);
             if (seg_laser_points.size()==0) seg_laser_points.AddTaggedPoints(sel_laser_points, 1000, LabelTag);
             if (seg_laser_points.size()==0) seg_laser_points = sel_laser_points;
             segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
             top.clear(); top.Initialise();
             for (node=map_line->begin(); node!=map_line->end(); node++) {
                  this_point = map_points.PointIterator(*node);
                  point = ObjectPoint(this_point->X(),this_point->Y(),seg_laser_points.Mean()[2], pn, 0,0,0,0,0,0);
                  top.push_back(PointNumber(pn));
                  points.push_back(point);
                  sel_map_points.push_back(point);
                  pn++;
             }
              top.SetAttribute(SegmentLabel, segnr);
              top.SetAttribute(LineLabelTag, 12);
              top.SetAttribute(LineLabelTag, segnr);
              top.SetAttribute(BuildingNumberTag, map_line->Number());
              tops.push_back(top);
              one_map_line.erase(one_map_line.begin(), one_map_line.end());
              wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
              one_map_line.push_back(top);
              sel_map_points.RemoveDoublePoints(one_map_line, 0.1);
              one_map_line.ReNumber(sel_map_points, 0, 0);
              number_offset = sel_map_points.HighestPointNumber().Number()+1;
              sel_map_points.DuplicateWithFixedZ(first_floor_height, number_offset);
              wall_map_lines.AddWalls(one_map_line, number_offset);
              wall_map_lines.SetAttribute(LineLabelTag, 5);
              wall_map_lines.ReNumber(sel_map_points, all_wall_points.size()+1, all_wall_lines.size()+1);
              all_wall_lines.insert(all_wall_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
              all_wall_points.insert(all_wall_points.end(), sel_map_points.begin(), sel_map_points.end());
             }
             }while (changed);
//       }while (hyponrs.size()<1);                      
             shapetops = tops.SelectAttributedLines(BuildingNumberTag, map_line->Number());
             if (shapetops.size()==0){ // if no shape has been reconstructed, just fit flat roof 
                sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
                seg_laser_points.ErasePoints();
                seg_laser_points.AddTaggedPoints(sel_laser_points, 1500, LabelTag);
                if (seg_laser_points.size()==0) seg_laser_points.AddTaggedPoints(sel_laser_points, 1000, LabelTag);
                if (seg_laser_points.size()==0) seg_laser_points = sel_laser_points;
                segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
                top.clear(); top.Initialise();
                for (node=map_line->begin(); node!=map_line->end(); node++) {
                  this_point = map_points.PointIterator(*node);
                  point = ObjectPoint(this_point->X(),this_point->Y(),seg_laser_points.Mean()[2], pn, 0,0,0,0,0,0);
                  top.push_back(PointNumber(pn));
                  points.push_back(point);
                  sel_map_points.push_back(point);
                  pn++;
                  }
                top.SetAttribute(SegmentLabel, segnr);
                top.SetAttribute(LineLabelTag, 12);
                top.SetAttribute(LineLabelTag, segnr);
                top.SetAttribute(BuildingNumberTag, map_line->Number());
                tops.push_back(top);
                one_map_line.erase(one_map_line.begin(), one_map_line.end());
                wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
                one_map_line.push_back(top);
                sel_map_points.RemoveDoublePoints(one_map_line, 0.1);
                one_map_line.ReNumber(sel_map_points, 0, 0);
                number_offset = sel_map_points.HighestPointNumber().Number()+1;
                flatroofpoints.AddPoints(seg_laser_points);

                sel_map_points.DuplicateWithFixedZ(first_floor_height, number_offset);
                wall_map_lines.AddWalls(one_map_line, number_offset);
                wall_map_lines.SetAttribute(LineLabelTag, 5);
                wall_map_lines.ReNumber(sel_map_points, all_wall_points.size()+1, all_wall_lines.size()+1);
                all_wall_lines.insert(all_wall_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
                all_wall_points.insert(all_wall_points.end(), sel_map_points.begin(), sel_map_points.end());                      
                }
             add_first_floor = true;//false             
             if (add_first_floor){
               sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
               top.clear(); top.Initialise();
               for (node=map_line->begin(); node!=map_line->end(); node++) {
                  this_point = map_points.PointIterator(*node);
             //     point = ObjectPoint(this_point->X(),this_point->Y(),first_floor_height, pn, 0,0,0,0,0,0);
                  point = ObjectPoint(this_point->X(),this_point->Y(),first_floor_height, pn, 0,0,0,0,0,0);
                  top.push_back(PointNumber(pn));
                  points.push_back(point);
                  sel_map_points.push_back(point);
                  pn++;
                }
            //    top.SetAttribute(LineLabelTag, 5);
                top.SetAttribute(LineLabelTag, 0);
                top.SetAttribute(BuildingNumberTag, map_line->Number());
                tops.push_back(top);
                one_map_line.erase(one_map_line.begin(), one_map_line.end());
                wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
                one_map_line.push_back(top);
                sel_map_points.RemoveDoublePoints(one_map_line, 0.1);
                one_map_line.ReNumber(sel_map_points, 0, 0);
                number_offset = sel_map_points.HighestPointNumber().Number()+1;
                sel_map_points.DuplicateWithFixedZ(floor_height, number_offset);
//                sel_map_points.DuplicateWithFixedZ(first_floor_height, number_offset);
                wall_map_lines.AddWalls(one_map_line, number_offset);
                wall_map_lines.SetAttribute(LineLabelTag, 5);
                wall_map_lines.insert(wall_map_lines.end(), one_map_line.begin(), one_map_line.end());
                wall_map_lines.ReNumber(sel_map_points, all_wall_points.size()+1, all_wall_lines.size()+1);
                all_wall_lines.insert(all_wall_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
                all_wall_points.insert(all_wall_points.end(), sel_map_points.begin(), sel_map_points.end());            
                first_floor_lines.insert(first_floor_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
                first_floor_points.insert(first_floor_points.end(), sel_map_points.begin(), sel_map_points.end());            
              }     
             keeptops.insert(keeptops.end(), tops.begin(), tops.end());
}
//tops.MakeCounterClockWise(points);
//points.RemoveDoublePoints(tops, 0.001);

/*LaserPoints emptysegpoints;
for (map_line = tops.begin(), index=0; map_line!=tops.end(); map_line++, index++){
    if (map_line->Attribute(SegmentLabel)==0){
       sel_laser_points.ErasePoints();
       sel_laser_points = laser_points.SelectTagValue(PolygonNumberTag, map_line->Attribute(BuildingNumberTag));
       seg_laser_points.ErasePoints();
       for (laser_point=sel_laser_points.begin(); laser_point!=sel_laser_points.end(); laser_point++) {
         if (laser_point->InsidePolygon(points, map_line->LineTopologyReference())){
          seg_laser_points.push_back(*laser_point);
          emptysegpoints.push_back(*laser_point);
          }
       }
       segnr = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
       map_line->Attribute(SegmentLabel) = segnr;
    }
}
       

emptysegpoints.Write("emptysegpoints.laser", false);
*/
flatroofpoints.Write("dontknowroofshape.laser", false);

LineTopologies emptysegtops, verticalroofparts;

for (map_line = nexttops.begin(); map_line!=nexttops.end(); map_line++){
 for (node = map_line->begin(); node!=map_line->end(); node++){
      this_point = points.PointIterator(*node);
      nextpoints.push_back(*this_point);
 }
}
nextpoints.RemoveDoublePoints(nexttops, 0.001);
nextpoints.Write("nextpoint.objpts");
nexttops.Write("nextpoint.top", false);
for (map_line = keeptops.begin(); map_line!=keeptops.end(); map_line++){
 for (node = map_line->begin(); node!=map_line->end(); node++){
      this_point = points.PointIterator(*node);
      keeppoints.push_back(*this_point);
 }
}
keeptops.MakeCounterClockWise(keeppoints);
//keeptops.SetAttribute(LineLabelTag, 4);
keeppoints.RemoveDoublePoints(keeptops, 0.1);
keeppoints.Write(map_points_output);
keeptops.Write(map_topology_output, false);

for (map_line = keeptops.begin(), index=0; map_line!=keeptops.end(); map_line++, index++){
    if (map_line->Attribute(LineLabelTag) == 0){
                                        keeptops.erase(map_line);
                                        map_line--;
    }
}
//keeptops.SetAttribute(LineLabelTag, 2);
dxffile = fopen ("modeldrivenroofs.dxf","w");
keeppoints.WriteDXF(dxffile, keeptops, true);
fclose(dxffile);
//return;
tops = keeptops;
points = keeppoints;

int countlargeres, countlargeressegment, b1, b2;
countlargeres = 0;

maplaserpoints.ErasePoints();
printf("\nstart calculating minimum distance to map points\n");
for (map_line = tops.begin(), index=0; map_line!=tops.end(); map_line++, index++){
  printf("%6d (%5.1f\%)\r", index, 100.0 * index / tops.size());
  if (map_line->Attribute(LineLabelTag)!=5){
  segnr = map_line->Attribute(SegmentLabel);
  seg_laser_points.ErasePoints();
  seg_laser_points.AddTaggedPoints(laser_points, segnr, SegmentNumberTag);  
  if (seg_laser_points.size()>0){
    seg_laser_points.DeriveTIN();
    edges.Erase();
    edges.Derive(seg_laser_points.TINReference());
  
    for (node=map_line->begin(); node!=map_line->end()-1; node++) {
       this_point = points.PointIterator(*node);
       pos = this_point->vect();
       nearestpoint = seg_laser_points.NearestPoint(pos, edges, false);
       maplaserpoint = LaserPoint(this_point->X(), this_point->Y(), this_point->Z());
       maplaserpoint.Residual() = 0;
       maplaserpoint.SetAttribute(SegmentNumberTag, segnr);
       maplaserpoint.SetAttribute(LabelTag, 0);
       if (seg_laser_points.Distance2NearestPoint(pos, nearestpoint)>thres1){
         maplaserpoint.Residual() = 0.6;
         countlargeres++;
         maplaserpoint.SetAttribute(LabelTag, 1);
//         if (seg_laser_points.Distance2NearestPoint(pos, nearestpoint)>thres2) maplaserpoint.Residual() = 0.6;
       }
       maplaserpoints.push_back(maplaserpoint);
    }
    }
  }
}
maplaserpoints.ReduceData(0.1);

if (include_quality){
countlargeres = 0;

countlargeressegment = 0;
LaserPoints          sel_laser_points2, sel_laser_points3, largemaplaserpoints, 
                     sel_laser_points4;
vector <int>         matchedsegment_numbers, buildingnumbers;
sel_laser_points3.ErasePoints();
sel_laser_points3.AddTaggedPoints(maplaserpoints, 1, LabelTag); // number of points inside 3d model
maplaserpoints.Write("qualitymodelpoints.laser", false);
fprintf(statfile,"\n***\nNumber of object points > 100 cm away from nearest laserpoint %d (out of %d)\n", sel_laser_points3.size(), maplaserpoints.size());
//fprintf(statfile,"\n***\nNumber of object points > 100 cm away from nearest laserpoint %d (out of %d)\n", countlargeres, maplaserpoints.size());




printf("\nstart calculating minimum distance to map points, perp to plane\n");
maplaserpoints.ErasePoints();
for (map_line = tops.begin(), index=0; map_line!=tops.end(); map_line++, index++){
  printf("%6d (%5.1f\%)\r", index, 100.0 * index / tops.size());
  segnr = map_line->Attribute(SegmentLabel);
 // seg_laser_points.ErasePoints();
 // seg_laser_points.AddTaggedPoints(laser_points, segnr, SegmentNumberTag);  
  if (map_line->Attribute(LineLabelTag)!=5){
     plane1 = laser_points.FitPlane(segnr, segnr, SegmentNumberTag);
    for (node=map_line->begin(); node!=map_line->end(); node++) {
       this_point = points.PointIterator(*node);
       pos = this_point->vect();
       projpos = plane1.Project(pos);
       maplaserpoint = LaserPoint(this_point->X(), this_point->Y(), this_point->Z());
       maplaserpoint.Residual() = 0;
       maplaserpoint.SetAttribute(SegmentNumberTag, segnr);
       maplaserpoint.Residual() = projpos.Distance(pos);
//       if (projpos.Distance(pos)>thres1) maplaserpoint.Residual() = 0.3;
//       if (projpos.Distance(pos)>thres2) maplaserpoint.Residual() = 0.6;
       maplaserpoints.push_back(maplaserpoint);
    }
  }
}

maplaserpoints.Write("qualitymodelpoints_distance2plane.laser", false);
buildingnumbers = maplaserpoints.AttributeValues(SegmentNumberTag);
b1 = buildingnumbers.size();
buildingnumbers = laser_points.AttributeValues(SegmentNumberTag);
b2 = buildingnumbers.size();
fprintf(statfile,"\n***\nNumber of segments used in model driven approach %d, out of %d = %4.2f\n", b1, b2, 100.0*b1/b2);



maplaserpoints.ErasePoints();

                     

printf("\nstart calculating residuals on laser points (this can take a while)\n");
   for (map_line = map_lines.begin(), index=0; map_line!=map_lines.end(); map_line++, index++){
         printf("%6d (%5.1f\%)\r", index, 100.0 * index / map_lines.size());
         sel_mapresults = tops.SelectAttributedLines(BuildingNumberTag, map_line->Number());
         sel_laser_points.ErasePoints();
         sel_laser_points = laser_points.SelectTagValue(PolygonNumberTag, map_line->Number());
         for (laser_point=sel_laser_points.begin(); laser_point!=sel_laser_points.end(); laser_point++) {
         keepresidual = 10;
         found = false;
             for (face=sel_mapresults.begin(); face!=sel_mapresults.end(); face++) {
                 if (face->size()>2){
                if (laser_point->InsidePolygon(points, face->LineTopologyReference())){
                   plane.Initialise(); 
                   for (node=face->begin(); node!=face->end()-1; node++) {
                       this_point = points.PointIterator(node->NumberRef());
                       plane.AddPoint(this_point->Position3DRef(), false);
                   }
                   plane.Recalculate();
                   found = true;
                //   residual = laser_point->Z() - plane.Z_At(laser_point->X(), laser_point->Y(), &success);
                   pos = laser_point->vect();
                   projpos = plane.Project(pos);
                   residual = projpos.Distance(pos);//laser_point->Z() - plane.Z_At(laser_point->X(), laser_point->Y(), &success);
                   if (fabs(residual)<fabs(keepresidual)) keepresidual = residual;
                }
                }
             }
          maplaserpoint = LaserPoint(laser_point->X(), laser_point->Y(), laser_point->Z());
          maplaserpoint.Residual() = keepresidual;
          maplaserpoint.SetAttribute(PolygonNumberTag, map_line->Number());
          maplaserpoint.SetAttribute(SegmentNumberTag, laser_point->Attribute(SegmentNumberTag));
          maplaserpoint.SetAttribute(LabelTag, 1);
          if (!found) {
             keepresidual =0.6;
             maplaserpoint.SetAttribute(LabelTag, 0);
          }
          maplaserpoints.push_back(maplaserpoint);
          if (found && fabs(keepresidual)>0.2){
               countlargeres++; 
               largemaplaserpoints.push_back(maplaserpoint);             
          }
          
         }
  }
maplaserpoints.Write("qualitylaserpoints_distance2face.laser", false);

sel_laser_points2.ErasePoints();
sel_laser_points2.AddTaggedPoints(largemaplaserpoints, 1, LabelTag); // number of largepoints inside 3d model
matchedsegment_numbers = sel_laser_points2.AttributeValues(SegmentNumberTag);
for (segment_number = matchedsegment_numbers.begin(); segment_number!=matchedsegment_numbers.end(); segment_number++){
    sel_laser_points3.ErasePoints();
    sel_laser_points3.AddTaggedPoints(sel_laser_points2, *segment_number,SegmentNumberTag);
    sel_laser_points.ErasePoints();
    sel_laser_points.AddTaggedPoints(laser_points,*segment_number,SegmentNumberTag);
    if (sel_laser_points3.size()>20 && ((1.0*sel_laser_points3.size())/(1.0*sel_laser_points.size()))>0.33){ //if segm is larger than 20p and ratio of high res is higher than 1/3
         sel_laser_points4.AddTaggedPoints(sel_laser_points2, *segment_number,SegmentNumberTag);
     countlargeressegment++;
     }
}
fprintf(statfile,"Number of points with large residual: %d (out of %d)\n", countlargeres, laser_points.size());

fprintf(statfile,"Number of segments with more than 20 points with large residual: %d (out of %d)\n", countlargeressegment, matchedsegment_numbers.size());
buildingnumbers = sel_laser_points4.AttributeValues(PolygonNumberTag);
b1 = buildingnumbers.size();
buildingnumbers = laser_points.AttributeValues(PolygonNumberTag);
b2 = buildingnumbers.size();
fprintf(statfile,"\n***\nNumber of buildings affected by these segment %d, out of %d = %4.2f\n", b1, b2, 100.0*b1/b2);
sel_laser_points4.Write("modeldrivenqualitylaserpoints_largesegmentswithlargedistance2face.laser", false);



maplaserpoints.ErasePoints();

ext_to_maplaserpoints.Write("modelridgepoints_extendedtomap.laser", false);
emptysegtops = tops.SelectAttributedLines(SegmentLabel, 0);
emptysegtops.Write("emptysegtops.top", false);

} //end bool if quality_report

all_wall_lines.SetAttribute(LineLabelTag, 4); //change it to 4, in order to possibly overlay with datadriven walls.
all_wall_lines.Write("modelwalls.top", false);
all_wall_points.Write("modelwalls.objpts");
all_wall_lines.SetAttribute(LineLabelTag, 5);

dxffile = fopen ("modeldrivenwalls.dxf","w");
all_wall_points.WriteDXF(dxffile, all_wall_lines, true);
fclose(dxffile);

dxffile = fopen ("modeldrivenfirstfloor.dxf","w");
first_floor_points.WriteDXF(dxffile, first_floor_lines, true);
fclose(dxffile);
next_pnr = keeppoints.HighestPointNumber().Number()+1;
line_number3 = keeptops.size();
all_wall_lines.ReNumber(all_wall_points, next_pnr, line_number3);
keeptops.insert(keeptops.end(), all_wall_lines.begin(), all_wall_lines.end());
keeppoints.insert(keeppoints.end(), all_wall_points.begin(), all_wall_points.end());  
for (map_line = keeptops.begin(), index=0; map_line!=keeptops.end(); map_line++, index++){
    if (map_line->Attribute(LineLabelTag) != 5 && map_line->Attribute(LineLabelTag) != 0){
       map_line->Attribute(LineLabelTag) = 1; //set to zero to get green roof faces...
    }
}

keeptops.MakeCounterClockWise(keeppoints);

dxffile = fopen ("dxffile_modeldriven.dxf","w");
keeppoints.WriteDXF(dxffile, keeptops, true);
fclose(dxffile);

keeppoints.Write("modeldrivenmodel.objpts");
keeptops.Write("modeldrivenmodel.top", false);
fclose(statfile);
keeptops.ReNumber(keeppoints, 0, 0);
for (map_line = keeptops.begin(), index=0; map_line!=keeptops.end(); map_line++, index++){
    if (map_line->Attribute(LineLabelTag) == 5){
                                          verticalroofparts.push_back(*map_line);
                                          keeptops.erase(map_line);
                                          map_line--;
    }
    if (map_line->Attribute(LineLabelTag) == 0){
                                          keeptops.erase(map_line);
                                        map_line--;
    }

//    else (map_line->Attribute(LineLabelTag) = 0); //set to zero to get red roof faces...
}
//keeptops.SetAttribute(LineLabelTag, 2);
dxffile = fopen ("modeldrivenroofs.dxf","w");
keeppoints.WriteDXF(dxffile, keeptops, true);
fclose(dxffile);
dxffile = fopen ("modeldrivenverticalroofparts.dxf","w");
keeppoints.WriteDXF(dxffile, verticalroofparts, true);
fclose(dxffile);
//return;
//keeptops.SetAttribute(LineLabelTag, 0);  

  
}
