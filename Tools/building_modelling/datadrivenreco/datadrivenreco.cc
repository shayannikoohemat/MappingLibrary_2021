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
 Date   : 19-09-2008

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
#include <LineSegment2D.h> 



/*
--------------------------------------------------------------------------------
                           The main function
--------------------------------------------------------------------------------
*/

void datadrivenreco(char *map_points_input, char *map_topology_input,
                      char *matched_intersection_points, char *matched_intersection_lines,
                    char *matchresults_topology_input, char *laser_input, 
                    char *map_points_output, char *map_topology_output, char *infofile)

{
  ObjectPoints                 map_points, targetshape_points, intsect_points, seltargetpoints,
                               sel_match_points, new_points, stepedgepoints, densifiedmappoints,
                               allout3dobj, out3dobj, temp_points, nearbymappoints,
                               sel_map_points, all_flat_building_points,
                               upper_points, dumendpoints, all_flat_partition_points, allsplittedpoints, polygonpoints,
                               allpoints, insidepoints, wallinsidepoints, sel_inside_points,
                               newsplittedpoints;
  LineTopologies               map_lines, sel_map_lines, targetshape_lines, intsect_lines,
                               matchresults, sel_matchresults, scoresel_matchresults, 
                               sel_targetshape_lines, sel_targetshape_linespoly,
                               sel_matchresultspertarget, sel_int_lines,
                               sel_int_lines2, targetint_lines, seltargetint_lines,
                               selsegshapelines, segshapelines, new_lines, copytarget_lines, 
                               combinedlines, notusedlines, dormerlines, snappedstepedgelines,
                               newsnappedstepedgelines, densifiedmaplines, flatroofs, selstepedges,
                               out3dtops, allout3dtop, selmapsegments, shortmergedsegmentlines, hyposforsegment, hyposforbuilding,
                               all_flat_building_lines, map_tin_lines, one_map_line, wall_map_lines,
                               upperlines, selcounterlines2, topstepedge, splittedlines, 
                               allsplittedlines, selsplittedlines, notyetallsplittedlines,
                               all_flat_partition_lines, sel_combinedlines, insertsplittedlines,
                               dormerlinestofitto, alllines, sel_mapresults2, sel_combinedlines2, insidelines,
                               sel_matchresults2, wallinsidelines, temp_lines,shapetops;
  LineTopologies::iterator     map_line,map_line2, sel_map_line, sel_target_line, sel_match_line, sel_match_line2,
                               target_line, notusedline, hypoforsegment, hypoforbuilding,
                               selnearbymapline, concave, partition, partition2, combline, face;
  LineTopology::iterator       node, nb_node, node2, prev_node, next_node;
  LineTopology                 matchedline, targetline, polygonline, snapline, out3dtop, top,
                               nearbymapline, insideline, *outside;
  LaserPoints                  laser_points, sel_laser_points, linehypotheses, checkcornerpoints, 
                               buildingcheckcornerpoints, keepcheckcornerpoints, keepbuildingcheckcornerpoints,
                               waswordts, waswordtsintargets, keepwaswordtsintargets,
                               wasnodewordtsegment, notused, extendfacepoints, newextendfacepoints,
                               fix_laser_points, shortlinepoints, selsnappoints, extendridgepoints,
                               stepedgemaplaserpoints, laser_points_leftover, partlaserpoints, 
                               selpartlaserpoints, partitionedlaserdata, pointsofnotusedsegments, dominantlabelpoints,
                               sel_laser_points2;
  LaserPoint                   linehypothese, checkcornerpoint, quadpoint, waswordt, extendfacepoint,
                               newextendfacepoint, maplaserpoint, shortlinepoint,
                               selsnappoint, extendridgepoint, lowestlaserpoint;
  LaserPoints::iterator        laser_point, check_point, next_point, laser_point2;
  LineNumber                   findline;
  int                          i, j, size1, diff, index_line, number_offset2, maybequad,
                               dormpoint1, dormpoint2, stepedgepoint1, stepedgepoint2, 
                               num1, num2, lastsegmentnumber, next_pnr, line_number3, 
                               cost, highestcov, target, building, count, iter,
                               searchdist, hundredangle, linenumber, hundreddistance, takendist, iter2, iter3,
                               number_offset, intfindline, count2, count3, label2, dominantsegment2, dumpointnumber,
                               seg1, seg2, maxcost, loop, index1, index2, buildingnr, lastbuildingnr, dominantlabel, dominantlabelsegment, index,
                               success, b1, b2, keephighnumber, number_offseti, segnr, nearestpoint;
  vector <int>                 target_numbers, scores, segment_numbers, matchedsegment_numbers, corner, quadsegments, mapnodenumbers,
                               flat_roofs, lowsegment_numbers, segment_numbers_model, low_segments, buildingnumbers,
                               segment_numbers_partitions,buildingnumbersaf;
  vector <int>::iterator       target_number, score, segment_number, msegment_number, mapnodenumber,
                               flat_roof, lowsegment_number, segment_number_m;
  FILE                         *statfile, *dxffile;
  bool                         triconnection, quadconnection, dormerreco, found, 
                               nothingchanged, foundprevnode, foundnode, shortline, changedheight,
                               buildingheightfound, hypoheightfound, oke, usedominantlabel, keep;
  Plane                        planecorner1, planecorner2, planecorner3, lowerplane, plane;
  Line3D                       linecorner;
  Vector3D                     normalpolygon;
  TIN                          tin;
  TINEdges                     edges;
  Position3D                   pos1, pos2, newpos, dumpos3d, projpos3d, pos23d, suppos1, 
                               suppos2, projpos1, projpos2, dumpos13d, dumpos23d, keeppos1, keeppos2,
                               projposonmap1, nbmppos13d, nbmppos23d, newpos2, pos, projpos;
  ObjectPoint                  cornerpoint, new_point, newedgepoint, oldpoint, centrepoint,
                               nbmpoint1, nbmpoint2, point;
  double                       xt1, xt2, yt1, yt2, xb1, xb2, yb1, yb2, dist, mindist, totdist,
                               perc0, perc5, dx, dy, dz, factor, minz, maxz, goal_z, goaldiff,
                                PI, angleplane, angleline, hypogoal_z, buildinggoal_z, anglenbmline,
                               scalar, scalar3d, nbmlength, linelength, perc100, 
                               floor_height, maxcostd, costd, mapheight, pdist, prevheight, maxdist,
                               residual, keepresidual;
  ObjectPoints::iterator       map_point, prev_point, this_point;
  LineTopologies               mergedsegmentlines, selmergedsegmentlines, goonwiththisone, concavelines,
                               selacceptedlines, selacceptedlines2, selacceptedlines3, counterlines,
                               polygonlines, selsegshapelinespoly, selacceptedlines4, 
                               selacceptedlines5, selacceptedlines6, selacceptedlines7, 
                               selacceptedlines8, stepedgelines,
                               selcounterlines, nearbymaplines, selnearbymaplines, shapestepedgelines, selmergedsegmentlines2,
                               sel_mapresults, ridgelines;
  Line2D                       line, perpline, nbmline;
  Position2D                   pos12d, pos22d, dumpos1, dumpos2, supdumpos1, supdumpos2,
                               nbmppos12d, nbmppos22d, projposonmap2d, pos32d, pos42d, dumposend;
  Line3D                       line3d, supline1, supline2, line3d2, nbmline3d;
  LaserPoints                  seg_laser_points, maplaserpoints, selmaplaserpoints, segmaplaserpoints;
  PointNumberList              pnl, component;
   ObjectPoints                 endpoints;
   ObjectPoints::iterator       endpoint, endpoint2, endpoint3, dumpoint;
   ObjectPoint                  defpoint, first_point, sec_point;

bool snapintersectionlines2mapline;
snapintersectionlines2mapline = false;

int max_distance_line2map, max_distance_assignpoints2map;
max_distance_line2map = 0;
max_distance_assignpoints2map = 3;



  // Read input data
  if (!map_points.Read(map_points_input)) {
    printf("Error reading map points from file %s\n", map_points_input);
    exit(0);
  }
  if (!map_lines.Read(map_topology_input)) {
    printf("Error reading map lines from file %s\n", map_topology_input);
    exit(0);
  }
  if (!intsect_points.Read(matched_intersection_points)) {
    printf("Error reading map points from file %s\n", matched_intersection_points);
    exit(0);
  }
  
  number_offset2 = intsect_points.HighestPointNumber().Number()+1;
  
  if (!intsect_lines.Read(matched_intersection_lines)) {
    printf("Error reading map lines from file %s\n", matched_intersection_lines);
    exit(0);
  }
  if (!matchresults.Read(matchresults_topology_input)) {
    printf("Error reading map lines from file %s\n", matchresults_topology_input);
    exit(0);
  }

  if (!targetshape_points.Read("targetshapes.objpts")) {
    printf("Error reading target shape points from file\n");
    exit(0);
  }
  if (!targetshape_lines.Read("targetshapes.top")) {
    printf("Error reading target shape lines from file\n");
    exit(0);
  }
  if (!targetint_lines.Read("targetintsectlines4.top")) {
    printf("Error reading target intsectlines from file\n");
    exit(0);
  }
  
    if (!segshapelines.Read("selshapelines.top")) {
    printf("Error reading selshapelines from file\n");
    exit(0);
  }
  if (!stepedgepoints.Read("fittedstepedgelines.objpts")) {
    printf("Error reading stepedge points from file\n");
    exit(0);
  }
  if (!stepedgelines.Read("fittedstepedgelines.top")) {
    printf("Error reading step edge lines from file\n");
    exit(0);                
}

  
  
  wasnodewordtsegment.Read("wasnodewordtsegment.laser");
  laser_points.Read(laser_input);
    statfile = fopen(infofile,"w");
    
    // test for complex flat roof buildings...
  /*  
    for (map_line = map_lines.begin(); map_line!=map_lines.end(); map_line++){
         sel_int_lines = intsect_lines.SelectAttributedLines(BuildingNumberTag, map_line->Number());
         sel_matchresults = matchresults.SelectAttributedLines(BuildingNumberTag, map_line->Number());
         sel_laser_points.ErasePoints();
         sel_laser_points = laser_points.SelectTagValue(PolygonNumberTag, map_line->Number());
         segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
         segment_numbers = sel_laser_points.AttributeValues(SegmentNumberTag);
         scores = sel_matchresults.AttributedValues(MatchResultTag);
         sel_matchresults2 = sel_matchresults.SelectAttributedLines(MatchResultTag, 0);
         if (sel_matchresults2.size() == 0 && sel_laser_points.size()>0){ //check if flat roof
             printf("possible complex structure in map line %d\n", sel_match_line->Number());
              }
         for (sel_match_line = sel_matchresults2.begin(); sel_match_line !=sel_matchresults2.end(); sel_match_line++){
            if (sel_match_line->Attribute(TargetNumberTag)==50){
             printf("complex structure in map line %d\n", sel_match_line->Number());
             }
             } 
}
    
    
    
    */// end of test complext flat roof buildings
    
    for (map_line = map_lines.begin(); map_line!=map_lines.end(); map_line++){
         sel_int_lines = intsect_lines.SelectAttributedLines(BuildingNumberTag, map_line->Number());
         sel_matchresults = matchresults.SelectAttributedLines(BuildingNumberTag, map_line->Number());
         sel_laser_points.ErasePoints();
         sel_laser_points = laser_points.SelectTagValue(PolygonNumberTag, map_line->Number());
         segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
         segment_numbers = sel_laser_points.AttributeValues(SegmentNumberTag);
 //        matchedsegment_numbers.erase(matchedsegment_numbers.begin(), matchedsegment_numbers.end());
         sel_matchresults2 = sel_matchresults.SelectAttributedLines(MatchResultTag, 0);
         sel_matchresults = sel_matchresults2;
         scores = sel_matchresults.AttributedValues(MatchResultTag);
         sort(scores.begin(), scores.end());
         
         for (score = scores.begin(); score!=scores.end(); score++){
           scoresel_matchresults = sel_matchresults.SelectAttributedLines(MatchResultTag, *score);
           target_numbers = scoresel_matchresults.AttributedValues(TargetNumberTag);
           for (target_number = target_numbers.begin(); target_number!=target_numbers.end(); target_number++){ // for every target in the selection
            sel_targetshape_lines = targetshape_lines.SelectAttributedLines(TargetNumberTag, *target_number);
            fprintf(statfile, "\ntarget %d, size %d ", *target_number, sel_targetshape_lines.size());
            if (sel_targetshape_lines.size()==0) fprintf (statfile, "\nNo targetshape lines found for target %d\n", *target_number);
            sel_targetshape_linespoly.erase(sel_targetshape_linespoly.begin(), sel_targetshape_linespoly.end());
            seltargetint_lines = targetint_lines.SelectAttributedLines(TargetNumberTag, *target_number);
            seltargetpoints.erase(seltargetpoints.begin(), seltargetpoints.end());
            checkcornerpoints.ErasePoints();
            for (sel_target_line = seltargetint_lines.begin(); sel_target_line !=seltargetint_lines.end(); sel_target_line++){
                fprintf(statfile, "line %d ", sel_target_line->Number());
                for(node = sel_target_line->begin(); node!=sel_target_line->end(); node++){
                         seltargetpoints.push_back(*(targetshape_points.PointIterator(*node)));
                         fprintf(statfile, "point %d ", node->Number());
                         checkcornerpoint.X() = node->Number();
                         checkcornerpoint.Y() = sel_target_line->Number();
                         checkcornerpoint.Z() = 0;
                         checkcornerpoints.push_back(checkcornerpoint);
                }
            }
            checkcornerpoints.SortOnCoordinates();
            keepcheckcornerpoints.ErasePoints();
            for (check_point = checkcornerpoints.begin(), next_point = checkcornerpoints.begin()+1; check_point != checkcornerpoints.end()-1; check_point++, next_point++){
                if (check_point->X() == next_point->X()){
                    check_point->Z() = next_point->Y();
                    fprintf (statfile, "\ntargetcornerpoint detected: %d on line %d (& %d)\n", int(check_point->X()), int(check_point->Y()), int(check_point->Z()));
                    keepcheckcornerpoints.push_back(*check_point);
                    maybequad = int (check_point->X());
                }
            }
            size1 = seltargetpoints.size();
            fprintf(statfile, "size1 is %d ", size1);
            seltargetpoints.RemoveDoublePoints(seltargetint_lines, 0.01);
            diff = size1-seltargetpoints.size();
            
            triconnection = false;
            quadconnection = false;
            bool detail = false;     
            // selection made next on: shape targets, only polygons of shape targets are selected
            for (sel_target_line = sel_targetshape_lines.begin(); sel_target_line !=sel_targetshape_lines.end(); sel_target_line++){
               if (sel_target_line->IsClosed() && sel_target_line->HasAttribute(NodeNumberTag) && sel_target_line->HasAttribute(TargetType) && sel_target_line->Attribute(TargetType)==ShapeTarget) sel_targetshape_linespoly.push_back(*sel_target_line);
//               if (sel_target_line->IsClosed() && sel_target_line->HasAttribute(NodeNumberTag)) sel_targetshape_linespoly.push_back(*sel_target_line);
    //soe270309             if (sel_target_line->Attribute(TargetType)==DetailTarget) detail =true;
    //              if (sel_target_line->Attribute(TargetType)==DetailTarget) detail =true;
                   }
            
            // continue with only the (perfect, score =0) targets for the moment...
            if (sel_targetshape_linespoly.size()>=0 && *score == 0 && !detail){
              
              sel_matchresultspertarget = scoresel_matchresults.SelectAttributedLines(TargetNumberTag, *target_number);
              fprintf(statfile, "\nBuilding %d, target %d (%d faces, %d lines, %d points), size of matched hypos %d, score %d\n", map_line->Number(), *target_number, sel_targetshape_linespoly.size(), seltargetint_lines.size(), seltargetpoints.size(), sel_matchresultspertarget.size(), *score);
              fprintf(statfile, "Diff is %d ", diff);
              if (diff == 2) triconnection = true, fprintf(statfile, "triconnection\n");
              if (diff == 3) quadconnection = true, fprintf(statfile, "quadconnection\n");
              if (diff == 4) triconnection = true, fprintf(statfile, "double triconnection\n");
              for (sel_target_line = sel_matchresultspertarget.begin(); sel_target_line!=sel_matchresultspertarget.end(); sel_target_line++){
                  copytarget_lines = seltargetint_lines;
                  sel_int_lines2 = sel_int_lines.SelectAttributedLines(HypoNumberTag, sel_target_line->Number());
                  sel_match_points.erase(sel_match_points.begin(), sel_match_points.end());
                  buildingcheckcornerpoints.ErasePoints();
     //             waswordtsintargets.ErasePoints();
                  for (sel_match_line = sel_int_lines2.begin(); sel_match_line !=sel_int_lines2.end(); sel_match_line++){
                       for(node = sel_match_line->begin(); node!=sel_match_line->end(); node++){
                           sel_match_points.push_back(*(intsect_points.PointIterator(*node)));
                           checkcornerpoint.X() = node->Number();
                          checkcornerpoint.Y() = sel_match_line->Attribute(CorrespondingTargetLineNumber);
                          checkcornerpoint.Z() = 0;
                          buildingcheckcornerpoints.push_back(checkcornerpoint);
                       }
                       if (triconnection) sel_match_line->Label() = 3, new_lines.push_back(*sel_match_line);
                       if (!triconnection && !quadconnection) sel_match_line->Label() = 2, new_lines.push_back(*sel_match_line);
                       
                  }
                  buildingcheckcornerpoints.SortOnCoordinates();
                  keepbuildingcheckcornerpoints.ErasePoints();
                  for (check_point = buildingcheckcornerpoints.begin(), next_point = buildingcheckcornerpoints.begin()+1; check_point != buildingcheckcornerpoints.end()-1; check_point++, next_point++){
                       if (check_point->X() == next_point->X()){
                           check_point->Z() = next_point->Y();
                           fprintf (statfile, "\nbuildingcornerpoint detected: %d on line %d (& %d)\n", int(check_point->X()), int(check_point->Y()), int(check_point->Z()));
                           keepbuildingcheckcornerpoints.push_back(*check_point);
                       }
                  }
                  for (check_point = keepcheckcornerpoints.begin(); check_point != keepcheckcornerpoints.end(); check_point++){
                      for (next_point = keepbuildingcheckcornerpoints.begin(); next_point != keepbuildingcheckcornerpoints.end(); next_point++){
                       if (check_point->Y() == next_point->Y() && check_point->Z() == next_point->Z()){
                           fprintf (statfile, "\nis this a match? %d %d %d - %d %d %d \n", int(check_point->X()), int(check_point->Y()),int(check_point->Z()),  int(next_point->X()),int(next_point->Y()),int(next_point->Z()));
                           waswordt.X() = int(check_point->X());
                           waswordt.Y() = int(next_point->X());
                           waswordt.Z() = int(sel_target_line->Number());//hypo nr
                           waswordtsintargets.push_back(waswordt);
                       }
                      }
                  }
                  sel_match_points.RemoveDoublePoints(sel_int_lines2, 0.01);
                  fprintf(statfile, "hyponr %d, %d segments, %d lines, %d points\n", sel_target_line->Number(), sel_target_line->size(), sel_int_lines2.size(), sel_match_points.size());
                  for (sel_match_line = sel_int_lines2.begin(); sel_match_line !=sel_int_lines2.end(); sel_match_line++){
                      if (sel_match_line->HasAttribute(CorrespondingTargetLineNumber)) fprintf(statfile, " line: %d (%d)", sel_match_line->Number(), sel_match_line->Attribute(CorrespondingTargetLineNumber));
                      else fprintf(statfile, " line: %d (%d)", sel_match_line->Number(), -1);
                      linehypothese.X()=sel_match_line->Number();
                      linehypothese.Y()=sel_match_line->Attribute(CorrespondingTargetLineNumber);
                      linehypothese.Z()=sel_target_line->Number();
                      linehypotheses.push_back(linehypothese);
                      
                      }
                      quadsegments.erase(quadsegments.begin(), quadsegments.end());
                 for(node = sel_target_line->begin(); node!=sel_target_line->end(); node++){
                          sel_laser_points.ErasePoints();
                          sel_laser_points.AddTaggedPoints(laser_points, node->Number(), SegmentNumberTag);
                         fprintf(statfile, "matseg %d (size = %d) ", node->Number(), sel_laser_points.size());
                         matchedsegment_numbers.push_back(node->Number());
                         quadsegments.push_back(node->Number());
                 }
                 bool actasquad = false;
          //       if (triconnection && sel_match_points.size()>4 && diff == 2) actasquad = true;
                 sort(quadsegments.begin(), quadsegments.end()); // the first three segments are the biggest...
                 if ((quadconnection || actasquad) && quadsegments.size()>2){
                        planecorner1 = laser_points.FitPlane(quadsegments[0], quadsegments[0], SegmentNumberTag);
                        planecorner2 = laser_points.FitPlane(quadsegments[1], quadsegments[1], SegmentNumberTag);
                        planecorner3 = laser_points.FitPlane(quadsegments[2], quadsegments[2], SegmentNumberTag);
                        fprintf(statfile, "\n");
                        if (Intersect2Planes(planecorner1, planecorner2, linecorner)) {
                           if (IntersectLine3DPlane(linecorner, planecorner3, cornerpoint)){
                               quadpoint = LaserPoint(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z(), 1, 1);
                               new_point = cornerpoint;
                               bool prevpointfound = false;
                               for (map_point = new_points.begin(); map_point!=new_points.end(); map_point++){
                                   if ((map_point->Position3DRef()-cornerpoint.Position3DRef()).Length()<0.01){
                                        new_point.Number() = map_point->Number();
                                        prevpointfound = true;
                                   }
                               }
                               if (!prevpointfound) {
                                                    new_point.Number() = number_offset2;
                                                    number_offset2++;
                                                    new_points.push_back(new_point);
                                                    } 
                               //} 
                              // }
                           }
                        }
                        for (sel_match_line = sel_int_lines2.begin(); sel_match_line !=sel_int_lines2.end(); sel_match_line++){
                           node=sel_match_line->begin();
                           pos1 = intsect_points.PointIterator(*node)->Position3DRef(); node++;
                           pos2 = intsect_points.PointIterator(*node)->Position3DRef();    
                           if ((pos1 - quadpoint.Position3DRef()).Length()<(pos2 - quadpoint.Position3DRef()).Length()){
                                     fprintf(statfile, "POS1 closer to quadpoint (%4.2f)\n", (pos1 - quadpoint.Position3DRef()).Length());
                                     node--;
                                     waswordt.X() = node->Number();
                                     node->Number()=new_point.Number();
                                     waswordt.Y() = node->Number();
                                     waswordts.push_back(waswordt);
                                     waswordt.X() = maybequad;
                                     waswordt.Z() = int(sel_target_line->Number());//hypo nr
                                     waswordtsintargets.push_back(waswordt);
                                     }
                           else {
                                 fprintf(statfile, "POS2 closer to quadpoint (%4.2f)\n", (pos2 - quadpoint.Position3DRef()).Length());
                                     waswordt.X() = node->Number();
                                     node->Number()=new_point.Number();
                                     waswordt.Y() = node->Number();
                                     waswordts.push_back(waswordt);
                                     waswordt.Z() = int(sel_target_line->Number());//hypo nr
                                     waswordt.X() = maybequad;
                                     waswordtsintargets.push_back(waswordt);
                                }
                           if (quadconnection) sel_match_line->Label() = 4;
                           if (triconnection) sel_match_line->Label() = 3;
                           new_lines.push_back(*sel_match_line);
                        }
                   }
              fprintf(statfile, "\n");
              }
              for (sel_target_line = sel_targetshape_linespoly.begin(); sel_target_line!=sel_targetshape_linespoly.end(); sel_target_line++){
                         fprintf(statfile, "nod %d ", sel_target_line->Attribute(NodeNumberTag));
                }
              fprintf(statfile, "\n");
            }
         }
         }
         fprintf(statfile, "building %d:\n", map_line->Number());
         for (segment_number = segment_numbers.begin(); segment_number!=segment_numbers.end(); segment_number++){
             fprintf(statfile, "seg %d ", *segment_number);
             bool used = false;
             for (msegment_number = matchedsegment_numbers.begin(); msegment_number!=matchedsegment_numbers.end(); msegment_number++){
                 if (*msegment_number==*segment_number) used = true;
             }
             if (!used) {
                        fprintf(statfile, "(not used) ");
                        notused.AddTaggedPoints(sel_laser_points, *segment_number, SegmentNumberTag);
                        fprintf(statfile, "Segment %d added to notusedpoints\n ", *segment_number);
                        notusedlines = sel_matchresults.ReturnLinesWithPoint(PointNumber (*segment_number));
                        for (notusedline = notusedlines.begin(); notusedline!=notusedlines.end(); notusedline++){
                            fprintf(statfile, "segment in hypo %d (target %d), with score %d and coverage %d\n", notusedline->Number(), notusedline->Attribute(TargetNumberTag), notusedline->Attribute(MatchResultTag), notusedline->Attribute(CoverageTag));
                            }
                        }
          }
          fprintf(statfile, "\n");
        
        
    }
    notused.RemoveTaggedPoints(1500, LabelTag);
    notused.RemoveTaggedPoints(1600, LabelTag);
    notused.RemoveTaggedPoints(1000, LabelTag);
    notused.Write("laserpointsnotused.laser", false);
    num1 = (notused.AttributeValues(SegmentNumberTag)).size();
    segment_numbers = laser_points.AttributeValues(SegmentNumberTag);
    fprintf(statfile,"# segments not used: %d (out of %d)\n", num1, segment_numbers.size());
    //laser_points_leftover.Write("laserpointsleftover2.laser", false);
     buildingnumbers = notused.AttributeValues(PolygonNumberTag);
     b1 = buildingnumbers.size();
     buildingnumbers = laser_points.AttributeValues(PolygonNumberTag);
     b2 = buildingnumbers.size();
     fprintf(statfile,"\n***\nNumber of buildings affected of notused points %d, out of %d\n", b1, b2);

     buildingnumbersaf = notused.AttributeValues(PolygonNumberTag);
LaserPoints affectedbuildingpoints;
//test to write affected buildings instead of points
     for (segment_number = buildingnumbers.begin(); segment_number !=buildingnumbers.end(); segment_number++){
     sel_laser_points.ErasePoints();
     sel_laser_points = notused.SelectTagValue(PolygonNumberTag, *segment_number);
     if (sel_laser_points.size()>10){
     sel_laser_points = laser_points.SelectTagValue(PolygonNumberTag, *segment_number);                                     
     for (laser_point=sel_laser_points.begin(); laser_point!=sel_laser_points.end(); laser_point++) {
         laser_point->Residual()=1;
         affectedbuildingpoints.push_back(*laser_point);
     }
     }
     else {
          sel_laser_points = laser_points.SelectTagValue(PolygonNumberTag, *segment_number);
          for (laser_point=sel_laser_points.begin(); laser_point!=sel_laser_points.end(); laser_point++) {
                laser_point->Residual()=0;
                affectedbuildingpoints.push_back(*laser_point);
          }
      }
      }         
          
     affectedbuildingpoints.Write("affectedbuildingpoints.laser", false);
     
     affectedbuildingpoints.ErasePoints();
     segment_numbers = laser_points.AttributeValues(SegmentNumberTag);
       for (segment_number = segment_numbers.begin(); segment_number !=segment_numbers.end(); segment_number++){
     sel_laser_points.ErasePoints();
     sel_laser_points = notused.SelectTagValue(SegmentNumberTag, *segment_number);
     if (sel_laser_points.size()>0){
     for (laser_point=sel_laser_points.begin(); laser_point!=sel_laser_points.end(); laser_point++) {
         laser_point->Residual()=1;
         affectedbuildingpoints.push_back(*laser_point);
     }
     }
     else {
          sel_laser_points = laser_points.SelectTagValue(SegmentNumberTag, *segment_number);
          for (laser_point=sel_laser_points.begin(); laser_point!=sel_laser_points.end(); laser_point++) {
                laser_point->Residual()=0;
                affectedbuildingpoints.push_back(*laser_point);
          }
      }
      }         
          
     affectedbuildingpoints.Write("affectedsegmentpoints.laser", false);
   
        
     
//return; //soe print above statistics...

    new_points.SortOnCoordinates();
    fprintf(statfile,"size of new_points %d\n", new_points.size());
    if (new_points.size()>1){
     prev_point = new_points.begin();
     for (map_point = new_points.begin()+1; map_point!=new_points.end(); map_point++){
         if ((map_point->Position3DRef()-prev_point->Position3DRef()).Length()<0.01){
           fprintf(statfile,"%d and %d have same position\n", prev_point->Number(), map_point->Number());
         }
        prev_point = map_point;
    }
    }          
    intsect_points.insert(intsect_points.end(), new_points.begin(), new_points.end());
    
    intsect_points.Write(map_points_output);
    for (sel_match_line = new_lines.begin(); sel_match_line !=new_lines.end(); sel_match_line++){
     for (node=sel_match_line->begin(); node!=sel_match_line->end();node++){
       for (check_point = waswordts.begin(); check_point != waswordts.end(); check_point++){
           if (node->Number()==int(check_point->X())) node->Number() = int(check_point->Y());
       }
     }
    }
    LineTopologies keep_new_lines;
    keep_new_lines = new_lines;
    
    for (sel_match_line = new_lines.begin(); sel_match_line !=new_lines.end(); sel_match_line++){
        for (sel_match_line2 = keep_new_lines.begin(); sel_match_line2 !=keep_new_lines.end(); sel_match_line2++){
            if (sel_match_line->Number()==sel_match_line2->Number()){
               if (sel_match_line2->Label() < sel_match_line->Label()) {
                   keep_new_lines.erase(sel_match_line2);
                   sel_match_line2--;
               }
            }
        }
    }
  //  intsect_points.RemoveDoublePoints(keep_new_lines, 0.0
    linehypotheses.SortOnCoordinates();
    for (laser_point2=waswordtsintargets.begin(); laser_point2!=waswordtsintargets.end(); laser_point2++){
       for (check_point = waswordts.begin(); check_point != waswordts.end(); check_point++){
   //     if (int(laser_point2->Y())==int(check_point->X())) laser_point2->Y() = int(check_point->Y());
       }
    }
    printf("1 ");
//    waswordtsintargets.ReduceData(0.5);
    keepwaswordtsintargets = waswordtsintargets;
    bool fixedwaswordt;
    printf("2           ");

   for (laser_point=linehypotheses.begin(); laser_point!=linehypotheses.end(); laser_point++){
       fprintf(statfile, "line %d with targetline %d hyponr %d\n", int(laser_point->X()), int(laser_point->Y()), int(laser_point->Z()));
       findline = LineNumber(int(laser_point->Y()));
       index_line = targetint_lines.FindLine(findline);
       targetline = targetint_lines[index_line];
       findline = LineNumber(int(laser_point->X()));
       index_line = keep_new_lines.FindLine(findline);
       matchedline = keep_new_lines[index_line];
       node = targetline.begin();
       node2 = matchedline.begin();
       fixedwaswordt = false;
       for (laser_point2=waswordtsintargets.begin(); laser_point2!=waswordtsintargets.end(); laser_point2++){
            if (laser_point2->Z()==laser_point->Z()&& node->Number()==int(laser_point2->X()) && node2->Number()==int(laser_point2->Y())){ 
               fprintf(statfile, "target point %d correspond with object point %d in hypo %d\n", int(laser_point2->X()), int(laser_point2->Y()),int(laser_point2->Z()));
               node++; node2++;
               fprintf(statfile, "left over: target point %d correspond with object point %d in hypo %d\n", node->Number(), node2->Number(),int(laser_point2->Z()));
               waswordt.X()=node->Number();
               waswordt.Y()=node2->Number();
               waswordt.Z()=laser_point2->Z();
               keepwaswordtsintargets.push_back(waswordt);
               node--; node2--;
               fixedwaswordt = true;
             }
       }
       node2++;
       for (laser_point2=waswordtsintargets.begin(); laser_point2!=waswordtsintargets.end(); laser_point2++){
            if (laser_point2->Z()==laser_point->Z()&& node->Number()==int(laser_point2->X()) && node2->Number()==int(laser_point2->Y())){ 
               fprintf(statfile, "target point %d correspond with object point %d in hypo %d\n", int(laser_point2->X()), int(laser_point2->Y()),int(laser_point2->Z()));
               node++; node2--;
               fprintf(statfile, "left over: target point %d correspond with object point %d in hypo %d\n", node->Number(), node2->Number(),int(laser_point2->Z()));
               waswordt.X()=node->Number();
               waswordt.Y()=node2->Number();
               waswordt.Z()=laser_point2->Z();
               keepwaswordtsintargets.push_back(waswordt);
               node--; node2++;
               fixedwaswordt = true;
             }
       }
       node++;node2--;
       for (laser_point2=waswordtsintargets.begin(); laser_point2!=waswordtsintargets.end(); laser_point2++){
            if (laser_point2->Z()==laser_point->Z()&& node->Number()==int(laser_point2->X()) && node2->Number()==int(laser_point2->Y())){ 
               fprintf(statfile, "target point %d correspond with object point %d in hypo %d\n", int(laser_point2->X()), int(laser_point2->Y()),int(laser_point2->Z()));
               node--; node2++;
               fprintf(statfile, "left over: target point %d correspond with object point %d in hypo %d\n", node->Number(), node2->Number(),int(laser_point2->Z()));
               waswordt.X()=node->Number();
               waswordt.Y()=node2->Number();
               waswordt.Z()=laser_point2->Z();
               keepwaswordtsintargets.push_back(waswordt);
               node++; node2--;
               fixedwaswordt = true;
             }
       }
       node2++;
       for (laser_point2=waswordtsintargets.begin(); laser_point2!=waswordtsintargets.end(); laser_point2++){
            if (laser_point2->Z()==laser_point->Z()&& node->Number()==int(laser_point2->X()) && node2->Number()==int(laser_point2->Y())){ 
               fprintf(statfile, "target point %d correspond with object point %d in hypo %d\n", int(laser_point2->X()), int(laser_point2->Y()),int(laser_point2->Z()));
               node--; node2--;
               fprintf(statfile, "left over: target point %d correspond with object point %d in hypo %d\n", node->Number(), node2->Number(),int(laser_point2->Z()));
               waswordt.X()=node->Number();
               waswordt.Y()=node2->Number();
               waswordt.Z()=laser_point2->Z();
               keepwaswordtsintargets.push_back(waswordt);
               node++; node2++;
               fixedwaswordt = true;
             }
       }
      if (!fixedwaswordt) { //if points are not fixed by more than one line, see if we can make a logic assignment
                          // this is neccessary because faces can have more than one lines that are not connected (mansard roof faces for example)
                          // if we use the logic of the target, we get some logic in the points of the polygon around the segment.
                          
    /*           fprintf(statfile, "free choice of combining points\n"); 
               node--; node2--;
               fprintf(statfile, "free fix of target point %d with object point %d in hypo %d\n", node->Number(), node2->Number(),int(laser_point->Z()));
               waswordt.X()=node->Number();
               waswordt.Y()=node2->Number();
               waswordt.Z()=laser_point->Z();
               keepwaswordtsintargets.push_back(waswordt);
               node++; node2++;
               fprintf(statfile, "free fix of target point %d with object point %d in hypo %d\n", node->Number(), node2->Number(),int(laser_point->Z()));
               waswordt.X()=node->Number();
               waswordt.Y()=node2->Number();
               waswordt.Z()=laser_point->Z();
               keepwaswordtsintargets.push_back(waswordt);                          
      */       
               node--; node2--;
               xt1 = (targetshape_points.PointIterator(*node))->X();
               yt1 = (targetshape_points.PointIterator(*node))->Y();
               xb1 = (intsect_points.PointIterator(*node2))->X();
               yb1 = (intsect_points.PointIterator(*node2))->Y();
               node++; node2++;
               xt2 = (targetshape_points.PointIterator(*node))->X();
               xb2 = (intsect_points.PointIterator(*node2))->X();
               yt2 = (targetshape_points.PointIterator(*node))->Y();
               yb2 = (intsect_points.PointIterator(*node2))->Y();
               node--; node2--;
               if (yt1<yt2){
                  if (yb1<yb2){
                               waswordt.X()=node->Number();
                               waswordt.Y()=node2->Number();
                               waswordt.Z()=laser_point->Z();
                               keepwaswordtsintargets.push_back(waswordt);
                               node++; node2++;
                               waswordt.X()=node->Number();
                               waswordt.Y()=node2->Number();
                               waswordt.Z()=laser_point->Z();
                               keepwaswordtsintargets.push_back(waswordt);   
                               }
                  if (yb1>yb2){
                               node2++;
                               waswordt.X()=node->Number();
                               waswordt.Y()=node2->Number();
                               waswordt.Z()=laser_point->Z();
                               keepwaswordtsintargets.push_back(waswordt);
                               node2--; node++;
                               waswordt.X()=node->Number();
                               waswordt.Y()=node2->Number();
                               waswordt.Z()=laser_point->Z();
                               keepwaswordtsintargets.push_back(waswordt);
                               }
                  }
               if (yt1>yt2) {
                   if (yb1>yb2){
                               waswordt.X()=node->Number();
                               waswordt.Y()=node2->Number();
                               waswordt.Z()=laser_point->Z();
                               keepwaswordtsintargets.push_back(waswordt);
                               node++; node2++;
                               waswordt.X()=node->Number();
                               waswordt.Y()=node2->Number();
                               waswordt.Z()=laser_point->Z();
                               keepwaswordtsintargets.push_back(waswordt);   
                               }
                  if (yb1<yb2){
                               node2++;
                               waswordt.X()=node->Number();
                               waswordt.Y()=node2->Number();
                               waswordt.Z()=laser_point->Z();
                               keepwaswordtsintargets.push_back(waswordt);
                               node2--; node++;
                               waswordt.X()=node->Number();
                               waswordt.Y()=node2->Number();
                               waswordt.Z()=laser_point->Z();
                               keepwaswordtsintargets.push_back(waswordt);
                               }
                  }
                                                            
               //   }
               if (yt1 == yt2){
                   if (xt1<xt2){
                        if (yb1<yb2){
                               waswordt.X()=node->Number();
                               waswordt.Y()=node2->Number();
                               waswordt.Z()=laser_point->Z();
                               keepwaswordtsintargets.push_back(waswordt);
                               node++; node2++;
                               waswordt.X()=node->Number();
                               waswordt.Y()=node2->Number();
                               waswordt.Z()=laser_point->Z();
                               keepwaswordtsintargets.push_back(waswordt);   
                        }
                        if (yb1>yb2){
                               node2++;
                               waswordt.X()=node->Number();
                               waswordt.Y()=node2->Number();
                               waswordt.Z()=laser_point->Z();
                               keepwaswordtsintargets.push_back(waswordt);
                               node2--; node++;
                               waswordt.X()=node->Number();
                               waswordt.Y()=node2->Number();
                               waswordt.Z()=laser_point->Z();
                               keepwaswordtsintargets.push_back(waswordt);
                         }
                    }
                   if (xt1>xt2){
                       if (yb1>yb2){
                               waswordt.X()=node->Number();
                               waswordt.Y()=node2->Number();
                               waswordt.Z()=laser_point->Z();
                               keepwaswordtsintargets.push_back(waswordt);
                               node++; node2++;
                               waswordt.X()=node->Number();
                               waswordt.Y()=node2->Number();
                               waswordt.Z()=laser_point->Z();
                               keepwaswordtsintargets.push_back(waswordt);   
                       }
                       if (yb1<yb2){
                               node2++;
                               waswordt.X()=node->Number();
                               waswordt.Y()=node2->Number();
                               waswordt.Z()=laser_point->Z();
                               keepwaswordtsintargets.push_back(waswordt);
                               node2--; node++;
                               waswordt.X()=node->Number();
                               waswordt.Y()=node2->Number();
                               waswordt.Z()=laser_point->Z();
                               keepwaswordtsintargets.push_back(waswordt);
                        }
                     }                 
                  }
                                         
               
                 }
       }
   
   
    
  //  keepwaswordtsintargets.AddPoints(waswordtsintargets);                 
   // keepwaswordtsintargets.ReduceData(0.1);
    waswordtsintargets = keepwaswordtsintargets;
    
    keepwaswordtsintargets.ErasePoints();
    for (laser_point2=waswordtsintargets.begin(); laser_point2!=waswordtsintargets.end(); laser_point2++){
       for (check_point = waswordts.begin(); check_point != waswordts.end(); check_point++){
        if (int(laser_point2->Y())==int(check_point->X())) {
                                                           fprintf(statfile,"changed %d into %d, hypo %d\n", int(laser_point2->Y()), int(check_point->Y()), int(laser_point2->Z())); 
      //                                                     laser_point2->Y() = int(check_point->Y());
                                                           }
       }
       keepwaswordtsintargets.push_back(*laser_point2);
    }

   //   keepwaswordtsintargets.ReduceData(0.5);
    keep_new_lines.Write(map_topology_output, false);

bool assignmapsegments;
assignmapsegments = true;
if (assignmapsegments){
printf("start assigning segments to map data...\n");
double nbh_radius = 1;
int    dominantsegment, lastsegment, newpointnumber, success;
Position3D              lastpos, newpos;
LaserPoint                   buildingedgepoint;
LaserPoints                  edgebuildingpoints, temp_height_points, low_laser_points;
vector <int>                 lowsegments, selsegmentnumbers;
vector <int>::iterator       lowsegment;
bool                         used;

newpointnumber = intsect_points.HighestPointNumber().Number()+1;                           
densifiedmaplines = map_lines;
densifiedmappoints = map_points;
//densifiedmaplines.Densify(densifiedmappoints, 1);
densifiedmappoints.RemoveDoublePoints(densifiedmaplines, 0.1);
low_laser_points.ErasePoints();
low_laser_points = laser_points; //all points
lowsegments = stepedgelines.AttributedValues(SecondSegmentLabel);

for (map_line = densifiedmaplines.begin(); map_line!=densifiedmaplines.end(); map_line++){
    sel_laser_points.ErasePoints();
    selmaplaserpoints.ErasePoints();
    sel_laser_points = low_laser_points.SelectTagValue(PolygonNumberTag, map_line->Number());
    lastsegment = -1;
 
    for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
        new_point = *(densifiedmappoints.PointIterator(*node2));
        temp_height_points.ErasePoints();  
        newpos = (densifiedmappoints.PointIterator(*node2))->Position3DRef();
        nbh_radius = 1;
      do{
        found = false;
        for (laser_point=sel_laser_points.begin(); laser_point!=sel_laser_points.end(); laser_point++){
          dist = (laser_point->vect2D()-new_point.vect2D()).Length();   
            if (dist < nbh_radius) {
                temp_height_points.push_back(*laser_point);
                }
           }
        dominantsegment = temp_height_points.MostFrequentAttributeValue(SegmentNumberTag, count);
        selsegmentnumbers = temp_height_points.AttributeValues(SegmentNumberTag);
        for (segment_number = selsegmentnumbers.begin(); segment_number !=selsegmentnumbers.end(); segment_number++){
            used = false;
            for (msegment_number = matchedsegment_numbers.begin(); msegment_number!=matchedsegment_numbers.end(); msegment_number++){
                if (*msegment_number==*segment_number) used = true;
            }
            if (used) {  
             buildingedgepoint = LaserPoint(new_point.X(), new_point.Y(), new_point.Z());
             plane = laser_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
             buildingedgepoint.Z() = plane.Z_At(new_point.X(), new_point.Y(), &success);
             buildingedgepoint.SetAttribute(LabelTag, *segment_number);
             buildingedgepoint.SetAttribute(SegmentNumberTag, *segment_number);
             edgebuildingpoints.push_back(buildingedgepoint);
             maplaserpoint.X() = node2->Number();
             maplaserpoint.Y() = *segment_number;
             maplaserpoint.Z() = 0;
             maplaserpoint.SetAttribute(SegmentNumberTag, *segment_number);
             maplaserpoint.SetAttribute(LabelTag, node2->Number());
             selmaplaserpoints.push_back(maplaserpoint);
             found = true;
             }
        }
//    }
    nbh_radius = nbh_radius+1;
    } while (!found && nbh_radius < max_distance_assignpoints2map);
    }
    selsegmentnumbers = selmaplaserpoints.AttributeValues(SegmentNumberTag);
    for (segment_number = selsegmentnumbers.begin(); segment_number !=selsegmentnumbers.end(); segment_number++){
        segmaplaserpoints.ErasePoints();
        segmaplaserpoints.AddTaggedPoints(selmaplaserpoints, *segment_number, SegmentNumberTag);
        mapnodenumbers = segmaplaserpoints.AttributeValues(LabelTag);
        lowerplane = laser_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
        foundprevnode = false;
        for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
          foundnode = false;
          for (mapnodenumber = mapnodenumbers.begin(); mapnodenumber!=mapnodenumbers.end(); mapnodenumber++){
              if (node2->Number() == *mapnodenumber) {
                 foundnode = true;
                 newpos = (densifiedmappoints.PointIterator(*node2))->Position3DRef();
              }
          }
          newpos = (densifiedmappoints.PointIterator(*node2))->Position3DRef();
          if (foundnode && foundprevnode){
   //           if ((foundnode && node2!=map_line->begin()) || foundprevnode){
              top.clear(); top.Initialise();
              newpointnumber++;
              newedgepoint = ObjectPoint(lastpos.GetX(), lastpos.GetY(), lowerplane.Z_At(lastpos.GetX(), lastpos.GetY(), &success), newpointnumber, 0,0,0,0,0,0); 
              top.push_back(PointNumber(newpointnumber));
              nearbymappoints.push_back(newedgepoint);
              newpointnumber++;
              newedgepoint = ObjectPoint(newpos.GetX(), newpos.GetY(), lowerplane.Z_At(newpos.GetX(), newpos.GetY(), &success), newpointnumber, 0,0,0,0,0,0); 
              top.push_back(PointNumber(newpointnumber));
              nearbymappoints.push_back(newedgepoint);
              top.SetAttribute(SegmentLabel, *segment_number);
              top.SetAttribute(LineLabelTag, *segment_number);
              top.SetAttribute(BuildingNumberTag, map_line->Number());
              nearbymaplines.push_back(top); 
          }
          if (foundnode){
                         foundprevnode = true;
                         lastpos = newpos;
                         }
          else foundprevnode = false;
          lastpos = newpos;
        }
    }
        
}
printf("finished assigning segments to map data...\n");

edgebuildingpoints.Write("edgebuildingpoints.laser", false);
nearbymappoints.RemoveDoublePoints(nearbymaplines, 0.1);
nearbymappoints.Write("assignedmaplines.objpts");
nearbymaplines.Write("assignedmaplines.top", false);
}
if (!assignmapsegments){
nearbymappoints.Read("assignedmaplines.objpts");
nearbymaplines.Read("assignedmaplines.top");
}

bool mergesegmentinfo = true;

if (mergesegmentinfo){
        
   LineTopology                 edgeline, suptop1, suptop2;
   LineTopology::iterator       suppoint;
   int                          newpointnumber, success, lastlinenumber, segnr;
   bool                         extrapointadded, foundsegment, loweredsegment;
   PI = 3.14159; 
   lastsegmentnumber = -1;
   printf("start merging segments info ...\n");
   newpointnumber = intsect_points.HighestPointNumber().Number()+1;
   fprintf(statfile,"size of keepnewlines: %d \n", keep_new_lines.size());
    
   for (sel_match_line = segshapelines.begin(); sel_match_line !=segshapelines.end(); sel_match_line++){
     for (node=sel_match_line->begin(); node!=sel_match_line->end();node++){
       for (check_point = waswordts.begin(); check_point != waswordts.end(); check_point++){
           if (node->Number()==int(check_point->X())) node->Number() = int(check_point->Y());
       }
     }
     prev_node = sel_match_line->begin();
     node = sel_match_line->begin()+1;
     if (node->Number()==prev_node->Number()) {
                                              segshapelines.erase(sel_match_line);
                                              sel_match_line--;
                                              }
    }
    
   for (map_line = segshapelines.begin(); map_line!=segshapelines.end(); map_line++){
       findline = LineNumber(map_line->Attribute(LineNumberTag));
       fprintf(statfile, "look for linenumber %d segment %d\n", map_line->Attribute(LineNumberTag), map_line->Attribute(SegmentLabel));
       index_line = keep_new_lines.FindLine(findline);
       if (index_line==-1) fprintf(statfile, "NOT FOUND, not accepted in matching\n");
       foundsegment = false;
       for (segment_number = matchedsegment_numbers.begin(); segment_number!=matchedsegment_numbers.end(); segment_number++){
           if (map_line->Attribute(SegmentLabel)==*segment_number) foundsegment = true;
       }

       // each line in segshapelines is double; one for each segment. if both segments are in matchedsegment numbers, and have a concave intersection that has not been accepted for any reason, still add this line...       
       if (index_line==-1 && foundsegment && map_line->Attribute(LineNumberTag) == lastlinenumber) { // line not in accepted lines... segment found .. and first segment also found 
         if (map_line->Attribute(LineLabelTag)==9){ // line concave line
            map_line--;
            seg1 = map_line->Attribute(SegmentLabel);
            map_line++;
            seg2 = map_line->Attribute(SegmentLabel);
//            if (seg1>seg2) 
            map_line--; //seg1 has a higher number, smaller, so detail face
            map_line->SetAttribute(TargetNumberTag, 100); //set targetnumber as nonexisting targetnumber 100
   //         concavelines.push_back(*map_line);
            fprintf(statfile, "this map line would have been pushed back %d label %d segment %d\n", map_line->Attribute(LineNumberTag), map_line->Attribute(LineLabelTag), map_line->Attribute(SegmentLabel));

//            if (seg1>seg2) 
            map_line++;
            map_line->SetAttribute(TargetNumberTag, 100); //set targetnumber as nonexisting targetnumber 100
  //          concavelines.push_back(*map_line);
           }
       }
       if (foundsegment) lastlinenumber = map_line->Attribute(LineNumberTag);
   }   
   keep_new_lines.insert(keep_new_lines.end(), concavelines.begin(), concavelines.end());
   fprintf(statfile,"size of keepnewlines: %d \n", keep_new_lines.size());   
   
   sort(matchedsegment_numbers.begin(), matchedsegment_numbers.end());
   for (segment_number = matchedsegment_numbers.begin(); segment_number!=matchedsegment_numbers.end(); segment_number++){
       if (*segment_number != lastsegmentnumber){
         selsegshapelines = segshapelines.SelectAttributedLines(SegmentLabel, *segment_number);
         goonwiththisone.erase(goonwiththisone.begin(), goonwiththisone.end());
         seg_laser_points.ErasePoints();
         seg_laser_points = laser_points.SelectTagValue(SegmentNumberTag, *segment_number);
         building = seg_laser_points.MostFrequentAttributeValue(PolygonNumberTag, count);
         plane = laser_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
         fprintf(statfile,"size of selsegshapelines: %d for segment %d\n", selsegshapelines.size(), *segment_number);
         for (map_line = selsegshapelines.begin(); map_line!=selsegshapelines.end(); map_line++){
           fprintf(statfile,"line %d, LineNumberTag %d, SegmentNumbertag %d for segment %d\n", map_line->Number(), map_line->Attribute(LineNumberTag), map_line->Attribute(SegmentLabel), *segment_number);
           findline = LineNumber(map_line->Attribute(LineNumberTag));
           index_line = keep_new_lines.FindLine(findline);
           selacceptedlines = keep_new_lines.SelectAttributedLines(LineNumberTag, map_line->Attribute(LineNumberTag));
           selacceptedlines2 = selacceptedlines.SelectAttributedLines(TargetNumberTag, 13);
           selacceptedlines3 = selacceptedlines.SelectAttributedLines(TargetNumberTag, 22);
    //       selacceptedlines4 = selacceptedlines.SelectAttributedLines(TargetNumberTag, 21);
//           selacceptedlines5 = selacceptedlines.SelectAttributedLines(TargetNumberTag, 17); //detail target
//           selacceptedlines6 = selacceptedlines.SelectAttributedLines(TargetNumberTag, 27); //detail
//           selacceptedlines7 = selacceptedlines.SelectAttributedLines(TargetNumberTag, 28); //detail
           selacceptedlines8 = selacceptedlines.SelectAttributedLines(TargetNumberTag, 100);
           oke = false;
           for (concave = selacceptedlines8.begin(); concave != selacceptedlines8.end(); concave++){
//              fprintf(statfile, "mapline %d, target %d, maplabel %d, map segment %d\n", concave->Attribute(LineNumberTag), concave->Attribute(TargetNumberTag), concave->Attribute(LineLabelTag), concave->Attribute(SegmentLabel)); 
               if (concave->Attribute(SegmentLabel) == *segment_number) oke = true;
           }
           if (selacceptedlines8.size()==0) oke = true;
           fprintf(statfile,"size of selacclines %d, size of selacclines2 %d\n", selacceptedlines.size(), selacceptedlines2.size());
           if (oke && selacceptedlines.size()>0 && selacceptedlines2.size()==0 && selacceptedlines3.size()==0 && selacceptedlines4.size()==0 && (selacceptedlines5.size()+selacceptedlines6.size()+selacceptedlines7.size() ==0)){ //yes accepted, but none targetnumber 13 or 22 or 21, or detail target 17,27,28
                                         goonwiththisone.push_back(*map_line);
                                         fprintf(statfile, "mapline %d, target %d, maplabel %d, map segment %d\n", map_line->Attribute(LineNumberTag), map_line->Attribute(TargetNumberTag), map_line->Attribute(LineLabelTag), map_line->Attribute(SegmentLabel));
                                         }
/*           if (index_line==-1) { // line not in accepted lines...
     //          selsegshapelines.erase(map_line); 
     //          map_line--;
           }
           else {
                edgeline = keep_new_lines[index_line];
                fprintf(statfile,"targetnumber of line %d (seg %d) for segment %d\n", edgeline.Attribute(TargetNumberTag),edgeline.Attribute(SegmentLabel), *segment_number);
                // exclude height jump intersection lines in this step...
                if (edgeline.Attribute(TargetNumberTag)!=13) goonwiththisone.push_back(keep_new_lines[index_line]);
                }
  */       }
         if (goonwiththisone.size()==0){ // if no accepted intersection lines, check for intersection lines from target 21 ( _/ type of intersection)
           for (map_line = selsegshapelines.begin(); map_line!=selsegshapelines.end(); map_line++){
           fprintf(statfile,"line %d, LineNumberTag %d, SegmentNumbertag %d for segment %d\n", map_line->Number(), map_line->Attribute(LineNumberTag), map_line->Attribute(SegmentLabel), *segment_number);
           findline = LineNumber(map_line->Attribute(LineNumberTag));
           index_line = keep_new_lines.FindLine(findline);
           selacceptedlines = keep_new_lines.SelectAttributedLines(LineNumberTag, map_line->Attribute(LineNumberTag));
           selacceptedlines2 = selacceptedlines.SelectAttributedLines(TargetNumberTag, 13);
           selacceptedlines3 = selacceptedlines.SelectAttributedLines(TargetNumberTag, 22);
 //          selacceptedlines4 = selacceptedlines.SelectAttributedLines(TargetNumberTag, 21);
           fprintf(statfile,"size of selacclines %d, size of selacclines2 %d\n", selacceptedlines.size(), selacceptedlines2.size());
           if (selacceptedlines.size()>0 && selacceptedlines2.size()==0 && selacceptedlines3.size()==0){ //yes accepted, but none targetnumber 13 or 22
                                         goonwiththisone.push_back(*map_line);
                                         }
           }
         }
         selsegshapelines = goonwiththisone;
         if (selsegshapelines.size()>0){ 
          fprintf(statfile,"size of accepted selsegshapelines: %d for segment %d\n", selsegshapelines.size(), *segment_number);
          endpoints = selsegshapelines.ReturnEndPointsOfTopologies(intsect_points);
           for (map_line = selsegshapelines.begin(); map_line!=selsegshapelines.end(); map_line++){
               fprintf(statfile,"map %d: ", map_line->Number());
               for (node=map_line->begin(); node!=map_line->end();node++){
                    fprintf(statfile,"%d ", node->Number());
               }
               fprintf(statfile,"\n");
           }
          shortlinepoints.ErasePoints();
          fprintf(statfile,"size of endpoints: %d for segment %d\n", endpoints.size(), *segment_number);
          if (int(endpoints.size()/2)*2 ==endpoints.size()){// size of endpoints is even
           selmergedsegmentlines.erase(selmergedsegmentlines.begin(), selmergedsegmentlines.end());
           shortmergedsegmentlines.erase(shortmergedsegmentlines.begin(), shortmergedsegmentlines.end());
           bool nothingchanged = true;
           if (selsegshapelines.size()>1 && endpoints.size()>2){ //first connect the two closest points with eachother if more than 1 int line is in the segment
             double mindist = 2;
             int iter = 0;
             do {
             nothingchanged = true;
             for (endpoint = endpoints.begin(); endpoint!=endpoints.end(); endpoint++){
              pos1 = endpoint->Position3DRef();
              counterlines = selsegshapelines.ReturnLinesWithPoint(endpoint->Number());
              fprintf(statfile,"point1 %d ", endpoint->Number());
              if (!shortmergedsegmentlines.Contains(PointNumber(endpoint->Number()))){
                for (endpoint2 = endpoints.begin(); endpoint2!=endpoints.end(); endpoint2++){
                 fprintf(statfile,"point2 %d ", endpoint2->Number());
                 if (endpoint !=endpoint2 ){
                   if (!counterlines.Contains(PointNumber(endpoint2->Number()))){
                     if (!shortmergedsegmentlines.Contains(PointNumber(endpoint2->Number()))){
                     pos2 = endpoint2->Position3DRef();
                     cost = selsegshapelines.CostBetweenTwoNodes(intsect_points, endpoint->Number(), endpoint2->Number());
//                     fprintf(statfile,"point2 %d ", endpoint2->Number());
                     fprintf(statfile,"dist %4.2f ", pos1.Distance2D(pos2));
                     fprintf(statfile,"cost %d ", cost);
                     if (pos1.Distance2D(pos2) < mindist && cost > 9) {
                         first_point = *endpoint;
                         sec_point = *endpoint2;
                         mindist = pos1.Distance2D(pos2);
                         fprintf(statfile,"mindist is %4.2f", mindist);
                         nothingchanged = false;
                     }
                     fprintf(statfile,"\n");
                     }
                   }
                }
            }
            }
          }
           
          top.clear(); top.Initialise();
          top.push_back(PointNumber(first_point.Number()));
          top.push_back(PointNumber(sec_point.Number()));
          top.SetAttribute(SegmentLabel, *segment_number);
          top.SetAttribute(LineLabelTag, 44);
          if (!nothingchanged) {
                               top.SetAttribute(BuildingNumberTag, building);
                               selmergedsegmentlines.push_back(top); // this excludes the line from further processing like adding points
                               shortmergedsegmentlines.push_back(top);
                               fprintf(statfile,"line with points %d and %d pushed back\n", first_point.Number(), sec_point.Number());
                               nothingchanged = true;
                               shortlinepoint.X() = first_point.Number();
                               shortlinepoint.Y() = sec_point.Number();
                               shortlinepoints.push_back(shortlinepoint);                     
                               mindist = 2;
                               } 
          iter++;                              
          } while (2*iter < endpoints.size() && nothingchanged);
          } //end if selshapelines > 1 && endpoints > 2
     
          loop = 0;
    //      do {
          maxcostd = -10;
          
          for (endpoint = endpoints.begin(); endpoint!=endpoints.end(); endpoint++){
            pos1 = endpoint->Position3DRef();
            fprintf(statfile,"point1 %d ", endpoint->Number());
            for (endpoint2 = endpoints.begin(); endpoint2!=endpoints.end(); endpoint2++){
                  if (endpoint2!=endpoint){
                  fprintf(statfile,"point2 %d ", endpoint2->Number());
                  pos2 = endpoint2->Position3DRef();
                  cost = selsegshapelines.CostBetweenTwoNodes(intsect_points, endpoint->Number(), endpoint2->Number());
                  fprintf(statfile,"dist %4.2f cost %d", pos1.Distance2D(pos2), cost);
                  dist = pos1.Distance2D(pos2);
                  costd = cost - dist;
                  fprintf(statfile,"cost - dist = %4.2f ", costd);
                  if (costd>maxcostd) {
                      maxcostd = costd;
                    first_point = *endpoint;
                    dumpoint = endpoint;
                    sec_point = *endpoint2;
                    mindist = pos1.Distance2D(pos2);
                    fprintf(statfile,"max so far");
                    }
                  fprintf(statfile,"\n");
                  }
              }
            }
            dumendpoints.erase(dumendpoints.begin(), dumendpoints.end());
            for (endpoint = endpoints.begin(); endpoint!=endpoints.end(); endpoint++){
                  if (endpoint->Number() == first_point.Number() || endpoint->Number() == sec_point.Number()){
                    dumendpoints.push_back(*endpoint);
                    }
            }
            for (endpoint = endpoints.begin(); endpoint!=endpoints.end(); endpoint++){
              if (endpoint->Number() != first_point.Number() && endpoint->Number() != sec_point.Number()){
                 dumendpoints.push_back(*endpoint);
              }
             }
          endpoints = dumendpoints;
            for (endpoint = endpoints.begin(); endpoint!=endpoints.end(); endpoint++){
                fprintf(statfile, " point number %d \n", endpoint->Number());
                }
            for (endpoint = endpoints.begin(); endpoint!=endpoints.end(); endpoint++){                
             if (!selmergedsegmentlines.Contains(PointNumber(endpoint->Number()))){
           shortline = false;
           for (laser_point = shortlinepoints.begin(); laser_point !=shortlinepoints.end();laser_point++){
              if (int(laser_point->X())==endpoint->Number()){
                 counterlines = shortmergedsegmentlines.ReturnLinesWithPoint(endpoint->Number()); //return the line from selmergedsegmentlines (label44)
                 fprintf(statfile,"size of counterlines for SHORT LINE: %d for segment %d, point %d\n", counterlines.size(), *segment_number, endpoint->Number());
                 shortline = true;
                 sel_match_line = counterlines.begin();
                 for (node=sel_match_line->begin(); node!=sel_match_line->end();node++){
                     if (node->Number()!=endpoint->Number()) {
                        defpoint = *(intsect_points.PointIterator(*node));
                     }
                 }
              }
           }
           
           if (!shortline){
           counterlines = selsegshapelines.ReturnLinesWithPoint(endpoint->Number());
           fprintf(statfile,"size of counterlines: %d for segment %d, point %d\n", counterlines.size(), *segment_number, endpoint->Number());
           }
           pos1 = endpoint->Position3DRef();
            sel_match_line = counterlines.begin(); //first take the second point as other point on same line, then adjust point later
            for (node=sel_match_line->begin(); node!=sel_match_line->end();node++){
                if (node->Number()!=endpoint->Number()) {
                                                        defpoint = *(intsect_points.PointIterator(*node));
                                                        suppos1 = defpoint.Position3DRef();
                                                        pos23d = suppos1;
                                                        pos22d = Position2D(defpoint.X(), defpoint.Y());
                                                        }
            }
            suptop1 = *sel_match_line;
            supline1 = Line3D(pos1, suppos1);
           dist = 10000;
           maxcost=0;
           maxcostd=-10;
           pos12d = Position2D(endpoint->X(), endpoint->Y());
           if (!shortline){
           for (endpoint2 = endpoints.begin(); endpoint2!=endpoints.end(); endpoint2++){
               fprintf(statfile, "end point2 %d ", endpoint2->Number());
               if (!selmergedsegmentlines.Contains(PointNumber(endpoint2->Number()))){
                if (selsegshapelines.size()>1){
                 if (!counterlines.Contains(PointNumber(endpoint2->Number()))){
                   pos2 = endpoint2->Position3DRef();
                   if (endpoint !=endpoint2){
                     cost = selsegshapelines.CostBetweenTwoNodes(intsect_points, endpoint->Number(), endpoint2->Number());
                     //if (pos1.Distance2D(pos2) < dist && cost >= maxcost) {
                     fprintf(statfile, "has cost %d\n",  cost);
                 //    dist = pos1.Distance(pos2);
                     costd = cost - pos1.Distance2D(pos2);//dist;
          //           if (costd > maxcostd) {
                       maxcostd = costd;
                       if (pos1.Distance2D(pos2)<dist){
                        defpoint = *endpoint2;
                        dist = pos1.Distance(pos2);
                        pos22d = Position2D(defpoint.X(), defpoint.Y());
                        pos23d = endpoint2->Position3DRef();
                        fprintf(statfile, "defpoint %d is taken as option", defpoint.Number());
     //                   }
                     }
                   }
                 }
                 }
                 else { //if selsegshapelines.size ==1
                      pos2 = endpoint2->Position3DRef();
                      if (endpoint !=endpoint2){
                        if (pos1.Distance2D(pos2) < dist) {
                          defpoint = *endpoint2;
                          dist = pos1.Distance(pos2);
                          pos22d = Position2D(defpoint.X(), defpoint.Y());
                          pos23d = endpoint2->Position3DRef();
                          fprintf(statfile, "defpoint %d is taken as option", defpoint.Number());
                        }
                      }
                 }
                }
            }
            }//end if not shortline
            counterlines = selsegshapelines.ReturnLinesWithPoint(defpoint.Number());
            sel_match_line = counterlines.begin();
            suptop2 = *sel_match_line;
            for (node=sel_match_line->begin(); node!=sel_match_line->end();node++){
                if (node->Number()!=defpoint.Number()) {
                                                        suppos2 = (intsect_points.PointIterator(*node))->Position3DRef();
                                                        }
            }
            supline2 = Line3D(pos23d, suppos2);
            top.clear(); top.Initialise();
            top.push_back(PointNumber(endpoint->Number()));
            extrapointadded = false;         
            line = Line2D(pos12d, pos22d);
            angleline = atan2(pos22d[0]-pos12d[0], pos22d[1]-pos12d[1])*180/PI;      
            line3d = Line3D(pos23d, pos1);
            
            cost = selsegshapelines.CostBetweenTwoNodes(intsect_points, endpoint->Number(), defpoint.Number());
            if (fabs(plane.Normal()[1])>0.00001) angleplane = atan2(plane.Normal()[0], plane.Normal()[1])*180/PI;
            fprintf(statfile, "%4.2f angle line and %4.2f plane angle\n", angleline, angleplane);
//            if ((fabs(angleplane - angleline)<5 || line3d.IsHorizontal(5*PI/180)) && cost > 9) shortline = true;
            if (((fabs(angleplane - angleline)<5) && cost > 9) ) shortline = true;
            if (endpoints.size()>3 && loop==0 && dist < 3) shortline = true;
            if (shortline) fprintf(statfile, "classified as shortline: %4.2f, %d\n", fabs(angleplane - angleline), cost);
            sel_laser_points.ErasePoints();
            sel_laser_points.AddTaggedPoints(laser_points, *segment_number, SegmentNumberTag);
            fprintf(statfile, "size of segment %d: %d \n", *segment_number, sel_laser_points.size());
//          
            sel_laser_points.DeriveDataBounds(0);                            
            hyposforbuilding = matchresults.SelectAttributedLines(BuildingNumberTag, building); //select all hypo's for building
            hyposforsegment = matchresults.ReturnLinesWithPoint(PointNumber (*segment_number)); //select all hypo's for this segment
            fprintf(statfile, "hypos for building %d, for segment %d \n", hyposforbuilding.size(), hyposforsegment.size());

            highestcov = 0;
            buildingheightfound = false;
            for (hypoforbuilding = hyposforbuilding.begin(); hypoforbuilding != hyposforbuilding.end(); hypoforbuilding++){
                if (hypoforbuilding->Attribute(MatchResultTag)==0 && hypoforbuilding->Attribute(TargetNumberTag)!=13 && hypoforbuilding->Attribute(CoverageTag)>highestcov){
                  buildinggoal_z = 0.01 * hypoforbuilding->Attribute(PredictedHeight); //to get height in m from cm
                  highestcov = hypoforbuilding->Attribute(CoverageTag);
                  target = hypoforbuilding->Attribute(TargetNumberTag);
                  buildingheightfound = true;
                  }
            }
            hypoheightfound = false;
            highestcov = 0;
            for (hypoforsegment = hyposforsegment.begin(); hypoforsegment != hyposforsegment.end(); hypoforsegment++){
                if (hypoforsegment->Attribute(MatchResultTag)==0 && hypoforsegment->Attribute(TargetNumberTag)!=13 && hypoforsegment->Attribute(CoverageTag)>highestcov){
                  hypogoal_z = 0.01 * hypoforsegment->Attribute(PredictedHeight); //to get height in m from cm
                  highestcov = hypoforsegment->Attribute(CoverageTag);
                  target = hypoforsegment->Attribute(TargetNumberTag);
                  hypoheightfound = true;
                  }
            }
            if (hypoheightfound && buildingheightfound) fprintf(statfile, "predicted height from total target (%d)= %4.2f, building(%d): %4.2f\n", target, hypogoal_z, building, buildinggoal_z);
            maxz = sel_laser_points.DataBounds().Maximum().Z();
            minz = sel_laser_points.DataBounds().Minimum().Z();
            pnl = seg_laser_points.TaggedPointNumberList(SegmentNumberTag, *segment_number);
            centrepoint = seg_laser_points.Centroid(pnl, newpointnumber);
            dumpos1 = Position2D(centrepoint.X(), centrepoint.Y());
     //       if (centrepoint.Z() < endpoint->Z()) goal_z = minz;
     //       else goal_z = maxz;  
            goal_z = minz;
            perc0 = sel_laser_points.ReturnHeightOfPercentilePoint(0); //0=minz, 100=maxz etc.
            perc5 = sel_laser_points.ReturnHeightOfPercentilePoint(4);
            fprintf(statfile, "minz = %4.2f, percentile %8f, %8f, max = %f \n", minz, perc0, perc5, maxz);
            changedheight = false;
            iter = 0;
            if (hypoheightfound && buildingheightfound) {
             do {
             if (fabs(goal_z-hypogoal_z)>0.5) fprintf(statfile, "HEIGHT DIFF more than 1 m with target"); //was 0.3
             else {
                  goal_z = hypogoal_z;
                  minz = hypogoal_z;
                  changedheight = true;
                  }
             if (fabs(goal_z-buildinggoal_z)>0.5 || fabs(hypogoal_z-buildinggoal_z)>0.5) fprintf(statfile, "HEIGHT DIFF more than 0.5 m with building or more that 0.5 between target and building");
             else {
                  goal_z = buildinggoal_z;
                  minz = buildinggoal_z;
                  changedheight = true;
                  }
             if (!changedheight && 0.1*(maxz-minz)< (perc5-minz)){ //if the lowest x% of the points differ more than 10% of the height diff take the height of xth percentile point
               goal_z = perc5;
               minz = perc5;
             }
             iter++;
             } while (iter<2 && !changedheight);
             }
             else {
              if (0.1*(maxz-minz)< (perc5-minz)){ //if the lowest x% of the points differ more than 10% of the height diff take the height of xth percentile point
               goal_z = perc5;
               minz = perc5;
             } 
             }
            // add other points here
            if (selsegshapelines.size()==1 && line3d.IsHorizontal(10*PI/180)){ // add condition here
            loweredsegment = false;
              if (!plane.IsHorizontal(10*PI/180) || plane.IsHorizontal(10*PI/180)){ //WATCH OUT CHANGED CODE, NOW THIS IS ALWAYS THE CASE
                 if (centrepoint.Z() < endpoint->Z()) goal_z = minz;
                 else goal_z = maxz;  
              
                selnearbymaplines = nearbymaplines.SelectAttributedLines(SegmentLabel, *segment_number);
                fprintf(statfile, "looking for intersection with point: %d\n", endpoint->Number());
                searchdist = 1;
                iter = 0;
                iter2 = 0;
                do {
                
                for (selnearbymapline = selnearbymaplines.begin(), linenumber=0; selnearbymapline != selnearbymaplines.end(); selnearbymapline++, linenumber++){
                    node=selnearbymapline->begin();
                    nbmpoint1 = *(nearbymappoints.PointIterator(*node));
                    nbmppos12d = Position2D(nbmpoint1.X(), nbmpoint1.Y());
                    node++;
                    nbmpoint2 = *(nearbymappoints.PointIterator(*node));
                    nbmppos22d = Position2D(nbmpoint2.X(), nbmpoint2.Y());
                    nbmline = Line2D(nbmppos12d, nbmppos22d);
                    anglenbmline = atan2(nbmppos22d[0]-nbmppos12d[0], nbmppos22d[1]-nbmppos12d[1])*180/PI;
      //              fprintf(statfile, "angle = %4.2f, anglenbm %4.2f, absdiff %4.2f\n", angleline, anglenbmline, fabs(angleline - anglenbmline));    
                    hundredangle = int(100*anglenbmline);
                    if (line3d.FindIntersection(nbmline, projposonmap1)){
                      if (pos1.Distance(projposonmap1)<pos23d.Distance(projposonmap1) && pos1.Distance(projposonmap1)<searchdist && fabs(angleline-anglenbmline)>20 && fabs(fabs(angleline-anglenbmline)-180)>20){
                          maplaserpoint.X() = projposonmap1.GetX();
                          maplaserpoint.Y() = projposonmap1.GetY();
                          maplaserpoint.Z() = projposonmap1.GetZ();
                          nbmppos13d = Position3D(nbmpoint1.X(), nbmpoint1.Y(), maplaserpoint.Z());
                          nbmppos23d = Position3D(nbmpoint2.X(), nbmpoint2.Y(), maplaserpoint.Z());
                          nbmline3d = Line3D(nbmppos13d, nbmppos23d);
//                          projposonmap2d = Position2D(maplaserpoint.X(), maplaserpoint.Y());
                          hundreddistance = int(100*pos1.Distance(projposonmap1));
                          scalar = nbmline3d.Scalar(projposonmap1);                          
                          nbmlength = (nbmppos13d - nbmppos23d).Length();
                          scalar = 1*scalar/nbmlength;
                          scalar3d = line3d.Scalar(projposonmap1);
                          linelength = (pos1 - pos23d).Length();
                          scalar3d = 1*scalar3d/linelength;
      //                    fprintf(statfile, "normalized scalar nbm line = %4.2f (length %4.2f), norm scalar 3d line %4.2f (length %4.2f)\n", scalar/nbmlength, nbmlength, scalar3d/linelength, linelength);    
                          maplaserpoint.SetAttribute(SegmentNumberTag, *segment_number);
                          maplaserpoint.SetAttribute(LabelTag, endpoint->Number());
                          maplaserpoint.Attribute(PlaneNumberTag) = hundreddistance;
                          if (scalar3d>=1 || scalar3d<=0){ //outside 3dline
                                       if (scalar>=0 && scalar<=1) { //but at map segment
                                           maplaserpoint.Residual()= 0;
                                           iter++;
                                           }
                                       else {
                                            maplaserpoint.Residual() = 0.3; // in prolongated line of map segment...
                                            iter2++;
                                            }
                          }
                          else  {
                                maplaserpoint.Residual() = 0.6;
                                iter++; //stop looking for other other points. the line allready exceeded the mapline
                                }

                        //  maplaserpoint.SetAttribute(PulseLengthTag, hundredangle); // TRICK transfer direction to PulseLengthTag
                          maplaserpoint.SetAttribute(PulseLengthTag, linenumber); // TRICK transfer linenumber to pulselengthtag
                          maplaserpoints.push_back(maplaserpoint);
                        }                
                     }
                }
                segmaplaserpoints.ErasePoints();
                segmaplaserpoints.AddTaggedPoints(maplaserpoints, *segment_number, SegmentNumberTag); //select points that belong to this segment
                selmaplaserpoints.ErasePoints();
                selmaplaserpoints.AddTaggedPoints(segmaplaserpoints, endpoint->Number(), LabelTag); //select points that belong to this endpoint
                searchdist = searchdist+1;
            //    } while (selmaplaserpoints.size()==0 && searchdist <10);
//                } while (iter==0 && iter2<2 && searchdist <max_distance_line2map);
                } while (iter==0 && iter2<2 && searchdist <10);

                // choose best option from intersection with mapline
                
                  selsnappoints.ErasePoints();
                  found = false;
                  takendist = 0;
                  for (laser_point=selmaplaserpoints.begin(); laser_point!=selmaplaserpoints.end(); laser_point++) {
                      if (laser_point->Residual()<0.2 && !found){
                        selsnappoints.push_back(*laser_point);
                        found = true;
                        takendist  = laser_point->Attribute(PlaneNumberTag);
                        }
                  }
                  if (selsnappoints.size()==0) takendist =1000; //dist in cm
                  for (laser_point=selmaplaserpoints.begin(); laser_point!=selmaplaserpoints.end(); laser_point++) {
                      if (laser_point->Residual()<0.5 && laser_point->Attribute(PlaneNumberTag)<0.5*takendist){
                        selsnappoints.push_back(*laser_point);
                        found = true;
                        }
                  }                             
//                  }
                  if (selsnappoints.size()==0){
                  for (laser_point=selmaplaserpoints.begin(); laser_point!=selmaplaserpoints.end(); laser_point++) {
                      if (laser_point->Residual()<1 && !found){
//                        selsnappoints.push_back(*laser_point);
//                        found = true;
                        }
                  }                             
                  }
                                  
                if (found) {
                if (selsnappoints.size()>1){ //keep only the best option
                   takendist = 1000;
                   for (laser_point=selsnappoints.begin(); laser_point!=selsnappoints.end(); laser_point++) {
                       if (laser_point->Attribute(PlaneNumberTag)<takendist){
                           takendist = laser_point->Attribute(PlaneNumberTag);
                           selsnappoint = *laser_point;
                           selsnappoint.SetAttribute(PulseLengthTag, laser_point->Attribute(PulseLengthTag));
                           }
                       } 
                }
                else {
                     selsnappoint = *(selsnappoints.begin());
                     selsnappoint.SetAttribute(PulseLengthTag, (selsnappoints.begin())->Attribute(PulseLengthTag));
                     }
                selsnappoint.SetAttribute(LabelTag, 3);
                maplaserpoints.push_back(selsnappoint);
                newpointnumber++;
                newpos2 = Position3D(selsnappoint.X(), selsnappoint.Y(), selsnappoint.Z()); //this is the higher new point (near to defpoint, pos23d)
                newedgepoint = ObjectPoint(newpos2.GetX(), newpos2.GetY(), newpos2.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                intsect_points.push_back(newedgepoint);
                extendridgepoint.X() = endpoint->Number();
                extendridgepoint.Y() = newpointnumber;
                extendridgepoint.Z() = *segment_number;
                extendridgepoint.Residual() = selsnappoint.Residual();
                extendridgepoint.Attribute(PulseLengthTag) = selsnappoint.Attribute(PulseLengthTag);
                extendridgepoints.push_back(extendridgepoint);
                }
                if(!plane.IsHorizontal(10*PI/180)){
                perpline = line.PerpendicularLine(pos12d);
                dumpos2 = perpline.Project(dumpos1);
                dumpos3d = Position3D(dumpos2.GetX(), dumpos2.GetY(), centrepoint.Z());
                projpos3d = plane.Project(dumpos3d);
                line3d2 = Line3D(pos1, projpos3d);
                newpos = line3d2.DetPositionZ(goal_z);
                newpointnumber++;
                fprintf(statfile, "created new low point %d\n", newpointnumber);
                newedgepoint = ObjectPoint(newpos.GetX(), newpos.GetY(), newpos.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                top.push_back(PointNumber(newpointnumber));
                top.SetAttribute(SegmentLabel, *segment_number);
                top.SetAttribute(LineLabelTag, 1);
                selmergedsegmentlines.push_back(top);
                top.clear(); top.Initialise();
                top.push_back(PointNumber(newpointnumber));
                extrapointadded = true;
                intsect_points.push_back(newedgepoint);
                }            
                if(plane.IsHorizontal(10*PI/180)){
                // add some code here to use information from this line to reconstruct this flat segment
                maxdist = 0;
                for (laser_point=sel_laser_points.begin(); laser_point!=sel_laser_points.end()-1; laser_point++) {
                     dist = line.DistanceToPoint(laser_point->Position2DOnly());
                     if (dist > maxdist) {
                        maxdist = dist;
                        dumposend = laser_point->Position2DOnly();
                     }
                }

                perpline = line.PerpendicularLine(pos12d);
                dumpos2 = perpline.Project(dumposend);
                dumpos3d = Position3D(dumpos2.GetX(), dumpos2.GetY(), centrepoint.Z());
                projpos3d = plane.Project(dumpos3d);
//                line3d2 = Line3D(pos1, projpos3d);
                newpos = projpos3d;//line3d2.DetPositionZ(goal_z);
                newpointnumber++;
                fprintf(statfile, "created new low point %d\n", newpointnumber);
                newedgepoint = ObjectPoint(newpos.GetX(), newpos.GetY(), newpos.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                top.push_back(PointNumber(newpointnumber));
                top.SetAttribute(SegmentLabel, *segment_number);
                top.SetAttribute(LineLabelTag, 2); //changed from 1 to 2
                selmergedsegmentlines.push_back(top);
    
                top.clear(); top.Initialise();
                top.push_back(PointNumber(newpointnumber));
                extrapointadded = true;
                intsect_points.push_back(newedgepoint);
                }            
            
                //now add second low point. first look if the higher point should snap to the map.
                fprintf(statfile, "looking for intersection with point: %d\n", defpoint.Number());
                
                searchdist = 1;
                iter = 0;
                iter2 = 0;
                do {
                
                for (selnearbymapline = selnearbymaplines.begin(), linenumber=0; selnearbymapline != selnearbymaplines.end(); selnearbymapline++, linenumber++){
                    node=selnearbymapline->begin();
                    nbmpoint1 = *(nearbymappoints.PointIterator(*node));
                    nbmppos12d = Position2D(nbmpoint1.X(), nbmpoint1.Y());
                    node++;
                    nbmpoint2 = *(nearbymappoints.PointIterator(*node));
                    nbmppos22d = Position2D(nbmpoint2.X(), nbmpoint2.Y());
                    nbmline = Line2D(nbmppos12d, nbmppos22d);
                    anglenbmline = atan2(nbmppos22d[0]-nbmppos12d[0], nbmppos22d[1]-nbmppos12d[1])*180/PI;
         //           fprintf(statfile, "angle = %4.2f, anglenbm %4.2f, absdiff %4.2f\n", angleline, anglenbmline, fabs(angleline - anglenbmline));    
                    hundredangle = int(100*anglenbmline);
                    if (line3d.FindIntersection(nbmline, projposonmap1)){
                      if (pos1.Distance(projposonmap1)>pos23d.Distance(projposonmap1) && pos23d.Distance(projposonmap1)<searchdist && fabs(angleline-anglenbmline)>20 && fabs(fabs(angleline-anglenbmline)-180)>20){
                          maplaserpoint.X() = projposonmap1.GetX();
                          maplaserpoint.Y() = projposonmap1.GetY();
                          maplaserpoint.Z() = projposonmap1.GetZ();
                          nbmppos13d = Position3D(nbmpoint1.X(), nbmpoint1.Y(), maplaserpoint.Z());
                          nbmppos23d = Position3D(nbmpoint2.X(), nbmpoint2.Y(), maplaserpoint.Z());
                          nbmline3d = Line3D(nbmppos13d, nbmppos23d);
                          hundreddistance = int(100*pos23d.Distance(projposonmap1));
//                          projposonmap2d = Position2D(maplaserpoint.X(), maplaserpoint.Y());
                          scalar = nbmline3d.Scalar(projposonmap1);                          
                          nbmlength = (nbmppos13d - nbmppos23d).Length();
                          scalar = 1*scalar/nbmlength;
                          scalar3d = line3d.Scalar(projposonmap1);
                          linelength = (pos1 - pos23d).Length();
                          scalar3d = 1*scalar3d/linelength;
                          fprintf(statfile, "normalized scalar nbm line = %4.2f (length %4.2f), norm scalar 3d line %4.2f (length %4.2f)\n", scalar/nbmlength, nbmlength, scalar3d/linelength, linelength);    
                          maplaserpoint.SetAttribute(SegmentNumberTag, *segment_number);
                          maplaserpoint.SetAttribute(LabelTag, defpoint.Number());
                          maplaserpoint.Attribute(PlaneNumberTag) = hundreddistance;
                          if (scalar3d>=1 || scalar3d<=0){ //outside 3dline
                                       if (scalar>=0 && scalar<=1) { //but at map segment
                                           maplaserpoint.Residual()= 0;
                                           iter++;
                                           }
                                       else {
                                            maplaserpoint.Residual() = 0.3; // in prolongated line of map segment...
                                            iter2++;
                                            }
                          }
                          else  {
                                maplaserpoint.Residual() = 0.6;
                                iter++; //stop looking for other other points. the line allready exceeded the mapline
                                }
                        //  maplaserpoint.SetAttribute(PulseLengthTag, hundredangle); // TRICK transfer direction to PulseLengthTag
                          maplaserpoint.SetAttribute(PulseLengthTag, linenumber); // TRICK linenumber direction to PulseLengthTag
                          maplaserpoints.push_back(maplaserpoint);
                        }                
                     }
                }
                segmaplaserpoints.ErasePoints();
                segmaplaserpoints.AddTaggedPoints(maplaserpoints, *segment_number, SegmentNumberTag); //select points that belong to this segment
                selmaplaserpoints.ErasePoints();
                selmaplaserpoints.AddTaggedPoints(segmaplaserpoints, defpoint.Number(), LabelTag); //select points that belong to this endpoint
                searchdist = searchdist+1;
//                } while (selmaplaserpoints.size()==0 && searchdist <10);
                } while (iter==0 && iter2<2 && searchdist <max_distance_line2map);
//                } while (iter==0 && iter2<2 && searchdist <10);
                // choose best option from intersection with mapline
                
                  selsnappoints.ErasePoints();
                  found = false;
                  takendist = 0;
                  for (laser_point=selmaplaserpoints.begin(); laser_point!=selmaplaserpoints.end(); laser_point++) {
                      if (laser_point->Residual()<0.2 && !found){
                        selsnappoints.push_back(*laser_point);
                        found = true;
                        takendist  = laser_point->Attribute(PlaneNumberTag);
                        }
                  }
                  if (selsnappoints.size()==0) takendist =1000; //dist in cm
                  for (laser_point=selmaplaserpoints.begin(); laser_point!=selmaplaserpoints.end(); laser_point++) {
                      if (laser_point->Residual()<0.5 && laser_point->Attribute(PlaneNumberTag)<0.5*takendist){
                        selsnappoints.push_back(*laser_point);
                        found = true;
                        }
                  }                             
//                  }
                  if (selsnappoints.size()==0){
                  for (laser_point=selmaplaserpoints.begin(); laser_point!=selmaplaserpoints.end(); laser_point++) {
                      if (laser_point->Residual()<1 && !found){
  //                      selsnappoints.push_back(*laser_point);
  //                      found = true;
                        }
                  }                             
                  }
                                  
                if (found) {
                if (selsnappoints.size()>1){ //keep only the best option
                   takendist = 1000;
                   for (laser_point=selsnappoints.begin(); laser_point!=selsnappoints.end(); laser_point++) {
                       if (laser_point->Attribute(PlaneNumberTag)<takendist){
                           takendist = laser_point->Attribute(PlaneNumberTag);
                           selsnappoint = *laser_point;
                           selsnappoint.SetAttribute(PulseLengthTag, laser_point->Attribute(PulseLengthTag));
                           }
                       } 
                }
                else {
                     selsnappoint = *(selsnappoints.begin());
                     selsnappoint.SetAttribute(PulseLengthTag, (selsnappoints.begin())->Attribute(PulseLengthTag));
                     }
                selsnappoint.SetAttribute(LabelTag, 3);
                maplaserpoints.push_back(selsnappoint);
                newpointnumber++;
                newpos2 = Position3D(selsnappoint.X(), selsnappoint.Y(), selsnappoint.Z()); //this is the higher new point (near to defpoint, pos23d)
                newedgepoint = ObjectPoint(newpos2.GetX(), newpos2.GetY(), newpos2.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                intsect_points.push_back(newedgepoint);
                extendridgepoint.X() = defpoint.Number();
                extendridgepoint.Y() = newpointnumber;
                extendridgepoint.Z() = *segment_number;
                extendridgepoint.Residual() = selsnappoint.Residual();
                extendridgepoint.Attribute(PulseLengthTag) = selsnappoint.Attribute(PulseLengthTag);
                extendridgepoints.push_back(extendridgepoint);
                }
          //      else{ // if not snap to mapline
                  if (!plane.IsHorizontal(10*PI/180)){
                newpointnumber++;
                perpline = line.PerpendicularLine(pos22d);
                dumpos2 = perpline.Project(dumpos1);
                dumpos3d = Position3D(dumpos2.GetX(), dumpos2.GetY(), centrepoint.Z());
                projpos3d = plane.Project(dumpos3d);
                line3d2 = Line3D(pos23d, projpos3d);
                newpos = line3d2.DetPositionZ(goal_z);
                newedgepoint = ObjectPoint(newpos.GetX(), newpos.GetY(), newpos.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                top.push_back(PointNumber(newpointnumber));
                intsect_points.push_back(newedgepoint);
                top.SetAttribute(SegmentLabel, *segment_number);
                top.SetAttribute(LineLabelTag, 4);
                selmergedsegmentlines.push_back(top);
                top.clear(); top.Initialise();
                top.push_back(PointNumber(newpointnumber));
                }
                if(plane.IsHorizontal(10*PI/180)){
                  // add some code here to use information from this line to reconstruct this flat segment
              perpline = line.PerpendicularLine(pos22d);
                dumpos2 = perpline.Project(dumposend);
                dumpos3d = Position3D(dumpos2.GetX(), dumpos2.GetY(), centrepoint.Z());
                projpos3d = plane.Project(dumpos3d);
//                line3d2 = Line3D(pos1, projpos3d);
                newpos = projpos3d;//line3d2.DetPositionZ(goal_z);
                newpointnumber++;
                fprintf(statfile, "created new low point %d\n", newpointnumber);
                newedgepoint = ObjectPoint(newpos.GetX(), newpos.GetY(), newpos.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                top.push_back(PointNumber(newpointnumber));
                top.SetAttribute(SegmentLabel, *segment_number);
                top.SetAttribute(LineLabelTag, 4);
                selmergedsegmentlines.push_back(top);
    
                top.clear(); top.Initialise();
                top.push_back(PointNumber(newpointnumber));
                extrapointadded = true;
                intsect_points.push_back(newedgepoint);
                }            
                          
              } //end if plane!horizontal || plane horizontal //OKE, stupid if argument i know
            }
            //end add other points
            
            if (selsegshapelines.size()>1 && line3d.IsHorizontal(10*PI/180)){ // add condition here
            }
            if (!shortline && selsegshapelines.size()>1 && !plane.IsHorizontal(10*PI/180) ){//&& !line3d.IsHorizontal(10*PI/180)){ // add condition here
            // case of a hori+tilted selmergedline
            loweredsegment = false;
              if (!supline1.IsHorizontal(10*PI/180)){
                 fprintf(statfile, "case !a\n");
                newpos = supline1.DetPositionZ(goal_z);
                newpointnumber++;
                newedgepoint = ObjectPoint(newpos.GetX(), newpos.GetY(), newpos.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                top.push_back(PointNumber(newpointnumber));
                intsect_points.push_back(newedgepoint);
                top.SetAttribute(SegmentLabel, *segment_number);
                top.SetAttribute(LineLabelTag, 55);
                selmergedsegmentlines.push_back(top);
                top.clear(); top.Initialise();
                top.push_back(PointNumber(newpointnumber));
         //       loweredsegment = true;              
                if (!supline2.IsHorizontal(10*PI/180)){ // supline 1 not horizontal, supline 2 not horizontal
                  fprintf(statfile, "case !a!b\n");
                  newpos = supline2.DetPositionZ(goal_z);
                  newpointnumber++;
                  newedgepoint = ObjectPoint(newpos.GetX(), newpos.GetY(), newpos.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                  top.push_back(PointNumber(newpointnumber));
                  intsect_points.push_back(newedgepoint);
                  top.SetAttribute(SegmentLabel, *segment_number);
                  top.SetAttribute(LineLabelTag, 4);
                  selmergedsegmentlines.push_back(top);
                  top.clear(); top.Initialise();
                  top.push_back(PointNumber(newpointnumber));
                  loweredsegment = true;              
                  }
                  else { //if supline 1 is not horizontal, and supline 2 is
                    fprintf(statfile, "case !ab\n");

                    supdumpos2=Position2D(suppos2.GetX(), suppos2.GetY());
//                    dumpos2 = Position2D(pos22d.GetX(), pos22d.GetY());
                    line = Line2D(supdumpos2, pos22d);
                    perpline = line.PerpendicularLine(pos22d);
                    dumpos2 = perpline.Project(dumpos1);
                    dumpos3d = Position3D(dumpos2.GetX(), dumpos2.GetY(), centrepoint.Z());
                    projpos3d = plane.Project(dumpos3d);
                    line3d2 = Line3D(pos23d, projpos3d);
                    newpos = line3d2.DetPositionZ(goal_z);
                    newpointnumber++;
                    newedgepoint = ObjectPoint(newpos.GetX(), newpos.GetY(), newpos.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                    top.push_back(PointNumber(newpointnumber));
                    intsect_points.push_back(newedgepoint);
                    top.SetAttribute(SegmentLabel, *segment_number);
                    top.SetAttribute(LineLabelTag, 4);
                    selmergedsegmentlines.push_back(top);
                    top.clear(); top.Initialise();
                    top.push_back(PointNumber(newpointnumber));
//                    loweredsegment = true;  this is the last piece to horizontal line, label green 1
                      extrapointadded = true;
                      
                                    selnearbymaplines = nearbymaplines.SelectAttributedLines(SegmentLabel, *segment_number);
                fprintf(statfile, "looking for intersection with point: %d\n", defpoint.Number());
                searchdist = 1;
                iter = 0;
                iter2 = 0;

                do {
                
                for (selnearbymapline = selnearbymaplines.begin(), linenumber=0; selnearbymapline != selnearbymaplines.end(); selnearbymapline++, linenumber++){
                    node=selnearbymapline->begin();
                    nbmpoint1 = *(nearbymappoints.PointIterator(*node));
                    nbmppos12d = Position2D(nbmpoint1.X(), nbmpoint1.Y());
                    node++;
                    nbmpoint2 = *(nearbymappoints.PointIterator(*node));
                    nbmppos22d = Position2D(nbmpoint2.X(), nbmpoint2.Y());
                    nbmline = Line2D(nbmppos12d, nbmppos22d);
                    anglenbmline = atan2(nbmppos22d[0]-nbmppos12d[0], nbmppos22d[1]-nbmppos12d[1])*180/PI;
      //              fprintf(statfile, "angle = %4.2f, anglenbm %4.2f, absdiff %4.2f\n", angleline, anglenbmline, fabs(angleline - anglenbmline));    
                    hundredangle = int(100*anglenbmline);
                    if (supline2.FindIntersection(nbmline, projposonmap1)){
                      if (pos23d.Distance(projposonmap1)<suppos2.Distance(projposonmap1) && pos23d.Distance(projposonmap1)<searchdist && fabs(angleline-anglenbmline)>20 && fabs(fabs(angleline-anglenbmline)-180)>20){
                          maplaserpoint.X() = projposonmap1.GetX();
                          maplaserpoint.Y() = projposonmap1.GetY();
                          maplaserpoint.Z() = projposonmap1.GetZ();
                          nbmppos13d = Position3D(nbmpoint1.X(), nbmpoint1.Y(), maplaserpoint.Z());
                          nbmppos23d = Position3D(nbmpoint2.X(), nbmpoint2.Y(), maplaserpoint.Z());
                          nbmline3d = Line3D(nbmppos13d, nbmppos23d);
//                          projposonmap2d = Position2D(maplaserpoint.X(), maplaserpoint.Y());
                          hundreddistance = int(100*pos23d.Distance(projposonmap1));

                          scalar = nbmline3d.Scalar(projposonmap1);                          
                          nbmlength = (nbmppos13d - nbmppos23d).Length();
                          scalar = 1*scalar/nbmlength;
                          scalar3d = supline2.Scalar(projposonmap1);
                          linelength = (pos23d - suppos2).Length();
                          scalar3d = 1*scalar3d/linelength;
      //                    fprintf(statfile, "normalized scalar nbm line = %4.2f (length %4.2f), norm scalar 3d line %4.2f (length %4.2f)\n", scalar/nbmlength, nbmlength, scalar3d/linelength, linelength);    
                          maplaserpoint.SetAttribute(SegmentNumberTag, *segment_number);
                          maplaserpoint.SetAttribute(LabelTag, defpoint.Number());
                          maplaserpoint.Attribute(PlaneNumberTag) = hundreddistance;
                          if (scalar3d>=1 || scalar3d<=0){ //outside 3dline
                                       if (scalar>=0 && scalar<=1) { //but at map segment
                                           iter++;
                                           }
                                       else {
                                            maplaserpoint.Residual() = 0.3; // in prolongated line of map segment...
                                            iter2++;
                                            }
                          }
                          else  {
                                maplaserpoint.Residual() = 0.6;
                                iter++; //stop looking for other other points. the line allready exceeded the mapline
                                }
                        //  maplaserpoint.SetAttribute(PulseLengthTag, hundredangle); // TRICK transfer direction to PulseLengthTag
                          maplaserpoint.SetAttribute(PulseLengthTag, linenumber); // TRICK transfer linenumber to pulselengthtag
                          maplaserpoints.push_back(maplaserpoint);
                        }                
                     }
                }
                segmaplaserpoints.ErasePoints();
                segmaplaserpoints.AddTaggedPoints(maplaserpoints, *segment_number, SegmentNumberTag); //select points that belong to this segment
                selmaplaserpoints.ErasePoints();
                selmaplaserpoints.AddTaggedPoints(segmaplaserpoints, defpoint.Number(), LabelTag); //select points that belong to this endpoint
                searchdist = searchdist+1;
//                } while (selmaplaserpoints.size()==0 && searchdist <10);
                              } while (iter==0 && iter2<2 && searchdist <max_distance_line2map);
//                } while (iter==0 && iter2<2 && searchdist <10);

                // choose best option from intersection with mapline
                
                  selsnappoints.ErasePoints();
                  found = false;
                  takendist = 0;
                  for (laser_point=selmaplaserpoints.begin(); laser_point!=selmaplaserpoints.end(); laser_point++) {
                      if (laser_point->Residual()<0.2 && !found){
                        selsnappoints.push_back(*laser_point);
                        found = true;
                        takendist  = laser_point->Attribute(PlaneNumberTag);
                        }
                  }
                  if (selsnappoints.size()==0) takendist =1000; //dist in cm
                  for (laser_point=selmaplaserpoints.begin(); laser_point!=selmaplaserpoints.end(); laser_point++) {
                      if (laser_point->Residual()<0.5 && laser_point->Attribute(PlaneNumberTag)<0.5*takendist){
                        selsnappoints.push_back(*laser_point);
                        found = true;
                        }
                  }                             
//                  }
                  if (selsnappoints.size()==0){
                  for (laser_point=selmaplaserpoints.begin(); laser_point!=selmaplaserpoints.end(); laser_point++) {
                      if (laser_point->Residual()<1 && !found){
    //                    selsnappoints.push_back(*laser_point);
    //                    found = true;
                        }
                  }                             
                  }
                                  
                if (found) {
                if (selsnappoints.size()>1){ //keep only the best option
                   takendist = 1000;
                   for (laser_point=selsnappoints.begin(); laser_point!=selsnappoints.end(); laser_point++) {
                       if (laser_point->Attribute(PlaneNumberTag)<takendist){
                           takendist = laser_point->Attribute(PlaneNumberTag);
                           selsnappoint = *laser_point;
                           selsnappoint.SetAttribute(PulseLengthTag, laser_point->Attribute(PulseLengthTag));
                           }
                       } 
                }
                else {
                     selsnappoint = *(selsnappoints.begin());
                     selsnappoint.SetAttribute(PulseLengthTag, (selsnappoints.begin())->Attribute(PulseLengthTag));
                     }
                selsnappoint.SetAttribute(LabelTag, 3);
                maplaserpoints.push_back(selsnappoint);
                newpointnumber++;
                newpos2 = Position3D(selsnappoint.X(), selsnappoint.Y(), selsnappoint.Z()); //this is the higher new point (near to defpoint, pos23d)
                newedgepoint = ObjectPoint(newpos2.GetX(), newpos2.GetY(), newpos2.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                intsect_points.push_back(newedgepoint);
                extendridgepoint.X() = defpoint.Number();
                extendridgepoint.Y() = newpointnumber;
                extendridgepoint.Z() = *segment_number;
                extendridgepoint.Residual() = selsnappoint.Residual();
                extendridgepoint.Attribute(PulseLengthTag) = selsnappoint.Attribute(PulseLengthTag);
                extendridgepoints.push_back(extendridgepoint);
                }
                      
                  }
                }
                else { //if supline 1 is horizontal
                    fprintf(statfile, "case a\n");

                    supdumpos1=Position2D(suppos1.GetX(), suppos1.GetY());
//                    dumpos2 = Position2D(pos22d.GetX(), pos22d.GetY());
                    line = Line2D(supdumpos1, pos12d);
                    perpline = line.PerpendicularLine(pos12d);
                    dumpos2 = perpline.Project(dumpos1);
                    dumpos3d = Position3D(dumpos2.GetX(), dumpos2.GetY(), centrepoint.Z());
                    projpos3d = plane.Project(dumpos3d);
                    line3d2 = Line3D(pos1, projpos3d);
                    newpos = line3d2.DetPositionZ(goal_z);
                    newpointnumber++;
                    newedgepoint = ObjectPoint(newpos.GetX(), newpos.GetY(), newpos.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                    top.push_back(PointNumber(newpointnumber));
                    intsect_points.push_back(newedgepoint);
                    top.SetAttribute(SegmentLabel, *segment_number);
                    top.SetAttribute(LineLabelTag, 1);
                    fprintf(statfile, "point number %d created between point %d and point %d\n", newpointnumber, endpoint->Number(), defpoint.Number());
                    selmergedsegmentlines.push_back(top);
                    top.clear(); top.Initialise();
                    top.push_back(PointNumber(newpointnumber));
                    
                       selnearbymaplines = nearbymaplines.SelectAttributedLines(SegmentLabel, *segment_number);
                fprintf(statfile, "looking for intersection with point: %d\n", endpoint->Number());
                searchdist = 1;
                iter = 0;
                iter2 = 0;
                do {
                
                for (selnearbymapline = selnearbymaplines.begin(), linenumber=0; selnearbymapline != selnearbymaplines.end(); selnearbymapline++, linenumber++){
                    node=selnearbymapline->begin();
                    nbmpoint1 = *(nearbymappoints.PointIterator(*node));
                    nbmppos12d = Position2D(nbmpoint1.X(), nbmpoint1.Y());
                    node++;
                    nbmpoint2 = *(nearbymappoints.PointIterator(*node));
                    nbmppos22d = Position2D(nbmpoint2.X(), nbmpoint2.Y());
                    nbmline = Line2D(nbmppos12d, nbmppos22d);
                    anglenbmline = atan2(nbmppos22d[0]-nbmppos12d[0], nbmppos22d[1]-nbmppos12d[1])*180/PI;
      //              fprintf(statfile, "angle = %4.2f, anglenbm %4.2f, absdiff %4.2f\n", angleline, anglenbmline, fabs(angleline - anglenbmline));    
                    hundredangle = int(100*anglenbmline);
                    if (supline1.FindIntersection(nbmline, projposonmap1)){
                      if (pos1.Distance(projposonmap1)<suppos1.Distance(projposonmap1) && pos1.Distance(projposonmap1)<searchdist && fabs(angleline-anglenbmline)>20 && fabs(fabs(angleline-anglenbmline)-180)>20){
                          fprintf(statfile, "%4.1f en %4.1f \n", pos1.Distance(projposonmap1),suppos1.Distance(projposonmap1));
                          fprintf(statfile, "%4.1f en %4.1f \n", projposonmap1.GetX(), projposonmap1.GetY());
                          maplaserpoint.X() = projposonmap1.GetX();
                          maplaserpoint.Y() = projposonmap1.GetY();
                          maplaserpoint.Z() = projposonmap1.GetZ();
                          nbmppos13d = Position3D(nbmpoint1.X(), nbmpoint1.Y(), maplaserpoint.Z());
                          nbmppos23d = Position3D(nbmpoint2.X(), nbmpoint2.Y(), maplaserpoint.Z());
                          nbmline3d = Line3D(nbmppos13d, nbmppos23d);
                          hundreddistance = int(100*pos1.Distance(projposonmap1));

//                          projposonmap2d = Position2D(maplaserpoint.X(), maplaserpoint.Y());
                          scalar = nbmline3d.Scalar(projposonmap1);                          
                          nbmlength = (nbmppos13d - nbmppos23d).Length();
                          scalar = 1*scalar/nbmlength;
                          scalar3d = supline1.Scalar(projposonmap1);
                          linelength = (pos1 - suppos1).Length();
                          scalar3d = 1*scalar3d/linelength;
      //                    fprintf(statfile, "normalized scalar nbm line = %4.2f (length %4.2f), norm scalar 3d line %4.2f (length %4.2f)\n", scalar/nbmlength, nbmlength, scalar3d/linelength, linelength);    
                          maplaserpoint.SetAttribute(SegmentNumberTag, *segment_number);
                          maplaserpoint.SetAttribute(LabelTag, endpoint->Number());
                          if (scalar3d>=1 || scalar3d<=0){ //outside 3dline
                                       if (scalar>=0 && scalar<=1) { //but at map segment
                                           iter++;
                                           maplaserpoint.Attribute(PlaneNumberTag) = hundreddistance;
                                           }
                                       else {
                                            maplaserpoint.Residual() = 0.3; // in prolongated line of map segment...
                                            iter2++;
                                            maplaserpoint.Attribute(PlaneNumberTag) = hundreddistance;
                                            }
                          }
                          else  {
                                maplaserpoint.Residual() = 0.6;
                                iter++; //stop looking for other other points. the line allready exceeded the mapline
                                maplaserpoint.Attribute(PlaneNumberTag) = hundreddistance;
                                fprintf(statfile, "%d residual 0.6 dist\n", hundreddistance);
                                }
                        //  maplaserpoint.SetAttribute(PulseLengthTag, hundredangle); // TRICK transfer direction to PulseLengthTag
                          maplaserpoint.SetAttribute(PulseLengthTag, linenumber); // TRICK transfer linenumber to pulselengthtag
                          maplaserpoints.push_back(maplaserpoint);
                        }                
                     }
                }
                segmaplaserpoints.ErasePoints();
                segmaplaserpoints.AddTaggedPoints(maplaserpoints, *segment_number, SegmentNumberTag); //select points that belong to this segment
                selmaplaserpoints.ErasePoints();
                selmaplaserpoints.AddTaggedPoints(segmaplaserpoints, endpoint->Number(), LabelTag); //select points that belong to this endpoint
                searchdist = searchdist+1;
                //} while (selmaplaserpoints.size()==0 && searchdist <10);
                } while (iter==0 && iter2<2 && searchdist <max_distance_line2map);
//                } while (iter==0 && iter2<2 && searchdist <10);

                // choose best option from intersection with mapline
                
                  selsnappoints.ErasePoints();
                  found = false;
                  takendist = 0;
                  for (laser_point=selmaplaserpoints.begin(); laser_point!=selmaplaserpoints.end(); laser_point++) {
                      if (laser_point->Residual()<0.2 && !found){
                        selsnappoints.push_back(*laser_point);
                        found = true;
                        takendist  = laser_point->Attribute(PlaneNumberTag);
                        fprintf(statfile, "option 1\n");
                        }
                  }
                  if (selsnappoints.size()==0) takendist =1000; //dist in cm
                  for (laser_point=selmaplaserpoints.begin(); laser_point!=selmaplaserpoints.end(); laser_point++) {
                      if (laser_point->Residual()<0.5 && laser_point->Attribute(PlaneNumberTag)<0.5*takendist){
                        selsnappoints.push_back(*laser_point);
                        found = true;
                        fprintf(statfile, "option 2\n");
                        }
                  }                             
//                  }
                  if (selsnappoints.size()==0){
                  for (laser_point=selmaplaserpoints.begin(); laser_point!=selmaplaserpoints.end(); laser_point++) {
                      if (laser_point->Residual()<1 && !found){
      //                  selsnappoints.push_back(*laser_point);
      //                  found = true;
                        fprintf(statfile, "option 3 %4.2f %4.2f\n", laser_point->X(), laser_point->Y());
                        }
                  }                             
                  }
                                  
                if (found) {
                fprintf(statfile, "size of selsnappoints %d\n", selsnappoints.size());
                if (selsnappoints.size()>1){ //keep only the best option
                   takendist = 1000;
                   for (laser_point=selsnappoints.begin(); laser_point!=selsnappoints.end(); laser_point++) {
                       if (laser_point->Attribute(PlaneNumberTag)<takendist){
                           takendist = laser_point->Attribute(PlaneNumberTag);
                           selsnappoint = *laser_point;
                           selsnappoint.SetAttribute(PulseLengthTag, laser_point->Attribute(PulseLengthTag));
                           fprintf(statfile, "taken dist = %d\n", takendist);
                           }
                       } 
                }
                else {
                     selsnappoint = selsnappoints[0];
                     selsnappoint.SetAttribute(PulseLengthTag, (selsnappoints.begin())->Attribute(PulseLengthTag));
                     fprintf(statfile, "taken pos %4.2f, %4.2f\n", selsnappoint.X(), selsnappoint.Y());
                     }
                selsnappoint.SetAttribute(LabelTag, 3);
                
                maplaserpoints.push_back(selsnappoint);
                newpointnumber++;
                newpos2 = Position3D(selsnappoint.X(), selsnappoint.Y(), selsnappoint.Z()); //this is the higher new point (near to defpoint, pos23d)
                newedgepoint = ObjectPoint(newpos2.GetX(), newpos2.GetY(), newpos2.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                intsect_points.push_back(newedgepoint);
                extendridgepoint.X() = endpoint->Number();
                extendridgepoint.Y() = newpointnumber;
                extendridgepoint.Z() = *segment_number;
                extendridgepoint.Residual() = selsnappoint.Residual();
                extendridgepoint.Attribute(PulseLengthTag) = selsnappoint.Attribute(PulseLengthTag);
                extendridgepoints.push_back(extendridgepoint);
                fprintf(statfile, "point number %d on map line taken is best position (%4.2f, %4.2f)\n", newpointnumber, newpos2.GetX(), newpos2.GetY());
                }
                    
    //                loweredsegment = true;              
                if (!supline2.IsHorizontal(10*PI/180)){ // supline 1 horizontal, supline 2 not horizontal
                  fprintf(statfile, "case a!b\n");
                  newpos = supline2.DetPositionZ(goal_z);
                  if ((newpos-pos23d).Length()>0.1){
                   newpointnumber++;
                   fprintf(statfile, "point number %d created between point %d and point %d\n", newpointnumber, endpoint->Number(), defpoint.Number());
                   newedgepoint = ObjectPoint(newpos.GetX(), newpos.GetY(), newpos.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                   top.push_back(PointNumber(newpointnumber));
                   intsect_points.push_back(newedgepoint);
                   top.SetAttribute(SegmentLabel, *segment_number);
                   top.SetAttribute(LineLabelTag, 4);
                   selmergedsegmentlines.push_back(top);
                   top.clear(); top.Initialise();
                   top.push_back(PointNumber(newpointnumber));
                   loweredsegment = true;              
                   }
                   }
                  else { //if supline 1 is horizontal, and supline 2 is horizontal
                    fprintf(statfile, "case ab\n");

                    supdumpos2=Position2D(suppos2.GetX(), suppos2.GetY());
//                    dumpos2 = Position2D(pos22d.GetX(), pos22d.GetY());
                    line = Line2D(supdumpos2, pos22d);
                    perpline = line.PerpendicularLine(pos22d);
                    dumpos2 = perpline.Project(dumpos1);
                    dumpos3d = Position3D(dumpos2.GetX(), dumpos2.GetY(), centrepoint.Z());
                    projpos3d = plane.Project(dumpos3d);
                    line3d2 = Line3D(pos23d, projpos3d);
                    newpos = line3d2.DetPositionZ(goal_z);
                    newpointnumber++;
                    newedgepoint = ObjectPoint(newpos.GetX(), newpos.GetY(), newpos.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                    top.push_back(PointNumber(newpointnumber));
                    intsect_points.push_back(newedgepoint);
                    top.SetAttribute(SegmentLabel, *segment_number);
                    top.SetAttribute(LineLabelTag, 4);
                    selmergedsegmentlines.push_back(top);
                    top.clear(); top.Initialise();
                    top.push_back(PointNumber(newpointnumber));  
     //               loweredsegment = true;        
                    extrapointadded = true;
                    
                               selnearbymaplines = nearbymaplines.SelectAttributedLines(SegmentLabel, *segment_number);
                fprintf(statfile, "looking for intersection with point: %d\n", defpoint.Number());
                searchdist = 1;
                iter = 0;
                iter2 = 0;

                do {
                
                for (selnearbymapline = selnearbymaplines.begin(), linenumber=0; selnearbymapline != selnearbymaplines.end(); selnearbymapline++, linenumber++){
                    node=selnearbymapline->begin();
                    nbmpoint1 = *(nearbymappoints.PointIterator(*node));
                    nbmppos12d = Position2D(nbmpoint1.X(), nbmpoint1.Y());
                    node++;
                    nbmpoint2 = *(nearbymappoints.PointIterator(*node));
                    nbmppos22d = Position2D(nbmpoint2.X(), nbmpoint2.Y());
                    nbmline = Line2D(nbmppos12d, nbmppos22d);
                    anglenbmline = atan2(nbmppos22d[0]-nbmppos12d[0], nbmppos22d[1]-nbmppos12d[1])*180/PI;
      //              fprintf(statfile, "angle = %4.2f, anglenbm %4.2f, absdiff %4.2f\n", angleline, anglenbmline, fabs(angleline - anglenbmline));    
                    hundredangle = int(100*anglenbmline);
                    if (supline2.FindIntersection(nbmline, projposonmap1)){
                      if (pos23d.Distance(projposonmap1)<suppos2.Distance(projposonmap1) && pos23d.Distance(projposonmap1)<searchdist && fabs(angleline-anglenbmline)>20 && fabs(fabs(angleline-anglenbmline)-180)>20){
                          maplaserpoint.X() = projposonmap1.GetX();
                          maplaserpoint.Y() = projposonmap1.GetY();
                          maplaserpoint.Z() = projposonmap1.GetZ();
                          nbmppos13d = Position3D(nbmpoint1.X(), nbmpoint1.Y(), maplaserpoint.Z());
                          nbmppos23d = Position3D(nbmpoint2.X(), nbmpoint2.Y(), maplaserpoint.Z());
                          nbmline3d = Line3D(nbmppos13d, nbmppos23d);
                          hundreddistance = int(100*pos23d.Distance(projposonmap1));

//                          projposonmap2d = Position2D(maplaserpoint.X(), maplaserpoint.Y());
                          scalar = nbmline3d.Scalar(projposonmap1);                          
                          nbmlength = (nbmppos13d - nbmppos23d).Length();
                          scalar = 1*scalar/nbmlength;
                          scalar3d = supline2.Scalar(projposonmap1);
                          linelength = (pos23d - suppos2).Length();
                          scalar3d = 1*scalar3d/linelength;
      //                    fprintf(statfile, "normalized scalar nbm line = %4.2f (length %4.2f), norm scalar 3d line %4.2f (length %4.2f)\n", scalar/nbmlength, nbmlength, scalar3d/linelength, linelength);    
                          maplaserpoint.SetAttribute(SegmentNumberTag, *segment_number);
                          maplaserpoint.SetAttribute(LabelTag, defpoint.Number());
                          if (scalar3d>=1 || scalar3d<=0){ //outside 3dline
                                       if (scalar>=0 && scalar<=1) { //but at map segment
                                           iter++;
                                           maplaserpoint.Attribute(PlaneNumberTag) = hundreddistance;
                                           }
                                       else {
                                            maplaserpoint.Residual() = 0.3; // in prolongated line of map segment...
                                            iter2++;
                                            maplaserpoint.Attribute(PlaneNumberTag) = hundreddistance;
                                            }
                          }
                          else  {
                                maplaserpoint.Residual() = 0.6;
                                iter++; //stop looking for other other points. the line allready exceeded the mapline
                                maplaserpoint.Attribute(PlaneNumberTag) = hundreddistance;
                                }
                        //  maplaserpoint.SetAttribute(PulseLengthTag, hundredangle); // TRICK transfer direction to PulseLengthTag
                          maplaserpoint.SetAttribute(PulseLengthTag, linenumber); // TRICK transfer linenumber to pulselengthtag
                          maplaserpoints.push_back(maplaserpoint);
                        }                
                     }
                }
                segmaplaserpoints.ErasePoints();
                segmaplaserpoints.AddTaggedPoints(maplaserpoints, *segment_number, SegmentNumberTag); //select points that belong to this segment
                selmaplaserpoints.ErasePoints();
                selmaplaserpoints.AddTaggedPoints(segmaplaserpoints, defpoint.Number(), LabelTag); //select points that belong to this endpoint
                searchdist = searchdist+1;
               // } while (selmaplaserpoints.size()==0 && searchdist <10);
                 } while (iter==0 && iter2<2 && searchdist <max_distance_line2map);
  //              } while (iter==0 && iter2<2 && searchdist <10);
                // choose best option from intersection with mapline
                
                  selsnappoints.ErasePoints();
                  found = false;
                  takendist = 0;
                  for (laser_point=selmaplaserpoints.begin(); laser_point!=selmaplaserpoints.end(); laser_point++) {
                      if (laser_point->Residual()<0.2 && !found){
                        selsnappoints.push_back(*laser_point);
                        found = true;
                        takendist  = laser_point->Attribute(PlaneNumberTag);
                        }
                  }
                  if (selsnappoints.size()==0) takendist =1000; //dist in cm
                  for (laser_point=selmaplaserpoints.begin(); laser_point!=selmaplaserpoints.end(); laser_point++) {
                      if (laser_point->Residual()<0.5 && laser_point->Attribute(PlaneNumberTag)<0.5*takendist){
                        selsnappoints.push_back(*laser_point);
                        found = true;
                        }
                  }                             
//                  }
                  if (selsnappoints.size()==0){
                  for (laser_point=selmaplaserpoints.begin(); laser_point!=selmaplaserpoints.end(); laser_point++) {
                      if (laser_point->Residual()<1 && !found){
        //                selsnappoints.push_back(*laser_point);
        //                found = true;
                        }
                  }                             
                  }
                                  
                if (found) {
                if (selsnappoints.size()>1){ //keep only the best option
                   takendist = 1000;
                   for (laser_point=selsnappoints.begin(); laser_point!=selsnappoints.end(); laser_point++) {
                       if (laser_point->Attribute(PlaneNumberTag)<takendist){
                           takendist = laser_point->Attribute(PlaneNumberTag);
                           selsnappoint = *laser_point;
                           selsnappoint.SetAttribute(PulseLengthTag, laser_point->Attribute(PulseLengthTag));
                           }
                       } 
                }
                else {
                     selsnappoint = *(selsnappoints.begin());
                     selsnappoint.SetAttribute(PulseLengthTag, (selsnappoints.begin())->Attribute(PulseLengthTag));
                     }

                selsnappoint.SetAttribute(LabelTag, 3);
                maplaserpoints.push_back(selsnappoint);
                newpointnumber++;
                newpos2 = Position3D(selsnappoint.X(), selsnappoint.Y(), selsnappoint.Z()); //this is the higher new point (near to defpoint, pos23d)
                newedgepoint = ObjectPoint(newpos2.GetX(), newpos2.GetY(), newpos2.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                intsect_points.push_back(newedgepoint);
                extendridgepoint.X() = defpoint.Number();
                extendridgepoint.Y() = newpointnumber;
                extendridgepoint.Z() = *segment_number;
                extendridgepoint.Residual() = selsnappoint.Residual();
                extendridgepoint.Attribute(PulseLengthTag) = selsnappoint.Attribute(PulseLengthTag);
                extendridgepoints.push_back(extendridgepoint);
                }
                    
                         
                  }
                } //end else (if supline 1 is horizontal)
                     

            }
            
            if (extrapointadded) {
                                 top.SetAttribute(LineLabelTag, 1);
                                 if (plane.IsHorizontal(10*PI/180)) top.SetAttribute(LineLabelTag, 2);
               //                  top.clear(); top.Initialise();
               //                  top.push_back(PointNumber(endpoint->Number()));
               //                  top.push_back(PointNumber(defpoint.Number()));
               //                  top.SetAttribute(SegmentLabel, *segment_number);
               //                  top.SetAttribute(LineLabelTag, 0);
               //                  selmergedsegmentlines.push_back(top);
                                 }
            else {
                 if (loweredsegment){
                      top.SetAttribute(LineLabelTag, 55);
                      }
                 else{               
                 fprintf(statfile, "defpoint %d is pushed back\n", defpoint.Number()); 
                 top.SetAttribute(LineLabelTag, 4);
                 }                 
                 }
               top.SetAttribute(SegmentLabel, *segment_number);
               top.push_back(PointNumber(defpoint.Number()));
               selmergedsegmentlines.push_back(top);
       //     } // end else (if only one 1 intersection line is in the segment..)
         //  } 
           loop++;
           fprintf(statfile, "loop : %d \n", loop); 
         }// while (loop * 2 < endpoints.size());
        // } // end if even endpoints
         } // end if selshapelines>1
         mergedsegmentlines.insert(mergedsegmentlines.end(), selmergedsegmentlines.begin(), selmergedsegmentlines.end());
         //keep_new_lines.insert(keep_new_lines.end(), selmergedsegmentlines.begin(), selmergedsegmentlines.end());
        
        
          
        }
       }
       selmergedsegmentlines.SetAttribute(BuildingNumberTag, building);
       selsegshapelines.SetAttribute(BuildingNumberTag, building);
       combinedlines.insert(combinedlines.end(), selmergedsegmentlines.begin(), selmergedsegmentlines.end());
       combinedlines.insert(combinedlines.end(), selsegshapelines.begin(), selsegshapelines.end());
       selsegshapelinespoly = combinedlines.SelectAttributedLines(SegmentLabel, *segment_number);
       fprintf(statfile,"size of selsegshapelinespoly: %d\n", selsegshapelinespoly.size());
    //   for (segshapepoly=selsegshapelinespoly.begin(); segshapepoly!=selsegshapelinespoly.end(); segshapepoly++){
            
       if (selsegshapelinespoly.size()>2) polygonline = selsegshapelinespoly.ReturnClosedPolygon(intsect_points);
       polygonline.SetAttribute(SegmentLabel, *segment_number);
       polygonline.SetAttribute(BuildingNumberTag, building);
       polygonlines.push_back(polygonline);
       }
       lastsegmentnumber = *segment_number;
   }
mergedsegmentlines.Write("mergedsegmentlines.top", false);
combinedlines.Write("combinedlines.top", false);
intsect_points.Write("combinedlines.objpts");
ridgelines = combinedlines.SelectAttributedLines(LineLabelTag, 6);
ridgelines.SetAttribute(LineLabelTag, 5);
ridgelines.Write("ridgelines.top", false);
intsect_points.Write("ridgelines.objpts");
//polygonlines.MakeCounterClockWise(intsect_points);
polygonlines.Write("polygonlines.top", false);
intsect_points.Write("polygonlines.objpts");
//return;
        for (sel_match_line2 = keep_new_lines.begin(); sel_match_line2 !=keep_new_lines.end(); sel_match_line2++){
               if (sel_match_line2->Attribute(TargetNumberTag)==13 ){//||sel_match_line2->Attribute(TargetNumberTag)==22){
                   keep_new_lines.erase(sel_match_line2);
                   sel_match_line2--;
               }
            }
keep_new_lines.Write(map_topology_output, false);          
intsect_points.Write(map_points_output);

fprintf(statfile,"Size before %d ", extendridgepoints.size());

extendridgepoints.SortOnCoordinates();
for (laser_point = extendridgepoints.begin(); laser_point != extendridgepoints.end()-1; laser_point++){
  check_point = laser_point;
  check_point++;
  if (check_point->X()==laser_point->X()) {
    if (check_point->Residual()>=laser_point->Residual()){
        check_point->Attribute(LabelTag) = 99;
    }
    else laser_point->Attribute(LabelTag)=99;
  }
}
extendridgepoints.RemoveTaggedPoints(99, LabelTag);
fprintf(statfile,"and after %d removing double points", extendridgepoints.size());
// this part to extend the lines from points that have been snapped to the mapline

for (check_point = extendridgepoints.begin(); check_point != extendridgepoints.end(); check_point++){
     counterlines = combinedlines.ReturnLinesWithPoint(int(check_point->X()));
     fprintf(statfile,"point %d, # of lines %d\n", int(check_point->X()), counterlines.size());
     for (map_line = counterlines.begin(); map_line!=counterlines.end(); map_line++){
         for (node2 = map_line->begin(); node2!=map_line->end(); node2++){
           if (node2->Number()==int(check_point->X())){ //look for the point that has to be changed
              node2->Number()= int(check_point->Y());
              fprintf(statfile,"changed point %d into %d on line %d\n", int(check_point->X()), node2->Number(), map_line->Number());
           }
         }
     }
 /*    counterlines.erase(counterlines.begin(), counterlines.end());
     counterlines = polygonlines.ReturnLinesWithPoint(int(check_point->X()));
     for (map_line = counterlines.begin(); map_line!=counterlines.end(); map_line++){
         for (node2 = map_line->begin(); node2!=map_line->end(); node2++){
           if (node2->Number()==int(check_point->X())){ //look for the point that has to be changed
              node2->Number()= int(check_point->Y());
           }
         }
     }
     counterlines.erase(counterlines.begin(), counterlines.end());
     counterlines = mergedsegmentlines.ReturnLinesWithPoint(int(check_point->X()));
     for (map_line = counterlines.begin(); map_line!=counterlines.end(); map_line++){
         for (node2 = map_line->begin(); node2!=map_line->end(); node2++){
           if (node2->Number()==int(check_point->X())){ //look for the point that has to be changed
              node2->Number()= int(check_point->Y());
           }
         }
     }
   */      
     if (counterlines.size()>0){
//         selcounterlines2 = counterlines.SelectAttributedLines(SegmentLabel, int(check_point->Z()));
         selcounterlines = counterlines.SelectAttributedLines(LineLabelTag, 1); //change also the positions of the other point on the line, only for lines with label 1
         suppos1 = (intsect_points.PointIterator(int(check_point->X())))->Position3DRef();
         suppos2 = (intsect_points.PointIterator(int(check_point->Y())))->Position3DRef();
         pos12d = (intsect_points.PointIterator(int(check_point->X())))->vect2D();
         pos22d = (intsect_points.PointIterator(int(check_point->Y())))->vect2D();
         for (map_line = selcounterlines.begin(); map_line!=selcounterlines.end(); map_line++){
           for (node2 = map_line->begin(); node2!=map_line->end(); node2++){
             if (node2->Number()!=int(check_point->Y())){ //look for the other point of line segment
                 oldpoint = *(intsect_points.PointIterator(*node2));
                 goal_z = oldpoint.Z(); //keep old z-value;
                 plane = laser_points.FitPlane(map_line->Attribute(SegmentLabel), map_line->Attribute(SegmentLabel), SegmentNumberTag);
                 seg_laser_points = laser_points.SelectTagValue(SegmentNumberTag, map_line->Attribute(SegmentLabel));
                 pnl = seg_laser_points.TaggedPointNumberList(SegmentNumberTag, map_line->Attribute(SegmentLabel));
                 centrepoint = seg_laser_points.Centroid(pnl, newpointnumber);
                 //dumpos1 = Position2D(centrepoint.X(), centrepoint.Y());
                 dumpos1 = Position2D(oldpoint.X(), oldpoint.Y());
                 line = Line2D(pos12d, pos22d);
                 angleline = atan2(pos22d[0]-pos12d[0], pos22d[1]-pos12d[1])*180/PI;
                 selnearbymaplines = nearbymaplines.SelectAttributedLines(SegmentLabel, int(check_point->Z()));
                 nearbymapline = selnearbymaplines[check_point->Attribute(PulseLengthTag)];
                 node=nearbymapline.begin();
                 nbmpoint1 = *(nearbymappoints.PointIterator(*node));
                 nbmppos12d = Position2D(nbmpoint1.X(), nbmpoint1.Y());
                 fprintf(statfile, "1: x,y = %4.2f, %4.2f\n", nbmpoint1.X(), nbmpoint1.Y());
                 node++;
                 nbmpoint2 = *(nearbymappoints.PointIterator(*node));
                 nbmppos22d = Position2D(nbmpoint2.X(), nbmpoint2.Y());
                 fprintf(statfile, "2: x,y = %4.2f, %4.2f\n", nbmpoint2.X(), nbmpoint2.Y());
                 anglenbmline = atan2(nbmppos22d[0]-nbmppos12d[0], nbmppos22d[1]-nbmppos12d[1])*180/PI;
                 
                 fprintf(statfile, "segment: %d (%d) point %d, angle = %4.2f, anglenbm %4.2f, absdiff %4.2f\n", int(check_point->Z()), map_line->Attribute(SegmentLabel), node2->Number(), angleline, anglenbmline, fabs(angleline - anglenbmline));    
              //   if (check_point->Residual()<0.2 && (fabs(angleline - anglenbmline)>70 && fabs(angleline - anglenbmline)<110) || (fabs(angleline - anglenbmline)>250 && fabs(angleline - anglenbmline)<290)){
                   if ((fabs(angleline - anglenbmline)>70 && fabs(angleline - anglenbmline)<110) || (fabs(angleline - anglenbmline)>250 && fabs(angleline - anglenbmline)<290)){
                   perpline = Line2D(nbmppos12d, nbmppos22d);
                   fprintf(statfile, "dir of map line taken\n");
                   }
                   else{
                      perpline = line.PerpendicularLine(pos22d); //perp line through new point
                      fprintf(statfile, "dir of perpendicular line taken\n");
                      }
                 dumpos2 = perpline.Project(dumpos1);
                 dumpos3d = Position3D(dumpos2.GetX(), dumpos2.GetY(), goal_z);
                 projpos3d = plane.Project(dumpos3d);
                 line3d2 = Line3D(suppos2, projpos3d);
                 newpos = line3d2.DetPositionZ(goal_z);
                 newpointnumber++;
                 newedgepoint = ObjectPoint(newpos.GetX(), newpos.GetY(), newpos.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                 intsect_points.push_back(newedgepoint);
                 newextendfacepoint.X() = node2->Number();
                 newextendfacepoint.Y() = newpointnumber;
                 newextendfacepoint.Z() = map_line->Attribute(SegmentLabel);
                 newextendfacepoints.push_back(newextendfacepoint);
              }
            }
          }
       }
}

// and now for the new extend positions...
 for (sel_match_line = combinedlines.begin(); sel_match_line !=combinedlines.end(); sel_match_line++){
     for (node=sel_match_line->begin(); node!=sel_match_line->end();node++){
       for (check_point = extendridgepoints.begin(); check_point != extendridgepoints.end(); check_point++){
           if (node->Number()==int(check_point->X())) {
              node->Number() = int(check_point->Y());
              fprintf(statfile,"changed point %d into %d on segment %d\n", int(check_point->X()), int(check_point->Y()), int(check_point->Z()));
           }
       }
       for (check_point = newextendfacepoints.begin(); check_point != newextendfacepoints.end(); check_point++){
           if (node->Number()==int(check_point->X())) {
              node->Number() = int(check_point->Y());
              fprintf(statfile,"changed point %d into %d on segment %d\n", int(check_point->X()), int(check_point->Y()), int(check_point->Z()));
           }
       }
     }
 }

for (sel_match_line = polygonlines.begin(); sel_match_line !=polygonlines.end(); sel_match_line++){
     for (node=sel_match_line->begin(); node!=sel_match_line->end();node++){
       for (check_point = extendridgepoints.begin(); check_point != extendridgepoints.end(); check_point++){
           if (node->Number()==int(check_point->X())) {
              node->Number() = int(check_point->Y());
           }
       }
       for (check_point = newextendfacepoints.begin(); check_point != newextendfacepoints.end(); check_point++){
           if (node->Number()==int(check_point->X())) {
              node->Number() = int(check_point->Y());
           }
       }
     }
  }        
for (sel_match_line = mergedsegmentlines.begin(); sel_match_line !=mergedsegmentlines.end(); sel_match_line++){
     for (node=sel_match_line->begin(); node!=sel_match_line->end();node++){
       for (check_point = extendridgepoints.begin(); check_point != extendridgepoints.end(); check_point++){
           if (node->Number()==int(check_point->X())) {
              node->Number() = int(check_point->Y());
           }
       }
       for (check_point = newextendfacepoints.begin(); check_point != newextendfacepoints.end(); check_point++){
           if (node->Number()==int(check_point->X())) {
              node->Number() = int(check_point->Y());
           }
       }
     }
  }        
maplaserpoints.Write("extendridgepoints.laser", false);
mergedsegmentlines.Write("mergedsegmentlines_afteradjstm.top", false);
combinedlines.Write("combinedlines_afteradjstm.top", false);
intsect_points.Write("combinedlines_afteradjstm.objpts");
extendfacepoints.ErasePoints();
newextendfacepoints.ErasePoints();
polygonlines.Write("polygonlines.top", false);
intsect_points.Write("polygonlines.objpts");
//return;

} // end if mergesegmentinfo

//  return;
//}

dormerreco = true;
if (dormerreco){
printf("start reconstructing dormers...\n");

   ObjectPoints                 endpoints;
   ObjectPoints::iterator       endpoint;
   ObjectPoint                  newedgepoint, first_point, sec_point;
   int                          newpointnumber, success, projpoint1, projpoint2,
                                dormpoint1, dormpoint2, buildingnr;
   Plane                        plane, supplane;
   Line2D                       line, perpline;
   Position2D                   pos12d, pos22d, dumpos1, dumpos2;
   Line3D                       line3d;
   Position3D                   pos1, pos2, newpos, dumpos3d, projpos3d, pos23d;
   LineTopology                 side;
   double                       dx, dy, dz, factor, minz, maxz, goal_z, goaldiff,
                                PI;
   int                          supsegmentnumber, dormersegmentnumber;
     
     newpointnumber = intsect_points.HighestPointNumber().Number()+1;                           
     scoresel_matchresults = matchresults.SelectAttributedLines(TargetNumberTag, 22);
                 
    for (sel_match_line2 = scoresel_matchresults.begin(); sel_match_line2 !=scoresel_matchresults.end(); sel_match_line2++){
         sel_int_lines = intsect_lines.SelectAttributedLines(HypoNumberTag, sel_match_line2->Number());
         fprintf(statfile,"size of int lines: %d dormer segments : ", sel_int_lines.size());
         endpoints = sel_int_lines.ReturnEndPointsOfTopologies(intsect_points);
         endpoint = endpoints.begin();
         pos1 = endpoint->Position3DRef();
         pos12d = Position2D(endpoint->X(), endpoint->Y());
         top.clear(); top.Initialise();
         top.push_back(PointNumber(endpoint->Number()));
         first_point = *endpoint;
         endpoint++;
         pos23d = endpoint->Position3DRef();
         pos22d = Position2D(endpoint->X(), endpoint->Y());
         line = Line2D(pos12d, pos22d);
         line3d = Line3D(pos23d, pos1);
         sec_point = *endpoint;
         node=sel_match_line2->begin(); //first segment, biggest
         fprintf(statfile,"%d ", node->Number());
         supsegmentnumber = node->Number();
         node++; //second segment, dormer segment...
         fprintf(statfile,"%d ", node->Number());
         dormersegmentnumber = node->Number();
         
         sel_laser_points.ErasePoints();
         sel_laser_points = laser_points.SelectTagValue(SegmentNumberTag, dormersegmentnumber);
         buildingnr = sel_laser_points.MostFrequentAttributeValue(PolygonNumberTag, count);
         bool switchsegment = false;
         if (sel_laser_points[0].Label()!=1600) switchsegment = true;
         if (switchsegment){
                            int dum;
                            dum = dormersegmentnumber;
                            dormersegmentnumber = supsegmentnumber;
                            supsegmentnumber = dum;
                            }
         maxdist = 0;
         for (laser_point=sel_laser_points.begin(); laser_point!=sel_laser_points.end()-1; laser_point++) {
           dist = line.DistanceToPoint(laser_point->Position2DOnly());
           if (dist > maxdist) {
               maxdist = dist;
               dumpos1 = laser_point->Position2DOnly();
               }
           }
         perpline = line.PerpendicularLine(pos12d);
         dumpos2 = perpline.Project(dumpos1);
         plane = laser_points.FitPlane(dormersegmentnumber, dormersegmentnumber, SegmentNumberTag);
         supplane = laser_points.FitPlane(supsegmentnumber, supsegmentnumber, SegmentNumberTag);
         dumpos3d = Position3D(dumpos2.GetX(), dumpos2.GetY(), 0);
         if (plane.Z_At(dumpos3d.GetX(), dumpos3d.GetY(),&success)<pos1.GetZ()+0.5 && plane.Z_At(dumpos3d.GetX(), dumpos3d.GetY(),&success)> supplane.Z_At(dumpos3d.GetX(), dumpos3d.GetY(),&success)){ //if corner of dormer is more than 0.5 m higher than intersection, strange shape, do not reconstruct...
           projpos3d = plane.Project(dumpos3d);       
           newpos = projpos3d;
           newpointnumber++;
           newedgepoint = ObjectPoint(dumpos3d.GetX(), dumpos3d.GetY(), plane.Z_At(dumpos3d.GetX(), dumpos3d.GetY(),&success), newpointnumber, 0,0,0,0,0,0); 
           dormpoint1 = newpointnumber;
           top.push_back(PointNumber(dormpoint1));
           intsect_points.push_back(newedgepoint);
           newpointnumber++;
           newedgepoint = ObjectPoint(dumpos3d.GetX(), dumpos3d.GetY(), supplane.Z_At(dumpos3d.GetX(), dumpos3d.GetY(),&success), newpointnumber, 0,0,0,0,0,0); 
           projpoint1 = newpointnumber;
           intsect_points.push_back(newedgepoint);
                  
         perpline = line.PerpendicularLine(pos22d);
         dumpos2 = perpline.Project(dumpos1);
         dumpos3d = Position3D(dumpos2.GetX(), dumpos2.GetY(), 0);
         projpos3d = plane.Project(dumpos3d);
         newpos = projpos3d;
         newpointnumber++;
         newedgepoint = ObjectPoint( dumpos3d.GetX(),  dumpos3d.GetY(),  plane.Z_At(dumpos3d.GetX(), dumpos3d.GetY(),&success), newpointnumber, 0,0,0,0,0,0);          
         dormpoint2 = newpointnumber;
         top.push_back(PointNumber(dormpoint2));
         intsect_points.push_back(newedgepoint);
         
         newpointnumber++;
         newedgepoint = ObjectPoint(dumpos3d.GetX(), dumpos3d.GetY(), supplane.Z_At(dumpos3d.GetX(), dumpos3d.GetY(),&success), newpointnumber, 0,0,0,0,0,0); 
         projpoint2 = newpointnumber;
         intsect_points.push_back(newedgepoint);
         
         top.push_back(PointNumber(sec_point.Number()));
         top.push_back(PointNumber(first_point.Number()));
         top.SetAttribute(SegmentLabel, dormersegmentnumber);
         top.SetAttribute(LineLabelTag, 10);
         top.SetAttribute(BuildingNumberTag, buildingnr);
         dormerlines.push_back(top);
         
         side.clear(); side.Initialise();
         side.push_back(PointNumber(dormpoint1));
         side.push_back(PointNumber(dormpoint2));
         side.SetAttribute(SegmentLabel, dormersegmentnumber);
         side.SetAttribute(LineLabelTag, 12);
         side.SetAttribute(BuildingNumberTag, buildingnr);
         dormerlinestofitto.push_back(side);
         
         side.clear(); side.Initialise();
         side.push_back(PointNumber(first_point.Number()));
         side.push_back(PointNumber(dormpoint1));
         side.push_back(PointNumber(projpoint1));
         side.push_back(PointNumber(first_point.Number()));
         side.SetAttribute(SegmentLabel, dormersegmentnumber);
         side.SetAttribute(LineLabelTag, 11);
         side.SetAttribute(BuildingNumberTag, buildingnr);
//         dormerlines.push_back(side);

         side.clear(); side.Initialise();
         side.push_back(PointNumber(sec_point.Number()));
         side.push_back(PointNumber(dormpoint2));
         side.push_back(PointNumber(projpoint2));
         side.push_back(PointNumber(sec_point.Number()));
         side.SetAttribute(SegmentLabel, dormersegmentnumber);
         side.SetAttribute(LineLabelTag, 11);
         side.SetAttribute(BuildingNumberTag, buildingnr);
//         dormerlines.push_back(side);

         side.clear(); side.Initialise();
         side.push_back(PointNumber(dormpoint1));
         side.push_back(PointNumber(dormpoint2));
         side.push_back(PointNumber(projpoint2));
         side.push_back(PointNumber(projpoint1));
         side.push_back(PointNumber(dormpoint1));
         side.SetAttribute(SegmentLabel, dormersegmentnumber);
         side.SetAttribute(LineLabelTag, 12);
         side.SetAttribute(BuildingNumberTag, buildingnr);
//         dormerlines.push_back(side);
         }
    }
    dormerlines.MakeCounterClockWise(intsect_points);
dormerlines.Write("dormersreco.top", false);
}


bool snapstepedge;
snapstepedge = true;
if (snapstepedge){
printf("start snapping stepedge to models...\n");

LineTopology      snappedline;
int               newpointnumber, success;
newpointnumber = intsect_points.HighestPointNumber().Number()+1;                           


    for (map_line = stepedgelines.begin(); map_line!=stepedgelines.end(); map_line++){
      selmergedsegmentlines = mergedsegmentlines.SelectAttributedLines(SegmentLabel, map_line->Attribute(SegmentLabel));
      if (selmergedsegmentlines.size()==0){
             selmergedsegmentlines = dormerlinestofitto.SelectAttributedLines(SegmentLabel, map_line->Attribute(SegmentLabel));
             fprintf(statfile,"dormer lines added to segment...\n");
                                           }
      if (selmergedsegmentlines.size()>0){
       mindist = 10000;
       found = false;
       for (sel_match_line = selmergedsegmentlines.begin(); sel_match_line!=selmergedsegmentlines.end();sel_match_line++){
           if (sel_match_line->Attribute(LineLabelTag)!=55){
            node=sel_match_line->begin();
            pos1 = (intsect_points.PointIterator(*node))->Position3DRef();
            node++;
            pos2 = (intsect_points.PointIterator(*node))->Position3DRef();
            line3d = Line3D(pos1, pos2);
            totdist=0;
            node=map_line->begin(); 
            suppos2 = (stepedgepoints.PointIterator(*node))->Position3DRef();
            dist = line3d.DistanceToPoint(suppos2);
            dumpos13d = line3d.Project(suppos2);
            totdist = totdist+dist; node++;
            suppos2 = (stepedgepoints.PointIterator(*node))->Position3DRef();
            dist = line3d.DistanceToPoint(suppos2);
            dumpos23d = line3d.Project(suppos2);
            totdist = totdist+dist;
            if (totdist<mindist){
                                 found=true;
                                mindist = totdist;
                                fprintf(statfile, "mindist = %4.2f for for segment: %d and mergedline %d\n", mindist, map_line->Attribute(SegmentLabel), sel_match_line->Number());
                                projpos1 = dumpos13d;
                                projpos2 = dumpos23d;
              } 
            }
         }
         if (found && mindist<2){
         top.clear(); top.Initialise();
         newpointnumber++;
         newedgepoint = ObjectPoint(projpos1.GetX(), projpos1.GetY(), projpos1.GetZ(), newpointnumber, 0,0,0,0,0,0); 
         stepedgepoint1 = newpointnumber;
         top.push_back(PointNumber(stepedgepoint1));
         intsect_points.push_back(newedgepoint);
         newpointnumber++;
         newedgepoint = ObjectPoint(projpos2.GetX(), projpos2.GetY(), projpos2.GetZ(), newpointnumber, 0,0,0,0,0,0); 
         stepedgepoint2 = newpointnumber;
         top.push_back(PointNumber(stepedgepoint2));
         intsect_points.push_back(newedgepoint);
         top.SetAttribute(SegmentLabel, map_line->Attribute(SegmentLabel));
         top.SetAttribute(SecondSegmentLabel, map_line->Attribute(SecondSegmentLabel));
         top.SetAttribute(LineLabelTag, 2);
         snappedstepedgelines.push_back(top);
         fprintf(statfile, "snapped step edge pushed back for high segment: %d\n", top.Attribute(SegmentLabel));
         
         // now reconstruct lower part by taking height of lower plane at same horizontal location
         lowerplane = laser_points.FitPlane(map_line->Attribute(SecondSegmentLabel), map_line->Attribute(SecondSegmentLabel), SegmentNumberTag);
         top.clear(); top.Initialise();
         newpointnumber++;
         newedgepoint = ObjectPoint(projpos1.GetX(), projpos1.GetY(), lowerplane.Z_At(projpos1.GetX(), projpos1.GetY(), &success), newpointnumber, 0,0,0,0,0,0); 
         stepedgepoint1 = newpointnumber;
         top.push_back(PointNumber(stepedgepoint1));
         intsect_points.push_back(newedgepoint);
         newpointnumber++;
         newedgepoint = ObjectPoint(projpos2.GetX(), projpos2.GetY(), lowerplane.Z_At(projpos2.GetX(), projpos2.GetY(), &success), newpointnumber, 0,0,0,0,0,0); 
         stepedgepoint2 = newpointnumber;
         top.push_back(PointNumber(stepedgepoint2));
         intsect_points.push_back(newedgepoint);
         top.SetAttribute(SegmentLabel, map_line->Attribute(SecondSegmentLabel)); //now turn around the segmentlabels ofcourse
         top.SetAttribute(SecondSegmentLabel, map_line->Attribute(SegmentLabel));
         top.SetAttribute(LineLabelTag, 3);
         snappedstepedgelines.push_back(top);
         fprintf(statfile, "snapped step edge pushed back for low segment: %d\n", top.Attribute(SegmentLabel));
         }
         else fprintf(statfile, "no nearby mergedsegment found for stepedgeline\n");
       }
       else {
            fprintf(statfile, "no mergedsegmentlines found for segment: %d\n", map_line->Attribute(SegmentLabel));
            // push back the fitted/original edge line... take original locations
            node=map_line->begin(); 
            projpos1 = (stepedgepoints.PointIterator(*node))->Position3DRef();
            node++;
            projpos2 = (stepedgepoints.PointIterator(*node))->Position3DRef();
         top.clear(); top.Initialise();
         newpointnumber++;
         newedgepoint = ObjectPoint(projpos1.GetX(), projpos1.GetY(), projpos1.GetZ(), newpointnumber, 0,0,0,0,0,0); 
         stepedgepoint1 = newpointnumber;
         top.push_back(PointNumber(stepedgepoint1));
         intsect_points.push_back(newedgepoint);
         newpointnumber++;
         newedgepoint = ObjectPoint(projpos2.GetX(), projpos2.GetY(), projpos2.GetZ(), newpointnumber, 0,0,0,0,0,0); 
         stepedgepoint2 = newpointnumber;
         top.push_back(PointNumber(stepedgepoint2));
         intsect_points.push_back(newedgepoint);
         top.SetAttribute(SegmentLabel, map_line->Attribute(SegmentLabel));
         top.SetAttribute(SecondSegmentLabel, map_line->Attribute(SecondSegmentLabel));
         top.SetAttribute(LineLabelTag, 2);
         snappedstepedgelines.push_back(top);
         fprintf(statfile, "original step edge pushed back for high segment: %d\n", top.Attribute(SegmentLabel));
         
         // now reconstruct lower part by taking height of lower plane at same horizontal location
         lowerplane = laser_points.FitPlane(map_line->Attribute(SecondSegmentLabel), map_line->Attribute(SecondSegmentLabel), SegmentNumberTag);
         top.clear(); top.Initialise();
         newpointnumber++;
         newedgepoint = ObjectPoint(projpos1.GetX(), projpos1.GetY(), lowerplane.Z_At(projpos1.GetX(), projpos1.GetY(), &success), newpointnumber, 0,0,0,0,0,0); 
         stepedgepoint1 = newpointnumber;
         top.push_back(PointNumber(stepedgepoint1));
         intsect_points.push_back(newedgepoint);
         newpointnumber++;
         newedgepoint = ObjectPoint(projpos2.GetX(), projpos2.GetY(), lowerplane.Z_At(projpos2.GetX(), projpos2.GetY(), &success), newpointnumber, 0,0,0,0,0,0); 
         stepedgepoint2 = newpointnumber;
         top.push_back(PointNumber(stepedgepoint2));
         intsect_points.push_back(newedgepoint);
         top.SetAttribute(SegmentLabel, map_line->Attribute(SecondSegmentLabel)); //now turn around the segmentlabels ofcourse
         top.SetAttribute(SecondSegmentLabel, map_line->Attribute(SegmentLabel));
         top.SetAttribute(LineLabelTag, 3);
         snappedstepedgelines.push_back(top);
         fprintf(statfile, "original step edge pushed back for low segment: %d\n", top.Attribute(SegmentLabel));
            
            
            
            }
    }                
  }//end snapstepedge
//combinedlines.insert(combinedlines.end(), snappedstepedgelines.begin(), snappedstepedgelines.end());

bool selectmapsegments;
selectmapsegments = true;
if (selectmapsegments){
double nbh_radius = 1;
int    dominantsegment, lastsegment, newpointnumber, success;
Position3D              lastpos, newpos;

LaserPoint                   buildingedgepoint;
LaserPoints                  edgebuildingpoints, temp_height_points, low_laser_points;
vector <int>                 lowsegments, selsegmentnumbers;
vector <int>::iterator       lowsegment;
bool                         used;

nearbymappoints.erase(nearbymappoints.begin(), nearbymappoints.end());
nearbymaplines.erase(nearbymaplines.begin(), nearbymaplines.end());
newpointnumber = intsect_points.HighestPointNumber().Number()+1;                           

densifiedmaplines = map_lines;
densifiedmappoints = map_points;
//densifiedmaplines.Densify(densifiedmappoints, 2);
//densifiedmappoints.RemoveDoublePoints(densifiedmaplines, 0.1);
low_laser_points.ErasePoints();
low_laser_points.AddTaggedPoints(laser_points, 1000, LabelTag); //only horizontal points
//low_laser_points = laser_points; //all points
lowsegments = stepedgelines.AttributedValues(SecondSegmentLabel);
for (map_line = densifiedmaplines.begin(); map_line!=densifiedmaplines.end(); map_line++){
    sel_laser_points.ErasePoints();
    selmaplaserpoints.ErasePoints();
    sel_laser_points = low_laser_points.SelectTagValue(PolygonNumberTag, map_line->Number());
    lastsegment = -1;
    for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
        new_point = *(densifiedmappoints.PointIterator(*node2));
        temp_height_points.ErasePoints();  
        newpos = (densifiedmappoints.PointIterator(*node2))->Position3DRef();
        for (laser_point=sel_laser_points.begin(); laser_point!=sel_laser_points.end(); laser_point++){
          dist = (laser_point->vect2D()-new_point.vect2D()).Length();   
            if (dist < nbh_radius) {
                temp_height_points.push_back(*laser_point);
                }
           }
        dominantsegment = temp_height_points.MostFrequentAttributeValue(SegmentNumberTag, count);
        used = false;
        selsegmentnumbers = temp_height_points.AttributeValues(SegmentNumberTag);
        for (segment_number = selsegmentnumbers.begin(); segment_number !=selsegmentnumbers.end(); segment_number++){
             buildingedgepoint = LaserPoint(new_point.X(), new_point.Y(), new_point.Z());
             plane = laser_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
             buildingedgepoint.Z() = plane.Z_At(new_point.X(), new_point.Y(), &success);
             buildingedgepoint.SetAttribute(LabelTag, *segment_number);
             buildingedgepoint.SetAttribute(SegmentNumberTag, *segment_number);
             edgebuildingpoints.push_back(buildingedgepoint);
             maplaserpoint.X() = node2->Number();
             maplaserpoint.Y() = *segment_number;
             maplaserpoint.Z() = 0;
             maplaserpoint.SetAttribute(SegmentNumberTag, *segment_number);
             maplaserpoint.SetAttribute(LabelTag, node2->Number());
             selmaplaserpoints.push_back(maplaserpoint);
        }
        if (selsegmentnumbers.size()>1){
             buildingedgepoint = LaserPoint(new_point.X(), new_point.Y(), new_point.Z());
             buildingedgepoint.Z() = 0;
             edgebuildingpoints.push_back(buildingedgepoint);
        }
        for (segment_number = segment_numbers.begin(); segment_number !=segment_numbers.end(); segment_number++){
            if (*segment_number==dominantsegment) {
              used = true;
              }
        }
        if (count>2 && lastsegment == dominantsegment){//&& used){
        
           lowerplane = laser_points.FitPlane(dominantsegment, dominantsegment, SegmentNumberTag);
           top.clear(); top.Initialise();
           newpointnumber++;
           newedgepoint = ObjectPoint(lastpos.GetX(), lastpos.GetY(), lowerplane.Z_At(lastpos.GetX(), lastpos.GetY(), &success), newpointnumber, 0,0,0,0,0,0); 
           top.push_back(PointNumber(newpointnumber));
           nearbymappoints.push_back(newedgepoint);
           newpointnumber++;
           newedgepoint = ObjectPoint(newpos.GetX(), newpos.GetY(), lowerplane.Z_At(newpos.GetX(), newpos.GetY(), &success), newpointnumber, 0,0,0,0,0,0); 
           top.push_back(PointNumber(newpointnumber));
           nearbymappoints.push_back(newedgepoint);
           top.SetAttribute(SegmentLabel, dominantsegment);
           top.SetAttribute(LineLabelTag, dominantsegment);
     //      nearbymaplines.push_back(top);            
         }
        lastsegment = dominantsegment;
        lastpos = newpos;
    }
    selsegmentnumbers = selmaplaserpoints.AttributeValues(SegmentNumberTag);
    
    for (segment_number = selsegmentnumbers.begin(); segment_number !=selsegmentnumbers.end(); segment_number++){
        segmaplaserpoints.ErasePoints();
        segmaplaserpoints.AddTaggedPoints(selmaplaserpoints, *segment_number, SegmentNumberTag);
        mapnodenumbers = segmaplaserpoints.AttributeValues(LabelTag);
        lowerplane = laser_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
        foundprevnode = false;
        for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
          foundnode = false;
          for (mapnodenumber = mapnodenumbers.begin(); mapnodenumber!=mapnodenumbers.end(); mapnodenumber++){
              if (node2->Number() == *mapnodenumber) {
                 foundnode = true;
                 newpos = (densifiedmappoints.PointIterator(*node2))->Position3DRef();
              }
          }
          if (foundnode && foundprevnode){
              top.clear(); top.Initialise();
              newpointnumber++;
              newedgepoint = ObjectPoint(lastpos.GetX(), lastpos.GetY(), lowerplane.Z_At(lastpos.GetX(), lastpos.GetY(), &success), newpointnumber, 0,0,0,0,0,0); 
              top.push_back(PointNumber(newpointnumber));
              nearbymappoints.push_back(newedgepoint);
              newpointnumber++;
              newedgepoint = ObjectPoint(newpos.GetX(), newpos.GetY(), lowerplane.Z_At(newpos.GetX(), newpos.GetY(), &success), newpointnumber, 0,0,0,0,0,0); 
              top.push_back(PointNumber(newpointnumber));
              nearbymappoints.push_back(newedgepoint);
              top.SetAttribute(SegmentLabel, *segment_number);
              top.SetAttribute(LineLabelTag, *segment_number);
              top.SetAttribute(BuildingNumberTag, map_line->Number());
              nearbymaplines.push_back(top); 
          }
          if (foundnode){
                         foundprevnode = true;
                         lastpos = newpos;
                         }
          else foundprevnode = false;
        }
    }
}
edgebuildingpoints.Write("edgebuildingpoints.laser", false);
nearbymappoints.RemoveDoublePoints(nearbymaplines, 0.1);
nearbymaplines.Write("nearbymaplines.top", false);
nearbymappoints.Write("nearbymaplines.objpts");
//return;

bool includesnappededge;
includesnappededge = true;
if (includesnappededge){
LineTopology      snappedline;
printf("Start including snapped step edges in segment outline...");
int               newpointnumber;
newpointnumber = intsect_points.HighestPointNumber().Number()+1;                           


    for (map_line = snappedstepedgelines.begin(); map_line!=snappedstepedgelines.end(); map_line++){
      if (map_line->Attribute(LineLabelTag)==3){ //only the lowest ones.. first look at the intersection lines and created lines from combinedlines
//       selmergedsegmentlines = mergedsegmentlines.SelectAttributedLines(SegmentLabel, map_line->Attribute(SegmentLabel));
       selmergedsegmentlines = combinedlines.SelectAttributedLines(SegmentLabel, map_line->Attribute(SegmentLabel)); 
       node= map_line->begin();
       dumpos1 = (intsect_points.PointIterator(*node))->vect2D();
       node++;
       dumpos2 = (intsect_points.PointIterator(*node))->vect2D();
       line = Line2D(dumpos1, dumpos2); //2d line of step edge
       if (selmergedsegmentlines.size()>0){ //go find a line about perpendicular to the snapped stepedge and extend this line to the stepedge
       mindist = 10000;
       for (sel_match_line = selmergedsegmentlines.begin(); sel_match_line!=selmergedsegmentlines.end();sel_match_line++){
           found = false;
           if (sel_match_line->Attribute(LineLabelTag)!=55 && sel_match_line->Attribute(LineLabelTag)!=1){        
            node=sel_match_line->begin(); 
            suppos1 = (intsect_points.PointIterator(*node))->Position3DRef();
            num1 = (intsect_points.PointIterator(*node))->Number();
            node++;
            suppos2 = (intsect_points.PointIterator(*node))->Position3DRef();
            num2 = (intsect_points.PointIterator(*node))->Number();
            line3d = Line3D(suppos1, suppos2);
            if (line3d.FindIntersection(line, projpos1)){
               if ((projpos1 - suppos1).Length()<2 || (projpos1 - suppos2).Length()<2){
                  found=true;
                  if ((projpos1 - suppos1).Length() < (projpos1 - suppos2).Length()){
                     keeppos1 = suppos1;
                     keeppos2 = projpos1;
                     extendfacepoint.X() = num1;
                     }
                     else {
                     keeppos1 = suppos2;
                     keeppos2 = projpos1;
                     extendfacepoint.X() = num2;
                     }
               }
            }        
      //      }
         if (found){
         top.clear(); top.Initialise();
         newpointnumber++;
         newedgepoint = ObjectPoint(keeppos1.GetX(), keeppos1.GetY(), keeppos1.GetZ(), newpointnumber, 0,0,0,0,0,0); 
         top.push_back(PointNumber(newpointnumber));
         intsect_points.push_back(newedgepoint);
         newpointnumber++;
         newedgepoint = ObjectPoint(keeppos2.GetX(), keeppos2.GetY(), keeppos2.GetZ(), newpointnumber, 0,0,0,0,0,0); 
         top.push_back(PointNumber(newpointnumber));
         intsect_points.push_back(newedgepoint);
         extendfacepoint.Y() = newpointnumber;
         extendfacepoint.Z() = map_line->Attribute(SegmentLabel);
         top.SetAttribute(SegmentLabel, map_line->Attribute(SegmentLabel));
         top.SetAttribute(LineLabelTag, 55); //label 55 indicates extension line
         newsnappedstepedgelines.push_back(top);
         extendfacepoints.push_back(extendfacepoint);
         }
         }
         }
         }
       else {
         fprintf(statfile, "no mergedsegmentlines found for segment: %d\n", map_line->Attribute(SegmentLabel));
         selnearbymaplines = nearbymaplines.SelectAttributedLines(SegmentLabel, map_line->Attribute(SegmentLabel));
         // now that there are no intersection lines and created lines, look for nearby map lines
         if (selnearbymaplines.size()>0){
           fprintf(statfile, "but %d nearby maplines found\n", selnearbymaplines.size());
           node=map_line->begin(); 
           suppos1 = (intsect_points.PointIterator(*node))->Position3DRef();
           num1 = (intsect_points.PointIterator(*node))->Number();
           node++;
           suppos2 = (intsect_points.PointIterator(*node))->Position3DRef();
           num2 = (intsect_points.PointIterator(*node))->Number();
           line3d = Line3D(suppos1, suppos2);                     //create 3dline from lower part of snapped stepedge
           for (selnearbymapline = selnearbymaplines.begin(), linenumber=0; selnearbymapline != selnearbymaplines.end(); selnearbymapline++, linenumber++){
                node= selnearbymapline->begin();
                nbmpoint1 = *(nearbymappoints.PointIterator(*node));
                nbmppos12d = Position2D(nbmpoint1.X(), nbmpoint1.Y());
                node++;
                nbmpoint2 = *(nearbymappoints.PointIterator(*node));
                nbmppos22d = Position2D(nbmpoint2.X(), nbmpoint2.Y());
                nbmline = Line2D(nbmppos12d, nbmppos22d);
                anglenbmline = atan2(nbmppos22d[0]-nbmppos12d[0], nbmppos22d[1]-nbmppos12d[1])*180/PI;
   //           fprintf(statfile, "angle = %4.2f, anglenbm %4.2f, absdiff %4.2f\n", angleline, anglenbmline, fabs(angleline - anglenbmline));    
                if (line3d.FindIntersection(nbmline, projposonmap1)){
                    if (suppos1.Distance(projposonmap1)<5 || suppos2.Distance(projposonmap1)<5){
                 
                    maplaserpoint.X() = projposonmap1.GetX();
                    maplaserpoint.Y() = projposonmap1.GetY();
                    maplaserpoint.Z() = projposonmap1.GetZ();
                    nbmppos13d = Position3D(nbmpoint1.X(), nbmpoint1.Y(), maplaserpoint.Z());
                    nbmppos23d = Position3D(nbmpoint2.X(), nbmpoint2.Y(), maplaserpoint.Z());
                    nbmline3d = Line3D(nbmppos13d, nbmppos23d);
//                          projposonmap2d = Position2D(maplaserpoint.X(), maplaserpoint.Y());
                    scalar = nbmline3d.Scalar(projposonmap1);                          
                    nbmlength = (nbmppos13d - nbmppos23d).Length();
                    scalar = 1*scalar/nbmlength;
                    scalar3d = line3d.Scalar(projposonmap1);
                    linelength = (suppos1 - suppos2).Length();
                    scalar3d = 1*scalar3d/linelength;
                    fprintf(statfile, "normalized scalar nbm line = %4.2f (length %4.2f), norm scalar 3d line %4.2f (length %4.2f)\n", scalar, nbmlength, scalar3d, linelength);    
                    maplaserpoint.SetAttribute(SegmentNumberTag, map_line->Attribute(SegmentLabel));
                    if (scalar3d>=0 && scalar3d<=1){ //inside 3dline
                        if (scalar>=0 && scalar<=1) { //inside map segment
                            maplaserpoint.Residual()= 0;
                        }
                        else maplaserpoint.Residual() = 0.3; // in prolongated line of map segment...
                     }
                     else  maplaserpoint.Residual() = 0.6;
                     maplaserpoint.SetAttribute(PulseLengthTag, linenumber); // TRICK transfer direction to PulseLengthTag
                     stepedgemaplaserpoints.push_back(maplaserpoint);
                  } 
                  }               
              }
              selmaplaserpoints.ErasePoints();
              selmaplaserpoints.AddTaggedPoints(stepedgemaplaserpoints, map_line->Attribute(SegmentLabel), SegmentNumberTag); //select points that belong to this segment
                  
       //    endpoints = selsegshapelines.ReturnEndPointsOfTopologies(nearbymappoints);
       //    fprintf(statfile,"size of endpoints: %d for segment %d\n", endpoints.size(), *segment_number);
         }
         else {
              fprintf(statfile, "and no nearby maplines\n");
         }
       }
       
       }
//       newsnappedstepedgelines.push_back(*map_line);
       }
    }                
  }//end includesnapedge

stepedgemaplaserpoints.Write("stepedgemaplaserpoints.laser", false);

int               newpointnumber;
newpointnumber = intsect_points.HighestPointNumber().Number()+1;                           


//combinedlines.insert(combinedlines.end(), newsnappedstepedgelines.begin(), newsnappedstepedgelines.end());
 for (sel_match_line = combinedlines.begin(); sel_match_line !=combinedlines.end(); sel_match_line++){
     for (node=sel_match_line->begin(); node!=sel_match_line->end();node++){
       for (check_point = extendfacepoints.begin(); check_point != extendfacepoints.end(); check_point++){
           if (node->Number()==int(check_point->X())) {
              node->Number() = int(check_point->Y());
              counterlines = mergedsegmentlines.ReturnLinesWithPoint(int(check_point->X()));
              if (counterlines.size()>0){
              selcounterlines = counterlines.SelectAttributedLines(LineLabelTag, 1);
              shapestepedgelines.insert(shapestepedgelines.end(), selcounterlines.begin(), selcounterlines.end());
              segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
              segment_numbers = selcounterlines.AttributedValues(SegmentLabel);
              for (segment_number = segment_numbers.begin(); segment_number!=segment_numbers.end(); segment_number++){
                  found = false;
                   for (laser_point = extendfacepoints.begin(); laser_point != extendfacepoints.end(); laser_point++){
                       if (*segment_number ==int(laser_point->Z())) {
                          found = true;
                       }
                   }
                   if (!found){
                         fprintf(statfile,"Segment %d not found in adjusted positions\n", *segment_number);   
                         suppos1 = (intsect_points.PointIterator(int(check_point->X())))->Position3DRef();
                         suppos2 = (intsect_points.PointIterator(int(check_point->Y())))->Position3DRef();
                         pos12d = (intsect_points.PointIterator(int(check_point->X())))->vect2D();
                         pos22d = (intsect_points.PointIterator(int(check_point->Y())))->vect2D();
                         for (map_line = selcounterlines.begin(); map_line!=selcounterlines.end(); map_line++){
                             if (map_line->Attribute(SegmentLabel)==*segment_number){
                               for (node2 = map_line->begin(); node2!=map_line->end(); node2++){
                                   if (node2->Number()!=int(check_point->X())){
                                     oldpoint = *(intsect_points.PointIterator(*node2));
                                     goal_z = oldpoint.Z(); //keep old z-value;
                                     plane = laser_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
                                     seg_laser_points = laser_points.SelectTagValue(SegmentNumberTag, *segment_number);
                                     pnl = seg_laser_points.TaggedPointNumberList(SegmentNumberTag, *segment_number);
                                     centrepoint = seg_laser_points.Centroid(pnl, newpointnumber);
                                     dumpos1 = Position2D(centrepoint.X(), centrepoint.Y());
                                     line = Line2D(pos12d, pos22d);
                                     perpline = line.PerpendicularLine(pos22d); //perp line through new point
                                     dumpos2 = perpline.Project(dumpos1);
                                     dumpos3d = Position3D(dumpos2.GetX(), dumpos2.GetY(), centrepoint.Z());
                                     projpos3d = plane.Project(dumpos3d);
                                     line3d2 = Line3D(suppos2, projpos3d);
                                     newpos = line3d2.DetPositionZ(goal_z);
                                     newpointnumber++;
                                     newedgepoint = ObjectPoint(newpos.GetX(), newpos.GetY(), newpos.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                                     intsect_points.push_back(newedgepoint);
                                     newextendfacepoint.X() = node2->Number();
                                     newextendfacepoint.Y() = newpointnumber;
                                     newextendfacepoint.Z() = *segment_number;
                                     newextendfacepoints.push_back(newextendfacepoint);
                                   }
                               }
                             }
                         }

                   }
              }
              fprintf(statfile,"Found %d lines of which %d labeled 1 on segment %d\n", counterlines.size(), selcounterlines.size(), int(check_point->Z()));
              }
              }
       }
     }
}
shapestepedgelines.Write("shapestepedgelines.top", false);
// and now for the new extend positions...
 for (sel_match_line = combinedlines.begin(); sel_match_line !=combinedlines.end(); sel_match_line++){
     for (node=sel_match_line->begin(); node!=sel_match_line->end();node++){
       for (check_point = newextendfacepoints.begin(); check_point != newextendfacepoints.end(); check_point++){
           if (node->Number()==int(check_point->X())) {
              node->Number() = int(check_point->Y());
              fprintf(statfile,"changed point %d into %d on segment %d\n", int(check_point->X()), int(check_point->Y()), int(check_point->Z()));
           }
       }
     }
 }
  for (sel_match_line = polygonlines.begin(); sel_match_line !=polygonlines.end(); sel_match_line++){
     for (node=sel_match_line->begin(); node!=sel_match_line->end();node++){
       for (check_point = extendfacepoints.begin(); check_point != extendfacepoints.end(); check_point++){
           if (node->Number()==int(check_point->X())) {
              node->Number() = int(check_point->Y());
           }
       }
       for (check_point = newextendfacepoints.begin(); check_point != newextendfacepoints.end(); check_point++){
           if (node->Number()==int(check_point->X())) {
              node->Number() = int(check_point->Y());
           }
       }
     }
  }
  for (sel_match_line = shapestepedgelines.begin(); sel_match_line !=shapestepedgelines.end(); sel_match_line++){
     for (node=sel_match_line->begin(); node!=sel_match_line->end();node++){
       for (check_point = extendfacepoints.begin(); check_point != extendfacepoints.end(); check_point++){
           if (node->Number()==int(check_point->X())) {
              node->Number() = int(check_point->Y());
           }
       }
       for (check_point = newextendfacepoints.begin(); check_point != newextendfacepoints.end(); check_point++){
           if (node->Number()==int(check_point->X())) {
              node->Number() = int(check_point->Y());
           }
       }
     }
  }
  shapestepedgelines.Write("shapestepedgelines_afteradj.top", false);
  for (sel_match_line = snappedstepedgelines.begin(); sel_match_line !=snappedstepedgelines.end(); sel_match_line++){
     for (node=sel_match_line->begin(); node!=sel_match_line->end();node++){
       for (check_point = extendfacepoints.begin(); check_point != extendfacepoints.end(); check_point++){
           if (node->Number()==int(check_point->X())) {
              node->Number() = int(check_point->Y());
           }
       }
       for (check_point = newextendfacepoints.begin(); check_point != newextendfacepoints.end(); check_point++){
           if (node->Number()==int(check_point->X())) {
              node->Number() = int(check_point->Y());
           }
       }
     }
  }

  bool recostepedgeface = false;
  if (recostepedgeface){
  int success, buildingnr;
  LineTopologies facestepedgelines;
  for (map_line = snappedstepedgelines.begin(); map_line!=snappedstepedgelines.end(); map_line++){
      if (map_line->Attribute(LineLabelTag)==2){ //only the higher ones; first look for shapestepedges ...
       plane = laser_points.FitPlane(map_line->Attribute(SegmentLabel), map_line->Attribute(SegmentLabel), SegmentNumberTag);
       lowerplane = laser_points.FitPlane(map_line->Attribute(SecondSegmentLabel), map_line->Attribute(SecondSegmentLabel), SegmentNumberTag);
       seg_laser_points.ErasePoints(); // select seglaser points, for polygonnumber
       seg_laser_points.AddTaggedPoints(laser_points, map_line->Attribute(SegmentLabel), SegmentNumberTag);
       buildingnr = seg_laser_points.MostFrequentAttributeValue(PolygonNumberTag, count);
       node = map_line->begin();
       pos12d = (intsect_points.PointIterator(*node))->vect2D();
       node++;
       pos22d = (intsect_points.PointIterator(*node))->vect2D();
       line = Line2D(pos12d, pos22d);
       selmergedsegmentlines = shapestepedgelines.SelectAttributedLines(SegmentLabel, map_line->Attribute(SecondSegmentLabel));
       fprintf(statfile, "size of selmergedsegmentlines %d for segment %d second segment %d\n", selmergedsegmentlines.size(), map_line->Attribute(SegmentLabel), map_line->Attribute(SecondSegmentLabel));
       if (selmergedsegmentlines.size()>0){
       
         for (sel_match_line = selmergedsegmentlines.begin(); sel_match_line !=selmergedsegmentlines.end(); sel_match_line++){
             node = sel_match_line->begin();
             pos32d = (intsect_points.PointIterator(*node))->vect2D();
             node++;
             pos42d = (intsect_points.PointIterator(*node))->vect2D();
             if (line.PointOnLine(pos32d, 0.1) && line.PointOnLine(pos42d, 0.1)){
                sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
                dumpointnumber=0;
                newedgepoint = ObjectPoint(pos12d.GetX(), pos12d.GetY(), 0, dumpointnumber, 0,0,0,0,0,0);
                sel_map_points.push_back(newedgepoint);
                dumpointnumber++;
                newedgepoint = ObjectPoint(pos22d.GetX(), pos22d.GetY(), 0, dumpointnumber, 0,0,0,0,0,0);
                sel_map_points.push_back(newedgepoint);
                dumpointnumber++;
                newedgepoint = ObjectPoint(pos32d.GetX(), pos32d.GetY(), 0, dumpointnumber, 0,0,0,0,0,0);
                sel_map_points.push_back(newedgepoint);
                dumpointnumber++;
                newedgepoint = ObjectPoint(pos42d.GetX(), pos42d.GetY(), 0, dumpointnumber, 0,0,0,0,0,0);
                sel_map_points.push_back(newedgepoint);
                sel_map_points.SortOnCoordinates();
                // now take the middle two as location for stepedgeface...
                oldpoint = sel_map_points[1];
                newpointnumber++;
                newedgepoint = ObjectPoint(oldpoint.X(), oldpoint.Y(), plane.Z_At(oldpoint.X(), oldpoint.Y(),&success), newpointnumber, 0,0,0,0,0,0);
                top.clear(); top.Initialise();
                top.push_back(PointNumber(newpointnumber));
                intsect_points.push_back(newedgepoint);
                oldpoint = sel_map_points[2];
                newpointnumber++;
                newedgepoint = ObjectPoint(oldpoint.X(), oldpoint.Y(), plane.Z_At(oldpoint.X(), oldpoint.Y(),&success), newpointnumber, 0,0,0,0,0,0);
                top.push_back(PointNumber(newpointnumber));
                intsect_points.push_back(newedgepoint);
                top.SetAttribute(SegmentLabel, map_line->Attribute(SegmentLabel));
                top.SetAttribute(SecondSegmentLabel, map_line->Attribute(SecondSegmentLabel));
                top.SetAttribute(LineLabelTag, 12);
                top.SetAttribute(BuildingNumberTag, buildingnr);
                topstepedge.push_back(top);
                newpointnumber++;
                newedgepoint = ObjectPoint(oldpoint.X(), oldpoint.Y(), lowerplane.Z_At(oldpoint.X(), oldpoint.Y(),&success), newpointnumber, 0,0,0,0,0,0);
                top.push_back(PointNumber(newpointnumber));
                intsect_points.push_back(newedgepoint);
                oldpoint = sel_map_points[1];
                newpointnumber++;
                newedgepoint = ObjectPoint(oldpoint.X(), oldpoint.Y(), lowerplane.Z_At(oldpoint.X(), oldpoint.Y(),&success), newpointnumber, 0,0,0,0,0,0);
                top.push_back(PointNumber(newpointnumber));
                intsect_points.push_back(newedgepoint);
                if (!top.IsClosed()) top.push_back(*(top.begin()));
                
                facestepedgelines.push_back(top);
             }
         }
       }
       else {
            selmergedsegmentlines2 = snappedstepedgelines.SelectAttributedLines(SegmentLabel, map_line->Attribute(SecondSegmentLabel));
            selmergedsegmentlines = selmergedsegmentlines2.SelectAttributedLines(LineLabelTag, 3);
            if (selmergedsegmentlines.size()>0){
         for (sel_match_line = selmergedsegmentlines.begin(); sel_match_line !=selmergedsegmentlines.end(); sel_match_line++){
             node = sel_match_line->begin();
             pos32d = (intsect_points.PointIterator(*node))->vect2D();
             node++;
             pos42d = (intsect_points.PointIterator(*node))->vect2D();
             if (line.PointOnLine(pos32d, 0.1) && line.PointOnLine(pos42d, 0.1)){
                sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
                dumpointnumber=0;
                newedgepoint = ObjectPoint(pos12d.GetX(), pos12d.GetY(), 0, dumpointnumber, 0,0,0,0,0,0);
                sel_map_points.push_back(newedgepoint);
                dumpointnumber++;
                newedgepoint = ObjectPoint(pos22d.GetX(), pos22d.GetY(), 0, dumpointnumber, 0,0,0,0,0,0);
                sel_map_points.push_back(newedgepoint);
                dumpointnumber++;
                newedgepoint = ObjectPoint(pos32d.GetX(), pos32d.GetY(), 0, dumpointnumber, 0,0,0,0,0,0);
                sel_map_points.push_back(newedgepoint);
                dumpointnumber++;
                newedgepoint = ObjectPoint(pos42d.GetX(), pos42d.GetY(), 0, dumpointnumber, 0,0,0,0,0,0);
                sel_map_points.push_back(newedgepoint);
                sel_map_points.SortOnCoordinates();
                // now take the middle two as location for stepedgeface...
                oldpoint = sel_map_points[1];
                newpointnumber++;
                newedgepoint = ObjectPoint(oldpoint.X(), oldpoint.Y(), plane.Z_At(oldpoint.X(), oldpoint.Y(),&success), newpointnumber, 0,0,0,0,0,0);
                top.clear(); top.Initialise();
                top.push_back(PointNumber(newpointnumber));
                intsect_points.push_back(newedgepoint);
                oldpoint = sel_map_points[2];
                newpointnumber++;
                newedgepoint = ObjectPoint(oldpoint.X(), oldpoint.Y(), plane.Z_At(oldpoint.X(), oldpoint.Y(),&success), newpointnumber, 0,0,0,0,0,0);
                top.push_back(PointNumber(newpointnumber));
                intsect_points.push_back(newedgepoint);
                top.SetAttribute(SegmentLabel, map_line->Attribute(SegmentLabel));
                top.SetAttribute(SecondSegmentLabel, map_line->Attribute(SecondSegmentLabel));
                top.SetAttribute(LineLabelTag, 13);
                top.SetAttribute(BuildingNumberTag, buildingnr);
                topstepedge.push_back(top);
                newpointnumber++;
                newedgepoint = ObjectPoint(oldpoint.X(), oldpoint.Y(), lowerplane.Z_At(oldpoint.X(), oldpoint.Y(),&success), newpointnumber, 0,0,0,0,0,0);
                top.push_back(PointNumber(newpointnumber));
                intsect_points.push_back(newedgepoint);
                oldpoint = sel_map_points[1];
                newpointnumber++;
                newedgepoint = ObjectPoint(oldpoint.X(), oldpoint.Y(), lowerplane.Z_At(oldpoint.X(), oldpoint.Y(),&success), newpointnumber, 0,0,0,0,0,0);
                top.push_back(PointNumber(newpointnumber));
                intsect_points.push_back(newedgepoint);
                if (!top.IsClosed()) top.push_back(*(top.begin()));
                
                facestepedgelines.push_back(top);
             }
         }
       }
            }
      }
  }
  facestepedgelines.Write("shapestepedgelines_faces.top", false);
}// end reco shape stepedgelines
  
bool ibs;
ibs = true;
if (ibs){
LineTopology buildingtop;
LineSegment2D            ls2d;
LineTopologies::iterator dum_line;
int                      intmapheight;
mapheight = map_points[0].Z();
printf("start intersecting map outline by step edges...\n");

for (map_line = topstepedge.begin(), index1=0; map_line!=topstepedge.end(); map_line++, index1++){
     printf("%7d  %5.1f \r", index1, 100.0 * index1 / topstepedge.size());
     fprintf(statfile, "step edge %d ", map_line->Number());
       node = map_line->begin();
       pos12d = (intsect_points.PointIterator(*node))->vect2D();
       node++;
       pos22d = (intsect_points.PointIterator(*node))->vect2D();
       line = Line2D(pos12d, pos22d);
       ls2d = LineSegment2D(pos12d, pos22d);
       buildingnr = map_line->Attribute(BuildingNumberTag);
       if (lastbuildingnr!=buildingnr && index1!=0){
          for (partition2 = notyetallsplittedlines.begin(), index2=1; partition2!=notyetallsplittedlines.end(); partition2++, index2++){
              partition2->SetAttribute(NodeNumberTag, index2);
          }
          for (dum_line = map_lines.begin(); dum_line!=map_lines.end(); dum_line++){
            if (dum_line->Number()== buildingnr) intmapheight = dum_line->Attribute(PredictedHeight);
          }
        notyetallsplittedlines.SetAttribute(PredictedHeight, intmapheight);
        allsplittedlines.insert(allsplittedlines.end(), notyetallsplittedlines.begin(), notyetallsplittedlines.end());
        notyetallsplittedlines.erase(notyetallsplittedlines.begin(), notyetallsplittedlines.end());
        }         
       selsplittedlines.erase(selsplittedlines.begin(),selsplittedlines.end());
       selsplittedlines = notyetallsplittedlines.SelectAttributedLines(BuildingNumberTag, buildingnr);
       if (selsplittedlines.size() == 0){
        index_line = map_lines.FindLine(LineNumber(buildingnr));
        buildingtop = map_lines[index_line];
        splittedlines.erase(splittedlines.begin(), splittedlines.end());
//        buildingtop.SplitPolygonByLine(map_points, line, 0.25, 1, splittedlines);
        if (buildingtop.SplitPolygonByLineSegment_nearest(map_points, ls2d, 0.5, 1, splittedlines)){
          splittedlines.SetAttribute(BuildingNumberTag, buildingnr);
          for (partition = splittedlines.begin(); partition!=splittedlines.end(); partition++){
              fprintf(statfile, "area of splitted map polygon %4.2f \n", partition->CalculateArea(map_points)); 
              if (partition->CalculateArea(map_points)==0){
                  splittedlines.erase(partition);
                  partition--;
              }
          }
          //notyetallsplittedlines.insert(notyetallsplittedlines.end(), splittedlines.begin(), splittedlines.end());
          notyetallsplittedlines = splittedlines;
          }
         if (splittedlines.size()==0){// else{
               buildingtop.SetAttribute(BuildingNumberTag, buildingnr);
               notyetallsplittedlines.push_back(buildingtop);
               }
        }
        else{
          if (selsplittedlines.size()<100){
          notyetallsplittedlines.erase(notyetallsplittedlines.begin(), notyetallsplittedlines.end());
          for (partition = selsplittedlines.begin(), index2=0; partition!=selsplittedlines.end(); partition++, index2++){
            printf("%7d  %5.1f  %7d  %5.1f\r", index1, 100.0 * index1 / topstepedge.size(), index2, 100.0*index2 / selsplittedlines.size());
            buildingtop = *partition;
            splittedlines.erase(splittedlines.begin(), splittedlines.end());
            if (buildingtop.SplitPolygonByLineSegment_nearest(map_points, ls2d, 0.5, 1, splittedlines)){
              splittedlines.SetAttribute(BuildingNumberTag, buildingnr);
              for (partition2 = splittedlines.begin(); partition2!=splittedlines.end(); partition2++){
                fprintf(statfile, "area of splitted map polygon %4.2f \n", partition2->CalculateArea(map_points)); 
                if (partition2->CalculateArea(map_points)==0){
                                                              splittedlines.erase(partition2);
                                                              partition2--;
                                                              }
              }

              notyetallsplittedlines.insert(notyetallsplittedlines.end(), splittedlines.begin(), splittedlines.end());
              }
            if (splittedlines.size()==0){//   else {
                   notyetallsplittedlines.push_back(*partition);
                   }
          //  notyetallsplittedlines = splittedlines;
           }
           }
        }
       lastbuildingnr = buildingnr;
}
// add the last one too...
for (partition2 = notyetallsplittedlines.begin(), index2=1; partition2!=notyetallsplittedlines.end(); partition2++, index2++){
     partition2->SetAttribute(NodeNumberTag, index2);
}
for (dum_line = map_lines.begin(); dum_line!=map_lines.end(); dum_line++){
 if (dum_line->Number()== buildingnr) intmapheight = dum_line->Attribute(PredictedHeight);
}
notyetallsplittedlines.SetAttribute(PredictedHeight, intmapheight);

allsplittedlines.insert(allsplittedlines.end(), notyetallsplittedlines.begin(), notyetallsplittedlines.end());
//allsplittedlines.insert(allsplittedlines.end(), notyetallsplittedlines.begin(), notyetallsplittedlines.end());
for (map_line = map_lines.begin(); map_line!=map_lines.end(); map_line++){
    selsplittedlines.erase(selsplittedlines.begin(),selsplittedlines.end());
    selsplittedlines = allsplittedlines.SelectAttributedLines(BuildingNumberTag, map_line->Number());
    if (selsplittedlines.size()==0) { 
      buildingtop = *map_line;
      buildingtop.SetAttribute(BuildingNumberTag, map_line->Number());
      buildingtop.SetAttribute(NodeNumberTag, 0);
      buildingtop.SetAttribute(PredictedHeight, map_line->Attribute(PredictedHeight));
      allsplittedlines.push_back(buildingtop);
      }
}


allsplittedlines.Write("allsplittedlines.top", false);
allsplittedpoints.erase(allsplittedpoints.begin(), allsplittedpoints.end());
for (map_point = map_points.begin(); map_point!=map_points.end();map_point++){
 //   map_point->Z() = mapheight;
}
for (map_line = allsplittedlines.begin(); map_line!=allsplittedlines.end(); map_line++){
  for (map_line2 = map_lines.begin(); map_line2!=map_lines.end(); map_line2++){
      if (map_line->Attribute(BuildingNumberTag)==map_line2->Number()){
        map_line->Attribute(PredictedHeight) = map_line2->Attribute(PredictedHeight);
      }
  }

    intmapheight = map_line->Attribute(PredictedHeight);
    for (node= map_line->begin(); node!=map_line->end();node++){
        new_point = *(map_points.PointIterator(*node));
        new_point.Z() = 0.01*(intmapheight-1000);
     //   fprintf(statfile,"new point height = 
//         allsplittedpoints.push_back(*(map_points.PointIterator(*node)));
         allsplittedpoints.push_back(new_point);
    }
}  
allsplittedpoints.RemoveDoublePoints(allsplittedlines, 0.1);
allsplittedlines.Write("allsplittedlines.top", false);       
allsplittedpoints.Write("allsplittedlines.objpts");

temp_lines = allsplittedlines;
temp_points = allsplittedpoints;
temp_lines.ReNumber(temp_points, 0, 0);
number_offseti = temp_points.HighestPointNumber().Number()+1;
temp_points.DuplicateWithFixedOffset(2.80, number_offseti);
wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
wall_map_lines.AddWalls(temp_lines, number_offseti);
wall_map_lines.SetAttribute(LineLabelTag, 6);
wall_map_lines.Write("firstfloor280.top", false);
temp_points.Write("firstfloor280.objpts");
} 
//return;

combinedlines.Write("combinedlines.top", false);
intsect_points.Write("combinedlines.objpts");
combinedlines.insert(combinedlines.end(), dormerlines.begin(), dormerlines.end());
combinedlines.insert(combinedlines.end(), flatroofs.begin(), flatroofs.end());
combinedlines.Write("combinedlines.top", false);
intsect_points.Write("combinedlines.objpts");
// see if buildings are missing; just fit flat roof
newpointnumber = intsect_points.HighestPointNumber().Number()+1;                           

 for (map_line = map_lines.begin(); map_line!=map_lines.end(); map_line++){
          shapetops = polygonlines.SelectAttributedLines(BuildingNumberTag, map_line->Number());
          printf("building %d, shapetops size %d\n", map_line->Number(), shapetops.size());
          sel_laser_points.ErasePoints();
          sel_laser_points = laser_points.SelectTagValue(PolygonNumberTag, map_line->Number());
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
                  point = ObjectPoint(this_point->X(),this_point->Y(),seg_laser_points.Mean()[2], newpointnumber, 0,0,0,0,0,0);
                  top.push_back(PointNumber(newpointnumber));
                  intsect_points.push_back(point);
                  sel_map_points.push_back(point);
                  newpointnumber++;
                  }
                top.SetAttribute(SegmentLabel, segnr);
                top.SetAttribute(LineLabelTag, 12);
                top.SetAttribute(LineLabelTag, segnr);
                top.SetAttribute(BuildingNumberTag, map_line->Number());
                polygonlines.push_back(top);
//                flatroofpoints.AddPoints(seg_laser_points);
       }
 }
 

//return; //19-11-2010 just to get combinedlines.top

polygonlines.insert(polygonlines.end(), dormerlines.begin(), dormerlines.end());
polygonlines.insert(polygonlines.end(), flatroofs.begin(), flatroofs.end());
polygonlines.Write("polygonlines.top", false);
intsect_points.Write("polygonlines.objpts");

dxffile = fopen ("isprs_roofs.dxf","w");
intsect_points.WriteDXF(dxffile, polygonlines, true);
fclose(dxffile);

int countlargeres, countlargeresnotused, countlargeressegment;
countlargeres = 0;
countlargeresnotused = 0;
countlargeressegment = 0;
LaserPoints          sel_laser_points3, largemaplaserpoints, sel_laser_points4;
countlargeres = 0;

maplaserpoints.ErasePoints();
printf("\nstart calculating residuals on laser points (this can take a while)\n");
   for (map_line = map_lines.begin(), index=0; map_line!=map_lines.end(); map_line++, index++){
         printf("%6d (%5.1f\%)\r", index, 100.0 * index / map_lines.size());
         sel_mapresults = polygonlines.SelectAttributedLines(BuildingNumberTag, map_line->Number());
         sel_laser_points.ErasePoints();
         sel_laser_points.AddTaggedPoints(laser_points, map_line->Number(), PolygonNumberTag);
         for (laser_point=sel_laser_points.begin(); laser_point!=sel_laser_points.end(); laser_point++) {
         keepresidual = 10;
         found = false;
             for (face=sel_mapresults.begin(); face!=sel_mapresults.end(); face++) {
                 if (face->size()>2){
                if (laser_point->InsidePolygon(intsect_points, face->LineTopologyReference())){
                   plane.Initialise(); 
                   for (node=face->begin(); node!=face->end()-1; node++) {
                       this_point = intsect_points.PointIterator(node->NumberRef());
                       plane.AddPoint(this_point->Position3DRef(), false);
                   }
                   plane.Recalculate();
                   found = true;
             //      residual = laser_point->Z() - plane.Z_At(laser_point->X(), laser_point->Y(), &success);
                   pos = laser_point->vect();
                   projpos = plane.Project(pos);
                   residual = projpos.Distance(pos);//laser_point->Z() - plane.Z_At(laser_point->X(), laser_point->Y(), &success);

                   if (fabs(residual)<fabs(keepresidual)) keepresidual = residual;
                }
                }
             }
  //       if (found){
          maplaserpoint = LaserPoint(laser_point->X(), laser_point->Y(), laser_point->Z());
          maplaserpoint.Residual() = keepresidual;
          maplaserpoint.SetAttribute(SegmentNumberTag, laser_point->Attribute(SegmentNumberTag));
          maplaserpoint.SetAttribute(PolygonNumberTag, map_line->Number());
          sel_mapresults2 = sel_mapresults.SelectAttributedLines(SegmentLabel, laser_point->Attribute(SegmentNumberTag));
          if (sel_mapresults2.size() == 0) found = false;//if this segment is not used do not take this into account
          if (found) maplaserpoint.SetAttribute(LabelTag, 1);
          if (!found) maplaserpoint.SetAttribute(LabelTag, 0);
          maplaserpoints.push_back(maplaserpoint);
          if (fabs(keepresidual)>0.2){
               countlargeres++;
               largemaplaserpoints.push_back(maplaserpoint);
               sel_laser_points2.ErasePoints();
               sel_laser_points2.AddTaggedPoints(notused, laser_point->Attribute(SegmentNumberTag), SegmentNumberTag); // check if this is a notused segment
               if (sel_laser_points2.size()>1)  {
          //       notusedmaplaserpoints.push_back(maplaserpoint);
                 countlargeresnotused++;
               }
          }
                                      
 //        }
         }
  }
maplaserpoints.Write("datadrivenqualitylaserpoints_distance2face.laser", false);

return;
bool slo;
slo = true;
if (slo){
   segment_numbers = laser_points.AttributeValues(SegmentNumberTag);
   sort(segment_numbers.begin(), segment_numbers.end());
   segment_numbers_model = combinedlines.AttributedValues(SegmentLabel);
   sort(segment_numbers_model.begin(), segment_numbers_model.end());
   
   for (segment_number = segment_numbers.begin(); segment_number!=segment_numbers.end(); segment_number++){
       found = false;
       for (segment_number_m = segment_numbers_model.begin(); segment_number_m!=segment_numbers_model.end(); segment_number_m++){
           if (*segment_number == *segment_number_m) found = true;
       }
       if (!found){
          for (segment_number_m = low_segments.begin(); segment_number_m!=low_segments.end(); segment_number_m++){
            if (*segment_number == *segment_number_m) found = true;
          }
       }
       if (!found){
        laser_points_leftover.AddTaggedPoints(laser_points, *segment_number, SegmentNumberTag);
       }
   }
laser_points_leftover.Write("laserpointsleftover.laser", false);
}


buildingnumbers = laser_points_leftover.AttributeValues(PolygonNumberTag);
b1 = buildingnumbers.size();
buildingnumbers = laser_points.AttributeValues(PolygonNumberTag);
b2 = buildingnumbers.size();
 
fprintf(statfile,"\n***\nNumber of buildings affected of leftover laserpoints %d, out of %d\n", b1, b2);

bool addintersectionpoints2partitions;
addintersectionpoints2partitions = true;//true;//false; //soe 120609//soe 240809
if (addintersectionpoints2partitions){

//mapheight = allsplittedpoints[0].Z();
newpointnumber = allsplittedpoints.HighestPointNumber().Number()+1;
keephighnumber = newpointnumber;
for (map_line = allsplittedlines.begin(), index1 =0; map_line!=allsplittedlines.end(); map_line++, index1++){
    
  //  mapheight = allsplittedpoints[0].Z();
    mapheight = 0.01*(map_line->Attribute(PredictedHeight)-1000);
//     printf("%7d  %5.1f \n", index1, 100.0 * index1 / allsplittedlines.size());
     sel_combinedlines.erase(sel_combinedlines.begin(), sel_combinedlines.end());
     sel_combinedlines2.erase(sel_combinedlines2.begin(), sel_combinedlines2.end());
     sel_combinedlines = polygonlines.SelectAttributedLines(BuildingNumberTag, map_line->Attribute(BuildingNumberTag));
     printf("size of selcombinedlines: %d, building number %d, mapheight %4.2f\n", sel_combinedlines.size(), map_line->Attribute(BuildingNumberTag), mapheight);
     if (sel_combinedlines.size()>0){ // it this building contains polygon lines
     for (combline = sel_combinedlines.begin(); combline!=sel_combinedlines.end(); combline++){
     if (combline->size()>2){
//          printf("size of combline: %d\n", combline->size());
         for (node= combline->begin(); node!=combline->end()-1;node++){
              top.clear(); top.Initialise();
              top.push_back(PointNumber(node->Number()));
              next_node=combline->NextNode(node);
              top.push_back(PointNumber(next_node->Number()));
              top.SetAttribute(SegmentLabel, combline->Attribute(SegmentLabel));//*segment_number);
              top.SetAttribute(BuildingNumberTag, map_line->Attribute(BuildingNumberTag));
              sel_combinedlines2.push_back(top); 
          }
          }
     }     
     } //deleted soe 240809, replaced lower
     sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());     
     for (node2 = map_line->begin(); node2!=map_line->end()-1;node2++){
         map_point = allsplittedpoints.PointIterator(*node2);
     //    mapheight = map_point->Z();
         map_point->Z() = mapheight;
         sel_map_points.push_back(*map_point);
     }
     //start new insertion 141009
     for (node= map_line->begin(); node!=map_line->end()-1;node++){
          nbmpoint1 = *(sel_map_points.PointIterator(*node));
          nbmppos12d = Position2D(nbmpoint1.X(), nbmpoint1.Y());
          next_node=map_line->NextNode(node);
          nbmpoint2 = *(sel_map_points.PointIterator(*next_node));
          nbmppos22d = Position2D(nbmpoint2.X(), nbmpoint2.Y());
          nbmline = Line2D(nbmppos12d, nbmppos22d);
          for (combline = sel_combinedlines2.begin(); combline!=sel_combinedlines2.end(); combline++){
               node2 = combline->begin();
               pos1 = intsect_points.PointIterator(*node2)->Position3DRef(); node2++;
               pos2 = intsect_points.PointIterator(*node2)->Position3DRef();
               line3d = Line3D(pos1, pos2);
               if (line3d.FindIntersection(nbmline, projposonmap1)){
                   maplaserpoint.X() = projposonmap1.GetX();
                   maplaserpoint.Y() = projposonmap1.GetY();
                   projposonmap2d = Position2D(maplaserpoint.X(), maplaserpoint.Y());
                   if (nbmppos12d.Distance(projposonmap2d)<=nbmppos12d.Distance(nbmppos22d) && nbmppos22d.Distance(projposonmap2d)<=nbmppos12d.Distance(nbmppos22d)){ // if the position is inside map segment
                     if ((pos1.Distance(projposonmap1)<=1.0*pos1.Distance(pos2) && pos2.Distance(projposonmap1)<=1.0*pos1.Distance(pos2)) || 
                         (pos1.Distance(projposonmap1)<=1.5 || pos2.Distance(projposonmap1)<=1.5)){ // if the position is inside face edge or next to end point
                            maplaserpoint.Z() = projposonmap1.GetZ();
                            maplaserpoint.SetAttribute(PolygonNumberTag, map_line->Attribute(BuildingNumberTag));
                            partlaserpoints.push_back(maplaserpoint);
                            maplaserpoint.Z() = mapheight;
                            maplaserpoint.SetAttribute(PolygonNumberTag, map_line->Attribute(BuildingNumberTag));
                            partlaserpoints.push_back(maplaserpoint);
                            newpointnumber++;
                            newedgepoint = ObjectPoint(maplaserpoint.X(), maplaserpoint.Y(), mapheight, newpointnumber, 0,0,0,0,0,0); 
                            sel_map_points.push_back(newedgepoint);
                     }
                   }
               }
          }
     }                  
     
     
     //end new insertion
     
//     if (map_line->CalculateArea(sel_map_points)>1){
/*     for (combline = sel_combinedlines2.begin(); combline!=sel_combinedlines2.end(); combline++){
         node = combline->begin();
         pos1 = intsect_points.PointIterator(*node)->Position3DRef(); node++;
         pos2 = intsect_points.PointIterator(*node)->Position3DRef();
         line3d = Line3D(pos1, pos2);
//         printf("%4.2f ", pos1.Distance(pos2));
         if (pos1.Distance(pos2)>0.1){//?
         for (node= map_line->begin(); node!=map_line->end()-1;node++){
              nbmpoint1 = *(sel_map_points.PointIterator(*node));
              nbmppos12d = Position2D(nbmpoint1.X(), nbmpoint1.Y());
              next_node=map_line->NextNode(node);
              nbmpoint2 = *(sel_map_points.PointIterator(*next_node));
              nbmppos22d = Position2D(nbmpoint2.X(), nbmpoint2.Y());
              nbmline = Line2D(nbmppos12d, nbmppos22d);
               if (line3d.FindIntersection(nbmline, projposonmap1)){
               if (pos1.Distance(projposonmap1)<=1.01*pos1.Distance(pos2) && pos2.Distance(projposonmap1)<=1.01*pos1.Distance(pos2)){ // if the position is inside face edge
       //        if (0.01*pos1.Distance(pos2)<=0.1){// && pos2.Distance(projposonmap1)<=1.01*pos1.Distance(pos2)){ // if the position is inside face edge
                  maplaserpoint.X() = projposonmap1.GetX();
                  maplaserpoint.Y() = projposonmap1.GetY();
                  projposonmap2d = Position2D(maplaserpoint.X(), maplaserpoint.Y());
                  if (nbmppos12d.Distance(projposonmap2d)<=nbmppos12d.Distance(nbmppos22d) && nbmppos22d.Distance(projposonmap2d)<=nbmppos12d.Distance(nbmppos22d)){ // if the position is inside map segment
                    maplaserpoint.Z() = projposonmap1.GetZ();
                    maplaserpoint.SetAttribute(PolygonNumberTag, map_line->Attribute(BuildingNumberTag));
                    partlaserpoints.push_back(maplaserpoint);
                    newpointnumber++;
                    newedgepoint = ObjectPoint(maplaserpoint.X(), maplaserpoint.Y(), mapheight, newpointnumber, 0,0,0,0,0,0); 
                    sel_map_points.push_back(newedgepoint);
             //       allsplittedpoints.push_back(newedgepoint);
                  }
      //            }
               }
             }
         }
         }
     }
  */
//     top  = *map_line;
//     top.InsertNodes(sel_map_points, 0.01);
//     insertsplittedlines.push_back(top);
       one_map_line.erase(one_map_line.begin(), one_map_line.end());
       one_map_line.push_back(*map_line);
//       sel_map_points.RemoveDoublePoints(one_map_line, 0.001);
       sel_map_points.RemoveDoublePoints(one_map_line, 0.01);
       top = *(one_map_line.begin());
//       printf("before %d ", top.size());
//       top.InsertNodes(sel_map_points, 0.001);
//       printf("and after %d \n", top.size());
       one_map_line.erase(one_map_line.begin(), one_map_line.end());
       one_map_line.push_back(top);
     //  one_map_line.ReNumber(sel_map_points, newsplittedpoints.size()+1, insertsplittedlines.size()+1);
       top = *(one_map_line.begin());
       outside = new LineTopology(top);
       outside->InsertNodes(sel_map_points);
//       printf("debug1 ");
       outside->MakeCounterClockWise(sel_map_points);
//       printf("debug2 ");

       newsplittedpoints.insert(newsplittedpoints.end(), sel_map_points.begin(), sel_map_points.end());
       allsplittedpoints.insert(allsplittedpoints.end(), sel_map_points.begin(), sel_map_points.end());
       top.SetAttribute(PredictedHeight, map_line->Attribute(PredictedHeight));
//       printf("debug3 ");
       selpartlaserpoints.ErasePoints();
       selpartlaserpoints.AddTaggedPoints(partlaserpoints, map_line->Attribute(BuildingNumberTag), PolygonNumberTag);
       sel_inside_points.erase(sel_inside_points.begin(), sel_inside_points.end());
       insertsplittedlines.push_back(*outside);
       insideline.clear(); insideline.Initialise();
       for (node2 = top.begin(); node2!=top.end()-1;node2++){
//         map_point = allsplittedpoints.PointIterator(*node2);
         new_point = *(newsplittedpoints.PointIterator(*node2));
         maplaserpoint.X()= new_point.X();
         maplaserpoint.Y()= new_point.Y();
         maplaserpoint.Z()= new_point.Z();
           keep = false;
           for (combline = sel_combinedlines.begin(); combline!=sel_combinedlines.end(); combline++){
               if(combline->size()>2){
               if (!keep && maplaserpoint.InsidePolygon(intsect_points, combline->LineTopologyReference())){
                  plane = laser_points.FitPlane(combline->Attribute(SegmentLabel), combline->Attribute(SegmentLabel), SegmentNumberTag);
                   keep = true;
                   new_point.Z() = plane.Z_At(maplaserpoint.X(), maplaserpoint.Y(), &success);
               }
               }
           }
           for (laser_point = selpartlaserpoints.begin(); laser_point!=selpartlaserpoints.end(); laser_point++){
               if (!keep && (laser_point->vect2D()-maplaserpoint.vect2D()).Length()<0.01) {
                  keep = true;
                  new_point.Z() = laser_point->Z();
                  }                  
           }
         if (keep) {
                   insideline.push_back(PointNumber(node2->Number()));
                   insidepoints.push_back(new_point);
                   sel_inside_points.push_back(new_point);
                   }
       }
       if (insideline.size()>1){
       insideline.SetAttribute(BuildingNumberTag, map_line->Attribute(BuildingNumberTag));
       if (!insideline.IsClosed()) insideline.push_back(*(insideline.begin()));
       insidelines.push_back(insideline);
       one_map_line.erase(one_map_line.begin(), one_map_line.end());
       wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
       one_map_line.push_back(insideline);
       sel_inside_points.RemoveDoublePoints(one_map_line, 0.01);
       if (sel_inside_points.size()>2){
       one_map_line.ReNumber(sel_inside_points, 0, 0);
       number_offseti = sel_inside_points.HighestPointNumber().Number()+1;
       sel_inside_points.DuplicateWithFixedZ(mapheight, number_offseti);
       wall_map_lines.AddWalls(one_map_line, number_offseti);

       wall_map_lines.ReNumber(sel_inside_points, wallinsidepoints.size()+1, wallinsidelines.size()+1);
       wallinsidelines.insert(wallinsidelines.end(), wall_map_lines.begin(), wall_map_lines.end());
       wallinsidepoints.insert(wallinsidepoints.end(), sel_inside_points.begin(), sel_inside_points.end());
       }
       }
 //     } //ins 240809 instead of upp    
}
partlaserpoints.Write("extranodes.laser", false);
//partlaserpoints.ReduceData(0.1);
partlaserpoints.Write("extranodes_2.laser", false);

/*for (map_line = allsplittedlines.begin(), index1 =0; map_line!=allsplittedlines.end(); map_line++, index1++){
 mapheight = 0.01*(map_line->Attribute(PredictedHeight)-1000);
 selpartlaserpoints.ErasePoints();
 selpartlaserpoints.AddTaggedPoints(partlaserpoints, map_line->Attribute(BuildingNumberTag), PolygonNumberTag);
 printf("%7d  %5.1f  %5.1f %d\r", index1, 100.0 * index1 / allsplittedlines.size(), map_line->CalculateArea(allsplittedpoints), selpartlaserpoints.size());
 top=*map_line;
 if (map_line->CalculateArea(allsplittedpoints)>4){
  for (laser_point = selpartlaserpoints.begin(); laser_point!=selpartlaserpoints.end(); laser_point++){
    newpointnumber++;
    newedgepoint = ObjectPoint(laser_point->X(), laser_point->Y(), mapheight, newpointnumber, laser_point->Z(),0,0,0,0,0); 
    allsplittedpoints.push_back(newedgepoint);
  }
  top.InsertNodes(allsplittedpoints, 0.0001);
  }
  insertsplittedlines.push_back(top);
}

partlaserpoints.Write("partlaserpoints.laser", false);
*/
//allsplittedlines.InsertNodes(allsplittedpoints);
//allsplittedpoints.RemoveDoublePoints(allsplittedlines, 0.01);
//allsplittedpoints.RemoveDoublePoints(insertsplittedlines, 0.01);
newsplittedpoints.RemoveDoublePoints(insertsplittedlines, 0.1);

insidelines.Write("allinsidelines.top", false);
insidepoints.Write("allinsidelines.objpts");


wallinsidelines.SetAttribute(LineLabelTag, 5);
wallinsidelines.Write("wallinsidelines.top", false);
wallinsidepoints.Write("wallinsidelines.objpts");

wallinsidelines.SetAttribute(LineLabelTag, 6);
wallinsidelines.Write("wallinsidelines_to_overlay.top", false);
wallinsidepoints.Write("wallinsidelines_to_overlay.objpts");
wallinsidelines.SetAttribute(LineLabelTag, 5);

dxffile = fopen ("dxffile_wall_inside.dxf","w");
wallinsidepoints.WriteDXF(dxffile, wallinsidelines, true);
fclose(dxffile);
printf("finished writing wall dxf...\n");
//was insersplittedlines
insertsplittedlines.Write("allinsertedsplittedlines.top", false);
//allsplittedpoints.Write("allinsertedsplittedlines.objpts");
newsplittedpoints.Write("allinsertedsplittedlines.objpts");
//map_points = newsplittedpoints;
allsplittedpoints = newsplittedpoints;
allsplittedlines = insertsplittedlines;
}

//return;


bool height2partitions;
height2partitions = true;//true;//false; //soe 120609
if (height2partitions){
printf("\nStart calculating height to partitioned map outline...");

LaserPoint             centroidofsegment, buildingedgepoint;
LaserPoints            partlaserdata, notusedlaserpoints, keepinsidepoints, edgebuildingpoints;
PointNumberList        pnlseg1;
ObjectPoint            objpseg1, point; 
int                        label, newpointnumber, success, nbh_radius, 
                           dominantsegment, highestsegment, keepnewpointnumber;
LaserPoints                temp_height_points;
vector <int>               selsegmentnumbers;
double                     first_floor_height, floor_height, heighthighsegment;
bool                       used;

newpointnumber = allsplittedpoints.HighestPointNumber().Number()+1;
nbh_radius = 1;
for (map_line = allsplittedlines.begin(); map_line!=allsplittedlines.end(); map_line++){
    printf("\nMapline: %d", map_line->Number());
     sel_laser_points.ErasePoints();
     floor_height = 0.01*(map_line->Attribute(PredictedHeight)-1000);
     sel_laser_points.AddTaggedPoints(laser_points, map_line->Attribute(BuildingNumberTag), PolygonNumberTag);
     segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
     segment_numbers = sel_laser_points.AttributeValues(SegmentNumberTag);
     sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
     partlaserdata.ErasePoints();
     
     for (node2 = map_line->begin(); node2!=map_line->end();node2++){
        sel_map_points.push_back(*(allsplittedpoints.PointIterator(*node2)));
        new_point = *(allsplittedpoints.PointIterator(*node2));
        buildingedgepoint = LaserPoint(new_point.X(), new_point.Y(), new_point.Z());
        edgebuildingpoints.push_back(buildingedgepoint);
     } 
     for (segment_number = segment_numbers.begin(); segment_number!=segment_numbers.end(); segment_number++){
         seg_laser_points.ErasePoints();
         keepinsidepoints.ErasePoints();
         seg_laser_points.AddTaggedPoints(sel_laser_points, *segment_number, SegmentNumberTag);
         pnlseg1 = seg_laser_points.SelectTagValueList(SegmentNumberTag, *segment_number);                  
         objpseg1 = seg_laser_points.Centroid(pnlseg1, *segment_number);
         centroidofsegment = LaserPoint(objpseg1.X(), objpseg1.Y(), objpseg1.Z());
// is fast but not always correct         if (centroidofsegment.InsidePolygon(sel_map_points, map_line->LineTopologyReference())){
         int perc = 0;
         for (laser_point = seg_laser_points.begin(); laser_point !=seg_laser_points.end(); laser_point++){
          if (laser_point->InsidePolygon(sel_map_points, map_line->LineTopologyReference())) {
             perc++;
             keepinsidepoints.push_back(*laser_point);
             }
         }
         seg_laser_points = keepinsidepoints;
         //if (perc*100.0 / seg_laser_points.size()>50){//more than half of segment inside        
           seg_laser_points.SetAttribute(PlaneNumberTag, map_line->Attribute(NodeNumberTag));
           partitionedlaserdata.AddPoints(seg_laser_points);
           notusedlaserpoints.ErasePoints();
           notusedlaserpoints.AddTaggedPoints(laser_points_leftover, *segment_number, SegmentNumberTag);
           if (notusedlaserpoints.size()==0 || seg_laser_points.size()>50) {
              partlaserdata.AddPoints(seg_laser_points); //only add if segment is used or if it is large
           //now add points on locations where map partition intersect with ridges
            }
      // }
     }
     if (partlaserdata.size()>0){
      top.clear(); top.Initialise();
      perc0 = partlaserdata.ReturnHeightOfPercentilePoint(0);
      perc5 = partlaserdata.ReturnHeightOfPercentilePoint(5);
      perc100 = partlaserdata.ReturnHeightOfPercentilePoint(100);
      if ((perc5-perc0)>(perc100-perc0)*0.1) perc0=perc5;
      fprintf(statfile, "%d and perc0 %4.2f", partlaserdata.size(), perc0);
      sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
      dominantlabel = partlaserdata.MostFrequentAttributeValue(LabelTag, count);
      usedominantlabel = false;
      if (dominantlabel<1550){
                              usedominantlabel = true;
                              dominantlabelpoints.ErasePoints();
                              dominantlabelpoints.AddTaggedPoints(partlaserdata, dominantlabel, LabelTag);
                              dominantlabelsegment = dominantlabelpoints.MostFrequentAttributeValue(SegmentNumberTag, count);
                              }
      for (node2 = map_line->begin(); node2!=map_line->end();node2++){
        // copy paste from reco upper floor soe 140409
        new_point = *(allsplittedpoints.PointIterator(*node2));
        temp_height_points.ErasePoints();  
        newpos = (allsplittedpoints.PointIterator(*node2))->Position3DRef();
        buildingedgepoint = LaserPoint(new_point.X(), new_point.Y(), new_point.Z());
        edgebuildingpoints.push_back(buildingedgepoint);
        found = false;
     //   floor_height = new_point.Z();
        highestsegment = -1;
        heighthighsegment = -10;
        nbh_radius = 1;
        do {
        for (laser_point=partlaserdata.begin(); laser_point!=partlaserdata.end(); laser_point++){
          dist = (laser_point->vect2D()-new_point.vect2D()).Length();   
            if (dist < nbh_radius && (laser_point->Attribute(LabelTag)!=1600)){//<1500 || laser_point->Attribute(LabelTag)>1600)) { //no dormer
                temp_height_points.push_back(*laser_point);
                if (laser_point->Z()>heighthighsegment){// && laser_point->Attribute(LabelTag)>1600) {
                   heighthighsegment = laser_point->Z();
                   highestsegment = laser_point->Attribute(SegmentNumberTag);
                   }                                         
                }
           }
           nbh_radius = nbh_radius+1;
           selsegmentnumbers = temp_height_points.AttributeValues(SegmentNumberTag);
           for (segment_number = selsegmentnumbers.begin(); segment_number !=selsegmentnumbers.end(); segment_number++){
                used = false;
                for (msegment_number = matchedsegment_numbers.begin(); msegment_number!=matchedsegment_numbers.end(); msegment_number++){
                    if (*msegment_number==*segment_number) used = true;
                }
                if (used) {  
                buildingedgepoint = LaserPoint(new_point.X(), new_point.Y(), new_point.Z());
                plane = laser_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
                buildingedgepoint.Z() = plane.Z_At(new_point.X(), new_point.Y(), &success);
                buildingedgepoint.SetAttribute(LabelTag, *segment_number);
                buildingedgepoint.SetAttribute(SegmentNumberTag, *segment_number);
          //      edgebuildingpoints.push_back(buildingedgepoint);
                maplaserpoint.X() = node2->Number();
                maplaserpoint.Y() = *segment_number;
                maplaserpoint.Z() = 0;
                maplaserpoint.SetAttribute(SegmentNumberTag, *segment_number);
                maplaserpoint.SetAttribute(LabelTag, node2->Number());
                selmaplaserpoints.push_back(maplaserpoint);
                found = true;
                }
            }
           dominantsegment = temp_height_points.MostFrequentAttributeValue(SegmentNumberTag, count);
//           } while (count<10 && nbh_radius<3);
           } while (!found && nbh_radius<3);
        if (temp_height_points.size()>0) {
          dominantsegment = temp_height_points.MostFrequentAttributeValue(SegmentNumberTag, count);
          fprintf(statfile, "%4.2f, domseg = %d", floor_height, dominantsegment);
          plane = laser_points.FitPlane(dominantsegment, dominantsegment, SegmentNumberTag);
          notusedlaserpoints.ErasePoints();
          notusedlaserpoints.AddTaggedPoints(laser_points_leftover, dominantsegment, SegmentNumberTag);
          if (notusedlaserpoints.size()!=0) {
            maplaserpoint.X() = new_point.X();
            maplaserpoint.Y() = new_point.Y();
            maplaserpoint.Z() = plane.Z_At(new_point.X(), new_point.Y(), &success);
            maplaserpoint.SetAttribute(SegmentNumberTag, dominantsegment);
            pointsofnotusedsegments.push_back(maplaserpoint);//segment_numbers_partitions.push_back(dominantsegment);
            }
          found = true;
         newpointnumber++;
        newedgepoint = ObjectPoint(new_point.X(), new_point.Y(), plane.Z_At(newpos.GetX(), newpos.GetY(),&success), newpointnumber, 0,0,0,0,0,0);
        if (newedgepoint.Z()<=floor_height+1) {
            newedgepoint.Z() = prevheight;
            }
            else{
                 prevheight = newedgepoint.Z();
            }
        top.push_back(PointNumber(newpointnumber)); 
        upper_points.push_back(newedgepoint);
        sel_map_points.push_back(newedgepoint);
        }
        else {
             newpointnumber++;

             newedgepoint = ObjectPoint(new_point.X(), new_point.Y(), perc0, newpointnumber, 0,0,0,0,0,0);

             top.push_back(PointNumber(newpointnumber));
             upper_points.push_back(newedgepoint);
             sel_map_points.push_back(newedgepoint);
             }
       } 
       if (sel_map_points.size()>2){
       top.SetAttribute(BuildingNumberTag, map_line->Attribute(BuildingNumberTag));
       top.SetAttribute(LineLabelTag, 12);
       one_map_line.erase(one_map_line.begin(), one_map_line.end());
       wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
       one_map_line.push_back(top);
       sel_map_points.RemoveDoublePoints(one_map_line, 0.1);
       one_map_line.ReNumber(sel_map_points, 0, 0);
       number_offset = sel_map_points.HighestPointNumber().Number()+1;
       sel_map_points.DuplicateWithFixedZ(floor_height, number_offset);
       wall_map_lines.AddWalls(one_map_line, number_offset);
       wall_map_lines.ReNumber(sel_map_points, all_flat_partition_points.size()+1, all_flat_partition_lines.size()+1);
       all_flat_partition_lines.insert(all_flat_partition_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
       all_flat_partition_points.insert(all_flat_partition_points.end(), sel_map_points.begin(), sel_map_points.end());
       }
       }
}
partitionedlaserdata.Write("partitionedlaserdata.laser", false);
edgebuildingpoints.Write("edgebuildingpoints.laser", false);
pointsofnotusedsegments.Write("pointsofnotusedsegments.laser", false);


all_flat_partition_lines.Write("partitionheights.top", false);
all_flat_partition_points.Write("partitionheights.objpts");
all_flat_partition_lines.SetAttribute(LineLabelTag, 6);
all_flat_partition_lines.Write("partitionheights_to_overlay.top", false);
all_flat_partition_points.Write("partitionheights_to_overlay.objpts");
all_flat_partition_lines.SetAttribute(LineLabelTag, 5);
}


bool flatreco = true;//true;//true;
if (flatreco){
printf("start reconstructing flat roofs...\n");

LineTopology  flatrooftop;
LaserPoints   seg_laser_points;
Plane         plane;
int           count, newpointnumber, success;
ObjectPoints::iterator       this_point;

    newpointnumber = allsplittedpoints.HighestPointNumber().Number()+1;                           

   sel_laser_points.ErasePoints();
   sel_laser_points.AddTaggedPoints(laser_points, 1500, LabelTag);
   segment_numbers.erase(segment_numbers.begin(), segment_numbers.end());
   segment_numbers = sel_laser_points.AttributeValues(SegmentNumberTag);
   for (segment_number = segment_numbers.begin(); segment_number!=segment_numbers.end(); segment_number++){
         seg_laser_points.ErasePoints();
         seg_laser_points.AddTaggedPoints(sel_laser_points, *segment_number, SegmentNumberTag);
         intfindline = seg_laser_points.MostFrequentAttributeValue(PolygonNumberTag, count);
         index_line = map_lines.FindLine(LineNumber(intfindline));
         flatrooftop = map_lines[index_line];
         sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
         plane = sel_laser_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
         top.clear(); top.Initialise();
         floor_height = 0.01*(flatrooftop.Attribute(PredictedHeight)-1000); //when predictedheight was generated (in matchdatasets) 10 m was added to the dtm height, in order to avoid problems up to 10m under sea level...
         printf("floor height flat rooftops %4.2f\n", floor_height);
         for (node2=flatrooftop.begin(); node2!=flatrooftop.end()-1; node2++) {
             this_point = map_points.PointIterator(*node2);
             newpointnumber++;
      //       floor_height = this_point->Z();
             if (plane.IsHorizontal(25*PI/180)){ // if plane not too steep (25 deg) fit a plane, take height of plane.
             newedgepoint = ObjectPoint(this_point->X(), this_point->Y(), plane.Z_At(this_point->X(), this_point->Y(), &success), newpointnumber, 0,0,0,0,0,0); 
             }
             else { //take mean height of points. plane too steep to be reliable ...?
             newedgepoint = ObjectPoint(this_point->X(), this_point->Y(), seg_laser_points.Mean()[2], newpointnumber, 0,0,0,0,0,0); 
                  }
             intsect_points.push_back(newedgepoint);
             top.push_back(PointNumber(newpointnumber));
             sel_map_points.push_back(newedgepoint);
             }
         if (!top.IsClosed()) top.push_back(*(top.begin()));
         top.SetAttribute(SegmentLabel, *segment_number);
         top.SetAttribute(LineLabelTag, 5);
         top.SetAttribute(LineNumberTag, intfindline);
         flatroofs.push_back(top);
         one_map_line.erase(one_map_line.begin(), one_map_line.end());
         wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
         one_map_line.push_back(top);
         sel_map_points.RemoveDoublePoints(one_map_line, 0.1);
         one_map_line.ReNumber(sel_map_points, 0, 0);
         number_offset = sel_map_points.HighestPointNumber().Number()+1;
         sel_map_points.DuplicateWithFixedZ(floor_height, number_offset);
         wall_map_lines.insert(wall_map_lines.end(), one_map_line.begin(), one_map_line.end());
         wall_map_lines.AddWalls(one_map_line, number_offset);
         wall_map_lines.ReNumber(sel_map_points, all_flat_building_points.size()+1, all_flat_building_lines.size()+1);
         all_flat_building_lines.insert(all_flat_building_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
         all_flat_building_points.insert(all_flat_building_points.end(), sel_map_points.begin(), sel_map_points.end());

         }
} //end flatreco

flatroofs.Write("flatroofs.top", false);
dxffile = fopen ("dxffile_flatbuildinglines.dxf","w");
all_flat_building_points.WriteDXF(dxffile, all_flat_building_lines, true);
fclose(dxffile);
all_flat_building_lines.Write("flatbuildinglines.top", false);
all_flat_building_points.Write("flatbuildinglines.objpts");

LineTopologies              all_flat_building_lines1;
ObjectPoints                all_flat_building_points1;

bool reconstruct_firstfloor;
reconstruct_firstfloor = true;//false; //soe 120609
if (reconstruct_firstfloor){
LaserPoints                 low_laser_points, partitionedlaserdata, sel_laser_points2;
int                         dominant_segment, success;
bool                        used;
printf("start reconstructing first floor ...\n");
newpointnumber = allsplittedpoints.HighestPointNumber().Number()+1;                           
partitionedlaserdata.Read("partitionedlaserdata.laser");
//densifiedmaplines = map_lines;// complete maplines
//densifiedmappoints = map_points;
//densifiedmappoints.RemoveDoublePoints(densifiedmaplines, 0.1);
densifiedmaplines = allsplittedlines; //partitioned lines
densifiedmappoints = allsplittedpoints; //was (stupid) mappoints
densifiedmappoints.RemoveDoublePoints(densifiedmaplines, 0.1);

low_laser_points.ErasePoints();
//low_laser_points.AddTaggedPoints(laser_points, 1000, LabelTag); //only horizontal points
low_laser_points = partitionedlaserdata; //


for (map_line = densifiedmaplines.begin(); map_line!=densifiedmaplines.end(); map_line++){
    sel_laser_points.ErasePoints();
    floor_height = 0.01*(map_line->Attribute(PredictedHeight)-1000);
    sel_map_points.erase(sel_map_points.begin(), sel_map_points.end());
    sel_laser_points = partitionedlaserdata.SelectTagValue(PolygonNumberTag, map_line->Attribute(BuildingNumberTag));
    sel_laser_points2 = sel_laser_points.SelectTagValue(PlaneNumberTag, map_line->Attribute(NodeNumberTag));
    used = false;
/*    for (sel_map_line = flatroofs.begin(); sel_map_line!=flatroofs.end(); sel_map_line++){
            if (sel_map_line->Attribute(LineNumberTag)==map_line->Attribute(BuildingNumberTag)) {
              used = true;
              fprintf(statfile, "YES MAP LINE ALLREADY USED IN FLATLINES\n");
              }
        }
  */      dominant_segment = sel_laser_points2.MostFrequentAttributeValue(SegmentNumberTag, count);
        fprintf(statfile,"sel2 size dom seg = %d\n", sel_laser_points2.size());
        sel_laser_points.ErasePoints();
        sel_laser_points = laser_points_leftover.SelectTagValue(SegmentNumberTag, dominant_segment);
 //       sel_laser_points = laser_points.SelectTagValue(SegmentNumberTag, dominant_segment);
        fprintf(statfile,"size of sellaserpoints %d\n", sel_laser_points.size());
        if (sel_laser_points.size()==0){// see if maybe flat segment
//        dominantlabel = sel_laser_points2.MostFrequentAttributeValue(LabelTag, count);
//        if (dominantlabel<1550) sel_laser_points = sel_laser_points2;
        }
    if (sel_laser_points.size()>10 && !used){
      top.clear(); top.Initialise();
      plane = sel_laser_points.FitPlane(dominant_segment, dominant_segment, SegmentNumberTag);  
      for (node2=map_line->begin(); node2!=map_line->end()-1; node2++) {
        new_point = *(densifiedmappoints.PointIterator(*node2));
        newpointnumber++;
     //   floor_height = new_point.Z();
        newedgepoint = ObjectPoint(new_point.X(), new_point.Y(), plane.Z_At(new_point.X(), new_point.Y(), &success) , newpointnumber, 0,0,0,0,0,0); 
        intsect_points.push_back(newedgepoint);
        top.push_back(PointNumber(newpointnumber));
        sel_map_points.push_back(newedgepoint);
        }
      if (!top.IsClosed()) top.push_back(*(top.begin()));
      top.SetAttribute(BuildingNumberTag, map_line->Attribute(BuildingNumberTag));
      top.SetAttribute(LineLabelTag, 11);
//      top.SetAttribute(LineLabelTag, 5);
      top.SetAttribute(SegmentLabel, dominant_segment);
      
      flatroofs.push_back(top);
      one_map_line.erase(one_map_line.begin(), one_map_line.end());
      wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
      one_map_line.push_back(top);
      sel_map_points.RemoveDoublePoints(one_map_line, 0.001);
      one_map_line.ReNumber(sel_map_points, 0, 0);
//      tin = sel_map_points.Triangulate(one_map_line);
//      map_tin_lines = LineTopologies(tin);
//      tin.Erase();
      number_offset = sel_map_points.HighestPointNumber().Number()+1;
      sel_map_points.DuplicateWithFixedZ(floor_height, number_offset);
      wall_map_lines.insert(wall_map_lines.end(), one_map_line.begin(), one_map_line.end());
      wall_map_lines.AddWalls(one_map_line, number_offset);
      wall_map_lines.ReNumber(sel_map_points, all_flat_building_points.size()+1, all_flat_building_lines.size()+1);
  //    wall_map_lines.SetAttribute(LineLabelTag, 11);
      all_flat_building_lines.insert(all_flat_building_lines.end(), wall_map_lines.begin(), wall_map_lines.end());
      all_flat_building_points.insert(all_flat_building_points.end(), sel_map_points.begin(), sel_map_points.end());
    }
    else {
        top.clear(); top.Initialise();
        for (node2=map_line->begin(); node2!=map_line->end()-1; node2++) {
          new_point = *(densifiedmappoints.PointIterator(*node2));
          newpointnumber++;
          newedgepoint = ObjectPoint(new_point.X(), new_point.Y(), floor_height+2.85, newpointnumber, 0,0,0,0,0,0); 
          intsect_points.push_back(newedgepoint);
          top.push_back(PointNumber(newpointnumber));
          sel_map_points.push_back(newedgepoint);
        }
            if (!top.IsClosed()) top.push_back(*(top.begin()));
      top.SetAttribute(BuildingNumberTag, map_line->Attribute(BuildingNumberTag));
      top.SetAttribute(LineLabelTag, 11);
//      top.SetAttribute(LineLabelTag, 5);
      flatroofs.push_back(top);
      one_map_line.erase(one_map_line.begin(), one_map_line.end());
      wall_map_lines.erase(wall_map_lines.begin(), wall_map_lines.end());
      one_map_line.push_back(top);
      sel_map_points.RemoveDoublePoints(one_map_line, 0.001);
      one_map_line.ReNumber(sel_map_points, 0, 0);
      number_offset = sel_map_points.HighestPointNumber().Number()+1;
      sel_map_points.DuplicateWithFixedZ(floor_height, number_offset);
      wall_map_lines.insert(wall_map_lines.end(), one_map_line.begin(), one_map_line.end());
      wall_map_lines.AddWalls(one_map_line, number_offset);
      wall_map_lines.SetAttribute(LineLabelTag, 5);
      wall_map_lines.ReNumber(sel_map_points, all_flat_building_points1.size()+1, all_flat_building_lines1.size()+1);
      all_flat_building_lines1.insert(all_flat_building_lines1.end(), wall_map_lines.begin(), wall_map_lines.end());
      all_flat_building_points1.insert(all_flat_building_points1.end(), sel_map_points.begin(), sel_map_points.end());
      }

}


//intsect_points.RemoveDoublePoints(flatroofs, 0.1);
flatroofs.Write("flatroofs.top", false);
all_flat_building_lines1.Write("flatbuildinglines1.top", false);
all_flat_building_points1.Write("flatbuildinglines1.objpts");
dxffile = fopen ("dxffile_flatbuildinglines.dxf","w");
all_flat_building_points1.WriteDXF(dxffile, all_flat_building_lines1, true);
fclose(dxffile);
} //end if reconstruct firstfloor


bool recoleftovers;
recoleftovers = false; //220609 soe
if (recoleftovers){
ObjectPoints       out3dobj, allout3dobj;
LineTopologies     out3dtops, allout3dtops;
LineTopology       out3dtop;
int success;
line_number3 = 0;
laser_points_leftover.ErasePoints();
segment_numbers = laser_points.AttributeValues(SegmentNumberTag);
sort(segment_numbers.begin(), segment_numbers.end());
segment_numbers_model = combinedlines.AttributedValues(SegmentLabel); 
sort(segment_numbers_model.begin(), segment_numbers_model.end());

   for (segment_number = segment_numbers.begin(); segment_number!=segment_numbers.end(); segment_number++){
       found = false;
       for (segment_number_m = segment_numbers_model.begin(); segment_number_m!=segment_numbers_model.end(); segment_number_m++){
           if (*segment_number == *segment_number_m) found = true;
       }
       if (!found){
          for (segment_number_m = low_segments.begin(); segment_number_m!=low_segments.end(); segment_number_m++){
            if (*segment_number == *segment_number_m) found = true;
          }
       }
       if (!found){ //look for segments that have been used to calc height of partition ; also exclude these
          for (segment_number_m = segment_numbers_partitions.begin(); segment_number_m!=segment_numbers_partitions.end(); segment_number_m++){
            if (*segment_number == *segment_number_m) found = true;
          }
       }
       if (!found){ //look for segments that have been used for flat roofs
          sel_map_lines = flatroofs.SelectAttributedLines(SegmentLabel, *segment_number);
          if (sel_map_lines.size()!=0) found = true;
       }

       if (!found){ //still not found, then reconstruct a rectangular shape and construct walls
       laser_points_leftover.AddTaggedPoints(laser_points, *segment_number, SegmentNumberTag);
        seg_laser_points.ErasePoints();
//        seg_laser_points.AddTaggedPoints(laser_points, *segment_number, SegmentNumberTag);
        seg_laser_points.AddTaggedPoints(partitionedlaserdata, *segment_number, SegmentNumberTag);
  //      seg_laser_points.AddTaggedPoints(pointsofnotusedsegments, *segment_number, SegmentNumberTag);
        buildingnr = seg_laser_points.MostFrequentAttributeValue(PolygonNumberTag, count);
        label2 = seg_laser_points.MostFrequentAttributeValue(LabelTag, count);
 /*       if (label2==1000){
        plane = laser_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
        OutliningParameters *opar=new OutliningParameters();
        SegmentationParameters *spar=new SegmentationParameters();
        opar->MaximumDistancePointIntersectionLine()=0.8;
        opar->MinimumAnglePreferredDirections()= 10;
        opar->HoughBinSizeDistance()= 0.1;
        opar->HoughBinSizeDirection() = 3;
        opar->MaximumDistancePointOutline()=0.5;
        opar->MaximumPointGapInOutlineSegment()=5;
        opar->MaximumGapSizeInOutlineSegment()=1;
        opar->MinimumNumberOfPointsInOutlineSegment()=5;
        out3dobj.erase(out3dobj.begin(), out3dobj.end());
        out3dtop.erase(out3dtop.begin(), out3dtop.end());
        spar->MaxDistanceInComponent() = 3;
        seg_laser_points.DeriveTIN();
        seg_laser_points.DeriveDataBounds(0);
        pdist = seg_laser_points.MedianInterPointDistance(seg_laser_points.size()-1);

     //   if (seg_laser_points.EnclosingRectangle(1, out3dobj,out3dtop)){
        seg_laser_points.DeriveContour3D(out3dobj, out3dtop, 1);
 //         if (seg_laser_points.EnclosingPolygon(out3dobj, out3dtop, *spar, *opar, false)){
          
            if (!out3dtop.IsClosed()) out3dtop.push_back(*(out3dtop.begin()));
            out3dtops.erase(out3dtops.begin(), out3dtops.end());
            printf("enclosingpolygon found, segment %d\n", *segment_number);
            for (map_point = out3dobj.begin(); map_point!=out3dobj.end();map_point++){
                 map_point->Z() = plane.Z_At(map_point->X(), map_point->Y(), &success);
            }
            out3dtop.Label() = 3;
            out3dtop.SetAttribute(BuildingNumberTag, buildingnr);
            out3dtop.SetAttribute(SegmentLabel, *segment_number);
            out3dtops.push_back(out3dtop);
            next_pnr = allout3dobj.HighestPointNumber().Number()+1;
            line_number3 = line_number3 + out3dtops.size();
            out3dtops.ReNumber(out3dobj, next_pnr, line_number3);
            allout3dtop.insert(allout3dtop.end(), out3dtops.begin(), out3dtops.end());
            allout3dobj.insert(allout3dobj.end(), out3dobj.begin(), out3dobj.end());
        //}
     }
   */  }
}
 laser_points_leftover.Write("laserpointsleftover2.laser", false);
 buildingnumbers = laser_points_leftover.AttributeValues(PolygonNumberTag);
 b1 = buildingnumbers.size();
 buildingnumbers = laser_points.AttributeValues(PolygonNumberTag);
 b2 = buildingnumbers.size();
 
 fprintf(statfile,"\n***\nNumber of buildings affected of leftover laserpoints2 %d, out of %d\n", b1, b2);
 //  allout3dtop.Write("enclosingpolygons.top", false);
//   allout3dobj.Write("enclosingpolygons.objpts");
     
}

                                                                                         
  for (sel_match_line = polygonlines.begin(); sel_match_line !=polygonlines.end(); sel_match_line++){
     polygonpoints.erase(polygonpoints.begin(), polygonpoints.end());
         for (node = sel_match_line->begin(); node !=sel_match_line->end(); node++){
            oldpoint = *(intsect_points.PointIterator(*node));
            polygonpoints.push_back(oldpoint);
     }
     normalpolygon = sel_match_line->Normal(polygonpoints);
//     printf("%4.2f\n", normalpolygon[2]);
     if (normalpolygon[2]>0) sel_match_line->RevertNodeOrder();
  }
  printf("Finished reverting node order\n");

dxffile = fopen ("dxffile_dormers.dxf","w");
intsect_points.WriteDXF(dxffile, dormerlines, true);
fclose(dxffile);

int pn;
line_number3 = 0;
pn = 0;
allpoints.erase(allpoints.begin(), allpoints.end());
alllines.erase(alllines.begin(), alllines.end());

printf("begin selecting output data\n");
for (map_line = polygonlines.begin(); map_line!=polygonlines.end(); map_line++, line_number3++){
    map_line->Number() = line_number3;
     top = *map_line;
    top.Number() = line_number3;
    for (node = top.begin(); node !=top.end(); node++, pn++){
            oldpoint = *(intsect_points.PointIterator(*node));
            oldpoint.Number() = pn;
            node->Number() = pn;
            allpoints.push_back(oldpoint);
    }
    alllines.push_back(top);
}
allpoints.RemoveDoublePoints(alllines, 0.1);
dxffile = fopen ("dxffile_polygonroofs.dxf","w");
allpoints.WriteDXF(dxffile, alllines, true);
fclose(dxffile);

allpoints.Write("allroofs.objpts");
alllines.Write("allroofs.top", false);

pn = allpoints.HighestPointNumber().Number()+1;      
printf("add dormers...\n");

for (map_line = dormerlines.begin(); map_line!=dormerlines.end(); map_line++, line_number3++){
    map_line->Number() = line_number3;
    top = *map_line;
    top.Number() = line_number3;

    for (node = top.begin(); node !=top.end(); node++, pn++){
            oldpoint = *(intsect_points.PointIterator(*node));
            oldpoint.Number() = pn;
            allpoints.push_back(oldpoint);
            node->Number() = pn;
    }
  //  top.push_back(*(top.begin()));
    alllines.push_back(top);
}
allpoints.RemoveDoublePoints(alllines, 0.1);
printf("added dormers...\n");
pn = allpoints.HighestPointNumber().Number()+1;      

//for (map_line = all_flat_building_lines.begin(); map_line!=all_flat_building_lines.end(); map_line++, line_number3++){
printf("add flat roof...\n");
 //commented out 230609 soe
for (map_line = flatroofs.begin(); map_line!=flatroofs.end(); map_line++, line_number3++){
    if (map_line->Attribute(LineLabelTag)==11){
    map_line->Number() = line_number3;
    top = *map_line;
    top.Number() = line_number3;
    for (node = top.begin(); node !=top.end(); node++, pn++){
            oldpoint = *(intsect_points.PointIterator(*node));
            oldpoint.Number() = pn;
            allpoints.push_back(oldpoint);
            node->Number() = pn;
    }
//    top.push_back(*(top.begin()));
    alllines.push_back(top);
}
}

allpoints.RemoveDoublePoints(alllines, 0.1);
//printf("print output files...\n");
pn = allpoints.HighestPointNumber().Number()+1;      
allpoints.Write("allpoints.objpts");
alllines.Write("allpoints.top", false);

temp_points = allpoints;
temp_lines = alllines;

temp_lines.SetAttribute(LineLabelTag, 6);
temp_points.Write("datadriven_to_overlay.objpts");
temp_lines.Write("datadriven_to_overlay.top", false);
//return;


maplaserpoints.ErasePoints();
printf("\nstart calculating minimum distance to map points\n");
 for (map_line = map_lines.begin(), index=0; map_line!=map_lines.end(); map_line++, index++){
//         printf("%6d (%5.1f\%)\r", index, 100.0 * index / map_lines.size());
         sel_mapresults = alllines.SelectAttributedLines(BuildingNumberTag, map_line->Number());
         sel_laser_points.ErasePoints();
         sel_laser_points.AddTaggedPoints(laser_points, map_line->Number(), PolygonNumberTag);
         for (map_line2 = sel_mapresults.begin(); map_line2!=sel_mapresults.end(); map_line2++){
              if (map_line2->size()>2){
              segnr = map_line2->Attribute(SegmentLabel);
              seg_laser_points.ErasePoints();
              seg_laser_points.AddTaggedPoints(laser_points, segnr, SegmentNumberTag);  
              if (seg_laser_points.size()>0){
//                  seg_laser_points.DeriveTIN();
//                  edges.Erase();
//                  edges.Derive(seg_laser_points.TINReference());
                  for (node=map_line2->begin(); node!=map_line2->end()-1; node++) {
//                       printf("4.1 ");
                       this_point = allpoints.PointIterator(*node);
//                       printf("4.2 ");
                       pos = this_point->vect();
                       found = false;
//                       nearestpoint = seg_laser_points.NearestPoint(pos, edges, false);
                       maplaserpoint = LaserPoint(this_point->X(), this_point->Y(), this_point->Z());
                       maplaserpoint.Residual() = 0.6;
                       maplaserpoint.SetAttribute(SegmentNumberTag, segnr);
                       maplaserpoint.SetAttribute(LabelTag, 0);
                       for (laser_point=seg_laser_points.begin(); !found, laser_point!=seg_laser_points.end(); laser_point++){
                             pos2 = Position3D(laser_point->X(), laser_point->Y(), laser_point->Z());
                             if (pos.Distance(pos2)<1){
//                               perc++;
                                 found = true;
                                 maplaserpoint.Residual() = 0;
                           }
                       }
  
                       if (!found){//seg_laser_points.Distance2NearestPoint(pos, nearestpoint)>1){
//                           maplaserpoint.Residual() = 0.6;
                           maplaserpoint.SetAttribute(LabelTag, 1);
                           countlargeres++;
                       }
                       maplaserpoints.push_back(maplaserpoint);
                  }
              }
              }
         }
 }
//maplaserpoints.ReduceData(0.1);
sel_laser_points3.ErasePoints();
sel_laser_points3.AddTaggedPoints(maplaserpoints, 1, LabelTag); // number of points inside 3d model
maplaserpoints.Write("datadrvnqualitymodelpoints.laser", false);
fprintf(statfile,"\n***\nNumber of object points > 100 cm away from nearest laserpoint %d (out of %d)\n", sel_laser_points3.size(), maplaserpoints.size());

countlargeres = 0;

maplaserpoints.ErasePoints();
printf("\nstart calculating residuals on laser points (this can take a while)\n");
   for (map_line = map_lines.begin(), index=0; map_line!=map_lines.end(); map_line++, index++){
         printf("%6d (%5.1f\%)\r", index, 100.0 * index / map_lines.size());
         sel_mapresults = alllines.SelectAttributedLines(BuildingNumberTag, map_line->Number());
         sel_laser_points.ErasePoints();
         sel_laser_points.AddTaggedPoints(laser_points, map_line->Number(), PolygonNumberTag);
         for (laser_point=sel_laser_points.begin(); laser_point!=sel_laser_points.end(); laser_point++) {
         keepresidual = 10;
         found = false;
             for (face=sel_mapresults.begin(); face!=sel_mapresults.end(); face++) {
                 if (face->size()>2){
                if (laser_point->InsidePolygon(allpoints, face->LineTopologyReference())){
                   plane.Initialise(); 
                   for (node=face->begin(); node!=face->end()-1; node++) {
                       this_point = allpoints.PointIterator(node->NumberRef());
                       plane.AddPoint(this_point->Position3DRef(), false);
                   }
                   plane.Recalculate();
                   found = true;
             //      residual = laser_point->Z() - plane.Z_At(laser_point->X(), laser_point->Y(), &success);
                   pos = laser_point->vect();
                   projpos = plane.Project(pos);
                   residual = projpos.Distance(pos);//laser_point->Z() - plane.Z_At(laser_point->X(), laser_point->Y(), &success);

                   if (fabs(residual)<fabs(keepresidual)) keepresidual = residual;
                }
                }
             }
  //       if (found){
          maplaserpoint = LaserPoint(laser_point->X(), laser_point->Y(), laser_point->Z());
          maplaserpoint.Residual() = keepresidual;
          maplaserpoint.SetAttribute(SegmentNumberTag, laser_point->Attribute(SegmentNumberTag));
          maplaserpoint.SetAttribute(PolygonNumberTag, map_line->Number());
          sel_mapresults2 = sel_mapresults.SelectAttributedLines(SegmentLabel, laser_point->Attribute(SegmentNumberTag));
//08052013          if (sel_mapresults2.size() == 0) found = false;//if this segment is not used do not take this into account
          if (found) maplaserpoint.SetAttribute(LabelTag, 1);
          if (!found) maplaserpoint.SetAttribute(LabelTag, 0);
          maplaserpoints.push_back(maplaserpoint);
          if (fabs(keepresidual)>0.2){
               countlargeres++;
               largemaplaserpoints.push_back(maplaserpoint);
               sel_laser_points2.ErasePoints();
               sel_laser_points2.AddTaggedPoints(notused, laser_point->Attribute(SegmentNumberTag), SegmentNumberTag); // check if this is a notused segment
               if (sel_laser_points2.size()>1)  {
          //       notusedmaplaserpoints.push_back(maplaserpoint);
                 countlargeresnotused++;
               }
          }
                                      
 //        }
         }
  }
maplaserpoints.Write("datadrivenqualitylaserpoints_distance2face.laser", false);
largemaplaserpoints.Write("datadrivenqualitylaserpoints_largedistance2face.laser", false);
sel_laser_points3.ErasePoints();
sel_laser_points3.AddTaggedPoints(maplaserpoints, 1, LabelTag); // number of points inside 3d model
fprintf(statfile,"points with residuals larger than 20 cm %d, out of %d (%4.2f)\n", countlargeres, maplaserpoints.size(), 100.0*countlargeres/maplaserpoints.size());
fprintf(statfile,"notusedpoints with residuals larger than 20 cm %d, out of %d notused (%4.2f)\n", countlargeresnotused, notused.size(), 100.0*countlargeresnotused/notused.size());
sel_laser_points2.ErasePoints();
sel_laser_points2.AddTaggedPoints(largemaplaserpoints, 1, LabelTag); // number of largepoints inside 3d model
fprintf(statfile,"large residuals inside 3d model %d (out of %d inside 3d model), %4.2f\n", sel_laser_points2.size(), sel_laser_points3.size(), 100.0*sel_laser_points2.size()/sel_laser_points3.size());

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
fprintf(statfile,"Number of segments with more than 20 points with large residual: %d (out of %d)\n", countlargeressegment, matchedsegment_numbers.size());
buildingnumbers = sel_laser_points4.AttributeValues(PolygonNumberTag);
b1 = buildingnumbers.size();
buildingnumbers = laser_points.AttributeValues(PolygonNumberTag);
b2 = buildingnumbers.size();
fprintf(statfile,"\n***\nNumber of buildings affected by these segment %d, out of %d = %4.2f\n", b1, b2, 100.0*b1/b2);


sel_laser_points4.Write("datadrivenqualitylaserpoints_largesegmentswithlargedistance2face.laser", false);



allpoints.Write("allpointspolygonroofs.objpts");
alllines.Write("allpointspolygonroofs.top", false);

//int countlargeres, countlargeressegment, b1, b2;
//insertsplittedlines.RemoveCollinearNodes(allsplittedpoints);
printf("add floor...\n");
for (map_line = insertsplittedlines.begin(); map_line!=insertsplittedlines.end(); map_line++, line_number3++){
    map_line->Number() = line_number3;
    top = *map_line;
    top.Number() = line_number3;
    top.Attribute(LineLabelTag) = 5;
    for (node = top.begin(); node !=top.end(); node++, pn++){
            oldpoint = *(allsplittedpoints.PointIterator(*node));
            oldpoint.Number() = pn;
            allpoints.push_back(oldpoint);
            node->Number() = pn;
    }
//    top.push_back(*(top.begin()));
    alllines.push_back(top);
}
allpoints.RemoveDoublePoints(alllines, 0.1);
printf("print output files...\n");
pn = allpoints.HighestPointNumber().Number()+1;

dxffile = fopen ("dxffile_alltrue.dxf","w");
allpoints.WriteDXF(dxffile, alllines, true);
fclose(dxffile);

//allpoints.Write("allpoints.objpts");
//alllines.Write("allpoints.top", false);


polygonlines.Write("polygonlines.top", false);
intsect_points.Write("polygonlines.objpts");
dxffile = fopen ("dxffile_polygonroofs.dxf","w");
intsect_points.WriteDXF(dxffile, polygonlines, true);
fclose(dxffile);

for (map_line = wallinsidelines.begin(); map_line!=wallinsidelines.end(); map_line++, line_number3++){
    map_line->Number() = line_number3;
    top = *map_line;
    top.Number() = line_number3;
    top.Attribute(LineLabelTag) = 5;
    for (node = top.begin(); node !=top.end(); node++, pn++){
            oldpoint = *(wallinsidepoints.PointIterator(*node));
            oldpoint.Number() = pn;
            allpoints.push_back(oldpoint);
            node->Number() = pn;
    }
  //  top.push_back(*(top.begin()));
 //   alllines.push_back(top);
}
allpoints.RemoveDoublePoints(alllines, 0.1);
pn = allpoints.HighestPointNumber().Number()+1;
allpoints.Write("allpoints.objpts");
alllines.Write("allpoints.top", false);
//return; 
for (map_line = all_flat_building_lines.begin(); map_line!=all_flat_building_lines.end(); map_line++, line_number3++){
    map_line->Number() = line_number3;
    top = *map_line;
    top.Number() = line_number3;
    top.Attribute(LineLabelTag) = 5;
    for (node = top.begin(); node !=top.end(); node++, pn++){
            oldpoint = *(all_flat_building_points.PointIterator(*node));
            oldpoint.Number() = pn;
            allpoints.push_back(oldpoint);
            node->Number() = pn;
    }
  //  top.push_back(*(top.begin()));
    alllines.push_back(top);
}
allpoints.RemoveDoublePoints(alllines, 0.1);


pn = allpoints.HighestPointNumber().Number()+1;
//return; 
for (map_line = all_flat_building_lines1.begin(); map_line!=all_flat_building_lines1.end(); map_line++, line_number3++){
    map_line->Number() = line_number3;
    top = *map_line;
    top.Number() = line_number3;
    top.Attribute(LineLabelTag) = 5;
    for (node = top.begin(); node !=top.end(); node++, pn++){
            oldpoint = *(all_flat_building_points1.PointIterator(*node));
            oldpoint.Number() = pn;
            allpoints.push_back(oldpoint);
            node->Number() = pn;
    }
  //  top.push_back(*(top.begin()));
    alllines.push_back(top);
}
allpoints.RemoveDoublePoints(alllines, 0.1);


printf("added walls...\n");
alllines.Write("datadrivenmodel.top", false);
allpoints.Write("datadrivenmodel.objpts");

return;
double inclination;
//LineTopologies::iterator        face;
int quad;
for (face=alllines.begin(); face!=alllines.end(); face++) {
   if (face->size()>2){
       plane.Initialise(); 
       for (node=face->begin(); node!=face->end()-1; node++) {
            this_point = allpoints.PointIterator(node->NumberRef());
            plane.AddPoint(this_point->Position3DRef(), false);
        }
        plane.Recalculate();
        inclination = Angle(plane.Normal(), Vector3D(0,0,1))*180/PI;
        if ((plane.Normal()[2])<0) {
             plane.Normal()[0] = plane.Normal()[0]*(-1);
             plane.Normal()[1] = plane.Normal()[1]*(-1);
             plane.Normal()[2] = plane.Normal()[2]*(-1);
         }
         if ((plane.Normal()[0])>0 && (plane.Normal()[1])>0) quad = 1; //to make it green 1 was a bit whitish
         if ((plane.Normal()[0])>0 && (plane.Normal()[1])<0) quad = 2;
         if ((plane.Normal()[0])<0 && (plane.Normal()[1])<0) quad = 3;
         if ((plane.Normal()[0])<0 && (plane.Normal()[1])>0) quad = 4;
         if (plane.IsVertical(10*PI/180)) quad = 5;
         if (plane.IsHorizontal(10*PI/180)) quad = 0;
         face->Attribute(LineLabelTag) = int(inclination/10);
         face->Attribute(LineLabelTag) = quad;
         
    }
}
allpoints.Write("allpoints_inclination.objpts");
alllines.Write("allpoints_inclination.top", false);


/*dxffile = fopen ("dxffile_roof.dxf","w");
//allpoints.WriteDXF(dxffile, alllines, false);
alllines.insert(alllines.end(), dormerlines.begin(), dormerlines.end());
//allpoints.WriteDXFMesh(dxffile, dormerlines, 8, false, false, true);
//fclose(dxffile);

dxffile = fopen ("dxffile_groundfloor.dxf","w");
allsplittedpoints.WriteDXF(dxffile, insertsplittedlines, false);
fclose(dxffile);
*/
//alllines.ReNumber(allpoints, 0, 0);

/*
all_flat_building_lines.ReNumber(all_flat_building_points, alllines.size()+1, allpoints.size()+1);
alllines.insert(alllines.end(), all_flat_building_lines.begin(), all_flat_building_lines.end());
allpoints.insert(allpoints.end(), all_flat_building_points.begin(), all_flat_building_points.end());

*/
printf("start removing collinear nodes...\n");
//all_flat_partition_lines.RemoveCollinearNodes(all_flat_partition_points);
printf("finished removing collinear nodes...\n");

dxffile = fopen ("dxffile_wall.dxf","w");
all_flat_partition_points.WriteDXF(dxffile, all_flat_partition_lines, true);
fclose(dxffile);
printf("finished writing wall dxf...\n");

//allpoints.WriteDXFMesh(dxffile, alllines, 7, false, true, true);
//alllines.insert(alllines.end(), dormerlines.begin(), dormerlines.end());
//allpoints.WriteDXFMesh(dxffile, dormerlines, 8, false, false, true);
//fclose(dxffile);
      


//alllines.insert(alllines.end(), flatbuilding
//insertsplittedlines.Write("allinsertedsplittedlines.top", false);
allsplittedpoints.Write("allsplittedlines.objpts");
printf("start writing remaining outputfiles...\n");

snappedstepedgelines.Write("snappedstepedgelines.top", false);
newsnappedstepedgelines.Write("newsnappedstepegelines.top", false);
//polygonlines.Write("polygonlines.top", false);
nearbymaplines.Write("nearbymaplines.top", false);
nearbymappoints.Write("nearbymaplines.objpts");
//maplaserpoints.RemoveTaggedPoints(1, LabelTag);
//maplaserpoints.RemoveTaggedPoints(2, LabelTag);


maplaserpoints.Write("maplaserpoints.laser", false);
intsect_points.Write(map_points_output);
//allout3dtop.Write("flatsnappedtomap.top", false);
//allout3dobj.Write("flatsnappedtomap.objpts");
fclose(statfile);

return;
}
