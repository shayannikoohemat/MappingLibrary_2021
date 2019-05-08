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
 Date   : 04-08-2008

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

void matchgraphs(char *map_points_input, char *map_topology_input,
                    char *map_points_input_graphs, char *map_topology_input_graphs,
                    char *map_points_output, char *map_topology_output, char *infofile,
                    char *laser_points_input)

{
  ObjectPoints                 map_points, map_points_graphs, matched_points,
                               out3dobj, allout3dobj, shape_points, fix_points, GBKN_points,
                               temp_points;
  LineTopologies               map_lines, map_lines_graphs, sel_map_lines,
                               sel_graph_lines, sel_matched_lines, matched_lines,
                               allout3dtop, out3dtops, shape_lines, sel_shape_lines, fix_lines, GBKN_lines,
                               hypolines, bestmatches, targetint_lines, selhypolines, notmatchedlines, notmatchedcompllines,
                               sel_shape_lines2;
  LineTopologies::iterator     map_line, map_line2, sel_map_line, map_lineg, matline, fix_line;
  LineTopology::iterator       node2, nb_node, node3, node;
  LineNumber                   findline;
  LineTopology                 matchedline, out3dtop, sel_shape_line, GBKN_line, out3dtopsmooth, shape_it_line,
                               hypoline;
  LaserPoints                  segm_laser_points, sel_laser_points, sel_segm_laser_points, outputlaserpoints,
                               sel2_points;
  LaserPoints::iterator        laser_point, next_point;
  int                          buildingnumber, lastbuildingnumber,
                               graphnumber, lastgraphnumber, i, j,
                               score, n, ii, jj, mm, tn, bn, bi, bj, ki, kki,kkj,
                               khi, khj,kbi,kbj, line_number4, tar1a, tar1b, mi, mj, line_numberh, index_line, linenr,
                               goal_z_incm;
  double                       goal_z, minz, maxz, perc4;
  vector<int>                  tv, bv, linenumbergraph, score_tar2a, score_tar2b,
                               score_tar3, match1av, match1bv, score_b2a, score_b2b, mo1, mo2,
                               keepmo1, keepmo2, hyp0, hypnew,
                               hyp0i, keepmo1i, keepmo2i, mo1i, mo2i;
  FILE                         *statfile, *allhypoinfo;
  FRSmatrix<int>               adjmatrix, targetmatrix, keeptargetmatrix,
                               linenumbermatrix, keepbuildingmatrix, linenumbertargetmatrix;
  bool                         method1, method2, startpair, method3, found_seclevel, found_thirdlevel;
  
  vector <int>               polygon_numbers, segment_numbers, polygon_numbersall, matchedlinenumbers, vbuildingnumbers, vgraphnumbers;
  vector <int>::iterator     polygon_number, segment_number, matchedlinenumber, vbuildingnumber, vgraphnumber;
  PointNumberList            component;
  PointNumberLists           polygons, segments;
  PointNumberLists::iterator polygon, segment;
  LaserPoints                seg_laser_points, sel3_points, fix_laser_points, parpoints,
                             keepparpoints, tarpoints, sel_parpoints, sel1_parpoints, potparpoints,
                             wasnodewordtsegment, sel4_points;
  LaserPoint                 fix_laser_point, parpoint, tarpoint, waswordt;
  bool                       enclosing, contour3d, keepmatchresults;
  TINEdges                   edges;
  enclosing = false;
  contour3d = true;
  
  


method1 = true;
method2 = true;
method3 = true;
  if (method2){
    LineTopology top;
    LineTopologies tops, dormertops;
    line_number4 = 0;
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 10;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(20));
    tops.push_back(top);
    top.clear();top.Initialise();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 11;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(20));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 11;
    top.Attribute(LineLabelTag) = 8;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 11;
    top.Attribute(LineLabelTag) = 8;
    top.push_back(PointNumber(20));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 12;
    top.Attribute(LineLabelTag) = 8;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 12;
    top.Attribute(LineLabelTag) = 8;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(40));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 12;
    top.Attribute(LineLabelTag) = 8;
    top.push_back(PointNumber(20));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 12;
    top.Attribute(LineLabelTag) = 8;
    top.push_back(PointNumber(20));
    top.push_back(PointNumber(40));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 13;
    top.Attribute(LineLabelTag) = 4;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(20));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(20));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 8;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 8;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(40));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 8;
    top.push_back(PointNumber(20));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 8;
    top.push_back(PointNumber(20));
    top.push_back(PointNumber(40));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 8;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(40));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(20));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 9;
    top.push_back(PointNumber(20));
    top.push_back(PointNumber(40));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 17;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(20));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 17;
    top.Attribute(LineLabelTag) = 9;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 17;
    top.Attribute(LineLabelTag) = 9;
    top.push_back(PointNumber(20));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 8;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(20));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(40));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 9;
    top.push_back(PointNumber(20));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
//    top.Number() = line_number4; line_number4++;
//    top.Attribute(TargetNumberTag) = 18;
//    top.Attribute(LineLabelTag) = 8;
//    top.push_back(PointNumber(20));
//    top.push_back(PointNumber(40));
//    tops.push_back(top);
//    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 19;
    top.Attribute(LineLabelTag) = 8;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(20));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 19;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(40));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 19;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 19;
    top.Attribute(LineLabelTag) = 9;
    top.push_back(PointNumber(20));
    top.push_back(PointNumber(40));
    tops.push_back(top);
    top.clear();top.Initialise();
//    top.Number() = line_number4; line_number4++;
//    top.Attribute(TargetNumberTag) = 19;
//    top.Attribute(LineLabelTag) = 8;
//    top.push_back(PointNumber(20));
//    top.push_back(PointNumber(30));
//    tops.push_back(top);
//    top.clear();top.Initialise();

    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20; //gambrel
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(20));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 7;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 7;
    top.push_back(PointNumber(20));
    top.push_back(PointNumber(40));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 21; //mansard
    top.Attribute(LineLabelTag) = 5;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(20));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22; //dormer
    top.Attribute(LineLabelTag) = 2;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(20));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 23; //mansardcorner convex
    top.Attribute(LineLabelTag) = 8;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(20));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 23;
    top.Attribute(LineLabelTag) = 5;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 23;
    top.Attribute(LineLabelTag) = 5;
    top.push_back(PointNumber(20));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 24; //mansardcorner concave
    top.Attribute(LineLabelTag) = 9;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(20));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 24;
    top.Attribute(LineLabelTag) = 5;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 24;
    top.Attribute(LineLabelTag) = 5;
    top.push_back(PointNumber(20));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 27;
    top.Attribute(LineLabelTag) = 9;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 27;
    top.Attribute(LineLabelTag) = 9;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(20));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 27;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(20));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 28;
    top.Attribute(LineLabelTag) = 9;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(20));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 28;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(20));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 28;
    top.Attribute(LineLabelTag) = 9;
    top.push_back(PointNumber(10));
    top.push_back(PointNumber(30));
    tops.push_back(top);
    top.clear();top.Initialise();
    
tops.Write("targetgraphs5.top", false);
 //   return;                      

tops.erase(tops.begin(), tops.end());
    line_number4 = 0;
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 10;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(1));
    top.push_back(PointNumber(2));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 10;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(1));
    top.push_back(PointNumber(4));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 10;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(2));
    top.push_back(PointNumber(3));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 10;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(3));
    top.push_back(PointNumber(4));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 10;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(1));
    top.push_back(PointNumber(5));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 10;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(2));
    top.push_back(PointNumber(6));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 10;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(5));
    top.push_back(PointNumber(6));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 10;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(3));
    top.push_back(PointNumber(6));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 10;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(5));
    top.push_back(PointNumber(4));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 10;
    top.Attribute(NodeNumberTag) = 10;
    top.Attribute(LineLabelTag) = 3;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(1));
    top.push_back(PointNumber(2));
    top.push_back(PointNumber(3));
    top.push_back(PointNumber(4));
    top.push_back(PointNumber(1));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 10;
    top.Attribute(NodeNumberTag) = 20;
    top.Attribute(LineLabelTag) = 1;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(1));
    top.push_back(PointNumber(2));
    top.push_back(PointNumber(6));
    top.push_back(PointNumber(5));
    top.push_back(PointNumber(1));
    tops.push_back(top);
    top.clear();top.Initialise();
    
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 11;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(11));
    top.push_back(PointNumber(12));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 11;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(11));
    top.push_back(PointNumber(14));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 11;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(12));
    top.push_back(PointNumber(13));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 11;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(13));
    top.push_back(PointNumber(14));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 11;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(11));
    top.push_back(PointNumber(15));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 11;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(12));
    top.push_back(PointNumber(16));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 11;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(15));
    top.push_back(PointNumber(16));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 11;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(13));
    top.push_back(PointNumber(16));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 11;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(14));
    top.push_back(PointNumber(15));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 11;
    top.Attribute(NodeNumberTag) = 10;
    top.Attribute(TargetType) = ShapeTarget;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(11));
    top.push_back(PointNumber(12));
    top.push_back(PointNumber(13));
    top.push_back(PointNumber(14));
    top.push_back(PointNumber(11));    
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 11;
    top.Attribute(NodeNumberTag) = 20;
    top.Attribute(LineLabelTag) = 1;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(11));
    top.push_back(PointNumber(12));
    top.push_back(PointNumber(16));
    top.push_back(PointNumber(15));
    top.push_back(PointNumber(11));    
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 11;
    top.Attribute(LineLabelTag) = 2;
    top.Attribute(NodeNumberTag) = 30;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(16));
    top.push_back(PointNumber(12));
    top.push_back(PointNumber(13));
    top.push_back(PointNumber(16));
    tops.push_back(top);
    top.clear();top.Initialise();
    
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 12;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(21));
    top.push_back(PointNumber(22));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 12;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(21));
    top.push_back(PointNumber(23));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 12;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(21));
    top.push_back(PointNumber(24));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 12;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(21));
    top.push_back(PointNumber(25));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 12;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(22));
    top.push_back(PointNumber(23));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 12;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(23));
    top.push_back(PointNumber(24));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 12;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(24));
    top.push_back(PointNumber(25));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 12;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(25));
    top.push_back(PointNumber(22));
    tops.push_back(top);
    top.clear();top.Initialise();
    
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 12;
    top.Attribute(NodeNumberTag) = 10;
    top.Attribute(LineLabelTag) = 1;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(21));
    top.push_back(PointNumber(24));
    top.push_back(PointNumber(25));
    top.push_back(PointNumber(21));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 12;
    top.Attribute(LineLabelTag) = 2;
    top.Attribute(NodeNumberTag) = 30;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(21));
    top.push_back(PointNumber(25));
    top.push_back(PointNumber(22));
    top.push_back(PointNumber(21));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 12;
    top.Attribute(LineLabelTag) = 3;
    top.Attribute(NodeNumberTag) = 20;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(21));
    top.push_back(PointNumber(22));
    top.push_back(PointNumber(23));
    top.push_back(PointNumber(21));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 12;
    top.Attribute(LineLabelTag) = 4;
    top.Attribute(NodeNumberTag) = 40;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(21));
    top.push_back(PointNumber(24));
    top.push_back(PointNumber(23));
    top.push_back(PointNumber(21));
    tops.push_back(top);
    top.clear();top.Initialise();
    
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 13;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(31));
    top.push_back(PointNumber(32));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;    
    top.Attribute(TargetNumberTag) = 13;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(32));
    top.push_back(PointNumber(33));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;    
    top.Attribute(TargetNumberTag) = 13;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(33));
    top.push_back(PointNumber(34));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;    
    top.Attribute(TargetNumberTag) = 13;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(34));
    top.push_back(PointNumber(31));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;    
    top.Attribute(TargetNumberTag) = 13;
    top.Attribute(LineLabelTag) = 0;
    top.push_back(PointNumber(31));
    top.push_back(PointNumber(35));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;    
    top.Attribute(TargetNumberTag) = 13;
    top.Attribute(LineLabelTag) = 0;
    top.push_back(PointNumber(32));
    top.push_back(PointNumber(36));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;    
    top.Attribute(TargetNumberTag) = 13;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(35));
    top.push_back(PointNumber(36));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;    
    top.Attribute(TargetNumberTag) = 13;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(36));
    top.push_back(PointNumber(37));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;    
    top.Attribute(TargetNumberTag) = 13;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(37));
    top.push_back(PointNumber(38));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;    
    top.Attribute(TargetNumberTag) = 13;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(38));
    top.push_back(PointNumber(35));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 13;
    top.Attribute(LineLabelTag) = 6;
    top.Attribute(NodeNumberTag) = 10;
    top.push_back(PointNumber(31));
    top.push_back(PointNumber(32));
    top.push_back(PointNumber(33));
    top.push_back(PointNumber(34));
    top.push_back(PointNumber(31));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 13;
    top.Attribute(LineLabelTag) = 0;
    top.push_back(PointNumber(31));
    top.push_back(PointNumber(32));
    top.push_back(PointNumber(36));
    top.push_back(PointNumber(35));
    top.push_back(PointNumber(31));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 13;
    top.Attribute(LineLabelTag) = 6;
    top.Attribute(NodeNumberTag) = 20;
    top.push_back(PointNumber(35));
    top.push_back(PointNumber(36));
    top.push_back(PointNumber(37));
    top.push_back(PointNumber(38));
    top.push_back(PointNumber(35));
    tops.push_back(top);
    top.clear();top.Initialise();

 
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(141));
    top.push_back(PointNumber(142));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(141));
    top.push_back(PointNumber(144));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(142));
    top.push_back(PointNumber(143));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(143));
    top.push_back(PointNumber(144));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(141));
    top.push_back(PointNumber(145));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(142));
    top.push_back(PointNumber(146));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(145));
    top.push_back(PointNumber(146));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(143));
    top.push_back(PointNumber(146));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(144));
    top.push_back(PointNumber(145));
    tops.push_back(top); 
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 1;
    top.Attribute(NodeNumberTag) = 10;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(145));
    top.push_back(PointNumber(146));
    top.push_back(PointNumber(142));
    top.push_back(PointNumber(141));
    top.push_back(PointNumber(145));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 3;
    top.Attribute(NodeNumberTag) = 20;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(144));
    top.push_back(PointNumber(143));
    top.push_back(PointNumber(142));
    top.push_back(PointNumber(141));
    top.push_back(PointNumber(144));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 2;
    top.Attribute(NodeNumberTag) = 30;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(143));
    top.push_back(PointNumber(142));
    top.push_back(PointNumber(146));
    top.push_back(PointNumber(143));
    tops.push_back(top); 
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 14;
    top.Attribute(LineLabelTag) = 4;
    top.Attribute(NodeNumberTag) = 40;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(144));
    top.push_back(PointNumber(145));
    top.push_back(PointNumber(141));
    top.push_back(PointNumber(144));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(61));
    top.push_back(PointNumber(62));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(62));
    top.push_back(PointNumber(63));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(64));
    top.push_back(PointNumber(65));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(65));
    top.push_back(PointNumber(66));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(66));
    top.push_back(PointNumber(67));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(67));
    top.push_back(PointNumber(68));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(68));
    top.push_back(PointNumber(69));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(69));
    top.push_back(PointNumber(64));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(61));
    top.push_back(PointNumber(64));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(61));
    top.push_back(PointNumber(69));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(62));
    top.push_back(PointNumber(65));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(62));
    top.push_back(PointNumber(68));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(63));
    top.push_back(PointNumber(66));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(63));
    top.push_back(PointNumber(67));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 4;
    top.Attribute(NodeNumberTag) = 10;
    top.Attribute(TargetType) = ConnectionTarget;
    top.push_back(PointNumber(61));
    top.push_back(PointNumber(62));
    top.push_back(PointNumber(68));
    top.push_back(PointNumber(69));
    top.push_back(PointNumber(61));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(NodeNumberTag) = 20;
    top.Attribute(LineLabelTag) = 1;
    top.Attribute(TargetType) = ConnectionTarget;
    top.push_back(PointNumber(63));
    top.push_back(PointNumber(62));
    top.push_back(PointNumber(65));
    top.push_back(PointNumber(66));
    top.push_back(PointNumber(63));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(LineLabelTag) = 3;
    top.Attribute(NodeNumberTag) = 30;
    top.Attribute(TargetType) = ConnectionTarget;
    top.push_back(PointNumber(63));
    top.push_back(PointNumber(62));
    top.push_back(PointNumber(68));
    top.push_back(PointNumber(67));
    top.push_back(PointNumber(63));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 16;
    top.Attribute(NodeNumberTag) = 40;
    top.Attribute(LineLabelTag) = 2;
    top.Attribute(TargetType) = ConnectionTarget;
    top.push_back(PointNumber(61));
    top.push_back(PointNumber(62));
    top.push_back(PointNumber(65));
    top.push_back(PointNumber(64));
    top.push_back(PointNumber(61));
    tops.push_back(top);
    top.clear();top.Initialise();
    
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 17;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(71));
    top.push_back(PointNumber(72));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 17;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(71));
    top.push_back(PointNumber(74));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 17;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(72));
    top.push_back(PointNumber(73));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 17;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(73));
    top.push_back(PointNumber(74));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 17;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(71));
    top.push_back(PointNumber(75));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 17;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(72));
    top.push_back(PointNumber(76));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 17;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(75));
    top.push_back(PointNumber(76));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 17;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(73));
    top.push_back(PointNumber(76));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 17;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(74));
    top.push_back(PointNumber(75));
    tops.push_back(top);
    top.clear();top.Initialise();  
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 17;
    top.Attribute(LineLabelTag) = 3;
    top.Attribute(NodeNumberTag) = 10;
    top.Attribute(TargetType) = DetailTarget;
    top.push_back(PointNumber(71));
    top.push_back(PointNumber(72));
    top.push_back(PointNumber(73));
    top.push_back(PointNumber(74));
    top.push_back(PointNumber(71));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 17;
    top.Attribute(LineLabelTag) = 1;
    top.Attribute(NodeNumberTag) = 20;
    top.Attribute(TargetType) = DetailTarget;
    top.push_back(PointNumber(71));
    top.push_back(PointNumber(72));
    top.push_back(PointNumber(76));
    top.push_back(PointNumber(75));
    top.push_back(PointNumber(71));
    tops.push_back(top);
    top.clear();top.Initialise();

    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 27;
    top.Attribute(LineLabelTag) = 3;
    top.Attribute(NodeNumberTag) = 20;
    top.Attribute(TargetType) = DetailTarget;
    top.push_back(PointNumber(71));
    top.push_back(PointNumber(72));
    top.push_back(PointNumber(73));
    top.push_back(PointNumber(74));
    top.push_back(PointNumber(71));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 27;
    top.Attribute(LineLabelTag) = 1;
    top.Attribute(NodeNumberTag) = 30;
    top.Attribute(TargetType) = DetailTarget;
    top.push_back(PointNumber(71));
    top.push_back(PointNumber(72));
    top.push_back(PointNumber(76));
    top.push_back(PointNumber(75));
    top.push_back(PointNumber(71));
    tops.push_back(top);
    top.clear();top.Initialise();

    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 28;
    top.Attribute(LineLabelTag) = 3;
    top.Attribute(NodeNumberTag) = 20;
    top.Attribute(TargetType) = DetailTarget;
    top.push_back(PointNumber(71));
    top.push_back(PointNumber(72));
    top.push_back(PointNumber(73));
    top.push_back(PointNumber(74));
    top.push_back(PointNumber(71));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 28;
    top.Attribute(LineLabelTag) = 1;
    top.Attribute(NodeNumberTag) = 30;
    top.Attribute(TargetType) = DetailTarget;
    top.push_back(PointNumber(71));
    top.push_back(PointNumber(72));
    top.push_back(PointNumber(76));
    top.push_back(PointNumber(75));
    top.push_back(PointNumber(71));
    tops.push_back(top);
    top.clear();top.Initialise();

    
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(81));
    top.push_back(PointNumber(82));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(82));
    top.push_back(PointNumber(83));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(83));
    top.push_back(PointNumber(84));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(84));
    top.push_back(PointNumber(85));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(85));
    top.push_back(PointNumber(86));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(86));
    top.push_back(PointNumber(81));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(81));
    top.push_back(PointNumber(87));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(87));
    top.push_back(PointNumber(88));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(88));
    top.push_back(PointNumber(89));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(89));
    top.push_back(PointNumber(90));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(82));
    top.push_back(PointNumber(88));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(83));
    top.push_back(PointNumber(89));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(84));
    top.push_back(PointNumber(90));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(85));
    top.push_back(PointNumber(90));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(86));
    top.push_back(PointNumber(87));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 4;
    top.Attribute(NodeNumberTag) = 10;
    top.push_back(PointNumber(81));
    top.push_back(PointNumber(82));
    top.push_back(PointNumber(83));
    top.push_back(PointNumber(84));
    top.push_back(PointNumber(85));
    top.push_back(PointNumber(86));
    top.push_back(PointNumber(81));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 1;
    top.Attribute(NodeNumberTag) = 20;
    top.push_back(PointNumber(82));
    top.push_back(PointNumber(83));
    top.push_back(PointNumber(89));
    top.push_back(PointNumber(88));
    top.push_back(PointNumber(82));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(NodeNumberTag) = 30;
    top.Attribute(LineLabelTag) = 2;
    top.push_back(PointNumber(83));
    top.push_back(PointNumber(84));
    top.push_back(PointNumber(90));
    top.push_back(PointNumber(89));
    top.push_back(PointNumber(83));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 18;
    top.Attribute(LineLabelTag) = 2;
    top.Attribute(NodeNumberTag) = 40;
    top.push_back(PointNumber(81));
    top.push_back(PointNumber(82));
    top.push_back(PointNumber(88));
    top.push_back(PointNumber(87));
    top.push_back(PointNumber(81));
    tops.push_back(top);
    
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 19;
    top.Attribute(LineLabelTag) = 4;
    top.Attribute(NodeNumberTag) = 10;
    top.push_back(PointNumber(81));
    top.push_back(PointNumber(82));
    top.push_back(PointNumber(83));
    top.push_back(PointNumber(84));
    top.push_back(PointNumber(85));
    top.push_back(PointNumber(86));
    top.push_back(PointNumber(81));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 19;
    top.Attribute(LineLabelTag) = 1;
    top.Attribute(NodeNumberTag) = 20;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(82));
    top.push_back(PointNumber(83));
    top.push_back(PointNumber(89));
    top.push_back(PointNumber(88));
    top.push_back(PointNumber(82));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 19;
    top.Attribute(NodeNumberTag) = 40;
    top.Attribute(LineLabelTag) = 2;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(83));
    top.push_back(PointNumber(84));
    top.push_back(PointNumber(90));
    top.push_back(PointNumber(89));
    top.push_back(PointNumber(83));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 19;
    top.Attribute(LineLabelTag) = 2;
    top.Attribute(NodeNumberTag) = 30;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(81));
    top.push_back(PointNumber(82));
    top.push_back(PointNumber(88));
    top.push_back(PointNumber(87));
    top.push_back(PointNumber(81));
    tops.push_back(top);
    
    top.clear();top.Initialise();    
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(201));
    top.push_back(PointNumber(202));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(201));
    top.push_back(PointNumber(204));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(202));
    top.push_back(PointNumber(203));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(203));
    top.push_back(PointNumber(204));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(201));
    top.push_back(PointNumber(205));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(202));
    top.push_back(PointNumber(206));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(205));
    top.push_back(PointNumber(206));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(205));
    top.push_back(PointNumber(208));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(206));
    top.push_back(PointNumber(207));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(207));
    top.push_back(PointNumber(208));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(203));
    top.push_back(PointNumber(210));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(204));
    top.push_back(PointNumber(209));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(209));
    top.push_back(PointNumber(210));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(209));
    top.push_back(PointNumber(208));
    tops.push_back(top);
    top.clear();top.Initialise();    
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(207));
    top.push_back(PointNumber(210));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(206));
    top.push_back(PointNumber(203));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(204));
    top.push_back(PointNumber(205));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 1;
    top.Attribute(NodeNumberTag) = 10;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(201));
    top.push_back(PointNumber(202));
    top.push_back(PointNumber(206));
    top.push_back(PointNumber(205));
    top.push_back(PointNumber(201));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 3;
    top.Attribute(NodeNumberTag) = 20;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(202));
    top.push_back(PointNumber(201));
    top.push_back(PointNumber(204));
    top.push_back(PointNumber(203));
    top.push_back(PointNumber(202));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 1;
    top.Attribute(NodeNumberTag) = 30;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(207));
    top.push_back(PointNumber(208));
    top.push_back(PointNumber(205));
    top.push_back(PointNumber(206));
    top.push_back(PointNumber(207));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 20;
    top.Attribute(LineLabelTag) = 3;
    top.Attribute(NodeNumberTag) = 40;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(203));
    top.push_back(PointNumber(204));
    top.push_back(PointNumber(209));
    top.push_back(PointNumber(210));
    top.push_back(PointNumber(203));
    tops.push_back(top);
    top.clear();top.Initialise();
    
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 21;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(211));
    top.push_back(PointNumber(212));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 21;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(211));
    top.push_back(PointNumber(214));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 21;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(212));
    top.push_back(PointNumber(213));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 21;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(213));
    top.push_back(PointNumber(214));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 21;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(215));
    top.push_back(PointNumber(216));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 21;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(216));
    top.push_back(PointNumber(217));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 21;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(217));
    top.push_back(PointNumber(218));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 21;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(218));
    top.push_back(PointNumber(219));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 21;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(219));
    top.push_back(PointNumber(212));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 21;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(211));
    top.push_back(PointNumber(220));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 21;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(220));
    top.push_back(PointNumber(215));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 21;
    top.Attribute(LineLabelTag) = 1;
    top.Attribute(NodeNumberTag) = 10;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(211));
    top.push_back(PointNumber(212));
    top.push_back(PointNumber(219));
    top.push_back(PointNumber(218));
    top.push_back(PointNumber(217));
    top.push_back(PointNumber(216));
    top.push_back(PointNumber(215));
    top.push_back(PointNumber(220));
    top.push_back(PointNumber(211));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 21;
    top.Attribute(LineLabelTag) = 6;
    top.Attribute(NodeNumberTag) = 20;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(211));
    top.push_back(PointNumber(212));
    top.push_back(PointNumber(213));
    top.push_back(PointNumber(214));
    top.push_back(PointNumber(211));
    tops.push_back(top);
    top.clear();top.Initialise();
    
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(221));
    top.push_back(PointNumber(222));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(221));
    top.push_back(PointNumber(224));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(222));
    top.push_back(PointNumber(223));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(223));
    top.push_back(PointNumber(224));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(225));
    top.push_back(PointNumber(226));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(226));
    top.push_back(PointNumber(227));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(227));
    top.push_back(PointNumber(228));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22;
    top.Attribute(LineLabelTag) = 0;
    top.push_back(PointNumber(223));
    top.push_back(PointNumber(229));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22;
    top.Attribute(LineLabelTag) = 0;
    top.push_back(PointNumber(230));
    top.push_back(PointNumber(224));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(229));
    top.push_back(PointNumber(222));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22;
    top.Attribute(LineLabelTag) = 3;
    top.push_back(PointNumber(221));
    top.push_back(PointNumber(230));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22;
    top.Attribute(LineLabelTag) = 6;
    top.push_back(PointNumber(230));
    top.push_back(PointNumber(229));
    tops.push_back(top);
  
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22;
    top.Attribute(LineLabelTag) = 1;
    top.Attribute(NodeNumberTag) = 10;
    top.Attribute(TargetType) = DetailTarget;
    top.push_back(PointNumber(228));
    top.push_back(PointNumber(227));
    top.push_back(PointNumber(226));
    top.push_back(PointNumber(225));
    top.push_back(PointNumber(228));
    //dormertops.push_back(top);
    tops.push_back(top);
 
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22;
    top.Attribute(LineLabelTag) = 6;
    top.Attribute(NodeNumberTag) = 20;
    top.Attribute(TargetType) = DetailTarget;
    top.push_back(PointNumber(221));
    top.push_back(PointNumber(222));
    top.push_back(PointNumber(223));
    top.push_back(PointNumber(224));
    top.push_back(PointNumber(221));
    //dormertops.push_back(top);
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22;
    top.Attribute(LineLabelTag) = 0;
    top.Attribute(TargetType) = DetailTarget;
    top.push_back(PointNumber(224));
    top.push_back(PointNumber(223));
    top.push_back(PointNumber(229));
    top.push_back(PointNumber(230));
    top.push_back(PointNumber(224));
    //dormertops.push_back(top);
    tops.push_back(top);
    
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22;
    top.Attribute(LineLabelTag) = 0;
    top.Attribute(TargetType) = DetailTarget;
    top.push_back(PointNumber(223));
    top.push_back(PointNumber(222));
    top.push_back(PointNumber(229));
    top.push_back(PointNumber(223));
    //dormertops.push_back(top);
    tops.push_back(top);
    
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 22;
    top.Attribute(LineLabelTag) = 0;
    top.Attribute(TargetType) = DetailTarget;
    top.push_back(PointNumber(221));
    top.push_back(PointNumber(224));
    top.push_back(PointNumber(230));
    top.push_back(PointNumber(221));
    //dormertops.push_back(top);
    tops.push_back(top);
    
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 23;
    top.Attribute(LineLabelTag) = 8;
    top.push_back(PointNumber(231));
    top.push_back(PointNumber(232));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 23;
    top.Attribute(LineLabelTag) = 5;
    top.push_back(PointNumber(232));
    top.push_back(PointNumber(234));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 23;
    top.Attribute(LineLabelTag) = 5;
    top.push_back(PointNumber(232));
    top.push_back(PointNumber(233));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 23;
    top.Attribute(LineLabelTag) = 1;
    top.Attribute(NodeNumberTag) = 10;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(231));
    top.push_back(PointNumber(232));
    top.push_back(PointNumber(234));
    top.push_back(PointNumber(235));
    top.push_back(PointNumber(231));
    //dormertops.push_back(top);
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 23;
    top.Attribute(LineLabelTag) = 2;
    top.Attribute(NodeNumberTag) = 20;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(231));
    top.push_back(PointNumber(232));
    top.push_back(PointNumber(233));
    top.push_back(PointNumber(237));
    top.push_back(PointNumber(231));
    //dormertops.push_back(top);
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 23;
    top.Attribute(LineLabelTag) = 6;
    top.Attribute(NodeNumberTag) = 30;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(232));
    top.push_back(PointNumber(233));
    top.push_back(PointNumber(236));
    top.push_back(PointNumber(234));
    top.push_back(PointNumber(232));
    //dormertops.push_back(top);
    tops.push_back(top);
       top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 24;
    top.Attribute(LineLabelTag) = 9;
    top.push_back(PointNumber(231));
    top.push_back(PointNumber(232));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 24;
    top.Attribute(LineLabelTag) = 5;
    top.push_back(PointNumber(232));
    top.push_back(PointNumber(234));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 24;
    top.Attribute(LineLabelTag) = 5;
    top.push_back(PointNumber(232));
    top.push_back(PointNumber(233));
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 24;
    top.Attribute(LineLabelTag) = 1;
    top.Attribute(NodeNumberTag) = 10;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(231));
    top.push_back(PointNumber(232));
    top.push_back(PointNumber(234));
    top.push_back(PointNumber(235));
    top.push_back(PointNumber(231));
    //dormertops.push_back(top);
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 24;
    top.Attribute(LineLabelTag) = 2;
    top.Attribute(NodeNumberTag) = 20;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(231));
    top.push_back(PointNumber(232));
    top.push_back(PointNumber(233));
    top.push_back(PointNumber(237));
    top.push_back(PointNumber(231));
    //dormertops.push_back(top);
    tops.push_back(top);
    top.clear();top.Initialise();
    top.Number() = line_number4; line_number4++;
    top.Attribute(TargetNumberTag) = 24;
    top.Attribute(LineLabelTag) = 6;
    top.Attribute(NodeNumberTag) = 30;
    top.Attribute(TargetType) = ShapeTarget;
    top.push_back(PointNumber(232));
    top.push_back(PointNumber(233));
    top.push_back(PointNumber(236));
    top.push_back(PointNumber(234));
    top.push_back(PointNumber(232));
    //dormertops.push_back(top);
    tops.push_back(top);
    
    top.clear();top.Initialise();
    
    LineTopologies seltops;
   // seltops = tops.SelectAttributedLines(TargetNumberTag, 13);
    ObjectPoint                    obj;
    ObjectPoints                   objs, dormerobjs;
    
    obj = ObjectPoint(10, 10, 20, 1, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(20, 10, 20, 2, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(20, 15, 15, 3, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(10, 15, 15, 4, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(10, 5, 15, 5, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(20, 5, 15, 6, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(30, 10, 20, 11, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(40, 10, 20, 12, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(45, 15, 15, 13, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(30, 15, 15, 14, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(30, 5, 15, 15, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(45, 5, 15, 16, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(60, 10, 20, 21, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(65, 15, 15, 22, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(55, 15, 15, 23, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(55, 5, 15, 24, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(65, 5, 15, 25, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(80, 5, 20, 31, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(80, 15, 20, 32, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(75, 15, 20, 33, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(75, 5, 20, 34, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(80, 5, 15, 35, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(80, 15, 15, 36, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(85, 5, 15, 38, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(85, 15, 15, 37, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(10, 50, 20, 61, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(10, 60, 20, 62, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(20, 60, 20, 63, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(15, 50, 15, 64, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(15, 55, 15, 65, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(20, 55, 15, 66, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(20, 65, 15, 67, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(5, 65, 15, 68, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(5, 50, 15, 69, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(30, 60, 20, 71, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(40, 60, 20, 72, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(35, 65, 15, 73, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(30, 65, 15, 74, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(30, 55, 15, 75, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(35, 55, 15, 76, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(60, 65, 20, 81, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(60, 60, 20, 82, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(55, 55, 15, 83, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(55, 45, 15, 84, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(50, 45, 10, 85, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(50, 65, 10, 86, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(70, 65, 10, 87, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(70, 50, 10, 88, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(60, 50, 10, 89, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(60, 45, 10, 90, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(100, 10, 20, 141, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(105, 10, 20, 142, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(110, 15, 15, 143, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(95, 15, 15, 144, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(95, 5, 15, 145, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(110, 5, 15, 146, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(80, 50, 20, 201, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(90, 50, 20, 202, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(90, 60, 15, 203, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(80, 60, 15, 204, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(80, 40, 15, 205, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(90, 40, 15, 206, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(90, 35, 10, 207, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(80, 35, 10, 208, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(80, 65, 10, 209, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(90, 65, 10, 210, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(105, 60, 15, 211, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(115, 60, 15, 212, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(115, 50, 15, 213, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(105, 50, 15, 214, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(100, 60, 15, 215, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(100, 65, 20, 216, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(120, 65, 20, 217, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(120, 60, 15, 218, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(115, 60, 15, 219, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(105, 60, 15, 220, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(130, 60, 15, 221, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(135, 60, 15, 222, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(135, 55, 15, 223, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(130, 55, 15, 224, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(125, 50, 5, 225, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(125, 65, 20, 226, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(140, 65, 20, 227, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(140, 50, 5, 228, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(135, 55, 10, 229, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(130, 55, 10, 230, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(150, 60, 15, 231, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(155, 60, 15, 232, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(155, 65, 15, 233, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(160, 55, 10, 234, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(150, 55, 10, 235, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(160, 65, 10, 236, 0,0,0,0,0,0);  objs.push_back(obj);
    obj = ObjectPoint(150, 65, 15, 237, 0,0,0,0,0,0);  objs.push_back(obj);
    
    tops.MakeCounterClockWise(objs);
   // seltops = tops;
    int number_offset;
    number_offset = objs.HighestPointNumber().Number()+1;
 /*   objs.DuplicateWithFixedZ(0, number_offset);
    seltops.AddWalls(tops, number_offset);
    for (fix_line= seltops.begin()+tops.size(); fix_line!=seltops.end(); fix_line++){
        fix_line->Label() = 0;//fix_line->Attribute(TargetNumberTag);//if (!sel_shape_lines.Contains(*fix_line)) {
    }
    for (fix_line= dormertops.begin(); fix_line!=dormertops.end(); fix_line++){
        for (node2=fix_line->begin(); node2!=fix_line->end(); node2++) {
             dormerobjs.push_back(*objs.PointIterator(*node2));
             }
    }
    
    dormertops.MakeCounterClockWise(dormerobjs);
    dormertops.ReNumber(dormerobjs, objs.HighestPointNumber().Number()+1, line_number4);
   // seltops.insert(seltops.end(), dormertops.begin(), dormertops.end());
 //   objs.insert(objs.end(), dormerobjs.begin(), dormerobjs.end());
    seltops.Write("targetshapes.top", false);
   */ 
    tops.Write("targetshapes.top", false);   
    objs.Write("targetshapes.objpts");
//    return;                      
    }//end method2
    
  if (method1){
  // Read input data
  if (!map_points.Read(map_points_input)) {
    printf("Error reading map points from file %s\n", map_points_input);
    exit(0);
  }
  if (!map_lines.Read(map_topology_input)) {
    printf("Error reading map lines from file %s\n", map_topology_input);
    exit(0);
  }
  if (!map_points_graphs.Read(map_points_input_graphs)) {
    printf("Error reading map points from file %s\n", map_points_input_graphs);
    exit(0);
  }
  if (!map_lines_graphs.Read(map_topology_input_graphs)) {
    printf("Error reading map lines from file %s\n", map_topology_input_graphs);
    exit(0);
    }
//   if (!targetint_lines.Read("targetintsectlines4.top")) { SOE changed 20-05-09

   if (!targetint_lines.Read("targetintsectlines4.top")) {
    printf("Error reading target intsectlines from file\n");
    exit(0);
  } 
    shape_points.Read("shapepoints.objpts");
    shape_lines.Read("shapepoints.top");
    int label_pair_step_edge = 4;
    line_number4 = 0;
    
    segm_laser_points.Read(laser_points_input); // laser points should have segmentnumber == object point number, after renumbering in top3d_buildings
    
 //   LaserPoints thinnedpoints;
  
 //   thinnedpoints.Read("southwest_segm.laser");
 //   thinnedpoints.ReduceData(0.5);
 //   thinnedpoints.Write("laserpointsthinned50cm.laser", false);
 //   return;
  
    
    
    
    statfile = fopen(infofile,"w");  
    lastbuildingnumber = map_lines.begin()->Attribute(BuildingNumberTag);
    allhypoinfo = fopen("allhypoinfo.asc", "w");
    line_numberh = 0;
    vbuildingnumbers = map_lines.AttributedValues(BuildingNumberTag);
    vgraphnumbers = map_lines_graphs.AttributedValues(TargetNumberTag);

    for (vbuildingnumber = vbuildingnumbers.begin(); vbuildingnumber!=vbuildingnumbers.end(); vbuildingnumber++){
//    for (map_line = map_lines.begin(); map_line!=map_lines.end(); map_line++){
    //    buildingnumber = int (map_line->Label()/1000);
//          buildingnumber = map_line->Attribute(BuildingNumberTag);
//          if (buildingnumber == lastbuildingnumber){
//                           sel_map_lines.push_back(*map_line);
//                           }
//          else {
          sel_map_lines = map_lines.SelectAttributedLines(BuildingNumberTag, *vbuildingnumber);
          buildingnumber = *vbuildingnumber;
          lastbuildingnumber = *vbuildingnumber;
//             sel_graph_lines.erase(sel_graph_lines.begin(), sel_graph_lines.end());
//             lastgraphnumber = map_lines_graphs.begin()->Attribute(TargetNumberTag);
             bv = sel_map_lines.TransformToGraph(map_points);
             n = int(sqrt(2*bv.size()))+1;
             bn = n;
             
             fprintf(statfile, "\n***BUILDING*** = %d, %d\n", lastbuildingnumber, sel_map_lines.size());
             fprintf(allhypoinfo,"\n***BUILDING*** = %d, %d\n", lastbuildingnumber, sel_map_lines.size());
              fprintf(allhypoinfo,"\n***graphnumbers , %d\n", vgraphnumbers.size());
             sel_laser_points.ErasePoints();
             sel_laser_points = segm_laser_points.SelectTagValue(PolygonNumberTag, lastbuildingnumber);
             if (bv.size()>0){
             adjmatrix = sel_map_lines.TransformToGraphMatrix(map_points);
             linenumbermatrix = sel_map_lines.TransformToGraphMatrix(map_points, false, true);
             // look for each target graph which lines in the building graph fits to the target
             for (vgraphnumber = vgraphnumbers.begin(); vgraphnumber!=vgraphnumbers.end();vgraphnumber++){
             sel_graph_lines = map_lines_graphs.SelectAttributedLines(TargetNumberTag, *vgraphnumber);
//             for (map_lineg = map_lines_graphs.begin(); map_lineg!=map_lines_graphs.end(); map_lineg++){
//                 graphnumber = map_lineg->Attribute(TargetNumberTag);
//                   if (graphnumber == lastgraphnumber){                       
                           // select all the lines belonging to the current target graph
//                           sel_graph_lines.push_back(*map_lineg);
//                           }
//                   else {
                     graphnumber = *vgraphnumber;
                     lastgraphnumber = *vgraphnumber;
                     tv = sel_graph_lines.TransformToGraph(map_points_graphs);
                     targetmatrix = sel_graph_lines.TransformToGraphMatrix(map_points_graphs);
                     linenumbertargetmatrix  = sel_graph_lines.TransformToGraphMatrix(map_points_graphs, false, true);
                     fprintf(statfile,"\ntargetmatrix %d, numrows = %d\n", lastgraphnumber, targetmatrix.NumRows());
                     tn = int(sqrt(2*tv.size()))+1;
                     startpair = false;
                     for (i=0 ;i!=tn-1;i++){
                       for (j=i+1;j!=tn;j++){
                         if (targetmatrix.Val(i,j)!=0) {
                             if (!startpair) {
                                tar1a = i;
                                tar1b = j;
                             }
                             fprintf(statfile," look for %d - %d: label %d", targetmatrix.Val(i,i), targetmatrix.Val(j,j), targetmatrix.Val(i,j));
                             if (!startpair) fprintf(statfile," = startpair\n");
                             else fprintf(statfile,"\n");
                             startpair = true;                            
                         }
                       }
                     }
                     fprintf(statfile," start with %d - %d: %d\n", targetmatrix.Val(tar1a,tar1a), targetmatrix.Val(tar1b,tar1b), targetmatrix.Val(tar1a,tar1b));
                     tarpoints.ErasePoints();
                     tarpoint.X() = lastgraphnumber; tarpoint.Y() = targetmatrix.Val(tar1a,tar1a); tarpoint.Z() = targetmatrix.Val(tar1a,tar1a); tarpoint.SetAttribute(UndefinedTag, 0); tarpoints.push_back(tarpoint); 
                     tarpoint.X() = lastgraphnumber; tarpoint.Y() = targetmatrix.Val(tar1b,tar1b); tarpoint.Z() = targetmatrix.Val(tar1b,tar1b); tarpoint.SetAttribute(UndefinedTag, 0); tarpoints.push_back(tarpoint); 

                     potparpoints.ErasePoints();
                     
                     match1av.erase(match1av.begin(), match1av.end());
                     match1bv.erase(match1bv.begin(), match1bv.end());
                     parpoints.ErasePoints();
                     int num1 =0;
                     for (bi=0 ;bi!=bn-1;bi++){
                        for (bj=bi+1;bj!=bn;bj++){
                           if (adjmatrix.Val(bi,bj)==targetmatrix.Val(tar1a,tar1b)){                   
                               fprintf(statfile,"\n hypo %d for targetnodes %d-%d, segmn %d - %d: (%d)\n", num1,targetmatrix.Val(tar1a,tar1a), targetmatrix.Val(tar1b,tar1b), adjmatrix.Val(bi,bi), adjmatrix.Val(bj,bj), adjmatrix.Val(bi,bj));
                               parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(tar1a,tar1a); parpoint.Z() = adjmatrix.Val(bi, bi); parpoint.SetAttribute(LabelTag, num1); parpoint.SetAttribute(ScanNumberTag, targetmatrix.Val(tar1a,tar1a)); parpoint.SetAttribute(PulseCountTag, bi); parpoint.SetAttribute(PolygonNumberTag, lastbuildingnumber); parpoint.SetAttribute(UndefinedTag, 0);parpoints.push_back(parpoint);potparpoints.push_back(parpoint);
                               parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(tar1b,tar1b); parpoint.Z() = adjmatrix.Val(bj, bj); parpoint.SetAttribute(LabelTag, num1); parpoint.SetAttribute(ScanNumberTag, targetmatrix.Val(tar1a,tar1a)); parpoint.SetAttribute(PulseCountTag, bj); parpoint.SetAttribute(PolygonNumberTag, lastbuildingnumber); parpoint.SetAttribute(UndefinedTag, 0);parpoints.push_back(parpoint);potparpoints.push_back(parpoint);                   
                               match1av.push_back(bi);
                               match1bv.push_back(bj);
                               num1++;
                           }
                        }
                     }
                     bool allreadyin = false;                                  
                     score_tar2a.erase(score_tar2a.begin(),score_tar2a.end());
                     for (j=tar1a;j!=tn;j++){
                       if (targetmatrix.Val(tar1a,j)!=0 && j!=tar1b && j!=tar1a) {
                           fprintf(statfile," then (1a) look for %d (connected with first node %d): label %d\n", targetmatrix.Val(j,j), targetmatrix.Val(tar1a,tar1a), targetmatrix.Val(tar1a,j));
                           score_tar2a.push_back(j);
//                           tarpoint.X() = tar1a; tarpoint.Y() = j; tarpoint.Z() = targetmatrix.Val(tar1a,j); tarpoints.push_back(tarpoint);
                           tarpoint.X() = lastgraphnumber; tarpoint.Y() = targetmatrix.Val(j,j); tarpoint.Z() = targetmatrix.Val(j,j); tarpoint.SetAttribute(UndefinedTag,1); tarpoints.push_back(tarpoint); 

                           for (mi=0;mi!=match1av.size();mi++){// for all hypothesis 
                            for (mj=0;mj!=bn;mj++){
                                allreadyin = false;
                                for (i=0; i!=match1bv.size();i++){ //check if mj is not allready part of match1bv... in same hypothesis group "mi"
                                  if (mj==match1bv[i] && mi==i){
                                      allreadyin = true;
                                      fprintf(statfile," allready in %d in hypo %d\n", adjmatrix.Val(mj,mj), mi);
                                  }
                                }
                                if (adjmatrix.Val(match1av[mi],mj)==targetmatrix.Val(tar1a,j)&& match1av[mi]!=mj && !allreadyin){
                                          fprintf(statfile," found (1a) for target node %d segment %d (hyponr %d): label %d (=%d)\n", targetmatrix.Val(j,j), adjmatrix.Val(mj,mj), mi, adjmatrix.Val(match1av[mi],mj),targetmatrix.Val(tar1a,j));
                                          parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(j,j); parpoint.Z() = adjmatrix.Val(mj,mj); parpoint.SetAttribute(LabelTag, mi), parpoint.SetAttribute(ScanNumberTag, targetmatrix.Val(j,j)), parpoint.SetAttribute(PulseCountTag, mj), parpoint.SetAttribute(UndefinedTag,1);
                                          sel_parpoints = parpoints.SelectTagValue(LabelTag, mi);
          //                                 fprintf(statfile," size of sel points: %d\n", sel_parpoints.size());
                                           allreadyin = false;
                                          for (laser_point = sel_parpoints.begin(); laser_point!=sel_parpoints.end(); laser_point++){
                                              if (laser_point->Z() == parpoint.Z()&& laser_point->Y() != parpoint.Y()){ //if same segment for different node...
          //                                       fprintf(statfile, " oeps, segment %d allready in same hypo, on different node (%d vs %d) hypo %d \n", int(laser_point->Z()), int(laser_point->Y()), int(parpoint.Y()), mi);
                                       //          allreadyin = true;
                                              }
                                          }                                          
                                          if (!allreadyin) {
                                                           parpoints.push_back(parpoint);
            //                                               potparpoints.push_back(parpoint);
                                                           }
                                         }
                            }
                           }
                         }
                       }
                       
                     score_tar2b.erase(score_tar2b.begin(),score_tar2b.end());
                      for (i=0;i!=tn;i++){
                        if (targetmatrix.Val(tar1b,i)!=0 && i!=tar1a && i!=tar1b) {
                           fprintf(statfile," then (1b) look for %d (connected with first node %d): label %d\n", targetmatrix.Val(i,i), targetmatrix.Val(tar1b,tar1b), targetmatrix.Val(tar1b,i));
                           score_tar2b.push_back(i);
                       //    tarpoint.X() = tar1b; tarpoint.Y() = i; tarpoint.Z() = targetmatrix.Val(tar1b,i); tarpoints.push_back(tarpoint); 
                           tarpoint.X() = lastgraphnumber; tarpoint.Y() = targetmatrix.Val(i,i); tarpoint.Z() = targetmatrix.Val(i,i); tarpoint.SetAttribute(UndefinedTag, 1); tarpoints.push_back(tarpoint); 

                           for (mi=0;mi!=match1bv.size();mi++){
                             for (mj=0;mj!=bn;mj++){//testsoe was mj=mi, seems so work fine 22-01-09
                                 allreadyin = false;
                                for (j=0; j!=match1av.size();j++){ //check if mj is not allready part of match1av... in same hypothesis group "mi"
                                  if (mj==match1av[j] && mi==j){
                                     allreadyin = true;
                                     fprintf(statfile," allready in %d in hypo %d\n", adjmatrix.Val(mj,mj), mi);
                                  }
                                }
          //                      fprintf(statfile," check %d in hypo %d, label = %d \n", adjmatrix.Val(mj,mj), mi, adjmatrix.Val(mj, match1bv[mi]));
                                if (adjmatrix.Val(mj, match1bv[mi])==targetmatrix.Val(i,tar1b)&& match1bv[mi]!=mj && !allreadyin){
                                      fprintf(statfile," found (1b) for target node %d segment %d (hyponr %d): label %d\n", targetmatrix.Val(i,i),adjmatrix.Val(mj,mj), mi, adjmatrix.Val(mj, match1bv[mi]));
                                      parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(i,i); parpoint.Z() = adjmatrix.Val(mj,mj); parpoint.SetAttribute(LabelTag, mi), parpoint.SetAttribute(ScanNumberTag, targetmatrix.Val(i,i)); parpoint.SetAttribute(PulseCountTag, mj);parpoint.SetAttribute(UndefinedTag,1); 
                                      sel_parpoints = parpoints.SelectTagValue(LabelTag, mi);
        //                              fprintf(statfile," size of sel points: %d\n", sel_parpoints.size());
                                      allreadyin = false;
                                      for (laser_point = sel_parpoints.begin(); laser_point!=sel_parpoints.end(); laser_point++){
                                         if (laser_point->Z() == parpoint.Z() && laser_point->Y()!=parpoint.Y()){//if same segment for different node...){
        //                                     fprintf(statfile, " oeps, segment %d allready in same hypo, on different node (%d vs %d) hypo %d \n", int(laser_point->Z()), int(laser_point->Y()), int(parpoint.Y()), mi);
                                         //    allreadyin = true;
                                          }
                                      }                       
                                      if (!allreadyin) {
                                                           parpoints.push_back(parpoint);
              //                                             potparpoints.push_back(parpoint);
                                                           }
                                    }
                            }
                           }
            
                         }
                       }
                               
                     score_tar3.erase(score_tar3.begin(),score_tar3.end());
                     bool trioconnection = false;
                     bool quadconnection = false;
                      for (i=0;i!=score_tar2b.size();i++){
                        for (j=0;j!=score_tar2a.size();j++){
                            
                            if (score_tar2b[i]==score_tar2a[j]){
                                trioconnection = true;
                                fprintf(statfile," closes on %d\n", targetmatrix.Val(score_tar2b[i],score_tar2a[j]));
                                tarpoint.X() = lastgraphnumber; tarpoint.Y() = targetmatrix.Val(score_tar2b[i],score_tar2b[i]); tarpoint.Z() = targetmatrix.Val(score_tar2b[i],score_tar2b[i]); tarpoint.SetAttribute(UndefinedTag, 2); tarpoints.push_back(tarpoint); 
                                sel_parpoints = parpoints.SelectTagValue(ScanNumberTag, targetmatrix.Val(score_tar2b[i],score_tar2a[j]));
                                for (ii=0; ii!=match1av.size();ii++){ //loop over number of hypos
                                found_seclevel = false;
                                  sel1_parpoints = sel_parpoints.SelectTagValue(LabelTag, ii);
                                  if (sel1_parpoints.size()>1) {
     //                                    fprintf(statfile, "start sorting...\n");
                                         sel1_parpoints.SortOnCoordinates();
                                         for (laser_point = sel1_parpoints.begin(), next_point = sel1_parpoints.begin()+1; laser_point !=sel1_parpoints.end()-1; laser_point++, next_point++){
                                         if (next_point->Label() == laser_point->Label() && next_point->Y() == laser_point->Y() && next_point->Z() == laser_point->Z()){
                       //                     laser_point->Label(-99);
                                            fprintf(statfile, " yes, found segment: %d for target-node %d, hypo %d \n", int(laser_point->Z()), int(laser_point->Y()), laser_point->Label());
                                            found_seclevel = true;
                                            laser_point->SetAttribute(PolygonNumberTag, lastbuildingnumber);
                                            laser_point->SetAttribute(UndefinedTag, 2);
                                            potparpoints.push_back(*laser_point);
                                          }
                                          }
                                  }
                                 sel1_parpoints.RemoveTaggedPoints(-99, LabelTag);

                                  for (laser_point = sel1_parpoints.begin(); laser_point!=sel1_parpoints.end(); laser_point++){
   //                                 fprintf(statfile, " segment: %d on target %d, node %d, hypo %d \n", int(laser_point->Z()), int(laser_point->X()), int(laser_point->Y()), laser_point->Label());
                                  }
                                }
                            }
                            else {
                 //              if (score_tar2b[i]<score_tar2a[j]){  
                               fprintf(statfile," then (2) look for %d - %d: label %d\n", targetmatrix.Val(score_tar2b[i],score_tar2b[i]), targetmatrix.Val(score_tar2a[j],score_tar2a[j]), targetmatrix.Val(score_tar2b[i],score_tar2a[j]));
//                               tarpoint.X() = score_tar2b[i]; tarpoint.Y() = score_tar2a[j]; tarpoint.Z() = targetmatrix.Val(score_tar2b[i],score_tar2a[j]); tarpoints.push_back(tarpoint);
                                tarpoint.X() = lastgraphnumber; tarpoint.Y() = targetmatrix.Val(score_tar2b[i],score_tar2b[i]); tarpoint.Z() = targetmatrix.Val(score_tar2b[i],score_tar2b[i]); tarpoint.SetAttribute(UndefinedTag, 3); tarpoints.push_back(tarpoint); 
                                tarpoint.X() = lastgraphnumber; tarpoint.Y() = targetmatrix.Val(score_tar2a[j],score_tar2a[j]); tarpoint.Z() = targetmatrix.Val(score_tar2a[j],score_tar2a[j]); tarpoint.SetAttribute(UndefinedTag, 3); tarpoints.push_back(tarpoint); 

                               for (ii=0; ii!=match1av.size();ii++){ //loop over number of hypos
                               found_thirdlevel = false;
                                  sel_parpoints = parpoints.SelectTagValue(LabelTag, ii);
 //                                 fprintf(statfile," size of sel points: %d, hypo %d\n", sel_parpoints.size(), ii);
                                  score_b2a.erase(score_b2a.begin(), score_b2a.end());
                                  score_b2b.erase(score_b2b.begin(), score_b2b.end());
                                  if (sel_parpoints.size()>2) {
                                      sel_parpoints.SortOnCoordinates();
                                      for (laser_point = sel_parpoints.begin(), next_point = sel_parpoints.begin()+1; laser_point !=sel_parpoints.end()-1; laser_point++, next_point++){
                                         if (next_point->Y() == laser_point->Y() && next_point->Z() == laser_point->Z()){
                                            laser_point->Label(-99);
                                          }
                                       }
                                  }
                                 sel_parpoints.RemoveTaggedPoints(-99, LabelTag);
                                  for (laser_point = sel_parpoints.begin(); laser_point!=sel_parpoints.end(); laser_point++){
                                      if (int(laser_point->Y())==targetmatrix.Val(score_tar2b[i], score_tar2b[i])){
//                                        fprintf(statfile, "found segm %d (%d) on target %d\n", int(laser_point->Z()), laser_point->Attribute(PulseCountTag), int(laser_point->Y()));
                                        score_b2a.push_back(laser_point->Attribute(PulseCountTag));
                                        parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(score_tar2b[i],score_tar2b[i]); parpoint.Z() = laser_point->Z(); parpoint.SetAttribute(LabelTag, ii), parpoint.SetAttribute(ScanNumberTag, targetmatrix.Val(score_tar2b[i],score_tar2b[i])); parpoint.SetAttribute(PulseCountTag, laser_point->Attribute(PulseCountTag)); parpoint.SetAttribute(PolygonNumberTag, lastbuildingnumber);parpoint.SetAttribute(UndefinedTag, 3);potparpoints.push_back(parpoint); 
                                      }
                                      if (int(laser_point->Y())==targetmatrix.Val(score_tar2a[j], score_tar2a[j])){
//                                        fprintf(statfile, "found segm %d (%d) on target %d\n", int(laser_point->Z()), laser_point->Attribute(PulseCountTag), int(laser_point->Y()));
                                        score_b2b.push_back(laser_point->Attribute(PulseCountTag));
                                        parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(score_tar2a[j],score_tar2a[j]); parpoint.Z() = laser_point->Z(); parpoint.SetAttribute(LabelTag, ii), parpoint.SetAttribute(ScanNumberTag, targetmatrix.Val(score_tar2a[j],score_tar2a[j])); parpoint.SetAttribute(PulseCountTag, laser_point->Attribute(PulseCountTag)); parpoint.SetAttribute(PolygonNumberTag, lastbuildingnumber);parpoint.SetAttribute(UndefinedTag, 3); potparpoints.push_back(parpoint);
                                      }
                                  }
                                  for (bi=0; bi!=score_b2a.size(); bi++){
                                      for (bj=0; bj!=score_b2b.size(); bj++){
                                          if (score_b2a[bi]!=score_b2b[bj]){
                                             if (adjmatrix.Val(score_b2a[bi], score_b2b[bj])==targetmatrix.Val(score_tar2b[i],score_tar2a[j])){
                                                  fprintf(statfile, "yes, (hypo %d) found match on segm %d & %d on label %d\n", ii, adjmatrix.Val(score_b2a[bi], score_b2a[bi]), adjmatrix.Val(score_b2b[bj], score_b2b[bj]), adjmatrix.Val(score_b2a[bi], score_b2b[bj]));
                                                  found_thirdlevel = true;
                                //                  parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(score_tar2b[i],score_tar2b[i]); parpoint.Z() = adjmatrix.Val(score_b2a[bi], score_b2a[bi]); parpoint.SetAttribute(LabelTag, ii), parpoint.SetAttribute(ScanNumberTag, targetmatrix.Val(score_tar2b[i],score_tar2b[i])); parpoint.SetAttribute(PulseCountTag, score_b2a[bi]); parpoint.SetAttribute(PolygonNumberTag, lastbuildingnumber);parpoint.SetAttribute(UndefinedTag, 3);potparpoints.push_back(parpoint); 
                                //                  parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(score_tar2a[j],score_tar2a[j]); parpoint.Z() = adjmatrix.Val(score_b2b[bj], score_b2b[bj]); parpoint.SetAttribute(LabelTag, ii), parpoint.SetAttribute(ScanNumberTag, targetmatrix.Val(score_tar2a[j],score_tar2a[j])); parpoint.SetAttribute(PulseCountTag, score_b2b[bj]); parpoint.SetAttribute(PolygonNumberTag, lastbuildingnumber);parpoint.SetAttribute(UndefinedTag, 3); potparpoints.push_back(parpoint);
                                                  }
                                          }
                                      }
                                  }
                                                                              
                    //            }                     
                            }
                            }
                         }
                      }
                      for (laser_point = tarpoints.begin(); laser_point !=tarpoints.end(); laser_point++){
            //               fprintf(allhypoinfo, " %d %d %d %d \n", int(laser_point->X()), int(laser_point->Y()), int(laser_point->Z()), int(laser_point->Attribute(UndefinedTag)));
                          }
                      for (ii=0; ii!=match1av.size();ii++){ //loop over number of hypos
                         sel_parpoints = potparpoints.SelectTagValue(LabelTag, ii);
                         if (sel_parpoints.size()>2) {
                           sel_parpoints.SortOnCoordinates();
                           for (laser_point = sel_parpoints.begin(), next_point = sel_parpoints.begin()+1; laser_point !=sel_parpoints.end()-1; laser_point++, next_point++){
                             if (next_point->Y() == laser_point->Y() && next_point->Z() == laser_point->Z()){
                                 laser_point->Label(-99);
                             }
                             }
                          sel_parpoints.RemoveTaggedPoints(-99, LabelTag);
                           }
                        fprintf(allhypoinfo, "targetnumber %d, hypo %d \n",lastgraphnumber, ii);
                        mo1.erase(mo1.begin(), mo1.end());
                        mo2.erase(mo2.begin(), mo2.end());
                        keepmo1.erase(keepmo1.begin(), keepmo1.end());
                        keepmo2.erase(keepmo2.begin(), keepmo2.end());
                        hyp0.erase(hyp0.begin(), hyp0.end());
                        hyp0.resize(tn);
                        mo1i.erase(mo1i.begin(), mo1i.end());
                        mo2i.erase(mo2i.begin(), mo2i.end());
                        keepmo1i.erase(keepmo1i.begin(), keepmo1i.end());
                        keepmo2i.erase(keepmo2i.begin(), keepmo2i.end());
                        hyp0i.erase(hyp0i.begin(), hyp0i.end());
                        hyp0i.resize(tn);
                        int ml1 = -1;
                        int ml2 = -2;
                        for (i=0 ;i!=tn;i++){
                          fprintf(allhypoinfo, "%d = ",targetmatrix.Val(i,i));
                          bool found = false;
                          bool multi = false;
                          hyp0[i]=-1;
                          hyp0i[i]=-1;
                          for (laser_point = sel_parpoints.begin(); laser_point!=sel_parpoints.end(); laser_point++){
                               if (int(laser_point->Y())==targetmatrix.Val(i, i)){
                                if (found) {
                                           multi = true;
                                           }
                                mo2.push_back(int(laser_point->Z()));
                                mo2i.push_back(laser_point->Attribute(PulseCountTag)); 
                                fprintf(allhypoinfo, " %d ", int(laser_point->Z()));
                                if (!found) {
                                           hyp0[i]=int(laser_point->Z());
                                           hyp0i[i]=laser_point->Attribute(PulseCountTag); 
                                           }
                                found = true;
                               }
                               
                          }
                          if (mo1.size()==0 && multi) {
                                             mo1=mo2;
                                             mo1i=mo2i;
                                             ml1 = i;
                                             mo2.erase(mo2.begin(), mo2.end());
                                             mo2i.erase(mo2i.begin(), mo2i.end());
                                             }
                          if (!found) fprintf(allhypoinfo, "*\n");
                          if (found && multi) fprintf(allhypoinfo, "multiple options (mo1 %d, mo2 %d)\n", mo1.size(), mo2.size());
                          if (found && !multi) fprintf(allhypoinfo, "\n");
                          if (mo1.size()>0) {
                                            keepmo1 = mo1;
                                            keepmo1i = mo1i;
                                            }
                          if (mo1.size()>0 && mo2.size()>1) {
                                           keepmo2 = mo2;
                                           ml2=i;
                                           keepmo2i = mo2i;
                                           }
                          
                         mo2.erase(mo2.begin(), mo2.end());
                         mo2i.erase(mo2i.begin(), mo2i.end());
                         }
                         int numtot=0;
                         int it1, it2, jm1;
                         
                         if (keepmo1.size()>0){
                           if (keepmo2.size()>0) {
                              
                              for (jm1=0; jm1!=keepmo1.size();jm1++){
                                for (jj=0; jj!=keepmo2.size();jj++){
                                 if (keepmo1[jm1]!=keepmo2[jj]){
                                     allreadyin = false;
                                     for (it1=0; it1!=jm1; it1++){
                                         if (keepmo2[jj]==keepmo1[it1]){
                                            allreadyin = true;
                                         }
                                     }
                                     if (!allreadyin) {
                                       fprintf(allhypoinfo,"ml1 = %d, ml2 = %d\n", ml1, ml2);
                                       fprintf (allhypoinfo,"hyp %d %d %d %d \n", hyp0[0], hyp0[1], keepmo1[jm1], keepmo2[jj]);
                                      // fprintf (allhypoinfo,"index %d %d %d %d \n", hyp0i[0], hyp0i[1], keepmo1i[jm1], keepmo2i[jj]);
                                       hypnew.erase(hypnew.begin(), hypnew.end());
                                       hypnew.resize(tn);
                                       hypoline.Number() = line_numberh;
                                       hypoline.Attribute(TargetNumberTag) = lastgraphnumber;
                                       hypnew[0]=hyp0i[0], hypnew[1]=hyp0i[1], hypnew[2]=keepmo1i[jm1], hypnew[3]=keepmo2i[jj];
                                       int lastsegm = -2;
                                       fprintf (allhypoinfo,"index ");
                                       for (i=0;i!=tn;i++){
                                          if (hypnew[i]==lastsegm){
                                                            hypnew[i]=-1;
                                                            }
                                          lastsegm = hypnew[i];
                                          fprintf(allhypoinfo,"%d ", hypnew[i]);
                                       }
                                       fprintf (allhypoinfo,"\n");
                                       int score = 0;
                                       sel3_points.ErasePoints();
                                       int coverage = 0;
                                       sel_matched_lines.erase(sel_matched_lines.begin(), sel_matched_lines.end());
                                     for (i=0 ;i!=tn;i++){
                                    fprintf(allhypoinfo," Node: %d matches with ", targetmatrix.Val(i,i));
                                     if (hypnew[i]!=-1) {
                                       fprintf(allhypoinfo," segment %d \n", adjmatrix.Val(hypnew[i],hypnew[i]));
                                       hypoline.push_back(PointNumber(adjmatrix.Val(hypnew[i],hypnew[i])));
                                       waswordt.X() = targetmatrix.Val(i,i); //targetnodenumber
                                       waswordt.Y() = adjmatrix.Val(hypnew[i],hypnew[i]); //segmentnumber
                                       waswordt.Z() = line_numberh; //linehypothesis number
                                       wasnodewordtsegment.push_back(waswordt);
                                       sel3_points.AddTaggedPoints(sel_laser_points, adjmatrix.Val(hypnew[i],hypnew[i]), SegmentNumberTag);
                                       for (j=i;j!=tn;j++){
                                           if (j!=i){
                                            fprintf(allhypoinfo,"int sections labels %d vs ", targetmatrix.Val(i,j));   
                                            if (hypnew[j]!=-1) {
                                              fprintf(allhypoinfo," %d\n", adjmatrix.Val(hypnew[i], hypnew[j]));
                                              findline = LineNumber(linenumbermatrix.Val(hypnew[i], hypnew[j]));
                                              index_line = shape_lines.FindLine(findline);
                                              matchedline = shape_lines[index_line];
                                              if (adjmatrix.Val(hypnew[i], hypnew[j])!=0 && (adjmatrix.Val(hypnew[i], hypnew[j])-targetmatrix.Val(i,j)==0)) matchedline.Attribute(CorrespondingTargetLineNumber)=linenumbertargetmatrix.Val(i,j); 
                                              if (adjmatrix.Val(hypnew[i], hypnew[j])!=0) sel_matched_lines.push_back(matchedline);
                                              if (targetmatrix.Val(i,j)!=0 && (adjmatrix.Val(hypnew[i], hypnew[j])-targetmatrix.Val(i,j)!=0)) score=score+5;
                                              if (targetmatrix.Val(i,j)==0 && (adjmatrix.Val(hypnew[i], hypnew[j])!=0)) score=score+15;
                                            }
                                            else { 
                                               fprintf(allhypoinfo," not found\n");
                                               if (targetmatrix.Val(i,j)!=0) score = score+5;
                                            }
                                           }
                                           }
                                        }
                                     else {
                                       fprintf(allhypoinfo," ... not found\n");
                                       score = score+20;
                                       hypoline.push_back(PointNumber(0));
                                       waswordt.X() = targetmatrix.Val(i,i); //targetnodenumber
                                       waswordt.Y() = -3; //segment not found
                                       waswordt.Z() = line_numberh; //linehypothesis number
                                       wasnodewordtsegment.push_back(waswordt);
                                     }
                                     }
                                     sel3_points.DeriveDataBounds(0);
                                     minz = sel3_points.DataBounds().Minimum().Z();
                                     maxz = sel3_points.DataBounds().Maximum().Z();
                                     goal_z = minz;
                                     perc4 = sel3_points.ReturnHeightOfPercentilePoint(4);
                                     fprintf(allhypoinfo, "minz = %4.2f, maxz = %4.2f, percentile %8f\n", minz, maxz, perc4);
                                     if (0.1*(maxz-minz)< (perc4-minz)){ //if the lowest x% of the points differ more than 10% of the height diff take the height of xth percentile point
                                         goal_z = perc4;
                                     }
                                     goal_z_incm = int(100*goal_z);
                                     hypoline.Attribute(PredictedHeight) = goal_z_incm;

                                     coverage = int(100.0*sel3_points.size()/sel_laser_points.size());
                                     fprintf(allhypoinfo," And the score is %d, coverage = %d\n", score, coverage);
                                     
                                     // make one topology from this match result, push back in hypolines
                                     hypoline.Attribute(MatchResultTag) = score;
                                     hypoline.Attribute(CoverageTag) = coverage;
                                     hypoline.Attribute(BuildingNumberTag) = lastbuildingnumber;
                                     hypolines.push_back(hypoline);
                                     hypoline.clear();
                                     
                                     
                                     // give the matched shapelines a label for the score, and give them the label for the accompanying hypoline
                                     for (fix_line= sel_matched_lines.begin(); fix_line!=sel_matched_lines.end(); fix_line++){
                                          fix_line->Attribute(TargetNumberTag) = lastgraphnumber;
                                          fix_line->Attribute(MatchResultTag) = score;
                                          fix_line->Attribute(BuildingNumberTag) = lastbuildingnumber;
                                          fix_line->Attribute(HypoNumberTag) = line_numberh;
                                          fix_line->Attribute(LineNumberTag) = fix_line->Number();
                                          fix_lines.push_back(*fix_line);
                                     }
                                     line_numberh++;                                     
                                     }
                                 }                                 
                                }
                              }
                            }
                            else { //if only keepmo1>0
                               fprintf(allhypoinfo,"ml1 = %d\n", ml1);
                               for (jm1=0;jm1!=keepmo1.size();jm1++){ // for all multiple options create hypo
                                   fprintf(allhypoinfo,"keepmo1 [%d]= %d\n", jm1, keepmo1[jm1]);
                                   allreadyin = false;
                                   for (it1=0; it1!=tn; it1++){
                                       if (it1!=ml1){
                                            if (keepmo1[jm1]==hyp0[it1]){
                                               allreadyin = true;
                                               fprintf(allhypoinfo,"what now in? %d\n", keepmo1[jm1]);
                                            }
                                       }
                                    }           
                                    if (!allreadyin){
                                       fprintf (allhypoinfo,"extra hyp "); 
                                       for (i=0;i!=tn;i++){
                                         if (i==ml1){
                                          fprintf (allhypoinfo," %d ", keepmo1[jm1]);
                                         }
                                         else{
                                            fprintf (allhypoinfo," %d ", hyp0[i]);    
                                         }
                                      }
                                      fprintf(allhypoinfo,"\n"); 
                     //               }
                     //               if (!allreadyin){
                      //                 fprintf (allhypoinfo,"index hyp "); 
                                       hypnew.erase(hypnew.begin(), hypnew.end());
                                       hypnew.resize(tn);
                                       hypoline.Number() = line_numberh;
                                       hypoline.Attribute(TargetNumberTag) = lastgraphnumber;
                     
                                       for (i=0;i!=tn;i++){
                                         if (i==ml1){
                   //                       fprintf (allhypoinfo," %d ", keepmo1i[jm1]);
                                          hypnew[i] = keepmo1i[jm1];
                                         }
                                         else{
                                              hypnew[i]=hyp0i[i];
                    //                        fprintf (allhypoinfo," %d ", hyp0i[i]);    
                                         }
                                      }
                    //                  fprintf(allhypoinfo,"\n");
                                      int lastsegm = -2;
                                      fprintf (allhypoinfo,"index ");
                                       for (i=0;i!=tn;i++){
                                          if (hypnew[i]==lastsegm){
                                                            hypnew[i]=-1;
                                                            }
                                          lastsegm = hypnew[i];
                                          fprintf(allhypoinfo,"%d ", hypnew[i]);
                                       }
                                       fprintf (allhypoinfo,"\n");
                                  
                                      int score = 0;
                                      sel3_points.ErasePoints();
                                 int coverage = 0;
                                 sel_matched_lines.erase(sel_matched_lines.begin(), sel_matched_lines.end());
                                     for (i=0 ;i!=tn;i++){
                                    fprintf(allhypoinfo," Node: %d matches with ", targetmatrix.Val(i,i));
                                     if (hypnew[i]!=-1) {
                                       fprintf(allhypoinfo," segment %d \n", adjmatrix.Val(hypnew[i],hypnew[i]));
                                       hypoline.push_back(PointNumber(adjmatrix.Val(hypnew[i],hypnew[i])));
                                       waswordt.X() = targetmatrix.Val(i,i); //targetnodenumber
                                       waswordt.Y() = adjmatrix.Val(hypnew[i],hypnew[i]); //segmentnumber
                                       waswordt.Z() = line_numberh; //linehypothesis number
                                       wasnodewordtsegment.push_back(waswordt);
                                       sel3_points.AddTaggedPoints(sel_laser_points, adjmatrix.Val(hypnew[i],hypnew[i]), SegmentNumberTag);

                                       for (j=i;j!=tn;j++){
                                           if (j!=i){
                                            fprintf(allhypoinfo,"int sections labels %d vs ", targetmatrix.Val(i,j));   
                                            if (hypnew[j]!=-1) {
                                              fprintf(allhypoinfo," %d\n", adjmatrix.Val(hypnew[i], hypnew[j]));
                                              findline = LineNumber(linenumbermatrix.Val(hypnew[i], hypnew[j]));
                                              index_line = shape_lines.FindLine(findline);
                                              matchedline = shape_lines[index_line];
                                              if (adjmatrix.Val(hypnew[i], hypnew[j])!=0 && (adjmatrix.Val(hypnew[i], hypnew[j])-targetmatrix.Val(i,j)==0)) matchedline.Attribute(CorrespondingTargetLineNumber)=linenumbertargetmatrix.Val(i,j);                                               
                                              if (adjmatrix.Val(hypnew[i], hypnew[j])!=0) sel_matched_lines.push_back(matchedline);
                                               if (targetmatrix.Val(i,j)!=0 && (adjmatrix.Val(hypnew[i], hypnew[j])-targetmatrix.Val(i,j)!=0)) score=score+5;
                                              if (targetmatrix.Val(i,j)==0 && (adjmatrix.Val(hypnew[i], hypnew[j])!=0)) score=score+15;
                                            }
                                            else { 
                                               fprintf(allhypoinfo," not found\n");
                                               if (targetmatrix.Val(i,j)!=0) score = score+5;
                                            }
                                           }
                                           }
                                        }
                                     else {
                                       fprintf(allhypoinfo," ... not found\n");
                                       score = score+20;
                                       hypoline.push_back(PointNumber(0));
                                       waswordt.X() = targetmatrix.Val(i,i); //targetnodenumber
                                       waswordt.Y() = -3; //segment not found
                                       waswordt.Z() = line_numberh; //linehypothesis number
                                       wasnodewordtsegment.push_back(waswordt);
                                     }
                                     }
                                     sel3_points.DeriveDataBounds(0);
                                     minz = sel3_points.DataBounds().Minimum().Z();
                                     maxz = sel3_points.DataBounds().Maximum().Z();
                                     goal_z = minz;
                                     perc4 = sel3_points.ReturnHeightOfPercentilePoint(4);
                                     fprintf(allhypoinfo, "minz = %4.2f, maxz = %4.2f, percentile %8f\n", minz, maxz, perc4);
                                     if (0.1*(maxz-minz)< (perc4-minz)){ //if the lowest x% of the points differ more than 10% of the height diff take the height of xth percentile point
                                         goal_z = perc4;
                                     }
                                     goal_z_incm = int(100*goal_z);
                                     hypoline.Attribute(PredictedHeight) = goal_z_incm;

                                      coverage = int(100.0*sel3_points.size()/sel_laser_points.size());
                                     fprintf(allhypoinfo," And the score is %d, coverage = %d\n", score, coverage);
                                     
                                     // make one topology from this match result, push back in hypolines
                                     hypoline.Attribute(MatchResultTag) = score;
                                     hypoline.Attribute(CoverageTag) = coverage;
                                     hypoline.Attribute(BuildingNumberTag) = lastbuildingnumber;
                                     hypolines.push_back(hypoline);
                                     hypoline.clear();
                                     
                                     // give the matched shapelines a label for the score, and give them the label for the accompanying hypoline
                                     for (fix_line= sel_matched_lines.begin(); fix_line!=sel_matched_lines.end(); fix_line++){
                                          fix_line->Attribute(TargetNumberTag) = lastgraphnumber;
                                          fix_line->Attribute(MatchResultTag) = score;
                                          fix_line->Attribute(BuildingNumberTag) = lastbuildingnumber;
                                          fix_line->Attribute(HypoNumberTag) = line_numberh;
                                          fix_line->Attribute(LineNumberTag) = fix_line->Number();
                                          fix_lines.push_back(*fix_line);
                                     }
                                     line_numberh++;
                                     
                                    }
                                    }
                                }
                             }
                             else { //if also keepmo1=0
                                 fprintf(allhypoinfo, "\n hyp0 ");
                                 int lastsegm = -2;
                                 for (i=0;i!=tn;i++){
                                     if (hyp0[i]==lastsegm){
                                                            hyp0[i]=-1;
                                                            hyp0i[i]=-1;
                                                            }
                                     lastsegm = hyp0[i];
                                 }
                                  
                                 for (i=0;i!=tn;i++){
                                     for (j=i;j!=tn;j++){
                                         if (j!=i && hyp0[j]==hyp0[i]){
                                            hyp0[j]=-1;
                                            hyp0i[j]=-1;
                                         }
                                     }
                                 }
                                      
                                 for (i=0;i!=tn;i++){
                                     fprintf(allhypoinfo, "%d ", hyp0[i]);
                                     }
                                 fprintf(allhypoinfo, "\n");
                                 hypnew.erase(hypnew.begin(), hypnew.end());
                                 hypnew.resize(tn);
                                 hypoline.Number() = line_numberh; 
                                 hypoline.Attribute(TargetNumberTag) = lastgraphnumber;
                                 fprintf(allhypoinfo, "index ");
                                 for (i=0;i!=tn;i++){
                                     hypnew[i]=hyp0i[i];
                                     fprintf(allhypoinfo, "%d ", hyp0i[i]);
                                     }
                                 fprintf(allhypoinfo, "\n");
                                 int score = 0;
                                 sel3_points.ErasePoints();
                                 int coverage = 0;
                                 sel_matched_lines.erase(sel_matched_lines.begin(), sel_matched_lines.end());
                                     for (i=0 ;i!=tn;i++){
                                    fprintf(allhypoinfo," Node: %d matches with ", targetmatrix.Val(i,i));
                                     if (hypnew[i]!=-1) {
                                       fprintf(allhypoinfo," segment %d \n", adjmatrix.Val(hypnew[i],hypnew[i]));
                                       hypoline.push_back(PointNumber(adjmatrix.Val(hypnew[i],hypnew[i])));
                                       waswordt.X() = targetmatrix.Val(i,i); //targetnodenumber
                                       waswordt.Y() = adjmatrix.Val(hypnew[i],hypnew[i]); //segmentnumber
                                       waswordt.Z() = line_numberh; //linehypothesis number
                                       wasnodewordtsegment.push_back(waswordt);
                                       sel3_points.AddTaggedPoints(sel_laser_points, adjmatrix.Val(hypnew[i],hypnew[i]), SegmentNumberTag);
                                       for (j=i;j!=tn;j++){
                                           if (j!=i){
                                            fprintf(allhypoinfo,"int sections labels %d vs ", targetmatrix.Val(i,j));   
                                            if (hypnew[j]!=-1) {
                                              fprintf(allhypoinfo," %d\n", adjmatrix.Val(hypnew[i], hypnew[j]));
                                              findline = LineNumber(linenumbermatrix.Val(hypnew[i], hypnew[j]));
                                              index_line = shape_lines.FindLine(findline);
                                              matchedline = shape_lines[index_line];
                                              if (adjmatrix.Val(hypnew[i], hypnew[j])!=0 && (adjmatrix.Val(hypnew[i], hypnew[j])-targetmatrix.Val(i,j)==0)) matchedline.Attribute(CorrespondingTargetLineNumber)=linenumbertargetmatrix.Val(i,j); 
                                              if (adjmatrix.Val(hypnew[i], hypnew[j])!=0) sel_matched_lines.push_back(matchedline);
                                              if (targetmatrix.Val(i,j)!=0 && (adjmatrix.Val(hypnew[i], hypnew[j])-targetmatrix.Val(i,j)!=0)) score=score+5;
                                              if (targetmatrix.Val(i,j)==0 && (adjmatrix.Val(hypnew[i], hypnew[j])!=0)) score=score+15;
                                            }
                                            else { 
                                               fprintf(allhypoinfo," not found\n");
                                               if (targetmatrix.Val(i,j)!=0) score = score+5;
                                            }
                                           }
                                           }
                                        }
                                     else {
                                       fprintf(allhypoinfo," ... not found\n");
                                       score = score+20;
                                       hypoline.push_back(PointNumber(0));
                                       waswordt.X() = targetmatrix.Val(i,i); //targetnodenumber
                                       waswordt.Y() = -3; //segment not found
                                       waswordt.Z() = line_numberh; //linehypothesis number
                                       wasnodewordtsegment.push_back(waswordt);
                                     }
                                     }
                                     sel3_points.DeriveDataBounds(0);
                                     minz = sel3_points.DataBounds().Minimum().Z();
                                     maxz = sel3_points.DataBounds().Maximum().Z();
                                     goal_z = minz;
                                     perc4 = sel3_points.ReturnHeightOfPercentilePoint(4);
                                     fprintf(allhypoinfo, "minz = %4.2f, maxz = %4.2f, percentile %8f\n", minz, maxz, perc4);
                                     if (0.1*(maxz-minz)< (perc4-minz)){ //if the lowest x% of the points differ more than 10% of the height diff take the height of xth percentile point
                                         goal_z = perc4;
                                     }
                                     coverage = int(100.0*sel3_points.size()/sel_laser_points.size());
                                     fprintf(allhypoinfo," And the score is %d, coverage = %d\n", score, coverage);
                                     goal_z_incm = int(100*goal_z);
                                     hypoline.Attribute(PredictedHeight) = goal_z_incm;
                                     
                                     // make one topology from this match result, push back in hypolines
                                     hypoline.Attribute(MatchResultTag) = score;
                                     hypoline.Attribute(CoverageTag) = coverage;
                                     hypoline.Attribute(BuildingNumberTag) = lastbuildingnumber;
                                     hypolines.push_back(hypoline);
                                     hypoline.clear();
                                     // give the matched shapelines a label for the score, and give them the label for the accompanying hypoline
                                     for (fix_line= sel_matched_lines.begin(); fix_line!=sel_matched_lines.end(); fix_line++){
                                          fix_line->Attribute(TargetNumberTag) = lastgraphnumber;
                                          fix_line->Attribute(MatchResultTag) = score;
                                          fix_line->Attribute(BuildingNumberTag) = lastbuildingnumber;
                                          fix_line->Attribute(HypoNumberTag) = line_numberh;
                                          fix_line->Attribute(LineNumberTag) = fix_line->Number();
                                          fix_lines.push_back(*fix_line);
                                     }
                                     line_numberh++;
                                     
                             }
    
                      }   
                      for (ii=0; ii!=match1av.size();ii++){ //loop over number of hypos
  //                         fprintf(allhypoinfo, "%d = %d\n", targetmatrix.Val(tar1a, tar1a), match1av[ii]);
  //                         fprintf(allhypoinfo, "%d = %d\n",  targetmatrix.Val(tar1b, tar1b), match1bv[ii]);
                           
                           sel_parpoints = potparpoints.SelectTagValue(LabelTag, ii);
                           sel_parpoints.SortOnCoordinates();
                           for (laser_point = sel_parpoints.begin(); laser_point !=sel_parpoints.end(); laser_point++){
                     //           fprintf(allhypoinfo, " %d %d %d %d %d %d\n", laser_point->Attribute(PolygonNumberTag), int(laser_point->X()), int(laser_point->Y()), int(laser_point->Z()), laser_point->Attribute(LabelTag), int(laser_point->Attribute(UndefinedTag)));
                                       }
                                  }
                    
                /*     for (i=0 ;i!=tn-1;i++){
                       for (j=i+1;j!=tn;j++){
                         if (targetmatrix.Val(i,j)!=0) {
                             fprintf(statfile," t0(%d,%d) = t0(%d, %d): (%d)", i, j, targetmatrix.Val(i,i), targetmatrix.Val(j,j), targetmatrix.Val(i,j));
                   
                             do {
                             keepbuildingmatrix = adjmatrix;
                             if (parpoints.size() > 0) parpoints.ErasePoints();
                    
                             for (bi=0 ;bi!=bn-1;bi++){
                                 for (bj=bi+1;bj!=bn;bj++){
                                   if (adjmatrix.Val(bi,bj)==targetmatrix.Val(i,j)){                   
                                       fprintf(statfile," b0(%d,%d) =b0(%d, %d): (%d)", bi, bj, adjmatrix.Val(bi,bi), adjmatrix.Val(bj,bj), adjmatrix.Val(bi,bj));
                                       parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(i,i); parpoint.Z() = adjmatrix.Val(bi, bi); parpoints.push_back(parpoint);
                                       parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(j,j); parpoint.Z() = adjmatrix.Val(bj, bj); parpoints.push_back(parpoint);
                                       match1a = bi;
                                       match1b = bj;
                                       sel_matched_lines.erase(sel_matched_lines.begin(), sel_matched_lines.end());
                                       findline = LineNumber(linenumbermatrix.Val(match1a,match1b));
                                       index_line = sel_map_lines.FindLine(findline);
                                          matchedline = sel_map_lines[index_line];
                                          matchedline.Label() = lastgraphnumber;
                                          matchedline.Label() = keepbuildingmatrix.Val(match1a,match1b) - 10*lastbuildingnumber;
                                          sel_matched_lines.push_back(matchedline);
                                          found_newedge = true;
                                          // remove from adjacency matrix
                                          adjmatrix[bi][bj]=0;
                                          adjmatrix[bj][bi]=0;
                                          hittargetmatrix[i][j]=10;
                                          hitbuildingmatrix[bi][bj]=10;
                                          if (!firsttargetnode) numfirstlevelhits++,hit_firstlevel=true;
                                           
                                      }
                                   }
                              }
                              if (found_newedge){                      
                              for (ii=j+1;ii!=tn;ii++){ //look for other target edges, at same row i
                                  if (ii!=i && targetmatrix.Val(i,ii)!=0) {
                                     hit_secondlevel=false;
                                     scorevector_col[ii]++;
                                     fprintf(statfile," t1(%d,%d): (%d)", i, ii, targetmatrix.Val(i,ii)- 10*lastgraphnumber);
                                     if (i>ii) hittargetmatrix[ii][i]=1;                                         
                                     else hittargetmatrix[i][ii]=1;
                                         found = false;
                                         for (bi=0; bi!=bn; bi++){  
                                         if (!found && bi != kkj && adjmatrix.Val(kki,bi)-10*lastbuildingnumber==targetmatrix.Val(i,ii)- 10*lastgraphnumber){
                                             fprintf(statfile," b1(%d,%d): (segmn %d, nodenr %d): (%d)", kki, bi, adjmatrix.Val(bi,bi),targetmatrix.Val(ii,ii), adjmatrix.Val(kki,bi)-10*lastbuildingnumber);
                                               if (i>ii) hittargetmatrix[ii][i]++;                                          
                                               else hittargetmatrix[i][ii]++;
                                               if (kki>bi) hitbuildingmatrix[bi][kki]++;                                          
                                               else hitbuildingmatrix[kki][bi]++;
                                               hit_secondlevel=true;
                                               score_bcol[bi]=ii;
                                             }
                                          }
                                          
                                     if (!hit_secondlevel) missed_secondlevel++;
                                     else numsecondlevelhits++;
                                     }
                                  }
                                 for (ii=i+1;ii!=tn;ii++){ //look for other target edges, at same column j
                                 if (ii!=j && targetmatrix.Val(ii,j)!=0) {
                                    hit_secondlevel=false;
                                    scorevector_row[ii]++;
                                    fprintf(statfile," t2(%d,%d): (%d)", ii, j, targetmatrix.Val(ii,j)- 10*lastgraphnumber);
                                    if (j>ii) hittargetmatrix[ii][j]=1;                                          
                                    else hittargetmatrix[j][ii]=1;
                                    found = false;
                                    for (bi=0; bi!=bn && !found; bi++){
                                        if (!found && bi != kki && adjmatrix.Val(bi,kkj)-10*lastbuildingnumber==targetmatrix.Val(ii,j)- 10*lastgraphnumber){
                                          fprintf(statfile," b2(%d,%d) = (segmn %d, nodenr %d): (%d)", bi, kkj, adjmatrix.Val(bi,bi), targetmatrix.Val(ii,ii), adjmatrix.Val(bi,kkj)-10*lastbuildingnumber);
                                               if (j>ii) hittargetmatrix[ii][j]++;                                          
                                               else hittargetmatrix[j][ii]++;
                                               if (kkj>bi) hitbuildingmatrix[bi][kkj]++;                                          
                                               else hitbuildingmatrix[kkj][bi]++;
                                               score_brow[bi]=ii;
                                               hit_secondlevel=true;
                                        }
                                     }
                                        
                                     if (!hit_secondlevel) missed_secondlevel++;
                                     else numsecondlevelhits++;
                                   }
                              }
                              keepkhi=400;
                              keepkbi=500;
                              for (khi=0; khi!=tn;khi++){
                                 if (scorevector_row[khi]!=0){
                                   for (khj=0; khj!=tn;khj++){
                                     if (scorevector_col[khj]!=0){ 
                                        if (khi!=khj){
                                           if (targetmatrix.Val(khi,khj)!=0){     //if two neighbouring targets share a node, print label, chain now makes circle     
                                             hit_thirdlevel=false;
                                             pot_penalty=false;                    
                                             hittargetmatrix[khi][khj]=targetmatrix.Val(khi,khj)-10*lastgraphnumber;
                                             fprintf(statfile,"\nshould find 4 chain circle on (%d, %d) = (%d, %d)\n", khi, khj, targetmatrix.Val(khi,khi), targetmatrix.Val(khj,khj));
                                             for (kbi=0; kbi!=bn;kbi++){
                                               if (score_brow[kbi]==khi){
                                                 for (kbj=0; kbj!=bn;kbj++){
                                                  if (score_bcol[kbj]==khj){
                                                    if (kbj!=kbi){                        
                                                      if (adjmatrix.Val(kbi,kbj)-10*lastbuildingnumber == targetmatrix.Val(khi,khj)-10*lastgraphnumber){
                                                        fprintf(statfile,"found match: on (%d, %d) = (%d, %d)\n", kbi, kbj, adjmatrix.Val(kbi,kbi), adjmatrix.Val(kbj,kbj));
                                                        hit_thirdlevel=true;
                                                        hitbuildingmatrix[kbi][kbj]=66;
                                                        match4a = kbi;
                                                        match4b = kbj;
                                                        findline = LineNumber(linenumbermatrix.Val(match4a,match4b));
                                                        index_line = sel_map_lines.FindLine(findline);
                                                        matchedline = sel_map_lines[index_line];
                                                        matchedline.Label() = lastgraphnumber;
                                                        matchedline.Label() = keepbuildingmatrix.Val(match4a,match4b) - 10*lastbuildingnumber;
                                                        sel_matched_lines.push_back(matchedline);
                                                        findline = LineNumber(linenumbermatrix.Val(match4a,match1b));
                                                        index_line = sel_map_lines.FindLine(findline);
                                                        matchedline = sel_map_lines[index_line];
                                                        matchedline.Label() = lastgraphnumber;
                                                        matchedline.Label() = keepbuildingmatrix.Val(match4a,match1b) - 10*lastbuildingnumber;
                                                        sel_matched_lines.push_back(matchedline);
                                                        findline = LineNumber(linenumbermatrix.Val(match4b,match1a));
                                                        index_line = sel_map_lines.FindLine(findline);
                                                        matchedline = sel_map_lines[index_line];
                                                        matchedline.Label() = lastgraphnumber;
                                                        matchedline.Label() = keepbuildingmatrix.Val(match4b,match1a) - 10*lastbuildingnumber;
                                                        sel_matched_lines.push_back(matchedline);
                                                        pot_penalty=true;
                                                        parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(khi,khi); parpoint.Z() = adjmatrix.Val(kbi,kbi); parpoints.push_back(parpoint); 
                                                        parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(khj,khj); parpoint.Z() = adjmatrix.Val(kbj,kbj); parpoints.push_back(parpoint); 
                                                        if ((adjmatrix.Val(kki,kbj)==0 && adjmatrix.Val(kbi,kkj)==0) || (adjmatrix.Val(kki,kbi)==0 && adjmatrix.Val(kbj,kkj)==0)){
                                                           pot_penalty=false;
                                                        }
                                                      }
                                                      }
                                                  }
                                                }
                                               }
                                             }
                                             if (!hit_thirdlevel) {
                                                                     missed_thirdlevel++;
                                                                     fprintf(statfile,"\ndid not find hit between (%d, %d) (nodenrs %d, %d)\n", khi, khj, targetmatrix.Val(khi,khi), targetmatrix.Val(khj,khj));
                                                                     }
                                             else  numthirdlevelhits++;                          
                                           }
                                           else{ // if they do not share a node
                 //                            fprintf(statfile,"\ntargets do not share a node\n");
                                             penalty_match = false;
                                             fprintf(statfile,"\nshould find 2 segments on (%d & %d), targetnodenumber %d & %d\n", khi, khj, targetmatrix.Val(khi,khi), targetmatrix.Val(khj,khj));
                                             for (kbi=0; kbi!=bn;kbi++){
                                               if (score_brow[kbi]==khi){
                                                 match3a = kbi;
                                                 fprintf(statfile,"\nfound one of 2 segments, %d, targetnodenumber %d segm %d\n", kbi, targetmatrix.Val(khi,khi), adjmatrix.Val(match3a, match3a));
                                                 parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(khi,khi); parpoint.Z() = adjmatrix.Val(kbi,kbi); parpoints.push_back(parpoint); 
                                                 for (kbj=0; kbj!=bn;kbj++){
                                                  if (score_bcol[kbj]==khj){
                                                      if (kbi!=kbj&& adjmatrix.Val(kbi,kbj)!=0){ //but if graph share a node, give a penalty hit
                                                        fprintf(statfile,"\npenalty match found: on (%d, %d)\n", kbi, kbj);
                                                        penalty_match=true;
                                                        hitbuildingmatrix[kbi][kbj]=55;
                                                      }
                                                  }
                                                 }   
                                                 if (penalty_match) numpenaltymatches++;
                                                 findline = LineNumber(linenumbermatrix.Val(match3a,match1b));
                                                 index_line = sel_map_lines.FindLine(findline);
                                                 matchedline = sel_map_lines[index_line];
                                                 matchedline.Label() = lastgraphnumber;
                                                 matchedline.Label() = keepbuildingmatrix.Val(match3a,match1b) - 10*lastbuildingnumber;
                                                 sel_matched_lines.push_back(matchedline);
                                                 }
                                               }     
                                               for (kbj=0; kbj!=bn;kbj++){
                                                  if (score_bcol[kbj]==khj){
                                                      match3b = kbj;
                                                      fprintf(statfile,"\nfound one of two segments, %d, targetnodenumber %d segm %d\n", kbj, targetmatrix.Val(khj,khj), adjmatrix.Val(match3b, match3b));
                                                      findline = LineNumber(linenumbermatrix.Val(match3b,match1a));
                                                      index_line = sel_map_lines.FindLine(findline);
                                                      matchedline = sel_map_lines[index_line];
                                                      matchedline.Label() = lastgraphnumber;
                                                      matchedline.Label() = keepbuildingmatrix.Val(match3b,match1a) - 10*lastbuildingnumber;
                                                      sel_matched_lines.push_back(matchedline);
                                                      parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(khj,khj); parpoint.Z() = adjmatrix.Val(kbj,kbj); parpoints.push_back(parpoint); 
                                                  }
                                                }
       
                                             }
                                             }
                                           else { // so, if khi == khj
                                               //if two neighbouring targets share the starting node, print label, chain now makes circle 
                 //                              fprintf(statfile,"\ntargets share starting node\n");
                                               pot_penalty=false;
                                               hit_thirdlevel=false;
                                      //         keepkbi=500; keepkhi =400;
                                               fprintf(statfile,"\nshould find 3 chain circle on (%d, %d), targetnodenumber %d, with labels %d and %d\n", khi, khj, targetmatrix.Val(khi,khi), targetmatrix.Val(i,khi)-10*lastgraphnumber, targetmatrix.Val(j,khi)-10*lastgraphnumber);
                                               if (keepkhi==400) keepkhi=khi;
                                               else {
                                                    if (keepkhi==khi) samekhi = 1;
                                                    else samekhi = 0;
                                                    }
                                                               
                                               for (kbi=0; kbi!=bn;kbi++){
                                                 if (score_brow[kbi]!=0){
                                                   for (kbj=0; kbj!=bn;kbj++){
                                                    if (score_bcol[kbj]!=0){
                                                        if (kbj==kbi){                      
                                                        fprintf(statfile,"found match: on (%d, %d) = segmn %d \n", kbi, kbj, adjmatrix.Val(kbi,kbi));
                                                        parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(khi,khi); parpoint.Z() = adjmatrix.Val(kbi,kbi); parpoints.push_back(parpoint);
                                                        hit_thirdlevel=true;
                                                        two_loops = false;
                                                        if (keepkbi==500) {
                                                             keepkbi=kbi; //first iteration do nothing
                                                             match3a = kbi;
                                                             }
                                                        else {
                                                             two_loops = true;
                                                             if (keepkbi==kbi) samekbi = 1;
                                                             else {
                                                                  samekbi = 0;
                                                                  match3b = kbi;
                                                                  }
                                                             if (samekhi==samekbi){
                                                               fprintf(statfile,"same structure as graph: %d\n", samekbi);                                                              
                                                               hit_thirdlevel=true;
                                                               }
                                                             else {
                                                               fprintf(statfile,"other structure than graph: %d vs %d\n", samekhi, samekbi);                                                              
                                                               hit_thirdlevel=false;
                                                             }
                                                        }
                                                        hitbuildingmatrix[kbi][kbj]=67;
                                                        }
                                                    }
                                                   }
                                                 }
                                                }
                                               if (!hit_thirdlevel) {
                                                                     missed_thirdlevel++;
                                                                     fprintf(statfile,"\ndid not find hit on (%d, %d), targetnodenumber %d \n", khi, khj, targetmatrix.Val(khi,khi));
                                                                     parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(khi,khi); parpoint.Z() = -1; parpoints.push_back(parpoint);
                                                                     }
                                                else  {
                                                      numthirdlevelhits++;
                                                      if (!two_loops){ //if only one 3chain loop in target && found in building
                                                      fprintf(statfile,"\ntwo loops, numthirdlevelhits %d", numthirdlevelhits);
                                                          findline = LineNumber(linenumbermatrix.Val(match3a,match1a));
                                                          index_line = sel_map_lines.FindLine(findline);
                                                          matchedline = sel_map_lines[index_line];
                                                          matchedline.Label() = lastgraphnumber;
                                                          matchedline.Label() = keepbuildingmatrix.Val(match3a,match1a) - 10*lastbuildingnumber;
                                                          sel_matched_lines.push_back(matchedline);
                                                          findline = LineNumber(linenumbermatrix.Val(match3a,match1b));
                                                          index_line = sel_map_lines.FindLine(findline);
                                                          matchedline = sel_map_lines[index_line];
                                                          matchedline.Label() = lastgraphnumber;
                                                          matchedline.Label() = keepbuildingmatrix.Val(match3a,match1b) - 10*lastbuildingnumber;
                                                          sel_matched_lines.push_back(matchedline);
                                                          parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(khi,khi); parpoint.Z() = adjmatrix.Val(match3a, match3a); parpoints.push_back(parpoint);

                                                          }
                                                          else {
                                                                fprintf(statfile,"\ntwo loops, numthirdlevelhits %d 1a=%d, 1b=%d 3a=%d 3b=%d", numthirdlevelhits, match1a, match1b, match3a, match3b);
                                                               findline = LineNumber(linenumbermatrix.Val(match1a,match3a));
                                                               index_line = sel_map_lines.FindLine(findline);
                                                                fprintf(statfile, "start finding1...%d, %d\n", linenumbermatrix.Val(match3a,match1a), index_line);
                                                               matchedline = sel_map_lines[index_line];
                                                               matchedline.Label() = lastgraphnumber;
                                                               fprintf(statfile, "start finding1.5...\n");
                                                               matchedline.Label() = keepbuildingmatrix.Val(match3a,match1a) - 10*lastbuildingnumber;
                                                               sel_matched_lines.push_back(matchedline);
                                                               fprintf(statfile, "start finding2...\n");
                                                               findline = LineNumber(linenumbermatrix.Val(match3a,match1b));
                                                               index_line = sel_map_lines.FindLine(findline);
                                                               matchedline = sel_map_lines[index_line];
                                                               matchedline.Label() = lastgraphnumber;
                                                               matchedline.Label() = keepbuildingmatrix.Val(match3a,match1b) - 10*lastbuildingnumber;
                                                                       fprintf(statfile, "start finding3...\n");
                                                        sel_matched_lines.push_back(matchedline);                                                               
                                                               findline = LineNumber(linenumbermatrix.Val(match3b,match1a));
                                                               index_line = sel_map_lines.FindLine(findline);
                                                               matchedline = sel_map_lines[index_line];
                                                               matchedline.Label() = lastgraphnumber;
                                                               matchedline.Label() = keepbuildingmatrix.Val(match3b,match1a) - 10*lastbuildingnumber;
                                                                fprintf(statfile, "start finding4...\n");
                                                               sel_matched_lines.push_back(matchedline);
                                                               findline = LineNumber(linenumbermatrix.Val(match3b,match1b));
                                                               index_line = sel_map_lines.FindLine(findline);
                                                               matchedline = sel_map_lines[index_line];
                                                               matchedline.Label() = lastgraphnumber;
                                                               matchedline.Label() = keepbuildingmatrix.Val(match3b,match1b) - 10*lastbuildingnumber;
                                                                fprintf(statfile, "start finding5...\n");
                                                               sel_matched_lines.push_back(matchedline);
                                                               parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(khi,khi); parpoint.Z() = adjmatrix.Val(match3a, match3a); parpoints.push_back(parpoint);
                                                               parpoint.X() = lastgraphnumber; parpoint.Y() = targetmatrix.Val(khi,khi); parpoint.Z() = adjmatrix.Val(match3b, match3b); parpoints.push_back(parpoint);
                                                               fprintf(statfile,"\ntwo points pushed back to parpoints\n");
                                                               }
                                                      }
                                               hittargetmatrix[i][j]=10+targetmatrix.Val(i,j)-10*lastgraphnumber;
                                               }
                                                                               
                                     }
                                     
                                   }
                                 }
                              } 
      //              
                                } // end if found new edge                      
                               if (parpoints.size()>2) {
                                                       fprintf(statfile, "start sorting...\n");
                                  parpoints.SortOnCoordinates();
                                  for (laser_point = parpoints.begin(), next_point = parpoints.begin()+1; laser_point !=parpoints.end()-1; laser_point++, next_point++){
                                       if (next_point->X() == laser_point->X() && next_point->Y() == laser_point->Y() && next_point->Z() == laser_point->Z()){
                                          laser_point->Label(-99);
                                       }
                                   }
                                }
                                parpoints.RemoveTaggedPoints(-99, LabelTag);
                                for (laser_point = parpoints.begin(); laser_point !=parpoints.end(); laser_point++){
                                     fprintf(statfile, "\n%d %d %d \n", int(laser_point->X()), int(laser_point->Y()), int(laser_point->Z()));
                                     }
                                if (pot_penalty) numpenaltymatches++;
                                if (numfirstlevelhits>0){
                                fprintf(statfile,"\nFirstlevelhits: %d\nSecondlevelhits: %d (missed %d)\nThirdlevelhits: %d (missed %d)\n",
                                      numfirstlevelhits, numsecondlevelhits, missed_secondlevel, numthirdlevelhits, missed_thirdlevel);
                                fprintf(statfile,"Penaltymatches: %d\n", numpenaltymatches);
                                keepmatchresults = false;
                                if (missed_secondlevel == 0 && missed_thirdlevel == 0 && numpenaltymatches == 0){
                                   fprintf(statfile,"Complete match on target: %d\n", lastgraphnumber);
                                   fprintf(statfile,"Size of matched lines: %d\n", sel_matched_lines.size()); 
                                   matched_lines.insert(matched_lines.end(), sel_matched_lines.begin(), sel_matched_lines.end());
             //                      if (count_should == 0 && count_found ==0) count_should = 1, count_found = 1;
                                   parpoints.SetAttribute(PolygonNumberTag, lastbuildingnumber);
                                   parpoints.SetAttribute(SegmentNumberTag, 1); //1=complete match
                                   keepmatchresults = true;
                                   }
                                  if (missed_secondlevel == 1 && missed_thirdlevel == 0 && numpenaltymatches == 0){                        
                                    fprintf(statfile,"Partially match on target: %d\n", lastgraphnumber);
                                    fprintf(statfile,"Size of matched lines: %d\n", sel_matched_lines.size());
                                    parpoints.SetAttribute(PolygonNumberTag, lastbuildingnumber);
                                    parpoints.SetAttribute(SegmentNumberTag, 2); //2= partly match
                                    keepmatchresults = true;
                                   } 
                                   
                                  if (missed_secondlevel == 0 && missed_thirdlevel != 0 && numpenaltymatches == 0){                        
                                   fprintf(statfile,"Partially match on target: %d\n", lastgraphnumber);
                                   fprintf(statfile,"Size of matched lines: %d\n", sel_matched_lines.size());
                                   parpoints.SetAttribute(PolygonNumberTag, lastbuildingnumber);
                                   parpoints.SetAttribute(SegmentNumberTag, 2); //2= partly match
                                   keepmatchresults = true;

                                   } 
                                  if (missed_secondlevel != 0 && missed_thirdlevel != 0 && numpenaltymatches == 0){                        
                                   fprintf(statfile,"Partially match on target: %d\n", lastgraphnumber);
                                   fprintf(statfile,"Size of matched lines: %d\n", sel_matched_lines.size());
                                   keepmatchresults = false;
                                   }
                                 
                                }
                                else {
                                     if (!firsttargetnode) fprintf(statfile,"\nNo first level match found\n");
                                     else fprintf(statfile,"\nNo further match found\n");
                                     keepmatchresults = false;
                                     }
                                numfirstlevelhits=0;
                                if (keepmatchresults){
                                   keepparpoints.AddPoints(parpoints);
                                   parpoints.ErasePoints();
                                   for (matline = sel_matched_lines.begin(); matline!=sel_matched_lines.end(); matline++){
                                         for (node2 = matline->begin(); node2!=matline->end();node2++){              
                                              sel_segm_laser_points.ErasePoints(); 
                                              sel_segm_laser_points.AddTaggedPoints(sel_laser_points, (map_points.PointIterator(*node2))->Number(), SegmentNumberTag);
                                              sel_segm_laser_points.SetAttribute(ScanNumberTag, lastgraphnumber); // just for information, set targetnumber in scannumbertag
                                              outputlaserpoints.AddPoints(sel_segm_laser_points);
                                              // find corresponding intersection lines in "shapelines"...
                                              linenr = shape_lines.FindLine(matline->Number());
                                              sel_shape_line.Initialise();
                                              sel_shape_line = shape_lines[linenr];
                                              sel_shape_line.Number() = line_number4;
                                              sel_shape_line.Attribute(LineLabelTag)=map_points.PointIterator(*node2)->Number(); // set label to segment number
                                              sel_shape_line.Attribute(TargetNumberTag)=lastgraphnumber; // set targetnumbertag
                                              sel_shape_lines.push_back(sel_shape_line);
                                              line_number4++;
                                         }
                                         
                                       }
                                   }
                                }while (found_newedge && !firsttargetnode);    
                      //           firsttargetnode=true; //after the first pass here, ftn = true    
                             }
                         }
                         }
                   */  
        
                     // go on to the next target graph
//                     sel_graph_lines.erase(sel_graph_lines.begin(), sel_graph_lines.end());
//                     sel_graph_lines.push_back(*map_lineg);
//                     lastgraphnumber = graphnumber;
//                     }
                  }
               }
               
//               }
               
//               sel_map_lines.erase(sel_map_lines.begin(), sel_map_lines.end());
//               sel_map_lines.push_back(*map_line);
//               lastbuildingnumber = buildingnumber;
 //          }
           

         }
         
    }


      fclose(statfile);
//    sel_matched_lines.Write("selshapelines.top", false);
//    shape_points.Write("selshapelines.objpts");
      // here we double the number of shape lines, every line is stored with each of the two segmentnumber
    line_number4=0;
    sel_shape_lines.erase(sel_shape_lines.begin(),sel_shape_lines.end());
    matchedlinenumbers = fix_lines.AttributedValues(LineNumberTag);
    for (matchedlinenumber = matchedlinenumbers.begin(); matchedlinenumber!=matchedlinenumbers.end(); matchedlinenumber++){
        
        linenr = map_lines.FindLine(*matchedlinenumber);
         fprintf(allhypoinfo,"linenr %d ", linenr);
        matchedline = map_lines[linenr];
        for (node2 = matchedline.begin(); node2!=matchedline.end();node2++){              
           // find corresponding intersection lines in "shapelines"...
           sel_shape_line.clear();
      //     sel_shape_line.Initialise();
           sel_shape_line = shape_lines[linenr];
           sel_shape_line.Number() = line_number4;
           fprintf(allhypoinfo,"linenr4 %d ", line_number4);

       //    sel_shape_line.Attribute(LineLabelTag)=map_points.PointIterator(*node2)->Number(); // set label to segment number
           sel_shape_line.Attribute(SegmentLabel)=map_points.PointIterator(*node2)->Number(); // set label to segment number
           fprintf(allhypoinfo,"segm %d ", map_points.PointIterator(*node2)->Number());

           sel_shape_line.Attribute(LineNumberTag)=linenr;
           if (sel_shape_line.Attribute(TargetNumberTag)!=22 && sel_shape_line.Attribute(TargetNumberTag)!=13){
             sel_shape_lines.push_back(sel_shape_line);
             line_number4++;
              fprintf(allhypoinfo,"\n");
             }
        }
    }
    sel_shape_lines.Write("selshapelines.top", false);
    shape_points.Write("selshapelines.objpts");
    

    for (map_line = shape_lines.begin(); map_line!= shape_lines.end(); map_line++){
        sel_shape_lines = fix_lines.SelectAttributedLines(LineNumberTag, map_line->Number());
        if (sel_shape_lines.size()==0) notmatchedlines.push_back(*map_line);
        sel_shape_lines2 = sel_shape_lines.SelectAttributedLines(MatchResultTag, 0);
        if (sel_shape_lines2.size()==0) notmatchedcompllines.push_back(*map_line);
        
    }
        notmatchedlines.Write("notmatchedlines.top", false);
        notmatchedcompllines.Write("notmatchedcompllines.top", false);
        
    
LineTopologies bestmatches2, bestmatches3;
LineTopology   bestmatch;
LaserPoints    points_on_majorshape;
int            highestcov, iter;
double         totalcov;
bool selectmajorshape = true;
if (selectmajorshape){
    vbuildingnumbers = segm_laser_points.AttributeValues(PolygonNumberTag); //map_lines.AttributedValues(BuildingNumberTag);
//    polygon_numbersall = segm_laser_points.AttributeValues(PolygonNumberTag); 
//    for (polygon_number = polygon_numbersall.begin(); polygon_number!=polygon_numbersall.end(); polygon_number++){
         for (vbuildingnumber = vbuildingnumbers.begin(); vbuildingnumber!=vbuildingnumbers.end(); vbuildingnumber++){
             
         selhypolines = hypolines.SelectAttributedLines(BuildingNumberTag, *vbuildingnumber);
         fprintf(allhypoinfo,"For building %d we found %d hypo's\n", *vbuildingnumber, selhypolines.size());
         sel_laser_points.ErasePoints();
         sel_laser_points.AddTaggedPoints(segm_laser_points, *vbuildingnumber, PolygonNumberTag);
         if (selhypolines.size()!= 0){
         bestmatches2 = selhypolines.SelectAttributedLines(MatchResultTag, 0);
         fprintf(allhypoinfo,"Of which %d have score 0\n", bestmatches2.size());
         totalcov = 0;
         iter = 0;
         sel3_points.ErasePoints();
         fprintf(allhypoinfo,"size of sel points %d\n", sel_laser_points.size());
         do {
             highestcov = 0;
             for (matline = bestmatches2.begin(); matline!=bestmatches2.end();matline++){
    //            if (matline->Attribute(TargetNumberTag)!=13 && matline->Attribute(TargetNumberTag)!=22 ) {//&& matline->Attribute(TargetNumberTag)!=21 
                if (matline->Attribute(CoverageTag)>highestcov && !bestmatches3.Contains(*matline)) {
                   highestcov = matline->Attribute(CoverageTag);
                    bestmatch = *matline;
                 }
    //             }
              }
              for (node = bestmatch.begin(); node!=bestmatch.end();node++){
                 fprintf(allhypoinfo,"looking for segment: %d\n", node->Number());
                 sel4_points.ErasePoints();
                 sel4_points.AddTaggedPoints(sel_laser_points, node->Number(), SegmentNumberTag);
                 if (!sel3_points.Contains(sel4_points[0])){
                   sel3_points.AddPoints(sel4_points);
                 }
              }
      //        sel3_points.ReduceData(0.01);
              totalcov = 100.0*sel3_points.size()/sel_laser_points.size();
              bestmatches3.push_back(bestmatch);
              fprintf(allhypoinfo,"size of sel3 points %d\n", sel3_points.size());
              iter++;
              points_on_majorshape.AddPoints(sel3_points);
//              } while (totalcov<75 && iter<10);//look for more best matches until at least 75% of laser points are used, or 10 max
//             } while (iter<1);   //select only the best one 
  //            } while (totalcov<75 && iter<3);//look for more best matches until at least 75% of laser points are used, or 10 max         
               } while (iter<bestmatches2.size());//go on untill all complete matches are done...
         }
         else { //if no hypolines...
            sel_laser_points.ErasePoints();
            sel_laser_points = segm_laser_points.SelectTagValue(PolygonNumberTag, *vbuildingnumber);
            if (sel_laser_points.size()==0) fprintf(allhypoinfo,"no laser points\n", bestmatches2.size());
            else { //do something with these points...
                 }         
         }
    }  
 }
//bestmatches = bestmatches3.SelectAttributedLines(TargetNumberTag, 10);
//bestmatches2 = bestmatches3.SelectAttributedLines(TargetNumberTag, 14);
//bestmatches.insert(bestmatches.end(), bestmatches2.begin(), bestmatches2.end());
bestmatches3.Write("bestmatchesperbuilding.top", false);
points_on_majorshape.Write("bestpointsperbuilding.laser", false);         
   fclose(allhypoinfo);
   //bestmatches = hypolines.SelectAttributedLines(MatchResultTag, 25);

   hypolines.Write("hypolines.top", false);
   bestmatches = fix_lines.SelectAttributedLines(TargetNumberTag, 22);
   bestmatches2 = bestmatches.SelectAttributedLines(MatchResultTag, 0);
   bestmatches2.Write("target22.top", false);
   bestmatches = fix_lines.SelectAttributedLines(MatchResultTag, 0);
   bestmatches.Write("matchedshapelines.top", false);
   wasnodewordtsegment.Write("wasnodewordtsegment.laser", false);
      return;
      
}                              
/*   
   //GBKN_points.Read("middelburgGBKN_sub.objpts");
   //GBKN_lines.Read("middelburgGBKN_sub.top");
   Plane                        plane;
   ObjectPoints                 shape_points_pertarget;
   allhypoinfo = fopen("allhypoinfo.asc", "w");
   for (laser_point = keepparpoints.begin(); laser_point !=keepparpoints.end(); laser_point++){
         fprintf(allhypoinfo, " %d %d %d %d %d\n", laser_point->Attribute(PolygonNumberTag), int(laser_point->X()), int(laser_point->Y()), int(laser_point->Z()), laser_point->Attribute(SegmentNumberTag));
         }
           
   fclose(allhypoinfo);
    map_points.Write("matchedgraphs.objpts");
    matched_lines.Write("matchedgraphs.top", false);
    
    //get some statistics from matchresults
    LaserPoints stat_points, missed_points;
    stat_points.AddTaggedPoints(keepparpoints, 2, SegmentNumberTag);
    vector <int>                 segment_numbers;
    vector <int>::iterator       segment_number;
    outputlaserpoints.ReduceData(0.01); 
   for (laser_point = stat_points.begin(); laser_point !=stat_points.end(); laser_point++){
        laser_point->Attribute(SegmentNumberTag) = int(laser_point->X());
        if (laser_point->Attribute(SegmentNumberTag)==17){
            missed_points.AddTaggedPoints(outputlaserpoints.SelectTagValue(ScanNumberTag, 17), laser_point->Attribute(PolygonNumberTag), PolygonNumberTag);
        }
   }
   missed_points.Write("missedpoints.laser", false);
   segment_numbers = stat_points.AttributeValues(SegmentNumberTag);
   for (segment_number=segment_numbers.begin();
             segment_number!=segment_numbers.end(); segment_number++) {
             fprintf(statfile, "Missed on target %d: %d\n", *segment_number, (stat_points.SelectTagValue(SegmentNumberTag, *segment_number)).size());
        }
    
  return;
    for (fix_line= shape_lines.begin(); fix_line!=shape_lines.end(); fix_line++){
        if (!sel_shape_lines.Contains(*fix_line)) {
                                                  shape_lines.erase(fix_line);
                                                  fix_line--;
                                                  }
    }
    shape_points_pertarget = shape_points;
    shape_points.RemoveDoublePoints(shape_lines, 0.05);
    shape_lines.Write("selshapelines.top", false);
    shape_points.Write("selshapelines.objpts");
    int diff, size1, size2, number_offset;
    diff = 0;
    

   
    return;
        polygon_numbers = outputlaserpoints.AttributeValues(PolygonNumberTag); // all matched laser points
//        polygon_numbersall = segm_laser_points.AttributeValues(PolygonNumberTag); // all segmented laser points
        if(!polygons.empty()) polygons.erase(polygons.begin(), polygons.end());
        for (polygon_number=polygon_numbers.begin();
             polygon_number!=polygon_numbers.end(); polygon_number++) {
             polygons.push_back(outputlaserpoints.TaggedPointNumberList(PolygonNumberTag, *polygon_number));
            }
  //          printf ("begin stepedge, size of segments: %d\n", segments.size());  
       for (polygon=polygons.begin(), polygon_number=polygon_numbers.begin();
            polygon!=polygons.end(); polygon++, polygon_number++) {
            if (enclosing){                         
            sel2_points.ErasePoints();
            OutliningParameters *opar=new OutliningParameters();
            SegmentationParameters *spar=new SegmentationParameters();
            opar->MaximumDistancePointIntersectionLine()=0.8;
            opar->MinimumAnglePreferredDirections()= 10;
            opar->HoughBinSizeDistance()= 0.1;
            opar->HoughBinSizeDirection() = 3;
            opar->MaximumDistancePointOutline()=0.5;
            opar->MaximumPointGapInOutlineSegment()=2;
            opar->MaximumGapSizeInOutlineSegment()=1;
            opar->MinimumNumberOfPointsInOutlineSegment()=5; 
            out3dobj.erase(out3dobj.begin(), out3dobj.end());
            out3dtop.erase(out3dtop.begin(), out3dtop.end());
            sel2_points.AddTaggedPoints(outputlaserpoints, *polygon_number, PolygonNumberTag);
            sel2_points.DeriveTIN();
            sel2_points.DeriveDataBounds(0);
            if (sel2_points.EnclosingPolygon(out3dobj,out3dtop, *spar,*opar,true)){
                if (!out3dtop.IsClosed()) out3dtop.push_back(*(out3dtop.begin()));
                out3dtops.erase(out3dtops.begin(), out3dtops.end());
                out3dtop.Label() = 3;
                out3dtops.push_back(out3dtop);
                next_pnr = allout3dobj.HighestPointNumber().Number()+1;
                line_number3 = line_number3 + out3dtops.size();
                out3dtops.ReNumber(out3dobj, next_pnr, line_number3);
                allout3dtop.insert(allout3dtop.end(), out3dtops.begin(), out3dtops.end());
                allout3dobj.insert(allout3dobj.end(), out3dobj.begin(), out3dobj.end());
            }
            } //end if enclosing
            if (contour3d){
     //          index_line = GBKN_lines.FindLine(LineNumber(*polygon_number));
      //         GBKN_line = GBKN_lines[index_line];
               sel2_points.ErasePoints();
               sel2_points.AddTaggedPoints(outputlaserpoints, *polygon_number, PolygonNumberTag);
               segment_numbers = sel2_points.AttributeValues(SegmentNumberTag);
               sel3_points.ErasePoints();
               sel3_points.AddTaggedPoints(segm_laser_points, *polygon_number, PolygonNumberTag);
               fprintf(statfile,"\nPolygon number %d, Percentage = %4.2f\n", *polygon_number, 100.0*sel2_points.size()/sel3_points.size());
               if (sel2_points.size() == sel3_points.size()){
                                      fprintf(statfile,"All points used in complete matches\n");
                                      outputlaserpoints.RemoveTaggedPoints(*polygon_number, PolygonNumberTag);
                                      sel3_points.Label(4);
                                      outputlaserpoints.AddPoints(sel3_points);
                                      } 
               if(!segments.empty()) segments.erase(segments.begin(), segments.end());
               // Determine the points of all segments
               for (segment_number=segment_numbers.begin();
                    segment_number!=segment_numbers.end(); segment_number++) {
                    segments.push_back(sel2_points.TaggedPointNumberList(SegmentNumberTag, *segment_number));
                }  
                for (segment=segments.begin(), segment_number=segment_numbers.begin();
                     segment!=segments.end(); segment++, segment_number++) {
                     seg_laser_points.ErasePoints();
                     fix_laser_points.ErasePoints();
                    // seg_laser_points = sel2_points.SelectTagValue(SegmentNumberTag, *segment_number);
              //       seg_laser_points.AddTaggedPoints(sel2_points, *segment_number, SegmentNumberTag);
                     fix_lines.erase(fix_lines.begin(), fix_lines.end());
                     fix_points.erase(fix_points.begin(), fix_points.end());
                     temp_points.erase(temp_points.begin(), temp_points.end());
                     for (fix_line= sel_shape_lines.begin(); fix_line!=sel_shape_lines.end(); fix_line++){
                          if (fix_line->Attribute(0) == *segment_number){
                            for (node2 = fix_line->begin(); node2!=fix_line->end();node2++){
                              fix_points.push_back(*(shape_points.PointIterator(*node2)));
                              fix_laser_point.X() = (shape_points.PointIterator(*node2))->X(); 
                              fix_laser_point.Y() = (shape_points.PointIterator(*node2))->Y(); 
                              fix_laser_point.Z() = (shape_points.PointIterator(*node2))->Z(); 
                              fix_laser_point.SetAttribute(SegmentNumberTag, *segment_number);
           //                   fix_laser_points.push_back(fix_laser_point);
                            }
                            fix_lines.push_back(*fix_line);
                           }
                     }
                //     sel2_points.AddPoints(fix_laser_points);
                     seg_laser_points = sel2_points.SelectTagValue(SegmentNumberTag, *segment_number);
                   //  fix_laser_points.AddTaggedPoints(sel2_points, *segment_number, SegmentNumberTag);
                //     seg_laser_points.DeriveTIN();
               //      seg_laser_points.DeriveDataBounds(0);
//                     fix_laser_points.DeriveContour3D(out3dobj, out3dtop, 1.5);
                     fix_laser_points = seg_laser_points;
                     fix_laser_points.DeriveTIN();
                     component.Erase();
                     edges.Erase();
                     if(!out3dobj.empty()) out3dobj.erase(out3dobj.begin(), out3dobj.end());
                     edges.Derive(fix_laser_points.TINReference());
                     fix_laser_points.RemoveLongEdges(edges, 3, true);
                     for (int i=0; i<fix_laser_points.size(); i++) component.push_back(PointNumber(i));
                     //            contour = seg_laser_points.DeriveContour(1, component, edges, false, LabelTag);
                     out3dtop = fix_laser_points.DeriveContour(1, component, 1, 1, edges);
                     temp_points = fix_laser_points.ConstructObjectPoints();
                     for (node2 = out3dtop.begin(); node2!=out3dtop.end();node2++){
                         out3dobj.push_back(*(temp_points.PointIterator(*node2)));
                     } 
                     if (!out3dtop.IsClosed()) out3dtop.push_back(*(out3dtop.begin()));
             //        printf("size of out3dtop = %d\n", out3dtop.size());
                     out3dtops.erase(out3dtops.begin(), out3dtops.end());
                     out3dtop.Label() = seg_laser_points[0].Label();
                     out3dtops.push_back(out3dtop);
   //                  out3dobj.Write("temp.objpts");
   //                  out3dtops.Write("temp.top", false);
              //         printf("size of fixlaserpoints: %d\n", fix_laser_points.size());
              //       out3dtopsmooth = fix_laser_points.SmoothOutline(out3dtop, 2, 0.5);
//                     out3dtop = out3dtopsmooth;
    //                 printf("size of smoothed out3dtop = %d\n", out3dtop.size());
   //                  
                     
                     out3dtops.erase(out3dtops.begin(), out3dtops.end());
                    
                     out3dtop.Label() = seg_laser_points[0].Label();
                     out3dtops.push_back(out3dtop);
              /*       temp_points = fix_points;
                      next_pnr = temp_points.HighestPointNumber().Number()+1;
                      out3dtops.ReNumber(out3dobj, next_pnr, 1);
                      out3dtop = out3dtops[0];
                      temp_points.insert(temp_points.end(), out3dobj.begin(), out3dobj.end());
                  //    out3dtop.InsertNodes(temp_points, 1.0);
                */      
  /*                   plane = fix_laser_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
                     if (out3dtop.Label() == 22){ // example possibility to adapt Snap function to graph number...
            //             out3dobj.SnapPoints(out3dtops, fix_lines, fix_points, plane, label_pair_step_edge, false, 0.2);
                         }
                     else {
                          out3dobj.SnapPoints(out3dtops, fix_lines, fix_points, plane, label_pair_step_edge, false, 1.0);
                          }
         //             out3dobj.SnapSegmentToMap(out3dtops, GBKN_line, GBKN_points, plane, 1.0);
                    
                     
                     next_pnr = allout3dobj.HighestPointNumber().Number()+1;
                     line_number3 = line_number3 + out3dtops.size() + diff;
                     out3dtops.ReNumber(out3dobj, next_pnr, line_number3);
   /*                  number_offset = out3dobj.HighestPointNumber().Number()+1;
                     out3dobj.DuplicateWithFixedZ(0, number_offset);
                     size1=allout3dtop.size();
                     allout3dtop.AddWalls(out3dtops, number_offset);
                     size2=allout3dtop.size();
                     diff = size2-size1;
     *//*                allout3dtop.insert(allout3dtop.end(), out3dtops.begin(), out3dtops.end());
                     allout3dobj.insert(allout3dobj.end(), out3dobj.begin(), out3dobj.end());
                }
             }
       }

    allout3dobj.Write(map_points_output);
    allout3dtop.Write(map_topology_output, false);
    
     fclose(statfile);
    outputlaserpoints.Write("outputmatchinglaserpoints.laser", false);
    for (fix_line= sel_shape_lines.begin(); fix_line!=sel_shape_lines.end(); fix_line++){
        fix_line->Label() = fix_line->Attribute(TargetNumberTag);//if (!sel_shape_lines.Contains(*fix_line)) {
    }
    shape_points_pertarget.RemoveDoublePoints(sel_shape_lines,0.05);

    sel_shape_lines.Write("shapelines_pertarget.top", false);
    shape_points_pertarget.Write("shapelines_pertarget.objpts");
 //   allout3dobj.SnapPoints(allout3dtop, shape_lines, shape_points, 2.0);
 //   allout3dobj.Write(map_points_output);
 //   allout3dtop.Write(map_topology_output, false);
    
    
 // return;
}// end method1

  
}

// old stuff
/*scorevector.erase(scorevector.begin(), scorevector.end());
                     scorevector.resize(tv.size(),0);
                     score = 0;
                     // begin look for matches - method 1   
                     if (method1){     
                     for (i=0; i!=bv.size();i++) {
                       for (j=0; j!=tv.size();j++){
                        
                       
//                         if (bv[i]-1000*lastbuildingnumber == tv[j] - 1000*lastgraphnumber){
                         if (bv[i]-10*lastbuildingnumber == tv[j] - 10*lastgraphnumber){
                            score++;
                            scorevector[j]++;
                            fprintf(statfile,"%d (%d) matches on target %d on %d and %d\n", linenumbergraph[i], bv[i], tv[j], i, j);
                            // now, translate i into ii,jj
                            mm = 0;
                            ii = -1;
                            do{
                                 ii++;
                                 mm = mm + (n-ii-1);
         //                        fprintf(statfile,"ii=%d and mm =%d\n", ii, mm);
                            } while (mm < i+1);
                     //       mm = mm + n -ii +1;
                            jj = -ii*(n-1) + 1 + i + int(0.5*(ii*ii + ii));
                            fprintf(statfile,"ii = %d, jj = %d\n", ii, jj);
                         }
                       }
                     }
                     }// end method 1, start method 2
 
                 
                 
                 tv = sel_graph_lines.TransformToGraph(map_points_graphs);               
               targetmatrix = sel_graph_lines.TransformToGraphMatrix(map_points_graphs);
               fprintf(statfile,"\ntargetmatrix %d, numrows = %d\n", lastgraphnumber, targetmatrix.NumRows());
               adjmatrix = sel_map_lines.TransformToGraphMatrix(map_points); //for every target, start with a new building adj matrix
               tn = int(sqrt(2*tv.size()))+1;
               for (i=0 ;i!=tn-1;i++){
                  for (j=i+1;j!=tn;j++){
                    if (targetmatrix.Val(i,j)!=0) {
                       fprintf(statfile," t4(%d,%d): (%d)", i, j, targetmatrix.Val(i,j)- 10*lastgraphnumber);
                       do {
                       found_newedge = false;
                       if (!keephits.empty()) keephits.erase(keephits.begin(), keephits.end());
                       for (bi=0 ;bi!=bn-1;bi++){
                         for (bj=bi+1;bj!=bn;bj++){
                            if (adjmatrix.Val(bi,bj)-10*lastbuildingnumber==targetmatrix.Val(i,j)- 10*lastgraphnumber){
                                fprintf(statfile," b4(%d,%d): (%d)", bi, bj, adjmatrix.Val(bi,bj)-10*lastbuildingnumber);
                                // put bi and bj in a vector, keep them for further searching
                                keephits.push_back(bi);
                                keephits.push_back(bj);
                                found_newedge = true;
                             }
                           }
                        }
                        if (found_newedge){
          //              fprintf(statfile, "found %2.0f new edges...\n", 0.5*keephits.size());                        
                        for (ii=i+1;ii!=tn;ii++){
                        if (targetmatrix.Val(ii,j)!=0) {
                           fprintf(statfile," t5(%d,%d): (%d)", ii, j, targetmatrix.Val(ii,j)- 10*lastgraphnumber);
                           for (ki=0 ; ki!=keephits.size();ki=ki+2){
                             kki = keephits[ki];
                             kkj = keephits[ki+1];
                             for (bi=0; bi!=bn; bi++){  
                                if (bi != kki && adjmatrix.Val(bi,kkj)-10*lastbuildingnumber==targetmatrix.Val(ii,j)- 10*lastgraphnumber){
                                   fprintf(statfile," b5(%d,%d): (%d)", bi, kkj, adjmatrix.Val(bi,kkj)-10*lastbuildingnumber);
                                   adjmatrix[bi][kkj]=0;
                                   adjmatrix[kkj][bi]=0;
                                }
                             }
                           }
                           targetmatrix[ii][j]=0;
                           targetmatrix[j][ii]=0;
                         }
                         }
                         for (ii=j+1;ii!=tn;ii++){
                            if (targetmatrix.Val(i,ii)!=0) {
                               fprintf(statfile," t6(%d,%d): (%d)", i, ii, targetmatrix.Val(i,ii)- 10*lastgraphnumber);
                               for (ki=0 ; ki!=keephits.size();ki=ki+2){
                                   kki = keephits[ki];
                                   kkj = keephits[ki+1];
                                   for (bi=0; bi!=bn; bi++){  
                                     if (bi != kkj && adjmatrix.Val(kki,bi)-10*lastbuildingnumber==targetmatrix.Val(i,ii)- 10*lastgraphnumber){
                                         fprintf(statfile," b6(%d,%d): (%d)", kki, bj, adjmatrix.Val(kki,bj)-10*lastbuildingnumber);
                                         adjmatrix[bj][kki]=0;
                                         adjmatrix[kki][bj]=0;
                                     }
                                   }
                                }
                             targetmatrix[i][ii]=0;
                             targetmatrix[ii][i]=0;
                             }
                          }
                         } //end found newedge
                         }while (found_newedge);
                        }
                       }
                    }
                 } 
                     
  */                   
