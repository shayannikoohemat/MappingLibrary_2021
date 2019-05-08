
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
 Date   : 23-04-2007

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
                           implementation of quality analysis as described in:
Oude Elberink, S.J. and Vosselman, G. (2007) Quality analysis of 
3D road reconstruction. In: ISPRS 2007 : Vol. XXXVI, Pt. 3 / W52 : 
Proceedings of the ISPRS workshop : Laser scanning 2007 and SilviLaser 2007, 
September 12-14, 2007, Espoo, Finland. International Society for Photogrammetry 
and Remote Sensing (ISPRS), 2007. pp. 305-310.
--------------------------------------------------------------------------------
*/

void calculatequality3dtop(char *map_points_input, char *map_topology_input,
                    char *laser_input, char *laser_points_inside, char *laser_points_outside)

{
  ObjectPoints                 map_points, map_points_ref, sel_map_points,
                               all_tin_points, diff_map_points;
  ObjectPoint                  sel_map_point, next_point;
  LineTopologies               map_lines, map_lines_ref, one_map_line, 
                               map_tin_lines, all_tin_lines;
  ObjectPoints::iterator       map_point;
  LineTopologies::iterator     map_line, last_map_line, map_line2;
  LineTopology::iterator       node2, nb_node, next_node;
  LaserPoints                  ref_laser_points, laserin, laserout, laser_points,
                               sel_laser_points, temp_height_points, diff_points,
                               secdiff_points, dum_laser_points;
  LaserPoint                   ref_laser_point, diff_point;
  LaserPoints::iterator        laser_point;
  PointNumberList              neighbourhood;
  PointNumberList::iterator    node;
  int                          countin, countout, index, success, count,
                               nearest_laser_point, dominant_segment, i;
  TIN                          tin;
  TINEdges                     edges;
  Plane                        plane;
  Position3D                   p1, p2, p3;
  double                       diff, lasercount, diffx, diffy, extrapolation_error,
                               extrapol_variance;
  Matrix3                      qmat;
  Vector3D                     v1, v2, v3, diffv, vr1, vr2, vr3;
  bool found;
  double nbh_radius =10;
  double dist, syst_laser_error;
  // Read input data
  if (!map_points.Read(map_points_input)) {
    printf("Error reading map points from file %s\n", map_points_input);
    exit(0);
  }
  if (!map_lines.Read(map_topology_input)) {
    printf("Error reading map lines from file %s\n", map_topology_input);
    exit(0);
  }
  if (!laser_points.Read(laser_input)) {
    printf("Error reading laser points from file %s\n", laser_input);
    exit(0);
  }

  map_points.RemoveDoublePoints(map_lines, 0.01);

  printf("Deriving TIN ..."); fflush(stdout);
 // laser_points.DeriveTIN();
  printf("done\nDeriving TIN edges ..."); fflush(stdout);
 // edges.Derive(laser_points.TINReference());
  
  
 bool quality = true;
  if (quality){
 // map_lines.Densify(map_points, 10); 
  for (map_line=map_lines.begin(), index=0; map_line!=map_lines.end(); map_line++, index++) {
      if (map_line->TOP10MajorClass() == TOP10_Road){
         if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end());
         if(!map_tin_lines.empty()) map_tin_lines.erase(map_tin_lines.begin(),map_tin_lines.end());                                
         sel_laser_points.ErasePoints();
           sel_laser_points = laser_points.SelectTagValue(PolygonNumberTag, map_line->Number());
           if (sel_laser_points.size()>=3){
           for (node2=map_line->begin(), count=0; node2!=map_line->end(); node2++, count++) {
           sel_map_point = *(map_points.PointIterator(*node2));
           temp_height_points.ErasePoints();
           dum_laser_points.ErasePoints();  
           for (laser_point=sel_laser_points.begin(); laser_point!=sel_laser_points.end(); laser_point++){
                dist = (laser_point->vect2D()-sel_map_point.vect2D()).Length();   
                if (dist < nbh_radius) {
                           temp_height_points.push_back(*laser_point);
                    }
           }
           ref_laser_point.X()= sel_map_point.X();
           ref_laser_point.Y()= sel_map_point.Y();
           ref_laser_point.Z()= sel_map_point.Z();
           temp_height_points.Label(sel_map_point.Number());
           dominant_segment = temp_height_points.MostFrequentAttributeValue(SegmentNumberTag, count);
           dum_laser_points = temp_height_points.SelectTagValue(SegmentNumberTag, dominant_segment);
           temp_height_points.ErasePoints();
           temp_height_points = dum_laser_points;
           ref_laser_point.Residual() = 1.0;
           if (temp_height_points.size() >= 3){
         //    plane = temp_height_points.FitPlane(sel_map_point.Number(),sel_map_point.Number(), LabelTag);
             qmat = temp_height_points.QualityPlane(sel_map_point.Number(), 0.08, LabelTag); //here laser point noise
             printf("\nsize: %d\n", temp_height_points.size());
      //     qmat.Print();
             diffx = fabs(sel_map_point.X() - temp_height_points.Mean().X());
             diffy = fabs(sel_map_point.Y() - temp_height_points.Mean().Y());           
             v1[0] = 1; v1[1] = 0; v1[2] = 0; 
             v2[0] = 0; v2[1] = 1; v2[2] = 0;
             v3[0] = 0; v3[1] = 0; v3[2] = 1;
             vr1 = operator*(qmat, v1);
             vr2 = operator*(qmat, v2);
             vr3 = operator*(qmat, v3);
             diffv[0] = diffx*diffx*vr1[0];
             diffv[1] = diffy*diffy*vr2[1];
             diffv[2] = vr3[2];
             printf("diffx = %5.2f, diffy = %5.2f, zprec = %5.2f\n", diffx, diffy, sqrt(diffv[0]+diffv[1]+diffv[2]));
           extrapolation_error = 0.0001*nbh_radius*nbh_radius; // maximum error, now translate this into variance
           extrapol_variance = extrapolation_error*extrapolation_error/9;
           syst_laser_error = (0.03*0.03+0.04*0.04 + 0.03*0.03); //gps, ins and strip errors
           ref_laser_point.Residual() = sqrt(diffv[0]+diffv[1]+diffv[2]+extrapol_variance+syst_laser_error);
           if (ref_laser_point.Residual() >= 2) ref_laser_point.Residual()=2.0; 
           }
           laserout.push_back(ref_laser_point);  
    //       laserin.AddTaggedPoints(temp_height_points, sel_map_point.Number(), LabelTag);  
           ref_laser_point.Z() =  ref_laser_point.Residual();
           ref_laser_point.SetAttribute(PolygonNumberTag, map_line->Number());                                       
           laserin.push_back(ref_laser_point);  
         }
      }
      }
  if (index == (index/5)*5)
      printf("%6d (%5.1f\%)\r", index, 100.0 * index / map_lines.size());
  }
}
 
//  printf("\nIn: %d, Out: %d\n", countin, countout);
//  if (!diff_map_points.Write(laser_points_inside))
  if (!laserin.Write(laser_points_inside, false))
        printf("Error writing reference points INSIDE as laser points.\n");
//  if (!secdiff_points.Write(laser_points_outside, false))
  if (!laserout.Write(laser_points_outside, false))
        printf("Error writing reference points OUTSIDE as laser points.\n");
  return;
}

