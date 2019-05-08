/*
--------------------------------------------------------------------------------

 Initial creation:
 Author : Sander Oude Elberink
 Date   : 18-07-2006

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

void Make3DBuildings(char *laser_input, char *map_points_input,
	       char *map_topology_input, 
           char *map_points_output, char *map_topology_output, 
           char *laser_output, char *laser_output2, char *infofile, int minsizesegment, double minlinelength)
{
  LaserPoints                  laser_points, sel_laser_points, building_laser_points,
                               non_building_laser_points, temp_height_points,
                               calc_points, seg_laser_points, intersection_points,
                               largest_segments_points, keep_sel_laser_points,
                               tilted_laser_points, total_tilted_points,
                               keep_laser_points, tot_largest_segments_points,
                               artpoints, total_mixed_points, mixed_points,
                               residual_points, four_planes_points, seg1_laser_points,
                               seg2_laser_points, dormer_points, all_dormer_points,
                               non_dormer_points, flat_points, stepedgelaserpoints, vertical_points;
  LaserPoint                   begin_intp, end_intp, buildingpoint, artpoint,
                               nearlaspoint, nearlaspoint2, centroidofdormer;
  LaserPoints::iterator        laser_point;
  TINEdges                     edges, edges_3d, edges1, edges2;
  ObjectPoints                 map_points, building_points_3d, sel_map_points, 
                               all_tin_points, all_segment_contour_points, segment_contour_points,
                               intsects, height_points, pairpoints, all_tin_building_points,
                               all_terrain_points, all_tin_building_maxpoints, 
                               max_map_points, shape_points, graph_points, selcornerpoints,
                               selridgepoints, stepedgepoints, selcornerpoints_four,
                               selridgepoints_four, nondormerobj, temp_points,
                               allnondormerobj, fittedstepedgepoints, origintsectpoints;
  LineTopologies               map_lines, sel_map_lines, one_map_line, 
                               map_tin_lines, all_tin_lines, keep_map_lines, segment_contours,
                               one_contour, temp_segment_contours, intsectlines,
                               height_lines, pairlines, temp_pairlines, all_tin_building_lines,
                               all_terrain_tin, all_tin_building_maxlines, max_map_line,
                               max_tin_lines, shapelines, graph_lines, selridgelines,
                               stepedgelines, selridgelines_four, graphlines, nondormertops, allnondormertops,
                               stepedgetopologyoutput, fittedstepedgelines, origintsectlines;
  ObjectPoints::iterator       map_point, previous_point, next_point, this_point;
  ObjectPoint                  new_map_point, new_point, beginp, endp,
                               objpseg1, objpseg2, height_point, max_point,
                               cornerpoint;
  LineTopologies::iterator     map_line, last_map_line, map_line2, segment_contour, segment_contour2,
                               buildoutline, map_line3; 
  LineTopology::iterator       node2;
  LineTopology                 contour, intsect, height_line, pairline, shapeline;
  LineTopology                 buildingline, smooth_contour, nondormertop;
  double                       height, dist, nbh_radius=4.0, max_slope = 0.5, PI, fnl1,fnl2,
                               runx, runy, area, pd, angle, pdist, mean, fixed_floor_height,
                               eave_height, local_eave_height, min_dist, mindir,
                               mindir2, max_height, height_score, gable_score,
                               gambrel_score, max_dist, min_height, loc_max_height, zdiff;   
  int                          i, j, number_offset, number_offset2, count, count2, dominant_segment,
                               success, next_pnr, highest_map_line_number, line_number,
                               countingintsect, line_number2, part, puntnr, 
                               total_size, cuma, quad, total_flat, total_vertical,
                               total_tilted, prev_count, line_number3, overall_flat,
                               overall_tilted, overall_vertical,
                               run_index, biggest_four, sn1, sn2, ds, fs,
                               nearest_laser_point, totedges, totedges2, totedges3,
                               rem_number_offset2, median_colour, new_segment_number,
                               median_colour_red, median_colour_green, median_colour_blue,
                               intdist, k, l, kk, ll, ii, jj, line_number4, edgepointnumber,
                               fi, lab1, lab2, num_tilted, num_horizontal, nearest_laser_point1,
                               nearest_laser_point2, num_shapelines, num_pairlines,
                               line_number6, size1, size2;
  vector <int>                 segment_numbers, tilted_segment_numbers, colour_values,
                               colour_red, colour_green, colour_blue, graph_vector,
                               close_vector;
  vector <int>::iterator       segment_number, segment_number2, close_iter;
  PointNumberLists             segments, pnls;
  PointNumberList              component, pnl, pnlseg1, pnlseg2, tpnl, 
                               neighbourhood1, neighbourhood2, step, highestseg;
  PointNumberList::iterator    item;
  PointNumberLists::iterator   segment, segment2, pnlit, pnlit2;        
  bool                         found, do_enclosingpolygon=false, do_contour=false, do_intersection =false,
                               foundcornerpoint, dominant_dir;         
  TIN                          tin;
  TINEdges::iterator           neighbours;         
  DataBoundsLaser              bounds2d;
  DataBounds3D                 polybounds;
  Planes                       planes, cornerplanes;
  Plane                        planecorner1, planecorner2, planecorner3, plane, planev, planeh, planel;
  Line3D                       linecorner, il2, il3, il4;
  Line2D                       il1;
  Planes::iterator             plane1, plane2;
  Position3D                   pos1, pos2, posseg1, posseg2, posmin, posmax,
                               runpos, laspos, newpos, pos3, pos4, pos5, midpos;
  Position2D                   centroid;
  PointNumber                  pn1, pn2;
  Covariance3D                 cov3d;
  Vector3D                     zenithvec, direction, outervec, dominant_outervec,
                               perp_dom_outervec;
  FILE                         *dxffile, *statfile, *graph_file;
  Building                     buildng;
  Buildings                    buildngs;
  HoughSpace                   houghspace;
  cov3d = Covariance3D(0,0,0,0,0,0);
  PI = 3.14159;
  zenithvec = Vector3D(0, 0, 1);
  num_shapelines = 0;
  num_pairlines = 0;
//  int minsizesegment;
//  minsizesegment = 300;
  
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
printf("size before (%d)", laser_points.size());
 laser_points.ReduceData(0.1);
printf("and after (%d) removing double points\n", laser_points.size());

printf("Processing segments larger than %d points.\n", minsizesegment);

graph_file = fopen("building_graph.txt", "w");
bool splitstrangeshapedsegments =true;

if (splitstrangeshapedsegments){
 printf("Splitting strange shaped segments... & removing small (<%d points) segments\n", minsizesegment);        

 LaserPoints relabeled_laser_points, seg2_laser_points;
 new_segment_number = 0;
 int dominant_segment2;
 SegmentationParameters *spar=new SegmentationParameters();
      
 
for (map_line=map_lines.begin(), run_index=0; map_line!=map_lines.end(); map_line++, run_index++) {
       sel_laser_points.ErasePoints();
       sel_laser_points.AddTaggedPoints(laser_points, map_line->Number(),
                                         PolygonNumberTag);
    //   segment_numbers = sel_laser_points.AttributeValues(SegmentNumberTag);
       count = 0;
                              
       if (sel_laser_points.size()>minsizesegment){
       do {
        dominant_segment = sel_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
        seg_laser_points.ErasePoints();
        seg_laser_points.AddTaggedPoints(sel_laser_points, dominant_segment, SegmentNumberTag);
        edges.Erase();
        seg_laser_points.DeriveTIN();
        edges.Derive(seg_laser_points.TINReference());      
 //       seg_laser_points.RemoveLongEdges(edges,1.5,true); //for enschede, works fine 20-08-09
        seg_laser_points.RemoveLongEdges(edges,1.5,true); //for enschede, 1.5 works fine 20-08-09

        seg_laser_points.LabelComponents(edges, SegmentNumberTag);
      
          do {
            dominant_segment2 = seg_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count2);
            new_segment_number++;
            seg2_laser_points.ErasePoints();
            seg2_laser_points.AddTaggedPoints(seg_laser_points, dominant_segment2, SegmentNumberTag);
            seg2_laser_points.SetAttribute(SegmentNumberTag, new_segment_number);
            relabeled_laser_points.AddPoints(seg2_laser_points);
            seg_laser_points.RemoveTaggedPoints(dominant_segment2, SegmentNumberTag);
            } while (count2>minsizesegment);
        sel_laser_points.RemoveTaggedPoints(dominant_segment, SegmentNumberTag);
        printf("%4.1f %d %d\r", (100.0*run_index)/map_lines.size(), sel_laser_points.size(), seg_laser_points.size());     
        dominant_segment = sel_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
        } while (count>minsizesegment );
        } 
  }
  relabeled_laser_points.Write("relabeledpoints.laser", false);
 
 //return;
 laser_points = relabeled_laser_points;
 relabeled_laser_points.ErasePoints();
}  
 
 
bool renumber_segmentnumbers = true;
keep_laser_points = laser_points.SelectLastPulse(); // only keep last pulse data, to remove vegetated areas above buildings...
if (2 * keep_laser_points.size() > laser_points.size()){
      laser_points = keep_laser_points; // only keep last pulse data, to remove vegetated areas above buildings...
      }
      else {
           keep_laser_points = laser_points;
      }
if (renumber_segmentnumbers){ // it can happen that 1 segment is present in more than one linetopology; here every linetopology gets unique lasersegment numbers
   printf("\nrenumbering segments...");
   new_segment_number = 0;                          
   line_number=0;
   LaserPoints renumbered_laser_points, edgepoints, alledgepoints, edgepoints2;
   Triangles *triangles;
   const Triangle *triangle;
   int itr;
   for (map_line=map_lines.begin(); map_line!=map_lines.end(); map_line++) {
       sel_laser_points.ErasePoints();                                                    
       sel_laser_points.AddTaggedPoints(laser_points, map_line->Number(),
                                         PolygonNumberTag);
    //   segment_numbers = sel_laser_points.AttributeValues(SegmentNumberTag);
       do {
        dominant_segment = sel_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
        new_segment_number++;                                       
        seg_laser_points.ErasePoints();
        seg_laser_points = sel_laser_points.SelectTagValue(SegmentNumberTag, dominant_segment);
        seg_laser_points.SetAttribute(SegmentNumberTag, new_segment_number);
        renumbered_laser_points.AddPoints(seg_laser_points);
        sel_laser_points.RemoveTaggedPoints(dominant_segment, SegmentNumberTag);
        dominant_segment = sel_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
        } while (count>minsizesegment );
     }
       
             
  laser_points.ErasePoints();
  laser_points = renumbered_laser_points;
  renumbered_laser_points.ErasePoints();
}

keep_laser_points = laser_points;
keep_laser_points.Write("relabeled_renumbered_points.laser", false);
/*
// code to visualize difference between points and fitted plane 
     segment_numbers = keep_laser_points.AttributeValues(SegmentNumberTag);
     for (segment_number=segment_numbers.begin();segment_number!=segment_numbers.end();segment_number++){
          sel_laser_points.ErasePoints();
          sel_laser_points.AddTaggedPoints(keep_laser_points, *segment_number, SegmentNumberTag);
          plane = sel_laser_points.FitPlane(*segment_number, *segment_number, SegmentNumberTag);
          keep_laser_points.RemoveTaggedPoints(*segment_number, SegmentNumberTag);
          for (laser_point = sel_laser_points.begin(); laser_point!=sel_laser_points.end(); laser_point++){
               newpos = Position3D(laser_point->X(), laser_point->Y(),laser_point->Z());
               zdiff = plane.Distance(newpos);
               laser_point->Z() = zdiff;
           }               
           keep_laser_points.AddPoints(sel_laser_points);

     }
keep_laser_points.Write("differencetoplane.laser", false);
keep_laser_points = laser_points;
 
 */
 statfile = fopen(infofile,"w");
 fprintf(statfile, "mapline / line_number / segment1 / segment2 / dist / dir1 / dir2 / dir3\n");
 
 // program consists of many sub functions, meant for development and debugging, not for production
 // sub functions can be switched on/off here.
 found=false;
 bool   produce_building_blocks = false;
 bool   recolorsegments = false;
 bool   force_polygon_check   = false;
 bool   found_enclosing, found_rect, calc_poly;
 bool   add_map_data = true;
 bool   bpartition = false;
 bool   calc_height = false;//true;
 bool   new_approach = false;//true;
 bool   do_normal = true;
 bool   do_step_edge =true;// true;
    bool remove_loose_lines;
     remove_loose_lines = false;
 bool   generate_artificial_points = false;
 bool   swapped;
 bool   make_graph = true;//true;
bool add_intsectlines = false;
bool add_pairlines = true;

// some parameters to be set
 bool   b1,  b2;
 double fixed_eave_height = 300;
 int label_pair_step_edge = 4;
 int label_loose_flat_segment = 2;
 int label_pair_intersection_line = 6;
 int label_direction_line = 7;
 double flat_angle = 10;
 double vertical_angle = 10;
 int countingstepedgepoints = 0;
 int line_numberstepedge=0;
 line_number4 = 0;
 edgepointnumber = 0;
sel_map_lines = map_lines;
//sel_map_lines.Densify(map_points, 0.25);
//map_points.RemoveDoublePoints(sel_map_lines, 0.01);
highest_map_line_number = 0;
line_number=0;
line_number2 =0;
line_number3 =0;
line_number6 = 0;
countingintsect = 0;
overall_flat = 0;
overall_vertical = 0;
overall_tilted = 0;
DataType        typed;
typed = SelectedMapData;
LineTopsIterVector::iterator   partvector;
number_offset = map_points.HighestPointNumber().Number()+1;
number_offset2 = map_points.HighestPointNumber().Number()+1;
Planes         tilted_planes;
Line3D           intline;

printf("start, sel_map_lines.size = %d\n", sel_map_lines.size());
for (map_line=sel_map_lines.begin(), run_index=0; map_line!=sel_map_lines.end(); map_line++, run_index++) {
      height = 0;
//    printf("start.\n");
    printf("%4.1f\r", (100.0*run_index)/sel_map_lines.size());
    fprintf(graph_file,"%d ", map_line->Number());
//    printf("%d\n", map_line->Number());
//     if (map_line->TOP10MajorClass()==TOP10_Building) {
      if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end());
      if(!selridgelines.empty()) selridgelines.erase(selridgelines.begin(), selridgelines.end()); 
      if(!selridgepoints.empty()) selridgepoints.erase(selridgepoints.begin(), selridgepoints.end()); 
      if(!selridgelines_four.empty()) selridgelines_four.erase(selridgelines_four.begin(), selridgelines_four.end()); 
      if(!selridgepoints_four.empty()) selridgepoints_four.erase(selridgepoints_four.begin(), selridgepoints_four.end()); 
 //     if(!shape_points.empty()) shape_points.erase(shape_points.begin(), shape_points.end());  
 //     if(!shapelines.empty()) shapelines.erase(shapelines.begin(), shapelines.end());
      if(!map_tin_lines.empty()) map_tin_lines.erase(map_tin_lines.begin(),map_tin_lines.end());
      if(!max_tin_lines.empty()) max_tin_lines.erase(max_tin_lines.begin(),max_tin_lines.end());
      if(!tilted_planes.empty()) tilted_planes.erase(tilted_planes.begin(), tilted_planes.end());
      if(!pnls.empty()) pnls.erase(pnls.begin(), pnls.end());
      if (make_graph) graph_lines.erase(graph_lines.begin(), graph_lines.end());
      if (make_graph) graph_points.erase(graph_points.begin(), graph_points.end());

      sel_laser_points.ErasePoints();                                                    
      sel_laser_points.AddTaggedPoints(laser_points, map_line->Number(),
                                         PolygonNumberTag);
      keep_sel_laser_points.ErasePoints();
      keep_sel_laser_points = sel_laser_points;
      
     if (sel_laser_points.size()<4) {
                                   height = -100;
                                     sel_map_lines.erase(map_line);
                                     map_line--, run_index--;
                                   }
     else {
     map_line->Label() = 1001;
 //         printf("laser heights: %5.2f\n", height);
       area = map_line->CalculateArea(map_points);
       pd = (1.0*sel_laser_points.size())/area;
       pdist = 1.0/sqrt(pd);
       printf("%4.2f, %6.2f, %4.2f\n", pd, area, pdist);
     if (generate_artificial_points){ //generate artificial points in buildings, for example to find gaps
       polybounds = map_line->Bounds(map_points);
       posmin = polybounds.Minimum();
       posmax = polybounds.Maximum();
       runx = int (posmin.X());
       runy = int (posmin.Y());
       do {
          artpoint.X() = runx;
          do {
              artpoint.Y() = runy;
              artpoint.Z() = 0;
              runpos = Position3D(runx, runy, 0);
              if (artpoint.InsidePolygon(map_points, map_line->LineTopologyReference())) {
                 nearlaspoint = sel_laser_points[0];                       
                 laspos = Position3D(nearlaspoint.X(), nearlaspoint.Y(),0);             
                 for (laser_point = sel_laser_points.begin(); laser_point!=sel_laser_points.end(); laser_point++){
                     newpos = Position3D(laser_point->X(), laser_point->Y(),0);
                     if (runpos.Distance(newpos)<runpos.Distance(laspos)) laspos = newpos;
                 } 
                 if (runpos.Distance(laspos)>dist){
                   artpoint.Label(map_line->Label());
                   artpoints.push_back(artpoint);
                   }
                 }
              runy = runy + dist;
              } while (runy < posmax.Y());
          runy = int (posmin.Y());   
          runx = runx + dist; 
       } while (runx < posmax.X());
     }
     if (do_normal) {
         segment_numbers = sel_laser_points.AttributeValues(SegmentNumberTag);
         printf("# of laser segments: %d\n", segment_numbers.size());
        if(!segments.empty()) segments.erase(segments.begin(), segments.end());
        count = 0;
        cuma = 0;
        total_flat = 0;
        total_vertical = 0;
        total_tilted = 0;
        num_tilted = 0;
        num_horizontal = 0;
        total_size = sel_laser_points.size();
        tilted_laser_points.ErasePoints();
        largest_segments_points.ErasePoints();
        biggest_four = 0;
        do {
        dominant_segment = sel_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
        segments.push_back(sel_laser_points.TaggedPointNumberList(SegmentNumberTag, dominant_segment));
        plane = sel_laser_points.FitPlane(dominant_segment, dominant_segment, SegmentNumberTag);
        
        printf ("angle = %5.2f", (plane.Normal()[0]*180/PI));
        if (!plane.IsHorizontal(flat_angle*PI/180)) {
                                             if (plane.IsVertical(vertical_angle*PI/180)){ //not horizontal, but yes vertical
                                               seg_laser_points.ErasePoints();
                                               seg_laser_points = sel_laser_points.SelectTagValue(SegmentNumberTag, dominant_segment);
                                               quad = 1;
                                               seg_laser_points.Label(2000); //to make it white
                       //                        largest_segments_points.AddPoints(seg_laser_points);
                       //                        tot_largest_segments_points.AddPoints(seg_laser_points);
                                               total_vertical=total_vertical+count;
                                               vertical_points.AddPoints(seg_laser_points);
                                                                                }
                                               else { //not horizontal && not vertical
//                                             largest_segments_points.AddTaggedPoints(sel_laser_points, dominant_segment, SegmentNumberTag);
                                               swapped = false;
                                               biggest_four++;
                                               if ((plane.Normal()[2])<0) {
                                                                           plane.Normal()[0] = plane.Normal()[0]*(-1);
                                                                           plane.Normal()[1] = plane.Normal()[1]*(-1);
                                                                           plane.Normal()[2] = plane.Normal()[2]*(-1);
                                                                           swapped = true;
                                                                           }
                                               if ((plane.Normal()[0])>0 && (plane.Normal()[1])>0) quad = 5; //to make it green 1 was a bit whitish
                                               if ((plane.Normal()[0])>0 && (plane.Normal()[1])<0) quad = 2;
                                               if ((plane.Normal()[0])<0 && (plane.Normal()[1])<0) quad = 3;
                                               if ((plane.Normal()[0])<0 && (plane.Normal()[1])>0) quad = 4;
                                       //        if (fabs(plane.Normal()[1])>0.00001) intangle = int(atan(plane.Normal()[0]/plane.Normal()[1])*180/PI);
                                               if (fabs(plane.Normal()[1])>0.00001) angle = atan2(plane.Normal()[0], plane.Normal()[1])*180/PI;
                                               else {
                                                    if (plane.Normal()[0]>0) angle = 90;
                                                    else angle = 270;
                                                 //intangle = 90;
                                               }
                                               if (angle < 0) angle = angle + 360; // make angles positive
                                               if (angle >= 180) angle = angle - 180; // combine opposite planes, just for visualisation
                                               
                                                                                              seg_laser_points.ErasePoints();
                                               seg_laser_points = sel_laser_points.SelectTagValue(SegmentNumberTag, dominant_segment);
                                               seg_laser_points.Label(1000+1000*quad);
                                               if (seg_laser_points.size()>minsizesegment){ //only keep bigger tilted segments
                                               pnl.erase(pnl.begin(),pnl.end());
                                                pnl = keep_laser_points.TaggedPointNumberList(SegmentNumberTag, dominant_segment);
                                               pnls.push_back(pnl); //visualise direction of horizontal line on the plane          
                                               outervec = plane.Normal().VectorProduct(zenithvec).Normalize();
                                               objpseg1 = keep_laser_points.Centroid(pnl, dominant_segment);
                                               objpseg2 = ObjectPoint(objpseg1.X()+outervec.X(), 
                                                          objpseg1.Y()+outervec.Y(), objpseg1.Z()+outervec.Z(), 
                                                          dominant_segment+new_segment_number, 0,0,0,0,0,0);
                                               if (swapped) {
                                                            plane.Normal()[0] = plane.Normal()[0]*(-1); //swap it back
                                                            plane.Normal()[1] = plane.Normal()[1]*(-1);
                                                            plane.Normal()[2] = plane.Normal()[2]*(-1);
                                                            }
                                               largest_segments_points.AddPoints(seg_laser_points);
                                               tot_largest_segments_points.AddPoints(seg_laser_points);
                                               tilted_laser_points.AddPoints(seg_laser_points);
                                               total_tilted_points.AddPoints(seg_laser_points);
                                               tilted_planes.push_back(plane);
                                               total_tilted = total_tilted+count;
                                               num_tilted++;
                                               }
                                     //          pnl = seg_laser_points.TaggedPointNumberList(LabelTag, 1000+1000*quad);
                                              
                                               }
                                             }
        else { //horizontal
             //       biggest_four++;
                      seg_laser_points.ErasePoints();
                      seg_laser_points = sel_laser_points.SelectTagValue(SegmentNumberTag, dominant_segment);
                      quad = 0;
                      seg_laser_points.Label(1000);
                      printf ("labelled");
                //      seg_laser_points.Label(biggest_four);
                      largest_segments_points.AddPoints(seg_laser_points);
                      tot_largest_segments_points.AddPoints(seg_laser_points);
                      total_flat = total_flat + count;
                      pnl.erase(pnl.begin(),pnl.end());
                      pnl = keep_laser_points.TaggedPointNumberList(SegmentNumberTag, dominant_segment);
                      printf ("pnl");
                      pnls.push_back(pnl);
                      printf ("pnls");
                      tilted_planes.push_back(plane); // also add horizontal planes...
                      printf ("plns");
                      num_horizontal++;
                      }
    //    }                                          
        cuma = cuma + count;
      //  fprintf (statfile, "segment %d, size = %d (%4.2f), cuma = %4.2f, normal = %d\n", dominant_segment, count, (100.0*count)/total_size, (100.0*cuma)/total_size, quad);                                       
          printf ("segment %d, size = %d (%4.2f), cuma = %4.2f, normal = %d\n", dominant_segment, count, (100.0*count)/total_size, (100.0*cuma)/total_size, quad);                                       
        sel_laser_points.RemoveTaggedPoints(dominant_segment, SegmentNumberTag);
        
        prev_count = count;
        dominant_segment = sel_laser_points.MostFrequentAttributeValue(SegmentNumberTag, count);
        } while (count>minsizesegment);// && tilted_planes.size()<40);//&& (100.0*count)/total_size >5);

//        } while (count>minsizesegment && tilted_planes.size()<40 && (100.0*count)/total_size >2);
        local_eave_height = fixed_eave_height;
        count = 0;
        largest_segments_points.DeriveDataBounds(0);
        tilted_laser_points.DeriveDataBounds(0);
        fprintf (statfile, "number of segments %d, number of flat: %d, number of tilted: %d\n", tilted_planes.size(), num_horizontal, num_tilted);
        printf ("number of segments %d, number of flat: %d, number of tilted: %d\n", tilted_planes.size(), num_horizontal, num_tilted);
        if (tilted_planes.size()==1){
            if (num_horizontal ==1) {
                               fprintf(statfile,"flat roof building found\n");
                               fprintf(graph_file," 0 0");
                               }
            else {
                 fprintf(statfile,"shed roof building found\n");
                 fprintf(graph_file," 0 1");
                 }
            fs = largest_segments_points.MostFrequentAttributeValue(SegmentNumberTag, count);
            flat_points.ErasePoints();
            flat_points = largest_segments_points.SelectTagValue(SegmentNumberTag, fs);
            tot_largest_segments_points.RemoveTaggedPoints(fs, SegmentNumberTag);
  //          largest_segments_points.RemoveTaggedPoints(fs, SegmentNumberTag);
            flat_points.Label(1500);
            tot_largest_segments_points.AddPoints(flat_points);
    //        largest_segments_points.AddPoints(flat_points);
            }
            max_dist = 0;
            max_height = 0;
            min_height = 100000;
            
            if (tilted_planes.size()>1){
            for (pnlit=pnls.begin(), plane1=tilted_planes.begin();
                    pnlit!=pnls.end(); pnlit++, plane1++) {
                for (pnlit2=pnlit+1, plane2=plane1+1;
                       pnlit2!=pnls.end(); pnlit2++, plane2++) {
                   if (Intersect2Planes(*plane1, *plane2, intline)){
                     if (keep_laser_points.IntersectFaces(*pnlit, *pnlit2, *plane1, *plane2, 0.5, pos1, pos2)){
//                   if (keep_laser_points.IntersectFaces(*pnlit, *pnlit2, *plane1, *plane2, 2*pdist, pos1, pos2)){
                        dist = pos1.Distance(pos2);            
                        if (dist>minlinelength){ //for the moment; only look at intersections longer than mll
                           countingintsect++;
                           pn1 = PointNumber(countingintsect);                        
                           countingintsect++;
                           beginp = ObjectPoint(pos1, pn1, cov3d);
                           pn2 = PointNumber(countingintsect);
                           endp = ObjectPoint(pos2, pn2, cov3d);
                           if (max_height<beginp.Z()) max_height = beginp.Z();
                           if (max_height<endp.Z()) max_height = endp.Z();
                           if (min_height>beginp.Z()) min_height = beginp.Z();
                           if (min_height>endp.Z()) min_height = endp.Z();
                           if (max_dist<dist)max_dist = dist;
                           countingintsect--;
                           countingintsect--;
                          }
                        }
                        }
                    }
                 }
            fprintf(statfile,"Max/min height: %8.2f / %8.2f, maxdist: %8.2f\n", max_height, min_height, max_dist);
            for (pnlit=pnls.begin(), plane1=tilted_planes.begin();
                    pnlit!=pnls.end(); pnlit++, plane1++) {
                for (pnlit2=pnlit+1, plane2=plane1+1, line_number2=0;
                       pnlit2!=pnls.end(); pnlit2++, plane2++, line_number2++) {
                   sn1 = (keep_laser_points[pnlit->begin()->Number()]).Attribute(SegmentNumberTag);
                   sn2 = (keep_laser_points[pnlit2->begin()->Number()]).Attribute(SegmentNumberTag);
     //              if (pnlit!=pnlit2){
                   fprintf(statfile,"analysing %d and %d: ", sn1, sn2);

                   if (Intersect2Planes(*plane1, *plane2, intline)){
                    b1 = (*plane1).IsHorizontal(flat_angle*PI/180);
                    b2 = (*plane2).IsHorizontal(flat_angle*PI/180);                            
                   fprintf(statfile," planes intersect ");
                    
                    count++;
                    if (keep_laser_points.IntersectFaces(*pnlit, *pnlit2, *plane1, *plane2, 0.5, pos1, pos2)){
                      fprintf(statfile," faces intersect ");

   //                 if (keep_laser_points.IntersectFaces(*pnlit, *pnlit2, *plane1, *plane2, 2*pdist, pos1, pos2)){
   //                     fprintf(statfile," faces intersect ");
                        countingintsect++;
                        pn1 = PointNumber(countingintsect);                        
                        countingintsect++;
                        beginp = ObjectPoint(pos1, pn1, cov3d);
                        if (beginp.Z()<local_eave_height) local_eave_height = beginp.Z();
                        pn2 = PointNumber(countingintsect);
                        endp = ObjectPoint(pos2, pn2, cov3d);
                        if (endp.Z()<local_eave_height) local_eave_height = endp.Z();
                        if (endp.Z()<beginp.Z()) loc_max_height = beginp.Z();
                        else loc_max_height = endp.Z();
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
                          selridgepoints.push_back(beginp);
                          selridgepoints.push_back(endp);
             //             selridgelines.push_back(intsect);
                            shapeline = intsect;
              //              shape_points.push_back(beginp);
              //              shape_points.push_back(endp);
       //                     shapeline.Densify(shape_points, 1.0);
//                            shapelines.push_back(shapeline);
                          
                          height_score = fabs(max_height - loc_max_height);
                          height_score = 1 - height_score / (max_height - min_height);
                          pnlseg1 = largest_segments_points.SelectTagValueList(SegmentNumberTag, sn1);
                          pnlseg2 = largest_segments_points.SelectTagValueList(SegmentNumberTag, sn2);
                          lab1 = (largest_segments_points[pnlseg1.begin()->Number()]).Attribute(LabelTag);
                          lab2 = (largest_segments_points[pnlseg2.begin()->Number()]).Attribute(LabelTag);
                          gambrel_score = 0;
                          if (intsect.Label()!=6) gable_score = 0;
                          else {
                               if (abs(lab1-lab2)==2000) gable_score = height_score * (pnlseg1.size()+pnlseg2.size())/tilted_laser_points.size();
                               else {
                                    if (abs(lab1-lab2)==0) gambrel_score = 1;
                                    gable_score = 0;
                                    }
                               }
                          fprintf(statfile, "%6d %8d %8d %8.2f %d %d %8.2f %8.2f %8.2f %8.2f\n", 
                            map_line->Number(), sn1, sn2, dist,
                            pnlseg1.size(), pnlseg2.size(), direction[2]*180/PI, 
                            gable_score, 100 * height_score, gambrel_score);
      //                    objpseg1 = keep_laser_points.Centroid(pnlseg1, sn1);
      //                    objpseg2 = keep_laser_points.Centroid(pnlseg2, sn2);  
                          objpseg1 = largest_segments_points.Centroid(pnlseg1, sn1);
                          objpseg2 = largest_segments_points.Centroid(pnlseg2, sn2);
                            lab1 = int(lab1/1000) -1;
                            lab2 = int(lab2/1000) -1;
                            if (intsect.Label()!=6){
                            il3 = Line3D(pos1, pos2);
                            
                            //select points from both segments that are near intersection line...
                            midpos[0] = (pos1[0] + pos2[0])/2;
                            midpos[1] = (pos1[1] + pos2[1])/2;
                            midpos[2] = (pos1[2] + pos2[2])/2;
                            seg1_laser_points.ErasePoints();
                            seg1_laser_points = largest_segments_points.SelectTagValue(SegmentNumberTag, sn1);
                            edges1.Erase();
                            seg1_laser_points.DeriveTIN();
                            edges1.Derive(seg1_laser_points.TINReference());     
                            nearest_laser_point1 = seg1_laser_points.NearestPoint(midpos,
                                                    edges1, false);
                            neighbourhood1 = seg1_laser_points.Neighbourhood(PointNumber(nearest_laser_point1), 2,
                                             edges1, false, false);
                            objpseg1 = seg1_laser_points.Centroid(neighbourhood1, sn1);
                            
                            seg2_laser_points.ErasePoints();
                            seg2_laser_points = largest_segments_points.SelectTagValue(SegmentNumberTag, sn2);
                            edges2.Erase();
                            seg2_laser_points.DeriveTIN();
                            edges2.Derive(seg2_laser_points.TINReference());                                 
                            nearest_laser_point2 = seg2_laser_points.NearestPoint(midpos,
                                                    edges2, false);
                            neighbourhood2 = seg2_laser_points.Neighbourhood(PointNumber(nearest_laser_point2), 2,
                                             edges2, false, false);
                            objpseg2 = seg2_laser_points.Centroid(neighbourhood2, sn2);
                                             
                            pos3 = Position3D(objpseg1.X(), objpseg1.Y(), objpseg1.Z());
                            pos4 = Position3D(objpseg2.X(), objpseg2.Y(), objpseg2.Z());
                            il4 = Line3D(pos3, pos4);
                            if (Intersection2Lines(il3, il4, pos5)){
                                 if (il4.Z(pos5[0])<pos5[2]) {
                                   printf ("convex\n");
                                   intsect.Label() = 8;
                                 }
                                 else {
                                      printf ("concave\n");
                                      intsect.Label() = 9; 
                                      }

                            }
                            }
                            else{ //if horizontal intersection line, label according to gambrel or mansard
                                  // and check if it might be a dormer -> also stepedges to the same segment
                                 pnlseg1 = largest_segments_points.SelectTagValueList(SegmentNumberTag, sn1);
                                 pnlseg2 = largest_segments_points.SelectTagValueList(SegmentNumberTag, sn2);                  
                                 objpseg1 = largest_segments_points.Centroid(pnlseg1, sn1);
                                 objpseg2 = largest_segments_points.Centroid(pnlseg2, sn2);
                          
                                 if (lab1-lab2==0) intsect.Label() = 7; //both same normal, gambrellabel = 7;
                                 if (lab1 == 0 || lab2 == 0) intsect.Label() = 5; //one horizontal, one tilted, mansardlabel = 5
                                 if (intsect.Label()!=6){ // if 7 or 5, look for dormers
                                   dormer_points.ErasePoints();
                                   non_dormer_points.ErasePoints(); 
                                   if (lab1 == 0){// && 1.0*pnlseg1.size()<3*pnlseg2.size()) { 
                                            dormer_points.AddTaggedPoints(largest_segments_points, sn1, SegmentNumberTag);
                                            non_dormer_points.AddTaggedPoints(largest_segments_points, sn2, SegmentNumberTag);
                                            ds = sn1;
                                            centroidofdormer = LaserPoint(objpseg1.X(), objpseg1.Y(), objpseg1.Z());
                                            }
                                   else {
                                        dormer_points.AddTaggedPoints(largest_segments_points, sn2, SegmentNumberTag);
                                        non_dormer_points.AddTaggedPoints(largest_segments_points, sn1, SegmentNumberTag);
                                        ds = sn2;
                                        centroidofdormer = LaserPoint(objpseg2.X(), objpseg2.Y(), objpseg2.Z());
                                        }
                             //      dormer_points.AddTaggedPoints(largest_segments_points, sn1, SegmentNumberTag);
                                   fprintf(statfile, "\n lab1 = %d, sn1 = %d, lab2 = %d, sn2 = %d, ds = %d\n", lab1, sn1, lab2, sn2, ds);
                                   fprintf(statfile, "\n X = %4.2f, Y = %4.2f, Z = %4.2f", centroidofdormer.X(), centroidofdormer.Y(), centroidofdormer.Z());
                                   
                                   non_dormer_points.DeriveTIN();
                                   edges.Erase();
                                   component.Erase();
                                   temp_points.erase(temp_points.begin(), temp_points.end());
                                   edges.Derive(non_dormer_points.TINReference());
                                 //  non_dormer_points.RemoveLongEdges(edges, 2*pdist, true);                                   
                                   for (int i=0; i<non_dormer_points.size(); i++) component.push_back(PointNumber(i));
                                   nondormertop = non_dormer_points.DeriveContour(1, component, 4, 3, edges);
                                   temp_points = non_dormer_points.ConstructObjectPoints();
                                   nondormerobj.erase(nondormerobj.begin(),nondormerobj.end());
                                   for (node2 = nondormertop.begin(); node2!=nondormertop.end();node2++){
                                        nondormerobj.push_back(*(temp_points.PointIterator(*node2)));
                                        } 
                                   if (!nondormertop.IsClosed()) nondormertop.push_back(*(nondormertop.begin()));
                                   if (centroidofdormer.InsidePolygon(nondormerobj, nondormertop.LineTopologyReference())){
                                    int perc = 0;
                                    for (laser_point=dormer_points.begin(); laser_point!=dormer_points.end(); laser_point++) {
                                         if (laser_point->InsidePolygon(nondormerobj, nondormertop.LineTopologyReference())){
                                            perc++;
                                         }
                                     }
                                     fprintf(statfile, "\n#### size of area = %4.2f ###\n", nondormertop.CalculateArea(nondormerobj));
                                     printf("\n#### size of area = %4.2f ###\n", nondormertop.CalculateArea(nondormerobj));                                   
                                     nondormertops.erase(nondormertops.begin(), nondormertops.end());
                                     nondormertop.Label() = non_dormer_points[0].Label();
                                     nondormertops.push_back(nondormertop);
                                     next_pnr = allnondormerobj.HighestPointNumber().Number()+1;
                                     line_number6 = line_number6+1;
                                     nondormertops.ReNumber(nondormerobj, next_pnr, line_number6);
                                     allnondormertops.insert(allnondormertops.end(), nondormertops.begin(), nondormertops.end());
                                     allnondormerobj.insert(allnondormerobj.end(), nondormerobj.begin(), nondormerobj.end());
                                     printf("\n#### size of dormerpoints = %d ###\n", dormer_points.size());//totedges, totedges2);
                                     if (dormer_points.size()>0){
                                       fprintf(statfile, "\n#### percentage in = %4.2f ###\n", perc*100.0/dormer_points.size());//totedges, totedges2);
                                       printf("\n#### percentage in = %4.2f ###\n", perc*100.0/dormer_points.size());//totedges, totedges2);
                                       if (perc*100.0/dormer_points.size()>70) {
                                           all_dormer_points.AddPoints(dormer_points);
                                           // indicate as dormer connection...
                                           intsect.Label() = 2; 
                                           tot_largest_segments_points.RemoveTaggedPoints(ds, SegmentNumberTag);
              //                             largest_segments_points.RemoveTaggedPoints(ds, SegmentNumberTag);
                                           dormer_points.Label(1600);
                                           tot_largest_segments_points.AddPoints(dormer_points);
              //                             largest_segments_points.AddPoints(dormer_points);
                                        }
                                     }
                                     }//end if centroidofdormer is inside
                                   }
                                 }
               //           objpseg1.Z() = int (lab1/1000) - 1;  
               //           objpseg2.Z() = int (lab2/1000) - 1; 
                          objpseg1 = largest_segments_points.Centroid(pnlseg1, sn1);
                          objpseg2 = largest_segments_points.Centroid(pnlseg2, sn2);
                          pairpoints.push_back(objpseg1);
                          pairpoints.push_back(objpseg2);
                          
                          pairline = LineTopology(line_number3, 1, sn1, sn2);
                            lab1 = int(lab1/1000) -1;
                            lab2 = int(lab2/1000) -1;
                             pairline.SetAttribute(BuildingNumberTag, map_line->Number());
                             pairline.Label() = intsect.Label();
                            shapeline.Label() = intsect.Label();
                            selridgelines.push_back(shapeline);
                            num_shapelines++;
                            pairlines.push_back(pairline);
                            num_pairlines++;
              //            shapelines.push_back(shapeline);
                          line_number3++;
                          if (make_graph) {
                                          pairline.Label() = intsect.Label();
                                          graph_lines.push_back(pairline);
                                          graphlines.push_back(pairline);
                                          graph_points.push_back(objpseg1);
                                          graph_points.push_back(objpseg2);
                                          }
                          }
                        }
                        }
                    }
     //            }
                 }
              }
            }
                    
      //   tilted_segment_numbers = tilted_laser_points.AttributeValues(SegmentNumberTag);
         overall_flat = overall_flat + total_flat;
         overall_tilted = overall_tilted + total_tilted;
         overall_vertical = overall_vertical + total_vertical;  
     } //end do_normal
     if (do_step_edge){
       sel_laser_points.ErasePoints();                                                    
       if (do_normal){
          sel_laser_points.AddPoints(largest_segments_points);
          sel_laser_points.RemoveTaggedPoints(1600, LabelTag);
          }
       else{            
         sel_laser_points.AddTaggedPoints(laser_points, map_line->Number(),
                                           PolygonNumberTag);
         }
         segment_numbers = sel_laser_points.AttributeValues(SegmentNumberTag);
        if(!segments.empty()) segments.erase(segments.begin(), segments.end());
        for (segment_number=segment_numbers.begin();
             segment_number!=segment_numbers.end(); segment_number++) {
             segments.push_back(sel_laser_points.TaggedPointNumberList(SegmentNumberTag, *segment_number));
            }
            printf ("begin stepedge, size of segments: %d\n", segments.size());  
       for (segment=segments.begin(), segment_number=segment_numbers.begin();
            segment!=segments.end(); segment++, segment_number++) {
            for (segment2=segment, segment_number2=segment_number; 
                    segment2!=segments.end(); segment2++, segment_number2++) {
                if (segment!=segment2){
                  seg_laser_points.ErasePoints();
                  seg_laser_points = sel_laser_points.SelectTagValue(SegmentNumberTag, *segment_number);
                  size1 = seg_laser_points.size();
                  seg_laser_points.AddTaggedPoints(sel_laser_points, *segment_number2, SegmentNumberTag);
                  size2 = seg_laser_points.size()-size1;
                  seg_laser_points.DeriveTIN();
                  edges.Erase();
                  edges.Derive(seg_laser_points.TINReference());
    //              seg_laser_points.RemoveLongEdges(edges, 2*pdist, true);
              seg_laser_points.RemoveLongEdges(edges, 0.75, true);
                  totedges2 = seg_laser_points.CountMixedEdges(edges, SegmentNumberTag);
                  mixed_points.ErasePoints();
//                  if (totedges2>20 && size1 > 100 && size2 > 100){ //if connected 2D && only continue if two larger segments
                  if (totedges2>20 && (size1 > 2*minsizesegment || size2 > 2*minsizesegment)){ //if connected 2D && only continue if at least one is larger segments
                           mixed_points = seg_laser_points.ReturnMixedEdges(edges, SegmentNumberTag);        
  //                         seg_laser_points.RemoveLongEdges(edges, 2*pdist, false); //now in 3D
                           seg_laser_points.RemoveLongEdges(edges, 0.75, false); //now in 3D
                           totedges3 = seg_laser_points.CountMixedEdges(edges, SegmentNumberTag);
                           sn1 = *segment_number;
                             sn2 = *segment_number2;
                             pnlseg1 = sel_laser_points.SelectTagValueList(SegmentNumberTag, sn1);
                             pnlseg2 = sel_laser_points.SelectTagValueList(SegmentNumberTag, sn2);
                             lab1 = (sel_laser_points[pnlseg1.begin()->Number()]).Attribute(LabelTag);
                             lab2 = (sel_laser_points[pnlseg2.begin()->Number()]).Attribute(LabelTag);
                           
                           if (totedges3 < 1 || (lab1==1000 && lab2==1000)){      // not in 3D connected or both horizontal, indicates step edge  
                           // put on if recolor segment  total_mixed_points.AddPoints(mixed_points);                             
                               dominant_dir = false;
                             if (lab1 > 2000 && lab2 >2000){ //  both tilted
                                // do not look for dominant direction
                                }
                                else {
                                     if (lab1 > 2000){ //segment1 is tilted
                                         plane = sel_laser_points.FitPlane(sn1, sn1, SegmentNumberTag); //fit plane through segment1
                                         dominant_outervec = plane.Normal().VectorProduct(zenithvec).Normalize(); //calc norm direction
                                         perp_dom_outervec[0] = -1*(dominant_outervec[1]);
                                         perp_dom_outervec[1] = dominant_outervec[0];
                                         perp_dom_outervec[2] = dominant_outervec[2];
                                         dominant_dir = true;
                                     }
                                     else if (lab2 > 2000){ //segment2 is tilted
                                         plane = sel_laser_points.FitPlane(sn2, sn2, SegmentNumberTag); //fit plane through segment2
                                         dominant_outervec = plane.Normal().VectorProduct(zenithvec).Normalize(); //calc norm direction
                                         perp_dom_outervec[0] = -1*(dominant_outervec[1]);
                                         perp_dom_outervec[1] = dominant_outervec[0];
                                         perp_dom_outervec[2] = dominant_outervec[2];
                                         dominant_dir = true;
                                     }
                                     else {
                                          // both horizontal, do not look for dominant direction
                                          }
                                }
                                     
                             plane = mixed_points.FitPlane(map_line->Number(), map_line->Number(), PolygonNumberTag); //fit plane through mixed points
                             outervec = plane.Normal().VectorProduct(zenithvec).Normalize();
                             if (dominant_dir) {
                                               mindir = zenithvec.Angle2D(outervec, dominant_outervec);
                                               mindir2 = zenithvec.Angle2D(outervec, perp_dom_outervec);
                                               if (fabs(sin(mindir))<fabs(sin(mindir2))){ 
                                                 outervec = dominant_outervec;
                                               }
                                               else {
                                                    outervec = perp_dom_outervec;
                                                }
                                               }
                             else {
                                  outervec = plane.Normal().VectorProduct(zenithvec).Normalize();
                             }
                             edgepointnumber = countingintsect;
                             pnl = mixed_points.TaggedPointNumberList(PolygonNumberTag, map_line->Number());
                             edgepointnumber++; countingintsect++;
                             objpseg1 = mixed_points.Centroid(pnl, edgepointnumber);
                             plane = keep_laser_points.FitPlane(sn1, sn1, SegmentNumberTag); //fit plane through first segment
                             objpseg1 = ObjectPoint(objpseg1.X()-outervec.X(), 
                                        objpseg1.Y()-outervec.Y(), 0, 
                                                          edgepointnumber, 0,0,0,0,0,0);
                             
                             objpseg1.Z() = plane.Z_At(objpseg1.X(), objpseg1.Y(), &success); // to calculate the height of stededge at one side
                             edgepointnumber++, line_number4++; countingintsect++;
                             objpseg2 = ObjectPoint(objpseg1.X()+outervec.X(), 
                                        objpseg1.Y()+outervec.Y(), 0, 
                                                          edgepointnumber, 0,0,0,0,0,0);
                             objpseg2.Z() = plane.Z_At(objpseg2.X(), objpseg2.Y(), &success);
                             stepedgepoints.push_back(objpseg1);
                             stepedgepoints.push_back(objpseg2);
                             pairline = LineTopology(line_number4, 1, edgepointnumber-1, edgepointnumber);
                             shapeline = LineTopology(line_number3, 1, edgepointnumber-1, edgepointnumber);
                             shapeline.Label() = label_pair_step_edge;
                             selridgelines.push_back(shapeline);
                             selridgepoints.push_back(objpseg1);
                             selridgepoints.push_back(objpseg2);
                             edgepointnumber++;
                             plane = keep_laser_points.FitPlane(sn2, sn2, SegmentNumberTag); //fit plane through second segment
                             objpseg1 = mixed_points.Centroid(pnl, edgepointnumber);
                             objpseg1 = ObjectPoint(objpseg1.X()-outervec.X(), 
                                        objpseg1.Y()-outervec.Y(), 0, 
                                                          edgepointnumber, 0,0,0,0,0,0);
                             objpseg1.Z() = plane.Z_At(objpseg1.X(), objpseg1.Y(), &success); // to calculate the height of stededge at the other side
                             edgepointnumber++, line_number4++;
                             objpseg2 = ObjectPoint(objpseg1.X()+2*outervec.X(), 
                                        objpseg1.Y()+2*outervec.Y(), 0, edgepointnumber, 0,0,0,0,0,0);
                             objpseg2.Z() = plane.Z_At(objpseg2.X(), objpseg2.Y(), &success);
                             objpseg1 = sel_laser_points.Centroid(pnlseg1, sn1);
                             objpseg2 = sel_laser_points.Centroid(pnlseg2, sn2);
                             pairpoints.push_back(objpseg1);
                             pairpoints.push_back(objpseg2);
                             pairline = LineTopology(line_number3, 1, sn1, sn2);
                             lab1 = int (lab1/1000) -1;
                             lab2 = int (lab2/1000) -1;
                             pairline.Label() = label_pair_step_edge;
                             pairline.SetAttribute(BuildingNumberTag, map_line->Number());
                             pairlines.push_back(pairline);
                             fprintf(statfile, "%6d %8d %8d %8d \n", 
                               map_line->Number(), line_number3, sn1, sn2);
                             
                             if (make_graph) {
                                          pairline.Label() = label_pair_step_edge;
                                          graph_lines.push_back(pairline);
                                          graphlines.push_back(pairline);
                                          graph_points.push_back(objpseg1);
                                          graph_points.push_back(objpseg2);
                                          }
                                          
                             // test fit plane through higher segment and mixed segment:
                             int highest, lowest;
                             if (objpseg1.Z()>objpseg2.Z()) {
                                                            highest = sn1;
                                                            lowest = sn2;
                                                            }
                             else {
                                  highest = sn2;
                                  lowest = sn1;
                                  }
                             stepedgelaserpoints.ErasePoints();
                             mixed_points.Label(2002);
                             stepedgelaserpoints.AddPoints(mixed_points);
                             stepedgelaserpoints.AddTaggedPoints(seg_laser_points, highest, SegmentNumberTag);
                             step = stepedgelaserpoints.TaggedPointNumberList(LabelTag, 2002);
                             highestseg = stepedgelaserpoints.TaggedPointNumberList(SegmentNumberTag, highest);
                             
                             planev = stepedgelaserpoints.FitPlane(2002, 2002, LabelTag); //fit plane through second segment
                             planeh = stepedgelaserpoints.FitPlane(highest, highest, SegmentNumberTag);
                             if (stepedgelaserpoints.IntersectFaces(step, highestseg, planev, planeh, 1, pos1, pos2)){
                               countingstepedgepoints++;
                               pn1 = PointNumber(countingstepedgepoints);                        
                               countingstepedgepoints++;
                               beginp = ObjectPoint(pos1, pn1, cov3d);
                               pn2 = PointNumber(countingstepedgepoints);
                               endp = ObjectPoint(pos2, pn2, cov3d);
                               intsect = LineTopology(line_numberstepedge, 4, pn1, pn2);
                               line_numberstepedge++;
                               fittedstepedgepoints.push_back(beginp);
                               fittedstepedgepoints.push_back(endp);
                               intsect.SetAttribute(LineNumberTag, line_number3);
                               intsect.SetAttribute(SegmentLabel, highest); // set segmentnumber to highest. second segment label to lowest segment
                               intsect.SetAttribute(SecondSegmentLabel, lowest); // set segmentnumber to highest. second segment label to lowest segment
                               intsect.SetAttribute(BuildingNumberTag, map_line->Number());
                               fittedstepedgelines.push_back(intsect);
                               }
                              line_number3++;            
                           }
                   }
                }
            }
       }
                  
     } //end do step edge
  
     if (remove_loose_lines){
     total_tilted_points.ErasePoints();
     temp_pairlines = graph_lines; 
     i=0;
     for (map_point=graph_points.begin(); map_point!=graph_points.end(); map_point++){
      count=0;
     for (map_line3=temp_pairlines.begin(); map_line3!=temp_pairlines.end(); map_line3++) {
      if (map_line3->Label()!= label_direction_line && map_line3->Label()!= label_pair_step_edge){  
        if (map_line3->Contains(map_point->NumberRef())) {
         count++;
         }
       }
      }
      if (count<2){
        for (map_line3=graph_lines.begin(); map_line3!=graph_lines.end(); map_line3++) {
          if (map_line3->Label()!= label_direction_line && map_line3->Label()!= label_pair_step_edge){    
          if (map_line3->Contains(map_point->NumberRef())) {
      //      pairlines.erase(map_line);
            seg_laser_points.ErasePoints();
            seg_laser_points = tot_largest_segments_points.SelectTagValue(SegmentNumberTag, map_point->Number());
            if (seg_laser_points[0].Attribute(LabelTag)==1000) {
//                  total_tilted_points.AddPoints(seg_laser_points);
                    // these loose segments are most probable dormers, for the moment remove them
                  tot_largest_segments_points.RemoveTaggedPoints(map_point->Number(), SegmentNumberTag);
                  graph_lines.erase(map_line3);
                  map_line3--;
                  i++;
  //          map_line->Label() = label_loose_flat_segment;
             }
           }
          }
         }
        }
       }
   //    fprintf(statfile, "%d dormers removed\n", i);
       graph_points.erase(graph_points.begin(), graph_points.end());
       for (map_line3=graph_lines.begin(); map_line3!=graph_lines.end(); map_line3++) {
             for (node2=map_line3->begin(); node2!=map_line3->end(); node2++) {
               graph_points.push_back(*(pairpoints.PointIterator(*node2)));
             }
       } 
      }
     
     
     if (make_graph && graph_points.size()>1){
       sel_laser_points.ErasePoints();
       if (add_intsectlines)       intsects.RemoveDoublePoints(intsectlines, 0.01);
       if(!selcornerpoints.empty()) selcornerpoints.erase(selcornerpoints.begin(), selcornerpoints.end()); 
       origintsectpoints.insert(origintsectpoints.end(), selridgepoints.begin(), selridgepoints.end()); //store original location of intersectionlines
       origintsectlines.insert(origintsectlines.end(), selridgelines.begin(), selridgelines.end());
       if (do_normal){
          sel_laser_points.AddPoints(largest_segments_points);
          }
       else{            
         sel_laser_points.AddTaggedPoints(laser_points, map_line->Number(),
                                           PolygonNumberTag);
         }             
        graph_points.RemoveDoublePoints(graph_lines, 0.01);
        graph_vector.resize(graph_points.size()*graph_points.size(),0);
       for (i=0; i<graph_points.size();i++){
             for (j=0; j<graph_points.size();j++){
               graph_vector[i*graph_points.size() + j] = 0;
             }
        }
        
        //for each graphline (line between two intersecting segments), put label in
        //adjacency maxtrix(actually vector)
        
        for (map_line2=graph_lines.begin(); map_line2!=graph_lines.end(); map_line2++){
             node2=map_line2->begin();
             sn1 = graph_points.PointIterator(*node2)->Number(); node2++;
             sn2 = graph_points.PointIterator(*node2)->Number();
             for (i=0; i<graph_points.size();i++){ //find point number in location in graph_points
                 if (graph_points[i].Number()==sn1) ii = i;
                 if (graph_points[i].Number()==sn2) jj = i;
             } 
             if (jj<ii) j=jj, jj=ii, ii=j;//swap ii and jj; start in graph with the smallest
             
             graph_vector[ii*graph_points.size() + jj] = map_line2->Label(); //put label in adjacency matrix
             graph_vector[jj*graph_points.size() + ii] = map_line2->Label();
//             printf("%d-%d %d-%d\n", sn1, sn2, ii, jj);
        }
        
        // for each graph point, put a neighbor in the close_vector
        fprintf(graph_file,"%d ", int (graph_points.size()*(graph_points.size()-1)/2));
        for (i=0; i<graph_points.size();i++){
            close_vector.clear();
            gable_score = 0;
            for (j=0; j<i; j++) fprintf(statfile,"  ");
             for (j=i+1; j<graph_points.size();j++){
               fprintf(statfile,"%d ", graph_vector[i*graph_points.size() + j]);
               fprintf(graph_file,"%d ", graph_vector[i*graph_points.size() + j]);
               pnlseg1 = sel_laser_points.SelectTagValueList(SegmentNumberTag, graph_points[i].Number());
               lab1 = (sel_laser_points[pnlseg1.begin()->Number()]).Attribute(LabelTag);
   //            if (graph_vector[i*graph_points.size() + j]==6){ //if int sect is horizontal, calculate probability gable roof
                  
     //             }
               // if this segment is not flat, look for more tilted hints, put tilted neighbours in close_vector
 //              if (lab1!= 1000){   
                 if (graph_vector[i*graph_points.size() + j]!=label_pair_step_edge && 
                   graph_vector[i*graph_points.size() + j]!=0) {
         //         fprintf(statfile,"(%d,%d)", graph_points[i].Number(), graph_points[j].Number());
                    pnlseg1 = sel_laser_points.SelectTagValueList(SegmentNumberTag, graph_points[j].Number());
                    lab2 = (sel_laser_points[pnlseg1.begin()->Number()]).Attribute(LabelTag);
                    if (lab1!= 1000 || lab2!=1000){ //try, add point if one of the two isn't horizontal
                      close_vector.push_back(j);
                    }
               }
              // }
             }  
             // now look if two neighbors are also neighbors from eachother 
             //(this indicates a corner point, defined by three planes)
             
            if(close_vector.size()>1){
               foundcornerpoint = false;                       
               for (k=0;k!=close_vector.size();k++){
                  for (l=k+1;l!=close_vector.size();l++){
                      kk = close_vector[k];
                      ll = close_vector[l];
                      if (graph_vector[kk*graph_points.size() + ll]!=label_pair_step_edge && 
                             graph_vector[kk*graph_points.size() + ll]!=0) {
                         fprintf(statfile, "(* %d, %d, %d)", graph_points[i].Number(), graph_points[kk].Number(), graph_points[ll].Number());
                         fprintf(statfile, "(* %d, %d, %d)", graph_vector[i*graph_points.size()+kk], graph_vector[kk*graph_points.size()+ll], graph_vector[i*graph_points.size()+ll]);
          //               fprintf(statfile, "(* %d, %d, %d)", i, kk, ll);
                         planecorner1 = sel_laser_points.FitPlane(graph_points[i].Number(), graph_points[i].Number(), SegmentNumberTag);
                         planecorner2 = sel_laser_points.FitPlane(graph_points[kk].Number(), graph_points[kk].Number(), SegmentNumberTag);
                         planecorner3 = sel_laser_points.FitPlane(graph_points[ll].Number(), graph_points[ll].Number(), SegmentNumberTag);
                         if (Intersect2Planes(planecorner1, planecorner2, linecorner)) {
                             if (IntersectLine3DPlane(linecorner, planecorner3, cornerpoint)){
                                 map_points.push_back(cornerpoint);
                                 selcornerpoints.push_back(cornerpoint);
                            //     foundcornerpoint = true;
                             }
                         }       
                      }
                  }
                }
                if (!foundcornerpoint){//no roof corners found, but may be part of a "four plane intersection" (look for circle in graph with four nodes)
                             //check if their is a common neighbour of two neighbours of point i
                  for (k=0;k!=close_vector.size();k++){        
                    for (l=k;l!=close_vector.size();l++){
                      if (l!=k){  
                      kk = close_vector[k];
                      ll = close_vector[l];
                      if (graph_vector[kk*graph_points.size() + ll]==0){
                      for (j=i;j<graph_points.size(); j++){
                        if (j!=i){ // not including i ofcourse // 
                          if (graph_vector[kk*graph_points.size()+j]!=label_pair_step_edge &&
                              graph_vector[kk*graph_points.size()+j]!=0 &&
                              graph_vector[ll*graph_points.size()+j]!=label_pair_step_edge &&
                              graph_vector[ll*graph_points.size()+j]!=0 &&
                              graph_vector[i*graph_points.size()+j]==0 && // i not connected to j 
                              graph_vector[kk*graph_points.size()+ll]==0){ // kk not connected to ll 
                              selcornerpoints_four.erase(selcornerpoints_four.begin(), selcornerpoints_four.end());
                              four_planes_points.AddTaggedPoints(sel_laser_points, graph_points[i].Number(), SegmentNumberTag);
                              four_planes_points.AddTaggedPoints(sel_laser_points, graph_points[kk].Number(), SegmentNumberTag);
                              four_planes_points.AddTaggedPoints(sel_laser_points, graph_points[ll].Number(), SegmentNumberTag);
                              four_planes_points.AddTaggedPoints(sel_laser_points, graph_points[j].Number(), SegmentNumberTag);
                              fprintf(statfile, "found four planes: %d, %d, %d, %d", graph_points[i].Number(), graph_points[kk].Number(), graph_points[ll].Number(), graph_points[j].Number());
                              //now let us try to make these four planes intersect
                              planecorner1 = sel_laser_points.FitPlane(graph_points[i].Number(), graph_points[i].Number(), SegmentNumberTag);
                              planecorner2 = sel_laser_points.FitPlane(graph_points[kk].Number(), graph_points[kk].Number(), SegmentNumberTag);
                              planecorner3 = sel_laser_points.FitPlane(graph_points[ll].Number(), graph_points[ll].Number(), SegmentNumberTag);
                              if (Intersect2Planes(planecorner1, planecorner2, linecorner)) {
                                 if (IntersectLine3DPlane(linecorner, planecorner3, cornerpoint)){
                                     artpoint = LaserPoint(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z(), 1, 1);
                                     if (artpoint.InsidePolygon(map_points,
                                     map_line->LineTopologyReference())){
                                                                         selcornerpoints_four.push_back(cornerpoint);
                     //                                                    selcornerpoints.push_back(cornerpoint);
                                                                         }
                                 }
                              }
                              planecorner1 = sel_laser_points.FitPlane(graph_points[j].Number(), graph_points[j].Number(), SegmentNumberTag);
                              planecorner2 = sel_laser_points.FitPlane(graph_points[kk].Number(), graph_points[kk].Number(), SegmentNumberTag);
                              planecorner3 = sel_laser_points.FitPlane(graph_points[ll].Number(), graph_points[ll].Number(), SegmentNumberTag);
                              if (Intersect2Planes(planecorner1, planecorner2, linecorner)) {
                                 if (IntersectLine3DPlane(linecorner, planecorner3, cornerpoint)){
                                     artpoint = LaserPoint(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z(), 1, 1);
                                     if (artpoint.InsidePolygon(map_points,
                                     map_line->LineTopologyReference())) {
                                                                         selcornerpoints_four.push_back(cornerpoint);
                     //                                                    selcornerpoints.push_back(cornerpoint);
                                                                         }
                                 }
                              }       
                              planecorner1 = sel_laser_points.FitPlane(graph_points[i].Number(), graph_points[i].Number(), SegmentNumberTag);
                              planecorner2 = sel_laser_points.FitPlane(graph_points[kk].Number(), graph_points[kk].Number(), SegmentNumberTag);
                              planecorner3 = sel_laser_points.FitPlane(graph_points[j].Number(), graph_points[j].Number(), SegmentNumberTag);
                              if (Intersect2Planes(planecorner1, planecorner2, linecorner)) {
                                 if (IntersectLine3DPlane(linecorner, planecorner3, cornerpoint)){
                                     artpoint = LaserPoint(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z(), 1, 1);
                                     if (artpoint.InsidePolygon(map_points,
                                     map_line->LineTopologyReference())) {
                                                                         selcornerpoints_four.push_back(cornerpoint);
                        //                                                 selcornerpoints.push_back(cornerpoint);
                                                                         }
                                 }
                              }
                              planecorner1 = sel_laser_points.FitPlane(graph_points[i].Number(), graph_points[i].Number(), SegmentNumberTag);
                              planecorner2 = sel_laser_points.FitPlane(graph_points[ll].Number(), graph_points[ll].Number(), SegmentNumberTag);
                              planecorner3 = sel_laser_points.FitPlane(graph_points[j].Number(), graph_points[j].Number(), SegmentNumberTag);
                              if (Intersect2Planes(planecorner1, planecorner2, linecorner)) {
                                 if (IntersectLine3DPlane(linecorner, planecorner3, cornerpoint)){
                                     artpoint = LaserPoint(cornerpoint.X(), cornerpoint.Y(), cornerpoint.Z(), 1, 1);
                                     if (artpoint.InsidePolygon(map_points,
                                     map_line->LineTopologyReference())) {
                                                                         selcornerpoints_four.push_back(cornerpoint);
                          //                                               selcornerpoints.push_back(cornerpoint);
                                                                         }
                                 }
                              }
                              printf("size of selcornerpoints: %d\n", selcornerpoints_four.size());
                              
/*                              if (selcornerpoints_four[0].Z() < selcornerpoints_four[2].Z()){
  //                                selcornerpoints.push_back(selcornerpoints_four[0]);
  //                                selcornerpoints.push_back(selcornerpoints_four[1]);
                                  objpseg1 = selcornerpoints_four[0];
                                  objpseg2 = selcornerpoints_four[1];
                                  }
                              else {
    //                               selcornerpoints.push_back(selcornerpoints_four[2]);
    //                               selcornerpoints.push_back(selcornerpoints_four[3]);
                                   objpseg1 = selcornerpoints_four[2];
                                   objpseg2 = selcornerpoints_four[3];
                                   }
                              edgepointnumber++;
                              objpseg1 = ObjectPoint(objpseg1.X(), objpseg1.Y(), objpseg1.Z(), edgepointnumber, 0,0,0,0,0,0);
                              edgepointnumber++;
                              objpseg2 = ObjectPoint(objpseg2.X(), objpseg2.Y(), objpseg2.Z(), edgepointnumber, 0,0,0,0,0,0);                                  
              //                selridgepoints_four.push_back(objpseg1);
              //                selridgepoints_four.push_back(objpseg2);
             //                 line_number4++;     
             //                 pairline = LineTopology(line_number4, 1, edgepointnumber-1, edgepointnumber);
             //                 selridgelines_four.push_back(pairline);  
//                              edgepointnumber++, line_number4++;
                              map_points.push_back(objpseg1);                                  
                              map_points.push_back(objpseg2);                                  
  */                         }
                         }
                       }
                       }
                       }
                     }
                  }
               }
            }
            fprintf(statfile,"\n");
            printf("\n");
            // now snap the ridgelines to the cornerpoints
            
            for (map_point = selcornerpoints.begin(); map_point!=selcornerpoints.end();map_point++){
                for (map_line2 = selridgelines.begin();map_line2!=selridgelines.end();map_line2++){
                    node2=map_line2->begin();
                    pos1 = selridgepoints.PointIterator(*node2)->Position3DRef(); node2++;
                    pos2 = selridgepoints.PointIterator(*node2)->Position3DRef();
                    linecorner = Line3D(pos1, pos2);
                    if (linecorner.PointOnLine(map_point->Position3DRef(), 0.01)){
                       if ((pos1 - map_point->Position3DRef()).Length()<(pos2 - map_point->Position3DRef()).Length()){
                           if ((pos1 - map_point->Position3DRef()).Length()<2) node2--, selridgepoints.PointIterator(*node2)->Position3DRef() = map_point->Position3DRef();
                           }
                       else {
                            if ((pos2 - map_point->Position3DRef()).Length()<2) selridgepoints.PointIterator(*node2)->Position3DRef() = map_point->Position3DRef();//pos2 = map_point->Position3DRef();
                            } 
                    }
/*                   countingintsect++;
                   pn1 = PointNumber(countingintsect);                        
                   countingintsect++;
                   beginp = ObjectPoint(pos1, pn1, cov3d);
                   pn2 = PointNumber(countingintsect);
                   endp = ObjectPoint(pos2, pn2, cov3d);
                   intsect = LineTopology(line_number3, 1, pn1, pn2);
                   line_number3++;
                   intsects.push_back(beginp);
                   intsects.push_back(endp);
                   intsect.Label() = 9;
                   intsectlines.push_back(intsect); 
  */              }
             }
             
        }
        shape_points.insert(shape_points.end(), selridgepoints.begin(),selridgepoints.end());
        shapelines.insert(shapelines.end(), selridgelines.begin(),selridgelines.end());
  //      selridgepoints.RemoveDoublePoints(selridgelines, 0.01);
if (add_intsectlines){
        intsectlines.insert(intsectlines.end(),selridgelines.begin(),selridgelines.end());
        intsects.insert(intsects.end(),selridgepoints.begin(),selridgepoints.end());
        }
        selridgepoints_four.RemoveDoublePoints(selridgelines_four, 0.01);
//        intsectlines.insert(intsectlines.end(),selridgelines_four.begin(),selridgelines_four.end());
//        intsects.insert(intsects.end(),selridgepoints_four.begin(),selridgepoints_four.end());
        
//        fprintf(statfile, "size of adjacency graph: %8d (1st element: %4.2f)\n", graph.Nrows()*graph.Ncols(), graph(i,j));
        fprintf(statfile, "size of adjacency graph: %8d (last element: %d)\n", graph_vector.size(), graph_vector[i*j-1]);
                     }
     if (do_contour){
       sel_laser_points.ErasePoints();                                                    
       if (do_normal){
          sel_laser_points.AddPoints(largest_segments_points);
          }
          else{            
      sel_laser_points.AddTaggedPoints(laser_points, map_line->Number(),
                                         PolygonNumberTag);
        }
         segment_numbers = sel_laser_points.AttributeValues(SegmentNumberTag);
//        printf("# of laser segments: %d\n", segment_numbers.size());
        if(!segments.empty()) segments.erase(segments.begin(), segments.end());
        // Determine the points of all segments
        for (segment_number=segment_numbers.begin();
             segment_number!=segment_numbers.end(); segment_number++) {
             segments.push_back(sel_laser_points.TaggedPointNumberList(SegmentNumberTag, *segment_number));
            }  
       // Determine the segment contours
       
       for (segment=segments.begin(), segment_number=segment_numbers.begin();
            segment!=segments.end(); segment++, segment_number++) {
            seg_laser_points.ErasePoints();
            seg_laser_points = sel_laser_points.SelectTagValue(SegmentNumberTag, *segment_number);
            // Create the TIN edges
    //        printf("0 segment %d\n", *segment_number);
            seg_laser_points.DeriveTIN();
             //  way to calculate convex hull
            component.Erase();
            edges.Erase();
            edges.Derive(seg_laser_points.TINReference());
            seg_laser_points.Write(laser_output, false);
            for (int i=0; i<seg_laser_points.size(); i++) component.push_back(PointNumber(i));
//            contour = seg_laser_points.DeriveContour(1, component, edges, false, LabelTag);
            contour = seg_laser_points.DeriveContour(1, component, 1.0, 1.0, edges);
         //   contour.SetAttribute(LineLabelTag, *segment_number);
            if(!segment_contour_points.empty()) segment_contour_points.erase(segment_contour_points.begin(), segment_contour_points.end());
            if(!temp_segment_contours.empty()) temp_segment_contours.erase(temp_segment_contours.begin(), temp_segment_contours.end());
            segment_contour_points = seg_laser_points.ConstructObjectPoints();
                if (contour.size()>0){
            
                if (!contour.IsClosed()) contour.push_back(*(contour.begin()));
                   line_number++;
                   next_pnr = all_segment_contour_points.HighestPointNumber().Number()+1;
                   contour.Label() = 2;       
                   temp_segment_contours.push_back(contour);   
                   temp_segment_contours.ReNumber(segment_contour_points, next_pnr, line_number);     
                   segment_contours.insert(segment_contours.end(), temp_segment_contours.begin(), temp_segment_contours.end());
                   all_segment_contour_points.insert(all_segment_contour_points.end(), 
                                             segment_contour_points.begin(), 
                                             segment_contour_points.end());
                   } 
   /*           //begin test enclosing
              seg_laser_points.DeriveTIN();
              edges.Erase();
              edges.Derive(seg_laser_points.TINReference());
              if(!segment_contour_points.empty()) segment_contour_points.erase(segment_contour_points.begin(), segment_contour_points.end());
              if(!temp_segment_contours.empty()) temp_segment_contours.erase(temp_segment_contours.begin(), temp_segment_contours.end());
              seg_laser_points.RemoveLongEdges(edges, 2, true);
              printf("size: %d ", seg_laser_points.size());
              seg_laser_points.Write(laser_output, false);
              found_enclosing = seg_laser_points.EnclosingPolygon(2, segment_contour_points, one_contour);
              if (found_enclosing){
                next_pnr = all_segment_contour_points.HighestPointNumber().Number()+1; 
                calc_poly=false;
                for (map_line2 = one_contour.begin(); map_line2!=one_contour.end(); map_line2++){
                  contour = *map_line2;
              //    printf("Contoursize: %d\n", contour.size());
                  if (contour.size()>0){
                  if (!contour.IsClosed()) contour.push_back(*(contour.begin()));
                     line_number++;
                     temp_segment_contours.push_back(contour);   
                     temp_segment_contours.ReNumber(segment_contour_points, next_pnr, line_number);     
                     segment_contours.insert(segment_contours.end(), temp_segment_contours.begin(), temp_segment_contours.end());
                     all_segment_contour_points.insert(all_segment_contour_points.end(), 
                                               segment_contour_points.begin(), 
                                               segment_contour_points.end());
                     calc_poly=true;
                   }
                 }
              }
   */           //end test enclosing            
         }
         }
     if (do_enclosingpolygon){   
         sel_laser_points.ErasePoints();                                                    
    //     if (do_normal){
    //        sel_laser_points.AddPoints(largest_segments_points);
    //       }
    //     else{            
            sel_laser_points.AddTaggedPoints(laser_points, map_line->Number(),
                                         PolygonNumberTag);
    //      }
          
          if (sel_laser_points.size()>10){
            sel_laser_points.DeriveTIN();
            sel_laser_points.Write(laser_output, false);
            if(!segment_contour_points.empty()) segment_contour_points.erase(segment_contour_points.begin(), segment_contour_points.end());
            if(!temp_segment_contours.empty()) temp_segment_contours.erase(temp_segment_contours.begin(), temp_segment_contours.end());
//            found_enclosing = sel_laser_points.EnclosingPolygon(20, segment_contour_points, one_contour);
                        
            if (found_enclosing){
              printf("2 size of one_contour: %d\n", one_contour.size());
              next_pnr = all_segment_contour_points.HighestPointNumber().Number()+1; 
              calc_poly=false;
              for (map_line2 = one_contour.begin(); map_line2!=one_contour.end(); map_line2++){
                contour = *map_line2;
                printf("Contoursize: %d\n", contour.size());
                if (contour.size()>0){
                if (!contour.IsClosed()) contour.push_back(*(contour.begin()));
                   line_number++;
                   temp_segment_contours.push_back(contour);   
                   temp_segment_contours.ReNumber(segment_contour_points, next_pnr, line_number);     
                   segment_contours.insert(segment_contours.end(), temp_segment_contours.begin(), temp_segment_contours.end());
                   all_segment_contour_points.insert(all_segment_contour_points.end(), 
                                             segment_contour_points.begin(), 
                                             segment_contour_points.end());
                   calc_poly=true;
                }
            }
          
           }
           if (!calc_poly){
             found_rect = sel_laser_points.EnclosingRectangle(0.5, segment_contour_points, contour);
             if (found_rect){
                            next_pnr = all_segment_contour_points.HighestPointNumber().Number()+1; 
                            printf("Foundrect Contoursize: %d\n", contour.size());
                            if (contour.size()>0){
                                if (!contour.IsClosed()) contour.push_back(*(contour.begin()));
                                line_number++;
                                temp_segment_contours.push_back(contour);   
                                temp_segment_contours.ReNumber(segment_contour_points, next_pnr, line_number);     
                                segment_contours.insert(segment_contours.end(), temp_segment_contours.begin(), temp_segment_contours.end());
                                all_segment_contour_points.insert(all_segment_contour_points.end(), 
                                             segment_contour_points.begin(), 
                                             segment_contour_points.end());
                } 
                             }
             }
           }
         }
     if (do_intersection){
     segment_numbers = sel_laser_points.AttributeValues(SegmentNumberTag);
     for (segment_number=segment_numbers.begin();segment_number!=segment_numbers.end();segment_number++){
          if (sel_laser_points.TaggedPointNumberList(SegmentNumberTag, *segment_number).size()<4) {
              segment_numbers.erase(segment_number);
  //            sel_laser_points.RemoveTaggedPoints(*segment_number, SegmentNumberTag);
              segment_number--;
              }
     }
     if(!temp_segment_contours.empty()) temp_segment_contours.erase(temp_segment_contours.begin(), temp_segment_contours.end());
     if(!segments.empty()) segments.erase(segments.begin(), segments.end());
     if(!planes.empty()) planes.erase(planes.begin(), planes.end());
        // Determine the points of all segments
        for (segment_number=segment_numbers.begin();
             segment_number!=segment_numbers.end(); segment_number++) {
             segments.push_back(sel_laser_points.TaggedPointNumberList(SegmentNumberTag, *segment_number));
             printf("segment size: %d (%d)\n", sel_laser_points.TaggedPointNumberList(SegmentNumberTag, *segment_number).size(), *segment_number);
            }    
     sel_laser_points.DeriveTIN();
     edges.Erase();
     edges.Derive(sel_laser_points.TINReference());
     puntnr = 0;
     // Determine the segment contours                         
     for (segment=segments.begin(), segment_number=segment_numbers.begin();
           segment!=segments.end(); segment++, segment_number++) {
           printf("segment number: %d\n", *segment_number);
           temp_segment_contours.push_back(sel_laser_points.DeriveContour(*segment_number, *segment,
                                             edges, true, SegmentNumberTag));
        }
        // Fit planes to the segments
        for (segment=segments.begin(), segment_number=segment_numbers.begin();
              segment!=segments.end(); segment++, segment_number++) {
                planes.push_back(sel_laser_points.FitPlane(*segment, *segment_number));
             }
        // Intersect segments
       for (segment_contour=temp_segment_contours.begin(), plane1=planes.begin(),
               segment=segments.begin();
               segment_contour!=temp_segment_contours.end();
               segment_contour++, plane1++, segment++) {
   //            for (segment_contour2=segment_contour+1, plane2=plane1+1, segment2=segment+1;
            for (segment_contour2=temp_segment_contours.begin(), plane2=planes.begin(), segment2=segments.begin();
                    segment_contour2!=temp_segment_contours.end();
                    segment_contour2++, plane2++, segment2++) {
                    // TODO: Maximum distance between point and intersection line to be transferred
                    if (sel_laser_points.IntersectFaces(*segment_contour, *segment_contour2, *plane1, *plane2,
                          0.8, pos1, pos2)) { //was 0.8
                          begin_intp = LaserPoint(pos1.GetX(), pos1.GetY(), pos1.GetZ());
                          begin_intp.SetAttribute(LabelTag, 1);
                          end_intp = LaserPoint(pos2.GetX(), pos2.GetY(), pos2.GetZ());
                          end_intp.SetAttribute(LabelTag, 1);
                          intersection_points.push_back(begin_intp);
                          intersection_points.push_back(end_intp);
                          countingintsect++;
                          pn1 = PointNumber(countingintsect);                        
                          countingintsect++;
                          beginp = ObjectPoint(pos1, pn1, cov3d);
                          pn2 = PointNumber(countingintsect);
                          endp = ObjectPoint(pos2, pn2, cov3d);
                          line_number2++;
                          intsect = LineTopology(line_number2, 1, pn1, pn2);
                          if (add_intsectlines){   intsects.push_back(beginp);
                            intsects.push_back(endp);
                            intsectlines.push_back(intsect);
                          }                          // make linetopology by making pointnumbers and objectpoints 110607
                          
                    }
               }
        }                
     }    
     if (produce_building_blocks){
        fixed_floor_height = 0;
        printf("start building block %d\n", map_line->Number());
        sel_laser_points.ErasePoints();                                                                              
        if (do_normal){
          sel_laser_points.AddPoints(largest_segments_points);
          }
          else{            
           sel_laser_points.AddTaggedPoints(laser_points, map_line->Number(),
                                         PolygonNumberTag);
          }
        edges.Erase();
        sel_laser_points.DeriveTIN();
        edges.Derive(sel_laser_points.TINReference());

        segment_numbers = sel_laser_points.AttributeValues(SegmentNumberTag);
        sel_laser_points.DeriveDataBounds(0);                            
        mean = sel_laser_points.Mean()[2];     
        height = sel_laser_points.DataBounds().Maximum().Z();
 //       fprintf(statfile, "%6d %6d %8.2f %8.2f \n",
 //              map_line->Number(), segment_numbers.size(), height, mean);
 //       sel_laser_points.SetAttribute(SegmentNumberTag, segment_numbers.size());
        residual_points.ErasePoints();
/*        for (laser_point = sel_laser_points.begin(); laser_point!=sel_laser_points.end(); laser_point++){
             if (abs(laser_point->Z() -mean) > 5) {
                                      laser_point->Residual()=1.0;
                                      residual_points.push_back(*laser_point);
                                      }
             else {
                  if (abs(laser_point->Z() -mean) > 3) laser_point->Residual()=0.4;
                  else laser_point->Residual() = 0.1;
                  }
        } 
  */      building_laser_points.AddPoints(sel_laser_points);
        segment_numbers = residual_points.AttributeValues(SegmentNumberTag);
        if(!temp_segment_contours.empty()) temp_segment_contours.erase(temp_segment_contours.begin(), temp_segment_contours.end());
        if(!segments.empty()) segments.erase(segments.begin(), segments.end());
   
        // Determine the points of all segments
 /*       for (segment_number=segment_numbers.begin();
             segment_number!=segment_numbers.end(); segment_number++) {
             segments.push_back(residual_points.TaggedPointNumberList(SegmentNumberTag, *segment_number));
 //            printf("segment size: %d (%d)\n", residual_points.TaggedPointNumberList(SegmentNumberTag, *segment_number).size(), *segment_number);
            }    
         residual_points.DeriveTIN();
         edges.Erase();
         edges.Derive(residual_points.TINReference());
         puntnr = 0;
         // Determine the segment contours                         
         for (segment=segments.begin(), segment_number=segment_numbers.begin();
              segment!=segments.end(); segment++, segment_number++) {
              printf("segment number: %d\n", *segment_number);
              temp_segment_contours.push_back(residual_points.DeriveContour(*segment_number, *segment,
                                              edges, true, SegmentNumberTag));
        }
        segment_contour_points = residual_points.ConstructObjectPoints();

        line_number++;
        next_pnr = all_segment_contour_points.HighestPointNumber().Number()+1; 
        temp_segment_contours.ReNumber(segment_contour_points, next_pnr, line_number);     
        segment_contours.insert(segment_contours.end(), temp_segment_contours.begin(), temp_segment_contours.end());
        all_segment_contour_points.insert(all_segment_contour_points.end(), 
                                             segment_contour_points.begin(), 
                                             segment_contour_points.end());                                     
        
   //     printf("%d, %4.2f\n", sel_laser_points.size(), height);                          
   */     
        // produce tin model with mean height...
        
        if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end()); 
        if(!max_map_points.empty()) max_map_points.erase(max_map_points.begin(),max_map_points.end()); 
        for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
             this_point = map_points.PointIterator(*node2);
             new_point = *this_point;             
             new_point.Z() = mean;//eave_height;//mean - (height - mean);//height;
             if (do_normal && local_eave_height!=fixed_eave_height) new_point.Z() = local_eave_height;
/*             nearest_laser_point = sel_laser_points.NearestPoint(new_point.Position3DRef(),
                                                     edges, true);
             nearlaspoint = sel_laser_points[nearest_laser_point];
             min_dist = 100;
             for (laser_point=sel_laser_points.begin(); laser_point!=sel_laser_points.end(); laser_point++){
                dist = (laser_point->vect2D()-new_point.vect2D()).Length();   
                if (dist < min_dist) {
                           min_dist = dist;
                           nearlaspoint2 = *laser_point;
                    }
             }
      //       printf("Dist = %5.2f\n", (nearlaspoint.vect2D()-nearlaspoint2.vect2D()).Length());    
             temp_height_points.ErasePoints();
             temp_height_points = sel_laser_points.SelectTagValue(SegmentNumberTag, nearlaspoint2.Attribute(SegmentNumberTag));
             temp_height_points.DeriveDataBounds(0);
             local_eave_height = temp_height_points.DataBounds().Minimum().Z();
             new_point.Z() = local_eave_height;
  */           new_point.Number() = number_offset2;
             number_offset2++;
             node2->Number()=new_point.Number();
//             height_point = ObjectPoint(new_point.X(), new_point.Y(), new_point.Z(), new_point.Number(),
  //                        0 , 0 , height, 0, 0, 0);
             max_point = new_point;
             max_point.Z() = height;
             sel_map_points.push_back(new_point);
             max_map_points.push_back(max_point);
          }
  /*        if (do_normal){ // add information from intersection/normal area...
            for (map_point=shape_points.begin(); map_point!=shape_points.end(); map_point++) {
                new_point = *map_point; 
                new_point.Number() = number_offset2;
                number_offset2++;
                sel_map_points.push_back(new_point);
                max_map_points.push_back(new_point);
                }
          }
    */      one_map_line.erase(one_map_line.begin(), one_map_line.end());
          one_map_line.push_back(*map_line);
          max_map_line.erase(max_map_line.begin(), max_map_line.end());
          max_map_line.push_back(*map_line);
          sel_map_points.RemoveDoublePoints(one_map_line, 0.1);
          max_map_points.RemoveDoublePoints(max_map_line, 0.1);
          if (sel_map_points.size()>3){ 
            one_map_line.ReNumber(sel_map_points, 0, 0);
            max_map_line.ReNumber(max_map_points, 0 ,0);
            tin = sel_map_points.Triangulate(one_map_line);
            map_tin_lines = LineTopologies(tin);
            tin.Erase();  
            tin = max_map_points.Triangulate(max_map_line);
            max_tin_lines = LineTopologies(tin);
            tin.Erase();  
            number_offset = sel_map_points.HighestPointNumber().Number()+1;
            sel_map_points.DuplicateWithFixedZ(fixed_floor_height, number_offset);
            max_map_points.DuplicateWithFixedZ(fixed_floor_height, number_offset);
            map_tin_lines.AddWalls(one_map_line, number_offset);
            max_tin_lines.AddWalls(max_map_line, number_offset);
  // map_tin_lines.insert(map_tin_lines.end(), one_map_line.begin(), one_map_line.end());
//     max_tin_lines.insert(max_tin_lines.end(), max_map_line.begin(), max_map_line.end());
            if (!all_tin_building_points.empty()){
              map_tin_lines.ReNumber(sel_map_points, (all_tin_building_points.end()-1)->Number()+1, (all_tin_building_lines.end()-1)->Number()+1);
              max_tin_lines.ReNumber(max_map_points, (all_tin_building_points.end()-1)->Number()+1, (all_tin_building_lines.end()-1)->Number()+1);
              }
            all_tin_building_lines.insert(all_tin_building_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
            all_tin_building_points.insert(all_tin_building_points.end(), sel_map_points.begin(), sel_map_points.end());                              
            all_tin_building_maxlines.insert(all_tin_building_maxlines.end(), max_tin_lines.begin(), max_tin_lines.end());
            all_tin_building_maxpoints.insert(all_tin_building_maxpoints.end(), max_map_points.begin(), max_map_points.end());        
          }
        
  /*      // produce tin model with maximum height...
        if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end()); 
        for (node2=map_line2->begin(); node2!=map_line2->end(); node2++) {
             this_point = map_points.PointIterator(*node2);
             new_point = *this_point;
             new_point.Z() = height;
             new_point.Number() = rem_number_offset2;
             rem_number_offset2++;
             node2->Number()=new_point.Number();
       //      height_point = ObjectPoint(new_point.X(), new_point.Y(), new_point.Z(), new_point.Number(),
       //                   0 , 0 , height, 0, 0, 0);
             sel_map_points.push_back(new_point);
          }
          one_map_line.erase(one_map_line.begin(), one_map_line.end());
          one_map_line.push_back(*map_line2);
          sel_map_points.RemoveDoublePoints(one_map_line, 0.1);
          if (sel_map_points.size()>3){ 
            one_map_line.ReNumber(sel_map_points, 0, 0);
            tin = sel_map_points.Triangulate(one_map_line);
            map_tin_lines = LineTopologies(tin);
            number_offset = sel_map_points.HighestPointNumber().Number()+1;
            sel_map_points.DuplicateWithFixedZ(fixed_floor_height, number_offset);
            map_tin_lines.AddWalls(one_map_line, number_offset);
            tin.Erase();  
            if (!all_tin_building_maxpoints.empty())
              map_tin_lines.ReNumber(sel_map_points, (all_tin_building_maxpoints.end()-1)->Number()+1, (all_tin_building_maxlines.end()-1)->Number()+1);
            all_tin_building_maxlines.insert(all_tin_building_maxlines.end(), map_tin_lines.begin(), map_tin_lines.end());
            all_tin_building_maxpoints.insert(all_tin_building_maxpoints.end(), sel_map_points.begin(), sel_map_points.end());                              
          }
*/      }
       if (recolorsegments){
         sel_laser_points.ErasePoints();                                                    
         if (do_normal){
            sel_laser_points.AddPoints(largest_segments_points);
            }
         else{            
           sel_laser_points.AddTaggedPoints(laser_points, map_line->Number(),
                                           PolygonNumberTag);
         }
         segment_numbers = sel_laser_points.AttributeValues(SegmentNumberTag);
         if(!segments.empty()) segments.erase(segments.begin(), segments.end());
         for (segment_number=segment_numbers.begin();
             segment_number!=segment_numbers.end(); segment_number++) {
             segments.push_back(sel_laser_points.TaggedPointNumberList(SegmentNumberTag, *segment_number));
            }  
         for (segment=segments.begin(), segment_number=segment_numbers.begin();
              segment!=segments.end(); segment++, segment_number++) {
                seg_laser_points.ErasePoints();
                seg_laser_points = sel_laser_points.SelectTagValue(SegmentNumberTag, *segment_number);
                colour_values = seg_laser_points.AttributeValues(ColourTag);
                colour_red.erase(colour_red.begin(), colour_red.end());
                colour_green.erase(colour_green.begin(), colour_green.end());
                colour_blue.erase(colour_blue.begin(), colour_blue.end());

                for (laser_point = seg_laser_points.begin(); laser_point!=seg_laser_points.end(); laser_point++){
                    median_colour = laser_point->Attribute(ColourTag);
                    median_colour_red = median_colour >> 16;
                    median_colour_green = ((median_colour >> 8) - ((median_colour>>16)<<8));
                    median_colour_blue = median_colour-((median_colour>>8)<<8);
                    
                    colour_red.push_back(median_colour_red);
                    colour_green.push_back(median_colour_green);
                    colour_blue.push_back(median_colour_blue);
                    }
                sort(colour_red.begin(), colour_red.end());
                sort(colour_green.begin(), colour_green.end());
                sort(colour_blue.begin(), colour_blue.end());
                median_colour_red = colour_red[int(colour_red.size()/2)];
                median_colour_green = colour_green[int(colour_green.size()/2)];
                median_colour_blue = colour_blue[int(colour_blue.size()/2)];
                
                seg_laser_points.SetAttribute(ColourTag, (median_colour_red << 16) + (median_colour_green << 8) + median_colour_blue);
                total_mixed_points.AddPoints(seg_laser_points);
         }
         
         }                     
                    
     if (calc_height){
     for (node2=map_line->begin(); node2!=map_line->end(); node2++) {
             this_point = map_points.PointIterator(*node2);
             new_point = *this_point;
             temp_height_points.ErasePoints();  
             for (laser_point=sel_laser_points.begin(); laser_point!=sel_laser_points.end(); laser_point++){
                dist = (laser_point->vect2D()-new_point.vect2D()).Length();   
                if (dist < nbh_radius) {
                           temp_height_points.push_back(*laser_point);
                    }
           }

            dominant_segment = temp_height_points.MostFrequentAttributeValue(SegmentNumberTag, count);
            if (count>6){
            plane = sel_laser_points.FitPlane(dominant_segment, dominant_segment,
                                            SegmentNumberTag);
                                            
//            calc_points = temp_height_points.SelectTagValue(SegmentNumberTag, dominant_segment);
//            calc_points.DeriveDataBounds(0);
//            height = calc_points.DataBounds().Maximum().Z();
             if (plane.IsVertical(3.14*15/180)){
                 printf("almost vertical segment removed...\n");
                 temp_height_points.RemoveTaggedPoints(dominant_segment, SegmentNumberTag);
                 dominant_segment = temp_height_points.MostFrequentAttributeValue(SegmentNumberTag, count);                                                
                 if (count>6){
                    plane = sel_laser_points.FitPlane(dominant_segment, dominant_segment,
                                            SegmentNumberTag);
                    }                                     
             }      
            if (!plane.IsVertical(PI*15/180)) height = plane.Z_At(new_point.X(), new_point.Y(), &success);
            }
            else {height = -100;}
             new_point.Z() = height;
             countingintsect++;
             new_point.Number() = countingintsect;//number_offset;           
//             number_offset++;
             if (node2 == map_line->end()-1) node2->Number() = (map_line->begin())->Number();
             else {node2->Number()=new_point.Number();}
             buildingpoint = LaserPoint(new_point.X(), new_point.Y(), new_point.Z());
             buildingpoint.SetAttribute(LabelTag, 2);
             intersection_points.push_back(buildingpoint);
             height_points.push_back(new_point);
//             sel_map_points.push_back(new_point);
      }
      line_number2++;
      height_line = *map_line;
      height_line.SetAttribute(LineLabelTag, line_number2);
      height_lines.push_back(height_line);
      }
   }
   fprintf(graph_file,"\n");

}
//intsects.RemoveDoublePoints(intsectlines, 1);
height_points.RemoveDoublePoints(height_lines, 0.01);

if (recolorsegments){
                     if (!total_mixed_points.Write(laser_output, false))
                     printf("Error writing laser point file.\n");    
                     return;
                     }


if (calc_height){
 PointNumberList::iterator    node, previous_node, next_node;
 double                       dist1, dist2;
 max_slope = 0.5;
 for (map_point=height_points.begin(); map_point!=height_points.end(); map_point++) {
    if (map_point->Z() == -100.0) {
      // Find a line with this point number
      map_line = height_lines.begin();
      while ((node = map_line->NodeIterator(map_point->Number())) ==
              map_line->end()) map_line++;
      // Find an earlier point
      count = 0;
      do {
        count++;
        node = map_line->PreviousNode(node);
        previous_point = height_points.PointIterator(*node);
      } while (previous_point->Z() == -100.0 && count < map_line->size());
      if (count == map_line->size()) continue; // No point with valid height
      // Find a later point
      node = map_line->NodeIterator(map_point->Number());
      do {
        node = map_line->NextNode(node);
        next_point = height_points.PointIterator(*node);
      } while (next_point->Z() == -100.0);
      // Take a weighted height
      dist1 = (map_point->vect2D() - previous_point->vect2D()).Length();
      dist2 = (map_point->vect2D() - next_point->vect2D()).Length();
      map_point->Z() = (dist1 * next_point->Z() +
                        dist2 * previous_point->Z()) / (dist1 + dist2);
    }
  }  

  printf("done\nWriting output...\n"); 
  height_points.Write(map_points_output);
  height_lines.Write(map_topology_output, false);
}
total_size = overall_flat + overall_tilted + overall_vertical;
printf("distribution: flat: %4.2f, vertical: %4.2f, tilted: %4.2f\n", 
                (100.0*overall_flat)/total_size, (100.0*overall_vertical)/total_size,
                (100.0*overall_tilted)/total_size);                  
all_dormer_points.Write("alldormerpoints.laser", false);
//allnondormerobj.RemoveDoublePoints(allnondormertops, 0.5);
allnondormerobj.Write("allnondormer.objpts");
allnondormertops.Write("allnondormer.top", false);

//return;            
//bool add_intsectlines;
add_intsectlines = false;

//bool add_pairlines;
add_pairlines = true;



if (do_contour){
    if (add_intsectlines){
        line_number++;                  
        next_pnr = all_segment_contour_points.HighestPointNumber().Number()+1;
        intsectlines.ReNumber(intsects, next_pnr, line_number);
        segment_contours.insert(segment_contours.end(), intsectlines.begin(), intsectlines.end());
        all_segment_contour_points.insert(all_segment_contour_points.end(), intsects.begin(), intsects.end());
        line_number=line_number+intsectlines.size();
                  }
    if (add_map_data){
  //      line_number++;
  //      next_pnr = all_segment_contour_points.HighestPointNumber().Number()+1;
  //      map_lines.ReNumber(map_points, next_pnr, line_number);
  //      segment_contours.insert(segment_contours.end(), map_lines.begin(), map_lines.end());
  //      all_segment_contour_points.insert(all_segment_contour_points.end(), map_points.begin(), map_points.end());
                  }
                          
    all_segment_contour_points.Write(map_points_output);
    segment_contours.Write(map_topology_output, false);
 //   dxffile = fopen ("contour_polygons.dxf","w");
//    dxffile = fopen (infofile,"w");
//    all_segment_contour_points.WriteDXFMesh(dxffile, segment_contours, 6, false, true, true);
//    fclose(dxffile);
    }
if (do_enclosingpolygon){

    all_segment_contour_points.Write(map_points_output);
    segment_contours.Write(map_topology_output, false);
    }
    bool walls2dtm = false;
if (produce_building_blocks){
                             
    if (walls2dtm){                         
    bool lowered;
    LaserPoint inside_laser_point;
    LaserPoints keep_building_points;
    LineTopology::iterator  nb_node;
    Position3D   p1, p2, p3;
    next_pnr = 0;
    
    keep_building_points = laser_points;
    laser_points.ErasePoints();
    if (!laser_points.Read("alllevel3segm.laser")) {
    printf("Error reading terrain laser points from file\n");
    exit(0);
    }
    printf("Size before (%d) ", laser_points.size());   
    laser_points.ReduceData(4.0);     
    printf("and after (%d) filtering.\n", laser_points.size());   
   // laser_points.AddPoints(keep_building_points);
    if(!sel_map_points.empty()) sel_map_points.erase(sel_map_points.begin(),sel_map_points.end()); 
    if(!map_tin_lines.empty()) map_tin_lines.erase(map_tin_lines.begin(),map_tin_lines.end());
    tin.Erase();
    for (i=0;i<laser_points.size();i++) {
                sel_map_points.push_back(ObjectPoint(laser_points[i].vect(), next_pnr, Covariance3D()));
                next_pnr ++;
              }
              
     tin = sel_map_points.Triangulate();
     map_tin_lines = LineTopologies(tin);
     tin.Erase();
     all_terrain_tin.insert(all_tin_lines.end(), map_tin_lines.begin(), map_tin_lines.end());
     all_terrain_points.insert(all_tin_points.end(), sel_map_points.begin(), sel_map_points.end());  
     dxffile = fopen ("terrainlevel3.dxf","w");     
     all_terrain_points.WriteDXFMesh(dxffile, all_terrain_tin, 9, false, true, true);
     fclose(dxffile);
     for (map_point=all_tin_building_points.begin(),run_index=0; map_point!=all_tin_building_points.end(); map_point++, run_index++){
         printf("%4.1f\r", (100.0*run_index)/all_tin_building_points.size());
      if (map_point->Z() == fixed_floor_height){
         inside_laser_point.X()=map_point->X();
         inside_laser_point.Y()=map_point->Y();
         inside_laser_point.Z()=map_point->Z();
         lowered = false;
                          
         for (map_line2 = all_terrain_tin.begin(); !lowered, map_line2!= all_terrain_tin.end(); map_line2++){
              if (inside_laser_point.InsidePolygon(all_terrain_points, map_line2->LineTopologyReference())){
                  nb_node=map_line2->begin();
                  p1 = Position3D(*(all_terrain_points.PointIterator(*nb_node))); nb_node++;
                  p2 = Position3D(*(all_terrain_points.PointIterator(*nb_node))); nb_node++;
                  p3 = Position3D(*(all_terrain_points.PointIterator(*nb_node)));
                  plane = Plane(p1, p2, p3);
                  if (plane.Z_At(inside_laser_point.X(), inside_laser_point.Y(), &success) != 0){
                      inside_laser_point.Z() = plane.Z_At(inside_laser_point.X(), inside_laser_point.Y(), &success);
                      map_point->Z() = plane.Z_At(inside_laser_point.X(), inside_laser_point.Y(), &success);//              
  //                    printf(" changed to %5.2f \n", inside_laser_point.Z());          
                      lowered = true;
                  }
               }
           }
      }
      }
    }
    line_number3++;
    next_pnr = intsects.HighestPointNumber().Number()+1;
/*    all_tin_building_lines.ReNumber(all_tin_building_points, next_pnr, line_number3);
    intsectlines.insert(intsectlines.end(), all_tin_building_lines.begin(), all_tin_building_lines.end());
    intsects.insert(intsects.end(), all_tin_building_points.begin(), all_tin_building_points.end());
    intsects.Write(map_points_output);
    intsectlines.Write(map_topology_output, false);                             
*/    all_tin_building_points.Write(map_points_output);
    all_tin_building_lines.Write(map_topology_output, false);
//    dxffile = fopen ("flat_buildings_meanheight.dxf","w");
    //write tin model to dxf file: mean height to layer 9, max height to layer 8
    dxffile = fopen (infofile,"w");
      all_tin_building_points.WriteDXFMesh(dxffile, all_tin_building_lines, 9, false, true, true);
   //   all_tin_building_maxpoints.WriteDXFMesh(dxffile, all_tin_building_maxlines, 8, false, false, true);
    fclose(dxffile);
    return;
    if (!building_laser_points.Write(laser_output, false))
          printf("Error writing laser point file.\n");    
    }
    

//    if (!building_laser_points.Write(laser_output, false))
//        printf("Error writing building laser point file.\n");  
if (do_intersection){
//    if (!all_road_lines.Write(map_topology_output))

    intsects.Write(map_points_output);
    intsectlines.Write(map_topology_output, false);
    }

if (do_normal){           
    
    if (generate_artificial_points){
       if (!artpoints.Write(laser_output, false))
            printf("Error writing laser point file.\n");    
   }                                     
            
    if (!tot_largest_segments_points.Write(laser_output2, false))
        printf("Error writing laser point file.\n");
    if (!do_contour && !do_enclosingpolygon && !produce_building_blocks){        
     if (add_map_data){
        line_number3++;
        if (add_intsectlines && add_pairlines){
          printf("Map lines added to intersection lines and pairlines\n");                   
          next_pnr = intsects.HighestPointNumber().Number()+1;
          sel_map_lines.ReNumber(map_points, next_pnr, line_number3);
          intsectlines.insert(intsectlines.end(), sel_map_lines.begin(), sel_map_lines.end());
          intsects.insert(intsects.end(), map_points.begin(), map_points.end());
//        intsectlines.Densify(intsects, 0.2);
          next_pnr = intsects.HighestPointNumber().Number()+1;
          line_number3 = line_number3 + sel_map_lines.size();
//          pairlines.ReNumber(pairpoints, next_pnr, line_number3);
//          intsectlines.insert(intsectlines.end(), pairlines.begin(), pairlines.end());
//          intsects.insert(intsects.end(), pairpoints.begin(), pairpoints.end());
//          next_pnr = intsects.HighestPointNumber().Number()+1;
//          line_number3 = line_number3 + pairlines.size();
          stepedgelines.ReNumber(stepedgepoints, next_pnr, line_number3);
          intsectlines.insert(intsectlines.end(), stepedgelines.begin(), stepedgelines.end());
          intsects.insert(intsects.end(), stepedgepoints.begin(), stepedgepoints.end());
          intsects.Write(map_points_output);
          intsectlines.Write(map_topology_output, false);
          }
         if (!add_intsectlines && add_pairlines){
             printf("Map lines added to pairlines\n");
             next_pnr = pairpoints.HighestPointNumber().Number()+1;
             sel_map_lines.ReNumber(map_points, next_pnr, line_number3);
      //       pairlines.insert(pairlines.end(), sel_map_lines.begin(), sel_map_lines.end());
      //       pairpoints.insert(pairpoints.end(), map_points.begin(), map_points.end());                   
             pairpoints.RemoveDoublePoints(pairlines, 0.05);
             pairpoints.Write(map_points_output);
             pairlines.Write(map_topology_output, false);
             shape_points.RemoveDoublePoints(shapelines, 0.01);
             shape_points.Write("shapepoints.objpts");
             shapelines.Write("shapepoints.top", false);
             pairpoints.RemoveDoublePoints(graphlines, 0.05);
             graphlines.Write("graphlines.top", false);
             pairpoints.Write("graphlines.objpts");
             stepedgetopologyoutput = pairlines.SelectAttributedLines(LineLabelTag, label_pair_step_edge);
             stepedgetopologyoutput.Write("stepedgetopology.top", false);
             fittedstepedgepoints.Write("fittedstepedgelines.objpts");
             fittedstepedgelines.Write("fittedstepedgelines.top", false);
             stepedgelaserpoints.Write("stepedgelaserpoints.laser", false);
             }
          if (add_intsectlines && !add_pairlines){
             printf("Map lines added to intsectlines\n");
             next_pnr = intsects.HighestPointNumber().Number()+1;
             sel_map_lines.ReNumber(map_points, next_pnr, line_number3);
             intsectlines.insert(intsectlines.end(), sel_map_lines.begin(), sel_map_lines.end());
             intsects.insert(intsects.end(), map_points.begin(), map_points.end());
             next_pnr = intsects.HighestPointNumber().Number()+1;
             line_number3 = line_number3 + sel_map_lines.size();
             stepedgelines.ReNumber(stepedgepoints, next_pnr, line_number3);
             intsectlines.insert(intsectlines.end(), stepedgelines.begin(), stepedgelines.end());
             intsects.insert(intsects.end(), stepedgepoints.begin(), stepedgepoints.end());
             intsects.Write(map_points_output);
             intsectlines.Write(map_topology_output, false);
             return;
             }   
        }
                                        
    pairpoints.Write(map_points_output);
    pairlines.Write(map_topology_output, false);
    origintsectpoints.Write("origintsectlines.objpts");
    origintsectlines.Write("origintsectlines.top", false);
//    intsects.Write(map_points_output);
//    intsectlines.Write(map_topology_output, false);
    if (!total_mixed_points.Write(laser_output, false))
//    if (!four_planes_points.Write(laser_output, false))
           printf("Error writing laser point file.\n");    
    }
    }
   vertical_points.Write("verticalsegments.laser", false);
  fclose(statfile);    
  fclose(graph_file);    
    
//}                                                      
//printf("Number offset: %d\n", number_offset); 
if (bpartition){
   ObjectPoints                   building_points, bmap_points;
   LineTopologies::const_iterator polygon;
   LineTopologies                 polygons;
   LineTopology                   line_segment;
   Line2D                         line;
   Position2D                     pos2D, pos2D2;
   int                            next_linenumber=0, num_points, dist_dim, next_number=0,
                                  best;
   double                         scalar_max, scalar_min, dist_min, dist_max, 
                                  min_dist, max_dist, scalar;
   LaserPoint                     map_laser_point, check_point;
   LaserPoints                    map_laser_points, all_map_laser_points, sel_map_laser_points;
   DataBoundsLaser                labelbounds;
   ObjectPoint                    point;
   
   if (add_map_data){
        bmap_points.insert(bmap_points.end(), map_points.begin(), 
                                             map_points.end());
        polygons.insert(polygons.end(), sel_map_lines.begin(), sel_map_lines.end());                                              
        next_linenumber = sel_map_lines.size(); //Tricky bussiness. TODO find better solution for this.
        next_number = map_points.HighestPointNumber().Number()+1;
   }
   for (buildoutline=sel_map_lines.begin(); buildoutline!= sel_map_lines.end(); buildoutline++) {
        printf("map line number: %d\n", buildoutline->Number());
      map_laser_points.ErasePoints();
      best = 0;
      for (node2=buildoutline->begin(); node2!=buildoutline->end(); node2++) {
            this_point = map_points.PointIterator(*node2);
            map_laser_point.X()=this_point->X(); 
            map_laser_point.Y()=this_point->Y();
            map_laser_point.Z()=0;
            map_laser_point.SetPointNumber(this_point->Number());
            map_laser_points.push_back(map_laser_point);
            all_map_laser_points.push_back(map_laser_point);
      }
      // new approach
      if (new_approach){
      for (node2=buildoutline->begin(); node2!=buildoutline->end()-1; node2++) {
            this_point = map_points.PointIterator(*node2);
            next_point = map_points.PointIterator(*(node2+1));
            pos2D = Position2D(this_point->X(), this_point->Y());
            pos2D2 = Position2D(next_point->X(), next_point->Y());
            line = Line2D(pos2D, pos2D2);
            sel_map_laser_points.ErasePoints();
            for (laser_point=map_laser_points.begin(); laser_point!=map_laser_points.end()-1; laser_point++) {
                if (line.DistanceToPoint(laser_point->Position2DOnly()) < 0.1) {
                    sel_map_laser_points.push_back(*laser_point);                                       
                }
            }
            printf("size of sel_map_laser: %d\n", sel_map_laser_points.size());
            if (sel_map_laser_points.size()>2){
            laser_point=sel_map_laser_points.begin(); 
            pos2D = Position2D(laser_point->Position2DOnly());
            scalar_min = scalar_max = line.Scalar(pos2D);
            min_dist = max_dist = line.DistanceToPointSigned(pos2D);
            for (laser_point=sel_map_laser_points.begin(); laser_point!=sel_map_laser_points.end()-1; laser_point++) {
                pos2D = laser_point->Position2DOnly();
                scalar = line.Scalar(pos2D);
                if (scalar > scalar_max) scalar_max = scalar;
                else if (scalar < scalar_min) scalar_min = scalar;
                dist = line.DistanceToPointSigned(pos2D);
                if (dist > max_dist) max_dist = dist;
                else if (dist < min_dist) min_dist = dist;
            }
            line.SetDistanceToOrigin(line.DistanceToOrigin() + max_dist);
            // Get the mid point of the line segment
            pos2D = line.Position((scalar_min + scalar_max) / 2.0);
            printf("Midpoint line %6.2f %6.2f \n", pos2D.X(), pos2D.Y());
            check_point.X() = pos2D.X();
            check_point.Y() = pos2D.Y();
            // Correct the distance if the wrong side was taken and flip the line
            if (check_point.InsidePolygon(map_points, buildoutline->LineTopologyReference())) {
                 line.SetDistanceToOrigin(line.DistanceToOrigin() - 
                  max_dist + min_dist);
                  line.SwapNormal();
                  scalar     = scalar_min;
                  scalar_min = -scalar_max;
                  scalar_max = -scalar;
            }
        // Get the end points of the segment
           pos2D = line.Position(scalar_min);
           point.X() = pos2D.X();
           point.Y() = pos2D.Y();
           printf("endpoint %d %6.2f %6.2f and ", next_number, pos2D.X(), pos2D.Y());
           point.Number() = next_number;
           bmap_points.push_back(point);
           line_segment.Erase();
           line_segment.push_back(PointNumber(next_number));
           next_number++;
           pos2D = line.Position(scalar_max);
           point.X() = pos2D.X();
           point.Y() = pos2D.Y();
           printf("%d %6.2f %6.2f \n", next_number, pos2D.X(), pos2D.Y());
           point.Number() = next_number;
           bmap_points.push_back(point);
           line_segment.push_back(PointNumber(next_number));
           next_number++;
           next_linenumber++;
           line_segment.SetAttribute(LineLabelTag, next_linenumber);
           polygons.push_back(line_segment);
           }
  /*         else { // only two points on one line
               point=*this_point;
               point.Number() = next_number;
               bmap_points.push_back(point);
               line_segment.Erase();
               line_segment.push_back(PointNumber(next_number));       
               next_number++;
               point=*next_point;
               point.Number() = next_number;
               bmap_points.push_back(point);
               line_segment.push_back(PointNumber(next_number));
               next_number++;
               polygons.push_back(line_segment);
               }
    */  }
      
      }
      else {
      // end new approach
      labelbounds = map_laser_points.DeriveDataBounds(0);
      dist_max = (labelbounds.Maximum() - labelbounds.Minimum()).Length() / 2.0;
      dist_min = -dist_max;
      dist_dim  = (int) ((dist_max - dist_min) / 0.05 + 0.01);
      printf("%6.2f %6.2f %5.2f %d\n", labelbounds.MidPoint().X(), labelbounds.MidPoint().Y(), dist_max, dist_dim);
      houghspace.Initialise(360, dist_dim, -2 * atan(1.0), dist_min, 2 * atan(1.0), dist_max,1); 
      houghspace.SetOffsets(labelbounds.MidPoint().X(), labelbounds.MidPoint().Y(), 0.0);
      for (laser_point=map_laser_points.begin(); laser_point!=map_laser_points.end()-1; laser_point++) {
           houghspace.AddPoint(laser_point->Position2DOnly());
      }
      
      line = houghspace.BestLine(&num_points, 0, 1);
      printf("%d, %d\n", num_points, best);
      while (num_points > 4 ){//&& best < 100){
      sel_map_laser_points.ErasePoints();
      for (laser_point=map_laser_points.begin(); laser_point!=map_laser_points.end()-1; laser_point++) {
        if (line.DistanceToPoint(laser_point->Position2DOnly()) < 0.05) {
            sel_map_laser_points.push_back(*laser_point);                                       
            houghspace.RemovePoint(laser_point->Position2DOnly());
        }
      }
      
      printf("%d\n", sel_map_laser_points.size());
      laser_point=sel_map_laser_points.begin(); 
      pos2D = Position2D(laser_point->Position2DOnly());
      scalar_min = scalar_max = line.Scalar(pos2D);
      min_dist = max_dist = line.DistanceToPointSigned(pos2D);
      for (laser_point=sel_map_laser_points.begin(); laser_point!=sel_map_laser_points.end()-1; laser_point++) {
           pos2D = laser_point->Position2DOnly();
           scalar = line.Scalar(pos2D);
           if (scalar > scalar_max) scalar_max = scalar;
           else if (scalar < scalar_min) scalar_min = scalar;
           dist = line.DistanceToPointSigned(pos2D);
           if (dist > max_dist) max_dist = dist;
             else if (dist < min_dist) min_dist = dist;
       }
      printf("%6.2f %6.2f %5.2f\n", min_dist, max_dist, scalar); 
      line.SetDistanceToOrigin(line.DistanceToOrigin() + max_dist);
      // Get the mid point of the line segment
      pos2D = line.Position((scalar_min + scalar_max) / 2.0);
      printf("Midpoint line %6.2f %6.2f \n", pos2D.X(), pos2D.Y());
      check_point.X() = pos2D.X();
      check_point.Y() = pos2D.Y();
      // Correct the distance if the wrong side was taken and flip the line
      if (check_point.InsidePolygon(map_points, buildoutline->LineTopologyReference())) {
         line.SetDistanceToOrigin(line.DistanceToOrigin() - 
                  max_dist + min_dist);
         line.SwapNormal();
         scalar     = scalar_min;
         scalar_min = -scalar_max;
         scalar_max = -scalar;
         }
        // Get the end points of the segment
        pos2D = line.Position(scalar_min);
      point.X() = pos2D.X();
      point.Y() = pos2D.Y();
      printf("endpoint %d %6.2f %6.2f and ", next_number, pos2D.X(), pos2D.Y());
      point.Number() = next_number;
      bmap_points.push_back(point);
      line_segment.Erase();
      line_segment.push_back(PointNumber(next_number));
      next_number++;
      pos2D = line.Position(scalar_max);
      point.X() = pos2D.X();
      point.Y() = pos2D.Y();
      printf("%d %6.2f %6.2f \n", next_number, pos2D.X(), pos2D.Y());
      point.Number() = next_number;
      bmap_points.push_back(point);
      line_segment.push_back(PointNumber(next_number));
      next_number++;
      polygons.push_back(line_segment);
      printf("numpts: %d, iteration: %d\n", num_points, best);
      best++;
      line = houghspace.BestLine(&num_points, 0, 1);
      }
   }
}
    bmap_points.RemoveDoublePoints(polygons, 0.05);
    if (!polygons.Write(map_topology_output, false))
    printf("Error writing the topology lines\n");   
 if (!bmap_points.Write(map_points_output))
    printf("Error writing the object points\n");    
     if (!all_map_laser_points.Write(laser_output, false))
        printf("Error writing building laser point file.\n");    

return;
} //end bpartition
}
/* old text. made on 30-5-07
  /*         //  way to calculate convex hull
            component.Erase();
            edges.Erase();
            edges.Derive(seg_laser_points.TINReference());
            for (int i=0; i<seg_laser_points.size(); i++) component.push_back(PointNumber(i));
            contour = seg_laser_points.DeriveContour(1, component, edges, false, LabelTag);
            if (!contour.IsClosed())
               contour.push_back(*(contour.begin()));
         //   contour.SetAttribute(LineLabelTag, *segment_number);

*/
