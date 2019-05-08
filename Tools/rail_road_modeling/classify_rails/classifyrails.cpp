/*
--------------------------------------------------------------------------------

 Initial creation:
 Author : Sander Oude Elberink
 Date   : 26-08-2008

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include "LaserPoints.h"
#include "DataGrid.h"
#include <LineSegment2D.h> 

/*
--------------------------------------------------------------------------------
                      The main classifyrails function
--------------------------------------------------------------------------------
*/

void classifyrails(char *laser_input, char *laser_output, double grid_size, bool do_ransac)
{
   PointNumberList                     list, neighbourhood;
   PointNumberList::iterator           node, nb_node, node2,noden;
   TINEdges                            edges, *edges2;
   LaserPoints::iterator               point, nb_point, goodpoint, point1, point2;
   LaserPoint                          dirpoint;
   LaserPoints                         points, sel_laser_points, lowpoints,
                                       loc_laser_points, outpoints,
                                       railpoints, wirepoints, dirpoints, inlierpoints,
                                       highpoints, growingpoints, keeppoints,
                                       linkedlaserpoints, sellinkedlaserpoints;
   int                                 first_number, i, count0, minpoint, cancelwire,
                                       countpotwire, countwire, newpointnumber,
                                       countlinks, iter, label, label2, count;
   double                              dx, dy, dz, dist, zmin, zp,slope, incept, r,
                                       segdir, height_of_wire, perpdist;
   float                               azimuth, azimuth_link, diffazimuth, first_orientation = 1000;
   bool                                pot=false, pot2 = false, found,
                                       foundlink, swap = false;
   Position3D                          pos, posl, fitpos, fitpos2, pos1, pos2, projpos,
                                       pos3, beginpos, endpos, midpos,
                                       pos21, pos22;
   vector<double>                      wireheights;
   ObjectPoints                        railmappoints, allrailmappoints,
                                       selrailmappoints, linkedpoints;
   ObjectPoints::iterator              endpoint, endpoint2, keeppoint;
   ObjectPoint                         railmappoint;
   LineTopologies                      railmaplines, allrailmaplines, endlines,
                                       counterlines, keepmaplines, extendlines,
                                       counterlines2, linkedlines;
   LineTopology                        railmapline, top, startline, extendedline;
   LineTopology::iterator              mapnode, map_node2;
   LineTopologies::iterator            map_line, map_line2;
   Line3D                              fittedline, line;
   Line2D                              line2d, line2d2;
   LineSegments3D                      segments;
   LineSegment3D                       linesegment, linesegment2;
   
   int                                 numhits;
   Plane                               plane;
   Vector3D                            direction;
   vector <int>         seg_nums;
   vector <int>::iterator       seg_id;
   double nbh_radius, percvalue, percvalue95, percvalue50, percentage;
  double min_x, max_x, min_y, max_y, min_z, max_z, x,y, PI = 3.14159;
  int numcols, numrows, nearest_laser_point;
  SegmentationParameters *segpar;
  segpar=new SegmentationParameters();
  segpar->MaxDistanceInComponent()=0.3;

  printf("grid size taken is %4.2f\n", grid_size);

  if (!points.Read(laser_input)) {
    printf("Error reading laser points from file %s\n", laser_input);
    exit(0);
  }
  
 /* for (point = points.begin(); point != points.end(); point++){
      point->X() = point->X() - 645000;
      point->Y() = point->Y() - 5257000;
      point->Z() = point->Z() - 900;
      }
 if (!points.Write(laser_output, false))
    printf("Error writing the output laser points\n");  
//}
  return;     
   
 // printf("grid size taken is %4.2f\n", grid_size);
//  outpoints = points.SimplifyMesh(0.05);
//if (!outpoints.Write(laser_output, false))
 //   printf("Error writing the output laser points\n");  
//}
//  return;
  
 //points.ReduceData(0.1);
 /* Initialise all points and derive the TIN */
 
   points.SetAttribute(IsProcessedTag, 0); /* Set all points unprocessed */
   points.SetUnFiltered(); /* Accept all points */
//   points.RemoveAlmostDoublePoints(false, 0.02);
  // points.DeriveTIN();
 //  edges.Derive(points.TINReference());
   points.SetAttribute(LabelTag, 0);
   points.DeriveDataBounds(0);
 
// bool do_ransac;
// do_ransac = false;
 if (do_ransac){  
   vector <int>         segmentnumbers;
vector <int>::iterator segmentnumber;
segmentnumbers = points.AttributeValues(SegmentNumberTag);
for (segmentnumber = segmentnumbers.begin(); segmentnumber!=segmentnumbers.end(); segmentnumber++){
 
    sel_laser_points.ErasePoints();
    sel_laser_points.AddTaggedPoints(points, *segmentnumber, SegmentNumberTag);
    sel_laser_points.RemoveAlmostDoublePoints(false, 0.01);
    printf("num of laserpoints: %d\n", sel_laser_points.size());    
        if (sel_laser_points.size()>4){
           fittedline = sel_laser_points.RANSAC3DLine(0.15, int(50.0*sel_laser_points.size()/100), 500, numhits, 1, segments);
           if (numhits >= int(50.0*sel_laser_points.size()/100)){
           printf("\nnum of hits: %d \n", numhits);
           printf("num of laserpoints: %d", sel_laser_points.size());
           for (point = sel_laser_points.begin(); point != sel_laser_points.end(); point++){
                if (fittedline.DistanceToPoint(point->Position3DRef()) <= 0.15) {
                inlierpoints.push_back(*point);
                }
           }
           
           railmappoints.erase(railmappoints.begin(), railmappoints.end());
           railmaplines.erase(railmaplines.begin(), railmaplines.end());
           segments.PointsWithTopology(railmappoints, railmaplines, 1);
           railmaplines.SetAttribute(LineLabelTag, *segmentnumber);
//           printf("num of lines: %d \n", railmaplines.size());
           if (!allrailmappoints.empty())
           railmaplines.ReNumber(railmappoints, (allrailmappoints.end()-1)->Number()+1, (allrailmaplines.end()-1)->Number()+1);
           allrailmaplines.insert(allrailmaplines.end(), railmaplines.begin(), railmaplines.end());
           allrailmappoints.insert(allrailmappoints.end(), railmappoints.begin(), railmappoints.end());
           }
           else {
               printf("\nNOTENOUGH!!! num of hits: %d \n", numhits);
               printf("NOTENOUGH!!! num of laserpoints: %d \n", sel_laser_points.size());

                }
        }
        }
printf("Averaging double points\n");  

allrailmappoints.AverageDoublePoints(allrailmaplines, 0.25, 0.1);
printf("Removing double points from %d to ", allrailmappoints.size());  
allrailmappoints.RemoveDoublePoints(allrailmaplines, 0.15);
printf("%d.\n", allrailmappoints.size());
allrailmaplines.Write("allrailmaplines2.top", false);
allrailmappoints.Write("allrailmappoints2.objpts");      
inlierpoints.Write("inlierpoints2.laser", false);   
 return;  
   
}
 /* Process all nodes */
 max_x =  (points.DataBounds().Maximum().X());
 max_y =  (points.DataBounds().Maximum().Y());
 min_x =  (points.DataBounds().Minimum().X());
 min_y =  (points.DataBounds().Minimum().Y());
 min_z =  (points.DataBounds().Minimum().Z());
 max_z =  (points.DataBounds().Maximum().Z());
 numcols =  int ((max_x-min_x)/grid_size)+1;
 numrows =  int ((max_y-min_y)/grid_size)+1;
 count = 0, count0 = 0;
 printf("min_x: %4.2f, min_y: %4.2f\n", min_x, min_y);
 printf("max_x: %4.2f, max_y: %4.2f\n", max_x, max_y);

 for (x=min_x; x<max_x; x+=(grid_size)){
    for (y=min_y; y<max_y; y+=(grid_size)){
        count0++;
        printf("%4.2f %4.2f, %4.2f \n", x, y, count0*100.0/(numcols*numrows));
        loc_laser_points.ErasePoints();
        for (point = points.begin(); point != points.end(); point++){
            if ((point->X() < x+0.5*grid_size) && (point->X() > x-0.5*grid_size)){
               if ((point->Y()< y+0.5*grid_size) && (point->Y() > y-0.5*grid_size)){
                  loc_laser_points.push_back(*point);
               }
            }
        }
        if (loc_laser_points.size()<2) continue;
        loc_laser_points.RemoveAlmostDoublePoints(false, 0.02);
        loc_laser_points.SwapXZ();
        loc_laser_points.SortOnCoordinates();
        loc_laser_points.SwapXZ();
        loc_laser_points.SetAttribute(SegmentNumberTag, count0);
  //      printf("size: %d\n", loc_laser_points.size());
        if (loc_laser_points.size()>4){
 //       printf("min: %4.2f", loc_laser_points[0].Z());
        percvalue = loc_laser_points[int(loc_laser_points.size()/10)].Z();
 //       printf(" 10th perc: %4.2f", percvalue);
        percvalue50 = loc_laser_points[int(50*loc_laser_points.size()/100)].Z();
        percvalue95 = loc_laser_points[int(98*loc_laser_points.size()/100)].Z();
 //       printf(" 95th perc: %4.2f", percvalue95);
  //      printf(" max: %4.2f", loc_laser_points[loc_laser_points.size()-1].Z());
        countwire = 0;
        countpotwire = 0;
        cancelwire = 0;
        wireheights.resize(0);
        
         if (loc_laser_points[loc_laser_points.size()-1].Z()-percvalue >= 0.5){
             for (point = loc_laser_points.begin(); point != loc_laser_points.end(); point++){
                if(point->Z() >= percvalue + 0.5) {
                   point->Label(10); //
                   if (point->Z() < percvalue + 4){
                                  point->Label(9);
                                  cancelwire++;
                                  }
                   if (point->Z() > percvalue + 5.5 && point->Z() < percvalue + 6.5){
                                  point->Label(11); //powerwire
                                  countpotwire++;
                                  wireheights.push_back(point->Z());
                                  } 
                   if (point->Z() > percvalue + 6.5 && point->Z() < percvalue + 20){
                                  point->Label(12); //dark blue, wire
                                  countwire++;
                                  } 
                   
//                   outpoints.push_back(*point);
                }
             }
             if (wireheights.size()>4){
                 sort(wireheights.begin(), wireheights.end());
                 height_of_wire = wireheights[int(wireheights.size()/2)];
             }
             for (point = loc_laser_points.begin(); point != loc_laser_points.end(); point++){
                  if(point->Z() >= percvalue + 0.5) {
                     if(cancelwire > int(loc_laser_points.size()/10)) point->Label(10); 
                     else {
                        if(point->Label()==11 && wireheights.size()>4 && point->Z()>height_of_wire+0.05) point->Label(12); 
                        }
  //                      point->Z() = point->Z() - percvalue;
                        highpoints.push_back(*point);
                     }
               }
            
             loc_laser_points.RemoveTaggedPoints(11, LabelTag);
             loc_laser_points.RemoveTaggedPoints(12, LabelTag);
           loc_laser_points.RemoveTaggedPoints(9, LabelTag);
           loc_laser_points.RemoveTaggedPoints(10, LabelTag);
           loc_laser_points.SwapXZ();
           loc_laser_points.SortOnCoordinates();
           loc_laser_points.SwapXZ();
           printf("new size: %d\n", loc_laser_points.size());
  //         printf("min: %4.2f", loc_laser_points[0].Z());
           percvalue = loc_laser_points[int(loc_laser_points.size()/10)].Z();
  //         printf(" 10th perc: %4.2f", percvalue);
           percvalue50 = loc_laser_points[int(50*loc_laser_points.size()/100)].Z();
           percvalue95 = loc_laser_points[int(98*loc_laser_points.size()/100)].Z();
           printf(" 95th perc: %4.2f", percvalue95);
           printf(" max: %4.2f", loc_laser_points[loc_laser_points.size()-1].Z());
          }
          count =0;
         for (point = loc_laser_points.begin(); point != loc_laser_points.end(); point++){    
                if (point->Z()>= percvalue95 - 0.10){
                    count++;
                }
           }
         percentage = 1.0 * count / (1.0*loc_laser_points.size());
         if (percentage < 0.7){// && cancelwire < 10){
           for (point = loc_laser_points.begin(); point != loc_laser_points.end(); point++){    
                point->Label(3); //yellow
//                if (point->Z()>=percvalue95 - 0.05 && (percvalue95 - percvalue >=0.1)){
                if (point->Z()>=percvalue95 - 0.1 && (percvalue95 - percvalue >=0.05)){
                    point->Label(8); //green
                    count++;
                }
                if (point->Z()<=percvalue){
                     point->Label(6); //cyan
                }
  //              point->Z() = point->Z() - percvalue;
   //             outpoints.push_back(*point);
            }
           }
           else{
                for (point = loc_laser_points.begin(); point != loc_laser_points.end(); point++){    
                  point->Label(4); //orange
                  if (point->Z()<=percvalue){
                      point->Label(6); //cyan
                  }
  //                point->Z() = point->Z() - percvalue;
//                  outpoints.push_back(*point);
                 }
            }
            lowpoints.ErasePoints();
            lowpoints.AddTaggedPoints(loc_laser_points, 6, LabelTag);
            lowpoints.AddTaggedPoints(loc_laser_points, 3, LabelTag);
            plane = lowpoints.FitPlane(3,6, LabelTag);
           for (point = loc_laser_points.begin(); point != loc_laser_points.end(); point++){    
             point->Residual() = 0;
             if (point->Label()==8){
                 pos = Position3D(point->X(), point->Y(), point->Z());
//                 plane = Plane(p1, p2, p3);
                 projpos = plane.Project(pos);
                 point->Residual() = projpos.Distance(pos) ;//fabs(plane.Z_At(laser_point->X(), laser_point->Y(), &success)-laser_point->Z());
                 if (point->Residual()<0.1) point->Label(5);
             }
             outpoints.push_back(*point);
             }          
   //         sel_laser_points.ErasePoints();
   //         sel_laser_points.AddTaggedPoints(loc_laser_points, 8 , LabelTag);
   //         if (sel_laser_points.size()>10){
    //            fittedline = sel_laser_points.RANSAC3DLine(0.1, int(75.0*sel_laser_points.size()/100), 50, numhits, 0.4, segments);
    //            printf("\nnum of hits: %d (out of %d) \n", numhits, sel_laser_points.size());
      //          segments.PointsWithTopology(allrailmappoints, allrailmaplines, 1);
 /*           fittedline = sel_laser_points.RANSAC3DLine(0.1, 5, 5, numhits, 0.5, segments);
            line2d = Line2D(Position2D(x,y+grid_size), Position2D(x+grid_size, y+grid_size));
            line2d2 = Line2D(Position2D(x,y), Position2D(x+grid_size, y));
            if (fittedline.FindIntersection(line2d, fitpos) && fittedline.FindIntersection(line2d2, fitpos2)){
//              printf("hoera %4.2f %4.2f %4.2f\n", fitpos.X(), fitpos.Y(), fitpos.Z());
                newpointnumber++;
                railmappoint = ObjectPoint(fitpos.GetX(), fitpos.GetY(), fitpos.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                top.clear(); top.Initialise();
                top.push_back(PointNumber(newpointnumber));
                allrailmappoints.push_back(railmappoint);
                newpointnumber++;
                railmappoint = ObjectPoint(fitpos2.GetX(), fitpos2.GetY(), fitpos2.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                top.push_back(PointNumber(newpointnumber));
                allrailmappoints.push_back(railmappoint);
                allrailmaplines.push_back(top);
                dirpoints.push_back(LaserPoint(fitpos.X(), fitpos.Y(), fitpos.Z()));
              }
   */         
  /*            sel_laser_points.SwapXY();
              sel_laser_points.SortOnCoordinates();
              sel_laser_points.SwapXY();
              point1 = sel_laser_points.begin();
              point2 = sel_laser_points.end()-1;
              dx = point2->X() - point1->X();
              dy = point2->Y() - point1->Y();
              segdir = atan(dx/dy);
              segdir = segdir * 180 / PI;
              printf("seg dir is %4.2f ", segdir);
              
  //            sel_laser_points.Fit2DLine(slope, incept);
  //            printf("slope %f\n", slope);
            for (point = sel_laser_points.begin(); point != sel_laser_points.end(); point++){    
               point->Z() = segdir;
            }
            
            dirpoints.AddPoints(sel_laser_points);       
        //    r = sel_laser_points.CorrelationCoefficient();
        //    r = fabs(r);
        //    sel_laser_points.SetAttribute(ResidualTag, float(r));
   //         outpoints.AddPoints(sel_laser_points);
    */   //     }
         
    /*    pos = Position3D(x,y,0);
        printf("x: %d, y: %d ", x,y);
        nearest_laser_point = points.NearestPoint(pos,
                                            edges, true);
        posl = Position3D(points[nearest_laser_point].X(), points[nearest_laser_point].Y(),0);
 //       printf("xl: %f, yl: %f\n", posl.X(), posl.Y());
        
        if (pos.Distance(posl)<1){
           nbh_radius = 1;
           neighbourhood = points.Neighbourhood(PointNumber(nearest_laser_point),
                                       nbh_radius, edges,
                                       true, false);
           loc_laser_points.ErasePoints();
            for (noden=neighbourhood.begin(); noden!=neighbourhood.end(); noden++){
                           loc_laser_points.push_back(points[noden->Number()]);
            }
           loc_laser_points.SwapXZ();
           loc_laser_points.SortOnCoordinates();
           loc_laser_points.SwapXZ();
           printf("size: %d\n", loc_laser_points.size());
           printf("min: %4.2f", loc_laser_points[0].Z());
           percvalue = loc_laser_points[int(loc_laser_points.size()/4)].Z();
           printf(" 25th perc: %4.2f", percvalue);
           percvalue95 = loc_laser_points[int(95*loc_laser_points.size()/100)].Z();
           printf(" 95th perc: %4.2f", percvalue95);
           printf(" max: %4.2f", loc_laser_points[loc_laser_points.size()-1].Z());
           zmin = 1000;
           if (loc_laser_points[loc_laser_points.size()-1].Z()-percvalue>1){
               loc_laser_points.ErasePoints();
               for (noden=neighbourhood.begin(); noden!=neighbourhood.end(); noden++){
                    if(points[noden->Number()].Z()<percvalue+0.5){
                           loc_laser_points.push_back(points[noden->Number()]);
                           }
                    else {
                        points[noden->Number()].Label(6);
                        }
               }
               loc_laser_points.SwapXZ();
               loc_laser_points.SortOnCoordinates();
               loc_laser_points.SwapXZ();
               printf("new size: %d\n", loc_laser_points.size());
               printf("min: %4.2f", loc_laser_points[0].Z());
               percvalue = loc_laser_points[int(loc_laser_points.size()/4)].Z();
               printf(" 25th perc: %4.2f", percvalue);
               percvalue95 = loc_laser_points[int(95*loc_laser_points.size()/100)].Z();
               printf(" 95th perc: %4.2f", percvalue95);
               printf(" max: %4.2f", loc_laser_points[loc_laser_points.size()-1].Z());
            }
           
           for (node2=neighbourhood.begin(); node2!=neighbourhood.end(); node2++){
               if (points[node2->Number()].Label()!=6){
                zp = points[node2->Number()].Z();
                points[node2->Number()].Label(5);
                if (zp<zmin){
                    zmin = zp;
                    minpoint = node2->Number();
                }
                }
           }
           lowpoints.push_back(points[minpoint]);
    
           for (node2=neighbourhood.begin(); node2!=neighbourhood.end(); node2++){          
                if (points[node2->Number()].Label()!=6){
                zp = points[node2->Number()].Z();
            //    if (zp>zmin+0.10 && zp<zmin+0.5){
                if (zp>percvalue95){
                    points[node2->Number()].Label(8);
                }
           }
           }
           for (node2=neighbourhood.begin(); node2!=neighbourhood.end(); node2++){          
                zp = points[node2->Number()].Z();
                if (zp<percvalue){
                    points[node2->Number()].Label(7);
                }
           }
        }  
    */  }
    }          
  }
//  lowpoints.ReduceData(0.2);

//railpoints.ErasePoints();
//railpoints.AddTaggedPoints(outpoints, 8, LabelTag);
wirepoints.ErasePoints();
wirepoints.AddTaggedPoints(highpoints, 11, LabelTag);
segpar->MinNumberOfPointsComponent()=200;

edges2 = wirepoints.DeriveEdges(*segpar);
  if (segpar->MaxDistanceInComponent() > 0.0)
    wirepoints.RemoveLongEdges(edges2->TINEdgesRef(), 
                      1,// segpar->MaxDistanceInComponent(),
                       segpar->DistanceMetricDimension() == 2);
wirepoints.LabelComponents(edges2->TINEdgesRef(), SegmentNumberTag);//SOE
//printf("Size of railpoints %d, wirepoints %d\n", railpoints.size(), wirepoints.size());
//allrailmaplines.Write("allrailmaplines.top", false);
//allrailmappoints.Write("allrailmappoints.objpts");
wirepoints.Write("wirepoints.laser", false);
//return;
printf("\nKeep rail road points near wires\n");
outpoints.SortOnCoordinates();
for (point = outpoints.begin(), i=0; point != outpoints.end(); point++, i++){
    if (point->Label()==8){
    printf("%4.2f\r", 100.0*i/outpoints.size());
    found = false;
//    for (point2 = outpoints.begin(); point2 != outpoints.end() && !found; point2++){
    for (point2 = wirepoints.begin(); point2 != wirepoints.end() && !found; point2++){
//        if (point2->Label()==11){
          dx = point2->X() - point->X();
          dy = point2->Y() - point->Y();
          if (dx<2 && dy<2){
                    dz = point2->Z() - point->Z();
                    dist = dx*dx + dy*dy;
                    if (dist<4 && dz > 5.2 && dz < 6.2) {
                    found = true;
                    point->SetAttribute(PlaneNumberTag, point2->Attribute(SegmentNumberTag));
                    }
                    }
  //      }
    }
    if (!found) point->Label(5);//was 3
    }
   // point->SetAttribute(SegmentNumberTag, point->Label());
    if (point->Label() == 8 || point->Label() == 11||point->Label() == 12) railpoints.push_back(*point); 
}
if (!outpoints.Write("outpoints.laser", false))
    printf("Error writing the output laser points\n");  
//return; //see if this is enough for modelling with KK's approach...
if (!highpoints.Write("highlaserpoints.laser", false))
    printf("Error writing the output laser points\n");  
    
//fit line through larger group of points of same label:
 grid_size = 1; 
 railpoints = outpoints;
 railpoints.AddTaggedPoints(highpoints, 11, LabelTag);
 railpoints.DeriveDataBounds(0);   
 max_x =  (railpoints.DataBounds().Maximum().X());
 max_y =  (railpoints.DataBounds().Maximum().Y());
 min_x =  (railpoints.DataBounds().Minimum().X());
 min_y =  (railpoints.DataBounds().Minimum().Y());
 min_z =  (railpoints.DataBounds().Minimum().Z());
 max_z =  (railpoints.DataBounds().Maximum().Z());
 numcols =  int ((max_x-min_x)/grid_size)+1;
 numrows =  int ((max_y-min_y)/grid_size)+1;
 count = 0, count0 = 0;
 printf("min_x: %4.2f, min_y: %4.2f\n", min_x, min_y);
 printf("max_x: %4.2f, max_y: %4.2f\n", max_x, max_y);

   seg_nums = railpoints.AttributeValues(SegmentNumberTag);
   sort(seg_nums.begin(), seg_nums.end());
   for (seg_id = seg_nums.begin(); seg_id!=seg_nums.end(); seg_id++){
       loc_laser_points.ErasePoints();
       loc_laser_points = railpoints.SelectTagValue(SegmentNumberTag, *seg_id);

/* for (x=min_x; x<max_x; x+=(grid_size)){
    for (y=min_y; y<max_y; y+=(grid_size)){
        count0++;
        printf("%4.2f %4.2f, %4.2f\r", x, y, count0*100.0/(numcols*numrows));
        loc_laser_points.ErasePoints();
        for (point = railpoints.begin(); point != railpoints.end(); point++){
            if ((point->X() < x+0.5*grid_size) && (point->X() > x -0.5*grid_size)){
               if ((point->Y()< y+0.5*grid_size) && (point->Y() > y -0.5*grid_size)){
   //               point->SetAttribute(SegmentNumberTag, count0);
                  loc_laser_points.push_back(*point);
               }
            }
        }
  */      loc_laser_points.RemoveAlmostDoublePoints(false, 0.01);
        if (loc_laser_points.size()>4){
        sel_laser_points.ErasePoints();
        sel_laser_points.AddTaggedPoints(loc_laser_points, 11, LabelTag);
        printf("num of laserpoints: wire %d of seg %d %d (%4.2f) \n", sel_laser_points.size(), *seg_id, loc_laser_points.size(), 100.0*sel_laser_points.size()/loc_laser_points.size());
          
//        sel_laser_points.RemoveAlmostDoublePoints(false, 0.01);
        if (sel_laser_points.size()>4){
        
           fittedline = sel_laser_points.RANSAC3DLine(0.05, int(50.0*sel_laser_points.size()/100), 500, numhits, 1, segments);
           if (numhits >= int(50.0*sel_laser_points.size()/100)){
           printf("\nnum of hits: %d \n", numhits);
           printf("num of laserpoints: %d \n", sel_laser_points.size());
           railmappoints.erase(railmappoints.begin(), railmappoints.end());
           railmaplines.erase(railmaplines.begin(), railmaplines.end());
           segments.PointsWithTopology(railmappoints, railmaplines, 1);
           railmaplines.SetAttribute(LineLabelTag, 11);
           printf("num of lines: %d \n", railmaplines.size());
           for (point = loc_laser_points.begin(); point != loc_laser_points.end(); point++){
                if (fittedline.DistanceToPoint(point->Position3DRef()) <= 0.12) {
                   inlierpoints.push_back(*point);
                }
           }
     
           if (!allrailmappoints.empty())
           railmaplines.ReNumber(railmappoints, (allrailmappoints.end()-1)->Number()+1, (allrailmaplines.end()-1)->Number()+1);
           allrailmaplines.insert(allrailmaplines.end(), railmaplines.begin(), railmaplines.end());
           allrailmappoints.insert(allrailmappoints.end(), railmappoints.begin(), railmappoints.end());
           }
        }
        sel_laser_points.ErasePoints();
        sel_laser_points.AddTaggedPoints(loc_laser_points, 8, LabelTag);
        segments.erase(segments.begin(), segments.end());
        numhits = 0;
        printf("num of laserpoints: rail %d of %d (%4.2f) \n", sel_laser_points.size(), loc_laser_points.size(), 100.0*sel_laser_points.size()/loc_laser_points.size());

  //      sel_laser_points.RemoveAlmostDoublePoints(false, 0.01);
        if (sel_laser_points.size()>4){
           printf("so.");
           fittedline = sel_laser_points.RANSAC3DLine(0.03, int(50.0*sel_laser_points.size()/100), 500, numhits, 1, segments);
           printf("num of hits %d segments %d \n", numhits, segments.size());

           if (numhits >= int(50.0*sel_laser_points.size()/100) && segments.size()==1){
           printf("\nnum of hits: %d \n", numhits);
           printf("num of laserpoints: %d \n", sel_laser_points.size());
           railmappoints.erase(railmappoints.begin(), railmappoints.end());
           railmaplines.erase(railmaplines.begin(), railmaplines.end());
           segments.PointsWithTopology(railmappoints, railmaplines, 0);
           railmaplines.SetAttribute(LineLabelTag, 8);
           printf("num of lines: %d \n", railmaplines.size());
    
//           direction = fittedline.Direction();
      //     printf("%5.2f, %5.2f, %5.2f/n", direction[0],direction[1], direction[2]);
//           system("pause");
  //           azimuth = 180*atan(direction[0]/direction[1])/PI;
//             azimuth = sqrt(direction[0]);
               beginpos = segments[0].BeginPoint();
               endpos = segments[0].EndPoint();
               midpos = segments[0].MiddlePoint();
               if (first_orientation == 1000){
               dx = fabs(endpos.GetX()-beginpos.GetX());
               dy = fabs(endpos.GetY()-beginpos.GetY());
               if (dy>dx) swap = true;
               }
               if (!swap && endpos.GetX()<beginpos.GetX()){
               beginpos = segments[0].EndPoint();
               endpos = segments[0].BeginPoint();
               }
               if (swap && endpos.GetY()<beginpos.GetY()){
               beginpos = segments[0].EndPoint();
               endpos = segments[0].BeginPoint();
               }
               
               if (beginpos.Distance(endpos)>0.3 && railmaplines.size()==1){
//                dy = endpos.GetY()-beginpos.GetY();
//                if (dy>0) azimuth = atan((endpos.GetX()-beginpos.GetX())/(endpos.GetY()-beginpos.GetY()));
//                if (dy<=0) azimuth = atan((beginpos.GetX()-endpos.GetX())/(endpos.GetY()-beginpos.GetY()));
  
              azimuth = atan2((endpos.GetX()-beginpos.GetX()),(endpos.GetY()-beginpos.GetY()));
              azimuth = azimuth *180/PI;
              if (azimuth<0) azimuth = azimuth + 360;
              first_orientation = azimuth;

              dirpoint = LaserPoint(midpos.X(), midpos.Y(), midpos.Z());
              dirpoint.SetAttribute(AverageAngleTag, azimuth);
              dirpoint.SetAttribute(SegmentNumberTag, *seg_id);
              dirpoints.push_back(dirpoint);
           for (point = loc_laser_points.begin(); point != loc_laser_points.end(); point++){
                if (fittedline.DistanceToPoint(point->Position3DRef()) <= 0.12) {
                point->SetAttribute(AverageAngleTag, azimuth);
               inlierpoints.push_back(*point);
                }
           }
       
           if (!allrailmappoints.empty())
           railmaplines.ReNumber(railmappoints, (allrailmappoints.end()-1)->Number()+1, (allrailmaplines.end()-1)->Number()+1);
           allrailmaplines.insert(allrailmaplines.end(), railmaplines.begin(), railmaplines.end());
           allrailmappoints.insert(allrailmappoints.end(), railmappoints.begin(), railmappoints.end());
           }
           }
        }
        }

  /*      line2d = Line2D(Position2D(x,y+grid_size), Position2D(x+grid_size, y+grid_size));
        line2d2 = Line2D(Position2D(x,y), Position2D(x+grid_size, y));
        if (fittedline.FindIntersection(line2d, fitpos) && fittedline.FindIntersection(line2d2, fitpos2)){
                newpointnumber++;
                railmappoint = ObjectPoint(fitpos.GetX(), fitpos.GetY(), fitpos.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                top.clear(); top.Initialise();
                top.push_back(PointNumber(newpointnumber));
                allrailmappoints.push_back(railmappoint);
                newpointnumber++;
                railmappoint = ObjectPoint(fitpos2.GetX(), fitpos2.GetY(), fitpos2.GetZ(), newpointnumber, 0,0,0,0,0,0); 
                top.push_back(PointNumber(newpointnumber));
                allrailmappoints.push_back(railmappoint);
                allrailmaplines.push_back(top);
                dirpoints.push_back(LaserPoint(fitpos.X(), fitpos.Y(), fitpos.Z()));
        }
    */   // }
//    }
 }
         
dirpoints.Write("dirpoints.laser", false);
//allrailmappoints.RemoveDoublePoints(allrailmaplines, 0.5);
printf("Averaging double points\n");  
allrailmaplines.Write("allrailmaplines0.top", false);
allrailmappoints.Write("allrailmappoints0.objpts"); 
allrailmappoints.AverageDoublePoints(allrailmaplines, 0.25, 0.1);
printf("Removing double points from %d to ", allrailmappoints.size());  
allrailmappoints.RemoveDoublePoints(allrailmaplines, 0.15);
printf("%d.\n", allrailmappoints.size());
//allrailmaplines.Write("allrailmaplines0.top", false);
//allrailmappoints.Write("allrailmappoints0.objpts"); 
inlierpoints.Write("inlierpoints.laser", false);   

  segpar->MinNumberOfPointsComponent()=50;
bool grow_inlierpoints = true;
if (grow_inlierpoints){

for (map_line=allrailmaplines.begin();map_line!=allrailmaplines.end()-1; map_line++) {
    mapnode = map_line->begin(); 
    pos1 =  (allrailmappoints.PointIterator(*mapnode))->Position3DRef();
    mapnode++;
    pos2 =  (allrailmappoints.PointIterator(*mapnode))->Position3DRef();
    linesegment = LineSegment3D(pos1, pos2);
    midpos = linesegment.MiddlePoint();
    foundlink = false;
    for (point = dirpoints.begin(); point != dirpoints.end(); point++){
        pos21 = point->Position3DRef();
        if (linesegment.DistanceToPoint(pos21) <= 1.6 &&
            linesegment.DistanceToPoint(pos21) >= 1.3 &&
            linesegment.Scalar(pos21)< 1 && 
            linesegment.Scalar(pos21)> 0){
           // dx = midpos.GetX()-pos21.GetX();
           // dy = midpos.GetY()-pos21.GetY();
            //if (dy>0) 
            line = Line3D(pos1, pos2);
            midpos = line.Project(pos21);
            azimuth_link = atan2((midpos.GetX()-pos21.GetX()),(midpos.GetY()-pos21.GetY()));
           // if (dy<=0) azimuth_link = atan((pos21.GetX()-midpos.GetX())/(midpos.GetY()-pos21.GetY()));
            
            azimuth_link = azimuth_link*180/PI;
            if (azimuth_link < 0) azimuth_link = azimuth_link + 360;
            azimuth = point->FloatAttribute(AverageAngleTag);
   //         if (first_orientation == 1000) first_orientation = azimuth;
   //         azimuth = azimuth - first_orientation;
   //         azimuth_link = az
            diffazimuth = azimuth - azimuth_link;
            if (diffazimuth < 0) diffazimuth = diffazimuth + 360;
            printf("x1 %10.2f x2 %10.2f, az1 %4.2f az2 %5.2f diff %5.2f\n", pos21.GetY(), midpos.GetY(), azimuth, azimuth_link, diffazimuth);
            linesegment2 = LineSegment3D(pos21, midpos);
            linesegment2.PointsWithTopology(linkedpoints, linkedlines, 1);
            foundlink = true;
            point->Label(4);
            if (diffazimuth<180) point->Label(5);
            linkedlaserpoints.push_back(*point);            
            }
            }
    if(!foundlink){
    countlinks = 0;
    for (point = railpoints.begin(); point != railpoints.end(); point++){
        if (point->Label()==8){
        pos21 = point->Position3DRef();
        if (linesegment.DistanceToPoint(pos21) <= 1.6 &&
            linesegment.DistanceToPoint(pos21) >= 1.3 &&
            linesegment.Scalar(pos21)< 1 && 
            linesegment.Scalar(pos21)> 0){
 //           growingpoints.push_back(*point);
            countlinks++;
        }
     }
     }
     if (countlinks>10){
     for (point = railpoints.begin(); point != railpoints.end(); point++){                       
        if (point->Label()==8){
        pos21 = point->Position3DRef();
        if (linesegment.DistanceToPoint(pos21) <= 1.6 &&
            linesegment.DistanceToPoint(pos21) >= 1.3 &&
            linesegment.Scalar(pos21)< 1 && 
            linesegment.Scalar(pos21)> 0){
            growingpoints.push_back(*point);
         }
       }
       }
       }
    }
}
    
linkedlaserpoints.Write("linkedpoints.laser", false);                       
growingpoints.Write("growingpoints.laser", false);                       
linkedlines.Write("linkedpoints.top", false);
linkedpoints.Write("linkedpoints.objpts"); 
//  system("pause");                     

growingpoints.AddPoints(inlierpoints);

edges2 = growingpoints.DeriveEdges(*segpar);
  if (segpar->MaxDistanceInComponent() > 0.0)
    growingpoints.RemoveLongEdges(edges2->TINEdgesRef(), 
                       segpar->MaxDistanceInComponent(),
                       segpar->DistanceMetricDimension() == 2);
growingpoints.LabelComponents(edges2->TINEdgesRef(), SegmentNumberTag);//SOE
keeppoints.ErasePoints();
   seg_nums = growingpoints.AttributeValues(SegmentNumberTag);
   for (seg_id = seg_nums.begin(); seg_id!=seg_nums.end(); seg_id++){
       sel_laser_points.ErasePoints();
        sel_laser_points = growingpoints.SelectTagValue(SegmentNumberTag, *seg_id);
        if (sel_laser_points.size()>segpar->MinNumberOfPointsComponent()) {
         found = false;
         iter = 0;
         sellinkedlaserpoints.ErasePoints();
         for (point = sel_laser_points.begin(); point!=sel_laser_points.end() && iter<2; point++){
              pos21 = point->Position3DRef();          
              for (point1 = linkedlaserpoints.begin(); point1!=linkedlaserpoints.end(); point1++){
                  pos1 = point1->Position3DRef();
                  if (pos1.Distance(pos21)<0.3) sellinkedlaserpoints.push_back(*point1);
               }
             }
             label = sellinkedlaserpoints.MostFrequentAttributeValue(LabelTag, count);
             label2 = sel_laser_points.MostFrequentAttributeValue(PlaneNumberTag, count);
             label = label + 10*label2;
             sel_laser_points.SetAttribute(LabelTag, label);
         if (sellinkedlaserpoints.size()>2) keeppoints.AddPoints(sel_laser_points);
         }
        }      
keeppoints.Write(laser_output, false);                       

  
  
}
return;
bool commonedge;
//selrailmappoints = allrailmaplines.ReturnEndPointsOfTopologies(allrailmappoints);
for (map_line=allrailmaplines.begin();map_line!=allrailmaplines.end()-1; map_line++) {
    mapnode = map_line->begin(); 
    counterlines.erase(counterlines.begin(), counterlines.end());
    counterlines = keepmaplines.ReturnLinesWithPoint(mapnode->Number()); 
    mapnode++;
    commonedge = false;    
    for (map_line2=counterlines.begin();map_line2!=counterlines.end(); map_line2++) {
          if (map_line->Number()!=map_line2->Number()) {
             for (map_node2 = map_line2->begin(); map_node2 != map_line2->end(); map_node2++){
                 if (mapnode->Number()==map_node2->Number()) commonedge = true;
             }
          }
    }
//    if (commonedge) 
    map_line2 = map_line;
    map_line2++;
    for (; map_line2!=allrailmaplines.end() && !commonedge; map_line2++){
        if(map_line->CommonEdge(*map_line2) && map_line->Attribute(LineLabelTag) == map_line2->Attribute(LineLabelTag)) commonedge = true;
        }
    if (!commonedge) keepmaplines.push_back(*map_line);
}

/*keepmaplines.erase(keepmaplines.begin(), keepmaplines.end());
for (map_line=allrailmaplines.begin();map_line!=allrailmaplines.end(); map_line++) {
    map_line->RevertNodeOrder();
    if (!allrailmaplines.Contains(*map_line)) keepmaplines.push_back(*map_line);
  }
printf("size of keepmaplines %d (out of %d).\n", keepmaplines.size(), allrailmaplines.size());  

allrailmaplines = keepmaplines;

*/
for (map_line=keepmaplines.begin();map_line!=keepmaplines.end(); map_line++) {
    for (mapnode = map_line->begin(); mapnode!=map_line->end(); mapnode++){
        selrailmappoints.push_back(*allrailmappoints.PointIterator(*mapnode));
        }
  }
printf("Removing double lines: from %d to ", allrailmaplines.size());  
printf("%d.\n", keepmaplines.size());

allrailmaplines.erase(allrailmaplines.begin(), allrailmaplines.end());
allrailmappoints.erase(allrailmappoints.begin(), allrailmappoints.end());

allrailmappoints = selrailmappoints;
allrailmaplines = keepmaplines;
allrailmappoints.RemoveDoublePoints(allrailmaplines, 0.05);
allrailmaplines.Write("allrailmaplines.top", false);
allrailmappoints.Write("allrailmappoints.objpts"); 
//return;
selrailmappoints.erase(selrailmappoints.begin(), selrailmappoints.end());  
for (endpoint = allrailmappoints.begin(); endpoint!=allrailmappoints.end(); endpoint++){
              pos1 = endpoint->Position3DRef();
              counterlines.erase(counterlines.begin(), counterlines.end());
              counterlines = allrailmaplines.ReturnLinesWithPoint(endpoint->Number()); //select lines containing this point
              if (counterlines.size()==1) {                                            //if only one line, then this point is an end point
                 startline = counterlines[0];
                 mapnode = startline.begin();
                 if (mapnode->Number()== endpoint->Number()) mapnode++;                // so, endpoint is the end point, map node the other point
                 pos2 = (allrailmappoints.PointIterator(*mapnode))->Position3DRef();
                 if (pos2.Distance(pos1)> 0.1) {
                    line = Line3D(pos2, pos1);                                         // make line from these points (pos2 at scalar 0, pos 1 (the end point) at scalar 1)
                    found = false;
                    dist = 1000;
                    for (endpoint2 = allrailmappoints.begin(); endpoint2!=allrailmappoints.end() && endpoint2!=endpoint && !found; endpoint2++){
                        counterlines2.erase(counterlines2.begin(), counterlines2.end());
                        counterlines2 = allrailmaplines.ReturnLinesWithPoint(endpoint2->Number()); //select lines containing this point
                        if (counterlines2.size()==1) {                                            //if only one line, then this point is an end point
                        pos3 = endpoint2->Position3DRef();
                          if (pos3.Distance(pos1) < dist){
                          dist = pos3.Distance(pos1);
                          keeppoint = endpoint2;
                  //        perpdist = line.DistanceToPoint(pos3);
                          }
/*           //             if (line.Scalar(posl)> 1 && line.Scalar(posl)< 3 &&
                        if (fabs(line.Scalar(pos3))> 1 && pos3.Distance(pos1) < 1.5 &&
                           line.DistanceToPoint(pos3) < 0.3){
                           extendedline = LineTopology(1,3,endpoint->Number(), endpoint2->Number());
                           extendlines.push_back(extendedline);
                           allrailmaplines.push_back(extendedline);
                           found = true;
                        }
                        }
                        }
                        if (!found){
                      for (endpoint2 = allrailmappoints.begin(); endpoint2!=allrailmappoints.end() && endpoint2!=endpoint && !found; endpoint2++){
                        counterlines2.erase(counterlines2.begin(), counterlines2.end());
                        counterlines2 = allrailmaplines.ReturnLinesWithPoint(endpoint2->Number()); //select lines containing this point
                        if (counterlines2.size()==1) {                                            //if only one line, then this point is an end point
                        pos3 = endpoint2->Position3DRef();                        
                          if (fabs(line.Scalar(pos3))> 1 && pos3.Distance(pos1) < 2.5 &&
                            line.DistanceToPoint(pos3) < 0.4){
                            extendedline = LineTopology(1,4,endpoint->Number(), endpoint2->Number());
                            extendlines.push_back(extendedline);
                            allrailmaplines.push_back(extendedline);
                            found = true;
                          }
                          else {
  //                           if (line.Scalar(posl)> 3 && line.Scalar(posl)< 5 &&
                             if (pos1.Distance(pos3) < 3.5 &&
                                line.DistanceToPoint(pos3) < 0.5){
                                extendedline = LineTopology(1,5,endpoint->Number(), endpoint2->Number());
                                extendlines.push_back(extendedline);
                                allrailmaplines.push_back(extendedline);
                                found = true;
                             }
                           }
                        }
              */          }
                    } 
                    if (dist< 1){
                     extendedline = LineTopology(1,3,endpoint->Number(), keeppoint->Number());
                     extendlines.push_back(extendedline);
                     allrailmaplines.push_back(extendedline);      
                    } 
                 }
                 endlines.insert(endlines.end(), counterlines.begin(), counterlines.end());
   //              dirpoints.push_back(LaserPoint(pos1.X(), pos1.Y(), pos1.Z()));
                 
                 }
}

for (map_line=endlines.begin();map_line!=endlines.end(); map_line++) {
    for (mapnode = map_line->begin(); mapnode!=map_line->end(); mapnode++){
        selrailmappoints.push_back(*allrailmappoints.PointIterator(*mapnode));
        }
  }
//dirpoints.Write("endpoints.laser", false);
allrailmaplines.Write("allrailmaplines2.top", false);
allrailmappoints.Write("allrailmappoints2.objpts");
extendlines.Write("extendlines.top", false);
endlines.Write("endlines.top", false);
selrailmappoints.Write("endlines.objpts");

if (!railpoints.Write("railpoints.laser", false))
    printf("Error writing the test output laser points\n");  

if (!outpoints.Write(laser_output, false))
    printf("Error writing the output laser points\n");  
    return;
/*
for (point = points.begin(), i=0; point != points.end(); point++,i++){
 printf("%5.1f\% \r", 100.0*i/points.size());
  //   printf("point number = %d\n", point->GetPointNumber());
     neighbourhood = points.Neighbourhood(PointNumber(i), 5,
                                             edges, true, false);
     zmin = point->Z(); 
     for (node2=neighbourhood.begin(), pot2 = false; node2!=neighbourhood.end() && !pot2; node2++){          
         zp = points[node2->Number()].Z();
         if (zp<zmin){
           zmin = zp;
           minpoint = node2->Number();
           }
     }
     lowpoints.push_back(points[minpoint]);
  }
//  lowpoints.ReduceData(0.2);

if (!lowpoints.Write("lowpointstest.laser", false))
    printf("Error writing the test output laser points\n");  


//return;  
     
 */
for (point = points.begin(), i=0; point != points.end(); point++,i++){
 printf("%5.1f\% \r", 100.0*i/points.size());
 /* Accepted point, delete all points within specified distance */
  //   printf("point number = %d\n", point->GetPointNumber());
     neighbourhood = points.Neighbourhood(PointNumber(i), 1.6,
                                             edges, true, false);
   //  sel_laser_points.ErasePoints();                                        
//     for (node2=neighbourhood.begin(), pot =false; node2!=neighbourhood.end() && !pot; node2++){
//         if (abs(points[node2->Number()].Z()-points[i].Z()-5.5)<0.2){
//             pot = true;
//          }
//     }                                                          
//     if (pot){
         for (node2=neighbourhood.begin(), pot2 = false; node2!=neighbourhood.end() && !pot2; node2++){          
             dx = fabs(points[node2->Number()].X()-points[i].X());
             dy = fabs(points[node2->Number()].Y()-points[i].Y());
             dz = fabs(points[node2->Number()].Z()-points[i].Z());
             dist = sqrt(dx*dx + dy*dy);
             if (dist > 1.3 && dz < 0.2){
               pot2 = true;         
//             sel_laser_points.push_back(points[node2->Number()]);
             }
             
          }
 //       }
      if (pot2){
        for (node2=neighbourhood.begin(); node2!=neighbourhood.end(); node2++){
           if (fabs(points[node2->Number()].Z()-points[i].Z()-5.5)<0.1){
               points[node2->Number()].Label(8);
               points[i].Label(7);
               }
        }                                                          
        }
    //    }
 //    printf ("Size of sel_laser_points = %d\n", sel_laser_points.size());
 }
  for (point = points.begin(), i=0; point != points.end(); point++,i++){
  printf("%5.1f\% \r", 100.0*i/points.size());
    if (points[i].Label()==7){
     neighbourhood = points.Neighbourhood(PointNumber(i), 1.5,
                                             edges, false, false);
        for (node2=neighbourhood.begin(), pot2 = false; node2!=neighbourhood.end() && !pot2; node2++){          
             dz = points[i].Z()-points[node2->Number()].Z(); // if this point is higher than surrounding, label surrounding
             if (dz>0.05){
               points[node2->Number()].Label(6);
             }
        }
     }               
  }

  for (point = points.begin(), i=0; point != points.end(); point++,i++){
  printf("%5.1f\% \r", 100.0*i/points.size());
    if (points[i].Label()==8){
     neighbourhood = points.Neighbourhood(PointNumber(i), 0.3,
                                             edges, true, false);
        for (node2=neighbourhood.begin(), pot2 = false; node2!=neighbourhood.end() && !pot2; node2++){          
             dz = points[node2->Number()].Z()-points[i].Z(); //if other point is higher than 10 cm above this point, label as 'other wire'
             if (dz>0.1 && dz<3){
               points[node2->Number()].Label(9);
             }
        }
     }               
  }
points.ReduceData(0.01);
//sel_laser_points.ReduceData(0.01);
//printf ("\nSize of sel_laser_points = %d\n", sel_laser_points.size());
if (!points.Write(laser_output, false))
    printf("Error writing the output laser points\n");  
}
