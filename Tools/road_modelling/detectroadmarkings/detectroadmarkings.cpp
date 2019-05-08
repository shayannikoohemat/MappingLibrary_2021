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
 Date   : 26-08-2008

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/


#include <stdlib.h>
#include "LaserPoints.h"
#include "DataGrid.h"
#include <LineSegment2D.h> 

/*
--------------------------------------------------------------------------------
                      The main function
--------------------------------------------------------------------------------
*/

void detectroadmarkings(char *laser_input, char *laser_output)
{
   PointNumberList                     neighbourhood;
   PointNumberList::iterator           node, nb_node, node2,noden;
   TINEdges                            edges;
   LaserPoints::iterator               point;
   LaserPoints                         points, sel_laser_points, outpoints, roadmarkpoints,
                                       histopoints, selhistopoints;
   LaserPoint                          histopoint;
   int                                 count0, thresholdvalue, same, notsame, i,
                                       highref, localthresholdvalue;
   double                              percvalue10, value, averagehits, rico,
                                       prevrico;
   vector<int>                         reflectancevalues, histovalues;
   vector<int>::iterator               refvalue;
   vector<int>                         histovector;
   Histogram                           histo(100, 0.0, 1000.0);
   Position3D                          pos;
   double grid_size = 3;
  double min_x, max_x, min_y, max_y, min_z, max_z, x,y, meanheight, prevx, prevy;
  bool                                 found;
  int numcols, numrows, binnumber, pointnumber;
  LineTopology top, ricotop;
  LineTopologies tops;
  ObjectPoints objpts, selobjpts;
  ObjectPoint  objpt;
  pointnumber = 0;
  if (!points.Read(laser_input)) {
    printf("Error reading laser points from file %s\n", laser_input);
    exit(0);
  }
  
   points.SetAttribute(IsProcessedTag, 0); /* Set all points unprocessed */
   points.SetUnFiltered(); /* Accept all points */
   points.SetAttribute(LabelTag, 0);
   points.DeriveDataBounds(0);
 //  points.DeriveTIN();
 //  edges.Derive(points.TINReference());
   
 /* Process all nodes */
 max_x =  (points.DataBounds().Maximum().X());
 max_y =  (points.DataBounds().Maximum().Y());
 min_x =  (points.DataBounds().Minimum().X());
 min_y =  (points.DataBounds().Minimum().Y());
 min_z =  (points.DataBounds().Minimum().Z());
 max_z =  (points.DataBounds().Maximum().Z());
 numcols =  int ((max_x-min_x)/grid_size)+1;
 numrows =  int ((max_y-min_y)/grid_size)+1;
 count0 = 0;
 printf("min_x: %4.2f, min_y: %4.2f\n", min_x, min_y);
 printf("max_x: %4.2f, max_y: %4.2f\n", max_x, max_y);
 prevx = min_x;
 prevy = min_y;
 for (x=min_x; x<max_x; x+=(grid_size)){
    prevy = min_y;
    for (y=min_y; y<max_y; y+=(grid_size)){
        count0++;
   
        printf("%4.2f %4.2f, %4.2f \n", x, y, count0*100.0/(numcols*numrows));
        sel_laser_points.ErasePoints();
        histo.Clear();
        for (point = points.begin(); point != points.end(); point++){
            if ((point->X() < x+0.5*grid_size) && (point->X() > x-0.5*grid_size)){// here the points are selected that are in a particular grid cell
               if ((point->Y()< y+0.5*grid_size) && (point->Y() > y-0.5*grid_size)){
                  sel_laser_points.push_back(*point);
               }
            }
        }
        if (sel_laser_points.size()>10){
        // get the reflectance values, put them in a vector
        meanheight = sel_laser_points.Mean()[2];
        reflectancevalues.erase(reflectancevalues.begin(), reflectancevalues.end());// = sel_laser_points.AttributeValues(ReflectanceTag);
        for (point = sel_laser_points.begin(); point != sel_laser_points.end(); point++) reflectancevalues.push_back(point->Attribute(ReflectanceTag));
        sort(reflectancevalues.begin(), reflectancevalues.end());
  /*      for (point = sel_laser_points.begin(); point != sel_laser_points.end(); point++){
                value = point->Attribute(ReflectanceTag); 
                histo.Add(value);             
                }
        histo.Write("histogram.xv");                
    */    // add here code that analyses the reflectance values, in order to get a proper threshold value
        histovector.erase(histovector.begin(), histovector.end());
        histovector.resize(100);
        highref=0;
        for (refvalue = reflectancevalues.begin(); refvalue < reflectancevalues.end(); refvalue++){
            binnumber = int((*refvalue)/10);
            if (binnumber>99) binnumber = 99;
            histovector[binnumber]++;
            if (binnumber>25) highref++;
        }
     selhistopoints.ErasePoints();
       selobjpts.erase(selobjpts.begin(), selobjpts.end());
       top.clear();
       top.Number() = count0;
       top.Attribute(LineLabelTag) = count0;
       ricotop.clear();
       ricotop.Number()=count0;
       ricotop.Attribute(LineLabelTag) = 1;
       prevrico = 0;
       found = false;
       for (i=2; i<histovector.size()-2;i++){
           averagehits = (histovector[i-2]+histovector[i-1]+histovector[i]+histovector[i+1]+histovector[i+2])/5;
           averagehits = 1.0*(histovector[i-1]+histovector[i]+histovector[i+1])/3;
           rico = histovector[i]+histovector[i+1]+histovector[i+2] - (histovector[i-2]+histovector[i-1]+histovector[i]);
           rico = histovector[i+2]+ 2*histovector[i+1] - 2*histovector[i-1]-histovector[i-2];
           if (prevrico<0 && rico>0 && !found) {
                    ricotop.Attribute(LineLabelTag) = 0;
                    histopoint = LaserPoint(i, histovector[i] , count0);
                    histopoint = LaserPoint(x -0.25*grid_size + 1.0*i/50, y + 1.0*averagehits/100, 1.0*i/10);
                    histopoint.Label(count0);
                    selhistopoints.push_back(histopoint);
                    localthresholdvalue = i*10;
                    found = true;
                    printf("%4.2f %4.2f, %4.2f %d\n", x, y, count0*100.0/(numcols*numrows), i*10);
           }
           if (found){
    //                  highref = highref+histovector[i];
                      }
           pointnumber++;
           top.push_back(PointNumber(pointnumber));
           pos = Position3D(x -0.25*grid_size + 1.0*i/50, y + 1.0*averagehits/100, meanheight);
           objpt = ObjectPoint(pos, PointNumber(pointnumber), Covariance3D(0,0,0,0,0,0));
     //      objpt = histopoint.ConstructObjectPoint(PointNumber(pointnumber), Covariance3D(0,0,0,0,0,0));
           selobjpts.push_back(objpt);
           pointnumber++;
           ricotop.push_back(PointNumber(pointnumber));
           pos = Position3D(x -0.25*grid_size + 1.0*i/50, y + 1.0*rico/200, meanheight);
           objpt = ObjectPoint(pos, PointNumber(pointnumber), Covariance3D(0,0,0,0,0,0));
     //      objpt = histopoint.ConstructObjectPoint(PointNumber(pointnumber), Covariance3D(0,0,0,0,0,0));
           selobjpts.push_back(objpt);
           prevrico = rico;
           }
           printf("%4.2f %4.2f, %4.2f %4.2f\n", x, y, count0*100.0/(numcols*numrows), 1.0*highref/sel_laser_points.size());
   //     if ((1.0*highref/sel_laser_points.size()) > 0.05) {
     if (found) {
           histopoints.AddPoints(selhistopoints);
           tops.push_back(top);
      //     tops.push_back(ricotop);
           objpts.insert(objpts.end(), selobjpts.begin(), selobjpts.end());
           }
        //thresholdvalue = ...
        
        thresholdvalue = 220; // for the moment just a fixed value
        if (found) thresholdvalue = localthresholdvalue;
        // trick to get sort the points based on heights
        sel_laser_points.SwapXZ();
        sel_laser_points.SortOnCoordinates();
        sel_laser_points.SwapXZ();
        // get the 10th percentile height (often represents a height on the ground)
        percvalue10 = sel_laser_points[int(sel_laser_points.size()/10)].Z();
//        printf(" 10th perc: %4.2f", percvalue);
        for (point = sel_laser_points.begin(); point != sel_laser_points.end(); point++){
            //    point->SetAttribute(PlaneNumberTag, int(x+y)); //just for understanding the grid size; all points within a grid cell get same planenumber tag
                point->SetAttribute(PlaneNumberTag, count0); //just for understanding the grid size; all points within a grid cell get same planenumber tag
                if(point->Z() >= percvalue10 + 0.1) {
                   point->Label(10); //point is higher than 0.1 meter above ground, set label to 10
                }
                else { // if near to the ground, look at reflectance value...
                     if (point->Attribute(ReflectanceTag)>thresholdvalue) point->Label(1); // if reflectance value larger than "threshold", label to 1
                 //    if (point->Attribute(ReflectanceTag)>thresholdvalue_min && point->Attribute(ReflectanceTag)<thresholdvalue_max) point->Label(1); // if reflectance value larger than "threshold", label to 1
                     }
             }
        outpoints.AddPoints(sel_laser_points);        
        }
        }
    }          
    roadmarkpoints.ErasePoints();
    roadmarkpoints.AddTaggedPoints(outpoints, 1, LabelTag);
    outpoints.DeriveTIN();
   edges.Derive(outpoints.TINReference());
   printf("\n");
   if (!histopoints.Write("histotest.laser", false))
    printf("Error writing the output laser points\n");  
    objpts.Write("histo.objpts");
    tops.Write("histo.top", false);
//}
//return;
    for (point = outpoints.begin(), i=0; point != outpoints.end(); point++,i++){
     printf("%5.1f\% \r", 100.0*i/points.size());
     if (point->Label()==1){
     neighbourhood = outpoints.Neighbourhood(PointNumber(i), 0.2,
                                             edges, true, false);
     for (node2=neighbourhood.begin(), same = 0, notsame = 0; node2!=neighbourhood.end(); node2++){
         if (outpoints[node2->Number()].Label()==1 || outpoints[node2->Number()].Label()==2) same++;
         else notsame++;
     }
     if (notsame > 5*same) point->Label(2);
     }
     else {
          if (point->Label()==0 && point->Attribute(ReflectanceTag)>thresholdvalue -50){
             neighbourhood = outpoints.Neighbourhood(PointNumber(i), 0.2,
                                             edges, true, false);
             for (node2=neighbourhood.begin(), same = 0, notsame = 0; node2!=neighbourhood.end(); node2++){
               if (outpoints[node2->Number()].Label()==1) same++;
               else notsame++;
             }
             if (2*notsame < same) point->Label(5);                   
           }
          }
     }
 // }

if (!outpoints.Write(laser_output, false))
//if (!histopoints.Write(laser_output, false))
    printf("Error writing the output laser points\n");  
}
