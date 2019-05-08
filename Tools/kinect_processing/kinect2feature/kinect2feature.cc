
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


/* Conversion of kinect data to object features
 *
 * Initial creation:
 * Author: Sander Oude Elberink
 * Date  : 10-11-2012
 */

#include <stdio.h>
#include "LaserPoints.h"

void kinect2feature(char *ascii_file, char *laseroutfile)
{
  LaserPoints  pointset, sel_laser_points, bodypoints;
        LaserPoints groundset, notgroundset, planeset;

  LaserPoint                   point, keepmaxpoint;
  LaserPoints::iterator        laser_point;
  ObjectPoint                  leftp, rightp, bottomp, topp;
  ObjectPoints                 mappoints;
  LineTopology                 widthline, lengthline;
  LineTopologies               lines;
  FILE                         *ascii;
  char                         line[2048], *name, *comma;
  int                          i, double_points, total_points, keepsegvalue,
                               bodynr, index1=0, largesnr, count;
  double                       value[4], maxh, maxheight, groundheight,
                               bodywidth, bodyheight;
  DataBounds3D                 bounds;
  TINEdges                     *edges;
  Position3D                   leftpointpos, rightpointpos, bottompos, toppos;
  Plane                        plane;
  Covariance3D                 cov3d;
  cov3d = Covariance3D(0,0,0,0,0,0);
  bool                         option2 =true;
  bool                         option1 =false;

/* Read the laser points */
 ascii = Open_Compressed_File(ascii_file, "r");
  if (!ascii) {
    fprintf(stderr, "Error opening input file %s\n", ascii_file);
    exit(0);
  }
double max_range = 300; // this is the range in cm between sensor and AOI
double min_range = 100;
maxheight = 0;
//read in ascii laser points, transfer to laser format
  total_points = double_points = 0;
  do {
      index1++;
    if (fgets(line, 1024, ascii)) {
    if (index1 == (index1/2)*2) {
 sscanf(line, "%lf %lf %lf",
             value+ 1, value+ 2, value+ 3);

// Copy the data to the laser point

      point.X() = value[2] ;
      point.Y() = value[3] ; //change depth to y
      point.Z() = -value[1] ; //now height of a person is in z direction

      if (point.Y() < max_range && point.Y()>min_range) {
         pointset.push_back(point);         
         }
      }
      }
      } while (!feof(ascii));

  SegmentationParameters *segpar;
  segpar=new SegmentationParameters();
  segpar->MaxDistanceInComponent()=10;
  segpar->SeedNeighbourhoodRadius()=10;
  
  segpar->GrowingRadius()=10;
  segpar->MaxDistanceSurface()=3;
   
//      bool option1;
      if (option1){
      
      pointset.DeriveDataBounds(0);
      groundheight = pointset.DataBounds().Minimum().Z();
      maxheight = groundheight;
      for (laser_point = pointset.begin();laser_point!=pointset.end();laser_point++){
           if (laser_point->Z()>groundheight+20 && laser_point->Z()<groundheight+250){
           laser_point->Attribute(LabelTag) = 2;
          }
          if (laser_point->Z()<groundheight+20)laser_point->Attribute(LabelTag) = 1;                                
      }
      
      
      pointset.SurfaceGrowing(*segpar);     
      groundset.AddTaggedPoints(pointset, 1, LabelTag); 

      largesnr = groundset.MostFrequentAttributeValue(SegmentNumberTag, count);
      plane = groundset.FitPlane(largesnr, largesnr, SegmentNumberTag);
      pointset.RemoveTaggedPoints(largesnr, SegmentNumberTag);

      for (laser_point = pointset.begin();laser_point!=pointset.end();laser_point++){
           if (laser_point->X()>-20 && laser_point->X()<50){
             laser_point->Attribute(LabelTag) = 3;
             if (laser_point->Z()>maxheight){
                  maxheight = laser_point->Z();
                  keepmaxpoint = *laser_point;
                  keepsegvalue = laser_point->Attribute(SegmentNumberTag);
                }
            }
            }
            bodypoints.AddTaggedPoints(pointset, 3, LabelTag);
            bodynr = bodypoints.MostFrequentAttributeValue(SegmentNumberTag, count);
            if (keepsegvalue == bodynr) sel_laser_points.AddTaggedPoints(pointset, keepsegvalue, SegmentNumberTag);
            else sel_laser_points.AddTaggedPoints(pointset, bodynr, SegmentNumberTag);
//  printf("maxheight = %5.1f cm\n", maxheight - groundheight);
//      pointset.SwapYZ(); // to directly switch to kinectview
      sel_laser_points.DeriveDataBounds(0);
      sel_laser_points.SortOnCoordinates();
      pointset.RemoveTaggedPoints(bodynr, SegmentNumberTag);
      leftpointpos = Position3D(sel_laser_points[0].X(),sel_laser_points[0].Y(),sel_laser_points[0].Z()); 
      rightpointpos = Position3D(sel_laser_points[sel_laser_points.size()-1].X(),sel_laser_points[sel_laser_points.size()-1].Y(),sel_laser_points[sel_laser_points.size()-1].Z()); 
      
//      bodyheight = sel_laser_points.DataBounds().Maximum().Z()-groundheight;
 
  //    groundheight = plane.Z_At(keepmaxpoint.X(), keepmaxpoint.Y(), &success);
      bottompos = plane.Project(keepmaxpoint.pos());
      toppos = keepmaxpoint.pos();
      bodyheight = toppos.Distance(bottompos);
      bodywidth = leftpointpos.Distance(rightpointpos);//sel_laser_points.DataBounds().Maximum().X() -sel_laser_points.DataBounds().Minimum().X();
      printf("bodyheight = %5.1f cm width %5.1f\n", bodyheight, bodywidth);
     leftp = ObjectPoint(leftpointpos, PointNumber(1), cov3d);
     rightp = ObjectPoint(rightpointpos, PointNumber(2), cov3d);
     topp = ObjectPoint(toppos, PointNumber(3), cov3d);
     bottomp = ObjectPoint(bottompos, PointNumber(4), cov3d);
     widthline = LineTopology(1,2, PointNumber(1),PointNumber(2));
     lengthline = LineTopology(1,2, PointNumber(3),PointNumber(4));
     lines.push_back(widthline);
     lines.push_back(lengthline);
     mappoints.push_back(leftp);
     mappoints.push_back(rightp);
     mappoints.push_back(topp);
     mappoints.push_back(bottomp);
     pointset.RemoveAttributes();
    sel_laser_points.AddPoints(groundset);
    pointset.AddPoints(sel_laser_points);
    mappoints.Write("mappoints.objpts");
    lines.Write("lines.top",false);
    pointset.Write(laseroutfile,false);
    return;
    }
    if (option2){
      pointset.DeriveDataBounds(0);
      groundheight = pointset.DataBounds().Minimum().Z();
      maxheight = groundheight;
      for (laser_point = pointset.begin();laser_point!=pointset.end();laser_point++){
           if (laser_point->Z()>groundheight+20 && laser_point->Z()<groundheight+250){ //specify some regions to label points, this can be rough
             laser_point->Attribute(LabelTag) = 2;
             if (laser_point->X()>-20 && laser_point->X()<50)laser_point->Attribute(LabelTag) = 3;          
             }
            if (laser_point->Z()<groundheight+20)laser_point->Attribute(LabelTag) = 1;
      }
      groundset.AddTaggedPoints(pointset, 1, LabelTag); 
      groundset.SurfaceGrowing(*segpar);     //segment the points with label one (between lowest point and +20cm)
      largesnr = groundset.MostFrequentAttributeValue(SegmentNumberTag, count);
      plane = groundset.FitPlane(largesnr, largesnr, SegmentNumberTag);
      planeset.AddTaggedPoints(groundset, largesnr, SegmentNumberTag);
      groundset.RemoveTaggedPoints(largesnr, SegmentNumberTag);
      groundset.RemoveAttributes();
      pointset.RemoveTaggedPoints(1, LabelTag);
      pointset.AddPoints(groundset);
      edges = pointset.DeriveEdges(*segpar);
      pointset.RemoveLongEdges(edges->TINEdgesRef(), 
                       segpar->MaxDistanceInComponent(),
                       false);

      pointset.LabelComponents(edges->TINEdgesRef(), SegmentNumberTag);//do connected component
      bodypoints.AddTaggedPoints(pointset, 3, LabelTag); //only keep points with label 3 (mid/mid in scene)
      bodynr = bodypoints.MostFrequentAttributeValue(SegmentNumberTag, count); //to get the most frequent component
      sel_laser_points.AddTaggedPoints(pointset, bodynr, SegmentNumberTag); //this component represents the body
      pointset.RemoveTaggedPoints(bodynr, SegmentNumberTag);
    
      sel_laser_points.DeriveDataBounds(0);
      sel_laser_points.SortOnCoordinates();
      leftpointpos = Position3D(sel_laser_points[0].X(),sel_laser_points[0].Y(),sel_laser_points[0].Z()); 
      rightpointpos = Position3D(sel_laser_points[sel_laser_points.size()-1].X(),sel_laser_points[sel_laser_points.size()-1].Y(),sel_laser_points[sel_laser_points.size()-1].Z()); 
      bodywidth = leftpointpos.Distance(rightpointpos);//sel_laser_points.DataBounds().Maximum().X() -sel_laser_points.DataBounds().Minimum().X();
      sel_laser_points.SwapXZ();
      sel_laser_points.SortOnCoordinates();
      toppos = Position3D(sel_laser_points[sel_laser_points.size()-1].Z(),sel_laser_points[sel_laser_points.size()-1].Y(),sel_laser_points[sel_laser_points.size()-1].X());           
      sel_laser_points.SwapXZ();
      bottompos = plane.Project(toppos);
      bodyheight = toppos.Distance(bottompos);
      printf("LENGTE: = %5.1f cm\n BREEDTE: = %5.1f\n AAPFACTOR = %5.2f!!\n", bodyheight, bodywidth, bodywidth/bodyheight);
      leftp = ObjectPoint(leftpointpos, PointNumber(1), cov3d);
      rightp = ObjectPoint(rightpointpos, PointNumber(2), cov3d);
      topp = ObjectPoint(toppos, PointNumber(3), cov3d);
      bottomp = ObjectPoint(bottompos, PointNumber(4), cov3d);
      widthline = LineTopology(1,2, PointNumber(1),PointNumber(2));
      lengthline = LineTopology(1,2, PointNumber(3),PointNumber(4));
      lines.push_back(widthline);
      lines.push_back(lengthline);
      mappoints.push_back(leftp);
      mappoints.push_back(rightp);
      mappoints.push_back(topp);
      mappoints.push_back(bottomp);
      pointset.RemoveAttributes();
      sel_laser_points.AddPoints(planeset);
      sel_laser_points.AddPoints(groundset);
      pointset.AddPoints(sel_laser_points);
      mappoints.Write("mappoints.objpts");
      lines.Write("lines.top",false);

      pointset.Write(laseroutfile,false);
      sel_laser_points.Write(laseroutfile,false);
      return;
      }
}
