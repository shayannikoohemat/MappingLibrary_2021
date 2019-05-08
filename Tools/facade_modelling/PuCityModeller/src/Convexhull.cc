
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


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <math.h>
#include "City.h"
#include "ObjectPoints.h"
#include "LineTopologies.h"
#include "LineSegment2D.h"
#include "LaserPatch.h"
#include "LaserPatches.h"
#include "Position3D.h"
#include "Lines2D.h"


#include <QMessageBox>

void City::CCH()
{

  CityWindow *window;
  
 
   LaserPoints lp,lp_tmp;
  
    Line2D line;
    Lines2D lines;

     
//    CreateConvexHull(laser_points,objpts_out,building_out, max_segment_value);
    
    laser_points.ConvexHull(objpts_out,building_out, max_segment_value);
    
    building_patches=LaserPatches(objpts_out, building_out);


  convexhulled=true;

  

}

 /*
  void City::CreateConvexHull(LaserPoints &lpts, ObjectPoints &objpts, LineTopologies &tops, int max)
  
  {
   Position3D p3d,a,b;
   LaserPatches lps_tmp;
   LaserPoints testpoints;
   double max_x=-65535, min_x=65535;
   double max_i=0, min_i=0;
   double tmp_min=0, tmp_max=0;
    Plane myplane;
    LineTopology Piece_out;
  ObjectPoint vertex_out;
  int obj_num=0;
  LineTopologies::const_iterator line_in, line_out;
  LineTopologies::const_iterator temp;
  int t=0;
   int i,vertical,j=0; 
    
   
   lps_tmp=LaserPatches(max+1);
   
   cout<<"There are "<<max<<" segments in total."<<endl;

 //start loop for all Pieces
   for(i=0;i<=max;i++){
   max_x=-32576999;
   min_x=33576999;
   max_i=0, min_i=0;
   tmp_min=0, tmp_max=0;
   
   testpoints=lpts.SelectTagValue(SegmentNumberTag,i);
   if(!testpoints.size())
   continue;
     
    //cout<<"No. "<<i<<" ";
   myplane=testpoints.FitPlane(i,i,SegmentNumberTag);  
   if(myplane.IsVertical(0.1))
   {vertical=1;
   //cout<<"Vertical ";
   }
   else vertical=0;

   lps_tmp[i].setPlane(myplane);

   for(j=0;j<testpoints.size();j++)  //make a segment more flat by projecting all points to plane
      {
      testpoints[j]=myplane.Project(testpoints[j]);
      }
   
   //cout<<"size: "<<testpoints.size()<<endl;
   for(j=0;j<testpoints.size();j++)   //Determine most left and right points
      {
         if((tmp_min=testpoints[j].GetX())<=min_x)
         {
               min_x=tmp_min;
               min_i=j;
         }
         if((tmp_max=testpoints[j].GetX())>=max_x)
         {
               max_x=tmp_max;
               max_i=j;
         }
       } 
   
    a= Position3D(testpoints[min_i].GetX(),testpoints[min_i].GetY(),testpoints[min_i].GetZ());
    b= Position3D(testpoints[max_i].GetX(),testpoints[max_i].GetY(),testpoints[max_i].GetZ());
   
   lps_tmp[i].push_back(a);
   lps_tmp[i].push_back(b);
   LaserPatch s1,s2;  
  
  
   for(j=0;j<testpoints.size();j++)
   {
       if((j!=min_i)&&(j!=max_i))
       {
          p3d=Position3D(testpoints[j].GetX(),testpoints[j].GetY(),testpoints[j].GetZ());
          if(((Vector2D)p3d).AreaSign(a,b)==-1)
              s1.push_back(p3d);
          else
              s2.push_back(p3d); 
       }
   }  
   FindHull(lps_tmp,s1,a,b,i,1,vertical);
   FindHull(lps_tmp,s2,b,a,i,0,vertical);  
  
  }
  
  testpoints.clear();
  cout<<"Find Hull complete"<<endl;
  
   objpts.clear();
 tops.clear();
 
 
  for(t=0;t<=max;t++)
  {
     if(lps_tmp[t].size()==0)
     continue;
                                                 
     Piece_out.clear();
     Piece_out.Label()=t;
  
     for(int t1=0;t1<lps_tmp[t].size();t1++)
     {
            
        vertex_out=ObjectPoint(lps_tmp[t][t1].GetX(),lps_tmp[t][t1].GetY(),lps_tmp[t][t1].GetZ(),obj_num,0,0,0,0,0,0);   
        Piece_out.push_back(PointNumber(obj_num));    
        objpts.push_back(vertex_out);
        obj_num++;
     }
    
     Piece_out.push_back(Piece_out[0]);          
    
     tops.push_back(Piece_out);     

          }
    // building_patches.clear();
    
     for(i=0;i<tops.size();i++) 
     tops[i].push_back(tops[i][0]);
  

}

void City::FindHull(LaserPatches &lps_tmp,LaserPatch &sk, Position3D P, Position3D Q, int num, int flag, int vertical)
{
    if(sk.size()==0)
        return;
    vector<Position3D>::iterator iter_tmp,iter_tmp2;
    LaserPatch s1,s2;
    Position3D C;   
    Vector2D vec,p1,q1,c1;
   
    Line2D l;
    if(vertical)  //if vertical, use XZ plane
    l=Line2D(Position2D(P.GetX(),P.GetZ()),Position2D(Q.GetX(),Q.GetZ()));
    else
    l=Line2D(Position2D(P.GetX(),P.GetY()),Position2D(Q.GetX(),Q.GetY()));
    
    double maxdistance=-32576,distance=0;
    vector<Position3D>::iterator iter1;
    
   
    for(iter=sk.begin();iter!=sk.end();iter++)
    {
      if(vertical)
      distance=l.DistanceToPoint(Position2D((*iter).GetX(),(*iter).GetZ()));
      else   
      distance=l.DistanceToPoint(Position2D((*iter).GetX(),(*iter).GetY()));
      if(distance>maxdistance) 
         {
          maxdistance=distance;
          iter_tmp=iter;
          }      
                                       
    }
    
    C=*iter_tmp;
    
    for(iter_tmp2=lps_tmp[num].begin();iter_tmp2!=lps_tmp[num].end();iter_tmp2++)
    {
    if(*iter_tmp2==P)
      {iter=iter_tmp2;iter++;break;}
    }
     
    
    lps_tmp[num].insert(iter,C);
  
    for(iter=sk.begin();iter!=sk.end();iter++)
    {
    if(iter!=iter_tmp)
    {
    if(vertical)
    { 
    vec=Vector2D((*iter).GetX(),(*iter).GetZ());
    p1=Vector2D(P.GetX(),P.GetZ());
    c1=Vector2D(C.GetX(),C.GetZ());
    q1=Vector2D(Q.GetX(),Q.GetZ());   
    }
    else
    {
    vec=(Vector2D)(*iter);
    p1=(Vector2D)P;
    c1=(Vector2D)C;
    q1=(Vector2D)Q;   
    }
    if(vec.AreaSign(p1,c1)==-1)
        s1.push_back(*iter);
        
    if(vec.AreaSign(c1,q1)==-1)
    s2.push_back(*iter);
       
 
    }    
    }
 
   
   
    FindHull(lps_tmp,s1,P,C,num,flag,vertical);
    FindHull(lps_tmp,s2,C,Q,num,flag,vertical);
}
*/

/*
void City::DeriveContour3D(LaserPoints &input_laser, ObjectPoints &contour_obj, LineTopology &contour_top, double max_edge_dist)
{

     ObjectPoint objpt;
     ObjectPoints objpts;
     PointNumberList        component;
     TINEdges edges;
     LineTopology           laser_contour;
     Vector3D point, direction;
     Vector3D vec1=Vector3D(0,0,1);
     Line3D horizontal_line;
     Position3D pt;
     Plane myplane;
     double angle;
     Rotation3D *rot1; 
     LaserPoints rotated;
      
     pt=input_laser[0];
     
     input_laser.Label(1);
     myplane=input_laser.FitPlane(1,1,LabelTag);
     
      for(int j=0;j<input_laser.size();j++)
     {
        point=myplane.Project(input_laser[j]); 
        input_laser[j].SetX(point.X());  
        input_laser[j].SetY(point.Y());  
        input_laser[j].SetZ(point.Z());     
     }
     
     objpts=input_laser.ConstructObjectPoints();
      
     Plane horizontal_plane(pt,vec1);
     
    
     
     
     Intersect2Planes(horizontal_plane, myplane, horizontal_line);
     angle=Angle(myplane.Normal(), vec1);
     
     rot1=new Rotation3D(horizontal_line.Direction(), angle);    
     
     rotated=input_laser;
     
     for(int j=0;j<rotated.size();j++)
     {      
        point=(rot1->Transpose ()).Rotate(rotated[j]);
        rotated[j].SetX(point.X());
        rotated[j].SetY(point.Y());
        rotated[j].SetZ(point.Z());
        }
     
     rotated.DeriveTIN();
     rotated.DeriveDataBounds(0);
     edges.Derive(rotated.TINReference()); 
     rotated.RemoveLongEdges(edges, max_edge_dist);
     
     component.clear();
     
     for (int j=0; j<rotated.size(); j++) 
         component.push_back(PointNumber(j));
     
     contour_top = rotated.DeriveContour(1, component, edges, false);
     
     contour_obj.clear();
     for(int j=0;j<contour_top.size();j++)
      {
             objpt=objpts.PointByNumber(contour_top[j].Number());
             contour_obj.push_back(objpt);
      }
}

*/
