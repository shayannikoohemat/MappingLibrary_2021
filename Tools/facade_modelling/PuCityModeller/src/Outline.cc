
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
#include <time.h>
#include "ObjectPoints.h"
#include "LineTopologies.h"
#include "LineSegment2D.h"
#include "LaserPatch.h"
#include "LaserPatches.h"
#include "Position3D.h"
#include "Lines2D.h"
#include <QProgressDialog>
#include <QSound>
#include <math.h>
#include <QMessageBox>


bool extending=true;

void City::CreateExtrusion()
{
     int seg1, seg2, seg3, type; //type =1 means direct project, type=2 means 2 segments combine
     LaserPoints lpts1, lpts2,lptw;
     int count_obj=0;
    ObjectPoints objpts;
    LineTopologies tops;
    ObjectPoint  left_low,left_high,right_low,right_high, middle_low, middle_high,tmp;
    double max_z, min_z;
    int max_z_index, min_z_index;
    LaserPoints lpts;
    
    lpts=extrusion_win->selected_laser_points;
    
     seg1=lpts[0].Attribute(SegmentNumberTag);
     seg2=-1;
     LineTopology top;
     
     cout<<"create extrusion."<<endl;
     
      type=1;
  for(int i=0;i<lpts.size();i++)
      {  
         if(lpts[i].Attribute(SegmentNumberTag)!=seg1&&type<=1)
           {
             seg2=lpts[i].Attribute(SegmentNumberTag);
             type++;
           }
         if(lpts[i].Attribute(SegmentNumberTag)!=seg1&&lpts[i].Attribute(SegmentNumberTag)!=seg2)
           {
           seg3=lpts[i].Attribute(SegmentNumberTag);
           type++;
           }
      }         
     Positions2D pts2d;
                          Line2D l2d_left, l2d_right;
                          Line3D line3d;
                          Plane plane_left, plane_right;
                          Position2D left2d, right2d, middle2d;
                          Position3D pt;
     
     
     switch(type){
                  case 2: 
                          cout<<"2 segments:  "<<seg1<<" "<<seg2<<endl;
                          lpts1=lpts.SelectTagValue(SegmentNumberTag, seg1);
                          lpts2=lpts.SelectTagValue(SegmentNumberTag, seg2);
                          //distance_min=3276800;
                          max_z=-3276800;min_z=3276800;

                    
                          for(int j=0;j<lpts.size();j++)//determin max and min z
                          {                                        
                               if(lpts[j].GetZ()<min_z)
                               {min_z=lpts[j].GetZ();min_z_index=j;}
                               if(lpts[j].GetZ()>max_z)
                               {max_z=lpts[j].GetZ();max_z_index=j;}                                            
                          }
                          
                          count_obj=cwindow_objpts.size();
                          
                          //determin left side intersection with wall
                          /*
                          pts2d.clear();
                          for(int j=0;j<lpts1.size();j++)//determin max and min z
                          {                                     
                                pts2d.push_back(Position2D(lpts1[j].GetX(),lpts1[j].GetY()));                                           
                          }                                          
                          l2d_left=Line2D(pts2d);
                          Intersection2Lines(l2d_left, GBKN, left2d);
                          */
                          plane_left=lpts1.FitPlane( seg1, seg1,SegmentNumberTag);
                          Intersect2Planes(wall,plane_left,line3d);
                          /*
                          left_high=ObjectPoint(left2d.GetX(),left2d.GetY(),max_z, count_obj++, 0,0,0,0,0,0);
                          left_low=ObjectPoint(left2d.GetX(),left2d.GetY(),min_z,count_obj++, 0,0,0,0,0,0);
                          */
                          pt=line3d.DetPositionZ(max_z);
                          left_high=ObjectPoint(pt.GetX(),pt.GetY(),max_z, count_obj++, 0,0,0,0,0,0);
                          pt=line3d.DetPositionZ(min_z);
                          left_low=ObjectPoint(pt.GetX(),pt.GetY(),min_z,count_obj++, 0,0,0,0,0,0);
                          
                          //determin right side intersection with wall
                          /*
                          pts2d.clear();
                          for(int j=0;j<lpts2.size();j++)//determin max and min z
                          {                                     
                                pts2d.push_back(Position2D(lpts2[j].GetX(),lpts2[j].GetY()));                                                             
                          }                                      
                          l2d_right=Line2D(pts2d);
                          Intersection2Lines(l2d_right, GBKN, right2d);
                          right_high=ObjectPoint(right2d.GetX(),right2d.GetY(),max_z, count_obj++, 0,0,0,0,0,0);
                          right_low=ObjectPoint(right2d.GetX(),right2d.GetY(),min_z, count_obj++, 0,0,0,0,0,0);
                          */
                          plane_right=lpts2.FitPlane(seg2, seg2,SegmentNumberTag);
                          Intersect2Planes(wall,plane_right,line3d);
                          pt=line3d.DetPositionZ(max_z);
                          right_high=ObjectPoint(pt.GetX(),pt.GetY(),max_z, count_obj++, 0,0,0,0,0,0);
                          pt=line3d.DetPositionZ(min_z);
                          right_low=ObjectPoint(pt.GetX(),pt.GetY(),min_z,count_obj++, 0,0,0,0,0,0);
                          
                          //determin 2 segment intersection
                          /*
                          Intersection2Lines(l2d_right, l2d_left, middle2d);
                          middle_high=ObjectPoint(middle2d.GetX(),middle2d.GetY(),max_z, count_obj++, 0,0,0,0,0,0);
                          middle_low=ObjectPoint(middle2d.GetX(),middle2d.GetY(),min_z, count_obj++, 0,0,0,0,0,0);
                          */
                          Intersect2Planes(plane_left,plane_right,line3d);
                          pt=line3d.DetPositionZ(max_z);
                          middle_high=ObjectPoint(pt.GetX(),pt.GetY(),max_z, count_obj++, 0,0,0,0,0,0);
                          pt=line3d.DetPositionZ(min_z);
                          middle_low=ObjectPoint(pt.GetX(),pt.GetY(),min_z,count_obj++, 0,0,0,0,0,0);
                          
                          //push all the objectpoints
                          cwindow_objpts.push_back(left_low);cwindow_objpts.push_back(left_high);
                          cwindow_objpts.push_back(right_low);cwindow_objpts.push_back(right_high);
                          cwindow_objpts.push_back(middle_low);cwindow_objpts.push_back(middle_high);
                          
                          //push all the tops
                          top=LineTopology();
                          top.push_back(left_high.Number());top.push_back(left_low.Number());
                          top.push_back(middle_low.Number());top.push_back(middle_high.Number());top.push_back(left_high.Number());
                          top.SetAttribute(TextureTag,1);
                          cwindow_tops.push_back(top);
     
                          top=LineTopology();
                          top.push_back(right_low.Number());top.push_back(right_high.Number());
                          top.push_back(middle_high.Number());top.push_back(middle_low.Number());top.push_back(right_low.Number());
                          top.SetAttribute(TextureTag,1);
                          cwindow_tops.push_back(top);
                          
                          top=LineTopology();
                          top.push_back(left_high.Number());top.push_back(middle_high.Number());
                          top.push_back(right_high.Number());top.push_back(left_high.Number());
                          cwindow_tops.push_back(top);
                          
                          top=LineTopology();
                          top.push_back(left_low.Number());top.push_back(right_low.Number());top.push_back(middle_low.Number());
                          top.push_back(left_low.Number());
                          cwindow_tops.push_back(top);
                          
                          break;
     
                  case 1: 
                       
                       LineTopology top_contour;
                       lpts.DeriveContour3D(objpts, top_contour, 0.2);
                       
                       top_contour.Smooth_Outline_3D(objpts);
       
                       
                       //tops.push_back(top);
                       
                       int first1, first2;
                       Position3D p3d;
                       first1=count_obj=cwindow_objpts.size();
                       top.clear();
                       for(int j=0;j<top_contour.size();j++)//push back the front facade
                       {
                               tmp=objpts.PointByNumber(top_contour[j].Number());
                               tmp.Number()=count_obj++;
                               tmp.Covar(0)=top_contour.size();
                               top.push_back(tmp.Number());
                               cwindow_objpts.push_back(tmp);
                       }
                       top.push_back(PointNumber(first1));
                       top.SetAttribute(TextureTag,1);
                       
                     //CCW managment
                       Vector3D direction=GBKN3D.Direction().Normalize();

                       Vector3D vec1=Vector3D(0,0,1);
                       double angle=Angle(vec1,GBKN3D.Direction());

                       Rotation3D *rot1=new Rotation3D(direction, angle);

                       ObjectPoints objpts_rotated; 

                       Position3D point;

                       objpts_rotated=cwindow_objpts;

                       for(int i=0;i<cwindow_objpts.size();i++)
                       {
                       point=(rot1->Transpose ()).Rotate(objpts[i]);
                       objpts_rotated[i].SetX(point.GetX());
                       objpts_rotated[i].SetY(point.GetY());
                       }

                       if(top.IsClockWise(objpts_rotated)) //CCW managment
                       {
                       top.RevertNodeOrder();
                       cout<<"extrusion facade clockwise, resvert."<<endl;
                       
                       } 
                   
                       
                     //finish of CCW management  
                     
                       cwindow_tops.push_back(top);          
                        
                       first2=cwindow_objpts.size();
                       top.clear();
                       for(int j=0;j<top_contour.size();j++) //push back the side faces
                       {
                               p3d=wall.Project(objpts.PointByNumber(top_contour[j].Number()));//projected point
                               tmp=ObjectPoint(p3d.GetX(),p3d.GetY(),p3d.GetZ(),count_obj++, 0,0,0,0,0,0);
                               tmp.Covar(0)=-top_contour.size();
                               top.push_back(tmp.Number());
                               cwindow_objpts.push_back(tmp);
                       }
                  
                       for(int j=0;j<top_contour.size()-1;j++)
                       {
                               top=LineTopology();
                               top.push_back(cwindow_objpts[first1+j].Number());
                               top.push_back(cwindow_objpts[first1+j+1].Number());
                               top.push_back(cwindow_objpts[first2+j+1].Number());
                               top.push_back(cwindow_objpts[first2+j].Number());
                               top.push_back(cwindow_objpts[first1+j].Number());
                               cwindow_tops.push_back(top);    
                       }
                       top.clear();
                       top.push_back(cwindow_objpts[first1+top_contour.size()-1].Number());
                       top.push_back(cwindow_objpts[first1].Number());
                       top.push_back(cwindow_objpts[first2].Number());
                       top.push_back(cwindow_objpts[first2+top_contour.size()-1].Number());
                       top.push_back(cwindow_objpts[first1+top_contour.size()-1].Number());
                       cwindow_tops.push_back(top);    
                             
                       break;
                          
                  }

    
     extrusion_win->map_points=cwindow_objpts;
     extrusion_win->Canvas()->ClearObjectData(true);
     extrusion_win->Canvas()->AddObjectData(&cwindow_objpts,&cwindow_tops,appearance[MapData]->DataAppearancePtr(), false, false);
     extrusion_win->Canvas()->update();
     
     
}


void City::FitWallLine()
{    
    ObjectPoint objpt;
    int line_min_point=3;
    walloutline_laser.clear();
    Line3D line;
    Line2D line2d;
    Line3D line3d;
   
    Position3D startpoint;
    Positions2D contour2D;
   
    //Wall outline
  
    int i=0;
    int j=0;
    int counter=0;
 
    PointNumberList plist;
    
    Vector3D normal;
    bool isVertical;
    
    if(!DeriveWallContour())
       DeriveWallContour2();
    contour.Label(0);
   
   
    //contour.Write("contour.laser",0);
    vector<int> contournumber;
   
    
    
    isVertical=wall.IsVertical(0.1);
 
    outline_obj.clear();
    outline_tops.clear();
   
    Position2D pt2D;
    Positions2D pts2D;
    Position3D point;

 
    line=Line3D(contour[0],contour[1]);
     //push the first point
    contour[0].Attribute(SegmentNumberTag)=1;
    walloutline_laser.push_back(contour[0]);
  
    

    startpoint=(Position3D)contour[0];
    counter=1;
    Position2D pt;
    Line3D line_tmp;
    double angle;
    int size;
    
    
    for(i=2;i<contour.size();i++)  //start analyzing all points
    {
    
    
    line_tmp=Line3D(contour[i-1],contour[i]);
    angle=abs(Angle2Lines(line_tmp, line)*180/PI);
    if(!line.PointOnLine(contour[i],terrestrial_parameters->dist_accurate))
    { 
        //cout<<"one line found!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! by fitting "<<counter<<" points."<<endl;
        contour[i-1].Attribute(SegmentNumberTag)=1;//confident vertice
        walloutline_laser.push_back(contour[i-1]); 
        contour[i].Attribute(SegmentNumberTag)=1;//confident vertices     
        walloutline_laser.push_back(contour[i]);      
        startpoint=contour[i]; //initialize for a new line
        counter=2;
        i++;
        if(i!=contour.size())
        line=Line3D((Position3D)contour[i-1],(Position3D)contour[i]);   
    }
    else if((line.PointOnLine(contour[i],terrestrial_parameters->dist_accurate))&&(angle>20)) //if distance is OK but angle too large
    {
        //cout<<"one minor changed line found!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! by fitting "<<counter<<" points."<<endl;
        contour[i-1].Attribute(SegmentNumberTag)=2;//not confident vertice
        walloutline_laser.push_back(contour[i-1]);      
        startpoint=contour[i]; //initialize for a new line
        counter=1;
        line=Line3D((Position3D)contour[i-1],(Position3D)contour[i]);    
       
    }
    else
    {
        //if the point is accurately on the line, update the line with this point                                                         
          line=Line3D(startpoint,(Position3D)contour[i]);     
          counter++;  
    }   
    
    
   
    } 
    
   
    contour[contour.size()-1].Attribute(SegmentNumberTag)=1;
    walloutline_laser.push_back(contour[contour.size()-1]);
                     
  
     //objpt=ObjectPoint(lpts[contournumber[0]].GetX(),lpts[contournumber[0]].GetY(),ground,0,0,0,0,0,0,0);//ground vertex
     
     
     //remove cracks
     
     outline_obj.clear();
     outline_tops.clear();
    
    counter=0;
    
   
    objpt=ObjectPoint(walloutline_laser[0].GetX(),walloutline_laser[0].GetY(),ground_level,counter++,0,0,0,0,0,0);//ground vertex
     outline_obj.push_back(objpt);
      
   
    
    for(i=0;i<walloutline_laser.size();i++) 
     { 
     
      if(walloutline_laser[i].Attribute(SegmentNumberTag)==2)//if this is unconfi
      {
      for(j=1;(i+j)<walloutline_laser.size();j++)
      {
         if((walloutline_laser[i+j].Attribute(SegmentNumberTag)==2)&&(walloutline_laser[i+j].Distance(walloutline_laser[i+j-1])<terrestrial_parameters->dist_accurate))//if this is unconfident, and no long edge
           continue;
         else break;
      }   
      if(j==1) //single unconfident vertex
      {
      objpt=ObjectPoint(walloutline_laser[i].GetX(),walloutline_laser[i].GetY(),walloutline_laser[i].GetZ(),counter++,0,0,0,0,0,0);
      outline_obj.push_back(objpt);
      }
      else//consequent unconfident vertices
      { 
      
      objpt=ObjectPoint(walloutline_laser[i].GetX(),walloutline_laser[i].GetY(),walloutline_laser[i].GetZ(),counter++,0,0,0,0,0,0);
      outline_obj.push_back(objpt);  
      i=i+j-1; 
      
      }
       }
      else{   

      objpt=ObjectPoint(walloutline_laser[i].GetX(),walloutline_laser[i].GetY(),walloutline_laser[i].GetZ(),counter++,0,0,0,0,0,0);
      outline_obj.push_back(objpt);
      }
      
      
      }
   
   
   
   objpt=ObjectPoint(walloutline_laser[walloutline_laser.size()-1].GetX(),walloutline_laser[walloutline_laser.size()-1].GetY(),ground_level,counter++,0,0,0,0,0,0);//ground vertex
   
   outline_obj.push_back(objpt);

     //remove inline extra vertices
      ObjectPoints outline_obj_tmp;
      
      outline_obj_tmp.push_back(outline_obj[0]);
       for(i=1;i<outline_obj.size()-1;i++) 
     { 
     
     line=Line3D(outline_obj[i-1],outline_obj[i]);
     line_tmp=Line3D(outline_obj[i],outline_obj[i+1]);
     
     angle=Angle2Lines(line,line_tmp)*180/PI;
     
     if(angle>=90)
     angle=180-angle;
     
     if(angle>20)
    outline_obj_tmp.push_back(outline_obj[i]);  
      }
      
   outline_obj_tmp.push_back(outline_obj[outline_obj.size()-1]);
  

   
   
   outline_obj=outline_obj_tmp;
   
    Line3D vertical=Line3D(Position3D(0,0,0),Position3D(0,0,1));
    Line3D horizontal=GBKN3D; 
    double angleHorizontal,angleVertical;
 /*  
    //horizontal and vertical 
      for(i=1;i<outline_obj.size()-2;i++)
    {
     line=Line3D(outline_obj[i],outline_obj[i+1]);
     angleHorizontal=Angle2Lines(line,horizontal )*180/PI;
     angleVertical=Angle2Lines(line,vertical)*180/PI;
     
    
     if(angleHorizontal<10)
     {
     
     if(outline_obj[i].GetZ()<outline_obj[i+1].GetZ())
     outline_obj[i+1].SetZ(outline_obj[i].GetZ());  
     else
     outline_obj[i].SetZ(outline_obj[i+1].GetZ());  
     }
     
     if(angleHorizontal>80)
     {
    
     if(outline_obj[i].GetZ()<outline_obj[i+1].GetZ())
     {outline_obj[i].SetX(outline_obj[i+1].GetX());  outline_obj[i].SetY(outline_obj[i+1].GetY()); }
     else
     {outline_obj[i+1].SetX(outline_obj[i].GetX());  outline_obj[i+1].SetY(outline_obj[i].GetY()); }
     }
  
         
     }
     //remove point in line again
     outline_obj_tmp.clear();
     outline_obj_tmp.push_back(outline_obj[0]);
      for(i=1;i<outline_obj.size()-1;i++) 
     { 
     
     line=Line3D(outline_obj[i-1],outline_obj[i]);
     line_tmp=Line3D(outline_obj[i],outline_obj[i+1]);
     
     angle=Angle2Lines(line,line_tmp)*180/PI;
     
     if(angle>20)
    outline_obj_tmp.push_back(outline_obj[i]);  
      }
     
     outline_obj_tmp.push_back(outline_obj[outline_obj.size()-1]);
     
     outline_obj=outline_obj_tmp;
    
     for(i=0;i<outline_obj.size();i++) 
     { 
        outline_obj[i].Number()=i;                                
     }
   */ 
   
     LineTopology lt;
   
    counter=0;
    
   Line2D line_temp((Position2D)outline_obj[0], (Position2D)outline_obj[outline_obj.size()-1]);
   
   if (line_temp.GetDisto()*GBKN.GetDisto()<0)
   {
   //cout<<"Reverse the GBKN line."<<endl;
   for(i=0;i<outline_obj.size();i++)            
    lt.push_back(PointNumber(outline_obj[i].Number()));
    
    lt.push_back(PointNumber(outline_obj[0].Number()));
   }
   else
   {
   for(i=outline_obj.size()-1;i>=0;i--)            
    lt.push_back(PointNumber(outline_obj[i].Number()));
    
    lt.push_back(PointNumber(outline_obj[outline_obj.size()-1].Number()));
   }
     //maintain counter-clockwise         
    
    
    lt.Smooth_Outline_3D(outline_obj);
    outline_tops.push_back(lt); 

  //outline_tops:   first is ground vertex right, then all the upper contour points, then ground vertex left , then finally ground vertex right
    
  
   //wall_win->Canvas()->ClearObjectData(true);
  
  
   wall_win->map_points=outline_obj;
   
   wall_win->Canvas()->AddObjectData(&outline_obj,&outline_tops,appearance[MapData]->DataAppearancePtr(), false, false);
    wall_win->Canvas()->update(); 
    has_outline=true;
    
    
    reference_obj=outline_obj;
    reference_tops=outline_tops;
    
    for(int i=0;i<reference_obj.size();i++)
    {
            point=ProjectOnBack(outline_obj[i],-0.05);
            reference_obj[i].SetX(point.GetX());
            reference_obj[i].SetY(point.GetY());
            reference_obj[i].SetZ(point.GetZ());
    }
    
   QSound::play("c:/CITY/sound/chimes.wav");
   
   wall_win->Canvas()->localplane=wall;
}




int City::DeriveWallContour()
{
    
    LaserPoints lpts_original;
    int step;
    dc_tolerance=(terrestrial_parameters->smoothness)*0.1;// the bigger, the smoother
    DataBoundsLaser bounds;
    
    ObjectPoint objpt;
    LineTopology top;
    
    
    double max_x,max_y;
    double min_x,min_y;
    
    
    max_x=max_y=-32768999;
    min_x=min_y=32768999;
    
    contour.clear();
   
    Positions2D pts2d,pts2d_sample;
    Position2D pt2d,pt2d_begin,pt2d_end;
    int line_begin, line_end;
    Position3D point;
    Line2D line;
    bool useX;
    
    
    wall_laser=*(wall_win->PointCloud());
    
    
    //find accuate outline
    
    ObjectPoints contour_obj;
          LineTopology contour_top;
          LineTopologies tops;
     
     //DeriveContour3D(wall_laser, contour_obj, contour_top,terrestrial_parameters->dist_accurate);   
     
     wall_laser.DeriveContour3D(contour_obj, contour_top,terrestrial_parameters->dist_accurate);    
   //  contour_top.Smooth_Outline_3D(contour_obj,15);     
    
     
    
     
           tops.clear();
           tops.push_back(contour_top);
         
    UpdateWall();      
    GBKN.Print();
    

    for(int i=0;i<wall_laser.size();i++)
            {
            point=wall.Project(wall_laser[i]);
            wall_laser[i].SetX(point.X());
            wall_laser[i].SetY(point.Y());
            wall_laser[i].SetZ(point.Z());      
            pts2d.push_back((Position2D)wall_laser[i]);        
            }
  
 
    line=Line2D(pts2d);
    
    double angle=abs(line.AngleOx())*180/PI;
    
    if (angle>=90)
    angle=180-angle;

    
    if(angle>45)  //parallel with x, compare x for the two end points
    {
    useX=false;
              for(int i=0;i<pts2d.size();i++) 
              {
               if(pts2d[i].GetY()<min_y)
               {line_begin=i; min_y=pts2d[i].GetY();}
               if(pts2d[i].GetY()>max_y)
               {line_end=i; max_y=pts2d[i].GetY();}        
               } 
               
    }    
     else // not parallel with x, compare y for the two end points
    {
          
          //need modification here!!!!
    useX=true;
               for(int i=0;i<pts2d.size();i++) 
              {                    
               if(pts2d[i].GetX()<min_x)
               {line_begin=i; min_x=pts2d[i].GetX();}
               if(pts2d[i].GetX()>max_x)
               {line_end=i; max_x=pts2d[i].GetX();}        
               } 
               
    }
    
    int contour_begin, contour_end;
    double max_left, max_right;
    max_left=max_right=-99999;
    int left_index, right_index,temp_index;
    
    for(int i=0;i<contour_top.size();i++)//determine left and right vertex of the upper boundary
    {
            objpt=contour_obj.PointByNumber(contour_top[i].Number());
            if(pts2d[line_begin].Distance((Position2D)objpt)<0.2)
            {
            if(max_left<objpt.GetZ())
            {left_index=i; max_left=objpt.GetZ(); }
            }
            
            if(pts2d[line_end].Distance((Position2D)objpt)<0.2)
            {
            if(max_right<objpt.GetZ())
            {right_index=i; max_right=objpt.GetZ(); }
            }       
    }
       cout<<"left and right index is"<<left_index<<" "<<right_index<<endl;
       
       if(left_index<0||left_index>contour_top.size()||right_index<0||right_index>contour_top.size())
       {cout<<"Use scanline contour generation"<<endl; return 0;}
       
    double z1_all, z2_all,mean1, mean2; 
    LaserPoints contour1, contour2;
    
    if(right_index<left_index) 
    {temp_index=right_index;right_index=left_index; left_index=temp_index;}
    
    //determine which part of the contour obj is upper
    z1_all=z2_all=0;
    for(int i=left_index;i<=right_index;i++)
    {
    objpt=contour_obj.PointByNumber(contour_top[i].Number());
    contour1.push_back(LaserPoint(objpt.GetX(),objpt.GetY(),objpt.GetZ())) ;   
    z1_all+=objpt.GetZ();
    }
    mean1=z1_all/contour1.size();
    
    for(int i=left_index;i>=0;i--)
    {
            objpt=contour_obj.PointByNumber(contour_top[i].Number());
            contour2.push_back(LaserPoint(objpt.GetX(),objpt.GetY(),objpt.GetZ())) ;   
            z2_all+=objpt.GetZ();
    }
    
    for(int i=contour_top.size()-1;i>=right_index;i--)
    {
            objpt=contour_obj.PointByNumber(contour_top[i].Number());
            contour2.push_back(LaserPoint(objpt.GetX(),objpt.GetY(),objpt.GetZ())) ;   
            z2_all+=objpt.GetZ();
    }    
    mean2=z2_all/contour2.size();
    
   
    
    if(mean1>mean2) 
      contour=contour1;
    else
      contour=contour2;
    

    int size=contour.size();
    LaserPoint tmp;
    line=Line2D((Position2D)contour[0],(Position2D)contour[size-1]);
    
    if(line.GetDisto()*GBKN.GetDisto()>0)
    {
    cout<<"same direction with GBKN"<<endl;

    }
    else
    {
     cout<<"opposite direction with GBKN. Turn around."<<endl;  //if opposite direction, turn around this line
    for(int i=0;i<size/2;i++)
    {
       tmp=contour[i];
       contour[i]=contour[size-i-1];
       contour[size-i-1]=tmp;
    }
    }
    
   
 return 1;    
}

void City::DeriveWallContour2()
{
    
    LaserPoints lpts_original;
    int step;
    dc_tolerance=(terrestrial_parameters->smoothness)*0.1;// the bigger, the smoother
    DataBoundsLaser bounds;
    
    ObjectPoint objpt;
    LineTopology top;
    
    
    double max_x,max_y;
    double min_x,min_y;
    

    max_x=max_y=-32768999;
    min_x=min_y=32768999;
    
    contour.clear();
   
    Positions2D pts2d,pts2d_sample;
    Position2D pt2d,pt2d_begin,pt2d_end;
    int line_begin, line_end;
    Line2D line;
    bool useX;
    double angle;
    Position3D point;
    
    //wall_laser=*(wall_win->PointCloud());
    
    for(int i=0;i<wall_laser.size();i++)
    {
            point=wall.Project(wall_laser[i]);
            wall_laser[i].SetX(point.X());
            wall_laser[i].SetY(point.Y());
            wall_laser[i].SetZ(point.Z());     
            pts2d.push_back((Position2D)wall_laser[i]);
    }
     
    line=Line2D(pts2d);
    line.Print();
    
    angle=abs(line.AngleOx())*180/PI;
    
    if (angle>=90)
    angle=180-angle;
    
    if(angle>45)  //parallel with x, compare x for the two end points
    {
    useX=false;
              for(int i=0;i<pts2d.size();i++) 
              {
               if(pts2d[i].GetY()<min_y)
               {line_begin=i; min_y=pts2d[i].GetY();}
               if(pts2d[i].GetY()>max_y)
               {line_end=i; max_y=pts2d[i].GetY();}        
               } 
               
    }    
     else // not parallel with x, compare y for the two end points
    {
          
          //need modification here!!!!
    useX=true;
               for(int i=0;i<pts2d.size();i++) 
              {                    
               if(pts2d[i].GetX()<min_x)
               {line_begin=i; min_x=pts2d[i].GetX();}
               if(pts2d[i].GetX()>max_x)
               {line_end=i; max_x=pts2d[i].GetX();}        
               } 
               
    }
    
   
    
    step=(int)pts2d[line_begin].Distance(pts2d[line_end])*20*terrestrial_parameters->step_time;
    cout<<"sample number is: "<<step<<endl;
   
    //sample step points from 2D GIS map, push these points to Pts2d
    //push the first sample point
   
    pt2d_begin=pts2d[line_begin];
    pts2d_sample.push_back(pt2d_begin);
    
    //determine the last sample point
    //objpt=objpts[tops[0][1].Number()-1];
    //pt2d_end=Position2D(objpt.GetX(),objpt.GetY());
    pt2d_end=pts2d[line_end];
    
    //push the other sample points
    double dist_x,dist_y;
    dist_x=pt2d_end.GetX()-pt2d_begin.GetX();
    dist_y=pt2d_end.GetY()-pt2d_begin.GetY();
    for(int i=1;i<=step-2;i++)
    {
    pt2d=Position2D(pt2d_begin.GetX()+i*dist_x/(step-1),pt2d_begin.GetY()+i*dist_y/(step-1));
    pts2d_sample.push_back(pt2d);
    }
    //push the last sample point
    pts2d_sample.push_back(pt2d_end);
   
    //line scan starting   
    double max;
   
    for(int i=0;i<pts2d_sample.size();i++)
    {
    max=-32768999;   
    
    
    for(int j=0;j<wall_laser.size();j++)
    {
      if(pts2d_sample[i].Distance((Position2D)wall_laser[j])<dc_tolerance)
      {
            max=max>wall_laser[j].GetZ()?max:wall_laser[j].GetZ();
          }
    }
    if(max>ground_level+3)
        contour.push_back(LaserPoint(pts2d_sample[i].GetX(),pts2d_sample[i].GetY(),max));
          
    }
    
    //remove the first and last point
   // contour.erase(contour.begin());
   // contour.pop_back();
    LaserPoint tmp;
    int size;
    
    size=contour.size();
    
    line=Line2D((Position2D)contour[0],(Position2D)contour[size-1]);
    
    if(line.GetDisto()*GBKN.GetDisto()>0)
    {
    cout<<"same direction with GBKN"<<endl;

    }
    else
    {
     cout<<"opposite direction with GBKN."<<endl;  //if opposite direction, turn around this line
    for(int i=0;i<size/2;i++)
    {
       tmp=contour[i];
       contour[i]=contour[size-i-1];
       contour[size-i-1]=tmp;
    }
    
     }

}

void City::UpdateWall() //reverse the fitted wall plane, if it is not in the same orientation with 2d mapline
{    
    Vector3D normal;
    Plane ground_plane(Vector3D(0,0,ground_level), Vector3D(0,0,1));      
    wall_laser.Label(1);
    wall=wall_laser.FitPlane(1,1,LabelTag);   
    LaserPoint center;  
    Line2D line_temp;
    //wall_center=building_out[big_segment].Bounds(objpts_out).MidPoint();;  // center is set to the center of the largest segment
                    
    normal=wall.Normal();
    
  //make sure the wall normal direction is to the street side
        if(normal.X()*GBKN.Getcosphi()<0)
          {
          normal.X()=-normal.X();
          normal.Y()=-normal.Y();
          normal.Z()=-normal.Z();
          wall.Distance()=-wall.Distance();
          }
    wall.SetNormal(normal);
  
  
     //update the GBKN again
  Intersect2Planes(wall, ground_plane, GBKN3D);
  line_temp=GBKN3D.ProjectOntoXOYPlane();
 
   wall_win->GBKN_local=GBKN;   
}    


void City::ImproveWallOutline()
{
     Position3D p0, p1,p_before, p_after,p0_new,p1_new;
     LineSegment2D line,line_before, line_after;
     Position2D p0_2d, p1_2d, p_before_2d,p_after_2d, p0_2d_new, p1_2d_new;
     
     int size=outline_obj.size();
     
     ExteriorOrientation temp_ext;
     QString ext_dir(current_dir+"/ext_indirect.txt");
     temp_ext.Read(ext_dir.toAscii());
     //improve one boundary
     p0=outline_obj[0]; p1=outline_obj[1]; p_before=outline_obj[outline_obj.size()-1]; p_after=outline_obj[2];
     
     
     
     if(Improvemodel(p1,p0,line))
     {
     p0_2d=p0.ToPixel(temp_ext,cyclorama->focal);      
     p0_2d.X()=p0_2d.GetX()+(cyclorama->hough_img->width-1)/2;
     p0_2d.Y()=(cyclorama->hough_img->height-1)/2-p0_2d.GetY();
     
     p1_2d=p1.ToPixel(temp_ext,cyclorama->focal);      
     p1_2d.X()=p1_2d.GetX()+(cyclorama->hough_img->width-1)/2;
     p1_2d.Y()=(cyclorama->hough_img->height-1)/2-p1_2d.GetY();
     
     p_before_2d=p_before.ToPixel(temp_ext,cyclorama->focal);      
     p_before_2d.X()=p_before_2d.GetX()+(cyclorama->hough_img->width-1)/2;
     p_before_2d.Y()=(cyclorama->hough_img->height-1)/2-p_before_2d.GetY();
     
     p_after_2d=p_after.ToPixel(temp_ext,cyclorama->focal);      
     p_after_2d.X()=p_after_2d.GetX()+(cyclorama->hough_img->width-1)/2;
     p_after_2d.Y()=(cyclorama->hough_img->height-1)/2-p_after_2d.GetY();

     line_before=LineSegment2D(p_before_2d,p0_2d);
     line_after=LineSegment2D(p1_2d,p_after_2d);
   
     Intersection2Lines(line_before, line, p0_2d_new);
     Intersection2Lines(line_after, line, p1_2d_new);
   
     p0_new=p0_2d_new.To3D(outline_obj,outline_tops[0], temp_ext, cyclorama->focal,cyclorama->hough_img->width,cyclorama->hough_img->height);
     p1_new=p1_2d_new.To3D(outline_obj,outline_tops[0], temp_ext, cyclorama->focal,cyclorama->hough_img->width,cyclorama->hough_img->height);
     
     outline_obj[0].SetX(p0_new.GetX()); outline_obj[0].SetY(p0_new.GetY()); outline_obj[0].SetZ(p0_new.GetZ());
     outline_obj[1].SetX(p1_new.GetX()); outline_obj[1].SetY(p1_new.GetY()); outline_obj[1].SetZ(p1_new.GetZ());
     }
     
     //improve another boundary
     p0=outline_obj[size-2]; p1=outline_obj[size-1]; p_before=outline_obj[size-3]; p_after=outline_obj[0];
     
    
     
      if(Improvemodel(p0,p1,line))
     {
      p0_2d=p0.ToPixel(temp_ext,cyclorama->focal);      
     p0_2d.X()=p0_2d.GetX()+(cyclorama->hough_img->width-1)/2;
     p0_2d.Y()=(cyclorama->hough_img->height-1)/2-p0_2d.GetY();
     
     p1_2d=p1.ToPixel(temp_ext,cyclorama->focal);      
     p1_2d.X()=p1_2d.GetX()+(cyclorama->hough_img->width-1)/2;
     p1_2d.Y()=(cyclorama->hough_img->height-1)/2-p1_2d.GetY();
     
     p_before_2d=p_before.ToPixel(temp_ext,cyclorama->focal);      
     p_before_2d.X()=p_before_2d.GetX()+(cyclorama->hough_img->width-1)/2;
     p_before_2d.Y()=(cyclorama->hough_img->height-1)/2-p_before_2d.GetY();
     
     p_after_2d=p_after.ToPixel(temp_ext,cyclorama->focal);      
     p_after_2d.X()=p_after_2d.GetX()+(cyclorama->hough_img->width-1)/2;
     p_after_2d.Y()=(cyclorama->hough_img->height-1)/2-p_after_2d.GetY();
     
     
     line_before=LineSegment2D(p_before_2d,p0_2d);
     line_after=LineSegment2D(p1_2d,p_after_2d);
     
     Intersection2Lines(line_before, line, p0_2d_new);
     Intersection2Lines(line_after, line, p1_2d_new);
        
     p0_new=p0_2d_new.To3D(outline_obj,outline_tops[0], temp_ext, cyclorama->focal,cyclorama->hough_img->width,cyclorama->hough_img->height);
     p1_new=p1_2d_new.To3D(outline_obj,outline_tops[0], temp_ext, cyclorama->focal,cyclorama->hough_img->width,cyclorama->hough_img->height);
     
     p0.PrintVector();p0_new.PrintVector();
     p1.PrintVector();p1_new.PrintVector();
     
     outline_obj[size-2].SetX(p0_new.GetX()); outline_obj[size-2].SetY(p0_new.GetY()); outline_obj[size-2].SetZ(p0_new.GetZ());
     outline_obj[size-1].SetX(p1_new.GetX()); outline_obj[size-1].SetY(p1_new.GetY()); outline_obj[size-1].SetZ(p1_new.GetZ());
     }
     
     
     
     
     wall_win->update();
     
}


