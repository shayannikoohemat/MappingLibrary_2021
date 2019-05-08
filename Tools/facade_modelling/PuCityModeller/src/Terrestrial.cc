
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
#include "LaserUnit.h"
#include "LaserSubUnit.h"
#include "LaserBlock.h"
#include <QSound>
#include <QMessageBox>
#include <QFile>


void City::FindFeature()
{
    
    if(max_segment_value<1)
    {
           QMessageBox::information(this, "Error",
                               "Laser points are not segmented yet.\n");
                             
        return;
    }                     
    
    texture_exist=QFile::exists(current_dir+"/texture.png");
    exter_exist=QFile::exists(current_dir+"/ext_indirect.txt");
    
    
    progress=new QProgressDialog("Finding features...",0,0,100,this,0);
    progress->setAutoClose(true);
    progress->show();
    CCH();
    
 
     
    //return; 
        
    progress->setValue(20);
    progress->update();
    
    
    LaserPatch mypiece;
    Plane myplane;
    
    
    LaserPatch street, roof;
  
    
    LineTopologies tops;

    LaserPoints lpts_out,lpts_tmp;
    
    //LaserPatches ps(objpts, building_out);
    LineTopologies out;

    
    double area;
    
    int max_index,i;
    Vector3D normal;
    Position3D center,wall_center;
    Positions3D centers;
    double x, y, z;
    x=y=z=0;
    LaserPoint center_laser;
    
    
    double areas[max_segment_value];
    int counter=0;
    
    cout<<max_segment_value<<" segments in total."<<endl;
    cout<<building_patches.size()<<" building patches in total."<<endl;
    
    cout<<"Calculating segment areas..."<<endl;
    //store the area values for each segment
    for(i=0;i<max_segment_value-2;i++)
    {
    cout<<"Segment "<<counter<<" processed.\r";
    if(!building_patches[i].size())
      area=0;
    else
      {
             area=abs(building_patches[i].area3D());
             counter++;
      }
      
    areas[i]=area;
    }
    
        
     progress->setValue(30);
      progress->setLabelText("Recognizing street...");
     progress->update();
  
  
    //recognize street 
      LaserPoints tmp;
      ground_laser.clear();
      Position3D left3d, right3d;
     
     double average_density;
      counter=0;
     for(i=0;i<building_patches.size();i++)
         {
                                           
         if(areas[0]==0)
         continue;
         
          normal=building_patches[i].getPlane().Normal();
          center=building_out[i].Bounds(objpts_out).MidPoint();
          
          //center=
          if(building_patches[i].getPlane().IsHorizontal(0.4)&&areas[i]>(terrestrial_parameters->street_min_area)&&center.GetZ()<(terrestrial_parameters->street_center_height)) 
              {
                      counter++;
                      street_tops.push_back(building_out[i]);
              //out[out.size()-1].Number()=tops[i].Number();
              
              tmp=laser_points.SelectTagValue(SegmentNumberTag,i);
              ground_laser=ground_laser+tmp; 
                      
              ground_level=center.GetZ();
              ground_index=i;
              }  
          }
      center=building_out[ground_index].Bounds(objpts_out).MidPoint(); cout<<"   ground index is"<<ground_index<<endl;
          
      //ground_win = new CityWindow(CityView,Ground, this, Qt::Window);
      cout<<counter<<" segments are recognized as street segments."<<endl;
      counter=0;
                                   
      if(ground_laser.size())  {
      ground_win->GBKN_local=GBKN;                   
      ground_win->AddLaserData(ground_laser.LaserPointsReference(),ground_win->appearance[LaserData]->DataAppearancePtr(), true);    
     ground_win->Canvas()->InitialiseTransformation(); 
     ground_win->Canvas()->update();           
     }
     ground_win->show();
     ground_win->showMinimized () ;
     
     //street_laser.Write("street.laser", 0); 
     //street_tops.Write("street.top");      
       //lpts_out.clear();
    //out.clear();
    cout<<"Ground level is: "<<ground_level<<endl;
    //Recognize wall
  progress->setValue(40);
  progress->setLabelText("Recognizing wall...");
  progress->update();
  
  wall_laser.clear(); 
  double max_area=0; 
  
  //Deal with no GBKN or updating GBKN

  left3d=Position3D(left_map.GetX(), left_map.GetY(), ground_level);
  right3d=Position3D(right_map.GetX(), right_map.GetY(),ground_level);
  
  for(i=0;i<building_patches.size();i++)
  {
  normal=building_patches[i].getPlane().Normal();
  center=building_out[i].Bounds(objpts_out).MidPoint();
  
  if(GBKN_OK)  //update the GBKN line
     {
        if(areas[i]>max_area&&building_patches[i].getPlane().IsVertical(0.3)&&AlignWall((Vector2D)normal,GBKN,10)&&GBKN.DistanceToPoint((Position2D)center)<0.5) 
          {
             max_index=i; max_area=areas[i];                                                                                                                                     
          }
     }
  else 
     {
       if(areas[i]>max_area&&building_patches[i].getPlane().IsVertical(0.3))
          {
             max_index=i; max_area=areas[i];                                                                                                                                     
          }
     }
         
  }       
  
  Plane ground_plane(Vector3D(0,0,ground_level), Vector3D(0,0,1));      
  //Intersect2Planes(building_patches[max_index].getPlane(), ground_plane, GBKN3D);
  //GBKN=GBKN3D.ProjectOntoXOYPlane();
  center=building_out[ground_index].Bounds(objpts_out).MidPoint();
  double distance;
 // if (GBKN.DistanceToPointSigned ((Position2D)center)<0)
  // {
  // cout<<"Reverse the GBKN line."<<endl;
  // GBKN.SwapNormal();
  // }
   
          //deal with low density data
          tmp=laser_points.SelectTagValue(SegmentNumberTag,max_index);
          average_density=tmp.size()/areas[max_index];
          
          cout<<"average point density is "<<average_density<<" points per square meter."<<endl;
                    
          if(average_density<200)  terrestrial_parameters->dist_accurate=2*terrestrial_parameters->dist_accurate;        
  
  
  
  //Recognize wall 
  double density;
  wall_laser.clear();
      for(i=0;i<building_patches.size();i++)
         {
          if(areas[0]==0)
         continue;
                                            
          normal=building_patches[i].getPlane().Normal();
          center=building_out[i].Bounds(objpts_out).MidPoint();
        // if(abs(ps[i].area3D())>40&&(abs(normal.X())>0.1)&&(abs(normal.Y())>0.1)&&(abs(normal.Z())<0.2)) 
       if((areas[i]>(terrestrial_parameters->wall_min_area))&&(building_patches[i].getPlane().IsVertical(0.3))&&AlignWall((Vector2D)normal,GBKN,10)&&GBKN.DistanceToPoint((Position2D)center)<(terrestrial_parameters->wall_map_max_d))                
                 {
                  
                    wall_tops.push_back(building_out[i]);           //this is a wall, push it in
                     
                     tmp=laser_points.SelectTagValue(SegmentNumberTag,i);
                 density=tmp.size()/areas[i];
                 
                 if(density<average_density*0.3)
                  {cout<<"abandon because density is"<<density<<endl;continue;}
                 
                  counter++;
                      
                      wall_laser=wall_laser+tmp;                     
              
              }
    
         }     
          //wall_win = new CityWindow(CityView, Wall, this, Qt::Window);
          cout<<counter<<" segments are recognized as wall segments."<<endl;
          counter=0;
          
          wall_win->show();
          
          if(wall_laser.size()){
      
          wall_win->GBKN_local=GBKN;   
                
          wall_win->Canvas()->ClearLaserData(false);
      
          wall_win->AddLaserData(wall_laser.LaserPointsReference(),wall_win->appearance[LaserData]->DataAppearancePtr(), true); 
      
          wall_win->Canvas()->InitialiseTransformation();
          wall_win->Canvas()->update();   
      
         
          wall_win->showMinimized () ;

          }
          else
          {
              QMessageBox::information(this, "Error",
                               "No wall is detected.\n");
              roof_win->show();roof_win->showMinimized () ;
              roof_extru_win->show();roof_extru_win->showMinimized () ;
              door_win->show(); door_win-> showMinimized () ;               
              extrusion_win->show(); extrusion_win->showMinimized () ;          
        return;
              } 
    UpdateWall();
   FitWallLine();    
    
    progress->setValue(60);
    progress->setLabelText("Recognizing roof...");
    progress->update();
    
    
    
    
  
   double wall_contour_min=99999;
   double wall_contour_max=-99999;
   for(i=0;i<contour.size();i++)
  {
        wall_contour_min= wall_contour_min<contour[i].GetZ()? wall_contour_min :  contour[i].GetZ();      
        wall_contour_max= wall_contour_max>contour[i].GetZ()? wall_contour_max :  contour[i].GetZ();                 
  }
   
   if(wall_contour_min==99999)
   wall_contour_min=wall_center.GetZ()+2;
   
   if(wall_contour_max==99999)
   wall_contour_max=wall_center.GetZ()+2;
    
   cout<<"wall contour min and max is "<<wall_contour_min<<" "<<wall_contour_max<<endl ;
    
    //Recognize Roof    
    double angle;
    roof_laser.clear();
    roof_tops.clear();
    Vector3D v=Vector3D(0,0,1);
    double z_min=32768999, z_max=-32768999;
         for(i=0;i<building_patches.size();i++)
         {
          normal=building_patches[i].getPlane().Normal();
          center=building_out[i].Bounds(objpts_out).MidPoint();
          angle=abs(Angle(normal,wall.Normal()))*180/PI;
          if(angle>90)
          angle=180-angle;
          
        if(angle>20&&areas[i]>(terrestrial_parameters->roof_min_area)&&center.GetZ()>wall_contour_min&&AlignWall((Vector2D)normal,GBKN,15)&&GBKN.DistanceToPointSigned((Position2D)center)<0)
              {
              //remove the long extrusion between roof and wall
              tmp=laser_points.SelectTagValue(SegmentNumberTag,i);
              z_min=32768999, z_max=-32768999;
              for(int k=0;k<tmp.size();k++)
              {
                      if(tmp[k].GetZ()<z_min)
                        z_min=tmp[k].GetZ();
                        
                      if(tmp[k].GetZ()>z_max)
                        z_max=tmp[k].GetZ();  
                      
              }
              cout<<"angle with wall is"<<Angle(normal,wall.Normal())<<endl;
              if((z_max-z_min)<0.5)//if this is a very small roof
              continue;
           
              counter++;
    
               roof_laser=roof_laser+laser_points.SelectTagValue(SegmentNumberTag,i);
               roof_tops.push_back(building_out[i]);
              
              }
              }
              cout<<counter<<" segments are recognized as roof segments."<<endl;
     
      counter=0;
      
     roof_win->GBKN_local=GBKN;   
     roof_win->Canvas()->ClearLaserData(true);         
     
    if(roof_laser.size())
    {
    roof_win->AddLaserData(roof_laser.LaserPointsReference(),roof_win->appearance[LaserData]->DataAppearancePtr(), true);
    roof_win->Canvas()->InitialiseTransformation();             
    }
   
    roof_win->show();
    roof_win->showMinimized () ;
    
    progress->setValue(70);
    progress->setLabelText("Recognizing roof extrusions...");
    progress->update();
    
    //Recognize roof extrusion    
    roof_extru_laser.clear();
    double roof_min, x_min, x_max, roof_extru_min;
    Position3D center_project;
    ObjectPoint objpt;
    int roof_index;
       double roof_highest=-99999;

    if(roof_laser.size()){
                          
     roof_min=roof_laser[0].GetZ();                     
    //determine the min roof height
    for(i=0;i<roof_laser.size();i++)
    {
      if(roof_laser[i].GetZ()<roof_min)
        roof_min=roof_laser[i].GetZ();
    }
    
   // roof_extru_tops.clear();
         for(i=0;i<building_patches.size();i++)
         {
          normal=building_patches[i].getPlane().Normal();
          center=building_out[i].Bounds(objpts_out).MidPoint();
         
        if((building_patches[i].getPlane().IsVertical(0.2))&&areas[i]>(terrestrial_parameters->roof_extru_min_area)&&center.GetZ()>roof_min&&AlignWall((Vector2D)normal,GBKN,30))
              { 
        
         x_min=roof_extru_min=32768999; x_max=-32768999;   roof_index=0;   
       
        //loop through all roofs, to find its master roof
        for(int k=0;k<roof_tops.size();k++)
        {
              for(int j=0;j<roof_tops[k].size();j++)
              {
               objpt=objpts_out.PointByNumber(roof_tops[k][j].Number());   
               x_min=x_min<objpt.GetX()?x_min:objpt.GetX();
               x_max=x_max>objpt.GetX()?x_max:objpt.GetX();
               z_min=z_min<objpt.GetZ()?z_min:objpt.GetZ();
               }
     
               if((center.GetX()-x_min)*(center.GetX()-x_max)<0)
               {
               roof_index=roof_tops[k].Label();
              // cout<<roof_index<<" roof is roof extrusion ("<<i <<")'s master roof."<<endl;
               break;
               }
       }
       if(roof_index==0)
       {continue;}    
        
        
         for(int j=0;j<building_out[roof_index].size();j++)
        {    
          objpt=objpts_out.PointByNumber(building_out[roof_index][j].Number());
          if(objpt.GetZ()>roof_highest)
            roof_highest=objpt.GetZ();
        } 
       
        if(center.GetZ()>roof_highest)
        continue;
        
        center_project=building_patches[roof_index].getPlane().Project(center);
        if(center_project.GetZ()>center.GetZ())
        continue;
       
     tmp=laser_points.SelectTagValue(SegmentNumberTag,i);
      z_min=99999;
      z_max=-9999;
      for(int k=0;k<tmp.size();k++)
              {
                      if(tmp[k].GetZ()<z_min)
                        z_min=tmp[k].GetZ();
                        
                      if(tmp[k].GetZ()>z_max)
                        z_max=tmp[k].GetZ();  
                      
              }
      if((z_max-z_min)<0.5)//if this is a very small roof
              continue;
                     
       counter++;
       roof_extru_laser=roof_extru_laser+tmp;        
              }
              
        }
      cout<<counter<<" segments are recognized as roof extrusion segments."<<endl;
      counter=0;
    
    roof_extru_win->GBKN_local=GBKN;   
      roof_extru_win->Canvas()->ClearLaserData(true); 
      
     if(roof_extru_laser.size())
     {
    roof_extru_win->AddLaserData(roof_extru_laser.LaserPointsReference(),roof_extru_win->appearance[LaserData]->DataAppearancePtr(), true);    
    roof_extru_win->Canvas()->InitialiseTransformation();            
    }  
}   
     roof_extru_win->show();
    roof_extru_win->showMinimized () ;
 
    
    progress->setValue(80);
    progress->setLabelText("Recognizing doors...");
    progress->update();
    
      //Recognize Door
      bool onwall;   
      door_laser.clear();
      door_tops.clear();
      
         for(i=0;i<building_patches.size();i++)
         {
       
          normal=building_patches[i].getPlane().Normal();
          center=building_out[i].Bounds(objpts_out).MidPoint();
          
          if(abs(GBKN.DistanceToPoint((Position2D)center))<terrestrial_parameters->door_wall_max_distance)
          onwall=true;
          else onwall=false;
          
    if(onwall&&(building_patches[i].OnGround(ground_level,0.3))&&(center.GetZ()>(0.1+ground_level))&&(center.GetZ()<(ground_level+terrestrial_parameters->door_max_height)/2)&&areas[i]>(terrestrial_parameters->door_min_area)&&areas[i]<(terrestrial_parameters->door_max_area)&&building_patches[i].getPlane().IsVertical(0.3)) 
              {
                  counter++;
                   door_laser=door_laser+laser_points.SelectTagValue(SegmentNumberTag,i);
                   
                   door_tops.push_back(building_out[i]);
              
              }
              }
   
   cout<<counter<<" segments are recognized as door segments."<<endl;
      counter=0;
      
     door_win->GBKN_local=GBKN;                           
     door_win->Canvas()->ClearLaserData(true);       
     
      if(door_laser.size())
      {                            
      door_win->AddLaserData(door_laser.LaserPointsReference(),door_win->appearance[LaserData]->DataAppearancePtr(), true);
      door_win->Canvas()->InitialiseTransformation();             
      }
          
  door_win->Canvas()->update();   
  door_win->show();
  door_win->showMinimized () ;
  
  progress->setValue(90);
  progress->setLabelText("Recognizing protrusions...");
  progress->update();
   //Recognize extrusion

   extrusion_laser.clear();
   extrusion_tops.clear();
   bool extrude;
   for(i=0;i<building_patches.size();i++)
         {
          normal=building_patches[i].getPlane().Normal();
          center=building_out[i].Bounds(objpts_out).MidPoint();
          angle=90-abs(Angle2D(GBKN.Direction(),(Vector2D)normal));
          
         distance= wall.Distance(center);
         
         if(distance>0.1&&distance<1)
         extrude=true;
         else
         extrude=false;
         
         double extru_highest=-99999;
    
          //if(areas[i]>2&&!AlignWall((Vector2D)normal,lines[0],0.2)&&center.Distance(wall.getPlane())<0&&center.GetZ()>(ground_level+1)) 
          
    if(areas[i]>2&&extrude&&center.GetZ()>(ground_level+1)&&building_patches[i].getPlane().IsVertical(0.2)) 
              {
                  //cout<<i<<" area: "<<areas[i]<<" distance to ground: "<<building_patches[i].Distance(street)<<" distance to wall:  " <<center.Distance(wall)<<endl;
              
               for(int j=0;j<building_out[i].size();j++)
              {
               objpt=objpts_out.PointByNumber(building_out[i][j].Number());   
               extru_highest=extru_highest>objpt.GetZ()?extru_highest:objpt.GetZ();
               }    
              
              if(extru_highest>wall_contour_max)
              continue;    
                  
                   extrusion_laser=extrusion_laser+laser_points.SelectTagValue(SegmentNumberTag,i);
                   extrusion_tops.push_back(building_out[i]);
                   
                   counter++;
                   
              //out[out.size()-1].Number()=tops[i].Number();       
              }
              }
              
         cout<<counter<<" segments are recognized as extrusion segments."<<endl;
      counter=0;
      cwindow_objpts.clear();
      cwindow_tops.clear();
      
       extrusion_win->GBKN_local=GBKN; 
       extrusion_win->Canvas()->ClearLaserData(true);
       extrusion_win->Canvas()->ClearObjectData(true);
       
      if(extrusion_laser.size())
      {                          
    extrusion_win->AddLaserData(extrusion_laser.LaserPointsReference(),extrusion_win->appearance[LaserData]->DataAppearancePtr(), true);
    extrusion_win->Canvas()->InitialiseTransformation();            
    }     
  
    extrusion_win->Canvas()->update();    
    extrusion_win->show();   
    extrusion_win->showMinimized () ;    
        
  progress->setValue(100);
  progress->setLabelText("Feature finding completed.");
  progress->hide();
  QSound::play("c:/CITY/sound/chimes.wav");
  return;
   
}




bool City::AlignX(Vector2D vec1,double thres)
{
     double angle;
     Vector2D x=Vector2D(1,0);
     angle=Angle2D(vec1,x);
     
     if(abs(angle)<thres||abs(angle-PI)<thres)
       { 
         return true;
         }
     else return false;    
}





bool City::AlignWall(Vector2D vec1, Line2D line, double err)
{
     double angle=abs(Angle2D(line.Direction(),vec1));

     if(abs(PI/2-angle)<(err*PI/180))
          return true;
     else 
          return false;                    
}




void City::setLabel(int index,int label)
{
     int number;
     double distance;
     
     if(window_tmp[index].Attribute(SegmentNumberTag)!=0)
       return ;
     else 
      window_tmp[index].Attribute(SegmentNumberTag)=label;
    
     for(int a=0;a<mytinedges[index].size();a++)
     {
          number=mytinedges[index][a].Number();
          
          distance=window_tmp[index].Distance(window_tmp[number]);
          if(distance>(terrestrial_parameters->window_width))
             {setLabel(number,label); }
     }     
}
int City::FindWindow()

{
    LaserPoints tmp;
    
    window_laser.clear();
    wall_laser=*(wall_win->PointCloud());
    FindHole(wall_laser, window_laser, tmp);
    
    CreateTopology(Window);
    window_win->Canvas()->ClearLaserData(true);
    window_win->Canvas()->ClearObjectData(true);
    window_win->AddLaserData(window_laser.LaserPointsReference(),window_win->appearance[LaserData]->DataAppearancePtr(), true);
    window_win->map_points=window_obj;
     window_win->GBKN_local=GBKN;   
    window_win->Canvas()->AddObjectData(&window_obj,&window_tops,appearance[MapData]->DataAppearancePtr(),false, false);   
    //save obj and top
    //window_obj.Write("window.objpts");
    //window_tops.Write("window.top",1);   
    window_win->Canvas()->InitialiseTransformation();   
    window_win->Canvas()->SetBackGroundColour(Qt::white);
     window_win->show();
    
    reference_tops.SetAttribute(TextureTag,1);
    reference_tops.SetAttribute(LineLabelTag,Wall);
    
    if(texture_exist&&exter_exist)
    {
    window_win->Canvas()->texture_objpts=&reference_obj,
    window_win->Canvas()->texture_tops=&reference_tops;
    window_win->Canvas()->MakeTextureData(current_dir);
    }
   
    window_win->Canvas()->update();
    
             
   // window_win->Canvas()->update();  
     //experiment texture
    

   
   
    //window_laser.Write("window.laser",1);
}

int City::FindHole(LaserPoints input, LaserPoints& hole, LaserPoints& border)
{ 

 bool hole_node;
 int count=1;

 double dist;
 int k,m;
 LineTopology top;
 LineTopologies tops;
 ObjectPoints objpts;
 LaserPoints wall_all;
 Plane localplane;
 bool useX;

    
    TIN *mytin; 
  //window_tmp=wall_laser;
  window_tmp=input;
  
  window_tmp.SetAttribute(SegmentNumberTag,0);
  
  
  cout<<"window_width is: "<<terrestrial_parameters->window_width<<endl;
  cout<<"The input laserpoints contains "<<window_tmp.size()<<" points"<<endl;
  border.clear();
  hole.clear();
      
     double x, y, z;
  
  if(abs(GBKN.AngleOx())>PI/4)
  useX=false;
  else useX=true;
  
    if(useX)
    window_tmp.SwapYZ();
    else
    window_tmp.SwapXZ();
    
    localplane=window_tmp.FitPlane(0,0,SegmentNumberTag);
    
    for(int i=0;i<window_tmp.size();i++)
    {
            window_tmp[i]=localplane.Project(window_tmp[i]);
    }
    //output inverse wall
    
    
    mytin=window_tmp.DeriveTIN();
    //(*mytin).Write("mytin.tin");
    
    mytinedges=TINEdges(window_tmp.TINReference());
    
   for (int i=0; i<window_tmp.size();i++) {
   
   hole_node=false;
     if(window_tmp[i].Attribute(SegmentNumberTag)!=0) continue;
    for (k=0; k<mytinedges[i].size(); k++) { //all the edges that connect to point window_tmp[i]
      m=mytinedges[i][k].Number();
    
     // cout<<m<<" "<<lpts[m].GetPointNumber()<<endl;
       dist = input[i].Distance(input[m]);
        if (dist > (terrestrial_parameters->window_width)) //if this is a long edge, then a hole/border is found
        {hole_node=true; break;}
        }
    
     if(hole_node)  
    { setLabel(i,count++);    }
     }   
 
  int t;

    wall_all.clear();
    MeshNumber *mesh;
    PointNumber *ptnumber;
    LaserPoints boundary;
    //hypothesis: all the inner triangles has 3 neighbours
    for(t=0;t<(*mytin).size();t++)
    {
    mesh=(*mytin)[t].Neighbours();    
    ptnumber= (*mytin)[t].Nodes();               
    if((mesh->Number()==-1)||((mesh+1)->Number()==-1)||((mesh+2)->Number()==-1))  //if any point is on the edge
       { 
       window_tmp[ptnumber->Number()].Label(-1);
       window_tmp[(ptnumber+1)->Number()].Label(-1);
       window_tmp[(ptnumber+2)->Number()].Label(-1);    
       }
   }
    
    for(t=0;t<window_tmp.size();t++)
    {
      if (window_tmp[t].Label()==-1)
         boundary.push_back(window_tmp[t]);
    }
    
    
     
    for(t=0;t<window_tmp.size();t++)
    {
    if(window_tmp[t].Attribute(SegmentNumberTag)!=0)
       wall_all.push_back(window_tmp[t]);
    }   
    
    
    LaserPoints tmp;
    double max_x,min_x,max_z,min_z,max_y,min_y;
    int max_x_index,min_x_index,max_z_index,min_z_index,max_y_index,min_y_index;
    Position3D p1,p2;
    bool small_window;
    window_max_no=count;

    for(int i=1;i<count;i++)
    {
      tmp=wall_all.SelectTagValue(SegmentNumberTag,i);
      if(tmp.size()<10)
         continue;
      //cout<<"cluster: "<<i<<"  size is: "<<tmp.size();
      for(t=0;t<tmp.size();t++)   //if any point is on the edge then this should go to border cluster
       {
       if(tmp[t].Label()==-1)
           break;
       }
      //cout<<"  t = "<<t<<endl; 
      max_x=-3276800;min_x=3276800;max_z=-3276800;min_z=3276800;max_y=-3276800;min_y=3276800;
      if(t==tmp.size())       //this is a hole(window)
         {
                      for(int j=0;j<tmp.size();j++)
                      {
                      
                      if(tmp[j].GetX()<min_x)
                      {min_x=tmp[j].GetX();min_x_index=j;}
                      
                      if(tmp[j].GetZ()<min_z)
                      {min_z=tmp[j].GetZ();min_z_index=j;}
                      
                      if(tmp[j].GetX()>max_x)
                      {max_x=tmp[j].GetX();max_x_index=j;}
                      
                      if(tmp[j].GetZ()>max_z)
                      {max_z=tmp[j].GetZ();max_z_index=j;}    
                      
                      if(tmp[j].GetY()<min_y)
                      {min_y=tmp[j].GetY();min_y_index=j;}
                      
                      if(tmp[j].GetY()>max_y)
                      {max_y=tmp[j].GetY();max_y_index=j;}            
                  
                      }  
                      
                     
                  
                      if(useX)
                      small_window=tmp[min_x_index].Distance(tmp[max_x_index])<(terrestrial_parameters->window_width);
                      else
                      small_window=tmp[min_y_index].Distance(tmp[max_y_index])<(terrestrial_parameters->window_width);
                      
                      if(small_window)
                       continue;
                  
                      //cout<<"range of this hole: "<<tmp[min_y_index].Distance(tmp[max_y_index])<<endl;
                           
                      hole=hole+tmp;    //update laser file
         }
      else                       //this is part of border
         border=border+tmp;      //update laser file
    }
    
    
    if(useX)
    hole.SwapYZ();
    else
    hole.SwapXZ();
    

  
}


void City::CreateTopology(FeatureWindowType ftype)
{
   LaserPoints mylaserpoints,testpoints;
   bool useX;
   
   switch(ftype){
          case Window:       mylaserpoints=window_laser;  break;
          case Door:         mylaserpoints=*(door_win->PointCloud());break;
                 }
                 
   mylaserpoints.Label(0);
   double max_x,min_x,max_z,min_z,max_y,min_y;
    int max_x_index,min_x_index,max_z_index,min_z_index,max_y_index,min_y_index,s;
    int left, right;
     ObjectPoint objpt1,objpt2,objpt3,objpt4;
     LineTopologies tops;
     ObjectPoints objpts;
       int count_obj=0;
       s=0;
   for(int i=0;i<mylaserpoints.size();i++)
      {  if(mylaserpoints[i].Attribute(SegmentNumberTag)>10000)
                continue;
           s=(s>mylaserpoints[i].Attribute(SegmentNumberTag))?s:mylaserpoints[i].Attribute(SegmentNumberTag);}   
    cout<<"maximum segments number is:"<<s<<endl;
   
   
   
   
   for(int i=0;i<=s;i++)
   {
   testpoints=mylaserpoints.SelectTagValue(SegmentNumberTag,i);
   if(testpoints.size()!=0)   // if there is laser points with this segment no 
     {
     //cout<<"a window segment"<<endl;
     max_x=-3276800;min_x=3276800;max_z=-3276800;min_z=3276800;max_y=-3276800;min_y=3276800;
     for(int j=0;j<testpoints.size();j++)
                      {
                      
                      if(testpoints[j].GetX()<min_x)
                      {min_x=testpoints[j].GetX();min_x_index=j;}
                      
                      if(testpoints[j].GetZ()<min_z)
                      {min_z=testpoints[j].GetZ();min_z_index=j;}
                      
                      if(testpoints[j].GetX()>max_x)
                      {max_x=testpoints[j].GetX();max_x_index=j;}
                      
                      if(testpoints[j].GetZ()>max_z)
                      {max_z=testpoints[j].GetZ();max_z_index=j;}    
                      
                      if(testpoints[j].GetY()<min_y)
                      {min_y=testpoints[j].GetY();min_y_index=j;}
                      
                      if(testpoints[j].GetY()>max_y)
                      {max_y=testpoints[j].GetY();max_y_index=j;}            
                  
                      }  
     if(abs(GBKN.AngleOx())>PI/4)
     useX=false;
     else
     useX=true;
     //useX=AlignX((Vector2D)(wall.Normal()),0.1);           
     
     if(useX)
     {left=min_x_index;right=max_x_index;}
     else
     {left=min_y_index;right=max_y_index;}
     
     Line2D line;
     int tmp;
     
     line=Line2D((Position2D)testpoints[left], (Position2D)testpoints[right]);
     
     
if(line.GetDisto()*GBKN.GetDisto()<0)
{         
tmp=left;
left=right;
right=tmp;            
}                      
      
     testpoints[left]=(LaserPoint)wall.Project(testpoints[left]);        
     testpoints[right]=(LaserPoint)wall.Project(testpoints[right]);   
    
     
     if(ftype==Door)
         min_z=ground_level;

       objpt1=ObjectPoint(testpoints[left].GetX(),testpoints[left].GetY(),min_z,count_obj++,0,0,0,0,0,0); //left bottom
     objpt2=ObjectPoint(testpoints[left].GetX(),testpoints[left].GetY(),max_z,count_obj++,0,0,0,0,0,0);   //left up
     objpt3=ObjectPoint(testpoints[right].GetX(),testpoints[right].GetY(),max_z,count_obj++,0,0,0,0,0,0);  //right up
     objpt4=ObjectPoint(testpoints[right].GetX(),testpoints[right].GetY(),min_z,count_obj++,0,0,0,0,0,0);  //right bottom
            
                      objpts.push_back(objpt1);
                      objpts.push_back(objpt2);  
                      objpts.push_back(objpt3);
                      objpts.push_back(objpt4);
     }
   
   }   
   
    int size=objpts.size();
    LineTopology top;
    //objpts.clear();
    
    for(int i=0;i<size/4;i++)
    {
          
    top=LineTopology(i,i);
    top.push_back(PointNumber(i*4));
    top.push_back(PointNumber(i*4+3));
    top.push_back(PointNumber(i*4+2));
    top.push_back(PointNumber(i*4+1));  
    top.push_back(PointNumber(i*4));
    tops.push_back(top);
    }
    switch(ftype){
          case Window:       window_obj=objpts; window_tops=tops; cout<<window_tops.size()<<" windows fitted."<<endl;break;
          case Door:         door_obj=objpts; door_tops=tops; cout<<door_tops.size()<<" door fitted."<<endl;break;
                 }
     
}

void City::SmoothRoof()
{
LineTopology::iterator iter;
LineTopologies::iterator iters;
Line3D line_a, line_b;
double angle_thres=0.35;
double angle;

for(iters=roof_tops.begin();iters!=roof_tops.end();iters++) //start analyzing each roof
    {
     for(iter=iters->begin();iter!=iters->end()-3;)
     {
       //fitline for this roof
       line_a=Line3D(roof_objpts.PointByNumber(iter->Number()),roof_objpts.PointByNumber((iter+1)->Number()));
       line_b=Line3D(roof_objpts.PointByNumber((iter+1)->Number()),roof_objpts.PointByNumber((iter+2)->Number()));
       angle=Angle2Lines(line_a,line_b);
       if(angle<angle_thres)
           iters->erase(iter+1);
       else
         iter++;
     }   
     iters->pop_back();
    }
    
}    




void City::CreateFinalModel()

{
 Buildings::iterator                      building;
 ObjectPoints                             *points;
  std::vector <LineTopologies *>           *polygon_sets;    
  std::vector <LineTopologies *>::iterator polygon_set;
  

if(!has_outline) //WallLine hasn't been generated
 FitWallLine();
 cout<<"create final model"<<endl;   
 

 ExtendRoof();
   
 fillback();
 
 fillgap();
 

 QPoint qp; 
 Position3D world_pos;
   
 world_pos=Position3D(83112, 435985, 5);
 //create estimated back wall
 if(!outline_full_win)
 {
     outline_full_win=new CityWindow(CityView, this, Qt::Window);
     subwindows.push_back(outline_full_win);   
     outline_full_win->Canvas()->ClearObjectData(false);
 }
     
 
 //outline_full_win->Canvas()->AddObjectData(&outline_full_obj,&outline_full_tops,appearance[MapData]->DataAppearancePtr(), false, false);

 
  
 qp=outline_full_win->Canvas()->World2Canvas(world_pos,true);

 //Door outline
  
   
   
    CreateTopology(Door);
 
 door_win->Canvas()->ClearObjectData(true);
 door_win->map_points=door_obj;
 door_win->GBKN_local=GBKN;
 door_win->Canvas()->AddObjectData(&door_obj,&door_tops,appearance[MapData]->DataAppearancePtr(),false, false); 
  
  OffsetWindow();
 //push everything
 /*
 outline_full_win->Canvas()->ClearObjectData(true);Canvas()->ClearObjectData(true);
 cout<<"Add main outline....";
 //outline_full_win->Canvas()->AddObjectData(&outline_full_obj,&outline_full_tops,appearance[MapData]->DataAppearancePtr(), false, false); 
 //if(window_obj.size())
     //outline_full_win->Canvas()->AddHoleObjectData(&outline_drilled_obj,&outline_drilled_tops,appearance[ModelData]->DataAppearancePtr(), false, true); 
 //else
 Canvas()->AddObjectData(&outline_obj,&outline_tops,appearance[MapData]->DataAppearancePtr(), false, false); 
 outline_full_win->Canvas()->AddObjectData(&outline_obj,&outline_tops,appearance[MapData]->DataAppearancePtr(), false, false);
 
 cout<<"estimated outline....";
 Canvas()->AddObjectData(&outline_estimated_obj,&outline_estimated_tops,appearance[MapData]->DataAppearancePtr(), false, false); 
 outline_full_win->Canvas()->AddObjectData(&outline_estimated_obj,&outline_estimated_tops,appearance[MapData]->DataAppearancePtr(), false, false); 
 
  //cout<<"door....";
 //if(door_tops.size())
 //{
 //Canvas()->AddObjectData(&door_obj,&door_tops,appearance[MapData]->DataAppearancePtr(), false, false); 
 //outline_full_win->Canvas()->AddObjectData(&door_obj,&door_tops,appearance[MapData]->DataAppearancePtr(), false, false); 
 //}
 
 cout<<"estimated gap between wall and roof....";
 if(gap_tops.size())
 {
 Canvas()->AddObjectData(&gap_obj,&gap_tops,appearance[MapData]->DataAppearancePtr(), false, false); 
 outline_full_win->Canvas()->AddObjectData(&gap_obj,&gap_tops,appearance[MapData]->DataAppearancePtr(), false, false); 
 }
 
 cout<<"roof....";
 if(roof_triangle_tops.size())
 {
 Canvas()->AddObjectData(&roof_triangle_obj,&roof_triangle_tops,appearance[MapData]->DataAppearancePtr(), false, false); 
 outline_full_win->Canvas()->AddObjectData(&roof_triangle_obj,&roof_triangle_tops,appearance[MapData]->DataAppearancePtr(), false, false); 
 }
 
 cout<<"window and door holes....";
 if(hole_out_tops.size())
 {
 Canvas()->AddObjectData(&hole_out_obj,&hole_out_tops,appearance[MapData]->DataAppearancePtr(), false, false);  
 outline_full_win->Canvas()->AddObjectData(&hole_out_obj,&hole_out_tops,appearance[MapData]->DataAppearancePtr(), false, false);
 }

 cout<<"roof extrusion....";
 if(roof_extru_tops.size())
 {
 Canvas()->AddObjectData(&roof_extru_obj,&roof_extru_tops,appearance[MapData]->DataAppearancePtr(), false, false);  
 outline_full_win->Canvas()->AddObjectData(&roof_extru_obj,&roof_extru_tops,appearance[MapData]->DataAppearancePtr(), false, false);  
 }

  
 // outline_full_win->Canvas()->AddObjectData(&roof_objpts,&roof_tops,appearance[ModelData]->DataAppearancePtr(), false, false); 
   
   //outline_win->Canvas()->AddLaserData(&walloutline_laser,appearance[LaserData]->DataAppearancePtr(), true);
cout<<"extrusion....";
if(cwindow_tops.size())
 {
 Canvas()->AddObjectData(&cwindow_objpts,&cwindow_tops,appearance[MapData]->DataAppearancePtr(), false, false); 
 outline_full_win->Canvas()->AddObjectData(&cwindow_objpts,&cwindow_tops,appearance[MapData]->DataAppearancePtr(), false, false); 
 }
   
*/ 
    
 
   


    //SaveDXF();
    
     //ObjectPoints objpts; LineTopologies tops;
    int offset=0;
    final_objpts.clear();
    final_tops.clear();
    //push wall outline
    LineTopology top;
    ObjectPoint objpt;
 
    outline_drilled_tops.SetAttribute(LineLabelTag, Wall);
    outline_drilled_tops.SetAttribute(TextureTag, 1);
    outline_estimated_tops.SetAttribute(LineLabelTag, Wall);
    //door_tops.SetAttribute(LineLabelTag, Door);
    hole_out_tops.SetAttribute(LineLabelTag,Window);
    outline_estimated_tops.SetAttribute(LineLabelTag,Wall);
    
    roof_triangle_tops.SetAttribute(LineLabelTag,Roof);
    roof_extru_tops.SetAttribute(LineLabelTag,RoofExtrusion);
    gap_tops.SetAttribute(LineLabelTag,Gap);
    
    int temp;
    for(int i=0;i<gap_tops.size();i++)
      for(int j=0;j<gap_tops[i].size()/2;j++)
      {
      temp=gap_tops[i][j].Number();
      gap_tops[i][j].Number()=gap_tops[i][gap_tops[i].size()-j-1].Number();
      gap_tops[i][gap_tops[i].size()-j-1].Number()=temp;
      }
    
    //push front
     for(int i=0;i<outline_drilled_obj.size();i++)
      {
            outline_drilled_obj[i].Number()+=offset;
            final_objpts.push_back(outline_drilled_obj[i]);
      }
    for(int i=0;i<outline_drilled_tops.size();i++)
      {
            for(int j=0;j<outline_drilled_tops[i].size();j++)
                    outline_drilled_tops[i][j].Number()+=offset;
            final_tops.push_back(outline_drilled_tops[i]);
      }
     offset+=3000;
   
  
   //push side and back wall
       
     for(int i=0;i<outline_estimated_obj.size();i++)
      {
            outline_estimated_obj[i].Number()+=offset;
            final_objpts.push_back(outline_estimated_obj[i]);
      }
    for(int i=0;i<outline_estimated_tops.size();i++)
      {
            for(int j=0;j<outline_estimated_tops[i].size();j++)
                    outline_estimated_tops[i][j].Number()+=offset;
            final_tops.push_back(outline_estimated_tops[i]);
      }
      offset+=3000; 
      
    /*
    //push door outline
    for(int i=0;i<door_obj.size();i++)
      {
            door_obj[i].Number()+=offset;
            objpts_out.push_back(door_obj[i]);
      }
    for(int i=0;i<door_tops.size();i++)
      {
            for(int j=0;j<door_tops[i].size();j++)
                    door_tops[i][j].Number()+=offset;
            building_out.push_back(door_tops[i]);
      }
    
     offset+=3000;
     */
      //push gap
 
    for(int i=0;i<gap_obj.size();i++)
      {
            gap_obj[i].Number()+=offset;
            final_objpts.push_back(gap_obj[i]);
      }
    for(int i=0;i<gap_tops.size();i++)
      {
            for(int j=0;j<gap_tops[i].size();j++)
                    gap_tops[i][j].Number()+=offset;
            final_tops.push_back(gap_tops[i]);
      }
  
     offset+=3000;
    
    //push window outline
    for(int i=0;i<hole_out_obj.size();i++)
      {
            hole_out_obj[i].Number()+=offset;
            final_objpts.push_back(hole_out_obj[i]);
      }
    for(int i=0;i<hole_out_tops.size();i++)
      {
            for(int j=0;j<hole_out_tops[i].size();j++)
                    hole_out_tops[i][j].Number()+=offset;
            final_tops.push_back(hole_out_tops[i]);
      }
     offset+=3000;
  
 
    //push roof outline
    for(int i=0;i<roof_triangle_obj.size();i++)
      {
            roof_triangle_obj[i].Number()+=offset;
            final_objpts.push_back(roof_triangle_obj[i]);
      }
    for(int i=0;i<roof_triangle_tops.size();i++)
      {
            for(int j=0;j<roof_triangle_tops[i].size();j++)
                    roof_triangle_tops[i][j].Number()+=offset;
            final_tops.push_back(roof_triangle_tops[i]);
      }
   // offset=objpts_out.size();
     offset+=3000;
  
    //push roof extrusion
    top.clear();
    for(int i=0;i<roof_extru_obj.size();i++)
      {           
            objpt=roof_extru_obj[i];
            objpt.Number()+=offset;
            final_objpts.push_back(objpt);
      }
    for(int i=0;i<roof_extru_tops.size();i++)
      {
            top=roof_extru_tops[i];
            for(int j=0;j<top.size();j++)
                    top[j].Number()+=offset;
            final_tops.push_back(top);
      }
    offset+=3000;
  
    //push  extrusion
    top.clear();
    for(int i=0;i<cwindow_objpts.size();i++)
      {
            objpt=cwindow_objpts[i];
            objpt.Number()+=offset;
            final_objpts.push_back(objpt);
      }
    for(int i=0;i<cwindow_tops.size();i++)
      {
            top=cwindow_tops[i];
            for(int j=0;j<top.size();j++)
                    top[j].Number()+=offset;
            final_tops.push_back(top);
      }
 
   for(int i=0;i<final_tops.size();i++)
   {
           final_tops[i].Number()=i;
   }
  
    outline_full_win->Canvas()->AddObjectData(&final_objpts,&final_tops,appearance[MapData]->DataAppearancePtr(), false, false);
    Canvas()->AddObjectData(&final_objpts,&final_tops,appearance[MapData]->DataAppearancePtr(), false, false);
    outline_full_win->map_points=final_objpts;
    map_points=final_objpts;
    outline_full_win->Canvas()->InitialiseTransformation();            
    outline_full_win->show();
   
   if(final_objpts.size()&&texture_exist&&exter_exist)
    {
    outline_full_win->Canvas()->texture_objpts=&final_objpts,
    outline_full_win->Canvas()->texture_tops=&final_tops;
    
    outline_full_win->Canvas()->MakeTextureData(current_dir);
    outline_full_win->Canvas()->update();
    }
  
   // objpts_out.Write("all.objpts");
  //  building_out.Write("all.top");
    
    
   //for anisha
   /*
   outline_obj.Write("wall.objpts");
   outline_tops.Write("wall.top");
   window_obj.Write("window.objpts");
   window_tops.Write("window.top");
    */
    
    
    cout<<"Final model visualization complete!!!"<<endl;
    QSound::play("c:/CITY/sound/tada.wav");   
   
   
   //improve model with image
   Image_Contour();
     
}




void City::OffsetWindow()
{
     Line2D line;
     ObjectPoint p0,p0_back,objpt;
     ObjectPoints hole_objpts;
     LineTopologies hole_tops;
     LineTopology top;
     LineTopologies tops;
     double back_offset;
     Position2D p2d_tmp1, p2d_tmp2;
     int size;
     int num_count,max_no_obj,size_tops;

     
     back_offset=-0.15;//to be changed to parameter
     
   
     //merge door and window
     hole_objpts=window_obj;
     hole_tops=window_tops;
     
     max_no_obj=window_obj.size();
     
     for(int i=0;i<door_obj.size();i++)
     {
       objpt=door_obj[i];
       
       objpt.Number()=objpt.Number()+max_no_obj;
       hole_objpts.push_back(objpt);
       
     }
     size_tops=window_tops.size();
     for(int i=0;i<door_tops.size();i++)
     {
       top=door_tops[i];
       top.Number()=top.Number()+size_tops;
       
       for(int j=0;j<top.size();j++)
        top[j].Number()=top[j].Number()+max_no_obj;
       
       hole_tops.push_back(top);
     }
     
    
     size=hole_objpts.size();
     //first , drill holes on wall outline.
     
     outline_drilled_obj=outline_obj;
     outline_drilled_tops=outline_tops;
     
     if(!size) return;
     
     outline_drilled_tops.SetAttribute(HoleMasterTag,15);
     
     max_no_obj=outline_drilled_obj.MaxPointNumber()+1;
     
     Plane outline_plane;
     Position3D point;
     //project all the points to outline plane;
    
     for(int i=0;i<outline_tops[0].size();i++)
             outline_plane.AddPoint(outline_obj.PointByNumber(outline_tops[0][i].Number()),false);
     
     outline_plane.Recalculate();
     
     for(int i=0;i<hole_objpts.size();i++)
     {
        point=outline_plane.Project(hole_objpts[i]);     
        
        hole_objpts[i].SetX(point.GetX());
        hole_objpts[i].SetY(point.GetY());
        hole_objpts[i].SetZ(point.GetZ());
             
     }
     
     for(int i=0;i<hole_objpts.size();i++)
     {
       objpt=hole_objpts[i];
       
       objpt.Number()=objpt.Number()+max_no_obj;
       outline_drilled_obj.push_back(objpt);
     }
     cout<<"set holes"<<endl;
     size_tops=outline_tops.size();
     for(int i=0;i<hole_tops.size();i++)
     {
       top=hole_tops[i];
       top.Number()=top.Number()+size_tops;
       top.SetAttribute(HoleTag, 15);
       
       for(int j=0;j<top.size();j++)
        top[j].Number()=top[j].Number()+max_no_obj;
       
       outline_drilled_tops.push_back(top);
     }
    
 // window_win->Canvas()->ClearObjectData(true);
  
  hole_out_obj=hole_objpts; hole_out_tops.clear();
  
     //create all the offset objectpoints
     for(int i=0;i<size;i++)
     {
     p0=hole_objpts[i];
     point=ProjectOnBack(p0, back_offset);
     p0_back=p0;
     p0_back.SetX(point.GetX());
     p0_back.SetY(point.GetY());
     p0_back.Number()=p0.Number()+size;
     hole_out_obj.push_back(p0_back);
     }
     
     //create and push back the offset paralled window patch
     num_count=size/4;
 
     
     //back side
     for(int i=size/4;i<size/2;i++)
     {
     top=LineTopology(num_count++,i-size/4);
     top.push_back(PointNumber(i*4));
     top.push_back(PointNumber(i*4+3));
     top.push_back(PointNumber(i*4+2));
     top.push_back(PointNumber(i*4+1));  
     top.push_back(PointNumber(i*4));
     top.SetAttribute(TextureTag,1);
     hole_out_tops.push_back(top);       
     }
     
     
     for(int i=0;i<size/4;i++)
     {
     //left        
     top=LineTopology(num_count++,i);
     top.push_back(PointNumber(i*4));
     top.push_back(PointNumber(i*4+size)); 
     top.push_back(PointNumber(i*4+size+1));
     top.push_back(PointNumber(i*4+1));  
     top.push_back(PointNumber(i*4));
     hole_out_tops.push_back(top);   
     //right
     top=LineTopology(num_count++,i);
     top.push_back(PointNumber(i*4+2));
     top.push_back(PointNumber(i*4+size+2));
     top.push_back(PointNumber(i*4+size+3)); 
     top.push_back(PointNumber(i*4+3));
     top.push_back(PointNumber(i*4+2));
     hole_out_tops.push_back(top);  
     
     //top
     top=LineTopology(num_count++,i);
     top.push_back(PointNumber(i*4+1));
     top.push_back(PointNumber(i*4+size+1));
     top.push_back(PointNumber(i*4+size+2));
     top.push_back(PointNumber(i*4+2));  
     top.push_back(PointNumber(i*4+1));
     hole_out_tops.push_back(top); 
       
     //bottom
     top=LineTopology(num_count++,i);
     top.push_back(PointNumber(i*4));
     top.push_back(PointNumber(i*4+3));  
     top.push_back(PointNumber(i*4+size+3));
     top.push_back(PointNumber(i*4+size));
     top.push_back(PointNumber(i*4));
     hole_out_tops.push_back(top); 
     }
     
    // hole_out_obj.Write("offset_hole.objpts");
    // hole_out_tops.Write("offset_hole.top");
     
}



bool City::Rectangular(ObjectPoints& objpts, LineTopologies& tops, LaserPoints testpoints, int count_obj)
{
LineTopology top;
 ObjectPoint objpt1,objpt2,objpt3,objpt4;
bool useX;    
 double max_x,min_x,max_z,min_z,max_y,min_y;
    int max_x_index,min_x_index,max_z_index,min_z_index,max_y_index,min_y_index;
    int left, right;
objpts.clear();
tops.clear();

 if(testpoints.size()==0) return false;
     //cout<<"a window segment"<<endl;
     max_x=-3276800;min_x=3276800;max_z=-3276800;min_z=3276800;max_y=-3276800;min_y=3276800;
     for(int j=0;j<testpoints.size();j++)
                      {
                      
                      if(testpoints[j].GetX()<min_x)
                      {min_x=testpoints[j].GetX();min_x_index=j;}
                      
                      if(testpoints[j].GetZ()<min_z)
                      {min_z=testpoints[j].GetZ();min_z_index=j;}
                      
                      if(testpoints[j].GetX()>max_x)
                      {max_x=testpoints[j].GetX();max_x_index=j;}
                      
                      if(testpoints[j].GetZ()>max_z)
                      {max_z=testpoints[j].GetZ();max_z_index=j;}    
                      
                      if(testpoints[j].GetY()<min_y)
                      {min_y=testpoints[j].GetY();min_y_index=j;}
                      
                      if(testpoints[j].GetY()>max_y)
                      {max_y=testpoints[j].GetY();max_y_index=j;}            
                  
                      }  
     useX=AlignX((Vector2D)(wall.Normal()),0.1);           
     
     if(!useX)
     {left=min_x_index;right=max_x_index;}
     else
     {left=min_y_index;right=max_y_index;}
     
     Line2D line;
     int tmp;
     
     line=Line2D((Position2D)testpoints[left], (Position2D)testpoints[right]);
          
     if(line.GetDisto()*GBKN.GetDisto()<0)
     {         
     tmp=left;
     left=right;
     right=tmp;            
     }                      
        
     objpt1=ObjectPoint(testpoints[left].GetX(),testpoints[left].GetY(),min_z,count_obj++,0,0,0,0,0,0);
     objpt2=ObjectPoint(testpoints[left].GetX(),testpoints[left].GetY(),max_z,count_obj++,0,0,0,0,0,0);
     objpt3=ObjectPoint(testpoints[right].GetX(),testpoints[right].GetY(),max_z,count_obj++,0,0,0,0,0,0);
     objpt4=ObjectPoint(testpoints[right].GetX(),testpoints[right].GetY(),min_z,count_obj++,0,0,0,0,0,0);
     
     objpts.push_back(objpt1);
     objpts.push_back(objpt2);
     objpts.push_back(objpt3);
     objpts.push_back(objpt4);
     
     top.push_back(PointNumber(objpt1.Number()));
     top.push_back(PointNumber(objpt2.Number()));
     top.push_back(PointNumber(objpt3.Number()));
     top.push_back(PointNumber(objpt4.Number()));
     top.push_back(PointNumber(objpt1.Number()));
     tops.push_back(top);
  

}

