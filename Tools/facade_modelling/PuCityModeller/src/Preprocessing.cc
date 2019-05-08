
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
#include <QMessageBox>
#include <QFileDialog>
#include <QString>
#include <direct.h>

void City::Select_reconstruct_map()
{
     /// Data appearance
    map_appearance=new DataAppearanceWindow(*appearance[LaserData],this);
    map_appearance->SetPointSize(9);
    double min_x=32768999;
    double max_x=-32768999;
    LaserPoints lpts;
    ObjectPoint p1, p2,p;
    ObjectPoint p_left, p_right;
    LineTopsIterVector::iterator line1, line2,line;
    PointNumberList::iterator vertex;
    int pn_left, pn_right;
    PointNumberList plist;
    ObjectPoints objpts;
    LineTopologies tops;
    Position2D p2d_left, p2d_right;
    Line2D line_perpen_left, line_perpen_right;
    Position2D p2d_l1, p2d_l2, p2d_r1, p2d_r2;
    ObjectPoint obj_l1, obj_l2, obj_r1, obj_r2;
    LineTopology top;
    LaserUnit lu;
    LaserPoints temp;
    LaserBlock lb;
    
    //SetSelectMapMode();
    
    
     if(!selected_map_data.size())
     { QMessageBox::information(this,  "Error.","There is no map line selected!");
     return;
     }
     else{
          
         for(line=selected_map_data.begin();line!=selected_map_data.end();line++)
         {
         for(vertex=(*line)->begin();vertex!=(*line)->end();vertex++)
           plist.push_back((*vertex));
         } 
          
         for(int i=0;i<plist.size();i++)
         {
         p=map_points.PointByNumber(plist[i].Number());
         if(p.GetX()<min_x)
          {min_x=p.GetX(); p_left=p; }
         if(p.GetX()>max_x)
          {max_x=p.GetX(); p_right=p; }
         } 
          
          map_laser.clear();
          map_laser.push_back(LaserPoint(p_left.GetX(),p_left.GetY(),p_left.GetZ()));
          map_laser.SetAttribute(SegmentNumberTag, 2);
          
          
         Canvas()->AddLaserData (&map_laser, map_appearance->DataAppearancePtr(), false, true);
          
         
          int r=QMessageBox::warning(this,tr("Adjust mapline"),
          tr("Is the red point on the left side of the whole mapline?\n"
          "Looking from street to house."),
          QMessageBox::Yes|QMessageBox::Default, 
          QMessageBox::No);
          
          if(r==QMessageBox::No)
          {
          Canvas()->RemoveLaserData(&map_laser,true);
          map_laser.clear();
          map_laser.push_back(LaserPoint(p_right.GetX(),p_right.GetY(),p_right.GetZ()));
          Canvas()->AddLaserData (&map_laser, map_appearance->DataAppearancePtr(), false, true);
          GBKN=Line2D(Position2D(p_right.GetX(),p_right.GetY()),Position2D(p_left.GetX(),p_left.GetY()));
          tops.push_back(LineTopology(0,0,p_right,p_left));    
          }
          else
          {
          GBKN=Line2D(Position2D(p_left.GetX(),p_left.GetY()),Position2D(p_right.GetX(),p_right.GetY()));
          tops.push_back(LineTopology(0,0,p_left,p_right));      
          }
          objpts.push_back(p_left);objpts.push_back(p_right);  
          
          //create a new directory
          map_top=tops;
          map_obj=objpts;      
          }
              
       p2d_left=Position2D(p_left.GetX(),p_left.GetY());   
       p2d_right=Position2D(p_right.GetX(),p_right.GetY());
       line_perpen_left=GBKN.PerpendicularLine(p2d_left);
       line_perpen_right=GBKN.PerpendicularLine(p2d_right);
       line_perpen_left.PointsAtDistance (p2d_left, 4, p2d_l1, p2d_l2);
       line_perpen_right.PointsAtDistance (p2d_right, 4, p2d_r1, p2d_r2);
       obj_l1=ObjectPoint(p2d_l1.GetX(),p2d_l1.GetY(),0, 0,  0,0,0,0,0,0);
       obj_l2=ObjectPoint(p2d_l2.GetX(),p2d_l2.GetY(),0, 1,  0,0,0,0,0,0);
       obj_r1=ObjectPoint(p2d_r1.GetX(),p2d_r1.GetY(),0, 2,  0,0,0,0,0,0);
       obj_r2=ObjectPoint(p2d_r2.GetX(),p2d_r2.GetY(),0, 3,  0,0,0,0,0,0);
       objpts.clear();tops.clear();
       objpts.push_back(obj_l1);objpts.push_back(obj_l2);objpts.push_back(obj_r1);objpts.push_back(obj_r2);
       top.push_back(PointNumber(0));top.push_back(PointNumber(1));
       top.push_back(PointNumber(3));top.push_back(PointNumber(2));top.push_back(PointNumber(0));
       tops.push_back(top);
       
       
       
  selection_tops=tops;
  selection_obj=objpts;               
 
 Canvas()->AddObjectData(&selection_obj,&selection_tops,appearance[MapData]->DataAppearancePtr(), false, true);
 

 if(!lb.ReadMetaData(tile_meta_file))

{
      QMessageBox::information(this,  "Error.","No metadata file specified, or cannot open that file. ");
      return;                              
}

lu=LaserUnit();

laser_points.clear();

lu=lb[0];

for(int i=0;i<lu.size();i++)
{
    lu[i].Select(temp, selection_obj.ObjectPointsRef(), selection_tops.LineTopologiesReference ());
    if(temp.size())
    
    laser_points=laser_points+temp; 
    temp.clear();
}    
 
 cout<<"Selection Complete! "<<laser_points.size()<< " points selected."<<endl; 
 
    if (laser_points.size())
    //ShowData(CityWindowPtr(), LaserData, true, SelectedLaserData, false, true);
    Canvas()->AddLaserData (&laser_points, appearance[LaserData]->DataAppearancePtr(), false, true);
    
   // laser_points.Write("SelectionLaser.laser",0);
   return;
 
 }

void City::Create_map_from_laser()
{
     
 LaserUnit lu;
 LaserPoints temp;
 LaserBlock lb;
     
 lu=LaserUnit();

if(!lb.ReadMetaData(tile_meta_file))

{
      QMessageBox::information(this,  "Error.","No metadata file specified, or cannot open that file. ");
      return;                              
}

laser_points.clear();
cout<<"lb size: "<<lb.size()<<endl;

lu=lb[0];

cout<<"lu size: "<<lu.size()<<endl;


for(int i=0;i<lu.size();i++)
{
    lu[i].Read();
    cout<<"laser subunit size is: "<<lu[i].size()<<endl;
    for(int j=0;j<lu[i].size();j++)
    {
      if((lu[i][j].GetZ()>=-0.2)&&lu[i][j].GetZ()<0.2)
         laser_points.push_back(lu[i][j]);
    }
}    
 
 cout<<"Selection Complete! "<<laser_points.size()<< " points selected."<<endl; 
 
    if (laser_points.size())
    Canvas()->AddLaserData (&laser_points, appearance[LaserData]->DataAppearancePtr(), false, true);
    
   return;

     
}













void City::Select_reconstruct_region()
{
LaserPoint lpt;
int counter1, counter2;
counter1=counter2=0;
ObjectPoints *tile_obj;
LineTopologies *tile_tops;
LaserUnit lu;
LaserPoints temp;
LaserBlock lb;


if (!(Canvas()->HasValidSelectionRectangle())){   QMessageBox::information(this,  "Error.","There is no selection box!"); 
                                                         return;}

Canvas()->selection_box->UpdateWorldCorners (Canvas());

tile_obj=Canvas()->selection_box->BoxPoints();
tile_tops=Canvas()->selection_box->BoxTopologies();

if(!lb.ReadMetaData(tile_meta_file))

{
      QMessageBox::information(this,  "Error.","No metadata file specified, or cannot open that file. ");
      return;                              
}

lu=LaserUnit();


laser_points.ErasePoints();
cout<<"lb size: "<<lb.size()<<endl;

lu=lb[0];

cout<<"lu size: "<<lu.size()<<endl;

for(int i=0;i<lu.size();i++)
{
    lu[i].Select(temp, tile_obj->ObjectPointsRef(), tile_tops->LineTopologiesReference ());
    if(temp.size())
    {
    laser_points=laser_points+temp; 
    temp.clear();
    }
}    
 
 cout<<"Selection Complete! "<<laser_points.size()<< " points selected."<<endl; 
 
    if (laser_points.size())
    {laser_points.DeriveDataBounds(0);
    ShowData(CityWindowPtr(), LaserData, true, SelectedLaserData, false, true);
     }
    
    //laser_points.Write("SelectionLaser.laser",0);

}

