
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

Position3D City::ProjectOnBack(Position3D point)
{
           Position3D out;
           Line2D line;
           Position2D p2d_tmp1, p2d_tmp2;
           
           line=GBKN.PerpendicularLine (Position2D(point)); 
           line.PointsAtDistance (Position2D(point), -terrestrial_parameters->back_offset, p2d_tmp1, p2d_tmp2) ;
           
           Line2D line_tmp1(Position2D(point), p2d_tmp1);
           Line2D line_tmp2(Position2D(point), p2d_tmp2);
           
           
           if(Angle2D(line_tmp1.Direction(),GBKN.Direction())<0)
           {
           out.SetX(p2d_tmp1.GetX());
           out.SetY(p2d_tmp1.GetY());
           out.SetZ(point.GetZ());
           }
           else
           {
           out.SetX(p2d_tmp2.GetX());
           out.SetY(p2d_tmp2.GetY());
           out.SetZ(point.GetZ());
           }
           return out;
}

Position3D City::ProjectOnBack(Position3D point,double dist)
{
           Position3D out;
           Line2D line;
           Position2D p2d_tmp1, p2d_tmp2;
           
           line=GBKN.PerpendicularLine (Position2D(point)); 
           line.PointsAtDistance (Position2D(point), dist, p2d_tmp1, p2d_tmp2) ;
           Line2D line_tmp1(Position2D(point), p2d_tmp1);
           Line2D line_tmp2(Position2D(point), p2d_tmp2);
           
           
           if(Angle2D(line_tmp1.Direction(),GBKN.Direction())<0)
           {
           out.SetX(p2d_tmp1.GetX());
           out.SetY(p2d_tmp1.GetY());
           out.SetZ(point.GetZ());
           }
           else
           {
           out.SetX(p2d_tmp2.GetX());
           out.SetY(p2d_tmp2.GetY());
           out.SetZ(point.GetZ());
           }
           return out;
}


void City::fillgap()
{
ObjectPoint p0_out_upper,p1_out_upper,p0_back_out_upper, p1_back_out_upper;
ObjectPoint p0_out_lower,p1_out_lower,p0_back_out_lower, p1_back_out_lower;
ObjectPoint p0,p1,p0_back,p1_back,p_tmp;
double gap_vertical=-0.2;

double dx0, dy0, dx1, dy1;
int counter=0;
LineTopology top;
LineTopologies tops;
ObjectPoints objpts;
Position3D point;

Line2D line, line_side, line_out1, line_out2,line_tmp;

Position2D p0_out_point, p1_out_point,p0_2d, p1_2d, p_out,pt1, pt2,p2d_tmp1, p2d_tmp2;

cout<<"start fill gap....."<<endl;


for(int i=0;i<roof_triangle_tops.size();i++){
        
top=roof_triangle_tops[i];
if(top.Label()!=Roof)
 continue;

p0=roof_triangle_obj.PointByNumber(top[0].Number());
p1=roof_triangle_obj.PointByNumber(top[1].Number());

//p0_back=outline_estimated_obj[(rlist[i][0].Number())];
//p1_back=outline_estimated_obj[(rlist[i][1].Number())];
point=ProjectOnBack(p0); 
p0_back=ObjectPoint(point.GetX(),point.GetY(),point.GetZ(), 0,0,0,0,0,0,0);
point=ProjectOnBack(p1); 
p1_back=ObjectPoint(point.GetX(),point.GetY(),point.GetZ(), 0,0,0,0,0,0,0);



line=Line2D((Position2D)p0,(Position2D)p1);


if(line.GetDisto()*GBKN.GetDisto()>0)
{
cout<<"same direction with GBKN. FillGap"<<endl;
}
else
{
cout<<"opposite direction with GBKN"<<endl;

//make sure p0 and p0_back is on left side.
p_tmp=p0;
p0=p1;
p1=p_tmp;

p_tmp=p0_back;
p0_back=p1_back;
p1_back=p_tmp;

}

p0_2d=(Position2D)p0;
line_side=Line2D((Position2D)p0_back,p0_2d);
line_side.PointsAtDistance(p0_2d, 0.15, pt1, pt2);

if((pt1.GetX()-p0.GetX())*(p0_back.GetX()-p0.GetX())>0)  //if pt1 is in between p0 and p0_back
p0_out_point=pt2;  //we want the other one
else 
p0_out_point=pt1;


p1_2d=(Position2D)p1;
line_side=Line2D((Position2D)p1_back,p1_2d);
line_side.PointsAtDistance(p1_2d, 0.15, pt1, pt2);

if((pt1.GetX()-p1.GetX())*(p1_back.GetX()-p1.GetX())>0)  //if pt1 is in between p0 and p0_back
p1_out_point=pt2;  //we want the other one
else 
p1_out_point=pt1;



//original
p0.Number()=counter++;
objpts.push_back(p0);

p1.Number()=counter++;
objpts.push_back(p1);

p1_back.Number()=counter++;
objpts.push_back(p1_back);

p0_back.Number()=counter++;
objpts.push_back(p0_back);

//out lower
p0_out_lower=ObjectPoint(p0_out_point.GetX(),p0_out_point.GetY(), p0.GetZ(), counter++, 0,0,0,0,0,0);
objpts.push_back(p0_out_lower);

p1_out_lower=ObjectPoint(p1_out_point.GetX(),p1_out_point.GetY(),p1.GetZ(), counter++, 0,0,0,0,0,0);
objpts.push_back(p1_out_lower);

p0_back_out_lower=ObjectPoint(p0_back.GetX(),p0_back.GetY(), p0_back.GetZ(), counter++, 0,0,0,0,0,0);
objpts.push_back(p0_back_out_lower);

p1_back_out_lower=ObjectPoint(p1_back.GetX(),p1_back.GetY(),p1_back.GetZ(), counter++, 0,0,0,0,0,0);
objpts.push_back(p1_back_out_lower);

//out upper
p0_out_upper=p0_out_lower;
p0_out_upper.SetZ(p0.GetZ()+gap_vertical);
p0_out_upper.Number()=counter++;
objpts.push_back(p0_out_upper);

p1_out_upper=p1_out_lower;
p1_out_upper.SetZ(p1.GetZ()+gap_vertical);
p1_out_upper.Number()=counter++;
objpts.push_back(p1_out_upper);

p0_back_out_upper=p0_back_out_lower;
p0_back_out_upper.SetZ(p0_back.GetZ()+gap_vertical);
p0_back_out_upper.Number()=counter++;
objpts.push_back(p0_back_out_upper);

p1_back_out_upper=p1_back_out_lower;
p1_back_out_upper.SetZ(p1_back.GetZ()+gap_vertical);
p1_back_out_upper.Number()=counter++;
objpts.push_back(p1_back_out_upper);


//start pushing tolopogies
top=LineTopology();
//front face
top.clear();
top.push_back(p0_out_upper.Number()); top.push_back(p0_out_lower.Number()); top.push_back(p1_out_lower.Number()); top.push_back(p1_out_upper.Number()); 
tops.push_back(top);
//left face
top.clear();
top.push_back(p0_out_upper.Number());top.push_back(p0_back_out_upper.Number());top.push_back(p0_back_out_lower.Number());top.push_back(p0_out_lower.Number());
tops.push_back(top);
//back face
top.clear();
top.push_back(p0_back_out_upper.Number());top.push_back(p1_back_out_upper.Number()); top.push_back(p1_back_out_lower.Number()); top.push_back(p0_back_out_lower.Number());
tops.push_back(top);
//right face
top.clear();
top.push_back(p1_back_out_upper.Number());top.push_back(p1_out_upper.Number());top.push_back(p1_out_lower.Number()); top.push_back(p1_back_out_lower.Number());
tops.push_back(top);

/*
//front bottom
top.clear();
top.push_back(p0.Number());top.push_back(p1.Number());top.push_back(p1_out_lower.Number());top.push_back(p0_out_lower.Number());
tops.push_back(top);

//left bottom
top.clear();
top.push_back(p0.Number());top.push_back(p0_back.Number());top.push_back(p0_back_out_lower.Number());top.push_back(p0_out_lower.Number());
tops.push_back(top);

//right bottom
top.clear();
top.push_back(p1.Number());top.push_back(p1_back.Number());top.push_back(p1_back_out_lower.Number());top.push_back(p1_out_lower.Number());
tops.push_back(top);
*/

//bottom
top.clear();
top.push_back(p0_out_lower.Number());top.push_back(p1_out_lower.Number());top.push_back(p1_back_out_lower.Number()); top.push_back(p0_back_out_lower.Number());   
tops.push_back(top);

//top
top.clear();
top.push_back(p0_out_upper.Number()); top.push_back(p1_out_upper.Number()); top.push_back(p1_back_out_upper.Number());  top.push_back(p0_back_out_upper.Number()); 
tops.push_back(top);
}

for(int i=0;i<tops.size();i++)
tops[i].push_back(tops[i][0]);

gap_obj=objpts;
gap_tops=tops;

}

void City::ExtendRoof()
{
int max=0;
LaserPoints lpts_tmp;
LineTopology top;
ObjectPoint p_left_low,p_right_low, p_left_high, p_right_high, p;

ObjectPoints upper, lower;

double upper_z,lower_z, height;
Line3D line_high, line_low, line;
double dx, dy;

for(int i=0;i<roof_laser.size();i++)
{
if(max<roof_laser[i].Attribute(SegmentNumberTag))
max=roof_laser[i].Attribute(SegmentNumberTag);
}

roof_laser.ConvexHull( roof_objpts, roof_tops,max);


//roof_tops.Write("roof.top");
//roof_objpts.Write("roof.objpts");

roof_win->Canvas()->AddObjectData(&roof_objpts,&roof_tops,appearance[MapData]->DataAppearancePtr(),false, false);
roof_win->map_points=roof_objpts;
roof_win->show();
roof_win->Canvas()->update();

bool useX;

double length;

Position3D point;

double angle=abs(GBKN.AngleOx())*180/PI;
    
  //  if (angle>=90)
    //angle=180-angle;
      
    if(angle>45)  // compare x for the two end points
    useX=false;
    else useX=true;


roof_triangle_obj.clear();
roof_triangle_tops.clear();

for(int i=0;i<roof_tops.size();i++)
{
upper_z=-99999;
lower_z=99999;  
upper.clear();
lower.clear();
      
Intersect2Planes (wall, building_patches[roof_tops[i].Label()].getPlane(),line_low);

for(int j=0;j<roof_tops[i].size();j++)
   {
p=roof_objpts.PointByNumber(roof_tops[i][j].Number());

if(p.GetZ()>upper_z)
 upper_z=p.GetZ();
 
if(p.GetZ()<lower_z)
 lower_z=p.GetZ(); 
   }
   
height=upper_z-lower_z;

cout<<"lower, upper and height"<<  lower_z<<" "<< upper_z<<" "<<height<<endl;

for(int j=0;j<roof_tops[i].size();j++)
   {
p=roof_objpts.PointByNumber(roof_tops[i][j].Number());

if(p.GetZ()>(lower_z+height*0.7))//determine points belong to upper boundary
  upper.push_back(p);
  
if(p.GetZ()<(lower_z+height*0.3))//determine points belong to lower boundary
  lower.push_back(p);  

  }
    //determine most left and right points of lower and upper
    if(useX)
    {
    p_left_high=p_right_high=upper[0];
    for(int j=0;j<upper.size();j++)
            {
               if(upper[j].GetX()<p_left_high.GetX())
               p_left_high=upper[j];
               
               if(upper[j].GetX()>p_right_high.GetX())
               p_right_high=upper[j];
            }

    p_left_low=p_right_low=lower[0];
     for(int j=0;j<lower.size();j++)
            {
               if(lower[j].GetX()<p_left_low.GetX())
               p_left_low=lower[j];
               
               if(lower[j].GetX()>p_right_low.GetX())
               p_right_low=lower[j];
            }
    }   
    else
    {
    p_left_high=p_right_high=upper[0];
    for(int j=0;j<upper.size();j++)
            {
               if(upper[j].GetY()<p_left_high.GetY())
               p_left_high=upper[j];
               
               if(upper[j].GetY()>p_right_high.GetY())
               p_right_high=upper[j];
            }

    p_left_low=p_right_low=lower[0];
     for(int j=0;j<lower.size();j++)
            {
               if(lower[j].GetY()<p_left_low.GetY())
               p_left_low=lower[j];
               
               if(lower[j].GetY()>p_right_low.GetY())
               p_right_low=lower[j];
            }
    }   
    
    line=Line3D(p_left_high, p_left_low);
    IntersectLine3DPlane (line, wall, point);
    double min_d=99999;
    double distance;
    for(int k=0;k<outline_obj.size();k++)//find the nearest wall outline
    {
            //distance=outline_obj[k].Distance(point);
            distance=line.DistanceToPoint(outline_obj[k]);
            if(distance<min_d)
               {p_left_low=outline_obj[k]; min_d=distance;}
    }
    p_left_low.SetZ(p_left_low.GetZ()+0.1);
    
    line=Line3D(p_right_high, p_right_low);
    IntersectLine3DPlane (line, wall, point);
    min_d=99999;
    for(int k=0;k<outline_obj.size();k++)
    {
            //distance=outline_obj[k].Distance(point);
             distance=line.DistanceToPoint(outline_obj[k]);
            if(distance<min_d)
               {p_right_low=outline_obj[k]; min_d=distance;}
    }
    p_right_low.SetZ(p_right_low.GetZ()+0.1);
    
  
    double min_z,max_z;
    
    min_z=p_left_low.GetZ()<p_right_low.GetZ()?p_left_low.GetZ():p_right_low.GetZ();
    p_left_low.SetZ(min_z);
    p_right_low.SetZ(min_z);
    
    
    max_z=p_left_high.GetZ()>p_right_high.GetZ()?p_left_high.GetZ():p_right_high.GetZ();
    p_left_high.SetZ(max_z);
    p_right_high.SetZ(max_z);
    //maintain counter clockwise
    Line2D line;
    line=Line2D((Position2D)p_left_low, (Position2D)p_right_low);
    
    if(line.GetDisto()*GBKN.GetDisto()<0) //if so, swap left and right
    {
    p=p_left_low; p_left_low=p_right_low; p_right_low=p;
    p=p_left_high; p_left_high=p_right_high; p_right_high=p;                                      
    }
    
    roof_triangle_obj.push_back(p_left_low);//0
    roof_triangle_obj.push_back(p_right_low);//1
    roof_triangle_obj.push_back(p_right_high);//2
    roof_triangle_obj.push_back(p_left_high);//3
    
    point=ProjectOnBack(p_left_low);//4
    roof_triangle_obj.push_back(ObjectPoint(point.GetX(),point.GetY(),point.GetZ(),0,0,0,0,0,0,0));
    point=ProjectOnBack(p_right_low);//5
    roof_triangle_obj.push_back(ObjectPoint(point.GetX(),point.GetY(),point.GetZ(),0,0,0,0,0,0,0));
    Plane wall_back(point, wall.Normal());
    point=wall_back.Project(p_right_high);//6
    roof_triangle_obj.push_back(ObjectPoint(point.GetX(),point.GetY(),point.GetZ(),0,0,0,0,0,0,0));
    point=wall_back.Project(p_left_high);;//7
    roof_triangle_obj.push_back(ObjectPoint(point.GetX(),point.GetY(),point.GetZ(),0,0,0,0,0,0,0));
    
}

for(int i=0;i<roof_triangle_obj.size();i++)
roof_triangle_obj[i].Number()=i;

       for(int i=0;i<roof_triangle_obj.size()/8;i++)
       {
        //roof facade
        top=LineTopology(i*5,Roof,i*8,i*8+1); top.push_back(PointNumber(i*8+2));top.push_back(PointNumber(i*8+3));
        top.SetAttribute(TextureTag,1);
        roof_triangle_tops.push_back(top);
        //roof left
        top=LineTopology(i*5+1,EstimatedRoof,i*8,i*8+3); top.push_back(PointNumber(i*8+7));top.push_back(PointNumber(i*8+4));
        roof_triangle_tops.push_back(top);
        //roof right
        top=LineTopology(i*5+2,EstimatedRoof,i*8+1,i*8+5); top.push_back(PointNumber(i*8+6));top.push_back(PointNumber(i*8+2));
        roof_triangle_tops.push_back(top);
        //roof top
        top=LineTopology(i*5+3,EstimatedRoof,i*8+3,i*8+2); top.push_back(PointNumber(i*8+6));top.push_back(PointNumber(i*8+7));
        roof_triangle_tops.push_back(top);
        //roof back
        top=LineTopology(i*5+4,EstimatedRoof,i*8+4,i*8+7); top.push_back(PointNumber(i*8+6));top.push_back(PointNumber(i*8+5));
        roof_triangle_tops.push_back(top);
        }
                
        //roof_triangle_tops.MakeCounterClockWise(roof_triangle_obj);
        for(int i=0;i<roof_triangle_tops.size();i++)
          roof_triangle_tops[i].push_back(roof_triangle_tops[i][0]);        
        

}


//back estimation
void City::fillback()
{
//Position3D pa, pb, pc, pd;
ObjectPoint p0, p1, p0_back, p1_back, p0_bottom,p0_back_bottom;
ObjectPoint p_tmp;
double back_offset;
int size, offset;
LineTopology top;
Line2D line;
Position2D p2d_tmp1, p2d_tmp2;
Position3D point;

outline_full_obj.clear();
outline_full_tops.clear();

cout<<"start fill back....."<<endl;

back_offset=-terrestrial_parameters->back_offset;

if(back_offset==0)  //if this is side wall
  {
  outline_full_obj=outline_obj;
  outline_full_tops=outline_tops;
  return;
  }

outline_estimated_obj.clear();
outline_estimated_tops.clear();
size=outline_obj.size();

offset=1000;
//create estimated back wall
//for(int m=0;m<outline_tops[0].size();m++)
for(int m=0;m<size;m++)
    {
    //p0_back=outline_obj.PointByNumber(outline_tops[0][m].Number());
    p0_back=outline_obj[m];
    point=ProjectOnBack(p0_back);
    p0_back.SetX(point.GetX());
    p0_back.SetY(point.GetY());
    outline_estimated_obj.push_back(p0_back);
    }
    top.clear();
    
    outline_estimated_tops=outline_tops;
    
int temp;    

size=outline_tops[0].size();

for(int m=0;m<size/2;m++)
    {
    temp=outline_estimated_tops[0][m].Number();
    outline_estimated_tops[0][m].Number()= outline_estimated_tops[0][size-m-1].Number() ;
    outline_estimated_tops[0][size-m-1].Number()=temp;    
    }    
    
//fill inbetween front and back
    
//first push the front object points 

for(int m=0;m<size;m++)
    {
        p0=outline_obj.PointByNumber(outline_estimated_tops[0][m].Number());
        p0.Number()=p0.Number()+offset;
        outline_estimated_obj.push_back(p0);
    }    
        
        
    for(int m=0;m<outline_tops[0].size()-1;m++)
    {
            p0_back=outline_estimated_obj.PointByNumber(outline_tops[0][m].Number()); 
            p1_back=outline_estimated_obj.PointByNumber(outline_tops[0][m+1].Number()); 
            p0=outline_estimated_obj.PointByNumber(outline_tops[0][m].Number()+offset); 
            p1=outline_estimated_obj.PointByNumber(outline_tops[0][m+1].Number()+offset); 
            top.clear();
            top.push_back(p0.Number());top.push_back(p0_back.Number()); top.push_back(p1_back.Number()); top.push_back(p1.Number());top.push_back(p0.Number());
            top.Label()=Wall;
            outline_estimated_tops.push_back(top);
    }
}
/*
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
  double range=1;
      
     double x, y, z;
  
    localplane=window_tmp.FitPlane(0,0,SegmentNumberTag);
    
    for(int i=0;i<window_tmp.size();i++)
    {
            window_tmp[i]=localplane.Project(window_tmp[i]);
    }
    //output inverse wall
    
    TINEdges *alltinedges, *mytinedges;
    mytin=window_tmp.DeriveTIN();
    //(*mytin).Write("mytin.tin");
    
    alltinedges=window_tmp.TINEdges3D(0.3);
    mytinedges=window_tmp.NeighbourhoodEdges3D(*alltinedges,range);
    
   for (int i=0; i<window_tmp.size();i++) {
   
   hole_node=false;
     if(window_tmp[i].Attribute(SegmentNumberTag)!=0) continue;
    for (k=0; k<mytinedges[i].size(); k++) { //all the edges that connect to point window_tmp[i]
      m=(*mytinedges)[i][k].Number();
    
     // cout<<m<<" "<<lpts[m].GetPointNumber()<<endl;
       dist = window_tmp[i].Distance(window_tmp[m]);
        if (dist > (terrestrial_parameters->window_width)) //if this is a long edge, then a hole/border is found
        {hole_node=true; break;}
        }
    
     if(hole_node)  
     setLabel(i,count++);    
     }   
 
  int t;

    wall_all.clear();
    PointNumberList ptnumberlist;
    LaserPoints boundary;
    //hypothesis: all the inner triangles has 3 neighbours
    for(t=0;t<(*mytin).size();t++)
    {       
    ptnumberlist=window_tmp.Neighbourhood3D(PointNumber(t),range, *mytinedges);
    
    for(int n=0;n<ptnumberlist.size();n++)
    {
    if((ptnumberlist[n].Number()==-1))  //if any point is on the edge, set all these neighbourhood -1
       { 
       for(int p=0;p<ptnumberlist.size();p++)
       window_tmp[ptnumberlist[p].Number()].Label(-1);
       break;
       }
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
                
                      if(tmp[min_y_index].Distance(tmp[max_y_index])<(terrestrial_parameters->window_width))
                       continue;
                      
                      p1=Position3D(tmp[min_y_index].GetX(),tmp[min_y_index].GetY(),max_z);
                      p2=Position3D(tmp[max_y_index].GetX(),tmp[max_y_index].GetY(),max_z);
                      double a,b;
                      a=abs(max_x-min_x);
                      b=abs(p1.Distance(p2));
                      
                      if((a/b>5)||(b/a>5))
                        {cout<<"Not normal shape, abandoned."; continue;}
                      
                     
                      //cout<<"range of this hole: "<<tmp[min_y_index].Distance(tmp[max_y_index])<<endl;
                           
                      hole=hole+tmp;    //update laser file
         }
      else                       //this is part of border
         border=border+tmp;      //update laser file
    }

    //border=boundary;
   
}

*/
