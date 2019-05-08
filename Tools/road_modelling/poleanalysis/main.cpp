
/*
                     Copyright 2010 University of Twente
 
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


#include <cstdlib>
#include <iostream>
#include <fstream>
#include "LaserPoints.h"
#include "LaserPoint.h"
#include "ObjectPoints.h"
#include "ObjectPoint2D.h"
#include "LineTopologies.h"
#include "Positions3D.h"
#include "LaserPatches.h"
#include "SegmentationParameters.h"
#include "Circle2D.h"
#include "DataBoundsLaser.h"
#include "Image.h"
#include "ImageGrid.h"
#include "InlineArguments.h"
#include <stdio.h>  /* defines FILENAME_MAX */
#ifdef WINDOWS
    #include <direct.h>
    #define GetCurrentDir _getcwd
#else
    #include <unistd.h>
    #define GetCurrentDir getcwd
 #endif


#define PI 3.1415926

using namespace std;

double section_height=0.25;
double system_height=2.4;




void itoa ( unsigned long val, char *buf, unsigned radix ) //function to convert int to char *
{
        char *p;                /* pointer to traverse string */
        char *firstdig;         /* pointer to first digit */
        char temp;              /* temp char */
        unsigned digval;        /* value of digit */

        p = buf;
        firstdig = p;           /* save pointer to first digit */

        do {
            digval = (unsigned) (val % radix);
            val /= radix;       /* get next digit */

            /* convert to ascii and store */
            if (digval >9)
                *p++ = (char ) (digval - 10 + 'a');  /* a letter */
            else
                *p++ = (char ) (digval + '0');       /* a digit */
        } while (val > 0);

        /* We now have the digit of the number in the buffer, but in reverse
           order.  Thus we reverse them now. */

        *p-- = '\0';            /* terminate string; p points to last digit */

        do {
            temp = *p;
            *p = *firstdig;
            *firstdig = temp;   /* swap *p and *firstdig */
            --p;
            ++firstdig;         /* advance to next two digits */
        } while (firstdig < p); /* repeat until halfway */
}


int Nearest(LaserPoint pt, LaserPoints pts)
{
double max_distance=999999;
double distance;
int index=-1;
for(int i=0;i<pts.size();i++)
 {
    distance=pts[i].Distance(pt);
    if(distance<max_distance)
       {max_distance=distance; index=i;}
 }           

return index;           
           
}

int part_type(LaserPoints lpts)

{
   
   if (lpts.size()<10)
   return 0;
  
 
   
  ObjectPoints objpts, objpts_local, objpts_convexhull, objpts_mbr, objpts_circle, objpts_bpolygon, objpts_xy;
  LineTopologies tops,tops_local, tops_ground, tops_facade, tops_onroad, tops_convexhull, tops_mbr, tops_circle, tops_bpolygon;
  LineTopology top;
  Plane myplane;
  DataBoundsLaser bound;
  Position3D point3d;
  double max_edge_dist=0.1;
  
  PointNumberList        component;
  LineTopology           laser_contour;
  TINEdges               edges, edges_rotate;
   
   lpts.Label(0);
   
  
   
   bound=lpts.DeriveDataBounds(false);
  
   for(int i=0;i<lpts.size();i++)
  {
   lpts[i].SetX(lpts[i].GetX()-bound.MidPoint().GetX());      
   lpts[i].SetY(lpts[i].GetY()-bound.MidPoint().GetY());        
   lpts[i].SetZ(lpts[i].GetZ()-bound.MidPoint().GetZ());          
  }

  //lpts.Write("part_temp.laser",0);
  
  LaserPoints lpts2d, lpts_project;
  Position3D point;
  Plane vertical_plane(lpts[0], Vector3D(1,0,0));
  
  lpts2d=lpts;
  lpts_project=lpts;
  
  for(int i=0;i<lpts.size();i++)
     lpts2d[i].SetZ(0);
    
  for(int i=0;i<lpts.size();i++)
  {   
   point=vertical_plane.Project(lpts[i]);
   lpts_project[i].SetX(point.GetX());
   lpts_project[i].SetY(point.GetY());
   lpts_project[i].SetZ(point.GetZ());
   } 
   //lpts_project.Write("project.laser",0);
  
  lpts_project.SwapXZ(); 
   //lpts_project.Write("project_swapped.laser",0);
   
  
   
  lpts2d.DeriveTIN();
  edges=TINEdges(lpts2d.TINReference());
  
  double area_horizontal;
  double area_vertical; 
 
  if(lpts2d.EnclosingRectangle(max_edge_dist, objpts_mbr, top))
  {
  area_horizontal=objpts_mbr[0].Distance(objpts_mbr[1])*objpts_mbr[0].Distance(objpts_mbr[2]);
  }
  
  
  objpts_mbr.clear();
  top.clear();
  lpts_project.DeriveTIN();
 
  if(lpts_project.EnclosingRectangle(max_edge_dist, objpts_mbr, top))
  {
 
  area_vertical=objpts_mbr[0].Distance(objpts_mbr[1])*objpts_mbr[0].Distance(objpts_mbr[2]);
  }
  //cout<<"segment "<<lpts[0].Attribute(SegmentNumberTag)<<" horiztontal and vertical area is "<<area_horizontal<<" "<<area_vertical<<endl;
  
  if(area_horizontal>15&&area_vertical>15)
  return 3;//tree
  
  edges.clear();
  myplane=lpts.FitPlane(0,0,LabelTag);
  //myplane.Print();
  //cout<<myplane.Normal().DotProduct(Vector3D(0,0,1))<<endl;;

  if(!myplane.IsVertical(10*PI/180))
    {//cout<<"not vertical enough"<<endl;
                                    return 0;}
  
  //force planar
   for(int i=0;i<lpts.size();i++)
  {
    point3d=myplane.Project(lpts[i]);
    lpts[i].SetX(point3d.GetX());   lpts[i].SetY(point3d.GetY());  lpts[i].SetZ(point3d.GetZ());    
  }
  
  //objpts_xy=lpts.ConstructObjectPoints () ;
 // for(int i=0;i<objpts_xy.size();i++)\
  //objpts_xy[i].SetZ(0);
  
  
  double angle_planes;
  angle_planes=Angle(myplane.Normal(), Vector3D(0,0,1));
  
  Line3D myline3d;
  Plane horizontal(lpts[0], Vector3D(0,0,1));
  
  Intersect2Planes (myplane, horizontal, myline3d);

   Rotation3D rot(myline3d.Direction(),angle_planes); 
   for(int i=0;i<lpts.size();i++)
  { 
           point3d=rot*lpts[i];
           lpts[i].SetX(point3d.GetX());   lpts[i].SetY(point3d.GetY());  lpts[i].SetZ(point3d.GetZ());    
  }

  for (int i=0; i<lpts.size(); i++) 
    component.push_back(PointNumber(i));
  
  LaserPoints lpts_temp;
  
  lpts_temp=lpts;
  
 
  lpts_temp.DeriveTIN();
  
  edges_rotate=TINEdges(lpts_temp.TINReference());
  
 
  lpts_temp.RemoveLongEdges(edges_rotate,0.4);
  laser_contour = lpts_temp.DeriveContour(1, component, edges_rotate, false);

  laser_contour.push_back(laser_contour[0]);
  
  objpts_convexhull=lpts_temp.ConstructObjectPoints  () ;
  laser_contour.Label()=1;
  tops_convexhull.push_back(laser_contour);
  
  

 //objpts_convexhull.Write("convexhull.objpts");
  //tops_convexhull.Write("convexhull.top");
  
 
  
  lpts.EnclosingRectangle(max_edge_dist, objpts_mbr, top);
  top.Label()=2;
  tops_mbr.push_back(top);
  
  //objpts_mbr.Write("mbr.objpts");
  //tops_mbr.Write("mbr.top");
  
  Circle2D mycircle;
  top.clear();
  mycircle=lpts.MBC();
  mycircle.Polygon(36, objpts_circle, top);
  top.Label()=3;
  tops_circle.push_back(top);
  //objpts_circle.Write(filename_circle_objpts.c_str());
  //tops_circle.Write(filename_circle_top.c_str());
  
  DataBounds3D bound3d;
  
  bound3d=objpts_mbr.Bounds();

  
  Image myimage_convex(bound3d.XRange(), bound3d.YRange(), 0.005);
  ImageGrid myimagegrid(bound3d.Minimum().GetX(), bound3d.Maximum().GetY(), 0.005);
  tops_convexhull[0].AddToBitmap(objpts_convexhull, myimage_convex, myimagegrid);
  //myimage_convex.Write(filename_xv.c_str());
  
  
  Image myimage_mbr(bound3d.XRange(), bound3d.YRange(), 0.005);
  tops_mbr[0].AddToBitmap(objpts_mbr, myimage_mbr, myimagegrid);
  //myimage_mbr.Write(filename_mbr_xv.c_str());
  
  Image myimage_circle(bound3d.XRange(), bound3d.YRange(), 0.005);
  tops_circle[0].AddToBitmap(objpts_circle, myimage_circle, myimagegrid);
 // myimage_circle.Write(filename_circle_xv.c_str());
  
  int count_mbr=0;
  int count_circle=0;
  int count_match_mbr=0;
  int count_match_circle=0;
  
  
  for(int i=0;i<myimage_mbr.NumRows();i++ )
    for(int j=0;j<myimage_mbr.NumColumns();j++)
    {
       if((*(myimage_mbr.Pixel(i,j))==255)||(*(myimage_convex.Pixel(i,j))==255))
         count_mbr++;
         
       if((*(myimage_mbr.Pixel(i,j))==255)&&(*(myimage_convex.Pixel(i,j))==255))
          count_match_mbr++;
         
        if((*(myimage_circle.Pixel(i,j))==255)||(*(myimage_convex.Pixel(i,j))==255))
         count_circle++;
         
       if((*(myimage_circle.Pixel(i,j))==255)&&(*(myimage_convex.Pixel(i,j))==255))
          count_match_circle++;
            
     }
  
  //cout<<"Union mbr with convexhull is "<< count_mbr<<endl;
 // cout<<"Union circle with convexhull is "<<count_circle<<endl;
 // cout<<"Match rate with mbr is "<<count_match_mbr*100/count_mbr<<" percent."<<endl;
 // cout<<"Match rate with circle is "<<count_match_circle*100/count_circle<<" percent."<<endl;  
  if(count_match_mbr*100/count_mbr>80)  
  return 1;//rectangle
  
  if(count_match_circle*100/count_circle>80)  
  return 2;//circle
  
  return 0;//unknown
  
}





LaserPoints slice_laser(LaserPoints lpts, double lower, double upper)
{
   LaserPoints sliced;
   
   for(int k=0;k<lpts.size();k++)
    {
    if(lpts[k].GetZ()>lower&&lpts[k].GetZ()<upper)
       sliced.push_back(lpts[k]);         
    }
   return sliced; 
            
}

void PrintUsage()
{
     printf("Usage: pa [-i <filename>] [-t <trajectory file>] [-o_report <output report file>] [-o_points] <output ascii points file>\n"); 
}

int main(int argc, char *argv[])
{
   

  LaserPoints lpts,lpts_traj, lpts_out,lpts_onroad, lpts_local, lpts_core, lpts_pole;
  ObjectPoints objpts, objpts_local;
  LineTopologies tops,tops_local, tops_ground, tops_facade, tops_onroad;
  Circle2D               circle_part;
  LaserPoint lpt_center, nearest_traj;
  LineTopology top;
  Positions3D traj;
   Plane localplane;
  LaserPoints lpts_out_vertical;
  
  LaserPoints pole, parts, part;
  
  double height, z_max, z_min;
  DataBoundsLaser bound, bound_local;
  
  Position3D pole_center;
  
  int pole_counter=1;
  
  int has_roadsign, has_warningsign, is_tree;
  
  InlineArguments *args = new InlineArguments(argc, argv);
  
  if (!args->Contains("-i")) {
    printf("Error: no input data specified with -i <filename>.\n");
    PrintUsage();
    exit(0);
  }
  
  if (!args->Contains("-t")) {
    printf("Error: no trajectory data specified with -t <filename>.\n");
    PrintUsage();
    exit(0);
  }
  
   if (!args->Contains("-o_report")) {
    printf("Error: no output report filename specified with -o_report <filename>.\n");
    PrintUsage();
    exit(0);
  }
  
   if (!args->Contains("-o_points")) {
    printf("Error: no output ascii file for points specified with -o_points <filename>.\n");
    PrintUsage();
    exit(0);
  }
  
  lpts.Read(args->String("-i"));
  
  
  traj.Read(args->String("-t"));
  
  
  for(int i=0;i<traj.size();i++)
  lpts_traj.push_back(LaserPoint(traj[i].GetX(),traj[i].GetY(), traj[i].GetZ()-system_height));
  
  //lpts.Label(0);
  
  SegmentationParameters *segpar;
  segpar=new SegmentationParameters();
  segpar->MaxDistanceInComponent()=0.3;
  
  //segpar->Read("segpar.ini");
   
  
  vector<int> indices=lpts.GetAttribute(LabelTag);
   int max_segment_value=0;  
   for(int i=0;i<indices.size();i++)
      max_segment_value=(max_segment_value>indices[i])?max_segment_value:indices[i]; 
  
  int number_sections;
   LaserPoints sections[1000];
   
   Position3D centers[1000];
   
   int index_minbound, index_maxbound;
   double area;
 
 double distance, max_distance;
   max_distance=0;
   
   double max_radius=0;
 
 int counter=1;
 
 int considered_sections;
 
 
 char cCurrentPath[FILENAME_MAX];

 if (!GetCurrentDir(cCurrentPath, sizeof(cCurrentPath)))
     {
     return errno;
     }

 string filename_report, filename_points;

 filename_report=cCurrentPath;
 filename_report+="\\";
 filename_report+=args->String("-o_report");
 
 filename_points=cCurrentPath;
 filename_points+="\\";
 filename_points+=args->String("-o_points");


 ofstream os, os_points;
   
   os.open(filename_report.data(), ios_base::app);
   os_points.open(filename_points.data(),ios_base::app);
   
   if(!os.is_open())
   cout<<"Cannot open "<<args->String("-o_report")<<endl;
   
   if(!os_points.is_open())
   cout<<"Cannot open "<<args->String("-o_points")<<endl;
   
    os.precision(2);
    os_points.precision(2);
   
  //LaserPoints lpts_output;
  
  //lpts_output.Read("poles.laser");
 
 
  for(int k=0;k<=max_segment_value; k++)
  {
    
  lpts_local=lpts.SelectTagValue(LabelTag, k);       
  
  if(lpts_local.size()<20)
  continue;
   
  
  lpts_core=lpts_local.SelectTagValue(ComponentNumberTag, 1);
  
  //lpts_output=lpts_output+lpts_core;
   
  if(lpts_core.size()<20)
  continue;
   
  
 
   bound=lpts_core.DeriveDataBounds(false);
   
   nearest_traj=lpts_traj[Nearest(bound.MidPoint(),lpts_traj)];
   
   if(bound.Minimum().Z()>nearest_traj.GetZ()+0.5)  //we don't want the poles flying in the air
   continue; 
   
   if(bound.ZRange()<1)  //we don't want very low poles
   continue;
   
   cout<<"dealing with pole "<<lpts_core[0].Attribute(SegmentNumberTag)<<" component label is "<<k<<endl;
   
   height=bound.ZRange(); 
   z_min=bound.Minimum().GetZ();
   
   number_sections=height/section_height+1;  
  

   
   considered_sections=number_sections>2?number_sections/2:1;
   
     max_radius=0;
     index_maxbound=-1;
   
   lpts_pole.clear();
   for(int i=0;i<considered_sections;i++)
     {
           sections[i]=slice_laser(lpts_core, z_min+section_height*i, z_min+section_height*i+section_height);       
           bound_local=sections[i].DeriveDataBounds(false);
           //area=bound_local.XRange()*bound_local.YRange();
           centers[i]=bound_local.MidPoint();
           
           max_distance=0;
           for(int j=0;j<sections[i].size();j++)
              {
                distance=sections[i][j].Position2DOnly().Distance(centers[i].Position2DOnly());
     
                     if(distance>max_distance)
                     {max_distance=distance;  }       
              }
          // cout<<"The radius of section "<<i<<" is "<<max_distance<<endl;   
           if(max_distance>0.5)
              continue;
           
           lpts_pole=lpts_pole+sections[i];
           
           if(max_distance>max_radius)
           { 
           max_radius=max_distance; 
           //pole_center=centers[i]; 
           index_maxbound=i;
           } 
         
     }
   
   if(considered_sections==1)
   {
   lpts_pole=lpts_core;
   max_radius=distance;
   index_maxbound=1;
   }
   
   bound_local=lpts_pole.DeriveDataBounds(false);
  
 // cout<<"The radius of the pole "<<k<<" is "<<max_radius<<" based on section: "<<index_maxbound<<endl;
  
  if(index_maxbound==-1)  //point density too low to retrieve sufficient points in each section
  continue; 
  
   Circle2D pole_circle(bound_local.MidPoint().Position2DOnly(), max_radius);
   
  
   
   parts.clear();
   for(int i=0;i<lpts_local.size();i++)
   {
   if(pole_circle.Inside(Position2D(lpts_local[i].GetX(), lpts_local[i].GetY())))
     lpts_local[i].SetAttribute(IsFilteredTag, 0);
   else  
      {
      lpts_local[i].SetAttribute(IsFilteredTag, k);
      parts.push_back(lpts_local[i]);
      }
   }
  

   os<<fixed<<bound_local.MidPoint().GetX()<<" "<<bound_local.MidPoint().GetY()<<" "<<bound_local.Minimum().GetZ()<<" "<<height<<" "<<max_radius;
  //Start to isolate poles and parts
  
   TINEdges    *edges;
 
  
  // Derive the edges that define the neighbour relations
  edges = parts.DeriveEdges(*segpar);
  
  // Remove long edges
  if (segpar->MaxDistanceInComponent() > 0.0)
    parts.RemoveLongEdges(edges->TINEdgesRef(), 
                       segpar->MaxDistanceInComponent(),
                       segpar->DistanceMetricDimension() == 2);
 
  // Label the connected components
  parts.LabelComponents(edges->TINEdgesRef(), SegmentNumberTag);
  
  vector<int> indices2=parts.GetAttribute(SegmentNumberTag);
  int max_segment_value2=0;
   for(int i=0;i<indices.size();i++)
      max_segment_value2=(max_segment_value2>indices[i])?max_segment_value2:indices[i]; 
    
   PointNumberList plist;
  int type;
  //counter_part=1;
   has_roadsign=has_warningsign=is_tree=0;
   
 
   
  for(int i=0;i<=max_segment_value2;i++)
  {
     part=parts.SelectTagValue(SegmentNumberTag,i);
     plist=parts.SelectTagValueList(SegmentNumberTag,i);
   
     
     if(part.size()==0)
     continue;
     
     
    // if(part.size()<10)
    // parts.Label (plist, 1, IsSelectedTag);
     
   
     type=part_type(part);
     
     switch(type)
     {
                 case 1: has_roadsign=1;break;
                 case 2: has_warningsign=1;break;
                 case 3: is_tree=1;break;              
     }
     
    
     //parts.Label(plist, type, IsFilteredTag);
     
  }
  
  //lpts_local.Label(is_tree);
  //lpts_output=lpts_output+lpts_local;
  
  for(int t=0;t<lpts_local.size();t++)
  {
  os_points<<fixed<<lpts_local[t].GetX()<<" "<<lpts_local[t].GetY()<<" "<<lpts_local[t].GetZ()<<" "<<is_tree<<" "<<lpts_local[t].HasAttribute(ComponentNumberTag)<<endl;
  }
  
  
  
  //parts.RemoveTaggedPoints(1, IsSelectedTag);
  os<<" "<<has_roadsign<<" "<<has_warningsign<<" "<<is_tree<<endl;
  cout<<"finish of this component"<<endl;
   }
  
  //parts.Write("pole_parts.laser",0);
  
  //lpts_output.Write(false);
  os.close();
  os_points.clear();

}
  
   
   
   

