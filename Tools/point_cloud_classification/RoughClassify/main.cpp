
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
#include "Road.h"
#include "BNF_io.h"
#include "Database4Cpp.h"
#include <cmath>


#define PI 3.1415926
using namespace std;

double ground_z_min, ground_z_max;
int header_lines=4;

//the parameter file deault values
double system_height=2.16;

int force_segmentation=0;
int local_begin=0;
int local_end=99999;

char *feature_name= (char *) "noname";
int segment_min_number_points=50;
double segment_min_height=1.5;
double segment_max_height=1000;
double segment_min_area=20;
double segment_max_area=10000;
double is_vertical_degree=5;
double cut_low=0;
double cut_high=1000;
double cut_elevation=1000;
int cut_min_number_points=10;
double cut_max_diameter=0.6;
int output_component=1;
int filter_building=1;


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



bool ReadOneFeature(FILE *fd)
{
char   *buffer, *line, *keyword;
  int    keyword_length;


  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
       
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      
      if (keyword) {
          
          // cout<<keyword<<endl;   
      if (!strncmp(keyword, "featureparameters", 17))
      {
       
      }            
        else if (!strncmp(keyword, "feature_name", 12))
          feature_name = BNF_String(line);
          
        else if (!strncmp(keyword, "segment_min_number_points", 25))
          segment_min_number_points = BNF_Integer(line);
          
        else if (!strncmp(keyword, "segment_min_height", 18))
          segment_min_height = BNF_Double(line);

        else if (!strncmp(keyword, "segment_max_height", 17))
          segment_max_height = BNF_Double(line);  
          
        else if (!strncmp(keyword, "segment_min_area", 16))
          segment_min_area = BNF_Double(line);   
        
        else if (!strncmp(keyword, "segment_max_area", 16))
          segment_max_area = BNF_Double(line);     
        
        else if (!strncmp(keyword, "is_vertical_degree", 18))
          is_vertical_degree = BNF_Double(line);
        
        else if (!strncmp(keyword, "cut_low", 7))
          cut_low = BNF_Double(line);
        
        else if (!strncmp(keyword, "cut_high", 8))
          cut_high = BNF_Double(line);
        
        else if (!strncmp(keyword, "cut_elevation", 13))
          cut_elevation = BNF_Double(line);
        
        else if (!strncmp(keyword, "cut_min_number_points", 21))
          cut_min_number_points = BNF_Integer(line);
          
        else if (!strncmp(keyword, "cut_max_diameter", 16))
          cut_max_diameter = BNF_Double(line);    
        
        else if (!strncmp(keyword, "output_component", 16))
          output_component = BNF_Integer(line); 
          
        else if (!strncmp(keyword, "filter_building", 15))
          filter_building = BNF_Integer(line);  

        else if (!strncmp(keyword, "force_segmentation", 18))
          {  }    
        
         else if (!strncmp(keyword, "local_begin", 11))               
          {}  
          
         else if (!strncmp(keyword, "local_end", 9))               
          { }  
         else if (!strncmp(keyword, "system_height", 13))
          { }    
        
        // end keyword
        else if (!strncmp(keyword, "endfeatureparameters",20)) {
          free(buffer);
          return true;
        }

        else {
          keyword[keyword_length] = 0;
          fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
        }
      }
    }
  }
  return false;
}  


bool ReadHeader(FILE *fd)
{
char   *buffer, *line, *keyword;
  int    keyword_length;

  buffer = (char *) malloc(MAXCHARS);
  for (int i=0;i<header_lines; i++) {
    line = fgets(buffer, MAXCHARS, fd);   
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      
      if (keyword) {
         if (!strncmp(keyword, "system_height", 13))
          {system_height=BNF_Double(line);  }   
         
         else if(!strncmp(keyword, "force_segmentation", 18))
          {force_segmentation=BNF_Integer(line);  }    
        
         else if (!strncmp(keyword, "local_begin", 11))               
          {local_begin=BNF_Integer(line);  }  
          
         else if (!strncmp(keyword, "local_end", 9))               
          {local_end=BNF_Integer(line);  }  
          
                   
       else {
          keyword[keyword_length] = 0;
          fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
          return false;
        }
      }
    }
  }
  free(buffer);
  return true;
}  


bool Write(const char *filename)
{
  FILE *fd;
  int indent=0;
  fd = fopen(filename, "w");
  if (fd == NULL) return false;
  
  
  BNF_Write_Double(fd, "system_height", indent, system_height, "%.2f");
  BNF_Write_Integer(fd, "force_segmentation", indent, force_segmentation, "%d");
  BNF_Write_Integer(fd, "local_begin", indent, local_begin,"%d");
  BNF_Write_Integer(fd, "local_end", indent, local_end,"%d");
  
  
  BNF_Write_String(fd, "featureparameters", indent, NULL);
  
  BNF_Write_String(fd, "feature_name", indent+2, "pole");
  
  BNF_Write_Integer(fd, "segment_min_number_points", indent+2,
                   segment_min_number_points, "%d");
  BNF_Write_Double(fd, "segment_min_height", indent+2,
                   segment_min_height, "%.2f");
  BNF_Write_Double(fd, "segment_max_height", indent+2,
                   segment_max_height, "%.2f"); 
          
  BNF_Write_Double(fd, "segment_min_area", indent+2,
                   segment_min_area, "%.2f");     
  BNF_Write_Double(fd, "segment_max_area", indent+2,
                   segment_max_area, "%.2f");                                  
  BNF_Write_Double(fd, "is_vertical_degree", indent+2,
                   is_vertical_degree, "%.f");        
  
  BNF_Write_Double(fd, "cut_low", indent+2,
                   cut_low, "%.2f");      
  BNF_Write_Double(fd, "cut_high", indent+2,
                   cut_high, "%.2f");                    
  BNF_Write_Double(fd, "cut_elevation", indent+2,
                   cut_elevation, "%.2f");                  
  BNF_Write_Double(fd, "cut_min_number_points", indent+2,
                   cut_min_number_points, "%d");  
  BNF_Write_Double(fd, "cut_max_diameter", indent+2,
                   cut_max_diameter, "%.2f");          
                                    
  BNF_Write_Integer(fd, "output_component", indent+2,
                   output_component, "%d");  
  BNF_Write_Integer(fd, "filter_building", indent+2,
                   filter_building, "%d");                                    
                
  BNF_Write_String(fd, "endfeatureparameters", indent, NULL);
  
  fclose(fd);
  return true;
}

int main(int argc, char *argv[])
{
   
  FILE   *fd;
  
  fd = fopen("featureparameters.ini", "r");
   
   if(!fd)
     {
          cout<<"No feature parameter file found, writing default values."<<endl;
          Write("featureparameters.ini");
           fd = fopen("featureparameters.ini", "r");
  
            if(!fd)
             {
              cout<<"Error with writing default values."<<endl;
              exit(0);
               }
          }
  
 
  
  
  
  LaserPoints lpts, lpts_out,lpts_onroad, lpts_symbol, lpts_traj, lpts_temp, lpts_circle;
  Positions3D traj;
  ObjectPoints objpts, *objpts_local;
  LineTopologies tops,tops_local, tops_ground, tops_facade, tops_onroad;
  LineTopology top;
  LineTopology *top_local;
  Circle2D               circle_part;
  LaserPoint lpt_center, nearest_traj;
  TINEdges    *edges;
    LaserPoints lpts_local;
    LaserPatch lp;
    int line_index;
  
  Road           road;
  Road::iterator road_part;
  ObjectPoints   road_part_points;
  LineTopologies road_part_tops;
  int counter;
  int max_segment_value=0;
  
  if(argc!=3)
  {
  cout<<"Error, incorrect parameters."<<endl<<"Usage: rc <roadmetafile> <trajectory file>"<<endl;
  exit(0);
   }
  
  
  road.ReadMetaData(argv[1]);
  traj.Read(argv[2]);
  
  
  for(int i=0;i<traj.size();i++)
  lpts_traj.push_back(LaserPoint(traj[i].GetX(),traj[i].GetY(), traj[i].GetZ()-system_height ));
  
  road.ReadRoadPartOutlines();
  
 // road.ReadRoadPartInventories();
  
  
 
  
  SegmentationParameters *segpar;
  segpar=new SegmentationParameters();
  if(!segpar->Read("segpar.ini"))
  {
  cout<<"No segmentation parameter file found, writing default values."<<endl;                               
  segpar->MaxDistanceInComponent()=0.3;
  segpar->Write("segpar.ini");
  }
  
  
  ReadHeader(fd);
   road_part=road.begin()+local_begin;
  counter=local_begin;
  
  // cout<<local_begin<< " "<<local_end<<" "<<force_segmentation<<endl;
//start the loop;  

  vector<int> indices; 

while(road_part!=road.end()&&(road_part-road.begin())<=local_end) 
{  
  cout<<"Processing road part "<<counter++<<endl;
  fseek ( fd , 0 , SEEK_SET );  
  
  objpts_local=road_part->OutlinePoints();
  top_local=road_part->OutlineTopology();
  
  tops_local.clear();
  
  tops_local.push_back(*top_local);
 
  
  
  road_part->ReadLaserPoints();
  lpts=road_part->LaserData();
  road_part->EraseLaserPoints();
  
  if((!lpts.HasAttribute(SegmentNumberTag))||force_segmentation)
 {
     cout<<"Segmenting..."<<endl;
     lpts.SurfaceGrowing(*segpar); 
     //lpts.Write(false);
  }
  
  
  indices=lpts.GetAttribute(SegmentNumberTag);
     
   for(int i=0;i<indices.size();i++)
      max_segment_value=(max_segment_value>indices[i])?max_segment_value:indices[i]; 
  indices.erase(indices.begin(), indices.end());
  vector<int>().swap(indices);
 

 //cout<<"Determing convex hulls..."<<endl;
 lpts.ConvexHull(objpts, tops, max_segment_value);
// lpts.ErasePoints();
   
  LaserPatches lps(objpts,tops);
 
  double areas[lps.size()];
  double area; int counter_component;
 
   
  cout<<"Determing trajectory boundary..."<<endl;
  
  LaserPoints lpts_traj_local;
  DataBoundsLaser bound_traj;
   
    // DEBUG LOOP
   //for (int loop=0; loop<1000; loop++) {
  
  lpts_traj.Select(lpts_traj_local,*objpts_local, tops_local);
  bound_traj=lpts_traj_local.DeriveDataBounds(0);
  
 // printf("Debug loop %d\r", loop);
 // DEBUG LOOP END
// } 
  
  ground_z_min=bound_traj.Minimum().GetZ()-0.5;
  ground_z_max=bound_traj.Maximum().GetZ()+0.5;
  
  cout<<"Z bounds of local trajectories are: "<<ground_z_min<<"  "<<ground_z_max<<endl; 
  
  
  
   cout<<"Calculating segments' areas..."<<endl;
    //store the area values for each segment
     
    Position3D center;
   
    tops_ground.clear();
    tops_facade.clear();
    
    for(int i=0;i<lps.size();i++)
    {
   
    if(!lps[i].size())
      areas[i]=0;
    else
      {
             areas[i]=abs(lps[i].area3D());
             center=tops[i].Bounds(objpts).MidPoint();
             
          if(areas[i]>10&&lps[i].getPlane().IsHorizontal(20*PI/180)&&center.GetZ()>ground_z_min&&center.GetZ()<ground_z_max)
             { tops[i].SetAttribute(ClassTag,1); tops_ground.push_back(tops[i]); }
      }
    }
  cout<<tops_ground.size()<<" segments recognized as ground."<<endl;
  //tops_ground.Write("ground.top");


cout<<"start determining building facades and onroad points..\r"<<endl;

ObjectPoint2D center2d;

bool inside=false;

for(int i=0;i<lps.size();i++)
    {
   
    if((!lps[i].size())||tops[i].HasAttribute(ClassTag)) //not if it is already a ground segment
      continue;
    
    center=tops[i].Bounds(objpts).MidPoint();
             center2d=ObjectPoint2D(center.GetX(), center.GetY(), 0, 0, 0, 0);
    
    if(areas[i]>20&&lps[i].getPlane().IsVertical(5*PI/180)&&(tops[i].Bounds(objpts).Minimum().GetZ()<ground_z_max+0.5)&&tops[i].Bounds(objpts).ZRange()>3)
       {tops_facade.push_back(tops[i]); }
       
       
             
             
             inside=false;
             
             for(int j=0;j<tops_ground.size();j++)
             {   
             if(center2d.InsidePolygon(objpts, tops_ground[j]))
             { inside=true; break;}
             
             }
             
    //if(!inside&&lps[i].getPlane().IsVertical(5*PI/180)&&(tops[i].Bounds(objpts).Minimum().GetZ()<ground_z_max+0.5))
      //  tops_facade.push_back(tops[i]);      
               
             if(inside)
              { tops_onroad.push_back(tops[i]);   continue; }       

    }


//tops_onroad.Write("onroad.top");

cout<<"Start writing classification labels to laser points...\r"<<endl;

lpts_onroad.ReInitialise();

  int segment_no;
  bool label_match=false;
   
  for(int i=0;i<lpts.size();i++)
   {
   if(!lpts[i].HasAttribute(SegmentNumberTag))
   continue;
   
   segment_no=lpts[i].Attribute(SegmentNumberTag);     
  
   label_match=false;
   for(int j=0;j<tops_ground.size();j++)
   {
          
           if(segment_no==tops_ground[j].Attribute(LineLabelTag))
             {label_match=true; break;}
          
   }
   if(label_match)//ground point
      {lpts[i].SetAttribute(ClassTag, 1); continue;}
      
   label_match=false;
   for(int j=0;j<tops_facade.size();j++)
   {      
           if(segment_no==tops_facade[j].Attribute(LineLabelTag))
             {label_match=true; break;}
          
   }  
   if(label_match) //facade point
      {lpts[i].SetAttribute(ClassTag, 2); } 
      
      
   label_match=false;
   for(int j=0;j<tops_onroad.size();j++)
   {      
           if(segment_no==tops_onroad[j].Attribute(LineLabelTag))
             {label_match=true; break;}
          
   }  
   if(label_match) //onroad point
      {lpts[i].SetAttribute(ClassTag, 3); lpts_onroad.push_back(lpts[i]);   continue;   }    
     
   }   
   
 lpts.Write(false);
   
cout<<"Dealing with onroad points.."<<endl;

string command_mk, command_del;
  string filename_temp;
  command_mk="mkdir features\\";
  command_mk+=road_part->Name();
  command_del="rmdir features\\";
  command_del+=road_part->Name();
  command_del+=" /Q";
  
  
  
  system(command_del.c_str());
  system(command_mk.c_str());
  


  // Derive the edges that define the neighbour relations
  edges = lpts_onroad.DeriveEdges(*segpar);
  
  // Remove long edges
  if (segpar->MaxDistanceInComponent() > 0.0)
    lpts_onroad.RemoveLongEdges(edges->TINEdgesRef(), 
                       segpar->MaxDistanceInComponent(),
                       segpar->DistanceMetricDimension() == 2);

  // Label the connected components
  lpts_onroad.LabelComponents(edges->TINEdgesRef(), LabelTag);

  // Delete the edges
  delete edges;
  
  
  //start analyze the onroad points for specific features
   double height;
   DataBoundsLaser bound, bound_circle;
   double max_distance,distance;
   PointNumberList plist;
  
  indices=lpts_onroad.GetAttribute(SegmentNumberTag);
   max_segment_value=0;  
   for(int i=0;i<indices.size();i++)
      max_segment_value=(max_segment_value>indices[i])?max_segment_value:indices[i]; 
   vector<int>().swap(indices);
  
  
  while(ReadOneFeature(fd))
   {
   cout<<"Searching for feature: "<<feature_name<<endl;  
   
  
   lpts_symbol.ReInitialise(); 
    
   
      
  
   
    for(int n=0;n<=max_segment_value;n++)
    {
   
    lpts_local=lpts_onroad.SelectTagValue(SegmentNumberTag,n);
    plist=lpts_onroad.SelectTagValueList(SegmentNumberTag,n);
    
    //cout<<"Segment No. "<<n <<" contains "<<lpts_local.size()<<" points, ";
    
    if(lpts_local.size()<segment_min_number_points)
    continue;
    

    line_index=tops.FindLineByTagValue(LineLabelTag, n);
    bound=lpts_local.DeriveDataBounds(false);
    height=bound.ZRange();
    
    //cout<< ", height is "<<height;
    
    if(height<segment_min_height)
    continue;
    
    if(height>segment_max_height)
    continue;
    
    if(areas[line_index]<segment_min_area)
    continue;
    
    if(areas[line_index]>segment_max_area)
    continue;
      
    if(!lps[line_index].getPlane().IsVertical(is_vertical_degree*PI/180))
    continue;
    
    counter_component=0;
    
    lpt_center=bound.MidPoint();
    
    //find the nearest traj point, to locate a more accurate terrain elevation
    nearest_traj=lpts_traj[Nearest(lpt_center,lpts_traj)];
    
    
    for(int j=0;j<lpts_local.size();j++)
     {
     
     if((lpts_local[j].Z()<bound.Minimum().GetZ()+cut_high)&&(lpts_local[j].Z()>bound.Minimum().GetZ()+cut_low)&&(lpts_local[j].Z()<nearest_traj.GetZ()+cut_elevation))
      lpts_circle.push_back(lpts_local[j]);
      }
    
    //cout<<"   the section between 0.5 to 2 meter contains "<<lpts_circle.size()<<" points"; 
     
    if(lpts_circle.size()<=cut_min_number_points) //there should be at least 6 points
    continue;
                          
    bound_circle=lpts_circle.DeriveDataBounds(false);
       
  
     max_distance=0;
     for(int j=0;j<lpts_circle.size();j++)
              {
                distance=lpts_circle[j].Position2DOnly().Distance(bound_circle.MidPoint().Position2DOnly());
     
                     if(distance>max_distance)
                     {max_distance=distance;  }       
              }
     lpts_circle.ErasePoints();
    
    //cout<<", max diameter is "<<max_distance*2;
    
    if(max_distance<cut_max_diameter/2)
       {     
       //cout<<" recognized."<<endl;
        lpts_onroad.Label(plist, 0, ComponentNumberTag);  
        continue;
       }
    
    if(areas[line_index]>20&&lps[line_index].getPlane().IsVertical(5*PI/180)&&height>3) //constaint for wall  
       lpts_onroad.Label(plist, 2, ComponentNumberTag);   
       
   // cout<<"segment no. "<<line_index<< "  area is "<<areas[line_index]<<" is vertical: "<<lps[line_index].getPlane().IsVertical(5*PI/180)<<endl;
    lpts_local.ErasePoints();
    
    }
    
   
// cout<<"here0"<<endl;
   
   //for all the connected components, if a part of a component is labelled, then push out the whole component to result pool
   
   if(output_component)
   {
   indices=lpts_onroad.GetAttribute(LabelTag);
   max_segment_value=0;  
   for(int i=0;i<indices.size();i++)
      max_segment_value=(max_segment_value>indices[i])?max_segment_value:indices[i]; 
   vector<int>().swap(indices);
    
    for(int i=0;i<=max_segment_value;i++)
      {
            lpts_local=lpts_onroad.SelectTagValue(LabelTag, i);
            
            if(lpts_local.HasAttribute(ComponentNumberTag)&&lpts_local.size()>segment_min_number_points)
               {
                if(!filter_building)
                  lpts_symbol=lpts_symbol+lpts_local; 
                
               else{ 
                  lpts_temp=lpts_local.SelectTagValue(ComponentNumberTag,2)  ;       //check whether facade is within this component                                                         
                   if(lpts_temp.size()==0)// we don't want facade connected component
                      lpts_symbol=lpts_symbol+lpts_local;
                   lpts_temp.ErasePoints();     
                   }         
               }
      } 
     lpts_local.ErasePoints();
  }
  else
  {
      lpts_symbol=lpts_onroad.SelectTagValue(ComponentNumberTag,1);
  }
  
   filename_temp="features\\" ;
   filename_temp+=road_part->Name();
   filename_temp+="\\";
   filename_temp+=feature_name;
   filename_temp+=".laser";
   lpts_symbol.Write(filename_temp.c_str(),0);
   lpts_symbol.ErasePoints();
   
   lpts_onroad.RemoveAttribute(ComponentNumberTag);
   
   }//end of this feature
   
   filename_temp="features\\";
    filename_temp+=road_part->Name();
    filename_temp+="\\onroad_components.laser";
   lpts_onroad.Write(filename_temp.c_str(),0);
   
   filename_temp="features\\";
    filename_temp+=road_part->Name();
    filename_temp+="\\outlines.objpts";
   objpts.Write(filename_temp.c_str());
   
   filename_temp="features\\";
    filename_temp+=road_part->Name();
    filename_temp+="\\facade.top";
   tops_facade.Write(filename_temp.c_str());

   
   
   road_part++;
   
   road_part->EraseLaserPoints();
   lpts_onroad.ErasePoints(true);
   lpts.ErasePoints(true);
   lps.Release();
   ObjectPoints temp_objpts;
   objpts.Erase();
   objpts.swap(temp_objpts);
   LineTopologies tops_temp;
   tops.Erase();
   tops.swap(tops_temp);
   tops_ground.Erase();
   tops_ground.swap(tops_temp);
   tops_facade.Erase();
   tops_facade.swap(tops_temp);
   tops_onroad.Erase();
   tops_onroad.swap(tops_temp);
   
   delete [] areas;
   
   //break;
}

delete segpar;
fclose(fd);
}
