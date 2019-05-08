#include <cstdlib>
#include <iostream>
#include <fstream>
#include "mystruct.h"
#include "LaserPoints.h"
#include "LaserPoint.h"
#include "LaserBlock.h" 
#include "LaserUnit.h"
#include "ObjectPoints.h"
#include "ObjectPoint.h"
#include "LineTopologies.h"
#include "Positions3D.h"
#include <stdio.h>
#include <windows.h>

Plane ReCalculate(int index, int segmentNo, LaserPoints Mypoints,TINEdges *edges, double near_range, Plane formerplane)
{
     
      PointNumberList taggedneighbourhood;
      Plane recalculate_plane;
      /*Plane recalculate_plane;
      int number;      
      const LaserPointTag tag=SegmentNumberTag;
      
      //cout<<"sdf"<<"\n";
      taggedneighbourhood=Mypoints.TaggedNeighbourhood(PointNumber(index),segmentNo,near_range,*edges,tag,false,false);
      cout<<"sdf"<<"\n";
      number=taggedneighbourhood.size();
      cout<<number<<"\n";

      if(number>=3){
        recalculate_plane = Mypoints.FitPlane(taggedneighbourhood, segmentNo);   
        return recalculate_plane;
      }
      else return formerplane;*/
      TINEdges::iterator          neighbours;
      LaserPoints::const_iterator point,nb_point;
      TINEdgeSet::iterator        nb_node;
      int                         total_number;
      double                      distance;
      
      distance=0.0;
      total_number=0;
     // point=Mypoints.begin()+index;
      neighbours=edges->begin()+ index;  
      for (nb_node=neighbours->begin(); nb_node!=neighbours->end(); nb_node++) 
      {
          nb_point=Mypoints.begin()+ nb_node->Number();
         // distance=point->Distance(nb_point->Position3DRef());
          if (nb_point->Attribute(SegmentNumberTag)==segmentNo /*&& distance<= near_range*/)
          {
           taggedneighbourhood.push_back(nb_point->Attribute(PointNumberTag));
           total_number++;
           }
      }      
      if(total_number<=3)
      {
        recalculate_plane = Mypoints.FitPlane(taggedneighbourhood,1);          
        return recalculate_plane;
      }
      else return formerplane;
}
