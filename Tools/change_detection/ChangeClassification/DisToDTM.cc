#include "class_par.h"
//rand the points z and caculate the average of z
double Rank(LaserPoints &mypoints, PointNumberList &mypnl,int return_h)//return_h 1 maximum, 2 minimum, 3 average
{
     PointNumberList::iterator lpt;  
       
     double z_max=0.00,z_min=9999.999,average=0.0;
     
     for(lpt=mypnl.begin();lpt!=mypnl.end();lpt++)
     {         
          if(mypoints[lpt->Number()].Z()>z_max)z_max=mypoints[lpt->Number()].Z();
          if(mypoints[lpt->Number()].Z()<z_min)z_min=mypoints[lpt->Number()].Z();
          average=mypoints[lpt->Number()].Z()+average;          
     }
     average=average/mypnl.size();
    
     if(return_h==1)return z_max;
     if(return_h==2)return z_min;
     if(return_h==3)return average;
     }
//using pointnumberlist to compute the center of a group of points     
ObjectPoint Center(LaserPoints &lpts,PointNumberList &pnl)
{
    PointNumberList::iterator   node;
    double                       x=0.0,y=0.0,z=0.0;
    ObjectPoint                 centerpt;
    int                         size=0;
    
    for(node=pnl.begin();node!=pnl.end();node++)
    {
        x=x+lpts[node->Number()].X();
        y=y+lpts[node->Number()].Y();
        z=z+lpts[node->Number()].Z();
        size++;
    }
    centerpt.Position3DRef().X()=x/size;
    centerpt.Position3DRef().Y()=y/size;
    centerpt.Position3DRef().Z()=z/size;
    return centerpt;
}   

//caculate the distance from center point to nearest DTM points  
double DisToDTM(PointNumberList &pnl_seg, LaserPoints &laser_points, LaserPoints &dtm_laser_points,TINEdges &edges,int return_height)
{
   // PointNumberList::iterator   noden;
    PointNumberList             neighbourhood;
   // PointNumberList::iterator   node;
    int                         nearest_dtm_point;
    ObjectPoint                 centrobj;    
    double                      z_dtm, z_seg, nbh_radius, seg2dtmOUT;   
    
    //find the nearest DTM points
    centrobj =Center(laser_points,pnl_seg);
    
    nearest_dtm_point = dtm_laser_points.NearestPoint(centrobj.Position3DRef(),edges, true);    
   // cout<<"nearest DTM point:"<<dtm_laser_points[nearest_dtm_point].X()<<","<<dtm_laser_points[nearest_dtm_point].Y()<<","<<dtm_laser_points[nearest_dtm_point].Z()<<endl;
    nbh_radius = 3;    
    do
    {
        neighbourhood = dtm_laser_points.Neighbourhood(PointNumber(nearest_dtm_point),nbh_radius, edges,true, false);
        nbh_radius=nbh_radius*2;
    }while(neighbourhood.size()<size && nbh_radius<radius);   
  
    //distance
     z_dtm = Rank(dtm_laser_points,neighbourhood,3);
     z_seg = Rank(laser_points,pnl_seg,return_height);
    // cout<<"z_dtm:"<<z_dtm<<"  z_seg:"<<z_seg<<" segmentno:"<<laser_points[pnl_seg.begin()->Number()].Attribute(SegmentNumberTag)<<endl;
    seg2dtmOUT = z_seg - z_dtm;
    neighbourhood.Erase();  
    
    return seg2dtmOUT;
}
