#include "class_par.h"
//residual of points fitting a plane
double PResidual(LaserPoints &mypts, PointNumberList &mypnl,Plane &plane)
{
     double rs=0.0;
     PointNumberList::iterator lpt;
     for(lpt=mypnl.begin();lpt!=mypnl.end();lpt++)
     {   
        double dist=plane.Distance(mypts[lpt->Number()].Position3DRef());
        if(dist<0)dist=(-1.0)*dist;
        rs=rs+ dist;
     }
     return (rs/mypts.size());
}

//residual of z
double Residual(LaserPoints &mypts,PointNumberList &mypnl,double h)
{
     double                      rs=0.0;
     PointNumberList::iterator   lpt;
     int                         size=0;
      
     for(lpt=mypnl.begin();lpt!=mypnl.end();lpt++)
     {
        double dist=mypts[lpt->Number()].Z()-h;
        if(dist<0)dist=(-1.0)*dist;
        rs=rs+dist*dist;
        size++;
     }
     return (sqrt(rs/(size-1)));
} 

//slope variance
double CheckSlope(LaserPoints &mypts,PointNumberList &mypnl,LaserPoints::iterator centerpoint)
{
     double                       slope=0.00f;
     double                       slope_rs=0.00f; 
     PointNumberList::iterator   lpt;
     int                         size=0;
     
     for(lpt=mypnl.begin();lpt!=mypnl.end();lpt++)
     {
        if((mypts[lpt->Number()].X()-centerpoint->X())!=0)
        slope=slope+(mypts[lpt->Number()].Y()-centerpoint->Y())/(mypts[lpt->Number()].X()-centerpoint->X());   
        size++;
       // cout<<(lpt->Y()-centerpoint->Y())<<endl;        
     }
     slope=slope/size;
     //cout<<slope<<endl;  
      for(lpt=mypnl.begin();lpt!=mypnl.end();lpt++)
     {
        if((mypts[lpt->Number()].X()-centerpoint->X())!=0)
        {
           double slp=(mypts[lpt->Number()].Y()-centerpoint->Y())/(mypts[lpt->Number()].X()-centerpoint->X())-slope;
           slope_rs=slope_rs+slp*slp;   
        }     
     }
     slope_rs=sqrt(slope_rs/(size-1));    
     return slope_rs;
} 

//percentage of points
double Percentage(LaserPoints &mypts, PointNumberList &mypnl)
{
      double                      per=0.0;
      int                        count=0,size=0;
      PointNumberList::iterator  lpt;
      
      for(lpt=mypnl.begin();lpt!=mypnl.end();lpt++)
     {
         if(mypts[lpt->Number()].FloatAttribute(v_zTag)<v_z||mypts[lpt->Number()].FloatAttribute(v_xTag)<v_x||mypts[lpt->Number()].FloatAttribute(v_yTag)<v_y) count++;                    
         size++;
     }     
     per=(count*100.0f)/size;
     return per;
}

//average of points value
double Average(LaserPoints &mypts, PointNumberList &mypnl,LaserPointTag tag)
{
      PointNumberList::iterator  lpt;
      double                      average=0.0;
      int                        size=0;
      
     for(lpt=mypnl.begin();lpt!=mypnl.end();lpt++)
     {
         average = average + mypts[lpt->Number()].FloatAttribute(tag);  
         size++;                          
     }     
     average=average/size;
     return average;
}


double NonFirstPulsePntRatio(LaserPoints &mypts,PointNumberList &mypnl)
{
      PointNumberList::iterator  lpt;
      double                      per=0.0;
      int                        count=0,size=0;
      
     for(lpt=mypnl.begin();lpt!=mypnl.end();lpt++)
     {
         if(mypts[lpt->Number()].Attribute(PulseCountTag)>1)count++;  
         size++;                     
     }     
     per=(count*1.0f)/size;
     return per;
}
