#include <cstdlib>
#include <iostream>
#include "struct.h" 
#include "LaserPoints.h"
#include <math.h>
using namespace std;

/*double P2PDistance(LaserPoint p_mid, LaserPoint p_nei)
{
      double distance=0.00;
      distance=sqrt((p_mid.GetX()-p_nei.GetX())*(p_mid.GetX()-p_nei.GetX())+(p_mid.GetY()-p_nei.GetY())*(p_mid.GetY()-p_nei.GetY())+(p_mid.GetZ()-p_nei.GetZ())*(p_mid.GetZ()-p_nei.GetZ()));
      return distance;
      };

LaserPoint FindNearestPoint(LaserPoints lpts, LaserPoint p_Mid)
{
       LaserPoints::iterator Lpt;
       LaserPoint myPoint;
       double dist;
       double min_dist=1.00; 
       for (Lpt=lpts.begin(); Lpt!=lpts.end(); Lpt++)    
       {
           dist=P2PDistance(p_Mid,*Lpt);
           cout<<dist<<"\n";
           if(dist<min_dist)
           {
                min_dist=dist;
                myPoint=*Lpt;
           }
       }
       cout<<min_dist<<"nearest \n";
          // cout<<min_dist<<"\n"; 
       return myPoint;
       };*/
