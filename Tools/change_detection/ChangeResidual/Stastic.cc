#include <cstdlib>
#include <iostream>
#include "mystruct.h"
       
int *Stastic(float data)
{
     int n= nClass;       
     int i;
     float j;
     for(i=0,j=-1.00f*myClass;i<n,j<=myClass;i++)
     {             
          if(data>=j && data <=j+subClass)Sta[i]++; 
          j=j+subClass;                                 
             }
     int *p=Sta;
     return p;
     };
