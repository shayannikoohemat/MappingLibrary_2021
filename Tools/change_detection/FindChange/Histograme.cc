#include <cstdlib>
#include <stdio.h>
#include <iostream>
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include "struct.h"
using namespace std;
int Max(int *a)
{
    int maxvalue=0;
    for (int i=0; i<nClass;i++)
    {
        if(a[i]>maxvalue) 
        {
           maxvalue=a[i];
           center=0.01f*(i-100);
           }
        }       
        return maxvalue;
    };
    
IplImage* Histograme(int *p)
{
     IplImage* hist_image = cvLoadImage("D:\\1.jpg", 1 );
     int MV=Max(p);
     int j=5;
     int intensity;
     
    // cvZero( hist_image ); 
    for (int i=0; i<nClass;i++)
    {
        if(MV==0) intensity=0;
        else
        {
            intensity = cvRound(p[i]*200/MV);
            //cout<<intensity<<"\n"; 
            cvRectangle(hist_image,cvPoint(j,hist_image->height-intensity-50),cvPoint(j+5, hist_image->height-50), CV_RGB(intensity,intensity,intensity),CV_FILLED ); 
            j=j+5;
        }
        }
        
     //cvNamedWindow( "MyHist", 1 );
     //cvShowImage( "MyHist",hist_image);
     return hist_image;
     //cvWaitKey(0);
  

     };
