#include <cstdlib>
#include <iostream>
#include <fstream>
#include "LaserPoints.h"
#include "LaserPoint.h"
#include "LaserBlock.h" 
#include "LaserUnit.h"
#include "ObjectPoints.h"
#include "ObjectPoint.h"
#include "LineTopologies.h"
#include "Positions3D.h"
#include "Planes.h"
#include "mystruct.h"
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <stdio.h>
#include <windows.h>

//check in the neighbourhood if there is a point     
void CompareDatasets(char *LsrPts1, char *LsrPts2, char *outputmix, char *imagePath)
{
// GV      IplImage* Histograme(int *);
      int *Stastic(float);
// GV      LaserPoint FindNearestPoint(LaserPoints, LaserPoint);
      Plane ReCalculate(int, int,LaserPoints &,TINEdges *, double,Plane &);
      
      LaserPoints lpts1, lpts2, mylpts, lpt_neighbors;
      LaserPoints::iterator lpt, lpt_nb, lpt_nearest;
      
        
      TINEdges *edges, *edge1, *edge2;     
      SegmentationParameters segm_par;
      int epoch1, epoch2, index, epoch, epoch_nb, epoch_nearest;
      PointNumberList neighbourhood;
      PointNumberList::iterator node;
      double range=1.0;    
      bool found_nearby_point;
      Planes planes1, planes2;
      Plane currentplane; 
//      Change change;
      double distance=0.0;
      int *mySta;      
      int segment_number=99999; 
      int near_index=0;
      //read data
      lpts1.Read(LsrPts1); epoch1 = lpts1.begin()->ScanNumber(true);
      lpts2.Read(LsrPts2); epoch2 = lpts2.begin()->ScanNumber(true);
      
      if(lpts1.size()>=3 && lpts2.size()>=3)
      {          
       // segm_par.SeedNeighbourhoodRadius()=2.0;
    
        lpts1.SurfaceGrowing(segm_par,true, false, &planes1);
        lpts2.SurfaceGrowing(segm_par,true, false, &planes2);
      // cout<<"segmented"<<"\n";
      
      //set the pointnumber attribute to each point    
      for (lpt=lpts1.begin(), index=0; lpt!=lpts1.end(); lpt++, index++)
        lpt->SetAttribute(PointNumberTag, index);
      for (lpt=lpts2.begin(), index=0; lpt!=lpts2.end(); lpt++, index++)
        lpt->SetAttribute(PointNumberTag, index);
      //cout<<"point number assigned"<<"\n";
      // Merge data sets
      mylpts = lpts1;
      mylpts.insert(mylpts.end(), lpts2.begin(), lpts2.end());
      // Generate edges of the merged data
      //use 120           
      segm_par.NumberOfNeighbours()=120;
      edges = mylpts.DeriveEdges(segm_par);
  
      //derive edges seperatedly    
      //change the segmentation parameter            
      segm_par.NumberOfNeighbours()=20;        
      edge1=lpts1.DeriveEdges(segm_par);
      edge2=lpts2.DeriveEdges(segm_par); 
      //cout<<"edge derived"<<"\n";    
                                      
      // Loop over all points
      for (lpt=mylpts.begin(), index=0; lpt!=mylpts.end(); lpt++, index++)
      {
          if(lpt->Attribute(LabelTag)==4 || lpt->Attribute(LabelTag)==5 || lpt->Attribute(LabelTag)==6 || lpt->Attribute(LabelTag)==7)
        { 
          neighbourhood = mylpts.Neighbourhood(PointNumber(index), range, *edges,false, true);
          epoch = lpt->ScanNumber(true);
          found_nearby_point = false;       
          for (node=neighbourhood.begin(); node!=neighbourhood.end()&&!found_nearby_point; node++) 
          {
              lpt_nb = mylpts.begin() + node->Number();
              epoch_nb = lpt_nb->ScanNumber(true);                  
              if (epoch != epoch_nb) 
              {
                  found_nearby_point=true; 
                  lpt_nearest=lpt_nb;                                            
              }   
          }              
          if (found_nearby_point) 
          {                  
                 segment_number=lpt_nearest->Attribute(SegmentNumberTag);  
                 near_index=lpt_nearest->Attribute(PointNumberTag);
                 //check to which epoch this nearest point belong to                 
                 if(epoch_nb==epoch1) 
                 {   
                    //recalculate the plane                                    
                    currentplane=ReCalculate(near_index,segment_number,lpts1,edge1,0.5,planes1[segment_number]);                                                                                   
                    distance=currentplane.Distance(lpt->Position3DRef());
                    mySta=Stastic(distance);
                    lpt->Residual()=distance;                   
                 }  
                 else 
                 { 
                    //recalculate the plane          
                    currentplane=ReCalculate(near_index,segment_number,lpts2,edge2,1,planes2[segment_number]);                   
                    distance=currentplane.Distance(lpt->Position3DRef());
                    lpt->Residual()=distance;  
                    mySta=Stastic(distance);  
                 }               
          }           
        }  
        else if (lpt->Attribute(LabelTag)==2 )
          lpt->Residual() = 2.0;
        else if (lpt->Attribute(LabelTag)==3 )
          lpt->Residual() = -2.0;
      }
      //also output the mixed version
      mylpts.Write(outputmix,false,false);  
      
      //Output Histogram and compute the center of the distribution         
//GV      cvSaveImage(imagePath,Histograme(mySta));             
   
      lpts1.ErasePoints();
      lpts2.ErasePoints();
      mylpts.ErasePoints();
     // Lpts1.ErasePoints();
     // Lpts2.ErasePoints();
      }
      else
      {
          if(lpts1.size()<3)
          {
              for(lpt=lpts2.begin();lpt!=lpts2.end();lpt++)
              {lpt->Label(3);}
              }
          else
          {
              for(lpt=lpts1.begin();lpt!=lpts1.end();lpt++)
              {lpt->Label(2);}
              }
          }
  } ;
