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
#include "struct.h"
#include <cv.h>
#include <cxcore.h>
#include <highgui.h>
#include <stdio.h>
#include <windows.h>

//check in the neighbourhood if there is a point     
void CompareDatasets(char *LsrPts1, char *LsrPts2, char *output1, char *output2,char *outputmix, char *imagePath)
{
      IplImage* Histograme(int *);
      int *Stastic(float);
      LaserPoint FindNearestPoint(LaserPoints, LaserPoint);
      Plane ReCalculate(int, int,LaserPoints,TINEdges *, double,Plane);
      
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
      Change change;
      double distance=0.0;
      int *mySta;      
      int segment_number=99999; 
      int near_index=0;
      //read data
      lpts1.Read(LsrPts1); epoch1 = lpts1.begin()->ScanNumber(true);
      lpts2.Read(LsrPts2); epoch2 = lpts2.begin()->ScanNumber(true);
      
      if(lpts1.size()>=3 && lpts2.size()>=3)
      {    
      //segement datasets            
        //segm_par.NumberOfNeighbours()=0.20;
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
      edges = mylpts.DeriveEdges(segm_par);
  
      //derive edges seperatedly            
      edge1=lpts1.DeriveEdges(segm_par);
      edge2=lpts2.DeriveEdges(segm_par); 
      //cout<<"edge derived"<<"\n";
  
      // Remove old labels
      mylpts.RemoveAttribute(LabelTag);  
                                      
      // Loop over all points
      for (lpt=mylpts.begin(), index=0; lpt!=mylpts.end(); lpt++, index++)
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
              // near_index =lpt_nb->Attribute(PointNumberTag);             
               if(lpt_nearest->HasAttribute(SegmentNumberTag))               
               {  
               //  cout<<"has segment number"<<"\n";
                 segment_number=lpt_nearest->Attribute(SegmentNumberTag);  
                 near_index=lpt_nearest->Attribute(PointNumberTag);                 
                 if(epoch_nb==epoch1) 
                 {   
                 // near_index = ((int) &*lpt_nearest - (int) &*(mylpts.begin())) / (int) sizeof(LaserPoint *);                                                 
                    currentplane=ReCalculate(near_index,segment_number,lpts1,edge1,0.5,planes1[segment_number]);  
                 //   currentplane=planes1[segment_number];
                    
                    //cout<<"n1"<<"\n";                           
                    if(currentplane.IsHorizontal(0.99)){ 
                    //recalculate t he plane                                                                        
                    distance=currentplane.Distance(lpt->Position3DRef());
                    mySta=Stastic(distance);
                   // lpt->Residual()=change.dist;
                   // cout<<change.dist<<"\n";                           
                    if(distance<=0.1f&& distance+0.1f>=0.0f)lpt->Label(4);                             
                    else if(distance<=0.4f && distance>0.1f) lpt->Label(5);
                    else if(distance+0.1f<=0.0f && distance+0.4f>0.0f) lpt->Label(5);
                    else if(distance>0.4f && distance<=0.75f) lpt->Label(6);
                    else if(distance+0.4f<=0.0f && distance+0.75f>0.00f) lpt->Label(6);
                    else if(distance>0.75f && distance<1.0f) lpt->Label(7);
                    else if(distance+0.75f<=0.0f && distance+1.0>0.0f) lpt->Label(7);
                          // else lpt->Label(1);
                          // fp<<change.dist<<" "<<change.drct_z<<"\n";                   
                    }
                    else lpt->Label(1);
                    //cout<<"labeled"<<"\n";
                 }  
                 else 
                 {              
                    // near_index = ((int) &*lpt_nearest - (int) &*(mylpts.begin())) / (int) sizeof(LaserPoint *);                       
                    currentplane=ReCalculate(near_index,segment_number,lpts2,edge2,1,planes2[segment_number]); 
                    //currentplane=planes2[segment_number];
                    //cout<<"n2"<<"\n";      
                    if(currentplane.IsHorizontal(0.99)){   
                    //recalculate the plane                   
                    distance=currentplane.Distance(lpt->Position3DRef());
                    //lpt->Residual()=change.dist;  
                    mySta=Stastic(distance);     
                   // cout<<change.dist<<"\n";                 
                    if(distance<=0.1f&& distance+0.1f>=0.0f)lpt->Label(4);                             
                    else if(distance<=0.4f && distance>0.1f) lpt->Label(5);
                    else if(distance+0.1f<=0.0f && distance+0.4f>0.0f) lpt->Label(5);
                    else if(distance>0.4f && distance<=0.75f) lpt->Label(6);
                    else if(distance+0.4<=0.0f && distance+0.75f>0.00f) lpt->Label(6);
                    else if(distance>0.75f && distance<1.0f) lpt->Label(7);
                    else if(distance+0.75f<=0.0f && distance+1.0>0.0f) lpt->Label(7);
                    }
                    else lpt->Label(1);
                  //  cout<<"labeled"<<"\n";
                 }  
               } 
               else lpt->Label(1);
          }  
          else 
          {
             //  cout<<"changed"<<"\n";
               if(epoch==epoch1)lpt->Label(2);
               else lpt->Label(3);
          }          
      }
      //also output the mixed version
      mylpts.Write(outputmix,false,false);   
      
      //Output Histogram and compute the center of the distribution         
      cvSaveImage(imagePath,Histograme(mySta)); 
               
      //seperate
      LaserPoints Lpts1, Lpts2;       
      for (lpt=mylpts.begin(); lpt!=mylpts.end(); lpt++)
      if (lpt->ScanNumber(true) == epoch2) Lpts2.push_back(*lpt);
      Lpts2.Write(output2, false, false);
           
      for (lpt=mylpts.begin(); lpt!=mylpts.end(); lpt++)
      if (lpt->ScanNumber(true) == epoch1) Lpts1.push_back(*lpt);
      Lpts1.Write(output1, false, false); 
      
      lpts1.ErasePoints();
      lpts2.ErasePoints();
      mylpts.ErasePoints();
      Lpts1.ErasePoints();
      Lpts2.ErasePoints();
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
