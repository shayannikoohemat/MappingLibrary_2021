#include "class_par.h"

LaserPoints PointAttributes(LaserPoints &lpts)
{                                                                                                              
    LaserPoints::iterator           lpt,lpt_nb;
    PointNumberList                 neighbourhood;
    PointNumberList::iterator       node;
    double                          range=1.0,neighbour_averh,neighbour_sigma;
    int                             index;
    TINEdges                        *edges;
    SegmentationParameters          segm_par; 
    
    segm_par.NumberOfNeighbours()=40;
    edges=lpts.DeriveEdges(segm_par);   
    
    //z_variance and slope variance        
    for (lpt=lpts.begin(),index=0;lpt!=lpts.end(); lpt++,index++)
      {
        if(lpt->Attribute(LabelTag)==VegetationClass)
        {
          //using 3D neighbour points
          neighbourhood = lpts.Neighbourhood(PointNumber(index), range, *edges,false, true); 
          neighbour_averh=Rank(lpts,neighbourhood,3);
          neighbour_sigma=Residual(lpts,neighbourhood,neighbour_averh);
          lpt->SetAttribute(v_zTag,(float)neighbour_sigma);// z_variance and record in v_zTag     
          double slprs;
          slprs=CheckSlope(lpts,neighbourhood,lpt);        
          lpt->SetAttribute(v_xTag,(float)slprs);     //slope variance and record in v_xTag           
        } 
      }   
    neighbourhood.Erase();       
    return lpts;
    lpts.ErasePoints();
}

LaserPoints CheckContext(LaserPoints &lpts)
{ 
    LaserPoints::iterator            lpt,lpt_nb,nbs;
    TINEdges                         *edges;      
    PointNumberList                  neighbourhood,pnl;
    PointNumberList::iterator        node;    
    int                              index,label,label_nb,relation,up_down;
    vector<int>                      components,seg_nums,seg_nums_new;
    vector<int>::iterator            component,seg_id;
    double                            range=0.1,slope,count=0,local_e=0.0,global_e=0.0;
    SegmentationParameters           segm_par; 
    Plane                            lpt_plane;
    double                            residual,average_vy,percentage_slp,percentage_vz;
    bool                             found_diff_point,found_diff;
  
    segm_par.DistanceMetricDimension()=2;
    segm_par.NumberOfNeighbours()=160;
    edges=lpts.DeriveEdges(segm_par);  
    
    lpts.RemoveAttribute(PointNumberTag);
    lpts.RemoveAttribute(ComponentNumberTag);
    lpts.RemoveAttribute(PulseCountTag);
    
    //reorder the point number
     for(lpt=lpts.begin(),index=0;lpt!=lpts.end();lpt++,index++)
    {
       lpt->SetAttribute(PointNumberTag,index);
    }
    //check if there is any point with different label in a ranged neighbourhood
    for(lpt=lpts.begin(),index=0;lpt!=lpts.end();lpt++,index++)
    {
        label=lpt->Attribute(LabelTag);
        //only for points labeled as vegetation
        if(label==VegetationClass)
        {      
          found_diff_point=false;
          do{
          neighbourhood = lpts.Neighbourhood(PointNumber(index), range, *edges,false, true); 
          range=range*2;
          count++;
          if(neighbourhood.size()>1)
          { 
            for (node=neighbourhood.begin(); node!=neighbourhood.end()&&!found_diff_point; node++) 
            {
                lpt_nb = lpts.begin() + node->Number();
                label_nb = lpt_nb->Attribute(LabelTag);                              
                //find the different labels    
                if (label_nb!=label) 
                {                  
                    found_diff_point=true;                                                                    
                } 
            }
            if(found_diff_point)
            {
               //give superority to building label in the nearest different label
               if(label_nb!=3)
               {
                  found_diff_point=false;
                  for (node=neighbourhood.begin(); node!=neighbourhood.end()&&!found_diff_point; node++) 
                  {
                      nbs=lpts.begin()+node->Number();
                      if(nbs->Attribute(LabelTag)==3){found_diff_point=true;lpt_nb=nbs;}
                  }
               }
               //coding the relationship using segment number, label and up-down relationship
               slope=lpt->Y()/lpt->X(); 
               if(lpt->Z()>lpt_nb->Z()) up_down=1;else up_down=2;           
               relation=lpt_nb->Attribute(SegmentNumberTag)*100+lpt_nb->Attribute(LabelTag)*10+up_down;
               lpt->SetAttribute(PulseCountTag,relation);  //record the context code in PulseCountTag
               if(lpt_nb->Attribute(LabelTag)==3)
               {
                  if(up_down==1)lpt->Label(AboveBuildingClass);
                  else lpt->Label(BuildingWallClass);
               }
              /* for (node=neighbourhood.begin(); node!=neighbourhood.end()&&!found_diff_point; node++) 
               {
                   nbs=lpts.begin()+node->Number();
                   neighbour_pts.push_back(*nbs);
               }  */                         
            }                    
          }
        neighbourhood.Erase(); 
        }while(!found_diff_point&&count<10);       
      }
    }   
    //count the number of different relationship codes
    components.push_back(99999);
    for(lpt=lpts.begin();lpt!=lpts.end();lpt++)
    {
       if(lpt->HasAttribute(PulseCountTag))
       {
          found_diff=false;
          for(component=components.begin();component!=components.end()&&!found_diff;component++)
          {
             if(lpt->Attribute(PulseCountTag)==(*component))found_diff=true;             
          }
          if(!found_diff)
          {components.push_back(lpt->Attribute(PulseCountTag));}
       }
       
    }  
    //regroup the points that have the same relationship and express them in ComponentNumberTag  
     for (lpt=lpts.begin(); lpt!=lpts.end();lpt++) 
       {      
          if(lpt->HasAttribute(PulseCountTag))
          {
            for (component=components.begin()+1,index=0;component!=components.end();component++,index++) 
            { 
                if(lpt->Attribute(PulseCountTag)==*component)lpt->SetAttribute(ComponentNumberTag,index);
            }
          }
       }
   components.clear();   
   seg_nums=lpts.AttributeValues(ComponentNumberTag);
   // re-component the unsegmented points in order to get rid of influence of unsegmented points on ground
   for(seg_id=seg_nums.begin();seg_id!=seg_nums.end();seg_id++)
   { 
      // seg_pts=lpts.SelectTagValue(ComponentNumberTag,*seg_id);
       pnl=lpts.TaggedPointNumberList(ComponentNumberTag, *seg_id);    
       for(node=pnl.begin();node!=pnl.end();node++)
       {
          if(!lpts[node->Number()].HasAttribute(SegmentNumberTag)&&lpts[node->Number()].Z()<6)          
          {             
              lpts.ConditionalReTag(*seg_id,99,ComponentNumberTag,node->Number(),PointNumberTag);
              lpts[node->Number()].SetAttribute(LabelTag,GroundClass);             
          }
          if(lpts[node->Number()].Attribute(PlaneNumberTag)>95)
          {
              lpts.ConditionalReTag(*seg_id,98,ComponentNumberTag,node->Number(),PointNumberTag); 
          } 
       }  
       //pnl.Erase();    
   }       
   
   //check connection relationship for each component
   seg_nums_new=lpts.AttributeValues(ComponentNumberTag);   
   for(seg_id=seg_nums_new.begin();seg_id!=seg_nums_new.end();seg_id++)
   { 
       //seg_pts=lpts.SelectTagValue(ComponentNumberTag,*seg_id);
       pnl=lpts.TaggedPointNumberList(ComponentNumberTag, *seg_id);
       int label_com,com_number;
       label_com=lpts[pnl.begin()->Number()].Attribute(LabelTag);
       com_number=lpts[pnl.begin()->Number()].Attribute(ComponentNumberTag);
       if(com_number>0&&(label_com==AboveBuildingClass||label_com==BuildingWallClass)){  
       {
           int correct_com=ConnComponent(lpts,pnl,edges);
           if(correct_com==4)
           { 
              lpts.ConditionalReTag(AboveBuildingClass,VegetationClass,LabelTag,*seg_id,ComponentNumberTag);
              lpts.ConditionalReTag(BuildingWallClass,VegetationClass,LabelTag,*seg_id,ComponentNumberTag);
              pnl.Erase(); 
           }
          else if(correct_com!=999)
           {               
              if(label_com==AboveBuildingClass)lpts.ConditionalReTag(VegetationClass,AboveBuildingClass,LabelTag,correct_com,ComponentNumberTag); 
              if(label_com==BuildingWallClass)lpts.ConditionalReTag(VegetationClass,BuildingWallClass,LabelTag,correct_com,ComponentNumberTag);                          
              pnl.Erase(); 
           }
       }       
     }            
   }  
     seg_nums.clear();
     seg_nums_new.clear();
     pnl.Erase();
     return lpts;       
}

/*//Energy for future use
LaserPoints Correct(LaserPoints &lpts)
{   
       float GlobalEnergy(LaserPoints &);
       float LocalEnergy(LaserPoints &, LaserPoints::iterator);
       
       LaserPoints                      neighbour_pts;
       LaserPoints::iterator            lpt,nbs;
       int                              index,label,count=0;
       PointNumberList                  neighbourhood;
       PointNumberList::iterator        node;
       TINEdges                         *edges;
       SegmentationParameters           segm_par;
       float                            E_old=0.01f,E,e[1],delta_E=0.0f,range=2.0;
       fstream                          fp;
       fp.open("d:\\dd.txt",ios::out);
       
       segm_par.DistanceMetricDimension()=2;
       segm_par.NumberOfNeighbours()=160;
       edges=lpts.DeriveEdges(segm_par);
       //E_old=GlobalEnergy(lpts);
      
       do{ 
       for(lpt=lpts.begin(),index=0;lpt!=lpts.end();lpt++,index++)
      {
        label=lpt->Attribute(LabelTag);
        if(label==4||label==3)
        {
          neighbourhood = lpts.Neighbourhood(PointNumber(index), range, *edges,false, true); 
          for (node=neighbourhood.begin(); node!=neighbourhood.end(); node++) 
          {
              nbs=lpts.begin()+node->Number();
              neighbour_pts.push_back(*nbs);
          }          
          neighbourhood.Erase(); 
          if(neighbour_pts.size()>0){
          //for each hyposis calculate the energy and compare to decide if change the label
          e[0]=LocalEnergy(neighbour_pts,lpt);
          if(label==4)lpt->SetAttribute(LabelTag,3);
          else lpt->SetAttribute(LabelTag,4);
          e[1]=LocalEnergy(neighbour_pts,lpt);//change the label and re-compute the energy again
          fp<<"e[0]:e[1]:"<<e[0]<<" "<<e[1]<<endl;
          if(e[0]>=e[1])
          {
            if(lpt->Attribute(LabelTag)==4)lpt->SetAttribute(LabelTag,3);
            else lpt->SetAttribute(LabelTag,4);            
            lpt->SetAttribute(v_xTag,e[0]);
            }
          else lpt->SetAttribute(v_xTag,e[1]);}
        }
      }
      E=GlobalEnergy(lpts);
      delta_E+=fabs(E_old-E);
     // E_old=E;
      count=count+1;
      cout<<count<<endl;
      cout<<"delta:"<<E_old<<" "<<E<<endl;
      }while(delta_E>0.5&&count<7);
      fp.close();
     return lpts;
     lpts.ErasePoints();
}*/
