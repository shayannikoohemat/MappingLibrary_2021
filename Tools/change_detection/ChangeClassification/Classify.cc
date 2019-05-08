#include "class_par.h"

LaserPoints Classify(LaserPoints &lpts,LaserPoints &dtm_pts,fstream &features)
{
    
    LaserPoints                   sel_pts,new_pts,unsegmented_pts;
    LaserPoints::iterator         lpt,lpt_nb;
    TINEdges                      dtm_edges,edges, *edges_com;
    Planes                        planes;
    Plane                         plane, myplane;
    PointNumberList               pnl,neighbourhood;
    PointNumberList::iterator     node;
    vector<int>                   seg_nums;
    vector<int>::iterator         seg_id, neighbour_seg_id;
    double                        distance2dtm=0.0,range;
    double                        residual=0.0,dist_edge=0.0,dist_edge_sigma=0.0;
    int                           segment_num=9999, surr_segments,size,index,red,green,blue;   

    //for pre-processing of dtmpoints and above ground points
    //re-number pointnumber
    for (lpt=dtm_pts.begin(), index=0; lpt!=dtm_pts.end(); lpt++, index++)
        lpt->SetAttribute(PointNumberTag, index);   
    //drive edges
    dtm_pts.DeriveTIN();   
    dtm_edges.Derive(dtm_pts.TINReference());  
    //re-number point number
    for (lpt=lpts.begin(), index=0; lpt!=lpts.end(); lpt++, index++)
        lpt->SetAttribute(PointNumberTag, index);
    //segmentation of above ground points
    segmentation_par.NumberOfNeighbours()=40;      
 /*   segmentation_par.SeedNeighbourhoodRadius()=5;//for AHN2 1.5
    segmentation_par.MinNumberOfPointsSeed()=3;//for AHN2 8
    segmentation_par.MaxDistanceSeedPlane()=1.0;//for AHN2  0.3
    segmentation_par.GrowingRadius()=1.0;  //for AHN2 1.0  */   
      
    lpts.SurfaceGrowing(segmentation_par,true,false,&planes);
   
    //segment-based features computation
    seg_nums=lpts.AttributeValues(SegmentNumberTag);
    for(seg_id=seg_nums.begin();seg_id!=seg_nums.end();seg_id++)
       {             
         surr_segments=0;
         distance2dtm=0.0;
         residual=0.0;        
          //get the points of the segment   
         sel_pts=lpts.SelectTagValue(SegmentNumberTag, *seg_id);          
          //get the pointlist of segment
         pnl=lpts.TaggedPointNumberList(SegmentNumberTag, *seg_id);  
        // cout<<"ok"<<endl;       
         features<<","<<*seg_id<<","<<sel_pts.size()<<",";
         if(sel_pts.size()>3)
         {                  
             //pulse count
             double pulse_per=NonFirstPulsePntRatio(lpts,pnl);
             //normal              
             plane= planes[*seg_id];
             double normal=plane.Normal().Z();
             if(normal<0)normal=-1*normal;
             sel_pts.SetAttribute(PlaneNumberTag,(int)(normal*100));
             features<<normal<<",";
             //compute the point space and point-space-sigma
             dist_edge=AverageEdgeLength(sel_pts);
             dist_edge_sigma=AverageEdgeSigma(sel_pts,dist_edge);
             sel_pts.SetAttribute(ResidualTag,(float)dist_edge);//---1 dist_edge
             //sel_pts.SetAttribute(v_yTag,(float)dist_edge_sigma); // dist_edge variance
             features<<dist_edge<<","<<dist_edge_sigma<<",";
             //distance to DTM
             if(dtm_pts.size()>0)distance2dtm=DisToDTM(pnl,lpts, dtm_pts,dtm_edges,1);  
             else distance2dtm=Rank(lpts,pnl,1);                        
             //sel_pts.SetAttribute(ResidualTag,(float)distance2dtm);  //---2           
             if(distance2dtm<0.0f) distance2dtm=distance2dtm*(-1.0f);             
             features<<distance2dtm<<",";
             //plane fitting residual
             myplane=lpts.FitPlane(pnl,1);
             residual=PResidual(lpts,pnl,myplane);  
             if(residual<0)residual=-1*residual;          
             //sel_pts.SetAttribute(ResidualTag,(float)residual);  //---3
             int size=sel_pts.size();
             //sel_pts.SetAttribute(PlaneNumberTag,size); //---4
             features<<residual<<","; 
            //pre-classification
            if(distance2dtm<0.8) sel_pts.SetAttribute(LabelTag, GroundClass);//need DTM Correction using other methods 
             if(distance2dtm>dist_to_DTM && sel_pts.size()>size_plane && (dist_edge < point_space_roof/*pulse_per<=0.6*/
             ||sel_pts.size()>size_wall)) {if(plane.IsVertical(0.2))sel_pts.SetAttribute(LabelTag, BuildingWallClass);else sel_pts.SetAttribute(LabelTag, BuildingRoofClass); }             
             else 
             {        
                  if(distance2dtm>dist_to_DTM &&residual>plane_res&& dist_edge>point_space/*pulse_per>0.6*/) 
                  sel_pts.SetAttribute(LabelTag, VegetationClass);
                  else 
                  {
                       if(distance2dtm<=dist_to_DTM){if(sel_pts.size()>size_low_veg){if(residual>plane_res)
                       {if(dist_edge>point_space/*pulse_per>0.6*/)sel_pts.SetAttribute(LabelTag, LowVegetationClass);else sel_pts.SetAttribute(LabelTag, LowVegetationClass);}else sel_pts.SetAttribute(LabelTag, VehicleClass);} else sel_pts.SetAttribute(LabelTag, VehicleClass);}//add normal and NDVI to seperate low vegetation and vehicle
                       else 
                       {
                            if(distance2dtm>dist_to_DTM&&(dist_edge>point_space/*pulse_per>0.6*/||residual>plane_res||sel_pts.size()<size_vehicle)) sel_pts.SetAttribute(LabelTag, VegetationClass);//the parameters are for vegetation prefered
                            else {if(plane.IsVertical(0.05))sel_pts.SetAttribute(LabelTag, BuildingWallClass);else sel_pts.SetAttribute(LabelTag, BuildingRoofClass);}
                       }
                  }                  
             }    
        } 
        else sel_pts.SetAttribute(ResidualTag,0);           
        new_pts.insert(new_pts.end(),sel_pts.begin(),sel_pts.end());  
        features<<*(sel_pts.GetAttribute(LabelTag).begin())<<"\n";
        sel_pts.ErasePoints();    
       // pnl.Erase();       
      }  
    //for unsegmented points  
    for(lpt=lpts.begin();lpt!=lpts.end();lpt++)
    {
       if(!lpt->HasAttribute(SegmentNumberTag)) 
       {
           if(lpt->Z()<3)lpt->SetAttribute(LabelTag,VehicleClass);
           else lpt->SetAttribute(LabelTag,VegetationClass);
           new_pts.push_back(*lpt);
       }   
    }     
 
  
  /*  //for ground points
    for(lpt=dtm_pts.begin();lpt!=dtm_pts.end();lpt++)
    {
       if(lpt->Z()<0.1)lpt->SetAttribute(LabelTag, 1);
       else lpt->SetAttribute(LabelTag, 2); 
    } 
    cout<<dtm_pts.size()<<endl;*/
    
  // water and ground classification
  //for water and ground
    dtm_pts=WaterClass(dtm_pts); 
    
    //merge dtm_pts into new_pts
    new_pts.insert(new_pts.end(),dtm_pts.begin(),dtm_pts.end());
    
    seg_nums.clear();
    pnl.Erase();
    lpts.ErasePoints();
    dtm_pts.ErasePoints();    
    dtm_edges.Erase();       
    
   // new_pts=PointAttributes(new_pts);
    new_pts=CheckContext(new_pts);
  //  new_pts=Correct(new_pts);
    return new_pts;  
 }    
