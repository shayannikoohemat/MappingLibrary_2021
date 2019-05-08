#include "class_par.h"

//for ground and water classes
LaserPoints WaterClass(LaserPoints & lpts)
{    
     
     LaserPoints sel_pts,new_pts;
     LaserPoints::iterator lpt;
     bool iswater;
     vector<int> seg_nums;
     vector<int>::iterator seg_id;
     SegmentationParameters segm_par;
     double distance=0.0;     
     
     lpts.SurfaceGrowing(segm_par,true,false);     
     
     //for unsegmented points
     for(lpt=lpts.begin();lpt!=lpts.end();lpt++)
     {
        if(!lpt->HasAttribute(SegmentNumberTag))sel_pts.push_back(*lpt);               
     }         
     if(sel_pts.size()>1)
     {
         distance=AverageEdgeLength(sel_pts);
         sel_pts.SetAttribute(ResidualTag,(float)distance); 
         new_pts.insert(new_pts.end(),sel_pts.begin(),sel_pts.end());      
         sel_pts.ErasePoints();
     }   
     //for segmented points
     seg_nums=lpts.AttributeValues(SegmentNumberTag);
     for(seg_id=seg_nums.begin();seg_id!=seg_nums.end();seg_id++)
     {
         sel_pts=lpts.SelectTagValue(SegmentNumberTag, *seg_id);
         distance=AverageEdgeLength(sel_pts);         
         sel_pts.SetAttribute(ResidualTag,(float)distance); 
         new_pts.insert(new_pts.end(),sel_pts.begin(),sel_pts.end());            
     }
      sel_pts.ErasePoints();    
           
     //classify water and ground
     for(lpt=new_pts.begin();lpt!=new_pts.end();lpt++)
     {
           if(lpt->FloatAttribute(ResidualTag)<wg_dist_threshold)lpt->SetAttribute(LabelTag,GroundClass);
           else lpt->SetAttribute(LabelTag,WaterSurfaceClass);               
     }
     
     seg_nums.clear();
     return new_pts;
     new_pts.ErasePoints();
     
}
