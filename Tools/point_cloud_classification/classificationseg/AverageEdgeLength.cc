#include "class_par.h"

double AverageEdgeLength(LaserPoints &pts)
{
       TINEdges::iterator          neighbours;
       LaserPoints::const_iterator point, nb_point;
       TINEdgeSet::iterator        nb_node;
       double                       d,dist,distance=0.0;
       int                         neighbour_pts;
       TINEdges                    *edges;
       SegmentationParameters             segm_par;
       
      // segm_par.DistanceMetricDimension() = 2;
     //  segm_par.NumberOfNeighbours() =40;
       edges=pts.DeriveEdges(segm_par);       
      
       //calculating the point space
       for (point=pts.begin(), neighbours=edges->begin(); point!=pts.end();point++, neighbours++) 
       {      
            neighbour_pts=0;  
            dist=d=0.0;    
            for (nb_node=neighbours->begin(); nb_node!=neighbours->end(); nb_node++) 
            {              
                nb_point = pts.begin() + nb_node->Number(); 
                d =  (*point - *nb_point).Length2D();     
                if(d<dist_threshold){neighbour_pts++; dist= dist+d; }                                
            }
           //if there are less than 5 qualified neighbour points the point density is quite high
            if(neighbour_pts<=5)distance=distance+1.0;
            else distance=distance+dist/neighbour_pts;          
       }
       distance=distance/pts.size();        
       return distance;
        
}

//test in 3D situation??no
double AverageEdgeSigma(LaserPoints &pts,double distance)
{
       TINEdges::iterator          neighbours;
       LaserPoints::const_iterator point, nb_point;
       TINEdgeSet::iterator        nb_node;
       double                       d,dist,sigma=0.0;
       int                         neighbour_pts;
       TINEdges                    *edges_sigma;
       SegmentationParameters      segm_par;
       
     //  segm_par.DistanceMetricDimension() = 2;
     //  segm_par.NumberOfNeighbours() =40;
       edges_sigma=pts.DeriveEdges(segm_par);
       
       //compute sigma of point space
       for (point=pts.begin(), neighbours=edges_sigma->begin(); point!=pts.end();point++, neighbours++) 
       {      
            neighbour_pts=0;  
            dist=d=0.0;    
            for (nb_node=neighbours->begin(); nb_node!=neighbours->end(); nb_node++) 
            {              
                nb_point = pts.begin() + nb_node->Number(); 
                d =  (*point - *nb_point).Length2D();     
                if(d<dist_threshold){neighbour_pts++; dist= dist+d; }  
                //for 2008 d<1.0
                //for AHN2 d<1.5                 
            }          
            if(neighbour_pts<=5)sigma=sigma+1.0;
            else sigma=sigma+(dist/neighbour_pts-distance)*(dist/neighbour_pts-distance);
           
       }
       sigma=sqrt(sigma/(pts.size()-1));    
       return sigma;
        
}
