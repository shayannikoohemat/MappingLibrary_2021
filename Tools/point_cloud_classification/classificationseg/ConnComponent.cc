#include "class_par.h"

int ConnComponent(LaserPoints &lpts, PointNumberList &pnl, TINEdges * edges)
{
    int                                         label=999,com_nb,com;//found no other component labeled 4
    TINEdges::iterator                          neighbours;   
    LaserPoints::const_iterator                 nb_point;
    PointNumberList::const_iterator             point;   
    TINEdgeSet::iterator                        nb_node;
    bool                                        found_point;
    PointNumberList                             nbs;
    
    found_point=false;
    for (point=pnl.begin(), neighbours=edges->begin(); point!=pnl.end()&& !found_point;point++, neighbours++) 
    {
        com=lpts[point->Number()].Attribute(ComponentNumberTag);
        for(nb_node=neighbours->begin(); nb_node!=neighbours->end();nb_node++)
        {            
           nb_point = lpts.begin() + nb_node->Number();           
           double d =  (lpts[point->Number()] - *nb_point).Length2D();
           if(d<1.0) 
           {  
              if(nb_point->Attribute(ComponentNumberTag)>0) { 
              com_nb=nb_point->Attribute(ComponentNumberTag);               
              if(nb_point->Attribute(LabelTag)==VegetationClass && com_nb!=com)
              {
                 nbs=lpts.TaggedPointNumberList(ComponentNumberTag, com_nb);  
                 if(nbs.size()>pnl.size()) found_point=true;
                 else label=com_nb;//found a small component with label 4
              }}
           }
        }
    }
    if(found_point)label=4;
    return label;
}
