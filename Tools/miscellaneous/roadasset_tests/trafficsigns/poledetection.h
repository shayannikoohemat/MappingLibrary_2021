//
// Created by root on 7/15/20.
//

#ifndef TRAFFICSIGNS_POLEDETECTION_H
#define TRAFFICSIGNS_POLEDETECTION_H

#endif //TRAFFICSIGNS_POLEDETECTION_H


#include <LaserPoints.h>

LaserPoints filter_comp_bySize(const LaserPoints &connComponents, double min_segSize, double max_segSize
        , char * root, int class_label);

LaserPoints filter_comp_byHeight (const LaserPoints &connComponents, double min_height_threshold,
                                  int class_label);

LaserPoints candidate_poles (LaserPoints connComponents, double min_radius, double max_radius);

LaserPoints lp_localshape(LaserPoints& lp, SegmentationParameters& seg_parameter);