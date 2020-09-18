//
// Created by NikoohematS on 21-2-2019.
//

#ifndef INDOOR_GEOMETRY_REOCNSTRUCTION_CREATEADJGRAPH_H
#define INDOOR_GEOMETRY_REOCNSTRUCTION_CREATEADJGRAPH_H

#endif //INDOOR_GEOMETRY_REOCNSTRUCTION_CREATEADJGRAPH_H

#include <LaserPoints.h>

struct MinimumRectangle{
    ObjectPoints rect_corners;
    LineTopology rect_edges;
    int rect_number{}; // e.g. segment number
    double rect_area{};
    int rect_pointSize{};
};

void CreateAdjGraph (LaserPoints &laserPoints, int min_segment_size,
                        double dist_threshold, char *root, bool verbose);