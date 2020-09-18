//
// Created by NikoohematS on 13-6-2018.
//

#ifndef INDOOR_GEOMETRY_REOCNSTRUCTION_SPACE_PARTITIONING_H
#define INDOOR_GEOMETRY_REOCNSTRUCTION_SPACE_PARTITIONING_H

#endif //INDOOR_GEOMETRY_REOCNSTRUCTION_SPACE_PARTITIONING_H

#include <LaserPoints.h>

LaserPoints Project_Spacepartions_To_2D(LaserPoints threeDspace_lp);

LaserPoints Intersect_Spacepartitions_Trajectory (LaserPoints spacepartitions_lp, LaserPoints trajectory,
                                                  double dist_threshold);