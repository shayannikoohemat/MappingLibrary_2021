//
// Created by NikoohematS on 2-5-2018.
//

#ifndef INDOOR_GEOMETRY_REOCNSTRUCTION_TRAJECTORYMANIPULATION_H
#define INDOOR_GEOMETRY_REOCNSTRUCTION_TRAJECTORYMANIPULATION_H

#endif //INDOOR_GEOMETRY_REOCNSTRUCTION_TRAJECTORYMANIPULATION_H

#include <iostream>
#include <vector>
#include "LaserPoints.h"


void TrajToPolyline (LaserPoints &traj_lp, int points_to_skip, LineTopologies &traj_polyline, ObjectPoints &vertices);
