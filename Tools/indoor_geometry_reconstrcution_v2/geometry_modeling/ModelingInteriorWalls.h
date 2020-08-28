//
// Created by NikoohematS on 20-11-2018.
//

#ifndef VISUALIZATION_TOOLS_MODELINGINTERIORWALLS_H
#define VISUALIZATION_TOOLS_MODELINGINTERIORWALLS_H

#endif //VISUALIZATION_TOOLS_MODELINGINTERIORWALLS_H


#include "LaserPoints.h"
#include "LaserPoint.h"


void Extend_Walls_Ceiling (const LaserPoints & merged_walls_lp, const Planes &walls_planes, const LaserPoints &ceiling_lp,
                           int min_segment_size, char *root, bool verbose);

void ModelingInteriorWalls (const LaserPoints &laserPoints, const Planes &planes, double max_intersection_dist,
                            int min_segment_size, char *root, bool verbose=false);

void GenerateVolumetricWalls (const ObjectPoints &wall_rectangles_vertices, LineTopologies &wall_rectangle_edges,
                              const std::map<int, double> &offset_distance_map, double fix_offset_dist,
                              char *root, bool verbose=false);

std::map<int, double> Generate_offset_dist_map (const LaserPoints &mergedSegments, double lower_wall_thickness=0.0,
                                                double upper_wall_thickness=0.0);