//
// Created by NikoohematS on 12-3-2019.
//

#ifndef INDOOR_GEOMETRY_REOCNSTRUCTION_NAVIGATIONGRAPH_H
#define INDOOR_GEOMETRY_REOCNSTRUCTION_NAVIGATIONGRAPH_H

#endif //INDOOR_GEOMETRY_REOCNSTRUCTION_NAVIGATIONGRAPH_H

#include <LaserPoints.h>

template <typename Iter>
Iter nextLocal(Iter iter)
{
    return ++iter;
}

LaserPoints Label_points_near_polygonEdges (LaserPoints &laserPoints, LineTopologies &polygons,
                                            ObjectPoints &vertices, double dist_threshold, bool planimetric);

LaserPoints Label_points_inside_polygons (LaserPoints &laserPoints, LineTopologies &polygons,
                                       ObjectPoints &vertices);

TIN Create_Constrained_Delaunay (LaserPoints &segmented_lp, int bound_segmentNumber,
                                 LineTopologies &bounds_holes_edges,  ObjectPoints &bounds_holes_vertices,
                                 ObjectPoints &holes_centers, ObjectPoints &all_points, char* root,
                                 double subsample_threhsold=0.01, bool planimetric= true);

LaserPoints Create_Triangle_Centroids (TIN &tin, ObjectPoints &vertices);

LaserPoints Create_Triangle_Medians_Intersection (TIN &tin, ObjectPoints &vertices, bool calculate_Z);

LaserPoints Create_TIN_Centroids_Graph (TIN &tin, ObjectPoints &vertices,
                                        std::pair<ObjectPoints, LineTopologies> &centroids_Graph);

void Triangulate_test(ObjectPoints &points_and_vertices, LineTopologies &edges, ObjectPoints &centers, char*root);