//
// Created by NikoohematS on 24-3-2017.
//
#include <iostream>
#include <LaserPoints.h>
#include "visualization_tools.h"

LaserPoints Project_points_to_Plane(LaserPoints lp, const Plane &plane)
{
    // generate laserpoints of projected points on the plane
    LaserPoints projected_points;
    for (int i=0; i<lp.size(); i++){
        Position3D projected_pos;
        LaserPoint projected_point;
        projected_pos = plane.Project(lp[i].Position3DRef());
        projected_point.SetX(projected_pos.GetX());
        projected_point.SetY(projected_pos.GetY());
        projected_point.SetZ(projected_pos.GetZ());
        projected_points.push_back(projected_point);
    }
    return projected_points;
}

