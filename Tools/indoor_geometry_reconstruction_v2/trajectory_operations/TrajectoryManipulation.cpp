//
// Created by NikoohematS on 2-5-2018.
//

#include <iostream>
#include <vector>
#include "LaserPoints.h"


/*
 *  make a polyline by connecting all the points in the trajectory
 *  the trajectory should be sorted by time
 *  points can be skipped by the $point_to_skip parameter
 * */
void TrajToPolyline (LaserPoints &traj_lp, int points_to_skip, LineTopologies &traj_polyline, ObjectPoints &vertices){

    LineTopology linesegments;
    int pointnumber=0;
    for (auto &p : traj_lp){

        if( pointnumber % points_to_skip == 0){ /// skip every #points_to_skip number of points
            ObjectPoint vertex;
            vertex.X () = p.X ();
            vertex.Y () = p.Y ();
            vertex.Z () = p.Z ();
            vertex.Number () = pointnumber;

            vertices.push_back (vertex);
            linesegments.push_back (PointNumber(pointnumber));
        }

        pointnumber++;
    }
    traj_polyline.push_back (linesegments);
}

