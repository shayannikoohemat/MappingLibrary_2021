//
// Created by NikoohematS on 23-1-2017.
//

#include <LaserPoint.h>
#include "LaserPoints.h"

/// The function gets a laserpoint and returns corresponding trajectory point,
/// NOTE: trajectory points should be sorted based on their time_tag
LaserPoint Laserpoint2ScanningPosition(LaserPoint p, LaserPoints sorted_traj_points,
                                       vector<double> trajectory_timetags_v){

    bool verbose =0;
    LaserPoint scanner_position; // this is corresponding trajectory_point
    double     p_timeTag;
    p_timeTag = p.DoubleAttribute(TimeTag);

    /// lower_bound returns the lower bound iterator of found time_tag
    std::vector<double>::iterator   lower_traj_it;
    lower_traj_it = std::lower_bound(trajectory_timetags_v.begin(), trajectory_timetags_v.end(), p_timeTag);

    if ((lower_traj_it - trajectory_timetags_v.begin())){ // check if returned iterator is not NULL

        if (verbose){
            //cout << "lower bound at position: " << (lower_traj_it - trajectory_timetags_v.begin()) << endl; // debugger
            double        timetag_matchedtrajectory; //traj_timetag
            timetag_matchedtrajectory = trajectory_timetags_v[lower_traj_it - trajectory_timetags_v.begin() -1]; //debugger
            printf("Traj TimeTag: %lf \n ", timetag_matchedtrajectory); //debugger
        }

        ///current Point of matched-trajectory;
        scanner_position = sorted_traj_points[lower_traj_it - trajectory_timetags_v.begin() -1];
        //double      traj_time_tag = scanner_position.DoubleAttribute(TimeTag); //debugger
        //printf("Current Traj TIME_TAG: %lf \n ", traj_time_tag); //debugger

        return(scanner_position);
    }
    else {
        if (verbose) cout << "value is out of bound!!! Matched scanner position is NOT found." << endl;
        EXIT_SUCCESS;
    }
}
