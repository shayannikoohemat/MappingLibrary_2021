//
// Created by NikoohematS on 21-2-2017.
//

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <LaserPoints.h>
#include <KNNFinder.h>
#include <map>
#include <sstream>
#include "utils.h"


vector<double> GetDoubleAttribute(LaserPoints lpoints, const LaserPointTag tag) {
    /// generate a vector of double tags
    std::vector<double> tags_vec;

    for (int i = 0; i < lpoints.size(); i++) {
        double value;
        if (lpoints[i].HasAttribute(tag)) {
            value = lpoints[i].DoubleAttribute(tag);
            tags_vec.push_back(value);
        }
    }
    return tags_vec;
}


bool comparison(int i, int j) { return (i < j); }

bool compareAttribute (const LaserPoint &p1, const LaserPoint &p2)
{
    return p1.DoubleAttribute(TimeTag)<p2.DoubleAttribute(TimeTag);
}

/* This function detect reflected from glass mls points by their time difference with closest trajectory-point time.
 * If there is a time difference more than threshold between the point and closest traj point and if
 * this happens for more than percentage of points in a segment, then that segment is labeled
 * as reflected segment.
 * */

 //@param: reflectedpoints_percentage: lower than 0.70 has the risk of losing real segments

void FilterSegments_ByTimeTag(LaserPoints lp, LaserPoints trajectory_lp, double timestamp_difference,
                               double max_dist_to_traj, double reflected_points_percentage, char* output_dir) {

    std::clock_t start;
    double duration;
    start = std::clock();

    char str_root[500];
    strcpy(str_root, output_dir);

    printf (" input point size:  %d \n ", lp.size());

/*    LaserPoints floorceiling_lp;
    char *fc_laserFile;
    fc_laserFile = (char *) "D://test//filterpoints//floor_ceiling.laser";
    floorceiling_lp.Read(fc_laserFile);*/
    /// Build KNN for points for later search
    //KNNFinder <LaserPoint> finder(floorceiling_lp);

    /// Build KNN for points for later search
    KNNFinder<LaserPoint> finder_traj(trajectory_lp);

    vector<int> segment_numbers;
    vector<int>::iterator segment_it;

    segment_numbers = lp.AttributeValues(SegmentNumberTag);  // vector of segment numbers
    printf("Number of laser segments: %d\n", segment_numbers.size());
    if (segment_numbers.size() == 0) printf("Error: Points are not segmented.\n");

    //timestamp_difference =130.0;
    //reflected_points_percentage = 0.60;
    double PI;
    PI = 3.14159;
    double flat_angle, vertical_angle;
    flat_angle = 10.0;
    vertical_angle = 15.0;

/*    double z_floor, z_ceiling;
    z_floor = -0.75;
    z_ceiling = 1.60;*/

    LaserPoints modified_points, reflected_points;
    lp.SetAttribute(LabelTag, 0);
    modified_points = lp;
    for (segment_it = segment_numbers.begin(); segment_it != segment_numbers.end(); segment_it++) {
        /// selecting points by segment and saving in segment_lpoints
        LaserPoints segment_lpoints;
        segment_lpoints = lp.SelectTagValue(SegmentNumberTag, *segment_it);

        /*       vector<double>  segment_timetags;
               //segment_timetags = AttributeValuesDouble(segment_lpoints, TimeTag);  // Too slow
               segment_timetags = GetDoubleAttribute(segment_lpoints, TimeTag);
               sort(segment_timetags.begin(), segment_timetags.end(), comparison); // sort ascending
               printf ("Number of tags: %d\n", segment_timetags.size());

               double timetag_max, timetag_min;
               timetag_min = *segment_timetags.begin();
               timetag_max = *segment_timetags.end();

               DataBoundsLaser db;
               db = segment_lpoints.DeriveDataBounds(0);

               LaserPoints croped_floorceil;
               floorceiling_lp.Select(croped_floorceil, db);
               croped_floorceil.Write("D://test//filterpoints//croped_lp2D.laser", false);*/

        Plane plane;
        plane = lp.FitPlane(*segment_it, *segment_it, SegmentNumberTag);
        LaserPoints floorpoints_underwallsegment, ceilingpoints_abovewallsegment;
        /// get Vertical planes
        if (plane.IsVertical(vertical_angle * PI / 180)) { //  radian
            //printf("Number of segment'points: %d\n", segment_lpoints.size());
            LaserPoints::iterator segpoint_it;
            int reflectedpointF_count = 1;
            int reflectedpointC_count = 1;
            int reflectedpointTraj_count = 1;
            int index1 = 1;
            for (segpoint_it = segment_lpoints.begin(); segpoint_it != segment_lpoints.end(); segpoint_it++) {
                index1++;
                double segmentpoint_timetag;
                segmentpoint_timetag = segpoint_it->DoubleAttribute(TimeTag);

                /// check the segment point timetag with trajectory timetag
                int traj_index;
                double traj_dist;
                //traj_index = finder_traj.FindIndex(*segpoint_it, 1, EPS_DEFAULT);
                finder_traj.FindIndexAndDistance(*segpoint_it, traj_index, traj_dist, 1, EPS_DEFAULT);
                if (traj_dist < max_dist_to_traj){  /// this is to avoid finding wrong trajectories
                    double traj_timetag;
                    traj_timetag = trajectory_lp[traj_index].DoubleAttribute(TimeTag);

                    /// check the segment point timetag with floor and ceiling timetag
/*                LaserPoint pfloor, pceiling;
                pfloor = *segpoint_it;
                pfloor.SetZ(z_floor);

                int floorpoint_index;
                floorpoint_index =  finder.FindIndex (pfloor, 1, EPS_DEFAULT);
                double floorpoint_timetag;
                floorpoint_timetag = floorceiling_lp[floorpoint_index].DoubleAttribute(TimeTag);
                floorpoints_underwallsegment.push_back(floorceiling_lp[floorpoint_index]); // for debug

                pceiling = *segpoint_it;
                pceiling.SetZ(z_ceiling);

                int ceiling_index;
                ceiling_index = finder.FindIndex(pceiling, 1, EPS_DEFAULT);
                double ceilingpoint_timetag;
                ceilingpoint_timetag = floorceiling_lp[ceiling_index].DoubleAttribute(TimeTag);

                if (abs(segmentpoint_timetag - floorpoint_timetag) > 200){
                    reflectedpointF_count++;
                };

                if (abs(segmentpoint_timetag - ceilingpoint_timetag) > 200){
                    reflectedpointC_count++;
                }; */

                    if (abs(segmentpoint_timetag - traj_timetag) > timestamp_difference) {
                        reflectedpointTraj_count++;
                    };
                }
            }
            double reflectedpointsF_percentage, reflectedpointsC_percentage, reflectedpointTraj_percentage;
            reflectedpointsF_percentage = reflectedpointF_count / (double) index1;
            reflectedpointsC_percentage = reflectedpointC_count / (double) index1;
            reflectedpointTraj_percentage = reflectedpointTraj_count / (double) index1;
            if (reflectedpointTraj_percentage > reflected_points_percentage) {
                lp.ConditionalReTag(0, 3, LabelTag, *segment_it, SegmentNumberTag); // label 7 has the same color as 0
                modified_points.RemoveTaggedPoints(*segment_it, SegmentNumberTag);
            }
        }
    }

    reflected_points = lp.SelectTagValue(LabelTag, 3);

    strcpy(str_root, output_dir);
    lp.Write(strcat(str_root, "lp_relabeled.laser"), false);
    strcpy(str_root, output_dir);
    modified_points.Write(strcat(str_root, "lp_modified_points.laser"), false);
    strcpy(str_root, output_dir);
    reflected_points.Write(strcat(str_root, "lp_reflected_points.laser"), false);

    duration = (std::clock() - start) / (double) CLOCKS_PER_SEC;
    std::cout << "total processing time: " << duration / 60 << "m" << '\n';
}


/*
 * This function select a sub-section/partition of laserpoints between two given time_tag
 * If user needs, is possible to output the txt file of the laser points, for large datasets is expensive
 *  lpoint_v is a vector of sorted_lpoints' timetags
 * */
LaserPoints PartitionPointsByTime(LaserPoints &sorted_lpoints,vector<double> lpoints_v, double t1, double t2){

    vector<double>::iterator pFirst_it, pLast_it;
    pFirst_it = lower_bound (lpoints_v.begin (), lpoints_v.end (), t1) -1; /// lower_bound of lower trajectory time
    /*NOTE: we dont return upper_bound of second trajectory time, we return the lower_bound,
     * because we are collecting laserpoints between t1 and t2*/
    pLast_it = lower_bound (lpoints_v.begin (), lpoints_v.end (), t2); /// lower_bound of upper trajectory time

    int first_bound_inx, last_bound_inx;
    first_bound_inx = pFirst_it - lpoints_v.begin ();
    last_bound_inx = pLast_it - lpoints_v.begin ();

    printf ("Traj bounds TIME_TAG: %lf < ... < %lf  \n", t1, t2); // debugger
    printf ("LaserPoints First, Last: ... < %lf, %lf < ... \n",
            lpoints_v[first_bound_inx], lpoints_v[last_bound_inx]); // debugger

    LaserPoints::const_iterator first = sorted_lpoints.begin() + first_bound_inx;
    LaserPoints::const_iterator last = sorted_lpoints.begin() + last_bound_inx;
    LaserPoints lpoints_partition;
    lpoints_partition.insert (lpoints_partition.end (), first, last);
    //vector<T> newVec(first, last);
    printf("laser points partition size: %d \n", lpoints_partition.size ());
    return lpoints_partition;
}

/*
 * This function uses "PartitionPointsByTime" function and gets the double t1 and t2 from the
 *  sorted trajectory. The first point.time in the traj is t1 and the last point.time is t2. Then based
 *  on these two times select the points from the laserpoints
 *  "SegmentNumberTag" is hardocded but it can be replaced by AttributeTag variable
 * */

void PartitionPointsByTrajecotry_test(LaserPoints &lpoints, LaserPoints &trajectory, char* out_root, bool sort_input){

    char str_root[500];

    FILE *traj_points_file = nullptr;
    FILE *laser_points_file= nullptr;
    if(out_root){

        /// open a file for traj points
        strcpy(str_root, out_root);
        traj_points_file = fopen(strcat(str_root, "traj_points.txt"), "w");
        fprintf(traj_points_file, "X, Y, Z, Time_Tag \n");

        /// open a file for points
        strcpy(str_root, out_root);
        laser_points_file = fopen(strcat(str_root, "laser_points.txt"), "w");
        fprintf(laser_points_file, "X, Y, Z, Time_Tag \n");
    }


    /// sort the trajectory
    if(sort_input){
        printf("sorting trajectory points... wait \n");
        sort(trajectory.begin (), trajectory.end (), compareAttribute);
        strcpy(str_root, out_root);
        if(out_root) trajectory.Write (strcat(str_root, "sorted_trajectory.laser"), false);
    }


    /// sort the points
    if(sort_input){
        printf("sorting laser points... wait \n");
        sort(lpoints.begin (), lpoints.end (), compareAttribute);
        strcpy(str_root, out_root);
        if(out_root) lpoints.Write (strcat(str_root, "sorted_laserpoints.laser"), false);
    }

    vector<double> traj_time_v;
    /// write the traj.txt to the disk
    if(out_root){
        for(auto &traj_p : trajectory){
            /// make a vector of trajectories' time_tag
            // traj_time_v.push_back (traj_p.DoubleAttribute (TimeTag)); /// not used
            fprintf(traj_points_file, "%.2f,%.2f,%.2f,%lf \n",
                    traj_p.X (), traj_p.Y(), traj_p.Z (), traj_p.DoubleAttribute (TimeTag));
        }
    }

    /// make a vector of lpoints' time_tag
    vector<double> lpoints_v;
    for (auto &p : lpoints){
        lpoints_v.push_back (p.DoubleAttribute (TimeTag));

        if(out_root){
            fprintf(laser_points_file, "%.2f,%.2f,%.2f,%lf \n",
                    p.X (), p.Y(), p.Z (), p.DoubleAttribute (TimeTag));
        }
    }

    /// main function
    LaserPoints laserpoints_partition;
    LaserPoint traj1, traj2;
    traj1 = trajectory.front ();
    traj2 = trajectory.back ();
    laserpoints_partition = PartitionPointsByTime (lpoints, lpoints_v,
                            traj1.DoubleAttribute (TimeTag), traj2.DoubleAttribute (TimeTag));

    /// if is necessary write the result as txt to the disk
    if(out_root){
        FILE *lp_partition_file;
        /// open a file for traj points
        strcpy(str_root, out_root);
        lp_partition_file = fopen(strcat(str_root, "laserpoints_partition.txt"), "w");
        fprintf(lp_partition_file, "X, Y, Z, Time_Tag \n");
        fprintf(lp_partition_file, "%.2f,%.2f,%.2f,%.4f \n",
                traj1.X (), traj1.Y(), traj1.Z (), traj1.DoubleAttribute (TimeTag));
        fprintf(lp_partition_file, "%.2f,%.2f,%.2f,%.4f \n",
                traj2.X (), traj2.Y(), traj2.Z (), traj2.DoubleAttribute (TimeTag));
        for(auto &p : laserpoints_partition){
            fprintf(lp_partition_file, "%.2f,%.2f,%.2f,%.4f \n",
                    p.X (), p.Y(), p.Z (), p.DoubleAttribute (TimeTag));
        }
        fclose(lp_partition_file);

        laserpoints_partition.push_back (traj1); /// the current station

        strcpy(str_root, out_root);
        laserpoints_partition.Write(strcat(str_root, "laserpoints_partition.laser"), false);
    }

    fclose(traj_points_file);
    fclose(laser_points_file);
}


/*
 * General function for partition laserpoints by trajectory segments
 * segments are used to select the t(begin) and t(end) of each segment for the partitioning
 * segment numbers are used to store the partitions in the disk
 * the fucntion "PartitionLpByTag" does the binary search of t(begin) and t(end) in the laserpoints
 * both laser points and trajectory points should be sorted
 * */
vector<LaserPoints> PartitionPointsByTrajecotry (LaserPoints &sorted_lpoints,
                                                 LaserPoints &segmented_trajectory, char* output, bool sort_input){

    //bool sort_input = false;

    /// sort the points
    if(sort_input){
        printf("sorting laser points... wait \n");
        sort(sorted_lpoints.begin (), sorted_lpoints.end (), compareAttribute);
        string tmp_file;
        std::stringstream sstm;
        sstm << output << "sorted_points_.laser";
        tmp_file = sstm.str();
        sorted_lpoints.Write(tmp_file.c_str(), false);
        sstm.clear();
        //sorted_lpoints.Write("D:/test/laserpoints_partitioning/sorted_points_.laser", false);
    }

    /// make a vector of lpoints' time_tag
    vector<double> lpoints_v;
    for (auto &p : sorted_lpoints){
        lpoints_v.push_back (p.DoubleAttribute (TimeTag));
    }

    /// partition the trajectory to laserpoints of segments
    vector <LaserPoints> traj_segments;
    if(segmented_trajectory.HasAttribute (SegmentNumberTag)){
        traj_segments = PartitionLpByTag (segmented_trajectory, SegmentNumberTag);
    }

    vector<LaserPoints> lp_partitions_v;
    if(!traj_segments.empty ()){
        /// loop through segments of the trajectory , select the first and last point time and select laserpoints
        /// based on these times
        int segment_nr =0;
        for(auto &segment : traj_segments){
            segment_nr = segment[0].Attribute (SegmentNumberTag);
            //printf("sorting trajectory points... wait \n");
            /// sort the trajectory segment by time
            sort(segment.begin (), segment.end (), compareAttribute); // ascending sort
            double t1, t2;
            t1 = segment.front ().DoubleAttribute (TimeTag);
            t2 = segment.back ().DoubleAttribute (TimeTag);
            LaserPoints lp_partition;
            lp_partition = PartitionPointsByTime (sorted_lpoints, lpoints_v, t1, t2);
            if(lp_partition.size () > 2){
                //if(!lp_partition.HasAttribute (SegmentNumberTag2)) {
                    //segment_nr++;
                    lp_partition.SetAttribute (SegmentNumberTag, segment_nr);
                //}
                lp_partitions_v.push_back (lp_partition);
            }
        }

    } else printf("trajectory doesn't have segmentation \n");


    /* write the result to check
     * */
    /// if there is output directory, write the partitions to the disk
    if (output){
        LaserPointTag int_tag;
        int_tag = SegmentNumberTag;  // hard coded
        printf("Write partitions based on <%s> to the disk ... \n", AttributeName(int_tag, true));
        for (auto &lp : lp_partitions_v) {
            int tag_nr;
            tag_nr = 0;
            tag_nr = (lp[1].Attribute(int_tag));
            int tag2_nr;
            tag2_nr = 0;
            if(lp.HasAttribute (PlaneNumberTag)){
                tag2_nr = (lp[1].Attribute (PlaneNumberTag));
            }

            if (lp.size() > 2){

                string tmp_file;
                std::stringstream sstm;
                sstm << output << "/laserpoints_" << tag2_nr << "_partition_" << tag_nr << ".laser";
                tmp_file = sstm.str();
                lp.Write(tmp_file.c_str(), false);
                sstm.clear();
            }
        }
    }

    return lp_partitions_v;
}


/*
 *  for zeb1 trajecotry: partition laser points to levels and stairs using the segmented trajecotry
 *    NOT a COMPLETED FUNCTION
 * */
void Partition_PointsBy_Trajectory(LaserPoints &lpoints, LaserPoints &trajectory){

/*    repair the trajectory over-segmentation in each floor by average Z-value
     for Zeb1 dataset
     */
    /// collect laserpoints per segment
    vector<int> segment_numbers;
    segment_numbers = lpoints.AttributeValues(SegmentNumberTag);  // vector of segment numbers
    sort(segment_numbers.begin (), segment_numbers.end ());
    vector<LaserPoints> segments_lpoints_v;
    std::map<int, LaserPoints> segments_lpoints_map;
    std::map<int, double> segments_averagZ_map;
    for(auto &segment : segment_numbers) {
        /// selecting points by segment and saving in segment_lpoints
        LaserPoints segment_lpoints;
        segment_lpoints = lpoints.SelectTagValue (SegmentNumberTag, segment);
        segments_lpoints_v.push_back (segment_lpoints);
        segments_lpoints_map.insert(make_pair (segment, segment_lpoints));
        vector<double> z_coords;
        for(auto &p : segment_lpoints) z_coords.push_back (p.GetZ ());
        sort(z_coords.begin (), z_coords.end ());
        double min_z, max_z;
        min_z = z_coords.front ();
        max_z = z_coords.back ();
        double average_z;
        average_z = (min_z + max_z) /2;
        segments_averagZ_map.insert(make_pair (segment, average_z));
    }

    for(int i=0; i <= segment_numbers.size (); i++){

    }
}

/// point: P, Projection_point: M, mirrored_point: P` Plane: plane, Normal:n, Normal line: PP`, t is the distance P to plane
/// then the M is calculated by the distance of the P to the plane and the normal
/// line equation: PP` = r + t * v where r is Pvector and v is the direction of the normal
/// t is the distance from the P to the plane
LaserPoints mirror_PointsToPlane(LaserPoints &glass_points, LaserPoints &reflected_points){

    Plane glass_surface_plane;
    glass_surface_plane = glass_points.FitPlane (glass_points[0].SegmentNumber ());
    Vector3D normal;
    normal = glass_surface_plane.Normal ();

    LaserPoints projected_points, mirrored_points;
    for (auto &reflected_p : reflected_points){

        /// calculate the projection points on the plane
        Position3D projection;
        double     dist_point;
        dist_point = glass_surface_plane.Distance(reflected_p.Position3DRef ());

        /// PP`_line = r + t * n
        /// projected point to the plane       /// just for double check, not necessary to calculate
/*        projection.X() = reflected_p.X() - dist_point * normal.X();
        projection.Y() = reflected_p.Y() - dist_point * normal.Y();
        projection.Z() = reflected_p.Z() - dist_point * normal.Z();
        LaserPoint p;
        p.X () = projection.GetX (); p.Y () = projection.GetY (); p.Z () = projection.GetZ ();
        projected_points.push_back (p);*/

        /// mirror the reflected_point about the plane
        Position3D mirrored;
        double   dist_mirrored_point;
        dist_mirrored_point = dist_point * 2;
        /// we subtract two times the distance of the P to the plane to get the position of the mirrored point
        mirrored.X() = reflected_p.X() - dist_mirrored_point * normal.X();
        mirrored.Y() = reflected_p.Y() - dist_mirrored_point * normal.Y();
        mirrored.Z() = reflected_p.Z() - dist_mirrored_point * normal.Z();
        LaserPoint pp;
        pp.X () = mirrored.GetX (); pp.Y () = mirrored.GetY (); pp.Z () = mirrored.GetZ ();
        mirrored_points.push_back (pp);

    }

/*    char *out_root;
    char str_root[500];
    strcpy(str_root, out_root);
    projected_points.Write(strcat(str_root, "projected_points.laser"), false);
    strcpy(str_root, out_root);
    mirrored_points.Write(strcat(str_root, "mirrored_points.laser"), false);*/

    return mirrored_points;

}

/* NOTE: reflected point here are already points that because of the sensor and the glass are reflected behind
 * the glass surface and we need to mirror them back to their right position
 * */
void mirror_PointsToPlane_test_function(LaserPoints &glass_points, LaserPoints &reflected_points,
                                        LaserPoints &traj_points, char *out_root){

    LineTopologies lines;   ///for line vertice
    ObjectPoints line_vertice; /// for line segments
    // to make objectpoints of line vertice
    Covariance3D cov3d;
    cov3d = Covariance3D(0, 0, 0, 0, 0, 0);
    int next_number;

    /// sort the trajectory
    sort(traj_points.begin (), traj_points.end (), compareAttribute);

    /// make a vector of trajectories' time_tag
    vector<double> traj_time_v;
    for(auto &traj_p : traj_points){
        traj_time_v.push_back (traj_p.DoubleAttribute (TimeTag));
    }

    Plane glass_surface_plane;
    int plane_number;
    if(glass_points.HasAttribute (SegmentNumberTag)){
        plane_number = glass_points[0].SegmentNumber ();
    } else plane_number =1;
    glass_surface_plane = glass_points.FitPlane (plane_number);
    Vector3D normal;
    normal = glass_surface_plane.Normal ();
    LaserPoints mirrored_points;
    int intervals=2000; /// each n-point to visualize the reflection
    int cnt=0;
    for (auto &reflected_p : reflected_points){
        cnt++;
        /// find the scanner position for the measured point
        std::vector<double>::iterator   lower_traj_it;
        lower_traj_it = std::lower_bound(traj_time_v.begin(), traj_time_v.end(), reflected_p.DoubleAttribute (TimeTag));
        int scanner_position_index;
        scanner_position_index = lower_traj_it - traj_time_v.begin() -1;
        LaserPoint scanner_position;
        scanner_position = traj_points[scanner_position_index];

        /// reconstruct a ray from the reflected point to the trajectory
        Line3D reflection_line;
        reflection_line = Line3D(Position3D(scanner_position), Position3D(reflected_p));
/*        double incident_angle;
        incident_angle = Angle (reflection_line.Direction (), normal);
        if (incident_angle > 1.57) {
            incident_angle = incident_angle - 3.1415;
        }
        printf("incident angle: %.1f \n", incident_angle * 180 / 3.1415);*/

        /// calculate the projection points on the plane
        Position3D projection;
        double     dist_point;
        dist_point = glass_surface_plane.Distance(reflected_p.Position3DRef ());

        /// mirror the reflected_point about the plane
        Position3D mirrored;
        double   dist_mirrored_point;
        dist_mirrored_point = dist_point * 2;
        /// we subtract two times the distance of the P to the plane to get the position of the mirrored point
        mirrored.X() = reflected_p.X() - dist_mirrored_point * normal.X();
        mirrored.Y() = reflected_p.Y() - dist_mirrored_point * normal.Y();
        mirrored.Z() = reflected_p.Z() - dist_mirrored_point * normal.Z();
        LaserPoint pp;
        pp.X () = mirrored.GetX (); pp.Y () = mirrored.GetY (); pp.Z () = mirrored.GetZ ();
        mirrored_points.push_back (pp);

        /* make a line between the intersection_point and the mirrored_point */
        /// get the intersection point of the reflection_line and the plane
        Position3D intersection_p;
        if(IntersectLine3DPlane (reflection_line, glass_surface_plane, intersection_p)){
            /// make a line between the intersection_point and the mirrored_point
            Line3D intersection_mirrored_line;
            intersection_mirrored_line = Line3D(intersection_p, mirrored);
        }

        /// make a line from the reflectedpoint and  the plane normal                              /// not used
        Line3D normal_line;
        normal_line = Line3D(Position3D(reflected_p), glass_surface_plane.Normal ());

        /// visualize the lines for a few reflected points
        if(cnt % intervals == 0){

            /// print the incident angle, just for info
            double incident_angle;
            incident_angle = Angle (reflection_line.Direction (), normal);
            if (incident_angle > 1.57) {  /// if more than 90 degree
                incident_angle = incident_angle - 3.1415;
            }
            printf("incident angle: %.1f \n", incident_angle * 180 / 3.1415);

            /// check the last number of vertices and return the last number
            if (line_vertice.empty()){
                next_number = 0;
            } else {
                next_number = (line_vertice.end() - 1)->Number();  /// this is actually last number in object points
            }

            /// line segment for the reflected_line (from traj to the reflected_point)
            LineTopology incident_line_segment; /// between the reflection point and the trajectory
            LineTopology reflected_line_segment;/// between the interesection (of previous line and the plane) and the mirror point

            ObjectPoint traj_vertex, reflection_vertex, mirror_vertex, intersection_vertex;
            PointNumber pnumber;  /// for line vertices numbering

            pnumber = PointNumber(next_number +1); /// increase the number one by one
            traj_vertex = ObjectPoint(scanner_position, pnumber, cov3d); /// make the begin vertex
            line_vertice.push_back (traj_vertex);  /// push begin vertex
            incident_line_segment.push_back (pnumber);  /// push begin vertex

            pnumber = PointNumber(next_number +2);
            reflection_vertex = ObjectPoint(Position3D(reflected_p), pnumber, cov3d); /// make the end vertex
            line_vertice.push_back (reflection_vertex); /// push end vertex
            incident_line_segment.push_back (pnumber); /// push end vertex

            pnumber = PointNumber(next_number +3);
            mirror_vertex = ObjectPoint(mirrored, pnumber, cov3d); /// /// make the begin vertex
            line_vertice.push_back (mirror_vertex);
            reflected_line_segment.push_back (pnumber); /// begin vertex

            pnumber = PointNumber(next_number +4);
            intersection_vertex = ObjectPoint(intersection_p, pnumber, cov3d); /// make the end vertex
            line_vertice.push_back (intersection_vertex);
            reflected_line_segment.push_back (pnumber); /// end vertex

            incident_line_segment.SetAttribute (BuildingPartNumberTag, 1);
            reflected_line_segment.SetAttribute (BuildingPartNumberTag, 2);
            lines.push_back (incident_line_segment);

            lines.push_back (reflected_line_segment);

        }
        //cnt++;
    }

    char str_root[500];
    strcpy(str_root, out_root);
    line_vertice.Write(strcat(str_root, "line_vertice.objpts"));
    strcpy(str_root, out_root);
    lines.Write(strcat(str_root, "lines.top"), false);

}



/*  segment each trajectory based on the time gaps  */

LaserPoints Segment_Trajectory_ByTime (LaserPoints trajectory, double time_threshold){

    LaserPoints new_trajectory;

    vector <LaserPoints> traj_segments_v;
    traj_segments_v = PartitionLpByTag (trajectory, SegmentNumberTag);

    int segment_no=0;
    for (auto & segment: traj_segments_v){
        sort(segment.begin (), segment.end (), compareAttribute);

        /// initialize the first point
        LaserPoint p1;
        p1 = trajectory[0];

        /// go through all points and compare the time with previous point
        for (auto &p: segment){
            LaserPoint p2;
            p2 = p;
            p2.SetAttribute (SegmentNumberTag, segment_no);
            if(fabs((p2.DoubleAttribute (TimeTag) - p1.DoubleAttribute (TimeTag))) > time_threshold){
                segment_no++;
                p2.SetAttribute (SegmentNumberTag, segment_no);
            }
            new_trajectory.push_back (p2);
            p1 = p2;
        }
    }

    return new_trajectory;
}



