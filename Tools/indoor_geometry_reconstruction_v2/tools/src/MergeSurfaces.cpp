//
// Created by NikoohematS on 5-11-2018.
//

#include <Vector3D.h>
#include "LaserPoints.h"
#include "../include/utils.h"
#include <map>

struct merged_surfaces_and_planes{
    LaserPoints surface;
    Plane plane;
};


Plane
forceVerticality(double verticality_angle_threshold, Vector3D &normal, LaserPoints &s, Plane &plane, bool verbose);

/// This function is similar to GenerateWallPatches.cpp but faster and for any arbitrary surface
/// there are three conditions for a merge to be occurred:

/* Parameters:
 * condition1: double max_angle_between_normals (in degrees): segments with normal angle less than this threshold will be merged.
 * condition2: double max_dist_between_planes: segments with planes distance less than this threshold will be merged.
 * condition3: double max_dist: segments with point distance less than this threshold will be merged.
 * double max_second_dist_between_planes: if two segments' plane are further than this threshold they are flagged for later check
 * bool calculate_middle_plane: if TRUE then when a merge happens for two faces of a wall,
 * bool calculate_weighted_plane: if TRUE then the size of segments (number of supporting points) will be considered to
 * assign a weight when calculating the plane. However, because is not tested, is not recommended to be used.
 * NOTE: either the "bool calculate_middle_plane" OR "bool calculate_weighted_plane" should be TRUE.
 * regardless of the distribution of the points, the plane between two former planes is calculated as the new plane.
 * Outputs: outputs are written to the disk: merged_segments.laser and merged_planes.planes
 * if verbose: then it generates: "projected_merged_segments.laser" where you can see the points projected to planes
 * */
void Mergesurfaces (const LaserPoints &segmented_lp, double max_dist_between_planes, double max_angle_between_normals,
                        double max_dist, int min_segment_size, char *root, double max_second_dist_between_planes,
                        bool calculate_middle_plane, bool calculate_weighted_plane,
                        bool force_verticality, double verticality_angle_threshold,
                        bool verbose){

    char str_root[500];
    strcpy (str_root, root);

    bool    merge_segments;
    double  pi = 4.0 * atan(1.0);
    double  max_angle_radian;
    max_angle_radian = max_angle_between_normals * pi / 180;

    if (!segmented_lp.HasAttribute (SegmentNumberTag)){
        printf ("Error: Laserpoints should have SegmentNumberTag! \n");
        EXIT_FAILURE;
    }
    /// first make a list of segments and sort them by size
    vector <LaserPoints> segments_vec;
    segments_vec = PartitionLpByTag (segmented_lp, SegmentNumberTag, root);

    /// sort segments by size of the segment
    sort(segments_vec.begin (), segments_vec.end (), compare_lp_size );

    /// collect segment_numbers of size-sorted segments,
    /// fit a plane to each segment using PlaneFitting function and least square
    vector<int> segment_numbers;
    vector <Plane> planes_vec;
    for(auto &s : segments_vec){
        if(s.HasAttribute (SegmentNumberTag)){
            segment_numbers.push_back (s[0].SegmentNumber ());
            /// fit the plane to the segment
            Plane plane;
            plane = s.FitPlane (s[0].SegmentNumber ());
            plane.Number () = s[0].SegmentNumber (); /// this is necessary because sometimes the plane constructor doesnt set the number
            Vector3D normal = plane.Normal();
            /// added Jul 2020 to make perfect vertical planes if user decides so
            if(force_verticality){
                Plane newPlane;
                newPlane = forceVerticality(verticality_angle_threshold, normal, s, plane, verbose);
                plane = newPlane;
            }

            planes_vec.push_back (plane);
        }
    }

    vector <int> merged_segment_numbers;
    vector <std::pair<LaserPoints, Plane> > merged_surfaces_and_planes;

    std::map <int, Plane> merged_planes_map;    /// final output
    std::map <int, LaserPoints> merged_segments_map;

    FILE *merge_info;
    strcpy (str_root,root);
    merge_info = fopen(strcat(str_root, "merge_info.txt"),"w");
    fprintf(merge_info, "seg1, seg2, merge_status, planes' dist, planes' angle, segments' dist \n");

    /// compare the segments two by two
    for (int i=0; i< segments_vec.size (); i++){
        float merge_dist;
        float merge_dist_signed = 0.0f;
        merge_dist = 0.0f;
        LaserPoints segment1;
        segment1 = segments_vec[i];
        /// we use ResidualTag for merge_distance. Later, when we make the volumetric walls, this Residual attribute
        /// is used for the thickness of the wall
        segment1.SetAttribute (ResidualTag, merge_dist);
        if (segment1.size () < min_segment_size) continue; /// if the segment is small skip it
        /// check if the segment is already merged go to the next one
        int seg_no1;
        seg_no1 = segment1[0].SegmentNumber ();
        if(find(merged_segment_numbers.begin (), merged_segment_numbers.end (),
                seg_no1) != merged_segment_numbers.end ()) continue;
        /// add the current segment to merged segments
        merged_segments_map.insert (std::pair <int, LaserPoints> (seg_no1, segment1));

        Plane plane1, original_plane1;
        plane1 = planes_vec[i]; /// if a merge happens, plane1 gets updated inside the second loop
        original_plane1 = planes_vec[i]; /// if a merge happens this remains unchanged
        /// add plane1 to the map of planes, later it will be updated if a merge happens
        merged_planes_map.insert (std::pair<int, Plane> (seg_no1, plane1));

        /* DEBUG */
/*        LaserPoints proj_segment1;
        for (auto &p : segment1){
            proj_segment1.push_back (plane1.Project (p.Position3DRef ()));
        }
        strcpy (str_root, root);
        proj_segment1.Write(strcat (str_root, "proj_segment1.laser"), false);*/
        /* END DEBUG */

        //TODO: finder is not used, can it be removed ???
        KNNFinder <LaserPoint> finder(segment1); /// to find distances of other segment to this one
        for(int j=i+1; j< segments_vec.size (); j++){
            LaserPoints segment2;
            segment2 = segments_vec[j];
            if(segment2.size () < min_segment_size) continue; /// if segment is small go to the next one
            int seg_no2;
            seg_no2 = segment2[0].SegmentNumber ();

            printf ("Checking segments: %d (%d) and %d (%d)  \r",
                    seg_no1, segment1.size (), seg_no2, segment2.size ());
            /// check if the segment is already merged go to the next one
            if(find(merged_segment_numbers.begin (), merged_segment_numbers.end (),
                    seg_no2) != merged_segment_numbers.end ()) continue;

            //if(seg_no1 == 58 && seg_no2 == 104){
            //    printf("debug \n");
            //}

            Plane plane2;
            plane2 = planes_vec[j];

            /// 1st condition: check if two normal vectors of segments are parallel
            /// Test angle between normal vectors
            merge_segments = true;
            double  angle_radian, angle_degrees;
            if (merge_segments) {
                angle_radian = Angle(plane1.Normal(), plane2.Normal()); // this can have negative values as well
                if (angle_radian > pi/2.0) angle_radian -= pi;
                if (fabs(angle_radian) > max_angle_radian) merge_segments = false;
                angle_degrees = fabs(angle_radian) * 180 / pi;
                if (verbose && !merge_segments)
                    printf("Segments %d and %d make large angle %.1f\n",
                           seg_no1, seg_no2, angle_degrees);
            } /// end of 1st condition (angle threshold)

            /// 2nd condition: check if two planes are within the threshold
            /// Test point to Plane distance
            double planes_distance = 0.0;
            double planes_distance_signed = 0.0;
            if (merge_segments){
                planes_distance_signed = plane1.Distance(plane2.CentreOfGravity());
                planes_distance = fabs(plane1.Distance(plane2.CentreOfGravity()));
                if (planes_distance > max_dist_between_planes) merge_segments = false;
                if(verbose && !merge_segments)
                    printf("plane %d and %d have large distance %.2f\n",
                           seg_no1, seg_no2, planes_distance);

                /// check if the planes distance is less than the second_distance threshold for later checking
                /// ??? here we should check as well whether the segments are approximate or not
                if (max_dist_between_planes < planes_distance && planes_distance < max_second_dist_between_planes){
                    /// we set the segment number of the first segment to the Component attribute of this one for later check
                    segment2.SetAttribute (ComponentNumberTag, seg_no1);
                }
            } /// end of 2nd condition (planes' distance)

            /// 3rd condition for merge: check if two segments are approximate
            /// check if the points of two segments are within a proximity (max_dist) of each other
            double min_dist;
            if (merge_segments){ /// if we put this in a condition then the dist just will be calculated for eligible segments, hence it is faster.

                vector <double> dists_v;
                dists_v = segment1.Distance (segment2, 1.0);
                if (!dists_v.empty ()){
                    min_dist = *min_element (dists_v.begin (), dists_v.end ());
                }
                if (fabs(min_dist) > max_dist) merge_segments = false;
                if(verbose && !merge_segments)
                    printf("Segment %d and %d are not in approximate: distance %.2f\n",
                           seg_no1, seg_no2, min_dist);
            } /// end of 3rd condition

            /// now we merged two segments and recalculate the plane for segment1 becasue all three conditions are fulfilled
            /// merging means points in segment2 get the segment number of segment1 and the plane will be recalculated
            LaserPoints segment1_old;
            Plane plane1_old;
            if (merge_segments)
            {
                /// make a copy of segment1 for calculating the weighted plane
                segment1_old = segment1;
                /// first, renumber segment2 and update the merge_dist on residualTag
                segment2.SetAttribute (SegmentNumberTag, seg_no1);
                segment2.SetAttribute (ResidualTag, merge_dist);
                /// then update the merged_segment_numbers for the next iteration
                merged_segment_numbers.push_back (seg_no2);
                /// update segment1 by adding segment2 to it
                segment1.AddPoints (segment2); /// very important to update segment1 if the merge is happening
                /// set the merge distance to segment1. This value later is used for the thickness of the wall
                if (planes_distance > merge_dist)
                {
                    merge_dist = static_cast<float>(planes_distance);
                    merge_dist_signed = static_cast<float>(planes_distance_signed);
                    /// now segment1 is segment1+segment2
                    segment1.SetAttribute (ResidualTag, merge_dist);
                }
                /// now we update segment1 in the merged_segments_map
                auto segment1_it = merged_segments_map.find(seg_no1);
                if(segment1_it != merged_segments_map.end ())
                {
                    segment1_it->second = segment1;
                }

                fprintf (merge_info, "%4d, %4d, %s, %.2f, %3.1f, %.2f \n",
                        seg_no1, seg_no2, "Yes", planes_distance, angle_degrees, min_dist);
            }

            /// calculate the plane of the new updated segment
            if (merge_segments)
            {
                Plane merged_plane;
                merged_plane = segment1.FitPlane (seg_no1); /// this will be updated if calculate_middle_plane=TRUE
                /// added Jul 2020 to make perfect vertical planes if user decides so
                if(force_verticality){
                    Plane newPlane;
                    newPlane = forceVerticality(verticality_angle_threshold, merged_plane.Normal(),
                            segment1, merged_plane, verbose);
                    merged_plane = newPlane;
                }
                if(calculate_middle_plane){ /// we use wall thickness (merge_dist) to find the middle of updated segment

                    Vector3D normal1, normal2;
                    normal1 = plane1.Normal ();
                    normal2 = plane2.Normal ();
                    /// we update the plane (or offset the plane to the middle of two former planes)
                   Vector3D middle_normal;
                    Position3D middle_pos;
/*                    if((original_plane1.Normal ().DotProduct (normal2)) < 0.0){ // changed from normal1 to original_plane1.Normal ()
                        //middle_normal = (original_plane1.Normal ().Normalize () - plane2.Normal ().Normalize ()) ;
                        //middle_pos = original_plane1.CentreOfGravity () - merge_dist/2 * original_plane1.Normal ();
                    } else {
                        //middle_normal = (original_plane1.Normal ().Normalize () + plane2.Normal ().Normalize ()) ;
                       // middle_pos = original_plane1.CentreOfGravity () + merge_dist/2 * original_plane1.Normal ();
                    }*/
                    middle_pos = original_plane1.CentreOfGravity () + merge_dist_signed/2 * original_plane1.Normal ();
                    middle_normal = normal1; // we keep using the normal from lpane1 as the dominant segment
                    /*NOTE:
                     * we can not use CenterOfGravity of plane1 becasue when it is updated the associated points are zero
                     * so the CenterofGravity is NAN */
                /* if we use plane1 to calculate the dist-to-origin, because everytime plane1
                 * is updated the plane will be wrongly shifted*/
                   //double plane1_dist_to_origin, plane2_dist_to_origin;
                   //plane1_dist_to_origin = plane1.Distance (Position3D(0,0,0));
                   //plane2_dist_to_origin = plane2.Distance (Position3D(0,0,0));
                   //double merged_plane_dist;
                   //merged_plane_dist = (fabs(plane1_dist_to_origin) + fabs(plane2_dist_to_origin)) /2;

                   /// the plane should be calculated each time based on the final thickness of the segment which is merge_dist
                   //middle_pos = (plane1_old.CentreOfGravity () + plane2.CentreOfGravity ()) /2;
/*                    if (planes_distance > merge_dist)
                    {
                        middle_pos = (original_plane1.CentreOfGravity () + plane2.CentreOfGravity ()) /2;
                    }*/

/*                    LaserPoints middle_point;
                    middle_point.push_back (middle_pos);
                    strcpy (str_root, root);
                    middle_point.Write(strcat (str_root, "middle_point.laser"), false);*/

                   Plane plane(middle_pos, middle_normal);
                   merged_plane = plane;
                    //double plane_dist;
                    //plane_dist = fabs(original_plane1.Distance (Position3D(0,0,0))) + merge_dist/2;
                    //merged_plane.SetDistance (plane_dist);
                   // merged_plane.SetNormal (middle_normal);
                  // merged_plane.CentreOfGravity () = middle_pos; ///
                   //merged_plane.SetDistance (merged_plane_dist); /// instead of middle_pos we use setDistance
                }
                /// very important to update the plane number
                merged_plane.Number () = seg_no1; /// make sure it gets the segment number

                /// this is not tested, so is not recommended
                if (calculate_weighted_plane){ /// this is not tested for all cases, plot the projected point to make sure the planes are correct.
                    /// we update the plane considering the size of each plane
                    Vector3D weighted_normal;
                    weighted_normal = ((segment1_old.size () * plane1.Normal () + segment2.size () * plane2.Normal ()) /
                                  (segment1_old.size () + segment2.size ())).Normalize ();
                    PointNumberList pointNumberList;
                    /// segment1 is the merged-segment (segment1+segment2)
                    pointNumberList = segment1.SelectTagValueList (SegmentNumberTag, seg_no1);
                    /// we use the weighted_pos to update the distance-to-origin of the merged_plane
                    Position3D weighted_pos;
                    weighted_pos = segment1.CentreOfGravity (pointNumberList);
                    merged_plane.SetNormal (weighted_normal);
                    merged_plane.CentreOfGravity () = weighted_pos;
                }
                /// after updating the merged_plane in either of two previous IF conditions then we update plane1.
                plane1 = merged_plane;
                /// update plane1 in the map of planes
                auto plane1_it = merged_planes_map.find (seg_no1);
                if ( plane1_it != merged_planes_map.end ()){
                    plane1_it->second = merged_plane;
                }

                /* check if plane1 and segment1 are updated */
                /// DEBUG
/*                LaserPoints projected_merged_segment;
                for (auto &p : segment1){
                    projected_merged_segment.push_back (plane1.Project (p.Position3DRef ()));
                }
                strcpy (str_root, root);
                projected_merged_segment.Write(strcat (str_root, "projected_merged_segment.laser"), false);
                strcpy (str_root, root);
                segment1.Write(strcat (str_root, "segment1.laser"), false);
                strcpy (str_root, root);
                segment2.Write(strcat (str_root, "segment2.laser"), false);*/
                /// END of DEBUG
            }
            if(!merge_segments){
                fprintf (merge_info, "%4d, %4d, %s, %.2f, %3.1f, %.2f \n",
                         seg_no1, seg_no2, "No ", planes_distance, angle_degrees, min_dist);
            }
        } /// end of second for (segment2)
    } /// end of first for (segment1)

    LaserPoints merged_segments;   /// final output
    LaserPoints projected_merged_segments; /// debug output
    /// collect merged_segments
    for (auto &seg_m : merged_segments_map){
        merged_segments.AddPoints (seg_m.second);

        /// output projected points on the plane for double check
        //if (verbose){
            Plane plane;
            auto plane_it = merged_planes_map.find (seg_m.first);
            if ( plane_it != merged_planes_map.end ()){
                plane = plane_it->second;
                for (auto &p : seg_m.second){
                    projected_merged_segments.push_back (plane.Project (p.Position3DRef ()));
                    /// this below part is not necessary because it recalculates planes
                    //plane.AddPoint (plane.Project (p.Position3DRef ()));
                }
                /* this is for making other parametrs of the plane */
                //plane.Recalculate (); // we shouldn't recalculate the plane as it may forced to be vertical
                //plane_it->second = plane;
            }
        //}
    }
    strcpy (str_root, root);
    merged_segments.Write(strcat (str_root, "merged_segments.laser"), false);

    ///collect merged planes
    Planes merged_planes;   /// final output
    for (auto &m : merged_planes_map){
        merged_planes.push_back (m.second);
    }

    /// write planes to the disk for later use
    strcpy (str_root, root);
    merged_planes.Write(strcat (str_root, "merged_planes.planes"));

    //if(verbose){
        strcpy (str_root, root);
        projected_merged_segments.Write(strcat (str_root, "projected_merged_segments.laser"), false);
    //}

    fclose(merge_info);

    std::cout << "NOTE: The calculated merge distance is saved in Residual Tag per merged segment. "  << std::endl;

/// renumber merged_segments if necessary
 // there is a separate function for this
}

Plane
forceVerticality(double verticality_angle_threshold, Vector3D &normal, LaserPoints &s, Plane &plane, bool verbose) {

    char str_root[500];
    auto *root = (char*) "/home/shayan/Drive_D/data/test/out/";
    strcpy (str_root, root);
    if(verbose){
        Vector3D vecZ = Vector3D(0,0,1);
        double segment_angle = Angle (normal, vecZ) ; /// this gives the rotation angle respecting Z-axis
        printf ("segment number and angle: %d and %.1f\n",
                s[0].SegmentNumber (), segment_angle * 180.0 / PI);
    }
    /// if plane is almostVertical make it perfect vertical
    Plane vertPlane = plane; // this associate the plane points to vertPlane
    if(plane.IsVertical(verticality_angle_threshold * PI / 180)){ // ToRadian
        //Plane vertPlane (s.Mean(), Vector3D(normal.X(), normal.Y(), 0.0)); // this causes the plane loses the associated points from the segment
        // NOTE: just overriding the plane.Normal() with a new normal doesn't work, it should be recalculated like below
        Vector3D newNormal (normal.X(), normal.Y(), 0.0); // normal.Z=0 makes the plane vertical
        double len=newNormal.SqLength();
        if( len==0.0 )
        {
            vertPlane.Normal()=Vector3D( 0.0, 0.0, 1.0 );
            vertPlane.Distance() = s.Mean().Z();
        }
        vertPlane.Normal()   = newNormal/sqrt(len);                 // set the new normal
        vertPlane.Distance() = (newNormal/sqrt(len)).DotProduct( s.Mean() );   // set the dist from origin
        vertPlane.Number () = s[0].SegmentNumber ();            // set plane number
      /// DEBUG /// export the projected points on the new plane
        LaserPoints projected_segment;
        for (auto &p : s) /// for each point in segment
            projected_segment.push_back (vertPlane.Project (p.Position3DRef ()));
        strcpy (str_root, root);
        projected_segment.Write(strcat (str_root, "proj_segment1.laser"), false);
        /// End of DEBUG
    }
    return vertPlane;
}

