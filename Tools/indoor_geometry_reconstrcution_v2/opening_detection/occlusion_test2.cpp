//
// Created by NikoohematS on 30-7-2018.
//

#include <ctime>
#include <iostream>
#include <LaserPoints.h>
#include "Laservoxel.h"
#include "Buffers.h"
//#include <boost/multi_array.hpp>
#include <cassert>
#include "indoor_reconstruction.h"

enum Color { DARKBLUE = 1, DARKGREEN, DARKTEAL, DARKRED, DARKPINK, DARKYELLOW, GRAY ,
    DARKGRAY, BLUE, GREEN, TEAL, RED, PINK, YELLOW, WHITE };
void SetColor(enum Color);

bool comparison (const LaserPoint &p1, const LaserPoint &p2)
{
    return p1.DoubleAttribute(TimeTag)<p2.DoubleAttribute(TimeTag);
}

inline int dist_comparison(double a, double b, double epsilon){
    if (fabs(a-b) <= epsilon) return 1;
    else if (a > b) return 2;
    else if (a < b) return 3;
}

LaserPoint Laserpoint2ScanningPosition(LaserPoint lp, LaserPoints sorted_traj_pnts,
                                       vector<double> traj_timetags_v);

bool InsideSegmentBounds(Position3D pos, DataBoundsLaser db){
    // this is for later use in intersection with the ray
    //DataBoundsLaser db=lp.DeriveDataBounds(0);
    double seg_min_X, seg_min_Y, seg_min_Z;
    double seg_max_X, seg_max_Y, seg_max_Z;
    seg_min_X=db.Minimum().GetX();
    seg_min_Y=db.Minimum().GetY();
    seg_min_Z=db.Minimum().GetZ();

    seg_max_X=db.Maximum().GetX();
    seg_max_Y=db.Maximum().GetY();
    seg_max_Z=db.Maximum().GetZ();

    double  intersection_X,intersection_Y,intersection_Z;
    intersection_X=pos.GetX();
    intersection_Y=pos.GetY();
    intersection_Z=pos.GetZ();

    if ( seg_min_X < intersection_X && intersection_X < seg_max_X &&
         seg_min_Y < intersection_Y && intersection_Y < seg_max_Y &&
         seg_min_Z < intersection_Z && intersection_Z < seg_max_Z ){
        return true;
    }
    return false;
}

void occlusion_test2(LaserPoints laserpoints, const LaserPoints &surfacepoints,
                    LaserPoints trajpoints, double closeness_to_surface_threshold, char *root) {

    bool verbose = false;
    bool sort_trajectory = true;
    bool surface_voxels_exist = false;
    bool remove_points_behindthesurface = false;
    bool several_trajectory_segments = false;  /// always false, otherwise there is a bug /// this has a bug: use it if the trajectory is separated to several segments in the same floor
    //vector<int> given_lp_segment_num; /// optional for skipping some of the laserpoints before merge buffer process
    bool skip_floor_points = false;  /// skip floor points for occlusion test (floor points should have label 5)
    bool skip_ceiling_points = false;  /// skip ceiling points for occlusion test (ceiling points should have label 6)
    bool skip_full_segments = true;   /// full segments are surfaces without hole, when X% percent of voxels are occupied
    bool set_floor_ceiling_height = false; /// for zeb1 dataset

    std::clock_t start;
    double duration;
    start = std::clock ();

    char str_root[500];
    strcpy (str_root, root); // initialize the str_root with root string

    SetColor (BLUE);
    printf (" input laser points size:  %d \n ", laserpoints.size ());
    SetColor (GRAY); /// set color back to default console

    SetColor (BLUE);
    printf (" input surface point size:  %d \n ", surfacepoints.size ());  /// walls or wall_patches
    SetColor (GRAY); /// set color back to default console

    SetColor (BLUE);
    printf (" input trajectory points size:  %d \n ", trajpoints.size ());
    SetColor (GRAY); /// set color back to default console

    SetColor (RED);
    if (laserpoints.size () < 2) {
        printf("laser points file is empty or not readable! \n");
        exit(0);
    }

    if (surfacepoints.size () < 2) {
        printf("surfacepoints file is empty or not readable! \n");
        exit(0);
    }

    if (trajpoints.size () < 2) {
        printf("trajpoints file is empty or not readable! \n");
        exit(0);
    }
    SetColor (GRAY);

    /// sort trajectory points based on timetag
    if (sort_trajectory){
        printf("sorting trajectory ... \n");
        std::sort (trajpoints.begin(), trajpoints.end(), comparison);
    }

    /// generate a vector of sorted traj_TimeTag
    std::vector<double> trajectory_timetags_v;
    LaserPoints sorted_traj_points;
    sorted_traj_points = trajpoints;
    if(!several_trajectory_segments){
        for(int i=0; i<sorted_traj_points.size(); i++)
        {
            double current_timetag;
            current_timetag = sorted_traj_points[i].DoubleAttribute(TimeTag);
            trajectory_timetags_v.push_back(current_timetag);
        }
        strcpy(str_root, root);
        sorted_traj_points.Write(strcat(str_root, "sorted_traj.laser"), false);
    }

    /// this is for zeb1 3rd floor trajectory with two different trajectory segments
    vector<int> traj_seg_numbers;
    traj_seg_numbers = trajpoints.AttributeValues (SegmentNumberTag);
    std::vector<double> trajectory_timetags_v1, trajectory_timetags_v2;
    LaserPoints sorted_traj_points_seg1, sorted_traj_points_seg2;
/*    if(several_trajectory_segments){
        /// generate a vector of sorted traj_TimeTag
        /// for first traj_segment
        sorted_traj_points_seg1 = trajpoints.SelectTagValue (SegmentNumberTag, traj_seg_numbers[0]);
        strcpy(str_root, root); sorted_traj_points_seg1.Write(strcat(str_root, "sorted_traj_points_seg1.laser"), false); /// debug
        for(int i=0; i<sorted_traj_points_seg1.size(); i++)
        {
            double current_timetag;
            current_timetag = sorted_traj_points_seg1[i].DoubleAttribute(TimeTag);
            trajectory_timetags_v1.push_back(current_timetag);
        }

        /// debug
        printf("trajectory segment1: begin/end: %lf / %lf \n", trajectory_timetags_v1.front (), trajectory_timetags_v1.back ());


        /// for second traj_segment
        sorted_traj_points_seg2 = trajpoints.SelectTagValue (SegmentNumberTag, traj_seg_numbers[1]);
        strcpy(str_root, root); sorted_traj_points_seg2.Write(strcat(str_root, "sorted_traj_points_seg2.laser"), false); /// debug
        for(int i=0; i<sorted_traj_points_seg2.size(); i++)
        {
            double current_timetag;
            current_timetag = sorted_traj_points_seg2[i].DoubleAttribute(TimeTag);
            trajectory_timetags_v2.push_back(current_timetag);
        }

        /// debug
        printf("trajectory segment2: begin/end: %lf / %lf \n", trajectory_timetags_v2.front (), trajectory_timetags_v2.back ());
    }*/

    /* processing surfaces (or segments):
     * 1. build a voxel per surface, 2. for each surface check the occlusion with all points in the point clouds
     * */

    /// build a vector of laserpoints of surfaces
    vector<LaserPoints> surfaces_vec;
    surfaces_vec = PartitionLpByTag (surfacepoints, SegmentNumberTag);

    int                     minsizesegment;
    minsizesegment          = 2000;
    double            vox_l = 0.10;
    double floor_average_z, ceiling_average_z;
    floor_average_z = -0.82;  /// for zeb1 dataset to remove points below the floor
    ceiling_average_z = 2.04; /// for zeb1 dataset to remove points above the ceiling
    LaserPoints segments_vox_centers; /// all voxel centers of all surfaces
    LaserPoints segments_planar_voxels; /// voxel centers near to the surface plane of all surfaces
    LaserPoints segments_planar_voxels_relabeled; /// voxel centers (near to the surface plane) after occlusion relabeling
    LaserPoints points_behind_surface; /// representing points behind the surface
    int surface_count=0;

    for(auto &surface : surfaces_vec){  /// surface or segment are the same
        surface_count++;
        SetColor(TEAL);
        printf("surfaces counter: %d / %d \n", surface_count, surfaces_vec.size());
        SetColor(GRAY);

        /// we do occlusion_test for segments bigger than minsizesegment
        if(surface.size () > minsizesegment){
            int segment_nu;
            segment_nu = surface[0].SegmentNumber ();

            /// calcualte surface plane and databounds
            Plane plane;
            plane = surface.FitPlane(segment_nu);
            //plane = surfacepoints.FitPlane(*segment_it, *segment_it, SegmentNumberTag);
            //planes.push_back(plane);

            DataBoundsLaser seg_databounds;
            seg_databounds =  surface.DeriveDataBounds(0);
            /// set the databounds to the floor and ceiling if it's above the ceiling and below the floor
            if(set_floor_ceiling_height){
                /// for the floor
                //if(seg_databounds.Minimum ().GetZ () < floor_average_z)
                    seg_databounds.SetMinimumZ (floor_average_z);
                /// for the ceiling
                //if(seg_databounds.Maximum ().GetZ () > ceiling_average_z)
                    seg_databounds.SetMaximumZ (ceiling_average_z);
            }

            ///initialize the LaserVoxel
            LaserVoxel vox(surface, vox_l);
            LaserPoints seg_vox_centers;

            seg_vox_centers = vox.export_voxel_centres(2); // occupied_voxel'label=11, unoccupied=10
            seg_vox_centers.SetAttribute(SegmentNumberTag, segment_nu);
            segments_vox_centers.AddPoints (seg_vox_centers);

            /// select voxel-centers in a specific distance of the segment_plane, we don't need all voxel centers
            LaserPoints seg_planar_voxels;
            bool full_point_segment = false; /// segment has no opening or gaps
            int planar_vox_cnt =0, occupy_cnt=0;
            for(int i=0; i<seg_vox_centers.size(); i++)
            {
                double dist_plane_voxel;
                dist_plane_voxel = plane.Distance(seg_vox_centers[i]);
                if(fabs(dist_plane_voxel) < vox_l/2) // sqrt(3)*vox_l is a big threshold
                {
                    planar_vox_cnt++;
                    if(seg_vox_centers[i].Attribute (LabelTag) == 11) // occupied
                        occupy_cnt++;
                    seg_planar_voxels.push_back(seg_vox_centers[i]);
                }
            }
            segments_planar_voxels.AddPoints (seg_planar_voxels);
            /// check if the segment has gaps or opening by comparing the number of occupy voxels to the whole
            double full_segment_indicator=0.0;
            if (planar_vox_cnt != 0){
                full_segment_indicator = (double) occupy_cnt / (double) planar_vox_cnt ;
                //printf("Percentage of occupied voxels/ segment num: %.2f / %d \n", full_segment_indicator, segment_nu);
            }
            if(full_segment_indicator > 0.8) full_point_segment = true;
            /// if the segment doenst have gap or opening skip it
            if(skip_full_segments && full_point_segment) {
                /// before skipping, relabel all voxels to occupied and add it to the relabeled final voxels
                seg_planar_voxels.SetAttribute (LabelTag, 11);
                segments_planar_voxels_relabeled.AddPoints (seg_planar_voxels);
                printf ("Skipping surface #%d because of no gap or hole! \n", segment_nu);
                continue; /// goes to the next surface
            }

            /// Build KNN for voxel centers to search that intersection point is near which voxel center
            //LaserPoints relabeled_surfacelpoints; /// points that will be relabel based on opening, occlusion,..
            //relabeled_surfacelpoints = surfaces_voxel; // we relabel surface points later
            KNNFinder <LaserPoint> finder(seg_planar_voxels);

            /// Occlusion reasoning
            /*     loop through main laserpoints and find scanner position per point    */
            /* then build a ray and intersect it with the current surface (e.g. wall, vertical segments) */
            bool do_pointprocessing = true; // for debuging purposes
            if (do_pointprocessing){
                int      index1=0;
                for(auto &p : laserpoints){

/*                   /// check if the p belongs to the surface skip it
                    if (p.HasAttribute (SegmentNumberTag)){
                        /// first check if the point belong to the current surface, if yes skip it
                        if(p.SegmentNumber () == segment_nu){
                            continue; /// skip this point and continues to the the next point
                        }
*//*                        std::vector<int>::iterator it;
                        it = find(given_lp_segment_num.begin (), given_lp_segment_num.end (), p.SegmentNumber ());
                        if(it != given_lp_segment_num.end ()){
                            continue; /// segment number found in the given vector so skip it
                        }*//*
                    }*/

                    if(p.HasAttribute (LabelTag) || p.HasAttribute (PlaneNumberTag)){
                        /// if p is a floor skip it
                        if(skip_floor_points && p.PlaneNumber () == 5 || p.Label () == 5){
                            continue;
                        }

                        /// if p is a ceiling skip it
                        if(skip_ceiling_points && p.PlaneNumber () == 6 || p.Label () == 6){
                            continue;
                        }
                    }

                    if (verbose) printf("Point TIME_TAG: %lf \n ", p.DoubleAttribute(TimeTag)); // debugger
                    index1++;
                    SetColor(GREEN);
                    printf("points counter: %d / %d\r", index1, laserpoints.size());
                    SetColor(GRAY);

                    double     p_timeTag;
                    p_timeTag = p.DoubleAttribute(TimeTag);

                    /// lower_bound returns the lower bound iterator of found time_tag
                    std::vector<double>::iterator   lower_traj_it;
                    LaserPoint scanner_position;
                    if(!several_trajectory_segments){
                        lower_traj_it = std::lower_bound(trajectory_timetags_v.begin(), trajectory_timetags_v.end(), p_timeTag);
                        if(lower_traj_it != trajectory_timetags_v.end ()){ /// check the validity of lower_bound return
                            int scanner_position_index;
                            scanner_position_index = lower_traj_it - trajectory_timetags_v.begin();
                            scanner_position = sorted_traj_points[scanner_position_index];
                            //printf("POINT T: %lf , ", p.DoubleAttribute (TimeTag));                 //debug
                            //printf("TRAJ T: %lf \n", scanner_position.DoubleAttribute (TimeTag));   //debug
                        }else{
                            continue; /// go to the next point
                        }
                    }

                    /// this part has a bug, doesnt return correct result
/*                    if(several_trajectory_segments){   /// for zeb1 3rd floor trajectory
                        std::vector<double>::iterator   lower_traj_it1, lower_traj_it2 ;
                        lower_traj_it1 = std::lower_bound(trajectory_timetags_v1.begin(), trajectory_timetags_v1.end(), p_timeTag);
                        lower_traj_it2 = std::lower_bound(trajectory_timetags_v2.begin(), trajectory_timetags_v2.end(), p_timeTag);
                        if(lower_traj_it1 != trajectory_timetags_v1.end ()){ /// check the validity of lower_bound return for traj_seg1
                            int scanner_position_index;
                            scanner_position_index = lower_traj_it1 - trajectory_timetags_v1.begin();
                            scanner_position = sorted_traj_points_seg1[scanner_position_index];
                        }else if (lower_traj_it2 != trajectory_timetags_v2.end ()) { /// check the validity of lower_bound return for traj_seg2
                            int scanner_position_index;
                            scanner_position_index = lower_traj_it2 - trajectory_timetags_v2.begin();
                            scanner_position = sorted_traj_points_seg2[scanner_position_index];
                        }else{
                            continue; /// go to the next point
                        }
                    }*/

                    if (verbose){ // to check if time_tag is correct
                        double scanner_position_time;
                        scanner_position_time = scanner_position.DoubleAttribute(TimeTag);
                        printf ("Traj TIME_TAG: %lf \n ", scanner_position_time); // debugger
                    }

                    /// if current_traj is not null then calculate the ray
                    if ((scanner_position.X() && scanner_position.Y()) != 0) {
                        //matched_trajectory.push_back(*point_it); // debugger
                        /// construct a ray between the point and corresponding scanner_position
                        Line3D ray;
                        Vector3D ray_direction;
                        ray = Line3D (Position3D (scanner_position.Position3DRef ()), Position3D (p.Position3DRef ()));
                        ray_direction = ray.Direction ();

                        /// get the intersection_point of ray and surface
                        bool raysurface_sameside=0;
                        Position3D intersectionPos;
                        if (IntersectLine3DPlane(ray, plane, intersectionPos)){ // returns the intersection point
                            Line3D traj_surf_3dline;
                            traj_surf_3dline = Line3D(Position3D(scanner_position), intersectionPos);
                            /// check if intersection_point and laserpoint are at the same side of the trajectory
                            if(ray_direction.DotProduct(traj_surf_3dline.Direction()) > 0) raysurface_sameside=1;
                        }
                        if (raysurface_sameside) {

                            ///check if intersection_point falls in the boundary of the segment points
                            LaserPoint intersectionPoint = LaserPoint (intersectionPos);
                            if (InsideSegmentBounds (intersectionPos, seg_databounds)) {
                                int voxelTag, voxelIndex;
                                /// return voxelindex and tag of surfaces-voxel-center using knn
                                voxelIndex = finder.FindIndex (intersectionPoint, 1, EPS_DEFAULT);
                                voxelTag = seg_planar_voxels[voxelIndex].Attribute (LabelTag);

                                bool occupied = false,
                                     opening = false;
                                if (11 == voxelTag) occupied = true;
                                if (13 == voxelTag) opening = true;

                                /// relabel current voxel centers, dont generate new ones
                                double scanPos_lpoint_dist, scanPos_intersectionP_dist;
                                scanPos_lpoint_dist         = fabs(scanner_position.Distance(Position3D(p.Position3DRef ())));
                                scanPos_intersectionP_dist  = fabs(scanner_position.Distance(intersectionPos));
                                /// check the distance of the point_scannerposition with intersectionpoint_scannerposition
                                int dist_compare;
                                dist_compare = dist_comparison(scanPos_intersectionP_dist, scanPos_lpoint_dist , closeness_to_surface_threshold);

                                /* if dist_compare=1 means point is on the surface  // occupied
                                 * if dist_compare=2 means the point is in front of the surface // occlusion
                                 * if dist_compare=3 means the point is behind the surface // opening
                                 * */
                                switch(dist_compare){
                                    case 1: // occupied
                                        if (!opening) {
                                            seg_planar_voxels[voxelIndex].SetAttribute(LabelTag, 11); // occupied
                                            //cout << "change to occupied" << endl;
                                        }
                                        break;
                                    case 2: // occlusion
                                        if (voxelTag <= 10){ /// it means if it is not already occupied, occluded or opening then change it
                                            seg_planar_voxels[voxelIndex].SetAttribute(LabelTag, 12); // occluded
                                            //cout << "change to occluded" << endl;
                                        }
                                        break;
                                    case 3: // opening
                                        if(!occupied){
                                            seg_planar_voxels[voxelIndex].SetAttribute(LabelTag, 13); // opening
                                        }
                                        // store points behind surface and label them to modify laserpoints
                                        p.SetAttribute(LabelTag, 8); // points behind the surface
                                        /// to see each point is behind which surface
                                        p.SetAttribute (ScanNumberTag, segment_nu);
                                        points_behind_surface.push_back(p); /// problem of duplicate
                                        //points_behind_surface_indices.push_back(point_it - laserpoints.begin());
                                        break;
                                    default:
                                        cout << "no label is assigned" << endl;
                                }

                            }
                        }
                    } /// constructing the ray
                } /// end of for loop for points
            } //do_pointprocessing
            segments_planar_voxels_relabeled.AddPoints (seg_planar_voxels);
            cout << "\n" << endl;
        } // if minsizesegment
    } /// end of for loop for surfaces

    cout << "\n" << endl;

    LaserPoints relabeled_opening, relabeled_occlusion, relabeled_occupied, relabeled_unknown;
    relabeled_unknown = segments_planar_voxels_relabeled.SelectTagValue(LabelTag, 10);      // empty voxels
    relabeled_occupied = segments_planar_voxels_relabeled.SelectTagValue(LabelTag, 11);     // occupied voxels
    relabeled_occlusion = segments_planar_voxels_relabeled.SelectTagValue(LabelTag, 12);    // occluded voxels
    relabeled_opening = segments_planar_voxels_relabeled.SelectTagValue(LabelTag, 13);      // opening voxels

    strcpy(str_root, root);
    relabeled_unknown.Write(strcat(str_root,"unknown_openings_points.laser"), false), strcpy(str_root, root);
    relabeled_occupied.Write(strcat(str_root,"occupied_points.laser"), false), strcpy(str_root, root);
    relabeled_occlusion.Write(strcat(str_root,"occlusion_points.laser"), false), strcpy(str_root, root);
    relabeled_opening.Write(strcat(str_root,"opening_points.laser"), false), strcpy(str_root, root);
    segments_planar_voxels_relabeled.Write(strcat(str_root,"segments_planar_voxels_relabeled.laser"), false); /// main result

    strcpy (str_root,root);
    segments_planar_voxels.Write(strcat(str_root,"segments_planar_voxels.laser"), false);
    strcpy (str_root,root);
    segments_vox_centers.Write(strcat(str_root,"segments_voxels.laser"), false);

/*    if(!points_behind_surface.empty ()){
        printf("...RemoveDoublePoints from points behind the surface...wait \n");
        points_behind_surface.RemoveDoublePoints (); /// because per surface its repeated
    }*/

    strcpy(str_root, root);
    points_behind_surface.Write(strcat(str_root,"points_behind_surface.laser"), false);

    /// store laserpoints before removing some of the points
    strcpy(str_root, root);
    laserpoints.Write(strcat(str_root,"laserpoints.laser"), false), strcpy(str_root, root);

    /// store original wall surfaces // just for double check
    strcpy(str_root, root);
    LaserPoints surface_lp;
    surface_lp = surfacepoints;
    surface_lp.Write(strcat(str_root,"surfaces_points.laser"), false), strcpy(str_root, root);

    LaserPoints modified_points; /// points that are filtered by points behind the surface
    if(remove_points_behindthesurface){
        printf("...removing points behind the surface...wait \n");
        laserpoints.RemoveTaggedPoints(8, LabelTag);  // points behind the surface have label 8
        strcpy(str_root, root);
        laserpoints.Write(strcat(str_root,"modified_laserpoints.laser"), false), strcpy(str_root, root);
    }

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"\n Occlusion test processing time: "<< duration/60 << "m" << '\n';
}

