//
// Created by NikoohematS on 23-1-2017.
//

#include <ctime>
#include <iostream>
#include <LaserPoints.h>
#include "Laservoxel.h"
#include "Buffers.h"
//#include <boost/multi_array.hpp>
#include <cassert>

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

LaserPoints read_ascii(char *ascii_file);

static int const M = 54;
static int const N = 61;
static int const P = 16;

inline int& set_elem(vector<int>& m_, size_t i_, size_t j_, size_t k_)
{
    // you could check indexes here e.g.: assert in debug mode
    return m_[i_*N*P + j_*P + k_];
}
inline const int& get_elem(const vector<int>& m_, size_t i_, size_t j_, size_t k_)
{
    // you could check indexes here e.g.: assert in debug mode
    return m_[i_*N*P + j_*P + k_];
}

void occlusion_test(LaserPoints laserpoints, const LaserPoints &surfacepoints,
                    LaserPoints trajpoints, Buffers segment_buffers,
                    double closeness_to_surface_threshold, char *root) {

    bool verbose =0;
    bool read_traj_ascii=0;
    bool write_traj_ascii=0;
    bool sort_trajectory=1;
    bool surface_voxels_exist=0;
    bool collect_surfaces_voxels=1;
    bool remove_points_behindthesurface=1;

    std::clock_t start;
    double duration;
    start = std::clock();

    char str_root[500];
    //char *root = (char*) "D://test//indoor_reconstruction//";
    //char *root = (char*) "E://BR_data//ZebR//out//occlusion_test//";
    //char *root = (char *) "E://BR_data//Diemen//process//out//occlusion_test//";
    strcpy (str_root,root); // initialize the str_root with root string
    //char *strout;
    //strout = strcat (str_root, "FBr_3rdfloor_122k_nofloorceiling2.laser");

    /// read laser points
    //LaserPoints laserpoints;
    //laserFile = (char*) "D:\\test\\occlusion_test\\3rooms\\3rooms_raw_seg20cm_nofloorceiling_thinned1cm_167k.laser";
    //laserpoints.Read(laserFile);
    SetColor(BLUE);
    printf (" input laser points size:  %d \n ", laserpoints.size());
    SetColor(GRAY); /// set color back to default console

/*    if (!laserpoints.Read(laserFile)) {
        printf("Error reading laser points from file %s\n", laserFile);
        exit(0);
    }*/

    /// read surface points (wall)
    //LaserPoints surfacepoints;
    //surface_laserFile = (char*) "D://test//occlusion_test//3rooms//surface_1_seg30cm.laser";
    //surfacepoints.Read(surface_laserFile);
    SetColor(BLUE);
    printf (" input surface point size:  %d \n ", surfacepoints.size());
    SetColor(GRAY); /// set color back to default console

/*    if (!surfacepoints.Read(surface_laserFile)) {
        printf("Error reading surface points from file %s\n", surface_laserFile);
        exit(0);
    }*/

    /// read trajectory points (scanner positions)
    //LaserPoints trajpoints;
    //trajFile = (char*) "D://test//occlusion_test//trajectory3_crop.laser";
    //trajFile = (char*) "D://test//occlusion_test//3rooms//trajectory2.laser";
    //trajpoints.Read(trajFile);
    SetColor(BLUE);
    printf (" input trajectory points size:  %d \n ", trajpoints.size());
    SetColor(GRAY); /// set color back to default console

/*    if (!trajpoints.Read(trajFile)) {
        printf("Error reading trajectory points from file %s\n", trajFile);
        exit(0);
    }*/

    /// or read trajectory points from ascii file
    if (read_traj_ascii){
        char* ascii_file;
        //strcpy (str_root,root);
        //ascii_file = strcat(str_root, "traj_sample_notsorted.txt");
        ascii_file = (char*) "E://BR_data//Diemen//process//diemen1_s_rec_traj.txt";
        trajpoints = read_ascii(ascii_file); // order of columns "value []"  is hard coded
        std::cout << "trajpoints size: " << trajpoints.size() << endl;
    }

    /// sort trajectory points based on timetag
    if (sort_trajectory){
        std::sort (trajpoints.begin(), trajpoints.end(), comparison);
    }

    /// write sorted laserpoints to ascii again
    if (write_traj_ascii){
        /* Open the output file */
        FILE *ascii_out;
        char* asciifile_out;
        strcpy (str_root,root);
        asciifile_out = strcat(str_root,"traj_sample_sorted.txt");
        ascii_out = fopen(asciifile_out, "w");
        if (!ascii_out) {
            fprintf(stderr, "Error opening output file %s\n", asciifile_out);
            exit(0);
        }
        /// loop over all points
        LaserPoints::iterator   pointIter;
        int store_x, store_y, store_z, store_t;
        store_x=1, store_y=2, store_z=3, store_t=4;
        for (pointIter = trajpoints.begin(); pointIter != trajpoints.end(); pointIter++){
            if (store_x) fprintf(ascii_out, "%11.3f", pointIter->X());
            if (store_y) fprintf(ascii_out, " %11.3f", pointIter->Y());
            if (store_z) fprintf(ascii_out, " %7.3f", pointIter->Z());
            if (store_t) fprintf(ascii_out, " %.5f", pointIter->DoubleAttribute(TimeTag));
            fprintf(ascii_out, "\n");
        }
        fclose(ascii_out);
    }


    /// generate a vector of sorted traj_TimeTag
    std::vector<double> trajectory_timetags_v;

    LaserPoints sorted_traj_points;
    sorted_traj_points = trajpoints;
    for(int i=0; i<sorted_traj_points.size(); i++)
    {
        double current_timetag;
        current_timetag = sorted_traj_points[i].DoubleAttribute(TimeTag);
        trajectory_timetags_v.push_back(current_timetag);
    }
    strcpy(str_root, root);
    sorted_traj_points.Write(strcat(str_root, "sorted_traj.laser"), false);

    /*  construct voxel per surface segment and store all of them in one pointclouds   */
    LaserPoints surfaces_voxel;         // all voxel centers for all surfaces
    LaserPoints segments_planar_voxels; // collected voxels close (vox_len/2) to the surfaces
    /// collecting list of point_numbers and planes for each segment
    Planes                  planes;
    PointNumberLists        point_lists;
    vector<DataBoundsLaser>    segments_bounds;
    int                     minsizesegment;
    minsizesegment          = 2000;
    double            vox_l = 0.10;

    vector<int>             segment_numbers;
    vector<int>::iterator   segment_it;
    segment_numbers = surfacepoints.AttributeValues(SegmentNumberTag);  // vector of segment numbers
    printf ("Number of surfacepoints segments: %d\n", segment_numbers.size());

    /// segment processing, fitting planes, and constructing laser_voxel
    for(segment_it = segment_numbers.begin(); segment_it != segment_numbers.end(); segment_it++) {

        /// find the corresponding buffer
        if (segment_buffers.size()){
            for (int j=0; j< segment_buffers.size(); j++) {

                if (segment_buffers[j].BuffSegmentNumber() == *segment_it){
                    //LaserPoints bufflp;
                    //bufflp = segment_buffers[j].BuffLaserPoints();  // bufflp is modified in wall processing so we don't use it
                    LaserPoints segment_lpoints;
                    segment_lpoints = surfacepoints.SelectTagValue(SegmentNumberTag, *segment_it);

                    if (segment_lpoints.size() > minsizesegment) {
                        Plane plane;
                        plane = segment_buffers[j].BuffPlane();
                        planes.push_back(plane);
                        DataBoundsLaser seg_db;
                        //seg_db = segment_buffers[j].BuffBounds();
                        seg_db = segment_lpoints.DeriveDataBounds(0);
                        segments_bounds.push_back(seg_db);
                        ///initialize the LaserVoxel
                        LaserVoxel vox(segment_lpoints, vox_l);
                        LaserPoints seg_vox_centers;

                        seg_vox_centers = vox.export_voxel_centres(2); // occupied_voxel'label=11, unoccupied=10
                        seg_vox_centers.SetAttribute(SegmentNumberTag, *segment_it);

                        /// select voxel-centers in a specific distance of the segment_plane, we don't need all voxel centers
                        if (collect_surfaces_voxels){
                            LaserPoints seg_planar_voxels;
                            for(int i=0; i<seg_vox_centers.size(); i++)
                            {
                                double dist_plane_voxel;
                                dist_plane_voxel = plane.Distance(seg_vox_centers[i]);
                                if(fabs(dist_plane_voxel) < vox_l/2) // ???
                                {
                                    segments_planar_voxels.push_back(seg_vox_centers[i]);
                                }
                            }
                        }
                        surfaces_voxel.AddPoints(seg_vox_centers);
                    }  // end minsizesemgent
                }
            } // for buffers

        }

        /// selecting points by segment and saving in segment_lpoints
        if (!segment_buffers.size()){
            LaserPoints segment_lpoints;
            segment_lpoints = surfacepoints.SelectTagValue(SegmentNumberTag, *segment_it);
            Plane plane;

            if (segment_lpoints.size() > minsizesegment) {

                //plane = segment_lpoints.FitPlane(*segment_it);
                plane = surfacepoints.FitPlane(*segment_it, *segment_it, SegmentNumberTag);
                planes.push_back(plane);
                DataBoundsLaser seg_db;
                seg_db =  segment_lpoints.DeriveDataBounds(0);
                segments_bounds.push_back(seg_db);
/*            /// Collect a vector of point numbers in a PointNumberLists
            PointNumberList point_list;
            point_list = surfacepoints.TaggedPointNumberList(SegmentNumberTag, *segment_it);
            point_lists.push_back(point_list);*/

                ///initialize the LaserVoxel
                LaserVoxel vox(segment_lpoints, vox_l);
                LaserPoints seg_vox_centers;

                seg_vox_centers = vox.export_voxel_centres(2); // occupied_voxel'label=11, unoccupied=10
                seg_vox_centers.SetAttribute(SegmentNumberTag, *segment_it);

                /// select voxel-centers in a specific distance of the segment_plane, we don't need all voxel centers
                if (collect_surfaces_voxels){
                    LaserPoints seg_planar_voxels;
                    for(int i=0; i<seg_vox_centers.size(); i++)
                    {
                        double dist_plane_voxel;
                        dist_plane_voxel = plane.Distance(seg_vox_centers[i]);
                        if(fabs(dist_plane_voxel) < vox_l/2) // ???
                        {
                            segments_planar_voxels.push_back(seg_vox_centers[i]);
                        }
                    }
                }
                surfaces_voxel.AddPoints(seg_vox_centers);
            }
        }
    }

    if (collect_surfaces_voxels){
        strcpy (str_root,root);
        segments_planar_voxels.Write(strcat(str_root,"segments_planar_voxels.laser"), false);

    }

    strcpy (str_root,root);
    surfaces_voxel.Write(strcat(str_root,"surfaces_voxel.laser"), false);

    /// Occlusion reasoning
    bool do_pointprocessing=true;
    if (do_pointprocessing){
        /*     loop through main laserpoints and find scanner position per point    */
        /* then build a ray and intersect it with surfaces (e.g. wall, vertical segments) */
        LaserPoints matched_trajectory;     /// points that have matched trajectory, otherwise will be removed
        LaserPoints points_behind_surface; /// representing points behind the surface
        LaserPoints::iterator point_it, mpoint_it;
        vector <int> points_behind_surface_indices;
        bool    surface_intersected=0;
        int      index1=0;
        int     count_surface_intersected;
        count_surface_intersected = 0;

        /// remove floor and ceiling segments if needed
        vector <int> floor_ceiling_segments;// = {1, 2, 3, 5}; //{2, 4}

        for (int i=0; i<floor_ceiling_segments.size(); i++){
            laserpoints.RemoveTaggedPoints(floor_ceiling_segments[i], SegmentNumberTag);
        }
        // debug
        //laserpoints.Write("D://test//indoor_reconstruction//laserpoints_nofloorceiling.laser", false);
        /// Build KNN for voxel centers to search that intersection point is near which voxel center
        LaserPoints relabeled_surfacelpoints; /// points that will be relabel based on opening, occlusion,..
        relabeled_surfacelpoints = surfaces_voxel; // we relabel surface points later
        KNNFinder <LaserPoint> finder(relabeled_surfacelpoints);

        for(point_it = laserpoints.begin(); point_it != laserpoints.end(); point_it++){

            if (verbose) printf("Point TIME_TAG: %lf \n ", point_it -> DoubleAttribute(TimeTag)); // debugger
            index1++;
            SetColor(GREEN);
            printf("points counter: %d / %d\r", index1, laserpoints.size());
            SetColor(GRAY);

            double     p_timeTag;
            p_timeTag = point_it -> DoubleAttribute(TimeTag);

            /// lower_bound returns the lower bound iterator of found time_tag
            // TODO: check the validity of lower_bound return
            std::vector<double>::iterator   lower_traj_it;
            lower_traj_it = std::lower_bound(trajectory_timetags_v.begin(), trajectory_timetags_v.end(), p_timeTag);
            int scanner_position_index;
            scanner_position_index = lower_traj_it - trajectory_timetags_v.begin() -1;
            LaserPoint scanner_position;
            //scanner_position = Laserpoint2ScanningPosition(*point_it, trajpoints, trajectory_timetags_v); // too expensive
            scanner_position = sorted_traj_points[scanner_position_index];
            //cout << "recorded_point: " << p_timeTag << ", ";
            //cout << "scanner_position: " << scanner_position.DoubleAttribute(TimeTag) << endl;
            //double t = scanner_position.DoubleAttribute(TimeTag);
            //printf("scanner_position: %lf \n", t );

            if (verbose){ // to check if time_tag is correct
                double scanner_position_time;
                scanner_position_time = scanner_position.DoubleAttribute(TimeTag);
                printf ("Traj TIME_TAG: %lf \n ", scanner_position_time); // debugger
            }
            /// if current_traj is not null then calculate the ray
            if ((scanner_position.X() && scanner_position.Y()) != 0){
                //matched_trajectory.push_back(*point_it); // debugger
                /// construct a ray between the point and corresponding scanner_position
                Line3D          ray;
                Vector3D        ray_direction;
                ray             = Line3D(Position3D(scanner_position), Position3D(*point_it));
                ray_direction   = ray.Direction();

                /* loop through surface_planar_voxel segment by segment, and their corresponding planes
                 * surface_planar_voxel are voxel_centers that fall in a threshold distance of each segment plane
                 */
                vector<DataBoundsLaser>::iterator seg_bounds_it;
                Planes::iterator        plane_it;
                for (seg_bounds_it = segments_bounds.begin(), plane_it = planes.begin();
                     seg_bounds_it != segments_bounds.end(), plane_it != planes.end(); seg_bounds_it++, plane_it++){

                    /* if laserpoints and surfacespoints have the same segment number, then that segment
                     * which the point belongs to it will be skipped.
                     */

                    // TODO: check plane number if is equal to segment num
                    if (point_it->HasAttribute(SegmentNumberTag) && point_it->Attribute(SegmentNumberTag) == plane_it->Number()){
                        continue;
                    }

                    /// get the intersection_point of ray and surface
                    bool raysurface_sameside=0;
                    Position3D intersectionPos;
                    if (IntersectLine3DPlane(ray, *plane_it, intersectionPos)){ // returns intersection point
                        Line3D traj_surf_3dline;
                        traj_surf_3dline = Line3D(Position3D(scanner_position), intersectionPos);
                        /// check if intersection_point and laserpoint are at the same side of the trajectory
                        if(ray_direction.DotProduct(traj_surf_3dline.Direction()) > 0) raysurface_sameside=1;
                    }

                    if (raysurface_sameside){
                        count_surface_intersected++;
                        //printf ("intersected segment: %d \r ", *segment_vox_it);
                        ///check if intersection_point falls in the boundary of the segment points
                         LaserPoint intersectionPoint=LaserPoint(intersectionPos);
                        if (InsideSegmentBounds(intersectionPos, *seg_bounds_it)){
                            int             voxelTag, voxelIndex;
                            /// return voxelindex and tag of surfaces-voxel-center using knn
                            voxelIndex      = finder.FindIndex(intersectionPoint,1, EPS_DEFAULT);
                            voxelTag        = relabeled_surfacelpoints[voxelIndex].Attribute(LabelTag);

                            bool    occupied    = false,
                                    opening     = false;
                            if (11 == voxelTag) occupied = true;
                            if (13 == voxelTag) opening  = true;

                            /// relabel current voxel centers, dont generate new ones
                            double scanPos_lpoint_dist, scanPos_intersectionP_dist;
                            scanPos_lpoint_dist         = fabs(scanner_position.Distance(Position3D(*point_it)));
                            scanPos_intersectionP_dist  = fabs(scanner_position.Distance(intersectionPos));

                            int dist_compare;
                            dist_compare = dist_comparison(scanPos_intersectionP_dist, scanPos_lpoint_dist , closeness_to_surface_threshold);

                            switch(dist_compare){
                                case 1: // occupied
                                    if (!opening) {
                                        relabeled_surfacelpoints[voxelIndex].SetAttribute(LabelTag, 11); // occupied
                                        //cout << "change to occupied" << endl;
                                    }
                                    break;
                                case 2: // occlusion
                                    if (voxelTag <= 10){ /// it means if it is not already occupied, occluded or opening then change it
                                        relabeled_surfacelpoints[voxelIndex].SetAttribute(LabelTag, 12); // occluded
                                        //cout << "change to occluded" << endl;
                                    }
                                    break;
                                case 3: // opening
                                    if(!occupied){
                                        relabeled_surfacelpoints[voxelIndex].SetAttribute(LabelTag, 13); // opening
                                    }
                                    // store points behind surface and their indices for modifying laserpoints
                                    point_it->SetAttribute(LabelTag, 8); // points behind the surface
                                    points_behind_surface.push_back(*point_it);
                                    //points_behind_surface_indices.push_back(point_it - laserpoints.begin());
                                    break;
                                default:
                                    cout << "no label is assigned" << endl;
                            }
                        }
                    }
                } // end of FOR surfaces_voxel and planes loop
            }
            //printf ("%d\n", count_surface_intersected);
            count_surface_intersected=0;
        } // end of FOR laserpoints loop

        cout << "\n" << endl;
        LaserPoints relabeled_opening, relabeled_occlusion, relabeled_occupied, relabeled_unknown;
        relabeled_unknown = relabeled_surfacelpoints.SelectTagValue(LabelTag, 10);      // empty voxels
        relabeled_occupied = relabeled_surfacelpoints.SelectTagValue(LabelTag, 11);     // occupied voxels
        relabeled_occlusion = relabeled_surfacelpoints.SelectTagValue(LabelTag, 12);    // occluded voxels
        relabeled_opening = relabeled_surfacelpoints.SelectTagValue(LabelTag, 13);      // opening voxels
        //points_behind_surface.SetAttribute(LabelTag, 8); // labeling for points behind the surface


        //TODO: extract points close to the segments' plane (e.g. close to vox_len/2) from relabeled_surfacelpoints

        strcpy(str_root, root);
        relabeled_unknown.Write(strcat(str_root,"unknown_openings_points.laser"), false), strcpy(str_root, root);
        relabeled_occupied.Write(strcat(str_root,"occupied_points.laser"), false), strcpy(str_root, root);
        relabeled_occlusion.Write(strcat(str_root,"occlusion_points.laser"), false), strcpy(str_root, root);
        relabeled_opening.Write(strcat(str_root,"opening_points.laser"), false), strcpy(str_root, root);
        relabeled_surfacelpoints.Write(strcat(str_root,"relabeled_surfacelpoints.laser"), false), strcpy(str_root, root);
        points_behind_surface.Write(strcat(str_root,"points_behind_surface.laser"), false), strcpy(str_root, root);
        //matched_trajectory.Write("D://test//occlusion_test//3rdFloor_2mil_thinned_seg3_22cm_refined//result//matched_with_traj.laser", false);


/*        if (remove_points_behindthesurface){  /// too expensive
            //Sorting indices for points behind the surface
            sort (points_behind_surface_indices.begin(), points_behind_surface_indices.end());
            LaserPoints modified_laserpoints;    /// remained laserpoints, because points behind the surface will be removed
            modified_laserpoints = laserpoints;
            printf("...removing points behind the surface...wait \n");
            for(int i=points_behind_surface_indices.size() - 1; i >= 0; i--){
                modified_laserpoints.erase(modified_laserpoints.begin() + points_behind_surface_indices[i]);
            }
            modified_laserpoints.Write(strcat(str_root,"modified_laserpoints.laser"), false), strcpy(str_root, root);
        }*/

        if (remove_points_behindthesurface){
            printf("...removing points behind the surface...wait \n");
            laserpoints.RemoveTaggedPoints(8, LabelTag);  // points behind the surface have label 8
            laserpoints.Write(strcat(str_root,"modified_laserpoints.laser"), false), strcpy(str_root, root);
        }

    }

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"\n Occlusion test processing time: "<< duration/60 << "m" << '\n';
}

