//
// Created by NikoohematS on 11/4/2016.
//

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <LaserPoints.h>
#include <map>
#include "../utils/utils.h"
#include "../utils/Buffer.h"
#include "../utils/Buffers.h"
#include "../utils/post_processing.h"

/// if windows set color console
enum Color { DARKBLUE = 1, DARKGREEN, DARKTEAL, DARKRED, DARKPINK, DARKYELLOW, GRAY ,
                DARKGRAY, BLUE, GREEN, TEAL, RED, PINK, YELLOW, WHITE };
void SetColor(enum Color);


//void occlusion_test(LaserPoints laserpoints, LaserPoints surfacepoints,
//                    LaserPoints trajpoints, Buffers segment_buffers, double closeness_to_surface_threshold);

/*
 * Hint: to test parameters, and see whether passing true for some bool operations (e.g., generalize walls), we suggest after
 * surface growing segmentation, downsample point clouds to less than 1 million points and run the algorithm
 * with different settings to make sure the results are acceptable. Then you can run it on a denser point clouds with the
 * same arguments.
 * Hint 2: always make an output directory and pass it to char* root_dir, so you dont mix the output files with input.
 * important parameters:
 * flat_angle=10.0 , vertical_angle=10.0: are the angle between the normal and positive direction of z-axis
 *  for separation of vertical and horizontal candidates
 * min_intersection_linelength = 0.10: bigger numbers mean more segments are excluded to be a wall,
 * if min_intersection_linelength = 0.20;  to exclude doors that partially are connected to the ceiling a 
 * line_length more than 10cm is better
 * if bool generate_wallpatches=true the result is "buffer_segmentation.laser" in the root_dir
 * sort_segments_by_number operation is expensive, do it if you want to read output info files
 * check almost_horizontal_segments_infofile.txt" and "almost_horizon_subset_infofile.txt" 
 * and "almost_horizon_segs_10%subset.laser" for best estimation of floor and ceiling height, then use this estimation 
 * as input of the function. The heighest ceiling and lowest floor should 
 *  be used as estimated input argument. Note if the height of floor to the ceiling is less than 2 meters, the program aborts.
 * ceiling_buffer=1.0, floor_buffer=0.5: for selection of candidate floor and ceilings near the estimated height
 * *** important: if the ceiling has a big hall and very different heights increase the ceiling_threshold ****
 * Check "candidate_ceil_floor_segments_byheight.laser" in root_dir to see if candidate floor and ceiling are correct,
 * if not, the thresholds and buffers for ceiling and floor should be changed accordingly to exclude or include segments
 * *** if the ceiling_buffer is relaxed (set to very high number) then better "bool apply_intersection_result =true" ***
 * max_intersection_dist=0.10 : for checking if two segments have intersection in the dist of faces of two segments
 * hardcoded parameters: ceiling_z, floor_z depends on the data, hardcoded if the estimation is wrong
 * important flags: apply_intersection_result=true: use intersection result for the floor and ceiling candidate
 *   * apply_intersection_result loops through almost horizontal segments and check if their oriented bboxes overlap,
 *   * if yes then check their
     * minimum bounding rectangles (MBR) intersect/overlap or not. For safety of intersection on adjacent segments
     * we resize the MBR to a smaller size of 0.98 of original size.
     * If intersection checking is positive then the lower segments
     * would be excluded from the list of candidate ceilings.
     * NOTE: there is no check if the minimum bounding rectangle is not null.
     * Function post_processing.cpp > filter_ceil_by_intersection:
     * important parameters: intersection_percentage=0.5, percentage of overlap for validation of the overlap between
     * two ceiling candidates. If the overlap is valid it means one of the polygons should be discarded as a valid ceiling
     * 
 * Check "graphinfo.txt" and "graphinfo2.txt" and "statWalls.txt" for the adjacency graph and topology of segments
 * * if bool wallwall_vertical_dist_condition=true then the function applies
 * wallwall_vertical_dist=0.20m : for separation of two vertical walls (one above the other) with different normal angles
 * Check "intersection_lines.objpts" and "intersection_lines.top" by importing in pcm and see if the intersections are correct.
 * Check "pendingwalls.laser" whether it can be a wall because of closeness and almost parallel plane to current wall
 *  if crop_walls_abovefloor_and_belowceiling= true the program crops wall points above the floor
 *  and below the ceiling. However, this is not necessary, unless we want to have a rectangle wall shape for occlusion test
 *  in that case set the values for ceiling_z_average and floor_z_average
 *
 *  Labels:
 *          no label                    =0,  white, dark pink
            almost horizontal           =1,  green
            almost vertical             =2,  blue
            label wall-slanted-wall     =3,  sharp yellow
            wall                        = 4, orange
            floor                       = 5, light yellow
            ceiling=                    = 6
            = 8 // wall points above the ceiling or lower than floorportant output:
            relabeled_walls =44 // pending walls

    Check the segments_pairpoints: "pairlines.objpts" and segments_pairlines: "pairlines.top" for the adjacency graph
     "unknown_points.laser"
     "floor.laser"
     "ceiling.laser"
    "walls.laser"
    "outputlaser.laser" all together
    "small_segments.laser"  segments smaller than minsegmentsize
    "relabeled_walls.laser"  or pending_walls are segments that can be wall because of closeness
    and almost parallel plane to current wall
    "notsegmented_points.laser"
    laserpoints.laser -> the original input file
    * unknown_points and ouput_points should contain unlabeled points, unlabeled horizontal and unlabeled vertical
 * */

void
indoorTopology(char *laserFile, char *root_dir, int minSegmentSize, bool verbose, double flat_angle,
               double vertical_angle, double max_intersection_dist, double min_intersection_linelength,
               bool sort_segments_by_number, bool user_defined_ceil_floor_height, double ceiling_z_userdefined,
               double floor_z_userdefined, double ceiling_buffer, double floor_buffer,
               double intersection_percentage, bool apply_intersection_result, bool wallwall_vertical_dist_condition,
               double wallwall_vertical_dist, bool crop_walls_abovefloor_and_belowceiling,
               double ceiling_z_average, double floor_z_average,
               bool generate_wallpatches, double Generate_WallPatches_dist_threshold,
               double Generate_WallPatches_angle_threshold, double Generate_WallPatches_dist_along_twosegments) {

    std::clock_t start;
    double duration;
    start = std::clock();

    char str_root[500];
    strcpy (str_root, root_dir); // initialize the str_root with root string

    /// read laser points
    LaserPoints laserpoints;
    laserpoints.Read(laserFile);
//    SetColor(YELLOW);
    printf (" input point size:  %d \n ", laserpoints.size());
//    SetColor(GRAY); /// set color back to default console

    if (!laserpoints.Read(laserFile)) {
        printf("Error reading laser points from file %s\n", laserFile);
        exit(0);
    }

    /// collect points without segment nr
    LaserPoints notsegmented_points;
    //if (verbose){
        LaserPoints::iterator point_it;
        for (point_it = laserpoints.begin(); point_it != laserpoints.end(); point_it++){
            if (!point_it->HasAttribute(SegmentNumberTag)){
                notsegmented_points.push_back(*point_it);
            }
        }
    //}


    /// default label ==0
    laserpoints.SetAttribute(LabelTag, 0);
    /// collecting list of point_numbers and planes for each segment
    Planes                  planes;
    PointNumberLists        point_lists;
    double                  PI=3.14159;
    LaserPoints             small_segments;
    Buffers                 segment_buffers;

    /*
 *  *** Important function for generating bigger segments as wall patch that is called buffer ***
     *  // time consuming operation, is not recommended for large files or many segments
     *  // generate wallpatches includes segment refinement
 * */
    if(generate_wallpatches){
        printf ("\n Generate wall-patch segments ... (Martin-Kada's generalisation approach) \n");
        //Generate_WallPatches_dist_threshold = 0.70, Generate_WallPatches_angle_threshold =20.0, Generate_WallPatches_dist_along_twosegments =0.40;
        segment_buffers = GenerateWallPatches(laserpoints, Generate_WallPatches_dist_threshold,
                                              Generate_WallPatches_angle_threshold,
                                              Generate_WallPatches_dist_along_twosegments, root_dir, verbose);

        LaserPoints buffer_points;
        buffer_points = segment_buffers.Buffers_laserpoints();
        buffer_points.SetAttribute(LabelTag, 0);
        laserpoints = buffer_points; // this initializes pointnumbers (because of pointnumbers bug)

        LaserPoints relabeled_buffer_points;  // relabeled to vertical & horizontal
        for (int i=0; i< segment_buffers.size(); i++){
            LaserPoints bufflp;
            bufflp = segment_buffers[i].BuffLaserPoints();
            //bufflp[i].GetPointNumber(); // bug

            if (bufflp.size() > minSegmentSize){
                Plane plane;
                plane = segment_buffers[i].BuffPlane();

                int segment_nr;
                segment_nr = segment_buffers[i].BuffSegmentNumber();

                /// label laserpoints with horizontal planes as 1
                if (plane.IsHorizontal(flat_angle * PI / 180)){ // radian
                    bufflp.ConditionalReTag(0, 1, LabelTag, segment_nr , SegmentNumberTag); // Horizontal 1
                }

                /// label laserpoints with vertical planes as 2
                if (plane.IsVertical(vertical_angle * PI / 180)) { //  radian
                    bufflp.ConditionalReTag(0, 2, LabelTag, segment_nr, SegmentNumberTag); // Vertical 2
                }

                planes.push_back(plane); // this plane belongs to the buffer and is NOT calculated from bufflaserpoints

                PointNumberList point_list;
                /// Collect a vector of point numbers with SegmentNumberTag value
                point_list = laserpoints.TaggedPointNumberList(SegmentNumberTag, segment_nr);
                //point_list.size(); //debug
                point_lists.push_back(point_list);
            } else{
                small_segments.AddPoints(bufflp);
            }

            relabeled_buffer_points.AddPoints(bufflp);
        }
        strcpy (str_root, root_dir);
        relabeled_buffer_points.Write(strcat(str_root,"buffer_segmentation.laser"), false);
        laserpoints = relabeled_buffer_points; // contains all buffers after renumbering their segmentation
    }

    FILE *segments_infofile;
    strcpy (str_root, root_dir);
    segments_infofile = fopen(strcat(str_root, "segments_infofile.txt"),"w");
    fprintf(segments_infofile, "segment_nr, segment_pointsize, centroid_height, segment_angle \n");

    /// this file is to analysis almost_horizontal segments to estimate floor and ceiling height
    FILE *almost_horizon_infofile;
    strcpy (str_root, root_dir);
    almost_horizon_infofile = fopen(strcat(str_root, "almost_horizontal_segments_infofile.txt"),"w");
    fprintf(almost_horizon_infofile, "segment_nr, segment_pointsize, centroid_height, segment_angle \n");

    /*
     * * if we didn't calculate wallpatches then we use original segments for wall detection
    * */
    std::map<int, int>      horizon_segment_sizep_map;
    std::map<int, double>   horizon_segments_centroidheight_map;
    vector<int>             segment_numbers;
    vector<LaserPoints>     horizon_segments_vec;
    if (!generate_wallpatches){
        vector<LaserPoints> segments_lp_vec;
        printf("Partitioning laserpoints by SegmentNumberTag...wait \n");
        segments_lp_vec = PartitionLpByTag (laserpoints, SegmentNumberTag, root_dir);
//        SetColor(YELLOW);
        printf("Number of segment partitions: %d \n", segments_lp_vec.size ());
//        SetColor(GRAY);

        /// sort segments by segment number
        if(sort_segments_by_number) {
            printf("Sorting segments by SegmentNumberTag...wait \n");
            sort(segments_lp_vec.begin (), segments_lp_vec.end (), compare_lp_segmenttag);  // too expensive
        }

        //segment_numbers = laserpoints.AttributeValues(SegmentNumberTag);  // vector of segment numbers
        //std::sort(segment_numbers.begin(), segment_numbers.end());
        //printf ("Number of laser segments: %d\n", segment_numbers.size());

        /// segment processing and fitting planes and labeling to =1 (horiz) and =2 (vertical)
        printf("Collecting segments' information and classify to vertical and horizonal...wait \n");
        int count=0;
        for(auto &segment_lpoints : segments_lp_vec){
            count ++;
            /// selecting points by segment and saving in segment_lpoints
            Plane   plane;
            if(segment_lpoints.size() > minSegmentSize) {
                int segment_number;
                segment_number = segment_lpoints[0].SegmentNumber ();

                plane = segment_lpoints.FitPlane (segment_number);
                plane.Number () = segment_number;  /// set the segment number as the plane number

                PointNumberList point_list;
                /// Collect a vector of point numbers with SegmentNumberTag value, used for polygon
                point_list = laserpoints.TaggedPointNumberList (SegmentNumberTag, segment_number);
                point_lists.push_back (point_list);

                /* get the distribution of segment's size, plane's angle and segment's center height
                 this information is useful for extraction of the ceiling and floor height candidate
                 and getting the angle distribution of the segments */
                double segment_angle=0.0;
                ObjectPoint segment_centroid;
                //if (verbose) {
                segment_angle = Angle (plane.Normal (), Vector3D (0, 0, 1)) * 180.0 / PI;
                if (segment_angle > 90.0) segment_angle = 180.0 - segment_angle;
                //}
                segment_centroid = laserpoints.Centroid (point_list, segment_number);
                fprintf (segments_infofile, "%d,%d,%.2f,%.2f \n",
                         segment_number, segment_lpoints.size (), segment_centroid.Z (), segment_angle);

                /// label laserpoints with horizontal planes as 1
                if (plane.IsHorizontal(flat_angle * PI / 180)) { //convert to radian
                    laserpoints.ConditionalReTag (0, 1, LabelTag, segment_number, SegmentNumberTag); // Horizontal 1
                    /// make an info file of almost horizontal segments
                    fprintf(almost_horizon_infofile, "%d,%d,%.2f,%.2f \n",
                            segment_number, segment_lpoints.size(), segment_centroid.Z (), segment_angle);
                    horizon_segments_centroidheight_map.insert ( std::pair<int, double>(segment_number, segment_centroid.Z ()));
                    horizon_segment_sizep_map.insert(std::pair<int, int> (segment_number, segment_lpoints.size ()));
                    horizon_segments_vec.push_back (segment_lpoints);
                }

                /// label laserpoints with vertical planes as 2
                if (plane.IsVertical(vertical_angle * PI / 180)) //  radian
                    laserpoints.ConditionalReTag(0, 2, LabelTag, segment_number, SegmentNumberTag); // Vertical 2

                planes.push_back(plane);

            } else {
                small_segments.AddPoints(segment_lpoints);
            }
        } /// end of segment plane fitting
    }

    LaserPoints relabeled_vert_horiz;
    relabeled_vert_horiz = laserpoints;
    strcpy (str_root, root_dir);
    relabeled_vert_horiz.Write(strcat(str_root, "relabeled_vert_horiz.laser"), false);

    LaserPoints almost_horizontal_segments;
    almost_horizontal_segments = relabeled_vert_horiz.SelectTagValue(LabelTag, 1);
    strcpy (str_root, root_dir);
    almost_horizontal_segments.Write(strcat(str_root, 
            "almost_horizontal_segments.laser"), false);                                          // debug

    /* estimate the ceiling and floor height by choosing dominant almost-horizontal segments
     *  and check their height distribution *** this is just a rough estimation
     * */
    /// sort vector of horizontal_segments by size
    sort(horizon_segments_vec.begin (), horizon_segments_vec.end (), compare_lp_size); // descending
    /// select 10% of big horizontal segments to check their height distribution
    int count_subset_horizon_segments; /// count of 10% to 20% of horizontal segments
    if(horizon_segments_vec.size () > 20){ /// if the size is less than 20 then we use all of the segments for min and max height
        count_subset_horizon_segments = static_cast<int> (horizon_segments_vec.size () * 0.10);
    } else count_subset_horizon_segments = horizon_segments_vec.size ();

    printf(" 10 percent count of horizontal segments are : %d \n", count_subset_horizon_segments); //debug
    /// this file is for analysing the almost_horizontal segments with the bigger size, to estimate the floor and ceiling height
    FILE *almost_horizon_subset_infofile;
    strcpy (str_root, root_dir);
    almost_horizon_subset_infofile = fopen(strcat(str_root, "almost_horizon_subset_infofile.txt"),"w");
    fprintf(almost_horizon_subset_infofile, "segment_nr, segment_pointsize, centroid_height \n");
    /// collect horizontal segments centroid height and store in an info_file
    /// just for double check
    LaserPoints horizon_segments_subset; /// 10% of big horizontal segments
    for( int i=0; i < count_subset_horizon_segments ; i++){
        if(verbose){
            int horizon_segment_nr;
            horizon_segment_nr = horizon_segments_vec[i][0].SegmentNumber ();
            double segment_centroid_height;
            segment_centroid_height = horizon_segments_centroidheight_map.find (horizon_segment_nr)->second;
            /// print the information in a file
            fprintf(almost_horizon_subset_infofile, "%d,%d,%.2f \n",
                    horizon_segment_nr, horizon_segments_vec[i].size(), segment_centroid_height);
        }
        horizon_segments_subset.AddPoints (horizon_segments_vec[i]);
    }

    /// store subset of horizontal segments to the disk for double check
    strcpy (str_root, root_dir);
    horizon_segments_subset.Write(strcat(str_root,
            "almost_horizon_segs_10%subset.laser"), false);                                       // debug

    /* important estimation of floor and ceiling height */
    /// if the height of floor to the ceiling is less than 2 meters exit the program
    double maxima_z, minima_z;
    double ceiling_z_estimation, floor_z_estimation;
    minima_z = horizon_segments_subset.DeriveDataBounds (0).Minimum ().GetZ (); /// important floor estimation
    maxima_z = horizon_segments_subset.DeriveDataBounds (0).Maximum ().GetZ (); /// important ceiling estimation

    if(fabs(maxima_z - minima_z) < 2.0){ 
//        SetColor (RED);
        printf ("Ceiling - Floor height = %.2f \n", fabs(maxima_z - minima_z));
        printf ("Error in the floor and ceiling height estimation, less than 2 meters!!! \n");
//        SetColor (GRAY);
        exit (EXIT_FAILURE);
    } else {
        ceiling_z_estimation = maxima_z ;
        floor_z_estimation = minima_z;
//        SetColor (YELLOW);
        printf("Ceiling Height estimation: %.2f \n", ceiling_z_estimation);
        printf("Floor Height estimation: %.2f \n", floor_z_estimation);
        std::cout << endl;
//        SetColor (GRAY);
    }

    /// user defined parameters if ceil and floor estimation is not correct
    if (user_defined_ceil_floor_height){
        ceiling_z_estimation = ceiling_z_userdefined;
        floor_z_estimation   = floor_z_userdefined;

        //SetColor (DARKPINK);
        printf("Ceiling Height user defined: %.2f \n", ceiling_z_userdefined);
        printf("Floor Height user defined: %.2f \n", floor_z_userdefined);
        //SetColor (GRAY);
    }

    /* exclude almost horizontal segments that are far below the ceiling or far above the floor */
    /// such segments can belong to shelves, boxes on the ground or clutter
    vector <int> candidate_ceil_segment_nr, candidate_floor_segment_nr;
    vector <LaserPoints> candidate_ceil_segment_vec, candidate_floor_segment_vec;
    vector<pair <ObjectPoints,LineTopology>> min_rectangle_ceil;
    vector<pair <ObjectPoints,LineTopology>> min_rectangle_floor;
    LaserPoints candidate_ceil_floor_segment_lp_byheight;
    LaserPoints not_ceil_floor_byheight_lp;
    vector<int> not_ceil_floor_byheight_seg_nr;

    /// collect ceiling and floor segments candidates by checking the distance of a horizontal segment to
    /// the estimated ceiling and floor height
    printf("horizon segments vec size: %d \n", horizon_segments_vec.size ());                             //debug
    for (auto &segment : horizon_segments_vec){
        int horizon_segment_nr;
        horizon_segment_nr = segment[0].SegmentNumber ();
        if (verbose) printf("horizontal segment: %d \n", horizon_segment_nr);                             //debug
        double segment_centroid_height;
        segment_centroid_height = horizon_segments_centroidheight_map.find (horizon_segment_nr)->second;
        ObjectPoints mbr_corners;
        LineTopology mbr_edges;
        DataBoundsLaser db = segment.DeriveDataBounds (0); /// is required for EnclosingRectangle
        /// collect candidate ceilings and their minimum enclosing rectangle
        if(fabs(ceiling_z_estimation - segment_centroid_height) <= ceiling_buffer){
            candidate_ceil_segment_nr.push_back(horizon_segment_nr);
            candidate_ceil_floor_segment_lp_byheight.AddPoints (segment);
            candidate_ceil_segment_vec.push_back (segment);
            if (verbose) printf("Derive TIN for ceiling candidate segment: %d \n", horizon_segment_nr);   //debug
            segment.DeriveTIN (); /// is required for EnclosingRectangle      // WARNING: DeriveTIN crashes sometimes
            segment.EnclosingRectangle (0.10, mbr_corners, mbr_edges);
            min_rectangle_ceil.emplace_back (std::make_pair (mbr_corners, mbr_edges));
            if (verbose) printf ("candidate ceiling number: %d \n", horizon_segment_nr);                 // debug
        }
        /// collect candidate floors and their minimum enclosing rectangle
        if(fabs(floor_z_estimation - segment_centroid_height) <= floor_buffer){
            candidate_floor_segment_nr.push_back(horizon_segment_nr);
            candidate_ceil_floor_segment_lp_byheight.AddPoints (segment);
            candidate_floor_segment_vec.push_back (segment);
            if (verbose) printf("Derive TIN for floor candidate segment: %d \n", horizon_segment_nr);     //debug
            segment.DeriveTIN (); /// is required for EnclosingRectangle
            segment.EnclosingRectangle (0.10, mbr_corners, mbr_edges);
            //double area = mbr_edges.CalculateArea (mbr_corners); // not tested
            min_rectangle_floor.emplace_back (std::make_pair (mbr_corners, mbr_edges));

            if (verbose)  printf ("candidate floor number: %d \n", horizon_segment_nr);                  // debug
        }

        /// now collect every horizontal segment between the floor and ceiling in one vectorfile
        if(fabs(ceiling_z_estimation - segment_centroid_height) > ceiling_buffer &&
           fabs(floor_z_estimation - segment_centroid_height) > floor_buffer){
            not_ceil_floor_byheight_seg_nr.push_back (horizon_segment_nr);
            not_ceil_floor_byheight_lp.AddPoints (segment);
        }
    }

    /// store candidate floor and ceiling for double check
    strcpy (str_root, root_dir);
    candidate_ceil_floor_segment_lp_byheight.Write(strcat(str_root,
            "candidate_ceil_floor_segments_byheight.laser"), false);                              // debug

    /// store not candidate floor and ceiling for double check
    strcpy (str_root,root_dir);
    not_ceil_floor_byheight_lp.Write(strcat(str_root,
            "not_ceil_floor_segments_byheight.laser"), false);                                    // debug

    /* loop through almost horizontal segments and check if their oriented bboxes overlap, if yes then check their
     * minimum bounding rectangles (MBR) intersect/overlap or not. For safety of intersection on adjacent segments
     * we resize the MBR to a smaller size of 0.98 of original size.
     * If intersection checking is positive then the lower segments
     * would be excluded from the list of candidate ceilings.
     * NOTE: there is no check if the minimum bounding rectangle is not null.
     */

    vector<int> not_ceiling_segments_nr; /// output of the function
    vector <LaserPoints> ceil_segment_byIntersection_vec;
    LaserPoints not_ceil_segments_lp; /// output of the function
    ceil_segment_byIntersection_vec = filter_ceil_by_intersection (horizon_segments_centroidheight_map,
                                                                   candidate_ceil_segment_vec, min_rectangle_ceil,
                                                                   not_ceiling_segments_nr, not_ceil_segments_lp,
                                                                   intersection_percentage, false);
    //printf("not_ceiling_segments_nr size: %d \n", not_ceiling_segments_nr.size ()); // debug
    /// debug
    LaserPoints candidates_ceilings;
    for(auto s_ceil : ceil_segment_byIntersection_vec){
        candidates_ceilings.AddPoints (s_ceil);
    }
    strcpy (str_root, root_dir);
    candidates_ceilings.Write(strcat(str_root,
            "candidates_ceilings_byintersection.laser"), false);                                  // debug



    /// we use the same function to check floors, just the "is_floor" flag would be true
    vector<int> not_floor_segments_nr; /// output of the function
    vector <LaserPoints> floor_segment_byIntersection_vec;
    LaserPoints not_floor_segments_lp; /// output of the function
    floor_segment_byIntersection_vec = filter_ceil_by_intersection (horizon_segments_centroidheight_map,
                                                                    candidate_floor_segment_vec, min_rectangle_floor,
                                                                    not_floor_segments_nr, not_floor_segments_lp,
                                                                    intersection_percentage, true);
    //printf("not_floor_segments_nr size: %d \n", not_floor_segments_nr.size ()); // debug
    /// debug
    LaserPoints candidates_floor;
    for(auto s_floor : floor_segment_byIntersection_vec){
        candidates_floor.AddPoints (s_floor);
    }
    strcpy (str_root, root_dir);
    candidates_floor.Write(strcat(str_root,
            "candidates_floors_byintersection.laser"), false);                                    // debug

    LaserPoints not_floor_ceiling_byintersection;
    not_floor_ceiling_byintersection.AddPoints (not_ceil_segments_lp);
    not_floor_ceiling_byintersection.AddPoints (not_floor_segments_lp);
    //strcpy (str_root,root);
    //not_floor_ceiling_byintersection.Write(strcat(str_root, 
    //                                      "not_floor_ceiling_by_intersection.laser"), false);                 // debug

    LaserPoints not_floor_ceiling_all;
    not_floor_ceiling_all.AddPoints (not_ceil_floor_byheight_lp);
    not_floor_ceiling_all.AddPoints (not_floor_ceiling_byintersection);
    /// store not candidate floor and ceiling for double check
    strcpy (str_root, root_dir);
    not_floor_ceiling_all.Write(strcat(str_root, 
            "not_floor_ceiling_all_byIntersection.laser"), false);


    printf("%d segments are larger than %d\n", planes.size(), minSegmentSize);
    
    ///// *************************************  Create the Adjacency graph *****************************************  //////
    ///  *** processing segments two by two for topology relation, and creating the adjacency graph ***
    LineTopology      intersection_line_segment;
    ObjectPoint       beginp_objpnt, endp_objpnt;
    ObjectPoints      segments_pairpoints, shapepoints_intersection;
    LineTopologies    segments_pairlines, shapelines_intersection;
    vector <std::pair <int , int> >  pending_walls;

    int     takelabel           = 0,
            labwallceiling      = 1,
            //labwall_notceiling = 11,
            labwallfloor        = 2,
            labwallwall         = 3,
            labelceilceil       = 4,
            labelfloorfloor     = 5,
            labelwallslantedwall= 6;

    PointNumberLists::iterator  point_list_it1;
    Planes::iterator            plane_it1;
    int     intersection_cnt = 0;
    int     line_number      = 0;

    FILE *statfile;
    strcpy (str_root, root_dir);
    statfile = fopen(strcat(str_root, "graphinfo.txt"),"w");
    fprintf(statfile, "segment_nr1, segment_nr2, takelabel, planes' angle \n");

    FILE *statfile2;
    strcpy (str_root, root_dir);
    statfile2 = fopen(strcat(str_root, "graphinfo2.txt"),"w");
    fprintf(statfile2, "segment_nr1, segment_nr2, takelabel, planes' angle, planes' distance \n");

    // debugger
    FILE *statWalls;
    strcpy (str_root, root_dir);
    statWalls = fopen(strcat(str_root, "statWalls.txt"),"w");
    fprintf(statWalls, "Segments may belong to the same wall..\n");
    fprintf(statWalls, "segment_nr1, segment_nr2, planes' angle, planes' distance \n");

    /// looping through point_lists and corresponding plane, each point_list representing a segment
    for (point_list_it1 = point_lists.begin(), plane_it1 = planes.begin();
         point_list_it1 != point_lists.end(),  plane_it1 != planes.end(); point_list_it1++, plane_it1++){

        int segment_nr1, label1;
        /// get the segment nr of the current pointlist (segment)
        segment_nr1 = (laserpoints[point_list_it1 -> begin() -> Number()]).Attribute(SegmentNumberTag);
        /// get the label, should be 1 for Horiz or 2 for vertic
        label1      = (laserpoints[point_list_it1 -> begin() -> Number()]).Attribute(LabelTag);

        if(apply_intersection_result) {
            std::vector<int>::iterator itt1;
            std::vector<int>::iterator itc1, itf1;
            itc1 = find(not_ceiling_segments_nr.begin (), not_ceiling_segments_nr.end (), segment_nr1); /// not ceiling by intersection check
            itf1 = find(not_floor_segments_nr.begin (), not_floor_segments_nr.end (), segment_nr1); /// not floor by intersection check
            /// check the floor or ceiling candidate validation by height
            itt1 = find(not_ceil_floor_byheight_seg_nr.begin (), not_ceil_floor_byheight_seg_nr.end (), segment_nr1);
            if (itt1 != not_ceil_floor_byheight_seg_nr.end ()) {
                continue;
            } else {
                if (itc1 != not_ceiling_segments_nr.end () || itf1 != not_floor_segments_nr.end ()) { /// segment_nr found in not_candidates_segments_nr
                    continue; /// if the segment is not a ceiling or floor candidate then continue to the next segment
                }
            }
        }


        PointNumberLists::iterator  point_list_it2;
        Planes::iterator            plane_it2;
        /// selecting second segment point list and second plane
        for (point_list_it2 = point_list_it1 + 1, plane_it2 = plane_it1 +1;
             point_list_it2 != point_lists.end(), plane_it2 != planes.end(); point_list_it2++, plane_it2++){

            int segment_nr2, label2;
            segment_nr2 = (laserpoints[point_list_it2 -> begin() -> Number()]).Attribute(SegmentNumberTag);
            label2      = (laserpoints[point_list_it2 -> begin() -> Number()]).Attribute(LabelTag);


            if(apply_intersection_result){
                std::vector<int>::iterator itc2, itf2;
                std::vector<int>::iterator itt2;
                itc2 = find(not_ceiling_segments_nr.begin (), not_ceiling_segments_nr.end (), segment_nr2);
                itf2 = find(not_floor_segments_nr.begin (), not_floor_segments_nr.end (), segment_nr2);
                itt2 = find(not_ceil_floor_byheight_seg_nr.begin (), not_ceil_floor_byheight_seg_nr.end (), segment_nr2);
                if (itt2 != not_ceil_floor_byheight_seg_nr.end ()) {
                    continue;
                } else {
                    if (itc2 != not_ceiling_segments_nr.end () || itf2 != not_floor_segments_nr.end ()) { /// segment_nr found in not_candidate_segments_nr
                        continue;  /// if the segment is not a ceiling or floor candidate then continue to the next segment
                    }
                }
            }


//            SetColor(GREEN);
            printf("Analysing segments %4d and %4d for adjacency: \r", segment_nr1, segment_nr2);
//            SetColor(GRAY); /// set color back to default console

            // debugger
/*            if (verbose && segment_nr1 == 2 && segment_nr2 == 7) {
                cout << "stop" << endl;
            }*/

            ///Determine the intersection line of two planes,
            Line3D      intersection_line;
            Vector3D    line_direction;
            if (Intersect2Planes(*plane_it1, *plane_it2, intersection_line)){
                takelabel =0;
                bool b1, b2;
                double planes_angle, planes_distance;
                //if(verbose){ /// not necessary computations just for information
                    b1 = (*plane_it1).IsHorizontal(flat_angle * PI / 180);
                    b2 = (*plane_it2).IsHorizontal(flat_angle * PI / 180);
                    /// calculate planes' normal
                    Vector3D normal1, normal2;
                    normal1 = (*plane_it1).Normal();
                    normal2 = (*plane_it2).Normal();
                    /// plane angle is calculated by angle between their normals
                    planes_angle = (acos(normal1.DotProduct(normal2) / (normal1.Length() * normal2.Length()))) * 180.0 / PI;
                    /// planes distance is calculated by the distance of one plane to centerofgravity of another plane
                    planes_distance = fabs((*plane_it1).Distance(plane_it2 -> CentreOfGravity()));
                //}

                /// intersect two planes to get the positions of intersection line
                /// **** important function ****
                Position3D  intersection_pos1, intersection_pos2;
                if (laserpoints.IntersectFaces(*point_list_it1, *point_list_it2,
                                               *plane_it1, *plane_it2, max_intersection_dist,/// 0.1 is default intersection threshold
                                               intersection_pos1, intersection_pos2)){

                    PointNumber pn1, pn2;
                    Covariance3D cov3d;
                    cov3d = Covariance3D(0, 0, 0, 0, 0, 0);

                    /// convert begin and end-point of line intersect to objpoint
                    intersection_cnt++;
                    pn1 = PointNumber(intersection_cnt);
                    beginp_objpnt = ObjectPoint(intersection_pos1, pn1, cov3d);

                    intersection_cnt++;
                    pn2 = PointNumber(intersection_cnt);
                    endp_objpnt = ObjectPoint(intersection_pos2, pn2, cov3d);
                    /// create the line_topology
                    intersection_line_segment = LineTopology(line_number, 1, pn1, pn2);

                    line_direction = intersection_line.Direction();

                    /// labeling the intersection line segment ... not necessary procedure
                    if(verbose){
                        /// label horizontal lines
                        if (fabs(line_direction[2]) < flat_angle * PI / 180){
                            intersection_line_segment.Label() = 6;  // almost horizontal, label 6
                        }else{
                            intersection_line_segment.Label()= 3;  // non-horizontal lines
                        }

                        if(b1 && b2) intersection_line_segment.Label() = 4; /// both planes are horizontal
                    }

                    /// calculate the intersection_line length
                    double line_length;
                    line_length = intersection_pos1.Distance(intersection_pos2);
                    //printf("line lenght: %.2f \n", line_length); cout << endl;

                    if (line_length > min_intersection_linelength){
                        ObjectPoint seg1_cent_objp, seg2_cent_objp;
                        seg1_cent_objp = laserpoints.Centroid(*point_list_it1, segment_nr1);
                        seg2_cent_objp = laserpoints.Centroid(*point_list_it2, segment_nr2);

                        /* labeling process of intersection line ***
                         * already we had label =1 as horizontal and label =2 as vertical for segments
                         * labwallceiling  = 1, labwall_notceiling  = 11,
                         * labwallfloor    = 2,
                         * labwallwall     = 3;
                         * */
                        //if(segment_nr1 == 57 && segment_nr2 == 68)
                        //    printf("debug \n");
                        takelabel = 0;
                        //printf ("\n"); // debug
                        //if (label1 ==2 && label2 == 2) takelabel = labwallwall;
                        //wallwall_vertical_dist = 0.50;
                        if (label1 ==2 && label2 ==2){
                            takelabel = labwallwall;
                            /// check if the intersection of two wall candidates is horizontal
                            // don't replace this angle threshold with flat_angle parameter, they are different.
                            if (intersection_line.IsHorizontal (10.0 * PI / 180)){
                                LaserPoints seg1_lp, seg2_lp;
                                DataBoundsLaser db1, db2;  /// collect data bounds of the segments to check the support relation
                                seg1_lp = laserpoints.SelectTagValue(SegmentNumberTag, segment_nr1);  // too expensive
                                seg2_lp = laserpoints.SelectTagValue(SegmentNumberTag, segment_nr2);  // too expensive
                                db1 = seg1_lp.DeriveDataBounds(0);
                                db2 = seg2_lp.DeriveDataBounds(0);
                                /// check the center of which segment is higher
                                if (seg1_cent_objp.Z () > seg2_cent_objp.Z ()){
                                    /* /// check the support relation between two segments ///
                                     if the minimum height of upper segment is higher than the maximum height
                                     of the lower segment, then the lower segment is supporting the upper segment */
                                    //printf ("db1.minZ: %2.2f , db2.maxZ %2.2f \n", db1.Minimum ().GetZ (), db2.Maximum ().GetZ ()); //debug
                                    //printf ("db1.minZ - db2.maxZ: %2.2f \n", fabs(db1.Minimum ().GetZ ()- db2.Maximum ().GetZ ())); // debug
                                    if(wallwall_vertical_dist_condition){
                                        if(fabs(db1.Minimum ().GetZ () - db2.Maximum ().GetZ () ) <=
                                           wallwall_vertical_dist) {
                                            takelabel = labelwallslantedwall;
                                        }
                                    } else takelabel = labelwallslantedwall;

                                } else { /// else center of seg2 is higher than center of seg1
                                    //printf ("ZC2 > ZC1 and \n db2.minZ: %2.2f , db1.maxZ %2.2f \n",
                                            //db2.Minimum ().GetZ (), db1.Maximum ().GetZ ()); //debug
                                    //printf ("db2.minZ - db1.maxZ: %2.2f \n", fabs(db2.Minimum ().GetZ ()- db1.Maximum ().GetZ ())); // debug
                                    if(wallwall_vertical_dist_condition){
                                        if(fabs(db2.Minimum ().GetZ () - db1.Maximum ().GetZ () ) <=
                                           wallwall_vertical_dist) {
                                            takelabel = labelwallslantedwall;
                                        }
                                    }else takelabel = labelwallslantedwall;

                                }
                                  //printf("takelabel: %d \n", takelabel);  //debug
                            }
                        }

                        if (label1 == 2 && label2 == 1 && seg2_cent_objp.Z() > seg1_cent_objp.Z()  &&
                            (fabs(seg2_cent_objp.Z() - ceiling_z_estimation)) <= ceiling_buffer) // check if horiz-segment is close to the ceiling
                            takelabel = labwallceiling;

                        if (label1 == 1 && label2 == 2 && seg1_cent_objp.Z() > seg2_cent_objp.Z() &&
                            (fabs(seg1_cent_objp.Z() - ceiling_z_estimation)) <= ceiling_buffer) // check if horiz-segment is close to the ceiling
                            takelabel = labwallceiling;

                        if (label1 == 2 && label2 == 1 && seg1_cent_objp.Z() > seg2_cent_objp.Z() &&
                                (fabs(seg2_cent_objp.Z() - floor_z_estimation) <= floor_buffer))  // check if horiz-segment is close to the floor
                            takelabel = labwallfloor;
                        if (label1 == 1 && label2 == 2 && seg2_cent_objp.Z() > seg1_cent_objp.Z() &&
                                (fabs(seg1_cent_objp.Z() - floor_z_estimation) <= floor_buffer)) // check if horiz-segment is close to the floor
                            takelabel = labwallfloor;

                        /// if both segments are almost-horizontal and near the ceiling estimated height is a ceiling candidate
                        if (label1 ==1 && label2 == 1){
                            if(((fabs(seg1_cent_objp.Z() - ceiling_z_estimation)) <= ceiling_buffer) ||
                               ((fabs(seg2_cent_objp.Z() - ceiling_z_estimation)) <= ceiling_buffer))
                                takelabel = labelceilceil;
                        }

                        /// if both segments are almost-horizontal and near the floor estimated height is a floor-candidate
                        if (label1 ==1 && label2 == 1){
                            if(((fabs(seg1_cent_objp.Z() - floor_z_estimation)) <= floor_buffer) ||
                               ((fabs(seg2_cent_objp.Z() - floor_z_estimation)) <= floor_buffer))
                                takelabel = labelfloorfloor;
                        }

                        //printf ("TakeLabel2: %d \n", takelabel); //debug

                        /// labwall_notceiling is for wall and horizontal_plane connection
                        /// by this we avoid vertical_planes connected to a horiz_plane get wall label
/*                            if (label1 == 2 && label2 == 1 && seg2_cent_objp.Z() > seg1_cent_objp.Z() &&
                                 (fabs(seg2_cent_objp.Z() - ceiling_z_estimation)) > 0.20)
                            takelabel = labwall_notceiling;
                        if (label1 == 1 && label2 == 2 && seg1_cent_objp.Z() > seg2_cent_objp.Z() &&
                                (fabs(seg1_cent_objp.Z() - ceiling_z_estimation)) > 0.20)
                            takelabel = labwall_notceiling;*/

                        line_number++;
                        LineTopology segments_pairline;
                        segments_pairline = LineTopology(line_number, 1, segment_nr1, segment_nr2);
                        /// label the line that pairs two segments (this is not intersection line)
                        segments_pairline.SetAttribute(BuildingPartNumberTag, takelabel); // to be visualised in PCM
                        segments_pairline.Label() = takelabel;

                        //LineTopologies segments_pairlines;
                        segments_pairlines.push_back(segments_pairline);

                        /// this is intersection line, we set to it new labels
                        LineTopology shapeline_intersection;
                        shapeline_intersection = intersection_line_segment;
                        shapeline_intersection.Label() = takelabel;
                        shapeline_intersection.SetAttribute(BuildingPartNumberTag, takelabel); // to be visualised in PCM

                        //ObjectPoints shapepoints_intersection;
                        shapepoints_intersection.push_back(beginp_objpnt);
                        shapepoints_intersection.push_back(endp_objpnt);

                        //LineTopologies shapelines_intersection;
                        shapelines_intersection.push_back(shapeline_intersection);
                        //shapelines_intersection_test.push_back(intersection_line_segment);

                        fprintf (statfile, "%4d, %4d, %d, %3.2f \n", segment_nr1, segment_nr2, takelabel, planes_angle);
                    }
                }
                // debugger
                fprintf (statfile2, "%4d, %4d, %d, %3.2f, %.2f \n", segment_nr1, segment_nr2, takelabel, planes_angle, planes_distance);
            }
        } // end of for point_list_it2

        ObjectPoint seg1_cent_objp; /// ??? define and calculate again?
        seg1_cent_objp = laserpoints.Centroid(*point_list_it1, segment_nr1);
        segments_pairpoints.push_back(seg1_cent_objp);
    } // end of for loop point_list_it

    printf ("\n");
    strcpy (str_root, root_dir);
    shapepoints_intersection.Write(strcat(str_root, "intersection_lines.objpts"));
    strcpy (str_root, root_dir);
    shapelines_intersection.Write(strcat(str_root, "intersection_lines.top"), false);
    //shapelines_intersection_test.Write("D://test//indoor_reconstruction//intersection_lines_test.top", false);

    segments_pairpoints.RemoveDoublePoints(segments_pairlines, 0.01);

    /// loop through graph nodes, each node (segments_pairpoints) representing a segment
    int wall_count = 0;
    ObjectPoints::iterator segments_pairpoints_it; /// this representing a segment
    LaserPoints outputpoints, wall_points, unknown_points, ceiling_points, floor_points;
    vector <int> relabeled_walls;  /// contain segment numbers that can be wall

    fprintf(statfile, "\n print statistics..\n");
    fprintf(statWalls, "\n List of walls ...\n");
    for (segments_pairpoints_it = segments_pairpoints.begin();
         segments_pairpoints_it != segments_pairpoints.end(); segments_pairpoints_it++){

        int connection_count    = 0;
        int count_wallwall      = 0,
            count_wallceiling   = 0,
            count_wallfloor     = 0,
            count_ceiliceil     = 0,
            count_floorfloor    = 0,
            count_wallslantedwall = 0;

/*        if (segments_pairpoints_it -> Number() == 67){
            cout << "debug" << endl;
        }*/

        /// for each node, we loop through graph-edges (segments_pairlines), graph-edges connect segments
        LineTopologies::iterator segments_pairlines_it;
        for (segments_pairlines_it = segments_pairlines.begin();
                segments_pairlines_it != segments_pairlines.end(); segments_pairlines_it++){
            /// find edges that contain this node (segments_pairpoints_it)
            if(segments_pairlines_it -> Contains(segments_pairpoints_it -> NumberRef())){
                connection_count++; /// count number of connections
                /// important for labeling
                if(segments_pairlines_it -> Attribute(LineLabelTag) == labwallceiling)        count_wallceiling++;
                //if(segments_pairlines_it -> Attribute(LineLabelTag) == labwall_notceiling)  count_wall_notceiling++;
                if(segments_pairlines_it -> Attribute(LineLabelTag) == labwallfloor)          count_wallfloor++;
                if(segments_pairlines_it -> Attribute(LineLabelTag) == labwallwall)           count_wallwall++;
                if(segments_pairlines_it -> Attribute(LineLabelTag) == labelceilceil)         count_ceiliceil++;
                if(segments_pairlines_it -> Attribute(LineLabelTag) == labelfloorfloor)       count_floorfloor++;
                if(segments_pairlines_it -> Attribute(LineLabelTag) == labelwallslantedwall)  count_wallslantedwall++;
            }
        }
        //printf ("Count slanted walls: %d \n", count_wallslantedwall); //debug
        /// collect points for current segment
        LaserPoints segment_laserpoints;
        int segments_pairpoints_nr; /// current segment_number
        segments_pairpoints_nr = segments_pairpoints_it -> Number();
        segment_laserpoints = laserpoints.SelectTagValue(SegmentNumberTag, segments_pairpoints_nr);

        if (connection_count == 0){
            segment_laserpoints.SetAttribute(LabelTag, 0); /// 0 is default labeltag
        }

        //if (verbose){
            int count1;
            takelabel = 0;
            takelabel = segment_laserpoints.MostFrequentAttributeValue(LabelTag, count1);
            // 0 default, 1 horizon and 2 vertical
        if (takelabel==0) fprintf(statfile, "segment %d (%d points) is not connected.\n",
                                  segments_pairpoints_nr, segment_laserpoints.size());
        if (takelabel==1) fprintf(statfile, "segment %d (%d points) is horizontal, has %d connections,\n",
                                  segments_pairpoints_nr, segment_laserpoints.size(), connection_count);
        if (takelabel==2) fprintf(statfile, "segment %d (%d points) is vertical, has %d connections,\n",
                                  segments_pairpoints_nr, segment_laserpoints.size(), connection_count);

            fprintf(statfile, "of which %d ww, %d wcl and %d wfl  and %d clcl and %d flfl and %d wslw connections.\n",
                    count_wallwall, count_wallceiling, count_wallfloor, count_ceiliceil, count_floorfloor, count_wallslantedwall);
        //}

        /// The segment connected to the floor AND ceiling, would be a wall is not always true
        //if (count_wallceiling == 1 && count_wallfloor ==1)  segment_laserpoints.SetAttribute(LabelTag, 4); /// label wall

        /* Not connected segments to the ceiling may get wall label, because any horizontal segment is considered ceiling
        * count_wallceiling == 1 always is not correct, because every segment may be connected more than once to the ceiling
        */
        if (count_wallceiling >= 1 && takelabel != 1){ /// we exclude horizontal planes getting wall label
            segment_laserpoints.SetAttribute(LabelTag, 4); /// label wall
            wall_count++;
            /// list of wall segments
            fprintf(statWalls, "%d, ", segments_pairpoints_nr);
            /// collect segments that can be wall because of closeness and almost parallel plane to current wall
            if (!pending_walls.empty()){
                vector<std::pair<int, int>>::iterator it;
                for (it = pending_walls.begin(); it != pending_walls.end(); it++){
                    if (it ->first == segments_pairpoints_nr){
                        relabeled_walls.push_back(it ->second);
                    }
                    if(it -> second == segments_pairpoints_nr){
                        relabeled_walls.push_back(it -> first);
                        //fprintf(statWalls, "(%d,%d), ", it ->first, it ->second);
                    }
                }
/*            it = find_if(pending_walls.begin(), pending_walls.end(),
                         [&segments_pairpoints_nr](const std::pair<int, int>& element)
                                { return element.second == segments_pairpoints_nr;}
                        );*/
            }
        }
        if (count_wallslantedwall > 0 && count_wallwall >0 && takelabel != 1)
            segment_laserpoints.SetAttribute(LabelTag, 3); /// label wall-slanted-wall

        if (count_wallfloor > 2 && count_wallwall==0 && takelabel != 2)       segment_laserpoints.SetAttribute(LabelTag, 5); /// label floor
        if (count_wallceiling > 2 && count_wallwall==0 && takelabel != 2)     segment_laserpoints.SetAttribute(LabelTag, 6); /// label ceiling
        if (count_wallceiling > 0 && count_wallwall==0 && count_ceiliceil > 0 && takelabel !=2) /// for non horizontal ceiling
            segment_laserpoints.SetAttribute(LabelTag, 6);
        if (count_wallfloor > 0 && count_wallwall==0 && count_floorfloor > 0 && takelabel !=2) /// label non horizontal floor (ramp)
            segment_laserpoints.SetAttribute(LabelTag, 5);

        /// store floor data
        if (segment_laserpoints[0].Attribute(LabelTag) == 5)
            floor_points.AddPoints(segment_laserpoints);
        /// store ceiling data
        if (segment_laserpoints[0].Attribute(LabelTag) == 6)
            ceiling_points.AddPoints(segment_laserpoints);
        /// store wall points
        if (segment_laserpoints[0].Attribute(LabelTag) == 4 ||    // walls
                segment_laserpoints[0].Attribute(LabelTag) == 3) // slanted walls
            wall_points.AddPoints(segment_laserpoints);
        /// store unknown points
        if (segment_laserpoints[0].Attribute(LabelTag) == 2 ||      //almost vertical points
                segment_laserpoints[0].Attribute(LabelTag) == 1 ||  // almost horizontal points
                segment_laserpoints[0].Attribute(LabelTag) == 0)    // not_vertical and not_horizontal
            unknown_points.AddPoints(segment_laserpoints);
        /// store all labeled points
        outputpoints.AddPoints(segment_laserpoints);
    }

    /// crop wall points above the floor and below the ceiling // this is not necessary, unless we want to have
    /// a rectangle wall shape for occlusion test
    if(crop_walls_abovefloor_and_belowceiling){
        DataBoundsLaser db_floor, db_ceiling;
        db_floor    = floor_points.DeriveDataBounds(0);
        db_ceiling  = ceiling_points.DeriveDataBounds(0);

        if (db_floor.MinimumZIsSet() && db_floor.MaximumZIsSet() &&
            db_ceiling.MinimumZIsSet() && db_ceiling.MaximumZIsSet()){
            //floor_z_average = (db_floor.Minimum().GetZ() + db_floor.Maximum().GetZ()) / 2;  // this estimation is not accurate
            //ceiling_z_average = (db_ceiling.Minimum().GetZ() + db_ceiling.Maximum().GetZ()) / 2; // this estimation is not accurate

            LaserPoints::iterator wallpoint_it;
            LaserPoints wall_croped;
            wall_croped = wall_points;
            for (wallpoint_it = wall_points.begin(); wallpoint_it != wall_points.end(); wallpoint_it++){

                if (wallpoint_it->GetZ() > ceiling_z_average || wallpoint_it->GetZ() < floor_z_average){
                    wallpoint_it->SetAttribute(LabelTag, 8);  // wall points above the ceiling or lower than floor
                }
            }
            strcpy (str_root, root_dir);
            wall_points.Write(strcat(str_root, "walls_cropped.laser"), false);
            wall_points.RemoveTaggedPoints(8, LabelTag);
        }
    }

    /* Important NOTE:
     * unknown_points and output_points should contain unlabeled points, unlabeled horizontal and unlabeled vertical
     * */
    unknown_points.AddPoints (not_floor_ceiling_all); /// there is a bug here if bool apply_intersection_result =true; then the intersected fl/cl is added to floor layer and unknown
    outputpoints.AddPoints(not_floor_ceiling_all);

    strcpy (str_root, root_dir);
    segments_pairpoints.Write(strcat(str_root, "pairlines.objpts")), strcpy (str_root, root_dir);
    segments_pairlines.Write(strcat(str_root, "pairlines.top"), false), strcpy (str_root, root_dir);
    unknown_points.Write(strcat(str_root, "unknown_points.laser"), false), strcpy (str_root, root_dir);
    floor_points.Write(strcat(str_root, "floor.laser"), false), strcpy (str_root, root_dir);
    ceiling_points.Write(strcat(str_root, "ceiling.laser"), false), strcpy (str_root, root_dir);
    wall_points.Write(strcat(str_root, "walls.laser"), false), strcpy (str_root, root_dir);
    outputpoints.Write(strcat(str_root, "outputlaser.laser"), false);

    /// look into unknown segments for candidate walls
    if (!relabeled_walls.empty()){
//        SetColor(GREEN);
        printf ("Relabeling pending walls ... \n");
//        SetColor(GRAY);

        vector<int>             seg_nrs;
        vector<int>::iterator   seg_it;
        LaserPoints relabeled_walls_lp;
        seg_nrs = unknown_points.AttributeValues(SegmentNumberTag);  // vector of segment numbers
        for(seg_it = seg_nrs.begin(); seg_it != seg_nrs.end(); seg_it++){

            vector<int>::iterator it;
            it = find(relabeled_walls.begin(), relabeled_walls.end(), *seg_it);
            if (*it == *seg_it){
                LaserPoints seg_lp;
                seg_lp = unknown_points.SelectTagValue(SegmentNumberTag, *seg_it);
                seg_lp.SetAttribute(LabelTag, 44); /// relabel as wall
                relabeled_walls_lp.AddPoints(seg_lp);
            }
        }
        strcpy (str_root, root_dir);
        relabeled_walls_lp.Write(strcat(str_root, "relabeled_walls.laser"), false);
    }



    strcpy (str_root, root_dir);
    small_segments.Write(strcat(str_root, "small_segments.laser"), false);
    strcpy (str_root, root_dir);
    notsegmented_points.Write(strcat(str_root, "notsegmented_points.laser"), false);

    fclose(statfile);
    fclose(statfile2);
    fclose(statWalls);
    fclose(segments_infofile);
    fclose(almost_horizon_infofile);
    fclose(almost_horizon_subset_infofile);
    /// write the orginal file to the output
    strcpy (str_root, root_dir);
    laserpoints.Write(strcat(str_root, "laserpoints.laser"), false);

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Total wall processing time: "<< duration/60 << "m" << '\n';

    /*
     * Occlusion test for opening detection
     * */
/*    if (do_occlusion_test){
        /// read trajectory points (scanner positions)
        char* trajFile;
        LaserPoints trajpoints;
        //trajFile = (char*) "D://test//indoor_reconstruction//data//trajectory3.laser";
         trajFile = (char *) "D:/test/zebrevo_ahmed/Revo_Trajectory.laser";
        trajpoints.Read(trajFile);
        strcpy (str_root,root);
        occlusion_test(laserpoints, wall_points, trajpoints, segment_buffers, 0.60, root);
    }*/

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"indoor topology processing time: "<< duration/60 << "m" << '\n';

}

