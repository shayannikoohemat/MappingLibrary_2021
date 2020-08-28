//
// Created by NikoohematS on 11/4/2016.
//

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <LaserPoints.h>
#include <map>
#include "Buffer.h"
#include "Buffers.h"
#include "indoor_reconstruction.h"


enum Color { DARKBLUE = 1, DARKGREEN, DARKTEAL, DARKRED, DARKPINK, DARKYELLOW, GRAY ,
                DARKGRAY, BLUE, GREEN, TEAL, RED, PINK, YELLOW, WHITE };
void SetColor(enum Color);

//LaserPoints segment_refinement(LaserPoints lp, int min_segment_size, double maxdistanceInComponent);

//void occlusion_test(LaserPoints laserpoints, LaserPoints surfacepoints,
//                    LaserPoints trajpoints, Buffers segment_buffers, double closeness_to_surface_threshold);

void indoorTopology_v1_old(char* laserFile, int minsizesegment, char* root, bool verbose){

    std::clock_t start;
    double duration;
    start = std::clock();

    char str_root[500];
    //char *root = (char*) "D://test//publication//TUM_testresult//";
    //char *root = (char*) "D://test//indoor_reconstruction//3rdfloor_601040_accuracycehck//";
    //auto *root = (char*) "E://BR_data//Diemen//process//out//";
    strcpy (str_root,root); // initialize the str_root with root string

    verbose = false;
    bool use_boost_lib = true;
    bool calculate_noraml =false;
    bool calculate_distance = false;
    bool do_segmentation = false;
    bool do_segment_refinement = false;
    bool generate_wallpatches = false; // generate wallpatches (M. Kada's approach) includes segment refinement
    bool do_occlusion_test= false;
    bool crop_walls_abovefloor_and_belowceiling =false; /// for firebrigade dataset or for glass floor and ceiling cases

    /// read laser points
    LaserPoints laserpoints;
    //// FB_sub1cm_seg12cm_refined_1770k_reflectremoved.laser
    //laserFile = (char*) "D://test//indoor_reconstruction//data//3rdfloor_modified_laserpoints_occlusiontestoutput.laser";
    //laserFile = (char*) "D://test//indoor_reconstruction//test_samples//crop_merge_seg8cm_withCeiling.laser";
    //laserFile = (char*) "D://test//indoor_reconstruction//3rdfloor_601040_accuracycehck//source//3rdFloor_thinned_rk4_3600k_seg1012cm.laser";
    //laserFile = (char*) "E://BR_data//ZebR//cc_sub2mil_basement_crop_seg10cm.laser";
    //laserFile = (char*) "D://test//publication//TUM_entrance.laser";
    laserpoints.Read(laserFile);
    SetColor(BLUE);
    printf (" input point size:  %d \n ", laserpoints.size());
    SetColor(GRAY); /// set color back to default console

    if (!laserpoints.Read(laserFile)) {
        printf("Error reading laser points from file %s\n", laserFile);
        exit(0);
    }

    SegmentationParameters *seg_parameter;
    seg_parameter = new SegmentationParameters;  /// ??? how to delete the object explicitly to avoid memory leak
    /// segmenting laserpoints
    if (do_segmentation){
        seg_parameter -> MaxDistanceInComponent()  = 0.3;
        seg_parameter -> SeedNeighbourhoodRadius() = 1.0;
        seg_parameter -> MaxDistanceSeedPlane()    = 0.20; // MaX Distance to the Plane
        seg_parameter -> GrowingRadius()           = 1.0;
        seg_parameter -> MaxDistanceSurface()      = 0.20;

        /// if segmentation crashes set the compatibility of generated exe to windows7
        printf("segmentation process... \n ");
        laserpoints.SurfaceGrowing(*seg_parameter);
    }

    /// collect points without segment nr
    LaserPoints notsegmented_points;
    if (verbose){
        LaserPoints::iterator point_it;
        for (point_it = laserpoints.begin(); point_it != laserpoints.end(); point_it++){
            if (!point_it->HasAttribute(SegmentNumberTag)){
                notsegmented_points.push_back(*point_it);
            }
        }
    }

    if (do_segment_refinement){
        printf("Processing segments larger than %d points.\n", minsizesegment);
        /// resegmenting laserpoints by removing longedges in the TIN and removing deformed segments
        LaserPoints refined_lp;
        refined_lp = segment_refinement(laserpoints, 2, 0.30);
        /// renew laserpoints with the result of segment_refinement
        laserpoints = refined_lp;
        strcpy (str_root,root);
        laserpoints.Write(strcat(str_root,"segment_refine_relabeled.laser"), false);
    }

    delete seg_parameter;

    /// default label ==0
    laserpoints.SetAttribute(LabelTag, 0);
    /// collecting list of point_numbers and planes for each segment
    Planes                  planes;
    PointNumberLists        point_lists;
    double                  PI;
    PI                      = 3.14159;
    double                  flat_angle, vertical_angle;
    flat_angle              = 50.01; //10.0 ;
    vertical_angle          = 39.99; //10.0;
    double                  max_intersection_dist;
    max_intersection_dist   = 0.10;

    vector<int>             segment_numbers;
    std::map<int, int>      horizon_segment_sizep_map;
    std::map<int, double>   horizon_segmens_centroidheight_map;
    vector<LaserPoints>     horizon_segments_vec;
    vector<int>::iterator   segment_it;
    LaserPoints             small_segments;
    Buffers                 segment_buffers;

    /*
 *  *** Important function for generating bigger segments as wall patch that is called buffer ***
 * */
    if(generate_wallpatches){
        printf ("\n Generate wall-patch segments ... (Martin-Kada's generalisation approach) \n");
        segment_buffers = GenerateWallPatches(laserpoints, 0.70, 20.0, 0.40, root);

        LaserPoints buffer_points;
        buffer_points = segment_buffers.Buffers_laserpoints();
        buffer_points.SetAttribute(LabelTag, 0);
        laserpoints = buffer_points; // this initializes pointnumbers (because of pointnumbers bug)

        LaserPoints relabeled_buffer_points;  // relabeled to vertical & horizontal
        for (int i=0; i< segment_buffers.size(); i++){
            LaserPoints bufflp;
            bufflp = segment_buffers[i].BuffLaserPoints();
            //bufflp[i].GetPointNumber(); // bug

            if (bufflp.size() > minsizesegment){
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
        strcpy (str_root,root);
        relabeled_buffer_points.Write(strcat(str_root,"buffer_segmentation.laser"), false);
        laserpoints = relabeled_buffer_points; // contains all buffers after renumbering their segmentation
    }


    FILE *segments_infofile;
    strcpy (str_root,root);
    segments_infofile = fopen(strcat(str_root, "segments_infofile.txt"),"w");
    fprintf(segments_infofile, "segment_nr, segment_pointsize, centroid_height, segment_angle \n");

    /// this file is to analysis almost_horizontal segments to estimate floor and ceiling height
    FILE *almost_horizon_infofile;
    strcpy (str_root,root);
    almost_horizon_infofile = fopen(strcat(str_root, "almost_horizontal_segments_infofile.txt"),"w");
    fprintf(almost_horizon_infofile, "segment_nr, segment_pointsize, centroid_height, segment_angle \n");
    /*
* if we didn't calculate wallpatches then we use original segments for wall detection
 *  and if we dont have boost library
* */
    if (!generate_wallpatches){
        segment_numbers = laserpoints.AttributeValues(SegmentNumberTag);  // vector of segment numbers
        std::sort(segment_numbers.begin(), segment_numbers.end());
        printf ("Number of laser segments: %d\n", segment_numbers.size());

        /// segment processing and fitting planes and labeling to 1 (horiz) and 2 (vertical)
        for(segment_it = segment_numbers.begin(); segment_it != segment_numbers.end(); segment_it++){
            //segment_lpoints.ErasePoints();
            /// selecting points by segment and saving in segment_lpoints
            LaserPoints     segment_lpoints;
            segment_lpoints = laserpoints.SelectTagValue(SegmentNumberTag, *segment_it);
            Plane   plane;
            if(segment_lpoints.size() > minsizesegment){

                plane = laserpoints.FitPlane(*segment_it, *segment_it, SegmentNumberTag);
                //plane = segment_lpoints.FitPlane (segment_lpoints[0].SegmentNumber ());
                //plane.Number () = segment_lpoints[0].SegmentNumber ();  /// set the segment number as the plane number

                PointNumberList point_list;
                //point_list.erase(point_list.begin(), point_list.end());  ???
                /// Collect a vector of point numbers with SegmentNumberTag value, used for polygon
                point_list = laserpoints.TaggedPointNumberList(SegmentNumberTag, *segment_it);
                point_lists.push_back(point_list);

                /// get the distribution of segment's size, plane's angle and segment's center height
                /// this information is useful for extraction of the ceiling and floor height candidate
                /// and getting the angle distribution of the segments
                double segment_angle;
                segment_angle = Angle (plane.Normal (), Vector3D(0,0,1)) * 180.0 / PI ;
                if (segment_angle > 90.0) segment_angle = segment_angle - 90.0;   //// not sure if this is correct

                ObjectPoint segment_centroid;
                segment_centroid = laserpoints.Centroid (point_list, *segment_it);
                fprintf(segments_infofile, "%d,%d,%.2f,%.2f \n",
                        *segment_it, segment_lpoints.size(), segment_centroid.Z (), segment_angle);

                /// label laserpoints with horizontal planes as 1
                if (plane.IsHorizontal(flat_angle * PI / 180)) { // 0.17 radian
                    laserpoints.ConditionalReTag (0, 1, LabelTag, *segment_it, SegmentNumberTag); // Horizontal 1
                    /// make an info file of almost horizontal segments
                    fprintf(almost_horizon_infofile, "%d,%d,%.2f,%.2f \n",
                            *segment_it, segment_lpoints.size(), segment_centroid.Z (), segment_angle);
                    horizon_segmens_centroidheight_map.insert ( std::pair<int, double>(*segment_it, segment_centroid.Z ()));
                    horizon_segment_sizep_map.insert(std::pair<int, int> (*segment_it, segment_lpoints.size ()));
                    horizon_segments_vec.push_back (segment_lpoints);
                }

                /// label laserpoints with vertical planes as 2
                if (plane.IsVertical(vertical_angle * PI / 180)) //  radian
                    laserpoints.ConditionalReTag(0, 2, LabelTag, *segment_it, SegmentNumberTag); // Vertical 2

                planes.push_back(plane);

            } else {
                small_segments.AddPoints(segment_lpoints);
            }
        } /// end of segment plane fitting
    }

    LaserPoints relabeled_vert_horiz;
    relabeled_vert_horiz = laserpoints;
    strcpy (str_root,root);
    relabeled_vert_horiz.Write(strcat(str_root, "relabeled_vert_horiz.laser"), false);

    /* estimate the ceiling and floor height by choosing dominanat almost-horizontal segments
     *  and check their height distribution *** this is just a rough estimation
     * */
    /// sort vector of horizontal_segments by size
    sort(horizon_segments_vec.begin (), horizon_segments_vec.end (), compare_lp_size); // descending
    /// select 10% of big horizontal segments to check their height distribution
    int count_subset_horizon_segments; /// count of 10% of horizontal segments
    count_subset_horizon_segments = static_cast<int> (horizon_segments_vec.size () * 0.10);
    //printf("10 percent of hirozntal segments is : %d \n", count_subset_horizon_segments); //debug

    vector<double> segment_centroid_height_v;
    /// this file is for analysiing the almost_horizontal segments with the bigger size, to estimate the floor and ceiling height
    FILE *almost_horizon_subset_infofile;
    strcpy (str_root,root);
    almost_horizon_subset_infofile = fopen(strcat(str_root, "almost_horizon_subset_infofile.txt"),"w");
    fprintf(almost_horizon_subset_infofile, "segment_nr, segment_pointsize, centroid_height \n");

    for( int i=0; i < count_subset_horizon_segments ; i++){
        int horizon_segment_nr;
        horizon_segment_nr = horizon_segments_vec[i][0].SegmentNumber ();
        double segment_centroid_height;
        segment_centroid_height = horizon_segmens_centroidheight_map.find (horizon_segment_nr)->second;
        segment_centroid_height_v.push_back (segment_centroid_height);


        /// print the information in a file
        fprintf(almost_horizon_subset_infofile, "%d,%d,%.2f \n",
                horizon_segment_nr, horizon_segments_vec[i].size(), segment_centroid_height);
    }


    double ceiling_z, floor_z;
    LaserPoints almost_horizontalpoints_tmp;
    //LaserPoints dominanat_seg1_lp, dominanat_seg2_lp;
    PointNumberList point_numlist1, point_numlist2;

    almost_horizontalpoints_tmp = relabeled_vert_horiz.SelectTagValue(LabelTag, 1);
    strcpy (str_root,root);
    almost_horizontalpoints_tmp.Write(strcat(str_root, "horiz.laser"), false);  /// debug
    int count1, count2;
    int dominanat_seg_no1, dominanat_seg_no2;
    dominanat_seg_no1 = almost_horizontalpoints_tmp.MostFrequentAttributeValue(SegmentNumberTag, count1);
    //dominanat_seg1_lp.AddTaggedPoints(lp_tmp, dominanat_seg_no1, SegmentNumberTag);
    point_numlist1 = almost_horizontalpoints_tmp.TaggedPointNumberList(SegmentNumberTag, dominanat_seg_no1);

    almost_horizontalpoints_tmp.RemoveTaggedPoints(dominanat_seg_no1, SegmentNumberTag);
    dominanat_seg_no2 = almost_horizontalpoints_tmp.MostFrequentAttributeValue(SegmentNumberTag, count2);
    //dominanat_seg2_lp.AddTaggedPoints(lp_tmp, dominanat_seg_no2, SegmentNumberTag);
    point_numlist2 = almost_horizontalpoints_tmp.TaggedPointNumberList(SegmentNumberTag, dominanat_seg_no2);

    ObjectPoint seg1_cent, seg2_cent;
    seg1_cent = laserpoints.Centroid(point_numlist1, dominanat_seg_no1);
    seg2_cent = laserpoints.Centroid(point_numlist2, dominanat_seg_no2);
    /// use dominant segment concept to estimate floor and ceiling height,
    /// so if floor, and ceiling have the most no of points
    //ceiling_z = max(seg1_cent.Z(), seg2_cent.Z());
    //floor_z   = min(seg1_cent.Z(), seg2_cent.Z());

    /// hard coded for visualisation, should be changed also in >> ceiling_z_average, end of the code
    ceiling_z = 1.07 ; //5.10; //1.70; //1.36  ;  //1.95; //4.75; 2.90; -1.62;  // change also
    floor_z   = -1.35; //1.60; //-1.75; //-0.85; //-0.73; //0.0;  0.0; -4.29; // hardcoded
    printf("Ceiling Height estimation: %.2f \n", ceiling_z);
    printf("Floor Height estimation: %.2f \n", floor_z);
/*    ceiling_z = max(dominanat_seg1_lp.DeriveDataBounds(0).Minimum().GetZ(),
                    dominanat_seg2_lp.DeriveDataBounds(0).Minimum().GetZ());*/

    printf("%d segments are larger than %d\n", planes.size(), minsizesegment);

    ///  *** processing segments two by two for topology relation ***
    LineTopology      intersection_line_topology;
    ObjectPoint       beginp_objpnt, endp_objpnt;
    ObjectPoints      segments_pairpoints, shapepoints_intersection;
    LineTopologies    segments_pairlines, shapelines_intersection;
    vector <std::pair <int , int> >  pending_walls;

    int     takelabel      =0,
            labwallceiling =1,
            //labwall_notceiling = 11,
            labwallfloor   =2,
            labwallwall    =3,
            labelceilceil = 4,
            labelfloorfloor = 5,
            labelwallslantedwall= 6;

    PointNumberLists::iterator  point_list_it1;
    Planes::iterator            plane_it1;
    int     intersection_cnt = 0;
    int     line_number      = 0;

    FILE *statfile;
    strcpy (str_root,root);
    statfile = fopen(strcat(str_root, "graphinfo.txt"),"w");
    fprintf(statfile, "segment_nr1, segment_nr2, takelabel, planes' angle \n");

    FILE *statfile2;
    strcpy (str_root,root);
    statfile2 = fopen(strcat(str_root, "graphinfo2.txt"),"w");
    fprintf(statfile2, "segment_nr1, segment_nr2, takelabel, planes' angle, planes' distance \n");

    // debugger
    FILE *statWalls;
    strcpy (str_root,root);
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

        PointNumberLists::iterator  point_list_it2;
        Planes::iterator            plane_it2;
        /// selecting second segment point list and second plane
        for (point_list_it2 = point_list_it1 + 1, plane_it2 = plane_it1 +1;
             point_list_it2 != point_lists.end(), plane_it2 != planes.end(); point_list_it2++, plane_it2++){

            int segment_nr2, label2;
            segment_nr2 = (laserpoints[point_list_it2 -> begin() -> Number()]).Attribute(SegmentNumberTag);
            label2      = (laserpoints[point_list_it2 -> begin() -> Number()]).Attribute(LabelTag);

            SetColor(GREEN);
            printf("Analysing segments: %4d and %4d \r", segment_nr1, segment_nr2);
            SetColor(GRAY); /// set color back to default console

            // debugger
            if (verbose && segment_nr1 == 2 && segment_nr2 == 7) {
                cout << "stop" << endl;
            }

            ///Determine the intersection line of two planes,
            Line3D      intersection_line;
            Vector3D    line_direction;
            if (Intersect2Planes(*plane_it1, *plane_it2, intersection_line)){

                bool b1, b2;
                b1 = (*plane_it1).IsHorizontal(flat_angle * PI / 180);
                b2 = (*plane_it2).IsHorizontal(flat_angle * PI / 180);

                /// calculate planes' normal if we need it
                double planes_angle = 0.0;
                Vector3D normal1, normal2;
                if (calculate_noraml){
                    normal1 = (*plane_it1).Normal();
                    normal2 = (*plane_it2).Normal();
                    /// plane angle is calculated by angle between their normals
                    planes_angle = (acos(normal1.DotProduct(normal2) / (normal1.Length() * normal2.Length()))) * 180.0 / PI;
                }

                /// claculate planes' distance if we need it
                double planes_distance = 0.0;
                if(calculate_distance){
                    /// here it may return negative values as distance
                    /// planes distance is calculated by the distance of one plane to centerofgravity of another plane
                    planes_distance = abs((*plane_it1).Distance(plane_it2 -> CentreOfGravity()));
                }

                /// check vertical segments' bbox overlap, if true they may belong to the same wall
                LaserPoints seg1_lp, seg2_lp;
                bool db_overlapXY;
                /// if both surfaces' planes are close and almost parallel
                if (calculate_distance && calculate_noraml){
                    if (planes_angle <= 10.0 && fabs(planes_distance) <= 0.30 && !b1 && !b2) { // if both surfaces are vertical
                        DataBoundsLaser db1, db2;  // we can calc dbs beforhand and save them in a vector
                        seg1_lp = laserpoints.SelectTagValue(SegmentNumberTag, segment_nr1);  // too expensive here
                        seg2_lp = laserpoints.SelectTagValue(SegmentNumberTag, segment_nr2);  // too expensive

                        db1 = seg1_lp.DeriveDataBounds(0);
                        db2 = seg2_lp.DeriveDataBounds(0);
                        db_overlapXY =  OverlapXY(db1, db2);
                        // TODO make an adjacency list graph instead make-pair
                        /// if two segments db overlaps in XY then save them as wall candidate
                        if (db_overlapXY){
                            pending_walls.push_back(std::make_pair(segment_nr1, segment_nr2));
                            //debugger
                            fprintf (statWalls, "%4d %4d, %3.2f, %.2f \n", segment_nr1, segment_nr2, planes_angle, planes_distance);
                        }
                    }
                }

                /// debugger
                /// check why two planes on top of each other doesnt have intersect faces
                if (verbose){
                    if (segment_nr1 ==3 && segment_nr2 ==6){

                        Position3D pos_seg1, pos_seg2;
                        pos_seg1 = plane_it1 ->CentreOfGravity();
                        pos_seg2 = plane_it2 ->CentreOfGravity();

                        ObjectPoint pos1_objp, pos2_objp;
                        ObjectPoints center_of_segments;
                        Covariance3D cov3d;
                        cov3d = Covariance3D(0, 0, 0, 0, 0, 0);
                        pos1_objp = ObjectPoint(pos_seg1, 1, cov3d);
                        pos2_objp = ObjectPoint(pos_seg2, 2, cov3d);
                        center_of_segments.push_back(pos1_objp);
                        center_of_segments.push_back(pos2_objp);
                        strcpy (str_root,root);
                        center_of_segments.Write(strcat(str_root, "center_of_segments.objpts"));

                        /// convert pointlistnumbers to laserpoints for control
                        LaserPoints current_pointslists;
                        PointNumberList pnl1, pnl2;
                        pnl1 = point_list_it1 -> PointNumberListReference();
                        for (int i=0; i< pnl1.size(); i++){
                            LaserPoint p;
                            p = laserpoints[pnl1[i].Number()];
                            current_pointslists.push_back(p);
                        }

                        pnl2 = point_list_it2 -> PointNumberListReference();
                        for (int i=0; i< pnl2.size(); i++){
                            LaserPoint p;
                            p = laserpoints[pnl2[i].Number()];
                            current_pointslists.push_back(p);
                        }
                        strcpy (str_root,root);
                        current_pointslists.Write(strcat(str_root, "current_two_pointnumberlists.laser"), false);
                    }
                }
                /// debugger end

                /// intersect two planes to get the positions of intersection
                /// **** important function ****
                Position3D  intersection_pos1, intersection_pos2;
                if (laserpoints.IntersectFaces(*point_list_it1, *point_list_it2,
                                               *plane_it1, *plane_it2, max_intersection_dist,/// 0.1 is intersection threshold
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
                    intersection_line_topology = LineTopology(line_number, 1, pn1, pn2);

                    line_direction = intersection_line.Direction();

                    /// label horizontal lines
                    if (fabs(line_direction[2]) < flat_angle * PI / 180)
                        intersection_line_topology.Label() = 6;  // almost horizontal, label 6
                    else intersection_line_topology.Label()= 3;  // non-horizontal lines

                    /// if two planes are horizontal do nothing ... not reliable
//                    if (b1 && b2){ // line created by two horizontal surfaces
                        intersection_line_topology.Label() = 4;
//                    }
 //                   else { //otherwise two planes are not horizontal
                        /// calculate the intersection_line length
                        double line_length;
                        double min_linelength = 0.1;
                        line_length = intersection_pos1.Distance(intersection_pos2);
                        //printf("line lenght: %.2f \n", line_length);

                        if (line_length > min_linelength){
                            ObjectPoint seg1_cent_objp, seg2_cent_objp;
                            seg1_cent_objp = laserpoints.Centroid(*point_list_it1, segment_nr1);
                            seg2_cent_objp = laserpoints.Centroid(*point_list_it2, segment_nr2);

                            /* labeling process of intersection line ***
                             * already we had label =1 as horizontal and label =2 as vertical for segments
                             * labwallceiling  = 1, labwall_notceiling  = 11,
                             * labwallfloor    = 2,
                             * labwallwall     = 3;
                             * */
                            if(segment_nr1 == 57 && segment_nr2 == 68)
                                printf("debug \n");
                            takelabel = 0;
                            //printf ("\n"); // debug
                            //TODO: handle the case when two almost vertical segments where one above each other their intersection is horizontal
                            if (label1 ==2 && label2 == 2) takelabel = labwallwall;
                            if (label1 ==2 && label2 ==2){
                                /// check if the intersection of two wall candidates is horizontal
                                if (intersection_line.IsHorizontal (10.0 * PI / 180)){
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
                                        printf ("db1.minZ: %2.2f , db2.maxZ %2.2f \n", db1.Minimum ().GetZ (), db2.Maximum ().GetZ ()); //debug
                                        printf ("db1.minZ - db2.maxZ: %2.2f \n", fabs(db1.Minimum ().GetZ ()- db2.Maximum ().GetZ ())); // debug
                                        if(fabs(db1.Minimum ().GetZ () - db2.Maximum ().GetZ () ) <= 0.10 ) {
                                            takelabel = labelwallslantedwall;
                                        }
                                    } else { /// else center of seg2 is higher than center of seg1
                                        printf ("ZC2 > ZC1 and \n db2.minZ: %2.2f , db1.maxZ %2.2f \n",
                                                db2.Minimum ().GetZ (), db1.Maximum ().GetZ ()); //debug
                                        printf ("db2.minZ - db1.maxZ: %2.2f \n", fabs(db2.Minimum ().GetZ ()- db1.Maximum ().GetZ ())); // debug
                                        if(fabs(db2.Minimum ().GetZ () - db1.Maximum ().GetZ () ) <= 0.10) {
                                            takelabel = labelwallslantedwall;
                                        }
                                    }
                                      printf("takelabel: %d \n", takelabel);  //debug
                                }
                            }

                            if (label1 == 2 && label2 == 1 && seg2_cent_objp.Z() > seg1_cent_objp.Z() &&
                                    (fabs(seg2_cent_objp.Z() - ceiling_z)) <= 0.5) // check if horiz-segment is close to the ceiling
                                takelabel = labwallceiling;

                            if (label1 == 1 && label2 == 2 && seg1_cent_objp.Z() > seg2_cent_objp.Z() &&
                                    (fabs(seg1_cent_objp.Z() - ceiling_z)) <= 0.5) // check if horiz-segment is close to the ceiling
                                takelabel = labwallceiling;

                            if (label1 == 2 && label2 == 1 && seg1_cent_objp.Z() > seg2_cent_objp.Z() &&
                                    (fabs(seg2_cent_objp.Z() - floor_z) <= 0.25))  // check if horiz-segment is close to the floor
                                takelabel = labwallfloor;
                            if (label1 == 1 && label2 == 2 && seg2_cent_objp.Z() > seg1_cent_objp.Z() &&
                                    (fabs(seg1_cent_objp.Z() - floor_z) <= 0.25)) // check if horiz-segment is close to the floor
                                takelabel = labwallfloor;

                            /// if both segments are almost-horizontal and near the ceiling
                            if (label1 ==1 && label2 == 1){
                                if(((fabs(seg1_cent_objp.Z() - ceiling_z)) <= 0.5) ||
                                   ((fabs(seg2_cent_objp.Z() - ceiling_z)) <= 0.5))
                                    takelabel = labelceilceil;
                            }

                            /// if both segments are almost-horizontal and near the floor
                            if (label1 ==1 && label2 == 1){
                                if(((fabs(seg1_cent_objp.Z() - floor_z)) <= 0.25) ||
                                   ((fabs(seg2_cent_objp.Z() - floor_z)) <= 0.25))
                                    takelabel = labelfloorfloor;
                            }

                            //printf ("TakeLabel2: %d \n", takelabel); //debug

                            /// labwall_notceiling is for wall and horizontal_plane connection
                            /// by this we avoid vertical_planes connected to a horiz_plane get wall label
/*                            if (label1 == 2 && label2 == 1 && seg2_cent_objp.Z() > seg1_cent_objp.Z() &&
                                     (fabs(seg2_cent_objp.Z() - ceiling_z)) > 0.20)
                                takelabel = labwall_notceiling;
                            if (label1 == 1 && label2 == 2 && seg1_cent_objp.Z() > seg2_cent_objp.Z() &&
                                    (fabs(seg1_cent_objp.Z() - ceiling_z)) > 0.20)
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
                            shapeline_intersection = intersection_line_topology;
                            shapeline_intersection.Label() = takelabel;
                            shapeline_intersection.SetAttribute(BuildingPartNumberTag, takelabel); // to be visualised in PCM

                            //ObjectPoints shapepoints_intersection;
                            shapepoints_intersection.push_back(beginp_objpnt);
                            shapepoints_intersection.push_back(endp_objpnt);

                            //LineTopologies shapelines_intersection;
                            shapelines_intersection.push_back(shapeline_intersection);
                            //shapelines_intersection_test.push_back(intersection_line_topology);

                            fprintf (statfile, "%4d %4d, %d, %3.2f \n", segment_nr1, segment_nr2, takelabel, planes_angle);
                        }
 //                   }
                }
                // debugger
                fprintf (statfile2, "%4d %4d, %d, %3.2f, %.2f \n", segment_nr1, segment_nr2, takelabel, planes_angle, planes_distance);
            }
        } // end of for point_list_it2

        ObjectPoint seg1_cent_objp; /// ??? define and calculate again?
        seg1_cent_objp = laserpoints.Centroid(*point_list_it1, segment_nr1);
        segments_pairpoints.push_back(seg1_cent_objp);
    } // end of for loop point_list_it

    printf ("\n");
    strcpy (str_root,root);
    shapepoints_intersection.Write(strcat(str_root, "intersection_lines.objpts"));
    strcpy (str_root,root);
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
            if (pending_walls.size()){
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

        if (count_wallfloor > 2 && count_wallwall==0)       segment_laserpoints.SetAttribute(LabelTag, 5); /// label floor
        if (count_wallceiling > 2 && count_wallwall==0)     segment_laserpoints.SetAttribute(LabelTag, 6); /// label ceiling
        if (count_wallceiling > 0 && count_wallwall==0 && count_ceiliceil > 0 && takelabel !=2)
            segment_laserpoints.SetAttribute(LabelTag, 6); /// label non horizontal ceiling
        if (count_wallfloor > 0 && count_wallwall==0 && count_floorfloor > 0 && takelabel !=2)
            segment_laserpoints.SetAttribute(LabelTag, 5); /// label non horizontal floor

        /// store floor data
        if (segment_laserpoints[0].Attribute(LabelTag) == 5)
            floor_points.AddPoints(segment_laserpoints);
        /// store ceiling data
        if (segment_laserpoints[0].Attribute(LabelTag) == 6)
            ceiling_points.AddPoints(segment_laserpoints);
        /// store wall points
        if (segment_laserpoints[0].Attribute(LabelTag) == 4 ||    // walls
                segment_laserpoints[0].Attribute(LabelTag) == 3) // slannted walls
            wall_points.AddPoints(segment_laserpoints);
        /// store unknown points
        if (segment_laserpoints[0].Attribute(LabelTag) == 2 || //almost vertical points
                segment_laserpoints[0].Attribute(LabelTag) == 1 || // almost horizontal points
                segment_laserpoints[0].Attribute(LabelTag) == 0) // not_vetical and not_horizontal
            unknown_points.AddPoints(segment_laserpoints);
        /// store all labeled points
        outputpoints.AddPoints(segment_laserpoints);
    }

    /// Prune wall points above the floor and below the ceiling // this is not necessary, unless we want to have
    /// a rectangle wall shape for occlusion test
    if(crop_walls_abovefloor_and_belowceiling){
        DataBoundsLaser db_floor, db_ceiling;
        db_floor    = floor_points.DeriveDataBounds(0);
        db_ceiling  = ceiling_points.DeriveDataBounds(0);

        double floor_z_average, ceiling_z_average;
        if (db_floor.MinimumZIsSet() && db_floor.MaximumZIsSet() &&
            db_ceiling.MinimumZIsSet() && db_ceiling.MaximumZIsSet()){
            floor_z_average = (db_floor.Minimum().GetZ() + db_floor.Maximum().GetZ())/2;
            ceiling_z_average = (db_ceiling.Minimum().GetZ() + db_ceiling.Maximum().GetZ())/2;

            ceiling_z_average = 5.11; //1.70;  //1.35; //1.95;   //4.75; 2.90 ; -1.62; // 1.95;   // hardcoded because of estimation problem
            floor_z_average   = 1.64; //-1.75; //-0.85; //-0.75;  //0.0;  0.0 ;-4.29; // -0.75;  // hardcoded because of estimation problem

            LaserPoints::iterator wallpoint_it;
            LaserPoints wall_croped;
            wall_croped = wall_points;
            for (wallpoint_it = wall_points.begin(); wallpoint_it != wall_points.end(); wallpoint_it++){

                if (wallpoint_it->GetZ() > ceiling_z_average || wallpoint_it->GetZ() < floor_z_average){
                    wallpoint_it->SetAttribute(LabelTag, 8);  // wall points above the ceiling or lower than floor
                }
            }
            strcpy (str_root,root);
            wall_points.Write(strcat(str_root, "walls_cropped.laser"), false);
            wall_points.RemoveTaggedPoints(8, LabelTag);
        }
    }

    strcpy (str_root,root);
    segments_pairpoints.Write(strcat(str_root, "pairlines.objpts")), strcpy (str_root,root);
    segments_pairlines.Write(strcat(str_root, "pairlines.top"), false), strcpy (str_root,root);

    unknown_points.Write(strcat(str_root, "unknown_points.laser"), false), strcpy (str_root,root);
    floor_points.Write(strcat(str_root, "floor.laser"), false), strcpy (str_root,root);
    ceiling_points.Write(strcat(str_root, "ceiling.laser"), false), strcpy (str_root,root);
    wall_points.Write(strcat(str_root, "walls.laser"), false), strcpy (str_root,root);
    outputpoints.Write(strcat(str_root, "outputlaser.laser"), false);

    /// look into unknown segments for candidate walls
    if (relabeled_walls.size()){
        SetColor(GREEN);
        printf ("Relabeling pending walls ... \n");
        SetColor(GRAY);

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
        strcpy (str_root,root);
        relabeled_walls_lp.Write(strcat(str_root, "relabeled_walls.laser"), false);
    }



    strcpy (str_root,root);
    small_segments.Write(strcat(str_root, "small_segments.laser"), false);
    if (verbose) strcpy (str_root,root),
                notsegmented_points.Write(strcat(str_root, "notsegmented_points.laser"), false);

    fclose(statfile);
    fclose(statfile2);
    fclose(statWalls);
    fclose(segments_infofile);
    fclose(almost_horizon_infofile);
    fclose(almost_horizon_subset_infofile);


    strcpy (str_root,root);
    laserpoints.Write(strcat(str_root, "laserpoints.laser"), false);

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Total wall processing time: "<< duration/60 << "m" << '\n';

    /*
     * Occlusion test for opening detection
     * */
    if (do_occlusion_test){
        /// read trajectory points (scanner positions)
        char* trajFile;
        LaserPoints trajpoints;
        //trajFile = (char*) "D://test//indoor_reconstruction//data//trajectory3.laser";
         trajFile = (char *) "D:/test/zebrevo_ahmed/Revo_Trajectory.laser";
        trajpoints.Read(trajFile);
        strcpy (str_root,root);
        occlusion_test(laserpoints, wall_points, trajpoints, segment_buffers, 0.60, root);
    }

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Total processing time: "<< duration/60 << "m" << '\n';

}

