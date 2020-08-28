//
// Created by NikoohematS on 12-4-2018.
//

#include <iostream>
#include "InlineArguments.h"
#include <cstdlib>
#include <limits>
#include <LaserPoints.h>
#include "Buffers.h"
#include "indoor_reconstruction.h"
#include "post_processing.h"
#include "PlaneFittingResidual.h"
#include "Laservoxel.h"
#include "TrajectoryManipulation.h"
#include "space_partitioning.h"
#include "Directory_Processing.h"
#include "LaserPyramid.h"
#include "visualization_tools.h"
#include "Longest_Path_Graph.h"
#include "CreateAdjGraph.h"
#include "NavigationGraph.h"


void test_function( char * in, char * out) {

    //InlineArguments     *args = new InlineArguments(argc, argv);

    /// Check on required input files
/*    if (args->Contains("-usage") ||
        !args->Contains("-input_laser"))
        //!args->Contains("-method"))
    {
        if (!args->Contains("-usage")) printf("Error: missing programme option./n");
        PrintUsage();
        return EXIT_SUCCESS;
    }*/

/**********************************************************************************************************************/
/***************         INDOOR TOPOLOGY            *************/
    /// Call the main functions
    //LaserPoints lpoints, surfpoints;
    //segment2D(lpoints, 250);
    //LaserPoints traj_lp;
    //double time, percentage;
    //FilterSegments_ByTimeTag(lpoints, traj_lp, 150.0 , 10.0, 0.60);
    //FilterPoints_ByTimeTag(lpoints, traj_lp, 150);
    //char* root = (char*) "D://test//indoor_reconstruction//";
    //char* root = (char*) "E:/publication_data/indoor_changeDetection/process/out/";
    //GenerateWallPatches(lpoints, 0.70, 20.0, 0.5, root);

    char* file_lp;
    //file_lp = (char*) "D://test//indoorgraph//data//fireBr_Cloud_16mil_sub_1cm_3rdFloor_2mil_thinned_seg1_12cm.laser"; // gym_raw_thinned_455k.laser //
    //file_lp = (char*) "D://test//indoor_reconstruction//data//FB_sub1cm_seg12cm_refined_1770k_reflectremoved.laser";
    //file_lp = (char*) "E://BR_data//ZebR//cc_sub2mil_basement_crop_seg10cm.laser";
    //file_lp  = (char *) "E://BR_data//Diemen//process//block3_seg05cm_1-7mil_seg10cm_label0.laser" ; //tworooms_labeled0.laser"
    //file_lp  = (char *) "D:/test/zebrevo_ahmed/Revo_cloud_thinned_seg10cm.laser" ; //Revo_cloud_crop.laser" ; //
    //file_lp = (char*) "E:/Laser_data/ETH_dataset/penthouse/penthouse_ConnectComp_01_segGV_1-4mil_crop.laser";//sample_collection_furniture.laser";  //slantedwalls.laser" ;  //penthouse_seg8cm_oneblock.laser" ;
    //file_lp = (char*) "E:/Laser_data/Kadaster/test_data/level2_kadaster_crop2_newsegmentation.laser";
    //file_lp = (char*) "D:/test/sims3d/test_seri2/ceiling_floor_test.laser";       //firebrigade_3rdfloor.laser";
   // file_lp = (char*) "E:/publication_data/indoor_changeDetection/data/BR_ZebRevo_seg10.laser";
    //file_lp = (char*) "E:/publication_data/FB_dataset/data/data/3rdfloor/original_data/3rdfloor_final_buffer_segmentation_3m.laser";
    //file_lp = (char*) "E:/publication_data/FB_dataset/test/2glassrooms.laser";
    //file_lp = (char*) "E:/publication_data/FB_dataset/data/data/3rdfloor/occlusion_test_goodresult/modified_laserpoints.laser";
    //file_lp = (char*) "E:/publication_data/FB_dataset/data/data/1stfloor/temp/wall19_38.laser";
    //file_lp = (char*) "E:/publication_data/FB_dataset/images/wall_doors_floor_ceiling.laser";
    //file_lp = (char*) "E:/publication_data/Delft_zebrevo/data/delft_2989k_1stfloor_seg20cm.laser";
   // file_lp = (char*) "E:/publication_data/Navvis/Kadaster/data/cadastre_4m_2cm_cc_seg10cm.laser";  // //indoor_top_test
   // file_lp = (char*) "../data/indoor_top_input.laser";  ///server path
   // file_lp = (char*) "E:/publication_data/Haaksbergen/data/second_floor_1537k_seg10.laser"; //first_floor_thinned_seg0810.laser" ; //first_floor_thinned.laser";
   //file_lp = (char*) "E:/publication_data/Haaksbergen/test/bar_cropresegmented.laser";
   // LaserPoints lpoints;
   // lpoints.Read(file_lp);
    //traj_lp.Read("E://BR_data//ZebR//traj_basement_crop.laser");
    //traj_lp.Read("E://BR_data//Diemen//process//traj_diemen1_s_rec.laser");
    //surfpoints.Read("E://BR_data//Diemen//process//out//outputlaser.laser");

    /*remove points with label -1 from backpack system*/
/*    LaserPoints lp_labeled0;
    for (auto p : lpoints){
        if(p.Attribute(LabelTag) != -1) {
            p.SetAttribute(LabelTag, 0);
            lp_labeled0.push_back(p);
        }
    }
    lp_labeled0.Write("E://BR_data//Diemen//process//block3_seg05cm_1-7mil_seg10cm_label0.laser", false);*/

    //auto *root = (char*) "D:/test/zebrevo_ahmed/out/";
    //auto *root = (char*) "E:/Laser_data/ETH_dataset/penthouse/out/" ;
    //auto *root = (char*) "E:/Laser_data/Kadaster/test_data/out/";
    //auto *root = (char*) "D:/test/sims3d/test_seri2/out/";
    //auto *root = (char*) "E:/publication_data/FB_dataset/process/out/indoortopology/";
   // auto *root = (char*) "E:/publication_data/FB_dataset/images/out/";
    //auto *root = (char*) "E:/publication_data/Delft_zebrevo/indoortopology/out/";
    //auto *root = (char*) "E:/publication_data/Navvis/Kadaster/indoor_topology/";
    //auto *root = (char*) "../output/indoortopology/"; ///server path
/*    auto *root = (char*) "E:/publication_data/Haaksbergen/test/out/";
    indoorTopology(file_lp, 2000, root, false);*/

    //LaserPoints refined_lp;
    //refined_lp = segment_refinement(lpoints, 2, 0.50);
    //refined_lp.Write("D://test//buffer_Kada//FB_sub1cm_seg8cm_665k_refined2_reflectremoved.laser", false);

    //LaserPoints traj_lp, lp;
    //DepthMap(traj_lp, lp);

    //int label;
    //WallAccuracy(label);
    //RemoveDuplicatePoints(lpoints, lp);
/**********************************************************************************************************************/
    /*****   OCLUSION_TEST  ******/
    //Buffers bfs;
    //traj_lp.Read("D:/test/zebrevo_ahmed/Revo_Trajectory.laser");
    //surfpoints.Read("D:/test/zebrevo_ahmed/out/indoortopology_nobuffer/walls.laser");
    //occlusion_test(lpoints, surfpoints, traj_lp, bfs , 0.12, root);
    //LaserPoints lp, traj_lp, wall_lp;
    //lp.Read("E:/publication_data/FB_dataset/data/data/3rdfloor/occlusion_test_goodresult/modified_laserpoints.laser");
    //traj_lp.Read("E:/publication_data/FB_dataset/data/data/3rdfloor/original_data/traj_3rdfloor.laser");
    //wall_lp.Read("E:/publication_data/FB_dataset/data/data/3rdfloor/occlusion_test_goodresult/surfaces_points.laser");
   // lp.Read("../data/laser_points1.laser"); // server path
   // traj_lp.Read("../data/trajectory1.laser");
   // wall_lp.Read("../data/walls1.laser");

   // lp.RemoveAttribute (SegmentNumberTag); /// when the generalization is applied
   // lp.RemoveAttribute (LabelTag); /// for Backpack data
    //auto *root = (char*) "E:/publication_data/FB_dataset/process/out/occlusion_test/";
    // auto *root = (char*) "../output/occlusion_test/";
    //occlusion_test2 (lp, wall_lp, traj_lp, 0.10, root);

/***********************************************************************************************************************/
    /*  edit occlusion_test result, remove excess points above each ceiling */

/*    LaserPoints lp_occ_result, lp_ceilings;
    lp_occ_result.Read("E:/publication_data/FB_dataset/data/data/1stfloor/temp/occlusion_points.laser");
    lp_ceilings.Read("E:/publication_data/FB_dataset/data/data/1stfloor/temp/ceiling_cropZ.laser");
    auto *root = (char*) "E:/publication_data/FB_dataset/process/out/occlusion_test/";
    filter_occlusion_result_by_ceiling (lp_occ_result, lp_ceilings, root);*/

/**********************************************************************************************************************/
    /// post_processing test
    //auto * output = (char*) "D:/test/sims3d/out/";
    //auto * output = (char*) "E:/Laser_data/ETH_dataset/penthouse/out/";
    //LaserPoints lp_segmented;
    //lp_segmented.Read ("D:/test/sims3d/data/post_processing/wall_crop2.laser");   //walls_simplified  //
    //lp_segmented.Read("E:/Laser_data/ETH_dataset/penthouse/intersection_problem2.laser") ; //sample_collection.laser");
    //intersect_segments (lp_segmented, 0.10, 0.10, 1.0, output);

/***********************************************************************************************************************/
    /// external function: visualization of planes
/*    LaserPoints lp_segments;
    lp_segments.Read("E:/Laser_data/ETH_dataset/penthouse/out/ceiling.laser") ;
    /// make laserpoints from each segment
    vector<LaserPoints> segments;
    segments = PartitionLpByTag (lp_segments, SegmentNumberTag);
    ObjectPoints plane_corners;
    LineTopology plane_edges;
    LineTopologies planes_edges;

    bool verbose=false;
    for(auto &segment : segments) {
        segment.Write ("E:/Laser_data/ETH_dataset/penthouse/out/segment.laser", false);
        VisulizePlane3D (segment, 0.10, plane_corners, plane_edges, true);
        planes_edges.push_back (plane_edges);
    }
    plane_corners.Write("E:/Laser_data/ETH_dataset/penthouse/out/plane_corners.objpts");
    planes_edges.Write("E:/Laser_data/ETH_dataset/penthouse/out/plane_edges.top", false);*/

/***********************************************************************************************************************/
    /*   Visualize a single segment */
/*    LaserPoints segment_lp;
    segment_lp.Read("E:/publication_data/FB_dataset/test/door.laser");
    ObjectPoints plane_corners;
    LineTopology plane_edge;
    LineTopologies plane_edges;
    VisulizePlane3D (segment_lp, 0.10, plane_corners, plane_edge, true);
    plane_edges.push_back (plane_edge);
    plane_corners.Write("E:/publication_data/FB_dataset/process/out/occlusion_test/plane_corners.objpts");
    plane_edges.Write("E:/publication_data/FB_dataset/process/out/occlusion_test/plane_edges.top", false);*/

/***********************************************************************************************************************/
    /*         Partition points by segment or another attribute and store them */
/*    LaserPoints segmented_lp;
    segmented_lp.Read("E:/publication_data/Navvis/Kadaster/indoor_topology/walls_reseg.laser");
    vector<LaserPoints> segments_vec;
    char *output;
    output = (char*) "";
    segments_vec = PartitionLpByTag (segmented_lp, SegmentNumberTag);

    /// sort by size if required
    //sort(segments_vec.begin (), segments_vec.end (), compare_lp_size );

    /// remove small segment if required
    LaserPoints reduced_lp;
    for (auto &s : segments_vec){
        if (s.size () > 2000) reduced_lp.AddPoints (s);
    }
    reduced_lp.Write("E:/publication_data/Navvis/Kadaster/indoor_topology/walls_reduced.laser", false);*/


/***********************************************************************************************************************/
    /* testing boost geometry library for intersecting minimum rectangles
     * testing EnclosingRectangle and ScaleRectangle functions
     * */
/*    LaserPoints segments_lp;
    segments_lp.Read ("E:/Laser_data/ETH_dataset/penthouse/intersection_problem.laser");    //floor_ceil.laser //intersection_problem

    vector<LaserPoints> segments;
    segments = PartitionLpByTag (segments_lp, SegmentNumberTag);

    vector <ObjectPoints> corners_vec ;
    vector <LineTopology> edges_vec;
    ObjectPoints corners;
    LineTopology edges;
    LineTopologies rectangles;
    int next_number; /// the next_number is the number after the last number in the corners file
    for (auto &segment : segments){
        /// derive databounds and derivetin for calculating the EnclosingRectangle
        DataBoundsLaser db = segment.DeriveDataBounds (0);
        segment.DeriveTIN ();
        /// EnclosingRectangle function handles the numbering of the corners for many segments
        /// corners and edges of the rectangle are the output of the function
        segment.EnclosingRectangle(0.2, corners, edges);
        /// get the numbering of corners for updating the Z value of the corners
        if (corners.empty()) next_number = 4; /// later next_number-4 =0
        else next_number = (corners.end() - 1)->Number() + 1;
        /// add z values to the 4 new corners
        for (int i=next_number-4; i < next_number ; i++){
            corners[i].Z() = (db.Maximum().GetZ() + db.Minimum ().GetZ ()) / 2;
        }
*//*        edges.push_back(PointNumber(next_number)); // close the polygon // clockwise
        edges.MakeClockWise(corners);*//*
        edges.SetAttribute (BuildingPartNumberTag, 1);
        rectangles.push_back (edges);

        corners_vec.push_back (corners);
        edges_vec.push_back (edges);
    }
    corners.Write("E:/Laser_data/ETH_dataset/penthouse/out/min_rectangles.objpts");
    rectangles.Write("E:/Laser_data/ETH_dataset/penthouse/out/min_rectangles.top", false);

    vector <ObjectPoints> new_corners_vec ;
    vector <LineTopology> new_edges_vec;
    ObjectPoints scaled_corners;
    LineTopologies new_rectangles;
    for (int i =0; i < corners_vec.size (); i++){

        scaled_corners = ScaleRectangle (corners_vec[i], edges_vec[i], 0.98);
        edges_vec[i].SetAttribute (BuildingPartNumberTag, 3);
        new_rectangles.push_back (edges_vec[i]);

        new_corners_vec.push_back (scaled_corners);
        new_edges_vec.push_back (edges_vec[i]);
    }*/
/*    new_corners_vec[0].Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles0.objpts");
    LineTopologies new_rectangles0; new_rectangles0.push_back (new_edges_vec[0]);
    new_rectangles0.Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles0.top", false);

    new_corners_vec[1].Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles1.objpts");
    LineTopologies new_rectangles1; new_rectangles1.push_back (new_edges_vec[1]);
    new_rectangles1.Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles1.top", false);

    new_corners_vec[2].Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles2.objpts");
    LineTopologies new_rectangles2; new_rectangles2.push_back (new_edges_vec[2]);
    new_rectangles2.Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles2.top", false);

    new_corners_vec[3].Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles3.objpts");
    LineTopologies new_rectangles3; new_rectangles3.push_back (new_edges_vec[3]);
    new_rectangles3.Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles3.top", false);

    new_corners_vec[4].Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles4.objpts");
    LineTopologies new_rectangles4; new_rectangles4.push_back (new_edges_vec[4]);
    new_rectangles4.Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles4.top", false);*/

    //scaled_corners.Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles.objpts");
    /// topology file is the same as before scale
    //new_rectangles.Write("E:/Laser_data/ETH_dataset/penthouse/out/scaled_rectangles.top", false);

/***********************************************************************************************************************/
    /*
     * test boost geometry for making polygons from objectpoints and intersect them
     * */
    //ObjectPoints corners1, corners2;
    //std::deque<boost_polygon> poly_out;
    //poly_out = intersect_polygons (corners_vec[0], corners_vec[1]);

    /*        {
        //get the databounds from the ls TO visualize
        DataBoundsLaser db = segment.DeriveDataBounds (0);
        LineTopology rect_topology;
        ObjectPoints corners;
        LineTopologies bounding_box;
        segment.DeriveTIN();
        //db = ls.DeriveDataBounds(0);
        segment.EnclosingRectangle(0.1, corners, rect_topology);

        int next_number; /// the next_number is the number after the last number in the corners file
        if (corners.empty()) next_number = 4; /// later next_number-4 =0
        else next_number = (corners.end() - 1)->Number() + 1;

        /// add z values to 4 corners to make upper rectangle
        for (int i=next_number -4; i < next_number ; i++){
            corners[i].Z() = db.Maximum().GetZ(); // upper rectangle
        }

        ObjectPoint obj_pnt;
        /// constructing below recangle
        for (int i=0; i < 4; i++){

            obj_pnt.X() = corners[next_number -4 + i].X();
            obj_pnt.Y() = corners[next_number -4 + i].Y();
            obj_pnt.Z() = db.Minimum().GetZ(); // below rectangle
            obj_pnt.Number() = next_number + i;
            corners.push_back(obj_pnt);
            rect_topology.push_back(PointNumber(next_number + i));
        }
        rect_topology.push_back(PointNumber(next_number)); // close the polygon // clockwise
        rect_topology.MakeCounterClockWise(corners);

        /// write minium enclosing rectangle to the disk
        corners.Write("E:/Laser_data/ETH_dataset/penthouse/out/min_rectangle.objpts");
        bounding_box.insert(bounding_box.end(), rect_topology);
        bounding_box.Write("E:/Laser_data/ETH_dataset/penthouse/out/min_rectangle.top", false);
    }*/

/***********************************************************************************************************************/
    /*
     * function planefittingresidual test
     * */
    //LaserPoints lp;
    //lp.Read("D:/test/reflection_removal/reflection_data/wall_glass_reflection_balcon_2.laser");   //wall_reflection_balcon_1.laser"
    //lp.Read("D:/test/indoor_reconstruction/final_segmentation.laser");
    //root = (char*) "D:/test/indoor_reconstruction/";
    //planefittingresidual_test (lp, root);

/***********************************************************************************************************************/
/*
 *  mirror reflected points to their correct location
 * */
/*   LaserPoints glass_lp, reflection_lp, trajectory_lp;
    glass_lp.Read("D:/test/reflection_removal/reflection_data/glass.laser");
    reflection_lp.Read("D:/test/reflection_removal/reflection_data/reflection.laser");
    trajectory_lp.Read("D:/test/reflection_removal/reflection_data/traj_wall_reflection_balcon_1.laser");
    out = (char*) "D:/test/reflection_removal/reflection_data/";*/
    //mirror_PointsToPlane (glass_lp, reflection_lp);

    /// visulize the reflection lines for few points
    //mirror_PointsToPlane_test_function (glass_lp, reflection_lp, trajectory_lp, out);

/***********************************************************************************************************************/
    /*
     * Partition Points By Trajecotry
     * */

    /* first Resegment trajectory by Time*/
    /*   segment trajectory by time */
/*    LaserPoints segmented_traj, trajectory;
    //trajectory.Read("E:/publication_data/FB_dataset/data/trajectory/trajectory_final.laser");
    trajectory.Read("E:/publication_data/Delft_zebrevo/trajectory/traj_1st_ground_resegmented.laser");
    segmented_traj = Segment_Trajectory_ByTime (trajectory, 1.0);
    segmented_traj.Write("E:/publication_data/Delft_zebrevo/trajectory/traj_1st_ground_resegmentedbytime.laser", false);*/

/*    LaserPoints lp1, lp2, lp3, lp4, lp5, lp6, lp7, all_lp, trajectory_resegmented;
    //lp1.Read("E:/publication_data/FB_dataset/data/raw_data/all_convertedBy_MappingLib_001.laser");       //cc_time_Z_partitioned.laser");
    //lp2.Read("E:/publication_data/FB_dataset/data/raw_data/all_convertedBy_MappingLib_002.laser");
    //lp3.Read("E:/publication_data/FB_dataset/data/raw_data/all_convertedBy_MappingLib_003.laser");
    //lp4.Read("E:/publication_data/FB_dataset/data/raw_data/all_convertedBy_MappingLib_004.laser");
    //lp5.Read("E:/publication_data/FB_dataset/data/raw_data/all_convertedBy_MappingLib_005.laser");
    //lp6.Read("E:/publication_data/FB_dataset/data/raw_data/all_convertedBy_MappingLib_006.laser");
    lp7.Read("E:/publication_data/FB_dataset/data/raw_data/all_convertedBy_MappingLib_007.laser");

*//*    vector<LaserPoints> lp_v;
    lp_v.push_back (lp1);
    lp_v.push_back (lp2);
    lp_v.push_back (lp3);
    lp_v.push_back (lp4);
    lp_v.push_back (lp5);
    lp_v.push_back (lp6);
    lp_v.push_back (lp7);*//*

    trajectory_resegmented.Read("E:/publication_data/FB_dataset/data/trajectory/final_traj_segmentation/trajectory_final_notSorted.laser");

    //LaserPoints lp_level2;
    //lp_level2 = lp.SelectTagValue (LabelTag, 2);

    vector<LaserPoints> partitions;
     auto * output = (char*) "E:/publication_data/FB_dataset/process/out";
     int cnt =0;
     //for (auto &lp: lp_v){
     //    cnt++;
         lp7.SetAttribute (PlaneNumberTag, 7); /// for naming the file: e.g. laserpoints_x
         partitions = PartitionPointsByTrajecotry (lp7, trajectory_resegmented, output, false); /// we use output files
     //}*/

/*    LaserPoints traj_segmented, lp;
    vector<LaserPoints> partitions;
    auto * output = (char*) "E:/publication_data/Delft_zebrevo/partitions/out/";
    traj_segmented.Read("E:/publication_data/Delft_zebrevo/trajectory/traj_test.laser");
    lp.Read("E:/publication_data/Delft_zebrevo/data/delft_1853k_cc.laser");
    partitions = PartitionPointsByTrajecotry (lp, traj_segmented, output, true);*/

    
/***********************************************************************************************************************/

    /* directory processing */
/*    std::vector<std::pair<string, string>> files_v;
    files_v = RecursiveDirFileNameAndPath ("E:/publication_data/FB_dataset/data/data/3rdfloor");
    int file_size=0;
    LaserPoints all_lp;
    for (auto &file : files_v){
        LaserPoints lp;
        string file_path;
        file_path = file.first; // filepath

        lp.Read (file_path.c_str ());
        printf ("file: %s .... file size: %d /n", file.second.c_str (), lp.size ());
        file_size += lp.size ();
        all_lp.AddPoints (lp);
    }
    printf("Total file size: %d /n", file_size);

    LaserPoints  corrected_lp;
    for (auto &p : all_lp){
        if (p.X () == 0.0 && p.Y () == 0.0) {
            continue; /// goes to the next p
        }
        corrected_lp.push_back (p);
    }
    corrected_lp.Write ("E:/publication_data/FB_dataset/data/data/3rdfloor_all.laser", false);*/

/*    LaserPoints lp;
    lp.Read("E://publication_data//FB_dataset//data//data//3rdfloor//indoortopology_nobuffer_final/laserpoints.laser");
    LaserPoints_info (lp, 500);*/

/***********************************************************************************************************************/
    /*
     * Generate voxels from the points and trajectory for the histogram
     * */
/*    LaserPoints points1, points, laser_points, trajectory;
    //points1.Read("E:/Laser_data/FireBrigadeBuilding_13102015/seri2_05022016/laser_converted/all_convertedBy_MappingLib_001.laser");
    //points.Read ("E:/Laser_data/FireBrigadeBuilding_13102015/seri2_05022016/laser_converted/all_convertedBy_MappingLib_003.laser");
    points.Read ("D:/test/laserpoints_partitioning/cc_time_clean.laser");
    trajectory.Read ("D:/test/laserpoints_partitioning/trajectory_fb_50_60seg_modified.laser");
    LaserVoxel point_vox(points, 0.10);
    LaserVoxel traj_vox(trajectory, 0.10);
    LaserPoints points_vox, trajs_vox;
    points_vox = point_vox.export_voxel_centres (1);
    trajs_vox = traj_vox.export_voxel_centres (5);
    //points_vox.Write("D:/test/laserpoints_partitioning/points_vox.laser", false);
    //trajs_vox.Write("D:/test/laserpoints_partitioning/trajs_vox.laser", false);
    LaserPoints points_vox_occupied, trajs_vox_occupied;
    points_vox_occupied = points_vox.SelectTagValue (LabelTag, 11); /// label=11 are occupied voxels
    trajs_vox_occupied = trajs_vox.SelectTagValue (LabelTag, 11); /// label=11 are occupied voxels

    points_vox_occupied.Write("D:/test/laserpoints_partitioning/vox_points_occupied.laser", false);
    trajs_vox_occupied.Write("D:/test/laserpoints_partitioning/vox_trajs_occupied.laser", false);

    char* out_root;
    char str_root[500];
    out_root = (char*) "D:/test/laserpoints_partitioning/";

    FILE *traj_points_file = nullptr;
    FILE *laser_points_file= nullptr;
    if(out_root){

        /// open a file for traj points
        strcpy(str_root, out_root);
        traj_points_file = fopen(strcat(str_root, "voxel_traj_points.txt"), "w");
        fprintf(traj_points_file, "X, Y, Z, Time_Tag /n");

        /// open a file for points
        strcpy(str_root, out_root);
        laser_points_file = fopen(strcat(str_root, "voxel_laser_points.txt"), "w");
        fprintf(laser_points_file, "X, Y, Z, Time_Tag /n");
    }

    /// write the traj.txt to the disk
    if(out_root){
        for(auto &traj_p : trajs_vox_occupied){
            double t;
            t = 0.0; //traj_p.DoubleAttribute (TimeTag)
            fprintf(traj_points_file, "%.2f,%.2f,%.2f,%lf /n",
                    traj_p.X (), traj_p.Y(), traj_p.Z (), t );
        }
    }

    /// write the traj.txt to the disk
    for (auto &p : points_vox_occupied){
        if(out_root){
            double t;
            t = 0.0;
            fprintf(laser_points_file, "%.2f,%.2f,%.2f,%lf /n",
                    p.X (), p.Y(), p.Z (), 0.0);
        }
    }

    fclose(traj_points_file);
    fclose(laser_points_file);*/

/***********************************************************************************************************************/
    /// test histogram
/*    LaserPoints lp;
    lp.Read ("D:/test/laserpoints_partitioning/cc_time.laser");
    Histogram hist;
    lp.AddToHistogram (hist, 3); /// doesn't work
    hist.Write ("D:/test/laserpoints_partitioning/hist.png");*/

/***********************************************************************************************************************/

    /// test flipping normal toward the trajectory location
    //LaserPoints lp, scanners_p;
    //lp.Read("D:/test/normal_estimation/laserpoints.laser");
    //scanners_p.Read("D:/test/normal_estimation/traj_wall_reflection_balcon_1.laser");
    //Normal_Flip (lp, scanners_p, 10);
    //lp.Write("D:/test/normal_estimation/laserpoints.laser", false);

/**********************************************************************************************************************/
/*
 * Label points to vertical and horizontal for histogram
 *
 * LaserPoints lp, labeled_lp;
    lp.Read("D:/test/laserpoints_partitioning/cc_time.laser");
    //scanners_p.Read("D:/test/normal_estimation/traj_wall_reflection_balcon_1.laser");
    labeled_lp = LabelPoints_To_Ver_Horizon (lp, 10.0);
    labeled_lp.Write("D:/test/laserpoints_partitioning/cc_time_vert_horizon.laser", false);*/

/**********************************************************************************************************************/
/*
 *   partition points to building levels by a given Z_peaks extracted from the histogram
 *   each two consecutive Z in the Z-peaks should represent the floor and ceiling of each level
 * */

/*    LaserPoints lp, labeled_lp;
    lp.Read("D:/test/laserpoints_partitioning/cc_time.laser");

    /// initialize the vector of z-peaks extracted from the local maxima of a histogram
    static const double arr[] = {-7.62, -4.90, -4.31, -1.16, -0.80, 1.95};
    vector<double> z_peaks(arr, arr + sizeof(arr) / sizeof(arr[0]) );

    PartitionPoints_By_Zpeaks (lp, z_peaks, 0.10);
    lp.Write("D:/test/laserpoints_partitioning/cc_time_Z_partitioned.laser", false);*/

/**********************************************************************************************************************/

    /*
     * filter backpack points
     * */
/*    LaserPoints lp, filtered_lp;
    lp.Read("E:/BR_data/Diemen/process/block3_seg05cm_1-7mil_seg10cm.laser");
    FilterPointsByTag (lp, LabelTag, -1, filtered_lp);
    filtered_lp.Write ("E:/BR_data/Diemen/process/block3_seg05cm_1-7mil_seg10cm_filtered.laser", false);

    /// print points to a ascii file
    FILE *output_points;
    output_points = fopen ("E:/BR_data/Diemen/process/block3_seg05cm_1-7mil_seg10cm_filtered.txt", "w");
    fprintf(output_points, "X, Y, Z, T, SN, L /n");
    for( auto &p : filtered_lp){
        fprintf(output_points, "%.3f,%.3f,%.3f,%lf,%d,%d /n ",
                p.X (), p.Y(), p.Z (), p.DoubleAttribute (TimeTag),
                p.Attribute (SegmentNumberTag), p.Attribute (LabelTag));
    }

    fclose(output_points);*/

/**********************************************************************************************************************/

/*    char* ascii_file;
    ascii_file = (char*) "E:/Laser_data/publication_data/Navvis/Kadaster/trace_15_merged2.txt";

    char* lp_outfile;
    lp_outfile = (char*) "E:/Laser_data/publication_data/Navvis/Kadaster/trace_15_merged2_trimT.laser";
    LaserPoints lp_out;

    read_custom_ascii (ascii_file, lp_out, lp_outfile);*/

/***********************************************************************************************************************/

/*    LaserPoints trajectory;
    trajectory.Read("D:/test/laserpoints_partitioning/traj_stair_1_left_modified.laser");

    ObjectPoints vertices;
    LineTopologies traj_polyline;
    TrajToPolyline (trajectory, 100, traj_polyline, vertices);

    traj_polyline.Write("D:/test/laserpoints_partitioning/traj_stair_1_left_modified.top", false);
    vertices.Write("D:/test/laserpoints_partitioning/traj_stair_1_left_modified.objpts");*/

/***********************************************************************************************************************/
/*         space partitioning file
      intersection between space partitions and the trajectory */
/*    LaserPoints lp_partitions;
    //lp_partitions.Read("E:/publication_data/FB_dataset/data/data/3rdfloor/space_partitioning/result/vox_centers_dilate_unoccupied_resegment_final.laser");
    lp_partitions.Read("E:/publication_data/BR_backpack/space_partitions/spaces/spaces_resegmented_final.laser");
    LaserPoints lp_traj;
    //lp_traj.Read ("E:/publication_data/FB_dataset/data/trajectory/final_traj_segmentation/traj_3rdfloor.laser");
    lp_traj.Read("E:/publication_data/BR_backpack/data/traj_diemen1_s_rec.laser");
    LaserPoints valid_partitions;
    //projected_lp = Project_Spacepartions_To_2D (lp);
    valid_partitions = Intersect_Spacepartitions_Trajectory (lp_partitions, lp_traj, 0.20);
    valid_partitions.Write("E:/publication_data/BR_backpack/space_partitions/spaces//valid_partitions.laser", false);*/

/**********************************************************************************************************************/
    /*  Filter reflected points by timetag */
    //LaserPoints lpoints;
    //lpoints.Read ("E:/publication_data/FB_dataset/data/data/3rdfloor/3rdfloor_sub2cm_seg_mergedbuffer.laser");
/*    lpoints.Read("E:/publication_data/FB_dataset/process/out/indoortopology2_nobuffer/walls.laser");

    LaserPoints traj_lp;
    traj_lp.Read ("E:/publication_data/FB_dataset/data/trajectory/final_traj_segmentation/traj_3rdfloor.laser");

    char *output;
    //output = (char*) "E:/publication_data/FB_dataset/data/data/3rdfloor/";
    output = (char*) "E:/publication_data/FB_dataset/process/out/indoortopology2_nobuffer/reflection_correction/";

    FilterSegments_ByTimeTag(lpoints, traj_lp, 150.0 , 10.0, 0.60, output);*/

/***********************************************************************************************************************/

    /*   other temporary functions */

    /*   correct the laserpoints with 0.0 value point */
/*    LaserPoints lp;
    lp.Read ("E:/publication_data/FB_dataset/process/out/laserpoints_4_partition_23.laser");

    LaserPoints  corrected_lp;
    for (auto &p : lp){
        if (p.X () == 0.0 && p.Y () == 0.0) {
            continue; /// goes to the next p
        }
        corrected_lp.push_back (p);
    }
    corrected_lp.Write ("E:/publication_data/FB_dataset/process/out/laserpoints_4_partition_23_corrected.laser", false);*/

    /*  laser files info */

/***********************************************************************************************************************/

    /***   Segmentation: surface growing for batch processing ***/
/*    std::vector<std::pair<string, string>> files_v;
    files_v = RecursiveDirFileNameAndPath ("E:/publication_data/FB_dataset/data/data/3rdfloor");

    vector<LaserPoints> lp_v;
    for (auto &file : files_v){
        LaserPoints lp;
        string file_path;
        file_path = file.first; // filepath
        lp.Read (file_path.c_str ());

        LaserPoints lp_seg;
        Segmentation_SurfaceGrowing (lp, lp_seg, 0.10, 0.12);
        lp_v.push_back (lp_seg);
    }*/

/*    LaserPoints lp;
    lp.Read (in);
    printf("laser points size: %d /n", lp.size ());
    lp.Write(out, false);*/
/***********************************************************************************************************************/
    /***   Segmentation: surface growing for single file ***/
/*    SegmentationParameters *seg_parameter;
    seg_parameter = new SegmentationParameters;
    /// segmenting laserpoints
    //seg_parameter -> MaxDistanceInComponent()  = 0.3;
    seg_parameter -> SeedNeighbourhoodRadius() = 1.0;
    seg_parameter -> MaxDistanceSeedPlane()    = 0.10; // MaX Distance to the Plane
    seg_parameter -> GrowingRadius()           = 1.0;
    seg_parameter -> MaxDistanceSurface()      = 0.12;
    seg_parameter -> SurfacesCompete () = true;

    /// if segmentation crashes set the compatibility of generated exe to windows7
    printf("segmentation process... /n ");
    LaserPoints lp;
    //lp.Read("E:/publication_data/FB_dataset/data/data/3rdfloor_4m_sorted_sub2cm.laser");
    lp.Read("../data/segmentation_input.laser"); // server path
    lp.SurfaceGrowing(*seg_parameter);
    vector<int> segment_nu_v;
    segment_nu_v = lp.AttributeValues (SegmentNumberTag);
    printf("Number of segments: %d /n", segment_nu_v.size ());

    lp.Write("../output/segmentation/segmented_lp.laser", false);*/
    //lp.Write("E:/publication_data/FB_dataset/data/data/3rdfloor_4m_sorted_sub2cm_segmented.laser", false);

    //LaserPoints lp;
    //lp.Read("E:/publication_data/FB_dataset/data/data/3rdfloor_14m_segmented.laser");
    //std::sort(lp.begin (), lp.end (), compare_p_time);
    //lp.Write ("E:/publication_data/FB_dataset/data/data/3rdfloor_14m_segmented_sorted.laser", false);

/**********************************************************************************************************************/

    /***    Generate wall patches (BUFFER) ****/
/*    LaserPoints laserpoints;
    //laserpoints.Read ("E:/publication_data/FB_dataset/process/out/buffer/walls_buffer/final_segmentation.laser");
    laserpoints.Read ("../data/generalization_input.laser");  // server path
    Buffers   segment_buffers;
    char* out_root = (char*) "../output/buffer/";
    //char* out_root = (char*) "E:/publication_data/FB_dataset/process/out/buffer/";
    segment_buffers = GenerateWallPatches(laserpoints, 0.60, 10.0, 0.40, out_root);

    LaserPoints buffer_points;
    buffer_points = segment_buffers.Buffers_laserpoints();
    buffer_points.SetAttribute(LabelTag, 0);
    laserpoints = buffer_points; // this initializes pointnumbers (because of pointnumbers bug)*/
    //laserpoints.Write("3rdfloor_thin_3m_mergebuffers.laser", false);
/***********************************************************************************************************************/
    /***  change detection operations ***/
/*    LaserPoints lp, changes_lp;
    lp.Read ("E:/publication_data/indoor_changeDetection/process/backpack_intersection_final/outputlaser.laser");
    changes_lp.Read ("E:/publication_data/indoor_changeDetection/data/BR_backpack_changes.laser");

    LaserPoints relabled_changes_lp;
    relabled_changes_lp = relabel_changed_points (changes_lp, lp);
    relabled_changes_lp.Write("E:/publication_data/indoor_changeDetection/process/backpack_intersection_final/relabled_changes_lp.laser", false);*/

/**********************************************************************************************************************/

     /*merge segmented laser points and renumber segments*/
    char * input_directory;
    input_directory = (char*) "E:/publication_data/modeling/tudelft/modeling_walls/merge_files";
    char * out_file;
    out_file = (char*) "E:/publication_data/modeling/tudelft/modeling_walls/wall_cl_renumbered.laser";
    LaserPoints lp_master;
    lp_master.Read("E:/publication_data/modeling/tudelft/modeling_walls/files_for_modeling/extended_wallwallfloor.laser");

    merge_lp_segmented (lp_master, input_directory, out_file);


/***********************************************************************************************************************/

    /*   renumber segment numbers  from 1 to n */
/*    LaserPoints lp_segmented, lp_renumbered;
    lp_segmented.Read("E:/publication_data/BR_backpack/door_detection/19doors_goodresult/top_door_lp_segmented_manaual_removedduplicate.laser");
    vector<LaserPoints> lp_segments;
    lp_segments = PartitionLpByTag (lp_segmented, SegmentNumberTag);
    int segment_nr =0;
    for(auto &segment : lp_segments){
        segment.SetAttribute (SegmentNumberTag, segment_nr++);
        lp_renumbered.AddPoints (segment);
    }

    lp_renumbered.Write("E:/publication_data/BR_backpack/door_detection/19doors_goodresult/top_door_lp_renumbered_manaual_removedduplicate.laser", false);*/


/***********************************************************************************************************************/
/*    LaserPoints lp1, lp2;
    lp1.Read("E:/publication_data/FB_dataset/data/raw_data/partitions/laserpoints_4_partition_23.laser");
    lp2.Read("E:/publication_data/FB_dataset/data/raw_data/partitions/laserpoints_5_partition_23.laser");

    lp1.AddPoints (lp2);
    lp1.Write ("E:/publication_data/FB_dataset/data/data/1stfloor/1stfloor.laser", false);
    printf ("# of points: %d /n", lp1.size ());*/

/**********************************************************************************************************************/
    /* trim and recover the time_tag for conversion and operation in cloudcompare*/

/*    LaserPoints lp;
    lp.Read("E:/publication_data/Delft_zebrevo/data/delft_sub5cm_3279k_cc_1stfloorTtrimmed.laser");

    char *ascii_out = (char*) "E:/publication_data/Delft_zebrevo/data/delft_3279k_1stfloor.txt";
    Trim_doubleTag (lp, TimeTag, 1481000000.0, ascii_out, true);    /// zeb1:1444729700.0 zebrevo: 1481000000.0
    //add_to_doubleTag (lp, TimeTag, 1444729700.0);
    lp.Write("E:/publication_data/Delft_zebrevo/data/delft_3279k_1stfloor.laser", false);
    printf("# of points: %d /n", lp.size ());*/


/***********************************************************************************************************************/
    /* trim ascii file time_tag for cloudcompare operations*/

/*    std::string ascii_in = "E:/publication_data/Delft_zebrevo/data/delft_sub2cm_16m_cc.txt";
    std::string ascii_out = "E:/publication_data/Delft_zebrevo/data/delft_sub2cm_16m_cc_Ttrimmed.txt";

    Trim_doubleTag (ascii_in, 3, 1481000000.0, ascii_out);*/

/**********************************************************************************************************************/

    /* crop the points of floor and ceiling for a given average height */
/*    LaserPoints lp, lp_croped;
    lp.Read("E:/publication_data/FB_dataset/data/data/1stfloor/1stfloor_goodresult2/walls.laser");
    double z_value = -0.85; /// z_value normally is the floor of the next level
    for(auto &p : lp) {
        if(p.GetZ () < z_value) lp_croped.push_back (p);
    }
    lp_croped.Write("E:/publication_data/FB_dataset/data/data/1stfloor/1stfloor_goodresult2/manual_labeled/walls_cropZ.laser", false);*/

/***********************************************************************************************************************/
/*    LaserPoints lp;
    lp.Read("E:/publication_data/BR_backpack/door_detection/19doors_goodresult/top_door_lp_segmented_manaual.laser");
    //int dominant_segment_nr, count;
    //dominant_segment_nr = lp.MostFrequentAttributeValue (SegmentNumberTag, count);
    //lp.RemoveTaggedPoints (dominant_segment_nr, SegmentNumberTag);

    lp.RemoveDoublePoints (false);
    lp.Write("E:/publication_data/BR_backpack/door_detection/19doors_goodresult/top_door_lp_segmented_removeduplicate.laser", false);*/

/***********************************************************************************************************************/
/*  detect false openings */

/*    LaserPoints lp_in, lp_out;
    lp_in.Read("E:/publication_data/FB_dataset/data/data/1stfloor/space_partitioning/occ_op.laser");
    lp_out = detect_false_openings (lp_in, 0.70);

    lp_out.Write("E:/publication_data/FB_dataset/data/data/1stfloor/space_partitioning/wall_false_openings.laser", false);*/

/**********************************************************************************************************************/

/*  Merge walls/ segments */

    //auto *root = (char*) "E:/publication_data/modeling/data/tudelft/modeling_walls/out/";
/*    auto *root = (char*) "E:/publication_data/modeling/data/Haaksbergen/test/";
    LaserPoints lp;
    //lp.Read("E:/publication_data/modeling/data/test/merge_delft_walls.laser"); //merge_delft_crop //merge_delft_corridor //merge_delft_walls
    //lp.Read("E:/publication_data/modeling/data/Haaksbergen/first_floor/walls_plNr_corrected_final.laser");
    lp.Read("E:/publication_data/modeling/data/Haaksbergen/test/topology_correction_test.laser");
    //lp.Read("E:/publication_data/modeling/data/tudelft/modeling_walls/walls_true_manual_corrected.laser");
    // Mergesurfaces (lp, planes' dist, planes' angle, segments' dist, segment_min_size, root, second_planesDist,
    // bool to calculate middle_plane, bool to calculate weighted_plane, verbose)
    double planes_dist = 0.5; /// maximum wall thickness
    double planes_angle = 5.0;
    double segments_dist = 1.0; /// a door way normally is around 0.80 to 1.10 m.
    int min_segSize = 500;
    double planes_dist_second = 1.2;
    Mergesurfaces (lp, planes_dist, planes_angle, segments_dist, min_segSize, root, 1.2, true, false, true);*/

/**********************************************************************************************************************/

/*   STAIR MODeling*/

/*    LaserPoints lp;
    lp.Read("E:/publication_data/Delft_zebrevo/stairs/3steps.laser");
    auto *root = (char*) "E:/publication_data/Delft_zebrevo/stairs/out/";

     Stair_Modeling (lp, 100, 0.02, 25.0, root, false);*/

/**********************************************************************************************************************/
/// search for the stair case ramp

    LaserPoints lp, lp_out;
    //lp.Read("E:/publication_data/Delft_zebrevo/stairs/stairs_people2.laser"); //stair_ramp  //stairs_people2
    //lp.Read("E:/publication_data/modeling/data/Haaksbergen/stairs/process/stair_3m_seg15cm.laser");
   // auto *root = (char*) "E:/publication_data/modeling/data/Haaksbergen/stairs/out/";
/*    lp_out = Coarse_Stair_Search (lp, 1500, 25.0, 50.0, 0.15, root, false);
    char rootStr[500];
    strcpy (rootStr,root);
    lp_out.Write(strcat(rootStr, "lp_out.laser"), false);*/
    //file_lp = (char*) "E:/publication_data/Delft_zebrevo/stairs/selected_ramps4_seg4cm_2.laser";
   // file_lp = (char*) "E:/publication_data/modeling/data/Haaksbergen/stairs/process/lp_out_segmented2_3cm_3degrees.laser";
   // lp.Read(file_lp);
    ///creating the adjacency graph
   // CreateAdjGraph (lp, 50, 0.1, root, false);  // threshold 0.01 m is enough.

/****************************************/

/* Navigation graph*/

/*    char rootStr[500];
    auto *root = (char*) "E:/publication_data/Navigation_graph/Haaksbergen/out/";
    strcpy (rootStr,root);

    LaserPoints segmented_lp;
    file_lp = (char*) "E:/publication_data/Navigation_graph/Haaksbergen/floor_wall.laser";
    segmented_lp.Read(file_lp);
    ObjectPoints corners_and_lpoints, centers, all_points, corners;
    LineTopologies polygons;
    TIN tin;
    tin = Create_Constrained_Delaunay (segmented_lp, 0, polygons, corners_and_lpoints, centers, all_points, root, 0.40, false);
    strcpy (rootStr,root);
    polygons.Write(strcat(rootStr,"polygons.top"), false);
    strcpy (rootStr, root);
    centers.Write(strcat(rootStr, "centers.objpts"));
    strcpy (rootStr,root);
    all_points.Write(strcat(rootStr, "all_points.objpts")); /// this is printing the lines and points
    strcpy (rootStr,root);
    tin.Write(strcat(rootStr, "TIN.tin"));

    FILE *file;
    strcpy (rootStr,root);
    file = fopen(strcat(rootStr,"tin.vrml"), "w");
    TINEdges tinEdges;
    tinEdges = TINEdges(tin);
    tinEdges.VRML_Write (file);*/

/***************************** Triangle Centroids ************************/

    //char rootStr[500];
    //auto *root = (char*) "E:/publication_data/Navigation_graph/Haaksbergen/out/";
    //TIN tin;
    //tin.Read ((char*) "E:/publication_data/Navigation_graph/Haaksbergen/out/TIN.tin");
    //ObjectPoints tin_points;
    //tin_points.Read ((char*) "E:/publication_data/Navigation_graph/Haaksbergen/out/map_tin_points.objpts");
/*    LaserPoints trinagle_centers;
    trinagle_centers = Create_Triangle_Centroids (tin, corners_and_lpoints);
    //trinagle_centers = Create_Triangle_Medians_Intersection (tin, tin_points, true);
    strcpy (rootStr,root);
    trinagle_centers.Write(strcat(rootStr, "trinagle_centroids.laser"), false);*/

/*************  label points near edges ****************/
/*    LaserPoints floor_points, lp_out;
    floor_points.Read("E:/publication_data/Navigation_graph/Haaksbergen/floor_points.laser");
    ObjectPoints vertices;
    LineTopologies  new_polygons, polygons_in;
    polygons_in.Read("E:/publication_data/Navigation_graph/Haaksbergen/out/polygons_in.top");
    vertices.Read ("E:/publication_data/Navigation_graph/Haaksbergen/out/vertices.objpts");
    lp_out = Label_points_near_polygonEdges (floor_points, polygons_in, vertices, 0.10, true);
    lp_out.Write("E:/publication_data/Navigation_graph/Haaksbergen/out/lp_out.laser", false);*/

/*   Constrained TIN test */
//// a simple test for triangulation
/*    LineTopologies polygons;
    ObjectPoints corners;
    ObjectPoints centers;
    char rootStr[500];
    auto *root = (char*) "E:/publication_data/Navigation_graph/Haaksbergen/out/";
    Triangulate_test (corners, polygons, centers, root);
    strcpy (rootStr,root);
    corners.Write(strcat(rootStr, "corners.objpts"));
    strcpy (rootStr,root);
    polygons.Write(strcat(rootStr, "polygons.top"), false);*/

   /************* label points inside the hole polygons ******/
/*        LaserPoints floor_points, lp_out;
     floor_points.Read("E:/publication_data/Navigation_graph/Haaksbergen/out/floor_points.laser");
     ObjectPoints vertices;
     LineTopologies polygons_in;
     polygons_in.Read("E:/publication_data/Navigation_graph/Haaksbergen/out/polygons_in.top");
     vertices.Read ("E:/publication_data/Navigation_graph/Haaksbergen/out/vertices_in.objpts");
     lp_out = Label_points_inside_polygons (floor_points, polygons_in, vertices);
     lp_out.Write("E:/publication_data/Navigation_graph/Haaksbergen/out/lp_out.laser", false);*/

/********************************************************************/

/* Making the graph from centroids */

/*    char rootStr[500];
    auto *root = (char*) "E:/publication_data/Navigation_graph/Haaksbergen/out/";
    TIN tin;
    tin.Read ((char*) "E:/publication_data/Navigation_graph/Haaksbergen/out/TIN.tin");
    ObjectPoints tin_points;
    tin_points.Read ((char*) "E:/publication_data/Navigation_graph/Haaksbergen/out/map_tin_points.objpts");

    FILE *file;
    strcpy (rootStr,root);
    file = fopen(strcat(rootStr,"TIN.vrml"), "w");
    tin.VRML_Write (file);

    std::pair <ObjectPoints, LineTopologies> centroid_Graph;
    LaserPoints centroids_lp;
    centroids_lp = Create_TIN_Centroids_Graph (tin, tin_points, centroid_Graph);
    strcpy (rootStr,root);
    centroids_lp.Write(strcat(rootStr, "triangle_centroids.laser"), false);

    LineTopologies edges;
    ObjectPoints nodes;
    nodes = centroid_Graph.first;
    edges = centroid_Graph.second;

    strcpy (rootStr,root);
    edges.Write(strcat(rootStr,"edges.top"), false);
    strcpy (rootStr, root);
    nodes.Write(strcat(rootStr, "nodes.objpts"));*/



} /// no code after here








