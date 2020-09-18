#include <iostream>
#include "InlineArguments.h"
#include <cstdlib>
#include <limits>
#include <map>
#include <LaserPoints.h>
#include "Plane.h"
#include "../utils/utils.h"
#include "../visualization_tools/visualization_tools.h"
#include "ModelingInteriorWalls.h"

using namespace std;

int main() {

    /* TODO: next week
     * Done: slanted walls during merge_surface can be enforced to be vertical so it generates more consistent surfaces
     * TO be done: run merge surface with Force_Verticality flag
     * Extend walls to SLABS
     * 
     * */


    char str_root[500];
    //char *root = (char*)  "E:/BR_data/Diemen/process/out/door_detection/";
    //char *root = (char*) "D:/test/morph2/out/";
   // char *root = (char*) "E:/publication_data/BR_backpack/door_detection/19doors_goodresult/";
    //char *root = (char*) "E:/publication_data/Delft_zebrevo/door_detection/out/doors_good/tall_door/";
    //char *root = (char*) "E:/publication_data/modeling/data/test/out/";
    //char *root = (char*) "../output/space_partitioning/"; ///server path output
    //strcpy (str_root,root); // initialize the str_root with root string

    LaserPoints lp;
    char* laserFile;
    //laserFile = (char*) "D://test//visualisation//tiltedplane2.laser";
    //laserFile = (char*) "D://test//visualisation//tiltedplane2.laser";
    //laserFile = (char*) "D:/test/sims3d/data/3rdfloor_601040_final/2nd_iteration_walls/modified_manually/floor.laser";
    //laserFile = (char*) "E:/publication_data/FB_dataset/data/data/1stfloor/door_detection/result/top_door_lp_segmented.laser";
    //laserFile = (char*) "E:/publication_data/modeling/data/test/tudelft_wall_floor_crop_small2.laser";
    //lp.Read(laserFile);
    //VisualizePlanes(lp);

    ObjectPoints polygoncorners;
    LineTopologies polygonlines;
   // double height=0.0;
   // double min_z, max_z;
    //min_z = max_z =0.0;
   // max_z = -5.25;//-0.68; //floor //2.10; ceiling
   // min_z = -7.40;//-0.86;
   // Minimum3DBox(polygoncorners, polygonlines, lp, min_z, max_z, height);

/*    strcpy (str_root,root);
    polygoncorners.Write(strcat(str_root, "doors.objpts"));
    strcpy (str_root,root);
    polygonlines.Write(strcat(str_root, "doors.top"), false);*/
    //polygoncorners.Write("D:/test/sims3d/data/3rdfloor_601040_final/2nd_iteration_walls/modified_manually/floor_corners.objpts");
    //polygonlines.Write("D:/test/sims3d/data/3rdfloor_601040_final/2nd_iteration_walls/modified_manually/floor_edges.top", false);

    //lp.Read ("D:/test/visualization/3dbox.slanted_walls.laser");
/*
    vector <int> seg_nums;
    seg_nums = lp.AttributeValues (SegmentNumberTag);
    LineTopologies segment_polygons;
    ObjectPoints points_obj;
    for (auto seg : seg_nums){
        LaserPoints segment_lp;
        segment_lp = lp.SelectTagValue (SegmentNumberTag, seg);
        LineTopology lines_top;
        VisulizePlane3D(segment_lp, 0.10, points_obj, lines_top, false);
    }*/

    /*visualize supervoxel polygons*/
    ObjectPoints vertices;
    LineTopologies polygons;
    LaserPoints supervoxels;
    //laserFile = (char*)  "E:/Laser_data/Stanford_annotated/supervoxel/office_25.laser"; //"D:/test/supervoxels.laser";
    //supervoxels.Read(laserFile);
    //GenerateConvexPolygon(vertices, polygons, supervoxels, LabelTag);
   // vertices.Write("E:/Laser_data/Stanford_annotated/supervoxel/polygons/office_25.objpts");
   // polygons.Write("E:/Laser_data/Stanford_annotated/supervoxel/polygons/office_25.top");


    /*Export 3D minimum rectangle to OFF files*/
/*    lp.Read("/home/shayan/Drive_D/data/test/tilted_walls.laser");
    //char* offFile_out;


    //root = (char*) "E:/publication_data/modeling/data/Haaksbergen/door_detection/";
    char* root = (char*) "/home/shayan/Drive_D/data/test/";
   // root = (char*) "E:/publication_data/Delft_zebrevo/stairs/out/";
    //double h=1.90; // just for doors
    double height = 0.0; //1.90;
    double max_z = 0.0; //9.00 ; //-5.25; //0.0;  //1.82   //2.10;
    double min_z = 0.0; //6.56 ; //-1.75;     //-7.40; //-1.78;  //-1.84 //-0.86;
    ObjectPoints corners;
    LineTopologies lines;
    int min_seg_size = 0;
   // Min3DBox_OFF(corners, lines, lp, min_seg_size, offFile_out, min_z, max_z, height); // to generate off files

    double max_edge_dist = 0.20;
    //VisualizePlanes (lp, min_seg_size, max_edge_dist, corners, lines, false); //// to generate horizontal rectangles
    VisualizePlanes (lp, min_seg_size, corners, lines, root, false);   /// to generate slanted and vertical rectnagles

   // double angle_threshold = 10.0;
   // fit_3D_minimumRectangle (lp, angle_threshold, max_edge_dist, min_seg_size, corners, lines);

    strcpy (str_root,root);
    corners.Write(strcat(str_root, "_corners.objpts"));
    strcpy (str_root,root);
    lines.Write(strcat(str_root, "_lines.top"), false);*/

    //lp.Read("/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/indoor_topology/out1/edited/walls4.laser");
    //    LaserPoints_info(lp, 100);

/*********************************************************************************************************************/
/*  Enforce verticality OR Horizontality */
/*    lp.Read("/home/shayan/Drive_D/data/test/tilted_wall1.laser");
    Enforce_verticality(lp, 1);*/

/***********************************************************************************************************************/
    /*  offset_polygon functions */   /* EnclosingRectangle_Rotation3D */

/*    ObjectPoints L_vertices, R_vertices, corners;
    LineTopology L_polygon, R_polygon;
    double offset_dist= 0.05;

    char* root;
    root = (char*) "/home/shayan/Drive_D/data/test/";

    LaserPoints segment;
    LineTopology edges;
    segment.Read("/home/shayan/Drive_D/data/test/projected_merged_segments0.laser");  //slanted_wall2
    EnclosingRectangle_Rotation3D (segment, 0.10, corners, edges, root); /// operates on one segment

    strcpy (str_root,root);
    corners.Write(strcat(str_root, "corners.objpts"));
    LineTopologies edgess;
    edgess.push_back (edges);
    strcpy (str_root,root);
    edgess.Write(strcat(str_root, "edges.top"), false);*/

/*    offset_polygon (corners, offset_dist, L_vertices, R_vertices, L_polygon, R_polygon);

    LineTopologies Le_polygon, Ri_polygon;
    Le_polygon.push_back (L_polygon);
    Ri_polygon.push_back (R_polygon);

    /// left polygon
    strcpy (str_root,root);
    L_vertices.Write(strcat(str_root, "L_vertices.objpts"));
    strcpy (str_root,root);
    Le_polygon.Write(strcat(str_root, "L_polygon.top"), false);

    /// right polygon
    strcpy (str_root,root);
    R_vertices.Write(strcat(str_root, "R_vertices.objpts"));
    strcpy (str_root,root);
    Ri_polygon.Write(strcat(str_root, "R_polygon.top"), false);*/

/***********************************************************************************************************************/
    /* Min3DRectangle_to_3DBox */ /* Minimum3DBox */ /* Min3DBox_OFF */
/*    double offset_dist= 0.05;

    LaserPoints segment;
    ObjectPoints corners;
    LineTopology edges;
    segment.Read("E:/publication_data/modeling/data/test/slanted_wall2.laser");
    EnclosingRectangle_Rotation3D (segment, 0.10, corners, edges, root);

    LineTopology threeDbox_edges;
    LineTopologies threeDbox_edgess,threeDbox_faces;
    ObjectPoints threeDbox_vertices;
    /// this can be an arbitrary oriented box
    //Min3DRectangle_to_3DBox (corners, offset_dist, threeDbox_vertices, threeDbox_edges); /// with vertices and edges
    Min3DRectangle_to_3DFaces (corners, offset_dist, threeDbox_vertices, threeDbox_faces); /// with vertices and faces

    //Minimum3DBox (threeDbox_vertices, threeDbox_edgess, segment); // this is a vertical box
    //char *offFile_out;
    //strcpy (str_root,root);
    //offFile_out = strcat (str_root, "threeDbox.off");
    //Min3DBox_OFF (threeDbox_vertices, threeDbox_faces, segment, 500, offFile_out, 0.0, 0.0, 0.0);

    //threeDbox_edgess.push_back (threeDbox_edges);

    /// 3dbox
    strcpy (str_root,root);
    threeDbox_vertices.Write(strcat(str_root, "threeDbox_verticesOff.objpts"));
    strcpy (str_root,root);
    //threeDbox_edgess.Write(strcat(str_root, "threeDbox_edges.top"), false);
    threeDbox_faces.Write(strcat(str_root, "threeDbox_faces.top"), false);*/

/**********************************************************************************************************************/

    /*  planes manipulation */ // offset, parallel, projection, ...
   // LaserPoints segment1, segment2, result_segment;
    //segment1.Read("E:/publication_data/modeling/data/test/seg69.laser");
    //segment2.Read("E:/publication_data/modeling/data/test/seg74.laser");
    //result_segment = test_planes_calculation (segment1, segment2);
    //result_segment.Write("E:/publication_data/modeling/data/test/out/result_projected_segment.laser", false);
   // Plane offset;
   // offset = offset_plane ((segment1.FitPlane(segment1[0].SegmentNumber ())), -1.0);
   // result_segment = Project_points_to_Plane (segment1, offset);
   // result_segment.Write("E:/publication_data/modeling/data/test/out/result_projected_segment.laser", false);
/*    Plane pl1, pl2;
    pl1 = segment1.FitPlane (segment1[0].SegmentNumber ());
    pl2 = segment2.FitPlane (segment2[0].SegmentNumber ());
    LaserPoints s1, s2;
    extend_segments_to_intersection (segment1, segment2, pl1, pl2, 1.0,  s1, s2);
    s1.AddPoints (s2);
    s1.Write(strcat(str_root, "s1.laser"), false);*/

    //EnclosingPolygon_test (lp, root);
   // LineTopology contour_edges;
   // ObjectPoints contour_points;
   // LineTopologies contour3Ds_edges;
    //EnclosingRectangle3D (lp, 0.20, contour_points, contour_edges, root); // incomplete function
    //lp.DeriveContour3D (contour_points, contour_edges, 0.20);
    //EnclosingRectangle_with_Rotation (lp, 0.20, contour_points, contour_edges, root, true); // incomplete function
    //EnclosingRectangle_Rotation3D(lp, 0.20, contour_points, contour_edges, root, true);
    //VisulizePlane3D (lp, 0.10, contour_points, contour_edges, false);
    /// write to the file

/*    contour3Ds_edges.push_back (contour_edges);
    strcpy(str_root, root);
    contour_points.Write(strcat(str_root, "rectangle3D_points.objpts"));
    strcpy(str_root, root);
    contour3Ds_edges.Write(strcat(str_root, "rectangle3D_edges.top"), false);*/

/**********************************************************************************************************************/
/* MergeSurfaces() from utils if necessary to create more consistent walls and planes,
 * then use the planes for ModelingInteriorWalls() */
    // Mergesurfaces (lp, planes' dist, planes' angle, segments' dist, segment_min_size, root, second_planesDist,
    // bool to calculate middle_plane, bool to calculate weighted_plane, verbose)
/*    auto *root = (char*) "/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/modeling/out/";
    lp.Read("/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/modeling/walls5.laser");
    double planes_dist = 0.5; /// maximum wall thickness
    double planes_angle = 5.0;
    double verticality_angle_threshold = 30.0;
    double segments_dist = 1.0; /// a door way normally is around 0.80 to 1.10 m.
    int min_segSize = 100;
    double planes_dist_second = 0.80;
    Mergesurfaces (lp, planes_dist, planes_angle, segments_dist, min_segSize, root, planes_dist_second,
            true, false,
            true, verticality_angle_threshold, false);*/

/**********************************************************************************************************************/
/* WALL to Ceiling/FLoor Extension */
/* first we extend merged_walls to the floor and ceiling separately, without any extension distance, it does it based on
 * the closest objects. Then in the next step we merge them in one file and we extend them using ModelingInteriorWalls, which
 * extend them two by two using an extension distance
 * This step is not necessary if all walls are connected to the ceiling and floors.
 */

/*    LaserPoints merged_walls;
    LaserPoints slabs; // ceiling OR Floor
    Planes wall_planes;

    //merged_walls.Read("E:/publication_data/modeling/data/test/merged_walls.laser");
    //ceil.Read("E:/publication_data/modeling/data/test/ceiling.laser");
    //wall_planes.Read((char*) "E:/publication_data/modeling/data/test/merged_planes_walls.planes");
    merged_walls.Read("/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/modeling/out/extendedWall_Fl_laserpoints.laser");
    slabs.Read("/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/modeling/ceiling_master.laser");
    wall_planes.Read((char*) "/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/modeling/out/merged_planes.planes");
    char *root = (char*) "/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/modeling/out/";
    Extend_Walls_Ceiling (merged_walls, wall_planes, slabs, 100, root, true);*/

/**********************************************************************************************************************/
/* MODELING WALLS STEP1 */  /* extension of undershoots */
    /*  Modeling walls  */ /* extension of undershoot segments and generating the minimum_rectangle */
    //laserFile = (char*) "E:/publication_data/modeling/data/test/tudelft_wall_floor_.laser"; //intersection_slanted_4segments.laser";
    //laserFile = (char*)    "E:/publication_data/modeling/tudelft/modeling_walls/out/wall_fl_cl_renumbered.laser";
    //laserFile = (char*)    "E:/publication_data/modeling/tudelft/modeling_walls/files_for_modeling/wall_cl_renumbered.laser";
    //laserFile = (char*) "E:/publication_data/modeling/data/test/off_test.laser";
    //laserFile = (char*) "E:/publication_data/modeling/data/Haaksbergen/test/merged_segments_modified.laser";
/*    laserFile = (char*) "/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/modeling/verified/extendedWall_Fl_Cl_modified2.laser";
    lp.Read(laserFile);
    Planes planes;
    planes.Read( (char*) "/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/modeling/verified/merged_planes.planes");
    //planes.Read((char*) "E:/publication_data/modeling/data/Haaksbergen/test/merged_planes.planes");
    //planes.Read ((char*) "E:/publication_data/modeling/tudelft/modeling_walls/files_for_modeling/merged_planes_walls.planes");
    double max_intersection_dist = 0.5; // 3.0 meters for TUDelf wall-wall extension
    char *root = (char*) "/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/modeling/out/";
    /////this generates extended.laser and rectangles.objpts and thickness of walls in residuals attribute.
    ModelingInteriorWalls (lp, planes, max_intersection_dist, 100, root, false);*/

/***********************************************************************************************************************/
/* MODELING WALLS STEP2 */
    /* modeling volumetric walls (generate boxes from rectangles/polygons) */
    /// the input of this function is the output of MergSurfaces() from indoor_geometry_reconstruction tool OR from
    /// or from ModelingInteriorWalls() from previous step.
    //laserFile = (char*)    "E:/publication_data/modeling/data/Haaksbergen/second_floor/walls_flcl_extended.laser";
    laserFile = (char*) "/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/modeling/out/extended_laserpoints.laser";
    lp.Read(laserFile);
    std::map <int, double> offset_dist_map;
    /// input values should be the expected wall thickness. Inputs divided by 2 will be offset_distance
    offset_dist_map = Generate_offset_dist_map (lp, 0.20, 0.40); /// this uses the residual attribute of the segments.

    /// the input of this function is the output of ModelingInteriorWalls()
    ObjectPoints walls_rectangle_vertices;
    LineTopologies walls_edges;
    double fix_offset_dist=0.10;  /// this is multiple by 2 is the thickness of the wall
    walls_rectangle_vertices.Read(
            "/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/modeling/out/rectangle_corners.objpts");
    walls_edges.Read(
            "/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/modeling/out/rectangle_edges.top");
    root = (char*) "/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/modeling/out/";
    GenerateVolumetricWalls (walls_rectangle_vertices, walls_edges, offset_dist_map, fix_offset_dist, root, false);

/*  MODELING WALLS STEP3
     convert the results to OFF format*/
    ObjectPoints walls_vertices;
    LineTopologies walls_faces;
    walls_vertices.Read("/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/modeling/out/walls_vertices.objpts");
    walls_faces.Read("/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/modeling/out/walls_faces.top");
    root = (char*) "/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/modeling/out/";
    LineTopologies_to_OFFBoxes (walls_vertices, walls_faces, root);

/**********************************************************************************************************************/

/* Convert linetopolgies to OFF. */
/*    ObjectPoints walls_vertices;
    LineTopologies walls_faces;
    walls_vertices.Read("E:/publication_data/Haaksbergen/test/out/pairlines.objpts");
    walls_faces.Read("E:/publication_data/Haaksbergen/test/out/pairlines.top");
    char *root = (char*) "E:/publication_data/Haaksbergen/test/out";
    LineTopologies_to_OFF (walls_vertices, walls_faces, root);*/


/**********************************************************************************************************************/
    /* VisulizePlane3D */
/*    laserFile = (char*) "E:/publication_data/modeling/data/test/floor_ramps.laser";
    lp.Read(laserFile);
    LineTopology edges_top;
    ObjectPoints corners;
    VisulizePlane3D (lp, 0.10, corners, edges_top, false);

    LineTopologies edges;
    edges.push_back (edges_top);
    strcpy(str_root, root);
    corners.Write(strcat(str_root, "corners.objpts"));
    strcpy(str_root, root);
    edges.Write(strcat (str_root, "edges.top"), false);*/

/**********************************************************************************************************************/

    /*  Extend_Segments_to_Intersection */
/*    laserFile = (char*) "E:/publication_data/modeling/data/test/segments_0_44.laser"; //.laser"; //segments_17_44 //intersection_problem
    lp.Read(laserFile);
    /// we assume lp has two segments
    vector <int> segment_numbers = lp.AttributeValues(SegmentNumberTag);
    LaserPoints seg1, seg2;
    seg1 = lp.SelectTagValue(SegmentNumberTag, segment_numbers[0]);
    seg2 = lp.SelectTagValue(SegmentNumberTag, segment_numbers[1]);
    Plane plane1, plane2;
    plane1 = seg1.FitPlane (segment_numbers[0]);
    plane2 = seg2.FitPlane (segment_numbers[1]);
    /// extend two segments
    LaserPoints segment1, segment2, lp_new;
    Extend_Segments_to_Intersection (seg1, seg2, plane1, plane2, 1.0, segment1, segment2);
    lp_new.AddPoints (segment1);
    lp_new.AddPoints (segment2);
    lp_new.Write(strcat (str_root, "lp_new.laser"), false);*/




/**********************************************************************************************************************/

}