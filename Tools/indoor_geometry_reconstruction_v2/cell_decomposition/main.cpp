#include <iostream>
#include <LaserPoints.h>
#include <map>
#include <Plane.h>
#include "intersecting_clippers.h"
#include "face_selection.h"
#include "../utils/utils.h"
#include "../visualization_tools/visualization_tools.h"
#include "sample_laserpoints.h"
#include "main_pipeline.h"
#include <boost/filesystem.hpp>
#include "alpha_shape.h"
//#include "planar_segmentation.h" (use planar_ransac.h which is newer)
#include "planar_ransac.h"


LaserPoints read_ascii(char *ascii_file);
void room2cellsdecomposition(char *input_ascii, std::string data_dir);
void cell_decomposition_wrapper(std::string data_dir);
void occup_grid(LaserPoints lp, double vox_size, char *out_dir);


int main() {

/*    char* inputlaser = (char*) "/mnt/DataPartition/CGI_UT/cell_decomposition/lp.laser";
    char* root_dir = (char*) "/mnt/DataPartition/CGI_UT/cell_decomposition/out";
    intersect_planes(inputlaser, root_dir, 100);*/

    char* root_dir = (char*) "/mnt/DataPartition/CGI_UT/cell_decomposition/out/";
    char str_root[500];
    strcpy (str_root, root_dir);

    /// create and read the bounding box of the data
    //char* inputlaser = (char*) "/mnt/DataPartition/CGI_UT/cell_decomposition/four_segments.laser";
    //LaserPoints lp;
    //lp.Read(inputlaser);
    /// create a cuboid with 6 faces around the input data as the bound planes
    ObjectPoints cube_global_vertices; // output
    LineTopologies cube_global_faces; // output
    ///Planes cube_planes; // output
    ///std::map<int, Positions3D> planes_pos_map; // output
    ///double enlarge_size = 0.5; //meter, currently has no effect
    ///cube_planes = bounding_cube(root_dir, lp, cube_global_vertices, cube_global_faces, enlarge_size , planes_pos_map); //dont use this one use "void bounding_cube()"
    //bounding_cube(root_dir, lp, 1.1, cube_global_vertices, cube_global_faces);
    //cube_global_vertices.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/cube_global_vertices_scaled.objpts");
    //cube_global_faces.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/cube_global_edges_scaled.top", false);
    LineTopologies box3d_faces;
    ObjectPoints box3d_vertices;
    //box3d_faces.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/s3dis/cube_global_edges.top", false);
    //box3d_vertices.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/s3dis/cube_global_vertices.objpts");
    box3d_faces.Read("/mnt/DataPartition/threed_modeling/threed_boxes/Area_1_conferenceRoom_2_3dbox.top", false);
    box3d_vertices.Read("/mnt/DataPartition/threed_modeling/threed_boxes/Area_1_conferenceRoom_2_3dbox.objpts");

    /// a test on linetopology tags
//    LineTopologies box3d_faces_tagged;
//    int cnt=0;
//    for(auto &face:box3d_faces){
//        face.SetAttribute(LineNumberTag, cnt); //# Attributes Tag number = 16 //we use this to assing the valid/invalid to the face
//        face.SetAttribute(LineLabelTag, cnt+100); // # Attributes Tag number=0 //we use this to assing the segment number to the face
//        //face.SetAttribute(LineTopologyTag(GeometryTag), cnt);
//        face.Number()=cnt;
//        box3d_faces_tagged.push_back(face);
//        cout << face.Attribute(LineNumberTag) << endl;
//        cnt++;
//    }
//    box3d_faces_tagged.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/s3dis/cube_global_edges_lnumtag.top", false);


    /// test Intersect_Planes_3DBoxFaces
//    LaserPoints segments;
//    LineTopologies polygons_edges;
//    ObjectPoints polygons_vertices;
//    int min_seg_size = 500;
//    segments.Read("/mnt/DataPartition/threed_modeling/laserpoints/laser/Area_1_conferenceRoom_2_seg.laser");
//    Intersect_Planes_3DBoxFaces(segments, min_seg_size, box3d_faces, box3d_vertices,
//                                polygons_edges, polygons_vertices, true, true);
//    polygons_vertices.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_vertices.objpts");
//    polygons_edges.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_edges.top", false);

    /// test SplitPolygons3DByPlane3D()
//    LaserPoints segment;
//    ObjectPoints new_polys_v; LineTopologies new_polys_e;
    //segment.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/segment60.laser"); // for the spliting plane
    /// input polygons are created by Intersect_Planes_3DBoxFaces()
//    SplitPolygons3DByPlane3D(polygons_vertices, polygons_edges, segment, new_polys_v, new_polys_e);
//    new_polys_v.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_decomposed_vertices.objpts");
//    new_polys_e.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_decomposed_edges.top", false);

    //////////// CELL DECOMPOSITION ///////////////////
    /// Use this with output of Intersect_Planes_3DBoxFaces() for CELL DECOMPOSITION
//    ObjectPoints new_polys_v; LineTopologies new_polys_e;
//    //segments.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/segments.laser"); // for the spliting planes
//    ///input polygons are created by Intersect_Planes_3DBoxFaces()
//    SplitPolygons3DByPlanes3D(polygons_vertices, polygons_edges, segments, min_seg_size, new_polys_v, new_polys_e, true);
//    new_polys_v.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_decomposed_vertices.objpts");
//    new_polys_e.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_decomposed_edges.top", false);


    /// test associating points to faces
    /// USE this with the results of cell decomposition: SplitPolygons3DByPlanes3D()
//    LineTopologies all_faces, valid_faces, invalid_faces;
//    LaserPoints updated_labels;
//    int min_seg_size = 500;
//    segments.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/room_segmented.laser");
//    new_polys_e.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/s3dis/out_cells/polygon_new_edges.top", false);
//    new_polys_v.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/s3dis/out_cells/polygon_new_vertices.objpts");
//    //updated_labels = associatePointsToFace3D(segments, min_seg_size, 0.12, 0.5, 50, new_polys_v, new_polys_e, valid_polygons);
//    updated_labels = associatePointsToFace3D_withTag(segments, min_seg_size, 0.08, 0.5, 1000, new_polys_v,
//                                                     new_polys_e, valid_faces, invalid_faces, all_faces); // s3dis: 500, 0.08, 0.5, 500,
//    updated_labels.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/updated_labels.laser", false);
//    new_polys_v.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_vertices.objpts"); // vertices are the same
//    valid_faces.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/valid_faces.top", false);
//    invalid_faces.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/invalid_faces.top", false);
//    all_faces.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/all_faces.top", false);

    /// test plane intersection with the bbox of the data
//    Plane plane;
//    LineTopologies polygon_edges;
//    ObjectPoints polygon_vertices;
//    LaserPoints segment;
//    segment.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/slanted_segment.laser");
//    int seg_num = segment[0].SegmentNumber();
//    plane = segment.FitPlane(seg_num);
//    plane.Number() = seg_num;
//    cout << "plane number: " << plane.Number() << endl;
//    if(Intersect_Plane_3DBoxFaces(box3d_faces, box3d_vertices, plane, seg_num,
//                                  polygon_edges, polygon_vertices, true)){
//        polygon_vertices.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_vertices.objpts");
//        polygon_edges.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_edges.top", false);
//    }

    /// test for reordering and renumbering the topology
    //LineTopologies edges;
    //edges.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_edges_tmp2.top", false);
    //ReOrderTopology(edges);
//    LineTopologies faces, new_faces;
//    ObjectPoints vertices, new_vertices;
//    faces.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_edges_tmppppp.top", false);
//    vertices.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_vertices_tmppppp.objpts");
//    RenumberTopology_and_Vertices(faces[0], vertices, new_faces, new_vertices);
//    new_vertices.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/new_vertices.objpts");
//    new_faces.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/new_faces.top", false);

    /// efficient ransac (defiend in planar_segmentation.h)
//    //char *input_ascii = "/mnt/DataPartition/threed_modeling/input_data/room1_crop_spacedel.txt";
//    char *input_ascii = "/mnt/DataPartition/threed_modeling/input_data/Area_1_conferenceRoom_2_sub.xyz";
//    char *outdir = "/mnt/DataPartition/threed_modeling/dump";
//    //Efficient_ransac::Parameters parameters;
//    LaserPoints laserpoints, lp_seg_out;
//    //laserpoints.Read("/mnt/DataPartition/threed_modeling/Area_1_conferenceRoom_2.laser");
//    double  probability     = 0.05 ;    // Set probability to miss the largest primitive at each iteration.
//    int     min_points      = 100  ;    // Detect shapes with at least n-min points.
//    double  epsilon         = 0.02 ;    // Set maximum Euclidean distance between a point and a shape.(increase this to get thicker planar shapes)
//    double  cluster_epsilon = 0.08;     //0.08 ;  // Set maximum Euclidean distance between points to be clustered.
//    double  normal_thresh   = 0.087;    // Set maximum normal deviation.// 0.9 < dot(surface_normal, point_normal);
//    int     nb_neighbors    = 20   ;
//    bool    estimate_normals = True ;
//    Efficient_ransac::Parameters ransac_parameters;
//    LaserPoints lp_segmented;
//    set_parameters(ransac_parameters, probability, min_points, epsilon, cluster_epsilon, normal_thresh); // parameters for S3DIS
//    efficient_RANSAC_with_point_access (input_ascii, outdir, ransac_parameters, nb_neighbors, lp_segmented, estimate_normals);
//    //efficient_RANSAC_with_point_access (laserpoints, outdir, ransac_parameters, nb_neighbors, lp_segmented, estimate_normals);

    /// use the new planar_ransac with reading labels and colors (defined in planar_ransac.h)
    std::string ascii_in="/mnt/DataPartition/threed_modeling/input_data/Area_1_conferenceRoom_2_sub.txt";
    std::string ascii_out="/mnt/DataPartition/threed_modeling/out/points_normal_segment.txt";
    LaserPoints lp_segmented;
    //calculate_normal_wrapper(ascii_in, ascii_out);
    double  probability     = 0.05 ;    // Set probability to miss the largest primitive at each iteration.
    int     min_points      = 100  ;    // Detect shapes with at least n-min points.
    double  epsilon         = 0.02 ;    // Set maximum Euclidean distance between a point and a shape.
    double  cluster_epsilon = 0.08;     //0.08 ;  // Set maximum Euclidean distance between points to be clustered.
    double  normal_thresh   = 0.087;    // Set maximum normal deviation.// 0.9 < dot(surface_normal, point_normal);
    Efficient_ransac::Parameters ransac_parameters;
    set_parameters(ransac_parameters, probability, min_points, epsilon, cluster_epsilon, normal_thresh);
    Point_set point_set;
    planar_ransac(ascii_in, ransac_parameters, point_set, lp_segmented);
    save_point_set(point_set, ascii_out);
    lp_segmented.Write("/mnt/DataPartition/threed_modeling/out/lp_segmented.laser", false);

    /// linetopology to off format
    ObjectPoints vertices;
    LineTopologies faces;
    //vertices.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_new_vertices.objpts");
    //faces.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_new_edges.top", false);
    //LineTopologies_withAttr_to_OFF(vertices, faces, LineLabelTag, root_dir, true);

//    LaserPoints lp;
//    lp.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/out/updated_labels.laser");
//    lp.SetAttribute(ScanNumberTag, lp.GetAttribute(ScanLineNumberTag));
//    lp.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/updated_labels2.laser", false);

    /// sample points in faces
    LineTopologies faces_edges;
    ObjectPoints faces_vertices;
    LaserPoints sampled_points;
    //faces_vertices.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/out/out_4segments_faceselection/polygon_new_vertices.objpts");
    //faces_edges.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/out/out_4segments_faceselection/polygon_new_edges.top", false);
    //sampled_points = Sample_Laserpoints_3D (faces_vertices, faces_edges, root_dir);
    //sampled_points = Face_to_Voxel(faces_vertices, faces_edges, root_dir, 0.2);
    //sampled_points = Face_to_Voxel_with_noise(faces_vertices, faces_edges, root_dir, 0.05, 0.025, ScanLineNumberTag);
   // sampled_points.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/sampled_points.laser", false);
//    LaserPoints sampled_points_noise, sampled_points_noiseG;
//    sampled_points_noise = sampled_points.AddNoise(0.1);
//    sampled_points_noise.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/sampled_points_noise.laser", false);
//    sampled_points_noiseG = sampled_points.AddNoiseG(0.2, 0.1);
//    sampled_points_noiseG.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/sampled_points_noiseG.laser", false);


   // char *input_ascii = (char*) "/mnt/DataPartition/threed_modeling/input_data/room1_crop_spacedel.txt";
   // std::string data_dir = "/mnt/DataPartition/threed_modeling/";
//    std::string filename, file_ext;
//    boost::filesystem::path p(input_ascii);
//    file_ext = p.filename().c_str();    // office1.txt
//    filename = p.stem().c_str();        // office1

//    /// write segmented laser file
//    std::string lp_seg_dir = "/mnt/DataPartition/threed_modeling/data/out_data/lp_seg_dir";
//    std::string lp_seg_path = lp_seg_dir + "/laser/" + filename + ".laser" ; // "/mnt/DataPartition/threed_modeling/data/out_data/lp_seg_dir/laser"
//    cout << lp_seg_path << endl;
//    LaserPoints lp_segmented;
//    lp_segmented.Read("/mnt/DataPartition/threed_modeling/out_lpoints/lp_segmented.laser");
//    char char_arr[500];
//    lp_segmented.Write(strcpy (char_arr, lp_seg_path.c_str()), false);
//    lp_seg_path = lp_seg_dir + "/laser/" + filename + "_2.laser" ;
//    lp_segmented.Write(strcpy (char_arr, lp_seg_path.c_str()), false);
//    ObjectPoints threedbox_vertices; //output
//    LineTopologies threedbox_faces; // output
//    std::string out_dir = "/mnt/DataPartition/threed_modeling/out_dir";
//    std::string off_outfile_str = out_dir + "/off/" + filename + ".off" ;
//    bounding_cube(strcpy(char_arr ,off_outfile_str.c_str()), lp_segmented, 1.1, threedbox_vertices, threedbox_faces);

    //room2cellsdecomposition(input_ascii, data_dir);
    std::string in_dir =  "/mnt/DataPartition/threed_modeling";
    //cell_decomposition_wrapper(in_dir);

    /// ascii to laserpoints
//    char *input_ascii = (char*) "/mnt/DataPartition/threed_modeling/input_data/Area_1_conferenceRoom_2.txt";
//    LaserPoints lp;
//    lp = read_ascii(input_ascii);
//    lp.Write("/mnt/DataPartition/threed_modeling/dump/Area_1_conferenceRoom_2.laser", false);

    /// calcualte area, get the convex hull, get the countour of points
 //   LaserPoints one_segment;
//    one_segment.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/wall_crop.laser");
//    ObjectPoints conv_hull_v, contour_v;
//    LineTopology conv_hull_e, contour_e;
//    LineTopologies conv_hull_es, contour_es;
//    one_segment.DeriveTIN();
//    one_segment.ConvexHull(conv_hull_v, conv_hull_es, 0.1);
//    conv_hull_v.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/conv_hull.objpts");
//    conv_hull_es.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/conv_hull.top", false);
    //one_segment.DeriveConvexhull();
    //one_segment.DeriveContour3D(contour_v, contour_e, 0.1);
//    PointNumberList pnumlist;
//    TINEdges tinedges;
//    contour_e = one_segment.DeriveContour(2, pnumlist, tinedges);
//    contour_es.push_back(contour_e);
//    contour_v.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/contour_v.objpts");
//    contour_es.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/contour_es.top", false);
//    TIN tin;
//    //covert laserpoints to objpoints
//    ObjectPoints segment_objpoints;
//    cout << one_segment.size() << endl;
//    segment_objpoints = one_segment.ConstructObjectPoints();
//    cout << segment_objpoints.size() << endl;
//    LineTopologies edges;
//    ObjectPoints centers;
//    segment_objpoints.Triangulate(edges, centers);
//    centers.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/centers.objpts");
//    edges.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/edges.top", false);

//    alphashape_w_fastlocation(one_segment, "/mnt/DataPartition/CGI_UT/cell_decomposition/alpha_sh.off");


    return 0;
}






