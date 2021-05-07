#include <iostream>
#include <LaserPoints.h>
#include <map>
#include <Plane.h>
#include "intersecting_clippers.h"
#include "face_selection.h"
#include "../utils/utils.h"
#include "../visualization_tools/visualization_tools.h"
#include "planar_segmentation.h"


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
    box3d_faces.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/s3dis/cube_global_edges.top", false);
    box3d_vertices.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/s3dis/cube_global_vertices.objpts");
    //box3d_faces.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/cube_global_edges_scaled.top", false);
    //box3d_vertices.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/cube_global_vertices_scaled.objpts");

    /// a test on linetopology tags
    //    LineTopologies box3d_faces_tagged;
//    int cnt=0;
//    for(auto &face:box3d_faces){
//        face.SetAttribute(LineNumberTag, cnt);
//        face.SetAttribute(LineLabelTag, cnt); //use this to assing the segment number
//        face.SetAttribute(LineTopologyTag(GeometryTag), cnt);
//        face.Number()=cnt;
//        box3d_faces_tagged.push_back(face);
//        cout << face.Attribute(LineNumberTag) << endl;
//        cnt++;
//    }
//    box3d_faces_tagged.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/s3dis/cube_global_edges_tagged2.top", false);


    /// test Intersect_Planes_3DBoxFaces
    LaserPoints segments;
    LineTopologies polygons_edges;
    ObjectPoints polygons_vertices;
    int min_seg_size = 500;
    segments.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/room_segmented.laser");
    Intersect_Planes_3DBoxFaces(segments, min_seg_size, box3d_faces, box3d_vertices,
                                polygons_edges, polygons_vertices, true, true);
    polygons_vertices.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_vertices.objpts");
    polygons_edges.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_edges.top", false);

    /// test SplitPolygons3DByPlane3D()
//    LaserPoints segment;
//    ObjectPoints new_polys_v; LineTopologies new_polys_e;
    //segment.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/segment60.laser"); // for the spliting plane
    /// input polygons are created by Intersect_Planes_3DBoxFaces()
//    SplitPolygons3DByPlane3D(polygons_vertices, polygons_edges, segment, new_polys_v, new_polys_e);
//    new_polys_v.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_new_vertices.objpts");
//    new_polys_e.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_new_edges.top", false);

    //////////// CELL DECOMPOSITION ///////////////////
    /// Use this with output of Intersect_Planes_3DBoxFaces() for CELL DECOMPOSITION
    ObjectPoints new_polys_v; LineTopologies new_polys_e;
    //segments.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/segments.laser"); // for the spliting planes
    ///input polygons are created by Intersect_Planes_3DBoxFaces()
    SplitPolygons3DByPlanes3D(polygons_vertices, polygons_edges, segments, min_seg_size, new_polys_v, new_polys_e, false);
    new_polys_v.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_new_vertices.objpts");
    new_polys_e.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_new_edges.top", false);


    /// test associating points to faces
    /// USE this with the results of cell decomposition: SplitPolygons3DByPlanes3D()
    LineTopologies valid_polygons;
    LaserPoints updated_labels;
    FaceSelection fs;
    //updated_labels = fs.associatePointsToFace3D(segments, min_seg_size, 0.12, 0.5, 50, new_polys_v, new_polys_e, valid_polygons); // s3dis: 500, 0.08, 0.5, 500,
    updated_labels = fs.associatePointsToFace3D_withTag(segments, min_seg_size, 0.08, 0.5, 1000, new_polys_v, new_polys_e, valid_polygons);
    updated_labels.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/updated_labels.laser", false);
    valid_polygons.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/valid_polygons.top", false);

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

    /// efficient ransac
    //char *filename = "/mnt/DataPartition/CGI_UT/cell_decomposition/room1_spacedel.txt";
    //char *outdir = "/mnt/DataPartition/CGI_UT/cell_decomposition/out";
    //Efficient_ransac::Parameters parameters;
    ///set_parameters(parameters, 0.05, 500, 0.02, 0.08, 0.087); // parameters for S3DIS
    //set_parameters(parameters, 0.05, 200, 0.02, 0.08, 0.087);
    ///basic_efficient_ransac0

    return 0;
}




