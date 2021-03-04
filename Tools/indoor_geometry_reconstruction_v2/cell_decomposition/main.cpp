#include <iostream>
#include <LaserPoints.h>
#include <map>
#include <Plane.h>
#include "intersecting_clippers.h"
#include "../utils/utils.h"
#include "../visualization_tools/visualization_tools.h"


int main() {

/*    char* inputlaser = (char*) "/mnt/DataPartition/CGI_UT/cell_decomposition/lp.laser";
    char* root_dir = (char*) "/mnt/DataPartition/CGI_UT/cell_decomposition/out";
    intersect_planes(inputlaser, root_dir, 100);*/

    char* root_dir = (char*) "/mnt/DataPartition/CGI_UT/cell_decomposition/out";
    char str_root[500];
    strcpy (str_root, root_dir);

    //char* inputlaser = (char*) "/media/shayan/DataPartition/driveD_data/data/cell_decomposition/lp.laser";
    //LaserPoints lp;
    //lp.Read(inputlaser);
    LineTopologies box3d_faces;
    ObjectPoints box3d_vertices;
    box3d_faces.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/cube_global_faces.top", false);
    box3d_vertices.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/cube_global_vertices.objpts");

    //vector<LaserPoints> object_seg_vec; // object can be walls, fl, cl or furniture or noise
    //object_seg_vec = PartitionLpByTag(lp, SegmentNumberTag, root_dir);

    // iterate through segments and create: intersection line segments
//    for (auto &segment : object_seg_vec){
//        cout << "Segment NUmber: " << segment[0].SegmentNumber() << endl;
//        if(segment.HasAttribute(SegmentNumberTag) && segment.size() > 100){
//            test_Intersect_Plane_3DRectnagle(segment, box3d_faces, box3d_vertices);
//        }
//    }
    LaserPoints segments;
    LineTopologies polygons_edges;
    ObjectPoints polygons_vertices;
    segments.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/pair_segments.laser");
    Intersect_Planes_3DBoxFaces(segments, 5, box3d_faces, box3d_vertices,
                                polygons_edges, polygons_vertices, true);
    polygons_vertices.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_vertices.objpts");
    polygons_edges.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_edges.top", false);

    return 0;
}




