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

    /// create and read the bounding box of the data
    //char* inputlaser = (char*) "/media/shayan/DataPartition/driveD_data/data/cell_decomposition/lp.laser";
    //LaserPoints lp;
    //lp.Read(inputlaser);
    LineTopologies box3d_faces;
    ObjectPoints box3d_vertices;
    box3d_faces.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/cube_global_faces.top", false);
    box3d_vertices.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/cube_global_vertices.objpts");

    /// test Intersect_Planes_3DBoxFaces
    LaserPoints segments;
    LineTopologies polygons_edges;
    ObjectPoints polygons_vertices;
    segments.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/triple_segments.laser");
    Intersect_Planes_3DBoxFaces(segments, 5, box3d_faces, box3d_vertices,
                                polygons_edges, polygons_vertices, true, true);
    polygons_vertices.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_vertices.objpts");
    polygons_edges.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_edges.top", false);

    /// test SplitPolygons3DByPlane3D()
//    LaserPoints segment;
//    ObjectPoints new_polys_v; LineTopologies new_polys_e;
//    segment.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/segment3.laser"); // for the spliting plane
//    // input polygons are created by Intersect_Planes_3DBoxFaces()
//    SplitPolygons3DByPlane3D(polygons_vertices, polygons_edges, segment, new_polys_v, new_polys_e);
//    new_polys_v.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_new_vertices.objpts");
//    new_polys_e.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_new_edges.top", false);

    /// test SplitPolygon3DByPlanes3D()
    ObjectPoints new_polys_v; LineTopologies new_polys_e;
    //segments.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/segments.laser"); // for the spliting planes
    // input polygons are created by Intersect_Planes_3DBoxFaces()
    SplitPolygon3DByPlanes3D(polygons_vertices, polygons_edges, segments, new_polys_v, new_polys_e);
    new_polys_v.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_new_vertices.objpts");
    new_polys_e.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_new_edges.top", false);

    /// test  vector< pair <int,int>> Collect_IntersectingPLanes ()
//    LaserPoints lp;
//    lp.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/lp_inclined.laser");
//    /// make a list of planes
//    Planes planes;
//    vector<LaserPoints> segments_vec;
//    segments_vec = PartitionLpByTag(lp, SegmentNumberTag, root_dir);
//    for (auto &s : segments_vec){
//        Plane plane;
//        plane = s.FitPlane(s[0].SegmentNumber());
//        plane.Number() = s[0].SegmentNumber();
//        planes.push_back(plane);
//    }
//    LineTopologies polygons;
//    ObjectPoints vertices;
//    std::map<int, int> pair_intersected_polygons;
//    Intersect_Planes_3DBoxFaces(lp, 5, box3d_faces, box3d_vertices, polygons, vertices, false, true);
//    collect_intersectingPlanes(planes, vertices, polygons);
//    vertices.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_inclined_vertices.objpts");
//    polygons.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_inclined_edges.top", false);

    /// split pairwise polygons
//    LaserPoints segments;
//    segments.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/triple_segments.laser");
//    cout << segments.size() << endl;
//    test_pairwise_split(segments, box3d_faces, box3d_vertices, 0.01);


    return 0;
}




