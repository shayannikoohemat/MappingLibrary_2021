#include <iostream>
#include <LaserPoints.h>
#include <map>
#include <Plane.h>
#include "intersecting_clippers.h"
#include "face_selection.h"
#include "../utils/utils.h"
#include "../visualization_tools/visualization_tools.h"


int main() {

/*    char* inputlaser = (char*) "/mnt/DataPartition/CGI_UT/cell_decomposition/lp.laser";
    char* root_dir = (char*) "/mnt/DataPartition/CGI_UT/cell_decomposition/out";
    intersect_planes(inputlaser, root_dir, 100);*/

    char* root_dir = (char*) "/mnt/DataPartition/CGI_UT/cell_decomposition/out/";
    char str_root[500];
    strcpy (str_root, root_dir);

    /// create and read the bounding box of the data
//    char* inputlaser = (char*) "/mnt/DataPartition/CGI_UT/cell_decomposition/haaksbergen/walls_hksb_2ndfloor.laser";
//    LaserPoints lp;
//    lp.Read(inputlaser);
//    /// create a cuboid with 6 faces around the input data as the bound planes
//    ObjectPoints cube_global_vertices; // output
//    LineTopologies cube_global_faces; // output
//    Planes cube_planes; // output
//    std::map<int, Positions3D> planes_pos_map; // output
//    cube_planes = bounding_cube(root_dir, lp, cube_global_vertices, cube_global_faces, planes_pos_map);
//    cube_global_vertices.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_vertices.objpts");
//    cube_global_faces.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygons_edges.top", false);
    LineTopologies box3d_faces;
    ObjectPoints box3d_vertices;
    box3d_faces.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/cube_global_edges.top", false);
    box3d_vertices.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/cube_global_vertices.objpts");

    /// test Intersect_Planes_3DBoxFaces
    LaserPoints segments;
    LineTopologies polygons_edges;
    ObjectPoints polygons_vertices;
    segments.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/four_segments.laser");
    Intersect_Planes_3DBoxFaces(segments, 100, box3d_faces, box3d_vertices,
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
//    //segments.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/segments.laser"); // for the spliting planes
//    ///input polygons are created by Intersect_Planes_3DBoxFaces()
    SplitPolygons3DByPlanes3D(polygons_vertices, polygons_edges, segments, new_polys_v, new_polys_e);
    new_polys_v.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_new_vertices.objpts");
    new_polys_e.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_new_edges.top", false);

    /// test  vector< pair <int,int>> Collect_IntersectingPlanes ()
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

    /// test if points inside polygon3D
    //LaserPoints segments;
    //segments.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/four_segments.laser");
    //segments.push_back(LaserPoint(108.0, 109.0, 5.0)); // inside for horizontal
    //segments.push_back(LaserPoint(108.0, 116.0, 10.0)); // outside for horizontal
    //segments.push_back(LaserPoint(99.22, 102.86, 10.84)); // slightly outside
    //segments.push_back(LaserPoint(99.19, 102.87, 10.88)); // inside
    //segments.push_back(LaserPoint(99.19, 103.13, 8.48));
    //ObjectPoints new_polys_v;
    //LineTopologies new_polys_e;
    //new_polys_v.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_new_vertices.objpts");
    //new_polys_e.Read("/mnt/DataPartition/CGI_UT/cell_decomposition/out/polygon_new_edges60-5.top", false);
    segments.SetAttribute(LabelTag, 10000); //WARNING!!! Change 10000 later to a not repeated number
    vector<LaserPoints> segments_vec;
    segments_vec = PartitionLpByTag(segments, SegmentNumberTag, root_dir);
    LaserPoints updated_labels, points_inside, points_outside;
    LineTopologies valid_polygons;
    for (auto &segment : segments_vec){
        cout << "Segment: " << segment[0].SegmentNumber() << endl;
        //if(segment[0].SegmentNumber() != 60) continue;
        Plane splane;
        splane = segment.FitPlane(segment[0].SegmentNumber());
        splane.Number() = segment[0].SegmentNumber();

        LaserPoints updated_tmp;
        LineTopologies::iterator poly_it;
        cout << "   polygon:";
        //for (auto &polygon : new_polys_e){ // crashes in debug mode ??!!
        for (poly_it=new_polys_e.begin(); poly_it!=new_polys_e.end(); poly_it++){
            LineTopology polygon = *poly_it;
            cout << "polygon.Number: " << polygon.Number() << endl;
            ObjectPoints vertices = GetCorresponding_vertices(new_polys_v, polygon);
            /// reconstruct a plane with 4 vertices
            /// to make sure points of a segment are not associated to an adjacent polygon with different orientation/normal
            Plane plane;
            plane = Plane(vertices[0].Position3DRef (),
                    vertices[1].Position3DRef (),vertices[2].Position3DRef ());
            plane.AddPoint (vertices[3].Position3DRef (), true); // true means recalculate the plane
            Vector3D normal;
            normal = plane.Normal ();
            /// check segment plane and polygon plane parallelism
            bool parallel=false;
            double angle_radian = Angle(plane.Normal(), splane.Normal()); // this can have negative values as well
            if (angle_radian > M_PI/2.0) angle_radian -= M_PI;
            //////TODO: later shorten this block by changing the "<" to ">"
            if (fabs(angle_radian) < 0.017) {
                parallel = true;
                //cout << " ... par/ ";
            } //else cout << " ... Notpar/ ";
            if(!parallel) continue; // if not prallel skip this plane
            //////
            updated_tmp = PointsInsidePolygon3D(segment, 0.10, vertices, polygon);
            updated_labels.AddPoints(updated_tmp.SelectTagValue(LabelTag, polygon.Number()));
            //updated_tmp.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/updated_tmp.laser", false);
            /// make a new set of polygons of those with points
            LaserPoints inside_points;
            inside_points = updated_tmp.SelectTagValue(LabelTag, polygon.Number());
            auto lasersize = inside_points.size();
            if(lasersize>1000) // why this throws unexpected errors?? double free or corruption (out) or
            {
                valid_polygons.push_back(polygon);
            }
        }
        cout << endl;
    }

    updated_labels.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/updated_labels.laser", false);
    valid_polygons.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/valid_polygons.top", false);
    return 0;
}




