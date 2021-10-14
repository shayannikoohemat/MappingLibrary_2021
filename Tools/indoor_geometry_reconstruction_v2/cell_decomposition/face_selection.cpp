#include "face_selection.h"
#include "intersecting_clippers.h"
#include <iostream>
#include <LaserPoints.h>
#include <map>
#include <Plane.h>
#include "../utils/utils.h"
#include "../visualization_tools/visualization_tools.h"


// for given segment(s) of laserpoints and several faces/polygons associate points to each face
/// polygons and faces are created by  Intersect_Planes_3DBoxFaces() and
/// SplitPolygons3DByPlanes3D(), respectively;
/// This function uses LineLabelTag in linetopology to identify faces belong to the same segment
/// and then set the points inside a face the same tag (ScanLineNumberTag)
/// faces without points or few points get an invalid tag(linenumbertag=101) and
/// faces with points are valid so get (linenumbertag=100)
/// NOTE: this function saves the face number in ScanLineNumberTag
LaserPoints associatePointsToFace3D_withTag(LaserPoints segments, int min_segsize,
                                                   double dist_threshold,
                                                   double area_threshld,
                                                   int min_points_for_face_selection,
                                                   ObjectPoints faces_v,
                                                   LineTopologies faces_e,
                                                   LineTopologies &faces_with_points,
                                                    LineTopologies &faces_without_points,
                                                    LineTopologies &all_faces)
{
    char* root_dir; //= (char*) "/mnt/DataPartition/CGI_UT/cell_decomposition/out/";
    //char str_root[500];
    //strcpy (str_root, root_dir);
    int big_number = 1.0e7; // this tag number should be bigger than number of polygons
    if(big_number < faces_e.size()) big_number = faces_e.size()*2;
    segments.SetAttribute(ScanLineNumberTag, big_number);
    vector<LaserPoints> segments_vec;
    segments_vec = PartitionLpByTag(segments, SegmentNumberTag, root_dir);

    LaserPoints updated_labels;

    for (auto &segment : segments_vec){
        if(segment.size() < min_segsize) continue;
        int segment_num = segment[0].SegmentNumber();
        //cout << "Segment: " << segment_num << endl;

        LaserPoints updated_tmp;
        LineTopologies::iterator poly_it;
        //for (auto &polygon : new_polys_e){ // crashes in debug mode ??!!
        for (poly_it=faces_e.begin(); poly_it!=faces_e.end(); poly_it++){
            LineTopology polygon = *poly_it;
            //cout << "polygon number: " << polygon.Number();
            /// check if the polygon and segment don't have the same tag continue to the next polygon
            if(polygon.HasAttribute(LineLabelTag)){
                int polygon_tag = polygon.Attribute(LineLabelTag);
                //cout << "   polygon tag: " << polygon_tag << endl;
                if(polygon_tag != segment_num)
                    continue;
            } else
            {
                cout << "Warning! polygon/face " << polygon.Number() << " doesn't have line labeltag." << endl;
                continue;
            }

            /// *** important function PointsInsidePolygon3D() ***
            /// here we assign the face-number to points inside each face. We use ScanLineNumberTag for this tag.
            ObjectPoints vertices = GetCorresponding_vertices(faces_v, polygon);
            updated_tmp = PointsInsidePolygon3D(segment, dist_threshold, vertices, polygon, ScanLineNumberTag);
            updated_labels.AddPoints(updated_tmp.SelectTagValue(ScanLineNumberTag, polygon.Number()));
            //updated_tmp.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/updated_tmp.laser", false);
            /// make a new set of polygons of those with points
            //double face_area = polygon.CalculateArea(vertices); // this should give the area of a single face

            LaserPoints inside_points;
            inside_points = updated_tmp.SelectTagValue(ScanLineNumberTag, polygon.Number());
            //TODO: fit a polygon to inside points and calculate the area of the coverage for valid face selection
            auto lasersize = inside_points.size();
            if(lasersize > min_points_for_face_selection) // why this throws unexpected errors??
            {
                // LineNumberTag unique code in Top files is 16.
                polygon.SetAttribute(LineNumberTag, 100); //valid
                faces_with_points.push_back(polygon);
                all_faces.push_back(polygon);
            } else
            {
                polygon.SetAttribute(LineNumberTag, 101); // invalid
                faces_without_points.push_back(polygon);
                all_faces.push_back(polygon);
            }
        }
        cout << endl;
    }
    return updated_labels;
}


// for given segment(s) of laserpoints and several faces/polygons associate points to each face
/// polygons and faces are created by  Intersect_Planes_3DBoxFaces() and
/// then SplitPolygons3DByPlanes3D(), respectively;
/// This function instead of LineNumberTag uses coplanarity of segment's plane and face's plane to
///  identify faces belonging to the same segment
/// NOTE: this function overrides the point labels
LaserPoints associatePointsToFace3D(LaserPoints segments, int min_segsize,
                                                   double dist_threshold,
                                                   double area_threshld,
                                                   int min_points_for_face_selection,
                                                   ObjectPoints faces_v,
                                                   LineTopologies faces_e,
                                                   LineTopologies &selected_faces)
{

    char* root_dir; //= (char*) "/mnt/DataPartition/CGI_UT/cell_decomposition/out/";
    //char str_root[500];
    //strcpy (str_root, root_dir);
    int big_number = 1.0e7; // this tag number should be bigger than number of polygons
    if(big_number < faces_e.size()) big_number = faces_e.size()*2;
    segments.SetAttribute(ScanLineNumberTag, big_number);
    vector<LaserPoints> segments_vec;
    segments_vec = PartitionLpByTag(segments, SegmentNumberTag, root_dir);
    LaserPoints updated_labels, points_inside, points_outside;
    for (auto &segment : segments_vec){
        if(segment.size() < min_segsize) continue;
        cout << "Segment: " << segment[0].SegmentNumber() << endl;
        Plane splane;
        splane = segment.FitPlane(segment[0].SegmentNumber());
        splane.Number() = segment[0].SegmentNumber();

        LaserPoints updated_tmp;
        LineTopologies::iterator poly_it;
        cout << "   polygon:";
        //for (auto &polygon : new_polys_e){ // crashes in debug mode ??!!
        for (poly_it=faces_e.begin(); poly_it!=faces_e.end(); poly_it++){
            LineTopology polygon = *poly_it;
            cout << "polygon.Number: " << polygon.Number() << endl;
            ObjectPoints vertices = GetCorresponding_vertices(faces_v, polygon);
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
            //////TODO: later shorten this block by changing the "<" into ">"
            if (fabs(angle_radian) < 0.017) {
                parallel = true;
                //cout << " ... par/ ";
            } //else cout << " ... Notpar/ ";
            if(!parallel) continue; // if not prallel skip this plane
            /// *** important function PointsInsidePolygon3D() ***
            updated_tmp = PointsInsidePolygon3D(segment, dist_threshold, vertices, polygon, ScanLineNumberTag);
            updated_labels.AddPoints(updated_tmp.SelectTagValue(ScanLineNumberTag, polygon.Number()));
            //updated_tmp.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/updated_tmp.laser", false);
            /// make a new set of polygons of those with points
            //polygon.CalculateArea(vertices);
            LaserPoints inside_points;
            inside_points = updated_tmp.SelectTagValue(ScanLineNumberTag, polygon.Number());
            //TODO: fit a polygon to inside points and calculate the area of the coverage for valid face selection
            auto lasersize = inside_points.size();
            if(lasersize > min_points_for_face_selection) // why this throws unexpected errors??
            {
                selected_faces.push_back(polygon);
            }
        }
        cout << endl;
    }
    return updated_labels;
}

