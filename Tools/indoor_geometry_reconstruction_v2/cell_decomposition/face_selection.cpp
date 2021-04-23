#include "face_selection.h"
#include "intersecting_clippers.h"
#include <iostream>
#include <LaserPoints.h>
#include <map>
#include <Plane.h>
#include "../utils/utils.h"
#include "../visualization_tools/visualization_tools.h"


// not finished
int FaceSelection::faceSelection(LaserPoints segments, int min_segsize, double dist_threshold,
                                 double area_threshld,
                                 ObjectPoints faces_v, LineTopologies faces_e)
{
    std::multimap<int, LineTopologies> face_seg_mmap;

    vector<Plane> faces_pl_vec;
    vector<LaserPoints> faces_p_vec;
    vector<ObjectPoints> faces_v_vec;
    vector<LineTopology> faces_vec;
    vector<Position3D> faces_cnt_vec;
    //vector<LaserPoints> faces_points_vec;
    std::map<int, LaserPoints> faces_points_m;
    /// populate vectors of faces
    for (auto &face : faces_e){
        ObjectPoints face_v = GetCorresponding_vertices(faces_v, face);
        LaserPoints face_vpoints;
        LaserPoints lp; // we add this here, and fill it later
        face_vpoints.AddPoints(face_v);
        Plane face_plane;
        face_plane = face_vpoints.FitPlane(face.Number());
        face_plane.Number() = face.Number();
        Position3D face_centroid = face_plane.CentreOfGravity();

        faces_pl_vec.push_back(face_plane);
        faces_v_vec.push_back(face_v);
        faces_vec.push_back(face);
        faces_cnt_vec.push_back(face_centroid);
        faces_p_vec.push_back(face_vpoints);
        //faces_points_vec.push_back(lp);
        faces_points_m.insert(pair<int, LaserPoints> (face.Number(), lp));
    }

    char* root_dir;
    vector<LaserPoints> segments_vec;
    segments_vec = PartitionLpByTag(segments, SegmentNumberTag, root_dir);
    for(auto &s: segments_vec){
        int seg_num = s[0].SegmentNumber();
        if(seg_num < 0) continue; // to skip non valid segments
        if(s.size() < min_segsize) continue; // to skip small segments
        Plane plane;
        plane = s.FitPlane(seg_num);
        plane.Number() = seg_num;
        for (auto it = faces_vec.begin() ; it != faces_vec.end(); it++){ // here we can loop through in faces_e LineTopologies, but this is safer
            /// quick check 1. face plane and seg plane are almost-parallel
            int inx = it - faces_vec.begin();
            Plane f_plane = faces_pl_vec[inx];
            Vector3D f_normal = f_plane.Normal();
            LineTopology face = *it;
            ObjectPoints face_v = faces_v_vec[inx];
            double area = face.CalculateArea(face_v);
            cout << "area: " << area << endl;
            if(area < area_threshld) continue;
            double product = f_normal.DotProduct(plane.Normal());
            cout << "Angle: " << Angle(f_normal, plane.Normal()) * 180.0 / PI;
            if ((abs(product) - 1) > 0.0001) { // if two vectors are parallel their DotProduc should be 1 or -1
                cout << "planes not parallel, skip! ... " << product << endl;
                continue; // not parallel, skip the face
            }
            ///quick check 2. face centroid is in proximity of the segment
            double dist;
            Position3D centroid = faces_cnt_vec[inx];
            dist = plane.Distance(centroid);
            if(dist > dist_threshold) continue;
            LaserPoints f_vertices = faces_p_vec[inx];
            LaserPoints f_lpoints;
            auto map_it = faces_points_m.find(face.Number());
            if(map_it != faces_points_m.end()) f_lpoints = map_it->second;
            for (auto &p : s) {
                if(p.InsidePolygon(f_vertices, face.PointNumberListReference(), true)){
                    // add point to the supported points of the face
                    // if a face has many supported points (from one segment) add the end then it is selected ???
                    f_lpoints.push_back(p);
                }
            }
            // I m not sure here we need to update the map faces_points_m() with the new f_lpoints or it is already updated???
        }
    }
}

// for given segment(s) of laserpoints and several faces/polygons associate points to each face
/// polygons and faces are created by  Intersect_Planes_3DBoxFaces() and
/// then SplitPolygons3DByPlanes3D(), respectively;
/// NOTE: this function overrides the labels
LaserPoints FaceSelection::associatePointsToFace3D(LaserPoints segments, int min_segsize,
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
    segments.SetAttribute(LabelTag, 1000000); //WARNING!!! Change 1000000 later to a not repeated number
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
            //////TODO: later shorten this block by changing the "<" to ">"
            if (fabs(angle_radian) < 0.017) {
                parallel = true;
                //cout << " ... par/ ";
            } //else cout << " ... Notpar/ ";
            if(!parallel) continue; // if not prallel skip this plane
            /// *** important function PointsInsidePolygon3D() ***
            updated_tmp = PointsInsidePolygon3D(segment, dist_threshold, vertices, polygon);
            updated_labels.AddPoints(updated_tmp.SelectTagValue(LabelTag, polygon.Number()));
            //updated_tmp.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/updated_tmp.laser", false);
            /// make a new set of polygons of those with points
            //polygon.CalculateArea(vertices);
            LaserPoints inside_points;
            inside_points = updated_tmp.SelectTagValue(LabelTag, polygon.Number());
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

