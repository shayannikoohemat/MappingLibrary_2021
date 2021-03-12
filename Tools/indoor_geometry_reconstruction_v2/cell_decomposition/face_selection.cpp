#include "face_selection.h"
#include "intersecting_clippers.h"
#include <iostream>
#include <LaserPoints.h>
#include <map>
#include <Plane.h>
#include "../utils/utils.h"
#include "../visualization_tools/visualization_tools.h"



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
int FaceSelection::associatePointsToFace3D()
{

}
