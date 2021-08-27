#include "sample_laserpoints.h"
#include "LaserPoints.h"
#include "intersecting_clippers.h"
#include <Plane.h>
#include <ctime>
#include "Laservoxel.h"



LaserPoints Sample_Laserpoints_3D(ObjectPoints faces_vertices, LineTopologies faces_edges, char *root_dir)
{
    char str_root[500];
    strcpy (str_root, root_dir);

    LaserPoints sampled_points;
    LineTopology face;
    ObjectPoints face_v;
    face = faces_edges[0];
    face_v = GetCorresponding_vertices(faces_vertices, face);
    /// convert the vertices to laser points to derive a TIN
    LaserPoints v_points;
    PointNumberList p_numlist;
    for (auto &v : face_v) {
        p_numlist.push_back(v.Number());
        v_points.push_back(v.Position3DRef());
    }
    ObjectPoint face_centroid = Objpts_Centroid(face_v);
    v_points.push_back(face_centroid.Position3DRef());
    v_points.Write("/mnt/DataPartition/CGI_UT/cell_decomposition/out/v_points.laser", false);
    /// creat a TIN from the vertices of the face
    v_points.DeriveTIN();
    TIN *tin = v_points.GetTIN();
    TINMesh *mesh = tin->data();

    char *TINout;
    TINout = strcat(str_root, "/tin.tin");
    tin->Write(TINout);

    /// 2nd method to create the tin directly from objectpoints
    // point_list = laserpoints.TaggedPointNumberList (SegmentNumberTag, segment_number);
    face_centroid.PrintVector();
    ObjectPoints face_centroids;
    face_centroids.push_back(face_centroid);
    LineTopologies face_topo;
    face_topo.push_back(face);
    TIN tin2;
    tin2 = face_v.Triangulate(face_topo, face_centroids);

    LineTopologies tin_lines;
    tin_lines = LineTopologies(tin2);

    strcpy (str_root, root_dir);
    tin_lines.Write(strcat (str_root, "tin_lines.top"), false);
    strcpy (str_root, root_dir);
    face_v.Write(strcat(str_root, "tin_points.objpts")); /// this contains TIN points and the order should not be modifed
    /// otherwise the TIN Edges after importing in pcm are not correct. The order is polygons+laserpoints without centers
    sampled_points.push_back(Position3D(face_centroid));
    sampled_points = Create_Triangle_Centroids(tin2, face_v);

    return sampled_points;
}


/// calculate the centroid of triangles
LaserPoints Create_Triangle_Centroids (TIN &tin, ObjectPoints &vertices){

    LaserPoints triangle_centers;
    int triangle_cnt=0;
    for(auto &mesh : tin){
        triangle_cnt++;
        PointNumber *meshNumber;
        meshNumber = mesh.Nodes ();
        //printf ("Mesh nodes: %d -> %d %d %d \n", meshNumber->Number (), /// meshnumber is suspicious, becasue it is sometimes repeated
        //        meshNumber[0].Number (), meshNumber[1].Number (), meshNumber[2].Number ());

        /// getting the centroid of the triangle
        int num1, num2, num3;
        num1 = meshNumber[0].Number ();
        num2 = meshNumber[1].Number ();
        num3 = meshNumber[2].Number ();

        ObjectPoint *p1, *p2, *p3;
        p1 = vertices.GetPoint (num1);
        p2 = vertices.GetPoint (num2);
        p3 = vertices.GetPoint (num3);

        Position3D pos1, pos2, pos3;
        pos1 = p1->Position3DRef ();
        pos2 = p2->Position3DRef ();
        pos3 = p3->Position3DRef ();

        Position3D triangle_centroid;
        triangle_centroid.X () = (pos1.X () + pos2.X () + pos3.X ()) / 3;
        triangle_centroid.Y () = (pos1.Y () + pos2.Y () + pos3.Y ()) / 3;
        triangle_centroid.Z () = (pos1.Z () + pos2.Z () + pos3.Z ()) / 3;

        triangle_centers.push_back (triangle_centroid);
        //printf ("Triangle: %d ", triangle_cnt);
    }
    printf ("# Triangle: %d \n", triangle_cnt);
    //printf ("\n");

    return triangle_centers;
}

ObjectPoint Objpts_Centroid(ObjectPoints points){

    vector<double> x_vec;
    vector<double> y_vec;
    vector<double> z_vec;

    for (auto &p : points){
        x_vec.push_back(p.X());
        y_vec.push_back(p.Y());
        z_vec.push_back(p.Z());
    }

    ObjectPoint centroid;
    centroid.X() = std::accumulate(x_vec.begin(), x_vec.end(), 0.0) / x_vec.size();
    centroid.Y() = std::accumulate(y_vec.begin(), y_vec.end(), 0.0) / y_vec.size();
    centroid.Z() = std::accumulate(z_vec.begin(), z_vec.end(), 0.0) / z_vec.size();

    return centroid;
}

LaserPoints Face_to_Voxel(ObjectPoints faces_vertices, LineTopologies faces_edges,
                          char *root_dir, double vox_l)
{
    std::clock_t start;
    double duration;
    start = std::clock();

    /// convert the face vertices to laserpoints
    LineTopology face;
    ObjectPoints face_v;
    face = faces_edges[0];
    face_v = GetCorresponding_vertices(faces_vertices, face);
    LaserPoints v_points;
    PointNumberList p_numlist;
    for (auto &v : face_v) v_points.push_back(v.Position3DRef());


    ///initialize the LaserVoxel
    LaserVoxel vox(v_points, vox_l);
    //vox.statistics();

    char *vox_centers_path;
    char str_root[500];
    strcpy (str_root,root_dir);
    vox_centers_path = strcat(str_root, "vox_centers.laser");
    vector< vector < vector < int > > > vec_ijk;

    /// generating vox_centers in the given path and generating a vector to relate voxel ijk and centers
    LaserPoints vox_centers, vox_centers_occupied;
    vec_ijk = vox.export_vox_centres(1, vox_centers);
    vox_centers.Write(vox_centers_path, false);
    //vox_centers.Read(vox_centers_path);

    /// generating occupied voxels
    //vox_centers_occupied = vox_centers.SelectTagValue(LabelTag, 1);
    //strcpy (str_root, root_dir);
    //vox_centers_occupied.Write(strcat(str_root, "vox_centers_occupied.laser"), false);

    /// select voxel-centers in a specific distance of the segment_plane, we don't need all voxel centers
    Plane plane;
    v_points.SetAttribute(SegmentNumberTag, 1); // 1 random number, later replace it with segment number
    plane = v_points.FitPlane(v_points[0].SegmentNumber());
    plane.Number()= v_points[0].SegmentNumber();
    LaserPoints face_voxel_centers;
    bool full_point_segment = false; /// segment has no opening or gaps
    int planar_vox_cnt =0, occupy_cnt=0;
    for(int i=0; i<vox_centers.size(); i++)
    {
        double dist_plane_voxel;
        dist_plane_voxel = plane.Distance(vox_centers[i]);
        //cout << "distance: " << dist_plane_voxel << endl;
        if(fabs(dist_plane_voxel) < vox_l/2) // sqrt(3)*vox_l is a big threshold
        {
            planar_vox_cnt++;
            //if(vox_centers[i].Attribute (LabelTag) == 11) // occupied
            //    occupy_cnt++;
            face_voxel_centers.push_back(vox_centers[i]);
        }
    }
    cout << "points near plane: " << planar_vox_cnt << endl;
    return face_voxel_centers;
}

/// use this to add points to a 3D polygon
//TODO: points inside the polygon
//TODO: run it for all faces with out points
LaserPoints Face_to_Voxel_with_noise(ObjectPoints faces_vertices, LineTopologies faces_edges,
                          char *root_dir, double vox_l, double noise_level)
{
    std::clock_t start;
    double duration;
    start = std::clock();
    LaserPoints all_sampled_points;
    for (auto &face : faces_edges){

        /// convert the face vertices to laserpoints
        ObjectPoints face_v;
        face_v = GetCorresponding_vertices(faces_vertices, face);
        LaserPoints v_points;
        PointNumberList p_numlist;
        for (auto &v : face_v) v_points.push_back(v.Position3DRef());

        ///initialize the LaserVoxel
        LaserVoxel vox(v_points, vox_l);
        //vox.statistics();

    //    char *vox_centers_path;
        char str_root[500];
    //    strcpy (str_root,root_dir);
    //    vox_centers_path = strcat(str_root, "vox_centers.laser");
        vector< vector < vector < int > > > vec_ijk;

        /// generating vox_centers in the given path and generating a vector to relate voxel ijk and centers
        LaserPoints vox_centers, vox_centers_occupied;
        vec_ijk = vox.export_vox_centres(1, vox_centers);
        //vox_centers.Write(vox_centers_path, false);

        /// select voxel-centers in a specific distance of the segment_plane, we don't need all voxel centers
        Plane plane;
        v_points.SetAttribute(SegmentNumberTag, 1); // 1 random number, later replace it with segment number
        plane = v_points.FitPlane(v_points[0].SegmentNumber());
        plane.Number()= v_points[0].SegmentNumber();
        LaserPoints face_voxel_centers;
        bool full_point_segment = false; /// segment has no opening or gaps
        int planar_vox_cnt =0, occupy_cnt=0;
        DataBoundsLaser db;
        db = v_points.DeriveDataBounds(0);
        for(int i=0; i<vox_centers.size(); i++)
        {
            double dist_plane_voxel;
            dist_plane_voxel = plane.Distance(vox_centers[i]);
            //cout << "distance: " << dist_plane_voxel << endl;
            if(fabs(dist_plane_voxel) < vox_l/2) // sqrt(3)*vox_l is a big threshold
            {
                //if(InsideSegmentBounds(vox_centers[i].Position3DRef(), db)){ // it is not accurate
                    planar_vox_cnt++;
                    face_voxel_centers.push_back(vox_centers[i]);
                //}
            }
        }
        //cout << "points near plane: " << planar_vox_cnt << endl;
        /// project points to the plane
        LaserPoints projected_points;
        projected_points = Project_points_to_Plane(face_voxel_centers, plane);
        //strcpy (str_root,root_dir);
        //projected_points.Write(strcat(str_root, "projected_points.laser"), false);

        /// remove duplicates
        projected_points.RemoveAlmostDoublePoints(false, vox_l/2);
        /// subsample points on the plane
        //LaserPoints projected_points_sub;
        //projected_points_sub = projected_points.SubSampleSimple(5);
        //strcpy (str_root,root_dir);
        //projected_points_sub.Write(strcat(str_root, "projected_points_sampled.laser"), false);

        /// add points on the plane to the voxel centers
        face_voxel_centers.AddPoints(projected_points);
        //strcpy (str_root,root_dir);
        //face_voxel_centers.Write(strcat(str_root, "face_voxel_centers.laser"), false);

        /// add Gaussian Noise to the points
        LaserPoints lp_with_noise;
        lp_with_noise = face_voxel_centers.AddNoiseG(noise_level, 0);
        //strcpy (str_root,root_dir);
        //lp_with_noise.Write(strcat(str_root, "face_voxel_centers_noise.laser"), false);

        /// add the face number to sampled points
        lp_with_noise.SetAttribute(ScanLineNumberTag, face.Number());

        /// add points of this face to all_samplepoints
        all_sampled_points.AddPoints(lp_with_noise);
    }


    return all_sampled_points;
}


bool InsideSegmentBounds(Position3D pos, DataBoundsLaser db){
    // this is for later use in intersection with the ray
    //DataBoundsLaser db=lp.DeriveDataBounds(0);
    double seg_min_X, seg_min_Y, seg_min_Z;
    double seg_max_X, seg_max_Y, seg_max_Z;
    seg_min_X=db.Minimum().GetX();
    seg_min_Y=db.Minimum().GetY();
    seg_min_Z=db.Minimum().GetZ();

    seg_max_X=db.Maximum().GetX();
    seg_max_Y=db.Maximum().GetY();
    seg_max_Z=db.Maximum().GetZ();

    double  intersection_X,intersection_Y,intersection_Z;
    intersection_X=pos.GetX();
    intersection_Y=pos.GetY();
    intersection_Z=pos.GetZ();

    if ( seg_min_X < intersection_X && intersection_X < seg_max_X &&
         seg_min_Y < intersection_Y && intersection_Y < seg_max_Y &&
         seg_min_Z < intersection_Z && intersection_Z < seg_max_Z ){
        return true;
    }
    return false;
}

LaserPoints Project_to_Plane(LaserPoints lp, const Plane &plane)
{
    // generate laserpoints of projected points on the plane
    LaserPoints projected_points;
    for (int i=0; i<lp.size(); i++){
        Position3D projected_pos;
        LaserPoint projected_point;
        projected_pos = plane.Project(lp[i].Position3DRef());
        projected_point.SetX(projected_pos.GetX());
        projected_point.SetY(projected_pos.GetY());
        projected_point.SetZ(projected_pos.GetZ());
        projected_points.push_back(projected_point);
    }
    return projected_points;
}
