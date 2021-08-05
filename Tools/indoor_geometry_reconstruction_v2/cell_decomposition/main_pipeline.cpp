#include "main_pipeline.h"
#include <LaserPoints.h>
#include <boost/filesystem.hpp>
#include "intersecting_clippers.h"
#include "face_selection.h"
#include "planar_segmentation.h"

/*
 *  1. read the data per room (wall+floor+ceiling+pillar+beams)
 *  2. run the planar segmentation (ransac) --> int efficient_RANSAC_with_point_access()
 *  (2 and 3 can be swapped)
 *  3. create the data boundingbox (cuboid) --> void bounding_cube()
 *  4. run the cell decomposition:
 *      4.1 intersect the planes with cuboid faces to extend all planes to the data bounds --> Intersect_Planes_3DBoxFaces()
 *      4.2 then split all intersected faces to create cell decomposition --> SplitPolygons3DByPlanes3D()
 *  5. associating points to faces, this returns points with label of the faces --> updated_labels = associatePointsToFace3D_withTag()
 *  6. sample points on faces without points (optional) can be done in python too -->  Face_to_Voxel_with_noise()
 *  7. convert faces to OFF format --> LineTopologies_withAttr_to_OFF()
 *      in this step, the relation between faces and associated points should be preserved (either in a separate file or in OFF file)
 *  8. label associated points of each face into valid (part of the structure) and invalid (not structure or points outside of the room)
*/


LaserPoints read_ascii(char *ascii_file);



void room2cellsdecomposition(char *input_ascii, char* dump_dir, LaserPoints lp_out, LaserPoints lp_segmented,
                             ObjectPoints threedbox_vertices, LineTopologies threedbox_faces) {

    /// 1. read txt points and convert it to laser. It works with space and comma delimited files and ignores the first line
    /// it expects: xyz rgb Label (for changing columns look at the ReadAscii.cpp)
    lp_out = read_ascii(input_ascii);

    /// parameter settings (better to read this from a json file)
    /// 2. create a threedbox around the data:
    double scalefactor = 1.1;
    bounding_cube(dump_dir, lp_out, scalefactor, threedbox_vertices, threedbox_faces);

    /// 3. planar segmentation (e.g. RANSAC) on txt file
    double  probability     = 0.05 ;
    int     min_points      = 500  ;
    double  epsilon         = 0.02 ;
    double  cluster_epsilon = 0.08 ;
    double  normal_thresh   = 0.087;
    int     nb_neighbors    = 20   ;
    bool    estimate_normals = True ;
    Efficient_ransac::Parameters ransac_parameters;
    set_parameters(ransac_parameters, probability, min_points, epsilon, cluster_epsilon, normal_thresh); // parameters for S3DIS
    efficient_RANSAC_with_point_access (input_ascii, dump_dir, ransac_parameters,
                                 nb_neighbors, lp_segmented, estimate_normals);

    /// 4. running the cell decomposition process
    /// 4.1 intersect planes to the bbox of the room
    /// (for further comments check the implementation in intersecting_clippers.cpp)
    LineTopologies polygons_edges;
    ObjectPoints polygons_vertices;
    int min_seg_size = 500;
    // outputs are polygons which are new faces created by intersection to the data bbox
    Intersect_Planes_3DBoxFaces(lp_segmented, min_seg_size, threedbox_faces, threedbox_vertices,
                                polygons_edges, polygons_vertices, true, true);
    /// 4.2 split all intersected faces to create cell decomposition
    ObjectPoints new_polys_v;
    LineTopologies new_polys_e;
    // outputs are new faces created by intersection with eachother
    SplitPolygons3DByPlanes3D(polygons_vertices, polygons_edges, lp_segmented, min_seg_size, new_polys_v, new_polys_e, false);

    /// 5 associating points to faces, this returns points with label of the faces
    LaserPoints updated_labels;
    LineTopologies faces_with_points, faces_without_points;
    // for s3dis: 500, 0.08, 0.5, 500
    double dist_threshold = 0.08; //meter
    double area_threshld = 0.5; //is not used in the function
    int min_points_for_face_selection = 500;
    // NOTE: check ScanLineNumberTag for updated labels of points
    //TODO: if LineNumberTag works use it to label the face as valid or invalid and make one *.top file instead of two in face_selection.cpp
    updated_labels = associatePointsToFace3D_withTag(lp_segmented, min_seg_size, dist_threshold, area_threshld, min_points_for_face_selection,
                                                     new_polys_v, new_polys_e, faces_with_points, faces_without_points);


}


// not finished
void pipeline_v0(char* project_dir, std::string proj_dir_str){
    /// flags to swith on/off process
    bool do_threedbox_creation = False;
    bool do_planar_segmentation = True;

    /// parameter settings (read this from a json file)
    /// 1. threedbox:
    double scalefactor = 1.1;

    /// 2. efficient ransac
    double  probability     = 0.05 ;
    int     min_points      = 500  ;
    double  epsilon         = 0.02 ;
    double  cluster_epsilon = 0.08 ;
    double  normal_thresh   = 0.087;
    int     nb_neighbors    = 20   ;
    bool    estimate_normals = True ;


    char* out_dir = "/mnt/DataPartition/threed_modeling/out_dir";
    std::string out_dir_str = proj_dir_str + "/out_dir";

    if(!boost::filesystem::create_directories(out_dir))
        cerr << "Warning! direcotry: " << out_dir << " exists!" << endl;

    std::string out_lpoints = proj_dir_str + "/out_lpoints";

    if(!boost::filesystem::create_directories(out_lpoints))
        cerr << "Warning! direcotry: " << out_lpoints << " exists!" << endl;

    std::string input_data_laser = proj_dir_str + "/input_data/room.laser";

    /// 1. create the 3d bbox
    LaserPoints lpoints;
    lpoints.Read(input_data_laser.c_str());
    ObjectPoints threedbox_vertices; // output
    LineTopologies threedbox_faces; // output
    if(do_threedbox_creation){
        bounding_cube(out_dir, lpoints, scalefactor, threedbox_vertices, threedbox_faces);
        //strcpy (str_outdir, out_dir);
        //threedbox_vertices.Write(strcat(str_outdir, "/threedbox_v.objpts"));
        std::string threedbox_v_path = out_dir_str + "/threedbox_v.objpts";
        threedbox_vertices.Write(threedbox_v_path.c_str());

        //strcpy (str_outdir, out_dir);
        //threedbox_faces.Write(strcat(str_outdir, "/threedbox_e.top"), false);
        std::string threedbox_e_path = out_dir_str + "/threedbox_e.top";
        threedbox_faces.Write(threedbox_e_path.c_str(), false);
    }


    /// 2. efficinet ransac
    if(do_planar_segmentation){
        char str_input[500];
        Efficient_ransac::Parameters ransac_parameters;
        set_parameters(ransac_parameters, probability, min_points, epsilon, cluster_epsilon, normal_thresh); // parameters for S3DIS
        char* input_data_ascii = (char*) strcat(strcpy (str_input, project_dir), "/input_data/room1_spacedel.txt");
        efficient_RANSAC_with_point_access (input_data_ascii, out_lpoints, ransac_parameters,
                                     nb_neighbors,lp_seg_out, estimate_normals);
    }

   // std::string output_seg_laser = outdir + "/lp_segmented.laser";
    //lp_segmented.Write(output_seg_laser.c_str(), false);

}
