#include <cstring>
#include "main_pipeline.h"
#include <LaserPoints.h>
#include <boost/filesystem.hpp>
#include "intersecting_clippers.h"
#include "face_selection.h"
#include "planar_segmentation.h"
#include "sample_laserpoints.h"
#include "../visualization_tools/visualization_tools.h" ///for converting to OFF format

/*
 *  1. read the data per room (wall+floor+ceiling+pillar+beams)
 *  2. create the data boundingbox (cuboid) --> void bounding_cube()
 *  3. run the planar segmentation (ransac) --> int efficient_RANSAC_with_point_access()
 *  (2 and 3 can be swapped)
 *  4. run the cell decomposition:
 *      4.1 intersect the planes with cuboid faces to extend all planes to the data bounds --> Intersect_Planes_3DBoxFaces()
 *      4.2 then split all intersected faces to create cell decomposition --> SplitPolygons3DByPlanes3D()
 *  5. associating points to faces, this returns points with label of the faces --> updated_labels = associatePointsToFace3D_withTag()
 *  6. sample points on faces without points using Face_to_Voxel_with_noise()
 *  7. convert faces to OFF format --> LineTopologies_withAttr_to_OFF()
 *      in this step, the relation between faces and associated points should be preserved (either in a separate file or in OFF file)
 *  8. (semi-auto) label associated points of each face into valid (part of the structure) and
 *      invalid (not structured objects or points outside of the room)
 *  9.  write lp_segmented as ascii
*/


LaserPoints read_ascii(char *ascii_file);


void room2cellsdecomposition(char *input_ascii, std::string data_dir) {

    char char_arr[500];
    std::string filename, f_name_ext;
    boost::filesystem::path p(input_ascii);
    f_name_ext = p.filename().c_str();    // office1.txt
    filename = p.stem().c_str();        // office1

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// creating necessary directories
    if(!boost::filesystem::create_directories(data_dir + "/out_data"))
        cerr << "Warning! direcotry: " << data_dir + "/out_data" << " exists!" << endl;
    std::string dump_dir = data_dir + "/out_data/dump";
    if(!boost::filesystem::create_directories(dump_dir))
        cerr << "Warning! direcotry: " << dump_dir << " exists!" << endl;

    /// for laserpoints files
    std::string lp_seg_dir = data_dir + "/out_data/lp_seg_dir";
    if(!boost::filesystem::create_directories(lp_seg_dir))
        cerr << "Warning! direcotry: " << lp_seg_dir << " exists!" << endl;
    if(!boost::filesystem::create_directories(lp_seg_dir + "/ascii"))
        cerr << "Warning! direcotry: " << lp_seg_dir + "/ascii" << " exists!" << endl;
    if(!boost::filesystem::create_directories(lp_seg_dir + "/laser"))
        cerr << "Warning! direcotry: " << lp_seg_dir + "/laser" << " exists!" << endl;

    /// for off files
    std::string OFF_dir = data_dir + "/out_data/off";
    if(!boost::filesystem::create_directories(OFF_dir))
        cerr << "Warning! direcotry: " << OFF_dir << " exists!" << endl;

    /// for faces and mesh
    std::string faces_mesh_dir = data_dir + "/out_data/faces_mesh";
    if(!boost::filesystem::create_directories(faces_mesh_dir))
        cerr << "Warning! direcotry: " << faces_mesh_dir << " exists!" << endl;

    /// for 3d boxes of rooms or any input
    std::string threed_boxes_dir = data_dir + "/out_data/threed_boxes";
    if(!boost::filesystem::create_directories(threed_boxes_dir))
        cerr << "Warning! direcotry: " << threed_boxes_dir << " exists!" << endl;

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /// main pipeline ////////

    /// 1. read txt points and convert it to laser. It works with space and comma delimited files and ignores the first line
    /// it expects: xyz rgb Label (for changing columns look at the ReadAscii.cpp)
    LaserPoints laserpoints;
    laserpoints = read_ascii(input_ascii);

    /// parameter settings (better to read this from a json file)
    /// 2. create a threedbox around the data:
    double scalefactor = 1.1;       // 1=fit to the data, 0.9=smaller than the data, 1.1= 0.1 larger than the data
    ObjectPoints threedbox_vertices; //output
    LineTopologies threedbox_faces; // output
    std::string threedbox_off_path = threed_boxes_dir + "/" + filename + "_3dbox.off";
    bounding_cube(strcpy(char_arr, threedbox_off_path.c_str()), laserpoints,
                  scalefactor, threedbox_vertices, threedbox_faces);

    /// 3. planar segmentation (e.g. RANSAC) on txt file
    double  probability     = 0.05 ;
    int     min_points      = 500  ;
    double  epsilon         = 0.02 ;
    double  cluster_epsilon = 0.08 ;
    double  normal_thresh   = 0.087;
    int     nb_neighbors    = 20   ;
    bool    estimate_normals = True ;
    Efficient_ransac::Parameters ransac_parameters;
    LaserPoints lp_segmented;
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

    /// 5 associating points to faces, this returns points with label of the faces and
    /// also valid faces (containing points) vs invalid faces (no points)
    LaserPoints updated_labels;
    LineTopologies faces_with_points, faces_without_points, all_faces;
    // for s3dis: 500, 0.08, 0.5, 500
    double dist_threshold = 0.08; //meter
    double area_threshld = 0.5; //is not used in the function
    int min_points_for_face_selection = 500;
    // NOTE: check ScanLineNumberTag for updated labels of points
    //LineNumberTag(id=16) is used to label the faces as valid(=100) or invalid(101) and also make one *.Top file (all_faces)
    //LineLabelTag(id=0) is used to transfer segment number of points to faces
    updated_labels = associatePointsToFace3D_withTag(lp_segmented, min_seg_size, dist_threshold, area_threshld, min_points_for_face_selection,
                                                     new_polys_v, new_polys_e, faces_with_points, faces_without_points, all_faces);

    /// 6 sample points on faces which don't have points (these can be valid faces where there was a gap/occlusion in data or invalid faces
    /// such as those are created during the cell decomposition)
    LaserPoints sampled_points;
    double vox_l = 0.05;
    double noise_level = 0.025;
    // TODO check if points get the face number as scanlinenumber
    sampled_points = Face_to_Voxel_with_noise(new_polys_v, faces_without_points, (char*) dump_dir.c_str(), vox_l, noise_level);
    sampled_points.SetAttribute(LabelTag, 101);
    updated_labels.SetAttribute(LabelTag, 100);
    //TODO: for invalid faces sample points are generated, these points can given a label and added to the orignila points
    //

    /// 7 convert faces to OFF format
    LineTopologies_withAttr_to_OFF(new_polys_v, all_faces, LineLabelTag, (char*) OFF_dir.c_str(), true);


    /// write all necessary files with the original name of the input file


    /// write the threed boxes of rooms from step2
    /// threedbox_vertices, threedbox_faces
    std::string threedboxes_v_path, threedboxes_f_path;
    threedboxes_v_path = threed_boxes_dir + "/" + filename + "_3dbox.objpts";
    threedboxes_f_path = threed_boxes_dir + "/" + filename + "_3dbox.top";
    threedbox_vertices.Write(strcpy(char_arr, threedboxes_v_path.c_str()));
    threedbox_faces.Write(strcpy(char_arr, threedboxes_f_path.c_str()), false);

    /// write segmented laser file from step3
    ///  the output laserfile will have: xyz, rgb, label, segment_num
    std::string lp_seg_path = lp_seg_dir + "/laser/" + filename + ".laser" ;
    lp_segmented.Write(strcpy(char_arr ,lp_seg_path.c_str()), false);



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
    LaserPoints lp_seg_out;
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
