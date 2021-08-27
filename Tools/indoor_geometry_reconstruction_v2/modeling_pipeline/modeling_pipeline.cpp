#include <cstring>
#include "modeling_pipeline.h"
#include <LaserPoints.h>
#include <boost/filesystem.hpp>
#include "planar_segmentation.h"
#include "../visualization_tools/visualization_tools.h"


LaserPoints read_ascii(char *ascii_file);

/// input file should be ascii *.txt with x y z
/// it reads *.las and *.laser as well
int threedmodeling(char* input_file, std::string data_dir, bool do_segmentation)
{
    char char_arr[500];
    std::string filename, f_name_ext, file_ext;
    boost::filesystem::path p(input_file);
    f_name_ext  = p.filename().c_str();    // office1.txt
    filename    = p.stem().c_str();        // office1
    file_ext    = p.extension().c_str();   // txt

    /// direcotry for dummy files and planar segmentation
    std::string dump_dir = data_dir + "/dump";
    if(!boost::filesystem::create_directories(dump_dir))
        cout << "Warning! direcotry: " << dump_dir << " exists!" << endl;

    std::string laser_dir = data_dir + "/laser";
    if(!boost::filesystem::create_directories(laser_dir))
        cout << "Warning! direcotry: " << laser_dir << " exists!" << endl;

    /////////////////////////////////////////////////////////////////////////////////////
    /// STEP1: reading input data
    LaserPoints laserpoints;
    if(file_ext == ".txt" || file_ext == ".pts" || file_ext == ".xyz"){
        laserpoints = read_ascii(input_file);
    }
    if(file_ext == ".las" || file_ext == ".laser"){
        laserpoints.Read(input_file);
    }
    std::string laserpoints_path = laser_dir + "/" + filename + ".laser" ;
    laserpoints.Write(strcpy(char_arr ,laserpoints_path.c_str()), false);

    if (do_segmentation){
        /// STEP2: planar segmentation (e.g. RANSAC) or surface growing on .laser file
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
        if (file_ext == ".txt" || file_ext == ".pts" || file_ext == ".xyz")
            cout << "input file:" << input_file << endl;
        else
            cerr << "input file for segmentation should be ascci: .txt, pts or xyz and space delimited!" << endl;

        efficient_RANSAC_with_point_access (input_file, dump_dir, ransac_parameters,
                                     nb_neighbors, lp_segmented, estimate_normals);
        /// write segmented laser file, the output laserfile will have: xyz, rgb, label, segment_num
        std::string lp_seg_path = laser_dir + "/" + filename + "_seg.laser" ;
        lp_segmented.Write(strcpy(char_arr ,lp_seg_path.c_str()), false);

        /// write segmented ascii file
        FILE *lp_seg_ascii;
        std::string lp_seg_path2 = laser_dir + "/" + filename + "_seg.txt" ;
        lp_seg_ascii = fopen(strcpy(char_arr, lp_seg_path2.c_str()),"w");
        fprintf(lp_seg_ascii, "x, y, z, segment_num \n");
        if (lp_segmented.HasAttribute(SegmentNumberTag)){
            for(auto &p : lp_segmented){
                 fprintf(lp_seg_ascii, "%.2f, %.2f, %.2f, %d \n", p.X(), p.Y(), p.Z(), p.Attribute(SegmentNumberTag));
            }
        } else
            cerr << "points don't have segment number to convert to ascii!!!" << endl;
        fclose(lp_seg_ascii);
    }


    /// direcotries for adj graph and classification files
    std::string topology_dir = data_dir + "/indoor_topology";
    if(!boost::filesystem::create_directories(topology_dir))
        cout << "Warning! direcotry: " << topology_dir << " exists!" << endl;
    if(!boost::filesystem::create_directories(topology_dir + "/process"))
        cout << "Warning! direcotry: " << topology_dir + "/process" << " exists!" << endl;
    if(!boost::filesystem::create_directories(topology_dir + "/results"))
        cout << "Warning! direcotry: " << topology_dir + "/results" << " exists!" << endl;

    /// direcotry for 3d models
    std::string modeling_dir = data_dir + "/indoor_modeling";
    if(!boost::filesystem::create_directories(modeling_dir))
        cout << "Warning! direcotry: " << modeling_dir << " exists!" << endl;

}
