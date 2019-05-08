#include <iostream>
#include <sstream>
#include <ctime>
#include <limits>
#include "LaserPoints.h"
#include "Mls_Preparation.h"
#include "indoor_graph.h"


//int family_tree();
int traverse();
int track_edges();
int labeledgraph();
int graphvis_out();
int graphviz2(void);
int graphvis_writelabels();


void selectpyramidpart(char *pyramid_filename, char *directory,
                       int level, int first_row, int last_row,
                       int first_column, int last_column);
int read_laserblocks(char *in_file, char * out_file);


int main() {

    /// time
    std::clock_t start;
    double duration;
    start = std::clock();

    /// input arguments
    char *directory, *output;
    directory =  (char*)  "E:/Laser_data/Stanford_annotated/";
    output =  (char*)     "E:/Laser_data/Stanford_annotated/out";

    char *pyramid_filename, *out_dir;
    pyramid_filename = (char *) "E:\\BR_data\\Diemen\\diemen_1\\reconstructed\\diemen1_s_rectm.pyramid";
    out_dir          = (char *) "E:\\BR_data\\Diemen";
    //selectpyramidpart(pyramid_filename, out_dir, 8, 0, 0 , 0, 0);
    char *in_file, *outfile;
    in_file = (char *) "diemen1_s_rect_L02m.block";
    outfile = (char *) "E:\\BR_data\\Diemen\\diemen_1\\reconstructed\\out_laser\\block2.laser";
    //read_laserblocks(in_file, outfile);


    /// step 1
   //ReducePointsBatch(directory, output, 4.0);

    /// step 2
    //MergeLaserFiles(output, directory);

    // step 3
    //LaserPoints lpoints;
    //lpoints.Read("E:/Laser_data/Stanford_annotated/segmented/annotated_10mil.laser");
    //output =  (char*) "E:/Laser_data/Stanford_annotated/segmented/";
    //SegmentationAndRefinement(lpoints, output, 100, 0.20, 0.10, 1, 0);

    // step 4
    //LaserPoints lp;
    //lp.Read("E:/Laser_data/Stanford_annotated/segmented/relabeled_segments.laser");
    //output =  (char*) "E:/Laser_data/Stanford_annotated/segmented/relabeledpoints_segments";
    LaserPoints segmented_laserpoints;
    //segmented_laserpoints.Read("E:/Laser_data/Stanford_annotated/crop/relabeled_segments_2526.laser");
    //CollectSegmentLaserpoints(segmented_laserpoints, output, true);  // too expensive
    //LpToSegmentLpUsingBoost(segmented_laserpoints);
    //PartitionLpByTag1Tag2(segmented_laserpoints, LabelTag, Label2Tag, output);
    //PartitionLpByTag(segmented_laserpoints, Label2Tag, output);


    //step 5
    //LaserPoints segmented_laserpoints, relabeledlp;
    SceneGraph sg;
    //relabeledlp.Read("E:/Laser_data/Stanford_annotated/crop/graph_test_notrelabeled.laser");
    //segmented_laserpoints.Read("E:/Laser_data/Stanford_annotated/crop/office25_26_segmented5cm.laser"); // office25_26_segmented5cm.laser"
    //SegmentToObject(segmented_laserpoints, output, false, true, true);
    //SegmentedObjectsToGraph(relabeledlp, output, false, true, true);
    //sg = Build_SceneGraph(segmented_laserpoints, output, false, true, true);


    // function 6
    char *infile, *dir;
    infile = (char*) "E:/Laser_data/Stanford_annotated/paraview/office2526_divisions/153025.csv";
    dir =  (char*) "E:/Laser_data/Stanford_annotated/paraview/office2526_divisions";
    vector<vector<string>> obj_groups, obj_groups_batch;
    //obj_groups = read_object_groups(infile, 1,2,7, true);
    //obj_groups_batch = read_object_groups_batch(dir, 1, 2, 7, true, ".csv");
    //translate_groups_to_objectnames(obj_groups_batch, true);
    //build_hierarchy_graph(sg, obj_groups_batch, output, true);

    // function7
    char* asciipcdfile;
    asciipcdfile =  (char*) "E:/Laser_data/Stanford_annotated/supervoxel/supervoxels_labeledcloud.pcd";
    output    =  (char*) "E:/Laser_data/Stanford_annotated/supervoxel/supervoxels_labeledcloud.laser";
    LaserPoints lp;
    //read_pcd_ascii(asciipcdfile, lp, output);
    //read_pcd_bin(asciipcdfile, lp, output);

    /// function 8
    char* origin, *merged;
    origin =  (char*) "E:/Laser_data/Stanford_annotated/supervoxel/office_2526_Cloud.laser";
    merged    =  (char*) "E:/Laser_data/Stanford_annotated/supervoxel/supervoxels_labeledcloud.laser";
    LaserPoints lp_origin, lp_merged;
    lp_origin.Read(origin);
    lp_merged.Read(merged);
    merge_attributes(lp_origin, lp_merged, SegmentNumberTag);
    lp_origin.Write("E:/Laser_data/Stanford_annotated/supervoxel/office_2526_Cloud_mergedAtt.laser", false);


    /// boost::tree function
    //family_tree();
    //traverse();
    //track_edges();
    //graphvis_out();
    //graphviz2();
    //graphvis_writelabels();


    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    printf("Total processing time: %.3f m \n", duration/60);

    std::cout << "Press ENTER to continue...";
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );
}