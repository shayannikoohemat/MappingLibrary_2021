#include <iostream>
#include <ctime>
#include "InlineArguments.h"
#include <cstdlib>
#include <limits>
#include <LaserPoints.h>
#include "indoorTopology_v2.h"


using namespace std;

void PrintUsage()
{
    printf("Usage: indoor_reconstruction -i <input laserpoints , foo.laser>\n");
    printf("                             -o <output laser file, foo_segmented.laser>\n");
    printf(  "                  -maxd_p <optional: max dist to plane, default: 0.05>\n");
    printf("                    -maxd_s <optional: max dist to surface, default: 0.05>\n");
}

void test_function(char*, char*);

/*void VisulizePlane3D (LaserPoints segment_lp, double max_dist, ObjectPoints &plane_corners,
                      LineTopology &plane_edges, bool verbose);*/

int main(int argc, char *argv[]) {

    //auto     args = new InlineArguments(argc, argv);
    InlineArguments     *args = new InlineArguments(argc, argv);


    std::clock_t start;
    double duration;
    start = std::clock();

    //test_function ();

    /// Check on required input files
/*    if (args->Contains("-usage")) {
        PrintUsage();
        exit(0);
    }

    if (!args->Contains("-i") ||
        !args->Contains("-o"))
    {
        printf("Error: missing programme option.\n");
        PrintUsage();
        return EXIT_SUCCESS;
    }*/

   // test_function (args->String ("-i"), args->String ("-o"));
/*    LaserPoints laserpoints;
    laserpoints.Read("../test.laser");
    segmentation_surfaceGrowing(laserpoints);
    laserpoints.Write("path-to-file/extended_laserpoints_seg.laser", false);*/

    // test the main function "indoor topology"
    ///  /home/shayan/Drive_D/data/test/test.laser
    //laserFile = (char*) "/home/shayan/Drive_D/data/test/test.laser";
    char *laserFile = (char*) "/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/VRR_cc_4cm_1stfloor_2mil_seg10cm.laser";
    LaserPoints laserPoints;
    laserPoints.Read(laserFile);
    char *root_dir = (char*) "/home/shayan/Drive_D/data/Wilhelmapier_VVR/process/out/";
    int minSegmentSize=1000;
    bool verbose=false;
    double flat_angle=10.0; double vertical_angle=10.0;
    double max_intersection_dist=0.10;
    double min_intersection_linelength=0.10;
    bool sort_segments_by_number=true; bool user_defined_ceil_floor_height=false;
    double ceiling_z_userdefined=0.0;
    double floor_z_userdefined=0.0; double ceiling_buffer=1.0; double floor_buffer=0.5;
    double intersection_percentage=0.50; bool apply_intersection_result=false;
    bool wallwall_vertical_dist_condition=false;
    double wallwall_vertical_dist=.30; bool crop_walls_abovefloor_and_belowceiling=false;
    double ceiling_z_average=0.0; double floor_z_average=0.0;
    bool generate_wallpatches=false; double Generate_WallPatches_dist_threshold=0.70;
    double Generate_WallPatches_angle_threshold=0.20;
    double Generate_WallPatches_dist_along_twosegments=0.40;

    indoorTopology(laserFile, root_dir, minSegmentSize, verbose, flat_angle, vertical_angle,max_intersection_dist,
            min_intersection_linelength, sort_segments_by_number, user_defined_ceil_floor_height,
            ceiling_z_userdefined, floor_z_userdefined, ceiling_buffer, floor_buffer,
            intersection_percentage, apply_intersection_result, wallwall_vertical_dist_condition,
            wallwall_vertical_dist, crop_walls_abovefloor_and_belowceiling, ceiling_z_average, floor_z_average,
            generate_wallpatches, Generate_WallPatches_dist_threshold, Generate_WallPatches_angle_threshold,
            Generate_WallPatches_dist_along_twosegments);


    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Total processing time: "<< duration/60 << "m" << '\n';
    std::cout<<"Total processing time: "<< duration << "s" << '\n';


/*    std::cout << "Press ENTER to continue...";
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );*/

    return EXIT_SUCCESS;
}