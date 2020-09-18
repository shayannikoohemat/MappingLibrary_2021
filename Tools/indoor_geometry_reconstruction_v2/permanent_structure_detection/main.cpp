#include <iostream>
#include <ctime>
#include "InlineArguments.h"
#include <cstdlib>
#include <limits>
#include <LaserPoints.h>
#include "indoorTopology_v2.h"


using namespace std;

/// this is the -help note when calling the function in the command prompt
void PrintUsage()
{
    printf("This function create an adjacency graph from segments and looks for permanent structures from ceilings to walls and floors\n");
    printf("A permanent structure is wall, floor and ceiling.\n");
    printf("Outputs are in output.laser and also individually, e.g. wall.laser\n");
    printf("       no label                    = 0,  white, dark pink\n"
           "            almost horizontal           = 1,  green\n"
           "            almost vertical             = 2,  blue\n"
           "            label wall-slanted-wall     = 3,  sharp yellow\n"
           "            wall                        = 4, orange\n"
           "            floor                       = 5, light yellow\n"
           "            ceiling                     = 6  light blue\n");
    printf("     Tip: press A in pcm GUI to see labels, Press G to see segments, \n"
           "            Press L to open Appearance settings window. \n");

    printf("Usage: indoor_reconstruction \n");
    printf("      -i              [input]    <input laser file , /data/foo.laser>\n");
    printf("      -root_dir       [root_dir] <output processing direcotry, ./process/>\n");
    printf("      -msegsize       [minSegmentSize] <min # of points for a segment to be processed, default: 500>\n");
    printf("      -v              [verbose] <verbose, set true if you need to Debug something>\n");
    printf("      -fang           [flat_angle] <angle threshold to separate almost-horizontal segments, default 10 degrees>\n");
    printf("      -vang           [vertical_angle] <angle threshold to separate almost-vertical segments, default 10 degrees>\n");
    printf("      -maxintdist     [max_intersection_dist] <max dist between two segments to be considered as adjacent in the graph,  default 0.10 m>\n");
    printf("      -minintline     [min_intersection_linelength] <min length of intersected line of two adjacent segment, default 0.10 m>\n");
    printf("      -sortsegments   [sort_segments_by_number] <if true it sorts the segments by segment number in the info-files,default FALSE, NOTE: expensive FLAG>\n");
    printf("      -udFlClz        [user_defined_ceil_floor_height] <if true user should define the ceiling & floor height using the next two parameters, default FALSE>\n");
    printf("      -clZ            [ceiling_z_userdefined] <if previous FLAG is true, set the highest ceiling Z-value here>\n");
    printf("      -flZ            [floor_z_userdefined] <if previous FLAG is true, set the lowest floor Z-value here>\n");
    printf("      -clBuffer       [ceiling_buffer] <within this buffer horizontal segments are considered ceiling, default 1.0 m>\n");
    printf("      -flBuffer       [floor_buffer] <within this buffer horizontal segments are considered floor, default 0.5 m>\n");
    printf("      -int_percentage [intersection_percentage] <percentage of overlap to discard one of the horizontal segments below the other one, default 0.50>\n");
    printf("      -apply_int      [apply_intersection_result] <if true then the lower segments would be excluded from candidate ceilings, default FALSE>\n");
    printf("      -wwv_dist_cond  [wallwall_vertical_dist_condition] <if true two vertical walls (one above the other) with different normals are separated, default FALSE>\n");
    printf("      -wwvert_dist    [wallwall_vertical_dist] <the threshold related to previous FLAG, default 0.30 m>\n");
    printf("      -cropwalls      [crop_walls_abovefloor_and_belowceiling] <if true wall points above the floor and below the ceiling average are cropped, default FALSE>\n");
    printf("      -clZ_ave        [ceiling_z_average] <the threshold related to previous FLAG>\n");
    printf("      -flZ_ave        [floor_z_average] <the threshold related to previous FLAG>\n");
    printf("      -gwp            [generate_wallpatches] <if true it needs three next parameters to generalize segments -> result: buffer_segmentation.laser>\n");
    printf("      -gwp_dist       [Generate_WallPatches_dist_threshold] <distance along the normal direction between two segments to be considered for generalization>\n");
    printf("      -gwp_angle      [Generate_WallPatches_angle_threshold] <angle difference between two segments normal vector to be considered for generalization>\n");
    printf("      -gwp_dist2      [Generate_WallPatches_dist_along_twosegments] <distance along two segments to be considered for generalization>\n");
}

int main(int argc, char *argv[])
{
    InlineArguments *args = new InlineArguments(argc, argv);

    std::clock_t start;
    double duration;
    start = std::clock();

    if (args->Contains("-help")) {
        PrintUsage();
        exit(0);
    }

    // Check on required input files
    if (args->Contains("-help") ||
        !args->Contains("-i") ||
        !args->Contains("-root_dir"))
    {
        if (!args->Contains("-help")) printf("Error: missing programme option.\n");
        PrintUsage();
        return EXIT_SUCCESS;
    }

    // Call the main function
    indoorTopology(args->String("-i"),
                        args->String("-root_dir"),
                        args->Integer("-msegsize", 500),
                        args->Contains ("-v"),
                        args->Double("-fang", 10),
                        args->Double("-vang", 10),
                        args->Double("-maxintdist", 0.10),
                        args->Double("minintline", 0.10),
                        args->Contains("-sortsegments"),
                        args->Contains("-udFlClz"),
                        args->Double("-clZ", 0),
                        args->Double("-flZ", 0),
                        args->Double("-clBuffer", 1.0),
                        args->Double("-flBuffer", 0.5),
                        args->Double("-int_percentage", 0.5),
                        args->Contains("-apply_int"),
                        args->Contains("-wwv_dist_cond"),
                        args->Double("-wwvert_dist", 0.30),
                        args->Contains("-cropwalls"),
                        args->Double("-clZ_ave", 0),
                        args->Double("-flZ_ave", 0),
                        args->Contains("-gwp"),
                        args->Double("-gwp_dist", 0.70),
                        args->Double("-gwp_angle", 20.0),
                        args->Double("-gwp_dist2", 0.40)
                        );

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Total processing time: "<< duration << "s" << '\n';


    return EXIT_SUCCESS;
}

/*
int main(int argc, char *argv[]) {

    std::clock_t start;
    double duration;
    start = std::clock();

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
    double Generate_WallPatches_angle_threshold=20.0;
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


*/
/*    std::cout << "Press ENTER to continue...";
    std::cin.ignore( std::numeric_limits<std::streamsize>::max(), '\n' );*//*


    return EXIT_SUCCESS;
}*/
