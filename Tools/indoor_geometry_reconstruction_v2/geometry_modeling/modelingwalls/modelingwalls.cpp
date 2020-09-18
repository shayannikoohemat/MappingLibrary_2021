#include <iostream>
#include <ctime>
#include <cstdlib>
#include <limits>
#include <map>
#include <LaserPoints.h>
#include "InlineArguments.h"
#include "../ModelingInteriorWalls.h"

using namespace std;

void PrintUsage()
{
    printf("This main calls the function ModelingInteriorWalls() to extend walls-floors-ceilings to each other \n"
           "and create new min-rectangles. \n");
    printf("Preferably generalize the input by MergeSurfaces() from utils, then use the planes output for this function.\n"
    " MergeSurfaces() makes sure both side of a wall are merged into one segment. \n");
    printf("This function generates new walls extended.laser and new min-rectangles as *.top & *.objpts \n"
           "A minimum_enclosing_rectangle_3D will be generated for all segments (in 3D e.g. slanted walls, ramps)\n");
    printf("Note: Thickness of walls is stored in residuals attribute\n");
    printf("Usage: modelingWalls\n");
    printf("      -i         [walls_laser] <input laser points (it can be just walls.laser or wall-floor-ceiling.laser), /data/foo.laser> \n");
    printf("      -wpl        [walls_planes] <calculated wall´s planes from MergeSurface.cpp. If passed empty, then it will be calculated, /data/walls.planes> \n");
    printf("      -max_intdist        [max_intersection_dist] <max dist for intersection.\n"
           " The intersection_dist is calculated from the intersection line to each segment> \n");
    printf("      -msegsize   [minSegmentSize] <min # of points for a segment to be processed.> \n");
    printf("      -root_dir   [root_dir] <output processing directory, ./process/> \n");
    printf("      -v   [verbose] <for DEBUG mode> \n");

}

int main(int argc, char *argv[]){

    InlineArguments *args = new InlineArguments(argc, argv);

    std::clock_t start;
    double duration;
    start = std::clock();

    if(args->Contains("help")) {
        PrintUsage();
        exit(0);
    }

    // check on required arguments
    if(args->Contains("-help") ||
            !args->Contains("-i") ||
            !args->Contains("-wpl") ||
            !args->Contains("-max_intdist")  ||
            !args->Contains("-msegsize") ||
            !args->Contains("-root_dir"))
    {
        if (!args->Contains("-help")) printf("Error: missing programme option.\n");
        PrintUsage();
        return EXIT_SUCCESS;
    }

    /// read the laser files and plane files from the user input
    LaserPoints wall_lp;
    wall_lp.Read(args->String("-i"));
    Planes wall_planes;
    wall_planes.Read(args->String("-wpl"));

    // calling the main function
    ModelingInteriorWalls(wall_lp, wall_planes,
                    args->Double("-max_intdist", 0.5),
                    args->Integer("-msegsize", 500),
                    args->String("-root_dir"),
                    args->Contains("-v")
                        );

    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Total processing time: "<< duration << "s" << '\n';

    return EXIT_SUCCESS;

}



/*
int main() {

    char* laserFile;
    laserFile = (char*) "../../../test_data/room_sample_seg.laser";
    LaserPoints lp;
    lp.Read(laserFile);
    Planes planes;
    planes.Read( (char*) "../../../merged_planes.planes");
    double max_intersection_dist = 0.5;
    char *root = (char*) "../../../test_data/";
    ///this generates extended.laser and rectangles.objpts and thickness of walls in residuals attribute per segment.
    ModelingInteriorWalls (lp, planes, max_intersection_dist, 100, root, false);

}*/
