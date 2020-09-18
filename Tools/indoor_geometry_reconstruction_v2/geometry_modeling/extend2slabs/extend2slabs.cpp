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
    printf("This main calls the function Extend_Walls_Ceiling() to extend walls to slabs(floor OR ceiling)\n");
    printf("Use this function, if walls need to be extended to ceiling or floor.\n");
    printf("Once use it for wall-ceiling and then use the output for wall-floor extension. \n");
    printf("This function generates new walls extendedWall_laserpoints.laser and new min-rectangles as *.top & *.objpts \n");
    printf("Usage: extend_walls_ceiling\n");
    printf("      -iw         [walls_laser] <input walls or merged-walls.laser file, /data/foo.laser> \n");
    printf("      -wpl        [walls_planes] <calculated wall´s planes. If passed empty, then it will be calculated, /data/walls.planes> \n");
    printf("      -isl        [ceiling_laser OR floor_laser] <slab laser file, ceiling or floor> \n");
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
            !args->Contains("-iw") ||
            !args->Contains("-wpl") ||
            !args->Contains("-isl")  ||
            !args->Contains("-msegsize") ||
            !args->Contains("-root_dir"))
    {
        if (!args->Contains("-help")) printf("Error: missing programme option.\n");
        PrintUsage();
        return EXIT_SUCCESS;
    }

    /// read the laser files and plane files from the user input
    LaserPoints wall_lp, slab_lp;
    wall_lp.Read(args->String("-iw"));
    slab_lp.Read(args->String("isl"));
    Planes wall_planes;
    wall_planes.Read(args->String("-wpl"));

    // calling the main function
    Extend_Walls_Ceiling(wall_lp, wall_planes, slab_lp,
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
