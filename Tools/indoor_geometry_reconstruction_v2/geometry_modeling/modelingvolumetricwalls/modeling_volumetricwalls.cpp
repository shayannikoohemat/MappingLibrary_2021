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
    printf("This main calls the function GenerateVolumetricWalls() and Generate_offset_dist_map()\n");
    printf(" It generates boxes in arbotrary orientation from input rectangles (*.objpts and *.top) \n");
    printf("The minimum-rectangle is offset to both sides of its plane within a given offset distance \n");
    printf("The offset distance is a map of rectangle numbers (segment number) and the distance threshold, which for most\n"
           " * of the walls should be the same and for floor and ceiling is different.\n");
    printf("If the map is empty then fix_offset_dist will be used. \n");
    printf("The output is written to the root as new objectpoints and linetopologies.\n");
    printf("In the next step the result can be converted to OFF format using /visualization_tools/LineTopologies_to_OFF() for 3D software.\n");
    printf("Usage: modeling_volumetricwalls\n");
    printf("      -i           [walls_laser] <input laser points (it can be just walls.laser or wall-floor-ceiling.laser), /data/foo.laser> \n");
    printf("      -lwth        [lower_wall_thickness] <lower threshold for wall thickness, default 0.20> \n");
    printf("      -uwth        [upper_wall_thickness] <upper threshold for wall thickness, default 0.40> \n");
    printf("      -vertices    [wall_rectangles_vertices] <vertices.objpts as rectangles vertices> \n");
    printf("      -edges       [wall_rectangle_edges] <edges.top as rectangles edges> \n");
    printf("      -offset_map  [offset_distance_map] <a map of wall thickness (offset_distance)"
           " based on the wall-thickness stored in Residual attributes in each segment. If the map is empty then fix_offset_dist will be used.> \n");
    printf("      -fod         [fix_offset_dist] <for the thickness of walls."
           " If the map is empty then this value multiple by 2 will be used. default is 0.10> \n");
    printf("      -root_dir    [root_dir] <output processing directory, ./process/> \n");
    printf("      -v           [verbose] <for DEBUG mode> \n");

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
            !args->Contains("-vertices") ||
            !args->Contains("-edges")  ||
            !args->Contains("-root_dir"))
    {
        if (!args->Contains("-help")) printf("Error: missing programme option.\n");
        PrintUsage();
        return EXIT_SUCCESS;
    }

    /// read the laser file from the user input
    LaserPoints merged_wall_lp; // or extended_laserpoints.laser
    merged_wall_lp.Read(args->String("-i"));

    std::map <int, double> offset_dist_map;
    /// input values should be the expected wall thickness. Inputs divided by 2 will be offset_distance
    /// if upper and lower values are set to 0 then the function expects the residual attribute of the segments as thickness.
    offset_dist_map = Generate_offset_dist_map (
            merged_wall_lp,
            args->Double("-lwth", 0.20), // set 0 if using merged_surfaces.laser and residuals as thickness
            args->Double("-uwth", 0.40)); // set 0 if using merged_surfaces.laser and residuals as thickness

    ObjectPoints walls_rectangle_vertices;
    LineTopologies walls_edges;
    walls_rectangle_vertices.Read(args->String("-vertices"));
    walls_edges.Read(args->String("-edges"));

    // calling the main function
    GenerateVolumetricWalls (
            walls_rectangle_vertices,
            walls_edges,
            offset_dist_map,
            args->Double("-fod", 0.10),
            args->String("-root_dir"),
            args->Contains("-v"));

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
