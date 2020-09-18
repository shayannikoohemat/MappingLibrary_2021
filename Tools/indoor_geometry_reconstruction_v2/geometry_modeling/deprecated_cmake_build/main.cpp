#include <iostream>
#include "InlineArguments.h"
#include <cstdlib>
#include <limits>
#include <map>
#include <LaserPoints.h>
#include "Plane.h"
#include "../../tools/include/visualization_tools.h"
#include "../../tools/include/utils.h"
#include "../ModelingInteriorWalls.h"

using namespace std;

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

}