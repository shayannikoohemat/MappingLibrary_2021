
#include <iostream>
#include "../../tools/include/utils.h"
#include "../../tools/include/visualization_tools.h"
#include <LaserPoints.h>

int main(){
    std::cout << "hello world" << std::endl;
    //LaserPoints lp;
    //LaserPoints_info(lp, 500);

    char * root_dir;
    ObjectPoints vertices;
    LineTopologies faces;
    LineTopologies_to_OFF(vertices, faces, root_dir);

    return 0;
}

