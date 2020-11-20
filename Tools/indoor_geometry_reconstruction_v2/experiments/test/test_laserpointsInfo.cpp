//
// Created by root on 6/25/20.
//


#include <iostream>
#include <ctime>
#include <LaserPoints.h>
#include "../../utils/utils.h"

using namespace std;

int main(){

    // computation time
    std::clock_t start;
    double duration;
    start = std::clock();

    LaserPoints lp;
    //lp.Read("/data/laserpoints.laser");
    lp.Read("/home/shayan/DataPartition/Paris_Lille/test/Lille-poles.laser");
    LaserPoints_info(lp, 50);


    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Total processing time: "<< duration/60 << "m" << '\n';


    return EXIT_SUCCESS;
}
