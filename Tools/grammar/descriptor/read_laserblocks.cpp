//
// Created by NikoohematS on 28-8-2017.
//

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "LaserBlock.h"

using namespace std;

int read_laserblocks(char *in_file, char * out_file) {
    LaserUnit::iterator      points;
    LaserBlock               block;
    LaserBlock::iterator     strip;
    int  istrip;

    //char *in_file;
    //in_file = (char *) "diemen1_s_rect_L01m.block"; //"diemen1_s_rectm.block";

    // Set up the block to be processed.
    if (!block.ReadMetaData(in_file)) {
        printf("Error reading block meta data file %s\n", in_file);
        exit(0);
    }

    // Loop over all strips
    LaserPoints lp;
    for (strip=block.begin(), istrip=0; strip!=block.end(); strip++, istrip++) {
        // Loop over all strip parts
        for (points=strip->begin(); points!=strip->end(); points++) {

            // Read points
            if (!points->Read(points->PointFile(), false)) {
                printf("Error reading points from file %s\n", points->PointFile());
                exit(0);
            }
            lp.AddPoints(*points);
            printf("number of points: %d\n",points->size());
        }
    }
    lp.Write(out_file, false);
    return(0);
}
