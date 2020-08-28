//
// Created by root on 6/18/20.
//

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <LaserPoints.h>
#include "../utils/utils.h"

// set the specific parameters for each function in the code, currently they are hard-coded
// basically these are just call functions of the main library to facilitate the data handling
/// WARNINGS: in Windows if segmentation crashes set the compatibility of generated exe to windows7, 10 ...
void data_preparation (LaserPoints &laserPoints, LaserPoints &out_laserPoints, char* root_dir,
        bool do_surface_growing=false, bool do_segment_refinement=false,
        bool down_sampling=false, bool ascii_to_laser=false, bool laser_to_ascii=false) {


    std::clock_t start;
    double duration;
    start = std::clock();

    char str_root[500];
    // initialize the str_root with root string
    strcpy (str_root, root_dir); // this initialization should be repeated after each use
    //laserPoints.Write(strcat(str_root,"name-to-concatinate.laser"), false);


    if (ascii_to_laser){
        /// do it here TODO
        //laserPoints.LoadFromAscii();
    }

    if (laser_to_ascii){
        /// do it here TODO
        //laserPoints.SaveToAscii();
    }


    if (down_sampling){

        /// do it here TODO
        //laserPoints.SubSample();
    }


    /// segmenting laserpoints
    SegmentationParameters *seg_parameter;
    seg_parameter = new SegmentationParameters;
    if (do_surface_growing) {
        seg_parameter->MaxDistanceInComponent() = 0.3;
        seg_parameter->SeedNeighbourhoodRadius() = 1.0;
        seg_parameter->MaxDistanceSeedPlane() = 0.10; // MaX Distance to the Plane
        seg_parameter->GrowingRadius() = 1.0;
        seg_parameter->MaxDistanceSurface() = 0.10;

        /// if segmentation crashes set the compatibility of generated exe to windows7
        printf("segmentation process... \n ");
        laserPoints.SurfaceGrowing(*seg_parameter);
        out_laserPoints = laserPoints;
    }

    /// remove long edges by making a TIN from segments and renumber segments, it can be expensive.
    if (do_segment_refinement){
        int minsizesegment = 100; // #points
        double maxDistanceInComp = 0.30; //meter
        printf("Refining segments larger than %d points.\n", minsizesegment);
        /// resegmenting laserpoints by removing longedges in the TIN and removing deformed segments
        LaserPoints refined_lp;
        out_laserPoints = segment_refinement(laserPoints, minsizesegment, maxDistanceInComp);
    }



    // processing time
    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Total processing time: "<< duration/60 << "m" << '\n';
    /// no code after here

}


/// hard coded parameters are fine, just change MaxDistanceSeedPlane() depends on your data and point spacing
/// This code adds segment attributes to the input laser data
void segmentation_surfaceGrowing(LaserPoints &laserpoints){
    SegmentationParameters *seg_parameter;
    seg_parameter = new SegmentationParameters;
    /// segmenting laserpoints
    seg_parameter -> MaxDistanceInComponent()  = 0.3;
    seg_parameter -> SeedNeighbourhoodRadius() = 1.0;
    seg_parameter -> MaxDistanceSeedPlane()    = 0.10; // MaX Distance to the Plane
    seg_parameter -> GrowingRadius()           = 1.0;
    seg_parameter -> MaxDistanceSurface()      = 0.10;

    /// in Windows if segmentation crashes set the compatibility of generated exe to windows7
    printf("segmentation process... \n ");
    laserpoints.SurfaceGrowing(*seg_parameter);
}
