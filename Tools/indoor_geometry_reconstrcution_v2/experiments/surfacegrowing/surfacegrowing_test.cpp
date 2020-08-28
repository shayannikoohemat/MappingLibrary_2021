
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <LaserPoints.h>


void surfacegrowing_test(char* input_file, char* output_file,
        double MaxDistToPlane=0.05, double GrowingRadius=1.0){

    LaserPoints laserpoints;
    laserpoints.Read(input_file);
    SegmentationParameters *seg_parameter;
    seg_parameter = new SegmentationParameters;
    /// segmenting laserpoints
    seg_parameter -> MaxDistanceInComponent()  = 0.3;
    seg_parameter -> SeedNeighbourhoodRadius() = GrowingRadius;
    seg_parameter -> MaxDistanceSeedPlane()    = MaxDistToPlane; // MaX Distance to the Plane
    seg_parameter -> GrowingRadius()           = GrowingRadius;
    seg_parameter -> MaxDistanceSurface()      = MaxDistToPlane;

    /// if segmentation crashes in Windows set the compatibility of generated .exe to windows7
    printf("segmentation process... \n ");
    laserpoints.SurfaceGrowing(*seg_parameter);

}
