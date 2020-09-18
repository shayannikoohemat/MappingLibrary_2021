
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <LaserPoints.h>
#include "../include/segmentation.h"


void surfacegrowing_test(char* input_file, char* output_file,
        double MaxDistToPlane, double GrowingRadius){

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

    laserpoints.Write(output_file, false);

}

LaserPoints Connected_Component_Segmentation(char* input_file, char* output_file,
        double MaxDistInComponent)
{

    // TEMP: Store highest segment number in num_planar_segments
/*    num_planar_segments = 0;
    if (!segmentation_parameters->EraseOldLabels()) {
        int min_segm_num;
        if (!laser_points.AttributeRange(SegmentNumberTag, min_segm_num, num_planar_segments))
            num_planar_segments = 0;
    }*/
    // END TEMP
    LaserPoints laser_points;
    SegmentationParameters *segmentation_parameters;
    segmentation_parameters->MaxDistanceInComponent() = MaxDistInComponent;
    // Derive the edges that define the neighbour relations
    TINEdges     *edges;
    edges = laser_points.DeriveEdges(*segmentation_parameters);

    // Remove long edges
    if (segmentation_parameters->MaxDistanceInComponent() > 0.0)
        laser_points.RemoveLongEdges(edges->TINEdgesRef(),
                                     segmentation_parameters->MaxDistanceInComponent(),
                                     segmentation_parameters->DistanceMetricDimension() == 2);

    // Label the connected components
    laser_points.LabelComponents(edges->TINEdgesRef(),
                                 segmentation_parameters->ComponentAttribute(),
                                 segmentation_parameters->EraseOldLabels());

    // Delete the edges
    laser_points.EraseNeighbourhoodEdges();

    // Remove labels of small components
    if (segmentation_parameters->MinNumberOfPointsComponent() > 1)
        laser_points.UnlabelSmallSegments(segmentation_parameters->ComponentAttribute(),
                                          segmentation_parameters->MinNumberOfPointsComponent());

    return laser_points;

}
