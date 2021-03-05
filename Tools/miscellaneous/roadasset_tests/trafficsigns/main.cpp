#include <iostream>
#include <CGAL/pca_estimate_normals.h>
#include "poledetection.h"
#include "pole_decomposition.h"
#include "../utils/MLS_preprocessing.h"
#include "shapeDetection_cgal.h"
#include "../../../indoor_geometry_reconstruction_v2/morphology_voxelspace_spacepartitioning/Morphology/ConnCompSegmentation.h"


void ClassAccuracy(int class_label, char *root, LaserPoints &laserpoints);

int poledetection_pipeline(char *root, LaserPoints &laserPoints);

void testing_RANSAC3DLine(LaserPoints &lpoints, char* root);

void poles(char *long_filter, char *infile,
           char *appendix, char *output_directory,
           bool output_meta_data, bool overwrite,
           int per,
           int nperpole,
           float height,
           int maxpts,
           int numpart,
           float maxdiagpart,
           float diffpos,
           float diffdiag,
           int tag);

int main()
{
    std::clock_t start;
    double duration;
    start = std::clock();
    char str_root[500];
    char *root = (char *) "/home/shayan/DataPartition/Paris_Lille/annotation_test/decomposition/out/";
    LaserPoints lpoints;
    //lpoints.Read((char *)"/home/shayan/DataPartition/Paris_Lille/annotation_test/decomposition/detected_poles.laser");
    //pole_decomposition_fittingLineRANSAC(lpoints, root, 0.15, true);
    //poledetection_pipeline((char *) "/home/shayan/DataPartition/Paris_Lille/annotation_test/process/", lpoints);
/*    char *infile = (char *)"/home/shayan/DataPartition/Paris_Lille/annotation_test/decomposition/dir/detected_poles.laser";
    char *appendix=NULL;
    char *long_filter=NULL;// = (char *)"/home/shayan/DataPartition/Paris_Lille/annotation_test/decomposition/detected_poles.laser";
    int percentiles = 4;
    int nperpole=3;
    float height_of_parts = 0.10;
    float max_num_pntpart = 5000.0;
    float class_min_num_rectparts = 2.0;
    float maxdiag_rectparts = 0.30;
    float diff_posrect_center_2adjrect = 0.05;
    float diff_diag_2adjrect = 0.20;
    int tag =2; // 2 for comp num tag
    poles(long_filter, infile, appendix, root, false, false,  percentiles, nperpole, height_of_parts,
            max_num_pntpart, class_min_num_rectparts, maxdiag_rectparts, diff_posrect_center_2adjrect, diff_diag_2adjrect, tag);*/

    lpoints.Read("/home/shayan/DataPartition/Paris_Lille/annotation_test/decomposition/pole2.laser");
    pole_decomposition_slicing(lpoints, root, 0.30, 0.05,
            SegmentNumberTag, true, 0.15, true);

    testing_RANSAC3DLine(lpoints, root);

    //fit a line to the accepted slices and extend it to the top of the pole and extract the inlier points as the pole
    //LaserPoints accepted_slicePoints;
    //accepted_slicePoints.Read("/home/shayan/DataPartition/Paris_Lille/annotation_test/decomposition/out/accepted_slicePoints.laser");
    //pole_decomposition_fittingLineRANSAC(accepted_slicePoints, root, 0.15, true);


    duration = ( std::clock() - start ) / (double) CLOCKS_PER_SEC;
    std::cout<<"Total processing time: "<< duration << "s" << '\n';

    return 0;
}

void testing_RANSAC3DLine(LaserPoints &lpoints, char* root){

    char str_root[500];
    Line3D line;
    int min_hits, numhits;
    LineSegments3D ransac_linesegments;
    double max_dist_toLine = 0.20;
    min_hits = int(40.0 * lpoints.size() / 100);
    line = lpoints.RANSAC3DLine(max_dist_toLine, min_hits,500, numhits, 1.0, ransac_linesegments);
    printf("min hits: %d <> num hits: %d\n", min_hits, numhits);
    LaserPoints inlierpoints, remaining_points;
    for (auto &point : lpoints) {
        if (line.DistanceToPoint(point.Position3DRef()) <= max_dist_toLine) {
            inlierpoints.push_back(point);
        } else remaining_points.push_back(point);
    }
    ObjectPoints fittedline_objpoints, fittedPoleLines_points;
    LineTopologies fittedline_top, fittedPoleLines_topo;
    ransac_linesegments.PointsWithTopology(fittedline_objpoints, fittedline_top, 1);
    fittedline_top.SetAttribute(LineLabelTag, 1);
    if (!fittedPoleLines_points.empty())
        fittedline_top.ReNumber(fittedline_objpoints, (fittedPoleLines_points.end() - 1)->Number() + 1,
                                (fittedPoleLines_topo.end() - 1)->Number() + 1);
    fittedPoleLines_topo.insert(fittedPoleLines_topo.end(), fittedline_top.begin(), fittedline_top.end());
    fittedPoleLines_points.insert(fittedPoleLines_points.end(), fittedline_objpoints.begin(), fittedline_objpoints.end());
    /// write carrier points and attachment points in separate files
    strcpy(str_root, root);
    inlierpoints.Write(strcat(str_root, "/RANSAC_inlierpoints.laser"), false);
    strcpy(str_root, root);
    remaining_points.Write(strcat(str_root, "/RANSAC_remaining_points.laser"), false);
    /// write lines of accepted pole carriers
    strcpy(str_root, root);
    fittedPoleLines_points.Write(strcat(str_root, "/RANSAC_polecarrier_points.objpts"));
    strcpy(str_root, root);
    fittedPoleLines_topo.Write(strcat(str_root, "/RANSAC_polecarrier_line.top"), false);

}

int poledetection_pipeline(char *root, LaserPoints &laserPoints) {

    char str_root[500];
    // read laser points and apply conn. comp. segmentation
    laserPoints.Read("/home/shayan/DataPartition/Paris_Lille/annotation_test/process/off-ground_points_cls40.laser");

    // first copy gt labels to plane number for later evaluation
    for (auto &p: laserPoints) p.SetAttribute(PlaneNumberTag, p.ScanNumber());
    strcpy(str_root, root);
    laserPoints.Write(strcat(str_root, "/offground_points_cls40_relabel.laser"), false);
    SegmentationParameters *segmentationParameters;
    segmentationParameters = new SegmentationParameters;
    segmentationParameters->DistanceMetricDimension()=2; // for 2D connect component analysis
    segmentationParameters ->MaxDistanceSeedPlane() = 0.5;
    LaserPoints concomp_lp;
    cout<< "Connected Component Analysis ... wait!" << endl;
    concomp_lp = Connected_Component_Segmentation(laserPoints, segmentationParameters);
    concomp_lp.RemoveSmallSegments(SegmentNumberTag, 5);
    strcpy(str_root, root);
    concomp_lp.Write(strcat(str_root, "/conncomp_segmentation.laser"), false);
    //concomp_lp = laserPoints;

    LaserPoints filteredPointsbySize;
   // char *root = "/home/shayan/Kaios/data/Getmapping/Lidar_Pegasus/traffic_signs/";
   // concomp_lp.Read("/home/shayan/Kaios/data/Getmapping/Lidar_Pegasus/traffic_signs/Cylinder_r5cm_crop.laser");


    //concomp_lp.Read("/home/shayan/DataPartition/kavel10_demo/offground_2mil.laser");

    cout<< "filtering ... wait!" << endl;
   /// criteria 1
   LaserPoints remained_fromsize;
   int class_label=3;
    filteredPointsbySize = filter_comp_bySize(concomp_lp, 100, 10000, root, class_label);
    strcpy(str_root, root); // char to string
    filteredPointsbySize.Write(strcat(str_root, "/filteredPointsbySize.laser"), false);

    /// criteria 2
    LaserPoints filteredPointsbyHeight;
    filteredPointsbyHeight = filter_comp_byHeight(filteredPointsbySize, 2, class_label);
    strcpy(str_root, root); // char to string
    filteredPointsbyHeight.Write(strcat(str_root, "/filteredPointsbyHeight.laser"), false);

    /// criteria 3
    //slice_by_height(filteredPointsbyHeight, 0.0); // harcoded

    /// criteria 4 fit a cylinder
   // printf ("# points: \d", concomp_lp.size());
   // candidate_poles (concomp_lp, 0, 0);

   /// evaluation
   /// add ground points and evaluate the class label accuracy
    cout<< "evaluating the results accuracy ... wait!" << endl;
   LaserPoints classified_lp, groundpoints;
   groundpoints.Read("/home/shayan/DataPartition/Paris_Lille/annotation_test/process/ground_points_cls40.laser");
    // first copy gt labels to plane number
    for (auto &p: groundpoints) {
        p.SetAttribute(PlaneNumberTag, p.ScanNumber());
        p.SetAttribute(LabelTag, 1); // 1 for groundpoints
    }
    strcpy(str_root, root);
    groundpoints.Write(strcat(str_root, "/ground_points_cls40_relabel.laser"), false);
   classified_lp = filteredPointsbyHeight;
   classified_lp.AddPoints(groundpoints);
    strcpy(str_root, root);
   classified_lp.Write(strcat(str_root, "/classified_points.laser"), false);

   ClassAccuracy(3, root, classified_lp);

/***********************************************************************************************************************/
   /// check the local shape
/*   LaserPoints lp_out;
   laserPoints.Read("/home/shayan/DataPartition/kavel10_demo/ts_test.laser");
    SegmentationParameters seg_parameter;
    //seg_parameter = new SegmentationParameters;
    seg_parameter.MaxDistanceInComponent() = 0.10;
    seg_parameter.MaxDistanceSeedPlane() = 0.10;
    lp_out = lp_localshape(laserPoints, seg_parameter);
   lp_out.Write("/home/shayan/DataPartition/kavel10_demo/out/ts_test_.laser", false);*/


/**********************************************************************************************************************/
     /// CGAL test
     //char* fname = (char*) "/home/shayan/Kaios/data/Getmapping/Lidar_Pegasus/traffic_signs/Shape_detection/Cylinder_Box_wNormals_sub.xyz";

     // estimate normals
/*     std::list<PointVectorPair> points;
    pca_normal_estimation(fname, points);
    // Saves point set.
    char* output_filename = (char*) "/home/shayan/Kaios/data/Getmapping/Lidar_Pegasus/traffic_signs/Shape_detection/00_wNormals.xyz";
    std::ofstream out(output_filename);
    out.precision(6);
    CGAL::write_xyz_points(
                out, points,
                CGAL::parameters::point_map(CGAL::First_of_pair_property_map<PointVectorPair>()).
                        normal_map(CGAL::Second_of_pair_property_map<PointVectorPair>()));*/
    //efficient_RANSAC_with_point_access(fname);
   // shapeDetection_cgal(fname, true);
    //populating_pointSet(fname);

/**********************************************************************************************************************/

/*    LaserPoints laserPoints;
    laserPoints.Read("/home/shayan/Kaios/data/Getmapping/Lidar_Pegasus/traffic_signs/traffic_signs_samples.laser");
    LaserPoints_info(laserPoints, 4000);*/

    return 0;
}

LaserPoints Connected_Component_Segmentation2D(LaserPoints &laser_points,SegmentationParameters *segmentation_parameters)
{

    // TEMP: Store highest segment number in num_planar_segments
/*    num_planar_segments = 0;
    if (!segmentation_parameters->EraseOldLabels()) {
        int min_segm_num;
        if (!laser_points.AttributeRange(SegmentNumberTag, min_segm_num, num_planar_segments))
            num_planar_segments = 0;
    }*/
    // END TEMP

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

/*
 * Given a point clouds and a predicted class label (e.g. class label =4), it compare the label with
 * ground truth label (given in PlaneNumbertag) as the true label per point. Then it calculates the FP, FN, TP, TN and
 * accuracy measure such as precision, recall, FScore.
 *
 * */
void ClassAccuracy(int class_label, char *root, LaserPoints &laserpoints){

    char str_root[500];
    strcpy (str_root,root); // initialize the str_root with root string

    LaserPoints true_pos_points, true_neg_points, false_pos_points, false_neg_points;


    LaserPoints::iterator p;
    for (p = laserpoints.begin(); p != laserpoints.end(); p++){
        if (p -> Label() == class_label && p -> PlaneNumber() == class_label) true_pos_points.push_back(*p);   // TP
        if (p -> Label() == class_label && p -> PlaneNumber() != class_label) false_pos_points.push_back(*p); // FP
        if (p -> Label() != class_label && p -> PlaneNumber() == class_label) false_neg_points.push_back(*p); // FN
        if (p -> Label() != class_label && p -> PlaneNumber() != class_label) true_neg_points.push_back(*p); // TN
    }

    /// just for debug and check
    true_pos_points.Write(strcat(str_root, "true_pos_points.laser"), false) , strcpy (str_root,root);
    true_neg_points.Write(strcat(str_root, "true_neg_points.laser"), false), strcpy (str_root,root);
    false_pos_points.Write(strcat(str_root, "false_pos_points.laser"), false), strcpy (str_root,root);
    false_neg_points.Write(strcat(str_root, "false_neg_points.laser"), false);

    printf (" \n *** Calculate the precision, recall and fscore *** \n");
    /// calculate the accuracy
    double     population_pos,    /// correct population in a class
    population_neg,    /// other classes population
    true_pos,          /// number of correctly detected hits
    true_neg,         /// number of correctly rejected cases (correctly assigned to other classes)
    false_pos,        /// number of wrongly detected objects in the class (false alarm)
    false_neg;      ///  number of wrongly assigned to other classes (missed objects)
    /// The assumption is points with PlaneNumberTag have correct label
    /// and points with LabelTag have detected or predicted label
    population_pos = laserpoints.SelectTagValue(PlaneNumberTag, class_label).size();
    population_neg = laserpoints.size() - population_pos;

    true_pos  = true_pos_points.size();
    true_neg  = true_neg_points.size();
    false_pos = false_pos_points.size();
    false_neg = false_neg_points.size();


    printf ("population_positive: %.0f \n", population_pos);
    printf ("population_positive by TP + FN: %.0f \n", true_pos + false_neg); // should be equal with population_pos
    printf ("population_negative: %.0f \n", population_neg);
    printf ("true_positive:       %.0f \n", true_pos);
    printf ("true_negative:       %.0f \n", true_neg);
    printf ("false_positive:      %.0f \n", false_pos);
    printf ("false_negative:      %.0f \n", false_neg);

    double  precision,          // correction
    recall1, recall2,   // completeness
    f1score;           // the harmonic mean of precision and recal

    precision =  true_pos / (true_pos + false_pos);
    recall1   =  true_pos / (true_pos + false_neg);
    recall2   =  true_pos / population_pos;
    f1score   =  2 * precision * recall1 / (precision + recall1);

    printf ("precision: %.2f \n", precision);
    printf ("recall :   %.2f \n", recall1);
    printf ("recall2 with total population: %.2f \n", recall2); // should be equall with recal1
    printf ("f1score:   %.2f \n", f1score);
    printf ("PointDensity: %.2f \n", laserpoints.PointDensity());
}

/*void removeDuplicateClassPoints(LaserPoints &lp_predicted_labeltag, LaserPoints &lp_gtruth_planetag){
    LaserPoints lp_all;
    lp_all.AddPoints(lp_predicted_labeltag);
    lp_all.AddPoints(lp_gtruth_planetag);
    lp_all.SortOnCoordinates();
    // remove duplicate points which dont have labeltag. Because if a point is duplicated and doesnt have labeltag it is
    // a predicted point and we remove it. If a point has a label tag and is not duplicate it remains because it is false positive.
    // if a point has both labeltag and planetag but it is duplicated then it should be removed.
    for (auto &p:lp_all){

    }
}*/


