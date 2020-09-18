//
// Created by NikoohematS on 5-4-2017.
//

#include <iostream>
#include <cstdlib>
#include <ctime>
#include <LaserPoints.h>

void RemoveDuplicatePoints(LaserPoints lpTarget, LaserPoints lpDuplicate){

    /*
     * NOTE: lpTarget is the bigger file and lpDuplicate is the smaller file.
     * This function just saves points which are in lpTarget and not in lpDuplicate
     * */
    /// read target laser points
    char* laserFile;
    laserFile = (char*) "D://test//indoor_reconstruction//data//accuracy_check//all_correct_labels_planenr_1710k.laser";  //Target_13442
    lpTarget.Read(laserFile);


    /// read duplicate points
    char* pOtherFile;
    pOtherFile = (char*) "D://test//indoor_reconstruction//data//accuracy_check//all_correct_labels_planenr_1697k.laser";  //Duplicate_9898   Duplicate_12814 //Duplicate_552
    lpDuplicate.Read(pOtherFile);

    lpTarget.SortOnCoordinates();
    lpDuplicate.SortOnCoordinates();



    //lpTarget.Write("D://test//indoor_reconstruction//data//accuracy_check//Target2_sorted.laser", false);
    //lpDuplicate.Write("D://test//indoor_reconstruction//data//accuracy_check//Duplicate2_sorted.laser", false);

    LaserPoints notMatched_points;
    LaserPoints::iterator match_begin, match_end;
    //match_begin = std::find(lpTarget.begin(), lpTarget.end(), *lpDuplicate.begin());

    LaserPoints::iterator ptarget, pduplicate;
    ptarget = lpTarget.begin();
    pduplicate = lpDuplicate.begin();
    int index =0;
    while (ptarget != lpTarget.end() && pduplicate != lpDuplicate.end()){
        index++;

        if (fabs(ptarget->X() - pduplicate->X()) < 0.0001 &&
                fabs(ptarget->Y() - pduplicate->Y()) < 0.0001 &&
                fabs(ptarget->Z() - pduplicate->Z()) < 0.0001)
        {
            // if points are equal
            ptarget++;
            pduplicate++;
        }else {
            // if points are not equal
            ptarget ->SetAttribute(LabelTag, 88);  // make sure we don't use a reserved label
            ptarget++;
        }

        if (pduplicate == lpDuplicate.end()-1){
            notMatched_points.insert(notMatched_points.begin(), ptarget, lpTarget.end());
        }
    }
    cout << index << endl;

    LaserPoints points_tmp;
    LaserPoint point;
    points_tmp = lpTarget.SelectTagValue(LabelTag, 88);
    notMatched_points.AddPoints(points_tmp);

    notMatched_points.Write("D://test//indoor_reconstruction//data//accuracy_check//notMatched_points.laser", false);
}


/*
 * Given a point clouds and a predicted class label (e.g. wall =4), it compare the label with
 * 2nd label (given in PlaneNumbertag) as true label per point and calculate the FP, FN, TP, TN and
 * accuracy measure such as precision, recall, FScore.
 *
 * */
void WallAccuracy(int class_label){

    char str_root[500];
    //char *root = (char*) "D://test//publication//TUM_testresult//";
    char *root = (char*) "D://test//indoor_reconstruction//data//groundtruth//groundtruth_withlabel//"
            "label_results_truelabels//results//";
    strcpy (str_root,root); // initialize the str_root with root string

    /// read laser points
    LaserPoints laserpoints;
    char* laserFile;
    laserFile = (char*) "D://test//indoor_reconstruction//data//groundtruth//groundtruth_withlabel//label_results_truelabels//1st_iteration//"
            "all_flcl_unknown_clutter_wall_truelabel_1stiter.laser";
            //"all_occlusiontest_trulabel.laser";
            //"wallsfloorceiling_pointbehsurf_clutter_result_and_truelabels_2400K_final_new.laser";
    laserpoints.Read(laserFile);

    /// read detected walls
    LaserPoints walls_points;
    char* walllaserFile;
    //walllaserFile = (char*) "D://test//indoor_reconstruction//data//accuracy_check//walls_2nditer.laser";
    //walls_points.Read(walllaserFile);

    LaserPoints true_pos_points, true_neg_points, false_pos_points, false_neg_points;

    class_label =6;
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