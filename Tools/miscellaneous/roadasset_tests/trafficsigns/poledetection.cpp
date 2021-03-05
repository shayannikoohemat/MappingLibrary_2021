//
// Created by root on 7/15/20.
//

#include "LaserPoints.h"
#include "../utils/MLS_preprocessing.h"
#include "LaserObjects.h"
#include "LaserPointsFitting.h"
//#include "BuildingPart.h"

// steps: 1. filter the ground 2. conn comp on non-ground points 3. remove large segments and very small segments
// 4. remove large planar segments, buildings and trees 5. look for cylindrical shapes

// we assume the input laser points has already filtered ground points, and has conn compon segmentation.

LaserPoints filter_comp_bySize(const LaserPoints &connComponents, double min_segSize, double max_segSize
                                    , char * root, int class_label){
    char str_root[500];

    LaserPoints filteredPoints;
    vector<LaserPoints> concomps_vec;
    concomps_vec = PartitionLpByTag(connComponents, SegmentNumberTag, root);
    for ( auto &segment : concomps_vec){
        //std::cout << "segment size:" << segment.size() << std::endl;
        if(segment.size() >= min_segSize && segment.size() <= max_segSize){
            segment.SetAttribute(LabelTag, 100); // acceptable segment size
            filteredPoints.AddPoints(segment);
        } else {
            segment.SetAttribute(LabelTag, 102); // filtered by size (large segments: building, trees, ...)
            filteredPoints.AddPoints(segment);
        }
    }

    return filteredPoints;
}

// each component is a conncomp segment
LaserPoints filter_comp_byHeight (const LaserPoints &connComponents, double min_height_threshold, int class_label){
    LaserPoints filtered_by_height;
    vector<LaserPoints> concomps_vec;
    char * root;
    concomps_vec = PartitionLpByTag(connComponents, SegmentNumberTag, root);
    for( auto & component : concomps_vec){
        if(component[0].Attribute(LabelTag) == 100){ // applying it on acceptable segments size with label 100
            DataBoundsLaser db = component.DeriveDataBounds(0);
            double comp_height = abs(db.Maximum().GetZ() - db.Minimum().GetZ()) ;
            if (comp_height >= min_height_threshold){
                component.SetAttribute(LabelTag, class_label); // acceptable height
                filtered_by_height.AddPoints(component);
            }
            else {
                component.SetAttribute(LabelTag, 99); // filtered by height (short segments in height)
                filtered_by_height.AddPoints(component);
            }
        } else filtered_by_height.AddPoints(component);
    }

    return filtered_by_height;
}

LaserPoints lp_localshape(LaserPoints& lp, SegmentationParameters &seg_parameter){

    LaserPoints lp_out;
    vector<LaserPoints> lp_segment_vec;
    char *root;
    lp_segment_vec = PartitionLpByTag(lp, SegmentNumberTag, root);
    for (auto &seg : lp_segment_vec){
        seg.CalculateLocalEigenValues(seg_parameter);
        seg.CalculateLocalShape();
        lp_out.AddPoints(seg);
    }
    return lp_out;
}

vector<LaserPoints> slice_by_height (const LaserPoints &lp, double slice_height){
    vector<LaserPoints> sliced_lp;

    //lp.SortOnCoordinates();
    //DataBoundsLaser db;
    //db = lp.DeriveDataBounds(0);
    //double min_z = db.Minimum().GetZ();

    LaserPoints lp1, lp2, lp3, lp4, lp5, lp6;

    for (auto &p : lp){
        double zVal = p.GetZ();
        if (zVal >= 36.00 && zVal <= 36.10) lp1.push_back(p);
        if (zVal >= 36.40 && zVal <= 36.50) lp2.push_back(p);
        if (zVal >= 36.90 && zVal <= 37.00) lp3.push_back(p);
        if (zVal >= 37.30 && zVal <= 37.40) lp4.push_back(p);
        if (zVal >= 38.00 && zVal <= 38.10) lp5.push_back(p);
        if (zVal >= 39.40 && zVal <= 39.50) lp6.push_back(p);

    }

    lp1.Write("/home/shayan/Kaios/data/Getmapping/Lidar_Pegasus/traffic_signs/lp1.laser", false);
    lp2.Write("/home/shayan/Kaios/data/Getmapping/Lidar_Pegasus/traffic_signs/lp2.laser", false);
    lp3.Write("/home/shayan/Kaios/data/Getmapping/Lidar_Pegasus/traffic_signs/lp3.laser", false);
    lp4.Write("/home/shayan/Kaios/data/Getmapping/Lidar_Pegasus/traffic_signs/lp4.laser", false);
    lp5.Write("/home/shayan/Kaios/data/Getmapping/Lidar_Pegasus/traffic_signs/lp5.laser", false);
    lp6.Write("/home/shayan/Kaios/data/Getmapping/Lidar_Pegasus/traffic_signs/lp6.laser", false);

    return sliced_lp;
}

bool ApproximateCylinder2(
        LaserPoints& laserPoints, int kNN )
    {
    if(laserPoints.size()<10)
    {
    cerr<<"Too few points in ApproximateCylinder\n";
    return 1;
    }

    LaserPoints normalPoints = laserPoints.Normals(kNN);

    //Add extra points on origin.
    int oldSize = normalPoints.size();
    normalPoints.reserve(normalPoints.size()*2);

    for(int i=0; i<oldSize;i++)
    normalPoints.push_back(LaserPoint(0,0,0));

    Vector3D closestPoint, axis;
    double radius;

    //Get the normal to the plane fitted on this Gaussian sphere.
    axis = normalPoints.Normal();

    //Give radius any value.
    radius = 10;

    //Closest point should be well approximated by laserPoints mean.
    closestPoint = laserPoints.Mean();

    //Return with the approximated cylinder
    return 0; //LaserCylinder(laserPoints,axis,closestPoint,radius);
}

///Fit a cylinder to laser points using the given approximation.
/*LaserCylinder FitCylinder(LaserPoints& laserPoints, const LaserCylinder& approximate)
{
    //Check for valid sizes.
    if(laserPoints.size()<10)
    {
        cerr<<"Invalid size in FitCylinder\n";
        return LaserCylinder();
    }
    DEBUG("FitCylinder");

    ENLSIPCylinder fitter(&laserPoints);


    cerr<<"*** Calling ENLSIPCylinder::Optimize ***\n";
    vector<double> final = fitter.Optimize(approximate.ToVector());

    NEWMAT::Matrix covF, covO, covC;
    //cerr<<"Finding covariance...";
    fitter.Covariance(final,1,&covF,&covO,&covC);
    //cerr<<"Done\n";
    cerr<<"Full: "<<covF;
    cerr<<"Obs: "<<covO;
    cerr<<"Cons: "<<covC;

    LaserCylinder ans;
    ans.FromVector(final);
    ans.UpdateBounds(laserPoints);

    return ans;
}*/

///Fit cylinder using RANSAC.
/*int FitCylinderRANSAC(
        LaserPoints& laserPointsIn,
        IndicesVector& selectedIndices,
        Vector3D& in_point1,
        Vector3D& in_point2,
        double& in_dRadius,
        Vector3D& in_axis,
        Vector3D& in_pointOnAxis,
        double &in_dChiSquare,
        bool doApproximation,
        int kNN,
        int nMaxIterations,
        double percentagePoints,
        double ransacIterations)
{
    const bool saveDebugOutput = 1;

    LaserPoints laserPoints = laserPointsIn.Select(selectedIndices);
    for(int i=0;i<laserPointsIn.size();i++)
        laserPointsIn[i].Reflectance() = i*10;

    if(selectedIndices.empty())
        laserPoints = laserPointsIn;

    double minDistance = -1;
    for(int iter=0;iter<ransacIterations;iter++)
    {
        cerr<<"iter: "<<iter<<endl;
        IndicesVector ransacIndices;
        set<int> usedIndices;
        int count = 0;

        for(int i=0;i<laserPoints.size();i++)
        {
            int index = GenerateRandom(0,laserPoints.size()-1);

            if(!usedIndices.count(index))
            {
                usedIndices.insert(index);
                count++;
            }
            if(count>= (percentagePoints*0.01*laserPoints.size()))
                break;
        }

        ransacIndices.resize(usedIndices.size());
        copy(usedIndices.begin(),usedIndices.end(),ransacIndices.begin());


        //ransacIndices = SampleRandomly(laserPoints,percentagePoints*0.01*laserPoints.size());

        cerr<<"Selected points "<<ransacIndices.size()<<" from "<<laserPoints.size()<<endl;

        if(saveDebugOutput)
        {
            char buff[4096];
            sprintf(buff,"ransac_%d.pts",iter);
            laserPoints.Select(ransacIndices).SaveToAscii(buff);
        }

        Vector3D point1, point2;
        double dRadius;
        Vector3D axis, pointOnAxis;
        double dChiSquare;
        LaserPoints sel = laserPoints.Select(ransacIndices);
        LaserCylinder cyl = FitCylinder(sel,ApproximateCylinder2(sel, 20));

        vector<double> distances = cyl.Distance(laserPoints);

        sort(distances.begin(),distances.end());
        double currentDistance = 0;

        for(int k=0;k<(distances.size()/2);k++)
        {
            currentDistance += distances[k];
        }
        cerr<<"currentDistance: "<<currentDistance<<endl;
        if(currentDistance<minDistance || minDistance<0)
        {
            minDistance = currentDistance;
            in_point1 = point1;
            in_point2 = point2;
            in_dRadius = dRadius;
            in_axis = axis;
            in_pointOnAxis = pointOnAxis;
            in_dChiSquare = dChiSquare;
        }
        cerr<<"minDistance: "<<minDistance<<endl<<endl<<endl;
        cerr<<"axis: "<<in_axis;
    }

}*/

LaserPoints candidate_poles (LaserPoints connComponents, double min_radius, double max_radius){
    int kNN = 20;
    LaserPoints normalPoints = connComponents.Normals(kNN);
    //Add extra points on origin.
    int oldSize = normalPoints.size();
    normalPoints.reserve(normalPoints.size()*2);

    for(int i=0; i<oldSize;i++)
        normalPoints.push_back(LaserPoint(0,0,0));
    Vector3D axis = normalPoints.Normal();
    Vector3D closestPoint = connComponents.Mean();
    double radius = 0.055;
    LaserCylinder lcyl;
    LaserCylinder appx(connComponents,axis,closestPoint,radius);
    //appx = ApproximateCylinder(connComponents, kNN);
    lcyl = FitCylinder(connComponents, appx);
    double rad = lcyl.Radius();
    printf("rad: %.4f", rad);
    vector<LaserPoints> concomps_vec;
    char * root;
    concomps_vec = PartitionLpByTag(connComponents, SegmentNumberTag, root);

    //FitCylinderRANSAC()
    LaserPoints candidatePoles;
/*    for (auto &comp : concomps_vec){

       LaserCylinder cylinder_lp, approximate;
        int kNN=20;
       approximate = ApproximateCylinder(comp,kNN); // the default radius is 10 for approximate
       //printf("extimated radius: %.2f\n", approximate.radius) ;
       //printf("approximated cylinder:\n", approximate.AxisDirection());
       cylinder_lp = FitCylinder(comp, approximate);
    }*/

    return candidatePoles;

}