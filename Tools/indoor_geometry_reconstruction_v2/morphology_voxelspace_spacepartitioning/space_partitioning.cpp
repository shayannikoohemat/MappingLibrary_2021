//
// Created by NikoohematS on 13-6-2018.
//

#include "space_partitioning.h"
#include "indoor_reconstruction.h"
#include "KNNFinder.h"

// Create a horizontal plane
//horizontal_plane = Plane(Vector3D(0.0, 0.0, 0.0), Vector3D(0.0, 0.0, 1.0));

LaserPoints Project_Spacepartions_To_2D(LaserPoints space_lp) {


    //LaserPoints projected_lp;
    // projected_lp = space_lp.ProjectToPlane (zvector); throws an error

    Vector3D zvector(0, 0, 1);
    //zvector.X () = 0.0; zvector.Y () = 0.0; zvector.Z () = 1.0;

    Vector3D xv,yv;
    zvector.PerpendicularVectors(xv, yv);

    LaserPoints pts;
    for(int k=0;k<space_lp.size ();k++)
    {
        LaserPoint p;
        double dx = xv.DotProduct(space_lp[k]);
        double dy = yv.DotProduct(space_lp[k]);
        p.X () = dx; p.Y () = dy; p.Z () = 0.0; p.Reflectance () = 1;
        if (space_lp.HasAttribute (SegmentNumberTag)){
            p.SegmentNumber () = space_lp[k].SegmentNumber ();
        }
        pts.push_back (p);
       // pts.push_back(LaserPoint(dx,dy,0,1)); /// throws an error
    }

    return pts;
}


/// this function always doesn't generate correct result
/// it is borrowed from Visualization tools and LaserScan
/// segmented_lp is preferred for better result
void EnclosingPolygon_test (LaserPoints lp){

    lp.Read("D:/test/space_parititioning/seg16.laser");
    for(auto &p : lp) p.Z () =0.0; /// make all Z-values 0
    lp.Write("D:/test/space_parititioning/lp_z0.laser", false);

    lp.RemoveDoublePoints (); /// very important to generate TIN
    lp.DeriveDataBounds (0);
    lp.DeriveTIN ();

    ObjectPoints vertices;
    LineTopologies polygon;
    LineTopology lines;

    /// if the lp has segment_number this segmentation parameters are not used
    SegmentationParameters *seg_parameter;
    seg_parameter = new SegmentationParameters;  /// initialization
    // seg_parameter -> MaxDistanceInComponent()  = 0.3;

    OutliningParameters outliningParameters;
    // outliningParameters.MaximumDistancePointIntersectionLine () = 0.20;
    // outliningParameters.MinimumAnglePreferredDirections ()      = 20.0;
    lp.EnclosingPolygon (vertices, lines, *seg_parameter, outliningParameters, true);

    polygon.push_back (lines);
    vertices.Write("D:/test/space_parititioning/vertices.objpts");
    polygon.Write("D:/test/space_parititioning/polygon.top", false);
}

LaserPoints Intersect_Spacepartitions_Trajectory (LaserPoints spacepartitions_lp, LaserPoints trajectory, double dist_threshold){

    /// per each segment in spaces_lp, find the closest traj point, if the distance is less than
    ///  vox_l or dist_threshold size then it means there is
    ///  an intersection in 3D between the sapce_partition and the trajectory

    vector<LaserPoints> spacepartitions_v;
    spacepartitions_v = PartitionLpByTag (spacepartitions_lp, SegmentNumberTag);

    //finder.FindKnn (trajectory, 1, dist, index, EPS_DEFAULT);
    LaserPoints valid_spacepartitions;
    for(auto &partition : spacepartitions_v){
        /// for debug
        int seg_nr;
        seg_nr = partition[0].SegmentNumber ();
        printf("Partition Number: %d \n", seg_nr);

        /// make a KNN from the partition_lp
        KNNFinder <LaserPoint> finder(partition);
        vector<double> dist;
        vector<int> index;
        /// find the closest trajectory to this partition and get the distance
        if(finder.FindKnn (trajectory, 1, dist, index, EPS_DEFAULT)){
            std::sort(dist.begin (), dist.end ());
            double nearest_traj_dist;
            nearest_traj_dist = dist[0];
            printf("Closest Traj Point: %3.2f \n", nearest_traj_dist);
            /// if the distance is short means there is an intersection and the space partition is inside the building
            if(nearest_traj_dist <= dist_threshold){
                valid_spacepartitions.AddPoints (partition);
            }
        }
    }
    return valid_spacepartitions;
}
