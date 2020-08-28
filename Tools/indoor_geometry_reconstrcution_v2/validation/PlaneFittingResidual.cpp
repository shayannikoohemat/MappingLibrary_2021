//
// Created by NikoohematS on 22-3-2018.
//

#include <LaserPoints.h>


/*
 * This set of functions fits a plane to the laserpoints, calculate the residuals and add it as ResidualTag to each point
 * The resisuals is calcualted as: res = d - n.p where d is the distance of the plane to the origin, n is the normal
 * of the plane and p is the 3D vector of the point.
 * */
void PlaneFittingResiduals(LaserPoints &laserpoints, int plane_number){
    Plane plane;
    plane = laserpoints.FitPlane (plane_number);
    Vector3D normal;
    normal = plane.Normal ();
    for (auto &p : laserpoints){
        /// construct a vector3D from the point
        Vector3D pv;
        pv = Vector3D(p.GetX (), p.GetY (), p.GetZ ());
        // signed distance of point to the plane
        double observation;
        observation = plane.Distance () - normal.DotProduct (pv);
        p.FloatAttribute (ResidualTag) = observation;
    }
}

void PlaneFittingResiduals(LaserPoints &laserpoints, const Plane &plane){
    Vector3D normal;
    normal = plane.Normal ();
    for (auto &p : laserpoints){
        /// construct a vector3D from the point
        Vector3D pv;
        pv = Vector3D(p.GetX (), p.GetY (), p.GetZ ());
        // signed distance of point to the plane
        double observation;
        observation = plane.Distance () - normal.DotProduct (pv);
        p.FloatAttribute (ResidualTag) = observation;
    }
}

void PlaneFittingResiduals(LaserPoints &laserpoints, const PointNumberList &pnl){
    Plane plane;
    plane = laserpoints.FitPlane (pnl);
    Vector3D normal;
    normal = plane.Normal ();
    for (auto &p : laserpoints){
        /// construct a vector3D from the point
        Vector3D pv;
        pv = Vector3D(p.GetX (), p.GetY (), p.GetZ ());
        // signed distance of point to the plane
        double observation;
        observation = plane.Distance () - normal.DotProduct (pv);
        p.FloatAttribute (ResidualTag) = observation;
    }
}

/*
 * /// MSE = 1/n Sigma(observed - prediction)^2
 * */
bool MeanSquaredError(LaserPoints &laserPoints, double &MSE){
    // for fitted plane the (observed - prediction) is the distance of each point to the fitted plane
    if(!laserPoints.HasAttribute (ResidualTag)){
        printf ("ERROR: laserpoints don't have residuals, calculate residuals per point \n");
        return false;
    }
    double sum_exp2=0.0;
    int n=0;
    for(auto &p : laserPoints){
        if(p.HasAttribute (ResidualTag)){
            ++n;
            /// power 2 of the residual
            sum_exp2  += pow (p.FloatAttribute (ResidualTag), 2.0);
        }
    }
    MSE = sum_exp2 / n;
    return true;
}


/*
 * /// RMSE = sqrt(1/n Sigma(observed - prediction)^2)
 * */
bool RootMeanSquaredError(LaserPoints &laserPoints, double &RMSE){
    // for fitted plane the (observed - prediction) is the distance of each point to the fitted plane
    if(!laserPoints.HasAttribute (ResidualTag)){
        printf ("ERROR: laserpoints don't have residuals, calculate residuals per point \n");
        return false;
    }
    double sum_exp2=0.0;
    int n=0;
    for(auto &p : laserPoints){
        if(p.HasAttribute (ResidualTag)){
            ++n;
            /// power 2 of the residual
            sum_exp2  += pow (p.FloatAttribute (ResidualTag), 2.0);
        }
    }

    RMSE = sqrt(sum_exp2 / n);
    return true;
}


/*
 * /// Normalized RMSE = sqrt(1/n Sigma(observed - prediction)^2) / (res_max - res_min)
 * */
bool NormalizedRootMeanSquaredError(LaserPoints &laserPoints, double &NRMSE){
    // for fitted plane the (observed - prediction) is the distance of each point to the fitted plane
    if(!laserPoints.HasAttribute (ResidualTag)){
        printf ("ERROR: laserpoints don't have residuals, calculate residuals per point \n");
        return false;
    }

    vector<double> residuals_v;


/*    double res_min1, res_max1;
    laserPoints.AttributeRange (ResidualTag, res_min1, res_max1); /// doesn't return correct min and max
   if(!laserPoints.AttributeRange (ResidualTag, res_min, res_max)) {
        return false;
    }*/

    double sum_exp2=0.0;
    int n=0;
    for(auto &p : laserPoints){
        if(p.HasAttribute (ResidualTag)){
            ++n;
            double res = p.FloatAttribute (ResidualTag);
            /// power 2 of the residual
            sum_exp2  += pow (res, 2.0);
            residuals_v.push_back (res);
        }
    }
    /// min and max are sorted as signed values
    double res_min, res_max;
    if(!residuals_v.empty ()){
        sort(residuals_v.begin (), residuals_v.end ());
        res_min = residuals_v.front ();
        res_max = residuals_v.back ();
    }else return false;

    double RMSE = sqrt(sum_exp2 / n);
    NRMSE = RMSE / fabs(res_max - res_min);
    return true;
}


/*
 * test the function planefittingresidual
 * */
void planefittingresidual_test(LaserPoints &lp, char *output_root ){

    /// collect laserpoints per segment
    vector<int> segment_numbers;
    segment_numbers = lp.AttributeValues(SegmentNumberTag);  // vector of segment numbers
    //std::sort(segment_numbers.begin(), segment_numbers.end());
    //printf ("Number of laser segments: %d\n", segment_numbers.size());

    /// open a file to write results
    char str_root[500];
    strcpy (str_root,output_root);

    FILE *segments_RMS;
    segments_RMS = fopen(strcat(str_root, "segments_RMS.txt"),"w");
    fprintf(segments_RMS, "Segment_num, MSE, RMSE, NRMSE \n");
    fprintf(segments_RMS, "RMSE_Mean at the end of the file! \n");

    vector <double> RMSE_all;
    ///  fitting planes per segment and ...
    for(auto &segment : segment_numbers){
        fprintf(segments_RMS, "%d, ", segment);
        /// selecting points by segment and saving in segment_lpoints
        LaserPoints     segment_lpoints;
        segment_lpoints = lp.SelectTagValue(SegmentNumberTag, segment); /// expensive method
        PlaneFittingResiduals (segment_lpoints, segment);
        double MSE;
        if(MeanSquaredError (segment_lpoints, MSE)){
            fprintf(segments_RMS, "%.4f, ", MSE);
        }

        double RMSE;
        if(RootMeanSquaredError (segment_lpoints, RMSE)){
            fprintf(segments_RMS, "%.4f, ", RMSE);
            RMSE_all.push_back (RMSE);
        }

        double NRMSE;
        if(NormalizedRootMeanSquaredError (segment_lpoints, NRMSE)){
            fprintf(segments_RMS, "%.4f", NRMSE);
        }
        fprintf(segments_RMS, "\n");
    }

    /// calculate the mean of total residuals
    double RMSE_mean;
    if(!RMSE_all.empty ()){
        RMSE_mean = accumulate (RMSE_all.begin (), RMSE_all.end (), 0.0) / RMSE_all.size ();
        fprintf(segments_RMS, "RMSE_Mean: %.4f \n", RMSE_mean);
    }

    fclose(segments_RMS);
    //fclose (almost_horizon_subset_infofile);
}



