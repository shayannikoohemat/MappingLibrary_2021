//
// Created by NikoohematS on 22-3-2018.
//

#ifndef INDOOR_GEOMETRY_REOCNSTRUCTION_PLANEFITTINGRESIDUAL_H
#define INDOOR_GEOMETRY_REOCNSTRUCTION_PLANEFITTINGRESIDUAL_H

#endif //INDOOR_GEOMETRY_REOCNSTRUCTION_PLANEFITTINGRESIDUAL_H

#include <LaserPoints.h>

/*
 * This set of functions fits a plane to the laserpoints, calculate the residuals and add it as ResidualTag to each point
 * The resisuals is calcualted as: res = d - n.p where d is the distance of the plane to the origin, n is the normal
 * of the plane and p is the 3D vector of the point.
 * */

void PlaneFittingResiduals(LaserPoints &laserpoints, int plane_number);

void PlaneFittingResiduals(LaserPoints &laserpoints, const Plane &plane);

void PlaneFittingResiduals(LaserPoints &laserpoints, const PointNumberList &pnl);

/// MSE = 1/n Sigma(observed - prediction)^2
bool RootMeanSquaredError(LaserPoints &laserPoints, double &RMSE);

/// RMSE = sqrt(1/n Sigma(observed - prediction)^2)
bool RootMeanSquaredError(LaserPoints &laserPoints, double &RMSE);

/// Normalized RMSE = sqrt(1/n Sigma(observed - prediction)^2) / (res_max - res_min)
bool NormalizedRootMeanSquaredError(LaserPoints &laserPoints, double &NRMSE);

/// test function
void planefittingresidual_test(LaserPoints &lp, char *output_root );

