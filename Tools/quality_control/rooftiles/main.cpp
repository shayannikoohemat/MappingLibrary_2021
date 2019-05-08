#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include "InlineArguments.h"
#include "Positions3D.h"
#include "ObjectPoints.h"
#include "LineTopologies.h"
#include "LaserPoints.h"
#include "GeneralUtility.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: rooftiles -l <laser points file with a roof in two strips>\n");
  printf("                 -r <object points file with ridge end points\n");
  printf("                 -o <statistics file>\n");
  printf("                 [-mind <double>] (minimum distance to ridge line, def: 1 m)\n");
  printf("                 [-maxd <double>] (maximum distance to ridge line, def: 10 m)\n");
  printf("                 [-incd <double>] (distance increment, def: 1 m)\n");
  printf("                 [-minn <integer>] (minimum number of points, def: 10)\n");
  printf("                 [-maxn <integer>] (maximum number of points, def: 100)\n");
  printf("                 [-incn <integer>] (number of points increment, def: 10)\n");
  printf("                 [-niter <integer>] (number of iterations, def: 25)\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  FILE            *fdo;
  ObjectPoints    ridge_points;
  Position3D      midpoint, proj_midpoint1, proj_midpoint2;
  vector<LaserPoints *> segments, subsets;
  vector<LaserPoints *>::iterator segment, subset;
  LaserPoints     laser_points, segment1, segment2, segment3, segment4,
                  subset1, subset2, subset3, subset4;
  LaserPoints::iterator laser_point;
  Plane           plane;
  Planes          planes;
  Line3D          ridge, new_ridge1, new_ridge2;
  Line2D          ridge2D;
  int             segment_number, max_num, min_num, num, inc_num, max_iter, iter,
                  index, num_distances;
  bool            done, stop;
  double          dist, min_dist, max_dist, inc_dist, max_dist2, sqsumplanefit,
                  fitresidual, sumdxy2, sumdxy, sumdz2, sumdz, dxy, dz;
  vector<double>  stddevdxy, stddevdz, stddevplanefit;
  vector<double>::iterator itr;
  double          average[50];

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-l") || !args->Contains("-r") || !args->Contains("-o")) {
    printf("Error: -l, -r, and -o are required arguments.\n");
    PrintUsage();
    exit(0);
  }
  
  // Open input files
  if (!laser_points.Read(args->String("-l"), false)) {
    printf("Error reading laser points from %s\n", args->String("-l"));
    exit(0);
  }
  if (!ridge_points.Read(args->String("-r"))) {
    printf("Error reading ridge points from %s\n", args->String("-r"));
    exit(0);
  }
  
  // Open output file
  fdo = fopen(args->String("-o"), "w");
  if (fdo == NULL) {
    printf("Error opening file %s\n", args->String("-o"));
    exit(0);
  }

  // Select the segments
  segment_number = laser_points.begin()->Attribute(SegmentNumberTag);
  laser_point = laser_points.begin() + 1;
  while (laser_point->Attribute(SegmentNumberTag) == segment_number)
    laser_point++;
  segment1.insert(segment1.end(), laser_points.begin(), laser_point);
  laser_points.erase(laser_points.begin(), laser_point);

  segment_number = laser_points.begin()->Attribute(SegmentNumberTag);
  laser_point = laser_points.begin() + 1;
  while (laser_point->Attribute(SegmentNumberTag) == segment_number)
    laser_point++;
  segment2.insert(segment2.end(), laser_points.begin(), laser_point);
  laser_points.erase(laser_points.begin(), laser_point);

  segment_number = laser_points.begin()->Attribute(SegmentNumberTag);
  laser_point = laser_points.begin() + 1;
  while (laser_point->Attribute(SegmentNumberTag) == segment_number)
    laser_point++;
  segment3.insert(segment3.end(), laser_points.begin(), laser_point);
  laser_points.erase(laser_points.begin(), laser_point);

  segment4.swap(laser_points);
  fprintf(fdo, "Segments have %d, %d, %d and %d points\n", 
          segment1.size(), segment2.size(), segment3.size(), segment4.size());  
  segments.push_back(&segment1); subsets.push_back(&subset1);
  segments.push_back(&segment2); subsets.push_back(&subset2);
  segments.push_back(&segment3); subsets.push_back(&subset3);
  segments.push_back(&segment4); subsets.push_back(&subset4);

  // Calculate ridge midpoint
  midpoint.vect() = (ridge_points[0].vect() + ridge_points[1].vect()) / 2.0;
  ridge = Line3D(ridge_points[0].Position3DRef(), ridge_points[1].Position3DRef());
  
  // Determine maximum distance 
  max_dist = 100000.0;
  for (segment=segments.begin(); segment!=segments.end(); segment++) {
    max_dist2 = 0.0;
    for (laser_point=(*segment)->begin(); laser_point!=(*segment)->end(); laser_point++) {
      dist = ridge.DistanceToPoint(laser_point->Position3DRef());
      if (dist > max_dist2) max_dist2 = dist;
    }
    if (max_dist2 < max_dist) max_dist = max_dist2;
  }
  fprintf(fdo, "Maximum distance %.2f, rounded to ", max_dist);
  if (max_dist > args->Double("-maxd", 10.0)) 
    max_dist = args->Double("-maxd", 10.0);
  min_dist = dist = args->Double("-mind", 1.0);
  inc_dist = args->Double("-incd", 1.0);
  while (dist < max_dist+0.01) dist += inc_dist;
  max_dist = dist - inc_dist;
  fprintf(fdo, "%.2f\n", max_dist);
  
  // Determine maximum number of points
  max_num = segment1.size();
  if (segment2.size() < max_num) max_num = segment2.size();
  if (segment3.size() < max_num) max_num = segment3.size();
  if (segment4.size() < max_num) max_num = segment4.size();
  max_num /= 5;
  fprintf(fdo, "Maximum number %d, rounded to ", max_num);
  if (max_num > args->Integer("-maxn", 100))
    max_num = args->Integer("-maxn", 100);
  min_num = num = args->Integer("-minn", 10);
  inc_num = args->Integer("-incn", 10);
  while (num <= max_num) num += inc_num;
  max_num = num - inc_num;
  fprintf(fdo, "%d\n", max_num);

  for (segment=segments.begin(); segment!=segments.end(); segment++)
    (*segment)->SetAttribute(IsSelectedTag, 1);
  max_iter = args->Integer("-niter", 25);
  Randomize();
  stop = false;
  // Loop over all distances
  for (dist=max_dist; dist >= min_dist - 0.01 && !stop; dist -= inc_dist) {

    // Delete all points that are further away than "dist"
    for (segment=segments.begin(); segment!=segments.end(); segment++) {
      for (laser_point=(*segment)->begin(); laser_point!=(*segment)->end(); laser_point++)
        if (ridge.DistanceToPoint(laser_point->Position3DRef()) > dist)
          laser_point->Attribute(IsSelectedTag) = 0;
      (*segment)->RemoveTaggedPoints(0, IsSelectedTag);
      if ((*segment)->size() < max_num) {
        stop = true;
        printf("Only %d points within %.1f m, less than maximum %d\n",
               (*segment)->size(), dist, max_num);
      }
    }

    // Loop over all number of points
    for (num=min_num; num <= max_num && !stop; num += inc_num) {
    
      // Initialise counters
      sumdxy = sumdxy2 = sumdz = sumdz2 = 0.0;
      sqsumplanefit = 0.0;
      
      // Loop iter times
      for (iter=0; iter<max_iter; iter++) {
  
        // Processing of all faces
        planes.Erase();
        for (subset=subsets.begin(), segment=segments.begin();
             subset!=subsets.end(); subset++, segment++) {

          // Collect "num" points from both segments by random subsampling
          (*subset)->ErasePoints();
          while ((*subset)->size() < num) {
            index = (int) GenerateRandom(0.0, (double) (*segment)->size() - 0.0001);
            laser_point = (*segment)->begin() + index;
            if (!(*subset)->Contains(*laser_point)) {
              (*subset)->push_back(*laser_point);
            }
          }
          
          // Fit plane
          plane = (*subset)->FitPlane(1, 1, IsSelectedTag);
          if (plane.Normal().Z() < 0.0) plane.SwapNormal();
          planes.push_back(plane);

          // Calculate standard deviation of plane fitting
          for (laser_point=(*subset)->begin(); laser_point!=(*subset)->end(); laser_point++) {
            fitresidual = plane.Distance(laser_point->Position3DRef());
            sqsumplanefit += fitresidual * fitresidual;
          }
        }

        // Output of intersection angle
        if (dist == max_dist && num == max_num && iter == 0) {
          fprintf(fdo, "Angle between planes is %.1f degrees\n",
                  Angle(planes[0].Normal(), planes[1].Normal()) * 45.0 / atan(1.0)); 
          fprintf(fdo, "Roof slope is %.2f\n\n",
                  planes[0].Normal().vect2D().Length() / planes[0].Normal().Z());
        }
        
        // Intersect planes
        Intersect2Planes(planes[0], planes[1], new_ridge1);
        Intersect2Planes(planes[2], planes[3], new_ridge2);
  
        // Project original mid point on first ridge
        proj_midpoint1 = new_ridge1.Project(midpoint);
        
        // Determine signed distance in the XOY plane
        ridge2D = new_ridge2.ProjectOntoXOYPlane();
        dxy = ridge2D.DistanceToPointSigned(proj_midpoint1.Position2DOnly());
        
        // Project the mid point of the first ridge onto the second ridge
        proj_midpoint2 = new_ridge2.Project(proj_midpoint1);
        
        // Use these two projections to derive the height offset
        dz  = proj_midpoint1.Z() - proj_midpoint2.Z();
          
        // Update statistics of ridge line variation
        sumdxy  += dxy;
        sumdxy2 += dxy * dxy;
        sumdz   += dz;
        sumdz2  += dz * dz;
      }
      // Store results 
      stddevdxy.push_back(sqrt((sumdxy2 - sumdxy * sumdxy / max_iter) / max_iter));
      stddevdz.push_back(sqrt((sumdz2 - sumdz * sumdz / max_iter) / max_iter));
      stddevplanefit.push_back(sqrt(sqsumplanefit / (4 * num * max_iter)));
    }
  }
  
  // Output of statistics
  fprintf(fdo, "Standard deviations of planimetric offets\n");
  fprintf(fdo, "# points:");
  for (num=min_num; num <= max_num; num += inc_num)
    fprintf(fdo, "  %3d  ", num);
  fprintf(fdo, "\nDistance\n");
  num_distances = 0;
  for (dist=max_dist, itr=stddevdxy.begin(); dist >= min_dist - 0.01; dist -= inc_dist) {
    fprintf(fdo, "%6.1f  ", dist);
    num_distances++;
    for (num=min_num, index=0; num <= max_num; num += inc_num, itr++, index++) {
      fprintf(fdo, "%6.4f ", *itr);
      if (dist == max_dist) average[index] = *itr;
      else average[index] += *itr;
    }
    fprintf(fdo, "\n");
  }
  fprintf(fdo, "average ");
  for (num=min_num, index=0; num <= max_num; num += inc_num, itr++, index++)
    fprintf(fdo, "%6.4f ", average[index]/num_distances);
  fprintf(fdo, "\n");

  fprintf(fdo, "\nStandard deviations of height offets\n");
  fprintf(fdo, "# points:");
  for (num=min_num; num <= max_num; num += inc_num)
    fprintf(fdo, "  %3d  ", num);
  fprintf(fdo, "\nDistance\n");
  for (dist=max_dist, itr=stddevdz.begin(); dist >= min_dist - 0.01; dist -= inc_dist) {
    fprintf(fdo, "%6.1f  ", dist);
    for (num=min_num, index=0; num <= max_num; num += inc_num, itr++, index++) {
      fprintf(fdo, "%6.4f ", *itr);
      if (dist == max_dist) average[index] = *itr;
      else average[index] += *itr;
    }
    fprintf(fdo, "\n");
  }
  fprintf(fdo, "average ");
  for (num=min_num, index=0; num <= max_num; num += inc_num, itr++, index++)
    fprintf(fdo, "%6.4f ", average[index]/num_distances);
  fprintf(fdo, "\n");

  fprintf(fdo, "\nAverage standard deviations of plane fitting\n");
  fprintf(fdo, "# points:");
  for (num=min_num; num <= max_num; num += inc_num)
    fprintf(fdo, "  %3d  ", num);
  fprintf(fdo, "\nDistance\n");
  for (dist=max_dist, itr=stddevplanefit.begin(); dist >= min_dist - 0.01; dist -= inc_dist) {
    fprintf(fdo, "%6.1f  ", dist);
    for (num=min_num, index=0; num <= max_num; num += inc_num, itr++, index++) {
      fprintf(fdo, "%6.4f ", *itr);
      if (dist == max_dist) average[index] = *itr;
      else average[index] += *itr;
    }
    fprintf(fdo, "\n");
  }
  fprintf(fdo, "average ");
  for (num=min_num, index=0; num <= max_num; num += inc_num, itr++, index++)
    fprintf(fdo, "%6.4f ", average[index]/num_distances);
  fprintf(fdo, "\n");
}
