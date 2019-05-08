/*
--------------------------------------------------------------------------------

 Initial creation:
 Author : George Vosselman
 Date   : 09-07-2008

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "BNF_io.h"
#include "LaserPoints.h"
#include "LineSegments2D.h"
#include "LineSegment3D.h"

/*
--------------------------------------------------------------------------------
                      The main roofcheck function
--------------------------------------------------------------------------------
*/

void VerifyRoads(const char *ref_pts_name, const char *road_pts_name,
                const char *road_top_name, const char *output_name)
{              
  LaserPoints              ref_points;
  ObjectPoints             road_points;
  LineTopologies           road_tops;
  LineTopologies::iterator road_top;
  LaserPoints::iterator    ref_point, nearest_point;
  ObjectPoints::iterator   road_point;
  int                      num_ref_points, ref_point_index, num_pairs=0, num;
  TINEdges                 *edges;
  TINEdges::iterator       edgeset;
  SegmentationParameters   par;
  FILE                     *fd;
  double                   dist;
  
  // Read data  
  ref_points.Read(ref_pts_name, false);
  road_points.Read(road_pts_name);
  road_tops.Read(road_top_name, false);
  
  // Make sure all points have the same height
  for (road_point=road_points.begin(); road_point!=road_points.end(); road_point++)
    road_point->Z() = 0.0;
  for (ref_point=ref_points.begin(); ref_point!=ref_points.end(); ref_point++)
    ref_point->Z() = 0.0;
  
  // Densify all road polygons to 1 cm
  printf("%d polygons\n", road_tops.size());
  for (road_top=road_tops.begin(), num=0; road_top!=road_tops.end(); road_top++, num++) {
    printf("pol %d\n", num);
    road_top->Densify(road_points, 0.01);
  }
  
//  road_tops.Densify(road_points, 0.01);
  
  // Add road points to reference points list
  num_ref_points = ref_points.size();
  for (road_point=road_points.begin(); road_point!=road_points.end(); road_point++)
    ref_points.push_back(LaserPoint(road_point->vect()));
    
  // Derive 2 NN list
  par.NumberOfNeighbours() = 2;
  par.DistanceMetricDimension() = 2;
  printf("before derive edges in %d points\n", ref_points.size());
  edges = ref_points.DeriveEdges(par);
  
  fd = fopen(output_name, "w");
  if (fd == NULL) {
    printf("Error opening output file %s\n", output_name);
    exit(0);
  }
  fprintf(fd, "no set1 x1 y1 z1 set2 x2 y2 z2\n");
  printf("output loop\n");
  // Output pairs of reference point and road point if the distance is below 0.6 m
  for (ref_point=ref_points.begin(), edgeset=edges->begin(), ref_point_index=0;
       ref_point_index < num_ref_points;
       ref_point++, edgeset++, ref_point_index++) {
    // Determine nearest point
    printf("%d %d %d\n", ref_point_index, edgeset->begin()->Number(),
           (edgeset->begin()+1)->Number());
    if ((edgeset->begin()+1)->Number() > num_ref_points) {
      nearest_point = ref_points.begin() + (edgeset->begin()+1)->Number();
      dist = ref_point->Distance(nearest_point->Position3DRef());
      if (dist < 0.6) {
        num_pairs++;
        fprintf(fd, "%3d 1 %10.2f %10.2f 0.0 2 %10.2f %10.2f 0.0\n",
                num_pairs, ref_point->X(), ref_point->Y(),
                nearest_point->X(), nearest_point->Y());
      }
    }
  }
  fclose(fd);
}
