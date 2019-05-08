/*
--------------------------------------------------------------------------------
 Extraction of rail road track segments from a high density point cloud 
 These segments are used as seeds for tracking rail roads over a larger
 dataset

 Initial creation:
 Author : George Vosselman
 Date   : 26-11-2008

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include "LaserPoints.h"

/*
--------------------------------------------------------------------------------
                         The main tracks function
--------------------------------------------------------------------------------
*/
void SelectRailPoints(LaserPoints &points, int knn,
                      double rail_height, double rail_height_margin)
{
  SegmentationParameters   par;
  TINEdges                 *edges;
  TINEdges::iterator       nbh;
  double                   minimum_height;
  TINEdgeSet::iterator     nb_node;
  LaserPoints::iterator    point, nb_point;
  
  // Derive edges from kd-tree
  par.NumberOfNeighbours() = (points.size() < knn) ? points.size() : knn;
  par.DistanceMetricDimension() = 2;
  edges = points.DeriveEdges(par);

  // Reset labels
  points.Label(0);
    
  // Loop over all points
  for (point=points.begin(), nbh=edges->begin(); point!=points.end(); point++, nbh++) {
    // Determine lowest point in neighbourhood
    minimum_height = point->Z();
    for (nb_node=nbh->begin(); nb_node!=nbh->end(); nb_node++) {
      nb_point = points.begin() + nb_node->Number();
      if (nb_point->Z() < minimum_height) minimum_height = nb_point->Z();
    }
    // Select points at approximate rail height
    if (point->Z() > minimum_height + rail_height - rail_height_margin &&
        point->Z() < minimum_height + rail_height + rail_height_margin)
      point->Label(1);
  }
  
  // Remove non track points
  points.RemoveTaggedPoints(0, LabelTag);
  
  // Delete edges
  edges->Erase();
}

void SeedTracks(LaserPoints &points, int knn, LineSegments3D &seed_segments,
                double rail_height, double rail_height_margin,
                double max_dist_to_rail, int min_num_hits, int max_num_tries,
                double max_rail_gap, double track_width,
                double track_width_margin)
{
  LaserPoints::iterator    point;
  double                   dist, angle;
  ObjectPoints             seed_objpts;
  LineTopologies           seed_tops;
  LineSegments3D           segments, all_segments;
  LineSegments3D::iterator segment, segment2;
  int                      num_hits;
  Position3D               projection_point, mid_point1, mid_point2;
  
  // Select potential rail points
  SelectRailPoints(points, knn, rail_height, rail_height_margin);
    
  // Extract lines of rails
  do {
    // Get a line
    points.RANSAC3DLine(max_dist_to_rail, min_num_hits,
                        max_num_tries, num_hits,
                        max_rail_gap, segments,
                        LabelTag, 1);
    if (segments.size()) {
      // Mark points near segment with label 2
      segment = segments.begin();
      for (point=points.begin(); point!=points.end(); point++)
        if (segment->Distance(point->Position3DRef()) < 0.2) point->Label(2);
      // Save largest (first) segment
      all_segments.push_back(*segment);
    }
  } while (segments.size());

  // Derive seed segments as centre lines from parallel lines that around
  // track_width apart
  for (segment=all_segments.begin(); segment!=all_segments.end()-1; segment++)
    for (segment2=segment+1; segment2!=all_segments.end(); segment2++) {
      dist = segment->Line3D::DistanceToPoint(segment2->MiddlePoint());
      angle = Angle2Lines(segment->Line3DReference(), segment2->Line3DReference());
      if (angle < atan(1.0) / 45 && dist > track_width - track_width_margin &&
          dist < track_width + track_width_margin) {
        // Construct the track mid points
        projection_point = segment->Project(segment2->BeginPoint());
        mid_point1.vect() = (projection_point.vect() + segment2->BeginPoint().vect()) / 2.0;
        projection_point = segment->Project(segment2->EndPoint());
        mid_point2.vect() = (projection_point.vect() + segment2->EndPoint().vect()) / 2.0;
        // Save the track centre line
        seed_segments.push_back(LineSegment3D(mid_point1, mid_point2,
                                segment->Number() + segment2->Number(), 0));
      }
    }

  // Write seed segments
  seed_segments.PointsWithTopology(seed_objpts, seed_tops, false);
  seed_objpts.Write("seed_tracks.objpts");
  seed_tops.Write("seed_tracks.top");

  // Remove unused points and output the remaining used points
  points.RemoveTaggedPoints(1, LabelTag);
  points.Write("seed_track_points_used.laser", false, false);
}
