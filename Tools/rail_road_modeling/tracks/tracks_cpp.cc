/*
--------------------------------------------------------------------------------
 Extraction of rail road tracks from a high density point cloud 

 Initial creation:
 Author : George Vosselman
 Date   : 10-10-2008

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include "LaserPyramid.h"

/*
--------------------------------------------------------------------------------
                         The main tracks function
--------------------------------------------------------------------------------
*/

void tracks_cpp(char *seed_file, char *meta_data_file,
                const char *track_point_file,
                const char *track_top_file, char *out_file, int knn)
{
  // Variables
  LaserPoints              points, tile_points,
                           left_rail_points, right_rail_points,
                           potential_rail_points, all_potential_rail_points,
                           strip_points, used_rail_points, current_points,
                           predicted_points, updated_points, line_points;
  LaserPoints::iterator    point;
  LineSegments3D           seed_segments;
  LineSegments3D::iterator seed_segment;
  bool                     reversed, dead_end, verbose=true, debug=false,
                           use_pyramid=false;
  LaserPyramid             pyramid;
  LaserBlock               block;
  Position2D               previous_pos2d, current_pos2d, predicted_pos2d,
                           left_pos2d, right_pos2d;
  Line2D                   track_line, left_rail, right_rail, perp_line;
  Line3D                   left_rail3d, right_rail3d;
  double                   current_scalar, predicted_scalar,
                           track_height, scalar, scalar_max, dist, dist_rails;
  int                      i, num_old_left_rail_points, num_old_right_rail_points,
                           track_point_number=0, scan_number;
  ObjectPoints             track_points;
  ObjectPoint              track_point;
  LineTopologies           tracks;
  LineTopology             track;
  DataBounds2D             bounds;

  // debug
  double scalar_diff;
  
  // Functions
  void SeedTracks(LaserPoints &, int, LineSegments3D &, double, double, double,
                  int, int, double, double, double);
  void SelectRailPoints(LaserPoints &, int, double, double);

  // Parameters
  double rail_height        = 0.20;
  double rail_height_margin = 0.10;
  double max_dist_to_rail   = 0.20;
  int    min_num_hits       = 100;
  int    max_num_tries      = 1000;
  double max_rail_gap       = 5.0;
  double track_width        = 1.50;
  double track_width_margin = 0.20;
  
  double prediction_length       = 3.0; // 5
  double prediction_width        = 2.0;  // 3
  int    min_num_new_rail_points = 3;   // 5

  // Read the laser data for seed detection
  if (!points.Read(seed_file, false)) {
    printf("Error reading laser data from file %s\n", seed_file);
    exit(0);
  }

  // Extract the seed tracks
  SeedTracks(points, knn, seed_segments, rail_height, rail_height_margin,
             max_dist_to_rail, min_num_hits, max_num_tries, max_rail_gap,
             track_width, track_width_margin);
             
  if (seed_segments.empty()) {
    printf("No seed segments extracted from %s\n", seed_file);
    exit(0);
  }
  
  // Read the pyramid meta data
  use_pyramid = (strstr(meta_data_file, "pyramid") != 0);
  if (use_pyramid) {
    if (!pyramid.ReadMetaData(meta_data_file)) {
      printf("Error reading pyramid meta data from file %s\n", meta_data_file);
      exit(0);
    }
  }  
  else {
    if (!block.ReadMetaData(meta_data_file)) {
      printf("Error reading block meta data from file %s\n", meta_data_file);
      exit(0);
    }
  }

  // Process seeds until stack is empty
  reversed = true;
  while (!seed_segments.empty() || !reversed) {
    // Reverse the seed segment or get a new one
    if (!reversed) {
      seed_segment->ReverseDirection();
      reversed = true;
    }
    else {
      seed_segment = seed_segments.begin();
      reversed = false;
    }
  
    if (verbose) printf("\n\n\n\nSeed from (%.2f, %.2f) to (%.2f, %.2f), %d points\n",
           seed_segment->BeginPoint().X(), seed_segment->BeginPoint().Y(),
           seed_segment->EndPoint().X(), seed_segment->EndPoint().Y(),
           seed_segment->Number());
    //
    // Initialise track from seed segments
    //
    track_line = Line2D(seed_segment->Line3DReference());
    // Current track position is the end point of the seed segment
    current_pos2d = Position2D(seed_segment->EndPoint().X(),
                               seed_segment->EndPoint().Y());
    current_scalar = track_line.Scalar(current_pos2d);
    // Previous track position is the middle point of the seed segment
    previous_pos2d = Position2D(seed_segment->MiddlePoint().X(),
                                seed_segment->MiddlePoint().Y());
    // Use mid point on rails to simulate laser points
    perp_line = track_line.PerpendicularLine(previous_pos2d);
    perp_line.PointsAtDistance(previous_pos2d, track_width/2.0,
                               left_pos2d, right_pos2d);
    track_height = seed_segment->MiddlePoint().Z();
    left_rail_points.ErasePoints();
    right_rail_points.ErasePoints();
    for (i=0; i<seed_segment->Number()/2; i++) {
      left_rail_points.push_back(LaserPoint(left_pos2d.X(), left_pos2d.Y(),
                                            track_height));
      right_rail_points.push_back(LaserPoint(right_pos2d.X(), right_pos2d.Y(),
                                             track_height));
    }
    left_rail_points.Label(1);
    right_rail_points.Label(1);
    // Store the seed segment centre as first point of this track
    track.Erase();
    track_point.vect() = seed_segment->MiddlePoint().vect();
    track_point.Number() = track_point_number;
    track_points.push_back(track_point);
    track.push_back(PointNumber(track_point_number));
    track_point_number++;
    //
    // Follow the track
    //
    dead_end = false;
    while (!dead_end) {
      // Predicted track position is "prediction_length" furtheron on the segment line
      predicted_scalar = current_scalar + prediction_length;
      predicted_pos2d = track_line.Position(predicted_scalar);
      current_points.push_back(LaserPoint(current_pos2d.X(), current_pos2d.Y(), 0.0)); // key points
      predicted_points.push_back(LaserPoint(predicted_pos2d.X(), predicted_pos2d.Y(), 0.0)); // key points
      
      // Get data in front of the current track
      points.ErasePoints();
      if (debug)
        printf("Selecting new points between scalars %.2f and %.2f\n",
               current_scalar, predicted_scalar);
      if (use_pyramid) {
        bounds = track_line.BoundingBox(current_scalar, predicted_scalar, prediction_width/2.0);
        pyramid.UpdateSelection(bounds, 0.01, tile_points, 1000000);
        tile_points.Select(points, track_line, prediction_width/2.0, current_scalar, predicted_scalar);
      }
      else
        block.SelectPoints(points, track_line, prediction_width/2.0,
                           current_scalar, predicted_scalar);
      if (debug) printf("%d points selected\n", points.size());
      
      // Select potential rail points strip by strip
      potential_rail_points.ErasePoints();
      while (!points.empty()) {
        strip_points.ErasePoints();
        scan_number = points.begin()->ScanNumber() / 10;
        strip_points.AddTaggedPoints(points, scan_number*10 + 1, ScanNumberTag);
        strip_points.AddTaggedPoints(points, scan_number*10 + 2, ScanNumberTag);
        strip_points.AddTaggedPoints(points, scan_number*10 + 3, ScanNumberTag);
        SelectRailPoints(strip_points, knn, rail_height, rail_height_margin);
        potential_rail_points.insert(potential_rail_points.end(),
                                     strip_points.begin(), strip_points.end());
        points.RemoveTaggedPoints(scan_number*10 + 1, ScanNumberTag);
        points.RemoveTaggedPoints(scan_number*10 + 2, ScanNumberTag);
        points.RemoveTaggedPoints(scan_number*10 + 3, ScanNumberTag);
      }
      if (debug) printf("Possibly %d points on rails\n", points.size());
      all_potential_rail_points.insert(all_potential_rail_points.end(),
                                       potential_rail_points.begin(),
                                       potential_rail_points.end());

      // Approximate left and right rail
      left_rail = Line2D(track_line.Normal().Y(), track_line.Normal().X(),
                         track_line.DistanceToOrigin() - track_width/2.0);
      right_rail = Line2D(track_line.Normal().Y(), track_line.Normal().X(),
                          track_line.DistanceToOrigin() + track_width/2.0);
                          
      // Select points on left and right rail
      num_old_left_rail_points  = left_rail_points.size();
      num_old_right_rail_points = right_rail_points.size();
      for (point=potential_rail_points.begin();
           point!=potential_rail_points.end(); point++) {
        if (left_rail.DistanceToPoint(point->Position2DOnly()) <
            max_dist_to_rail) {
          point->Label(2);
          left_rail_points.push_back(*point);
        }
        else if (right_rail.DistanceToPoint(point->Position2DOnly()) <
                 max_dist_to_rail) {
          point->Label(2);
          right_rail_points.push_back(*point);
        }
      }      
      
      // Check if new rail points have been found
      if (left_rail_points.size() - num_old_left_rail_points <
          min_num_new_rail_points ||
          right_rail_points.size() - num_old_right_rail_points <
          min_num_new_rail_points)
        dead_end = true;
      if (debug) printf("Left %d points (was %d), right %d (was %d)\n",
             left_rail_points.size() - num_old_left_rail_points,
             num_old_left_rail_points,
             right_rail_points.size() - num_old_right_rail_points,
             num_old_right_rail_points);
             
      // Look for switches (not yet implemented)
      
      // Re-estimate rails and track lines
      left_rail3d.Erase();
      for (point=left_rail_points.begin(); point!=left_rail_points.end(); point++)
        left_rail3d.AddPoint(point->Position3DRef(), false);
      left_rail3d.Recalculate();
      right_rail3d.Erase();
      for (point=right_rail_points.begin(); point!=right_rail_points.end(); point++)
        right_rail3d.AddPoint(point->Position3DRef(), false);
      right_rail3d.Recalculate();
      left_rail = Line2D(left_rail3d);
      right_rail = Line2D(right_rail3d);
      // Make sure we keep the same direction
      if (left_rail.Scalar(predicted_pos2d) < left_rail.Scalar(current_pos2d))
        left_rail.SwapNormal();
      if (right_rail.Scalar(predicted_pos2d) < right_rail.Scalar(current_pos2d))
        right_rail.SwapNormal();

      // Use weighted average of directions of rails for track direction
      track_line = Line2D((left_rail.Getsinphi() * left_rail_points.size() +
                           right_rail.Getsinphi() * right_rail_points.size()) /
                          (left_rail_points.size() + right_rail_points.size()),
                          (left_rail.Getcosphi() * left_rail_points.size() +
                           right_rail.Getcosphi() * right_rail_points.size()) /
                          (left_rail_points.size() + right_rail_points.size()),
                          0.0);
      // For distance take the average between the two rails
      dist_rails = fabs(track_line.DistanceToPointSigned(left_rail3d.CentreOfGravity().Position2DOnly()) -
                    track_line.DistanceToPointSigned(right_rail3d.CentreOfGravity().Position2DOnly()));
      track_line.SetDistanceToOrigin(track_line.DistanceToPointSigned(
                                         (left_rail3d.CentreOfGravity().Position2DOnly() +
                                          right_rail3d.CentreOfGravity().Position2DOnly()) / 2.0));
      // Save some key points
      line_points.push_back(LaserPoint(left_rail3d.CentreOfGravity().X(),
                                       left_rail3d.CentreOfGravity().Y(),
                                       left_rail3d.CentreOfGravity().Z()));
      line_points.push_back(LaserPoint(left_rail3d.CentreOfGravity().X() + 5.0 * left_rail3d.Direction().X(),
                                       left_rail3d.CentreOfGravity().Y() + 5.0 * left_rail3d.Direction().Y(),
                                       left_rail3d.CentreOfGravity().Z() + 5.0 * left_rail3d.Direction().Z()));
      line_points.push_back(LaserPoint(right_rail3d.CentreOfGravity().X(),
                                       right_rail3d.CentreOfGravity().Y(),
                                       right_rail3d.CentreOfGravity().Z()));
      line_points.push_back(LaserPoint(right_rail3d.CentreOfGravity().X() + 5.0 * right_rail3d.Direction().X(),
                                       right_rail3d.CentreOfGravity().Y() + 5.0 * right_rail3d.Direction().Y(),
                                       right_rail3d.CentreOfGravity().Z() + 5.0 * right_rail3d.Direction().Z()));
      
      // Make sure we keep the same direction
      if (track_line.Scalar(predicted_pos2d) < track_line.Scalar(current_pos2d))
        track_line.SwapNormal();

      if (debug) { 
        printf("Recalc "); track_line.Print();
        printf("Left   "); left_rail.Print();
        printf("Right  "); right_rail.Print();
      }
      // For continuing tracks, use updated current position as next node
      current_pos2d = track_line.Project(current_pos2d); // Update current position
      updated_points.push_back(LaserPoint(current_pos2d.X(), current_pos2d.Y(), 0.0)); // key point
      if (!dead_end) {
        track_point.X() = current_pos2d.X();
        track_point.Y() = current_pos2d.Y();
        track_point.Z() = (left_rail3d.CentreOfGravity().Z() +
                           right_rail3d.CentreOfGravity().Z()) / 2.0;
      }
      
      // Find last point location on the rail
      scalar_max = track_line.Scalar(current_pos2d);
      for (point=left_rail_points.begin(); point!=left_rail_points.end(); point++) {
        scalar = track_line.Scalar(point->Position2DOnly());
        if (scalar > scalar_max) scalar_max = scalar;
      }
      for (point=right_rail_points.begin(); point!=right_rail_points.end(); point++) {
        scalar = track_line.Scalar(point->Position2DOnly());
        if (scalar > scalar_max) scalar_max = scalar;
      }
      // Use the last point to update the current position
      current_scalar = scalar_max;
      current_pos2d  = track_line.Position(current_scalar);
             
      // In case of a dead end, use this last point to terminate track
      if (dead_end) {
        track_point.X() = current_pos2d.X();
        track_point.Y() = current_pos2d.Y();
        track_point.Z() = (left_rail3d.CentreOfGravity().Z() +
                           right_rail3d.CentreOfGravity().Z()) / 2.0;

        // debug
        if (verbose)
          printf("Point %d (%.2f, %.2f, %.2f)\n",
                 track_point_number,
                 track_point.X(), track_point.Y(), track_point.Z());
        track_point.Number() = track_point_number;
        track_points.push_back(track_point);
        track.push_back(PointNumber(track_point_number));
        track_point_number++;
        track_point.X() = current_pos2d.X() + track_line.Direction().X() * 10.0;
        track_point.Y() = current_pos2d.Y() + track_line.Direction().Y() * 10.0;


      }
      if (verbose)
        printf("Point %d (%.2f, %.2f, %.2f)\n",
               track_point_number,
               track_point.X(), track_point.Y(), track_point.Z());
               
      // Add next node to the track
      track_point.Number() = track_point_number;
      track_points.push_back(track_point);
      track.push_back(PointNumber(track_point_number));
      track_point_number++;
      
      // Delete points of previous prediction
      left_rail_points.RemoveLabeledPoints(1);
      left_rail_points.Label(1);
      right_rail_points.RemoveLabeledPoints(1);
      right_rail_points.Label(1);
      used_rail_points.insert(used_rail_points.begin(), left_rail_points.begin(),
                              left_rail_points.end());
      used_rail_points.insert(used_rail_points.begin(), right_rail_points.begin(),
                              right_rail_points.end());
      if (debug)
        printf("After cleaning %d left and %d right rail points left.\n",
               left_rail_points.size(), right_rail_points.size());
    }
    // Store track
    tracks.push_back(track);
      
    // Remove seed from stack if both directions have been processed
    if (reversed) seed_segments.erase(seed_segment);
    
    // debug
//    if (tracks.size() == 4) seed_segments.erase(seed_segments.begin(), seed_segments.end());
  }
  
  // Write tracks
  track_points.Write(track_point_file);
  tracks.Write(track_top_file);
  all_potential_rail_points.Write("potential_track_points.laser", false, false);
  used_rail_points.Write("used_track_points.laser", false, false);
  current_points.Label(0); // Red
  predicted_points.Label(1); // Green
  updated_points.Label(2); // Blue
  line_points.Label(3);
  current_points.insert(current_points.end(), predicted_points.begin(), predicted_points.end());
  current_points.insert(current_points.end(), updated_points.begin(), updated_points.end());
  current_points.insert(current_points.end(), line_points.begin(), line_points.end());
  current_points.Write("key_points.laser", false, false);
}
