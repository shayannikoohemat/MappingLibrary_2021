/*
                     Copyright 2010 University of Twente 
 
       This file is part of the Mapping libraries and tools, developed
  for research, education and projects in photogrammetry and laser scanning.

  The Mapping libraries and tools are free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the License,
                   or (at your option) any later version.

 The Mapping libraries and tools are distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
          along with the Mapping libraries and tools.  If not, see
                      <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------*/


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

void RoofVerify(int first_roof_number_in, const char *stat_verified_file_name,
                const char *result_file_name,
                const char *output_directory_verified, 
                const char *output_directory_failed,
                double distance_to_point_in_other_strip,
                int number_of_neighbours, double min_size_bbox,
                double max_dist_point_to_ridge,
                double max_dist_point_to_face,
                double min_ridge_length,
                double max_dist_ridge_to_ridge,
                double max_angle_between_planes,
                int first_roof_number_out_verified,
                int first_roof_number_out_failed,
                bool afn_in_strip_number,
                bool remove_double_points,
                int fixed_minimum_number_of_points,
                double max_ridge_slope_angle,
                double min_intersection_angle_planes,
                double min_ridge_angle,
                int max_num_roofs,
                double max_size_bbox,
                const char *segm_par_file,
                bool five_digit_numbers
                )
{
  char                              *file_name, roof_name[11], *ch, *laser_name,
                                    *points_name, *top_name, roof_name_v[12],
                                    roof_name_f[12];
  LaserPoints                       laser_points, strip1, strip2,
                                    plane_point_set1a, plane_point_set1b, 
                                    plane_point_set2a, plane_point_set2b,
                                    roof_side_a, roof_side_b;                                    
  LaserPoints::iterator             laser_point, nb_point;
  int                               strip_number1, strip_number2, roof_number_in,
                                    roof_number_verified, roof_number_failed, 
                                    num_pts, 
                                    num_plane_size_fit_failed=0,
                                    num_angle_failed=0, 
                                    num_intersection_angle_failed=0,
                                    num_plane_size_nearby_failed=0,
                                    num_plane_size_concom_failed=0,
                                    num_bbox_failed=0,
                                    num_intersection_failed=0,
                                    num_ridgelength_failed=0, 
                                    num_ridge_dist_failed=0,
                                    num_ridgeslope_failed=0,  
                                    num_ridge_angle_failed=0,
                                    segment_number, new_segment_number,
                                    new_label, num_roof, num_removed,
                                    segment_number1a, segment_number2a, i, count,
                                    component_number, min_num_pts_plane,
                                    min_num_pts_plane_knowing_angle;
  double                            size_bbox, angle, angle_min, angle2;
  bool                              done, failed, cropped_points;
  TINEdges                          *edges=NULL;
  SegmentationParameters            parameters, segmentation_parameters;
  PointNumber                       node;
  PointNumberList                   neighbourhood, roof_face1, roof_face2;
  PointNumberList::iterator         nb_node;
  vector<LaserPoints *>             strips, plane_point_sets, roof_sides;
  vector<LaserPoints*>::iterator    strip, plane_point_set, roof_side;
  ObjectPoints                      box_points, ridge_points;
  ObjectPoint                       ridge_point;
  LineTopology                      box_top, matched_line;
  LineTopologies                    matched_lines, box_tops;
  LineSegments2D                    box_segments;
  LineSegments3D                    ridges;
  LineSegments3D::iterator          ridge, ridge1, ridge2;
  FILE                              *statistics_fd, *pcm_fd, *result_fd=NULL;
  Position3D                        pos1, pos2;
  Plane                             plane;
  Planes                            planes;

  void CropToOverlappingRegions(vector<LaserPoints *> &, vector<LaserPoints *> &,
                              SegmentationParameters &, int, bool, int, double);
  bool FitAllPlanes(Planes &, vector<LaserPoints *> &, bool &, FILE *,
                    double, int, int &, int &, int);
  int MinimumNumberOfPoints(double);
  double StandardDeviation(double, int);
    
  // Transfer some values
  roof_number_in = first_roof_number_in;
  roof_number_verified = first_roof_number_out_verified;
  roof_number_failed = first_roof_number_out_failed;

  // Pointer organisation
  strips.push_back(&strip1);
  strips.push_back(&strip2);
  roof_sides.push_back(&roof_side_a);
  roof_sides.push_back(&roof_side_b);
  plane_point_sets.push_back(&plane_point_set1a);
  plane_point_sets.push_back(&plane_point_set1b);
  plane_point_sets.push_back(&plane_point_set2a);
  plane_point_sets.push_back(&plane_point_set2b);

  // Parameters for connected component analysis
  parameters.NumberOfNeighbours() = number_of_neighbours;
  parameters.MaxDistanceInComponent() = distance_to_point_in_other_strip;
  if (fixed_minimum_number_of_points)
    parameters.MinNumberOfPointsComponent() = min_num_pts_plane = 
      fixed_minimum_number_of_points;
  else 
    parameters.MinNumberOfPointsComponent() = min_num_pts_plane = 
      MinimumNumberOfPoints(2.0 * atan(1.0)); // Initially, take value for 90 degree angles
  parameters.ComponentAttribute() = ComponentNumberTag;
  
  // Parameters for re-segmentation
  if (segm_par_file) {
    if (!segmentation_parameters.Read(segm_par_file)) {
      printf("Error reading segmentation parameters from file %s\n", segm_par_file);
      exit(0);
    }
  }
  else printf("Using default segmentation parameters\n");

  // Statistics files
  statistics_fd = fopen(stat_verified_file_name, "w");
  if (statistics_fd == NULL) {
    printf("Error opening statistics file %s\n", stat_verified_file_name);
    exit(0);
  }
  else
    fprintf(statistics_fd, "  bld strip1      X1         Y1      Z1     strip2      X2         Y2      Z2     dXY    dZ    stdev  manual\n");

  if (result_file_name) {
    result_fd = fopen(result_file_name, "w");
    if (result_fd == NULL) {
      printf("Error opening file for result messages %s\n", result_file_name);
      exit(0);
    }
    fprintf(result_fd, "# in   # out  result\n");
  }
  
  // Process all meta data files produced by roofcheck
  if (five_digit_numbers)
    sprintf(roof_name, "roof%5d", roof_number_in);
  else
    sprintf(roof_name, "roof%6d", roof_number_in);
  for (ch=roof_name; *ch!=0; ch++) if (*ch == ' ') *ch = '0';
  laser_name = (char *) malloc(strlen(roof_name) + 7);
  sprintf(laser_name, "%s%s", roof_name, ".laser");
  printf("#roof  #fail (  plane size    |   angle    |     |intersect  |     ridge      )\n");
  printf("             (fit near concom |strip gable |bbox |near  leng |dist slope angle)\n");
  printf("%6d%6d%6d%6d%6d%6d%6d%7d%6d%6d%6d%6d%6d\r", 
         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
  num_roof = 0;
  while (BNF_FileExists(laser_name) && (max_num_roofs == 0 || num_roof < max_num_roofs)) {
    num_roof++;
    laser_points.Read(laser_name, false);
    free(laser_name);
    
    // Remove double points, maybe no longer needed for other data sets
    if (remove_double_points) laser_points.RemoveDoublePoints(true);
    
    // Determine the strip numbers and segment numbers of one of the planes per strip
    strip_number1 = laser_points.begin()->ScanNumber(afn_in_strip_number);
    segment_number1a = laser_points.begin()->Attribute(SegmentNumberTag);
    for (laser_point=laser_points.begin()+1, done=false;
         !done && laser_point!=laser_points.end(); laser_point++) {
      strip_number2 = laser_point->ScanNumber(afn_in_strip_number);
      if (strip_number1 != strip_number2) {
        done = true;
        segment_number2a = laser_point->Attribute(SegmentNumberTag);
      }
    }
    if (!done) {
      printf("Error determining strip numbers for roof %d\n", roof_number_in);
      printf("Processed %d points with strip number %d\n", laser_points.size(),
             strip_number1);
      exit(0);
    }

    // Split points over two planes in two strips
    plane_point_set1a.ErasePoints(); plane_point_set1b.ErasePoints();
    plane_point_set2a.ErasePoints(); plane_point_set2b.ErasePoints();
    for (laser_point=laser_points.begin(); laser_point!=laser_points.end(); laser_point++) {
      if (laser_point->ScanNumber(afn_in_strip_number) == strip_number1)
        if (laser_point->Attribute(SegmentNumberTag) == segment_number1a)
          plane_point_set1a.push_back(*laser_point);
        else
          plane_point_set1b.push_back(*laser_point);
      else
        if (laser_point->Attribute(SegmentNumberTag) == segment_number2a)
          plane_point_set2a.push_back(*laser_point);
        else
          plane_point_set2b.push_back(*laser_point);
    }          
    failed = false;
    
    // Re-segment and only keep the largest segment per plane
    if (!failed) {        
      for (plane_point_set=plane_point_sets.begin(), i=0; 
           plane_point_set!=plane_point_sets.end() && !failed;
           plane_point_set++, i++) {
        (*plane_point_set)->SurfaceGrowing(segmentation_parameters, true, false);
        // Determine number of most frequent segment label
        component_number =
          (*plane_point_set)->MostFrequentAttributeValue(SegmentNumberTag, count);
        if (count < parameters.MinNumberOfPointsComponent()) {
          if (result_fd)
            fprintf(result_fd, "%5d %5d Failed: Only %d points left in largest segment\n",
                    roof_number_in, roof_number_failed, count); 
          failed = true;
          roof_number_failed++;
          num_plane_size_concom_failed++;
        }
        // Remove points part of the smaller components
        else if (count < (*plane_point_set)->size()) {
          (*plane_point_set)->CropTaggedPoints(component_number, SegmentNumberTag);
          cropped_points = true;
        }
        // Reset the segment numbers, to make sure they will be unique when merged
        // over the different planes
        (*plane_point_set)->SetAttribute(SegmentNumberTag, i+2);
      }
    }
    
    // Fit planes and check that points are close to the plane
    // Only continue if enough points on the plane are left
    if (!failed) {
      failed = FitAllPlanes(planes, plane_point_sets, cropped_points, result_fd,
                            max_dist_point_to_face, min_num_pts_plane,
                            roof_number_failed, num_plane_size_fit_failed,
                            roof_number_in);
    }

    // Find corresponding planes
    if (!failed) {
      angle = Angle(planes[0].Normal(), planes[2].Normal());
      angle2 = Angle(planes[0].Normal(), planes[3].Normal());
      // Swap the planes of the second strip if the angle between planes 0 and 2
      // is larger than the angle between planes 0 and 3.
      if (angle > angle2) {
        plane_point_set2a.swap(plane_point_set2b);
        plane = planes[2]; planes[2] = planes[3]; planes[3] = plane;
      }
    }
    
    // Eliminate points without nearby points of the other strip
    if (!failed) {
      CropToOverlappingRegions(roof_sides, plane_point_sets, parameters,
                               number_of_neighbours, afn_in_strip_number,
                               strip_number2, distance_to_point_in_other_strip);

      // Check if enough points are left
      for (plane_point_set=plane_point_sets.begin(), i=0; 
           plane_point_set!=plane_point_sets.end() && !failed;
           plane_point_set++, i++) {
        if ((*plane_point_set)->size() < min_num_pts_plane) {
          if (result_fd)
            fprintf(result_fd, "%5d %5d Failed: Only %d points left after checking nearby points\n",
                    roof_number_in, roof_number_failed, (*plane_point_set)->size()); 
          failed = true;
          roof_number_failed++;
          num_plane_size_nearby_failed++;
        }
      }
    }
    
    // Select the largest connected component to represent the plane point cloud
    if (!failed) {
      cropped_points = false;
      for (plane_point_set=plane_point_sets.begin(), i=0; 
           plane_point_set!=plane_point_sets.end() && !failed;
           plane_point_set++, i++) {
        // Derive all knn edges
        edges = (*plane_point_set)->DeriveEdges(parameters);
        // Delete long ones
        (*plane_point_set)->RemoveLongEdges(edges->TINEdgesRef(), 
                                            parameters.MaxDistanceInComponent(), false);
        // Label connected ocmponents
       (*plane_point_set)->LabelComponents(edges->TINEdgesRef(), 
                                           parameters.ComponentAttribute());

        // Determine number of most frequent component label
        component_number =
          (*plane_point_set)->MostFrequentAttributeValue(ComponentNumberTag, count);

        if (count < parameters.MinNumberOfPointsComponent()) {
          if (result_fd)
            fprintf(result_fd, "%5d %5d Failed: Only %d points left in largest connected component\n",
                    roof_number_in, roof_number_failed, count); 
          failed = true;
          roof_number_failed++;
          num_plane_size_concom_failed++;
        }
        // Remove points part of the smaller components
        else if (count < (*plane_point_set)->size()) {
          (*plane_point_set)->CropTaggedPoints(component_number,
                                               parameters.ComponentAttribute());
          cropped_points = true;
        }
        edges->Erase();
      }
    }
    
    // Fit planes again and check that points are close to the plane
    // Only continue if enough points on the plane are left
    if (!failed) {
      failed = FitAllPlanes(planes, plane_point_sets, cropped_points, result_fd,
                            max_dist_point_to_face, min_num_pts_plane,
                            roof_number_failed, num_plane_size_fit_failed,
                            roof_number_in);
    }
    
    // Check angle difference
    if (!failed) {
      // Check the angle between the two A-planes
      angle = Angle(planes[0].Normal(), planes[2].Normal());
      if (angle > max_angle_between_planes) {
        if (result_fd)
          fprintf(result_fd, "%5d %5d Failed: Angle of %.1f degrees between corresponding A-side planes\n",
                  roof_number_in, roof_number_failed, angle * 45 / atan(1.0));     
        failed = true;
        roof_number_failed++;
        num_angle_failed++;
      }
      // Check the angle between the two B-planes
      if (!failed) {
        angle = Angle(planes[1].Normal(), planes[3].Normal());
        if (angle > max_angle_between_planes) {
          if (result_fd)
            fprintf(result_fd, "%5d %5d Failed: Angle of %.1f degrees between corresponding B-side planes\n",
                    roof_number_in, roof_number_failed, angle * 45 / atan(1.0));     
          failed = true;
          roof_number_failed++;
          num_angle_failed++;
        }        
      }
    }
    
    // Check the intersection angle of the roof planes
    if (!failed) {
      angle = Angle(planes[0].Normal(), planes[1].Normal());
      if (angle > 2.0 * atan(1.0)) angle = 4.0 * atan(1.0) - angle;
      if (angle < min_intersection_angle_planes) {
        if (result_fd)
          fprintf(result_fd, "%5d %5d Failed: Angle of %.1f degrees between roof planes\n",
                  roof_number_in, roof_number_failed, angle * 45 / atan(1.0));     
        failed = true;
        roof_number_failed++;
        num_intersection_angle_failed++;
      }
    }

    // Check on the segment size with an angle depended minimum size
    if (!failed) {
      // Set minimum number of points as a function of the intersection angle
      if (fixed_minimum_number_of_points)
        min_num_pts_plane_knowing_angle = fixed_minimum_number_of_points;
      else
        min_num_pts_plane_knowing_angle = MinimumNumberOfPoints(angle);
      // Check if enough points are left
      for (plane_point_set=plane_point_sets.begin(), i=0; 
           plane_point_set!=plane_point_sets.end() && !failed;
           plane_point_set++, i++) {
        if ((*plane_point_set)->size() < min_num_pts_plane_knowing_angle) {
          if (result_fd)
            fprintf(result_fd, "%5d %5d Failed: Only %d points left (%d needed for angle %.1f)\n",
                    roof_number_in, roof_number_failed, (*plane_point_set)->size(),
                    min_num_pts_plane_knowing_angle, angle * 45.0 / atan(1.0)); 
          failed = true;
          roof_number_failed++;
          num_plane_size_nearby_failed++;
        }
      }
    }
    
    // If points of smaller segments were removed, again eliminate points
    // without nearby points in the other strip and re-estimate planes
    if (!failed && cropped_points) {
      CropToOverlappingRegions(roof_sides, plane_point_sets, parameters,
                               number_of_neighbours, afn_in_strip_number,
                               strip_number2, distance_to_point_in_other_strip);

      // Check if enough points are left
      for (plane_point_set=plane_point_sets.begin(), i=0; 
           plane_point_set!=plane_point_sets.end() && !failed;
           plane_point_set++, i++) {
        if ((*plane_point_set)->size() < min_num_pts_plane_knowing_angle) {
          if (result_fd)
            fprintf(result_fd, "%5d %5d Failed: Only %d points left after checking nearby points (%d needed for angle %.1f)\n",
                    roof_number_in, roof_number_failed, (*plane_point_set)->size(),
                    min_num_pts_plane_knowing_angle, angle * 45.0 / atan(1.0)); 
          failed = true;
          roof_number_failed++;
          num_plane_size_nearby_failed++;
        }
      }
      
      // Re-estimate planes
      if (!failed)
        failed = FitAllPlanes(planes, plane_point_sets, cropped_points, result_fd,
                              max_dist_point_to_face, min_num_pts_plane_knowing_angle,
                              roof_number_failed, num_plane_size_fit_failed,
                              roof_number_in);
    }
    
    // Check the bounding box size
    if (!failed) {
      for (plane_point_set=plane_point_sets.begin(), i=0; 
           plane_point_set!=plane_point_sets.end() && !failed;
           plane_point_set++, i++) {
        (*plane_point_set)->DeriveTIN();
        (*plane_point_set)->DeriveDataBounds(0);
        box_points.Erase();
        box_top.Erase();
        (*plane_point_set)->EnclosingRectangle(1.0, box_points, box_top);
        box_segments = LineSegments2D(box_points, box_top);
        size_bbox = box_segments.begin()->Length();
        if (size_bbox < min_size_bbox || size_bbox > max_size_bbox) {
          if (result_fd)
            fprintf(result_fd, "%5d %5d Failed: Bounding box size 1 of segment %d %.2f\n",
                    roof_number_in, roof_number_failed, i, size_bbox); 
          failed = true;
          roof_number_failed++;
          num_bbox_failed++;
        }
        else {
          size_bbox = (box_segments.begin()+1)->Length();
          if (size_bbox < min_size_bbox || size_bbox > max_size_bbox) {
            if (result_fd)
              fprintf(result_fd, "%5d %5d Failed: Bounding box size 2 of segment %d %.2f\n",
                      roof_number_in, roof_number_failed, i, size_bbox); 
            failed = true;
            roof_number_failed++;
            num_bbox_failed++;
          }
        }
        // Re-fit the plane
        segment_number = (*plane_point_set)->begin()->Attribute(SegmentNumberTag);
        planes[i] = (*plane_point_set)->FitPlane(segment_number, segment_number,
                                                 SegmentNumberTag);
      }
    }

    // Intersect planes of a strip, threshold on distance of point to ridge line
    if (!failed) {
      for (strip=strips.begin(), i=0; strip!=strips.end() && !failed; strip++, i+=2) {
        // First merge the data of the two planes within a strip
        (*strip)->ErasePoints();
        (*strip)->insert((*strip)->end(), plane_point_sets[i]->begin(),
                         plane_point_sets[i]->end());
        (*strip)->insert((*strip)->end(), plane_point_sets[i+1]->begin(),
                         plane_point_sets[i+1]->end());

        // Get the points of the roof faces, required for segment intersection
        segment_number = (*strip)->begin()->Attribute(SegmentNumberTag);
        roof_face1 = (*strip)->TaggedPointNumberList(SegmentNumberTag, segment_number);
        segment_number = ((*strip)->end()-1)->Attribute(SegmentNumberTag);
        roof_face2 = (*strip)->TaggedPointNumberList(SegmentNumberTag, segment_number);

        // Intersect planes
        if ((*strip)->IntersectFaces(roof_face1, roof_face2, planes[i], planes[i+1],
                                     max_dist_point_to_ridge, pos1, pos2)) {
          ridges.push_back(LineSegment3D(pos1, pos2));
        }
        else {
          if (result_fd)
            fprintf(result_fd, "%5d %5d Failed: No intersection line in strip %d\n",
                    roof_number_in, roof_number_failed, i/2+1);
          laser_points.ErasePoints();
          laser_points.swap(**strip);
          failed = true;
          roof_number_failed++;
          num_intersection_failed++;
        }
        roof_face1.Erase();
        roof_face2.Erase();
      }
    }
    
    
    // Check length and slope of ridge lines, also maximum length to avoid dikes?
    if (!failed) {
      for (ridge=ridges.begin(); ridge!=ridges.end() && !failed; ridge++) {    
        // Check ridge length
        if (ridge->Length() < min_ridge_length) {
          if (result_fd)
            fprintf(result_fd, "%5d %5d Failed: Ridge length only %.2f\n",
                    roof_number_in, roof_number_failed, ridge->Length());
          failed = true;
          roof_number_failed++;
          num_ridgelength_failed++;
        }
        else { // Check ridge slope angle as well
          angle = Angle(Vector3D(0.0, 0.0, 1.0), ridge->Direction());
          if (angle > 2.0 * atan(1.0)) angle = 4.0 * atan(1.0) - angle;
          angle = 2.0 * atan(1.0) - angle;
          if (angle > max_ridge_slope_angle) {
            if (result_fd)
              fprintf(result_fd, "%5d %5d Failed: Ridge slope of %.2f degree\n",
                      roof_number_in, roof_number_failed, angle * 45.0 / atan(1.0));
            failed = true;
            roof_number_failed++;
            num_ridgeslope_failed++;
          }
        }      
      }
    }
    
    // Check distance between the two ridge lines
    if (!failed) {
      ridge1 = ridges.begin();
      pos1 = ridge1->MiddlePoint();
      ridge2 = ridge1+1;
      // Check the distance
      if (ridge2->Distance(pos1) < max_dist_ridge_to_ridge) {
        // Project the mid point of ridge 1 onto ridge 2
        pos2 = ridge2->Project(pos1);
        // Check if this projection is in between the end points of the segment
        if (ridge2->Scalar(pos2) <= ridge2->ScalarBegin() &&
            ridge2->Scalar(pos2) >= ridge2->ScalarEnd()) {
          failed = true;
          if (result_fd)
            fprintf(result_fd, "%5d %5d Failed: Ridge mid point not projected on other ridge\n",
                    roof_number_in, roof_number_failed);
          roof_number_failed++;
          num_ridge_dist_failed++;
        } 
      }
      else {
        failed = true;
        if (result_fd)
          fprintf(result_fd, "%5d %5d Failed: Large distance (%.2f m) between ridge lines\n",
                  roof_number_in, roof_number_failed, ridge2->Distance(pos1));
        roof_number_failed++;
        num_ridge_dist_failed++;
      }
    }
    
    // Check angle between the two ridge lines
    if (!failed) {
      angle = Angle(ridge1->Direction(), ridge2->Direction());
      if (angle > 2.0 * atan(1.0)) angle = 4.0 * atan(1.0) - angle;
      if (angle > min_ridge_angle) {
        failed = true;
        if (result_fd)
          fprintf(result_fd, "%5d %5d Failed: Large angle (%.2f degrees) between ridge lines\n",
                  roof_number_in, roof_number_failed, angle * 45.0 / atan(1.0));
        roof_number_failed++;
        num_ridge_angle_failed++;
      }
    }
    
    // Merge latest selection of laser points
    laser_points.ErasePoints();
    num_pts = plane_point_set1a.size(); // To be used for accepted roofs later
    laser_points.swap(plane_point_set1a);
    laser_points.insert(laser_points.end(), plane_point_set1b.begin(), plane_point_set1b.end());
    laser_points.insert(laser_points.end(), plane_point_set2a.begin(), plane_point_set2a.end());
    laser_points.insert(laser_points.end(), plane_point_set2b.begin(), plane_point_set2b.end());
    
    // Done with testing, output to various files
    if (!failed) { 
      // Determine smallest roof plane
      if (plane_point_set1b.size() < num_pts) num_pts = plane_point_set1b.size();
      if (plane_point_set2a.size() < num_pts) num_pts = plane_point_set2a.size();
      if (plane_point_set2b.size() < num_pts) num_pts = plane_point_set2b.size();
      // Recalculate angle
      angle = Angle(planes[0].Normal(), planes[1].Normal());
      if (angle > 2.0 * atan(1.0)) angle = 4.0 * atan(1.0) - angle;
      // Output of statistics
      fprintf(statistics_fd,
              "%4d %3d %10.3f %10.3f %7.3f %3d %10.3f %10.3f %7.3f %5.3f %6.3f %6.4f %1d\n",
              roof_number_verified, 
              strip_number1, pos1.X(), pos1.Y(), pos1.Z(),
              strip_number2, pos2.X(), pos2.Y(), pos2.Z(), 
              (pos2 - pos1).Length2D(), pos2.Z() - pos1.Z(),
              StandardDeviation(angle, num_pts), 1);
      if (result_fd)
        fprintf(result_fd, "%5d %5d Verified\n", roof_number_in, roof_number_verified);
      // Generation of output file names
      sprintf(roof_name_v, "roofv%6d", roof_number_verified);
      for (ch=roof_name_v; *ch!=0; ch++) if (*ch == ' ') *ch = '0';
      laser_name = ComposeFileName(output_directory_verified, roof_name_v, "laser");
      points_name = ComposeFileName(output_directory_verified, roof_name_v, "objpts");
      top_name = ComposeFileName(output_directory_verified, roof_name_v, "top");
      // Update counts
      roof_number_verified++;
    }
    else if (result_fd) {
      // Generation of output file names
      sprintf(roof_name_f, "rooff%6d", roof_number_failed-1);
      for (ch=roof_name_f; *ch!=0; ch++) if (*ch == ' ') *ch = '0';
      laser_name = ComposeFileName(output_directory_failed, roof_name_f, "laser");
      if (ridges.size()) {
        points_name = ComposeFileName(output_directory_failed, roof_name_f, "objpts");
        top_name = ComposeFileName(output_directory_failed, roof_name_f, "top");
      }
    }
    // Output of laser points
    laser_points.Write(laser_name, false, false);
    // Output of meta data file in PCM format
    if (!failed)
      pcm_fd = fopen(ComposeFileName(output_directory_verified, roof_name_v, "pcm"), "w");
    else
      pcm_fd = fopen(ComposeFileName(output_directory_failed, roof_name_f, "pcm"), "w");
    fprintf(pcm_fd, "pointcloudmapping:\n");
    fprintf(pcm_fd, "  laser_points: \"%s\"\n", laser_name);
    if (ridges.size()) {
      fprintf(pcm_fd, "  map_points: \"%s\"\n", points_name);
      fprintf(pcm_fd, "  map_topology: \"%s\"\n", top_name);
    }
    fprintf(pcm_fd, "endpointcloudmapping:\n");
    fclose(pcm_fd);

    // Ridge lines
    matched_lines.Erase();
    for (ridge=ridges.begin(), i=0; ridge!=ridges.end(); ridge++, i++) {
      // Generation ridge end points
      ridge_point.Number() = i*2;
      ridge_point.vect() = ridge->BeginPoint().vect();
      ridge_points.push_back(ridge_point);
      ridge_point.Number() = i*2+1;
      ridge_point.vect() = ridge->EndPoint().vect();
      ridge_points.push_back(ridge_point);
      // Generation of ridge line topology
      matched_line.Erase();
      matched_line.push_back(PointNumber(i*2));
      matched_line.push_back(PointNumber(i*2+1));
      matched_line.Label() = 1;
      matched_line.Number() = i;
      matched_line.Attribute(BuildingNumberTag) = i;
      if (ridge == ridges.begin()) matched_line.Attribute(StripNumberTag) = strip_number1;
      else matched_line.Attribute(StripNumberTag) = strip_number2;
      matched_lines.push_back(matched_line);
    }
    // Output of ridge points and topology
    if (ridges.size()) {
      ridge_points.Write(points_name);
      matched_lines.Write(top_name, false);
    }
 
    // Erase data
    laser_points.ErasePoints();
    ridge_points.Erase();
    ridges.erase(ridges.begin(), ridges.end());       

    // Get next filename
    roof_number_in++;
    if (five_digit_numbers)
      sprintf(roof_name, "roof%5d", roof_number_in);
    else
      sprintf(roof_name, "roof%6d", roof_number_in);
    for (ch=roof_name; *ch!=0; ch++) if (*ch == ' ') *ch = '0';
    laser_name = (char *) malloc(strlen(roof_name) + 7);
    sprintf(laser_name, "%s%s", roof_name, ".laser");

    // Progress output
    printf("%6d%6d%6d%6d%6d%6d%6d%7d%6d%6d%6d%6d%6d\r", num_roof,
           roof_number_failed - first_roof_number_out_failed, 
           num_plane_size_fit_failed, num_plane_size_nearby_failed,
           num_plane_size_concom_failed, num_angle_failed, num_intersection_angle_failed,
           num_bbox_failed, num_intersection_failed, num_ridgelength_failed,
           num_ridge_dist_failed, num_ridgeslope_failed, num_ridge_angle_failed);
  } // end of loop over roof point files        
  fclose(statistics_fd);
  if (result_fd) fclose(result_fd);
}

void CropToOverlappingRegions(vector<LaserPoints *> &roof_sides,
                              vector<LaserPoints *> &plane_point_sets,
                              SegmentationParameters &parameters,
                              int number_of_neighbours,
                              bool afn_in_strip_number,
                              int strip_number2,
                              double distance_to_point_in_other_strip)
{
  vector<LaserPoints*>::iterator    plane_point_set, roof_side;
  int                               i;
  TINEdges                          *edges=NULL;
  LaserPoints::iterator             laser_point, nb_point;
  bool                              done;
  PointNumberList                   neighbourhood;
  PointNumberList::iterator         nb_node;
  PointNumber                       node;
  
  for (i=0, roof_side=roof_sides.begin(); i<2; i++, roof_side++) {
    (*roof_side)->ErasePoints();
    (*roof_side)->insert((*roof_side)->end(), plane_point_sets[i]->begin(),
                         plane_point_sets[i]->end());
    (*roof_side)->insert((*roof_side)->end(), plane_point_sets[i+2]->begin(),
                         plane_point_sets[i+2]->end());
    edges = (*roof_side)->DeriveEdges(parameters);
    (*roof_side)->RemoveAttribute(IsSelectedTag);
    for (laser_point=(*roof_side)->begin(), node=PointNumber(0);
         laser_point!=(*roof_side)->end(); laser_point++, node++) {
      // Get all points in a fixed distance neighbourhood
      neighbourhood = 
        (*roof_side)->Neighbourhood(node, distance_to_point_in_other_strip,
                                    edges->TINEdgesRef(), false, true);
      // Select points without neighbours in the other strip
      for (nb_node=neighbourhood.begin(), done=false;
           nb_node!=neighbourhood.end() && !done; nb_node++) {
        nb_point = (*roof_side)->begin() + nb_node->Number();
        if (laser_point->ScanNumber(afn_in_strip_number) !=
            nb_point->ScanNumber(afn_in_strip_number)) done = true;
      }
      if (!done) laser_point->Select();
    }
    // Erase selected points
    (*roof_side)->RemoveTaggedPoints(1, IsSelectedTag);
    edges->Erase();

    // Split the remaining points again into the two strips
    for (laser_point=(*roof_side)->begin(), done=false;
         laser_point!=(*roof_side)->end() && !done; laser_point++) {
      if (laser_point->ScanNumber(afn_in_strip_number) == strip_number2) {
        done = true;
        plane_point_sets[i]->ErasePoints();
        if (laser_point != (*roof_side)->begin())
          plane_point_sets[i]->insert(plane_point_sets[i]->begin(),
                                      (*roof_side)->begin(), laser_point-1);
        plane_point_sets[i+2]->ErasePoints();
        plane_point_sets[i+2]->insert(plane_point_sets[i+2]->begin(),
                                      laser_point, (*roof_side)->end());
      }
    }
  }
}

bool FitAllPlanes(Planes &planes, vector<LaserPoints *> &plane_point_sets,
                  bool &cropped_points, FILE *result_fd,
                  double max_dist_point_to_face,
                  int min_num_pts_plane,
                  int &num_failed, int &num_plane_size_fit_failed,
                  int roof_number_in)
{
  vector<LaserPoints *>::iterator plane_point_set;
  Plane                           plane;
  int                             segment_number, i, num_removed;
  bool                            failed=false;
  
  planes.Erase();
  for (plane_point_set=plane_point_sets.begin(), i=0; 
       plane_point_set!=plane_point_sets.end() && !failed;
       plane_point_set++, i++) {
    segment_number = (*plane_point_set)->begin()->Attribute(SegmentNumberTag);
    plane = (*plane_point_set)->FitPlane(segment_number, segment_number,
                                         SegmentNumberTag);
    (*plane_point_set)->Label(0);
    (*plane_point_set)->Label(plane, max_dist_point_to_face, 0, 0, 0, 1, LabelTag);
    num_removed = (*plane_point_set)->RemoveLabeledPoints(1);
    if ((*plane_point_set)->size() < min_num_pts_plane) {
      if (result_fd)
        fprintf(result_fd, "%5d %5d Failed: Only %d points left after plane fit\n",
                roof_number_in, num_failed, (*plane_point_set)->size()); 
      failed = true;
      num_failed++;
      num_plane_size_fit_failed++;
    }
    if (num_removed) cropped_points = true;
    // Make sure the normal is pointing upwards
    if (plane.Normal().Z() < 0.0) plane.SwapNormal();
    planes.push_back(plane);
  }
  return failed;
}

/* Function to determine the minimum number of required points on a roof plane
   to ensure that the standard deviation in the planimetric distance between 
   to roof ridges due to the structure of roof tiles is less than stdev. A value
   of 1.3 cm leads to a maximum influence of 10% in the typical 3.0 cm total 
   standard deviation in the planimetric distances (due to the navigation
   solution and roof tiles). The function has been determined emperically
   based on random subsampling of point clouds on larger roofs. More points
   are required for flatter gable roofs.
   
*/

int MinimumNumberOfPoints(double angle_rad)
{
  double low_angle, high_angle, low_scale, high_scale, scale,
         low_offset, high_offset, offset, num;
  double angle = angle_rad * 45.0 / atan(1.0);
  double stdev = 0.02; // Influence of maximum 25%
  
  // TODO: Check definition of angle
  
  if (angle > 90.0) angle = 180.0 - angle;
  if (angle < 20.0) return 1000000000; // Nothing goes below 20 degrees
  
  // Linear interpolation of scale and offset in four intervals
  
  if (angle < 30.0) {
    low_angle   = 20.0;
    high_angle  = 30.0;
    low_scale   =  0.2685;
    high_scale  =  0.2084;
    low_offset  = -0.0042;
    high_offset = -0.0044;
  }
  else if (angle < 44.0) {
    low_angle   = 30.0;
    high_angle  = 44.0;
    low_scale   =  0.2084;
    high_scale  =  0.1774;
    low_offset  = -0.0044;
    high_offset = -0.0048;
  }
  else if (angle < 70.0) {
    low_angle   = 44.0;
    high_angle  = 70.0;
    low_scale   =  0.1774;
    high_scale  =  0.1185;
    low_offset  = -0.0048;
    high_offset = -0.0031;
  }
  else {
    low_angle   = 70.0;
    high_angle  = 90.0;
    low_scale   =  0.1185;
    high_scale  =  0.0948;
    low_offset  = -0.0031;
    high_offset = -0.0030;
  }
  scale  = ((angle - low_angle) * high_scale + (high_angle - angle) * low_scale) /
           (high_angle - low_angle);
  offset = ((angle - low_angle) * high_offset + (high_angle - angle) * low_offset) /
           (high_angle - low_angle);
           
  // Derive required minimum number of points
  num = scale / (stdev - offset);
  num = num * num;
  
  return ((int) num);
}

/* Function to estimate the standard deviation of the ridge line distance
   due to roof tile structures.
*/

double StandardDeviation(double angle_rad, int num)
{
  double low_angle, high_angle, low_scale, high_scale, scale,
         low_offset, high_offset, offset, stdev;
  double angle = angle_rad * 45.0 / atan(1.0);
  
  if (angle > 90.0) angle = 180.0 - angle;
//  if (angle < 20.0) return 1000000000.0; // Nothing goes below 20 degrees
  
  // Linear interpolation of scale and offset in four intervals
  
  if (angle < 30.0) {
    low_angle   = 20.0;
    high_angle  = 30.0;
    low_scale   =  0.2685;
    high_scale  =  0.2084;
    low_offset  = -0.0042;
    high_offset = -0.0044;
  }
  else if (angle < 44.0) {
    low_angle   = 30.0;
    high_angle  = 44.0;
    low_scale   =  0.2084;
    high_scale  =  0.1774;
    low_offset  = -0.0044;
    high_offset = -0.0048;
  }
  else if (angle < 70.0) {
    low_angle   = 44.0;
    high_angle  = 70.0;
    low_scale   =  0.1774;
    high_scale  =  0.1185;
    low_offset  = -0.0048;
    high_offset = -0.0031;
  }
  else {
    low_angle   = 70.0;
    high_angle  = 90.0;
    low_scale   =  0.1185;
    high_scale  =  0.0948;
    low_offset  = -0.0031;
    high_offset = -0.0030;
  }
  scale  = ((angle - low_angle) * high_scale + (high_angle - angle) * low_scale) /
           (high_angle - low_angle);
  offset = ((angle - low_angle) * high_offset + (high_angle - angle) * low_offset) /
           (high_angle - low_angle);
           
  // Calculate standard deviation
  stdev = scale / sqrt((double) num) + offset;
  if (stdev < 0.0) stdev = 0.00001;
  
  return stdev;
}
