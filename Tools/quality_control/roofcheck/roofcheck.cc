
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
 Date   : 16-01-2007

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "LaserBlock.h"
#include "LineSegment3D.h"

/*
--------------------------------------------------------------------------------
                      The main roofcheck function
--------------------------------------------------------------------------------
*/

void RoofCheck(LaserBlock &block_overlaps,
               bool collect_all_ridge_points, ObjectPoints &ridge_points,
               LineTopologies &ridge_lines,
               bool collect_all_roof_points, LaserPoints &merged_roof_points,
               const char *statistics_file_name,
               SegmentationParameters &parameters,
               bool store_segmented_points, bool store_results_per_roof,
               double max_dist_point_to_ridge, double min_ridge_length,
               double max_ridge_slope, double max_dist_ridge_to_ridge,
               bool afn_in_strip_number, int first_building_number,
			   int restart_strip_offset, int restart_part_offset)
{
  LaserBlock::iterator               strip_overlap;
  LaserUnit::iterator                overlap_part;
  LaserPoints                        strip1, strip2;
  LaserPoints::const_iterator        overlap_point;
  LaserPoints::iterator              point;
  LaserSubUnit                       roof_point_set;
  LaserUnit                          roof_point_sets1, roof_point_sets2,
                                     *roof_point_sets;
  LaserUnit::iterator                roof_point_set1, roof_point_set2;
  vector<LaserPoints *>              strips;
  vector<LaserPoints*>::iterator     strip;
  Plane                              plane;
  Planes                             planes;
  Planes::iterator                   plane1, plane2;
  PointNumberList                    roof_face;
  PointNumberList::iterator          roof_node;
  PointNumberLists                   roof_faces;
  PointNumberLists::iterator         roof_face1, roof_face2;
  int                                max_segment_number, segment_number, index,
                                     stripnumbertag, stripnumber, part_number,
                                     stripnumber1, stripnumber2, match_number;
  double                             slope_angle, orientation;
  Position3D                         pos1, pos2;
  Vector3D                           ridge;
  ObjectPoint                        ridge_point, matched_point;
  ObjectPoints                       matched_points;
  LineTopology                       ridge_line, matched_line;
  LineTopologies                     matched_lines;
  vector<LineSegment3D>              ridge_segments1, ridge_segments2,
                                     *ridge_segments;
  vector<LineSegment3D>::iterator    ridge_segment1, ridge_segment2;
  FILE                               *statistics_fd, *pcm_fd;
  char                               roof_name[11], *ch, filename[21];
    
  // Open output file for statistics
  if (statistics_file_name) {
  	if (!BNF_FileExists(statistics_file_name)) {
      statistics_fd = fopen(statistics_file_name, "w");
      if (statistics_fd == NULL) {
        printf("Error opening statistics file %s\n", statistics_file_name);
        exit(0);
      }
      fprintf(statistics_fd, " bld strip1   X1         Y1      Z1    strip2   X2         Y2      Z2     dXY    dZ\n");
    }
    else {
      statistics_fd = fopen(statistics_file_name, "a");
      if (statistics_fd == NULL) {
        printf("Error opening statistics file %s\n", statistics_file_name);
        exit(0);
      }
    }
  }

  // Standard topology for matched ridge lines
  if (store_results_per_roof) {
    matched_line.push_back(PointNumber(0));
    matched_line.push_back(PointNumber(1));
    matched_line.Label() = 1;
    matched_line.Number() = 0;
    matched_lines.push_back(matched_line);
    matched_line.Erase();
    matched_line.push_back(PointNumber(2));
    matched_line.push_back(PointNumber(3));
    matched_line.Number() = 1000;
    matched_lines.push_back(matched_line);
    matched_lines.Write("ridges.top");
  }

  match_number = first_building_number - 1;
  ridge_line.Number() = 0;
  ridge_point.Number() = 0;
  if (restart_strip_offset || restart_part_offset)
  	printf("Restarting at part %d of overlap %d\n",
	       restart_part_offset, restart_strip_offset);
  // Loop over all strip overlaps
  for (strip_overlap=block_overlaps.begin()+restart_strip_offset-1;
       strip_overlap!=block_overlaps.end(); strip_overlap++) {
    // Loop over all parts of this overlap
    for (overlap_part=strip_overlap->begin(), part_number=1;
         overlap_part!=strip_overlap->end(); overlap_part++, part_number++) {
      if (strip_overlap == block_overlaps.begin() + restart_strip_offset - 1 &&
          part_number == 1 && restart_part_offset > 1) {
	    part_number = restart_part_offset;
	    overlap_part += restart_part_offset - 1;
	  }
      // Clear strip points and TINs of previous part
      strip1.ErasePoints();
      strip1.EraseTIN();
      strip2.ErasePoints();
      strip2.EraseTIN();
      // Read the points of this part
      if (!overlap_part->Read(overlap_part->PointFile(), false)) {
        printf("Error reading overlap part %s\n", overlap_part->Name());
        exit(0);
      }
      // Check if the strip number is in StripNumberTag, otherwise assume
      // strip number in LabelTag
      if (!overlap_part->empty()) {
        if (overlap_part->begin()->HasAttribute(ScanNumberTag))
          stripnumbertag = (int) ScanNumberTag;
        else
          stripnumbertag = (int) LabelTag;
        // First split the points into separate point sets for the two strips
        stripnumber1 =
          overlap_part->begin()->Attribute((LaserPointTag) stripnumbertag);
        stripnumber2 =
         (overlap_part->end()-1)->Attribute((LaserPointTag) stripnumbertag);
      }
      if (afn_in_strip_number) {
        stripnumber1 /= 10;
        stripnumber2 /= 10;
      }
      for (overlap_point=overlap_part->begin();
           overlap_point!=overlap_part->end(); overlap_point++) {
        stripnumber = overlap_point->Attribute((LaserPointTag) stripnumbertag);
        if (afn_in_strip_number) stripnumber /= 10;
        if (stripnumber == stripnumber1) strip1.push_back(*overlap_point);
        else strip2.push_back(*overlap_point);
      }
      printf("Overlap part %s contains %d points of strip %d and %d points of strip %d\n",
             overlap_part->Name(), strip1.size(), stripnumber1, strip2.size(), stripnumber2);
      strips.erase(strips.begin(), strips.end());
      strips.push_back(&strip1);
      strips.push_back(&strip2);
      overlap_part->ErasePoints();

      ridge_segments    = &ridge_segments1;
      roof_point_sets   = &roof_point_sets1;
      for (strip=strips.begin(); strip!=strips.end(); strip++) {         
        // Segment the strip part, but avoid segmenting the part of the second
        // strip if the first strip did not contain any ridges
        max_segment_number = 0;
        if (strip == strips.begin() || !ridge_segments1.empty()) {
          (*strip)->SurfaceGrowing(parameters);

          // Determine the highest segment number
          for (point=(*strip)->begin(); point!=(*strip)->end(); point++) {
            if (point->HasAttribute(SegmentNumberTag))
              if (point->Attribute(SegmentNumberTag) > max_segment_number)
                max_segment_number = point->Attribute(SegmentNumberTag);
          }
        }

        /*--- Extract large sloped faces -------------------------------------*/

        // Fit planes
        for (segment_number=0; segment_number<=max_segment_number;
             segment_number++) {
          plane = (*strip)->FitPlane(segment_number, segment_number,
                                     SegmentNumberTag);
          // Check plane size
          if (plane.NumberOfPoints() >= parameters.MinNumberOfPointsComponent()) {
            // Check plane slope
            slope_angle = acos(fabs(plane.Normal().Z()));
            if (slope_angle > 5.0 * atan(1.0) / 45.0 &&
                slope_angle < 75.0 * atan(1.0) / 45.0) {
              // Store plane and plane points for later
              planes.push_back(plane);
              for (point=(*strip)->begin(), index=0; point!=(*strip)->end();
                   point++, index++)
                if (point->Attribute(SegmentNumberTag) == segment_number)
                  roof_face.push_back(PointNumber(index));
              roof_faces.push_back(roof_face);
              roof_face.Erase();
            }
          }
        }
        /*----- Extract gable roofs ------------------------------------------*/
        for (plane1=planes.begin(), roof_face1=roof_faces.begin();
             plane1!=planes.end(); plane1++, roof_face1++) {
          for (plane2=plane1+1, roof_face2=roof_face1+1;
               plane2!=planes.end(); plane2++, roof_face2++) {
            if ((*strip)->IntersectFaces(roof_face1->PointNumberListReference(),
                                         roof_face2->PointNumberListReference(),
                                         *plane1, *plane2,
                                         max_dist_point_to_ridge, pos1, pos2)) {
              ridge = pos1.vect() - pos2.vect();
              // Check ridge length and whether we have a ridge (and not a valley)
              if (ridge.Length() > min_ridge_length &&
                  pos1.Z() > plane1->CentreOfGravity().Z() &&
                  pos1.Z() > plane2->CentreOfGravity().Z()) {
                // Check ridge slope
                if (fabs(ridge.Z()) / ridge.Length() < max_ridge_slope) {
                  orientation = ridge.Direction2D(Vector3D(0.0, 0.0, 0.0)) *
                                45.0 / atan(1.0);
                  if (orientation < 0.0) orientation += 180.0;
                  printf("Ridge line, length %5.2f, orientation %5.1f\n",
                        ridge.Length(), orientation);
                  ridge_segments->push_back(LineSegment3D(pos1, pos2));
                  // Add the laser points in the roof faces to the roof specific point set
                  roof_point_set.ErasePoints();
                  for (roof_node=roof_face1->begin();
                       roof_node!=roof_face1->end(); roof_node++) {
                    point = (*strip)->begin() + roof_node->Number();
                    roof_point_set.push_back(*point);
                  }
                  for (roof_node=roof_face2->begin();
                       roof_node!=roof_face2->end(); roof_node++) {
                    point = (*strip)->begin() + roof_node->Number();
                    roof_point_set.push_back(*point);
                  }
                  roof_point_sets->push_back(roof_point_set);
                }
              }
            }
          }
        }
        // Clean up planes and roof face point number lists
        planes.Erase();
        roof_faces.Erase();
        ridge_segments = &ridge_segments2;
        roof_point_sets = &roof_point_sets2;
      }
      // Store the segmented strip parts
      if (store_segmented_points) {
/*
        overlap_part->insert(overlap_part->end(), strip1.begin(), strip1.end());
        overlap_part->insert(overlap_part->end(), strip2.begin(), strip2.end());
        overlap_part->Write();
*/
        overlap_part->swap(strip1);
        overlap_part->insert(overlap_part->end(), strip2.begin(), strip2.end());
        strip2.ErasePoints();
        overlap_part->Write();
        overlap_part->ErasePoints();
      }
      // Match ridge lines of the two line sets
      for (ridge_segment1=ridge_segments1.begin(),
           roof_point_set1=roof_point_sets1.begin();
           ridge_segment1!=ridge_segments1.end();
           ridge_segment1++, roof_point_set1++) {
        // Determine mid point of ridge line
        pos1 = ridge_segment1->MiddlePoint();
        for (ridge_segment2=ridge_segments2.begin(),
             roof_point_set2=roof_point_sets2.begin();
             ridge_segment2!=ridge_segments2.end();
             ridge_segment2++, roof_point_set2++) {
          // Check the distance
          if (ridge_segment2->Distance(pos1) < max_dist_ridge_to_ridge) {
            // Project the mid point of ridge 1 onto ridge 2
            pos2 = ridge_segment2->Project(pos1);
            // Check if this projection is in between the end points of the segment
            if (ridge_segment2->Scalar(pos2) >= ridge_segment2->ScalarBegin() &&
                ridge_segment2->Scalar(pos2) <= ridge_segment2->ScalarEnd()) {
              // This is a corresponding ridge pair, construct name
              match_number++;
              sprintf(roof_name, "roof%6d", match_number);
              for (ch=roof_name; *ch!=0; ch++)
                if (*ch == ' ') *ch = '0';
              matched_points.erase(matched_points.begin(), matched_points.end());
              // Calculate statistics on vertical and horizontal distance
              printf("Ridge pair %d, height offset %5.2f, planimetric offset %5.2f\n",
                     match_number, pos2.Z() - pos1.Z(), (pos2 - pos1).Length2D());
              // Store the ridge pair in the line topology output
              // Line segment in first strip
              ridge_point.Number() += 1;
              ridge_point.vect() = ridge_segment1->BeginPoint().vect();
              if (collect_all_ridge_points) {
                ridge_points.push_back(ridge_point);
                ridge_line.Erase();
                ridge_line.Number() += 1;
                ridge_line.push_back(ridge_point.NumberRef());
              }
              if (store_results_per_roof) {
                matched_point = ridge_point;
                matched_point.Number() = 0;
                matched_points.push_back(matched_point);
              }
              ridge_point.Number() += 1;
              ridge_point.vect() = ridge_segment1->EndPoint().vect();
              if (collect_all_ridge_points) {
                ridge_points.push_back(ridge_point);
                ridge_line.push_back(ridge_point.NumberRef());
                ridge_lines.push_back(ridge_line);
              }
              if (store_results_per_roof) {
                matched_point = ridge_point;
                matched_point.Number() = 1;
                matched_points.push_back(matched_point);
              }
              // Line segment in second strip
              ridge_point.Number() += 1;
              ridge_point.vect() = ridge_segment2->BeginPoint().vect();
              if (collect_all_ridge_points) {
                ridge_points.push_back(ridge_point);
                ridge_line.Erase();
                ridge_line.Number() += 1;
                ridge_line.push_back(ridge_point.NumberRef());
              }
              if (store_results_per_roof) {
                matched_point = ridge_point;
                matched_point.Number() = 2;
                matched_points.push_back(matched_point);
              }
              ridge_point.Number() += 1;
              ridge_point.vect() = ridge_segment2->EndPoint().vect();
              if (collect_all_ridge_points) {
                ridge_points.push_back(ridge_point);
                ridge_line.push_back(ridge_point.NumberRef());
                ridge_lines.push_back(ridge_line);
              }
              if (store_results_per_roof) {
                matched_point = ridge_point;
                matched_point.Number() = 3;
                matched_points.push_back(matched_point);
              }
              // Write results to statistics file
              if (statistics_file_name)
                fprintf(statistics_fd,
                        "%4d %3d %10.3f %10.3f %7.3f %3d %10.3f %10.3f %7.3f %5.3f %6.3f %3d\n",
                        match_number, 
                        stripnumber1, pos1.X(), pos1.Y(), pos1.Z(),
                        stripnumber2, pos2.X(), pos2.Y(), pos2.Z(), 
                        (pos2 - pos1).Length2D(), pos2.Z() - pos1.Z(),
                        part_number);
              if (store_results_per_roof) {
                // Store the laser points of this building
                roof_point_set.ErasePoints();
                roof_point_set.insert(roof_point_set.begin(),
                                      roof_point_set1->begin(),
                                      roof_point_set1->end());
                roof_point_set.insert(roof_point_set.end(),
                                      roof_point_set2->begin(),
                                      roof_point_set2->end());
                if (collect_all_roof_points)
                  merged_roof_points.insert(merged_roof_points.end(),
                                            roof_point_set.begin(),
                                            roof_point_set.end());
                sprintf(filename, "%s.laser", roof_name);
                roof_point_set.Write(filename, 0 , false);
                // Store the end and mid points of this building
                matched_point.vect() = pos1.vect();
                matched_point.Number() = 4;
                matched_points.push_back(matched_point);
                matched_point.vect() = pos2.vect();
                matched_point.Number() = 5;
                matched_points.push_back(matched_point);
                sprintf(filename, "%s.objpts", roof_name);
                matched_points.Write(filename);
                // Write the PCM meta data file
                sprintf(filename, "%s.pcm", roof_name);
                pcm_fd = fopen(filename, "w");
                fprintf(pcm_fd, "pointcloudmapping:\n");
                sprintf(filename, "%s.laser", roof_name);
                fprintf(pcm_fd, "  laser_points: \"%s\"\n", filename);
                sprintf(filename, "%s.objpts", roof_name);
                fprintf(pcm_fd, "  map_points: \"%s\"\n", filename);
                fprintf(pcm_fd, "  map_topology: \"ridges.top\"\n");
                fprintf(pcm_fd, "endpointcloudmapping:\n");
                fclose(pcm_fd);
              }
            }
          }
        }
      }
      // Clear the two line sets of this strip overlap part
      ridge_segments1.erase(ridge_segments1.begin(), ridge_segments1.end());      
      ridge_segments2.erase(ridge_segments2.begin(), ridge_segments2.end());      
      // Clear the point sets of the roofs
      for (roof_point_set1=roof_point_sets1.begin();
           roof_point_set1!=roof_point_sets1.end(); roof_point_set1++)
        roof_point_set1->ErasePoints();
      roof_point_sets1.erase(roof_point_sets1.begin(), roof_point_sets1.end());
      for (roof_point_set2=roof_point_sets2.begin();
           roof_point_set2!=roof_point_sets2.end(); roof_point_set2++)
        roof_point_set2->ErasePoints();
      roof_point_sets2.erase(roof_point_sets2.begin(), roof_point_sets2.end());
      
      // Close the statistics file and re-open it to force saving of
      // all results up to now.
      if (statistics_file_name) {
        fclose(statistics_fd);
        statistics_fd = fopen(statistics_file_name, "a");
      }
    }
  }
  
  // Close the statistics file
  if (statistics_file_name) fclose(statistics_fd);
}
/* TODO: 
- Extend Line2D and Line3D with LineNumber
- Define class LineSegments3D
*/
