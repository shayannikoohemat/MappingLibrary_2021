/*
                    Copyright 2013 University of Twente
 
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

#include "LaserBlock.h"
#include "BSplineFit.h"

void TransformFromFrameToWorld(LaserPoints &points, double start_time,
                               double end_time, BSplineFit &omega_spline,
                               BSplineFit &phi_spline, BSplineFit &kappa_spline,
							   BSplineFit &X_spline, BSplineFit &Y_spline,
							   BSplineFit &Z_spline)
{
  double                time;
  LaserPoints::iterator point;
  Rotation3D            rotation;
  Vector3D              translation;
  
  for (point=points.begin(); point!=points.end(); point++) {
  	time = point->DoubleAttribute(TimeTag);
  	if (time < start_time || time > end_time) continue;
  	rotation = Rotation3D(omega_spline.Value(time), phi_spline.Value(time),
						  kappa_spline.Value(time));
	translation = Vector3D(X_spline.Value(time), Y_spline.Value(time),
	                       Z_spline.Value(time));
    point->vect() = rotation * point->vect() + translation;
  }
}




void SaveTransformedPointSet(int interval, double start_time, double end_time,
                             const LaserBlock &local_block,
                             BSplineFit &omega_spline, BSplineFit &phi_spline,
						     BSplineFit &kappa_spline, BSplineFit &X_spline,
						     BSplineFit &Y_spline, BSplineFit &Z_spline,
						     char *base_name, char *dir, Planes &planes)
{
  LaserPoints           copied_points;
  LaserPoint            frame_position;
  double                time, time_interval;
  char                  *name, *point_file_name, *ch;
  Planes::iterator      plane;
  LaserPoints::iterator point;
  int                   i, sensor;

  // Make a copy of the local point sets
  for (sensor=0; sensor<3; sensor++) {
  	copied_points.insert(copied_points.end(), local_block[sensor][0].begin(),
	                     local_block[sensor][0].end());
  }
  
  // Transform with initial splines
  TransformFromFrameToWorld(copied_points, start_time, end_time,
                            omega_spline, phi_spline, kappa_spline,
	                        X_spline, Y_spline, Z_spline);

  // Add residuals
  for (point=copied_points.begin(); point!=copied_points.end(); point++) {
  	if (!point->HasAttribute(PlaneNumberTag)) continue;
    plane = planes.begin() + point->Attribute(PlaneNumberTag);
    point->FloatAttribute(ResidualTag) =
      plane->Distance(point->Position3DRef());
  }

  // Add the sensor positions of the two intervals
  time_interval = end_time - start_time;
  time = start_time + time_interval / 4.0;
  frame_position.X() = X_spline.Value(time);
  frame_position.Y() = Y_spline.Value(time);
  frame_position.Z() = Z_spline.Value(time);
  frame_position.Attribute(ScanLineNumberTag) = interval-1;
  copied_points.push_back(frame_position);
  time += time_interval / 2.0;
  frame_position.X() = X_spline.Value(time);
  frame_position.Y() = Y_spline.Value(time);
  frame_position.Z() = Z_spline.Value(time);
  frame_position.Attribute(ScanLineNumberTag) = interval;
  copied_points.push_back(frame_position);
           
  // Compose file name
  name = (char *) malloc(strlen(base_name) + 6);
  if (!name) {printf("Error allocating name in SaveTransformedPointSet\n"); exit(0);}
  sprintf(name, "%s%5d", base_name, interval);
  for (i=0, ch=name+strlen(base_name); i<5; i++, ch++)
    if (*ch == ' ') *ch = '0';
  point_file_name = ComposeFileName(dir, name, ".laser");
  
  // Write points and delete them
  copied_points.Write(point_file_name, 0, false);
  copied_points.ErasePoints();
  
  // Release memory
  free(point_file_name);
  free(name);
}

void SaveTransformedBlock(double start_time, double end_time, LaserBlock &block,
                          BSplineFit &omega_spline, BSplineFit &phi_spline,
						  BSplineFit &kappa_spline, BSplineFit &X_spline,
						  BSplineFit &Y_spline, BSplineFit &Z_spline,
						  char *directory, char *appendix,
						  double interval_duration)
{
  LaserBlock            transformed_block;
  LaserBlock::iterator  strip;
  LaserUnit             transformed_strip;
  LaserUnit::iterator   pointset;
  LaserSubUnit          transformed_pointset;
  LaserPoint            frame_position;
  double                time;
  char                  *name;
  LaserPoints::iterator point;
  int                   interval, first_interval, last_interval;
  bool                  found;

  for (strip=block.begin(); strip!=block.end(); strip++) {
  	for (pointset=strip->begin(); pointset!=strip->end(); pointset++) {
  		
  	  // Check if a part of the point set is within the time interval
  	  if (pointset->StartTime() > end_time ||
		  pointset->EndTime() < start_time) continue;
  		
  	  // Create new meta data
  	  if (!pointset->Name()) pointset->LaserDataFiles::DeriveName();
      name = (char *) malloc(strlen(pointset->Name()) + strlen(appendix) + 1);
      if (!name) {printf("Error allocating name in SaveTransformedBlock\n"); exit(0);}
      sprintf(name, "%s%s", pointset->Name(), appendix);
      printf("Point set %s\r", name); fflush(stdout);
      transformed_pointset.SetName(name);
      transformed_pointset.DataOrganisation() = StripWise;
      transformed_pointset.DeriveMetaDataFileName(directory);
      transformed_pointset.DerivePointFileName(directory);
      free(name);
  	  
  	  // Read the points in the frame coordinate system
  	  if (!pointset->Read()) {
  	  	printf("Error reading point set from %s\n", pointset->PointFile());
  	  	exit(0);
  	  }
  	  if (!pointset->size()) continue;

  	  // Remove points before the trajectory start time
  	  if (pointset->begin()->DoubleAttribute(TimeTag) < start_time) {
  	  	// Find first point after start time
  	  	for (point=pointset->begin(), found=false;
			 point!=pointset->end() && !found; point++) {
		  if (point->DoubleAttribute(TimeTag) >= start_time) found = true;
		}
		if (!found) {
		  printf("Point set %s does not contain a point within the time bounds\n",
		         pointset->Name());
		  pointset->ErasePoints();
		  continue;
		}
		else {
		  pointset->erase(pointset->begin(), point);
		}
  	  }
  	  
  	  // Remove points after the trajectory end time
  	  if ((pointset->end()-1)->DoubleAttribute(TimeTag) > end_time) {
  	  	// Find last point before end time
  	  	for (point=pointset->end()-2, found=false;
			 point!=pointset->begin()-1 && !found; point--) {
		  if (point->DoubleAttribute(TimeTag) <= end_time) found = true;
		}
		if (!found) {
		  printf("Point set %s does not contain a point within the time bounds\n",
		         pointset->Name());
		  pointset->ErasePoints();
		  continue;
		}
		else {
		  pointset->erase(point+1, pointset->end());
		}
  	  }
  	  
      // Transform with  splines
      TransformFromFrameToWorld(pointset->LaserPointsReference(),
	                            start_time, end_time,
                                omega_spline, phi_spline, kappa_spline,
	                            X_spline, Y_spline, Z_spline);
  	  
      // Add reconstructed trajectory points
      first_interval = (int) ((pointset->begin()->DoubleAttribute(TimeTag) - start_time) /
                       interval_duration);
      last_interval = (int) (((pointset->end()-1)->DoubleAttribute(TimeTag) - start_time) /
                       interval_duration);
      time = ((double) first_interval + 0.5) * interval_duration;
      for (interval=first_interval; interval<=last_interval;
	       interval++, time+=interval_duration) {
        frame_position.X() = X_spline.Value(time);
        frame_position.Y() = Y_spline.Value(time);
        frame_position.Z() = Z_spline.Value(time);
        frame_position.DoubleAttribute(TimeTag) = time;
        frame_position.Attribute(ScanLineNumberTag) = interval;
        pointset->push_back(frame_position);
      }
                       
  	  // Swap with the transformed point set (only points are swapped, not the meta data)
  	  pointset->swap(transformed_pointset);
  	  
  	  // Store transformed data and delete the points
  	  transformed_pointset.Write(transformed_pointset.PointFile(), 0, false);
  	  transformed_pointset.ErasePoints();
  	  
  	  // Store the meta data
  	  transformed_strip.push_back(transformed_pointset);
  	}
  	
  	// Set the strip name
    name = (char *) malloc(strlen(strip->Name()) + strlen(appendix) + 1);
    if (!name) {printf("Error allocating name (2) in SaveTransformedBlock\n"); exit(0);}
    sprintf(name, "%s%s", strip->Name(), appendix);
    transformed_strip.SetName(name);
    transformed_strip.DeriveMetaDataFileName(directory);
    transformed_strip.DataOrganisation() = StripWise;
    free(name);
  	
  	// Store the strip meta data
  	transformed_block.push_back(transformed_strip);
  	transformed_strip.erase(transformed_strip.begin(), transformed_strip.end());
  }
  printf("\n");
  
  // Set the block name
  name = (char *) malloc(strlen(block.Name()) + strlen(appendix) + 1);
  if (!name) {printf("Error allocating name (3) in SaveTransformedBlock\n"); exit(0);}
  sprintf(name, "%s%s", block.Name(), appendix);
  transformed_block.SetName(name);
  transformed_block.DeriveMetaDataFileName(directory);
  free(name);
  
  // Write all meta data
  transformed_block.WriteMetaData(1, 1);

// old code for residuals and trajectory points
  // Add residuals
//  for (point=copied_points.begin(); point!=copied_points.end(); point++) {
//  	if (!point->HasAttribute(PlaneNumberTag)) continue;
//    plane = planes.begin() + point->Attribute(PlaneNumberTag);
//    point->FloatAttribute(ResidualTag) =
//      plane->Distance(point->Position3DRef());
//  }
}


