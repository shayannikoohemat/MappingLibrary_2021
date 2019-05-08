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

#include "Planes.h"
#include "LaserPoints.h"
#include "LaserScanLines.h"
#include "BSplineFit.h"

void SaveWalls(int interval, Planes &planes, vector<DataBounds3D> &bounds_of_planes,
               char *directory)
{
  vector<DataBounds3D>::iterator bounds;
  Planes::iterator               plane;
  ObjectPoints                   points;
  ObjectPoint                    point;
  LineTopologies                 tops;
  int                            i, point_number, plane_number;
  Plane                          horizontal_plane;
  Line3D                         line;
  double                         scalar, min_scalar, max_scalar;
  char                           *pcm_file_name, *point_file_name,
                                 *top_file_name, *laser_file_name,
                                 name[30];
  FILE                           *pcm_file;
  
  // Create a horizontal plane
  horizontal_plane = Plane(Vector3D(0.0, 0.0, 0.0), Vector3D(0.0, 0.0, 1.0));

  // Loop over all planes
  point_number = 0;
  for (plane_number=0, plane=planes.begin(), bounds=bounds_of_planes.begin();
       plane != planes.end(); plane_number++, plane++, bounds++) {

    // Skip obsolete planes that have been merged with other planes
    if (plane->Number() != plane_number) continue;
    
    // Skip floor and ceiling
    if (fabs(plane->Normal().Z()) > 0.99) continue;

    // Construct a horizontal line through the plane
    Intersect2Planes(*plane, horizontal_plane, line);

    // Determine the scalar range of the bounding box corners
    min_scalar = 1e10; max_scalar = -1e10;
    scalar = line.Scalar(Position3D(bounds->Minimum().X(),
	                                bounds->Minimum().Y(),
									bounds->Minimum().Z()));
	if (scalar < min_scalar) min_scalar = scalar;
	if (scalar > max_scalar) max_scalar = scalar;
    scalar = line.Scalar(Position3D(bounds->Minimum().X(),
	                                bounds->Maximum().Y(),
									bounds->Minimum().Z()));
	if (scalar < min_scalar) min_scalar = scalar;
	if (scalar > max_scalar) max_scalar = scalar;
    scalar = line.Scalar(Position3D(bounds->Maximum().X(),
	                                bounds->Minimum().Y(),
									bounds->Minimum().Z()));
	if (scalar < min_scalar) min_scalar = scalar;
	if (scalar > max_scalar) max_scalar = scalar;
    scalar = line.Scalar(Position3D(bounds->Maximum().X(),
	                                bounds->Maximum().Y(),
									bounds->Minimum().Z()));
	if (scalar < min_scalar) min_scalar = scalar;
	if (scalar > max_scalar) max_scalar = scalar;

	// Store topology
	tops.push_back(LineTopology(plane_number, point_number, point_number+1,
	                            point_number+2, point_number+3));
	(tops.end()-1)->Label() = 1; // PCM MapLabel
	(tops.end()-1)->Attribute(6) = plane_number; // PCM Building number

	// Construct and store wall corners
	point.Number() = point_number;
	point.vect()   = line.Position(min_scalar).vect();
	point.Z()      = bounds->Minimum().Z();
	points.push_back(point);
	point.Number() = point_number+1;
	point.vect() = line.Position(max_scalar).vect();
	point.Z()      = bounds->Minimum().Z();
	points.push_back(point);
	point.Number() = point_number+2;
	point.Z()      = bounds->Maximum().Z();
	points.push_back(point);
	point.Number() = point_number+3;
	point.vect()   = line.Position(min_scalar).vect();
	point.Z()      = bounds->Maximum().Z();
	points.push_back(point);
	point_number += 4;
  }
  
  // Save points, topology and PCM meta data
  sprintf(name, "approx%5d", interval);
  for (i=6; i<11; i++) if (name[i] == ' ') name[i] = '0';
  laser_file_name = ComposeFileName(directory, name, ".laser");
  point_file_name = ComposeFileName(directory, name, ".objpts");
  top_file_name = ComposeFileName(directory, name, ".top");
  pcm_file_name = ComposeFileName(directory, name, ".pcm");
  points.Write(point_file_name);
  tops.Write(top_file_name, false);
  pcm_file = fopen(pcm_file_name, "w");
  fprintf(pcm_file, "pointcloudmapping:\n");
  fprintf(pcm_file, "  laser_points: \"%s\"\n", laser_file_name);
  fprintf(pcm_file, "  map_points: \"%s\"\n", point_file_name);
  fprintf(pcm_file, "  map_topology: \"%s\"\n", top_file_name);
  fprintf(pcm_file, "endpointcloudmapping:\n");
  fclose(pcm_file);
  free(laser_file_name); free(point_file_name);
  free(top_file_name); free(pcm_file_name);
}


ObjectPoints face_points;
LineTopologies face_tops;         

void SaveInitialPlanes(LaserPoints &sensordata, LaserScanLines::const_iterator segment,
                       Line3D &line, Plane &plane,
				       const Rotation3D &rotation, const Vector3D &translation)
{
  Position3D world_pos1, world_pos2;
  int numpts;

  // Get transformed points and project them onto the line
  world_pos1.vect() = rotation * segment->begin(sensordata)->vect() + translation;
  world_pos1 = line.Project(world_pos1);
  world_pos2.vect() = rotation * segment->end(sensordata)->vect() + translation;
  world_pos2 = line.Project(world_pos2);
  
  // Add the face topology
  numpts = face_points.size();
  face_tops.push_back(LineTopology(plane.Number(), numpts, numpts+1, numpts+2,
                                   numpts+3));
                                   
  // Add the four corners for horizontal planes
  if (fabs(plane.Normal().Z() > 0.99)) {
    face_points.push_back(ObjectPoint(Vector3D(world_pos1.X(), world_pos1.Y(), 
	                                           (world_pos1.Z()+world_pos2.Z())/2.0),
                                      PointNumber(numpts), Covariance3D()));
    face_points.push_back(ObjectPoint(Vector3D(world_pos1.X(), world_pos2.Y(), 
	                                           (world_pos1.Z()+world_pos2.Z())/2.0),
                                      PointNumber(numpts+1), Covariance3D()));
    face_points.push_back(ObjectPoint(Vector3D(world_pos2.X(), world_pos2.Y(), 
	                                           (world_pos1.Z()+world_pos2.Z())/2.0),
                                      PointNumber(numpts+2), Covariance3D()));
    face_points.push_back(ObjectPoint(Vector3D(world_pos2.X(), world_pos1.Y(), 
	                                           (world_pos1.Z()+world_pos2.Z())/2.0),
                                      PointNumber(numpts+3), Covariance3D()));
  }
  // Add the four corners for vertical planes
  else {
    face_points.push_back(ObjectPoint(Vector3D(world_pos1.X(), world_pos1.Y(), -1.5),
                                      PointNumber(numpts), Covariance3D()));
    face_points.push_back(ObjectPoint(Vector3D(world_pos1.X(), world_pos1.Y(), 1.5),
                                      PointNumber(numpts+1), Covariance3D()));
    face_points.push_back(ObjectPoint(Vector3D(world_pos2.X(), world_pos2.Y(), 1.5),
                                      PointNumber(numpts+2), Covariance3D()));
    face_points.push_back(ObjectPoint(Vector3D(world_pos2.X(), world_pos2.Y(), -1.5),
                                      PointNumber(numpts+3), Covariance3D()));
  }
  
  // Save the data
  face_points.Write("wall_faces.objpts");
  face_tops.Write("wall_faces.top", false);
}

void SaveInitialPlanes(const LaserPoints &sensordata, 
                       LaserScanLines::const_iterator segment,
                       Line3D &line, Plane &plane,
				       BSplineFit &omega_spline, BSplineFit &phi_spline,
                       BSplineFit &kappa_spline, BSplineFit &X_spline,
				       BSplineFit &Y_spline, BSplineFit &Z_spline)
{
  double time;
  Rotation3D rotation;
  Vector3D translation;
  Position3D world_pos1, world_pos2;
  int numpts;

  // Get transformed points and project them onto the line
  time = segment->begin(sensordata)->DoubleAttribute(TimeTag);
  rotation = Rotation3D(omega_spline.Value(time),
			            phi_spline.Value(time), kappa_spline.Value(time));
  translation = Vector3D(X_spline.Value(time), Y_spline.Value(time),
		                 Z_spline.Value(time));
  world_pos1.vect() = rotation * segment->begin(sensordata)->vect() + translation;
  world_pos1 = line.Project(world_pos1);
  
  
  time = segment->end(sensordata)->DoubleAttribute(TimeTag);
  rotation = Rotation3D(omega_spline.Value(time),
			            phi_spline.Value(time), kappa_spline.Value(time));
  translation = Vector3D(X_spline.Value(time), Y_spline.Value(time),
		                 Z_spline.Value(time));
  world_pos2.vect() = rotation * segment->end(sensordata)->vect() + translation;
  world_pos2 = line.Project(world_pos2);
  
  // Add the face topology
  numpts = face_points.size();
  face_tops.push_back(LineTopology(plane.Number(), numpts, numpts+1, numpts+2,
                                   numpts+3));
                                   
  // Add the four corners for horizontal planes
  if (fabs(plane.Normal().Z() > 0.99)) {
    face_points.push_back(ObjectPoint(Vector3D(world_pos1.X(), world_pos1.Y(), 
	                                           (world_pos1.Z()+world_pos2.Z())/2.0),
                                      PointNumber(numpts), Covariance3D()));
    face_points.push_back(ObjectPoint(Vector3D(world_pos1.X(), world_pos2.Y(), 
	                                           (world_pos1.Z()+world_pos2.Z())/2.0),
                                      PointNumber(numpts+1), Covariance3D()));
    face_points.push_back(ObjectPoint(Vector3D(world_pos2.X(), world_pos2.Y(), 
	                                           (world_pos1.Z()+world_pos2.Z())/2.0),
                                      PointNumber(numpts+2), Covariance3D()));
    face_points.push_back(ObjectPoint(Vector3D(world_pos2.X(), world_pos1.Y(), 
	                                           (world_pos1.Z()+world_pos2.Z())/2.0),
                                      PointNumber(numpts+3), Covariance3D()));
  }
  // Add the four corners for vertical planes
  else {
    face_points.push_back(ObjectPoint(Vector3D(world_pos1.X(), world_pos1.Y(), -1.5),
                                      PointNumber(numpts), Covariance3D()));
    face_points.push_back(ObjectPoint(Vector3D(world_pos1.X(), world_pos1.Y(), 1.5),
                                      PointNumber(numpts+1), Covariance3D()));
    face_points.push_back(ObjectPoint(Vector3D(world_pos2.X(), world_pos2.Y(), 1.5),
                                      PointNumber(numpts+2), Covariance3D()));
    face_points.push_back(ObjectPoint(Vector3D(world_pos2.X(), world_pos2.Y(), -1.5),
                                      PointNumber(numpts+3), Covariance3D()));
  }

  // Save the data
  face_points.Write("wall_faces.objpts");
  face_tops.Write("wall_faces.top", false);
}


