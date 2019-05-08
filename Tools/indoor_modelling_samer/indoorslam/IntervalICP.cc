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

#include "LaserPoints.h"

extern "C" void svd_(int *, int *, int *, double *, double *, int *, double *,
                     int *, double *, int *, double *);


// ICP on the data of two intervals
bool IntervalICP(LaserPoints &original_points, int start_second_interval,
                 double &tx, double &ty, double &kappa)
{
  SegmentationParameters    par;
  TINEdges                  *edges;
  TINEdges::iterator        edgeset;
  TINEdgeSet::iterator      node;
  LaserPoints               points;
  LaserPoints::iterator     point, original_point, nb_point;
  int                       index, *index_nearest, num_pairs, n1=1, n2=2, ierr,
                            max_iteration=10, iteration;
  bool                      found, quitloop;
  Vector3D                  old_centroid, new_centroid, old_pos, new_pos, translation;
  double                    dx, dy, dkappa, covar[4], u[4], v[4], w[2], rv1[2],
                            cosa, sina, distsum;
  Rotation3D                rotation;

  // Make a local copy of the points
  points.insert(points.begin(), original_points.begin(), original_points.end());

  // Initialise parameters
  par.NeighbourhoodStorageModel() = 2; // knn
  par.NumberOfNeighbours() = 10;
  tx = ty = kappa = 0.0;
  iteration = 0;
  index_nearest = (int *) malloc(points.size() * sizeof(int));
  if (!index_nearest) {printf("Error allocating index_nearest\n"); exit(0);}
  
  do {
  	iteration++;
  	
    // Transform points of the second interval with approximated transformation
    if (iteration > 0) {
      rotation = Rotation3D(0.0, 0.0, kappa);
      translation = Vector3D(tx, ty, 0.0);
      for (point=points.begin()+start_second_interval,
	       original_point=original_points.begin()+start_second_interval;
		   point!=points.end(); point++, original_point++) {
	    point->vect() = rotation * original_point->vect() + translation;
	  }
    }
  
    // Derive KNN edges
    edges = points.DeriveEdges(par);
    
    // Look for nearest points from both intervals
    num_pairs = 0;
    distsum = 0.0;
    for (point=points.begin(), index=0, edgeset=edges->begin();
         point!=points.end(); point++, index++, edgeset++) {
         	
      // Check if the point is valid
      if (point->Label() < 0) {
      	index_nearest[index] = -1;
        continue;
      }

      // Loop over the nearest points for this point
      for (node=edgeset->begin(), found=false, quitloop=false;
	       node!=edgeset->end() && !found && !quitloop; node++) {

        // Check whether the two points are from different intervals
        if (index < start_second_interval) {
      	  if (node->Number() < start_second_interval) continue;
        }
        else {
      	  if (node->Number() >= start_second_interval) continue;
        }
      
        // Check if the point is valid
        nb_point = points.begin() + node->Number();
        if (nb_point->Label() < 0) continue;
        
        // Only use points within 0.1 m
        if (point->Distance(nb_point->Position3DRef()) > 0.1) {
          quitloop = true; // Next points will only be further away
          continue;
        }
        
        // Store the index of the nearest point
        found = true;
        index_nearest[index] = node->Number();
        num_pairs++;
      
	    // Use this pair to compute centroids
	    if (index < start_second_interval) {
	  	  old_centroid += point->vect();
	  	  new_centroid += points[node->Number()].vect();
	    }
	    else {
	  	  old_centroid += points[node->Number()].vect();
	  	  new_centroid += point->vect();
	    }
	  
	    distsum += point->Distance(points[node->Number()].Position3DRef());
      }
      if (!found) index_nearest[index] = -1;
    }
  
    // Calculate centroids
    if (num_pairs > 1) {
      old_centroid /= (double) num_pairs;
      new_centroid /= (double) num_pairs;
    }
    else {
  	  printf("Insufficient points for ICP\n");
  	  return false; // Insufficient points to estimate translation and rotation
    }
  
    // Construct covariance matrix
    for (index=0; index<4; index++) covar[index] = 0.0;
    for (point=points.begin(), index=0;
         point!=points.end(); point++, index++) {
      if (index_nearest[index] == -1) continue;
      if (index < start_second_interval) {
        old_pos = point->vect() - old_centroid;
        new_pos = points[index_nearest[index]].vect() - new_centroid;
      }
      else {
        old_pos = points[index_nearest[index]].vect() - old_centroid;
        new_pos = point->vect() - new_centroid;
      }
      covar[0] += old_pos.X() * new_pos.X();
      covar[1] += old_pos.X() * new_pos.Y();
      covar[2] += old_pos.Y() * new_pos.X();
      covar[3] += old_pos.Y() * new_pos.Y();
    }
  
    // Determine rotation
    svd_(&n2, &n2, &n2, covar, w, &n1, u, &n1, v, &ierr, rv1);
    cosa = v[0] * u[0] + v[1] * u[1];  // r = v * u^T
    sina = v[0] * u[2] + v[1] * u[3];
    dkappa = atan2(-sina, cosa);
  
    // Determine the offset (-R old_centroid + new_centroid
    dx = new_centroid.X() - (cosa * old_centroid.X() + sina * old_centroid.Y());
    dy = new_centroid.Y() - (-sina * old_centroid.X() + cosa * old_centroid.Y());
  
    // Update parametrs (incorrect!)
    tx -= dx;
    ty -= dy;
    kappa -= dkappa;
//    printf("dx %.3f dy %.3f dkappa %.3f average distance %.3f, %d pairs\n",
//	       tx, ty, kappa * 45.0 / atan(1.0), distsum / (double) num_pairs, num_pairs);
    
    // Erase edges
    points.EraseNeighbourhoodEdges();
  } while (distsum / (double) num_pairs > 0.001 && iteration < max_iteration);
  
  free(index_nearest);
  points.ErasePoints();
  
  return true;
}

