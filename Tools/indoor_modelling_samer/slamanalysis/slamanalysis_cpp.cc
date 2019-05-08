
/*
                   Copyright 2014 University of Twente
 
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
 Date   : 23-03-2014

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "DataBounds3D.h"
#include "ObjectPoints.h"
#include "LineTopologies.h"
#include "Planes.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                      The main indoorsimulation function
--------------------------------------------------------------------------------
*/

void slamanalysis_cpp(char *root_name, int num_files, int first_number)
{
  vector<Planes>                   all_planes, all_horizontal_planes;
  vector<ObjectPoints>             all_corners;
  vector<LineTopologies>           all_walls;
  vector<Planes>::iterator         planes, horizontal_planes;
  vector<ObjectPoints>::iterator   corners;
  vector<LineTopologies>::iterator walls;
  int                              experiment, success, root_length, i,
                                   index, best_index, num;
  double                           dist, min_dist, pi=4.0*atan(1.0),
                                   sum, sqsum, angle;
  char                             experiment_name[100], *file_name;
  bool                             found;
  PointNumberList::iterator        node0, node1;
  ObjectPoints::iterator           point0, point1;
  Planes::iterator                 plane, plane2, vertical_plane, horizontal_plane;
  LineTopologies::iterator         wall;
  Position3D                       new_wall_centre, real_corner, real_corner2;
  Positions3D                      wall_centres, ideal_centres, ideal_corners; 
  Positions3D::iterator            wall_centre, ideal_corner;
  Plane                            stack_plane;
  LineTopology                     stack_wall;
  vector<vector<int> >             wall_groups;
  vector<vector<int> >::iterator   wall_group;
  vector<int>::iterator            wall_index, wall_index2;
  vector<int>                      inside_walls, outside_walls;

  root_length = strlen(root_name);
  for (experiment=first_number; experiment<num_files+first_number; experiment++) {
  	
	// Compose experiment name
  	sprintf(experiment_name, "%s%5d", root_name, experiment);
  	for (i=0; i<4; i++)
	  if (experiment_name[i+root_length] == ' ')
	    experiment_name[i+root_length] = '0';
	
	// Read planes
  	file_name = ComposeFileName("./", experiment_name, ".planes");
  	all_planes.push_back(Planes(file_name, &success));
  	free(file_name);
  	
	// Read corners
  	file_name = ComposeFileName("./", experiment_name, ".objpts");
  	all_corners.push_back(ObjectPoints(file_name));
  	free(file_name);
  	
	// Read wall topologies
  	file_name = ComposeFileName("./", experiment_name, ".top");
  	all_walls.push_back(LineTopologies(file_name, &success));
  	free(file_name);
  	
  	if (experiment == first_number)
  	  printf("%d planes, %d corners, %d walls in experiment %2d\n",
	         (all_planes.end()-1)->size(), (all_corners.end()-1)->size(),
	         (all_walls.end()-1)->size(), experiment);
  }
  
  // Separate the two horizontal planes from each set of planes
  for (experiment=0, planes=all_planes.begin(); experiment<num_files;
       experiment++, planes++) {
    all_horizontal_planes.push_back(Planes());
    horizontal_planes = all_horizontal_planes.end() - 1;
    for (plane=planes->begin(), vertical_plane=plane;
	     plane!=planes->end(); plane++) {
      if (fabs(plane->Normal().Z() < 0.99)) {
      	*vertical_plane = *plane;
      	vertical_plane++;
      }
      else horizontal_planes->push_back(*plane);
    }
    if (vertical_plane != planes->end()) 
	  planes->erase(vertical_plane, planes->end());
	if (experiment == 0)
      printf("%d vertical and %d horizontal planes in experiment %d\n",
	         planes->size(), horizontal_planes->size(), experiment+first_number);
  }

  // Remove the walls with a small height as well as those with less than
  // 10000 points.
  for (experiment=0, corners=all_corners.begin(), planes=all_planes.begin(),
       walls=all_walls.begin(); experiment<num_files;
       experiment++, corners++, planes++, walls++) {
    for (plane=planes->begin(), wall=walls->begin(); wall!=walls->end();
	     plane++, wall++) {
	  node0 = wall->begin();
	  node1 = node0 + 2;
	  point0 = corners->PointIterator(*node0);
	  point1 = corners->PointIterator(*node1);
	  if (fabs(point0->Z() - point1->Z()) < 1.0 ||
	      plane->Attribute(PT_NumberOfPoints) < 10000) {
	  	planes->erase(plane); plane--;
	  	walls->erase(wall); wall--;
	  }
	}
	if (experiment == 0)
	  printf("%2d walls left in experiment %2d\n", walls->size(), experiment+first_number);
	if (walls->size() != 26) {
	  printf("Error: %d walls instead of expected 26 walls\n", walls->size());
	  exit(0);
	}
  }

  // Create ideal centres of inside and outside walls
  ideal_centres.push_back(Position3D(0.0, 1.5, 0.0)); // Inside
  ideal_centres.push_back(Position3D(7.0, 3.5, 0.0));
  ideal_centres.push_back(Position3D(0.0, 5.5, 0.0));
  ideal_centres.push_back(Position3D(-7.0, 3.5, 0.0));
  ideal_centres.push_back(Position3D(0.0, -1.5, 0.0)); // Outside
  ideal_centres.push_back(Position3D(10.0, 3.5, 0.0));
  ideal_centres.push_back(Position3D(0.0, 8.5, 0.0));
  ideal_centres.push_back(Position3D(-10.0, 3.5, 0.0));
  
  // Determine centres for the walls of the first experiment
  planes = all_planes.begin();
  for (plane=planes->begin(); plane!=planes->end(); plane++) {
  	// Determine the nearest ideal wall centre
    min_dist = 100.0;
    for (wall_centre=ideal_centres.begin(), index=0;
	     wall_centre!=ideal_centres.end(); wall_centre++, index++) {
	  dist = fabs(plane->Distance(*wall_centre));
	  if (dist < min_dist) {
	  	min_dist = dist;
	  	best_index = index;
	  }
	}
	wall_centre = ideal_centres.begin() + best_index;
	// Project the nearest ideal centre onto the wall plane
	new_wall_centre = plane->Project(*wall_centre);
	wall_centres.push_back(new_wall_centre);
  }

  // Put the walls in the same sequence in all experiments
  // Apart from the first three walls, only swapping of two subsequent
  // walls may be required.
  for (experiment=1, planes=all_planes.begin()+1,
       walls=all_walls.begin()+1; experiment<num_files;
       experiment++, planes++, walls++) {
    // Check the first wall
    wall = walls->begin();
    plane = planes->begin();
    for (i=0, min_dist=1e10; i<3; i++) {
      dist = fabs(plane->Distance(wall_centres[i]));
	  if (dist < min_dist) {
	  	best_index = i;
	  	min_dist = dist;
	  }	
    }
    if (best_index != 0) {
      stack_wall = *wall;
      *wall = *(walls->begin() + best_index);
      *(walls->begin() + best_index) = stack_wall;
      stack_plane = *plane;
      *plane = *(planes->begin() + best_index);
      *(planes->begin() + best_index) = stack_plane;
    }
    // Check if any further pair of planes should be swapped
    for (plane=planes->begin()+1, wall=walls->begin()+1, i=1;
	     plane!=planes->end()-1; plane++, wall++, i++) {
      if (fabs(plane->Distance(wall_centres[i])) >
          fabs(plane->Distance(wall_centres[i+1]))) {
        stack_wall = *(walls->begin() + i);
        *(walls->begin() + i) = *(walls->begin() + i + 1);
        *(walls->begin() + i + 1) = stack_wall;
        stack_plane = *(planes->begin() + i);
        *(planes->begin() + i) = *(planes->begin() + i + 1);
        *(planes->begin() + i + 1) = stack_plane;
      }
    }
  }
  
  // Check on all sequences
  for (experiment=1, planes=all_planes.begin()+1; experiment<num_files;
       experiment++, planes++) {
    for (plane=planes->begin(), wall_centre=wall_centres.begin();
	     plane!=planes->end(); plane++, wall_centre++) {
	  if (fabs(plane->Distance(*wall_centre)) > 0.2) {
	    printf("Large distance %5.3f in plane %d experiment %d\n",
		       fabs(plane->Distance(*wall_centre)), plane->Number(), experiment);
		exit(0);
	  }
	}
  }
  
  // Determine inside and outside walls
  for (plane=all_planes.begin()->begin(), i=0;
       plane!=all_planes.begin()->end(); plane++, i++) {
    min_dist = 100.0;
    for (wall_centre=ideal_centres.begin(), index=0;
	     wall_centre!=ideal_centres.end(); wall_centre++, index++) {
	  dist = fabs(plane->Distance(*wall_centre));
	  if (dist < min_dist) {
	  	min_dist = dist;
	  	best_index = index;
	  }
	}
	if (best_index < 4) inside_walls.push_back(i);
    else outside_walls.push_back(i);
  }
  wall_groups.push_back(inside_walls);
  wall_groups.push_back(outside_walls);
  
  // Calculate standard deviations of right angles of two consecutive walls
  printf("Noise in angles of two consecutive walls\n");
  for (wall_group=wall_groups.begin(); wall_group!=wall_groups.end();
       wall_group++) {
    // Loop over all angles
    for (wall_index=wall_group->begin(), wall_index2=wall_index+1;
	     wall_index2!=wall_group->end(); wall_index++, wall_index2++) {
	  sum = sqsum = 0.0;
	  // Loop over all experiments
      for (experiment=0, planes=all_planes.begin(); experiment<num_files;
           experiment++, planes++) {
        angle = Angle((*planes)[*wall_index].Normal(),
		              (*planes)[*wall_index2].Normal());
		if (angle > 0.0) angle -= pi / 2.0;
		sum += angle;
		sqsum += angle * angle;
      }
      printf("Planes %2d and %2d, angle av %7.4f  stdev %6.4f  sizes %6d %6d\n",
	         *wall_index, *wall_index2, sum/num_files * 180.0 / pi,
			 sqrt(sqsum/num_files) * 180.0 / pi,
			 (all_planes.begin()->begin()+*wall_index)->Attribute(PT_NumberOfPoints),
			 (all_planes.begin()->begin()+*wall_index2)->Attribute(PT_NumberOfPoints));
	}
  }

  // Calculate standard deviations of angles between first and later walls
  printf("Noise in angles of first wall with later walls\n");
  for (wall_group=wall_groups.begin(); wall_group!=wall_groups.end();
       wall_group++) {
    // Loop over all angles
    for (wall_index=wall_group->begin(), wall_index2=wall_index+1;
	     wall_index2!=wall_group->end(); wall_index2++) {
	  sum = sqsum = 0.0;
	  // Loop over all experiments
      for (experiment=0, planes=all_planes.begin(); experiment<num_files;
           experiment++, planes++) {
        angle = Angle((*planes)[*wall_index].Normal(),
		              (*planes)[*wall_index2].Normal());
		while (angle > pi / 4.0) angle -= pi / 2.0;
		sum += angle;
		sqsum += angle * angle;
      }
      printf("Planes %2d and %2d, angle av %7.4f  stdev %6.4f  sizes %6d %6d\n",
	         *wall_index, *wall_index2, sum/num_files * 180.0 / pi,
			 sqrt(sqsum/num_files) * 180.0 / pi,
			 (all_planes.begin()->begin()+*wall_index)->Attribute(PT_NumberOfPoints),
			 (all_planes.begin()->begin()+*wall_index2)->Attribute(PT_NumberOfPoints));
	}
  }
  
  // Determine which walls belong together
  planes = all_planes.begin();
  wall_groups.erase(wall_groups.begin(), wall_groups.end());
  for (i=0; i<9; i++) wall_groups.push_back(vector<int>());
  for (wall_centre=wall_centres.begin(), i=0; wall_centre!=wall_centres.end();
       wall_centre++, i++) {
    for (plane=planes->begin(), index=0, found=false; 
	     plane!=planes->end() && !found; plane++, index++) {
      if (fabs(plane->Distance(*wall_centre)) < 0.2) {
      	wall_groups[index].push_back(i);
      	found = true;
      }
    }
  }
  // Wall 7 or 8 equals wall 2, hence those walls are already in group 2
  if (wall_groups[8].size())
    wall_groups[7] = wall_groups[8]; 
  wall_groups.erase(wall_groups.end()-1);
  
  printf("Wall groups\n");  
  for (wall_group=wall_groups.begin(); wall_group!=wall_groups.end();
       wall_group++) {
    printf("Group:");
    for (wall_index=wall_group->begin(); wall_index!=wall_group->end();
	     wall_index++)
	  printf(" %d", *wall_index);
	wall_index = wall_group->begin();
	printf("   centre X %6.2f  Y %6.2f\n", wall_centres[*wall_index].X(),
	       wall_centres[*wall_index].Y());
  }
 
  // Calculate standard deviations of distances between wall centres and first
  // plane of the group
  printf("Noise in distance of centre points to reference plane\n");
  for (wall_group=wall_groups.begin(); wall_group!=wall_groups.end();
       wall_group++) {
    // Loop over all pairs of walls
    for (wall_index=wall_group->begin(), wall_index2=wall_index+1;
	     wall_index2!=wall_group->end(); wall_index2++) {
	  // Get the wall centre of the second wall of the first experiment
	  wall_centre = wall_centres.begin() + *wall_index2;
	  sum = sqsum = 0.0;
	  // Loop over all experiments
      for (experiment=0, walls=all_walls.begin(), corners=all_corners.begin(),
	       planes=all_planes.begin();
	       experiment<num_files; experiment++, walls++, corners++, planes++) {
	       	
	    // Project the wall centre of the first experiment onto the second 
		// plane of this experiment
	    plane2 = planes->begin() + *wall_index2;
	    new_wall_centre = plane2->Project(*wall_centre);
	    
  	    // Calculate distance to the first plane and update statistics
  	    plane = planes->begin() + *wall_index; // Reference plane
        dist = plane->Distance(new_wall_centre);
		sum += dist;
		sqsum += dist * dist;
      }
      printf("Planes %2d and %2d, centre dist av %7.4f  stdev %6.4f  sizes %6d %6d\n",
	         *wall_index, *wall_index2, sum/num_files, sqrt(sqsum/num_files),
			 (all_planes.begin()->begin()+*wall_index)->Attribute(PT_NumberOfPoints),
			 (all_planes.begin()->begin()+*wall_index2)->Attribute(PT_NumberOfPoints));
	}
  }
  
  // Set ideal corners
  for (double x=-10; x<11; x+=20) { // Outside corners
  	for (double y=-1.5; y<9; y+=10) {
  	  for (double z=-1.5; z<1.5; z+=2.5) {
  	  	ideal_corners.push_back(Position3D(x, y, z));
  	  }
  	}
  }
  for (double x=-7; x<11; x+=14) { // Inside corners
  	for (double y=1.5; y<6; y+=4) {
  	  for (double z=-1.5; z<1.5; z+=2.5) {
  	  	ideal_corners.push_back(Position3D(x, y, z));
  	  }
  	}
  }

  // Calculate standard deviations of distances between wall corners and first
  // plane of the group
  printf("Noise in distance of corner points to reference plane\n");
  for (wall_group=wall_groups.begin(); wall_group!=wall_groups.end();
       wall_group++) {
    // Loop over all pairs of walls
    for (wall_index=wall_group->begin(), wall_index2=wall_index+1;
	     wall_index2!=wall_group->end(); wall_index2++) {
	  sum = sqsum = 0.0;
	  // Loop over all experiments
      for (experiment=0, walls=all_walls.begin(), corners=all_corners.begin(),
	       planes=all_planes.begin();
	       experiment<num_files; experiment++, walls++, corners++, planes++) {
  	    plane = planes->begin() + *wall_index; // Reference plane
  	    plane2 = planes->begin() + *wall_index2; // Later measurement of the same plane
  	    // Loop over all ideal corners
  	    for (ideal_corner=ideal_corners.begin();
		     ideal_corner!=ideal_corners.end(); ideal_corner++) {
		  
		  // Only use the corners of a wall, i.e. those close to the wall plane
		  if (fabs(plane->Distance(*ideal_corner)) > 0.5) continue;
		  
		  // Project the ideal corner on the measured plane
		  real_corner = plane2->Project(*ideal_corner);
		  
		  // Calculate distance and update statistics
          dist = plane->Distance(real_corner);
	  	  sum += dist;
		  sqsum += dist * dist;
		}
      }
      printf("Planes %2d and %2d, corner dist av %7.4f  stdev %6.4f  sizes %6d %6d\n",
	         *wall_index, *wall_index2, 0.25*sum/num_files, sqrt(0.25*sqsum/num_files),
			 (all_planes.begin()->begin()+*wall_index)->Attribute(PT_NumberOfPoints),
			 (all_planes.begin()->begin()+*wall_index2)->Attribute(PT_NumberOfPoints));
	}
  }
  
  // Calculate standard deviation of the wall height at the outer corners
  sum = sqsum = 0.0;
  for (ideal_corner=ideal_corners.begin(); // Loop over first four corners
       ideal_corner!=ideal_corners.begin()+4; ideal_corner++) {
    for (horizontal_planes=all_horizontal_planes.begin();
	     horizontal_planes!=all_horizontal_planes.end(); horizontal_planes++) {
	  real_corner  = horizontal_planes->begin()->Project(*ideal_corner);
	  real_corner2 = (horizontal_planes->begin()+1)->Project(*ideal_corner);
	  dist = fabs(real_corner.Z() - real_corner2.Z()) - 2.5;
	  sum += dist;
	  sqsum += dist * dist;
	}
  }
  printf("\n");
  num = 4 * all_horizontal_planes.size();
  printf("Wall height in outer corners, av %7.4f  stdev %7.5f\n",
         sum/num, sqrt((sqsum-sum*sum/num)/num));
         
}
