
/*
                    Copyright 2015 University of Twente
 
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
  Labels that are identical are removed. Labels of the first data set are
  kept if labels differ.

 Initial creation:
 Author : George Vosselman
 Date   : 05-06-2015

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

using namespace std;

#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <malloc.h>
#include <time.h>
#include "LaserPoints.h"

/*
--------------------------------------------------------------------------------
                         The main comparelabels function
--------------------------------------------------------------------------------
*/

void comparelabels(char *ref_file, char *class_file, char *error_ref_file,
                   char *error_class_file)
{
  LaserPoints refpoints, classpoints;
  LaserPoints::iterator refpoint, classpoint;
  std::vector<int>           refvalues, classvalues;
  std::vector<int>::iterator value;
  int                        label, refindex, classindex, *counts, num_labels,
                             *sums, sum, *sumse, sume, sumall, qual;
  bool                       use_label5=true;
  
  // Read the input data
  if (!refpoints.Read(ref_file, false)) {
  	printf("Error reading data from file %s\n", ref_file);
  	exit(0);
  }
  if (!classpoints.Read(class_file, false)) {
  	printf("Error reading data from file %s\n", class_file);
  	exit(0);
  }

  // Check if both datasets have the same number of points
  if (refpoints.size() != classpoints.size()) {
  	printf("Error: Datasets to be compared have different sizes (%d and %d)\n",
  	       refpoints.size(), classpoints.size());
  	exit(0);
  }
  
  // Check if the first point has the same coordinates in both datasets
  if (!(refpoints.begin()->vect() == classpoints.begin()->vect())) {
  	printf("Error: First points of both datasets have different coordinates\n");
  	printf("       Point sets to compare should have points in the same sequence.\n");
  }
  
  // Determine all used labels
  refpoints.AttributeValues(LabelTag, refvalues);
  classpoints.AttributeValues(LabelTag, classvalues);
  refvalues.insert(refvalues.end(), classvalues.begin(), classvalues.end());
  std::sort(refvalues.begin(), refvalues.end());
  refvalues.erase( std::unique( refvalues.begin(), refvalues.end() ), refvalues.end() );
  // Delete class label 5
  if (!use_label5) refvalues.erase(refvalues.end()-1, refvalues.end());
  
  // Produce error statistics
  num_labels = refvalues.size();
  counts = (int *) calloc(num_labels * num_labels, sizeof(int));
  for (refpoint=refpoints.begin(), classpoint=classpoints.begin();
       refpoint!=refpoints.end(); refpoint++, classpoint++) {
    if (refpoint->HasAttribute(LabelTag) && classpoint->HasAttribute(LabelTag)) {
      label = refpoint->Label();
      value = std::find(refvalues.begin(), refvalues.end(), label);
      refindex = std::distance(refvalues.begin(), value);
      label = classpoint->Label();
      if (label == 5 && !use_label5) continue;
      value = std::find(refvalues.begin(), refvalues.end(), label);
      classindex = std::distance(refvalues.begin(), value);
      counts[refindex * num_labels + classindex]++;
    }
  }
  
  printf("\nCounts ");
  for (refindex=0; refindex<num_labels; refindex++)
    printf("%7d", refvalues[refindex]);
  printf("  Total\n");
  sums = (int *) calloc(num_labels, sizeof(int));
  for (classindex=0; classindex<num_labels; classindex++) {
  	printf("%7d", refvalues[classindex]);
    for (refindex=0, sum=0; refindex<num_labels; refindex++) {
      printf("%7d", counts[refindex * num_labels + classindex]);
      sums[refindex] += counts[refindex * num_labels + classindex];
      sum += counts[refindex * num_labels + classindex];
    }
    printf("%7d\n", sum);
  }
  printf("  Total");
  for (refindex=0, sumall=0; refindex<num_labels; refindex++) {
    printf("%7d", sums[refindex]);
    sumall += sums[refindex];
  }
  printf("%7d\n", sumall);

  printf("\nPerc.  ");
  for (refindex=0; refindex<num_labels; refindex++) {
    printf("%7d", refvalues[refindex]);
  }
  printf("   Cor.\n");
  sumse = (int *) calloc(num_labels, sizeof(int));
  for (classindex=0, qual=0; classindex<num_labels; classindex++) {
  	printf("%7d", refvalues[classindex]);
    for (refindex=0, sum=0, sume=0; refindex<num_labels; refindex++) {
      printf("%7.1f", 100.0 * (double) counts[refindex * num_labels + classindex] / sumall);
      sum += counts[refindex * num_labels + classindex];
      if (refindex != classindex) {
        sumse[refindex] += counts[refindex * num_labels + classindex];
        sume += counts[refindex * num_labels + classindex];
      }
      else qual += counts[refindex * num_labels + classindex];
    }
    printf("%7.1f\n", 100.0 * (1.0 - (double) sume / sum));
  }
  printf("  Comp.");
  for (refindex=0; refindex<num_labels; refindex++) {
    printf("%7.1f", 100.0 * (1.0 - (double) sumse[refindex] / sums[refindex]));
  }
  printf("%7.1f\n", 100.0 * (double) qual / sumall);
 
  
  // Remove the labels of correctly classified points
  for (refpoint=refpoints.begin(), classpoint=classpoints.begin();
       refpoint!=refpoints.end(); refpoint++, classpoint++) {
    if (refpoint->Label() == classpoint->Label()) {
      refpoint->RemoveAttribute(LabelTag);
      classpoint->RemoveAttribute(LabelTag);
    }
  }
  
  // Store the output data
  if (error_ref_file) refpoints.Write(error_ref_file, 0, false);
  if (error_class_file) classpoints.Write(error_class_file, 0, false);
}
