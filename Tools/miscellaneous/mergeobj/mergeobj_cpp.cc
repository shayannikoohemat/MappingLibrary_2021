
/*
    Copyright 2010 University of Twente and Delft University of Technology
 
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
 Merge two datasets with object points and topology

 Initial creation:
 Author : George Vosselman
 Date   : 08-05-2001

*/
/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include "LineTopologies.h"
#include "ObjectPoints.h"

void mergeobj_cpp(char *ip1_file, char *it1_file,
                  char *ip2_file, char *it2_file,
                  char *op_file, char *ot_file, int label)
{
  LineTopologies   top1, top2;
  ObjectPoints     points1, points2;
  LineTopologies::iterator line;
  ObjectPoints::iterator   point;

/* Read all files */

  if (!points1.Read(ip1_file)) {
    fprintf(stderr, "Error reading object points from file %s.\n", ip1_file);
    exit(0);
  }
  if (!top1.Read(it1_file)) {
    fprintf(stderr, "Error reading topology from file %s.\n", it1_file);
    exit(0);
  }
  if (!points2.Read(ip2_file)) {
    fprintf(stderr, "Error reading object points from file %s.\n", ip2_file);
    exit(0);
  }
  if (!top2.Read(it2_file)) {
    fprintf(stderr, "Error reading topology from file %s.\n", it2_file);
    exit(0);
  }

/* Renumber all points and topology of the first data set */

  top1.ReNumber(points1, 0, 0);

/* Renumber all points and topology of the second data set. Use the number
 * of points of the first data set as first point number in the second one.
 */

  top2.ReNumber(points2, points1.size(), top1.size());

// Set the labels to 1 or 2, so the origin can be identified in the merged set

  if (label) {
    for (line=top1.begin(); line!=top1.end(); line++) line->Label() = 1;
    for (line=top2.begin(); line!=top2.end(); line++) line->Label() = 2;
  }

/* Merge the two data sets */

  if (points2.size()) 
    for (point=points2.begin(); point!=points2.end(); point++)
      points1.push_back(*point);
//  points1.insert(points1.end(), points2.begin(), points2.end()); caused problems
  if (top2.size())
    for (line=top2.begin(); line!=top2.end(); line++)
      top1.push_back(*line);
//  top1.insert(top1.end(), top2.begin(), top2.end()); caused problems

/* Write the merged data sets */

  points1.Write(op_file);
  top1.Write(ot_file);
}
