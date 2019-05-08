
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

 Initial creation:
 Author : George Vosselman
 Date   : 06-09-2006

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
#include "LaserPoints.h"
#include "KNNFinder.h"

/*
--------------------------------------------------------------------------------
                         The main neighbours function
--------------------------------------------------------------------------------
*/
bool neighbours_cpp(char *infile, char *outfile,
                   int use_tin_nbh, int use_knn_nbh,
                   int knn, int use_dmax, bool planimetric, double dmax)
{
  LaserPoints        points;
  TINEdges           *edges;
  TINEdges::iterator nbh;
  FILE               *outfd;
  int                i, dimension;

  // Read the laser data
  if (!points.Read(infile)) {
    printf("Error reading input file %d\n", infile);
    return false;
  }
  dimension = 3;
  if (planimetric) dimension = 2;
  
  // Derive the neighbourhood
  edges = new TINEdges();
  if (use_tin_nbh) {
    points.DeriveTIN();
    edges->Derive(points.TINReference());
  }
  else if (use_knn_nbh) {
    KNNFinder <LaserPoint> finder(points, dimension);
    vector<int>            indices;
    TINEdgeSet             edgeset;
    for (i=0; i<(int) points.size(); i++) {
      indices = finder.FindIndices(points[i], knn+1);
      edgeset.Erase();
      for (int j=0; j<knn+1; j++)
        if (indices[j] != i) edgeset.push_back(PointNumber(indices[j]));
      edges->push_back(edgeset);
    }
    finder.FreeResources();
  }
  else {
    printf("No neighbourhood definition specified\n");
    return false;
  }
  
  if (use_dmax)
    points.RemoveLongEdges(edges->TINEdgesRef(), dmax, planimetric);

  outfd = fopen(outfile, "w");
  if (!outfd) {
    printf("Error opening output file %s\n", outfile);
    return false;
  }
  for (i=0, nbh=edges->begin(); nbh<edges->end(); i++, nbh++) {
    fprintf(outfd, "%d %d\n", i, nbh->size());
    for (TINEdgeSet::iterator node=nbh->begin(); node!=nbh->end(); node++)
      fprintf(outfd, "%d ", node->Number());
    fprintf(outfd, "\n");
  }
  fclose(outfd);
  return true;
}
