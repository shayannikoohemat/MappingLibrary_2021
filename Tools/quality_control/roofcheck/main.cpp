
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


#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include "InlineArguments.h"
#include "LaserBlock.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: roofcheck -i <block meta file>\n");
  printf("         [-op <output points of all ridge lines (*.objpts)>]\n");
  printf("         [-ot <output topology of all ridge lines (*.top)>]\n");
  printf("         [-ol <output laser points of all extracted roofs (*.laser)>]\n");
  printf("         [-os (overwrite laser data with segmented points)]\n");
  printf("         [-or (output ridge lines and laser data for every roof)]\n");
  printf("         [-s <output statistics (default: statistics.txt)>]\n");
  printf("         [-dpr <maximum distance laser point to ridge (default: 1.0)>]\n");
  printf("         [-minrl <minimum ridge length (default: 3.0)\n");
  printf("         [-maxrsl <maximum ridge slope (default: 0.05)\n");
  printf("         [-drr <maximum distance between matched ridges (default: 2.0)>]\n");
  printf("         [-par <parameter file (default: roofcheck.par)>]\n");
  printf("         [-savepar <parameter file (default: roofcheck.par)>]\n");
  printf("         [-afn (divide strip number by 10 to eliminate AFN number)\n");
  printf("         [-bldno <first building number (default: 0)>]\n");
  printf("         [-rs <first overlap strip after a restart (default: 1)>\n");
  printf("         [-rp <first overlap strip part after a restart (default: 1)>\n");
  printf("Type \"roofcheck -parameters\" to get the listing of segmentation parameters\n");
}

void PrintParameters()
{
  printf("Segmentation parameters of the roofcheck programme\n");
  printf("  [-tin OR -octree OR -knn (default)] : neighbourhood type\n");
  printf("  [-ocbinmax <size> (def: 100)        : maximum octree bin size\n");
  printf("  [-ocbinoverlap <size> (def: 1.0)    : maximum octree bin overlap\n");
  printf("  [-knn <number> (def: 20)            : number of nearest neighbours\n");
  printf("  [-dim <2 or 3> (def: 3)             : neighbour metric dimension\n");
  printf("  [-minsegsize <size> (def: 50)       : minimum segment size\n\n");
  printf("  [-seednbh <0: direct (def), 1: dist>: seed neighbourhood definition\n");
  printf("  [-seedradius <number> (def: 1.0)    : seed neighbourhood radius\n");
  printf("  [-maxslope <degrees> (def: 60.0)    : maximum plane slope\n");
  printf("  [-binsizeslope <degrees> (def: 3.0) : Hough bin size of slope\n");
  printf("  [-binsizedist <number> (def: 0.2)   : Hough bin size of distance\n");
  printf("  [-minsizeseed <number> (def: 10)    : minimum seed size (#pts)\n");
  printf("  [-maxdistseed <number> (def: 0.2)   : maximum distance of point to seed\n\n");
  printf("  [-grownbh <0: direct (def), 1: dist>: growing neighbourhood definition\n");
  printf("  [-growradius <number> (def: 1.0)    : growing search radius\n");
  printf("  [-maxdistgrow <number> (def: 0.3)   : maximum distance of point to surface\n");
  printf("  [-mindistrecompute <num> (def: 0.15): minimum distance to recompute surface\n");
  printf("  [-compete]                          : let planes compete for points on edge\n");
}

int main(int argc, char *argv[])
{
  InlineArguments        *args = new InlineArguments(argc, argv);
  LaserBlock             block;
  ObjectPoints           objpts;
  LineTopologies         top;
  SegmentationParameters par;
  LaserPoints            roof_points;
  
  void                   RoofCheck(LaserBlock &, bool, ObjectPoints &,
                         LineTopologies &, bool, LaserPoints &, const char *,
                         SegmentationParameters &, bool, bool,
                         double, double, double, double, bool, int, int, int);

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
  if (args->Contains("-parameters")) {
    PrintParameters();
    exit(0);
  }
                       
  if (!args->Contains("-i")) {
    printf("Error: -i is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  // Open the block
  if (!block.ReadMetaData(args->String("-i"))) {
    printf("Error reading block %s\n", args->String("-i"));
    exit(0);
  }
  
  // Try to read segmentation parameters if specified
  if (args->Contains("-par")) par.Read(args->String("-par", "roofcheck.par"));
  
  // Get all individually set segmentation parameters
  // Neighbourhood storage model
  if (args->Contains("-tin")) par.NeighbourhoodStorageModel() = 0;
  else if (args->Contains("-octree")) {
    par.NeighbourhoodStorageModel() = 1;
    if (args->Contains("-ocbinmax"))
      par.OctreeBinMaxNumberOfPoints() = args->Integer("-ocbinmax", 100);
    if (args->Contains("-ocbinoverlap"))
      par.OctreeBinOverlap() = args->Double("-ocbinoverlap", 1.0);
  }
  else {
    if (!args->Contains("-par")) 
      par.NeighbourhoodStorageModel() = 2; // knn, default
    if (args->Contains("-knn"))
      par.NumberOfNeighbours() = args->Integer("-knn", 20);
  }
  par.DistanceMetricDimension() = args->Integer("-dim", 3);
  // Minimum component size
  if (args->Contains("-minsigsize"))
    par.MinNumberOfPointsComponent() = args->Integer("-minsegsize", 50);
  // Seed selection parameters
  if (args->Contains("-seednbh"))
    par.SeedNeighbourhoodDefinition() = args->Integer("-seednbh", 0);
  if (args->Contains("-seedradius"))
    par.SeedNeighbourhoodRadius() = args->Double("-seedradius", 1.0);
  if (args->Contains("-maxslope"))
    par.MaxSlopeAngle() = args->Double("-maxslope", 60.0) * atan(1.0) / 45.0;
  if (args->Contains("-binsizeslope"))
    par.BinSizeSlopeAngle() = args->Double("-binsizeslope", 3.0) * atan(1.0) / 45.0;
  if (args->Contains("-binsizedist"))
    par.BinSizeDistance() = args->Double("-binsizedist", 0.2);
  if (args->Contains("-minsizeseed"))
    par.MinNumberOfPointsSeed() = args->Integer("-minsizeseed", 10);
  if (args->Contains("-maxdistseed"))
    par.MaxDistanceSeedPlane() = args->Double("-maxdistseed", 0.2);  
  // Surface growing parameters
  par.SurfaceModel() = 0; // Plane! Smooth surface is not allowed
  if (args->Contains("-grownbh"))
    par.GrowingNeighbourhoodDefinition() = args->Integer("-grownbh", 0);
  if (args->Contains("-growradius"))
    par.GrowingRadius() = args->Double("-growradius", 1.0);
  if (args->Contains("-maxdistgrow"))
    par.MaxDistanceSurface() = args->Double("-maxdistgrow", 0.3);
  if (args->Contains("-mindistrecompute"))
    par.MinDistanceRecompute() = args->Double("-mindistrecompute", 0.15);
  if (args->Contains("-compete"))
    par.SurfacesCompete() = args->Contains("-compete");

  // Analyse the buildings in the overlap
  RoofCheck(block,
            args->Contains("-op"), objpts, top,
            args->Contains("-ol"), roof_points, 
            args->String("-s", "statistics.txt"),
            par, args->Contains("-os"), args->Contains("-or"),
            args->Double("-dpr", 1.0), args->Double("-minrl", 3.0),
            args->Double("-maxrsl", 0.05), args->Double("-drr", 2.0),
            args->Contains("-afn"), args->Integer("-bldno", 0),
			args->Integer("-rs", 1), args->Integer("-rp", 1));

  // Write extracted points and topology
  if (args->Contains("-op"))
    objpts.Write(args->String("-op", "ridge_lines.objpts"));
  if (args->Contains("-ot")) top.Write(args->String("-ot", "ridge_lines.top"));
  if (args->Contains("-ol"))
    roof_points.Write(args->String("-ol", "roof_points.laser"), false, false);

  // Save segmentation parameters
  if (args->Contains("-savepar"))
    par.Write(args->String("-savepar", "roofcheck.par"));
  
  return EXIT_SUCCESS;
}
