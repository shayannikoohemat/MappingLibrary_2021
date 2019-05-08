
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
#include <math.h>
#include "InlineArguments.h"
#include "LaserBlock.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: roofverify\n");
  printf("         [-numin <first input roof number> (def: 0)]\n");
  printf("         [-s <output statistics (default: statsverified.txt)>]\n");
  printf("         [-r <output verification results (default: verification_results.txt)>]\n");
  printf("         [-odir <output directory for accepted roofs (default ./)]\n");
  printf("         [-odirf <output directory for rejected roofs (default same as -odir)]\n");
  printf("         [-dpp <maximum distance to point in other strip (default 1.5)]\n");
  printf("         [-knn <number> (def: 20) : number of nearest neighbours]\n");
  printf("         [-mins <number> (def: 1.5) : minimum size of bounding box]\n");
  printf("         [-maxs <number> (def: 40.0) : maximum size of bounding box]\n");
  printf("         [-minnumpts <number> (def: 0) : mimimum number of points on a plane]\n");
  printf("                if 0 then the minimum number is derived based on ridge line accuracy\n");
  printf("         [-dpr <number> (def: 1.0) : maximum distance of point to ridge]\n");
  printf("         [-dpf <number> (def: 0.1) : maximum distance of point to face]\n");
  printf("         [-minrl <number> (def: 2.0) : minimum ridge length]\n");
  printf("         [-maxrs <number> (def: 3.0 degrees) : maximum ridge slope]\n");
  printf("         [-drr <number> (def: 1.0) : maximum distance between ridges]\n");
  printf("         [-maxa <number> (def: 5.0 degrees) : maximum angle between corresponding planes]\n");
  printf("         [-mina <number> (def: 20 degrees) : minimum intersection angle between planes]\n");
  printf("         [-maxra <number> (def: 3.0 degrees) : maximum angle between ridge lines]\n");
  printf("         [-numout <first output number of an accepted roof (default: 0)>]\n");
  printf("         [-numoutf <first output number of a rejected roof (default: 0)>]\n");
  printf("         [-afn] : Scan direction number stored in scan number]\n");
  printf("         [-rd] : Remove double points\n");
  printf("         [-maxnum <number> (def: 0=no limit) : maximum number of roofs to process]\n");
  printf("         [-par <file name> : file with segmentation parameters]\n");
  printf("         [-d5 Use 5 digits for roof numbers in input files (otherwise 6 digits are used)]\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  void            RoofVerify(int, const char *, const char *, const char *,
                             const char *,
                             double, int, double,  double,
                             double, double, double, double, int, int, 
                             bool, bool, int, 
                             double, double, double, int, double,
                             const char *, bool);

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  // Verify the buildings in the overlap
  RoofVerify(args->Integer("-numin", 0), args->String("-s", "statsverified.txt"),
             args->String("-r", "verification_results.txt"),
             args->String("-odir", "./"), 
             args->String("-odirf", args->String("-odir", "./")),
             args->Double("-dpp", 1.5),
             args->Integer("-knn", 20), args->Double("-mins", 1.5),
             args->Double("-dpr", 1.0), args->Double("-dpf", 0.1),
             args->Double("-minrl", 2.0), args->Double("-drr", 1.0), 
             args->Double("-maxa", 5.0) * atan(1.0) / 45.0,
             args->Integer("-numout", 0), 
             args->Integer("-numoutf", 0), 
             args->Contains("-afn"),
             args->Contains("-rd"),  args->Integer("-minnumpts", 0),
             args->Double("-maxrs", 3.0) * atan(1.0) / 45.0,
             args->Double("-mina", 20.0) * atan(1.0) / 45.0,
             args->Double("-maxra", 3.0) * atan(1.0) / 45.0,
             args->Integer("-maxnum", 0),
             args->Double("-maxs", 40.0),
             args->String("-par", NULL),
             args->Contains("-d5"));

  return EXIT_SUCCESS;
}
