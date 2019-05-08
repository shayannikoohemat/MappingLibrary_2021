
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


#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"
#include "LaserPoint.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: settag -i <meta file or point file> OR\n");
  printf("              -f <file filter>\n");
  printf("              -t <tag to be (re-)set>\n");
  printf("              -v <tag integer value>\n");
  printf("              -vf <tag float value\n\n");
  printf("Note that the input files will be overwritten\n\n");
  printf("Some possible tag numbers are:\n");
  printf("  ReflectanceTag         %d\n", ReflectanceTag);
  printf("  PulseCountTag          %d\n", PulseCountTag);
  printf("  LabelTag               %d\n", LabelTag);
  printf("  IsFilteredTag          %d\n", IsFilteredTag);
  printf("  IsProcessedTag         %d\n", IsProcessedTag);
  printf("  ColourTag              %d\n", ColourTag);
  printf("  ResidualTag            %d\n", ResidualTag);
  printf("  IsSelectedTag          %d\n", IsSelectedTag);
  printf("  PlaneNumberTag         %d\n", PlaneNumberTag);
  printf("  SegmentNumberTag       %d\n", SegmentNumberTag);
  printf("  ScanNumberTag          %d\n", ScanNumberTag);
  printf("  PointNumberTag         %d\n", PointNumberTag);
  printf("  PulseLengthTag         %d\n", PulseLengthTag);
  printf("  PolygonNumberTag       %d\n", PolygonNumberTag);
  printf("  IntensityTag           %d\n", IntensityTag);
  printf("  v_xTag                 %d\n", v_xTag);
  printf("  v_yTag                 %d\n", v_yTag);
  printf("  v_zTag                 %d\n", v_zTag);
  printf("  cv_xyTag               %d\n", cv_xyTag);
  printf("  cv_xzTag               %d\n", cv_xzTag);
  printf("  cv_yzTag               %d\n", cv_yzTag);
  printf("  NumRaysTag             %d\n", NumRaysTag);
  printf("  ScalarTag              %d\n", ScalarTag);
  printf("  TimeTag                %d\n", TimeTag);
  printf("  AngleTag               %d\n", AngleTag);
  printf("  ComponentNumberTag,    %d\n", ComponentNumberTag);
  printf("  NormalXTag             %d\n", NormalXTag);
  printf("  NormalYTag             %d\n", NormalYTag);
  printf("  NormalZTag             %d\n", NormalZTag);
  printf("  FlatnessTag            %d\n", FlatnessTag);
  printf("  LinearityTag           %d\n", LinearityTag);
  printf("  ScaledNormalXTag       %d\n", ScaledNormalXTag);
  printf("  ScaledNormalYTag       %d\n", ScaledNormalYTag);
  printf("  ScaledNormalZTag       %d\n", ScaledNormalZTag);
  printf("  SegmentStartTileNumber %d\n", SegmentStartTileNumberTag);
  
  printf("\nFor more attributes check LaserPoint.h in the LaserScan library.\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void settag_cpp(char *, char *, int, int, float);
  
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i") && !args->Contains("-f")) {
    printf("Error: -i or -f is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (args->Contains("-i") && args->Contains("-f")) {
    printf("Error: -i and -f are mutually exclusive arguments.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-t")) {
    printf("Error: -t is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (args->Integer("-t", -1) == ResidualTag) {
    if (!args->Contains("-vf")) {
      printf("Error: -vf is a required argument.\n");
      PrintUsage();
      exit(0);
    }
  }
  else {
    if (!args->Contains("-v")) {
      printf("Error: -v or -vf is a required argument.\n");
      PrintUsage();
      exit(0);
    }
  }
  
  settag_cpp(args->String("-f"), args->String("-i"),
             args->Integer("-t", -1),
             args->Integer("-v", -1), (float) args->Double("-vf", -1.0));

  return EXIT_SUCCESS;
}
