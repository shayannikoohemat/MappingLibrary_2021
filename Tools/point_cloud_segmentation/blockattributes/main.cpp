
/*
    Copyright 2013 University of Twente and Delft University of Technology
 
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


/*-----------------------------------------------------------

  blockattributes
  
  Derivation of segment attributes in a tiled block of laser data
  Segments may extend over multiple tiles.
  
------------------------------------------------------------*/

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include "InlineArguments.h"
#include "SegmentationParameters.h"

using namespace std;

void PrintUsage()
{
  printf("blockattributes derives attributes of segments in laser scanning blocks\n");
  printf("Segments may cross tile boundaries.\n\n");
  printf("Usage: blockattributes -i <block file name>\n");
  printf("                       -attribute_switch1 -attribute_switch2 ... \n");
  printf("                       [-par <segmentation parameters for local nbh def>]\n");
  printf("\nType blockattributes -list to list all segment attribute switches\n");
}

void PrintAttributes()
{
  printf("Valid attribute switches are:\n");
  printf(" -AverageReflectance\n -AveragePulseCount\n -AverageIsFiltered\n");
  printf(" -AverageIsProcessed\n -AverageResidual\n -AverageIsSelected\n");
  printf(" -AveragePulseLength\n -AverageScalar\n -AverageTime\n -AverageAngle\n");
  printf(" -AverageXCoordinate\n -AverageYCoordinate\n -AverageZCoordinate\n");
  printf(" -AverageHeightAboveGround\n -AveragePointSpacing\n");
  printf(" -AverageLambda0Local\n -AverageLambda1Local\n -AverageLambda2Local\n");
  printf(" -AverageLambda0LocalScaled\n -AverageLambda1LocalScaled\n -AverageLambda2LocalScaled\n");
  printf(" -AverageFlatnessLocal\n -AverageLinearityLocal\n");
  printf(" -SlopeAngleVariance\n");
  printf(" -MinReflectance\n -MinPulseCount\n -MinLabel\n -MinResidual\n -MinPlaneNumber\n");
  printf(" -MinScanNumber\n -MinPulseLength\n -MinPolygonNumber\n -MinScalar\n");
  printf(" -MinTime\n -MinAngle\n -MinXCoordinate\n -MinYCoordinate\n -MinZCoordinate\n");
  printf(" -MinHeightAboveGround\n");
  printf(" -MaxReflectance\n -MaxPulseCount\n -MaxLabel\n -MaxResidual\n -MaxPlaneNumber\n");
  printf(" -MaxScanNumber\n -MaxPulseLength\n -MaxPolygonNumber\n -MaxScalar\n");
  printf(" -MaxTime\n -MaxAngle\n -MaxXCoordinate\n -MaxYCoordinate\n -MaxZCoordinate\n");
  printf(" -MaxHeightAboveGround\n");
  printf(" -SegmentSize\n -PercFirstPulse\n -PercSecondPulse\n -PercThirdPulse\n");
  printf(" -PercFourthPulse\n -PercLastPulse\n -PercNotFirstPulse\n -PercNotLastPulse\n");
  printf(" -PercMultiPulse\n -PercIsFiltered\n -PercIsProcessed\n -PercIsSelected\n");
  printf(" -PercNearOtherSegment\n");
  printf(" -Inclination\n -Azimuth\n -Lambda0\n -Lambda1\n -Lambda2\n");
  printf(" -Lambda0Scaled\n -Lambda1Scaled\n -Lambda2Scaled\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  vector<LaserPointTag> tags;
  SegmentationParameters parameters;

  void blockattributes(char *, vector<LaserPointTag> &,
                       const SegmentationParameters &);

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // List switches
  if (args->Contains("-list")) {
  	PrintAttributes();
  	return EXIT_SUCCESS;
  }
  
  // Check on required input files
  if (!args->Contains("-i")) {
    printf("Error: no input data specified with -i <filename>.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-par") &&
      (args->Contains("-AverageLambda0Local") || 
	   args->Contains("-AverageLambda1Local") ||
	   args->Contains("-AverageLambda2Local"))) {
    printf("Error: Segmentation parameters should be specified with -par if.\n");
    printf("       local eigenvalues need to be computed.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Find an attribute
  if (args->Contains("-AverageReflectance")) tags.push_back(AverageReflectanceTag);
  if (args->Contains("-AveragePulseCount")) tags.push_back(AveragePulseCountTag);
  if (args->Contains("-AverageIsFiltered")) tags.push_back(AverageIsFilteredTag);
  if (args->Contains("-AverageIsProcessed")) tags.push_back(AverageIsProcessedTag);
  if (args->Contains("-AverageColour")) tags.push_back(AverageColourTag);
  if (args->Contains("-AverageResidual")) tags.push_back(AverageResidualTag);
  if (args->Contains("-AverageIsSelected")) tags.push_back(AverageIsSelectedTag);
  if (args->Contains("-AveragePulseLength")) tags.push_back(AveragePulseLengthTag);
  if (args->Contains("-AverageScalar")) tags.push_back(AverageScalarTag);
  if (args->Contains("-AverageTime")) tags.push_back(AverageTimeTag);
  if (args->Contains("-AverageAngle")) tags.push_back(AverageAngleTag);
  if (args->Contains("-AverageXCoordinate")) tags.push_back(AverageXCoordinateTag);
  if (args->Contains("-AverageYCoordinate")) tags.push_back(AverageYCoordinateTag);
  if (args->Contains("-AverageZCoordinate")) tags.push_back(AverageZCoordinateTag);
  if (args->Contains("-AverageHeightAboveGround")) tags.push_back(AverageHeightAboveGroundTag);
  if (args->Contains("-AveragePointSpacing")) {
    tags.push_back(AveragePointSpacingTag);
    tags.push_back(EdgeCountTag);
  }
  if (args->Contains("-AverageLambda0Local")) tags.push_back(AverageLambda0LocalTag);
  if (args->Contains("-AverageLambda1Local")) tags.push_back(AverageLambda1LocalTag);
  if (args->Contains("-AverageLambda2Local")) tags.push_back(AverageLambda2LocalTag);
  if (args->Contains("-AverageFlatnessLocal")) tags.push_back(AverageFlatnessLocalTag);
  if (args->Contains("-AverageLinearityLocal")) tags.push_back(AverageLinearityLocalTag);
  if (args->Contains("-AverageLambda0LocalScaled")) tags.push_back(AverageLambda0LocalScaledTag);
  if (args->Contains("-AverageLambda1LocalScaled")) tags.push_back(AverageLambda1LocalScaledTag);
  if (args->Contains("-AverageLambda2LocalScaled")) tags.push_back(AverageLambda2LocalScaledTag);
  if (args->Contains("-SlopeAngleVariance")) {
    tags.push_back(SlopeAngleVarianceTag);
    tags.push_back(Sum1Tag);
    tags.push_back(Sum2Tag);
  }
  if (args->Contains("-MinReflectance")) tags.push_back(MinReflectanceTag);
  if (args->Contains("-MinPulseCount")) tags.push_back(MinPulseCountTag);
  if (args->Contains("-MinLabel")) tags.push_back(MinLabelTag);
  if (args->Contains("-MinResidual")) tags.push_back(MinResidualTag);
  if (args->Contains("-MinPlaneNumber")) tags.push_back(MinPlaneNumberTag);
  if (args->Contains("-MinScanNumber")) tags.push_back(MinScanNumberTag);
  if (args->Contains("-MinPulseLength")) tags.push_back(MinPulseLengthTag);
  if (args->Contains("-MinPolygonNumber")) tags.push_back(MinPolygonNumberTag);
  if (args->Contains("-MinScalar")) tags.push_back(MinScalarTag);
  if (args->Contains("-MinTime")) tags.push_back(MinTimeTag);
  if (args->Contains("-MinAngle")) tags.push_back(MinAngleTag);
  if (args->Contains("-MinXCoordinate")) tags.push_back(MinXCoordinateTag);
  if (args->Contains("-MinYCoordinate")) tags.push_back(MinYCoordinateTag);
  if (args->Contains("-MinZCoordinate")) tags.push_back(MinZCoordinateTag);
  if (args->Contains("-MinHeightAboveGround")) tags.push_back(MinHeightAboveGroundTag);
  if (args->Contains("-MaxReflectance")) tags.push_back(MaxReflectanceTag);
  if (args->Contains("-MaxPulseCount")) tags.push_back(MaxPulseCountTag);
  if (args->Contains("-MaxLabel")) tags.push_back(MaxLabelTag);
  if (args->Contains("-MaxResidual")) tags.push_back(MaxResidualTag);
  if (args->Contains("-MaxPlaneNumber")) tags.push_back(MaxPlaneNumberTag);
  if (args->Contains("-MaxScanNumber")) tags.push_back(MaxScanNumberTag);
  if (args->Contains("-MaxPulseLength")) tags.push_back(MaxPulseLengthTag);
  if (args->Contains("-MaxPolygonNumber")) tags.push_back(MaxPolygonNumberTag);
  if (args->Contains("-MaxScalar")) tags.push_back(MaxScalarTag);
  if (args->Contains("-MaxTime")) tags.push_back(MaxTimeTag);
  if (args->Contains("-MaxAngle")) tags.push_back(MaxAngleTag);
  if (args->Contains("-MaxXCoordinate")) tags.push_back(MaxXCoordinateTag);
  if (args->Contains("-MaxYCoordinate")) tags.push_back(MaxYCoordinateTag);
  if (args->Contains("-MaxZCoordinate")) tags.push_back(MaxZCoordinateTag);
  if (args->Contains("-MaxHeightAboveGround")) tags.push_back(MaxHeightAboveGroundTag);
  if (args->Contains("-SegmentSize")) tags.push_back(SegmentSizeTag);
  if (args->Contains("-PercFirstPulse")) tags.push_back(PercFirstPulseTag);
  if (args->Contains("-PercSecondPulse")) tags.push_back(PercSecondPulseTag);
  if (args->Contains("-PercThirdPulse")) tags.push_back(PercThirdPulseTag);
  if (args->Contains("-PercFourthPulse")) tags.push_back(PercFourthPulseTag);
  if (args->Contains("-PercLastPulse")) tags.push_back(PercLastPulseTag);
  if (args->Contains("-PercNotFirstPulse")) tags.push_back(PercNotFirstPulseTag);
  if (args->Contains("-PercNotLastPulse")) tags.push_back(PercNotLastPulseTag);
  if (args->Contains("-PercMultiPulse")) tags.push_back(PercMultiPulseTag);
  if (args->Contains("-PercIsFiltered")) tags.push_back(PercIsFilteredTag);
  if (args->Contains("-PercIsProcessed")) tags.push_back(PercIsProcessedTag);
  if (args->Contains("-PercIsSelected")) tags.push_back(PercIsSelectedTag);
  if (args->Contains("-PercNearOtherSegment")) tags.push_back(PercNearOtherSegmentTag);
  if (args->Contains("-Inclination")) tags.push_back(InclinationTag);
  if (args->Contains("-Azimuth")) tags.push_back(AzimuthTag);
  if (args->Contains("-Lambda0")) tags.push_back(Lambda0Tag);
  if (args->Contains("-Lambda1")) tags.push_back(Lambda1Tag);
  if (args->Contains("-Lambda2")) tags.push_back(Lambda2Tag);
  if (args->Contains("-Lambda0Scaled")) tags.push_back(Lambda0ScaledTag);
  if (args->Contains("-Lambda1Scaled")) tags.push_back(Lambda1ScaledTag);
  if (args->Contains("-Lambda2Scaled")) tags.push_back(Lambda2ScaledTag);
  if (tags.empty()) {
    printf("Error: No attribute specified.\n");
    PrintAttributes();
    return EXIT_SUCCESS;
  }
  
  // Derive all attributes
  blockattributes(args->String("-i"), tags, parameters); 

  return EXIT_SUCCESS;
}
