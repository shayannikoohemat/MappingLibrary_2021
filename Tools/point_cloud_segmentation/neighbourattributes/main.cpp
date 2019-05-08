
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


/*-----------------------------------------------------------

  neighbourattributes
  
  Derivation of attributes of neighbouring segments in a tiled block of laser data
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
  printf("neighbourattributes derives attributes of neighbouring segments in laser scanning blocks\n");
  printf("Segments may cross tile boundaries.\n\n");
  printf("Usage: neighbourattributes -i <block file name>\n");
  printf("                           -attribute_switch1 -attribute_switch2 ... \n");
  printf("                           -fdn (fixed distance neighbourhood) OR\n");
  printf("                           -knn <number of neighbours (def: 20)>\n");
  printf("                           -dim <dimension (def: 2)>\n");
  printf("                           -dmax <maximum distance to neighbour>\n");
  printf("                           -os <output file for segment attributes>\n");
  printf("                           -on <output file for neighbour attributes>\n");
  printf("                           -all (output all attributes, no need for switches)\n");
  printf("\nType neighbourattributes -list to list all attribute switches\n");
}

void PrintAttributes()
{
  printf("Valid attribute switches are:\n");
  printf(" -NumberOfPoints\n -AverageHeightDifference\n");
  printf(" -AverageAngle (average angle between local normal vectors)\n");
  printf(" -Lambda0\n -Lambda1\n -Lambda2\n"); 
  printf(" -Lambda0Scaled\n -Lambda1Scaled\n -Lambda2Scaled\n"); 
  printf(" -Lambda0LocalScaled\n -Lambda1LocalScaled\n -Lambda2LocalScaled\n"); 
}

enum NeighbourAttributeTag { NA_NumberOfPointsTag, NA_AverageHeightDifferenceTag,
                             NA_AverageAngleTag,
                             // Point distribution attributes
							 NA_Lambda0Tag=100, NA_Lambda1Tag, NA_Lambda2Tag,
							 NA_Lambda0ScaledTag, NA_Lambda1ScaledTag, NA_Lambda2ScaledTag,
							 NA_Lambda0LocalScaledTag, NA_Lambda1LocalScaledTag, NA_Lambda2LocalScaledTag,
							 // Sums 
							 NA_Sum1Tag=200
};
typedef enum NeighbourAttributeTag NeighbourAttributeTag;


int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  vector<NeighbourAttributeTag> tags;
  SegmentationParameters parameters;
  bool all;

  void neighbourattributes(char *, vector<NeighbourAttributeTag> &,
                           SegmentationParameters &, char *, char *);

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
  
  // Check on required input
  if (!args->Contains("-i")) {
    printf("Error: no input data specified with -i <filename>.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-os")) {
    printf("Error: no file name for segment output specified with -os <filename>.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-on")) {
    printf("Error: no file name for neighbourhood output specified with -on <filename>.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-fdn") && !args->Contains("-knn")) {
  	printf("Error: neighbourhood should be defined by either -fdn or -knn\n");
  	PrintUsage();
  	return EXIT_SUCCESS;
  }
  if (args->Contains("-fdn") && args->Contains("-knn")) {
  	printf("Error: -fdn or -knn cannot be used in combination\n");
  	PrintUsage();
  	return EXIT_SUCCESS;
  }

  // Find an attribute
  all = args->Contains("-all");
  if (args->Contains("-NumberOfPoints") || all) tags.push_back(NA_NumberOfPointsTag);
  if (args->Contains("-AverageHeightDifference") || all) tags.push_back(NA_AverageHeightDifferenceTag);
  if (args->Contains("-Lambda0") || all) tags.push_back(NA_Lambda0Tag);
  if (args->Contains("-Lambda1") || all) tags.push_back(NA_Lambda1Tag);
  if (args->Contains("-Lambda2") || all) tags.push_back(NA_Lambda2Tag);
  if (args->Contains("-Lambda0Scaled") || all) tags.push_back(NA_Lambda0ScaledTag);
  if (args->Contains("-Lambda1Scaled") || all) tags.push_back(NA_Lambda1ScaledTag);
  if (args->Contains("-Lambda2Scaled") || all) tags.push_back(NA_Lambda2ScaledTag);
  if (args->Contains("-Lambda0LocalScaled") || all) tags.push_back(NA_Lambda0LocalScaledTag);
  if (args->Contains("-Lambda1LocalScaled") || all) tags.push_back(NA_Lambda1LocalScaledTag);
  if (args->Contains("-Lambda2LocalScaled") || all) tags.push_back(NA_Lambda2LocalScaledTag);
  if (args->Contains("-AverageAngle") || all) {
    tags.push_back(NA_AverageAngleTag);
    tags.push_back(NA_Sum1Tag);
  }
  if (tags.empty()) {
    printf("Error: No attribute specified.\n");
    PrintAttributes();
    return EXIT_SUCCESS;
  }
  
  // Set the neighbourhood parameters
  parameters.NeighbourhoodStorageModel() = 2;  // kd-tree
  parameters.DistanceMetricDimension() = args->Integer("-dim", 2); // default 2D
  if (args->Contains("-fdn")) {
    parameters.GrowingNeighbourhoodDefinition() = 1; // All edges within radius
    parameters.GrowingRadius() = args->Double("-dmax", 1.0);
  }
  else {
    parameters.GrowingNeighbourhoodDefinition() = 0; // Only direct edges
  	parameters.NumberOfNeighbours() = args->Integer("-knn", 20);
    if (args->Contains("-dmax"))
      parameters.GrowingRadius() = args->Double("-dmax", 1.0);
  }
  
  // Derive all attributes
  neighbourattributes(args->String("-i"), tags, parameters, args->String("-os"),
                      args->String("-on")); 

  return EXIT_SUCCESS;
}
