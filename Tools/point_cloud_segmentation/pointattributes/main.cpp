
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

  pointattributes
  
  Derivation of point attributes in a block of laser data
  
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
  printf("pointattributes derives attributes of points in laser scanning blocks\n\n");
  printf("Usage: pointattributes -i <meta file name>\n");
  printf("               -attribute_switch1 -attribute_switch2 ... \n");
  printf("               [-dtm <height image>] (needed for -HeightAboveGround)\n");
  printf("               [-g <height grid>] (needed for -HeightAboveGround)\n");
  printf("               [-par <segmentation parameter file> (to define neighbourhood)\n");
  printf("\nType pointattributes -list to list all attribute switches\n");
}

void PrintAttributes()
{
  printf("Valid attribute switches are:\n");
  printf(" -HeightAboveGround\n");
  printf(" -NearOtherSegment\n");
  printf(" -NormalInSegment (stored in NormalXTag, etc.) \n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  vector<LaserPointTag> tags;

  void pointattributes(char *, vector<LaserPointTag> &, char *, char *, char *);

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

  // Find an attribute
  if (args->Contains("-HeightAboveGround")) {
    tags.push_back(HeightAboveGroundTag);
    if (!args->Contains("-dtm")) {
      printf("Error: DTM specified by -dtm are needed for -HeightAboveGround\n");
      exit(0);
    }
    if (!args->Contains("-g")) {
      printf("Error: DTM grid specified by -g are needed for -HeightAboveGround\n");
      exit(0);
    }
  }
  if (args->Contains("-NearOtherSegment")) {
    tags.push_back(NearOtherSegmentTag);
    if (!args->Contains("-par")) {
      printf("Error: Segmentation parameters specified by -par needed for -NearOtherSegment\n");
      exit(0);
    }
  }
  if (args->Contains("-NormalInSegment")) {
    tags.push_back(NormalXTag);
    if (!args->Contains("-par")) {
      printf("Error: Segmentation parameters specified by -par needed for -NormalInSegment\n");
      exit(0);
    }
  }
  
  if (tags.empty()) {
    printf("Error: No attribute specified.\n");
    PrintAttributes();
    return EXIT_SUCCESS;
  }
  
  // Derive all attributes
  pointattributes(args->String("-i"), tags, args->String("-dtm"),
                  args->String("-g"), args->String("-par"));

  return EXIT_SUCCESS;
}
