
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


/*-----------------------------------------------------------

  comparelabels
  
  Labels that are identical are removed. Labels of the first data set are
  kept if labels differ.
  
------------------------------------------------------------*/

#include <cstdlib>
#include <iostream>
#include <math.h>
#include "InlineArguments.h"
#include "SegmentationParameters.h"
#include "Line2D.h"

using namespace std;

void PrintUsage()
{
  printf("comparelabels compares the labels of two laser point sets. The sequence\n");
  printf("of the point sets has to be identical. If labels are identical, a label\n");
  printf("is removed. Otherwise one of the labels is kept.\n");
  printf("Usage: comparelabels -r <Reference points>\n");
  printf("                     -c <Classified points>\n");
  printf("                     -er <Points with reference labels of the wrong points\n");
  printf("                     -ec <Points with classification labels of the wrong points\n");
}

int main(int argc, char *argv[])
{
  InlineArguments        *args = new InlineArguments(argc, argv);

  void comparelabels(char *, char *, char *, char *);

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input files
  if (!args->Contains("-r") || !args->Contains("-c")) {
    printf("Error: no input files specified with -r <filename> and -c <filename>.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Compare the point sets
  comparelabels(args->String("-r"), args->String("-c"),
                args->String("-er"), args->String("-ec"));

  return EXIT_SUCCESS;
}
