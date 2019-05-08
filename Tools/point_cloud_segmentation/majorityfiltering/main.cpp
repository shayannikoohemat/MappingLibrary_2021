
/*
    Copyright 2013 University of Twente
 
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
#include <math.h>
#include "InlineArguments.h"
#include "SegmentationParameters.h"

using namespace std;

void PrintUsage()
{
  printf("Majority filtering of point attributes.\n");
  printf("Usage: majorityfiltering [-i <file name> OR -f <file filter>] : input files\n");
  printf("        -par <parameter file (default: majorityfiltering.par)>\n");
  printf("        [-odir <directory name>]            : output directory\n");
  printf("        [-app <appendix>]                   : append string for output files\n");
  printf("        [-meta]                             : generate meta data files\n");
  printf("        [-overwrite]                        : overwrite old files\n");
  printf("Use pcm to create the parameter file.\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  SegmentationParameters par;

  void majorityfiltering(char *, char *, char *, char *, bool, bool,
                         const SegmentationParameters &);

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input files
  if (!args->Contains("-i") && !args->Contains("-f")) {
    printf("Error: no input data specified with -i <filename> or -f <filter>.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (args->Contains("-i") && args->Contains("-f")) {
    printf("Error: -i and -f can not be used simultaneously!\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-app") && !args->Contains("-overwrite")) {
    printf("Error: no appendix for output files specified with -app <appendix>.\n");
    printf("       The input file is only overwritten if you specify -overwrite.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Try to read segmentation parameters if specified
  if (!par.Read(args->String("-par", "majorityfiltering.par"))) {
  	printf("Error reading parameters from %s\n",
	       args->String("-par", "majorityfiltering.par"));
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Segment the data
  majorityfiltering(args->String("-f"), args->String("-i"), args->String("-app"),
                    args->String("-odir"), args->Contains("-meta"),
                    args->Contains("-overwrite"), par); 

  return EXIT_SUCCESS;
}
