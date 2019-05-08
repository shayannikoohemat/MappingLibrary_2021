
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


/*-----------------------------------------------------------

  blocksegments
  
  Merging of segments across tile boundaries based on segment attributes
  
------------------------------------------------------------*/

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include "InlineArguments.h"
#include "BNF_io.h"
#include "SegmentationParameters.h"

using namespace std;

void PrintUsage()
{
  printf("blocksegments merges segments across tile boundaries based on attributes.\n");
  printf("Use growsegments to extract segments within tiles.\n");
  printf("blocksegments will not merge planar surfaces. This is done by blocksurfaces.\n");
  printf("Usage: blocksegments -i <block_file name>   : input file\n");
  printf("        [-odir <directory name>]            : output directory\n");
  printf("        [-app <appendix>]                   : append string for output files\n");
  printf("        [-meta]                             : generate meta data files\n");
  printf("        [-overwrite]                        : overwrite old files\n");
  printf("        [-par <parameter file (default: blocksegments.par)>]\n");
  printf("        [-bz <distance (default: 5.0)>]     : border zone width\n");
  printf("        [-minnp <# points (default: 5)>]    : minimum # points in segment to merge\n");
  printf("You can generate the segmentation parameter file with PCM.\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  SegmentationParameters par;

  void blocksegments(char *, char *, char *, bool, bool,
                     const SegmentationParameters &, double, int);

  printf("%d input arguments\n", argc);
  
  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input files
  if (!args->Contains("-i")) {
    printf("Error: no input data specified with -i <block file name>.\n");
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
  if (!args->Contains("-par") && !BNF_FileExists("blocksegments.par")) {
  	printf("Error: no segmentation parameter file specified and file\n");
  	printf("       blocksegments.par does not exist.\n");
  	exit(0);
  }
  par.Read(args->String("-par", "blocksegments.par"));
  par.SegmentAttribute()=ComponentNumberTag;
  // Segment the data
  blocksegments(args->String("-i"), args->String("-app"),
                args->String("-odir"), args->Contains("-meta"), 
                args->Contains("-overwrite"), par,
	            args->Double("-bz", 5.0),
				args->Integer("-minnp", 5)); 

  return EXIT_SUCCESS;
}
