
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


/*-----------------------------------------------------------

  blocksurfaces
  
  Segmentation of laser block into planar or smooth surfaces
  
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
  printf("blocksurfaces merges surfaces across tile boundaries. Before using\n");
  printf("this programme, first use mergesurfaces to merge surfaces within tiles.\n\n");
  printf("Usage: blocksurfaces -i <block_file name>    : input file\n");
  printf("        [-odir <directory name>]            : output directory\n");
  printf("        [-app <appendix>]                   : append string for output files\n");
  printf("        [-meta]                             : generate meta data files\n");
  printf("        [-overwrite]                        : overwrite old files\n");
  printf("        [-par <parameter file (default: blocksurfaces.par)>]\n");
  printf("        [-bz <distance (default: 5.0)>]     : border zone width\n");
  printf("        [-tilefit] (fit planes to tile data instead of to data in border zone only\n");
  printf("        [-minl (minimum length of the border line connecting two surfaces, def: 0.5)\n");
  printf("You can generate the segmentation parameter file with PCM or segmentlaser.\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  SegmentationParameters par;

  void blocksurfaces(char *, char *, char *, bool, bool,
                    const SegmentationParameters &, double, bool, double);

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
  if (!args->Contains("-par") && !BNF_FileExists("blocksurfaces.par")) {
  	printf("Error: no segmentation parameter file specified and file\n");
  	printf("       blocksurfaces.par does not exist.\n");
  	exit(0);
  }
  par.Read(args->String("-par", "blocksurfaces.par"));


  // Segment the data
  blocksurfaces(args->String("-i"), args->String("-app"),
                args->String("-odir"), args->Contains("-meta"), 
                args->Contains("-overwrite"), par,
	            args->Double("-bz", 5.0), args->Contains("-tilefit"),
				args->Double("-minl", 0.5)); 

  return EXIT_SUCCESS;
}
