
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
|    Written By: George Vosselman
|          Date: Nov 05, 2003
| Modifications:
|
------------------------------------------------------------*/

#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include "InlineArguments.h"
#include "viff.h"
#include "LaserBlock.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: laser2viff\n");
  printf("  [-i <meta data input file>] OR\n");
  printf("  [-f <meta data input file filter>\n\n");
  printf("  -s <pixel size in m>\n\n");
  printf("  [-bi] Take bounds from meta data OR\n");
  printf("  [-bm <meta data file>] Take bounds from specified file OR\n");
  printf("  [-bu] Use bounds specified by following arguments\n\n");
  printf("  [-height] Create height image AND/OR\n");
  printf("  [-reflectance] Create reflectance strength image AND/OR\n");
  printf("  [-colour] Create colour image AND/OR\n");
  printf("  [-pulselength] Create pulse length image AND/OR\n");
  printf("  [-pointcount] Create point count image\n\n");
  printf("  [-xmin <value>, -xmax <value>] X range of area to be imaged\n");
  printf("  [-ymin <value>, -ymax <value>] X range of area to be imaged\n");
  printf("  [-zmin <value>] Height to be mapped to grey value 0\n");
  printf("  [-zmax <value>] Height to be mapped to grey value 255\n");
  printf("  [-rmin <value>] Reflectance to be mapped to grey value 0\n");
  printf("  [-rmax <value>] Reflectance to be mapped to grey value 255\n");
  printf("  [-plmin <value>] Pulse length to be mapped to grey value 0\n");
  printf("  [-plmax <value>] Pulse length to be mapped to grey value 255\n\n");
  printf("  [-oh <file name>] File name of height image if not auto-generated\n");
  printf("  [-or <file name>] File name of refectance strength image if not\n");
  printf("                    auto-generated\n");
  printf("  [-oc <file name>] File name of colour image if not auto-generated\n");
  printf("  [-opl <file name>] File name of pulse length image if not auto-generated\n");
  printf("  [-opc <file name>] File name of point count image if not auto-generated\n\n");
  printf("  [-gg] Generate grid definition file(s)\n");
  printf("  [-hgrid <file name>] File name of height grid if not auto-generated\n");
  printf("  [-rgrid <file name>] File name of refectance strength grid if not\n");
  printf("                       auto-generated\n");
  printf("  [-cgrid <file name>] File name of colour grid if not auto-generated\n");
  printf("  [-plgrid <file name>] File name of pulse length grid if not auto-generated\n");
  printf("  [-pcgrid <file name>] File name of point count grid if not auto-generated\n\n");
  printf("  [-odir <directory>] Output directory of created images\n");
  printf("  [-update] Update meta data file with image and grid file names\n");
  printf("  [-om <file name>] File name of meta data output if not overwriting input file\n");
  printf("  [-in <number>] Interpolation method: 1: nearest neighbour (def),\n");
  printf("                 2: barycentric, 3: linear in TIN, 4: minimum, 5: maximum\n");
  printf("  [-mms <value, def: 10.0>] Maximum size of TIN mesh to be interpolated\n");
  printf("  [-type <number>] Image type: 1: unsigned char, 2: float\n");
  printf("  [-si] Generate images for all strips of a block\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  int             viff_type, fileclass;
  void laser2viff_cpp(char *, char *, int, int, int, int, int, int, double, int,
                      char *, int, int, double, int, double, int, double,
                      int, double, int, double, int, double, 
                      int, int, int, int,
                      int, int, int, int,
                      char *, char *, char *, char *, char *, char *, int,
                      char *, int, char *, char *, char *, char *, char *,
                      int, double);
  LaserBlock block;
  LaserBlock::iterator strip;
  bool strip_wise=false;

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
  if (!args->Contains("-s")) {
    printf("Error: no scale specified with -s <scale>.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-bi") && !args->Contains("-bm") &&
      !args->Contains("-bu")) {
    printf("Error: no bounds specified with -bi, -bm or -bu.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  viff_type = VFF_TYP_1_BYTE;
  if (args->Contains("-type"))
    if (args->Integer("-type", 1) == 2) viff_type = VFF_TYP_FLOAT;

  // Check if the main laser2viff function needs to be called for multiple
  // strips of a block
  
  if (args->Contains("-i") && args->Contains("-si")) {
    if (!block.Create(args->String("-i"), &fileclass, false, false)) {
      fprintf(stderr, "Error reading meta data file %s\n", args->String("-i"));
      exit(0);
    }
    if (fileclass == LASER_BLOCK) {
      strip_wise = true;
      printf("Strip wise imaging of block %s\n", block.Name());
    }
  }

  if (!strip_wise)
    laser2viff_cpp(args->String("-f"), args->String("-i"),
                   args->Contains("-height"), args->Contains("-reflectance"),
                   args->Contains("-colour"), args->Contains("-pulselength"),
                   args->Contains("-pointcount"), viff_type,
                   args->Double("-s", 100.0), args->Contains("-bi"),
                   args->String("-bm"), args->Contains("-bu"),
                   args->Contains("-xmin"), args->Double("-xmin", 0.0),
                   args->Contains("-xmax"), args->Double("-xmax", 0.0),
                   args->Contains("-ymin"), args->Double("-ymin", 0.0),
                   args->Contains("-ymax"), args->Double("-ymax", 0.0),
                   args->Contains("-zmin"), args->Double("-zmin", 0.0),
                   args->Contains("-zmax"), args->Double("-zmax", 0.0),
                   args->Contains("-rmin"), args->Integer("-rmin", 0),
                   args->Contains("-rmax"), args->Integer("-rmax", 0),
                   args->Contains("-plmin"), args->Integer("-plmin", 0),
                   args->Contains("-plmax"), args->Integer("-plmax", 0),
                   args->String("-odir"), args->String("-oh"),
                   args->String("-or"), args->String("-oc"),
                   args->String("-opl"), args->String("-opc"),
                   args->Contains("-update"), args->String("-om"),
                   args->Contains("-gg"), args->String("-hgrid"),
                   args->String("-rgrid"), args->String("-cgrid"),
                   args->String("-plgrid"), args->String("-pcgrid"),
                   args->Integer("-in", 1), args->Double("-mms", 10.0));
  else {
    printf("Block has %d strips\n", block.size());
    for (strip=block.begin(); strip!=block.end(); strip++)
      laser2viff_cpp(args->String("-f"), strip->MetaDataFile(),
                     args->Contains("-height"), args->Contains("-reflectance"),
                     args->Contains("-colour"), args->Contains("-pulselength"),
                     args->Contains("-pointcount"), viff_type,
                     args->Double("-s", 100.0), args->Contains("-bi"),
                     args->String("-bm"), args->Contains("-bu"),
                     args->Contains("-xmin"), args->Double("-xmin", 0.0),
                     args->Contains("-xmax"), args->Double("-xmax", 0.0),
                     args->Contains("-ymin"), args->Double("-ymin", 0.0),
                     args->Contains("-ymax"), args->Double("-ymax", 0.0),
                     args->Contains("-zmin"), args->Double("-zmin", 0.0),
                     args->Contains("-zmax"), args->Double("-zmax", 0.0),
                     args->Contains("-rmin"), args->Integer("-rmin", 0),
                     args->Contains("-rmax"), args->Integer("-rmax", 0),
                     args->Contains("-plmin"), args->Integer("-plmin", 0),
                     args->Contains("-plmax"), args->Integer("-plmax", 0),
                     args->String("-odir"), args->String("-oh"),
                     args->String("-or"), args->String("-oc"),
                     args->String("-opl"), args->String("-opc"),
                     args->Contains("-update"), args->String("-om"),
                     args->Contains("-gg"), args->String("-hgrid"),
                     args->String("-rgrid"), args->String("-cgrid"),
                     args->String("-plgrid"), args->String("-pcgrid"),
                     args->Integer("-in", 1), args->Double("-mms", 10.0));
  }
  return EXIT_SUCCESS;
}
