
/*
                Copyright 2017 University of Twente
 
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
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("hokuyo2laser converts a cvs file with Hokuyo readings from ROS to the\n");
  printf("laser format. A sequence of recordings is considered a strip which\n");
  printf("consists of multiple strip parts. Meta data for the strip and strip parts\n");
  printf("are generated. When available scanner calibration data can be applied to\n");
  printf("the recorded ranges and angles in the conversion process.\n");
  printf("If the scanning system configuration file is supplied, the calibration data\n");
  printf("is extracted from that file. The scanner orientation data is also applied\n");
  printf("to the point coordinates. The scanner id needs to be specified with -scanner.\n");
  printf("Alternatively, three input cvs files can be supplied and all data will be converted\n");
  printf("to three strips, together with block meta data\n\n");
  printf("Usage: hokuyo2laser -i <input csv file from ROS>\n");
  printf("                    OR\n");
  printf("                    -i0, -i1 and -i2 for three input cvs files from ROS\n");
  printf("                    -root <root name of output files>\n");
  
  printf("\n       Optional parameters\n");
  printf("                    -cal <scanner calibration file>\n");
  printf("                    -sys <scanner system configuration file>\n");
  printf("                    -imu <IMU readings file>\n");
  printf("                    -scanner <id>\n");
  printf("                    -odir <output directory name, def: .\\>\n");
  printf("                    -maxp <maximum number of points per output file, e.g. 1.000.000> OR\n");
  printf("                    -maxl <maximum number of scan lines per output file (e.g. 1000)> OR\n");
  printf("                    -maxt <maximum time interval per output file (e.g. 25 seconds)>\n");
  printf("                    -tref <csv file with data of last operating scanner> (def: current input file)>\n");
  printf("					  -sclstep n (only write every nth scanline)\n");
}

int main(int argc, char *argv[])
{
  int             maxp, maxl, maxt, tstart;
  InlineArguments *args = new InlineArguments(argc, argv);

  void hokuyo2laser_cpp(char *, char *, char *, char *, char *, char *, 
                        int, int, double, char *, char *, char *, int, int,
						char *);
  
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i") && !args->Contains("-i0")) {
    printf("Error: No input file specified with -i or -i0.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-root")) {
    printf("Error: No root name specified for output files with -root.\n");
    PrintUsage();
    exit(0);
  }
  if (args->Contains("-i0") && (!args->Contains("-i1") || !args->Contains("-i2"))) {
  	printf("Error: Input file specified with -i0 without using -i1 and/or -i2\n");
  	PrintUsage();
  	exit(0);
  }
  if (!args->Contains("-maxp") && !args->Contains("-maxl") &&
      !args->Contains("-maxt")) {
    printf("Error: Maximum file size should be specified with -maxp, -maxl or -maxt.\n");
    PrintUsage();
    exit(0);
  }
  if ((args->Contains("-maxp") && args->Contains("-maxl")) ||
      (args->Contains("-maxp") && args->Contains("-maxt")) ||
	  (args->Contains("-maxl") && args->Contains("-maxt"))) {
	printf("Error: -maxp, -maxl, and -maxt are mutually exclusive.\n");
    PrintUsage();
    exit(0);
  }
  if (args->Contains("-cal") && args->Contains("-sys")) {
  	printf("Error: -cal and -sys are mutually exclusive.\n");
  	PrintUsage();
  	exit(0);
  }
  if (args->Contains("-sys") && !args->Contains("-scanner") &&
      args->Contains("-i")) {
  	printf("Error: -scanner needs to be used to specify the scanner id when using\n");
  	printf("       a scanner system configuration file and -i for the input file\n");
  	PrintUsage();
  	exit(0);
  }
  if (!args->Contains("-sys") && args->Contains("-scanner")) {
  	printf("Warning: -scanner ignored as no scanner system configuration file is specified.\n");
  }
  if (args->Contains("-scanner")) {
  	if (args->Integer("-scanner", 0) < 0 || args->Integer("-scanner", 0) > 2) {
  	  printf("Error: scanner id should be 0, 1, or 2.\n");
  	  exit(0);
  	}
  }
  if (args->Contains("-sclstep")) {
  	if (args->Integer("-sclstep", 0) < 0) {
  	  printf("Error: scanline step should be greater than 0\n");
  	  exit(0);
  	}
  }
  if (args->Contains("-imu") && !args->Contains("-maxt")) {
  	printf("Error: IMU data can only be imported based on time intervals.\n");
  	printf("       Use parameter -maxt.\n");
    PrintUsage();
    exit(0);
  }

  hokuyo2laser_cpp(args->String("-i"), args->String("-i0"), 
                   args->String("-i1"), args->String("-i2"),   
                   args->String("-root"), args->String("-odir"),
                   args->Integer("-maxp", 0), args->Integer("-maxl", 0),
				   args->Double("-maxt", 0.0), args->String("-tref"),
				   args->String("-cal"), args->String("-sys"),
				   args->Integer("-scanner", -1), args->Integer("-sclstep",1),
				   args->String("-imu"));

  return EXIT_SUCCESS;
}
