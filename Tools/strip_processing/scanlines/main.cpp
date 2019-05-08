
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
|
|  Routine Name: main() - Derive the ends of the scan lines
|
|
|    Written By: George Vosselman
|          Date: Jul 10, 2003
| Modifications:
|
------------------------------------------------------------*/

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: scanlines [-i <meta data file of strip or block>] OR\n");
  printf("                 [-f <file filter of meta data files>]\n");
  printf("                 [-hmax <maximum topography height (def: 100)]\n");
  printf("                 [-update] if meta data file is to be updated\n");
  printf("                 [-odir <output directory for scan line files>]\n");
  printf("                 [-o <scan line file name (def: auto-generated)]\n");
  printf("                 [-om <meta data output file (def: auto-generated)]\n");
  printf("                 [-fp <number of scan lines to be used for flight path (def: 0=no flight path)>]\n");
  printf("                 [-fh <approximate flight height (def: 300 m)>\n");
  
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void scanlines_cpp(char *, char *, double, int, char *,
                     char *, char *, int, double);

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input files
  if (!args->Contains("-f") && !args->Contains("-i")) {
    printf("Error: no input data specified with -f <filter> or -i <file>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (args->Contains("-f") && args->Contains("-i")) {
    printf("Error: -f and -i are mutual exclusive arguments\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (args->Contains("-o") && args->Contains("-odir")) {
    printf("Error: -o and -odir are mutual exclusive arguments\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Derive the scan lines
  scanlines_cpp(args->String("-f"), args->String("-i"),
                args->Double("-hmax", 100.0),
                args->Contains("-update"), args->String("-odir"),
                args->String("-o"), args->String("-om"),
                args->Integer("-fp", 0),
                args->Double("-fh", 300.0));

  return EXIT_SUCCESS;
}
