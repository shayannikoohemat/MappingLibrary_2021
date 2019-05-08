
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


#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"
#include "LaserUnit.h"

using namespace std;

void PrintUsage()
{
  printf("sortstrip sorts the points of a strip along the strip's axis.\n");
  printf("If a strip is distributed over two strip files, both can be used\n");
  printf("as input to generate one output file\n\n");

  printf("Usage: sortstrip -i <strip meta data file>\n");
  printf("                 [-i2 <meta data file of a second strip part>]\n");
  printf("                 -root <root name of the output strip>\n");
  printf("                 -odir <output directory>\n");
  printf("                 -wdir <working directory>\n");
  printf("                 [-v] (output progress)\n");
  printf("                 [-s] (keep scalar tag after sorting)\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  LaserUnit       strip, strip2;

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i")) {
    printf("Error: -i is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-root")) {
    printf("Error: -root is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-odir")) {
    printf("Error: -odir is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-wdir")) {
    printf("Error: -wdir is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  // Read the meta data
  if (!strip.ReadMetaData(args->String("-i"))) {
    printf("Error reading input meta data from %s\n", args->String("-i"));
    exit(0);
  }
  
  // Read meta data of a second strip part and insert it into the first strip
  if (args->Contains("-i2")) {
    if (!strip2.ReadMetaData(args->String("-i2"))) {
      printf("Error reading input meta data from %s\n",
             args->String("-i2"));
      exit(0);
    }
    if (strip2.size())
      strip.insert(strip.end(), strip2.begin(), strip2.end());
  }
  
  // Sort the strip
  strip.SortAlongStripAxis(args->String("-root"), args->String("-odir"),
                           args->String("-wdir"), args->Contains("-v"),
                           !args->Contains("-s")); 

  return EXIT_SUCCESS;
}
