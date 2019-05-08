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

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("Merge two sets of object points and topologies\n\n");
  printf("With option -l, a 1 is copied to the label of topologies of the first\n");
  printf("set of lines, and a 2 to the labels of the second set of lines.\n\n");
  printf("Usage: mergeobj -ip1 <pointset 1> -it1 <topology 1> -ip2 <pointset 2>\n");
  printf("                -it2 <topology 2> -op <merged pointset> -ot <merged topology>\n");
  printf("                [-l (flag for copying origin number (1 or 2) into label)]\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  void mergeobj_cpp(char *, char *, char *, char *, char *, char *, int);
  
// Usage

  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
// Check on required input and output files

  if (!args->Contains("-ip1") || !args->Contains("-ip2")) {
    printf("Error: input points should be specified with -ip1 <pointset 1> and -ip2 <pointset 2>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-it1") || !args->Contains("-it2")) {
    printf("Error: input topologies should be specified with -it1 <topology 1> and -it2 <topology 2>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-op") || !args->Contains("-ot")) {
    printf("Error: output data should be specified with -op and -ot\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

// Call the merge routine

  mergeobj_cpp(args->String("-ip1"), args->String("-it1"),
               args->String("-ip2"), args->String("-it2"),
               args->String("-op"), args->String("-ot"), args->Contains("-l"));
  
  return EXIT_SUCCESS;
}
