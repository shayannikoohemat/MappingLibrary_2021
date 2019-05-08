
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
#include <stdio.h>
#include "InlineArguments.h"

using namespace std;

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  
  void createkernel_cpp(double, double, double, double, double, double,
                        double, double, double, double, double, double, 
                        double, double, char *);
                        
  if (!args->Contains("-dmax") || !args->Contains("-hmax") ||
      !args->Contains("-int") || !args->Contains("-o") ||
      args->Contains("-usage")) {
    if (!args->Contains("-usage")) printf("Error: Incomplete argument list\n");
    system("C:\\cygwin\\bin\\man createkernel");
    exit(0);
  }
                       
  createkernel_cpp(args->Double("-h0", 0.0),
                   args->Double("-d1", -1.0), args->Double("-h1", 0.0),
                   args->Double("-d2", -1.0), args->Double("-h2", 0.0),
                   args->Double("-d3", -1.0), args->Double("-h3", 0.0),
                   args->Double("-d4", -1.0), args->Double("-h4", 0.0),
                   args->Double("-d5", -1.0), args->Double("-h5", 0.0),
                   args->Double("-dmax", 0.0), args->Double("-hmax", 0.0),
                   args->Double("-int", 0.0), args->String("-o"));
                   
  return EXIT_SUCCESS;
}
