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

extern "C" int intor_c(char *, char *, char *, double, double, double,
                       int, int, double, double, double, double, double);

void PrintUsage()
{
  printf("Estimation of interior orientation parameters in a scanned photograph\n");
  printf("based on measured fiducial marks. Lens distortion parameters are\n");
  printf("optional.\n\n");
  printf("Usage: extor -imgpts <Image coordinates of fiducial marks\n");
  printf("             -phtpts <Photo coordinates of fiducial marks>\n");
  printf("             -o <Interior orientation file> (output)\n");
  printf("             -cc <Camera constant (mm)>\n");
  printf("             -xh <x-coordinate of principal point>\n");
  printf("             -yh <y-coordinate of principal point>\n");
  printf("             -dim_c <Number of columns>\n");
  printf("             -dim_r <Number of rows>\n");
  printf("             [-k1 <First radial lens distortion parameter>]\n");
  printf("             [-k2 <Second radial lens distortion parameter>]\n");
  printf("             [-k3 <Third radial lens distortion parameter>]\n");
  printf("             [-p1 <First tangential lens distortion parameter>]\n");
  printf("             [-p2 <Second tangential lens distortion parameter>]\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  
// Usage

  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
// Check on required input and output parameters

  if (!args->Contains("-imgpts")) {
    printf("Error: Image points should be specified with -imgpts\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-phtpts")) {
    printf("Error: Photo points should be specified with -phtpts\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: Interior orientation file should be specified with -o\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-cc")) {
    printf("Error: Camera constant should be specified with -cc\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-xh") || !args->Contains("-yh")) {
    printf("Error: Principal point coordinates should be specified with -xh and -yh\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-dim_r") || !args->Contains("-dim_c")) {
    printf("Error: Image sizes should be specified with -dim_r and -dim_c\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

// Call the intor C routine

  intor_c(args->String("-imgpts"), args->String("-phtpts"), args->String("-o"),
          args->Double("-cc", 1.0),
          args->Double("-xh", 0.0), args->Double("-yh", 0.0),
          args->Integer("-dim_r", 1), args->Integer("-dim_c", 1),
          args->Double("-k1", 0.0), args->Double("-k2", 0.0), args->Double("-k3", 0.0),
          args->Double("-p1", 0.0), args->Double("-p2", 0.0));
  
  return EXIT_SUCCESS;
}
