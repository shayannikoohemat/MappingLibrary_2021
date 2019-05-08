/*-----------------------------------------------------------

Construction of buildings in 3D Topography from 2D Topography and laser
scanner data

------------------------------------------------------------*/

#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"
#include "dxf.h"
using namespace std;


void PrintUsage()
{
  printf("Usage: Make3DBuildings -il <laser points>\n");
  printf("                       -ip <input map points (2D object points)>\n");
  printf("                       -it <input map topology>\n");
  printf("                       -op <output points (3D object points)>\n");
  printf("                       -ot <output topology>\n");
  printf("                       -ol <output building laser file>\n");
  printf("                       -ol2 <output nonbuilding laser file>\n");
  printf("                       -info <output information file>\n");
  printf("                       [-mss <minimum segment size, # points (default: 50)>]\n");
  printf("                       [-mll <minimum line length, in m (default: 0.5 m)>]\n");
  }

int main(int argc, char *argv[])
{
    InlineArguments *args = new InlineArguments(argc, argv);

  void Make3DBuildings(char *, char *, char *, char *, char *, char *, char *, char*, int, double);

  // Check on required input files
  if (args->Contains("-usage") ||
      !args->Contains("-il") ||
      !args->Contains("-ip") ||
      !args->Contains("-it") ||
      !args->Contains("-op") || 
      !args->Contains("-ot") ||
      !args->Contains("-ol") ||
      !args->Contains("-ol2") ||
      !args->Contains("-info")) { 
    if (!args->Contains("-usage")) printf("Error: missing programme option.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Call the main function
  Make3DBuildings(args->String("-il"), 
            args->String("-ip"), args->String("-it"),
            args->String("-op"), args->String("-ot"), 
            args->String("-ol"), args->String("-ol2"), args->String("-info"), args->Integer("-mss", 50),
            args->Double("-mll", 0.5));
  return EXIT_SUCCESS;
}
