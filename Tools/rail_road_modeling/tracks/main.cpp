#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: tracks -s <seed point file>\n");
  printf("              -m <meta data of block or pyramid>\n");
  printf("              [-op <output track points, def: tracks.objpts>]\n");
  printf("              [-ot <output track topology, def: tracks.top>]\n");
  printf("              [-ol <output laser point file>]\n");
  printf("              [-knn <number, def: 20>] (number of points in neighbourhood)\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void tracks_cpp(char *, char *, const char *, const char *, char *, int);

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input files
  if (!args->Contains("-s")) {
    printf("Error: no input seed point file specified with -s <filename>.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  if (!args->Contains("-m")) {
    printf("Error: no input meta data file specified with -m <filename>.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }


  // Call to main function
  tracks_cpp(args->String("-s"), args->String("-m"),
             args->String("-op", "tracks.objpts"),
             args->String("-ot", "tracks.top"),
             args->String("-ol"),
             args->Integer("-knn", 20));

  return EXIT_SUCCESS;
}
