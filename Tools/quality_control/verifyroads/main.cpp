#include <cstdlib>
#include <iostream>
#include <stdio.h>
#include <math.h>
#include "InlineArguments.h"
#include "LaserBlock.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: verifyroads\n");
  printf("         -r <file with reference points in laser format\n");
  printf("         -p <file with road points in objpts format\n");
  printf("         -t <file with road topology in top format\n");
  printf("         -o <file with point pairs for offset estimation\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  void            VerifyRoads(const char *, const char *,
                              const char *, const char *);

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  // Verify the roads
  VerifyRoads(args->String("-r", NULL), args->String("-p", NULL),
              args->String("-t", NULL), args->String("-o", NULL));

  return EXIT_SUCCESS;
}
