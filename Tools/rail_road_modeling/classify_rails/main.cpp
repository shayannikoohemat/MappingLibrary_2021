/*-----------------------------------------------------------
|
|  Routine Name: main() - Provide information on header and first point of laser file
|
|       Purpose: main program for classifying rail roads and wires,
                 based on paper LS2013, Oude Elberink et al, 2013
|
|         Input:

|        Output: 
|
|    Written By: Sander Oude Elberink
|          Date: 2012-2013
| Modifications:
|
------------------------------------------------------------*/
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"

void PrintUsage()
{
  printf("Usage: classifyrails  -il <laser points>\n");
  printf("                       -ol  <laser output points>\n");
}

using namespace std;

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void classifyrails(char *, char *, double, bool);

// Check on required input files

  if (!args->Contains("-il") ||
      !args->Contains("-ol")) {
    printf("Error: missing programme option.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

// Call the main function

  classifyrails(args->String("-il"), args->String("-ol"), 
                 args->Double("-gs", 1.0), args->Contains("-doransac"));

  return EXIT_SUCCESS;
}
