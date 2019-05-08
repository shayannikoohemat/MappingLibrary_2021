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
|  Routine Name: main() - 
|
|       Purpose: detect points on road markings in MLS data
|
|         Input:
|		char *clui_info->il_file; {Laser point file}
|		int   clui_info->il_flag; {TRUE if -il specified}
|       
|       char *clui_info->o_file; {Laser point file}
|		int   clui_info->o_flag; {TRUE if -ol specified}

|        Output: 
|       Returns: laser point file
|
|    Initiated By: Sander Oude Elberink
|          Date: April 17, 2012
| Modifications:
|
------------------------------------------------------------*/
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include "InlineArguments.h"

void PrintUsage()
{
  printf("Usage: detectroadmarkings  -il <laser points>\n");
  printf("                       -ol  <laser output points>\n");
}

using namespace std;

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void detectroadmarkings(char *, char *);

// Check on required input files

  if (!args->Contains("-il") ||
      !args->Contains("-ol")) {
    printf("Error: missing programme option.\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

// Call the main function

  detectroadmarkings(args->String("-il"), args->String("-ol"));

  return EXIT_SUCCESS;
}
