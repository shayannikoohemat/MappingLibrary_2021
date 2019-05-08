
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
|  Routine Name: main() - Creation of meta data files of a laser altimetry block
|
|       Purpose: main program for createblock
|
|         Input:
|		char *clui_info->f_string; {File filter of point files or meta data files}
|		int   clui_info->f_flag; {TRUE if -f specified}
|
|		char *clui_info->name_string; {string}
|		int   clui_info->name_flag; {TRUE if -name specified}
|
|		char *clui_info->b_file; {File name of block meta data}
|		int   clui_info->b_flag; {TRUE if -b specified}
|
|		char *clui_info->dir_string; {Directory for strip meta files}
|		int   clui_info->dir_flag; {TRUE if -dir specified}
|
|        Output:
|       Returns:
|
|    Written By: 
|          Date: May 04, 1999
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
  printf("Usage: createblock -f <file filter for laser point or meta data files>\n");
  printf("                   -b <name of the block meta data file (output)>\n");
  printf("                   -name <block name>\n");
  printf("                   -dir <directory for meta data files\n");
  printf("Note that file filters and directory names are best specified between double quotes (\")\n");
}
  
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);

  void createblock_cpp(char *, char *, char *, char *);

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input files
  if (!args->Contains("-f")) {
    printf("Error: no input data specified with -f <filter>\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Create the block
  createblock_cpp(args->String("-f"), args->String("-name"),
                  args->String("-b"), args->String("-dir"));

  return EXIT_SUCCESS;
}
