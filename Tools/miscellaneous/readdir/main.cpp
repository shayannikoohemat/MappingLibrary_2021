
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
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <malloc.h>
#include "InlineArguments.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(const char *, const char *, int *);

using namespace std;

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  char *file_filter, *directory, *filename, *dirname;
  int icon, icon2;
  bool readsubdir;
  
  // Usage
  if (args->Contains("-usage")) {
    printf("readdir [-f <file filter>]\n");
    printf("        [-s] (explore one level of subdirectories\n");
    return EXIT_SUCCESS;
  }
  
  if (args->Contains("-f")) file_filter = args->String("-f");
  else file_filter = (char *) "*.*";

  // Set up the file filter for the input file(s)
  directory = (char *) malloc(strlen(file_filter));
  parse_filter(&file_filter, directory);
  icon = icon2 = 0;
  readsubdir = args->Contains("-s");

  if (readsubdir) {
    while ((dirname = get_full_filename("./", "*", &icon2)) != NULL) { // loop over all dirs
      if (DirectoryExists(dirname)) {
        icon = 0;
//        printf("Exploring directory %s\n", dirname);
        while ((filename = get_full_filename(dirname, file_filter, &icon)) != NULL) {
          printf("%s\n", filename);
        }
      }
    }    
  }
  else {
    while ((filename = get_full_filename(directory, file_filter, &icon)) != NULL) {
      printf("readdir: Context %d,  file: %s\n", icon, filename);
    }
  }
  return EXIT_SUCCESS;
}
