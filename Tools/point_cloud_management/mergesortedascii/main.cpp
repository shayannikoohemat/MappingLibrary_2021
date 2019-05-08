
/*
                       Copyright 2016 University of Twente 
 
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
| Merge sorted ASCII files such that the combined records are also sorted
|
------------------------------------------------------------*/

#include <stdio.h>
#include "string.h"
#include <cstdlib>
#include <iostream>
#include <vector>
#include "InlineArguments.h"

#define MAXCHARS 256

using namespace std;

void PrintUsage()
{
  printf("mergesortedascii merges sorted ASCII files such that the records of\n");
  printf("the combined files are also sorted.\n");
  
  printf("Usage: mergesortedascii -i <text file with names of files to be merged>\n");
  printf("                        -o <output file>\n");
}
  
int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  vector <FILE *> input_fds;
  FILE            *file_list, *output_fd, *input_fd;
  char            *file_name, *record, *next_record;
  vector <char *> records;
  int             len, i, num_files, next_file, istart, *num_in, num_out;
  bool            done, found;

  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check on required input and output file
  if (!args->Contains("-i")) {
    printf("Error: no input data specified with -i\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-o")) {
    printf("Error: no output data specified with -o\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }

  // Read the file list and open all input files
  file_list = fopen(args->String("-i"), "r");
  if (!file_list) {
  	printf("Error opening file list %s\n", args->String("-i"));
  	exit(0);
  }
  file_name = (char *) malloc(MAXCHARS);
  while (fgets(file_name, MAXCHARS, file_list)) {
  	len = strlen(file_name);
  	if (file_name[len-1] == 10) file_name[len-1] = 0;
  	input_fd = fopen(file_name, "r");
  	if (!input_fd) {
  	  printf("Error opening input file \"%s\"\n", file_name);
  	  exit(0);
  	}
  	input_fds.push_back(input_fd);
  }
  num_files = input_fds.size();
  printf("%d input files opened\n", num_files);
  
  // Open the output file
  output_fd = fopen(args->String("-o"), "w");
  if (!output_fd) {
  	printf("Error opening output file %s\n", args->String("-o"));
  	exit(0);
  }
  
  // Create record buffers for all input files and read first records
  num_in = (int *) malloc(num_files * sizeof(int));
  for (i=0; i<num_files; i++) {
  	record = (char *) malloc(MAXCHARS);
    if (!fgets(record, MAXCHARS, input_fds[i])) {
      printf("Error reading first record from file %d\n", i);
      exit(0);
    }
  	records.push_back(record);
  	num_in[i] = 1;
  }
  
  // Process all records
  done = false;
  num_out = 0;
  while (!done) {
  	// Determine the first file with a record
  	for (i=0, found=false; i<num_files && found==false; i++) {
  	  if (records[i]) {
  	  	istart = i+1;
  	  	next_record = records[i];
  	  	next_file   = i;
  	  	found = true;
  	  }
  	}
  	if (!found) {
  	  done = true;
  	  continue;
  	}
  	
  	// Determine the alphabetically first record
  	for (i=istart; i<num_files; i++) {
  	  if (records[i]) {
  	    if (strcmp(next_record, records[i]) > 0) {
  	      next_record = records[i];
  	      next_file   = i;
  	    }
  	  }
  	}
  	
  	// Write the next record
  	fprintf(output_fd, "%s", next_record);
  	num_out++;
  	
  	// Read the next record from this file
    if (!fgets(records[next_file], MAXCHARS, input_fds[next_file])) {
      printf("At the end of file %d after reading %d records\n", next_file,
	         num_in[next_file]);
      records[next_file] = NULL;
    }
    else num_in[next_file]++;
  }
  
  printf("Merged %d records\n", num_out);
  
  return EXIT_SUCCESS;
}
