
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
#include "InlineArguments.h"
#include "LaserBlock.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: blockoverlaps -i <block meta file>\n");
  printf("         -g <grid size of coverage image>\n");
  printf("         -max <maximum number of points in an overlap part\n");
  printf("         -odir <output directory>\n");
  printf("         -sn <strip number output text file>\n");
}

int main(int argc, char *argv[])
{
  InlineArguments      *args = new InlineArguments(argc, argv);
  LaserBlock           block, overlaps;
  LaserBlock::iterator strip1, strip2;
  LaserUnit            overlap;
  char                 *name;
  FILE                 *fd;

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i")) {
    printf("Error: -i is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  // Open the block
  if (!block.ReadMetaData(args->String("-i"), false, false)) {
    printf("Error reading block: %s\n", args->String("-i"));
    exit(0);
  }
  if (block.size() < 2) {
    printf("Error: no two strips in this block\n");
    exit(0);
  }

  // Loop over the strips
  strip1 = block.begin();
  if (!strip1->ReadMetaData(strip1->MetaDataFile())) {
    printf("Error reading strip meta data from %s\n", strip1->MetaDataFile());
    exit(0);
  }
  for (strip1=block.begin(), strip2=strip1+1; strip2!=block.end();
       strip1++, strip2++) {
    if (!strip2->ReadMetaData(strip2->MetaDataFile())) {
      printf("Error reading strip meta data from %s\n", strip2->MetaDataFile());
      exit(0);
    }
    if (strip1->size() && strip2->size()) {
      if (strip1->begin()->PointFile() && strip2->begin()->PointFile()) {
        printf("Extracting overlap between strip %s and strip %s ...",
               strip1->Name(), strip2->Name());
        fflush(stdout);
        // Construct the overlap
        overlap = strip1->Overlap(*strip2, args->Double("-g", 10.0),
                                  args->Integer("-max", 500000),
                                  args->String("-odir"));
        overlaps.push_back(overlap);
        strip1->erase(strip1->begin(), strip1->end()); // Meta data no longer needed
      }
    }
  }
  
  // Write the block meta data file
  name = (char *) malloc(strlen(block.Name()) + 10);
  sprintf(name, "%s_overlaps", block.Name());
  overlaps.SetName(name);
  free(name);
  overlaps.DeriveMetaDataFileName(args->String("-odir"));
  overlaps.WriteMetaData(false, false);
  
  // Write the strip number file
  if (args->Contains("-sn")) {
    fd = fopen(args->String("-sn", "strip_numbers.txt"), "w");
    if (fd == NULL) {
      printf("Error opening strip number file %d\n",
             args->String("-sn", "strip_numbers.txt"));
      exit(0);
    }
    for (strip1=block.begin(); strip1!=block.end(); strip1++)
      fprintf(fd, "%4d  %s\n", strip1->StripNumber(), strip1->Name());
    fclose(fd);
  }
   
  return EXIT_SUCCESS;
}
