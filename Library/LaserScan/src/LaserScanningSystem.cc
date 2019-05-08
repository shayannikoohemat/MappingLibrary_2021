
/*
                  Copyright 2017 University of Twente 
 
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



/*
--------------------------------------------------------------------------------
 Collection of functions for class LaserScanningSystem

 Initial creation
 Author : George Vosselman
 Date   : 205-02-2017

 Update #1
 Author : 
 Date   : 
 Changes: 

--------------------------------------------------------------------------------
*/

/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include "LaserScanningSystem.h"
#include "BNF_io.h"
#include "stdmath.h"   /* Definition of MAX and MIN */

/*
--------------------------------------------------------------------------------
                           Copy assignment
--------------------------------------------------------------------------------
*/

LaserScanningSystem LaserScanningSystem::operator = (const LaserScanningSystem &system)
{
  if (this == &system) return *this;  // Check for self assignment
  if (!empty()) erase(begin(),end());
  if (!system.empty()) insert(begin(), system.begin(), system.end());
  StringCopy(&meta_file, system.meta_file);
  StringCopy(&name, system.name);
  return(*this);
}

/*
--------------------------------------------------------------------------------
                Initialise the scanning system information
--------------------------------------------------------------------------------
*/

void LaserScanningSystem::Initialise()
{
  meta_file = name = NULL;
}

void LaserScanningSystem::ReInitialise()
{
  if (!empty()) erase(begin(), end());
  if (meta_file) {free(meta_file);  meta_file = NULL;}
  if (name) {free(name);  name = NULL;}
}

/*
--------------------------------------------------------------------------------
                   Read scanning system information
--------------------------------------------------------------------------------
*/

int LaserScanningSystem::Read(const char *filename)
{

  char *buffer, *line, *keyword;
  int  keyword_length;
  FILE *fd;
  bool found_start=false;

  ReInitialise();
  StringCopy(&meta_file, filename);
  fd = fopen(meta_file, "r");
  if (!fd) {
  	printf("Error opening meta data file of laser scanning system %s\n",
	       meta_file);
	exit(0);
  }
  buffer = (char *) malloc(MAXCHARS);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
        if (!strncmp(keyword, "laserscanningsystem", MAX(keyword_length, 19))) {
          found_start = true;
          continue;
        }
        if (!found_start) continue;

        if (!strncmp(keyword, "name", MAX(keyword_length, 4)))
          name = BNF_String(line);

        else if (!strncmp(keyword, "laserscanner", MAX(keyword_length, 12))) {
          push_back(LaserScanner());
          (end()-1)->Read(fd);
        }

        else if (!strncmp(keyword, "endlaserscanningsystem", MAX(keyword_length, 22))) {
          free(buffer);
          fclose(fd);
          return 1;
      }
      else
        fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
      }
    }
  }
  if (!found_start)
    fprintf(stderr, "Error: Did not find keyword laserscanningsystem.\n");
  else
    fprintf(stderr, "Error: Did not find keyword endlaserscanningsystem.\n");
  free(buffer);
  fclose(fd);
}


/*
--------------------------------------------------------------------------------
                   Write scanning system information to file
--------------------------------------------------------------------------------
*/

int LaserScanningSystem::Write(const char *filename)
{
  StringCopy(&meta_file, filename);
  return(Write());
}

int LaserScanningSystem::Write() const
{
  FILE *fd;
  int  indent = 0;
  LaserScanningSystem::const_iterator scanner;
  
  // Open output file
  if (!(fd = fopen(meta_file, "w"))) {
  	printf("Error opening laser scanning system meta file %s\n", meta_file);
  	exit(0);
  }

  // Write scanner data
  BNF_Write_String(fd, "laserscanningsystem", indent, NULL);
  if (name) BNF_Write_String(fd, "name", indent+2, name);
  for (scanner=begin(); scanner!=end(); scanner++)
    scanner->Write(fd, indent+2);
  BNF_Write_String(fd, "endlaserscanningsystem", indent, NULL);
}
