
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


/*
--------------------------------------------------------------------------------
 Creation of a new laser altimetry block based on a file filter of strip wise
 laser data files. Output of the program are block and strip meta data files.

 Initial creation:
 Author : george Vosselman
 Date   : 20-04-1999

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
#include "LaserBlock.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);

/*
--------------------------------------------------------------------------------
                         The main createblock function
--------------------------------------------------------------------------------
*/

void createblock_cpp(char *filter, char *name,
                     char *block_file, char *strip_directory)
{
  LaserBlock::iterator stripptr;
  char                 *directory, *dummy;

/* Create the block based on the file filter */

  LaserBlock block = LaserBlock(filter);

/* Set the block name and block meta data file name */

  block.SetName(name);
  block.SetMetaDataFile(block_file);

/* Extract the strip directory from the block file name in case no directory
 * is specified */

  if (strip_directory) directory = strip_directory;
  else {
    dummy = block_file;
    directory = (char *) malloc(MAXCHARS);
    parse_filter(&dummy, directory);
  }

/* Set the strip names and strip meta data file names */

  for (stripptr=block.begin(); stripptr!=block.end(); stripptr++) {
    stripptr->DeriveMetaDataFileName(directory);
    stripptr->DataOrganisation() = StripWise;
  }

/* Write the meta data */

  block.WriteMetaData(1, 0);
}
