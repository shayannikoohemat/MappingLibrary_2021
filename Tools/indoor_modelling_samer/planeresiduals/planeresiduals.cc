
/*
    Copyright 2019 University of Twente
 
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
 Calculation of residuals of plane fitting, based on a block of laser points
 with plane number attributes and a set of planes

 Initial creation:
 Author : George Vosselman
 Date   : 07-02-2019

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
#include <malloc.h>
#include "LaserBlock.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(const char *, const char *, int *);

/*
--------------------------------------------------------------------------------
                         The main unlabelsurface function
--------------------------------------------------------------------------------
*/

void planeresiduals(char *long_filter, char *infile,
                    char *appendix, char *output_directory,
                    bool output_meta_data, bool overwrite,
                    char *plane_file, char *statistics_file)
{
  char                  *directory, *filter, *filename, *newname;
  int                   icon, fileclass, plane_number, num_unique, *count;
  double                residual, *sum, *sqsum, average, stdev;
  LaserBlock            block;
  LaserBlock::iterator  unitptr;
  LaserUnit::iterator   subunitptr;
  LaserSubUnit          newsubunit;
  LaserPoints::iterator point;
  Planes                planes;
  Planes::iterator      plane;
  FILE                  *stats_fd;

  // Read the planes
  if (!planes.Read(plane_file)) {
  	printf("Error reading planes from file %s\n", plane_file);
  	exit(0);
  }
  printf("Read %d planes\n", planes.size());
  
  // Sort the planes and check the numbers
  planes.Sort();
  planes.Write("atest_sorted.planes");
  num_unique = 1;
  for (plane=planes.begin()+1; plane!=planes.end(); plane++)
    if (plane->Number() != (plane-1)->Number()) num_unique++;
  printf("%d plane numbers in range %d-%d, %d unique numbers\n", planes.size(),
         planes.begin()->Number(), (planes.end()-1)->Number(), num_unique);
         
  planes.Erase();
  planes.Read(plane_file);
  
  // Allocate counter arrays
  count = (int *)    calloc(planes.size(), sizeof(int));
  sum   = (double *) calloc(planes.size(), sizeof(double));
  sqsum = (double *) calloc(planes.size(), sizeof(double));

  // Set up the file filter for the input file(s)
  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

  // Collect all points from all input files
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    // Set up a laser block
    if (!block.Create(filename, &fileclass)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }

    // Loop over all units and subunits
    for (unitptr=block.begin(); unitptr!=block.end(); unitptr++) {
      for (subunitptr=unitptr->begin(); subunitptr!=unitptr->end();
	       subunitptr++) {

        // Derive the names of the output files. Note that the variable
        // newsubunit is only used for checking the existance of files. New file 
        // names are later transfered to subunitptr.
        if (!subunitptr->Name()) subunitptr->LaserDataFiles::DeriveName();
        printf("Part %s\r", subunitptr->Name()); fflush(stdout);
        if (appendix) {
          newname = (char *) malloc(strlen(subunitptr->Name()) +
                                    strlen(appendix) + 1);
          sprintf(newname, "%s%s", subunitptr->Name(), appendix);
        }
        else {
          newname = (char *) malloc(strlen(subunitptr->Name()) + 1);
          sprintf(newname, "%s", subunitptr->Name());
        }
        newsubunit.SetName(newname);
        newsubunit.DataOrganisation() = subunitptr->DataOrganisation();
        newsubunit.DeriveMetaDataFileName(output_directory);
        newsubunit.DerivePointFileName(output_directory);
        free(newname);

        // Do not go ahead if the point file and meta file already exists and 
        // overwriting these files is not permitted.
        if (overwrite || !BNF_FileExists(newsubunit.MetaDataFile()) ||
            !BNF_FileExists(newsubunit.PointFile())) {
        
          // Read the point data
          if (!subunitptr->Read()) {
            fprintf(stderr, "Error reading laser points from file %s\n",
                    subunitptr->PointFile());
            exit(0);
          }

          // After reading all data, the file names can be overwritten with the
          // names of the output files.
          subunitptr->SetName(newsubunit.Name());
          subunitptr->DeriveMetaDataFileName(output_directory);
          subunitptr->DerivePointFileName(output_directory);
          
          // Remove seek offset as all tiles are written separately
          subunitptr->SetSeekOffset(0);

          // Set residual value for every point
          for (point=subunitptr->begin(); point!=subunitptr->end(); point++) {
            if (point->HasAttribute(PlaneNumberTag)) {
              plane_number = point->Attribute(PlaneNumberTag);
              if (plane_number != planes[plane_number].Number())  {
              	plane_number = planes[plane_number].Number();
              	point->Attribute(PlaneNumberTag) = plane_number;
              }
              if (plane_number != planes[plane_number].Number()) {
               	printf("Mismatch in plane numbers: %d %d\n",
				       plane_number, planes[plane_number].Number());
//				exit(0);
              }
              if (plane_number < 0) {
              	printf("Error: negative plane number %d\n", plane_number);
              	exit(0);
              }
              if (plane_number >= planes.size()) {
              	printf("Error: plane number above %d: %d\n", planes.size()-1,
				       plane_number);
				exit(0);
              }
              residual = planes[plane_number].Distance(point->Position3DRef());
              point->FloatAttribute(ResidualTag) = residual;
              count[plane_number]++;
              sum[plane_number] += residual;
              sqsum[plane_number] += residual * residual;
            }
          }

          // Write the points and meta data
          subunitptr->Write(false);
          if (output_meta_data) subunitptr->WriteMetaData();

          // Erase the points
          subunitptr->ErasePoints();
        }
        else {
          printf("Sub unit %s was already done.\n", newsubunit.Name());
          if (!subunitptr->ReadMetaData(newsubunit.MetaDataFile())) {
            fprintf(stderr, "Error reading meta data from file %s\n",
                    newsubunit.MetaDataFile());
            exit(0);
          }
        }
      }

      // Output of meta data at unit level
      if (output_meta_data &&
          (fileclass == LASER_STRIP || fileclass == LASER_BLOCK) &&
          unitptr->DataOrganisation() & StripWise) {
        if (!unitptr->Name()) unitptr->LaserDataFiles::DeriveName();
        if (appendix) {
          newname = (char *) malloc(strlen(unitptr->Name()) +
                                    strlen(appendix) + 1);
          sprintf(newname, "%s%s", unitptr->Name(), appendix);
        }
        else {
          newname = (char *) malloc(strlen(unitptr->Name()) + 1);
          sprintf(newname, "%s", unitptr->Name());
        }
        unitptr->SetName(newname);
        free(newname);
        unitptr->DeriveMetaDataFileName(output_directory);
        if (unitptr->begin()->IsCompleteStrip())
          unitptr->DerivePointFileName(output_directory);
        else unitptr->SetPointFile(NULL);
        unitptr->WriteMetaData();
      }
    }

    // Output of meta data at block level
    if (output_meta_data && fileclass == LASER_BLOCK) {
      if (!block.Name()) block.DeriveName();
      if (appendix) {
        newname = (char *) malloc(strlen(block.Name()) +
                                  strlen(appendix) + 1);
        sprintf(newname, "%s%s", block.Name(), appendix);
      }
      else {
        newname = (char *) malloc(strlen(block.Name()) + 1);
        sprintf(newname, "%s", block.Name());
      }
      block.SetName(newname);
      free(newname);
      block.DeriveMetaDataFileName(output_directory);
      block.WriteMetaData();
    }
  }
  printf("\n");
  
  // Output of statistics per plane
  stats_fd = stdout;
  if (statistics_file) {
  	stats_fd = fopen(statistics_file, "w");
  	if (stats_fd == NULL) {
  	  printf("Error opening statistics file %s\n", statistics_file);
  	  stats_fd = stdout;
  	}
  }
  for (plane_number=0; plane_number<planes.size(); plane_number++) {
  	if (count[plane_number]) {
  	  average = sum[plane_number] / count[plane_number];
  	  stdev   = sqrt((sqsum[plane_number] - 
		              count[plane_number] * average * average)
					  / count[plane_number]);
	  fprintf(stats_fd, "%6d %11d %7.4f %7.4f\n", plane_number, count[plane_number],
	          average, stdev);
	          
	  // Add sums to plane 0
	  if (plane_number > 0) {
	    count[0] += count[plane_number];
	    sum[0]   += sum[plane_number];
	    sqsum[0] += sqsum[plane_number];
	  }
  	}
  }
  average = sum[0] / count[0];
  stdev   = sqrt((sqsum[0] - count[0] * average * average) / count[0]);
  fprintf(stats_fd, "\noverall %10d %7.4f %7.4f\n", count[0],
	      average, stdev);
  if (stats_fd != stdout) fclose(stats_fd);
}
