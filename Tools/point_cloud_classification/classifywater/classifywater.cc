
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
 Classification of water in laser altimetry data.
 Files can be specified explicitly or by a file filter.
 The classified points are written to file. Optionally, a meta data file is
 generated.

 Initial creation:
 Author : George Vosselman
 Date   : 18-03-2011

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
#include <time.h>
#include "LaserBlock.h"
#include "TINEdges.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                         Declaration of external functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(const char *, const char *, int *);
extern int Compare_Double(const void *, const void *);
extern void timer_start(clock_t *);
extern void timer_end(clock_t, char *);

/*
--------------------------------------------------------------------------------
                         The main segmentlaser function
--------------------------------------------------------------------------------
*/

void classifywater(char *long_filter, char *infile,
                   char *appendix, char *output_directory,
                   bool output_meta_data, bool overwrite,
                   const SegmentationParameters &parameters)
{
  char                 *directory, *filter, *filename, *newname;
  int                  icon, fileclass, *segment_size, num_segments, segment;
  double               *distance_sum, distance;
  LaserBlock           block;
  LaserBlock::iterator unitptr;
  LaserUnit::iterator  subunitptr;
  LaserPoints::iterator point, nb_point;
  LaserSubUnit         newsubunit;
  clock_t              start;
  Planes               planes;
  Planes::iterator     plane;
  TINEdges             *edges;
  TINEdges::iterator   edgeset;
  PointNumberList::iterator node;
  vector <vector <double> > all_distances;
  vector <vector <double> >::iterator segment_distances;
  vector <double> distance_vector;
  FILE *fd;
  
  double minimum_point_spacing = 0.3;
  int    minimum_segment_size = 50;
  double minimum_Z_component = 0.95;
  double minimum_distance_difference = 0.20;

  fd = fopen("distances.txt", "w");
  
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
        if (overwrite || !FileExists(newsubunit.MetaDataFile()) ||
            !FileExists(newsubunit.PointFile())) {
        
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

          // Clear data of previous iteration
          planes.Erase();
          if (segment_size) free(segment_size);
          if (distance_sum) free(distance_sum);
          for (segment_distances=all_distances.begin();
               segment_distances!=all_distances.end(); segment_distances++)
            segment_distances->erase(segment_distances->begin(), segment_distances->end());
          all_distances.erase(all_distances.begin(), all_distances.end());

          // Segment the data
          timer_start(&start);
          subunitptr->SurfaceGrowing(parameters, false, true, &planes);
          timer_end(start, (char *) "segmenting");
          
          // For every segment, determine the mean distance to the nearest neighbour
          num_segments = planes.size();
          segment_size = (int *) calloc(num_segments, sizeof(int));
          distance_sum = (double *) calloc(num_segments, sizeof(double));
          for (segment=0; segment<num_segments; segment++)
            all_distances.push_back(distance_vector);
          edges = subunitptr->GetNeighbourhoodEdges();
          for (point=subunitptr->begin(), edgeset=edges->begin();
               point!=subunitptr->end(); edgeset++, point++) {
            if (point->HasAttribute(SegmentNumberTag)) {
              segment = point->Attribute(SegmentNumberTag);
              node = edgeset->begin()+10;
              nb_point = subunitptr->begin() + node->Number();
              distance = point->Distance(nb_point->Position3DRef());
              all_distances[segment].push_back(distance);
              segment_size[segment]++;
              distance_sum[segment] += distance;
            }
          }
          
          // Check segments on
          // 1) size
          // 2) slope
          // 3) average distance to nearest neighbouring point
          // Set size of selected water segment to -1
          for (segment=0, plane=planes.begin(), segment_distances=all_distances.begin();
               segment<num_segments;
               segment++, plane++, segment_distances++) {
            // Sort segment distances if at least minimum_segment_size
            if (segment_size[segment] > minimum_segment_size) {
              qsort((void *) &*(segment_distances->begin()), segment_distances->size(),
                    sizeof(double), Compare_Double);
              printf("Segment %3d size %5d  normal %.2f  distance %.2f\n",
                     segment, segment_size[segment], fabs(plane->Normal().Z()),
                     distance_sum[segment] / segment_size[segment]);
              printf("  10 %.2f  25 %.2f  50 %.2f  75 %.2f  90 %.2f  67-33 %.2f 67-33/size %.2f\n",
                     *(segment_distances->begin() + (int) (0.10 * segment_distances->size())),
                     *(segment_distances->begin() + (int) (0.25 * segment_distances->size())),
                     *(segment_distances->begin() + (int) (0.50 * segment_distances->size())),
                     *(segment_distances->begin() + (int) (0.75 * segment_distances->size())),
                     *(segment_distances->begin() + (int) (0.90 * segment_distances->size())),
                     
                     *(segment_distances->begin() + (int) (0.67 * segment_distances->size())) -
                     *(segment_distances->begin() + (int) (0.33 * segment_distances->size())),
                     
                     (*(segment_distances->begin() + (int) (0.67 * segment_distances->size())) -
                      *(segment_distances->begin() + (int) (0.33 * segment_distances->size()))) /
                     (distance_sum[segment] / segment_size[segment])
                     
                     );
              fprintf(fd, "%3d %5d %5.2f d ", segment, segment_size[segment], fabs(plane->Normal().Z()));
              for (float perc=0.0; perc<0.96; perc+=0.05)
                fprintf(fd, "%6.2f ", *(segment_distances->begin() + (int) (perc * segment_distances->size())));
              fprintf(fd, "%6.2f ", *(segment_distances->begin() + segment_distances->size() - 1));
              fprintf(fd, "\n");
            }


            if (segment_size[segment] > minimum_segment_size &&
                fabs(plane->Normal().Z()) > minimum_Z_component &&
                *(segment_distances->begin() + (int) (0.67 * segment_distances->size())) -
                *(segment_distances->begin() + (int) (0.33 * segment_distances->size())) > 
                minimum_distance_difference)
              segment_size[segment] = -1;
          }

          // Label all points with 2 (water) or 1 (other)
          for (point=subunitptr->begin(); point!=subunitptr->end(); point++) {
            point->Label(1);
            if (point->HasAttribute(SegmentNumberTag))
              if (segment_size[point->Attribute(SegmentNumberTag)] == -1)
                point->Label(2);
          }
          
          // Write the points and meta data
          subunitptr->Write();
          if (output_meta_data) subunitptr->WriteMetaData();

          // Erase the points, the TIN, and the TIN edges
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
  
  fclose(fd);
}
