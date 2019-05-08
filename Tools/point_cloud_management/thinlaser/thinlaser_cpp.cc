
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
 Thinning of a laser altimetry data set 

 Initial creation:
 Author : George Vosselman
 Date   : 06-07-1999

*/


/*
--------------------------------------------------------------------------------
                               Include files
--------------------------------------------------------------------------------
*/

#include <stdio.h>
#include <time.h>
#include "LaserBlock.h"
#include "BNF_io.h"

/*
--------------------------------------------------------------------------------
                         Declaration of external functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(const char *, const char *, int *);

extern void timer_start(clock_t *);
extern void timer_end(clock_t, char *);

/*
--------------------------------------------------------------------------------
                         The main thinlaser function
--------------------------------------------------------------------------------
*/

void thinlaser_cpp(char *in_file, char *out_file,
                   char *out_meta_file, int method,
                   double min_distance, int point_reduction_factor,
                   int start_point_offset, int scanline_reduction_factor,
                   double knn_reduction_factor, int knn_start, int knn_max,
				   char *appendix, char *output_directory, bool overwrite,
				   bool output_meta_data)
{
  LaserPoints              points;
  LaserUnit                strip;
  LaserUnit::iterator      partptr;
  int                      filetype, first_number;
  clock_t                  timer;
  void                     MLS_preprocessing(LaserPoints &, double);

  char                 *directory, *filter, *filename, *newname;
  int                  icon, fileclass;
  LaserBlock           block;
  LaserBlock::iterator unitptr;
  LaserUnit::iterator  subunitptr;
  LaserSubUnit         newsubunit;


  // Check the file type
  filetype = BNF_LaserFileType(in_file);
  switch (filetype) {

/*
--------------------------------------------------------------------------------
                          Thinning of a point set
--------------------------------------------------------------------------------
*/

    case LASER_RAW_DATA:

      // Read the laser data
      if (!points.Read(in_file, false)) {
        fprintf(stderr, "Error reading laser data from file %s\n", in_file);
        exit(0);
      }

      // Reduce the data
      timer_start(&timer);
      switch (method) {
        case 1: // Reduction based on point spacing in 2D
          points.DeriveTIN();
          points.ReduceData(min_distance);
          break;
        case 2: // Reduction based on point spacing in 3D, use knn=10
          points.ReduceData(min_distance, 10);
          break;
        case 3: // Reduction by taking every n'th point
          points.ReduceData(point_reduction_factor, 0, start_point_offset);
          break;
        case 4: // Reduction by randomly selecting from every set of n points
          points.ReduceData(point_reduction_factor, 1);
          break;
        case 5: // Reduction by taking every n'th point using knn
          points.ReduceData(knn_start, knn_max, knn_reduction_factor);
          break;
        case 6: // Reduction using scan line information
        case 7:
        case 8:
          printf("Error: Data reduction using scan lines needs strip meta data with scan line information as input\n");
          exit(0);
        case 9: // MLS preprocessing
          MLS_preprocessing(points, min_distance);
          break;
      }
      timer_end(timer, (char *) "thinning");
      
      // Write the output data. Optionally, also create a meta data file.
      points.SetPointFile(out_file);
      points.Write(false);
      if (out_meta_file) points.WriteMetaData(out_meta_file);
      
      break;
      
/*
--------------------------------------------------------------------------------
                            Thinning of a strip
--------------------------------------------------------------------------------
*/

    case LASER_STRIP:
 
      // Read the strip data, including the scan line information
      if (!strip.ReadMetaData(in_file)) {
        fprintf(stderr, "Error reading strip meta data from file %s\n",in_file);
        exit(0);
      }
      if (method >= 6) {
        if (strip.LaserScanLinesReference().ScanLinesFile() == NULL) {
          fprintf(stderr, "Error: the strip meta data file does not contain a reference to the scan line information file.\n");
          exit(0);
        }
        else if (!strip.LaserScanLinesReference().Read()) {
          fprintf(stderr, "Error reading scan line information from file %s\n",
                  strip.LaserScanLinesReference().ScanLinesFile());
        }
      }

      // Thin the strip parts
      first_number = 0;
      for (partptr=strip.begin(); partptr!=strip.end(); partptr++) {

        // Read the strip part point data
        if (!partptr->Read(partptr->PointFile(), false)) {
          fprintf(stderr, "Error reading laser data from file %s\n",
                  partptr->PointFile());
          exit(0);
        }

        // Thin the strip part data
        switch (method) {
          case 1: // Reduction based on point spacing in 2D
            partptr->DeriveTIN();
            partptr->LaserPoints::ReduceData(min_distance);
            break;
          case 2: // Reduction based on point spacing in 3D, use knn=10
            partptr->LaserPoints::ReduceData(min_distance, 10);
            break;
          case 3: // Reduction by taking every n'th point
            partptr->LaserPoints::ReduceData(point_reduction_factor, 0, start_point_offset);
            break;
          case 4: // Reduction by randomly selecting from every set of n points
            partptr->LaserPoints::ReduceData(point_reduction_factor, 1);
            break;
          case 5: // Reduction by taking every n'th point using knn
            partptr->LaserPoints::ReduceData(knn_start, knn_max, knn_reduction_factor);
            break;
          case 6: // Reduction using scan line information
          case 7:
          case 8:
            partptr->ReduceData(strip.LaserScanLinesReference(), first_number,
                                point_reduction_factor, (method != 6), 
                                scanline_reduction_factor, (method == 8));
            break;
          case 9: // MLS specific
            printf("Error: MLS scanning is only available on point data files, not on strips\n");
            exit(0);
        }
        first_number += partptr->size();

        // Copy the points to the set of thinned points
        points.insert(points.begin(), partptr->begin(), partptr->end());

        // Delete the read points
        partptr->ErasePoints();
      }

      // Output of the thinned points and their meta data
      points.Scanner().SetPointType(strip.begin()->Scanner().PointType());
      points.SetPointFile(out_file);
      points.Write(false);
      if (out_meta_file) {
        if (strip.size() > 1) strip.erase(strip.begin()+1, strip.end());
        strip.begin()->SetAsCompleteStrip();
        strip.SetPointFile(out_file);
        strip.LaserScanLinesReference().SetScanLinesFile(NULL);
        strip.WriteMetaData(out_meta_file);
      }
      break;


/*
--------------------------------------------------------------------------------
                          Thinning of a laser block
--------------------------------------------------------------------------------
*/

    case LASER_BLOCK:

      // Set up the file filter for the input file(s)
      filter = in_file;
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
            printf("Processing %s\r", subunitptr->Name()); fflush(stdout);

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

              // Thin the data
          
              switch (method) {
                case 1: // Reduction based on point spacing in 2D
                  subunitptr->DeriveTIN();
                  subunitptr->LaserPoints::ReduceData(min_distance);
                  break;
                case 2: // Reduction based on point spacing in 3D, use knn=10
                  subunitptr->LaserPoints::ReduceData(min_distance, 10);
                  break;
                case 3: // Reduction by taking every n'th point
                  subunitptr->LaserPoints::ReduceData(point_reduction_factor, 0, start_point_offset);
                  break;
                case 4: // Reduction by randomly selecting from every set of n points
                  subunitptr->LaserPoints::ReduceData(point_reduction_factor, 1);
                  break;
                case 5: // Reduction by taking every n'th point using knn
                  subunitptr->LaserPoints::ReduceData(knn_start, knn_max, knn_reduction_factor);
                  break;
                case 6: // Reduction using scan line information
                case 7:
                case 8:
                  printf("Error: Data reduction using scan lines needs strip meta data with scan line information as input\n");
                  exit(0);
                case 9: // MLS preprocessing
                  MLS_preprocessing(*subunitptr, min_distance);
                  break;
              }
          

              // Write the points and meta data
              subunitptr->Write(subunitptr->PointFile(), 0, false);
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
      break;

/*
--------------------------------------------------------------------------------
                            No other file types
--------------------------------------------------------------------------------
*/

    default:
      fprintf(stderr, "Error: input should be a file with strip or block meta data or point data\n");

  } /* End of file type switch */
}

void MLS_preprocessing(LaserPoints &points, double min_distance)
{
  LaserPoints::iterator       point1, point2, point;
  bool                        done=false;
  int                         count;
  double                      height_diff, sum;
  std::vector<LaserPoints>    clusters;
  std::vector<LaserPoints>::iterator cluster;
  LaserPoints                 new_cluster;
  
  // Check there is something to thin out
  if (points.size() < 2) return;
  
  // Sort points on XYZ coordinates
  points.SortOnCoordinates();
  
  // Set all points as unfiltered
  points.SetUnFiltered();
  
  point1 = points.begin();
  while (!done) {
    // Find sequence with the same XY coordinates
    point2 = point1+1;
    count = 0;
    while (!done) {
      if (point2 == points.end()) done = true;
      else if (fabs(point2->X() - point1->X()) < 0.001 &&
               fabs(point2->Y() - point1->Y()) < 0.001) {
        count++, point2++;
      }
      else done = true;
    }
    // Process sequences of same XY coordinates
    if (count) {
      // Put points into clusters
      for (point=point1; point!=point2; point++) {
        for (cluster=clusters.begin(), done=false;
             cluster!=clusters.end() && !done; cluster++) {
          height_diff = cluster->begin()->Z() - point->Z();
          if (fabs(height_diff) < min_distance) {
            cluster->push_back(*point);
            done = true;
          }
        }
        // Create new cluster if needed
        if (!done) {
          new_cluster.push_back(*point);
          clusters.push_back(new_cluster);
          new_cluster.ErasePoints();
        }
      }
      // Calculate average height for each cluster
      for (cluster=clusters.begin(); cluster!=clusters.end(); cluster++) {
        // Calculate height sum
        sum = 0.0;
        for (point=cluster->begin(); point!=cluster->end(); point++)
          sum += point->Z();
        // Store average heights in the first couple of points
        point1->Z() = sum / cluster->size();
        point1++;
        // Clear cluster
        cluster->ErasePoints();
      }
      // Set the remaining points to filtered
      for (; point1!=point2; point1++) point1->SetFiltered();
      // Clear clusters
      clusters.erase(clusters.begin(), clusters.end());
    }
    // Prepare for next sequence
    if (point2 != points.end()) {
      done = false;
      point1 = point2;
    }
  }
  
  // Delete all marked points
  points.RemoveTaggedPoints(1, IsFilteredTag);
  
  // Remove tag from all remaining points
  points.RemoveAttribute(IsFilteredTag);
}
