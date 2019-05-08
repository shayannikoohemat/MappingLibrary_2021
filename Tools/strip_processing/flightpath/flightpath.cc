
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
 Reconstruction of flight path from laser data. The input can be
 a block file or a strip file. Files can be specified explicitly or
 by a file filter.

 Initial creation:
 Author : George Vosselman
 Date   : 26-06-2003

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
#include <math.h>
#include "LaserBlock.h"
#include "BNF_io.h"
#include "Line2D.h"
#include "Line3D.h"
#include "Histogram.h"

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(char *, char *, int *);

/*
--------------------------------------------------------------------------------
                         The main scanlines function
--------------------------------------------------------------------------------
*/

void fp_cpp(char *long_filter, char *infile,
            int method, double flight_height, 
            double fixed_angle_increment)
{
  char                        *directory, *filter, *filename, *name, *newname;
  int                         icon, fileclass, i, found, histo[1001];
  LaserBlock                  block;
  LaserBlock::iterator        stripptr;
  LaserUnit::iterator         partptr;
  LaserPoints::const_iterator first_point, last_point, point,
                              begin_point, end_point, ref_point;
  ObjectPoint                 fp_point;
  LaserScanLines              scanlines;
  LaserScanLines::iterator    scanline, full_scanline;
  int                         first_number, last_number,
                              number, strip_number, line_number, max_num_pts;
  double                      max_dist, dist, angle, pi=4.0*atan(1.0),
                              u_s, u_p; 
  Position3D                  fp_begin, fp_end, fl_pos;
  ObjectPoints                fp_end_points;
  Line3D                      flight_line;
  Line2D                      flight_line_2D;
//Line2D                      sline;
  LaserPoint                  angle_point, fl_point;
  LaserPoints                 angle_points, fl_points;
  LineTopology                fl_top;
  LineTopologies              fl_tops;

/* Set up the file filter for the input file(s) */

  if (long_filter) filter = long_filter;
  else {filter = NULL; StringCopy(&filter, infile);}
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

/* Process all input files */

  block.Initialise();
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    
/* Set up a laser block and check the type of input */

    if (!block.Create(filename, &fileclass)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }
    if (fileclass != LASER_BLOCK && fileclass != LASER_STRIP) {
      fprintf(stderr, "Error: Input file should contain meta data of block or strip.\n");
      exit(0);
    }

// Loop over all strips

    for (stripptr=block.begin(), strip_number=0;
         stripptr!=block.end();
         stripptr++, strip_number++) {

// Check the presence of the scan line file and read the scan line information.

      scanlines = stripptr->LaserScanLinesReference();
      if (!scanlines.ScanLinesFile()) {
        fprintf(stderr,
                "Error: Strip file %s contains no scan line information.\n",
                stripptr->MetaDataFile());
        fprintf(stderr, "       Use program scanlines to extract this.\n");
        exit(0);
      }
      else if (!scanlines.Read()) {
        fprintf(stderr, "Error reading scan line information from file %s\n",
                scanlines.ScanLinesFile());
        exit(0);
      }
      printf("Number of scan lines %d\n", (int) scanlines.size());

/* Read all strip parts */

      for (partptr=stripptr->begin(); partptr!=stripptr->end(); partptr++) {
        if (!partptr->Read()) {
          fprintf(stderr, "Error reading laser points from file %s\n",
                  partptr->PointFile());
          exit(0);
        }
      }

// Find the first and the last scan line with at least 90% of the points

      max_num_pts = 0;
      for (scanline=scanlines.begin(); scanline!=scanlines.end(); scanline++)
        if (scanline->NumberOfPoints() > max_num_pts)
          max_num_pts = scanline->NumberOfPoints();
      found = 0;
      for (scanline=scanlines.begin(); scanline!=scanlines.end() && !found;
           scanline++) {
        if (scanline->NumberOfPoints() > max_num_pts * 0.9) {
          found = 1;
          full_scanline = scanline;
        }
      }
      first_point = full_scanline->begin(stripptr->begin()->LaserPointsPointer(), 0);
      last_point  = full_scanline->end(stripptr->begin()->LaserPointsPointer(), 0);
      fp_begin.vect() = (first_point->vect() + last_point->vect()) / 2.0;
      fp_begin.Z()    = flight_height;
      found = 0;
      for (scanline=scanlines.end()-1; scanline!=scanlines.begin()-1 && !found;
           scanline--) {
        if (scanline->NumberOfPoints() > max_num_pts * 0.9) {
          found = 1;
          full_scanline = scanline;
        }
      }
      
      
// ERROR? This seems to assume that the last scan line is still within the first strip part!

      first_point = full_scanline->begin(stripptr->begin()->LaserPointsPointer(), 0);
      last_point  = full_scanline->end(stripptr->begin()->LaserPointsPointer(), 0);
      fp_end.vect() = (first_point->vect() + last_point->vect()) / 2.0;
      fp_end.Z()    = flight_height;

      printf("Flight path from (%10.2f, %10.2f, %6.2f)\n",
             fp_begin.X(), fp_begin.Y(), fp_begin.Z());
      printf("           until (%10.2f, %10.2f, %6.2f)\n",
             fp_end.X(), fp_end.Y(), fp_end.Z());
      fp_point.vect() = fp_begin.vect();
      fp_point.Number() = 0;
      fp_end_points.push_back(fp_point);
      fp_point.vect() = fp_end.vect();
      fp_point.Number() = 1;
      fp_end_points.push_back(fp_point);
//    fp_end_points.Write("end.objpts");
      flight_line = Line3D(fp_begin, fp_end);
      flight_line_2D = Line2D(Position2D(fp_begin.vect2D()),
                              Position2D(fp_end.vect2D()));

/* Process all scan lines */

// ERROR? partptr is never incremented. Only the first part is processed

      partptr      = stripptr->begin();
      first_number = 0;
      last_number  = partptr->size() - 1;
      for (i=0; i<=max_num_pts; i++) histo[i] = 0;
      for (line_number = 0, scanline=scanlines.begin();
           scanline!=scanlines.end();
           line_number++, scanline++) {

// Add number of points on scan line to histogram

        histo[scanline->NumberOfPoints()]++;

/* Determine first and last point of a scan line */

        first_point = scanline->begin(partptr->LaserPointsPointer(), 0);
        last_point = scanline->end(partptr->LaserPointsPointer(), 0);

// Determine scanner position

        if (scanline->ScannerPosition(partptr->LaserPointsReference(),
                                      flight_height, fl_pos, method,
                                      fixed_angle_increment)) {
          fl_point.vect() = fl_pos.vect();
          fl_points.push_back(fl_point);
//          if (line_number == 1199)
//            sline = Line2D(Position2D(first_point->vect2D()),
//                           Position2D(last_point->vect2D()));

// Determine angles

          angle_point.Y() = line_number * 0.3;
          u_s =
            flight_line_2D.DistanceToPointSigned(Position2D(fl_point.vect2D()));

          for (point=first_point; point<=last_point; point++) {
            u_p =
              flight_line_2D.DistanceToPointSigned(Position2D(point->vect2D()));
            angle = atan((u_p - u_s) / (fl_point.Z() - point->Z()));
  
            angle_point.X() = angle * 180.0 / pi;
            angle_point.Z() = point->Z();
            angle_point.Reflectance() = point->Reflectance();
            angle_points.push_back(angle_point);
/*          if (line_number == 1199) printf("%8.3f", point->Z());
            if (line_number == 1199)
              printf("%8.3f",
                     sline.DistanceToPointSigned(Position2D(point->vect2D())));
*/
          }
//        if (line_number == 1199) printf("\n");
        }
        

      }
// Write flight path points
      fl_points.Scanner().SetPointType(NormalPoint);
      fl_points.Write("fl.laser", 1);
// Create and write flight path topology
      for (i=0; i<fl_points.size(); i++) fl_top.push_back(PointNumber(i));
      fl_tops.push_back(fl_top);
      fl_tops.Write("fl.top");
// Write angle points
      angle_points.Scanner().SetPointType(ReflectancePoint);
      angle_points.Write("angles.laser", 1);
// Write the histogram
//      for (i=0; i<=max_num_pts; i++) printf("%3d %5d\n", i, histo[i]);

/* Delete all point data of the strip */

      for (partptr=stripptr->begin(); partptr!=stripptr->end(); partptr++)
        partptr->ErasePoints();
    }
  }
}
