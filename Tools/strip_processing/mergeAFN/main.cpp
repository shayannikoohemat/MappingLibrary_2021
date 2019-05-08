
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
#include "LaserPoints.h"
#include "Line2D.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: mergeAFN -a <A scan point file>\n");
  printf("         -f <F scan point file>\n");
  printf("         -n <N scan point file>\n");
  printf("         -o <output file>\n");
  printf("         -xyzonly <for files without RGB ret information\n");
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  void            MergeAFN(char *, char *, char *, char *, bool);

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-a") || !args->Contains("-f") ||
      !args->Contains("-n") || !args->Contains("-o")) {
    printf("Error: -a, -f, -n and -o are required arguments.\n");
    PrintUsage();
    exit(0);
  }

  MergeAFN(args->String("-a"), args->String("-f"),
           args->String("-n"), args->String("-o"),
           args->Contains("-xyzonly"));
  
  return EXIT_SUCCESS;
}

void ReadPoints(FILE *fd, LaserPoints &points, LaserPoint &next_point,
                bool xyz_only, bool &eof, int &num_read)
{
  char line[1025];
  double x, y, z;
  int    r, g, b, ret;

  points.ErasePoints();
  if (eof) return;
  points.push_back(next_point);
  do {
    if (fgets(line, 1024, fd)) {
      num_read++;
      if (xyz_only) {
        sscanf(line, "%lf %lf %lf", &x, &y, &z);
        next_point.X() = x;
        next_point.Y() = y;
        next_point.Z() = z;
        if (feof(fd)) eof = true;
        return;
      }
      sscanf(line, "%lf %lf %lf %d %d %d %d", &x, &y, &z, &r, &g, &b, &ret);
      next_point.SetColour(r, g, b);
      next_point.PulseCountWithFlag() = ret;
      next_point.X() = x;
      next_point.Y() = y;
      next_point.Z() = z;
      if (ret > 0) points.push_back(next_point);
    }
  } while (!feof(fd) && ret > 0);
  if (feof(fd)) eof = true;
}

Line2D StripDirection(char *filename, bool xyz_only)
{
  FILE *fd;
  LaserPoints points, ignore;
  LaserPoint  point, start_point, end_point;
  bool eof = false;
  int i, j;
  
  // Read the first 1000 points and determine the centre point
  fd = fopen(filename, "r");
  if (!fd) {
    printf("Error reading file %s\n", filename);
    exit(0);
  }
  for (i=0; i<1000 && !eof; i++) {
    ReadPoints(fd, ignore, point, xyz_only, eof, j);
    points.push_back(point);
  }
  points.DeriveDataBounds(0);
  start_point = points.Bounds().MidPoint();
  points.ErasePoints();
  
  // Skip some 100000 points
  for (i=0; i<100000 && !eof; i++)
    ReadPoints(fd, ignore, point, xyz_only, eof, j);
  if (eof) {
    printf("Error in determining strip direction, less than 100 scan lines\n");
    exit(0);
  }
  
  // Read another 1000 points and determine the centre point
  for (i=0; i<1000 && !eof; i++) {
    ReadPoints(fd, ignore, point, xyz_only, eof, j);
    points.push_back(point);
  }
  points.DeriveDataBounds(0);
  end_point = points.Bounds().MidPoint();
  points.ErasePoints();
  fclose(fd);
  
  // Construct the line between these two centre points
  return Line2D(start_point.Position2DOnly(), end_point.Position2DOnly());
}

LaserPoints * FirstScan(LaserPoints &a_points, LaserPoints &f_points,
                        LaserPoints &n_points, Line2D &line,
                        double &lowest_scalar)
{
  LaserPoints *first;
  double      scalar;
  
  lowest_scalar = 1e10;
  first = NULL;
  if (!a_points.empty()) {
    lowest_scalar = line.Scalar(a_points.begin()->Position2DOnly());
    first = &a_points;
  }
  if (!f_points.empty()) {
    scalar = line.Scalar(f_points.begin()->Position2DOnly());
    if (scalar < lowest_scalar) {
      lowest_scalar = scalar;
      first = &f_points;
    }
  }
  if (!n_points.empty()) {
    scalar = line.Scalar(n_points.begin()->Position2DOnly());
    if (scalar < lowest_scalar) first = &n_points;
  }
  return first;
}

void MergeAFN(char *a_filename, char *f_filename, char *n_filename,
              char *o_filename, bool xyz_only)
{
  FILE *a_fd, *f_fd, *n_fd, *o_fd;
  LaserPoints a_points, f_points, n_points, *first;
  LaserPoints::iterator point;
  LaserPoint a_next, f_next, n_next;
  bool a_eof=false, f_eof=false, n_eof=false;
  Line2D line;
  double scalar, max_scalar=-1e10;
  int a_read=0, f_read=0, n_read=0, a_write=0, f_write=0, n_write=0;
  
  // Determine the strip direction
  line = StripDirection(a_filename, xyz_only);

  // Open files
  a_fd = fopen(a_filename, "r");
  f_fd = fopen(f_filename, "r");
  n_fd = fopen(n_filename, "r");
  if (!a_fd || !f_fd || !n_fd) {
    printf("Error opening one of the input files\n");
    exit(0);
  }
  o_fd = fopen(o_filename, "w");
  if (!o_fd) {
    printf("Error opening output file\n");
    exit(0);
  }
  
  // Fill the next point buffers
  ReadPoints(a_fd, a_points, a_next, xyz_only, a_eof, a_read);
  ReadPoints(f_fd, f_points, f_next, xyz_only, f_eof, f_read);
  ReadPoints(n_fd, n_points, n_next, xyz_only, n_eof, n_read);
  ReadPoints(a_fd, a_points, a_next, xyz_only, a_eof, a_read);
  ReadPoints(f_fd, f_points, f_next, xyz_only, f_eof, f_read);
  ReadPoints(n_fd, n_points, n_next, xyz_only, n_eof, n_read);
  first = NULL;
  
  do {
    // Read next points
    if (first == &a_points) {
      a_write++;
      if (scalar > max_scalar) {
        max_scalar = scalar;
        printf("A %10.2f A %d/%d F %d/%d N %d/%d\r", max_scalar,
               a_read, a_write, f_read, f_write, n_read, n_write);
      }
      ReadPoints(a_fd, a_points, a_next, xyz_only, a_eof, a_read);
    }
    else if (first == &f_points) {
      f_write++;
      if (scalar > max_scalar) {
        max_scalar = scalar;
        printf("A %10.2f A %d/%d F %d/%d N %d/%d\r", max_scalar,
               a_read, a_write, f_read, f_write, n_read, n_write);
      }
      ReadPoints(f_fd, f_points, f_next, xyz_only, f_eof, f_read);
    }
    else if (first == &n_points) {
      n_write++;
      if (scalar > max_scalar) {
        max_scalar = scalar;
        printf("A %10.2f A %d/%d F %d/%d N %d/%d\r", max_scalar,
               a_read, a_write, f_read, f_write, n_read, n_write);
      }
      ReadPoints(n_fd, n_points, n_next, xyz_only, n_eof, n_read);
    }
    // Determine first one to use
    first = FirstScan(a_points, f_points, n_points, line, scalar);
    // Output of points
    if (first != NULL) {
      for (point=first->begin(); point!=first->end(); point++)
        if (xyz_only) 
          fprintf(o_fd, "%.2f %.2f %.2f\n",
                  point->X(), point->Y(), point->Z());
        else
          fprintf(o_fd, "%.2f %.2f %.2f %d %d %d %d\n",
                 point->X(), point->Y(), point->Z(),
                 point->Red(), point->Green(), point->Blue(),
                 point->PulseCount());
    }
  } while (!a_points.empty() || !f_points.empty() || !n_points.empty());
  
  fclose(a_fd);
  fclose(f_fd);
  fclose(n_fd);
  fclose(o_fd);
}
