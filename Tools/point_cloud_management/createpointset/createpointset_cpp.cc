
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
 Selection of laser altimetry data based on bounds from
 - a meta data file
 - image coordinates of a rectangular area
 - a labeled image
 - the program arguments
 All selected points are written to one output file. Optionally a meta data
 file is also produced.

 Initial creation:
 Author : George Vosselman
 Date   : 22-04-1999

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
#include "ObjectPoints2D.h"
#include "ImageGrid.h"
#include "ImagePoints.h"

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(char *, char *, int *);

/*
--------------------------------------------------------------------------------
                         The main laser2tin function
--------------------------------------------------------------------------------
*/

void createpointset_cpp(char *long_filter, char *infile,
                        char *meta_bound_file, char *corner_file,
                        char *polygon_point_file, char *topology_file,
                        char *image_file, int userdata, char *grid_file,
                        int xmin_set, double xmin, int xmax_set, double xmax,
                        int ymin_set, double ymin, int ymax_set, double ymax,
                        int zmin_set, double zmin, int zmax_set, double zmax,
                        int rmin_set, int rmin, int rmax_set, int rmax,
                        int pcmin_set, int pcmin, int pcmax_set, int pcmax,
                        int plmin_set, int plmin, int plmax_set, int plmax,
                        int lmin_set, int lmin, int lmax_set, int lmax,
                        char *point_file, char *meta_file)
{
  char                     *directory, *filter, *filename;
  int                      icon, success, filetype;
  double                   xminc, xmaxc, yminc, ymaxc;
  LaserPoints              selection;
  LaserBlock               block;
  LaserBlock::iterator     unitptr;
  LaserUnit::iterator      subunitptr;
  DataBoundsLaser          bounds;
  ObjectPoints2D           *corners;
  ObjectPoints2D::iterator corner;
  ImageGrid                *grid;
  ImagePoints              *imagepoints;
  Image                    label_image;
  LineTopologies           *topology;

/* Check if the grid file is specified for the second and third selection mode*/

  if ((corner_file || image_file) && !grid_file) {
    fprintf(stderr, "Error: required image grid file was not specified.\n");
    exit(0);
  }

/*
--------------------------------------------------------------------------------
                    Gather the appropriate input data
--------------------------------------------------------------------------------
*/

/* Bounds from meta data file */

  if (meta_bound_file) {
    if (!bounds.Extract(meta_bound_file)) {
      fprintf(stderr, "Error extracting bounds from file %s\n",
              meta_bound_file);
      exit(0);
    }
  }

/* Bounds from rectangular image part */

  else if (corner_file) { 

    /* Read image points and grid file */

    imagepoints = new ImagePoints(corner_file, &success);
    if (!success) {
      fprintf(stderr, "Error reading image points from file %s\n",
              corner_file);
      exit(0);
    }
    grid = new ImageGrid(grid_file, &success);
    if (!success) {
      fprintf(stderr, "Error reading grid specification from file %s\n",
              grid_file);
      exit(0);
    }

    /* Convert to XY coordinates and set minima and maxima */

    corners = new ObjectPoints2D(*imagepoints, *grid);
    corner = corners->begin();
    xminc = xmaxc = corner->X();
    yminc = ymaxc = corner->Y();
    corner++;
    if (corner->X() < xminc) xminc = corner->X();
    if (corner->X() > xmaxc) xmaxc = corner->X();
    if (corner->Y() < yminc) yminc = corner->Y();
    if (corner->Y() > ymaxc) ymaxc = corner->Y();
    bounds.SetMinimumX(xminc);   bounds.SetMaximumX(xmaxc);
    bounds.SetMinimumY(yminc);   bounds.SetMaximumY(ymaxc);
    delete imagepoints;
    delete grid;
    delete corners;
  }

/* Bounds from polygons */

  else if (polygon_point_file) {

    /* Read the point, topology and grid files */

    imagepoints = new ImagePoints(polygon_point_file, &success);
    if (!success) {
      fprintf(stderr, "Error reading image polygon points from file %s\n",
              polygon_point_file);
      exit(0);
    }
    topology = new LineTopologies(topology_file, &success);
    if (!success) {
      fprintf(stderr, "Error reading polygon topology from file %s\n",
              topology_file);
      exit(0);
    }
    grid = new ImageGrid(grid_file, &success);
    if (!success) {
      fprintf(stderr, "Error reading grid specification from file %s\n",
              grid_file);
      exit(0);
    }

    /* Convert the image points to 2D object points */

    corners = new ObjectPoints2D(*imagepoints, *grid);
    delete imagepoints;
  }

/* Bounds from labeled image */

  else if (image_file) { 

/* Read the labeled image and the grid specification */

    label_image.Initialise();
    if (!label_image.Read(image_file)) {
      fprintf(stderr, "Error reading labeled image from file %s\n",
              image_file);
      exit(0);
    }
    grid = new ImageGrid(grid_file, &success);
    if (!success) {
      fprintf(stderr, "Error reading grid specification from file %s\n",
              grid_file);
      exit(0);
    }
  }

/* Bounds from user specified values */

  else if (userdata) { 
    bounds.Initialise();
    if (xmin_set) bounds.SetMinimumX(xmin);
    if (xmax_set) bounds.SetMaximumX(xmax);
    if (ymin_set) bounds.SetMinimumY(ymin);
    if (ymax_set) bounds.SetMaximumY(ymax);
    if (zmin_set) bounds.SetMinimumZ(zmin);
    if (zmax_set) bounds.SetMaximumZ(zmax);
    if (rmin_set) bounds.SetMinimumReflectance((int) rmin);
    if (rmax_set) bounds.SetMaximumReflectance((int) rmax);
    if (pcmin_set) bounds.SetMinimumPulseCount((int) pcmin);
    if (pcmax_set) bounds.SetMaximumPulseCount((int) pcmax);
    if (plmin_set) bounds.SetMinimumPulseLength((int) plmin);
    if (plmax_set) bounds.SetMaximumPulseLength((int) plmax);
    if (lmin_set) bounds.SetMinimumLabel((int) lmin);
    if (lmax_set) bounds.SetMaximumLabel((int) lmax);
  }
  else {
    fprintf(stderr, "Error: No selection mode specified.\n");
    exit(0);
  }

/* Initialise the selected point set */

  selection.Initialise();

/* Set up the file filter for the input file(s) */

  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

/* Process all input files */

  block.Initialise();
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    
/* Set up a laser block */

    if (!block.Create(filename, &filetype)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }

/* Loop over all units */

    for (unitptr=block.begin(); unitptr!=block.end(); unitptr++) {

/* Loop over all sub units */

      for (subunitptr=unitptr->begin();
           subunitptr!=unitptr->end();
           subunitptr++) {

/* Read the points, select points within bounds, and delete the points */

        if (!subunitptr->Read()) {
          fprintf(stderr, "Error reading laser points from file %s\n",
                  subunitptr->PointFile());
          exit(0);
        }
        if (image_file)
          subunitptr->LaserPoints::Select(selection, label_image, *grid);
        else if (polygon_point_file)
          subunitptr->LaserPoints::Select(selection,
                                          corners->RefObjectPoints2D(),
                                          topology->LineTopologiesReference());
        else
          subunitptr->Select(selection, bounds);
        subunitptr->ErasePoints();

/* Check the data type */

        if (selection.Scanner().PointType() == UnknownPoint)
          selection.Scanner().SetPointType(subunitptr->Scanner().PointType());
      }
    }
  }

/* Output of the point data */

  if (!selection.empty()) {
    selection.SetPointFile(point_file);
    selection.Write();

/* Optional output of the meta data */

    if (meta_file) {
      selection.SetMetaDataFile(meta_file);
      selection.WriteMetaData();
    }
  }
}
