
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
 Derivation of height, reflectance and colour images from point files in a laser
 altimetry block, strip, strip part, tile, strip tile, point set or from a
 point file.
 Files can be specified explicitly or by a file filter.
 Meta data can be updated with the new name(s) of the image file(s).

 Initial creation:
 Author : George Vosselman
 Date   : 26-04-1999

 Update #1
 Author : Avril Behan
 Date   : ??-10-1999
 Purpose: Added interpolation in TIN

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

/*
--------------------------------------------------------------------------------
                         Declaration of C functions
--------------------------------------------------------------------------------
*/

extern "C" void parse_filter(char **, char *);
extern "C" char *get_full_filename(char *, char *, int *);

/*
--------------------------------------------------------------------------------
                         The main laser2viff function
--------------------------------------------------------------------------------
*/

void laser2viff_cpp(char *long_filter, char *infile,
                       int height_flag, int reflectance_flag, int colour_flag,
                       int pulselength_flag, int pointcount_flag,
                       int viff_type, double pixelsize,
                       int input_bounds, char *bounds_file, int spec_bounds,
                       int xminset, double xmin, int xmaxset, double xmax,
                       int yminset, double ymin, int ymaxset, double ymax,
                       int zminset, double zmin, int zmaxset, double zmax,
                       int rminset, int rmin, int rmaxset, int rmax,
                       int plminset, int plmin, int plmaxset, int plmax,
                       char *image_directory,
                       char *height_file, char *reflectance_file,
                       char *colour_file, char *pulselength_file,
                       char *pointcount_file,
                       int update, char *meta_file_out,
                       int write_grid, char *height_grid_file,
                       char *reflectance_grid_file, char *colour_grid_file,
                       char *pulselength_grid_file, char *pointcount_grid_file,
                       int interpolation_method, double max_mesh_size)
{
  char                 *directory, *filter, *filename, *name;
  int                  icon, fileclass;
  LaserBlock           block;
  LaserBlock::iterator stripptr;
  LaserUnit::iterator  partptr;
  ImagedData           *imageddata;
  DataBoundsLaser      oldbounds, newbounds;

/* Set up the file filter for the input file(s) */

  if (long_filter) filter = long_filter;
  else filter = infile;
  directory = (char *) malloc(strlen(filter));
  parse_filter(&filter, directory);
  icon = 0;

/* Get the bounds for imaging */

  if (bounds_file) {
    newbounds.Extract(bounds_file);
    if (!newbounds.MinimumXIsSet() || !newbounds.MinimumYIsSet() ||
        !newbounds.MaximumXIsSet() || !newbounds.MaximumYIsSet()) {
      fprintf(stderr, "Error: Not all X and Y bounds were specified.\n");
      exit(0);
    }
  }
  else if (spec_bounds) {
    newbounds.Initialise();
    if (!xminset || !xmaxset || !yminset || !ymaxset) {
      fprintf(stderr, "Error: Not all X and Y bounds were specified.\n");
      exit(0);
    }
    newbounds.SetMinimumX(xmin);   newbounds.SetMaximumX(xmax);
    newbounds.SetMinimumY(ymin);   newbounds.SetMaximumY(ymax);
    if (height_flag && (!zminset || !zmaxset)) {
      fprintf(stderr, "Error: Height image requested, but height bounds were not specified.\n");
      exit(0);
    }
    newbounds.SetMinimumZ(zmin);   newbounds.SetMaximumZ(zmax);
    if (reflectance_flag && (!rminset || !rmaxset)) {
      fprintf(stderr, "Error: Reflectance image requested, but reflectance bounds were not specified.\n");
      exit(0);
    }
    if (colour_flag && (!rminset || !rmaxset)) {
      fprintf(stderr, "Error: Colour image requested, but bounds were not specified.\n");
      exit(0);
    }
    if (pulselength_flag && (!plminset || !plmaxset)) {
      fprintf(stderr, "Error: Pulse length image requested, but pulse length bounds were not specified.\n");
      exit(0);
    }
    newbounds.SetMinimumReflectance(rmin);
    newbounds.SetMaximumReflectance(rmax);
    newbounds.SetMinimumPulseLength(plmin);
    newbounds.SetMaximumPulseLength(plmax);
  }
  
/* Process all input files */

  block.Initialise();
  while ((filename = get_full_filename(directory, filter, &icon)) != NULL) {
    
/* Set up a laser block */

    if (!block.Create(filename, &fileclass)) {
      fprintf(stderr, "Error reading meta data file %s\n", filename);
      exit(0);
    }

/* Go to the appropriate level */

    switch (fileclass) {

      case LASER_BLOCK:

/* Copy the bounds */

        if (!input_bounds) {
          oldbounds = block.Bounds();
          block.SetBounds(newbounds);
        }

/* Create the height image */

        if (height_flag) {
          block.CreateHeightImage(pixelsize, viff_type, interpolation_method,
                                  max_mesh_size);
          imageddata = block.ImagedHeight();
          if (height_file) imageddata->SetImageFile(height_file);
          else if (!imageddata->ImageFile()) {
            name = block.Name();
            if (!name) name = block.DeriveName();
            if (!name) {
              fprintf(stderr, "Error: Can not derive name for height image.\n");
              exit(0);
            }
            block.DeriveHeightImageFileName(name, image_directory);
          }
          imageddata->WriteImage();
          imageddata->DeleteImage();
          if (write_grid || height_grid_file) {
            if (height_grid_file) imageddata->SetGridFile(height_grid_file);
            else imageddata->DeriveGridFileName();
            imageddata->WriteGrid();
          }
        }

/* Create the reflectance image */

        if (reflectance_flag) {
          block.CreateReflectanceImage(pixelsize, viff_type,
                                       interpolation_method, max_mesh_size);
          imageddata = block.ImagedReflectance();
          if (reflectance_file) imageddata->SetImageFile(reflectance_file);
          else if (!imageddata->ImageFile()) {
            name = block.Name();
            if (!name) name = block.DeriveName();
            if (!name) {
              fprintf(stderr,
                      "Error: Can not derive name for reflectance image.\n");
              exit(0);
            }
            block.DeriveReflectanceImageFileName(name, image_directory);
          }
          imageddata->WriteImage();
          imageddata->DeleteImage();
          if (write_grid || reflectance_grid_file) {
            if (reflectance_grid_file)
              imageddata->SetGridFile(reflectance_grid_file);
            else imageddata->DeriveGridFileName();
            imageddata->WriteGrid();
          }
        }

/* Create the colour image */

        if (colour_flag) {
          block.CreateColourImage(pixelsize, viff_type,
                                  interpolation_method, max_mesh_size);
          imageddata = block.ImagedColour();
          if (colour_file) imageddata->SetImageFile(colour_file);
          else if (!imageddata->ImageFile()) {
            name = block.Name();
            if (!name) name = block.DeriveName();
            if (!name) {
              fprintf(stderr,
                      "Error: Can not derive name for colour image.\n");
              exit(0);
            }
            block.DeriveColourImageFileName(name, image_directory);
          }
          imageddata->WriteImage();
          imageddata->DeleteImage();
          if (write_grid || colour_grid_file) {
            if (colour_grid_file)
              imageddata->SetGridFile(colour_grid_file);
            else imageddata->DeriveGridFileName();
            imageddata->WriteGrid();
          }
        }

/* Create the pulse length image */

        if (pulselength_flag) {
          block.CreatePulseLengthImage(pixelsize, viff_type,
                                       interpolation_method, max_mesh_size);
          imageddata = block.ImagedPulseLength();
          if (pulselength_file) imageddata->SetImageFile(pulselength_file);
          else if (!imageddata->ImageFile()) {
            name = block.Name();
            if (!name) name = block.DeriveName();
            if (!name) {
              fprintf(stderr,
                      "Error: Can not derive name for pulse length image.\n");
              exit(0);
            }
            block.DerivePulseLengthImageFileName(name, image_directory);
          }
          imageddata->WriteImage();
          imageddata->DeleteImage();
          if (write_grid || pulselength_grid_file) {
            if (pulselength_grid_file)
              imageddata->SetGridFile(pulselength_grid_file);
            else imageddata->DeriveGridFileName();
            imageddata->WriteGrid();
          }
        }

/* Create the point count image */

        if (pointcount_flag) {
          block.CreatePointCountImage(pixelsize, viff_type,
                                      interpolation_method, max_mesh_size);
          imageddata = block.ImagedPointCount();
          if (pointcount_file) imageddata->SetImageFile(pointcount_file);
          else if (!imageddata->ImageFile()) {
            name = block.Name();
            if (!name) name = block.DeriveName();
            if (!name) {
              fprintf(stderr,
                      "Error: Can not derive name for point count image.\n");
              exit(0);
            }
            block.DerivePointCountImageFileName(name, image_directory);
          }
          imageddata->WriteImage();
          imageddata->DeleteImage();
          if (write_grid || pointcount_grid_file) {
            if (pointcount_grid_file)
              imageddata->SetGridFile(pointcount_grid_file);
            else imageddata->DeriveGridFileName();
            imageddata->WriteGrid();
          }
        }

/* Update the meta data */

        if (update) {
          if (!input_bounds) block.SetBounds(oldbounds);/* Restore old bounds */
          if (meta_file_out) block.SetMetaDataFile(meta_file_out);
          if (!block.WriteMetaData()) {
            fprintf(stderr, "Error writing block meta data to file %s\n",
                    block.MetaDataFile());
            exit(0);
          }
        }
        break;

      case LASER_STRIP:
        stripptr = block.begin();

/* Copy the bounds */

        if (!input_bounds) {
          oldbounds = stripptr->Bounds();
          stripptr->SetBounds(newbounds);
        }

/* Create the height image */

        if (height_flag) {
          stripptr->CreateHeightImage(pixelsize, viff_type,
                                      interpolation_method, max_mesh_size);
          imageddata = stripptr->ImagedHeight();
          if (height_file) imageddata->SetImageFile(height_file);
          else if (!imageddata->ImageFile()) {
            name = stripptr->Name();
            if (!name) name = stripptr->DeriveName();
            if (!name) {
              fprintf(stderr, "Error: Can not derive name for height image.\n");
              exit(0);
            }
            stripptr->DeriveHeightImageFileName(name, image_directory);
          }
          imageddata->WriteImage();
          imageddata->DeleteImage();
          if (write_grid || height_grid_file) {
            if (height_grid_file) imageddata->SetGridFile(height_grid_file);
            else imageddata->DeriveGridFileName();
            imageddata->WriteGrid();
          }
        }

/* Create the reflectance image */

        if (reflectance_flag) {
          stripptr->CreateReflectanceImage(pixelsize, viff_type,
                                           interpolation_method, max_mesh_size);
          imageddata = stripptr->ImagedReflectance();
          if (reflectance_file) imageddata->SetImageFile(reflectance_file);
          else if (!imageddata->ImageFile()) {
            name = stripptr->Name();
            if (!name) name = stripptr->DeriveName();
            if (!name) {
              fprintf(stderr,
                      "Error: Can not derive name for reflectance image.\n");
              exit(0);
            }
            stripptr->DeriveReflectanceImageFileName(name, image_directory);
          }
          imageddata->WriteImage();
          imageddata->DeleteImage();
          if (write_grid || reflectance_grid_file) {
            if (reflectance_grid_file)
              imageddata->SetGridFile(reflectance_grid_file);
            else imageddata->DeriveGridFileName();
            imageddata->WriteGrid();
          }
        }

/* Create the colour image */

        if (colour_flag) {
          stripptr->CreateColourImage(pixelsize, viff_type,
                                      interpolation_method, max_mesh_size);
          imageddata = stripptr->ImagedColour();
          if (colour_file) imageddata->SetImageFile(colour_file);
          else if (!imageddata->ImageFile()) {
            name = stripptr->Name();
            if (!name) name = stripptr->DeriveName();
            if (!name) {
              fprintf(stderr,
                      "Error: Can not derive name for colour image.\n");
              exit(0);
            }
            stripptr->DeriveColourImageFileName(name, image_directory);
          }
          imageddata->WriteImage();
          imageddata->DeleteImage();
          if (write_grid || colour_grid_file) {
            if (colour_grid_file)
              imageddata->SetGridFile(colour_grid_file);
            else imageddata->DeriveGridFileName();
            imageddata->WriteGrid();
          }
        }

/* Create the pulse length image */

        if (pulselength_flag) {
          stripptr->CreatePulseLengthImage(pixelsize, viff_type,
                                           interpolation_method, max_mesh_size);
          imageddata = stripptr->ImagedPulseLength();
          if (pulselength_file) imageddata->SetImageFile(pulselength_file);
          else if (!imageddata->ImageFile()) {
            name = stripptr->Name();
            if (!name) name = stripptr->DeriveName();
            if (!name) {
              fprintf(stderr,
                      "Error: Can not derive name for pulse length image.\n");
              exit(0);
            }
            stripptr->DerivePulseLengthImageFileName(name, image_directory);
          }
          imageddata->WriteImage();
          imageddata->DeleteImage();
          if (write_grid || pulselength_grid_file) {
            if (pulselength_grid_file)
              imageddata->SetGridFile(pulselength_grid_file);
            else imageddata->DeriveGridFileName();
            imageddata->WriteGrid();
          }
        }

/* Create the point count image */

        if (pointcount_flag) {
          stripptr->CreatePointCountImage(pixelsize, viff_type,
                                          interpolation_method, max_mesh_size);
          imageddata = stripptr->ImagedPointCount();
          if (pointcount_file) imageddata->SetImageFile(pointcount_file);
          else if (!imageddata->ImageFile()) {
            name = stripptr->Name();
            if (!name) name = stripptr->DeriveName();
            if (!name) {
              fprintf(stderr,
                      "Error: Can not derive name for point count image.\n");
              exit(0);
            }
            stripptr->DerivePointCountImageFileName(name, image_directory);
          }
          imageddata->WriteImage();
          imageddata->DeleteImage();
          if (write_grid || pointcount_grid_file) {
            if (pointcount_grid_file)
              imageddata->SetGridFile(pointcount_grid_file);
            else imageddata->DeriveGridFileName();
            imageddata->WriteGrid();
          }
        }

/* Update the meta data */

        if (update) {
          if (!input_bounds) stripptr->SetBounds(oldbounds);/* Restore bounds */
          if (meta_file_out) stripptr->SetMetaDataFile(meta_file_out);
          if (!stripptr->WriteMetaData()) {
            fprintf(stderr, "Error writing strip meta data to file %s\n",
                    stripptr->MetaDataFile());
            exit(0);
          }
        }
        break;

      case LASER_SUB_UNIT:
      case LASER_POINT_SET:
      case LASER_RAW_DATA:
        stripptr = block.begin();
        partptr  = stripptr->begin();

/* Copy the bounds */

        if (!input_bounds) {
          oldbounds = partptr->Bounds();
          partptr->SetBounds(newbounds);
        }

/* Create the height image */

        if (height_flag) {
          partptr->CreateHeightImage(pixelsize, viff_type,
                                     interpolation_method, max_mesh_size);
          imageddata = partptr->ImagedHeight();
          if (height_file) imageddata->SetImageFile(height_file);
          else if (!imageddata->ImageFile()) {
            name = partptr->Name();
            if (!name) name = partptr->LaserDataFiles::DeriveName();
            if (!name) {
              fprintf(stderr, "Error: Can not derive name for height image.\n");
              exit(0);
            }
            partptr->DeriveHeightImageFileName(name, image_directory);
          }
          imageddata->WriteImage();
          imageddata->DeleteImage();
          if (write_grid || height_grid_file) {
            if (height_grid_file) imageddata->SetGridFile(height_grid_file);
            else imageddata->DeriveGridFileName();
            imageddata->WriteGrid();
          }
        }

/* Create the reflectance image */

        if (reflectance_flag) {
          partptr->CreateReflectanceImage(pixelsize, viff_type,
                                          interpolation_method, max_mesh_size);
          imageddata = partptr->ImagedReflectance();
          if (reflectance_file) imageddata->SetImageFile(reflectance_file);
          else if (!imageddata->ImageFile()) {
            name = partptr->Name();
            if (!name) name = partptr->LaserDataFiles::DeriveName();
            if (!name) {
              fprintf(stderr,
                      "Error: Can not derive name for reflectance image.\n");
              exit(0);
            }
            partptr->DeriveReflectanceImageFileName(name, image_directory);
          }
          imageddata->WriteImage();
          imageddata->DeleteImage();
          if (write_grid || reflectance_grid_file) {
            if (reflectance_grid_file)
              imageddata->SetGridFile(reflectance_grid_file);
            else imageddata->DeriveGridFileName();
            imageddata->WriteGrid();
          }
        }

/* Create the colour image */

        if (colour_flag) {
          partptr->CreateColourImage(pixelsize, viff_type,
                                          interpolation_method, max_mesh_size);
          imageddata = partptr->ImagedColour();
          if (colour_file) imageddata->SetImageFile(colour_file);
          else if (!imageddata->ImageFile()) {
            name = partptr->Name();
            if (!name) name = partptr->LaserDataFiles::DeriveName();
            if (!name) {
              fprintf(stderr,
                      "Error: Can not derive name for colour image.\n");
              exit(0);
            }
            partptr->DeriveColourImageFileName(name, image_directory);
          }
          imageddata->WriteImage();
          imageddata->DeleteImage();
          if (write_grid || colour_grid_file) {
            if (colour_grid_file)
              imageddata->SetGridFile(colour_grid_file);
            else imageddata->DeriveGridFileName();
            imageddata->WriteGrid();
          }
        }

/* Create the pulse length image */

        if (pulselength_flag) {
          partptr->CreatePulseLengthImage(pixelsize, viff_type,
                                          interpolation_method, max_mesh_size);
          imageddata = partptr->ImagedPulseLength();
          if (pulselength_file) imageddata->SetImageFile(pulselength_file);
          else if (!imageddata->ImageFile()) {
            name = partptr->Name();
            if (!name) name = partptr->LaserDataFiles::DeriveName();
            if (!name) {
              fprintf(stderr,
                      "Error: Can not derive name for pulse length image.\n");
              exit(0);
            }
            partptr->DerivePulseLengthImageFileName(name, image_directory);
          }
          imageddata->WriteImage();
          imageddata->DeleteImage();
          if (write_grid || pulselength_grid_file) {
            if (pulselength_grid_file)
              imageddata->SetGridFile(pulselength_grid_file);
            else imageddata->DeriveGridFileName();
            imageddata->WriteGrid();
          }
        }

/* Create the point count image */

        if (pointcount_flag) {
          partptr->CreatePointCountImage(pixelsize, viff_type,
                                         interpolation_method, max_mesh_size);
          imageddata = partptr->ImagedPointCount();
          if (pointcount_file) imageddata->SetImageFile(pointcount_file);
          else if (!imageddata->ImageFile()) {
            name = partptr->Name();
            if (!name) name = partptr->LaserDataFiles::DeriveName();
            if (!name) {
              fprintf(stderr,
                      "Error: Can not derive name for point count image.\n");
              exit(0);
            }
            partptr->DerivePointCountImageFileName(name, image_directory);
          }
          imageddata->WriteImage();
          imageddata->DeleteImage();
          if (write_grid || pointcount_grid_file) {
            if (pointcount_grid_file)
              imageddata->SetGridFile(pointcount_grid_file);
            else imageddata->DeriveGridFileName();
            imageddata->WriteGrid();
          }
        }

/* Update the meta data */

        if (update) {
          if (!input_bounds) partptr->SetBounds(oldbounds); /* Restore bounds */
          switch (fileclass) {
            case LASER_SUB_UNIT:

              if (meta_file_out) partptr->SetMetaDataFile(meta_file_out);
              if (!partptr->WriteMetaData()) {
                fprintf(stderr, "Error writing meta data to file %s\n",
                        partptr->MetaDataFile());
                exit(0);
              }
              break;

            case LASER_POINT_SET:
            case LASER_RAW_DATA:

              if (meta_file_out) partptr->SetMetaDataFile(meta_file_out);
              else if (fileclass == LASER_RAW_DATA)
                partptr->LaserPoints::DeriveMetaDataFileName(image_directory);
              if (!partptr->LaserPoints::WriteMetaData()) {
                fprintf(stderr,
                        "Error writing point set meta data to file %s\n",
                        partptr->MetaDataFile());
                exit(0);
              }
              break;
          }
        }
        break;

      default:
        fprintf(stderr, "Error: Unknown file class (%d).\n", fileclass);
        exit(0);
    }
  }
}
