
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
 Collection of functions for class LaserUnit

 Initial creation
 Author : George Vosselman
 Date   : 29-03-1999

 Update #1
 Author : George Vosselman
 Date   : 19-11-1999
 Changes: Added support for tiled strips and blocks

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
#include "LaserUnit.h"
#include "LaserBlock.h"
#include "BNF_io.h"
#include "stdmath.h"

#include <time.h>

#ifdef windows		
		#include <windows.h>
#endif	


extern int LargerScalar(const LaserPoint *point1, const LaserPoint *point2);
extern int LargerXYZ(const LaserPoint *point1, const LaserPoint *point2);

/*
--------------------------------------------------------------------------------
                     Construct unit from point or meta file name
--------------------------------------------------------------------------------
*/

LaserUnit::LaserUnit(const char *filename)
{
  Initialise();
  switch (BNF_LaserFileClass(filename)) {
    case LASER_RAW_DATA:
      SetPointFile(filename);
      push_back(CompleteStrip());
      break;
    case LASER_STRIP:
      data_org = StripWise;
      ReadMetaData(filename);
      break;
    default:
      fprintf(stderr, "Error: This LaserUnit constructor was only implemented for file names of raw laser data or strip meta data\n");
      break;
  }

// Copy scanner information from first sub unit

  if (size()) scanner = begin()->Scanner();
}

/*
--------------------------------------------------------------------------------
                     Copy constructor and assignment
--------------------------------------------------------------------------------
*/

LaserUnit::LaserUnit(const LaserUnit &unit)
{
  Initialise();
  if (!empty()) erase(begin(), end());
  if (!unit.empty()) insert(begin(), unit.begin(), unit.end());
  LaserPointsInfoReference() = unit.LaserPointsInfoReference();
  LaserDataFilesReference()  = unit.LaserDataFilesReference();
  strip_number               = unit.strip_number;
  strip_transform            = unit.strip_transform;
  data_org                   = unit.data_org;
  StringCopy(&flight_path_file, unit.flight_path_file);
  flight_path                = unit.flight_path;
  strip_axis                 = unit.strip_axis;
  scanlines                  = unit.scanlines;
  StringCopy(&tie_points_file, unit.tie_points_file);
  tie_points                 = unit.tie_points;
  StringCopy(&control_points_file, unit.control_points_file);
  control_points             = unit.control_points;
}

LaserUnit & LaserUnit::operator = (const LaserUnit &unit)
{
  if (this == &unit) return *this;  // Check for self assignment
  if (!empty()) erase(begin(), end());
  if (!unit.empty()) insert(begin(), unit.begin(), unit.end());
  LaserPointsInfoReference() = unit.LaserPointsInfoReference();
  LaserDataFilesReference()  = unit.LaserDataFilesReference();
  strip_number               = unit.strip_number;
  strip_transform            = unit.strip_transform;
  data_org                   = unit.data_org;
  StringCopy(&flight_path_file, unit.flight_path_file);
  flight_path                = unit.flight_path;
  strip_axis                 = unit.strip_axis;
  scanlines                  = unit.scanlines;
  StringCopy(&tie_points_file, unit.tie_points_file);
  tie_points                 = unit.tie_points;
  StringCopy(&control_points_file, unit.control_points_file);
  control_points             = unit.control_points;
  return(*this);
}

/*
--------------------------------------------------------------------------------
                         Initialise the meta data
--------------------------------------------------------------------------------
*/

void LaserUnit::Initialise()
{
  LaserPointsInfo::Initialise();
  data_org = UnknownOrganisation;
  meta_file = name = point_file = flight_path_file = NULL;
  tie_points_file = control_points_file = NULL;
  strip_number = 0;
  strip_transform.Initialise();
  scanlines.Initialise();
  strip_axis = NULL;
}

void LaserUnit::ReInitialise()
{
  if (!empty()) erase(begin(), end());  /* Delete old sub units */
  LaserPointsInfo::ReInitialise();
  strip_number = 0;
  data_org = UnknownOrganisation;
  if (meta_file) free(meta_file);                 meta_file        = NULL;
  if (name) free(name);                           name             = NULL;
  if (point_file) free(point_file);               point_file       = NULL;
  if (flight_path_file) free(flight_path_file);   flight_path_file = NULL;
  if (tie_points_file) free(tie_points_file);     tie_points_file  = NULL;
  if (control_points_file) free(control_points_file); control_points_file =NULL;
  if (!flight_path.empty())
    flight_path.erase(flight_path.begin(), flight_path.end());
  if (!tie_points.empty())
    tie_points.erase(tie_points.begin(), tie_points.end());
  if (!control_points.empty())
    control_points.erase(control_points.begin(), control_points.end());
  if (strip_axis) strip_axis = NULL;
  strip_transform.Initialise();
  scanlines.ReInitialise();
}

/*
--------------------------------------------------------------------------------
                       Read the unit meta data
--------------------------------------------------------------------------------
*/


int LaserUnit::ReadMetaData(const char *filename, bool read_subunits)
{
  FILE           *fd;
  char           *buffer, *line, *keyword, *file;
  int            keyword_length, success, strip_in_parts=0;
  bool           new_strip_axis=false;
  
/* Open the file */

  fd = Open_Compressed_File(filename, "r");
  if (fd == NULL) {
    fprintf(stderr, "Error opening laser strip database %s\n", filename);
    return(0);
  }

/* Verify that the first keyword is "laserstrip" */

  buffer = (char *) malloc(MAXCHARS);
  do {
    line = fgets(buffer, MAXCHARS, fd);
  } while (line && Is_Comment(line));
  if (line == NULL) {
    fprintf(stderr, "Error reading strip database %s\n", filename);
    fclose(fd);
    return(0);
  }
  keyword = BNF_KeyWord(line, &keyword_length);
  if (strncmp(keyword, "laserstrip", MAX(keyword_length, 10))) {
    fprintf(stderr,"Error: File %s is not a laser strip database.\n", filename);
    fprintf(stderr,"       First keyword is %s, not laserstrip.\n", keyword);
    exit(0);
  }

/* Initialise all meta data */

  ReInitialise();

/* Process all lines */

  data_org = StripWise;
  SetMetaDataFile(filename);
  while ((line = fgets(buffer, MAXCHARS, fd))) {
    if (!Is_Comment(line)) {
      keyword = BNF_KeyWord(line, &keyword_length);
      if (keyword) {
	
        if (!strncmp(keyword, "name", MAX(keyword_length, 4)))
          name = BNF_String(line);

        else if (!strncmp(keyword, "points", MAX(keyword_length, 6)))
          point_file = BNF_String(line);

        else if (!strncmp(keyword, "tin", MAX(keyword_length, 3)))
          tin_file = BNF_String(line);

        else if (!strncmp(keyword, "flight_path", MAX(keyword_length, 11)))
          flight_path_file = BNF_String(line);

        else if (!strncmp(keyword, "stripaxis", MAX(keyword_length, 9))) {
          if (strip_axis == NULL) strip_axis = new Line2D();
          new_strip_axis = true;
        }
        
        else if (!strncmp(keyword, "normal_direction", MAX(keyword_length, 16))) {
          if (strip_axis == NULL || !new_strip_axis) {
          	printf("Warning: keyword normal_direction found without preceeding stripaxis\n");
          	strip_axis = new Line2D();
          	new_strip_axis = true;
          }
          strip_axis->SetNormalDirection(BNF_Double(line) * atan(1.0) / 45.0);
        }
		
        else if (!strncmp(keyword, "distance_to_origin", MAX(keyword_length, 18))) {
          if (strip_axis == NULL || !new_strip_axis) {
          	printf("Warning: keyword distance_to_origin found without preceeding stripaxis\n");
          	strip_axis = new Line2D();
          	new_strip_axis = true;
          }
          strip_axis->SetDistanceToOrigin(BNF_Double(line));
        }
		
        else if (!strncmp(keyword, "endstripaxis", MAX(keyword_length, 12))) {
          if (strip_axis == NULL) {
          	printf("Error: keyword endstripaxis found without preceeding stripaxis\n");
          }
		}
		
        else if (!strncmp(keyword, "scanlines", MAX(keyword_length, 9)))
          scanlines.SetScanLinesFile(BNF_String(line));

        else if (!strncmp(keyword, "tie_points", MAX(keyword_length, 10)))
          tie_points_file = BNF_String(line);

        else if (!strncmp(keyword, "control_points", MAX(keyword_length, 14)))
          control_points_file = BNF_String(line);

        else if (!strncmp(keyword, "info", MAX(keyword_length, 4)))
          LaserPointsInfo::ReadMetaData(fd);

        else if (!strncmp(keyword, "orientation", MAX(keyword_length, 11)))
          strip_transform.Orientation().Read(fd);

        else if (!strncmp(keyword, "strip_part", MAX(keyword_length, 10))) {
          if (data_org != StripWise) {
            fprintf(stderr,
                   "Error: A strip can not have both strip parts and tiles!\n");
            fprintf(stderr, "Error was detected in strip %s\n", filename);
            exit(0);
          }
          file = BNF_String(line);
          if (file) {
            if (read_subunits) {
              push_back(LaserSubUnit(file, &success));
              if (!success) erase(end()-1); 
              else strip_in_parts = 1;
            }
            else {
              push_back(LaserSubUnit());
              (end()-1)->SetMetaDataFile(file);
            }
          }
        }
        
        else if (!strncmp(keyword, "strip_tile", MAX(keyword_length, 10))) {
          if (strip_in_parts) {
            fprintf(stderr,
                   "Error: A strip can not have both strip parts and tiles!\n");
            fprintf(stderr, "Error was detected in strip %s\n", filename);
            exit(0);
          }
          file = BNF_String(line);
          if (file) {
            if (read_subunits) {
              push_back(LaserSubUnit(file, &success));
              if (!success) erase(end()-1); 
              else data_org = StripTileWise;
            }
            else {
              push_back(LaserSubUnit());
              (end()-1)->SetMetaDataFile(file);
            }
          }
        }

        else if (!strncmp(keyword, "endlaserstrip", MAX(keyword_length,13))) {
          // In case there is no strip part, generate one strip part to contain
          // the whole strip. Only do so if there is a point file name in the strip
          // meta data. Otherwise, consider this to be an empty strip.
          if (empty() && PointFile()) push_back(CompleteStrip());
          free(buffer);
          fclose(fd);
          return(1);
        }

        else {
          keyword[keyword_length] = 0;
          fprintf(stderr, "Warning: Unknown keyword (%s) ignored.\n", keyword);
        }
      }
    }
  }
  fprintf(stderr, "Error: Did not find keyword endlaserstrip.\n");
  fclose(fd);
  return(0);
}


/*
--------------------------------------------------------------------------------
                   Write the unit meta data to file
--------------------------------------------------------------------------------
*/

int LaserUnit::WriteMetaData() const
{
  return(WriteMetaData(meta_file, 0));
}

int LaserUnit::WriteMetaData(const char *filename) const
{
  return(WriteMetaData(filename, 0));
}

int LaserUnit::WriteMetaData(int write_sub_units) const
{
  return(WriteMetaData(meta_file, write_sub_units));
}

int LaserUnit::WriteMetaData(const char *filename, int write_sub_units) const
{
  FILE       *fd;
  char       *file;
  int        indent;
  LaserUnit::const_iterator subunit;

/* If the data is organised in a tiled block, the laser unit class is a dummy
 * container class. Meta data of this class is not written, but the meta data
 * of the sub units (the tiles) are written.
 */

  if (data_org == TileWise) {
    if (write_sub_units) 
      for (subunit=begin(); subunit!=end(); subunit++)
        (void) subunit->WriteMetaData();
  }

/* Write the meta data in case of a strip with or without parts or tiles */

  else {

/* Open the file */

    fd = fopen(filename, "w");
    if (fd == NULL) {
      fprintf(stderr, "Error opening strip database %s\n", filename);
      return(0);
    }

/* Write the meta data */

    indent = 0;
    BNF_Write_String(fd, "laserstrip", indent, NULL);
    indent += 2;
    if (name) BNF_Write_String(fd, "name", indent, name);
    if (point_file) BNF_Write_String(fd, "points", indent, point_file);
    if (tin_file) BNF_Write_String(fd, "tin", indent, tin_file);
    if (flight_path_file) BNF_Write_String(fd, "flight_path", indent,
    					   flight_path_file);
    if (strip_axis != NULL) {
      BNF_Write_String(fd, "stripaxis", indent, NULL);
      BNF_Write_Double(fd, "normal_direction", indent+2,
	                   strip_axis->GetNormalDirection() * 45.0 / atan(1.0), "%.6f");
      BNF_Write_Double(fd, "distance_to_origin", indent+2,
	                   strip_axis->DistanceToOrigin(), "%.3f");
      BNF_Write_String(fd, "endstripaxis", indent, NULL);
    }
    file = scanlines.ScanLinesFile();
    if (file) BNF_Write_String(fd, "scanlines", indent, file);
    if (tie_points_file)
      BNF_Write_String(fd, "tie_points", indent, tie_points_file);
    if (control_points_file)
      BNF_Write_String(fd, "control_points", indent, control_points_file);
    LaserPointsInfo::WriteMetaData(fd, indent);
    strip_transform.Orientation().Write(fd, indent);
    for (subunit=begin();
         subunit!=end();
         subunit++) {
      if (subunit->DataOrganisation() & TileWise) {
        BNF_Write_String(fd, "strip_tile", indent, subunit->MetaDataFile());
        if (write_sub_units) (void) subunit->WriteMetaData();
      }
      else if (!subunit->IsCompleteStrip()) {
        BNF_Write_String(fd, "strip_part", indent, subunit->MetaDataFile());
        if (write_sub_units) (void) subunit->WriteMetaData();
      }
    }
    indent -= 2;
    BNF_Write_String(fd, "endlaserstrip", indent, NULL);

/* Close the file and return */

    fclose(fd);
  }
  return(1);
}

/*
--------------------------------------------------------------------------------
                            Derive and set some names
--------------------------------------------------------------------------------
*/

char *LaserUnit::DeriveMetaDataFileName(const char *directory)
{
  switch (data_org) {
    case StripWise:
    case StripTileWise:
    default:
      return(LaserDataFiles::DeriveMetaDataFileName(directory, ".strip"));
    case TileWise:
      fprintf(stderr, "Warning: no meta data file at LaserUnit level in case of block tiles.\n");
      return(NULL);
  }
}

char *LaserUnit::DeriveScanLinesFileName(const char *directory)
{
  if (!name)
    if (!DeriveName()) return(NULL); /* We need a name */
  scanlines.SetScanLinesFile(ComposeFileName(directory, name, "scanlines"));
  return(scanlines.ScanLinesFile());
}

char *LaserUnit::DeriveFlightPathFileName(const char *directory)
{
  char *name_fp;
  if (!name)
    if (!DeriveName()) return(NULL); // We need a name
  if (flight_path_file) free(flight_path_file);
  name_fp = (char *) malloc(strlen(name) + 4);
  sprintf(name_fp, "%s_fp", name);
  flight_path_file = ComposeFileName(directory, name_fp, ".pos3d");
  free(name_fp);
}

void LaserUnit::SetFlightPathFileName(const char *fp_name)
{
  StringCopy(&flight_path_file, fp_name);
}

/*
--------------------------------------------------------------------------------
                      Read and write the flight path points
--------------------------------------------------------------------------------
*/

int LaserUnit::ReadFlightPath()
{
  // Check if there is a flight path file
  if (!flight_path_file) {
    fprintf(stderr, "Error: There is no flight path information in strip %s\n",
            name);
    return(0);
  }

  // Read the file
  if (!flight_path.Read(flight_path_file)) {
    fprintf(stderr, "Error reading flight path of strip %s from file %s\n",
            name, flight_path_file);
    return(0);
  }

  return(1);
}

int LaserUnit::WriteFlightPath() const
{
  // Check if there is a flight path file
  if (!flight_path_file) {
    fprintf(stderr, "Error: There is no flight path file name in strip %s\n",
            name);
    return(0);
  }
  
  // Check if there is data
  if (flight_path.empty()) {
    fprintf(stderr, "Error: the flight path contains no points in strip %s\n",
            name);
    return(0);
  }

  // Read the file
  if (!flight_path.Write(flight_path_file)) {
    fprintf(stderr, "Error writing flight path of strip %s from file %s\n",
            name, flight_path_file);
    return(0);
  }

  return(1);
}

/*
--------------------------------------------------------------------------------
            Generate a single sub unit containing the whole unit
--------------------------------------------------------------------------------
*/

LaserSubUnit LaserUnit::CompleteStrip() const
{
  LaserSubUnit  subunit;
  ImagedData    *imaged_data;
  int           num, file_id;
  unsigned char max_num_attributes;
  double        x_off, y_off, z_off;
  FILE          *fd;

/* Copy all relevant meta data from the laser unit class */

  subunit.Initialise();
  subunit.SetName(name);
  subunit.SetPointFile(point_file);
  subunit.SetTINFile(tin_file);
  subunit.SetScanLineSubSet(-1, -1);
  subunit.SetBounds(bounds);
  imaged_data = subunit.ImagedHeight();
  imaged_data->SetImageFile(imaged_height.ImageFile());
  imaged_data->SetGridFile(imaged_height.GridFile());
  imaged_data = subunit.ImagedReflectance();
  imaged_data->SetImageFile(imaged_reflectance.ImageFile());
  imaged_data->SetGridFile(imaged_reflectance.GridFile());
  imaged_data = subunit.ImagedColour();
  imaged_data->SetImageFile(imaged_colour.ImageFile());
  imaged_data->SetGridFile(imaged_colour.GridFile());
  subunit.SetPointDensity(point_density);
  subunit.SetBoundaryFile(boundary_file);
  subunit.Scanner() = scanner;

// Get additional information from the header of the point data file

  if (BNF_FileExists(point_file)) {
#ifdef windows
    if ((fd = Open_Compressed_File(point_file, "rb")) == NULL) {
#else
    if ((fd = Open_Compressed_File(point_file, "r")) == NULL) {
#endif
      fprintf(stderr, "Warning: Could not read header information of file %s\n",
              point_file);
    }
    else {
      file_id = subunit.ReadHeader(fd, &num, &x_off, &y_off, &z_off,
                                   &max_num_attributes);
      Close_Compressed_File(fd);
    }
  }

/* Signal that this sub unit contains a complete strip */

  subunit.DataOrganisation() = StripWise;
  subunit.SetAsCompleteStrip();

  return(subunit);
}

/*
--------------------------------------------------------------------------------
                    Add data of laser points to an image 
--------------------------------------------------------------------------------
*/

void LaserUnit::ImageData(Image &image, Image &dist, const DataGrid &grid,
                          ImagedDataType datatype, int interpolation_method,
                          double max_mesh_size)
{
  LaserUnit::iterator subunit;

/* Collect data in all sub units */

  for (subunit=begin(); subunit!=end(); subunit++) {
    if (subunit->DataOrganisation() == StripWise && Name() != NULL &&
        subunit->Name() != NULL) {
      if (size() > 1)
        printf("Imaging strip part %s\r", subunit->Name());
      else
        printf("Imaging strip %s\r", Name());
    }
    else if (DataOrganisation() == TileWise) {
      printf("Imaging tile %s\r", subunit->Name());
    }
    subunit->ImageData(image, dist, grid, datatype, interpolation_method,
                        max_mesh_size);
  }
  printf("\n");
  
}
 
/*
--------------------------------------------------------------------------------
                    Derive the data bounds of the unit
--------------------------------------------------------------------------------
*/

DataBoundsLaser LaserUnit::DeriveDataBounds(int use_known_bounds)
{
  LaserUnit::iterator subunit;

  // Check if bounds are already known
  if (use_known_bounds && bounds.XYZBoundsSet()) return(bounds);

  // Collect the bounds by a loop over the sub units
  bounds.Initialise();
  for (subunit=begin(); subunit!=end(); subunit++)
    bounds.Update(subunit->DeriveDataBounds(use_known_bounds));

  return(bounds);
}

/*
--------------------------------------------------------------------------------
          Derive the zero, first and second order non central moments
--------------------------------------------------------------------------------
*/

void LaserUnit::Moments(double &mt00, double &mt10, double &mt01,
                        double &mt20, double &mt11, double &mt02)
{
  LaserUnit::iterator subunit;
  int                 delete_points;
  double              m00, m10, m01, m20, m11, m02;

  mt00 = mt10 = mt01 = mt20 = mt11 = mt02 = 0.0;
  for (subunit=begin(); subunit!=end(); subunit++) {

    // See if the points are already read
    if (subunit->size()) delete_points = 0;
    else {
      delete_points = 1;
      if (!subunit->Read()) {
        fprintf(stderr, "Error reading points in LaserUnit::Moments\n");
        return;
      }
    }

    // Derive moments of sub unit
    subunit->Moments(m00, m10, m01, m20, m11, m02);

    // Update unit moments
    mt00 += m00;
    mt10 += m10;
    mt01 += m01;
    mt20 += m20;
    mt11 += m11;
    mt02 += m02;

    // Erase the points again
    if (delete_points) subunit->ErasePoints();
  }
}

/*
--------------------------------------------------------------------------------
                         Derive the scan lines
--------------------------------------------------------------------------------
*/

void LaserUnit::DeriveScanLines(double max_topography_height,
                                int num_scanlines_flight_path,
                                double approximate_flight_height)
{
  double median_dist;
  LaserUnit::iterator strip_part;
  bool erase_points=false;
  Position3D flight_path_point;
  int num_scanlines_summed=0;

/* Check if we are dealing with a laser strip */

  if (data_org != StripWise) {
    fprintf(stderr, "Error: Unit of laser points is not a laser strip.\n");
    fprintf(stderr, "       Scan lines will not be derived.\n");
    return;
  }

  // Read points if required
  if (begin()->empty()) {
    if (!begin()->Read(begin()->PointFile(), false)) {
      printf("Error reading points from file %s\n", begin()->PointFile());
      return;
    }
    erase_points = true;
  }
  
/* Determine the median point distance between the first 100 point pairs of
 * the first strip part.
 */

  median_dist = begin()->MedianInterPointDistance(100);

/* Determine the scan lines of the different strip parts */

  if (!scanlines.empty()) scanlines.erase(scanlines.begin(), scanlines.end());
  for (strip_part=begin(); strip_part!=end(); strip_part++) {
    printf("Strip %s, part %s\r", Name(), strip_part->Name());
    strip_part->DeriveScanLines(scanlines, median_dist, max_topography_height,
                                approximate_flight_height, num_scanlines_flight_path,
                                flight_path, flight_path_point, num_scanlines_summed);
    if (strip_part == begin() && erase_points) begin()->ErasePoints();
  }
  printf("\n");
}

/*
--------------------------------------------------------------------------------
                         Define the tiles of a unit
--------------------------------------------------------------------------------
*/

void LaserUnit::DefineTiles(LaserDataOrganisation org, double width,
                            double height, double border)
{
  int             ir, ic, numrows, numcols;
  DataBoundsLaser tile_bounds;

  data_org = org;
  numrows = (int) ((bounds.YRange()-0.01) / height) + 1;
  numcols = (int) ((bounds.XRange()-0.01) / width) + 1;

  for (ir=0; ir<numrows; ir++) {
    for (ic=0; ic<numcols; ic++) {
      tile_bounds.SetMinimumX(bounds.Minimum().X() + ic * width);
      tile_bounds.SetMaximumX(tile_bounds.Minimum().X() + width);
      tile_bounds.SetMaximumY(bounds.Maximum().Y() - ir * height);
      tile_bounds.SetMinimumY(tile_bounds.Maximum().Y() - height);
      push_back(LaserSubUnit(org, ir, ic, tile_bounds, border));
    }
  }
}

/*
--------------------------------------------------------------------------------
                Determine number of tile rows and columns
--------------------------------------------------------------------------------
*/

int LaserUnit::NumberOfTileRows() const
{
  int max_row=0;
  LaserUnit::const_iterator tile;
  
  if (empty()) return 0;
  if (DataOrganisation() != TileWise) {
    printf("Warning: block is not organised in tiles.\n");
    return 0;
  }
  for (tile=begin(); tile!=end(); tile++)
    if (tile->TileRow() > max_row) max_row = tile->TileRow();
  return max_row+1;
}

int LaserUnit::NumberOfTileColumns() const
{
  int max_column=0;
  LaserUnit::const_iterator tile;
  
  if (empty()) return 0;
  if (DataOrganisation() != TileWise) {
    printf("Warning: block is not organised in tiles.\n");
    return 0;
  }
  for (tile=begin(); tile!=end(); tile++)
    if (tile->TileColumn() > max_column) max_column = tile->TileColumn();
  return max_column+1;
}

/*
--------------------------------------------------------------------------------
                         Set the names of the tiles
--------------------------------------------------------------------------------
*/

void LaserUnit::SetTileNames(const char *rootname, const char *directory,
                             bool use_subdirs, bool six_digits)
{
  LaserUnit::iterator tile;
  char                *subdir_checked, *subdirectory;
  int                 len;
  
  // Allocate strings for creation of subdirectories for every row of tiles
  if (use_subdirs) {
    subdir_checked = (char *) calloc(NumberOfTileRows()+1, sizeof(char));
    subdirectory = (char *) malloc(strlen(directory)+8);
    len = strlen(directory);
    if (directory[len-1] == '/') sprintf(subdirectory, "%srow", directory);
    else sprintf(subdirectory, "%s/row", directory);
    len = strlen(subdirectory);
  }

  for (tile=begin(); tile!=end(); tile++)
    if (use_subdirs) {
      // Compose subdirectory name
      sprintf(subdirectory+len, "%3d", tile->TileRow());
      if (subdirectory[len] == ' ') subdirectory[len] = '0';
      if (subdirectory[len+1] == ' ') subdirectory[len+1] = '0';
      // Make sure the directory exists
      if (!subdir_checked[tile->TileRow()]) {
        if (!DirectoryExists(subdirectory)) {
	 #ifdef windows	
	  // Directory does not exist
          if (!CreateDirectory(subdirectory, NULL)) {
            printf("Error creating subdirectory %s\n", subdirectory);
            free(subdirectory); free(subdir_checked);
            return;
          }
	  #else
           printf ("creation of subdirs only implementedin windows so far\n");
	   free(subdirectory); free(subdir_checked);
            return;
	  #endif
          
        }
        subdir_checked[tile->TileRow()] = true;
      }
      // Set file names in subdirectory
      tile->SetFileNames(rootname, subdirectory, &*begin(), six_digits);             
    }
    else
      tile->SetFileNames(rootname, directory, &*begin(), six_digits);

  if (use_subdirs) {free(subdirectory); free(subdir_checked);}
}

/*
--------------------------------------------------------------------------------
                         Insert points into tiles
--------------------------------------------------------------------------------
*/

void LaserUnit::InsertIntoTiles(LaserPoints &pts,
                                bool preserve_multiple_reflections)
{
  LaserUnit::iterator tile;

  for (tile=begin(); tile!=end(); tile++)
    tile->InsertIntoTile(pts, preserve_multiple_reflections);
}

void LaserUnit::InsertIntoTilesWithoutBorder(LaserPoints &pts,
                                             bool preserve_multiple_reflections)
{
  LaserUnit::iterator tile, first_pulse_tile;
  double              min_X, max_Y, width, height;
  int                 num_row, num_column, tile_row, tile_column;
  LaserPoints         new_points;
  LaserPoints::iterator pt;
  bool                  preserve;
    
  if (empty()) return; // No tiles available
  
  // Check if all tiles are there
  num_row = NumberOfTileRows();
  num_column = NumberOfTileColumns();
  if (num_row * num_column != size()) {
    printf("Error: function InsertIntoTilesWithoutBorder can only be used\n");
    printf("       if all tiles in a rectangle are available\n");
    return;
  }
  
  // Check if preservation of multiple reflections is possible
  if (preserve_multiple_reflections && 
      !pts.begin()->HasAttribute(PulseCountTag)) {
    printf("Warning: Multiple reflections can not be preserved if no pulse\n");
    printf("         count attributes are available\n");
    preserve = false;
    return;
  }
  else preserve = preserve_multiple_reflections;
  
  // Get information on tile locations
  tile = begin();
  if (tile->TileRow() != 0 || tile->TileColumn() != 0) {
    printf("Error: function InsertIntoTilesWithoutBorder can only be used\n");
    printf("       when tiles are stored in sequence\n");
    return;
  }
  min_X = tile->TileBounds().Minimum().X() - 
          tile->TileColumn() * tile->TileBounds().XRange();
  max_Y = tile->TileBounds().Maximum().Y() +
          tile->TileRow() * tile->TileBounds().YRange();
  width = tile->TileBounds().XRange();
  height = tile->TileBounds().YRange();
  
  // Put all points in the right tile
  tile = end();
  for (pt=pts.begin(); pt!=pts.end(); pt++) {
    if (!preserve_multiple_reflections ||
        pt->IsPulseType(FirstPulse)) {
      tile_row = (int) ((max_Y - pt->Y()) / height);
      tile_column = (int) ((pt->X() - min_X) / width);
      if (tile_row >= 0 && tile_row < num_row &&
          tile_column >= 0 && tile_column < num_column)
        tile = begin() + num_column * tile_row + tile_column;
      else
        tile = end();
      first_pulse_tile = tile;
    }
    else {
      tile = first_pulse_tile;
    }
    if (tile != end()) tile->push_back(*pt);
  }

  // Update the tiles on disk with the additional points
  for (tile=begin(); tile!=end(); tile++) {
    if (tile->size()) {
      // If the tile already exists, merge the two point sets
      if (BNF_FileExists(tile->PointFile())) {
        // Temporarily store points in new_points buffer
        new_points.swap(tile->LaserPointsReference());
        // Read old points
        if (!tile->Read(tile->PointFile(), false)) {
          printf("Error re-reading tile %s\n", tile->PointFile());
          return;
        }
        // Add new points
        tile->insert(tile->end(), new_points.begin(), new_points.end());
        // Erase additional points
        new_points.ErasePoints();
      }
      // Write tile with additional points
      tile->Write(false);
      // Erase points of tile
      tile->ErasePoints();
    }
  }
}

/*
--------------------------------------------------------------------------------
                            Remove empty tiles
--------------------------------------------------------------------------------
*/

void LaserUnit::RemoveEmptyTiles()
{
  LaserUnit::iterator tile, filled_tile;

/* Tiles are considered empty if the point file of a tile is non-existent */

  for (tile=begin(), filled_tile=begin(); tile!=end(); tile++) {
    if (BNF_FileExists(tile->PointFile())) {
      *filled_tile = *tile;
      filled_tile++;
    }
  }
  if (filled_tile != end()) erase(filled_tile, end());
}

/*
--------------------------------------------------------------------------------
                             Erase all points
--------------------------------------------------------------------------------
*/

void LaserUnit::ErasePoints()
{
  LaserUnit::iterator subunit;

  for (subunit=begin(); subunit!=end(); subunit++) subunit->ErasePoints();
}

/*
--------------------------------------------------------------------------------
                      Read all points up to a maximum
--------------------------------------------------------------------------------
*/

int LaserUnit::ReadPoints(int max_num_pts)
{
  LaserUnit::iterator subunit;
  int                 num_read=0;
  
  for (subunit=begin(); subunit!=end() && num_read <= max_num_pts; subunit++) {
    if (!subunit->Read()) return -1;
    num_read += subunit->size();
  }

  if (num_read > max_num_pts) {
    ErasePoints();
    return -1;
  }
  return num_read;
}

/*
--------------------------------------------------------------------------------
               Extract the points in the overlap with another strip
--------------------------------------------------------------------------------
*/

int LaserUnit::Overlap(LaserUnit &strip2, LaserPoints &overlap,
                       double gridsize)
{
  double                xrange, yrange;
  int                   num_rows, num_cols, row, column, value;
  ImagedData            coverage;
  LaserUnit::iterator   part;
  bool                  delete_data;
  LaserPoints::iterator point;
  ImagePoint            imagepoint;
  
  // Make sure we have bounds
  DeriveDataBounds(1);
  
  // Initialise a coverage image
  xrange = bounds.XRange();   yrange = bounds.YRange();
  if (xrange == 0.0 || yrange == 0.0) {
    fprintf(stderr, "Error: Range of X- and/or Y-coordinates equals zero.\n");
    fprintf(stderr, "       No image created!\n");
    exit(0);
  }
  num_rows = (int) (yrange / gridsize);
  if (num_rows < yrange / gridsize) num_rows++;
  num_cols = (int) (xrange / gridsize);
  if (num_cols < xrange / gridsize) num_cols++;
  coverage.SetGrid(bounds.HeightDataGrid(gridsize));
  coverage.NewImage(num_rows, num_cols, VFF_TYP_1_BYTE, 1);
  coverage.ClearImage();
  printf("Initialised image of %d rows and %d columns\n", num_rows, num_cols);
  
  // Mark grid cells with 1 if there is data of strip 1
  for (part=begin(); part!=end(); part++) {
    delete_data = (part->size() == 0);
    if (part->size() == 0) {
      if (!part->Read(part->PointFile(), false)) {
        fprintf(stderr, "Error reading laser points from file %s\n",
                part->PointFile());
        exit(0);
      }
    }
    for (point=part->begin(); point!=part->end(); point++) {
      imagepoint = point->Map_Into_Image(1, coverage.Grid()->ImageGridReference());
      row    = (int) (imagepoint.Row() + 0.5);
      column = (int) (imagepoint.Column() + 0.5);
      if (coverage.Inside(row, column))
        *(coverage.Pixel(row, column)) = 1;
    }
    if (delete_data) part->ErasePoints();
  }

  // Collect all points of strip 2 and mark all corresponding coverage 
  // cells with 2 or 3
  for (part=strip2.begin(); part!=strip2.end(); part++) {
    delete_data = (part->size() == 0);
    if (part->size() == 0) {
      if (!part->Read(part->PointFile(), false)) {
        fprintf(stderr, "Error reading laser points from file %s\n",
                part->PointFile());
        exit(0);
      }
    }
    for (point=part->begin(); point!=part->end(); point++) {
      imagepoint = point->Map_Into_Image(1, coverage.Grid()->ImageGridReference());
      row    = (int) (imagepoint.Row() + 0.5);
      column = (int) (imagepoint.Column() + 0.5);
      if (coverage.Inside(row, column)) {
        value = *(coverage.Pixel(row, column));
        if (value == 0) {
          *(coverage.Pixel(row, column)) = 2;
        }
        else if (value == 1 || value == 3) {
          overlap.push_back(*point);
          (overlap.end()-1)->Label(2);
          if (value == 1) *(coverage.Pixel(row, column)) = 3;
        }
      }
    }
    if (delete_data) part->ErasePoints();
  }

  // Collect all points of strip 1 in the overlap
  for (part=begin(); part!=end(); part++) {
    delete_data = (part->size() == 0);
    if (part->size() == 0) {
      if (!part->Read(part->PointFile(), false)) {
        fprintf(stderr, "Error reading laser points from file %s\n",
                part->PointFile());
        exit(0);
      }
    }
    for (point=part->begin(); point!=part->end(); point++) {
      imagepoint = point->Map_Into_Image(1, coverage.Grid()->ImageGridReference());
      row    = (int) (imagepoint.Row() + 0.5);
      column = (int) (imagepoint.Column() + 0.5);
      if (coverage.Inside(row, column)) {
        if (*(coverage.Pixel(row, column)) == 3) {
          overlap.push_back(*point);
          (overlap.end()-1)->Label(1);
        }
      }
    }
    if (delete_data) part->ErasePoints();
  }

  return (int) overlap.size();
}

/*
--------------------------------------------------------------------------------
         Derive the strip axis from the first and last 5000 points
--------------------------------------------------------------------------------
*/

Line2D LaserUnit::DetermineStripAxis(LaserPoint &start_point, LaserPoint &end_point)
{
  LaserUnit::iterator part;
  DataBoundsLaser     part_bounds;

  // Get the centre of the first 5000 points of the strip
  part = begin();
  if (!part->Read(part->PointFile(), false)) {
    printf("Error reading strip part from file %s\n", begin()->PointFile());
    exit(0);
  }
  part->DeriveDataBounds(1);
  part_bounds = part->Bounds();
  if (part->size() > 5000) part->erase(part->begin() + 5000, part->end());
  part->DeriveDataBounds(0);
  start_point = part->Bounds().MidPoint();
  part->SetBounds(part_bounds); // Leave proper part bounds
  part->ErasePoints();
  
  // Get the centre of the last 5000 points of the strip
  part = end() - 1;
  if (!part->Read(part->PointFile(), false)) {
    printf("Error reading strip part from file %s\n", begin()->PointFile());
    exit(0);
  }
  part->DeriveDataBounds(1);
  part_bounds = part->Bounds();
  if (part->size() > 5000) part->erase(part->begin(), part->end()-5000);
  part->DeriveDataBounds(0);
  end_point = part->Bounds().MidPoint();
  part->SetBounds(part_bounds);
  part->ErasePoints();

  // Derive the strip axis
  strip_axis = new Line2D(start_point.Position2DOnly(),
                          end_point.Position2DOnly());
                      
  // Make sure the start point has the lowest scalar along the axis
  if (strip_axis->Scalar(start_point.Position2DOnly()) >
      strip_axis->Scalar(end_point.Position2DOnly()))
    strip_axis->SwapNormal();
    
  return *strip_axis;
}

/*
--------------------------------------------------------------------------------
         Derive the strip axis by fitting a line to all points
--------------------------------------------------------------------------------
*/

Line2D LaserUnit::DetermineStripAxis(bool verbose)
{
  LaserUnit::iterator   part;
  LaserPoints::iterator point;
  Line3D                strip_axis_3D;
  bool                  erase_points;

  // First determine 3D line (line fitting not implemented for 2D ;-)
  for (part=begin(); part!=end(); part++) {
    if (part->empty()) {
      erase_points = true;
      if (!part->Read(part->PointFile(), false)) {
        printf("Error reading %s in function LaserUnit::DetermineStripAxis()\n",
               part->PointFile());
        return Line2D();
      }
    }
    if (verbose) {
	  if (part == end()-1) printf("Part %s\n", part->Name());
	  else {printf("Part %s\r", part->Name()); fflush(stdout);}
    }
    for (point=part->begin(); point!=part->end(); point++)
      strip_axis_3D.AddPoint(point->Position3DRef(), false);
      
    if (erase_points) part->ErasePoints();
  }
  strip_axis_3D.Recalculate();
  
  // Return projection of 3D line onto the XOY plane
  strip_axis = new Line2D();
  *strip_axis = strip_axis_3D.ProjectOntoXOYPlane();
  return *strip_axis;
}

/*
--------------------------------------------------------------------------------
               Extract the points in the overlap with another strip
               and store the points on disk
--------------------------------------------------------------------------------
*/

LaserUnit & LaserUnit::Overlap(LaserUnit &strip2, double gridsize,
                               int max_num_pts, char *output_directory)
{
  double                xrange, yrange, scalar, end_scalar2;
  int                   num_rows, num_cols, row, column, value, strip_name_len,
                        i, interval, low_index, high_index, index, num_data,
						point_set_num=0;
  ImagedData            coverage;
  LaserUnit::iterator   part, overlap_part1, overlap_part2;
  bool                  delete_data;
  LaserPoints::iterator point;
  ImagePoint            imagepoint;
  LaserUnit             *overlap = new LaserUnit(), overlap1;
  LaserSubUnit          overlap_part;
  vector<double>        scalar_bounds;
  vector<double>::iterator scalar_bound;
  char                  *name, point_set_name[20], *ch;
  bool                  verbose=false, found;
  DataBoundsLaser       overlap_bounds;
  LaserBlock            overlaps1;

//---- Compose the overlap name ------------------------------------------------

  name = (char *) malloc(40+strlen(Name())+strlen(strip2.Name()));
  sprintf(name, "overlap_%s_%s", Name(), strip2.Name());
  strip_name_len = strlen(name);
  overlap->SetName(name);
  overlap->DeriveMetaDataFileName(output_directory);
  overlap->DataOrganisation() = StripWise;

//---- Initialise the coverage image -------------------------------------------

  // Make sure we have bounds
  printf("\nChecking data bounds in both strips\n");
  DeriveDataBounds(1);
  strip2.DeriveDataBounds(1);
  
  // Check that the bounding boxes overlap
  if (!OverlapXY(bounds, strip2.Bounds())) {
  	printf("Error: the bounding boxes of strips %s and %s don't overlap\n",
	       Name(), strip2.Name());
	return *overlap;
  }
  
  // Set overlap bounds
  overlap_bounds.SetMinimumX(fmax(bounds.Minimum().X(), strip2.Bounds().Minimum().X()));
  overlap_bounds.SetMaximumX(fmin(bounds.Maximum().X(), strip2.Bounds().Maximum().X()));
  overlap_bounds.SetMinimumY(fmax(bounds.Minimum().Y(), strip2.Bounds().Minimum().Y()));
  overlap_bounds.SetMaximumY(fmin(bounds.Maximum().Y(), strip2.Bounds().Maximum().Y()));
  overlap_bounds.SetMinimumZ(fmax(bounds.Minimum().Z(), strip2.Bounds().Minimum().Z()));
  overlap_bounds.SetMaximumZ(fmin(bounds.Maximum().Z(), strip2.Bounds().Maximum().Z()));
 
  // Calculate number of rows and columns
  xrange = overlap_bounds.XRange();   yrange = overlap_bounds.YRange();
  if (xrange == 0.0 || yrange == 0.0) {
    fprintf(stderr, "Error: Range of X- and/or Y-coordinates equals zero.\n");
    fprintf(stderr, "       No image created!\n");
    exit(0);
  }
  num_rows = (int) (yrange / gridsize);
  if (num_rows < yrange / gridsize) num_rows++;
  num_cols = (int) (xrange / gridsize);
  if (num_cols < xrange / gridsize) num_cols++;
  if (verbose) printf("X-range %.2f, Y-range %.2f, Grid size %.2f\n",
                      xrange, yrange, gridsize);
  
  // Initialise the image
  printf("Allocating image of %d x %d pixels\n", num_rows, num_cols);
  coverage.SetGrid(overlap_bounds.HeightDataGrid(gridsize));
  coverage.NewImage(num_rows, num_cols, VFF_TYP_1_BYTE, 1);
  if (coverage.GetImage() == NULL) {
    printf("Aborting LaserUnit::Overlap\n");
    exit(0);
  }
  coverage.ClearImage();
  
//---- Mark the cells covered by strip 1 ---------------------------------------

  for (part=begin(); part!=end(); part++) {
    if (verbose) printf("Imaging strip 1, part %s\n", part->Name());
    else printf("Imaging strip 1, part %s\r", part->Name());
    
	// Check if the bounding box of this part of strip 1 is within the overlap
	// of the two strips
    if (!OverlapXY(part->Bounds(), overlap_bounds)) continue;
 
    // Read the next strip part
    delete_data = (part->size() == 0);
    if (part->size() == 0) {
      if (!part->Read(part->PointFile(), false)) {
        fprintf(stderr, "Error reading laser points from file %s\n",
                part->PointFile());
        exit(0);
      }
    }
    // Before starting, check if the data has scalars of their position
    // along the strip axis. This will be needed later on.
    if (part == begin()) {
	  if (!part->begin()->HasAttribute(ScalarTag)) {
        printf("Error: The points don't have scalars from sorting along the strip axis\n");
        printf("       Use the programme sortstrip with option -s before deriving strip overlaps\n");
        exit(0);
      }
    }
    // Now image the points of the strip part
    for (point=part->begin(); point!=part->end(); point++) {
      imagepoint = point->Map_Into_Image(1, coverage.Grid()->ImageGridReference());
      row    = (int) (imagepoint.Row() + 0.5);
      column = (int) (imagepoint.Column() + 0.5);
      if (coverage.Inside(row, column))
        *(coverage.Pixel(row, column)) = 1;
    }
    if (delete_data) part->ErasePoints();
  }
  if (verbose) printf("Points of strip 1 put in coverage image\n");
  else printf("\n");
  
//---- Collect points of strip 2 in the overlap --------------------------------

  overlap_part.DataOrganisation() = StripWise;
  for (part=strip2.begin(); part!=strip2.end(); part++) {
    if (verbose) printf("Collecting points of strip 2, part %s\n", part->Name());
    else printf("Collecting points of strip 2, part %s\r", part->Name());
    
	// Check if the bounding box of this part of strip 2 is within the overlap
	// of the two strips
    if (!OverlapXY(part->Bounds(), overlap_bounds)) continue;
    
    // Read part of strip 2
    delete_data = (part->size() == 0);
    if (part->size() == 0) {
      if (!part->Read(part->PointFile(), false)) {
        fprintf(stderr, "Error reading laser points from file %s\n",
                part->PointFile());
        exit(0);
      }
    }
    
    // Store the scalars of the first and last point of strip 2
    if (scalar_bounds.empty())
      scalar_bounds.push_back(part->begin()->FloatAttribute(ScalarTag));
    end_scalar2 = (part->end()-1)->FloatAttribute(ScalarTag);
      
    // Check if points are in a cell with points of strip 1
    for (point=part->begin(); point!=part->end(); point++) {
      imagepoint = point->Map_Into_Image(1, coverage.Grid()->ImageGridReference());
      row    = (int) (imagepoint.Row() + 0.5);
      column = (int) (imagepoint.Column() + 0.5);
      if (coverage.Inside(row, column)) {
        value = *(coverage.Pixel(row, column));
        if (value == 0) {
          *(coverage.Pixel(row, column)) = 2;
        }
        // Store the point in the overlap strip part
        else if (value == 1 || value == 3) {
          overlap_part.push_back(*point);
          (overlap_part.end()-1)->Label(strip2.StripNumber());
          if (value == 1) *(coverage.Pixel(row, column)) = 3;
          // If the maximum overlap part size is reached, write the points
          // to disk and save the part bound
          if (overlap_part.size() >= max_num_pts / 2) {
            // Compose overlap part name
            sprintf(name+strip_name_len, "_%3d", overlap->size()+1);
            for (i=strip_name_len; i<strlen(name); i++)
              if (name[i] == ' ') name[i] = '0';
            overlap_part.SetName(name);
            overlap_part.DeriveMetaDataFileName(output_directory);
            overlap_part.DerivePointFileName(output_directory);
            // Write and erase points of the overlap part
            overlap_part.Write(false);
            if (verbose) printf("Wrote overlap part with %d points\n", overlap_part.size());
            overlap_part.WriteMetaData();
            overlap_part.ErasePoints();
            // Add part to overlap LaserUnit
            overlap->push_back(overlap_part);
            // Save the part bound
            scalar_bounds.push_back(point->FloatAttribute(ScalarTag));
          }
        }
      }
    }
    if (delete_data) part->ErasePoints();
  }
  scalar_bounds.push_back(end_scalar2);
  if (!verbose) printf("\n");
  
  // Write coverage in case of debugging
  if (verbose) {
    printf("Writing coverage to coverage.xv\n");
    coverage.WriteImage("coverage.xv");
  }

  // If one part, store the overlap point file name under the strip itself
  if (overlap->size() == 0) { // Store in strip if only one part
    overlap->DerivePointFileName(output_directory);
    overlap_part.SetAsCompleteStrip();
    overlap_part.SetName(name);
    overlap_part.DerivePointFileName(output_directory);
  }      
  else { // Otherwise, add point file to the last overlap part
    sprintf(name+strip_name_len, "_%3d", overlap->size()+1);
    for (i=strip_name_len; i<strlen(name); i++) if (name[i] == ' ') name[i] = '0';
    overlap_part.SetName(name);
    overlap_part.DeriveMetaDataFileName(output_directory);
    overlap_part.DerivePointFileName(output_directory);
    overlap_part.WriteMetaData();
  }
  // Write and erase points of the overlap (last or only) part
  overlap_part.Write(false);
  if (verbose) printf("Wrote last overlap part with %d points\n", overlap_part.size());
  overlap_part.ErasePoints();
  // Add part to overlap LaserUnit
  overlap->push_back(overlap_part);
  if (verbose) printf("Points of strip 2 put in coverage image and %d strip parts\n",
                      overlap->size());

//---- Collect points of strip 1 in the overlap --------------------------------

  // Print the part bound scalars
  if (verbose) {
    printf("%d scalar bounds\n", scalar_bounds.size());
    for (i=0; i<scalar_bounds.size(); i++)
      printf("%.2f  ", scalar_bounds[i]);
    printf("\n");
  }
  
  // Create empty point containers for each overlap part to store points
  // of the parts of strip 1. Also, create a container for all point sets of 
  // parts of strip 1 that overlap with a part of strip 2.
  for (overlap_part2=overlap->begin(); overlap_part2!=overlap->end();
       overlap_part2++) {
    overlap1.push_back(LaserSubUnit());
    overlaps1.push_back(LaserUnit());
  }
    
  // Loop over all parts of strip 1
  for (part=begin(); part!=end(); part++) {
    if (verbose) printf("Collecting points of strip 1, part %s\n", part->Name());
    else printf("Collecting points of strip 1, part %s\r", part->Name());

	// Check if the bounding box of this part of strip 1 is within the overlap
	// of the two strips
    if (!OverlapXY(part->Bounds(), overlap_bounds)) continue;

    // Read the part data    
    delete_data = (part->size() == 0);
    if (part->size() == 0) {
      if (!part->Read(part->PointFile(), false)) {
        fprintf(stderr, "Error reading laser points from file %s\n",
                part->PointFile());
        exit(0);
      }
    }
    interval = 0;
    for (point=part->begin(); point!=part->end(); point++) {
      imagepoint = point->Map_Into_Image(1, coverage.Grid()->ImageGridReference());
      row    = (int) (imagepoint.Row() + 0.5);
      column = (int) (imagepoint.Column() + 0.5);
      if (coverage.Inside(row, column)) {
        if (*(coverage.Pixel(row, column)) == 3) {
          scalar = strip2.StripAxis()->Scalar(point->Position2DOnly());
          point->FloatAttribute(ScalarTag) = scalar;

          // Determine the right scalar interval
  	      // Usually it will be the same and no search is needed
          if (scalar < scalar_bounds[interval] ||
		      scalar >= scalar_bounds[interval+1]) {
		  	// If not, we're probably lucky in the adjacent interval
		  	found = false;
		  	// Check one interval ahead
		  	if (!found && interval < scalar_bounds.size()-2) {
              if (scalar >= scalar_bounds[interval+1] &&
		          scalar < scalar_bounds[interval+2]) {
		        interval++;
		        found = true;
		      }
		  	}
		  	// Check one interval back
		  	if (interval > 0) {
              if (scalar >= scalar_bounds[interval-1] &&
		          scalar < scalar_bounds[interval]) {
		        interval--;
		        found = true;
		      }
		  	}
		  	// If not, we really need to search
		  	if (!found) {
			  low_index = 0;
			  high_index = scalar_bounds.size() - 1;
		  	  if (scalar < scalar_bounds[low_index]) continue; // Before first scalar
			  if (scalar > scalar_bounds[high_index]) continue; // After last scalar
			  while (!found) {
				index = (low_index + high_index) / 2;
				if (scalar < scalar_bounds[index]) high_index = index;
				else low_index = index;
				if (high_index - low_index == 1) {
				  found = true;
				  interval = low_index;
				}
		  	  }	
		  	}
		  }
		  	
		  // Store the point in its interval container
		  overlap1[interval].push_back(*point);
        }
      }
    }
    
	// Delete data of this strip part
    if (delete_data) part->ErasePoints();
    
    // Add the points of strip 1 to those of strip 2
    num_data = 0;
    for (overlap_part1=overlap1.begin(), overlap_part2=overlap->begin(),
	     interval=0;
	     overlap_part1!=overlap1.end();
		 overlap_part1++, overlap_part2++, interval++) {
	  if (overlap_part1->size() == 0) continue; // No points in this scalar interval
	  num_data++;
	  if (num_data == 1 && verbose)
	    printf("Storing points of strip 1 to overlap part(s)");
	  if (num_data && verbose) printf(" %d", interval);
/*
	  // Read data of strip 2
      if (!overlap_part2->Read(overlap_part2->PointFile(), false)) {
        printf("Error reading overlap part of strip 2 from file %s\n",
               overlap_part2->PointFile());
        exit(0);
      }

      // Add data of strip 1
      overlap_part2->insert(overlap_part2->end(),
	                        overlap_part1->begin(), overlap_part1->end());
	  // Save the data
	  overlap_part2->Write(false);
	  // Delete all points of this overlap part
	  overlap_part1->ErasePoints();
	  overlap_part2->ErasePoints();
*/
      // Save this point set in a temporary file
      point_set_num++;
      sprintf(point_set_name, "tmp_set_%6d", point_set_num);
      ch = point_set_name + 8;
      while (*ch != 0) {if (*ch == ' ') *ch = '0'; ch++;}
      overlap_part1->SetName(point_set_name);
      overlap_part1->DerivePointFileName("./");
      overlap_part1->Write(false);
      
	  // Delete all points of this overlap part
	  overlap_part1->ErasePoints();
	  
	  // Store the meta data in the container
	  overlaps1[interval].push_back(*overlap_part1);
	}
 	if (num_data && verbose) printf("\n");
  }
  if (!verbose) printf("\n");

  // Collect the part of all point sets of strip 1 and store them
  // with the data of strip 2
  for (overlap_part2=overlap->begin(), interval=0;
	   overlap_part2!=overlap->end();
	   overlap_part2++, interval++) {
	// Read the strip 2 data of this overlap part
    if (!overlap_part2->Read(overlap_part2->PointFile(), false)) {
      printf("Error reading overlap part of strip 2 from file %s\n",
             overlap_part2->PointFile());
      exit(0);
    }

    // Loop over all point sets with strip 1 data on this interval
	for (overlap_part1=overlaps1[interval].begin();
	     overlap_part1!=overlaps1[interval].end(); overlap_part1++) {
	  // Read data from temporary file
      if (!overlap_part1->Read(overlap_part1->PointFile(), false)) {
        printf("Error reading overlap part of strip 2 from file %s\n",
               overlap_part1->PointFile());
        exit(0);
      }
      
      printf("Combining data in %s from %s\r", overlap_part2->Name(),
	         overlap_part1->Name());
	  fflush(stdout);

      // Add the data to the data of strip 2
      overlap_part2->insert(overlap_part2->end(), overlap_part1->begin(),
	                        overlap_part1->end());
	  // Erase data of strip 1
	  overlap_part1->ErasePoints();
    }
    // Save the combined data on this overlap interval
    overlap_part2->Write(false);
    // Erase the data and the containers for this interval
    overlap_part2->ErasePoints();
    overlaps1[interval].erase(overlaps1[interval].begin(),
	                          overlaps1[interval].end());
  }
  printf("\n");
  
  // Write the meta data
  overlap->WriteMetaData();
  
  // Clear image
  coverage.DeleteImage();
  return *overlap;
}


/*
--------------------------------------------------------------------------------
                Sort points along the strip axis
--------------------------------------------------------------------------------
*/

void LaserUnit::SortAlongStripAxis(char *sorted_strip_name,
                                   char *output_directory,
                                   char *working_directory,
                                   bool verbose, bool remove_scalar_tag)
{
  int part_number, strip_number, max_part_size=500000, last_part_size,
      double_count=0, part1_number, part2_number;
  char part_name[100], *ch, command[200];
  double min_scalar, max_scalar, scalar;
  bool increment1;
  LaserBlock merged_strips, strips;
  LaserBlock::iterator strip1, strip2, merged_strip;
  LaserUnit strip;
  LaserUnit::iterator part, next_part, part1, part2;
  LaserSubUnit merged_part;
  LaserPoints::iterator point1, point2, point;
  LaserPoint previous_point;
  bool store_sequence, debug=false;
  clock_t start_time;
    
  // Determine strip axis
  start_time = clock();
  if (verbose) printf("Determining strip axis\n");
  DetermineStripAxis(verbose);
  
  // Sort all strip parts along the strip axis and store the minimum and
  // maximum scalar along on the strip axis
  for (part=begin(), part_number=1; part!=end(); part++, part_number++) {
    // Read points
    if (!part->Read(part->PointFile(), false)) {
      printf("Error reading %s in function LaserUnit::SortAlongStripAxis(...)\n",
             part->PointFile());
      return;
    }
    
    if (verbose) printf("Sorting tile %s\r", part->Name());
    
    // Get rid of empty parts
    if (part->empty()) {
      if (part+1 != end()) *part = *(end()-1);
      erase(end()-1);
      part--;
      part_number--;
    }
    
    // Otherwise sort and save the points
    else {
      // Sort points along the strip axis
      part->SortAlongLine(*StripAxis(), false, false);
    
      // Store minimum and maximum scalar
      part->Bounds().SetMinimumValue(ScalarTag,
                                     part->begin()->FloatAttribute(ScalarTag));
      part->Bounds().SetMaximumValue(ScalarTag,
                                     (part->end()-1)->FloatAttribute(ScalarTag));
      
      // Write sorted points
      sprintf(part_name, "A_%6d", part_number);
      ch = part_name; while (*ch) { if (*ch == ' ') *ch = '0'; ch++; }
      part->SetName(part_name);
      part->DerivePointFileName(working_directory);
      part->Write(part->PointFile(), false, false);
    
      // Erase the points
      part->ErasePoints(false);
    }
  }
  if (debug) printf("\nSorted %d parts and saved results in A set\n", size());

  // Compose sequences of tiles such that the maximum scalar of a tile is
  // smaller than the minimum scalar of the next tile
  while (!empty()) {
    // Initialse a tile sequence if it is empty with the tile with the smallest
    // minimum scalar
    if (strip.empty()) {
      min_scalar = begin()->Bounds().Minimum().FloatAttribute(ScalarTag);
      next_part   = begin();
      for (part=begin()+1; part!=end(); part++) {
        if (part->Bounds().Minimum().FloatAttribute(ScalarTag) < min_scalar) {
          min_scalar = part->Bounds().Minimum().FloatAttribute(ScalarTag);
          next_part = part;
        }
      }
      strip.push_back(*next_part);
      erase(next_part);
    }
    // Find the tile with the smallest minimum scalar larger than the 
    // maximum scalar of the previous tile
    max_scalar = (strip.end()-1)->Bounds().Maximum().FloatAttribute(ScalarTag);
    min_scalar = 1e30;
    next_part = end();
    for (part=begin(); part!=end(); part++) {
      scalar = part->Bounds().Minimum().FloatAttribute(ScalarTag);
      if (scalar >= max_scalar && scalar < min_scalar) {
        min_scalar = scalar;
        next_part = part;
      }
    }
    store_sequence = false;
    if (next_part != end()) {
      strip.push_back(*next_part);
      erase(next_part);
    }
    // If there are no such further tiles, store the tile sequence
    else store_sequence = true;
    if (store_sequence || empty()) {
      merged_strips.push_back(LaserUnit());
      (merged_strips.end()-1)->swap(strip); // This avoids copying and initialises strip
    }
  }
  
  // If there's only one sequence, create a sequence with just the last part
  // of the first sequence. This is done to ensure that double points are
  // removed from already correctly sorted strips.
  if (merged_strips.size() == 1) {
    strip.push_back(*(merged_strips.begin()->begin()));
    merged_strips.push_back(LaserUnit());
    (merged_strips.end()-1)->swap(strip);
    printf("Added second sequence with one part\n");
  }

  // Continue merging the strips until we're done
  while (merged_strips.size() > 1) {
  	if (debug) {
  	  printf("Processing sets %c\n", part_name[0]);
  	  for (strip1=merged_strips.begin(), strip_number=1; 
		   strip1!=merged_strips.end(); strip1++, strip_number++) {
  	    printf("Strip %d with parts:", strip_number);
  	    for (part1=strip1->begin(); part1!=strip1->end(); part1++)
  	      printf(" %s", part1->Name());
  	    printf("\n");
  	  }
  	}
  	
    // Clear meta data of strips
    for (strip1=strips.begin(); strip1!=strips.end(); strip1++)
      strip1->ReInitialise();
    strips.ReInitialise();
    // Move merged strips to strips
    strips.swap(merged_strips);
    // Increment starting letter of part names
    part_name[0]++; // A -> B -> C, etc.
    part_number = 0;
    // Select first two strips be be merged
    strip1 = strips.begin();
    strip2 = strip1 + 1;
    strip_number = 1;
    // Merge the next two strips until done
    while (strip1 != strips.end() && strip2 != strips.end()) {
      if (verbose) printf("Merging sets %d (%d parts) and %d (%d parts) out of %d %c sets    \n",
                          strip_number, strip1->size(), 
                          strip_number+1, strip2->size(),
                          strips.size(), part_name[0]-1);
      strip.ReInitialise();
      part1 = strip1->begin(); part1_number = 1;
      part2 = strip2->begin(); part2_number = 1;
      merged_part.ReInitialise();
      if (!part1->Read(part1->PointFile(), false)) {
        printf("Error reading %s\n", part1->PointFile());
        exit(0);
      }
      else if (verbose) {
        printf("Read part %d of set %d\r", part1_number, strip_number);
        fflush(stdout);
      }
      if (part1->empty()) {
        printf("No points in file %s\n", part1->PointFile());
        printf("Sets in this sequence are:");
        for (part=strip1->begin(); part!=strip1->end(); part++)
          printf(" %s", part->Name());
        printf("\n");
      }
      if (!part2->Read(part2->PointFile(), false)) {
        printf("Error reading %s\n", part2->PointFile());
        exit(0);
      }
      else if (verbose) {
        printf("Read part %d of set %d\r", part2_number, strip_number+1);
        fflush(stdout);
      }
      if (part2->empty()) {
        printf("No points in file %s\n", part2->PointFile());
        printf("Sets in this sequence are:");
        for (part=strip2->begin(); part!=strip2->end(); part++)
          printf(" %s", part->Name());
        printf("\n");
      }
      point1 = part1->begin();
      point2 = part2->begin();
      // Merge until there are no more strip parts
      while (part1 != strip1->end() || part2 != strip2->end()) {
        // Strip 1 done, point comes from strip 2
        if (part1 == strip1->end()) {
          if (LargerXYZ(&*point2, &previous_point)) {
            merged_part.push_back(*point2);
            previous_point = *point2;
          }
          else double_count++;
          increment1 = false;
        }
        // Strip 2 done, point comes from strip 1
        else if (part2 == strip2->end()) {
          if (LargerXYZ(&*point1, &previous_point)) {
            merged_part.push_back(*point1);
            previous_point = *point1;
          }
          else double_count++;
          increment1 = true;
        }
        // Take point with smallest scalar
        else {
          if (LargerScalar(&*point1, &*point2) == -1) {
            if (LargerXYZ(&*point1, &previous_point)) {
              merged_part.push_back(*point1);
              previous_point = *point1;
            }
            else double_count++;
            increment1 = true;
          }
          else {
            if (LargerXYZ(&*point2, &previous_point)) {
              merged_part.push_back(*point2);
              previous_point = *point2;
            }
            else double_count++;
            increment1 = false;
          }
        }

        // Get the next point
        if (increment1) { // Next point in strip 1
          point1++;
          if (point1 == part1->end()) {
            part1->ErasePoints();
            part1++;
            if (part1 != strip1->end()) {
              if (!part1->Read(part1->PointFile(), false)) {
                printf("Error reading %s\n", part1->PointFile());
                exit(0);
              }
              else if (verbose) {
              	part1_number++;
                printf("Read part %d of set %d\r", part1_number, strip_number);
                fflush(stdout);
              }
              point1 = part1->begin();
            }
          }
        }
        else { // Take next point in strip 2
          point2++;
          if (point2 == part2->end()) {
            part2->ErasePoints();
            part2++;
            if (part2 != strip2->end()) {
              if (!part2->Read(part2->PointFile(), false)) {
                printf("Error reading %s\n", part2->PointFile());
                exit(0);
              }
              else if (verbose) {
              	part2_number++;
                printf("Read part %d of set %d\r", part2_number, strip_number+1);
                fflush(stdout);
              }
              point2 = part2->begin();
            }
          }
        }

        // Write the merged part if it is full or there is no next point
        if (merged_part.size() == max_part_size ||
            (part1 == strip1->end() && part2 == strip2->end())) {
          // Set names
          if (strips.size() == 2) { // Last merge round, use final names
            sprintf(part_name, "%s_%6d", sorted_strip_name, strip.size()+1);
            ch = part_name+strlen(sorted_strip_name); 
            while (*ch) { if (*ch == ' ') *ch = '0'; ch++; }
            merged_part.SetName(part_name);
            merged_part.DataOrganisation() = StripWise;
            merged_part.DerivePointFileName(output_directory);
            merged_part.DeriveMetaDataFileName(output_directory);
            last_part_size = merged_part.size();
            if (remove_scalar_tag) merged_part.RemoveAttribute(ScalarTag);
          }
          else {
            part_number++;
            sprintf(part_name+2, "%6d", part_number);
            ch = part_name+2; while (*ch) { if (*ch == ' ') *ch = '0'; ch++; }
            merged_part.SetName(part_name);
            merged_part.DerivePointFileName(working_directory);
          }
          // Write points
          if (!merged_part.Write(merged_part.PointFile(), false, false)) {
            printf("Error writing file %s\n", merged_part.PointFile());
            exit(0);
          }
          // Erase points
          merged_part.ErasePoints();
          // Add part to strip
          strip.push_back(merged_part);
          merged_part.ReInitialise();
        }
      }

      // Store merged strip meta data if we need to continue
      if (strips.size() > 2) {
        merged_strips.push_back(strip);
        strip.ReInitialise();
      }
      
      // Clean up the files of the two merged strips
      for (part1=strip1->begin(); part1!=strip1->end(); part1++) {
        if (verbose)
	      printf("Deleting temporary file of part %s\r",
                 part1->Name());
        if (debug) printf("\n");
        sprintf(command, "del %s\\%s.laser", working_directory, part1->Name());
        system(command);
      }
      for (part2=strip2->begin(); part2!=strip2->end(); part2++) {
        if (verbose)
	      printf("Deleting temporary file of part %s\r",
                 part2->Name());
        if (debug) printf("\n");
        sprintf(command, "del %s\\%s.laser", working_directory, part2->Name());
        system(command);
      }
      
      // Initialise next two strips to be merged
      strip1 += 2;
      strip2 += 2;
      strip_number += 2;
      
      // Copy meta data of strip1 in case this is a single strip left
      if (strip2 == strips.end()) merged_strips.push_back(*strip1);
    }
  }
  
  // Copy the strip axis to the sorted strip
  strip.SetStripAxis(*(StripAxis()));
  
  // Save meta data of sorted strip
  strip.DataOrganisation() = StripWise;
  strip.SetName(sorted_strip_name);
  strip.DeriveMetaDataFileName(output_directory);
  strip.WriteMetaData(true);
  
  if (verbose) {
    printf("\nSorted strip %s with %d points in %.2f minutes         \n", sorted_strip_name,
           (strip.size()-1)*max_part_size + last_part_size,
           (double) (clock() - start_time) / (60 * CLOCKS_PER_SEC));
    if (double_count) printf("%d double points removed\n", double_count);
  }
}

/*
--------------------------------------------------------------------------------
                      Determine the number of points
--------------------------------------------------------------------------------
*/

long long int LaserUnit::NumberOfPoints() const
{
  LaserUnit                 *unit;
  LaserUnit::const_iterator subunit;
  long long int             num_pts = 0;
  
  // If needed read meta data in local unit
  if (empty()) {
    unit = new LaserUnit();
    if (MetaDataFile())
      if (!unit->ReadMetaData(MetaDataFile()))
        printf("Error reading unit meta data from file %s\n", MetaDataFile());
    for(subunit=unit->begin(); subunit!=unit->end(); subunit++)
      num_pts += subunit->NumberOfPoints();
    delete unit;
  }
  else
    for(subunit=begin(); subunit!=end(); subunit++)
      num_pts += subunit->NumberOfPoints();
  
  return num_pts;
}
