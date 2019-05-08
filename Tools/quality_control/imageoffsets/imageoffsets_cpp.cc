
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
 Imaging results of computation of offsets between strips

 Initial creation:
 Author : George Vosselman
 Date   : 27-08-2008

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
                      The main imageoffsets function
--------------------------------------------------------------------------------
*/

void imageoffsets_cpp(const char *offset_file, const char * block_file,
                      const char *image_directory,
                      bool hoff, bool hrms, bool hstdev, 
                      bool poff, bool prms, bool pstdev, bool pqual,
                      int min_num_roofs)
{
  LaserBlock             block_overlaps;
  LaserBlock::iterator   strip_overlap;
  FILE                   *offset_fd, *image_fd;
  Image                  strip_image, block_image;
  std::vector<int>       strip_number1, strip_number2, num_roofs;
  std::vector<double>    dx, dy, dz, rms_xy, rms_z, stdev_xy, stdev_z,
                         overlap_size, stdev_tiles; 
  char                   line[201], *image_file_name;
  int                    n1, n2, n3, type, index, strip_number, band,
                         strip_numbera, strip_numberb, overlap_index, pix,
                         numpix, namelength, scanner_no, day_no, time_no;
  double                 d1, d2, d3, d4, d5, d6, d7, d8, d9, value,
                         threshold, red, green, blue, multiplier,
                         spacing_uncertainty;
  bool                   image_selector[7], image_initialised=false, found;
  Image                  strip_overlap_image, output_image, overlap_size_image;
  xvimage                *strip_overlap_viff, *output_viff, *overlap_size_viff;
  unsigned char          *output_pixel, *strip_overlap_pixel;
  float                  *overlap_size_pixel;
  char                   *image_name, *ch;

  // Transfer output selectors to array
  if (!hoff && !hrms && !hstdev && !poff && !prms && !pstdev && !pqual) {
    printf("Error: No output measure selected by -hoff, -hrms, -hstdev or\n");
    printf("       -poff, -prms, -pstdev, or -pqual.\n");
    exit(0);
  }
  image_selector[0] = hoff;
  image_selector[1] = hrms;
  image_selector[2] = hstdev;
  image_selector[3] = poff;
  image_selector[4] = prms;
  image_selector[5] = pstdev;
  image_selector[6] = pqual;
  
  // Open offset and block files
  offset_fd = fopen(offset_file, "r");
  if (!offset_fd) {
    printf("Error opening offset file %s\n", offset_file);
    exit(0);
  }
  if (!block_overlaps.ReadMetaData(block_file)) {
    printf("Error reading block file %s\n", block_file);
    exit(0);
  }

  // Read all offset data
  fgets(line, 200, offset_fd); // Skip first line
  while (!feof(offset_fd)) {
    fgets(line, 200, offset_fd);
    sscanf(line, "%d%d%d%lf%lf%lf%lf%lf%lf%lf", &n1, &n2, &n3, &d1, &d2, &d3,
           &d4, &d5, &d6, &d7, &d8, &d9);
    strip_number1.push_back(n1);
    strip_number2.push_back(n2);
    num_roofs.push_back(n3);
    dx.push_back(d1);
    dy.push_back(d2);
    dz.push_back(d3);
    rms_xy.push_back(d4);
    rms_z.push_back(d5);
    stdev_xy.push_back(d6);
    stdev_z.push_back(d7);
    overlap_size.push_back(d8);
    stdev_tiles.push_back(d9);
  }
  
  // Derive uncertainty due to point spacing, assuming 10 pts/m2
  spacing_uncertainty = 0.5 / sqrt(10.0);

  // Loop over all possible image types
  for (type=0; type<7; type++) {
    if (image_selector[type]) {
      // Loop over all overlaps
      for (strip_overlap=block_overlaps.begin();
           strip_overlap!=block_overlaps.end();
           strip_overlap++) {
        // Determine overlap image file name
        image_file_name = 
          strip_overlap->DeriveHeightImageFileName(strip_overlap->Name(),
                                                   image_directory);
//        printf("Overlap image %s\n", image_file_name);
        // Read the strip overlap image
        if (!strip_overlap_image.Read(image_file_name)) {
          printf("Error reading strip overlap image %s\n");
          exit(0);
        }
        numpix = strip_overlap_image.NumRows() * strip_overlap_image.NumColumns();
        
        // Initialise output image of the same size
        if (!image_initialised) {
          image_initialised = true;
          output_viff = output_image.NewImage(strip_overlap_image.NumRows(),
                                              strip_overlap_image.NumColumns(),
                                              VFF_TYP_1_BYTE, 3);
          if (output_viff == NULL) {
            printf("Error allocating output image of %dx%d pixels\n",
                   strip_overlap_image.NumRows(),
                   strip_overlap_image.NumColumns());
            exit(0);
          }
          output_image.ClearImage();
          output_viff->color_space_model = VFF_CM_ntscRGB;
          
          overlap_size_viff = overlap_size_image.NewImage(strip_overlap_image.NumRows(),
                                                          strip_overlap_image.NumColumns(),
                                                          VFF_TYP_FLOAT, 1); 
          if (overlap_size_viff == NULL) {
            printf("Error allocating strip size image of %dx%d pixels\n",
                   strip_overlap_image.NumRows(),
                   strip_overlap_image.NumColumns());
            exit(0);
          }
          overlap_size_image.ClearImage();
        }
        // Convert strip overlap name to strip numbers
        ch = strip_overlap->Name();
        namelength = strlen(ch);
        for (index=0; index<namelength; index++, ch++)
          if (*ch == '_') *ch = ' ';
        ch = strip_overlap->Name();
        sscanf(ch+8, "%d%d%d", &scanner_no, &day_no, &time_no);
        strip_numbera = scanner_no * 1000000 + day_no * 10000 + time_no;
        sscanf(ch+20, "%d%d%d", &scanner_no, &day_no, &time_no);
        strip_numberb = scanner_no * 1000000 + day_no * 10000 + time_no;
        for (index=0; index<namelength; index++, ch++) // Put back underscores for the next loop
          if (*ch == ' ') *ch = '_';
        // Look up offset results
        for (index=0, found=false; index<strip_number1.size() && !found; index++) {
          if ((strip_numbera == strip_number1[index] &&
               strip_numberb == strip_number2[index]) ||
              (strip_numbera == strip_number2[index] &&
               strip_numberb == strip_number1[index])) {
            found = true;
            overlap_index = index;
          }
        }
        // Determine colour for this strip overlap
        if (!found || num_roofs[overlap_index] < min_num_roofs) { // No reliable offset estimated, use white colour
          red = green = blue = 1.0;
//          printf("No reliable offset between strips %d and %d\n", strip_numbera, strip_numberb);
        }
        else {
          switch (type) {
            case 0: // Height offset
              value = fabs(dz[overlap_index]);
              threshold = 0.05; break;
            case 1: // Height RMS
              value = rms_z[overlap_index];
              threshold = 0.05; break;
            case 2: // Height standard deviation
              value = stdev_z[overlap_index];
              threshold = 0.05; break;
            case 3: // Planimetric offset
              value = sqrt(dx[overlap_index] * dx[overlap_index] +
                           dy[overlap_index] * dy[overlap_index]);
              threshold = 0.05; break;
            case 4: // Planimetric RMS
              // Correction for roof tile induced uncertainty
              rms_xy[overlap_index] = 
                sqrt(rms_xy[overlap_index] * rms_xy[overlap_index] -
                     stdev_tiles[overlap_index] * stdev_tiles[overlap_index]);
              value = rms_xy[overlap_index];
              threshold = 0.05; break;
            default:
            case 5: // Planimetric standard deviation
              // Correction for roof tile induced uncertainty
              stdev_xy[overlap_index] = 
                sqrt(stdev_xy[overlap_index] * stdev_xy[overlap_index] -
                     2 * stdev_tiles[overlap_index] * stdev_tiles[overlap_index]);
              value = stdev_xy[overlap_index];
              threshold = 0.05; break;
            case 6: // Planimetric quality check
              // Correction for roof tile induced uncertainty
              stdev_xy[overlap_index] = 
                sqrt(stdev_xy[overlap_index] * stdev_xy[overlap_index] -
                     2 * stdev_tiles[overlap_index] * stdev_tiles[overlap_index]);
              value = spacing_uncertainty + // Maximum error due to point spacing
                      sqrt(dx[overlap_index] * dx[overlap_index] + // Planimetric offset / 2
                           dy[overlap_index] * dy[overlap_index])/2.0 +
                      3.0 * stdev_xy[overlap_index] / sqrt(2.0); // Planimetric confidence interval / sqrt(2)
              threshold = 0.5; break;
          }
          if (value <= threshold) {
            red = blue = 0.0; green = 1.0;
          }
          else if (value <= threshold * 1.2) {
            blue = 0.0; red = green = 1.0;
          }
          else {
            red = 1.0; green = blue = 0.0;
          }
        }
        
        printf("Overlap %s in colour %d%d%d\r", strip_overlap->Name(),
               (int) red, (int) green, (int) blue);
        // Copy non-zero pixels of overlap image with appropriate colour into output image
        strip_overlap_viff = strip_overlap_image.GetImage();
        for (band=0, output_pixel=(unsigned char *) output_viff->imagedata;
             band<3; band++) {
          switch (band) {
            case 0: multiplier = red; break;
            case 1: multiplier = green; break;
            default:
            case 2: multiplier = blue; break;
          }
          
          for (pix=0, strip_overlap_pixel=(unsigned char *) strip_overlap_viff->imagedata,
               overlap_size_pixel=(float *) overlap_size_viff->imagedata;
               pix<numpix; 
               pix++, strip_overlap_pixel++, output_pixel++, overlap_size_pixel++)
            if (*strip_overlap_pixel != 0 && overlap_size[overlap_index] >= *overlap_size_pixel) {
              // + 255 / 2 to make the image brighter
              *output_pixel = (unsigned char) (((float) (*strip_overlap_pixel) + 255.0) * multiplier / 2.0);
              *overlap_size_pixel = (float) overlap_size[overlap_index];
            }
        }
      }
  
      // Derive file name of output image
      image_name = (char *) malloc(strlen(block_overlaps.Name()) + 20);
      strcpy(image_name, block_overlaps.Name());
      switch (type) {
        case 0: strcat(image_name, ".hoff.xv"); break;   // Height offset
        case 1: strcat(image_name, ".hrms.xv"); break;   // Height RMS
        case 2: strcat(image_name, ".hstdev.xv"); break; // Height standard deviation
        case 3: strcat(image_name, ".poff.xv"); break;   // Planimetric offset
        case 4: strcat(image_name, ".prms.xv"); break;   // Planimetric RMS
        case 5: strcat(image_name, ".pstdev.xv"); break; // Planimetric standard deviation
        case 6: strcat(image_name, ".pqual.xv"); break;  // Planimetric quality check
        default: break;
      }

      // Write output image
      output_image.Write(image_name);
      free(image_name);
    }
  }
  printf("\n");
}
