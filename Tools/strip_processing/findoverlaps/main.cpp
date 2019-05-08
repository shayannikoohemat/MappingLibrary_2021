
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
#include "LaserBlock.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: findoverlaps -i <block meta file>\n");
  printf("         -g <grid size of strip images>\n");
  printf("         -min <minimum number of square meter overlap for output (def: 1000)\n");
  printf("         -odir <output directory for block file for overlap extraction>\n");
  printf("         -r <reduction factor for first overlap check (def: 20)\n");
}

int ImageOverlap(const Image &image1, const Image &image2)
{
  const unsigned char *pixel1, *pixel2, *image_end;
  int num_pix_overlap = 0;
  
  // Count number of pixels in overlap
  image_end=image1.end();
  for (pixel1=image1.begin(), pixel2=image2.begin();
       pixel1!=image_end; pixel1++, pixel2++) 
  if (*pixel1 > 0 && *pixel2 > 0) num_pix_overlap++;

  return num_pix_overlap;
}

Image * ReducedBinaryImage(const Image &image, int factor)
{
  int numrows, numcols, numrowsred, numcolsred, row, col, rowred, colred;
  Image *imagered;
  const unsigned char *pixel;
  
  
  // Determine sizes
  numrows = image.NumRows();
  numcols = image.NumColumns();
  numrowsred = numrows / factor + 1;
  numcolsred = numcols / factor + 1;
  
  // Initialise reduced image
  imagered = new Image(numrowsred, numcolsred);
  imagered->ClearImage();
  
  // Set pixel values of reduced image
  for (row=0, pixel=image.begin(); row<numrows; row++) {
    rowred = row / factor;
    for (col=0; col<numcols; col++, pixel++) {
      if (*pixel) {
        colred = col / factor;
        *(imagered->Pixel(rowred, colred)) = 1;
      }
    }
  }
  
  return imagered;    
}

int main(int argc, char *argv[])
{
  InlineArguments           *args = new InlineArguments(argc, argv);
  LaserBlock                block, overlaps;
  LaserBlock::iterator      strip1, strip2;
  LaserUnit                 strip, empty_strip;
  char                      *name, *image_file;
  FILE                      *fd;
  double                    min_overlap=args->Double("-min", 1000.0),
                            overlap_area, pixel_area;
  vector<Image *>           icons;
  vector<Image *>::iterator icon1, icon2;
  int                       reduction_factor=args->Integer("-r", 20);
  bool                      quit=false;

  Image *icon;
  
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i")) {
    printf("Error: -i is a required argument.\n");
    PrintUsage();
    exit(0);
  }
  if (!args->Contains("-g")) {
    printf("Error: -g is a required argument.\n");
    PrintUsage();
    exit(0);
  }

  // Open the block (without reading strip part meta data)
  if (!block.ReadMetaData(args->String("-i"), true, false)) {
    printf("Error reading block: %s\n", args->String("-i"));
    exit(0);
  }
  if (block.size() < 2) {
    printf("Error: no two strips in this block\n");
    exit(0);
  }
  
  // First do a quick check on the presence of images
  for (strip1=block.begin(); strip1!=block.end(); strip1++) {
    image_file = strip1->ImagedHeight()->ImageFile();
    if (!image_file) {
      printf("No image for strip %s\n", strip1->Name());
      quit = true;
    }
  }
  if (quit) exit(0);
  
  // Create small images of all strip height images
  for (strip1=block.begin(); strip1!=block.end(); strip1++) {
    if (!strip1->ImagedHeight()->ReadImage()) {
      printf("Error reading height image of strip %s\n", strip1->Name());
      exit(0);
    }
    printf("Generating icon image for strip %s\r", strip1->Name());
    icon = ReducedBinaryImage(strip1->ImagedHeight()->ImageReference(),
                              reduction_factor);
    icons.push_back(icon);
    strip1->ImagedHeight()->DeleteImage();
  }
  printf("Generated %d icon images\n", icons.size());

  // Set up empty strip
  empty_strip.SetName("empty");
  empty_strip.DeriveMetaDataFileName(args->String("-odir"));
  empty_strip.WriteMetaData();
  empty_strip.DataOrganisation() = StripWise;
  strip.DataOrganisation() = StripWise;

  pixel_area = args->Double("-g", 0.0) * args->Double("-g", 0.0);
  // Loop over the strips
  for (strip1=block.begin(), icon1=icons.begin(); strip1!=block.end()-1;
       strip1++, icon1++) {
    // Read strip image
    if (!strip1->ImagedHeight()->ReadImage()) {
      printf("Error reading height image of strip %s, strip will be skipped\n", strip1->Name());
      continue;
    }
    for (strip2=strip1+1, icon2=icon1+1; strip2!=block.end(); strip2++, icon2++) {
      printf("Checking overlap of strips %s and %s\r", strip1->Name(), strip2->Name());
      // Check if icons overlap
      if (!ImageOverlap(**icon1, **icon2)) continue;
      printf("Icon images of strips %s and %s overlap\r", strip1->Name(), strip2->Name());
      // Read strip image
      if (!strip2->ImagedHeight()->ReadImage()) {
        printf("Error reading height image of strip %s, strip will be skipped\n", strip2->Name());
        continue;
      }
      // Output meta data of these strips if the overlap exceeds minimum size
      overlap_area = ImageOverlap(strip1->ImagedHeight()->ImageReference(),
                                  strip2->ImagedHeight()->ImageReference()) *
                     pixel_area;
      if (overlap_area >= min_overlap) {
        printf("Strips %s and %s overlap with %d m2             \n", strip1->Name(), strip2->Name(),
               (int) overlap_area);
        strip.SetMetaDataFile(strip1->MetaDataFile());
        overlaps.push_back(strip);
        strip.SetMetaDataFile(strip2->MetaDataFile());
        overlaps.push_back(strip);
        overlaps.push_back(empty_strip);
      }
      // Erase image of strip 2
      strip2->ImagedHeight()->DeleteImage();
    }
    // Erase image of strip 1
    strip1->ImagedHeight()->DeleteImage();
  }
  
  // Write the block meta data file
  name = (char *) malloc(strlen(block.Name()) + 10);
  sprintf(name, "%s_ordered", block.Name());
  overlaps.SetName(name);
  free(name);
  overlaps.DeriveMetaDataFileName(args->String("-odir"));
  overlaps.WriteMetaData(false, false);

  return EXIT_SUCCESS;
}
