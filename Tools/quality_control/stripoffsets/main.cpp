
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
#include <stdio.h>
#include <math.h>
#include "InlineArguments.h"
#include "Positions3D.h"
#include "ObjectPoints.h"
#include "LineTopologies.h"
#include "LaserBlock.h"

using namespace std;

void PrintUsage()
{
  printf("Usage: stripoffsets -i <output file of roofcheck or roofverify>\n");
  printf("                    -o <offsets>\n");
  printf("                    [-a] (append to offsets file)\n");
  printf("                    [-p] (compute offsets for each overlap part)\n");
  printf("                    [-s] (compute offsets for each set of -min <> points\n");
  printf("                    [-r] (compute offsets for each set of roofs with <> degrees orientation range\n");
  printf("                    [-min] <minimum number of roofs to compute offset (default: 2)>\n");
  printf("                    [-vecp] <offset vector points>\n");
  printf("                    [-vect] <offset vector topology>\n");
  printf("                    [-vecs] <offset vector scale (default: 1000.0)>\n");
  printf("                    [-res <output file with residuals after offset correction>]\n");
  printf("                    [-b <block meta data file for retrieving overlap sizes>]\n");
}

int StripNumber(char *name)
{
  int scanner, day, time;
  sscanf(name, "%1d", &scanner);
  sscanf(name+2, "%2d", &day);
  sscanf(name+5, "%4d", &time);
  return scanner*1000000 + day*10000 + time;
}

int main(int argc, char *argv[])
{
  InlineArguments *args = new InlineArguments(argc, argv);
  double          sumsinsin, sumsincos, sumcoscos, sumxsincos, sumysincos,
                  sumxcoscos, sumysinsin, sumdsin, sumdcos, sumdz,
                  sqsumdxy, sqsumdz, sumx, sumy, sumz, overlap_size,
                  x1, y1, z1, x2, y2, z2, alpha, d, sinalpha, cosalpha,
                  a[2][2], ainv[2][2], v[2], dx, dy, dz, det, dist, d1, d2,
                  rmsdxy, rmsdz, stddxy, stddz, scale, min_alpha, max_alpha,
                  pi = 4.0 * atan(1.0), alpha_range, grid_size,
                  stdobs, weight, sumvarobs;
  char            line[201], *name;
  int             stripno1, stripno2, oldstripno1, oldstripno2, numpts,
                  bldno, partno, oldpartno, min_numpts, ptno=0,
                  stripno1_check, stripno2_check;
  bool            output_per_part, output_per_set, output_per_range,
                  output_overlap_size, found;
  Positions3D     pts1, pts2;
  Positions3D::iterator pt1, pt2;
  FILE            *fdi, *fdo, *fdres;
  ObjectPoints    objpts;
  ObjectPoint     objpt;
  LineTopologies  tops;
  LineTopology    top;
  std::vector<int>           bldnos, stripnos1, stripnos2;
  std::vector<int>::iterator itrbldno, itrstripno1, itrstripno2;
  LaserBlock                 block;
  LaserBlock::iterator       overlap;

  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i") || !args->Contains("-o")) {
    printf("Error: -i and -o are required arguments.\n");
    PrintUsage();
    exit(0);
  }

  if (args->Contains("-p") && args->Contains("-s")) {
    printf("Error: -p and -s are mutually exclusive.\n");
    PrintUsage();
    exit(0);
  }
  
  // Open block meta data file if specified and read all strip meta data files
  output_overlap_size = args->Contains("-b");
  if (output_overlap_size) {
    if (!block.ReadMetaData(args->String("-b"), true, false)) {
      printf("Error reading block meta data from file %s\n",
             args->String("-b"));
      exit(0);
    }
  }
  
  // Open roof offsets file and skip the first line
  fdi = fopen(args->String("-i"), "r");
  if (fdi == NULL) {
    printf("Error opening file %s\n", args->String("-i"));
    return 0;
  }
  fgets(line, 200, fdi);

  // Open output files
  if (args->Contains("-a")) fdo = fopen(args->String("-o"), "a");
  else fdo = fopen(args->String("-o"), "w");
  if (fdo == NULL) {
    printf("Error opening file %s\n", args->String("-o"));
    return 0;
  }
  
  if (args->Contains("-res")) {
    if (args->Contains("-a"))
      fdres = fopen(args->String("-res", "offset_residuals.txt"), "a");
    else
      fdres = fopen(args->String("-res", "offset_residuals.txt"), "w");
    if (fdres == NULL) {
      printf("Error opening residual output file %s\n",
             args->String("-res", "offset_residuals.txt"));
      exit(0);
    }
    if (!args->Contains("-a"))
      fprintf(fdres, "  bld strip1      X1         Y1      Z1    strip2      X2         Y2      Z2  dXY (res) dZ (res)\n");
  }
  else fdres = NULL;
  
  output_per_part = args->Contains("-p");
  output_per_set = args->Contains("-s");
  output_per_range = args->Contains("-r");
  alpha_range = args->Double("-r", 30.0) * pi / 180.0;
  min_numpts = args->Integer("-min", 2);
  scale = args->Double("-vecs", 1000.0);
  max_alpha = min_alpha = 0.0;
  // Process the roof offsets
  oldstripno1 = oldstripno2 = -1;
  oldpartno = partno = 0; // Disabling part number usage
  if (output_per_part || output_per_set)
    fprintf(fdo, " Strip 1  Strip 2 Part  #Roofs  dX      dY      dZ   RMSxy    RMSz   STDxy    STDz   Av.X      Av.Y      Av.Z\n");
  else if (output_per_range)
    fprintf(fdo, " Strip 1  Strip 2 Part  #Roofs  dX      dY      dZ   RMSxy    RMSz   STDxy    STDz   Av.X      Av.Y      Av.Z    range\n");
  else if (output_overlap_size)
    fprintf(fdo, " Strip 1  Strip 2  #Roofs    dX      dY      dZ   RMSxy    RMSz   STDxy    STDz   overlap size  STDobs\n");
  else
    fprintf(fdo, " Strip 1  Strip 2  #Roofs    dX      dY      dZ   RMSxy    RMSz   STDxy    STDz   STDobs\n");
  while (!feof(fdi)) {
    fgets(line, 200, fdi);
    sscanf(line, "%d%d%lf%lf%lf%d%lf%lf%lf%lf%lf%lf", &bldno, &stripno1, &x1, &y1, &z1,
           &stripno2, &x2, &y2, &z2, &d1, &d2, &stdobs);
    if (stdobs < 0.001) stdobs = 0.001;
    if (stripno1 != oldstripno1 || stripno2 != oldstripno2 ||
        (partno != oldpartno && output_per_part) ||
        (numpts == min_numpts && output_per_set) ||
        (max_alpha - min_alpha > alpha_range && output_per_range && numpts >= min_numpts) ||
        feof(fdi)) {
      // Calculate and write strip offsets
      if (oldstripno1 != -1 && oldpartno != -1 && numpts >= min_numpts &&
          (max_alpha - min_alpha > alpha_range || !output_per_range)) {
        // Equation system
        a[0][0] = sumcoscos;
        a[0][1] = sumsincos;
        a[1][0] = sumsincos;
        a[1][1] = sumsinsin;
        v[0] = sumxcoscos + sumysincos - sumdcos;
        v[1] = sumxsincos + sumysinsin - sumdsin;
        det = a[0][0] * a[1][1] - a[0][1] * a[1][0];
        ainv[0][0] = a[1][1] / det;
        ainv[0][1] = -a[0][1] / det;
        ainv[1][0] = -a[1][0] / det;
        ainv[1][1] = a[0][0] / det;
        // Offsets
        dx = ainv[0][0] * v[0] + ainv[0][1] * v[1];
        dy = ainv[1][0] * v[0] + ainv[1][1] * v[1];
        dz = sumdz / numpts;
        // RMS before applying offsets
        rmsdxy = sqrt(sqsumdxy / numpts);
        rmsdz  = sqrt(sqsumdz / numpts);
        // RMS after applying offsets
        sqsumdxy = sqsumdz = 0.0;
        for (pt1=pts1.begin(), pt2=pts2.begin(), itrbldno=bldnos.begin(),
             itrstripno1=stripnos1.begin(), itrstripno2=stripnos2.begin();
             pt1!=pts1.end();
             pt1++, pt2++, itrbldno++, itrstripno1++, itrstripno2++) {
          if (pt1->X() != pt2->X() || pt1->Y() != pt2->Y()) {
            alpha = atan2(pt1->Y() - pt2->Y(), pt1->X() - pt2->X());
            d = pt1->X() * cos(alpha) + pt1->Y() * sin(alpha);
            dist = (pt2->X() - dx) * cos(alpha) + (pt2->Y() - dy) * sin(alpha) - d;
          }
          else dist = 0;
          sqsumdxy += dist * dist;
          sqsumdz += (pt1->Z() - pt2->Z() + dz) * (pt1->Z() - pt2->Z() + dz);
          if (fdres)
            fprintf(fdres,
              "%6d %6d %10.3f %10.3f %7.3f %6d %10.3f %10.3f %7.3f %6.3f %6.3f\n",
              *itrbldno, *itrstripno1, pt1->X(), pt1->Y(), pt1->Z(),
              *itrstripno2, pt2->X(), pt2->Y(), pt2->Z(), dist, pt1->Z() - pt2->Z() + dz);
        }
        stddxy = sqrt(sqsumdxy / (numpts-2));
        stddz  = sqrt(sqsumdz / (numpts-1));
        if (output_per_part || output_per_set)
          fprintf(fdo, "%8d %8d %2d %5d %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %10.2f %10.2f %6.2f\n",
                  oldstripno1, oldstripno2, oldpartno, numpts, 
                  dx, dy, dz, rmsdxy, rmsdz, stddxy, stddz,
                  sumx/numpts, sumy/numpts, sumz/numpts);
        else if (output_per_range)
          fprintf(fdo, "%8d %8d %2d %5d %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %10.2f %10.2f %6.2f %6.2f\n",
                  oldstripno1, oldstripno2, oldpartno, numpts, 
                  dx, dy, dz, rmsdxy, rmsdz, stddxy, stddz,
                  sumx/numpts, sumy/numpts, sumz/numpts, (max_alpha - min_alpha) * 180.0 / pi);
        else if (output_overlap_size) {
          // Locate overlap in block
          for (overlap=block.begin(), found=false; overlap!=block.end() && !found; overlap++) {
            name = overlap->Name();
            stripno1_check = StripNumber(name+8);
            stripno2_check = StripNumber(name+20);
            if ((oldstripno1 == stripno1_check && oldstripno2 == stripno2_check) ||
                (oldstripno1 == stripno2_check && oldstripno2 == stripno1_check)) {
              found = true;
              // Read overlap image and grid
              if (!overlap->ReadImagedHeight()) {
                printf("Error reading height image of %s\n", name);
                exit(0);
              }
              // Determine overlap size
              grid_size = overlap->ImagedHeight()->Grid()->Pixelsize();
              overlap_size = overlap->ImagedHeight()->NumberNonZeroPixels() *
                             grid_size * grid_size;
              // Delete the height image and the overlap
              overlap->ImagedHeight()->DeleteImage();
              block.erase(overlap);
            }
          }
          fprintf(fdo, "%8d %8d %5d %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %10.0f %6.4f\n",
                  oldstripno1, oldstripno2, numpts, dx, dy, dz, 
                  rmsdxy, rmsdz, stddxy, stddz, overlap_size,
                  sqrt(sumvarobs/numpts));
        }
        else
          fprintf(fdo, "%8d %8d %5d %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %6.4f\n",
                  oldstripno1, oldstripno2, numpts, dx, dy, dz, 
                  rmsdxy, rmsdz, stddxy, stddz, sqrt(sumvarobs/numpts));
        // Store offset vectors
        if (args->Contains("-vecp") && args->Contains("-vect")) {
          // Line
          top.Erase();
          objpt.Number() = ptno;
          objpt.X() = sumx/numpts;
          objpt.Y() = sumy/numpts;
          objpt.Z() = sumz/numpts;
          objpts.push_back(objpt);
          top.push_back(PointNumber(ptno));
          ptno++;
          objpt.Number() = ptno;
          objpt.X() += dx * scale;
          objpt.Y() += dy * scale;
          objpts.push_back(objpt);
          top.push_back(PointNumber(ptno));
          ptno++;
          tops.push_back(top);
          // Arrow
          if (dx*dx + dy*dy > 0.0) {
            top.Erase();
            dz = 0.1 * sqrt(dx*dx + dy*dy);
            dx /= dz;
            dy /= dz;
            objpt.Number() = ptno;
            objpt.X() += -dy - dx;
            objpt.Y() += dx - dy;
            objpts.push_back(objpt);
            top.push_back(PointNumber(ptno));
            top.push_back(PointNumber(ptno-1));
            ptno++;
            objpt.Number() = ptno;
            objpt.X() += 2.0 * dy;
            objpt.Y() -= 2.0 * dx;
            objpts.push_back(objpt);
            top.push_back(PointNumber(ptno));
            ptno++;
            tops.push_back(top);
          }
        }
      }
      // Reset sums
      numpts = 0;
      sumsinsin = sumsincos = sumcoscos = 0.0;
      sumxsincos = sumysincos = sumxcoscos = sumysinsin = 0.0;
      sumdsin = sumdcos = sumdz = 0.0;
      sqsumdxy = sqsumdz = 0.0;
      sumx = sumy = sumz = 0.0;
      sumvarobs = 0.0;
      min_alpha = max_alpha = 0.0;
      oldstripno1 = stripno1; 
      oldstripno2 = stripno2;
      oldpartno   = partno;
      // Clear buffers
      pts1.erase(pts1.begin(), pts1.end());
      pts2.erase(pts2.begin(), pts2.end());
      bldnos.erase(bldnos.begin(), bldnos.end());
      stripnos1.erase(stripnos1.begin(), stripnos1.end());
      stripnos2.erase(stripnos2.begin(), stripnos2.end());
    }

    // Derive line parameters
    if (x1 == x2 && y1 == y2) {
      sinalpha = cosalpha = 0.0;
    }
    else {
      alpha = atan2(y1-y2, x1-x2);
      sinalpha = sin(alpha);
      cosalpha = cos(alpha);
    }
    d = x1 * cosalpha + y1 * sinalpha;
    dist = x2 * cosalpha + y2 * sinalpha - d;
    
    // Update range
    if (min_alpha == 0.0 && max_alpha == 0.0) {
      if (x1 != x2 || y1 != y2)
        min_alpha = alpha;
        max_alpha = alpha;
      }
    else {
      // Get within 90 degrees of min_alpha
      while (alpha < min_alpha - pi/2.0) alpha += pi;
      while (alpha > min_alpha + pi/2.0) alpha -= pi;
      if (alpha < min_alpha) min_alpha = alpha;
      if (alpha > max_alpha) max_alpha = alpha;
    }
    
    // Update sums
    numpts++;
    weight      = 1.0 / (stdobs * stdobs);
    weight = 1.0; // Disabling weights
    sumsinsin  += sinalpha * sinalpha * weight;
    sumsincos  += sinalpha * cosalpha * weight;
    sumcoscos  += cosalpha * cosalpha * weight;
    sumxsincos += x2 * sinalpha * cosalpha * weight;
    sumysincos += y2 * sinalpha * cosalpha * weight;
    sumxcoscos += x2 * cosalpha * cosalpha * weight;
    sumysinsin += y2 * sinalpha * sinalpha * weight;
    sumdsin    += d * sinalpha * weight;
    sumdcos    += d * cosalpha * weight;
    sumdz      += z2 - z1;
    sqsumdxy   += dist * dist;
    sqsumdz    += (z2 - z1) * (z2 - z1);
    sumx       += x1;
    sumy       += y1;
    sumz       += z1;
    sumvarobs  += stdobs * stdobs;
    
    // Save information for computation of standard deviations and residuals
    pts1.push_back(Position3D(x1, y1, z1));
    pts2.push_back(Position3D(x2, y2, z2));
    bldnos.push_back(bldno);
    stripnos1.push_back(stripno1);
    stripnos2.push_back(stripno2);
  }
  
  // Write records with overlap sizes for overlaps without (sufficient) ridge lines
  if (output_overlap_size) {
    for (overlap=block.begin(); overlap!=block.end(); overlap++) {
      // Determine strip numbers
      name = overlap->Name();
      stripno1_check = StripNumber(name+8);
      stripno2_check = StripNumber(name+20);
      // Read overlap image and grid
      if (!overlap->ReadImagedHeight()) {
        printf("Error reading height image of %s\n", name);
        exit(0);
      }
      // Determine overlap size
      grid_size = overlap->ImagedHeight()->Grid()->Pixelsize();
      overlap_size = overlap->ImagedHeight()->NumberNonZeroPixels() *
                     grid_size * grid_size;
      // Delete the height image
      overlap->ImagedHeight()->DeleteImage();
      // Write output record
      fprintf(fdo, "%8d %8d %5d %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %7.3f %10.0f\n",
                  stripno1_check, stripno2_check, 0, 0.0, 0.0, 0.0, 
                  0.0, 0.0, 0.0, 0.0, overlap_size);
    }
  }
  
  // Close files
  fclose(fdi);
  fclose(fdo);
  
  if (args->Contains("-vecp") && args->Contains("-vect")) {
    objpts.Write(args->String("-vecp"));
    tops.Write(args->String("-vect"), false);
  }
  return EXIT_SUCCESS;
}
