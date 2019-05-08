
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

//
#ifndef EdgeTools_H
#define EdgeTools_H

#include <cstdlib>
#include <iostream>
#include <math.h>
//#include "InlineArguments.h"
#include "Image.h"
#include "Database4Cpp.h"
#include "ImageLines.h"
#include "ImagePoints.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"

#include "cv.h"
#include "highgui.h"
#include <string.h>



using namespace std;
extern "C" int llinex(unsigned char *, int, int, double, double,
                      int, int, int, int,  int, int, float, float,
                      ImgLines **);

extern "C++" bool llinexBurns(char* imagename,char* linesfilename, ImageLines &iml, int bucket_width, int min_num_pixels, double min_magnitude, double vote, int gradient_mask);

bool extractlines_grow(char* imagename,char* linesfilename, ImageLines &iml, int win=3, int t=1000, double minl=10.0, double maxw=3.0);
bool extractlines_burns(char* imagename,char* linesfilename, ImageLines &iml, int bucket_width=8, int min_num_pixels=25, double min_magnitude=5.0, double vote=0.5, int gradient_mask=2);
#endif
