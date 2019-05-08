
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
#include <math.h>
#include "InlineArguments.h"
#include "Image.h"
#include "Database4Cpp.h"
#include "ImageLines.h"
#include "ImagePoints.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"

//open cv stuff
#ifdef _CH_
#pragma package <opencv>
#endif
#ifndef _EiC
#include "cv.h"
#include "highgui.h"
#include <string.h>
#endif

using namespace std;


extern "C++" bool llinexBurns(char* imagename, ImageLines &iml, int bucket_width=8, int min_num_pixels=25, double min_magnitude=5.0, double vote=0.5, int gradient_mask=2);
extern "C++" void llinexBurns_arg(int argc, char ** argv);
                        
int main(int argc, char *argv[])
{
//1. possibility:
//directly call the operator using the given args   
/*
llinexBurns_arg(argc, argv);
*/


//or: call the wrapper
/**/
ImageLines iml;
ImagePoints     image_points;
  ImagePoints::iterator image_point;  
  LineTopologies  line_tops;
  ObjectPoints    object_points;
  ObjectPoint     object_point;

llinexBurns("p.jpg", iml, 8,25,5,0.5,1);


//output to objpts and top file for visualisation in pcm
iml.Convert2LineTops(&image_points, &line_tops); 
line_tops.Write("p.top");
      
for (image_point=image_points.begin(); image_point!=image_points.end(); image_point++) {          
	object_point.Number() = image_point->Number();
	object_point.X() = image_point->Column();
	object_point.Y() = -image_point->Row();
	object_point.Z() = 0.0;
	object_points.push_back(object_point);
}	
        object_points.Write("p.objpts");
      
/**/
  return EXIT_SUCCESS;
}
