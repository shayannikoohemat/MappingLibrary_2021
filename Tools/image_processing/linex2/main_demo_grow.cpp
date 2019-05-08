
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

void PrintUsage()
{
  printf("Usage: linex -i <input any value image>\n");
  printf("             -win <window size, default: 3>\n");
  printf("             -t <gradient strength threshold>\n");
  printf("             -minl <minimum line length, default: 10 pixels>\n");
  printf("             -maxw <maximum line region width, default: 3 pixels>\n");
  printf("             -ol <extracted lines>\n");  
  printf("             -op <extracted end points of lines\n");
  printf("             -ot <extracted line topology\n");
  printf("             -g <output image with extracted lines on original image >\n"); //added Adam Nyaruhuma
}

extern "C" int llinex(unsigned char *, int, int, double, double,
                      int, int, int, int,  int, int, float, float, 
                      ImgLines **);
                        
int main(int argc, char *argv[])
{
  
  ImgLines        *lines_ptr=NULL;
  InlineArguments *args = new InlineArguments(argc, argv);
  
  // Variables for conversion to points and topology
  ImageLines      image_lines;
  ImageLines::iterator image_line;
  ImagePoints     image_points;
  ImagePoints::iterator image_point;  
  LineTopologies  line_tops;
  ObjectPoints    object_points;
  ObjectPoint     object_point;
    
  // Usage
  if (args->Contains("-usage")) {
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
  // Check required arguments
  if (!args->Contains("-i")) {
    printf("Error: No input image specified with -i\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-g") &&//output image
      !args->Contains("-ol") &&
      !(args->Contains("-op") && args->Contains("-ot")) &&
      !(args->Contains("-oo") && args->Contains("-ot"))) {
    printf("Error: No output file specified with -g or -ol or combination of -op and -ot\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  if (!args->Contains("-t")) {
    printf("Error: No gradient strength threshold specified with -t\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  // load the image in cv format ie IplImage
  IplImage *myimage;
  if ((myimage = cvLoadImage( args->String("-i"), 1)) == 0) {
    printf("Error reading input image %s\n", args->String("-i"));
    return EXIT_SUCCESS;  
  }
  //convert to gray image
  IplImage *myimage_gray= cvCreateImage(cvSize(myimage->width,myimage->height), IPL_DEPTH_8U, 1);  
  cvCvtColor(myimage, myimage_gray, CV_BGR2GRAY);
  //release memory - finished processing image
  cvReleaseImage(&myimage);
  
  //required for llinex processing
  uchar* image_data=(uchar*)myimage_gray->imageData;
  int height=myimage_gray->height;
  int width=myimage_gray->width;
  
  // Call C interface
  llinex(image_data, height, width, 0.0, 0.0,
         0, 0, height, width,
         args->Integer("-win", 3), args->Integer("-t", 1000),
         (float) args->Double("-minl", 10.0), (float) args->Double("-maxw", 3.0),
         &lines_ptr);
         
  //release image memory - finished processing
  cvReleaseImage(&myimage_gray); 
    
  // Output image lines 
  if (lines_ptr != NULL) {    
    if (args->Contains("-ol")) Put_ImgLines(lines_ptr, args->String("-ol"));

    // Conversion to image points and topology
    if (args->Contains("-ot")) {
      image_lines.C2Cpp(lines_ptr);
      image_lines.Convert2LineTops(&image_points, &line_tops); 
      if (args->Contains("-op")) image_points.Write(args->String("-op"));
      line_tops.Write(args->String("-ot"));
      if (args->Contains("-oo")) {
        for (image_point=image_points.begin(); image_point!=image_points.end(); image_point++) {          
          object_point.Number() = image_point->Number();
          object_point.X() = image_point->Column();
          object_point.Y() = -image_point->Row();
          object_point.Z() = 0.0;
          object_points.push_back(object_point);
        }	
        object_points.Write(args->String("-oo"));
      }
    }
    
    //show lines on original image
    if (args->Contains("-g")) {
      image_lines.C2Cpp(lines_ptr);
      
      // load the original image again - to be marked with lines
      myimage=cvLoadImage( args->String("-i"));
      //mark lines in image          
      for (int i_line=0;i_line<image_lines.size();i_line++){                    
          //line coordinates
          int begin_point_x=(int)round(image_lines[i_line][0].Y());
          int begin_point_y=(int)round(image_lines[i_line][0].X());
          int end_point_x=(int)round(image_lines[i_line][1].Y());
          int end_point_y=(int)round(image_lines[i_line][1].X());
          //mark line in the image 
          cvLine(   myimage, 
                    cvPoint(begin_point_x,begin_point_y),
                    cvPoint(end_point_x,end_point_y),
                    cvScalar(0,0,255), 1);//(0,0,255) for red, 1 pixel line width
          }
      //write lined_image to file
      if( !cvSaveImage(args->String("-g"), myimage) ){
          printf("failed to write lined image file\n");
          exit(0);
          }
      //release memory
      cvReleaseImage(&myimage);
      }
  }  
  return EXIT_SUCCESS;
}
