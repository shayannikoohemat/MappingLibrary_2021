
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

#include <cstring>
#include <string>
#include "Database4Cpp.h"
#include "ImageLines.h"
using namespace std;


//open cv stuff
#ifdef _CH_
#pragma package <opencv>
#endif
#ifndef _EiC
#include "cv.h"
#include "highgui.h"
#include <string.h>
#endif
extern "C" int llinex(unsigned char *, int, int, double, double,
                      int, int, int, int,  int, int, float, float, 
                      ImgLines **);
                     
ImageLines cvLinex(char *image, double roff, double coff,
           int r1_roi, int c1_roi, int nrows_roi, int ncols_roi,
           int window_size, int s_thres, float minlength, float maxwidth)
{     
  // load the image in cv format ie IplImage
  IplImage *myimage;

  if ((myimage = cvLoadImage( image, 1)) == 0) {
    printf("Error reading input image %s\n", image);
    exit(0);  
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
  if (nrows_roi==100000){nrows_roi=height;}
  if (ncols_roi==100000){ncols_roi=width;}
  ImgLines        *lines_ptr=NULL;
  llinex(image_data, height, width, roff, coff,
         r1_roi, c1_roi, nrows_roi, ncols_roi,window_size, s_thres,minlength, maxwidth,
         &lines_ptr);
  //release memory
  cvReleaseImage(&myimage_gray);
  ImageLines      image_lines;
  if (lines_ptr != NULL){
    // Conversion to image points and topology
    image_lines.C2Cpp(lines_ptr);
    //Free_ImgLines(lines);
    ImgLine *plin;
    plin = lines_ptr->lines;
    for(int i = 0; i < lines_ptr->num_lines; i++, plin++)
         free(plin->pts);
    free(lines_ptr->lines);
    free(lines_ptr);
    lines_ptr = NULL;
    }
  return image_lines;
  }
