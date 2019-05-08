
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
#include "EdgeTools.h"

bool extractlines_grow(char* imagename,char* linesfilename, ImageLines &iml, int win, int t, double minl, double maxw)
{//see linex2

	ImgLines *lines_ptr=NULL;

	// load the image in cv format ie IplImage
	  IplImage *myimage;
	  if ((myimage = cvLoadImage( imagename, 1)) == 0) {
	    printf("Error reading input image %s\n", imagename);
	    return 0;
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
	         win, t,
	         minl,maxw,
	         &lines_ptr);

	  //release image memory - finished processing
	  cvReleaseImage(&myimage_gray);

	  if (lines_ptr == NULL) { printf("Error during line extraction\n");
	    return 0;
	  }

	  iml.C2Cpp(lines_ptr);
	  //FREE lines ptr
	  Free_ImgLines(lines_ptr);lines_ptr=NULL;

	  if (!iml.size()) {printf("no lines found\n"); return 0;}

	  iml.Write(linesfilename);

	  //write also an image file, superimposing the lines as debug info
	  char image_file_debug[500];
	  	  	sprintf(image_file_debug,"%sLinesGrow.png",imagename);
	  	  	printf("writing debug image to %s\n",image_file_debug);
	  // load the original image again

	       myimage=cvLoadImage(imagename);
	       //mark lines in image
	       for (int i_line=0;i_line<iml.size();i_line++){
	           //line coordinates
	           int begin_point_x=(int)round(iml[i_line][0].Y());
	           int begin_point_y=(int)round(iml[i_line][0].X());
	           int end_point_x=(int)round(iml[i_line][1].Y());
	           int end_point_y=(int)round(iml[i_line][1].X());
	           //mark line in the image
	           cvLine(   myimage,
	                     cvPoint(begin_point_x,begin_point_y),
	                     cvPoint(end_point_x,end_point_y),
	                     cvScalar(0,0,255), 1);//(0,0,255) for red, 1 pixel line width
	           }
	       //write lined_image to file
	       if( !cvSaveImage(image_file_debug, myimage) ){
	           printf("failed to write lined image file\n");

	           //return 0;
	           }
	       //release memory
	       cvReleaseImage(&myimage);



	  return 1;
}

bool extractlines_burns(char* imagename,char* linesfilename, ImageLines &iml, int bucket_width, int min_num_pixels, double min_magnitude, double vote, int gradient_mask)
{
	return llinexBurns(imagename, linesfilename, iml, bucket_width, min_num_pixels, min_magnitude, vote,gradient_mask);
	
}
