
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
  printf(
  "\n-----Necessary parameters----- \n"
  "-a <algorithm l for llinex or b for burns>\n"
  "-i <input any value image>\n"
  "-oo <extracted end points of lines -to be spaecified togather with: \n"
  "-ot <extracted line topology\n"
  "-g <output image with extracted lines on original image - to be specified alone>\n"
  "-------linex optional parameters------ \n"
  "-rx <minimum x of the ROI>\n"
  "-ry <minimum y of the ROI>\n"
  "-rRow <number of row in ROI>\n"
  "-rCol <number of colunm in ROI>\n"
  "-win <window size, default: 3>\n"
  "-t <gradient strength threshold, default 1000>\n"
  "-minl <minimum line length, default: 10 pixels>\n"
  "-maxw <maximum line region width, default: 3 pixels>\n"
  " -------burns optional parameters-------\n"
  "-b option is used to specify the bucket width durring the\n"
  "   edgel labeling stage\n"
  "     Default 8\n"
  "-c option is used to specifify the minimum number of pixels\n"
  "   in any given line support region.  Regions smaller than\n"
  "   c pixels will be ignored. minimum is 3, as a region with\n"
  "   less than 3 points can not identify a plane\n"
  "     Default 4\n"
  "-m option is used to specifify the minimum magnitude of a\n"
  "   contributing edgel.  Edgels will a gradient magnitude\n"
  "   smaller than f wil not contribute to any region.\n"
  "     Default 5.0\n"
  "-v option is used to specifify the minimum vote ratio of\n"
  "   lines.  A line will only result from a region if\n"
  "   (votes / pixels) > f.\n"
  "     Default 0.5\n"
  "-M specifies the mask to use for computing.\n"
  "   the gradients.  Valid Masks are(0,1,2,3,4,5): \n"
  "   Sobel, Prewitt,Isotropic,Optimal3,Optimal5,Derivative3\n");
}
extern "C++" ImageLines cvLinex(char *image, double roff, double coff,
           int r1_roi, int c1_roi, int nrows_roi, int ncols_roi,
           int window_size, int s_thres, float minlength, float maxwidth);
           
extern "C++" bool llinexBurns(char* imagename, ImageLines &iml, int bucket_width=8, int min_num_pixels=25, double min_magnitude=5.0, double vote=0.5, int gradient_mask=2);
extern "C++" void llinexBurns_arg(int argc, char ** argv);


                        
int main(int argc, char *argv[])
{
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
  if (!args->Contains("-a")) {
    printf("Error: algorithm (b for burns or l for llinex) specified with -a\n");
    PrintUsage();
    return EXIT_SUCCESS;
  }
  
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
  
  //choose method to use: cvLinex or burns
  if (*(args->String("-a"))=='l'){
      //roi width and height set as 1000000 for process whole image
      image_lines=cvLinex(args->String("-i"),0.0,0.0,args->Integer("-ry", 0),args->Integer("-rx", 0),
             args->Integer("-rRow", 100000), args->Integer("-rCol", 100000),
             args->Integer("-win", 3), args->Integer("-t", 1000),
             (float) args->Double("-minl", 10.0), (float) args->Double("-maxw", 3.0));
      }  
  else{llinexBurns(args->String("-i"), image_lines, args->Integer("-b", 8),
                   args->Integer("-c", 25),args->Double("-m", 5.0),
                   args->Double("-v", 0.5),args->Integer("-M", 0));
      }
    
  // Output lines to objpts and top files
  if (image_lines.size()!=0) {    
    if (args->Contains("-ot")) {      
      //for output
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
      // load the image in cv format ie IplImage
      IplImage *myimage;
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
