
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

//THis is the same as burns.c, except for the fact that a debug image is written, and the internal memory management is a little different
#include <cstdlib>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <iostream>
#include "Database.h"
#include "Database4Cpp.h"
#include "ImageLines.h"

#ifdef _CH_
#pragma package <opencv>
#endif
#ifndef _EiC
#include "cv.h"
#include "highgui.h"
#include <string.h>
//#include "OpeCvImageElement.h"
#endif

using namespace std;
/* Useful macros */
//#define MAX(a,b) (((a)>(b))?(a):(b))
//#define MIN(a,b) (((a)<(b))?(a):(b))

/* Debugging Macros */
#define printint(val) printf("%s = %d\n",#val,val);
#define printfloat(val) printf("%s = %f\n",#val,val);


/***** This section might be scary         *****/
/***** We are making (in the end) an array *****/
/***** of pointers to functions            *****/

typedef void(*hv_gradientmask)();  // pointer to a gradient function

/* Declarations of mask functions */
void hv_sobel();
void hv_prewitt();
void hv_iso();
void hv_opt3();
void hv_opt5();
void hv_deriv3();

/* Yes, this is an array of function pointers */
hv_gradientmask masks[] =  {hv_sobel,hv_prewitt,hv_iso,      hv_opt3,   hv_opt5  ,hv_deriv3};
char       *masknames[] =  {(char *) "Sobel", (char *) "Prewitt", (char *) "Isotropic",
                            (char *) "Optimal3", (char *) "Optimal5", (char *) "Derivative3"};

#define NUM_MASKS (sizeof(masks)/sizeof(hv_gradientmask))

/* Gradient mask constants */
#define SOBEL   0
#define PREWITT 1
#define ISO     2
#define OPT3    3
#define OPT5    4

/* Default Settings */
#define EIGHT_CONNECT_DEFAULT 0
#define NUM_BUCKETS_DEFAULT   8
#define VERBOSE_DEFAULT       0
#define COUNT_THRESH_DEFAULT  4
#define VOTE_THRESH_DEFAULT   0.5
#define MAG_THRESH_DEFAULT    5.0
#define GRADIENT_MASK_DEFAULT SOBEL

int    eight_connect = EIGHT_CONNECT_DEFAULT;   // Use an 8-connected neighborhood for floodfill
int    num_buckets   = NUM_BUCKETS_DEFAULT;     // number of angle labels
int    verbose       = VERBOSE_DEFAULT;         // print out extra stuff
int    count_thresh  = COUNT_THRESH_DEFAULT;    // minimum pixels in a region
double vote_thresh   = VOTE_THRESH_DEFAULT;     // minimum number of votes
double mag_thresh    = MAG_THRESH_DEFAULT;      // minimum magnitude of edgel 
int    gradient_mask = GRADIENT_MASK_DEFAULT;   // which gradient mask

/* floatingpoint point */
typedef struct S_XYPoint { 
  double x,y;
} XYPoint;

/* Linked list of int points */
typedef struct S_PointLst {
  int x,y;
  struct S_PointLst *next;
} PointLst;

/* Linked list of regions */
typedef struct S_Region {
  PointLst *p_hd;   // Head of "pixels in my region" list
  PointLst *p_tl;   // Tail of "pixels in my region" list
  int minx,maxx;    // X extents
  int miny,maxy;    // Y extents
  int count;        // Number of pixels in me
  int votes;        // Number of votes for me
  int lab;          // What label am I a region of (angle)
  int valid;        // Am I a valid region?
  double percent;   // percentage of votes for me
  double height;    // My average height

  double M[3][3];   // M-Matrix
  double K;         // K value for computing Mi
  double Mi[3][3];  // M, inverted.
  double B[3];      // B
  double MiB[3];    // Mi * B
  XYPoint pts[4];   // My 4 extent-intersection points. (sorted)
  struct S_Region *next;
} Region;

/* Image dimensions */
int width; 
int height;

/** The following arrays of arrays are allocated **/
/** when needed and freed when no nolger in use  **/

/* Array of arrays of pixel values */
int **image;

/* Array of arrays of gradient magnitudes */
double **mag;

/* Array of arrays of gradient angles */
double **ang;

/* Array of arrays of horizontal gradients */
double **h_mag;

/* Array of arrays of vertical gradients */
double **v_mag;

/* Array of arrays of Labelings */
int **Lab1;
int **Lab2;

/* Array of arrays of pointers to regions    */
/* Reg1[y][x] is a pointer to the region     */
/* that pixel x,y is in for the 1st labeling */
Region ***Reg1;
Region ***Reg2;

/* Head and tail for the two region lists */
Region *RegList1_hd=NULL,*RegList1_tl=NULL;
Region *RegList2_hd=NULL,*RegList2_tl=NULL;


/***************************************************/
/***** Various edge detection masks that store *****/
/***** their results in h_mag and v_mag        *****/
/***************************************************/

/* Sobel 3x3 Mask */
void hv_sobel() {
  int x,y;

  if(verbose>1) {
    printf("  Applying Sobel Masks\n");
    fflush(stdout);
  }

#define SOBEL_A 1
#define SOBEL_B 2
#define SOBEL_DIV 8.0

  for(y=1;y<height-1;y++) {
    for(x=1;x<width-1;x++) {
      h_mag[y][x] = 
	(image[y-1][x-1] * -SOBEL_A +
	 image[y  ][x-1] * -SOBEL_B +
	 image[y+1][x-1] * -SOBEL_A +
	 image[y-1][x+1] *  SOBEL_A +
	 image[y  ][x+1] *  SOBEL_B +
	 image[y+1][x+1] *  SOBEL_A) / SOBEL_DIV;
      v_mag[y][x] = 
	(image[y-1][x-1] * -SOBEL_A +
	 image[y-1][x  ] * -SOBEL_B  +
	 image[y-1][x+1] * -SOBEL_A +
	 image[y+1][x-1] *  SOBEL_A +
	 image[y+1][x  ] *  SOBEL_B  +
	 image[y+1][x+1] *  SOBEL_A) / SOBEL_DIV;
    }
  }
}

/* Prewitt 3x3 Mask */
void hv_prewitt() {
  int x,y;

  if(verbose>1) {
    printf("  Applying Prewit Masks\n");
    fflush(stdout);
  }

#define PREWITT_A 1
#define PREWITT_B 1
#define PREWITT_DIV 6.0

  for(y=1;y<height-1;y++) {
    for(x=1;x<width-1;x++) {
      h_mag[y][x] = 
	(image[y-1][x-1] * -PREWITT_A +
	 image[y  ][x-1] * -PREWITT_B +
	 image[y+1][x-1] * -PREWITT_A +
	 image[y-1][x+1] *  PREWITT_A +
	 image[y  ][x+1] *  PREWITT_B +
	 image[y+1][x+1] *  PREWITT_A) / PREWITT_DIV;
      v_mag[y][x] = 
	(image[y-1][x-1] * -PREWITT_A +
	 image[y-1][x  ] * -PREWITT_B  +
	 image[y-1][x+1] * -PREWITT_A +
	 image[y+1][x-1] *  PREWITT_A +
	 image[y+1][x  ] *  PREWITT_B  +
	 image[y+1][x+1] *  PREWITT_A) / PREWITT_DIV;
    }
  }
}

/* Frei and Chen's isotropic 3x3 Mask */
void hv_iso() {
  int x,y;

  if(verbose>1) {
    printf("  Applying Isotropic Masks\n");
    fflush(stdout);
  }

#define ISO_A 1.0
#define ISO_B M_SQRT2
#define ISO_DIV (4.0 + 2.0*M_SQRT2)

  for(y=1;y<height-1;y++) {
    for(x=1;x<width-1;x++) {
      h_mag[y][x] = 
	(image[y-1][x-1] * -ISO_A +
	 image[y  ][x-1] * -ISO_B +
	 image[y+1][x-1] * -ISO_A +
	 image[y-1][x+1] *  ISO_A +
	 image[y  ][x+1] *  ISO_B +
	 image[y+1][x+1] *  ISO_A) / ISO_DIV;
      v_mag[y][x] = 
	(image[y-1][x-1] * -ISO_A +
	 image[y-1][x  ] * -ISO_B  +
	 image[y-1][x+1] * -ISO_A +
	 image[y+1][x-1] *  ISO_A +
	 image[y+1][x  ] *  ISO_B  +
	 image[y+1][x+1] *  ISO_A) / ISO_DIV;
    }
  }
}

/* "Optimal" 3x3 Mask */
void hv_opt3() {
  int x,y;

  if(verbose>1) {
    printf("  Applying \"Optimal\" 3x3 Masks\n");
    fflush(stdout);
  }

#define OPT3_A 0.112737
#define OPT3_B 0.274526

  for(y=1;y<height-1;y++) {
    for(x=1;x<width-1;x++) {
      h_mag[y][x] = 
	image[y-1][x-1] * -OPT3_A +
	image[y  ][x-1] * -OPT3_B +
	image[y+1][x-1] * -OPT3_A +
	image[y-1][x+1] *  OPT3_A +
	image[y  ][x+1] *  OPT3_B +
	image[y+1][x+1] *  OPT3_A;
      v_mag[y][x] = 
	image[y-1][x-1] * -OPT3_A +
	image[y-1][x  ] * -OPT3_B  +
	image[y-1][x+1] * -OPT3_A +
	image[y+1][x-1] *  OPT3_A +
	image[y+1][x  ] *  OPT3_B  +
	image[y+1][x+1] *  OPT3_A;
    }
  }
}

/* "Optimal" 5x5 Mask */
void hv_opt5() {
  int x,y;

  if(verbose>1) {
    printf("  Applying \"Optimal\" 5x5 Masks\n");
    fflush(stdout);
  }

#define OPT5_A 0.003776
#define OPT5_B 0.026786
#define OPT5_C 0.046548
#define OPT5_D 0.010199
#define OPT5_E 0.070844
#define OPT5_F 0.122572

  for(y=2;y<height-2;y++) {
    for(x=2;x<width-2;x++) {
      h_mag[y][x] = 
	image[y-2][x-2] * -OPT5_A + 
	image[y-1][x-2] * -OPT5_B + 
	image[y  ][x-2] * -OPT5_C +
	image[y+1][x-2] * -OPT5_B +
	image[y+2][x-2] * -OPT5_A +

	image[y-2][x-1] * -OPT5_D + 
	image[y-1][x-1] * -OPT5_E +
	image[y  ][x-1] * -OPT5_F +
	image[y+1][x-1] * -OPT5_E +
	image[y+2][x-1] * -OPT5_D +

	image[y-2][x+1] *  OPT5_D +
	image[y-1][x+1] *  OPT5_E +
	image[y  ][x+1] *  OPT5_F +
	image[y+1][x+1] *  OPT5_E +
	image[y+2][x+1] *  OPT5_D +

	image[y-2][x+2] *  OPT5_A +
	image[y-1][x+2] *  OPT5_B +
	image[y  ][x+2] *  OPT5_C +
	image[y+1][x+2] *  OPT5_B +
	image[y+2][x+2] *  OPT5_A;
      v_mag[y][x] = 
	image[y-2][x-2] * -OPT5_A + 
	image[y-2][x-1] * -OPT5_B + 
	image[y-2][x  ] * -OPT5_C +
	image[y-2][x+1] * -OPT5_B +
	image[y-2][x+2] * -OPT5_A +

	image[y-1][x-2] * -OPT5_D + 
	image[y-1][x-1] * -OPT5_E +
	image[y-1][x  ] * -OPT5_F +
	image[y-1][x+1] * -OPT5_E +
	image[y-1][x+2] * -OPT5_D +

	image[y+1][x-2] *  OPT5_D +
	image[y+1][x-1] *  OPT5_E +
	image[y+1][x  ] *  OPT5_F +
	image[y+1][x+1] *  OPT5_E +
	image[y+1][x+2] *  OPT5_D +

	image[y+2][x-2] *  OPT5_A +
	image[y+2][x-1] *  OPT5_B +
	image[y+2][x  ] *  OPT5_C +
	image[y+2][x+1] *  OPT5_B +
	image[y+2][x+2] *  OPT5_A;
    }
  }
}

/* Derivative 3x1 Mask */
void hv_deriv3() {
  int x,y;

  if(verbose>1) {
    printf("  Applying Derivative Masks\n");
    fflush(stdout);
  }

#define DERIV3_A 1
#define DERIV3_DIV 2.0
  for(y=1;y<height-1;y++) {
    for(x=1;x<width-1;x++) {
      h_mag[y][x] = 
	(image[y  ][x-1] * -DERIV3_A +
	 image[y  ][x+1] *  DERIV3_A) / DERIV3_DIV;
      v_mag[y][x] = 
	(image[y-1][x  ] * -DERIV3_A +
	 image[y+1][x  ] *  DERIV3_A) / DERIV3_DIV;
    }
  }
}

/* Load a image change to gray scale image and store */
/* in width, height and image                */
void load_image(const char *fname) {     
  IplImage *cv_image = 0;  
  if( (cv_image = cvLoadImage( fname, 1)) == 0 )
       {printf("Unable to open file %s\n",fname);
       exit(0);}
  //obtain gray image
  IplImage *cv_image_gray= cvCreateImage(cvSize(cv_image->width,cv_image->height), IPL_DEPTH_8U, 1);
  cvCvtColor(cv_image, cv_image_gray, CV_BGR2GRAY);
  //release memory of cv_image used
  cvReleaseImage(&cv_image);
  
  width=cv_image_gray->width;
  height=cv_image_gray->height;
  int x,y;  
  image = (int **)malloc(height*sizeof(int *));
  for(y=0;y<height;y++) {
    image[y] = (int *)malloc(width*sizeof(int));
    for(x=0;x<width;x++) {
       uchar* pixel_val =(uchar*)(cv_image_gray->imageData + y*cv_image_gray->widthStep + x);
       int int_pixel_val= (int)*pixel_val;
       image[y][x] = int_pixel_val;
       }
    }
  //release memory
  cvReleaseImage(&cv_image_gray);
  }

/* Compute the gradient magnitude and angles */
void compute_gradients() {
  int x,y;

  if(verbose>1) {
    printf("  Allocating gradient h/v image spaces\n");
    fflush(stdout);
  }

  h_mag = (double **)malloc(height*sizeof(double *));
  v_mag = (double **)malloc(height*sizeof(double *));

  for(y=0;y<height;y++) {
    h_mag[y] = (double *)malloc(width*sizeof(double));
    memset(h_mag[y],0,width*sizeof(double));
    v_mag[y] = (double *)malloc(width*sizeof(double));
    memset(v_mag[y],0,width*sizeof(double));
  }

  if(verbose>1) {
    printf("  Computing h/v gradients\n");
    fflush(stdout);
  }

  /* Yes, this is an array of function pointers */
  masks[gradient_mask]();

  if(verbose>1) {
    printf("  Allocating magnitude / angle image spaces\n");
    fflush(stdout);
  }

  mag = (double **)malloc(height*sizeof(double *));
  ang = (double **)malloc(height*sizeof(double *));
  for(y=0;y<height;y++) {
    mag[y] = (double *)malloc(width*sizeof(double));
    memset(mag[y],0,width*sizeof(double));
    ang[y] = (double *)malloc(width*sizeof(double));
    memset(ang[y],0,width*sizeof(double));
  }

  if(verbose>1) {
    printf("  Computing angles and magnitudes\n");
    fflush(stdout);
  }

  for(y=0;y<height;y++) {
    for(x=0;x<width;x++) {
      mag[y][x] = sqrt(h_mag[y][x] * h_mag[y][x] + v_mag[y][x] * v_mag[y][x]);
      ang[y][x] = atan2(v_mag[y][x] , h_mag[y][x])+M_PI;
    }
  }  

  if(verbose>1) {
    printf("  Freeing gradient h/v image spaces\n");
    fflush(stdout);
  }

  for(y=0;y<height;y++) {
    free(h_mag[y]);
    free(v_mag[y]);
  }
  free(h_mag);
  free(v_mag);

}

/* Label the image space with angle labels */
void label_angles(int nbuckets) {
  int x,y;
  double BucketWidth;

  if(verbose>1) {
    printf("  Allocating label space\n");
    fflush(stdout);
  }

  Lab1 = (int **)malloc(height*sizeof(int *));
  Lab2 = (int **)malloc(height*sizeof(int *));

  for(y=0;y<height;y++) {
    Lab1[y] = (int *)malloc(width*sizeof(int));
    memset(Lab1[y],0,width*sizeof(int));
    Lab2[y] = (int *)malloc(width*sizeof(int));
    memset(Lab2[y],0,width*sizeof(int));
  }

  BucketWidth = 2.0*M_PI / nbuckets;
  if(verbose>1) {
    printf("  ");
    printfloat(BucketWidth);
    fflush(stdout);
  }

  if(verbose>1) {
    printf("  Labeling\n");
    fflush(stdout);
  }

  for(y=0;y<height;y++) {
    for(x=0;x<width;x++) {
      if(mag[y][x] >= mag_thresh) {
	Lab1[y][x] = int(ang[y][x] / BucketWidth)+1;
	Lab1[y][x] = Lab1[y][x]>nbuckets?Lab1[y][x] - nbuckets:Lab1[y][x];
	
	Lab2[y][x] = int(ang[y][x]/BucketWidth + 0.5)+1;
	Lab2[y][x] = Lab2[y][x]>nbuckets?Lab2[y][x] - nbuckets:Lab2[y][x];
      }
    }
  }
}

/* Perform a Flood fill */
/* lab is array of arrays of labels */ 
/* reg is array of arrays of pointers to regions*/ 
/* x and y are our current Location */
/* rg is the region we are consitering joining */
int floodfill(int **lab,Region ***reg,int x,int y,Region *rg) {
  PointLst *pl;
  if(x<0 || x>=width || y<0 || y>= height || reg[y][x] != NULL || lab[y][x] != rg->lab)
    return 0;

  reg[y][x] = rg;

  pl = (PointLst *)malloc(sizeof(PointLst));
  pl->x = x;
  pl->y = y;
  pl->next = NULL;
  if(rg->p_hd == NULL)
    rg->p_hd = pl;
  else
    rg->p_tl->next = pl;
  rg->p_tl = pl;

  rg->count++;
  rg->minx = MIN(rg->minx,x);
  rg->miny = MIN(rg->miny,y);
  rg->maxx = MAX(rg->maxx,x);
  rg->maxy = MAX(rg->maxy,y);

  floodfill(lab,reg,x+1,y,rg);
  floodfill(lab,reg,x-1,y,rg);
  floodfill(lab,reg,x,y+1,rg);
  floodfill(lab,reg,x,y-1,rg);
  if(eight_connect) {
    floodfill(lab,reg,x+1,y+1,rg);
    floodfill(lab,reg,x-1,y-1,rg);
    floodfill(lab,reg,x-1,y+1,rg);
    floodfill(lab,reg,x+1,y-1,rg);
  }

  return 1;
}

/* Returns an empty New Region */
Region *NewRegion() {
  Region *rg;
  rg = (Region *)malloc(sizeof(Region));
  memset(rg,0,sizeof(Region));  // Set all elements to 0

  rg->minx=width;   // Init anything thats supposed 
  rg->miny=height;  // to be nonzero
  rg->valid = 1;
  return rg;
}

/* Perform the connected components to produce regions */
/* From Labels */
/* lab is an array of arrays of labels, passed by reference, hence 3 *'s */
/* reg is an array of arrays of pointers to regions, passed by reference,
    hence 4 *'s  */
/* r_hd and r_tl are the head and tail pointers, passed by reference: 2 *'s*/

void connected_components(int ***lab,Region ****reg, Region **r_hd, Region **r_tl) {
  int x,y;
  Region *rg;

  if(verbose>1) {
    printf("  Allocating Region Space\n");
    fflush(stdout);
  }

  *reg = (Region ***)malloc(height*sizeof(Region **));
  for(y=0;y<height;y++) {
    (*reg)[y] = (Region **)malloc(width*sizeof(Region *));
    memset((*reg)[y],0,width*sizeof(Region *));
  }

  if(verbose>1) {
    printf("  Floodfilling\n");
    fflush(stdout);
  }	
  rg = NewRegion();
  
  for(y=0;y<height;y++)
    for(x=0;x<width;x++) {
      if((*lab)[y][x]!=0) {
          rg->lab = (*lab)[y][x];
	      floodfill(*lab,*reg,x,y,rg);
	      if(rg->count) {
	          if(*r_hd == NULL)
                 *r_hd = rg;
              else
	              (*r_tl)->next = rg;
              *r_tl = rg;
              rg = NewRegion();
              }           
           }
       }
      
  free(rg); 
}

/* Prune out the "too small" regions */
void region_prune() {
  Region *RegPtr;

  for(RegPtr = RegList1_hd ; RegPtr!=NULL ; RegPtr=RegPtr->next) {
    if(RegPtr->count < count_thresh) {
      if(verbose>2) {
	printf("  Region with < %d pixels\n",count_thresh);
	fflush(stdout);
      }
      RegPtr->valid = 0;
    }      
  }
}


/* Given the plane equation and y and z, produce x */
double xofy(double y,double *X, double z) {
  return   -(y*X[1]+X[2]-z)/X[0];
}

/* Given the plane equation and x and z, produce y */
double yofx(double x,double *X, double z) {
  return -(x*X[0]+X[2]-z)/X[1];
}

/* Do all the math to rutn a region into a segment */
void region_computations() {
  Region *RegPtr;
  PointLst *PtPtr;
  int done,i,j;
  XYPoint tmp;

  // For each region
  for(RegPtr = RegList1_hd ; RegPtr!=NULL ; RegPtr=RegPtr->next) {
    // If its a real region
    if(RegPtr->valid) {
      // Compute M and B
      for(PtPtr= RegPtr->p_hd; PtPtr != NULL ; PtPtr=PtPtr->next) {
	register int x,y;
	x=PtPtr->x;
	y=PtPtr->y;
	RegPtr->M[0][0] += mag[y][x] * x * x;
	RegPtr->M[0][1] += mag[y][x] * x * y;
	RegPtr->M[0][2] += mag[y][x] * x;
	RegPtr->M[1][1] += mag[y][x] * y * y;
	RegPtr->M[1][2] += mag[y][x] * y;
	RegPtr->M[2][2] += mag[y][x];
      
	RegPtr->B[0] += mag[y][x] * image[y][x] * x;
	RegPtr->B[1] += mag[y][x] * image[y][x] * y;
	RegPtr->B[2] += mag[y][x] * image[y][x];
      }

      RegPtr->M[1][0] = RegPtr->M[0][1];
      RegPtr->M[2][0] = RegPtr->M[0][2];
      RegPtr->M[2][1] = RegPtr->M[1][2];

      // Compute K (for comuting Mi)
      RegPtr->K  = -RegPtr->M[0][0]*RegPtr->M[1][1]*RegPtr->M[2][2]+
	RegPtr->M[0][0]*RegPtr->M[1][2]*RegPtr->M[1][2]+
	RegPtr->M[0][1]*RegPtr->M[0][1]*RegPtr->M[2][2]-
	2*RegPtr->M[0][1]*RegPtr->M[0][2]*RegPtr->M[1][2]+
	RegPtr->M[0][2]*RegPtr->M[0][2]*RegPtr->M[1][1];
      
      if(RegPtr->maxx == RegPtr->minx) {
	// Handle vertical regions trivially
	RegPtr->pts[0].x = RegPtr->maxx;
	RegPtr->pts[0].y = RegPtr->miny;
	RegPtr->pts[1].x = RegPtr->maxx;
	RegPtr->pts[1].y = RegPtr->miny;
	RegPtr->pts[2].x = RegPtr->maxx;
	RegPtr->pts[2].y = RegPtr->maxy;
	RegPtr->pts[3].x = RegPtr->maxx;
	RegPtr->pts[3].y = RegPtr->maxy;	
      } else if(RegPtr->maxy == RegPtr->miny) {
	// Handle horizontal regions trivially
	RegPtr->pts[0].x = RegPtr->minx;
	RegPtr->pts[0].y = RegPtr->maxy;
	RegPtr->pts[1].x = RegPtr->minx;
	RegPtr->pts[1].y = RegPtr->maxy;
	RegPtr->pts[2].x = RegPtr->maxx;
	RegPtr->pts[2].y = RegPtr->maxy;
	RegPtr->pts[3].x = RegPtr->maxx;
	RegPtr->pts[3].y = RegPtr->maxy;	
      } else if(fabs(RegPtr->K) == 0.0) {
	// if its not horizontal, or vertical
        // and K=0, then we cannot invert M
        // So we cant use this region
	RegPtr->valid=0;
	printf("ERROR Region with singular M matrix! -- Skipping!\n");
	fflush(stdout);
      } else {
	// For "all other" regions
	// Compute M inverse
	RegPtr->Mi[0][0] = (-RegPtr->M[1][1]*RegPtr->M[2][2]+RegPtr->M[1][2]*RegPtr->M[1][2])/RegPtr->K;
	RegPtr->Mi[0][1] = (RegPtr->M[0][1]*RegPtr->M[2][2]-RegPtr->M[0][2]*RegPtr->M[1][2])/RegPtr->K;
	RegPtr->Mi[0][2] = (-RegPtr->M[0][1]*RegPtr->M[1][2]+RegPtr->M[0][2]*RegPtr->M[1][1])/RegPtr->K;
	RegPtr->Mi[1][0] = RegPtr->Mi[0][1];
	RegPtr->Mi[1][1] = (-RegPtr->M[0][0]*RegPtr->M[2][2]+RegPtr->M[0][2]*RegPtr->M[0][2])/RegPtr->K;
	RegPtr->Mi[1][2] = (RegPtr->M[0][0]*RegPtr->M[1][2]-RegPtr->M[0][1]*RegPtr->M[0][2])/RegPtr->K;
	RegPtr->Mi[2][0] = RegPtr->Mi[0][2];
	RegPtr->Mi[2][1] = RegPtr->Mi[1][2];
	RegPtr->Mi[2][2] = (-RegPtr->M[0][0]*RegPtr->M[1][1]+RegPtr->M[0][1]*RegPtr->M[0][1])/RegPtr->K;
      
	// Compire Mi * B
	RegPtr->MiB[0] = (RegPtr->Mi[0][0] * RegPtr->B[0]) + (RegPtr->Mi[0][1] * RegPtr->B[1]) + (RegPtr->Mi[0][2] * RegPtr->B[2]);
	RegPtr->MiB[1] = (RegPtr->Mi[1][0] * RegPtr->B[0]) + (RegPtr->Mi[1][1] * RegPtr->B[1]) + (RegPtr->Mi[1][2] * RegPtr->B[2]);
	RegPtr->MiB[2] = (RegPtr->Mi[2][0] * RegPtr->B[0]) + (RegPtr->Mi[2][1] * RegPtr->B[1]) + (RegPtr->Mi[2][2] * RegPtr->B[2]);
      
	// Compute average height
	RegPtr->height = RegPtr->B[2] / RegPtr->M[2][2];
          
	// compute 4 bounding box intersections
	RegPtr->pts[0].x = xofy(RegPtr->miny,RegPtr->MiB,RegPtr->height);
	RegPtr->pts[0].y = RegPtr->miny;
	RegPtr->pts[1].x = xofy(RegPtr->maxy,RegPtr->MiB,RegPtr->height);
	RegPtr->pts[1].y = RegPtr->maxy;
	RegPtr->pts[2].x = RegPtr->minx;
	RegPtr->pts[2].y = yofx(RegPtr->minx,RegPtr->MiB,RegPtr->height);
	RegPtr->pts[3].x = RegPtr->maxx;
	RegPtr->pts[3].y = yofx(RegPtr->maxx,RegPtr->MiB,RegPtr->height);

	// Sort them (by x for "more horizontal" regions)
        //           (by y for "more vertical" regions)
	j=1;
	do{
	  done=1;
	  for(i=0;i<4-j;i++) {
	    if( (fabs(RegPtr->MiB[0]) > fabs(RegPtr->MiB[1]))?
		(RegPtr->pts[i].y > RegPtr->pts[i+1].y):
		(RegPtr->pts[i].x > RegPtr->pts[i+1].x)) {
	      tmp = RegPtr->pts[i];
	      RegPtr->pts[i] = RegPtr->pts[i+1];
	      RegPtr->pts[i+1] = tmp;
	      done=0;
	    }
	  }
	  j++;
	} while(!done);
      }
    }
  }
}

/* All pixels cast their vote for their favorite region */
void vote() {
  int x,y;
  Region *RegPtr;

  // cast votes
  for(y=0;y<height;y++)
    for(x=0;x<width;x++)
      if(Lab1[y][x]!=0) {
	// Dont vote for invalid regions!
	if(Reg1[y][x]->count*Reg1[y][x]->valid >= Reg2[y][x]->count*Reg2[y][x]->valid)
	  Reg1[y][x]->votes++;
	else
	  Reg2[y][x]->votes++;
      }

  // Tally the votes
  for(RegPtr=RegList1_hd ; RegPtr!=NULL ; RegPtr=RegPtr->next)
    if(RegPtr->valid)
      RegPtr->percent = (double)RegPtr->votes / RegPtr->count;
}

/* Save all the lines in a nice file */
void save_lines(char *fname) {
  FILE *fout;
  Region *RegPtr;
  fout = fopen(fname,"w");
  for(RegPtr = RegList1_hd ; RegPtr!=NULL ; RegPtr=RegPtr->next) {
    if(RegPtr->valid && RegPtr->percent >= vote_thresh)
      fprintf(fout,"%f %f %f %f\n",RegPtr->pts[1].x,RegPtr->pts[1].y,RegPtr->pts[2].x,RegPtr->pts[2].y);
  }
  fclose(fout);
}


/* Save image with the lines in a nice file */
void save_lines_image(char*foriginal, char *fname) {//foriginal original  image
  //obtain gray image
  IplImage *myimage_out = 0;  
          if( (myimage_out = cvLoadImage(foriginal, 1)) == 0 )
             exit(0); 
  //IplImage *myimage_out= cvCreateImage(cvSize(width,height), IPL_DEPTH_8U, 1); 
  cvLine(myimage_out, cvPoint(0,0),cvPoint(20,20),cvScalar(0,0,255), 1);
  Region *RegPtr;

  int line_count=0;

  for(RegPtr = RegList1_hd ; RegPtr!=NULL ; RegPtr=RegPtr->next) {
    if(RegPtr->valid && RegPtr->percent >= vote_thresh){
           line_count++;
       cvLine(myimage_out, 
              cvPoint((int)RegPtr->pts[1].x,(int)RegPtr->pts[1].y),///////int rounds douwn!!!!!
              cvPoint((int)RegPtr->pts[2].x,(int)RegPtr->pts[2].y),
              cvScalar(0,0,255), 1);
   
       }
  }
/*WRITE TO IMGLINES*/
//Store the data into ImgLines
ImgLines *lines;
ImgLine  *line;
ImageLines image_lines;
lines = (ImgLines *) malloc(sizeof(ImgLines));
  lines->lines = (ImgLine *) malloc(line_count * sizeof(ImgLine));
  if (lines == NULL || lines->lines == NULL) {
    fprintf(stderr, "Error allocating space for lines.\n");
    exit(0);
  }
  strcpy(lines->img_name," ");
  lines->num_lines = 0;

for(RegPtr = RegList1_hd ; RegPtr!=NULL ; RegPtr=RegPtr->next) {
    if(RegPtr->valid && RegPtr->percent >= vote_thresh){
	(lines->num_lines)++;
	line = lines->lines + lines->num_lines - 1;
          line->num_pts = 2;
          line->pts = (ImgPt *) malloc(2 * sizeof(ImgPt));
          line->num = lines->num_lines;
	  line->label = 0;
          line->pts[0].r = RegPtr->pts[1].y;
          line->pts[0].c = RegPtr->pts[1].x;
          line->pts[0].num = 10 * line->num + 1;
          line->pts[0].v_r = line->pts[0].v_c = line->pts[0].cv_rc = 0;
          line->pts[1].r = RegPtr->pts[2].y;
          line->pts[1].c = RegPtr->pts[2].x;
          line->pts[1].num = 10 * line->num + 2;
          line->pts[1].v_r = line->pts[1].v_c = line->pts[1].cv_rc = 0;
	}
 }
 image_lines.C2Cpp(lines);
 image_lines.Write(fname);

//Free_ImgLines(lines);

// free(lines->lines);
// free (lines);
ImgLine *plin;
   
   plin = lines->lines;
   for(int i = 0; i < lines->num_lines; i++, plin++)
      free(plin->pts);
   free(lines->lines);
   free(lines);
   lines = NULL;   
/**/

  //write lined_image to file
char fname_image[512];
sprintf(fname_image,"%sLinesBurns.png",foriginal);

  if( !cvSaveImage(fname_image, myimage_out) ){
      printf("failed to write lined image file\n");
      exit(0);
      } 
  //release memory
  cvReleaseImage(&myimage_out);
}





/* Print out gobs of usage information */
void usage(char *prg) {
  int i;
  printf("Usage:\n"
	 "%s [-b n] [-c n] [-m f] [-v f] [-4|8] [-V] [-M m] pgmfile linefile\n"
	 "\n"
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
	 "-8 Use an eight connected neighborhood in flood fill.\n"
	 "-4 Use a four connected neighborhood in flood fill (Default).\n"
	 "-V Be verbose. Multiple V's increase verbosity.\n"
	 "-M specifies the mask to use for computing.\n"
	 "   the gradients.  Valid Masks are:\n"
	 ,prg);  
  for(i=0;i<NUM_MASKS;i++) {
    printf("     %s\n",masknames[i]);
 }
  exit(0);
}

int llinexBurns_arg(int argcount, char **argvalues)
{
 int opt,k;
  char *pgmin;
  char *linesout;
  // Cope with command line options
  while((opt = getopt(argcount, argvalues, "b:c:m:v:V48M:"))!=EOF) {
    switch(opt) {
    case 'M':
      for(k=0;k<NUM_MASKS;k++)
	if(!strcasecmp(optarg,masknames[k])) {
	  gradient_mask=k;
	  break;
	}
      if(k==NUM_MASKS) {
	printf("Invalid mask '%s' specified\n",optarg);
	usage(argvalues[0]);
      }
      break;
    case 'b':
      num_buckets = atoi(optarg);
      break;
    case 'c':
      count_thresh = atoi(optarg);
      if(count_thresh<3) {
	printf("-c value MUST be 3 or larger\n");
	usage(argvalues[0]);
      }
      break;
    case 'm':
      mag_thresh = atof(optarg);
      break;
    case 'v':
      vote_thresh = atof(optarg);
      break;
    case 'V':
      verbose++;
      break;
    case '8':
      eight_connect=1;
      break;
    case '4':
      eight_connect=0;
      break;
    case '?':
      usage(argvalues[0]);
      break;      
    }
  }
  k=0;
  while(optind<argcount) {
    switch(k++) {
    case 0:
      pgmin=argvalues[optind];
      break;
    case 1:
      linesout=argvalues[optind];
      break;
    }
    optind++;
  }

  if(k!=2) {
    printf("%s: Need exactly 2 filenames got %d\n",argvalues[0],k);
    usage(argvalues[0]);
  }
  
  if(verbose>1) {
    printint(num_buckets);
    printint(count_thresh);
    printfloat(mag_thresh);
    printfloat(vote_thresh);
  }
  /* Do all the "stuff needed" */
  if(verbose)
    printf("Loading image %s\n",pgmin);
  load_image(pgmin);  
  if(verbose)
    printf("Computing image gradient\n");
  compute_gradients();

  if(verbose)
    printf("Labeling regions\n");
  label_angles(num_buckets);
  
  if(verbose)
    printf("Connected Components1\n");
  connected_components(&Lab1,&Reg1,&RegList1_hd,&RegList1_tl);
   
  if(verbose)
    printf("Connected Components2\n");
  connected_components(&Lab2,&Reg2,&RegList2_hd,&RegList2_tl);
  /* Concat the two region lists together */
  RegList1_tl->next = RegList2_hd;
  
  if(verbose)
    printf("Performing region pruning\n");
  region_prune();

  if(verbose)
    printf("Performing region computations\n");
  region_computations();

  if(verbose)
    printf("Voting\n");
  vote();

  if(verbose)
    printf("Writing lines file %s\n",linesout);
    //save_lines(linesout);                /////commented - saving image not lines for now! 
   save_lines_image(pgmin,linesout);

//Free some memory

  for(int y=0;y<height;y++) 
	{free(image[y]);
	free(mag[y]);
	free(ang[y]);
        free(Lab1[y]);
        free(Lab2[y]);
	free(Reg1[y]);
	free(Reg2[y]);

	}

	free(image);
	free(mag);
	free(ang);
	free(Lab1);
	free(Lab2);
	free(Reg1);
 	free(Reg2);
        free(RegList1_hd);RegList1_hd=NULL;
	free(RegList1_tl);RegList1_tl=NULL;
	free(RegList2_hd);RegList2_hd=NULL;
	free(RegList2_tl);RegList2_tl=NULL;
  return(1);
}

bool llinexBurns(char* imagename,char* linesfilename, ImageLines &iml, int bucket_width=8, int min_num_pixels=25, double min_magnitude=5.0, double vote=0.5, int gradient_mask=2)//all other values are hardcoded
{
int argcount=13;
char** argval;


argval = (char **)malloc(argcount*sizeof(char *));
  for (int i=0; i< argcount; i++) argval[i]= (char*) malloc (255*sizeof(char));
 
strcpy(argval[0],"dummy");
strcpy(argval[1],"-b");
sprintf(argval[2],"%d",bucket_width);
strcpy(argval[3],"-c");
sprintf(argval[4],"%d",min_num_pixels);
//strcpy(argval[2],"50");
strcpy(argval[5],"-m");
sprintf(argval[6],"%.2f",min_magnitude);
strcpy(argval[7],"-v");
sprintf(argval[8],"%.2f",vote);
strcpy(argval[9],"-M");
strcpy(argval[10],masknames[gradient_mask]);
strcpy(argval[11],imagename);
strcpy(argval[12],linesfilename);
//strcpy(argval[11],"../datasets/microdrone_ipb/seq4/constraints_sift_dist_io/img_undistorted_0001.jpg");
//strcpy(argval[12],"/tmp/outlines.dat");

printf("converted the parameters to args:\n");
for (int i=0; i< argcount; i++)
printf("%s ",argval[i]);
printf("\n");

llinexBurns_arg(argcount, argval);

//for the iml: load the file... not elegant, but easy
iml.Read(linesfilename);

for (int i=0; i< argcount; i++) free(argval[i]);
free(argval);

return 1;
}
