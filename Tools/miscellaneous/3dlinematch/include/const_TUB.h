
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
#include <float.h> //for some eps values
#include <limits.h>
#include <stdlib.h> //for RAND_MAX

#ifndef _CONST_H
#define _CONST_H

#define BLACK           0                                         // Grauwerte
#define GRAY          127
#define WHITE         255

#define OUTSIDE_IMAGE   BLACK
#define DEFAULT_NAME    "none"
#define AUTHOR          "M. Heinrichs & V. Rodehorst {matzeh,vr}@cs.tu-berlin.de"
#define COMMENT         "#->"

#define GRAY_IMAGE      1                                         // Bildtypen
#define COLOR_IMAGE     3                                               // RGB
#define ALPHA_IMAGE     4                                              // RGBA

#define MAX_GRAY      255                                // Maximaler Grauwert
#define MAX_SHORT     65535
#define MAX_INT       INT_MAX
#define MIN_INT       INT_MIN
#define MAX_UINT      UINT_MAX
#define MAX_REAL      FLT_MAX //DBL_MAX
#define MIN_REAL      FLT_MIN //1.0e-120
#define RANDOM_MAX	  RAND_MAX
#define RANDOM_MAX30  0x3FFFFFFF                              // 30 bit random 
#define INFINITY      MAX_REAL

#define INT_EPS           1.0e-8          // Epsilon 10-8 = sqrt machine precision
#define REAL_EPS      DBL_EPSILON     //smallest number so that 1.0+REAL_EPS!=1.0
#define BUF_SIZE      1024               // Maximale Laenge einer Zeichenkette
#define RADIX         _DBL_RADIX
#define NUM_RAND_RUNS 24                          //for generating gauss noise


#define INIT_ERROR    -10                                      // Systemfehler
#define MEM_ERROR     -11
#define PBM_ERROR     -20                                       // Dateifehler
#define FLOAT_ERROR   -21
#define READ_ERROR    -22
#define WRITE_ERROR   -23
#define OPEN_ERROR    -30                                    // Benutzerfehler
#define OPTION_ERROR  -31
#define PARAM_ERROR   -32
#define USAGE_ERROR   -33

#ifdef WIN32                                              // Float Darstellung
#define MACH_CODE       5
#define WINVER          0x0501                            //Windows NT and XP
#else
#define MACH_CODE       0
#endif

#ifndef PI
#define PI 3.14159265358979323846                                 // Kreiszahl
#endif

#ifndef TWOPI
#define TWOPI 6.2831853071795864769                              // 2*Kreiszahl
#endif
                  // Numerical Recipies
#define ITMAX           200
#define TOL             2.0e-4
#define CGOLD           0.3819660
#define ZEPS            1.0e-20
#define GOLD            1.618034
#define GLIMIT          100.0
#define TINY            DBL_EPSILON           // 1.0e-32
#define FTOL            FLT_EPSILON    


#define BAD_DISP       -32768       // value for a bad disparity (double or int)
#define OCCLUSION      -32767       // value for a occluded pixel
#define NO_OFFSET       999999999     // value for no offset
#define NEW_RAW_FORMAT -1234          // Indicates new raw format 
#define MAX_SIFT_DIST   32000         // Min quality of a SIFT match = 128*SQR(x), where x is the max allowed difference 

#endif
