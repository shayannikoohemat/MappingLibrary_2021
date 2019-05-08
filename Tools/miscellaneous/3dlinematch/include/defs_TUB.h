
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
//// ===========================================================================
//   T H E M A     : Architectural model Refinement using Terrestrial Image
//                   Sequences from a Trifocal sensor (ARTIST)
// ---------------------------------------------------------------------------
//   I N C L U D E : defs.h
//   V E R S I O N : 1.1
//   D A T U M     : 8. November 2006
// ---------------------------------------------------------------------------
//   (C) 2006, M. Heinrichs & V. Rodehorst {matzeh,vr}@cs.tu-berlin.de
// ===========================================================================

#ifndef _DEFS_H
#define _DEFS_H

#ifdef INTEL_COMPILER
#pragma warning (disable:181)  // Argument is incompatible with corresponding string conversion
#pragma warning (disable:981)  // operands are evaluated in unspecified order
#pragma warning (disable:1418) // external function definition with no prior declaration
//extern "C" long _ftol2(double);
//extern "C" long _ftol2_sse(double x) { return _ftol2(x); }
#endif

//#ifdef INTEL_COMPILER
#define PTRINT long				// use pointer as 32bit-integer
//#else
//#define PTRINT __int64          // use pointer as 32bit-integer
//#endif

#define _CRT_SECURE_NO_DEPRECATE  1                     // Visual Studio 2005
#define _CRT_NONSTDC_NO_DEPRECATE 1

#define PIQUARTER      0.78539816
#define PIHALF         1.57079632
#define PITHREEQUARTER 2.35619449
#define BIN_NORM       1.27323954  

#define END_CRITERIA(x) (fabs((x)/MIN(anorm,1.0)) < TINY)
#define MAXITS 40

#include <math.h>                                        // Standard-Includes
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <time.h>
#include <sys/timeb.h>
//#include <direct.h>
#include <ctype.h>

#include "const_TUB.h"                                          // Eigene Includes
#include "types_TUB.h"

#ifdef linux
typedef unsigned int UINT;
#endif 
extern  output report;
extern  char errorbuf[];
extern  homo2 NOHOMO2;
extern  homo3 NOHOMO3;
extern  bool conditionEssentialMatrix;



#define ROUND(x)       (int)((x) > 0 ? (x) + 0.5 : (x) - 0.5) /**< Returns the closest \c int to the \c real argument \a x. */
#define RNDUP(x)       (int)ceil(x) /**< Returns the value of the next \c int which is higher than the real argument \a x. */

#define ODD(x)          ((x) & 1)
#define EVEN(x)         !((x) & 1)
#define ABS(x)         ((x) > 0 ? (x) : -(x))  /**< Return the absolute value of the \c int or \c real number \a x. */  
#define MIN(x,y)       ((x) < (y) ? (x) : (y)) /**< Return the smaller of the two \c int or \c real values \a x and \a y. */ 
#define MAX(x,y)       ((x) > (y) ? (x) : (y)) /**< Return the greater of the two \c int or \c real values \a x and \a y. */
#define MINABS(x,y)    MIN(ABS(x),ABS(y)) /**< Return the smaller of the two \c int or \c real absolute values of \a x and \a y. */
#define MAXABS(x,y)    MAX(ABS(x),ABS(y)) /**< Return the greater of the two \c int or \c real absolute values of \a x and \a y. */
#define SQR(x)         ((x) * (x)) /**< Return the square of the \c int or \c real value \a x. */ 
#define SGN(x)         ((x) < -MIN_REAL ? -1 : ((x) > MIN_REAL ? 1 : 0))  // Vorzeichen   
#define MOD(x,y)       (((x)>=0)?((x)%(y)):((y)-1-(-(x)+(y)-1)%(y)))
#define PYTHAG(x,y)    sqrt(SQR(x) + SQR(y))

#define COLOR(r,g,b)   (((b)<<16) | ((g)<<8) | (r))
#define TO_COLOR(g)    COLOR(g, g, g)
#define TO_GRAY(r,g,b) (byte)(((r) + (g) + (b)) / 3)

#define SETBIT(i,n)    ((i) |= (1 << (n))) /**< Set bit \a n of \a i */
#define GETBIT(i,n)    ((i) & (1 << (n))) /**< Get bit \a n of \a i */
#define CLRBIT(i,n)    ((i) &= ~(1 << (n))) /**< Clear bit \a n of \a i */

#define GET_R(r)       ((r) & 0xff)                //  RGB Komponenten aus dem
#define GET_G(g)       (((g) >> 8) & 0xff)         // kompakten Integer-Format
#define GET_B(b)       (((b) >> 16) & 0xff)        //              extrahieren

#define GET_GRAY(g)    (byte)((GET_R(g) + GET_G(g) + GET_B(g)) / 3)

#define BLACK_COLOR    TO_COLOR(BLACK)
#define GRAY_COLOR     TO_COLOR(GRAY)
#define WHITE_COLOR    TO_COLOR(WHITE)

#define RED_COLOR      COLOR(MAX_GRAY, 0, 0)
#define GREEN_COLOR    COLOR(0, MAX_GRAY, 0)
#define BLUE_COLOR     COLOR(0, 0, MAX_GRAY)
#define CYAN_COLOR     COLOR(0, MAX_GRAY, MAX_GRAY)
#define MAGENTA_COLOR  COLOR(MAX_GRAY, 0, MAX_GRAY)
#define YELLOW_COLOR   COLOR(MAX_GRAY, MAX_GRAY, 0)

#define CLIP_GRAY(g)      (byte)MIN(MAX((g), 0), 255)
#define CLIP_GRAY_REAL(g) (byte)CLIP_GRAY_INT(ROUND(g))
#define CLIP_GRAY_double(g) (byte)CLIP_GRAY_INT(ROUND(g))
#define CLIP_GRAY_INT(x)  (byte)((x)&0xffffff00?((x)&0xf0000000?0:255):(x))  //turbo minmax
#define CLIP_GRAY_UINT(x) (byte)((x)&0xffffff00?255:(x)) //turbo max

#define SWAP(typ, a, b) { typ temp = a; a = b; b = temp; }

#define RAD2DEG(a)     ((a)*180.0 / PI) /**< Converts an angle \a a measured in radians to degrees. */
#define DEG2RAD(a)     ((a)*PI / 180.0) /**< Converts an angle \a a measured in degrees to radians. */
#define POSANGLE(a)    ((a) < 0.0 ? ((a) + 2.0*PI) : ((a) >= 2.0 * PI ? ((a) - 2.0*PI) : (a)))
#define MINANGLE(a)    ((a) < -PI ? ((a) + 2.0*PI) : ((a) >= PI ? ((a) - 2.0*PI) : (a)))

#define SIGN(u,v)      ((v)>=0.0 ? ABS(u) : -ABS(u))   // Numerical Recipies
#define SHFT(a,b,c,d)  (a)=(b); (b)=(c); (c)=(d);

#define ACOT(a)        atan2(1.0, (a))

#define ISZERO(a)      (ABS(a) < TINY)
#define ISNONZERO(a)   (ABS(a) > TINY)
#define ISPOSITIVE(a)  ((a) >  TINY)
#define ISNEGATIVE(a)  ((a) < -TINY)
#define ISEQUAL(a,b)   (ABS((a-b)) < TINY)
#define ISINEQUAL(a,b) (ABS((a-b)) > TINY)
#define ISINFINITE(a)  (ABS(a) == INFINITY)
#define ISFINITE(a)	   (ABS(a) != INFINITY)
#define SETINFINITY(a) (ISPOSITIVE(a) ? INFINITY : (ISNEGATIVE(a) ? -INFINITY : 0.0))
#define DIVIDE(a,b)    (ISZERO(b) ? SETINFINITY(a) : ((a) / (b)))


#endif
