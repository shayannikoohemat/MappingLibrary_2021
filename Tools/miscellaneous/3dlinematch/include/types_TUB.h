
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
#ifndef _TYPES_H
#define _TYPES_H

// ===========================================================================
//                               Aufzaehlungstypen
// ===========================================================================

enum robust {HUBER, TUKEY, LMEDS}; typedef enum robust robust;			       // Robuste Methoden
enum output {QUIET, NORMAL, VERBOSE}; typedef enum output output;               // Anzeigeoptionen
enum optimize {POWELL, LEVMAR}; typedef enum  optimize optimize;		    // Nicht-lineare Optimierungen
enum  stitch {CAMERA, COORD_PROGEO, FACTOR, BOTH}; typedef enum  stitch stitch;	   // Verknuepfung
enum  selectNumber {USENUM, USEMAX, USEALL}; typedef enum  selectNumber selectNumber;	           // Punktauswahl
enum asciityp {XY, NXY, NXYQ, XYRGB, NXYRGB, NXYRGBQ, XYZ, NXYZ, NXYZQ, XYZRGB, NXYZRGB, NXYZRGBQ, ROMA}; typedef enum  asciityp asciityp;
enum filePermission {EXIST=0, WRITE=2, READ=4, READWRITE=6}; typedef enum  filePermission filePermission; // Filecheck
enum MatchDirection {BASE_HORI, HORI_BASE, BASE_VERT, VERT_BASE, HORI_VERT, VERT_HORI} ; typedef enum MatchDirection MatchDirection;
enum norm {L1NORM, L2NORM, LINFNORM, FROBENIUS, MAXNORM, MINNORM, MEANNORM, EUCLID, ADAPTNORM}; typedef enum norm norm;
enum CornerDetector {LOWE, FOERSTNER_KLT, FOERSTNER, HARRIS}; typedef enum CornerDetector CornerDetector;
enum tens {STRUCTURE, BOUNDARY, ENERGY}; typedef enum  tens tens; // Image processing tensors
enum accumulation {UNIFORM, GAUSSIAN, HOURGLASS}; typedef enum  accumulation accumulation;      // Gradient sum

// ===========================================================================
//                             Basis - Datentypen
// ===========================================================================

#ifndef __cplusplus
#include <stdbool.h>
#endif

typedef unsigned char  uchar;                                  // Abkuerzungen
#ifdef _WIN32
typedef unsigned short ushort;
typedef unsigned int   uint;
typedef unsigned long  ulong;
//typedef unsigned short ushort;
typedef unsigned int   UINT;
//typedef unsigned long  ulong;
#endif

//typedef double double;                                        // Fliesskommazahl
typedef uchar  byte;                                         // 8 Bit = 1 Byte

// ===========================================================================
//                      Anwendungsspezifische Strukturen
// ===========================================================================

typedef struct homo2 {                       // Homogener 2D Bildpunkt (x,y,w) 
	long id;                                 // bzw. 2D Gerade (a,b,c)
	union { double x, a; };
	union { double y, b; };
	union { double w, c; };                                     
} homo2;

typedef struct homo3 {                     // Homogener Objektpunkt (X,Y,Z,W)
    long id;                               // bzw. Ebene (A,B,C,D)
	union { double X, A; };
	union { double Y, B; };
	union { double Z, C; }; 
	union { double W, D; }; 
} homo3;

typedef struct vektor {                                              // Vektor
    int len;
    double *v;
} vektor;

typedef struct matrix {                                              // Matrix
    int cols, rows;
    double **m;
} matrix;

typedef struct image {                                          // Bild (Byte)
    char *name;
    int w, h;
    int lineInc;
	int tagc;
	char **tagv;
    byte typ;
    byte *d;
} image;

typedef struct raster {                                        // Bild (Float)
    int w, h;
    double *d;
    double **m;
} raster;

typedef struct cube {                                       // TriFocal-Tensor
    int cols, rows, dep;
    double ***c;
    matrix *P[3];
} cube;

typedef struct pyramid {                                // Aufl�ungs Pyramide
    int num;
    int level;
    double sigma;
    double factor;
} pyramid;

typedef struct celem {                                         // Corner Liste
    long id;
    double x, y;
    byte level;
    double value;												   // cornerness / weight
    double cxx, cxy, cyy;                                        // covariance matrix
    struct celem *next;
} celem;


typedef struct clist {                                         // Corner Liste
    long num;
    bool compact;
    struct celem *start;
    struct celem *end;
} clist;

typedef struct pelem {                                       // Bildpunkt Liste
    long id;
    double x, y;
	int  color;
	double quality;
    struct pelem *next;
} pelem;

typedef struct plist {                                          
	long num;
	bool compact;
    struct pelem *start;
	struct pelem *end;
} plist;

typedef struct xelem {                                     // Objektpunkt Liste
    long id;
    double X, Y, Z;
	int  color;
	double quality;
    struct xelem *next;
} xelem;

typedef struct xlist {                                          
	long num;
	bool compact;
	struct xelem *start;
	struct xelem *end;
} xlist;

typedef struct mesh {                                                // 3D Mesh
    int cols;
    int rows;
    bool clean;
    xelem ***m;
} mesh;

typedef struct mask {                                      // Convolution mask
    double sigma;											 // Standard deviation
    int radius;                                            // sqrt(2)*PI*sigma
    int size;												     // 2*radius+1
    double *d1;									   // One-dimensinal mask data
    double *d2;                      // if two-dimensional mask, the y direction
} mask;

typedef struct listarg {                                     // Korrespondenzen 
    struct plist *p1;
	struct plist *p2;
	struct plist *p3;
    struct xlist *x1;
	struct xlist *x2;
} listarg;

typedef struct gelem {                                                  // Gen
    int  *gen;
    double fitness;
    struct gelem *next;
} gelem;

typedef struct glist {                                            // Chromosom
	long num;
	struct gelem *start;
	struct gelem *end;
} glist;

typedef struct relem {                      // Reconstruction
	matrix *P;                              // Projection matrices
    plist *pl;                              // Image points
    long id;
	struct relem *prev;
    struct relem *next;
} relem;

typedef struct rlist {                      // Reconstruction
	long num;                               // Number of Images
    xlist *xl;                              // Object points
	struct relem *start;
	struct relem *end;
} rlist;

typedef struct region {                                      // Bildausschnitt
    int x, y;
    int w, h;
} region;

typedef struct trans {
    double tx, ty, tz;                                            // Translation
    double sx, sy, sz;                                             // Skalierung
} trans;

typedef struct box {                                           // Bounding Box
    double x1, y1;                                                  // 4 corners
    double x2, y2;
    double x3, y3;
    double x4, y4;
    double xn, yn;                                                 // min corner
    double xx, yx;                                                 // max corner
} box;

typedef struct cparam {                                    // Corner Parameter
    double sigma;							 // Standard deviation for convolution
    double sigma2;						// Standard deviation for accumulation
    double round;
    double weight;
    int adapt;
    int maxnum;
    int radius;                                 // Mask radius for convolution
    int radius2;                               // Mask radius for accumulation
    CornerDetector method;         // Foerstner-KLT, Foerstner, HarrisStephens
	tens tensor;                   // Structure-, energy- or boundary tensor
	accumulation accu;			   // Uniform, Gaussian or hourglass summation
    bool subpix;
    pyramid pyr;
} cparam;

typedef struct mparam {                                     // Match Parameter
    int size;
    double sigma;
    double value;
    double weight;
    int level;
} mparam;

typedef struct camera {
    matrix *P;                // Projection matrix;
	homo2 s;                  // Symmetry point
    double k1, k2, k3;          // Radial distortion coefficients
} camera;

typedef struct KeypointSt {
  double row, col;              // Subpixel location of keypoint.
  double scale, ori;            // Scale and orientation (range [-PI,PI])
  byte descrip[128];          // Vector of descriptor values
  struct KeypointSt *next;    // Pointer to next keypoint in list.
} *Keypoint;

typedef struct {                                 // Levenber-Marquardt
  double ftol;       // relative error desired in the sum of squares.
  double xtol;       // relative error between last two approximations.
  double gtol;       // orthogonality desired between fvec and its derivs.
  double epsilon;    // step used to calculate the jacobian.
  double stepbound;  // initial bound to steps in the outer loop.
  double fnorm;      // norm of the residue vector fvec.
  int maxcall;       // maximum number of iterations.
  int nfev;          // actual number of iterations.
  int info;          // status of minimization.
} lm_control_type;

typedef void (lm_evaluate_ftype) (double *par, double *fvec, double *data, int m, double (*func)(int , double *, double *));

typedef struct MatchedPointOld {
    ushort x[3], y[3];
    uint color;
} MatchedPointOld;

typedef struct MatchListOld {
    int size;
    MatchedPointOld *points;
    double offX[3], offY[3];
} MatchListOld;

typedef struct MatchedPoint {
    double X[3], Y[3];
    uint color;
} MatchedPoint;

typedef struct MatchList {
    int size;
    MatchedPoint *points;
    double offX[3], offY[3];
} MatchList;

#endif
