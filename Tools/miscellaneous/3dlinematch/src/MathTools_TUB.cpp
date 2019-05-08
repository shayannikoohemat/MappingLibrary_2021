
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
//
//FROM M. Hinchrichs/V. Rodehorst

#include "MathTools_TUB.h"
output report = VERBOSE;
char errorbuf[BUF_SIZE];
homo2 NOHOMO2 = {0, 0.0, 0.0, 0.0};
homo3 NOHOMO3 = {0, 0.0, 0.0, 0.0, 0.0};

//definitions for correllation functions
#define GX(l)  (rx.d[l] + gx.d[l] + bx.d[l])
#define GY(l)  (ry.d[l] + gy.d[l] + by.d[l])
#define GXY(l) (rx.d[l]*ry.d[l] + gx.d[l]*gy.d[l] + bx.d[l]*by.d[l])
#define GX2(l) (SQR(rx.d[l]) + SQR(gx.d[l]) + SQR(bx.d[l]))
#define GY2(l) (SQR(ry.d[l]) + SQR(gy.d[l]) + SQR(by.d[l]))
#define ADD(l) (addr.d[l] + addg.d[l] + addb.d[l])

//for file-functions
static char *pointformat[] = {"UNKNOWN", "XY", "N-XY", "N-XY-Q", "XY-RGB", "N-XY-RGB", "N-XY-RGB-Q", "ROMA-BKO"};
static char *coordformat[] = {"UNKNOWN", "XYZ", "N-XYZ", "N-XYZ-Q", "XYZ-RGB", "N-XYZ-RGB", "N-XYZ-RGB-Q", "ROMA-PKO"};

//for eigen-functions
#define RADIX 2.0 //needed for machine accuracy
#define MAX_EIG_ITER 16 //max number of itterations to find a eigenvector and max trys for random vectors. if this is exceeded the vector is too difficult to be estimated
#define LARGE_GROWTH 1000 //min growth for random start value
#define EIG_ZERO 1e-14  //eigenvalues with a complex part less than this number are assumed to be only double!
#define UPDATE_EIGVALUE_TOLLERANCE 0.5 //the eigenvalue could only be "enhanced" by +-1/0.5= +-2
#define EIG_CONVERGENCE 1e-16

//for cheirality-functions
#define PHI2 0.1

//CANNY
#define PIQUARTER      0.78539816
#define PIHALF         1.57079632
#define PITHREEQUARTER 2.35619449
#define BIN_NORM       1.27323954   //  4 / PI

matrix* NonMaxSuppression (image *input, double sigma){
    image *tmp=NULL;
    matrix *g1=NULL, *g2=NULL;
    double x, y, a, deg, g[2]={0};
    int i, j;

    if(!CheckImage(input)){
        Warning("CannyEdge: No input data!");
        return NULL;
    }

    const int w= input->w;
    const int h= input->h;
    const int ws= input->w-1;
    const int hs= input->h-1;

    //get gradient
    if(input->typ==COLOR_IMAGE){
        tmp= ColorToGray(input);
        Gradient(sigma, 0, tmp, &g1, &g2);
        tmp = FreeImage(tmp);
    }
    else {
        Gradient(sigma, 0, input, &g1, &g2);
    }

    //calc magnitude and direction and put direction to a 4bin histogram
    //g1: magnitude
    //g2: direction

    for(i=0; i<h; ++i){
        for(j=0; j<w; ++j){
            x=g1->m[i][j];
            y=g2->m[i][j];
            g1->m[i][j]= sqrt(SQR(x)+SQR(y));
            deg= atan2(y,x); //angle in radiants
            g2->m[i][j] = deg<0.0?deg+PI:deg; //mirror lower semi circle
        }
    }

    //set borderpixel to 0
    memset(g1->m[0], 0, sizeof(double)*w); //first line
    memset(g1->m[hs], 0, sizeof(double)*w); //last line
    for(i=1; i<hs; ++i){
        g1->m[i][0]=0;
        g1->m[i][ws]=0;
    }

    //non maxima suppression in a 8-neighbourhood according to the direction
    for(i=1; i<hs; ++i){
        for(j=1; j<ws; ++j){
            a   = g1->m[i][j];
            deg = g2->m[i][j];

            if(deg<PIHALF){ //0...90
                if(deg<PIQUARTER){ //0..45
                    deg =deg*BIN_NORM;
                    g[0]=(deg)*g1->m[i+1][j+1]+ (1.0-deg)*g1->m[i  ][j+1];
                    g[1]=(deg)*g1->m[i-1][j-1]+ (1.0-deg)*g1->m[i  ][j-1];
                }
                else{ //45..90
                    deg =(deg-PIQUARTER)*BIN_NORM;
                    g[0]=(deg)*g1->m[i+1][j  ]+ (1.0-deg)*g1->m[i+1][j+1];
                    g[1]=(deg)*g1->m[i-1][j  ]+ (1.0-deg)*g1->m[i-1][j-1];
                }
            }
            else{ //90..189
                if(deg<PITHREEQUARTER){ //90..135
                    deg =(deg-PIHALF)*BIN_NORM;
                    g[0]=(deg)*g1->m[i+1][j-1]+ (1.0-deg)*g1->m[i+1][j  ];
                    g[1]=(deg)*g1->m[i-1][j+1]+ (1.0-deg)*g1->m[i-1][j  ];
                }
                else{ //135..180
                    deg =(deg-PITHREEQUARTER)*BIN_NORM;
                    g[0]=(deg)*g1->m[i  ][j-1]+ (1.0-deg)*g1->m[i+1][j-1];
                    g[1]=(deg)*g1->m[i  ][j+1]+ (1.0-deg)*g1->m[i-1][j+1];
                }
            }

            if(g[0]>a || g[1]>a)
                g1->m[i][j]=0;
        }
    }
    g2 = mat_free(g2);
    return g1;
}


matrix* CannyEdge (image *input, double sigma, double lowThresh, double highThresh){
    image *tmp=NULL;
    matrix *g1=NULL, *g2=NULL;
    double a=0, x, y, deg, g[2]={0};
    int i, j, count;

    if(!CheckImage(input)){
        Warning("CannyEdge: No input data!");
        return NULL;
    }

    const int w= input->w;
    const int h= input->h;
    const int ws= input->w-1;
    const int hs= input->h-1;

    //get gradient
    if(input->typ==COLOR_IMAGE){
        tmp= ColorToGray(input);
        Gradient(sigma, 0, tmp, &g1, &g2);
        tmp = FreeImage(tmp);
    }
    else {
        Gradient(sigma, 0, input, &g1, &g2);
    }

    //calc magnitude and direction and put direction to a 4bin histogram
    //g1: magnitude
    //g2: direction
    a=0;
    for(i=0; i<h; ++i){
        for(j=0; j<w; ++j){
            x=g1->m[i][j];
            y=g2->m[i][j];
            g1->m[i][j]= sqrt(SQR(x)+SQR(y));
            a +=g1->m[i][j]; //for average gradient
            deg= atan2(y,x); //angle in radiants
            g2->m[i][j] = deg<0.0?deg+PI:deg; //mirror lower semi circle
        }
    }

    //adjust thresholds
    a/=(double)w*h;
    lowThresh *=a;
    highThresh*=lowThresh;

    //set borderpixel to 0
    memset(g1->m[0], 0, sizeof(double)*w); //first line
    memset(g1->m[hs], 0, sizeof(double)*w); //last line
    for(i=1; i<hs; ++i){
        g1->m[i][0]=0;
        g1->m[i][ws]=0;
    }

    //non maxima suppression in a 8-neighbourhood according to the direction
    for(i=1; i<hs; ++i){
        for(j=1; j<ws; ++j){
            a   = g1->m[i][j];
            deg = g2->m[i][j];

            if(deg<PIHALF){ //0...90
                if(deg<PIQUARTER){ //0..45
                    deg =deg*BIN_NORM;
                    g[0]=(deg)*g1->m[i+1][j+1]+ (1.0-deg)*g1->m[i  ][j+1];
                    g[1]=(deg)*g1->m[i-1][j-1]+ (1.0-deg)*g1->m[i  ][j-1];
                }
                else{ //45..90
                    deg =(deg-PIQUARTER)*BIN_NORM;
                    g[0]=(deg)*g1->m[i+1][j  ]+ (1.0-deg)*g1->m[i+1][j+1];
                    g[1]=(deg)*g1->m[i-1][j  ]+ (1.0-deg)*g1->m[i-1][j-1];
                }
            }
            else{ //90..189
                if(deg<PITHREEQUARTER){ //90..135
                    deg =(deg-PIHALF)*BIN_NORM;
                    g[0]=(deg)*g1->m[i+1][j-1]+ (1.0-deg)*g1->m[i+1][j  ];
                    g[1]=(deg)*g1->m[i-1][j+1]+ (1.0-deg)*g1->m[i-1][j  ];
                }
                else{ //135..180
                    deg =(deg-PITHREEQUARTER)*BIN_NORM;
                    g[0]=(deg)*g1->m[i  ][j-1]+ (1.0-deg)*g1->m[i+1][j-1];
                    g[1]=(deg)*g1->m[i  ][j+1]+ (1.0-deg)*g1->m[i-1][j+1];
                }
            }

            if(g[0]>a || g[1]>a)
                g1->m[i][j]=0;
        }
    }

    //set output to 0:
    memset(g2->m[0], 0, sizeof(double)*h*w);

    //pre format: cathegorize to three values: 0 for g1<lowThres, 1 for lowThres<=g1<highThres, 2 for g2>=highThres,
    count =0;
    for(i=0; i<h; ++i){
        for(j=0; j<w; ++j){
            x= g1->m[i][j];
            if(x<lowThresh){
                g1->m[i][j]=0;
            }
            else{
                if(x<highThresh){
                    g1->m[i][j]=1;
                }
                else{
                    g1->m[i][j]=2;
                    g2->m[i][j]=1.0;
                    ++count;
                }
            }
        }
    }

    //hysteresis part
    while(count>0){
        count = 0;
        for(i=1; i<hs; ++i){
            for(j=1; j<ws; ++j){
                if(ISEQUAL(g1->m[i][j],2)){
                    if( ISEQUAL(g1->m[i-1][j-1], 1)){
                        g1->m[i-1][j-1]=2;
                        g2->m[i-1][j-1]=1.0;
                        ++count;
                    }
                    if( ISEQUAL(g1->m[i-1][j  ], 1)){
                        g1->m[i-1][j  ]=2;
                        g2->m[i-1][j  ]=1.0;
                        ++count;
                    }
                    if( ISEQUAL(g1->m[i-1][j+1], 1)){
                        g1->m[i-1][j+1]=2;
                        g2->m[i-1][j+1]=1.0;
                        ++count;
                    }
                    if( ISEQUAL(g1->m[i  ][j-1], 1)){
                        g1->m[i  ][j-1]=2;
                        g2->m[i  ][j-1]=1.0;
                        ++count;
                    }
                    if( ISEQUAL(g1->m[i  ][j+1], 1)){
                        g1->m[i  ][j+1]=2;
                        g2->m[i  ][j+1]=1.0;
                        ++count;
                    }
                    if( ISEQUAL(g1->m[i+1][j-1], 1)){
                        g1->m[i+1][j-1]=2;
                        g2->m[i+1][j-1]=1.0;
                        ++count;
                    }
                    if( ISEQUAL(g1->m[i+1][j  ], 1)){
                        g1->m[i+1][j  ]=2;
                        g2->m[i+1][j  ]=1.0;
                        ++count;
                    }
                    if( ISEQUAL(g1->m[i+1][j+1], 1)){
                        g1->m[i+1][j+1]=2;
                        g2->m[i+1][j+1]=1.0;
                        ++count;
                    }
                    g1->m[i][j]=3; //done here!
                }
            }
        }
    }

    g1= mat_free(g1);

    return g2;
}

//CHECK
//#include <io.h>

// ===========================================================================
//
// ===========================================================================

double CheckZero (double t)
{
    if (ISZERO(t)) {
        Warning ("Division by zero!");
        t = SGN(t) * MIN_REAL;
    }
    return t;
}

// ===========================================================================
//
// ===========================================================================

bool CheckCube (cube *C)
{
    if ((C == NULL) || (C->cols < 1) || (C->rows < 1) || (C->dep < 1)) {
        Warning ("Cube empty!");
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckPoints (pelem *p)
{
    if (p == NULL) {
        Warning ("Point element empty!");
		return false;
    }
	return true;
}

bool CheckPoints (plist *pl)
{
    if ((pl == NULL) || (pl->num < 1)) {
        Warning ("Point list empty!");
		return false;
    }
	return true;
}

bool CheckSizePoints (plist *pl, int num)
{
    if (pl == NULL) return false;
	if (pl->num != num) {
		Warning ("Point list %d has not required size %d", pl->num, num);
		return false;
    }
	return true;
}

bool CheckSamePoints (plist *pl1, plist *pl2)
{
    if (!CheckPoints(pl1) || !CheckPoints(pl2)) return false;
	if (pl1->num != pl2->num) {
		Warning ("Point list %d and %d have different size %d", pl1->num, pl2->num);
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckCoords (xelem *x)
{
    if (x == NULL) {
        Warning ("Coord element empty!");
		return false;
    }
	return true;
}

bool CheckCoords (xlist *xl)
{
    if ((xl == NULL) || (xl->num < 1)) {
        Warning ("Coord list empty!");
		return false;
    }
	return true;
}

bool CheckSizeCoords (xlist *xl, int num)
{
    if (xl == NULL) return false;
	if (xl->num != num) {
		Warning ("Coord list %d has not required size %d", xl->num, num);
		return false;
   }
	return true;
}

bool CheckSameCoords (xlist *xl1, xlist *xl2)
{
    if (!CheckCoords(xl1) || !CheckCoords(xl2)) return false;

	if (xl1->num != xl2->num) {
		Warning ("Point list %d and %d have different size %d", xl1->num, xl2->num);
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckCorners (celem *c)
{
    if (c == NULL) {
        Warning ("Corner element empty!");
		return false;
    }
	return true;
}

bool CheckCorners (clist *cl)
{
    if ((cl == NULL) || (cl->num < 1)) {
        Warning ("Corner list empty!");
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckMask (mask *M)
{
    if ((M == NULL) || (M->radius < 1) || (M->d1 == NULL)) {
        Warning ("Mask empty!");
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckMatrix (matrix *M)
{
    if ((M == NULL) || (M->cols < 1) || (M->rows < 1)) {
        Warning ("Matrix empty!");
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckFiniteMatrix (matrix *M)
{
    long i;

	if (!CheckMatrix (M)) return false;

	for (i=0; i<M->rows * M->cols; ++i) {
		if (fabs(M->m[0][i]) > MAX_REAL) {
			Warning ("Matrix element infinite!");
		    return false;
		}
	}
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckSquareMatrix (matrix *M)
{
	if (!CheckMatrix (M)) return false;

    if (M->rows != M->cols) {
        Warning ("%d x %d matrix not square!", M->rows, M->cols);
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckSymMatrix (matrix *M)
{
    int i, j;

	if (!CheckSquareMatrix (M)) return false;

    for (i=0; i<M->rows; ++i) {
        for (j=i+1; j<M->cols; ++j) {
            if (fabs(M->m[i][j] - M->m[j][i]) > REAL_EPS) {
                Warning ("matrix non symmetric!");
				return false;
            }
        }
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckPosSymMatrix (matrix *M)
{
    int i, s;

	if (!CheckSymMatrix (M)) return false;

    s = SGN(M->m[0][0]);             // Globales negatives Vorzeichen moeglich

    for (i=0; i<M->cols; ++i) {
        if (SGN(M->m[i][i]) != s) {
            Warning ("matrix not positiv definite!");
			return false;
        }
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckSizeMatrix (matrix *M, int rows, int cols)
{
	if (M == NULL) return false;

    if ((M->rows != rows) || (M->cols != cols)) {
        Warning ("%d x %d matrix has not required size %d x %d!", M->rows, M->cols, rows, cols);
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckCompatible (matrix *A, matrix *B)
{
	if (!CheckMatrix (A) || !CheckMatrix (B)) return false;

    if (A->cols != B->rows) {
        Warning ("matrices %d x %d and %d x %d are incompatible!",
                  A->rows, A->cols, B->rows, B->cols);
		return false;
    }
	return true;
}

bool CheckCompatible (matrix *A, vektor *B)
{
	if (!CheckMatrix (A) || !CheckVector (B)) return false;

    if (A->cols != B->len) {
        Warning ("matrix %d x %d and vector %d are incompatible!",
                  A->rows, A->cols, B->len);
		return false;
    }
	return true;
}

bool CheckCompatible (vektor *A, matrix *B)
{
	if (!CheckVector (A) || !CheckMatrix (B)) return false;

    if (A->len != B->rows) {
        Warning ("vector %d and matrix %d x %d are incompatible!",
                  A->len, B->rows, B->cols);
		return false;
    }
	return true;
}
bool CheckCompatibleMatrix (matrix *A, matrix *B)
{
	if (!CheckMatrix (A) || !CheckMatrix (B)) return false;

    if (A->cols != B->rows) {
        printf ("matrices %d x %d and %d x %d are incompatible!",
                  A->rows, A->cols, B->rows, B->cols);
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckSameMatrix (matrix *A, matrix *B)
{
	if (!CheckMatrix (A) || !CheckMatrix (B)) return false;

    if ((A->rows != B->rows) || (A->cols != B->cols)) {
        Warning ("matrices %d x %d and %d x %d have different size!",
                  A->rows, A->cols, B->rows, B->cols);
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckMesh (mesh *M)
{
    if ((M == NULL) || (M->cols < 1) || (M->rows < 1)) {
        Warning ("Mesh empty!");
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckVector (vektor *V)
{
    if ((V == NULL) || V->len < 1) {
        Warning ("vector empty!");
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckSizeVector (vektor *V, int len)
{
	if (V == NULL) return false;

    if (V->len != len) {
        Warning ("vector %d has not required size %d", V->len, len);
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckSameVector (vektor *A, vektor *B)
{
	if (!CheckVector (A) || !CheckVector (B)) return false;

    if (A->len != B->len) {
        Warning ("vectors %d and %d have different size!", A->len, B->len);
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckIndex (vektor *A, int i)
{
    if (!CheckVector (A)) return false;

    if ((i < 0) || (i >= A->len)) {
        Warning ("illegal index %d!", i);
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckIndex (plist *pl, int i)
{
    if (!CheckPoints(pl)) return false;

    if ((i < 0) || (i >= pl->num)) {
        Warning ("illegal index %d!", i);
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckIndex (xlist *xl, int i)
{
    if (!CheckCoords(xl)) return false;

    if ((i < 0) || (i >= xl->num)) {
        Warning ("illegal index %d!", i);
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckIndex (clist *cl, int i)
{
    if (!CheckCorners(cl)) return false;

    if ((i < 0) || (i >= cl->num)) {
        Warning ("illegal index %d!", i);
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckIndex (matrix *M, int r, int c)
{
    if (!CheckMatrix(M)) return false;

    if ((r < 0) || (c < 0) || (r >= M->rows) || (c >= M->cols)) {
        Warning ("illegal index (%d, %d)!", r, c);
		return false;
    }
	return true;
}

bool CheckRowIndex (matrix *M, int i)
{
    if (!CheckMatrix (M)) return false;

    if ((i < 0) || (i >= M->rows)) {
        Warning ("illegal rows index %d!", i);
		return false;
    }
	return true;
}

bool CheckRowIndex (mesh *M, int i)
{
    if (!CheckMesh(M)) return false;

    if ((i < 0) || (i >= M->rows)) {
        Warning ("illegal rows index %d!", i);
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckColIndex (matrix *M, int i)
{
    if (!CheckMatrix (M)) return false;

    if ((i < 0) || (i >= M->cols)) {
        Warning ("illegal column index %d!", i);
		return false;
    }
	return true;
}

bool CheckColIndex (mesh *M, int i)
{
    if (!CheckMesh (M)) return false;

    if ((i < 0) || (i >= M->cols)) {
        Warning ("illegal column index %d!", i);
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckImage (image *pic)
{
    if (pic == NULL) {
        Warning ("image empty!");
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckGrayImage (image *pic)
{
    if (!CheckImage (pic)) return false;

    if (pic->typ != GRAY_IMAGE) {
        Warning ("no gray value image!");
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckColorImage (image *pic)
{
    if (!CheckImage (pic)) return false;

    if (pic->typ != COLOR_IMAGE) {
        Warning ("no color image!");
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckSizeImage (image *pic1, image *pic2)
{
    if (!CheckImage (pic1) || !CheckImage (pic2)) return false;

    if ((pic1->w != pic2->w) || (pic1->h != pic2->h)) {
        Warning ("image %d x %d and %d x %d have different size!",
                  pic1->w, pic1->h, pic2->w, pic2->h);
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckSquareImage (image *pic)
{
    if (!CheckImage (pic)) return false;

    if (pic->w != pic->h) {
        Warning ("image %d x %d not square!", pic->w, pic->h);
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckTypImage (image *pic1, image *pic2)
{
    if (!CheckImage (pic1) || !CheckImage (pic2)) return false;

    if (pic1->typ != pic2->typ) {
        Warning ("images have different color type!");
		return false;
    }
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckSameImage (image *pic1, image *pic2)
{
    return (CheckSizeImage (pic1, pic2) && CheckTypImage (pic1, pic2));
}

// ===========================================================================
//
// ===========================================================================

bool CheckPointsSorted (plist *pl)
{
	long a, b;
	pelem *p;

	if (CheckPoints(pl) && (pl->num > 1)) {
		a = pl->start->id;
		if (pl->compact) {
			for (p=pl->start+1; p <= pl->end; ++p) {
                b = p->id;
				if (a > b) return false;
				a = b;
			}
		} else {
			for (p=pl->start->next; p; p=p->next) {
                b = p->id;
				if (a > b) return false;
				a = b;
			}
		}
	}
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckCoordsSorted (xlist *xl)
{
	long a, b;
	xelem *x;

	if (CheckCoords(xl) && (xl->num > 1)) {
		a = xl->start->id;
		if (xl->compact) {
			for (x=xl->start+1; x <= xl->end; ++x) {
                b = x->id;
				if (a > b) return false;
				a = b;
			}
		} else {
			for (x=xl->start->next; x; x=x->next){
                b = x->id;
				if (a > b) return false;
				a = b;
			}
		}
	}
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckCornerValueSorted (clist *cl)
{
	double a, b;
	celem *c;

	if (CheckCorners(cl) && (cl->num > 1)) {
		a = cl->start->id;
		if (cl->compact) {
			for (c=cl->start+1; c <= cl->end; ++c) {
                b = c->value;
				if (a > b) return false;
				a = b;
			}
		} else {
			for (c=cl->start->next; c; c=c->next){
                b = c->value;
				if (a > b) return false;
				a = b;
			}
		}
	}
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckMeshSorted (mesh *M)
{
	xelem **m, **mstart, **mend;
	long id = -1;

	if (CheckMesh(M)) {
		mstart = M->m[0];
		mend = mstart + M->rows*M->cols;
		for (m = mstart; m < mend; ++m){
			if (*m) {
				if ((*m)->id <= id) {
					return false;
				}
				id = (*m)->id;
			}
		}
	}
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckDoubleGene (gelem *g, int size)
{
	int i, j;

	if (g && (size > 1)) {
		for (i=0; i<size-1; ++i) {
			for (j=i+1; j<size; ++j) {
				if (g->gen[i] == g->gen[j]) {
					return false;
				}
			}
		}
	}
	return true;
}

// ===========================================================================
//
// ===========================================================================

double
CheckOrientation (matrix *P, homo3 *X)
{
	if (!CheckSizeMatrix(P, 3, 4) || !X) return 0.0;

	return (P->m[2][0] * X->X + P->m[2][1] * X->Y + P->m[2][2] * X->Z + P->m[2][3] * X->W);
}

double
CheckOrientation (matrix *P, xelem *X)
{
	if (!CheckSizeMatrix(P, 3, 4) || !X) return 0.0;

	return (P->m[2][0] * X->X + P->m[2][1] * X->Y + P->m[2][2] * X->Z + P->m[2][3]);
}

// ===========================================================================
//
// ===========================================================================

bool CheckID (homo2 *p1, homo2 *p2)
{
	if (!p1 || !p2) return false;

	if (p1->id != p2->id) {
		Warning ("CheckID: ID mismatch!");
		return false;
	}
	return true;
}

bool CheckID (homo2 *p1, pelem *p2)
{
	if (!p1 || !p2) return false;

	if (p1->id != p2->id) {
		Warning ("CheckID: ID mismatch!");
		return false;
	}
	return true;
}

bool CheckID (pelem *p1, pelem *p2)
{
	if (!p1 || !p2) return false;

	if (p1->id != p2->id) {
		Warning ("CheckID: ID mismatch!");
		return false;
	}
	return true;
}

bool CheckID (homo2 *p, homo3 *x)
{
	if (!p || !x) return false;

	if (p->id != x->id) {
		Warning ("CheckID: ID mismatch!");
		return false;
	}
	return true;
}

bool CheckID (homo2 *p, xelem *x)
{
	if (!p || !x) return false;

	if (p->id != x->id) {
		Warning ("CheckID: ID mismatch!");
		return false;
	}
	return true;
}

bool CheckID (pelem *p, homo3 *x)
{
	if (!x || !p) return false;

	if (x->id != p->id) {
		Warning ("CheckID: ID mismatch!");
		return false;
	}
	return true;
}

bool CheckID (pelem *p, xelem *x)
{
	if (!p || !x) return false;

	if (p->id != x->id) {
		Warning ("CheckID: ID mismatch!");
		return false;
	}
	return true;
}

bool CheckID (homo3 *x1, homo3 *x2)
{
	if (!x1 || !x2) return false;

	if (x1->id != x2->id) {
		Warning ("CheckID: ID mismatch!");
		return false;
	}
	return true;
}

bool CheckID (homo3 *x1, xelem *x2)
{
	if (!x1 || !x2) return false;

	if (x1->id != x2->id) {
		Warning ("CheckID: ID mismatch!");
		return false;
	}
	return true;
}

bool CheckID (xelem *x1, xelem *x2)
{
	if (!x1 || !x2) return false;

	if (x1->id != x2->id) {
		Warning ("CheckID: ID mismatch!");
		return false;
	}
	return true;
}

bool CheckID (pelem *p1, homo2 *p2)
{
	return CheckID(p2, p1);
}

bool CheckID (homo3 *x, homo2 *p)
{
	return CheckID(p, x);
}

bool CheckID (xelem *x, homo2 *p)
{
	return CheckID(p, x);
}

bool CheckID (homo3 *x, pelem *p)
{
	return CheckID(p, x);
}

bool CheckID (xelem *x, pelem *p)
{
	return CheckID(p,x);
}

bool CheckID (xelem *x1, homo3 *x2)
{
	return CheckID(x2, x1);
}

bool CheckID (homo2 *h[], int num)
{
	int i;
	long id = 0;

	if (!h || (num < 2)) return false;

	for (i=0; i<num; ++i) { // First ID
		if (h[i]) {
			id = h[i]->id; break;
		}
	}
	for ( ; i<num; ++i) {
		if (h[i] && (h[i]->id != id)) {
			Warning ("CheckID: ID mismatch!");
			return false;
		}
	}
	return true;
}

bool CheckID (homo3 *h[], int num)
{
	int i;
	long id = 0;

	if (!h || (num < 2)) return false;

	for (i=0; i<num; ++i) { // First ID
		if (h[i]) {
			id = h[i]->id; break;
		}
	}
	for ( ; i<num; ++i) {
		if (h[i] && (h[i]->id != id)) {
			Warning ("CheckID: ID mismatch!");
			return false;
		}
	}
	return true;
}

bool CheckID (pelem p[], int num)
{
	int i;
	long id;

	if (!p || (num < 2)) return false;

	id = p[0].id;
	for (i=1; i<num; ++i) {
		if (p[i].id != id) {
			Warning ("CheckID: ID mismatch!");
			return false;
		}
	}
	return true;
}

bool CheckID (pelem *p[], int num)
{
	int i;
	long id = 0;

	if (!p || (num < 2)) return false;

	for (i=0; i<num; ++i) { // First ID
		if (p[i]) {
			id = p[i]->id; break;
		}
	}
	for ( ; i<num; ++i) { // First ID
		if (p[i] && p[i]->id != id) {
			Warning ("CheckID: ID mismatch!");
			return false;
		}
	}
	return true;
}

bool CheckID (xelem x[], int num)
{
	int i;
	long id;

	if (!x || (num < 2)) return false;

	id = x[0].id;
	for (i=1; i<num; ++i) {
		if (x[i].id != id) {
			Warning ("CheckID: ID mismatch!");
			return false;
		}
	}
	return true;
}

bool CheckID (xelem *x[], int num)
{
	int i;
	long id = 0;

	if (!x || (num < 2)) return false;

	for (i=0; i<num; ++i) { // First ID
		if (x[i]) {
			id = x[i]->id; break;
		}
	}
	for ( ; i<num; ++i) { // First ID
		if (x[i] && x[i]->id != id) {
			Warning ("CheckID: ID mismatch!");
			return false;
		}
	}
	return true;
}

// ===========================================================================
//
// ===========================================================================

bool CheckUnifiedID (plist *pl1, plist *pl2)
{
    pelem *p, *q;

	if (CheckPoints(pl1) && CheckPoints(pl2)) {
	    if (pl1->compact && pl2->compact) {
			for (p=pl1->start, q=pl2->start; (p<=pl1->end) && (q<=pl2->end); ++p, ++q) {
				if (p->id != q->id) return false;
			}
			if ((p > pl1->end) && (q > pl2->end)) return true;
		} else if (pl1->compact) {
			for (p=pl1->start, q=pl2->start; (p<=pl1->end) && q; ++p, q=q->next) {
				if (p->id != q->id) return false;
			}
			if ((p > pl1->end) && !q) return true;
		} else if (pl2->compact) {
			for (p=pl1->start, q=pl2->start; p && (q<=pl2->end); p=p->next, ++q) {
				if (p->id != q->id) return false;
			}
			if (!p && (q > pl2->end)) return true;
		} else {
			for (p=pl1->start, q=pl2->start; p && q; p=p->next, q=q->next) {
				if (p->id != q->id) return false;
			}
			if (!(p || q)) return true;
		}
	}
	return false;
}

bool CheckUnifiedID (xlist *xl1, xlist *xl2)
{
    xelem *x, *y;

	if (CheckCoords(xl1) && CheckCoords(xl2)) {
		if (xl1->compact && xl2->compact) {
			for (x=xl1->start, y=xl2->start; (x<=xl1->end) && (y<=xl2->end); ++x, ++y) {
				if (x->id != y->id) return false;
			}
			if ((x > xl1->end) && (y > xl2->end)) return true;
		} else if (xl1->compact) {
			for (x=xl1->start, y=xl2->start; (x<=xl1->end) && y; ++x, y=y->next) {
				if (x->id != y->id) return false;
			}
			if ((x > xl1->end) && !y) return true;
		} else if (xl2->compact) {
			for (x=xl1->start, y=xl2->start; x && (y<=xl2->end); x=x->next, ++y) {
				if (x->id != y->id) return false;
			}
			if (!x && (y > xl2->end)) return true;
		} else {
			for (x=xl1->start, y=xl2->start; x && y; x=x->next, y=y->next) {
				if (x->id != y->id) return false;
			}
			if (!(x || y)) return true;
		}
	}
	return false;
}

bool CheckUnifiedID (xlist *xl, plist *pl)
{
    xelem *x;
    pelem *p;

	if (CheckCoords(xl) && CheckPoints(pl)) {
		if (xl->compact && pl->compact) {
			for (x=xl->start, p=pl->start; (x<=xl->end) && (p<=pl->end); ++x, ++p) {
				if (x->id != p->id) return false;
			}
			if ((x > xl->end) && (p > pl->end)) return true;
		} else if (xl->compact) {
			for (x=xl->start, p=pl->start; (x<=xl->end) && p; ++x, p=p->next) {
				if (x->id != p->id) return false;
			}
			if ((x > xl->end) && !p) return true;
		} else if (pl->compact) {
			for (x=xl->start, p=pl->start; x && (p<=pl->end); x=x->next, ++p) {
				if (x->id != p->id) return false;
			}
			if (!x && (p > pl->end)) return true;
		} else {
			for (x=xl->start, p=pl->start; x && p; x=x->next, p=p->next) {
				if (x->id != p->id) return false;
			}
			if (!(x || p)) return true;
		}
	}
	return false;
}

bool CheckUnifiedID (plist *pl, xlist *xl)
{
	return CheckUnifiedID (xl, pl);
}

// ===========================================================================
//
// ===========================================================================
//not enabled for linux
bool CheckFile(char *file, filePermission perm){
    //check if given file fullfills the requred permission in perm
    //0 = existance
    //2 = write
    //4 = read
    //6 = read/write
#ifndef linux
/*
    FILE *fp=NULL;
    if(perm==2 &&_access( file,0)!=0){ //file for writing does not exist! check if we can create it!
        fp= fopen(file, "w");
        if(fp){
            fclose(fp);
            remove(file);
            return true;
        }
        else{
            return false;
        }

    }
	return _access( file, perm ) ==0;
*/
#endif
#ifdef linux
	return 1;
#endif

}


//coord

static xlist *InsertCoord (xlist *list, xelem *m)
{
    if (CheckCoords(m)) {
        if (list) {
            CoordList(list); // Convert compact array to linked list

            if (list->end) {
                list->end->next = m;
            } else {
                list->start = m;
            }
            list->end = m;
            list->num++;
        } else {
            list = NewCoordList();
            if (list) {
                list->start = list->end = m;
                list->num = 1;
            }
        }
    }
    return list;
}

/**
 Append a coordinate to the end of a coordinate list.

 @param[in]		list	Coordinate list or NULL.
 @param[in]		x, y, z Coordinates.
 @param[in]		id		Identification number.
 @param[in]		color	RGB coordinate color.
 @param[in]		quality	Estimated coordinate reliability.
 @result				Extended coordinate list.
 @note					Compact array will be converted to a linked list!
*/
xlist *AddCoord (xlist *list, double x, double y, double z, long id, int color, double quality)
{
    xelem *m;

    m = NewCoord ();
    if (m) {
        m->X       = x;
        m->Y       = y;
        m->Z       = z;
        m->id      = id;
        m->color   = color;
        m->quality = quality;
        m->next    = NULL;
    }
    return (InsertCoord (list, m));
}

/**
 Append a coordinate with quality 1.0 to the end of a coordinate list.

 @param[in]		list	Coordinate list or NULL.
 @param[in]		x, y, z Coordinates.
 @param[in]		id		Identification number.
 @param[in]		color	RGB coordinate color.
 @result				Extended coordinate list.
 @note					Compact array will be converted to a linked list!
*/
xlist *AddCoord (xlist *list, double x, double y, double z, long id, int color)
{
    return AddCoord (list, x, y, z, id, color, 1.0);
}

/**
 Append a white coordinate to the end of a coordinate list.

 @param[in]		list	Coordinate list or NULL.
 @param[in]		x, y, z Coordinates.
 @param[in]		id		Identification number.
 @param[in]		quality	Estimated coordinate reliability.
 @result				Extended coordinate list.
 @note					Compact array will be converted to a linked list!
*/
xlist *AddCoord (xlist *list, double x, double y, double z, long id, double quality)
{
    return AddCoord (list, x, y, z, id, WHITE_COLOR, quality);
}

/**
 Append a white coordinate with quality 1.0 to the end of a coordinate list.

 @param[in]		list	Coordinate list or NULL.
 @param[in]		x, y, z Coordinates.
 @param[in]		id		Identification number.
 @result				Extended coordinate list.
 @note					Compact array will be converted to a linked list!
*/
xlist *AddCoord (xlist *list, double x, double y, double z, long id)
{
    return AddCoord (list, x, y, z, id, WHITE_COLOR, 1.0);
}

/**
 Append a homogeneous coordinate to the end of a coordinate list.

 @param[in]		list	Coordinate list or NULL.
 @param[in]		X		Homogeneous coordinate with identification number.
 @param[in]		color	RGB coordinate color.
 @param[in]		quality	Estimated coordinate reliability.
 @result				Extended coordinate list.
 @note					Compact array will be converted to a linked list!
*/
xlist *AddCoord (xlist *list, homo3 *X, int color, double quality)
{
	if (X) {
		NormHomo(X);
		return AddCoord (list, X->X, X->Y, X->Z, X->id, color, quality);
	}
	return list;
}

/**
 Append a homogeneous coordinate with quality 1.0 to the end of a coordinate list.

 @param[in]		list	Coordinate list or NULL.
 @param[in]		X		Homogeneous coordinate with identification number.
 @param[in]		color	RGB coordinate color.
 @result				Extended coordinate list.
 @note					Compact array will be converted to a linked list!
*/
xlist *AddCoord (xlist *list, homo3 *X, int color)
{
    return AddCoord (list, X, color, 1.0);
}

/**
 Append a white homogeneous coordinate to the end of a coordinate list.

 @param[in]		list	Coordinate list or NULL.
 @param[in]		X		Homogeneous coordinate with identification number.
 @param[in]		quality	Estimated coordinate reliability.
 @result				Extended coordinate list.
 @note					Compact array will be converted to a linked list!
*/
xlist *AddCoord (xlist *list, homo3 *X, double quality)
{
    return AddCoord (list, X, WHITE_COLOR, quality);
}

/**
 Append a white homogeneous coordinate with quality 1.0 to the end of a coordinate list.

 @param[in]		list	Coordinate list or NULL.
 @param[in]		X		Homogeneous coordinate with identification number.
 @result				Extended coordinate list.
 @note					Compact array will be converted to a linked list!
*/
xlist *AddCoord (xlist *list, homo3 *X)
{
    return AddCoord (list, X, WHITE_COLOR, 1.0);
}

/**
 Append a coordinate element to the end of a coordinate list.

 @param[in]		list	Coordinate list or NULL.
 @param[in]		X		Coordinate element with identification number, color and quality.
 @result				Extended coordinate list.
 @note					Compact array will be converted to a linked list!
*/
xlist *AddCoord (xlist *list, xelem *X)
{
    return (InsertCoord (list, CloneCoord(X)));
}

/**
 Duplicate a coordinate element.

 @param[in]		X		Coordinate element with identification number, color and quality.
 @result				New allocated coordinate with duplicated elements and no successor or NULL.
*/
xelem *CloneCoord (xelem *X)
{
    xelem *Y = NULL;

    if (CheckCoords(X)) {
        Y = NewCoord ();
        if (Y) {
            memcpy(Y, X, sizeof(xelem));
            Y->next = NULL;
        }
    }
    return Y;
}

/**
 Duplicate to a coordinate array.

 @param[in]		xl		Coordinate list.
 @result				New allocated compact coordinate array with duplicated elements or NULL.
*/
xlist *CloneCoordArray (xlist *xl)
{
    xlist *list = NULL;

    if (CheckCoords(xl)) {
        list = NewCoordList(xl->num);
        CopyCoord(xl, list);
    }
    return list;
}

/**
 Duplicate to a coordinate list.

 @param[in]		xl		Coordinate list.
 @result				New allocated linked coordinate list with duplicated elements or NULL.
*/
xlist *CloneCoordList (xlist *xl)
{
    xlist *list = NULL;
    xelem *x;

    if (CheckCoords(xl)) {
        for (x = xl->start; x; x = x->next) {
            list = AddCoord(list, x);
        }
    }
    return list;
}

/**
 Duplicate a coordinate list or array.

 @param[in]		xl		Coordinate list.
 @result				New allocated linked coordinate list with duplicated elements or NULL.
*/
xlist *CloneCoord (xlist *xl)
{
    if (CheckCoords(xl)) {
		if (xl->compact) {
			return CloneCoordArray(xl);
		} else {
			return CloneCoordList(xl);
		}
	}
	return NULL;
}

/**
 Partial duplicate to a coordinate array.

 @param[in]		xl		Coordinate list.
 @param[in]		s		Start index.
 @param[in]		e		End index.
 @result				New allocated compact coordinate array with \f$ e-s+1 \f$ duplicated elements or NULL.
*/
xlist *CloneCoordArray (xlist *xl, int s, int e)
{
    xlist *list = NULL;

    if (CheckIndex(xl, s) && CheckIndex(xl, e)) {
        if (s > e) {
            SWAP(int, s, e);
        }
        list = NewCoordList(e-s+1);
        CopyCoord(xl, s, e, list);
    }
    return list;
}

/**
 Partial duplicate to a coordinate list.

 @param[in]		xl		Coordinate list.
 @param[in]		s		Start index.
 @param[in]		e		End index.
 @result				New allocated linked coordinate list with \f$ e-s+1 \f$ duplicated elements or NULL.
*/
xlist *CloneCoordList (xlist *xl, int s, int e)
{
    xlist *list = NULL;
    xelem *x;
    long i, num;

    if (CheckIndex(xl, s) && CheckIndex(xl, e)) {
        if (s > e) {
            SWAP(int, s, e);
        }
        num = e-s+1;
        x = GetCoordPos(xl, s);
        if (x) {
            if (xl->compact) {
                for (i=0; (x<=xl->end) && (i<num); ++i, ++x) {
                    list = AddCoord(list, x);
                }
            } else {
                for (i=0; x && (i<num); ++i, x=x->next) {
                    list = AddCoord(list, x);
                }
            }
        }
    }
    return list;
}

/**
 Partial duplicate of a coordinate list or array.

 @param[in]		xl		Coordinate list.
 @param[in]		s		Start index.
 @param[in]		e		End index.
 @result				New allocated linked coordinate list with \f$ e-s+1 \f$ duplicated elements or NULL.
*/
xlist *CloneCoord (xlist *xl, int s, int e)
{
    if (CheckCoords(xl)) {
		if (xl->compact) {
			return CloneCoordArray(xl, s, e);
		} else {
			return CloneCoordList(xl, s, e);
		}
	}
	return NULL;
}

/**
 Copy a coordinate element.

 @param[in]		X		Coordinate element with identification number, color and quality.
 @param[out]	Y		Overwritten coordinate with duplicated elements and old successor.
*/
void CopyCoord (xelem *X, xelem *Y)
{
    xelem *next;

    if (CheckCoords(X) && CheckCoords(Y)) {
        next = Y->next;
        memcpy(Y, X, sizeof(xelem)); // keep old pointer
        Y->next = next;
    }
}

/**
 Partial copy of a coordinate list.

 @param[in]		xl		Coordinate list with \a n elements.
 @param[out]	yl		Overwritten coordinate list with \f$ \min(n, num) \f$ copied elements and old successors.
*/
void CopyCoord (xlist *xl, xlist *yl)
{
    xelem *x, *y;

    if (CheckCoords(xl) && CheckCoords(yl)) {
        if (xl->num > yl->num) {
            Warning("CopyCoord: skipped %d elements!", xl->num - yl->num);
        }
        if (xl->compact && yl->compact) {
            for (x=xl->start, y=yl->start; (x<=xl->end) && (y<=yl->end); ++x, ++y) {
                memcpy(y, x, sizeof(xelem));
                y->next = y+1;
            }
            yl->end->next = NULL;
        } else if (xl->compact) {
            for (x=xl->start, y=yl->start; (x<=xl->end) && y; ++x, y=y->next) {
                CopyCoord(x, y);
            }
        } else if (yl->compact) {
            for (x=xl->start, y=yl->start; x && (y<=yl->end); x=x->next, ++y) {
                memcpy(y, x, sizeof(xelem));
                y->next = y+1;
            }
            yl->end->next = NULL;
        } else {
            for (x=xl->start, y=yl->start; x && y; x=x->next, y=y->next) {
                CopyCoord(x, y);
            }
        }
    }
}

/**
 Partial copy of a coordinate list.

 @param[in]		xl		Coordinate list with \a n elements.
 @param[in]		s		Start index.
 @param[in]		e		End index.
 @param[out]	yl		Overwritten coordinate list with \f$ e-s+1 \f$ copied elements and old successors.
*/
void CopyCoord (xlist *xl, int s, int e, xlist *yl)
{
    xelem *x, *y;
    long i, num;

    if (CheckIndex(xl, s) && CheckIndex(xl, e) && CheckCoords(yl)) {
        if (s > e) {
            SWAP(int, s, e);
        }
        num = e-s+1;
        if (num > yl->num) {
            Warning("CopyCoord: skipped %d elements!", num - yl->num);
        }
        x = GetCoordPos(xl, s);
        if (x) {
            if (xl->compact && yl->compact) {
                for (y=yl->start, i=0; (x<=xl->end) && (y<=yl->end) && (i < num); ++i, ++x, ++y) {
                    memcpy(y, x, sizeof(xelem));
                    y->next = y+1;
                }
                yl->end->next = NULL;
            } else if (xl->compact) {
                for (y=yl->start, i=0; (x<=xl->end) && y && (i < num); ++i, ++x, y=y->next) {
                    CopyCoord(x, y);
                }
            } else if (yl->compact) {
                for (y=yl->start, i=0; x && (y<=yl->end) && (i < num); ++i, x=x->next, ++y) {
                    memcpy(y, x, sizeof(xelem));
                    y->next = y+1;
                }
                yl->end->next = NULL;
            } else {
                for (y=yl->start, i=0; x && y && (i < num); ++i, x=x->next, y=y->next) {
                    CopyCoord(x, y);
                }
            }
        }
    }
}

/**
 Partial copy of a coordinate list.

 @param[in]		xl		Coordinate list with \a n elements.
 @param[out]	yl		Overwritten coordinate list with \f$ \min(n, num) \f$ copied elements and old successors.
 @param[in]		d		Destination start index.
*/
void CopyCoord (xlist *xl, xlist *yl, int d)
{
    xelem *x, *y;
	long i;

    if (CheckCoords(xl) && CheckIndex(yl, d)) {
        if (xl->num > yl->num - d) {
            Warning("CopyCoord: skipped %d elements!", xl->num - yl->num + d);
        }
        if (xl->compact && yl->compact) {
            for (x=xl->start, y=yl->start+d; (x<=xl->end) && (y<=yl->end); ++x, ++y) {
                memcpy(y, x, sizeof(xelem));
                y->next = y+1;
            }
            yl->end->next = NULL;
        } else if (xl->compact) {
			for (y=yl->start, i=0; (i < d) && y; ++i, y=y->next); // Skip d elements
            for (x=xl->start; (x<=xl->end) && y; ++x, y=y->next) {
                CopyCoord(x, y);
            }
        } else if (yl->compact) {
            for (x=xl->start, y=yl->start+d; x && (y<=yl->end); x=x->next, ++y) {
                memcpy(y, x, sizeof(xelem));
                y->next = y+1;
            }
            yl->end->next = NULL;
        } else {
			for (y=yl->start, i=0; (i < d) && y; ++i, y=y->next); // Skip d elements
            for (x=xl->start; x && y; x=x->next, y=y->next) {
                CopyCoord(x, y);
            }
        }
    }
}

/**
 Partial copy of a coordinate list.

 @param[in]		xl		Coordinate list with \a n elements.
 @param[in]		s		Start index.
 @param[in]		e		End index.
 @param[out]	yl		Overwritten coordinate list with \f$ e-s+1 \f$ copied elements and old successors.
 @param[in]		d		Destination start index.
*/
void CopyCoord (xlist *xl, int s, int e, xlist *yl, int d)
{
    xelem *x, *y;
    long i, num;

    if (CheckIndex(xl, s) && CheckIndex(xl, e) && CheckIndex(yl, d)) {
        if (s > e) {
            SWAP(int, s, e);
        }
        num = e-s+1;
        if (num > yl->num-d) {
            Warning("CopyCoord: skipped %d elements!", num - yl->num + d);
        }
        x = GetCoordPos(xl, s);
        if (x) {
            if (xl->compact && yl->compact) {
                for (y=yl->start, i=0; (x<=xl->end) && (y<=yl->end) && (i < num); ++i, ++x, ++y) {
                    memcpy(y, x, sizeof(xelem));
                    y->next = y+1;
                }
                yl->end->next = NULL;
            } else if (xl->compact) {
				for (y=yl->start, i=0; (i < d) && y; ++i, y=y->next); // Skip d elements
                for (i=0; (x<=xl->end) && y && (i < num); ++i, ++x, y=y->next) {
                    CopyCoord(x, y);
                }
            } else if (yl->compact) {
                for (y=yl->start, i=0; x && (y<=yl->end) && (i < num); ++i, x=x->next, ++y) {
                    memcpy(y, x, sizeof(xelem));
                    y->next = y+1;
                }
                yl->end->next = NULL;
            } else {
				for (y=yl->start, i=0; (i < d) && y; ++i, y=y->next); // Skip d elements
                for (i=0; x && y && (i < num); ++i, x=x->next, y=y->next) {
                    CopyCoord(x, y);
                }
            }
        }
    }
}

/**
 Delete a specified coordinate element from the list.

 @param[in,out]		xl		Modified coordinate list.
 @param[in]			X		coordinate element.
*/
void DeleteCoord (xlist *xl, xelem *X)
{
    xelem *prev=NULL, *act;

    if (CheckCoords(xl) && CheckCoords(X)) {
        CoordList(xl);     // Convert compact array to linked list
        for (act=xl->start; act; ) {
            if (act == X) {
                if (!act->next) {            // Last element
                    xl->end = prev;
                }
                if (prev){
                    prev->next = act->next;
                    hfree(act); act = NULL;
                } else {
                    xl->start = act->next;
                    hfree(act); act = NULL;
                }
                xl->num -= 1;
                return;
            } else {
                prev = act;
                act = act->next;
            }
        }
        Warning("DeleteCoord: Coordinate element not found in list!");
    }
}

/**
 Delete only the first coordinates with the specified id.

 @param[in,out]		xl		Modified coordinate list.
 @param[in]			id		Identification number.
*/
void DeleteCoordID (xlist *xl, long id)
{
    xelem *prev=NULL, *act;

    if (CheckCoords(xl)) {
        CoordList(xl);     // Convert compact array to linked list
        for (act=xl->start; act; ) {
            if (act->id == id) {
                if (!act->next) {            // Last element
                    xl->end = prev;
                }
                if (prev) {
                    prev->next = act->next;
                    hfree(act);
                    act = prev->next;
                } else {
                    xl->start = act->next;
                    hfree(act);
                    act = xl->start;
                }
                xl->num -= 1;
                return;               // Delete only the first ID
            } else {
                prev = act;
                act = act->next;
            }
        }
        Warning("DeleteCoordID: Coordinate element with ID %d not found in list!", id);
    }
}

/**
 Delete all coordinates with the specified id.

 @param[in,out]		xl		Modified coordinate list.
 @param[in]			id		Identification number.
*/
void DeleteCoordIDs (xlist *xl, long id)
{
    xelem *prev=NULL, *act;

    if (CheckCoords(xl)) {
        CoordList(xl);     // Convert compact array to linked list
        for (act=xl->start; act; ) {
            if (act->id == id) {
                if (!act->next) {            // Last element
                    xl->end = prev;
                }
                if (prev) {
                    prev->next = act->next;
                    hfree(act);
                    act = prev->next;
                } else {
                    xl->start = act->next;
                    hfree(act);
                    act = xl->start;
                }
                xl->num -= 1;
            } else {
                prev = act;
                act = act->next;
            }
        }
    }
}

/**
 Delete coordinate at the specified position.

 @param[in,out]		xl		Modified coordinate list.
 @param[in]			pos		Postion in the list.
*/
void DeleteCoordPos (xlist *xl, long pos)
{
    xelem *prev=NULL, *act;
    long i;

    if (CheckCoords(xl)) {
        CoordList(xl);     // Convert compact array to linked list
        if ((pos >= 0) && (pos < xl->num)) {
            for (i=0, act=xl->start; act; ++i) {
                if (i == pos) {
                    if (!act->next) {            // Last element
                        xl->end = prev;
                    }
                    if (prev){
                        prev->next = act->next;
                        hfree(act); act = NULL;
                    } else {
                        xl->start = act->next;
                        hfree(act); act = NULL;
                    }
                    xl->num -= 1;
                    return;
                } else {
                    prev = act;
                    act = act->next;
                }
            }
        }
//      Warning("DeleteCoordPos: Coordinate element not found at position %d in list!", pos);
    }
}

/**
 Free memory of a coordinate list.

 @param[in]			list	Coordinate list.
 @result					NULL.
*/
xlist *FreeCoord (xlist *list)
{
    xelem *next, *p;

    if (list) {
        if (list->compact) {
			if (list->start) {
				hfree (list->start); list->start = NULL;
			}
        } else {
            for (p = list->start; p; p = next) {
                next = p->next;
                hfree (p); p = NULL;
            }
        }
        hfree (list); list = NULL;
    }
    return NULL;
}

/**
 Get coordinate with the specified identification number.

 Binary search for sorted ID numbers in compact array

   BinarySearch(A[0..N-1], value) {
       low = 0
       high = N - 1
       while (low <= high) {
           mid = (low + high) / 2
           if (A[mid] > value)
               high = mid - 1
           else if (A[mid] < value)
               low = mid + 1
           else
               return mid // found
       }
       return -1 // not found
   }

 @param[in]			xl		Coordinate list.
 @param[in]			id		Identification number.
 @result					First coordinate element with the specified ID or NULL.
*/
xelem *GetCoordID (xlist *xl, long id)
{
    long low = 0, mid, high;
    xelem *x;

    if (CheckCoords(xl)) {
        if (xl->compact) {
            high = xl->num - 1;
            while (low <= high) {
                mid = (low + high)/2;
                x = xl->start + mid;
                if (x->id > id) {
                    high = mid-1;
                } else if (x->id < id) {
                    low = mid+1;
                } else {
                    return x;
                }
            }
        } else {
            for (x=xl->start; x; x=x->next) {
                if (x->id == id) {
                    return x;
                }
            }
        }
    }
    Warning ("GetCoordID: illegal ID %d!", id);

    return NULL;
}

/**
 Get coordinate at the specified position.

 @param[in]			xl		Coordinate list.
 @param[in]			pos		Postion in the list.
 @result					Coordinate element at the specified position or NULL.
*/
xelem *GetCoordPos (xlist *xl, long pos)
{
    xelem *x;
    long i;

    if (CheckIndex(xl, pos)) {
        if (xl->compact) {
            return xl->start + pos;
        } else {
            for (x=xl->start, i=0; x && (i < pos); ++i, x=x->next);
            return x;
        }
    }
    return NULL;
}

/**
 Allocate memory for a new coordinate element.

 @result					New allocated coordinate element or NULL.
*/
xelem *NewCoord (void)
{
    return (xelem *) halloc (1, sizeof (xelem));
}

/**
 Allocate memory for n new coordinate elements.

 @param[in]			num		Number of coordinate elements.
 @result					New allocated coordinate element or NULL.
*/
xelem *NewCoord (int num)
{
    return (xelem *) halloc (num, sizeof (xelem));
}

/**
 Allocate memory for a new coordinate list.

 @result					New allocated empty coordinate list or NULL.
*/
xlist *NewCoordList (void)
{
    xlist *list = NULL;

    list = (xlist *) halloc (1, sizeof (xlist));
    if (list) {
        list->start = list->end = NULL;
        list->num = 0;
        list->compact = false;
    }
    return list;
}

/**
 Allocate memory for a new compact coordinate array.

 @param[in]			num		Number of coordinate elements.
 @result					New allocated coordinate array with num elements or NULL.
 @note						Coordinate elements are not linked!
*/
xlist *NewCoordList (int num)
{
    xlist *list = NULL;
    xelem *start, *end;

    list = (xlist *) halloc (1, sizeof (xlist));
    if (list) {
        start = NewCoord (num);
        if (start) {
            end = start + num-1;
            list->start   = start;
            list->num     = num;
            list->end     = end;
            list->compact = true;
        } else {
            hfree(list); list = NULL;
        }
    }
    return list;
}

/**
 Swap the contents of two different coordinate lists.

 @param[in,out]		xl1		Modified first coordinate list with \a m elements.
 @param[in,out]		xl2		Modified second coordinate list with \a n elements.
*/
void SwapCoord(xlist *xl1, xlist *xl2)
{
    if (CheckCoords(xl1) && CheckCoords(xl2)) {
        SWAP(long, xl1->num, xl2->num);
        SWAP(bool, xl1->compact, xl2->compact);
        SWAP(xelem *, xl1->start, xl2->start);
        SWAP(xelem *, xl1->end, xl2->end);
    }
}

//@}


/** @name Arithmetic Operations with Scalars */
//@{

/**
 Add scalar to Euclidean coordinate.

 @param[in]			h	    Euclidean coordinate.
 @param[in]			s	    Scalar.
 @result					Added coordinate or NOHOMO3.
*/
homo3 CoordAdd (homo3 *h, double s)
{
    homo3 r;

    if (!h) {
        Warning ("CoordAdd failed!"); return NOHOMO3;
    }
    NormHomo(h);
    if (ISZERO(s)) {
        r.X = h->X;
        r.Y = h->Y;
        r.Z = h->Z;
    } else {
        r.X = h->X + s;
        r.Y = h->Y + s;
        r.Z = h->Z + s;
    }
    r.W = 1.0;
    r.id = h->id;

    return r;
}

/**
 Add scalar to Euclidean coordinate.

 @param[in]			x	    Euclidean coordinate.
 @param[in]			s	    Scalar.
 @result					Added coordinate or NOHOMO3.
*/
homo3 CoordAdd (xelem *x, double s)
{
    homo3 r;

    if (!x) {
        Warning ("CoordAdd failed!"); return NOHOMO3;
    }
    if (ISZERO(s)) {
        r.X = x->X;
        r.Y = x->Y;
        r.Z = x->Z;
    } else {
        r.X = x->X + s;
        r.Y = x->Y + s;
        r.Z = x->Z + s;
    }
    r.W = 1.0;
    r.id = x->id;

    return r;
}

/**
 Add scalar to Euclidean coordinate list.

 @param[in,out]		xl		Modified Euclidean coordinate list.
 @param[in]			s		Scalar.
*/
void CoordAdd (xlist *xl, double s)
{
    xelem *x;

    if (CheckCoords(xl) && ISNONZERO(s)) {
        if (xl->compact) {
            for (x=xl->start; x<=xl->end; ++x) {
                x->X += s;
                x->Y += s;
                x->Z += s;
            }
        } else {
            for (x=xl->start; x; x=x->next) {
                x->X += s;
                x->Y += s;
                x->Z += s;
            }
        }
    }
}

/**
 Add scalar to Euclidean coordinate list.

 @param[in]		xl1		Euclidean coordinate list.
 @param[in]		s		Scalar.
 @param[out]	xl2		Overwritten Euclidean coordinate list with added scalar.
*/
void CoordAdd (xlist *xl1, double s, xlist *xl2)
{
    xelem *x, *y;

    if (CheckSameCoords(xl1,xl2)) {
        if (ISZERO(s)) {
            CopyCoord(xl1, xl2);
        } else {
            if (xl1->compact && xl2->compact) {
                for (x=xl1->start, y=xl2->start; (x<=xl1->end) && (y<=xl2->end); ++x, ++y) {
                    y->X = x->X + s;
                    y->Y = x->Y + s;
                    y->Z = x->Z + s;
                    y->id = x->id;
                }
            } else if (xl1->compact) {
                for (x=xl1->start, y=xl2->start; (x<=xl1->end) && y; ++x, y=y->next) {
                    y->X = x->X + s;
                    y->Y = x->Y + s;
                    y->Z = x->Z + s;
                    y->id = x->id;
                }
            } else if (xl2->compact) {
                for (x=xl1->start, y=xl2->start; x && (y<=xl2->end); x=x->next, ++y) {
                    y->X = x->X + s;
                    y->Y = x->Y + s;
                    y->Z = x->Z + s;
                    y->id = x->id;
                }
            } else {
                for (x=xl1->start, y=xl2->start; x && y; x=x->next, y=y->next) {
                    y->X = x->X + s;
                    y->Y = x->Y + s;
                    y->Z = x->Z + s;
                    y->id = x->id;
                }
            }
        }
    }
}

/**
 Add scalar to Euclidean coordinate lists.

 @param[in]		xl		First Euclidean coordinate list.
 @param[in]		s		Scalar.
 @result	    		New allocated Euclidean coordinate array with added scalar.
*/
xlist *CoordAddNew (xlist *xl, double s)
{
    xlist *list = NULL;

    if (CheckCoords(xl)) {
        list = NewCoordList(xl->num);
        CoordAdd(xl, s, list);
        RelinkCoord(list);
    }
    return list;
}

/**
 Divide Euclidean coordinate by a scalar.

 @param[in]			h	    Euclidean coordinate.
 @param[in]			s	    Scalar.
 @result					Multiplied coordinate or NOHOMO3.
*/
homo3 CoordDiv (homo3 *h, double s)
{
    homo3 r;

    if (!h) {
        Warning ("CoordDiv failed!"); return NOHOMO3;
    }
    if (ISZERO(s)) {
        Warning ("CoordDiv: division by zero!");
    }
    NormHomo(h);
    r.X = h->X;
    r.Y = h->Y;
    r.Z = h->Z;
    r.W = s;
    r.id = h->id;

    return r;
}

/**
 Divide Euclidean coordinate by a scalar.

 @param[in]			x	    Euclidean coordinate.
 @param[in]			s	    Scalar.
 @result					Divided coordinate or NOHOMO3.
*/
homo3 CoordDiv (xelem *x, double s)
{
    homo3 r;

    if (!x) {
        Warning ("CoordDiv failed!"); return NOHOMO3;
    }
    if (ISZERO(s)) {
        Warning ("CoordDiv: division by zero!");
    }
    r.X = x->X;
    r.Y = x->Y;
    r.Z = x->Z;
    r.W = s;
    r.id = x->id;

    return r;
}

/**
 Divide Euclidean coordinate list by a scalar.

 @param[in,out]		xl		Modified Euclidean coordinate list.
 @param[in]			s		Scalar.
*/
void CoordDiv (xlist *xl, double s)
{
    xelem *x;

    if (ISZERO(s)) {
        Warning ("CoordDiv: division by zero!"); return;
    }
    if (CheckCoords(xl) && ISINEQUAL(s, 1.0)) {
        if (xl->compact) {
            for (x=xl->start; x<=xl->end; ++x) {
                x->X /= s;
                x->Y /= s;
                x->Z /= s;
            }
        } else {
            for (x=xl->start; x; x=x->next) {
                x->X /= s;
                x->Y /= s;
                x->Z /= s;
            }
        }
    }
}

/**
 Divide Euclidean coordinate list by a scalar.

 @param[in]		xl1		Euclidean coordinate list.
 @param[in]		s		Scalar.
 @param[out]	xl2		Overwritten Euclidean coordinate list divided by a scalar.
*/
void CoordDiv (xlist *xl1, double s, xlist *xl2)
{
    xelem *x, *y;

    if (ISZERO(s)) {
        Warning ("CoordDiv: division by zero!"); return;
    } else if (ISEQUAL(s, 1.0)) {
        CopyCoord(xl1, xl2);
    } else if (CheckSameCoords(xl1,xl2)) {
        if (xl1->compact && xl2->compact) {
            for (x=xl1->start, y=xl2->start; (x<=xl1->end) && (y<=xl2->end); ++x, ++y) {
                y->X = x->X / s;
                y->Y = x->Y / s;
                y->Z = x->Z / s;
                y->id = x->id;
            }
        } else if (xl1->compact) {
            for (x=xl1->start, y=xl2->start; (x<=xl1->end) && y; ++x, y=y->next) {
                y->X = x->X / s;
                y->Y = x->Y / s;
                y->Z = x->Z / s;
                y->id = x->id;
            }
        } else if (xl2->compact) {
            for (x=xl1->start, y=xl2->start; x && (y<=xl2->end); x=x->next, ++y) {
                y->X = x->X / s;
                y->Y = x->Y / s;
                y->Z = x->Z / s;
                y->id = x->id;
            }
        } else {
            for (x=xl1->start, y=xl2->start; x && y; x=x->next, y=y->next) {
                y->X = x->X / s;
                y->Y = x->Y / s;
                y->Z = x->Z / s;
                y->id = x->id;
            }
        }
    }
}

/**
 Divide Euclidean coordinate list by a scalar.

 @param[in]		xl		Euclidean coordinate list.
 @param[in]		s		Scalar.
 @result	    		New allocated Euclidean coordinate array divided by a scalar.
*/
xlist *CoordDivNew (xlist *xl, double s)
{
    xlist *list = NULL;

    if (CheckCoords(xl)) {
        list = NewCoordList(xl->num);
        CoordDiv(xl, s, list);
        RelinkCoord(list);
    }
    return list;
}

/**
 Multiply scalar to Euclidean coordinate.

 @param[in]			h	    Euclidean coordinate.
 @param[in]			s	    Scalar.
 @result					Multiplied coordinate or NOHOMO3.
*/
homo3 CoordMult (homo3 *h, double s)
{
    homo3 r;

    if (!h) {
        Warning ("CoordMult failed!"); return NOHOMO3;
    }
    NormHomo(h);
    if (ISZERO(s)) {
        r.X = r.Y = r.Z = 0.0;
    } else if (ISEQUAL(s, 1.0)) {
        r.X = h->X;
        r.Y = h->Y;
        r.Z = h->Z;
    } else {
        r.X = h->X * s;
        r.Y = h->Y * s;
        r.Z = h->Z * s;
    }
    r.W = 1.0;
    r.id = h->id;

    return r;
}

/**
 Multiply scalar to Euclidean coordinate.

 @param[in]			x	    Euclidean coordinate.
 @param[in]			s	    Scalar.
 @result					Multiplied coordinate or NOHOMO3.
*/
homo3 CoordMult (xelem *x, double s)
{
    homo3 r;

    if (!x) {
        Warning ("CoordMult failed!"); return NOHOMO3;
    }
    if (ISZERO(s)) {
        r.X = r.Y = r.Z = 0.0;
    } else if (ISEQUAL(s, 1.0)) {
        r.X = x->X;
        r.Y = x->Y;
        r.Z = x->Z;
    } else {
        r.X = x->X * s;
        r.Y = x->Y * s;
        r.Z = x->Z * s;
    }
    r.W = 1.0;
    r.id = x->id;

    return r;
}

/**
 Multiply scalar to Euclidean coordinate list.

 @param[in,out]		xl		Modified Euclidean coordinate list.
 @param[in]			s		Scalar.
*/
void CoordMult (xlist *xl, double s)
{
    xelem *x;

    if (CheckCoords(xl) && ISINEQUAL(s, 1.0)) {
        if (ISZERO(s)) {
            if (xl->compact) {
                for (x=xl->start; x<=xl->end; ++x) {
                    x->X = x->Y = x->Z = 0.0;
                }
            } else {
                for (x=xl->start; x; x=x->next) {
                    x->X = x->Y = x->Z = 0.0;
                }
            }
        } else {
            if (xl->compact) {
                for (x=xl->start; x<=xl->end; ++x) {
                    x->X *= s;
                    x->Y *= s;
                    x->Z *= s;
                }
            } else {
                for (x=xl->start; x; x=x->next) {
                    x->X *= s;
                    x->Y *= s;
                    x->Z *= s;
                }
            }
        }
    }
}

/**
 Multiply scalar to Euclidean coordinate list.

 @param[in]		xl1		Euclidean coordinate list.
 @param[in]		s		Scalar.
 @param[out]	xl2		Overwritten Euclidean coordinate list with multiplied scalar.
*/
void CoordMult (xlist *xl1, double s, xlist *xl2)
{
    xelem *x, *y;

    if (ISEQUAL(s, 1.0)) {
        CopyCoord(xl1, xl2);
    } else if (CheckSameCoords(xl1,xl2)) {
        if (ISZERO(s)) {
            if (xl1->compact && xl2->compact) {
                for (x=xl1->start, y=xl2->start; (x<=xl1->end) && (y<=xl2->end); ++x, ++y) {
                    y->X = y->Y = y->Z = 0.0;
                    y->id = x->id;
                }
            } else if (xl1->compact) {
                for (x=xl1->start, y=xl2->start; (x<=xl1->end) && y; ++x, y=y->next) {
                    y->X = y->Y = y->Z = 0.0;
                    y->id = x->id;
                }
            } else if (xl2->compact) {
                for (x=xl1->start, y=xl2->start; x && (y<=xl2->end); x=x->next, ++y) {
                    y->X = y->Y = y->Z = 0.0;
                    y->id = x->id;
                }
            } else {
                for (x=xl1->start, y=xl2->start; x && y; x=x->next, y=y->next) {
                    y->X = y->Y = y->Z = 0.0;
                    y->id = x->id;
                }
            }
        } else {
            if (xl1->compact && xl2->compact) {
                for (x=xl1->start, y=xl2->start; (x<=xl1->end) && (y<=xl2->end); ++x, ++y) {
                    y->X = x->X * s;
                    y->Y = x->Y * s;
                    y->Z = x->Z * s;
                    y->id = x->id;
                }
            } else if (xl1->compact) {
                for (x=xl1->start, y=xl2->start; (x<=xl1->end) && y; ++x, y=y->next) {
                    y->X = x->X * s;
                    y->Y = x->Y * s;
                    y->Z = x->Z * s;
                    y->id = x->id;
                }
            } else if (xl2->compact) {
                for (x=xl1->start, y=xl2->start; x && (y<=xl2->end); x=x->next, ++y) {
                    y->X = x->X * s;
                    y->Y = x->Y * s;
                    y->Z = x->Z * s;
                    y->id = x->id;
                }
            } else {
                for (x=xl1->start, y=xl2->start; x && y; x=x->next, y=y->next) {
                    y->X = x->X * s;
                    y->Y = x->Y * s;
                    y->Z = x->Z * s;
                    y->id = x->id;
                }
            }
        }
    }
}

/**
 Multiply scalar to Euclidean coordinate list.

 @param[in]		xl		Euclidean coordinate list.
 @param[in]		s		Scalar.
 @result	    		New allocated Euclidean coordinate array with multiplied scalar.
*/
xlist *CoordMultNew (xlist *xl, double s)
{
    xlist *list = NULL;

    if (CheckCoords(xl)) {
        list = NewCoordList(xl->num);
        CoordMult(xl, s, list);
        RelinkCoord(list);
    }
    return list;
}

/**
 Subtract scalar from Euclidean coordinate.

 @param[in]			h	    Euclidean coordinate.
 @param[in]			s	    Scalar.
 @result					Subtracted coordinate or NOHOMO3.
*/
homo3 CoordSub (homo3 *h, double s)
{
    homo3 r;

    if (!h) {
        Warning ("CoordSub failed!"); return NOHOMO3;
    }
    NormHomo(h);
    if (ISZERO(s)) {
        r.X = h->X;
        r.Y = h->Y;
        r.Z = h->Z;
    } else {
        r.X = h->X - s;
        r.Y = h->Y - s;
        r.Z = h->Z - s;
    }
    r.W = 1.0;
    r.id = h->id;

    return r;
}

/**
 Subtract scalar from Euclidean coordinate.

 @param[in]			x	    Euclidean coordinate.
 @param[in]			s	    Scalar.
 @result					Subtracted coordinate or NOHOMO3.
*/
homo3 CoordSub (xelem *x, double s)
{
    homo3 r;

    if (!x) {
        Warning ("CoordSub failed!"); return NOHOMO3;
    }
    if (ISZERO(s)) {
        r.X = x->X;
        r.Y = x->Y;
        r.Z = x->Z;
    } else {
        r.X = x->X - s;
        r.Y = x->Y - s;
        r.Z = x->Z - s;
    }
    r.W = 1.0;
    r.id = x->id;

    return r;
}

/**
 Subtract scalar from Euclidean coordinate list.

 @param[in,out]		xl		Modified Euclidean coordinate list.
 @param[in]			s		Scalar.
*/
void CoordSub (xlist *xl, double s)
{
    xelem *x;

    if (CheckCoords(xl) && ISNONZERO(s)) {
        if (xl->compact) {
            for (x=xl->start; x<=xl->end; ++x) {
                x->X -= s;
                x->Y -= s;
                x->Z -= s;
            }
        } else {
            for (x=xl->start; x; x=x->next) {
                x->X -= s;
                x->Y -= s;
                x->Z -= s;
            }
        }
    }
}

/**
 Subtract scalar from Euclidean coordinate list.

 @param[in]		xl1		Euclidean coordinate list.
 @param[in]		s		Scalar.
 @param[out]	xl2		Overwritten Euclidean coordinate list with subtracted scalar.
*/
void CoordSub (xlist *xl1, double s, xlist *xl2)
{
    xelem *x, *y;

    if (CheckSameCoords(xl1,xl2)) {
        if (ISZERO(s)) {
            CopyCoord(xl1, xl2);
        } else {
            if (xl1->compact && xl2->compact) {
                for (x=xl1->start, y=xl2->start; (x<=xl1->end) && (y<=xl2->end); ++x, ++y) {
                    y->X = x->X - s;
                    y->Y = x->Y - s;
                    y->Z = x->Z - s;
                    y->id = x->id;
                }
            } else if (xl1->compact) {
                for (x=xl1->start, y=xl2->start; (x<=xl1->end) && y; ++x, y=y->next) {
                    y->X = x->X - s;
                    y->Y = x->Y - s;
                    y->Z = x->Z - s;
                    y->id = x->id;
                }
            } else if (xl2->compact) {
                for (x=xl1->start, y=xl2->start; x && (y<=xl2->end); x=x->next, ++y) {
                    y->X = x->X - s;
                    y->Y = x->Y - s;
                    y->Z = x->Z - s;
                    y->id = x->id;
                }
            } else {
                for (x=xl1->start, y=xl2->start; x && y; x=x->next, y=y->next) {
                    y->X = x->X - s;
                    y->Y = x->Y - s;
                    y->Z = x->Z - s;
                    y->id = x->id;
                }
            }
        }
    }
}

/**
 Subtract scalar from Euclidean coordinate lists.

 @param[in]		xl		First Euclidean coordinate list.
 @param[in]		s		Scalar.
 @result	    		New allocated Euclidean coordinate array with subtracted scalar.
*/
xlist *CoordSubNew (xlist *xl, double s)
{
    xlist *list = NULL;

    if (CheckCoords(xl)) {
        list = NewCoordList(xl->num);
        CoordSub(xl, s, list);
        RelinkCoord(list);
    }
    return list;
}

//@}

/** @name Arithmetic Element-by-element Operations */
//@{

/**
 Add corresponding elements of two Euclidean coordinates.

 @param[in]			h1	    First Euclidean coordinate.
 @param[in]			h2	    Second Euclidean coordinate.
 @result					Added coordinate or NOHOMO3.
 @note						Coordinate ID is zero.
*/
homo3 CoordAdd (homo3 *h1, homo3 *h2)
{
    homo3 r;

    if (!h1 || !h2) {
        Warning ("CoordAdd failed!"); return NOHOMO3;
    }
    NormHomo(h1);
    NormHomo(h2);
    r.X = h1->X + h2->X;
    r.Y = h1->Y + h2->Y;
    r.Z = h1->Z + h2->Z;
    r.W = 1.0;
    r.id = 0;

    return r;
}

/**
 Add corresponding elements of two Euclidean coordinates.

 @param[in]			x1	    First Euclidean coordinate.
 @param[in]			h2	    Second Euclidean coordinate.
 @result					Added coordinate or NOHOMO3.
 @note						Coordinate ID is zero.
*/
homo3 CoordAdd (xelem *x1, homo3 *h2)
{
    homo3 r;

    if (!x1 || !h2) {
        Warning ("CoordAdd failed!"); return NOHOMO3;
    }
    NormHomo(h2);
    r.X = x1->X + h2->X;
    r.Y = x1->Y + h2->Y;
    r.Z = x1->Z + h2->Z;
    r.W = 1.0;
    r.id = 0;

    return r;
}

/**
 Add corresponding elements of two Euclidean coordinates.

 @param[in]			h1	    First Euclidean coordinate.
 @param[in]			x2	    Second Euclidean coordinate.
 @result					Added coordinate or NOHOMO3.
 @note						Coordinate ID is zero.
*/
homo3 CoordAdd (homo3 *h1, xelem *x2)
{
    homo3 r;

    if (!h1 || !x2) {
        Warning ("CoordAdd failed!"); return NOHOMO3;
    }
    NormHomo(h1);
    r.X = h1->X + x2->X;
    r.Y = h1->Y + x2->Y;
    r.Z = h1->Z + x2->Z;
    r.W = 1.0;
    r.id = 0;

    return r;
}

/**
 Add corresponding elements of two Euclidean coordinates.

 @param[in]			x1	    First Euclidean coordinate.
 @param[in]			x2	    Second Euclidean coordinate.
 @result					Added coordinate or NOHOMO3.
 @note						Coordinate ID is zero.
*/
homo3 CoordAdd (xelem *x1, xelem *x2)
{
    homo3 r;

    if (!x1 || !x2) {
        Warning ("CoordAdd failed!"); return NOHOMO3;
    }
    r.X = x1->X + x2->X;
    r.Y = x1->Y + x2->Y;
    r.Z = x1->Z + x2->Z;
    r.W = 1.0;
    r.id = 0;

    return r;
}

/**
 Add corresponding elements with identical IDs of two Euclidean coordinate lists.

 @param[in,out]		xl1		Modified first Euclidean coordinate list.
 @param[in]			xl2		Second Euclidean coordinate list.
*/
void CoordAdd (xlist *xl1, xlist *xl2)
{
    xelem *x, *y;

    if (CheckSameCoords(xl1,xl2)) {
        if (xl1->compact && xl2->compact) {
            for (x=xl1->start, y=xl2->start; (x<=xl1->end) && (y<=xl2->end); ++x, ++y) {
                if (CheckID(x, y)) {
                    x->X += y->X;
                    x->Y += y->Y;
                    x->Z += y->Z;
                }
            }
        } else if (xl1->compact) {
            for (x=xl1->start, y=xl2->start; (x<=xl1->end) && y; ++x, y=y->next) {
                if (CheckID(x, y)) {
                    x->X += y->X;
                    x->Y += y->Y;
                    x->Z += y->Z;
                }
            }
        } else if (xl2->compact) {
            for (x=xl1->start, y=xl2->start; x && (y<=xl2->end); x=x->next, ++y) {
                if (CheckID(x, y)) {
                    x->X += y->X;
                    x->Y += y->Y;
                    x->Z += y->Z;
                }
            }
        } else {
            for (x=xl1->start, y=xl2->start; x && y; x=x->next, y=y->next) {
                if (CheckID(x, y)) {
                    x->X += y->X;
                    x->Y += y->Y;
                    x->Z += y->Z;
                }
            }
        }
    }
}

/**
 Add corresponding elements with identical IDs of two Euclidean coordinate lists.

 @param[in]		xl1		First Euclidean coordinate list.
 @param[in]		xl2		Second Euclidean coordinate list.
 @param[out]	xl3		Overwritten Euclidean coordinate list with added elements.
*/
void CoordAdd (xlist *xl1, xlist *xl2, xlist *xl3)
{
    xelem *x, *y, *z;

    if (CheckSameCoords(xl1,xl2) && CheckSameCoords(xl1,xl3)) {
        if (xl1->compact && xl2->compact) {
            if (xl3->compact) {
                for (x=xl1->start, y=xl2->start, z=xl3->start; (x<=xl1->end) && (y<=xl2->end) && (z<=xl3->end); ++x, ++y, ++z) {
                    if (CheckID(x, y)) {
                        z->X = x->X + y->X;
                        z->Y = x->Y + y->Y;
                        z->Z = x->Z + y->Z;
                        z->id = x->id;
                    }
                }
            } else {
                for (x=xl1->start, y=xl2->start, z=xl3->start; (x<=xl1->end) && (y<=xl2->end) && z; ++x, ++y, z=z->next) {
                    if (CheckID(x, y)) {
                        z->X = x->X + y->X;
                        z->Y = x->Y + y->Y;
                        z->Z = x->Z + y->Z;
                        z->id = x->id;
                    }
                }
            }
        } else if (xl1->compact) {
            if (xl3->compact) {
                for (x=xl1->start, y=xl2->start, z=xl3->start; (x<=xl1->end) && y && (z<=xl3->end); ++x, y=y->next, ++z) {
                    if (CheckID(x, y)) {
                        z->X = x->X + y->X;
                        z->Y = x->Y + y->Y;
                        z->Z = x->Z + y->Z;
                        z->id = x->id;
                    }
                }
            } else {
                for (x=xl1->start, y=xl2->start, z=xl3->start; (x<=xl1->end) && y && z; ++x, y=y->next, z=z->next) {
                    if (CheckID(x, y)) {
                        z->X = x->X + y->X;
                        z->Y = x->Y + y->Y;
                        z->Z = x->Z + y->Z;
                        z->id = x->id;
                    }
                }
            }
        } else if (xl2->compact) {
            if (xl3->compact) {
                for (x=xl1->start, y=xl2->start, z=xl3->start; x && (y<=xl2->end) && (z<=xl3->end); x=x->next, ++y, ++z) {
                    if (CheckID(x, y)) {
                        z->X = x->X + y->X;
                        z->Y = x->Y + y->Y;
                        z->Z = x->Z + y->Z;
                        z->id = x->id;
                    }
                }
            } else {
                for (x=xl1->start, y=xl2->start, z=xl3->start; x && (y<=xl2->end) && z; x=x->next, ++y, z=z->next) {
                    if (CheckID(x, y)) {
                        z->X = x->X + y->X;
                        z->Y = x->Y + y->Y;
                        z->Z = x->Z + y->Z;
                        z->id = x->id;
                    }
                }
            }
        } else {
            if (xl3->compact) {
                for (x=xl1->start, y=xl2->start, z=xl3->start; x && y && (z<=xl3->end); x=x->next, y=y->next, ++z) {
                    if (CheckID(x, y)) {
                        z->X = x->X + y->X;
                        z->Y = x->Y + y->Y;
                        z->Z = x->Z + y->Z;
                        z->id = x->id;
                    }
                }
            } else {
                for (x=xl1->start, y=xl2->start, z=xl3->start; x && y && z; x=x->next, y=y->next, z=z->next) {
                    if (CheckID(x, y)) {
                        z->X = x->X + y->X;
                        z->Y = x->Y + y->Y;
                        z->Z = x->Z + y->Z;
                        z->id = x->id;
                    }
                }
            }
        }
    }
}

/**
 Add corresponding elements with identical IDs of two Euclidean coordinate lists.

 @param[in]		xl1		First Euclidean coordinate list.
 @param[in]		xl2		Second Euclidean coordinate list.
 @result	    		New allocated Euclidean coordinate array with added elements.
*/
xlist *CoordAddNew (xlist *xl1, xlist *xl2)
{
    xlist *list = NULL;

    if (CheckSameCoords(xl1, xl2)) {
        list = NewCoordList(xl1->num);
        CoordAdd(xl1, xl2, list);
        RelinkCoord(list);
    }
    return list;
}

/**
 Divide corresponding elements of two Euclidean coordinates.

 @param[in]			h1	    First Euclidean coordinate.
 @param[in]			h2	    Second Euclidean coordinate.
 @result					Divided coordinate or NOHOMO3.
 @note						Coordinate ID is zero.
*/
homo3 CoordDiv (homo3 *h1, homo3 *h2)
{
    homo3 r;

    if (!h1 || !h2) {
        Warning ("CoordDiv failed!"); return NOHOMO3;
    }
    NormHomo(h1);
    NormHomo(h2);
    r.X = DIVIDE(h1->X, h2->X);
    r.Y = DIVIDE(h1->Y, h2->Y);
    r.Z = DIVIDE(h1->Z, h2->Z);
    r.W = 1.0;
    r.id = 0;

    return r;
}

/**
 Divide corresponding elements of two Euclidean coordinates.

 @param[in]			x1	    First Euclidean coordinate.
 @param[in]			h2	    Second Euclidean coordinate.
 @result					Divided coordinate or NOHOMO3.
 @note						Coordinate ID is zero.
*/
homo3 CoordDiv(xelem *x1, homo3 *h2)
{
    homo3 r;

    if (!x1 || !h2) {
        Warning ("CoordDiv failed!"); return NOHOMO3;
    }
    NormHomo(h2);
    r.X = DIVIDE(x1->X, h2->X);
    r.Y = DIVIDE(x1->Y, h2->Y);
    r.Z = DIVIDE(x1->Z, h2->Z);
    r.W = 1.0;
    r.id = 0;

    return r;
}

/**
 Divide corresponding elements of two Euclidean coordinates.

 @param[in]			h1	    First Euclidean coordinate.
 @param[in]			x2	    Second Euclidean coordinate.
 @result					Divided coordinate or NOHOMO3.
 @note						Coordinate ID is zero.
*/
homo3 CoordDiv (homo3 *h1, xelem *x2)
{
    homo3 r;

    if (!h1 || !x2) {
        Warning ("CoordDiv failed!"); return NOHOMO3;
    }
    NormHomo(h1);
    r.X = DIVIDE(h1->X, x2->X);
    r.Y = DIVIDE(h1->Y, x2->Y);
    r.Z = DIVIDE(h1->Z, x2->Z);
    r.W = 1.0;
    r.id = 0;

    return r;
}

/**
 Divide corresponding elements of two Euclidean coordinates.

 @param[in]			x1	    First Euclidean coordinate.
 @param[in]			x2	    Second Euclidean coordinate.
 @result					Divided coordinate or NOHOMO3.
 @note						Coordinate ID is zero.
*/
homo3 CoordDiv (xelem *x1, xelem *x2)
{
    homo3 r;

    if (!x1 || !x2) {
        Warning ("CoordDiv failed!"); return NOHOMO3;
    }
    r.X = DIVIDE(x1->X, x2->X);
    r.Y = DIVIDE(x1->Y, x2->Y);
    r.Z = DIVIDE(x1->Z, x2->Z);
    r.W = 1.0;
    r.id = 0;

    return r;
}

/**
 Divide corresponding elements with identical IDs of two Euclidean coordinate lists.

 @param[in,out]		xl1		Modified first Euclidean coordinate list.
 @param[in]			xl2		Second Euclidean coordinate list.
*/
void CoordDiv (xlist *xl1, xlist *xl2)
{
    xelem *x, *y;

    if (CheckSameCoords(xl1,xl2)) {
        if (xl1->compact && xl2->compact) {
            for (x=xl1->start, y=xl2->start; (x<=xl1->end) && (y<=xl2->end); ++x, ++y) {
                if (CheckID(x, y)) {
                    x->X = DIVIDE(x->X, y->X);
                    x->Y = DIVIDE(x->Y, y->Y);
                    x->Z = DIVIDE(x->Z, y->Z);
                }
            }
        } else if (xl1->compact) {
            for (x=xl1->start, y=xl2->start; (x<=xl1->end) && y; ++x, y=y->next) {
                if (CheckID(x, y)) {
                    x->X = DIVIDE(x->X, y->X);
                    x->Y = DIVIDE(x->Y, y->Y);
                    x->Z = DIVIDE(x->Z, y->Z);
                }
            }
        } else if (xl2->compact) {
            for (x=xl1->start, y=xl2->start; x && (y<=xl2->end); x=x->next, ++y) {
                if (CheckID(x, y)) {
                    x->X = DIVIDE(x->X, y->X);
                    x->Y = DIVIDE(x->Y, y->Y);
                    x->Z = DIVIDE(x->Z, y->Z);
                }
            }
        } else {
            for (x=xl1->start, y=xl2->start; x && y; x=x->next, y=y->next) {
                if (CheckID(x, y)) {
                    x->X = DIVIDE(x->X, y->X);
                    x->Y = DIVIDE(x->Y, y->Y);
                    x->Z = DIVIDE(x->Z, y->Z);
                }
            }
        }
    }
}

/**
 Divide corresponding elements with identical IDs of two Euclidean coordinate lists.

 @param[in]		xl1		First Euclidean coordinate list.
 @param[in]		xl2		Second Euclidean coordinate list.
 @param[out]	xl3		Overwritten Euclidean coordinate list with divided elements.
*/
void CoordDiv (xlist *xl1, xlist *xl2, xlist *xl3)
{
    xelem *x, *y, *z;

    if (CheckSameCoords(xl1,xl2) && CheckSameCoords(xl1,xl3)) {
        if (xl1->compact && xl2->compact) {
            if (xl3->compact) {
                for (x=xl1->start, y=xl2->start, z=xl3->start; (x<=xl1->end) && (y<=xl2->end) && (z<=xl3->end); ++x, ++y, ++z) {
                    if (CheckID(x, y)) {
                        z->X = DIVIDE(x->X, y->X);
                        z->Y = DIVIDE(x->Y, y->Y);
                        z->Z = DIVIDE(x->Z, y->Z);
                        z->id = x->id;
                    }
                }
            } else {
                for (x=xl1->start, y=xl2->start, z=xl3->start; (x<=xl1->end) && (y<=xl2->end) && z; ++x, ++y, z=z->next) {
                    if (CheckID(x, y)) {
                        z->X = DIVIDE(x->X, y->X);
                        z->Y = DIVIDE(x->Y, y->Y);
                        z->Z = DIVIDE(x->Z, y->Z);
                        z->id = x->id;
                    }
                }
            }
        } else if (xl1->compact) {
            if (xl3->compact) {
                for (x=xl1->start, y=xl2->start, z=xl3->start; (x<=xl1->end) && y && (z<=xl3->end); ++x, y=y->next, ++z) {
                    if (CheckID(x, y)) {
                        z->X = DIVIDE(x->X, y->X);
                        z->Y = DIVIDE(x->Y, y->Y);
                        z->Z = DIVIDE(x->Z, y->Z);
                        z->id = x->id;
                    }
                }
            } else {
                for (x=xl1->start, y=xl2->start, z=xl3->start; (x<=xl1->end) && y && z; ++x, y=y->next, z=z->next) {
                    if (CheckID(x, y)) {
                        z->X = DIVIDE(x->X, y->X);
                        z->Y = DIVIDE(x->Y, y->Y);
                        z->Z = DIVIDE(x->Z, y->Z);
                        z->id = x->id;
                    }
                }
            }
        } else if (xl2->compact) {
            if (xl3->compact) {
                for (x=xl1->start, y=xl2->start, z=xl3->start; x && (y<=xl2->end) && (z<=xl3->end); x=x->next, ++y, ++z) {
                    if (CheckID(x, y)) {
                        z->X = DIVIDE(x->X, y->X);
                        z->Y = DIVIDE(x->Y, y->Y);
                        z->Z = DIVIDE(x->Z, y->Z);
                        z->id = x->id;
                    }
                }
            } else {
                for (x=xl1->start, y=xl2->start, z=xl3->start; x && (y<=xl2->end) && z; x=x->next, ++y, z=z->next) {
                    if (CheckID(x, y)) {
                        z->X = DIVIDE(x->X, y->X);
                        z->Y = DIVIDE(x->Y, y->Y);
                        z->Z = DIVIDE(x->Z, y->Z);
                        z->id = x->id;
                    }
                }
            }
        } else {
            if (xl3->compact) {
                for (x=xl1->start, y=xl2->start, z=xl3->start; x && y && (z<=xl3->end); x=x->next, y=y->next, ++z) {
                    if (CheckID(x, y)) {
                        z->X = DIVIDE(x->X, y->X);
                        z->Y = DIVIDE(x->Y, y->Y);
                        z->Z = DIVIDE(x->Z, y->Z);
                        z->id = x->id;
                    }
                }
            } else {
                for (x=xl1->start, y=xl2->start, z=xl3->start; x && y && z; x=x->next, y=y->next, z=z->next) {
                    if (CheckID(x, y)) {
                        z->X = DIVIDE(x->X, y->X);
                        z->Y = DIVIDE(x->Y, y->Y);
                        z->Z = DIVIDE(x->Z, y->Z);
                        z->id = x->id;
                    }
                }
            }
        }
    }
}

/**
 Divide corresponding elements with identical IDs of two Euclidean coordinate lists.

 @param[in]		xl1		First Euclidean coordinate list.
 @param[in]		xl2		Second Euclidean coordinate list.
 @result	    		New allocated Euclidean coordinate array with divided elements.
*/
xlist *CoordDivNew (xlist *xl1, xlist *xl2)
{
    xlist *list = NULL;

    if (CheckSameCoords(xl1, xl2)) {
        list = NewCoordList(xl1->num);
        CoordDiv(xl1, xl2, list);
        RelinkCoord(list);
    }
    return list;
}

/**
 Multiply corresponding elements of two Euclidean coordinates.

 @param[in]			h1	    First Euclidean coordinate.
 @param[in]			h2	    Second Euclidean coordinate.
 @result					Multiplied coordinate or NOHOMO3.
 @note						Coordinate ID is zero.
*/
homo3 CoordMult (homo3 *h1, homo3 *h2)
{
    homo3 r;

    if (!h1 || !h2) {
        Warning ("CoordMult failed!"); return NOHOMO3;
    }
    NormHomo(h1);
    NormHomo(h2);
    r.X = h1->X * h2->X;
    r.Y = h1->Y * h2->Y;
    r.Z = h1->Z * h2->Z;
    r.W = 1.0;
    r.id = 0;

    return r;
}

/**
 Multiply corresponding elements of two Euclidean coordinates.

 @param[in]			x1	    First Euclidean coordinate.
 @param[in]			h2	    Second Euclidean coordinate.
 @result					Multiplied coordinate or NOHOMO3.
 @note						Coordinate ID is zero.
*/
homo3 CoordMult(xelem *x1, homo3 *h2)
{
    homo3 r;

    if (!x1 || !h2) {
        Warning ("CoordMult failed!"); return NOHOMO3;
    }
    NormHomo(h2);
    r.X = x1->X * h2->X;
    r.Y = x1->Y * h2->Y;
    r.Z = x1->Z * h2->Z;
    r.W = 1.0;
    r.id = 0;

    return r;
}

/**
 Multiply corresponding elements of two Euclidean coordinates.

 @param[in]			h1	    First Euclidean coordinate.
 @param[in]			x2	    Second Euclidean coordinate.
 @result					Multiplied coordinate or NOHOMO3.
 @note						Coordinate ID is zero.
*/
homo3 CoordMult (homo3 *h1, xelem *x2)
{
    homo3 r;

    if (!h1 || !x2) {
        Warning ("CoordMult failed!"); return NOHOMO3;
    }
    NormHomo(h1);
    r.X = h1->X * x2->X;
    r.Y = h1->Y * x2->Y;
    r.Z = h1->Z * x2->Z;
    r.W = 1.0;
    r.id = 0;

    return r;
}

/**
 Multiply corresponding elements of two Euclidean coordinates.

 @param[in]			x1	    First Euclidean coordinate.
 @param[in]			x2	    Second Euclidean coordinate.
 @result					Multiplied coordinate or NOHOMO3.
 @note						Coordinate ID is zero.
*/
homo3 CoordMult (xelem *x1, xelem *x2)
{
    homo3 r;

    if (!x1 || !x2) {
        Warning ("CoordMult failed!"); return NOHOMO3;
    }
    r.X = x1->X * x2->X;
    r.Y = x1->Y * x2->Y;
    r.Z = x1->Z * x2->Z;
    r.W = 1.0;
    r.id = 0;

    return r;
}

/**
 Multiply corresponding elements with identical IDs of two Euclidean coordinate lists.

 @param[in,out]		xl1		Modified first Euclidean coordinate list.
 @param[in]			xl2		Second Euclidean coordinate list.
*/
void CoordMult (xlist *xl1, xlist *xl2)
{
    xelem *x, *y;

    if (CheckSameCoords(xl1,xl2)) {
        if (xl1->compact && xl2->compact) {
            for (x=xl1->start, y=xl2->start; (x<=xl1->end) && (y<=xl2->end); ++x, ++y) {
                if (CheckID(x, y)) {
                    x->X *= y->X;
                    x->Y *= y->Y;
                    x->Z *= y->Z;
                }
            }
        } else if (xl1->compact) {
            for (x=xl1->start, y=xl2->start; (x<=xl1->end) && y; ++x, y=y->next) {
                if (CheckID(x, y)) {
                    x->X *= y->X;
                    x->Y *= y->Y;
                    x->Z *= y->Z;
                }
            }
        } else if (xl2->compact) {
            for (x=xl1->start, y=xl2->start; x && (y<=xl2->end); x=x->next, ++y) {
                if (CheckID(x, y)) {
                    x->X *= y->X;
                    x->Y *= y->Y;
                    x->Z *= y->Z;
                }
            }
        } else {
            for (x=xl1->start, y=xl2->start; x && y; x=x->next, y=y->next) {
                if (CheckID(x, y)) {
                    x->X *= y->X;
                    x->Y *= y->Y;
                    x->Z *= y->Z;
                }
            }
        }
    }
}

/**
 Multiplicate corresponding elements with identical IDs of two Euclidean coordinate lists.

 @param[in]		xl1		First Euclidean coordinate list.
 @param[in]		xl2		Second Euclidean coordinate list.
 @param[out]	xl3		Overwritten Euclidean coordinate list with multiplied elements.
*/
void CoordMult (xlist *xl1, xlist *xl2, xlist *xl3)
{
    xelem *x, *y, *z;

    if (CheckSameCoords(xl1,xl2) && CheckSameCoords(xl1,xl3)) {
        if (xl1->compact && xl2->compact) {
            if (xl3->compact) {
                for (x=xl1->start, y=xl2->start, z=xl3->start; (x<=xl1->end) && (y<=xl2->end) && (z<=xl3->end); ++x, ++y, ++z) {
                    if (CheckID(x, y)) {
                        z->X = x->X * y->X;
                        z->Y = x->Y * y->Y;
                        z->Z = x->Z * y->Z;
                        z->id = x->id;
                    }
                }
            } else {
                for (x=xl1->start, y=xl2->start, z=xl3->start; (x<=xl1->end) && (y<=xl2->end) && z; ++x, ++y, z=z->next) {
                    if (CheckID(x, y)) {
                        z->X = x->X * y->X;
                        z->Y = x->Y * y->Y;
                        z->Z = x->Z * y->Z;
                        z->id = x->id;
                    }
                }
            }
        } else if (xl1->compact) {
            if (xl3->compact) {
                for (x=xl1->start, y=xl2->start, z=xl3->start; (x<=xl1->end) && y && (z<=xl3->end); ++x, y=y->next, ++z) {
                    if (CheckID(x, y)) {
                        z->X = x->X * y->X;
                        z->Y = x->Y * y->Y;
                        z->Z = x->Z * y->Z;
                        z->id = x->id;
                    }
                }
            } else {
                for (x=xl1->start, y=xl2->start, z=xl3->start; (x<=xl1->end) && y && z; ++x, y=y->next, z=z->next) {
                    if (CheckID(x, y)) {
                        z->X = x->X * y->X;
                        z->Y = x->Y * y->Y;
                        z->Z = x->Z * y->Z;
                        z->id = x->id;
                    }
                }
            }
        } else if (xl2->compact) {
            if (xl3->compact) {
                for (x=xl1->start, y=xl2->start, z=xl3->start; x && (y<=xl2->end) && (z<=xl3->end); x=x->next, ++y, ++z) {
                    if (CheckID(x, y)) {
                        z->X = x->X * y->X;
                        z->Y = x->Y * y->Y;
                        z->Z = x->Z * y->Z;
                        z->id = x->id;
                    }
                }
            } else {
                for (x=xl1->start, y=xl2->start, z=xl3->start; x && (y<=xl2->end) && z; x=x->next, ++y, z=z->next) {
                    if (CheckID(x, y)) {
                        z->X = x->X * y->X;
                        z->Y = x->Y * y->Y;
                        z->Z = x->Z * y->Z;
                        z->id = x->id;
                    }
                }
            }
        } else {
            if (xl3->compact) {
                for (x=xl1->start, y=xl2->start, z=xl3->start; x && y && (z<=xl3->end); x=x->next, y=y->next, ++z) {
                    if (CheckID(x, y)) {
                        z->X = x->X * y->X;
                        z->Y = x->Y * y->Y;
                        z->Z = x->Z * y->Z;
                        z->id = x->id;
                    }
                }
            } else {
                for (x=xl1->start, y=xl2->start, z=xl3->start; x && y && z; x=x->next, y=y->next, z=z->next) {
                    if (CheckID(x, y)) {
                        z->X = x->X * y->X;
                        z->Y = x->Y * y->Y;
                        z->Z = x->Z * y->Z;
                        z->id = x->id;
                    }
                }
            }
        }
    }
}

/**
 Multiply corresponding elements with identical IDs of two Euclidean coordinate lists.

 @param[in]		xl1		First Euclidean coordinate list.
 @param[in]		xl2		Second Euclidean coordinate list.
 @result	    		New allocated Euclidean coordinate array with multiplied elements.
*/
xlist *CoordMultNew (xlist *xl1, xlist *xl2)
{
    xlist *list = NULL;

    if (CheckSameCoords(xl1, xl2)) {
        list = NewCoordList(xl1->num);
        CoordMult(xl1, xl2, list);
        RelinkCoord(list);
    }
    return list;
}

/**
 Substract corresponding elements of two Euclidean coordinates.

 @param[in]			h1	    First Euclidean coordinate.
 @param[in]			h2	    Second Euclidean coordinate.
 @result					Coordinate difference or NOHOMO3.
 @note						Coordinate ID is zero.
*/
homo3 CoordSub (homo3 *h1, homo3 *h2)
{
    homo3 r;

    if (!h1 || !h2) {
        Warning ("CoordSub failed!"); return NOHOMO3;
    }
    NormHomo(h1);
    NormHomo(h2);
    r.X = h1->X - h2->X;
    r.Y = h1->Y - h2->Y;
    r.Z = h1->Z - h2->Z;
    r.W = 1.0;
    r.id = 0;

    return r;
}

/**
 Substract corresponding elements of two Euclidean coordinates.

 @param[in]			x1	    First Euclidean coordinate.
 @param[in]			h2	    Second Euclidean coordinate.
 @result					Coordinate difference or NOHOMO3.
 @note						Coordinate ID is zero.
*/
homo3 CoordSub (xelem *x1, homo3 *h2)
{
    homo3 r;

    if (!x1 || !h2) {
        Warning ("CoordSub failed!"); return NOHOMO3;
    }
    NormHomo(h2);
    r.X = x1->X - h2->X;
    r.Y = x1->Y - h2->Y;
    r.Z = x1->Z - h2->Z;
    r.W = 1.0;
    r.id = 0;

    return r;
}

/**
 Substract corresponding elements of two Euclidean coordinates.

 @param[in]			h1	    First Euclidean coordinate.
 @param[in]			x2	    Second Euclidean coordinate.
 @result					Coordinate difference or NOHOMO3.
 @note						Coordinate ID is zero.
*/
homo3 CoordSub (homo3 *h1, xelem *x2)
{
    homo3 r;

    if (!h1 || !x2) {
        Warning ("CoordSub failed!"); return NOHOMO3;
    }
    NormHomo(h1);
    r.X = h1->X - x2->X;
    r.Y = h1->Y - x2->Y;
    r.Z = h1->Z - x2->Z;
    r.W = 1.0;
    r.id = 0;

    return r;
}

/**
 Substract corresponding elements of two Euclidean coordinates.

 @param[in]			x1	    First Euclidean coordinate.
 @param[in]			x2	    Second Euclidean coordinate.
 @result					Coordinate difference or NOHOMO3.
 @note						Coordinate ID is zero.
*/
homo3 CoordSub (xelem *x1, xelem *x2)
{
    homo3 r;

    if (!x1 || !x2) {
        Warning ("CoordSub failed!"); return NOHOMO3;
    }
    r.X = x1->X - x2->X;
    r.Y = x1->Y - x2->Y;
    r.Z = x1->Z - x2->Z;
    r.W = 1.0;
    r.id = 0;

    return r;
}

/**
 Substract corresponding elements with identical IDs of two Euclidean coordinate lists.

 @param[in,out]		xl1		Modified first Euclidean coordinate list.
 @param[in]			xl2		Second Euclidean coordinate list.
*/
void CoordSub (xlist *xl1, xlist *xl2)
{
    xelem *x, *y;

    if (CheckSameCoords(xl1,xl2)) {
        if (xl1->compact && xl2->compact) {
            for (x=xl1->start, y=xl2->start; (x<=xl1->end) && (y<=xl2->end); ++x, ++y) {
                if (CheckID(x, y)) {
                    x->X -= y->X;
                    x->Y -= y->Y;
                    x->Z -= y->Z;
                }
            }
        } else if (xl1->compact) {
            for (x=xl1->start, y=xl2->start; (x<=xl1->end) && y; ++x, y=y->next) {
                if (CheckID(x, y)) {
                    x->X -= y->X;
                    x->Y -= y->Y;
                    x->Z -= y->Z;
                }
            }
        } else if (xl2->compact) {
            for (x=xl1->start, y=xl2->start; x && (y<=xl2->end); x=x->next, ++y) {
                if (CheckID(x, y)) {
                    x->X -= y->X;
                    x->Y -= y->Y;
                    x->Z -= y->Z;
                }
            }
        } else {
            for (x=xl1->start, y=xl2->start; x && y; x=x->next, y=y->next) {
                if (CheckID(x, y)) {
                    x->X -= y->X;
                    x->Y -= y->Y;
                    x->Z -= y->Z;
                }
            }
        }
    }
}

/**
 Subtract corresponding elements with identical IDs of two Euclidean coordinate lists.

 @param[in]		xl1		First Euclidean coordinate list.
 @param[in]		xl2		Second Euclidean coordinate list.
 @param[out]	xl3		Overwritten Euclidean coordinate list with subtracted elements.
*/
void CoordSub (xlist *xl1, xlist *xl2, xlist *xl3)
{
    xelem *x, *y, *z;

    if (CheckSameCoords(xl1,xl2) && CheckSameCoords(xl1,xl3)) {
        if (xl1->compact && xl2->compact) {
            if (xl3->compact) {
                for (x=xl1->start, y=xl2->start, z=xl3->start; (x<=xl1->end) && (y<=xl2->end) && (z<=xl3->end); ++x, ++y, ++z) {
                    if (CheckID(x, y)) {
                        z->X = x->X - y->X;
                        z->Y = x->Y - y->Y;
                        z->Z = x->Z - y->Z;
                        z->id = x->id;
                    }
                }
            } else {
                for (x=xl1->start, y=xl2->start, z=xl3->start; (x<=xl1->end) && (y<=xl2->end) && z; ++x, ++y, z=z->next) {
                    if (CheckID(x, y)) {
                        z->X = x->X - y->X;
                        z->Y = x->Y - y->Y;
                        z->Z = x->Z - y->Z;
                        z->id = x->id;
                    }
                }
            }
        } else if (xl1->compact) {
            if (xl3->compact) {
                for (x=xl1->start, y=xl2->start, z=xl3->start; (x<=xl1->end) && y && (z<=xl3->end); ++x, y=y->next, ++z) {
                    if (CheckID(x, y)) {
                        z->X = x->X - y->X;
                        z->Y = x->Y - y->Y;
                        z->Z = x->Z - y->Z;
                        z->id = x->id;
                    }
                }
            } else {
                for (x=xl1->start, y=xl2->start, z=xl3->start; (x<=xl1->end) && y && z; ++x, y=y->next, z=z->next) {
                    if (CheckID(x, y)) {
                        z->X = x->X - y->X;
                        z->Y = x->Y - y->Y;
                        z->Z = x->Z - y->Z;
                        z->id = x->id;
                    }
                }
            }
        } else if (xl2->compact) {
            if (xl3->compact) {
                for (x=xl1->start, y=xl2->start, z=xl3->start; x && (y<=xl2->end) && (z<=xl3->end); x=x->next, ++y, ++z) {
                    if (CheckID(x, y)) {
                        z->X = x->X - y->X;
                        z->Y = x->Y - y->Y;
                        z->Z = x->Z - y->Z;
                        z->id = x->id;
                    }
                }
            } else {
                for (x=xl1->start, y=xl2->start, z=xl3->start; x && (y<=xl2->end) && z; x=x->next, ++y, z=z->next) {
                    if (CheckID(x, y)) {
                        z->X = x->X - y->X;
                        z->Y = x->Y - y->Y;
                        z->Z = x->Z - y->Z;
                        z->id = x->id;
                    }
                }
            }
        } else {
            if (xl3->compact) {
                for (x=xl1->start, y=xl2->start, z=xl3->start; x && y && (z<=xl3->end); x=x->next, y=y->next, ++z) {
                    if (CheckID(x, y)) {
                        z->X = x->X - y->X;
                        z->Y = x->Y - y->Y;
                        z->Z = x->Z - y->Z;
                        z->id = x->id;
                    }
                }
            } else {
                for (x=xl1->start, y=xl2->start, z=xl3->start; x && y && z; x=x->next, y=y->next, z=z->next) {
                    if (CheckID(x, y)) {
                        z->X = x->X - y->X;
                        z->Y = x->Y - y->Y;
                        z->Z = x->Z - y->Z;
                        z->id = x->id;
                    }
                }
            }
        }
    }
}

/**
 Subtract corresponding elements with identical IDs of two Euclidean coordinate lists.

 @param[in]		xl1		First Euclidean coordinate list.
 @param[in]		xl2		Second Euclidean coordinate list.
 @result	    		New allocated Euclidean coordinate array with subtracted elements.
*/
xlist *CoordSubNew (xlist *xl1, xlist *xl2)
{
    xlist *list = NULL;

    if (CheckSameCoords(xl1, xl2)) {
        list = NewCoordList(xl1->num);
        CoordSub(xl1, xl2, list);
        RelinkCoord(list);
    }
    return list;
}

//@}

/** @name Other Element-by-element Operations */
//@{

/**
 Absolute values of an Euclidean coordinate.

 @param[in]			h	    Euclidean coordinate.
 @result					Coordinate or NOHOMO3.
*/
homo3 CoordAbs (homo3 *h)
{
    homo3 r;

    if (!h) {
        Warning ("CoordAbs failed!"); return NOHOMO3;
    }
    NormHomo(h);
    r.X = ABS(h->X);
    r.Y = ABS(h->Y);
    r.Z = ABS(h->Z);
    r.W = 1.0;
    r.id = h->id;

    return r;
}

/**
 Absolute values of an Euclidean coordinate.

 @param[in]			x	    Euclidean coordinate.
 @result					Coordinate or NOHOMO3.
*/
homo3 CoordAbs (xelem *x)
{
    homo3 r;

    if (!x) {
        Warning ("CoordAbs failed!"); return NOHOMO3;
    }
    r.X = ABS(x->X);
    r.Y = ABS(x->Y);
    r.Z = ABS(x->Z);
    r.W = 1.0;
    r.id = x->id;

    return r;
}

/**
 Absolute values of an Euclidean coordinate list.

 @param[in,out]		xl		Modified Euclidean coordinate list.
*/
void CoordAbs (xlist *xl)
{
    xelem *x;

    if (CheckCoords(xl)) {
        if (xl->compact) {
            for (x=xl->start; x<=xl->end; ++x) {
                x->X = ABS(x->X);
                x->Y = ABS(x->Y);
                x->Z = ABS(x->Z);
            }
        } else {
            for (x=xl->start; x; x=x->next) {
                x->X = ABS(x->X);
                x->Y = ABS(x->Y);
                x->Z = ABS(x->Z);
            }
        }
    }
}

/**
 Absolute values of an Euclidean coordinate list.

 @param[in]		xl1		Euclidean coordinate list.
 @param[out]	xl2		Overwritten Euclidean coordinate list with absolute values.
*/
void CoordAbs (xlist *xl1, xlist *xl2)
{
    xelem *x, *y;

    if (CheckSameCoords(xl1,xl2)) {
        if (xl1->compact && xl2->compact) {
            for (x=xl1->start, y=xl2->start; (x<=xl1->end) && (y<=xl2->end); ++x, ++y) {
                y->X = ABS(x->X);
                y->Y = ABS(x->Y);
                y->Z = ABS(x->Z);
                y->id = x->id;
            }
        } else if (xl1->compact) {
            for (x=xl1->start, y=xl2->start; (x<=xl1->end) && y; ++x, y=y->next) {
                y->X = ABS(x->X);
                y->Y = ABS(x->Y);
                y->Z = ABS(x->Z);
                y->id = x->id;
            }
        } else if (xl2->compact) {
            for (x=xl1->start, y=xl2->start; x && (y<=xl2->end); x=x->next, ++y) {
                y->X = ABS(x->X);
                y->Y = ABS(x->Y);
                y->Z = ABS(x->Z);
                y->id = x->id;
            }
        } else {
            for (x=xl1->start, y=xl2->start; x && y; x=x->next, y=y->next) {
                y->X = ABS(x->X);
                y->Y = ABS(x->Y);
                y->Z = ABS(x->Z);
                y->id = x->id;
            }
        }
    }
}

/**
 Absolute values of an Euclidean coordinate list.

 @param[in]		xl		First Euclidean coordinate list.
 @result	    		New allocated Euclidean coordinate array with absolute values.
*/
xlist *CoordAbsNew (xlist *xl)
{
    xlist *list = NULL;

    if (CheckCoords(xl)) {
        list = NewCoordList(xl->num);
        CoordAbs(xl, list);
        RelinkCoord(list);
    }
    return list;
}

/**
 Squared values of an Euclidean coordinate.

 @param[in]			h	    Euclidean coordinate.
 @result					Coordinate or NOHOMO3.
*/
homo3 CoordSqr (homo3 *h)
{
    homo3 r;

    if (!h) {
        Warning ("CoordSqr failed!"); return NOHOMO3;
    }
    NormHomo(h);
    r.X = SQR(h->X);
    r.Y = SQR(h->Y);
    r.Z = SQR(h->Z);
    r.W = 1.0;
    r.id = h->id;

    return r;
}

/**
 Squared values of an Euclidean coordinate.

 @param[in]			x	    Euclidean coordinate.
 @result					Coordinate or NOHOMO3.
*/
homo3 CoordSqr (xelem *x)
{
    homo3 r;

    if (!x) {
        Warning ("CoordSqr failed!"); return NOHOMO3;
    }
    r.X = SQR(x->X);
    r.Y = SQR(x->Y);
    r.Z = SQR(x->Z);
    r.W = 1.0;
    r.id = x->id;

    return r;
}

/**
 Squared values of an Euclidean coordinate list.

 @param[in,out]		xl		Modified Euclidean coordinate list.
*/
void CoordSqr (xlist *xl)
{
    CoordMult(xl, xl);
}

/**
 Squared values of an Euclidean coordinate list.

 @param[in]		xl1		Euclidean coordinate list.
 @param[out]	xl2		Overwritten Euclidean coordinate list with squared values.
*/
void CoordSqr (xlist *xl1, xlist *xl2)
{
    CoordMult(xl1, xl1, xl2);
}

/**
 Squared values of an Euclidean coordinate list.

 @param[in]		xl		First Euclidean coordinate list.
 @result	    		New allocated Euclidean coordinate array with squared values.
*/
xlist *CoordSqrNew (xlist *xl)
{
    return CoordMultNew(xl, xl);
}

/**
 Square root values of an Euclidean coordinate.

 @param[in]			h	    Euclidean coordinate.
 @result					Coordinate or NOHOMO3.
*/
homo3 CoordSqrt (homo3 *h)
{
    homo3 r;

    if (!h) {
        Warning ("CoordSqrt failed!"); return NOHOMO3;
    }
    NormHomo(h);
    r.X = sqrt(h->X);
    r.Y = sqrt(h->Y);
    r.Z = sqrt(h->Z);
    r.W = 1.0;
    r.id = h->id;

    return r;
}

/**
 Square root values of an Euclidean coordinate.

 @param[in]			x	    Euclidean coordinate.
 @result					Coordinate or NOHOMO3.
*/
homo3 CoordSqrt (xelem *x)
{
    homo3 r;

    if (!x) {
        Warning ("CoordSqrt failed!"); return NOHOMO3;
    }
    r.X = sqrt(x->X);
    r.Y = sqrt(x->Y);
    r.Z = sqrt(x->Z);
    r.W = 1.0;
    r.id = x->id;

    return r;
}

/**
 Square root values of an Euclidean coordinate list.

 @param[in,out]		xl		Modified Euclidean coordinate list.
*/
void CoordSqrt (xlist *xl)
{
    xelem *x;

    if (CheckCoords(xl)) {
        if (xl->compact) {
            for (x=xl->start; x<=xl->end; ++x) {
                x->X = sqrt(x->X);
                x->Y = sqrt(x->Y);
                x->Z = sqrt(x->Z);
            }
        } else {
            for (x=xl->start; x; x=x->next) {
                x->X = sqrt(x->X);
                x->Y = sqrt(x->Y);
                x->Z = sqrt(x->Z);
            }
        }
    }
}

/**
 Absolute values of an Euclidean coordinate list.

 @param[in]		xl1		Euclidean coordinate list.
 @param[out]	xl2		Overwritten Euclidean coordinate list with absolute values.
*/
void CoordSqrt (xlist *xl1, xlist *xl2)
{
    xelem *x, *y;

    if (CheckSameCoords(xl1,xl2)) {
        if (xl1->compact && xl2->compact) {
            for (x=xl1->start, y=xl2->start; (x<=xl1->end) && (y<=xl2->end); ++x, ++y) {
                y->X = sqrt(x->X);
                y->Y = sqrt(x->Y);
                y->Z = sqrt(x->Z);
                y->id = x->id;
            }
        } else if (xl1->compact) {
            for (x=xl1->start, y=xl2->start; (x<=xl1->end) && y; ++x, y=y->next) {
                y->X = sqrt(x->X);
                y->Y = sqrt(x->Y);
                y->Z = sqrt(x->Z);
                y->id = x->id;
            }
        } else if (xl2->compact) {
            for (x=xl1->start, y=xl2->start; x && (y<=xl2->end); x=x->next, ++y) {
                y->X = sqrt(x->X);
                y->Y = sqrt(x->Y);
                y->Z = sqrt(x->Z);
                y->id = x->id;
            }
        } else {
            for (x=xl1->start, y=xl2->start; x && y; x=x->next, y=y->next) {
                y->X = sqrt(x->X);
                y->Y = sqrt(x->Y);
                y->Z = sqrt(x->Z);
                y->id = x->id;
            }
        }
    }
}

/**
 Absolute values of an Euclidean coordinate list.

 @param[in]		xl		First Euclidean coordinate list.
 @result	    		New allocated Euclidean coordinate array with absolute values.
*/
xlist *CoordSqrtNew (xlist *xl)
{
    xlist *list = NULL;

    if (CheckCoords(xl)) {
        list = NewCoordList(xl->num);
        CoordSqrt(xl, list);
        RelinkCoord(list);
    }
    return list;
}

//@}

/** @name Geometric Coordinate Operations */
//@{


// ===========================================================================
//
// ===========================================================================

double CoordDist (homo3 *h1, homo3 *h2)
{
    return sqrt (CoordDistSqr(h1, h2));
}

double CoordDist (xelem *x1, homo3 *h2)
{
    return sqrt (CoordDistSqr(x1, h2));
}

double CoordDist (homo3 *h1, xelem *x2)
{
    return sqrt (CoordDistSqr(h1, x2));
}

double CoordDist (xelem *x1, xelem *x2)
{
    return sqrt (CoordDistSqr(x1, x2));
}

// ===========================================================================
//
// ===========================================================================

double CoordDistSqr (homo3 *h1, homo3 *h2)
{
    if (!h1 || !h2) {
        Warning ("CoordDistSqr failed!"); return 0.0;
    }
    NormHomo (h1);
    NormHomo (h2);
    return (SQR(h1->X - h2->X) + SQR(h1->Y - h2->Y) + SQR(h1->Z - h2->Z));
}

double CoordDistSqr (xelem *x1, homo3 *h2)
{
    if (!x1 || !h2) {
        Warning ("CoordDistSqr failed!"); return 0.0;
    }
    NormHomo (h2);
    return (SQR(x1->X - h2->X) + SQR(x1->Y - h2->Y) + SQR(x1->Z - h2->Z));
}

double CoordDistSqr (homo3 *h1, xelem *x2)
{
    if (!h1 || !x2) {
        Warning ("CoordDistSqr failed!"); return 0.0;
    }
    NormHomo (h1);
    return (SQR(h1->X - x2->X) + SQR(h1->Y - x2->Y) + SQR(h1->Z - x2->Z));
}

double CoordDistSqr (xelem *x1, xelem *x2)
{
    if (!x1 || !x2) {
        Warning ("CoordDistSqr failed!"); return 0.0;
    }
    return (SQR(x1->X - x2->X) + SQR(x1->Y - x2->Y) + SQR(x1->Z - x2->Z));
}

// ===========================================================================
//
// ===========================================================================

double CoordLen (homo3 *h)
{
    if (!h) {
        Warning ("CoordLen failed!"); return 0.0;
    }
    return sqrt(SQR(h->X) + SQR(h->Y) + SQR(h->Z) + SQR(h->W));
}

double CoordLen (xelem *x)
{
    if (!x) {
        Warning ("CoordLen failed!"); return 0.0;
    }
    return sqrt(SQR(x->X) + SQR(x->Y) + SQR(x->Z) + 1.0);
}

/**
 Scalar/inner/dot product of two Euclidean coordinates.

 @param[in]			h1	    First Euclidean coordinate.
 @param[in]			h2	    Second Euclidean coordinate.
 @result					Scalar product or 0.
 @note						Coordinate ID is zero.
*/
double CoordProd (homo3 *h1, homo3 *h2)
{
    if (!h1 || !h2) {
        Warning ("CoordProd failed!"); return 0.0;
    }
    return h1->X*h2->X + h1->Y*h2->Y + h1->Z*h2->Z;
}

/**
 Scalar/inner/dot product of two Euclidean coordinates.

 @param[in]			x1	    First Euclidean coordinate.
 @param[in]			h2	    Second Euclidean coordinate.
 @result					Scalar product or 0.
 @note						Coordinate ID is zero.
*/
double CoordProd (xelem *x1, homo3 *h2)
{
    if (!x1 || !h2) {
        Warning ("CoordProd failed!"); return 0.0;
    }
    return x1->X*h2->X + x1->Y*h2->Y + x1->Z*h2->Z;
}

/**
 Scalar/inner/dot product of two Euclidean coordinates.

 @param[in]			h1	    First Euclidean coordinate.
 @param[in]			x2	    Second Euclidean coordinate.
 @result					Scalar product or 0.
 @note						Coordinate ID is zero.
*/
double CoordProd (homo3 *h1, xelem *x2)
{
    if (!h1 || !x2) {
        Warning ("CoordProd failed!"); return 0.0;
    }
    return h1->X*x2->X + h1->Y*x2->Y + h1->Z*x2->Z;
}

/**
 Scalar/inner/dot product of two Euclidean coordinates.

 @param[in]			x1	    First Euclidean coordinate.
 @param[in]			x2	    Second Euclidean coordinate.
 @result					Scalar product or 0.
 @note						Coordinate ID is zero.
*/
double CoordProd (xelem *x1, xelem *x2)
{
    if (!x1 || !x2) {
        Warning ("CoordProd failed!"); return 0.0;
    }
    return x1->X*x2->X + x1->Y*x2->Y + x1->Z*x2->Z;
}

// ===========================================================================
//
// ===========================================================================

void ScaleCoord (homo3 *h, double f)
{
    double t, s;

    if (h) {
        t = h->W;
        if (ISNONZERO(t)) {
            s = f / t;
            h->X *= s;
            h->Y *= s;
            h->Z *= s;
            h->W = 1.0;
        }
    }
}

// ===========================================================================
//
// ===========================================================================

homo3 TransformCoord (matrix *H, homo3 *X)
{
    homo3 r;

    if (!CheckSizeMatrix (H, 4, 4) || !X) {
        Warning ("TransformCoord failed!"); return NOHOMO3;
    }
    r.X = H->m[0][0] * X->X + H->m[0][1] * X->Y + H->m[0][2] * X->Z + H->m[0][3] * X->W;
    r.Y = H->m[1][0] * X->X + H->m[1][1] * X->Y + H->m[1][2] * X->Z + H->m[1][3] * X->W;
    r.Z = H->m[2][0] * X->X + H->m[2][1] * X->Y + H->m[2][2] * X->Z + H->m[2][3] * X->W;
    r.W = H->m[3][0] * X->X + H->m[3][1] * X->Y + H->m[3][2] * X->Z + H->m[3][3] * X->W;
    r.id = X->id;

    return r;
}

void TransformCoord (matrix *H, xelem *X)
{
    homo3 r;

    if (!CheckSizeMatrix (H, 4, 4) || !X) {
        Warning ("TransformCoord: no data!"); return;
    }
    r.X = H->m[0][0] * X->X + H->m[0][1] * X->Y + H->m[0][2] * X->Z + H->m[0][3];
    r.Y = H->m[1][0] * X->X + H->m[1][1] * X->Y + H->m[1][2] * X->Z + H->m[1][3];
    r.Z = H->m[2][0] * X->X + H->m[2][1] * X->Y + H->m[2][2] * X->Z + H->m[2][3];
    r.W = H->m[3][0] * X->X + H->m[3][1] * X->Y + H->m[3][2] * X->Z + H->m[3][3];
    if (ISZERO(r.W)) {
        Warning ("TransformCoord failed!"); return;
    }
    X->X = r.X / r.W;
    X->Y = r.Y / r.W;
    X->Z = r.Z / r.W;
}

void TransformCoord (matrix *H, xlist *xl)
{
    xelem *X;

    if (xl) {
        for (X=xl->start; X; X=X->next) {
            TransformCoord (H, X);
        }
    }
}

// ===========================================================================
//
// ===========================================================================

homo3 TransformCoordNew (matrix *H, xelem *X)
{
    homo3 r;

    CheckSizeMatrix(H, 4, 4);

    if (!X) {
        Warning ("TransformCoord: no data!"); return NOHOMO3;
    }
    r.X = H->m[0][0] * X->X + H->m[0][1] * X->Y + H->m[0][2] * X->Z + H->m[0][3];
    r.Y = H->m[1][0] * X->X + H->m[1][1] * X->Y + H->m[1][2] * X->Z + H->m[1][3];
    r.Z = H->m[2][0] * X->X + H->m[2][1] * X->Y + H->m[2][2] * X->Z + H->m[2][3];
    r.W = H->m[3][0] * X->X + H->m[3][1] * X->Y + H->m[3][2] * X->Z + H->m[3][3];
    r.id = X->id;

    return r;
}

homo3 TransformNormCoordNew (matrix *H, xelem *X)
{
    homo3 r;

    r = TransformCoordNew (H, X);
    NormHomo(&r);

    return r;
}

xlist *TransformCoordNew (matrix *H, xlist *xl)
{
    xelem *X;
    xlist *list = NULL;
    homo3 t;

    if (xl) {
        for (X=xl->start; X; X=X->next) {
            t = TransformCoordNew (H, X);
            list = AddCoord(list, &t);
        }
    }
    return list;
}

// ===========================================================================
//
// ===========================================================================

homo3 TransformNormCoord (matrix *H, homo3 *h)
{
    homo3 r;

    r = TransformCoord (H, h);
    NormHomo (&r);
//  r.id = h->id;

    return r;
}

//@}

/** @name Coordinate Statistics */
//@{

//@}


/** @name Coordinate Reorganisation */
//@{

/**
 Convert linked coord list to a compact array.

 @param[in]		xl		Modified coord list.
 @result				True, if conversion was successful.
*/
bool CoordArray (xlist *xl)
{
    xelem *x, *y, *next, *start;

    if (CheckCoords(xl)) {
        if (xl->compact) {
            return true;
        } else {
            start = NewCoord (xl->num);
            if (start) {
                for (x = xl->start, y = start; x; x = next, ++y) {
                    CopyCoord(x, y);
                    y->next = y+1;
                    next = x->next;
                    hfree (x); x = NULL;
                }
                xl->start     = start;
                xl->end       = start + xl->num-1;
                xl->compact   = true;
                xl->end->next = NULL;
                return true;
            }
        }
    }
    return false;
}

/**
 Convert compact array to a linked coord list.

 @param[in]		xl		Modified coord array.
 @result				True, if conversion was successful.
*/
bool CoordList (xlist *xl)
{
    xlist *list = NULL;
    xelem *x;

    if (CheckCoords(xl)) {
        if (!xl->compact) {
            return true;
        } else {
            Warning("LinkedCoord: Convert array to linked list");

            for (x = xl->start; x <= xl->end; ++x) {
                list = AddCoord(list, x);
            }
            hfree (xl->start); // xl->start = NULL;
//			xl->num     = list->num;
            xl->compact = false;
            xl->start   = list->start;
            xl->end     = list->end;
            hfree (list); list = NULL;
            return true;
        }
    }
    return false;
}

/**
 Join two coord lists \f$ \mathbf{c} = \mathbf{xl}(0) \ldots \mathbf{xl}(m-1), \mathbf{yl}(0) \ldots \mathbf{yl}(n-1) \f$.

 @param[in]		xl	Modified first coord list with \a m elements.
 @param[in]		yl	Modified second coord list with \a n elements.
 @result			New allocated coord list \a C with \a m + \a n merged elements or NULL.
 @note				Input lists are empty but the structure is not free.
*/
xlist *JoinCoord (xlist *xl, xlist *yl)
{
    xlist *res;
	long num;

	if (!xl && !yl) return NULL;

	res = NewCoordList();
	if (!res) {
		Warning ("JoinCooord: out of memory"); return NULL;
	}
	if (!xl) {
		SwapCoord(yl, res); return res;
	}
	if (!yl) {
		SwapCoord(xl, res); return res;
	}
	num = xl->num + yl->num;
	if (num < 1) {
		return NULL;
	}
    if (xl->compact) { // Join compact xl and linked yl
		res->start = (xelem *) hrealloc(xl->start, num, sizeof(xelem));
		if (!res->start) {
			Warning ("JoinCoord: out of memory"); FreeCoord(res); return NULL;
		}
		res->end = res->start + num-1;
		res->compact = true;
		res->num = num;
		CopyCoord(yl, res, xl->num);
		RelinkCoord (res);
		hfree(yl->start);
	} else if (yl->compact) { // Join compact yl and linked xl
		res->start = (xelem *) hrealloc(yl->start, num, sizeof(xelem));
		if (!res->start) {
			Warning ("JoinCoord: out of memory"); FreeCoord(res); return NULL;
		}
		res->end = res->start + num-1;
		res->compact = true;
		res->num = num;
		CopyCoord(xl, res, yl->num);
		RelinkCoord (res);
		hfree(xl->start);
	} else { // Join linked xl and yl
		xl->end->next = yl->start; // link lists
		res->start = xl->start;
		res->end = yl->end;
		res->num = xl->num + yl->num;
	}
	xl->num = 0; xl->start = xl->end = NULL; // Clean old lists
	yl->num = 0; yl->start = yl->end = NULL;

//	Warning ("JoinCoord: Function was changed to modify input lists and non-unified IDs ...");
//  res = UnifyID(xl); // ??? here

    return res;
}

/**
 Join multiple coord lists.

 @param[in]		xl	Array of coord lists.
 @param[in]		n	Number of lists.
 @result			New allocated coord list with merged lists or NULL.
*/
xlist *JoinCoordNew (xlist *xl[], int n)
{
    xlist *res = NULL;
	long num = 0;
	int i;

    if (!xl) return NULL;

	for (i=0; i<n; ++i) {
		if (xl[i]) {
			num += xl[i]->num;
		}
	}
	if (num < 1) {
		return NULL;
	}
	res = NewCoordList(num);
	if (!res) {
		Warning ("JoinCoordNew: out of memory"); return NULL;
	}
	res->end = res->start + num-1;
	res->compact = true;
	res->num = num;

	num = 0;
	for (i=0; i<n; ++i) {
		if (xl[i]) {
			CopyCoord(xl[i], res, num);
			num += xl[i]->num;
		}
	}
	RelinkCoord(res);

    return res;
}

/**
 Join two coord lists \f$ \mathbf{x1}(0) \ldots \mathbf{x1}(m-1), \mathbf{x2}(0) \ldots \mathbf{x2}(n-1)\f$.

 @param[in]		x1	First coord list with \a m elements.
 @param[in]		x2	Second coord list with \a n elements.
 @result			New allocated coord list \a C with \a m + \a n merged elements or NULL.
*/
xlist *JoinCoordNew (xlist *x1, xlist *x2)
{
	xlist *xl[2];

	xl[0] = x1; xl[1] = x2;

	return JoinCoordNew (xl, 2);
}

/**
 Join three coord lists \f$ \mathbf{x1}(0) \ldots \mathbf{x1}(m-1), \mathbf{x2}(0) \ldots \mathbf{x2}(n-1), \mathbf{x3}(0) \ldots \mathbf{x3}(o-1)\f$.

 @param[in]		x1	First coord list with \a m elements.
 @param[in]		x2	Second coord list with \a n elements.
 @param[in]		x3	Third corner list with \a o elements.
 @result			New allocated coord list \a C with \a m + \a n + \a o merged elements or NULL.
*/
xlist *JoinCoordNew (xlist *x1, xlist *x2, xlist *x3)
{
	xlist *xl[3];

	xl[0] = x1; xl[1] = x2; xl[2] = x3;

	return JoinCoordNew (xl, 3);
}

/**
 Link elements of a compact coord array.

 @param[in]		xl	Coord list.
*/
void RelinkCoord (xlist *xl)
{
    xelem *x;

    if (CheckCoords(xl) && xl->compact) {
        for (x=xl->start; x<=xl->end; ++x) {
            x->next = x+1;
        }
        xl->end->next = NULL;
    }
}

static int CompareIDx (xelem *x1, xelem *x2)
{
	if (x1->id == x2->id) {
		return (int)((PTRINT)x1->next - (PTRINT)x2->next);
	} else {
		return (int)(x1->id - x2->id);
	}
}

/**
 Sort coord IDs into ascending order without linking \f$ \mathbf{cl}(0) \le \ldots \le \mathbf{cl}(n-1) \f$.

 @param[in,out]	xl	Modified coord list with \a n elements.
*/
void SortCoordUnlinked (xlist *xl)
{
    if (!CheckCoordsSorted(xl)) {
        if (CoordArray(xl)) {          // Convert linked list to compact array
            qsort((char *)xl->start, (unsigned)xl->num, sizeof(xelem), (int (*)(const void *, const void *)) CompareIDx);
	    //qsort((char *)xl->start, (unsigned)xl->num, sizeof(xelem), (int (*)(const xelem *, const xelem *)) CompareID);

        }
    }
}

/**
 Sort coord IDs into ascending order \f$ \mathbf{cl}(0) \le \ldots \le \mathbf{cl}(n-1) \f$.

 @param[in,out]	xl	Modified coord list with \a n elements.
*/
void SortCoord (xlist *xl)
{
	SortCoordUnlinked (xl);
    RelinkCoord (xl);
}

//@}


/** @name Logical Coordinate Operations */
//@{

//@}


/** @name Other Operations */
//@{
// ===========================================================================
//
// ===========================================================================

void PrintCoord (xelem *x)
{
    if (x) {
        Report ("ID             X           Y           Z       R     G     B     Q");
        Report ("--------+-----------+-----------+-----------+-----+-----+-----+--------------");
        Report ("%07ld   %9.3f   %9.3f   %9.3f   %3d   %3d   %3d   %e", x->id, x->X, x->Y, x->Z,
                 GET_R(x->color), GET_G(x->color), GET_B(x->color), x->quality);
    }
}

void PrintCoord (xlist *xl)
{
    char *buf = "unsorted";
    xelem *x;

    if (xl) {
        if (xl->compact) {
            Debug ("Compact coordinate array with %d %s elements", xl->num, (CheckCoordsSorted(xl) ? buf+2 : buf));
        } else {
            Debug ("Linked coordinate list with %d %s elements", xl->num, (CheckCoordsSorted(xl) ? buf+2 : buf));
        }
        Report ("ID             X           Y           Z       R     G     B     Q");
        Report ("--------+-----------+-----------+-----------+-----+-----+-----+--------------");
		if (xl->compact) {
			for (x=xl->start; x<=xl->end; ++x) {
				Report ("%07ld   %9.3f   %9.3f   %9.3f   %3d   %3d   %3d   %e", x->id, x->X, x->Y, x->Z,
						 GET_R(x->color), GET_G(x->color), GET_B(x->color), x->quality);
			}
		} else {
			for (x=xl->start; x; x=x->next) {
				Report ("%07ld   %9.3f   %9.3f   %9.3f   %3d   %3d   %3d   %e", x->id, x->X, x->Y, x->Z,
						 GET_R(x->color), GET_G(x->color), GET_B(x->color), x->quality);
			}
		}
    }
}

//FILE
void
WritePBM (char *name, image *pic)
{
    FILE *fp;
    int i, w, h, typ;

    if (!CheckImage(pic)) {
        return;
    }
    w = pic->w; h = pic->h; typ = pic->typ;

    if ((fp = fopen (name, "wb")) == NULL) {
        ErrorExit (OPEN_ERROR, name);
    }
    if (pic->typ == GRAY_IMAGE) {
        Report ("\tGrauwertbild '%s' speichern...", name);
        fprintf (fp, "P5\n");
    }
    if (pic->typ == COLOR_IMAGE) {
        Report ("\tFarbbild '%s' speichern...", name);
        fprintf (fp, "P6\n");
    }
	WriteTags (fp, pic->tagc, pic->tagv);

	fprintf (fp, "%d %d %d\n", w, h, MAX_GRAY);
    for (i=0; i<h; ++i) {
        if (fwrite (pic->d + i*w*typ, typ * sizeof(byte), w, fp) != (uint)w) {
            fclose (fp); ErrorExit (WRITE_ERROR, name);
        }
    }
    fclose (fp);
}


// ===========================================================================
//
// ===========================================================================

void
WriteCorners (char *name, cparam *par, image *pic, clist *cl)
{
    FILE *fp;
    celem *c;

    if (CheckCorners(cl)) {
        Debug ("Punktliste '%s' speichern...", name);

        if ((fp = fopen (name, "wb")) == NULL) {
            ErrorExit (OPEN_ERROR, name);
        }
        fprintf (fp, "# ============================================\n");
        fprintf (fp, "#             C O R N E R  L I S T\n");
        fprintf (fp, "# ============================================\n#\n");
        fprintf (fp, "#   Image name:    %s\n", pic->name);
        fprintf (fp, "#   Image size:    %d x %d\n", pic->w, pic->h);
        fprintf (fp, "#   Corner Method: %s with %s tensor\n", (par->method == HARRIS) ? "Harris" : ((par->method == FOERSTNER) ? "Foerstner" : "Foerstner-KLT"), (par->tensor == STRUCTURE) ? "structure" : ((par->tensor == BOUNDARY) ? "boundary" : "energy" ));
		fprintf (fp, "#   Accumulation:  %s with radius %d\n", (par->accu == UNIFORM) ? "Uniform" : ((par->accu == GAUSSIAN) ? "Gaussian" : "Hourglass" ), par->radius2);
        fprintf (fp, "#   Convolution:   Sigma  = %lf\n", par->sigma);
        fprintf (fp, "#   Cornerness:    Weight = %lf\n", par->weight);
        fprintf (fp, "#   Form factor:   Round  = %lf\n", par->round);
        fprintf (fp, "#   Subpixel:      %s\n", par->subpix ? "Yes" : "No");
        if (par->adapt) {
            fprintf (fp, "#   Adaptive:      Region = %d x %d\n", par->adapt, par->adapt);
        } else {
            fprintf (fp, "#   Adaptive:      No\n");
        }
        if (par->pyr.num > 1) {
			fprintf (fp, "#   Image pyramid: Resolution level:   %d\n", par->pyr.level);
			fprintf (fp, "#                  Scaling factor:     %lf\n", par->pyr.factor);
            fprintf (fp, "#                  Smoothing:  Sigma = %lf\n", par->pyr.sigma);
        } else {
            fprintf (fp, "#   Image pyramid: No\n");
        }
		if (cl->start && ISZERO(cl->start->cxx) || ISZERO(cl->start->cyy)) {
			fprintf (fp, "#\n#            Position      Weight\n");
			fprintf (fp, "# ID       ( X       Y )\n");
			fprintf (fp, "# -------------------------------\n#\n");
			for (c=cl->start; c; c=c->next) {
				fprintf (fp, "%06ld   %7.2f %7.2f   %.3f\n", c->id, c->x, c->y, c->value);
			}
		} else {
			fprintf (fp, "#\n#            Position      Weight             Covariances\n");
			fprintf (fp, "# ID       ( X       Y )           (  XX          XY         YY    )\n");
			fprintf (fp, "# ------------------------------------------------------------------\n#\n");
			for (c=cl->start; c; c=c->next) {
				fprintf (fp, "%06ld   %7.2f %7.2f   %.3f   %.3e % .3e %.3e\n", c->id, c->x, c->y, c->value, c->cxx, c->cxy, c->cyy);
			}
		}
        fclose (fp);
    }
}

// ===========================================================================
//
// ===========================================================================

void
WriteMatches (char *name, plist *pl1, plist *pl2)
{
    FILE *fp;
    pelem *p1, *p2;

    if (pl1 && pl2) {
        Debug ("Matchliste '%s' speichern...", name);

        if ((fp = fopen (name, "wb")) == NULL) {
            ErrorExit (OPEN_ERROR, name);
        }
        for (p1=pl1->start, p2=pl2->start; p1 && p2; p1=p1->next, p2=p2->next) {
            fprintf (fp, "%06ld   %7.2f %7.2f   %7.2f %7.2f\n", p1->id, p1->x, p1->y, p2->x, p2->y);
        }
        fclose (fp);
    }
}

// ===========================================================================
//
// ===========================================================================

void
WritePoints (char *name, plist *pl, asciityp t, int skip)
{
    FILE *fp;
    pelem *p;
	long i;

    if (pl) {
		skip = MAX(skip, 1);
        if ((fp = fopen (name, "wb")) == NULL) {
            ErrorExit (OPEN_ERROR, name);
        }
		switch (t) {
			case XY:	Report ("\tPunktliste '%s' im %s-Format speichern...", name, pointformat[1]);
				        for (i=0, p=pl->start; p; p=p->next, ++i) {
							if (!(i%skip)) fprintf (fp, "%e %e\n", p->x, p->y);
						}
						break;
			case NXY:	Report ("\tPunktliste '%s' im %s-Format speichern...", name, pointformat[2]);
				        for (i=0, p=pl->start; p; p=p->next, ++i) {
							if (!(i%skip)) fprintf (fp, "%09ld   %e %e\n", p->id, p->x, p->y);
						}
						break;
			case NXYQ:	Report ("\tPunktliste '%s' im %s-Format speichern...", name, pointformat[3]);
				        for (i=0, p=pl->start; p; p=p->next, ++i) {
							if (!(i%skip)) fprintf (fp, "%09ld   %e %e   %e\n", p->id, p->x, p->y, p->quality);
						}
						break;
			case XYRGB:	Report ("\tPunktliste '%s' im %s-Format speichern...", name, pointformat[4]);
				        for (i=0, p=pl->start; p; p=p->next, ++i) {
							if (!(i%skip)) fprintf (fp, "%e %e   %3d %3d %3d\n", p->x, p->y, GET_R(p->color), GET_G(p->color), GET_B(p->color));
						}
						break;
			case NXYRGB: Report ("\tPunktliste '%s' im %s-Format speichern...", name, pointformat[5]);
				        for (i=0, p=pl->start; p; p=p->next, ++i) {
							if (!(i%skip)) fprintf (fp, "%09ld   %e %e   %3d %3d %3d\n", p->id, p->x, p->y, GET_R(p->color), GET_G(p->color), GET_B(p->color));
						}
						break;
			case NXYRGBQ: Report ("\tPunktliste '%s' im %s-Format speichern...", name, pointformat[6]);
				        for (i=0, p=pl->start; p; p=p->next, ++i) {
							if (!(i%skip)) fprintf (fp, "%09ld   %e %e   %3d %3d %3d   %e\n", p->id, p->x, p->y, GET_R(p->color), GET_G(p->color), GET_B(p->color), p->quality);
						}
						break;
			case ROMA:  Report ("\tPunktliste '%s' im %s-Format speichern...", name, pointformat[7]);
				        for (i=0, p=pl->start; p; p=p->next, ++i) {
							if (!(i%skip)) fprintf (fp, "%09ld   %e %e 0.0   0.0 0.0 0.0   0 0\n", p->id, p->x, p->y);
						}
		}
        fclose (fp);

		Debug("%ld Bildpunkte gespeichert", pl->num);
    }
}

void
WritePoints (char *name, plist *pl, int skip)
{
    pelem *p;
	long i;
	bool n = false, q = false, cols = false;

    for (i=0, p=pl->start; p; p=p->next, ++i) {
		if (p->id != i)              n   = true;
		if (p->color != WHITE_COLOR) cols = true;
		if (ISINEQUAL(p->quality,1.0))       q   = true;
	}
	if (cols && q) {
		WritePoints (name, pl, NXYRGBQ, skip);
	} else if (cols) {
		if (n) {
			WritePoints (name, pl, NXYRGB, skip);
		} else {
			WritePoints (name, pl, XYRGB, skip);
		}
	} else if (q) {
		WritePoints (name, pl, NXYQ, skip);
	} else {
		if (n) {
			WritePoints (name, pl, NXY, skip);
		} else {
			WritePoints (name, pl, XY, skip);
		}
	}
}

void
WritePoints (char *name, plist *pl, asciityp t)
{
	WritePoints (name, pl, t, 1);
}

void
WritePoints (char *name, plist *pl)
{
	WritePoints (name, pl, 1);
}

// ===========================================================================
//
// ===========================================================================

void
WriteCoord (char *name, xlist *xl, asciityp t, int skip)
{
    FILE *fp;
    xelem *x;
	long i;

    if (xl) {
		skip = MAX(skip, 1);
        if ((fp = fopen (name, "wb")) == NULL) {
            ErrorExit (OPEN_ERROR, name);
        }
		switch (t) {
			case XYZ:	Report ("\tKoordinatenliste '%s' im %s-Format speichern...", name, coordformat[1]);
				        for (i=0, x=xl->start; x; x=x->next, ++i) {
							if (!(i%skip)) fprintf (fp, "%e %e %e\n", x->X, x->Y, x->Z);
						}
						break;
			case NXYZ:	Report ("\tKoordinatenliste '%s' im %s-Format speichern...", name, coordformat[2]);
				        for (i=0, x=xl->start; x; x=x->next, ++i) {
							if (!(i%skip)) fprintf (fp, "%09ld   %e %e %e\n", x->id, x->X, x->Y, x->Z);
						}
						break;
			case NXYZQ:	Report ("\tKoordinatenliste '%s' im %s-Format speichern...", name, coordformat[3]);
				        for (i=0, x=xl->start; x; x=x->next, ++i) {
							if (!(i%skip)) fprintf (fp, "%09ld   %e %e %e   %e\n", x->id, x->X, x->Y, x->Z, x->quality);
						}
						break;
			case XYZRGB: Report ("\tKoordinatenliste '%s' im %s-Format speichern...", name, coordformat[4]);
				        for (i=0, x=xl->start; x; x=x->next, ++i) {
							if (!(i%skip)) fprintf (fp, "%e %e %e  %3d %3d %3d\n", x->X, x->Y, x->Z, GET_R(x->color), GET_G(x->color), GET_B(x->color));
						}
						break;
			case NXYZRGB: Report ("\tKoordinatenliste '%s' im %s-Format speichern...", name, coordformat[5]);
				        for (i=0, x=xl->start; x; x=x->next, ++i) {
							if (!(i%skip)) fprintf (fp, "%09ld   %e %e %e   %3d %3d %3d\n", x->id, x->X, x->Y, x->Z, GET_R(x->color), GET_G(x->color), GET_B(x->color));
						}
						break;
			case NXYZRGBQ: Report ("\tKoordinatenliste '%s' im %s-Format speichern...", name, coordformat[6]);
				        for (i=0, x=xl->start; x; x=x->next, ++i) {
							if (!(i%skip)) fprintf (fp, "%09ld   %e %e %e   %3d %3d %3d   %e\n", x->id, x->X, x->Y, x->Z, GET_R(x->color), GET_G(x->color), GET_B(x->color), x->quality);
						}
						break;
			case ROMA:  Report ("\tKoordinatenliste '%s' im %s-Format speichern...", name, coordformat[7]);
				        for (i=0, x=xl->start; x; x=x->next, ++i) {
							if (!(i%skip)) fprintf (fp, "%09ld   %e %e %e   0.0 0.0 0.0   0 0\n", x->id, x->X, x->Y, x->Z);
						}
		}
        fclose (fp);

		Debug("%ld Objektpunkte gespeichert", xl->num);
    }
}

void
WriteCoord (char *name, xlist *xl, int skip)
{
    xelem *x;
	long i;
	bool n = false, q = false, cols = false;

    for (i=0, x=xl->start; x; x=x->next, ++i) {
		if (x->id != i)              n   = true;
		if (x->color != WHITE_COLOR) cols = true;
		if (ISINEQUAL(x->quality,1.0))       q   = true;
	}
	if (cols && q) {
		WriteCoord (name, xl, NXYZRGBQ, skip);
	} else if (cols) {
		if (n) {
			WriteCoord (name, xl, NXYZRGB, skip);
		} else {
			WriteCoord (name, xl, XYZRGB, skip);
		}
	} else if (q) {
		WriteCoord (name, xl, NXYZQ, skip);
	} else {
		if (n) {
			WriteCoord (name, xl, NXYZ, skip);
		} else {
			WriteCoord (name, xl, XYZ, skip);
		}
	}
}

void
WriteCoord (char *name, xlist *xl, asciityp t)
{
	WriteCoord (name, xl, t, 1);
}

void
WriteCoord (char *name, xlist *xl)
{
	WriteCoord (name, xl, 1);
}


// ===========================================================================
//
// ===========================================================================

static void
WriteMatrixContent (FILE *fp, matrix *M)
{
    int i, j;

    CheckMatrix (M);

    fprintf (fp, "%d %d\n\n", M->rows, M->cols);
    for (j=0; j<M->rows; ++j) {
        for (i=0; i<M->cols; ++i) {
            fprintf (fp, "%13.5e ", ISNONZERO(M->m[j][i]) ? M->m[j][i] : 0.0);
        }
        fprintf (fp, "\n");
    }
}

// ===========================================================================
//
// ===========================================================================

static void
WriteMatrixContentMatlab (FILE *fp, char *label, matrix *M)
{
    int i, j;

    CheckMatrix (M);

    fprintf (fp, "%s=", label);
    //first rows with opening bracket
    j=0;
    fprintf (fp, "[ ");
    for (i=0; i<M->cols; ++i) {
        fprintf (fp, "%20.17e ", ISNONZERO(M->m[j][i]) ? M->m[j][i] : 0.0);
    }
    fprintf (fp, ";\n");

    for (j=1; j<M->rows-1; ++j) {
        fprintf (fp, "  ");
        for (i=0; i<M->cols; ++i) {
            fprintf (fp, "%20.17e ", ISNONZERO(M->m[j][i]) ? M->m[j][i] : 0.0);
        }
        fprintf (fp, ";\n");
    }
    //last rows with closing bracket
    j=M->rows-1;
    fprintf (fp, "  ");
    for (i=0; i<M->cols; ++i) {
        fprintf (fp, "%20.17e ", ISNONZERO(M->m[j][i]) ? M->m[j][i] : 0.0);
    }
    fprintf (fp, "]\n");

}

// ===========================================================================
//
// ==========================================================================

void
WriteMatrix (char *name, matrix *M)
{
    FILE *fp;

    Debug ("Matrix '%s' schreiben...", name);

    if ((fp = fopen (name, "wb")) == NULL) {
        ErrorExit (OPEN_ERROR, name);
    }
	WriteMatrixContent (fp, M);

	fclose(fp);
}

// ===========================================================================
//
// ==========================================================================

void
WriteMatrixMatlab (char *name, char *label, matrix *M)
{
    FILE *fp;

    Debug ("Matrix '%s' schreiben...", name);

    if ((fp = fopen (name, "wb")) == NULL) {
        ErrorExit (OPEN_ERROR, name);
    }
	WriteMatrixContentMatlab (fp, label, M);

	fclose(fp);
}

// ===========================================================================
//
// ==========================================================================

void
WriteMesh (char *name, mesh *M)
{
    FILE *fp;
	int i, j;

    Debug ("Write mesh '%s' ...", name);

    if ((fp = fopen (name, "wb")) == NULL) {
        ErrorExit (OPEN_ERROR, name);
    }
    fprintf (fp, "%d %d\n\n", M->rows, M->cols);
    for (j=0; j<M->rows; ++j) {
        for (i=0; i<M->cols; ++i) {
			if (M->m[j][i]) {
				fprintf (fp, "%d ", M->m[j][i]->id);
			} else {
				fprintf (fp, "0 ");
			}
        }
        fprintf (fp, "\n");
    }
	fclose(fp);
}

// ===========================================================================
//
// ==========================================================================

void
WriteWaveFront (char *name, mesh *M)
{
	xelem *a, *b, *c, *d;
    FILE *fp;
	int i, j;
	long quad = 0, tri = 0;
    char *buf;

	if (CheckMesh(M)) {
		if (!M->clean) {
			mesh_clean (M);
		}
		buf = FilenameExtension(name, "mtl");
		Debug ("Write WaveFront material '%s' ...", buf);
		if ((fp = fopen (buf, "wb")) == NULL) {
			ErrorExit (OPEN_ERROR, name);
		}
		hfree(buf);

		fprintf (fp, "# WaveFront *.mtl ASCI file (generated by %s)\n\n", AUTHOR);
        fprintf (fp, "newmtl texture\n\tKa 0.5 0.5 0.5\n\tKd 0.5 0.5 0.5\n\tKs 0.5 0.5 0.5\n\tTf 0.5 0.5 0.5\n");
		buf = BaseFilenameExtension(name, "ppm");
        fprintf (fp, "\tillum 1\n\td 1\n\tNs 6\n\tsharpness 60\n\tNi 1\n\tmap_Kd %s\n\n", buf);
		hfree(buf);
		fclose(fp);

		buf = FilenameExtension(name, "ppm");
		Debug ("Place the texture map in '%s'!", buf);
		hfree(buf);

		Debug ("Write WaveFront object '%s' ...", name);
		if ((fp = fopen (name, "wb")) == NULL) {
			ErrorExit (OPEN_ERROR, name);
		}
		fprintf (fp, "# WaveFront *.obj ASCI file (generated by %s)\n\n", AUTHOR);
		buf = BaseFilenameExtension(name, "mtl");
		fprintf (fp, "mtllib %s\n", buf);
		hfree(buf);
		fprintf (fp, "g %s\nusemtl texture\n", NULL);
		for (j=1; j<M->rows; ++j) {
			for (i=1; i<M->cols; ++i) {
				a = M->m[j-1][i-1];            // a - b
				b = M->m[j-1][i];              // |   |
				c = M->m[j][i-1];              // c - d
				d = M->m[j][i];
				if (a && b && c && d) {
				    fprintf (fp, "v %f %f %f\n", a->X, a->Y, a->Z);
				    fprintf (fp, "v %f %f %f\n", b->X, b->Y, b->Z);
				    fprintf (fp, "v %f %f %f\n", d->X, d->Y, d->Z);
				    fprintf (fp, "v %f %f %f\n", c->X, c->Y, c->Z);
				    fprintf (fp, "vt %f %f\n", (double)(i-1)/M->cols, (double)(j-1)/M->rows);
				    fprintf (fp, "vt %f %f\n", (double)i/M->cols, (double)(j-1)/M->rows);
				    fprintf (fp, "vt %f %f\n", (double)i/M->cols, (double)j/M->rows);
				    fprintf (fp, "vt %f %f\n", (double)(i-1)/M->cols, (double)j/M->rows);
					fprintf (fp, "f -4/-4 -3/-3 -2/-2 -1/-1\n\n"); // a b d c
					quad++;
				} else if (b && c && d) {
				    fprintf (fp, "v %f %f %f\n", b->X, b->Y, b->Z);
				    fprintf (fp, "v %f %f %f\n", d->X, d->Y, d->Z);
				    fprintf (fp, "v %f %f %f\n", c->X, c->Y, c->Z);
				    fprintf (fp, "vt %f %f\n", (double)i/M->cols, (double)(j-1)/M->rows);
				    fprintf (fp, "vt %f %f\n", (double)i/M->cols, (double)j/M->rows);
				    fprintf (fp, "vt %f %f\n", (double)(i-1)/M->cols, (double)j/M->rows);
					fprintf (fp, "f -3/-3 -2/-2 -1/-1\n\n"); // b d c
					tri++;
				} else if (a && c && d) {
				    fprintf (fp, "v %f %f %f\n", a->X, a->Y, a->Z);
				    fprintf (fp, "v %f %f %f\n", d->X, d->Y, d->Z);
				    fprintf (fp, "v %f %f %f\n", c->X, c->Y, c->Z);
				    fprintf (fp, "vt %f %f\n", (double)(i-1)/M->cols, (double)(j-1)/M->rows);
				    fprintf (fp, "vt %f %f\n", (double)i/M->cols, (double)j/M->rows);
				    fprintf (fp, "vt %f %f\n", (double)(i-1)/M->cols, (double)j/M->rows);
					fprintf (fp, "f -3/-3 -2/-2 -1/-1\n\n"); // a d c
					tri++;
				} else if (a && b && d) {
				    fprintf (fp, "v %f %f %f\n", a->X, a->Y, a->Z);
				    fprintf (fp, "v %f %f %f\n", b->X, b->Y, b->Z);
				    fprintf (fp, "v %f %f %f\n", d->X, d->Y, d->Z);
				    fprintf (fp, "vt %f %f\n", (double)(i-1)/M->cols, (double)(j-1)/M->rows);
				    fprintf (fp, "vt %f %f\n", (double)i/M->cols, (double)(j-1)/M->rows);
				    fprintf (fp, "vt %f %f\n", (double)i/M->cols, (double)j/M->rows);
					fprintf (fp, "f -3/-3 -2/-2 -1/-1\n\n"); // a b d
					tri++;
				} else if (a && b && c) {
				    fprintf (fp, "v %f %f %f\n", a->X, a->Y, a->Z);
				    fprintf (fp, "v %f %f %f\n", b->X, b->Y, b->Z);
				    fprintf (fp, "v %f %f %f\n", c->X, c->Y, c->Z);
				    fprintf (fp, "vt %f %f\n", (double)(i-1)/M->cols, (double)(j-1)/M->rows);
				    fprintf (fp, "vt %f %f\n", (double)i/M->cols, (double)(j-1)/M->rows);
				    fprintf (fp, "vt %f %f\n", (double)(i-1)/M->cols, (double)j/M->rows);
					fprintf (fp, "f -3/-3 -2/-2 -1/-1\n\n"); // a b c
					tri++;
				}
			}
		}
		fprintf (fp, "# %d squares and %d triangles.\n", quad, tri);

		Debug ("Completed %d squares and %d triangles.", quad, tri);

		fclose(fp);
	}
}

// ===========================================================================
//
// ===========================================================================

void
WriteCube (char *name, cube *C)
{
    int i,j,k;
    FILE *fp;

    Debug ("Cube '%s' schreiben...", name);

    CheckCube (C);

    if ((fp = fopen (name, "wb")) == NULL) {
        ErrorExit (OPEN_ERROR, name);
    }
    fprintf (fp, "%d %d %d \n\n", C->dep, C->rows, C->cols);
    for (k=0; k<C->dep; ++k) {
        for (j=0; j<C->rows; ++j) {
            for (i=0; i<C->cols; ++i) {
                fprintf (fp, "%13.5e ", ISNONZERO(C->c[k][j][i]) ? C->c[k][j][i] : 0.0);
            }
            fprintf (fp, "\n");
        }
        fprintf (fp, "\n");
    }
	WriteMatrixContent (fp, C->P[0]);
	WriteMatrixContent (fp, C->P[1]);
	WriteMatrixContent (fp, C->P[2]);

	fclose (fp);
}

// ===========================================================================
//
// ===========================================================================

bool
ReadNewLine (FILE *fp)
{
	bool w = false;
    int c;

    do {
         if ((c = fgetc (fp)) < 0) return w;
         if ((c != 9) && (c != 10) && (c != 13) && (c != 32)) w = true; // Whitespace tab, cr, lf, space
    } while (c != '\n');

	return w;
}

// ===========================================================================
//
// ===========================================================================

int
ReadString (FILE *fp, char *buf)
{
    int c;
    char *p;

    do {
        if (fscanf (fp, "%s", buf) < 1) {
            return 0;
        }
        if ((p = strchr (buf, '#')) != NULL) {
            *p = '\0';
            if (report == VERBOSE) Dump ("DEBUG:  Kommentar: %s", p+1);
            do {
                if ((c = fgetc (fp)) < 0) {
                    return 0;
                }
                if (report == VERBOSE) Dump ("%c", c);
            } while (c != '\n');
        }
    } while (buf[0] == '\0');

    return 1;
}

// ===========================================================================
//
// ===========================================================================

int ReadColNum (FILE *fp)
{
    char buf[BUF_SIZE];
	int i, cols = 0;
	output quiet;
	bool w;

	quiet = report; report = QUIET;
	do {
		 fseek (fp, 0, SEEK_SET);
		 for (i=0; i<cols; ++i) {
			 if (!ReadString (fp, buf)) {
				 fseek (fp, 0, SEEK_SET);
				 report = quiet;
				 return cols;
			 }
		 }
       	 w = ReadNewLine (fp);
		 if (w) ++cols;
    } while (w);

	fseek (fp, 0, SEEK_SET);
	report = quiet;

	return cols;
}

// ===========================================================================
//
// ===========================================================================

static matrix *
ReadMatrixContent (FILE *fp)
{
    char buf[BUF_SIZE];
    matrix *M;
    int i, j, r, c;

    ReadString (fp, buf); r = atoi(buf);
    ReadString (fp, buf); c = atoi(buf);

    M = mat_alloc (r, c);
    Debug ("Groesse: %d x %d", r, c);

    for (j=0; j<r; ++j) {
        for (i=0; i<c; ++i) {
            if (ReadString (fp, buf)) {
                M->m[j][i] = atof(buf);
            } else {
                Warning ("ReadMatrix: Zu wenig Elemente!");
            }
        }
    }
    return M;
}

// ===========================================================================
//
// ==========================================================================

matrix *
ReadMatrix (char *name)
{
    matrix *M;
    FILE *fp;

    Debug ("Matrix '%s' lesen...", name);

    if(!CheckFile(name, READ)){
        ErrorExit (OPEN_ERROR, name);
    }
    fp = fopen (name, "rb");

	M = ReadMatrixContent (fp);
	fclose (fp);

    return M;
}

// ===========================================================================
//
// ==========================================================================

mesh *ReadMesh (char *name)
{
    char buf[BUF_SIZE];
    int i, j, r, c;
    FILE *fp;
    mesh *M;
	long id;

    Debug ("Read mesh '%s' ...", name);

    if(!CheckFile(name, READ)){
        ErrorExit (OPEN_ERROR, name);
    }
    fp = fopen (name, "rb");

    ReadString (fp, buf); r = atoi(buf);
    ReadString (fp, buf); c = atoi(buf);

    M = mesh_alloc (r, c);
    Debug ("Size: %d x %d", r, c);

    for (j=0; j<r; ++j) {
        for (i=0; i<c; ++i) {
            if (ReadString (fp, buf)) {
				id = atoi(buf);
				if (id) {
					mesh_setid(M, j, i, id);
				}
            } else {
                Warning ("ReadMesh: Zu wenig Elemente!");
            }
        }
    }
	fclose (fp);
	if (!CheckMeshSorted(M)) {
        Warning ("ReadMesh: nodes are not sorted!");
	}
    return M;
}


// ===========================================================================
//
// ===========================================================================

void
ReadMatches (char *name, plist **pl1, plist **pl2)
{
    FILE *fp;
    char buf[BUF_SIZE];
    double x1, y1, x2, y2;
    int c, cols=5;
    long id, d=0;

    Debug ("Matchliste '%s' einlesen...", name);

    if(!CheckFile(name, READ)){
        ErrorExit (OPEN_ERROR, name);
    }
    fp = fopen (name, "rb");

    *pl1 = *pl2 = NULL;
    do {
        c  = ReadString (fp, buf); id = atoi (buf);
        c += ReadString (fp, buf); x1 = atof (buf);
        c += ReadString (fp, buf); y1 = atof (buf);
        c += ReadString (fp, buf); x2 = atof (buf);
        c += ReadString (fp, buf); y2 = atof (buf);
        if (c == cols) {
            d++;
			*pl1 = AddPoint (*pl1, x1, y1, id);
			*pl2 = AddPoint (*pl2, x2, y2, id);
        } else if (c > 0) {
            Warning ("Syntaxfehler in %s", name);
        }
    } while (c == cols);

    Debug ("%d Punktkorrespondenzen eingelesen", d);

    fclose (fp);
}

// ===========================================================================
//
// ===========================================================================

image *
ReadPBM (char *name)
{
    FILE *fp;
    image *pic = NULL;
    char s[2], buf[BUF_SIZE];
    int c, i, l, w, h, typ;

    Report ("\tPBM Bild '%s' einlesen ...", name);

    if(!CheckFile(name, READ)){
        ErrorExit (OPEN_ERROR, name);
    }
    fp = fopen (name, "rb");

    if (fread (s, sizeof(byte), 2, fp) != 2) {
        fclose (fp); ErrorExit (READ_ERROR, name);
    }
    if (s[0] != 'P') {
        fclose (fp); ErrorExit (PBM_ERROR, name);
    }
    ReadString (fp, buf); w = atoi(buf);
    ReadString (fp, buf); h = atoi(buf);

    Debug ("Groesse: %d x %d Pixel", w, h);

    ReadString (fp, buf); c = atoi(buf);
    if ((c != 255) && report) {
        Warning ("Maxval = %d", c);
    }
    while (fgetc (fp) != '\n');

    if (s[1] == '5') {
        Debug ("Grauwertdaten lesen...");
        pic = AllocImage (w, h, GRAY_IMAGE, name);
    } else if (s[1] == '6') {
        Debug ("RGB Farbdaten lesen...");
        pic = AllocImage (w, h, COLOR_IMAGE, name);
    } else {
        fclose (fp); ErrorExit (PBM_ERROR, name);
    }
	typ = pic->typ;
    for (i = 0; i < h; ++i) {
        l = (int)fread (pic->d + (i*w*typ), typ * sizeof(byte), w, fp);
        if (l != w) {
            Warning ("Nur %d Pixel in Zeile %d", l, i);
            fclose (fp); return pic;
        }
    }
    fclose (fp);

	ReadTags (name, &pic->tagc, &pic->tagv);

    return pic;
}

// ===========================================================================
//
// ===========================================================================

// ===========================================================================
//   ASCII - Point Formats:
//     XY
//     N-XY
//     N-XY-Q
//     XY-RGB
//     N-XY-RGB
//     N-XY-RGB-Q
//     ROMA-BKO
// ===========================================================================

void
ReadPoints (char *name, plist **pl)
{
    FILE *fp;
    char buf[BUF_SIZE];
    double x, y, q;
    int r, g, b, c, cols;
    long id, d=0;
	bool w = false;

    if(!CheckFile(name, READ)){
        ErrorExit (OPEN_ERROR, name);
    }
    fp = fopen (name, "rb");

    cols = ReadColNum(fp);
	if (cols == 9) {
		Report ("\tBildpunkte '%s' im %s-Format einlesen ...", name, pointformat[7]);
	} else {
		if ((cols < 2) || (cols > 7)) {
			cols = 0;
		}
		Report ("\tBildpunkte '%s' im %s-Format einlesen ...", name, pointformat[cols-1]);
	}
    *pl = NULL;

	switch (cols) {
		case 2:    do { // XY
						c  = ReadString (fp, buf); x = atof (buf);
						c += ReadString (fp, buf); y = atof (buf);
       					w |= ReadNewLine (fp);

						if (c == cols) {
							d++; *pl = AddPoint (*pl, x, y, d);
						} else if (c > 0) {
							Warning ("Syntaxfehler in %s", name);
						}
					} while (c == cols);
					break;
		case 3:		do { // N-XY
						c  = ReadString (fp, buf); id = atol (buf);
						c += ReadString (fp, buf); x = atof (buf);
						c += ReadString (fp, buf); y = atof (buf);
       					w |= ReadNewLine (fp);

						if (c == cols) {
							d++; *pl = AddPoint (*pl, x, y, id);
						} else if (c > 0) {
							Warning ("Syntaxfehler in %s", name);
						}
					} while (c == cols);
					break;
		case 4:		do { // N-XY-Q
						c  = ReadString (fp, buf); id = atol (buf);
						c += ReadString (fp, buf); x = atof (buf);
						c += ReadString (fp, buf); y = atof (buf);
						c += ReadString (fp, buf); q = atoi (buf);
       					w |= ReadNewLine (fp);

						if (c == cols) {
							d++; *pl = AddPoint (*pl, x, y, id, q);
						} else if (c > 0) {
							Warning ("Syntaxfehler in %s", name);
						}
					} while (c == cols);
					break;
		case 5:		do { // XY-RGB
						c  = ReadString (fp, buf); x = atof (buf);
						c += ReadString (fp, buf); y = atof (buf);
						c += ReadString (fp, buf); r = atoi (buf);
						c += ReadString (fp, buf); g = atoi (buf);
						c += ReadString (fp, buf); b = atoi (buf);
       					w |= ReadNewLine (fp);

						if (c == cols) {
							d++; *pl = AddPoint (*pl, x, y, d, COLOR(r, g, b));
						} else if (c > 0) {
							Warning ("Syntaxfehler in %s", name);
						}
					} while (c == cols);
					break;
		case 6:		do { // N-XY-RGB
						c  = ReadString (fp, buf); id = atol (buf);
						c += ReadString (fp, buf); x = atof (buf);
						c += ReadString (fp, buf); y = atof (buf);
						c += ReadString (fp, buf); r = atoi (buf);
						c += ReadString (fp, buf); g = atoi (buf);
						c += ReadString (fp, buf); b = atoi (buf);
       					w |= ReadNewLine (fp);

						if (c == cols) {
							d++; *pl = AddPoint (*pl, x, y, id, COLOR(r, g, b));
						} else if (c > 0) {
							Warning ("Syntaxfehler in %s", name);
						}
					} while (c == cols);
					break;
		case 7:		do { // N-XY-RGB-Q
						c  = ReadString (fp, buf); id = atol (buf);
						c += ReadString (fp, buf); x = atof (buf);
						c += ReadString (fp, buf); y = atof (buf);
						c += ReadString (fp, buf); r = atoi (buf);
						c += ReadString (fp, buf); g = atoi (buf);
						c += ReadString (fp, buf); b = atoi (buf);
						c += ReadString (fp, buf); q = atof (buf);
       					w |= ReadNewLine (fp);

						if (c == cols) {
							d++; *pl = AddPoint (*pl, x, y, id, COLOR(r, g, b), q);
						} else if (c > 0) {
							Warning ("Syntaxfehler in %s", name);
						}
					} while (c == cols);
					break;
		case 9:		do { // ROMA-BKO
						c  = ReadString (fp, buf); id = atol (buf);
						c += ReadString (fp, buf); x = atof (buf);
						c += ReadString (fp, buf); y = atof (buf);
						c += ReadString (fp, buf);
						c += ReadString (fp, buf);
						c += ReadString (fp, buf);
						c += ReadString (fp, buf);
						c += ReadString (fp, buf);
						c += ReadString (fp, buf);
       					w |= ReadNewLine (fp);

						if (c == cols) {
							d++; *pl = AddPoint (*pl, x, y, id);
						} else if (c > 0) {
							Warning ("Syntaxfehler in %s", name);
						}
					} while (c == cols);
	}
	if (w == true) Warning ("Spalten ignoriert!");
    Debug ("%ld Bildpunkte eingelesen", d);

    fclose (fp);
}

// ===========================================================================
//   ASCII - Coord Formats:
//     XYZ
//     N-XYZ
//     N-XYZ-Q
//     XYZ-RGB
//     N-XYZ-RGB
//     N-XYZ-RGB-Q
//     ROMA-PKO
// ===========================================================================

void
ReadCoord (char *name, xlist **xl)
{
    FILE *fp;
    char buf[BUF_SIZE];
    double x, y, z, q;
    int c, r, g, b, cols;
    long id, d=0;
	bool w = false;

    if(!CheckFile(name, READ)){
        ErrorExit (OPEN_ERROR, name);
    }
    fp = fopen (name, "rb");

    cols = ReadColNum(fp);
	if ((cols < 3) || (cols > 9)) cols = 0;

    Report ("\tObjektpunkte '%s' im %s-Format einlesen ...", name, coordformat[cols-2]);

    *xl = NULL;

	switch (cols) {
		case 3: 	do { // XYZ
						c  = ReadString (fp, buf); x = atof (buf);
						c += ReadString (fp, buf); y = atof (buf);
						c += ReadString (fp, buf); z = atof (buf);
       					w |= ReadNewLine (fp);
						if (c == cols) {
							d++; *xl = AddCoord (*xl, x, y, z, d);
						} else if (c > 0) {
							Warning ("Syntaxfehler in %s", name);
						}
					} while (c == cols);
					break;
		case 4: 	do { // N-XYZ
						c  = ReadString (fp, buf); id = atoi (buf);
						c += ReadString (fp, buf); x = atof (buf);
						c += ReadString (fp, buf); y = atof (buf);
						c += ReadString (fp, buf); z = atof (buf);
       					w |= ReadNewLine (fp);
						if (c == cols) {
							d++; *xl = AddCoord (*xl, x, y, z, id);
						} else if (c > 0) {
							Warning ("Syntaxfehler in %s", name);
						}
					} while (c == cols);
					break;
		case 5:		do { // N-XYZ-Q
						c  = ReadString (fp, buf); id = atoi (buf);
						c += ReadString (fp, buf); x = atof (buf);
						c += ReadString (fp, buf); y = atof (buf);
						c += ReadString (fp, buf); z = atof (buf);
						c += ReadString (fp, buf); q = atof (buf);
       					w |= ReadNewLine (fp);
						if (c == cols) {
							d++; *xl = AddCoord (*xl, x, y, z, id, q);
						} else if (c > 0) {
							Warning ("Syntaxfehler in %s", name);
						}
					} while (c == cols);
					break;
		case 6:		do { // XYZ-RGB
						c  = ReadString (fp, buf); x = atof (buf);
						c += ReadString (fp, buf); y = atof (buf);
						c += ReadString (fp, buf); z = atof (buf);
						c += ReadString (fp, buf); r = atoi (buf);
						c += ReadString (fp, buf); g = atoi (buf);
						c += ReadString (fp, buf); b = atoi (buf);
       					w |= ReadNewLine (fp);
						if (c == cols) {
							d++; *xl = AddCoord (*xl, x, y, z, d, COLOR(r, g, b));
						} else if (c > 0) {
							Warning ("Syntaxfehler in %s", name);
						}
					} while (c == cols);
					break;
		case 7:		do { // N-XYZ-RGB
						c  = ReadString (fp, buf); id = atoi (buf);
						c += ReadString (fp, buf); x = atof (buf);
						c += ReadString (fp, buf); y = atof (buf);
						c += ReadString (fp, buf); z = atof (buf);
						c += ReadString (fp, buf); r = atoi (buf);
						c += ReadString (fp, buf); g = atoi (buf);
						c += ReadString (fp, buf); b = atoi (buf);
       					w |= ReadNewLine (fp);
						if (c == cols) {
							d++; *xl = AddCoord (*xl, x, y, z, id, COLOR(r, g, b));
						} else if (c > 0) {
							Warning ("Syntaxfehler in %s", name);
						}
					} while (c == cols);
					break;
		case 8:		do { // N-XYZ-RGB-Q
						c  = ReadString (fp, buf); id = atoi (buf);
						c += ReadString (fp, buf); x = atof (buf);
						c += ReadString (fp, buf); y = atof (buf);
						c += ReadString (fp, buf); z = atof (buf);
						c += ReadString (fp, buf); r = atoi (buf);
						c += ReadString (fp, buf); g = atoi (buf);
						c += ReadString (fp, buf); b = atoi (buf);
						c += ReadString (fp, buf); q = atof (buf);
       					w |= ReadNewLine (fp);
						if (c == cols) {
							d++; *xl = AddCoord (*xl, x, y, z, id, COLOR(r, g, b), q);
						} else if (c > 0) {
							Warning ("Syntaxfehler in %s", name);
						}
					} while (c == cols);
					break;
		case 9:		do { // ROMA-PKO
						c  = ReadString (fp, buf); id = atoi (buf);
						c += ReadString (fp, buf); x = atof (buf);
						c += ReadString (fp, buf); y = atof (buf);
						c += ReadString (fp, buf); z = atof (buf);
						c += ReadString (fp, buf);
						c += ReadString (fp, buf);
						c += ReadString (fp, buf);
						c += ReadString (fp, buf);
						c += ReadString (fp, buf);
       					w |= ReadNewLine (fp);
						if (c == cols) {
							d++; *xl = AddCoord (*xl, x, y, z, id);
						} else if (c > 0) {
							Warning ("Syntaxfehler in %s", name);
						}
					} while (c == cols);
	}
	if (w == true) Warning ("Spalten ignoriert!");
    Debug ("%ld Kontrollpunkte eingelesen", d);

    fclose (fp);
}

// ===========================================================================
//
// ===========================================================================



// ===========================================================================
//
// ===========================================================================


// ===========================================================================
//
// ===========================================================================

// ===========================================================================
//
// ===========================================================================

void WriteDisp (char *fname, matrix *Disp, double min, double factor, bool markUnmatchedAreaRed)
{
    const int w= Disp->cols;
    const int h= Disp->rows;
    const byte channels = (byte) (markUnmatchedAreaRed?3:1);
    const byte badDisp[3] ={0,0,255};
    const byte occlusion[3]={0,255,0};
    int x, y, c, pos;
	double *DispRow=NULL;
    byte color;
    image *pic =NULL;

    printf("\nScaling for disparity image: %f, offset: %f",factor,min);

    if(CheckMatrix(Disp)){
        pic=AllocImage(w,h, channels, fname);

        pos=0;
        for (y = 0; y < h; ++y) {
            DispRow = Disp->m[y];
            for(x = 0; x < w; ++x) {
                if( ISNONZERO( BAD_DISP-DispRow[x] ) ){
                    if(ISNONZERO(OCCLUSION-DispRow[x])){
                        color= (CLIP_GRAY_double(factor*(DispRow[x]-min)));
					    for(c=0;c<channels;++c){
						    pic->d[pos++]=color;
					    }
                    }
                    else{
					    for(c=0;c<channels;++c){
	                        pic->d[pos++]=occlusion[c];
					    }
                    }
                }
                else{
				    for(c=0;c<channels;++c){
					    pic->d[pos++]=badDisp[c]; //r
				    }
			    }
            }
        }

        WritePBM(fname, pic);
        FreeImage(pic);
    }
}

//IMAGE
void ImageCopy (image *pic1, image *pic2)
{
	if (CheckSameImage (pic1, pic2)) {
		memcpy (pic2->d, pic1->d, pic1->w * pic1->h * pic1->typ);
	}
}

// ===========================================================================
//
// ===========================================================================

image *ImageClone (image *pic1)
{
    image *pic2 = NULL;

    CheckImage (pic1);

    pic2 = AllocImage (pic1->w, pic1->h, pic1->typ, pic1->name);
    ImageCopy (pic1, pic2);

    return pic2;
}

// ===========================================================================
//
// ===========================================================================

image *ImageRotate (image *pic1)
{
    image *pic2;
    int w, h, i, j, k;

    CheckImage (pic1);
    w = pic1->h; h = pic1->w;

    pic2 = AllocImage (w, h, pic1->typ, pic1->name);

    for (i=0; i < h; ++i) {
        for (j=0; j < w; ++j) {
            for (k=0; k < pic1->typ; ++k) {
                pic2->d[((i+1)*w-j-1)*pic1->typ+k] = pic1->d[(j*h+i)*pic1->typ+k];
            }
        }
    }
    return pic2;
}

// ===========================================================================
//
// ===========================================================================

void ImagePatch (int x, int y, image *pic1, image *pic2)
{
    int i, j, x2, y2, w, h, wp, hp, t, k;

    CheckTypImage (pic1, pic2);

    w  = pic1->w; h  = pic1->h;
    wp = pic2->w; hp = pic2->h;

    t = MIN(pic1->typ, pic2->typ);

    if ((x < 0) || (y < 0) || (x+wp > w) || (y+hp > h)) {
        for (i=0; i<hp; ++i) {
            for (j=0; j<wp; ++j) {
                x2 = MIN (MAX (0, x + j), w - 1);
                y2 = MIN (MAX (0, y + i), h - 1);
                for (k=0; k<pic1->typ; ++k) {
                    pic2->d[(i*wp + j)*t+k] = pic1->d[(y2 * w + x2)*t+k];
                }
            }
        }
    } else {
        for (i=0; i<hp; ++i) {
            memcpy (pic2->d + (i*wp) * t, pic1->d + ((y+i)*w + x) * t, wp * t);
        }
    }
}

// ===========================================================================
//
// ===========================================================================

image *FreeImage (image *pic)
{
	int i;

    if (pic) {
        if (pic->d) {
            hfree (pic->d);
            pic->d = NULL;
        }
        if((pic->tagc > 0) && (pic->tagv)){
            for (i=0; i<pic->tagc; ++i){
                if (pic->tagv[i]) hfree(pic->tagv[i]);
            }
            hfree(pic->tagv);
        }
        hfree (pic);
    }
	return NULL;
}

// ===========================================================================
//
// ===========================================================================

image *AllocImage (int w, int h, byte typ, char *name)
{
    image *pic = NULL;

	if ((w > 0) && (h > 0)) {
		pic = (image *) halloc (1, sizeof(image));

		pic->w = w;
		pic->h = h;
		pic->typ = typ;
		if (name == NULL) {
			pic->name = DEFAULT_NAME;
		} else {
			pic->name = strdup(name);
		}
		pic->d = (byte *) halloc (typ * w * h, sizeof(byte));
        pic->lineInc = w*typ;
	}
    return pic;
}

// ===========================================================================
//
// ===========================================================================

image *GrayToColor (image *pic1)
{
    image *pic2;
    byte *c, *g;
    long i;

	CheckImage(pic1);

    if (pic1->typ == COLOR_IMAGE) {
        return ImageClone (pic1);
    }
    pic2 = AllocImage (pic1->w, pic1->h, COLOR_IMAGE, pic1->name);

    for (i=0, c=pic2->d, g=pic1->d; i < (pic1->w * pic1->h); i++, c+=3) {
        *c = *(c+1) = *(c+2) = *g++;
    }
    return pic2;
}

// ===========================================================================
//
// ===========================================================================

image *ColorToGray (image *pic1)
{
    image *pic2;
    byte *c, *g;
    long i;

	CheckImage(pic1);

    if (pic1->typ == GRAY_IMAGE) {
        return ImageClone (pic1);
    }
    pic2 = AllocImage (pic1->w, pic1->h, GRAY_IMAGE, pic1->name);

    for (i=0, g=pic2->d, c=pic1->d; i < (pic1->w * pic1->h); i++, c+=3) {
        *g++ = TO_GRAY(*c, *(c+1), *(c+2));
    }
    return pic2;
}

// ===========================================================================
//
// ===========================================================================

void JoinImages (image *pic1, image *pic2)
{
    long i;

	if (CheckSameImage (pic1, pic2)) {
		for (i=0; i < (pic1->w * pic1->h * pic1->typ); ++i) {
			pic1->d[i] = CLIP_GRAY_UINT((pic1->d[i] + pic2->d[i]) / 2);
		}
	}
}

void JoinImages (image *pic1, image *pic2, image *pic3)
{
    long i;

	if (CheckSameImage (pic1, pic2) && CheckSameImage (pic1, pic3)) {
		for (i=0; i < (pic1->w * pic1->h * pic1->typ); ++i) {
			pic1->d[i] = CLIP_GRAY_UINT((pic1->d[i] + pic2->d[i] + pic3->d[i]) / 3);
		}
	}
}

// ===========================================================================
//
// ===========================================================================

void ImageAdd (image *pic1, image *pic2, int cols, matrix **r)
{
	double *rptr, *eptr;
	byte *ptr1, *ptr2, typ;

    if (!CheckSameImage (pic1, pic2)) return;

    *r = mat_alloc(pic1->w, pic1->h);
	ptr1 = pic1->d + cols;
	ptr2 = pic2->d + cols;
	rptr = (*r)->m[0];
    eptr= rptr+pic1->w * pic1->h;
	typ = pic1->typ;

    for (;rptr<eptr;) {
        *rptr++ = *ptr1 + *ptr2;
		ptr1 += typ;
		ptr2 += typ;
    }
}

// ===========================================================================
//
// ===========================================================================

void ImageDiff (image *pic1, image *pic2, int cols, matrix **r)
{
	double *rptr, *eptr;
	byte *ptr1, *ptr2, typ;

    if (!CheckSameImage (pic1, pic2)) return;

    *r = mat_alloc(pic1->w, pic1->h);
	ptr1 = pic1->d + cols;
	ptr2 = pic2->d + cols;
	rptr = (*r)->m[0];
    eptr = rptr+pic1->w* pic1->h;
	typ = pic1->typ;
    for (; rptr < eptr;) {
        *rptr++ = *ptr1 - *ptr2;
		ptr1 += typ;
		ptr2 += typ;
    }
}

// ===========================================================================
//
// ===========================================================================

void ImageDiffCol (image *pic1, image *pic2, matrix **r)
{
    double *aptr, *eptr;
    byte *p1ptr, *p2ptr;
    int j;
    const int typ= pic1->typ;

    if (!CheckSameImage (pic1, pic2)) return;

    *r= mat_alloc(pic1->w, pic1->h);
    aptr= (*r)->m[0];
    eptr= aptr+ pic1->w* pic1->h;
    p1ptr=pic1->d;
    p2ptr=pic2->d;

    for (;aptr<eptr; ++aptr) {
        for (j=0; j < typ; ++j) {
            *aptr += *p1ptr++ -*p2ptr++;
        }
    }
}

// ===========================================================================
//
// ===========================================================================

void SeperateColor (image *pic, image **red, image **green, image **blue)
{
    int w, h;
    long i;
    byte *cptr, *rptr, *gptr, *bptr;

	if (!CheckColorImage (pic)) {
		Warning ("SeperateColor: no data!");
		*red = *green = *blue = NULL; return;
	}
    w = pic->w; h = pic->h;

    *red   = AllocImage (w, h, GRAY_IMAGE, pic->name);
    *green = AllocImage (w, h, GRAY_IMAGE, pic->name);
    *blue  = AllocImage (w, h, GRAY_IMAGE, pic->name);
    rptr = (*red)->d;
    gptr = (*green)->d;
    bptr = (*blue)->d;
	cptr = pic->d;

    for (i=0; i < (w * h); ++i) {
        *rptr++ = *cptr++;
        *gptr++ = *cptr++;
        *bptr++ = *cptr++;
    }
}

image *JoinColor (image *red, image *green, image *blue)
{
    int w, h;
    long i;
    byte *cptr, *rptr, *gptr, *bptr;
	image *pic;

	if (!CheckSameImage (red, green) || !CheckSameImage (blue, green)) {
		Warning ("JoinColor: no data!"); return NULL;
	}
    w = red->w; h = red->h;
    pic = AllocImage (w, h, COLOR_IMAGE, red->name);

    rptr = red->d;
    gptr = green->d;
    bptr = blue->d;
	cptr = pic->d;

    for (i=0; i < (w * h); ++i) {
        *cptr++ = *rptr++;
        *cptr++ = *gptr++;
        *cptr++ = *bptr++;
    }
	return pic;
}

// ===========================================================================
//
// ===========================================================================

void InvertImage (image *pic)
{
    long i;

    CheckImage (pic);

    for (i = 0; i < (pic->w * pic->h * pic->typ); ++i) {
        pic->d[i] = (byte)(MAX_GRAY - pic->d[i]);
    }
}

// ===========================================================================
//
// ===========================================================================

void FillImage (image *pic, int color)
{
    byte c, *p;
    long i;

    CheckImage (pic);

    if (pic->typ == GRAY_IMAGE) {
        c = GET_GRAY(color);
        for (i=0; i < (pic->w * pic->h); ++i) {
            pic->d[i] = c;
        }
    } else {
        for (i=0, p=pic->d; i < (pic->w * pic->h); ++i) {
            *p++ = (byte)GET_R(color);
            *p++ = (byte)GET_G(color);
            *p++ = (byte)GET_B(color);
        }
    }
}

// ===========================================================================
//
// ===========================================================================

image *GetPattern (image *pic, celem *c, int size)
{
    image *pat;
    region roi;
    int r, ix, iy;

	if (!CheckImage (pic) || !c) {
		Warning("GetPattern: no data!"); return NULL;
	}
    r = MAX(1, size >> 1);
	if ((c->x < r) || (c->y < r) ||
		(c->x >= pic->w - r) || (c->y >= pic->h - r)) {
        return NULL;
	}
	ix = ROUND(c->x);
	iy = ROUND(c->y);
    SetROI (ix - r, iy - r, size, size, &roi);
    pat = ImageShift (&roi, c->x - (double)ix, c->y - (double)iy, pic);

    return pat;
}

image *GetPattern (image *pic, pelem *p, int size)
{
    image *pat;
    region roi;
    int r, ix, iy;

	if (!CheckImage (pic) || !p) {
		Warning("GetPattern: no data!"); return NULL;
	}
    r = MAX(1, size >> 1);
	if ((p->x < r) || (p->y < r) ||
		(p->x >= pic->w - r) || (p->y >= pic->h - r)) {
        return NULL;
	}
	ix = ROUND(p->x);
	iy = ROUND(p->y);
    SetROI (ix - r, iy - r, size, size, &roi);
    pat = ImageShift (&roi, p->x - (double)ix, p->y - (double)iy, pic);

    return pat;
}

// ===========================================================================
//
// ===========================================================================

static void MinMaxImage (image *pic, int *min, int *max)
{
    long i;

	CheckImage(pic);
    *min = 255, *max = 0;

    for (i=0; i<pic->w*pic->h*pic->typ; ++i) {
        *min = MIN (*min, pic->d[i]);
        *max = MAX (*max, pic->d[i]);
    }
}

// ===========================================================================
//
// ===========================================================================

void NormalizeImage (image *pic)
{
    long i;
    int min, max;

	CheckImage(pic);
    MinMaxImage (pic, &min, &max);

    if (max > min) {
        for (i=0; i<pic->w*pic->h*pic->typ; ++i) {
            pic->d[i] = CLIP_GRAY_double ((double)(pic->d[i] - min) * 255.0 / (double)(max - min));
        }
    }
}

// ===========================================================================
//
// ===========================================================================

double IntensityMeanLayer (image *pic, int cols)
{
    double mean = 0;
    long i, s;

	CheckImage(pic);
    s = pic->w * pic->h;

    for (i=0; i<s; ++i) {
        mean += (double)pic->d[i * pic->typ + cols];
    }
    mean /= (double)s;

    return (mean / 255.0);
}

//improc
void Convolution (image *in, int cols, mask *kernel, image *out)
{
    double sum, *kptr, *kstart, *kend, *tptr, *tstart, *tend, *tbase;
    byte *iptr, *istart, *iend, *optr;
    int s, z, r, w, h, typ, lineInc;
	matrix *tmp;

	if (!CheckSameImage(in,out) || !CheckMask(kernel)) {
		return;
	}
	w       = in->w;
	h       = in->h;
    typ     = in->typ;
    lineInc = in->lineInc;

	tmp = mat_alloc(h, w);
	if (!tmp) return;
	tbase  = tmp->m[0];

	r      = kernel->radius;
	kstart = kernel->d1;
	kend   = kstart + kernel->size;
    for (istart=in->d + cols, iend=istart+lineInc-typ, tptr=tbase, z=0; z < h; ++z, istart+=lineInc, iend+=lineInc) {
        for (s=0; s < w; ++s) {
            sum = 0.0;
            for (kptr=kstart, iptr=istart+(s-r)*typ; kptr < kend; iptr+=typ) {
                if ((iptr >= istart) && (iptr <= iend)) {
                    sum += *kptr++ * (double)*iptr;               // / MAX_GRAY;
                } else if (iptr < istart) {       // Mirror elements on border
                    sum += *kptr++ * (double)*(istart+(istart-iptr-typ));
                } else {
                    sum += *kptr++ * (double)*(iend+(iend-iptr+typ));
                }
            }
            *tptr++ = sum;
        }
    }
	if (kernel->d2) {                                            // asymmetric
		kstart = kernel->d2;
		kend   = kstart + kernel->size;
	}
    for (optr=out->d + cols, z=0; z < h; ++z) {
        for (tstart=tbase, tend=tstart+(h-1)*w, s=0; s < w; ++s, ++tstart, ++tend, optr+=typ) {
            sum = 0.0;
            for (kptr=kstart, tptr=tstart+(z-r)*w; kptr < kend; tptr+=w) {
                if ((tptr >= tstart) && (tptr <= tend)) {
                    sum += *kptr++ * *tptr;
                } else if (tptr < tstart) {       // Mirror elements on border
                    sum += *kptr++ * *(tstart+(tstart-tptr-w));
                } else {
                    sum += *kptr++ * *(tend+(tend-tptr+w));
                }
            }
            *optr = CLIP_GRAY_INT(ROUND(sum));
        }
    }
	tmp = mat_free(tmp);
}

/**
 Apply local operators to image layers using convolution of filter masks.

 @param[in,out]	pic		Modified input image.
 @param[in]		cols		RGB color layer number [0,1,2].
 @param[in]		kernel	Convolution kernel.

 @note			Asymmetric mathematical kernels must be mirrored for image processing!
*/
void Convolution (image *pic, int cols, mask *kernel)
{
	Convolution (pic, cols, kernel, pic);
}

/**
 Apply local operators to image layers using convolution of filter masks.

 @param[in]		in		Input image.
 @param[in]		cols		RGB color layer number [0,1,2].
 @param[in]		kernel	Convolution kernel.
 @param[out]	out		New allocated gray image with filtered elements.

 @note			Asymmetric mathematical kernels must be mirrored for image processing!
*/
void Convolution (image *in, int cols, mask *kernel, image **out)
{
	if (CheckImage(in) && out) {
		*out = AllocImage (in->w, in->h, GRAY_IMAGE, in->name);
		Convolution (in, cols, kernel, *out);
	}
}

/**
 Apply local operators to image layers using convolution of filter masks.

 @param[in]		pic		Input image.
 @param[in]		cols		RGB color layer number [0,1,2].
 @param[in]		kernel	Convolution kernel.
 @param[out]	mat		Overwritten output matrix with filtered elements.

 @note			Asymmetric mathematical kernels must be mirrored for image processing!
*/
void Convolution (image *pic, int cols, mask *kernel, matrix *mat)
{
    double sum, *kptr, *kstart, *kend, *mptr, *tptr, *tstart, *tend, *tbase;
    byte *iptr, *istart, *iend;
    int s, z, r, w, h, typ, lineInc;
	matrix *tmp;

	if (!CheckImage(pic) || !CheckMask(kernel) || !CheckMatrix(mat)) {
		return;
	}
	w       = pic->w;
	h       = pic->h;
    typ     = pic->typ;
    lineInc = pic->lineInc;

	tmp = mat_alloc(h, w);
	if (!tmp) return;
	tbase  = tmp->m[0];

	r      = kernel->radius;
	kstart = kernel->d1;
	kend   = kstart + kernel->size;
    for (istart=pic->d + cols, iend=istart+lineInc-typ, tptr=tbase, z=0; z < h; ++z, istart+=lineInc, iend+=lineInc) {
        for (s=0; s < w; ++s) {
            sum = 0.0;
            for (kptr=kstart, iptr=istart+(s-r)*typ; kptr < kend; iptr+=typ) {
                if ((iptr >= istart) && (iptr <= iend)) {
                    sum += *kptr++ * (double)*iptr;               // / MAX_GRAY;
                } else if (iptr < istart) {       // Mirror elements on border
                    sum += *kptr++ * (double)*(istart+(istart-iptr-typ));
                } else {
                    sum += *kptr++ * (double)*(iend+(iend-iptr+typ));
                }
            }
            *tptr++ = sum;
        }
    }
	if (kernel->d2) {                                            // asymmetric
		kstart = kernel->d2;
		kend   = kstart + kernel->size;
	}
    for (mptr=mat->m[0], z=0; z < h; ++z) {
        for (tstart=tbase, tend=tstart+(h-1)*w, s=0; s < w; ++s, ++tstart, ++tend) {
            sum = 0.0;
            for (kptr=kstart, tptr=tstart+(z-r)*w; kptr < kend; tptr+=w) {
                if ((tptr >= tstart) && (tptr <= tend)) {
                    sum += *kptr++ * *tptr;
                } else if (tptr < tstart) {       // Mirror elements on border
                    sum += *kptr++ * *(tstart+(tstart-tptr-w));
                } else {
                    sum += *kptr++ * *(tend+(tend-tptr+w));
                }
            }
            *mptr++ = sum;
        }
    }
	tmp = mat_free(tmp);
}

/**
 Apply local operators to image layers using convolution of filter masks.

 @param[in]		pic		Input image.
 @param[in]		cols		RGB color layer number [0,1,2].
 @param[in]		kernel	Convolution kernel.
 @param[out]	mat		New allocated output matrix with filtered elements.

 @note			Asymmetric mathematical kernels must be mirrored for image processing!
*/
void Convolution (image *pic, int cols, mask *kernel, matrix **mat)
{
	if (CheckImage(pic) && mat) {
		*mat = mat_alloc (pic->h, pic->w);
		Convolution (pic, cols, kernel, *mat);
	}
}

/**
 Apply local operators to matrices using convolution of filter masks.

 @param[in]		in		Input matrix.
 @param[in]		kernel	Convolution kernel.
 @param[out]	out		Overwritten output matrix with filtered elements.

 @note			Asymmetric mathematical kernels must be mirrored for image processing!
*/
void Convolution (matrix *in, mask *kernel, matrix *out)
{
    double sum, *iptr, *istart, *iend, *kptr, *kstart, *kend, *optr, *tptr, *tstart, *tend, *tbase;
    int s, z, r, w, h;
	matrix *tmp;

	if (!CheckSameMatrix(in,out) || !CheckMask(kernel)) {
		return;
	}
	w = in->cols;
	h = in->rows;

	tmp = mat_alloc(h, w);
	if (!tmp) return;
	tbase  = tmp->m[0];

	r      = kernel->radius;
	kstart = kernel->d1;
	kend   = kstart + kernel->size;
    for (istart=in->m[0], iend=istart+w-1, tptr=tbase, z=0; z < h; ++z, istart+=w, iend+=w) {
        for (s=0; s < w; ++s) {
            sum = 0.0;
            for (kptr=kstart, iptr=istart+s-r; kptr < kend; ++iptr) {
                if ((iptr >= istart) && (iptr <= iend)) {
                    sum += *kptr++ * *iptr;         // / MAX_GRAY;
                } else if (iptr < istart) {     // Mirror elements on border
                    sum += *kptr++ * *(istart+(istart-iptr-1));
                } else {
                    sum += *kptr++ * *(iend+(iend-iptr+1));
                }
            }
            *tptr++ = sum;
        }
    }
	if (kernel->d2) {                                               // asymmetric
		kstart = kernel->d2;
		kend   = kstart + kernel->size;
	}
    for (optr=out->m[0], z=0; z < h; ++z) {
        for (tstart=tbase, tend=tstart+(h-1)*w, s=0; s < w; ++s, ++tstart, ++tend) {
            sum = 0.0;
            for (kptr=kstart, tptr=tstart+(z-r)*w; kptr < kend; tptr+=w) {
                if ((tptr >= tstart) && (tptr <= tend)) {
                    sum += *kptr++ * *tptr;
                } else if (tptr < tstart) {     // Mirror elements on border
                    sum += *kptr++ * *(tstart+(tstart-tptr-w));
                } else {
                    sum += *kptr++ * *(tend+(tend-tptr+w));
                }
            }
            *optr++ = sum;
        }
    }
	tmp = mat_free(tmp);
}

/**
 Apply local operators to matrices using convolution of filter masks.

 @param[in,out]	mat		Modified input matrix.
 @param[in]		kernel	Convolution kernel.

 @note			Asymmetric mathematical kernels must be mirrored for image processing!
*/
void Convolution (matrix *mat, mask *kernel)
{
	Convolution (mat, kernel, mat);
}

/**
 Apply local operators to matrices using convolution of filter masks.

 @param[in]		in		Input matrix.
 @param[in]		kernel	Convolution kernel.
 @param[out]	out		New allocated output matrix with filtered elements.

 @note			Asymmetric mathematical kernels must be mirrored for image processing!
*/
void Convolution (matrix *in, mask *kernel, matrix **out)
{
	if (CheckMatrix(in) && out) {
		*out = mat_alloc (in->rows, in->cols);
		Convolution (in, kernel, *out);
	}
}
//@}

/** @name Mean-value Smoothing (Box Filter) */
//@{

/**
 Image layer filtering using a mean value mask with a given radius.

 @param[in]		radius	Filter mask radius.
 @param[in]		cols		RGB color layer number [0,1,2].
 @param[in,out]	pic		Modified input image layer.
*/
void Mean (int radius, int cols, image *pic)
{
	mask *kernel;

	kernel = mask_mean(radius);
	Convolution (pic, cols, kernel);
	kernel = mask_free(kernel);
}

/**
 Image layer filtering using a mean value mask with a given radius.

 @param[in]		radius	Filter mask radius.
 @param[in]		cols		RGB color layer number [0,1,2].
 @param[in]		in		Input image.
 @param[out]	out		Overwritten filtered output image layer.
*/
void Mean (int radius, int cols, image *in, image *out)
{
	mask *kernel;

	kernel = mask_mean(radius);
	Convolution (in, cols, kernel, out);
	kernel = mask_free(kernel);
}

/**
 Image layer filtering using a mean value mask with a given radius.

 @param[in]		radius	Filter mask radius.
 @param[in]		cols		RGB color layer number [0,1,2].
 @param[in]		in		Input image.
 @param[out]	out		New allocated gray value image with filtered result.
*/
void Mean (int radius, int cols, image *in, image **out)
{
	mask *kernel;

	kernel = mask_mean(radius);
	Convolution (in, cols, kernel, out);
	kernel = mask_free(kernel);
}

/**
 Image filtering using a mean value mask with a given radius.

 @param[in]		radius	Filter mask radius.
 @param[in,out]	pic		Modified input image.
*/
void Mean (int radius, image *pic)
{
	mask *kernel;
    int i;

	if (CheckImage(pic)) {
		kernel = mask_mean(radius);
		for (i = 0; i < pic->typ; ++i) {
			Convolution (pic, i, kernel);
		}
		kernel = mask_free(kernel);
	}
}

/**
 Image filtering using a mean value mask with a given radius.

 @param[in]		radius	Filter mask radius.
 @param[in]		in		Input image.
 @param[out]	out		Overwrittem filtered output image.
*/
void Mean (int radius, image *in, image *out)
{
	mask *kernel;
    int i;

	if (CheckTypImage(in, out)) {
		kernel = mask_mean(radius);
		for (i = 0; i < in->typ; ++i) {
			Convolution (in, i, kernel, out);
		}
		kernel = mask_free(kernel);
	}
}

/**
 Image filtering using a mean value mask with a given radius.

 @param[in]		radius	Filter mask radius.
 @param[in]		in		Input image.
 @param[out]	out		New allocated and filtered output image.
*/
void Mean (int radius, image *in, image **out)
{
	mask *kernel;
    int i;

	if (CheckImage(in) && out) {
		*out = AllocImage (in->w, in->h, in->typ, in->name);
		kernel = mask_mean(radius);
		for (i = 0; i < in->typ; ++i) {
			Convolution (in, i, kernel, *out);
		}
		kernel = mask_free(kernel);
	}
}

/**
 Matrix filtering using a mean value mask with a given radius.

 @param[in]		radius	Filter mask radius.
 @param[in,out]	mat		Modified input matrix.
*/
void Mean (int radius, matrix *mat)
{
	mask *kernel;

	kernel = mask_mean(radius);
	Convolution (mat, kernel);
	kernel = mask_free(kernel);
}

/**
 Matrix filtering using a mean value mask with a given radius.

 @param[in]		radius	Filter mask radius.
 @param[in]		in		Input matrix.
 @param[out]	out		Overwrittem filtered output matrix.
*/
void Mean (int radius, matrix *in, matrix *out)
{
	mask *kernel;

	kernel = mask_mean(radius);
	Convolution (in, kernel, out);
	kernel = mask_free(kernel);
}

/**
 Matrix filtering using a mean value mask with a given radius.

 @param[in]		radius	Filter mask radius.
 @param[in]		in		Input matrix.
 @param[out]	out		New allocated and filtered output matrix.
*/
void Mean (int radius, matrix *in, matrix **out)
{
	mask *kernel;

	kernel = mask_mean(radius);
	Convolution (in, kernel, out);
	kernel = mask_free(kernel);
}
//@}

/** @name Gaussian Smoothing */
//@{

/**
 Image layer filtering using a Gaussian mask with a given standard deviation.

 @param[in]		sigma	Standard deviation.
 @param[in]		cols		RGB color layer number [0,1,2].
 @param[in,out]	pic		Modified image layer.
*/
void Gauss (double sigma, int cols, image *pic)
{
	mask *kernel;

	kernel = mask_gauss(sigma);
	Convolution (pic, cols, kernel);
	kernel = mask_free(kernel);
}

/**
 Image layer filtering using a Gaussian mask with a given standard deviation.

 @param[in]		sigma	Standard deviation.
 @param[in]		cols		RGB color layer number [0,1,2].
 @param[in]		in		Input image.
 @param[out]	out		Overwritten filtered output image layer.
*/
void Gauss (double sigma, int cols, image *in, image *out)
{
	mask *kernel;

	kernel = mask_gauss(sigma);
	Convolution (in, cols, kernel, out);
	kernel = mask_free(kernel);
}

/**
 Image layer filtering using a Gaussian mask with a given standard deviation.

 @param[in]		sigma	Standard deviation.
 @param[in]		cols		RGB color layer number [0,1,2].
 @param[in]		in		Input image.
 @param[out]	out		New allocated gray value image with filtered result.
*/
void Gauss (double sigma, int cols, image *in, image **out)
{
	mask *kernel;

	kernel = mask_gauss(sigma);
	Convolution (in, cols, kernel, out);
	kernel = mask_free(kernel);
}

/**
 Image filtering using a Gaussian mask with a given standard deviation.

 @param[in]		sigma	Standard deviation.
 @param[in,out]	pic		Modified input image.
*/
void Gauss (double sigma, image *pic)
{
	mask *kernel;
    int i;

	if (CheckImage(pic)) {
		kernel = mask_gauss(sigma);
		for (i = 0; i < pic->typ; ++i) {
			Convolution (pic, i, kernel);
		}
		kernel = mask_free(kernel);
	}
}

/**
 Image filtering using a Gaussian mask with a given standard deviation.

 @param[in]		sigma	Standard deviation.
 @param[in]		in		Input image.
 @param[out]	out		Overwritten filtered output image.
*/
void Gauss (double sigma, image *in, image *out)
{
	mask *kernel;
    int i;

	if (CheckTypImage(in, out)) {
		kernel = mask_gauss(sigma);
		for (i = 0; i < in->typ; ++i) {
			Convolution (in, i, kernel, out);
		}
		kernel = mask_free(kernel);
	}
}

/**
 Image filtering using a Gaussian mask with a given standard deviation.

 @param[in]		sigma	Standard deviation.
 @param[in]		in		Input image.
 @param[out]	out		New allocated and filtered output image.
*/
void Gauss (double sigma, image *in, image **out)
{
	mask *kernel;
    int i;

	if (CheckImage(in) && out) {
		*out = AllocImage (in->w, in->h, in->typ, in->name);
		kernel = mask_gauss(sigma);
		for (i = 0; i < in->typ; ++i) {
			Convolution (in, i, kernel, *out);
		}
		kernel = mask_free(kernel);
	}
}

/**
 Matrix filtering using a Gaussian mask with a given standard deviation.

 @param[in]		sigma	Standard deviation.
 @param[in,out]	mat		Modified input matrix.
*/
void Gauss (double sigma, matrix *mat)
{
	mask *kernel;

	kernel = mask_gauss(sigma);
	Convolution (mat, kernel);
	kernel = mask_free(kernel);
}

/**
 Matrix filtering using a Gaussian mask with a given standard deviation.

 @param[in]		sigma	Standard deviation.
 @param[in]		in		Input matrix.
 @param[out]	out		Overwritten filtered output matrix.
*/
void Gauss (double sigma, matrix *in, matrix *out)
{
	mask *kernel;

	kernel = mask_gauss(sigma);
	Convolution (in, kernel, out);
	kernel = mask_free(kernel);
}

/**
 Matrix filtering using a Gaussian mask with a given standard deviation.

 @param[in]		sigma	Standard deviation.
 @param[in]		in		Input matrix.
 @param[out]	out		New allocated and filtered output matrix.
*/
void Gauss (double sigma, matrix *in, matrix **out)
{
	mask *kernel;

	kernel = mask_gauss(sigma);
	Convolution (in, kernel, out);
	kernel = mask_free(kernel);
}
//@}

/** @name Partial Derivatives */
//@{

/**
 Image layer filtering using Gradient-of-Gaussian masks with a given standard deviation.

 @param[in]		sigma	Standard deviation.
 @param[in]		cols		RGB color layer number [0,1,2].
 @param[in]		pic		Input image.
 @param[out]	gx		Overwritten horizontal filtered output matrix.
 @param[out]	gy		Overwritten vertical filtered output matrix.

 @note			gx = NULL or gy = NULL allow selective computation.
*/
void Gradient (double sigma, int cols, image *pic, matrix *gx, matrix *gy)
{
	mask *kernel;

	if (gx && gy) {
		kernel = mask_gradientX(sigma);
		Convolution (pic, cols, kernel, gx);
		mask_transpose(kernel);
		Convolution (pic, cols, kernel, gy);
		kernel = mask_free(kernel);
	} else if (gx) {
		kernel = mask_gradientX(sigma);
		Convolution (pic, cols, kernel, gx);
		kernel = mask_free(kernel);
	} else if (gy) {
		kernel = mask_gradientY(sigma);
		Convolution (pic, cols, kernel, gy);
		kernel = mask_free(kernel);
	}
}

/**
 Image layer filtering using Gradient-of-Gaussian masks with a given standard deviation.

 @param[in]		sigma	Standard deviation.
 @param[in]		cols		RGB color layer number [0,1,2].
 @param[in]		pic		Input image.
 @param[out]	gx		New allocated and horizontal filtered output matrix.
 @param[out]	gy		New allocated and vertical filtered output matrix.

 @note			gx = NULL or gy = NULL allow selective computation.
*/
void Gradient (double sigma, int cols, image *pic, matrix **gx, matrix **gy)
{
	mask *kernel;

	if (gx && gy) {
		kernel = mask_gradientX(sigma);
		Convolution (pic, cols, kernel, gx);
		mask_transpose(kernel);
		Convolution (pic, cols, kernel, gy);
		kernel = mask_free(kernel);
	} else if (gx) {
		kernel = mask_gradientX(sigma);
		Convolution (pic, cols, kernel, gx);
		kernel = mask_free(kernel);
	} else if (gy) {
		kernel = mask_gradientY(sigma);
		Convolution (pic, cols, kernel, gy);
		kernel = mask_free(kernel);
	}
}

/**
 Color image filtering using Gradient-of-Gaussian masks with a given standard deviation.

 @param[in]		sigma		Standard deviation.
 @param[in]		pic			Input image.
 @param[out]	rx, gx, bx	Overwritten horizontal filtered RGB output matrices.
 @param[out]	ry, gy, by	Overwritten vertical filtered RGB output matrices.
*/
void Gradient (double sigma, image *pic, matrix *rx, matrix *ry, matrix *gx, matrix *gy, matrix *bx, matrix *by)
{
	mask *kernel;

	if (CheckColorImage(pic)) {
		kernel = mask_gradientX(sigma);
		Convolution (pic, 0, kernel, rx);
		Convolution (pic, 1, kernel, gx);
		Convolution (pic, 2, kernel, bx);
		mask_transpose(kernel);
		Convolution (pic, 0, kernel, ry);
		Convolution (pic, 1, kernel, gy);
		Convolution (pic, 2, kernel, by);
		kernel = mask_free(kernel);
	}
}

/**
 Color image filtering using Gradient-of-Gaussian masks with a given standard deviation.

 @param[in]		sigma		Standard deviation.
 @param[in]		pic			Input image.
 @param[out]	rx, gx, bx	New allocated and horizontal filtered RGB output matrices.
 @param[out]	ry, gy, by	New allocated and vertical filtered RGB output matrices.
*/
void Gradient (double sigma, image *pic, matrix **rx, matrix **ry, matrix **gx, matrix **gy, matrix **bx, matrix **by)
{
	int w, h;

	if (CheckColorImage(pic)) {
		w = pic->w; h = pic->h;
		*rx = mat_alloc (h, w);	*ry = mat_alloc (h, w);
		*gx = mat_alloc (h, w);	*gy = mat_alloc (h, w);
		*bx = mat_alloc (h, w); *by = mat_alloc (h, w);
		Gradient (sigma, pic, *rx, *ry, *gx, *gy, *bx, *by);
	}
}

/**
 Matrix filtering using Gradient-of-Gaussian masks with a given standard deviation.

 @param[in]		sigma	Standard deviation.
 @param[in]		mat		Input matrix.
 @param[out]	gx		Overwritten horizontal filtered output matrix.
 @param[out]	gy		Overwwritten vertical filtered output matrix.

 @note			gx = NULL or gy = NULL allow selective computation.
*/
void Gradient (double sigma, matrix *mat, matrix *gx, matrix *gy)
{
	mask *kernel;

	if (gx && gy) {
		kernel = mask_gradientX(sigma);
		Convolution (mat, kernel, gx);
		mask_transpose(kernel);
		Convolution (mat, kernel, gy);
		kernel = mask_free(kernel);
	} else if (gx) {
		kernel = mask_gradientX(sigma);
		Convolution (mat, kernel, gx);
		kernel = mask_free(kernel);
	} else if (gy) {
		kernel = mask_gradientY(sigma);
		Convolution (mat, kernel, gy);
		kernel = mask_free(kernel);
	}
}

/**
 Matrix filtering using Gradient-of-Gaussian masks with a given standard deviation.

 @param[in]		sigma	Standard deviation.
 @param[in]		mat		Input matrix.
 @param[out]	gx		New allocated and horizontal filtered output matrix.
 @param[out]	gy		New allocated and vertical filtered output matrix.

 @note			gx = NULL or gy = NULL allow selective computation.
*/
void Gradient (double sigma, matrix *mat, matrix **gx, matrix **gy)
{
	mask *kernel;

	if (gx && gy) {
		kernel = mask_gradientX(sigma);
		Convolution (mat, kernel, gx);
		mask_transpose(kernel);
		Convolution (mat, kernel, gy);
		kernel = mask_free(kernel);
	} else if (gx) {
		kernel = mask_gradientX(sigma);
		Convolution (mat, kernel, gx);
		kernel = mask_free(kernel);
	} else if (gy) {
		kernel = mask_gradientY(sigma);
		Convolution (mat, kernel, gy);
		kernel = mask_free(kernel);
	}
}

/**
 Fast image layer filtering using \f$ 3 \times 3 \f$ Sobel masks.

 @param[in]		cols		RGB color layer number [0,1,2].
 @param[in]		pic		Input image.
 @param[out]	gx		Overwritten horizontal filtered output matrix.
 @param[out]	gy		Overwritten vertical filtered output matrix.

 @note			gx = NULL or gy = NULL allow selective computation.
*/
void Sobel (int cols, image *pic, matrix *gx, matrix *gy)
{
	mask *kernel;

	if (gx && gy) {
		kernel = mask_sobelX();
		Convolution (pic, cols, kernel, gx);
		mask_transpose(kernel);
		Convolution (pic, cols, kernel, gy);
		kernel = mask_free(kernel);
	} else if (gx) {
		kernel = mask_sobelX();
		Convolution (pic, cols, kernel, gx);
		kernel = mask_free(kernel);
	} else if (gy) {
		kernel = mask_sobelY();
		Convolution (pic, cols, kernel, gy);
		kernel = mask_free(kernel);
	}
}

/**
 Fast image layer filtering using \f$ 3 \times 3 \f$ Sobel masks.

 @param[in]		cols		RGB color layer number [0,1,2].
 @param[in]		pic		Input image.
 @param[out]	gx		New allocated and horizontal filtered output matrix.
 @param[out]	gy		New allocated and vertical filtered output matrix.

 @note			gx = NULL or gy = NULL allow selective computation.
*/
void Sobel (int cols, image *pic, matrix **gx, matrix **gy)
{
	mask *kernel;

	if (gx && gy) {
		kernel = mask_sobelX();
		Convolution (pic, cols, kernel, gx);
		mask_transpose(kernel);
		Convolution (pic, cols, kernel, gy);
		kernel = mask_free(kernel);
	} else if (gx) {
		kernel = mask_sobelX();
		Convolution (pic, cols, kernel, gx);
		kernel = mask_free(kernel);
	} else if (gy) {
		kernel = mask_sobelY();
		Convolution (pic, cols, kernel, gy);
		kernel = mask_free(kernel);
	}
}

/**
 Fast color image filtering using \f$ 3 \times 3 \f$ Sobel masks.

 @param[in]		pic			Input image.
 @param[out]	rx, gx, bx	Overwritten horizontal filtered RGB output matrices.
 @param[out]	ry, gy, by	Overwritten vertical filtered RGB output matrices.
*/
void Sobel (image *pic, matrix *rx, matrix *ry, matrix *gx, matrix *gy, matrix *bx, matrix *by)
{
	mask *kernel;

	if (CheckColorImage(pic)) {
		kernel = mask_sobelX();
		Convolution (pic, 0, kernel, rx);
		Convolution (pic, 1, kernel, gx);
		Convolution (pic, 2, kernel, bx);
		mask_transpose(kernel);
		Convolution (pic, 0, kernel, ry);
		Convolution (pic, 1, kernel, gy);
		Convolution (pic, 2, kernel, by);
		kernel = mask_free(kernel);
	}
}

/**
 Fast color image filtering using \f$ 3 \times 3 \f$ Sobel masks.

 @param[in]		pic			Input image.
 @param[out]	rx, gx, bx	New allocated and horizontal filtered RGB output matrices.
 @param[out]	ry, gy, by	New allocated and vertical filtered RGB output matrices.
*/
void Sobel (image *pic, matrix **rx, matrix **ry, matrix **gx, matrix **gy, matrix **bx, matrix **by)
{
	int w, h;

	if (CheckColorImage(pic)) {
		w = pic->w; h = pic->h;
		*rx = mat_alloc (h, w);	*ry = mat_alloc (h, w);
		*gx = mat_alloc (h, w);	*gy = mat_alloc (h, w);
		*bx = mat_alloc (h, w); *by = mat_alloc (h, w);
		Sobel (pic, *rx, *ry, *gx, *gy, *bx, *by);
	}
}

/**
 Fast matrix filtering using \f$ 3 \times 3 \f$ Sobel masks.

 @param[in]		mat		Input matrix.
 @param[out]	gx		Overwritten horizontal filtered output matrix.
 @param[out]	gy		Overwwritten vertical filtered output matrix.

 @note			gx = NULL or gy = NULL allow selective computation.
*/
void Sobel (matrix *mat, matrix *gx, matrix *gy)
{
	mask *kernel;

	if (gx && gy) {
		kernel = mask_sobelX();
		Convolution (mat, kernel, gx);
		mask_transpose(kernel);
		Convolution (mat, kernel, gy);
		kernel = mask_free(kernel);
	} else if (gx) {
		kernel = mask_sobelX();
		Convolution (mat, kernel, gx);
		kernel = mask_free(kernel);
	} else if (gy) {
		kernel = mask_sobelY();
		Convolution (mat, kernel, gy);
		kernel = mask_free(kernel);
	}
}

/**
 Fast matrix filtering using \f$ 3 \times 3 \f$ Sobel masks.

 @param[in]		mat		Input matrix.
 @param[out]	gx		New allocated and horizontal filtered output matrix.
 @param[out]	gy		New allocated and vertical filtered output matrix.

 @note			gx = NULL or gy = NULL allow selective computation.
*/
void Sobel (matrix *mat, matrix **gx, matrix **gy)
{
	mask *kernel;

	if (gx && gy) {
		kernel = mask_sobelX();
		Convolution (mat, kernel, gx);
		mask_transpose(kernel);
		Convolution (mat, kernel, gy);
		kernel = mask_free(kernel);
	} else if (gx) {
		kernel = mask_sobelX();
		Convolution (mat, kernel, gx);
		kernel = mask_free(kernel);
	} else if (gy) {
		kernel = mask_sobelY();
		Convolution (mat, kernel, gy);
		kernel = mask_free(kernel);
	}
}
//@}

// ===========================================================================
//
// ===========================================================================

matrix *GetPatch (int x, int y, matrix *dat, int r)
{
    matrix *pat;
    int i, j, s;

	if (!CheckMatrix(dat)) return NULL;
    if ((r < 1) || (x-r < 0) || (x+r >= dat->cols) || (y-r < 0) || (y+r >= dat->rows)) {
        return NULL;
    }
    s = 2 * r + 1;
    pat = mat_alloc (s, s);

    for (i=0; i<s; ++i) {
        for (j=0; j<s; ++j) {
            pat->m[i][j] = dat->m[y-r + i][x-r + j];
        }
    }
    return pat;
}

// ===========================================================================
//
// ===========================================================================

image *ImageScale (double factor, image *pic)
{
    long m;
    int w, h, i, j, k;
    double x, y;
    image *pic2 = NULL;

    if (!CheckImage(pic)) return NULL;

	if (ISNONZERO(factor)) {
        if (ISEQUAL(factor, 1.0)){
            pic2 = ImageClone(pic);
        } else {
		    w = ROUND((double)pic->w * factor);
		    h = ROUND((double)pic->h * factor);

		    pic2 = AllocImage (w, h, pic->typ, pic->name);

		    for (i=0; i<h; ++i) {
			    for (j=0; j<w; ++j) {
				    x = (double)j / factor;
				    y = (double)i / factor;
				    m = (i*w+j) * pic->typ;
				    for (k = 0; k < pic->typ; ++k) {
					    pic2->d[m+k] = InterpolBiCubic (x, y, k, pic);
				    }
			    }
		    }
        }
	}
    return pic2;
}

// ===========================================================================
//
// ===========================================================================

image *ImageShift (region *roi, double x, double y, image *pic1)
{
    long m;
    int w, h, i, j, k;
    double ox, oy;
    image *pic2;

    if (!roi || !CheckImage(pic1)) return NULL;

    ImageCheckROI (roi, pic1);
    w = roi->w;
    h = roi->h;
    pic2 = AllocImage (w, h, pic1->typ, pic1->name);

    for (j=0; j<h; ++j) {
        oy = (double)(roi->y + j) + y;
        for (i=0; i<w; ++i) {
            ox = (double)(roi->x + i) + x;
            m = (j*w+i) * pic2->typ;
            for (k = 0; k < pic2->typ; ++k) {
                pic2->d[m+k] = InterpolBiLinear (ox, oy, k, pic1);
            }
        }
    }
    return pic2;
}

// ===========================================================================
//
// ===========================================================================

image *ImageUndistort (image *pic1, camera *cam)
{
    long m;
    int w, h, i, j, k;
    double z, d, t = 0.0;
	double x, y, x2, y2;
    image *pic2;

	Debug ("Undistort Image");

    if (!CheckImage(pic1) || !cam || !CheckSizeMatrix(cam->P, 3, 4)) return NULL;

	w = pic1->w;
	h = pic1->h;
    pic2 = AllocImage (w, h, pic1->typ, pic1->name);

    for (j=0; j<h; ++j) {
		y = (double)j - cam->s.y;
        for (i=0; i<w; ++i) {
			x = (double)i - cam->s.x;
            z = SQR(x) + SQR(y);
			d = 1.0 + cam->k1 * z + cam->k2 * SQR(z) + cam->k3 * z*SQR(z);
            x2 = cam->s.x + x * d;
			if ((x2 >= 0) && (x2 < w)) {
				y2 = cam->s.y + y * d;
				if ((y2 >= 0) && (y2 < h)) {
					t = MAX(t, sqrt(SQR(x + cam->s.x - x2) + SQR(y + cam->s.y - y2)));
					m = (j*w+i) * pic2->typ;
					for (k = 0; k < pic2->typ; ++k) {
						pic2->d[m+k] = InterpolBiCubic (x2, y2, k, pic1);
					}
				}
			}
        }
    }
	Debug ("Maximum distortion: %.2f [pix]", t);

    return pic2;
}

// ===========================================================================
//
// ===========================================================================

void GradientVal (matrix *gx, matrix *gy, matrix **g)
{
    double *gxptr, *gyptr, *gptr, *eptr;

    if (!CheckMatrix(gx) || !CheckMatrix(gy) || !g) return;

    *g = mat_alloc(gx->rows, gx->cols);

    gxptr= gx->m[0];
    gyptr= gy->m[0];
    gptr=(*g)->m[0];
    eptr= gxptr+gx->cols*gx->rows;

    for(; gxptr<eptr; ++gxptr, ++gyptr)
        *gptr++= sqrt(SQR(*gxptr)+SQR(*gyptr));
}

// ===========================================================================
//
// ===========================================================================

double GradientMean (matrix *gx, matrix *gy)
{
    double mean = 0, *gxptr, *gyptr, *eptr;
    long s;

    if (!CheckMatrix(gx) || !CheckMatrix(gy)) return 0.0;

    s = gx->cols * gx->rows;
    gxptr=gx->m[0];
    gyptr=gy->m[0];
    eptr= gxptr+s;

    for(; gxptr<eptr; ++gxptr, ++gyptr)
        mean+= sqrt(SQR(*gxptr)+SQR(*gyptr));

	if (s > 0) {
		mean /= (double)s;
	}
    return SQR(mean);
}

// ===========================================================================
//
// ===========================================================================

double GradientProd (matrix *ax, matrix *ay, matrix *bx, matrix *by)
{
    double mean = 0, *axptr, *ayptr, *bxptr, *byptr, *eptr;

    if (!CheckSameMatrix(ax, ay) || !CheckSameMatrix(bx, by) || !CheckSameMatrix(ax,bx)) return 0.0;

    axptr=ax->m[0];
    ayptr=ay->m[0];
    bxptr=bx->m[0];
    byptr=by->m[0];
    eptr= axptr+ax->cols * ax->rows;

    if (axptr <  eptr) {
        for(; axptr<eptr;)
            mean += *axptr++ * *bxptr++ + *ayptr++ * *byptr++;
		mean /= (double)(ax->cols * ax->rows);
	}
    return mean;
}

// ===========================================================================
//
// ===========================================================================

void HistogramEqualization(image *pic)
{
    if(!CheckImage(pic)) return;

    byte *aptr, *eptr;
    const int typ=pic->typ;
    double min[3] ={MAX_GRAY}, max[3]={0};
    int i;
    eptr= pic->d+ (pic->h*pic->w*pic->typ);

    //find min /max for all channels
    for(i=0, aptr= pic->d; aptr<eptr; ++aptr, ++i){
        if(i==typ) i=0;
        min[i]= MIN(min[i], *aptr);
        max[i]= MAX(max[i], *aptr);
    }

    //adjust scale
    for(i=0; i<typ; ++i) max[i]= 1.0/(max[i]-min[i]);
    //equalization
    for(i=0, aptr= pic->d; aptr<eptr; ++aptr, ++i){
        if(i==typ) i=0;
        *aptr=CLIP_GRAY_double((*aptr-min[i])*max[i]);
    }
}

//mask
mask *mask_alloc(int radius)
{
	mask *kernel = NULL;

	if (radius > 0) {
		kernel = (mask *) halloc(1, sizeof(mask));
		if (kernel) {
			kernel->sigma = 0.0;
			kernel->radius = radius;
			kernel->size = 2 * radius + 1;
			kernel->d1 = NULL;
			kernel->d2 = NULL;
		}
	}
	return kernel;
}

/**
 Allocate memory for a convolution mask.

 @param[in]		sigma	Standard deviation > EPS.
 @result				Convolution mask structure or NULL.
*/
mask *mask_alloc(double sigma)
{
	mask *kernel = NULL;

	if (ISPOSITIVE(sigma)) {
		kernel = (mask *) halloc(1, sizeof(mask));
		if (kernel) {
			kernel->sigma = sigma;
			kernel->radius = mask_radius(sigma);
			kernel->size = 2 * kernel->radius + 1;
			kernel->d1 = NULL;
			kernel->d2 = NULL;
		}
	}
	return kernel;
}

/**
 Free memory of a convolution mask.

 @param[in]		M	Allocated convolution mask.
 @result			NULL.
*/
mask *mask_free(mask *M)
{
	if (M) {
		if (M->d1) {
			 hfree(M->d1); M->d1 = NULL;
		}
		if (M->d2) {
			 hfree(M->d2); M->d2 = NULL;
		}
		hfree(M); M = NULL;
	}
	return NULL;
}

/**
 1D convolution mask for Gaussian smoothing \f$ G(x,\sigma)= \frac{1}{\sqrt{2 \pi} \sigma} \exp^{-\frac{x^2}{2 \sigma^2}} \f$. The sum of all elements is around 1, limited by the finite mask.

 @param[in]		sigma	Standard deviation.
 @result				Separated convolution mask.
*/
mask *mask_gauss(double sigma)
{
	mask *kernel;
    double sq2, sq2Pi, *ptr;
    int i, r;

	kernel = mask_alloc(sigma);
	if (kernel) {
		sq2   = 1.0/(2.0*SQR(sigma));
		sq2Pi = 1.0/(sqrt(2.0*PI)*sigma);
		r     = kernel->radius;
        kernel->d1 = (double *) halloc(kernel->size, sizeof (double));
	    if (kernel->d1) {
			ptr = kernel->d1;
			for (i = -r; i <= r; ++i) {
				*ptr++ = exp(-(double)SQR(i) * sq2) * sq2Pi;
			}
		}
	}
	return kernel;
}

/**
 Two 1D convolution masks for horizontal gradients. The sum of all elements is 0.

 @param[in]		sigma	Standard deviation.
 @result				Separated convolution mask.

 @note					The asymmetric partial derivative is mirrored for correlation instead of convolution!
*/
mask *mask_gradientX(double sigma)
{
	double sq, sq2, sq2Pi, *ptr;
	mask *kernel;
	int i, r;

	kernel = mask_alloc(sigma);
	if (kernel) {
		sq2   = 1.0/(2.0*SQR(sigma));
		sq2Pi = 1.0/(sqrt(2.0*PI)*sigma);
		sq    = sq2Pi/SQR(sigma);
		r     = kernel->radius;
        kernel->d1 = (double *) halloc(kernel->size, sizeof (double)); // Horizontal Gaussian derivative
	    if (kernel->d1) {
			ptr = kernel->d1;
			for (i = -r; i <= r; ++i) {
				*ptr++ = exp(-(double)SQR(i) * sq2) * (double)i * sq;  // Mirrored asymmetric mask
			}
		}
        kernel->d2 = (double *) halloc(kernel->size, sizeof (double)); // Vertical Gaussian smoothing
	    if (kernel->d2) {
			ptr = kernel->d2;
			for (i = -r; i <= r; ++i) {
				*ptr++ = exp(-(double)SQR(i) * sq2) * sq2Pi;
			}
		}
	}
	return kernel;
}

/**
 Two 1D convolution masks for vertical gradients.

 @param[in]		sigma	Standard deviation.
 @result				Separated convolution mask.

 @note					The asymmetric partial derivative is mirrored for correlation instead of convolution!
*/
mask *mask_gradientY(double sigma)
{
	double sq, sq2, sq2Pi, *ptr;
	mask *kernel;
	int i, r;

	kernel = mask_alloc(sigma);
	if (kernel) {
		sq2   = 1.0/(2.0*SQR(sigma));
		sq2Pi = 1.0/(sqrt(2.0*PI)*sigma);
		sq    = sq2Pi/SQR(sigma);
		r     = kernel->radius;
        kernel->d1 = (double *) halloc(kernel->size, sizeof (double)); // Horizontal Gaussian smoothing
	    if (kernel->d1) {
			ptr = kernel->d1;
			for (i = -r; i <= r; ++i) {
				*ptr++ = exp(-(double)SQR(i) * sq2) * sq2Pi;
			}
		}
        kernel->d2 = (double *) halloc(kernel->size, sizeof (double)); // Vertical Gaussian derivative
	    if (kernel->d2) {
			ptr = kernel->d2;
			for (i = -r; i <= r; ++i) {
				*ptr++ = exp(-(double)SQR(i) * sq2) * (double)i * sq;  // Mirrored asymmetric mask
			}
		}
	}
	return kernel;
}

/**
 1D convolution mask for mean value smoothing (Box Filter).

 @param[in]		radius	Mask radius.
 @result				Separated convolution mask.
*/
mask *mask_mean(int radius)
{
    double *aptr, *eptr, mean;
	mask *kernel;

	kernel = mask_alloc(radius);
	if (kernel) {
        kernel->d1 = (double *) halloc(kernel->size, sizeof (double));
	    if (kernel->d1) {
			mean = 1.0/kernel->size;
			aptr = kernel->d1;
			eptr = aptr + kernel->size;
			for (; aptr < eptr; ) {
				*aptr++ = mean;
			}
		}
    }
	return kernel;
}

/**
 Estimate mask radius by given standard deviation.

 @param[in]		sigma	Standard deviation.
 @result				Mask radius > 0.
*/
int mask_radius (double sigma)
{
//    return MAX(ROUND(sqrt(2.0) * PI * sigma), 1);
      return MAX((int)(3.0 * sigma), 1); // 2*sigma <= r <= 3*sigma
}

/**
 Scale the two-dimensional mask elements.

 @param[in,out]		kernel	Modified convolution kernel.
 @param[in]			scale	Scaling factor > 0.
*/
void mask_scale (mask *kernel, double scale)
{
    double *aptr, *eptr, s;

    if (scale < 0.0) {
        Warning ("mask_scale: scaling factor negative!"); return;
    }
	s = sqrt(scale);           // separated 1D filters are multiplied
    if (ISZERO(s)) {
        Warning ("mask_scale: scaling factor is zero!"); return;
    }
	if (CheckMask(kernel)) {
		aptr = kernel->d1;
		eptr = aptr + kernel->size;
		for (; aptr < eptr; ) {
			*aptr++ *= s;
		}
	    if (kernel->d2) {
			aptr = kernel->d2;
			eptr = aptr + kernel->size;
			for (; aptr < eptr; ) {
				*aptr++ *= s;
			}
		}
	}
}

/**
 Two 1D convolution masks for the approximation of horizontal gradients \f[ G_x = \frac{1}{4} \left[ \begin{array}{ccc} -1 & 0 & 1\\ -2 & 0 & 2 \\ -1 & 0 & 1 \end{array} \right] = \frac{1}{4} \left[ \begin{array}{c} 1 \\ 2 \\ 1 \end{array} \right] \cdot \left[ \begin{array}{ccc} -1 & 0 & 1 \end{array} \right] \f].

 @result				Separated \f$ 3 \times 3 \f$ convolution mask.

 @note					The asymmetric partial derivative is mirrored for correlation instead of convolution!
*/
mask *mask_sobelX(void)
{
	mask *kernel;

	kernel = mask_alloc(1);
	if (kernel) {
        kernel->d1 = (double *) halloc(3, sizeof (double)); // Horizontal derivative
	    if (kernel->d1) {
			kernel->d1[0] = -1.0;                  // Mirrored asymmetric mask
			kernel->d1[2] =  1.0;
		}
        kernel->d2 = (double *) halloc(3, sizeof (double));  // Vertical smoothing
	    if (kernel->d2) {
			kernel->d2[0] = kernel->d2[2] = 0.25;
			kernel->d2[1] = 0.5;
		}
	}
	return kernel;
}

/**
 Two 1D convolution masks for the approximation of vertical gradients \f[ G_y = \frac{1}{4} \left[ \begin{array}{ccc} -1 & -2 & -1\\ 0 & 0 & 0 \\ 1 & 2 & 1 \end{array} \right] = \frac{1}{4} \left[ \begin{array}{c} -1 \\ 0 \\ 1 \end{array} \right] \cdot \left[ \begin{array}{ccc} 1 & 2 & 1 \end{array} \right] \f].

 @result				Separated \f$ 3 \times 3 \f$ convolution mask.

 @note					The asymmetric partial derivative is mirrored for correlation instead of convolution!
*/
mask *mask_sobelY(void)
{
	mask *kernel;

	kernel = mask_alloc(1);
	if (kernel) {
        kernel->d1 = (double *) halloc(3, sizeof (double)); // Horizontal smoothing
	    if (kernel->d1) {
			kernel->d1[0] = kernel->d1[2] = 0.25;
			kernel->d1[1] = 0.5;
		}
        kernel->d2 = (double *) halloc(3, sizeof (double)); // Vertical derivative
	    if (kernel->d2) {
			kernel->d2[0] = -1.0;                  // Mirrored asymmetric mask
			kernel->d2[2] =  1.0;
		}
	}
	return kernel;
}

/**
 Sum of the two-dimensional mask elements.

 @param[in]		kernel	Convolution kernel.
 @result				Sum of kernel elements.
*/
double mask_sum (mask *kernel)
{
	double sum1=0.0, sum2=0.0, *aptr, *eptr;

	if (CheckMask(kernel)) {
		aptr = kernel->d1;
		eptr = aptr + kernel->size;
		for (; aptr < eptr; ) {
			sum1 += *aptr++;
		}
	    if (kernel->d2) {
			aptr = kernel->d2;
			eptr = aptr + kernel->size;
			for (; aptr < eptr; ) {
				sum2 += *aptr++;
			}
			return (sum1 * sum2); // asymmetric mask
		}
		return SQR(sum1);         // symmetric mask
	}
    return 0.0;
}

/**
 Exchange columns and rows of an asymmetrix mask.

 @param[in,out]		kernel	Modified convolution mask.
*/
void mask_transpose(mask *kernel)
{
	if (CheckMask(kernel) && kernel->d2) {
		SWAP(double *, kernel->d1, kernel->d2);
	}
}

//match
MatchListOld *
ReadMatchedPointsOld (char *name)
{
    MatchListOld *list;
    FILE *fp;

    Debug ("Punktkorrespondenzen '%s' lesen...", name);

    if ((fp = fopen (name, "rb")) == NULL) {
        ErrorExit (OPEN_ERROR, name);
    }
    list = (MatchListOld *) halloc (1, sizeof(MatchListOld));
    if (fread(&list->size, sizeof(int), 1, fp) != 1){
        Warning ("Unexpected eof, while reading size! File: %s \n",name);
		if (list) {
			hfree (list); list = NULL;
		}
        fclose(fp);
        return NULL;
    }
    list->points = (MatchedPointOld*) halloc (list->size, sizeof(MatchedPointOld));
    Debug ("Groesse: %d", list->size);
    if (fread(list->points, sizeof(MatchedPointOld), list->size, fp) != (uint)list->size){
        Warning ("Unexpected eof, while reading matches! File: %s Expected Matches: %d\n", name, list->size);
		if (list->points) {
			hfree (list->points); list->points = NULL;
		}
		if (list) {
			hfree (list); list = NULL;
		}
        fclose(fp);
        return NULL;
    }

    //try to read the offsets, this is a newer version so older list do not have these values and eof is no error!

    if (fread(list->offX, sizeof(double), 3, fp) == (uint)3){
        if (fread(list->offY, sizeof(double), 3, fp) != (uint)3){
            Debug("Fehler beim lesen der Offsets: x-Offsets gefunden, aber keine y-Offsets!");
            list->offX[0]=list->offX[1]=list->offX[2]=NO_OFFSET;
        }
    }


    fclose(fp);
    return list;
}




MatchList *
ReadMatchedPoints (char *name)
{
    MatchList *list;
    FILE *fp;
    int size, returnedSize, i, pointsToRead;
    char listVersion[9];
    int const maxNumber = 65536/sizeof(MatchedPoint);

    Debug ("Reading Matches '%s'...", name);

    if ((fp = fopen (name, "rb")) == NULL) {
        ErrorExit (OPEN_ERROR, name);
    }
    if (fread(&size, sizeof(int), 1, fp) != 1){
        Warning ("Unexpected eof, while reading size! File: %s \n",name);
        fclose(fp);
        return NULL;
    }
    list = AllocMatch(size);

    if(size==0){
        fclose(fp);
        return list;
    }

    if (fread(listVersion, sizeof(char), 9, fp) != 9){
        Warning ("Unexpected eof, while reading version! File: %s \n",name);
        fclose(fp);
        list = FreeMatch(list);
        return NULL;
    }


    if( 'N'==listVersion[0]&&
        'E'==listVersion[1]&&
        'W'==listVersion[2]&&
        'F'==listVersion[3]&&
        'O'==listVersion[4]&&
        'R'==listVersion[5]&&
        'M'==listVersion[6]&&
        'A'==listVersion[7]&&
        'T'==listVersion[8]
        ){
        //read new style (double values)
        Debug ("Size: %d", list->size);
        for(i=0; i<size; i+=maxNumber){
            pointsToRead = (i+maxNumber)>size?size-i:maxNumber;

            returnedSize = (int)fread(list->points+i, sizeof(MatchedPoint), pointsToRead, fp);
            if(returnedSize!=pointsToRead){
                Warning ("Unexpected eof, while reading matches! File: %s \nFound matches: %d \nExpected Matches: %d\n", name, i, size);
                fclose(fp);
                list = FreeMatch(list);
                return NULL;
            }
        }

        //try to read the offsets, this is a newer version so older list do not have these values and eof is no error!

        if (fread(list->offX, sizeof(double), 3, fp) == (uint)3){
            if (fread(list->offY, sizeof(double), 3, fp) != (uint)3){
                Debug("Error during offset rading: x-offsets fount but no y-offsets!");
                list->offX[0]=list->offX[1]=list->offX[2]=NO_OFFSET;
            }
        }
        fclose(fp);
    }
    else{ //read old style (short values *10)
        fclose(fp);
        MatchListOld *oldList;

        Debug ("Old format found. Converting matches...");
        //read
        oldList = ReadMatchedPointsOld(name);

        if(oldList){
            //copy
            for(i=0; i<size;++i){
                list->points[i].color= oldList->points[i].color;
                list->points[i].X[0] = oldList->points[i].x[0]*0.1;
                list->points[i].X[1] = oldList->points[i].x[1]*0.1;
                list->points[i].X[2] = oldList->points[i].x[2]*0.1;
                list->points[i].Y[0] = oldList->points[i].y[0]*0.1;
                list->points[i].Y[1] = oldList->points[i].y[1]*0.1;
                list->points[i].Y[2] = oldList->points[i].y[2]*0.1;
            }
            list->offX[0]=oldList->offX[0];
            list->offX[1]=oldList->offX[1];
            list->offX[2]=oldList->offX[2];
            list->offY[0]=oldList->offY[0];
            list->offY[1]=oldList->offY[1];
            list->offY[2]=oldList->offY[2];
            Debug ("Converted: %d", size);

            //cleanup
			if (oldList->points) {
				hfree(oldList->points); oldList->points = NULL;
			}
			if (oldList) {
				hfree(oldList); oldList = NULL;
			}
        }
        else{
            Debug("Error reading old format!");
            list = FreeMatch(list);
        }
    }

    return list;
}

// ===========================================================================
//
// ===========================================================================

void WriteMatchedPoints(char *fname, MatchList *pointList)
{
    FILE *fp;
    int i, checkValue, wrotePoints, pointsToWrite;
    char version[9] ={ 'N', 'E', 'W', 'F', 'O', 'R', 'M', 'A', 'T'};
    int const maxNumber = 65536/sizeof(MatchedPoint);

    wrotePoints =0;
    Debug ("Punktkorrespondenzen '%s' schreiben...", fname);

    if (pointList && (pointList->size > 0)) {


        if ((fp = fopen (fname, "wb")) == NULL) {
            ErrorExit (OPEN_ERROR, fname);
        }
        fwrite (&pointList->size, sizeof(int), 1, fp);
        fwrite (version, sizeof(char),9, fp);
        for(i=0; i< pointList->size; i+=maxNumber){

            //write approx 64k blocks

            pointsToWrite = (i+maxNumber)>pointList->size?pointList->size-i:maxNumber;
            checkValue = (int)fwrite (pointList->points+i, sizeof(MatchedPoint), pointsToWrite, fp);
            if(checkValue!=pointsToWrite){
                Warning("Error writing matchlist at point %d", i);
            }
            else{
                wrotePoints+=pointsToWrite;
            }
        }
        fwrite (pointList->offX, sizeof(double), 3, fp);
        fwrite (pointList->offY, sizeof(double), 3, fp);

		fclose(fp);
        if(wrotePoints!=pointList->size){
            //set correct size!
            if ((fp = fopen (fname, "ab")) == NULL) {
                ErrorExit (OPEN_ERROR, fname);
            }
            if(fseek( fp, 0, SEEK_SET )==0){
                fwrite (&wrotePoints, sizeof(int), 1, fp);
            }
            fclose(fp);
        }
    }
}

// ===========================================================================
//
// ===========================================================================

MatchList* BuildMatchList(image** pics, matrix* Disp)
{
    MatchList *list;
    int i,j, pos[3], sizes[3], images, size, listPos;
    double checkValue, checkValue2, x,y,disp;
    int r=0, g=0, b=0;
    char *ext;

    size =0;
    for (j=0, y=0; j<Disp->rows; ++j, ++y){
        for (i=0, x=0; i<Disp->cols; ++i, ++x){
            disp = Disp->m[j][i];
            checkValue =disp-BAD_DISP;
            checkValue2=disp-OCCLUSION;
            if(ISNONZERO(checkValue)&&ISNONZERO(checkValue2)&&
                x<pics[0]->w&&
                (x+disp<pics[1]->w||(pics[2]&&y+disp<pics[2]->h))
                ){
                size++;
            }
        }
    }
    list=AllocMatch(size);

    sizes[0]= pics[0]->w*pics[0]->h*pics[0]->typ;
    sizes[1]= pics[1]->w*pics[1]->h*pics[1]->typ;
    if(pics[2])
        sizes[2]= pics[2]->w*pics[2]->h*pics[2]->typ;
    else
        sizes[2]=-1;
    listPos=0;
    for (j=0, y=0; j<Disp->rows; ++j, ++y){
        for (i=0, x=0; i<Disp->cols; ++i, ++x){
            disp = Disp->m[j][i];
            checkValue =disp-BAD_DISP;
            checkValue2=disp-OCCLUSION;

            if(ISNONZERO(checkValue)&&ISNONZERO(checkValue2)&&
                x<pics[0]->w&&
                (x+disp<pics[1]->w||(pics[2]&&y+disp<pics[2]->h))
                ){
                list->points[listPos].X[0] = x;
                list->points[listPos].Y[0] = y;
                list->points[listPos].X[1] = x+disp;
                list->points[listPos].Y[1] = y;
                list->points[listPos].X[2] = x;
                list->points[listPos].Y[2] = y+disp;
                pos[0]= (i+j*pics[0]->w)*pics[0]->typ;
                pos[1]= (i+ROUND(Disp->m[j][i])+j*pics[1]->w)*pics[1]->typ;
                if(pics[2])
                    pos[2]= (i+(j+ROUND(Disp->m[j][i]))*pics[2]->w)*pics[2]->typ;
                else
                    pos[2]=-1;
                images =0;
                if(pics[0]->typ==COLOR_IMAGE){
                    if(pos[0]>=0 && pos[0]<sizes[0]){
                        images++;
                        r =pics[0]->d[pos[0]];
                        g =pics[0]->d[pos[0]+1];
                        b =pics[0]->d[pos[0]+2];
                    }
                    if(pos[1]>=0 && pos[1]<sizes[1]){
                        r+=pics[1]->d[pos[1]];
                        g+=pics[1]->d[pos[1]+1];
                        b+=pics[1]->d[pos[1]+2];
                        images++;
                    }
                    if(pos[2]>=0 && pos[2]<sizes[2]){
                        r+=pics[2]->d[pos[2]];
                        g+=pics[2]->d[pos[2]+1];
                        b+=pics[2]->d[pos[2]+2];
                        images++;
                    }
                    r=r%images>images/2?1+(r/images):r/images; //round
                    g=g%images>images/2?1+(g/images):g/images; //round
                    b=b%images>images/2?1+(b/images):b/images; //round
                    list->points[listPos].color = COLOR(r, g, b);
                }
                else{
                    if(pos[0]>=0 && pos[0]<sizes[0]){
                        r =pics[0]->d[pos[0]];
                        images++;
                    }
                    if(pos[1]>=0 && pos[1]<sizes[1]){
                        r+=pics[1]->d[pos[1]];
                        images++;
                    }
                    if(pos[2]>=0 && pos[2]<sizes[2]){
                        r+=pics[2]->d[pos[2]];
                        images++;
                    }
                    r=r%images>images/2?1+(r/images):r/images; //round
                    list->points[listPos].color = r;
                }
                listPos++;
            }
        }
    }
    //extract offsets if given!

    if(pics[2]){

        j= MIN(pics[0]->tagc, MIN(pics[1]->tagc,pics[2]->tagc));
        if(j>=2){
            for(j=0; j<3;++j){
                //read tags from pics[j]
                for(i=0; i<pics[j]->tagc;++i){
                    if ((ext = strstr (pics[j]->tagv[i], "offset X: ")) != NULL) {
                        list->offX[j] = atof(ext+10);
                    }
                    if ((ext = strstr (pics[j]->tagv[i], "offset Y: ")) != NULL) {
                        list->offY[j] = atof(ext+10);
                    }
                }
            }
        }
    }
    else{

        j= MIN(pics[0]->tagc, pics[1]->tagc);
        if(j>=2){
            for(j=0; j<2;++j){
                //read tags from pics[j]
                for(i=0; i<pics[j]->tagc;++i){
                    if ((ext = strstr (pics[j]->tagv[i], "offset X: ")) != NULL) {
                        list->offX[j] = atof(ext);
                    }
                    if ((ext = strstr (pics[j]->tagv[i], "offset Y: ")) != NULL) {
                        list->offY[j] = atof(ext);
                    }
                }
            }
        }
    }

    return list;
}

MatchList *AllocMatch(int size){
    MatchList *list;
    list = (MatchList*) halloc(1, sizeof(MatchList));
    list->size = size;
    list->points=(MatchedPoint*)halloc(list->size,sizeof(MatchedPoint));
    list->offX[0]=list->offX[1]=list->offX[2]=NO_OFFSET;
    list->offY[0]=list->offY[1]=list->offY[2]=NO_OFFSET;
    return list;

}

MatchList *FreeMatch(MatchList *list){
    if(list){
		if(list->points) {
            hfree(list->points);
		}
        hfree(list);
    }
	return NULL;
}

//matrix
matrix *mat_alloc (int rows, int cols)
{
    matrix *result;
    int r;

    if ((rows < 1) || (cols < 1)) {
        Warning ("mat_alloc: matrix %d x %d too small", rows, cols); return NULL;
    }
    result = (matrix*) halloc(1, sizeof(matrix));
    if (!result) {
        return NULL;
    }
    result->rows = rows;
    result->cols = cols;
    result->m = (double **) halloc (rows, sizeof (double *));
    if (!result->m) {
        hfree(result); result = NULL;
        return NULL;
    }
    result->m[0] = (double *) halloc (cols * rows, sizeof (double));
    if (!result->m[0]) {
        hfree(result->m); result->m = NULL;
        hfree(result); result = NULL;
        return NULL;
    }
    for (r = 1; r < rows; ++r) {
        result->m[r] = result->m[r-1] + cols;
    }
    return result;
}

/**
 Duplicate a matrix \f$ \mathbf{b}(i,j) = \mathbf{a}(i,j) \f$ for \f$ i=0, \ldots ,r-1 \f$ and \f$ j=0, \ldots ,c-1 \f$.

 @param[in]		A	Source matrix with \f$ r \times c \f$ elements.
 @result			New allocated destination matrix \a B with \f$ r \times c \f$ duplicated elements or NULL.
*/
matrix *mat_clone (matrix *A)
{
    matrix *B = NULL;

    if (CheckMatrix (A)) {
        B = mat_alloc (A->rows, A->cols);
        mat_copy (A, B);
    }
    return B;
}

/**
 Duplicate a submatrix \f$ \mathbf{b}(i,j) = \mathbf{a}(i+rs,j+cs) \f$ for \f$ i=0, \ldots ,er-sr \f$ and \f$ j=0, \ldots ,ec-sc \f$.

 @param[in]		A		Source matrix with \f$ r \times c \f$ elements.
 @param[in]		sr,sc	Start index in \a A.
 @param[in]		er,ec	End index in \a A.
 @result				New allocated destination matrix \a B with duplicated \a er-sr+1 rows and \a ec-sc+1 columns or NULL.
*/
matrix *mat_clone(matrix *A, int sr, int sc, int er, int ec)
{
    matrix *B = NULL;

    if (CheckIndex(A, sr, sc) && CheckIndex(A, er, ec)) {
        if (sr > er) {
            SWAP(int, sr, er);
        }
        if (sc > ec) {
            SWAP(int, sc, ec);
        }
        B = mat_alloc(er-sr+1, ec-sc+1);
        mat_copy(A, sr, sc, er, ec, B);
    }
    return B;
}


// ===========================================================================
//
// ===========================================================================

matrix *homo_mat_cross (homo2 *a, matrix *B)
{
    matrix *A, *C = NULL;

	if (CheckSizeMatrix (B, 3, 3)) {
		A = homo_mat_skew (a);
		C = mat_prod_new (A, B);
		A = mat_free (A);
	}
    return C;
}

matrix *homo_mat_skew (homo2 *h)
{
    matrix *M;

    M = mat_alloc (3, 3);
    M->m[0][1] = -h->w;
    M->m[0][2] =  h->y;
    M->m[1][0] =  h->w;
    M->m[1][2] = -h->x;
    M->m[2][0] = -h->y;
    M->m[2][1] =  h->x;

    return M;
}
/**
 Partial matrix copy \f$ \mathbf{b}(i,j) = \mathbf{a}(i,j) \f$ for \f$ i=0, \ldots ,\min(m,p)-1 \f$ and \f$ j=0, \ldots ,\min(n,q)-1 \f$.

 @param[in]		A	Source matrix with \f$ m \times n \f$ elements.
 @param[out]	B	Overwritten destination matrix with \f$ p \times q \f$ elements.
*/
void mat_copy (matrix *A, matrix *B)
{
    int m, n, p, q, i;

    if (CheckMatrix (A) && CheckMatrix (B)) {
        m = A->rows; n = A->cols;
        p = B->rows; q = B->cols;
        if ((m == p) && (n == q)) {
            memcpy(B->m[0], A->m[0], m*n*sizeof(double));
        } else {
            if ((m > p) || (n > q)) {
                Warning("mat_copy: skipped %d rows and %d cols!", m-p, n-q);
            }
            for (i = 0; i < MIN(m, p); ++i) {
                memcpy(B->m[i], A->m[i], MIN(n, q)* sizeof(double));
            }
        }
    }
}

/**
 Partial matrix copy \f$ \mathbf{b}(i+r,j+c) = \mathbf{a}(i,j) \f$ for \f$ i=0, \ldots ,\min(m,p-r)-1 \f$ and \f$ j=0, \ldots ,\min(n,q-c)-1 \f$.

 @param[in]		A	Source matrix with \f$ m \times n \f$ elements.
 @param[out]	B	Overwritten destination matrix with \f$ p \times q \f$ elements.
 @param[in]		r,c	Start index in matrix \a B.
*/
void mat_copy (matrix *A, matrix *B, int r, int c)
{
    int m, n, p, q, i;

    if (CheckMatrix(A) && CheckIndex(B, r, c)) {
        m = A->rows;     n = A->cols;
        p = B->rows - r; q = B->cols - c;
        if ((m > p) || (n > q)) {
            Warning("mat_copy: skipped %d rows and %d cols!", m-p, n-q);
        }
        for (i = 0; i < MIN(m, p); ++i) {
            memcpy(B->m[i+r]+c, A->m[i], MIN(n, q)* sizeof(double));
        }
    }
}

/**
 Partial matrix copy \f$ \mathbf{b}(i+dr,j+dc) = \mathbf{a}(i+sr,j+sc) \f$ for \f$ i=0, \ldots ,\min(er-sr,p-dr-1)\f$ and \f$ j=0, \ldots ,\min(ec-sc,q-dc-1)\f$.

 @param[in]		A		Source matrix with \f$ m \times n \f$ elements.
 @param[in]		sr,sc	Start index in \a A.
 @param[in]		er,ec	End index in \a A.
 @param[out]	B		Overwritten destination matrix with \f$ p \times q \f$ elements.
 @param[in]		dr,dc	Start index in \a B.
*/
void mat_copy (matrix *A, int sr, int sc, int er, int ec, matrix *B, int dr, int dc)
{
    int m, n, p, q, i;

    if (CheckIndex(A, sr, sc) && CheckIndex(A, er, ec) && CheckIndex(B, dr, dc)) {
        if (sr > er) {
            SWAP(int, sr, er);
        }
        if (sc > ec) {
            SWAP(int, sc, ec);
        }
        m = er-sr+1;     n = ec-sc+1;
        p = B->rows - dr; q = B->cols - dc;
        if ((m > p) || (n > q)) {
            Warning("mat_copy: skipped %d rows and %d cols!", m-p, n-q);
        }
        for (i = 0; i < MIN(m, p); ++i) {
            memcpy(B->m[i+dr]+dc, A->m[i+sr]+sc, MIN(n, q)*sizeof(double));
        }
    }
}

/**
 Partial matrix copy \f$ \mathbf{b}(i,j) = \mathbf{a}(i+sr,j+sc) \f$ for \f$ i=0, \ldots ,\min(er-sr,p-1)\f$ and \f$ j=0, \ldots ,\min(ec-sc,q-1)\f$.

 @param[in]		A		Source matrix with \f$ m \times n \f$ elements.
 @param[in]		sr,sc	Start index in \a A.
 @param[in]		er,ec	End index in \a A.
 @param[out]	B		Overwritten destination matrix with \f$ p \times q \f$ elements.
*/
void mat_copy (matrix *A, int sr, int sc, int er, int ec, matrix *B)
{
    mat_copy (A, sr, sc, er, ec, B, 0, 0);
}

/**
 Partial copy of a matrix column \f$ \mathbf{b}(i,n) = \mathbf{a}(i,m) \f$ for \f$ i=0, \ldots ,\min(m,p)-1\f$.

 @param[in]		A	Source matrix with \f$ m \times n \f$ elements.
 @param[in]		m	Column index in \a A.
 @param[out]	B	Destination matrix with \f$ p \times q \f$ elements and overwritten column.
 @param[in]		n	Column index in \a B.
*/
void mat_copycol (matrix *A, int m, matrix *B, int n)
{
    int i;

    if (CheckColIndex (A, m) && CheckColIndex (B, n)) {
        for (i=0; i<MIN(A->rows, B->rows); ++i) {
             B->m[i][n] = A->m[i][m];
        }
    }
}

/**
 Partial copy of a matrix rows \f$ \mathbf{b}(n,i) = \mathbf{a}(m,i) \f$ for \f$ i=0, \ldots ,\min(n,q)-1\f$.

 @param[in]		A	Source matrix with \f$ m \times n \f$ elements.
 @param[in]		m	rows index in \a A.
 @param[out]	B	Destination matrix with \f$ p \times q \f$ elements and overwritten rows.
 @param[in]		n	rows index in \a B.
*/
void mat_copyrow (matrix *A, int m, matrix *B, int n)
{
    if (CheckRowIndex (A, m) && CheckRowIndex (B, n)) {
        memcpy(B->m[n], A->m[m], MIN(A->cols, B->cols)*sizeof(double));
    }
}

/**
 Fill matrix with given initial value \a v \f$ \mathbf{a}(i,j) = v \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Modified matrix with \f$ m \times n \f$ elements.
 @param[in]		v	Matrix element.
*/
void mat_fill (matrix *A, double v)
{
    double *aptr, *eptr;

    if (CheckMatrix (A)) {
        aptr = A->m[0];
        eptr = aptr + A->cols*A->rows;
        for( ; aptr<eptr;) {
            *aptr++ = v;
        }
    }
}

/**
 Fill submatrix with given initial value \a v \f$ \mathbf{a}(i,j) = v \f$ for \f$ i=sr, \ldots ,er \f$ and \f$ j=sc, \ldots ,ec \f$.

 @param[in,out]	A		Modified matrix with \f$ m \times n \f$ elements.
 @param[in]		sr,sc	Start index in \a A.
 @param[in]		er,ec	End index in \a A.
 @param[in]		v		Matrix element.
*/
void mat_fill (matrix *A, int sr, int sc, int er, int ec, double v)
{
    int i,  j;

    if (CheckIndex (A, sr, sc) && CheckIndex (A, er, ec)) {
        if (sr > er) {
            SWAP(int, sr, er);
        }
        if (sc > ec) {
            SWAP(int, sc, ec);
        }
        for (i = sr; i <= er; ++i) {
            for (j = sc; j <= ec; ++j) {
                A->m[i][j] = v;
            }
        }
    }
}

/**
 Fill matrix column with given initial value \a v \f$ \mathbf{a}(i,n) = v \f$ for \f$ i=0, \ldots ,m-1 \f$.

 @param[in,out]	A	Modified matrix with \f$ m \times n \f$ elements.
 @param[in]		n	Column index in \a A.
 @param[in]		v	Matrix element.
*/
void mat_fillcol (matrix *A, int n, double v)
{
    int i;

    if (CheckColIndex (A, n)) {
        for (i=0; i<A->rows; ++i) {
             A->m[i][n] = v;
        }
    }
}

/**
 Fill matrix rows with given initial value \a v \f$ \mathbf{a}(n,i) = v \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Modified matrix with \f$ m \times n \f$ elements.
 @param[in]		n	rows index in \a A.
 @param[in]		v	Matrix element.
*/
void mat_fillrow (matrix *A, int n, double v)
{
    double *aptr, *eptr;

    if (CheckRowIndex (A, n)) {
        aptr = A->m[0];
        eptr = aptr+A->cols;
        for ( ; aptr<eptr; ) {
            *aptr++ = v;
        }
    }
}

/**
 Free memory of a matrix

 @param[in]		A	Allocated matrix with \f$ m \times n \f$ elements.
 @result			NULL.
*/
matrix *mat_free (matrix *A)
{
    if (A) {
        if (A->m) {
            if (A->m[0]) {
                hfree (A->m[0]); A->m[0] = NULL;
            }
            hfree (A->m); A->m = NULL;
        }
        hfree(A); A = NULL;
    }
    return NULL;
}

/**
 Allocate identity matrix

 @param[in]	r, c	Number of matrix rows and columns.
 @result			Allocated identity matrix with \f$ r \times c \f$ elements or NULL.
*/
matrix *mat_identity (int r, int c)
{
    matrix *A;
    double *aptr, *eptr;

    A = mat_alloc(r, c);
    if (A) {
        aptr = A->m[0];
        eptr = aptr + MIN(c,r)*c;
        for ( ; aptr<eptr; aptr+=c+1) {
            *aptr = 1.0; /* fill diagonal entries */
        }
    }
    return A;
}

/**
 Allocate a square identity matrix

 @param[in]	s	Size of the matrix.
 @result		Allocated identity matrix with \f$ s \times s \f$ elements or NULL.
*/
matrix *mat_identity (int s)
{
    return mat_identity(s, s);
}

/**
 Matrix allocation and initialization with variable number of elements.

 @param[in]		rows  	Number of matrix rows and columns.
 @param[in]		...		Variable list of  \f$ rows \times cols \f$ double values (e.g. 2,2,1.,0.,0.,1.).

 @warning				Number and type of arguments are important but cannot be checked!
*/
matrix *mat_init(int rows, ...)
{
    matrix *A;
    double *aptr, *eptr;
    va_list args;
    int cols;

    va_start(args, rows);
    cols = va_arg(args, int);

    A = mat_alloc(rows, cols);
    if (A) {
        aptr = A->m[0];
        eptr = aptr + rows*cols;
        for (; aptr < eptr; ) {
            *aptr++ = va_arg(args, double);
        }
    }
    va_end(args);

    return A;
}

/**
 Fill matrix with random values \f$ \mathbf{a}(i,j) = {\rm rand}(-1,1) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in,out]		A	Modified input matrix with \f$ m \times n \f$ random elements.
*/
void mat_random(matrix *A)
{
    double *aptr, *eptr;

    if (CheckMatrix(A)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        for (; aptr < eptr; ) {
            *aptr++ = Random(2.0) - 1.0;
        }
    }
}
//@}


/** @name Arithmetic Operations with Scalars */
//@{

/**
 Add scalar to a matrix \f$ \mathbf{a}(i,j) = \mathbf{a}(i,j) + s \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Modified input matrix with \f$ m \times n \f$ elements.
 @param[in]		s	Scalar.
*/
void mat_add (matrix *A, double s)
{
    double *aptr, *eptr;

    if (CheckMatrix (A) && ISNONZERO(s)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        for ( ; aptr<eptr; ) {
            *aptr++ += s;
        }
    }
}

/**
 Add scalar to a matrix \f$ \mathbf{b}(i,j) = \mathbf{a}(i,j) + s \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[in]		s	Scalar.
 @param[out]	B	Overwritten result matrix with \f$ m \times n \f$ added elements.
*/
void mat_add (matrix *A, double s, matrix *B)
{
    double *aptr, *bptr, *eptr;

    if (CheckSameMatrix (A, B)) {
        if (ISZERO(s)) {
            mat_copy(A, B);
        } else {
            aptr = A->m[0];
            eptr = aptr + A->rows*A->cols;
            bptr = B->m[0];
            for ( ; aptr<eptr; ) {
                *bptr++ = *aptr++ + s;
            }
        }
    }
}

/**
 Add scalar to a matrix \f$ \mathbf{b}(i,j) = \mathbf{a}(i,j) + s \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @param[in]		s	Scalar.
 @result			New allocated result matrix \a B with \f$ m \times n \f$ added elements or NULL.
*/
matrix *mat_add_new (matrix *A, double s)
{
    matrix *B = NULL;

    if (CheckMatrix(A)) {
        B = mat_alloc(A->rows, A->cols);
        mat_add(A, s, B);
    }
    return B;
}

/**
 Substract scalar from a matrix \f$ \mathbf{a}(i,j) = \mathbf{a}(i,j) - s \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Modified input matrix with \f$ m \times n \f$ elements.
 @param[in]		s	Scalar.
*/
void mat_sub (matrix *A, double s)
{
    double *aptr, *eptr;

    if (CheckMatrix (A) && ISNONZERO(s)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        for ( ; aptr<eptr; ) {
            *aptr++ -= s;
        }
    }
}

/**
 Substract scalar from a matrix \f$ \mathbf{b}(i,j) = \mathbf{a}(i,j) - s \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[in]		s	Scalar.
 @param[out]	B	Overwritten result matrix with \f$ m \times n \f$ substracted elements.
*/
void mat_sub (matrix *A, double s, matrix *B)
{
    double *aptr, *bptr, *eptr;

    if (CheckSameMatrix (A, B)) {
        if (ISZERO(s)) {
            mat_copy(A, B);
        } else {
            aptr = A->m[0];
            eptr = aptr + A->rows*A->cols;
            bptr = B->m[0];
            for ( ; aptr<eptr; ) {
                *bptr++ = *aptr++ - s;
            }
        }
    }
}

/**
 Substract scalar from a matrix \f$ \mathbf{b}(i,j) = \mathbf{a}(i,j) - s \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @param[in]		s	Scalar.
 @result		New allocated result matrix \a B with \f$ m \times n \f$ substracted elements or NULL.
*/
matrix *mat_sub_new (matrix *A, double s)
{
    matrix *B = NULL;

    if (CheckMatrix(A)) {
        B = mat_alloc(A->rows, A->cols);
        mat_sub(A, s, B);
    }
    return B;
}

/**
 Multiply scalar with a matrix \f$ \mathbf{a}(i,j) = \mathbf{a}(i,j) \cdot s \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Modified input matrix with \f$ m \times n \f$ elements.
 @param[in]		s	Scalar.
*/
void mat_mult (matrix *A, double s)
{
    double *aptr, *eptr;

    if (CheckMatrix (A)) {
        if (ISZERO(s)) {
            mat_fill(A, 0.0);
        } else if (ISINEQUAL(s, 1.0)) {
            aptr = A->m[0];
            eptr = aptr + A->rows*A->cols;
            for ( ; aptr<eptr; ) {
                *aptr++ *= s;
            }
        }
    }
}

/**
 Multiply scalar with a matrix \f$ \mathbf{b}(i,j) = \mathbf{a}(i,j) \cdot s \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[in]		s	Scalar.
 @param[out]	B	Overwritten result matrix with \f$ m \times n \f$ multiplied elements.
*/
void mat_mult (matrix *A, double s, matrix *B)
{
    double *aptr, *bptr, *eptr;

    if (CheckSameMatrix (A, B)) {
        if (ISZERO(s)) {
            mat_fill(B, 0.0);
        } else if (ISEQUAL(s, 1.0)) {
            mat_copy(A, B);
        } else {
            aptr = A->m[0];
            eptr = aptr + A->rows*A->cols;
            bptr = B->m[0];
            for ( ; aptr<eptr; ) {
                *bptr++ = *aptr++ * s;
            }
        }
    }
}

/**
 Multiply scalar with a matrix \f$ \mathbf{b}(i,j) = \mathbf{a}(i,j) \cdot s \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @param[in]		s	Scalar.
 @result		New allocated result matrix \a B with \f$ m \times n \f$ multiplied elements or NULL.
*/
matrix *mat_mult_new (matrix *A, double s)
{
    matrix *B = NULL;

    if (CheckMatrix(A)) {
        B = mat_alloc(A->rows, A->cols);
        mat_mult(A, s, B);
    }
    return B;
}

/**
 Divide matrix by a scalar \f$ \mathbf{a}(i,j) = \mathbf{a}(i,j) / s \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Modified input matrix with \f$ m \times n \f$ elements.
 @param[in]		s	Scalar.
 @result			True, if matrix division was successful.
*/
bool mat_div (matrix *A, double s)
{
    double *aptr, *eptr, f;

    if (CheckMatrix (A)) {
        if (ISZERO(s)) {
            Warning ("mat_div: division by zero!"); return false;
        } else if (ISINEQUAL(s, 1.0)) {
            aptr = A->m[0];
            eptr = aptr + A->rows*A->cols;
            f = 1.0/s;
            for ( ; aptr<eptr; ) {
                *aptr++ *= f;
            }
        }
    }
    return true;
}

/**
 Divide matrix by a scalar \f$ \mathbf{b}(i,j) = \mathbf{a}(i,j) / s \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[in]		s	Scalar.
 @param[out]	B	Overwritten result matrix with \f$ m \times n \f$ divided elements.
 @result			True, if matrix division was successful.
*/
bool mat_div (matrix *A, double s, matrix *B)
{
    double *aptr, *bptr, *eptr, f;

    if (CheckSameMatrix (A, B)) {
        if (ISZERO(s)) {
            Warning ("mat_div: division by zero!");	return false;
        } else if (ISEQUAL(s, 1.0)) {
            mat_copy(A, B);
        } else {
            aptr = A->m[0];
            eptr = aptr + A->rows*A->cols;
            bptr = B->m[0];
            f = 1.0/s;
            for ( ; aptr<eptr; ) {
                *bptr++ = *aptr++ * f;
            }
        }
    }
    return true;
}

/**
 Divide matrix by a scalar \f$ \mathbf{b}(i,j) = \mathbf{a}(i,j) / s \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @param[in]		s	Scalar.
 @result		New allocated result matrix \a B with \f$ m \times n \f$ divided elements or NULL.
*/
matrix *mat_div_new (matrix *A, double s)
{
    matrix *B = NULL;

    if (CheckMatrix(A)) {
        B = mat_alloc(A->rows, A->cols);
        if (!mat_div(A, s, B)) {
            B = mat_free(B);
        }
    }
    return B;
}
//@}

/** @name Arithmetic Element-by-element Operations */
//@{

/**
 Add corresponding elements of two matrices \f$ \mathbf{a}(i,j) = \mathbf{a}(i,j) + \mathbf{b}(i,j) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Modified first matrix with \f$ m \times n \f$ elements.
 @param[in]		B	Second matrix with \f$ m \times n \f$ elements.
*/
void mat_add (matrix *A, matrix *B)
{
    double *aptr, *bptr, *eptr;

    if (CheckSameMatrix (A, B)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        bptr = B->m[0];
        for ( ; aptr<eptr; ) {
            *aptr++ += *bptr++;
        }
    }
}

/**
 Add corresponding elements of two matrices \f$ \mathbf{c}(i,j) = \mathbf{a}(i,j) + \mathbf{b}(i,j) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[in]		B	Second matrix with \f$ m \times n \f$ elements.
 @param[out]	C	Overwritten result matrix with \f$ m \times n \f$ added elements.
*/
void mat_add (matrix *A, matrix *B, matrix *C)
{
    double *aptr, *bptr, *cptr, *eptr;

    if (CheckSameMatrix (A, B) && CheckSameMatrix(A, C)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        bptr = B->m[0];
        cptr = C->m[0];
        for ( ; aptr<eptr; ) {
            *cptr++ = *aptr++ + *bptr++;
        }
    }
}

/**
 Add corresponding elements of two matrices \f$ \mathbf{c}(i,j) = \mathbf{a}(i,j) + \mathbf{b}(i,j) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[in]		B	Second matrix with \f$ m \times n \f$ elements.
 @result			New allocated result matrix \a C with \f$ m \times n \f$ added elements or NULL.
*/
matrix *mat_add_new (matrix *A, matrix *B)
{
    matrix *C = NULL;

    if (CheckMatrix(A)) {
        C = mat_alloc(A->rows, A->cols);
        mat_add(A, B, C);
    }
    return C;
}

/**
 Substract corresponding elements of two matrices \f$ \mathbf{a}(i,j) = \mathbf{a}(i,j) - \mathbf{b}(i,j) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Modified first matrix with \f$ m \times n \f$ elements.
 @param[in]		B	Second matrix with \f$ m \times n \f$ elements.
*/
void mat_sub (matrix *A, matrix *B)
{
    double *aptr, *bptr, *eptr;

    if (CheckSameMatrix (A, B)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        bptr = B->m[0];
        for ( ; aptr<eptr; ) {
            *aptr++ -= *bptr++;
        }
    }
}

/**
 Substract corresponding elements of two matrices \f$ \mathbf{c}(i,j) = \mathbf{a}(i,j) - \mathbf{b}(i,j) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[in]		B	Second matrix with \f$ m \times n \f$ elements.
 @param[out]	C	Overwritten result matrix with \f$ m \times n \f$ substracted elements.
*/
void mat_sub (matrix *A, matrix *B, matrix *C)
{
    double *aptr, *bptr, *cptr, *eptr;

    if (CheckSameMatrix (A, B) && CheckSameMatrix(A, C)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        bptr = B->m[0];
        cptr = C->m[0];
        for ( ; aptr<eptr; ) {
            *cptr++ = *aptr++ - *bptr++;
        }
    }
}

/**
 Substract corresponding elements of two matrices \f$ \mathbf{c}(i,j) = \mathbf{a}(i,j) - \mathbf{b}(i,j) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[in]		B	Second matrix with \f$ m \times n \f$ elements.
 @result			New allocated result matrix \a C with \f$ m \times n \f$ substracted elements or NULL.
*/
matrix *mat_sub_new (matrix *A, matrix *B)
{
    matrix *C = NULL;

    if (CheckMatrix(A)) {
        C = mat_alloc(A->rows, A->cols);
        mat_sub(A, B, C);
    }
    return C;
}

/**
 Multiply corresponding elements of two matrices \f$ \mathbf{a}(i,j) = \mathbf{a}(i,j) \cdot \mathbf{b}(i,j) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Modified first matrix with \f$ m \times n \f$ elements.
 @param[in]		B	Second matrix with \f$ m \times n \f$ elements.
*/
void mat_mult (matrix *A, matrix *B)
{
    double *aptr, *bptr, *eptr;

    if (CheckSameMatrix (A, B)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        bptr = B->m[0];
        for ( ; aptr<eptr; ) {
            *aptr++ *= *bptr++;
        }
    }
}

/**
 Multiply corresponding elements of two matrices \f$ \mathbf{c}(i,j) = \mathbf{a}(i,j) \cdot \mathbf{b}(i,j) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[in]		B	Second matrix with \f$ m \times n \f$ elements.
 @param[out]	C	Overwritten result matrix with \f$ m \times n \f$ multiplied elements.
*/
void mat_mult (matrix *A, matrix *B, matrix *C)
{
    double *aptr, *bptr, *cptr, *eptr;

    if (CheckSameMatrix (A, B) && CheckSameMatrix(A, C)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        bptr = B->m[0];
        cptr = C->m[0];
        for ( ; aptr<eptr; ) {
            *cptr++ = *aptr++ * *bptr++;
        }
    }
}

/**
 Multiply corresponding elements of two matrices \f$ \mathbf{c}(i,j) = \mathbf{a}(i,j) \cdot \mathbf{b}(i,j) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[in]		B	Second matrix with \f$ m \times n \f$ elements.
 @result			New allocated result matrix \a C with \f$ m \times n \f$ multiplied elements or NULL.
*/
matrix *mat_mult_new (matrix *A, matrix *B)
{
    matrix *C = NULL;

    if (CheckMatrix(A)) {
        C = mat_alloc(A->rows, A->cols);
        mat_mult(A, B, C);
    }
    return C;
}

/**
 Divide corresponding elements of two matrices \f$ \mathbf{a}(i,j) = \mathbf{a}(i,j) / \mathbf{b}(i,j) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.
 If a denominator \f$ \mathbf{b}(i,j) \f$ is zero, the corresponding element is set to \f$ [-\infty, 0, \infty] \f$ depending on \f$ \mathbf{a}(i,j) \f$.

 @param[in,out]	A	Modified first matrix with \f$ m \times n \f$ elements.
 @param[in]		B	Second matrix with \f$ m \times n \f$ elements.
*/
void mat_div (matrix *A, matrix *B)
{
    double *aptr, *bptr, *eptr;

    if (CheckSameMatrix (A, B)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        bptr = B->m[0];
        for ( ; aptr < eptr; aptr++, bptr++) {
            *aptr = DIVIDE(*aptr, *bptr);
        }
    }
}

/**
 Divide corresponding elements of two matrices \f$ \mathbf{c}(i,j) = \mathbf{a}(i,j) / \mathbf{b}(i,j) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.
 If a denominator \f$ \mathbf{b}(i,j) \f$ is zero, the corresponding element is set to \f$ [-\infty, 0, \infty] \f$ depending on \f$ \mathbf{a}(i,j) \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[in]		B	Second matrix with \f$ m \times n \f$ elements.
 @param[out]	C	Overwritten result matrix with \f$ m \times n \f$ divided elements.
*/
void mat_div (matrix *A, matrix *B, matrix *C)
{
    double *aptr, *bptr, *cptr, *eptr;

    if (CheckSameMatrix (A, B) && CheckSameMatrix(A, C)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        bptr = B->m[0];
        cptr = C->m[0];
        for (; aptr < eptr; aptr++, bptr++, cptr++) {
            *cptr = DIVIDE(*aptr, *bptr);
        }
    }
}

/**
 Divide corresponding elements of two matrices \f$ \mathbf{c}(i,j) = \mathbf{a}(i,j) / \mathbf{b}(i,j) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.
 If a denominator \f$ \mathbf{b}(i,j) \f$ is zero, the corresponding element is set to \f$ [-\infty, 0, \infty] \f$ depending on \f$ \mathbf{a}(i,j) \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[in]		B	Second matrix with \f$ m \times n \f$ elements.
 @result			New allocated result matrix \a C with \f$ m \times n \f$ divided elements or NULL.
*/
matrix *mat_div_new (matrix *A, matrix *B)
{
    matrix *C = NULL;

    if (CheckMatrix(A)) {
        C = mat_alloc(A->rows, A->cols);
        mat_div(A, B, C);
    }
    return C;
}

/**
 Multiplication of two matrices with \f$ \mathbf{a}(i,j) = \sum\limits_k { \mathbf{a}(i,k) \cdot \mathbf{b}(k,j) } \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Overwritten first matrix with \f$ m \times n \f$ elements.
 @param[in]		B	Second matrix with \f$ n \times n \f$ elements.
*/
void mat_prod (matrix *A, matrix *B)
{
    int r, c, i, m, n;
    double sum;
    matrix *C = NULL;

    if (CheckCompatible(A,B) && CheckSquareMatrix(B)) {
        m = A->rows;
        n = A->cols;
        C = mat_alloc (m, n);
        if (C) {
            for (r = 0; r < m; ++r) {
                for (c = 0; c < n; ++c) {
                    sum = 0.0;
                    for (i = 0; i < n; ++i) {
                        sum += A->m[r][i] * B->m[i][c];
                    }
                    C->m[r][c] = sum;
                }
            }
            mat_swap(C, A);
            C = mat_free(C);
        }
    }
}

matrix *mat_prod_B_is_DIAGMAT (matrix *A, matrix *B)
{
    int    r, c, i;
    double sum;
    matrix *C = NULL;

	if (CheckCompatibleMatrix (A, B)) {
		C = mat_alloc (A->rows, B->cols);
		for (r = 0; r < A->rows; ++r) {
			for (c = 0; c < B->cols; ++c) {
				/*
				sum = 0.0;
				for (i = 0; i < A->cols; ++i) {
					sum += A->m[r][i] * B->m[i][c];
				}
				sum +=
				C->m[r][c] = sum;
				*/
				C->m[r][c] = A->m[r][c] * B->m[c][c];
			}
		}
	}
    return C;
}
/**
 Multiplication of two compatible matrices \f$ \mathbf{c}(i,j) = \sum\limits_k { \mathbf{a}(i,k) \cdot \mathbf{b}(k,j) } \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,q-1 \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[in]		B	Second matrix with \f$ p \times q \f$ elements.
 @result			New allocated result matrix \a C with \f$ m \times q \f$ elements or NULL.
*/
matrix *mat_prod_new (matrix *A, matrix *B)
{
    int r, c, i;
    double sum;
    matrix *C = NULL;

    if (CheckCompatible (A, B)) {
        C = mat_alloc (A->rows, B->cols);
        for (r = 0; r < A->rows; ++r) {
            for (c = 0; c < B->cols; ++c) {
                sum = 0.0;
                for (i = 0; i < A->cols; ++i) {
                    sum += A->m[r][i] * B->m[i][c];
                }
                C->m[r][c] = sum;
            }
        }
    }
    return C;
}
//@}


/** @name Other Element-by-element Operations */
//@{

/**
 Compute the matrix absolute values \f$ \mathbf{a}(i,j) = |\mathbf{a}(i,j)| \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Modified input matrix with \f$ m \times n \f$ elements.
*/
void mat_abs(matrix *A)
{
    double *aptr, *eptr;

    if (CheckMatrix(A)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        for (; aptr < eptr; ) {
            *aptr++ = ABS(*aptr);
        }
    }
}

/**
 Compute the matrix absolute values \f$ \mathbf{b}(i,j) = |\mathbf{a}(i,j)| \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[out]	B	Overwritten result matrix with \f$ m \times n \f$ absolute values.
*/
void mat_abs(matrix *A, matrix *B)
{
    double *aptr, *eptr, *bptr;

    if (CheckSameMatrix(A, B)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        bptr = B->m[0];
        for (; aptr < eptr; aptr++) {
            *bptr++ = ABS(*aptr);
        }
    }
}

/**
 Compute the matrix absolute values \f$ \mathbf{b}(i,j) = |\mathbf{a}(i,j)| \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @result			New allocated result matrix with \f$ m \times n \f$ absolute values or NULL.
*/
matrix *mat_abs_new(matrix *A)
{
    matrix *B = NULL;

    if (CheckMatrix(A)) {
        B = mat_alloc(A->rows, A->cols);
        mat_abs(A, B);
    }
    return B;
}

/**
 Square values of the matrix elements \f$ \mathbf{a}(i,j) = \mathbf{a}(i,j)^2 \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in,out]		A	Modified input matrix with \f$ m \times n \f$ elements.
*/
void mat_sqr(matrix *A)
{
    mat_mult(A, A);
}

/**
 Square values of the matrix elements \f$ \mathbf{b}(i,j) = \mathbf{a}(i,j)^2 \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[out]	B	Overwritten result matrix with \f$ m \times n \f$ squared elements.
*/
void mat_sqr(matrix *A, matrix *B)
{
    mat_mult(A, A, B);
}

/**
 Square values of the matrix elements \f$ \mathbf{b}(i,j) = \mathbf{a}(i,j)^2 \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @result			New allocated result matrix with \f$ m \times n \f$ squared elements or NULL.
*/
matrix *mat_sqr_new(matrix *A)
{
    return mat_mult_new(A, A);
}

/**
 Square roots of the matrix elements \f$ \mathbf{a}(i,j) = \sqrt{\mathbf{a}(i,j)} \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in,out]		A	Modified input matrix with \f$ m \times n \f$ elements.
*/
void mat_sqrt(matrix *A)
{
    double *aptr, *eptr;

    if (CheckMatrix(A)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        for ( ; aptr < eptr; ) {
            *aptr++ = sqrt(*aptr);
        }
    }
}

/**
 Square roots of the matrix elements \f$ \mathbf{b}(i,j) = \sqrt{\mathbf{a}(i,j)} \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[out]	B	Overwritten result matrix with \f$ m \times n \f$ elements.
*/
void mat_sqrt(matrix *A, matrix *B)
{
    double *aptr, *eptr, *bptr;

    if (CheckSameMatrix(A, B)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        bptr = B->m[0];
        for (; aptr < eptr; ) {
            *bptr++ = sqrt(*aptr++);
        }
    }
}

/**
 Square roots of the matrix elements \f$ \mathbf{b}(i,j) = \sqrt{\mathbf{a}(i,j)} \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @result			New allocated result matrix with \f$ m \times n \f$ elements or NULL.
*/
matrix *mat_sqrt_new(matrix *A)
{
    matrix *B = NULL;

    if (CheckMatrix (A)) {
        B = mat_alloc(A->rows, A->cols);
        mat_sqrt (A, B);
    }
    return B;
}
//@}


/** @name Geometric Matrix Operations */
//@{

/**
 The frobenius norm corresponds to the L2-norm \f$ \| \mathbf{a} \|_2 = \sqrt{\sum_{i=0}^{m-1} \sum_{j=0}^{n-1} \mathbf{a}(i,j)^2 } \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @result			Matrix norm.
*/
double mat_frobenius (matrix *A)
{
    double sum = 0.0;
    double *aptr, *eptr;

    if (CheckMatrix (A)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        for ( ; aptr<eptr; ++aptr) {
            sum += SQR(*aptr);
        }
    }
    return sqrt(sum);
}

/**
 The matrix p-norm \f$ \| \mathbf{a} \|_p = \left( \sum_{i=0}^{m-1} \sum_{j=0}^{n-1} | \mathbf{a}(i,j) |^p \right) ^{\frac 1p} \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @param[in]		p	Use L1-, L2-, LInf-norm, Euclidean, max, min or mean value
 @result			Matrix norm.
*/
double mat_norm (matrix *A, norm p)
{
    switch (p) {
        case L1NORM:    return mat_sumabs(A);
        case L2NORM:    return mat_frobenius(A);
        case LINFNORM:  return mat_maxabs(A);
        case FROBENIUS: return mat_frobenius(A);
        case EUCLID:    if (A) return (A->m[0][A->rows*A->cols-1]);
        case MAXNORM:   return mat_max(A);
        case MINNORM:   return mat_min(A);
        case MEANNORM:  return mat_mean(A);
        default:		Warning ("mat_norm: unknown norm!");
    }
    return 0.0;
}

/**
 The standard matrix norm corresponds to the L2-norm \f$ \| \mathbf{a} \|_2 = \sqrt{\sum_{i=0}^{m-1} \sum_{j=0}^{n-1} \mathbf{a}(i,j)^2 } \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @result			Matrix norm.
*/
double mat_norm (matrix *A)
{
    return mat_frobenius (A);
}

/**
 Normalize matrix property to 1.

 @param[in,out]		A	Modified input matrix with \f$ m \times n \f$ elements.
 @param[in]			p	Use L1-, L2-, LInf-norm, max-, min-, mean value, Euclidean \f$ A(m-1,n-1) = 1 \f$ or adaptive normalization \f$ A(i,j) = \frac{(A(i,j)-min)}{(max-min)} \f$
 @result				True, if matrix normalization was successful.
*/
bool mat_normalize(matrix *A, norm p)
{
    double min, max, *aptr, *eptr, len;

    if (p == ADAPTNORM) {
        mat_minmax (A, &min, &max);
        if (max > min) {
            aptr = A->m[0];
            eptr = aptr + A->cols*A->rows;
            for (; aptr<eptr; ) {
                *aptr++ = (*aptr - min) / (max - min);
            }
        }
    } else {
        len = mat_norm(A, p);
        if (ISNONZERO(len)) {
            mat_scale(A, 1.0/len);
            return true;
        }
    }
    return false;
}

/**
 Normalize standard matrix norm to 1 \f$ \mathbf{a} = \mathbf{a} / |\mathbf{a}| \f$.

 @param[in,out]		A	Modified input matrix with \f$ m \times n \f$ elements.
 @result				True, if matrix normalization was successful.
*/
bool mat_normalize(matrix *A)
{
    return mat_normalize(A, L2NORM);
}

/**
 Normalize matrix property to 1.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[in]		p	Use L1-, L2-, LInf-norm, max-, min-, mean value, Euclidean \f$ A(m-1,n-1) = 1 \f$ or adaptive normalization \f$ A(i,j) = \frac{(A(i,j)-min)}{(max-min)} \f$
 @param[out]	B	Overwritten result matrix with \f$ m \times n \f$ normalized elements.
 @result			True, if matrix normalization was successful.
*/
bool mat_normalize(matrix *A, norm p, matrix *B)
{
    double min, max, *aptr, *eptr, len;

    if (p == ADAPTNORM) {
        mat_minmax (A, &min, &max);
        if (max > min) {
            aptr = A->m[0];
            eptr = aptr + A->cols*A->rows;
            for (; aptr<eptr; ) {
                *aptr++ = (*aptr - min) / (max - min);
            }
        }
    } else {
        len = mat_norm(A, p);
        if (ISNONZERO(len)) {
            mat_scale(A, 1.0/len, B);
            return true;
        }
    }
    return false;
}

/**
 Normalize standard matrix norm to 1 \f$ \mathbf{b} = \mathbf{a} / |\mathbf{a}| \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[out]	B	Overwritten result matrix with \f$ m \times n \f$ normalized elements.
 @result			True, if matrix normalization was successful.
*/
bool mat_normalize(matrix *A, matrix *B)
{
    return mat_normalize(A, L2NORM, B);
}

/**
 Normalize new matrix property to 1.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @param[in]		p	Use L1-, L2-, LInf-norm, max-, min-, mean value, Euclidean \f$ A(m-1,n-1) = 1 \f$ or adaptive normalization \f$ A(i,j) = \frac{(A(i,j)-min)}{(max-min)} \f$
 @result			New allocated result matrix with \f$ m \times n \f$ normalized elements or NULL.
*/
matrix *mat_normalize_new(matrix *A, norm p)
{
    matrix *B = NULL;

    if (CheckMatrix(A)) {
        B = mat_alloc(A->rows, A->cols);
        if (!mat_normalize(A, p, B)) {
            B = mat_free(B);
        }
    }
    return B;
}

/**
 Normalize new standard matrix norm to 1 \f$ \mathbf{b} = \mathbf{a} / |\mathbf{a}| \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @result			New allocated result matrix with \f$ m \times n \f$ normalized elements or NULL.
*/
matrix *mat_normalize_new(matrix *A)
{
    return mat_normalize_new(A, L2NORM);
}

/**
 Normalize matrix column property to 1.

 @param[in,out]	A	Modified matrix with \f$ m \times n \f$ elements.
 @param[in]		c	Matrix column index.
 @param[in]		p	Use L1-, L2-, LInf-norm, max-, min-, mean value, Euclidean \f$ A(m-1,c) = 1 \f$ or adaptive normalization \f$ A(i,c) = \frac{(A(i,c)-min)}{(max-min)} \f$
*/
void mat_normalizecol (matrix *A, int c, norm p)
{
    vektor *v;

    v = mat_getcol(A, c);
    vec_normalize(v, p);
    mat_setcol(A, c, v);
    v = vec_free(v);
}

/**
 Normalize matrix properties columnwise to 1.

 @param[in,out]	A	Modified matrix with \f$ m \times n \f$ elements.
 @param[in]		p	Use L1-, L2-, LInf-norm, max-, min-, mean value, Euclidean \f$ A(m-1,c) = 1 \f$ or adaptive normalization \f$ A(i,c) = \frac{(A(i,c)-min)}{(max-min)} \f$
*/
void mat_normalizecol (matrix *A, norm p)
{
    int i;

    if (CheckMatrix (A)) {
        for (i=0; i<A->cols; ++i) {
            mat_normalizecol (A, i, p);
        }
    }
}

/**
 Normalize matrix rows property to 1.

 @param[in,out]	A	Modified matrix with \f$ m \times n \f$ elements.
 @param[in]		r	Matrix rows index.
 @param[in]		p	Use L1-, L2-, LInf-norm, max-, min-, mean value, Euclidean \f$ A(r,n-1) = 1 \f$ or adaptive normalization \f$ A(r,j) = \frac{(A(r,j)-min)}{(max-min)} \f$
*/
void mat_normalizerow (matrix *A, int r, norm p)
{
    vektor *v;

    v = mat_getrow(A, r);
    vec_normalize(v, p);
    mat_setrow(A, r, v);
    v = vec_free(v);
}

/**
 Normalize matrix properties rowwise to 1.

 @param[in,out]	A	Modified matrix with \f$ m \times n \f$ elements.
 @param[in]		p	Use L1-, L2-, LInf-norm, max-, min-, mean value, Euclidean \f$ A(r,n-1) = 1 \f$ or adaptive normalization \f$ A(r,i) = \frac{(A(r,i)-min)}{(max-min)} \f$
*/
void mat_normalizerow (matrix *A, norm p)
{
    int i;

    if (CheckMatrix (A)) {
        for (i=0; i<A->rows; ++i) {
            mat_normalizerow (A, i, p);
        }
    }
}

/**
 The dyadic product of two homogeneous image points is the matrix obtained by multiplying a column vector \a a with a rows vector \a b

\f[ \mathbf{a} \otimes \mathbf{b} \rightarrow \left[ \begin{array}{ccc} a_x \\ a_y \\ a_w \end{array} \right] \left[ \begin{array}{ccc} b_x & b_y & b_w \end{array} \right] = \left[ \begin{array}{ccc} a_xb_x & a_xb_y & a_xb_w \\ a_yb_x & a_yb_y & a_yb_w \\ a_wb_x & a_wb_y & a_wb_w \end{array} \right] \f]

 @param[in]	a, b	Homogeneous image points.
 @result			New allocated result matrix \a C with \f$ 3 \times 3 \f$ elements or NULL.
*/
matrix *mat_dyadic (homo2 *a, homo2 *b)
{
    matrix *A;

    if (!a || !b) return NULL;

    A = mat_alloc (3, 3);
    A->m[0][0] = a->x*b->x; A->m[0][1] = a->x*b->y; A->m[0][2] = a->x*b->w;
    A->m[1][0] = a->y*b->x; A->m[1][1] = a->y*b->y; A->m[1][2] = a->y*b->w;
    A->m[2][0] = a->w*b->x; A->m[2][1] = a->w*b->y; A->m[2][2] = a->w*b->w;

    return A;
}

/**
 The dyadic product of two image points is the matrix obtained by multiplying a column vector \a a with a rows vector \a b

\f[ \mathbf{a} \otimes \mathbf{b} \rightarrow \left[ \begin{array}{ccc} a_x \\ a_y \\ 1 \end{array} \right] \left[ \begin{array}{ccc} b_x & b_y & 1 \end{array} \right] = \left[ \begin{array}{ccc} a_xb_x & a_xb_y & a_x \\ a_yb_x & a_yb_y & a_y \\ b_x & b_y & 1 \end{array} \right] \f]

 @param[in]	a, b	Image points.
 @result			New allocated result matrix \a C with \f$ 3 \times 3 \f$ elements or NULL.
*/
matrix *mat_dyadic (pelem *a, pelem *b)
{
    matrix *A;

    if (!a || !b) return NULL;

    A = mat_alloc (3, 3);
    A->m[0][0] = a->x*b->x; A->m[0][1] = a->x*b->y; A->m[0][2] = a->x;
    A->m[1][0] = a->y*b->x; A->m[1][1] = a->y*b->y; A->m[1][2] = a->y;
    A->m[2][0] =      b->x; A->m[2][1] =      b->y; A->m[2][2] = 1.0;

    return A;
}

/**
 The dyadic product of two homogeneous object points is the matrix obtained by multiplying a column vector \a a with a rows vector \a b

\f[ \mathbf{a} \otimes \mathbf{b} \rightarrow \left[ \begin{array}{cccc} a_x \\ a_y \\ a_z \\ a_w \end{array} \right] \left[ \begin{array}{cccc} b_x & b_y & b_z & b_w \end{array} \right] = \left[ \begin{array}{cccc} a_xb_x & a_xb_y & a_xb_z & a_xb_w \\ a_yb_x & a_yb_y & a_yb_z & a_yb_w \\ a_zb_x & a_zb_y & a_zb_z & a_zb_w \\ a_wb_x & a_wb_y & a_wb_z & a_wb_w \end{array} \right] \f]

 @param[in]	a, b	Homogeneous object points.
 @result			New allocated result matrix \a C with \f$ 4 \times 4 \f$ elements or NULL.
*/
matrix *mat_dyadic (homo3 *a, homo3 *b)
{
    matrix *A;

    if (!a || !b) return NULL;

    A = mat_alloc (4, 4);
    A->m[0][0] = a->X*b->X; A->m[0][1] = a->X*b->Y; A->m[0][2] = a->X*b->Z; A->m[0][3] = a->X*b->W;
    A->m[1][0] = a->Y*b->X; A->m[1][1] = a->Y*b->Y; A->m[1][2] = a->Y*b->Z; A->m[1][3] = a->Y*b->W;
    A->m[2][0] = a->Z*b->X; A->m[2][1] = a->Z*b->Y; A->m[2][2] = a->Z*b->Z; A->m[2][3] = a->Z*b->W;
    A->m[3][0] = a->W*b->X; A->m[3][1] = a->W*b->Y; A->m[3][2] = a->W*b->Z; A->m[3][3] = a->W*b->W;

    return A;
}

/**
 The dyadic product of two object points is the matrix obtained by multiplying a column vector \a a with a rows vector \a b

\f[ \mathbf{a} \otimes \mathbf{b} \rightarrow \left[ \begin{array}{cccc} a_x \\ a_y \\ a_z \\ 1 \end{array} \right] \left[ \begin{array}{cccc} b_x & b_y & b_z & 1 \end{array} \right] = \left[ \begin{array}{cccc} a_xb_x & a_xb_y & a_xb_z & a_x \\ a_yb_x & a_yb_y & a_yb_z & a_y \\ a_zb_x & a_zb_y & a_zb_z & a_z \\ b_x & b_y & b_z & 1 \end{array} \right] \f]

 @param[in]	a, b	Object points.
 @result			New allocated result matrix \a C with \f$ 4 \times 4 \f$ elements or NULL.
*/
matrix *mat_dyadic (xelem *a, xelem *b)
{
    matrix *A;

    if (!a || !b) return NULL;

    A = mat_alloc (4, 4);
    A->m[0][0] = a->X*b->X; A->m[0][1] = a->X*b->Y; A->m[0][2] = a->X*b->Z; A->m[0][3] = a->X;
    A->m[1][0] = a->Y*b->X; A->m[1][1] = a->Y*b->Y; A->m[1][2] = a->Y*b->Z; A->m[1][3] = a->Y;
    A->m[2][0] = a->Z*b->X; A->m[2][1] = a->Z*b->Y; A->m[2][2] = a->Z*b->Z; A->m[2][3] = a->Z;
    A->m[3][0] =      b->X; A->m[3][1] =      b->Y; A->m[3][2] =      b->Z; A->m[3][3] = 1.0;

    return A;
}

/**
 Dyadic vector product \f$ \mathbf{C}(i,j) = \mathbf{a}(i) \otimes \mathbf{b}(j) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		a	First vector with \a m elements.
 @param[in]		b	Second vector with \a n elements.
 @result			New allocated result matrix \a C with \f$ m \times n \f$ elements or NULL.

 @note				Identical with vec_dyadic() function.
*/
matrix *mat_dyadic (vektor *a, vektor *b)
{
    matrix *A;
    int r, c;

    if (!CheckVector(a) || !CheckVector(b)) return NULL;

    A = mat_alloc (a->len, b->len);
    for (r = 0; r < A->rows; ++r) {
        for (c = 0; c < A->cols; ++c) {
            A->m[r][c] = a->v[r] * b->v[c];
        }
    }
    return A;
}

/**
 Matrix scaling \f$ \mathbf{a}(i,j) = \mathbf{a}(i,j) \cdot s \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$ and \f$ s \ne 0 \f$.

 @param[in,out] A	Modified input matrix with \f$ m \times n \f$ elements.
 @param[in]		s	Scaling factor.
*/
void mat_scale (matrix *A, double s)
{
    if (ISZERO(s)) {
        Warning ("mat_scale: scaling factor is zero!"); return;
    }
    mat_mult(A, s);
}

/**
 Matrix scaling \f$ \mathbf{b}(i,j) = \mathbf{a}(i,j) \cdot s \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$ and \f$ s \ne 0 \f$.

 @param[in]		A	First matrix with \f$ m \times n \f$ elements.
 @param[in]		s	Scaling factor.
 @param[out]	B	Overwritten result matrix with \f$ m \times n \f$ scaled elements.
*/
void mat_scale(matrix *A, double s, matrix *B)
{
    if (ISZERO(s)) {
        Warning ("mat_scale: scaling factor is zero!"); return;
    }
    mat_mult(A, s, B);
}

/**
 Matrix scaling \f$ \mathbf{b}(i,j) = \mathbf{a}(i,j) \cdot s \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$ and \f$ s \ne 0 \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @param[in]		s	Scaling factor.
 @result			New allocated result matrix with \f$ m \times n \f$ scaled elements or NULL.
*/
matrix *mat_scale_new(matrix *A, double s)
{
    if (ISZERO(s)) {
        Warning ("mat_scale_new: scaling factor is zero!"); return NULL;
    }
    return mat_mult_new(A, s);
}
//@}


/** @name Matrix Statistics */
//@{

/**
 Maximum matrix element \f$ s = \max( \mathbf{a}(i,j) ) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @result			Maximum element of the matrix.
*/
double mat_max(matrix *A)
{
    double max = -MAX_REAL, *aptr, *eptr;

    if (CheckMatrix(A)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        for (; aptr < eptr; ++aptr) {
            max = MAX(*aptr, max);
        }
    }
    return max;
}

/**
 Matrix absolute maximum corresponds to the Linfty-norm \f$ \| \mathbf{a} \|_\infty = \max( | \mathbf{a}(i,j) | ) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @result			Maximum absolute value of the matrix.
*/
double mat_maxabs(matrix *A)
{
    double max = 0.0, *aptr, *eptr;

    if (CheckMatrix(A)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        for (; aptr < eptr; ++aptr) {
            max = MAX(ABS(*aptr), max);
        }
    }
    return max;
}

/**
 Mean value of the matrix elements \f$ \overline{\mathbf{a}} = \frac 1{m \cdot n} \sum_{i=0}^{m-1} \sum_{j=0}^{n-1} \mathbf{a}(i,j) \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @result			Mean value of the matrix elements.
*/
double mat_mean(matrix *A)
{
    if (CheckMatrix (A)) {
        return (mat_sum(A) / (A->rows*A->cols));
    }
    return 0.0;
}

/**
 Mean of the matrix absolute values \f$ \overline{\mathbf{a}} = \frac 1{m \cdot n} \sum_{i=0}^{m-1} \sum_{j=0}^{n-1} | \mathbf{a}(i) | \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @result			Mean value of the matrix elements.
*/
double mat_meanabs (matrix *A)
{
    if (CheckMatrix(A)) {
        return (mat_sumabs(A) / (A->rows*A->cols));
    }
    return 0.0;
}

/**
 The median is the middle element of the sorted matrix \f$ s = \mathbf{a}(0,0) \le \ldots \le \mathbf{a}(i,j) \le \ldots \le \mathbf{a}(m-1,n-1) \f$ for \f$ i=m/2 \f$ and \f$ j=n/2 \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @result			Median element of the matrix.
*/
double mat_median(matrix *A)
{
    double res = 0.0;
    matrix *B;

    B = mat_sort_new(A);
    if (CheckMatrix(B)) {
        res = B->m[0][B->rows*B->cols / 2];
        B = mat_free(B);
    }
    return res;
}

/**
 The median is the middle element of the sorted matrix \f$ s = | \mathbf{a}(0,0) | \le \ldots \le | \mathbf{a}(i,j) | \le \ldots \le | \mathbf{a}(m-1,n-1) | \f$ for \f$ i=m/2 \f$ and \f$ j=n/2 \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @result			Median element of the absolute matrix.
*/
double mat_medianabs(matrix *A)
{
    double res = 0.0;
    matrix *B;

    B = mat_abs_new(A);
    if (CheckMatrix(B)) {
        mat_sort(B);
        res = B->m[0][B->rows*B->cols / 2];
        B = mat_free(B);
    }
    return res;
}

/**
 Minimum matrix element \f$ s = \min( \mathbf{a}(i,j) ) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @result			Minimum element of the matrix.
*/
double mat_min(matrix *A)
{
    double min = MAX_REAL, *aptr, *eptr;

    if (CheckMatrix(A)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        for (; aptr < eptr; ++aptr) {
            min = MIN(*aptr, min);
        }
    }
    return min;
}

/**
 Minimum matrix absolute value \f$ s = \min( | \mathbf{a}(i,j) | ) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @result			Minimum absolute value of the matrix.
*/
double mat_minabs(matrix *A)
{
    double min = MAX_REAL, *aptr, *eptr;

    if (CheckMatrix(A)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        for (; aptr < eptr; ++aptr) {
            min = MIN(ABS(*aptr), min);
        }
    }
    return min;
}

/**
 Maximum and minimum matrix elements.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @param[out]	min Minimum element of the matrix.
 @param[out]	max Maximum element of the matrix.
*/
void mat_minmax(matrix *A, double *min, double *max)
{
    double *aptr, *eptr;

    *min =  MAX_REAL;
    *max = -MAX_REAL;

    if (CheckMatrix(A)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        for (; aptr < eptr; ++aptr) {
            *min = MIN(*aptr, *min);
            *max = MAX(*aptr, *max);
        }
    }
}

/**
 Maximum and minimum matrix absolute values.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @param[out]	min Minimum absolute value of the matrix.
 @param[out]	max Maximum absolute value of the matrix.
*/
void mat_minmaxabs(matrix *A, double *min, double *max)
{
    double *aptr, *eptr;

    *min = MAX_REAL;
    *max = 0.0;

    if (CheckMatrix(A)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        for (; aptr < eptr; ++aptr) {
            *min = MIN(ABS(*aptr), *min);
            *max = MAX(ABS(*aptr), *max);
        }
    }
}

/**
 Biased standard deviation of the matrix elements \f$ \sigma_n = \sqrt{ \frac 1{m \cdot n} \sum_{i=0}^{m-1} \sum_{j=0}^{n-1} \left(\mathbf{a}(i,j) - \overline{\mathbf{a}} \right)^2} \f$ and mean value \f$ \overline{\mathbf{a}} = \frac 1{m \cdot n} \sum_{i=0}^{m-1} \sum_{j=0}^{n-1} \mathbf{a}(i,j) \f$ <a href="http://www.uni-giessen.de/tomas.sauer/Ankuend/HaFNum.pdf">(Fast version)</a>.

 @param[in]	   A	Matrix with \f$ m \times n \f$ elements.
 @param[out]   mean	Mean value of the elements in \a A.
 @result			Standard deviation of the elements in \a A.
*/
double mat_std(matrix *A, double *mean)
{
    return sqrt(mat_var(A, mean));
}

/**
 Biased standard deviation of the matrix elements \f$ \sigma_n = \sqrt{\frac 1{m \cdot n} \sum_{i=0}^{m-1} \sum_{j=0}^{n-1} \left(\mathbf{a}(i,j) - \overline{\mathbf{a}} \right)^2} \f$ for the mean value \f$ \overline{\mathbf{a}} \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @result			Standard deviation of the elements in \a A.
*/
double mat_std(matrix *A)
{
    return sqrt(mat_var(A));
}

/**
 Unbiased standard deviation of the matrix elements \f$ \sigma_n = \sqrt{\frac 1{(m \cdot n)-1} \sum_{i=0}^{m-1} \sum_{j=0}^{n-1} \left(\mathbf{a}(i,j) - \overline{\mathbf{a}} \right)^2} \f$ and mean value \f$ \overline{\mathbf{a}} = \frac 1{m \cdot n} \sum_{i=0}^{m-1} \sum_{j=0}^{n-1} \mathbf{a}(i,j) \f$ <a href="http://www.uni-giessen.de/tomas.sauer/Ankuend/HaFNum.pdf">(Fast version)</a>.

 @param[in]	   A	Matrix with \f$ n \le 2 \f$ elements.
 @param[out]   mean	Mean value of the elements in \a A.
 @result			Standard deviation of the elements in \a A.
*/
double mat_std2(matrix *A, double *mean)
{
    return sqrt(mat_var2(A, mean));
}

/**
 Unbiased standard deviation of the matrix elements \f$ \sigma_n = \sqrt{\frac 1{(m \cdot n)-1} \sum_{i=0}^{m-1} \sum_{j=0}^{n-1} \left(\mathbf{a}(i,j) - \overline{\mathbf{a}} \right)^2} \f$ for the mean value \f$ \overline{\mathbf{a}} \f$.


 @param[in]		A	Matrix with \f$ n \le 2 \f$ elements.
 @result			Standard deviation of the elements in \a A.
*/
double mat_std2(matrix *A)
{
    return sqrt(mat_var2(A));
}

/**
 Sum all matrix elements \f$ s = \sum_{i=0}^{m-1} \sum_{j=0}^{n-1} \mathbf{a}(i,j) \f$.

 @param[in]		A	Matrix with \f$ n \le 2 \f$ elements.
 @result		Sum of all elements in \a A.
*/
double mat_sum(matrix *A)
{
    double *aptr, *eptr, sum = 0.0;

    if (CheckMatrix (A)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        for (; aptr < eptr; ) {
            sum += *aptr++;
        }
    }
    return sum;
}

/**
 Matrix absolute sum corresponds to the L1-norm \f$ \| \mathbf{a} \|_1 = \sum_{i=0}^{m-1} \sum_{j=0}^{n-1} | \mathbf{a}(i,j) | \f$.

 @param[in]		A	Matrix with \f$ n \le 2 \f$ elements.
 @result		Sum of all absolute values in \a A.
*/
double mat_sumabs(matrix *A)
{
    double *aptr, *eptr, sum = 0.0;

    if (CheckMatrix(A)) {
        aptr = A->m[0];
        eptr = aptr + A->rows*A->cols;
        for (; aptr < eptr; ++aptr) {
            sum += ABS(*aptr);
        }
    }
    return sum;
}

/**
 Biased variance of the matrix elements \f$ \sigma_n^2 = \frac 1{m \cdot n} \sum_{i=0}^{m-1} \sum_{j=0}^{n-1} \left(\mathbf{a}(i,j) - \overline{\mathbf{a}} \right)^ 2 \f$ and mean value \f$ \overline{\mathbf{a}} = \frac 1{m \cdot n} \sum_{i=0}^{m-1} \sum_{j=0}^{n-1} \mathbf{a}(i,j) \f$ <a href="http://www.uni-giessen.de/tomas.sauer/Ankuend/HaFNum.pdf">(Fast version)</a>.

 @param[in]		A		Matrix with \f$ n \le 2 \f$ elements.
 @param[out]	mean	Mean value of the elements in \a A.
 @result		Variance of the elements in \a A.
*/
double mat_var(matrix *A, double *mean)
{
    double var = 0.0, diff, *aptr;
    long i, num;

    *mean = 0.0;
    if (CheckMatrix(A)) {
        num = A->rows*A->cols;
        for (aptr = A->m[0], i=1; i<=num; ++i, ++aptr) {
            diff = *aptr - *mean;
            var += (i-1) * SQR(diff) / i;     // Mixed int/double calculation ...
            *mean += diff / i;
        }
        var /= num;
    }
    return var;
}

/**
 Biased variance of the matrix elements \f$ \sigma_n^2 = \frac 1{m \cdot n} \sum_{i=0}^{m-1} \sum_{j=0}^{n-1} \left(\mathbf{a}(i,j) - \overline{\mathbf{a}} \right)^2 \f$ with mean value \f$ \overline{\mathbf{a}} \f$.

 @param[in]		A	Matrix with \f$ n \le 2 \f$ elements.
 @result		Variance of the elements in \a A.
*/
double mat_var(matrix *A)
{
    double var = 0.0, diff, mean, *aptr;
    long i, num;

    mean = 0.0;
    if (CheckMatrix(A)) {
        num = A->rows*A->cols;
        for (aptr = A->m[0], i=1; i<=num; ++i, ++aptr) {
            diff = *aptr - mean;
            var += (i-1) * SQR(diff) / i;     // Mixed int/double calculation ...
            mean += diff / i;
        }
        var /= num;
    }
    return var;
}

/**
 Unbiased variance of the matrix elements \f$ \sigma_n^2 = \frac 1{(m \cdot n)-1} \sum_{i=0}^{m-1} \sum_{j=0}^{n-1} \left(\mathbf{a}(i,j) - \overline{\mathbf{a}} \right)^ 2 \f$ and mean value \f$ \overline{\mathbf{a}} = \frac 1{m \cdot n} \sum_{i=0}^{m-1} \sum_{j=0}^{n-1} \mathbf{a}(i,j) \f$ <a href="http://www.uni-giessen.de/tomas.sauer/Ankuend/HaFNum.pdf">(Fast version)</a>.

 @param[in]		A		Matrix with \f$ n \le 2 \f$ elements.
 @param[out]	mean	Mean value of the elements in \a A.
 @result		Variance of the elements in \a A.
*/
double mat_var2(matrix *A, double *mean)
{
    double var = 0.0, diff, *aptr;
    long i, num;

    *mean = 0.0;
    if (CheckMatrix(A)) {
        num = A->rows*A->cols;
        if (num < 2) {
            Warning("mat_var2: size %d too small", num);
        } else {
            for (aptr = A->m[0], i=1; i<=num; ++i, ++aptr) {
                diff = *aptr - *mean;
                var += (i-1) * SQR(diff) / i;     // Mixed int/double calculation ...
                *mean += diff / i;
            }
            var /= num - 1;
        }
    }
    return var;
}

/**
 Unbiased variance of the matrix elements \f$ \sigma_n^2 = \frac 1{(m \cdot n)-1} \sum_{i=0}^{m-1} \sum_{j=0}^{n-1} \left(\mathbf{a}(i,j) - \overline{\mathbf{a}} \right)^2 \f$ with mean value \f$ \overline{\mathbf{a}} \f$.

 @param[in]		A	Matrix with \f$ n \le 2 \f$ elements.
 @result		Variance of the elements in \a A.
*/
double mat_var2(matrix *A)
{
    double var = 0.0, diff, mean, *aptr;
    long i, num;

    mean = 0.0;
    if (CheckMatrix(A)) {
        num = A->rows*A->cols;
        if (num < 2) {
            Warning("mat_var2: size %d too small", num);
        } else {
            for (aptr = A->m[0], i=1; i<=num; ++i, ++aptr) {
                diff = *aptr - mean;
                var += (i-1) * SQR(diff) / i;     // Mixed int/double calculation ...
                mean += diff / i;
            }
            var /= num - 1;
        }
    }
    return var;
}
//@}


/** @name Matrix Reorganisation */
//@{

matrix *mat_combine (matrix *A, matrix *B, int rows, int cols)
{
    int r, c;

    if (CheckMatrix (B) &&
        CheckRowIndex (A, rows) &&
        CheckRowIndex (A, rows + B->rows - 1) &&
        CheckColIndex (A, cols) &&
        CheckColIndex (A, cols + B->cols - 1)) {

        for (r = 0; r < B->rows; ++r) {
            for (c = 0; c < B->cols; ++c) {
                A->m[r+rows][c+cols] = B->m[r][c];
            }
        }
    }
    return A;
}

vektor *mat_getcol (matrix *M, int n)
{
    vektor *r = NULL;
    int i;

    if (CheckColIndex (M, n)) {
        r = vec_alloc (M->rows);
        for (i=0; i<M->rows; ++i) {
            r->v[i] = M->m[i][n];
        }
    }
    return r;
}

vektor *mat_getrow (matrix *M, int n)
{
    vektor *r = NULL;

    if (CheckRowIndex (M, n)) {
        r = vec_alloc (M->cols);
        memcpy(r->v, M->m[n], sizeof(double)* M->cols);
    }
    return r;
}

void mat_swap_rows (matrix *M, int row1, int row2)
{

   double *ptr1, *ptr2, *buf;
   long size;

   if (CheckRowIndex (M, row1) && CheckRowIndex (M, row2)) {
       if (row1 != row2) {
          ptr1 = M->m[row1];
          ptr2 = M->m[row2];
          size= M->cols*sizeof(double);
          buf=(double*)halloc(M->cols, sizeof(double));

          memcpy(buf, ptr1, size);
          memcpy(ptr1, ptr2, size);
          memcpy(ptr2, buf, size);

          hfree(buf);
       }
   }
}

void mat_swap_cols (matrix *M, int col1, int col2)
{
   int i;

   if (CheckRowIndex (M, col1) && CheckRowIndex (M, col2)) {
       if (col1 != col2) {
          for (i = 0; i < M->rows; ++i) {
              SWAP(double, M->m[i][col1], M->m[i][col2]);
          }
       }
   }
}

void mat_setcol (matrix *M, int n, vektor *V)
{
    int i;

    if (CheckColIndex (M, n) && CheckVector (V)) {
        for (i=0; i<MIN(M->rows,V->len); ++i) {
             M->m[i][n] = V->v[i];
        }
    }
}

void mat_setrow (matrix *M, int n, vektor *V)
{
    if (CheckRowIndex (M, n) && CheckVector (V)) {
        memcpy(M->m[n], V->v, sizeof(double)*MIN(M->cols,V->len));
    }
}

// ===========================================================================
//
// ===========================================================================

void mat_set_at (matrix *A, int row, int col, double s)
{

	if (CheckMatrix (A)) {
				A->m[row][col] = s;
	}
}

void 	vec_set_at (vektor *A, int r, double s)
{
	if (CheckVector (A)) {
					A->v[r] = s;
		}

}

double 	vec_get_at (vektor *A, int r)
{
	return A->v[r];
}

double mat_get_at (matrix *M, int r, int c)
{

	//if (CheckMatrix (M))
		return (M->m[r][c]);

}

int mat_get_rows(matrix *M) {return M->rows;}
int mat_get_cols(matrix *M) {return M->cols;}

// ===========================================================================
//
// ===========================================================================
matrix *mat_stack_rows (matrix *A, matrix *B)
{
	//put the rows of B und A and return new Mat
	matrix *C;
	if (!CheckMatrix(A) || !CheckMatrix(B)) return NULL;
	if (A->cols != B->cols) return NULL;

	C=mat_alloc(A->rows+B->rows,A->cols);

	int row_count=0;
	for (int r=0;r<A->rows;r++)
	{mat_copyrow(A,r,C,row_count);
	row_count++;
	}
	for (int r=0;r<B->rows;r++)
	{mat_copyrow(B,r,C,row_count);
	row_count++;
	}

	return C;
}

// ===========================================================================
//
// ===========================================================================

bool CheckMatrixIdentity(matrix *A, matrix *B, double tolerance)
{
	int i,j;
	if (!CheckSameMatrix (A,B)) return false;

	for (i=0; i<A->rows; ++i) {
	        for (j=0; j<A->cols; ++j) {
	            if (fabs(A->m[i][j] - B->m[i][j]) > tolerance) {
	                //printf ("matrices not identical!");
	            	//printf("In CheckMatrixIdentity: difference exceeding tol:%f, values: %f and %f\n",fabs(A->m[i][j] - B->m[i][j]),A->m[i][j],B->m[i][j]);
					return false;
	            }
	        }
	    }

	return true;

}

bool CheckMatrixIdentityRelativeError(matrix *A, matrix *B, double tolerance)
{
	int i,j;
	if (!CheckSameMatrix (A,B)) return false;

	for (i=0; i<A->rows; ++i) {
	        for (j=0; j<A->cols; ++j) {
	        	double deno;
	        	if (!ISZERO(A->m[i][j])) deno=A->m[i][j];
	        	else if (!ISZERO(B->m[i][j])) deno=B->m[i][j];
	        	else deno=1;

	            if ((fabs(A->m[i][j] - B->m[i][j]) > tolerance) && ((fabs(fabs(A->m[i][j] - B->m[i][j])/deno) > tolerance))) {
	                //printf ("matrices not identical!");
	            	printf("In CheckMatrixIdentityRelativeError: difference exceeding tol:%f, values: %f and %f, relative error:%f \n",fabs(A->m[i][j] - B->m[i][j]),A->m[i][j],B->m[i][j], fabs(fabs(A->m[i][j] - B->m[i][j])/deno));
					return false;
	            }
	        }
	    }

	return true;

}
// ===========================================================================
//
// ===========================================================================

void mat_set (matrix *A, double s)
{
    int r, c;

	if (CheckMatrix (A)) {
		for (r = 0; r < A->rows; ++r) {
			for (c = 0; c < A->cols; ++c) {
				A->m[r][c] = s;
			}
		}
	}
}

void mat_eye (matrix *A)
{
    int r, c;
    mat_set(A,0);
    if (A->rows != A->cols) {printf("no quadratic mat for eye!\n"); exit;}
    mat_set(A,0);

    for (r = 0; r < A->rows; ++r) {
				A->m[r][r] = 1;
		}
}


int mat_rank (matrix *A)
{
	 matrix  *U, *V;
	 vektor *d;

	 int rank=0;

	if (!mat_svd (A, &U, &d, &V, false)) {printf("error in rank computation (svd)\n"); exit(0);}


	for (int c=0;c<d->len;c++)
			if (!ISZERO(d->v[c])) rank++;

	return rank;
	mat_free(U); vec_free(d); mat_free(V);
	return rank;

}

/**
 Swap the contents of two different matrices.

 @param[in,out]		A	Modified first matrix with \f$ m \times n \f$ elements.
 @param[in,out]		B	Modified second matrix with \f$ p \times q \f$ elements.
*/
void mat_swap(matrix *A, matrix *B)
{
    if (CheckMatrix(A) && CheckMatrix(B)) {
        SWAP(int, A->rows, B->rows);
        SWAP(int, A->cols, B->cols);
        SWAP(double **, A->m, B->m);
    }
}

/**
 Sort matrix elements rowwise into ascending order \f$ \mathbf{a} = \mathbf{a}(0,0) \le \ldots \le \mathbf{a}(m-1,n-1) \f$.

 @param[in,out]	A	Modified input matrix with \f$ m \times n \f$ elements.
*/
void mat_sort(matrix *A)
{
    if (CheckMatrix (A)) {
        Sortdouble(A->m[0], A->rows*A->cols);
    }
}

/**
 Sort matrix elements rowwise into ascending order \f$ \mathbf{b} = \mathbf{a}(0,0) \le \ldots \le \mathbf{a}(m-1,n-1) \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @result			New allocated result matrix with \f$ m \times n \f$ sorted elements or NULL.
*/
matrix *mat_sort_new(matrix *A)
{
    matrix *B;

    B = mat_clone(A);
    mat_sort(B);

    return B;
}

void mat_transpose (matrix *M)
{
    int r, c;

    if (CheckSquareMatrix (M)) {
        for (r = 0; r < M->rows; ++r) {
            for (c = 0; c < M->cols; ++c) {
                if (c < r) {
                    SWAP(double, M->m[c][r], M->m[r][c]);
                }
            }
        }
    }
}

matrix *mat_transpose_new (matrix *M)
{
    matrix *MT = NULL;
    int r, c;

    if (CheckMatrix (M)) {
        MT = mat_alloc(M->cols, M->rows);
        for (r = 0; r < M->rows; ++r) {
            for (c = 0; c < M->cols; ++c) {
                MT->m[c][r] = M->m[r][c];
            }
        }
    }
    return MT;
}

//@}


/** @name Logical Matrix Operations */
//@{

//@}


/** @name Other Operations */
//@{

/**
 Print matrix on screen

 @param[in]		A		Matrix with \f$ m \times n \f$ elements.
*/
void mat_print (matrix *A)
{
    int i, j;

    if (CheckMatrix(A) && report) {
//		Debug ("Matrix size = %d cols x %d rows", A->cols, A->rows);
        for (j=0; j<A->rows; ++j) {
            printf ("%d \t| ",j);
            for (i=0; i<A->cols; ++i) {
                printf ("% e ", ISNONZERO(A->m[j][i]) ? A->m[j][i] : 0.0);
            }
            printf ("|\n");
        }
    }
}

/**
 Print matrix with label on screen

 @param[in]		label	String with the matrix name.
 @param[in]		A		Matrix with \f$ m \times n \f$ elements.
*/
void mat_print (char *label, matrix *A)
{
    Report ("%s\n", label);
    mat_print (A);
}
//@}

// ===========================================================================
//   Adapted version of ludcmp
//   Copyright (C) 1986-2002 Numerical Recipes Software.
//   P.O. Box 380243, Cambridge, MA 02238 (USA), http://www.nr.com/
// ===========================================================================

static bool ludcmp(matrix *a, int *indx, double *d)
{
    int i, imax=0, j, k, n;
    double big, temp;
    vektor *vv;

    *d = 1.0;

    if (!CheckSquareMatrix(a)) return false;

    n = a->rows;
    vv = vec_alloc(n);

    for (i=0; i<n; i++) {
        big = 0.0;
        for (j=0; j<n; j++) {
            if ((temp = ABS(a->m[i][j])) > big) {
                big = temp;
            }
        }
        if (ISZERO(big)) {
//			Warning ("Singular matrix in routine ludcmp");
            vv = vec_free(vv); *d = 0.0;
            return false;
        }
        vv->v[i] = 1.0 / big;
    }
    for (k=0; k<n; k++) {
        big = 0.0;
        for (i=k; i<n; i++) {
            temp = vv->v[i] * ABS(a->m[i][k]);
            if (temp > big) {
                big = temp;
                imax = i;
            }
        }
        if (k != imax) {
            for (j=0; j<n; j++) {
                temp = a->m[imax][j];
                a->m[imax][j] = a->m[k][j];
                a->m[k][j] = temp;
            }
            *d = -*d;
            vv->v[imax] = vv->v[k];
        }
        indx[k] = imax;
        if (ISZERO(a->m[k][k])) {
            a->m[k][k] = TINY;
        }
        for (i=k+1; i<n; i++) {
            temp = a->m[i][k] /= a->m[k][k];
            for (j=k+1; j<n; j++)
                a->m[i][j] -= temp * a->m[k][j];
        }
    }
    vv = vec_free(vv);

    return true;
}

double mat_det (matrix *A)
{
    matrix *T;
    int j, *indx;
    double d = 0.0;

    if (!CheckSquareMatrix (A)) return 0.0;
    if (A->rows == 1) {
        return A->m[0][0];
    } else if (A->rows == 2){
        return (A->m[0][0]*A->m[1][1]-A->m[0][1]*A->m[1][0]);
    }
    /*
    else if (A->rows == 3){
        //Sarrus rule
        d =  A->m[0][0]*A->m[1][1]*A->m[2][2]
            +A->m[1][0]*A->m[2][1]*A->m[0][2]
            +A->m[2][0]*A->m[0][1]*A->m[1][2]
            -A->m[0][2]*A->m[1][1]*A->m[2][0]
            -A->m[1][2]*A->m[2][1]*A->m[0][0]
            -A->m[2][2]*A->m[0][1]*A->m[1][0];
        return d;
    }
    */
    T = mat_clone(A);
    indx = (int *) halloc(T->rows, sizeof(int));
    if (indx) {
        if (ludcmp(T, indx, &d)) {
            for (j=0; j<T->rows; j++) {
                d *= T->m[j][j];
            }
        }
        hfree(indx); indx = NULL;
    }
    T = mat_free(T);

    return d;
}

// ===========================================================================
//
// ===========================================================================

double mat_trace(matrix *A)
{
    double res = 0.0;
    double *aptr, *eptr;

    if (CheckSquareMatrix (A)) {
        aptr=A->m[0];
        eptr= aptr+A->cols*A->rows;
        for(;aptr<eptr;aptr+=A->cols+1)
            res+=*aptr;
    }

    return res;
}

// ===========================================================================
//
// ===========================================================================

matrix *mat_skew (vektor *V)
{
    matrix *M = NULL;

    if (CheckSizeVector (V, 3)) {
        M = mat_alloc (3, 3);
        M->m[0][1] = -V->v[2];
        M->m[0][2] =  V->v[1];
        M->m[1][0] =  V->v[2];
        M->m[1][2] = -V->v[0];
        M->m[2][0] = -V->v[1];
        M->m[2][1] =  V->v[0];
    }
    return M;
}

// ===========================================================================
//
// ===========================================================================

matrix *mat_skew (homo2 *h)
{
    matrix *M;

    M = mat_alloc (3, 3);
    M->m[0][1] = -h->w;
    M->m[0][2] =  h->y;
    M->m[1][0] =  h->w;
    M->m[1][2] = -h->x;
    M->m[2][0] = -h->y;
    M->m[2][1] =  h->x;

    return M;
}

// ===========================================================================
//
// ===========================================================================

matrix *mat_cross (vektor *a, matrix *B)
{
    matrix *A, *C = NULL;

    if (CheckSizeMatrix (B, 3, 3)) {
        A = mat_skew (a);
        C = mat_prod_new (A, B);
        A = mat_free (A);
    }
    return C;
}

// ===========================================================================
//
// ===========================================================================

matrix *mat_cross (homo2 *a, matrix *B)
{
    matrix *A, *C = NULL;

    if (CheckSizeMatrix (B, 3, 3)) {
        A = mat_skew (a);
        C = mat_prod_new (A, B);
        A = mat_free (A);
    }
    return C;
}

// ===========================================================================
//
// ===========================================================================

vektor *mat_pseudo_inverse (matrix *A, vektor *b)
{
    matrix *AT, *ATA;
    vektor *t, *x = NULL;

    if (CheckMatrix (A)) {
        if (!b || (b->len != A->rows)) {
            Error ("mat_pseudo_inverse: empty vektor b"); exit (1);
        }
        AT = mat_transpose_new (A);
        t = vec_transform_new (AT, b);

        ATA = mat_prod_new (AT, A);
        AT = mat_free (AT);

        mat_invert (ATA);

        x = vec_transform_new (ATA, t);
        ATA = mat_free (ATA);
        t = vec_free (t);
    }
    return x;
}

// ===========================================================================
//
// ===========================================================================

// Invert 2 x 2 matrix [a b; c d]
bool mat_invert(double *a, double *b, double *c, double *d)
{
    double r, f;

    r = *a * *d - *b * *c;
    if (ISZERO(r)) {
        return false;
    }
    f = 1.0/r;
    SWAP(double, *a, *d);
    *a *=  f;
    *b *= -f;
    *c *= -f;
    *d *=  f;
    return true;
}

// Invert 2 x 2 symmetric matrix [a b; b c]
bool mat_invert(double *a, double *b, double *c)
{
    double r, f;

    r = *a * *c - SQR(*b);
    if (ISZERO(r)) {
        return false;
    }
    f = 1.0/r;
    SWAP(double, *a, *c);
    *a *=  f;
    *b *= -f;
    *c *=  f;
    return true;
}

// ===========================================================================
//   Adapted version of lubksb
//   Copyright (C) 1986-2002 Numerical Recipes Software.
//   P.O. Box 380243, Cambridge, MA 02238 (USA), http://www.nr.com/
// ===========================================================================

static void lubksb (matrix *a, int *indx, vektor *b)
{
    int i, ii=0, ip, j;
    double sum;
    int n;

    if (!CheckSquareMatrix(a)) return;

    n = a->rows;
    for (i=0; i<n; i++) {
        ip = indx[i];
        sum = b->v[ip];
        b->v[ip] = b->v[i];
        if (ii != 0) {
            for (j=ii-1;j<i;j++) {
                sum -= a->m[i][j]*b->v[j];
            }
        } else if (ISNONZERO(sum)) {
            ii=i+1;
        }
        b->v[i] = sum;
    }
    for (i=n-1; i>=0; i--) {
        sum = b->v[i];
        for (j=i+1; j<n; j++) {
            sum -= a->m[i][j]*b->v[j];
        }
        b->v[i] = sum / a->m[i][i];
    }
}

matrix *mat_invert_new3x3_noCheck (matrix *A){
    matrix *B=NULL;
    double scale = mat_det(A);
    if(ISNONZERO(scale)){
        B= mat_alloc(3, 3);

        B->m[0][0]= A->m[1][1]*A->m[2][2]-A->m[1][2]*A->m[2][1];
        B->m[0][1]= A->m[0][2]*A->m[2][1]-A->m[0][1]*A->m[2][2];
        B->m[0][2]= A->m[0][1]*A->m[1][2]-A->m[0][2]*A->m[1][1];
        B->m[1][0]= A->m[1][2]*A->m[2][0]-A->m[1][0]*A->m[2][2];
        B->m[1][1]= A->m[0][0]*A->m[2][2]-A->m[0][2]*A->m[2][0];
        B->m[1][2]= A->m[0][2]*A->m[1][0]-A->m[0][0]*A->m[1][2];
        B->m[2][0]= A->m[1][0]*A->m[2][1]-A->m[1][1]*A->m[2][0];
        B->m[2][1]= A->m[0][1]*A->m[2][0]-A->m[0][0]*A->m[2][1];
        B->m[2][2]= A->m[0][0]*A->m[1][1]-A->m[0][1]*A->m[1][0];

        mat_scale(B, 1.0/scale);
    }
    else{
        Warning("mat_invert_new: Singular 3x3 Matrix!");
    }
    return B;
}


matrix *mat_invert_new (matrix *A)
{
    matrix *T, *B;
    double d;
    int i,j,n,*indx;
    vektor *cols;

    if (!CheckSquareMatrix(A)) {
        return NULL;
    }
    if(A->cols==2){
        B=mat_clone(A);
        if(!mat_invert(&(B->m[0][0]),&(B->m[0][1]), &(B->m[1][0]), &(B->m[1][1]))){
            B=mat_free(B);
            Warning("mat_invert_new: Singular 2x2 Matrix!");
        }
    }
    else if(A->cols==3){
        B=mat_invert_new3x3_noCheck(A);
    }
    else{
        T = mat_clone(A);
        n = T->rows;
        indx = (int *)halloc(n, sizeof(int));
        if (!ludcmp(T, indx, &d)) {
            mat_free(T);
            if (indx) {
                hfree(indx); indx = NULL;
            }
            return NULL;
        }
        cols = vec_alloc(n);
        B = mat_alloc(A->rows, A->cols);
        for (j=0; j<n; j++) {
            memset(cols->v, 0, n*sizeof(double));

            cols->v[j] = 1.0;
            lubksb(T, indx, cols);
            for (i=0; i<n; i++) {
                B->m[i][j] = cols->v[i];
            }
        }
        mat_free(T);
        if (indx) {
            hfree(indx); indx = NULL;
        }
        cols = vec_free(cols);
    }

    return B;
}

// ===========================================================================
//
// ===========================================================================

bool mat_invert (matrix *A)
{
	bool ret=true;
	matrix *T;

    T = mat_invert_new (A);
	if (T) {
		//tst: A*T != I
		matrix *tst=mat_prod_new(A,T);
		//mat_print("In mat_invert TUB: tst, must be I",tst);
		double min,max;
		mat_minmax(tst,&min,&max);
		//mat_print("TST",tst);
		if (fabs(max) > 1.01) ret=false;
		if (fabs(min) > 0.01) ret=false;
		//printf("in mat_invert: test: min=%e, max=%e, return=%d\n",min,max,ret);
		if (ret) mat_copy(T, A);
		T = mat_free(T);
		tst= mat_free(tst);
	}
	else ret=false;


return ret;
}
// ===========================================================================
//
// ===========================================================================

matrix *mat_diag (vektor *A)
{
    matrix *B = NULL;
    double *aptr;
    int i;

    if (CheckVector(A)) {
        B = mat_alloc(A->len, A->len);
        aptr = A->v;
        for (i=0; i<A->len; ++i) {
             B->m[i][i] = *aptr++;
        }
    }
    return B;
}

// ===========================================================================
//
// ===========================================================================

int mat_threscount (matrix *dat, double thres)
{
    int num = 0;
    double *pos, *end;

    pos= dat->m[0];
    end= pos+dat->cols*dat->rows;

    for(;pos<end;)
        if(*pos++>thres) ++num;

    return num;
}

// ===========================================================================
//
// ===========================================================================

void mat_patch (int x, int y, matrix *dat1, matrix *dat2)
{
    int i, j, x2, y2;

    if(!CheckMatrix(dat1) || !CheckMatrix(dat2))
        return;

    const int w  = dat1->cols, h  = dat1->rows, wp = dat2->cols, hp = dat2->rows;

    if ((x < 0) || (y < 0) || (x+wp > w) || (y+hp > h)) {
        for (i=0; i<hp; ++i) {
            for (j=0; j<wp; ++j) {
                x2 = MIN (MAX (0, x + j), w - 1);
                y2 = MIN (MAX (0, y + i), h - 1);
                dat2->m[i][j] = dat1->m[y2][x2];
            }
        }
    } else {
        for (i=0; i<hp; ++i) {
            memcpy (dat2->m[i], dat1->m[y+1] + x, wp * sizeof(double));
        }
    }
}

//MEsH
mesh *mesh_alloc (int rows, int cols)
{
    mesh *result;
    int r;

    if ((rows < 1) || (cols < 1)) {
		Warning ("mesh_alloc: mesh %d x %d too small", rows, cols); return NULL;
    }
    result = (mesh *) halloc(1, sizeof(mesh));
	if (!result) {
		return NULL;
	}
    result->rows = rows;
    result->cols = cols;
    result->m = (xelem ***) halloc (rows, sizeof (xelem **));
	if (!result->m) {
		hfree(result); result = NULL;
		return NULL;
	}
    result->m[0] = (xelem **) halloc (cols * rows, sizeof (xelem *));
	if (!result->m[0]) {
		hfree(result->m); result->m = NULL;
		hfree(result); result = NULL;
		return NULL;
	}
    for (r = 1; r < rows; ++r) {
        result->m[r] = result->m[r-1] + cols;
    }
    return result;
}

/**
 Free mesh.

 @param[in]		M	Mesh.
 @result			NULL.
*/
mesh *mesh_free (mesh *M)
{
    xelem **aptr, **eptr;

    if (M) {
        if (M->m) {
            if (M->m[0]) {
				aptr = M->m[0];
				eptr = aptr+M->cols*M->rows;
				for (; aptr<eptr; ++aptr) {
					if (*aptr) {
						hfree(*aptr); *aptr = NULL;
					}
				}
                hfree (M->m[0]); M->m[0] = NULL;
            }
            hfree (M->m); M->m = NULL;
        }
        hfree(M); M = NULL;
    }
	return NULL;
}

/**
 Set ID number in the mesh at position (\a r, \a c).

 @param[in,out]	M	Mesh.
 @param[in]		rows	rows position.
 @param[in]		cols	column position.
 @param[in]		id	Point identification number.
*/
void mesh_setid (mesh *M, int rows, int cols, long id)
{
	xelem *X;

	if (CheckRowIndex(M, rows) && CheckColIndex(M, cols)) {
		X = NewCoord();
		if (X) {
			X->id = id;
			M->m[rows][cols] = X;
			M->clean = false;
		}
	}
}

/**
 Search upwards the next ID in the mesh.

 @param[in]		M		Mesh.
 @param[in]		id		Start identification number.
 @param[in]		high	End identification number.
 @result				Nearest ID.
*/
long mesh_next (mesh *M, long id, long high)
{
	for (; (id <= high) && !(M->m[0][id]); ++id); return id;
}

/**
 Search downwards the next ID in the mesh.

 @param[in]		M		Mesh.
 @param[in]		id		Start identification number.
 @param[in]		low		End identification number.
 @result				Nearest ID.
*/
long mesh_previous (mesh *M, long id, long low)
{
	for (; (id >= low) && !(M->m[0][id]); --id); return id;
}

/**
 Binary search for sorted ID numbers in the mesh.

   BinarySearch(A[0..N-1], value) {
       low = 0
       high = N - 1
       while (low <= high) {
           mid = (low + high) / 2
           if (A[mid] > value)
               high = mid - 1
           else if (A[mid] < value)
               low = mid + 1
           else
               return mid // found
       }
       return -1 // not found
   }

 @param[in]		M	Mesh.
 @param[in]		id	Point identification number.
 @result			Point coordinate or NULL.
*/
xelem *mesh_search (mesh *M, long id)
{
	long low = 0, mid, high;
	xelem *ptr;

	if (CheckMesh(M)) {
		high = M->rows * M->cols - 1;
		while (low <= high) {
			mid = mesh_next(M, (low + high)/2, high);
			if (mid > high) {
				mid = mesh_previous(M, (low + high)/2, low);
			}
			if (mid < low) {
				return NULL;       // mid != NULL not found!
			}
			ptr = M->m[0][mid];
			if (ptr->id > id) {
				high = mesh_previous(M, mid-1, low);
			} else if (ptr->id < id) {
				low = mesh_next(M, mid+1, high);
			} else {
				return ptr;
			}
		}
	}
	return NULL;
}

/**
 Set point coordinates in a sorted mesh at position (\a r, \a c).

 @param[in,out]	M		Mesh.
 @param[in]		id	    Point identification number.
 @param[in]		x,y,z	Spatial point position.
 @param[in]		color	Optional RGB point color.
 @param[in]		quality	Optional point reconstruction reliability.

 @note					Next pointer is used to mark only an ID (NULL) or a full point (1)
*/
void mesh_setpoint (mesh *M, long id, double x, double y, double z, int color, double quality)
{
	xelem *ptr;

	ptr = mesh_search (M, id);
	if (ptr) {
		ptr->X       = x;
		ptr->Y       = y;
		ptr->Z       = z;
		ptr->color   = color;
		ptr->quality = quality;
		ptr->next    = (xelem *)1;  // ID with point
	}
}

void mesh_setpoint (mesh *M, long id, double x, double y, double z, int color)
{
	mesh_setpoint (M, id, x, y, z, color, 1.0);
}

void mesh_setpoint (mesh *M, long id, double x, double y, double z, double quality)
{
	mesh_setpoint (M, id, x, y, z, WHITE_COLOR, quality);
}

void mesh_setpoint (mesh *M, long id, double x, double y, double z)
{
	mesh_setpoint (M, id, x, y, z, WHITE_COLOR, 1.0);
}

/**
 Set point coordinates in the sorted mesh at position (\a r, \a c).

 @param[in,out]	M		mesh.
 @param[in]		X		Spatial point position with ID.
 @param[in]		color	Optional RGB point color.
 @param[in]		quality	Optional point reconstruction reliability.

 @note					Next pointer is used to mark only an ID (NULL) or a full point (1)
*/

void mesh_setpoint (mesh *M, homo3 *X, int color, double quality)
{
	NormHomo(X);
	mesh_setpoint (M, X->id, X->X, X->Y, X->Z, color, quality);
}

void mesh_setpoint (mesh *M, homo3 *X, int color)
{
	mesh_setpoint (M, X, color, 1.0);
}

void mesh_setpoint (mesh *M, homo3 *X, double quality)
{
	mesh_setpoint (M, X, WHITE_COLOR, quality);
}

void mesh_setpoint (mesh *M, homo3 *X)
{
	mesh_setpoint (M, X, WHITE_COLOR, 1.0);
}

/**
 Set point coordinates in the mesh at position (\a r, \a c).

 @param[in,out]	M		mesh.
 @param[in]		X		Spatial point with ID, color and quality.

 @note					Next pointer is used to mark only an ID (NULL) or a full point (1)
*/
void mesh_setpoint (mesh *M, xelem *X)
{
	xelem *ptr;

	if (X) {
		ptr = mesh_search (M, X->id);
		if (ptr) {
			CopyCoord(X, ptr);
			ptr->next = (xelem *)1; // ID with point
		}
	}
}

/**
 Set point coordinates, color and quality in the mesh at all positions with IDs.

 @param[in,out]	M		mesh.
 @param[in]		xl		Spatial point list with IDs, colors and qualities.

 @note					Next pointer is used to mark only an ID (NULL) or a full point (1)
*/
void mesh_setpoint (mesh *M, xlist *xl)
{
	xelem **aptr, **eptr, *ptr, *x;

	if (CheckMesh(M) && xl) {
        aptr = M->m[0];
        eptr = aptr + M->cols*M->rows;
		for (x = xl->start; x && (aptr < eptr); x=x->next, ++aptr) {
            while (!(*aptr) && (aptr < eptr)){
                ++aptr;
            }
			ptr = *aptr;
			if (ptr && (ptr->id == x->id)) {
				CopyCoord(x, ptr);
				ptr->next = (xelem *)1; // ID with point
			}
		}
	}
}

/**
 Print info about a mesh.

 @param[in]		M	Mesh.
*/
void mesh_info (mesh *M)
{
	xelem **aptr, **eptr;
	long inum = 0, pnum = 0, size;

	if (CheckMesh(M)) {
		size = M->rows * M->cols;
		Debug ("Mesh size %d x %d = %d nodes:", M->rows, M->cols, size);
        aptr = M->m[0];
        eptr = aptr + size;
		for (; aptr<eptr; ++aptr) {
			if (*aptr) {
				inum++;
				if ((*aptr)->next == (xelem *)1) {
					pnum++;
				}
			}
		}
		if (size) {
			if (M->clean) {
				if (inum) Debug ("%9d ID nodes with points (%.1f%%)", inum, (double)inum*100/size);
			} else {
				if (inum) Debug ("%9d ID nodes (%.1f%%)", inum, (double)inum*100/size);
				if (pnum) Debug ("%9d point nodes (%.1f%%)", pnum, (double)pnum*100/inum);
				if (inum-pnum) Debug ("%9d unset points", inum-pnum);
			}
			if (size-inum) Debug ("%9d free nodes", size-inum);
		}
	}
}

/**
 Clean mesh ID nodes without point information.

 @param[in]		M	Mesh.
*/
void mesh_clean (mesh *M)
{
	xelem **aptr, **eptr;
	long num = 0;

	if (CheckMesh(M)) {
        aptr = M->m[0];
        eptr = aptr + M->cols*M->rows;
		for (; aptr<eptr; ++aptr) {
			if (*aptr && (*aptr)->next == NULL) {
				hfree(*aptr); *aptr = NULL;
				num++;
			}
		}
		M->clean = true;
		if (num) {
			Debug ("Delete %d ID nodes without points", num);
		}
	}
}

/**
 Convolution of mesh points in Z-direction with a filter kernel.

 @param[in,out]	M		Mesh.
 @param[in]		kernel	Filter kernel.

 @note                  ID nodes without points are removed!
*/
void mesh_convolve (mesh *M, mask *kernel)
{
	int i, j, r, num, cols, rows, size;
	xelem **m, **mptr, **mstart, **mend;
	double sum, *t, *tptr, *kptr, *kstart, *kend;
	matrix *T;

	if (CheckMesh(M) && CheckMask(kernel)) {
		if (!M->clean) {
			mesh_clean (M);
		}
		cols = M->cols;
		rows = M->rows;
		T = mat_alloc(rows, cols);
		if (CheckMatrix(T)) {
			r      = kernel->radius;
			size   = kernel->size;
			kstart = kernel->d1;
			kend   = kstart + size;
			for (mstart=mptr=M->m[0], mend=mstart+cols-1, tptr=T->m[0], j=0; j<rows; ++j, mstart+=cols, mend+=cols) {
				for (i=0; i<cols; ++i, ++tptr, ++mptr) {
                    if (*mptr && (*mptr)->next) {
					    sum = 0.0; num = 0;
					    for (kptr = kstart, m = mptr-r; kptr < kend; ++kptr, ++m) {
							if ((m >= mstart) && (m <= mend) && *m) {
								sum += *kptr * (*m)->Z;
								num++;
						    }
					    }
						if (num == size) {
							*tptr = sum;
						} else {
							*tptr = size*sum/num;
						}
                    }
				}
			}
			if (kernel->d2) {
				kstart = kernel->d2;
				kend   = kstart + kernel->size;
			}
			for (mptr=M->m[0], tptr=T->m[0], j=0; j<rows; ++j) {
				for (mstart=M->m[0], mend=mstart+(rows-1)*cols, i=0; i<cols; ++i, ++mptr, ++mstart, ++mend, ++tptr) {
                    if (*mptr && (*mptr)->next) {
						sum = 0.0; num = 0;
						for (kptr=kstart, m=mptr-r*cols, t=tptr-r*cols; kptr < kend; ++kptr, m+=cols, t+=cols) {
							if ((m >= mstart) && (m <= mend) && *m) {
								sum += *kptr * *t;
								num++;
							}
						}
						if (num == size) {
							(*mptr)->Z = sum;
						} else {
							(*mptr)->Z = size*sum/num;
						}
					}
				}
			}
			T = mat_free(T);
		}
	}
}

/**
 Mean filter for mesh points in Z-direction.

 @param[in,out]	M	Mesh.
 @param[in]		r	Filter mask radius r > 0.
*/
void mesh_mean (mesh *M, int r)
{
	mask *kernel;

	kernel = mask_mean(r);
	if (kernel) {
		Debug ("Separated %d x %d mean filter for mesh points ...", kernel->size, kernel->size);
		mesh_convolve (M, kernel);
		kernel = mask_free(kernel);
	}
}

/**
 Gaussian filter for mesh points in Z-direction.

 @param[in,out]	M		Mesh.
 @param[in]		sigma	Standard deviation > 0.
*/
void mesh_gauss (mesh *M, double sigma)
{
	mask *kernel;

	kernel = mask_gauss(sigma);
	if (kernel) {
		Debug ("Separated %d x %d gaussian filter for mesh points ...", kernel->size, kernel->size);
		mesh_convolve (M, kernel);
		kernel = mask_free(kernel);
	}
}

/**
 Two-dimensional accurate median filter for mesh points in Z-direction.

 @param[in,out]	M	Mesh.
 @param[in]		r	Filter mask radius.
*/
void mesh_median (mesh *M, int r)
{
	const int size = 2*r+1;
	int i, j, k, l, num, cols, rows;
	xelem *kptr, **mptr;
	matrix *T;
	vektor *t;
	double *aptr, *eptr;

	if (CheckMesh(M) && (r > 0)) {
		Debug ("Two-dimensional %d x %d median filter for mesh points ...", size, size);
		if (!M->clean) {
			mesh_clean (M);
		}
		cols = M->cols;
		rows = M->rows;
		t = vec_alloc(SQR(size));
		T = mat_alloc(rows, cols);
		if (CheckMatrix(T) && CheckVector(t)) {
			for (mptr=M->m[0], aptr=T->m[0], j=0; j<rows; ++j) {
				for (i=0; i<cols; ++i, ++aptr, ++mptr) {
                    if (*mptr) {
						for (num = 0, eptr=t->v, k=j-r; k<=j+r; ++k) {
							if ((k >= 0) && (k < rows)) {
								for (l=i-r; l<=i+r; ++l) {
									if ((l >= 0) && (l < cols)) {
										kptr = M->m[k][l];
										if (kptr) {
											*eptr++ = kptr->Z;
											num++;
										}
									}
								}
							}
						}
						*aptr = vec_median(t, num);
					}
				}
			}
			t = vec_free(t);

			aptr = T->m[0];
			eptr = aptr + cols*rows;
			for (mptr = M->m[0]; aptr < eptr; ++aptr, ++mptr) {
                if (*mptr) {
					(*mptr)->Z = *aptr;
				}
			}
			T = mat_free(T);
		}
	}
}

/**
 Test mesh functions.
*/
void mesh_testall(void)
{
	int i,j;
	mesh *M;

	M = mesh_alloc(10,10);
	for (j=0; j<10; j++) {
		for (i=0; i<10; i++) {
			if ((i < 7) && (j < 8)) {
				mesh_setid(M, j, i, 1000+j*10+i);
			}
		}
	}
	WriteMesh("test.msh", M);
	mesh_info(M);
	M = mesh_free(M);

	M = ReadMesh("test.msh");
	for (j=0; j<10; j++) {
		for (i=0; i<10; i++) {
			if ((i != 2) || (j != 3)) {
				mesh_setpoint(M, 1000+j*10+i, i, j, Random(10.0), COLOR((byte)Random(255.0), (byte)Random(255.0), (byte)Random(255.0)));
			}
		}
	}
	mesh_info(M);
	WriteWaveFront("test.obj", M);

	mesh_mean(M, 1);
	mesh_median(M, 1);
	mesh_gauss(M, 1.0);

	M = mesh_free(M);
}

//MESSAGE
void Dump (char *fmt, ...)
{
    va_list args;

    va_start (args, fmt);
    vfprintf (stdout, fmt, args);
    va_end (args);
}

// ===========================================================================
//
// ===========================================================================

void Report (char *fmt, ...)
{
    va_list args;

    if (report) {
        va_start (args, fmt);
        vfprintf (stdout, fmt, args);
        fprintf (stdout, "\n");
        va_end (args);
    }
}

// ===========================================================================
//
// ===========================================================================

void Debug (char *fmt, ...)
{
    va_list args;

    if (report == VERBOSE) {
        va_start (args, fmt);
        fprintf (stdout, "DEBUG:  ");
        vfprintf (stdout, fmt, args);
        fprintf (stdout, "\n");
        va_end (args);
    }
}

// ===========================================================================
//
// ===========================================================================

void Warning (char *fmt, ...)
{
    va_list args;

    va_start (args, fmt);
	sprintf (errorbuf, fmt, args);

    if (report) {
        fprintf (stderr, "WARNING: ");
        vfprintf (stderr, fmt, args);
        fprintf (stderr, "\n");
    }
    va_end (args);
}

// ===========================================================================
//
// ===========================================================================

void Error (char *fmt, ...)
{
    va_list args;

    va_start (args, fmt);
	sprintf (errorbuf, fmt, args);

    fprintf (stderr, "ERROR: ");
    vfprintf (stderr, fmt, args);
    fprintf (stderr, "\n");
    va_end (args);

    exit (-1);
}

// ===========================================================================
//
// ===========================================================================

void ReportLine (char *sym, int len)
{
    int i;

    if (report) {
        for (i=0; i<len; ++i) {
            fprintf (stdout, "%s", sym);
        }
        fprintf (stdout, "\n");
    }
}

// ===========================================================================
//
// ===========================================================================

void DebugLine (char *sym, int len)
{
    if (report == VERBOSE) {
        ReportLine (sym, len);
    }
}


static void ComplexDiv(double a, double b, double c, double d, double *r, double *i)
{
    double t;

	t = SQR(c)+SQR(d);
	if (ISNONZERO(t)) {
		*r = (a*c + b*d) / t;
        *i = (b*c - a*d) / t;
	} else {
		Warning ("ComplexDiv: division by zero!");
		*r = *i = 0.0;
	}
}


/**
Given a matrix a->m[0..n-1][0..n-1], this routine replaces it by a balanced matrix with identical
eigenvalues. A symmetric matrix is already balanced and is unaffected by this procedure. The
parameter RADIX should be the machine's floating-point radix.
*/
static void mat_balance (matrix *a, vektor *scale)
{
	int i, j, n;
	double s, r, g, f, c, sqrdx=SQR(RADIX);
	bool done = false;

    if (!CheckSquareMatrix(a)) {
        return;
    }
    n = a->rows;

	while (!done) {
		done = true;
		for (i=0; i<n; ++i) {
			r = c = 0.0;
			for (j=0; j<n; ++j)
				if (j != i) {
					c += ABS(a->m[j][i]);
					r += ABS(a->m[i][j]);
				}
			if (ISNONZERO(c) && ISNONZERO(r)) {
				g = r / RADIX;
				f = 1.0;
				s = c + r;
				while (c < g) {
					f *= RADIX;
					c *= sqrdx;
				}
				g = r * RADIX;
				while (c > g) {
					f /= RADIX;
					c /= sqrdx;
				}
				if ((c + r)/f < 0.95*s) {
					done = false;
					g = 1.0/f;
					if (scale) {
						scale->v[i] *= f;
					}
					for (j=0; j<n; ++j) a->m[i][j] *= g;
					for (j=0; j<n; ++j) a->m[j][i] *= f;
				}
			}
		}
	}
}

static void mat_balbak (matrix *zz, vektor *scale)
{
	int i, j, n;

    if (!CheckSquareMatrix(zz) || !CheckVector(scale)) {
        return;
    }
    n = scale->len;

	for (i=0; i<n; ++i) {
		for (j=0; j<n; ++j) {
			zz->m[i][j] *= scale->v[i];
		}
	}
}

/**
Reduction to Hessenberg form by the elimination method. The double, nonsymmetric matrix
a->m[0..n-1][0..n-1] is replaced by an upper Hessenberg matrix with identical eigenvalues. Recommended,
but not required, is that this routine be preceded by balanc. On output, the
Hessenberg matrix is in elements a->m[i][j] with i <= j+1. Elements with i > j+1 are to be
thought of as zero, but are returned with random values.
*/
static void mat_elmhes (matrix *a, int *perm)
{
	int i,j,m,n;
	double y,x;

    if (!CheckSquareMatrix(a)) {
        return;
    }
    n = a->rows;

	for (m=1; m<n-1; ++m) {
		x = 0.0;
		i = m;
		for (j=m; j<n; ++j) {
			if (ABS(a->m[j][m-1]) > ABS(x)) {
				x=a->m[j][m-1];
				i=j;
			}
		}
		if (perm) {
			perm[m] = i;
		}
		if (i != m) {
			for (j=m-1; j<n; ++j) SWAP(double, a->m[i][j], a->m[m][j]);
			for (j=0; j<n; ++j) SWAP(double, a->m[j][i], a->m[j][m]);
		}
		if (ISNONZERO(x)) {
			for (i=m+1; i<n; ++i) {
                y = a->m[i][m-1];
				if (ISNONZERO(y)) {
					y /= x;
					a->m[i][m-1] = y;
					for (j=m; j<n; ++j) a->m[i][j] -= y*a->m[m][j];
					for (j=0; j<n; ++j) a->m[j][m] += y*a->m[j][i];
				}
			}
		}
	}
/* ???
    for(i=2;i<n;++i){
        for(j=0;j<i-1;++j){
            a->m[i][j]=0;
        }
    }
*/
}

static void mat_eltran(matrix *a, matrix *zz, int *perm)
{
    int mp, k, i, j, n;

    if (!CheckSquareMatrix(a) || !CheckSameMatrix(a,zz) || !perm) {
        return;
    }
    n = a->rows;

	for (mp=n-2; mp>0; mp--) {
		for (k=mp+1; k<n; ++k) {
			zz->m[k][mp] = a->m[k][mp-1];
		}
		i = perm[mp];
		if (i != mp) {
			for (j=mp; j<n; j++) {
				zz->m[mp][j] = zz->m[i][j];
				zz->m[i][j] = 0.0;
			}
			zz->m[i][mp] = 1.0;
		}
	}
}

/**
Finds all eigenvalues of an upper Hessenberg matrix a[0..n-1][0..n-1]. On input a can be
exactly as output from elmhes x11.5; on output it is untouched. The double and imaginary parts
of the eigenvalues are returned in wr[0..n-1] and wi[0..n-1], respectively.
*/
static bool mat_hqr(matrix *a, vektor **wr, vektor **wi)
{
    int nn,m,l,k,j,its,i,mmin,n;
    double z,y,x,w,v,u,t,s,r=0.0,q=0.0,p=0.0,anorm=0.0;

	if (wr && wi) {
		if (!CheckSquareMatrix(a)){
			*wr = *wi = NULL;
			return false;
		}
		n= a->rows;
		*wr = vec_alloc(n);
		*wi = vec_alloc(n);

		for (i=0;i<n;++i)
			for (j=MAX(i-1,0);j<n;++j)
				anorm += ABS(a->m[i][j]);
		nn=n-1;
		t=0.0;

		while (nn >= 0) {
			its=0;
			do {
				for (l=nn;l>0;l--) {
					s=ABS(a->m[l-1][l-1])+ABS(a->m[l][l]);
					if (ISZERO(s)) s=anorm;
// OLD
					if (ABS(a->m[l][l-1]) <= INT_EPS*s) {
/* NEW
					temp=(fabs(a->m[l][l-1]) + s);  //see bug report
					if (ISEQUAL(temp, s)) {
*/
						a->m[l][l-1] = 0.0;
						break;
					}
				}
				x=a->m[nn][nn];
				if (l == nn) {
					(*wr)->v[nn]=x+t;
					(*wi)->v[nn--]=0.0;
				} else {
					y=a->m[nn-1][nn-1];
					w=a->m[nn][nn-1]*a->m[nn-1][nn];
					if (l == nn-1) {
						p=0.5*(y-x);
						q=SQR(p)+w;
						z=sqrt(ABS(q));
						x += t;
						if (q >= 0.0) {
							z=p+SIGN(z,p);
							(*wr)->v[nn-1]=(*wr)->v[nn]=x+z;
							(*wi)->v[nn-1]=(*wi)->v[nn]=0.0;
							if (ISNONZERO(z)) (*wr)->v[nn]=x-w/z;
						} else {
							(*wr)->v[nn]=(*wr)->v[nn-1]=x+p;
							(*wi)->v[nn]=-z;                  // Sign BUG???
							(*wi)->v[nn-1]=z;
						}
						nn -= 2;
					} else {
						if (its == 30) {
							Warning ("Too many iterations in hqr");
							*wr = vec_free(*wr); *wi = vec_free(*wi);
							return false;
						}
						if (its == 10 || its == 20) {
							t += x;
							for (i=0;i<nn+1;++i) a->m[i][i] -= x;
							s=ABS(a->m[nn][nn-1])+ABS(a->m[nn-1][nn-2]);
							y=x=0.75*s;
							w = -0.4375*s*s;
						}
						++its;
						for (m=nn-2;m>=l;m--) {
							z=a->m[m][m];
							r=x-z;
							s=y-z;
							p=(r*s-w) / a->m[m+1][m] + a->m[m][m+1];
							q=a->m[m+1][m+1]-z-r-s;
							r=a->m[m+2][m+1];
							s=ABS(p)+ABS(q)+ABS(r);
							p /= s;
							q /= s;
							r /= s;
							if (m == l) break;
							u=ABS(a->m[m][m-1])*(ABS(q)+ABS(r));
							v=ABS(p)*(ABS(a->m[m-1][m-1])+ABS(z)+ABS(a->m[m+1][m+1]));
// OLD
							if (u <= INT_EPS*v) break;
/* NEW
							temp=u+v; //see bug report
                            if (ISEQUAL(temp, v)) break;
*/
						}
						for (i=m;i<nn-1;++i) {
							a->m[i+2][i]=0.0;
							if (i != m) a->m[i+2][i-1]=0.0;
						}
						for (k=m;k<nn;++k) {
							if (k != m) {
								p=a->m[k][k-1];
								q=a->m[k+1][k-1];
								r=0.0;
								if (k+1 != nn) r=a->m[k+2][k-1];
								x=ABS(p)+ABS(q)+ABS(r);
								if (ISNONZERO(x)) {
									p /= x;
									q /= x;
									r /= x;
								}
							}
							s=SIGN(sqrt(p*p+q*q+r*r),p);
							if (ISNONZERO(s)) {
								if (k == m) {
									if (l != m)
									    a->m[k][k-1] = -a->m[k][k-1];
								} else
									a->m[k][k-1] = -s*x;
								p += s;
								x=p/s;
								y=q/s;
								z=r/s;
								q /= p;
								r /= p;
								for (j=k;j<n;++j) {
									p=a->m[k][j]+q*a->m[k+1][j];
									if (k+1 != nn) {
										p += r*a->m[k+2][j];
										a->m[k+2][j] -= p*z;
									}
									a->m[k+1][j] -= p*y;
									a->m[k][j] -= p*x;
								}
								mmin = nn < k+3 ? nn : k+3;
								for (i=l;i<mmin+1;++i) {
									p=x*a->m[i][k]+y*a->m[i][k+1];
									if (k+1 != nn) {                // see bug report!
										p += z*a->m[i][k+2];
										a->m[i][k+2] -= p*r;
									}
									a->m[i][k+1] -= p*q;
									a->m[i][k] -= p;
								}
							}
						}
					}
				}
			} while (l+1 < nn);
		}
		return true;
	}
	return false;
}

static bool mat_hqr2(matrix *a, matrix *zz, vektor **wr, vektor **wi)
{
    int nn,na,m,l,k,j,its,i,mmin,n;
    double z=0.0,y,x,w,v,u,t,s=0.0,r=0.0,q=0.0,p=0.0,anorm=0.0,ra,sa,vr,vi;

	if (wr && wi) {
		if (!CheckSquareMatrix(a) || !CheckSameMatrix(a, zz)){
			*wr = *wi = NULL;
			return false;
		}
		n = a->rows;
		*wr = vec_alloc(n);
		*wi = vec_alloc(n);

		for (i=0;i<n;++i)
			for (j=MAX(i-1,0);j<n;++j)
				anorm += ABS(a->m[i][j]);
		nn=n-1;
		t=0.0;

		while (nn >= 0) {
			its=0;
			do {
				for (l=nn;l>0;l--) {
					s=ABS(a->m[l-1][l-1])+ABS(a->m[l][l]);
					if (ISZERO(s)) s=anorm;
// OLD
					if (ABS(a->m[l][l-1]) <= INT_EPS*s) {
/* NEW
					temp=(fabs(a->m[l][l-1]) + s);  //see bug report
					if (ISEQUAL(temp, s)) {
*/
						a->m[l][l-1] = 0.0;
						break;
					}
				}
				x=a->m[nn][nn];
				if (l == nn) {
					(*wr)->v[nn]=a->m[nn][nn]=x+t;
					(*wi)->v[nn]=0.0;
					nn--;
				} else {
					y=a->m[nn-1][nn-1];
					w=a->m[nn][nn-1]*a->m[nn-1][nn];
					if (l == nn-1) {
						p=0.5*(y-x);
						q=SQR(p)+w;
						z=sqrt(ABS(q));
						x += t;
						a->m[nn][nn]=x;
						a->m[nn-1][nn-1]=y+t;
						if (q >= 0.0) {
							z=p+SIGN(z,p);
							(*wr)->v[nn-1]=(*wr)->v[nn]=x+z;
							(*wi)->v[nn-1]=(*wi)->v[nn]=0.0;
							if (ISNONZERO(z)) (*wr)->v[nn]=x-w/z;
							x=a->m[nn][nn-1];
							s=ABS(x)+ABS(z);
							p=x/s;
							q=z/s;
							r=sqrt(p*p+q*q);
							p /= r;
							q /= r;
							for (j=nn-1;j<n;j++) {
								z=a->m[nn-1][j];
								a->m[nn-1][j]=q*z+p*a->m[nn][j];
								a->m[nn][j]=q*a->m[nn][j]-p*z;
							}
							for (i=0;i<=nn;i++) {
								z=a->m[i][nn-1];
								a->m[i][nn-1]=q*z+p*a->m[i][nn];
								a->m[i][nn]=q*a->m[i][nn]-p*z;
							}
							for (i=0;i<n;i++) {
								z=zz->m[i][nn-1];
								zz->m[i][nn-1]=q*z+p*zz->m[i][nn];
								zz->m[i][nn]=q*zz->m[i][nn]-p*z;
							}
						} else {
							(*wr)->v[nn]=(*wr)->v[nn-1]=x+p;
							(*wi)->v[nn]=-z;                    // Sign BUG???
							(*wi)->v[nn-1]=z;
						}
						nn -= 2;
					} else {
						if (its == 30) {
							Warning ("Too many iterations in hqr2");
							*wr = vec_free(*wr); *wi = vec_free(*wi);
							return false;
						}
						if (its == 10 || its == 20) {
							t += x;
							for (i=0;i<nn+1;++i) a->m[i][i] -= x;
							s=ABS(a->m[nn][nn-1])+ABS(a->m[nn-1][nn-2]);
							y=x=0.75*s;
							w = -0.4375*s*s;
						}
						++its;
						for (m=nn-2;m>=l;m--) {
							z=a->m[m][m];
							r=x-z;
							s=y-z;
							p=(r*s-w) / a->m[m+1][m] + a->m[m][m+1];
							q=a->m[m+1][m+1]-z-r-s;
							r=a->m[m+2][m+1];
							s=ABS(p)+ABS(q)+ABS(r);
							p /= s;
							q /= s;
							r /= s;
							if (m == l) break;
							u=ABS(a->m[m][m-1])*(ABS(q)+ABS(r));
							v=ABS(p)*(ABS(a->m[m-1][m-1])+ABS(z)+ABS(a->m[m+1][m+1]));
// OLD
							if (u <= INT_EPS*v) break;
/* NEW
							temp=u+v; //see bug report
                            if (ISEQUAL(temp, v)) break;
*/
						}
						for (i=m;i<nn-1;++i) {
							a->m[i+2][i]=0.0;
							if (i != m) a->m[i+2][i-1]=0.0;
						}
						for (k=m;k<nn;++k) {
							if (k != m) {
								p=a->m[k][k-1];
								q=a->m[k+1][k-1];
								r=0.0;
								if (k+1 != nn) r=a->m[k+2][k-1];
								x=ABS(p)+ABS(q)+ABS(r);
								if (ISNONZERO(x)) {
									p /= x;
									q /= x;
									r /= x;
								}
							}
							s=SIGN(sqrt(p*p+q*q+r*r),p);
							if (ISNONZERO(s)) {
								if (k == m) {
									if (l != m)
									    a->m[k][k-1] = -a->m[k][k-1];
								} else
									a->m[k][k-1] = -s*x;
								p += s;
								x=p/s;
								y=q/s;
								z=r/s;
								q /= p;
								r /= p;
								for (j=k;j<n;++j) {
									p=a->m[k][j]+q*a->m[k+1][j];
									if (k+1 != nn) {
										p += r*a->m[k+2][j];
										a->m[k+2][j] -= p*z;
									}
									a->m[k+1][j] -= p*y;
									a->m[k][j] -= p*x;
								}
								mmin = nn < k+3 ? nn : k+3;
								for (i=0;i<mmin+1;++i) {
									p=x*a->m[i][k]+y*a->m[i][k+1];
									if (k+1 != nn) {                // see bug report!
										p += z*a->m[i][k+2];
										a->m[i][k+2] -= p*r;
									}
									a->m[i][k+1] -= p*q;
									a->m[i][k] -= p;
								}
								for (i=0; i<n; i++) {
									p=x*zz->m[i][k]+y*zz->m[i][k+1];
									if (k+1 != nn) {
										p += z*zz->m[i][k+2];
										zz->m[i][k+2] -= p*r;
									}
									zz->m[i][k+1] -= p*q;
									zz->m[i][k] -= p;
								}
							}
						}
					}
				}
			} while (l+1 < nn);
		}
		if (ISNONZERO(anorm)) {
			for (nn=n-1;nn>=0;nn--) {
				p=(*wr)->v[nn];
				q=(*wi)->v[nn];
				na=nn-1;
				if (ISZERO(q)) {
					m=nn;
					a->m[nn][nn]=1.0;
					for (i=nn-1;i>=0;i--) {
						w=a->m[i][i]-p;
						r=0.0;
						for (j=m;j<=nn;j++)
							r += a->m[i][j]*a->m[j][nn];
						if ((*wi)->v[i] < 0.0) {
							z=w;
							s=r;
						} else {
							m=i;

							if (ISZERO((*wi)->v[i])) {
								t=w;
								if (ISZERO(t))
									t=INT_EPS*anorm;
								a->m[i][nn]=-r/t;
							} else {
								x=a->m[i][i+1];
								y=a->m[i+1][i];
								q=SQR((*wr)->v[i]-p)+SQR((*wi)->v[i]);
								t=(x*s-z*r)/q;
								a->m[i][nn]=t;
								if (ABS(x) > ABS(z))
									a->m[i+1][nn]=(-r-w*t)/x;
								else
									a->m[i+1][nn]=(-s-y*t)/z;
							}
							t=ABS(a->m[i][nn]);
							if (INT_EPS*t*t > 1)
								for (j=i;j<=nn;j++)
									a->m[j][nn] /= t;
						}
					}
				} else if (q < 0.0) {
					m=na;
					if (ABS(a->m[nn][na]) > ABS(a->m[na][nn])) {
						a->m[na][na]=q/a->m[nn][na];
						a->m[na][nn]=-(a->m[nn][nn]-p)/a->m[nn][na];
					} else {
						ComplexDiv(0.0, -a->m[na][nn], a->m[na][na]-p, q, &a->m[na][na], &a->m[na][nn]);
					}
					a->m[nn][na]=0.0;
					a->m[nn][nn]=1.0;
					for (i=nn-2;i>=0;i--) {
						w=a->m[i][i]-p;
						ra=sa=0.0;
						for (j=m;j<=nn;j++) {
							ra += a->m[i][j]*a->m[j][na];
							sa += a->m[i][j]*a->m[j][nn];
						}
						if ((*wi)->v[i] < 0.0) {
							z=w;
							r=ra;
							s=sa;
						} else {
							m=i;
							if (ISZERO((*wi)->v[i])) {
								ComplexDiv(-ra, -sa, w, q, &a->m[i][na], &a->m[i][nn]);
							} else {
								x=a->m[i][i+1];
								y=a->m[i+1][i];
								vr=SQR((*wr)->v[i]-p)+SQR((*wi)->v[i])-q*q;
								vi=2.0*q*((*wr)->v[i]-p);
								if (ISZERO(vr) && ISZERO(vi))
									vr=INT_EPS*anorm*(ABS(w)+ABS(q)+ABS(x)+ABS(y)+ABS(z));
                                ComplexDiv(x*r-z*ra+q*sa, x*s-z*sa-q*ra, vr, vi, &a->m[i][na], &a->m[i][nn]);
								if (ABS(x) > ABS(z)+ABS(q)) {
									a->m[i+1][na]=(-ra-w*a->m[i][na]+q*a->m[i][nn])/x;
									a->m[i+1][nn]=(-sa-w*a->m[i][nn]-q*a->m[i][na])/x;
								} else {
									ComplexDiv(-r-y*a->m[i][na], -s-y*a->m[i][nn], z, q, &a->m[i+1][na], &a->m[i+1][nn]);
								}
							}
						}
						t=MAX(ABS(a->m[i][na]),ABS(a->m[i][nn]));
						if (INT_EPS*t*t > 1)
							for (j=i;j<=nn;j++) {
								a->m[j][na] /= t;
								a->m[j][nn] /= t;
							}
					}
				}
			}
			for (j=n-1;j>=0;j--) {
				for (i=0;i<n;i++) {
					z=0.0;
					for (k=0;k<=j;k++)
						z += zz->m[i][k]*a->m[k][j];
					zz->m[i][j]=z;
				}
			}
		}
		return true;
	}
	return false;
}

static void mat_sortvecs(matrix *zz, vektor *wr, vektor *wi)
{
	int i, j, k, n;
	double xr, xi;
	vektor *t;

	if (!CheckSquareMatrix(zz) || !CheckVector(wr) || !CheckVector(wi)) {
		return;
	}
	n = zz->rows;
	t = vec_alloc(n);

	for (j=1; j<n; j++) {
		xr = wr->v[j];
		xi = wi->v[j];
		for (k=0; k<n; k++) {
			t->v[k] = zz->m[k][j];
		}
		for (i=j-1; i>=0; i--) {
			if (wr->v[i] >= xr) break;
			wr->v[i+1] = wr->v[i];
			wi->v[i+1] = wi->v[i];
			for (k=0; k<n; k++) {
				zz->m[k][i+1] = zz->m[k][i];
			}
		}
		wr->v[i+1] = xr;
		wi->v[i+1] = xi;
		for (k=0; k<n; k++) {
			zz->m[k][i+1] = t->v[k];
		}
	}
	t = vec_free(t);
}

bool mat_eigen(matrix *aa, matrix **eigVec, vektor **eigVal)
{
	int i, j, n, sols=0, *perm=NULL;
	vektor *wr, *wi, *scale, *t;
	matrix *a, *zz;
    double len;

	if (!CheckSquareMatrix(aa) || !eigVec || !eigVal) {
	    return false;
	}
	*eigVec = NULL; *eigVal = NULL;

	a = mat_clone(aa);
	n = a->rows;

	scale = vec_alloc(n);
	for (i=0; i<n; ++i) {
		scale->v[i] = 1.0;
	}
	mat_balance(a, scale);

	perm = (int *) halloc(n, sizeof(int));
	mat_elmhes(a, perm);

	zz = mat_identity(n);
	mat_eltran(a, zz, perm); free (perm); perm = NULL;
	if (!mat_hqr2(a, zz, &wr, &wi)) {
		a = mat_free(a); zz = mat_free(zz); return false;
	}
	a = mat_free(a);
	mat_balbak(zz, scale); scale = vec_free(scale);
	mat_sortvecs(zz, wr, wi);

	for (i=0; i<n; ++i) {
		if(ISZERO(wi->v[i]) && ISNONZERO(wr->v[i])) {
			sols++;
		}
	}
	if (sols) {
		*eigVal = vec_alloc(sols);
		*eigVec = mat_alloc(n, sols);
		for (i=0, j=0; i<n && j<sols; ++i) {
			if (ISZERO(wi->v[i]) && ISNONZERO(wr->v[i])) {
				(*eigVal)->v[j] = wr->v[i];
				t = mat_getcol(zz, i);
                len= vec_len(t);
                if(ISNONZERO(len))
    		        vec_scale(t,1.0/len, t);
				mat_setcol(*eigVec, j++, t); t = vec_free(t);
			}
		}
	}
	zz = mat_free(zz);
	wr = vec_free(wr);
	wi = vec_free(wi);
	if (sols) {
		return true;
	}
	return false;
}

// ===========================================================================
//
// ===========================================================================

static void EvalEpipol (matrix *F, homo2 *e, plist *pl)
{
    homo2 t;
	pelem *p;

    InitError ();
    for (p=pl->start; p; p=p->next) {
        t = TransformPointNew (F, p);
        AccumulateError (HomoMult(&t, e));
    }
    PrintError ();
}

// ===========================================================================
//
// ===========================================================================

void EvalEpipoles (matrix *F, homo2 *e1, homo2 *e2, plist *pl1, plist *pl2)
{
    homo2 t;
    matrix *FT;

	if (report == VERBOSE) {
		if (!pl1 || !pl2 || (pl1->num < 1) || (pl1->num != pl2->num)) {
			Warning ("EvalEpipoles: No data !"); return;
		}
		t = TransformHomo (F, e1);
		if (HomoLen(&t) > INT_EPS) {
			Warning ("Eval F e1 = 0: %e %e %e", t.x, t.y, t.w);

			Debug ("Eval e2^T F x1 = 0:");
			EvalEpipol (F, e2, pl1);
		}
		FT = mat_transpose_new (F);

		t = TransformHomo (FT, e2);
		if (HomoLen(&t) > INT_EPS) {
			Warning ("Eval F^T e2 = 0: %e %e %e", t.x, t.y, t.w);

			Debug ("Eval e1^T F^T x2 = 0:");
			EvalEpipol (FT, e1, pl2);
		}
		FT = mat_free (FT);
	}
}

// ===========================================================================
//
// ===========================================================================

void CalcEpipoles (matrix *F, homo2 *e1, homo2 *e2)
{
    vektor *V;

	if (mat_svd_nullvector (F, &V)) {
		*e1 = VecToNormHomo2 (V);
		V = vec_free (V);
	} else {
		Warning ("CalcEpipoles failed!");
		*e1 = NOHOMO2;
	}
	if (mat_svd_left_nullvector (F, &V)) {
		*e2 = VecToNormHomo2 (V);
		V = vec_free (V);
	} else {
		Warning ("CalcEpipoles failed!");
		*e2 = NOHOMO2;
	}
}

// ===========================================================================
//   (x',y') = (x/w, y/w, 1), w != 0
// ===========================================================================

//NORM
bool NormHomo (homo2 *h)
{
    double t;

	if (h) {
		t = h->w;
		if (ISNONZERO(h->w)) {
            t = 1.0/t;
			h->x *= t;
			h->y *= t;
			h->w = 1.0;
			return true;
		}
	}
	return false;
}

// ===========================================================================
//   (x',y',z') = (x/w, y/w, z/w, 1), w != 0
// ===========================================================================

bool NormHomo (homo3 *h)
{
    double t;

	if (h) {
		t = h->W;
		if (ISNONZERO(t)) {
            t = 1.0/t;
			h->X *= t;
			h->Y *= t;
			h->Z *= t;
			h->W = 1.0;
			return true;
		}
	}
	return false;
}

// ===========================================================================
//
// ===========================================================================

static void FindCenter (homo2 *h, int num, trans *t)
{
    int i;
    double div;

	if (h && t && (num > 0)) {
		t->tx = t->ty = 0.0;
        div= 1.0/(double) num;
		for (i=0; i<num; ++i) {
			t->tx += h[i].x;
			t->ty += h[i].y;
		}
		t->tx *= div;
		t->ty *= div;
	}
}

static void FindCenter (plist *pl, trans *t)
{
	pelem *p;
    double div;

	if (pl && t && (pl->num > 0)) {
		t->tx = t->ty = 0.0;
        div= 1.0/(double) pl->num;
		for (p=pl->start; p; p=p->next) {
			t->tx += p->x;
			t->ty += p->y;
		}
		t->tx *= div;
		t->ty *= div;
	}
}

// ===========================================================================
//
// ===========================================================================

static void FindCenter (homo3 *h, int num, trans *t)
{
    int i;
    double div;

	if (h && t && (num > 0)) {
		t->tx = t->ty = t->tz = 0.0;
        div= 1.0/(double) num;
		for (i=0; i<num; ++i) {
			t->tx += h[i].X;
			t->ty += h[i].Y;
			t->tz += h[i].Z;
		}
		t->tx *= div;
		t->ty *= div;
		t->tz *= div;
	}
}

static void FindCenter (xlist *xl, trans *t)
{
    xelem *x;
    double div;

	if (xl && t && (xl->num > 0)) {
		t->tx = t->ty = t->tz = 0.0;
        div= 1.0/(double) xl->num;
		for (x=xl->start; x; x=x->next) {
			t->tx += x->X;
			t->ty += x->Y;
			t->tz += x->Z;
		}
		t->tx *= div;
		t->ty *= div;
		t->tz *= div;
	}
}

// ===========================================================================
//
// ===========================================================================

static void FindScale (homo2 *h, int num, trans *t)
{
    int i;
    double div;

	if (h && t && (num > 0)) {
		t->sx = t->sy = 0.0;
        div = 1.0/(double) num;
		for (i=0; i<num; ++i) {
			t->sx += fabs(h[i].x - t->tx);
			t->sy += fabs(h[i].y - t->ty);
		}
		t->sx *= div;
		t->sy *= div;
	}
}

static void FindScale (plist *pl, trans *t)
{
    pelem *p;
    double div;

	if (pl && t && (pl->num > 0)) {
		t->sx = t->sy = 0.0;
        div = 1.0/(double) pl->num;
		for (p=pl->start; p; p=p->next) {
			t->sx += ABS(p->x - t->tx);
			t->sy += ABS(p->y - t->ty);
		}
		t->sx *= div;
		t->sy *= div;
	}
}

// ===========================================================================
//
// ===========================================================================

static void FindScale (homo3 *h, int num, trans *t)
{
    int i;
    double div;

	if (h && t && (num > 0)) {
		t->sx = t->sy = t->sz = 0.0;
        div = 1.0/(double) num;
		for (i=0; i<num; ++i) {
			t->sx += fabs(h[i].X - t->tx);
			t->sy += fabs(h[i].Y - t->ty);
			t->sz += fabs(h[i].Z - t->tz);
		}
		t->sx *= div;
		t->sy *= div;
		t->sz *= div;
	}
}

static void FindScale (xlist *xl, trans *t)
{
    xelem *x;
    double div;

	if (xl && t && (xl->num > 0)) {
		t->sx = t->sy = t->sz = 0.0;
        div = 1.0/(double) xl->num;
		for (x=xl->start; x; x=x->next) {
			t->sx += fabs(x->X - t->tx);
			t->sy += fabs(x->Y - t->ty);
			t->sz += fabs(x->Z - t->tz);
		}
		t->sx *= div;
		t->sy *= div;
		t->sz *= div;
	}
}

// ===========================================================================
//
// ===========================================================================

matrix *SimilarityMatrix2 (trans *t)
{
    matrix *T;
	double s;

    T = mat_identity (3);
	if (t) {
		T->m[0][2] = -t->tx;
		T->m[1][2] = -t->ty;
		s = t->sx;
		if (ISNONZERO(s)) {
			T->m[0][0] /= s; T->m[0][2] /= s;
		}
		s = t->sy;
		if (ISNONZERO(s)) {
			T->m[1][1] /= s; T->m[1][2] /= s;
		}
	}
    return T;
}

// ===========================================================================
//
// ===========================================================================

matrix *SimilarityMatrix3 (trans *t)
{
    matrix *T;
	double s;

    T = mat_identity (4);
	if (t) {
		T->m[0][3] = -t->tx;
		T->m[1][3] = -t->ty;
		T->m[2][3] = -t->tz;
		s = t->sx;
		if (ISNONZERO(s)) {
			T->m[0][0] /= s; T->m[0][3] /= s;
		}
		s = t->sy;
		if (ISNONZERO(s)) {
			T->m[1][1] /= s; T->m[1][3] /= s;
		}
		s = t->sz;
		if (ISNONZERO(s)) {
			T->m[2][2] /= s; T->m[2][3] /= s;
		}
	}
    return T;
}

// ===========================================================================
//
// ===========================================================================

matrix *InvSimilarityMatrix2 (trans *t)
{
    matrix *T;

    T = mat_identity (3);
	if (t) {
		T->m[0][0] = t->sx; T->m[0][2] = t->tx;
		T->m[1][1] = t->sy; T->m[1][2] = t->ty;
	}
    return T;
}

// ===========================================================================
//
// ===========================================================================

matrix *InvSimilarityMatrix3 (trans *t)
{
    matrix *T;

    T = mat_identity (4);
	if (t) {
		T->m[0][0] = t->sx; T->m[0][3] = t->tx;
		T->m[1][1] = t->sy; T->m[1][3] = t->ty;
		T->m[2][2] = t->sz; T->m[2][3] = t->tz;
	}
    return T;
}

// ===========================================================================
//
// ===========================================================================

void CondHomo (homo2 *h, int num, homo2 **hn, trans *t)
{
    matrix *H;
    int i;

	*hn = NULL;
	if (h && t && (num > 0)) {
		FindCenter (h, num, t);
		FindScale (h, num, t);

		H = SimilarityMatrix2 (t);

		*hn = (homo2 *) halloc (num, sizeof (homo2));
		for (i=0; i<num; ++i) {
			(*hn)[i] = TransformNormPoint (H, &h[i]);
		}
		H = mat_free (H);
	}
}

void CondHomo (plist *pl, plist **pn, trans *t)
{
    matrix *H;
	pelem *p;
	homo2 h;

	*pn = NULL;
	if (pl && t && (pl->num > 0)) {
		FindCenter (pl, t);
		FindScale (pl, t);

		H = SimilarityMatrix2 (t);

		for (p=pl->start; p; p=p->next) {
			h = TransformPointNew (H, p);
			*pn = AddPoint(*pn, &h);
		}
		H = mat_free (H);
	}
}

// ===========================================================================
//
// ===========================================================================

void CondHomo (homo3 *h, int num, homo3 **hn, trans *t)
{
    matrix *H;
    int i;

	*hn = NULL;
	if (h && t && (num > 0)) {
		FindCenter (h, num, t);
		FindScale (h, num, t);

		H = SimilarityMatrix3 (t);

		*hn = (homo3 *) halloc (num, sizeof (homo3));
		for (i=0; i<num; ++i) {
			(*hn)[i] = TransformNormCoord (H, &h[i]);
		}
		H = mat_free (H);
	}
}

void CondHomo (xlist *xl, xlist **xn, trans *t)
{
    matrix *H;
	xelem *x;
	homo3 h;

	*xn = NULL;
	if (xl && t && (xl->num > 0)) {
		FindCenter (xl, t);
		FindScale (xl, t);

		H = SimilarityMatrix3 (t);

		for (x=xl->start; x; x=x->next) {
			h = TransformCoordNew (H, x);
			*xn = AddCoord(*xn, &h);
		}
		H = mat_free (H);
	}
}

// ===========================================================================
//
// ===========================================================================

void EvalRotation (matrix *R)
{
    double t;
    int i;

	if (report == VERBOSE) {
		CheckMatrix(R);

		for (i=0; i<3; ++i) {
			t = SQR(R->m[i][0]) + SQR(R->m[i][1]) + SQR(R->m[i][2]);
			if (fabs(1.0 - t) > FTOL) {
				Warning ("Sum in rows %d: %e", i+1, t);
			}
			t = SQR(R->m[0][i]) + SQR(R->m[1][i]) + SQR(R->m[2][i]);
			if (fabs(1.0 - t) > FTOL) {
				Warning ("Sum in cols %d: %e", i+1, t);
			}
		}
		if (fabs(R->m[0][0]*R->m[0][1] + R->m[1][0]*R->m[1][1] + R->m[2][0]*R->m[2][1]) > FTOL ||
			fabs(R->m[0][1]*R->m[0][2] + R->m[1][1]*R->m[1][2] + R->m[2][1]*R->m[2][2]) > FTOL ||
			fabs(R->m[0][2]*R->m[0][0] + R->m[1][2]*R->m[1][0] + R->m[2][2]*R->m[2][0]) > FTOL) {
			Warning ("Illegal Rotation Matrix ?");
		}
		t = mat_det(R);
		if (fabs(1.0 - t) > FTOL) {
			Warning ("det(R) = 1: %e", t);
		}
	}
}

// ===========================================================================
//
// ===========================================================================

bool NormMatrix (matrix *A)
{
	double t;

    if (CheckMatrix (A)) {
		t = A->m[A->rows-1][A->cols-1];
		if (ISNONZERO(t)) {
			mat_scale (A, 1.0 / t);
			return true;
		}
	}
	return false;
}

// ===========================================================================
//   | E3 | = 1
// ===========================================================================

bool NormEssentialMatrix (matrix *A)
{
	double t;

    if (CheckMatrix (A)) {
		t = sqrt(SQR(A->m[2][0]) + SQR(A->m[2][1]) + SQR(A->m[2][2]));
		if (ISNONZERO(t)) {
			mat_scale (A, 1.0 / t);
			return true;
		}
	}
	return false;
}

// ===========================================================================
//
// ===========================================================================

bool NormVector (vektor *A)
{
	double t;

    if (CheckVector (A)) {
		t = A->v[A->len-1];
		if (ISNONZERO(t)) {
			vec_scale (A, 1.0 / t, A);
			return true;
		}
	}
	return false;
}

// ===========================================================================
//
// ===========================================================================

bool NormCamera (matrix *P)
{
    matrix *M;
    double t;

    if (CheckMatrix (P)) {
		t = sqrt(SQR(P->m[2][0]) + SQR(P->m[2][1]) + SQR(P->m[2][2]));
		if (ISNONZERO(t)) {

			//Debug ("Scale Camera Matrix with %f", 1.0 / t);
			mat_scale (P, 1.0 / t);

			M = mat_alloc (3,3);
			mat_copy (P, 0, 0, 2, 2, M);
			if (mat_det(M) < 0) {
		        //Debug ("Flip Camera Matrix Sign");
				mat_scale (P, -1.0);
			}
			M = mat_free (M);
			return true;
		}
	//	EvalCamera (P);
	}
	return false;
}

// ===========================================================================
//
// ===========================================================================

matrix *CenterCamera (matrix *P, int w, int h)
{
    matrix *T, *M;
    trans t;

    CheckMatrix (P);

    t.tx = (double)w * 0.5;
    t.ty = (double)h * 0.5;
    t.sx = t.sy = (double)(w + h);

    T = SimilarityMatrix2 (&t);
    M = mat_prod_new (T, P);

    NormCamera (M);

    return M;
}

// ===========================================================================
//
// ===========================================================================

matrix *DecondHomo (trans *t1, trans *t2, matrix *H1)
{
    matrix *T, *HT, *H2;

    if (!CheckMatrix (H1)) return NULL;

	if (t1) {
		if (H1->cols == 3) {
			T = SimilarityMatrix2 (t1);
		} else if (H1->cols == 4) {
			T = SimilarityMatrix3 (t1);
		} else {
			Warning ("DenormalizeHomo: illegal transformation"); return H1;
		}
		HT = mat_prod_new (H1, T);
		T = mat_free (T);
	} else {
		HT = mat_clone (H1);
	}
	if (t2) {
		if (HT->rows == 3) {
			T = InvSimilarityMatrix2 (t2);
		} else if (HT->rows == 4) {
			T = InvSimilarityMatrix3 (t2);
		} else {
			Warning ("DenormalizeHomo: illegal transformation"); return H1;
		}
		H2 = mat_prod_new (T, HT);
		T = mat_free (T);
	} else {
		H2 = mat_clone (HT);
	}
	HT = mat_free (HT);
    NormMatrix (H2);

    return H2;
}

// ===========================================================================
//
// ===========================================================================

matrix *DecondRelativeOrientation (trans *t1, trans *t2, matrix *F1)
{
    matrix *T, *FT, *F2;

    if (!CheckMatrix (F1)) return NULL;

	if (t1) {
		T = SimilarityMatrix2 (t1);
		FT = mat_prod_new (F1, T);
		T = mat_free (T);
	} else {
		FT = mat_clone (F1);
	}
	if (t2) {
		T = SimilarityMatrix2 (t2);
		mat_transpose (T);
		F2 = mat_prod_new (T, FT);
		T = mat_free (T);
	} else {
		F2 = mat_clone (FT);
	}
    FT = mat_free (FT);
    NormMatrix (F2);

    return F2;
}

//POINT
static plist *InsertPoint (plist *list, pelem *m)
{
    if (CheckPoints(m)) {
        if (list) {
            PointList(list); // Convert compact array to linked list

            if (list->end) {
                list->end->next = m;
            } else {
                list->start = m;
            }
            list->end = m;
            list->num++;
        } else {
            list = NewPointList();
            if (list) {
                list->start = list->end = m;
                list->num = 1;
            }
        }
    }
    return list;
}

/**
 Append a point to the end of a point list.

 @param[in]		list	Point list or NULL.
 @param[in]		x, y    Point coordinates.
 @param[in]		id		Identification number.
 @param[in]		color	RGB point color.
 @param[in]		quality	Estimated point reliability.
 @result				Extended point list.
 @note					Compact array will be converted to a linked list!
*/
plist *AddPoint (plist *list, double x, double y, long id, int color, double quality)
{
    pelem *m;

    m = NewPoint ();
    if (m) {
        m->x       = x;
        m->y       = y;
        m->id      = id;
        m->color   = color;
        m->quality = quality;
        m->next    = NULL;
    }
    return (InsertPoint (list, m));
}

/**
 Append a point with quality 1.0 to the end of a point list.

 @param[in]		list	Point list or NULL.
 @param[in]		x, y    Point coordinates.
 @param[in]		id		Identification number.
 @param[in]		color	RGB point color.
 @result				Extended point list.
 @note					Compact array will be converted to a linked list!
*/
plist *AddPoint (plist *list, double x, double y, long id, int color)
{
    return AddPoint (list, x, y, id, color, 1.0);
}

/**
 Append a white point to the end of a point list.

 @param[in]		list	Point list or NULL.
 @param[in]		x, y    Point coordinates.
 @param[in]		id		Identification number.
 @param[in]		quality	Estimated point reliability.
 @result				Extended point list.
 @note					Compact array will be converted to a linked list!
*/
plist *AddPoint (plist *list, double x, double y, long id, double quality)
{
    return AddPoint (list, x, y, id, WHITE_COLOR, quality);
}

/**
 Append a white point with quality 1.0 to the end of a point list.

 @param[in]		list	Point list or NULL.
 @param[in]		x, y    Point coordinates.
 @param[in]		id		Identification number.
 @result				Extended point list.
 @note					Compact array will be converted to a linked list!
*/
plist *AddPoint (plist *list, double x, double y, long id)
{
    return AddPoint (list, x, y, id, WHITE_COLOR, 1.0);
}

/**
 Append a homogeneous point to the end of a point list.

 @param[in]		list	Point list or NULL.
 @param[in]		p		Homogeneous point with identification number.
 @param[in]		color	RGB point color.
 @param[in]		quality	Estimated point reliability.
 @result				Extended point list.
 @note					Compact array will be converted to a linked list!
*/
plist *AddPoint (plist *list, homo2 *p, int color, double quality)
{
	if (p) {
		NormHomo(p);
		return AddPoint (list, p->x, p->y, p->id, color, quality);
	}
	return list;
}

/**
 Append a homogeneous point with quality 1.0 to the end of a point list.

 @param[in]		list	Point list or NULL.
 @param[in]		p		Homogeneous point with identification number.
 @param[in]		color	RGB point color.
 @result				Extended point list.
 @note					Compact array will be converted to a linked list!
*/
plist *AddPoint (plist *list, homo2 *p, int color)
{
    return AddPoint (list, p, color, 1.0);
}

/**
 Append a white homogeneous point to the end of a point list.

 @param[in]		list	Point list or NULL.
 @param[in]		p		Homogeneous point with identification number.
 @param[in]		quality	Estimated point reliability.
 @result				Extended point list.
 @note					Compact array will be converted to a linked list!
*/
plist *AddPoint (plist *list, homo2 *p, double quality)
{
    return AddPoint (list, p, WHITE_COLOR, quality);
}

/**
 Append a white homogeneous point with quality 1.0 to the end of a point list.

 @param[in]		list	Point list or NULL.
 @param[in]		p		Homogeneous point with identification number.
 @result				Extended point list.
 @note					Compact array will be converted to a linked list!
*/
plist *AddPoint (plist *list, homo2 *p)
{
    return AddPoint (list, p, WHITE_COLOR, 1.0);
}

/**
 Append a point to the end of a point list.

 @param[in]		list	Point list or NULL.
 @param[in]		p		Point element with identification number, color and quality.
 @result				Extended point list.
 @note					Compact array will be converted to a linked list!
*/
plist *AddPoint (plist *list, pelem *p)
{
    return InsertPoint (list, ClonePoint (p));
}

/**
 Append a corner to the end of a point list.

 @param[in]		list	Point list or NULL.
 @param[in]		c		Corner element with identification number and value.
 @result				Extended point list.
 @note					Compact array will be converted to a linked list!
*/
plist *AddPoint (plist *list, celem *c)
{
	if (c) {
		return AddPoint (list, c->x, c->y, c->id, WHITE_COLOR, c->value);
	}
	return list;
}

/**
 Append all unknown IDs from pointsToAdd to the end of a point list.

 @param[in]		list	        Point list or NULL.
 @param[in]		pointsToAdd		Point list with new IDs.
 @result				Extended point list.
 @note					Compact array will be converted to a linked list!
*/

plist *AddOnlyNewPointIDs(plist *list, plist *pointsToAdd){
    pelem *pe[2]={NULL};
    plist *result;

    if(!CheckPoints(pointsToAdd))
        return list;
    if(!CheckPoints(list))
        return ClonePoint(pointsToAdd);

    SortPoint(list);
    result = ClonePoint(list);

    for(pe[1]=pointsToAdd->start; pe[1]; pe[1]= pe[1]->next){
        pe[0] = GetPointID(list, pe[1]->id);
        if(!pe[0])
            result= AddPoint(result, pe[1]);
    }
    SortPoint(result);
    SwapPoint(list, result);
    result = FreePoint(result);
    return list;

}

/**
 Duplicate a point element.

 @param[in]		p		Point element with identification number, color and quality.
 @result				New allocated point with duplicated elements and no successor or NULL.
*/
pelem *ClonePoint (pelem *p)
{
    pelem *q = NULL;

    if (CheckPoints(p)) {
        q = NewPoint ();
        if (q) {
            memcpy(q, p, sizeof(pelem));
            q->next = NULL;
        }
    }
    return q;
}

/**
 Duplicate to a point array.

 @param[in]		pl		Point list.
 @result				New allocated compact point array with duplicated elements or NULL.
*/
plist *ClonePointArray (plist *pl)
{
    plist *list = NULL;

    if (CheckPoints(pl)) {
        list = NewPointList(pl->num);
        CopyPoint(pl, list);
    }
    return list;
}

/**
 Duplicate to a point list.

 @param[in]		pl		Point list.
 @result				New allocated linked point list with duplicated elements or NULL.
*/
plist *ClonePointList (plist *pl)
{
    plist *list = NULL;
    pelem *p;

    if (CheckPoints(pl)) {
        for (p = pl->start; p; p = p->next) {
            list = AddPoint(list, p);
        }
    }
    return list;
}

/**
 Duplicate a point list or array.

 @param[in]		pl		Point list.
 @result				New allocated linked point list with duplicated elements or NULL.
*/
plist *ClonePoint (plist *pl)
{
    if (CheckPoints(pl)) {
		if (pl->compact) {
			return ClonePointArray(pl);
		} else {
			return ClonePointList(pl);
		}
	}
	return NULL;
}

/**
 Partial duplicate to a point array.

 @param[in]		pl		Point list with \a n elements.
 @param[in]		s		Start index.
 @param[in]		e		End index.
 @result				New allocated compact point array with \f$ e-s+1 \f$ duplicated elements or NULL.
*/
plist *ClonePointArray (plist *pl, int s, int e)
{
    plist *list = NULL;

    if (CheckIndex(pl, s) && CheckIndex(pl, e)) {
        if (s > e) {
            SWAP(int, s, e);
        }
        list = NewPointList(e-s+1);
        CopyPoint(pl, s, e, list);
    }
    return list;
}

/**
 Partial duplicate to a point list.

 @param[in]		pl		Point list with \a n elements.
 @param[in]		s		Start index.
 @param[in]		e		End index.
 @result				New allocated linked point list with \f$ e-s+1 \f$ duplicated elements or NULL.
*/
plist *ClonePointList (plist *pl, int s, int e)
{
    plist *list = NULL;
    pelem *p;
    long i, num;

    if (CheckIndex(pl, s) && CheckIndex(pl, e)) {
        if (s > e) {
            SWAP(int, s, e);
        }
        num = e-s+1;
        p = GetPointPos(pl, s);
        if (p) {
            if (pl->compact) {
                for (i=0; (p<=pl->end) && (i<num); ++i, ++p) {
                    list = AddPoint(list, p);
                }
            } else {
                for (i=0; p && (i<num); ++i, p=p->next) {
                    list = AddPoint(list, p);
                }
            }
        }
    }
    return list;
}

/**
 Partial duplicate of a point list or array.

 @param[in]		pl		Point list with \a n elements.
 @param[in]		s		Start index.
 @param[in]		e		End index.
 @result				New allocated linked point list with \f$ e-s+1 \f$ duplicated elements or NULL.
*/
plist *ClonePoint (plist *pl, int s, int e)
{
    if (CheckPoints(pl)) {
		if (pl->compact) {
			return ClonePointArray(pl, s, e);
		} else {
			return ClonePointList(pl, s, e);
		}
	}
	return NULL;
}

/**
 Copy a point element.

 @param[in]		p		Point element with identification number, color and quality.
 @param[out]	q		Overwritten point with duplicated elements and old successor.
*/
void CopyPoint (pelem *p, pelem *q)
{
    pelem *next;

    if (CheckPoints(p) && CheckPoints(q)) {
        next = q->next;
        memcpy(q, p, sizeof(pelem));   // keep old successor
        q->next = next;
    }
}

/**
 Partial copy of a point list.

 @param[in]		pl		Point list with \a n elements.
 @param[out]	ql		Overwritten point list with \f$ \min(n, num) \f$ copied elements and old successors.
*/
void CopyPoint (plist *pl, plist *ql)
{
    pelem *p, *q;

    if (CheckPoints(pl) && CheckPoints(ql)) {
        if (pl->num > ql->num) {
            Warning("CopyPoint: skipped %d elements!", pl->num - ql->num);
        }
        if (pl->compact && ql->compact) {
            for (p=pl->start, q=ql->start; (p<=pl->end) && (q<=ql->end); ++p, ++q) {
                memcpy(q, p, sizeof(pelem));
                q->next = q+1;
            }
            ql->end->next = NULL;
        } else if (pl->compact) {
            for (p=pl->start, q=ql->start; (p<=pl->end) && q; ++p, q=q->next) {
                CopyPoint(p, q);
            }
        } else if (ql->compact) {
            for (p=pl->start, q=ql->start; p && (q<=ql->end); p=p->next, ++q) {
                memcpy(q, p, sizeof(pelem));
                q->next = q+1;
            }
            ql->end->next = NULL;
        } else {
            for (p=pl->start, q=ql->start; p && q; p=p->next, q=q->next) {
                CopyPoint(p, q);
            }
        }
    }
}

/**
 Partial copy of a point list.

 @param[in]		pl		Point list with \a n elements.
 @param[in]		s		Start index.
 @param[in]		e		End index.
 @param[out]	ql		Overwritten point list with \f$ e-s+1 \f$ copied elements and old successors.
*/
void CopyPoint (plist *pl, int s, int e, plist *ql)
{
    pelem *p, *q;
    long i, num;

    if (CheckIndex(pl,s) && CheckIndex(pl,e) && CheckPoints(ql)) {
        if (s > e) {
            SWAP(int, s, e);
        }
        num = e-s+1;
        if (num > ql->num) {
            Warning("CopyPoint: skipped %d elements!", num - ql->num);
        }
        p = GetPointPos(pl, s);
        if (p) {
            if (pl->compact && ql->compact) {
                for (q=ql->start, i=0; (p<=pl->end) && (q<=ql->end) && (i < num); ++i, ++p, ++q) {
                    memcpy(q, p, sizeof(pelem));
                    q->next = q+1;
                }
                ql->end->next = NULL;
            } else if (pl->compact) {
                for (q=ql->start, i=0; (p<=pl->end) && q && (i < num); ++i, ++p, q=q->next) {
                    CopyPoint(p, q);
                }
            } else if (ql->compact) {
                for (q=ql->start, i=0; p && (q<=ql->end) && (i < num); ++i, p=p->next, ++q) {
                    memcpy(q, p, sizeof(pelem));
                    q->next = q+1;
                }
                ql->end->next = NULL;
            } else {
                for (q=ql->start, i=0; p && q && (i < num); ++i, p=p->next, q=q->next) {
                    CopyPoint(p, q);
                }
            }
        }
    }
}

/**
 Partial copy of a point list.

 @param[in]		pl		Point list with \a n elements.
 @param[out]	ql		Overwritten point list with \f$ \min(n, num) \f$ copied elements and old successors.
 @param[in]		d		Destination start index.
*/
void CopyPoint (plist *pl, plist *ql, int d)
{
    pelem *p, *q;
	long i;

    if (CheckPoints(pl) && CheckIndex(ql, d)) {
        if (pl->num > ql->num - d) {
            Warning("CopyPoint: skipped %d elements!", pl->num - ql->num + d);
        }
        if (pl->compact && ql->compact) {
            for (p=pl->start, q=ql->start+d; (p<=pl->end) && (q<=ql->end); ++p, ++q) {
                memcpy(q, p, sizeof(pelem));
                q->next = q+1;
            }
            ql->end->next = NULL;
        } else if (pl->compact) {
			for (q=ql->start, i=0; (i < d) && q; ++i, q=q->next); // Skip d elements
            for (p=pl->start; (p<=pl->end) && q; ++p, q=q->next) {
                CopyPoint(p, q);
            }
        } else if (ql->compact) {
            for (p=pl->start, q=ql->start+d; p && (q<=ql->end); p=p->next, ++q) {
                memcpy(q, p, sizeof(pelem));
                q->next = q+1;
            }
            ql->end->next = NULL;
        } else {
			for (q=ql->start, i=0; (i < d) && q; ++i, q=q->next); // Skip d elements
            for (p=pl->start; p && q; p=p->next, q=q->next) {
                CopyPoint(p, q);
            }
        }
    }
}

/**
 Partial copy of a point list.

 @param[in]		pl		Point list with \a n elements.
 @param[in]		s		Start index.
 @param[in]		e		End index.
 @param[out]	ql		Overwritten point list with \f$ e-s+1 \f$ copied elements and old successors.
 @param[in]		d		Destination start index.
*/
void CopyPoint (plist *pl, int s, int e, plist *ql, int d)
{
    pelem *p, *q;
    long i, num;

    if (CheckIndex(pl,s) && CheckIndex(pl,e) && CheckIndex(ql, d)) {
        if (s > e) {
            SWAP(int, s, e);
        }
        num = e-s+1;
        if (num > ql->num - d) {
            Warning("CopyPoint: skipped %d elements!", num - ql->num + d);
        }
        p = GetPointPos(pl, s);
        if (p) {
            if (pl->compact && ql->compact) {
                for (q=ql->start+d, i=0; (p<=pl->end) && (q<=ql->end) && (i < num); ++i, ++p, ++q) {
                    memcpy(q, p, sizeof(pelem));
                    q->next = q+1;
                }
                ql->end->next = NULL;
            } else if (pl->compact) {
				for (q=ql->start, i=0; (i < d) && q; ++i, q=q->next); // Skip d elements
                for (i=0; (p<=pl->end) && q && (i < num); ++i, ++p, q=q->next) {
                    CopyPoint(p, q);
                }
            } else if (ql->compact) {
                for (q=ql->start+d, i=0; p && (q<=ql->end) && (i < num); ++i, p=p->next, ++q) {
                    memcpy(q, p, sizeof(pelem));
                    q->next = q+1;
                }
                ql->end->next = NULL;
            } else {
				for (q=ql->start, i=0; (i < d) && q; ++i, q=q->next); // Skip d elements
                for (i=0; p && q && (i < num); ++i, p=p->next, q=q->next) {
                    CopyPoint(p, q);
                }
            }
        }
    }
}

/**
 Delete a specified point element from the list.

 @param[in,out]		pl		Modified point list.
 @param[in]			p		Point element.
*/
void DeletePoint (plist *pl, pelem *p)
{
    pelem *prev=NULL, *act;

    if (CheckPoints(pl) && CheckPoints(p)) {
        PointList(pl);     // Convert compact array to linked list
        for (act = pl->start; act; ) {
            if (act == p) {
                if (!act->next) {            // Last element
                    pl->end = prev;
                }
                if (prev){
                    prev->next = act->next;
                    hfree(act); act = NULL;
                } else {
                    pl->start = act->next;
                    hfree(act); act = NULL;
                }
                pl->num -= 1;
                return;
            } else {
                prev = act;
                act = act->next;
            }
        }
//      Warning("DeletePoint: Point element not found in list!");
    }
}

/**
 Delete only the first point with the specified id.

 @param[in,out]		pl		Modified point list.
 @param[in]			id		Identification number.
*/
void DeletePointID (plist *pl, long id)
{
    pelem *prev=NULL, *act;

    if (CheckPoints(pl)) {
        PointList(pl);     // Convert compact array to linked list
        for (act = pl->start; act; ) {
            if (act->id == id) {
                if (!act->next) {            // Last element
                    pl->end = prev;
                }
                if (prev){
                    prev->next = act->next;
                    hfree(act);
                    act = prev->next;
                } else {
                    pl->start = act->next;
                    hfree(act);
                    act = pl->start;
                }
                pl->num -= 1;
                return;               // Delete only the first ID
            } else {
                prev = act;
                act = act->next;
            }
        }
//      Warning("DeletePointID: Point element with ID %d not found in list!", id);
    }
}

/**
 Delete all points with the specified id.

 @param[in,out]		pl		Modified point list.
 @param[in]			id		Identification number.
*/
void DeletePointIDs (plist *pl, long id)
{
    pelem *prev=NULL, *act;

    if (CheckPoints(pl)) {
        PointList(pl);     // Convert compact array to linked list
        for (act = pl->start; act; ) {
            if (act->id == id) {
                if (!act->next) {            // Last element
                    pl->end = prev;
                }
                if (prev){
                    prev->next = act->next;
                    hfree(act);
                    act = prev->next;
                } else {
                    pl->start = act->next;
                    hfree(act);
                    act = pl->start;
                }
                pl->num -= 1;
            } else {
                prev = act;
                act = act->next;
            }
        }
    }
}

/**
 Delete point at the specified position.

 @param[in,out]		pl		Modified point list.
 @param[in]			pos		Postion in the list.
*/
void DeletePointPos (plist *pl, long pos)
{
    pelem *prev=NULL, *act;
    long i;

    if (CheckPoints(pl)) {
        PointList(pl);     // Convert compact array to linked list
        if ((pos >= 0) && (pos < pl->num)) {
            for (i=0, act = pl->start; act; ++i) {
                if (i == pos) {
                    if (!act->next) {            // Last element
                        pl->end = prev;
                    }
                    if (prev){
                        prev->next = act->next;
                        hfree(act); act = NULL;
                    } else {
                        pl->start = act->next;
                        hfree(act); act = NULL;
                    }
                    pl->num -= 1;
                    return;
                } else {
                    prev = act;
                    act = act->next;
                }
            }
        }
//      Warning("DeletePointPos: Point not found at position %d in list!", pos);
    }
}

/**
 Free memory of a point list.

 @param[in]			list	Point list.
 @result					NULL.
*/
plist *FreePoint (plist *list)
{
    pelem *next, *p;

    if (list) {
        if (list->compact) {
			if (list->start) {
				hfree (list->start); list->start = NULL;
			}
        } else {
            for (p = list->start; p; p = next) {
                next = p->next;
                hfree (p); p = NULL;
            }
        }
        hfree (list); list = NULL;
    }
    return NULL;
}

/**
 Get point with the specified identification number.

 Binary search for sorted ID numbers in compact array

   BinarySearch(A[0..N-1], value) {
       low = 0
       high = N - 1
       while (low <= high) {
           mid = (low + high) / 2
           if (A[mid] > value)
               high = mid - 1
           else if (A[mid] < value)
               low = mid + 1
           else
               return mid // found
       }
       return -1 // not found
   }

 @param[in]			pl		Point list.
 @param[in]			id		Identification number.
 @result					First point element with the specified ID or NULL.
*/
pelem *GetPointID (plist *pl, long id)
{
    long low = 0, mid, high;
    pelem *p;

    if (CheckPoints(pl)) {
        if (pl->compact) {
            high = pl->num - 1;
            while (low <= high) {
                mid = (low + high)/2;
                p = pl->start + mid;
                if (p->id > id) {
                    high = mid-1;
                } else if (p->id < id) {
                    low = mid+1;
                } else {
                    return p;
                }
            }
        } else {
            for (p=pl->start; p; p=p->next) {
                if (p->id == id) {
                    return p;
                }
            }
        }
    }
//  Warning ("GetPointID: illegal ID %d !", id);

    return NULL;
}

/**
 Get point at the specified position.

 @param[in]			pl		Point list.
 @param[in]			pos		Postion in the list.
 @result					Point element at the specified position or NULL.
*/
pelem *GetPointPos (plist *pl, long pos)
{
    pelem *p;
    long i;

    if (CheckIndex(pl, pos)) {
        if (pl->compact) {
            return pl->start + pos;
        } else {
            for (p=pl->start, i=0; p && (i < pos); ++i, p=p->next);
            return p;
        }
    }
    return NULL;
}

/**
 Allocate memory for a new point element.

 @result					New allocated point element or NULL.
*/
pelem *NewPoint (void)
{
    return (pelem *) halloc (1, sizeof (pelem));
}

/**
 Allocate memory for n new point elements.

 @param[in]			num		Number of point elements.
 @result					New allocated point element or NULL.
*/
pelem *NewPoint (int num)
{
    return (pelem *) halloc (num, sizeof (pelem));
}

/**
 Allocate memory for a new point list.

 @result					New allocated empty point list or NULL.
*/
plist *NewPointList (void)
{
    plist *list = NULL;

    list = (plist *) halloc (1, sizeof (plist));
    if (list) {
        list->start = list->end = NULL;
        list->num = 0;
        list->compact = false;
    }
    return list;
}

/**
 Allocate memory for a new compact point array.

 @param[in]			num		Number of point elements.
 @result					New allocated point array with num elements or NULL.
 @note						Point elements are not linked!
*/
plist *NewPointList (int num)
{
    plist *list = NULL;
    pelem *start, *end;

    list = (plist *) halloc (1, sizeof (plist));
    if (list) {
        start = NewPoint (num);
        if (start) {
            end = start + num-1;
            list->start   = start;
            list->num     = num;
            list->end     = end;
            list->compact = true;
        } else {
            hfree(list); list = NULL;
        }
    }
    return list;
}

/**
 Swap the contents of two different point lists.

 @param[in,out]		pl1		Modified first point list with \a m elements.
 @param[in,out]		pl2		Modified second point list with \a n elements.
*/
void SwapPoint(plist *pl1, plist *pl2)
{
    if (CheckPoints(pl1) && CheckPoints(pl2)) {
        SWAP(long, pl1->num, pl2->num);
        SWAP(bool, pl1->compact, pl2->compact);
        SWAP(pelem *, pl1->start, pl2->start);
        SWAP(pelem *, pl1->end, pl2->end);
    }
}

//@}


/** @name Arithmetic Operations with Scalars */
//@{

/**
 Add scalar to Euclidean point.

 @param[in]			h	    Euclidean point.
 @param[in]			s	    Scalar
 @result					Added point or NOHOMO2.
*/
homo2 PointAdd (homo2 *h, double s)
{
    homo2 r;

    if (!h) {
        Warning ("PointAdd failed!"); return NOHOMO2;
    }
    NormHomo(h);
    if (ISZERO(s)) {
        r.x = h->x;
        r.y = h->y;
    } else {
        r.x = h->x + s;
        r.y = h->y + s;
    }
    r.w = 1.0;
    r.id = h->id;

    return r;
}

/**
 Add scalar to Euclidean point.

 @param[in]			p	    Euclidean point.
 @param[in]			s	    Scalar.
 @result					Added point or NOHOMO2.
*/
homo2 PointAdd (pelem *p, double s)
{
    homo2 r;

    if (!p) {
        Warning ("PointAdd failed!"); return NOHOMO2;
    }
    if (ISZERO(s)) {
        r.x = p->x;
        r.y = p->y;
    } else {
        r.x = p->x + s;
        r.y = p->y + s;
    }
    r.w = 1.0;
    r.id = p->id;

    return r;
}

/**
 Add scalar to Euclidean point list.

 @param[in,out]		pl		Modified Euclidean point list.
 @param[in]			s		Scalar.
*/
void PointAdd (plist *pl, double s)
{
    pelem *p;

    if (CheckPoints(pl) && ISNONZERO(s)) {
        if (pl->compact) {
            for (p=pl->start; p<=pl->end; ++p) {
                p->x += s;
                p->y += s;
            }
        } else {
            for (p=pl->start; p ; p=p->next) {
                p->x += s;
                p->y += s;
            }
        }
    }
}

/**
 Add scalar to Euclidean point list.

 @param[in]		pl1		Euclidean point list.
 @param[in]		s		Scalar.
 @param[out]	pl2		Overwritten Euclidean point list with added scalar.
*/
void PointAdd (plist *pl1, double s, plist *pl2)
{
    pelem *p, *q;

    if (CheckSamePoints(pl1,pl2)) {
        if (ISZERO(s)) {
            CopyPoint(pl1, pl2);
        } else {
            if (pl1->compact && pl2->compact) {
                for (p=pl1->start, q=pl2->start; (p<=pl1->end) && (q<=pl2->end); ++p, ++q) {
                    q->x = p->x + s;
                    q->y = p->y + s;
                    q->id = p->id;
                }
            } else if (pl1->compact) {
                for (p=pl1->start, q=pl2->start; (p<=pl1->end) && q; ++p, q=q->next) {
                    q->x = p->x + s;
                    q->y = p->y + s;
                    q->id = p->id;
                }
            } else if (pl2->compact) {
                for (p=pl1->start, q=pl2->start; p && (q<=pl2->end); p=p->next, ++q) {
                    q->x = p->x + s;
                    q->y = p->y + s;
                    q->id = p->id;
                }
            } else {
                for (p=pl1->start, q=pl2->start; p && q; p=p->next, q=q->next) {
                    q->x = p->x + s;
                    q->y = p->y + s;
                    q->id = p->id;
                }
            }
        }
    }
}

/**
 Add scalar to Euclidean point list.

 @param[in]		pl		Euclidean point list.
 @param[in]		s		Scalar.
 @result	    		New allocated Euclidean point array with added scalar.
*/
plist *PointAddNew (plist *pl, double s)
{
    plist *list = NULL;

    if (CheckPoints(pl)) {
        list = NewPointList(pl->num);
        PointAdd(pl, s, list);
        RelinkPoint(list);
    }
    return list;
}

/**
 Divide Euclidean point by a scalar.

 @param[in]			h	    Euclidean point.
 @param[in]			s	    Scalar
 @result					Divided point or NOHOMO2.
*/
homo2 PointDiv (homo2 *h, double s)
{
    homo2 r;

    if (!h) {
        Warning ("PointDiv failed!"); return NOHOMO2;
    }
    if (ISZERO(s)) {
        Warning ("PointDiv: division by zero!");
    }
    NormHomo(h);
    r.x = h->x;
    r.y = h->y;
    r.w = s;
    r.id = h->id;

    return r;
}

/**
 Divide Euclidean point by a scalar.

 @param[in]			p	    Euclidean point.
 @param[in]			s	    Scalar.
 @result					Divided point or NOHOMO2.
*/
homo2 PointDiv (pelem *p, double s)
{
    homo2 r;

    if (!p) {
        Warning ("PointDiv failed!"); return NOHOMO2;
    }
    if (ISZERO(s)) {
        Warning ("PointDiv: division by zero!");
    }
    r.x = p->x;
    r.y = p->y;
    r.w = s;
    r.id = p->id;

    return r;
}

/**
 Divide Euclidean point list by a scalar.

 @param[in,out]		pl		Modified Euclidean point list.
 @param[in]			s		Scalar.
*/
void PointDiv (plist *pl, double s)
{
    pelem *p;

    if (ISZERO(s)) {
        Warning ("PointDiv: division by zero!"); return;
    }
    if (CheckPoints(pl) && ISINEQUAL(s, 1.0)) {
        if (pl->compact) {
            for (p=pl->start; p<=pl->end; ++p) {
                p->x /= s;
                p->y /= s;
            }
        } else {
            for (p=pl->start; p ; p=p->next) {
                p->x /= s;
                p->y /= s;
            }
        }
    }
}

/**
 Divide Euclidean point list by a scalar.

 @param[in]		pl1		Euclidean point list.
 @param[in]		s		Scalar.
 @param[out]	pl2		Overwritten Euclidean point list divided by a scalar.
*/
void PointDiv (plist *pl1, double s, plist *pl2)
{
    pelem *p, *q;

    if (ISZERO(s)) {
        Warning ("PointDiv: division by zero!"); return;
    } else if (ISEQUAL(s, 1.0)) {
        CopyPoint(pl1, pl2);
    } else if (CheckSamePoints(pl1,pl2)) {
        if (pl1->compact && pl2->compact) {
            for (p=pl1->start, q=pl2->start; (p<=pl1->end) && (q<=pl2->end); ++p, ++q) {
                q->x = p->x / s;
                q->y = p->y / s;
                q->id = p->id;
            }
        } else if (pl1->compact) {
            for (p=pl1->start, q=pl2->start; (p<=pl1->end) && q; ++p, q=q->next) {
                q->x = p->x / s;
                q->y = p->y / s;
                q->id = p->id;
            }
        } else if (pl2->compact) {
            for (p=pl1->start, q=pl2->start; p && (q<=pl2->end); p=p->next, ++q) {
                q->x = p->x / s;
                q->y = p->y / s;
                q->id = p->id;
            }
        } else {
            for (p=pl1->start, q=pl2->start; p && q; p=p->next, q=q->next) {
                q->x = p->x / s;
                q->y = p->y / s;
                q->id = p->id;
            }
        }
    }
}

/**
 Divide Euclidean point list by a scalar.

 @param[in]		pl		Euclidean point list.
 @param[in]		s		Scalar.
 @result	    		New allocated Euclidean point array divided by a scalar.
*/
plist *PointDivNew (plist *pl, double s)
{
    plist *list = NULL;

    if (CheckPoints(pl)) {
        list = NewPointList(pl->num);
        PointDiv(pl, s, list);
        RelinkPoint(list);
    }
    return list;
}

/**
 Multiply scalar to Euclidean point.

 @param[in]			h	    Euclidean point.
 @param[in]			s	    Scalar
 @result					Multiplied point or NOHOMO2.
*/
homo2 PointMult (homo2 *h, double s)
{
    homo2 r;

    if (!h) {
        Warning ("PointMult failed!"); return NOHOMO2;
    }
    NormHomo(h);
    if (ISZERO(s)) {
        r.x = r.y = 0.0;
    } else if (ISEQUAL(s, 1.0)) {
        r.x = h->x;
        r.y = h->y;
    } else {
        r.x = h->x * s;
        r.y = h->y * s;
    }
    r.w = 1.0;
    r.id = h->id;

    return r;
}

/**
 Multiply scalar to Euclidean point.

 @param[in]			p	    Euclidean point.
 @param[in]			s	    Scalar.
 @result					Multiplied point or NOHOMO2.
*/
homo2 PointMult (pelem *p, double s)
{
    homo2 r;

    if (!p) {
        Warning ("PointMult failed!"); return NOHOMO2;
    }
    if (ISZERO(s)) {
        r.x = r.y = 0.0;
    } else if (ISEQUAL(s, 1.0)) {
        r.x = p->x;
        r.y = p->y;
    } else {
        r.x = p->x * s;
        r.y = p->y * s;
    }
    r.w = 1.0;
    r.id = p->id;

    return r;
}

/**
 Multiply scalar to Euclidean point list.

 @param[in,out]		pl		Modified Euclidean point list.
 @param[in]			s		Scalar.
*/
void PointMult (plist *pl, double s)
{
    pelem *p;

    if (CheckPoints(pl) && ISINEQUAL(s, 1.0)) {
        if (ISZERO(s)) {
            if (pl->compact) {
                for (p=pl->start; p<=pl->end; ++p) {
                    p->x = p->y = 0.0;
                }
            } else {
                for (p=pl->start; p ; p=p->next) {
                    p->x = p->y = 0.0;
                }
            }
        } else {
            if (pl->compact) {
                for (p=pl->start; p<=pl->end; ++p) {
                    p->x *= s;
                    p->y *= s;
                }
            } else {
                for (p=pl->start; p ; p=p->next) {
                    p->x *= s;
                    p->y *= s;
                }
            }
        }
    }
}

/**
 Multiply scalar to Euclidean point list.

 @param[in]		pl1		Euclidean point list.
 @param[in]		s		Scalar.
 @param[out]	pl2		Overwritten Euclidean point list with multiplied scalar.
*/
void PointMult (plist *pl1, double s, plist *pl2)
{
    pelem *p, *q;

    if (ISEQUAL(s, 1.0)) {
        CopyPoint(pl1, pl2);
    } else if (CheckSamePoints(pl1,pl2)) {
        if (ISZERO(s)) {
            if (pl1->compact && pl2->compact) {
                for (p=pl1->start, q=pl2->start; (p<=pl1->end) && (q<=pl2->end); ++p, ++q) {
                    q->x = q->y = 0.0; q->id = p->id;
                }
            } else if (pl1->compact) {
                for (p=pl1->start, q=pl2->start; (p<=pl1->end) && q; ++p, q=q->next) {
                    q->x = q->y = 0.0; q->id = p->id;
                }
            } else if (pl2->compact) {
                for (p=pl1->start, q=pl2->start; p && (q<=pl2->end); p=p->next, ++q) {
                    q->x = q->y = 0.0; q->id = p->id;
                }
            } else {
                for (p=pl1->start, q=pl2->start; p && q; p=p->next, q=q->next) {
                    q->x = q->y = 0.0; q->id = p->id;
                }
            }
        } else {
            if (pl1->compact && pl2->compact) {
                for (p=pl1->start, q=pl2->start; (p<=pl1->end) && (q<=pl2->end); ++p, ++q) {
                    q->x = p->x * s;
                    q->y = p->y * s;
                    q->id = p->id;
                }
            } else if (pl1->compact) {
                for (p=pl1->start, q=pl2->start; (p<=pl1->end) && q; ++p, q=q->next) {
                    q->x = p->x * s;
                    q->y = p->y * s;
                    q->id = p->id;
                }
            } else if (pl2->compact) {
                for (p=pl1->start, q=pl2->start; p && (q<=pl2->end); p=p->next, ++q) {
                    q->x = p->x * s;
                    q->y = p->y * s;
                    q->id = p->id;
                }
            } else {
                for (p=pl1->start, q=pl2->start; p && q; p=p->next, q=q->next) {
                    q->x = p->x * s;
                    q->y = p->y * s;
                    q->id = p->id;
                }
            }
        }
    }
}

/**
 Multiply scalar to Euclidean point list.

 @param[in]		pl		Euclidean point list.
 @param[in]		s		Scalar.
 @result	    		New allocated Euclidean point array with multiplied scalar.
*/
plist *PointMultNew (plist *pl, double s)
{
    plist *list = NULL;

    if (CheckPoints(pl)) {
        list = NewPointList(pl->num);
        PointMult(pl, s, list);
        RelinkPoint(list);
    }
    return list;
}

/**
 Subtract scalar from Euclidean point.

 @param[in]			h	    Euclidean point.
 @param[in]			s	    Scalar
 @result					Subtracted point or NOHOMO2.
*/
homo2 PointSub (homo2 *h, double s)
{
    homo2 r;

    if (!h) {
        Warning ("PointSub failed!"); return NOHOMO2;
    }
    NormHomo(h);
    if (ISZERO(s)) {
        r.x = h->x;
        r.y = h->y;
    } else {
        r.x = h->x - s;
        r.y = h->y - s;
    }
    r.w = 1.0;
    r.id = h->id;

    return r;
}

/**
 Subtract scalar from Euclidean point.

 @param[in]			p	    Euclidean point.
 @param[in]			s	    Scalar.
 @result					Subtracted point or NOHOMO2.
*/
homo2 PointSub (pelem *p, double s)
{
    homo2 r;

    if (!p) {
        Warning ("PointSub failed!"); return NOHOMO2;
    }
    if (ISZERO(s)) {
        r.x = p->x;
        r.y = p->y;
    } else {
        r.x = p->x - s;
        r.y = p->y - s;
    }
    r.w = 1.0;
    r.id = p->id;

    return r;
}

/**
 Subtract scalar from Euclidean point list.

 @param[in,out]		pl		Modified Euclidean point list.
 @param[in]			s		Scalar.
*/
void PointSub (plist *pl, double s)
{
    pelem *p;

    if (CheckPoints(pl) && ISNONZERO(s)) {
        if (pl->compact) {
            for (p=pl->start; p<=pl->end; ++p) {
                p->x -= s;
                p->y -= s;
            }
        } else {
            for (p=pl->start; p ; p=p->next) {
                p->x -= s;
                p->y -= s;
            }
        }
    }
}

/**
 Subtract scalar from Euclidean point list.

 @param[in]		pl1		Euclidean point list.
 @param[in]		s		Scalar.
 @param[out]	pl2		Overwritten Euclidean point list with subtracted scalar.
*/
void PointSub (plist *pl1, double s, plist *pl2)
{
    pelem *p, *q;

    if (CheckSamePoints(pl1,pl2)) {
        if (ISZERO(s)) {
            CopyPoint(pl1, pl2);
        } else {
            if (pl1->compact && pl2->compact) {
                for (p=pl1->start, q=pl2->start; (p<=pl1->end) && (q<=pl2->end); ++p, ++q) {
                    q->x = p->x - s;
                    q->y = p->y - s;
                    q->id = p->id;
                }
            } else if (pl1->compact) {
                for (p=pl1->start, q=pl2->start; (p<=pl1->end) && q; ++p, q=q->next) {
                    q->x = p->x - s;
                    q->y = p->y - s;
                    q->id = p->id;
                }
            } else if (pl2->compact) {
                for (p=pl1->start, q=pl2->start; p && (q<=pl2->end); p=p->next, ++q) {
                    q->x = p->x - s;
                    q->y = p->y - s;
                    q->id = p->id;
                }
            } else {
                for (p=pl1->start, q=pl2->start; p && q; p=p->next, q=q->next) {
                    q->x = p->x - s;
                    q->y = p->y - s;
                    q->id = p->id;
                }
            }
        }
    }
}

/**
 Substract scalar from Euclidean point list.

 @param[in]		pl		Euclidean point list.
 @param[in]		s		Scalar.
 @result	    		New allocated Euclidean point array with substracted scalar.
*/
plist *PointSubNew (plist *pl, double s)
{
    plist *list = NULL;

    if (CheckPoints(pl)) {
        list = NewPointList(pl->num);
        PointSub(pl, s, list);
        RelinkPoint(list);
    }
    return list;
}

//@}

/** @name Arithmetic Element-by-element Operations */
//@{

/**
 Add corresponding elements of two points in Euclidean coordinates.

 @param[in]			h1	    First Euclidean point.
 @param[in]			h2	    Second Euclidean point.
 @result					Added point or NOHOMO2.
 @note						Point ID is zero.
*/
homo2 PointAdd (homo2 *h1, homo2 *h2)
{
    homo2 r;

    if (!h1 || !h2) {
        Warning ("PointAdd failed!"); return NOHOMO2;
    }
    NormHomo(h1);
    NormHomo(h2);
    r.x = h1->x + h2->x;
    r.y = h1->y + h2->y;
    r.w = 1.0;
    r.id = 0;

    return r;
}

/**
 Add corresponding elements of two points in Euclidean coordinates.

 @param[in]			p1	    First Euclidean point.
 @param[in]			h2	    Second Euclidean point.
 @result					Added point or NOHOMO2.
 @note						Point ID is zero.
*/
homo2 PointAdd (pelem *p1, homo2 *h2)
{
    homo2 r;

    if (!p1 || !h2) {
        Warning ("PointAdd failed!"); return NOHOMO2;
    }
    NormHomo(h2);
    r.x = p1->x + h2->x;
    r.y = p1->y + h2->y;
    r.w = 1.0;
    r.id = 0;

    return r;
}

/**
 Add corresponding elements of two points in Euclidean coordinates.

 @param[in]			h1	    First Euclidean point.
 @param[in]			p2	    Second Euclidean point.
 @result					Added point or NOHOMO2.
 @note						Point ID is zero.
*/
homo2 PointAdd (homo2 *h1, pelem *p2)
{
    homo2 r;

    if (!h1 || !p2) {
        Warning ("PointAdd failed!"); return NOHOMO2;
    }
    NormHomo(h1);
    r.x = h1->x + p2->x;
    r.y = h1->y + p2->y;
    r.w = 1.0;
    r.id = 0;

    return r;
}

/**
 Add corresponding elements of two points in Euclidean coordinates.

 @param[in]			p1	    First Euclidean point.
 @param[in]			p2	    Second Euclidean point.
 @result					Added point or NOHOMO2.
 @note						Point ID is zero.
*/
homo2 PointAdd (pelem *p1, pelem *p2)
{
    homo2 r;

    if (!p1 || !p2) {
        Warning ("PointAdd failed!"); return NOHOMO2;
    }
    r.x = p1->x + p2->x;
    r.y = p1->y + p2->y;
    r.w = 1.0;
    r.id = 0;

    return r;
}

/**
 Add corresponding elements with identical IDs of two Euclidean point lists.

 @param[in,out]		pl1		Modified first Euclidean point list.
 @param[in]			pl2		Second Euclidean point list.
*/
void PointAdd (plist *pl1, plist *pl2)
{
    pelem *p, *q;

    if (CheckSamePoints(pl1,pl2)) {
        if (pl1->compact && pl2->compact) {
            for (p=pl1->start, q=pl2->start; (p<=pl1->end) && (q<=pl2->end); ++p, ++q) {
                if (CheckID(p, q)) {
                    p->x += q->x;
                    p->y += q->y;
                }
            }
        } else if (pl1->compact) {
            for (p=pl1->start, q=pl2->start; (p<=pl1->end) && q; ++p, q=q->next) {
                if (CheckID(p, q)) {
                    p->x += q->x;
                    p->y += q->y;
                }
            }
        } else if (pl2->compact) {
            for (p=pl1->start, q=pl2->start; p && (q<=pl2->end); p=p->next, ++q) {
                if (CheckID(p, q)) {
                    p->x += q->x;
                    p->y += q->y;
                }
            }
        } else {
            for (p=pl1->start, q=pl2->start; p && q; p=p->next, q=q->next) {
                if (CheckID(p, q)) {
                    p->x += q->x;
                    p->y += q->y;
                }
            }
        }
    }
}

/**
 Add corresponding elements with identical IDs of two Euclidean point lists.

 @param[in]		pl1		First Euclidean point list.
 @param[in]		pl2		Second Euclidean point list.
 @param[out]	pl3		Overwritten Euclidean point list with added elements.
*/
void PointAdd (plist *pl1, plist *pl2, plist *pl3)
{
    pelem *p, *q, *r;

    if (CheckSamePoints(pl1,pl2) && CheckSamePoints(pl1,pl3)) {
        if (pl1->compact && pl2->compact) {
            if (pl3->compact) {
                for (p=pl1->start, q=pl2->start, r=pl3->start; (p<=pl1->end) && (q<=pl2->end) && (r<=pl3->end); ++p, ++q, ++r) {
                    if (CheckID(p, q)) {
                        r->x = p->x + q->x;
                        r->y = p->y + q->y;
                        r->id = p->id;
                    }
                }
            } else {
                for (p=pl1->start, q=pl2->start, r=pl3->start; (p<=pl1->end) && (q<=pl2->end) && r; ++p, ++q, r=r->next) {
                    if (CheckID(p, q)) {
                        r->x = p->x + q->x;
                        r->y = p->y + q->y;
                        r->id = p->id;
                    }
                }
            }
        } else if (pl1->compact) {
            if (pl3->compact) {
                for (p=pl1->start, q=pl2->start, r=pl3->start; (p<=pl1->end) && q && (r<=pl3->end); ++p, q=q->next, ++r) {
                    if (CheckID(p, q)) {
                        r->x = p->x + q->x;
                        r->y = p->y + q->y;
                        r->id = p->id;
                    }
                }
            } else {
                for (p=pl1->start, q=pl2->start, r=pl3->start; (p<=pl1->end) && q && r; ++p, q=q->next, r=r->next) {
                    if (CheckID(p, q)) {
                        r->x = p->x + q->x;
                        r->y = p->y + q->y;
                        r->id = p->id;
                    }
                }
            }
        } else if (pl2->compact) {
            if (pl3->compact) {
                for (p=pl1->start, q=pl2->start, r=pl3->start; p && (q<=pl2->end) && (r<=pl3->end); p=p->next, ++q, ++r) {
                    if (CheckID(p, q)) {
                        r->x = p->x + q->x;
                        r->y = p->y + q->y;
                        r->id = p->id;
                    }
                }
            } else {
                for (p=pl1->start, q=pl2->start, r=pl3->start; p && (q<=pl2->end) && r; p=p->next, ++q, r=r->next) {
                    if (CheckID(p, q)) {
                        r->x = p->x + q->x;
                        r->y = p->y + q->y;
                        r->id = p->id;
                    }
                }
            }
        } else {
            if (pl3->compact) {
                for (p=pl1->start, q=pl2->start, r=pl3->start; p && q && (r<=pl3->end); p=p->next, q=q->next, ++r) {
                    if (CheckID(p, q)) {
                        r->x = p->x + q->x;
                        r->y = p->y + q->y;
                        r->id = p->id;
                    }
                }
            } else {
                for (p=pl1->start, q=pl2->start, r=pl3->start; p && q && r; p=p->next, q=q->next, r=r->next) {
                    if (CheckID(p, q)) {
                        r->x = p->x + q->x;
                        r->y = p->y + q->y;
                        r->id = p->id;
                    }
                }
            }
        }
    }
}

/**
 Add corresponding elements with identical IDs of two Euclidean point lists.

 @param[in]		pl1		First Euclidean point list.
 @param[in]		pl2		Second Euclidean point list.
 @result	    		New allocated Euclidean point array with added elements.
*/
plist *PointAddNew (plist *pl1, plist *pl2)
{
    plist *list = NULL;

    if (CheckSamePoints(pl1, pl2)) {
        list = NewPointList(pl1->num);
        PointAdd(pl1, pl2, list);
        RelinkPoint(list);
    }
    return list;
}

/**
 Divide corresponding elements of two points in Euclidean coordinates.

 @param[in]			h1	    First Euclidean point.
 @param[in]			h2	    Second Euclidean point.
 @result					Divided point or NOHOMO2.
 @note						Point ID is zero.
*/
homo2 PointDiv (homo2 *h1, homo2 *h2)
{
    homo2 r;

    if (!h1 || !h2) {
        Warning ("PointDiv failed!"); return NOHOMO2;
    }
    NormHomo(h1);
    NormHomo(h2);
    r.x = DIVIDE(h1->x, h2->x);
    r.y = DIVIDE(h1->y, h2->y);
    r.w = 1.0;
    r.id = 0;

    return r;
}

/**
 Divide corresponding elements of two points in Euclidean coordinates.

 @param[in]			p1	    First Euclidean point.
 @param[in]			h2	    Second Euclidean point.
 @result					Divided point or NOHOMO2.
 @note						Point ID is zero.
*/
homo2 PointDiv (pelem *p1, homo2 *h2)
{
    homo2 r;

    if (!p1 || !h2) {
        Warning ("PointDiv failed!"); return NOHOMO2;
    }
    NormHomo(h2);
    r.x = DIVIDE(p1->x, h2->x);
    r.y = DIVIDE(p1->y, h2->y);
    r.w = 1.0;
    r.id = 0;

    return r;
}

/**
 Divide corresponding elements of two points in Euclidean coordinates.

 @param[in]			h1	    First Euclidean point.
 @param[in]			p2	    Second Euclidean point.
 @result					Divided point or NOHOMO2.
 @note						Point ID is zero.
*/
homo2 PointDiv (homo2 *h1, pelem *p2)
{
    homo2 r;

    if (!h1 || !p2) {
        Warning ("PointDiv failed!"); return NOHOMO2;
    }
    NormHomo(h1);
    r.x = DIVIDE(h1->x, p2->x);
    r.y = DIVIDE(h1->y, p2->y);
    r.w = 1.0;
    r.id = 0;

    return r;
}

/**
 Divide corresponding elements of two points in Euclidean coordinates.

 @param[in]			p1	    First Euclidean point.
 @param[in]			p2	    Second Euclidean point.
 @result					Divided point or NOHOMO2.
 @note						Point ID is zero.
*/
homo2 PointDiv (pelem *p1, pelem *p2)
{
    homo2 r;

    if (!p1 || !p2) {
        Warning ("PointDiv failed!"); return NOHOMO2;
    }
    r.x = DIVIDE(p1->x, p2->x);
    r.y = DIVIDE(p1->y, p2->y);
    r.w = 1.0;
    r.id = 0;

    return r;
}

/**
 Divide corresponding elements with identical IDs of two Euclidean point lists.

 @param[in,out]		pl1		Modified first Euclidean point list.
 @param[in]			pl2		Second Euclidean point list.
*/
void PointDiv (plist *pl1, plist *pl2)
{
    pelem *p, *q;

    if (CheckSamePoints(pl1,pl2)) {
        if (pl1->compact && pl2->compact) {
            for (p=pl1->start, q=pl2->start; (p<=pl1->end) && (q<=pl2->end); ++p, ++q) {
                if (CheckID(p, q)) {
                    p->x = DIVIDE(p->x, q->x);
                    p->y = DIVIDE(p->y, q->y);
                }
            }
        } else if (pl1->compact) {
            for (p=pl1->start, q=pl2->start; (p<=pl1->end) && q; ++p, q=q->next) {
                if (CheckID(p, q)) {
                    p->x = DIVIDE(p->x, q->x);
                    p->y = DIVIDE(p->y, q->y);
                }
            }
        } else if (pl2->compact) {
            for (p=pl1->start, q=pl2->start; p && (q<=pl2->end); p=p->next, ++q) {
                if (CheckID(p, q)) {
                    p->x = DIVIDE(p->x, q->x);
                    p->y = DIVIDE(p->y, q->y);
                }
            }
        } else {
            for (p=pl1->start, q=pl2->start; p && q; p=p->next, q=q->next) {
                if (CheckID(p, q)) {
                    p->x = DIVIDE(p->x, q->x);
                    p->y = DIVIDE(p->y, q->y);
                }
            }
        }
    }
}

/**
 Divide corresponding elements with identical IDs of two Euclidean point lists.

 @param[in]		pl1		First Euclidean point list.
 @param[in]		pl2		Second Euclidean point list.
 @param[out]	pl3		Overwritten Euclidean point list with divided elements.
*/
void PointDiv (plist *pl1, plist *pl2, plist *pl3)
{
    pelem *p, *q, *r;

    if (CheckSamePoints(pl1,pl2) && CheckSamePoints(pl1,pl3)) {
        if (pl1->compact && pl2->compact) {
            if (pl3->compact) {
                for (p=pl1->start, q=pl2->start, r=pl3->start; (p<=pl1->end) && (q<=pl2->end) && (r<=pl3->end); ++p, ++q, ++r) {
                    if (CheckID(p, q)) {
                        r->x = DIVIDE(p->x, q->x);
                        r->y = DIVIDE(p->y, q->y);
                        r->id = p->id;
                    }
                }
            } else {
                for (p=pl1->start, q=pl2->start, r=pl3->start; (p<=pl1->end) && (q<=pl2->end) && r; ++p, ++q, r=r->next) {
                    if (CheckID(p, q)) {
                        r->x = DIVIDE(p->x, q->x);
                        r->y = DIVIDE(p->y, q->y);
                        r->id = p->id;
                    }
                }
            }
        } else if (pl1->compact) {
            if (pl3->compact) {
                for (p=pl1->start, q=pl2->start, r=pl3->start; (p<=pl1->end) && q && (r<=pl3->end); ++p, q=q->next, ++r) {
                    if (CheckID(p, q)) {
                        r->x = DIVIDE(p->x, q->x);
                        r->y = DIVIDE(p->y, q->y);
                        r->id = p->id;
                    }
                }
            } else {
                for (p=pl1->start, q=pl2->start, r=pl3->start; (p<=pl1->end) && q && r; ++p, q=q->next, r=r->next) {
                    if (CheckID(p, q)) {
                        r->x = DIVIDE(p->x, q->x);
                        r->y = DIVIDE(p->y, q->y);
                        r->id = p->id;
                    }
                }
            }
        } else if (pl2->compact) {
            if (pl3->compact) {
                for (p=pl1->start, q=pl2->start, r=pl3->start; p && (q<=pl2->end) && (r<=pl3->end); p=p->next, ++q, ++r) {
                    if (CheckID(p, q)) {
                        r->x = DIVIDE(p->x, q->x);
                        r->y = DIVIDE(p->y, q->y);
                        r->id = p->id;
                    }
                }
            } else {
                for (p=pl1->start, q=pl2->start, r=pl3->start; p && (q<=pl2->end) && r; p=p->next, ++q, r=r->next) {
                    if (CheckID(p, q)) {
                        r->x = DIVIDE(p->x, q->x);
                        r->y = DIVIDE(p->y, q->y);
                        r->id = p->id;
                    }
                }
            }
        } else {
            if (pl3->compact) {
                for (p=pl1->start, q=pl2->start, r=pl3->start; p && q && (r<=pl3->end); p=p->next, q=q->next, ++r) {
                    if (CheckID(p, q)) {
                        r->x = DIVIDE(p->x, q->x);
                        r->y = DIVIDE(p->y, q->y);
                        r->id = p->id;
                    }
                }
            } else {
                for (p=pl1->start, q=pl2->start, r=pl3->start; p && q && r; p=p->next, q=q->next, r=r->next) {
                    if (CheckID(p, q)) {
                        r->x = DIVIDE(p->x, q->x);
                        r->y = DIVIDE(p->y, q->y);
                        r->id = p->id;
                    }
                }
            }
        }
    }
}

/**
 Divide corresponding elements with identical IDs of two Euclidean point lists.

 @param[in]		pl1		First Euclidean point list.
 @param[in]		pl2		Second Euclidean point list.
 @result	    		New allocated Euclidean point array with divided elements.
*/
plist *PointDivNew (plist *pl1, plist *pl2)
{
    plist *list = NULL;

    if (CheckSamePoints(pl1, pl2)) {
        list = NewPointList(pl1->num);
        PointDiv(pl1, pl2, list);
        RelinkPoint(list);
    }
    return list;
}

/**
 Multiply corresponding elements of two points in Euclidean coordinates.

 @param[in]			h1	    First Euclidean point.
 @param[in]			h2	    Second Euclidean point.
 @result					Multiplied point or NOHOMO2.
 @note						Point ID is zero.
*/
homo2 PointMult (homo2 *h1, homo2 *h2)
{
    homo2 r;

    if (!h1 || !h2) {
        Warning ("PointMult failed!"); return NOHOMO2;
    }
    NormHomo(h1);
    NormHomo(h2);
    r.x = h1->x * h2->x;
    r.y = h1->y * h2->y;
    r.w = 1.0;
    r.id = 0;

    return r;
}

/**
 Multiply corresponding elements of two points in Euclidean coordinates.

 @param[in]			p1	    First Euclidean point.
 @param[in]			h2	    Second Euclidean point.
 @result					Multiplied point or NOHOMO2.
 @note						Point ID is zero.
*/
homo2 PointMult (pelem *p1, homo2 *h2)
{
    homo2 r;

    if (!p1 || !h2) {
        Warning ("PointMult failed!"); return NOHOMO2;
    }
    NormHomo(h2);
    r.x = p1->x * h2->x;
    r.y = p1->y * h2->y;
    r.w = 1.0;
    r.id = 0;

    return r;
}

/**
 Multiply corresponding elements of two points in Euclidean coordinates.

 @param[in]			h1	    First Euclidean point.
 @param[in]			p2	    Second Euclidean point.
 @result					Multiplied point or NOHOMO2.
 @note						Point ID is zero.
*/
homo2 PointMult (homo2 *h1, pelem *p2)
{
    homo2 r;

    if (!h1 || !p2) {
        Warning ("PointMult failed!"); return NOHOMO2;
    }
    NormHomo(h1);
    r.x = h1->x * p2->x;
    r.y = h1->y * p2->y;
    r.w = 1.0;
    r.id = 0;

    return r;
}

/**
 Multiply corresponding elements of two points in Euclidean coordinates.

 @param[in]			p1	    First Euclidean point.
 @param[in]			p2	    Second Euclidean point.
 @result					Multiplied point or NOHOMO2.
 @note						Point ID is zero.
*/
homo2 PointMult (pelem *p1, pelem *p2)
{
    homo2 r;

    if (!p1 || !p2) {
        Warning ("PointMult failed!"); return NOHOMO2;
    }
    r.x = p1->x * p2->x;
    r.y = p1->y * p2->y;
    r.w = 1.0;
    r.id = 0;

    return r;
}

/**
 Multiply corresponding elements with identical IDs of two Euclidean point lists.

 @param[in,out]		pl1		Modified first Euclidean point list.
 @param[in]			pl2		Second Euclidean point list.
*/
void PointMult (plist *pl1, plist *pl2)
{
    pelem *p, *q;

    if (CheckSamePoints(pl1,pl2)) {
        if (pl1->compact && pl2->compact) {
            for (p=pl1->start, q=pl2->start; (p<=pl1->end) && (q<=pl2->end); ++p, ++q) {
                if (CheckID(p, q)) {
                    p->x *= q->x;
                    p->y *= q->y;
                }
            }
        } else if (pl1->compact) {
            for (p=pl1->start, q=pl2->start; (p<=pl1->end) && q; ++p, q=q->next) {
                if (CheckID(p, q)) {
                    p->x *= q->x;
                    p->y *= q->y;
                }
            }
        } else if (pl2->compact) {
            for (p=pl1->start, q=pl2->start; p && (q<=pl2->end); p=p->next, ++q) {
                if (CheckID(p, q)) {
                    p->x *= q->x;
                    p->y *= q->y;
                }
            }
        } else {
            for (p=pl1->start, q=pl2->start; p && q; p=p->next, q=q->next) {
                if (CheckID(p, q)) {
                    p->x *= q->x;
                    p->y *= q->y;
                }
            }
        }
    }
}

/**
 Multiply corresponding elements with identical IDs of two Euclidean point lists.

 @param[in]		pl1		First Euclidean point list.
 @param[in]		pl2		Second Euclidean point list.
 @param[out]	pl3		Overwritten Euclidean point list with multiplied elements.
*/
void PointMult (plist *pl1, plist *pl2, plist *pl3)
{
    pelem *p, *q, *r;

    if (CheckSamePoints(pl1,pl2) && CheckSamePoints(pl1,pl3)) {
        if (pl1->compact && pl2->compact) {
            if (pl3->compact) {
                for (p=pl1->start, q=pl2->start, r=pl3->start; (p<=pl1->end) && (q<=pl2->end) && (r<=pl3->end); ++p, ++q, ++r) {
                    if (CheckID(p, q)) {
                        r->x = p->x * q->x;
                        r->y = p->y * q->y;
                        r->id = p->id;
                    }
                }
            } else {
                for (p=pl1->start, q=pl2->start, r=pl3->start; (p<=pl1->end) && (q<=pl2->end) && r; ++p, ++q, r=r->next) {
                    if (CheckID(p, q)) {
                        r->x = p->x * q->x;
                        r->y = p->y * q->y;
                        r->id = p->id;
                    }
                }
            }
        } else if (pl1->compact) {
            if (pl3->compact) {
                for (p=pl1->start, q=pl2->start, r=pl3->start; (p<=pl1->end) && q && (r<=pl3->end); ++p, q=q->next, ++r) {
                    if (CheckID(p, q)) {
                        r->x = p->x * q->x;
                        r->y = p->y * q->y;
                        r->id = p->id;
                    }
                }
            } else {
                for (p=pl1->start, q=pl2->start, r=pl3->start; (p<=pl1->end) && q && r; ++p, q=q->next, r=r->next) {
                    if (CheckID(p, q)) {
                        r->x = p->x * q->x;
                        r->y = p->y * q->y;
                        r->id = p->id;
                    }
                }
            }
        } else if (pl2->compact) {
            if (pl3->compact) {
                for (p=pl1->start, q=pl2->start, r=pl3->start; p && (q<=pl2->end) && (r<=pl3->end); p=p->next, ++q, ++r) {
                    if (CheckID(p, q)) {
                        r->x = p->x * q->x;
                        r->y = p->y * q->y;
                        r->id = p->id;
                    }
                }
            } else {
                for (p=pl1->start, q=pl2->start, r=pl3->start; p && (q<=pl2->end) && r; p=p->next, ++q, r=r->next) {
                    if (CheckID(p, q)) {
                        r->x = p->x * q->x;
                        r->y = p->y * q->y;
                        r->id = p->id;
                    }
                }
            }
        } else {
            if (pl3->compact) {
                for (p=pl1->start, q=pl2->start, r=pl3->start; p && q && (r<=pl3->end); p=p->next, q=q->next, ++r) {
                    if (CheckID(p, q)) {
                        r->x = p->x * q->x;
                        r->y = p->y * q->y;
                        r->id = p->id;
                    }
                }
            } else {
                for (p=pl1->start, q=pl2->start, r=pl3->start; p && q && r; p=p->next, q=q->next, r=r->next) {
                    if (CheckID(p, q)) {
                        r->x = p->x * q->x;
                        r->y = p->y * q->y;
                        r->id = p->id;
                    }
                }
            }
        }
    }
}

/**
 Multiply corresponding elements with identical IDs of two Euclidean point lists.

 @param[in]		pl1		First Euclidean point list.
 @param[in]		pl2		Second Euclidean point list.
 @result	    		New allocated Euclidean point array with multiplied elements.
*/
plist *PointMultNew (plist *pl1, plist *pl2)
{
    plist *list = NULL;

    if (CheckSamePoints(pl1, pl2)) {
        list = NewPointList(pl1->num);
        PointMult(pl1, pl2, list);
        RelinkPoint(list);
    }
    return list;
}

/**
 Substract corresponding elements of two points in Euclidean coordinates.

 @param[in]			h1	    First Euclidean point.
 @param[in]			h2	    Second Euclidean point.
 @result					Point difference or NOHOMO2.
 @note						Point ID is zero.
*/
homo2 PointSub (homo2 *h1, homo2 *h2)
{
    homo2 r;

    if (!h1 || !h2) {
        Warning ("PointSub failed!"); return NOHOMO2;
    }
    NormHomo(h1);
    NormHomo(h2);
    r.x = h1->x - h2->x;
    r.y = h1->y - h2->y;
    r.w = 1.0;
    r.id = 0;

    return r;
}

/**
 Substract corresponding elements of two points in Euclidean coordinates.

 @param[in]			p1	    First Euclidean point.
 @param[in]			h2	    Second Euclidean point.
 @result					Point difference or NOHOMO2.
 @note						Point ID is zero.
*/
homo2 PointSub (pelem *p1, homo2 *h2)
{
    homo2 r;

    if (!p1 || !h2) {
        Warning ("PointSub failed!"); return NOHOMO2;
    }
    NormHomo(h2);
    r.x = p1->x - h2->x;
    r.y = p1->y - h2->y;
    r.w = 1.0;
    r.id = 0;

    return r;
}

/**
 Substract corresponding elements of two points in Euclidean coordinates.

 @param[in]			h1	    First Euclidean point.
 @param[in]			p2	    Second Euclidean point.
 @result					Point difference or NOHOMO2.
 @note						Point ID is zero.
*/
homo2 PointSub (homo2 *h1, pelem *p2)
{
    homo2 r;

    if (!h1 || !p2) {
        Warning ("PointSub failed!"); return NOHOMO2;
    }
    NormHomo(h1);
    r.x = h1->x - p2->x;
    r.y = h1->y - p2->y;
    r.w = 1.0;
    r.id = 0;

    return r;
}

/**
 Substract corresponding elements of two points in Euclidean coordinates.

 @param[in]			p1	    First Euclidean point.
 @param[in]			p2	    Second Euclidean point.
 @result					Point difference or NOHOMO2.
 @note						Point ID is zero.
*/
homo2 PointSub (pelem *p1, pelem *p2)
{
    homo2 r;

    if (!p1 || !p2) {
        Warning ("PointSub failed!"); return NOHOMO2;
    }
    r.x = p1->x - p2->x;
    r.y = p1->y - p2->y;
    r.w = 1.0;
    r.id = 0;

    return r;
}

/**
 Substract corresponding elements with identical IDs of two Euclidean point lists.

 @param[in,out]		pl1		Modified first Euclidean point list.
 @param[in]			pl2		Second Euclidean point list.
*/
void PointSub (plist *pl1, plist *pl2)
{
    pelem *p, *q;

    if (CheckSamePoints(pl1,pl2)) {
        if (pl1->compact && pl2->compact) {
            for (p=pl1->start, q=pl2->start; (p<=pl1->end) && (q<=pl2->end); ++p, ++q) {
                if (CheckID(p, q)) {
                    p->x -= q->x;
                    p->y -= q->y;
                }
            }
        } else if (pl1->compact) {
            for (p=pl1->start, q=pl2->start; (p<=pl1->end) && q; ++p, q=q->next) {
                if (CheckID(p, q)) {
                    p->x -= q->x;
                    p->y -= q->y;
                }
            }
        } else if (pl2->compact) {
            for (p=pl1->start, q=pl2->start; p && (q<=pl2->end); p=p->next, ++q) {
                if (CheckID(p, q)) {
                    p->x -= q->x;
                    p->y -= q->y;
                }
            }
        } else {
            for (p=pl1->start, q=pl2->start; p && q; p=p->next, q=q->next) {
                if (CheckID(p, q)) {
                    p->x -= q->x;
                    p->y -= q->y;
                }
            }
        }
    }
}

/**
 Substract corresponding elements with identical IDs of two Euclidean point lists.

 @param[in]		pl1		First Euclidean point list.
 @param[in]		pl2		Second Euclidean point list.
 @param[out]	pl3		Overwritten Euclidean point list with substracted elements.
*/
void PointSub (plist *pl1, plist *pl2, plist *pl3)
{
    pelem *p, *q, *r;

    if (CheckSamePoints(pl1,pl2) && CheckSamePoints(pl1,pl3)) {
        if (pl1->compact && pl2->compact) {
            if (pl3->compact) {
                for (p=pl1->start, q=pl2->start, r=pl3->start; (p<=pl1->end) && (q<=pl2->end) && (r<=pl3->end); ++p, ++q, ++r) {
                    if (CheckID(p, q)) {
                        r->x = p->x - q->x;
                        r->y = p->y - q->y;
                        r->id = p->id;
                    }
                }
            } else {
                for (p=pl1->start, q=pl2->start, r=pl3->start; (p<=pl1->end) && (q<=pl2->end) && r; ++p, ++q, r=r->next) {
                    if (CheckID(p, q)) {
                        r->x = p->x - q->x;
                        r->y = p->y - q->y;
                        r->id = p->id;
                    }
                }
            }
        } else if (pl1->compact) {
            if (pl3->compact) {
                for (p=pl1->start, q=pl2->start, r=pl3->start; (p<=pl1->end) && q && (r<=pl3->end); ++p, q=q->next, ++r) {
                    if (CheckID(p, q)) {
                        r->x = p->x - q->x;
                        r->y = p->y - q->y;
                        r->id = p->id;
                    }
                }
            } else {
                for (p=pl1->start, q=pl2->start, r=pl3->start; (p<=pl1->end) && q && r; ++p, q=q->next, r=r->next) {
                    if (CheckID(p, q)) {
                        r->x = p->x - q->x;
                        r->y = p->y - q->y;
                        r->id = p->id;
                    }
                }
            }
        } else if (pl2->compact) {
            if (pl3->compact) {
                for (p=pl1->start, q=pl2->start, r=pl3->start; p && (q<=pl2->end) && (r<=pl3->end); p=p->next, ++q, ++r) {
                    if (CheckID(p, q)) {
                        r->x = p->x - q->x;
                        r->y = p->y - q->y;
                        r->id = p->id;
                    }
                }
            } else {
                for (p=pl1->start, q=pl2->start, r=pl3->start; p && (q<=pl2->end) && r; p=p->next, ++q, r=r->next) {
                    if (CheckID(p, q)) {
                        r->x = p->x - q->x;
                        r->y = p->y - q->y;
                        r->id = p->id;
                    }
                }
            }
        } else {
            if (pl3->compact) {
                for (p=pl1->start, q=pl2->start, r=pl3->start; p && q && (r<=pl3->end); p=p->next, q=q->next, ++r) {
                    if (CheckID(p, q)) {
                        r->x = p->x - q->x;
                        r->y = p->y - q->y;
                        r->id = p->id;
                    }
                }
            } else {
                for (p=pl1->start, q=pl2->start, r=pl3->start; p && q && r; p=p->next, q=q->next, r=r->next) {
                    if (CheckID(p, q)) {
                        r->x = p->x - q->x;
                        r->y = p->y - q->y;
                        r->id = p->id;
                    }
                }
            }
        }
    }
}

/**
 Subtract corresponding elements with identical IDs of two Euclidean point lists.

 @param[in]		pl1		First Euclidean point list.
 @param[in]		pl2		Second Euclidean point list.
 @result	    		New allocated Euclidean point array with substracted elements.
*/
plist *PointSubNew (plist *pl1, plist *pl2)
{
    plist *list = NULL;

    if (CheckSamePoints(pl1, pl2)) {
        list = NewPointList(pl1->num);
        PointSub(pl1, pl2, list);
        RelinkPoint(list);
    }
    return list;
}

//@}

/** @name Other Element-by-element Operations */
//@{

/**
 Absolute values of an Euclidean point.

 @param[in]			h	    Euclidean point.
 @result					Point or NOHOMO2.
*/
homo2 PointAbs (homo2 *h)
{
    homo2 r;

    if (!h) {
        Warning ("PointAbs failed!"); return NOHOMO2;
    }
    NormHomo(h);
    r.x = ABS(h->x);
    r.y = ABS(h->y);
    r.w = 1.0;
    r.id = h->id;

    return r;
}

/**
 Absolute values of an Euclidean point.

 @param[in]			p	    Euclidean point.
 @result					Point or NOHOMO2.
*/
homo2 PointAbs (pelem *p)
{
    homo2 r;

    if (!p) {
        Warning ("PointAbs failed!"); return NOHOMO2;
    }
    r.x = ABS(p->x);
    r.y = ABS(p->y);
    r.w = 1.0;
    r.id = p->id;

    return r;
}

/**
 Absolute values of an Euclidean point list.

 @param[in,out]		pl		Modified Euclidean point list.
*/
void PointAbs (plist *pl)
{
    pelem *p;

    if (CheckPoints(pl)) {
        if (pl->compact) {
            for (p=pl->start; p<=pl->end; ++p) {
                p->x = ABS(p->x);
                p->y = ABS(p->y);
            }
        } else {
            for (p=pl->start; p ; p=p->next) {
                p->x = ABS(p->x);
                p->y = ABS(p->y);
            }
        }
    }
}

/**
 Absolute values of an Euclidean point list.

 @param[in]		pl1		Euclidean point list.
 @param[out]	pl2		Overwritten Euclidean point list with absolute values.
*/
void PointAbs (plist *pl1, plist *pl2)
{
    pelem *p, *q;

    if (CheckSamePoints(pl1,pl2)) {
        if (pl1->compact && pl2->compact) {
            for (p=pl1->start, q=pl2->start; (p<=pl1->end) && (q<=pl2->end); ++p, ++q) {
                q->x = ABS(p->x);
                q->y = ABS(p->y);
                q->id = p->id;
            }
        } else if (pl1->compact) {
            for (p=pl1->start, q=pl2->start; (p<=pl1->end) && q; ++p, q=q->next) {
                q->x = ABS(p->x);
                q->y = ABS(p->y);
                q->id = p->id;
            }
        } else if (pl2->compact) {
            for (p=pl1->start, q=pl2->start; p && (q<=pl2->end); p=p->next, ++q) {
                q->x = ABS(p->x);
                q->y = ABS(p->y);
                q->id = p->id;
            }
        } else {
            for (p=pl1->start, q=pl2->start; p && q; p=p->next, q=q->next) {
                q->x = ABS(p->x);
                q->y = ABS(p->y);
                q->id = p->id;
            }
        }
    }
}

/**
 Absolute values of an Euclidean point list.

 @param[in]		pl		Euclidean point list.
 @result	    		New allocated Euclidean point array with absolute values.
*/
plist *PointAbsNew (plist *pl)
{
    plist *list = NULL;

    if (CheckPoints(pl)) {
        list = NewPointList(pl->num);
        PointAbs(pl, list);
        RelinkPoint(list);
    }
    return list;
}

/**
 Squared values of an Euclidean point.

 @param[in]			h	    Euclidean point.
 @result					Point or NOHOMO2.
*/
homo2 PointSqr (homo2 *h)
{
    homo2 r;

    if (!h) {
        Warning ("PointSqr failed!"); return NOHOMO2;
    }
    NormHomo(h);
    r.x = SQR(h->x);
    r.y = SQR(h->y);
    r.w = 1.0;
    r.id = h->id;

    return r;
}

/**
 Squared values of an Euclidean point.

 @param[in]			p	    Euclidean point.
 @result					Point or NOHOMO2.
*/
homo2 PointSqr (pelem *p)
{
    homo2 r;

    if (!p) {
        Warning ("PointSqr failed!"); return NOHOMO2;
    }
    r.x = SQR(p->x);
    r.y = SQR(p->y);
    r.w = 1.0;
    r.id = p->id;

    return r;
}

/**
 Squared values of an Euclidean point list.

 @param[in,out]		pl		Modified Euclidean point list.
*/
void PointSqr (plist *pl)
{
    PointMult(pl, pl);
}

/**
 Squared values of an Euclidean point list.

 @param[in]		pl1		Euclidean point list.
 @param[out]	pl2		Overwritten Euclidean point list with squared values.
*/
void PointSqr (plist *pl1, plist *pl2)
{
    PointMult(pl1, pl1, pl2);
}

/**
 Squared values of an Euclidean point list.

 @param[in]		pl		Euclidean point list.
 @result	    		New allocated Euclidean point array with squared values.
*/
plist *PointSqrNew (plist *pl)
{
    return PointMultNew(pl, pl);
}

/**
 Square root values of an Euclidean point.

 @param[in]			h	    Euclidean point.
 @result					Point or NOHOMO2.
*/
homo2 PointSqrt (homo2 *h)
{
    homo2 r;

    if (!h) {
        Warning ("PointSqrt failed!"); return NOHOMO2;
    }
    NormHomo(h);
    r.x = sqrt(h->x);
    r.y = sqrt(h->y);
    r.w = 1.0;
    r.id = h->id;

    return r;
}

/**
 Square root values of an Euclidean point.

 @param[in]			p	    Euclidean point.
 @result					Point or NOHOMO2.
*/
homo2 PointSqrt (pelem *p)
{
    homo2 r;

    if (!p) {
        Warning ("PointSqrt failed!"); return NOHOMO2;
    }
    r.x = sqrt(p->x);
    r.y = sqrt(p->y);
    r.w = 1.0;
    r.id = p->id;

    return r;
}

/**
 Square root values of an Euclidean point list.

 @param[in,out]		pl		Modified Euclidean point list.
*/
void PointSqrt (plist *pl)
{
    pelem *p;

    if (CheckPoints(pl)) {
        if (pl->compact) {
            for (p=pl->start; p<=pl->end; ++p) {
                p->x = sqrt(p->x);
                p->y = sqrt(p->y);
            }
        } else {
            for (p=pl->start; p ; p=p->next) {
                p->x = sqrt(p->x);
                p->y = sqrt(p->y);
            }
        }
    }
}

/**
 Square root values of an Euclidean point list.

 @param[in]		pl1		Euclidean point list.
 @param[out]	pl2		Overwritten Euclidean point list with square root values.
*/
void PointSqrt (plist *pl1, plist *pl2)
{
    pelem *p, *q;

    if (CheckSamePoints(pl1,pl2)) {
        if (pl1->compact && pl2->compact) {
            for (p=pl1->start, q=pl2->start; (p<=pl1->end) && (q<=pl2->end); ++p, ++q) {
                q->x = sqrt(p->x);
                q->y = sqrt(p->y);
                q->id = p->id;
            }
        } else if (pl1->compact) {
            for (p=pl1->start, q=pl2->start; (p<=pl1->end) && q; ++p, q=q->next) {
                q->x = sqrt(p->x);
                q->y = sqrt(p->y);
                q->id = p->id;
            }
        } else if (pl2->compact) {
            for (p=pl1->start, q=pl2->start; p && (q<=pl2->end); p=p->next, ++q) {
                q->x = sqrt(p->x);
                q->y = sqrt(p->y);
                q->id = p->id;
            }
        } else {
            for (p=pl1->start, q=pl2->start; p && q; p=p->next, q=q->next) {
                q->x = sqrt(p->x);
                q->y = sqrt(p->y);
                q->id = p->id;
            }
        }
    }
}

/**
 Square root values of an Euclidean point list.

 @param[in]		pl		Euclidean point list.
 @result	    		New allocated Euclidean point array with square root values.
*/
plist *PointSqrtNew (plist *pl)
{
    plist *list = NULL;

    if (CheckPoints(pl)) {
        list = NewPointList(pl->num);
        PointSqrt(pl, list);
        RelinkPoint(list);
    }
    return list;
}

//@}

/** @name Geometric Point Operations */
//@{

// ===========================================================================
//
// ===========================================================================

homo2 PointCross (homo2 *h1, homo2 *h2)
{
    homo2 r;

    if (!h1 || !h2) {
        Warning ("PointCross failed!"); return NOHOMO2;
    }
    r.x = h1->y * h2->w - h1->w * h2->y;
    r.y = h1->w * h2->x - h1->x * h2->w;
    r.w = h1->x * h2->y - h1->y * h2->x;
    r.id = 0;

    return r;
}

homo2 PointCross (pelem *p1, homo2 *h2)
{
    homo2 r;

    if (!p1 || !h2) {
        Warning ("PointCross failed!"); return NOHOMO2;
    }
    r.x = p1->y * h2->w -         h2->y;
    r.y =         h2->x - p1->x * h2->w;
    r.w = p1->x * h2->y - p1->y * h2->x;
    r.id = 0;

    return r;
}

homo2 PointCross (homo2 *h1, pelem *p2)
{
    homo2 r;

    if (!h1 || !p2) {
        Warning ("PointCross failed!"); return NOHOMO2;
    }
    r.x = h1->y *       - h1->w * p2->y;
    r.y = h1->w * p2->x - h1->x        ;
    r.w = h1->x * p2->y - h1->y * p2->x;
    r.id = 0;

    return r;
}

homo2 PointCross (pelem *p1, pelem *p2)
{
    homo2 r;

    if (!p1 || !p2) {
        Warning ("PointCross failed!"); return NOHOMO2;
    }
    r.x = p1->y         - p2->y        ;
    r.y = p2->x         - p1->x        ;
    r.w = p1->x * p2->y - p1->y * p2->x;
    r.id = 0;

    return r;
}

// ===========================================================================
//
// ===========================================================================

homo2 PointCrossNorm (homo2 *h1, homo2 *h2)
{
    homo2 r;

    r = PointCross (h1, h2);
    NormHomo (&r);
//  r.id = 0;

    return r;
}

homo2 PointCrossNorm (pelem *p1, homo2 *h2)
{
    homo2 r;

    r = PointCross (p1, h2);
    NormHomo (&r);
//  r.id = 0;

    return r;
}

homo2 PointCrossNorm (homo2 *h1, pelem *p2)
{
    homo2 r;

    r = PointCross (h1, p2);
    NormHomo (&r);
//  r.id = 0;

    return r;
}

homo2 PointCrossNorm (pelem *p1, pelem *p2)
{
    homo2 r;

    r = PointCross (p1, p2);
    NormHomo (&r);
//  r.id = 0;

    return r;
}

// ===========================================================================
//
// ===========================================================================

double PointDist (homo2 *h1, homo2 *h2)
{
    return sqrt (PointDistSqr(h1, h2));
}

double PointDist (pelem *h1, homo2 *h2)
{
    return sqrt (PointDistSqr(h1, h2));
}

double PointDist (homo2 *h1, pelem *h2)
{
    return sqrt (PointDistSqr(h1, h2));
}

double PointDist (pelem *h1, pelem *h2)
{
    return sqrt (PointDistSqr(h1, h2));
}

// ===========================================================================
//
// ===========================================================================

double PointDistSqr (homo2 *h1, homo2 *h2)
{
    if (!h1 || !h2) {
        Warning ("PointDistSqr failed!"); return 0.0;
    }
    NormHomo (h1);
    NormHomo (h2);
    return (SQR(h1->x - h2->x) + SQR(h1->y - h2->y));
}

double PointDistSqr (pelem *p1, homo2 *h2)
{
    if (!p1 || !h2) {
        Warning ("PointDistSqr failed!"); return 0.0;
    }
    NormHomo (h2);
    return (SQR(p1->x - h2->x) + SQR(p1->y - h2->y));
}

double PointDistSqr (homo2 *h1, pelem *p2)
{
    if (!h1 || !p2) {
        Warning ("PointDistSqr failed!"); return 0.0;
    }
    NormHomo (h1);
    return (SQR(h1->x - p2->x) + SQR(h1->y - p2->y));
}

double PointDistSqr (pelem *p1, pelem *p2)
{
    if (!p1 || !p2) {
        Warning ("PointDistSqr failed!"); return 0.0;
    }
    return (SQR(p1->x - p2->x) + SQR(p1->y - p2->y));
}

// ===========================================================================
//
// ===========================================================================

double PointLen (homo2 *h)
{
    if (!h) {
        Warning ("PointLen failed!"); return 0.0;
    }
    return sqrt (SQR(h->x) + SQR(h->y) + SQR(h->w));
}

double PointLen (pelem *p)
{
    if (!p) {
        Warning ("PointLen failed!"); return 0.0;
    }
    return sqrt (SQR(p->x) + SQR(p->y) + 1.0);
}

/**
 Scalar/inner/dot product of two points in Euclidean coordinates.

 @param[in]			h1	    First Euclidean point.
 @param[in]			h2	    Second Euclidean point.
 @result					Scalar product or 0.
 @note						Point ID is zero.
*/
double PointProd (homo2 *h1, homo2 *h2)
{
    if (!h1 || !h2) {
        Warning ("PointProd failed!"); return 0.0;
    }
    NormHomo(h1);
    NormHomo(h2);
    return h1->x*h2->x + h1->y*h2->y;
}

/**
 Scalar/inner/dot product of two points in Euclidean coordinates.

 @param[in]			p1	    First Euclidean point.
 @param[in]			h2	    Second Euclidean point.
 @result					Scalar product or 0.
 @note						Point ID is zero.
*/
double PointProd (pelem *p1, homo2 *h2)
{
    if (!p1 || !h2) {
        Warning ("PointProd failed!"); return 0.0;
    }
    NormHomo(h2);
    return p1->x*h2->x + p1->y*h2->y;
}

/**
 Scalar/inner/dot product of two points in Euclidean coordinates.

 @param[in]			h1	    First Euclidean point.
 @param[in]			p2	    Second Euclidean point.
 @result					Scalar product or 0.
 @note						Point ID is zero.
*/
double PointProd (homo2 *h1, pelem *p2)
{
    if (!h1 || !p2) {
        Warning ("PointProd failed!"); return 0.0;
    }
    NormHomo(h1);
    return h1->x*p2->x + h1->y*p2->y;
}

/**
 Scalar/inner/dot product of two points in Euclidean coordinates.

 @param[in]			p1	    First Euclidean point.
 @param[in]			p2	    Second Euclidean point.
 @result					Scalar product or 0.
 @note						Point ID is zero.
*/
double PointProd (pelem *p1, pelem *p2)
{
    if (!p1 || !p2) {
        Warning ("PointProd failed!"); return 0.0;
    }
    return p1->x*p2->x + p1->y*p2->y;
}

// ===========================================================================
//
// ===========================================================================

void ScalePoint (homo2 *h, double f)
{
    double t, s;

    if (h) {
        t = h->w;
        if (ISNONZERO(t)) {
            s = f / t;
            h->x *= s;
            h->y *= s;
            h->w = 1.0;
        }
    }
}

void ScalePoint( plist *pl, double factor){

    pelem *pE;
    if(pl&& ISINEQUAL(factor, 1.0)){
        for(pE=pl->start; pE; pE=pE->next){
            pE->x*=factor;
            pE->y*=factor;
        }
    }
}

void ScalePoint( pelem *pE, double factor){
    if(pE && ISINEQUAL(factor, 1.0)){
        pE->x*=factor;
        pE->y*=factor;
    }
}

// ===========================================================================
//
// ===========================================================================

homo2 TransformPoint (matrix *H, homo2 *h)
{
    homo2 r;

    if (!CheckSizeMatrix (H, 3, 3) || !h) {
        Warning ("TransformPoint failed!"); return NOHOMO2;
    }
    r.x = H->m[0][0] * h->x + H->m[0][1] * h->y + H->m[0][2] * h->w;
    r.y = H->m[1][0] * h->x + H->m[1][1] * h->y + H->m[1][2] * h->w;
    r.w = H->m[2][0] * h->x + H->m[2][1] * h->y + H->m[2][2] * h->w;
    r.id = h->id;

    return r;
}

void TransformPoint (matrix *H, pelem *p)
{
    homo2 r;

    CheckSizeMatrix (H, 3, 3);
    if (!p) {
        Warning ("TransformPoint failed!"); return;
    }
    r.x = H->m[0][0] * p->x + H->m[0][1] * p->y + H->m[0][2];
    r.y = H->m[1][0] * p->x + H->m[1][1] * p->y + H->m[1][2];
    r.w = H->m[2][0] * p->x + H->m[2][1] * p->y + H->m[2][2];
    if (ISZERO(r.w)) {
        Warning ("TransformPoint failed!"); return;
    }
    p->x = r.x / r.w;
    p->y = r.y / r.w;
}

void TransformPoint (matrix *H, plist *pl)
{
    pelem *p;

    if (pl) {
        for (p=pl->start; p; p=p->next) {
            TransformPoint (H, p);
        }
    }
}

// ===========================================================================
//
// ===========================================================================

homo2 TransformPointNew (matrix *H, pelem *p)
{
    homo2 r;

    if(!CheckSizeMatrix (H, 3, 3)){
        return NOHOMO2;
    }
    if (!p) {
        Warning ("TransformPoint failed!"); return NOHOMO2;
    }
    r.x = H->m[0][0] * p->x + H->m[0][1] * p->y + H->m[0][2];
    r.y = H->m[1][0] * p->x + H->m[1][1] * p->y + H->m[1][2];
    r.w = H->m[2][0] * p->x + H->m[2][1] * p->y + H->m[2][2];
    r.id = p->id;
    NormHomo(&r);

    return r;
}

plist *TransformPointNew (matrix *H, plist *pl)
{
    homo2 t;
    pelem *p;
    plist *list = NULL;

    if (pl) {
        for (p=pl->start; p; p=p->next) {
            t = TransformPointNew (H, p);
            list = AddPoint(list, &t);
        }
    }
    return list;
}

// ===========================================================================
//
// ===========================================================================

homo2 TransformNormPoint (matrix *H, homo2 *h)
{
    homo2 r;

    r = TransformPoint (H, h);
    NormHomo (&r);
//  r.id = h->id;

    return r;
}

// ===========================================================================
//
// ===========================================================================

homo2 TransformNormHomo (matrix *H, homo2 *h)
{
    homo2 r;

    r = TransformHomo (H, h);
    NormHomo (&r);
//  r.id = h->id;

    return r;
}

// ===========================================================================
//
// ===========================================================================

homo3 TransformNormHomo (matrix *H, homo3 *h)
{
    homo3 r;

    r = TransformHomo (H, h);
    NormHomo (&r);
//  r.id = h->id;

    return r;
}
//@}

// ===========================================================================
//
// ===========================================================================

void ScaleHomo (homo2 *h, double f)
{
    double t, s;

	if (h) {
		t = h->w;
		if (ISNONZERO(t)) {
			s = f / t;
			h->x *= s;
			h->y *= s;
			h->w = 1.0;
		}
	}
}

// ===========================================================================
//
// ===========================================================================

void ScaleHomo (homo3 *h, double f)
{
    double t, s;

	if (h) {
		t = h->W;
		if (ISNONZERO(t)) {
			s = f / t;
			h->X *= s;
			h->Y *= s;
			h->Z *= s;
			h->W = 1.0;
		}
	}
}

// ===========================================================================
//
// ===========================================================================

homo2 HomoCross (homo2 *h1, homo2 *h2)
{
    homo2 r;

	if (!h1 || !h2) {
		Warning ("HomoCross failed!"); return NOHOMO2;
	}
	r.x = h1->y * h2->w - h1->w * h2->y;
	r.y = h1->w * h2->x - h1->x * h2->w;
	r.w = h1->x * h2->y - h1->y * h2->x;
	r.id = 0;

    return r;
}

homo2 HomoCross (pelem *p1, homo2 *h2)
{
    homo2 r;

	if (!p1 || !h2) {
		Warning ("HomoCross failed!"); return NOHOMO2;
	}
	r.x = p1->y * h2->w -         h2->y;
	r.y =         h2->x - p1->x * h2->w;
	r.w = p1->x * h2->y - p1->y * h2->x;
	r.id = 0;

    return r;
}

homo2 HomoCross (homo2 *h1, pelem *p2)
{
    homo2 r;

	if (!h1 || !p2) {
		Warning ("HomoCross failed!"); return NOHOMO2;
	}
	r.x = h1->y *       - h1->w * p2->y;
	r.y = h1->w * p2->x - h1->x        ;
	r.w = h1->x * p2->y - h1->y * p2->x;
	r.id = 0;

    return r;
}

homo2 HomoCross (pelem *p1, pelem *p2)
{
    homo2 r;

	if (!p1 || !p2) {
		Warning ("HomoCross failed!"); return NOHOMO2;
	}
	r.x = p1->y         - p2->y        ;
	r.y = p2->x         - p1->x        ;
	r.w = p1->x * p2->y - p1->y * p2->x;
	r.id = 0;

    return r;
}

// ===========================================================================
//
// ===========================================================================

homo2 NormHomoCross (homo2 *h1, homo2 *h2)
{
    homo2 r;

    r = HomoCross (h1, h2);
    NormHomo (&r);
//  r.id = 0;

    return r;
}

homo2 NormHomoCross (pelem *p1, homo2 *h2)
{
    homo2 r;

    r = HomoCross (p1, h2);
    NormHomo (&r);
//  r.id = 0;

    return r;
}

homo2 NormHomoCross (homo2 *h1, pelem *p2)
{
    homo2 r;

    r = HomoCross (h1, p2);
    NormHomo (&r);
//  r.id = 0;

    return r;
}

homo2 NormHomoCross (pelem *p1, pelem *p2)
{
    homo2 r;

    r = HomoCross (p1, p2);
    NormHomo (&r);
//  r.id = 0;

    return r;
}

// ===========================================================================
//
// ===========================================================================

double HomoLen (homo2 *h)
{
	if (!h) {
		Warning ("HomoLen failed!"); return 0.0;
	}
	return sqrt (SQR(h->x) + SQR(h->y) + SQR(h->w));
}

// ===========================================================================
//
// ===========================================================================

double HomoLen (homo3 *h)
{
	if (!h) {
		Warning ("HomoLen failed!"); return 0.0;
	}
    return sqrt (SQR(h->X) + SQR(h->Y) + SQR(h->Z) + SQR(h->W));
}

// ===========================================================================
//
// ===========================================================================

double HomoDistSqr (homo2 *h1, homo2 *h2)
{
	if (!h1 || !h2) {
		Warning ("HomoDistSqr failed!"); return 0.0;
	}
	NormHomo (h1);
	NormHomo (h2);
	return (SQR(h1->x - h2->x) + SQR(h1->y - h2->y));
}

double HomoDistSqr (pelem *p1, homo2 *h2)
{
	if (!p1 || !h2) {
		Warning ("HomoDistSqr failed!"); return 0.0;
	}
	NormHomo (h2);
	return (SQR(p1->x - h2->x) + SQR(p1->y - h2->y));
}

double HomoDistSqr (homo2 *h1, pelem *p2)
{
	if (!h1 || !p2) {
		Warning ("HomoDistSqr failed!"); return 0.0;
	}
	NormHomo (h1);
	return (SQR(h1->x - p2->x) + SQR(h1->y - p2->y));
}

double HomoDistSqr (pelem *p1, pelem *p2)
{
	if (!p1 || !p2) {
		Warning ("HomoDistSqr failed!"); return 0.0;
	}
	return (SQR(p1->x - p2->x) + SQR(p1->y - p2->y));
}

// ===========================================================================
//
// ===========================================================================

double HomoDist (homo2 *h1, homo2 *h2)
{
	return sqrt (HomoDistSqr(h1, h2));
}

double HomoDist (pelem *h1, homo2 *h2)
{
	return sqrt (HomoDistSqr(h1, h2));
}

double HomoDist (homo2 *h1, pelem *h2)
{
	return sqrt (HomoDistSqr(h1, h2));
}

double HomoDist (pelem *h1, pelem *h2)
{
	return sqrt (HomoDistSqr(h1, h2));
}

// ===========================================================================
//
// ===========================================================================

double HomoDistSqr (homo3 *h1, homo3 *h2)
{
	if (!h1 || !h2) {
		Warning ("HomoDistSqr failed!"); return 0.0;
	}
	NormHomo (h1);
	NormHomo (h2);
	return (SQR(h1->X - h2->X) + SQR(h1->Y - h2->Y) + SQR(h1->Z - h2->Z));
}

double HomoDistSqr (xelem *x1, homo3 *h2)
{
	if (!x1 || !h2) {
		Warning ("HomoDistSqr failed!"); return 0.0;
	}
	NormHomo (h2);
	return (SQR(x1->X - h2->X) + SQR(x1->Y - h2->Y) + SQR(x1->Z - h2->Z));
}

double HomoDistSqr (homo3 *h1, xelem *x2)
{
	if (!h1 || !x2) {
		Warning ("HomoDistSqr failed!"); return 0.0;
	}
	NormHomo (h1);
	return (SQR(h1->X - x2->X) + SQR(h1->Y - x2->Y) + SQR(h1->Z - x2->Z));
}

double HomoDistSqr (xelem *x1, xelem *x2)
{
	if (!x1 || !x2) {
		Warning ("HomoDistSqr failed!"); return 0.0;
	}
	return (SQR(x1->X - x2->X) + SQR(x1->Y - x2->Y) + SQR(x1->Z - x2->Z));
}

// ===========================================================================
//
// ===========================================================================

double HomoDist (homo3 *h1, homo3 *h2)
{
    return sqrt (HomoDistSqr(h1, h2));
}

double HomoDist (xelem *x1, homo3 *h2)
{
    return sqrt (HomoDistSqr(x1, h2));
}

double HomoDist (homo3 *h1, xelem *x2)
{
    return sqrt (HomoDistSqr(h1, x2));
}

double HomoDist (xelem *x1, xelem *x2)
{
    return sqrt (HomoDistSqr(x1, x2));
}

double HomoMult (homo2 *h1, homo2 *h2)
{
	if (!h1 || !h2) {
		Warning ("HomoMult failed!"); return 0.0;
	}
//	NormHomo (h1);
//	NormHomo (h2);
	return (h1->x * h2->x + h1->y * h2->y + h1->w * h2->w);
}

double HomoMult (pelem *p1, homo2 *h2)
{
	if (!p1 || !h2) {
		Warning ("HomoMult failed!"); return 0.0;
	}
//	NormHomo (h2);
	return (p1->x * h2->x + p1->y * h2->y + h2->w);
}

double HomoMult (homo2 *h1, pelem *p2)
{
	return HomoMult (p2, h1);
}

double HomoMult (pelem *p1, pelem *p2)
{
	if (!p1 || !p2) {
		Warning ("HomoMult failed!"); return 0.0;
	}
	return (p1->x * p2->x + p1->y * p2->y + 1.0);
}

// ===========================================================================
//
// ===========================================================================

double HomoMult (homo3 *h1, homo3 *h2)
{
	if (!h1 || !h2) {
		Warning ("HomoMult failed!"); return 0.0;
	}
//	NormHomo (h1);
//	NormHomo (h2);
	return (h1->X * h2->X + h1->Y * h2->Y + h1->Z * h2->Z + h1->W * h2->W);
}

double HomoMult (xelem *x1, homo3 *h2)
{
	if (!x1 || !h2) {
		Warning ("HomoMult failed!"); return 0.0;
	}
//	NormHomo (h2);
	return (x1->X * h2->X + x1->Y * h2->Y + x1->Z * h2->Z + h2->W);
}

double HomoMult (homo3 *h1, xelem *x2)
{
	return HomoMult (x2, h1);
}

double HomoMult (xelem *x1, xelem *x2)
{
	if (!x1 || !x2) {
		Warning ("HomoMult failed!"); return 0.0;
	}
	return (x1->X * x2->X + x1->Y * x2->Y + x1->Z * x2->Z + 1.0);
}

// ===========================================================================
//
// ===========================================================================

homo2 TransformHomo (matrix *H, homo2 *h)
{
    homo2 r;

    if (!CheckSizeMatrix (H, 3, 3) || !h) {
		Warning ("TransformHomo failed!"); return NOHOMO2;
	}
	r.x = H->m[0][0] * h->x + H->m[0][1] * h->y + H->m[0][2] * h->w;
	r.y = H->m[1][0] * h->x + H->m[1][1] * h->y + H->m[1][2] * h->w;
	r.w = H->m[2][0] * h->x + H->m[2][1] * h->y + H->m[2][2] * h->w;
	r.id = h->id;

	return r;
}

// ===========================================================================
//
// ===========================================================================

homo3 TransformHomo (matrix *H, homo3 *X)
{
    homo3 r;

    if (!CheckSizeMatrix (H, 4, 4) || !X) {
		Warning ("TransformHomo failed!"); return NOHOMO3;
	}
	r.X = H->m[0][0] * X->X + H->m[0][1] * X->Y + H->m[0][2] * X->Z + H->m[0][3] * X->W;
	r.Y = H->m[1][0] * X->X + H->m[1][1] * X->Y + H->m[1][2] * X->Z + H->m[1][3] * X->W;
	r.Z = H->m[2][0] * X->X + H->m[2][1] * X->Y + H->m[2][2] * X->Z + H->m[2][3] * X->W;
	r.W = H->m[3][0] * X->X + H->m[3][1] * X->Y + H->m[3][2] * X->Z + H->m[3][3] * X->W;
    r.id = X->id;

	return r;
}
// ===========================================================================
//
// ===========================================================================


// ===========================================================================
//
// ===========================================================================



/** @name Point Statistics */
//@{

//@}


/** @name Point Reorganisation */
//@{

/**
 Convert linked point list to a compact array.

 @param[in]		pl		Modified point list.
 @result				True, if conversion was successful.
*/
bool PointArray (plist *pl)
{
    pelem *p, *q, *next, *start;

    if (CheckPoints(pl)) {
        if (pl->compact) {
            return true;
        } else {
            start = NewPoint (pl->num);
            if (start) {
                for (p = pl->start, q = start; p; p = next, ++q) {
                    CopyPoint(p, q);
                    q->next = q+1;
                    next = p->next;
                    hfree (p); p = NULL;
                }
                pl->start     = start;
                pl->end       = start + pl->num-1;
                pl->compact   = true;
                pl->end->next = NULL;
                return true;
            }
        }
    }
    return false;
}

/**
 Convert compact array to a linked point list.

 @param[in]		pl		Modified point array.
 @result				True, if conversion was successful.
*/
bool PointList (plist *pl)
{
    plist *list = NULL;
    pelem *p;

    if (CheckPoints(pl)) {
        if (!pl->compact) {
            return true;
        } else {
            Warning("PointList: Convert array to linked list");

            for (p = pl->start; p <= pl->end; ++p) {
                list = AddPoint(list, p);
            }
            hfree (pl->start); // pl->start = NULL;
//			pl->num     = list->num;
            pl->compact = false;
            pl->start   = list->start;
            pl->end     = list->end;
            hfree (list); list = NULL;
            return true;
        }
    }
    return false;
}

/**
 Join two point lists \f$ \mathbf{c} = \mathbf{a}(0) \ldots \mathbf{a}(m-1), \mathbf{b}(0) \ldots \mathbf{b}(n-1) \f$.

 @param[in]		pl	Modified first point list with \a m elements.
 @param[in]		ql	Modified second point list with \a n elements.
 @result			New allocated point list \a C with \a m + \a n merged elements or NULL.
 @note				Input lists are empty but the structure is not free.
*/
plist *JoinPoint (plist *pl, plist *ql)
{
    plist *res;
	long num;

	if (!pl && !ql) return NULL;

	res = NewPointList();
	if (!res) {
		Warning ("JoinPoint: out of memory"); return NULL;
	}
	if (!pl) {
		SwapPoint(ql, res); return res;
	}
	if (!ql) {
		SwapPoint(pl, res); return res;
	}
	num = pl->num + ql->num;
	if (num < 1) {
		return NULL;
	}
    if (pl->compact) { // Join compact pl and linked ql
		res->start = (pelem *) hrealloc(pl->start, num, sizeof(pelem));
		if (!res->start) {
			Warning ("JoinPoint: out of memory"); FreePoint(res); return NULL;
		}
		res->end = res->start + num-1;
		res->compact = true;
		res->num = num;
		CopyPoint(ql, res, pl->num);
		RelinkPoint (res);
		hfree(ql->start);
	} else if (ql->compact) { // Join compact yl and linked xl
		res->start = (pelem *) hrealloc(ql->start, num, sizeof(pelem));
		if (!res->start) {
			Warning ("JoinPoint: out of memory"); FreePoint(res); return NULL;
		}
		res->end = res->start + num-1;
		res->compact = true;
		res->num = num;
		CopyPoint(pl, res, ql->num);
		RelinkPoint (res);
		hfree(pl->start);
	} else { // Join linked pl and ql
		pl->end->next = ql->start; // link lists
		res->start = pl->start;
		res->end = ql->end;
		res->num = pl->num + ql->num;
	}
	pl->num = 0; pl->start = pl->end = NULL; // Clean old lists
	ql->num = 0; ql->start = ql->end = NULL;
    return res;
}

/**
 Join multiple point lists.

 @param[in]		pl	Array of point lists.
 @param[in]		n	Number of lists.
 @result			New allocated point list with merged lists or NULL.
*/
plist *JoinPointNew (plist *pl[], int n)
{
    plist *res = NULL;
	long num = 0;
	int i;

    if (!pl) return NULL;

	for (i=0; i<n; ++i) {
		if (pl[i]) {
			num += pl[i]->num;
		}
	}
	if (num < 1) {
		return NULL;
	}
	res = NewPointList(num);
	if (!res) {
		Warning ((char*) "JoinPointNew: out of memory"); return NULL;
	}
	res->end = res->start + num-1;
	res->compact = true;
	res->num = num;

	num = 0;
	for (i=0; i<n; ++i) {
		if (pl[i]) {
			CopyPoint(pl[i], res, num);
			num += pl[i]->num;
		}
	}
	RelinkPoint (res);

    return res;
}

/**
 Join two point lists \f$ \mathbf{p1}(0) \ldots \mathbf{p1}(m-1), \mathbf{p2}(0) \ldots \mathbf{p2}(n-1)\f$.

 @param[in]		p1	First point list with \a m elements.
 @param[in]		p2	Second point list with \a n elements.
 @result			New allocated point list \a C with \a m + \a n merged elements or NULL.
*/
plist *JoinPointNew (plist *p1, plist *p2)
{
	plist *pl[2];

	pl[0] = p1; pl[1] = p2;

	return JoinPointNew (pl, 2);
}

/**
 Join three point lists \f$ \mathbf{p1}(0) \ldots \mathbf{p1}(m-1), \mathbf{p2}(0) \ldots \mathbf{p2}(n-1), \mathbf{p3}(0) \ldots \mathbf{p3}(o-1)\f$.

 @param[in]		p1	First point list with \a m elements.
 @param[in]		p2	Second point list with \a n elements.
 @param[in]		p3	Third point list with \a o elements.
 @result			New allocated point list \a C with \a m + \a n + \a o merged elements or NULL.
*/
plist *JoinPointNew (plist *p1, plist *p2, plist *p3)
{
	plist *pl[3];

	pl[0] = p1; pl[1] = p2; pl[2] = p3;

	return JoinPointNew (pl, 3);
}

/**
 Link elements of a compact point array.

 @param[in]		pl	Point list.
*/
void RelinkPoint (plist *pl)
{
    pelem *p;

    if (CheckPoints(pl) && pl->compact) {
        for (p=pl->start; p<=pl->end; ++p) {
            p->next = p+1;
        }
        pl->end->next = NULL;
    }
}


static int
CompareIDp (pelem *p1, pelem *p2)
{
	if (p1->id == p2->id) {
		return (int)((PTRINT)p1->next - (PTRINT)p2->next);
	} else {
		return (int)(p1->id - p2->id);
	}
}

/**
 Sort point IDs into ascending order without linking \f$ \mathbf{cl}(0) \le \ldots \le \mathbf{cl}(n-1) \f$.

 @param[in,out]	pl	Modified point list with \a n elements.
*/

void SortPointUnlinked (plist *pl)
{
    if (!CheckPointsSorted(pl)) {
        if (PointArray(pl)) {         // Convert linked list to compact array
            qsort((char *)pl->start, (unsigned)pl->num, sizeof(pelem), (int (*)(const void *, const void *)) CompareIDp);
            //qsort((char *)pl->start, (unsigned)pl->num, sizeof(pelem), (int (*)(pelem *, pelem *)) CompareID);
        }
    }
}

/**
 Sort point IDs into ascending order \f$ \mathbf{cl}(0) \le \ldots \le \mathbf{cl}(n-1) \f$.

 @param[in,out]	pl	Modified point list with \a n elements.
*/
void SortPoint (plist *pl)
{
	SortPointUnlinked (pl);
    RelinkPoint(pl);
}

static int
CompareDisparity (plist *p1, plist *p2)
{
    return (int)(p1->num - p2->num);
}

/**
 Sort corresponding point disparities into ascending order \f$ \mathbf{cl}(0) \le \ldots \le \mathbf{cl}(n-1) \f$.

 @param[in,out]	pl1	Modified first point list with \a n elements.
 @param[in,out]	pl2	Modified second point list with \a n elements.
*/
void SortPointDisparity (plist **pl1, plist **pl2)
{
    pelem *p1, *p2;
    int i, num;
    plist *s, *sl1 = NULL, *sl2 = NULL;

    if (*pl1 && *pl2) {
        num = (*pl1)->num;
        if ((num != (*pl2)->num) || (num < 1)) {
            Warning ("Lists have different sizes!"); return;
        }
        s = (plist *) halloc (num, sizeof (plist));
        for (i=0, p1=(*pl1)->start, p2=(*pl2)->start; p1 && p2; p1=p1->next, p2=p2->next, ++i) {
            s[i].num   = (long)PointDistSqr(p1, p2);
            s[i].start = p1;
            s[i].end   = p2;
        }
        qsort((char *)s, (unsigned)num, sizeof(plist), (int (*)(const void *, const void *)) CompareDisparity);

        for (i=0; i<num; ++i) {
            sl1 = AddPoint(sl1, s[i].start);
            sl2 = AddPoint(sl2, s[i].end);
        }
        hfree(s);
        FreePoint(*pl1);
        FreePoint(*pl2);
        *pl1 = sl1;
        *pl2 = sl2;
    }
}
//@}


/** @name Logical Point Operations */
//@{

//@}


/** @name Other Operations */
//@{

// ===========================================================================
//
// ===========================================================================

void PrintPoint (pelem *p)
{
    if (p) {
        Report ("ID             X           Y       R     G     B     Q");
        Report ("--------+-----------+-----------+-----+-----+-----+--------------");
        Report ("%07ld   %9.3f   %9.3f   %3d   %3d   %3d   %e", p->id, p->x, p->y,
                 GET_R(p->color), GET_G(p->color), GET_B(p->color), p->quality);
    }
}

void PrintPoint (plist *pl)
{
    char *buf = "unsorted";
    pelem *p;

    if (pl) {
        if (pl->compact) {
            Debug ("Compact point array with %d %s elements", pl->num, (CheckPointsSorted(pl) ? buf+2 : buf));
        } else {
            Debug ("Linked point list with %d %s elements", pl->num, (CheckPointsSorted(pl) ? buf+2 : buf));
        }
        Report ("ID             X           Y       R     G     B     Q");
        Report ("--------+-----------+-----------+-----+-----+-----+--------------");
		if (pl->compact) {
			for (p=pl->start; p<=pl->end; ++p) {
				Report ("%07ld   %9.3f   %9.3f   %3d   %3d   %3d   %e", p->id, p->x, p->y,
						 GET_R(p->color), GET_G(p->color), GET_B(p->color), p->quality);
			}
		} else {
			for (p=pl->start; p; p=p->next) {
				Report ("%07ld   %9.3f   %9.3f   %3d   %3d   %3d   %e", p->id, p->x, p->y,
						 GET_R(p->color), GET_G(p->color), GET_B(p->color), p->quality);
			}
		}
    }
}

//REsample


// ===========================================================================
//
// ===========================================================================

byte InterpolBiLinear (double x, double y, int cols, image *pic)
{
    int ix,iy;
    double dx,dy;
    double a,b,c,d;
    ulong i;

    ix = (int)floor(x);
    iy = (int)floor(y);

    if ((ix < 0) || (iy < 0) || (ix >= pic->w) || (iy >= pic->h)) {
        return OUTSIDE_IMAGE;
    }
    i = (iy * pic->w + ix) * pic->typ + cols;

    a = (double)pic->d[i];
    if (ix == pic->w - 1) {
        b = a;
    } else {
        b = (double)pic->d[i + pic->typ];
    }
    if (iy == pic->h - 1) {
        c = a;
    } else {
        c = (double)pic->d[i + pic->w * pic->typ];
    }
    if (ix == pic->w - 1) {
        d = c;
    } else if (iy == pic->h - 1) {
        d = b;
    } else {
        d = (double)pic->d[i + (pic->w + 1) * pic->typ];
    }
    dx = x - (double)ix;
    dy = y - (double)iy;

    a += (c-a) * dy;                               // bi-lineare Interpolation
    b += (d-b) * dy;
    a += (b-a) * dx;

    return CLIP_GRAY_double (a);
}

// ===========================================================================
//   cubic parameter a,b,c,d (param[0..3]) der 4 Punkte
//   x[0..3] = [-1,0,1,2] und f(x) = f[0..3]
// ===========================================================================

static void CubicParam (double *f, double *param)
{
    param[0] = (  -f[0] + 3*f[1] - 3*f[2] + f[3]) / 6;
    param[1] = (   f[0] - 2*f[1] +   f[2]       ) / 2;
    param[2] = (-2*f[0] - 3*f[1] + 6*f[2] - f[3]) / 6;
    param[3] = f[1];
}

// ===========================================================================
//   x^3+b x^2+c x+d mit {a,b,c,d}=q[0..3]
// ===========================================================================

static double CubicEval (double *q, double x)
{
    return ((((q[0] * x) + q[1]) * x + q[2]) * x + q[3]);
}

// ===========================================================================
//
// ===========================================================================

byte InterpolBiCubic (double x, double y, int cols, image *pic)
{
    int ix, iy, cx, cy, j, k;
    double dx, dy;
    double f[4], q[4], v[4];
    ulong i;

    if ((pic->w < 4) || (pic->h < 4)) {
		Warning ("InterpolBiCubic: image too small!"); return OUTSIDE_IMAGE;
    }
    ix = (int)floor (x);
    iy = (int)floor (y);

    if ((ix < 0) || (iy < 0) || (ix >= pic->w) || (iy >= pic->h)) {
        return OUTSIDE_IMAGE;
    }
    dx = x - (double)ix;
    dy = y - (double)iy;

    cx = MIN (MAX (ix - 1, 0), pic->w - 4);
    for (j = 0; j < 4; ++j) {
        cy = MIN (MAX (iy - 1 + j, 0), pic->h - 1);
        for (k = 0; k < 4; ++k) {
            i = (cy * pic->w + cx + k) * pic->typ + cols;
            f[k] = (double)pic->d[i];
        }
        CubicParam (f, q);
        v[j] = CubicEval (q, dx);
    }
    CubicParam (v, q);

    return CLIP_GRAY_double (CubicEval (q, dy));
}

//TAGS
void
WriteTags (FILE *fp, int tagc, char *tagv[])
{
	char *buf, *p, *t;
	int i;

	for (i=0; i<tagc; ++i) {
		t = buf = strdup(tagv[i]);
		if (buf) {
			fprintf (fp, COMMENT);
			while ((p = strchr (buf, '\n')) != NULL) {
				*p = '\0';
				fprintf (fp, "%s\n%s", buf, COMMENT);
				buf = p+1;
			}
			fprintf (fp, "%s\n", buf);
			if (t) {
				hfree(t); t = NULL;
			}
		}
	}
}

// ===========================================================================
//
// ===========================================================================

void
ReadTags (char *name, int *tagc, char **tagv[])
{
	char buf[BUF_SIZE];
	FILE *fp;
	int off, j;
	bool cr;

	if (tagc) {
		*tagc = 0;
		off = (int)strlen(COMMENT);

		if ((fp = fopen (name, "rb")) == NULL) {
			ErrorExit (OPEN_ERROR, name);
		}
		do {
			if (fgets (buf, BUF_SIZE, fp) == NULL) {
				break;
			}
			if (!strncmp(buf, COMMENT, off)) {
				(*tagc)++;
			}
		} while (strchr (buf, '\n') != NULL);

		if (*tagc) Debug ("Found %d comments", *tagc);

		if (*tagc && tagv) {
			fseek (fp, 0, SEEK_SET);

			*tagv = (char **) halloc(*tagc, sizeof(char *));
			j = 0;
			do {
			    if (fgets (buf, BUF_SIZE, fp) == NULL) {
					break;
				}
				if (!strncmp(buf, COMMENT, off)) {
					buf[strlen(buf) - 1] = '\0';                 // remove return
					(*tagv)[j++] = strdup(buf + off);            // remove comment
					cr = true;
				} else {
					cr = false;
				}
			} while (cr || (strchr (buf, '\n') != NULL));
		}
	    fclose (fp);
	}
}

//VEKTOR
vektor *vec_alloc(int len)
{
    vektor *A;

    if (len < 1) {
        Warning ("vec_alloc: size %d too small", len); return NULL;
    }
    A = (vektor *) halloc(1, sizeof (vektor));
    if (A) {
        A->v = (double *) halloc(len, sizeof (double));
        if (A->v) {
            A->len = len;
        } else {
            A->len = 0;
        }
    }
    return A;
}

/**
 Duplicate a vector \f$ \mathbf{b}(i) = \mathbf{a}(i) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	Source vector with \a n elements.
 @result			New allocated destination vector \a B with \a n duplicated elements or NULL.
*/
vektor *vec_clone (vektor *A)
{
    vektor *B = NULL;

    if (CheckVector(A)) {
        B = vec_alloc(A->len);
        vec_copy(A, B);
    }
    return B;
}

/**
 Partial duplicate a vector \f$ \mathbf{b}(i) = \mathbf{a}(i+s) \f$ for \f$ i=0, \ldots ,e-s \f$.

 @param[in]		A	Source vector with \a n elements.
 @param[in]		s	Start index in \a A.
 @param[in]		e	End index in \a A.
 @result			New allocated destination vector \a B with duplicated \a e-s+1 elements or NULL.
*/
vektor *vec_clone(vektor *A, int s, int e)
{
    vektor *B = NULL;

    if (CheckIndex(A, s) && CheckIndex(A, e)) {
        if (s > e) {
            SWAP(int, s, e);
        }
        B = vec_alloc(e-s+1);
        vec_copy(A, s, e, B);
    }
    return B;
}

/**
 Partial vector copy \f$ \mathbf{b}(i) = \mathbf{a}(i) \f$ for \f$ i=0, \ldots ,\min(m,n)-1 \f$.

 @param[in]		A	Source vector with \a m elements.
 @param[out]	B	Overwritten destination vector with \a n elements.
*/
void vec_copy(vektor *A, vektor *B)
{
    int a, b;

    if (CheckVector(A) && CheckVector(B)) {
        a = A->len;
        b = B->len;
        if (a > b) {
            Warning("vec_copy: skipped %d elements!", a-b);
        }
        memcpy(B->v, A->v, sizeof(double) * MIN(a, b));
    }
}

/**
 Partial vector copy \f$ \mathbf{b}(i+s) = \mathbf{a}(i) \f$ for \f$ i=0, \ldots ,\min(m,n-s)-1 \f$.

 @param[in]		A	Source vector with \a m elements.
 @param[out]	B	Overwritten destination vector with \a n elements.
 @param[in]		s	Start index in vector \a B.
*/
void vec_copy(vektor *A, vektor *B, int s)
{
    int a, b;

    if (CheckVector(A) && CheckIndex(B, s)) {
        a = A->len;
        b = B->len - s;
        if (a > b) {
            Warning("vec_copy: skipped %d elements!", a-b);
        }
        memcpy(B->v + s, A->v, sizeof(double) * MIN(a, b));
    }
}

/**
 Partial vector copy \f$ \mathbf{b}(i+s_2) = \mathbf{a}(i+s_1) \f$ for \f$ i=0, \ldots ,\min(e-s_1,n-s_2-1) \f$.

 @param[in]		A	Source vector with \a n elements.
 @param[in]		s1	Start index in \a A.
 @param[in]		e	End index in \a A.
 @param[out]	B	Overwritten destination vector with \a n elements.
 @param[in]		s2	Start index in \a B.
*/
void vec_copy(vektor *A, int s1, int e, vektor *B, int s2)
{
    int a, b;

    if (CheckIndex(A, s1) && CheckIndex(A, e) && CheckIndex(B, s2)) {
        if (s1 > e) {
            SWAP(int, s1, e);
        }
        a = e-s1+1;
        b = B->len - s2;
        if (a > b) {
            Warning("vec_copy: skipped %d elements!", a-b);
        }
        memcpy(B->v + s2, A->v + s1, sizeof(double) * MIN(a, b));
    }
}

/**
 Partial vector copy \f$ \mathbf{b}(i) = \mathbf{a}(i+s) \f$ for \f$ i=0, \ldots ,\min(e-s,n-1) \f$ elements.

 @param[in]		A	Source vector with \a n elements.
 @param[in]		s	Start index in \a A.
 @param[in]		e	End index in \a A.
 @param[out]	B	Overwritten destination vector with \a n elements.
*/
void vec_copy(vektor *A, int s, int e, vektor *B)
{
    vec_copy(A, s, e, B, 0);
}

/**
 Fill vector with given initial value \a v \f$ \mathbf{a}(i) = v \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Modified input vector with \a n elements.
 @param[in]		v	Vector element.
*/
void vec_fill(vektor *A, double v)
{
    double *aptr, *eptr;

    if (CheckVector(A)) {
        aptr = A->v;
        eptr = aptr + A->len;
        for (; aptr < eptr; ) {
            *aptr++ = v;
        }
    }
}

/**
 Fill vector with given initial value \a v from index \a s to index \a e \f$ \mathbf{a}(i+s) = v \f$ for \f$ i=0, \ldots ,e-s \f$.

 @param[in,out]	A	Modified input vector with \a n elements.
 @param[in]		s	Start index in \a A.
 @param[in]		e	End index in \a A.
 @param[in]		v	Vector element.
*/
void vec_fill(vektor *A, int s, int e, double v)
{
    double *aptr, *eptr;

    if (CheckIndex(A, s) && CheckIndex(A, e)) {
        if (s > e) {
            SWAP(int, s, e);
        }
        aptr = A->v + s;
        eptr = aptr + e;
        for (; aptr <= eptr; ) {
            *aptr++ = v;
        }
    }
}

/**
 Free memory of a vector

 @param[in]		A	Allocated vector with \a n elements.
 @result			NULL.
*/
vektor *vec_free(vektor *A)
{
    if (A) {
        if (A->v) {
            hfree(A->v);
        }
        hfree(A);
    }
    return NULL;
}

/**
 Vector allocation and initialization with variable number of elements.

 @param[in]		len		Number of vector elements.
 @param[in]		...		Variable list of \f$ len > 0 \f$ double values.

 @warning				Number and type of arguments are important but cannot be checked!
*/
vektor *vec_init(int len, ...)
{
    vektor *A = NULL;
    double *aptr, *eptr;
    va_list args;

    va_start(args, len);
    if (len < 1) {
        Warning ("vec_init: size %d too small", len);
    } else {
        A = vec_alloc(len);
        if (A) {
            aptr = A->v;
            eptr = aptr + len;
            for (; aptr < eptr; ) {
                *aptr++ = va_arg(args, double);
            }
        }
    }
    va_end(args);

    return A;
}

/**
 Fill vector with random values \f$ \mathbf{a}(i) = {\rm rand}(-1,1) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in,out]		A	Modified input vector with \a n random elements.
*/
void vec_random(vektor *A)
{
    double *aptr, *eptr;

    if (CheckVector(A)) {
        aptr = A->v;
        eptr = aptr + A->len;
        for (; aptr < eptr; ) {
            *aptr++ = Random(2.0) - 1.0;
        }
    }
}

/**
 Dynamically reallocate new vector length.

 @param[in,out]	A	Modified input vector with \a n elements.
 @param[in]		len	New vector length.
*/
void vec_realloc(vektor *A, int len)
{
    double *res, *aptr, *eptr;

    if (len <= 0) {
        Warning ("vec_realloc: size %d too small", len); return;
    }
    if (CheckVector(A)) {
		if (A->len == len) {
			return;                                       // Nothing to do ...
		}
        res = (double *) hrealloc (A->v, len, sizeof (double));
        if (!res) {
            return;                                        // A left unchanged
        }
        if (len > A->len) {                           // Initialize new memory
            aptr = res + A->len;
            eptr = res + len;
            for (; aptr < eptr; ) {
                *aptr++ = 0.0;
            }
        }
        A->v = res;                                           // Update header
        A->len = len;
    }
}
//@}


/** @name Arithmetic Operations with Scalars */
//@{

/**
 Add scalar to a vector \f$ \mathbf{a}(i) = \mathbf{a}(i) + s \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Modified input vector with \a n elements.
 @param[in]		s	Scalar.
*/
void vec_add(vektor *A, double s)
{
    double *aptr, *eptr;

    if (CheckVector(A) && ISNONZERO(s)) {
        aptr = A->v;
        eptr = aptr + A->len;
        for (; aptr < eptr; ) {
            *aptr++ += s;
        }
    }
}

/**
 Add scalar to a vector \f$ \mathbf{b}(i) = \mathbf{a}(i) + s \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	First vector with \a n elements.
 @param[in]		s	Scalar.
 @param[out]	B	Overwritten result vector with \a n added elements.
*/
void vec_add(vektor *A, double s, vektor *B)
{
    double *aptr, *eptr, *bptr;

    if (CheckSameVector(A, B)) {
        if (ISZERO(s)) {
            vec_copy(A, B);
        } else {
            aptr = A->v;
            eptr = aptr + A->len;
            bptr = B->v;
            for (; aptr < eptr; ) {
                *bptr++ = *aptr++ + s;
            }
        }
    }
}

/**
 Add scalar to a vector \f$ \mathbf{b}(i) = \mathbf{a}(i) + s \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	Vector with \a n elements.
 @param[in]		s	Scalar.
 @result			New allocated result vector \a B with \a n added elements or NULL.
*/
vektor *vec_add_new(vektor *A, double s)
{
    vektor *B = NULL;

    if (CheckVector(A)) {
        B = vec_alloc(A->len);
        vec_add (A, s, B);
    }
    return B;
}

/**
 Substract scalar from a vector \f$ \mathbf{a}(i) = \mathbf{a}(i) - s \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Modified input vector with \a n elements.
 @param[in]		s	Scalar.
*/
void vec_sub(vektor *A, double s)
{
    double *aptr, *eptr;

    if (CheckVector(A) && ISNONZERO(s)) {
        aptr = A->v;
        eptr = aptr + A->len;
        for (; aptr < eptr; ) {
            *aptr++ -= s;
        }
    }
}

/**
 Substract scalar from a vector \f$ \mathbf{b}(i) = \mathbf{a}(i) - s \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	First vector with \a n elements.
 @param[in]		s	Scalar.
 @param[out]	B	Overwritten result vector with \a n substracted elements.
*/
void vec_sub(vektor *A, double s, vektor *B)
{
    double *aptr, *eptr, *bptr;

    if (CheckSameVector(A, B)) {
        if (ISZERO(s)) {
            vec_copy(A, B);
        } else {
            aptr = A->v;
            eptr = aptr + A->len;
            bptr = B->v;
            for (; aptr < eptr; ) {
                *bptr++ = *aptr++ - s;
            }
        }
    }
}

/**
 Substract scalar from a vector \f$ \mathbf{b}(i) = \mathbf{a}(i) - s \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	Vector with \a n elements.
 @param[in]		s	Scalar.
 @result		New allocated result vector \a B with \a n substracted elements or NULL.
*/
vektor *vec_sub_new(vektor *A, double s)
{
    vektor *B = NULL;

    if (CheckVector(A)) {
        B = vec_alloc(A->len);
        vec_sub(A, s, B);
    }
    return B;
}

/**
 Multiply vector with a scalar \f$ \mathbf{a}(i) = \mathbf{a}(i) \cdot s \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in,out] A	Modified input vector with \a n elements.
 @param[in]		s	Scalar.
*/
void vec_mult(vektor *A, double s)
{
    double *aptr, *eptr;

    if (CheckVector(A)) {
		if (ISZERO(s)) {
			vec_fill(A, 0.0);
		} else if (ISINEQUAL(s, 1.0)) {
			aptr = A->v;
			eptr = aptr + A->len;
			for (; aptr < eptr; ) {
				*aptr++ *= s;
			}
		}
	}
}

/**
 Multiply vector with a scalar \f$ \mathbf{b}(i) = \mathbf{a}(i) \cdot s \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	First vector with \a n elements.
 @param[in]		s	Scalar.
 @param[out]	B	Overwritten result vector with \a n multiplied elements.
*/
void vec_mult(vektor *A, double s, vektor *B)
{
    double *aptr, *eptr, *bptr;

    if (CheckSameVector(A, B)) {
		if (ISZERO(s)) {
			vec_fill(B, 0.0);
		} else if (ISEQUAL(s, 1.0)) {
			vec_copy(A, B);
		} else {
			aptr = A->v;
			eptr = aptr + A->len;
			bptr = B->v;
			for (; aptr < eptr; ) {
				*bptr++ = *aptr++ * s;
			}
		}
	}
}

/**
 Multiply vector with a scalar \f$ \mathbf{b}(i) = \mathbf{a}(i) \cdot s \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	Vector with \a n elements.
 @param[in]		s	Scalar.
 @result			New allocated result vector \a B with \a n multiplied elements or NULL.
*/
vektor *vec_mult_new(vektor *A, double s)
{
    vektor *B = NULL;

    if (CheckVector(A)) {
        B = vec_alloc(A->len);
        vec_mult(A, s, B);
    }
    return B;
}

/**
 Divide vector by a scalar \f$ \mathbf{a}(i) = \mathbf{a}(i) / s \f$ for \f$ i=0, \ldots ,n-1 \f$ and \f$ s \ne 0 \f$.

 @param[in,out] A	Modified input vector with \a n elements.
 @param[in]		s	Scalar.
 @result			True, if vector division was successful.
*/
bool vec_div(vektor *A, double s)
{
    double *aptr, *eptr, f;

    if (CheckVector(A)) {
		if (ISZERO(s)) {
			Warning ("vec_div: division by zero!"); return false;
		} else if (ISINEQUAL(s, 1.0)) {
			aptr = A->v;
			eptr = aptr + A->len;
			f = 1.0/s;
			for (; aptr < eptr; ) {
				*aptr++ *= f;
			}
		}
	}
    return true;
}

/**
 Divide vector by a scalar \f$ \mathbf{b}(i) = \mathbf{a}(i) / s \f$ for \f$ i=0, \ldots ,n-1 \f$ and \f$ s \ne 0 \f$.

 @param[in]		A	First vector with \a n elements.
 @param[in]		s	Scalar.
 @param[out]	B	Overwritten result vector with \a n elements.
 @result			True, if vector division was successful.
*/
bool vec_div(vektor *A, double s, vektor *B)
{
    double *aptr, *eptr, *bptr, f;

    if (CheckSameVector(A, B)) {
		if (ISZERO(s)) {
			Warning ("vec_div: division by zero!");	return false;
		} else if (ISEQUAL(s, 1.0)) {
			vec_copy(A, B);
		} else {
			aptr = A->v;
			eptr = aptr + A->len;
			bptr = B->v;
			f = 1.0/s;
			for (; aptr < eptr; ) {
				*bptr++ = *aptr++ * f;
			}
		}
	}
    return true;
}

/**
 Divide vector by a scalar \f$ \mathbf{b}(i) = \mathbf{a}(i) / s \f$ for \f$ i=0, \ldots ,n-1 \f$ and \f$ s \ne 0 \f$.

 @param[in]		A	Vector with \a n elements.
 @param[in]		s	Scalar.
 @result			New allocated result vector \a B with \a n divided elements or NULL.
*/
vektor *vec_div_new(vektor *A, double s)
{
    vektor *B = NULL;

    if (CheckVector(A)) {
        B = vec_alloc(A->len);
        if (!vec_div(A, s, B)) {
            B = vec_free(B);
        }
    }
    return B;
}
//@}

/** @name Arithmetic Element-by-element Operations */
//@{

/**
 Add corresponding elements of two vectors \f$ \mathbf{a}(i) = \mathbf{a}(i) + \mathbf{b}(i) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Modified first vector with \a n elements.
 @param[in]		B	Second vector with \a n elements.
*/
void vec_add(vektor *A, vektor *B)
{
    double *aptr, *eptr, *bptr;

    if (CheckSameVector(A, B)) {
        aptr = A->v;
        eptr = aptr + A->len;
        bptr = B->v;
        for (; aptr < eptr; ) {
            *aptr++ += *bptr++;
        }
    }
}

/**
 Add corresponding elements of two vectors \f$ \mathbf{c}(i) = \mathbf{a}(i) + \mathbf{b}(i) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	First vector with \a n elements.
 @param[in]		B	Second vector with \a n elements.
 @param[out]	C	Overwritten result vector with \a n added elements.
*/
void vec_add(vektor *A, vektor *B, vektor *C)
{
    double *aptr, *eptr, *bptr, *cptr;

    if (CheckSameVector(A, B) && CheckSameVector(A, C)) {
        aptr = A->v;
        eptr = aptr + A->len;
        bptr = B->v;
        cptr = C->v;
        for (; aptr < eptr; ) {
            *cptr++ = *aptr++ + *bptr++;
        }
    }
}

/**
 Add corresponding elements of two vectors \f$ \mathbf{c}(i) = \mathbf{a}(i) + \mathbf{b}(i) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	First vector with \a n elements.
 @param[in]		B	Second vector with \a n elements.
 @result			New allocated result vector \a C with \a n added elements or NULL.
*/
vektor *vec_add_new(vektor *A, vektor *B)
{
    vektor *C = NULL;

    if (CheckVector(A)) {
        C = vec_alloc(A->len);
        vec_add(A, B, C);
    }
    return C;
}

/**
 Substract corresponding elements of two vectors \f$ \mathbf{a}(i) = \mathbf{a}(i) - \mathbf{b}(i) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Modified first vector with \a n elements.
 @param[in]		B	Second vector with \a n elements.
*/
void vec_sub(vektor *A, vektor *B)
{
    double *aptr, *eptr, *bptr;

    if (CheckSameVector(A, B)) {
        aptr = A->v;
        eptr = aptr + A->len;
        bptr = B->v;
        for (; aptr < eptr; ) {
            *aptr++ -= *bptr++;
        }
    }
}

/**
 Substract corresponding elements of two vectors \f$ \mathbf{c}(i) = \mathbf{a}(i) - \mathbf{b}(i) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	First vector with \a n elements.
 @param[in]		B	Second vector with \a n elements.
 @param[out]	C	Overwritten result vector with \a n substracted elements.
*/
void vec_sub(vektor *A, vektor *B, vektor *C)
{
    double *aptr, *eptr, *bptr, *cptr;

    if (CheckSameVector(A, B) && CheckSameVector(A, C)) {
        aptr = A->v;
        eptr = aptr + A->len;
        bptr = B->v;
        cptr = C->v;
        for (; aptr < eptr; ) {
            *cptr++ = *aptr++ - *bptr++;
        }
    }
}

/**
 Substract corresponding elements of two vectors \f$ \mathbf{c}(i) = \mathbf{a}(i) - \mathbf{b}(i) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	First vector with \a n elements.
 @param[in]		B	Second vector with \a n elements.
 @result		New allocated result vector \a C with \a n substracted elements or NULL.
*/
vektor *vec_sub_new(vektor *A, vektor *B)
{
    vektor *C = NULL;

    if (CheckVector(A)) {
        C = vec_alloc(A->len);
        vec_sub (A, B, C);
    }
    return C;
}

/**
 Multiply corresponding elements of two vectors \f$ \mathbf{a}(i) = \mathbf{a}(i) \cdot \mathbf{b}(i) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Modified first vector with \a n elements.
 @param[in]		B	Second vector with \a n elements.
*/
void vec_mult(vektor *A, vektor *B)
{
    double *aptr, *eptr, *bptr;

    if (CheckSameVector(A, B)) {
        aptr = A->v;
        eptr = aptr + A->len;
        bptr = B->v;
        for (; aptr < eptr; ) {
            *aptr++ *= *bptr++;
        }
    }
}

/**
 Multiply corresponding elements of two vectors \f$ \mathbf{c}(i) = \mathbf{a}(i) \cdot \mathbf{b}(i) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	First vector with \a n elements.
 @param[in]		B	Second vector with \a n elements.
 @param[out]	C	Overwritten result vector with \a n multiplied elements.
*/
void vec_mult(vektor *A, vektor *B, vektor *C)
{
    double *aptr, *eptr, *bptr, *cptr;

    if (CheckSameVector(A, B) && CheckSameVector(A, C)) {
        aptr = A->v;
        eptr = aptr + A->len;
        bptr = B->v;
        cptr = C->v;
        for (; aptr < eptr; ) {
            *cptr++ = *aptr++ * *bptr++;
        }
    }
}

/**
 Multiply corresponding elements of two vectors \f$ \mathbf{c}(i) = \mathbf{a}(i) \cdot \mathbf{b}(i) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	First vector with \a n elements.
 @param[in]		B	Second vector with \a n elements.
 @result			New allocated result vector \a C with \a n multplied elements or NULL.
*/
vektor *vec_mult_new(vektor *A, vektor *B)
{
    vektor *C = NULL;

    if (CheckVector(A)) {
        C = vec_alloc(A->len);
        vec_mult (A, B, C);
    }
    return C;
}

/**
 Divide corresponding elements of two vectors \f$ \mathbf{a}(i) = \mathbf{a}(i) / \mathbf{b}(i) \f$ for \f$ i=0, \ldots ,n-1 \f$.\n
 If a denominator \f$ \mathbf{b}(i) \f$ is zero, the corresponding element is set to \f$ [-\infty, 0, \infty] \f$ depending on \f$ \mathbf{a}(i) \f$.

 @param[in,out]	A	Modified first vector with \a n elements.
 @param[in]		B	Second vector with \a n elements.
*/
void vec_div(vektor *A, vektor *B)
{
    double *aptr, *eptr, *bptr;

    if (CheckSameVector(A, B)) {
        aptr = A->v;
        eptr = aptr + A->len;
        bptr = B->v;
        for (; aptr < eptr; aptr++, bptr++) {
			*aptr = DIVIDE(*aptr, *bptr);
        }
    }
}

/**
 Divide corresponding elements of two vectors \f$ \mathbf{c}(i) = \mathbf{a}(i) / \mathbf{b}(i) \f$ for \f$ i=0, \ldots ,n-1 \f$.\n
 If a denominator \f$ \mathbf{b}(i) \f$ is zero, the corresponding element is set to \f$ [-\infty, 0, \infty] \f$ depending on \f$ \mathbf{a}(i) \f$.

 @param[in]		A	First vector with \a n elements.
 @param[in]		B	Second vector with \a n elements.
 @param[out]	C	Overwritten result vector with \a n divided elements.
*/
void vec_div(vektor *A, vektor *B, vektor *C)
{
    double *aptr, *eptr, *bptr, *cptr;

    if (CheckSameVector(A, B) && CheckSameVector(A, C)) {
        aptr = A->v;
        eptr = aptr + A->len;
        bptr = B->v;
        cptr = C->v;
        for (; aptr < eptr; aptr++, bptr++, cptr++) {
			*cptr = DIVIDE(*aptr, *bptr);
        }
    }
}

/**
 Divide corresponding elements of two vectors \f$ \mathbf{c}(i) = \mathbf{a}(i) / \mathbf{b}(i) \f$ for \f$ i=0, \ldots ,n-1 \f$.\n
 If a denominator \f$ \mathbf{b}(i) \f$ is zero, the corresponding element is set to \f$ [-\infty, 0, \infty] \f$ depending on \f$ \mathbf{a}(i) \f$.

 @param[in]		A	First vector with \a n elements.
 @param[in]		B	Second vector with \a n elements.
 @result			New allocated result vector \a C with \a n divided elements or NULL.
*/
vektor *vec_div_new(vektor *A, vektor *B)
{
    vektor *C = NULL;

    if (CheckVector(A)) {
        C = vec_alloc(A->len);
        vec_div(A, B, C);
    }
    return C;
}
//@}


/** @name Other Element-by-element Operations */
//@{

/**
 Compute the vector absolute values \f$ \mathbf{a}(i) = |\mathbf{a}(i)| \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in,out]	A	Modified input vector with \a n elements.
*/
void vec_abs(vektor *A)
{
    double *aptr, *eptr;

    if (CheckVector(A)) {
        aptr = A->v;
        eptr = aptr + A->len;
        for (; aptr < eptr; ) {
            *aptr++ = ABS(*aptr);
        }
    }
}

/**
 Compute the vector absolute values \f$ \mathbf{b}(i) = |\mathbf{a}(i)| \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	First vector with \a n elements.
 @param[out]	B	Overwritten result vector with \a n absolute values.
*/
void vec_abs(vektor *A, vektor *B)
{
    double *aptr, *eptr, *bptr;

    if (CheckSameVector(A, B)) {
        aptr = A->v;
        eptr = aptr + A->len;
        bptr = B->v;
        for (; aptr < eptr; aptr++) {
            *bptr++ = ABS(*aptr);
        }
    }
}

/**
 Compute the vector absolute values \f$ \mathbf{b}(i) = |\mathbf{a}(i)| \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	Vector with \a n elements.
 @result			New allocated result vector \a B with \a n absolute values or NULL.
*/
vektor *vec_abs_new(vektor *A)
{
    vektor *B = NULL;

    if (CheckVector(A)) {
        B = vec_alloc(A->len);
        vec_abs(A, B);
    }
    return B;
}

/**
 Square values of the vector elements \f$ \mathbf{a}(i) = \mathbf{a}(i)^2 \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in,out]		A	Modified input vector with \a n elements.
*/
void vec_sqr(vektor *A)
{
    vec_mult(A, A);
}

/**
 Square values of the vector elements \f$ \mathbf{b}(i) = \mathbf{a}(i)^2 \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	First vector with \a n elements.
 @param[out]	B	Overwritten result vector with \a n squared elements.
*/
void vec_sqr(vektor *A, vektor *B)
{
    vec_mult(A, A, B);
}

/**
 Square values of the vector elements \f$ \mathbf{b}(i) = \mathbf{a}(i)^2 \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	Vector with \a n elements.
 @result			New allocated result vector with \a n squared elements or NULL.
*/
vektor *vec_sqr_new(vektor *A)
{
    return vec_mult_new(A, A);
}

/**
 Square roots of the vector elements \f$ \mathbf{a}(i) = \sqrt{\mathbf{a}(i)} \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in,out]		A	Modified input vector with \a n elements.
*/
void vec_sqrt(vektor *A)
{
    double *aptr, *eptr;

    if (CheckVector(A)) {
        aptr = A->v;
        eptr = aptr + A->len;
        for (; aptr < eptr; ) {
            *aptr++ = sqrt(*aptr);
        }
    }
}

/**
 Square roots of the vector elements \f$ \mathbf{b}(i) = \sqrt{\mathbf{a}(i)} \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	First vector with \a n elements.
 @param[out]	B	Overwritten result vector with \a n elements.
*/
void vec_sqrt(vektor *A, vektor *B)
{
    double *aptr, *eptr, *bptr;

    if (CheckSameVector(A, B)) {
        aptr = A->v;
        eptr = aptr + A->len;
        bptr = B->v;
        for (; aptr < eptr; ) {
            *bptr++ = sqrt(*aptr++);
        }
    }
}

/**
 Square roots of the vector elements \f$ \mathbf{b}(i) = \sqrt{\mathbf{a}(i)} \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	Vector with \a n elements.
 @result			New allocated result vector with \a n elements or NULL.
*/
vektor *vec_sqrt_new(vektor *A)
{
    vektor *B = NULL;

    if (CheckVector (A)) {
        B = vec_alloc(A->len);
        vec_sqrt (A, B);
    }
    return B;
}
//@}


/** @name Geometric Vector Operations */
//@{

/**
 Included angle of two vectors \f$ \theta = \arccos \left( \frac {\mathbf{a} \cdot \mathbf{b}} {|\mathbf{a}| |\mathbf{b}|} \right) \f$.

 @param[in]		A	First vector with n elements.
 @param[in]		B	Second vector with n elements.
 @result			Included angle in radiant.
*/
double vec_angle(vektor *A, vektor *B)
{
    double d, s = 0.0;

    if (CheckSameVector(A,B)) {
        d = vec_len(A) * vec_len(B);
        if (ISNONZERO(d)) {
            s = acos(vec_prod(A,B) / d);
        }
    }
    return s;
}

/**
 Cross product of two vectors with two elements \f$ s = \mathbf{a} \times \mathbf{b} = a_0 \cdot b_1 - a_1 \cdot b_0 \f$.

 @param[in]		A	First vector with 2 elements.
 @param[in]		B	Second vector with 2 elements.
 @param[out]	s	Result of the vector cross product.
*/
void vec_cross(vektor *A, vektor *B, double *s)
{
    if (CheckSizeVector(A, 2) && CheckSizeVector(B, 2)) {
        *s = A->v[1] * B->v[2] - A->v[2] * B->v[1];
    } else {
        *s = 0.0;
    }
}

/**
 Cross product of two vectors with three elements \f$ \mathbf{a} = \mathbf{a} \times \mathbf{b} \f$.

 @param[in,out]	A	Modified input vector with 3 elements.
 @param[in]		B	Second vector with 3 elements.
*/
void vec_cross(vektor *A, vektor *B)
{
    double a, b, c;

    if (CheckSizeVector(A, 3) && CheckSizeVector(B, 3)) {
        a = A->v[1] * B->v[2] - A->v[2] * B->v[1];
        b = A->v[2] * B->v[0] - A->v[0] * B->v[2];
        c = A->v[0] * B->v[1] - A->v[1] * B->v[0];
        A->v[0] = a;
        A->v[1] = b;
        A->v[2] = c;
    }
}

/**
 Cross product of two vectors with three elements \f$ \mathbf{c} = \mathbf{a} \times \mathbf{b}\f$.

 @param[in]		A	First vector with 3 elements.
 @param[in]		B	Second vector with 3 elements.
 @param[out]	C	Overwritten result vector with 3 elements.
*/
void vec_cross(vektor *A, vektor *B, vektor *C)
{
    if (CheckSizeVector(A, 3) && CheckSizeVector(B, 3) && CheckSizeVector(C, 3)) {
        C->v[0] = A->v[1] * B->v[2] - A->v[2] * B->v[1];
        C->v[1] = A->v[2] * B->v[0] - A->v[0] * B->v[2];
        C->v[2] = A->v[0] * B->v[1] - A->v[1] * B->v[0];
    }
}

/**
 Cross product of two vectors with three elements \f$ \mathbf{c} = \mathbf{a} \times \mathbf{b}\f$.

 @param[in]		A	First vector with 3 elements.
 @param[in]		B	Second vector with 3 elements.
 @result			New allocated result vector \a C with 3 elements or NULL.
*/
vektor *vec_cross_new(vektor *A, vektor *B)
{
    vektor *C = NULL;

    if (CheckSizeVector(A, 3) && CheckSizeVector(B, 3)) {
        C = vec_alloc(3);
        if (C) {
            C->v[0] = A->v[1] * B->v[2] - A->v[2] * B->v[1];
            C->v[1] = A->v[2] * B->v[0] - A->v[0] * B->v[2];
            C->v[2] = A->v[0] * B->v[1] - A->v[1] * B->v[0];
        }
    }
    return C;
}

/**
 The Euclidean length or vector magnitude corresponds to the L2-norm \f$ \| \mathbf{a} \|_2 = \sqrt{\sum_{i=0}^{n-1} \mathbf{a}(i)^2 } \f$.

 @param[in]		A	Vector with \a n elements.
 @result			Vector length.
*/
double vec_len(vektor *A)
{
    double sum = 0.0, *aptr, *eptr;

    if (CheckVector(A)) {
        aptr = A->v;
        eptr = aptr + A->len;
        for (; aptr < eptr; aptr++) {
            sum += SQR(*aptr);
        }
    }
    return sqrt(sum);
}

/**
 The vector p-norm \f$ \| \mathbf{a} \|_p = \left( \sum_{i=0}^{n-1} | \mathbf{a}(i) |^p \right) ^{\frac 1p} \f$.

 @param[in]		A	Vector with \a n elements.
 @param[in]		p	Type L1NORM (\a p=1), L2NORM (\a p=2), LINFNORM (\f$ p=\infty \f$).
 @result			Vector norm.
*/
double vec_norm (vektor *A, norm p)
{
	switch (p) {
		case L1NORM:    return vec_sumabs(A);
		case L2NORM:    return vec_len(A);
		case LINFNORM:  return vec_maxabs(A);
		case EUCLID:    if (A) return (A->v[A->len-1]);
		case MAXNORM:   return vec_max(A);
		case MINNORM:   return vec_min(A);
		case MEANNORM:  return vec_mean(A);
		default:		Warning ("vec_norm: unknown norm!");
	}
	return 0.0;
}

/**
 The standard vector norm corresponds to the L2-norm \f$ \| \mathbf{a} \|_2 = \sqrt{\sum_{i=0}^{n-1} \mathbf{a}(i)^2 } \f$.

 @param[in]		A	Vector with \a n elements.
 @result			Vector norm.
*/
double vec_norm (vektor *A)
{
	return vec_len(A);
}

/**
 Normalize vector properties to 1.

 @param[in,out]		A	Modified input vector with \a n elements.
 @param[in]			p	Use L1-, L2-, LInf-norm, max-, min-, mean value, Euclidean \f$ A(n-1) = 1 \f$ or adaptive normalization \f$ A(i) = \frac{(A(i)-min)}{(max-min)} \f$
 @result				True, if vector normalization was successful.
*/
bool vec_normalize(vektor *A, norm p)
{
    double min, max, *aptr, *eptr, len;

	if (p == ADAPTNORM) {
		vec_minmax (A, &min, &max);
		if (max > min) {
			aptr = A->v;
            eptr = aptr + A->len;
			for (; aptr<eptr; ) {
				*aptr++ = (*aptr - min) / (max - min);
			}
		}
	} else {
		len = vec_norm(A, p);
		if (ISNONZERO(len)) {
			vec_scale(A, 1.0/len);
			return true;
		}
	}
    return false;
}

/**
 Normalize vector length to 1 \f$ \mathbf{a} = \mathbf{a} / |\mathbf{a}| \f$.

 @param[in,out]		A	Modified input vector with \a n elements.
 @result				True, if vector normalization was successful.
*/
bool vec_normalize(vektor *A)
{
	return vec_normalize(A, L2NORM);
}

/**
 Normalize vector property to 1.

 @param[in]		A	First vector with \a n elements.
 @param[in]		p	Use L1-, L2-, LInf-norm, max-, min-, mean value, Euclidean \f$ A(n-1) = 1 \f$ or adaptive normalization \f$ A(i) = \frac{(A(i)-min)}{(max-min)} \f$
 @param[out]	B	Overwritten result vector with \a n normalized elements.
 @result			True, if vector normalization was successful.
*/
bool vec_normalize(vektor *A, norm p, vektor *B)
{
    double min, max, *aptr, *eptr, len;

	if (p == ADAPTNORM) {
		vec_minmax (A, &min, &max);
		if (max > min) {
			aptr = A->v;
            eptr = aptr + A->len;
			for (; aptr<eptr; ) {
				*aptr++ = (*aptr - min) / (max - min);
			}
		}
	} else {
		len = vec_norm(A, p);
		if (ISNONZERO(len)) {
			vec_scale(A, 1.0/len, B);
			return true;
		}
	}
    return false;
}

/**
 Normalize vector length to 1 \f$ \mathbf{b} = \mathbf{a} / |\mathbf{a}| \f$.

 @param[in]		A	First vector with \a n elements.
 @param[out]	B	Overwritten result vector with \a n normalized elements.
 @result			True, if vector normalization was successful.
*/
bool vec_normalize(vektor *A, vektor *B)
{
	return vec_normalize(A, L2NORM, B);
}

/**
 Normalize new vector property to 1.

 @param[in]		A	Vector with \a n elements.
 @param[in]		p	Use L1-, L2-, LInf-norm, max-, min-, mean value, Euclidean \f$ A(n-1) = 1 \f$ or adaptive normalization \f$ A(i) = \frac{(A(i)-min)}{(max-min)} \f$
 @result			New allocated result vector \a B with \a n normalized elements or NULL.
*/
vektor *vec_normalize_new(vektor *A, norm p)
{
    vektor *B = NULL;

    if (CheckVector(A)) {
        B = vec_alloc(A->len);
        if (!vec_normalize(A, p, B)) {
            B = vec_free(B);
        }
    }
    return B;
}

/**
 Normalize new vector length to 1 \f$ \mathbf{b} = \mathbf{a} / |\mathbf{a}| \f$.

 @param[in]		A	Vector with \a n elements.
 @result			New allocated result vector \a B with \a n normalized elements or NULL.
*/
vektor *vec_normalize_new(vektor *A)
{
	return vec_normalize_new(A, L2NORM);
}

/**
 Scalar/inner/dot product \f$ s = \sum_{i=0}^{n-1} \mathbf{a}(i) \cdot \mathbf{b}(i) \f$.

 @param[in]		A	First vector with \a n elements.
 @param[in]		B	Second vector with \a n elements.
 @result			Scalar product.
*/
double vec_prod(vektor *A, vektor *B)
{
    double sum = 0.0, *aptr, *eptr, *bptr;

    if (CheckSameVector(A, B)) {
        aptr = A->v;
        eptr = aptr + A->len;
        bptr = B->v;
        for (; aptr < eptr; ) {
            sum += *aptr++ * *bptr++;
        }
    }
    return sum;
}

/**
 Vector scaling \f$ \mathbf{a}(i) = \mathbf{a}(i) \cdot s \f$ for \f$ i=0, \ldots ,n-1 \f$ and \f$ s \ne 0 \f$.

 @param[in,out] A	Modified input vector with \a n elements.
 @param[in]		s	Scaling factor.
*/
void vec_scale(vektor *A, double s)
{
    if (ISZERO(s)) {
        Warning ("vec_scale: scaling factor is zero!"); return;
    }
    vec_mult(A, s);
}

/**
 Vector scaling \f$ \mathbf{b}(i) = \mathbf{a}(i) \cdot s \f$ for \f$ i=0, \ldots ,n-1 \f$ and \f$ s \ne 0 \f$.

 @param[in]		A	First vector with \a n elements.
 @param[in]		s	Scaling factor.
 @param[out]	B	Overwritten result vector with \a n scaled elements.
*/
void vec_scale(vektor *A, double s, vektor *B)
{
    if (ISZERO(s)) {
        Warning ("vec_scale: scaling factor is zero!"); return;
    }
    vec_mult(A, s, B);
}

/**
 Vector scaling \f$ \mathbf{b}(i) = \mathbf{a}(i) \cdot s \f$ for \f$ i=0, \ldots ,n-1 \f$ and \f$ s \ne 0 \f$.

 @param[in]		A	Vector with \a n elements.
 @param[in]		s	Scaling factor.
 @result			New allocated result vector \a B with \a n scaled elements or NULL.
*/
vektor *vec_scale_new(vektor *A, double s)
{
    if (ISZERO(s)) {
        Warning ("vec_scale_new: scaling factor is zero!"); return NULL;
    }
    return vec_mult_new(A, s);
}
//@}


/** @name Vector Operations with Matrices */
//@{

/**
 Matrix diagonal elements \f$ \mathbf{b}(i) = \mathbf{A}(i,i) \f$ for \f$ i=0, \ldots ,\min(m,n)-1 \f$.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @result			New allocated result vector \a B with \f$ \min(m,n) \f$ diagonal elements or NULL.
*/
vektor *vec_diag(matrix *A)
{
    vektor *B = NULL;
    double *bptr;
    long i;

    if (CheckMatrix(A)) {
        B = vec_alloc(MIN(A->cols, A->rows));
        if (B) {
            bptr = B->v;
            for (i=0; i<B->len; ++i) {
                *bptr++ = A->m[i][i];
            }
        }
    }
    return B;
}

/**
 Dyadic/outer vector product \f$ \mathbf{C}(i,j) = \mathbf{a}(i) \otimes \mathbf{b}(j) \f$ for \f$ i=0, \ldots ,m-1 \f$ and \f$ j=0, \ldots ,n-1 \f$.

 @param[in]		A	First vector with \a m elements.
 @param[in]		B	Second vector with \a n elements.
 @result			New allocated result matrix \a C with \f$ m \times n \f$ elements or NULL.

 @note				Identical with mat_dyadic() function.
*/
matrix *vec_dyadic(vektor *A, vektor *B)
{
    return mat_dyadic(A, B);
}

/**
 Apply transformation to a vector \f$ \mathbf{b}(j) = \mathbf{A}(j,i) \cdot \mathbf{b}(i) \f$ for \f$ i,j=0, \ldots ,n-1 \f$.

 @param[in]		A	Matrix with \f$ n \times n \f$ elements.
 @param[in,out]	B	Modified input vector with \a n elements.
*/
void vec_transform(matrix *A, vektor *B)
{
    vektor *C;

    if (CheckCompatible(B, A)) {
        C = vec_transform_new (A, B);
        vec_swap (C, B);
        C = vec_free(C);
    }
}

/**
 Apply transformation to a vector \f$ \mathbf{c}(j) = \mathbf{A}(j,i) \cdot \mathbf{b}(i) \f$ for \f$ i=0, \ldots ,n-1 \f$ and \f$ j=0, \ldots ,m-1 \f$ elements.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @param[in]		B	First vector with \a n elements.
 @param[out]	C	Overwritten result vector with \a m transformed elements.
*/
void vec_transform(matrix *A, vektor *B, vektor *C)
{
    double sum, *aptr, *eptr, *bptr, *fptr, *cptr;

    if (CheckCompatible (A, B) && CheckCompatible(C, A)) {
        aptr = A->m[0];
        eptr = aptr + (A->cols * A->rows);
        fptr = B->v + B->len;
        cptr = C->v;
        for (; aptr < eptr; ) {
            sum = 0.0;
            for (bptr = B->v; bptr < fptr; ) {
                sum += *aptr++ * *bptr++;
            }
            *cptr++ = sum;
        }
    }
}

/**
 Apply transformation to a vector \f$ \mathbf{c}(j) = \mathbf{A}(j,i) \cdot \mathbf{b}(i) \f$ for \f$ i=0, \ldots ,n-1 \f$ and \f$ j=0, \ldots ,m-1 \f$ elements.

 @param[in]		A	Matrix with \f$ m \times n \f$ elements.
 @param[in]		B	Vector with \a n elements.
 @result		New allocated result vector \a C with \a m transformed elements or NULL.
*/
vektor *vec_transform_new(matrix *A, vektor *B)
{
    vektor *C = NULL;

    if (CheckMatrix(A)) {
        C = vec_alloc(A->rows);
        vec_transform(A, B, C);
    }
    return C;
}
//@}

/** @name Vector Statistics */
//@{

/**
 Maximum vector element \f$ s = \max( \mathbf{a}(i) ) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	Vector with \a n elements.
 @result			Maximum element of the vector.
*/
double vec_max(vektor *A)
{
    double max = -MAX_REAL, *aptr, *eptr;

    if (CheckVector(A)) {
        aptr = A->v;
        eptr = aptr + A->len;
        for (; aptr < eptr; ++aptr) {
            max = MAX(*aptr, max);
        }
    }
    return max;
}

/**
 Vector absolute maximum corresponds to the Linfty-norm \f$ \| \mathbf{a} \|_\infty = \max( | \mathbf{a}(i) | ) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	Vector with \a n elements.
 @result			Maximum absolute value of the vector.
*/
double vec_maxabs(vektor *A)
{
    double max = 0.0, *aptr, *eptr;

    if (CheckVector(A)) {
        aptr = A->v;
        eptr = aptr + A->len;
        for (; aptr < eptr; ++aptr) {
            max = MAX(ABS(*aptr), max);
        }
    }
    return max;
}

/**
 Mean value of the vector elements \f$ \overline{\mathbf{a}} = \frac 1n \sum_{i=0}^{n-1} \mathbf{a}(i) \f$.

 @param[in]		A	Vector with \a n elements.
 @result			Mean value of the vector elements.
*/
double vec_mean(vektor *A)
{
    if (CheckVector (A)) {
        return (vec_sum(A) / A->len);
    }
    return 0.0;
}

/**
 Mean of the vector absolute values \f$ \overline{\mathbf{a}} = \frac 1n \sum_{i=0}^{n-1} | \mathbf{a}(i) | \f$.

 @param[in]		A	Vector with \a n elements.
 @result			Mean value of the vector elements.
*/
double vec_meanabs (vektor *A)
{
    if (CheckVector(A)) {
        return (vec_sumabs(A) / A->len);
    }
    return 0.0;
}

/**
 The median is the middle element of the sorted vector \f$ s = \mathbf{a}(0) \le \ldots \le \mathbf{a}(i) \le \ldots \le \mathbf{a}(n-1) \f$ for \f$ i=n/2 \f$.

 @param[in]		A	Vector with \a n elements.
 @result			Median element of the vector.
*/
double vec_median(vektor *A)
{
    double res = 0.0;
    vektor *B;

    B = vec_sort_new(A);
    if (CheckVector(B)) {
        res = B->v[B->len / 2];
        B = vec_free(B);
    }
    return res;
}

/**
 The median is the middle element of the sorted vector \f$ s = \mathbf{a}(0) \le \ldots \le \mathbf{a}(i) \le \ldots \le \mathbf{a}(num-1) \f$ for \f$ i=num/2 \f$.

 @param[in]		A	Vector with \a n elements.
 @param[in]		num	Number of relevant elements.
 @result			Median element of the vector.
*/
double vec_median(vektor *A, int num)
{
    double res = 0.0;
    vektor *B;

	B = vec_clone(A, 0, num-1);
	if (CheckVector(B)) {
		vec_sort(B);
		res = B->v[num / 2];
		B = vec_free(B);
	}
    return res;
}

/**
 The median is the middle element of the sorted vector \f$ s = | \mathbf{a}(0) | \le \ldots \le | \mathbf{a}(i) | \le \ldots \le | \mathbf{a}(n-1) | \f$ for \f$ i=n/2 \f$.

 @param[in]		A	Vector with \a n elements.
 @result			Median element of the absolute vector.
*/
double vec_medianabs(vektor *A)
{
    double res = 0.0;
    vektor *B;

	B = vec_abs_new(A);
    if (CheckVector(B)) {
		vec_sort(B);
        res = B->v[B->len / 2];
        B = vec_free(B);
    }
    return res;
}

/**
 The median is the middle element of the sorted vector \f$ s = | \mathbf{a}(0) | \le \ldots \le | \mathbf{a}(i) | \le \ldots \le | \mathbf{a}(num-1) | \f$ for \f$ i=num/2 \f$.

 @param[in]		A	Vector with \a n elements.
 @param[in]		num	Number of relevant elements.
 @result			Median element of the absolute vector.
*/
double vec_medianabs(vektor *A, int num)
{
    double res = 0.0;
    vektor *B;

	B = vec_clone(A, 0, num-1);
	if (CheckVector(B)) {
		vec_abs(B);
		vec_sort(B);
		res = B->v[num / 2];
		B = vec_free(B);
	}
    return res;
}

/**
 Minimum vector element \f$ s = \min( \mathbf{a}(i) ) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	Vector with \a n elements.
 @result			Minimum element of the vector.
*/
double vec_min(vektor *A)
{
    double min = MAX_REAL, *aptr, *eptr;

    if (CheckVector(A)) {
        aptr = A->v;
        eptr = aptr + A->len;
        for (; aptr < eptr; ++aptr) {
            min = MIN(*aptr, min);
        }
    }
    return min;
}

/**
 Minimum vector absolute value \f$ s = \min( | \mathbf{a}(i) | ) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	Vector with \a n elements.
 @result			Minimum absolute value of the vector.
*/
double vec_minabs(vektor *A)
{
    double min = MAX_REAL, *aptr, *eptr;

    if (CheckVector(A)) {
        aptr = A->v;
        eptr = aptr + A->len;
        for (; aptr < eptr; ++aptr) {
            min = MIN(ABS(*aptr), min);
        }
    }
    return min;
}

/**
 Maximum and minimum vector elements.

 @param[in]		A	Vector with \a n elements.
 @param[out]	min Minimum element of the vector.
 @param[out]	max Maximum element of the vector.
*/
void vec_minmax(vektor *A, double *min, double *max)
{
    double *aptr, *eptr;

    *min =  MAX_REAL;
    *max = -MAX_REAL;

    if (CheckVector(A)) {
        aptr = A->v;
        eptr = aptr + A->len;
        for (; aptr < eptr; ++aptr) {
            *min = MIN(*aptr, *min);
            *max = MAX(*aptr, *max);
        }
    }
}

/**
 Maximum and minimum vector absolute values.

 @param[in]		A	Vector with \a n elements.
 @param[out]	min Minimum absolute value of the vector.
 @param[out]	max Maximum absolute value of the vector.
*/
void vec_minmaxabs(vektor *A, double *min, double *max)
{
    double *aptr, *eptr;

    *min = MAX_REAL;
    *max = 0.0;

    if (CheckVector(A)) {
        aptr = A->v;
        eptr = aptr + A->len;
        for (; aptr < eptr; ++aptr) {
            *min = MIN(ABS(*aptr), *min);
            *max = MAX(ABS(*aptr), *max);
        }
    }
}

/**
 Biased standard deviation of the vector elements \f$ \sigma_n = \sqrt{\frac 1n \sum_{i=0}^{n-1} \left(\mathbf{a}(i) - \overline{\mathbf{a}} \right)^2} \f$ and mean value \f$ \overline{\mathbf{a}} = \frac 1n \sum_{i=0}^{n-1} \mathbf{a}(i) \f$ <a href="http://www.uni-giessen.de/tomas.sauer/Ankuend/HaFNum.pdf">(Fast version)</a>.

 @param[in]	   A	Vector with \a n elements.
 @param[out]   mean	Mean value of the elements in \a A.
 @result			Standard deviation of the elements in \a A.
*/
double vec_std(vektor *A, double *mean)
{
    return sqrt(vec_var(A, mean));
}

/**
 Biased standard deviation of the vector elements \f$ \sigma_n = \sqrt{\frac 1n \sum_{i=0}^{n-1} \left(\mathbf{a}(i) - \overline{\mathbf{a}} \right)^2} \f$ for the mean value \f$ \overline{\mathbf{a}} \f$.

 @param[in]		A	Vector with \a n elements.
 @result			Standard deviation of the elements in \a A.
*/
double vec_std(vektor *A)
{
    return sqrt(vec_var(A));
}

/**
 Unbiased standard deviation of the vector elements \f$ \sigma_n = \sqrt{\frac 1{n-1} \sum_{i=0}^{n-1} \left(\mathbf{a}(i) - \overline{\mathbf{a}} \right)^2} \f$ and mean value \f$ \overline{\mathbf{a}} = \frac 1n \sum_{i=0}^{n-1} \mathbf{a}(i) \f$ <a href="http://www.uni-giessen.de/tomas.sauer/Ankuend/HaFNum.pdf">(Fast version)</a>.

 @param[in]	   A	Vector with \f$ n \le 2 \f$ elements.
 @param[out]   mean	Mean value of the elements in \a A.
 @result			Standard deviation of the elements in \a A.
*/
double vec_std2(vektor *A, double *mean)
{
    return sqrt(vec_var2(A, mean));
}

/**
 Unbiased standard deviation of the vector elements \f$ \sigma_n = \sqrt{\frac 1{n-1} \sum_{i=0}^{n-1} \left(\mathbf{a}(i) - \overline{\mathbf{a}} \right)^2} \f$ for the mean value \f$ \overline{\mathbf{a}} \f$.


 @param[in]		A	Vector with \f$ n \le 2 \f$ elements.
 @result			Standard deviation of the elements in \a A.
*/
double vec_std2(vektor *A)
{
    return sqrt(vec_var2(A));
}

/**
 Sum all vector elements \f$ s = \sum_{i=0}^{n-1} \mathbf{a}(i) \f$.

 @param[in]		A	Vector with \a n elements.
 @result		Sum of all elements in \a A.
*/
double vec_sum(vektor *A)
{
    double *aptr, *eptr, sum = 0.0;

    if (CheckVector (A)) {
        aptr = A->v;
        eptr = aptr + A->len;
        for (; aptr < eptr; ) {
            sum += *aptr++;
        }
    }
    return sum;
}

/**
 Vector absolute sum corresponds to the L1-norm \f$ \| \mathbf{a} \|_1 = \sum_{i=0}^{n-1} | \mathbf{a}(i) | \f$.

 @param[in]		A	Vector with \a n elements.
 @result		Sum of all absolute values in \a A.
*/
double vec_sumabs(vektor *A)
{
    double *aptr, *eptr, sum = 0.0;

    if (CheckVector(A)) {
        aptr = A->v;
        eptr = aptr + A->len;
        for (; aptr < eptr; ++aptr) {
            sum += ABS(*aptr);
        }
    }
    return sum;
}

/**
 Biased variance of the vector elements \f$ \sigma_n^2 = \frac 1n \sum_{i=0}^{n-1} \left(\mathbf{a}(i) - \overline{\mathbf{a}} \right)^ 2 \f$ and mean value \f$ \overline{\mathbf{a}} = \frac 1n \sum_{i=0}^{n-1} \mathbf{a}(i) \f$ <a href="http://www.uni-giessen.de/tomas.sauer/Ankuend/HaFNum.pdf">(Fast version)</a>.

 @param[in]		A	Vector with \a n elements.
 @param[out]	mean	Mean value of the elements in \a A.
 @result		Variance of the elements in \a A.
*/
double vec_var(vektor *A, double *mean)
{
    double var = 0.0, diff, *aptr;
    long i;

    *mean = 0.0;
    if (CheckVector(A)) {
        for (aptr = A->v, i=1; i<=A->len; ++i, ++aptr) {
            diff = *aptr - *mean;
            var += (i-1) * SQR(diff) / i;     // Mixed int/double calculation ...
            *mean += diff / i;
        }
        var /= A->len;
    }
    return var;
}

/**
 Biased variance of the vector elements \f$ \sigma_n^2 = \frac 1n \sum_{i=0}^{n-1} \left(\mathbf{a}(i) - \overline{\mathbf{a}} \right)^2 \f$ with mean value \f$ \overline{\mathbf{a}} \f$.

 @param[in]		A	Vector with \a n elements.
 @result		Variance of the elements in \a A.
*/
double vec_var(vektor *A)
{
    double var = 0.0, diff, mean, *aptr;
    long i;

    mean = 0.0;
    if (CheckVector(A)) {
        for (aptr = A->v, i=1; i<=A->len; ++i, ++aptr) {
            diff = *aptr - mean;
            var += (i-1) * SQR(diff) / i;     // Mixed int/double calculation ...
            mean += diff / i;
        }
        var /= A->len;
    }
    return var;
}

/**
 Unbiased variance of the vector elements \f$ \sigma_n^2 = \frac 1{n-1} \sum_{i=0}^{n-1} \left(\mathbf{a}(i) - \overline{\mathbf{a}} \right)^ 2 \f$ and mean value \f$ \overline{\mathbf{a}} = \frac 1n \sum_{i=0}^{n-1} \mathbf{a}(i) \f$ <a href="http://www.uni-giessen.de/tomas.sauer/Ankuend/HaFNum.pdf">(Fast version)</a>.

 @param[in]		A	Vector with \f$ n \le 2 \f$ elements.
 @param[out]	mean	Mean value of the elements in \a A.
 @result		Variance of the elements in \a A.
*/
double vec_var2(vektor *A, double *mean)
{
    double var = 0.0, diff, *aptr;
    long i;

    *mean = 0.0;
    if (CheckVector(A)) {
        if (A->len < 2) {
            Warning("vec_var2: size %d too small", A->len);
        } else {
            for (aptr = A->v, i=1; i<=A->len; ++i, ++aptr) {
                diff = *aptr - *mean;
                var += (i-1) * SQR(diff) / i;     // Mixed int/double calculation ...
                *mean += diff / i;
            }
            var /= A->len - 1;
        }
    }
    return var;
}

/**
 Unbiased variance of the vector elements \f$ \sigma_n^2 = \frac 1{n-1} \sum_{i=0}^{n-1} \left(\mathbf{a}(i) - \overline{\mathbf{a}} \right)^2 \f$ with mean value \f$ \overline{\mathbf{a}} \f$.

 @param[in]		A	Vector with \f$ n \le 2 \f$ elements.
 @result		Variance of the elements in \a A.
*/
double vec_var2(vektor *A)
{
    double var = 0.0, diff, mean, *aptr;
    long i;

    mean = 0.0;
    if (CheckVector(A)) {
        if (A->len < 2) {
            Warning("vec_var2: size %d too small", A->len);
        } else {
            for (aptr = A->v, i=1; i<=A->len; ++i, ++aptr) {
                diff = *aptr - mean;
                var += (i-1) * SQR(diff) / i;     // Mixed int/double calculation ...
                mean += diff / i;
            }
            var /= A->len - 1;
        }
    }
    return var;
}
//@}


/** @name Vector Reorganisation */
//@{

/**
 Merge two vectors \f$ \mathbf{c} = \mathbf{a}(0) \ldots \mathbf{a}(m-1), \mathbf{b}(0) \ldots \mathbf{b}(n-1) \f$.

 @param[in]		A	First vector with \a m elements.
 @param[in]		B	Second vector with \a n elements.
 @result			New allocated vector \a C with \a m + \a n merged elements or NULL.
*/
vektor *vec_merge(vektor *A, vektor *B)
{
    vektor *C;

    if (!CheckVector(A)) {
        return vec_clone(B);
    }
    if (!CheckVector(B)) {
        return vec_clone(A);
    }
    C = vec_alloc(A->len + B->len);
    vec_copy(A, C);
    vec_copy(B, C, A->len);

    return C;
}

/**
 Merge three vectors \f$ \mathbf{d} = \mathbf{a}(0) \ldots \mathbf{a}(m-1), \mathbf{b}(0) \ldots \mathbf{b}(n-1), \mathbf{c}(0) \ldots \mathbf{c}(k-1) \f$.

 @param[in]		A	First vector with \a m elements.
 @param[in]		B	Second vector with \a n elements.
 @param[in]		C	Third vector with \a k elements.
 @result			New allocated vector \a D with \a m + \a n + \a k merged elements or NULL.
*/
vektor *vec_merge(vektor *A, vektor *B, vektor *C)
{
    vektor *D;

    if (!CheckVector(A)) {
        return vec_merge(B, C);
    }
    if (!CheckVector(B)) {
        return vec_merge(A, C);
    }
    if (!CheckVector(C)) {
        return vec_merge(A, B);
    }
    D = vec_alloc(A->len + B->len + C->len);
    vec_copy(A, D);
    vec_copy(B, D, A->len);
    vec_copy(C, D, A->len + B->len);

    return D;
}

/**
 Merge \a num vectors.

 @param[in]		A	Array of vectors.
 @param[in]		num	number of vectors.
 @result			New allocated vector with merged elements or NULL.
*/
vektor *vec_merge(vektor *A[], int num)
{
    vektor *B = NULL;
	int i, c=0, len = 0;

	if (A) {
		for (i=0; i<num; ++i) {
			if (A[i]) {
				len += A[i]->len;
			}
		}
		if (len < 1) {
			return NULL;
		}
		B = vec_alloc(len);
		if (B) {
			for (i=0; i<num; ++i) {
				if (A[i]) {
					vec_copy(A[i], B, c);
					c += A[i]->len;
				}
			}
		}
	}
    return B;
}

/**
 Reverse order of vector elements \f$ \mathbf{a}(i) = \mathbf{a}(n-i-1) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in,out]		A	Modified input vector with \a n elements.
*/
void vec_reverse(vektor *A)
{
    vektor *B;

    B = vec_reverse_new(A);
    vec_swap(B, A);
    B = vec_free(B);
}

/**
 Reverse order of vector elements \f$ \mathbf{b}(i) = \mathbf{a}(n-i-1) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	First vector with \a n elements.
 @param[out]	B	Overwritten result vector with \a n elements.
*/
void vec_reverse(vektor *A, vektor *B)
{
    double *aptr, *bptr, *eptr;

    if (CheckSameVector(A, B)) {
        aptr = A->v;
        eptr = aptr + A->len - 1;
        bptr = B->v;
        for (; eptr >= aptr; ) {
            *bptr++ = *eptr--;
        }
    }
}

/**
 Reverse order of vector elements \f$ \mathbf{b}(i) = \mathbf{a}(n-i-1) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	Vector with \a n elements.
 @result			New allocated result vector \a B with \a n elements in reversed order or NULL.
*/
vektor *vec_reverse_new(vektor *A)
{
    vektor *B = NULL;

    if (CheckVector(A)) {
        B = vec_alloc(A->len);
        vec_reverse (A, B);
    }
    return B;
}

/**
 Shuffle the position of vector elements randomly.

 @param[in,out]	A	Modified input vector with \a n mixed elements.
*/
void vec_shuffle(vektor *A)
{
    double *aptr;
    int len, i, j;

    if (CheckVector (A)) {
        len = A->len;
        aptr = A->v;
        for (i=0; i <len; ++i, ++aptr) {
            j = (int)Random((double)len);
            if (i != j) {
                SWAP(double, *aptr, A->v[j]);
            }
        }
    }
}

/**
 Shuffle the position of vector elements randomly.

 @param[in,out]	A	Modified input vector with \a n mixed elements.
*/
vektor *vec_shuffle_new(vektor *A)
{
    vektor *B;

    B = vec_clone(A);
    vec_shuffle(B);

    return B;
}

/**
 Sort vector elements into ascending order \f$ \mathbf{a} = \mathbf{a}(0) \le \ldots \le \mathbf{a}(n-1) \f$.

 @param[in,out]	A	Modified input vector with \a n elements.
*/
void vec_sort(vektor *A)
{
    if (CheckVector (A)) {
        Sortdouble(A->v, A->len);
    }
}

/**
 Sort vector elements into ascending order \f$ \mathbf{b} = \mathbf{a}(0) \le \ldots \le \mathbf{a}(n-1) \f$.

 @param[in]		A	Vector with \a n elements.
 @result			New allocated result vector \a B with \a n sorted elements or NULL.
*/
vektor *vec_sort_new(vektor *A)
{
    vektor *B;

    B = vec_clone(A);
    vec_sort(B);

    return B;
}

/**
 Split a vector in two vectors \f$ \mathbf{a} = \mathbf{a}(0) \ldots \mathbf{a}(i-1),  \f$ and \f$ \mathbf{b} = \mathbf{a}(i) \ldots \mathbf{a}(n-1) \f$.

 @param[in,out]	A	Modified first vector with \a n elements.
 @param[in]		i	Split position.
 @param[out]	B	Overwritten second vector with \a n-i elements.
*/
void vec_split(vektor *A, int i, vektor *B)
{
    int len;

    if (CheckIndex(A, i)) {
        len = A->len;
        if (CheckSizeVector(B, len-i)) {
            vec_copy(A, i, len-1, B);
            vec_realloc(A, i);
        }
    }
}

/**
 Split a vector in two vectors \f$ \mathbf{b} = \mathbf{a}(0) \ldots \mathbf{a}(i-1),  \f$ and \f$ \mathbf{c} = \mathbf{a}(i) \ldots \mathbf{a}(n-1) \f$.

 @param[in]		A	Vector with \a n elements.
 @param[in]		i	Split position.
 @param[out]	B	Overwritten first vector with \a i elements.
 @param[out]	C	Overwritten second vector with \a n-i elements.
*/
void vec_split (vektor *A, int i, vektor *B, vektor *C)
{
    int len;

    if (CheckIndex(A, i)) {
        len = A->len;
        if (CheckSizeVector(B, i) && CheckSizeVector(C, len-i)) {
            vec_copy(A, 0, i-1, B);
            vec_copy(A, i, len-1, C);
        }
    }
}

/**
 Split a vector in two vectors \f$ \mathbf{b} = \mathbf{a}(0) \ldots \mathbf{a}(i-1),  \f$ and \f$ \mathbf{c} = \mathbf{a}(i) \ldots \mathbf{a}(n-1) \f$.

 @param[in]		A	Vector with \a n elements.
 @param[in]		i	Split position.
 @param[out]	B	New allocated vector part with first \a i elements.
 @param[out]	C	New allocated vector part with last \a n-i elements.
*/
void vec_split_new(vektor *A, int i, vektor **B, vektor **C)
{
    if (B && C) {
        if (CheckIndex(A, i)) {
            *B = vec_clone(A, 0, i-1);
            *C = vec_clone(A, i, A->len-1);
        } else {
            *B = *C = NULL;
        }
    }
}

/**
 Swap the contents of two different vectors.

 @param[in,out]		A	Modified first vector with \a m elements.
 @param[in,out]		B	Modified second vector with \a n elements.
*/
void vec_swap(vektor *A, vektor *B)
{
    if (CheckVector(A) && CheckVector(B)) {
        SWAP(int, A->len, B->len);
        SWAP(double *, A->v, B->v);
    }
}
//@}


/** @name Logical Vector Operations */
//@{

/**
 Test two vectors for pair-wise equality \f$ \mathbf{a}(i) = \mathbf{b}(i) \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	First vector with \a n elements.
 @param[in]		B	Second vector with \a n elements.
 @result			True, if all vector elements are equal.
*/
bool vec_equal(vektor *A, vektor *B)
{
    double *aptr, *bptr, *eptr;

    if (!CheckSameVector(A, B)) {
        return false;
    }
    aptr = A->v;
    eptr = aptr + A->len;
    bptr = B->v;
    for (; aptr < eptr; ++aptr, ++bptr) {
        if (ISINEQUAL(*aptr, *bptr)) {
            return false;
        }
    }
    return true;
}

/**
 Test vector for infinite values \f$ \mathbf{a}(i) \ne \infty \f$ for \f$ i=0, \ldots ,n-1 \f$.

 @param[in]		A	vector with \a n elements.
 @result			True, if all vector elements are finite.
*/
bool vec_finite(vektor *A)
{
    double *aptr, *eptr;

    if (!CheckVector(A)) {
        return false;
    }
    aptr = A->v;
    eptr = aptr + A->len;
    for (; aptr < eptr; ++aptr) {
        if (ISINFINITE(*aptr)) {
            return false;
        }
    }
    return true;
}

/**
 Test two vectors for orthogonality \f$ \mathbf{a} \cdot \mathbf{b} = 0 \f$.

 @param[in]		A	First vector with \a n elements.
 @param[in]		B	Second vector with \a n elements.
 @result			True, if all vector elements are equal.
*/
bool vec_orthogonal(vektor *A, vektor *B)
{
    if (ISZERO(vec_prod(A,B))) {
        return true;
    }
    return false;
}
//@}


/** @name Other Operations */
//@{

/**
 Print vector on screen

 @param[in]		A		Vector with \a n elements.
*/
void vec_print(vektor *A)
{
    int i;

    if (report) {
        if (CheckVector (A)) {
//          Debug ("Vector length = %d elements", A->len);
            for (i=0; i<A->len; ++i) {
                printf ("| %13.5e |\n", ISNONZERO(A->v[i]) ? A->v[i] : 0.0);
            }
        } else {
            Debug("Vector empty");
        }
    }
}

/**
 Print vector with label on screen

 @param[in]		label	String with the vector name.
 @param[in]		A		Vector with \a n elements.
*/
void vec_print(char *label, vektor *A)
{
    Report ("%s", label);
	vec_print(A);
}


//UTIL

#include "MersenneTwister.h" //good random number generator
MTRand randomNumberGenerator;  //one instance auto seeded

#ifdef _DEBUG

#include "StackWalker.h"

StackWalker sw;

bool memListInitialized=false;

memList mem[1023];

void InitMem(){
    memset(mem, 0, 1023*sizeof(memList));
}

#endif

// ===========================================================================
//
// ===========================================================================
// CalcTime removed, it's not used anyway.
//#ifndef linux
//double CalcTime (struct __timeb64 *start, struct __timeb64 *stop)
//{
//	return ((double)((stop->time*1000+stop->millitm)-(start->time*1000+start->millitm)))*1e-3;
//}
//#endif
// ===========================================================================
//
// ===========================================================================

void InitRandom (void)
{
    //this is done automatically during startup

    //old calls:
    //randomNumberGenerator.seed();
    //srand((uint)time(NULL));
}

// ===========================================================================
//
// ===========================================================================

double Random (double x)
{
    return randomNumberGenerator.rand()*x;

    //unsigned int i;
    //if(x>RANDOM_MAX){
    //    i=(rand()<<15) | rand(); //30 bit random
    //    x= (x * (double)i / (double)RANDOM_MAX30);
    //}
    //else{
    //    i= rand(); //15 bit random
    //    x= (x * (double)i / (double)RANDOM_MAX);
    //}
    //return x;
}

// ===========================================================================
//
// ===========================================================================

void SetROI (int x, int y, int w, int h, region *roi)
{
	if (roi) {
		roi->x = x;
		roi->y = y;
		roi->w = w;
		roi->h = h;
	}
}

// ===========================================================================
//
// ===========================================================================

void MatrixCheckROI (region *roi, matrix *dat)
{
	if (roi && dat) {
		roi->x = MIN (MAX (0, roi->x), dat->cols-1);
		roi->y = MIN (MAX (0, roi->y), dat->rows-1);
		roi->w = MIN (MAX (0, roi->w), dat->cols - roi->x);
		roi->h = MIN (MAX (0, roi->h), dat->rows - roi->y);
	}
}

// ===========================================================================
//
// ===========================================================================

void ImageCheckROI (region *roi, image *pic)
{
	if (roi && pic) {
		roi->x = MIN (MAX (0, roi->x), pic->w-1);
		roi->y = MIN (MAX (0, roi->y), pic->h-1);
		roi->w = MIN (MAX (0, roi->w), pic->w - roi->x);
		roi->h = MIN (MAX (0, roi->h), pic->h - roi->y);
	}
}

// ===========================================================================
//
// ===========================================================================
// ===========================================================================
//
// ===========================================================================

void
Rot2Angle (matrix *R, double *x, double *y, double *z)
{
    if (ISZERO(R->m[0][0]) && ISZERO(R->m[1][0])) {
        *x = 0.0;
        *y = PI * 0.5;
        *z = atan2 (R->m[0][1], R->m[1][1]);
    } else {
        *x = atan2 (R->m[1][0], R->m[0][0]);
        *y = atan2 (-R->m[2][0], sqrt (SQR(R->m[0][0]) + SQR(R->m[1][0])));
        *z = atan2 (R->m[2][1], R->m[2][2]);
    }
}

void
Rot2Angle_XYZ (matrix *R, double *x, double *y, double *z)
{


    double sin_phi=R->m[0][2]; *y=asin(sin_phi);
  //double tan_om;
	if (!ISZERO(R->m[2][2]))
		{
		//tan_om=-R->m[1][2]/R->m[2][2];
		//*x=atan(tan_om);
		*x=atan2(-R->m[1][2],R->m[2][2]);
		}
	//else {printf("ERROR: can not determine omega"); exit(0);}
	else
		{printf("Attention: unsolved ambiguity for omega (tan = inf)\n");
		//*x=PI/2;
		*x=atan2(-R->m[1][2],R->m[2][2]);
		}

	//double tan_kap;
	if (!ISZERO(R->m[0][0]))
		{//tan_kap=-R->m[0][1]/R->m[0][0];
		//*z=atan(tan_kap);
		*z=atan2(-R->m[0][1],R->m[0][0]);
		}
	else
		{printf("Attention: unsolved ambiguity for kappa (tan = inf)\n");
		//*z=PI/2;
		*z=atan2(-R->m[0][1],R->m[0][0]);
		}
	//else {printf("ERROR: can not determine kappa"); exit(0);}



}

// ===========================================================================
//
// ===========================================================================

matrix *mat_rotx(double a)
{
	matrix *R;
	double c, s;

    s = sin(a); c = cos(a);
    if(ABS(s)<REAL_EPS) //below machine precision
        s=0;
    if(ABS(s)>1.0-REAL_EPS) //below machine precision
        s=s>0?1.0:-1.0;
    if(ABS(c)<REAL_EPS) //below machine precision
        c=0;
    if(ABS(c)>1.0-REAL_EPS) //below machine precision
        c=c>0?1.0:-1.0;

	R = mat_alloc(3,3);
    R->m[0][0] =  1; R->m[0][1] =  0; R->m[0][2] =  0;
	R->m[1][0] =  0; R->m[1][1] =  c; R->m[1][2] = -s;
    R->m[2][0] =  0; R->m[2][1] =  s; R->m[2][2] =  c;

	return R;
}

matrix *mat_roty(double a)
{
	matrix *R;
	double c, s;

    s = sin(a); c = cos(a);
    if(ABS(s)<REAL_EPS) //below machine precision
        s=0;
    if(ABS(s)>1.0-REAL_EPS) //below machine precision
        s=s>0?1.0:-1.0;
    if(ABS(c)<REAL_EPS) //below machine precision
        c=0;
    if(ABS(c)>1.0-REAL_EPS) //below machine precision
        c=c>0?1.0:-1.0;

	R = mat_alloc(3,3);
    R->m[0][0] =  c; R->m[0][1] =  0; R->m[0][2] =  s;
	R->m[1][0] =  0; R->m[1][1] =  1; R->m[1][2] =  0;
    R->m[2][0] = -s; R->m[2][1] =  0; R->m[2][2] =  c;

	return R;
}

matrix *mat_rotz(double a)
{
	matrix *R;
	double c, s;

    s = sin(a); c = cos(a);
    if(ABS(s)<REAL_EPS) //below machine precision
        s=0;
    if(ABS(s)>1.0-REAL_EPS) //below machine precision
        s=s>0?1.0:-1.0;
    if(ABS(c)<REAL_EPS) //below machine precision
        c=0;
    if(ABS(c)>1.0-REAL_EPS) //below machine precision
        c=c>0?1.0:-1.0;

	R = mat_alloc(3,3);
    R->m[0][0] =  c; R->m[0][1] = -s; R->m[0][2] =  0;
	R->m[1][0] =  s; R->m[1][1] =  c; R->m[1][2] =  0;
    R->m[2][0] =  0; R->m[2][1] =  0; R->m[2][2] =  1;

	return R;
}

void
Angle2Rot (double x, double y, double z, matrix *R)
{
	matrix *R1, *R2, *R3, *T, *T2;
/*
    matrix *R0;

	// Special iWitness rotation
	R0 = mat_rotx(DEG2RAD(90.0));
	R1 = mat_roty(x);
	R2 = mat_rotx(-y);
	R3 = mat_rotz(z);

	T = mat_prod(R1, R0);
	T2 = mat_prod(R2, T); T = mat_free(T);
	T = mat_prod(R3, T2); T2 = mat_free(T2);
	mat_copy (T, R); T = mat_free(T);

	R0 = mat_free(R0);
	R1 = mat_free(R1);
	R2 = mat_free(R2);
	R3 = mat_free(R3);
*/
    // XYZ Euler angles
	R1 = mat_rotx(x);
	R2 = mat_roty(y);
	R3 = mat_rotz(z);

	T = mat_prod_new(R2, R1);
	T2 = mat_prod_new(R3, T); T = mat_free(T);
	mat_copy (T2, R); T2 = mat_free(T2);

	R1 = mat_free(R1);
	R2 = mat_free(R2);
	R3 = mat_free(R3);
/*
    double sx, cx, sy, cy, sz, cz;

    sx = sin(x); cx = cos(x);
	sy = sin(y); cy = cos(y);
	sz = sin(z); cz = cos(z);

//  // XYZ Euler angles
    R->m[0][0] =  cz*cy;
    R->m[0][1] = -sz*cx+cz*sy*sx;
    R->m[0][2] =  sz*sx+cz*sy*cx;
    R->m[1][0] =  sz*cy;
    R->m[1][1] =  cz*cx+sz*sy*sx;
    R->m[1][2] = -cz*sx+sz*sy*cx;
    R->m[2][0] = -sy;
    R->m[2][1] =  cy*sx;
    R->m[2][2] =  cy*cx;
*/
}

void
Angle2Rot_XYZ (double x, double y, double z, matrix *R)
{
	matrix *R1, *R2, *R3, *T, *T2;

    // XYZ Euler angles
	R1 = mat_rotx(x);
	R2 = mat_roty(y);
	R3 = mat_rotz(z);

	T = mat_prod_new(R1, R2);
	T2 = mat_prod_new(T, R3); T = mat_free(T);
	mat_copy (T2, R); T2 = mat_free(T2);

	R1 = mat_free(R1);
	R2 = mat_free(R2);
	R3 = mat_free(R3);

}



char *NumberFilename (char *filename, int num)
{
    char buf[BUF_SIZE], *ext, *base;

    base = strdup (filename);
    if ((ext = strrchr (base, '.')) != NULL) {
        *ext++ = '\0';
        sprintf (buf, "%s%04d.%s", base, num, ext);
    } else {
        sprintf (buf, "%s%04d", base, num);
    }
	hfree (base);

    return strdup (buf);
}

char *FilenameExtension (char *filename, char *end)
{
    char buf[BUF_SIZE], *ext, *base;

    base = strdup (filename);
	if (base) {
		if ((ext = strrchr (base, '.')) != NULL) {
			*ext = '\0';
		}
		sprintf (buf, "%s.%s", base, end);
		hfree (base);
	}
    return strdup (buf);
}

char *BaseFilename (char *filename)
{
    char buf[BUF_SIZE], *ext, *base;

    base = strdup (filename);
	if (base) {
		if ((ext = strrchr (base, '\\')) != NULL) { // delete filename directories
			sprintf (buf, "%s", ext+1);
			hfree (base);
			return strdup (buf);
		}
	}
	return base;
}

char *BaseFilenameExtension (char *filename, char *end)
{
	char *base, *ext = NULL;

	base = BaseFilename (filename);
	if (base) {
		ext = FilenameExtension (base, end);
		hfree(base);
	}
	return ext;
}

// ===========================================================================
//
// ===========================================================================

void *halloc (uint num, uint size)
{
    void *ptr;

	if ((num < 1) || (size < 1)) {
		Warning ("halloc: size too small!"); return NULL;
	}
	if (num > MAX_UINT/size) {
		Warning ("halloc: size too large!"); return NULL;  // prevent overflow
	}
    ptr = (void *) calloc (num, size);
    if (!ptr) {
		Warning ("halloc: out of memory!"); return NULL;
    }
#ifdef _DEBUG
    if(!memListInitialized){
        memListInitialized=true;
        InitMem();
    }
    if(recordHalloc){
        sw.ShowCallstack();
        memElem* el;
        int bin= ((unsigned int)(ptr))%1023;
        el = (memElem*)calloc(1, sizeof(memElem));
        el->fileName =callerName;
        callerName=NULL;
        el->pointer=ptr;
        el->size=num*size;

        if(!mem[bin].end && !mem[bin].start){
            mem[bin].start=el;
            mem[bin].end=el;
        }
        else{
            if(mem[bin].start)
                mem[bin].start->prev=el;
            el->next=mem[bin].start;
            mem[bin].start=el;
        }
        mem[bin].num++;
    }
#endif
    return ptr;
}

// ===========================================================================
//
// ===========================================================================

void *hrealloc (void *ptr, uint num, uint size)
{
	void *res;

	if ((num < 1) || (size < 1)) {
		Warning ("hrealloc: size too small!"); return NULL;
	}
	if (num > MAX_UINT/size) {
		Warning ("hrealloc: size too large!"); return NULL;  // prevent overflow
	}
    res = (void *) realloc (ptr, num*size);
    if (!res) {
		Warning ("hrealloc: out of memory!"); return NULL;
    }
#ifdef _DEBUG
    if(!memListInitialized){
        memListInitialized=true;
        InitMem();
    }
    if(recordHalloc && res!=ptr){
        memElem* el;
        int bin= ((unsigned int)(ptr))%1023;
        //delete old pointer
        for(el=mem[bin].start;el; el=el->next){
            if(el->pointer==ptr){
                if(el->prev){
                    el->prev->next=el->next;
                }
                else if(mem[bin].start==el){
                    mem[bin].start=el->next;
                }
                if(el->next){
                    el->next->prev=el->prev;
                }
                else if(mem[bin].end==el){
                    mem[bin].end=el->prev;
                }
                mem[bin].num--;
                free (el->fileName);
                free (el);
                break;
            }
        }

        //store new pointer
        sw.ShowCallstack();
        bin= ((unsigned int)(res))%1023;
        el = (memElem*)calloc(1, sizeof(memElem));
        el->fileName =callerName;
        callerName=NULL;
        el->pointer=res;
        el->size=num*size;
        if(!mem[bin].end && !mem[bin].start){
            mem[bin].start=el;
            mem[bin].end=el;
        }
        else{
            if(mem[bin].start)
                mem[bin].start->prev=el;
            el->next=mem[bin].start;
            mem[bin].start=el;
        }
        mem[bin].num++;
    }
#endif
    return res;
}

#ifdef _DEBUG
void debug_free(void *ptr){
    if(!memListInitialized){
        memListInitialized=true;
        InitMem();
    }
    memElem *el;
    int bin = ((unsigned int)(ptr))%1023;
    if(recordHalloc){
        for(el=mem[bin].start;el; el=el->next){
            if(el->pointer==ptr){
                if(el->prev){
                    el->prev->next=el->next;
                }
                else if(mem[bin].start==el){
                    mem[bin].start=el->next;
                }

                if(el->next){
                    el->next->prev=el->prev;
                }

                else if(mem[bin].end==el){
                    mem[bin].end=el->prev;
                }
                mem[bin].num--;
                free (el->fileName);
                free (el);
                break;
            }
        }
    }
    free(ptr);
}

void    PrintMemStack(){
    int size=0, i, numOfLeaks;
    memElem *el;

    numOfLeaks=0;
    for(i=0; i<1023;++i){
        numOfLeaks+=mem[i].num;
    }
    if(numOfLeaks>0){
        printf("Number of memory leaks: %d\n", numOfLeaks);
        for(i=0; i<1023;++i){
            for(el=mem[i].start;el; el=el->next){
                printf("%4d:\n%s",i,el->fileName);
                printf("\n");
                size+=el->size;
            }
        }
        printf("Total size in KB: %f\n", (double)size/1024.0);
    }
}
#endif

// ===========================================================================
//
// ===========================================================================

void
ErrorExit (int code, char *text)
{
    if (code == USAGE_ERROR) {
        exit (0);
    }
    switch (code) {
        case PBM_ERROR:
             Error ("Datei '%s' nicht im Portable BitMap P5/6 Format.", text);
             break;
        case FLOAT_ERROR:
             Error ("'%s' ist keine Datei im Float Format.", text);
             break;
        case MEM_ERROR :
             Error ("nicht genuegend Speicher.");
             break;
        case OPEN_ERROR:
             Error ("kann '%s' nicht oeffnen.", text);
             break;
        case OPTION_ERROR:
             Error ("korrekte Option lautet '%s'.", text);
             break;
        case PARAM_ERROR:
             Error ("Parameter '%s' wird nicht unterstuetzt.", text);
             break;
        case READ_ERROR:
             Error ("Lesefehler in '%s'.", text);
             break;
        case WRITE_ERROR:
             Error ("Schreibfehler in '%s'.", text);
             break;
        default:
			 Error ("%s (Code %d).", text, code);
    }
    exit (code);
}
// ===========================================================================
// extracts the file name in oldFilename, replace it with newFilename and returns a new string
// ===========================================================================

char *ExchangeFilename (char *oldFilename, char *newFilename)
{
    char buf[BUF_SIZE], *ext, *base, *oldE;

    base = strdup (oldFilename);
    oldE=base;

    do{
        ext = strrchr (oldE, '\\');
        if(ext)
            oldE= ext+1;
        else
            oldE=NULL;
    }while(ext!=NULL);

    if (oldE != NULL) {
        *oldE= '\0'; //cut off rest of base (the old filename)
        sprintf (buf, "%s%s", base, newFilename);
    } else {
        sprintf (buf, ".\\%s", newFilename);
    }
    return strdup (buf);
}
#ifndef linux
/*
bool createDirectory(char *directory){
        // create directory if neccessary
	if(_access( directory, 0 ) !=0){
	    if(_mkdir(directory)<0){
            Error("Error creating directory: %s\n", directory);
			return false;
		}
	}
    return true;
}
*/
#endif
static int
CompareNumberdouble (double *x1, double *x2)
{
    return SGN(*x1 - *x2); //if *x1 equals *x2 they can be swapped.
}

// ===========================================================================
// Sortdouble sorts the first num elements given in the array a with qsort
// ===========================================================================

void Sortdouble (double *a, int num)
{
	qsort((void *)a, (unsigned)num, sizeof(double), (int (*)(const void *, const void *)) CompareNumberdouble);
}

// ===========================================================================
// GetdoubleSorted returns a pointer to the queried element within MIN_REAL margin
// This makes sense to check if elementToFind is in the array
// if the element is not found, NULL is returned
// ===========================================================================

double* GetdoubleSorted (double *a, double elementToFind, int num)
{
	return (double*)bsearch(&elementToFind, (void *)a, (unsigned)num, sizeof(double), (int (*)(const void *, const void *)) CompareNumberdouble);
}

static int
CompareNumberInt (int *x1, int *x2)
{
    return *x1 - *x2;
}

// ===========================================================================
// SortInt sorts the first num elements given in the array a with qsort
// ===========================================================================

void SortInt (int *a, int num)
{
	qsort((void *)a, (unsigned)num, sizeof(int), (int (*)(const void *, const void *)) CompareNumberInt);
}

// ===========================================================================
// GetIntSorted returns a pointer to the queried element
// This makes sense to check if elementToFind is in the array
// if the element is not found, NULL is returned
// ===========================================================================

int* GetIntSorted (int *a, int elementToFind, int num)
{
	return (int*)bsearch(&elementToFind, (void *)a, (unsigned)num, sizeof(int), (int (*)(const void *, const void *)) CompareNumberInt);
}

// ===========================================================================
// GaussNoise generates gauss noise with given mean and sigma
// ===========================================================================

double GaussNoise(double mean, double variance){
    return randomNumberGenerator.randNorm(mean, variance);
    //double X=0;
    //int i;
    //
    //for (i=0; i<NUM_RAND_RUNS; ++i){
    //    X+=Random(1.0);
    //}

    //X = X - (double)NUM_RAND_RUNS/2.0;                // set mean to 0
    //X = X * sqrt(12.0 / (double)NUM_RAND_RUNS);        // adjust variance to 1


    //X= mean+sqrt(variance)* X; //set desited mean and variance

    //return X;
}

// ===========================================================================
// Triangulation of one spatial point 'X' seen in 'num' images at position 'h'
// ===========================================================================

bool Triangulation (matrix *P[], homo2 *h[], int num, homo3 *X)
{
    matrix *A;
    vektor *V;
    int i, j;
	bool res;

	if (!P || !h || (num < 2)) {
		Warning ("Triangluation: no data!"); return false;
	}
	for (i=0; i<num; ++i) {
		if (!CheckSizeMatrix(P[i], 3, 4)) return false;
		if (!h[i]) {
			Warning ("Triangulation: no data!"); return false;
		}
	}
	for (i=1; i<num; ++i) {
		if (!CheckID(h[0], h[i])) return false;
	}


	A = mat_alloc (2*num, 4);
	for (j=0; j<num; ++j) {
		NormHomo (h[j]);
		for (i=0; i<4; ++i) {
			A->m[2*j][i]   = h[j]->x * P[j]->m[2][i] - P[j]->m[0][i];
			A->m[2*j+1][i] = h[j]->y * P[j]->m[2][i] - P[j]->m[1][i];

		}
	}
	res = mat_svd_nullvector (A, &V); A = mat_free (A);

	if (res) {
		*X = VecToNormHomo3 (V);
		V = vec_free (V);
	} else {
		*X = NOHOMO3;
	}
	X->id = h[0]->id;

	return res;
}


bool Triangulation (matrix *P1, matrix *P2, homo2 *h1, homo2 *h2, homo3 *X)
{
	matrix *P[2];
	homo2 *h[2];

	P[0] = P1; P[1] = P2; h[0] = h1; h[1] = h2;
    return Triangulation (P, h, 2, X);
}

bool Triangulation (matrix *P1, matrix *P2, matrix *P3, homo2 *h1, homo2 *h2, homo2 *h3, homo3 *X)
{
	matrix *P[3];
	homo2 *h[3];

	P[0] = P1; P[1] = P2; P[2] = P3; h[0] = h1; h[1] = h2; h[2] = h3;
    return Triangulation (P, h, 3, X);
}

// ===========================================================================
// Triangulation of 'n' spatial points 'xl' seen in 'm' images at position 'h'
// ===========================================================================

xlist *Triangulation (matrix *P[], int m, homo2 *h[], int n)
{
    xlist *xl = NULL;
	homo3 X;
    homo2 **p;
	int i, j;

	if (!P || !h || (m < 2) || (n < 1)) {
		Warning ("Triangulation: no data!"); return NULL;
	}
	p = (homo2 **) halloc (m, sizeof(homo2 *));
	if (p) {
		for (i=0; i<n; ++i) {
			for (j=0; j<m; ++j) {
				if (h[j]) {
					p[j] = &h[j][i];
				}
			}
			if (Triangulation (P, p, m, &X)) {
				xl = AddCoord (xl, &X);
			}
		}
		free (p); p = NULL;
	}
    return xl;
}

xlist *Triangulation (matrix *P1, matrix *P2, homo2 *h1, homo2 *h2, int n)
{
	matrix *P[2];
	homo2 *h[2];

	P[0] = P1; P[1] = P2; h[0] = h1; h[1] = h2;
    return Triangulation (P, 2, h, n);;
}

xlist *Triangulation (matrix *P1, matrix *P2, matrix *P3, homo2 *h1, homo2 *h2, homo2 *h3, int n)
{
	matrix *P[3];
	homo2 *h[3];

	P[0] = P1; P[1] = P2; P[2] = P3; h[0] = h1; h[1] = h2; h[2] = h3;
    return Triangulation (P, 3, h, n);
}

// ===========================================================================
// Triangulation of one spatial point 'X' seen in 'num' images at position 'p'
// ===========================================================================

bool Triangulation (matrix *P[], pelem *p[], int num, homo3 *X)
{
    matrix *A;
    vektor *V;
    int i, j;
	bool res;

	if (!P || !p || (num < 2)) {
		Warning ("Triangluation: no data!"); return false;
	}
	for (i=0; i<num; ++i) {
		if (!CheckSizeMatrix(P[i], 3, 4)) return false;
		if (!p[i]) {
			Warning ("Triangulation: no data!"); return false;
		}
	}
	for (i=1; i<num; ++i) {
		if (!CheckID(p[0], p[i])) return false;
	}
	A = mat_alloc (2*num, 4);
	for (j=0; j<num; ++j) {
		for (i=0; i<4; ++i) {
			A->m[2*j][i]   = p[j]->x * P[j]->m[2][i] - P[j]->m[0][i];
			A->m[2*j+1][i] = p[j]->y * P[j]->m[2][i] - P[j]->m[1][i];
		}
	}
	res = mat_svd_nullvector (A, &V); A = mat_free (A);
	if (res) {
		*X = VecToNormHomo3 (V);
		V = vec_free (V);
	} else {
		*X = NOHOMO3;
	}
	X->id = p[0]->id;

	return res;
}

bool Triangulation (matrix *P1, matrix *P2, pelem *p1, pelem *p2, homo3 *X)
{
	matrix *P[2];
	pelem *p[2];

	P[0] = P1; P[1] = P2; p[0] = p1; p[1] = p2;
    return Triangulation (P, p, 2, X);
}

bool Triangulation (matrix *P1, matrix *P2, matrix *P3, pelem *p1, pelem *p2, pelem *p3, homo3 *X)
{
	matrix *P[3];
	pelem *p[3];

	P[0] = P1; P[1] = P2; P[2] = P3; p[0] = p1; p[1] = p2; p[2] = p3;
    return Triangulation (P, p, 3, X);
}

// ===========================================================================
// Triangulation of 'n' spatial points 'xl' seen in 'm' images at position 'pl'
// ===========================================================================

xlist *Triangulation (matrix *P[], int m, plist *pl[])
{
    xlist *xl = NULL;
	homo3 X;
    pelem **p;
    register int i, j;
    int color[3];

	if (!P || !pl || (m < 2)) {
		Warning ("Triangulation: no data!"); return NULL;
	}
	p = (pelem **) halloc (m, sizeof(pelem *));
	if (p) {
		for (i=0; i<m; ++i) p[i] = pl[i]->start;

		for (j=0; j < pl[0]->num; ++j) {
            //color mean
            color[0]=0;
            color[1]=0;
            color[2]=0;
            for (i=0; i<m; ++i){
                color[0]+=GET_R(p[i]->color);
                color[1]+=GET_G(p[i]->color);
                color[2]+=GET_B(p[i]->color);
            }
            color[0]=CLIP_GRAY(color[0]/m);
            color[1]=CLIP_GRAY(color[1]/m);
            color[2]=CLIP_GRAY(color[2]/m);
            color[0]=COLOR(color[0], color[1], color[2]);
			if (Triangulation (P, p, m, &X)) {
				xl = AddCoord (xl, &X, color[0]);
			}
			for (i=0; (i < m) && p[i]; ++i) p[i] = p[i]->next;
		}
		free (p); p = NULL;
	}
    return xl;
}

xlist *Triangulation (matrix *P1, matrix *P2, plist *pl1, plist *pl2)
{
    xlist *xl;
	matrix *P[2];
	plist *pl[2];

	P[0] = P1; P[1] = P2; pl[0] = pl1; pl[1] = pl2;
    xl = Triangulation (P, 2, pl);

    return xl;
}

xlist *Triangulation (matrix *P1, matrix *P2, matrix *P3, plist *pl1, plist *pl2, plist *pl3)
{
    xlist *xl;
	matrix *P[3];
	plist *pl[3];

	P[0] = P1; P[1] = P2; P[2] = P3; pl[0] = pl1; pl[1] = pl2; pl[2] = pl3;
    xl = Triangulation (P, 3, pl);

    return xl;
}

homo2 ClosestPoint (double a, double b, double c)
{
    homo2 h;

    h.x = -a * c;
    h.y = -b * c;
    h.w = SQR(a) + SQR(b);

    return h;
}

// ===========================================================================
//
// ===========================================================================
static void NormalizeEpipol (homo2 *e)
{
    double s;

    s = CheckZero(sqrt(SQR(e->x) + SQR(e->y)));

    e->x /= s;
    e->y /= s;
    e->w /= s;
}
// ===========================================================================
//
// ===========================================================================

static matrix *EpipolRotationMatrix (homo2 e)
{
    matrix *T;

    T = mat_alloc (3, 3);
    T->m[0][0] =  e.x; T->m[0][1] = e.y;
    T->m[1][0] = -e.y; T->m[1][1] = e.x;
    T->m[2][2] =  1.0;

    return T;
}
// ===========================================================================
//   s(t) = t^2/(1+f1^2t^2) + (ct+d)^2/((at+b)^2+f2^2(ct+d)^2)
// ===========================================================================

static double CostFunction (double f1, double f2, double a, double b, double c, double d,
						  double t)
{
    double h1, h2, h3, s;

    h1 = SQR(c*t + d);
    h2 = 1.0 + SQR(f1) * SQR(t);
    h3 = SQR(a*t + b) + SQR(f2) * h1;

    if (ISZERO(h2) || ISZERO(h3)) {
        s = MAX_REAL;
    } else {
        s = SQR(t) / h2 + h1 / h3;
    }
    return s;
}

// ===========================================================================
//
// ===========================================================================

static double EvalCostFunction (double f1, double f2, double a, double b, double c,
							  double d, vektor *t)
{
    double tmin = MAX_REAL, h1, h2;
    int i;

    CheckVector (t);

    h1 = SQR(f1);
    h2 = SQR(a) + SQR(f2)*SQR(c);

    if (ISNONZERO(h1) && ISNONZERO(h2)) {
        tmin = MIN(tmin, 1.0 / h1 + SQR(c) / h2);   // asymptotic value t->inf
    }
    for (i=0; i<t->len; ++i) {
        tmin = MIN(tmin, CostFunction (f1, f2, a, b, c, d, t->v[i]));
    }
    return tmin;
}

// ===========================================================================
//
// ===========================================================================

static double SolvePolynom (double f1, double f2, double a, double b, double c, double d)
{
    vektor *coef, *res;
    double tmin = MAX_REAL;
    double h, sa, sb, sc, sd, sf1, sf2;
    double ssa, ssb, ssc, ssd, ssf1, ssf2;

    h = b*c - a*d;

    sa  = SQR(a);  ssa  = SQR(sa);
    sb  = SQR(b);  ssb  = SQR(sb);
    sc  = SQR(c);  ssc  = SQR(sc);
    sd  = SQR(d);  ssd  = SQR(sd);
    sf1 = SQR(f1); ssf1 = SQR(sf1);
    sf2 = SQR(f2); ssf2 = SQR(sf2);

    coef = vec_alloc (7);
    coef->v[0] = b * d * h;
    coef->v[1] = ssb - sa * sd + ssd * ssf2 + sb * (sc + 2.0 * sd * sf2);
    coef->v[2] = -sa * c * d + a * b * (4.0 * sb + sc - 2.0 * sd * (sf1 - 2.0 * sf2)) +
                 2.0 * c * d * (2.0 * sd * ssf2 + sb * (sf1 + 2.0 * sf2));
    coef->v[3] = 2.0 * (4.0 * a * b * c * d * sf2 + sa * (3.0 * sb + sd * (-sf1 + sf2)) +
                 sc * (3.0 * sd * ssf2 + sb * (sf1 + sf2)));
    coef->v[4] = 2.0 * a * c * h * sf1 + b * d * h * ssf1 +
                 4.0 * (a * b * sc * sf2 + sa * c * d * sf2 + sc * c * d * ssf2 + sa * a * b);
    coef->v[5] = ssa + sb * sc * ssf1 + ssc * ssf2 + sa * (-sd * ssf1 + 2.0 * sc * sf2);
    coef->v[6] = a * c * h * ssf1;

    res = poly_root (coef);
    coef = vec_free (coef);

    if (res) {
        tmin = EvalCostFunction (f1, f2, a, b, c, d, res);
    }
    return tmin;
}


static void CorrectMatch (plist *pl1, plist *pl2, matrix *F, plist **sl1, plist **sl2)
{
    matrix *TI1, *TI2, *TT;
    matrix *R1, *R2, *RT1, *RT2;
    matrix *T, *F2;
    homo2 *h1, *h2, e1, e2;
    pelem *p, *q;
    trans t1, t2;
    double t;
    int i, num, num2;

    Debug ("Correct Match:");

    PointToHomo (pl1, &h1, &num);
    PointToHomo (pl2, &h2, &num2);
    if ((num < 1) || (num != num2)) {
        Error ("CorrectMatch: No data !");
    }
    for (i=0; i<num; ++i) {

        t1.tx = h1[i].x;
        t1.ty = h1[i].y;
        t1.sx = t1.sy = 1.0;
        TI1 = InvSimilarityMatrix2 (&t1);

        t2.tx = h2[i].x;
        t2.ty = h2[i].y;
        t2.sx = t2.sy = 1.0;
        TI2 = InvSimilarityMatrix2 (&t2);

        T = mat_prod_new (F, TI1);
        TT = mat_transpose_new (TI2);
        F2 = mat_prod_new (TT, T);                        // F2 = T2^-T * F * T^-1
        T = mat_free (T); TT = mat_free (TT);

        CalcEpipoles (F2, &e1, &e2);

        NormalizeEpipol (&e1);
        NormalizeEpipol (&e2);
        R1 = EpipolRotationMatrix (e1);
        R2 = EpipolRotationMatrix (e2);

        RT1 = mat_transpose_new (R1);
        R1 = mat_free (R1);

        RT2 = mat_transpose_new (R2);
        T = mat_prod_new (F2, RT1);
        F2 = mat_free (F2);

        F2 = mat_prod_new (R2, T);                           // F2 = R2 * F2 * R^T
        R2 = mat_free (R2);
        T = mat_free (T);

        t = SolvePolynom (e1.w, e2.w, F2->m[1][1], F2->m[1][2], F2->m[2][1], F2->m[2][2]);

        h1[i] = ClosestPoint (t * e1.w, 1.0, -t);
        h2[i] = ClosestPoint (-e2.w * (F2->m[2][1] * t + F2->m[2][2]),
                              F2->m[1][1] * t + F2->m[1][2], F2->m[2][1] * t + F2->m[2][2]);
        F2 = mat_free (F2);

        T = mat_prod_new (TI1, RT1);                               // T1^-1 * R1^T
        TI1 = mat_free (TI1);
        RT1 = mat_free (RT1);
        h1[i] = TransformNormHomo (T, &h1[i]);
        T = mat_free (T);

        T = mat_prod_new (TI2, RT2);                               // T2^-1 * R2^T
        TI2 = mat_free (TI2);
        RT2 = mat_free (RT2);
        h2[i] = TransformNormHomo (T, &h2[i]);
        T = mat_free (T);
    }
    *sl1 = *sl2 = NULL;
    for (i=0, p=pl1->start, q=pl2->start; i<num; i++, p=p->next, q=q->next) {
		if (CheckID(p,q)) {
			*sl1 = AddPoint (*sl1, h1[i].x, h1[i].y, p->id);
			*sl2 = AddPoint (*sl2, h2[i].x, h2[i].y, q->id);
		}
    }
	if (h1) {
		free (h1); h1 = NULL;
	}
	if (h2) {
		free (h2); h2 = NULL;
	}
}

// ===========================================================================
//
// ===========================================================================

// ===========================================================================
//
// ===========================================================================

// ===========================================================================
//
// ===========================================================================

xlist *OptimalTriangulation (plist *pl1, plist *pl2, matrix *F, matrix *P1, matrix *P2)
{
    plist *ml1, *ml2;
    xlist *xl;

    CorrectMatch (pl1, pl2, F, &ml1, &ml2);
    xl = Triangulation (P1, P2, ml1, ml2);

    return xl;
}

// ===========================================================================
//
// ===========================================================================

void AddCalibration (matrix *K, matrix *P)
{
	matrix *T;

	if (!CheckSizeMatrix(P, 3, 4) || !CheckSizeMatrix(K, 3, 3)) return;

	T = mat_prod_new(K, P); mat_copy(T, P);	T = mat_free(T);
}

// ===========================================================================
//
// ===========================================================================

matrix *AddCalibrationNew (matrix *K, matrix *P)
{
	matrix *Q;

	Q = mat_clone(P);
	AddCalibration (K, Q);

	return Q;
}


// ===========================================================================
//
// ===========================================================================

matrix *RecomposeCamera(matrix *K, matrix *R, homo3 *C)
{
	matrix *KR, *P;
	homo2 d;

	KR = mat_prod_new(K, R);
	NormHomo (C);
    d.x = C->X;
    d.y = C->Y;
    d.w = C->Z;
	d = TransformHomo(KR, &d);
    P = mat_alloc (3, 4);
    mat_copy (KR, P);
	KR = mat_free (KR);

	P->m[0][3]=-d.x;
	P->m[1][3]=-d.y;
	P->m[2][3]=-d.w;

	NormCamera(P);

	return P;
}

// ===========================================================================
//
// ===========================================================================

double GetCameraBase(matrix *P)
{
	homo3 C;

    C = ProjectionCenter(P);
	NormHomo(&C);

	return sqrt(SQR(C.X) + SQR(C.Y) + SQR(C.Z));
}

// ===========================================================================
//
// ===========================================================================

void ScaleCameraBase(matrix *P, double b)
{
	matrix *K, *R, *T;
	homo3 C;
	double len;

	DecomposeCamera (P, &K, &R);
    C = ProjectionCenter (P);
	NormHomo(&C);
	len = sqrt (SQR(C.X) + SQR(C.Y) + SQR(C.Z));
	if (ISNONZERO(len)) {
		ScaleHomo(&C, b/len);
	}
	T = RecomposeCamera(K, R, &C);
	mat_copy(T, P);
	T = mat_free (T);
}

// ===========================================================================
//   P' = P H^-1
// ===========================================================================

matrix *TransformCameraNew (matrix *P, matrix *H)
{
    matrix *T, *HI;

    HI = mat_invert_new (H);
    T = mat_prod_new (P, HI);
    HI = mat_free (HI);

    NormCamera (T);

    return T;
}

// ===========================================================================
//
// ===========================================================================

void TransformCamera (matrix *P, matrix *H)
{
    matrix *T;

	T = TransformCameraNew (P, H);
	mat_copy (T, P);
	T = mat_free(T);
}

// ===========================================================================
//   P -> [I|0]  mit  H = [    P   ]
//                        [ 0 0 0 1]
// ===========================================================================

matrix *CanonicCameraBase (matrix *P)
{
    matrix *H;

    H = mat_alloc (4,4);
    mat_copy (P, H);
    H->m[3][3] = 1.0;

    return H;
}

// ===========================================================================
//   Pi' = Bi Pi H^-1  fuer  i = 1,..,num
// ===========================================================================

void TransformCameraBase (matrix *P[], matrix *B[], int num)
{
    matrix *T, *H = NULL;
    int i;

    if (num) {
        for (i=0; i<num; ++i) {
            T = mat_prod_new (B[i], P[i]);
            if (!i) {
                H = CanonicCameraBase (T);
            }
            P[i]=mat_free(P[i]); //here was a memory leak
            P[i] = TransformCameraNew (T, H);
            T = mat_free (T);
        }
        H = mat_free (H);
    }
}

// ===========================================================================
//                |1 0 0 0|
//   P1 = [I|0] = |0 1 0 0|
//                |0 0 1 0|
// ===========================================================================

matrix *NormalizedCameraMatrix (void)
{
	matrix *P, *T;

    P = mat_alloc (3, 4);
    T = mat_identity (3);
    mat_copy (T, P);
	T = mat_free (T);

	return P;
}

// ===========================================================================
//
//   P1 = [K|0]
//
// ===========================================================================

matrix *CalibratedCameraMatrix (matrix *K)
{
	matrix *P;

    P = mat_alloc (3, 4);
    mat_copy(K, P);

	return P;
}


// ===========================================================================
//                |1 0 0 0|
//   P1 = [I|0] = |0 1 0 0|
//                |0 0 1 0|
//
//   P2 = [[e2]x F + e2 a^T | b e2]  mit  a = 0, b = 1
// ===========================================================================

void ProjectiveCameraMatrix (matrix *F, homo2 *e, matrix **P1, matrix **P2)
{
    matrix *T;

    *P1 = NormalizedCameraMatrix ();
    *P2 = mat_alloc (3, 4);
    T = homo_mat_cross (e, F);
    mat_copy (T, *P2); T = mat_free (T);

    (*P2)->m[0][3] = e->x;
    (*P2)->m[1][3] = e->y;
    (*P2)->m[2][3] = e->w;

    NormCamera (*P1);
    NormCamera (*P2);
}

// ===========================================================================
//
// ===========================================================================

double ProjectSymError (matrix *P1, matrix *P2, plist *pl1, plist *pl2, xlist *xl)
{
    pelem *q, *s;
    xelem *p;
    double r, res = 0.0;

    Debug ("Symmetric Reproject Error:");

    InitError ();
    for (p=xl->start, q=pl1->start, s=pl2->start; p && q && s; p=p->next, q=q->next, s=s->next) {
		if (CheckID(q,s) && CheckID(q,p)) {
			r = CalcSymReproject (q, s, P1, P2, p);
			res += r;
			AccumulateError (r);
		}
    }
    PrintError ();

    return res;
}

// ===========================================================================
//
// ===========================================================================

double ProjectError (matrix *P, plist *pl, xlist *xl)
{
    pelem *q;
    xelem *p;
    double r, res = 0.0;

    Debug ("Project Error: image distance [pix]");

    InitError ();
    for (p=xl->start, q=pl->start; p && q; p=p->next, q=q->next) {
		if (CheckID(q,p)) {
			r = CalcReproject (q, P, p);
			res += r;
			AccumulateError (r);
		}
    }
    PrintError ();

    return res;
}

// ===========================================================================
//
// ===========================================================================

static double BundleError (matrix *P[], int m, homo2 *x[], homo3 *X, int n)
{
    double r, res = 0.0;
    int i, j;

    Debug ("Bundle Error:");

    InitError ();
    for (i=0; i<m; ++i) {
        for (j=0; j<n; ++j) {
            r = CalcReproject (&x[i][j], P[i], &X[j]);
            res += r;
            AccumulateError (r);
        }
    }
    PrintError ();

    return res;
}

// ===========================================================================
//
// ===========================================================================

void EvalBundleError (plist *pl[], matrix *P[], int num, xlist *xl)
{
    homo2 **x;
    homo3 *X;
    int i, n, m;

	if (report == VERBOSE) {
		if (xl && pl && P && num) {
			m = xl->num;
			x = (homo2 **)halloc(num, sizeof(homo2 *));
			for (i=0; i<num; ++i) {
				PointToHomo (pl[i], &x[i], &n);
				if (n != m) {
					Warning("EvalBundleError: illegal point number");
				}
			}
			CoordToHomo (xl, &X, &n);

			BundleError (P, num, x, X, m);

			if (X) {
				free (X); X = NULL;
			}
			for (i=0; i<num; ++i) {
				if (x[i]) {
					free (x[i]); x[i] = NULL;
				}
			}
			if (x) {
				free (x); x = NULL;
			}
		}
	}
}

// ===========================================================================
//   x = P X
// ===========================================================================

homo2 Project (matrix *P, homo3 *X)
{
    homo2 r;

	if (!CheckSizeMatrix (P, 3, 4) || !X) {
		Warning("Project failed!"); return NOHOMO2;
	}
	r.x = P->m[0][0] * X->X + P->m[0][1] * X->Y + P->m[0][2] * X->Z + P->m[0][3] * X->W;
	r.y = P->m[1][0] * X->X + P->m[1][1] * X->Y + P->m[1][2] * X->Z + P->m[1][3] * X->W;
	r.w = P->m[2][0] * X->X + P->m[2][1] * X->Y + P->m[2][2] * X->Z + P->m[2][3] * X->W;
	r.id = X->id;
	NormHomo (&r);

	return r;
}

homo2 Project (matrix *P, xelem *X)
{
    homo2 r;

	if (!CheckSizeMatrix (P, 3, 4) || !X) {
		Warning("Project failed!"); return NOHOMO2;
	}
	r.x = P->m[0][0] * X->X + P->m[0][1] * X->Y + P->m[0][2] * X->Z + P->m[0][3];
	r.y = P->m[1][0] * X->X + P->m[1][1] * X->Y + P->m[1][2] * X->Z + P->m[1][3];
	r.w = P->m[2][0] * X->X + P->m[2][1] * X->Y + P->m[2][2] * X->Z + P->m[2][3];
	r.id = X->id;
	NormHomo (&r);

	return r;
}

// ===========================================================================
//   X = P^T x
// ===========================================================================

homo3 BackProject (matrix *PT, homo2 *x)
{
    homo3 r;

	if (!CheckSizeMatrix (PT, 4, 3) || !x) {
		Warning ("BackProject failed!"); return NOHOMO3;
	}
	r.X = PT->m[0][0] * x->x + PT->m[0][1] * x->y + PT->m[0][2] * x->w;
	r.Y = PT->m[1][0] * x->x + PT->m[1][1] * x->y + PT->m[1][2] * x->w;
	r.Z = PT->m[2][0] * x->x + PT->m[2][1] * x->y + PT->m[2][2] * x->w;
	r.W = PT->m[3][0] * x->x + PT->m[3][1] * x->y + PT->m[3][2] * x->w;
	r.id = x->id;
	NormHomo (&r);

	return r;
}

homo3 BackProject (matrix *PT, pelem *x)
{
    homo3 r;

	if (!CheckSizeMatrix (PT, 4, 3) || !x) {
		Warning ("BackProject failed!"); return NOHOMO3;
	}
	r.X = PT->m[0][0] * x->x + PT->m[0][1] * x->y + PT->m[0][2];
	r.Y = PT->m[1][0] * x->x + PT->m[1][1] * x->y + PT->m[1][2];
	r.Z = PT->m[2][0] * x->x + PT->m[2][1] * x->y + PT->m[2][2];
	r.W = PT->m[3][0] * x->x + PT->m[3][1] * x->y + PT->m[3][2];
	r.id = x->id;
	NormHomo (&r);

	return r;
}

// ===========================================================================
//   x = P X
// ===========================================================================

plist *Project (matrix *P, homo3 *h, int num)
{
	plist *pl = NULL;
	homo2 x;
	int i;

	if (!h || (num < 1)) {
		Warning ("Project: no data!"); return NULL;
	}
	for (i=0; i<num; ++i) {
		x = Project (P, &h[i]);
		pl = AddPoint(pl, &x);
	}
    return pl;
}

plist *Project (matrix *P, xlist *xl)
{
	plist *pl = NULL;
	xelem *p;
	homo2 x;

	if (!xl || (xl->num < 1)) {
		Warning ("Project: no data!"); return NULL;
	}
	for (p=xl->start; p; p=p->next) {
		x = Project (P, p);
		pl = AddPoint(pl, &x);
	}
    return pl;
}

// ===========================================================================
//   P C = 0
// ===========================================================================
/*
static void EvalCameraCenter (matrix *P, homo3 c)
{
    homo2 t;
    double err;

    t = Project(P, c);
    err = sqrt (SQR(t.x) + SQR(t.y));
    if (err > PHI) {
        Warning ("EvalCameraCenter P C = 0: %e", err);
    }
}
*/
// ===========================================================================
//   | p3 | = 1
//   (q1 x q3) (q2 x q3) = 0
// ===========================================================================

void EvalMetricCamera (matrix *P)
{
    double t;
    homo3 q1, q2;

	if (report == VERBOSE) {
		CheckMatrix (P);

		t = sqrt(SQR(P->m[2][0]) + SQR(P->m[2][1]) + SQR(P->m[2][2]));
		if (fabs(1.0 - t) > INT_EPS) {
			Warning ("Eval | p3 | <> 1: %e", t);
		}
		q1.X = P->m[0][1] * P->m[2][2] - P->m[0][2] * P->m[2][1];
		q1.Y = P->m[0][2] * P->m[2][0] - P->m[0][0] * P->m[2][2];
		q1.Z = P->m[0][0] * P->m[2][1] - P->m[0][1] * P->m[2][0];

		q2.X = P->m[1][1] * P->m[2][2] - P->m[1][2] * P->m[2][1];
		q2.Y = P->m[1][2] * P->m[2][0] - P->m[1][0] * P->m[2][2];
		q2.Z = P->m[1][0] * P->m[2][1] - P->m[1][1] * P->m[2][0];

		t = q1.X * q2.X + q1.Y * q2.Y + q1.Z * q2.Z;
		if (ISNONZERO(t)) {
			Warning ("Eval (q1 x q3) (q2 x q3) = 0: %f", t);
		}
	}
}

// ===========================================================================
//
// ===========================================================================
/*
homo3 ProjectionCenterOpt (matrix *P)
{
    matrix *T;
	homo3 c;
	int i;

	if (!CheckSizeMatrix(P, 3, 4)) return NOHOMO3;

	T = mat_alloc (3, 3);
	for (i=0; i<3; ++i) {
		T->m[i][0] = P->m[i][1];
		T->m[i][1] = P->m[i][2];
		T->m[i][2] = P->m[i][3];
	}
	for (i=0; i<3; ++i) {
		T->m[i][0] = P->m[i][0];
		T->m[i][1] = P->m[i][2];
		T->m[i][2] = P->m[i][3];
	}
	c.y = -mat_det (T);
	for (i=0; i<3; ++i) {
		T->m[i][0] = P->m[i][0];
		T->m[i][1] = P->m[i][1];
		T->m[i][2] = P->m[i][3];
	}
	c.z = mat_det (T);
	for (i=0; i<3; ++i) {
		T->m[i][0] = P->m[i][0];
		T->m[i][1] = P->m[i][1];
		T->m[i][2] = P->m[i][2];
	}
	c.w = -mat_det (T);
	T = mat_free (T);

	NormHomo (&c);

//	EvalCameraCenter (P, c);

	return c;
}
*/
// ===========================================================================
//
// ===========================================================================

homo3 ProjectionCenter (matrix *P)
{
    vektor *V;
    homo3 c;

	if (mat_svd_nullvector (P, &V)) {
		c = VecToNormHomo3 (V);
		V = vec_free (V);
//	    EvalCameraCenter (P, c);
		return c;
	}
	Warning("Project failed!");

    return NOHOMO3;
}

// ===========================================================================
//
// ===========================================================================

void DecomposeCamera (matrix *P, matrix **K, matrix **R)
{
    matrix *M;

    NormCamera (P);
    M = mat_alloc (3,3);
    mat_copy (P, M);
    mat_rqd (M, K, R);
    M = mat_free (M);

    EvalRotation (*R);
}

// ===========================================================================
//
// ===========================================================================

matrix *CameraCalibMatrix (matrix *P)
{
	matrix *K, *R;

	DecomposeCamera (P, &K, &R); R = mat_free(R);

	return K;
}

// ===========================================================================
//
// ===========================================================================

matrix *CameraRotMatrix (matrix *P)
{
	matrix *K, *R;

	DecomposeCamera (P, &K, &R); K = mat_free(K);

	return R;
}

// ===========================================================================
//
// ===========================================================================

void PrintCameraMatrix (matrix *P)
{
    matrix *K, *R;
    homo3 c, v;

	if (!CheckSizeMatrix(P, 3, 4)) return;

	mat_print("    Projection Matrix P:", P);

	DecomposeCamera (P, &K, &R);
    c = ProjectionCenter (P);
    v = CameraDirection (P);

	mat_print("    Calibration Matrix K:", K);
	mat_print("    Rotation Matrix R:", R);

	Report ("\nEXTERIOR ORIENTATION:");
	Report ("=====================");
    Report ("    Projection Center C:");
	Report ("\tX     = %9.6f", c.X);
	Report ("\tY     = %9.6f", c.Y);
	Report ("\tZ     = %9.6f", c.Z);
	if (R) {
		mat_print ("    Rotation Matrix R:", R);
		Report ("    XYZ Euler Angles:");
		Report ("\tomega = %7.3f [deg]", RAD2DEG(MINANGLE(atan2(-R->m[2][1], R->m[2][2]))));
		Report ("\tphi   = %7.3f [deg]", RAD2DEG(MINANGLE(asin(R->m[2][0]))));
		Report ("\tkappa = %7.3f [deg]", RAD2DEG(MINANGLE(atan2(-R->m[1][0], R->m[0][0]))));
		R = mat_free (R);
	}
    Report ("    Viewing Direction:");
	Report ("\tX     = %7.3f", v.X);
	Report ("\tY     = %7.3f", v.Y);
	Report ("\tZ     = %7.3f", v.Z);

	if (K) {
		Report ("\nINTERIOR ORIENTATION:");
		Report ("=====================");
		Report ("    Principal Distance c: %7.2f [pix]", K->m[0][0]);
		Report ("    Principal Point xh  : %7.2f [pix]", K->m[0][2]);
		Report ("                    yh  : %7.2f [pix]", K->m[1][2]);
		Report ("    Skew Factor s       :   %.2f [deg]",
				RAD2DEG(MINANGLE(ACOT(-K->m[0][1] / CheckZero(K->m[0][0])))));
		Report ("    Aspect Ratio a      :    %f", K->m[1][1] / CheckZero (K->m[0][0]));
        K = mat_free (K);
	}
	Report ("");
}

// ===========================================================================
//   Adapted version of balanc
//   Copyright (C) 1986-2002 Numerical Recipes Software.
//   P.O. Box 380243, Cambridge, MA 02238 (USA), http://www.nr.com/
// ===========================================================================

static void balanc (double **a, int n)
{
    int last, j, i;
    double s, r, g, f, c, sqrdx;

    sqrdx = 4.0;
    last = 0;

    while (last == 0) {
        last = 1;
        for (i=0; i<n; ++i) {
            r = c = 0.0;
            for (j=0; j<n; ++j) {
                if (j != i) {
                    c += fabs(a[j][i]);
                    r += fabs(a[i][j]);
                }
            }
            if ((ISNONZERO(c)) && (ISNONZERO(r))) {
                g = r / 2.0;
                f = 1.0;
                s = c + r;
                while (c < g) {
                    f *= 2.0;
                    c *= sqrdx;
                }
                g = r * 2.0;
                while (c > g) {
                    f /= 2.0;
                    c /= sqrdx;
                }
                if ((c + r) / f < 0.95 * s) {
                    last = 0;
                    g = 1.0 / f;
                    for (j=0; j<n; ++j) {
                        a[i][j] *= g;
                    }
                    for (j=0; j<n; ++j) {
                        a[j][i] *= f;
                    }
                }
            }
        }
    }
}

static double pythag (double a, double b)
{
    double absa,absb;

    absa = ABS(a);
    absb = ABS(b);
    if (absa > absb) {
        return absa*sqrt(1.0+SQR(absb/absa));
    } else {
        return (ISZERO(absb) ? 0.0 : absb*sqrt(1.0+SQR(absa/absb)));
    }
}

// ===========================================================================
//   Adapted version of svdcmp
//   Copyright (C) 1986-2002 Numerical Recipes Software.
//   P.O. Box 380243, Cambridge, MA 02238 (USA), http://www.nr.com/
// ===========================================================================

static bool svdcmp(double **a, int m, int n, double *w, double **v)
{
    int flag,i,its,j,jj,k,l=0,nm=0;
    double anorm,c,f,g,h,s,scale,x,y,z,*rv1;
//    double volatile temp; //see bugreport

    rv1 = (double *) halloc (n, sizeof(double));
    g=scale=anorm=0.0;
                                    // Householder reduction to bidigonal form
    for (i=0;i<n;++i) {
        l=i+2;
        rv1[i]=scale*g;
        g=s=scale=0.0;
        if (i < m) {
            for (k=i;k<m;++k) scale += ABS(a[k][i]);
            if (ISNONZERO(scale)) {
                for (k=i;k<m;++k) {
                    a[k][i] /= scale;
                    s += a[k][i]*a[k][i];
                }
                f=a[i][i];
                g = -SIGN(sqrt(s),f);
                h=f*g-s;
                a[i][i]=f-g;
                for (j=l-1;j<n;++j) {
                    for (s=0.0,k=i;k<m;++k) s += a[k][i]*a[k][j];
                    f=s/h;
                    for (k=i;k<m;++k) a[k][j] += f*a[k][i];
                }
                for (k=i;k<m;++k) a[k][i] *= scale;
            }
        }
        w[i]=scale *g;
        g=s=scale=0.0;
        if (i+1 <= m && i+1 != n) { // see bug report!
            for (k=l-1;k<n;++k) scale += ABS(a[i][k]);
            if (ISNONZERO(scale)) {
                for (k=l-1;k<n;++k) {
                    a[i][k] /= scale;
                    s += a[i][k]*a[i][k];
                }
                f=a[i][l-1];
                g = -SIGN(sqrt(s),f);
                h=f*g-s;
                a[i][l-1]=f-g;
                for (k=l-1;k<n;++k) rv1[k]=a[i][k]/h;
                for (j=l-1;j<m;++j) {
                    for (s=0.0,k=l-1;k<n;++k) s += a[j][k]*a[i][k];
                    for (k=l-1;k<n;++k) a[j][k] += s*rv1[k];
                }
                for (k=l-1;k<n;++k) a[i][k] *= scale;
            }
        }
        anorm=MAX(anorm,(fabs(w[i])+fabs(rv1[i])));
    }

                                 // Accumulation of right-hand transformations
    for (i=n-1;i>=0;i--) {
        if (i < n-1) {
            if (ISNONZERO(g)) {
                                // double division to avoid possible underflow
                for (j=l;j<n;++j)
                    v[j][i]=(a[i][j]/a[i][l])/g;
                for (j=l;j<n;++j) {
                    for (s=0.0,k=l;k<n;++k) s += a[i][k]*v[k][j];
                    for (k=l;k<n;++k) v[k][j] += s*v[k][i];
                }
            }
            for (j=l;j<n;++j) v[i][j]=v[j][i]=0.0;
        }
        v[i][i]=1.0;
        g=rv1[i];
        l=i;
    }
                                  // Accumulation of left-hand transformations
    for (i=MIN(m,n)-1;i>=0;i--) {
        l=i+1;
        g=w[i];
        for (j=l;j<n;++j) a[i][j]=0.0;
        if (ISNONZERO(g)) {
            g=1.0/g;
            for (j=l;j<n;++j) {
                for (s=0.0,k=l;k<m;++k) s += a[k][i]*a[k][j];
                f=(s/a[i][i])*g;
                for (k=i;k<m;++k) a[k][j] += f*a[k][i];
            }
            for (j=i;j<m;++j) a[j][i] *= g;
        } else for (j=i;j<m;++j) a[j][i]=0.0;
        ++a[i][i];
    }
                                      // diagonalization of the bidigonal form
    for (k=n-1;k>=0;k--) {                         // loop over singlar values
        for (its=1;its<=MAXITS;++its) {            // loop over allowed iterations
            flag=true;
            for (l=k;l>=0;l--) {                         // test for splitting
                nm=l-1;
// NEW
				if (l == 0 || ABS(rv1[l]) <= INT_EPS*anorm) {
					flag=false;
					break;
				}
				if (abs(w[nm]) <= INT_EPS*anorm) break;
/* OLD
                temp=fabs(rv1[l])+anorm; //see bug report
                if (ISEQUAL(temp,anorm)||nm<0) {
                    flag=false;
                    break;
                }
                temp=fabs(w[nm])+anorm; //see bug report
                if (ISEQUAL(temp,anorm)) break;
*/
            }
            if (flag) {
                c=0.0;
                s=1.0;
                for (i=l;i<k+1;++i) {
                    f=s*rv1[i];
                    rv1[i]=c*rv1[i];
// NEW
					if (abs(f) <= INT_EPS*anorm) break;
/* OLD
                    temp=ABS(f)+anorm; //see bug report
                    if (ISEQUAL(temp,anorm)) break;
*/
                    g=w[i];
                    h=pythag(f,g);
                    w[i]=h;
                    h=1.0/h;
                    c=g*h;
                    s = -f*h;
                    for (j=0;j<m;++j) {
                        y=a[j][nm];
                        z=a[j][i];
                        a[j][nm]=y*c+z*s;
                        a[j][i]=z*c-y*s;
                    }
                }
            }
            z=w[k];
            if (l == k) {                                       // convergence
                if (z < 0.0) {
                    w[k] = -z;
                    for (j=0;j<n;++j) v[j][k] = -v[j][k];
                }
                break;
            }
			if (its == MAXITS) {
				Warning ("svdcmp: No convergence in %d iterations !", MAXITS);
                return false;
			}
            x=w[l];                          // shift from bottom 2-by-2 minor
            nm=k-1;
            y=w[nm];
            g=rv1[nm];
            h=rv1[k];
            f=((y-z)*(y+z)+(g-h)*(g+h))/(2.0*h*y);
            g=pythag(f,1.0);
                                                     // next QR transformation
            f=((x-z)*(x+z)+h*((y/(f+SIGN(g,f)))-h))/x;
            c=s=1.0;
            for (j=l;j<=nm;++j) {
                i=j+1;
                g=rv1[i];
                y=w[i];
                h=s*g;
                g=c*g;
                z=pythag(f,h);
                rv1[j]=z;
                c=f/z;
                s=h/z;
                f=x*c+g*s;
                g=g*c-x*s;
                h=y*s;
                y *= c;
                for (jj=0;jj<n;++jj) {
                    x=v[jj][j];
                    z=v[jj][i];
                    v[jj][j]=x*c+z*s;
                    v[jj][i]=z*c-x*s;
                }
                z=pythag(f,h);
                w[j]=z;                    // rotation can be arbitrary id z=0
                if (ISNONZERO(z)) {
                    z=1.0/z;
                    c=f*z;
                    s=h*z;
                }
                f=c*g+s*y;
                x=c*y-s*g;
                for (jj=0;jj<m;++jj) {
                    y=a[jj][j];
                    z=a[jj][i];
                    a[jj][j]=y*c+z*s;
                    a[jj][i]=z*c-y*s;
                }
            }
            rv1[l]=0.0;
            rv1[k]=f;
            w[k]=x;
        }
    }
	if (rv1) {
		free (rv1); rv1 = NULL;
	}
	return true;
}

// ===========================================================================
//   Sortieren nach den Singul�werten
// ===========================================================================

static void
mat_svd_sort (matrix *U, vektor *d, matrix *V)
{
    int i, j, k, min, n;

    n = U->cols;

    for (i = n-1; i >= 0; i--) {
        min = i;

        for (j = 0; j < i; ++j) {                         // find minimal mean
            if (d->v[j] < d->v[min]) {
                min = j;
            }
        }
        if (i != min) {
            SWAP (double, d->v[min], d->v[i]);

            for (k = 0; k < n; ++k) {
                SWAP (double, U->m[k][i], U->m[k][min]);
            }
            for (k = 0; k < n; ++k) {
                SWAP (double, V->m[k][i], V->m[k][min]);
            }
        }
    }
}

// ===========================================================================
//   Faktorisierung der Matrix A = U D V^T
// ===========================================================================

bool
mat_svd (matrix *A, matrix **U, vektor **d, matrix **V, bool sort)
{
    int r, c;
	bool res = false;

	if (U && d && V) {
		*U = *V = NULL; *d = NULL;

		if (!CheckFiniteMatrix (A)) {
			return false;
		}
		r = A->rows;
		c = A->cols;
		if (r < c) {
			r = c;
		}
		*U = mat_alloc (r, c); mat_copy (A, *U);
		*d = vec_alloc (c);
		*V = mat_alloc (c, c);

		res = svdcmp ((*U)->m, r, c, (*d)->v, (*V)->m);
		if (res) {
			if (sort) {
				mat_svd_sort (*U, *d, *V);
			}
		} else {
			*U = mat_free(*U);
			*d = vec_free(*d);
			*V = mat_free(*V);
		}
	}
	return res;
}

// ===========================================================================
//   Rekomposition A = U D V^T
// ===========================================================================

void mat_svd_recompose (matrix *U, vektor *d, matrix *V, matrix **A)
{
    matrix *T, *D, *VT;
    int c, r, i;

	if (A) {
		*A = NULL;

		if (!CheckMatrix (U) || !CheckVector (d) || !CheckSquareMatrix (V)) return;

		c = U->cols;
		r = U->rows;
		if ((r < c) || (V->cols != c) || (d->len != c)) {
			Warning ("mat_svd_recompose: illegal size"); return;
		}
		D = mat_alloc (c, c);                                // Diagonal matrix D
		for (i=0; i<c; ++i) {
			D->m[i][i] = d->v[i];
		}
		VT = mat_transpose_new (V);

		T = mat_prod_new (D, VT); D = mat_free (D);	VT = mat_free (VT);
		*A = mat_prod_new (U, T); T = mat_free (T);
	}
}

// ===========================================================================
//   Kleinste Quadrate L�ung A x = b
// ===========================================================================

bool mat_least_squares (matrix *A, vektor *b, vektor **x)
{
    matrix *U, *V, *T;
    vektor *b2, *y, *d;
    bool res = false;
    int i, m;

	if (x) {
		*x = NULL;

		if (!CheckMatrix (A)) return false;

		if (!b || (b->len != A->rows)) {
			Warning ("mat_least_squares: illegal vector size b"); return false;
		}
		m = A->cols;

		res = mat_svd (A, &U, &d, &V, false);
		if (res) {
			T = mat_transpose_new (U);
			b2 = vec_transform_new (T, b);
			U = mat_free (U);
			T = mat_free (T);

			y = vec_alloc (m);
			for (i=0; i<m; ++i) {
				if (ISNONZERO(d->v[i])) {
					y->v[i] = b2->v[i] / d->v[i];
				} else {
//	                res = false;
					y->v[i] = b2->v[i];
				}
			}
			b2 = vec_free (b2);
			d = vec_free (d);

			*x = vec_transform_new (V, y);
			V = mat_free (V);
			y = vec_free (y);
		}
	}
	return res;
}

// ===========================================================================
//   Rechter Nullraum
// ===========================================================================

bool mat_svd_nullspace (matrix *A, int num, vektor ***N)
{
    matrix *U, *V;
	vektor *d;
	bool res = false;
	int i;

	if (N) {
		*N = NULL;

		res = mat_svd (A, &U, &d, &V, true);
		if (res) {
			U = mat_free (U);
			d = vec_free (d);

			*N = (vektor **) halloc (num, sizeof(vektor *));
			for (i=0; i<num; ++i) {
				(*N)[i] = mat_getcol (V, V->rows-1 - i);
			}
			V = mat_free (V);
		}
	}
	return res;
}

// ===========================================================================
//   Rechter Nullvektor (Rechte Spalte von V)
// ===========================================================================

bool mat_svd_nullvector (matrix *A, vektor **N)
{
	bool res = false;
	matrix *B, *AT, *U, *V;
	vektor *d;

	if (N) {
		*N = NULL;

		if (!CheckMatrix(A)) return false;
		//DBUG MG: disalbe this possibility. Do not really know what it might be good for...
		//if (A->rows > 2*A->cols) {
		if (1==0){
			AT = mat_transpose_new(A);
			B = mat_prod_new(AT, A); AT = mat_free (AT);
			res = mat_svd (B, &U, &d, &V, true);
			if (res) {
				U = mat_free (U); d = vec_free (d);
				*N = mat_getcol (V, V->rows-1);
				V = mat_free (V);
			}
			B = mat_free (B);
		} else {
			res = mat_svd (A, &U, &d, &V, true);
			if (res) {
				U = mat_free (U); d = vec_free (d);
				*N = mat_getcol (V, V->rows-1);
				V = mat_free (V);
			}
		}
	}
	return res;
}

// ===========================================================================
//   Linker Nullvektor (Rechte Spalte von U)
// ===========================================================================

bool mat_svd_left_nullvector (matrix *A, vektor **N)
{
	matrix *U, *V;
	vektor *d;
	bool res = false;

	if (N) {
		*N = NULL;

		res = mat_svd (A, &U, &d, &V, true);
		if (res) {
			d = vec_free (d);
			V = mat_free (V);

			*N = mat_getcol (U, U->cols-1);
			U = mat_free (U);
		}
	}
	return res;
}

// ===========================================================================
//   Fast SVD for essential matrix (D. Nister)
// ===========================================================================

bool mat_svd_essential (matrix *E, matrix **U, matrix **V)
{
    vektor *ea, *eb, *ec;
    vektor *ca, *cb, *cc;
    vektor *va, *vb, *vc;
    vektor *ua, *ub, *uc;
	double la, lb, lc;

	if (U && V) {
		*U = *V = NULL;

		if (!CheckMatrix(E)) return false;

		ea = mat_getrow(E, 0);
	    eb = mat_getrow(E, 1);
		ec = mat_getrow(E, 2);

		ca = vec_cross_new(ea, eb); la = vec_len(ca);
		cb = vec_cross_new(ea, ec); lb = vec_len(cb);
		cc = vec_cross_new(eb, ec); lc = vec_len(cc);

		if ((la >= lb) && (la >= lc)) {
			vc = ca;
			va = ea;
			eb = vec_free (eb); ec = vec_free (ec);
			cb = vec_free (cb); cc = vec_free (cc);
		} else if ((lb >= la) && (lb >= lc)) {
			vc = cb;
			va = ea;
			eb = vec_free (eb); ec = vec_free (ec);
			ca = vec_free (ca); cc = vec_free (cc);
		} else {
			vc = cc;
			va = eb;
			ea = vec_free (ea); ec = vec_free (ec);
			ca = vec_free (ca); cb = vec_free (cb);
		}
		vec_norm (vc);
		vec_norm (va);
		vb = vec_cross_new(vc, va);
		ua = vec_transform_new(E, va); vec_norm (ua);
		ub = vec_transform_new(E, vb); vec_norm (ub);
		uc = vec_cross_new(ua, ub);

		*V = mat_alloc(3,3);
		mat_setcol(*V, 0, va); va = vec_free(va);
		mat_setcol(*V, 1, vb); vb = vec_free(vb);
		mat_setcol(*V, 2, vc); vc = vec_free(vc);

		*U = mat_alloc(3,3);
		mat_setcol(*U, 0, ua); ua = vec_free(ua);
		mat_setcol(*U, 1, ub); ub = vec_free(ub);
		mat_setcol(*U, 2, uc); uc = vec_free(uc);

		return true;
	}
	return false;
}


static void eval_rqd (matrix *M, matrix *R, matrix *Q)
{
    matrix *T;
    double err;
    int i, j, n;

	if (report == VERBOSE) {
		n = Q->cols;
		T = mat_prod_new (R, Q);

		err = 0.0;
		for (i=0; i<n; ++i) {
			for (j=0; j<n; ++j) {
				err += fabs (M->m[i][j] - T->m[i][j]);
			}
		}
		if (err > INT_EPS) {
			Warning ("EvalRQ: Recomposition RQ = M differ: %f", err);
			mat_print ("M original:", M);
			mat_print ("M recomposed:", T);
		}
		T = mat_free (T);
	}
}

// ===========================================================================
//
// ===========================================================================

static void eval_qrd (matrix *M, matrix *Q, matrix *R)
{
    matrix *T;
    double err;
    int i, j, n;

	if (report == VERBOSE) {
		n = Q->cols;
		T = mat_prod_new (Q, R);

		err = 0.0;
		for (i=0; i<n; ++i) {
			for (j=0; j<n; ++j) {
				err += fabs (M->m[i][j] - T->m[i][j]);
			}
		}
		if (err > INT_EPS) {
			Warning ("EvalQR: Recomposition QR = M differ: %f", err);
			mat_print ("M original:", M);
			mat_print ("M recomposed:", T);
		}
		T = mat_free (T);
	}
}

// ===========================================================================
//
// ===========================================================================

static void ConstraintQR (matrix *Q, matrix *R)
{
	int i, j, n;

	if (!CheckSquareMatrix(Q) || !CheckSameMatrix(R, Q)) {
		return;
	}
	n = Q->cols;
    for (i=0; i<n; ++i) {
        if (R->m[i][i] < 0) {
            for (j=0; j<n; ++j) {
                R->m[i][j] *= -1.0;
                Q->m[j][i] *= -1.0;
            }
        }
    }
}

static void ConstraintRQ (matrix *R, matrix *Q)
{
	int i, j, n;

	if (!CheckSquareMatrix(R) || !CheckSameMatrix(R, Q)) {
		return;
	}
	n = R->cols;
    for (i=0; i<n; ++i) {
        if (R->m[i][i] < 0) {
            for (j=0; j<n; ++j) {
                R->m[j][i] *= -1.0;
                Q->m[i][j] *= -1.0;
            }
        }
    }
}

// ===========================================================================
//   Adapted version of qrdcmp
//   Copyright (C) 1986-2002 Numerical Recipes Software.
//   P.O. Box 380243, Cambridge, MA 02238 (USA), http://www.nr.com/
// ===========================================================================

bool qrdcmp(matrix *a, matrix **Q, matrix **R)
{
	int i, j, k, n;
	double scale, sigma, sum, tau;
	vektor *c, *d;
	bool sing = false;

	*R = *Q = NULL;
	if (!CheckSquareMatrix(a)) return sing;

	n = a->cols;
	c = vec_alloc(n);
	d = vec_alloc(n);
	*R = mat_clone(a);

	for (k=0; k<n-1; k++) {
		scale = 0.0;
		for (i=k; i<n; i++) {
			scale = MAX(scale, ABS((*R)->m[i][k]));
		}
		if (ISZERO(scale)) {
			sing = true;
			c->v[k] = d->v[k] = 0.0;
		} else {
			for (i=k; i<n; i++) {
				(*R)->m[i][k] /= scale;
			}
			for (sum=0.0, i=k; i<n; i++) {
				sum += SQR((*R)->m[i][k]);
			}
			sigma = SIGN(sqrt(sum), (*R)->m[k][k]);
			(*R)->m[k][k] += sigma;
			c->v[k] = sigma * (*R)->m[k][k];
			d->v[k] = -scale * sigma;
			for (j=k+1; j<n; j++) {
				for (sum=0.0, i=k; i<n; i++) {
					sum += (*R)->m[i][k] * (*R)->m[i][j];
				}
				tau = sum / c->v[k];
				for (i=k; i<n; i++) {
					(*R)->m[i][j] -= tau * (*R)->m[i][k];
				}
			}
		}
	}
	d->v[n-1] = (*R)->m[n-1][n-1];
	if (ISZERO(d->v[n-1])) {
		sing = true;
	}
	*Q = mat_alloc(n, n);

	for (i=0; i<n; i++) {
		for (j=0; j<n; j++) {
			(*Q)->m[i][j] = 0.0;
		}
		(*Q)->m[i][i] = 1.0;
	}
	for (k=0; k<n-1; k++) {
		if (ISNONZERO(c->v[k])) {
			for (j=0; j<n; j++) {
				sum = 0.0;
				for (i=k; i<n; i++) {
					sum += (*R)->m[i][k] * (*Q)->m[i][j];
				}
				sum /= c->v[k];
				for (i=k; i<n; i++) {
					(*Q)->m[i][j] -= sum * (*R)->m[i][k];
				}
			}
		}
	}
	for (i=0; i<n; i++) {
		(*R)->m[i][i] = d->v[i];
		for (j=0;j<i;j++) {
			(*R)->m[i][j] = 0.0;
		}
	}
	c = vec_free(c);
	d = vec_free(d);

	return sing;
}

bool mat_qrd (matrix *A, matrix **Q, matrix **R)
{
	*Q = *R = NULL;
	if (!CheckSquareMatrix (A)) {
		return false;
	}
	if (qrdcmp(A, Q, R)) {
		Warning("mat_qrd: Singularity in QR decomposition");
		*Q = mat_free(*Q);
		*R = mat_free(*R);
		return false;
	}
	ConstraintQR (*Q, *R);
    eval_qrd (A, *Q, *R);

	return true;
}

// ===========================================================================
//
// ===========================================================================

bool mat_rqd_alt (matrix *A, matrix **R, matrix **Q)
{
	matrix *T;

	*Q = *R = NULL;
	if (!CheckSquareMatrix (A)) {
		return false;
	}
	T = mat_invert_new(A);
	if (qrdcmp(T, Q, R)) {
		Warning("mat_rqd: Singularity in QR decomposition");
		T = mat_free(T);
		*Q = mat_free(*Q);
		*R = mat_free(*R);
		return false;
	}
	T = mat_free(T);
	mat_invert(*Q);
	mat_invert(*R);

	ConstraintRQ (*R, *Q);
	eval_rqd (A, *R, *Q);

	return true;
}

// ===========================================================================
//   RQ-Decomposition A = RQ
//   R: Upper Triangular
//   Q: Othogonal Matrix
// ===========================================================================

static void Givens (double a, double b, double *s, double *c)
{
    double t;

    t = CheckZero(sqrt(SQR(a) + SQR(b)));
    *s = a / t;
    *c = b / t;
}

// ===========================================================================
//
// ===========================================================================

bool mat_rqd (matrix *A, matrix **R, matrix **Q)
{
    matrix *Qx, *Qy, *Qz, *T, *T2;
    double c, s;

    if (!CheckSizeMatrix (A, 3, 3)) return false;

    T = mat_clone (A);

    Givens (-T->m[2][1], T->m[2][2], &s, &c);
    Qx = mat_alloc (3, 3);
    Qx->m[0][0] =  1;
    Qx->m[1][1] =  c;
    Qx->m[1][2] = -s;
    Qx->m[2][1] =  s;
    Qx->m[2][2] =  c;
    T2 = mat_prod_new (T, Qx); T = mat_free (T);

    Givens (T2->m[2][0], T2->m[2][2], &s, &c);
    Qy = mat_alloc (3, 3);
    Qy->m[0][0] =  c;
    Qy->m[0][2] =  s;
    Qy->m[1][1] =  1;
    Qy->m[2][0] = -s;
    Qy->m[2][2] =  c;
    T = mat_prod_new (T2, Qy); T2 = mat_free (T2);

    Givens (-T->m[1][0], T->m[1][1], &s, &c);
    Qz = mat_alloc (3, 3);
    Qz->m[0][0] =  c;
    Qz->m[0][1] = -s;
    Qz->m[1][0] =  s;
    Qz->m[1][1] =  c;
    Qz->m[2][2] =  1;

    *R = mat_prod_new (T, Qz); T = mat_free (T);

    mat_transpose (Qx);
    mat_transpose (Qy);
    mat_transpose (Qz);

    T = mat_prod_new (Qy, Qx);
    Qy = mat_free (Qy); Qx = mat_free (Qx);

    *Q = mat_prod_new (Qz, T);
    Qz = mat_free (Qz); T = mat_free (T);
                                                // R diagonal entries positive
	(*R)->m[1][0] = (*R)->m[2][0] = (*R)->m[2][1] = 0.0;  // lower triangle zero
	ConstraintRQ (*R, *Q);

    eval_rqd (A, *R, *Q);

	return true;
}

homo3 CameraDirection (matrix *P)
{
    matrix *M;
    homo3 v;
    double t;

	if (!CheckSizeMatrix(P, 3, 4)) return NOHOMO3;

    NormCamera (P);
    M = mat_alloc (3, 3);
    mat_copy (P, M);

    t = mat_det(M);
    v.X = t * M->m[2][0];
    v.Y = t * M->m[2][1];
    v.Z = t * M->m[2][2];
    v.W = t;
    NormHomo (&v);

    M = mat_free (M);

    return v;
}

// ===========================================================================
//
// ===========================================================================
// ===========================================================================
// Fast Triangulation for minimum essential matrix computation (Nister)
// ===========================================================================

static xlist *FastTriangulation (matrix *E, matrix *P2, homo2 *h1, homo2 *h2, int n)
{
    xlist *xl = NULL;
	matrix *PT;
	homo3 X, C;
    homo2 h, c;
	int i;

	if (!E || !P2 || !h1 || !h2 || (n < 1)) {
		Warning ("FastTriangulation: no data!"); return NULL;
	}
    PT = mat_transpose_new(P2);
	for (i=0; i<n; ++i) {
		if (CheckID(&h1[i], &h2[i])) {
			h = TransformHomo(E, &h1[i]); h.w = 0.0;
			c = HomoCross(&h2[i], &h);
			C = BackProject(PT, &c);
			X.X = h1[i].x*C.W;
			X.Y = h1[i].y*C.W;
			X.Z = h1[i].w*C.W;
			X.W = -(h1[i].x*C.X + h1[i].y*C.Y + h1[i].w*C.Z);
			X.id = h1[i].id;
			xl = AddCoord (xl, &X);
		}
	}
	PT = mat_free(PT);

    return xl;
}

static xlist *FastTriangulation (matrix *E, matrix *P2, plist *pl1, plist *pl2)
{
    xlist *xl = NULL;
	matrix *PT;
	homo3 X, C;
	homo2 h, c;
    pelem *p, *q;

	if (!E || !P2 || !pl1 || !pl2 || (pl1->num < 1) || (pl1->num != pl2->num)) {
		Warning ("FastTriangulation: no data!"); return NULL;
	}
    PT = mat_transpose_new(P2);
	for (p=pl1->start, q=pl2->start; p && q; p=p->next, q=q->next) {
		if (CheckID(p,q)) {
			h = TransformPointNew(E, p); h.w = 0.0;
			c = HomoCross(q, &h);
			C = BackProject(PT, &c);
			X.X = p->x*C.W;
			X.Y = p->y*C.W;
			X.Z =      C.W;
			X.W = -(p->x*C.X + p->y*C.Y + C.Z);
			X.id = p->id;
			xl = AddCoord (xl, &X);
		}
	}
	PT = mat_free(PT);

    return xl;
}

// ===========================================================================
//
// ===========================================================================

int CountInFront(matrix *P1, matrix *P2, plist *pl1, plist *pl2)
{
	xlist *xl;
	xelem *x;
	int c = 0;

	if (CheckMatrix(P1) && CheckMatrix(P2)) {
		xl = Triangulation (P1, P2, pl1, pl2);
		if (xl) {
			for (x = xl->start; x; x=x->next) {
				c += (IsInFront(P1, x) ? 1 : 0) + (IsInFront(P2, x) ? 1 : 0);
			}
			xl = FreeCoord(xl);
		}
	}
	return c;
}


int CountInFront(matrix *P1, matrix *P2, matrix *E, plist *pl1, plist *pl2)
{
	xlist *xl;
	xelem *x;
	int c = 0;

	if (CheckMatrix(P1) && CheckMatrix(P2) && CheckMatrix(E)) {
		NormCamera (P2);

		xl = FastTriangulation (E, P2, pl1, pl2);
		if (xl) {
		    for (x = xl->start; x; x=x->next) {
			    c += (CheckOrientation (P1, x) > 0) + (CheckOrientation (P2, x) > 0);
		    }
			xl = FreeCoord(xl);
		}
	}
	return c;
}
// ===========================================================================
//
// ===========================================================================


int CountInFront(matrix *P1, matrix *P2, homo2 *h1, homo2 *h2, int num)
{
	xlist *xl;
	xelem *x;
	int c = 0;

	if (CheckMatrix(P1) && CheckMatrix(P2)) {
		xl = Triangulation (P1, P2, h1, h2, num);
		if (xl) {
			for (x = xl->start; x; x=x->next) {
				c += (IsInFront(P1, x) ? 1 : 0) + (IsInFront(P2, x) ? 1 : 0);
			}
			xl = FreeCoord(xl);
		}
	}
	return c;
}


int CountInFront(matrix *P1, matrix *P2, matrix *E, homo2 *h1, homo2 *h2, int num)
{
	xlist *xl;
	xelem *x;
	int c = 0;

	if (CheckMatrix(P1) && CheckMatrix(P2) && CheckMatrix(E)) {
		NormCamera (P2);

		xl = FastTriangulation (E, P2, h1, h2, num);
		if (xl) {
			for (x = xl->start; x; x=x->next) {
				c += (CheckOrientation (P1, x) > 0) + (CheckOrientation (P2, x) > 0);
			}
			xl = FreeCoord(xl);
		}
	}
	return c;
}


static double Depth (matrix *P, homo3 *X)
{
    matrix *M;
    double w, d = 0.0, t, s;

	if (CheckSizeMatrix(P, 3, 4) && X) {
		M = mat_alloc (3, 3);
		mat_copy (P, M);
		s = SGN(mat_det(M));
		M = mat_free (M);

		w = P->m[2][0] * X->X + P->m[2][1] * X->Y +	P->m[2][2] * X->Z + P->m[2][3] * X->W;
		t = X->W * sqrt(SQR(P->m[2][0]) + SQR(P->m[2][1]) + SQR(P->m[2][2]));
		if (ISNONZERO(t)) {
			d = s *	w / t;
		}
	}
    return d;
}

static double Depth (matrix *P, xelem *X)
{
    matrix *M;
    double w, d=0.0, t, s;

	if (CheckSizeMatrix(P, 3, 4) && X) {
		M = mat_alloc (3, 3);
		mat_copy (P, M);
		s = SGN(mat_det(M));
		M = mat_free (M);

		w = P->m[2][0] * X->X + P->m[2][1] * X->Y + P->m[2][2] * X->Z + P->m[2][3];
		t = sqrt(SQR(P->m[2][0]) + SQR(P->m[2][1]) + SQR(P->m[2][2]));
		if (ISNONZERO(t)) {
			d = s * w / t;
		}
	}
    return d;
}

// ===========================================================================
//
// ===========================================================================

bool IsInFront (matrix *P, homo3 *X)
{
    return Depth(P, X) > 1.0; //assuming image plane at 1. No point can lie inside the camera
}

bool IsInFront (matrix *P, xelem *X)
{
    return Depth(P, X) > 1.0; //assuming image plane at 1. No point can lie inside the camera
}
// ===========================================================================
//
// ===========================================================================

static void GetTwistedPair (matrix *E, matrix **P1, matrix **P2)
{
    matrix *V, *U, *D, *T, *T2;
	vektor *t;
//	vektor *d;

//	if (!mat_svd (E, &U, &d, &V, true)) {
	if (!mat_svd_essential (E, &U, &V)) {      // FAST
		Warning ("GetTwistedPair failed!");
		*P1 = *P2 = NULL;
		return;
	}
//	d = vec_free(d);                               // E = U diag(1,1,0) V^T
	if (mat_det(U) < 0)
        mat_scale(U, -1.0);    // det(U) > 0
	if (mat_det(V) < 0)
        mat_scale(V, -1.0);    // det(V) > 0

	D = mat_alloc(3,3);                         //     | 0  1 0 |
	D->m[0][1] = D->m[2][2] = 1;                // D = |-1  0 0 |
	D->m[1][0] = -1;                            //     | 0  0 1 |

    t = mat_getcol (U, 2);						// t = [U_02 U_12 U_22]^T,   | t | = 1

	*P1 = mat_alloc(3,4);                        // P1 = [ U D V^T | t ]
	mat_transpose (V);
	T = mat_prod_new(D, V);
	T2 = mat_prod_new(U, T); T = mat_free(T);
	mat_copy (T2, *P1); T2 = mat_free (T2);
    mat_setcol (*P1, 3, t);

	*P2 = mat_alloc(3,4);                        // P2 = [ U D^T V^T | t ]
	mat_transpose (D);
	T = mat_prod_new(D, V); D = mat_free(D); V = mat_free(V);
	T2 = mat_prod_new(U, T); U = mat_free(U); T = mat_free(T);
	mat_copy (T2, *P2); T2 = mat_free (T2);
    mat_setcol (*P2, 3, t);

	t = vec_free(t);
}
// ===========================================================================
//
// ===========================================================================

static void RecoverCameras (matrix *E, matrix **P1, matrix **P2, matrix **P3, matrix **P4)
{
	GetTwistedPair (E, P1, P3);

	*P2 = mat_clone(*P1);
	if (*P2) {
		(*P2)->m[0][3] = -(*P2)->m[0][3];
		(*P2)->m[1][3] = -(*P2)->m[1][3];
		(*P2)->m[2][3] = -(*P2)->m[2][3];
	}
	*P4 = mat_clone(*P3);
	if (*P4) {
		(*P4)->m[0][3] = -(*P4)->m[0][3];
		(*P4)->m[1][3] = -(*P4)->m[1][3];
		(*P4)->m[2][3] = -(*P4)->m[2][3];
	}
}

int RecoverMotion (matrix *E, homo2 *h1, homo2 *h2, int num)
{
    matrix *P1, *P21, *P22, *P23, *P24;
	int c1, c2, c3, c4;
	int m = 0;

	if (CheckMatrix(E) && h1 && h2 && (num > 0)) {
		P1 = NormalizedCameraMatrix ();
		RecoverCameras (E, &P21, &P22, &P23, &P24);

        c1 = CountInFront(P1, P21, E, h1, h2, num); P21 = mat_free(P21);
		c2 = CountInFront(P1, P22, E, h1, h2, num); P22 = mat_free(P22);
		c3 = CountInFront(P1, P23, E, h1, h2, num); P23 = mat_free(P23);
		c4 = CountInFront(P1, P24, E, h1, h2, num); P24 = mat_free(P24);
/*
        c1 = CountInFront(P1, P21, h1, h2, num); P21 = mat_free(P21);
		c2 = CountInFront(P1, P22, h1, h2, num); P22 = mat_free(P22);
		c3 = CountInFront(P1, P23, h1, h2, num); P23 = mat_free(P23);
		c4 = CountInFront(P1, P24, h1, h2, num); P24 = mat_free(P24);
*/
		P1 = mat_free(P1);

		m = MAX(c1, MAX(c2, MAX(c3, c4)));
	}
	return m;
}

int RecoverMotion (matrix *E, plist *pl1, plist *pl2)
{
    matrix *P1, *P21, *P22, *P23, *P24;
	int c1, c2, c3, c4;
	int m = 0;

	if (CheckMatrix(E) && pl1 && pl2) {
		P1 = NormalizedCameraMatrix ();
		RecoverCameras(E, &P21, &P22, &P23, &P24);

		c1 = CountInFront(P1, P21, E, pl1, pl2); P21 = mat_free(P21);
		c2 = CountInFront(P1, P22, E, pl1, pl2); P22 = mat_free(P22);
		c3 = CountInFront(P1, P23, E, pl1, pl2); P23 = mat_free(P23);
		c4 = CountInFront(P1, P24, E, pl1, pl2); P24 = mat_free(P24);
/*
		c1 = CountInFront(P1, P21, pl1, pl2); P21 = mat_free(P21);
		c2 = CountInFront(P1, P22, pl1, pl2); P22 = mat_free(P22);
		c3 = CountInFront(P1, P23, pl1, pl2); P23 = mat_free(P23);
		c4 = CountInFront(P1, P24, pl1, pl2); P24 = mat_free(P24);
*/
		P1 = mat_free(P1);

		m = MAX(c1, MAX(c2, MAX(c3, c4)));
	}
	return m;
}

int RecoverMotion (matrix *E, homo2 *h1, homo2 *h2, int num, matrix **P2)
{
    matrix *P1, *P21, *P22, *P23, *P24;
	int c1, c2, c3, c4;
	int m = 0;

	if (CheckMatrix(E) && h1 && h2 && (num > 0)) {
		RecoverCameras (E, &P21, &P22, &P23, &P24);

		P1 = NormalizedCameraMatrix ();

		c1 = CountInFront(P1, P21, E, h1, h2, num);
		c2 = CountInFront(P1, P22, E, h1, h2, num);
		c3 = CountInFront(P1, P23, E, h1, h2, num);
		c4 = CountInFront(P1, P24, E, h1, h2, num);
/*
		c1 = CountInFront(P1, P21, h1, h2, num);
		c2 = CountInFront(P1, P22, h1, h2, num);
		c3 = CountInFront(P1, P23, h1, h2, num);
		c4 = CountInFront(P1, P24, h1, h2, num);
*/
		P1 = mat_free(P1);

		m = MAX(c1, MAX(c2, MAX(c3, c4)));

		if (m == c1) {
            *P2 = P21; P22 = mat_free(P22); P23 = mat_free(P23); P24 = mat_free(P24); return c1;
		}
		if (m == c2) {
            *P2 = P22; P21 = mat_free(P21); P23 = mat_free(P23); P24 = mat_free(P24); return c2;
		}
		if (m == c3) {
            *P2 = P23; P21 = mat_free(P21); P22 = mat_free(P22); P24 = mat_free(P24); return c3;
		}
		if (m == c4) {
            *P2 = P24; P21 = mat_free(P21); P22 = mat_free(P22); P23 = mat_free(P23); return c4;
		}
	}
	return 0;
}

int RecoverMotion (matrix *E, plist *pl1, plist *pl2, matrix **P2)
{
    matrix *P1, *P21, *P22, *P23, *P24;
	int c1, c2, c3, c4;
	int m;

	if (CheckMatrix(E) && pl1 && pl2) {
		RecoverCameras(E, &P21, &P22, &P23, &P24);

		P1 = NormalizedCameraMatrix ();

		c1 = CountInFront(P1, P21, E, pl1, pl2);
		c2 = CountInFront(P1, P22, E, pl1, pl2);
		c3 = CountInFront(P1, P23, E, pl1, pl2);
		c4 = CountInFront(P1, P24, E, pl1, pl2);
/*
		c1 = CountInFront(P1, P21, pl1, pl2);
		c2 = CountInFront(P1, P22, pl1, pl2);
		c3 = CountInFront(P1, P23, pl1, pl2);
		c4 = CountInFront(P1, P24, pl1, pl2);
*/
		P1 = mat_free(P1);

		m = MAX(c1, MAX(c2, MAX(c3, c4)));
		if (m == c1) {
            *P2 = P21; P22 = mat_free(P22); P23 = mat_free(P23); P24 = mat_free(P24); return c1;
		}
		if (m == c2) {
            *P2 = P22; P21 = mat_free(P21); P23 = mat_free(P23); P24 = mat_free(P24); return c2;
		}
		if (m == c3) {
            *P2 = P23; P21 = mat_free(P21); P22 = mat_free(P22); P24 = mat_free(P24); return c3;
		}
		if (m == c4) {
            *P2 = P24; P21 = mat_free(P21); P22 = mat_free(P22); P23 = mat_free(P23); return c4;
		}
	}
	return 0;
}

// ===========================================================================
//
// ===========================================================================

homo2 VecToHomo2 (vektor *A)
{
    homo2 h;
    int l;

    if (!CheckVector (A)) return NOHOMO2;

    l = A->len;
    if (l == 3) {
        h.x = A->v[0];
        h.y = A->v[1];
        h.w = A->v[2];
    } else if (l == 2) {
        h.x = A->v[0];
        h.y = A->v[1];
        h.w = 1.0;
    } else {
        Warning ("VecToHomo2: illegal vektor size %d", l);
		return NOHOMO2;
    }
    return h;
}

// ===========================================================================
//
// ===========================================================================

homo3 VecToHomo3 (vektor *A)
{
    homo3 X;
    int l;

    if (!CheckVector (A)) return NOHOMO3;

    l = A->len;

    if (l == 4) {
        X.X = A->v[0];
        X.Y = A->v[1];
        X.Z = A->v[2];
        X.W = A->v[3];
    } else if (l == 3) {
        X.X = A->v[0];
        X.Y = A->v[1];
        X.Z = A->v[2];
        X.W = 1.0;
    } else {
        Warning ("VecToHomo3: illegal vektor size %d", l);
		return NOHOMO3;
    }
    return X;
}

// ===========================================================================
//
// ===========================================================================

homo2 VecToNormHomo2 (vektor *A)
{
    homo2 h;

    h = VecToHomo2 (A);
    NormHomo (&h);

    return h;
}

// ===========================================================================
//
// ===========================================================================

homo3 VecToNormHomo3 (vektor *A)
{
    homo3 X;

    X = VecToHomo3 (A);
    NormHomo (&X);

    return X;
}

// ===========================================================================
//
// ===========================================================================

vektor *HomoToVec (homo2 h)
{
    vektor *A;

    A = vec_alloc (3);
    A->v[0] = h.x;
    A->v[1] = h.y;
    A->v[2] = h.w;

    return A;
}

// ===========================================================================
//
// ===========================================================================

vektor *HomoToVec (homo3 *X)
{
    vektor *A;

    A = vec_alloc (4);
    A->v[0] = X->X;
    A->v[1] = X->Y;
    A->v[2] = X->Z;
    A->v[3] = X->W;

    return A;
}

// ===========================================================================
//
// ===========================================================================

void PointToHomo (plist *pl, homo2 **h, int *num)
{
    pelem *p;
    int i;

    *num = pl->num;

    if (*num < 1) {
        Error ("PointToHomo2: No data !");
    }
    *h = (homo2 *) halloc (*num, sizeof (homo2));

    for (i=0, p=pl->start; p; p=p->next, ++i) {
        (*h)[i].x = p->x;
        (*h)[i].y = p->y;
        (*h)[i].w = 1.0;
        (*h)[i].id = p->id;
    }
}

// ===========================================================================
//
// ===========================================================================

void CoordToHomo (xlist *xl, homo3 **h, int *num)
{
    xelem *p;
    int i;

    *num = xl->num;

    if (*num < 1) {
        Error ("CoordToHomo3: No data !");
    }
    *h = (homo3 *) halloc (*num, sizeof (homo3));

    for (i=0, p=xl->start; p; p=p->next, ++i) {
        (*h)[i].X = p->X;
        (*h)[i].Y = p->Y;
        (*h)[i].Z = p->Z;
        (*h)[i].W = 1.0;
        (*h)[i].id = p->id;
    }
}

// ===========================================================================
//
// ===========================================================================

void VecPosToMat (vektor *A, int p, matrix *B)
{
    int i;

    CheckVector (A);
    CheckMatrix (B);

    if ((p < 0) || (p >= A->len)) {
        Error ("MatToVecPos: illegal position");
    }
    for (i = 0; i < MIN(A->len, B->rows * B->cols); ++i) {
        B->m[0][i] = A->v[p+i];
    }
}

// ===========================================================================
//
// ===========================================================================

void VecToMat (vektor *A, matrix *B)
{
    VecPosToMat (A, 0, B);
}

// ===========================================================================
//
// ===========================================================================

void VecToNormMat (vektor *A, matrix *B)
{
    VecToMat (A, B);
    NormMatrix (B);
}

// ===========================================================================
//
// ===========================================================================

void MatToVecPos (matrix *A, vektor *B, int p)
{
    int i;

    CheckMatrix (A);
    CheckVector (B);

    if ((p < 0) || (p >= B->len)) {
        Error ("MatToVecPos: illegal position");
    }
    for (i = 0; i < MIN(A->rows * A->cols, B->len); ++i) {
         B->v[i+p] = A->m[0][i];
    }
}

// ===========================================================================
//
// ===========================================================================

void MatToVec (matrix *A, vektor *B)
{
    MatToVecPos (A, B, 0);
}

// ===========================================================================
//
// ===========================================================================

void MatchToPoints (MatchList *ml, plist **pl1, plist **pl2, plist **pl3)
{
    MatchedPoint *p;
    long i;

    *pl1 = *pl2 = *pl3 = NULL;
	if (ml) {
		for (i=0; i<ml->size; ++i) {
			p = &(ml->points[i]);
            *pl1 = AddPoint (*pl1, p->X[0], p->Y[0], i, (int)p->color);
			*pl2 = AddPoint (*pl2, p->X[1], p->Y[1], i, (int)p->color);
			*pl3 = AddPoint (*pl3, p->X[2], p->Y[2], i, (int)p->color);
		}
	}
}


// ===========================================================================
//   Adapted version of hqr
//   Copyright (C) 1986-2002 Numerical Recipes Software.
//   P.O. Box 380243, Cambridge, MA 02238 (USA), http://www.nr.com/
// ===========================================================================

static bool hqr (double **a, int n, double *wr, double *wi)
{
    int nn, m, l, k, j, its, i, mmin;
    double z, y, x, w, v, u, t = 0.0, s, r=0, q=0, p=0, anorm;

    anorm = fabs(a[1][1]);

    for (i=1; i<n; ++i) {
        for (j=(i-1); j<n; ++j) {
            anorm += fabs(a[i][j]);
        }
    }
    nn = n-1;

    while (nn >= 0) {
        its = 0;
        do {
            for (l=nn; l>0; l--) {
                s = fabs(a[l-1][l-1]) + fabs(a[l][l]);
                if (ISZERO(s)) {
                    s = anorm;
                }
                if (ISZERO((double)(fabs(a[l][l-1]) + s) - s)) break;
            }
            x = a[nn][nn];
            if (l == nn) {
                wr[nn] = x + t;
                wi[nn--] = 0.0;
            } else {
                y = a[nn-1][nn-1];
                w = a[nn][nn-1] * a[nn-1][nn];
                if (l == (nn-1)) {
                    p = 0.5 * (y - x);
                    q = SQR(p)+w;
                    z = sqrt(fabs(q));
                    x += t;
                    if (q >= 0.0) {
                        z = p + SIGN(z,p);
                        wr[nn-1] = wr[nn] = x + z;
                        if ((ISNONZERO(z))) {
                            wr[nn] = x - w / z;
                        }
                        wi[nn-1] = wi[nn] = 0.0;
                    } else {
                        wr[nn-1] = wr[nn] = x + p;
                        wi[nn-1] = -(wi[nn] = z);
                    }
                    nn -= 2;
                } else {
                    if (its == 30) {
 						return false;
                    }
                    if (its == 10 || its == 20) {
                        t += x;
                        for (i=0; i<=nn; ++i) {
                            a[i][i] -= x;
                        }
                        s = fabs(a[nn][nn-1]) + fabs(a[nn-1][nn-2]);
                        y = x = 0.75 * s;
                        w = -0.4375 * s * s;
                    }
                    ++its;
                    for (m=(nn-2); m>=l; m--) {
                        z = a[m][m];
                        r = x - z;
                        s = y - z;
                        p = (r * s - w) / a[m+1][m] + a[m][m+1];
                        q = a[m+1][m+1] - z - r - s;
                        r = a[m+2][m+1];
                        s = fabs(p) + fabs(q) + fabs(r);
                        p /= s;
                        q /= s;
                        r /= s;
                        if (m == l) break;
                        u = fabs(a[m][m-1]) * (fabs(q) + fabs(r));
                        v = fabs(p) * (fabs(a[m-1][m-1]) + fabs(z) + fabs(a[m+1][m+1]));
                        if (ISZERO((double)(u+v) -v)) break;
                    }
                    for (i=m+2; i<=nn; ++i) {
                        a[i][i-2] = 0.0;
                        if (i != (m+2)) {
                            a[i][i-3] = 0.0;
                        }
                    }
                    for (k=m; k<=nn-1; ++k) {
                        if (k != m) {
                            p = a[k][k-1];
                            q = a[k+1][k-1];
                            r = 0.0;
                            if (k != (nn-1)) {
                                r = a[k+2][k-1];
                            }
                            x = fabs(p) + fabs(q) + fabs(r);
                            if ((ISNONZERO(x))) {
                                p /= x;
                                q /= x;
                                r /= x;
                            }
                        }
                        s = SIGN(sqrt(p*p + q*q + r*r), p);
                        if ((ISNONZERO(s))) {
                            if (k == m) {
                                if (l != m) {
                                    a[k][k-1] = -a[k][k-1];
                                }
                            } else {
                                a[k][k-1] = -s*x;
                            }
                            p += s;
                            x = p / s;
                            y = q / s;
                            z = r / s;
                            q /= p;
                            r /= p;
                            for (j=k; j<=nn; ++j) {
                                p = a[k][j] + q * a[k+1][j];
                                if (k != (nn-1)) {
                                    p += r * a[k+2][j];
                                    a[k+2][j] -= p * z;
                                }
                                a[k+1][j] -= p * y;
                                a[k][j] -= p * x;
                            }
                            mmin = nn<k+3 ? nn : k+3;
                            for (i=l; i<=mmin; ++i) {
                                p = x * a[i][k] + y * a[i][k+1];
                                if (k != (nn-1)) {
                                    p += z * a[i][k+2];
                                    a[i][k+2] -= p * r;
                                }
                                a[i][k+1] -= p * q;
                                a[i][k] -= p;
                            }
                        }
                    }
                }
            }
        } while (l < nn-1);
    }
    return true;
}

// ===========================================================================
//   Adapted version of zrhqr
//   Copyright (C) 1986-2002 Numerical Recipes Software.
//   P.O. Box 380243, Cambridge, MA 02238 (USA), http://www.nr.com/
// ===========================================================================

// ===========================================================================
//   Adapted version of zrhqr
//   Copyright (C) 1986-2002 Numerical Recipes Software.
//   P.O. Box 380243, Cambridge, MA 02238 (USA), http://www.nr.com/
// ===========================================================================

vektor *poly_root (vektor *A)
{
    int i, j, n, sol;
    double *res, *img;
    matrix *hess;
    vektor *B;
	bool ok;

    CheckVector (A);

    for (n = A->len-1; n > 0; n--) {
        if (ISNONZERO(A->v[n])) {
            break;
        }
    }
    if (n < 1) {                                     // degree = 0, no polynom
		Warning ("poly_root: illegal parameter !");
		return NULL;
    }
    if (n == 1) {                                                // degree = 1
        B = vec_alloc (1);
        B->v[0] = -A->v[0] / A->v[1];
        return B;
    }
    hess = mat_alloc (n, n);                                     // degree > 1

    for (j=1; j<=n; ++j) {
        hess->m[0][j-1] = -A->v[n-j] / A->v[n];
        for (i=1; i<n; ++i) {
            hess->m[i][j-1] = 0.0;
        }
        if (j != n) {
            hess->m[j][j-1] = 1.0;
        }
    }
    balanc (hess->m, n);

    res = (double *) halloc (n, sizeof(double));
    img = (double *) halloc (n, sizeof(double));

    ok = hqr (hess->m, n, res, img);
    hess = mat_free (hess);

	if (!ok) {
        Warning ("poly_root: Too many iterations!");
		if (res) {
			free (res); res = NULL;
		}
		if (img) {
			free (img); img = NULL;
		}
        return NULL;
	}
    for (i=0, sol=0; i<n; ++i) {                       // count double solutions
//		Debug("res[%d]: %f", i, res[i]);
//		Debug("img[%d]: %f", i, img[i]);
        if (ISZERO(img[i])) {
            sol++;
        }
    }
    if (sol < 1) {
        Warning ("poly_root: no double solution!");
		if (res) {
			free (res); res = NULL;
		}
		if (img) {
			free (img); img = NULL;
		}
        return NULL;
    }
    B = vec_alloc (sol);
    for (i=0, j=0; i<n; ++i) {
        if (ISZERO(img[i])) {
            B->v[j++] = res[i];
        }
    }
	if (res) {
		free (res); res = NULL;
	}
	if (img) {
		free (img); img = NULL;
	}
    return B;
}

// ===========================================================================
//

// ===========================================================================
//
// ===========================================================================

void EvalHomography (matrix *H, homo2 *h1, homo2 *h2, int num)
{
    vektor *res;
    int i;

    if (report == VERBOSE) {
		if (!H) {
			Debug ("Planar 2D Homography: empty!"); return;
		}
		mat_print ("Planar 2D Homography:", H);
		if (num < 1) {
			Debug ("EvalHomography: no points!"); return;
		}
        Debug ("Planar 2D Homography (using %d points):", num);

//      Debug ("Eval symmetric distance: d(x1, H^-1 x2)^2 + d(x2, H x1)^2:");
		Debug ("Eval distance: d(x', H x):");

        res = HomographyError (H, h1, h2, num);
		if (res) {
			InitError ();
			for (i=0; i<res->len; ++i) {
				AccumulateError (res->v[i]);
			}
			PrintError ();

			res = vec_free (res);
		}
    }
}

void EvalHomography (matrix *H, plist *pl1, plist *pl2)
{
    vektor *res;
    int i;

    if (report == VERBOSE) {
		if (!H) {
			Debug ("Planar 2D Homography: empty!"); return;
		}
		mat_print ("Planar 2D Homography:", H);
		if (!pl1 || !pl2 || pl1->num < 1 || pl2->num < 1) {
			Debug ("EvalHomography: no points!"); return;
		}
        Debug ("Planar 2D Homography (using %d points):", pl1->num);

//      Debug ("Eval symmetric distance: d(x1, H^-1 x2)^2 + d(x2, H x1)^2:");
		Debug ("Eval distance: d(x', H x):");

        res = HomographyError (H, pl1, pl2);
		if (res) {
			InitError ();
			for (i=0; i<res->len; ++i) {
				AccumulateError (res->v[i]);
			}
			PrintError ();

			res = vec_free (res);
		}
    }
}
//
// ===========================================================================

static matrix *FindHomography (homo2 *h1, homo2 *h2, int num)
{
    matrix *A, *H = NULL;
    vektor *V;
    int i, j;

    Debug ("Find 2D->2D Homography with basic DLT...");
    if (!h1 || !h2 || (num < 4)) {
        Warning ("need >= 4 point correspondences !"); return NULL;
    }
    A = mat_alloc (2*num, 9);

    for (j=0, i=0; i<num; ++i) {
		if (CheckID(&h1[i], &h2[i])) {
			A->m[j][0] =  h1[i].x * h2[i].w;
			A->m[j][1] =  h1[i].y * h2[i].w;
			A->m[j][2] =  h1[i].w * h2[i].w;
			A->m[j][6] = -h1[i].x * h2[i].x;
			A->m[j][7] = -h1[i].y * h2[i].x;
			A->m[j][8] = -h1[i].w * h2[i].x;
			j++;

			A->m[j][3] =  h1[i].x * h2[i].w;
			A->m[j][4] =  h1[i].y * h2[i].w;
			A->m[j][5] =  h1[i].w * h2[i].w;
			A->m[j][6] = -h1[i].x * h2[i].y;
			A->m[j][7] = -h1[i].y * h2[i].y;
			A->m[j][8] = -h1[i].w * h2[i].y;
			j++;
		}
    }
	if (mat_svd_nullvector (A, &V)) {
		H = mat_alloc (3, 3);
		VecToNormMat (V, H);
		V = vec_free (V);
	}
	A = mat_free (A);

    return H;
}

static matrix *FindHomography (plist *pl1, plist *pl2)
{
    matrix *A, *H = NULL;
    vektor *V;
	pelem *p, *q;
    int i;

    Debug ("Find 2D->2D Homography with basic DLT...");
    if (!pl1 || !pl2 || (pl1->num < 4) || (pl1->num != pl2->num)) {
        Warning ("need >= 4 point correspondences !"); return NULL;
    }
    A = mat_alloc (2*pl1->num, 9);

    for (i=0, p=pl1->start, q=pl2->start; p && q; p=p->next, q=q->next) {
		if (CheckID(p, q)) {
			A->m[i][0] =  p->x;
			A->m[i][1] =  p->y;
			A->m[i][2] =  1.0;
			A->m[i][6] = -p->x * q->x;
			A->m[i][7] = -p->y * q->x;
			A->m[i][8] =        -q->x;
			i++;

			A->m[i][3] =  p->x;
			A->m[i][4] =  p->y;
			A->m[i][5] =  1.0;
			A->m[i][6] = -p->x * q->y;
			A->m[i][7] = -p->y * q->y;
			A->m[i][8] =        -q->y;
			i++;
		}
    }
	if (mat_svd_nullvector (A, &V)) {
		H = mat_alloc (3, 3);
		VecToNormMat (V, H);
		V = vec_free (V);
	}
	A = mat_free (A);

    return H;
}

// ===========================================================================
//
// ===========================================================================

// ===========================================================================
//
// ===========================================================================

matrix *NormalizedHomography (homo2 *h1, homo2 *h2, int num)
{
    matrix *HN, *HD = NULL;
    homo2 *hn1, *hn2;
    trans t1, t2;

	CondHomo (h1, num, &hn1, &t1);
	CondHomo (h2, num, &hn2, &t2);

	HN = FindHomography (hn1, hn2, num);
	if (hn1) {
		free (hn1); hn1 = NULL;
	}
	if (hn2) {
		free (hn2); hn2 = NULL;
	}
	HD = DecondHomo (&t1, &t2, HN);
	HN = mat_free (HN);

	EvalHomography (HD, h1, h2, num);

    return HD;
}

matrix *NormalizedHomography (plist *pl1, plist *pl2)
{
    matrix *HN, *HD = NULL;
	plist *pn1, *pn2;
    trans t1, t2;

	CondHomo (pl1, &pn1, &t1);
	CondHomo (pl2, &pn2, &t2);

	HN = FindHomography (pn1, pn2);
	pn1 = FreePoint (pn1); pn2 = FreePoint (pn2);

	HD = DecondHomo (&t1, &t2, HN);
	HN = mat_free (HN);

	EvalHomography (HD, pl1, pl2);

    return HD;
}

// ===========================================================================
//
// ===========================================================================

static double min, max, sum;
static int num;

void InitError (void)
{
    min = MAX_REAL;
    max = 0.0;
    sum = 0.0;
    num = 0;
}

// ===========================================================================
//
// ===========================================================================

void AccumulateError (double err)
{
    min = MIN (min, err);
    max = MAX (max, err);
    sum += fabs(err);
    num++;
}

// ===========================================================================
//
// ===========================================================================

void PrintError (void)
{
    if (num) {
//		Debug ("\tData: %d points", num);
        Debug ("\tMin: %9.6f", min);
        Debug ("\tMax: %9.6f", max);
        Debug ("\tSum: %9.6f", sum);
        Debug ("\tAve: %9.6f", sum / (double)num);
    } else {
        Debug ("PrintError: No Data !");
    }
}

// ===========================================================================
//   d(x2, H x1)^2
// ===========================================================================

double CalcResidual (matrix *H, homo2 *h1, homo2 *h2)
{
	homo2 h;

	h = TransformHomo (H, h1);
    return HomoDistSqr (&h, h2);
}

double CalcResidual (matrix *H, pelem *p1, pelem *p2)
{
	homo2 h;

	h = TransformPointNew (H, p1);
    return HomoDistSqr (&h, p2);
}

// ===========================================================================
//   d(x, P X)^2
// ===========================================================================

double CalcResidual (matrix *P, homo3 *X, homo2 *x)
{
	homo2 h;

	h = Project (P, X);
    return HomoDistSqr (&h, x);
}

double CalcResidual (matrix *P, xelem *X, pelem *x)
{
	homo2 h;

	h = Project (P, X);
    return HomoDistSqr (&h, x);
}

// ===========================================================================
//   d(X2, H X1)^2
// ===========================================================================

double CalcResidual (matrix *H, homo3 *h1, homo3 *h2)
{
	homo3 X;

	X = TransformHomo (H, h1);
    return HomoDistSqr (&X, h2);
}

double CalcResidual (matrix *H, xelem *x1, xelem *x2)
{
	homo3 X;

	X = TransformCoordNew (H, x1);
    return HomoDistSqr (&X, x2);
}

// ===========================================================================
//   d(x1, H^-1 x2)^2 + d(x2, H x1)^2
// ===========================================================================

double CalcSymResidual (matrix *H, matrix *HI, homo2 *h1, homo2 *h2)
{
    return (CalcResidual (H, h1, h2) + CalcResidual (HI, h2, h1));
}

double CalcSymResidual (matrix *H, matrix *HI, pelem *p1, pelem *p2)
{
    return (CalcResidual (H, p1, p2) + CalcResidual (HI, p2, p1));
}

// ===========================================================================
//   d(X1, H^-1 X2)^2 + d(X2, H X1)^2
// ===========================================================================

double CalcSymResidual (matrix *H, matrix *HI, homo3 *h1, homo3 *h2)
{
    return (CalcResidual (H, h1, h2) + CalcResidual (HI, h2, h1));
}

double CalcSymResidual (matrix *H, matrix *HI, xelem *x1, xelem *x2)
{
    return (CalcResidual (H, x1, x2) + CalcResidual (HI, x2, x1));
}

// ===========================================================================
//   (x2^T F x1)^2 / (F x1)_1^2+(F x1)_2^2
// ===========================================================================

double CalcEpipolarDist (matrix *F, homo2 *h1, homo2 *h2)
{
    homo2 l;
    double div, r = 0.0;

    l = TransformNormHomo (F, h1);
    div = SQR(l.x) + SQR(l.y);

	if (ISNONZERO(div)) {
		r = SQR(HomoMult(h2, &l)) / div;
	}
    return r;
}

double CalcEpipolarDist (matrix *F, pelem *p1, pelem *p2)
{
    homo2 l;
    double div, r = 0.0;

    l = TransformPointNew (F, p1);
    div = SQR(l.x) + SQR(l.y);

	if (ISNONZERO(div)) {
		r = SQR(HomoMult(p2, &l)) / div;
	}
    return r;
}

// ===========================================================================
//  d(x2,F x1)^2 + d(x1,F^T x2)^2
//                           1                           1
//  (x2^T F x1)^2 ----------------------- + ---------------------------
//                (F x1)_1^2 + (F x1)_2^2   (F^T x2)_1^2 + (F^T x2)_2^2
// ===========================================================================

double CalcSymEpipolarDist (matrix *F, matrix *FT, homo2 *h1, homo2 *h2)
{
    return (CalcEpipolarDist (F, h1, h2) + CalcEpipolarDist (FT, h2, h1));
}

double CalcSymEpipolarDist (matrix *F, matrix *FT, pelem *p1, pelem *p2)
{
    return (CalcEpipolarDist (F, p1, p2) + CalcEpipolarDist (FT, p2, p1));
}

// ===========================================================================
//   d(x,P X)
// ===========================================================================

double CalcReproject (homo2 *x, matrix *P, homo3 *X)
{
	homo2 h;

	h = Project (P, X);
    return HomoDist (x, &h);
}

double CalcReproject (homo2 *x, matrix *P, xelem *X)
{
	homo2 h;

	h = Project (P, X);
    return HomoDist (x, &h);
}

double CalcReproject (pelem *p, matrix *P, homo3 *X)
{
	homo2 h;

	h = Project (P, X);
    return HomoDist (p, &h);
}


double CalcReproject (pelem *p, matrix *P, xelem *X)
{
	homo2 h;

	h = Project (P, X);
    return HomoDist (p, &h);
}

// ===========================================================================
//   d(x,PX)^2 + d(x',P'X)^2
// ===========================================================================

double CalcSymReproject (homo2 *x1, homo2 *x2, matrix *P1, matrix *P2, homo3 *X)
{
    return (CalcReproject (x1, P1, X) + CalcReproject (x2, P2, X));
}

double CalcSymReproject (homo2 *x1, homo2 *x2, matrix *P1, matrix *P2, xelem *X)
{
    return (CalcReproject (x1, P1, X) + CalcReproject (x2, P2, X));
}

double CalcSymReproject (pelem *p1, pelem *p2, matrix *P1, matrix *P2, homo3 *X)
{
    return (CalcReproject (p1, P1, X) + CalcReproject (p2, P2, X));
}

double CalcSymReproject (pelem *p1, pelem *p2, matrix *P1, matrix *P2, xelem *X)
{
    return (CalcReproject (p1, P1, X) + CalcReproject (p2, P2, X));
}

// ===========================================================================
//   |d(x,P X)|  mit (k1, k2, k3)
// ===========================================================================

double CalcRadialDist (homo2 *x, camera *cam, homo3 *X)
{
	homo2 r;

	r = Project (cam, X);

    return HomoDist (x, &r);
}

double CalcRadialDist (pelem *p, camera *cam, xelem *X)
{
	homo2 r;

	r = Project (cam, X);

    return HomoDist (p, &r);
}

// ===========================================================================
//
// ===========================================================================

void CalcTriFocalDist (cube *C, homo2 *x1, homo2 *x2, homo2 *x3, double *r)
{
    homo3 X;

	if (CheckCube (C) && r) {
		if (C->P[0] && C->P[1] && C->P[2]) {
			if (Triangulation (C->P[0], C->P[1], C->P[2], x1, x2, x3, &X)) {
			    *r = (CalcReproject (x1, C->P[0], &X) + CalcReproject (x2, C->P[1], &X) + CalcReproject (x3, C->P[2], &X))/3.0;
				return;
			}
		} else {
			Warning ("CalcTriFocalDist: Missing Cameras");
		}
	}
	if (r) {
		*r = MAX_REAL;
	}
}

void CalcTriFocalDist (cube *C, pelem *p1, pelem *p2, pelem *p3, double *r)
{
    homo3 X;

	if (CheckCube (C) && r) {
		if (C->P[0] && C->P[1] && C->P[2]) {
			if (Triangulation (C->P[0], C->P[1], C->P[2], p1, p2, p3, &X)) {
			    *r = (CalcReproject (p1, C->P[0], &X) + CalcReproject (p2, C->P[1], &X)+ CalcReproject (p3, C->P[2], &X))/3.0;
				return;
			}
		} else {
			Warning ("CalcTriFocalDist: Missing Cameras");
		}
	}
	if (r) {
		*r =MAX_REAL;
	}
}

// ===========================================================================
//
// ===========================================================================
// ===========================================================================

// ===========================================================================
//
// ===========================================================================

vektor *HomographyError (matrix *H, homo2 *h1, homo2 *h2, int num)
{
    matrix *HI;
    vektor *res = NULL;
    double r;
    int i;

	if (CheckSizeMatrix(H, 3, 3) && h1 && h2 && (num > 0)) {
		res = vec_alloc (num);

		HI = mat_invert_new (H);

		for (i=0; i<num; ++i) {
			r = CalcSymResidual(H, HI, &h1[i], &h2[i]);
//			res->v[i] = fabs(r);
			res->v[i] = sqrt(fabs(r)) * 0.5;
		}
		HI = mat_free (HI);
	}
    return res;
}
vektor *HomographyError (matrix *H, plist *pl1, plist *pl2)
{
    matrix *HI;
    vektor *res = NULL;
    double r;
	pelem *p, *q;
    int i;

	if (CheckSizeMatrix(H, 3, 3) && pl1 && pl2 && (pl1->num > 0) && (pl1->num == pl2->num)) {
		res = vec_alloc (pl1->num);

		HI = mat_invert_new (H);

		for (i=0, p=pl1->start, q=pl2->start; p && q && (i < pl1->num); p=p->next, q=q->next, ++i) {
			r = CalcSymResidual (H, HI, p, q);
//			res->v[i] = fabs(r);
			res->v[i] = sqrt(fabs(r)) * 0.5;
		}
		HI = mat_free (HI);
	}
    return res;
}

// ===========================================================================
//
// ===========================================================================

vektor *HomographyError (matrix *H, homo3 *h1, homo2 *h2, int num)
{
    vektor *res = NULL;
    double r;
    int i;

	if (CheckSizeMatrix(H, 3, 4) && h1 && h2 && (num > 0)) {
		res = vec_alloc (num);

		for (i=0; i<num; ++i) {
			r = CalcResidual(H, &h1[i], &h2[i]);
//			res->v[i] = fabs(r);
			res->v[i] = sqrt(fabs(r));
		}
	}
    return res;
}

vektor *HomographyError (matrix *H, xlist *xl, plist *pl)
{
    vektor *res = NULL;
	xelem *p;
	pelem *q;
    double r;
    int i;

	if (CheckSizeMatrix(H, 3, 4) && xl && pl && (xl->num > 0) && (xl->num == pl->num)) {
		res = vec_alloc (xl->num);

		for (i=0, p=xl->start, q=pl->start; p && q && (i < xl->num); p=p->next, q=q->next, ++i) {
			r = CalcResidual (H, p, q);
//			res->v[i] = fabs(r);
			res->v[i] = sqrt(fabs(r));
		}
	}
    return res;
}

// ===========================================================================
//
// ===========================================================================
// ===========================================================================
//
// ===========================================================================

vektor *HomographyError (matrix *H, homo3 *h1, homo3 *h2, int num)
{
    matrix *HI;
    vektor *res = NULL;
    double r;
    int i;

	if (CheckSizeMatrix(H, 4, 4) && h1 && h2 && (num > 0)) {
		res = vec_alloc (num);

		HI = mat_invert_new (H);

		for (i=0; i<num; ++i) {
			r = CalcSymResidual(H, HI, &h1[i], &h2[i]);
//			res->v[i] = fabs(r);
			res->v[i] = sqrt(fabs(r)) * 0.5;
		}
		HI = mat_free (HI);
	}
    return res;
}

vektor *HomographyError (matrix *H, xlist *xl, xlist *xel)
{
    matrix *HI;
    vektor *res = NULL;
    double r;
	xelem *p, *q;
    int i;

	if (CheckSizeMatrix(H, 4, 4) && xl && xel && (xl->num > 0) && (xl->num == xel->num)) {
		res = vec_alloc (xl->num);

		HI = mat_invert_new (H);

		for (i=0, p=xl->start, q=xel->start; p && q && (i < xl->num); p=p->next, q=q->next, ++i) {
			r = CalcSymResidual(H, HI, p, q);
//			res->v[i] = fabs(r);
			res->v[i] = sqrt(fabs(r)) * 0.5;
		}
		HI = mat_free (HI);
	}
    return res;
}

// ===========================================================================
//
// ===========================================================================

void EvalHomography (matrix *H, xlist *xl, xlist *xel)
{
    vektor *res;
    int i;

    if (report == VERBOSE) {
		if (!H) {
			Debug ("Spatial 3D Homography: empty!"); return;
		}
		mat_print ("Spatial 3D Homography:", H);
		if (!xl || !xel || xl->num < 1 || xel->num < 1) {
			Debug ("EvalHomography: no points!"); return;
		}
        Debug ("Spatial 3D Homography (using %d points):", xl->num);

//      Debug ("Eval symmetric distance: d(x, H^-1 x')^2 + d(x', H x)^2:");
        Debug ("Eval distance: d(X', H X):");

        res = HomographyError (H, xl, xel);
		if (res) {
			InitError ();
			for (i=0; i<res->len; ++i) {
				AccumulateError (res->v[i]);
			}
			PrintError ();

			res = vec_free (res);
		}
    }
}

// ===========================================================================
//
// ===========================================================================
// ===========================================================================
//
// ===========================================================================

void EvalHomography (matrix *P, homo3 *h1, homo2 *h2, int num)
{
    vektor *res;
    int i;

    if (report == VERBOSE) {
		if (!P) {
			Debug ("Projection Matrix: empty!"); return;
		}
		mat_print ("Projection Matrix:", P);
		if (num < 1) {
			Debug ("EvalHomography: no points!"); return;
		}
        Debug ("Projection Matrix (using %d points):", num);

		Debug ("Eval distance: d(x, P X):");

        res = HomographyError (P, h1, h2, num);
		if (res) {
			InitError ();
			for (i=0; i<res->len; ++i) {
				AccumulateError (res->v[i]);
			}
			PrintError ();

			res = vec_free (res);
		}
    }
}

void EvalHomography (matrix *P, xlist *xl, plist *pl)
{
    vektor *res;
    int i;

    if (report == VERBOSE) {
		if (!P) {
			Debug ("Projection Matrix: empty!"); return;
		}
		mat_print ("Projection Matrix:", P);
		if (!xl || !pl || xl->num < 1 || pl->num < 1) {
			Debug ("EvalHomography: no points!"); return;
		}
        Debug ("Projection Matrix (using %d points):", xl->num);

		Debug ("Eval distance: d(x, P X):");

        res = HomographyError (P, xl, pl);
		if (res) {
			InitError ();
			for (i=0; i<res->len; ++i) {
				AccumulateError (res->v[i]);
			}
			PrintError ();

			res = vec_free (res);
		}
    }
}

// ===========================================================================
//
// ===========================================================================
// ===========================================================================
//
// ===========================================================================

static matrix *FindHomography (homo3 *h1, homo3 *h2, int num)
{
    matrix *A, *H = NULL;
    vektor *V;
	int i, j;

    Debug ("FindHomography 3D with basic DLT...");

    if (!h1 || !h2 || (num < 5)) {
		Warning ("FindHomography: need >= 5 point correspondences!"); return NULL;
    }
    A = mat_alloc (3*num, 16);

    for (j=0, i=0; i<num; ++i) {
		if (CheckID(&h1[i], &h2[i])) {
			A->m[j][ 0] =  h1[i].X * h2[i].W;
			A->m[j][ 1] =  h1[i].Y * h2[i].W;
			A->m[j][ 2] =  h1[i].Z * h2[i].W;
			A->m[j][ 3] =  h1[i].W * h2[i].W;
			A->m[j][12] = -h1[i].X * h2[i].X;
			A->m[j][13] = -h1[i].Y * h2[i].X;
			A->m[j][14] = -h1[i].Z * h2[i].X;
			A->m[j][15] = -h1[i].W * h2[i].X;
			j++;

			A->m[j][ 4] =  h1[i].X * h2[i].W;
			A->m[j][ 5] =  h1[i].Y * h2[i].W;
			A->m[j][ 6] =  h1[i].Z * h2[i].W;
			A->m[j][ 7] =  h1[i].W * h2[i].W;
			A->m[j][12] = -h1[i].X * h2[i].Y;
			A->m[j][13] = -h1[i].Y * h2[i].Y;
			A->m[j][14] = -h1[i].Z * h2[i].Y;
			A->m[j][15] = -h1[i].W * h2[i].Y;
			j++;

			A->m[j][ 8] =  h1[i].X * h2[i].W;
			A->m[j][ 9] =  h1[i].Y * h2[i].W;
			A->m[j][10] =  h1[i].Z * h2[i].W;
			A->m[j][11] =  h1[i].W * h2[i].W;
			A->m[j][12] = -h1[i].X * h2[i].Z;
			A->m[j][13] = -h1[i].Y * h2[i].Z;
			A->m[j][14] = -h1[i].Z * h2[i].Z;
			A->m[j][15] = -h1[i].W * h2[i].Z;
			j++;
/*
			A->m[j][ 0] =  h1[i].X * h2[i].Y;
			A->m[j][ 1] =  h1[i].Y * h2[i].Y;
			A->m[j][ 2] =  h1[i].Z * h2[i].Y;
			A->m[j][ 3] =  h1[i].W * h2[i].Y;
			A->m[j][ 4] = -h1[i].X * h2[i].X;
			A->m[j][ 5] = -h1[i].Y * h2[i].X;
			A->m[j][ 6] = -h1[i].Z * h2[i].X;
			A->m[j][ 7] = -h1[i].W * h2[i].X;
			j++;

			A->m[j][ 0] =  h1[i].X * h2[i].Z;
			A->m[j][ 1] =  h1[i].Y * h2[i].Z;
			A->m[j][ 2] =  h1[i].Z * h2[i].Z;
			A->m[j][ 3] =  h1[i].W * h2[i].Z;
			A->m[j][ 8] = -h1[i].X * h2[i].X;
			A->m[j][ 9] = -h1[i].Y * h2[i].X;
			A->m[j][10] = -h1[i].Z * h2[i].X;
			A->m[j][11] = -h1[i].W * h2[i].X;
			j++;

			A->m[j][ 4] =  h1[i].X * h2[i].Z;
			A->m[j][ 5] =  h1[i].Y * h2[i].Z;
			A->m[j][ 6] =  h1[i].Z * h2[i].Z;
			A->m[j][ 7] =  h1[i].W * h2[i].Z;
			A->m[j][ 8] = -h1[i].X * h2[i].Y;
			A->m[j][ 9] = -h1[i].Y * h2[i].Y;
			A->m[j][10] = -h1[i].Z * h2[i].Y;
			A->m[j][11] = -h1[i].W * h2[i].Y;
			j++;
*/
		}
    }
	if (mat_svd_nullvector (A, &V)) {
		H = mat_alloc (4, 4);
		VecToNormMat (V, H);
		V = vec_free (V);
	}
	A = mat_free (A);

    return H;
}
// ===========================================================================
//
// ===========================================================================

void EvalHomography (matrix *H, homo3 *h1, homo3 *h2, int num)
{
    vektor *res;
    int i;

    if (report == VERBOSE) {
		if (!H) {
			Debug ("Spatial 3D Homography: empty!"); return;
		}
		mat_print ("Spatial 3D Homography:", H);
		if (num < 1) {
			Debug ("EvalHomography: no points!"); return;
		}
        Debug ("Spatial 3D Homography (using %d points):", num);

//      Debug ("Eval symmetric distance: d(x, H^-1 x')^2 + d(x', H x)^2:");
        Debug ("Eval distance: d(X', H X):");

        res = HomographyError (H, h1, h2, num);
		if (res) {
			InitError ();
			for (i=0; i<res->len; ++i) {
				AccumulateError (res->v[i]);
			}
			PrintError ();

			res = vec_free (res);
		}
    }
}


// ===========================================================================
//
// ===========================================================================

matrix *NormalizedHomography (homo3 *h1, homo3 *h2, int num)
{
    matrix *HN, *HD = NULL;
    homo3 *hn1, *hn2;
    trans t1, t2;

	CondHomo (h1, num, &hn1, &t1);
	CondHomo (h2, num, &hn2, &t2);
	HN = FindHomography (h1, h2, num);
	if (hn1) {
		free (hn1); hn1 = NULL;
	}
	if (hn2) {
		free (hn2); hn2 = NULL;
	}
	HD = DecondHomo (&t1, &t2, HN);
	HN = mat_free (HN);

	EvalHomography (HD, h1, h2, num);

    return HD;
}

static matrix *FindHomography (xlist *xl1, xlist *xl2)
{
    matrix *A, *H = NULL;
	xelem *p, *q;
    vektor *V;
	int i;

    Debug ("FindHomography 3D with basic DLT...");

    if (!xl1 || !xl2 || (xl1->num < 5) || (xl1->num != xl2->num)) {
		Warning ("FindHomography: need >= 5 point correspondences!"); return NULL;
    }
    A = mat_alloc (3*xl1->num, 16);

    for (i=0, p=xl1->start, q=xl2->start; p && q; p=p->next, q=q->next) {
		if (CheckID(p, q)) {
			A->m[i][ 0] =  p->X;
			A->m[i][ 1] =  p->Y;
			A->m[i][ 2] =  p->Z;
			A->m[i][ 3] =  1.0;
			A->m[i][12] = -p->X * q->X;
			A->m[i][13] = -p->Y * q->X;
			A->m[i][14] = -p->Z * q->X;
			A->m[i][15] =        -q->X;
			i++;

			A->m[i][ 4] =  p->X;
			A->m[i][ 5] =  p->Y;
			A->m[i][ 6] =  p->Z;
			A->m[i][ 7] =  1.0;
			A->m[i][12] = -p->X * q->Y;
			A->m[i][13] = -p->Y * q->Y;
			A->m[i][14] = -p->Z * q->Y;
			A->m[i][15] =        -q->Y;
			i++;

			A->m[i][ 8] =  p->X;
			A->m[i][ 9] =  p->Y;
			A->m[i][10] =  p->Z;
			A->m[i][11] =  1.0;
			A->m[i][12] = -p->X * q->Z;
			A->m[i][13] = -p->Y * q->Z;
			A->m[i][14] = -p->Z * q->Z;
			A->m[i][15] =        -q->Z;
			i++;
		}
    }
	if (mat_svd_nullvector (A, &V)) {
		H = mat_alloc (4, 4);
		VecToNormMat (V, H);
		V = vec_free (V);
	}
	A = mat_free (A);

    return H;
}

// ===========================================================================
//
// ===========================================================================

matrix *NormalizedHomography (xlist *xl1, xlist *xl2)
{
    matrix *HN, *HD = NULL;
    xlist *xn1, *xn2;
    trans t1, t2;

	CondHomo (xl1, &xn1, &t1);
	CondHomo (xl2, &xn2, &t2);
	HN = FindHomography (xn1, xn2);
	xn1 = FreeCoord (xn1); xn2 = FreeCoord (xn2);

	HD = DecondHomo (&t1, &t2, HN);
	HN = mat_free (HN);

	EvalHomography (HD, xl1, xl2);

    return HD;
}

// ===========================================================================
//
// ===========================================================================

void
EvalHomography (matrix *H, matrix *F)
{
    matrix *HI, *T, *T1, *T2;
	homo2 e1, e2, h;
    double t;

	if (report == VERBOSE) {
		if (CheckSizeMatrix(H,3,3) && CheckSizeMatrix(F,3,3)) {
			mat_print ("Compatible Homography:", H);

			CalcEpipoles (F, &e1, &e2);

			T = mat_transpose_new (H);
			T1 = mat_prod_new (T, F); T = mat_free (T);
			T = mat_transpose_new (F);
			T2 = mat_prod_new (T, H); T = mat_free (T);
			T = mat_add_new (T1, T2);
			T1 = mat_free (T1);
			T2 = mat_free (T2);
			mat_print ("H^T F + F^T H = 0:", T);
			T = mat_free (T);

			h = TransformNormHomo (H, &e1);
			t = HomoDist (&h, &e2);
			if (ISNONZERO(t)) {
				Warning ("H e1 = e2: %f", t);
			}
			HI = mat_invert_new (H);

			h = TransformNormHomo (HI, &e2);
			t = HomoDist (&h, &e1);
			if (ISNONZERO(t)) {
				Warning ("H^-1 e2 = e1: %f", t);
			}
			HI = mat_free (HI);

			if (ISZERO(mat_det (H))) {
				Warning ("det(H) = 0!");
			}
		}
	}
}

// ===========================================================================
//
// ===========================================================================
// ===========================================================================
//
// ===========================================================================

double RadialDistError (camera *cam, plist *pl, xlist *xl)
{
    pelem *q;
    xelem *p;
    double r, res = 0.0;

    Debug ("Radial Distortion Error: image distance [pix]");

	if (!cam || !CheckSizeMatrix(cam->P, 3, 4)) return 0.0;

	if (pl && xl) {
		InitError ();
		for (p=xl->start, q=pl->start; p && q; p=p->next, q=q->next) {
			if (CheckID(p,q)) {
				r = CalcRadialDist (q, cam, p);
				res += r;
				AccumulateError (r);
			}
		}
		PrintError ();
	}
    return res;
}

// ===========================================================================
//
// ===========================================================================

homo2 RadialDist (camera *cam, homo2 *p)
{
    homo2 r;
	double z, d, x, y;

	if (!cam || !p) return NOHOMO2;

	NormHomo (p);

	x = p->x - cam->s.x;
	y = p->y - cam->s.y;
	z = SQR(x) + SQR(y);

	d = 1.0 + cam->k1 * z + cam->k2 * SQR(z) + cam->k3 * z*SQR(z);
	r.x = cam->s.x + x * d;
	r.y = cam->s.y + y * d;
	r.w = 1.0;
	r.id = 0;

    return r;
}

// ===========================================================================
//
// ===========================================================================

homo2 Project (camera *cam, homo3 *X)
{
	homo2 p;

	if (!cam) return NOHOMO2;

    p = Project (cam->P, X);

    return RadialDist (cam, &p);
}

homo2 Project (camera *cam, xelem *X)
{
	homo2 p;

	if (!cam) return NOHOMO2;

    p = Project (cam->P, X);

    return RadialDist (cam, &p);
}

// ===========================================================================
//
// ===========================================================================

void UpdateSymmetryPoint (camera *cam)
{
	matrix *R, *K;

	if (cam && CheckSizeMatrix(cam->P, 3, 4)) {
		DecomposeCamera (cam->P, &K, &R);
		R = mat_free (R);
		if (K) {
			cam->s.x = K->m[0][2];
			cam->s.y = K->m[1][2];
			K = mat_free (K);
		} else {
			cam->s.x = cam->s.y = 0.0;                  // Eigentlich Bildmitte !!
		}
		cam->s.w = 1.0;
		cam->s.id = 0;
	}
}

// ===========================================================================
//
// ===========================================================================
void PrintCameraMatrix (camera *cam)
{
	if (cam) {
	    PrintCameraMatrix(cam->P);

		Report("    Symmetry Point  xs  : %7.2f [pix]", cam->s.x);
		Report("                    ys  : %7.2f [pix]", cam->s.y);
		Report("    Radial Distortion:");
		Report("\tk1    = % e", cam->k1);
		Report("\tk2    = % e", cam->k2);
		Report("\tk3    = % e\n", cam->k3);
	}
}

