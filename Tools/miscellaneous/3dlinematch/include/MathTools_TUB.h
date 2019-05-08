
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
#ifndef MathTools_H
#define MathTools_H

///Math stuff from Matthias Hinrichs and V. Rodehorst, TUM
#include "defs_TUB.h"
//#include "types_TUB.h"
// ===========================================================================
//                                   canny.cpp
// ===========================================================================
matrix* NonMaxSuppression (image *input, double sigma);
matrix *CannyEdge (image *input, double sigma, double lowThresh, double highThresh);

// ===========================================================================
//                                   check.cpp
// ===========================================================================
bool    CheckColIndex (matrix *M, int i);
bool    CheckColIndex (mesh *M, int i);
bool    CheckCompatibleMatrix (matrix *A, matrix *B);
bool    CheckColorImage (image *pic);
bool    CheckCompatible (matrix *A, matrix *B);
bool    CheckCompatible (matrix *A, vektor *B);
bool    CheckCompatible (vektor *A, matrix *B);
bool    CheckCoords (xelem *x);
bool    CheckCoords (xlist *xl);
bool    CheckCoordsSorted (xlist *xl);
bool    CheckCorners (celem *c);
bool    CheckCorners (clist *cl);
bool    CheckCornerValueSorted (clist *cl);
bool    CheckCube (cube *C);
bool    CheckDoubleGene (gelem *g, int size);
bool    CheckFile(char *file, filePermission perm);
bool    CheckFiniteMatrix (matrix *M);
bool    CheckGrayImage (image *pic);
bool    CheckID (homo2 *p, homo3 *x);
bool    CheckID (homo2 *p, xelem *x);
bool    CheckID (homo2 *p1, homo2 *p2);
bool    CheckID (homo2 *p1, pelem *p2);
bool    CheckID (homo3 *x, homo2 *p);
bool    CheckID (homo3 *x, pelem *p);
bool    CheckID (homo3 *x1, homo3 *x2);
bool    CheckID (homo3 *x1, xelem *x2);
bool    CheckID (pelem *p, homo3 *x);
bool    CheckID (pelem *p, xelem *x);
bool    CheckID (pelem *p1, homo2 *p2);
bool    CheckID (pelem *p1, pelem *p2);
bool    CheckID (xelem *x, homo2 *p);
bool    CheckID (xelem *x, pelem *p);
bool    CheckID (xelem *x1, homo3 *x2);
bool    CheckID (xelem *x1, xelem *x2);
bool    CheckID (homo2 *h[], int num);
bool    CheckID (homo3 *h[], int num);
bool    CheckID (pelem p[], int num);
bool    CheckID (pelem *p[], int num);
bool    CheckID (xelem x[], int num);
bool    CheckID (xelem *x[], int num);
bool    CheckImage (image *pic);
bool    CheckIndex (vektor *A, int i);
bool    CheckIndex (clist *cl, int i);
bool    CheckIndex (plist *pl, int i);
bool    CheckIndex (xlist *xl, int i);
bool    CheckIndex (matrix *M, int r, int c);
bool    CheckMask (mask *M);
bool    CheckMatrix (matrix *M);
bool    CheckMesh (mesh *M);
bool	CheckMeshSorted (mesh *M);
double    CheckOrientation (matrix *P, homo3 *X);
double    CheckOrientation (matrix *P, xelem *X);
bool    CheckPoints (pelem *p);
bool    CheckPoints (plist *pl);
bool    CheckPointsSorted (plist *pl);
bool    CheckPosSymMatrix (matrix *M);
bool    CheckRowIndex (matrix *M, int i);
bool    CheckRowIndex (mesh *M, int i);
bool    CheckSameCoords (xlist *xl1, xlist *xl2);
bool    CheckSameImage (image *pic1, image *pic2);
bool    CheckSameMatrix (matrix *A, matrix *B);
bool    CheckSamePoints (plist *pl1, plist *pl2);
bool    CheckSameVector (vektor *A, vektor *B);
bool    CheckSizeCoords (xlist *xl, int num);
bool    CheckSizeImage (image *pic1, image *pic2);
bool    CheckSizeMatrix (matrix *M, int rows, int cols);
bool    CheckSizePoints (plist *pl, int num);
bool    CheckSizeVector (vektor *V, int len);
bool    CheckSquareImage (image *pic);
bool    CheckSquareMatrix (matrix *M);
bool    CheckSymMatrix (matrix *M);
bool    CheckTypImage (image *pic1, image *pic2);
bool    CheckUnifiedID (plist *pl, xlist *xl);
bool    CheckUnifiedID (plist *pl1, plist *pl2);
bool    CheckUnifiedID (xlist *xl, plist *pl);
bool    CheckUnifiedID (xlist *xl1, xlist *xl2);
bool    CheckVector (vektor *V);
double    CheckZero (double t);


// ===========================================================================
//                                  coord.cpp
// ===========================================================================
xlist  *AddCoord (xlist *list, homo3 *X);
xlist  *AddCoord (xlist *list, homo3 *X, int color);
xlist  *AddCoord (xlist *list, homo3 *X, int color, double quality);
xlist  *AddCoord (xlist *list, homo3 *X, double quality);
xlist  *AddCoord (xlist *list, double x, double y, double z, long id);
xlist  *AddCoord (xlist *list, double x, double y, double z, long id, int color);
xlist  *AddCoord (xlist *list, double x, double y, double z, long id, int color, double quality);
xlist  *AddCoord (xlist *list, double x, double y, double z, long id, double quality);
xlist  *AddCoord (xlist *list, xelem *X);
xelem  *CloneCoord (xelem *X);
xlist  *CloneCoord (xlist *xl);
xlist  *CloneCoord (xlist *xl, int s, int e);
xlist  *CloneCoordArray (xlist *xl);
xlist  *CloneCoordArray (xlist *xl, int s, int e);
xlist  *CloneCoordList (xlist *xl);
xlist  *CloneCoordList (xlist *xl, int s, int e);
homo3   CoordAbs (homo3 *x);
homo3   CoordAbs (xelem *x);
void    CoordAbs (xlist *xl);
void    CoordAbs (xlist *xl1, xlist *xl2);
xlist  *CoordAbsNew (xlist *xl);
homo3   CoordAdd (homo3 *h, double s);
homo3   CoordAdd (xelem *x, double s);
void    CoordAdd (xlist *xl, double s);
void    CoordAdd (xlist *xl1, double s, xlist *xl2);
homo3   CoordAdd (homo3 *h1, homo3 *h2);
homo3   CoordAdd (homo3 *h1, xelem *h2);
homo3   CoordAdd (xelem *h1, homo3 *h2);
homo3   CoordAdd (xelem *h1, xelem *h2);
void    CoordAdd (xlist *xl1, xlist *xl2);
void    CoordAdd (xlist *xl1, xlist *xl2, xlist *xl3);
xlist  *CoordAddNew (xlist *xl, double s);
xlist  *CoordAddNew (xlist *xl1, xlist *xl2);
bool    CoordArray (xlist *xl);
void    CoordDiff (xlist *xl1, xlist *xl2);
double    CoordDist (homo3 *h1, homo3 *h2);
double    CoordDist (homo3 *h1, xelem *x2);
double    CoordDist (xelem *x1, homo3 *h2);
double    CoordDist (xelem *x1, xelem *x2);
double    CoordDistSqr (homo3 *h1, homo3 *h2);
double    CoordDistSqr (homo3 *h1, xelem *x2);
double    CoordDistSqr (xelem *x1, homo3 *h2);
double    CoordDistSqr (xelem *x1, xelem *x2);
homo3   CoordDiv (homo3 *h, double s);
homo3   CoordDiv (xelem *x, double s);
void    CoordDiv (xlist *xl, double s);
void    CoordDiv (xlist *xl1, double s, xlist *xl2);
homo3   CoordDiv (homo3 *h1, homo3 *h2);
homo3   CoordDiv (homo3 *h1, xelem *h2);
homo3   CoordDiv (xelem *h1, homo3 *h2);
homo3   CoordDiv (xelem *h1, xelem *h2);
void    CoordDiv (xlist *xl1, xlist *xl2);
void    CoordDiv (xlist *xl1, xlist *xl2, xlist *xl3);
xlist  *CoordDivNew (xlist *xl, double s);
xlist  *CoordDivNew (xlist *xl1, xlist *xl2);
double    CoordLen (homo3 *h);
double    CoordLen (xelem *x);
bool    CoordList (xlist *xl);
homo3   CoordMult (homo3 *h, double s);
homo3   CoordMult (xelem *x, double s);
void    CoordMult (xlist *xl, double s);
void    CoordMult (xlist *xl1, double s, xlist *xl2);
homo3   CoordMult (homo3 *h1, homo3 *h2);
homo3   CoordMult (homo3 *h1, xelem *h2);
homo3   CoordMult (xelem *h1, homo3 *h2);
homo3   CoordMult (xelem *h1, xelem *h2);
void    CoordMult (xlist *xl1, xlist *xl2);
void    CoordMult (xlist *xl1, xlist *xl2, xlist *xl3);
xlist  *CoordMultNew (xlist *xl, double s);
xlist  *CoordMultNew (xlist *xl1, xlist *xl2);
double    CoordProd (homo3 *h1, homo3 *h2);
double    CoordProd (homo3 *h1, xelem *h2);
double    CoordProd (xelem *h1, homo3 *h2);
double    CoordProd (xelem *h1, xelem *h2);
homo3   CoordSqr (homo3 *x);
homo3   CoordSqr (xelem *x);
void    CoordSqr (xlist *xl);
void    CoordSqr (xlist *xl1, xlist *xl2);
xlist  *CoordSqrNew (xlist *xl);
homo3   CoordSqrt (homo3 *x);
homo3   CoordSqrt (xelem *x);
void    CoordSqrt (xlist *xl);
void    CoordSqrt (xlist *xl1, xlist *xl2);
xlist  *CoordSqrtNew (xlist *xl);
homo3   CoordSub (homo3 *h, double s);
homo3   CoordSub (xelem *x, double s);
void    CoordSub (xlist *xl, double s);
void    CoordSub (xlist *xl1, double s, xlist *xl2);
homo3   CoordSub (homo3 *h1, homo3 *h2);
homo3   CoordSub (homo3 *h1, xelem *h2);
homo3   CoordSub (xelem *h1, homo3 *h2);
homo3   CoordSub (xelem *h1, xelem *h2);
void    CoordSub (xlist *xl1, xlist *xl2);
void    CoordSub (xlist *xl1, xlist *xl2, xlist *xl3);
xlist  *CoordSubNew (xlist *xl, double s);
xlist  *CoordSubNew (xlist *xl1, xlist *xl2);
void    CopyCoord (xelem *X, xelem *Y);
void    CopyCoord (xlist *xl, xlist *yl);
void    CopyCoord (xlist *xl, xlist *yl, int d);
void    CopyCoord (xlist *xl, int s, int e, xlist *yl);
void    CopyCoord (xlist *xl, int s, int e, xlist *yl, int d);
//int   CountCoord (xlist *xl, xlist *xel);
void    DeleteCoord (xlist *xl, xelem *X);
void    DeleteCoordID (xlist *pl, long id);
void    DeleteCoordIDs (xlist *pl, long id);
void    DeleteCoordPos (xlist *pl, long id);
xlist  *FreeCoord (xlist *xl);
xelem  *GetCoordID (xlist *xl, long id);
xelem  *GetCoordPos (xlist *xl, long pos);
xlist  *JoinCoord (xlist *xl, xlist *yl);
xlist  *JoinCoordNew (xlist *xl[], int num);
xlist  *JoinCoordNew (xlist *x1, xlist *x2);
xlist  *JoinCoordNew (xlist *x1, xlist *x2, xlist *x3);
xelem  *NewCoord (void);
xelem  *NewCoord (int num);
xlist  *NewCoordList (void);
xlist  *NewCoordList (int num);
void    PrintCoord (xelem *x);
void    PrintCoord (xlist *xl);
void    RelinkCoord (xlist *xl);
void    ScaleCoord (homo3 *h, double f);
void    SortCoord (xlist *xl);
void    SortCoordUnlinked (xlist *xl);
void    SwapCoord(xlist *xl1, xlist *xl2);
homo3   TransformCoord (matrix *H, homo3 *X);
void    TransformCoord (matrix *H, xelem *X);
void    TransformCoord (matrix *H, xlist *xl);
homo3   TransformCoordNew (matrix *H, xelem *X);
xlist  *TransformCoordNew (matrix *H, xlist *xl);
homo3   TransformNormCoord (matrix *H, homo3 *X);
homo3   TransformNormCoordNew (matrix *H, xelem *X);


// ===========================================================================
//                                   file.cpp
// ===========================================================================
void    ReadCoord (char *name, xlist **xl);
void    ReadCoord (char *name, xlist **xl, asciityp t);
void    ReadCorners (char *name, clist **clist);
cube   *ReadCube (char *name);
void    ReadMatches (char *name, plist **pl1, plist **pl2);
matrix *ReadMatrix (char *name);
mesh   *ReadMesh (char *name);
bool    ReadNewLine (FILE *fp);
image  *ReadPBM (char *name);
void    ReadPoints (char *name, plist **pl);
void    ReadPoints (char *name, plist **pl, asciityp t);
int     ReadString (FILE *fp, char *buf);
void    WriteCoord (char *name, xlist *xl);
void    WriteCoord (char *name, xlist *xl, asciityp t);
void    WriteCoord (char *name, xlist *xl, int skip);
void    WriteCoord (char *name, xlist *xl, asciityp t, int skip);
void    WriteCoordVRML (char *name, xlist *xl);
void    WriteCornerImage (char *name, image *pic, clist *list);
void    WriteCornerEllipseImage (char *name, image *pic, clist *list);
void    WriteCornerEllipseAxesImage (char *name, image *pic, clist *list);
void    WriteCorners (char *name, cparam *par, image *pic, clist *clist);
void    WriteCube (char *name, cube *C);
void    WriteDisp (char *fname, matrix *Disp, double min, double factor, bool markUnmatchedAreaRed);
void    WriteMatches (char *name, plist *pl1, plist *pl2);
void    WriteMatrix (char *name, matrix *M);
void    WriteMatrixMatlab (char *name, char*label,  matrix *M);
void    WriteMesh (char *name, mesh *M);
void    WritePBM (char *name, image *pic);
void    WritePoints (char *name, plist *pl);
void    WritePoints (char *name, plist *pl, asciityp t);
void    WritePoints (char *name, plist *pl, int skip);
void    WritePoints (char *name, plist *pl, asciityp t, int skip);
void    WriteReconstruction (char *name, matrix *P1, matrix *P2, matrix *P3, matrix *P4, xlist *xl, int w, int h);
void    WriteWaveFront (char *name, mesh *M);


// ===========================================================================
//                                  image.cpp
// ===========================================================================
image  *AllocImage (int w, int h, byte typ, char *name);
image  *ColorToGray (image *pic1);
void    FillImage (image *pic, int color);
image  *FreeImage (image *pic);
image  *GetPattern (image *pic, celem *c, int size);
image  *GetPattern (image *pic, pelem *p, int size);
image  *GrayToColor (image *pic);
void    ImageAdd (image *pic1, image *pic2, int col, matrix **r);
image  *ImageClone (image *pic);
void    ImageCopy (image *pic1, image *pic2);
void    ImageDiff (image *pic1, image *pic2, int col, matrix **r);
void    ImageDiffCol (image *pic1, image *pic2, matrix **r);
void    ImagePatch (int x, int y, image *pic1, image *pic2);
image  *ImageRotate (image *pic);
double    IntensityMeanLayer (image *pic, int col);
void    InvertImage (image *pic);
image  *JoinColor (image *red, image *green, image *blue);
void    JoinImages (image *pic1, image *pic2);
void    JoinImages (image *pic1, image *pic2, image *pic3);
void    NormalizeImage (image *pic);
void    SeperateColor (image *pic, image **red, image **green, image **blue);

// ===========================================================================
//                                 imgproc.cpp
// ===========================================================================
void    Convolution (image *in, int col, mask *kernel, image **out);
void    Convolution (image *in, int col, mask *kernel, image *out);
void    Convolution (image *pic, int col, mask *kernel);
void    Convolution (image *pic, int col, mask *kernel, matrix **mat);
void    Convolution (image *pic, int col, mask *kernel, matrix *mat);
void    Convolution (matrix *in, mask *kernel, matrix **out);
void    Convolution (matrix *in, mask *kernel, matrix *out);
void    Convolution (matrix *mat, mask *kernel);
void    Gauss (double sigma, image *in, image **out);
void    Gauss (double sigma, image *in, image *out);
void    Gauss (double sigma, image *pic);
void    Gauss (double sigma, int col, image *in, image **out);
void    Gauss (double sigma, int col, image *in, image *out);
void    Gauss (double sigma, int col, image *pic);
void    Gauss (double sigma, matrix *in, matrix **out);
void    Gauss (double sigma, matrix *in, matrix *out);
void    Gauss (double sigma, matrix *mat);
matrix *GetPatch (int x, int y, matrix *dat, int r);
void    Gradient (double sigma, image *pic, matrix **rx, matrix **ry, matrix **gx, matrix **gy, matrix **bx, matrix **by);
void    Gradient (double sigma, image *pic, matrix *rx, matrix *ry, matrix *gx, matrix *gy, matrix *bx, matrix *by);
void    Gradient (double sigma, int col, image *pic, matrix **gx, matrix **gy);
void    Gradient (double sigma, int col, image *pic, matrix *gx, matrix *gy);
void    Gradient (double sigma, matrix *mat, matrix *gx, matrix *gy);
void    Gradient (double sigma, matrix *pic, matrix **gx, matrix **gy);
double    GradientMean (matrix *gx, matrix *gy);
double    GradientProd (matrix *ax, matrix *ay, matrix *bx, matrix *by);
void    GradientVal (matrix *gx, matrix *gy, matrix **g);
image  *ImageScale (double factor, image *pic);
image  *ImageShift (region *roi, double x, double y, image *pic1);
image  *ImageUndistort (image *pic1, camera *cam);
void    Mean (int radius, image *in, image **out);
void    Mean (int radius, image *in, image *out);
void    Mean (int radius, image *pic);
void    Mean (int radius, int col, image *in, image **out);
void    Mean (int radius, int col, image *in, image *out);
void    Mean (int radius, int col, image *pic);
void    Mean (int radius, matrix *in, matrix **out);
void    Mean (int radius, matrix *in, matrix *out);
void    Mean (int radius, matrix *mat);
void    Sobel (image *pic, matrix **rx, matrix **ry, matrix **gx, matrix **gy, matrix **bx, matrix **by);
void    Sobel (image *pic, matrix *rx, matrix *ry, matrix *gx, matrix *gy, matrix *bx, matrix *by);
void    Sobel (int col, image *pic, matrix **gx, matrix **gy);
void    Sobel (int col, image *pic, matrix *gx, matrix *gy);
void    Sobel (matrix *mat, matrix *gx, matrix *gy);
void    Sobel (matrix *pic, matrix **gx, matrix **gy);
void    HistogramEqualization(image *pic);


// ===========================================================================
//                                   mask.cpp
// ===========================================================================
mask   *mask_alloc(int radius);
mask   *mask_alloc(double sigma);
mask   *mask_free(mask *M);
mask   *mask_gauss(double sigma);
mask   *mask_gradientX(double sigma);
mask   *mask_gradientY(double sigma);
mask   *mask_mean(int radius);
int     mask_radius(double sigma);
void    mask_scale(mask *kernel, double scale);
mask   *mask_sobelX(void);
mask   *mask_sobelY(void);
double    mask_sum(mask *kernel);
void    mask_transpose(mask *kernel);

// ===========================================================================
//                                  match.cpp
// ===========================================================================
MatchList *BuildMatchList(image** pics, matrix* Disp);
MatchList *ReadMatchedPoints (char *name);
void       WriteMatchedPoints(char *fname, MatchList *pointList);
MatchList *FreeMatch(MatchList *list);
MatchList *AllocMatch(int size);

// ===========================================================================
//                                  matrix.cpp
// ===========================================================================
matrix *homo_mat_cross (homo2 *a, matrix *B);
matrix *homo_mat_skew (homo2 *h);
void    mat_abs(matrix *A);
void    mat_abs(matrix *A, matrix *B);
matrix *mat_abs_new(matrix *A);
void	mat_add (matrix *A, double s);
void    mat_add (matrix *A, double s, matrix *B);
matrix *mat_add_new (matrix *A, double s);
void    mat_add (matrix *A, matrix *B);
void    mat_add (matrix *A, matrix *B, matrix *C);
matrix *mat_add_new (matrix *A, matrix *B);
matrix *mat_alloc (int rows, int cols);
matrix *mat_clone (matrix *A);
matrix *mat_clone(matrix *A, int sr, int sc, int er, int ec);
matrix *mat_combine (matrix *A, matrix *B, int row, int col);
void    mat_copy (matrix *A, matrix *B);
void    mat_copy (matrix *A, matrix *B, int r, int c);
void    mat_copy (matrix *A, int sr, int sc, int er, int ec, matrix *B);
void    mat_copy (matrix *A, int sr, int sc, int er, int ec, matrix *B, int dr, int dc);
void    mat_copycol (matrix *M, int m, matrix *N, int n);
void    mat_copyrow (matrix *M, int m, matrix *N, int n);
matrix *mat_cross (homo2 *a, matrix *B);
matrix *mat_cross (vektor *A, matrix *B);
double    mat_det (matrix *A);
matrix *mat_diag (vektor *A);
bool    mat_div (matrix *A, double s);
bool    mat_div (matrix *A, double s, matrix *B);
matrix *mat_div_new (matrix *A, double s);
void    mat_div (matrix *A, matrix *B);
void    mat_div (matrix *A, matrix *B, matrix *C);
matrix *mat_div_new (matrix *A, matrix *B);
matrix *mat_dyadic (homo2 *h1, homo2 *h2);
matrix *mat_dyadic (homo3 *h1, homo3 *h2);
matrix *mat_dyadic (pelem *p1, pelem *p2);
matrix *mat_dyadic (vektor *a, vektor *b);
matrix *mat_dyadic (xelem *x1, xelem *x2);
void    mat_fill (matrix *A, double s);
void    mat_fill (matrix *A, int sr, int sc, int er, int ec, double s);
void    mat_fillcol (matrix *M, int n, double s);
void    mat_fillrow (matrix *M, int n, double s);
matrix *mat_free (matrix *mat);
vektor *mat_getcol (matrix *M, int n);
vektor *mat_getrow (matrix *M, int n);
matrix *mat_identity (int s);
matrix *mat_identity (int r, int c);
matrix *mat_init(int row, ...);
bool    mat_invert (matrix *A);
bool    mat_invert(double *a, double *b, double *c);
bool    mat_invert(double *a, double *b, double *c, double *d);
matrix *mat_invert_alt (matrix *A);
matrix *mat_invert_new (matrix *A);
double    mat_max(matrix *A);
double    mat_maxabs(matrix *A);
double    mat_mean(matrix *A);
double    mat_meanabs (matrix *A);
double    mat_median(matrix *A);
double    mat_medianabs(matrix *A);
double    mat_min(matrix *A);
double    mat_minabs(matrix *A);
void    mat_minmax(matrix *A, double *min, double *max);
void    mat_minmaxabs(matrix *A, double *min, double *max);
void    mat_mult (matrix *A, double s);
void    mat_mult (matrix *A, double s, matrix *B);
matrix *mat_mult_new (matrix *A, double s);
void    mat_mult (matrix *A, matrix *B);
void    mat_mult (matrix *A, matrix *B, matrix *C);
matrix *mat_mult_new (matrix *A, matrix *B);
double    mat_norm (matrix *A); // Changed Name -> mat_normalize
double    mat_norm (matrix *A, norm p);
bool    mat_normalize(matrix *A);
bool	mat_normalize(matrix *A, norm p);
bool    mat_normalize(matrix *A, matrix *B);
bool    mat_normalize(matrix *A, norm p, matrix *B);
matrix *mat_normalize_new(matrix *A);
matrix *mat_normalize_new(matrix *A, norm p);
void    mat_normalizecol (matrix *M, int n, norm p);
void    mat_normalizecol (matrix *M, norm p);
void    mat_normalizerow (matrix *M, int m, norm p);
void    mat_normalizerow (matrix *M, norm p);
void    mat_patch (int x, int y, matrix *dat1, matrix *dat2);
void    mat_print (matrix *M);
void    mat_print (char *label, matrix *M);
void    mat_prod (matrix *A, matrix *B);
matrix *mat_prod_B_is_DIAGMAT (matrix *A, matrix *B);
matrix *mat_prod_new (matrix *A, matrix *B);
vektor *mat_pseudo_inverse (matrix *A, vektor *b);
void    mat_random (matrix *A);
void    mat_scale (matrix *A, double s);
void    mat_scale (matrix *A, double s, matrix *B);
matrix *mat_scale_new (matrix *A, double s);
void    mat_setcol (matrix *M, int n, vektor *V);
void    mat_setrow (matrix *M, int n, vektor *V);
matrix *mat_skew (homo2 *h);
matrix *mat_skew (vektor *V);
void    mat_sqr(matrix *A);
void    mat_sqr(matrix *A, matrix *B);
matrix *mat_sqr_new(matrix *A);
void    mat_sqrt(matrix *A);
void    mat_sqrt(matrix *A, matrix *B);
matrix *mat_sqrt_new(matrix *A);
double    mat_std(matrix *A);
double    mat_std(matrix *A, double *mean);
double    mat_std2(matrix *A);
double    mat_std2(matrix *A, double *mean);
void    mat_sort(matrix *A);
matrix *mat_sort_new(matrix *A);
void    mat_sub (matrix *A, double s);
void    mat_sub (matrix *A, double s, matrix *B);
matrix *mat_sub_new (matrix *A, double s);
void    mat_sub (matrix *A, matrix *B);
void    mat_sub (matrix *A, matrix *B, matrix *C);
matrix *mat_sub_new (matrix *A, matrix *B);
double    mat_sum(matrix *A);
double    mat_sumabs(matrix *A);
void	mat_swap (matrix *A, matrix *B);
void    mat_swap_cols (matrix *M, int col1, int col2);
void    mat_swap_rows (matrix *M, int row1, int row2);
int     mat_threscount (matrix *dat, double thres);
double    mat_trace(matrix *A);
void    mat_transpose (matrix *M);
matrix *mat_transpose_new (matrix *M);
double    mat_var(matrix *A);
double    mat_var(matrix *A, double *mean);
double    mat_var2(matrix *A);
double    mat_var2(matrix *A, double *mean);
//added by MG
void    mat_set (matrix *A, double s);
void  mat_eye (matrix *A);
void 	mat_set_at (matrix *A, int row, int col, double s);
double 	mat_get_at (matrix *M, int r, int c);
matrix *mat_stack_rows (matrix *A, matrix *B); //stacks the rows from B und the one from A (needed eg for F_from_P)
bool CheckMatrixIdentity(matrix *A, matrix *B, double tolerance);
bool CheckMatrixIdentityRelativeError(matrix *A, matrix *B, double tolerance);
int 	mat_rank (matrix *A);
void 	vec_set_at (vektor *A, int r, double s);
double 	vec_get_at (vektor *A, int r);
int mat_get_rows(matrix *M);
int mat_get_cols(matrix *M);


// ===========================================================================
//                                 mesh.cpp
// ===========================================================================
mesh   *mesh_alloc(int row, int col);
void    mesh_clean (mesh *M);
void    mesh_convolve(mesh *M, mask *kernel);
mesh   *mesh_free(mesh *M);
void    mesh_gauss(mesh *M, double sigma);
void    mesh_info(mesh *M);
void    mesh_mean(mesh *M, int r);
void	mesh_median(mesh *M, int r);
long    mesh_next(mesh *M, long id, long high);
long    mesh_previous(mesh *M, long id, long low);
xelem  *mesh_search(mesh *M, long id);
void    mesh_setid(mesh *M, int row, int col, long id);
void    mesh_setpoint(mesh *M, homo3 *X);
void    mesh_setpoint(mesh *M, homo3 *X, int color);
void    mesh_setpoint(mesh *M, homo3 *X, int color, double quality);
void    mesh_setpoint(mesh *M, homo3 *X, double quality);
void    mesh_setpoint(mesh *M, long id, double x, double y, double z);
void    mesh_setpoint(mesh *M, long id, double x, double y, double z, int color);
void    mesh_setpoint(mesh *M, long id, double x, double y, double z, int color, double quality);
void    mesh_setpoint(mesh *M, long id, double x, double y, double z, double quality);
void    mesh_setpoint(mesh *M, xelem *X);
void    mesh_setpoint(mesh *M, xlist *xl);
void    mesh_testall(void);

// ===========================================================================
//                                 message.cpp
// ===========================================================================
void    Debug (char *fmt, ...);
void    DebugLine (char *sym, int len);
void    Dump (char *fmt, ...);
void    Error (char *fmt, ...);
void    Report (char *fmt, ...);
void    ReportLine (char *sym, int len);
void    Warning (char *fmt, ...);

// ===========================================================================
//                                   norm.cpp
// ===========================================================================
matrix *CenterCamera (matrix *P, int w, int h);
void    CondHomo (homo2 *h, int num, homo2 **hn, trans *t);
void    CondHomo (plist *h, plist **hn, trans *t);
void    CondHomo (homo3 *h, int num, homo3 **hn, trans *t);
void    CondHomo (xlist *h, xlist **hn, trans *t);
matrix *DecondRelativeOrientation (trans *t1, trans *t2, matrix *F1);
matrix *DecondHomo (trans *t1, trans *t2, matrix *H1);
cube   *DecondTriFocal (trans *t1, trans *t2, trans *t3, cube *C);
void    EvalRotation (matrix *R);
matrix *InvSimilarityMatrix2 (trans *t);
matrix *InvSimilarityMatrix3 (trans *t);
bool    NormCamera (matrix *P);
bool    NormCube (cube *C);
bool    NormEssentialMatrix (matrix *A);
bool    NormMatrix (matrix *A);
bool    NormVector (vektor *A);
bool    NormHomo (homo2 *h);
bool    NormHomo (homo3 *X);
matrix *SimilarityMatrix2 (trans *t);
matrix *SimilarityMatrix3 (trans *t);
matrix *UncenterCamera (matrix *Q, int w, int h);


// ===========================================================================
//                                  point.cpp
// ===========================================================================
plist  *AddPoint (plist *list, homo2 *p);
plist  *AddPoint (plist *list, homo2 *p, int color);
plist  *AddPoint (plist *list, homo2 *p, int color, double quality);
plist  *AddPoint (plist *list, homo2 *p, double quality);
plist  *AddPoint (plist *list, pelem *p);
plist  *AddPoint (plist *list, double x, double y, long id);
plist  *AddPoint (plist *list, double x, double y, long id, int color);
plist  *AddPoint (plist *list, double x, double y, long id, int color, double quality);
plist  *AddPoint (plist *list, double x, double y, long id, double quality);
plist  *AddOnlyNewPointIDs(plist *list, plist *pointsToAdd);
pelem  *ClonePoint (pelem *p);
plist  *ClonePoint (plist *pl);
plist  *ClonePoint (plist *pl, int s, int e);
plist  *ClonePointArray (plist *pl);
plist  *ClonePointArray (plist *pl, int s, int e);
plist  *ClonePointList (plist *pl);
plist  *ClonePointList (plist *pl, int s, int e);
void    CopyPoint (pelem *p, pelem *q);
void    CopyPoint (plist *pl, plist *ql);
void    CopyPoint (plist *pl, plist *ql, int d);
void    CopyPoint (plist *pl, int s, int e, plist *ql);
void    CopyPoint (plist *pl, int s, int e, plist *ql, int d);
void    DeletePoint (plist *pl, pelem *p);
void    DeletePointID (plist *pl, long id);
void    DeletePointIDs (plist *pl, long id);
void    DeletePointPos (plist *pl, long pos);
plist  *FreePoint (plist *pl);
pelem  *GetPointID (plist *pl, long id);
pelem  *GetPointPos (plist *pl, long pos);
plist  *JoinPoint (plist *pl, plist *ql);
plist  *JoinPointNew (plist *pl[], int num);
plist  *JoinPointNew (plist *p1, plist *p2);
plist  *JoinPointNew (plist *p1, plist *p2, plist *p3);
pelem  *NewPoint (void);
pelem  *NewPoint (int num);
plist  *NewPointList (void);
plist  *NewPointList (int num);
homo2   PointAbs (homo2 *p);
homo2   PointAbs (pelem *p);
void    PointAbs (plist *pl);
void    PointAbs (plist *pl1, plist *pl2);
plist  *PointAbsNew (plist *pl);
homo2   PointAdd (homo2 *h, double s);
homo2   PointAdd (pelem *p, double s);
void    PointAdd (plist *pl, double s);
void    PointAdd (plist *pl1, double s, plist *pl2);
homo2   PointAdd (homo2 *h1, homo2 *h2);
homo2   PointAdd (homo2 *h1, pelem *h2);
homo2   PointAdd (pelem *h1, homo2 *h2);
homo2   PointAdd (pelem *h1, pelem *h2);
void    PointAdd (plist *pl1, plist *pl2);
void    PointAdd (plist *pl1, plist *pl2, plist *pl3);
plist  *PointAddNew (plist *pl, double s);
plist  *PointAddNew (plist *pl1, plist *pl2);
bool    PointArray (plist *pl);
homo2   PointCross (homo2 *h1, homo2 *h2);
homo2   PointCross (homo2 *h1, pelem *p2);
homo2   PointCross (pelem *p1, homo2 *h2);
homo2   PointCross (pelem *p1, pelem *p2);
homo2   PointCrossNorm (homo2 *h1, homo2 *h2);
homo2   PointCrossNorm (homo2 *h1, pelem *p2);
homo2   PointCrossNorm (pelem *p1, homo2 *h2);
homo2   PointCrossNorm (pelem *p1, pelem *p2);
double    PointDist (homo2 *h1, homo2 *h2);
double    PointDist (homo2 *h1, pelem *p2);
double    PointDist (pelem *p1, homo2 *h2);
double    PointDist (pelem *p1, pelem *p2);
double    PointDistSqr (homo2 *h1, homo2 *h2);
double    PointDistSqr (homo2 *h1, pelem *p2);
double    PointDistSqr (pelem *p1, homo2 *h2);
double    PointDistSqr (pelem *p1, pelem *p2);
homo2   PointDiv (homo2 *h, double s);
homo2   PointDiv (pelem *p, double s);
void    PointDiv (plist *pl, double s);
void    PointDiv (plist *pl1, double s, plist *pl2);
homo2   PointDiv (homo2 *h1, homo2 *h2);
homo2   PointDiv (homo2 *h1, pelem *h2);
homo2   PointDiv (pelem *h1, homo2 *h2);
homo2   PointDiv (pelem *h1, pelem *h2);
void    PointDiv (plist *pl1, plist *pl2);
void    PointDiv (plist *pl1, plist *pl2, plist *pl3);
plist  *PointDivNew (plist *pl, double s);
plist  *PointDivNew (plist *pl1, plist *pl2);
double    PointLen (homo2 *h);
double    PointLen (pelem *p);
bool    PointList (plist *pl);
homo2   PointMult (homo2 *h, double s);
homo2   PointMult (pelem *p, double s);
void    PointMult (plist *pl, double s);
void    PointMult (plist *pl1, double s, plist *pl2);
homo2   PointMult (homo2 *h1, homo2 *h2);
homo2   PointMult (homo2 *h1, pelem *h2);
homo2   PointMult (pelem *h1, homo2 *h2);
homo2   PointMult (pelem *h1, pelem *h2);
void    PointMult (plist *pl1, plist *pl2);
void    PointMult (plist *pl1, plist *pl2, plist *pl3);
plist  *PointMultNew (plist *pl, double s);
plist  *PointMultNew (plist *pl1, plist *pl2);
double    PointProd (homo2 *h1, homo2 *h2);
double    PointProd (homo2 *h1, pelem *h2);
double    PointProd (pelem *h1, homo2 *h2);
double    PointProd (pelem *h1, pelem *h2);
homo2   PointSqr (homo2 *p);
homo2   PointSqr (pelem *p);
void    PointSqr (plist *pl);
void    PointSqr (plist *pl1, plist *pl2);
plist  *PointSqrNew (plist *pl);
homo2   PointSqrt (homo2 *p);
homo2   PointSqrt (pelem *p);
void    PointSqrt (plist *pl);
void    PointSqrt (plist *pl1, xlist *pl2);
plist  *PointSqrtNew (plist *pl);
homo2   PointSub (homo2 *h, double s);
homo2   PointSub (pelem *p, double s);
void    PointSub (plist *pl, double s);
void    PointSub (plist *pl1, double s, plist *pl2);
homo2   PointSub (homo2 *h1, homo2 *h2);
homo2   PointSub (homo2 *h1, pelem *h2);
homo2   PointSub (pelem *h1, homo2 *h2);
homo2   PointSub (pelem *h1, pelem *h2);
void    PointSub (plist *pl1, plist *pl2);
void    PointSub (plist *pl1, plist *pl2, plist *pl3);
plist  *PointSubNew (plist *pl, double s);
plist  *PointSubNew (plist *pl1, plist *pl2);
void    PrintPoint (pelem *p);
void    PrintPoint (plist *pl);
void    RelinkPoint (plist *pl);
void    ScalePoint (homo2 *h, double f);
void    ScalePoint( plist *pl, double f);
void    ScalePoint( pelem *pl, double f);
void    SortPoint (plist *pl);
void    SortPointUnlinked (plist *pl);
void    SortPointDisparity (plist **pl1, plist **pl2);
void    SwapPoint (plist *pl1, plist *pl2);
homo2   TransformNormPoint (matrix *H, homo2 *h);
homo2   TransformPoint (matrix *H, homo2 *h);
void    TransformPoint (matrix *H, pelem *p);
void    TransformPoint (matrix *H, plist *pl);
homo2   TransformPointNew (matrix *H, pelem *p);
plist  *TransformPointNew (matrix *H, plist *pl);


// ===========================================================================
//                                 resample.cpp
// ===========================================================================
byte    InterpolBiCubic (double x, double y, int col, image *pic);
byte    InterpolBiLinear (double x, double y, int col, image *pic);


// ===========================================================================
//                                   tags.cpp
// ===========================================================================
void    WriteTags (FILE *fp, int tagc, char *tagv[]);
void    ReadTags (char *name, int *tagc, char **tagv[]);


// ===========================================================================
//                                  vector.cpp
// ===========================================================================
void    vec_abs (vektor *A);
void    vec_abs (vektor *A, vektor *B);
vektor *vec_abs_new (vektor *A);
void    vec_add (vektor *A, double s);
void    vec_add (vektor *A, double s, vektor *B);
void    vec_add (vektor *A, vektor *B);
void    vec_add (vektor *A, vektor *B, vektor *C);
vektor *vec_add_new (vektor *A, double s);
vektor *vec_add_new (vektor *A, vektor *B);
vektor *vec_alloc (int len);
double	vec_angle (vektor *A, vektor *B);
vektor *vec_clone (vektor *A);
vektor *vec_clone (vektor *A, int s, int e);
void    vec_copy (vektor *A, int s, int e, vektor *B);
void    vec_copy (vektor *A, int s, int e, vektor *B, int off);
void    vec_copy (vektor *A, vektor *B);
void    vec_copy (vektor *A, vektor *B, int off);
void    vec_cross (vektor *A, vektor *B);
void    vec_cross (vektor *A, vektor *B, double *s);
void    vec_cross (vektor *A, vektor *B, vektor *C);
vektor *vec_cross_new (vektor *A, vektor *B);
vektor *vec_diag (matrix *A);
bool    vec_div (vektor *A, double s);
bool    vec_div (vektor *A, double s, vektor *B);
void    vec_div (vektor *A, vektor *B);
void    vec_div (vektor *A, vektor *B, vektor *C);
vektor *vec_div_new (vektor *A, double s);
vektor *vec_div_new (vektor *A, vektor *B);
matrix *vec_dyadic (vektor *A, vektor *B);
bool    vec_equal (vektor *A, vektor *B);
void    vec_fill (vektor *A, int s, int e, double v);
void    vec_fill (vektor *A, double v);
bool    vec_finite (vektor *A, vektor *B);
vektor *vec_free (vektor *A);
vektor *vec_init (int len, ...);
double    vec_len (vektor *A);
double    vec_max (vektor *A);
double    vec_maxabs (vektor *A);
double    vec_mean (vektor *A);
double    vec_meanabs (vektor *A);
double    vec_median (vektor *A);
double    vec_median (vektor *A, int num);
double    vec_medianabs (vektor *A);
double    vec_medianabs (vektor *A, int num);
vektor *vec_merge (vektor *A, vektor *B);
vektor *vec_merge (vektor *A, vektor *B, vektor *C);
vektor *vec_merge (vektor *A[], int num);
double    vec_min (vektor *A);
double    vec_minabs (vektor *A);
void    vec_minmax (vektor *A, double *min, double *max);
void    vec_minmaxabs (vektor *A, double *min, double *max);
void    vec_mult (vektor *A, double s);
void    vec_mult (vektor *A, double s, vektor *B);
void    vec_mult (vektor *A, vektor *B);
void    vec_mult (vektor *A, vektor *B, vektor *C);
vektor *vec_mult_new (vektor *A, double s);
vektor *vec_mult_new (vektor *A, vektor *B);
//double	vec_norm (vektor *A); // Changed Name -> vec_normalize
double    vec_norm (vektor *A, norm p);
bool    vec_normalize (vektor *A);
bool    vec_normalize (vektor *A, norm p);
bool    vec_normalize (vektor *A, vektor *B);
bool    vec_normalize (vektor *A, norm p, vektor *B);
vektor *vec_normalize_new (vektor *A);
vektor *vec_normalize_new (vektor *A, norm p);
void    vec_print (vektor *A);
void    vec_print (char *label, vektor *A);
double    vec_prod (vektor *A, vektor *B);
void    vec_random (vektor *A);
void    vec_hrealloc (vektor *A, int len);
void    vec_reverse (vektor *A);
void    vec_reverse (vektor *A, vektor *B);
vektor *vec_reverse_new (vektor *A);
void    vec_scale (vektor *A, double s);
void    vec_scale (vektor *A, double s, vektor *B);
vektor *vec_scale_new (vektor *A, double s);
void    vec_shuffle(vektor *A);
vektor *vec_shuffle_new(vektor *A);
void    vec_sort (vektor *A);
vektor *vec_sort_new (vektor *A);
void    vec_split (vektor *A, int i, vektor *B);
void    vec_split (vektor *A, int i, vektor *B, vektor *C);
void    vec_split_new (vektor *A, int i, vektor **B, vektor **C);
void    vec_sqr (vektor *A);
void    vec_sqr (vektor *A, vektor *B);
vektor *vec_sqr_new (vektor *A);
void    vec_sqrt (vektor *A);
void    vec_sqrt (vektor *A, vektor *B);
vektor *vec_sqrt_new (vektor *A);
double    vec_std (vektor *A);
double    vec_std (vektor *A, double *mean);
double    vec_std2 (vektor *A);
double    vec_std2 (vektor *A, double *mean);
void    vec_sub (vektor *A, double s);
void    vec_sub (vektor *A, double s, vektor *B);
void    vec_sub (vektor *A, vektor *B);
void    vec_sub (vektor *A, vektor *B, vektor *C);
vektor *vec_sub_new (vektor *A, double s);
vektor *vec_sub_new (vektor *A, vektor *B);
double    vec_sum (vektor *A);
double	vec_sumabs (vektor *A);
void    vec_swap (vektor *A, vektor *B);
void    vec_testall (void);
void    vec_transform (matrix *A, vektor *B);
void    vec_transform (matrix *A, vektor *B, vektor *C);
vektor *vec_transform_new (matrix *A, vektor *B);
double    vec_var (vektor *A);
double    vec_var (vektor *A, double *mean);
double    vec_var2 (vektor *A);
double    vec_var2 (vektor *A, double *mean);

// ===========================================================================
//                                   util.cpp
// ===========================================================================
void    Angle2Rot (double x, double y, double z, matrix *R);
void    Angle2Rot_XYZ (double x, double y, double z, matrix *R);
char   *BaseFilename (char *filename);
char   *BaseFilenameExtension (char *filename, char *end);
// double    CalcTime (struct __timeb64 *start, struct __timeb64 *stop); // Removed, it not used anyway. GV.
bool    createDirectory(char *directory);
void    ErrorExit (int code, char *text);
char   *ExchangeFilename (char *oldFilename, char *newFilename);
char   *FilenameExtension (char *filename, char *end);
double    GaussNoise(double mean, double variance);
void   *halloc (uint num, uint size);
void   *hrealloc (void *ptr, uint num, uint size);
void    ImageCheckROI (region *roi, image *pic);
void    InitRandom (void);
void    MatrixCheckROI (region *roi, matrix *dat);
char   *NumberFilename (char *filename, int num);
double    Random (double r);
void    Rot2Angle (matrix *R, double *x, double *y, double *z);
void    Rot2Angle_XYZ (matrix *R, double *x, double *y, double *z);
void    SetROI (int x, int y, int w, int h, region *roi);
void    Sortdouble (double *a, int num);
double*   GetdoubleSorted (double *a, double elementToFind, int num);
void    SortInt (int *a, int num);
int*    GetIntSorted (int *a, int elementToFind, int num);

#ifdef _DEBUG
void    PrintMemStack(void);
void    debug_free (void* ptr);
#define hfree(a) debug_free((a))
#endif
#ifndef _DEBUG
#define hfree(a) free(a)
#endif

// ===========================================================================
//                                 triangle
// ===========================================================================
bool    Triangulation (matrix *P1, matrix *P2, homo2 *h1, homo2 *h2, homo3 *X);
xlist  *Triangulation (matrix *P1, matrix *P2, homo2 *h1, homo2 *h2, int n);
bool    Triangulation (matrix *P1, matrix *P2, pelem *p1, pelem *p2, homo3 *X);
xlist  *Triangulation (matrix *P1, matrix *P2, plist *pl1, plist *pl2);
bool    Triangulation (matrix *P1, matrix *P2, matrix *P3, homo2 *h1, homo2 *h2, homo2 *h3, homo3 *X);
xlist  *Triangulation (matrix *P1, matrix *P2, matrix *P3, homo2 *h1, homo2 *h2, homo2 *h3, int n);
bool    Triangulation (matrix *P1, matrix *P2, matrix *P3, pelem *p1, pelem *p2, pelem *p3, homo3 *X);
xlist  *Triangulation (matrix *P1, matrix *P2, matrix *P3, plist *pl1, plist *pl2, plist *pl3);
bool    Triangulation (matrix *P[], homo2 *h[], int num, homo3 *X);
xlist  *Triangulation (matrix *P[], int m, homo2 *h[], int n);
bool    Triangulation (matrix *P[], pelem *p[], int num, homo3 *X);
xlist  *Triangulation (matrix *P[], int m, plist *pl[]);

// ===========================================================================
//                                 triopt
// ===========================================================================
homo2   ClosestPoint (double a, double b, double c);
xlist  *OptimalTriangulation (plist *pl1, plist *pl2, matrix *F, matrix *P1, matrix *P2);


// ===========================================================================
//                                  camera
// ===========================================================================
void    AddCalibration (matrix *K, matrix *P);
matrix *AddCalibrationNew (matrix *K, matrix *P);
homo3   BackProject (matrix *PT, homo2 *x);
homo3   BackProject (matrix *PT, pelem *x);
matrix *CalibratedCameraMatrix (matrix *K);
matrix *CameraCalibMatrix (matrix *P);
matrix *CameraRotMatrix (matrix *P);
matrix *CanonicCameraBase (matrix *P);
void    DecomposeCamera (matrix *P, matrix **K, matrix **R);
void    EvalBundleError (plist *pl[], matrix *P[], int num, xlist *xl);
void    EvalMetricCamera (matrix *P);
double    GetCameraBase(matrix *P);
matrix *NormalizedCameraMatrix (void);
void    PrintCameraMatrix (matrix *P);
homo2   Project (matrix *P, homo3 *X);
homo2   Project (matrix *P, xelem *X);
plist  *Project (matrix *P, homo3 *xl, int num);
plist  *Project (matrix *P, xlist *xl);
double    ProjectError (matrix *P, plist *pl, xlist *xl);
homo3   ProjectionCenter (matrix *P);
void    ProjectiveCameraMatrix (matrix *F, homo2 *e, matrix **P1, matrix **P2);
double    ProjectSymError (matrix *P1, matrix *P2, plist *pl1, plist *pl2, xlist *xl);
matrix *RecomposeCamera (matrix *K, matrix *R, homo3 *C);
void    ScaleCameraBase(matrix *P, double b);
void    TransformCamera (matrix *P, matrix *H);
void    TransformCameraBase (matrix *P[], matrix *B[], int num);
matrix *TransformCameraNew (matrix *P, matrix *H);

// ===========================================================================
//                                    svd
// ===========================================================================
bool    mat_least_squares (matrix *A, vektor *b, vektor **x);
bool    mat_svd (matrix *A, matrix **U, vektor **d, matrix **V, bool sort);
bool    mat_svd_left_nullvector (matrix *A, vektor **N);
bool    mat_svd_nullspace (matrix *A, int num, vektor ***N);
bool    mat_svd_nullvector (matrix *A, vektor **N);
void    mat_svd_recompose (matrix *U, vektor *d, matrix *V, matrix **A);
bool    mat_svd_essential (matrix *A, matrix **U, matrix **V);

// ===========================================================================
//                                 givens
// ===========================================================================
bool    mat_qrd (matrix *A, matrix **Q, matrix **R);
bool    mat_rqd (matrix *A, matrix **R, matrix **Q);
bool    mat_rqd_alt (matrix *A, matrix **R, matrix **Q);

// ===========================================================================
//                                 cheiral
// ===========================================================================
homo3   CameraDirection (matrix *P);
bool    IsInFront (matrix *P, homo3 *X);
bool    IsInFront (matrix *P, xelem *X);
int     RecoverMotion (matrix *E, homo2 *h1, homo2 *h2, int num);
int     RecoverMotion (matrix *E, plist *pl1, plist *pl2);
int     RecoverMotion (matrix *E, homo2 *h1, homo2 *h2, int num, matrix **P2);
int     RecoverMotion (matrix *E, plist *pl1, plist *pl2, matrix **P2);
int     CountInFront(matrix *P1, matrix *P2, plist *pl1, plist *pl2);

// ===========================================================================
//                                 transfer
// ===========================================================================
void    CoordToHomo (xlist *xl, homo3 **h, int *num);
vektor *HomoToVec (homo2 *h);
vektor *HomoToVec (homo3 *h);
void    MatchToPoints (MatchList *ml, plist **pl1, plist **pl2, plist **pl3);
void    MatToVec (matrix *A, vektor *B);
void    MatToVecPos (matrix *A, vektor *B, int p);
void    PointToHomo (plist *pl, homo2 **h, int *num);
void    VecPosToMat (vektor *A, int p, matrix *B);
homo2   VecToHomo2 (vektor *A);
homo3   VecToHomo3 (vektor *A);
void    VecToMat (vektor *A, matrix *B);
homo2   VecToNormHomo2 (vektor *A);
homo3   VecToNormHomo3 (vektor *A);
void    VecToNormMat (vektor *A, matrix *B);

// ===========================================================================
//                                  epipol
// ===========================================================================
void    CalcEpipoles (matrix *F, homo2 *e1, homo2 *e2);
void    EvalEpipoles (matrix *F, homo2 *e1, homo2 *e2, plist *pl1, plist *pl2);

vektor *poly_root (vektor *A);

// ===========================================================================
//                                   homo
// ===========================================================================
matrix *CompatibleHomography (matrix *F, homo2 *e, homo2 *h1, homo2 *h2, int num);
matrix *CompatibleHomography (matrix *F, homo2 *e, plist *pl1, plist *pl2);
void    EvalHomography (matrix *H, matrix *F);
void    EvalHomography (matrix *H, plist *pl1, plist *pl2);
void    EvalHomography (matrix *H, xlist *xl, plist *pl);
void    EvalHomography (matrix *H, xlist *xl, xlist *xel);
homo2   HomoCross (homo2 *h1, homo2 *h2);
homo2   HomoCross (pelem *p1, homo2 *h2);
homo2   HomoCross (homo2 *h1, pelem *p2);
homo2   HomoCross (pelem *p1, pelem *p2);
double    HomoDist (homo2 *h1, homo2 *h2);
double    HomoDist (pelem *p1, homo2 *h2);
double    HomoDist (homo2 *h1, pelem *p2);
double    HomoDist (pelem *p1, pelem *p2);
double    HomoDist (homo3 *h1, homo3 *h2);
double    HomoDist (xelem *x1, homo3 *h2);
double    HomoDist (homo3 *h1, xelem *x2);
double    HomoDist (xelem *x1, xelem *x2);
double    HomoDistSqr (homo2 *h1, homo2 *h2);
double    HomoDistSqr (pelem *p1, homo2 *h2);
double    HomoDistSqr (homo2 *h1, pelem *p2);
double    HomoDistSqr (pelem *p1, pelem *p2);
double    HomoDistSqr (homo3 *h1, homo3 *h2);
double    HomoDistSqr (xelem *x1, homo3 *h2);
double    HomoDistSqr (homo3 *h1, xelem *x2);
double    HomoDistSqr (xelem *x1, xelem *x2);
double    HomoLen (homo2 *h);
double    HomoLen (homo3 *h);
double    HomoMult (homo2 *h1, homo2 *h2);
double    HomoMult (pelem *p1, homo2 *h2);
double    HomoMult (homo2 *h1, pelem *p2);
double    HomoMult (pelem *p1, pelem *p2);
double    HomoMult (homo3 *h1, homo3 *h2);
double    HomoMult (xelem *x1, homo3 *h2);
double    HomoMult (homo3 *h1, xelem *x2);
double    HomoMult (xelem *x1, xelem *x2);
vektor *HomographyError (matrix *H, homo3 *h1, homo3 *h2);
vektor *HomographyError (matrix *H, homo2 *h1, homo2 *h2);
vektor *HomographyError (matrix *H, homo3 *h1, homo2 *h2);
vektor *HomographyError (matrix *H, xlist *xl, plist *pl);
vektor *HomographyError (matrix *H, plist *pl1, plist *pl2);
vektor *HomographyError (matrix *H, xlist *xl, xlist *xel);
vektor *HomographyError (matrix *H, homo2 *h1, homo2 *h2, int num);
matrix *NormalizedHomography (plist *pl1, plist *pl2);
matrix *NormalizedHomography (xlist *xl, plist *pl);
matrix *NormalizedHomography (xlist *xl, xlist *xel);
homo2   NormHomoCross (homo2 *h1, homo2 *h2);
homo2   NormHomoCross (pelem *p1, homo2 *h2);
homo2   NormHomoCross (homo2 *h1, pelem *p2);
homo2   NormHomoCross (pelem *p1, pelem *p2);
void    ScaleHomo (homo2 *h, double f);
void    ScaleHomo (homo3 *h, double f);
homo2   TransformHomo (matrix *H, homo2 *h);
homo3   TransformHomo (matrix *H, homo3 *X);
homo2   TransformNormHomo (matrix *H, homo2 *h);
homo3   TransformNormHomo (matrix *H, homo3 *X);

// ===========================================================================
//                                  eigen
// ===========================================================================
//void    mat_balance (matrix *a);
//void    mat_elmhes (matrix *a);
//bool    mat_hqr (matrix *a, vektor **wr, vektor **wi);
bool    mat_eigen (matrix *a, matrix **eigVec, vektor **eigVal);
bool    mat_eigen_alt (matrix *a, matrix **eigVec, vektor **eigVal);

// ===========================================================================
//                                  error
// ===========================================================================
void    AccumulateError (double err);
double    CalcEpipolarDist (matrix *F, homo2 *h1, homo2 *h2);
double    CalcEpipolarDist (matrix *F, pelem *p1, pelem *p2);
double    CalcRadialDist (homo2 *x, camera *cam, homo3 *X);
double    CalcRadialDist (pelem *x, camera *cam, xelem *X);
double    CalcReproject (homo2 *x, matrix *P, homo3 *X);
double    CalcReproject (homo2 *x, matrix *P, xelem *X);
double    CalcReproject (pelem *x, matrix *P, homo3 *X);
double    CalcReproject (pelem *x, matrix *P, xelem *X);
double    CalcResidual (matrix *H, homo2 *h1, homo2 *h2);
double    CalcResidual (matrix *H, pelem *p1, pelem *p2);
double    CalcResidual (matrix *H, homo3 *h1, homo2 *h2);
double    CalcResidual (matrix *H, xelem *x1, pelem *p2);
double    CalcResidual (matrix *H, homo3 *h1, homo3 *h2);
double    CalcResidual (matrix *H, xelem *x1, xelem *x2);
double    CalcSymEpipolarDist (matrix *F, matrix *FT, homo2 *h1, homo2 *h2);
double    CalcSymEpipolarDist (matrix *F, matrix *FT, pelem *h1, pelem *h2);
double    CalcSymReproject (homo2 *x1, homo2 *x2, matrix *P1, matrix *P2, homo3 *X);
double    CalcSymReproject (homo2 *x1, homo2 *x2, matrix *P1, matrix *P2, xelem *X);
double    CalcSymReproject (pelem *x1, pelem *x2, matrix *P1, matrix *P2, homo3 *X);
double    CalcSymReproject (pelem *x1, pelem *x2, matrix *P1, matrix *P2, xelem *X);
double    CalcSymResidual (matrix *H, matrix *HI, homo2 *h1, homo2 *h2);
double    CalcSymResidual (matrix *H, matrix *HI, pelem *p1, pelem *p2);
double    CalcSymResidual (matrix *H, matrix *HI, homo3 *h1, homo3 *h2);
double    CalcSymResidual (matrix *H, matrix *HI, xelem *x1, xelem *x2);
void    CalcTriFocalDist (cube *C, homo2 *h1, homo2 *h2, homo2 *h3, double *r);
void    CalcTriFocalDist (cube *C, pelem *p1, pelem *p2, pelem *p3, double *r);
void    InitError (void);
void    PrintError (void);
// ===========================================================================
//                                  radial
// ===========================================================================
homo2   RadialDist (camera *cam, homo2 *p);
double    RadialDistError (camera *cam, plist *pl, xlist *xl);
void    PrintCameraMatrix (camera *cam);
homo2   Project (camera *cam, homo3 *X);
homo2   Project (camera *cam, xelem *X);
void    UpdateSymmetryPoint (camera *cam);

#endif //MathTools_
