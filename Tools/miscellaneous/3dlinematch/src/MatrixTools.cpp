
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
/*
 * MatrixTools.cpp
 *
 *  Created on: 17-Jul-2008
 *      Author: gerke
 */

#include "MatrixTools.h"
// The LAPACK library functions
#ifndef STANDALONE
  extern "C" {
    int dgetrf_(int *m, int *n, double *a, int *lda, int *ipiv, int *info);
    int dgbtrf_(int *m, int *n, int *kl, int *ku, double *ab, int *ldab, int *ipiv, int *info);
    int dgetri_(int *n, double *a, int *lda, int *ipiv, double *work, int *lwork, int *info);
  }
#endif

void add_noise(matrix *A, double max_rel)
{
	InitRandom ();
	//printf("debug: in add_noise: max_rel=%f\n",max_rel);
	//idea: each value gets an addition of +/- max_rel, the actual value is random
	for (int r=0;r<A->rows;r++)
		 for (int c=0;c<A->cols;c++)
		 {
			 double DX=max_rel*2*A->m[r][c];
			 double Xmin=A->m[r][c]-0.5*DX;

			 double factor=Random(1); //0<=factor<=1;
			 //printf("debug: in add noise:  A alt=%f DX=%f, Xmin= %f, factor=%f, ",A->m[r][c], DX, Xmin, factor);

			 A->m[r][c]=Xmin+(DX*factor);

			 //printf("A neu=%f\n",A->m[r][c]);

		 }


}

void add_outliers(matrix *A, double number_rel , double size_rel) //outliers: number of outliers (percentage) and size (offset: percentage of actual value)
{
	InitRandom ();
	int number_abs=(int) (A->rows*A->cols*number_rel);
	printf("in add_outliers: relative number: %f absolute number:%d\n", number_rel, number_abs);
	std::vector<std::pair<int, int> > processed_elements;

	//randomly choose an element
	for (int i=0; i<number_abs;i++)
	{
		int r=(int) Random(A->rows);
		int c=(int) Random(A->cols);

		//look, if this element was already changed
		bool skip_it=0;
		for (uint p=0;p<processed_elements.size();p++)
		 if ((processed_elements[p].first == r) && (processed_elements[p].second==c)) skip_it=1;

	    if (skip_it) {i--;continue;}

	    double sign; if (r%2==0) sign=-1; else sign=1;
	    double offset=A->m[r][c]*sign*size_rel;
	    A->m[r][c]+=offset;
	    printf("Added outlier to %d,%d, offset added:%f\n",r,c,offset);

		processed_elements.push_back(std::make_pair(r,c));
	}
}

double mat_max_norm_val_vec(matrix *v, int &max_r)
{
	double ret=1e-10;
	max_r=-99;
	int c=0;
	for (int r=0;r<v->rows;r++)
			 {double ret_old=ret;
			 ret=MAX(ret,fabs(v->m[r][c]));

			 if (ret_old!=ret)
					 max_r=r;

			 }
	return ret;
}

bool mat_PseudoInverseofP(matrix *P, matrix **PP)
{
	bool ret=1;
	//Accoring to Hartley Zisserman: P+=P^T(P P^T)^(-1)
	matrix *Ptrans=mat_transpose_new(P);
	matrix *tmp=mat_prod_new(P,Ptrans);
	mat_invert(tmp);
	*PP=mat_prod_new(Ptrans,tmp);

	 //TEST: PP+=I
	   matrix *tst=mat_prod_new(P,*PP);
	  // mat_print("tst",tst);

	   double max_abs_value=mat_largest_abs_element(tst);
	   //printf("max_abs_value=%.6f\n",max_abs_value);

	   if (abs(1-max_abs_value) > 0.00001) ret=0;

	mat_free(Ptrans);
	mat_free(tmp);
	mat_free(tst);

	return ret;
}

double mat_largest_abs_element(matrix *A)
{
	double val=-1;

	 for (int r=0;r<A->rows;r++)
		 for (int c=0;c<A->cols;c++)
			 val=MAX(fabs(A->m[r][c]),val);

	 return val;
}

bool mat_save(char * fname, matrix * A)
{

	FILE *filep;

	if ((filep=fopen(fname,"w")) == NULL)
		{
			printf("CAN NOT OPEN %s!\n", fname);
			return(0);
		}

    int i, j;

    if (CheckMatrix(A)) {
//		Debug ("Matrix size = %d cols x %d rows", A->cols, A->rows);
    	fprintf(filep,"%d %d\n",A->cols, A->rows);
        for (j=0; j<A->rows; ++j) {
            //printf ("\t| ");
            for (i=0; i<A->cols; ++i) {
                fprintf (filep,"% e ", ISNONZERO(A->m[j][i]) ? A->m[j][i] : 0.0);
            }
            fprintf (filep,"\n");
        }
    }

    fclose(filep);
    return 1;
}

bool mat_load(char * fname, matrix **A)
{
	//matrix must be allocated already
	FILE *filep;

		if ((filep=fopen(fname,"r")) == NULL)
			{
				printf("CAN NOT OPEN %s!\n", fname);
				return(0);
			}

	int c,r,i,j;
	int Ac=mat_get_cols(*A);
	int Ar=mat_get_rows(*A);
	double val;
    fscanf(filep,"%d %d\n",&c,&r);

    if (!CheckMatrix(*A) || c<Ac || r<Ar)
    {
    	printf("Error: A not allocated or matrix saved has smaller dimension than allocated\n");
    	return 0;
    }

    else if (c>Ac || r>Ar)
    {
    	printf("Attention: saved matrix is larger (dimensions) than the allocated one: only %dx%d: only those elements are read\n",Ar,Ac);

    }
    //iterate over all, because of the newline, but only set the output-cell if inbound
    for (j=0; j<r; ++j) {
             //printf ("\t| ");
             for (i=0; i<c; ++i) {
                 fscanf (filep,"%lf", &val);
                 if ((j<Ar) && (i<Ac))mat_set_at(*A,j,i,val);
             }
             fscanf (filep,"\n");
         }

    fclose(filep);
    return 1;
}

matrix* sort_matrix_per_row(matrix *A)
{
	matrix *B = NULL;

	    if (CheckMatrix(A)) {
	        B = mat_alloc(A->rows, A->cols);

	        for (int r=0;r<A->rows;r++)
	        {
	        vektor *row_v=mat_getrow(A,r);
	        vec_sort(row_v);
	        mat_setrow(B,r,row_v);
	        vec_free(row_v);
	        }
	    }
	    return B;
}

bool LSA_Gauss_Markov(matrix *A, matrix *P, matrix *AtP, matrix *Ninv, matrix *l, double k, matrix *XNull, matrix **v, double *test1_val_zero, bool exit_on_blunder)
{
	bool LSA_ok=1;
	bool blunder_detected=0;
	matrix*n, *x_dach;
	matrix *test1,*test2,*tmp;
	/*LOOP of iterations for blunder check start*/
				bool do_blunder_iteration=1;

				while(do_blunder_iteration)
				{

				n=mat_prod_new(AtP,l);
				x_dach=mat_prod_new(Ninv,n);
/*
				mat_print("\nXNull", XNull);
			    mat_print("\nl",l);
			    mat_print("\nx_dach",x_dach);
*/


			    	//verbesserter Unbekanntenvektor
			    	//matrix *tmp_XNull;
			    	//tmp_XNull=mat_add_new(XNull,x_dach);
			    	//mat_free(XNull);
			    	//XNull=mat_clone(tmp_XNull);
			    	//mat_free(tmp_XNull);
					mat_add(XNull,x_dach);
			       //mat_print("\nXDach (i.e. XNull for next iteration)",XNull);


			    	tmp=mat_prod_new(A,x_dach);

			    	*v=mat_sub_new(tmp,l);
			    	//mat_print("v",*v);

					//robust estimation: find largest v(i)>k;, see niemeyer, p. 202...
			    	int max_r;
			    	double max_v=mat_max_norm_val_vec(*v, max_r); //search |v| > k
			    	//printf("maximum v:%f, at %d\n", max_v, max_r);

			    	if ((fabs(max_v)  > k) && (max_r != -99))
			    	{   printf("maximum v:%f, l at that posisition: %f at %d\n", max_v, l->m[max_r][0], max_r);
			    		printf("observation at %d in L is considered as blunder: l=%f!\n",max_r,l->m[max_r][0]);
			    		blunder_detected=1;
			    		/*
			    		//modify l
			    		//redundanzanteil aus qvv
			    		double r=Qvv->m[max_r][max_r];
			    		//sigma vi
			    		double sig_vi=sqrt(mat_get_at(P,max_r,max_r))*sqrt(r); //sqrt(P...): sigma for l=sigma naught here
			    		double d=max_v-signof(max_v)*k*sig_vi;

			    		printf("r=%f, sig_vi=%f, d=%f, old l=%f\n",r,sig_vi,d,l->m[max_r][0]);
			    		//some confusion with the sign
			    		l->m[max_r][0]-=signof(l->m[max_r][0])*d/r;

			    		printf("new l=%f\n",l->m[max_r][0]);

			    		//!TODO: save blunder elemeents (index in L-vector) for an enhanced Qxx later
			    		n=mat_free(n);
			    		x_dach=mat_free(x_dach);
			    		v=mat_free(v);
			    		tmp=mat_free (tmp);

			    		do_blunder_iteration=1;
			    		*/

			    		/*
			    		printf("The weight in SIG_LL is decreased and A will be computed the next time!\n");
			    		mat_set_at(QLL,max_r,max_r,1E-10);
			    		mat_set_at(P,max_r,max_r,1e10);
			    		next_time_compute_A=1;
						*/
			    		do_blunder_iteration=0;

			    	}
			    	else
					{
			    		//printf("no blunder detected\n");
			    		do_blunder_iteration=0;
					}
				}
			    /*LOOP of iterations for blunder check end*/

			    	tmp=mat_free (tmp);
			    	tmp=mat_prod_new(mat_transpose_new(*v),P);

			    	test1=mat_prod_new(tmp,*v);
			    	test2=mat_prod_new(tmp,l);
			    	mat_scale(test2,-1);
			    	*test1_val_zero=test1->m[0][0];

			    	//mat_print("test1",test1);
			    	//mat_print("test2",test2);


			    	if (!CheckMatrixIdentityRelativeError(test1,test2,0.1)){ printf("Information: ERROR in algorithm!: vtPv not -vtPl: most probably error during inversion -> A too big?");LSA_ok=0;}
			    	else LSA_ok=1;

			    	//blunder test==> break?
			    	if (blunder_detected)
			    	{
			    		printf("A blunder was detected\n");
			    	if (!exit_on_blunder) printf("No action on this blunder is taken. There should be no blunder, as they are filtered out during estimation (redundancy?)!\n");

			    	else {
			    		printf("exit because of detected blunder\n");
			    		LSA_ok=0;
			    	}

			    	}
			    	//other test (niemeyer, s. 127): vTPv==lTPl-nTxdach
			    	test2=mat_free(test2);
			    	tmp=mat_free (tmp);
			    	tmp=mat_prod_new(mat_transpose_new(l),P);
			    	test2=mat_prod_new(tmp,l); //summand 1 (lTPl)

			    	tmp=mat_free (tmp);
			    	tmp=mat_prod_new(mat_transpose_new(n),x_dach); //summand 2 (nTxdach

			    	matrix *sub=mat_sub_new (test2, tmp);

			    	if (LSA_ok && !CheckMatrixIdentityRelativeError(test1,sub,0.1)){ printf("Information: ERROR in algorithm!: vtPv not ltPl-ntxdach ");LSA_ok=0;}

			    	mat_free(sub);

n=mat_free(n);
x_dach=mat_free(x_dach);
test1=mat_free(test1); test2=mat_free(test2);
tmp=mat_free(tmp);
return LSA_ok;
}

matrix * mat_invert_opencv(matrix *A)
{
 if (!CheckSquareMatrix(A)) {printf("in mat_invert_lapack: matrix not square!"); exit(0);}
 int N=A->cols;

 CvMat *B=cvCreateMat(N, N, CV_64FC1);

 for (int r=0; r<N;r++)
 	 for (int c=0;c<N;c++)
 			cvmSet(B, r, c, A->m[r][c]);


 //Inversion (if return==0-->ERROR)
CvMat *Binv=cvCreateMat(N, N, CV_64FC1);
/*
double det=cvInvert(B,Binv,CV_LU);
printf("Det of inverse:%e\n",det);
if (det==0) printf("Determinant of inverse==0! Error");
*/
double err=cvInvert(B,Binv,CV_SVD);
//printf("Errorcode of inversion:%e\n",err);

matrix *RET=mat_alloc(N,N);
 for (int r=0; r<N;r++)
  	 for (int c=0;c<N;c++)
  			RET->m[r][c]=cvmGet(Binv, r,c);

cvReleaseMat(&B);
cvReleaseMat(&Binv);

return RET;
}

#ifndef STANDALONE
matrix * mat_invert_lapack(matrix *A)
{
 if (!CheckSquareMatrix(A)) {printf("in mat_invert_lapack: matrix not square!"); exit(0);}
 int N=A->cols;

 double *Ad = new double[N*N];
 for (int r=0;r<N;r++)
	 for (int c=0;c<N;c++)
		 Ad[r*N+c]=A->m[r][c];


 // Set up storage
     int *ipiv = new int[N];
     int info;

     // Call for the LU factorisation
       dgetrf_(&N, &N, Ad, &N, ipiv, &info);


     if (info>0) {
       printf("info= %d\n",info);
       delete[] ipiv;
       exit (0);
     }

     // Set up workspace
     int lwork = N;
     double *workd = new double[lwork];

     // Now, call for inverse
     dgetri_(&N, Ad, &N, ipiv, workd, &lwork, &info);

     delete[] ipiv;
     delete[] workd;

     if (info>0) {
            printf("info= %d\n",info);
            delete[] ipiv;
            exit (0);
          }

     matrix *ret=mat_clone(A);
      for (int r=0;r<N;r++)
      	 for (int c=0;c<N;c++)
      		 ret->m[r][c]=Ad[r*N+c];

      delete[] Ad;
      return ret;
}

matrix * mat_invert_lapack_bandmat(matrix *A, int width)
{
 if (!CheckSquareMatrix(A)) {printf("in mat_invert_lapack_bandmat: matrix not square!"); exit(0);}
 int Norig=A->cols;

 //We need to rearrange A, see Lapack manual
 //KL:number of diagonals below the main diag (subdiagonals), KU: above. We assume a symmetric band, hence KU=KL=(width-1)
 //AB(kl+ku+1+i-j,j) = A(i,j) for max(1,j-ku)<=i<=min(m,j+kl)
 if (((width-1)%2) !=0) {printf("Error: width not odd\n"); exit(0);}
  int KU=(width-1);//2;
  int KL=(width-1);//2;
 int Ntmp=max(Norig,2*KL+KU+1);
 printf("Ntmp=%d",Ntmp);

 if (Ntmp != Norig) {printf ("Error: change of matrix size not yet implemented!\n"); exit(0);}

 matrix * Atmp=mat_alloc(Ntmp,Ntmp);mat_set(Atmp,0);
 for (int j=0;j<Ntmp;j++)
	 for (int i=max(0,j-KU);i<min(Ntmp,j+KL+1);i++)
	 {
		if (fabs(A->m[i][j])>1E-15)
		 Atmp->m[KL+KU+1+i-j][j]=A->m[i][j];
		//Atmp->m[KL+KU+1+i-j+1][j]=A->m[i][j];
		else Atmp->m[KL+KU+1+i-j][j]=1E-15;
		//else Atmp->m[KL+KU+1+i-j+1][j]=1E-15;
	 }

 double *Ad = new double[Ntmp*Ntmp];
 for (int r=0;r<Ntmp;r++)
	 for (int c=0;c<Ntmp;c++)
		 Ad[r*Ntmp+c]=Atmp->m[r][c];


 // Set up storage
     int *ipiv = new int[Ntmp];
     int info;


     printf("vor LU");
     for (int r=0;r<Ntmp;r++)
     {    printf("\n");
     	for (int c=0;c<Ntmp;c++)
     		printf("%e ",Ad[r*Ntmp+c]);
     }


     int ldab=Ntmp;
     // Call for the LU factorisation
     dgbtrf_(&Ntmp, &Ntmp, &KL, &KU, Ad, &ldab, ipiv, &info);

     printf("\n\nnach LU");
         for (int r=0;r<Ntmp;r++)
         {    printf("\n");
         	for (int c=0;c<Ntmp;c++)
         		printf("%e ",Ad[r*Ntmp+c]);
         }
         printf("\n");
/**/
     if (info>0) {
       printf("nach LU factorisierung info= %d\n",info);
       delete[] ipiv;
       exit (0);
     }
/**/
     // Set up workspace
     int lwork = Ntmp;
     double *workd = new double[lwork];

     // Now, call for inverse
     //IS IT STILL THE SAME?
     //dgetri_(&N, &(AInv(0,0)), &N, ipiv, workd, &lwork, &info);
     dgetri_(&Ntmp, Ad, &Ntmp, ipiv, workd, &lwork, &info);

     delete[] ipiv;
     delete[] workd;
/**/
     if (info>0) {
            printf("info= %d\n",info);
            delete[] ipiv;
            exit (0);
          }
/**/
     matrix *ret=mat_clone(A);
      for (int r=0;r<Ntmp;r++)
      	 for (int c=0;c<Ntmp;c++)
      		 ret->m[r][c]=Ad[r*Ntmp+c];
mat_print("ret matrix",ret);

bool flag=true;
matrix *tst=mat_prod_new(A,ret);
  mat_print("tst, needs to be I",tst);
  double min,max;
  mat_minmax(tst,&min,&max);
  printf("in mat_invert_lapack: test: min=%e, max=%e\n",min,max);
	if (fabs(max) > 1.01) flag=false;
	if (fabs(min) > 0.01) flag=false;

	tst=mat_free(tst);
      delete[] Ad;
      Atmp=mat_free(Atmp);

      if (flag) return ret;
      else {
    	  ret=mat_free(ret);
    	  return NULL;
      }
}
bool mat_PseudoInverse(matrix *A, matrix **P)
{bool ret=1;

	//We use the SVD to compute the Pseudo inverse of A
	/*Wikipedia
	 * The singular value decomposition can be used for computing the pseudoinverse of a matrix.
	 * Indeed, the pseudoinverse M+ of the matrix M with singular value decomposition M = UDV* is

    M+ = V D+  U*, \

where D+ s the pseudoinverse of D , which is formed by replacing every nonzero entry by its reciprocal.
	 */

	matrix *U,*U_tmp, *V;
	vektor *d;

	//mat_print("in pseudo: original",A);
	//mat_svd (matrix *A, matrix **U, vektor **d, matrix **V, bool sort)
	mat_svd (A,&U_tmp, &d, &V,0);

	//printf("size of A:%dx%d, U:%dx%d, d:%d, V:%dx%d\n",A->rows, A->cols, U_tmp->rows,U_tmp->cols,d->len,V->rows,V->cols);
	//Transpose U and V, reziprokals of d (and put it in matrix)

/*
	mat_print("U_tmp",U_tmp);
	mat_print("V",V);
	vec_print("d",d);
*/

	mat_transpose (U_tmp);
	mat_transpose (V);
	matrix *d_mat=mat_alloc(d->len,d->len);mat_set(d_mat,0);
	for(int j=0; j<d->len; ++j)
		if (!ISZERO(d->v[j]))
		{
		 //d->v[j]=1/d->v[j];
		 d_mat->m[j][j]=1/d->v[j];
		}
	//if r<c, the U-matrix needs to be resized, since it is made quadratic in mat_svd (just cut the upper left rc part)
	//attention!U_tmp is now already transposed
		if (A->rows < A->cols)
		{
			U=mat_alloc(A->cols,A->rows);
			mat_copy(U_tmp,U);
		}
		else U=mat_clone(U_tmp);


	//matrix *P;
	//mat_svd_recompose (V, d, U, &P);
	/*
		mat_print("ende U",U);
		mat_print("V",V);
		mat_print("d_mat",d_mat);
    */
   matrix *tmp=mat_prod_new(V,d_mat);
   //matrix *P=mat_prod_new(tmp,U);
   *P=mat_prod_new(tmp,U);

   //mat_print("t",tmp);
   //mat_print("pseudo:",*P);

   //TEST: PP+=I
   matrix *tst=mat_prod_new(A,*P);
   mat_print("tst, needs to be I",tst);
   double min,max;
   mat_minmax(tst,&min,&max);
   printf("in mat_invert_Pseudo: test: min=%e, max=%e\n",min,max);
	if (fabs(max) > 1.01) ret=false;
	if (fabs(min) > 0.01) ret=false;

   mat_free(U);mat_free(U_tmp);
	mat_free(V);
	mat_free(tmp);
	mat_free(tst);
	vec_free(d);

	return ret;

}
#endif
