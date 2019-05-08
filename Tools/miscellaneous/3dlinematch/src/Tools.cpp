
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
#include<Tools.h>
///Adapted from Hartley Zisserman Matlab code, fix for P:3x4 (N=3)

bool P_convert(double *P, matrix **P_mat)
{
	*P_mat=mat_alloc(3,4);
	//put data into P_mat;
		int count=0;
		for(int row=0;row<=2;row++)
			for(int col=0;col<=3;col++)
			{
			mat_set_at(*P_mat,row,col,P[count]);
			//printf ("P_convert: P(%d,%d)=%f\n",row,col,P[count]);
			count++;
			}
		return 1;
}




bool KRt_from_P(double *P, matrix **K, matrix**R, matrix **t)
{
	//H-Matrix(3,3): copy of first three columns from P (ommit last one)
	/* MATLAB CODE for normalis.
	 K = K / K(N,N);
	  if K(1,1) < 0
	    D = diag([-1 -1 ones(1,N-2)]);
	    K = K * D;
	    R = D * R;

	  %  test = K*R;
	  %  vgg_assert0(test/test(1,1) - H/H(1,1), 1e-07)
	  end
    *///t= -P(:,1:N)\P(:,end); = -H\last column-vektor of P  (\: left division, i.e. t=(-inv(H)*lastcolumnvektor

	//N=3;


	matrix *P_mat;
	P_mat=mat_alloc(3,4);

	matrix *H;
	H=mat_alloc(3,3);

	//put data into P_mat;
	int count=0;
	for(int row=0;row<=2;row++)
		for(int col=0;col<=3;col++)
		{
		mat_set_at(P_mat,row,col,P[count]);
		//printf ("P(%d,%d)=%f\n",row,col,P[count]);
		count++;
		}

	for (int c=0;c<=2;c++)
	mat_copycol(P_mat,c,H,c);

	//mat_print("P_mat",P_mat);
	//mat_print("H",H);
	//[K,R] = vgg_rq(H); (K has to be normalized, i.e. K(3,3)==1;

	if (!mat_rqd (H, K, R)){ printf("Errror in RQ-D!\n"); return 0;}
	//mat_print("K not normalized",*K);
	//mat_print("R",*R);

	//normalisation of K
	if (ISZERO(mat_get_at(*K,2,2))) { printf("avoid division by zero!\n"); return 0;}
	mat_scale(*K,1/mat_get_at(*K,2,2));
	//mat_print("K normalized",*K);

	//!!TEST: Sacle in y<0
	//mat_set_at(*K,1,1,mat_get_at(*K,1,1)*-1);
	//mat_print("K MODIFIED",*K);
	//TEST
	//mat_set_at(*K,0,0,-1);
	//TODO: test for K(0,0)<0
	if (mat_get_at(*K,0,0)<0)
	{
		matrix *D=mat_identity (3);
		mat_set_at(D,0,0,-1);
		mat_set_at(D,1,1,-1);
		*K=mat_prod_new(*K,D);
		*R=mat_prod_new(D,*R);
		mat_free(D);
	}
	//mat_print("K final",*K);
	//mat_print("R final",*R);

	vektor *p_v4=mat_getcol (P_mat, 3);
	matrix *p_m4=mat_alloc(3,1);
	mat_setcol (p_m4, 0 , p_v4);



	 //  test = K*R;
	//vgg_assert0(test/test(1,1) - H/H(1,1), 1e-07)
	/*
	matrix *test_mat=mat_prod_new(*K,*R);
	mat_scale(test_mat,1/mat_get_at(test_mat,0,0));
	matrix *H_scaled=mat_scale_new(H,1/mat_get_at(H,0,0));
	mat_print("test_mat",test_mat);
	mat_print("H_scaled",H_scaled);
	mat_print("sub",mat_sub_new(test_mat,H_scaled));
	mat_free(test_mat);
	mat_free(H_scaled);
	*/

	//calc. of t
	mat_invert(H);
	mat_scale(H,-1);
	*t=mat_prod_new(H,p_m4);

	mat_free(P_mat);
	mat_free(H);
	mat_free(p_m4);
	vec_free(p_v4);

	return 1;
}

bool KRt_from_P(matrix *P, matrix **K, matrix**R, matrix **t)
{
	//H-Matrix(3,3): copy of first three columns from P (ommit last one)
	/* MATLAB CODE for normalis.
	 K = K / K(N,N);
	  if K(1,1) < 0
	    D = diag([-1 -1 ones(1,N-2)]);
	    K = K * D;
	    R = D * R;

	  %  test = K*R;
	  %  vgg_assert0(test/test(1,1) - H/H(1,1), 1e-07)
	  end
    *///t= -P(:,1:N)\P(:,end); = -H\last column-vektor of P  (\: left division, i.e. t=(-inv(H)*lastcolumnvektor



	matrix *H;
	H=mat_alloc(3,3);

	for (int c=0;c<=2;c++)
	mat_copycol(P,c,H,c);

	//mat_print("P_mat",P_mat);
	//mat_print("H",H);
	//[K,R] = vgg_rq(H); (K has to be normalized, i.e. K(3,3)==1;

	if (!mat_rqd (H, K, R)) printf("Errror in RQ-D!\n");
	//mat_print("K not normalized",*K);
	//mat_print("R",*R);

	//normalisation of K
	mat_scale(*K,1/mat_get_at(*K,2,2));
	//mat_print("K normalized",*K);

	if (mat_get_at(*K,0,0)<0)
	{
		matrix *D=mat_identity (3);
		mat_set_at(D,0,0,-1);
		mat_set_at(D,1,1,-1);
		*K=mat_prod_new(*K,D);
		*R=mat_prod_new(D,*R);
		mat_free(D);
	}

	vektor *p_v4=mat_getcol (P, 3);
	matrix *p_m4=mat_alloc(3,1);
	mat_setcol (p_m4, 0 , p_v4);



	 //  test = K*R;
	//vgg_assert0(test/test(1,1) - H/H(1,1), 1e-07)
	/*
	matrix *test_mat=mat_prod_new(*K,*R);
	mat_scale(test_mat,1/mat_get_at(test_mat,0,0));
	matrix *H_scaled=mat_scale_new(H,1/mat_get_at(H,0,0));
	mat_print("test_mat",test_mat);
	mat_print("H_scaled",H_scaled);
	mat_print("sub",mat_sub_new(test_mat,H_scaled));
	mat_free(test_mat);
	mat_free(H_scaled);
	*/

	//calc. of t
	mat_invert(H);
	mat_scale(H,-1);
	*t=mat_prod_new(H,p_m4);

	mat_free(H);
	mat_free(p_m4);
	vec_free(p_v4);

	return 1;
}

bool P_from_KRt(matrix *K, matrix*R, matrix *t, matrix **P)
{
	*P = mat_alloc (3, 4);
	matrix *KR;
	homo2 d;

	KR = mat_prod_new(K, R);

    	d.x = mat_get_at(t,0,0);
    	d.y = mat_get_at(t,1,0);
    	d.w = mat_get_at(t,2,0);
	d = TransformHomo(KR, &d);

        mat_copy (KR, *P);
	KR = mat_free (KR);

	mat_set_at(*P,0,3,-d.x);
	mat_set_at(*P,1,3,-d.y);
	mat_set_at(*P,2,3,-d.w);

	NormCamera(*P);

return 1;
}

void JoinDual(double A[4],double B[4],double *C)
{
	  C[0]=A[3]*B[0]-A[0]*B[3];
	  C[1]=A[3]*B[1]-A[1]*B[3];
	  C[2]=A[3]*B[2]-A[2]*B[3];
	  C[3]=-A[2]*B[1]+A[1]*B[2];
	  C[4]=A[2]*B[0]-A[0]*B[2];
	  C[5]=-A[1]*B[0]+A[0]*B[1];

}

bool Q_from_P(double *P, vector<double> &Q)
{
	double L[6];

	     JoinDual(&P[4],&P[8],L);
	     Q[0]=L[0];
	     Q[1]=L[1];
	     Q[2]=L[2];
	     Q[3]=L[3];
	     Q[4]=L[4];
	     Q[5]=L[5];

	     JoinDual(&P[8],&P[0],L);
	     Q[6]=L[0];
	     Q[7]=L[1];
	     Q[8]=L[2];
	     Q[9]=L[3];
	     Q[10]=L[4];
	     Q[11]=L[5];

	     JoinDual(&P[0],&P[4],L);
	     Q[12]=L[0];
	     Q[13]=L[1];
	     Q[14]=L[2];
	     Q[15]=L[3];
	     Q[16]=L[4];
	     Q[17]=L[5];

	   return true;
}

void lQL(vector<double> Q, vector<double> L, vector<double> &l)
{
l.resize(3);
l[0]=Q[0]*L[0]+Q[1]*L[1]+Q[2]*L[2]+Q[3]*L[3]+Q[4]*L[4]+Q[5]*L[5];
l[1]=Q[6]*L[0]+Q[7]*L[1]+Q[8]*L[2]+Q[9]*L[3]+Q[10]*L[4]+Q[11]*L[5];
l[2]=Q[12]*L[0]+Q[13]*L[1]+Q[14]*L[2]+Q[15]*L[3]+Q[16]*L[4]+Q[17]*L[5];

double d=sqrt(l[0]*l[0]+l[1]*l[1]);

//normalize
l[0]/=d;
l[1]/=d;
l[2]/=d;
}

void lQL(vector<double> Q, matrix* L, vector<double> &l)
{
	vector<double> Lvec; Lvec.resize(6);
		for (int i=0; i<6;i++)
			Lvec[i]=L->m[i][0];

		lQL(Q,Lvec,l);

}

void dlQdLi_num(vector<double> Q, vector<double> L, int L_index, double *dl1, double *dl2, double *dl3)
{

	double h=1E-6;
	vector<double> l_min, l_plus;

	L[L_index]+=h;
	lQL(Q,L,l_plus);

	L[L_index]-=(2*h);
	lQL(Q,L,l_min);

	*dl1=(l_plus[0]-l_min[0])/(2*h);
	*dl2=(l_plus[1]-l_min[1])/(2*h);
	*dl3=(l_plus[2]-l_min[2])/(2*h);

	L[L_index]+=(h);

}

void dlQdLi_num(vector<double> Q, matrix* L, int L_index, double *dl1, double *dl2, double *dl3)
{
	vector<double> Lvec; Lvec.resize(6);
	for (int i=0; i<6;i++)
		Lvec[i]=L->m[i][0];

	dlQdLi_num(Q,Lvec,L_index,dl1,dl2,dl3);
}

bool F_from_P(double *P_left, double *P_right, matrix**F)
{
	/*
	X1 = P1([2 3],:); //attention: here index-1
	X2 = P1([3 1],:);
	X3 = P1([1 2],:);
	Y1 = P2([2 3],:);
	Y2 = P2([3 1],:);
	Y3 = P2([1 2],:);

	F = [det([X1; Y1]) det([X2; Y1]) det([X3; Y1])
	     det([X1; Y2]) det([X2; Y2]) det([X3; Y2])
	     det([X1; Y3]) det([X2; Y3]) det([X3; Y3])];
	*/

	matrix *P_left_mat;
		P_left_mat=mat_alloc(3,4);
	matrix *P_right_mat;
		P_right_mat=mat_alloc(3,4);


		//put data into P_left/right_mat;
		int count=0;
		for(int row=0;row<=2;row++)
			for(int col=0;col<=3;col++)
			{
			mat_set_at(P_left_mat,row,col,P_left[count]);
			mat_set_at(P_right_mat,row,col,P_right[count]);
			count++;
			}

	matrix *X1,*X2, *X3;
		X1=mat_alloc(2,4);
		X2=mat_alloc(2,4);
		X3=mat_alloc(2,4);
	matrix *Y1,*Y2, *Y3;
		Y1=mat_alloc(2,4);
		Y2=mat_alloc(2,4);
		Y3=mat_alloc(2,4);

		mat_copyrow(P_left_mat,1,X1,0);
		mat_copyrow(P_left_mat,2,X1,1);

		mat_copyrow(P_left_mat,2,X2,0);
		mat_copyrow(P_left_mat,0,X2,1);

		mat_copyrow(P_left_mat,0,X3,0);
		mat_copyrow(P_left_mat,1,X3,1);

		mat_copyrow(P_right_mat,1,Y1,0);
		mat_copyrow(P_right_mat,2,Y1,1);

		mat_copyrow(P_right_mat,2,Y2,0);
		mat_copyrow(P_right_mat,0,Y2,1);

		mat_copyrow(P_right_mat,0,Y3,0);
		mat_copyrow(P_right_mat,1,Y3,1);

		mat_print("P_left",P_left_mat);
		//mat_print("X1",X1); mat_print("X2",X2);mat_print("X3",X3);

		mat_print("P_right",P_right_mat);
		//mat_print("Y1",Y1); mat_print("Y2",Y2);mat_print("Y3",Y3);

		/*
		 F = [det([X1; Y1]) det([X2; Y1]) det([X3; Y1])
	     det([X1; Y2]) det([X2; Y2]) det([X3; Y2])
	     det([X1; Y3]) det([X2; Y3]) det([X3; Y3])];
	     */

		*F=mat_alloc(3,3);
		//mat_print("x1_stack_y1",mat_stack_rows(X1,Y1));

		mat_set_at(*F,0,0,mat_det(mat_stack_rows(X1,Y1)));
			mat_set_at(*F,0,1,mat_det(mat_stack_rows(X2,Y1)));
				mat_set_at(*F,0,2,mat_det(mat_stack_rows(X3,Y1)));

		mat_set_at(*F,1,0,mat_det(mat_stack_rows(X1,Y2)));
			mat_set_at(*F,1,1,mat_det(mat_stack_rows(X2,Y2)));
				mat_set_at(*F,1,2,mat_det(mat_stack_rows(X3,Y2)));

		mat_set_at(*F,2,0,mat_det(mat_stack_rows(X1,Y3)));
					mat_set_at(*F,2,1,mat_det(mat_stack_rows(X2,Y3)));
						mat_set_at(*F,2,2,mat_det(mat_stack_rows(X3,Y3)));

	//normalize F, to F(3,3)==1??
    double scale=1/mat_get_at(*F,2,2);
    printf("F-Matrix: element 3,3, before norm:%lf ==> scale=%lf\n",mat_get_at(*F,2,2),scale);
	if (ISZERO(scale)) {scale=TINY*2; if (mat_get_at(*F,2,2) <0) scale*=-1;printf("Attention: scale is zero, set to TINY*2!:%e\n",scale);}//return 0;}
	mat_scale(*F,scale);

	int rank_F=mat_rank(*F);
	if (rank_F !=2) {printf("Error: rank of F is not 2, it is %d, exit!\n", rank_F); return 0;}

	mat_free(P_left_mat);
	mat_free(P_right_mat);
	mat_free(X1);
	mat_free(X2);
	mat_free(X3);
	mat_free(Y1);
	mat_free(Y2);
	mat_free(Y3);

	return 1;
}

bool check_value_in_vec(vector<int> &v, int val)
{
	for (uint i=0; i<v.size();i++)
		if (v[i]==val) return 1;

	return 0;
}

bool check_value_in_vec(vector<unsigned int> &v, unsigned int val)
{
	for (uint i=0; i<v.size();i++)
		if (v[i]==val) return 1;

	return 0;
}

bool get_lines(int view, const char* dirname, const char* image, ImageLines  &ILI){
	//loads (eventually first extracts) edges from file and writes to the 2d edge structure
	
	char imagename[500];
	strcpy(imagename,image);

	char line_file[500];
	sprintf(line_file,"%s/imagelines-%d.dat",dirname,view);

	if (!ILI.Read(line_file))
	{
       	printf("%s not found on disk: extract lines now, imagename=%s!\n", line_file, imagename);
		//CALL extraction. This gives back directly imagelines_tmp
		//Default options for line growing int win=3, int t=1000, double minl=10.0, double maxw=3.0);
       	//use the following settings to obain only one line per microdrone img which is valid 3,1000,330
		//if (!extractlines_grow(image_file,line_file,ILI,3,300,150)) {printf("error during line extraction (grow)\n"); return 0;}

       	//if (!extractlines_grow(image_file,line_file,ILI,3,90,35)) {printf("error during line extraction (grow)\n"); return 0;}
       	//if (!extractlines_grow(image_file,line_file,ILI,2,20,19)) {printf("error during line extraction (grow)\n"); return 0;}

       	//default options for burns , int bucket_width=8, int min_num_pixels=25, double min_magnitude=5.0, double vote=0.5, int gradient_mask=2
       	//burns-parameter before debug where only long lines are used: image_file,line_file,ILI, 8, 70, 8, 0.8, 0
		if (!extractlines_burns(imagename,line_file,ILI, 8, /*350*/200, 5, 0.8, 0)) {printf("error during line extraction (burns)\n"); return 0;}
		else printf("successfully extracted lines and wrote to %s for later use\n",line_file);
	}
	else printf("successfully loaded lines from %s\n",line_file);

	//printf("end of get_lines\n");
	return 1;
}
