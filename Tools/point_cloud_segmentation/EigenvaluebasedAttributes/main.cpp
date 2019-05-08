  #include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <math.h>

#include "LaserPoints.h"
#include "InlineArguments.h"


#include "DataBounds3D.h"
#include "cv.h"
#include "highgui.h"
#include "KNNFinder.h"


using namespace std;

/*

CVAPI(void)  cvCalcCovarMatrix( const CvArr** vects, int count,
                                CvArr* cov_mat, CvArr* avg, int flags );

//Finds eigen values and vectors of a symmetric matrix
CVAPI(void)  cvEigenVV( CvArr* mat, CvArr* evects, CvArr* evals,
                        double eps CV_DEFAULT(0),
                        int lowindex CV_DEFAULT(-1),
                        int highindex CV_DEFAULT(-1));


*/

void PrintUsage()
{
printf("EigenvaluebasedAttributes: computes for each point the covariance matrix from the n (-knn) neighbors and from that structure tensor some features, like scattering, eigen entropy, curvature etc. In addition after that, again per point all the values of a certain point feature in the same neighborhood are collected and the std.dev. is computed, to have an idea about the behaviour in the neighborhood\n\n");
  printf("Usage: \tEigenvaluebasedAttributes -i <input laserfile>  -o<prefix of name of outputfile (extension NOT necessary)>\n\n");
printf("optional: -knn xx for neighborhood search (default:10) -knnAUTO (enables the optimization of search for K, see Diss. Weinmann, search for K within -knnAUTOStart...-knnAUTOStop in steps of 2) -INACHUS (just print out std_nz in the format x y z r g b std_nz, nothing more, to be converted to LAS\n");
} 

int median(vector <int> &dat)
{
  //printf("given data in median:\n");
  //for (int i=0; i< dat.size(); i++)
  //  printf("data: %.1lf\n",dat[i]);
  
  std::sort(dat.begin(), dat.end()); 
  
 //printf("return val=%.0f\n",dat[(uint) floor(dat.size()/2)]);
 // printf("\n\n");
  
  return dat[(uint) floor(dat.size()/2)];
}

int main(int argc, char *argv[])
{


LaserPoints l;
LaserPoints::iterator point;


printf("EigenvaluebasedAttributes\n");

 InlineArguments *args = new InlineArguments(argc, argv);
 
  
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i")  || !args->Contains("-o")) {
    printf("Error: not all arguments given\n");
    PrintUsage();
    exit(0);
  }
 

int kNN=args->Integer("-knn",10);

int kNN_autostart=args->Integer("-knnAUTOStart",15);
int kNN_autostop=args->Integer("-knnAUTOStop",50);


//open laserpointsfile
printf("\n Reading laserpoints...\n");
l.ReInitialise();
	  if (!l.Read(args->String("-i"),1)) {
		  fprintf(stderr, "Error reading file %s\n", args->String("-i"));
		  exit(0);
		}

printf("...done\n");


if (args->Contains("-INACHUS")) printf("Switch INACHUS set - only computing std_nz\n");

if (args->Contains("-knnAUTO")) printf("An automatic search for best k for each point is implemented, based on minimum Entropy, see Diss. Weinmann, start with %d, stop at %d for search, stepsize 2\n", kNN_autostart, kNN_autostop);

KNNFinder<LaserPoint> finder(l);

int num_points=l.size();
vector<float> scatter(num_points), linearity(num_points), planarity(num_points), curvature(num_points),entropy(num_points); //per point store the feature value. This is because we later want to iterate again over those and dreive some statistics in a neighborhood.
vector<float> std_scatter(num_points),std_linearity(num_points), std_planarity(num_points), std_curvature(num_points), std_entropy(num_points); //...the std.dev of this feature in the neighborhood

vector<float> nz(num_points);
vector<float> std_nz(num_points);

//in case we need to find the optimum k we store it per point since it is needed in the second loop for nz
//it is initialized with KNN, so later we do not need to switch between automatic or given KNN
vector<int> optK(num_points); for (int i=0;i<num_points;i++) optK[i]=kNN;

if (!args->Contains("-INACHUS"))
for (uint i=0;i<l.size();i++)
   {
 
      Vector3D v(l[i]);  
      
	if (args->Contains("-knnAUTO")) //need to look for best k first
	  {
	    double min_entropy=1.5; int min_entropy_k;
  
	    //iterate over all possible ks - hard coded!!!
	    for (int kl=kNN_autostart;kl<=kNN_autostop;kl+=2)
	    {//stupid copy and paste....
	      
	       vector <int>  neighboors = finder.FindIndices (v, kl);
		  //setup the matrix for the calccov
		  CvMat** srcMatrix = new CvMat*[kl+1];
		  
		  for (int k=0;k<=kl;k++)  //one more because we also have the point in question to add	
			  {srcMatrix[k]=cvCreateMat(1,3,CV_32FC1);cvZero(srcMatrix[k]);}

		  CvMat *dstMatrix=cvCreateMat(3,3,CV_32FC1);
		  CvMat *avgVector=cvCreateMat(1,3,CV_32FC1);
		  CvMat *evects=cvCreateMat(3,3,CV_32FC1);
		  CvMat *evals=cvCreateMat(3,1,CV_32FC1);
		  cvZero(evects);
		  cvZero(evals);
		  
		  cvmSet(srcMatrix[0], 0, 0, l[i].GetX());
		  cvmSet(srcMatrix[0], 0, 1, l[i].GetY());
		  cvmSet(srcMatrix[0], 0, 2, l[i].GetZ());

		  for (int k=1;k<=kl;k++)
		  {
		  cvmSet(srcMatrix[k], 0, 0, l[neighboors[k-1]].GetX());
		  cvmSet(srcMatrix[k], 0, 1, l[neighboors[k-1]].GetY());
		  cvmSet(srcMatrix[k], 0, 2, l[neighboors[k-1]].GetZ());

		  }

		  //DOIT
		  cvCalcCovarMatrix((const CvArr**)srcMatrix, kl,dstMatrix, avgVector,CV_COVAR_NORMAL);
		  
		  cvEigenVV( dstMatrix, evects, evals,DBL_EPSILON, 0, 0);
		  
		  
		  double lam1=cvmGet(evals, 0, 0);
		  double lam2=cvmGet(evals, 1, 0);
		  double lam3=cvmGet(evals, 2, 0);
  
		  //for the Entropy we need to noralize the lambda, avoid zero
		  lam1=MAX(lam1,1E-10);lam2=MAX(lam2,1E-10);lam3=MAX(lam3,1E-10);
		  double lam_sum=lam1+lam2+lam3;
		  lam1/=lam_sum; lam2/=lam_sum;lam3/=lam_sum;
		  double entropy=-1*(lam1*log(lam1)+lam2*log(lam2)+lam3*log(lam3));
		  
		  //check for the minimum
		  if (entropy<min_entropy){min_entropy=entropy;min_entropy_k=kl;}
		  
		  //FREE the MEM!DOUBLECHECK
		  
		  for (int k=0;k<=kl;k++) cvReleaseMat(&srcMatrix[k]); free(srcMatrix);

		  cvReleaseMat(&dstMatrix);
		  cvReleaseMat(&avgVector);
		  cvReleaseMat(&evects);
		  cvReleaseMat(&evals);
			
	    }
	    
	    optK[i]=min_entropy_k;
	    //printf("point %d, optimum k=%d\n",i,optK[i]);
	   }
      
     
     
      //so here we either have the default kNN per point, or the optimum one, stored in optK
      
      vector <int>  neighboors = finder.FindIndices (v, optK[i]);
	//setup the matrix for the calccov
	CvMat** srcMatrix = new CvMat*[optK[i]+1];
	
	for (int k=0;k<=optK[i];k++)  //one more because we also have the point in question to add	
		{srcMatrix[k]=cvCreateMat(1,3,CV_32FC1);cvZero(srcMatrix[k]);}

	CvMat *dstMatrix=cvCreateMat(3,3,CV_32FC1);
	CvMat *avgVector=cvCreateMat(1,3,CV_32FC1);
	CvMat *evects=cvCreateMat(3,3,CV_32FC1);
	CvMat *evals=cvCreateMat(3,1,CV_32FC1);
	cvZero(evects);
	cvZero(evals);
	
	//fill in the data. First: this point, then the neigbors
	//srcMatrix[0]->data.db[0] = l[i].GetX(); 
	//srcMatrix[0]->data.db[1] = l[i].GetY(); 
	//srcMatrix[0]->data.db[2] = l[i].GetZ(); 
	cvmSet(srcMatrix[0], 0, 0, l[i].GetX());
	cvmSet(srcMatrix[0], 0, 1, l[i].GetY());
	cvmSet(srcMatrix[0], 0, 2, l[i].GetZ());

	for (int k=1;k<=optK[i];k++)
	{
	cvmSet(srcMatrix[k], 0, 0, l[neighboors[k-1]].GetX());
	cvmSet(srcMatrix[k], 0, 1, l[neighboors[k-1]].GetY());
	cvmSet(srcMatrix[k], 0, 2, l[neighboors[k-1]].GetZ());

	//srcMatrix[k]->data.db[0] = l[neighboors[k-1]].GetX();
	//srcMatrix[k]->data.db[1] = l[neighboors[k-1]].GetY();	
	//srcMatrix[k]->data.db[2] = l[neighboors[k-1]].GetZ();
	}

	//DOIT
	cvCalcCovarMatrix((const CvArr**)srcMatrix, optK[i]+1,dstMatrix, avgVector,CV_COVAR_NORMAL);
	
	cvEigenVV( dstMatrix, evects, evals,DBL_EPSILON, 0, 0);
	
	
	double lam1=cvmGet(evals, 0, 0);
	double lam2=cvmGet(evals, 1, 0);
	double lam3=cvmGet(evals, 2, 0);

	//scatter
	scatter[i]=lam3/lam1;
	linearity[i]=(lam1-lam2)/lam1;
	planarity[i]=(lam2-lam3)/lam1;
	curvature[i]=lam3/(lam1+lam2+lam3);
	
	//for the Entropy we need to noralize the lambda, avoid zero
	lam1=MAX(lam1,1E-10);lam2=MAX(lam2,1E-10);lam3=MAX(lam3,1E-10);
	double lam_sum=lam1+lam2+lam3;
	lam1/=lam_sum; lam2/=lam_sum;lam3/=lam_sum;
	entropy[i]=-1*(lam1*log(lam1)+lam2*log(lam2)+lam3*log(lam3));

	//debug
	/*
	printf("This point: %.4f %.4f %.4f\n",l[i].GetX(),l[i].GetY(),l[i].GetZ());
	printf("mean\n");
	 for(int l = 0; l<3; l++)
                cout << cvmGet(avgVector, 0, l) << "\t";
        cout << endl; 
	
printf("Cov\n");
	for (int r=0; r<3;r++)
	{
	 for (int c=0;c<3;c++)
		printf("%.4f   ",cvmGet(dstMatrix, r, c));
	printf("\n");
	}
	
	printf("Lam1 %.4f, lam2 %.4f,  lam3 %.4f \n",lam1,lam2,lam3);
	*/
			
	

	//FREE the MEM!DOUBLECHECK
	
	for (int k=0;k<=optK[i];k++) cvReleaseMat(&srcMatrix[k]); free(srcMatrix);

	cvReleaseMat(&dstMatrix);
	cvReleaseMat(&avgVector);
	cvReleaseMat(&evects);
	cvReleaseMat(&evals);
	
	int percentage=(int) (100*i/l.size());
	if ((percentage % 1)==0) 
	printf("%d percent done\r",percentage);
  
   }
   
//ALSO add the z component of the local plane
//nz needs to be done anyways, also with -INACHUS

//does not work with the automatic approach to use the container function for Normals sinde we have indivicual k.s. However, a compromise is to comput the median optimum k
int medK=median(optK);
printf("median K=%d\n",medK);
vector<Vector3D> Normals=l.Normals(medK,1,NULL);
    for (uint i=0; i< l.size(); i++)  
    {
	    nz[i]=fabs(Normals[i].Z());
    }      

//normalize the std_nz to 0...1
double max_std_nz=0;
//now do the std dev in the neighborhood
//again iterate over each point, find the neigbors and compute mean and stddev for all the values
for (uint i=0;i<l.size();i++)
   {

      Vector3D v(l[i]);
      vector <int>  neighboors = finder.FindIndices (v, optK[i]);
	//setup the matrix for the calccov
	CvMat** srcMatrix1 = new CvMat*[optK[i]+1];
	
	for (int k=0;k<=optK[i];k++)  //one more because we also have the point in question to add	
		{srcMatrix1[k]=cvCreateMat(1,1,CV_32FC1);cvZero(srcMatrix1[k]);}

	CvMat *dstMatrix1=cvCreateMat(1,1,CV_32FC1);
	CvMat *avgVector1=cvCreateMat(1,1,CV_32FC1);
	
	//do the following per feature
	cvmSet(srcMatrix1[0], 0, 0, nz[i]);
	for (int k=1;k<=optK[i];k++) cvmSet(srcMatrix1[k], 0, 0, nz[neighboors[k-1]]);

	cvCalcCovarMatrix((const CvArr**)srcMatrix1, optK[i]+1,dstMatrix1, avgVector1,CV_COVAR_NORMAL);
	std_nz[i]=sqrt(cvmGet(dstMatrix1, 0, 0));

	if (max_std_nz < std_nz[i]) max_std_nz=std_nz[i];

	//this was std_nz -- if -INACHUS - continue...
	if (args->Contains("-INACHUS"))
	{
		//FREE the MEM!DOUBLECHECK
	
		for (int k=0;k<=optK[i];k++) cvReleaseMat(&srcMatrix1[k]); free(srcMatrix1);

		cvReleaseMat(&dstMatrix1);
		cvReleaseMat(&avgVector1);
		continue;
	}
	//debug
	/*
	printf("This point: %.4f %.4f %.4f \n",l[i].GetX(),l[i].GetY(),l[i].GetZ());
	printf("mean of scatter feature\n");
                cout << cvmGet(avgVector1, 0, 0) << "\t";

	
	printf("variance ");
		printf("%.4f   \n",cvmGet(dstMatrix1, 0, 0));
	*/
	
	
	//next feature	
        cvmSet(srcMatrix1[0], 0, 0, scatter[i]);
	for (int k=1;k<=optK[i];k++) cvmSet(srcMatrix1[k], 0, 0, scatter[neighboors[k-1]]);

	cvCalcCovarMatrix((const CvArr**)srcMatrix1, optK[i]+1,dstMatrix1, avgVector1,CV_COVAR_NORMAL);
	std_scatter[i]=sqrt(cvmGet(dstMatrix1, 0, 0));

	//next feature	
	cvmSet(srcMatrix1[0], 0, 0, linearity[i]);
	for (int k=1;k<=optK[i];k++) cvmSet(srcMatrix1[k], 0, 0, linearity[neighboors[k-1]]);

	cvCalcCovarMatrix((const CvArr**)srcMatrix1, optK[i]+1,dstMatrix1, avgVector1,CV_COVAR_NORMAL);
	std_linearity[i]=sqrt(cvmGet(dstMatrix1, 0, 0));

	//next feature	
	cvmSet(srcMatrix1[0], 0, 0, planarity[i]);
	for (int k=1;k<=optK[i];k++) cvmSet(srcMatrix1[k], 0, 0, planarity[neighboors[k-1]]);

	cvCalcCovarMatrix((const CvArr**)srcMatrix1, optK[i]+1,dstMatrix1, avgVector1,CV_COVAR_NORMAL);
	std_planarity[i]=sqrt(cvmGet(dstMatrix1, 0, 0));
 
	//next feature	
	cvmSet(srcMatrix1[0], 0, 0, curvature[i]);
	for (int k=1;k<=optK[i];k++) cvmSet(srcMatrix1[k], 0, 0, curvature[neighboors[k-1]]);

	cvCalcCovarMatrix((const CvArr**)srcMatrix1, optK[i]+1,dstMatrix1, avgVector1,CV_COVAR_NORMAL);
	std_curvature[i]=sqrt(cvmGet(dstMatrix1, 0, 0));

	//next feature	
	cvmSet(srcMatrix1[0], 0, 0, entropy[i]);
	for (int k=1;k<=optK[i];k++) cvmSet(srcMatrix1[k], 0, 0, entropy[neighboors[k-1]]);

	cvCalcCovarMatrix((const CvArr**)srcMatrix1, optK[i]+1,dstMatrix1, avgVector1,CV_COVAR_NORMAL);
	std_entropy[i]=sqrt(cvmGet(dstMatrix1, 0, 0));


	//FREE the MEM!DOUBLECHECK
	
	for (int k=0;k<=optK[i];k++) cvReleaseMat(&srcMatrix1[k]); free(srcMatrix1);

	cvReleaseMat(&dstMatrix1);
	cvReleaseMat(&avgVector1);
  
   }


char name_out[600];
//char name_matrix[600];

strcpy(name_out,args->String("-o"));
strcat(name_out,"_std_nz.pts");
  FILE *data_std_nz;
if ((data_std_nz=fopen(name_out,"w")) == NULL)
		{
			printf("CAN NOT OPEN %s!\n", name_out);
			exit(0);
		}

 FILE *data_scatter; FILE *data_std_scatter;  FILE *data_nz;  FILE *data_lin;   FILE *data_std_lin;  FILE *data_plan;
 FILE *data_std_plan;  FILE *data_curve;   FILE *data_std_curve;  FILE *data_std_entro;  FILE *data_entro;

if (!args->Contains("-INACHUS")) //only add the others if INAChUS is not set 
	{

	strcpy(name_out,args->String("-o"));
	strcat(name_out,"_scatter.pts");
	 
	if ((data_scatter=fopen(name_out,"w")) == NULL)
			{
				printf("CAN NOT OPEN %s!\n", name_out);
				exit(0);
			}

	strcpy(name_out,args->String("-o"));
	strcat(name_out,"_std_scatter.pts");
	 
	if ((data_std_scatter=fopen(name_out,"w")) == NULL)
			{
				printf("CAN NOT OPEN %s!\n", name_out);
				exit(0);
			}

	strcpy(name_out,args->String("-o"));
	strcat(name_out,"_nz.pts");
	 
	if ((data_nz=fopen(name_out,"w")) == NULL)
			{
				printf("CAN NOT OPEN %s!\n", name_out);
				exit(0);
			}


	strcpy(name_out,args->String("-o"));
	strcat(name_out,"_linearity.pts");
	 
	if ((data_lin=fopen(name_out,"w")) == NULL)
			{
				printf("CAN NOT OPEN %s!\n", name_out);
				exit(0);
			}

	strcpy(name_out,args->String("-o"));
	strcat(name_out,"_std_linearity.pts");
	
	if ((data_std_lin=fopen(name_out,"w")) == NULL)
			{
				printf("CAN NOT OPEN %s!\n", name_out);
				exit(0);
			}

	strcpy(name_out,args->String("-o"));
	strcat(name_out,"_planarity.pts");
	
	if ((data_plan=fopen(name_out,"w")) == NULL)
			{
				printf("CAN NOT OPEN %s!\n", name_out);
				exit(0);
			}

	strcpy(name_out,args->String("-o"));
	strcat(name_out,"_std_planarity.pts");

	if ((data_std_plan=fopen(name_out,"w")) == NULL)
			{
				printf("CAN NOT OPEN %s!\n", name_out);
				exit(0);
			}

	strcpy(name_out,args->String("-o"));
	strcat(name_out,"_curvature.pts");
	
	if ((data_curve=fopen(name_out,"w")) == NULL)
			{
				printf("CAN NOT OPEN %s!\n", name_out);
				exit(0);
			}

	strcpy(name_out,args->String("-o"));
	strcat(name_out,"_std_curvature.pts");
	
	if ((data_std_curve=fopen(name_out,"w")) == NULL)
			{
				printf("CAN NOT OPEN %s!\n", name_out);
				exit(0);
			}

	strcpy(name_out,args->String("-o"));
	strcat(name_out,"_entropy.pts");
	
	if ((data_entro=fopen(name_out,"w")) == NULL)
			{
				printf("CAN NOT OPEN %s!\n", name_out);
				exit(0);
			}

	strcpy(name_out,args->String("-o"));
	strcat(name_out,"_std_entropy.pts");
	
	if ((data_std_entro=fopen(name_out,"w")) == NULL)
			{
				printf("CAN NOT OPEN %s!\n", name_out);
				exit(0);
			}
}

for (uint i=0; i< l.size(); i++)
	  {
  	LaserPoint lp=l[i].LaserPointRef();
	    double X=lp.GetX();
	    double Y=lp.GetY();
	    double Z=lp.GetZ();
	    int R=lp.Red();
	     int G=lp.Green();
             int B=lp.Blue();

            fprintf(data_std_nz,"%.6f %.6f %.6f %d %d %d %.6f\n",X,Y,Z,R,G,B,std_nz[i]/max_std_nz); 	 

		if (!args->Contains("-INACHUS")) //only add the others if INAChUS is not set 
		  {
		   fprintf(data_scatter,"%.6f %.6f %.6f %.6f\n",X,Y,Z,scatter[i]); 
		   fprintf(data_std_scatter,"%.6f %.6f %.6f %.6f\n",X,Y,Z,std_scatter[i]); 
		   fprintf(data_nz,"%.6f %.6f %.6f %.6f\n",X,Y,Z,nz[i]); 
		   fprintf(data_lin,"%.6f %.6f %.6f %.6f\n",X,Y,Z,linearity[i]); 
		   fprintf(data_std_lin,"%.6f %.6f %.6f %.6f\n",X,Y,Z,std_linearity[i]); 
		   fprintf(data_plan,"%.6f %.6f %.6f %.6f\n",X,Y,Z,planarity[i]); 
		   fprintf(data_std_plan,"%.6f %.6f %.6f %.6f\n",X,Y,Z,std_planarity[i]);
		   fprintf(data_curve,"%.6f %.6f %.6f %.6f\n",X,Y,Z,curvature[i]); 
		   fprintf(data_std_curve,"%.6f %.6f %.6f %.6f\n",X,Y,Z,std_curvature[i]);
		   fprintf(data_entro,"%.6f %.6f %.6f %.6f\n",X,Y,Z,entropy[i]); 
		   fprintf(data_std_entro,"%.6f %.6f %.6f %.6f\n",X,Y,Z,std_entropy[i]);
		   }

	   }

fclose(data_std_nz);
if (!args->Contains("-INACHUS")) //only add the others if INAChUS is not set 
		  {
		fclose(data_scatter);
		fclose(data_std_scatter);
		fclose(data_nz);
		fclose(data_lin);
		fclose(data_std_lin);
		fclose(data_plan);
		fclose(data_std_plan);
		fclose(data_curve);
		fclose(data_std_curve);
		fclose(data_entro);
		fclose(data_std_entro);
	}

if (args->Contains("-INACHUS")) printf("use command such as txt2las (older versions do not work (scale intensity) txt2las -i ina_std_nz.pts -o ina_std_nz.las -parse xzyRGBi -scale_intensity 64000\n call of laspublish eg: laspublish -i ina_std_nz.las -rgb -copy_source_files -overwrite  -title \"testportal\" -o \"portal.html\" -odir \"testportal\" -olaz");

  return EXIT_SUCCESS;
}
