#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <math.h>
#include "LaserPoints.h"

//#include "InlineArguments.h"
#include "DataBounds3D.h"
#include "cv.h"
#include "highgui.h"
#include "KNNFinder.h"

using namespace std;

//this programm implements a simple filter which aims to remove outliers and noise. Aim is it adapts to the data, so no parameters needed. 
//two strategies are persued: large variation in local density is a hint to outliers, at the same time large local spread around a point can also be a hint. 
//approach:
//first do a connected component anaylsis - remove all clusters with less than 500 points (which is around 400 pixesl and with 2cm GSD a total area of eg 20x20 so 40x40cm). Max distance between points should be 10cm

//build a kd tree
//iterate over each nth point (to save time, n eg 10) and make some statistics regarding the neigborhood: from the next neighbor (only 1), collect all distances and compute the overall  median and some ranks, do the same for the 100th neigbor ,,,> far reach
//iterate again over each point and compute the 1st and 100th neighbor. Only accept if both are within the 95% (?) rank of the respective global value.
//in fact the rank indicates our assumption of how much noiseis in the data, so if we say 95% weassume 5% of noise

using namespace std;




void PrintUsage()
{
printf("\n");
  printf("\nUsage: simple_noise_outlier_filter <LASER-file>\n");
} 

int main(int argc, char *argv[])
{
  
 int n=10; //skip every n.th point for the initial statistics, however, as expected, the std is quite sensitive to the number of samples. Probablz not so helpful, but median and rank are insensitive
  
if (argc != 2) {printf("wrong arguments, just provide the laser file\n"); exit(0);}
  
LaserPoints LS, LSout;
LS.ReInitialise();
LSout.ReInitialise();

 
//read the reference file
printf("\n Reading cloud (.LASER)\n");

	  if (!LS.Read(argv[1],1)) {
		  fprintf(stderr, "Error reading pc\n");
		  exit(0);
		}

//connected components
TINEdges     *edges;
     SegmentationParameters* segmentation_parameters = new SegmentationParameters();
     segmentation_parameters->MaxDistanceInComponent()=0.1; 
     segmentation_parameters->MinNumberOfPointsComponent()=500;
     segmentation_parameters->ComponentAttribute()=SegmentNumberTag;
     segmentation_parameters->DistanceMetricDimension()=3;
     
     printf("Distance between components: %.1lf; min number of points per component:%d, distancemetric dimension: %d, store component number in %s\n",segmentation_parameters->MaxDistanceInComponent(),segmentation_parameters->MinNumberOfPointsComponent(), segmentation_parameters->	DistanceMetricDimension(),AttributeName(segmentation_parameters->ComponentAttribute(),1));
      // Derive the edges that define the neighbour relations
     edges = LS.DeriveEdges(*segmentation_parameters);
     
     
     // Remove long edges
    if (segmentation_parameters->MaxDistanceInComponent() > 0.0)
        LS.RemoveLongEdges(edges->TINEdgesRef(), 
                       segmentation_parameters->MaxDistanceInComponent(),
                       segmentation_parameters->DistanceMetricDimension()==3);
		       
     // Label the connected components
      LS.LabelComponents(edges->TINEdgesRef(), segmentation_parameters->ComponentAttribute());
    delete edges;
    delete segmentation_parameters;
    
    LS.RemoveSmallSegments(segmentation_parameters->ComponentAttribute(), segmentation_parameters->MinNumberOfPointsComponent());
   
    LS.Write("connected_components.laser",0);
    
  //end connected components

		
//Set up the knnfinder 
KNNFinder<LaserPoint> finder(LS,3); 

//compute initial statistics
//we will have ceil(LS.size()/n) distances, so we initilize a CV matrix of that size. CV matrix** is needed for the covariance/average computations using the cv methods
//int dim=ceil(LS.size()/n);
//CvMat** srcMatrix = new CvMat*[dim];

//for (int p=0;p<=dim;p++)  	
// {srcMatrix[p]=cvCreateMat(1,1,CV_32FC1);cvZero(srcMatrix[p]);}
 
 //for the rank we use ::vector
 vector <double> dists_1(0);
 vector <double> dists_10(0);

 //int index=0;
    for (int i=0; i<LS.size(); i+=n)
	  {
	   //per point get the next neighbor and put its distance to the matrix
	    Vector3D v(LS[i]);
	    double dist  = finder.FindDistance (v,2); //2, because 1 is the point itself)
	    //printf("dist=%.6lf\n", dist);
//	    cvmSet(srcMatrix[index], 0, 0, dist); index++;
	    
	    dists_1.push_back(dist);
	    dist  = finder.FindDistance (v,12); 
	    
	    dists_10.push_back(dist);
	    
	    
	  }

//compute covmatrix and then std dev, mean and some ranks (although mean should not be important)
//CvMat *dstMatrix=cvCreateMat(1,1,CV_32FC1);
//CvMat *avgVector=cvCreateMat(1,1,CV_32FC1);
//cvCalcCovarMatrix((const CvArr**)srcMatrix, dim,dstMatrix, avgVector,CV_COVAR_NORMAL);

//double std=sqrt(cvmGet(dstMatrix, 0, 0));
//double mean=cvmGet(avgVector, 0, 0);
//sort for ranks
std::sort(dists_1.begin(), dists_1.end()); 
  
double median_1=dists_1[(uint) floor(dists_1.size()/2)];
double rank_1=dists_1[(uint) floor(dists_1.size()*.95)]; //95% rank

std::sort(dists_10.begin(), dists_10.end()); 
  
double median_10=dists_10[(uint) floor(dists_10.size()/2)];
double rank_10=dists_10[(uint) floor(dists_10.size()*.95)];
  
//printf("stddev=%.4lf, mean=%.4lf, median=%.4lf, rank90=%.4lf\n", std, mean, median, rank_ninety);
printf("median_1=%.4lf, rank_1=%.4lf\n", median_1, rank_1);
printf("median_10=%.4lf, rank_10=%.4lf\n", median_10, rank_10);


dists_1.resize(0);
dists_10.resize(0);

//now iterate over all points. get again the 1st and 100th neigbor. Only accept the point if both distances are smaller than the respective rank90
  for (int i=0; i<LS.size(); i++)
	  {
	   //per point get the next neighbor and put its distance to the matrix
	    Vector3D v(LS[i]);
	    double dist_1  = finder.FindDistance (v,2); //2, because 1 is the point itself)
	    double dist_10  = finder.FindDistance (v,12); 
	    
	    if (dist_1 <= rank_1 && dist_10 <=rank_10) LSout.push_back(LS[i]);
	    
	  }

LSout.Write("filtered.laser",0);
//FREE the MEM!DOUBLECHECK
//	for (int p=0;p<=dim;p++)   cvReleaseMat(&srcMatrix[p]); free(srcMatrix);
//	cvReleaseMat(&dstMatrix);
//	cvReleaseMat(&avgVector);
	  
return EXIT_SUCCESS;
}
