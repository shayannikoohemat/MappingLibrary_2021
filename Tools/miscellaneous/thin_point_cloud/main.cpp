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

//this programm implements a simple thinning alg. The pre-defined min distance between points is approximated:

//build a kd tree
//start with an arbitrary point, set it as "selected" (set an attribute)
//take iteratively the closests point and mark the first point  which is further away than the "min" dist as "selected" (if it is not "unselected" already (because then it is already close to another remaining point). All others (which are closer) mark "unselected". Continue with the latest "selected" with the procedure. The loop is ended when no point is selected anymore.

using namespace std;




void PrintUsage()
{
printf("\n");
  printf("\nUsage: thin_point_cloud <LASER-file> <min_dist>\n");
} 

int main(int argc, char *argv[])
{
  
 
if (argc != 3) {printf("wrong arguments, provide the laser file and a min_dist\n"); PrintUsage(); exit(0);}
  
LaserPoints LS, LSout;
LS.ReInitialise();
LSout.ReInitialise();

 
//read the reference file
printf("\n Reading cloud (.LASER)\n");

	  if (!LS.Read(argv[1],1)) {
		  fprintf(stderr, "Error reading pc\n");
		  exit(0);
		}

double min_dist=atof(argv[2]);

printf("min distance=%.2f\n",min_dist);

//set reflectance to 0 because this is used for marking the pitns (0: not touched yet, 1: do not take, 2: take it)
		
for (int i=0;i<LS.size();i++) LS[i].SetReflectance(0);


//Set up the knnfinder 
KNNFinder<LaserPoint> finder(LS,3); 

    for (int i=0;i<LS.size();i++) //i will be controlled within the loop
	  {
	
	   //mark this one as selected if it is not already selected. for this misuse the reflectance attribute (0: not touched yet, 1: do not take, 2: take it)
 		if (LS[i].Reflectance()!=0) continue;
		printf("%d\r",i);

		LS[i].SetReflectance(2);
	    	Vector3D v(LS[i]);

	   //the idea was to get the next neighbor iteratitvely, but it is expensive since in the knnfinder in order to get the nth point, first all 1...n-1 points are looked up.
	   //hence just put the 10 closests points 
	   vector<double> distance_hit_v;vector<int> index_hit_v;
		
	   finder.FindKnn(v,11,distance_hit_v,index_hit_v);

		for (int k=2;k<12;k++) //start with 2, because 1 is the point itself)
			{	
	    			double distance_hit=distance_hit_v[k];int index_hit=index_hit_v[k];
				
				//if this point has already a setting (1 or 2) go to the next k
				if (LS[index_hit].Reflectance()!=0) continue;

				if (distance_hit < min_dist)
					{//do not use this one: mark it not take and to go the next one
					 LS[index_hit].SetReflectance(1);
					}
				else 
					{//...it is further away - so select it and break the k loop
					 LS[index_hit].SetReflectance(2);
					 break; //break the k-loop
					}
		
	   		}//else - if we reach end of k-loop without finding a point we just go to the next i...
	    
	  }

//copy all points which are selected to the out-instance, set refl. again to 0 to make it a clean set
for (int i=0;i<LS.size();i++) if (LS[i].Reflectance()==2){ LS[i].SetReflectance(0); LSout.push_back(LS[i]);}


LSout.Write("thinned.laser",0);
//FREE the MEM!DOUBLECHECK
//	for (int p=0;p<=dim;p++)   cvReleaseMat(&srcMatrix[p]); free(srcMatrix);
//	cvReleaseMat(&dstMatrix);
//	cvReleaseMat(&avgVector);
	  
return EXIT_SUCCESS;
}
