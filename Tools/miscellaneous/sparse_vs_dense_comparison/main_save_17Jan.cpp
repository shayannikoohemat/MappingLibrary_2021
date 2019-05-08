#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <math.h>
#include "LaserPoints.h"


#include "DataBounds3D.h"
#include "KNNFinder.h"

using namespace std;

//this program simply takes two laserfiles, puts one in the kdtree and for each point in the other one finds the closest on
//done for the PFG paper to assign GPS positions (from all GPS readings, needed to compute the speed) and a set of XY and error in one direction
//
using namespace std;




void PrintUsage()
{
printf("\nThis small software reads a reference (sparse point cloud) [first argument] and another (dense PC) [second argument] and finds for each point in the ref the n (10, hardcoded) closest points (XY) (if distance is smaller than 1m (hard coded). The Z-difference for each point is printed to file (i.e. per xy in reference there are max 10 height differences\n");
  printf("\nUsage: sparse_vs_dense_comparison ref.las/er dense.las/er\n");
} 

int main(int argc, char *argv[])
{

LaserPoints refLS, denseLS;

 if (argc!=3)
 {
    printf("Error: not all arguments given\n");
    PrintUsage();
    exit(0);
  }
 
//read the reference file
printf("\n Reading reference cloud ...\n");
refLS.ReInitialise();
	  if (!refLS.Read(argv[1],1)) {
		  fprintf(stderr, "Error reading filefor speed\n");
		  exit(0);
		}
		
//read the tiepoint file
printf("\n Reading dense cloud ... \n");
denseLS.ReInitialise();
	  if (!denseLS.Read(argv[2],1)) {
		  fprintf(stderr, "Error reading dense file\n");
		  exit(0);
		}


		
  FILE* outputF;
	  if ((outputF=fopen("/tmp/distances.txt","w")) == NULL)
		{
			printf("CAN NOT OPEN outputfile!\n");
			return(0);
		}
//Set up the knnfinder for the dense file
KNNFinder<LaserPoint> finder_dense(denseLS, 2); //initiaize with 2 dimensions!!!

//ietrate over the referenceL and for each poitn find the closest one from dense
int NUMBER_THRESH=10;
double DIST_THRESH=1.0;

    for (int i=0; i<refLS.size(); i++)
	  {
	   Vector3D v(refLS[i]);
	   Vector2D v_2_ref(refLS[i]);
	   

 	  vector<double> matching_error(0);
         for (int k=1;k<=NUMBER_THRESH;k++) //break the loop  when the distance becomes too big
	   {
	     int index_hit; double distance_hit;
	     finder_dense.FindIndexAndDistance (v, index_hit, distance_hit, k, DIST_THRESH*2); //for knn be more relaxed re dist thresh
	     
	      //distance: only 2D!!!
	    Vector2D v_2_test(denseLS[index_hit]);
	      
	      double distanceP=(v_2_ref-v_2_test).Length();
	      
	      if (distanceP<=DIST_THRESH) matching_error.push_back(denseLS[index_hit].GetZ()-refLS[i].GetZ());
		else break;
		
	   }


	    
	 //fprintf(outputF,"%.3f %.3f %.2f %.2f\n",refLS[i].GetX(), refLS[i].GetY(),speedLS[index].GetZ(),errorLS[i].GetZ());
         fprintf(outputF,"%.3f %.3f",refLS[i].GetX(), refLS[i].GetY());
	for (int j=0;j<matching_error.size();j++) fprintf(outputF," %.2f",matching_error[j]); fprintf(outputF," \n"); 
 
	  
	  }

	
	  fclose(outputF);
printf("see /tmp/distances.txt\n");
return EXIT_SUCCESS;
}
