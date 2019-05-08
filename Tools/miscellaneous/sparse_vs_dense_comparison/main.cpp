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
printf("\nThis small software reads a reference (sparse point cloud) [first argument] and another (dense PC) [second argument] and finds for each point in the ref the n (10, hardcoded) closest points (XY) (if distance is smaller than 1m (hard coded). The X Y Z-difference for each point is printed to file (i.e. per xy in reference there are max 10 height differences\n");
  printf("\nUsage: sparse_vs_dense_comparison ref.las/er dense.las/er basename_for_outputfile\n");
} 

int main(int argc, char *argv[])
{

LaserPoints refLS, denseLS;

 if (argc!=4)
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


		
FILE* outputF_2D_X,*outputF_2D_Y,*outputF_2D_Z;
char fname[255];
strcpy(fname,argv[3]);
strcat(fname,"_2D_deltaX.txt");

printf("outputname, for example 2D:%s\n",fname);

	  if ((outputF_2D_X=fopen(fname,"w")) == NULL)
		{
			printf("CAN NOT OPEN outputfile!\n");
			return(0);
		}

strcpy(fname,argv[3]);
strcat(fname,"_2D_deltaY.txt");

	  if ((outputF_2D_Y=fopen(fname,"w")) == NULL)
		{
			printf("CAN NOT OPEN outputfile!\n");
			return(0);
		}

strcpy(fname,argv[3]);
strcat(fname,"_2D_deltaZ.txt");

	  if ((outputF_2D_Z=fopen(fname,"w")) == NULL)
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
	   

 	  vector<double> matching_errorX(0),matching_errorY(0),matching_errorZ(0);
         for (int k=1;k<=NUMBER_THRESH;k++) //break the loop  when the distance becomes too big
	   {
	     int index_hit; double distance_hit;
	     finder_dense.FindIndexAndDistance (v, index_hit, distance_hit, k, DIST_THRESH*2); //for knn be more relaxed re dist thresh
	     
	      //distance: only 2D!!!
	    Vector2D v_2_test(denseLS[index_hit]);
	      
	      double distanceP=(v_2_ref-v_2_test).Length();
	      
	      if (distanceP<=DIST_THRESH) 
		{
		matching_errorX.push_back(denseLS[index_hit].GetX()-refLS[i].GetX());
		matching_errorY.push_back(denseLS[index_hit].GetY()-refLS[i].GetY());
		matching_errorZ.push_back(denseLS[index_hit].GetZ()-refLS[i].GetZ());
		}
		else break;
		
	   }


	    
	 //fprintf(outputF,"%.3f %.3f %.2f %.2f\n",refLS[i].GetX(), refLS[i].GetY(),speedLS[index].GetZ(),errorLS[i].GetZ());
         fprintf(outputF_2D_X,"%.3f %.3f",refLS[i].GetX(), refLS[i].GetY());
	fprintf(outputF_2D_Y,"%.3f %.3f",refLS[i].GetX(), refLS[i].GetY());
	fprintf(outputF_2D_Z,"%.3f %.3f",refLS[i].GetX(), refLS[i].GetY());
	
	for (int j=0;j<matching_errorZ.size();j++) 
	{fprintf(outputF_2D_X," %.2f",matching_errorX[j]); 
	fprintf(outputF_2D_Y," %.2f",matching_errorY[j]); 
	fprintf(outputF_2D_Z," %.2f",matching_errorZ[j]); 
	}

	fprintf(outputF_2D_X," \n"); fprintf(outputF_2D_Y," \n"); fprintf(outputF_2D_Z," \n"); 
 
	  
	  }

	
	  fclose(outputF_2D_X);fclose(outputF_2D_Y);fclose(outputF_2D_Z);

finder_dense.FreeResources();

//NOW DO THE SAME FOR 3D KNN!!!
FILE* outputF_3D_X,*outputF_3D_Y,*outputF_3D_Z;
strcpy(fname,argv[3]);
strcat(fname,"_3D_deltaX.txt");

printf("outputname, for example 3D:%s\n",fname);

	  if ((outputF_3D_X=fopen(fname,"w")) == NULL)
		{
			printf("CAN NOT OPEN outputfile!\n");
			return(0);
		}

strcpy(fname,argv[3]);
strcat(fname,"_3D_deltaY.txt");

	  if ((outputF_3D_Y=fopen(fname,"w")) == NULL)
		{
			printf("CAN NOT OPEN outputfile!\n");
			return(0);
		}

strcpy(fname,argv[3]);
strcat(fname,"_3D_deltaZ.txt");

	  if ((outputF_3D_Z=fopen(fname,"w")) == NULL)
		{
			printf("CAN NOT OPEN outputfile!\n");
			return(0);
		}

//Set up the knnfinder for the dense file
KNNFinder<LaserPoint> finder_dense3d(denseLS, 3); //initiaize with 3 dimensions!!!

//ietrate over the referenceL and for each poitn find the closest one from dense

    for (int i=0; i<refLS.size(); i++)
	  {
	   Vector3D v(refLS[i]);
	   
	   

 	  vector<double> matching_errorX(0),matching_errorY(0),matching_errorZ(0);
         for (int k=1;k<=NUMBER_THRESH;k++) //break the loop  when the distance becomes too big
	   {
	     int index_hit; double distance_hit;
	     finder_dense3d.FindIndexAndDistance (v, index_hit, distance_hit, k, DIST_THRESH*2); //for knn be more relaxed re dist thresh
	     
	      //distance: now in 3D!!!
	    Vector3D v_3_test(denseLS[index_hit]);
	      
	      double distanceP=(v-v_3_test).Length();
	      
	      if (distanceP<=DIST_THRESH) 
		{
		matching_errorX.push_back(denseLS[index_hit].GetX()-refLS[i].GetX());
		matching_errorY.push_back(denseLS[index_hit].GetY()-refLS[i].GetY());
		matching_errorZ.push_back(denseLS[index_hit].GetZ()-refLS[i].GetZ());
		}
		else break;
		
	   }


	    
	 //fprintf(outputF,"%.3f %.3f %.2f %.2f\n",refLS[i].GetX(), refLS[i].GetY(),speedLS[index].GetZ(),errorLS[i].GetZ());
         fprintf(outputF_3D_X,"%.3f %.3f",refLS[i].GetX(), refLS[i].GetY());
	fprintf(outputF_3D_Y,"%.3f %.3f",refLS[i].GetX(), refLS[i].GetY());
	fprintf(outputF_3D_Z,"%.3f %.3f",refLS[i].GetX(), refLS[i].GetY());
	
	for (int j=0;j<matching_errorZ.size();j++) 
	{fprintf(outputF_3D_X," %.2f",matching_errorX[j]); 
	fprintf(outputF_3D_Y," %.2f",matching_errorY[j]); 
	fprintf(outputF_3D_Z," %.2f",matching_errorZ[j]); 
	}

	fprintf(outputF_3D_X," \n"); fprintf(outputF_3D_Y," \n"); fprintf(outputF_3D_Z," \n"); 
 
	  
	  }

	
	  fclose(outputF_3D_X);fclose(outputF_3D_Y);fclose(outputF_3D_Z);


return EXIT_SUCCESS;
}
