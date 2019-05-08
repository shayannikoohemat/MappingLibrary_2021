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
printf("\n");
  printf("\nUsage: easy_registration\n");
} 

int main(int argc, char *argv[])
{

LaserPoints speedLS, errorLS;


 
//read the reference file
printf("\n Reading speed cloud ... (X Y , speed as Z)\n");
speedLS.ReInitialise();
	  if (!speedLS.Read("xy_speed.laser.gz",1)) {
		  fprintf(stderr, "Error reading filefor speed\n");
		  exit(0);
		}
		
//read the tiepoint file
printf("\n Reading error cloud ... ( X Y, error as Z)\n");
errorLS.ReInitialise();
	  if (!errorLS.Read("xy_error_NS.laser.gz",1)) {
		  fprintf(stderr, "Error reading error file\n");
		  exit(0);
		}
		
  FILE* outputF;
	  if ((outputF=fopen("/tmp/assignments.txt","w")) == NULL)
		{
			printf("CAN NOT OPEN outputfile!\n");
			return(0);
		}
//Set up the knnfinder for the speed file
KNNFinder<LaserPoint> finder_ref(speedLS, 2); //initiaize with 2 dimensions!!!

//ietrate over the error LS and for each poitn find the closest one from speed, i.e. the z is then the assigned spped
    for (int i=0; i<errorLS.size(); i++)
	  {
	   Vector3D v(errorLS[i]);
	   Vector2D v_2_er(errorLS[i]);
	   
	int index = finder_ref.FindIndex(v,1);
    
	
        Vector2D v_2_sp(speedLS[index]);
    
      double distanceP_2P=(v_2_er-v_2_sp).Length();
  
      if (distanceP_2P>1)  { 
	printf("no neighbor for %.2f %.2f, distance:%.2f\n",errorLS[i].GetX(), errorLS[i].GetY(), distanceP_2P);
	
	continue;
	}  
       //ouput: X Y speed error
	    
	 fprintf(outputF,"%.3f %.3f %.2f %.2f\n",errorLS[i].GetX(), errorLS[i].GetY(),speedLS[index].GetZ(),errorLS[i].GetZ());
 
	  
	  }

	
	  fclose(outputF);
return EXIT_SUCCESS;
}
