#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <math.h>
#include "LaserPoints.h"
#include "InlineArguments.h"
#include <map>

#include "DataBounds3D.h"
#include "KNNFinder.h"

using namespace std;


using namespace std;

 

void PrintUsage()
{
printf("AHN2_normalize_heights: takes as input two AHN2 files of the same tile: ground only (gxxx) and above ground file (uxx). It then computes the normalized heights, i.e. shifts the above ground points to the level of the surrounding ground");
  printf("\nUsage: \tAHN2_normalize_heights -ground <las- or laz- or laserfile> -above_ground <las- or laz- or laserfile> -o <output las name>\n");
} 

int main(int argc, char *argv[])
{


printf("AHN2_normalize_heights\n");



 InlineArguments *args = new InlineArguments(argc, argv);
 
 LaserPoints g_l, a_l, o_l;
 o_l.ReInitialise();
   
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-ground") || !args->Contains("-above_ground") || !args->Contains("-o"))
 {
    printf("Error: not all arguments given\n");
    PrintUsage();
    exit(0);
  }
 
//read the ground 
printf("\n Reading ground point cloud ...\n");
g_l.ReInitialise();
	  if (!g_l.Read(args->String("-ground"),1)) {
		  fprintf(stderr, "Error reading file %s\n", args->String("-ground"));
		  exit(0);
		}
		
//read the above ground points
printf("\n Reading above ground points...\n");
a_l.ReInitialise();
	  if (!a_l.Read(args->String("-above_ground"),1)) {
		  fprintf(stderr, "Error reading file %s\n", args->String("-above_ground"));
		  exit(0);
		}
//set up the kn tree
  KNNFinder<LaserPoint> finder_ls_ground(g_l);
   
   for (int i=0;i<a_l.size();i++) //iterate over all above ground points
   {
 
      Vector3D v(a_l[i]);
      int index = finder_ls_ground.FindIndex(v,1); //find the closest point in the ground points
      double distanceP=(g_l[index]-v).Length();
      
      
      double Z_this=a_l[i].GetZ();
      double Z_ground=g_l[index].GetZ();
      double normalized=Z_this-Z_ground;
      normalized=normalized>=0?normalized:0;
      
      o_l.push_back(LaserPoint(a_l[i].GetX(),a_l[i].GetY(),normalized));
   }
   
   o_l.Write(args->String("-o"),0);
   
return EXIT_SUCCESS;
}
