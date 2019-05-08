#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <math.h>
#include "LaserPoints.h"
#include "InlineArguments.h"

typedef unsigned int uint;

using namespace std;



void PrintUsage()
{
printf("ExportSegmentPlaneParameters: Fits a plane to the segment (only one!!!) given in a point cloud and exports the plane parameters: XYZ offset vector, XYZ normal direction vector \n\n");
  printf("Usage: \tprojectLaserToImage -i <input laserfile> -o <output textfile with plane parameters>\n\n");
} 


int main(int argc, char *argv[])
{

printf("ExportSegmentPlaneParameters\n");

 InlineArguments *args = new InlineArguments(argc, argv);
 
 LaserPoints l;
  
  if (args->Contains("-usage")) {
    PrintUsage();
    exit(0);
  }
                       
  if (!args->Contains("-i") || !args->Contains("-o")) {
    printf("Error: not all arguments given\n");
    PrintUsage();
    exit(0);
  }
  
 
//open laserpointsfile
printf("\n Reading laserpoints...\n");
l.ReInitialise();
	  if (!l.Read(args->String("-i"),1)) {
		  fprintf(stderr, "Error reading file %s\n", args->String("-i"));
		  exit(0);
		}

printf("...done\n");

vector< int >  list_of_segments= l.AttributeValues(SegmentNumberTag);

if (list_of_segments.size()!=1) {
 printf("Error: wrong number of segments. It must be 1, but it is %u\n",list_of_segments.size()); 
 return 0;
}

if (l.size()<3) {
 printf("Error: Less than 3 points in the pointcloud!\n"); 
 return 0;
}

FILE * out;  
//printf("\nwriting voxel information to %s\n",filename);
if ((out=fopen(args->String("-o"),"w")) == NULL)
		{
			printf("CANNOT OPEN %s!\n", args->String("-o"));
			return 0;
		}
 
 

//fit a plane to the points

Plane plorig(l[0],l[1],l[2]);
     for(uint k=3;k<l.size();k++)
       plorig.AddPoint(l[k],true);
     
Vector3D PlaneNormal=plorig.Normal();
Position3D PlaneCentreGrav=plorig.CentreOfGravity();

printf("point on the plane (centre of gravity): %.4f %.4f %.4f\n",PlaneCentreGrav.X(),PlaneCentreGrav.Y(),PlaneCentreGrav.Z());
printf("normal vector: %.5f %.5f %.5f\n",PlaneNormal.X(),PlaneNormal.Y(),PlaneNormal.Z());

fprintf(out,"%.4f %.4f %.4f\n",PlaneCentreGrav.X(),PlaneCentreGrav.Y(),PlaneCentreGrav.Z());
fprintf(out,"%.5f %.5f %.5f\n",PlaneNormal.X(),PlaneNormal.Y(),PlaneNormal.Z());

 printf("wrote plane parameters to %s\n",args->String("-o"));
  return EXIT_SUCCESS;
}
