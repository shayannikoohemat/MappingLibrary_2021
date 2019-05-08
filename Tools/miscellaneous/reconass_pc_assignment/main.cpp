#include <stdlib.h>
#include <stdio.h>
#include <cstdlib>
#include <iostream>
#include <vector>
#include <math.h>
#include "LaserPoints.h"

#include "InlineArguments.h"
#include "DataBounds3D.h"
#include "KNNFinder.h"

using namespace std;

//this programm takes two point clouds as input -ifc_pc: output from plane points assignment (i.e. an artificial pc computed from a bim model and assigned the ifc id) -matching_PC: the observed building, dense point cloud, same coordinate system (RECONASS pilot process). It puts one of the points in the kdtree and finds the closest other point
//output: 3 point cloud 1) matching_PC, annotated with the corresponding ID from IFC model 2) remaining non-assigned matching Points 3) remaining non used IFC points

using namespace std;




void PrintUsage()
{
printf("\n");
  printf("\nUsage: reconass_pc_assignment -ifc_pc <LASER-file> -matching_pc <LAS-file> \n");
} 

int main(int argc, char *argv[])
{
  
  double dist_tresh=0.4; //0.15 for the reconass model; //snapping distance_hit
  
InlineArguments *args = new InlineArguments(argc, argv);
                       
  if (!args->Contains("-ifc_pc") || !args->Contains("-matching_pc"))
 {
    printf("Error: not all arguments given\n");
    PrintUsage();
    exit(0);
  }
  
LaserPoints ifcLS, matchLS;


 
//read the reference file
printf("\n Reading ifc cloud (.LASER)\n");
ifcLS.ReInitialise();
	  if (!ifcLS.Read(args->String("-ifc_pc"),1)) {
		  fprintf(stderr, "Error reading ifc pc\n");
		  exit(0);
		}
		
//read the tiepoint file
printf("\n Reading matching point cloud (.LAS)\n");
matchLS.ReInitialise();
	  if (!matchLS.Read(args->String("-matching_pc"),1)) {
		  fprintf(stderr, "Error reading matching pc file\n");
		  exit(0);
		}
		
LaserPoints hitsLS, excessmatchLS, excessifcLS; hitsLS.ReInitialise(); excessifcLS.ReInitialise(); excessifcLS.ReInitialise();

  FILE* hits, *excessmatch, *excessifc;
	  if ((hits=fopen("matching_pc_with_id.pts","w")) == NULL)
		{
			printf("CAN NOT OPEN outputfile!\n");
			return(0);
		}

 if ((excessmatch=fopen("matching_pc_with_no_hit.pts","w")) == NULL)
		{
			printf("CAN NOT OPEN outputfile!\n");
			return(0);
		}	
		
 if ((excessifc=fopen("ifc_with_no_hit.pts","w")) == NULL)
		{
			printf("CAN NOT OPEN outputfile!\n");
			return(0);
		}	
				
		
//Set up the knnfinder for the speed file
KNNFinder<LaserPoint> finder_ifc(ifcLS,3); 

//ietrate over the matching and for each poitn find the closest one from ifc, distance threshold is a variable
//each point found write, together with the id of the ifc point (SegmentNumber) into th first file. If not, put the point in the second file. Mark hits in the IFC as found. Later then iterate over the ifc poitns and write all which are not assigned in the 3rd file
    for (int i=0; i<matchLS.size(); i++)
	  {
	   Vector3D v(matchLS[i]);
	   
	   printf ("%d from %d points\r",i+1,matchLS.size());
	   
		//take the next xx matches for a point until we reach the threshold. This is because the artifical pc from ifc might be denser than the other one
	    
	      for (int k=1;;k++)
	      {
		int index_hit; double distance_hit;
		finder_ifc.FindIndexAndDistance (v, index_hit, distance_hit, k);
		
		if (distance_hit <= dist_tresh)
		{
		  //hit .... write to file 1 (including SegmentNumber), set the flag for the ifcLS
		  
		  ifcLS[index_hit].Select();
		  
		  //only add this point if it is the first time (k==1), else continue
		  if (k==1)
		  {
		  int ifc_id=ifcLS[index_hit].SegmentNumber(); //might be that there is another segment id, but if the threshold is small it should not matter so much. Important is that the ifcLS gets the "select" attribute
		  matchLS[i].SetSegmentNumber(ifc_id);
		  hitsLS.push_back(matchLS[i]);
		  //omit coloer, because of unproper 16bit handling in lastools (pts export) and the mappinglib
		  //fprintf(hits,"%.3f  %.3f  %.3f %d %d %d %d\n",matchLS[i].GetX(), matchLS[i].GetY(),matchLS[i].GetZ(), matchLS[i].Red(), matchLS[i].Green(),
		  fprintf(hits,"%.3f  %.3f  %.3f %d\n",matchLS[i].GetX(), matchLS[i].GetY(),matchLS[i].GetZ(), ifc_id);
		  
		  } 
		  
		  else continue;
		}
		
		else
		{
		//no hit, but only if k is still 1 ... write to file 2
		
		if (k==1)
		  {excessmatchLS.push_back(matchLS[i]);
		  //fprintf(excessmatch,"%.3f  %.3f  %.3f %d %d %d\n",matchLS[i].GetX(), matchLS[i].GetY(),matchLS[i].GetZ(), matchLS[i].Red(), matchLS[i].Green(), matchLS[i].Blue());
		  fprintf(excessmatch,"%.3f  %.3f  %.3f\n",matchLS[i].GetX(), matchLS[i].GetY(),matchLS[i].GetZ());
		  }
		break; //break the k-loop here anyway;  
		}
	      }
	  }

	for (int i=0; i<ifcLS.size(); i++)
	    if (!ifcLS[i].IsSelected())
	    {
	      excessifcLS.push_back(ifcLS[i]);
	      fprintf(excessifc,"%.3f  %.3f  %.3f %d\n",ifcLS[i].GetX(), ifcLS[i].GetY(),ifcLS[i].GetZ(),ifcLS[i].SegmentNumber());
	    }
	  
	
	hitsLS.Write("matching_pc_with_id.laser",0);
	excessmatchLS.Write("matching_pc_with_no_hit.laser",0);
	excessifcLS.Write("ifc_with_no_hit.laser",0);
	
	  fclose(hits);
	  fclose(excessmatch);
	  fclose(excessifc);
	  
return EXIT_SUCCESS;
}
