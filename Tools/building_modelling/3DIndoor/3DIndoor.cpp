/*-----------------------------------------------------------
scanner data
Initial Creation:Biao Xiong
Data: Jan 22, 2014
------------------------------------------------------------*/
#include <stdio.h>
#include "InlineArguments.h"
//#include "vld.h"
#include "LineTopology.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"
#include "InferenceEngine.h"
#include "AdjacentGraph.h"

using namespace std;

void PrintUsage();

//-il 1.laser -ip 1.objpts -it 1.top -op 1_bld.objpts -ot 1_bld.top
//-il 5.laser -ip 5.objpts -it 5.top -op 5_bld.objpts -ot 5_bld.top
//-il 9.laser -ip 9.objpts -it 9.top -op 9_bld.objpts -ot 9_bld.top
//-il 15.laser -ip 15.objpts -it 15.top -op 15_bld.objpts -ot 15_bld.top
//-il 2_7.laser -ip 2_7.objpts -it 2_7.top -op 2_7_bld.objpts -ot 2_7_bld.top
int main(int argc, char *argv[])
{
	LineTopologies lineTops;
	ObjectPoints objPnts;
	lineTops.Read("D:/eos_mapping/Tools/bin/cuboid.top");
	objPnts.Read("D:/eos_mapping/Tools/bin/cuboid.objpts");

	//search all primitives
	std::vector<int> vecFaceNums;
	SearchSegNumsInBld(lineTops, vecFaceNums);

	AdjacentGraph adjGraph = AdjacentGraph(vecFaceNums, &lineTops);
	adjGraph.DoSearchCliques();
	vector<vector<int> > vecCliques = adjGraph.GetLeastCliques();
	for(int i=0; i<vecCliques.size(); i++) {
		for(int j=0; j<vecCliques[i].size(); j++) {
			printf("%d ", vecCliques[i][j]);
		}
		printf("\n");
	}


	InlineArguments args = InlineArguments(argc, argv);

	// Check on required input files
	if (args.Contains("-usage") ||
		!args.Contains("-il") ||
		!args.Contains("-ip") ||
		!args.Contains("-it") ||
		!args.Contains("-op") ||
		!args.Contains("-ot")) 
	{
		if (!args.Contains("-usage")) printf("Error: missing programme option.\n");
		PrintUsage();
		return EXIT_SUCCESS;
	}



	return EXIT_SUCCESS;
}

void PrintUsage()
{
	printf("Usage: 3dhydro2tin -il <laser points>\n");
	printf("                 -ip <input map points (3D object points)>\n");
	printf("                 -it <input map topology>\n");
	printf("                 -o <output points (3D object points)>\n");
}
