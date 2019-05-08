/*-----------------------------------------------------------
Initial Creation:Biao Xiong

Recontruct building by minimum cycles
Useage: 3DBuildingMC -i gable.laser -op gable.objpts -ot gable.top
------------------------------------------------------------*/
#include <stdio.h>
#include "InlineArguments.h"
#include "LaserPoints.h"
#include "Buildings.h"
#include "LineTopology.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"
#include "InferenceEngine.h"
#include "AdjacentGraph.h"

using namespace std;

void PrintUsage();
void RecontructBuilding(const std::string& strLasPath, const std::string& strObjPntPath, const std::string& strLineTopPath);

int main(int argc, char *argv[])
{
	InlineArguments args = InlineArguments(argc, argv);

	// Check on required input files
	if (args.Contains("-usage") ||
		!args.Contains("-i") ||
		!args.Contains("-op") ||
		!args.Contains("-ot")) 
	{
		if (!args.Contains("-usage")) printf("Error: missing programme option.\n");
		PrintUsage();
		return EXIT_SUCCESS;
	}
	
	RecontructBuilding(args.String("-i"), args.String("-op"), args.String("-ot"));

	return EXIT_SUCCESS;
}

void PrintUsage()
{
	printf("Usage: 3dhydro2tin -i <laser points>\n");
	printf("                 -op <input map points (3D object points)>\n");
	printf("                 -ot <input map topology>\n");
}

void RecontructBuilding(const std::string& strLasPath, const std::string& strObjPntPath, const std::string& strLineTopPath)
{
	LaserPoints lasPnts;
	std::vector<Plane> vecPlanes;
	BldRecPar bldRecPar = GetIEGlobelVari();
	Buildings buildings;
	ObjectPoints map_points;
	
	
	
	lasPnts.Read(strLasPath.c_str());
	ReconstructBuilding(lasPnts, vecPlanes, buildings, map_points);
	ConstructPCMModel(buildings, map_points);
	buildings.WriteModelData(strLineTopPath.c_str());
	map_points.Write(strObjPntPath.c_str());
}














