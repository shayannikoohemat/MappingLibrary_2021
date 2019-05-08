/*--------------------------------------------------
Initial creation:
Author : Biao Xiong
Date   : 26-03-2011
Description: 
Revise:
----------------------------------------------------------*/

/*-----------------------------------------------------------
| Algorithm Description:
| classify laser points of urban scene into different types
| including ground, trees, buildings now
| this program could be expanded to detect vehicles, street lamp and so on
| This program based on segments analyzing of point clouds.
|------------------------------------------------------------*/

#include <cstdlib>
#include <iostream>
#include <vector>
#include "InlineArguments.h"
#include "LaserPoints.h"
#include "TINEdges.h"
#include "TINEdgeSet.h"
#include "LineTopology.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"
#include "ObjectPoint.h"
#include "PointNumber.h"
#include "PointNumberList.h"
#include "Plane.h"
#include "Vector3D.h"
#include "Position3D.h"
#include "TINEdges.h"
#include "Vector3D.h"
#include "HoughSpace.h"
#include "LaserOctree.h"
#include <math.h>

using namespace std;

void PrintUsage()
{
  printf("Usage: Make3DBuildings -i <input laser points>\n"); 
  printf("                       -oa <output all points (classified laser points)>\n");
  printf("                       -ob <output building points (classified laser points)>\n");
 }

bool UrbanSceneClassify(const char* strInLaserPntPath, const char* strOutLaserPntsPath, const char* strOutBuildLPntsPath);
bool IsGround(const LaserPoints& lps, const PointNumberList& pnlCurSeg);
bool IsBuilding(const LaserPoints& lps, const PointNumberList& pnlCurSeg);
bool IsTree(const LaserPoints& lps, const PointNumberList& pnlCurSeg);
double LastPulsePercentInSeg(const LaserPoints& lps, const PointNumberList& pnlCurSeg);
bool ComputerAreaVolum(const LaserPoints& lps, const PointNumberList& pnlCurSeg, 
	double& dArea, double& dVolum, float fTileWidth=0.1);
void MyAttributeValues(const LaserPoints& lps, const LaserPointTag& tag, vector<int> & vecAttrValues);
void MyGetAllSegments(const LaserPoints& lps, vector<PointNumberList>& vecSegPnl, 
	const LaserPointTag& tag=SegmentNumberTag);
float VehicalProb(float altitude, float area, float groundRatio);
bool UrbanSceneSegment(const char* strInLaserPntPath, const char* strOutLaserPntsPath);

int main(int argc, char *argv[])
{
    InlineArguments *args = new InlineArguments(argc, argv);
    
    // Check on required input files
    if (args->Contains("-usage") || !args->Contains("-i")
       || !args->Contains("-oa") || !args->Contains("-ob")) 
    {
       if (!args->Contains("-usage")) printf("Error: missing programme option.\n");
       PrintUsage();
       return EXIT_SUCCESS;
    }
       
    //bool ReconstructBuilding(char*, char*, char*);
    // Call the main function
    
   // UrbanSceneClassify(args->String("-i"), args->String("-oa"), args->String("-ob") );
    UrbanSceneSegment(args->String("-i"), args->String("-oa"));    
    delete args;
    
  //  getchar();
    return EXIT_SUCCESS;
}

//1 green; 4 yellow; 0 brown;2 blue
enum ObjectClass {
	OC_Unknown = INT_MAX, 
	OC_Tree = 1, //green
	OC_Building = 4,
	OC_Vehicle = 2,
	OC_Ground = 3 //yellow
};

bool UrbanSceneSegment(const char* strInLaserPntPath, const char* strOutLaserPntsPath)
{
    LaserPoints lps;
	if (!lps.Read(strInLaserPntPath)) {
		printf("Error reading laser points from file: %s\n", strInLaserPntPath);
		return false;	
	}
	
	SegmentationParameters	segParames;
	segParames.MinDistanceRecompute() = 0.25;
	segParames.MaxDistanceInComponent() = 2.0;
	segParames.MaxDistanceSurface() = 0.35;
	lps.SurfaceGrowing(segParames);
	lps.Write(strOutLaserPntsPath, 0, 1);
}

bool UrbanSceneClassify(const char* strInLaserPntPath, const char* strOutLaserPntsPath, const char* strOutBuildLPntsPath)
{
	LaserPoints lps;
	if (!lps.Read(strInLaserPntPath)) {
		printf("Error reading laser points from file: %s\n", strInLaserPntPath);
		return false;	
	}
	
	SegmentationParameters	segParames;
	segParames.MinDistanceRecompute() = 0.25;
	segParames.MaxDistanceInComponent() = 2.0;
	segParames.MaxDistanceSurface() = 0.35;

	printf("Start Classification...\n");
	PointNumberList pnlCurSegPnts, pnlBufferPnts;
	vector<PointNumberList> vecAllSegPnl;
	MyGetAllSegments(lps, vecAllSegPnl);
	//if in-segmented, do it
	if(vecAllSegPnl.empty()) {
       lps.SurfaceGrowing(segParames);
	   MyGetAllSegments(lps, vecAllSegPnl);
    }
	
	//construct octree
 	DataBoundsLaser bounds = lps.DeriveDataBounds(0);
	LaserOctree lOctree;
	lOctree.Initialise(bounds.Bounds3D(), 0.1, 100);
	lOctree.Insert(lps);

	int pntCount;
	double lastPulsePercent;
	double area;
	double volum;
	int adjLabel;
	double pntSpace = lps.MedianInterPointDistance((int)min(5000.0, (double)lps.size()-2));
	lps.Label(OC_Unknown);

	//////////////////////////////////////////////////////////////////////////
	//detect ground and vehicles
	//fist loop, search for very large segments, which is definitely ground
	for (int i=0; i<vecAllSegPnl.size(); ++i) {
		//pnlCurSegPnts = vecAllSegPnl[i];
		area = vecAllSegPnl[i].size()*pntSpace*pntSpace;
		if (area>2000.0) 
			lps.Label(vecAllSegPnl[i], OC_Ground);
	}
	printf("Classified large ground surface\n");

	//second loop, search for smaller segments, which has enough nearby ground points
	//and has small height difference with them
	//now can only flat ground. Warning!
	double minX, minY, minZ, maxX, maxY, maxZ;
	double range, groundPntRate, meanGrdHei, meanSegHei;
	LaserPoint* pTemPnt;
	Position3D ceter;
	int iterateCount;
	Plane curSegPlane;
	for (int i=0; i<vecAllSegPnl.size(); ++i) {
		//pnlCurSegPnts = vecAllSegPnl[i];
		if (vecAllSegPnl[i].size() < 10 || !lps[vecAllSegPnl[i][0].Number()].HasAttribute(LabelTag)
			|| lps[vecAllSegPnl[i][0].Number()].Attribute(LabelTag)!=OC_Unknown)
			continue;

		//search for boundary box of this segment
		area = vecAllSegPnl[i].size()*pntSpace*pntSpace;
		minX = minY = minZ = 9999999;
		maxX = maxY = maxZ = -9999999;
		meanSegHei = 0;
		for (int j=0; j<vecAllSegPnl[i].size(); ++j) {
			pTemPnt = &lps[vecAllSegPnl[i][j].Number()];
			if (minX > pTemPnt->X()) minX = pTemPnt->X();
			if (minY > pTemPnt->Y()) minY = pTemPnt->Y();
			if (minZ > pTemPnt->Z()) minZ = pTemPnt->Z();
			if (maxX < pTemPnt->X()) maxX = pTemPnt->X();
			if (maxY < pTemPnt->Y()) maxY = pTemPnt->Y();
			if (maxZ < pTemPnt->Z()) maxZ = pTemPnt->Z();
			meanSegHei += pTemPnt->Z();
		}
		meanSegHei /= vecAllSegPnl[i].size();

		//search for ground points in the buffer area
		//buffer area includes a box which positions at the center of segment and it's with is rang
		//iteratively enlarge this buffer area if there is not enough ground points in it
		range = (maxX-minX)>(maxY-minY)?(maxX-minX):(maxY-minY);
		range = range>(maxZ-minZ)?range:(maxZ-minZ);
		ceter = Position3D((minX+maxX)/2, (minY+maxY)/2, (minZ+maxZ)/2);
		iterateCount = 0;
		float probability;
		do {
			lOctree.Neighbourhood(pnlBufferPnts, ceter, lps, range);
			groundPntRate = 0.0;
			meanGrdHei = 0.0;
			int bufferpntcounds = pnlBufferPnts.size();
			for (int j=0; j<pnlBufferPnts.size(); ++j)	{
				pTemPnt = &lps[pnlBufferPnts[j].Number()];
				if (pTemPnt->Label() != OC_Ground) continue;
				groundPntRate += 1.0;
				meanGrdHei += pTemPnt->Z();
			}
			meanGrdHei /= groundPntRate+0.0001;
			groundPntRate /= pnlBufferPnts.size()-vecAllSegPnl[i].size()+0.0001;
			range *= 1.5;
			++iterateCount;
		} while( (pnlBufferPnts.size()<20 || groundPntRate<0.4)
          && iterateCount < 5 );
          
		probability = VehicalProb(meanSegHei-meanGrdHei, area, groundPntRate);
		if (fabs(meanGrdHei-meanSegHei) < 0.6 && area > 20)
			lps.Label(vecAllSegPnl[i], OC_Ground);
		if (meanSegHei-meanGrdHei>0.2 && meanSegHei-meanGrdHei<3.0
			&& groundPntRate > 0.6 && area<20 && area > 2) {
			curSegPlane = lps.FitPlane(vecAllSegPnl[i], 0);
			if (!curSegPlane.IsVertical(3.14*20/180))
				lps.Label(vecAllSegPnl[i], OC_Vehicle);}
	}
	printf("Classified small ground surface and Vehicles\n");

	//////////////////////////////////////////////////////////////////////////
	//detect trees and buildings
	segParames.MinDistanceRecompute() = 0.15;
	segParames.MaxDistanceInComponent() = 1.0;
	segParames.MaxDistanceSurface() = 0.20;
	PointNumberList temLpsPnl = lps.TaggedPointNumberList(LabelTag, OC_Unknown);
	LaserPoints temLps;
	temLps.reserve(temLpsPnl.size());
	for (int i=0; i<temLpsPnl.size(); i++) 
		temLps.push_back(lps[temLpsPnl[i].Number()]);
	//re-segment the left laser points with finer parameters
	temLps.SurfaceGrowing(segParames);
	MyGetAllSegments(temLps, vecAllSegPnl);
	printf("re-segment the left laser points\n");
	double lastPulseRatio;

	for (int i=0; i<vecAllSegPnl.size(); i++) {
		lastPulseRatio = LastPulsePercentInSeg(temLps, vecAllSegPnl[i]);
		area = vecAllSegPnl[i].size()*pntSpace*pntSpace;
		if (lastPulseRatio>0.88 && area>1.0)
			temLps.Label(vecAllSegPnl[i], OC_Building);
		else{
			temLps.Label(vecAllSegPnl[i], OC_Tree);
			if (lastPulseRatio>0.8)	{
				curSegPlane = temLps.FitPlane(vecAllSegPnl[i], 0);
				if (curSegPlane.IsVertical(3.14*10/180) 
					|| curSegPlane.IsHorizontal(3.14*10/180) )
					temLps.Label(vecAllSegPnl[i], OC_Building);
			}
		
		}
	}
	//relabel original laser points
	for (int i=0; i<temLps.size(); i++) {
		if (temLps[i].Label() == OC_Building) 
			lps[temLpsPnl[i].Number()].Label(OC_Building);
		else if (temLps[i].Label() == OC_Tree) 
			lps[temLpsPnl[i].Number()].Label(OC_Tree);
	}
	printf("classified buildings and trees\n");


/*	for (int iSeg=0; iSeg<vecAllSegNumber.size(); iSeg++) {
		//reconstruct the building which contain current laser point segment
		pnlCurSegPnts = lps.TaggedPointNumberList(SegmentNumberTag, vecAllSegNumber[iSeg]);
		
		pntCount = pnlCurSegPnts.size();
		lastPulsePercent = 0.0;
		for (int i=0; i<pntCount; i++) {
			if (lps[pnlCurSegPnts[i].Number()].IsPulseType(LastPulse))
				lastPulsePercent++;
		}
		lastPulsePercent /= pntCount;

		if (pntCount>40000) 
			lps.Label(pnlCurSegPnts, OC_Ground);
			//lps.Label(OC_Ground);//this segment has no laser data or is processed
		else if (lastPulsePercent<0.8)
			lps.Label(pnlCurSegPnts, OC_Tree);
		else if (pntCount>100)
			lps.Label(pnlCurSegPnts, OC_Building);
//		else 
//			lps.Label(OC_Unknown);
	}

	//search for ground points
	for (int iSeg=0; iSeg<vecAllSegNumber.size(); iSeg++) {
		//reconstruct the building which contain current laser point segment
		pnlCurSegPnts = lps.TaggedPointNumberList(SegmentNumberTag, vecAllSegNumber[iSeg]);
		area = pnlCurSegPnts.size()*pntSpace*pntSpace;
		if (area>500.0) 
			lps.Label(pnlCurSegPnts, OC_Ground);
	}*/

	// Remove class numbers "OC_Unknown"
	for (int i=0; i<lps.size(); ++i) {
		if (lps[i].Attribute(LabelTag) == OC_Unknown)
			lps[i].RemoveAttribute(LabelTag);
	}
	lps.Write(strOutLaserPntsPath, 0, 1);

	//write out building points
	lps.clear();
	lps.AddLabeledPoints(temLps, OC_Building);
	lps.Write(strOutBuildLPntsPath, 0, 1);
}

double LastPulsePercentInSeg(const LaserPoints& lps, const PointNumberList& pnlCurSeg)
{
	int pntCount = pnlCurSeg.size();
	double lastPulsePercent = 0.0;
	for (int i=0; i<pntCount; ++i) {
		if (lps[pnlCurSeg[i].Number()].IsPulseType(LastPulse))
			lastPulsePercent += 1;
	}
	lastPulsePercent /= pntCount+0.01;

	return lastPulsePercent;
}

bool ComputerAreaVolum(const LaserPoints& lps, const PointNumberList& pnlCurSeg, 
	double& dArea, double& dVolum, float fTileWidth)
{
	//calculate boundary box of this segment
	double maxX = -99999999; 
	double maxY = -99999999;  
	double minX = 99999999;
	double minY = 99999999;
	LaserPoint lpTem;
	for (int i=0; i<pnlCurSeg.size(); ++i)	{
		lpTem = lps[pnlCurSeg[i].Number()];
		maxX = maxX>lpTem.X()?maxX:lpTem.X();
		maxY = maxY>lpTem.Y()?maxY:lpTem.Y();
		minX = minX<lpTem.X()?minX:lpTem.X();
		minY = minY>lpTem.Y()?minY:lpTem.Y();
	}

	//computer max and min height for each tile
	int xTileCounts = int((maxX-minX)/fTileWidth);
	int yTileCounts = int((maxY-minY)/fTileWidth);
	vector<float> vecMinHeight(xTileCounts*yTileCounts, 99999999.0);
	vector<float> vecMaxHeight(xTileCounts*yTileCounts, -99999999.0);
	int xGrid, yGrid, nGrid;
	for (int i=0; i<pnlCurSeg.size(); ++i)	{
		lpTem = lps[pnlCurSeg[i].Number()];
		xGrid = int((lpTem.X()-minX)/fTileWidth);
		yGrid = int((lpTem.Y()-minY)/fTileWidth);
		nGrid = yGrid*xTileCounts+xGrid;
		if (vecMaxHeight[nGrid]<lpTem.Z())  
			vecMaxHeight[nGrid] = lpTem.Z();
		if (vecMinHeight[nGrid]>lpTem.Z())  
			vecMinHeight[nGrid] = lpTem.Z();
	}

	dArea = 0.0;
	dVolum = 0.0;
	for (int i=0; i<vecMinHeight.size(); ++i) {
		if (vecMinHeight[i]>-99999999.0 && vecMaxHeight[i]<99999999.0) {
			dArea += 1;
			dVolum += vecMaxHeight[i]-vecMinHeight[i];
		}
	}

	dArea *= fTileWidth*fTileWidth;
	dVolum *= fTileWidth*fTileWidth;

	return true;
}

void MyAttributeValues(const LaserPoints& lps, const LaserPointTag& tag, vector<int> & vecAttrValues)
{
	vecAttrValues.clear();
	int attrValueCounts = 500;
	vector<bool> vecHasTag(attrValueCounts, false);
	int value;

	for (int i=0; i<lps.size(); ++i) {
		if (!lps[i].HasAttribute(tag)) continue;
		value = lps[i].Attribute(tag);
		if (value>attrValueCounts-1) { 
			vecHasTag.insert(vecHasTag.begin()+attrValueCounts, value, false);
			attrValueCounts += value;
		}
		if (!vecHasTag[value])
			vecHasTag[value] = true;		
	}

	vecAttrValues.reserve(attrValueCounts);
	for (int i=0; i<vecHasTag.size(); ++i) {
		if (vecHasTag[i]) 
			vecAttrValues.push_back(i);
	}
}

//use more memory to get faster
void MyGetAllSegments(const LaserPoints& lps, vector<PointNumberList>& vecSegPnl, const LaserPointTag& tag)
{
	vecSegPnl.clear();

	PointNumberList pnlTemSeg;
	int attrValueCounts = 5000;
	vector<PointNumberList> vecTemSegPnl(attrValueCounts, pnlTemSeg);
	int tagValue;

	for (int i=0; i<lps.size(); ++i) {
		if (!lps[i].HasAttribute(tag)) continue;
		tagValue = lps[i].Attribute(tag);
		if (tagValue>attrValueCounts-1) {
			vecTemSegPnl.insert(vecTemSegPnl.begin()+attrValueCounts, tagValue, pnlTemSeg);
			attrValueCounts += tagValue;
		}

		if (vecTemSegPnl[tagValue].empty())
			vecTemSegPnl.reserve(500);
		vecTemSegPnl[tagValue].push_back(PointNumber(i));
	}

	vecSegPnl.reserve(attrValueCounts/2);
	for (int i=0; i<vecTemSegPnl.size(); ++i) {
		if (!vecTemSegPnl[i].empty())
			vecSegPnl.push_back(vecTemSegPnl[i]);
	}
}

//calculate the probability of a segment be a vehicle
//altitude: height difference between vehicle and ground
//area: vehicle area
//groundRatio: ground points number / all pints number in buffer area
float VehicalProb(float altitude, float area, float groundRatio)
{
//	if (altitude < 0.3 || area<0.5 || groundRatio< 0.1)
//		return 0.0;
	static float meanAlt = log(2.0);
	static float sigmaAlt = 0.7;
	static float meanArea = 15.0;
	static float segmaArea = 4.0;
	static float meanGrdRat = 0.6;
	static float sigmaGrdRat = 0.1;
//	static float constant = 1/sqrtf(2*3.14159);

	float p1 = exp(-pow((log(altitude)-meanAlt)/sigmaAlt,2)/2);
	float p2 = exp(-pow((area-meanAlt)/meanArea,2)/2);

	return p1*p2*groundRatio;
	//return 1-(1-p1)*(1-p2)*(1-groundRatio);
}
