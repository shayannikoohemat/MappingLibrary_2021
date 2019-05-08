/*--------------------------------------------------
Initial creation:
Author : Biao Xiong
Date   : 26-01-2011
Description: 
Revise:
----------------------------------------------------------*/
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
//#include "LineTopsIterVector.h"
//#include "VectorPoint.h"
//#include "VRML_io.h"
//#include "dxf.h"
//#include "TIN.h"
//#include "Building.h"
//#include "Buildings.h"
//#include "stdmath.h"
//#include "triangle.h"
//#include "Database.h"
#include "InferenceEngine.h"
#include <vector>
#include <math.h>
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <strings.h>
//#include <Matrix3.h>
//using namespace std;


#define USING_CONSTRAINT 1

#define INTERSECT_FACE_MIN_DIST 0.5

LaserPoints g_laserPnts;//
LineTopologies g_BuildLineTops;//0 Attribute left hand segment number; 1 Attribute left hand segment number
ObjectPoints g_BuildPoints;//

//int g_nObjPntsCount = 0;
//int g_nTopLinesCount = 0;

bool ReconstructBuilding(char* strInLaserPntPath, char* strOutObjPntsPath, char* strOutTopLinesPath)
{ 
	// strInLaserPntPath = "E:\\test data\\gable.laser";
	// strOutObjPntsPath = "E:\\test data\\gable.objpts";
	// strOutTopLinesPath = "E:\\test data\\gable.top";

	//strInLaserPntPath = "E:\\test data\\U-shape.laser";
	//strOutObjPntsPath = "E:\\test data\\U-shape.objpts";
	//strOutTopLinesPath = "E:\\test data\\U-shape.top";

	if (!g_laserPnts.Read(strInLaserPntPath)) 	
	{
		printf("Error reading laser points from file: %s\n", strInLaserPntPath);
		return false;	
	}

	g_laserPnts.ReduceData(0.05);//
	g_laserPnts.DeriveTIN();
	TINEdges edges;
	edges.Derive(g_laserPnts.TINReference()); 
	g_laserPnts.RemoveLongEdges(edges, 0.75, false);
	g_laserPnts.SetAttribute(IsProcessedTag, -1); //-1 unprocessed 1 processed
	//int nSegPntsNum = 0
	//int nDorminSegNum = 0;
	vector<int> vecAllSegNumber, vecCurSegNumber;
	//vector<LaserPoints> vecSegPnts;
	PointNumberList pnlCurSegPnts;
	vector<PointNumberList> vecLocalSegPnts;
	vector<Plane> vecLocalPlanes;
	ObjectPoints localObjPnts;

	//Attribute 0: left hand face Number; Attribute 1: right hand face Number; Attribute 2: is/not refined tag
	LineTopologies localLineTops;
	//nDorminSegNum = g_laserPnts.MostFrequentAttributeValue(SegmentNumberTag, nSegPntsNum);
	vecAllSegNumber = g_laserPnts.AttributeValues(SegmentNumberTag);
	if(vecAllSegNumber.empty()) 
	{
		return false;//point cloud has not been segmented
		printf("laser point cloud has not beed segmented");
	}

	printf("\n\n|--------------------------------------------------|\n");
	printf("Start reconstruct building ...\n");
	Plane temPlane;
	temPlane.Label() = -1;//-1 unfitted 1 fisted
	//g_vecPlanes = vector<Plane>( vecAllSegNumber.size()+1, temPlane) ;
	int nBuildingCount = 0;

	/*for debug 
	//    printf("All Segment Numbers");
	//   	for (int iSeg=0; iSeg<vecAllSegNumber.size(); iSeg++)
	//        printf("%d",vecAllSegNumber[iSeg]);*/

	//main loop for reconstruct roof segment by segment
	for (int iSeg=0; iSeg<vecAllSegNumber.size(); iSeg++)
	{//reconstruct the building which contain current laser point segment
		pnlCurSegPnts = g_laserPnts.TaggedPointNumberList(SegmentNumberTag, vecAllSegNumber[iSeg]);
		if (pnlCurSegPnts.size() < 100
			|| g_laserPnts[pnlCurSegPnts[0].Number()].Attribute(IsProcessedTag) != -1)
			continue;//this segment has no laser data or is processed

		vecLocalSegPnts.clear();		
		vecLocalPlanes.clear();
		localObjPnts.clear();
		localLineTops.clear();		

		if(!ReconstructLocalSegments(g_laserPnts, vecLocalPlanes, pnlCurSegPnts, vecLocalSegPnts, localObjPnts, localLineTops))
			 continue;

		//need to be done before refine other faces
		RefineSimpleDormer(g_laserPnts, vecLocalPlanes, localObjPnts, localLineTops);
		
		////RefineOutBoundaryPlnae(g_laserPnts, vecLocalPlanes, vecLocalSegPnts, localObjPnts, localLineTops);

		RefineCornerOf3Faces(vecLocalPlanes, localObjPnts, localLineTops);
		RefineCornerOf4Faces(vecLocalPlanes, localObjPnts, localLineTops);

		RefineOutBoundaryLine4(g_laserPnts, vecLocalPlanes,vecLocalSegPnts, localObjPnts, localLineTops);
		////RefineLocalBuilding(vecLocalPlanes,localObjPnts,localLineTops);
		//GetCountour(g_laserPnts, vecLocalSegPnts, localObjPnts, localLineTops);
		/*
		vector<ObjectPoint> vecAdjNodes;
		ObjectPoint curNode;
		//set a vector to record which Object position is Searched
		int nMaxNumber = -1;
		for (int i=0; i<localObjPnts.size(); i++)
		{
		if (localObjPnts[i].Number() > nMaxNumber)
		nMaxNumber = localObjPnts[i].Number();
		}

		vector<bool> vecIsSearched(nMaxNumber+1, false);
		PointNumber numCurObjPnt;
		vector<PointNumber> vecAdjPntNum;

		//refine reconstruction result
		for (int iPnt=0; iPnt<localObjPnts.size(); iPnt++)
		{
		numCurObjPnt = localObjPnts[iPnt].Number();

		if (!SearchNeigborNodes(localObjPnts, vecIsSearched, numCurObjPnt, vecAdjPntNum, 0.5))
		continue;//has no neighbor points

		double averageX, averageY, averageZ;
		averageX = localObjPnts[iPnt].GetX();
		averageY = localObjPnts[iPnt].GetY();
		averageZ = localObjPnts[iPnt].GetZ();

		ObjectPoint temObjPnt;
		for (int iNeib=0; iNeib<vecAdjPntNum.size(); iNeib++)
		{
		temObjPnt = localObjPnts.PointByNumber(vecAdjPntNum[iNeib].Number());
		averageX += temObjPnt.GetX();
		averageY += temObjPnt.GetY();
		averageZ += temObjPnt.GetZ();
		}
		//average the points, just the simplest way to get final position now
		averageX /= 1+vecAdjPntNum.size();
		averageY /= 1+vecAdjPntNum.size();
		averageZ /= 1+vecAdjPntNum.size();
		//set new position to all outpouring points
		localObjPnts[iPnt].SetX(averageX);
		localObjPnts[iPnt].SetY(averageY);
		localObjPnts[iPnt].SetZ(averageZ);
		for (int iNeib=0; iNeib<vecAdjPntNum.size(); iNeib++)
		{
		int temObjInd = localObjPnts.PointIndexByNumber(vecAdjPntNum[iNeib].Number());
		localObjPnts[temObjInd].SetX(averageX);
		localObjPnts[temObjInd].SetY(averageY);
		localObjPnts[temObjInd].SetZ(averageZ);
		}
		}*/ 

		LineTopology temLineTop;
		ObjectPoint temObjPnt;
		int objPntNum;
		for (int i=0; i < localLineTops.size(); i++)
		{
			temLineTop = localLineTops[i];
			objPntNum = g_BuildPoints.size();

			for (int j=0; j<temLineTop.size(); j++)
			{
                double number = temLineTop[j].Number();
				temObjPnt = localObjPnts[temLineTop[j].Number()];
				temObjPnt.Number() = objPntNum+j;
				g_BuildPoints.push_back(temObjPnt);

				temLineTop[j].Number() = objPntNum+j;
			}

			temLineTop.Number() = g_BuildLineTops.size();
			g_BuildLineTops.push_back(temLineTop);
		}
		//g_BuildPoints.insert(g_BuildPoints.end(), localObjPnts.begin(), localObjPnts.end());
		//g_BuildLineTops.insert(g_BuildLineTops.end(), localLineTops.begin(), localLineTops.end());

		printf("%dst building has been reconstructed\n", ++nBuildingCount);
	}

	printf("End reconstruct building !\n");
	printf("|--------------------------------------------------|\n");


	if(!g_BuildPoints.Write(strOutObjPntsPath))
	{
		printf("Error writing building points to file: %s\n", strOutObjPntsPath);
		return false;	
	}

	if(!g_BuildLineTops.Write(strOutTopLinesPath, false))
	{
		printf("Error writing building top lines to file: %s\n", strOutTopLinesPath);
		return false;	
	}
}

bool SearchNeigborNodes(ObjectPoints& localObjPnts, vector<bool>& vecIsSearched, 
	PointNumber numCurObjPnt, vector<PointNumber>& vecAdjPntNum, double fMinDist)
{
	vecIsSearched[numCurObjPnt.Number()] = true;
	vecAdjPntNum.clear();

	ObjectPoint curObjPnt = localObjPnts.PointByNumber(numCurObjPnt.Number());	
	ObjectPoint temObjPnt;
	PointNumber numTemObjPnt;
	for (int i=0; i<localObjPnts.size(); i++)
	{
		temObjPnt = localObjPnts[i];
		numTemObjPnt = temObjPnt.Number();

		//has been searched
		if (vecIsSearched[numTemObjPnt.Number()])
			continue;
		//find suitable point
		if (temObjPnt.Distance(curObjPnt) <= fMinDist)
		{
			vecIsSearched[numTemObjPnt.Number()] = true;
			vecAdjPntNum.push_back(numTemObjPnt);
		}
	}

	if (vecAdjPntNum.empty())
		return false;
	else
		return true;	
}

bool SearchAdjacentSegments(const LaserPoints& gLaserPnts, PointNumberList pnlOrgSeg, 
	vector<PointNumberList>& vecAdjSegPnts, int nMinSegPntNum, int nMinAdjPntNum)
{
	if (pnlOrgSeg.empty())
		return false;

	TINEdges edges;
	edges.Derive(gLaserPnts.TINReference());
	TINEdgeSet tesNeibor;
	//PointNumberList pnlAdjPnts;
	vector<int> vecSegNumber;
	LaserPoint lpTemPnt;
	int curSegNumber = (gLaserPnts[pnlOrgSeg[0].Number()]).Attribute(SegmentNumberTag);
	int temSegNumber;
	bool bStored = false;

	//search for SegmentNumber of possible adjacent segments
	for (int i=0; i<pnlOrgSeg.size(); i++)
	{
		tesNeibor = edges[pnlOrgSeg[i].Number()];
		for (PointNumberList::iterator itr=tesNeibor.begin(); itr!=tesNeibor.end(); itr++)
		{
			lpTemPnt = gLaserPnts[itr->Number()];

			if(!lpTemPnt.HasAttribute(SegmentNumberTag))
				continue;//invalide segment label

			temSegNumber = lpTemPnt.Attribute(SegmentNumberTag);
			if (temSegNumber == curSegNumber || lpTemPnt.Attribute(IsProcessedTag) != -1)
				continue;//laser point in current segment or this segment is processed
			bStored = false;
			for (int j=0; j<vecSegNumber.size(); j++)
			{
				if (temSegNumber == vecSegNumber[j])
				{
					bStored = true;//this segment number has been stored
					break;
				}
			}

			if(!bStored)
				vecSegNumber.push_back(temSegNumber);
		}
	}

	//search for adjacent segments
	PointNumberList pnlTemSeg;
	LaserPoints orgLaserPnts;
	for (int j=0; j<pnlOrgSeg.size(); j++)
		orgLaserPnts.push_back(gLaserPnts[j]);

	LaserPoints temLaserPnts;
	for (int i=0; i<vecSegNumber.size(); i++)
	{
		temSegNumber = vecSegNumber[i];
		pnlTemSeg = gLaserPnts.TaggedPointNumberList(SegmentNumberTag, temSegNumber);
		if (pnlTemSeg.size()<nMinSegPntNum)
			continue;//this segment is too small

		temLaserPnts.clear();
		for (int j=0; j<pnlTemSeg.size(); j++)
			temLaserPnts.push_back(gLaserPnts[j]);

		if (IsAdjacent(&orgLaserPnts, &temLaserPnts))
			vecAdjSegPnts.push_back(pnlTemSeg);
	}

	if (vecAdjSegPnts.empty())
		return false;
	else
		return true;
}

bool SearchLineBy2Faces(const LineTopologies& localLineTops, const int faceNum1, const int faceNum2, LineTopology& lineTop)
{
	LineTopology temLineTop;
	bool bFlag = false;
	vector<int> vecTemLineTopNums;

	SearchLinesOnPlane(localLineTops, faceNum1, vecTemLineTopNums);

	for (int i=0; i<vecTemLineTopNums.size(); i++)
	{
		temLineTop = localLineTops[vecTemLineTopNums[i]];

		if (temLineTop.Attribute(0) == faceNum2 
			|| temLineTop.Attribute(1) == faceNum2)
		{
			lineTop = temLineTop;
			bFlag = true;
			break;
		}
	}

	return bFlag;
}

//search faces which are neighboring to face 1 and face 2 at the same time
bool SearchFaceBy2Faces(const LineTopologies& localLineTops, const int faceNum1, const int faceNum2, std::vector<int>& vecFaceNums)
{
	vecFaceNums.clear();

	vector<int> AdjFaces1, AdjFaces2, totAdjFace;
	SearchAdjacentFaces(localLineTops, faceNum1, AdjFaces1);
	SearchAdjacentFaces(localLineTops, faceNum2, AdjFaces2);

	totAdjFace = AdjFaces1;
	totAdjFace.insert(totAdjFace.end(), AdjFaces2.begin(), AdjFaces2.end());
	std::sort(totAdjFace.begin(), totAdjFace.end());

	for (int i=0; i<totAdjFace.size()-1; i++)
	{
		if (totAdjFace[i] == faceNum1 || totAdjFace[i] == faceNum2)
			continue;

		if (totAdjFace[i] == totAdjFace[i+1] )
			vecFaceNums.push_back(totAdjFace[i]);
	}

	if (vecFaceNums.empty())
		return false;
	else 
		return true;
}

//search lines whose node is near to current node of current node,
//if one of these lines has been refined, stop search
bool SearchNodesOnNeigborLine(const ObjectPoints& localObjPnts, const LineTopologies& localLineTops, 
	const int numCurLine, const int curNode, vector<int>& vecNeibNodes, double minDist)
{
	Position3D starPnt = (Position3D)localObjPnts[localLineTops[numCurLine][0].Number()];
	Position3D endPnt = (Position3D)localObjPnts[localLineTops[numCurLine][1].Number()];
	Position3D curPnt = (Position3D)localObjPnts[localLineTops[numCurLine][curNode].Number()];
	Line3D curLine(starPnt, endPnt);

	LineTopology temLineTop;
	Position3D temStartPnt, temEndPnt;
	Vector3D intersectPnt;
	Line3D temLine;
	int temNodeNum;

	vecNeibNodes.clear();
	for (int i=0; i<localLineTops.size(); i++)
	{
		if (i == numCurLine )
			continue;

		temLineTop = localLineTops[i];
		temStartPnt = (Position3D)localObjPnts[temLineTop[0].Number()];
		temEndPnt = (Position3D)localObjPnts[temLineTop[1].Number()];
		temLine = Line3D(temStartPnt, temEndPnt);

		if (!MyIntersect2Lines(curLine, temLine, intersectPnt)
			|| (intersectPnt-(Vector3D)curPnt).Length() > minDist)
			continue;

		if((intersectPnt-temStartPnt).Length() < minDist)
		{
			temNodeNum = temLineTop[0].Number();
			if(temLineTop.Attribute(2) != -1)
				return false;//stop search
		}
		else if((intersectPnt-temEndPnt).Length() < minDist)
		{
			temNodeNum = temLineTop[1].Number();
			if(temLineTop.Attribute(2) != -1)
				return false;//stop search
		}	
		else
			continue;

		vecNeibNodes.push_back(temNodeNum);
	}

	if (vecNeibNodes.empty())
		return false;
	else
		return true;
}

/*
bool Detect3DLineByHoughTrans(const LaserPoints& lLaserPnts, const Plane& refPlane, std::vector<Position3D>& vecLineNodes)
{
	HoughSpace houghspace;
	LaserPoints temLaserPnts = lLaserPnts;
	Position3D temPos;
	Vector3D dir = refPlane.Normal();
	Line2D  line;
	double minPntNums = 5;
	double minDist = 0.2;
	double dist;
	vector<int> vecLineSegPnts;
	Position2D p1, p2;
	Position3D pos3d1, pos3d2;
	int pntNums;
	LaserPoint temLaserPnt;
	
	//transform points to x-y plane according to their original plane
	for (int i=0; i<lLaserPnts.size(); i++)
	{
		if (fabs(dir.Z())>fabs(dir.X()) && fabs(dir.Z())>fabs(dir.Y()))//horizontal, on x-y plane
		{
			temPos.X() = lLaserPnts[i].X();
			temPos.Y() = lLaserPnts[i].Y();
			temPos.Z() = 0;
		}
		else if (fabs(dir.X())>fabs(dir.Y()) && fabs(dir.X())>fabs(dir.Z()))//on y-z plane
		{
			temPos.X() = lLaserPnts[i].Z();
			temPos.Y() = lLaserPnts[i].Y();
			temPos.Z() = 0;
		}
		else//on x-z plane
		{
			temPos.X() = lLaserPnts[i].X();
			temPos.Y() = lLaserPnts[i].Z();
			temPos.Z() = 0;
		}

		temLaserPnts[i].Position3DRef() = temPos;
	}

	//construct Hough space
	temLaserPnts.Label(-1);
	temLaserPnts.InitialiseHoughSpace(houghspace, -1, 85*PI/180, 3*PI/180, 0.5, 1, LabelTag);
	temLaserPnts.IncrementHoughSpace(houghspace);

	do 
	{
		pntNums = 0;
		line = houghspace.BestLine(&pntNums, 0, 3); 

		if (pntNums < minPntNums) continue;
		//pntNums = selLaserPnts.Label(plane, minDist, -1, -1,  1, 0);
		
		//remove detected points from hough space
		//and get planes of corresponding segments
		pntNums = 0;
		vecLineSegPnts.clear();
		for (int i=0; i<temLaserPnts.size(); i++)
		{	
			temLaserPnt = temLaserPnts[i];
			dist = line.DistanceToPoint(temLaserPnt.Position2DOnly());
			if (dist > minDist) continue;

			pntNums++;
			houghspace.RemovePoint(temLaserPnt.X(), temLaserPnt.Y(), temLaserPnt.Z());

			vecLineSegPnts.push_back(i);
		}

		//to do, fit the line again

		if (pntNums < minPntNums) continue;
		
		double min = 9999999.0;
		double max = -99999999.0;
		double scalar;
		//search for start and end node
		for (int i=0; i<vecLineSegPnts.size(); i++)
		{
			temLaserPnt = temLaserPnts[i];
			scalar = line.Scalar(temLaserPnt.Position2DOnly());

			if (scalar < min) min = scalar;
			if (scalar > max) max = scalar;
		}

		p1 = line.Position(min);
		p2 = line.Position(max);

		//re-transform to original plane
		if (fabs(dir.Z())>fabs(dir.X()) && fabs(dir.Z())>fabs(dir.Y()))//horizontal, on x-y plane
		{
			pos3d1.X() = p1.X(); pos3d1.Y() = p1.Y(); pos3d1.Z() = 0.0;
			pos3d2.X() = p2.X(); pos3d2.Y() = p2.Y(); pos3d2.Z() = 0.0;
		}
		else if (fabs(dir.X())>fabs(dir.Y()) && fabs(dir.X())>fabs(dir.Z()))//on y-z plane
		{
			pos3d1.X() = 0.0; pos3d1.Y() = p1.Y(); pos3d1.Z() = p1.X();
			pos3d2.X() = 0.0; pos3d2.Y() = p2.Y(); pos3d2.Z() = p2.X();
		}
		else//on x-z plane
		{
			pos3d1.X() = p1.X(); pos3d1.Y() = 0.0; pos3d1.Z() = p1.Y();
			pos3d2.X() = p2.X(); pos3d2.Y() = 0.0; pos3d2.Z() = p2.Y();
		}

		pos3d1 = refPlane.Project(pos3d1);
		pos3d2 = refPlane.Project(pos3d2);

		//out put
		ObjectPoint objPnt;
		LineTopology lineTop;

		objPnt.X() = pos3d1.X(); objPnt.Y() = pos3d1.Y(); objPnt.Z() = pos3d1.Z(); 
		objPnt.Number() = localObjPnts.size();
		localObjPnts.push_back(objPnt);
		lineTop.push_back(objPnt.Number());

		objPnt.X() = pos3d2.X(); objPnt.Y() = pos3d2.Y(); objPnt.Z() = pos3d2.Z(); 
		objPnt.Number() = localObjPnts.size();
		localObjPnts.push_back(objPnt);
		lineTop.push_back(objPnt.Number());

		lineTop.Number() = localLineTops.size();
		lineTop.SetAttribute(0, plane.Number());
		lineTop.SetAttribute(1, temPlane.Number());
		localLineTops.push_back(lineTop);
		
	} while (pntNums > minPntNums);
}*/

bool IsAdjacent(LaserPoints* pLLaserPnts, LaserPoints* pRLaserPnts, double nMinDistance, int nMinAdjPnts)
{
	if (!pLLaserPnts || !pRLaserPnts)
		return false;

	LaserPoints totPnts;
	pLLaserPnts->Label( 0);
	pRLaserPnts->Label( 1);
	int lPntsCount = totPnts.AddPoints(*pLLaserPnts);
	int rPntsCount = totPnts.AddPoints(*pRLaserPnts);

	totPnts.DeriveTIN();
	TINEdges edges;
	edges.Derive(totPnts.TINReference());
	LaserPoints::const_iterator itrCurPnt, itrTemPnt;
	TINEdgeSet neibTinEdgeSet;
	PointNumberList pnlAdjacent;
	PointNumberList::iterator itrPnlCurPnt;
	//Search for neighbor laser point between 2 segments
	for (itrCurPnt=totPnts.begin(); itrCurPnt!=totPnts.end(); itrCurPnt++) 
	{
		if (!itrCurPnt->HasAttributeValue(LabelTag, 0)) 
			continue;//not left segment laser point

		neibTinEdgeSet = edges[itrCurPnt-totPnts.begin()];
		for (itrPnlCurPnt=neibTinEdgeSet.begin(); itrPnlCurPnt<neibTinEdgeSet.end(); itrPnlCurPnt++)
		{
			itrTemPnt = totPnts.begin() + itrPnlCurPnt->Number();
			if (!itrTemPnt->HasAttributeValue(LabelTag, 1))
				continue;//not right laser laser point

			if (nMinDistance > (*itrTemPnt - *itrCurPnt).Length())
				pnlAdjacent.push_back(itrPnlCurPnt->Number());
		}
	}

	//erase repeat points
	pnlAdjacent.Sort();
	for (PointNumberList::iterator itr=pnlAdjacent.begin(); itr!=pnlAdjacent.end(); itr++)
	{
		PointNumberList::iterator rightItr = itr+1;
		if (rightItr == pnlAdjacent.end())
			break;
		if(rightItr->Number() == itr->Number())
			pnlAdjacent.erase(rightItr);
	}

	if (pnlAdjacent.size() >= nMinAdjPnts)
		return true;
	else
		return false;
}

//iterative search neighbor segments and detect intersection lines between them
bool ReconstructLocalSegments(LaserPoints& gLaserPnts, vector<Plane>& vecLocalPlanes, const PointNumberList& pnlCurSegPnts,
	vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	gLaserPnts.Label(pnlCurSegPnts, 1, IsProcessedTag);
	vecLocalSegPnts.push_back(pnlCurSegPnts);
	vector<PointNumberList> vecAdjSegments;

	SearchAdjacentSegments(g_laserPnts, pnlCurSegPnts, vecAdjSegments);

	PointNumberList pnlTemSegPnts;
	Plane temPlane, curPlane;
	int curPlaneInd, temPlaneInd;

	int curSegNumber = gLaserPnts[pnlCurSegPnts[0].Number()].Attribute(SegmentNumberTag);
	curPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, curSegNumber);
	if (curPlaneInd == -1)//unfitted
	{
		curPlane = g_laserPnts.FitPlane(pnlCurSegPnts);

		//plane constraint
		if (USING_CONSTRAINT == 1
			&& ConstraintPlane(g_laserPnts, pnlCurSegPnts, curPlane))
			curPlane.Label() = 1;
		else
			curPlane.Label() = 0;

		curPlane.Number() = curSegNumber;
		vecLocalPlanes.push_back(curPlane);
		curPlaneInd = vecLocalPlanes.size()-1;
	}
	else 
		curPlane = vecLocalPlanes[curPlaneInd];

	for (int i=0; i<vecAdjSegments.size(); i++)
	{
		pnlTemSegPnts = vecAdjSegments[i];
		int temSegNumber = gLaserPnts[pnlTemSegPnts[0].Number()].Attribute(SegmentNumberTag);
		temPlaneInd = IndexPlaneBySegNumber(vecLocalPlanes, temSegNumber);

		if (temPlaneInd == -1)//unfitted
		{
			temPlane = g_laserPnts.FitPlane(pnlTemSegPnts);

			//plane constraint
			if (USING_CONSTRAINT == 1
				&& ConstraintPlane(g_laserPnts, pnlTemSegPnts, temPlane))
				temPlane.Label() = 1;
			else
				temPlane.Label() = 0;

			temPlane.Number() = temSegNumber;
			vecLocalPlanes.push_back(temPlane);
			temPlaneInd = vecLocalPlanes.size()-1;
		}
		else 
			temPlane = vecLocalPlanes[temPlaneInd];

		Position3D startPos, endPos;
		g_laserPnts.IntersectFaces(pnlCurSegPnts, pnlTemSegPnts, curPlane, temPlane, INTERSECT_FACE_MIN_DIST, startPos, endPos);
		if (startPos.Distance(endPos) < 0.5 )
			continue;//intersection line is too short

		if (USING_CONSTRAINT == 1)
		{//inter sect line constraint

			Plane plan1 = curPlane;
			Plane plan2 = temPlane;

			/*if (ConstraintIntersecLine(g_laserPnts, pnlCurSegPnts, pnlTemSegPnts, curPlane, temPlane, startPos, endPos))
			{//plane constraint and intersect constraint will never happen at the same time
			//so fitted plane can only changed once

			curPlane.Label() = 1;
			curPlane.Number() = curSegNumber;
			vecLocalPlanes[curPlaneInd] = curPlane;

			temPlane.Label() = 1;
			temPlane.Number() = temSegNumber;
			vecLocalPlanes[temPlaneInd] = temPlane;
			}*/

			plan1.Label() = 1;
			plan1.Number() = curSegNumber;
			vecLocalPlanes[curPlaneInd] = plan1;

			plan2.Label() = 1;
			plan2.Number() = temSegNumber;
			vecLocalPlanes[temPlaneInd] = plan2;

		}

		int objPntNum = localObjPnts.size();
		ObjectPoint startObjPnt = ObjectPoint(startPos.GetX(),startPos.GetY(),startPos.GetZ(), objPntNum,0,0,0,0,0,0);
		ObjectPoint endObjPnt = ObjectPoint(endPos.GetX(),endPos.GetY(),endPos.GetZ(),objPntNum+1,0,0,0,0,0,0);
		LineTopology intsectLine = LineTopology(localLineTops.size(), -1, objPntNum, objPntNum+1);

		//set segmentnumber to highest. second segment label to lowest segment
		//int leftSegNumber = g_laserPnts[pnlCurSegPnts[0].Number()].Attribute(SegmentNumberTag);
		//int rightSegNumber = g_laserPnts[vecAdjSegments[i][0].Number()].Attribute(SegmentNumberTag);
		intsectLine.SetAttribute(0, curSegNumber);//left hand segment number
		intsectLine.SetAttribute(1, temSegNumber); //right hand segment number
		intsectLine.SetAttribute(2, -1); //unrefined
		localObjPnts.push_back(startObjPnt);
		localObjPnts.push_back(endObjPnt);
		localLineTops.push_back(intsectLine);
		//Iterative search and reconstruct all segments neighbor with this segments 
		//And their neighbors
		ReconstructLocalSegments(gLaserPnts, vecLocalPlanes, pnlTemSegPnts, vecLocalSegPnts, localObjPnts, localLineTops);
	}

	return true;
}

bool ConstraintPlane(const LaserPoints& gLaserPnts, const PointNumberList& pnlSegPnts, Plane& plane, double minAngle)
{
	Vector3D normal;
	bool bFlag = false;

	if (plane.IsHorizontal(minAngle))
	{
		normal = Vector3D(0,0,1);
		plane= gLaserPnts.FitPlane (pnlSegPnts, normal, plane.Number());
		bFlag = true;
	}
	else if(plane.IsVertical(minAngle))
	{
		normal = plane.Normal();
		normal = Vector3D(normal.X(), normal.Y(), 0);
		normal = normal.Normalize();
		plane= gLaserPnts.FitPlane(pnlSegPnts, normal, plane.Number());
		bFlag = true;
	}

	return bFlag;
}

bool ConstraintIntersecLine(const LaserPoints& gLaserPnts, const PointNumberList& pnlLeftSeg, const PointNumberList& pnlRightSeg,
	Plane& leftPlane, Plane& rightPlane, Position3D& startPos, Position3D& endPos,	double minAngle)
{
	//one plane is already horizontal/vertical
	//the intersect line is sure to be horizontal/vertical
	//it is not needed to constraint the intersect line
	if (leftPlane.IsHorizontal(minAngle) || rightPlane.IsHorizontal(minAngle)
		|| leftPlane.IsVertical(minAngle) || rightPlane.IsVertical(minAngle) )
		return false;

	Vector3D direction = (Vector3D)startPos - (Vector3D)endPos;
	direction.Normalize();

	if (direction.Z() < 0)
		direction *= -1;

	bool bFlag = false;

	if ( direction.DotProduct(Vector3D(0,0,1)) < sin(minAngle))
	{//horizontal intersect line
		Vector3D leftNormal = leftPlane.Normal();
		Vector3D rightNormal = rightPlane.Normal();
		double sigmal;

		if (leftNormal.Z() < 0)
			leftNormal *= -1;

		if (rightNormal.Z() < 0)
			rightNormal *= -1;

		//constraint the two planes intersect vertical line with same angle
		if (fabs(leftNormal.Z()-rightNormal.Z()) < sin(minAngle))
		{
			leftNormal.Z() = (leftNormal.Z()+rightNormal.Z())/2;
			leftNormal.Z() = rightNormal.Z();
		}

		//constraint the intersect line to be horizontal
		if ( fabs(leftNormal.X()) > fabs(leftNormal.Y()) )
		{
			sigmal = (leftNormal.Y()/leftNormal.X() + rightNormal.Y()/rightNormal.X())/2;
			leftNormal.Y() = sigmal*leftNormal.X();
			rightNormal.Y() = sigmal*rightNormal.X();
		}
		else
		{
			sigmal = (leftNormal.X()/leftNormal.Y() + rightNormal.X()/rightNormal.Y())/2;
			leftNormal.X() = sigmal*leftNormal.Y();
			rightNormal.X() = sigmal*rightNormal.Y();
		}

		leftNormal.Normalize();
		rightNormal.Normalize();
		leftPlane = gLaserPnts.FitPlane(pnlLeftSeg, leftNormal, leftPlane.Number());
		rightPlane = gLaserPnts.FitPlane(pnlRightSeg, rightNormal, rightPlane.Number());

		gLaserPnts.IntersectFaces(pnlLeftSeg, pnlRightSeg, leftPlane, rightPlane, INTERSECT_FACE_MIN_DIST, startPos, endPos);
		bFlag = true;
	}
	else if (direction.DotProduct(Vector3D(0,0,1)) < cos(minAngle))
	{//vertical intersect line
		//do nothing now
		bFlag = false;
	}
	else
	{//no constraint can be used
		bFlag = false;
	}

	return bFlag;
}

int IndexPlaneBySegNumber(const vector<Plane>& vecPlanes, int segNumber)
{
	int index = -1;

	for (int i=0; i<vecPlanes.size(); i++)
	{
		if (vecPlanes[i].Number() == segNumber)
		{
			index = i;
			break;
		}
	}

	return index;
}

//Old try to find 3 faces intersect at one point 
//then hypotheses this intersect point
bool RefineCornerOf3Faces(vector<Plane>& vecLocalPlanes, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	double maxAngle = 5*PI/180;
	double maxDistance = 10.0;

	vector<int> curAdjFaceNums;
	vector<int> temAdjFaceNums;
	vector<int> thirdFaceNums;//store the third face who can form 3face corner with iPlane and temFace;

	LineTopology lineTop1, lineTop2, lineTop3;
	Line3D line1, line2, line3, stableLine;
	Plane plane;
	Position3D intesectPnt;
	int pntNum11, pntNum12, pntNum21, pntNum22, pntNum31, pntNum32;
	int planeNum1, planeNum2, planeNum3;

	for (int iPlane=0; iPlane<vecLocalPlanes.size(); iPlane++)
		vecLocalPlanes[iPlane].Label() = -1;//-1 un-refined; 1 refined

	for (int iPlane=0; iPlane<vecLocalPlanes.size(); iPlane++)
	{
		planeNum1 = vecLocalPlanes[iPlane].Number();
		SearchAdjacentFaces(localLineTops, planeNum1, curAdjFaceNums);

		for (int j=0; j<curAdjFaceNums.size(); j++)
		{
			planeNum2 = curAdjFaceNums[j];
			if (vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planeNum2)].Label() != -1)
				continue;//refined

			SearchFaceBy2Faces(localLineTops, planeNum1, planeNum2, thirdFaceNums);
			/*		SearchAdjacentFaces(localLineTops, planeNum2, temAdjFaceNums);
			vector<int> vecTem = curAdjFaceNums;
			vecTem.insert(vecTem.end(), temAdjFaceNums.begin(), temAdjFaceNums.end());
			std::sort(vecTem.begin(), vecTem.end());

			//search the third face
			thirdFaceNums.clear();
			for (int k=0; k<vecTem.size()-1; k++)
			{
			if (vecTem[k] == planeNum1 || vecTem[k] == planeNum2)
			continue;

			if (vecTem[k] == vecTem[k+1] )
			thirdFaceNums.push_back(vecTem[k]);
			}*/

			for (int k=0; k<thirdFaceNums.size(); k++)
			{
				planeNum3 = thirdFaceNums[k];
				if (vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planeNum3)].Label() != -1)
					continue;//refined

				if (!SearchLineBy2Faces(localLineTops, planeNum1, planeNum2, lineTop1)
					|| !SearchLineBy2Faces(localLineTops, planeNum1, planeNum3, lineTop2)
					|| !SearchLineBy2Faces(localLineTops, planeNum3, planeNum2, lineTop3))
					continue;

				pntNum11 = lineTop1[0].Number();
				pntNum12 = lineTop1[1].Number();
				pntNum21 = lineTop2[0].Number();
				pntNum22 = lineTop2[1].Number();
				pntNum31 = lineTop3[0].Number();
				pntNum32 = lineTop3[1].Number();

				line1 = Line3D(localObjPnts[pntNum11] , localObjPnts[pntNum12]);
				line2 = Line3D(localObjPnts[pntNum21] , localObjPnts[pntNum22]);
				line3 = Line3D(localObjPnts[pntNum31] , localObjPnts[pntNum32]);

				//find most stable line, which is vertical or horizontal
				if (Angle(line1.Direction(), Vector3D(0, 0, 1)) < maxAngle 
					|| line1.IsHorizontal(maxAngle) )
				{
					stableLine = line1;
					plane = vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planeNum3)];
				}
				else if (Angle(line2.Direction(), Vector3D(0, 0, 1)) < maxAngle 
					|| line2.IsHorizontal(maxAngle))
				{
					stableLine = line2;
					plane = vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planeNum2)];
				}
				else 
				{
					stableLine = line3;
					plane = vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planeNum1)];
				}

				if (MyIntersectLine3DPlane(stableLine, plane, intesectPnt))
				{
					ReplaceLineNode(localObjPnts, lineTop1, intesectPnt);
					ReplaceLineNode(localObjPnts, lineTop2, intesectPnt);
					ReplaceLineNode(localObjPnts, lineTop3, intesectPnt);
				}

				/*				//first line
				double dist1 = localObjPnts[pntNum11].Distance(intesectPnt);
				double dist2 = localObjPnts[pntNum12].Distance(intesectPnt);
				if ( dist1 < dist2 && dist1 < maxDistance )
				{
				localObjPnts[pntNum11].SetX(intesectPnt.X());
				localObjPnts[pntNum11].SetY(intesectPnt.Y());
				localObjPnts[pntNum11].SetZ(intesectPnt.Z());
				}
				else if (dist2 < maxDistance)
				{
				localObjPnts[pntNum12].SetX(intesectPnt.X());
				localObjPnts[pntNum12].SetY(intesectPnt.Y());
				localObjPnts[pntNum12].SetZ(intesectPnt.Z());
				}

				//second line
				dist1 = localObjPnts[pntNum21].Distance(intesectPnt) ;
				dist2 = localObjPnts[pntNum22].Distance(intesectPnt);
				if ( dist1 < dist2 && dist1 < maxDistance )
				{
				localObjPnts[pntNum21].SetX(intesectPnt.X());
				localObjPnts[pntNum21].SetY(intesectPnt.Y());
				localObjPnts[pntNum21].SetZ(intesectPnt.Z());
				}
				else if (dist2 < maxDistance)
				{
				localObjPnts[pntNum22].SetX(intesectPnt.X());
				localObjPnts[pntNum22].SetY(intesectPnt.Y());
				localObjPnts[pntNum22].SetZ(intesectPnt.Z());
				}

				//third line
				dist1 = localObjPnts[pntNum31].Distance(intesectPnt) ;
				dist2 = localObjPnts[pntNum32].Distance(intesectPnt);
				if ( dist1 < dist2 && dist1 < maxDistance )
				{
				localObjPnts[pntNum31].SetX(intesectPnt.X());
				localObjPnts[pntNum31].SetY(intesectPnt.Y());
				localObjPnts[pntNum31].SetZ(intesectPnt.Z());
				}
				else if (dist2 < maxDistance)
				{
				localObjPnts[pntNum32].SetX(intesectPnt.X());
				localObjPnts[pntNum32].SetY(intesectPnt.Y());
				localObjPnts[pntNum32].SetZ(intesectPnt.Z());
				}*/
			}
		}

		vecLocalPlanes[iPlane].Label() = 1;
	}

	return true;
}

bool Search4AdjFaces(const std::vector<Plane>& vecLocalPlanes, const LineTopologies& localLineTops, vector<vector<int> >& vecOutAdjFaces)
{
	vecOutAdjFaces.clear();
	vector<int> AdjFaces1, AdjFaces2, AdjFaces3, AdjFaces4;
	vector<int> forthFaces;
	int planeNum1, planeNum2, planeNum3, planeNum4;
	bool bFlag = false;
	vector<int> curAdjFaceNum, temAdjFaceNums;
	LineTopology temLinTop;

	//search for 4face corners
	for (int iPlane=0; iPlane<vecLocalPlanes.size(); iPlane++)
	{
		planeNum1 = vecLocalPlanes[iPlane].Number();
		SearchAdjacentFaces(localLineTops, planeNum1, AdjFaces1);

		for (int j=0; j<AdjFaces1.size(); j++)
		{
			planeNum2 = AdjFaces1[j];

			for (int k=j+1; k<AdjFaces1.size(); k++)
			{
				planeNum3 = AdjFaces1[k];

				//there is a link bettwen face 2 and face 3, they form a 3 face corner with face 1;
				if (SearchLineBy2Faces(localLineTops, planeNum2, planeNum3, temLinTop))
					continue;

				SearchFaceBy2Faces(localLineTops, planeNum2, planeNum3, forthFaces);

				for (int l=0; l< forthFaces.size(); l++)
				{
					//  1 ----2
					//  | \  /
					//  |   4  
					//  | /   
					//  3 
					//face 1 was searched
					//or face 1 and face 4 have link, so face 1, 4, 2 and face 1, 4, 3 form two 3-face corner
					planeNum4 = forthFaces[l];
					if (planeNum4 == planeNum1
						|| SearchLineBy2Faces(localLineTops, planeNum1, planeNum4, temLinTop))
						continue;

					//  1 ----2
					//  | \   |
					//  |  5  |
					//  |   \ |
					//  3 ----4
					//search is there a node connecting face 1 and face 4 which is not face 2 and face 3
					//In this way faces 1-2-3-4 can not form 4face corner, but 1-3-4-5 and 1-2-3-5 form 4face corner
					vector<int> temVec;
					SearchFaceBy2Faces(localLineTops, planeNum1, planeNum4, temVec);

					if ( temVec.size() > 2)
						continue;

					temAdjFaceNums.clear();
					temAdjFaceNums.push_back(planeNum1);
					temAdjFaceNums.push_back(planeNum2);
					temAdjFaceNums.push_back(planeNum3);
					temAdjFaceNums.push_back(planeNum4);
					vecOutAdjFaces.push_back(temAdjFaceNums);
				}
			}
		}
	}

	//////////////////////////////////////////////////////////////////////////
	//sort the planes according to graph and their number
	for (int i=0; i<vecOutAdjFaces.size(); i++)
	{
		planeNum1 = vecOutAdjFaces[i][0];
		planeNum2 = vecOutAdjFaces[i][1];
		planeNum3 = vecOutAdjFaces[i][2];
		planeNum4 = vecOutAdjFaces[i][3];

		if (planeNum1 > planeNum4)
			std::swap(planeNum1, planeNum4);

		if (planeNum2 > planeNum3)
			std::swap(planeNum2, planeNum3);

		if (planeNum1 > planeNum2)
		{
			std::swap(planeNum1, planeNum2);
			std::swap(planeNum3, planeNum4);
		}

		vecOutAdjFaces[i][0] = planeNum1;
		vecOutAdjFaces[i][1] = planeNum2;
		vecOutAdjFaces[i][2] = planeNum3;
		vecOutAdjFaces[i][3] = planeNum4;
	}


	/////////////////////////////////////////////////////
	//remove repeating loops
	for (int i=0; i<vecOutAdjFaces.size(); i++)
	{
		curAdjFaceNum = vecOutAdjFaces[i];
		for (int j=i+1; j<vecOutAdjFaces.size(); j++)
		{
			temAdjFaceNums = vecOutAdjFaces[j];

			if (curAdjFaceNum[0]==temAdjFaceNums[0] 
			&& curAdjFaceNum[1]==temAdjFaceNums[1]
			&& curAdjFaceNum[2]==temAdjFaceNums[2] 
			&& curAdjFaceNum[3]==temAdjFaceNums[3])
			{
				vecOutAdjFaces.erase(vecOutAdjFaces.begin()+j);
				j--;
			}
		}
	}

	if (vecOutAdjFaces.empty())
		return false;
	else
		return true;
}


bool RefineCornerOf4Faces(std::vector<Plane>& vecLocalPlanes, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	double maxAngle = 5*PI/180;
	double maxDistance = 10.0;
	double angPlan2Line = 20*PI/180;

	vector<int> curAdjFaceNums;
	vector<int> thirdFaceNums;//store the third face who can form 3face corner with iPlane and temFace;

	LineTopology lineTop1, lineTop2, lineTop3, lineTop4;
	Line3D line1, line2, line3, line4, stableLine;
	Plane plane1, plane2, plane3, plane4;
	Plane plane;
	Position3D intesectPnt, averagePnt;
	int nIntPntNum;
	int pntNum11, pntNum12, pntNum21, pntNum22, pntNum31,pntNum32, pntNum41, pntNum42;
	int planeNum1, planeNum2, planeNum3, planeNum4;
	vector<vector<int> > vecAdjFaces;

	Search4AdjFaces(vecLocalPlanes, localLineTops, vecAdjFaces);

	for (int i=0; i<vecAdjFaces.size(); i++)
	{
		curAdjFaceNums = vecAdjFaces[i];

		SearchLineBy2Faces(localLineTops, curAdjFaceNums[0], curAdjFaceNums[1], lineTop1);
		SearchLineBy2Faces(localLineTops, curAdjFaceNums[0], curAdjFaceNums[2], lineTop2);
		SearchLineBy2Faces(localLineTops, curAdjFaceNums[3], curAdjFaceNums[1], lineTop3);
		SearchLineBy2Faces(localLineTops, curAdjFaceNums[3], curAdjFaceNums[2], lineTop4);

		pntNum11 = lineTop1[0].Number(); pntNum12 = lineTop1[1].Number();
		pntNum21 = lineTop2[0].Number(); pntNum22 = lineTop2[1].Number();
		pntNum31 = lineTop3[0].Number(); pntNum32 = lineTop3[1].Number();
		pntNum41 = lineTop4[0].Number(); pntNum42 = lineTop4[1].Number();

		line1 = Line3D(localObjPnts[pntNum11], localObjPnts[pntNum12]);
		line2 = Line3D(localObjPnts[pntNum21], localObjPnts[pntNum22]);
		line3 = Line3D(localObjPnts[pntNum31], localObjPnts[pntNum32]);
		line4 = Line3D(localObjPnts[pntNum41], localObjPnts[pntNum42]);

		//determine whether intersection points are near enough
		Position3D intPnt1, intPnt2, intPnt3, intPnt4;
		MyIntersect2Lines(line1,line2,intPnt1);
		MyIntersect2Lines(line2,line3,intPnt2);
		MyIntersect2Lines(line3,line4,intPnt3);
		MyIntersect2Lines(line4,line1,intPnt4);

		averagePnt.X() = 0; averagePnt.Y() = 0; averagePnt.Z() = 0;
		averagePnt.X() += intPnt1.X()/4; averagePnt.Y() += intPnt1.Y()/4; averagePnt.Z() += intPnt1.Z()/4;
		averagePnt.X() += intPnt2.X()/4; averagePnt.Y() += intPnt2.Y()/4; averagePnt.Z() += intPnt2.Z()/4;
		averagePnt.X() += intPnt3.X()/4; averagePnt.Y() += intPnt3.Y()/4; averagePnt.Z() += intPnt3.Z()/4;
		averagePnt.X() += intPnt4.X()/4; averagePnt.Y() += intPnt4.Y()/4; averagePnt.Z() += intPnt4.Z()/4;

		if (averagePnt.Distance(intPnt1)>1.0 
			|| averagePnt.Distance(intPnt2)>1.0
			|| averagePnt.Distance(intPnt3)>1.0 
			|| averagePnt.Distance(intPnt4)>1.0)
			continue;//if the intersection pints are too far away, they cannot form 4faces corner

		int planSegNum1 = curAdjFaceNums[0];
		int planSegNum2 = curAdjFaceNums[1];
		int planSegNum3 = curAdjFaceNums[2];
		int planSegNum4 = curAdjFaceNums[3];

		plane1 = vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planSegNum1)];
		plane2 = vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planSegNum2)];
		plane3 = vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planSegNum3)];
		plane4 = vecLocalPlanes[IndexPlaneBySegNumber(vecLocalPlanes, planSegNum4)];

		bool bFlag = false;
		nIntPntNum = 0;
		averagePnt.X() = 0;
		averagePnt.Y() = 0;
		averagePnt.Z() = 0;

		// fist line
		if (MyIsParaller(line1.Direction(), Vector3D(0, 0, 1), maxAngle) 
			|| line1.IsHorizontal(maxAngle))
		{
			if (!MyIsParaller(plane3, line1,  angPlan2Line))
			{
				bFlag = true;
				MyIntersectLine3DPlane(line1, plane3, intesectPnt);
				nIntPntNum++;
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane4, line1, angPlan2Line))
			{
				bFlag = true;
				MyIntersectLine3DPlane(line1, plane4, intesectPnt);
				nIntPntNum++;
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}
		}

		// second line
		if (MyIsParaller(line2.Direction(), Vector3D(0, 0, 1), maxAngle) 
			|| line2.IsHorizontal(maxAngle))
		{
			if (!MyIsParaller(plane2, line2, angPlan2Line))
			{
				bFlag = true;
				nIntPntNum++;
				MyIntersectLine3DPlane(line2, plane2, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane4, line2, angPlan2Line))
			{
				bFlag = true;
				nIntPntNum++;
				MyIntersectLine3DPlane(line2, plane4, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}
		}

		//third line
		if (MyIsParaller(line3.Direction(), Vector3D(0, 0, 1), maxAngle)
			|| line3.IsHorizontal(maxAngle))
		{
			if (!MyIsParaller(plane1, line3, angPlan2Line))
			{
				bFlag = true;
				nIntPntNum++;
				MyIntersectLine3DPlane(line3, plane1, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane3, line3, angPlan2Line))
			{
				bFlag = true;
				nIntPntNum++;
				MyIntersectLine3DPlane(line3, plane3, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}
		}

		//forth line
		if (MyIsParaller(line4.Direction(), Vector3D(0, 0, 1), maxAngle) < maxAngle 
			|| line4.IsHorizontal(maxAngle))
		{
			if (!MyIsParaller(plane1, line4, angPlan2Line))
			{
				bFlag = true;
				nIntPntNum++;
				MyIntersectLine3DPlane(line4, plane1, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane2, line4, angPlan2Line))
			{
				bFlag = true;
				nIntPntNum++;
				MyIntersectLine3DPlane(line4, plane2, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}
		}

		//if there is no stable line, just choose one. 
		//later the longest line would be chosen as stable line
		if (bFlag == false)
		{
			if (!MyIsParaller(plane3, line1, angPlan2Line))
			{
				MyIntersectLine3DPlane(line1, plane3, intesectPnt);
				nIntPntNum++;
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane4, line1, angPlan2Line))
			{
				MyIntersectLine3DPlane(line1, plane4, intesectPnt);
				nIntPntNum++;
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane2, line2, angPlan2Line))
			{
				nIntPntNum++;
				MyIntersectLine3DPlane(line2, plane2, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane4, line2, angPlan2Line))
			{
				nIntPntNum++;
				MyIntersectLine3DPlane(line2, plane4, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane1, line3, angPlan2Line))
			{
				nIntPntNum++;
				MyIntersectLine3DPlane(line3, plane1, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane3, line3, angPlan2Line))
			{
				nIntPntNum++;
				MyIntersectLine3DPlane(line3, plane3, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane1, line4, angPlan2Line))
			{
				nIntPntNum++;
				MyIntersectLine3DPlane(line4, plane1, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}

			if (!MyIsParaller(plane2, line4, angPlan2Line))
			{
				nIntPntNum++;
				MyIntersectLine3DPlane(line4, plane2, intesectPnt);
				averagePnt.X() += intesectPnt.X();
				averagePnt.Y() += intesectPnt.Y();
				averagePnt.Z() += intesectPnt.Z();
			}
		}

		averagePnt.X() /= nIntPntNum;
		averagePnt.Y() /= nIntPntNum;
		averagePnt.Z() /= nIntPntNum;

		ReplaceLineNode(localObjPnts, lineTop1, averagePnt);
		ReplaceLineNode(localObjPnts, lineTop2, averagePnt);
		ReplaceLineNode(localObjPnts, lineTop3, averagePnt);
		ReplaceLineNode(localObjPnts, lineTop4, averagePnt);
	}
}

bool RefineOutBoundaryPlnae(const LaserPoints& gLaserPnts, std::vector<Plane>& vecLocalPlanes,
	std::vector<PointNumberList>& vecLocalSegPnts, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	PointNumberList pnlOrgAllBoundPnts, pnlOrgTemBoundPnts, pnlTemBoundPnts;
	PointNumberList pnlTemSegPnts;
	vector<int> vecTemLineNums;
	int curPlaneNum;
	LaserPoint temLaserPnt;
	Line3D temLine;
	ObjectPoint p1, p2;
	double dist;
	double minDist = 0.5;
	bool bNearEdge;
	Plane plane;
	int pntNums;
	int minPntNums = 5;
	PointNumber orgPntNum;
	vector<int> vecPlneNums, vecTemLineNum;

	//double minDist = 0.3;

	//search for out boundary points
	//which first are boundary points and don't near inner edge
	for (int i=0; i<vecLocalSegPnts.size(); i++)
	{
		pnlTemSegPnts = vecLocalSegPnts[i];
		curPlaneNum = gLaserPnts[pnlTemSegPnts[0].Number()].Attribute(SegmentNumberTag);
		SearchLinesOnPlane(localLineTops, curPlaneNum, vecTemLineNums);
		MyDeriveContour(gLaserPnts, pnlTemSegPnts, pnlTemBoundPnts);
	
		for (int j=0; j<pnlTemBoundPnts.size(); j++)
		{
			temLaserPnt = gLaserPnts[pnlTemBoundPnts[j].Number()];

			bNearEdge = false;
			for (int k=0; k<localLineTops.size(); k++)
			{
				p1 = localObjPnts[localLineTops[k][0].Number()];
				p2 = localObjPnts[localLineTops[k][1].Number()];
				temLine = Line3D(p1, p2);

				dist = temLine.DistanceToPoint( (Position3D)temLaserPnt);
				if (dist < minDist)
				{
					bNearEdge = true;
					break;
				}
			}

			if (!bNearEdge) pnlOrgAllBoundPnts.push_back(pnlTemBoundPnts[j]);
		}
	}

	LaserPoints selLaserPnts;
	for (int i=0; i<pnlOrgAllBoundPnts.size(); i++)
		selLaserPnts.push_back(gLaserPnts[pnlOrgAllBoundPnts[i].Number()]);
	
	//selLaserPnts.Write("c:\\laserpoints.laser", 0, true);
	selLaserPnts.Label(-1);
	
	//search planes which fit the out boundary points
	HoughSpace houghspace;
	selLaserPnts.InitialiseHoughSpace(houghspace, -1, 85*PI/180, 5*PI/180, 0.5, 1, LabelTag);
	selLaserPnts.IncrementHoughSpace(houghspace);
	do 
	{
		pntNums = 0;
		plane = houghspace.BestPlane(&pntNums, 0, 3); 
		
		if (pntNums < minPntNums) continue;
		pntNums = selLaserPnts.Label(plane, minDist, -1, -1,  1, 0);
		
		//fit the plane again
		if (pntNums < minPntNums) continue;
		plane = selLaserPnts.FitPlane(1, 1);
		if (pntNums < minPntNums) continue;
		pntNums = selLaserPnts.Label(plane, minDist, 0, 1, 1, 0);
		
		if (pntNums >= minPntNums)
		{
			pnlTemSegPnts = selLaserPnts.TaggedPointNumberList(LabelTag,1);
			vecPlneNums.clear();
			pnlOrgTemBoundPnts.clear();
			//double minHeight = 99999999;

			//remove detected points from hough space
			//and get planes of corresponding segments
			for (int i=0; i<pnlTemSegPnts.size(); i++)
			{	
				temLaserPnt = selLaserPnts[pnlTemSegPnts[i].Number()];
				houghspace.RemovePoint(temLaserPnt.X(), temLaserPnt.Y(), temLaserPnt.Z());

				//if (temLaserPnt.Z() < minHeight) 
				//	minHeight = temLaserPnt.Z();

				//transform template point number to original point number
				orgPntNum = pnlOrgAllBoundPnts[pnlTemSegPnts[i].Number()];
				pnlOrgTemBoundPnts.push_back(orgPntNum);
				int planNum= gLaserPnts[orgPntNum.Number()].Attribute(SegmentNumberTag);
				//IndexPlaneBySegNumber(vecLocalPlanes, gLaserPnts[orgPntNum.Number()].Attribute(SegmentNumberTag));
				vecPlneNums.push_back(planNum);
			}

			//remove redundancy plane numbers
			std::sort(vecPlneNums.begin(), vecPlneNums.end());
			for (int i=1; i<vecPlneNums.size(); i++)
			{
				if (vecPlneNums[i] == vecPlneNums[i-1])
				{
					vecPlneNums.erase(vecPlneNums.begin()+i);
					i--;
				}
			}
            
            Plane temPlane;
			//if plane is near horizontal, constraint fit it to be horizontal
			if (plane.IsHorizontal(15*PI/180))
			{
				temPlane = gLaserPnts.FitPlane(pnlOrgTemBoundPnts, Vector3D(0,0,1), 0);
				plane = temPlane;
            }
				
			//Warning:
			//hypothesis plane does not correspond to a point segment
			//so it's number do not have corresponding segment number
			plane.Number() = 100000000+vecLocalPlanes.size();
			vecLocalPlanes.push_back(plane);
			
			//intersect hypothesis plane with planes of segment points
			for (int i=0; i<vecPlneNums.size(); i++)
			{
				int planInd= IndexPlaneBySegNumber(vecLocalPlanes, vecPlneNums[i]) ;
				if(planInd<0) continue; 
				Plane temPlane = vecLocalPlanes[planInd];
				Position3D p1, p2;
				bool bFindSeg = false;
				PointNumberList pnlTemSeg;
				
				//search for corresponding point segment with plane number
				for (int j=0; j<vecLocalSegPnts.size(); j++)
				{
					pnlTemSeg = vecLocalSegPnts[j];
					if (gLaserPnts[pnlTemSeg[0].Number()].Attribute(SegmentNumberTag) == temPlane.Number())
					{
						bFindSeg = true;
						break;
					}
				}
				if (!bFindSeg) continue;//cannot find corresponding point segment, skip it

				//MyIsParaller()
				gLaserPnts.IntersectFaces(pnlTemSeg, pnlOrgTemBoundPnts, temPlane, plane, INTERSECT_FACE_MIN_DIST, p1, p2);

				if (p1.Distance(p2) < 0.5 )
					continue;//intersection line is too short

				ObjectPoint objPnt;
				LineTopology lineTop;
				
				objPnt.X() = p1.X(); objPnt.Y() = p1.Y(); objPnt.Z() = p1.Z(); 
				objPnt.Number() = localObjPnts.size();
				localObjPnts.push_back(objPnt);
				lineTop.push_back(objPnt.Number());

				objPnt.X() = p2.X(); objPnt.Y() = p2.Y(); objPnt.Z() = p2.Z(); 
				objPnt.Number() = localObjPnts.size();
				localObjPnts.push_back(objPnt);
				lineTop.push_back(objPnt.Number());

				lineTop.Number() = localLineTops.size();
				lineTop.SetAttribute(0, plane.Number());
				lineTop.SetAttribute(1, temPlane.Number());
				localLineTops.push_back(lineTop);
			}
		}
	} while (pntNums > minPntNums);
	
	return true;
}



//nstruct this point
/*bool RefineLocalBuilding(const vector<Plane>& vecLocalPlanes, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
//	Line3D curLine, leftLine, rightLine;
//	Vector3D dir;
//	Plane plane;
//	Position3D intPnt;
LineTopology topCurLine, topLeftLine, topRightLine;
vector<LineTopology> vecLineTops;
int lFaceNum, rFaceNum;
vector<LineTopology> vecPossibleNeibLines, vecNeibLineTops;
vector<int> vecNeibPlanes;
int numNeibPlane;

for (int i=0; i< localLineTops.size(); i++)
{
if (!localLineTops[i].HasAttribute(0) && !localLineTops[i].HasAttribute(1))
return false;//each intersect line are not formed by intersecting two faces
}

//for (int i=0; i< localLineTops.size(); i++)
//	localLineTops[i].SetAttribute(2, -1);//-1 un-refined; 1 refined

for (int i=0; i< localLineTops.size(); i++)
{
topCurLine = localLineTops[i];

//if(topCurLine.Attribute(2) == 1)
//	continue;//this line is refined, move to next line

//if (!topCurLine.HasAttribute(0) && !topCurLine.HasAttribute(1))
//	return false;

lFaceNum = topCurLine.Attribute(0);
rFaceNum = topCurLine.Attribute(1);

//search for lines which are on the left face or right face of current line
vecPossibleNeibLines.clear();
for (int j=i+1; j < localLineTops.size(); j++)
{
LineTopology temLine = localLineTops[j];

if (temLine.Attribute(0) == lFaceNum || temLine.Attribute(0) == rFaceNum 
|| temLine.Attribute(1) == lFaceNum || temLine.Attribute(1) == lFaceNum )
vecPossibleNeibLines.push_back(temLine);			
}

if(vecPossibleNeibLines.size() < 2)
continue; //not enough neighbor lines

//search for face which is neighbor with left face and right at the same time
vecNeibPlanes.clear();
vecNeibLineTops.clear();
for (int j=0; j < vecPossibleNeibLines.size(); j++)
{		
topLeftLine = vecPossibleNeibLines[j]; 

for (int k=j+1; k < vecPossibleNeibLines.size(); k++)
{
//has same neighbor face
topRightLine = vecPossibleNeibLines[k];

if ( topLeftLine.Attribute(0) == topRightLine.Attribute(0) 
|| topLeftLine.Attribute(0) == topRightLine.Attribute(1))
numNeibPlane = topLeftLine.Attribute(0);
else if (topLeftLine.Attribute(1) == topRightLine.Attribute(0)
|| topLeftLine.Attribute(1) == topRightLine.Attribute(1))
numNeibPlane = topLeftLine.Attribute(1);
else
continue;

//the same neighbor face should not be the left face or right face of current line
if (numNeibPlane != lFaceNum && numNeibPlane != rFaceNum)
{
vecNeibPlanes.push_back(numNeibPlane);
vecLineTops.push_back(topLeftLine);
vecLineTops.push_back(topRightLine);
}
}
}

//refine end points of 3 lines which intersect at one point
Position3D startPnt = (Position3D)localObjPnts[topCurLine[0].Number()];
Position3D endPnt =  (Position3D)localObjPnts[topCurLine[1].Number()];
Line3D curLine = Line3D(startPnt, endPnt);

for (int j=0; j<vecNeibPlanes.size(); j++)
{
int planeIndex = IndexPlaneBySegNumber(vecLocalPlanes, vecNeibPlanes[j]);
if (planeIndex == -1)
continue;

Plane plane = vecLocalPlanes[planeIndex];
Position3D intPnt;

if (!MyIntersectLine3DPlane(curLine, plane, intPnt))
continue; //line does not intersect with plane

ReplaceLineNode(localObjPnts, topCurLine, intPnt);
ReplaceLineNode(localObjPnts, vecLineTops[j+0], intPnt);
ReplaceLineNode(localObjPnts, vecLineTops[j+1], intPnt);
}
}
}*/

//try to find nearby nodes on nearby lines
//then replays this nodes with their average points
bool RefineLocalBuilding(const vector<Plane>& vecLocalPlanes, ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	double maxPntDist = 0.5;
	Plane curPlane;
	vector<int> vecTemTopNums;
	Vector3D pnt1, pnt2, pnt3, pnt4;
	int numPnt1, numPnt2, numPnt3, numPnt4;
	Line3D curLine, temLine;
	Vector3D intPos;

	for (int i=0; i<vecLocalPlanes.size(); i++)
	{
		curPlane = vecLocalPlanes[i];
		if(!SearchLinesOnPlane(localLineTops, curPlane.Number(), vecTemTopNums)
			|| vecTemTopNums.size() < 2)
			continue;

		for (int j=0; j<vecTemTopNums.size(); j++ )
		{
			if (localLineTops[j].Attribute(2) != -1)
				continue;

			numPnt1 = localLineTops[vecTemTopNums[j]][0].Number();
			numPnt2 = localLineTops[vecTemTopNums[j]][1].Number();
			pnt1 = (Vector3D)localObjPnts[numPnt1];
			pnt2 = (Vector3D)localObjPnts[numPnt2];
			curLine = Line3D(pnt1, pnt2-pnt1);

			for (int k=j+1; k<vecTemTopNums.size(); k++)
			{
				if (localLineTops[k].Attribute(2) != -1)
					continue;

				numPnt3 = localLineTops[vecTemTopNums[k]][0].Number();
				numPnt4 = localLineTops[vecTemTopNums[k]][1].Number();
				pnt3 = (Vector3D)localObjPnts[numPnt3];
				pnt4 = (Vector3D)localObjPnts[numPnt4];
				temLine = Line3D(pnt3, pnt4-pnt3);

				if(!MyIntersect2Lines(curLine, temLine, intPos))
					continue;

				/*		if ((pnt1-intPos).Length() < maxPntDist)
				{
				localObjPnts[numPnt1].SetX(intPos.X());
				localObjPnts[numPnt1].SetY(intPos.Y());
				localObjPnts[numPnt1].SetZ(intPos.Z());
				} 
				else if ( (pnt2-intPos).Length() < maxPntDist)
				{
				localObjPnts[numPnt2].SetX(intPos.X());
				localObjPnts[numPnt2].SetY(intPos.Y());
				localObjPnts[numPnt2].SetZ(intPos.Z());
				}

				if ( (pnt3-intPos).Length() < maxPntDist)
				{
				localObjPnts[numPnt3].SetX(intPos.X());
				localObjPnts[numPnt3].SetY(intPos.Y());
				localObjPnts[numPnt3].SetZ(intPos.Z());
				} 
				else if ( (pnt4-intPos).Length() < maxPntDist)
				{
				localObjPnts[numPnt4].SetX(intPos.X());
				localObjPnts[numPnt4].SetY(intPos.Y());
				localObjPnts[numPnt4].SetZ(intPos.Z());
				}*/
			}
		}
	}

	/*	vector<int> vecNeibNodes;
	Vector3D averagePnt;

	for (int i=0; i<localLineTops.size(); i++)
	{
	if (localLineTops[i].Attribute(2) != -1)
	continue;//this line has been refined

	//first node of this line	
	if (SearchNodesOnNeigborLine(localObjPnts, localLineTops, i, 0, vecNeibNodes, 1.0))
	{
	averagePnt = Vector3D(0, 0, 0);
	for (int j=0; j<vecNeibNodes.size(); j++)
	averagePnt += (Vector3D)localObjPnts[vecNeibNodes[j]];

	averagePnt /= localObjPnts.size();

	localObjPnts[localLineTops[i][0].Number()].SetX(averagePnt.X());
	localObjPnts[localLineTops[i][0].Number()].SetY(averagePnt.Y());
	localObjPnts[localLineTops[i][0].Number()].SetZ(averagePnt.Z());

	for (int j=0; j<vecNeibNodes.size(); j++)
	{
	localObjPnts[j].SetX(averagePnt.X());
	localObjPnts[j].SetY(averagePnt.Y());
	localObjPnts[j].SetZ(averagePnt.Z());
	}
	}

	//second node of this line
	if (SearchNodesOnNeigborLine(localObjPnts, localLineTops, i, 1, vecNeibNodes,1.0))
	{		
	averagePnt = Vector3D(0, 0, 0);
	for (int j=0; j<vecNeibNodes.size(); j++)
	averagePnt += (Vector3D)localObjPnts[vecNeibNodes[j]];

	averagePnt /= localObjPnts.size();

	localObjPnts[localLineTops[i][1].Number()].SetX(averagePnt.X());
	localObjPnts[localLineTops[i][1].Number()].SetY(averagePnt.Y());
	localObjPnts[localLineTops[i][1].Number()].SetZ(averagePnt.Z());

	for (int j=0; j<vecNeibNodes.size(); j++)
	{
	localObjPnts[j].SetX(averagePnt.X());
	localObjPnts[j].SetY(averagePnt.Y());
	localObjPnts[j].SetZ(averagePnt.Z());
	}
	}

	localLineTops[i].SetAttribute(2, 1);
	}*/

	return true;
}


/*-----------------------------------
//0: start point; 1: end point; 2: max distance point; 3 last point
1------1-------2
\              \
2              4
\			    \
3------3-------4
*/
bool RefineSimpleDormer(const LaserPoints& gLaserPnts, const vector<Plane>& vecLocalPlanes, 
	ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	Plane curPlane;
	//LineTopologies temLineTops;
	vector<int> vecNiebLineNums;
	LineTopology curLineTop, temLineTop;
	Line3D line1, line2, line3, line4;
	Vector3D dir2;
	Position3D pnt1, pnt2, pnt3, pnt4, temPnt;
	LaserPoints laserPnts;
	double maxDist;

	for (int i=0; i<vecLocalPlanes.size(); i++)
	{
		curPlane = vecLocalPlanes[i];
		vecNiebLineNums.clear();
		SearchLinesOnPlane(localLineTops, curPlane.Number(), vecNiebLineNums);

		if ( vecNiebLineNums.size() != 1 //dormer plane only have one intersect line
			|| !curPlane.IsHorizontal(10*PI/180) //dormer plane should be horizontal
			/*|| temLineTops[0].Attribute(2) != -1*/) //intersect line should not be refined
			continue;

		curLineTop = localLineTops[vecNiebLineNums[0]];
		pnt1 = (Position3D)localObjPnts[curLineTop[0].Number()];
		pnt2 = (Position3D)localObjPnts[curLineTop[1].Number()];
		line1 = Line3D(pnt1, pnt2);

		//refine start point and end point according to laser points of this plane
		PointNumberList pntNumList = gLaserPnts.TaggedPointNumberList(SegmentNumberTag, curPlane.Number());
		double scaleBegin, scaleEnd;
		gLaserPnts.FaceNearLine(pntNumList, line1, 100, scaleBegin, scaleEnd);
		pnt1 = line1.Position(scaleBegin);
		pnt2 = line1.Position(scaleEnd);

		dir2 = line1.Direction().VectorProduct(curPlane.Normal());
		line2 = Line3D(pnt1, dir2);
		line4 = Line3D(pnt2, dir2);

		//search point 3
		laserPnts.clear();
		laserPnts.AddTaggedPoints(gLaserPnts, curPlane.Number(), SegmentNumberTag);
		maxDist = -99999.0;
		for (int j=0; j<laserPnts.size(); j++)
		{
			temPnt = line2.Project( Position3D(laserPnts[j].X(), laserPnts[j].Y(), laserPnts[j].Z()) );

			double temDist = temPnt.Distance(pnt1);
			if(temDist > maxDist )
			{
				maxDist = temDist;
				pnt3 = temPnt;
			}
		}

		pnt4 = pnt3 - pnt1 + pnt2;

		//first line
		localObjPnts[curLineTop[0].Number()].SetX(pnt1.X());
		localObjPnts[curLineTop[0].Number()].SetY(pnt1.Y());
		localObjPnts[curLineTop[0].Number()].SetZ(pnt1.Z());
		localObjPnts[curLineTop[1].Number()].SetX(pnt2.X());
		localObjPnts[curLineTop[1].Number()].SetY(pnt2.Y());
		localObjPnts[curLineTop[1].Number()].SetZ(pnt2.Z());
		localLineTops[vecNiebLineNums[0]].SetAttribute(2, 1);//refined

		int objPntNum = localObjPnts.size();
		int lineTopNum = localLineTops.size();
		//second line
		localObjPnts.push_back(ObjectPoint(pnt3.GetX(),pnt3.GetY(),pnt3.GetZ(),objPntNum,0,0,0,0,0,0));
		temLineTop = LineTopology(lineTopNum, -1, curLineTop[0].Number(), objPntNum);
		temLineTop.SetAttribute(0, curPlane.Number());
		temLineTop.SetAttribute(1, -1);//only have one neighbor plane
		temLineTop.SetAttribute(2, 1);//refined
		localLineTops.push_back(temLineTop);

		//third line
		localObjPnts.push_back(ObjectPoint(pnt4.GetX(),pnt4.GetY(),pnt4.GetZ(),objPntNum+1,0,0,0,0,0,0));
		temLineTop = LineTopology(lineTopNum+1, -1, objPntNum+1, objPntNum);
		temLineTop.SetAttribute(0, curPlane.Number());
		temLineTop.SetAttribute(1, -1);//only have one neighbor plane
		temLineTop.SetAttribute(2, 1);//refined
		localLineTops.push_back(temLineTop);

		//forth line
		temLineTop = LineTopology(lineTopNum+2, -1, curLineTop[1].Number(), objPntNum+1);
		temLineTop.SetAttribute(0, curPlane.Number());
		temLineTop.SetAttribute(1, -1);//only have one neighbor plane
		temLineTop.SetAttribute(2, 1);//refined
		localLineTops.push_back(temLineTop);
	}

	return true;
}

bool SearchLinesOnPlane(const LineTopologies& localLineTops, const int curPlaneNum, vector<int>& vecOutTopNums)
{
	//	int planeNum = plane.Number();
	LineTopology temLineTop;

	vecOutTopNums.clear();

	for (int i=0; i<localLineTops.size(); i++)
	{
		temLineTop = localLineTops[i];

		if (temLineTop.Attribute(0) == curPlaneNum
			|| temLineTop.Attribute(1) == curPlaneNum)
		{
			vecOutTopNums.push_back(i);
		}
	}

	if (vecOutTopNums.size()>0)
		return true;
	else
		return false;
}

bool SearchAdjacentFaces(const LineTopologies& localLineTops, const int curPlaneNum, vector<int>& vecAdjFaceNums)
{
	vecAdjFaceNums.clear();

	vector<int> vecLineTopNums;
	if (!SearchLinesOnPlane(localLineTops, curPlaneNum, vecLineTopNums))
		return false;

	int lPlaneNum, rPlaneNum;
	for ( int i=0; i<vecLineTopNums.size(); i++)
	{
		lPlaneNum = localLineTops[vecLineTopNums[i]].Attribute(0);
		rPlaneNum = localLineTops[vecLineTopNums[i]].Attribute(1);

		if (lPlaneNum != -1 && lPlaneNum != curPlaneNum)
			vecAdjFaceNums.push_back(lPlaneNum);
		else if( rPlaneNum != -1 )
			vecAdjFaceNums.push_back(rPlaneNum);
	}

	return true;
}


bool ReplaceLineNode(ObjectPoints& localObjPnts, const LineTopology& lineTop, const Position3D& point, double maxDistance )
{
	if (lineTop[0].Number() < 0
		|| lineTop[1].Number() < 0
		|| lineTop[0].Number() >= localObjPnts.size() 
		|| lineTop[1].Number() >= localObjPnts.size())
		return false;

	Position3D startPnt = (Position3D)localObjPnts[lineTop[0].Number()];
	Position3D endPnt = (Position3D)localObjPnts[lineTop[1].Number()];
	double dist1 = point.Distance(startPnt);
	double dist2 = point.Distance(endPnt);

	if (dist1 <= dist2 && dist1 < maxDistance)
	{
		localObjPnts[lineTop[0].Number()].SetX(point.X());
		localObjPnts[lineTop[0].Number()].SetY(point.Y());
		localObjPnts[lineTop[0].Number()].SetZ(point.Z());
	}
	else if(dist2 < maxDistance)
	{
		localObjPnts[lineTop[1].Number()].SetX(point.X());
		localObjPnts[lineTop[1].Number()].SetY(point.Y());
		localObjPnts[lineTop[1].Number()].SetZ(point.Z());
	}

	return true;
}


bool MyIntersect2Lines(const Line3D &lin1, const Line3D &lin2, Vector3D &pos)
{
	if ( Distance2Lines(lin1, lin2) > 0.5)
		return false;

	double angle = Angle2Lines(lin1, lin2);

	//first intersection
	Vector3D pnt1 = lin1.FootPoint();
	Vector3D dir1 = lin1.Direction().Normalize();
	double dist1 = lin2.DistanceToPoint(pnt1);
	Vector3D intsectPnt1 = pnt1 + dir1 * dist1/sin(angle); 
	if (lin2.DistanceToPoint(intsectPnt1) > 1)
		intsectPnt1 = pnt1 - dir1 * dist1/sin(angle); //inverse direction

	//second intersection
	Vector3D pnt2 = lin2.FootPoint();
	Vector3D dir2 = lin2.Direction().Normalize();
	double dist2 = lin1.DistanceToPoint(pnt2);
	Vector3D intsectPnt2 = pnt2 + dir2 * dist2/sin(angle); 
	if (lin1.DistanceToPoint(intsectPnt2) > 1)
		intsectPnt2 = pnt2 - dir2 * dist2/sin(angle); //inverse direction

	pos = (intsectPnt1 + intsectPnt2)/2;
	//	pos.SetX((intsectPnt1.X()+intsectPnt2.X())/2);
	//	pos.SetY((intsectPnt1.Y()+intsectPnt2.Y())/2);
	//	pos.SetZ((intsectPnt1.Z()+intsectPnt2.Z())/2);

	return true;
}


bool MyIntersectLine3DPlane(const Line3D &line, const Plane &plane, Position3D &point)
{
	Vector3D normal, direction;
	double   denom, scalar;

	/* Retrieve directions */

	normal = plane.Normal();
	direction = line.Direction();

	/* Check intersection angle */

	denom = normal.DotProduct(direction);
	if (fabs(denom) < 0.00001) return(0);

	/* Calculate intersection point */

	scalar = (plane.Distance() - line.FootPoint().DotProduct(normal)) / denom;
	point = line.Position(scalar);

	return(1);
}

double MyIsParaller(const Vector3D& lDir, const Vector3D& rDir, double maxAngle)
{
	double angle = acos(lDir.DotProduct(rDir)/(lDir.Length() * rDir.Length()));

	if (angle > PI/2)
		angle = PI - angle;

	if (angle < maxAngle)
		return true;
	else 
		return false;
}


double MyIsParaller(const Plane& plane, const Line3D& line, double maxAngle)
{
	Vector3D lDir = plane.Normal();
	Vector3D rDir = line.Direction();

	double angle = acos(lDir.DotProduct(rDir)/(lDir.Length() * rDir.Length()));

	if (angle > PI/2)
		angle = PI - angle;

	if (PI/2 - angle < maxAngle)
		return true;
	else
		return false;
}

bool GetCountour(const LaserPoints& gLaserPnts, const vector<PointNumberList>& vecLocalSegPnts, 
    ObjectPoints& localObjPnts, LineTopologies& localLineTops)
{
	int PlaneNum;
	PointNumberList pnlFace;
	TINEdges tinEdges;// = gLaserPnts.DeriveTIN();
	tinEdges.Derive(gLaserPnts.TINReference());
	LineTopology temLineTop;
	//LaserPoints FaceLaserPnts;
	localLineTops.clear();
	localObjPnts.clear();
	ObjectPoint temObjPnt(0, 0, 0,0, 0, 0, 0, 0, 0, 0);
	double pntSpace = gLaserPnts.MedianInterPointDistance((int)min(1000.0, (double)gLaserPnts.size()-2));

	for (int i=0; i<vecLocalSegPnts.size(); i++)
	{
		pnlFace = vecLocalSegPnts[i];
		MyDeriveContour(gLaserPnts, pnlFace, temLineTop);

		for (int j=0; j<temLineTop.size(); j++)
		{
			temObjPnt.X() = gLaserPnts[temLineTop[j].Number()].X();
			temObjPnt.Y() = gLaserPnts[temLineTop[j].Number()].Y();
			temObjPnt.Z() = gLaserPnts[temLineTop[j].Number()].Z();

			temObjPnt.Number() = localObjPnts.size();
			temLineTop[j].Number() = temObjPnt.Number();
			localObjPnts.push_back(temObjPnt);
		}
		temLineTop.Number() = localLineTops.size();
		localLineTops.push_back(temLineTop);
	}

	return true;
}

//clockwise search for boundary points
//factor:  a weight to determine minimum length of edge
//minimum length = factor * mean point space
bool MyDeriveContour(const LaserPoints& gLaserPnts, const PointNumberList &pnlSeg, PointNumberList& contour, 
	 const double factor, const Plane* pSegPlane)
{
	contour.clear();

	//if there are only 3 node, just return them
	if (pnlSeg.size() == 3) {
		contour.push_back(pnlSeg[0].Number());
		contour.push_back(pnlSeg[1].Number());
		contour.push_back(pnlSeg[2].Number());
		contour.push_back(pnlSeg[0].Number());
		return true;
	}
	else if (pnlSeg.size()<3) {
		return false;
	}

	TINEdgeSet				      edgeset;
	PointNumber startpoint, tempoint, prevpoint, nextpoint, curpoint;
	double                          maxX, X;
	bool bFoundNext;
	int startIndx, searchCount;
	int iterateNum;
	double temAng, prevAng, minAng;

	//fit plane 
	//	int segNumber = gLaserPnts[pnlSeg[0].Number()].Attribute(SegmentNumberTag);
	Plane plane; 
	Vector3D refDir;
	if (pSegPlane) plane = *pSegPlane;
	else plane = gLaserPnts.FitPlane(pnlSeg, 0);
	refDir = Vector3D(0,1,0);
	
	//find reference direction
//	if (plane.IsVertical(PI*10/180))
//		refDir = Vector3D(0,0,1);
//	else
//		refDir = Vector3D(0,1,0);

	//transform points to reference plane
	//initialize a local laser points and TIN
	Position2D temPos2D;
	LaserPoints lpLocalSeg;
	PointNumberList pnlLocalSeg;//(pnlSeg.size(), PointNumber(-1));
	vector<bool> vecIsValidTag(pnlSeg.size(), true);
	LaserPoint temLaserPnt;
	for (int i=0; i<pnlSeg.size(); i++)	{
		pnlLocalSeg.push_back(PointNumber(i));
		temLaserPnt = gLaserPnts[pnlSeg[i].Number()];
		temPos2D = Project2RefPlane(plane, temLaserPnt.Position3DRef());
		temLaserPnt.X() = temPos2D.X();
		temLaserPnt.Y() = temPos2D.Y();
		temLaserPnt.Z() = 0;
		lpLocalSeg.push_back(temLaserPnt);
	}
	//for debug
	//lpLocalSeg.Label(0);
	//lpLocalSeg.Write("E:\\test_data\\zzzprojectedPoints.laser", 0, true);
	lpLocalSeg.DeriveTIN();
	TINEdges tinEdges; 
	tinEdges.Derive(lpLocalSeg.TINReference());
	double minEdgeLen = lpLocalSeg.MedianInterPointDistance((int)min(1000.0, (double)lpLocalSeg.size()-2));
	minEdgeLen = factor*minEdgeLen;

	/////////////////////////////////////////////
	//search for start point, 
	//those points have not enough neighbor points will be skipped
	int neibEdgeCount;
	iterateNum = 0;
	do 	{
		maxX = -9999999;
		for (int i=0; i<pnlLocalSeg.size(); i++) {
			curpoint = pnlLocalSeg[i];
			if (vecIsValidTag[curpoint.Number()] == false) continue;
			//if (refDir == Vector3D(0,0,1))
			//	X = lpLocalSeg[curpoint.Number()].Z();
			//else
			//	X = lpLocalSeg[curpoint.Number()].X();
			X = lpLocalSeg[curpoint.Number()].X();
			if (X > maxX) {
				maxX = X; 
				startpoint = curpoint;
			}
		}

		//find whether this point has enough neighbor points
		neibEdgeCount = 0;
		if (vecIsValidTag[startpoint.Number()] == false) continue;
		edgeset = tinEdges[startpoint.Number()];
		LaserPoint p1, p2;
		p1 = lpLocalSeg[startpoint.Number()];
		for (int i=0; i<edgeset.size(); i++) {
			tempoint = edgeset[i];
			if (vecIsValidTag[tempoint.Number()] == false) continue;
			p2 = lpLocalSeg[tempoint.Number()];
			if (p1.Distance(p2) < minEdgeLen) ++neibEdgeCount;
		}
		if(neibEdgeCount < 2) vecIsValidTag[startpoint.Number()] = false;
		++iterateNum;
	} while (neibEdgeCount < 2 && iterateNum < 100);
	
	contour.push_back(startpoint);

	curpoint = startpoint;
	prevpoint.Number() = -1;
	iterateNum = 0;
	Vector2D vec2d;
	double distance;
	do {
		edgeset = tinEdges[curpoint.Number()];
		bFoundNext = false;
		minAng = 4*PI;

		//search for next point
		for (int i=0; i<edgeset.size(); i++)
		{
			tempoint = edgeset[i];

			//skip invalid node and previous node
			if (vecIsValidTag[tempoint.Number()] == false
				||tempoint.Number() == prevpoint.Number())
				continue;

			vec2d.X() = lpLocalSeg[tempoint.Number()].X() - lpLocalSeg[curpoint.Number()].X();
			vec2d.Y() = lpLocalSeg[tempoint.Number()].Y() - lpLocalSeg[curpoint.Number()].Y();
			distance = vec2d.Length();
			if (distance > minEdgeLen) continue;//this line is too long, skip

			temAng = TINEdgeAngle(lpLocalSeg, curpoint, tempoint, plane, refDir);
			//first point of the contour
			if (prevpoint.Number() == -1) {
				if (temAng < minAng ) {
					bFoundNext = true;
					minAng = temAng;
					nextpoint = tempoint;
				}
			}
			else {//not first point of the contour
				double angleDiff = temAng - prevAng;
				if (angleDiff < 0)	angleDiff += 2*PI;
				if ( angleDiff < minAng) {
					bFoundNext = true;
					minAng = angleDiff;
					nextpoint = tempoint;
				}
			}
		}

		if (bFoundNext) {
			prevpoint = curpoint;
			curpoint = nextpoint;
			contour.push_back(curpoint);		  
			prevAng = TINEdgeAngle(lpLocalSeg, curpoint, prevpoint, plane, refDir);
		}
		else {
			//if cannot find next node, current node is invalid
			//go back to previous node and search for another neighbor node
			vecIsValidTag[curpoint.Number()] = false;
			contour.erase(contour.end()-1);
			
			if (contour.empty())
				bFoundNext = false;
			else if (contour.size() == 1) {
				curpoint = contour[contour.size()-1];
				prevpoint.Number() = -1;
				bFoundNext = true;
			}
			else  {
				curpoint = contour[contour.size()-1];
				prevpoint = contour[contour.size()-2];
				bFoundNext = true;
			}		
		}

		iterateNum++;
	} while (bFoundNext && nextpoint.Number() != startpoint.Number() && iterateNum < 2000);

	//transform to original number
	for (int i=0; i<contour.size(); i++)
		contour[i] = pnlSeg[contour[i].Number()];

	//for debug
/*	ObjectPoint objPnt;
	LineTopology lineTop;
	LineTopologies lineTops;
	ObjectPoints objPnts;
	LaserPoints lpTem;
	LaserPoint lp;
	for (int i=0; i<contour.size(); i++){
		lp = lpLocalSeg[contour[i].Number()];
		objPnt.X()=lp.X(); objPnt.Y()=lp.Y(); objPnt.Z()=lp.Z();
		objPnt.Number() = i;
		objPnts.push_back(objPnt);
		lineTop.push_back(objPnt.Number());
	}
	lineTops.push_back(lineTop);
	objPnts.Write("E:\\test_data\\zzzBoundaryPoints.objpts");
	lineTops.Write("E:\\test_data\\zzzBoundaryPoints.top");*/

	return true;
}

//project points onto the reference plane
//and then computer the angle with reference direction
double TINEdgeAngle(const LaserPoints& gLaserPoints, const PointNumber& startPntNum, 
	const PointNumber& endPntNum, const Plane& refPlane,const Vector3D& refDirection)
{
	double angle;
	Vector3D startPnt, endPnt;
	startPnt.X() = gLaserPoints[startPntNum.Number()].X();
	startPnt.Y() = gLaserPoints[startPntNum.Number()].Y();
	startPnt.Z() = gLaserPoints[startPntNum.Number()].Z();

	endPnt.X() = gLaserPoints[endPntNum.Number()].X();
	endPnt.Y() = gLaserPoints[endPntNum.Number()].Y();
	endPnt.Z() = gLaserPoints[endPntNum.Number()].Z();

	if (refDirection == Vector3D(0,1,0))
	{//project to X-Y plane
		endPnt.Z() = startPnt.Z(); 
		angle = Angle(endPnt-startPnt, refDirection);

		if (endPnt.X() < startPnt.X())
			angle = 2*PI - angle;
	}
	else 
	{
		if (fabs(refPlane.Normal().X()) > fabs(refPlane.Normal().Y()))
		{//project to Y-Z plane
			endPnt.X() = startPnt.X(); 
			angle = Angle(endPnt-startPnt, refDirection);

			if (endPnt.Y() < startPnt.Y())
				angle = 2*PI - angle;
		}
		else
		{//project to X-Z plane
			endPnt.Y() = startPnt.Y(); 
			angle = Angle(endPnt-startPnt, refDirection);

			if (endPnt.X() < startPnt.X())
				angle = 2*PI - angle;
		}
	}

	return angle;
}
