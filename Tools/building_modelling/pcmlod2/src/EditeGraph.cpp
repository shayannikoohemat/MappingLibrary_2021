#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <vector>
#include <algorithm>
#include <stdio.h>
#include <QtWidgets/QMessageBox>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QMenu>
#include "PointNumberList.h"
#include "PointCloudMapper.h"
#include "InferenceEngine.h"
//#include "ModelParWin.h"
#include "GraphEditDictionary.h"
#include "ModelBldWithOccMap.h"
//#include <iostream.h>

using namespace std;

/// add actions to edit building graph tool
void PointCloudMapper::CreateEditGraphToolBar()
{
	QToolBar* tbarEditGraph = new QToolBar("Edit building graph", this);
	tbarEditGraph->setObjectName("Edit graph tools");
	tbarEditGraph->setIconSize(QSize(18,18));
	this->addToolBar(Qt::TopToolBarArea, tbarEditGraph);
	QMenu *model_menu  = menuBar()->addMenu(tr("&Building"));

	QAction* action;
	//intersect two laser segments tool
	action = new QAction(QIcon(":/buttons/intersectsegs.xpm"), 
		"Intersect two segments", this);
	action->setStatusTip("Intersect two segments");
	action->setShortcut(Qt::Key_I);
	QAction::connect(action, SIGNAL(triggered()), this, SLOT(ToggelIntersectSegments()));
	tbarEditGraph->addAction(action);
	model_menu->addAction(action);

	//delete one intersection tool
	action = new QAction(QIcon(":/buttons/deleteintersection.xpm"),
		"Delete intersection", this);
	action->setStatusTip("Delete selected intersection");
	action->setShortcut(Qt::Key_W);
	QAction::connect(action, SIGNAL(triggered()), this, SLOT(ToggelDeleteIntersect()));
	tbarEditGraph->addAction(action);
	model_menu->addAction(action);

	//merge points patches
	action = new QAction(QIcon(":/buttons/MergeSegments.xpm"), 
		"Merge two segments",  this);
	action->setStatusTip("Merge two segments");
	action->setShortcut(Qt::Key_U);
	QAction::connect(action, SIGNAL(triggered()), this, SLOT(ToggelMergeSegments()));
	tbarEditGraph->addAction(action);
	model_menu->addAction(action);


	//estimate one missing roof plane from 3 support planes
	action = new QAction(QIcon(":/buttons/estimateplane.xpm"),
		"Estimate a plane", this);
	action->setStatusTip("Estimate a plane");
	QAction::connect(action, SIGNAL(triggered()), this, SLOT(ToggelEstimatePlane()));
	tbarEditGraph->addAction(action);
	model_menu->addAction(action);
	//use foot print to reconstruct building
	//
	tbarEditGraph->addSeparator();

	//tbarEditGraph->addWidget()
	//reconstruct building model
	action = new QAction(QIcon(":/buttons/ConstructBld.xpm"), "Reconstruction", this);
	action->setStatusTip("Reconstruct model from points");
	action->setShortcut(Qt::Key_Y);
	QAction::connect(action, SIGNAL(triggered()), this, SLOT(ToggelReconstrcBldModel()));
	tbarEditGraph->addAction(action);
	model_menu->addAction(action);

	//scatter building model into ridges
	action = new QAction(QIcon(":/buttons/ScatterBld.xpm"), "Scatter building models", this);
	action->setStatusTip("Scatter building models");
	action->setShortcut(Qt::Key_J);
	QAction::connect(action, SIGNAL(triggered()), this, SLOT(ToggelScatterBldModel()));
	tbarEditGraph->addAction(action);
	model_menu->addAction(action);

	//make building model from ridge lines
	action = new QAction(QIcon(":/buttons/showmodels.xpm"), "Construct PCM Buildings", this);
	action->setStatusTip("Construct PCM Buildings");
	QAction::connect(action, SIGNAL(triggered()), this, SLOT(ToggelConstructPCMModel()));
	tbarEditGraph->addAction(action);
	model_menu->addAction(action);
}


void PointCloudMapper::ToggelIntersectSegments()
{
	//QMessageBox::information(this, "Error", "No laser segment is selected!");

	if (selected_laser_points.empty()) {
		QMessageBox::information(this, 
			"Error", "No laser segment is selected! Please Select Laser Segment.");
		return;
	}

	Plane plane1, plane2;
	vector<PointNumberList> vecSegPnls;
	MyDeriveSegPNL(selected_laser_points, vecSegPnls, SegmentNumberTag);
	if (vecSegPnls.size() != 2) {
		QMessageBox::information(this, "Error", 
			"laser segments is not equal to two! Please Select Laser Segment.");
		return;
	}

	int lSegNum = selected_laser_points[vecSegPnls[0][0].Number()].Attribute(SegmentNumberTag);
	int rSegNum = selected_laser_points[vecSegPnls[1][0].Number()].Attribute(SegmentNumberTag);
	Line3D intersect;
	plane1 = selected_laser_points.FitPlane(vecSegPnls[0]);
	plane2 = selected_laser_points.FitPlane(vecSegPnls[1]);
	Position3D startPos, endPos;
	selected_laser_points.IntersectFaces(vecSegPnls[0], vecSegPnls[1],
		plane1, plane2, 1.0, startPos, endPos);

	//search for most possible reconstructed building
	Buildings::iterator curBld;
	int indBld1 = IndPcmBldBySegNum(buildings, lSegNum);
	int indBld2 = IndPcmBldBySegNum(buildings, rSegNum);
	if (indBld1 != -1)
		curBld = buildings.begin() + indBld1;
	else if(indBld2 != -1)
		curBld = buildings.begin() + indBld2;
	else {
		buildings.push_back(Building(buildings.NextNumber(),&map_points));
		curBld =  buildings.end()-1;
	}

	int indLine;
	LineTopologies::iterator new_selection;
	LineTopologies* lineTops = curBld->MapDataPtr();
	if (lineTops && SearchLineBy2FacesPcm(*lineTops, lSegNum, rSegNum, indLine)) {
		//refine old intersection
		map_points[(*lineTops)[indLine][0].Number()].Position3DRef() = startPos;
		map_points[(*lineTops)[indLine][1].Number()].Position3DRef() = endPos;
		new_selection = lineTops->begin()+indLine;
	}
	else {//add new intersection
		AddLine2PcmBld(*curBld, map_points, startPos, endPos, lSegNum, rSegNum);
		new_selection = curBld->MapDataPtr()->end()-1;
	}

	selected_map_data.clear();
	selected_map_data.push_back(new_selection);


	ShowData(PCMWindowPtr(), MapData, true, SelectedMapData, true, true, true);
	Canvas()->update();
}

void PointCloudMapper::ToggelDeleteIntersect()
{
	LineTopsIterVector temMapData = selected_map_data;
	selected_map_data.clear();
	Building* building;
	LineTopologies::iterator curSelectLine;
	LineTopologies one_line_set;
	int BldNum, temBldNum;
	bool bSameBld = true;
	LineTopologies* pMapTops = NULL;
	LineTopologies::iterator temLine;
	vector<char> vecIsValid;

	for (int iTop=0; iTop<temMapData.size(); iTop++) {
		curSelectLine = temMapData[iTop];
		if(!curSelectLine->HasAttribute(BuildingNumberTag)) {
			bSameBld = false; 
			break;
		}
		temBldNum = curSelectLine->Attribute(BuildingNumberTag);
		if (iTop==0)
			BldNum = temBldNum;
		else if (temBldNum != BldNum) {
			bSameBld = false;
			break;
		}
	}

	if (!bSameBld) {
		QMessageBox::information(this, "Error", "Intersection lines are not from same building");
		goto endFlag;//stop
	}
	
	building = buildings.BuildingPtr(BldNum);
	if(building == NULL) {
		goto endFlag;//stop
	}

	pMapTops = building->MapDataPtr();
	if (pMapTops == NULL) goto endFlag;

	vecIsValid = vector<char>(pMapTops->size(), true);

	//search the lines to be deleted
	for (int iTop=0; iTop<temMapData.size(); iTop++) {
		curSelectLine = temMapData[iTop];

		for (int jTop=0; jTop<pMapTops->size(); ++jTop) {
			temLine = pMapTops->begin() + jTop;
			if (curSelectLine == temLine ) {
				vecIsValid[jTop] = false;
				break;
			}
		}
	}

	//delete lines
	for (int i=0; i<vecIsValid.size(); ++i) {
		if (vecIsValid[i]) continue;

		pMapTops->erase(pMapTops->begin()+i);
		vecIsValid.erase(vecIsValid.begin()+i);
		i--;
	}

endFlag:
	if (!buildings.empty())
		ShowData(PCMWindowPtr(), MapData, true, SelectedMapData, true, true, true);
	else
		ShowData(PCMWindowPtr(), MapData, false, SelectedMapData, false, true, true);
	Canvas()->update();
}

void PointCloudMapper::ToggelMergeSegments()
{
	//selected_laser_points;
	if (selected_laser_points.empty()) {
		QMessageBox::information(this, "Warning","No point segment is selected!");
		return;
	}

	vector<PointNumberList> vecPnlSegs;
	MyDeriveSegPNL(selected_laser_points, vecPnlSegs, SegmentNumberTag);
	if (vecPnlSegs.size() != 2) {
		QMessageBox::information(this, "Warning",
		"Number of Selected point segments is not two!");
		return;
	}

	//segNum2 is set as segNum1
	LaserPoints new_selected_laspnt;
	int segNum1 = selected_laser_points[vecPnlSegs[0][0].Number()].Attribute(SegmentNumberTag);
	int segNum2 = selected_laser_points[vecPnlSegs[1][0].Number()].Attribute(SegmentNumberTag);
	if (segNum1>segNum2) {//make sure segNum1 is smaller then segNum2
		std::swap(segNum1, segNum2);
		std::swap(vecPnlSegs[0], vecPnlSegs[1]);
	}
	selected_laser_points.ReTag(segNum2, segNum1, SegmentNumberTag);
	laser_points.ReTag(segNum2, segNum1, SegmentNumberTag);
	MyDeriveSegPNL(laser_points, vecPnlSegs, SegmentNumberTag);

	//refine nearby segment attributes of building topology graph
	Buildings::iterator curBld;
	LineTopologies* curMapTops;
	//try to find the most possible building from segment number
	int indBld = IndPcmBldBySegNum(buildings, segNum1);
	if (indBld==-1) indBld = IndPcmBldBySegNum(buildings, segNum2);
	LineTopologies::iterator itrLine;
	int lRoofNum, rRoofNum;
	LineTopology temLineTop;
	int temLineInd;
	bool bRedundantLine;
	Plane lPlane, rPlane;
	Position3D startPnt, endPnt;
	int indSeg1, indSeg2;

	if (indBld != -1) {
		curBld = buildings.begin()+indBld;
		curMapTops = curBld->MapDataPtr();

		if (curMapTops != NULL) {
			for (int iLine=0; iLine<curMapTops->size(); iLine++) {
				itrLine = curMapTops->begin()+iLine;
				lRoofNum = itrLine->Attribute(SegmentLabel);
				rRoofNum = itrLine->Attribute(SecondSegmentLabel);
				
				//only process lines one segment2
				if (lRoofNum!=segNum2 && rRoofNum!=segNum2)
					continue;
				//make sure rRoofNum is equal to segNum2
				if (segNum2 == lRoofNum) std::swap(lRoofNum, rRoofNum);
				bRedundantLine = false;

				if (segNum1==lRoofNum) {
					//this intersection is formed by the two merging segments,
					curMapTops->erase(itrLine);
					iLine--;
				}
				else if(SearchLineBy2FacesPcm(*curMapTops, lRoofNum, segNum1, temLineInd)) {
					//revise current intersection
					indSeg1 = IndexSegBySegNumber(laser_points, vecPnlSegs, segNum1, SegmentNumberTag);
					indSeg2 = IndexSegBySegNumber(laser_points, vecPnlSegs, lRoofNum, SegmentNumberTag);
					lPlane = laser_points.FitPlane(vecPnlSegs[indSeg1]);
					rPlane = laser_points.FitPlane(vecPnlSegs[indSeg2]);
					laser_points.IntersectFaces(vecPnlSegs[indSeg1], vecPnlSegs[indSeg2], 
						lPlane, rPlane, 0.5, startPnt, endPnt);
					map_points[(*itrLine)[0].Number()].Position3DRef() = startPnt;
					map_points[(*itrLine)[1].Number()].Position3DRef() = endPnt;
					itrLine->SetAttribute(SegmentLabel, lRoofNum);
					itrLine->SetAttribute(SecondSegmentLabel, segNum1);

					//remove the redundant intersection
					curMapTops->erase(curMapTops->begin()+temLineInd);
					--iLine;//go one step back
				}
				else {//re-tag intersection
					itrLine->SetAttribute(SegmentLabel, lRoofNum);
					itrLine->SetAttribute(SecondSegmentLabel, segNum1);
				}
			}

			//////////////////////////////////////////////////////////////////////////
			//refit all other lines
			vector<int> vecBoundTopNum;
		//	SearchLinesOnPlanePcm(*curMapTops, segNum1, vecBoundTopNum);
			vecBoundTopNum.clear();
			int temSegNum1, temSegNum2;
			for (int i=0; i<vecBoundTopNum.size(); ++i) {

			}
		}
	}
	
	selected_map_data.clear();
	if (!buildings.empty())
		ShowData(PCMWindowPtr(), MapData, true, SelectedMapData, false, true, true);
	else
		ShowData(PCMWindowPtr(), MapData, false, SelectedMapData, false, true, true);
	Canvas()->update();
}

/// assume a plane between two parallel segments to connect the two segments.
void MyHypesisizeOneFace(const LaserPoints& lasPnts, const PointNumberList& pnlSeg1, const PointNumberList& pnlSeg2)
{
	Plane plane1, plane2, plane3;
	plane1 = lasPnts.FitPlane(pnlSeg1);
	plane2 = lasPnts.FitPlane(pnlSeg2);

	double ang = Angle(plane1.Normal(), plane2.Normal());
	ang = ang*180/3.14159;
	if(ang>90.0) ang = 180.0-ang;

	if (ang > 15.0) return;//the two segments are not parallel

	//project all points to plane1
	LaserPoints temLasPnts;
	LaserPoint pnt;
	Position3D pos;
	for (int i=0; i<pnlSeg1.size(); i++) {
		pos = plane1.Project(lasPnts[pnlSeg1[i].Number()].Position3DRef());
		pnt.Position3DRef() = pos;
		pnt.SetAttribute(SegmentNumberTag, 0);
		temLasPnts.push_back(pnt);
	}

	for (int i=0; i<pnlSeg2.size(); i++) {
		pos = plane1.Project(lasPnts[pnlSeg2[i].Number()].Position3DRef());
		pnt.Position3DRef() = pos;
		pnt.SetAttribute(SegmentNumberTag, 1);
		temLasPnts.push_back(pnt);
	}

	//get all points on the boundaries, 
	//using mixed tag for identify boundary points
	temLasPnts.DeriveTIN();
	TINEdges edges;
	edges.Derive(temLasPnts.TINReference()); 
	temLasPnts.RemoveLongEdges(edges, 1.0, false);
	LaserPoints boundPnts;
//	temLasPnts.CountMixedEdges();
	LaserPoints boundPnt = temLasPnts.ReturnMixedEdges(edges, SegmentNumberTag);
}

void PointCloudMapper::ToggelEstimatePlane()
{
	//QMessageBox::information(this, "Error", "Estimate a plane");
	if (selected_laser_points.empty()) {
		QMessageBox::information(this, "Warning","No point segment is selected!");
		return;
	}

	vector<PointNumberList> vecPnlSegs;
	MyDeriveSegPNL(selected_laser_points, vecPnlSegs, SegmentNumberTag);
	if (vecPnlSegs.size() != 3)	{
		QMessageBox::information(this, "Error", "Wrong point segment number.");
		return;
	}

	Plane plane1, plane2, plane3, estiPlane;
	plane1 = selected_laser_points.FitPlane(vecPnlSegs[0]);
	plane2 = selected_laser_points.FitPlane(vecPnlSegs[1]);
	plane3 = selected_laser_points.FitPlane(vecPnlSegs[2]);

	double aveDist;
	Vector3D estNormal;
	
	double ang1 = Angle(plane1.Normal(), plane3.Normal());
	double ang2 = Angle(plane2.Normal(), plane3.Normal());
	double ang3 = Angle(plane1.Normal(), plane2.Normal());
	ang1 = ang1*180/3.14159;
	ang2 = ang2*180/3.14159;
	ang3 = ang3*180/3.14159;
	if (ang1>90.0) ang1 = 180.0 - ang1;
	if (ang2>90.0) ang2 = 180.0 - ang2;
	if (ang3>90.0) ang3 = 180.0 - ang3;


	bool bFoundFlag = false;
	if (fabs(ang1-ang2) < 10.0 ) {//plane3 is the third plane
		//keep everything
		bFoundFlag = true;
	}
	else if (fabs(ang1-ang3) < 10.0) {//plane1 is the third plane
		std::swap(ang2, ang3);
		std::swap(vecPnlSegs[0], vecPnlSegs[2]);
		std::swap(plane1, plane3);
		bFoundFlag = true;
	}
	else if (fabs(ang2-ang3) < 10.0) 	{//plane 2 is the third plane
		std::swap(ang1, ang3);
		std::swap(vecPnlSegs[1], vecPnlSegs[2]);
		std::swap(plane2, plane3);
		bFoundFlag = true;
	}

	if(!bFoundFlag) {
		QMessageBox::information(this, "Error", "No suitable plane is estimated");
		return;
	}

	//the estimated plane should be opposite to the third plane
	estNormal = plane3.Normal();
	estNormal.X() *= -1.0;
	estNormal.Y() *= -1.0;
		
	//plane3.SetNormal(estNormal);
	//plane3.SetDistance(aveDist);

	LaserPoints temLasPnts;
	for (int i=0; i<vecPnlSegs[0].size(); i++)
		temLasPnts.push_back(selected_laser_points[vecPnlSegs[0][i].Number()]);
	for (int i=0; i<vecPnlSegs[1].size(); i++) 
		temLasPnts.push_back(selected_laser_points[vecPnlSegs[1][i].Number()]);

	temLasPnts.DeriveTIN();
	TINEdges edges;
	edges.Derive(temLasPnts.TINReference()); 
	//temLasPnts.RemoveLongEdges(edges, 1.0, false);
	//	temLasPnts.CountMixedEdges();
	LaserPoints boundPnts = temLasPnts.ReturnMixedEdges(edges, SegmentNumberTag);
	if(boundPnts.size()<3) return;
	//boundPnts.Write("c:\\boundpnt.laser");

	vector<double> vecDist;
	Vector3D pos;
	for (int i=0; i<boundPnts.size(); ++i) {
		pos = boundPnts[i].Position3DRef();
		vecDist.push_back(pos.DotProduct(estNormal));
	}
	std::sort(vecDist.begin(), vecDist.end());
	double minDis = vecDist[0];
	double maxDis = vecDist[vecDist.size()-1];
	double rang = (maxDis-minDis)/(boundPnts.size()/4);

	//construct distance histogram
	int ind = 0;
	vector<int> vecHist(boundPnts.size()/4+1, 0);
	for (int i=0; i<vecDist.size(); ++i) {
		ind = (vecDist[i]-minDis)/rang;
		vecHist[ind]++;
	}

	int maxVote= -1;
	int indWin;
	for (int i=0; i<vecHist.size(); ++i) {
		if (vecHist[i] > maxVote) {
			indWin = i;
			maxVote = vecHist[i];
		}
	}

	estiPlane.SetNormal(estNormal);
	estiPlane.SetDistance(minDis+(indWin+0.5)*rang);

	//refit the plane
	PointNumberList pnlSeg4;
	LaserPoints newSelectPnts;
	LaserPoints::iterator point;
	for (int iLoops=0; iLoops<2; iLoops++) {
		pnlSeg4.clear();
		newSelectPnts.clear();
		for (int i=0; i<vecPnlSegs[0].size(); ++i) {
			point = selected_laser_points.begin()+vecPnlSegs[0][i].Number();
			if (fabs(estiPlane.Distance(point->Position3DRef())) < 0.3) {
				pnlSeg4.push_back(vecPnlSegs[0][i]);
				newSelectPnts.push_back(*point);
			}
		}
		for (int i=0; i<vecPnlSegs[1].size(); ++i) {
			point = selected_laser_points.begin()+vecPnlSegs[1][i].Number();
			if (fabs(estiPlane.Distance(point->Position3DRef())) < 0.3) {
				pnlSeg4.push_back(vecPnlSegs[1][i]);
				newSelectPnts.push_back(*point);
			}
		}
		//newSelectPnts.Write("c:\\selected_pnts.laser");
		estiPlane = selected_laser_points.FitPlane(pnlSeg4, estNormal, 0);
	}

	LaserPoints::iterator itrPnt;
	int maxSegNum = -9999;
	int temSegNum;
	for (itrPnt = laser_points.begin(); itrPnt != laser_points.end(); ++itrPnt) {
		temSegNum = itrPnt->Attribute(SegmentNumberTag);
		if (temSegNum > maxSegNum) 	
			maxSegNum = temSegNum;	
	}

	maxSegNum++;
	for (itrPnt = laser_points.begin(); itrPnt != laser_points.end(); ++itrPnt) {
		for (int i=0; i<pnlSeg4.size(); i++)	{
			if(selected_laser_points[pnlSeg4[i].Number()].Position3DRef() 
				==  itrPnt->Position3DRef()) {
				itrPnt->SetAttribute(SegmentNumberTag, maxSegNum);
				break;
			}
		}
	}

	//add intersection line 
	Position3D startPos, endPos;
	//int lPlaneNum, rPlaneNum;
	ObjectPoint objPnts;
	int objPntNum;
	LineTopology lineTop;
	LineTopologies::iterator  new_selection; 
	selected_map_data.clear();
	int segNum1 = selected_laser_points[vecPnlSegs[0][0].Number()].Attribute(SegmentNumberTag);
	int segNum2 = selected_laser_points[vecPnlSegs[1][0].Number()].Attribute(SegmentNumberTag);
	int segNum3 = selected_laser_points[vecPnlSegs[2][0].Number()].Attribute(SegmentNumberTag);

	int indBld = IndPcmBldBySegNum(buildings, segNum1);
	//Building curBld(buildings.NextNumber(), &map_points);
	Buildings::iterator curBld;
	if (indBld == -1) return;
	curBld = buildings.begin()+indBld;


	selected_laser_points.IntersectFaces(pnlSeg4, vecPnlSegs[0], 
		estiPlane, plane1, 0.5, startPos, endPos);
	AddLine2PcmBld(*curBld, map_points, startPos, endPos, maxSegNum, segNum1);

	//intersect 2
	selected_laser_points.IntersectFaces(pnlSeg4, vecPnlSegs[1], 
		estiPlane, plane2, 0.5, startPos, endPos);
	AddLine2PcmBld(*curBld, map_points, startPos, endPos, maxSegNum, segNum2);

	//intersect 3, if it exits
	selected_laser_points.DeriveTIN();
	edges.clear();
	edges.Derive(selected_laser_points.TINReference());
	bool bFlagForthPlane = false;
	if(IsAdjacent(selected_laser_points, edges, pnlSeg4, vecPnlSegs[2], 0.3, 3)) {
		selected_laser_points.IntersectFaces(pnlSeg4, vecPnlSegs[2], 
			estiPlane, plane3, 0.5, startPos, endPos);
		bFlagForthPlane = true;
		AddLine2PcmBld(*curBld, map_points, startPos, endPos, maxSegNum, segNum3);
	}

	//buildings.push_back(curBld);
	if(bFlagForthPlane) {
		new_selection = curBld->MapDataPtr()->end()-1;
		selected_map_data.push_back(new_selection);
	}
	new_selection = curBld->MapDataPtr()->end()-2;
	selected_map_data.push_back(new_selection);
	new_selection = curBld->MapDataPtr()->end()-3;
	selected_map_data.push_back(new_selection);

	ShowData(PCMWindowPtr(), MapData, true, SelectedMapData, true, true, true);
	selected_laser_points.clear();
	//selected_laser_points.AddPoints(boundPnts);
	selected_laser_points.AddPoints(newSelectPnts);
	Canvas()->update();
}


void PointCloudMapper::ToggelReconstrcBldModel()
{
	if (selected_laser_points.empty() && laser_points.empty()) {
		QMessageBox::information(this, "Warning","No point is available for reconstruction!");
		return;
	}
	
	vector<Plane> vecPlanes;

	if (!selected_laser_points.empty()) {
		vector<PointNumberList> vecSegPnls;
		MyDeriveSegPNL(selected_laser_points, vecSegPnls, SegmentNumberTag);
		int segNum, bldInd;
		segNum = selected_laser_points[vecSegPnls[0][0].Number()].Attribute(SegmentNumberTag);
		bldInd = IndPcmBldBySegNum(buildings, segNum);
		if (bldInd == -1) {
			buildings.push_back(Building(buildings.NextNumber(), &map_points));
			bldInd = buildings.size()-1;
			LineTopology temLineTop;
			ObjectPoint temObjPnt;
			AddLine2PcmBld(buildings[bldInd], map_points,temObjPnt,temObjPnt,segNum,-1,0);
			//ReconstructBldWithRidges(selected_laser_points, buildings[bldInd], map_points);
			ReconstructBldWithRidges(laser_points, buildings[bldInd], map_points);
		}
		else {
			ReconstructBldWithRidges(laser_points, buildings[bldInd], map_points);
		}
	}
	else {
		ReconstructBuilding(laser_points, vecPlanes, buildings, map_points);
	}
	
	selected_laser_points.clear();
	selected_map_data.clear();
	if (!buildings.empty())
		ShowData(PCMWindowPtr(), MapData, true, SelectedMapData, false, true, true);
	else
		ShowData(PCMWindowPtr(), MapData, false, SelectedMapData, false, true, true);
	Canvas()->update();
}


void PointCloudMapper::ToggelScatterBldModel()
{
	if (selected_laser_points.empty()) {
		QMessageBox::information(this, "Warning","No point segment is selected!");
		return;
	}

	vector<PointNumberList> vecPnlSegs;
	MyDeriveSegPNL(selected_laser_points, vecPnlSegs, SegmentNumberTag);
	int segNum, bldNum;
	segNum = selected_laser_points[vecPnlSegs[0][0].Number()].Attribute(SegmentNumberTag);
	bldNum = IndPcmBldBySegNum(buildings, segNum);
	if (bldNum == -1) {
		QMessageBox::information(this, "Warning","No Building is available!");
		return;
	}

	ScatterBuilding(laser_points, buildings[bldNum], map_points);
	if (buildings[bldNum].MapDataPtr() == NULL
		|| buildings[bldNum].MapDataPtr()->empty()) {
			buildings.erase(buildings.begin()+bldNum);
	}
		
	selected_map_data.clear();
	ShowData(PCMWindowPtr(), MapData, true, SelectedMapData, false, true, true);
	Canvas()->update();
}

void ComputeResidential(LaserPoints& gLaserPnts, vector<Plane>& vecPlanes,
	 vector<PointNumberList>& vecPnlSegs, LineTopologies& roofPolygons, Buildings::iterator curBld )
{
	vector<int> vecSegNums(1000);
	SearchSegNumsInBld(*curBld, vecSegNums);
	ObjectPoints* pMapPnt = curBld->Points(MapData);
	if (!pMapPnt) return;
	
	int indSeg, indPlane;
	double distance;
	LaserPoints::iterator curPnt;
	bool bInsideBld;
	
	for (int iSeg=0; iSeg<vecSegNums.size(); ++iSeg) 	{
		indSeg = IndexSegBySegNumber(gLaserPnts, vecPnlSegs, vecSegNums[iSeg], SegmentNumberTag);
		indPlane = IndexPlaneBySegNumber(vecPlanes, vecSegNums[iSeg]);

		if (indSeg<0 || indPlane<0) continue;

		for (int iPnt=0; iPnt<vecPnlSegs[indSeg].size(); iPnt++) {
			curPnt = gLaserPnts.begin()+vecPnlSegs[indSeg][iPnt].Number();

			bInsideBld = false;
			for (int iPolygon=0; iPolygon<roofPolygons.size(); iPolygon++) {
				if(curPnt->InsidePolygon(*pMapPnt, roofPolygons[iPolygon])) {
					bInsideBld = true;
					break;
				}
			}
			if (!bInsideBld) continue;

			distance = vecPlanes[indPlane].Distance(*curPnt);
			curPnt->SetAttribute(ResidualTag, (float)distance);
		}
	}
}

//convert the roof polygon into PCM models
//measure the distance of points to roof planes.
void PointCloudMapper::ToggelConstructPCMModel()
{
	if(buildings.empty())
		return;

    ConstructPCMModel(buildings, map_points);
	model_points = map_points;

	ShowData(PCMWindowPtr(), ModelData, true, SelectedModelData, false, true, true);
	Canvas()->update();
}
