
/*
    Copyright 2010 University of Twente and Delft University of Technology
 
       This file is part of the Mapping libraries and tools, developed
  for research, education and projects in photogrammetry and laser scanning.

  The Mapping libraries and tools are free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
  published by the Free Software Foundation, either version 3 of the License,
                   or (at your option) any later version.

 The Mapping libraries and tools are distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
                GNU General Public License for more details.

      You should have received a copy of the GNU General Public License
          along with the Mapping libraries and tools.  If not, see
                      <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------*/

//
//
/*
 * ObjectSpaceLine.h
 *
 *  Created on: 28 Sep 2009
 *      Author: gerke, Tian and Nyaruhuma
 */

#ifndef OBJECTSPACELINE_H_
#define OBJECTSPACELINE_H_
#include "MathTools_TUB.h"

#include <cstdlib>
#include <iostream>
#include <map>
#include "Image.h"
#include "LineSegments2D.h"
#include "LineSegment3D.h"
#include "Database4Cpp.h"
#include "ImageLines.h"
#include "ImagePoints.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"
#include "DataBounds3D.h"
#include "DataBounds2D.h"
#include "Line3D.h"
#include <ObjectPoints.h>
#include <CameraPoint.h>
#include <CameraPoints.h>
#include <InteriorOrientation.h>
#include <ImageGrid.h>
#include <ObjectPoints2D.h>
#include <Plane.h>
#ifdef STANDALONE
#include "Tools.h"
#else
#include "ProjGeomTools.h"
#endif

///Convert Pmatrix into InteriorOrientation and Exteriororientation in Mapping library format
bool compute_int_ext_from_P(double **PMat, int h, int w, InteriorOrientation &inttmp, ExteriorOrientation &exttmp, double offset_XNull=0.0, double offset_YNull=0.0);
///Construct a 3D line in objectspace by plane intersection using Mapping Libary. Input: int and ext ori left (A) and right (B) and edges in A and B
bool Construct3DLine_maplib(InteriorOrientation &IOA, ExteriorOrientation &EOA, LineSegment2D &edgeA,InteriorOrientation &IOB, ExteriorOrientation &EOB, LineSegment2D &edgeB, Line3D &line);
///Construct a 3D line in objectspace by plane intersection using PMatrix and homogenous plane representation. Input: P left (A) and right (B) and edges in A and B
bool Construct3DLine_pmat(double **PA, LineSegment2D &edgeA, double **PB, LineSegment2D &edgeB, Line3D &line);
///help function for onstruct3DLine_pmat
bool Construct3DLine(double **PA, LineSegment2D &edgeA, double **PB, LineSegment2D &edgeB, vector<double> &L);
///help function for onstruct3DLine_pmat
void ConstructPlane(double P[12], LineSegment2D edge, vector<double> &pi);
///help function for onstruct3DLine_pmat
bool IntersectTwoPlanes(vector<double> &A, vector<double> &B, vector<double> &L);
///help function for onstruct3DLine_pmat
void PluckerToEuclidean(vector<double> &L, Line3D &line);
///Viewing Ray in object space defined by image point and camera proj center, PMatrix representation
bool ViewingRay(matrix *P, Position2D imagepoint, Line3D &ray);
///Viewing Ray in object space defined by image point and camera proj center, Mapping lib representation
bool ViewingRay(InteriorOrientation &IO, ExteriorOrientation &EO, Position2D imagepoint, Line3D &ray);
///Point3d to image
ImagePoint project3D2img(Position3D &pos, InteriorOrientation *Into, ExteriorOrientation *Exto);
///Linesegment 3D to image
LineSegment2D project3D2img(LineSegment3D &line, InteriorOrientation *Into, ExteriorOrientation *Exto);
///Helpfunction: compares to 2D lineesegments, thresholds: angle, distance, minimum overlap
bool check_LineSegment2D_identity(LineSegment2D &A,LineSegment2D &B, double parallel_thresh=5, double dist_thresh=3, double min_overlap=0.7); //5,3,0.7



class ObjectSpaceLine : public Line3D{

	friend class ObjectSpaceLines;
protected:
	int unique_instance_id_objectspacelines; //the unique id from the container class, this is an indicator for the image pairs from stereo intersection
	vector <unsigned int> ImageIdsV; //a vector containing the ids of participating images
	vector <double*> PMatV; //a vector of Projectmatrices (the sequence is the same as in the ImageIds, so a assignment image id-> P Mat is possible)
	vector < vector < double > > QMatV; //a vector for the LineProjectionMatrices. NEcessary for the adjustment
	vector <InteriorOrientation> IntOriV;
	vector <ExteriorOrientation> ExtOriV;
	vector <LineSegment2D> ImageLineV; //a vector of image edges per image (the sequence is the same as in the ImageIds, so a assignment image id-> P Mat is possible)
	vector <double> ImageLine_dresidualV; //a vector where per image edge the residual error is stored (distance image edge - reprojected line). It is filled during LSA
	vector <LineSegment3D> ObjectLineV;//The corresponding segments in object space. The Line3D also to be saved in this inherited class is of course the infinte line
	//!attention: in contrat to ObjetSpaceLines here only one segment per image is allowed. But: This does not mean that the same image can be listed several times.
	LineSegment3D ObjectLineCombined; //this line is composed from all instances in ObjectLineV: it connects the two outer points

	int imgheight,imgwidth;

	void computeQs();
	//bool check_linesegments3D_inb
public:
	ObjectSpaceLine();
	ObjectSpaceLine(const ObjectSpaceLine &osl); //copy constructor
	ObjectSpaceLine& operator=(const ObjectSpaceLine& osl);

	virtual ~ObjectSpaceLine();
	void dump();
	int No_of_unique_ImageIds();

	//if we have a valid 3D line, we can compute the 3D segments as projeced from the individual images onto this infinite line
	bool compute_corresponding_objectsegments(bool after_line_adjustment=0);
	bool compute_ObjectLineCombined();
	bool check_if_ObjectLineCombined_in_DataBounds(DataBounds3D bounds);

	//do the non-linear adustment: we have given an estimation fo the ObjectSpaceLine (ObjectLineCombined) and at least 2 views
	//code adapted from Yixiang Tian
	bool LSA_line();
private: //addition of data only through the friend class ObjectSpaceLines
	void setImageExtent(int h, int w)
			{
				this->imgheight=h;
				this->imgwidth=w;
			}

		void set_unique_instance_id_objectspacelines(int i){unique_instance_id_objectspacelines=i;}
		int get_unique_instance_id_objectspacelines(){return unique_instance_id_objectspacelines;}

	  //!Add some data
		void add_ImageInfo(unsigned int Id, double *PMat, LineSegment2D &ImageLines);
		void add_ImageInfo(unsigned int Id, double *PMat, ImageLines &iml, int lineid);
		void add_ObjectLineSegment(LineSegment3D seg) {this->ObjectLineV.push_back(seg);}//only to be used in ObjectSpaceLines::FindMultipleViews(int min_views);
		double compute_lineresidualRMSE();
};


#endif /* OBJECTSPACELINE_H_ */
