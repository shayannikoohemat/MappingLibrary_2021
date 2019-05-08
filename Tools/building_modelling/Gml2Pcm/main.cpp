
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
#include <string.h>
#include <iostream>
#include <fstream>
#include <time.h> 
#include <algorithm>
#include "citygml.h"
#include "LineTopology.h"
#include "ObjectPoint.h"
#include "LineTopologies.h"
#include "ObjectPoints.h"
#include "PointNumber.h"
#include "PointNumberList.h"
#include "TIN.h"
#include "InlineArguments.h"

using namespace citygml;
using namespace std;

void getRoofPolygon(const citygml::CityModel* city, ObjectPoints& objPnts, LineTopologies& lineTops) ;

void PrintUsage()
{
	printf("Usage: CityGml2PCM -i <citygml file> \n");
	printf("                   -ol <line topology file>\n");
	printf("                   -oo <point objects output file>\n");
}

int main(int argc, char *argv[])
{
    InlineArguments *args = new InlineArguments(argc, argv);

	if (args->Contains("-usage")) {
		PrintUsage();
		exit(0);
	}

	if (!args->Contains("-i") ) {
		printf("Error: -i is a required argument.\n");
		PrintUsage();
		exit(0);
	}
	if (!args->Contains("-ol") && !args->Contains("-oo")) {
		printf("Error: -i and -f are exclusive arguments.\n");
		PrintUsage();
		exit(0);
	}
	
	citygml::ParserParams params;
	char* filename = args->String("-i");
	char* strOutObjPntsPath = args->String("-oo");
	char* strOutTopLinesPath = args->String("-ol");
	citygml::CityModel *city = citygml::load(filename, params );
	if (!city)	{
		printf("Can read %s", filename);
		getchar();
		return -1;
	}

	ObjectPoints objPnts;
	LineTopologies lineTops;
	getRoofPolygon(city, objPnts, lineTops);
	objPnts.Write(strOutObjPntsPath);
	lineTops.Write(strOutTopLinesPath);

	delete city;


  return EXIT_SUCCESS;
}

void getRoofPolygon(const citygml::CityModel* city, ObjectPoints& objPnts, LineTopologies& lineTops) 
{
	if (!city) return;
	objPnts.clear();
	lineTops.clear();

	//CityObjectsMap objMap = city->getCityObjectsMap();
	const CityObjects* buildObjs = city->getCityObjectsByType(COT_Building);
	const CityObject* buildObj;
	const Geometry* roof;
	const Polygon* polygon;
	std::vector<TVec3d> vertices;
	LineTopology temLineTop;
	ObjectPoint temObjPnt;
	int nPntObjNum = 0;
	int nPolygonNum = 0;
	int startNum;
	int nBuilding = 0;
	int nBldPartNum;
	ClassTagValue classTag;
	GeometryType geotype; 

	for (unsigned int i=0; i<(unsigned int)buildObjs->size(); ++i)	{
		buildObj = (*buildObjs)[i];
		
		nBldPartNum = 0;
		for (unsigned int j=0; j<(unsigned int)buildObj->size(); ++j) {
			roof = buildObj->getGeometry(j);
			
			geotype = roof->getType();
			switch (roof->getType())
			{
			case GT_Roof: classTag = RoofClass;	break;
			case GT_Wall: classTag = WallClass; break;
			case GT_Ground: classTag = MapClass; break;
			default: classTag = UnknownClass; break;
			}

			//if (!(geotype==GT_Roof || geotype==GT_Wall)) 
			//	continue;
			if (geotype!=GT_Roof)
				continue;
		
			for (unsigned int k=0; k<roof->size(); ++k)	{
				polygon = (*roof)[k];
				vertices = polygon->getVertices();
				temLineTop.clear();
				startNum = nPntObjNum;
				for (unsigned int l=0; l<(unsigned int)vertices.size(); ++l) {
					temObjPnt.X() = vertices[l].x;
					temObjPnt.Y() = vertices[l].y;
					temObjPnt.Z() = vertices[l].z;
					temObjPnt.Number() = nPntObjNum;
					objPnts.push_back(temObjPnt);
					temLineTop.push_back(PointNumber(nPntObjNum));
					nPntObjNum++;
				}
				//enclose the polygon
				temLineTop.push_back(PointNumber(startNum));
				temLineTop.Attribute(LineLabelTag) = classTag;
				temLineTop.Attribute(BuildingNumberTag) = nBuilding;
				temLineTop.Attribute(BuildingPartNumberTag) = nBldPartNum;
				temLineTop.Number() = nPolygonNum;
				lineTops.push_back(temLineTop);
				++nPolygonNum;
				++nBldPartNum;
			}
		}//end for building parts
		++nBuilding;
	}//end for buildingobjs
}
