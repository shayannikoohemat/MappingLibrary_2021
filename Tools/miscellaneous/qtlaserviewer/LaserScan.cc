
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


/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : August 2004
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Data structure to hold global point cloud related data for current project
*
*--------------------------------------------------------------------*/
#include "LaserScan.h"


/*****************************************************************************
 *
 * 		Class LaserScan
 *
 *****************************************************************************/

///Default Constructor.
LaserScan::LaserScan(LaserPoints* pL, Orientation3D* pEx,int _id)
:pLaserPoints(pL),pOrientation(pEx),id(_id)
{

}

///Use the arguments passed by reference.
LaserScan::LaserScan(const LaserPoints& pts, const Orientation3D& ex,int _id)
:id(_id)
{
	pLaserPoints = new LaserPoints(pts);
	pOrientation = new Orientation3D(ex);
}

///Create from a given fileName.
LaserScan::LaserScan(const char* fileName)
{
	FILE* pFile = fopen(fileName,"rb");
	if(pFile)
	{
		fclose(pFile);
		pLaserPoints = new LaserPoints;
		pLaserPoints->SetPointFile((char*)fileName);
		pLaserPoints->Read();
		this->fileName = fileName;
	}
	else
	{
		cerr<<"ERROR: cannot read "<<fileName<<" for creating LaserRecording\n";
		pLaserPoints = NULL;
	}
	pOrientation = NULL;
	id = -1;
}

//Copy constructor. Just copy pointers.
LaserScan::LaserScan(const LaserScan& ls)
{
	pLaserPoints = ls.pLaserPoints;
	pOrientation = ls.pOrientation;
	name = ls.name;
	fileName = ls.fileName;
	id = ls.id;
}

//Destructor. Don't free memory.
LaserScan::~LaserScan()
{
}

//Return size of points.
unsigned int 
LaserScan::size() const
{
	if(!pLaserPoints)
		return 0;
	return pLaserPoints->size();
}

///Free the resources.
void 
LaserScan::FreeResources()
{
	delete pLaserPoints;
	delete pOrientation;
	pLaserPoints = NULL;
	pOrientation = NULL;
}

///Return a reference to internal pointer.
const LaserPoints*
LaserScan:: GetLaserPoints() const
{
	return pLaserPoints;
}

///Append the transformed points to the end of passed laserPoint vector.
int 
LaserScan::GetLaserPoints(LaserPoints& pts) const
{
	if(pLaserPoints)
	{
		//TODO: Apply the transformation here.
		pts.reserve(pts.size()+pLaserPoints->size());
		for(int i=0;i<pLaserPoints->size();i++)
			pts.push_back((*pLaserPoints)[i]);
		return pLaserPoints->size();
	}
	else
		return 0;
}

///Return a read only pointer to internal exterior orientation.
const Orientation3D* 
LaserScan:: GetOrientation() const
{
	return pOrientation;
}

///Read only access to laserpoints. Use sparingly as it is very inefficient due to
///internal checks.
const LaserPoint& 
LaserScan::operator[](unsigned int i) const
{
	if(!pLaserPoints)
	{
		cerr<<"LaserPoint& operator[]: pLaserPoints is NULL\n";
		return LaserPoint();
	}
	if(i<0 || i>pLaserPoints->size())
	{
		cerr<<"LaserPoint& operator[]: index "<<i<<" is invalid\n";
		return LaserPoint();
	}

	//TODO:Check for pOrientation and apply if it is there.
	if(!pOrientation)
		return (*pLaserPoints)[i];
	else
	{
		cerr<<"TODO:Check for pOrientation and apply if it is there.\n";
		return (*pLaserPoints)[i];
	}
}

///Returns the name.
string 
LaserScan::GetName() const
{
	return name;
}

///Returns fileName.
string 
LaserScan::GetFileName() const
{
	return fileName;
}

///Sets a new name.
string 
LaserScan::SetName(const string& s)
{
	name = s;
}

///Prints some information about the object on screen.
void
LaserScan::Print() const
{
	fprintf(stderr,"LaserScan: %p\n",this);
	fprintf(stderr,"name: %s fileName: %s pLaserPoints: %p pOrientationOrientaion: %p\n",name.c_str(),fileName.c_str(),pLaserPoints,pOrientation);
	if(!pLaserPoints)
	{
		fprintf(stderr,"size(pLaserPoints): %d\n",pLaserPoints->size());
	}
}

void LaserScan::SetOrientation(Orientation3D ort)
{
		if(pOrientation)
		{
			(*pOrientation) = ort;
		}
}

///Return transformed points.
LaserPoints LaserScan::GetTransformedPoints()const
{
	LaserPoints pts;
	if(pLaserPoints && pOrientation)
	{
		pts = (*pLaserPoints)*(*pOrientation);
	}
	return pts;
}

ostream& LaserScan::SaveXML(ostream& os)const
{
	os<<"<scan>\n";
	os<<"<scanner>"<<scanner<<"</scanner>\n";
	os<<"<id>"<<id<<"</id>\n";
	os<<"<filename>"<<fileName<<"</filename>\n";
	///os<<"<units>"mm </units>
	os<<"<exterior>\n";
	
	QuaternionRotation quat(*pOrientation);
	Vector3D trans = *pOrientation;
	os<<"\t<rotation type=\"quaternion\">"<<quat[0]<<" "
		<<quat[1]<<" "<<quat[2]<<"  "<<quat[3]<<"</rotation>\n";
	os<<"\t<translation>"<<trans[0]<<" "<<trans[1]<<"  "<<trans[2]<<"</translation>\n";
	os<<"</exterior>\n";
	os<<"<name>"<<name<<"</name>\n";
	os<<"</scan>\n";
}
