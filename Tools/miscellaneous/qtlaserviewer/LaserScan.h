
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


#ifndef _LASER_SCAN_H_
#define _LASER_SCAN_H_
/*--------------------------------------------------------------------
*   Project   : Automated reconstruction of industrial installations
*
*   File made : August 2004
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Holds one scan with associated information.
*
*--------------------------------------------------------------------*/
#include "Orientation3D.h"
#include <LaserPoints.h>
#include "LaserObjects.h"

/*****************************************************************************
 *
 * 		Class LaserScan
 *
 *****************************************************************************/

/*
Keeps data related to one LaserScan. Consists of
1: PointCloud
2: Registration information. Currently just exterior orientation. If NULL then no scan
	to world transformation has been done yet.
*/	
class LaserScan
{
public:
	///Default Constructor.
	LaserScan(LaserPoints* pL=NULL, Orientation3D* pEx=NULL,int __id=-1);
		
	///Use the arguments passed by reference.
	LaserScan(const LaserPoints& pts, const Orientation3D& ex, int __id=-1);
	
	///Create from a given fileName.
	LaserScan(const char* fileName);
	
	//Copy constructor. Just copy pointers.
	LaserScan(const LaserScan& ls);
	
	//Destructor. Don't free memory.
	~LaserScan();
	
	//Return size of points.
	unsigned int size() const;
	
	///Free the resources.
	void FreeResources();
	
	///Return a reference to internal pointer.
	const LaserPoints* GetLaserPoints() const;
	
	///Append the transformed points to the end of passed laserPoint vector.
	int GetLaserPoints(LaserPoints& pts) const;
	
	///Return transformed points.
	LaserPoints GetTransformedPoints()const;
	
	///Return a read only pointer to internal exterior orientation.
	const Orientation3D* GetOrientation() const;
	
	///Change the exterior orientation.
	void SetOrientation(Orientation3D ort);
			
	///Read only access to laserpoints. Use sparingly as it is very inefficient due to
	///internal checks.
	const LaserPoint& operator[](unsigned int i) const;
	
	///Returns the name.
	string GetName() const ;
	
	///Returns fileName.
	string GetFileName() const;
	
	///Sets a new name.
	string SetName(const string& s);
	
	///Returns id.
	int GetId() const
	{
		return id;
	};
	
	///Sets a new id.
	void SetId(int __id)
	{
		id = __id;
	}
	
	string& Scanner()
	{
		return scanner;
	}
	
	string& Name()
	{
		return name;
	}
	
	string& FileName()
	{
		return fileName;
	}
	
	int& Id()
	{
		return id;
	}
	
	ostream& SaveXML(ostream& os)const;
	
	///Prints information about the object on screen.
	void Print() const;
private:
	///Pointer to laser points
	LaserPoints* pLaserPoints;
	
	///Pointer to Orientation orientation.
	Orientation3D* pOrientation;	
	
	///Name of scan.
	string name;
	
	///Name of file.
	string fileName;
	
	///Name of the scanner.
	string scanner;
	
	///integer id of the scan.
	int id;
};

#endif //_LASER_SCAN_H_
