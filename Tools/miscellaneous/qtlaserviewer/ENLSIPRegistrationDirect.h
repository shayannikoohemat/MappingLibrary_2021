
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
*   File made : Feb 2005
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Simultaneour Registration and fitting of objects.
*
*--------------------------------------------------------------------*/
#ifndef _ENLSIP_REGISTRATION_DIRECT_ENLSIP_H_
#define _ENLSIP_REGISTRATION_DIRECT_ENLSIP_H_

#include "RegistrationStructs.h"
#include "LaserTransform3D.h"
//--------------------------------------------------------------
//Class for optimizing using only the parameters.
class ENLSIPRegistrationDirect: public ENLSIPFitting
{
public:	
	typedef std::map<int, LaserPoints*> PointsMap;
	typedef std::map<int, LaserTransform3D> TransformsMap;
	typedef vector<LaserObject*> ObjectsVector;
	//Constructor.
	ENLSIPRegistrationDirect::ENLSIPRegistrationDirect(std::map<int, LaserPoints*> _pointsMap,
		std::map<int, LaserTransform3D> _transformsMap,
		vector<LaserObject*> _objectsVector);

		
	///This function calculates the function values for all points.
	virtual bool EvaluateFunction(double* x,double* f,int count);
	
	///This function calculates residual value for the constraints.
	virtual bool EvaluateConstraint(double* x, double* f,int count);
	
	
	///Reimplement the optimize function.
	vector<double> Optimize(double* chiSquare=NULL);
	
	///Optimize with LargeScale method, no constraints,
	///At*A is formed internally. Space runs out otherwise for big problems.
	vector<double> OptimizeLargeScale(double* chiSquare=NULL);
		
	///Read the point using a 64-bit index
	//Also apply the transform.
	LaserPoint operator()(int64 index)
	{
		int scanId = index>>32;
		int pointId = index;
		
		if(pointsMap.find(scanId)!=pointsMap.end())
		{
			LaserPoints* pPts = pointsMap[scanId];
			
			//If there is a transform lets apply it else we can skip.
			if(transformsMap.find(scanId)!=transformsMap.end())
			{
				return transformsMap[scanId]*((*pPts)[pointId]);
			}
			else
				return (*pPts)[pointId];
		}
		else
		{
			cerr<<"Cannot find point with scanId: "<<scanId
				<<" pointId "<<pointId<<" "<<__FILE__<<": "<<__LINE__<<endl;
		}
	}
	
	///Return the count of params.
	int ParamCount()const
	{
		int count = 0;
		//First for the scans.
		count += transformsMap.size()*7;
		
		//Now for the objects.
		for(int i=0;i<objectsVector.size();i++)
			count += objectsVector[i]->ParamCount();
			
		return count;
	}
	
	///Return the number of observations.
	int ObsCount() const
	{
		//equals the number of points in objects indices vector.
		int count = 0;
		
		for(int i=0;i<objectsVector.size();i++)
			count += objectsVector[i]->GetIndices().size();
			
		return count;
	}
	
	///Save all values from a given pointer.
	template<class T>
	bool SaveToVector(T vals)
	{
		int count = 0;
	
		for(TransformsMap::iterator iter = transformsMap.begin();iter!=transformsMap.end();iter++)
		{
			iter->second.SaveToVector(vals+count);
			count += 7;
		}
	
		//Now for the object parameters.
		for(int i=0;i<objectsVector.size();i++)
		{
			vector<double> p = objectsVector[i]->ParamVector();
			copy(p.begin(),p.end(),vals+count);
			count += p.size();
		}
	}
	
	///Load all values from a given pointer.
	template<class T>
	void LoadFromVector(T vals)
	{
		int count = 0;
	
		for(TransformsMap::iterator iter = transformsMap.begin();iter!=transformsMap.end();iter++)
		{
			iter->second.LoadFromVector(vals+count);
			count += 7;
		}
	
		//Now for the object parameters.
		for(int i=0;i<objectsVector.size();i++)
		{
			int wanted = objectsVector[i]->ParamCount();
			vector<double> p(vals+count,vals+count+wanted);
			objectsVector[i]->FromVector(p);
			count += wanted;
		}
	}
	
	//Updates maps from object number to its parameter start
	//and from scan number to its parameter start.
	void GetParamMaps(std::map<int,int>& scanParamMap, std::map<int,int>& objParamMap)
	{
		int count = 0;
		
		scanParamMap.clear();
		for(TransformsMap::iterator iter = transformsMap.begin();iter!=transformsMap.end();iter++)
		{
			scanParamMap[iter->first] = count;
		}
	
		//Now for the object parameters.
		objParamMap.clear();
		for(int i=0;i<objectsVector.size();i++)
		{
			vector<double> p = objectsVector[i]->ParamVector();
			objParamMap[i] = count;
			
			count += p.size();
		}
	}
	//Makes atA matrix adding stabilizing term of given magnitude.
	//stepSize if for partial derivative calculation.
	class NEWMAT::SymmetricMatrix;
	class NEWMAT::Matrix;
	
	void MakeAtAMatrix(NEWMAT::SymmetricMatrix& AtA,
				NEWMAT::Matrix& Aty,
				double stepSize = 1e-6,
				double stabilizingTerm=1e-12);
				
	//Calculate StdDeviation of parameters.
	vector<double> CalculateStdDeviation();				
	
	///Calculates SumSquare.
	double SumSquare(const vector<double>& values);
	
	///Calculates SumSquare.
	double SumSquare();
	

	///A static function for executing the test.
	static void ExecuteTest();

protected: 
	//Maps a scan id to LaserPoints.
	std::map<int, LaserPoints*> pointsMap;
	
	//Maps a scan id to its transform.
	std::map<int, LaserTransform3D> transformsMap;
	
	//A vector of all objects, either the combined ones which are being used
	//for registration and fitting, or the others which are visible in one scan
	//and are being used only for fitting.
	vector<LaserObject*> objectsVector;
};

#endif //_ENLSIP_REGISTRATION_DIRECT_ENLSIP_H_
