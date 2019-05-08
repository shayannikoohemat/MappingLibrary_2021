
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
#include "GeometricFittingENLSIP.h"
#include "TextFileUtilities.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "Vector3D.h"
#include "RotationParameter.h"
#include "LaserPoints.h"
#include "LaserPointsUtility.h"
#include "LaserPointsProcessing.h"
#include "ENLSIPRegistrationDirect.h"
#include "LaserPointsProcessing.h"
#include "GeneralUtility.h"
#include "StlUtilities.h"
#include "LaserObjects.h"


#define SHOW_DEBUG 1
#define SH cerr<<__LINE__<<": "<<__FILE__<<flush<<endl;

//Constructor.
ENLSIPRegistrationDirect::ENLSIPRegistrationDirect(std::map<int, LaserPoints*> _pointsMap,
		std::map<int, LaserTransform3D> _transformsMap,vector<LaserObject*> _objectsVector)
:ENLSIPFitting(7,8,3,1),pointsMap(_pointsMap),transformsMap(_transformsMap),
objectsVector(_objectsVector)
{
	functionCounter = constraintCounter = 0;
	obsCount = ObsCount();;
		
	//Add seven parameters per scan.
	paramCount= ParamCount();;
}

///This function calculates the function values for all points.
bool ENLSIPRegistrationDirect::EvaluateFunction(double* x,double* f,int count)
{
	if(SHOW_DEBUG)
		cerr<<"Calling ENLSIPRegistrationDirect::EvaluateFunction \n";
#if 1
	
	//Must load the values from the current x vector.
	this->LoadFromVector(x);
	
	//Now populte the observation vector.
	int obsCount = 0;
	for(int i=0;i<objectsVector.size();i++)
	{
		vector<int64>& indices = objectsVector[i]->GetIndices();
		
		for(int j=0;j<indices.size();j++)
		{
			if(obsCount>=count)
			{
				cerr<<"obsCount "<<obsCount<<" more than count "<<count<<" "<<__FILE__
					<<": "<<__LINE__<<endl<<flush;
			}
			f[obsCount] = objectsVector[i]->DistanceFromPoint((*this)(indices[j]));
			obsCount++;
		}
	}
	
	//Calculate MSE.
	double sumSq = 0;
	for(int i=0;i<count;i++)
	{
		sumSq += pow(f[i],2);
	}
	if(SHOW_DEBUG)
	{
		static double lastSumsq = 0;
		if(fabs(lastSumsq-sumSq)>1e-3)
		{
			//cerr<<"Sumsq: "<<sumSq<<flush<<endl;
			fprintf(stderr,"Sumsq: %9.7f  lastSumsq: %9.7f\n",sumSq,lastSumsq);
			lastSumsq = sumSq;
		}
		
	}
#endif		
	return 0;
}

///Calculates SumSquare.
double ENLSIPRegistrationDirect::SumSquare(const vector<double>& values)
{
	//Must load the values from the current x vector.
	this->LoadFromVector(values.begin());
	
	return SumSquare();
}

double ENLSIPRegistrationDirect::SumSquare()
{
	//calculate and Return sum square.
	double sumSq = 0;
	for(int i=0;i<objectsVector.size();i++)
	{
		vector<int64>& indices = objectsVector[i]->GetIndices();
		
		for(int j=0;j<indices.size();j++)
		{
			sumSq += pow(objectsVector[i]->DistanceFromPoint((*this)(indices[j])),2);
		}
	}
	return sumSq;
}
	


///This function calculates residual value for the constraints.
bool ENLSIPRegistrationDirect::EvaluateConstraint(double* x, double* f,int count)
{
	if(count<=0)
		return 0;
	if(SHOW_DEBUG)
		cerr<<"Calling ENLSIPRegistrationDirect::EvaluateConstraint with count: "<< count <<"\n";
		
	//We need to add one contraint per transform object.
	for(int i=0;i<transformsMap.size();i++)
	{
		//Length of the quaternion should be one.
		double *xx = x + i*7;
		f[i] = 0;//xx[0]*xx[0] + xx[1]*xx[1] + xx[2]*xx[2] + xx[3]*xx[3] - 1;
	}
	
	if(SHOW_DEBUG)
	{
		for(int i=0;i<transformsMap.size();i++)
			;//cerr<<"Equality Constraint "<<i<<": "<<f[i]<<endl;
	}
	return 0;
}

vector<double> ENLSIPRegistrationDirect::Optimize(double* chiSq)
{
	if(SHOW_DEBUG)
		cerr<<"Calling ENLSIPRegistrationDirect::Optimize\n";

	vector<double> initialValues(ParamCount());
SH	
	this->SetObsCount(ObsCount());
SH	
	//We need one equation per transform.
	SetEqCount(transformsMap.size());
	SetEqCount(0);
	SetIneqCount(0);
SH	
	//Copy initial values.
	this->SaveToVector(initialValues.begin());	
SH		
	cerr<<"initial values has size: "<<initialValues.size()<<endl;
	cerr<<"Number of observations is: "<<obsCount<<endl;
	//Optimize and return the final results.
	return ENLSIPFitting::Optimize(initialValues,chiSq);	
SH						
}	

vector<double> ENLSIPRegistrationDirect::OptimizeLargeScale(double* chiSquare)
{
	if(SHOW_DEBUG)
		cerr<<"Calling ENLSIPRegistrationDirect::OptimizeLargeScale\n";
	
	//For calculating partial derivatives.
	double stepSize = 1e-6;
					
	int paramCount = ParamCount();
	vector<double> initialValues(paramCount), initialCopy(paramCount);
		
	this->SetObsCount(ObsCount());

	//Copy initial values.
	this->SaveToVector(initialValues.begin());	
	
	cerr<<"initial values has size: "<<initialValues.size()<<endl;
	
	double oldSumSq = SumSquare(initialValues);
	cerr<<"initial sumSq: "<<oldSumSq<<endl;
		
	//Now we start with large scale method implementation.
	NEWMAT::SymmetricMatrix AtA(paramCount), ATemp(paramCount), AtACopy(paramCount);
	NEWMAT::Matrix Aty(paramCount,1);
	
	double lambda = 1e-10;
	double newSumSq;
	//We have to populate the AtA matrix, observation by observation.
	//Now populte the observation vector.
	int maxIter = 20;
	for(int iter = 0;iter<maxIter;iter++)
	{	
		AtA = 0; AtACopy = 0; ATemp = 0;
		Aty = 0;
	//Update AtA Matrix.
	MakeAtAMatrix(AtA,Aty,stepSize,1e-12);
	//Levenberg-Marquardt update.
	for(int m=0;m<20;m++)
	{
		
#define NO_SVD 1		
		AtACopy = AtA;
		for(int i=0;i<paramCount;i++)
			AtACopy(i+1,i+1) = AtACopy(i+1,i+1)*(1 + lambda);//+AtACopy(i+1,i+1));
		
		//AtA and Aty are updated try to solve the system.
		NEWMAT::Matrix delta;

#if(NO_SVD)
		delta = AtACopy.i()*Aty;
#else		
	
		 //A  = U * D * V.t()
		//Solution using Svd.
 		NEWMAT::DiagonalMatrix D;
		NEWMAT::Matrix U,V;
		

		cerr<<"Lambda: "<<lambda<<" Trace(AtaCopy): "<<AtACopy.Trace()<<endl<<flush;
		cerr<<"Max: "<<AtACopy.MaximumAbsoluteValue()<<" Min: "<<AtA.MinimumAbsoluteValue()<<endl<<flush;
		
		double maxDiagonal = -1e20, minDiagonal = 1e20;
		for(int i=1;i<=paramCount;i++)
		{
			maxDiagonal = max(maxDiagonal,fabs(AtACopy(i,i)));
			minDiagonal = min(minDiagonal,fabs(AtACopy(i,i)));
		}
		cerr<<"MaxD: "<<maxDiagonal<<" Min: "<<minDiagonal<<endl<<flush;
		
		SVD(AtACopy, D, U, V);

	
		//cerr<<"Svd: "<<D(1)<<" "<<D(paramCount)<<endl<<flush;
		//cerr<<"SumSquareOfDiff: "<<(AtACopy - U*D*V.t()).SumSquare()<<endl;
		//cerr<<"SumSquareOfDiff 2: "<<(AtACopy - V*D*U.t()).SumSquare()<<endl;
		
		double maxSvd = D(1);
		int zeroCount = 0;
		
		for(int i=1;i<=paramCount;i++)
		{
			if(D(i)/maxSvd>1e-16)
				D(i) = 1.00/D(i);
			else
			{
				D(i) = 0;
				zeroCount++;
			}
			//cerr<<"svd["<<(i+1)<<"]: "<<D(i+1)<<endl;
		}
		delta = (V*D*U.t())*Aty;
		cerr<<"zeroCount: "<<zeroCount;
#endif		
		
		vector<double> newValues = initialValues;
		for(int i=0;i<paramCount;i++)
		{
			newValues[i] -= delta(i+1,1);
			//cerr<<"delta["<<i<<"]: "<<delta(i+1,1)<<endl;
		}
			
		newSumSq = SumSquare(newValues);
		
		cerr<<" newSumSq: "<<newSumSq<<" for lambda "<<lambda<<endl;
		
		if((oldSumSq-newSumSq)>1e-3)
		{
			initialValues = newValues;
			oldSumSq = newSumSq;
			lambda /= 10;
			break;
		}
		else
			lambda  *= 10;
	}
	
		cerr<<iter+1<<" new SumSq is: "<<newSumSq<<" with lambda "<<lambda<<endl;
		if((newSumSq-oldSumSq)>1e-3 || lambda>1e20)
		{
			cerr<<"Breaking as no lambda is satisfactory enough\n"<<flush;
			break;
		}			
	}
	return initialValues;
}


#if 1
//Makes atA matrix adding stabilizing term of given magnitude.
//stepSize if for partial derivative calculation.
void ENLSIPRegistrationDirect::MakeAtAMatrix(NEWMAT::SymmetricMatrix& AtA,
			NEWMAT::Matrix& Aty,
			double stepSize,
			double stabilizingTerm)
{
	int paramCount = ParamCount();
	vector<double> initialValues(paramCount), initialCopy(paramCount);
		
	this->SetObsCount(ObsCount());

	//Copy initial values.
	this->SaveToVector(initialValues.begin());	
	
	//Now we start with large scale method implementation.
	Aty = NEWMAT::Matrix(paramCount,1);
	AtA = NEWMAT::SymmetricMatrix(paramCount);
	NEWMAT::SymmetricMatrix ATemp(paramCount);
	NEWMAT::Matrix oneRow(1,paramCount);
	
	//For mapping from index to scanNumber.
	std::map<int,int> scanParamMap;
	std::map<int,int> objParamMap;
	GetParamMaps(scanParamMap,objParamMap);
	
	//We have to populate the AtA matrix, observation by observation.
	//Now populte the observation vector.
	AtA = 0; Aty = 0;
	
	//Stabilize AtA by adding a very low value to the diagonal.
	for(int i=0;i<paramCount;i++)
		AtA(i+1,i+1) = stabilizingTerm;
	for(int i=0;i<objectsVector.size();i++)
	{
		this->LoadFromVector(initialValues.begin());
		vector<int64>& indices = objectsVector[i]->GetIndices();
	
		vector<double> paramVector = objectsVector[i]->ParamVector();
		int objStart = objParamMap[i];
						
		for(int j=0;j<indices.size();j++)
		{
			int64 index = indices[j];
			int scanNumber = index>>32;
			int pointNumber = index;
			
			//Populating one row.
			oneRow = 0;
			double value0 = objectsVector[i]->DistanceFromPoint((*this)(indices[j]));
			
			//For object parameters.
			for(int k=0;k<paramVector.size();k++)
			{
				vector<double> paramCopy = paramVector;
				paramCopy[k] += stepSize;
				objectsVector[i]->FromVector(paramCopy);
				
				double value1 = objectsVector[i]->DistanceFromPoint((*this)(indices[j]));
				oneRow(1,objStart+k+1) = (value1-value0)/(paramCopy[k]-paramVector[k]);
			}
			
			//for scan parameters.
			if(transformsMap.find(scanNumber)!=transformsMap.end())
			{
				LaserTransform3D t = transformsMap[scanNumber];
				
				LaserPoint pt = (*(pointsMap[scanNumber]))[pointNumber];
				
				int scanStart = scanParamMap[scanNumber];
				
				vector<double> tVec(7);
				t.SaveToVector(tVec.begin());
				
				for(int k=0;k<7;k++)
				{
					vector<double> tVecCopy = tVec;
					tVecCopy[k] += stepSize;
					t.LoadFromVector(tVecCopy.begin());
					
					double value1 = objectsVector[i]->DistanceFromPoint(t*pt);
					oneRow(1,scanStart+k+1) = (value1-value0)/(tVecCopy[k]-tVec[k]);
				}
			}
			
			//one Row has been updated, must proceed with update of Ata.
			ATemp<<oneRow.t()*oneRow;
			AtA += ATemp;
			
			Aty += oneRow.t()*value0;
		}
	}
	
}
#endif



/*
//Only for transformation parameters.
vector<double> ENLSIPRegistrationDirect::OptimizeRegistration(double* chiSq)
{
	if(SHOW_DEBUG)
		cerr<<"Calling ENLSIPRegistrationDirect::OptimizeRegistration\n";
#define SH cerr<<__LINE__<<": "<<__FILE__<<flush<<endl;
	vector<double> initialValues(ParamCount());
SH	
	this->SetObsCount(ObsCount());
SH	
	//We need one equation per transform.
	SetEqCount(transformsMap.size());
	SetIneqCount(0);
SH	
	//Copy initial values.
	this->SaveToVector(initialValues.begin());	
SH		
	cerr<<"initial values has size: "<<initialValues.size()<<endl;
	//Optimize and return the final results.
	return ENLSIPFitting::Optimize(initialValues,chiSq);	
SH						
}
	
*/				
///A static function for executing the test.
void ENLSIPRegistrationDirect::ExecuteTest()
{

}


//Calculate StdDeviation of parameters.
vector<double> ENLSIPRegistrationDirect::CalculateStdDeviation()
{
	//For calculating partial derivatives.
	double stepSize = 1e-6;
					
	//Now we start with large scale method implementation.
	NEWMAT::SymmetricMatrix AtA(paramCount);
	NEWMAT::Matrix Aty(paramCount,1);
	
	//Update AtA Matrix.
	MakeAtAMatrix(AtA,Aty,stepSize,1e-12);
	
	NEWMAT::Matrix Cov = AtA.i();
	
	vector<double> stdDev(Cov.Nrows());
	
	for(int i=1;i<=Cov.Nrows();i++)
	{
		stdDev[i-1] = sqrt(Cov(i,i));
	}
	
	return stdDev;
}

