
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
*   Purpose   : Fitting of geometric objects using ENLSIP
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
#include "ENLSIPRegistrationIndirect.h"
#include "LaserPointsProcessing.h"
#include "GeneralUtility.h"
#include "StlUtilities.h"
#include "LaserObjects.h"
#include "LaserPointsFitting.h"


#define SHOW_DEBUG 0


//Constructor.
ENLSIPRegistrationIndirect::ENLSIPRegistrationIndirect(RegistrationData* pData)
:ENLSIPFitting(7,20,1,3)
{
	pRegData = pData;
	
	if(pData)
	{
		obsCount = pRegData->GetObsCount();;
		
		//Add seven parameters per scan.
		paramCount= pRegData->GetParamCount();
		
		if(paramCount<7)
		{
			cerr<<"Serious Error: ENLSIPRegistrationIndirect has less than 7 unknowns \n";
			cerr<<"Forcing switch to 7 parameters\n";
			paramCount = 7;
		}
		
	}
	functionCounter = constraintCounter = 0;
}

///This function calculates the function values for all points.
bool ENLSIPRegistrationIndirect::EvaluateFunction(double* x,double* f,int count)
{
	if(SHOW_DEBUG)
	{
		cerr<<"Calling ENLSIPRegistrationIndirect::EvaluateFunction \n";
		for(int i=0;i<7;i++)
			cerr<<"EvaluateFunction x["<<i<<"]: "<<x[i]<<endl;
	}
		
	int obsNumber = 0;
	for(RegistrationData::CorrMap::iterator i=pRegData->correspondences.begin();i!=pRegData->correspondences.end();i++)
	{
		//Get the reference and registration target.
		RegistrationTarget tRef = pRegData->targets[i->first];
		RegistrationTarget tReg = pRegData->targets[i->second[0]];
		
		
		
		vector<double> refParams = tRef.parameters;
		int refScanNumber = i->first.scanNumber;
		if(pRegData->HasTransform(refScanNumber))
		{
			pRegData->LoadTransform(refScanNumber,x);
			refParams = pRegData->transforms[refScanNumber].ApplyToObject(tReg);	
		}
		
		int scanNumber = i->second[0].scanNumber;

		
		pRegData->LoadTransform(scanNumber,x);
		
		if(!pRegData->HasTransform(scanNumber))
			cerr<<"ERR: We donot have valid transform\n";
					
		vector<double> trParams = pRegData->transforms[scanNumber].ApplyToObject(tReg);
				
		for(int j=0;j<trParams.size();j++)
		{
			if(obsNumber>=count)
			{
				cerr<<"ERROR: ENLSIPRegistrationIndirect::EvaluateFunction more observations than accounted for\n";
				continue;
			}
			
			//For plane orientation error is really serious, we have to take absolute value.
			if(tRef.type == "LaserPlane")
			{
				Vector3D normalRef;
				Vector3D normalTr;
				
				double maxRho = std::max(fabs(refParams[j+3]),fabs(trParams[j+3]));
				for(int kk=0;kk<3;kk++)
				{
					f[obsNumber] = maxRho*fabs((refParams[j]) - (trParams[j]));
					normalRef[kk] = refParams[j];
					normalTr[kk] = trParams[j];
					obsNumber++; j++;
				}
				normalRef = normalRef.Normalize();
				normalTr = normalTr.Normalize();
				double dot = fabs(normalRef.DotProduct(normalTr));
				f[obsNumber] = fabs((refParams[j]) - (trParams[j]))*(2-dot);
				obsNumber++;
			}
			else
			{
				f[obsNumber] = refParams[j] - trParams[j];
				obsNumber++;
			}
		}
	}
	
	if(obsCount > count)
		cerr<<"ERROR: ENLSIPRegistrationIndirect::EvaluateFunction obsCount>count";
	
	//The unused observations must be set to zero manually.
	for(int i=obsNumber;i<count;i++)
		f[i] = 0;

	
	double sumSq = 0;
	for(int i=0;i<count;i++)
	{
		sumSq += pow(f[i],2);
	}
	if(SHOW_DEBUG)
		printf("ENLSIPRegistrationIndirect::EvaluateFunction Sumsq: %f \n",sumSq);
	return 0;
}

///This function calculates residual value for the constraints.
bool ENLSIPRegistrationIndirect::EvaluateConstraint(double* x, double* f,int count)
{
	if(SHOW_DEBUG)
		cerr<<"Calling ENLSIPRegistrationIndirect::EvaluateConstraint with count: "<< count <<"\n";
		
	if(EqCount()!=1 || IneqCount()!=3)
		cerr<<"ERROR: ENLSIPRegistrationIndirect::EvaluateConstraint invalid count\n";	
		
	//Length of the quaternion should be one.
	f[0] = x[0]*x[0] + x[1]*x[1] + x[2]*x[2] + x[3]*x[3] - 1;
	
	//A bound constraint on x,y,z translation.
	const int ineqStart = 1;
	
	for(int i=0;i<3;i++)
	{
		f[ineqStart+2*i+0] = x[4+i] + 1e6;
		f[ineqStart+2*i+1] = -x[4+i] + 1e6;
	}

	if(SHOW_DEBUG)
	{
		for(int i=0;i<ineqStart;i++)
			cerr<<"Equality Constraint "<<i<<": "<<f[i]<<endl;
		for(int i=ineqStart;i<(ineqStart+6);i++)
			cerr<<"Ineq constraint: "<<i<<": "<<f[i]<<endl;
	}
	return 0;
}

///A static function for executing the test.
void ENLSIPRegistrationIndirect::ExecuteTest()
{
	
}



vector<double> ENLSIPRegistrationIndirect::Optimize(vector<double>& initialValues,double * chiSquare, string* errorMessage, int* errorCode)
{

	if(SHOW_DEBUG)
		cerr<<"ENLSIPRegistrationIndirect::Optimize"<<endl;
	return ENLSIPFitting::Optimize(initialValues,chiSquare,errorMessage,errorCode);
}						