
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
*   Purpose   : Registration of scans using parameters of fitted objects
*
*--------------------------------------------------------------------*/
#ifndef _ENLSIP_REGISTRATION_ENLSIP_H_
#define _ENLSIP_REGISTRATION_ENLSIP_H_

#include "RegistrationStructs.h"
//--------------------------------------------------------------
//Class for optimizing using only the parameters.
class ENLSIPRegistrationIndirect: public ENLSIPFitting
{
public:	
	//Constructor.
	ENLSIPRegistrationIndirect(RegistrationData* pData = NULL);
		
	///This function calculates the function values for all points.
	virtual bool EvaluateFunction(double* x,double* f,int count);
	
	///This function calculates residual value for the constraints.
	virtual bool EvaluateConstraint(double* x, double* f,int count);
	
	virtual vector<double> Optimize(vector<double>& initialValues,
						double * chiSquare = NULL,
						string* errorMessage = NULL,
						int* errorCode=NULL);

	///A static function for executing the test.
	static void ExecuteTest();

protected: 
	RegistrationData* pRegData;
};

#endif //_ENLSIP_REGISTRATION_ENLSIP_H_
