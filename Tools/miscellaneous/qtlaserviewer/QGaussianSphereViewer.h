
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
*   File made : December 2005
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : A widget to show gaussian sphere along with the points.
*
*--------------------------------------------------------------------*/

#include "QGLPointsCanvas.h"

class QGaussianSphereViewer:public QWidget
{
	Q_OBJECT
public:
	///Creates a new canvas 
	QGaussianSphereViewer(QWidget* parent,const LaserPoints& laserPoints, 
			const LaserPoints& normals, const char* name="");
	
	///Destructor.
	~QGaussianSphereViewer();
	
public slots:
	///Map selected points to normals.
	void MapPoints2Normals();
	
	///Map normals to points.
	void MapNormals2Points();
	
protected:
	QGLPointsCanvas* pointsCanvas;
	QGLPointsCanvas* normalsCanvas;
};
		
	
