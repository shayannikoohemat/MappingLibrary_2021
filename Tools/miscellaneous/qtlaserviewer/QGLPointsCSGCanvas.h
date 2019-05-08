
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
*   File made : July 2005
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Class to handle display of LaserPoints+CSG editing.
*
*--------------------------------------------------------------------*/
#ifndef _QGL_POINTS_CSG_CANVAS_H_
#define _QGL_POINTS_CSG_CANVAS_H_

#include "QGLPointsCanvas.h"
#include "LaserCSGObject.h" 

class QGLPointsCSGCanvas: public QGLPointsCanvas
{
	Q_OBJECT 

public:
	
	///Creates a new canvas 
	QGLPointsCSGCanvas(QWidget* parent=NULL,const char* name="QGLPointsCSGCanvas");

	///This is the virtual destructor.
	~QGLPointsCSGCanvas();
	
	//Gives default values to class state variables.
	void InitializeState();

public slots:	
	///This slot handles menu items which are still experimental.
	void ExperimentalFunctions(int id);
	
	///Rotate using current Opengl rotation.
	void Print();
	
	///Refresh the view. Just call display list and do some selection related drawing.
	void Repaint();
	///Shows a popup menu.
	void ShowPopupMenu();
	
		
public:	
	//Event handlers
	void paintGL();
	void mousePressEvent ( QMouseEvent * e );
	void mouseReleaseEvent ( QMouseEvent * e );
	void mouseDoubleClickEvent ( QMouseEvent * e );
	void mouseMoveEvent ( QMouseEvent * e );
protected:
	VirtualTrackball csgTrackball;
	Vector3D csgRotCenter;
	Orientation3D oldOr;
	double xAngle, yAngle;
	bool csgManipulationMode;
};	
			

#endif //_LASER_POINTS_CANVAS_H_


