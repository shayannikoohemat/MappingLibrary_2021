
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


/*   Project   : Automated reconstruction of industrial installations
*
*   File made : August 2002
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Class to handle display of LaserPoints on a glCanvas.
*
*--------------------------------------------------------------------*/
#include <QActionGroup>
#include <QAction>
#include <QMenuBar>
#include "QGaussianSphereViewer.h"
#include "QSegmentationBrowser.h"
#include "QGLPointsCanvas.h"
#include "ShMacro.h"
#include <time.h>
#include <algorithm>
#include <stdarg.h>
#include <iostream>
#include <fstream>
#include <LaserSegments.h>
#include "VisualizationUtility.h"
#include "VrmlUtility.h"
#include "GeneralUtility.h"
#include "LaserPointsFitting.h"
#include "KNNFinder.h"
#define ENABLE_DEBUGGING 0
#include "DebugFunc.h"
#include "StlUtilities.h"
#include <QRegistrationICP.h>
#include <QRegistrationIndirect.h>
#include <LaserScan.h>


#define DUMP_MATRIX(s) {double matrix[16];glGetDoublev(GL_MODELVIEW_MATRIX,matrix);for(int i=0;i<16;i++) printf("%s Matrix: %d  %lf\n",(char*)s,i,matrix[i]);}
#define INVERT(X) (0xFFFFFFFF^(X))
#define TURNOFF(X,Y) ((Y)&INVERT(X))
#define POPUP_MSG(str) QMessageBox::information(NULL,"User message", str,QMessageBox::Ok);
#define ERR_MSG(str_in) {QString str = str_in; QMessageBox::warning(NULL,"Error message", str,QMessageBox::Ok,0);}

//Globals for supporting copy/paste operation.
static LaserPoints GlobalCopiedPoints;
static vector<LaserObject*> GlobalCopiedObjects;
static LaserSegments GlobalSegments; //Global segmentation that saves point ids.
static QString GlobalSegmentsFileName;

///Some Helper functions.

///For some actions we put values in QAction::data, so the slot has to respond
///differently when the QObject::sender() in a QAction. This function checks if that is 
///the case and if the "data" which is a QVarinat really has an integer value.
static bool CheckQActionAndInt(QObject* sender,int& value)
{
	QAction* src = NULL;
	if(src=dynamic_cast<QAction*>(sender))
	{
		bool result; src->data().toInt(&result);
		
		if(result)
		{
			
			value = src->data().toInt();
			return true;
		}
	}
	return false;
}

///Same function as above but for double values.
static bool CheckQActionAndDouble(QObject* sender,double& value)
{
	QAction* src = NULL;
	if(src=dynamic_cast<QAction*>(sender))
	{
		bool result; src->data().toDouble(&result);
		
		if(result)
		{
			value = src->data().toDouble();
			return true;
		}
	}
	return false;
}

///Make a dialog with the given set of widgets putting them in a vertical layout.
///NOTE: Make sure the last argument is always NULL, otherwise you will get a crash dump.
///Example:
/// MakeDialog("My dialog",w1,w2,w3,NULL);
/// Use it here and then delete, the delete of dilaog will also destory the widgets so be careful.
QDialog* MakeDialog(const char* title, ... )
{
   	QDialog *dlg  = new QDialog;
    QPushButton *Okbtn = new QPushButton("Ok",dlg);
    
    QObject::connect(Okbtn, SIGNAL(clicked()),
                     dlg,  SLOT(accept()));
    
    
    QVBoxLayout *layout = new QVBoxLayout;

   	dlg->setWindowTitle(title);
    
	va_list marker;
   	va_start( marker, title );     /* Initialize variable arguments. */
   	
   	QWidget* w = va_arg(marker,QWidget*);
	while( w != 0 )
	{
		layout->addWidget(w);
		w = va_arg(marker,QWidget*);
	}
	va_end( marker );              /* Reset variable arguments.      */
    layout->addWidget(Okbtn);

    dlg->setLayout(layout);	
    dlg->setModal(true);
    
    return dlg;
}


///Creates a new canvas 
QGLPointsCanvas::QGLPointsCanvas(QWidget* parent,const char* name)
:QGLGeneralCanvas(parent,name)
{
	this->InitializeState();
}

///Create canvas and assign laser points.
QGLPointsCanvas::QGLPointsCanvas(const LaserPoints& pts, QWidget* parent,const char* name)
:QGLGeneralCanvas(parent,name)
{
	this->InitializeState();
	SetLaserPoints(pts);
}

//Gives default values to class state variables.
void QGLPointsCanvas::InitializeState()

{
	DEBUG("QGLPointsCanvas::InitializeState");
	//Initialize base class.
	MakeActionsAndMenubar();
	QGLGeneralCanvas::InitializeState();
	
	//Initialize this class.
	buttonPressCallback = NULL;
	buttonPressUserData = NULL;
	pointsCanvasDisplayMode = DISPLAY_DATA_POINTS|DISPLAY_CLOUD|DISPLAY_SELECTED_POINTS|DISPLAY_MODELED_OBJECTS;
	colorMappingMode = UseReflectance;
	SetPointSize(2);
	bDepthSensitiveSelection = false;
	normalLengthFactor = 1;
	SetKnn(30);
	controlPointCounter = 0;
	rotation = Rotation3D();
	translation = Vector3D();
	subSamplingFactor = 1;
	dZoomFactor = 1;
	shiftX = shiftY = shiftZ = 0;
	autoHistoEqualize = false;
	bUseDisplayList = false;
	
	UpdateActions();
	
}


char* QGLPointsCanvas::FileName()
{
	if(laserPoints.PointFile())
		strcpy(fileName,laserPoints.PointFile());
	else
		fileName[0] = NULL;

	return fileName;
}



QGLPointsCanvas::~QGLPointsCanvas ()
{
	//Free laser object pointers.
	for(int i=0;i<laserObjects.size();i++)
	{
		delete laserObjects[i];
	}
	if(displayList)
	{
		glDeleteLists(displayList,1);
	}

}


///renders Laser points.
void		
QGLPointsCanvas::DrawLaserPoints()
{
	if(laserPoints.empty())
		return;
		
	//Make a local variable with the same name as class variable.
	//While inspecting (rotating and moving data) subsample with a higher degree.
	int subSamplingFactor = max(1,this->subSamplingFactor);
	
	if(captureOn)
		subSamplingFactor = this->subSamplingFactor*1;
		
	if(colorMappingMode<2)
	{
		DrawLaserPointsGL(laserPoints, colorMappingMode==UseReflectance,colorScheme,subSamplingFactor);
	}
	else
	{
		glBegin(GL_POINTS);
		for(int i=0;i<laserPoints.size();i+=subSamplingFactor)
		{
			LaserPoint pt = laserPoints[i];
			glColor3ub(pt.Red(),pt.Green(),pt.Blue());
			//cerr<<i<<"color: "<<pt.Red()<<" "<<pt.Green()<<"  "<<pt.Blue()<<endl;
			//glColor3ub(255,i,255);
			glVertex3f(pt.X(),pt.Y(),pt.Z());
		}
		glEnd();
	}
}
	
///render selection.
void 
QGLPointsCanvas::DrawSelectedPoints()
{
	if(selectedIndices.empty())
		return;
		
	SetDrawing3DViewport();
	SetViewVolume();
	ApplyGLTransformation();
	ApplyTransform();
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	DrawLaserPointsSelectionGL(
		laserPoints,
		selectedIndices,
		colorScheme!=GLColorSchemeScaledRed?GLColorSchemeScaledRed:GLColorSchemeScaledGreen,
		/*useReflectance*/false);
}

void QGLPointsCanvas::DrawSelectedObjects()
{
	if(selectedObjectIndices.empty())
		return;
		
	SetDrawing3DViewport();
	SetViewVolume();
	ApplyGLTransformation();
	ApplyTransform();
	glDisable(GL_LIGHTING);
	glEnable(GL_DEPTH_TEST);
	glLineWidth(1);
	glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
	for(int i=0;i<selectedObjectIndices.size();i++)
		laserObjects[selectedObjectIndices[i]]->Draw(Vector3D(1,1,0),this);
}

void QGLPointsCanvas::DrawSelectedSegments()
{
	DrawSelectedPoints();
}

void
QGLPointsCanvas::UpdateDisplayList ()
{
	ActivateCanvas();
	//Just delete the list, will be update in the paintGL.
	if(displayList)
	{
		glDeleteLists (displayList, 1);
		displayList = 0;
	}
}

void
QGLPointsCanvas::ApplyGLRotationToLaserPoints (LaserPoints & laserPoints)
{
	DataBoundsLaser dataBounds = laserPoints.DataBounds ();
	
	SetViewVolume();
	
	double offsetX, offsetY, offsetZ;
	offsetX = dataBounds.Minimum ().X () + dataBounds.XRange () * 0.5;
	offsetY = dataBounds.Minimum ().Y () + dataBounds.YRange () * 0.5;
	offsetZ = dataBounds.Minimum ().Z () + dataBounds.ZRange () * 0.5;

	glMatrixMode (GL_MODELVIEW);

	//push the matrix so that changes can be undone.
	glPushMatrix ();

	glLoadIdentity ();
	//glScalef(dZoomFactor,dZoomFactor,dZoomFactor); We are not going to scale 
	//We don't want extra shift either glTranslatef(shiftX,shiftY,shiftZ);

	//And translate back once the rotation is done around the centroid of data.
	glTranslatef (offsetX, offsetY, offsetZ);
	trackball.MultiplyWithRotationMatrix ();
	glTranslatef (-offsetX, -offsetY, -offsetZ);

	//Now read the current matrix.
	double pMatrix[16];
	glGetDoublev (GL_MODELVIEW_MATRIX, pMatrix);

	//Apply this transformation to each point in the LaserPoints.
	int i;
	for (i = 0; i < laserPoints.size (); i++)
	{
		MultiplyLaserPointWithMatrix (pMatrix, laserPoints[i]);
	}

	//Pop the matrix to undo the changes.   
	glPopMatrix ();
}


void
QGLPointsCanvas::ApplyGLTransformationForPicking (int x, int y, int width,
													int height)
{
	GLint viewport[4];

	SetDrawing3DViewport();
	//Get the viewport parameters
	glGetIntegerv (GL_VIEWPORT, viewport);

	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();

	if (width > 0 && height > 0)
	{
		gluPickMatrix (x + 0.5 * width, viewport[3] - (y + 0.5 * height),
					   width, height, viewport);
	}
	else
	{
		gluPickMatrix (x, viewport[3] - y, 5.0, 5.0, viewport);
	}

	DataBoundsLaser dataBounds = laserPoints.DataBounds ();
	
	SetViewVolume();
	
	double offsetX, offsetY, offsetZ;
	offsetX = dataBounds.Minimum ().X () + dataBounds.XRange () * 0.5;
	offsetY = dataBounds.Minimum ().Y () + dataBounds.YRange () * 0.5;
	offsetZ = dataBounds.Minimum ().Z () + dataBounds.ZRange () * 0.5;

	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
	glScalef (dZoomFactor, dZoomFactor, dZoomFactor);
	glTranslatef (shiftX, shiftY, shiftZ);
	trackball.MultiplyWithRotationMatrix ();
	glTranslatef (-offsetX, -offsetY, -offsetZ);
	ApplyTransform();
}

void QGLPointsCanvas::DrawGLObjects()
{
	ActivateCanvas();
	SetDrawing3DViewport();
	
	glEnable (GL_DEPTH_TEST );
	glPolygonMode (GL_FRONT_AND_BACK, polygonMode);
	
	//Do all Tranformations.
	ApplyGLTransformation ();
	ApplyTransform();

}

void QGLPointsCanvas::DrawSegmentedPoints()
{
	DEBUG("DrawSegmentedPoints");
	if(segmentedVector.empty())
	{
		return;
	}
	SetDrawing3DViewport();
	
	glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST );
	glDisable(GL_LIGHTING);

	ApplyGLTransformation ();
	ApplyTransform();
	
	::DrawSegmentsVectorGL(laserPoints,segmentedVector);
}


void QGLPointsCanvas::DrawGLSelection()
{
	if (selectedIndices.size ())
	{
		//Draw selection
		DrawSelectedPoints();

	}
	else if (selectedSegmentIndices.size ())
	{
		//Make temporary selected laserpoints.
		LaserPoints pts;
		glDisable(GL_LIGHTING);
		glDisable(GL_DEPTH_TEST);
		for(int j=0;j<selectedSegmentIndices.size();j++)
		{
			IndicesVector& pointIndices = segmentedVector[selectedSegmentIndices[j]];
			
			DrawLaserPointsSelectionGL(
						laserPoints,
						pointIndices,
						colorScheme!=GLColorSchemeScaledRed?GLColorSchemeScaledRed:GLColorSchemeScaledGreen,
						colorMappingMode==UseReflectance);
		}
	}
}


void QGLPointsCanvas::DrawModeledObjects()
{
	if(laserObjects.empty())
		return;
		
	SetDrawing3DViewport();
	SetViewVolume();
	ApplyGLTransformation();
	ApplyTransform();
		
	//Turn on glColor material.
	glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
	glEnable(GL_COLOR_MATERIAL);
	
	glEnable(GL_DEPTH_TEST);
	//draw copied object through pointers and virtual function.
	for(int i=0;i<laserObjects.size();i++)
		laserObjects[i]->Draw(this);
		
	//Disable color material.
	glDisable(GL_COLOR_MATERIAL);
}

void 
QGLPointsCanvas::GetBoundingBox(Vector3D& minimum,Vector3D& maximum,Vector3D& range, Vector3D& middle)
{
	
	if(laserPoints.empty())
	{
		minimum = Vector3D(-10,-10,-10);
		maximum = Vector3D(10,10,10);
	}
	else
	{
		DataBoundsLaser dataBounds = laserPoints.DataBounds ();
	
		minimum = dataBounds.Minimum();
		maximum = dataBounds.Maximum();
	}
	range = maximum - minimum;
	middle = range*0.5+minimum;		
}


void
QGLPointsCanvas::Repaint()
{
	updateGL();
}

#define DEBUG_QT_MESSAGE(a) //{static int count ;cerr<<"QGLPointsCanvas::"<<#a<<" : "<<count++<<endl;}
#define PROCESS_QT_MESSAGE(a) {DEBUG_QT_MESSAGE(a);return QGLGeneralCanvas::a(e);}
#define ENABLE_DEBUGGING 0
#include "DebugFunc.h"
void 
QGLPointsCanvas::mousePressEvent ( QMouseEvent * e )
{
#if 1
	int x = e->x();
	int y = e->y();
	Qt::KeyboardModifiers state =  e->modifiers();
	Qt::MouseButtons  button = e->buttons();
	
	lastState = state;
	lastButton = button;
	startX = lastX = x;
	startY = lastY = y;
	captureOn = true;

	if(button & Qt::LeftButton)
	{
		//Rectangular selection.
		if (SHIFT_ON || CONTROL_ON)
			selectionOn = true;
	}
	else if(button & Qt::RightButton)
	{
		//Free-hand selection
		if (SHIFT_ON || CONTROL_ON)
		{
			selectionOn = true;
			xFreehandPoints.clear();
			yFreehandPoints.clear();
		}
	}
	else if(button & Qt::MidButton)
	{
		//User defined callback will be called only if control key is pressed
		if (buttonPressCallback && (CONTROL_ON) && SHIFT_ON && ALT_ON)
		{
			buttonPressCallback (buttonPressUserData);
		}
		//Rectangular selection.
		else if (SHIFT_ON || CONTROL_ON)
		{
			selectionOn = true;
		}
	}
		
	//Load an appropriate cursor.
	if(selectionOn)
		setCursor(Qt::CrossCursor);
	else if(button & Qt::LeftButton)
		setCursor(Qt::ArrowCursor);
	else if(button & Qt::MidButton)
		setCursor(Qt::SizeAllCursor);
	else if(button & Qt::RightButton)
		setCursor(Qt::SizeVerCursor);
	//Grab mouse. Must release in mouse release.
	grabMouse();
#endif	
	DEBUG("QGLPointsCanvas::mousePressEvent");
	QWidget::mousePressEvent(e);
}
void 
QGLPointsCanvas::mouseReleaseEvent ( QMouseEvent * e )
{
#if 1
	int x = e->x();
	int y = e->y();
	
	//Somehow in qt-4 the state and button are invalid for release event.
	Qt::KeyboardModifiers state =  e->modifiers();
	Qt::MouseButtons  button = e->buttons();
	
	//Use the old ones for the time being.
	button = lastButton;
	state = lastState;
	
	//If segment size is zero, we don't want to select segments.
	if(segmentedVector.empty())
		pointsCanvasDisplayMode = pointsCanvasDisplayMode &(DISPLAY_SEGMENTED_POINTS^0xFFFF);

	if (captureOn && selectionOn)
	{
		if(lastX > x)
		{
			swap(x,lastX);
		}

		if(lastY > y)
		{
			swap(y,lastY);
		}
		
		//Get the selection in a new vector.
		IndicesVector currentSelection;
		if(button & Qt::LeftButton)
		{
			//Window selection mode for points.
			if(pointsCanvasDisplayMode & DISPLAY_SEGMENTED_POINTS)
			{
				IndicesVector currentSegmentSelection;
				WindowSelectSegments(lastX, lastY,
									(int)fabs ((double)(x - lastX)),
									(int)fabs ((double)(y - lastY)),
									currentSegmentSelection);
				
				for(int i=0;i<currentSegmentSelection.size();i++)
				{
					for(int j=0;j<segmentedVector[currentSegmentSelection[i]].size();j++)
						currentSelection.push_back(segmentedVector[currentSegmentSelection[i]][j]);
				}
				
			}
			else
			{
				WindowSelectPoints(lastX, lastY,
									(int)fabs ((double)(x - lastX)),
									(int)fabs ((double)(y - lastY)),
									currentSelection,ALT_ON);
				//cerr<<"Current selection has size: "<<currentSelection.size()<<endl;
				
			}
		}
		else if(button & Qt::RightButton)
		{
			//Freehand selection mode.
			FreehandSelectPoints (currentSelection);
		
		}
		
		if(RIGHT_PRESS || LEFT_PRESS)
		{
			if(SHIFT_ON && CONTROL_ON)
			{
				//Deselect the selected points if possible.

				//Make temorary sets..
				IndicesSet selectedSet(selectedIndices.begin(),selectedIndices.end());
				IndicesSet currentSet(currentSelection.begin(),currentSelection.end());

				//Find their intersection.
				IndicesSet intersectionSet;
				insert_iterator<IndicesSet> resultIter(intersectionSet,intersectionSet.begin());
				set_intersection(selectedSet.begin(),selectedSet.end(),currentSet.begin(),currentSet.end(),resultIter);

				//Find the difference of old selection and intersection.
				IndicesSet differenceSet;
				insert_iterator<IndicesSet> diffIter(differenceSet,differenceSet.begin());
				set_difference(selectedSet.begin(),selectedSet.end(),intersectionSet.begin(),intersectionSet.end(),diffIter);


				//Copy the difference set to selectedIndices.
				IndicesSet::const_iterator iter = differenceSet.begin();
				selectedIndices.reserve(differenceSet.size());
				selectedIndices.clear();

				for(;iter!=differenceSet.end();iter++)
					selectedIndices.push_back(*iter);


			}
			else if(SHIFT_ON)
			{
				//Replace old selection by new one.
				selectedIndices = currentSelection;
			}
			else if(CONTROL_ON)
			{
				//Take union of two selections.

				//Make temporary sets.
				IndicesSet selectedSet(selectedIndices.begin(),selectedIndices.end());
				IndicesSet currentSet(currentSelection.begin(),currentSelection.end());

				//Take the Union of sets.
				IndicesSet combinedSet;
				insert_iterator<IndicesSet> resultIter(combinedSet,combinedSet.begin());
				set_union(selectedSet.begin(),selectedSet.end(),currentSet.begin(),currentSet.end(),resultIter);

				//Copy the Union set to selectedIndices.
				IndicesSet::const_iterator iter = combinedSet.begin();
				selectedIndices.reserve(combinedSet.size());
				selectedIndices.clear();

				for(;iter!=combinedSet.end();iter++)
					selectedIndices.push_back(*iter);
			}
		}
		
		//This is for object selection.
		if(MID_PRESS)
		{			
			IndicesVector currentObjectSelection;
			WindowSelectObjects(lastX, lastY,
									(int)fabs ((double)(x - lastX)),
									(int)fabs ((double)(y - lastY)),
									currentObjectSelection);
			
			if(SHIFT_ON && CONTROL_ON)
			{
				//Deselect the selected objects if possible.
				
				//Make temorary sets..
				IndicesSet selectedSet(selectedObjectIndices.begin(),selectedObjectIndices.end());
				IndicesSet currentSet(currentObjectSelection.begin(),currentObjectSelection.end());
				
				//Find their intersection.
				IndicesSet intersectionSet;
				insert_iterator<IndicesSet> resultIter(intersectionSet,intersectionSet.begin());
				set_intersection(selectedSet.begin(),selectedSet.end(),currentSet.begin(),currentSet.end(),resultIter);
								
				//Find the difference of old selection and intersection.
				IndicesSet differenceSet;
				insert_iterator<IndicesSet> diffIter(differenceSet,differenceSet.begin());
				set_difference(selectedSet.begin(),selectedSet.end(),intersectionSet.begin(),intersectionSet.end(),diffIter);


				//Copy the difference set to selectedIndices.
				IndicesSet::const_iterator iter = differenceSet.begin();
				selectedObjectIndices.reserve(differenceSet.size());
				selectedObjectIndices.clear();

				for(;iter!=differenceSet.end();iter++)
					selectedObjectIndices.push_back(*iter);
			}
			else if(SHIFT_ON)
			{
				//Replace old selection by new one.
				selectedObjectIndices = currentObjectSelection;
			}
			else if(CONTROL_ON)
			{
				//Take union of two selections.

				//Make temporary sets.
				IndicesSet selectedSet(selectedObjectIndices.begin(),selectedObjectIndices.end());
				
				for(int i=0;i<currentObjectSelection.size();i++)
				{
					if(!selectedSet.count(currentObjectSelection[i]))
						selectedObjectIndices.push_back(currentObjectSelection[i]);
				}
				
			}
		}
	}
	else if(RIGHT_PRESS && (abs (x - startX)<5 && fabs ((double)(y - startY))<5))
	{
		//Show popup menu.
		//before doing that release the mouse as it conflicts with the modal dialog boxes.
		releaseMouse();
		ShowPopupMenu();
	}
	
	if(ALT_ON && selectedIndices.size()>=1)
	{	
		AddControlPoint();		
	}
	
	emit SegmentSelectionChanged(selectedSegmentIndices.size());
	emit PointSelectionChanged(selectedIndices.size());
	emit ObjectSelectionChanged(selectedObjectIndices.size());
	
	//Renumber the selected control points in the order selected.
	if(selectedObjectIndices.size())
	{
		int count = 0;
		for(int i=0;i<selectedObjectIndices.size();i++)
		{
			if(laserObjects[selectedObjectIndices[i]]->Name()=="LaserControlPoint")
				laserObjects[selectedObjectIndices[i]]->SetId(++count);
		}
		emit ControlPointSelectionChanged(count);
	}
	
	if(captureOn)
		UpdateDisplayList();

	captureOn = false;
	selectionOn = false;
	
	//Release mouse grab.
	releaseMouse();
	unsetCursor();
	Repaint();
#endif	
	DEBUG("QGLPointsCanvas::mouseReleaseEvent");
	QWidget::mouseReleaseEvent(e);
}
	
void 
QGLPointsCanvas::mouseDoubleClickEvent ( QMouseEvent * e )
{
	PROCESS_QT_MESSAGE(mouseDoubleClickEvent)
}

///Add a control point.
void 
QGLPointsCanvas::AddControlPoint()
{
	if(selectedIndices.empty())
		return;
		
	if(laserObjects.empty())
		controlPointCounter = 0;
		
	laserObjects.push_back(new LaserControlPoint(laserPoints[selectedIndices[0]],
							laserPoints.DataBounds().MaxRange()/50.00,Vector3D(0.8,0.1,0.1),++controlPointCounter,""));
	selectedIndices.resize(0);
		
	//cerr<<"Adding a new control point\n";
			
	emit SegmentSelectionChanged(selectedSegmentIndices.size());
	emit PointSelectionChanged(selectedIndices.size());
	emit ObjectSelectionChanged(selectedObjectIndices.size());
	
	Repaint();
}

void 
QGLPointsCanvas::mouseMoveEvent ( QMouseEvent * e )
{
#if 1
	int x = e->x();
	int y = e->y();
	int w = width();
	int h = height();

	Qt::KeyboardModifiers state =  e->modifiers();
	Qt::MouseButtons  button = e->buttons();
		
	//Settings for drawing the outline
	glDisable(GL_LIGHTING);glDisable(GL_DEPTH_TEST);glLineWidth(SELECTION_OUTLINE_WIDTH);
	
	if (captureOn && selectionOn 
		&& (lastButton & Qt::LeftButton || lastButton & Qt::MidButton))
	{
		Repaint ();
		SetDrawing2DViewport();
		
		 
		if(lastButton & Qt::MidButton)
		{
			glLineStipple(1, 0xFF00);
			glEnable( GL_LINE_STIPPLE);
		}
		
		glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
		glDrawBuffer (GL_FRONT);
		glMatrixMode (GL_PROJECTION);
		glLoadIdentity ();
		gluOrtho2D (0, w, 0, h);

		glMatrixMode (GL_MODELVIEW);
		glLoadIdentity ();
		
		
		if(CONTROL_ON && SHIFT_ON)
			glColor3f(1.0,1.0,1.0);
		else if(CONTROL_ON)
			glColor3f(0.0,1.0,0.0);
		else if(SHIFT_ON)
			glColor3f(1.0,0.0,0.0);
			
		//Settings for drawing the outline
		glDisable(GL_LIGHTING);glDisable(GL_DEPTH_TEST);glLineWidth(SELECTION_OUTLINE_WIDTH);
	

		glBegin (GL_QUADS);
		glVertex2f (lastX, h - lastY);
		glVertex2f (x, h - lastY);
		glVertex2f (x, h - y);
		glVertex2f (lastX, h - y);
		glEnd ();
		glFlush ();
		glDrawBuffer (GL_BACK);
		glDisable( GL_LINE_STIPPLE);	
	}
	else if (captureOn && selectionOn && lastButton & Qt::RightButton)
	{
		//Free hand selection.	
		Repaint ();
		SetDrawing2DViewport();
		glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
		glDrawBuffer (GL_FRONT);
		glMatrixMode (GL_PROJECTION);
		glLoadIdentity ();
		gluOrtho2D (0, w, 0, h);

		glMatrixMode (GL_MODELVIEW);
		glLoadIdentity ();
		
		if(CONTROL_ON && SHIFT_ON)
			glColor3f(1.0,1.0,1.0);
		else if(CONTROL_ON)
			glColor3f(0.0,1.0,0.0);
		else if(SHIFT_ON)
			glColor3f(1.0,0.0,0.0);
			
		xFreehandPoints.push_back(x);
		yFreehandPoints.push_back(h-y);

		//Settings for drawing the outline
		glDisable(GL_LIGHTING);glDisable(GL_DEPTH_TEST);glLineWidth(SELECTION_OUTLINE_WIDTH);
	
		glBegin (GL_LINE_STRIP);
		for(int i=0;i<xFreehandPoints.size();i++)
			glVertex2f (xFreehandPoints[i],yFreehandPoints[i]);
		glEnd ();
		glFlush ();
		glDrawBuffer (GL_BACK);
	}


	else if (captureOn && lastButton & Qt::LeftButton)
	{
		trackball.ProcessMouse (lastX,lastY, x, y, w, h);
		lastX = x;
		lastY = y;
		Repaint ();
	}

	else if (!selectionOn && captureOn && lastButton & Qt::MidButton)
	{
		shiftX +=GetNormalizedShift (x - lastX);
		shiftY -=GetNormalizedShift (y - lastY);
		lastX = x;
		lastY = y;
		Repaint ();
	}

	else if (captureOn && lastButton & Qt::RightButton)
	{
		if (y > lastY)
		{
			dZoomFactor *=(1.00 + GetNormalizedZoom (fabs ((double)(y - lastY))));
		}
		else
		{
			dZoomFactor /= (1.00 +GetNormalizedZoom (fabs((double)(y -lastY))));
		}
		DEBUG ("dZoomFactor: %4.2f", dZoomFactor);
		lastX = x;
		lastY = y;
		Repaint ();
	}
//	setFocus();
#endif	
	DEBUG("QGLPointsCanvas::mouseMoveEvent")
	//QWidget::mouseMoveEvent(e);
}

void 
QGLPointsCanvas::wheelEvent ( QWheelEvent * e )
{
	if(e->delta()>0)
	{
		dZoomFactor *= (1.00 +GetNormalizedZoom (e->delta()));	
		Repaint();
	}
	else if(e->delta()<0)
	{
		dZoomFactor /= (1.00 +GetNormalizedZoom (-e->delta()));	
		Repaint();
	}
	
	PROCESS_QT_MESSAGE(wheelEvent)
}

void 
QGLPointsCanvas::keyPressEvent ( QKeyEvent * e )
{

	Qt::KeyboardModifiers state =  e->modifiers();
	int key = e->key();
		
	switch (key)
	{
	//Add the id's of current selection to global segmentation.
	case Qt::Key_A:
	{
		//Remove last segment.
		if (CONTROL_ON && GlobalSegments.size())
		{
			GlobalSegments.resize(GlobalSegments.size()-1);
			cerr<<"Removed last segment from Global segments. New size: "<<GlobalSegments.size()<<endl;
		}
		else if(selectedIndices.size())
		{
			LaserPoints sel = laserPoints.Select(selectedIndices);			
			vector<int> seg;
			for(int i=0;i<sel.size();i++)
				seg.push_back(sel[i].GetId());
			GlobalSegments.push_back(seg);
			
			cerr<<"Another segment added to Global segments. New size: "<<GlobalSegments.size()<<endl;
		}
		
	}
	break;
	
	//Empty the global segmentation.
	case Qt::Key_E:
	{
		if(GlobalSegments.size())
		{
			if(QMessageBox::question(NULL,"Confirm clear", 
					"Do you want to clear the global segments?",
					QMessageBox::Yes,QMessageBox::Default|QMessageBox::No)==QMessageBox::Yes)
			{
				GlobalSegments.clear();
				cerr<<"Global segments has no size of : "<<GlobalSegments.size();
			}
		}
	}
	break;
	
	//Save global segments
	case Qt::Key_G:
	{
		if(SHIFT_ON && CONTROL_ON || GlobalSegmentsFileName.isEmpty())
		{
			GlobalSegmentsFileName = QFileDialog::getSaveFileName(this,
					"Choose the file to save/load segments to/from" 
					".",
                    "laser point files (*.segments)");
		}
		else if(SHIFT_ON)
		{
			QString fn = GlobalSegmentsFileName;
			if ( !fn.isEmpty() ) 
			{
				//If NULL set the points to this laserPoints. But make sure you don't
				//modify it, or close this window otherwise may get segmentation fault.
				if(GlobalSegments.GetLaserPoints()==NULL)
					GlobalSegments.SetLaserPoints(&laserPoints);

				GlobalSegments.Load((char*)(fn.toStdString().c_str()));
				cerr<<"Global segments loaded from"<<fn.toStdString()<<" with "<<GlobalSegments.size()<<" regions"<<endl;
				
			}		
		}
		else
		{
		
			QString fn = GlobalSegmentsFileName;
			if ( !fn.isEmpty() ) 
			{
				char buff[4096];
				snprintf(buff,4096,"mv %s %s.backup ",fn.toStdString().c_str(),fn.toStdString().c_str());
				system(buff);

				//If NULL set the points to this laserPoints. But make sure you don't
				//modify it, or close this window otherwise may get segmentation fault.
				if(GlobalSegments.GetLaserPoints()==NULL)
					GlobalSegments.SetLaserPoints(&laserPoints);

				GlobalSegments.Save((char*)(fn.toStdString().c_str()));
				cerr<<"Global segments saved to "<<fn.toStdString()<<endl;
			}
		}
		cerr<<"G_exit\n";
	}
	break;
	
	case Qt::Key_Plus:
	{
		IncreasePointSize();
	}
	break;
	
	case Qt::Key_Minus:
	{
		DecreasePointSize();
	}
	break;
	
	case Qt::Key_I:
	{
		ShowInformation();
	}
	break;

	
	
	//Change normal length factor.
	//Shift+F1 normalLengthFactor/=2
	//Ctrl+F1 normalLengthFactor*=2
	case Qt::Key_L:
	{
		if (SHIFT_ON)
			normalLengthFactor/= 2;
		else if(CONTROL_ON)	
			normalLengthFactor *= 2;
		cerr<<"normalLengthFactor: "<<normalLengthFactor<<endl;
		UpdateDisplayList();
		Repaint();
	}
	break;
	
	//Change k-nearest neighbours.
	//Shift+F1 kNN/=2
	//Ctrl+F1 kNN*=2
	case Qt::Key_K:
	{
		if(SHIFT_ON)
			kNN /= 2;
		else if(CONTROL_ON)
			kNN *= 2;
		SetKnn(kNN);
		cerr<<"KNN: "<<kNN<<endl;
	}
	break;
	
	
	//Paste objects or points.
	//Ctrl+V paste objects.
	//Shift+V paste points.
	case Qt::Key_V:
	{
		if(CONTROL_ON)
		{
			PasteObjects();
		}
		else if(SHIFT_ON)
			PastePoints();
	}
	break;
	
	//Enables TIN
	case Qt::Key_T:
	{
		if(!(displayType&DISPLAY_TIN))
		{
			SetDisplayType (DISPLAY_TIN|DISPLAY_SELECTED_POINTS|DISPLAY_MODELED_OBJECTS);
		}
		else
			SetDisplayType (DISPLAY_CLOUD|DISPLAY_DATA_POINTS|DISPLAY_SELECTED_POINTS|DISPLAY_MODELED_OBJECTS);
		
	}
	break;

	//Toggle filling modes.
	case Qt::Key_F:
	{
		//Toggle filling for tin.
		if (polygonMode == GL_FILL)
			polygonMode = GL_LINE;
		else if(polygonMode == GL_LINE)
			polygonMode = GL_POINT;
		else
			polygonMode = GL_FILL;
		UpdateDisplayList();
		Repaint ();
	}
	break;

	case Qt::Key_R:
	{
		//Reset the view.
		if (CONTROL_ON)
		{
			ResetView ();
		}
		else
		{
			//Toggle reflectance.
			SetColorMappingMode (ColorMappingMode(((int)colorMappingMode+1)%3));
			UpdateDisplayList();
			Repaint ();
		}
	}
	break;

	//Crop the points. Keep only the selected ones.
	case Qt::Key_X:
		if(SHIFT_ON)
			Crop();
	break;
	
	//Open a new viewer with selected points.
	case Qt::Key_N:
	{
		this->InNewWindow();
	}
	break;
	
	//Change color scheme.
	case Qt::Key_C:
		if (CONTROL_ON && SHIFT_ON)
		{
			//Toggle colorbar. If visible hide and vice versa.
			ToggleColorbar ();
		}
		else
		{
			SetPalette(colorScheme+1);
		}
	break;
	
	//repaint the screen.
	case Qt::Key_F5:
	{
		UpdateDisplayList();
		Repaint();
		break;
	}
	
	//Show help screen.
	case Qt::Key_H:
	{
		ShowHelp();
	}
	break;

	
	case Qt::Key_S:
	if(CONTROL_ON)
	{
		smoothNormals = !smoothNormals;
		cerr<<"smoothNormals: "<<smoothNormals<<endl;
		UpdateDisplayList();
		Repaint();
	}
	else
	{
		//select all points
		SelectAll();
		Repaint ();
	}
	break;
	}

	PROCESS_QT_MESSAGE(keyPressEvent)
}

void 
QGLPointsCanvas::keyReleaseEvent ( QKeyEvent * e )
{
	PROCESS_QT_MESSAGE(keyReleaseEvent)
}

void 
QGLPointsCanvas::focusInEvent ( QFocusEvent * e)
{
	PROCESS_QT_MESSAGE(focusInEvent)
}

void 
QGLPointsCanvas::focusOutEvent ( QFocusEvent * e)
{
	PROCESS_QT_MESSAGE(focusOutEvent)
}


void 
QGLPointsCanvas::enterEvent ( QEvent * e)
{
	setFocus();
	PROCESS_QT_MESSAGE(enterEvent)
}

void 
QGLPointsCanvas::leaveEvent ( QEvent * e)
{
	clearFocus();
	PROCESS_QT_MESSAGE(leaveEvent)
}


void 
QGLPointsCanvas::paintGL( )
{
	DEBUG("QGLPointsCanvas::paintGL ()");
	
	ActivateCanvas();
	if (!laserPoints.size ())
	{
		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		DrawText(-0.8,0,0.1,Vector3D(1.0,1.0,0.0),"Nothing to show!!!");
		glFlush ();
	}
	else
	{
		//Beware, glEnable can't use |(or'ed) parameter list
		//must call separately for each.
		
		glEnable (GL_LINE_SMOOTH);
		glEnable(GL_POLYGON_SMOOTH);
		glEnable(GL_POINT_SMOOTH);
		glPolygonMode (GL_FRONT_AND_BACK, polygonMode);
		glPointSize(pointSize);
#if 0		
		
		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_LIGHTING);
		glEnable(GL_NORMALIZE);
		glHint(GL_POINT_SMOOTH_HINT,GL_NICEST);
		glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);
#endif		

		glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
		SetupGLLighting();
		
		SetDrawing3DViewport();
		SetViewVolume();
		ApplyGLTransformation();
		ApplyTransform();
		
		if(bUseDisplayList)
		{
			if (!displayList)
			{
				displayList = glGenLists (1);
				//Begin compiling the list
				glNewList(displayList,GL_COMPILE);
				DrawAll();
				glEndList();
			}
			glCallList(displayList);
		}
		else
			DrawAll();
			
		if(pointsCanvasDisplayMode & DISPLAY_MODELED_OBJECTS)
		{
			glEnable(GL_LIGHTING);
			glEnable(GL_DEPTH_TEST);
			DrawModeledObjects();
			DrawSelectedObjects();
		}

		if(pointsCanvasDisplayMode & DISPLAY_SELECTED_POINTS)
			DrawSelectedPoints();
		
		if(pointsCanvasDisplayMode & DISPLAY_SEGMENTED_POINTS)
			DrawSelectedSegments();

		//axis indicator.
		glEnable(GL_LIGHTING);
		glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
		DrawAxisIndicator();

		//color bar.
		if(colorbarOn)
		{
			glDisable(GL_LIGHTING);
			DrawColorbar();

			//upper and lower limit of the color bar.
			double minRef,maxRef,scaleFactor;
			laserPoints.ReflectanceRange(&minRef,&maxRef,&scaleFactor);
			DrawColorbarMinMax(minRef,maxRef);
		}
		glFlush ();
	}

	//According the documentation its not necessary to call swap buffers
	//as its automatically done by OnPaint
	DEBUG("QGLPointsCanvas::~~~paintGL");
}

void 
QGLPointsCanvas::DrawAll( )
{
	if(pointsCanvasDisplayMode & DISPLAY_TIN)
	{
		if(polygonMode == GL_FILL)
		{
			glEnable(GL_LIGHTING);
			glEnable(GL_COLOR_MATERIAL);
		}
		else
		{
			glDisable(GL_LIGHTING);
			glDisable(GL_COLOR_MATERIAL);
		}
		glEnable(GL_DEPTH_TEST);

		DrawLaserPointsTINGL(laserPoints,colorScheme,colorMappingMode==UseReflectance,smoothNormals);
	}
	if(pointsCanvasDisplayMode & DISPLAY_DATA_POINTS)
	{
		glDisable(GL_LIGHTING);
		glEnable(GL_DEPTH_TEST);
		DrawLaserPoints();

		//Draw normal vectors.
		if(normalPoints.size() == laserPoints.size())
		{
			glLineWidth(1);
			glDisable(GL_LIGHTING);
			DrawLaserPointsNormalsGL(laserPoints,normalPoints,normalLengthFactor,GLColorSchemeScaledRed,GLColorSchemeScaledGreen,
							colorMappingMode==UseReflectance);
		}
	}
	
	if(pointsCanvasDisplayMode & DISPLAY_SEGMENTED_POINTS)
	{
		glDisable(GL_LIGHTING);
		glClear(GL_DEPTH_BUFFER_BIT);
		glEnable(GL_DEPTH_TEST);
		
		//Draw segments.
		if(!segmentedVector.empty())
		{
			SetDrawing3DViewport();
	
			glClear(GL_DEPTH_BUFFER_BIT);
			glEnable(GL_DEPTH_TEST );
			glDisable(GL_LIGHTING);

			::DrawSegmentsVectorGL(laserPoints,segmentedVector);
		}
	}
}


void 
QGLPointsCanvas::closeEvent ( QCloseEvent * e )
{
	PROCESS_QT_MESSAGE(closeEvent)
}


void 
QGLPointsCanvas::contextMenuEvent ( QContextMenuEvent * e )
{
	//As Mouse is being grabbed this doesn't seem to function well.
	//So instead we will show the context menu ourselves in mouse-release event.
	//provided the movement has been smaller than a set threshold.
	//Before doing that we will release the mouse.
	
	//ShowPopupMenu();
	//PROCESS_QT_MESSAGE(contextMenuEvent)
	UpdateActions();
}

void 
QGLPointsCanvas::dragEnterEvent ( QDragEnterEvent * e)
{
	PROCESS_QT_MESSAGE(dragEnterEvent)
}


void 
QGLPointsCanvas::dragMoveEvent ( QDragMoveEvent * e)
{
	PROCESS_QT_MESSAGE(dragMoveEvent)
}


void 
QGLPointsCanvas::dragLeaveEvent ( QDragLeaveEvent * e)
{
	PROCESS_QT_MESSAGE(dragLeaveEvent)
}


void 
QGLPointsCanvas::dropEvent ( QDropEvent * e)
{
	PROCESS_QT_MESSAGE(dropEvent)
}


void 
QGLPointsCanvas::showEvent ( QShowEvent * e)
{
	ActivateCanvas();
	int x, y, width, height;
	GetGeometry (x, y, width, height);
	glViewport (0, 0, (GLint) width, (GLint) height);
	
	UpdateDisplayList ();
	Repaint ();
	
	emit SegmentSelectionChanged(selectedSegmentIndices.size());
	emit PointSelectionChanged(selectedIndices.size());
	emit ObjectSelectionChanged(selectedObjectIndices.size());
	emit PointSizeChanged(pointSize);
	emit CopiedPointsChanged(GlobalCopiedPoints.size());
	emit CopiedObjectsChanged(GlobalCopiedObjects.size());
	emit KnnChanged(kNN);
	emit ColormapChanged(colorScheme);
	emit ColorMappingModeChanged((int)colorMappingMode);
	
	PROCESS_QT_MESSAGE(showEvent)
}

void 
QGLPointsCanvas::hideEvent ( QHideEvent * e)
{
	PROCESS_QT_MESSAGE(hideEvent)
}

void QGLPointsCanvas:: SegmentPointCloud()
{
	LaserSmoothSegments segments(&laserPoints,kNN);
	segmentedVector = segments;
	PostSegmentationProcessing();
}

///Segment into connected segments.
void QGLPointsCanvas::PostSegmentationProcessing()
{
	//Update display.
	pointsCanvasDisplayMode = pointsCanvasDisplayMode | DISPLAY_SEGMENTED_POINTS;
	SetDisplayType(pointsCanvasDisplayMode);
	
	//Update selection and emit signals.
	selectedSegmentIndices.clear();
	emit SegmentSelectionChanged(selectedSegmentIndices.size());	
}

///Switch to segment drawing mode.
void QGLPointsCanvas::SegmentIntoComponents()
{
	QCheckBox *box = new QCheckBox("Use reflectance for VRML export: ",this);
	box->setChecked(true);
	
	LaserConnectedSegments segments(&laserPoints,kNN);
	segmentedVector = segments;
	PostSegmentationProcessing();
}

///Segments points into planar regions.	
void QGLPointsCanvas::SegmentIntoPlanes( )
{
	double angleThresholdDegrees = 10;
	double distanceThreshold = 1; 
	
	//Ask the user to input these values.
	angleThresholdDegrees = QInputDialog::getDouble(this, "Settings of plane segmentation", "Angle threshold (Degrees): ", angleThresholdDegrees, 0);
	distanceThreshold = QInputDialog::getDouble(this, "Settings of plane segmentation", "Distance threshold: ", distanceThreshold, 0);
	
	//Do segmentation.
	LaserPlanarSegments segments(&laserPoints,kNN,angleThresholdDegrees,distanceThreshold);
	segmentedVector = segments;
		
	PostSegmentationProcessing();
}

void QGLPointsCanvas:: SegmentDepthmapUB()
{
	float* pfDepthmap;
	BYTE* pbDepthmap;
	unsigned int* pSegmentedData;
	int nWidth,nHeight;
	int i,j,nOffset;
	
	nWidth = GetWidth();
	nHeight = GetHeight();
	
	pbDepthmap = new BYTE[nWidth*nHeight];
	pSegmentedData = new unsigned int[nWidth*nHeight];
	
	SetDrawing3DViewport();
	SetViewVolume();
	ApplyGLTransformation();
	ApplyTransform();
	glPointSize(pointSize);
	DrawLaserPoints();
	pfDepthmap = ReadDepthBuffer (nWidth, nHeight,true);
	
	for(i=0;i<nWidth*nHeight;i++)
	{
		pbDepthmap[i] = pfDepthmap[i];
	}
//	ApplyUBRangeSegmentation(pbDepthmap,pSegmentedData,nHeight,nWidth,NULL);
	
	//free resources.
	free(pfDepthmap);
	delete[] pbDepthmap;
	
		
	//Read depth buffer again.
	SetDrawing3DViewport();
	SetViewVolume();
	ApplyGLTransformation();
	ApplyTransform();
	glPointSize(pointSize);
	DrawLaserPoints();
	float* depthBuffer = ReadDepthBuffer (nWidth, nHeight, false,0);
	
	// Feedback buffer, for mapping points from depthmap to laserPoints.
	int feedbackBufferSize = laserPoints.size () * 10;

	//Return if zero elements.
	if (!feedbackBufferSize)
		return ;

	ActivateCanvas();

	GLfloat *feedbackBuffer = new GLfloat[feedbackBufferSize];

	glFeedbackBuffer (feedbackBufferSize, GL_3D_COLOR, feedbackBuffer);

	// Set the rendermode to the "select" mode
	glRenderMode (GL_FEEDBACK);
	glEnable (GL_DEPTH_TEST);
	glPointSize (pointSize);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	//Do all GL transformations.
	ApplyGLTransformation ();
	ApplyTransform();

	for (int i = 0; i < laserPoints.size (); i++)
	{
		//Note: It is necessary to use glBegin() and glEnd() for each point.
		//otherwise always one hit will occur.
		glPassThrough (i);
		glBegin (GL_POINTS);
		glVertex3f (laserPoints[i].X (), laserPoints[i].Y (),
					laserPoints[i].Z ());
		glEnd ();
	}
	glFlush ();
	int size = glRenderMode (GL_RENDER);
	
	GetSegmentedLaserIndices (size,
						feedbackBuffer,
						depthBuffer,
						pSegmentedData,
						nWidth,
						nHeight,
						segmentedVector,
						0,0,nWidth,nHeight);
	delete[] pSegmentedData;
	free(depthBuffer);

	LaserPoints data;
	data.clear();
	data.reserve(laserPoints.size());
	for(i=0;i<segmentedVector.size();i++)
	{
		IndicesVector& vec = segmentedVector[i];
		for(j=0;j<vec.size();j++)
		{
			LaserPoint pt = laserPoints[vec[j]];
			pt.Reflectance() = i+2;
			data.push_back(pt);
		}
	}
	pointsCanvasDisplayMode = pointsCanvasDisplayMode | DISPLAY_SEGMENTED_POINTS;
	SetDisplayType(pointsCanvasDisplayMode);	
}

void
QGLPointsCanvas::SetLaserPoints (const LaserPoints & pts)
{
	if((&pts)!=(&laserPoints))
		laserPoints = pts;
	
	selectedIndices.resize (0);
	if(laserPoints.size())
		laserPoints.DeriveDataBounds (0);
	
	UpdateDisplayList ();
	Repaint ();
	
	//clear the points realted data.
	laserObjects.clear();
	segmentedVector.clear();
	selectedObjectIndices.clear();
	selectedSegmentIndices.clear();
	
	//emit the signals to notify change.
	emit SegmentSelectionChanged(selectedSegmentIndices.size());
	emit PointSelectionChanged(selectedIndices.size());
	emit ObjectSelectionChanged(selectedObjectIndices.size());
}

///Sets count of k nearest neighbours.
void QGLPointsCanvas::SetKnn(int _kNN)
{
	kNN = MAX(_kNN,5);
	emit KnnChanged(kNN);
}

///Sets count of k nearest neighbours.
void QGLPointsCanvas::SetKnn()
{
	int _kNN;
	if(CheckQActionAndInt(sender(),_kNN))
		this->SetKnn(_kNN);
}

//-------------------------------------------------------------
//Macro for setting reflectance equal to distance and all normals equal to ideal ones
#define POST_PROCESS_FIT()	\
	object->Print(); cerr<<"RMS residual for this object fit is: "<< sqrt(SumSquare(object->Distance(sLaserPoints))/(double)sLaserPoints.size())<<endl;\
	if(0){for(int i=0;i<laserPoints.size();i++)\
		laserPoints[i].Reflectance() = object->DistanceFromPoint(laserPoints[i])*1e3;	\
	normalPoints.resize(laserPoints.size());  \ 
	for(int i=0;i<laserPoints.size();i++) \ 
		normalPoints[i] = object->NormalAtPoint(laserPoints[i]);  \
}	

#define POST_PROCESS_FIT() 
//-------------------------------------------------------------
//~~Macro ends
		
		

void
QGLPointsCanvas::FitTorus()
{
	LaserPoints sLaserPoints = GetSelectedLaserPoints();
	LaserObject* object = new LaserTorus(::FitTorus(sLaserPoints, ApproximateTorus(sLaserPoints)));
	laserObjects.push_back(object);
	
	POST_PROCESS_FIT();
}

void
QGLPointsCanvas::FitPlane()
{
	LaserPoints sLaserPoints = GetSelectedLaserPoints();
	LaserObject* object = new LaserPlane(::FitPlane(sLaserPoints));
	laserObjects.push_back(object);
	
	POST_PROCESS_FIT();
}

void
QGLPointsCanvas::FitSphere()
{
	LaserPoints sLaserPoints = GetSelectedLaserPoints();
	LaserObject* object = new LaserSphere(::FitSphere(sLaserPoints, ApproximateSphere(sLaserPoints)));
	laserObjects.push_back(object);
	
	POST_PROCESS_FIT();
}



void
QGLPointsCanvas::FitCylinder()
{
	LaserPoints sLaserPoints = GetSelectedLaserPoints();
	
	LaserObject* object = new LaserCylinder(::FitCylinder(sLaserPoints,ApproximateCylinder(sLaserPoints,kNN)));
	laserObjects.push_back(object);
	
	POST_PROCESS_FIT();
}

void QGLPointsCanvas::Subsample()
{
	double stepSize = QInputDialog::getDouble(this, "Enter Sub-sampling step size?", "Sub-sampling step: ", 1, 1e-12);
	//Subsample using space partition but with constant distance along each axis.
	LaserPoints pts = laserPoints.SubSample(stepSize);
	SetLaserPoints(pts);
	
	repaint();
}

void QGLPointsCanvas::MakeNormalsConsistent()
{
	vector<double> rhos;
	vector<Vector3D> consistent = ::MakeNormalsConsistent(laserPoints,rhos,kNN);
	normalPoints = consistent;
}

void QGLPointsCanvas::ExportTINVrml()
{
	//write tin to a file.
	QString fn = QFileDialog::getSaveFileName(this,
					"Save TIN to a new file"
                    ".",
                    "VRML files (*.wrl *.txt)");
                    
	if(!fn.isEmpty())
	{
		
		QCheckBox *box = new QCheckBox("Use reflectance for VRML export: ",this);
		box->setCheckState(Qt::Checked);
		
		QDialog* dlg = MakeDialog("Export TIN settings",box,NULL);
		dlg->exec();           
		bool useRef = box->checkState()& Qt::Checked;
			
		FILE* pFile = OpenVrmlFile(fn.toStdString().c_str());
		if(pFile)
		{
			LaserPointsTIN2Vrml(pFile,laserPoints,colorScheme, useRef);
			fclose(pFile);
		}
		else
		{
			ERR_MSG(QString("Failed to open file %1").arg(fn));
		}
	}
}

#if 0 	
#define MAKE_FDN_FINDER() double partitionSize,selSize;\
		cout<<"PartitionSize? ";cin>>partitionSize;\
		cout<<"Neighbourhood size?";cin>>selSize; \
		FDNFinder<LaserPoint> finder(laserPoints,partitionSize);
	else if(id==6001)
	{
	 	LaserPoints GetNormalIntersections(const LaserPoints& laserPoints,
						const vector<Vector3D>& normals,
						const int kNNBegin, const int kNNEnd, 
						const double distanceThreshold,
						const double intersectionThreshold);
		double 	distanceThreshold, intersectionThreshold;
		int kNNBegin, kNNEnd;
		
		cout<<"distanceThreshold?";cin>>distanceThreshold;
		cout<<"intersectionThreshold?";cin>>intersectionThreshold;
		cout<<"kNNBegin?";cin>>kNNBegin;
		cout<<"kNNEnd?";cin>>kNNEnd;
		LaserPoints pts = GetNormalIntersections(laserPoints,
						laserPoints.Normals(kNN),
						kNNBegin,kNNEnd,
						distanceThreshold,intersectionThreshold);
						
		//Now show in new viewer.
		QLaserViewer* viewer = new QLaserViewer(this,"Intersection of normals");
		viewer->SetLaserPoints(&pts);
		viewer->show();
	}
	
	else if(id==6000)		
	{
		//Segment using user specified thresholds.
		void
		SegmentUsingSmoothnessConstraintWithFixedThresholds(
						const LaserPoints& laserPoints,
						SegmentsVector& segmentedIndices,				
						const int kNN,
						double thresholdSeed,
						double thresholdMerge,
						double angularThresholdDegrees,
						bool useStrictNormalSimilarity);
						
		double thresholdSeed, thresholdMerge,angularThresholdDegrees;
		bool useStrictNormalSimilarity;
		
		cout<<"thresholdSeed?";cin>>thresholdSeed;
		cout<<"thresholdMerge?";cin>>thresholdMerge;
		cout<<"angularThresholdDegrees?";cin>>angularThresholdDegrees;
		cout<<"useStrictNormalSimilarity?";cin>>useStrictNormalSimilarity;
		
		segmentedVector.clear();
		SegmentUsingSmoothnessConstraintWithFixedThresholds(laserPoints,segmentedVector,				
					kNN,thresholdSeed,thresholdMerge,angularThresholdDegrees,useStrictNormalSimilarity);
					
		pointsCanvasDisplayMode = pointsCanvasDisplayMode | DISPLAY_SEGMENTED_POINTS;
		SetDisplayType(pointsCanvasDisplayMode);
		selectedSegmentIndices.clear();
		emit SegmentSelectionChanged(selectedSegmentIndices.size());
	
	}
	//Write Fdn planes to a file.
	else if(id==7002)
	{
		MAKE_FDN_FINDER();
		cout<<"Filename?"<<endl;
		string fileName;
		cin>>fileName;
		
		FILE* pFile = fopen(fileName.c_str(),"wt");
		
		if(pFile)
		{
			for(int i=0;i<laserPoints.size();i++)
			{
				vector<int> sel = finder.FindIndices(i,selSize);
				
				Vector3D normal;
				double distance = DBL_MAX;

				LaserPoints sLaserPoints = laserPoints.Select(sel);
				
				vector<int> sel_temp;
				for(int i=0;i<sLaserPoints.size();i++)
					sel_temp.push_back(i);

				Positions3D hull;
				double residual;
				if(FitPlaneToLaserPoints(sLaserPoints,sel_temp,normal,distance,&residual,&hull))
					distance = DBL_MAX;

				fprintf(pFile,"%8.6f  %8.6f  %8.6f  %8.6f  %d\n",normal[0],normal[1],normal[2],distance,sel.size());
								
				if(i%(laserPoints.size()/100)==0)
					fprintf(stderr,"Fdn planes: %d/%d         %4.1f%%\r",i,laserPoints.size(),(double)(i)/laserPoints.size()*100.00);
			}
			fclose(pFile);
			cerr<<"Done with writing fdn planes to "<<fileName<<endl;
		}
		else
		{
			cerr<<"Failed to open file: "<<fileName<<endl;
		}
	}
	//Write Fdn normals to a file.
	else if(id==7001)
	{
		MAKE_FDN_FINDER();
		cout<<"Filename?"<<endl;
		string fileName;
		cin>>fileName;
		
		FILE* pFile = fopen(fileName.c_str(),"wt");
		
		if(pFile)
		{
			for(int i=0;i<laserPoints.size();i++)
			{
				vector<int> sel = finder.FindIndices(i,selSize);
				
				Vector3D normal = laserPoints.Select(sel).Normal();
				for(int k=0;k<3;k++)
					fprintf(pFile,"%8.6f  ",normal[k]);
				fprintf(pFile,"\n");
				
				if(i%(laserPoints.size()/100)==0)
					fprintf(stderr,"Fdn normals: %d/%d         %4.1f%%\r",i,laserPoints.size(),(double)(i)/laserPoints.size()*100.00);
			}
			fclose(pFile);
			cerr<<"Done with writing fdn normals to "<<fileName<<endl;
		}
		else
		{
			cerr<<"Failed to open file: "<<fileName<<endl;
		}
	}
	//Write Fdn neighbours to a file.
	else if(id==7000)
	{
		MAKE_FDN_FINDER();
		cout<<"Filename?"<<endl;
		string fileName;
		cin>>fileName;
		
		FILE* pFile = fopen(fileName.c_str(),"wt");
		
		if(pFile)
		{
			for(int i=0;i<laserPoints.size();i++)
			{
				vector<int> sel = finder.FindIndices(i,selSize);
				for(int k=0;k<sel.size();k++)
					fprintf(pFile,"%d ",sel[k]);
				fprintf(pFile,"\n");
				
				if(i%(laserPoints.size()/100)==0)
					fprintf(stderr,"%d/%d         %4.1f%%\r",i,laserPoints.size(),(double)(i)/laserPoints.size()*100.00);
			}
			fclose(pFile);
			cerr<<"Done with writing fdn's to "<<fileName<<endl;
		}
		else
		{
			cerr<<"Failed to open file: "<<fileName<<endl;
		}
	}
	//Set scan id.	
	else if(id==8001)
	{
	 	cout<<"New scan id?";
		int id;
		cin>>id;
		
		for(int i=0;i<laserPoints.size();i++)
			laserPoints[i].SetId(id);
	}
	else if(id==8000)
	{
		QRegistrationInterface *reg = new QRegistrationInterface(this);
		
		//Load first point cloud.
		QString fn1 = QFileDialog::getOpenFileName(".",
                    "laser point files (*.laser *.laser.gz *.pts *.txt)",
                    this,
                    "open first point cloud",
                    "Choose a file to open" );
					
		//Load first point cloud.
		QString fn2 = QFileDialog::getOpenFileName(".",
                    "laser point files (*.laser *.laser.gz *.pts *.txt)",
                    this,
                    "open second point cloud",
                    "Choose a file to open" );
  		LaserPoints* pts1 = new LaserPoints((char*)(fn1.toStdString().c_str()));
		Orientation3D* or1 = new Orientation3D;
		LaserPoints* pts2 = new LaserPoints((char*)(fn2.toStdString().c_str()));;
		Orientation3D* or2 = new Orientation3D;
		
		LaserScan* s1 = new LaserScan(pts1,or1)	;	
		LaserScan* s2 = new LaserScan(pts2,or2);		
		
		reg->SetScans(s1,s2);
		reg->show();
	
	}
	else if(id==4000)		
	{
		//Save normals with residuals.
		QString fn = QFileDialog::getSaveFileName(
                    ".",
                    "normals(*.pts *.txt *.ascii)",
                    this,
                    "save normals+residuals to new ascii point cloud file",
                    "Choose a file to Save" );
  
		if ( !fn.isEmpty() ) 
		{
			FILE* pFile = fopen((char*)(fn.toStdString().c_str()),"wt");
			
			if(!pFile)
			{
				cerr<<"Failed to save to "<<fn<<endl;
				return;
			}
			vector<double> errorVector;
			vector<double> distances;
			vector<double> areas;

			//Calculate normals.
			CalculateLaserPointsNormalsANN(laserPoints,normalPoints,distances,areas,errorVector,kNN);
			
			for(int i=0;i<normalPoints.size();i++)
			{
				fprintf(pFile,"%9.6f  %9.6f  %9.6f  %9.6f\n",normalPoints[i][0],normalPoints[i][1],normalPoints[i][2],errorVector[i]);
			}
			fclose(pFile);
		}
		
	}
	else if(id==3000)
	{
		VrmlFile segmentation("segmentation.wrl");
		segmentation.Write(laserPoints,segmentedVector,0);
	}	
	//Show normal of each segment on a gaussian sphere. Its reflectance is equal
	//to the count of points.
	else if(id==2006)
	{
		if(segmentedVector.empty())
			return;
		
		LaserPoints segGSphere;
		for(int i=0;i<segmentedVector.size();i++)
		{
			if(segmentedVector[i].size()<20)
				continue;
			LaserPoint pt = laserPoints.Normal(segmentedVector[i]);
			pt.Reflectance() = segmentedVector[i].size();
			segGSphere.push_back(pt);
		}
		
		//Now show in viewer
		QLaserViewer* viewer = new QLaserViewer(this,"Normal of segments shown on Gaussian sphere");
		viewer->SetLaserPoints(&segGSphere);
		viewer->show();
	}
	//Give random color to objects.
	else if(id==2007)
	{
		for(int i=0;i<laserObjects.size();i++)
		{
			laserObjects[i]->SetColor(GenerateRandomColor());
		}
	}
		
	//action = expMenu->insertItem("Show residuals",this,SLOT(ExperimentalFunctions(int)));
	//expMenu->setItemParameter(id,2005);
	else if(id==2005)
	{
		//Show residuals as reflectance.
		vector<double> errorVector;
		vector<double> distances;
		vector<double> areas;
		
		
		CalculateLaserPointsNormalsANN(laserPoints,
				normalPoints,
				distances,
				areas,
				errorVector,
				kNN);
				
					
		//Show residuals as another point cloud in a new viewer.
		LaserPoints pts = laserPoints;
	
		double dMax = *(max_element(errorVector.begin(),errorVector.end()));
		double dMin = *(min_element(errorVector.begin(),errorVector.end()));
		double diff = (dMax-dMin)?(dMax-dMin):1;
		for(int i=0;i<pts.size();i++)
			pts[i].Reflectance() = 1e4*(errorVector[i]-dMin)/diff;
	
		//	Now show in new viewer.
		QLaserViewer* viewer = new QLaserViewer(this,"ResidualsOfNormalEstimation");
		viewer->SetLaserPoints(&pts);
		viewer->show();
	}
	else if(id==786)
	{
		
		cerr<<"Acis is not turned on for the time being\n";
	}
	else if(id==787)
	{
		LaserPoints sel;
		GetSelectedLaserPoints(sel);
		//Fit quadric to selected points.
		double EigValues[3];
		Vector3D EigVectors[3];
		vector<double> result = sel.FitQuadric(EigVectors,EigValues);
		for(int i=0;i<result.size();i++)
		{
			//cerr<<"q["<<i<<"] = "<<result[i]<<"\n";		
			
		}
		for(int i=0;i<3;i++)
			cerr<<"EigValue: "<<EigValues[i]<<": "<<EigVectors[i]<<endl;	
	}
	else if(id ==788)
	{
		SegmentIntoQuadrics(laserPoints,segmentedVector,kNN);			
	}
	else if(id==1000)
	{
		QObjectBasedRegistration* reg = new QObjectBasedRegistration(
		" "," "," ", " ");
		/*
		"/nfs/data_ge0pc208/Erlangen/laser/scan_ss/Scan1.laser",
		"/nfs/data_ge0pc208/Erlangen/models/Scan1.obj",
		"/nfs/data_ge0pc208/Erlangen/laser/scan_ss/Scan2.laser",
		"/nfs/data_ge0pc208/Erlangen/models/Scan2.obj");
		*/
		
		/*
		"/nfs/data_ge0pc208/BoilerRoomData/laser/scan1.laser",
		"/nfs/data_ge0pc208/BoilerRoomData/models/scan1.obj",
		"/nfs/data_ge0pc208/BoilerRoomData/laser/scan4.laser",
		"/nfs/data_ge0pc208/BoilerRoomData/models/scan4.obj");
		*/
		
		reg->show();
	}
	else if(id==1001)
	{
		//Add a sphere, without fitting just as a registration target.
		LaserPoints sLaserPoints;
		GetSelectedLaserPoints(sLaserPoints);
		
		Vector3D position = sLaserPoints.Mean();
		vector<double> extents = sLaserPoints.Extents();
		double radius = *max_element(extents.begin(),extents.end());
	
		LaserSphere* object = new LaserSphere(position,radius,GenerateRandomColor());
		object->SetIndices(sLaserPoints.GetIndices());
		laserObjects.push_back(object);
	
	}
	else if(id==1002)
	{
		//Invert the normal of the plane.
		if(!selectedObjectIndices.empty() 
			&& laserObjects[selectedObjectIndices[0]]->Name() == "LaserPlane")
		{
			LaserPlane* pLaserPlane = (LaserPlane*)(laserObjects[selectedObjectIndices[0]]);
			pLaserPlane->plane.Normal() = pLaserPlane->plane.Normal() * (-1);
			pLaserPlane->plane.Distance() = pLaserPlane->plane.Distance() * (-1);
		}
	}
	else if(id==1003)
	{
		//Print the parmeters of the object.
		if(selectedObjectIndices.size())
		{
			for(int i=0;i<	selectedObjectIndices.size();i++)
				laserObjects[selectedObjectIndices[i]]->Print();
		}
	}
	else if(id==1004)
	{
		//Reflectance should show distance from object.
		if(selectedObjectIndices.size())
		{
			//Calculate distance from the first object to all points
			LaserObject* object = laserObjects[selectedObjectIndices[0]];
			vector<double> distances = object->Distance(laserPoints);
			
			//Set as reflectance.
			laserPoints.SetReflectance(distances);
			UpdateDisplayList();Repaint();
			
			//Print some stats
			cerr<<"Showing distance from: ";
			object->Print();
			cerr<<"min: "<<Min(distances)<<" max: "<<Max(distances)<<" range: "<<Range(distances)<<endl;
		}
	}
	
	else if(id==400)
	{
		if(segmentedVector.empty())
			return;
		QString fn = QFileDialog::getSaveFileName(
                    ".",
                    "laser segmentation files (*.segments *.txt)",
                    this,
                    "Save point cloud segmentation",
                    "Choose a file to save" );
		if ( !fn.isEmpty() ) 
		{
			LaserSegments temp(&laserPoints);
			
			temp = segmentedVector;
			temp.Save((char*)(fn.toStdString().c_str()));
		}
		
#if 0		
		system("mkdir segmentation");
		
		for(int i=0;i<segmentedVector.size();i++)
		{
			char buffer[4096];
			sprintf(buffer,"segmentation/seg_%09d.pts",i);
			FILE* pFile = fopen(buffer,"wt");
			
			if(pFile)
			{
				for(int k=0;k<segmentedVector[i].size();k++)
				{
					int index = segmentedVector[i][k];
					LaserPoint pt = laserPoints[index];
					fprintf(pFile,"%9.6f  %9.6f %9.6f %d\n",pt[0],pt[1],pt[2],index);
				}
				fclose(pFile);
			}
		}
		fprintf(stderr,"Saved segmentation to directory segmentation\\ \n");
#endif		

	}
	else if(id==401)
	{
		QString fn = QFileDialog::getOpenFileName(
                    ".",
                    "laser segmentation files (*.segments *.txt)",
                    this,
                    "open new point cloud segmentation",
                    "Choose a file to open" );
		if ( !fn.isEmpty() ) 
		{
			LaserSegments temp(&laserPoints);
			temp.Load((char*)(fn.toStdString().c_str()));
			
			segmentedVector = temp;
			
			pointsCanvasDisplayMode = pointsCanvasDisplayMode | DISPLAY_SEGMENTED_POINTS;
			SetDisplayType(pointsCanvasDisplayMode);
			selectedSegmentIndices.clear();

			emit SegmentSelectionChanged(selectedSegmentIndices.size());
		}
	}
	else if(id==1) //Grow regions.
	{
#if 1	
		vector<int>
		GrowFromSeed(const LaserPoints& laserPoints,
			int seedPointIndex,
			double maxAngleInDegrees,
			const int kNN,
			bool breakOnMaxAngle=true);
			
		vector<vector<int> >
		SplitIntoRegions(const LaserPoints& laserPoints,
			double maxAngleInDegrees,
			const int kNN,
			bool breakOnMaxAngle=true);
			
			
		double maxAngle = 30;
		cout<<"maxAngle?"<<endl;
		cin>>maxAngle;
#if 0		
		cerr<<"Starting GrowFromSeed with maxAngle "<<maxAngle<<" deg and kNN "<<kNN<<"..."<<flush;
		selectedIndices = GrowFromSeed(laserPoints,	selectedIndices[0],maxAngle,kNN,colorMappingMode==UseReflectance);
#else
		double maxDistance = 30;
		cout<<"maxDistance?"<<endl;
		cin>>maxDistance;
		cerr<<"Starting GrowFromSeed with maxAngle "<<maxAngle<<" deg and max distance: "<<maxDistance<<" and kNN "<<kNN<<"..."<<flush;
		vector<int>
		selectedIndices = GrowFromSeed(laserPoints,
			normalPoints,
			selectedIndices[0],
			maxAngle,
			maxDistance,//In terms of distance from central point.
			kNN);

#endif		
		
		//segmentedVector = SplitIntoRegions(laserPoints,60,kNN);
		pointsCanvasDisplayMode = pointsCanvasDisplayMode | DISPLAY_SEGMENTED_POINTS;
		SetDisplayType(pointsCanvasDisplayMode);
		cerr<<"Done\n"<<flush;
#endif		
	}
	else if(id==10)
	{
		double start = clock();
		MAKE_FDN_FINDER();
		vector<int> count = finder.GetFdnCount(selSize);
		double stop = clock();
		cerr<<"Box based lookup time: "<<stop-start<<endl<<endl;
		
		/*
		
		start = clock();
		count = finder.GetFdnCountIndirect(selSize,count);
		stop = clock();
		cerr<<"Direct KNN based lookup time: "<<stop-start;
		*/
		
		
		
		laserPoints.SetReflectance(count);
	}
	else if(id==9)
	{
		//Select Fdn
		MAKE_FDN_FINDER();
		selectedIndices = finder.FindIndices(selectedIndices[0],selSize);
	}
	else if(id ==5)
	{
		//action = expMenu->insertItem("Calculate curvature Fdn",this,SLOT(ExperimentalFunctions(int)));
		MAKE_FDN_FINDER();
				
		vector<double> majorCurvatures, minorCurvatures;
		vector<Vector3D> majorDirections, minorDirections;
	
		finder.CalculateCurvatures(selSize,&majorCurvatures,&minorCurvatures,
					&majorDirections, &minorDirections);
	
		
		normalPoints = minorDirections;//finder.CalculateNormals(selSize);
		laserPoints.SetReflectance(majorCurvatures);
	
	}
	else if(id==6)
	{
		//action = expMenu->insertItem("Calculate normals Fdn",this,SLOT(ExperimentalFunctions(int)));
		MAKE_FDN_FINDER();	
		normalPoints = finder.CalculateNormals(selSize);
					
	}
	else if(id==7)
	{
		//action = expMenu->insertItem("Show normal points",this,SLOT(ExperimentalFunctions(int)));
		QLaserViewer* viewer = new QLaserViewer(this,"new viewer");
		viewer->SetLaserPoints(&normalPoints);
		viewer->show();
	}
	
	else if(id==2)
	{
		if(segmentedVector.size())
		{
			for(int i=0;i<segmentedVector.size();i++)
			{
				LaserPoints remaining = DetectPlanesAndCylindersUsingHough(laserPoints.Select(segmentedVector[i]),kNN,
							&laserObjects,NULL,false,false);
			}
		}
		else
		{
			LaserPoints sel;
			GetSelectedLaserPoints(sel);
			LaserPoints remaining = DetectPlanesAndCylindersUsingHough(sel,kNN,
								&laserObjects,NULL,false,false);
		}
	}
	else if(id == 3)
	{
		int oldSize = laserObjects.size();
		//Only plane detection.
		if(segmentedVector.size())
		{
			for(int i=0;i<segmentedVector.size();i++)
			{
				if(segmentedVector[i].size() < 2*kNN)
					continue;
				
				LaserPoints remaining = DetectPlaneUsingHough(laserPoints.Select(segmentedVector[i]),kNN,
							&laserObjects,NULL,false,false);
							
				int iter = 1;
				cerr<<"---remaining size is: "<<remaining.size()<<endl;
				while(remaining.size()>kNN && iter<10)
				{
					remaining = DetectPlaneUsingHough(remaining,kNN,
								&laserObjects,NULL,false,false);
					iter++;
				}
			}
		}
		else
		{
			LaserPoints sel;
			GetSelectedLaserPoints(sel);
			LaserPoints remaining = DetectPlaneUsingHough(sel,kNN,
								&laserObjects,NULL,false,false);
								
			int iter = 1;
			cerr<<"---remaining size is: "<<remaining.size()<<endl;
			while(remaining.size()>kNN && iter<10)
			{
				remaining = DetectPlaneUsingHough(remaining,kNN,
							&laserObjects,NULL,false,false);
				iter++;
			}
		}
		cerr<<"\nFinished Hough plane.\nold objects: "<<oldSize
			<<" new objects: "<<laserObjects.size()<<endl;
	
	}
	else if(id == 4)
	{
		//Fit plane to all segments greater than 2*kNN
		int oldSize = laserObjects.size();
		//Only plane detection.
		for(int i=0;i<segmentedVector.size();i++)
		{
			if(segmentedVector[i].size() > 2*kNN)
			{
				selectedIndices = segmentedVector[i];
				FitPlane();
			}
		}
		cerr<<"\nFinished Hough plane.\nold objects: "<<oldSize
			<<" new objects: "<<laserObjects.size()<<endl;
	
	}
	
}
#endif

///This slot handles menu items which are still experimental.
void QGLPointsCanvas::ExperimentalFunctions(int id)
{
	
}

void
QGLPointsCanvas::ShowPopupMenu ()
{
	UpdateActions();
	QMenu *rootMenu = new QMenu(this);
	rootMenu->addMenu(this->fileMenu);	
	rootMenu->addMenu(this->viewMenu);
	rootMenu->addMenu(this->selectionMenu);		
	rootMenu->addMenu(this->editMenu);	
	rootMenu->addMenu(this->fitMenu);	
	rootMenu->addMenu(this->segmentationMenu);	
	rootMenu->addMenu(this->settingsMenu);	
	rootMenu->addMenu(this->helpMenu);	
	rootMenu->exec(QCursor::pos());
	delete rootMenu;
	return;
}

void QGLPointsCanvas::CopyObjects()
{
	//Clear the old objects.
	Delete(GlobalCopiedObjects);
	GlobalCopiedObjects.clear();
	
	//copy the selected objects
	for(int i=0;i<selectedObjectIndices.size();i++)
		GlobalCopiedObjects.push_back(laserObjects[selectedObjectIndices[i]]->Clone());
		
	emit CopiedObjectsChanged(GlobalCopiedObjects.size());
	
	UpdateActions();
}
void QGLPointsCanvas::PasteObjects()
{
	for(int i=0;i<GlobalCopiedObjects.size();i++)
		laserObjects.push_back(GlobalCopiedObjects[i]);
	GlobalCopiedObjects.clear();
	
	emit CopiedObjectsChanged(GlobalCopiedObjects.size());
	
	UpdateActions();
	Repaint();
}
void QGLPointsCanvas::CopyPoints()
{
	GetSelectedLaserPoints(GlobalCopiedPoints);
}

void QGLPointsCanvas::PastePoints()
{
	laserPoints.reserve(laserPoints.size()+GlobalCopiedPoints.size());

	for(int i=0;i<GlobalCopiedPoints.size();i++)
		laserPoints.push_back(GlobalCopiedPoints[i]);
	
	SetLaserPoints(laserPoints);
}



void QGLPointsCanvas::SelectAll()
{
	selectedIndices.resize(laserPoints.size());
	
	for(int i=0;i<laserPoints.size();i++)
	{
		selectedIndices[i]=i;
	}
	Repaint();
	
	emit PointSelectionChanged(selectedIndices.size());
}

void QGLPointsCanvas::DeselectAll()
{
	selectedIndices.resize(0);
	
	emit PointSelectionChanged(selectedIndices.size());
	
	Repaint();
}

void QGLPointsCanvas::Crop()
{
	LaserPoints temp;
	temp.resize(selectedIndices.size());
	
	for(int i=0;i<selectedIndices.size();i++)
	{
		temp[i]=laserPoints[selectedIndices[i]];
	}
	SetLaserPoints(temp);
}


void QGLPointsCanvas::InvertSelection()
{
	IndicesSet selectedSet(selectedIndices.begin(),selectedIndices.end());
	IndicesVector totalIndices;
	
	//Make a vector of all indices.
	totalIndices.resize(laserPoints.size());
	for(int i=0;i<laserPoints.size();i++)
	{
		totalIndices[i]=i;
	}
	IndicesSet totalSet(totalIndices.begin(),totalIndices.end());
	
	//Take the difference to get inverted selection.
	IndicesSet invertedSet;
	insert_iterator<IndicesSet> resultIter(invertedSet,invertedSet.begin());
	set_difference(totalSet.begin(),totalSet.end(),selectedSet.begin(),selectedSet.end(),resultIter);
	
	//Copy the difference set to selectedIndices.
	IndicesSet::const_iterator iter = invertedSet.begin();
	selectedIndices.reserve(invertedSet.size());
	selectedIndices.clear();
	
	for(;iter!=invertedSet.end();iter++)
		selectedIndices.push_back(*iter);
		
	emit PointSelectionChanged(selectedIndices.size());
		
	Repaint();
}
int QGLPointsCanvas::GetSelectedLaserPoints(LaserPoints& selectedPoints)
{
	if(selectedIndices.empty())
		SelectAll();
		
	selectedPoints = laserPoints.Select(selectedIndices);
	
	return selectedPoints.size();
}

LaserPoints QGLPointsCanvas::GetSelectedLaserPoints()
{
	LaserPoints sLaserPoints;
	GetSelectedLaserPoints(sLaserPoints);
	
	return sLaserPoints;
}


LaserPoints & QGLPointsCanvas::GetLaserPoints ()
{
	return laserPoints;
}


void
QGLPointsCanvas::SetButtonPressCallback (UserDefinedCallback Callback,
										   void *userData)
{
	buttonPressCallback = Callback;
	buttonPressUserData = userData;
}

void
QGLPointsCanvas::SetDisplayType (int d)
{
	pointsCanvasDisplayMode = d;
	
	if((d&DISPLAY_SEGMENTED_POINTS) && segmentedVector.empty())
	{
		QMessageBox::information(this,"Information","The segmentation results are empty. \nSwitching back to data point display.");
		d = d ^(DISPLAY_SEGMENTED_POINTS);
		pointsCanvasDisplayMode = d|DISPLAY_DATA_POINTS;
	}
	
	UpdateDisplayList ();
	Repaint ();
}

void
QGLPointsCanvas::SetColorMappingMode(int bUse)
{
	colorMappingMode = ColorMappingMode(bUse);
	UpdateDisplayList ();
	
	emit ColorMappingModeChanged(colorMappingMode);
	
	Repaint ();
}

int
QGLPointsCanvas::GetColorMappingMode ()
{
	return (int)(colorMappingMode);
}

int
QGLPointsCanvas::WindowSelectPoints (int x1, int y1, int width, int height,IndicesVector& selectedIndices,bool bSinglePointSelection)
{

	DEBUG("QGLPointsCanvas::WindowSelect\n");
	// Initializing buffer in which selected points are stored
	int selectionBufferSize = laserPoints.size () * 4;
	float *depthBuffer = NULL;
	int x, y, w, h;

	//Return if zero elements.
	if (!selectionBufferSize)
		return -1;

	ActivateCanvas();
	
	if(bDepthSensitiveSelection)
	{
		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		SetDrawing3DViewport();
		SetViewVolume();
		ApplyGLTransformation();
		ApplyTransform();
		glPointSize(pointSize);
		DrawLaserPoints();
		GetGeometry (x, y, w, h);
		depthBuffer = ReadDepthBuffer (w, h, false,0);
	}

	GLuint *selectionBuffer = new GLuint[selectionBufferSize];
	glSelectBuffer (selectionBufferSize, selectionBuffer);

	GLuint i, hits, picked;
	// Set the rendermode to the "select" mode
	glRenderMode (GL_SELECT);
	glEnable (GL_DEPTH_TEST);
	glPointSize (pointSize);

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Initilize the name stack
	glInitNames ();
	glPushName (0);

	//Do all GL transformations.
	ApplyGLTransformationForPicking (x1, y1, width, height);


	for (int i = 0; i < laserPoints.size (); i++)
	{
		//Note: It is necessary to use glBegin() and glEnd() for each point.
		//otherwise always one hit will occur.
		glLoadName (i);
		glBegin (GL_POINTS);
		glVertex3f (laserPoints[i].X (), laserPoints[i].Y (),
					laserPoints[i].Z ());
		glEnd ();
	}
	glFlush ();

	hits = glRenderMode (GL_RENDER);
	if (hits <= 0)
	{
		DEBUG ("Nothing selected\n");
	}
	else
	{
		DEBUG("Selection hits = %d\n", hits);
	}

	selectedIndices.reserve (hits);
	selectedIndices.resize (0);
	// Loop over the points selected
	int maxZ = -1;
	int minZ = -1;
	for (i = 1; i <= hits; i++)
	{
		// Get the selected point id's from the buffer
		picked = selectionBuffer[(i - 1) * 4 + 3];
		DEBUG ("Selected point number %d\n", i - 1);
		
		
		if(bSinglePointSelection)
		{
			int names = selectionBuffer[(i-1)*4 + 0 ];
			int z_min = selectionBuffer[(i-1)*4 + 1 ];
			int z_max = selectionBuffer[(i-1)*4 + 2 ];
			if(minZ<0 || minZ>z_max)
			{
				selectedIndices.resize(1);
				selectedIndices[0] = picked;
				minZ = z_max;
			}
		}
		else
			selectedIndices.push_back (picked);
		
	}
	delete[]selectionBuffer;
	glFlush ();

	if(bDepthSensitiveSelection)
	{
		// Initializing buffer in which selected points will be stored
		int feedbackBufferSize = laserPoints.size () * 10;

		//Return if zero elements.
		if (!feedbackBufferSize)
			return -1;

		ActivateCanvas();

		GLfloat *feedbackBuffer = new GLfloat[feedbackBufferSize];



		glFeedbackBuffer (feedbackBufferSize, GL_3D_COLOR, feedbackBuffer);

		// Set the rendermode to the "select" mode
		glRenderMode (GL_FEEDBACK);
		glEnable (GL_DEPTH_TEST);
		glPointSize (pointSize);
		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//Do all GL transformations.
		ApplyGLTransformation ();
		ApplyTransform();

		for (int i = 0; i < laserPoints.size (); i++)
		{
			//Note: It is necessary to use glBegin() and glEnd() for each point.
			//otherwise always one hit will occur.
			glPassThrough (i);
			glBegin (GL_POINTS);
			glVertex3f (laserPoints[i].X (), laserPoints[i].Y (),
						laserPoints[i].Z ());
			glEnd ();
		}
		glFlush ();
		int size = glRenderMode (GL_RENDER);
		selectedIndices.resize (0);
		AnalyzeFeedbackAndDepthBuffer (size, feedbackBuffer, depthBuffer, w,h,
									   selectedIndices, x1, h - y1 - height,
									   x1 + width, h - y1);

		DEBUG("\n*************************\n");
		DEBUG("hits: %d  size = %d \n", hits, size);

		delete[]feedbackBuffer;
		free (depthBuffer);
	}
	return 0;
}

int
QGLPointsCanvas::FreehandSelectPoints (IndicesVector& selectedIndices)
{

	DEBUG("QGLPointsCanvas::FreehandSelectPoints\n");
	// Initializing buffer in which selected points are stored
	int selectionBufferSize = laserPoints.size () * 4;
	float *depthBuffer = NULL, *freehandBuffer=NULL;
	int x, y, w, h;
	float xMin,yMin,xMax,yMax;
	int x1,y1,width,height;
	
	//Return if zero elements.
	if (!selectionBufferSize)
		return -1;

	ActivateCanvas();
	glEnable(GL_DEPTH_TEST);
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	SetDrawing3DViewport();
	ApplyGLTransformation();
	ApplyTransform();
	DrawLaserPointsGL(laserPoints,0,colorScheme,1);
	GetGeometry (x, y, w, h);
	depthBuffer = ReadDepthBuffer (w, h, false,0);
	
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();
	
	if(colorbarOn)
		glOrtho (0, w*(1.00-colorbarRatioX), 0, h,-1000,1000);
	else
		glOrtho (0, w, 0, h,-1000,1000);

	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
	glColor3f(1.0,1.0,1.0);
	
	glBegin (GL_POLYGON);
		for(int i=0;i<xFreehandPoints.size();i++)
			glVertex3f (xFreehandPoints[i],yFreehandPoints[i],100);
	glEnd ();
	
	freehandBuffer = ReadDepthBuffer(w,h,false,0);
	
	bool debugging = false;
	LaserPoints pts;
	
	if(debugging)
	{
		pts.resize(w*h);
		for(int i=0;i<w*h;i++)
			pts[i] = LaserPoint(i%w,i/w,0,freehandBuffer[i]*100);
		ShowLaserPoints(pts,"freehandBuffer");
	
		for(int i=0;i<w*h;i++)
			pts[i] = LaserPoint(i%w,i/w,0,depthBuffer[i]*100);
		ShowLaserPoints(pts,"depthBuffer_b4");
	}

	
	for(int i=0;i<w*h;i++)
	{
		if(freehandBuffer[i]==1)
		{
			depthBuffer[i]=1;
		}
		else if(!bDepthSensitiveSelection)
			depthBuffer[i] = 0.5;
	}
	free(freehandBuffer);
	
	if(debugging)
	{
		pts.resize(w*h);
		for(int i=0;i<w*h;i++)
			pts[i] = LaserPoint(i%w,i/w,0,depthBuffer[i]*100);
		ShowLaserPoints(pts,"depthBuffer_after");
	}

			
	xMin = yMin = 1e7;
	xMax = yMax = -1e7;
	for(int i=0;i<xFreehandPoints.size();i++)
	{
		xMin = MIN(xMin,xFreehandPoints[i]);
		yMin = MIN(yMin,yFreehandPoints[i]);
		xMax = MAX(xMax,xFreehandPoints[i]);
		yMax = MAX(yMax,yFreehandPoints[i]);
	}
	x1 = xMin;
	width = xMax - xMin;
	y1 = h - yMax;
	height = yMax - yMin;

	
	GLuint *selectionBuffer = new GLuint[selectionBufferSize];
	glSelectBuffer (selectionBufferSize, selectionBuffer);
	
	GLuint i, hits, picked;
	// Set the rendermode to the "select" mode
	glRenderMode (GL_SELECT);
	glEnable (GL_DEPTH_TEST);
	glPointSize (pointSize);

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Initilize the name stack
	glInitNames ();
	glPushName (0);

	//Do all GL transformations.
	ApplyGLTransformationForPicking (x1, y1, width, height);


	for (int i = 0; i < laserPoints.size (); i++)
	{
		//Note: It is necessary to use glBegin() and glEnd() for each point.
		//otherwise always one hit will occur.
		glLoadName (i);
		glBegin (GL_POINTS);
		glVertex3f (laserPoints[i].X (), laserPoints[i].Y (),
					laserPoints[i].Z ());
		glEnd ();
	}
	glFlush ();

	hits = glRenderMode (GL_RENDER);
	if (hits <= 0)
	{
		DEBUG ("Nothing selected\n");
	}
	else
	{
		DEBUG("Selection hits = %d\n", hits);
	}

	selectedIndices.reserve (hits);
	selectedIndices.resize (0);
	// Loop over the points selected
	for (i = 1; i <= hits; i++)
	{
		// Get the selected point id's from the buffer
		picked = selectionBuffer[(i - 1) * 4 + 3];
		DEBUG ("Selected point number %d\n", i - 1);
		selectedIndices.push_back (picked);
	}
	delete[]selectionBuffer;
	glFlush ();

	if(bDepthSensitiveSelection || 1)
	{
		// Initializing buffer in which selected points will be stored
		int feedbackBufferSize = laserPoints.size () * 10;

		//Return if zero elements.
		if (!feedbackBufferSize)
			return -1;

		ActivateCanvas();

		GLfloat *feedbackBuffer = new GLfloat[feedbackBufferSize];

		glFeedbackBuffer (feedbackBufferSize, GL_3D_COLOR, feedbackBuffer);

		// Set the rendermode to the "select" mode
		glRenderMode (GL_FEEDBACK);
		glEnable (GL_DEPTH_TEST);
		glPointSize (pointSize);
		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//Do all GL transformations.
		SetDrawing3DViewport();
		ApplyGLTransformation ();
		ApplyTransform();

		for (int i = 0; i < laserPoints.size (); i++)
		{
			//Note: It is necessary to use glBegin() and glEnd() for each point.
			//otherwise always one hit will occur.
			glPassThrough (i);
			glBegin (GL_POINTS);
			glVertex3f (laserPoints[i].X (), laserPoints[i].Y (),
						laserPoints[i].Z ());
			glEnd ();
		}
		glFlush ();
		int size = glRenderMode (GL_RENDER);
		selectedIndices.resize (0);
		AnalyzeFeedbackAndDepthBuffer (size, feedbackBuffer, depthBuffer, w,h,
									   selectedIndices, x1, h - y1 - height,
									   x1 + width, h - y1,bDepthSensitiveSelection?0.1:100);

		DEBUG("\n*************************\n");
		DEBUG("hits: %d  size = %d \n", hits, size);

		delete[]feedbackBuffer;
		free (depthBuffer);
	}
	return 0;
}
	
	
	

int
QGLPointsCanvas::WindowSelectSegments (int x1, int y1, int width, int height,IndicesVector& selectedSegmentIndices)
{

#define ENABLE_DEBUGGING 0
#include "DebugFunc.h"
	const bool bAnalyzeDepthBuffer = false;
	DEBUG("QGLPointsCanvas::WindowSelectSegments\n");
	// Initializing buffer in which selected points are stored
	int selectionBufferSize = laserPoints.size () * 4;
	float *depthBuffer = NULL;
	int x, y, w, h;
	int index;
	GLuint i,j, hits, picked;

	//Return if zero elements.
	if (!selectionBufferSize)
		return -1;

	ActivateCanvas();
	
	if(bAnalyzeDepthBuffer)
	{
		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		SetDrawing3DViewport();
		SetViewVolume();
		ApplyGLTransformation();
		ApplyTransform();
		glPointSize(pointSize);
		DrawLaserPoints();
		GetGeometry (x, y, w, h);
		depthBuffer = ReadDepthBuffer (w, h, false,0);
		LaserPoints pts;
		pts.resize(w*h);
		for(i=0; i<w*h;i++)
		{
			pts[i]=LaserPoint(i%w,i/w,depthBuffer[i]);
			pts[i].Reflectance()=depthBuffer[i]*255;			
		}
		//For debugging show buffer in a new window.
		if(0)
		{
			ShowLaserPoints(pts,"Analyze_depth_buffer");
		}
		
	}

	GLuint *selectionBuffer = new GLuint[selectionBufferSize];
	glSelectBuffer (selectionBufferSize, selectionBuffer);

	// Set the rendermode to the "select" mode
	glRenderMode (GL_SELECT);
	glEnable (GL_DEPTH_TEST);
	glPointSize (pointSize);

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Initilize the name stack
	glInitNames ();
	glPushName (0);

	//Do all GL transformations.
	ApplyGLTransformationForPicking (x1, y1, width, height);


	for (i = 0; i < segmentedVector.size (); i++)
	{
		//Note: It is necessary to use glBegin() and glEnd() for each point.
		//otherwise always one hit will occur.
		glLoadName (i);
		glBegin (GL_POINTS);
		IndicesVector& v = segmentedVector[i];
		for(j=0;j<v.size();j++)
		{
			index = v[j];
			glVertex3f (laserPoints[index].X (), laserPoints[index].Y (),laserPoints[index].Z ());
		}
		glEnd ();
	}
	glFlush ();

	hits = glRenderMode (GL_RENDER);
	if (hits <= 0)
	{
		DEBUG ("Nothing selected\n");
	}
	else
	{
		DEBUG("Selection hits = %d\n", hits);
	}

	selectedSegmentIndices.reserve (hits);
	selectedSegmentIndices.resize (0);
	// Loop over the points selected
	for (i = 1; i <= hits; i++)
	{
		// Get the selected point id's from the buffer
		picked = selectionBuffer[(i - 1) * 4 + 3];
		DEBUG("Selected segment number %d\n", picked);
		selectedSegmentIndices.push_back(picked);

	}
	delete[]selectionBuffer;
	glFlush ();

	if(bAnalyzeDepthBuffer)
	{
		// Initializing buffer in which selected points will be stored
		int feedbackBufferSize = laserPoints.size () * 10;

		//Return if zero elements.
		if (!feedbackBufferSize)
			return -1;

		ActivateCanvas();

		GLfloat *feedbackBuffer = new GLfloat[feedbackBufferSize];

		glFeedbackBuffer (feedbackBufferSize, GL_3D_COLOR, feedbackBuffer);

		// Set the rendermode to the "select" mode
		glRenderMode (GL_FEEDBACK);
		glEnable (GL_DEPTH_TEST);
		glPointSize (pointSize);
		glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		//Do all GL transformations.
		ApplyGLTransformation ();
		ApplyTransform();

		for (i = 0; i < laserPoints.size (); i++)
		{
			//Note: It is necessary to use glBegin() and glEnd() for each point.
			//otherwise always one hit will occur.
			glPassThrough (i);
			glBegin (GL_POINTS);
			glVertex3f (laserPoints[i].X (), laserPoints[i].Y (),
						laserPoints[i].Z ());
			glEnd ();
		}
		glFlush ();
		int size = glRenderMode (GL_RENDER);
		selectedIndices.resize (0);
		AnalyzeFeedbackAndDepthBuffer (size, feedbackBuffer, depthBuffer, w,h,
									   selectedIndices, x1, h - y1 - height,
									   x1 + width, h - y1);

		DEBUG("\n*************************\n");
		DEBUG("hits: %d  size = %d \n", hits, size);

		delete[]feedbackBuffer;
		free (depthBuffer);
	}
	return 0;
}



void QGLPointsCanvas::WriteVRMLModeledObjects(char* fileName)
{
	ofstream file(fileName);
	int i;
	file<<"#VRML V2.0 utf8\n";
	for(i=0;i<laserObjects.size();i++)
		laserObjects[i]->WriteVRML(file);
	file.close();
}

void QGLPointsCanvas::ShowPointInfo()
{
	int size = selectedIndices.size();
	double x, y, z;
	x = y = z = 0;
	for(int i=0;i<selectedIndices.size();i++)
	{
		x += laserPoints[selectedIndices[i]].X();
		y += laserPoints[selectedIndices[i]].Y();
		z += laserPoints[selectedIndices[i]].Z();
	}
	char info[256];
	sprintf(info, "%.3f %.3f %.3f", x/(double)size, y/(double)size,	z/(double)size);
	if ( size == 1)
	{
		QMessageBox::information(this,"Point Coordinates","",info);
	}
	else
	{
		QMessageBox::information(this,"Average of Point Coordinates","",info);
	}
}

void QGLPointsCanvas::CalculateNormals()
{
	normalPoints = laserPoints.Normals(kNN);
}

vector<double> QGLPointsCanvas::CalculateCurvature()
{
	return laserPoints.Curvatures(kNN);
}

void QGLPointsCanvas::ShowGaussianSphere()
{
	normalPoints = laserPoints.Normals(kNN);
	QGaussianSphereViewer* br = new QGaussianSphereViewer(this,laserPoints,normalPoints);
	br->show();	
	br->raise();
}

///Shows curvature in another view.
void QGLPointsCanvas::ShowCurvature()
{
	vector<double> curv = this->CalculateCurvature();
	LaserPoints pts = laserPoints;
	pts.SetReflectance(curv);
	
	QGLPointsCanvas* v = new QGLPointsCanvas;
	v->SetLaserPoints(pts);
	v->SetDescription("Curvature");
	v->show();
}

///Returns number of selected points.
int QGLPointsCanvas::PointSelectionCount()
{
	return selectedIndices.size();
}
	
///Returns if we have point selection
bool QGLPointsCanvas::HasPointSelection()
{
	return !(selectedIndices.empty());
}
	

///Export data to ascii file.
void QGLPointsCanvas::ExportAscii(const char* fileName)
{
	if(fileName)
	{
		FILE* pFile = fopen(fileName,"wt");
		
		for(int i=0;i<laserPoints.size();i++)
			fprintf(pFile,"%6.4f %6.4f %6.4f %6.4f\n",
						laserPoints[i].X(),laserPoints[i].Y(),laserPoints[i].Z(),laserPoints[i].Reflectance());
		fclose(pFile);
	}
}

///Export data to vrml file.
void QGLPointsCanvas::ExportVrml(const char* fileName)
{
	if(fileName)
	{
		FILE* pFile = OpenVrmlFile((char*)fileName);
		LaserPoints2Vrml(pFile,laserPoints,colorScheme,colorMappingMode==UseReflectance);
		CloseVrmlFile(pFile);	
		
		const int buffSize = 4096;
		char buff[buffSize], buffCommand[buffSize];
		snprintf(buff,buffSize,"temp_%d",clock());
		WriteVRMLModeledObjects((char*)buff);
		
		snprintf(buffCommand,buffSize,"cat %s >> %s;rm %s;vrmlview %s &",buff,fileName,buff,fileName);
		cerr<<buffCommand;
		system(buffCommand);
	}
}

///Cut the points.
void QGLPointsCanvas::CutPoints()
{
	CopyPoints();
	InvertSelection();
	Crop();
}


///Gets information in form of a string.
QString QGLPointsCanvas:: GetInformationString()
{
	if(laserPoints.empty())
		return QString("There are no points in the current scan.");
		
	DataBoundsLaser dataBounds = laserPoints.DataBounds ();
	
	double minRef, maxRef, scaleFactor;
	laserPoints.ReflectanceRange(&minRef,&maxRef,&scaleFactor);
	
	vector<Vector3D> orthoDirs;
	vector<double> extents(3);
	
	if(laserPoints.size()>10)
	{
		extents = laserPoints.Extents(&orthoDirs);
	}
	 
	Vector3D mean = laserPoints.Mean();
	QString str;
	
	str.sprintf("<h1> Information for LaserPoints </h1>"
			"<p> <b>FileName:</b> %s <br>"
			" <b>Number of points:</b> %d  <b>Selected points:</b> %d<br>"
			" <b>Number of segments:</b> %d <b>Selected segments:</b> %d<br>"
			" <p><b>Data bounds </b> <br>"
			"      x: (%4.3f %4.3f) <br>"
			"      y: (%4.3f %4.3f) <br>"
			"      z: (%4.3f %4.3f) <br>"
			"      ref: (%4.3f %4.3f) <br>"
			"      Mean: (%4.7f %4.7f %4.7f) <br>"
			"<p> <b>Data Range </b><br>"
			"      x: %4.3f  y: %4.3f z: %4.3f ref: %4.3f <br>"
			"<p> <b>Aspect Ratio:</b> %4.3f <br>"
			"<p> <b>OrthoFrame:</b> <br>"
			"      x:(%4.3f %4.3f %4.3f)<br>"
			"      y: (%4.3f %4.3f %4.3f) <br>"
			"      z: (%4.3f %4.3f %4.3f) <br>"
			"<p> <b>Extents:</b> %4.3f %4.3f %4.3f <br>",
			FileName(),
			laserPoints.size(),selectedIndices.size(),
			segmentedVector.size(),selectedSegmentIndices.size(),
			dataBounds.Minimum ().X(),dataBounds.Maximum ().X(),
			dataBounds.Minimum ().Y(),dataBounds.Maximum ().Y(),
			dataBounds.Minimum ().Z(),dataBounds.Maximum ().Z(),
			minRef,maxRef,
			mean[0],mean[1],mean[2],
			dataBounds.XRange(),dataBounds.YRange(),dataBounds.ZRange(),maxRef-minRef,
			extents[0]/std::max(extents[1],1e-12), //The aspect ratio of the fitted plane, ratio of x and y extents. Ignore z.
			orthoDirs[0].X(),orthoDirs[0].Y(),orthoDirs[0].Z(),
			orthoDirs[1].X(),orthoDirs[1].Y(),orthoDirs[1].Z(),
			orthoDirs[2].X(),orthoDirs[2].Y(),orthoDirs[2].Z(),
			extents[0], extents[1],extents[2]);
			
	return str;
}

void QGLPointsCanvas::SetPalette(int index)
{
	colorScheme = GLColorScheme(index%8);
	UpdateDisplayList();
	
	emit ColormapChanged(colorScheme);
	Repaint();
}
///Changes the palette.
void QGLPointsCanvas:: ChangePalette()
{
	int old = colorScheme;
	SetPalette(old+1);
}


///Increases point size.
int QGLPointsCanvas::IncreasePointSize()
{
	SetPointSize(pointSize+1);
}
	
///Decreases point size.
int QGLPointsCanvas::DecreasePointSize()
{
	SetPointSize(pointSize-1);
}

int
QGLPointsCanvas::WindowSelectObjects (int x1, int y1, int width, int height,IndicesVector& selectedIndices)
{
#define SELECTION_TYPE_LASER_OBJECT 123456
	DEBUG("QGLPointsCanvas::WindowSelectObjects\n");
	// Initializing buffer in which selected points are stored
	int selectionBufferSize = laserObjects.size () * 40;
	int x, y, w, h;

	//Return if zero elements.
	if (!selectionBufferSize)
		return -1;

	ActivateCanvas();
	
	GLuint *selectionBuffer = new GLuint[selectionBufferSize];
	glSelectBuffer (selectionBufferSize, selectionBuffer);

	
	GLuint i, hits, picked;
	// Set the rendermode to the "select" mode
	glRenderMode (GL_SELECT);
	glEnable (GL_DEPTH_TEST);
	glPointSize (pointSize);

	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Initilize the name stack
	glInitNames ();
	
	//Do all GL transformations.
	ApplyGLTransformationForPicking (x1, y1, width, height);

	glPushName(SELECTION_TYPE_LASER_OBJECT);
	glPushName(0);
	for (int i = 0; i < laserObjects.size (); i++)
	{
		//Note: It is necessary to use glBegin() and glEnd() for each point.
		//otherwise always one hit will occur.
		glLoadName (i);
			laserObjects[i]->Draw();
		glEnd ();
	}
	glPopName();
	glPopName();
	glFlush ();

	hits = glRenderMode (GL_RENDER);
	if (hits <= 0)
	{
		DEBUG ("Nothing selected\n");
	}
	else
	{
		DEBUG("Selection hits = %d\n", hits);
	}

	
	selectedIndices.reserve (hits);
	selectedIndices.resize (0);
	// Loop over the points selected
	
	//Save the pointer for deletion.
	GLuint *tobeDeleted = selectionBuffer;
	
	//cerr<<"hits: "<<hits<<endl<<flush;
	for (i = 1; i <= hits; i++)
	{
		// Get the selected object id's from the buffer
		int namesCount = *selectionBuffer++;
		//cerr<<"name_count: "<<namesCount<<endl<<flush;
		double z_min = *selectionBuffer++;
		double z_max = *selectionBuffer++;
		/*
		for(int j=0;j<namesCount;j++)
		{
			cerr<<"name "<<j<<" : "<<selectionBuffer[j]<<endl;
		}
		cerr<<"\n\n";
		*/
		
		
		if(selectionBuffer[0]==SELECTION_TYPE_LASER_OBJECT)
		{
			selectedIndices.push_back(selectionBuffer[1]);
		}
		selectionBuffer += namesCount;
	}
	
	delete[]tobeDeleted;
	//cerr<<"Selection is done with "<<selectedIndices.size()<<"\n\n";
	
	return 0;
}

///Sets the color bar.
void QGLPointsCanvas::SetColorbar(int state)
{
	colorbarOn = state;
	Repaint();
}



QString QGLPointsCanvas:: GetHelpString()
{
	QString str;
	str.sprintf(
		"<h4> Mouse help for Laser Viewer </h4>"
		"<p>"
		
		"<h4> View manipulation </h4>"
		"<b>Left press :</b> Rotate the view <br>"
		"<b>Middle press :</b> Shift the view <br>"
		"<b>Right press :</b> Scale the view <br>"
		
		"<h4> Window point selection  </h4>"
		"<b>Shift+Left :</b> Window select points, new selection<br>"
		"<b>Ctrl+Left :</b> Window select points, additive selection<br>"
		"<b>Ctrl+Shift+Left:</b> Window de-select points<br>"
		
		"<h4> Freehand point selection  </h4>"
		"<b>Shift+Right :</b> Freehand select points, new selection<br>"
		"<b>Ctrl+Right :</b> Freehand select points, additive selection<br>"
		"<b>Ctrl+Shift+Right:</b> Freehand de-select points<br>"

		"<h4> Window object selection  </h4>"
		"<b>Shift+Middle :</b> Window select objects, new selection<br>"
		"<b>Ctrl+Middle :</b> Window select objects, additive selection<br>"
		"<b>Ctrl+Shift+Middle:</b> Window de-select objects<br>"
		
		"<h4> Keyboard help for Laser Viewer </h4>"
		"<b>Key + :</b> Increase point size <br>"
		"<b>Key - :</b> Decrease point size <br>"
		"<p>"
		"<b>Ctrl+L :</b> Inrease normal vector length<br>"
		"<b>Shift+L :</b> Decrease normal vector length<br>"
		"<p>"
		"<b>Ctrl+K :</b> Inrease k nearest neighbours<br>"
		"<b>Shift+L :</b> Decrease k nearest neighbours<br>"
		"<p>"
		"<b>Ctrl+V :</b> Paste points<br>"
		"<b>Shift+V :</b> Paste objects<br>"
		"<p>"
		"<b>T :</b> Toggle display of TIN<br>"
		"<p>"
		"<b>F :</b> Cycle through fill modes. Filled, line, and point<br>"
		"<p>"
		"<b>R :</b> Switch b/w use of reflectance and range<br>"
		"<b>Ctrl+R :</b> Reset view to default settings<br>"
		"<p>"
		"<b>Shift+X :</b> Crop the point cloud<br>"
		"<p>"
		"<b>N:</b> Open selection in new window<br>"
		"<p>"
		"<b>S:</b> Select all points<br>"
		"<p>"
		"<b>H:</b> Show this help screen<br>");

	return str;
}


///Get all ControlPoints.
LaserPoints QGLPointsCanvas::GetSelectedControlPoints() const
{
	LaserPoints pts;
	
	for(int i=0;i<selectedObjectIndices.size();i++)
	{
		if(laserObjects[selectedObjectIndices[i]]->Name()=="LaserControlPoint")
		{
			Vector3D center = ((LaserSphere*)(laserObjects[selectedObjectIndices[i]]))->centre;
			pts.push_back(center);
		}
	}
	return pts;
}

///Get All ControlPoints.
LaserPoints QGLPointsCanvas::GetAllControlPoints() const
{
	LaserPoints pts;
	
	for(int i=0;i<laserObjects.size();i++)
	{
		if(laserObjects[i]->Name()=="LaserControlPoint")
		{
			pts.push_back(LaserPoint(((LaserSphere*)(laserObjects[i]))->centre));
		}
	}
	return pts;
}

///Get Rotation.
Rotation3D QGLPointsCanvas::GetRotation() const
{
	return rotation;
}

///Get Translation.
Vector3D QGLPointsCanvas::GetTranslation() const
{
	return translation;
}

///Set Rotation.
void QGLPointsCanvas::SetRotation(Rotation3D rot)
{
	rotation = rot;
	emit PointTransformChanged();
}

///Set Translation.
void QGLPointsCanvas:: SetTranslation(Vector3D trans)
{
	translation = trans;
	emit PointTransformChanged();
}

///Set Transform.
void QGLPointsCanvas:: SetTransform(Rotation3D rot, Vector3D trans)
{
	rotation = rot;
	translation = trans;
	emit PointTransformChanged();
}

///Apply point transformations.
void QGLPointsCanvas::ApplyTransform()const
{
/*	
	AngleAxisRotation rot(rotation);
	glMatrixMode(GL_MODELVIEW);
	glTranslatef(translation.X(),translation.Y(),translation.Z());
	glRotatef(M_PI/180.00*rot.Angle(),rot.X(),rot.Y(),rot.Z());
	cerr<<"Translation "<<translation<<endl;
*/	
}


///Pastes objects from global copied clipboard.
void QGLPointsCanvas::DeleteObjects()
{
	vector<LaserObject*> selectedObjects = Select(laserObjects,selectedObjectIndices);
	//Select the inverse and make it the current set of objects.
	laserObjects = SelectInverse(laserObjects,selectedObjectIndices);
	selectedObjectIndices.clear();
	
	//Delete the pointers of the selected objects.
	for(int i=0;i<selectedObjects.size();i++)
	{
		//cerr<<"Deleting object of type "<<selectedObjects[i]->Name()<<"...";
		//A bug in deletion of csg objects causes problems.
		//Memory leak to be fixed later.
		if(selectedObjects[i]->Name()!="LaserCsgObject")
			delete selectedObjects[i];
		//cerr<<"Done\n";
	}
	//cerr<<"2 Now here we have "<<laserObjects.size()<<" objects"<<endl;
	//cerr<<"Now painting\n";
	this->paintGL();
	
}
	
	
///Decreases point size.
int QGLPointsCanvas::SetPointSize(int newSize)
{
	pointSize = newSize;
	pointSize = (pointSize>=1)?pointSize:1;
	UpdateDisplayList();Repaint();
	
	emit PointSizeChanged(pointSize);
}
	
//Open selection in new window.
QLaserViewer* QGLPointsCanvas::InNewWindow()
{
	QGLPointsCanvas* viewer = new QGLPointsCanvas;
	viewer->SetLaserPoints(GetSelectedLaserPoints());
	viewer->show(); viewer->raise();
}

///Set Auto histoequalize mode.
void QGLPointsCanvas::SetAutoHistoequalize(int mode)
{
	autoHistoEqualize = mode;
}

void QGLPointsCanvas::SetDepthSensitiveSelection(int bNew)
{
	bDepthSensitiveSelection = bNew;
}

void QGLPointsCanvas::SetNormalLengthFactor(int factor)
{
	normalLengthFactor = factor/1000.00;
	UpdateDisplayList(); Repaint();
	
}

void QGLPointsCanvas::SetNormalLengthFactor(double factor)
{
	normalLengthFactor = factor;
	UpdateDisplayList(); Repaint();
	
}

void QGLPointsCanvas::SetSmoothNormals(int bNew)
{
	smoothNormals = bNew;
	UpdateDisplayList(); Repaint();
}

void QGLPointsCanvas::ShowInformation()
{
	QString str = this->GetInformationString();
	cerr<<str.toStdString()<<endl;
	QMessageBox::information(this,"Information",str);
}

void QGLPointsCanvas::ShowHelp()
{
	QString str = this->GetHelpString();
	QMessageBox::information(this,"Keyboard and Mouse help",str);
}

void QGLPointsCanvas::ShowHelpAbout()
{
	
			
	QMessageBox::information(this,"About QtLaserViewer",
	QString("<h1> About QtLaserViewer </h1>"
	"<p> <b>QtLaserViewer</b> is a program for interactive viewing,"
	"selection, editing, segmentation, and fitting of point clouds</p>"
	"<p>  </p>"
	"<p>  </p>"
	"<p><b>Author:</b>   Tahir Rabbani </p>"
	"<p><b>Email:</b>   t.rabbani@lr.tudelft.nl </p>"
	"<p>  </p>"
	"<p>  </p>"));;
}

///Histoequalize laser points.
void QGLPointsCanvas::HistoEqualizeLaserPoints()
{
	laserPoints = laserPoints.Histoequalize();
	SetLaserPoints(laserPoints);
	Repaint();
}

///Load points from a file.
bool QGLPointsCanvas::LoadPoints()
{
	QString fn = QFileDialog::getOpenFileName(this,
					"Open new point cloud",
                    ".",
                    "laser point files (*.laser *.laser.gz *.pts *.txt)");
                            
  
	if ( !fn.isEmpty() ) 
	{
        this->LoadPoints((char*)(fn.toStdString().c_str()));
	}
}

bool QGLPointsCanvas::LoadPoints(const char* fileName)
{
	LaserPoints& pts = laserPoints; 
	pts.SetPointFile((char*)(fileName));
	cerr<<"Reading from "<<fileName<<"...";
	pts.Read();
	cerr<<"Done with "<<pts.size()<<" points\n";
	cerr<<"Starting to display them...";
	this->SetLaserPoints(pts);
	cerr<<"Done!!!\n";
}

///Load points from a file.
bool QGLPointsCanvas::LoadAscii()
{
	QString fn = QFileDialog::getOpenFileName(this,
					"Open new point cloud",
                    ".",
                    "laser point in ascii file (*.pts *.ascii *.txt)");  
	if ( !fn.isEmpty() ) 
	{
        LaserPoints pts; 
		pts.LoadFromAscii(fn.toStdString().c_str());
		this->SetLaserPoints(pts);
	}
}
	
//Saves points to ascii file.
bool QGLPointsCanvas::SaveAscii()
{
 	QString fn = QFileDialog::getSaveFileName(this,
 					"Save to new ascii point cloud file",
                    ".",
                    "laser ascii files (*.pts *.txt *.ascii)");
  
	if ( !fn.isEmpty() ) 
	{
        LaserPoints pts;
		GetSelectedLaserPoints(pts); 
		pts.SaveToAscii((char*)(fn.toStdString().c_str()));
	}
}
	

///Save points to laser file.
bool QGLPointsCanvas::SavePoints()
{
 	QString fn = QFileDialog::getSaveFileName(this,
				 	"Save to new point cloud file",
                    ".",
                    "laser point files (*.laser *.laser.gz *.pts )");
					  
	if ( !fn.isEmpty() ) 
	{
        LaserPoints pts;
		GetSelectedLaserPoints(pts); 
		pts.SetPointFile((char*)(fn.toStdString().c_str()));
		pts.Write();
	}
}

///Load objects from a file.
bool QGLPointsCanvas::LoadObjects()
{
	QString fn = QFileDialog::getOpenFileName(this,
					"Load objects from text file",
                    ".",
                    "ascii objects file (*.obj *.txt)");
					  
	if ( !fn.isEmpty() ) 
	{
        this->LoadObjects((char*)fn.toStdString().c_str());
	}
	UpdateActions();
}

bool QGLPointsCanvas::LoadObjects(const char* fileName)
{
	LaserObjectsVector objects;
	objects.Load((char*)fileName);
	GlobalCopiedObjects = objects;
	
	for(int i=0;i<objects.size();i++)
		laserObjects.push_back(objects[i]->Clone());
				
	emit CopiedObjectsChanged(GlobalCopiedObjects.size());
	UpdateActions();
}

///Save objects to laser file.
bool QGLPointsCanvas::SaveObjects()
{
	QString fn = QFileDialog::getSaveFileName(this,
                    "Save objects to a text file",
                    ".",
                  	"ascii objects file (*.obj *.txt)");
	if ( !fn.isEmpty() ) 
	{
        LaserObjectsVector selectedObjects = Select(laserObjects,selectedObjectIndices);
		selectedObjects.Save(fn.toStdString().c_str());
	}
	UpdateActions();
}

void QGLPointsCanvas::SetFillMode(int f)
{
	polygonMode = f;
	Repaint();
}

///Save objects to laser file.
void QGLPointsCanvas::StretchPoints()
{
	//Stretch in all dimensions.
	//LaserPoints pts = laserPoints;
	//StretchLaserPoints (pts);
	//SetLaserPoints (pts);
}

///Rotate using current Opengl rotation.
void QGLPointsCanvas::RotatePoints()
{
	LaserPoints temp = laserPoints;
	ApplyGLRotationToLaserPoints(temp);
	SetLaserPoints(temp);
}
	
void QGLPointsCanvas::Print()
{
	QString fileName = QFileDialog::getSaveFileName(this,"Save EPS file...",".","*.eps"); 		
	if(!fileName.isEmpty())
	{
		WriteEPS(fileName.toStdString().c_str());
	}
}

///Segments points into planar regions.	
void QGLPointsCanvas::DeleteSegmentation( )
{
	segmentedVector.resize(0);
	selectedSegmentIndices.resize(0);
	Repaint();
}

///Sets the subsampling factor.
void QGLPointsCanvas::SetSubsamplingFactor(int _subsampling)
{
	if(subSamplingFactor!=_subsampling)
	{
		subSamplingFactor = max(1,_subsampling);
		UpdateDisplayList();
		Repaint();
	}
}

///Gets the subsampling factor.
int QGLPointsCanvas::GetSubsamplingFactor()const
{
	return subSamplingFactor;
}

///Shows the segments browser.
void QGLPointsCanvas::ShowSegmentBrowser()
{
	QSegmentationBrowser *segBrowser = new 	QSegmentationBrowser(this,&segmentedVector,"Seg");
	segBrowser->exec();
	delete segBrowser;
}

///Shows the laser list browser.
void QGLPointsCanvas::ShowLaserListBrowser()
{
	cerr<<"TODO: ShowLaserListBrowser\n";
/*	
	QLaserListBrowser *browser = new QLaserListBrowser;
	browser->show();
*/	
}
	


///Growing functions.
void QGLPointsCanvas::GrowCylinders()
{
/*
	double angleThresholdDegrees;
	double distanceThreshold;
	cerr<<"Angle Threshold degrees? ";cin>>angleThresholdDegrees;
	cerr<<"distanceThreshold? "; cin>>distanceThreshold;
	
	LaserPoints sLaserPoints = GetSelectedLaserPoints();
	vector<int> sel;
	vector<LaserCylinder*> objects = ::GrowCylinders(sLaserPoints,kNN,angleThresholdDegrees,distanceThreshold,sel);
	
	//vector<LaserObject*> objects = ::GrowCylindersAndPlanes(sLaserPoints,kNN,angleThresholdDegrees,distanceThreshold,sel);
				
	if(objects.size())
	{
		selectedIndices = Select(selectedIndices,sel);
		for(int i=0;i<objects.size();i++)
			laserObjects.push_back(objects[i]);
	}
*/	
}

void QGLPointsCanvas::GrowPlanes()
{
/*	
	double angleThresholdDegrees;
	double distanceThreshold;
	cerr<<"Angle Threshold degrees? ";cin>>angleThresholdDegrees;
	cerr<<"distanceThreshold? "; cin>>distanceThreshold;
	
	LaserPoints sLaserPoints = GetSelectedLaserPoints();
	vector<int> sel;
	vector<LaserPlane*> objects = ::GrowPlanes(sLaserPoints,kNN,angleThresholdDegrees,distanceThreshold,sel);
				
	if(objects.size())
	{
		selectedIndices = Select(selectedIndices,sel);
		for(int i=0;i<objects.size();i++)
			laserObjects.push_back(objects[i]);
	}
*/	
}


void QGLPointsCanvas::GrowSpheres()
{

}


void QGLPointsCanvas::GrowTori()
{

}

///Hough transform functions.
void QGLPointsCanvas::HoughCylinder()
{


}


void QGLPointsCanvas::HoughPlane()
{

}


void QGLPointsCanvas::HoughCylinderAndPlane()
{


}



///Get selected indices.
vector<int>  QGLPointsCanvas::GetSelectedIndices()const
{
	return selectedIndices;
}

///Set selected indices.
void  QGLPointsCanvas::SetSelectedIndices(const vector<int>& indices)
{
	selectedIndices = indices;	
	emit PointSelectionChanged(selectedIndices.size());
	
	repaint();
}

///Set fill mode - from Sender()->Data
void QGLPointsCanvas::SetFillMode()
{
	int value;
	if(CheckQActionAndInt(sender(),value))
		SetFillMode(value);
}

///Set fill mode - from Sender()->Data
void QGLPointsCanvas::SetPalette()
{
	int value;
	if(CheckQActionAndInt(sender(),value))
		SetPalette(value);
}

///Sets the subsampling factor.
void QGLPointsCanvas::SetSubsamplingFactor()
{
	int value;
	if(CheckQActionAndInt(sender(),value))
		SetSubsamplingFactor(value);
}

///Sets colormapping mode - from sender()->data
void QGLPointsCanvas::SetColorMappingMode()
{
	int value;
	if(CheckQActionAndInt(sender(),value))
		SetColorMappingMode(value);
}

///Sets display type - from sender()->Data.
void QGLPointsCanvas::SetDisplayType()
{
	int value;
	if(CheckQActionAndInt(sender(),value))
		SetDisplayType(value);
}

///Set point size - from sender->data.
int QGLPointsCanvas::SetPointSize()
{
	int value;
	if(CheckQActionAndInt(sender(),value))
		SetPointSize(value);
}

///Set Auto histoequalize mode - from sender()->Data().
void QGLPointsCanvas::SetAutoHistoequalize()
{
	int value;
	if(CheckQActionAndInt(sender(),value))
		SetAutoHistoequalize(value);
}

///Sets depth sensitive selection - from sender()->data().
void QGLPointsCanvas::SetDepthSensitiveSelection()
{
	int value;
	if(CheckQActionAndInt(sender(),value))
		SetDepthSensitiveSelection(value);
}

///Normal length factor - from sender->data().
void QGLPointsCanvas::SetNormalLengthFactor()
{
	int value;
	if(CheckQActionAndInt(sender(),value))
		SetNormalLengthFactor(value);
}

///Set the normal smoothing - from sender()->data.
void QGLPointsCanvas::SetSmoothNormals()
{
	int value;
	if(CheckQActionAndInt(sender(),value))
		SetSmoothNormals(value);
}

///Sets the color bar - from sender()->Data.
void QGLPointsCanvas::SetColorbar()
{
	int value;
	if(CheckQActionAndInt(sender(),value))
		SetColorbar(value);
}

///Set use of display lists.
void QGLPointsCanvas::SetUseDisplayList(int bUse)
{
	bUseDisplayList = bUse;
	Repaint();
}

///Set use of display lists - from sender()->Data().
void QGLPointsCanvas::SetUseDisplayList()
{
	int value;
	if(CheckQActionAndInt(sender(),value))
		SetUseDisplayList(value);
}


void QGLPointsCanvas::MakeActionsAndMenubar()
{
	QAction* action = NULL;
	
	//Make a main menubar and add it to a layout.
	//This is the best way to add it to a widget other than mainwindow.
	menuBar = new QMenuBar(this);;
	
	if(!this->layout())
		this->setLayout(new QVBoxLayout);
	this->layout()->setMenuBar(menuBar);
	
	fileMenu = new QMenu("File",this);
	
	fileOpenGroup = new QActionGroup(this);
	
	fileLoadLaserAction = action = new QAction("Load Points (laser file)",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(LoadPoints()));
	fileOpenGroup->addAction(fileLoadLaserAction);
	
	fileLoadLaserAsciiAction = action = new QAction("Load Points (ascii file)",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(LoadAscii()));
	fileOpenGroup->addAction(action);
	
	fileLoadObjectsAction = action = new QAction("Load Objects",this);
	QObject::connect(action,SIGNAL(triggered()),SLOT(LoadObjects()));
	fileOpenGroup->addAction(action);
	
	fileLoadSegmentationAction = action = new QAction("Load Segmentation",this);
	QObject::connect(action,SIGNAL(triggered()),SLOT(LoadSegmentation()));
	fileOpenGroup->addAction(action);
	
	fileLoadSelectedIndicesAction = action = new QAction("Load Selected Indices",this);
	QObject::connect(action,SIGNAL(triggered()),SLOT(LoadSelectedIndices()));
	fileOpenGroup->addAction(action);
	
	fileLoadNormalsAction = action = new QAction("Load Normals",this);
	QObject::connect(action,SIGNAL(triggered()),SLOT(LoadNormals()));
	fileOpenGroup->addAction(action);
	
	fileSaveGroup = new QActionGroup(this);
	
	fileSaveLaserAction = action = new QAction("Save Points (laser file)",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SavePoints()));
	fileSaveGroup->addAction(action);
		
	fileSaveLaserAsciiAction = action = new QAction("Save Points (ascii file)",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SaveAscii()));
	fileSaveGroup->addAction(action);
		
	fileSaveObjectsAction = action = new QAction("Save Objects",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SaveObjects()));
	fileSaveGroup->addAction(action);
	
	fileSaveSegmentationAction = action = new QAction("Save Segmentation",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SaveSegmentation()));
	fileSaveGroup->addAction(action);
	
	fileSaveSelectedIndicesAction = action = new QAction("Save Selected Indices",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SaveSelectedIndices()));
	fileSaveGroup->addAction(action);
	
	fileSaveNormalsAction = action = new QAction("Save Normals",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SaveNormals()));
	fileSaveGroup->addAction(action);
	
	fileSaveKnnAction = action = new QAction("Save KNN indices",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SaveKnnIndices()));
	fileSaveGroup->addAction(action);
	
	fileExportVRMLAction = action = new QAction("Export VRML",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(ExportVRML()));
	fileSaveGroup->addAction(action);
		
	exitAction = action = new QAction("Exit",this);
	QObject::connect(action,SIGNAL(triggered()),QCoreApplication::instance(),SLOT(quit()));
	
	fileMenu->addActions(fileOpenGroup->actions());
	fileMenu->addSeparator();
	fileMenu->addActions(fileSaveGroup->actions());
	fileMenu->addSeparator();
	fileMenu->addAction(exitAction);
	
	menuBar->addMenu(fileMenu);

	viewMenu = new QMenu("View",this);
	paletteMenu = new QMenu("Palette",this);
	
	paletteGroup = new QActionGroup(this);
	paletteGroup->setExclusive(true);
	
	paletteGrayAction = action = new QAction("Gray",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetPalette()));
	action->setShortcut(QKeySequence("Ctrl+G"));
	action->setData(GLColorSchemeScaledGray);
	action->setCheckable(true);
	paletteGroup->addAction(action);
		
	paletteStandardAction = action = new QAction("Standard",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetPalette()));
	action->setData(GLColorSchemeStandard);
	action->setCheckable(true);
	paletteGroup->addAction(action);
		
	paletteJetAction = action = new QAction("Jet",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetPalette()));
	action->setData(GLColorSchemeJet);
	action->setCheckable(true);
	paletteGroup->addAction(action);
		
	paletteHotAction = action = new QAction("Hot",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetPalette()));
	action->setData(GLColorSchemeHot);
	action->setCheckable(true);
	paletteGroup->addAction(action);
		
	paletteCoolAction = action = new QAction("Cool",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetPalette()));
	action->setData(GLColorSchemeCool);
	action->setCheckable(true); 
	paletteGroup->addAction(action);
		
	paletteArmyAction = action = new QAction("Army",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetPalette()));
	action->setData(GLColorSchemeArmy);
	action->setCheckable(true);
	paletteGroup->addAction(action); 
	
	paletteElevationAction = action = new QAction("Elevation",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetPalette()));
	action->setData(GLColorSchemeElevation);
	action->setCheckable(true); 
	paletteGroup->addAction(action);
		
	palettePastelAction = action = new QAction("Pastel",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetPalette()));
	action->setData(GLColorSchemePastel);
	action->setCheckable(true); 
	paletteGroup->addAction(action);
		
	paletteGrayGreenAction = action = new QAction("GrayGreen",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetPalette()));
	action->setData(GLColorSchemeGrayGreen);
	action->setCheckable(true); 
	paletteGroup->addAction(action);
		
	paletteGrayBlueAction = action = new QAction("GrayBlue",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetPalette()));
	action->setData(GLColorSchemeGrayBlue);
	action->setCheckable(true); 
	paletteGroup->addAction(action);
		
	paletteThresholdedAction = action = new QAction("Thresholded",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetPalette()));
	action->setData(GLColorSchemeThresholdedPalette);
	action->setCheckable(true); 
	paletteGroup->addAction(action);
		
	paletteRedAction = action = new QAction("Red",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetPalette()));
	action->setData(GLColorSchemeScaledRed);
	action->setCheckable(true); 
	paletteGroup->addAction(action);
		
	paletteGreenAction = action = new QAction("Green",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetPalette()));
	action->setData(GLColorSchemeScaledGreen);
	action->setCheckable(true); 
	paletteGroup->addAction(action);
		
	paletteBlueAction = action = new QAction("Blue",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetPalette()));
	action->setData(GLColorSchemeScaledBlue);
	action->setCheckable(true); 
	paletteGroup->addAction(action);
		
	paletteMenu->addActions(paletteGroup->actions());
	viewMenu->addMenu(paletteMenu);
	
	displayMenu = new QMenu("Display",this);
	
	displayGroup = new QActionGroup(this);
	displayGroup->setExclusive(false);
	
	displayPointCloudAction = action = new QAction("Point-cloud",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetDisplayType()));
	action->setCheckable(true); 
	displayGroup->addAction(action);
		
	displayTINAction = action = new QAction("TIN",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetDisplayType()));
	action->setCheckable(true); 
	displayGroup->addAction(action);
	
	displaySelectedPointsAction = action = new QAction("Selected points",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetDisplayType()));
	action->setCheckable(true); 
	displayGroup->addAction(action);
	
	displaySegmentationAction = action = new QAction("Segmentation",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetDisplayType()));
	action->setCheckable(true); 
	displayGroup->addAction(action);
	
	displayModelsAction = action = new QAction("Models",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetDisplayType()));
	action->setCheckable(true); 
	displayGroup->addAction(action);
	
	displayMenu->addSeparator();
	
	displayColorbarAction = action = new QAction("Colorbar",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(ToggleColorbar()));
	action->setCheckable(true); 
	displayGroup->addAction(action);
		
	displayBkColorAction = action = new QAction("Background color",this);
	displayGroup->addAction(action);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(GetNewBackgroundColor()));
	
	displayMenu->addActions(displayGroup->actions());
	viewMenu->addMenu(displayMenu);
	
	pointSizeGroup = new QActionGroup(this);
	pointSizeMenu = new QMenu("Point size",this);
	
	int pointSizeArray[6] = {1,2,4,8,12,20};
	
	for(int i=0;i<6;i++)
	{
		int val = pointSizeArray[i];
		action = new QAction(QString("%1").arg(val),this);
		QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetPointSize()));
		pointSizeGroup->addAction(action);
		action->setData(val);
		action->setCheckable(true);
	}
	
	pointSizeIncreaseAction = action = new QAction("Increase",this);
	QObject::connect(action,SIGNAL(triggered()),SLOT(IncreasePointSize()));
	pointSizeDecreaseAction = action = new QAction("Decrease",this);
	QObject::connect(action,SIGNAL(triggered()),SLOT(DecreasePointSize()));
	
	pointSizeMenu->addActions(pointSizeGroup->actions());
	pointSizeMenu->addSeparator();
	pointSizeMenu->addAction(pointSizeIncreaseAction);
	pointSizeMenu->addAction(pointSizeDecreaseAction);
	
	viewMenu->addMenu(pointSizeMenu);
	
	subSampleGroup = new QActionGroup(this);
	subSampleMenu = new QMenu("Subsampling interval",this);
	int subsampleArray[11] = {1,2,4,10,20,50,100,-1,1000,4000,10000};
	for(int i=0;i<11;i++)
	{
		int val = subsampleArray[i];
		if(val<0)
		{
			//subSampleMenu->addSeparator();
			continue;
		}
		action = new QAction(QString("%1").arg(val),this);
		QObject::connect(action,SIGNAL(triggered()),SLOT(SetSubsamplingFactor()));
		subSampleGroup->addAction(action);
		action->setData(val);
		action->setCheckable(true);
	}
	
	subSampleMenu->addActions(subSampleGroup->actions());
	viewMenu->addMenu(subSampleMenu);
	
	colorCodingMenu = new QMenu("Color coding",this);
	
	colorCodingGroup = new QActionGroup(this);
	colorCodingGroup->setExclusive(true);
	
	useReflectanceAction = action = new QAction("Use reflectance",this);
	QObject::connect(action,SIGNAL(triggered()),SLOT(SetColorMappingMode()));
	action->setData(1);
	action->setCheckable(true);
	colorCodingGroup->addAction(action);
		
	useRangeAction = action = new QAction("Use range",this);
	QObject::connect(action,SIGNAL(triggered()),SLOT(SetColorMappingMode()));
	action->setData(0);
	action->setCheckable(true);
	colorCodingGroup->addAction(action);
		
	useColorAction = action = new QAction("Use color",this);
	QObject::connect(action,SIGNAL(triggered()),SLOT(SetColorMappingMode()));
	action->setData(2);
	action->setCheckable(true);
	colorCodingGroup->addAction(action);
		
	colorCodingMenu->addActions(colorCodingGroup->actions());
	viewMenu->addMenu(colorCodingMenu);
		
	autoHistoEqualizeAction = action = new QAction("Auto histoequalize",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetAutoHistoequalize()));
	action->setCheckable(true);
	
	colorCodingMenu->addSeparator();
	colorCodingMenu->addAction(action);
		
	viewMenu->addSeparator();
	

	viewPointGroup = new QActionGroup(this);
	
	viewFromXAction = action = new QAction("View from X",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetViewFromX()));
	viewPointGroup->addAction(action);
	
	viewFromYAction = action = new QAction("View from Y",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetViewFromY()));
	viewPointGroup->addAction(action);
	
	viewFromZAction = action = new QAction("View from Z",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetViewFromZ()));
	viewPointGroup->addAction(action);
		
	resetViewAction = action = new QAction("Reset view",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(ResetView()));
	viewPointGroup->addAction(action);
	
	viewPointMenu = new QMenu("View points",this);
	viewPointMenu->addActions(viewPointGroup->actions());
	viewMenu->addMenu(viewPointMenu);
	
	menuBar->addMenu(viewMenu);
	
	
	
	selectionMenu = new QMenu("Selection",this);
	
	selectionGroup = new QActionGroup(this);
	selectionGroup->setExclusive(false);
	
	selectAllAction = action = new QAction("Select all",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SelectAll()));
	selectionGroup->addAction(action);
	
	deselectAllAction = action = new QAction("Deselect all",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(DeselectAll()));
	selectionGroup->addAction(action);
		
	invertSelectionAction = action = new QAction("Invert selection",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(InvertSelection()));
	selectionGroup->addAction(action);
		
	selectionMenu->addActions(selectionGroup->actions());
	selectionMenu->addSeparator();
	
	depthSensitiveSelectionAction = action = new QAction("Depth sensitive",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetDepthSensitiveSelection()));
	action->setCheckable(true);
		
	selectionMenu->addAction(depthSensitiveSelectionAction);
	selectionMenu->addSeparator();
	
	inNewViewerAction = action = new QAction("In new viewer",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(InNewWindow()));
		
	selectionMenu->addAction(inNewViewerAction);
	menuBar->addMenu(selectionMenu);
	
		
	editMenu = new QMenu("Edit",this);
	
	copyGroup = new QActionGroup(this);
	
	copyPointsAction = action = new QAction("Copy points",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(CopyPoints()));
	copyGroup->addAction(action);
		
	copyObjectsAction = action = new QAction("Copy objects",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(CopyObjects()));
	copyGroup->addAction(action);
		
	pasteGroup = new QActionGroup(this);
	
	pastePointsAction = action = new QAction("Paste points",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(PastePoints()));
	pasteGroup->addAction(action);
		
	pasteObjectsAction = action = new QAction("Paste objects",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(PasteObjects()));
	pasteGroup->addAction(action);
		
	cropGroup = new QActionGroup(this);
	
	cropPointsAction = action = new QAction("Crop points",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(Crop()));
	cropGroup->addAction(action);
		
	deleteGroup = new QActionGroup(this);
	
	deleteObjectsAction = action = new QAction("Delete objects",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(DeleteObjects()));
	deleteGroup->addAction(action);
		
	histoEqualizeAction = action = new QAction("Histoequalize points",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(HistoEqualizeLaserPoints()));
		
	editMenu->addActions(copyGroup->actions());
	editMenu->addSeparator();
	editMenu->addActions(pasteGroup->actions());
	editMenu->addSeparator();
	editMenu->addActions(cropGroup->actions());
	editMenu->addSeparator();
	editMenu->addActions(deleteGroup->actions());
	editMenu->addSeparator();
	editMenu->addAction(histoEqualizeAction);
	
	menuBar->addMenu(editMenu);
	
	fitMenu = new QMenu("Fitting",this);
	
	fittingGroup = new QActionGroup(this);
	fittingGroup->setExclusive(false);
	
	fitPlaneAction = action = new QAction("Plane",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(FitPlane()));
	fittingGroup->addAction(action);
		
	fitSphereAction = action = new QAction("Sphere",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(FitSphere()));
	fittingGroup->addAction(action);
		
	fitCylinderAction = action = new QAction("Cylinder",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(FitCylinder()));
	fittingGroup->addAction(action);
		
	fitTorusAction = action = new QAction("Torus",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(FitTorus()));
	fittingGroup->addAction(action);
		
	fitMenu->addActions(fittingGroup->actions());
	fitMenu->addSeparator();
	
	processingGroup = new QActionGroup(this);
	processingGroup->setExclusive(false);
	
	calculateNormalsAction = action = new QAction("Calculate normals",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(CalculateNormals()));
	processingGroup->addAction(action);
		
	calculateCurvatureAction = action = new QAction("Calculate curvature",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(ShowCurvature()));
	processingGroup->addAction(action);
		
	showGaussianSphere = action = new QAction("Show Gaussian sphere",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(ShowGaussianSphere()));
	processingGroup->addAction(action);
		
	fitMenu->addActions(processingGroup->actions());
	
	menuBar->addMenu(fitMenu);
	
	segmentationMenu = new QMenu("Segmentation",this);
	
	segmentationGroup = new QActionGroup(this);
	
	segmentSmoothAction = action = new QAction("Into smooth patches",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SegmentPointCloud()));
	segmentationGroup->addAction(action);
		
	segmentPlanarAction = action = new QAction("Into planar patches",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SegmentIntoPlanes()));
	segmentationGroup->addAction(action);
		
	segmentConnectedAction = action = new QAction("Into connected components",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SegmentIntoComponents()));
	segmentationGroup->addAction(action);
		
	showSegmentBrowserAction = action = new QAction("Show segmentation browser...",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(ShowSegmentBrowser()));
	segmentationGroup->addAction(action);
		
	showLaserListBrowserAction = action = new QAction("Show laser list browser",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(ShowLaserListBrowser()));
	segmentationGroup->addAction(action);
		
	deleteSegmentationAction = action = new QAction("Delete segmentation",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(DeleteSegmentation()));
	segmentationGroup->addAction(action);
		
	segmentationMenu->addActions(segmentationGroup->actions());
		
	menuBar->addMenu(segmentationMenu);
	
	settingsMenu = new QMenu("Settings",this);
	
	kNNMenu = new QMenu("KNN",this);
	
	kNNGroup = new QActionGroup(this);
	
	currentKnnAction = action = new QAction("CurrentKnn",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(DoNothing()));
	currentKnnAction->setEnabled(false);
	
	kNNMenu->addAction(action);
	
	int kNNValues[] = {5,10,20,40,60,80,100,120,140,160};
	
	for(int i=0;i<10;i++)
	{
		int val = kNNValues[i];
		action = new QAction(QString("%1").arg(val),this);
		QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetKnn()));
		action->setData(val);
		action->setCheckable(true);
		kNNGroup->addAction(action);
	}
	
	kNNMenu->addActions(kNNGroup->actions());
	settingsMenu->addMenu(kNNMenu);
	
	normalLengthGroup = new QActionGroup(this);
	normalLengthGroup->setExclusive(false);
	
	normalLengthMenu = new QMenu("Normal length",this);
		
	currentNormalLengthAction = action = new QAction(" ",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(DoNothing()));
	action->setEnabled(false);
		
	normalLengthMenu->addAction(currentNormalLengthAction);
	
	int normalLengthArray[] = {1,10,100,1000,10000,100000,1000000,10000000,100000000};
	
	for(int i=0;i<9;i++)
	{
		int val = normalLengthArray[i];
		action = new QAction(QString("%1").arg(val*0.001),this);
		QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetNormalLengthFactor()));
		action->setData(val);
		action->setCheckable(true);
		normalLengthGroup->addAction(action);
	}
	
	normalLengthMenu->addActions(normalLengthGroup->actions());
	settingsMenu->addMenu(normalLengthMenu);
	
	polygonFillMenu = new QMenu("Polygon fill",this);
    polygonFillGroup = new QActionGroup(this);
			
	polygonFilledAction = action = new QAction("Filled",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetFillMode()));
	action->setData(GL_FILL);
	action->setCheckable(true);
	polygonFillGroup->addAction(action);
		
	polygonLineAction = action = new QAction("Line",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetFillMode()));
	action->setData(GL_LINE);
	action->setCheckable(true);
	polygonFillGroup->addAction(action);
		
	polygonPointAction = action = new QAction("Point",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetFillMode()));
	action->setData(GL_POINT);
	action->setCheckable(true);
	polygonFillGroup->addAction(action);
		
	polygonFillMenu->addActions(polygonFillGroup->actions());
	settingsMenu->addMenu(polygonFillMenu);
	
	useDisplayListAction = action = new QAction("Use display list",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(SetUseDisplayList()));
	action->setCheckable(true);
		
	settingsMenu->addSeparator();
	settingsMenu->addAction(useDisplayListAction);
	
	menuBar->addMenu(settingsMenu);
	
	registrationMenu = new QMenu("Registration");
	
	registerICP = action = new QAction("ICP method...",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(RegisterICP()));
	registrationMenu->addAction(action);
	
	registerIndirect = action = new QAction("Indirect method...",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(RegisterIndirect()));
	registrationMenu->addAction(action);
	
	menuBar->addMenu(registrationMenu);
		
	helpMenu = new QMenu("Help",this);
	
	helpGroup = new QActionGroup(this);
	
	action = new QAction("Keyboard and mouse",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(ShowHelp()));
	helpGroup->addAction(action);
	
	action = new QAction("Information",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(ShowInformation()));
	helpGroup->addAction(action);
	
	action = new QAction("About QtLaserViewer...",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(ShowHelpAbout()));
	helpGroup->addAction(action);
	
	helpMenu->addActions(helpGroup->actions());

	menuBar->addMenu(helpMenu);
	
	//Set connections for updating actions.
	QObject::connect(menuBar,SIGNAL(hovered()),this,SLOT(UpdateActions()));
	
	QObject::connect(fileMenu,SIGNAL(hovered()),this,SLOT(UpdateActions()));
	QObject::connect(viewMenu,SIGNAL(hovered()),this,SLOT(UpdateActions()));
	QObject::connect(selectionMenu,SIGNAL(hovered()),this,SLOT(UpdateActions()));
	QObject::connect(editMenu,SIGNAL(hovered()),this,SLOT(UpdateActions()));
	QObject::connect(fitMenu,SIGNAL(hovered()),this,SLOT(UpdateActions()));
	QObject::connect(segmentationMenu,SIGNAL(hovered()),this,SLOT(UpdateActions()));
	QObject::connect(settingsMenu,SIGNAL(hovered()),this,SLOT(UpdateActions()));
	QObject::connect(helpMenu,SIGNAL(hovered()),this,SLOT(UpdateActions()));
	
	QObject::connect(menuBar,SIGNAL(aboutToShow()),this,SLOT(UpdateActions()));
	QObject::connect(fileMenu,SIGNAL(aboutToShow()),this,SLOT(UpdateActions()));
	QObject::connect(viewMenu,SIGNAL(aboutToShow()),this,SLOT(UpdateActions()));
	QObject::connect(selectionMenu,SIGNAL(aboutToShow()),this,SLOT(UpdateActions()));
	QObject::connect(editMenu,SIGNAL(aboutToShow()),this,SLOT(UpdateActions()));
	QObject::connect(fitMenu,SIGNAL(aboutToShow()),this,SLOT(UpdateActions()));
	QObject::connect(segmentationMenu,SIGNAL(aboutToShow()),this,SLOT(UpdateActions()));
	QObject::connect(settingsMenu,SIGNAL(aboutToShow()),this,SLOT(UpdateActions()));
	QObject::connect(helpMenu,SIGNAL(aboutToShow()),this,SLOT(UpdateActions()));	
	
//	UpdateActions();	
	
	
}

void QGLPointsCanvas::UpdateActions()
{
	fileSaveLaserAction->setEnabled(this->PointSelectionCount());
	fileSaveLaserAsciiAction->setEnabled(this->PointSelectionCount());
	fileSaveObjectsAction->setEnabled(this->selectedObjectIndices.size());
	
	fileSaveSegmentationAction->setEnabled(segmentedVector.size());
	fileSaveSelectedIndicesAction->setEnabled(selectedIndices.size());
	fileSaveNormalsAction->setEnabled(normalPoints.size());
		
	paletteGrayAction->setChecked(colorScheme==GLColorSchemeScaledGray);
	paletteStandardAction->setChecked(colorScheme==GLColorSchemeStandard);
	paletteJetAction->setChecked(colorScheme==GLColorSchemeJet);
	paletteHotAction->setChecked(colorScheme==GLColorSchemeHot);
	paletteCoolAction->setChecked(colorScheme==GLColorSchemeCool);
	paletteArmyAction->setChecked(colorScheme==GLColorSchemeArmy);
	paletteElevationAction->setChecked(colorScheme==GLColorSchemeElevation);
	palettePastelAction->setChecked(colorScheme==GLColorSchemePastel);
	paletteGrayGreenAction->setChecked(colorScheme==GLColorSchemeGrayGreen);
	paletteGrayBlueAction->setChecked(colorScheme==GLColorSchemeGrayBlue);
	paletteThresholdedAction->setChecked(colorScheme==GLColorSchemeThresholdedPalette);
	paletteRedAction->setChecked(colorScheme==GLColorSchemeScaledRed);
	paletteGreenAction->setChecked(colorScheme==GLColorSchemeScaledGreen);
	paletteBlueAction->setChecked(colorScheme==GLColorSchemeScaledBlue);
	
	displayPointCloudAction->setData(pointsCanvasDisplayMode ^ (DISPLAY_CLOUD|DISPLAY_DATA_POINTS));
	displayPointCloudAction->setChecked(pointsCanvasDisplayMode&(DISPLAY_CLOUD|DISPLAY_DATA_POINTS));
	
	displayTINAction->setData(pointsCanvasDisplayMode ^ DISPLAY_TIN);
	displayTINAction->setChecked(pointsCanvasDisplayMode&(DISPLAY_TIN));
	
	displaySelectedPointsAction->setChecked(pointsCanvasDisplayMode&(DISPLAY_SELECTED_POINTS));
	displaySelectedPointsAction->setData(pointsCanvasDisplayMode ^ DISPLAY_SELECTED_POINTS);
	
	displaySegmentationAction->setChecked(pointsCanvasDisplayMode&(DISPLAY_SEGMENTED_POINTS));
	displaySegmentationAction->setData(pointsCanvasDisplayMode ^ DISPLAY_SEGMENTED_POINTS);
	
	displayModelsAction->setChecked(pointsCanvasDisplayMode&(DISPLAY_MODELED_OBJECTS));
	displayModelsAction->setData(pointsCanvasDisplayMode ^ DISPLAY_MODELED_OBJECTS);
	
	displayColorbarAction->setChecked(colorbarOn);
	
	const QList<QAction*>& listPointSizeActions = pointSizeGroup->actions();
	for (int i = 0; i < listPointSizeActions.size(); ++i) 
	{
		listPointSizeActions.at(i)->setChecked(listPointSizeActions.at(i)->data().toInt() == pointSize);
	}
	
	const QList<QAction*>&	listSubSampleActions = subSampleGroup->actions();
	for (int i = 0; i < listSubSampleActions.size(); ++i) 
	{
		listSubSampleActions.at(i)->setChecked(listSubSampleActions.at(i)->data().toInt() == subSamplingFactor);
	}	

	useReflectanceAction->setChecked(colorMappingMode==UseReflectance);
	useRangeAction->setChecked(colorMappingMode==UseRange);
	useColorAction->setChecked(colorMappingMode==UseColor);
	
	autoHistoEqualizeAction->setData(!autoHistoEqualize);
	autoHistoEqualizeAction->setChecked(autoHistoEqualize);
	
	deselectAllAction->setEnabled(this->PointSelectionCount());
	invertSelectionAction->setEnabled(this->PointSelectionCount());
	
	depthSensitiveSelectionAction->setChecked(bDepthSensitiveSelection);
	depthSensitiveSelectionAction->setData(!bDepthSensitiveSelection);
		
	copyPointsAction->setEnabled(this->PointSelectionCount());
	copyObjectsAction->setEnabled(selectedObjectIndices.size());
	pastePointsAction->setEnabled(GlobalCopiedPoints.size());
	pasteObjectsAction->setEnabled(GlobalCopiedObjects.size());
	cropPointsAction->setEnabled(this->PointSelectionCount());
	deleteObjectsAction->setEnabled(selectedObjectIndices.size());
	
	histoEqualizeAction->setEnabled(laserPoints.size());
	
	fitPlaneAction->setEnabled(this->PointSelectionCount());
	fitSphereAction->setEnabled(this->PointSelectionCount());
	fitCylinderAction->setEnabled(this->PointSelectionCount());
	fitTorusAction->setEnabled(this->PointSelectionCount());
	
	calculateNormalsAction->setEnabled(laserPoints.size());
	calculateCurvatureAction->setEnabled(laserPoints.size());
	showGaussianSphere->setEnabled(laserPoints.size());
	
	segmentSmoothAction->setEnabled(laserPoints.size());
	segmentPlanarAction->setEnabled(laserPoints.size());
	segmentConnectedAction->setEnabled(laserPoints.size());
	
	showSegmentBrowserAction->setEnabled(segmentedVector.size());
	deleteSegmentationAction->setEnabled(segmentedVector.size());
	
	currentKnnAction->setText(QString("Current KNN = %1").arg(kNN));
	const QList<QAction*>& listKnnActions = kNNGroup->actions();
	for (int i = 0; i < listKnnActions.size(); ++i) 
	{
		listKnnActions.at(i)->setChecked(listKnnActions.at(i)->data().toInt() == kNN);
	}

	currentNormalLengthAction->setText(QString("Current length = %1").arg(normalLengthFactor));
    const QList<QAction *>& listNormalLengthActions = normalLengthGroup->actions();
	for (int i = 0; i < listNormalLengthActions.size(); ++i) 
	{
		listNormalLengthActions.at(i)->setChecked(fabs((listNormalLengthActions.at(i)->data().toInt()) - normalLengthFactor*1000.00)<1e-6);
	}
	
	
	polygonFilledAction->setChecked(polygonMode==GL_FILL);
	polygonLineAction->setChecked(polygonMode==GL_LINE);
	polygonPointAction->setChecked(polygonMode==GL_POINT);
	
	useDisplayListAction->setChecked(bUseDisplayList);
	useDisplayListAction->setData(!bUseDisplayList);

}

void QGLPointsCanvas::SetMenuVisible(bool status)
{
	menuBar->setVisible(status);
}
	

using std::ofstream;
using std::ifstream;
///Load segmentation from a file.
bool QGLPointsCanvas::LoadSegmentation(QString fileName)
{
	segmentedVector.Load(fileName.toStdString());	
}

///Load segmentation from a file. Ask user for the file name too.
bool QGLPointsCanvas::LoadSegmentation() 
{
	QString fileName = QFileDialog::getOpenFileName(this,
					"Open segments to a file" 
					".",
                    "ascii segmentation file (*.segments )"); 
	if(!fileName.isEmpty())
	{
		LoadSegmentation(fileName);
	}
}

///Save segmentation to a file.
bool QGLPointsCanvas::SaveSegmentation(QString fileName) 
{
	segmentedVector.Save(fileName.toStdString()); 
}

///Save segmentation to a file. Ask user for the file name too.
bool QGLPointsCanvas::SaveSegmentation() 
{
	QString fileName = QFileDialog::getSaveFileName(this,
					"Save new segments file" 
					".",
                    "ascii segmentation file (*.segments )"); 
	if(!fileName.isEmpty())
	{
		SaveSegmentation(fileName);
	}
}

///Load normals from a file.
bool QGLPointsCanvas::LoadNormals(QString fileName) 
{
	normalPoints.LoadFromAscii((char*)(fileName.toStdString().c_str()));
	UpdateDisplayList();
	Repaint();
}

///Load normls from a file. Ask user for the file name too.
bool QGLPointsCanvas::LoadNormals() 
{ 
	QString fileName = QFileDialog::getOpenFileName(this,
					"Open normals file" 
					".",
                    "normals file (*.normals )"); 
	if(!fileName.isEmpty())
	{
		LoadNormals(fileName);
	}	
}

///Save normals to a file.
bool QGLPointsCanvas::SaveNormals(QString fileName) 
{ 
	CalculateNormals();
	normalPoints.SaveToAscii((char*)(fileName.toStdString().c_str()));
}

///Save normals to a file. Ask user for the file name too.
bool QGLPointsCanvas::SaveNormals() 
{ 
	QString fileName = QFileDialog::getSaveFileName(this,
					"Save new normals file" 
					".",
                    "normals file (*.normals )"); 
	if(!fileName.isEmpty())
	{
		SaveNormals(fileName);
	}
}

///Load selected indices from a file.
bool QGLPointsCanvas::LoadSelectedIndices(QString fileName) 
{ 
	ifstream infile ((char*)(fileName.toStdString().c_str()));
	
	if(infile.is_open() && infile.good())
	{
		std::set<int> readIndices;
		int val;
		
		while(infile.good() && !infile.eof())
		{
			infile>>val;
			
			if(val>=0 && val<laserPoints.size())
				readIndices.insert(val);		
		}
		if(readIndices.size())
		{
			selectedIndices.resize(readIndices.size());
			copy(readIndices.begin(),readIndices.end(),selectedIndices.begin());
			
			emit PointSelectionChanged(selectedIndices.size());
			UpdateDisplayList();
			Repaint();
		}
		else
		{
			ERR_MSG("0 good indices read from file. Keeping old selection");
		}
		infile.close();	
	}
	else
	{
		ERR_MSG(QString("Failed to open %1 for loading selected indices").arg(fileName));
	}
}

///Load selected indices from a file. Ask user for the file name too.
bool QGLPointsCanvas::LoadSelectedIndices() 
{ 
	QString fileName = QFileDialog::getOpenFileName(this,
					"Open indices file" 
					".",
                    "indices file (*.indices *.sel *.txt *.ascii )"); 
	if(!fileName.isEmpty())
	{
		LoadSelectedIndices(fileName);
	}
}

///Save selected indices to a file.
bool QGLPointsCanvas::SaveSelectedIndices(QString fileName) 
{ 
	std::ofstream outfile ((char*)(fileName.toStdString().c_str()));
	
	if(outfile.is_open() && outfile.good())
	{
		for(int i=0;i<selectedIndices.size();i++)
			outfile<<selectedIndices[i]<<"  ";
		outfile<<endl;
		outfile.close();	
	}
	else
	{
		ERR_MSG(QString("Failed to open %1 for saving selected indices").arg(fileName));
	}
}

///Save selected indices to a file. Ask user for the file name too.
bool QGLPointsCanvas::SaveSelectedIndices() 
{ 
	QString fileName = QFileDialog::getSaveFileName(this,
					"Save indices file" 
					".",
                    "indices file (*.indices *.sel *.txt *.ascii )"); 
	if(!fileName.isEmpty())
	{
		SaveSelectedIndices(fileName);
	}
}

///Save KNN to a file.
bool QGLPointsCanvas::SaveKnnIndices(QString fileName) 
{ 
	std::ofstream outfile ((char*)(fileName.toStdString().c_str()));
	
	if(outfile.is_open() && outfile.good())
	{
		KNNFinder<LaserPoint> finder(laserPoints);
		
		for(int i=0;i<laserPoints.size();i++)
		{
			std::vector<int> indices = finder.FindIndices(laserPoints[i],kNN);
			
			for(int j=0;j<indices.size();j++)
				outfile<<indices[j]<<" ";
			outfile<<endl;		
		}
		outfile.close();	
	}
	else
	{
		ERR_MSG(QString("Failed to open %1 for saving KNN indices").arg(fileName));
	}
}

///Save KNN to a file. Ask for the filename.
bool QGLPointsCanvas::SaveKnnIndices() 
{ 
	QString fileName = QFileDialog::getSaveFileName(this,
					"Save KNN indices file" 
					".",
                    "knn files(*.knn *.txt *.ascii )"); 
	if(!fileName.isEmpty())
	{
		SaveKnnIndices(fileName);
	}
}


///Export vrml
bool QGLPointsCanvas::ExportVRML(QString fileName,bool bPointCloud, bool bTIN, bool bSegmentation,bool bNormals, bool bObjects, bool bAxisIndicator) 
{ 
	///Constructor. We don't need a default one.
	VrmlFile vrmlFile(fileName.toStdString().c_str());
	
	if(!vrmlFile.IsValid())
	{
		ERR_MSG(QString("Failed to export VRML to %1").arg(fileName));
		return 0;
	}
	
	bool useReflectance = (colorMappingMode == UseReflectance);
	if(bPointCloud)
		vrmlFile.Write(laserPoints,colorScheme,useReflectance);
		
	if(bTIN)
		LaserPointsTIN2Vrml(vrmlFile.GetFile(),laserPoints,colorScheme, useReflectance);
		
	if(bSegmentation && segmentedVector.size())
		vrmlFile.Write(laserPoints, segmentedVector, false);
			
	if(bAxisIndicator)
		vrmlFile.WriteAxes3D(laserPoints.Mean());
		
	if(bNormals && normalPoints.size())
		LaserPointsNormals2Vrml(vrmlFile.GetFile(),laserPoints,normalPoints);
		
	if(bObjects && laserObjects.size())
	{
		vrmlFile.Close();
		std::ofstream outFile;
		outFile.open (fileName.toStdString().c_str(), ofstream::out | ofstream::app);
		
		for(int i=0;i<laserObjects.size();i++)
			laserObjects[i]->WriteVRML(outFile);
		outFile.close();
	}
	
	return true;

}

///Export vrml. Ask user for file name and what to save.
bool QGLPointsCanvas::ExportVRML() 
{ 
	QString fileName = QFileDialog::getSaveFileName(this,
					"Save new vrml file",
                    ".",
                    "vrml file(*.wrl *.ascii *.txt )");										
						
	if(!fileName.isEmpty())
	{
		QCheckBox *boxPointCloud = new QCheckBox("Point cloud",this);
		boxPointCloud->setCheckState(Qt::Checked);
		
		QCheckBox *boxTIN = new QCheckBox("TIN",this);
		boxTIN->setCheckState(Qt::Unchecked);
		
		QCheckBox *boxSegmentation = new QCheckBox("Segmentation",this);
		boxSegmentation->setCheckState(Qt::Unchecked);
		
		QCheckBox *boxNormals = new QCheckBox("Normals",this);
		boxNormals->setCheckState(Qt::Unchecked);
		
		QCheckBox *boxObjects = new QCheckBox("Objects",this);
		boxObjects->setCheckState(Qt::Unchecked);
		
		QCheckBox *boxAxisIndicator = new QCheckBox("Axis Indicator",this);
		boxAxisIndicator->setCheckState(Qt::Unchecked);
		
		QDialog* dlg = MakeDialog("Export VRML settings",boxPointCloud,boxTIN,boxSegmentation,
								boxNormals,boxObjects,boxAxisIndicator,NULL);
		
		dlg->exec();           
		
		ExportVRML(fileName,
			boxPointCloud->checkState()& Qt::Checked, 
			boxTIN->checkState()& Qt::Checked, 
			boxSegmentation->checkState()& Qt::Checked,
			boxNormals->checkState()& Qt::Checked,
			boxObjects->checkState()& Qt::Checked,
			boxAxisIndicator->checkState()& Qt::Checked); 
		
		delete dlg;
		//ERR_MSG(QString("Failed to open %1 for exporting VRML").arg(fileName));
	}
	
}


void QGLPointsCanvas::SetLaserObjects(const vector<LaserObject*>& newObjects)
{
	for(int i=0;i<laserObjects.size();i++)
	{
		delete laserObjects[i];
	}
	selectedObjectIndices.clear();
	laserObjects = newObjects;
}

///Return pointer to selected object.
LaserObject* QGLPointsCanvas::GetSelectedObject()const 
{
	if(selectedObjectIndices.size())
	{
		return laserObjects[selectedObjectIndices[0]];
	}
	return NULL;
}

///Delete Selected Object.
void QGLPointsCanvas::DeleteSelectedObject()
{
	if(selectedObjectIndices.size())
	{
		vector<LaserObject*> newObjects;
		for(int i=0;i<laserObjects.size();i++)
		{
			if(i==selectedObjectIndices[0])
			{
				delete laserObjects[selectedObjectIndices[0]];
			}
			else
				newObjects.push_back(laserObjects[i]);
		}
		laserObjects = newObjects;
		selectedObjectIndices.clear();
	}	
}

///Register using ICP.
void QGLPointsCanvas::RegisterICP()
{
	QRegistrationICP::Test();
}

///Register using Indirect model based method.
void QGLPointsCanvas::RegisterIndirect()
{
	QRegistrationIndirect::Test();
}