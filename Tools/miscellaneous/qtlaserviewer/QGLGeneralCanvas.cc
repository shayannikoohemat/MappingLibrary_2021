
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
*   File made : August 2003
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Parent class to handle general canvas related functions.
				Will consist of mainly virtual functions, and serve as a common
				parent of GLContainerCanvas and GLPointsCanvas etc.
*
*--------------------------------------------------------------------*/
#include "QGLGeneralCanvas.h"
#include "OpenGLUtility.h"
#include "GeneralUtility.h"
#include "ExteriorOrientation.h"
#include "Quaternion.h"
#define ENABLE_DEBUGGING 0
#include "DebugFunc.h"
#define Z_MAX_FACTOR 10.0
#define GL_VIEWPOINT_ACTIVE() (selectedViewpoint>=0 && selectedViewpoint<glViewpointVector.size())

#define LINE_WIDTH 1
void
QGLGeneralCanvas::CreateTestForm(int nWidth,int nHeight)
{
	cerr<<"TODO: QGLGeneralCanvas::CreateTestForm\n";
}

void
QGLGeneralCanvas::SetupGLLighting(bool bEnable)
{
	if(bEnable)
	{
		float materialAmbient[] = {0.2,0.2,0.2,1.0};
		float materialDiffuse[] = {1.0,1.0,1.0,1.0};
		float materialSpecular[] = {0.5,0.5,0.5,0.5};
		float materialShininess[] = {120};
	
		glMaterialfv(GL_FRONT_AND_BACK,GL_AMBIENT,materialAmbient);
		glMaterialfv(GL_FRONT_AND_BACK,GL_DIFFUSE,materialDiffuse);
		glMaterialfv(GL_FRONT_AND_BACK,GL_SPECULAR,materialSpecular);
		glMaterialfv(GL_FRONT_AND_BACK,GL_SHININESS,materialShininess);
		glColorMaterial(GL_FRONT_AND_BACK,GL_DIFFUSE);
			
		const float level = 0.9;
		const GLfloat lightAmbient[4]  = {0.2, 0.2, 0.2, 1.0};
		const GLfloat lightSpecular[4] = {0.5, 0.5, 0.5, 1.0};
		const GLfloat lightDiffuse[4]  = {level, level, level, 1.0};
		const GLfloat lightPosition[4] = {0.0,0.0,1.0,0.0};

		glLightfv(GL_LIGHT0, GL_AMBIENT,  lightAmbient);
		glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);
		glLightfv(GL_LIGHT0, GL_DIFFUSE,  lightDiffuse);
		glLightModeli(GL_LIGHT_MODEL_TWO_SIDE,1);
	
		glMatrixMode(GL_MODELVIEW);
		glPushMatrix();
			glLoadIdentity();
			glLightfv(GL_LIGHT0, GL_POSITION,lightPosition);
		glPopMatrix();

		glEnable(GL_LIGHTING);
		glEnable(GL_LIGHT0);
		
		glEnable(GL_COLOR_MATERIAL);
		glEnable(GL_POLYGON_SMOOTH);
		glEnable(GL_NORMALIZE);
	}
	else
	{
		glDisable(GL_LIGHTING);
	}
}

///Creates a new canvas 
QGLGeneralCanvas::QGLGeneralCanvas(QWidget* parent,const char* name)
:QGLWidget(parent)
{
	this->InitializeState();
}
	

//Gives default values to class state variables.
void QGLGeneralCanvas::InitializeState()
{
	captureOn = false;
	lastX = lastY = 0;
	rotationSpeed = 0.5;
	rotationX = rotationY = rotationZ = 15;
	shiftX = shiftY = shiftZ = 0;
	dZoomFactor = 1;
	displayList = 0;
	colorScheme = (GLColorScheme) 1;
	displayType = WireFrame;
	polygonMode = GL_FILL;
	viewingVolumeShiftFactor = 0;
	selectionOn = false;
	colorbarOn = false;
	colorbarRatioX = 0.15;
	colorbarRatioY = 1;
	selectedViewpoint = -1;
	const int maxDim = 10;
	minimumBound = Vector3D(-maxDim,-maxDim,-maxDim);
	maximumBound = Vector3D(maxDim,maxDim,maxDim);
	
	isSlerpRecordingOn = false;
	isNewSlerpInfoOn = false;
	
	this->setMinimumWidth(256);
	this->setMinimumHeight(256);
}

QGLGeneralCanvas::~QGLGeneralCanvas ()
{
	if (displayList)
	{
		ActivateCanvas();
		glDeleteLists (displayList, 1);
	}
}

void
QGLGeneralCanvas::ToggleColorbar ()
{
	SetColorbar(!colorbarOn);
}

void 
QGLGeneralCanvas::SetColorbar(bool state)
{
	colorbarOn = state;
	Repaint ();
}

void
QGLGeneralCanvas::UpdateDisplayList ()
{
	if (isVisible())
	{
		ActivateCanvas();
		if (!displayList)
		{
			displayList = glGenLists (1);
		}
		glNewList(displayList,GL_COMPILE);
		{
			Vector3D max,min,range,mid;
			GetBoundingBox(max,min,range,mid);
			
			Color3D c = GenerateRandomColor();
			glColor3f(c.X(),c.Y(),c.Z());
			//glutSolidSphere(range.X()*0.2,20,20);
			
			//Make a stack of spheres distributed in x,y,z
			//just for testing of General canvas.
			double x,y,z;
			for(x=-10;x<=10;x+=3)
				for(y=-10;y<=10;y+=3)
					for(z=-10;z<=10;z+=3)
			{
				glMatrixMode(GL_MODELVIEW);
				glPushMatrix();
				glTranslatef(x,y,z);
				glColor3f((x+10)/20.00,(y+10)/20.00,(z+10)/20.00);
				glutSolidSphere(1,5,5);
				glPopMatrix();
			}
			
		}			
		glEndList();
	}
}

void 
QGLGeneralCanvas::GetBoundingBox(Vector3D& minimum,Vector3D& maximum,Vector3D& range, Vector3D& middle)
{
	//We don't have any bounds so give some arbitrary values.
	minimum = minimumBound;
	maximum = maximumBound;
	
	//Calculate range etc.
	range = maximum - minimum;
	middle.X() = 0.5*range.X()+minimum.X();
	middle.Y() = 0.5*range.Y()+minimum.Y();
	middle.Z() = 0.5*range.Z()+minimum.Z();
}

void
QGLGeneralCanvas::ApplyGLTransformation ()
{
	//Do all Tranformations.
	if(GL_VIEWPOINT_ACTIVE())
	{
		glViewpointVector[selectedViewpoint].ApplyGLTransformation(GetWidth()/(double)GetHeight(),0.1,10000);
	}
	else
	{
		selectedViewpoint = -1;
		glMatrixMode (GL_PROJECTION);
		glLoadIdentity ();
		SetViewVolume();
	
		//Get bounds and adjust various factors accordingly;
		Vector3D minimum,maximum,range,middle;
		GetBoundingBox(minimum,maximum,range,middle);
		double offsetX, offsetY, offsetZ;
		offsetX = middle.X();
		offsetY = middle.Y();
		offsetZ = middle.Z();
		
		//Apply the transformations.
		glMatrixMode (GL_MODELVIEW);
		glLoadIdentity ();
		glScalef (dZoomFactor, dZoomFactor, dZoomFactor);
		glTranslatef (shiftX, shiftY, shiftZ);
		trackball.MultiplyWithRotationMatrix ();
		glTranslatef (-offsetX, -offsetY, -offsetZ);
	}
}

void
QGLGeneralCanvas::ApplyGLTransformationForPicking (int x, int y, int width,int height)
{
	GLint viewport[4];

	SetDrawing3DViewport();
	//Get the viewport parameters
	glGetIntegerv (GL_VIEWPORT, viewport);

	glMatrixMode (GL_PROJECTION);
	glLoadIdentity ();

	if (width > 0 && height > 0)
	{
		if(colorbarOn)
		{
			gluPickMatrix (x + 0.5 * width, viewport[3] - (y + 0.5 * height),width, height, viewport);
		}
		else
		{
			gluPickMatrix (x + 0.5 * width, viewport[3] - (y + 0.5 * height), width, height, viewport);
		}
	}
	else
	{
		gluPickMatrix (x, viewport[3] - y, 5.0, 5.0, viewport);
	}
	

	//Get bounds and adjust various factors accordingly;
	Vector3D minimum,maximum,range,middle;
	GetBoundingBox(minimum,maximum,range,middle);
	
	SetViewVolume();

	double offsetX, offsetY, offsetZ;
	offsetX = middle.X();
	offsetY = middle.Y();
	offsetZ = middle.Z();

	glMatrixMode (GL_MODELVIEW);
	glLoadIdentity ();
	glScalef (dZoomFactor, dZoomFactor, dZoomFactor);
	glTranslatef (shiftX, shiftY, shiftZ);
	trackball.MultiplyWithRotationMatrix ();
	glTranslatef (-offsetX, -offsetY, -offsetZ);

}

void QGLGeneralCanvas::SetDrawing3DViewport()
{
	int x, y, w, h;
	GetGeometry (x, y, w, h);
	
	if (colorbarOn)
	{
		glViewport (0, 0, (int)((1-colorbarRatioX) * w),(int)(colorbarRatioY*h));
	}
	else
	{
		glViewport (0, 0, w, h);
	}
}

void QGLGeneralCanvas::SetDrawing2DViewport()
{
	int x, y, w, h;
	GetGeometry (x, y, w, h);
	
	glViewport (0, 0, w, h);
}


	
void QGLGeneralCanvas::SetColorbarViewport()
{
	int x, y, w, h;
	GetGeometry (x, y, w, h);
	
	if (colorbarOn)
	{
		glViewport ((int)((1.00-colorbarRatioX)*w), 0, (int)(colorbarRatioX * w), (int)h);
	}
}

void QGLGeneralCanvas::DrawAxisIndicator()
{
	const bool draw3D = true;
	const float volumeDimension = 100;
	const float axisSize = 20;
	const float radius = axisSize/10;
	const float coneRadius = radius*2;
	const float coneHeight = radius*3;
	const int coneStacks = 20;
	const float reverse = 1;//1 for BottomLeft, -1 for TopRight
	float xDim, yDim;
	
	glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
	glClear(GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glLineWidth(LINE_WIDTH);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	
	int canvasWidth = GetWidth();
	int canvasHeight = GetHeight();
	double aspect = 1;
	
	if(colorbarOn)
		canvasWidth -= (colorbarRatioX*canvasWidth);
	
   	//As the size of view port for laser points varies with colorbar
	//so aspect ration calculation needs to be more intelligent.
	if (canvasWidth <= canvasHeight) 
	{
      	aspect = (GLdouble)canvasHeight/(GLdouble)canvasWidth;
      	glOrtho (-volumeDimension, volumeDimension, 
				-volumeDimension*aspect, volumeDimension*aspect,
			 	-volumeDimension*10, volumeDimension*10);  
		xDim = volumeDimension;
		yDim = volumeDimension*aspect;
   }
   else 
   {
      	aspect = (GLdouble)canvasWidth/(GLdouble)canvasHeight;
      	glOrtho (-volumeDimension*aspect, volumeDimension*aspect, 
			-volumeDimension, volumeDimension,
			-volumeDimension*10, volumeDimension*10);
		xDim = volumeDimension*aspect;
		yDim = volumeDimension;		
   }

	glMatrixMode (GL_MODELVIEW);
	glPushMatrix();
		glLoadIdentity ();
		glTranslatef(reverse*(-xDim+axisSize),reverse*(-yDim+axisSize),volumeDimension*0.5);
		trackball.MultiplyWithRotationMatrix ();
	
	QFont font( "Arial", 14, QFont::Bold );		
	if(draw3D)
	{
		//Draw 3d axis indicator from cylinders,cones and cubes.
		SetupGLLighting();
		//X-axis
		glColor3f(1,0,0);
		DrawCylinder(Vector3D(-radius,0,0), Vector3D(axisSize,0,0), radius);
		glDisable(GL_LIGHTING);
		renderText(axisSize*1.2,-radius,0,"X",font);
		glEnable(GL_LIGHTING);
		glPushMatrix();
		glTranslatef(axisSize-radius,0,0);
		glRotatef(90,0,1,0);
		glutSolidCone(coneRadius,coneHeight,coneStacks,coneStacks);
		glPopMatrix();

		//Y-axis
		glColor3f(0,1,0);
		DrawCylinder(Vector3D(0,-radius,0), Vector3D(0,axisSize,0), radius);
		glDisable(GL_LIGHTING);
		renderText(-radius,axisSize*1.2,0,"Y",font);
		glEnable(GL_LIGHTING);
		glPushMatrix();
		glTranslatef(0,axisSize-radius,0);
		glRotatef(270,1,0,0);
		glutSolidCone(coneRadius,coneHeight,coneStacks,coneStacks);
		glPopMatrix();

		//Z-axis
		glColor3f(0,0,1);
		DrawCylinder(Vector3D(0,0,-radius), Vector3D(0,0,axisSize), radius);
		glDisable(GL_LIGHTING);
		renderText(0,-radius,axisSize*1.2,"Z",font);
		glEnable(GL_LIGHTING);
		glPushMatrix();
		glTranslatef(0,0,axisSize-radius);
		glutSolidCone(coneRadius,coneHeight,coneStacks,coneStacks);
		glPopMatrix();
		
		//Box at origin , to hide intersection edges, a bit bigger to fool depth buffer.
		glColor3f(1.0f,1.0f,1.0f);
		glutSolidCube(3*radius);
		
	}
	else
	{
		//draw axis indicator made from lines only.
		glBegin(GL_LINES);
			
			//X-axis
			glColor3f(1.0,0,0);
			glVertex3f(0,0,0);
			glVertex3f(axisSize,0,0);
			
			//Y-axis
			glColor3f(0,1.0,0);
			glVertex3f(0,0,0);
			glVertex3f(0,axisSize,0);
			
			//Z-axis
			glColor3f(0,0,1);
			glVertex3f(0,0,0);
			glVertex3f(0,0,axisSize);
		glEnd();
	}
	glPopMatrix();
	
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glLineWidth(LINE_WIDTH);
}


void QGLGeneralCanvas::DrawDescriptionString()
{
	const float volumeDimension = 100;
	const float axisSize = 12;
	const float reverse = 1;//1 for BottomLeft, -1 for TopRight
	float stroke_scale = 0.08;
	void* bitmap_fonts[7] = {
			GLUT_BITMAP_9_BY_15,
			GLUT_BITMAP_8_BY_13,
			GLUT_BITMAP_TIMES_ROMAN_10,
			GLUT_BITMAP_TIMES_ROMAN_24,
			GLUT_BITMAP_HELVETICA_10,
			GLUT_BITMAP_HELVETICA_12,
			GLUT_BITMAP_HELVETICA_18
			};

	void* stroke_fonts[2] = {
			GLUT_STROKE_ROMAN,
			GLUT_STROKE_MONO_ROMAN
			};
	char msg[] = "QGLGeneralCanvas";
	string description = this->description;
	
	if(GL_VIEWPOINT_ACTIVE())
		description = glViewpointVector[selectedViewpoint].GetName();
	const char* s = description.c_str();
	void* font = stroke_fonts[1];
	
	glDisable(GL_DEPTH_TEST);

	glLineWidth(LINE_WIDTH);
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho (-volumeDimension,volumeDimension,
			-volumeDimension,volumeDimension,
			-volumeDimension,volumeDimension);
		
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix(); 
	glLoadIdentity();
	{
		glTranslatef(-volumeDimension+3*axisSize,-volumeDimension+0.5*axisSize, 0.0);
		glPushMatrix(); 
		{
			glScalef(stroke_scale, stroke_scale, stroke_scale);
			glColor3f(1.0,1.0,0);
			if (s && strlen(s)) 
			{
				while (*s) 
				{
					glutStrokeCharacter(font, *s);
					s++;
				}
			}
		} 
		glPopMatrix();
	} 
	glPopMatrix();
}

void QGLGeneralCanvas::DrawText(double dX, double dY, double dScale,Vector3D textColor,char* szString)
{
	const float volumeDimension = 100;
	const float axisSize = 12;
	const float reverse = 1;//1 for BottomLeft, -1 for TopRight
	float stroke_scale = dScale;
	void* bitmap_fonts[7] = {
			GLUT_BITMAP_9_BY_15,
			GLUT_BITMAP_8_BY_13,
			GLUT_BITMAP_TIMES_ROMAN_10,
			GLUT_BITMAP_TIMES_ROMAN_24,
			GLUT_BITMAP_HELVETICA_10,
			GLUT_BITMAP_HELVETICA_12,
			GLUT_BITMAP_HELVETICA_18
			};

	void* stroke_fonts[2] = {
			GLUT_STROKE_ROMAN,
			GLUT_STROKE_MONO_ROMAN
			};
	char* s = szString;
	void* font = stroke_fonts[1];
	
	glDisable(GL_DEPTH_TEST);
	glViewport(0,0,width(),height());
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	glOrtho (-volumeDimension,volumeDimension,
			-volumeDimension,volumeDimension,
			-volumeDimension,volumeDimension);
		
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix(); 
	glLoadIdentity();
	{
		glTranslatef(volumeDimension*dX,volumeDimension*dY, 0.0);
		glPushMatrix(); 
		{
			glScalef(stroke_scale, stroke_scale, stroke_scale);
			glColor3f(textColor.X(),textColor.Y(),textColor.Z());
			if (s && strlen(s)) 
			{
				while (*s) 
				{
					glutStrokeCharacter(font, *s);
					s++;
				}
			}
		} 
		glPopMatrix();
	} 
	glPopMatrix();
}

int QGLGeneralCanvas::ActivateCanvas()
{
	makeCurrent();
}

void QGLGeneralCanvas::SetViewVolume()
{
	//Get bounds and adjust various factors accordingly;
	Vector3D minimum,maximum,range,middle;
	GetBoundingBox(minimum,maximum,range,middle);
	double widthMax =MAX (range.X(),MAX (range.Y(),range.Z()));
	double aspect = 1;
	widthMax *= 0.6;
	double zMax = Z_MAX_FACTOR*widthMax;
	double viewingShift = viewingVolumeShiftFactor * zMax;
	int canvasWidth = GetWidth();
	int canvasHeight = GetHeight();
	
	
	if(colorbarOn)
		canvasWidth -= (colorbarRatioX*canvasWidth);
	
   	//As the size of view port for laser points varies with colorbar
	//so aspect ration calculation needs to be more intelligent.
	if (canvasWidth <= canvasHeight) 
	{
      aspect = (GLdouble)canvasHeight/(GLdouble)canvasWidth;
      glOrtho (-widthMax, widthMax, -widthMax*aspect, widthMax*aspect,
			 -zMax + viewingShift, zMax + viewingShift);  
   }
   else 
   {
      aspect = (GLdouble)canvasWidth/(GLdouble)canvasHeight;
      glOrtho (-widthMax*aspect, widthMax*aspect, -widthMax, widthMax,
			 -zMax + viewingShift, zMax + viewingShift);
	}

}


void QGLGeneralCanvas::DrawGLObjects()
{	
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glPolygonMode (GL_FRONT_AND_BACK, polygonMode);
	glHint(GL_POINT_SMOOTH_HINT,GL_NICEST);
	glHint(GL_LINE_SMOOTH_HINT,GL_NICEST);

	//Do all Tranformations.
	SetDrawing3DViewport();
	ApplyGLTransformation ();
	
	//update list if not so already.	
	if (!displayList)
		this->UpdateDisplayList();
	
	glEnable(GL_LIGHTING);
	glCallList (displayList);
	
	glDisable(GL_LIGHTING);
	glDisable(GL_DEPTH_TEST);
	

}

string
QGLGeneralCanvas::GetDescription()
{
	return description;	
}

void
QGLGeneralCanvas::SetDescription(string s)
{
	description = s;
	Repaint();
}


void QGLGeneralCanvas::DrawGLSelection()
{
	DEBUG("QGLGeneralCanvas::DrawGLSelection() ");

	//Do all Tranformations.
	SetDrawing3DViewport();
	ApplyGLTransformation ();
	
	glEnable(GL_LIGHTING);
	//Draw selection drawing here.
}

void QGLGeneralCanvas::DrawColorbar()
{
	if (colorbarOn)
	{
		SetColorbarViewport();
		glMatrixMode (GL_PROJECTION);
		glLoadIdentity ();
		gluOrtho2D (-0.2, 1.2, -0.2, 1.2);
		glPolygonMode (GL_FRONT_AND_BACK, GL_FILL);
		glMatrixMode (GL_MODELVIEW);
		glLoadIdentity ();
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);
		::DrawColorbar(colorScheme);
		SetDrawing3DViewport();
	}
}

void
QGLGeneralCanvas::Repaint ()
{
	DEBUG("Repaint");
	updateGL();	
	DEBUG("~~~Repaint");	
}
#define DEBUG_QT_MESSAGE(a) //{static int count ;cerr<<"QGLGeneralCanvas::"<<#a<<" : "<<count++<<endl;}
#define PROCESS_QT_MESSAGE(a) {DEBUG_QT_MESSAGE(a);return QGLWidget::a(e);}


void
QGLGeneralCanvas::initializeGL ( )
{
	UpdateClearColor();
}
void 
QGLGeneralCanvas::mousePressEvent ( QMouseEvent * e )
{
	Repaint();
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
		if (SHIFT_ON || CONTROL_ON)
				selectionOn = true;
	}
	
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
	PROCESS_QT_MESSAGE(mousePressEvent);
}
void 
QGLGeneralCanvas::mouseReleaseEvent ( QMouseEvent * e )
{
	Repaint();
	int x = e->x();
	int y = e->y();
	Qt::KeyboardModifiers state = e->modifiers();
	Qt::MouseButtons  button = e->buttons();
	
	if (captureOn && selectionOn)
	{
		//Get the selection in a new vector.
		IndicesVector currentSelection;
		if(lastX > x)
		{
			int temp = x;
			x = lastX;
			lastX = temp;
		}
		
		if(lastY > y)
		{
			int temp = y;
			y = lastY;
			lastY = temp;
		}
		
		WindowSelectObjects(lastX, lastY,
				(int)std::abs ((int)x - (int)lastX),
				(int)std::abs ((int)y - (int)lastY),
				currentSelection);
							  
		Repaint ();
	}
	
	//End the capture and selection mode.
	captureOn = false;
	selectionOn = false;

	
	//Call user defined callback if assigned.
	if((button & Qt::MidButton) && ((abs (x - startX)<5&& std::abs ((int)y - (int)startY)<5)))
	{
		//Show pop-up menu with viewpoint list
		ShowViewpointPopupMenu();
	}
	//Release mouse grab.
	releaseMouse();
	unsetCursor();
	PROCESS_QT_MESSAGE(mouseReleaseEvent);
}
	
void 
QGLGeneralCanvas::mouseDoubleClickEvent ( QMouseEvent * e )
{
	Repaint();
	PROCESS_QT_MESSAGE(mouseDoubleClickEvent)
}

void 
QGLGeneralCanvas::mouseMoveEvent ( QMouseEvent * e )
{
	int x = e->x();
	int y = e->y();
	int w = width();
	int h = height();
	Qt::KeyboardModifiers state = e->modifiers();
	Qt::MouseButtons  button = e->buttons();

	if (captureOn && selectionOn)
	{
		Repaint ();
		SetDrawing2DViewport();
		glPolygonMode (GL_FRONT_AND_BACK, GL_LINE);
		glDrawBuffer (GL_FRONT);
		glMatrixMode (GL_PROJECTION);
		glLoadIdentity ();
		gluOrtho2D (0, w, 0, h);

		glDisable (GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);
		glMatrixMode (GL_MODELVIEW);
		glLoadIdentity ();
		
		if(CONTROL_ON && SHIFT_ON)
			glColor3f(1.0,1.0,1.0);
		else if(CONTROL_ON)
			glColor3f(0.0,1.0,0.0);
		else if(SHIFT_ON)
			glColor3f(1.0,0.0,0.0);
		
		glLineWidth(SELECTION_OUTLINE_WIDTH);
		glBegin (GL_QUADS);
		glVertex2f (lastX, h - lastY);
		glVertex2f (x, h - lastY);
		glVertex2f (x, h - y);
		glVertex2f (lastX, h - y);
		glEnd ();
		glFlush ();
		glDrawBuffer (GL_BACK);
	}

	else if (captureOn && (lastButton & Qt::LeftButton))
	{
		DEBUG("Processing rotation  ");
		trackball.ProcessMouse (lastX,lastY, x, y, w, h);
		lastX = x;
		lastY = y;
		Repaint ();
	}

	else if (captureOn && (lastButton & Qt::MidButton))
	{
		shiftX +=GetNormalizedShift (x - lastX);
		shiftY -=GetNormalizedShift (y - lastY);
		lastX = x;
		lastY = y;
		Repaint ();
	}

	else if (captureOn && (lastButton & Qt::RightButton))
	{
		if (y > lastY)
		{
			dZoomFactor *= (1.00 +  GetNormalizedZoom (std::abs ((int)y - (int)lastY)));
		}
		else
		{
			dZoomFactor /= (1.00 + GetNormalizedZoom (std::abs((int)y - (int)lastY)));
		}
		lastX = x;
		lastY = y;
		Repaint ();
	}
	setFocus();
	PROCESS_QT_MESSAGE(mouseMoveEvent)
}


void 
QGLGeneralCanvas::wheelEvent ( QWheelEvent * e )
{
	PROCESS_QT_MESSAGE(wheelEvent)
}

void 
QGLGeneralCanvas::keyPressEvent ( QKeyEvent * e )
{
	Qt::KeyboardModifiers state = e->modifiers();
	int key = e->key();
	
	switch (key)
	{
	case Qt::Key_A:
	{
		if(CONTROL_ON && SHIFT_ON)
		{
			BeginSlerpRecording();
		}
	}
	break;
	
	case Qt::Key_Q:
	{
		if(CONTROL_ON && SHIFT_ON)
		{
			EndSlerpRecording();
		}
	}
	break;

	case Qt::Key_K:
	{
		if(CONTROL_ON && SHIFT_ON)
		{
			SaveSlerpFrames("TestSlerpFrames");
		}
	}
	break;

	

	
	case Qt::Key_X:
	if(CONTROL_OFF && SHIFT_OFF && ALT_OFF)
	{
		//rotate to x view.
		trackball.SetAngleAxis(90,0,1,0);
		Repaint();
	}
	break;
	
	case Qt::Key_Y:
	if(CONTROL_OFF && SHIFT_OFF && ALT_OFF)
	{
		//rotate to y view
		trackball.SetAngleAxis(-90,1,0,0);
		Repaint();
	}
	break;
	
	case Qt::Key_Z:
	if(CONTROL_OFF && SHIFT_OFF && ALT_OFF)
	{
		//rotate to z view
		trackball.SetAngleAxis(90,0,0,1);
		Repaint();
	}
	break;
	
	case Qt::Key_F:
	if(CONTROL_OFF && SHIFT_OFF && ALT_OFF)
	{
		//Toggle filling for tin.
		if (polygonMode == GL_FILL)
			polygonMode = GL_LINE;
		else if(polygonMode == GL_LINE)
			polygonMode = GL_POINT;
		else
			polygonMode = GL_FILL;
		Repaint ();
	}
	break;

	case Qt::Key_Up:
	if(CONTROL_OFF && SHIFT_OFF && ALT_OFF)
	{
		//Shift the viewing volume along z. Shifts the clipping planes.
		viewingVolumeShiftFactor +=	(0.1 * ((CONTROL_ON) ? 0.2 : 1));
		Repaint ();
	}
	break;

	case Qt::Key_Down:
	if(CONTROL_OFF && SHIFT_OFF && ALT_OFF)
	{
		//Shift the clipping planes in opposite direction.
		viewingVolumeShiftFactor -= (0.1 * ((CONTROL_ON) ? 0.2 : 1));
		Repaint ();
	}
	break;

	case Qt::Key_R:
		//Reset the view.
		if (CONTROL_ON)
		{
			ResetView ();
		}
	break;
	
	//Print to EPS file.
	case Qt::Key_P:
	{
		if(CONTROL_ON && SHIFT_ON)
		{
			QFileDialog* fd = new QFileDialog( this);
    		fd->setFileMode(QFileDialog::AnyFile);
  
                       QStringList filters;
                       filters << "*.eps";
			fd->setNameFilters(filters);
  
			fd->setViewMode( QFileDialog::Detail );
  
			QStringList fileName;
			if ( fd->exec() == QDialog::Accepted )
			{
				fileName = fd->selectedFiles();
				WriteEPS(fileName[0].toStdString().c_str());
			}
		}
	}
	break;
	}//End switch
	PROCESS_QT_MESSAGE(keyPressEvent)
}

void 
QGLGeneralCanvas::keyReleaseEvent ( QKeyEvent * e )
{
	PROCESS_QT_MESSAGE(keyReleaseEvent)
}

void 
QGLGeneralCanvas::focusInEvent ( QFocusEvent * e)
{
	PROCESS_QT_MESSAGE(focusInEvent)
}

void 
QGLGeneralCanvas::focusOutEvent ( QFocusEvent * e)
{
	PROCESS_QT_MESSAGE(focusOutEvent)
}


void 
QGLGeneralCanvas::enterEvent ( QEvent * e)
{
	setFocus();
	PROCESS_QT_MESSAGE(enterEvent)
}

void 
QGLGeneralCanvas::leaveEvent ( QEvent * e)
{
	clearFocus();
	PROCESS_QT_MESSAGE(leaveEvent)
}


void 
QGLGeneralCanvas::paintGL( )
{
	DEBUG("QGLGeneralCanvas::paintGL ()");
	
	if (!isVisible())
	{
		DEBUG("!!!QGLGeneralCanvas::paintGL(): form is not visible or canvas is hidden ");
		return;
	}
	
	ActivateCanvas();
	SetupGLLighting();
	glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
		DrawGLObjects();
		DrawGLSelection();	
		DrawAxisIndicator();
		glDisable(GL_LIGHTING);
		DrawDescriptionString();
		DrawColorbar();
	glFlush ();
	
	//According the documentation its not necessary to call swap buffers
	//as its automatically done by OnPaint
	//glXSwapBuffers (fl_display, fl_get_canvas_id (canvas));
	//swapBuffers();
	DEBUG("QGLGeneralCanvas::~~~paintGL");
}

void 
QGLGeneralCanvas::swapBuffers( )
{
	DEBUG("Swap Buffers");
	QGLWidget::swapBuffers();
}

void 
QGLGeneralCanvas::closeEvent ( QCloseEvent * e )
{
	PROCESS_QT_MESSAGE(closeEvent)
}


void 
QGLGeneralCanvas::contextMenuEvent ( QContextMenuEvent * e )
{
	PROCESS_QT_MESSAGE(contextMenuEvent)
}

void 
QGLGeneralCanvas::dragEnterEvent ( QDragEnterEvent * e)
{
	PROCESS_QT_MESSAGE(dragEnterEvent)
}


void 
QGLGeneralCanvas::dragMoveEvent ( QDragMoveEvent * e)
{
	PROCESS_QT_MESSAGE(dragMoveEvent)
}


void 
QGLGeneralCanvas::dragLeaveEvent ( QDragLeaveEvent * e)
{
	PROCESS_QT_MESSAGE(dragLeaveEvent)
}


void 
QGLGeneralCanvas::dropEvent ( QDropEvent * e)
{
	PROCESS_QT_MESSAGE(dropEvent)
}


void 
QGLGeneralCanvas::showEvent ( QShowEvent * e)
{
	ActivateCanvas();
	int x, y, width, height;
	GetGeometry (x, y, width, height);
	glViewport (0, 0, (GLint) width, (GLint) height);
	UpdateClearColor ();
	UpdateDisplayList ();
	Repaint ();
	PROCESS_QT_MESSAGE(showEvent)
}

void 
QGLGeneralCanvas::hideEvent ( QHideEvent * e)
{
	PROCESS_QT_MESSAGE(hideEvent)
}


int QGLGeneralCanvas::GetWidth()
{
	int x,y,w,h;
	GetGeometry(x,y,w,h);
	return w;
}
int QGLGeneralCanvas::GetHeight()
{
	int x,y,w,h;
	GetGeometry(x,y,w,h);
	return h;
}	
int QGLGeneralCanvas::GetTop()
{
	int x,y,w,h;
	GetGeometry(x,y,w,h);
	return y;
}	
int QGLGeneralCanvas::GetLeft()
{
	int x,y,w,h;
	GetGeometry(x,y,w,h);
	return x;
}		

///Prints to Postscript.
void 
QGLGeneralCanvas::WriteEPS(const char* fileName)
{
	int format = GL2PS_EPS;
	int sort = GL2PS_BSP_SORT;
	int options = GL2PS_NO_PS3_SHADING|GL2PS_SIMPLE_LINE_OFFSET | GL2PS_SILENT|GL2PS_OCCLUSION_CULL | GL2PS_BEST_ROOT;//| GL2PS_DRAW_BACKGROUND;		int nbcol, 
	int nbcol = 0;
	char  file[1024];
	
	for(int count =0 ; count <2; count++)
	{
		format = (count)?GL2PS_EPS:GL2PS_PS;
		sprintf(file,"%s.%s",fileName,(count)?"eps":"ps");
		FILE *fp;
		int state = GL2PS_OVERFLOW, buffsize = 0;
		GLint viewport[4];

		//Activate the canvas.
		ActivateCanvas();

		viewport[0] = 0;
		viewport[1] = 0;
		viewport[2] = GetWidth();
		viewport[3] = GetHeight();

		fp = fopen(file, "w");
		if(!fp)
		{
			fprintf(stderr,"Unable to open file %s for writing\n", file);
			return;
		}

		fprintf(stderr,"Saving Postscript to file %s... \n", file);
		while(state == GL2PS_OVERFLOW)
		{
			int oneMB = 1024*1024;
			buffsize = (10*oneMB);
			gl2psEnable(GL2PS_LINE_STIPPLE|GL2PS_POLYGON_OFFSET_FILL);
			gl2psBeginPage(file, "test", viewport, format, sort, options,
				GL_RGBA, 0, NULL, nbcol, nbcol, nbcol, 
				buffsize, fp, file);
			ActivateCanvas();
			Repaint();
			state = gl2psEndPage();
		}

		fclose(fp);
		fprintf(stderr,"Done!\n");

		char command[1024];
		sprintf(command,"kghostview %s &",file);
		fprintf(stderr,"Executing: %s \n");
		system(command);
	}
}

void
QGLGeneralCanvas::GetGeometry (int &x, int &y, int &width, int &height)
{
	QRect rect = geometry();
	x = rect.x(); y = rect.y();
	width = rect.width(); height = rect.height(); 
}

void
QGLGeneralCanvas::SetGeometry (int x, int y, int width, int height)
{
	setGeometry(x, y, width, height);
}

double
QGLGeneralCanvas::GetNormalizedShift (double shiftInPixels)
{
	int x, y, canvasWidth, canvasHeight;
	GetGeometry (x, y, canvasWidth, canvasHeight);
	Vector3D minimum,maximum,range,middle;
	GetBoundingBox(minimum,maximum,range,middle);
		
	double widthMax = MAX (range.X(),MAX(range.Y(),range.Z()));
	return shiftInPixels * widthMax / (double) canvasWidth;
}

double
QGLGeneralCanvas::GetNormalizedZoom (double shiftInPixels)
{
	int x, y, canvasWidth, canvasHeight;
	GetGeometry (x, y, canvasWidth, canvasHeight);

	Vector3D minimum,maximum,range,middle;
	GetBoundingBox(minimum,maximum,range,middle);
	
	double widthMax = MAX (range.X(),MAX(range.Y(),range.Z()));
	return shiftInPixels / (double) canvasHeight;
}

void
QGLGeneralCanvas::ShowViewpointPopupMenu()
{
	if(glViewpointVector.empty())
		return;
	/*
	Window ParentWindow = FL_ObjWin(canvas);
	const int maxEntries = 2096;
	fl_setpup_maxpup(maxEntries);
	char buff[256];
	
	FL_PUP_ENTRY entries[maxEntries]={{0}};
	string titles[maxEntries];
	string shortcuts[maxEntries];
	
	//Populate the popup.
	int i;
	for(i=0;i<glViewpointVector.size()&&i<maxEntries;i++)
	{
		buff[0]='a'+i; buff[1]=(char)NULL;
		shortcuts[i] = string(buff);
		titles[i] = glViewpointVector[i].GetName();// + string("  ")+shortcuts[i];
		entries[i].text = titles[i].c_str();
		entries[i].mode = FL_PUP_RADIO|((i==selectedViewpoint)?FL_PUP_CHECK:FL_PUP_NONE);
		entries[i].shortcut = shortcuts[i].c_str();
	}
	
	//Add one entry to come back to normal view.
	buff[0]='q'; buff[1]=(char)NULL;
	shortcuts[i] = string(buff);
	titles[i] = string("Normal View")+ string("  ")+shortcuts[i];
	entries[i].text = titles[i].c_str();
	entries[i].mode = FL_PUP_NONE;
	entries[i].shortcut = shortcuts[i].c_str();

	
	//create popup menu.
	int menu = fl_newpup (ParentWindow);
	fl_setpup_entries (menu, entries);

	//Display popup
	int nSelectedIndex = fl_dopup (menu);
	
	//Take action if something is selected.
	//Note that the index starts from 1 so must subtract 1 to go back to zero based.
	if (nSelectedIndex >= 1)
	{
		selectedViewpoint = nSelectedIndex-1;
		Repaint();
	}
	
	//Don't forget to free the pup.
	fl_freepup (menu);
	*/
	cerr<<"TODO QGLGeneralCanvas::ShowViewpointPopupMenu\n";

}

void
QGLGeneralCanvas::Show ()
{
	show();
}

void
QGLGeneralCanvas::ResetView ()
{
	shiftX = shiftY = shiftZ = 0;
	dZoomFactor = 1;
	displayType = WireFrame;
	polygonMode = GL_FILL;
	viewingVolumeShiftFactor = 0;
	selectionOn = false;
	trackball = VirtualTrackball();
	trackball.SetAngleAxis(0,1,1,1);
	
	Repaint();
}

GLViewpointVector&
QGLGeneralCanvas::GetGLViewpointVector()
{
	return glViewpointVector;
}

void 
QGLGeneralCanvas::SetGLViewpointVector(GLViewpointVector& vpVector)
{
	glViewpointVector = vpVector;
}



void
QGLGeneralCanvas::Hide ()
{
	hide();
}

void
QGLGeneralCanvas::SetDisplayType (DisplayType d)
{
	displayType = d;
	UpdateDisplayList ();
	Repaint ();
}


int
QGLGeneralCanvas::WindowSelectObjects (int x1, int y1, int width, int height,IndicesVector& selectedIndices)
{
	DEBUG ("QGLGeneralCanvas::WindowSelectObjects");
		//Nothing to select in a generic viewer.
	return 0;
}

///Set bounding box.
void
QGLGeneralCanvas::SetBoundingBox(Vector3D minimum,Vector3D maximum)
{
	maximumBound = maximum;
	minimumBound = minimum;
}

/// converts to string
string
QGLGeneralCanvas::ToString()
{
	char buff[1024];
	snprintf(buff,1024,"QGLGeneralCanvas %p",this);
	return string(buff);
}

///Writes a vrml file.
void 
QGLGeneralCanvas::WriteVrml(const char* fileName)
{
	cerr<<"Cannot write vrml file "<<fileName<<" for QGLGeneralCanvas\n";
}

///Writes frame buffer to a given image file with a specified format.
///If drawArrow is true an arrow is drawn at current mouse position
void 
QGLGeneralCanvas::WriteFrameBuffer(const char* fileName,const char* format,bool drawArrow)
{
	this->Repaint();
	cerr<<"TODO QGLGeneralCanvas::WriteFrameBuffer doesn't have SaveColorBuffer\n";
	//SaveColorBuffer((char*)fileName,(char*)format,GetCanvas(),drawArrow);
}
///Slerp based frame recording functions.
///Resets the slerpInfos array, and sets the flag for slerp based recording.
void 
QGLGeneralCanvas::BeginSlerpRecording()
{
	if(slerpInfoVector.size())
	{
		cerr<<"Do you want to start a new slerp session? Old settings would be lost\n";
		cerr<<"Enter zero to abort anything else to continue:\n";
		int input;
		cin>>input;
		
		if(input)
		{		
			cerr<<"Starting a new slerp recording session\n";
			isSlerpRecordingOn = true;
			isNewSlerpInfoOn = false;
			slerpInfoVector.clear();
		}
	}
	else
	{
		cerr<<"Starting a new slerp recording session\n";
		isSlerpRecordingOn = true;
		isNewSlerpInfoOn = false;
	}		
}

	
///Turns off slerp info recording. Once its off its possible to save frames 
///slerps which are already recorded by calling SaveSlerpFrames	
void 
QGLGeneralCanvas::EndSlerpRecording()
{
	if(isNewSlerpInfoOn)
		cerr<<"Please finish current slerp before ending the recording session\n";
	else
	{
		isSlerpRecordingOn = false;
		cerr<<"Slerp recording session finished with "<<slerpInfoVector.size()<<" iterms\n";
		cerr<<"You can save frames if you want\n";
	}
}

///Inserts a new record into slerp array, using the current state as the start 	
void 
QGLGeneralCanvas::BeginNewSlerpInfo()
{
	if(isSlerpRecordingOn)
	{
		if(isNewSlerpInfoOn)
		{
			cerr<<"Please finish current slerp before starting a new one\n";
		}
		else
		{
			cerr<<"Starting a new slerp info record\n";
			trackball.GetQuaternion(slerpQuaternion0);
			slerpTranslation0=Vector3D(shiftX,shiftY,shiftZ);
			isNewSlerpInfoOn = true;
		}
	}		 
}
	
///Finishes current slerp info using current state as final parameter.
void 
QGLGeneralCanvas::EndNewSlerpInfo()
{
	if(isSlerpRecordingOn)
	{
		if(isNewSlerpInfoOn)
		{
			trackball.GetQuaternion(slerpQuaternion1);
			slerpTranslation1=Vector3D(shiftX,shiftY,shiftZ);
			isNewSlerpInfoOn = false;
			
			slerpInfoVector.push_back(SlerpInfo(slerpQuaternion0,slerpQuaternion1,slerpTranslation0,slerpTranslation1));
			
			cerr<<"New slerp info record finished\n";
		}
	}
}

			
	
///Saves frames using the full array.
///framesPerSlerp*slerpInfoVector.size() frames are stored.
void 
QGLGeneralCanvas::SaveSlerpFrames(const char* fileNameHeader, int framesPerSlerp)
{

	framesPerSlerp = (framesPerSlerp>0)?framesPerSlerp:5;
	double stepSize = 1.00/(double)framesPerSlerp;
	int frameCounter = 0;
	char fileName[1024];
	
	for(int i=0;i<slerpInfoVector.size();i++)
	{
		for(double step=0;step<=1;step+=stepSize)
		{
			slerpInfoVector[i].ApplySlerp(step,trackball,shiftX,shiftY,shiftZ);	
			
			sprintf(fileName,"%s_%d",fileNameHeader,frameCounter+10000);
			cerr<<"SaveSlerpFrames: saving "<<fileName<<endl;
			WriteFrameBuffer(fileName);
			frameCounter++;

		}
	}
}

///Sets view from x-axis.
void QGLGeneralCanvas::SetViewFromX() 
{
	//rotate to x view.
	trackball.SetAngleAxis(90,0,1,0);
	Repaint();
}

///Sets view from y-axis.
void QGLGeneralCanvas::SetViewFromY()
{
	//rotate to y view
	trackball.SetAngleAxis(90,1,0,0);
	Repaint();
}


///Sets view from z-axis.
void QGLGeneralCanvas::SetViewFromZ()
{
	trackball.SetAngleAxis(90,0,0,1);
	Repaint();
}

void QGLGeneralCanvas::DrawColorbarMinMax(double Min,double Max)
{
	if (colorbarOn)
	{
		//cerr<<"Drawing colorbar limits "<<Max<<"  "<<Min<<"\n";
		glDisable(GL_DEPTH_TEST);
		glDisable(GL_LIGHTING);
		
		int x, y, w, h;
		GetGeometry (x, y, w, h);
		glViewport (0, 0, w,h);
		
		char buffer[4096];
		sprintf(buffer,"%6.3g",Min);
		DrawText(0.75,-0.8, 0.03,Vector3D(1,1,1),buffer)	;
		
		sprintf(buffer,"%6.3g",Max);
		DrawText(0.75,0.75, 0.03,Vector3D(1,1,1),buffer);	
		
		SetDrawing3DViewport();
	}
}

///Sets a new background color.
void QGLGeneralCanvas::SetBackgroundColor(QColor c)
{
	backgroundColor = c;
	paintGL();
}
	
///Asks the user to specify a new background color.
void QGLGeneralCanvas::GetNewBackgroundColor()
{
	backgroundColor = QColorDialog::getColor(backgroundColor);
	UpdateClearColor();
	update();
}

//specify clear color to gl.
void QGLGeneralCanvas::UpdateClearColor()
{
	glClearColor(backgroundColor.red()/255.00,backgroundColor.green()/255.00,backgroundColor.blue()/255.00,0);
}	

//specify a color to OpenGl.
void QGLGeneralCanvas::GLColor(Vector3D v)
{
	for(int i=0;i<3;i++)
	{
		v[i] = (v[i]<0)?0:v[i];
		v[i] = (v[i]>1)?1:v[i];
	}
	glColor3f(v[0],v[1],v[2]);
}

//specify a color to OpenGL.
void QGLGeneralCanvas::GLColor(QColor c)
{
	glColor3f(c.red()/255.00,c.green()/255.00,c.blue()/255.00);
}

///Sets polygon mode.
void QGLGeneralCanvas::SetPolygonMode(int p)
{
	polygonMode = p;
}




