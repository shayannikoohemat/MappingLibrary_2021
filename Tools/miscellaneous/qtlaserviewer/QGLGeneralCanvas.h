
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
				parent of QGLContainerCanvas and QGLPointsCanvas etc.
*				This class in similar to GLGeneralCanvas but uses Qt instead of xforms.				
*
*--------------------------------------------------------------------*/
#ifndef _QGL_GENERAL_CANVAS_H_
#define _QGL_GENERAL_CANVAS_H_
#include <GL/glut.h>
#include <QtGui>
#include <QtOpenGL>

#include <math.h>
#include <iostream>
#include <Vector3D.h>
#include "Trackball.h"
#include "Palettes.h"
#include "MyConstants.h"
#include "MyTypeDefinitions.h"
#include "GLViewpoint.h"

#define SELECTION_OUTLINE_WIDTH 	2 //width of lines used to draw selection rectangles, and polylines
#define SHIFT_ON (state & Qt::ShiftModifier)
#define CONTROL_ON (state & Qt::ControlModifier)
#define ALT_ON (state & Qt::AltModifier)

#define SHIFT_OFF !(SHIFT_ON)
#define CONTROL_OFF !(CONTROL_ON)
#define ALT_OFF !(ALT_ON)

#define LEFT_PRESS (button & Qt::LeftButton)
#define RIGHT_PRESS (button & Qt::RightButton)
#define MID_PRESS (button & Qt::MidButton)

using namespace std;

class QGLGeneralCanvas: public QGLWidget
{
	Q_OBJECT
public:
	///Creates a new canvas 
	QGLGeneralCanvas(QWidget* parent,const char* name="");
	
	///Destructor.
	~QGLGeneralCanvas();
public slots:
	///Sets polygon mode.
	void SetPolygonMode(int polygonMode);
	
	///Prints to Postscript.
	virtual void WriteEPS(const char* fileName);
	
	///Reset the view to default parameters.
	void ResetView();
	
	///Toggles color bar.
	void ToggleColorbar();
	
	///Toggles color bar.
	void SetColorbar(bool state);
	
	///Sets view from x-axis.
	void SetViewFromX() ;
	
	///Sets view from y-axis.
	void SetViewFromY() ;
	
	///Sets view from z-axis.
	void SetViewFromZ() ;
	
	///Sets a new background color.
	void SetBackgroundColor(QColor c);
	
	///Asks the user to specify a new background color.
	void GetNewBackgroundColor();
	
public:
	///static function to test QGLGeneralCanvas on a form.
	virtual void CreateTestForm(int nWidth=512,int nHeight=512);

	///Update disply list.
	virtual void UpdateDisplayList();
	
	//specify clear color to gl.
	void UpdateClearColor();
	
	//specify a color to OpenGl.
	void GLColor(Vector3D v);
	
	//specify a color to OpenGL.
	void GLColor(QColor c);
	
	///Show the canvas
	virtual void Repaint();
	
	///Select object in a given window.
	virtual int WindowSelectObjects(int x1,int y1,int width,int height,IndicesVector& selectedIndices);
		
	///Gets bounding box of the view volume.	
	virtual void GetBoundingBox(Vector3D& minimun,Vector3D& maximum,Vector3D& range,Vector3D& middle);
	
	///Set bounding box.
	virtual void SetBoundingBox(Vector3D minimum,Vector3D maximum);
	
	///Shows a popup menu with all available viewpoints. Changes view point if something is selected.
	void ShowViewpointPopupMenu();	
		
	///Converts the object to a string. Useful for debugging.
	virtual string ToString();

	///Applies GL transformation 
	virtual void ApplyGLTransformation();
	
	///Sets up lighting etc.
	void SetupGLLighting(bool bEnable=true);
	
	///Gets view point vector
	GLViewpointVector& GetGLViewpointVector();
	
	///Sets view point vector.
	void SetGLViewpointVector(GLViewpointVector& vpVector);
	
	///Draws a 2D rectangle at given coordinates in given color.
	void Draw2DRectangle(double x,double y,double width,double height,Color3D color);
	
	///Draws text at given points.
	void DrawText(double dX, double dY, double dScale,Vector3D textColor,char* szString);
	
	///Gets normalized shift.
	double GetNormalizedShift(double shiftInPixels);
	
	///Gets normalized Zoom.
	double GetNormalizedZoom(double shiftInPixels);
	
	///Sets canvas geometry.
	void SetGeometry(int x,int y,int width, int height);
	
	///Gets canvas geometry.
	void GetGeometry(int& x,int& y,int& width, int& height);
	
	///Checks for canvas and form visibility.
	bool IsVisible();
	
	
	///Writes a vrml file.
	virtual void WriteVrml(const char* fileName);
	
	///Writes frame buffer to a given image file with a specified format.
	///If drawArrow is true an arrow is drawn at current mouse position
	virtual void WriteFrameBuffer(const char* fileName,const char* format = "jpg",bool drawArrow = false);
	
	///Sets display type for filled objects.
	enum DisplayType{WireFrame=1,Filled=2,Smooth=4,Points=8};
	void SetDisplayType(DisplayType d);
	
	///Gets the description string.
	string GetDescription();
	
	///Sets the description string.
	void SetDescription(string s);
	
	///Show the canvas.
	void Show();
	
	///Hide the canvas.
	void Hide();
	
	///Apply GL Transformation for picking.
	void ApplyGLTransformationForPicking(int x,int y,int width,int height);
	
	///Returns width of the canvas.
	int GetWidth();
	
	///Return height of the canvas.
	int GetHeight();
	
	///Get top y of the canvas. (Bottom = GetTop()+GetHeight())
	int GetTop();
	
	///Get Left x of canvas. (Right = GeLeft()+GetWidth())
	int GetLeft();
	
protected:
	///Initialize state variables.
	virtual void InitializeState();	
	void SetDrawing3DViewport();
	void SetDrawing2DViewport();
	void SetColorbarViewport();
	void DrawAxisIndicator();
	virtual void DrawDescriptionString();
	virtual void DrawGLObjects();
	virtual void DrawGLSelection();
	void DrawColorbar();
	void DrawColorbarMinMax(double Min,double Max);

	int ActivateCanvas();
	void SetViewVolume();
	
	void swapBuffers();
	void paintGL();
	//We will override some of the virtual functions.
	//Inside they will call our own exposed virtual functions.
	void mousePressEvent ( QMouseEvent * e );
	void mouseReleaseEvent ( QMouseEvent * e );
	void mouseDoubleClickEvent ( QMouseEvent * e );
	void mouseMoveEvent ( QMouseEvent * e );
 	void wheelEvent ( QWheelEvent * e );
 	void keyPressEvent ( QKeyEvent * e );
 	void keyReleaseEvent ( QKeyEvent * e );
 	void focusInEvent ( QFocusEvent * e);
 	void focusOutEvent ( QFocusEvent * e);
 	void enterEvent ( QEvent * e);
 	void leaveEvent ( QEvent * e);
 	void closeEvent ( QCloseEvent * e );
	void contextMenuEvent ( QContextMenuEvent * e );
	void dragEnterEvent ( QDragEnterEvent * e);
	void dragMoveEvent ( QDragMoveEvent * e);
	void dragLeaveEvent ( QDragLeaveEvent * e);
	void dropEvent ( QDropEvent * e);
	void showEvent ( QShowEvent * e);
	void hideEvent ( QHideEvent * e);
	
	///Responsible for initialization. 
	void initializeGL ();
			
	//Data members.
	GLuint displayList;
		
	VirtualTrackball trackball;
	
	//State variables.
	bool captureOn;
	int startX;
	int startY;
	int lastX;
	int lastY;
	Qt::KeyboardModifiers lastState;
	Qt::MouseButtons lastButton;
	double rotationSpeed;
	double rotationX;
	double rotationY;
	double rotationZ;
	double shiftX;
	double shiftY;
	double shiftZ;
	double dZoomFactor;
	GLColorScheme colorScheme;
	GLuint polygonMode;
	DisplayType displayType;
	double viewingVolumeShiftFactor;
	bool selectionOn;
	bool colorbarOn;
	float colorbarRatioX;
	float colorbarRatioY;
	string description;
	
	///View point list and selection index.
	GLViewpointVector glViewpointVector;
	int selectedViewpoint;
	
	///Maximum and minimum bounds.
	Vector3D minimumBound;
	Vector3D maximumBound;
	
	//Slerp based recording section.
	struct SlerpInfo
	{
		double q0[4];
		double q1[4];
		Vector3D trans0;
		Vector3D trans1;
		SlerpInfo()
		{
			for(int i=0;i<4;i++)
			{
				q0[i]=1;
				q1[i]=2;
			}
			trans0 = Vector3D(0,0,0);
			trans1 = Vector3D(3,3,-3);
		}
		SlerpInfo(double dq0[],double dq1[],Vector3D t0,Vector3D t1):
				trans0(t0),trans1(t1)
		{
			for(int i=0;i<4;i++)
			{
				q0[i]=dq0[i];
				q1[i]=dq1[i];
			}
		};
	
		SlerpInfo(const SlerpInfo& b)
		{
			for(int i=0;i<4;i++)
			{
				q0[i]=b.q0[i];
				q1[i]=b.q1[i];
			}
			trans0 = b.trans0;
			trans1 = b.trans1;
		}
	
		void ApplySlerp(double t,VirtualTrackball& trackball,double &tx,double &ty,double &tz)
		{
			trackball.Slerp(q0,q1,t);
			tx = trans0.X()+t*(trans1.X()-trans0.X());
			ty = trans0.Y()+t*(trans1.Y()-trans0.Y());
			tz = trans0.Z()+t*(trans1.Z()-trans0.Z());				
		}
			
	};
	
	//mostly book keeping variables for slerp recording.
	vector<SlerpInfo> slerpInfoVector;
	bool isSlerpRecordingOn;
	bool isNewSlerpInfoOn;
	double slerpQuaternion0[4],slerpQuaternion1[4];
	Vector3D slerpTranslation0,slerpTranslation1;
	
	///Resets the slerpInfos array, and sets the flag for slerp based recording.
	void BeginSlerpRecording();
	
	///Turns off slerp info recording. Once its off its possible to save frames 
	///slerps which are already recorded by calling SaveSlerpFrames	
	void EndSlerpRecording();
		
	///Inserts a new record into slerp array, using the current state as the start 	
	void BeginNewSlerpInfo();
	
	///Finishes current slerp info using current state as final parameter.
	void EndNewSlerpInfo();
	
	///Saves frames using the full array.
	///framesPerSlerp*slerpInfoVector.size() frames are stored.
	virtual void SaveSlerpFrames(const char* fileNameHeader, int framesPerSlerp = 10);
	
	///The background color.
	QColor backgroundColor;
};	
			

#endif //_QGL_GENERAL_CANVAS_H_


