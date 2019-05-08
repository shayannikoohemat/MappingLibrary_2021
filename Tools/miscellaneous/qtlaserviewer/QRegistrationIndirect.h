
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
*   File made : Feb 2006
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Object based indirect registration of point clouds
*
*--------------------------------------------------------------------*/
#ifndef __Q_REGISTRATION_INDIRECT_H__
#define __Q_REGISTRATION_INDIRECT_H__

#include "QGLPointsCanvas.h"
#include <QDialog>
#include <QListView>
#include <iostream>
#include "LaserPoints.h"
#include "LaserScan.h"
#include "LaserTransform3D.h"
#include "ENLSIPRegistrationIndirect.h"

using std::cerr;

class QGLPointsCanvas;
class QListView;
class QTabWidget;
class QRegistrationListView;
class LaserScan;

class  QRegistrationIndirect : public QDialog
{
    Q_OBJECT

public:
    //Standard constructor as required by Qt::QWidget
	QRegistrationIndirect( QWidget* parent = 0, Qt::WindowFlags fl = 0 );
	
    ///Constructor that takes file names of laser and object files.
	QRegistrationIndirect(const char* laserRef,const char* objRef,
		const char* laserReg, const char* objReg);
	
	///Initialize by creating controls, menus and setting layouts.
	void Initialize();
	
	///Destructor.
	~QRegistrationIndirect();

	///Set data from the passed points and object vectors.	
	void SetData(const LaserPoints& refScan,const LaserPoints& regScan,
				const LaserObjectsVector& refObj=LaserObjectsVector(), 
				const LaserObjectsVector& regObj = LaserObjectsVector());
	
	///Update the view if the registration parameters have changed.
	void UpdateView();
	
	///For testing the class in its standard interface.
	static void Test();
	
	///Load project from a given obp file.
	void LoadProject(std::string fileName);

public slots:
	///Add a pair of objects to correspondence list. Also remove it from the corresponding
	///point canvas so that it cannot be manipulate anymore. 
 	virtual void onAdd();
	
	///Do one set of iterations for registration.
	virtual void onRegister();
	
	///Load a project from obp file. Also ask for the file name.
	virtual void onLoadProject();
		
	///Update the settings page to show the value of setting variables.
	virtual void UpdateSettings();
	
	///Read changed values from settings page and apply them to settings variables.
	virtual void ApplySettings();
	
	//Update actions.
	virtual void UpdateActions();
	
private:
	QPixmap MakeColoredPixmap(QColor color,int width=16,int height = 16);
	LaserScan *pRefScan, *pRegScan;
		
	///The controls for showing the 3d interaction.
	QGLPointsCanvas* referencePointsCanvas;
	QGLPointsCanvas* underRegPointsCanvas;
	QGLPointsCanvas* combinedPointsCanvas;
	
	//Tabs and pages.
	QTabWidget* tabWidget;
	QWidget* regPage;
	QWidget* combinedPage;
	QWidget* settingsPage;
	
	///Controls for settings page.
	QSpinBox* maxIterSpin;
	QPushButton* applyBtn;
		
	//Actions 
	QAction* addAction;
	QAction* deleteAction;
	QAction* registerAction;
	QAction* loadProjectAction;
	
	//Menubar.
	QMenuBar* menuBar;
	
	//Maximum number of iteration for adjustment.
	int m_maxIter;
	
	///Data about correspondences etc.
	RegistrationData regData;
};

#endif // __Q_REGISTRATION_INDIRECT_H__
