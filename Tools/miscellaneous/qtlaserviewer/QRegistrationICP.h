
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
*   File made : March 2004 
*   Author    : Tahir Rabbani
*	Modified  :
*   Purpose   : Implements an interface for doing registration of point clouds.
*
*--------------------------------------------------------------------*/
#ifndef __Q_REGISTRATION_ICP_H__
#define __Q_REGISTRATION_ICP_H__

#include "QGLPointsCanvas.h"
#include <QDialog>
#include <QListView>
#include <iostream>
#include "LaserPoints.h"
#include "LaserScan.h"

using std::cerr;

class QGLPointsCanvas;
class QListView;
class QTabWidget;
class QRegistrationListView;
class LaserScan;

class  QRegistrationICP : public QDialog
{
    Q_OBJECT

public:
    QRegistrationICP( QWidget* parent = 0, Qt::WindowFlags fl = 0 );
	
	void Initialize();
	~QRegistrationICP();
	
	void SetScans(LaserScan* _pRefScan,LaserScan* _pRegScan);
	void UpdateView();
	void ApplyIncrementalOrientation(Orientation3D* orient);
	
	///Load project from a given icp file.
	void LoadProject(std::string fileName);

public slots:
	///Load a project from obp file. Also ask for the file name.
	virtual void onLoadProject();
	
	///Approximate registration using targets.
	virtual void ApproximateRegister();
	
	///Registration using Iterative Closest Point(ICP) with the specified settings.
    virtual void ICPRegister();
	
	///Add a target in the reference scan, using the selected points. Also, deselect all points after that.
	virtual void AddRefTarget();
	
	///Add a target in the being-registered scan, using the selected points. Also, deselect all points after that.
	virtual void AddRegTarget();
	
	///Update the settings page to show the value of setting variables.
	virtual void UpdateSettings();
	
	///Read changed values from settings page and apply them to settings variables.
	virtual void ApplySettings();
	
	//Update actions.
	virtual void UpdateActions();
	
	//Function for testing.
	static void Test();
	
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
	QSpinBox* thresholdSpin;
	QCheckBox* useNormalsChkbox;
	QCheckBox* autoApproximateChkBox;
	QSpinBox* maxIterSpin;
	QSpinBox* knnSpin;
	QPushButton* applyBtn;
		
	//Actions 
	QAction* registerApproxAction;
	QAction* registerICPAction;
	QAction* addRefTargetAction;
	QAction* addRegTargetAction;
	QAction* loadProjectAction;
	
	//Menubar.
	QMenuBar* menuBar;
	
	int m_maxIter;
	double m_percentageThreshold;
	bool m_useNormals;
	int m_kNN;
	bool m_autoApproximateCorrespondence;
};

#endif // __Q_REGISTRATION_ICP_H__
