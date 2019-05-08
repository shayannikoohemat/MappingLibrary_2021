
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
#include <QTabWidget>
#include <QDoubleValidator>
#include <QSizePolicy>
#include "QRegistrationIndirect.h"
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
#include "QtUtility.h"

/*****************************************************************************
 *
 * Class QRegistrationIndirect
 *
 *****************************************************************************/

///Make the interface and do the stuff common to all constructors.
void QRegistrationIndirect::Initialize()
{
	m_maxIter = 5;
	
	///Make actions 
	QAction* action = addAction = new QAction("Add object-pair",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(onAdd()));
	
	action = deleteAction = new QAction("Delete object-pair",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(onDelete()));
	
	action = registerAction = new QAction("Register indirect",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(onRegister()));
	
	action = loadProjectAction = new QAction("Load project...",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(onLoadProject()));
	
	///The controls for showing the 3d interaction.
	referencePointsCanvas = new QGLPointsCanvas(this);
	underRegPointsCanvas = new QGLPointsCanvas(this);
	combinedPointsCanvas = new QGLPointsCanvas(this);
	
	//Hide the menu bars.
	referencePointsCanvas->SetMenuVisible(false);
	underRegPointsCanvas->SetMenuVisible(false);
	combinedPointsCanvas->SetMenuVisible(false);
	
	//Create the tabs.
	QTabWidget* tabWidget = new QTabWidget(this);
	
	//Create the pages.
	QWidget* regPage = new QWidget;
	QWidget* combinedPage = new QWidget;
	QWidget* settingsPage = new QWidget;
	
	///Make and set the layout.
	
	//This is the main layout.
	QVBoxLayout* vl = NULL;
	QHBoxLayout* hl = NULL;
	QVBoxLayout *layout = new QVBoxLayout;
	this->setLayout(layout);
	layout->addWidget(tabWidget);
	
	//Registration page.
	const int spaceSize = 10;
	QHBoxLayout* canvasLayout = new QHBoxLayout;
	
	QVBoxLayout *refLayout = new QVBoxLayout;
	refLayout->addWidget(referencePointsCanvas);
	QPushButton* btn = new QPushButton("Add object pair",this);
	QObject::connect(btn,SIGNAL(clicked()),this,SLOT(onAdd()));
	btn->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
	refLayout->addLayout(HorizontalCenteredLayout(btn));
	
	canvasLayout->addLayout(refLayout);
    
	canvasLayout->addSpacing(spaceSize);
	
	QVBoxLayout *regLayout = new QVBoxLayout;
	regLayout->addWidget(underRegPointsCanvas);
	QPushButton* regbtn = new QPushButton("Register indirect",this);
	QObject::connect(regbtn,SIGNAL(clicked()),this,SLOT(onRegister()));
	regbtn->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
	regLayout->addLayout(HorizontalCenteredLayout(regbtn));
	
	canvasLayout->addLayout(regLayout);
		
	vl = new QVBoxLayout;
	
	vl->addLayout(canvasLayout);
	regPage->setLayout(vl);
	tabWidget->addTab(regPage,"Correspondences");
	
	//combined page.
	vl = new QVBoxLayout;
	vl->addWidget(combinedPointsCanvas);
	combinedPage->setLayout(vl);
	tabWidget->addTab(combinedPage,"Combined");
    
	//Settings page.
		
	//Main controls
	maxIterSpin = new QSpinBox(this);
	maxIterSpin->setRange(1,25);
	
	applyBtn = new QPushButton("Apply settings",this);
	QObject::connect(applyBtn,SIGNAL(clicked()),this,SLOT(ApplySettings()));
	
	
	//Their layouts
	QGridLayout* gl = new QGridLayout;
	int count = 0;
	gl->addWidget(new QLabel("Percentage threshold",this),count,0);
		
	vl = new QVBoxLayout;
	hl = new QHBoxLayout;
	hl->addLayout(gl);
	hl->addStretch();
	vl->addLayout(hl);
	
	hl = new QHBoxLayout;
	//hl->addStretch();
	hl->addWidget(applyBtn);
	hl->addStretch();
	vl->addLayout(hl);
	
	vl->addStretch();
		
	settingsPage->setLayout(vl);
	tabWidget->addTab(settingsPage,"Settings");
	UpdateSettings();
	
	//Menu bar.
	menuBar = new QMenuBar(this);	
	this->layout()->setMenuBar(menuBar);
	
	QMenu* menu = new QMenu("File",this);
	menu->addAction(loadProjectAction);
	menuBar->addMenu(menu);
	
	
	menu = new QMenu("Targets",this);
	menu->addAction(addAction);
	menuBar->addMenu(menu);
	
	menu = new QMenu("Register",this);
	menu->addAction(registerAction);
	menuBar->addMenu(menu);
		
	//We will assume the id of 0 for ref and 1 for reg scans.
	//Add the transform to the table.
	regData.transforms[1] = LaserTransform3D();
}

QRegistrationIndirect::QRegistrationIndirect( QWidget* parent, Qt::WindowFlags fl )
    : QDialog( parent, fl )
{
	Initialize();
}

/*
 *  Destroys the object and frees any allocated resources
 */
QRegistrationIndirect::~QRegistrationIndirect()
{
    // no need to delete child widgets, Qt does it all for us
}

QPixmap QRegistrationIndirect::MakeColoredPixmap(QColor color,int width,int height)
{
	QPixmap pix(width,height);
	pix.fill(color);
	return pix;
	
}


void QRegistrationIndirect::UpdateSettings()
{
	maxIterSpin->setValue(m_maxIter);
}

void QRegistrationIndirect::ApplySettings()
{
	m_maxIter = maxIterSpin->value();
	UpdateSettings();

}

//Update actions.
void QRegistrationIndirect::UpdateActions()
{
	
}


QRegistrationIndirect::QRegistrationIndirect(const char* laserRef,const char* objRef,
		const char* laserReg, const char* objReg)
{
	Initialize();
	referencePointsCanvas->LoadPoints(laserRef);
	underRegPointsCanvas->LoadPoints(laserReg);
	
	referencePointsCanvas->LoadObjects(objRef);
	underRegPointsCanvas->LoadObjects(objReg);
	
	UpdateView();
}		

//On adding a correspondence.
void QRegistrationIndirect::onAdd()
{
	static int id = 1;
	LaserObject* ref = referencePointsCanvas->GetSelectedObject();
	LaserObject* reg= underRegPointsCanvas->GetSelectedObject();
	
	if(ref && reg)
	{
		ref = ref->Clone();
		reg = reg->Clone();
		
		//Update the canvas.
		referencePointsCanvas->DeleteSelectedObject();
		underRegPointsCanvas->DeleteSelectedObject();
		
		underRegPointsCanvas->Repaint();
		referencePointsCanvas->Repaint();
		
		//Update the registration structure.
		int nRefId = id++;
		int nRegId = id++;
		
		RegTargetId regId(1,nRegId);
		RegTargetId refId(0,nRefId);
			
		RegistrationTarget refTarget(refId,ref->Name().c_str(),ref->ParamVector());
		RegistrationTarget regTarget(regId,reg->Name().c_str(),reg->ParamVector());
			
		//Add the target to our map.
		regData.targets[regId] = regTarget;
		regData.targets[refId] = refTarget;
		
		
		//Add the correspondence.
		regData.correspondences[refId].push_back(regId);
		
		//we are done so must delete the objects.
		delete ref;
		delete reg;
		
		regData.Print();
		cerr<<"\n*****************\n\n";
	}
}

void QRegistrationIndirect::onRegister()
{
	cerr<<"Register pressed\n";
	vector<double> initial(7);
	
	regData.transforms[1].SaveToVector(initial.begin());
	Print(initial,cerr);
	
	regData.transforms[1].Print();
	
	cerr<<"Starting...\n";
	ENLSIPRegistrationIndirect optimizer(&regData);
	optimizer.SetObsCount(100);
		
	vector<double> final = optimizer.Optimize(initial);
	
	cerr<<"Done with optimization \n";
	cerr<<"initial.size(): "<<initial.size()<<" final.size(): "<<final.size()<<endl;
	cerr<<setw(16)<<"Initial"<<setw(16)<<"final\n";
	for(int k=0;k<initial.size();k++)
		cerr<<setw(16)<<initial[k]<<setw(16)<<final[k]<<endl;
	
	regData.PrintCorrespondences();
	
	UpdateView();
	
}

void QRegistrationIndirect::onLoadProject()
{
	QString fn = QFileDialog::getOpenFileName(this,
					"Open new Object Based Registration Project (obp)",
                    ".",
                    "object registration project (*.obp)");
                            
  
	if ( !fn.isEmpty() ) 
	{
        this->LoadProject(fn.toStdString());
	}
}

void QRegistrationIndirect::SetData(const LaserPoints& refScan,const LaserPoints& regScan,
				const LaserObjectsVector& refObj, const LaserObjectsVector& regObj)
{
	referencePointsCanvas->SetLaserPoints(refScan);	
	referencePointsCanvas->SetLaserObjects(refObj);	
	underRegPointsCanvas->SetLaserPoints(regScan);	
	underRegPointsCanvas->SetLaserObjects(regObj);
	
	regData.transforms[1] = LaserTransform3D();	
	UpdateView();
}				

void QRegistrationIndirect::Test()
{
	LaserPoints pts1= MakeSinePoints();
	
	AngleAxisRotation aa(Vector3D(1,0,0),0.3);
	Rotation3D rot = aa.to_matrix();
	Vector3D trans = Vector3D(1,1,1)*0.2;
	LaserPoints pts2 = MakeSinePoints()*Orientation3D(trans,rot);
		
	QRegistrationIndirect* reg = new QRegistrationIndirect();
	reg->SetData(pts1,pts2);
	reg->show();
}

void QRegistrationIndirect::UpdateView()
{

	int maxPoints = 10000;
	int sub = max((int)referencePointsCanvas->laserPoints.size()/maxPoints,1);
	LaserPoints refCopy = referencePointsCanvas->laserPoints.SubSampleSimple(sub);
	LaserPoints regCopy = underRegPointsCanvas->laserPoints.SubSampleSimple(sub);

	combinedPointsCanvas->SetLaserPoints(refCopy.SetReflectance(0) + 
			regCopy.Transform(regData.transforms[1].rot.to_matrix(),
						regData.transforms[1].trans).SetReflectance(10));
}


///Load project from a given obp file.
void QRegistrationIndirect::LoadProject(std::string fileName)
{
 	ifstream file(fileName.c_str());
	std::string str;

	//Reference points.
	file>>str;
	referencePointsCanvas->LoadPoints(str.c_str());

	//Reference objects.
	file>>str;
	referencePointsCanvas->LoadObjects(str.c_str());

	//Registration points.
	file>>str;
	underRegPointsCanvas->LoadPoints(str.c_str());

	//Reference objects.
	file>>str;
	underRegPointsCanvas->LoadObjects(str.c_str());
	
	file.close();

	UpdateView();
}
