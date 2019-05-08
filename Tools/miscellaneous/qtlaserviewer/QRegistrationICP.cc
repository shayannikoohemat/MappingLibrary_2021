
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
#include "QRegistrationICP.h"
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
 * Class QRegistrationICP
 *
 *****************************************************************************/

///Make the interface and do the stuff common to all constructors.
void QRegistrationICP::Initialize()
{
	//settings of the processing
	m_maxIter = 5;
	m_percentageThreshold = 50;
	m_useNormals = true;
	m_kNN = 20;
	m_autoApproximateCorrespondence = false;
	
	///Make actions 
	QAction* action = registerApproxAction = new QAction("Register approximate",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(ApproximateRegister()));
	
	action = registerICPAction = new QAction("Register ICP",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(ICPRegister()));
	
	action = addRefTargetAction = new QAction("Add Target to Ref",this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(AddRefTarget()));
	
	action = addRegTargetAction = new QAction("Add Target to Reg", this);
	QObject::connect(action,SIGNAL(triggered()),this,SLOT(AddRegTarget()));
	
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
	QPushButton* btn = new QPushButton("Add control point",this);
	QObject::connect(btn,SIGNAL(clicked()),this,SLOT(AddRefTarget()));
	btn->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
	refLayout->addLayout(HorizontalCenteredLayout(btn));
	
	canvasLayout->addLayout(refLayout);
    
	canvasLayout->addSpacing(spaceSize);
	
	QVBoxLayout *regLayout = new QVBoxLayout;
	regLayout->addWidget(underRegPointsCanvas);
	QPushButton* regbtn = new QPushButton("Add control point",this);
	QObject::connect(regbtn,SIGNAL(clicked()),this,SLOT(AddRegTarget()));
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
	thresholdSpin = new QSpinBox(this);
	thresholdSpin->setRange(1,100);
	
	useNormalsChkbox = new QCheckBox("",this);
	autoApproximateChkBox = new QCheckBox("",this);		
	
	maxIterSpin = new QSpinBox(this);
	maxIterSpin->setRange(1,25);
	
	knnSpin = new QSpinBox(this);
	knnSpin->setRange(1,1000);
	
	applyBtn = new QPushButton("Apply settings",this);
	QObject::connect(applyBtn,SIGNAL(clicked()),this,SLOT(ApplySettings()));
	
	
	//Their layouts
	QGridLayout* gl = new QGridLayout;
	int count = 0;
	gl->addWidget(new QLabel("Percentage threshold",this),count,0);
	gl->addWidget(thresholdSpin,count,1);
	
	count++;
	gl->addWidget(new QLabel("Use normals",this),count,0);
	gl->addWidget(useNormalsChkbox,count,1);
	
	count++;
	gl->addWidget(new QLabel("Maximum iterations",this),count,0);
	gl->addWidget(maxIterSpin,count,1);
	
	count++;
	gl->addWidget(new QLabel("k nearest neighbours",this),count,0);
	gl->addWidget(knnSpin,count,1);
	gl->setSizeConstraint(QLayout::SetMinimumSize);
	
	count++;
	gl->addWidget(new QLabel("Automatic search in ApproxReg",this),count,0);
	gl->addWidget(autoApproximateChkBox,count,1);
	
	
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
	menu->addAction(addRefTargetAction);
	menu->addAction(addRegTargetAction);
	menuBar->addMenu(menu);
	
	menu = new QMenu("Register",this);
	menu->addAction(registerApproxAction);
	menu->addAction(registerICPAction);
	menuBar->addMenu(menu);
	
	
	
	///Make and set the layout.
}
/* 
 *  Constructs a QRegistrationICP as a child of 'parent', with the 
 *  name 'name' and widget flags set to 'f'.
 *
 */
QRegistrationICP::QRegistrationICP( QWidget* parent, Qt::WindowFlags fl )
    : QDialog( parent, fl )
{
	Initialize();
}

/*
 *  Destroys the object and frees any allocated resources
 */
QRegistrationICP::~QRegistrationICP()
{
    // no need to delete child widgets, Qt does it all for us
}

QPixmap QRegistrationICP::MakeColoredPixmap(QColor color,int width,int height)
{
	QPixmap pix(width,height);
	pix.fill(color);
	return pix;
	
}

void QRegistrationICP::ApplyIncrementalOrientation(Orientation3D* orient)
{

	Orientation3D old = *(pRegScan->GetOrientation());
	
	Rotation3D deltaRot = *orient;
	Rotation3D oldRot = old;
	
	Vector3D trans;
	Orientation3D newOrient(deltaRot*Vector3D(old)+Vector3D(*orient),deltaRot*oldRot);
	
	//cerr<<"Setting orientation\n"<<flush;
	pRegScan->SetOrientation(newOrient);
	//cerr<<"Done\n"<<flush;
	
	//cerr<<"Update view\n"<<flush;
	UpdateView();
	//cerr<<"Done\n"<<flush;
}

void QRegistrationICP::UpdateView()
{
	LaserPoints refPoints, regPoints;
	
	if(pRefScan)
	{
		refPoints = pRefScan->GetTransformedPoints();
	}
	referencePointsCanvas->SetLaserPoints(refPoints);
	
	if(pRegScan)
	{	
		regPoints = pRegScan->GetTransformedPoints();
	}
	underRegPointsCanvas->SetLaserPoints(regPoints);
	
	refPoints.SetReflectance(10);
	regPoints.SetReflectance(20);
	
	combinedPointsCanvas->SetLaserPoints(regPoints+refPoints);
	
}

void QRegistrationICP::SetScans(LaserScan* _pRefScan,LaserScan* _pRegScan)
{
	pRefScan = _pRefScan;
	pRegScan = _pRegScan;
	UpdateView();
}

void QRegistrationICP::AddRefTarget()
{
	if(referencePointsCanvas->PointSelectionCount())
	{
		referencePointsCanvas->AddControlPoint();	
	}
}

void QRegistrationICP::AddRegTarget()
{
	if(underRegPointsCanvas->PointSelectionCount())
	{
		underRegPointsCanvas->AddControlPoint();	
	}
}

void QRegistrationICP::ApproximateRegister()
{
	LaserPoints fixed,reg;
	Orientation3D orient;
	 
	//First try with selected points
	fixed = referencePointsCanvas->GetSelectedControlPoints();
	reg = underRegPointsCanvas->GetSelectedControlPoints();
	
	if(fixed.size()<3 || reg.size()<3)
	{
		//Selected point has failed now we will try automatic correpondence detection.
		fixed = referencePointsCanvas->GetAllControlPoints();
		reg = underRegPointsCanvas->GetAllControlPoints();
		
		if(fixed.size()<3 || reg.size()<3)
		{
			QMessageBox::warning(this,"Insufficient control points",
			QString("Insufficient control points a: %1 b: %2. Cannot Register").arg(fixed.size()).arg(reg.size()));
			return;
		}
	}
	//Now we have enough points for automatic detection
	//We will try all combination of 3 correspondences and keep the one with the best
	//registration result.
	LaserPoints regSel;
	LaserPoints fixedSel;
	
	if(m_autoApproximateCorrespondence)
	{		
		cerr<<"Automatic approximate\n";
		
		regSel.resize(3);
		fixedSel.resize(3);
		double minError = -1;

		
		LaserPoints refPoints = pRefScan->GetTransformedPoints().SubSampleSimple(10);
		LaserPoints regPoints = pRegScan->GetTransformedPoints().SubSampleSimple(10);

		for(int ii=0;ii<reg.size();ii++)
			for(int jj=ii+1;jj<reg.size();jj++)
				for(int kk=jj+1;kk<reg.size();kk++)
		{
			if(ii==jj || jj==kk || ii==kk)
						continue;	
			regSel[0] = reg[ii];
			regSel[1] = reg[jj];
			regSel[2] = reg[kk];		


			for(int i=0;i<fixed.size();i++)
				for(int j=0;j<(fixed.size());j++)
					for(int k=0;k<(fixed.size());k++)
			{
				if(i==j || j==k || i==k)
					continue;

				fixedSel[0] = fixed[i];
				fixedSel[1] = fixed[j];
				fixedSel[2] = fixed[k];


			#if 1
				cerr<<"iterating with "<<(i)<<"  "
					<<(j)<<"  "
					<<(k)<<"  "<<endl;
			#endif					

				Orientation3D current;
				FindTransformation(regSel,fixedSel,&current);

				LaserPoints regTrans = (regPoints)*(current);
				LaserPoints refTrans = refPoints;

				vector<double> distances = regTrans.Distance(refTrans);
				sort(distances.begin(),distances.end());

				double currentMin = distances[m_percentageThreshold*(distances.size()-1)];

				cerr<<"currentMin error: "<<currentMin<<endl;;

				if(minError<0 || currentMin<minError)
				{
					//cerr<<"Old min error: "<<minError;
					minError = currentMin;
					//cerr<<"    New min error: "<<minError<<endl;
					orient = current;
				}
			}
		}
		ApplyIncrementalOrientation(&orient);
	}
	else
	{
		cerr<<"Manual approximate\n";
		
		//Make them compatible.
		fixed.resize(min(fixed.size(),reg.size()));
		reg.resize(fixed.size());
				
		Orientation3D current;
		FindTransformation(reg,fixed,&current);
		orient = current;
		ApplyIncrementalOrientation(&orient);
	}
}


void QRegistrationICP::ICPRegister()
{
	if(!pRefScan || !pRegScan || pRefScan->size()<10 || pRegScan->size()<10)
	{
		QMessageBox::warning(this,"No points to register through ICP",
			"No data available for registering with ICP.");
		return;
	}
	LaserPoints fixed = pRefScan->GetTransformedPoints();
	LaserPoints reg = pRegScan->GetTransformedPoints();
	
	//Now find the transformation.
	Orientation3D orient;
	
	
	
#if 0
	#define SH(a) cerr<<#a<<": "<<a<<endl;	
	SH(m_maxIter);
	SH(m_percentageThreshold);
	SH(m_useNormals);
	SH(m_kNN);
	cerr<<"reg.size() "<<reg.size()<<endl;
	cerr<<"fixed.size() "<<fixed.size()<<endl;
	cerr<<"ICP...\n";
#endif 	
	RegisterUsingICP(reg, fixed,&orient,m_maxIter,m_percentageThreshold,m_useNormals,m_kNN);
	
#if 0	
	cerr<<"ICP...Done\n";
#endif	
	
	ApplyIncrementalOrientation(&orient);
 
}     

void QRegistrationICP::UpdateSettings()
{
	thresholdSpin->setValue(m_percentageThreshold);
	useNormalsChkbox->setCheckState(m_useNormals?(Qt::Checked):(Qt::Unchecked));
	maxIterSpin->setValue(m_maxIter);
	knnSpin->setValue(m_kNN);
	autoApproximateChkBox->setCheckState(m_autoApproximateCorrespondence?(Qt::Checked):(Qt::Unchecked));
}

void QRegistrationICP::ApplySettings()
{
	m_percentageThreshold = thresholdSpin->value();
	m_useNormals = useNormalsChkbox->checkState() == (Qt::Checked); 
	m_maxIter = maxIterSpin->value();
	m_kNN = knnSpin->value();
	m_autoApproximateCorrespondence = autoApproximateChkBox->checkState()==(Qt::Checked); 
	
	UpdateSettings();

}

//Update actions.
void QRegistrationICP::UpdateActions()
{
#if 0	
	registerApproxAction->setEnabled(referencePointsCanvas->
	registerICPAction
	addRefTargetAction
	addRegTargetAction
#endif	
	
}



void QRegistrationICP::Test()
{
	LaserScan* s1 = new LaserScan(MakeSinePoints(),Orientation3D(),1);
	
	AngleAxisRotation aa(Vector3D(1,0,0),0.3);
	Rotation3D rot = aa.to_matrix();
	Vector3D trans = Vector3D(1,1,1)*0.2;
	LaserScan* s2 = new LaserScan(MakeSinePoints(),Orientation3D(trans,rot),2);
		
	QRegistrationICP* reg = new QRegistrationICP();
	reg->SetScans(s1,s2);
	reg->show();
}

///Load project from a given icp file.
void QRegistrationICP::LoadProject(std::string fileName)
{
 	ifstream file(fileName.c_str());
	std::string str;

	LaserPoints pts1, pts2;
	//Reference points.
	file>>str;
	pts1.Read(str.c_str());

	//Registration points.
	file>>str;
	pts2.Read(str.c_str());
	
	file.close();
	
	LaserScan* s1 = new LaserScan(pts1,Orientation3D(),1);
	LaserScan* s2 = new LaserScan(pts2,Orientation3D(),2);
	this->SetScans(s1,s2);

	UpdateView();
}

void QRegistrationICP::onLoadProject()
{
	QString fn = QFileDialog::getOpenFileName(this,
					"Open new Object Based Registration Project (obp)",
                    ".",
                    "object registration project (*.icp)");
                            
  
	if ( !fn.isEmpty() ) 
	{
        this->LoadProject(fn.toStdString());
	}
}
