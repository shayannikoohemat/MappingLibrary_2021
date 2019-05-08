
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
#include "QGaussianSphereViewer.h"
#include "Histo3D.h"
#include "KNNFinder.h"

///Creates a new canvas 
QGaussianSphereViewer::QGaussianSphereViewer(QWidget* parent,const LaserPoints& laserPoints, 
		const LaserPoints& normalPoints, const char* name)
{
	pointsCanvas = new QGLPointsCanvas  ;
	normalsCanvas = new QGLPointsCanvas ;	
	
	QWidget *window  = this;
    QPushButton *button1 = new QPushButton("-->");
    button1->setMaximumWidth(48);
    connect(button1,SIGNAL(clicked()),this,SLOT(MapPoints2Normals()));
    QPushButton *button2 = new QPushButton("<--");
    button2->setMaximumWidth(48);
    connect(button2,SIGNAL(clicked()),this,SLOT(MapNormals2Points()));
    
    QVBoxLayout* buttonLayout = new QVBoxLayout;
    buttonLayout->addStretch();
    buttonLayout->addWidget(button1);
    buttonLayout->addWidget(button2);
    buttonLayout->addStretch();
  
    QHBoxLayout *layout = new QHBoxLayout;
    
	QVBoxLayout* vL = new QVBoxLayout;
    QLabel* label = new QLabel("Point cloud");
    label->setAlignment(Qt::AlignHCenter);
    label->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    vL->addWidget(label);
    pointsCanvas->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
    vL->addWidget(pointsCanvas);
    layout->addLayout(vL);
    
    layout->addLayout(buttonLayout);
    
    vL = new QVBoxLayout;
    label = new QLabel("Gaussian sphere");
    label->setAlignment(Qt::AlignHCenter);
    label->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    vL->addWidget(label);
    normalsCanvas->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
    vL->addWidget(normalsCanvas);
    layout->addLayout(vL);
    
    //We have to calculate the density of the normals.
	Histo3D<int> hist(normalPoints,0.025);
	
	LaserPoints histPointsIn = hist.ToPoints();
	LaserPoints histPoints = histPointsIn;
	KNNFinder<LaserPoint> finder(histPointsIn);
	//Filter the reflectance in hist points.
	int kNN = 10;
	
	if(histPointsIn.size()>kNN)
	{
		for(int i=0;i<histPointsIn.size();i++)
		{
			vector<int> indices = finder.FindIndices(histPointsIn[i],kNN);
		
			for(int j=0;j<indices.size();j++)
				histPoints[i].Reflectance() = histPoints[i].Reflectance() + histPointsIn[indices[j]].Reflectance();
		}				
	}
	
/*	
	QGLPointsCanvas *test = new QGLPointsCanvas(histPoints);
	test->show();
*/	
	
	
	LaserPoints densityPoints;
	densityPoints.reserve(normalPoints.size());
	
	for(int i=0;i<normalPoints.size();i++)
	{
		densityPoints.push_back(histPoints[finder.FindIndex(normalPoints[i])]);
	}
	
	densityPoints = normalPoints;
	for(int i=0;i<normalPoints.size();i++)
	{
		densityPoints[i].Reflectance() = histPoints[finder.FindIndex(normalPoints[i])].Reflectance();
	}
	
	
	
    pointsCanvas->SetLaserPoints(laserPoints);
    normalsCanvas->SetLaserPoints(densityPoints);
        
    window->setLayout(layout);
    window->setWindowState(Qt::WindowMaximized);
}		

///Destructor.
QGaussianSphereViewer::~QGaussianSphereViewer()
{
	
}


///Map selected points to normals.
void QGaussianSphereViewer::MapPoints2Normals()
{
	normalsCanvas->SetSelectedIndices(pointsCanvas->GetSelectedIndices());
}

///Map normals to points.
void QGaussianSphereViewer::MapNormals2Points()
{
	pointsCanvas->SetSelectedIndices(normalsCanvas->GetSelectedIndices());	
}
	

