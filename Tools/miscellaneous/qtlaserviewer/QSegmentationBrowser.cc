
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
#include "QSegmentationBrowser.h"

///Creates a new canvas 
QSegmentationBrowser::QSegmentationBrowser(QWidget* parent,LaserSegments* segs, QString name)
{
	pointsCanvas = new QGLPointsCanvas  ;
	pSegments = (segs);
	
	pointsCanvas->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
	pointsCanvas->setMinimumWidth(256);
	pointsCanvas->setMinimumHeight(256);
	
	info = new QLabel;
	info->setAlignment(Qt::AlignHCenter);
    info->setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Fixed);
    
	spinBox = new QSpinBox;
	spinBox->setSizePolicy(QSizePolicy::Fixed,QSizePolicy::Fixed);
	connect(spinBox,SIGNAL(valueChanged(int)),this,SLOT(ShowSegment(int)));
	spinBox->setMinimum(0);
	spinBox->setMaximum(segs->size()-1);
	
	
    QVBoxLayout* layout = new QVBoxLayout;
    layout->addWidget(info);
    layout->addWidget(pointsCanvas);
    
    QHBoxLayout* hL = new QHBoxLayout;
    hL->addStretch();
    hL->addWidget(spinBox);
    hL->addStretch();
    layout->addLayout(hL);
    
    QAction* nextAction = new QAction("Next",this);
    connect(nextAction,SIGNAL(triggered()),this,SLOT(ShowNext()));
    nextAction->setShortcut(Qt::Key_PageUp);
    
    
    QAction* prevAction = new QAction("Previous",this);
    connect(prevAction,SIGNAL(triggered()),this,SLOT(ShowPrevious()));
    prevAction->setShortcut(Qt::Key_PageDown);
    
    QAction* lastAction = new QAction("Last",this);
    connect(lastAction,SIGNAL(triggered()),this,SLOT(ShowLast()));
    lastAction->setShortcut(Qt::Key_End);
    
    
    QAction* firstAction = new QAction("First",this);
    connect(firstAction,SIGNAL(triggered()),this,SLOT(ShowFirst()));
    firstAction->setShortcut(Qt::Key_Home);
    
    this->addAction(nextAction);
    this->addAction(prevAction);
    this->addAction(firstAction);
    this->addAction(lastAction);
	   
    this->setLayout(layout);
    this->setWindowTitle(name);
    this->setContextMenuPolicy(Qt::ActionsContextMenu);
    
    this->ShowFirst();
}		

///Destructor.
QSegmentationBrowser::~QSegmentationBrowser()
{
	
}

///Show the selected segment.
void QSegmentationBrowser::ShowSegment(int sel)
{
	if(sel!=selectedSegment && sel>=0 && sel<pSegments->size())
	{
		selectedSegment = sel;
		LaserPoints pts = (*pSegments)(sel);
		pointsCanvas->SetLaserPoints( pts);
		info->setText(QString("Segment %1 with %2 points").arg(selectedSegment).arg(pts.size()));
		spinBox->setValue(selectedSegment);
	}
}

///Show next
void QSegmentationBrowser::ShowNext()
{
	ShowSegment(selectedSegment+1);
}

///Show previous.
void QSegmentationBrowser::ShowPrevious()
{
	ShowSegment(selectedSegment-1);
}

///Show first.
void QSegmentationBrowser::ShowFirst()
{
	ShowSegment(0);
}


///Show last one.
void QSegmentationBrowser::ShowLast()
{
	ShowSegment(pSegments->size()-1);	
}


	

