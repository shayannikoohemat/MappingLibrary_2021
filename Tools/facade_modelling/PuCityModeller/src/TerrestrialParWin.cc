
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


#include <math.h>
#include "TerrestrialParWin.h"
#include <QPainter>
#include <QValidator>
#include <QSpinBox>
#include <QLineEdit>
#include <QComboBox>
#include <QCheckBox>
#include <iostream>




TerrestrialParametersWindow::TerrestrialParametersWindow(QWidget *parent) :QWidget(parent)
{
   QLineEdit        *line_edit;
   QDoubleValidator *positive_double;
   QSpinBox         *spin_box;
   positive_double = new QDoubleValidator(0.01, 1.0e10, 2, this);
   int              y;
  setFixedSize(330, 500);
  //for findfeature
  street_min_area=5;
  street_center_height=0;
  wall_min_area=4;
  wall_map_max_d=0.7;
  roof_min_area=0.5;
  roof_extru_min_area=0.5;
  door_wall_max_distance=0.5;
  door_min_area=1;
  door_max_area=10;
  door_max_height=2.5;
  extru_wall_max_distance=0.4;
  window_width=0.4;
  
  //for derive contour
  step_time=1;
  smoothness=2;
  
  //for fit wall outline
  dist_accurate=0.1;
  
  //for visualization
  back_offset=8.0;  

// FindFeature parameters

  // Min street segment area
  line_edit = new QLineEdit(QString("%1").arg(street_min_area, 0, 'f', 1), this);
  line_edit->setValidator(positive_double);
  y = 35; line_edit->setGeometry(200, y, 40, 18);
  connect(line_edit, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetStreetMinArea(const QString &)));
          
  // Min street general center height
  line_edit = new QLineEdit(QString("%1").arg(street_center_height, 0, 'f', 1), this);
  line_edit->setValidator(positive_double);
  y = y+25; line_edit->setGeometry(200, y, 40, 18);
  connect(line_edit, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetStreetCenterHeight(const QString &)));        
          
  // Min wall segment area
  line_edit = new QLineEdit(QString("%1").arg(wall_min_area, 0, 'f', 1), this);
  line_edit->setValidator(positive_double);
  y  = y+25; line_edit->setGeometry(200, y, 40, 18);
  connect(line_edit, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetWallMinArea(const QString &)));
  
  // max wall to mapline distance
  line_edit = new QLineEdit(QString("%1").arg(wall_map_max_d, 0, 'f', 1), this);
  line_edit->setValidator(positive_double);
  y  = y+ 25; line_edit->setGeometry(200, y, 40, 18);
  connect(line_edit, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetWalltoMapMaxDistance(const QString &)));
          
  // Min roof segment area
  line_edit = new QLineEdit(QString("%1").arg(roof_min_area, 0, 'f', 1), this);
  line_edit->setValidator(positive_double);
  y  = y+ 25; line_edit->setGeometry(200, y, 40, 18);
  connect(line_edit, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetRoofMinArea(const QString &)));
          
  // Min roof extrusion segment area
  line_edit = new QLineEdit(QString("%1").arg(roof_extru_min_area, 0, 'f', 1), this);
  line_edit->setValidator(positive_double);
  y  = y+ 25; line_edit->setGeometry(200, y, 40, 18);
  connect(line_edit, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetRoofExtruMinArea(const QString &)));
          
  // Door wall max distance
  line_edit = new QLineEdit(QString("%1").arg(door_wall_max_distance, 0, 'f', 1), this);
  line_edit->setValidator(positive_double);
  y  = y+ 25; line_edit->setGeometry(200, y, 40, 18);
  connect(line_edit, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetDoorWallMinDistance(const QString &)));
  
  // Door segment min area
  line_edit = new QLineEdit(QString("%1").arg(door_min_area, 0, 'f', 1), this);
  line_edit->setValidator(positive_double);
  y  = y+ 25; line_edit->setGeometry(200, y, 40, 18);
  connect(line_edit, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetDoorMinArea(const QString &)));
  
  //Door segment max area
  line_edit = new QLineEdit(QString("%1").arg(door_max_area, 0, 'f', 1), this);
  line_edit->setValidator(positive_double);
  y  = y+ 25; line_edit->setGeometry(200, y, 40, 18);
  connect(line_edit, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetDoorMaxArea(const QString &)));
          
  //Door max height
  line_edit = new QLineEdit(QString("%1").arg(door_max_height, 0, 'f', 1), this);
  line_edit->setValidator(positive_double);
  y  = y+ 25; line_edit->setGeometry(200, y, 40, 18);
  connect(line_edit, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetDoorMaxHeight(const QString &)));     
          
  //Extrusion wall max distance
  line_edit = new QLineEdit(QString("%1").arg(extru_wall_max_distance, 0, 'f', 1), this);
  line_edit->setValidator(positive_double);
  y  = y+ 25; line_edit->setGeometry(200, y, 40, 18);
  connect(line_edit, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetExtruWallMaxDistance(const QString &)));
  
  //Window width
  line_edit = new QLineEdit(QString("%1").arg(window_width, 0, 'f', 1), this);
  line_edit->setValidator(positive_double);
  y  = y+ 25; line_edit->setGeometry(200, y, 40, 18);
  connect(line_edit, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetWindowWidth(const QString &)));        
   
  /* 
  // Detail level
  spin_box = new QSpinBox(this);
  spin_box->setMinimum(1);
  spin_box->setMaximum(50);
  spin_box->setSingleStep(1);
  spin_box->setValue(step_time);
  y += 50; spin_box->setGeometry(200, y, 50, 18);
  connect(spin_box, SIGNAL(valueChanged(int)),
          this, SLOT(SetContourLevel(int)));   
    */      
  //Smoothness level
  spin_box = new QSpinBox(this);
  spin_box->setMinimum(1);
  spin_box->setMaximum(50);
  spin_box->setSingleStep(1);
  spin_box->setValue(smoothness);
  y +=50; spin_box->setGeometry(200, y, 50, 18);
  connect(spin_box, SIGNAL(valueChanged(int)),
          this, SLOT(SetSmoothness(int)));           
   
  //max distance point to line
  line_edit = new QLineEdit(QString("%1").arg(dist_accurate, 0, 'f', 1), this);
  line_edit->setValidator(positive_double);
  y  = y+ 25; line_edit->setGeometry(200, y, 40, 18);
  connect(line_edit, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaxp2linedist(const QString &)));
          
  //back offset
  line_edit = new QLineEdit(QString("%1").arg(back_offset, 0, 'f', 1), this);
  line_edit->setValidator(positive_double);
  y  = y+ 50; line_edit->setGeometry(200, y, 40, 18);
  connect(line_edit, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetBackOffset(const QString &)));
          
       
          
}

void TerrestrialParametersWindow::paintEvent(QPaintEvent *)
{
  QPainter paint(this);
  int      y;

  // Neighbourhood definitions
  y = 23; paint.drawText(12, y, QString("Find feature setting"));
  y += 25; paint.drawText(32, y, QString("Street segment min area:"));
  y += 25; paint.drawText(32, y, QString("Street level height:"));
  y += 25; paint.drawText(32, y, QString("Wall segment min area:"));
  y += 25; paint.drawText(32, y, QString("Wall2map max distance:"));
  y += 25; paint.drawText(32, y, QString("Roof segment min area:"));
  y += 25; paint.drawText(32, y, QString("Roof extrusion min area:"));
  y += 25; paint.drawText(32, y, QString("Door wall max distance:"));
  y += 25; paint.drawText(32, y, QString("Door segment min area:"));
  y += 25; paint.drawText(32, y, QString("Door segment max area:"));
  y += 25; paint.drawText(32, y, QString("Door max height:"));
  y += 25; paint.drawText(32, y, QString("Extrusion wall max distance:"));
  y += 25; paint.drawText(32, y, QString("Window width length:"));
  y += 25; paint.drawText(12, y, QString("Wall outline settings"));
 // y += 25; paint.drawText(32, y, QString("Detail level: "));
  y += 25; paint.drawText(32, y, QString("Smoothness level: "));
  y += 25; paint.drawText(32, y, QString("Max distance point 2 line:"));
  y += 25; paint.drawText(12, y, QString("Final model settings"));
  y += 25; paint.drawText(32, y, QString("Estimating side wall offset:"));
  
}

void TerrestrialParametersWindow::SetStreetMinArea(const QString &area)
{ street_min_area=area.toDouble(); }

void TerrestrialParametersWindow::SetStreetCenterHeight(const QString &area)
{ street_center_height=area.toDouble(); }

void TerrestrialParametersWindow::SetWallMinArea(const QString &area)
{ wall_min_area=area.toDouble(); }

void TerrestrialParametersWindow::SetWalltoMapMaxDistance(const QString &area)
{ wall_map_max_d=area.toDouble();  }

void TerrestrialParametersWindow::SetRoofMinArea(const QString &area)
{ roof_min_area=area.toDouble();  }

void TerrestrialParametersWindow::SetRoofExtruMinArea(const QString &area)
{ roof_extru_min_area=area.toDouble();  }

void TerrestrialParametersWindow::SetDoorWallMaxDistance(const QString &area)
{ door_wall_max_distance=area.toDouble();  }

void TerrestrialParametersWindow::SetDoorMinArea(const QString &area)
{ door_min_area=area.toDouble();  }

void TerrestrialParametersWindow::SetDoorMaxArea(const QString &area)
{ door_max_area=area.toDouble(); }

void TerrestrialParametersWindow::SetDoorMaxHeight(const QString &area)
{ door_max_height=area.toDouble(); }

void TerrestrialParametersWindow::SetExtruWallMaxDistance(const QString &area)
{ extru_wall_max_distance=area.toDouble(); }

 void TerrestrialParametersWindow::SetWindowWidth(const QString &dist)
 { window_width=dist.toDouble(); }
 

void TerrestrialParametersWindow::SetMaxp2linedist(const QString &dist)
{ dist_accurate=dist.toDouble();
 }
 
 void TerrestrialParametersWindow::SetContourLevel(int level)
{ step_time=level;
 }
 void TerrestrialParametersWindow::SetSmoothness(int level)
{ smoothness=level;
 }
 
 
 void TerrestrialParametersWindow::SetBackOffset(const QString &offset)
{ back_offset=offset.toDouble();
 }
