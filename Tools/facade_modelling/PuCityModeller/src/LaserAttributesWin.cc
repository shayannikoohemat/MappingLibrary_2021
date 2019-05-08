
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
#include "LaserAttributesWin.h"
#include <QPainter>
#include <QSpinBox>

LaserAttributesWindow::LaserAttributesWindow(QWidget *parent) :
  QWidget(parent), LaserPoint()
{
  QSpinBox         *spin_box;
  int              y;

  setFixedSize(180, 120);
  
  // Label attribute
  SetAttribute(LabelTag, 0);
  spin_box = new QSpinBox(this);
  spin_box->setMinimum(0);
  spin_box->setMaximum(99999);
  spin_box->setSingleStep(1);
  spin_box->setValue(0);
  y = 15; spin_box->setGeometry(120, y, 50, 18);
  connect(spin_box, SIGNAL(valueChanged(int)),
          this, SLOT(SetLabelAttribute(int)));

  // IsFiltered attribute
  SetAttribute(IsFilteredTag, 0);
  spin_box = new QSpinBox(this);
  spin_box->setMinimum(0);
  spin_box->setMaximum(1);
  spin_box->setSingleStep(1);
  spin_box->setValue(0);
  y += 20; spin_box->setGeometry(120, y, 50, 18);
  connect(spin_box, SIGNAL(valueChanged(int)),
          this, SLOT(SetIsFilteredAttribute(int)));

  // PlaneNumber attribute
  SetAttribute(PlaneNumberTag, 0);
  spin_box = new QSpinBox(this);
  spin_box->setMinimum(0);
  spin_box->setMaximum(99999);
  spin_box->setSingleStep(1);
  spin_box->setValue(0);
  y += 20; spin_box->setGeometry(120, y, 50, 18);
  connect(spin_box, SIGNAL(valueChanged(int)),
          this, SLOT(SetPlaneNumberAttribute(int)));

  // SegmentNumber attribute
  SetAttribute(SegmentNumberTag, 0);
  spin_box = new QSpinBox(this);
  spin_box->setMinimum(0);
  spin_box->setMaximum(99999);
  spin_box->setSingleStep(1);
  spin_box->setValue(0);
  y += 20; spin_box->setGeometry(120, y, 50, 18);
  connect(spin_box, SIGNAL(valueChanged(int)),
          this, SLOT(SetSegmentNumberAttribute(int)));
}

void LaserAttributesWindow::paintEvent(QPaintEvent *)
{
  QPainter paint(this);
  int      y;

  y = 25; paint.drawText(12, y, QString("Label:"));
  y += 20; paint.drawText(12, y, QString("Is filtered:"));
  y += 20; paint.drawText(12, y, QString("Plane number:"));
  y += 20; paint.drawText(12, y, QString("Segment number:"));
}

void LaserAttributesWindow::SetLabelAttribute(int new_label)
{ SetAttribute(LabelTag, new_label); }

void LaserAttributesWindow::SetIsFilteredAttribute(int new_filter_state)
{ SetAttribute(IsFilteredTag, new_filter_state); }

void LaserAttributesWindow::SetPlaneNumberAttribute(int new_number)
{ SetAttribute(PlaneNumberTag, new_number); }

void LaserAttributesWindow::SetSegmentNumberAttribute(int new_number)
{ SetAttribute(SegmentNumberTag, new_number); }
