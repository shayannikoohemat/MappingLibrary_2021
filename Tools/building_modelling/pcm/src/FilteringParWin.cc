
/*
                  Copyright 2010 University of Twente
 
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
#include "FilteringParWin.h"
#include <QPainter>
#include <QValidator>

FilteringParametersWindow::FilteringParametersWindow(QWidget *parent) :
  QWidget(parent), FilteringParameters()
{
  QDoubleValidator *positive_double, *non_negative_double;
  int              y;

  positive_double = new QDoubleValidator(0.0001, 1.0e10, 2, this);
  non_negative_double = new QDoubleValidator(0.00, 1.0e10, 2, this);

  setFixedSize(240, 150);

// Allowed height difference parameters

  // Height difference for points at same location
  y = 27;
  height_dif_line[0] = new QLineEdit(QString("%1").arg(HeightDifAtSameLocation(), 0, 'f', 1),
                            this);
  height_dif_line[0]->setValidator(non_negative_double);
  height_dif_line[0]->setGeometry(170, y, 40, 18);
  connect(height_dif_line[0], SIGNAL(textChanged(const QString &)),
          this, SLOT(SetHeightDifAtSameLocation(const QString &)));

  // Maximum filter radius
  filter_radius_line = new QLineEdit(QString("%1").arg(FilterRadius(), 0, 'f', 1),
                            this);
  filter_radius_line->setValidator(positive_double);
  y += 20; filter_radius_line->setGeometry(170, y, 40, 18);
  connect(filter_radius_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetFilterRadius(const QString &)));

  // Height difference for points at maximum filter radius
  height_dif_line[1] = new QLineEdit(QString("%1").arg(HeightDifAtFilterRadius(), 0, 'f', 1),
                            this);
  height_dif_line[1]->setValidator(positive_double);
  y += 20; height_dif_line[1]->setGeometry(170, y, 40, 18);
  connect(height_dif_line[1], SIGNAL(textChanged(const QString &)),
          this, SLOT(SetHeightDifAtFilterRadius(const QString &)));

  // Parameters controlling the points to be returned
  remove_non_ground_points_selector = new QCheckBox(this);
  remove_non_ground_points_selector->setChecked(RemoveNonGroundPoints());
  y += 20; remove_non_ground_points_selector->setGeometry(190, y+2, 40, 18);
  connect(remove_non_ground_points_selector, SIGNAL(toggled(bool)),
          this, SLOT(SetRemoveNonGroundPoints(bool)));
}

void FilteringParametersWindow::Update()
{
  height_dif_line[0]->setText(QString("%1").arg(HeightDifAtSameLocation(), 0, 'f', 1));
  filter_radius_line->setText(QString("%1").arg(FilterRadius(), 0, 'f', 1));
  height_dif_line[1]->setText(QString("%1").arg(HeightDifAtFilterRadius(), 0, 'f', 1));
  remove_non_ground_points_selector->setChecked(RemoveNonGroundPoints());
}

void FilteringParametersWindow::paintEvent(QPaintEvent *event)
{
  QPainter paint(this);
  int      y;

  if (event) y = 20; // To avoid compiler warnings
  // Allowed height difference parameters
  y = 20; paint.drawText(12, y, QString("Allowed height difference"));
  y += 20; paint.drawText(32, y, QString("Difference at same location:"));
  y += 20; paint.drawText(32, y, QString("Filter radius:"));
  y += 20; paint.drawText(32, y, QString("Difference at filter radius:"));
  y += 20; paint.drawText(32, y, QString("Remove non ground points"));
}

void FilteringParametersWindow::SetHeightDifAtSameLocation(const QString &new_height_dif)
{ if (strlen(new_height_dif.toLatin1().data()) == 0) return;
  HeightDifAtSameLocation() = new_height_dif.toDouble(); }

void FilteringParametersWindow::SetHeightDifAtFilterRadius(const QString &new_height_dif)
{ if (strlen(new_height_dif.toLatin1().data()) == 0) return;
  HeightDifAtFilterRadius() = new_height_dif.toDouble(); }

void FilteringParametersWindow::SetFilterRadius(const QString &new_radius)
{ if (strlen(new_radius.toLatin1().data()) == 0) return;
  FilterRadius() = new_radius.toDouble(); }

void FilteringParametersWindow::SetRemoveNonGroundPoints(bool new_switch_value)
{ remove_non_ground_points = new_switch_value; }
