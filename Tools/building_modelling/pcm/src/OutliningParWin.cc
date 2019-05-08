
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
#include "OutliningParWin.h"
#include <QPainter>
#include <QValidator>

OutliningParametersWindow::OutliningParametersWindow(QWidget *parent) :
  QWidget(parent), OutliningParameters()
{
  QDoubleValidator *positive_double, *non_negative_double;
  int              y;
  double           degree = atan(1.0) / 45.0;

  positive_double = new QDoubleValidator(0.01, 1.0e10, 2, this);
  non_negative_double = new QDoubleValidator(0.00, 1.0e10, 2, this);

  setFixedSize(300, 180);

  // Maximum distance between a point and an intersection line to determine segment ends
  max_dist_point_intersection_line =
    new QLineEdit(QString("%1").arg(MaximumDistancePointIntersectionLine(), 0, 'f', 1),
                  this);
  max_dist_point_intersection_line->setValidator(positive_double);
  y = 10; max_dist_point_intersection_line->setGeometry(240, y, 40, 18);
  connect(max_dist_point_intersection_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaximumDistancePointIntersectionLine(const QString &)));

  // Minimum angle between preferred directions
  min_angle_directions_line =
    new QLineEdit(QString("%1").arg(MinimumAnglePreferredDirections()/degree, 0, 'f', 1),
                  this);
  min_angle_directions_line->setValidator(positive_double);
  y += 20; min_angle_directions_line->setGeometry(240, y, 40, 18);
  connect(min_angle_directions_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMinimumAnglePreferredDirections(const QString &)));
    
  // Hough bin size for distance
  hough_bin_size_dist_line =
    new QLineEdit(QString("%1").arg(HoughBinSizeDistance(), 0, 'f', 1),
                  this);
  hough_bin_size_dist_line->setValidator(positive_double);
  y += 20; hough_bin_size_dist_line->setGeometry(240, y, 40, 18);
  connect(hough_bin_size_dist_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetHoughBinSizeDistance(const QString &)));
    
  // Hough bin size for direction
  hough_bin_size_dir_line =
    new QLineEdit(QString("%1").arg(HoughBinSizeDirection()/degree, 0, 'f', 1),
                  this);
  hough_bin_size_dir_line->setValidator(positive_double);
  y += 20; hough_bin_size_dir_line->setGeometry(240, y, 40, 18);
  connect(hough_bin_size_dir_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetHoughBinSizeDirection(const QString &)));
    
  // Maximum distance between a point and an intersection line to determine segment ends
  max_dist_point_outline_line =
    new QLineEdit(QString("%1").arg(MaximumDistancePointOutline(), 0, 'f', 1),
                  this);
  max_dist_point_outline_line->setValidator(positive_double);
  y += 20; max_dist_point_outline_line->setGeometry(240, y, 40, 18);
  connect(max_dist_point_outline_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaximumDistancePointOutline(const QString &)));

  // Maximum gap size in outline segment in number of points
  max_num_pts_gap_selector = new QSpinBox(this);
  max_num_pts_gap_selector->setMinimum(0);
  max_num_pts_gap_selector->setMaximum(9999);
  max_num_pts_gap_selector->setSingleStep(1);
  max_num_pts_gap_selector->setValue(MaximumPointGapInOutlineSegment());
  y += 20; max_num_pts_gap_selector->setGeometry(240, y, 50, 18);
  connect(max_num_pts_gap_selector, SIGNAL(valueChanged(int)),
          this, SLOT(SetMaximumPointGapInOutlineSegment(int)));

  // Maximum gap size in outline segment in meters
  max_size_gap_line =
    new QLineEdit(QString("%1").arg(MaximumGapSizeInOutlineSegment(), 0, 'f', 1),
                  this);
  max_size_gap_line->setValidator(non_negative_double);
  y += 20; max_size_gap_line->setGeometry(240, y, 40, 18);
  connect(max_size_gap_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaximumGapSizeInOutlineSegment(const QString &)));
    
  // Minumum number of points in an outline segment
  min_num_pts_segment_selector = new QSpinBox(this);
  min_num_pts_segment_selector->setMinimum(2);
  min_num_pts_segment_selector->setMaximum(9999);
  min_num_pts_segment_selector->setSingleStep(1);
  min_num_pts_segment_selector->setValue(MinimumNumberOfPointsInOutlineSegment());
  y += 20; min_num_pts_segment_selector->setGeometry(240, y, 50, 18);
  connect(min_num_pts_segment_selector, SIGNAL(valueChanged(int)),
          this, SLOT(SetMinimumNumberOfPointsInOutlineSegment(int)));
}

void OutliningParametersWindow::Update()
{
  double           degree = atan(1.0) / 45.0;

  max_dist_point_intersection_line->setText(QString("%1").arg(
    MaximumDistancePointIntersectionLine(), 0, 'f', 1));
  min_angle_directions_line->setText(QString("%1").arg(
    MinimumAnglePreferredDirections()/degree, 0, 'f', 1));
  hough_bin_size_dist_line->setText(QString("%1").arg(
    HoughBinSizeDistance(), 0, 'f', 1));
  hough_bin_size_dir_line->setText(QString("%1").arg(
    HoughBinSizeDirection()/degree, 0, 'f', 1));
  max_dist_point_outline_line->setText(QString("%1").arg(
    MaximumDistancePointOutline(), 0, 'f', 1));
  max_num_pts_gap_selector->setValue(MaximumPointGapInOutlineSegment());
  max_size_gap_line->setText(QString("%1").arg(
    MaximumGapSizeInOutlineSegment(), 0, 'f', 1));
  min_num_pts_segment_selector->setValue(MinimumNumberOfPointsInOutlineSegment());
}

void OutliningParametersWindow::paintEvent(QPaintEvent *event)
{
  QPainter paint(this);
  int      y;

  if (event) y = 20; // To avoid compiler warnings
  y = 20;  paint.drawText(12, y, QString("Maximum distance point to intersection line:"));
  y += 20; paint.drawText(12, y, QString("Minimum angle between dominant directions:"));
  y += 20; paint.drawText(12, y, QString("Hough bin size for distance:"));
  y += 20; paint.drawText(12, y, QString("Hough bin size for direction:"));
  y += 20; paint.drawText(12, y, QString("Maximum distance point to outline:"));
  y += 20; paint.drawText(12, y, QString("Maximum number of points in gap:"));
  y += 20; paint.drawText(12, y, QString("Maximum gap size in meter:"));
  y += 20; paint.drawText(12, y, QString("Minimum number of points on outline segment:"));
}

void OutliningParametersWindow::SetMaximumDistancePointIntersectionLine(const QString &new_value)
{ if (strlen(new_value.toLatin1().data()) == 0) return;
  max_dist_point_intersection = new_value.toDouble(); }

void OutliningParametersWindow::SetMinimumAnglePreferredDirections(const QString &new_value)
{ double degree = atan(1.0) / 45.0;
  if (strlen(new_value.toLatin1().data()) == 0) return;
  min_angle_directions = new_value.toDouble() * degree; }

void OutliningParametersWindow::SetHoughBinSizeDistance(const QString &new_value)
{ if (strlen(new_value.toLatin1().data()) == 0) return;
  bin_size_dist = new_value.toDouble(); }

void OutliningParametersWindow::SetHoughBinSizeDirection(const QString &new_value)
{ double degree = atan(1.0) / 45.0;
  if (strlen(new_value.toLatin1().data()) == 0) return;
  bin_size_dir = new_value.toDouble() * degree; }

void OutliningParametersWindow::SetMaximumDistancePointOutline(const QString &new_value)
{ if (strlen(new_value.toLatin1().data()) == 0) return;
  max_dist_point_outline = new_value.toDouble(); }

void OutliningParametersWindow::SetMaximumPointGapInOutlineSegment(int new_number)
{ max_num_pts_gap = new_number; }

void OutliningParametersWindow::SetMaximumGapSizeInOutlineSegment(const QString &new_value)
{ if (strlen(new_value.toLatin1().data()) == 0) return;
  max_size_gap = new_value.toDouble(); }

void OutliningParametersWindow::SetMinimumNumberOfPointsInOutlineSegment(int new_number)
{ min_num_pts_segment = new_number; }
