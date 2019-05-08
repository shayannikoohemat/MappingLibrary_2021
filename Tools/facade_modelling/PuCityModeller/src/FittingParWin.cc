
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
#include "FittingParWin.h"
#include <QPainter>
#include <QValidator>

FittingParametersWindow::FittingParametersWindow(QWidget *parent) :
  QWidget(parent), FittingParameters()
{
  QDoubleValidator *positive_double;
  int              y;
  double           degree = 45 / atan(1.0);

  positive_double = new QDoubleValidator(0.01, 1.0e10, 2, this);

  setFixedSize(350, 590);

// Plane detection parameters

  // Switch for letting surfaces compete
  segmentation_selector = new QCheckBox(this);
  segmentation_selector->setChecked(UseCurrentSegmentation());
  y = 25; segmentation_selector->setGeometry(205, y+2, 40, 18);
  connect(segmentation_selector, SIGNAL(toggled(bool)),
          this, SLOT(SetUseCurrentSegmentation(bool)));

  // Hough model
  hough_model_selector = new QComboBox(this);
  hough_model_selector->setEditable(false);
  hough_model_selector->setFocusPolicy(Qt::NoFocus);
  hough_model_selector->addItem(QString("Slope rate"));
  hough_model_selector->addItem(QString("Slope angle"));
  hough_model_selector->setCurrentIndex(HoughSpaceModel());
  y += 20; hough_model_selector->setGeometry(115, y, 90, 18);
  connect(hough_model_selector, SIGNAL(activated(int)),
          this, SLOT(SetHoughSpaceModel(int)));

  // Maximum slope
  max_slope_line = new QLineEdit(QString("%1").arg(MaximumSlope(), 0, 'f', 1), this);
  max_slope_line->setValidator(positive_double);
  y += 20; max_slope_line->setGeometry(140, y, 40, 18);
  connect(max_slope_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaximumSlope(const QString &)));

  // Bin size of slope parameter
  slope_bin_line = new QLineEdit(QString("%1").arg(SlopeBinSize(), 0, 'f', 1), this);
  slope_bin_line->setValidator(positive_double);
  y += 20; slope_bin_line->setGeometry(140, y, 40, 18);
  connect(slope_bin_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetSlopeBinSize(const QString &)));

  // Bin size of distance parameter
  dist_bin_line =
    new QLineEdit(QString("%1").arg(DistanceBinSize(), 0, 'f', 1), this);
  dist_bin_line->setValidator(positive_double);
  y += 20; dist_bin_line->setGeometry(140, y, 40, 18);
  connect(dist_bin_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetDistanceBinSize(const QString &)));

  // Minimum roof point height
  min_height_line =
    new QLineEdit(QString("%1").arg(MinimumRoofPointHeight(), 0, 'f', 1), this);
  min_height_line->setValidator(positive_double);
  y += 20; min_height_line->setGeometry(220, y, 40, 18);
  connect(min_height_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMinimumRoofPointHeight(const QString &)));

  // Minimum number of plane points
  min_numpts_selector = new QSpinBox(this);
  min_numpts_selector->setMinimum(3);
  min_numpts_selector->setSingleStep(1);
  min_numpts_selector->setValue(MinimumNumberOfPlanePoints());
  y += 20; min_numpts_selector->setGeometry(253, y, 35, 18);
  connect(min_numpts_selector, SIGNAL(valueChanged(int)),
          this, SLOT(SetMinimumNumberOfPlanePoints(int)));

  // Local maximum window size
  local_win_size = new QSpinBox(this);
  local_win_size->setMinimum(3);
  local_win_size->setSingleStep(1);
  local_win_size->setValue(LocalMaximumWindowSize());
  y += 20; local_win_size->setGeometry(220, y, 35, 18);
  connect(local_win_size, SIGNAL(valueChanged(int)),
          this, SLOT(SetLocalMaximumWindowSize(int)));

  // Maximum distance of point to a plane
  max_dist_plane_line =
    new QLineEdit(QString("%1").arg(MaximumDistancePointToPlane(), 0, 'f', 1),
                  this);
  max_dist_plane_line->setValidator(positive_double);
  y += 20; max_dist_plane_line->setGeometry(220, y, 40, 18);
  connect(max_dist_plane_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaximumDistancePointToPlane(const QString &)));

// Plane constraints

  // Minimum distance between two planes
  min_dist_planes_line =
    new QLineEdit(QString("%1").arg(MinimumDistanceTwoPlanes(), 0, 'f', 1),
                  this);
  min_dist_planes_line->setValidator(positive_double);
  y += 50; min_dist_planes_line->setGeometry(240, y, 40, 18);
  connect(min_dist_planes_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMinimumDistanceTwoPlanes(const QString &)));

  // Minimum angle between two planes
  min_angle_planes_line =
    new QLineEdit(QString("%1").arg(MinimumAngleTwoPlanes() * degree, 0, 'f', 1), this);
  min_angle_planes_line->setValidator(positive_double);
  y += 20; min_angle_planes_line->setGeometry(240, y, 40, 18);
  connect(min_angle_planes_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMinimumAngleTwoPlanes(const QString &)));

  // Maximum angle of a horizontal plane
  max_angle_hor_line =
    new QLineEdit(QString("%1").arg(MaximumAngleHorizontal() * degree, 0, 'f', 1), this);
  max_angle_hor_line->setValidator(positive_double);
  y += 20; max_angle_hor_line->setGeometry(240, y, 40, 18);
  connect(max_angle_hor_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaximumAngleHorizontal(const QString &)));

// Partitioning by intersection lines

  // Minimum distance intersection line to partition edge
  min_dist_inter_line =
    new QLineEdit(QString("%1").arg(MinimumDistanceIntersection(), 0, 'f', 1),
                  this);
  min_dist_inter_line->setValidator(positive_double);
  y += 50; min_dist_inter_line->setGeometry(270, y, 40, 18);
  connect(min_dist_inter_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMinimumDistanceIntersection(const QString &)));

  // Minimum angle intersection line to partition edge
  min_angle_inter_line =
    new QLineEdit(QString("%1").arg(MinimumAngleIntersection() * degree, 0, 'f', 1),
                  this);
  min_angle_inter_line->setValidator(positive_double);
  y += 20; min_angle_inter_line->setGeometry(270, y, 40, 18);
  connect(min_angle_inter_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMinimumAngleIntersection(const QString &)));

  // Maximum distance point to intersection line
  max_dist_inter_line =
    new QLineEdit(QString("%1").arg(MaximumDistancePointToIntersection(),
                                    0, 'f', 1), this);
  max_dist_inter_line->setValidator(positive_double);
  y += 20; max_dist_inter_line->setGeometry(300, y, 40, 18);
  connect(max_dist_inter_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaximumDistancePointToIntersection(const QString &)));

// Partitioning by height jump edges

  // Minimum distance height jump edge to partition edge
  min_dist_jump_line =
    new QLineEdit(QString("%1").arg(MinimumDistanceHeightJump(), 0, 'f', 1),
                  this);
  min_dist_jump_line->setValidator(positive_double);
  y += 50; min_dist_jump_line->setGeometry(270, y, 40, 18);
  connect(min_dist_jump_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMinimumDistanceHeightJump(const QString &)));

  // Minimum angle height jump edge to partition edge
  min_angle_jump_line =
    new QLineEdit(QString("%1").arg(MinimumAngleHeightJump() * degree, 0, 'f', 1),
                  this);
  min_angle_jump_line->setValidator(positive_double);
  y += 20; min_angle_jump_line->setGeometry(270, y, 40, 18);
  connect(min_angle_jump_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMinimumAngleHeightJump(const QString &)));

  // Maximum distance point to height jump edge
  max_dist_jump_line =
    new QLineEdit(QString("%1").arg(MaximumDistancePointToHeightJump(),
                                    0, 'f', 1), this);
  max_dist_jump_line->setValidator(positive_double);
  y += 20; max_dist_jump_line->setGeometry(300, y, 40, 18);
  connect(max_dist_jump_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMaximumDistancePointToHeightJump(const QString &)));

  // Minimum jump height
  min_jump_line =
    new QLineEdit(QString("%1").arg(MinimumJumpHeight(), 0, 'f', 1), this);
  min_jump_line->setValidator(positive_double);
  y += 20; min_jump_line->setGeometry(170, y, 40, 18);
  connect(min_jump_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMinimumJumpHeight(const QString &)));

// General partitioning parameters

  // Switch for using initial partitioning of map outline
  outline_selector = new QCheckBox(this);
  outline_selector->setChecked(UseInitialPartitioning());
  y += 50; outline_selector->setGeometry(205, y+2, 40, 18);
  connect(outline_selector, SIGNAL(toggled(bool)),
          this, SLOT(SetUseInitialPartitioning(bool)));

  // Minimum partition size
  min_part_line =
    new QLineEdit(QString("%1").arg(MinimumPartitionSize(), 0, 'f', 1), this);
  min_part_line->setValidator(positive_double);
  y += 20; min_part_line->setGeometry(210, y, 40, 18);
  connect(min_part_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMinimumPartitionSize(const QString &)));

  // Minimum partition percentage
  min_perc_line =
    new QLineEdit(QString("%1").arg(MinimumPartitionPercentage(), 0, 'f', 1),
                  this);
  min_perc_line->setValidator(positive_double);
  y += 20; min_perc_line->setGeometry(210, y, 40, 18);
  connect(min_perc_line, SIGNAL(textChanged(const QString &)),
          this, SLOT(SetMinimumPartitionPercentage(const QString &)));
}

void FittingParametersWindow::Update()
{
  double degree = 45 / atan(1.0);

  segmentation_selector->setChecked(UseCurrentSegmentation());
  hough_model_selector->setCurrentIndex(HoughSpaceModel());
  if (hough_model == 1) { // Convert to degrees
    max_slope_line->setText(QString("%1").arg(MaximumSlope() * degree, 0, 'f', 1));
    slope_bin_line->setText(QString("%1").arg(SlopeBinSize() * degree, 0, 'f', 1));
  }
  else {
    max_slope_line->setText(QString("%1").arg(MaximumSlope(), 0, 'f', 1));
    slope_bin_line->setText(QString("%1").arg(SlopeBinSize(), 0, 'f', 1));
  }
  dist_bin_line->setText(QString("%1").arg(DistanceBinSize(), 0, 'f', 1));
  min_height_line->setText(QString("%1").arg(MinimumRoofPointHeight(), 0, 'f', 1));
  min_numpts_selector->setValue(MinimumNumberOfPlanePoints());
  local_win_size->setValue(LocalMaximumWindowSize());
  max_dist_plane_line->setText(QString("%1").arg(MaximumDistancePointToPlane(), 0, 'f', 1));
  min_dist_planes_line->setText(QString("%1").arg(MinimumDistanceTwoPlanes(), 0, 'f', 1));
  min_angle_planes_line->setText(QString("%1").arg(MinimumAngleTwoPlanes() * degree, 0, 'f', 1));
  max_angle_hor_line->setText(QString("%1").arg(MaximumAngleHorizontal() * degree, 0, 'f', 1));
  min_dist_inter_line->setText(QString("%1").arg(MinimumDistanceIntersection(), 0, 'f', 1));
  min_angle_inter_line->setText(QString("%1").arg(MinimumAngleIntersection() * degree, 0, 'f', 1));
  max_dist_inter_line->setText(QString("%1").arg(MaximumDistancePointToIntersection(), 0, 'f', 1));
  min_dist_jump_line->setText(QString("%1").arg(MinimumDistanceHeightJump(), 0, 'f', 1));
  min_angle_jump_line->setText(QString("%1").arg(MinimumAngleHeightJump() * degree, 0, 'f', 1));
  max_dist_jump_line->setText(QString("%1").arg(MaximumDistancePointToHeightJump(), 0, 'f', 1));
  min_jump_line->setText(QString("%1").arg(MinimumJumpHeight(), 0, 'f', 1));
  outline_selector->setChecked(UseInitialPartitioning());
  min_part_line->setText(QString("%1").arg(MinimumPartitionSize(), 0, 'f', 1));
  min_perc_line->setText(QString("%1").arg(MinimumPartitionPercentage(), 0, 'f', 1));
}

void FittingParametersWindow::paintEvent(QPaintEvent *)
{
  QPainter paint(this);
  int      y;

  // Plane detection parameters
  y = 20; paint.drawText(12, y, QString("Plane detection parameters"));
  y += 20; paint.drawText(32, y, QString("Use current segmentation"));
  y += 20; paint.drawText(32, y, QString("Hough model:"));
  y += 20; paint.drawText(32, y, QString("Maximum slope:"));
  y += 20; paint.drawText(32, y, QString("Bin size slope:"));
  y += 20; paint.drawText(32, y, QString("Bin size distance:")); y += 20;
  paint.drawText(32, y, QString("Minimum height of roof points:")); y += 20;
  paint.drawText(32, y, QString("Minimum number of points in a plane:"));
  y += 20; paint.drawText(32, y, QString("Local maximum window size:"));
  y += 20; paint.drawText(32, y, QString("Maximum distance point-plane:"));

  // Plane constraints
  y+=30; paint.drawText(12, y, QString("Plane constraints"));
  y+=20; paint.drawText(32, y, QString("Minimum distance between planes"));
  y+=20; paint.drawText(32, y, QString("Minimum angle between planes"));
  y+=20; paint.drawText(32, y, QString("Minimum angle of horizontal plane"));

  // Partition by intersection lines
  y+=30; paint.drawText(12, y, QString("Partitioning by intersection lines"));
  y+=20;
  paint.drawText(32, y, QString("Minimum distance line to partition edge"));
  y+=20;
  paint.drawText(32, y, QString("Minimum angle line with partition edge"));
  y+=20;
  paint.drawText(32, y, QString("Maximum distance point to intersection line"));

  // Partition by height jump edges
  y+=30; paint.drawText(12, y, QString("Partitioning by height jump edges"));
  y+=20;
  paint.drawText(32, y, QString("Minimum distance edge to partition edge"));
  y+=20;
  paint.drawText(32, y, QString("Minimum angle edge with partition edge"));
  y+=20;
  paint.drawText(32, y, QString("Maximum distance point to height jump edge"));
  y+=20; paint.drawText(32, y, QString("Minimum jump height"));

  // General partition parameters
  y+=30; paint.drawText(12, y, QString("General partition parameters"));
  y+=20; paint.drawText(32, y, QString("Use initial partitioning"));
  y+=20; paint.drawText(32, y, QString("Minimum partition size"));
  y+=20; paint.drawText(32, y, QString("Minimum partition percentage"));
}

void FittingParametersWindow::SetUseCurrentSegmentation(bool new_switch_value)
{ use_current_segmentation = new_switch_value; }

void FittingParametersWindow::SetHoughSpaceModel(int new_hough_model)
{ hough_model = new_hough_model; }

void FittingParametersWindow::SetMaximumSlope(const QString &new_slope_max)
{
  slope_max = new_slope_max.toDouble();
  if (hough_model == 1) slope_max *= atan(1.0) / 45.0; // degrees to radians
}

void FittingParametersWindow::SetSlopeBinSize(const QString &new_slope_bin)
{
  slope_bin = new_slope_bin.toDouble();
  if (hough_model == 1) slope_bin *= atan(1.0) / 45.0; // degrees to radians
}

void FittingParametersWindow::SetDistanceBinSize(const QString &new_dist_bin)
{ dist_bin = new_dist_bin.toDouble(); }

void FittingParametersWindow::SetMinimumRoofPointHeight
  (const QString &new_min_height)
{ min_height = new_min_height.toDouble(); }

void FittingParametersWindow::SetMinimumNumberOfPlanePoints(int new_min_numpts)
{ min_numpts = new_min_numpts; }

void FittingParametersWindow::SetLocalMaximumWindowSize(int new_local_max_win)
{ local_max_window = new_local_max_win; }

void FittingParametersWindow::SetMaximumDistancePointToPlane
  (const QString &new_max_dist_point_to_plane)
{ max_dist_point_to_plane = new_max_dist_point_to_plane.toDouble(); }

void FittingParametersWindow::SetMinimumDistanceTwoPlanes
  (const QString &new_min_dist_planes)
{ min_dist_planes = new_min_dist_planes.toDouble(); }

void FittingParametersWindow::SetMinimumAngleTwoPlanes
  (const QString &new_min_angle_planes)
{ min_angle_planes = new_min_angle_planes.toDouble() * atan(1.0) / 45.0; }

void FittingParametersWindow::SetMaximumAngleHorizontal
  (const QString &new_max_angle_horizontal)
{ max_angle_horizontal = new_max_angle_horizontal.toDouble() * atan(1.0) / 45.0; }

void FittingParametersWindow::SetMinimumDistanceIntersection
  (const QString &new_min_dist_intersection_partition)
{ min_dist_intersection_partition =
    new_min_dist_intersection_partition.toDouble();
}

void FittingParametersWindow::SetMinimumAngleIntersection
  (const QString &new_min_angle_intersection_partition)
{ min_angle_intersection_partition =
    new_min_angle_intersection_partition.toDouble() * atan(1.0) / 45.0;
}

void FittingParametersWindow::SetMaximumDistancePointToIntersection
  (const QString &new_max_dist_point_to_intersection)
{ max_dist_point_to_intersection =
    new_max_dist_point_to_intersection.toDouble();
}

void FittingParametersWindow::SetMinimumDistanceHeightJump
  (const QString &new_min_dist_height_jump_partition)
{ min_dist_height_jump_partition =
    new_min_dist_height_jump_partition.toDouble();
}

void FittingParametersWindow::SetMinimumAngleHeightJump
  (const QString &new_min_angle_height_jump_partition)
{ min_angle_height_jump_partition =
    new_min_angle_height_jump_partition.toDouble() * atan(1.0) / 45.0;
}

void FittingParametersWindow::SetMaximumDistancePointToHeightJump
  (const QString &new_max_dist_point_to_height_jump)
{ max_dist_point_to_height_jump =
    new_max_dist_point_to_height_jump.toDouble();
}

void FittingParametersWindow::SetMinimumJumpHeight
  (const QString &new_min_jump_height)
{ min_jump_height = new_min_jump_height.toDouble(); }

void FittingParametersWindow::SetUseInitialPartitioning(bool new_switch_value)
{ use_initial_partitioning = new_switch_value; }

void FittingParametersWindow::SetMinimumPartitionSize
  (const QString &new_min_partition_size)
{ min_partition_size = new_min_partition_size.toDouble(); }

void FittingParametersWindow::SetMinimumPartitionPercentage
  (const QString &new_min_partition_percentage)
{ min_partition_percentage = new_min_partition_percentage.toDouble(); }
