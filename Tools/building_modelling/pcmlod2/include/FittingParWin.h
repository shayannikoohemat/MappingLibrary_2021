
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


#ifndef FITTINGPARWIN_H
#define FITTINGPARWIN_H

#include <stdio.h>
#include <stdlib.h>
#include "FittingParameters.h"
#include <limits.h>
#include <QWidget>
#include <QSpinBox>
#include <QLineEdit>
#include <QComboBox>
#include <QCheckBox>

class FittingParametersWindow : public QWidget, public FittingParameters
{
  Q_OBJECT

  protected:
    /// Selector for using current or new segmentation
    QCheckBox *segmentation_selector;
    
    /// Selector for Hough model
    QComboBox *hough_model_selector;
    
    /// Edit line for maximum slope
    QLineEdit *max_slope_line;
    
    /// Edit line for slope bin size
    QLineEdit *slope_bin_line;
    
    /// Edit line for distance bin size
    QLineEdit *dist_bin_line;
    
    /// Edit line for minimum roof height
    QLineEdit *min_height_line;
    
    /// Selector for minimum number of points
    QSpinBox *min_numpts_selector;
    
    /// Selector for local maximum window size
    QSpinBox *local_win_size;
    
    /// Edit line for maximum distance of point to a plane
    QLineEdit *max_dist_plane_line;
    
    // Edit line for minimum distance between two planes
    QLineEdit *min_dist_planes_line;
    
    /// Edit line for minimum angle between two planes
    QLineEdit *min_angle_planes_line;
    
    /// Edit line for maximum angle of a horizontal plane
    QLineEdit *max_angle_hor_line;
    
    // Edit line for minimum distance intersection line to partition edge
    QLineEdit *min_dist_inter_line;
    
    // Edit line for minimum angle intersection line to partition edge
    QLineEdit *min_angle_inter_line;
    
    // Edit line for maximum distance point to intersection line
    QLineEdit *max_dist_inter_line;
    
    // Edit line for minimum distance height jump edge to partition edge
    QLineEdit *min_dist_jump_line;
    
    // Edit line for minimum angle height jump edge to partition edge
    QLineEdit *min_angle_jump_line;
    
    // Edit line for maximum distance point to height jump edge
    QLineEdit *max_dist_jump_line;
    
    // Edit line for minimum jump height
    QLineEdit *min_jump_line;
    
    // Selector for outline partitioning
    QCheckBox *outline_selector;
    
    // Edit line for minimum partition size
    QLineEdit *min_part_line;
    
    // Edit line for minimum partition percentage
    QLineEdit *min_perc_line;
    
  public:

    /// Constructor
    FittingParametersWindow(QWidget *parent=NULL);

    /// Default destructor
    ~FittingParametersWindow() {};

    // Update the setting of switches with the current parameters
    void Update();

  protected:

    /// Paint some text
    void paintEvent(QPaintEvent *event);
    
  public slots:

    /// Set use current segmentation switch
    void SetUseCurrentSegmentation(bool new_switch_value);
    
    /// Set the hough space model
    void SetHoughSpaceModel(int new_hough_model);

    /// Set the maximum slope
    void SetMaximumSlope(const QString &new_slope_max);

    /// Set the slope bin size
    void SetSlopeBinSize(const QString &new_slope_bin);

    /// Set the distance bin size
    void SetDistanceBinSize(const QString &new_dist_bin);

    /// Set the minimum height of roof points
    void SetMinimumRoofPointHeight(const QString &new_min_height);

    /// Set the minimum number of points in a plane
    void SetMinimumNumberOfPlanePoints(int new_min_numpts);

    /// Set the local maximum window size
    void SetLocalMaximumWindowSize(int new_local_max_window);

    /// Set the maximum distance between a point and a plane
    void SetMaximumDistancePointToPlane
      (const QString &new_max_dist_point_to_plane);

    /// Set the minimum distance between two planes
    void SetMinimumDistanceTwoPlanes(const QString &new_min_dist_planes);

    /// Set the minimum angle between two planes
    void SetMinimumAngleTwoPlanes(const QString &new_min_angle_planes);

    /// Set the maximum angle of a horizontal plane
    void SetMaximumAngleHorizontal(const QString &new_max_angle_horizontal);

    /// Set the minimum length of an intersection line inside a partition
    void SetMinimumDistanceIntersection
      (const QString &new_min_dist_intersection_partition);

    /// Set the minimum angle between an intersection line and a partition edge
    void SetMinimumAngleIntersection
      (const QString &new_min_angle_intersection_partition);

    /// Set the maximum distance of a point to an intersection line
    void SetMaximumDistancePointToIntersection
      (const QString &new_max_dist_point_to_intersection);

    /// Set the minimum length of a height jump edge inside a partition
    void SetMinimumDistanceHeightJump
      (const QString &new_min_dist_height_jump_partition);

    /// Set the minimum angle between a height jump edge and a partition edge
    void SetMinimumAngleHeightJump
      (const QString &new_min_angle_height_jump_partition);

    /// Set the maximum distance of a point to a height jump edge
    void SetMaximumDistancePointToHeightJump
      (const QString &new_max_dist_point_to_height_jump);

    /// Set the minmum height of a height jump edge
    void SetMinimumJumpHeight(const QString &new_min_jump_height);

    /// Set use initial partitioning switch
    void SetUseInitialPartitioning(bool new_switch_value);
    
    /// Set the minmum partition size
    void SetMinimumPartitionSize(const QString &new_min_partition_size);

    /// Set the minmum partition percentage
    void SetMinimumPartitionPercentage
      (const QString &new_min_partition_percentage);

};
#endif // FITTINGPARWIN_H
