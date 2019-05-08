
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


#ifndef OUTLININGPARWIN_H
#define OUTLININGPARWIN_H

#include <stdio.h>
#include <stdlib.h>
#include "OutliningParameters.h"
#include <limits.h>
#include <QWidget>
#include <QSpinBox>
#include <QLineEdit>
#include <QComboBox>
#include <QCheckBox>

class OutliningParametersWindow : public QWidget, 
                                  public OutliningParameters
{
  Q_OBJECT

  protected:

    /// Edit line for maximum dist between a point and an intersection line
    QLineEdit *max_dist_point_intersection_line;
  
    /// Edit line for minimum angle between preferred directions
    QLineEdit *min_angle_directions_line;
  
    /// Edit line for Hough space bin size for distance
    QLineEdit *hough_bin_size_dist_line;
  
    /// Edit line for Hough space bin size for direction
    QLineEdit *hough_bin_size_dir_line;
  
    /// Edit line for maximum dist between a point and an outline
    QLineEdit *max_dist_point_outline_line;
  
    /// Selector for maximum gap size in outline segment in number of points
    QSpinBox *max_num_pts_gap_selector;

    /// Edit line for maximum gap size in outline segment in meters
    QLineEdit *max_size_gap_line;
  
    /// Selector for minumum number of points in an outline segment
    QSpinBox *min_num_pts_segment_selector;

  
  public:

    /// Constructor
    OutliningParametersWindow(QWidget *parent=NULL);

    /// Default destructor
    ~OutliningParametersWindow() {};

  protected:

    /// Paint some text
    void paintEvent(QPaintEvent *event);

  public slots:
         
    /// Set maximum distance between a point and an intersection line
    void SetMaximumDistancePointIntersectionLine(const QString &new_value);

    /// Set minimum angle between preferred directions
    void SetMinimumAnglePreferredDirections(const QString &new_value);

    /// Set Hough space bin size for distance
    void SetHoughBinSizeDistance(const QString &new_value);

    /// Set Hough space bin size for direction
    void SetHoughBinSizeDirection(const QString &new_value);

    /// Set maximum distance between a point and an outline
    void SetMaximumDistancePointOutline(const QString &new_value);

    /// Set maximum gap size in outline segment in number of points
    void SetMaximumPointGapInOutlineSegment(int new_number);

    /// Set maximum gap size in outline segment in meters
    void SetMaximumGapSizeInOutlineSegment(const QString &new_value);

    /// Set minumum number of points in an outline segment
    void SetMinimumNumberOfPointsInOutlineSegment(int new_number);
    
    /// Update switches according to current parameter settings
    void Update();
};
#endif // OUTLININGPARWIN_H
