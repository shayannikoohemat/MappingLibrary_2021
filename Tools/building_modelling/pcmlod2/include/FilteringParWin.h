
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


#ifndef FILTERINGPARWIN_H
#define FILTERINGPARWIN_H

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <QWidget>
#include <QLineEdit>
#include <QCheckBox>
#include "FilteringParameters.h"

class FilteringParametersWindow : public QWidget,
                                  public FilteringParameters
{
  Q_OBJECT

  protected:

    /// Edit lines for specifying allowed height differences
    QLineEdit *height_dif_line[2];

    /// Filter radius
    QLineEdit *filter_radius_line;

    /// Output selector
    QCheckBox *remove_non_ground_points_selector;
        
  public:

    /// Constructor
    FilteringParametersWindow(QWidget *parent=NULL);

    /// Default destructor
    ~FilteringParametersWindow() {};

  protected:

    /// Paint some text
    void paintEvent(QPaintEvent *event);

  public slots:

    /// Update the parameter settings in the window
    void Update();
             
    /// Set height difference at 0 m
    void SetHeightDifAtSameLocation(const QString &);
    
    /// Set the filter radius
    void SetFilterRadius(const QString &);
    
    /// Set height difference at maximum filter radius
    void SetHeightDifAtFilterRadius(const QString &);
    
    /// Set remove non-ground points switch
    void SetRemoveNonGroundPoints(bool new_switch_value);
};
#endif // FILTERINGPARWIN_H
