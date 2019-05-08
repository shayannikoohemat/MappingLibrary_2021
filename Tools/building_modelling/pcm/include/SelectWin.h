
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


#ifndef SELECTWIN_H
#define SELECTWIN_H

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <QWidget>
#include <QLineEdit>
#include <QComboBox>
#include "LaserPoints.h"
#include "DataAppearance.h"

enum SelectWindowType {PointSelection=0, DisplayFilter};
                  
class SelectWindow : public QWidget
{
  Q_OBJECT

  protected:
    /// Select window type (0 = point selection, 1 = display filter)
    int window_type;

    /// Selector for point attributes
    QComboBox *point_attribute_selector[3];
  
    /// Edit line for minimum point attribute values
    QLineEdit *point_minimum_line[3];
  
    /// Edit line for maximum point attribute values
    QLineEdit *point_maximum_line[3];
    
    /// Selector for segment attributes
    QComboBox *segment_attribute_selector[3];
  
    /// Edit line for minimum segment attribute values
    QLineEdit *segment_minimum_line[3];
  
    /// Edit line for maximum segment attribute values
    QLineEdit *segment_maximum_line[3];
    
    /// Selector for logical operation
    QComboBox *operation_selector[5];
    
    /// Laser points
    LaserPoints *points;
    
    /// Selected laser points
    LaserPoints *selected_points;
  
    /// Segmentation parameters
    SegmentationParameters *segmentation_parameters;
    
    /// Laser data appearance for updating display filter bounds
    DataAppearance *laser_appearance;
    
  private:
    /// Retrieve point attributes from the window
    void RetrievePointAttributes(LaserPointTag *, double *minimum, double *maximum);
    
  public:

    /// Constructor
    SelectWindow(LaserPoints *laser_points, LaserPoints *selected_laser_points, 
                 SegmentationParameters *parameters, DataAppearance *appearance,
				 QWidget *parent=NULL);

    /// Default destructor
    ~SelectWindow() {};

  protected:

    /// Paint some text
    void paintEvent(QPaintEvent *event);
    
  private:
    /// Retrieve point attribute settings
    void RetrievePointBounds(int *attributes, double *minima, double *maxima, int num);

    /// Retrieve segment attribute settings
    void RetrieveSegmentBounds(int *attributes, double *minima, double *maxima, int num);

    /// Select data and display selected data in main window
    void SelectData(bool display_selected_data);
    
  public slots:
         
    /// Select points
    void SelectData();
       
    /// Delete points
    void DeleteData();
       
    /// Crop points
    void CropData();
    
    /// Clear selection
    void ClearSelection();
       
    /// Set display filter
    void SetDisplayFilter();
       
    /// Clear display filter
    void ClearDisplayFilter();
       
  signals:
    /// Update main window with new selection
    void RequestDisplayUpdate();
    
    /// Update main window with new display filter
    void RequestDisplayFilterUpdate();
};
#endif // SELECTWIN_H
