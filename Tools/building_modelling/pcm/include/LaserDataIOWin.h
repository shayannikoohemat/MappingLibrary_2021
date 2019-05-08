
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


#ifndef LASERDATAIOWIN_H
#define LASERDATAIOWIN_H

#include <stdio.h>
#include <stdlib.h>
#include "LaserPoints.h"
#include <limits.h>
#include <QWidget>
#include <QSpinBox>
#include <QCheckBox>
#include <QPushButton>
#include <QApplication>

class LaserDataIOWindow : public QWidget 
{
  Q_OBJECT

  protected:

    /// Pointer to the main application
    QApplication *main_application;
    
    /// Attribute selectors
    QCheckBox *attribute_selector[15];
    
    /// Column selectors
    QSpinBox *column_selector[15];

    /// Number of header lines selector
    QSpinBox *headerlines_selector;
    
    /// Output of number of points selector
    QCheckBox *header_selector;
    
    /// Import button
    QPushButton *import_button;
    
    /// Export button
    QPushButton *export_button;
        
    /// Laser data file name
    char *filename;
    
    /// Laser points
    LaserPoints *laser_points;
 
    bool currently_importing;
    
  public:

    /// Constructor
    LaserDataIOWindow(QApplication *app, QWidget *parent=NULL);

    /// Default destructor
    ~LaserDataIOWindow() {};

    /// Import laser data
    void Import(char *name, LaserPoints *points);

    /// Export laser data
    void Export(char *name, LaserPoints *points);

  protected:

    /// Paint some text
    void paintEvent(QPaintEvent *event);

  private slots:
             
    /// Import laser data
    void ImportLaserData();
    
    /// Export laser data
    void ExportLaserData();
    
  signals:
    /// Request data display in main window
    void RequestLaserDataDisplay();

    /// Request export message in main window
    void RequestExportMessage();
};
#endif // LASERDATAIOWIN_H
