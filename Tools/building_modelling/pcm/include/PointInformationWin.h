
/*
                  Copyright 2013 University of Twente
 
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


#ifndef POINTINFORMATIONWIN_H
#define POINTINFORMATIONWIN_H

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <QWidget>
#include "LaserPoint.h"
                  
class PointInformationWindow : public QWidget
{
  Q_OBJECT

  protected:
  	/// Copy of the point information to be displayed (saved for repainting)
  	LaserPoint displayed_point;
  	
  	/// Maximum number of attributes of points shown so far
  	int max_num_attributes;
  	
  	/// If false, the window still needs to be positioned next to the main window
  	bool positioned;
    
  public:

    /// Constructor
    PointInformationWindow(QWidget *parent=NULL);

    /// Default destructor
    ~PointInformationWindow() {};

  protected:

    /// Paint some text
    void paintEvent(QPaintEvent *event);
    
  public slots:
  	/// Display point information
  	void DisplayPointInformation(const LaserPoint &point,
	                             const QRect &main_frame_geometry);
};
#endif // POINTINFORMATIONWIN_H
