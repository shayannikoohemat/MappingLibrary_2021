
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


#ifndef TERRESTRIALPARWIN_H
#define TERRESTRIALPARWIN_H

#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <QWidget>

class TerrestrialParametersWindow : public QWidget
{
  Q_OBJECT

  public:

    /// Constructor
    TerrestrialParametersWindow(QWidget *parent=NULL);

    /// Default destructor
    ~TerrestrialParametersWindow() {};
    
    
    double street_min_area;
    double street_center_height;
    double wall_min_area;
    double wall_map_max_d;
    double roof_min_area;
    double roof_extru_min_area;
    double door_wall_max_distance;
    double door_min_area;
    double door_max_area;
    double door_max_height;
    double extru_wall_max_distance;
    double window_width;
    
    //for derive contour
    int step_time;
    int smoothness;
    
    //for fit wall outline
    double dist_accurate;
  
    //for visualization
    
    double back_offset;
   
    
    
  protected:

    /// Paint some text
    void paintEvent(QPaintEvent *event);
  

  public slots:
  void SetStreetMinArea(const QString &area);
  void SetStreetCenterHeight(const QString &area);
    void SetWallMinArea(const QString &area);
    void SetWalltoMapMaxDistance(const QString &area);
    void SetRoofMinArea(const QString &area);
    void SetRoofExtruMinArea(const QString &area);
    void SetDoorWallMaxDistance(const QString &area);
    void SetDoorMinArea(const QString &area);
    void SetDoorMaxArea(const QString &area);
    void SetDoorMaxHeight(const QString &area);
    void SetExtruWallMaxDistance(const QString &area);
    void SetMaxp2linedist(const QString &dist);
    void SetContourLevel(const int level);
    void SetSmoothness(const int level);
    void SetWindowWidth(const QString &dist);
    void SetBackOffset(const QString &dist);
    
   private:
            
    

   

   
};
#endif // TERRESTRIALPARWIN_H
