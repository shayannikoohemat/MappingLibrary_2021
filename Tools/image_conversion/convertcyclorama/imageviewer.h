
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

#ifndef IMAGEVIEWER_H
#define IMAGEVIEWER_H

#include <QMainWindow>
#include <QPrinter>
#include <QImage>

class QAction;
class QLabel;
class QMenu;
class QScrollArea;
class QScrollBar;

class ImageViewer : public QMainWindow
{
    Q_OBJECT

public:
    ImageViewer();

private slots:
    void open();
    void print();
    void zoomIn();
    void zoomOut();
    void normalSize();
    void ConvertPerspective();
    void about();
 protected:
       /// Process mouse press events
      void mousePressEvent(QMouseEvent *event);
      /// Process mouse release events
      void mouseReleaseEvent(QMouseEvent *event);   
      
           
private:
    void createActions();
    void createMenus();
    void updateActions();
    void scaleImage(double factor);
   
    void set_boundary(int);
    int rad2pixel(double);
    double pixel2rad(int);
     
     
    QLabel *imageLabel;
    QScrollArea *scrollArea;
    double scaleFactor;

    QPrinter printer;

    QAction *openAct;
    QAction *printAct;
    QAction *exitAct;
    QAction *zoomInAct;
    QAction *zoomOutAct;
    QAction *normalSizeAct;
    QAction *fitToWindowAct;
    QAction *aboutAct;
    QAction *aboutQtAct;
    QAction *perspectiveAct;
    
    QMenu *fileMenu;
    QMenu *viewMenu;
    QMenu *helpMenu;
    
    QImage *src;
    QImage img,temp_left, temp_right;
    
    int left_pix_bound, right_pix_bound;
     int count_bound;
     bool shift_down, ctrl_down;
};

#endif
