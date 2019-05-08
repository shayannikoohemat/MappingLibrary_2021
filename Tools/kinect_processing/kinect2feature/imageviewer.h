/****************************************************************************
**
** Copyright (C) 2005-2005 Trolltech AS. All rights reserved.
**
** This file is part of the example classes of the Qt Toolkit.
**
** This file may be used under the terms of the GNU General Public
** License version 2.0 as published by the Free Software Foundation
** and appearing in the file LICENSE.GPL included in the packaging of
** this file.  Please review the following information to ensure GNU
** General Public Licensing requirements will be met:
** http://www.trolltech.com/products/qt/opensource.html
**
** If you are unsure which license is appropriate for your use, please
** review the following information:
** http://www.trolltech.com/products/qt/licensing.html or contact the
** sales department at sales@trolltech.com.
**
** This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
** WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
**
****************************************************************************/

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
