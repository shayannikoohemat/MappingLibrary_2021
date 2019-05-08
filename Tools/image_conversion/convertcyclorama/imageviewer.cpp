
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

using namespace std;

#include <QtGui>
#include <QLabel>
#include <QFileDialog>
#include <QMessageBox>
#include <QAction>
#include <QPrintDialog>
#include <QMenu>
#include <QMenuBar>
#include "math.h"
#include <iostream>

#include "imageviewer.h"

#define PI 3.1415926

ImageViewer::ImageViewer()
{
    imageLabel = new QLabel;
    imageLabel->setBackgroundRole(QPalette::Base);
    imageLabel->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    imageLabel->setScaledContents(true);
    

    //scrollArea = new QScrollArea;
    //scrollArea->setBackgroundRole(QPalette::Dark);
    //scrollArea->setWidget(imageLabel);
    setCentralWidget(imageLabel);
 
    createActions();
    createMenus();
    count_bound=0;
    left_pix_bound=right_pix_bound=-1;
    setWindowTitle(tr("Cyclorama converter"));
    resize(500, 400);
}

void ImageViewer::open()
{
    QString fileName = QFileDialog::getOpenFileName(this,
                                    tr("Open File"), QDir::currentPath());
    if (!fileName.isEmpty()) {
        src=new QImage(fileName);
        img=src->convertToFormat(QImage::Format_RGB32);
        
        if (img.isNull()) {
            QMessageBox::information(this, tr("Cyclorama converter"),
                                     tr("Cannot load %1.").arg(fileName));
            return;
        }
        imageLabel->setPixmap(QPixmap::fromImage(img));
        scaleFactor = 1.0;

        printAct->setEnabled(true);
        
        updateActions();
        count_bound=0;
        
            imageLabel->adjustSize();
        
    }
}

void ImageViewer::print()
{
    Q_ASSERT(imageLabel->pixmap());
    QPrintDialog dialog(&printer, this);
    if (dialog.exec()) {
        QPainter painter(&printer);
        QRect rect = painter.viewport();
        QSize size = imageLabel->pixmap()->size();
        size.scale(rect.size(), Qt::KeepAspectRatio);
        painter.setViewport(rect.x(), rect.y(), size.width(), size.height());
        painter.setWindow(imageLabel->pixmap()->rect());
        painter.drawPixmap(0, 0, *imageLabel->pixmap());
    }
}

void ImageViewer::zoomIn()
{
    scaleImage(1.25);
}

void ImageViewer::zoomOut()
{
    scaleImage(0.8);
}

void ImageViewer::normalSize()
{
    imageLabel->adjustSize();
    scaleFactor = 1.0;
}


void ImageViewer::about()
{
    QMessageBox::about(this, tr("About Cyclorama converter"),
            tr("<p>The <b>Cyclorama converter</b> is a super software. </p>"));
}

void ImageViewer::createActions()
{
    openAct = new QAction(tr("&Open..."), this);
    openAct->setShortcut(tr("Ctrl+O"));
    connect(openAct, SIGNAL(triggered()), this, SLOT(open()));

    printAct = new QAction(tr("&Print..."), this);
    printAct->setShortcut(tr("Ctrl+P"));
    printAct->setEnabled(false);
    connect(printAct, SIGNAL(triggered()), this, SLOT(print()));

    exitAct = new QAction(tr("E&xit"), this);
    exitAct->setShortcut(tr("Ctrl+Q"));
    connect(exitAct, SIGNAL(triggered()), this, SLOT(close()));

    zoomInAct = new QAction(tr("Zoom &In (25%)"), this);
    zoomInAct->setShortcut(tr("Ctrl++"));
    zoomInAct->setEnabled(false);
    connect(zoomInAct, SIGNAL(triggered()), this, SLOT(zoomIn()));

    zoomOutAct = new QAction(tr("Zoom &Out (25%)"), this);
    zoomOutAct->setShortcut(tr("Ctrl+-"));
    zoomOutAct->setEnabled(false);
    connect(zoomOutAct, SIGNAL(triggered()), this, SLOT(zoomOut()));

    normalSizeAct = new QAction(tr("&Normal Size"), this);
    normalSizeAct->setShortcut(tr("Ctrl+S"));
    normalSizeAct->setEnabled(false);
    connect(normalSizeAct, SIGNAL(triggered()), this, SLOT(normalSize()));

     perspectiveAct=new QAction(tr("&Convert perspective"), this);
     perspectiveAct->setShortcut(tr("Ctrl+C"));
     connect(perspectiveAct, SIGNAL(triggered()), this, SLOT(ConvertPerspective()));

    aboutAct = new QAction(tr("&About"), this);
    connect(aboutAct, SIGNAL(triggered()), this, SLOT(about()));

    aboutQtAct = new QAction(tr("About &Qt"), this);
    connect(aboutQtAct, SIGNAL(triggered()), qApp, SLOT(aboutQt()));
}

void ImageViewer::createMenus()
{
    fileMenu = new QMenu(tr("&File"), this);
    fileMenu->addAction(openAct);
    fileMenu->addAction(printAct);
    fileMenu->addSeparator();
    fileMenu->addAction(exitAct);

    viewMenu = new QMenu(tr("&View"), this);
    viewMenu->addAction(zoomInAct);
    viewMenu->addAction(zoomOutAct);
    viewMenu->addAction(normalSizeAct);
    viewMenu->addSeparator();
     
     viewMenu->addAction(perspectiveAct);

    helpMenu = new QMenu(tr("&Help"), this);
    helpMenu->addAction(aboutAct);
    helpMenu->addAction(aboutQtAct);

    menuBar()->addMenu(fileMenu);
    menuBar()->addMenu(viewMenu);
    menuBar()->addMenu(helpMenu);
}

void ImageViewer::updateActions()
{
    
}

void ImageViewer::scaleImage(double factor)
{
    Q_ASSERT(imageLabel->pixmap());
    scaleFactor *= factor;
    imageLabel->resize(scaleFactor * imageLabel->pixmap()->size());

    //adjustScrollBar(scrollArea->horizontalScrollBar(), factor);
   //adjustScrollBar(scrollArea->verticalScrollBar(), factor);

    zoomInAct->setEnabled(scaleFactor < 3.0);
    zoomOutAct->setEnabled(scaleFactor > 0.333);
}


/// Process mouse press events
 void ImageViewer::mousePressEvent(QMouseEvent *event)
   {
         //transform from imageviewer coordinate to imagelabel coordiate
      if(src==NULL)
      return;
      switch(event->button())
         {
         case Qt::LeftButton:
          
               set_boundary((event->x()*src->width())/imageLabel->width()); 
               break;
         }
   }
    /// Process mouse release events

 void ImageViewer::mouseReleaseEvent(QMouseEvent *event)
    {
         switch(event->button())
         {
         case Qt::LeftButton:
           
               break;
         }
   }
void ImageViewer::set_boundary(int x)
{
    
  
     
     if((count_bound%2)==0)
        {
                           
                          cout<<"draw left "<<x<<endl;
                           temp_left=src->convertToFormat(QImage::Format_RGB32);
                            QPainter painter(&temp_left);
                            painter.setCompositionMode(QPainter::CompositionMode_DestinationOver);
                           painter.setPen(QPen(Qt::black, 6, Qt::DashDotLine, Qt::RoundCap));
                           left_pix_bound=x;
                           painter.drawLine(QPoint(x,0),QPoint(x,temp_left.height()));
                           imageLabel->setPixmap(QPixmap::fromImage(temp_left));
                      
        }                  
     else    
        {
             cout<<"draw right "<<x<<endl;
             temp_right=src->convertToFormat(QImage::Format_RGB32);
             QPainter painter(&temp_right);
             painter.setCompositionMode(QPainter::CompositionMode_DestinationOver);
              painter.setPen(QPen(Qt::black, 6, Qt::DashDotLine, Qt::RoundCap));
              painter.drawLine(QPoint(left_pix_bound,0),QPoint(left_pix_bound,temp_right.height()));
             painter.setPen(QPen(Qt::red, 6, Qt::DashDotLine, Qt::RoundCap));
             right_pix_bound=x;
             painter.drawLine(QPoint(x,0),QPoint(x,temp_right.height()));
             
             imageLabel->setPixmap(QPixmap::fromImage(temp_right));
        }
        
      count_bound++;
}




void ImageViewer::ConvertPerspective()
{
     if(count_bound<2)
      {
      QMessageBox::information(this, "Error",
                                 "Left or right boundary not set.");
      return;
      }
     //calculate left and right pixel automatically
     
     
     double focal=500.0;
   
     printf("left and right is: %i   %i  \n",left_pix_bound, right_pix_bound);
     
     int w1, w2, w0,temp;
     int  w_dist, h_dist,  x, y, x0_cyclo, y0_cyclo, x_cyclo, y_cyclo, delta_w, delta_h;
     double alpha,beta, delta_x, delta_y;
     QImage *dist_image;
     QRgb pix;
     
     w1=left_pix_bound;  w2=right_pix_bound;
     
     
     if(w2<w1)
     {
        w2+=4800;         
     }
     
     
     w0=(w1+w2)/2;
     
     delta_w=w0-w1;  //add 50 pixels to left and right
     
     w2=w0+delta_w;
      
     printf("w1 and w2 is: %i   %i  \n",w1,w2);
     
     w_dist=focal*tan(pixel2rad(delta_w));
     
     h_dist=focal*tan(60*PI/180);
     
     
    printf("w_dist and h_dist is: %i   %i  \n",w_dist,h_dist);
     
     dist_image=new QImage(2*w_dist+1, 2*h_dist+1, src->format());
     
     x0_cyclo=w0;
     y0_cyclo=1200;   

    for(x=0;x<2*w_dist+1;x++)
       for(y=0;y<2*h_dist+1;y++)
       {
        delta_x=w_dist-x;  delta_y=h_dist-y;
        
        alpha=atan(delta_x/focal);
        
        beta=atan(delta_y*cos(alpha)/focal);
        
        x_cyclo=x0_cyclo-rad2pixel(alpha);
        y_cyclo=y0_cyclo-rad2pixel(beta);
        
        if(x_cyclo>=4800)
            x_cyclo=x_cyclo-4800;      
              
         pix=src->pixel(x_cyclo, y_cyclo);
         //printf("pixel is %i, %i \n",qRed(pix),qGreen(pix));
         dist_image->setPixel(x,y,pix);
         }
   
   
   QString texture_dir= QFileDialog::getSaveFileName(this,
                                    tr("Save file as"), QDir::currentPath());;
   
   dist_image->save(texture_dir.toLatin1(), "JPG"); 
       
}

int ImageViewer::rad2pixel(double rad)
{
    double pixel=rad*180.0*4800.0/(PI*360.0);
    
    return (int)pixel;
}

double ImageViewer::pixel2rad(int pixel)
{
    return pixel*PI/2400.0;
}
