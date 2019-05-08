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


#include <math.h>
#include "PointInformationWin.h"
#include <QPainter>
#include <QScreen>
#include <QGuiApplication>

PointInformationWindow::PointInformationWindow(QWidget *parent) : QWidget(parent)
{
  max_num_attributes = 0;
  setFixedSize(250, 120 + 20 * max_num_attributes);	
  displayed_point = LaserPoint();
  setWindowTitle("Point information");
  positioned = false;
}

void PointInformationWindow::paintEvent(QPaintEvent *event)
{
  int      i, y, counted_doubles=0, xoff=30, yoff=-10, pc;
  QPainter paint(this);
  char     line[100];
  LaserPointTag tag;

  if (event) y = 20; // To avoid compiler warnings
  // Coordinates
  y = 20; paint.drawText(10, y, QString("Coordinates"));
  y+=20;  paint.drawText(20, y, QString("X"));
  sprintf(line, "%15.2f", displayed_point.X());
  paint.drawText(QRect(100+xoff, y+yoff, 100, 20), Qt::AlignRight, QString(line));
  y+=20;  paint.drawText(20, y, QString("Y"));
  sprintf(line, "%15.2f", displayed_point.Y());
  paint.drawText(QRect(100+xoff, y+yoff, 100, 20), Qt::AlignRight, QString(line));
  y+=20;  paint.drawText(20, y, QString("Z"));
  sprintf(line, "%15.2f", displayed_point.Z());
  paint.drawText(QRect(100+xoff, y+yoff, 100, 20), Qt::AlignRight, QString(line));

  // Attributes
  y += 30; paint.drawText(10, y, QString("Attributes"));
  for (i=0; i<displayed_point.NumAttributes(); i++) {
  	tag = (LaserPointTag) displayed_point.AttributeTags()[i];
	if (tag < NoTag) {
	  y += 20; paint.drawText(20, y, QString(AttributeName(tag, true)));
	  switch (AttributeType(tag)) {
	  	default:
	  	case IntegerAttributeType:
	  	  if (tag == ColourTag) {
	  	  	sprintf(line, "(%3d, %3d, %3d)", displayed_point.Red(),
	  	  	        displayed_point.Green(), displayed_point.Blue());
	  	  }	
	  	  else if (tag == PulseCountTag || tag == PulseCountWithFlagTag) {
	  	  	pc = displayed_point.Attribute(PulseCountWithFlagTag);
	  	  	sprintf(line, "%d", pc - ((pc >> 30) << 30));
	  	  }
	  	  else {
	  	    sprintf(line, "%d", displayed_point.Attribute(tag));
	  	  }
		  break;
	  	case FloatAttributeType:
	  	  if (tag == AzimuthTag || tag == InclinationTag || tag == AngleTag)
	  	    sprintf(line, "%.3f", displayed_point.FloatAttribute(tag) * 45.0 / atan(1.0));
	  	  else
	  	    sprintf(line, "%.3f", displayed_point.FloatAttribute(tag));
	      break;
	  	case DoubleAttributeType:
	  	  sprintf(line, "%.4f", displayed_point.DoubleAttribute(tag)); break;
	  }
      paint.drawText(QRect(100+xoff, y+yoff, 100, 20), Qt::AlignRight, QString(line));
      // Add last pulse flag
      if (tag == PulseCountTag || tag == PulseCountWithFlagTag) {
      	y += 20; paint.drawText(20, y, QString("Last pulse flag"));
      	sprintf(line, "%d", (pc >> 30));
        paint.drawText(QRect(100+xoff, y+yoff, 100, 20), Qt::AlignRight, QString(line));
        counted_doubles--;
      }
  	}
  	else counted_doubles++;
  }
  
  // Resize if needed
  if (displayed_point.NumAttributes() - counted_doubles > max_num_attributes) {
  	max_num_attributes = displayed_point.NumAttributes() - counted_doubles;
    setFixedSize(250, 120 + 20 * max_num_attributes);	
  }
}

void PointInformationWindow::DisplayPointInformation(const LaserPoint &point,
                                                     const QRect &main_frame_geometry)
{
  // Copy the point
  displayed_point = point;
  
  // Position the window next to the main window
  if (!positioned) {
  	int x, screen_width=QGuiApplication::primaryScreen()->availableSize().width();
  	// See if there's space on the left side
  	if (main_frame_geometry.x() > frameGeometry().width())
	  x = main_frame_geometry.x() - frameGeometry().width();
	// Then check for space on the right side
  	else if (screen_width - main_frame_geometry.x() - main_frame_geometry.width() >
	         frameGeometry().width())
  	  x = main_frame_geometry.x() + main_frame_geometry.width();
  	// If not, put the window at the far right
  	else
  	  x = screen_width - frameGeometry().width();
  	move(x, main_frame_geometry.y());
  	positioned = true;
  }

  // Repaint information window
  raise(); // Keep the window on top, even when mouse was clicked on canvas
  paintEvent(NULL);
  update();
}
