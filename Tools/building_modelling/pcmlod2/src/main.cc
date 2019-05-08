
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


#include "PointCloudMapper.h"
#include <QApplication>
#include <QFont>

#include "digphot_arch.h"

int main( int argc, char ** argv )
{
  //QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
  QApplication a( argc, argv );
#ifdef linux
  QFont font=a.font();
  font.setPointSize(9);
  a.setFont(font);
#endif
  PointCloudMapper *main_window = new PointCloudMapper(&a);
  if (argc == 2) main_window->OpenProject(argv[1]);
  else if (argc == 3) main_window->OpenProject(argv[1], argv[2]);
  if (argc == 1) main_window->setWindowTitle( "Point cloud mapper" );
  else main_window->setWindowTitle( QString("Point cloud mapper - ").append(argv[1]) );
  main_window->show();
  a.connect(&a, SIGNAL(lastWindowClosed()), main_window, SLOT(QuitPCM()) );
  return a.exec();
}
