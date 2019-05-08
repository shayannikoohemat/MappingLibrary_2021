
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


#include "City.h"
#include <qnamespace.h> 
#include <qapplication.h> 
#include <QMessageBox>
#include <QPixmap>
#include <QFont>
#include <Qt>
#include <QPlastiqueStyle>
#include <qsplashscreen.h> 
#include "digphot_arch.h"
#include <QDir>
#include <iostream.h>
#include <QPluginLoader>
#include <QTranslator>
int main( int argc, char ** argv )
{
 QSplashScreen *splash = NULL;
  QApplication a( argc, argv );

QPluginLoader jpegloader("c:/CITY/bin/qjpeg1.dll");
jpegloader.load();

 QTranslator translator;
        translator.load("spreadsheet_cn");
        a.installTranslator(&translator);

  QFont font=a.font();
  font.setPointSize(10);
  a.setFont(font);
 // QDir::addResourceSearchPath("c:\CITY");
  QString splash_str("C:\\CITY\\splash.jpg");
 //QImageReader imager(splash_str);
 //printf("\n %i \n",imager.error());
//cout<<(imager.errorString ()).toAscii()<<endl;
  QApplication::setStyle(new QPlastiqueStyle);
   QPixmap pixmap(splash_str);
		splash = new QSplashScreen(pixmap);
		font.setPointSize(16);
        splash->setFont(font);
		splash->show();
		Qt::Alignment bottomright=Qt::AlignRight|Qt::AlignBottom;
		splash->showMessage("Loading...",bottomright, Qt::white);
  City *main_window = new City();
  if (argc > 1) main_window->SelectProject(argv[1]);
 
  //splash->message( "Loading...");
  
 
  
  main_window->setWindowTitle( "City" );
  main_window->show(); 
   //splash->finish( main_window );
  main_window->showMaximized ();
  

 
  delete splash;

 // City *main_window = new City();
  if (argc > 1) main_window->SelectProject(argv[1]);
  main_window->setWindowTitle( "City" );
  main_window->show();
  a.connect(&a, SIGNAL(lastWindowClosed()), &a, SLOT(quit()) );
  return a.exec();
}
