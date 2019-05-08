
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


#include <QApplication>
#include <QSplashScreen>
#include <QMessageBox>
#include <QPixmap>
#include <QThread>
#include "QGLGeneralCanvas.h"
#include "QGLPointsCanvas.h"
#include "LaserPointsUtility.h"
#include <iostream>
#include <cstdio>
#include <QtProgressDisplay.h>
#include <LaserSegments.h>
#include <MgcCommand.h>
#include <string>

#include <QRegistrationICP.h>
#include <QRegistrationIndirect.h>
#include <LaserScan.h>


//Only for linux
#ifdef linux
	#include <unistd.h>
#endif

void Redirect()
{
	freopen ("stderr.txt","w",stderr);
	freopen ("stdout.txt","w",stdout);
}
  

int  main(int argc, char *argv[])
{
    //TEST: stxxl here.
	//int main_stxxl();main_stxxl();
			
	//***** Done testing *****
	
	//Redirect stderr to a file so that we can see the contents.
	//There is no console attached to an application in windows.
	Redirect();
	
	static string Usage = "USAGE: qtlaserviewer \n" 
	"\t [-pts input_points] [-seg segmentation_file] [-obj objects_file]\n"
	"\t [-sel=selection_file]  [-normals normals_file] \n"
	"\t [-help->will show help screen] [-splash=show splash screen]\n";
			
	#define SHOW_USE cout << Usage <<endl;
	
	
	
	
	QApplication app(argc, argv);

	//We have to make sure that we show graphical progress bar in Qt applications.
	ProgressDisplayManager::SetProgressObject(new QtProgressDisplay<int>());

	QSplashScreen *splash = NULL;
	QGLPointsCanvas ptCanvas(NULL);
    
    //Process command line
	string ptsFile, selFile, segFile, normalsFile, objFile;
	bool bTest;
		
	Command kCmd(argc,argv);
	
	//Show the splash screen
	
	if(kCmd.Boolean("splash",bTest))
	{
		QPixmap pixmap(":/images/Splash.png");
		splash = new QSplashScreen(pixmap);
		splash->show();

		//Lets sleep for a while.
	#ifdef windows		
		::Sleep(1500);
	#else
		//::sleep(1500);
	#endif				

	}

	
	if(kCmd.Boolean("help",bTest))
	{
		QMessageBox::information(NULL,"Usage help", QString(Usage.c_str()),QMessageBox::Ok);
	}
	
	//Read points
	kCmd.String("pts",ptsFile);
	if(!kCmd.GetLastError())
	{
		ptCanvas.LoadPoints(ptsFile.c_str());	
	}
	else
	{
		LaserPoints pts = MakeSinePoints( );
    	ptCanvas.SetLaserPoints(pts);
	}
	
	//Read selection.
	kCmd.String("sel",selFile);
	if(!kCmd.GetLastError())
	{
		ptCanvas.LoadSelectedIndices(selFile.c_str());
	}
	
	//Read objects.
	kCmd.String("obj",objFile);
	if(!kCmd.GetLastError())
	{
		ptCanvas.LoadObjects(objFile.c_str());
	}
	
	//Read segmentation.
	kCmd.String("seg",segFile);
	if(!kCmd.GetLastError())
	{
		ptCanvas.LoadSegmentation(segFile.c_str());
	}
	
	//Read normals.
	kCmd.String("normals",normalsFile);
	if(!kCmd.GetLastError())
	{
		ptCanvas.LoadNormals(normalsFile.c_str());
	}
	
	
	delete(splash);
	
	//Done with command line now show the browser.
	ptCanvas.showMaximized();
	
	return app.exec();
}
