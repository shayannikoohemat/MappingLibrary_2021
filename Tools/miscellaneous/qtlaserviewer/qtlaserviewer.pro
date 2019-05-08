
# ***************************************************************************
# *   copyright            : (C) 2010 by University of Twente               *
# ***************************************************************************
#
# ***************************************************************************
# *                                                                         *
# *   This program is free software; you can redistribute it and/or modify  *
# *   it under the terms of the GNU General Public License as published by  *
# *   the Free Software Foundation; either version 2 of the License, or     *
# *   (at your option) any later version.                                   *
# *                                                                         *
# ***************************************************************************

QT           += opengl 

MOC_DIR = ./moc/
OBJECTS_DIR = ./obj/

TEMPLATE = app
CONFIG += warn_off release

#if on windows define windows. used by Libraries internally
win32 {
	DEFINES += windows
}

LIBS +=  -L $(MAPPING_LIB_DIR) -lLaserScan -lPhotogrammetry -lEispack -lLinpack -lgfortran -lnewmat -lANN -lENLSIP -lm

DEPENDPATH += . include src

INCLUDEPATH += . include  ../../../Library/LaserScan/include ../../../Library/Photogrammetry/include ../../../Library/Buildings/include
INCLUDEPATH += ../../../Foreign/newmat ../../../Foreign/ENLSIP
INCLUDEPATH += ../../../Foreign/ANN/include/ANN
#INCLUDEPATH += ./Utils ./OpenGL ./Processing ./Laser

#if on windows define windows. used by Libraries internally
win32 {
	DEFINES += windows
}

QMAKE_CXXFLAGS += -Duse_namespace -fpermissive
#*******************************************************************************
RESOURCES     = resources.qrc

HEADERS       +=  QGLGeneralCanvas.h   QGLPointsCanvas.h  QGaussianSphereViewer.h QSegmentationBrowser.h    \
					QRegistrationICP.h QRegistrationIndirect.h         

SOURCES  +=  main.cpp QGLGeneralCanvas.cc QGLPointsCanvas.cc \
			QGaussianSphereViewer.cc  QSegmentationBrowser.cc 
SOURCES +=  QRegistrationICP.cc QRegistrationIndirect.cc \
			LaserScan.cc  QtUtility.cc 
SOURCES +=  ENLSIPRegistrationIndirect.cc  ENLSIPRegistrationDirect.cc

#FORMS += 


