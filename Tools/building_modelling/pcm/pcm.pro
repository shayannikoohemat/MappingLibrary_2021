
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



# Qmake Project file
# George Vosselman, modifications for port to Qt4 without Qt3 support

CONFIG += qt thread
QT += opengl widgets

LIBS +=  -lBuildings -lLaserScan -llas -lPhotogrammetry -lEispack -lLinpack -lgfortran -lGL -lGLU -lglut -L $(MAPPING_LIB_DIR)
LIBS += -lnewmat -lANN
MOC_DIR = ./moc/
OBJECTS_DIR = ./obj/
RESOURCES = ./button_icons.qrc
TEMPLATE = app
DEPENDPATH += . include src
INCLUDEPATH += . include  ../../../Library/LaserScan/include ../../../Library/Photogrammetry/include ../../../Library/Buildings/include ../../../Foreign/ANN/include/ANN
QMAKE_LFLAGS = -no-pie
#if on windows define windows. used by Libraries internally
win32 {
	DEFINES += windows
}

# Input
HEADERS += include/DataAppearance.h \
           include/DataAppearWin.h \
           include/ExtensionLine.h \
           include/FilteringParWin.h \
           include/FittingParWin.h \
	   include/glfont.h \
	   include/LaserDataIOWin.h \
	   include/OutliningParWin.h \
	   include/PCMCanvas.h \
	   include/PCMWindow.h \
           include/PointCloudMapper.h \
           include/PointInformationWin.h \
           include/PyramidBrowseThread.h \
           include/QGLCanvas.h \
	   include/SegmentParWin.h \
           include/SelectionBox.h \
           include/SelectWin.h \
           include/SplitLine.h \
           include/TileInformation.h

SOURCES += src/BackGround.cc \
           src/DataAppearance.cc \
           src/DataAppearWin.cc \
           src/EditLine.cc \
           src/ExtensionLine.cc \
           src/FilteringParWin.cc \
           src/FittingParWin.cc \
	   src/glfont.cc \
	   src/LaserDataIOWin.cc \
           src/main.cc \
	   src/OutliningParWin.cc \
           src/PCMCanvas.cc \
           src/PCMData.cc \
           src/PCMWindow.cc \
           src/PointCloudMapper.cc \
           src/PointInformationWin.cc \
           src/PyramidBrowseThread.cc \
           src/QGLCanvas.cc \
           src/SegmentLaserData.cc \
	   src/SegmentParWin.cc \
           src/SelectionBox.cc \
           src/SelectWin.cc \
           src/SplitLine.cc \
	   src/TileInformation.cc
