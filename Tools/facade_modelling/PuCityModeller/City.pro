
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
# Stephan Heuel, 04.07.2005
# George Vosselman, modifications for port to Qt4 without Qt3 support

CONFIG += qt
QT += opengl widgets xml
QTPLUGIN += qjpeg

LIBS +=  -lBuildings -lLaserScan -lPhotogrammetry -lEispack -lLinpack -lANN -lnewmat -lgfortran  -L $(MAPPING_LIB_DIR)
LIBS += -L ..\..\..\Foreign\OpenCV\OpenCV-2.4.4\lib -lopencv_highgui244 -lopencv_core244 -lopencv_imgproc244
MOC_DIR = ./moc/
RC_FILE=./City_private.rc
OBJECTS_DIR = ./obj/
RESOURCES = ./button_icons.qrc
TEMPLATE = app
DEPENDPATH += . include src
INCLUDEPATH += . include  ../../../Library/LaserScan/include ../../../Library/Photogrammetry/include ../../../Library/Buildings/include ../../../Foreign/ANN/include/ANN
INCLUDEPATH += -I"../../../Foreign/OpenCV/OpenCV-2.4.4/include/opencv" -I "..\..\..\Foreign\OpenCV\OpenCV-2.4.4\modules\core\include" -I "..\..\..\Foreign\OpenCV\OpenCV-2.4.4\modules\imgproc\include" -I "..\..\..\Foreign\OpenCV\OpenCV-2.4.4\modules\video\include" -I "..\..\..\Foreign\OpenCV\OpenCV-2.4.4\modules\features2d\include" -I "..\..\..\Foreign\OpenCV\OpenCV-2.4.4\modules\flann\include" -I "..\..\..\Foreign\OpenCV\OpenCV-2.4.4\modules\calib3d\include" -I "..\..\..\Foreign\OpenCV\OpenCV-2.4.4\modules\objdetect\include" -I "..\..\..\Foreign\OpenCV\OpenCV-2.4.4\modules\legacy\include" -I "..\..\..\Foreign\OpenCV\OpenCV-2.4.4\modules\highgui\include"
TRANSLATIONS=spreadsheet_cn.ts
#if on windows define windows. used by Libraries internally
win32 {
	DEFINES += WIN32
}

# Input
HEADERS += include/DataAppearance.h \
           include/DataAppearWin.h \
           include/ExtensionLine.h \
           include/FittingParWin.h \
           include/cityCanvas.h \
           include/cityWindow.h \
           include/City.h \
           include/QGLCanvas.h \
	   include/SegmentParWin.h \
           include/SelectionBox.h \
           include/SplitLine.h \
           include/TerrestrialParWin.h
#          include/imageviewer.h

SOURCES += src/BackGround.cc \
           src/DataAppearance.cc \
           src/DataAppearWin.cc \
           src/EditLine.cc \
           src/ExtensionLine.cc \
           src/FittingParWin.cc \
           src/main.cc \
           src/cityCanvas.cc \
           src/cityData.cc \
           src/cityWindow.cc \
           src/City.cc \
           src/QGLCanvas.cc \
           src/SegmentLaserData.cc \
	   src/SegmentParWin.cc \
           src/SelectionBox.cc \
           src/SplitLine.cc \
           src/Terrestrial.cc \
           src/Convexhull.cc \
             src/Fill.cc \
#            src/RoofExtrusion.cc \
             src/Outline.cc \
             src/Preprocessing.cc \
           src/TerrestrialParWin.cc
#          src/Image.cc \
#          src/imageviewer.cc \
#          src/Freeform.cc
