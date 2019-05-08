
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

CONFIG += qt thread
QT += opengl widgets

HEADERS       = imageviewer.h
SOURCES       = imageviewer.cpp \
                main.cpp

INCLUDEPATH += C:/Qt/5.0.1/qtbase/include/QtPrintSupport

LIBS += -lQt5PrintSupport

# install
target.path = $$[QT_INSTALL_EXAMPLES]/widgets/imageviewer
sources.files = $$SOURCES $$HEADERS $$RESOURCES $$FORMS imageviewer.pro
sources.path = $$[QT_INSTALL_EXAMPLES]/widgets/imageviewer
INSTALLS += target sources
