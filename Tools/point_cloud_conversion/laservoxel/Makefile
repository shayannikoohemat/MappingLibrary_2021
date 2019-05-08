
#
#    Copyright 2010 University of Twente and Delft University of Technology
# 
#       This file is part of the Mapping libraries and tools, developed
#  for research, education and projects in photogrammetry and laser scanning.
#
#  The Mapping libraries and tools are free software: you can redistribute it
#    and/or modify it under the terms of the GNU General Public License as
#  published by the Free Software Foundation, either version 3 of the License,
#                   or (at your option) any later version.
#
# The Mapping libraries and tools are distributed in the hope that it will be
#    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
#        MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#                GNU General Public License for more details.
#
#      You should have received a copy of the GNU General Public License
#          along with the Mapping libraries and tools.  If not, see
#                      <http://www.gnu.org/licenses/>.
#
#----------------------------------------------------------------------------


#############################################################################
# Makefile for building: laservoxel_lx
#############################################################################

####### Compiler, tools and options

CC            = gcc
CXX           = g++

INCPATH       = -I. -Iinclude -I/home/gerke/opencv2/include/opencv -I../../../Library/LaserScan/include -I../../../Library/Photogrammetry/include -I../../../Library/Buildings/include -I../../../Foreign/ENLSIP -I../../../Foreign/newmat -I../../../Foreign/ANN/include/ANN -I../../../Foreign/NR/utils -I/usr/local/gcc342/include -I/usr/local/include 
LINK          = g++
LFLAGS        = -Wl
LIBS          = -L/home/gerke/Mapping/Library/lib -lBuildings_lx -lLaserScan_lx -lPhotogrammetry_lx -L/home/gerke/opencv2/lib -lhighgui -lcv -lcxcore -lcvaux -lNR_lx -lEispack_lx -lLinpack_lx -lnewmat_lx -lANN_lx -lENLSIP_lx -lg2c 
RM = rm -f
####### Output directory

OBJECTS_DIR   = obj/

####### Files

SOURCES       = src/laservoxel.cpp \
		src/main.cpp 
OBJECTS       = obj/laservoxel.o \
		obj/main.o
TARGET        = ../../bin/laservoxel


first: all
####### Implicit rules
.PHONY: all all-before all-after clean clean-custom

.SUFFIXES: .c .o .cpp .cc .cxx .C

.cpp.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.cc.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.cxx.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.C.o:
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o $@ $<

.c.o:
	$(CC) -c $(CFLAGS) $(INCPATH) -o $@ $<

####### Build rules

all: Makefile $(TARGET)

clean: clean-custom
	${RM} $(OBJECTS) $(TARGET)

$(TARGET):  $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)

####### Compile

obj/laservoxel.o: src/laservoxel.cpp include/laservoxel.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o obj/laservoxel.o src/laservoxel.cpp

obj/main.o: src/main.cpp include/laservoxel.h
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o obj/main.o src/main.cpp

####### Install

install:   FORCE

uninstall:   FORCE

FORCE:

