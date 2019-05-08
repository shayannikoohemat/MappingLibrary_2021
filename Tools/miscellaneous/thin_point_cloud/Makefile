#############################################################################
# Makefile for building: laservoxel_lx
#############################################################################

####### Compiler, tools and options

CC            = gcc
CXX           = g++
DEFINES       = -DSTANDALONE
CFLAGS        = -pipe -g -Wall -W -D_REENTRANT $(DEFINES)
CXXFLAGS      = -pipe -g -Wno-deprecated -Wall -W -D_REENTRANT $(DEFINES)
INCPATH       = -I. -I$(OPENCVROOT)/include/opencv -I../../../Library/LaserScan/include -I../../../Library/Photogrammetry/include -I../../../Library/Buildings/include -I../../../Foreign/ENLSIP -I../../../Foreign/newmat -I../../../Foreign/ANN/include/ANN -I../../../Foreign/NR/utils -I/usr/local/gcc342/include -I/usr/local/include -I../../altimetry/laservoxel/include
LINK          = g++
LFLAGS        = -Wl
LIBS          = -L$(OPENCV_LIB_DIR) -lopencv_highgui -lopencv_core -lopencv_imgproc -lopencv_legacy -L$(MAPPING_LIB_DIR) -lBuildings -lLaserScan -lPhotogrammetry -lEispack -lLinpack -lnewmat -llas -lANN -lENLSIP -lg2c 
RM = rm -f
####### Output directory

OBJECTS_DIR   = obj/

####### Files
SOURCES       = main.cpp
OBJECTS       = main.o
TARGET        = ../../bin/thin_point_cloud


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
#clean: 
#	$(RM) $(OBJECTS) $(TARGET) *~ core *.core
all: Makefile $(TARGET)

$(TARGET):  $(OBJECTS)  
	$(LINK) $(LFLAGS) -o $(TARGET) $(OBJECTS) $(OBJCOMP) $(LIBS)

####### Compile

obj/main.o: main.cpp 
	$(CXX) -c $(CXXFLAGS) $(INCPATH) -o main.o main.cpp

####### Install

install:   FORCE

uninstall:   FORCE

FORCE:

