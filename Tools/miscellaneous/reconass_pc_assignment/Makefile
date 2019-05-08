#############################################################################
# Makefile for building: laservoxel_lx
#############################################################################

####### Compiler, tools and options

CC            = gcc
CXX           = g++
DEFINES       = -DSTANDALONE
CFLAGS        = -pipe -g -Wall -W -D_REENTRANT $(DEFINES)
CXXFLAGS      = -pipe -g -Wno-deprecated -Wall -W -D_REENTRANT $(DEFINES)
INCPATH       = -I. -Iinclude  -I../../../Library/LaserScan/include -I../../../Library/Photogrammetry/include -I../../../Library/Buildings/include -I../../../Foreign/ENLSIP -I../../../Foreign/newmat -I../../../Foreign/ANN/include/ANN 
LINK          = g++
LFLAGS        = -Wl
LIBS          = -L$(MAPPING_LIB_DIR) -lBuildings -lLaserScan -lPhotogrammetry -lEispack -lLinpack -lnewmat -llas -lANN -lENLSIP -lg2c 
RM = rm -f
####### Output directory

OBJECTS_DIR   = obj/

####### Files
SOURCES       = main.cpp
OBJECTS       = main.o
TARGET        = ../../bin/reconass_pc_assignment


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

