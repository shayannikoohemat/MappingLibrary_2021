# Qmake Project file

QMAKE_CXXFLAGS_DEBUG += -Wno-deprecated

LIBS += -L/home/gerke/Mapping/Library/lib\
-lBuildings_lx\
-lLaserScan_lx\
-lPhotogrammetry_lx\
-llinex_lx\
-L/home/gerke/opencv2/lib -lhighgui -lcv -lcxcore -lcvaux \
-llapack_LINUX\
-lblas_LINUX\
-lF77_lx\
-lNR_lx\
-lEispack_lx\
-lLinpack_lx\
-lnewmat_lx\
-lANN_lx\
-lENLSIP_lx\
-lg2c\
-lm\
-ldl

     
MOC_DIR = ./moc/
OBJECTS_DIR = ./obj/
DEPENDPATH += . include src 
INCLUDEPATH += . include\  
		/home/gerke/opencv2/include/opencv\	
               /home/gerke/Mapping/Library/LaserScan/include\
               /home/gerke/Mapping/Library/Photogrammetry/include\
               /home/gerke/Mapping/Library/Buildings/include\
	       /home/gerke/Mapping/Foreign/ENLSIP\
               /home/gerke/Mapping/Foreign/newmat\
               /home/gerke/Mapping/Foreign/ANN/include/ANN\
              /home/gerke/Mapping/Foreign/NR/utils\
	       /home/gerke/Mapping/Tools/digphot/linex\
              /usr/local/gcc342/include\
	      /usr/local/include

win32 {
	DEFINES += windows
}
  
DEFINES +=STANDALONE

HEADERS += include/MathTools_TUB.h\
	   include/EdgeTools.h\
           include/MatrixTools.h\
	   include/defs_TUB.h\
           include/const_TUB.h\
           include/types_TUB.h\
           include/Tools.h\
	   include/ObjectSpaceLine.h\
	   include/ObjectSpaceLines.h

SOURCES += src/MathTools_TUB.cpp\
	   src/main.cpp\
	   src/EdgeTools.cpp\
           src/MatrixTools.cpp\
           src/Tools.cpp\
	   src/ObjectSpaceLine.cpp\
	   src/ObjectSpaceLines.cpp
	  
           
	   

