# Qmake Project file

QMAKE_CXXFLAGS_DEBUG += -Wno-deprecated -fpermissive
QMAKE_CXXFLAGS_RELEASE += -fpermissive

LIBS += -L $(OPENCV_LIB_DIR) -L $(MAPPING_LIB_DIR)
LIBS += -lBuildings\
-lLaserScan\
-lPhotogrammetry\
-llinex\
-lopencv_highgui244\
-lopencv_core244\
-lopencv_imgproc244\
-lEispack\
-lLinpack\
-lnewmat\
-lANN\
-lENLSIP\
-lgfortran\
-lm

     
release:DESTDIR = ../../bin/
MOC_DIR = ./moc/
OBJECTS_DIR = ./obj/
DEPENDPATH += . include src 
INCLUDEPATH += . include\  
              ../../../Library/LaserScan/include\
              ../../../Library/Photogrammetry/include\              
              ../../../Library/Buildings/include\
	      ../../../Foreign/ENLSIP\
              ../../../Foreign/newmat\
              ../../../Foreign/ANN/include/ANN\
	      ../image_processing/linex\
	      ../../../Foreign/OpenCV/OpenCV-2.4.4/include/opencv\
	      ../../../Foreign/OpenCV/OpenCV-2.4.4/modules/core/include\
	      ../../../Foreign/OpenCV/OpenCV-2.4.4/modules/imgproc/include\
	      ../../../Foreign/OpenCV/OpenCV-2.4.4/modules/ml/include\
	      ../../../Foreign/OpenCV/OpenCV-2.4.4/modules/contrib/include\
	      ../../../Foreign/OpenCV/OpenCV-2.4.4/modules/video/include\
	      ../../../Foreign/OpenCV/OpenCV-2.4.4/modules/features2d/include\
	      ../../../Foreign/OpenCV/OpenCV-2.4.4/modules/flann/include\
	      ../../../Foreign/OpenCV/OpenCV-2.4.4/modules/calib3d/include\
	      ../../../Foreign/OpenCV/OpenCV-2.4.4/modules/objdetect/include\
	      ../../../Foreign/OpenCV/OpenCV-2.4.4/modules/legacy/include\
	      ../../../Foreign/OpenCV/OpenCV-2.4.4/modules/highgui/include

win32 {
	DEFINES += windows
	CONFIG += console
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
	  
           
	   

