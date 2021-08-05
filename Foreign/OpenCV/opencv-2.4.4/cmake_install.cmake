# Install script for directory: D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4

# Set the install prefix
IF(NOT DEFINED CMAKE_INSTALL_PREFIX)
  SET(CMAKE_INSTALL_PREFIX "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/install")
ENDIF(NOT DEFINED CMAKE_INSTALL_PREFIX)
STRING(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
IF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  IF(BUILD_TYPE)
    STRING(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  ELSE(BUILD_TYPE)
    SET(CMAKE_INSTALL_CONFIG_NAME "Release")
  ENDIF(BUILD_TYPE)
  MESSAGE(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
ENDIF(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)

# Set the component getting installed.
IF(NOT CMAKE_INSTALL_COMPONENT)
  IF(COMPONENT)
    MESSAGE(STATUS "Install component: \"${COMPONENT}\"")
    SET(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  ELSE(COMPONENT)
    SET(CMAKE_INSTALL_COMPONENT)
  ENDIF(COMPONENT)
ENDIF(NOT CMAKE_INSTALL_COMPONENT)

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")
  FILE(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/opencv2" TYPE FILE FILES "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/opencv2/opencv_modules.hpp")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "main")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/install/OpenCVConfig.cmake")
  IF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
  IF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
FILE(INSTALL DESTINATION "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/install" TYPE FILE FILES "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/win-install/OpenCVConfig.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/install/OpenCVConfig-version.cmake")
  IF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
  IF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  ENDIF (CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
FILE(INSTALL DESTINATION "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/install" TYPE FILE FILES "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/win-install/OpenCVConfig-version.cmake")
ENDIF(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")

IF(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  INCLUDE("D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/zlib/cmake_install.cmake")
  INCLUDE("D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/libtiff/cmake_install.cmake")
  INCLUDE("D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/libjpeg/cmake_install.cmake")
  INCLUDE("D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/libjasper/cmake_install.cmake")
  INCLUDE("D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/libpng/cmake_install.cmake")
  INCLUDE("D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/3rdparty/openexr/cmake_install.cmake")
  INCLUDE("D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/include/cmake_install.cmake")
  INCLUDE("D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/modules/cmake_install.cmake")
  INCLUDE("D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/doc/cmake_install.cmake")
  INCLUDE("D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/data/cmake_install.cmake")
  INCLUDE("D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/apps/cmake_install.cmake")

ENDIF(NOT CMAKE_INSTALL_LOCAL_ONLY)

IF(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
ELSE(CMAKE_INSTALL_COMPONENT)
  SET(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
ENDIF(CMAKE_INSTALL_COMPONENT)

FILE(WRITE "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/${CMAKE_INSTALL_MANIFEST}" "")
FOREACH(file ${CMAKE_INSTALL_MANIFEST_FILES})
  FILE(APPEND "D:/Software/Mapping/Foreign/OpenCV/opencv-2.4.4/${CMAKE_INSTALL_MANIFEST}" "${file}\n")
ENDFOREACH(file)
