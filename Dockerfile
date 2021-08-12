FROM ubuntu:18.04

ENV DEBIAN_FRONTEND="noninteractive"

RUN apt-get update &&\
    apt-get install -y apt-transport-https ca-certificates gnupg software-properties-common wget &&\
    wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null &&\
    apt-add-repository 'deb https://apt.kitware.com/ubuntu/ bionic main'

RUN apt-get update && apt-get install -y \
        gfortran \
        libopencv-dev \
        libboost-filesystem-dev \
        libboost-regex-dev \
        build-essential \
        mesa-utils \
        freeglut3-dev \
        cmake

WORKDIR /opt

COPY . .

RUN mkdir -p ./build/Library/lib64_linux 

ENV MAPPING_LIB_DIR /opt/build/Library/lib64_linux
ENV MAPPING_INCLUDE_DIR /opt/Library/

WORKDIR /opt/Foreign

RUN make -f Makefile.linux

RUN apt-get install -y libann-dev

WORKDIR /opt/Library/Photogrammetry

RUN mkdir obj &&\
    make -f Makefile_Photogrammetry.linux

WORKDIR /opt/Library/LaserScan

RUN mkdir obj &&\
    make -f Makefile_LaserScan.linux 

WORKDIR /opt/Library/Buildings

RUN mkdir obj &&\
    make -f Makefile_Buildings.linux 

WORKDIR /opt/Tools/road_assets/pole_like_detection

RUN cp /opt/Foreign/LAStools/LASlib/lib/liblas.a /opt/build/Library/lib64_linux &&\
    mkdir build && cd build && cmake .. && make

