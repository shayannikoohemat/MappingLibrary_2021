### Ubuntu Installation
`(added by Shayan Ni. 2019-20)`

Notes for installing MappingLibrary tested on `linux 16.04` with current gcc (5.4.0)

#### Install dependencies using package manager
```
sudo apt-get install qtbase5-dev
sudo apt-get install qtcreator 
sudo apt-get install gfortran 
sudo apt-get install libopencv-dev              for imageprocessing tools 
sudo apt-get install freeglut3-dev              for openGl  
sudo apt-get install mesa-utils                 for openGl
sudo apt-get install libboost-filesystem-dev    for shayan's tools
sudo apt-get install libboost-regex-dev         for shayan's tools
```  

#### git checkout the repository 
`git clone https://yourusername@bitbucket.org/kaiosai/ITCMappingLibrary.git /MAPPING_LIBRARY_SOURCE_DIR`  

#### Install 3rd party libraries   
Foreign libraries are 3rd party libraries which can be installed from their main repository and 
 linked to the main library in the cmake but we recommend first try the static versions which are 
available through a zip file as the versions are compatible.

#### Get Foreign folder
* Download tar file from SURF drive: [Foreign libs](https://surfdrive.surf.nl/files/index.php/s/D6dOBHRlTtWmei9).
* Unpack tar to `MAPPING_LIBRARY_SOURCE_DIR`  
**Result:** new folder `<MAPPING_LIBRARY_SOURCE_DIR>/Foreign`
 
#### Create and expose library binaries directory
Create directory:

```
mkdir <MAPPING_LIBRARY_SOURCE_DIR>/Library/lib64_linux
```
Expose directory:

```
export MAPPING_LIB_DIR=<MAPPING_LIBRARY_SOURCE_DIR>/Library/lib64_linux
```

#### Alternative: Foreign binaries
Alternatively, you can download `lib64_linux binaries` for Foreign libraries from
 [Foreign binaries](https://surfdrive.surf.nl/files/index.php/s/6zBKgBp9Q6izpBb)
 These binaries are tested on `ubuntu 16.04 TLS`
 
 
#### Build Foreign
For the main library, you need at least: `libEispack, libLinpack, libnewmat, libANN, liblas`   
**```Note: All of the Foreign libs should be present in lib64_linux.```**

If not yet done, in `makefiles`, probably just for `Makefile_Photogrammetry` and `Laserscan` 
`Eispack` and `Linpack` replace **g77** by **gfortran**, and remove `.exe` extension from `gcc.exe` and `g++.exe`
you can use Makefile.linux in the root of Foreign but you need to do slight modification in the names
 and comment `NR` to make it work.  


 ```
 cd Foreign
 make -f Makefile.linux
 ```

**build ANN**  
ANN has to be build seperately.You can do this in 2 different ways:  
1. install `libann-dev` by `sudo apt-get install libann-dev` and change `-lANN` to `-lann` in `pcm.pro`  
**OR**  
2. in `makefile.linux` comment:  
`	#cd test ; $(MAKE) $@`  
`	#cd sample ; $(MAKE) $@`  
add `include <cstring>` and `include <stdlib.h>` to `aan2fig.cpp`  
make new folders `lib` and `bin`  in `ANN root`, then run: 

 ```
 cd ANN
 make -f Makefile_linux linux-g++
 cp lib/libANN.a $MAPPING_LIB_DIR
 ```

**build lastools**  
lastools has to be build separately, becasue some of them are commercial, e.g. lasground.
Build lastools for linux from their github: [LAStools](https://github.com/LAStools/LAStools).   
Manually copy `liblas.a` from `LASlib/src` to `lib64_linux`

```
cd lastools
make
cp LASlib/src/liblas.a $MAPPING_LIB_DIR
```
**Result:** The following library binaries are in the folder `MAPPING_LIB_DIR`
* libANN.a
* libEispack.a
* libENLSIP.a? SKIPPED
* liblas.a
* libLinpack.a
* libnewmat.a

>**TODO** 
> including [PDAL](https://pdal.io/), [PCL](https://github.com/PointCloudLibrary/pcl/wiki)
> and [CGAL](https://www.cgal.org/) as 3rd party libraries.  
>TODO: ENLSIP skipped.


#### Building mapping library

If not yet done, in `makefiles` replace **g77** by **gfortran**, and remove `.exe` extension from `gcc.exe` and `g++.exe`  
run `makefiles.linux` in Library root
`$make -f makefile.linux`  

```
cd Library
```
Build Photogrammetry
```
cd Photogrammetry
mkdir obj
make -f Makefile_Photogrammetry.linux 
```
Build LaserScan
```
cd LaserScan
mkdir obj
make -f Makefile_LaserScan.linux 
```
Build Buildings
```
cd Buildings
mkdir obj
make -f Makefile_Buildings.linux 
```

**Result**:  
After compiling foreign and Library in lib64_linux you should have:  
```
libEispacka, liblas.a, libLinpack.a, libnewmat.a libBuildings.a, libphotogrammetry.a, libLaserScan.a
```


#### Build PCM with Qt Creator

PCM or PointCloudMapper is the interface for mapping library.

Folder: `Tools/building_modelling/pcm` 

1. Open project file (`pcm.pro`) with Qt Creator
2. Select kit (For example: *Desktop Qt 5.15.0 GCC*)
3. Go to `Projects > Build > Build Environment`
4. Add entry:
`MAPPING_LIB_DIR = <MAPPING_LIBRARY_SOURCE_DIR>/Library/lib64_linux`
5. Build target 'pcm' 
6. Run target 'pcm'  

Further notes for compiling pcm:
```
- Alternatively, use pcm.pro file in the qtcreator to compile
- Add LIBS = -lGL -lGLU -lglut
- comment this line in pcm/main.cc: `//QApplication::setAttribute(Qt::AA_EnableHighDpiScaling);`
```

#### Troubleshooting

During compiling Library fix below errors:

For problems with `MAXFLOAT`  
 remove the `#ifdefs` which differentiate between Windows and Linux 
	-> in `Image.cc`, …  
and change `MAXINT` to `MAX_INPUT`
	
For problems with `memcpy` (and similar):  
 add `#include <cstring>`
	-> in `line3d.cc, linetopology.cc, orientation3d.cc, rotation3d.cc, rotation2d.cc, vector3d.cc, objectpoints.cc`

in `components.cc` if error: `In member function ‘void LaserPoints::RemoveSmallSegments(LaserPointTag, int)`
then change `segment_number_counts.push_back(count_array);` to `segment_number_counts.push_back(&count_array)`; --> NOTE the & sign in &count_array
similarly in other functions.

For Ubuntu 14.04, some implicit type conversions have to be made explicit
	-> in `objectpoints.cc Fprintfs and printfs `
	
#### GCC PIE Troubleshooting (by Martijn)
`On Linux (Ubuntu 18.04 and 20.04) tested with gcc (7.5.0 and 9.3.0)`  
When the following error is encountered:

```
XXX.a(YYY.o): relocation R_X86_64_32 against symbol `ZZZ' can not be used when making a PIE object; recompile with -fPIE
```
GCC has PIE enabled. You can check that with `gcc -v`. This will show something like `--enable-default-pie`. Disabling this globaly means recompiling GCC.

Alternatively, provide the linker flag `-no-pie` when linking to the libraries.

In QMake PRO file:
```
QMAKE_LFLAGS = -no-pie
```


In CMake CMakeLists.txt file:
```
target_link_options(<insert target name> PRIVATE -no-pie)
```

#### Directory tree structure

```
├── Foreign
│   ├── ANN
│   ├── citygml
│   ├── eigen
│   ├── Eispack
│   ├── ENLSIP
│   ├── f2c
│   ├── lastools
│   ├── Linpack
│   ├── Makefile.linux
│   ├── Makefile.win
│   ├── newmat
│   └── OpenCV (optional)
├── Library
│   ├── Buildings
│   ├── Documentation
│   ├── LaserScan
│   ├── lib64_linux
│   ├── Makefile.linux
│   ├── Makefile.win
│   ├── Photogrammetry
│   └── Roads
├── Makefile.win
├── move_binaries.txt
├── README.md
└── Tools
    ├── bin
    ├── building_modelling
    ├── ...

```
