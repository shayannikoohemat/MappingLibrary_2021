#include <cstdlib>
#include <iostream>
#include <fstream>
#include "LaserPoints.h"
#include "LaserPoint.h"
#include "LaserBlock.h" 
#include "LaserUnit.h"
#include "ObjectPoints.h"
#include "ObjectPoint.h"
#include "LineTopologies.h"
#include "Positions3D.h"
#include "Planes.h"
#include <stdio.h>
#include <windows.h>
#include <math.h>
#include "FilteringParameters.h"
#include "LaserPoints.h"

//type of classes
enum ClassType { WaterSurfaceClass=1,GroundClass,BuildingRoofClass,VegetationClass,VehicleClass,
                 LowVegetationClass,AboveBuildingClass,BuildingWallClass};
typedef enum ClassType ClassType;

//parameters for segmentation--in order to get oversegmentation on vegetation
static SegmentationParameters segmentation_par; 
//parameters for feature threshold
//preclassification thresholds
static float dist_to_DTM=3.0;
static float point_space_roof=0.35;
static float point_space=0.3;// for building point_space+0.05
static float plane_res=2.0;
static int size_plane=200;
static int size_wall=1000;
static int size_low_veg=150;
static int size_vehicle=100;
//for 2008 the parameters are size200,1000,100 dist_edge 0.3 & dist_edge_roof 0.35 residual 2 
//for AHN2 the parameters are size100, 500,50 dist_edeg 0.45&dist_edge_roof 0.55 residual 3   

//correction thresholds
static float v_z=0.2;  //for z_variance
static float v_x=1.50;  //for slope_variance
static float v_y=0.065;  //for point_space_sigma
static float per_xyz=55.00;  //for percentage on the walls
static float plc_per_seg=0.60;  //for pulse count per segment
//for 2008 v_zTag 0.25;v_xTag1.00;v_yTag0.065;per_xyz0.65
//for AHN2 v_zTag0.25;v_xTag1.00;v_yTag0.06;per_xyz=0.7
    
//for calculating features
static float dist_threshold=1.0; 
//for 2008 d<1.0
//for AHN2 d<1.5  
static float wg_dist_threshold=0.45; //0.55 for AHN2, 0.35 for 2008

//for search DTM Points
static float radius=20;
static float size=20;
 
double DisToDTM(PointNumberList &, LaserPoints &, LaserPoints &, TINEdges &, int);
double PResidual(LaserPoints &, PointNumberList & ,Plane &);
double Residual(LaserPoints &, PointNumberList & ,double);
LaserPoints PointAttributes(LaserPoints &);
LaserPoints CheckContext(LaserPoints &);
LaserPoints Correct(LaserPoints &);
double AverageEdgeLength(LaserPoints &);
double AverageEdgeSigma(LaserPoints &,double);
LaserPoints WaterClass(LaserPoints &);
double NonFirstPulsePntRatio(LaserPoints &, PointNumberList &);
double CheckSlope(LaserPoints &,PointNumberList &,LaserPoints::iterator);
double Rank(LaserPoints &,PointNumberList &, int );//1=max height 2=min height 3=average height
LaserPoints Classify(LaserPoints &,LaserPoints &, fstream &);

double Percentage(LaserPoints &, PointNumberList &);
int ConnComponent(LaserPoints &, PointNumberList &, TINEdges *);
 

        

 
