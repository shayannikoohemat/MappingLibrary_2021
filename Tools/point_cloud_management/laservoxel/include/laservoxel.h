#ifndef LaserVoxel_H
#define LaserVoxel_H

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "LaserPoints.h"
#include "DataBoundsLaser.h"
#include <vector>
#include <map>

#include "cv.h"
#include "highgui.h"
#include "KNNFinder.h"

#ifndef uint
typedef unsigned int uint;
#endif


class LaserVoxel
{
  
  private:
    vector< vector < vector < LaserPoints* > > > Vox;
    ///edge length of a voxel
    double vox_length; 
    ///number of voxels in XYZ
    uint vox_num_X, vox_num_Y, vox_num_Z;
    ///min value in real X Y Z (bounding box)
    double min_X, min_Y, min_Z;
    ///given a real object coordinate the indices for the Voxel field are computed
    uint index_from_real(double real, double min) {return (uint) floor((real-min)/vox_length);}
    uint index_from_realX(double real) {return (index_from_real(real,min_X));}
    uint index_from_realY(double real) {return (index_from_real(real,min_Y));}
    uint index_from_realZ(double real) {return (index_from_real(real,min_Z));}
    
    IplImage* plane_segment_to_image();
    void image_to_plane_segment(IplImage* img);
    bool fill_gaps_for_coplanar_points(LaserPoints &ls,double kernelsize_objectspace, LaserPoints *FillUpDataSegment=NULL,bool project_all_to_plane=0);
    
 public:
   //LaserVoxel(void);
   LaserVoxel(LaserPoints &ls,double vox_l,bool verbose=1);
   LaserVoxel(double vox_l);
   ~LaserVoxel();
  
   void statistics();
   
   vector<LaserPoints> LaserPoints_per_voxel(); //contains the physical (not Pointer) points in a voxel. Regardless of the voxel's spatial position. needed in 3D scene interpretation, subdivision of points
   ///Filter: min. number of points in a voxel
   void filter_size(uint min=2,bool verbose=1);
   ///Filter according to neighb.hood: in the 26-neighb.hood of a voxel a min. number of points is required, otherwise it is deleted
   void filter_neighbour(uint min=1,bool verbose=1);
   int number_of_points_in_voxel_at(double X, double Y, double Z, LaserPoints *PointsInVoxel=NULL);
   LaserPoints export_all();
   void export_all_ply(char * filename,LaserPoints l);
   LaserPoints export_vox_centres(bool also_voids=false); ///only exports the centres of voxels (default: where laserpoints are)
   void export_vox_centres_text(char * filename);
   void export_vox_latice(ObjectPoints &objpts,LineTopologies &linetops);
   
   void fill_gaps(LaserPoints &ls,double kernelsize_objectspace, LaserPoints *filledL, LaserPoints *FillUpData=NULL, bool project_all_to_plane=0); //FillUpData: Optional laserdata of a better density (and noise?) to fill gaps
   
};

//Description of approach to gap filling

//given: Segmented laserpoints: all points of a segment are supposed to represent a plane in 3D (eg. computed through segmentation in PCM)
//each segement is then treated individually:
 
  //1) put them into a local voxelfield
  //2) compute the plane normal (only from voxelcentres)
  //3) use LaserPoints::ProjectToPlane to project the voxelcentres to a plane
  //4) those points on the plane: put into another voxel (actually it is than a raster since dz=0)
  //5) create an IPLimage from that raster
  //6) do some image processing: closing using openCV 
  //7) convert this image back to the voxel/raster field from step 4)
  //8) after that we need to find those points which have been added: for all voxels in the initial voxelfield (step 1) (ALSO EMPTY ONES):
    //check, if the voxel which is on the projection to that plane is empty or not. Change the actual voxel accordingly.
    //-> this has the advantage that it is quite flexible, so if (may be in another image processing) some points are removed, they will also be removed from the voxelfield
  //if desired  (switch -gp) only the points as projected to the estimated plane are returned.
  //9) the LaserPoints instance is then composed out of the final voxel centres

#endif
