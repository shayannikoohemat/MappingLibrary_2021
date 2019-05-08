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

#ifndef uint
typedef unsigned int uint;
#endif


class LaserVoxel
{
  
  private:
    LaserPoints* original_points;
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
    
    uint index_for_gco(uint &i,uint &j,uint &k,uint &l,uint &num_labels); //given the index per X Y Z and label it returns the index in the onedimensionl vector data[num_voxels*num_labels]. Makes it easier to find pos. in the 3D lattice
    void ijkl_from_index_for_gco(uint &index, uint &i ,uint &j,uint &k,uint &l,uint &num_labels); //reverse fct from the above one
    
 public:
   //LaserVoxel(void);
   LaserVoxel(LaserPoints &ls,double vox_l,bool verbose=1,bool assume_voxel_centres=0);
   LaserVoxel(double vox_l);
   ~LaserVoxel();
  
   void statistics();
   
   uint get_vox_num_X() {return vox_num_X;}
   uint get_vox_num_Y() {return vox_num_Y;}
   uint get_vox_num_Z() {return vox_num_Z;}
   vector<LaserPoints> LaserPoints_per_voxel(); //contains the physical (not Pointer) points in a voxel. Regardless of the voxel's spatial position. needed in 3D scene interpretation, subdivision of points
   ///Filter: min. number of points in a voxel
   void filter_size(uint min=2,bool verbose=1);
   bool Vox_at_ijk_is_filled(uint i, uint j, uint k) {if (Vox[i][j][k]!=NULL) return 1;else return 0;}
   ///Filter according to neighb.hood: in the 26-neighb.hood of a voxel a min. number of points is required, otherwise it is deleted
   void filter_neighbour(uint min=1,bool verbose=1);
   int neighbors_at_ijk(uint i, uint j, uint k,uint n_size, bool assume_voxel_centres, LaserPoints &neig, double &X, double &Y, double &Z, bool only_2D);
   int number_of_points_in_voxel_at(double X, double Y, double Z, LaserPoints *PointsInVoxel=NULL);
   LaserPoints export_all();
   void export_all_ply(char * filename,LaserPoints l);
   LaserPoints export_vox_centres(bool also_voids=false,bool assume_voxel_centres=0); ///only exports the centres of voxels (default: where laserpoints are)
   void export_vox_centres_text(char * filename);
   bool export_vox_centre_at(uint i, uint j, uint k, LaserPoint &lp,bool assume_voxel_centres);
   //attention: labels=numClasses+1!!!
   void fill_data_terms_from_supervised(int **data,uint & num_labels,double & mean, double &stddev,::map < uint, vector < double > > &responses_per_segment_TRAINING, ::map < uint, vector < double > > & responses_per_segment_VALIDATION, ::map < uint, vector < double > > &responses_per_segment_OTHERS);
   
   //here the background is assumed to be amongst the classes!!!
   void fill_data_terms_for_unsupervised_segment(int **data,uint & num_labels,::map < uint, vector < int > > &energy_per_segment,int scale_to);
   
   void fill_data_terms_for_unsupervised_voxel(int **data,uint & num_labels,::map < uint, vector < int > > &energy_per_voxel,int scale_to);
   
   //reading the optimization result: per voxel one label. We need the label info to assign the correct color
   void update_labels(int **result,uint & num_labels,::map<uint, int> & label_R, ::map<uint, int>  & label_G, ::map<uint, int> & label_B);
   void append_to_outputfiles(LaserPoints & out_training, LaserPoints & out_validation, LaserPoints &out_others);
   void add_a_LaserPoint_to_vox(uint i, uint j, uint k, LaserPoint &lp);
   void empty_voxel_at_ikj(uint i, uint j, uint k){if (Vox[i][j][k]!=NULL){ delete Vox[i][j][k]; Vox[i][j][k]=NULL;}}
   void write_world_file(char *tfw_filename, double offset_for_tfw_X, double offset_for_tfw_Y);
};

#endif
